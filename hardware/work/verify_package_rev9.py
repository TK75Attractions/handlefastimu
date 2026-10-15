from pathlib import Path
import json, zipfile
import xml.etree.ElementTree as ET

base=Path('hardware');root=base/'Rev9'
ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
old=zipfile.ZipFile(base/'Rev8/組立・はんだ付け工程表.xlsx')
new=zipfile.ZipFile(root/'組立・はんだ付け工程表.xlsx')
for filename in ['xl/worksheets/sheet1.xml','xl/worksheets/sheet2.xml']:
 a=ET.fromstring(old.read(filename));b=ET.fromstring(new.read(filename))
 for key in ['dataValidations','conditionalFormatting','mergeCells']:
  aa=[ET.tostring(x) for x in a.findall(f's:{key}',ns)]
  bb=[ET.tostring(x) for x in b.findall(f's:{key}',ns)]
  assert aa==bb,(filename,key)
 assert len(a.findall('.//s:tablePart',ns))==len(b.findall('.//s:tablePart',ns))
 relpath=filename.replace('worksheets/','worksheets/_rels/')+'.rels'
 rels=lambda z:sorted((p.get('Type'),p.get('Target')) for p in ET.fromstring(z.read(relpath)))
 assert rels(old)==rels(new),(filename,'relationships')
 for tag in ['pane']:
  aa=a.find(f'.//s:{tag}',ns);bb=b.find(f'.//s:{tag}',ns)
  for key in ['xSplit','ySplit','state','activePane']:
   assert aa.get(key)==bb.get(key),(filename,tag,key)
 f=lambda t:{c.get('r'):c.find('s:f',ns).text for c in t.findall('.//s:c',ns) if c.find('s:f',ns) is not None}
 assert f(a)==f(b),'formula changed'
for name in [n for n in old.namelist() if n.startswith('xl/tables/')]:
 assert old.read(name)==new.read(name),(name,'table features')
old.close();new.close()
m=json.loads((root/'部品座標.json').read_text())
expected={
 'R1':('1kΩ','SIG1','ADC0'),'R2':('1kΩ','SIG2','ADC1'),
 'R7':('100kΩ','GPIO27','ADC0'),'R8':('100kΩ','GPIO27','ADC1'),
 'C1':('100nF','ADC0','GND'),'C2':('100nF','ADC1','GND'),
 'C3':('100nF','3V3','GND'),'C6':('10µF','3V3','GND')}
assert {p['ref']:(p['value'],p['net_p'],p['net_q']) for p in m['parts']}==expected
assert [p['pin'] for p in m['ads']]==['A3','A2','A1','A0','ALRT','ADDR','SDA','SCL','GND','VDD']
e=json.loads((root/'配線座標.json').read_text())
assert len(e)==43 and sum(x['wire']=='0.5mmスズメッキ線' for x in e)==10
md=(root/'実装手順と全配線表.md').read_text()
for i in range(1,44):assert f'|{i}|' in md
schedule=(root/'組立・はんだ付け工程表.md').read_text()
for stale in ['列20=VDD','VDDを列20側','全37本','残り31本']:
 assert stale not in schedule,stale
for filename in ['部品配置','裏面① 電源・GND','裏面② I²C','裏面③ ペダル・ADC・GPIO27','裏面④ 全配線（①＋②＋③）','統合電気回路図']:
 for suffix in ['.svg','.png']:assert (root/(filename+suffix)).stat().st_size>1000
for p in root.glob('*.inspect.ndjson'):p.unlink()
with zipfile.ZipFile(base/'Rev9_配線図一式.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
 for p in sorted(root.rglob('*')):
  if p.is_file() and not p.name.startswith(('~$','.')):z.write(p,p.relative_to(base))
print(json.dumps({'workbook_preserved_native_features':True,'passive_netlist':True,'pin_orientation':True,'package_files':len(zipfile.ZipFile(base/'Rev9_配線図一式.zip').namelist())},ensure_ascii=False))
