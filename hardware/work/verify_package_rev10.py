from pathlib import Path
import json,zipfile,xml.etree.ElementTree as ET

base=Path('hardware');root=base/'Rev10';oldroot=base/'Rev8'
assert json.loads((root/'配線座標.json').read_text())==json.loads((oldroot/'配線座標.json').read_text())
for p in oldroot.glob('*.svg'):
 if p.name=='統合電気回路図.svg':continue
 assert (root/p.name).read_text()==p.read_text().replace('Rev.8','Rev.10'),p.name
m=json.loads((root/'IMU_Cタイプ/配線座標.json').read_text())
assert list(m['socket'].values())==['INT','AD0','XCL','XDA','SDA','SCL','GND','VCC']
for imu in [1,2]:
 edges=m[f'IMU{imu}'];assert len(edges)==9
 assert sum(e['wire']=='0.5mmスズメッキ線' for e in edges)==3
 assert edges[-1]['from']==[9,3] and edges[-1]['net']==('GND' if imu==1 else '3V3')
 for kind in ['部品配置','裏面配線']:
  for ext in ['.svg','.png']:assert (root/f'IMU_Cタイプ/IMU{imu}_{kind}{ext}').stat().st_size>1000
for filename in ['部品配置','裏面① 電源・GND','裏面② I²C','裏面③ ペダル・ADC・GPIO27','裏面④ 全配線（①＋②＋③）','統合電気回路図']:
 for ext in ['.svg','.png']:assert (root/(filename+ext)).stat().st_size>1000
ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
old=zipfile.ZipFile(oldroot/'組立・はんだ付け工程表.xlsx');new=zipfile.ZipFile(root/'組立・はんだ付け工程表.xlsx')
for filename in ['xl/worksheets/sheet1.xml','xl/worksheets/sheet2.xml']:
 a=ET.fromstring(old.read(filename));b=ET.fromstring(new.read(filename))
 for key in ['dataValidations','conditionalFormatting','mergeCells']:
  assert [ET.tostring(x) for x in a.findall(f's:{key}',ns)]==[ET.tostring(x) for x in b.findall(f's:{key}',ns)],(filename,key)
 assert len(a.findall('.//s:tablePart',ns))==len(b.findall('.//s:tablePart',ns))
 relpath=filename.replace('worksheets/','worksheets/_rels/')+'.rels'
 rels=lambda z:sorted((p.get('Type'),p.get('Target')) for p in ET.fromstring(z.read(relpath)))
 assert rels(old)==rels(new)
 pa=a.find('.//s:pane',ns);pb=b.find('.//s:pane',ns)
 for key in ['xSplit','ySplit','state','activePane']:assert pa.get(key)==pb.get(key)
 formulas=lambda t:{c.get('r'):c.find('s:f',ns).text for c in t.findall('.//s:c',ns) if c.find('s:f',ns) is not None}
 assert formulas(a)==formulas(b),'formula changed'
for name in [n for n in old.namelist() if n.startswith('xl/tables/')]:assert old.read(name)==new.read(name)
old.close();new.close()
schedule=(root/'組立・はんだ付け工程表.md').read_text()
assert '1×6P' not in schedule and 'VIN' not in schedule and '3Vo' not in schedule
assert '列20=VDD' in schedule and '行20/22レール' in schedule
for p in root.glob('*.inspect.ndjson'):p.unlink()
with zipfile.ZipFile(base/'Rev10_配線図一式.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
 for p in sorted(root.rglob('*')):
  if p.is_file() and not p.name.startswith(('~$','.')):z.write(p,p.relative_to(base))
print(json.dumps({'base':'Rev.8','main_svg_and_routes_preserved':True,'photo_pin_order':True,'IMU1_AD0':'GND','IMU2_AD0':'3V3','spreadsheet_native_features_preserved':True,'zip_files':len(zipfile.ZipFile(base/'Rev10_配線図一式.zip').namelist())},ensure_ascii=False))
