from pathlib import Path
from html import escape
from collections import defaultdict
import json, shutil, math

P=Path('hardware/Rev9'); P.mkdir(parents=True,exist_ok=True)
COL={'3V3':'#d43c51','GND':'#293b4d','SDA':'#188849','SCL':'#2673c7',
     'SIG1':'#b68215','SIG2':'#b26120','ADC0':'#9a54b5','ADC1':'#ca5089','GPIO27':'#e79610'}
# Circuit side remains UP. Rotate in the plane, placing the pin row at TOP.
ads_order=['A3','A2','A1','A0','ALRT','ADDR','SDA','SCL','GND','VDD']
ads={(20+i,2):n for i,n in enumerate(ads_order)}
tb=[('MPU1電源',14,24,['3V3','GND']),('MPU1信号',17,24,['SDA','SCL']),
    ('MPU2電源',20,24,['3V3','GND']),('MPU2信号',23,24,['SDA','SCL']),
    ('ペダル1',26,24,['3V3','SIG1','GND']),('ペダル2',30,24,['3V3','SIG2','GND'])]
parts=[('R1','1kΩ',(22,12),(26,12),'SIG1','ADC0'),
       ('C1','100nF',(26,14),(28,14),'ADC0','GND'),
       ('R7','100kΩ',(22,16),(26,16),'GPIO27','ADC0'),
       ('R2','1kΩ',(28,12),(32,12),'SIG2','ADC1'),
       ('C2','100nF',(32,14),(30,14),'ADC1','GND'),
       ('R8','100kΩ',(28,16),(32,16),'GPIO27','ADC1'),
       ('C3','100nF',(33,4),(33,6),'3V3','GND'),
       ('C6','10µF',(1,22),(1,24),'3V3','GND')]
E=[]
def add(net,p,q,stage,bends=(),taps=(),bare=False):
 E.append({'net':net,'from':p,'to':q,'stage':stage,'via':list(bends),'taps':list(taps),
           'wire':'0.5mmスズメッキ線' if bare else 'AWG24被覆線'})
# Large board zones, with each distribution joint assigned its own hole.
add('3V3',(1,22),(13,22),'rail_power',taps=[(3,22),(5,22),(7,22),(9,22),(11,22)],bare=True)
add('GND',(1,24),(13,24),'rail_power',taps=[(3,24),(5,24),(7,24),(9,24),(11,24)],bare=True)
add('SDA',(16,8),(16,20),'rail_i2c',taps=[(16,12),(16,16)],bare=True)
add('SCL',(19,5),(19,20),'rail_i2c',taps=[(19,10),(19,16)],bare=True)
add('ADC0',(26,12),(26,17),'analog',taps=[(26,14),(26,16)],bare=True)
add('ADC1',(32,12),(32,17),'analog',taps=[(32,14),(32,16)],bare=True)
add('GPIO27',(22,21),(29,21),'gpio',taps=[(24,21)],bare=True)
add('GND',(20,11),(28,11),'power',taps=[(24,11),(26,11)],bare=True)
add('3V3',(33,2),(33,4),'power',taps=[(33,3)],bare=True)
add('GND',(32,19),(32,24),'terminal_power',taps=[(32,21)],bare=True)
# Four power and four ground connections use widely separated backside lanes.
add('3V3',(14,24),(7,22),'terminal_power',[(14,21),(7,21)])
add('GND',(15,24),(7,24),'terminal_power',[(15,22),(14,21),(14,11),(6,11),(6,23)])
add('3V3',(20,24),(9,22),'terminal_power',[(20,14),(9,14)])
add('GND',(21,24),(9,24),'terminal_power',[(21,22),(18,21),(18,15),(8,15),(8,23)])
add('3V3',(26,24),(11,22),'terminal_power',[(26,22),(27,21),(27,19),(18,9),(11,9)])
add('GND',(28,24),(11,24),'terminal_power',[(28,22),(17,21),(17,17),(10,17),(10,23)])
add('3V3',(30,24),(13,22),'terminal_power',[(30,22),(31,21),(31,19),(12,19),(13,20)])
add('GND',(32,21),(13,24),'terminal_power',[(12,23)])
add('SDA',(17,24),(16,20),'terminal_i2c',[(16,23)])
add('SCL',(18,24),(19,20),'terminal_i2c',[(19,23)])
add('SDA',(23,24),(16,16),'terminal_i2c',[(23,22)])
add('SCL',(24,24),(19,16),'terminal_i2c',[(24,22)])
add('3V3',(3,3),(3,22),'power',[(2,4),(2,21)])
add('GND',(13,3),(3,24),'power',[(12,4),(4,4),(4,23)])
add('3V3',(29,2),(33,2),'power',[(31,1)])
add('GND',(28,2),(26,11),'power',[(28,8),(26,10)])
add('3V3',(33,3),(5,22),'power',[(21,3),(21,13),(5,13)])
add('GND',(33,6),(28,11),'power',[(32,5),(32,10)])
add('GND',(20,11),(5,24),'power',[(15,11),(15,21)])
add('GND',(25,2),(28,2),'power',[(25,1),(28,1)])
add('SDA',(13,8),(16,8),'i2c')
add('SCL',(13,5),(19,5),'i2c')
add('SDA',(26,2),(16,12),'i2c',[(26,4),(17,4),(17,11)])
add('SCL',(27,2),(19,10),'i2c',[(27,5),(20,5),(20,9)])
add('SIG1',(27,24),(22,12),'analog',[(27,22),(25,21),(25,20),(23,20),(23,13)])
add('SIG2',(31,24),(28,12),'analog',[(31,22),(33,21),(33,8),(29,8),(29,11)])
add('ADC0',(23,2),(26,17),'analog',[(23,6),(25,6),(25,17)])
add('ADC1',(22,2),(32,17),'analog',[(22,7),(31,7),(31,17)])
add('GND',(28,14),(24,11),'analog',[(27,13),(27,12),(24,10)])
add('GND',(30,14),(32,19),'analog',[(30,17)])
add('GPIO27',(3,13),(22,21),'gpio',[(1,13),(1,20),(12,20),(12,23),(22,23)])
add('GPIO27',(22,16),(24,21),'gpio',[(23,17),(24,19)])
add('GPIO27',(28,16),(29,21),'gpio',[(29,17)])
assert len(E)==43

pins={}; req=defaultdict(list)
def claim(p,n):
 assert pins.setdefault(p,n)==n,(p,n,pins[p])
 req[n].append(p)
for p,n in [((3,3),'3V3'),((13,3),'GND'),((13,8),'SDA'),((13,5),'SCL'),((3,13),'GPIO27')]:claim(p,n)
ads_nets={'VDD':'3V3','GND':'GND','SDA':'SDA','SCL':'SCL','ADDR':'GND','A0':'ADC0','A1':'ADC1'}
for p,n in ads.items():
 if n in ads_nets:claim(p,ads_nets[n])
for _,c,r,names in tb:
 for j,n in enumerate(names):claim((c+j,r),n)
part_holes=[]
for ref,val,p,q,np,nq in parts:
 claim(p,np);claim(q,nq);part_holes.extend([p,q])
assert len(set(part_holes))==len(part_holes),'component leads share holes'
parent={}
def root(p):
 parent.setdefault(p,p)
 if parent[p]!=p:parent[p]=root(parent[p])
 return parent[p]
for e in E:
 for p in [e['from'],e['to'],*e['taps']]:claim(p,e['net'])
 a=root(e['from'])
 for p in [e['to'],*e['taps']]:parent[root(p)]=root(a)
for n,ps in req.items():assert len({root(p) for p in ps})==1,n
assert len({root(ps[0]) for ps in req.values()})==9
def mx(c):return 6.86+(c-1)*2.54
def my(r):return 6.79+(r-1)*2.54
def overlaps(a,b):return a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]
boxes=[('ESP32',(10.67,1.9,38.61,56.3)),('ADS1115',(53.0,6.79,83.0,27.49)),
       ('C6',(mx(1)-2.5,my(23)-2.5,mx(1)+2.5,my(23)+2.5))]
for ref,val,p,q,np,nq in parts:
 if ref=='C6':continue
 cx=(mx(p[0])+mx(q[0]))/2;cy=(my(p[1])+my(q[1]))/2
 hx,hy=(3.15,1.25) if ref.startswith('R') else ((2,1) if p[1]==q[1] else (1,2))
 boxes.append((ref,(cx-hx,cy-hy,cx+hx,cy+hy)))
for title,c,r,names in tb:
 cx=(mx(c)+mx(c+len(names)-1))/2;cy=my(r);hx=(5.54 if len(names)==2 else 8.08)/2
 boxes.append((title,(cx-hx,cy-3.1,cx+hx,cy+3.1)))
for i,(name,a) in enumerate(boxes):
 assert 0<=a[0]<a[2]<=95 and 0<=a[1]<a[3]<=72,(name,a)
 for title,b in boxes[i+1:]:assert not overlaps(a,b),(name,title)
 for x,y in [(3,3),(92,3),(3,69),(92,69)]:
  nx=min(max(x,a[0]),a[2]);ny=min(max(y,a[1]),a[3])
  assert (nx-x)**2+(ny-y)**2>3.2**2,(name,'mounting hole')
usb_keepout=(18,56.3,31,72)
for name,a in boxes:
 if name!='ESP32':assert not overlaps(a,usb_keepout),(name,'USB plug')
# --- Drawing ---------------------------------------------------------------
def svg(name, stages, front=False):
    W,H=1660,1180
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         '<rect width="100%" height="100%" fill="white"/>',
         '<style>text{font-family:Arial,"Hiragino Sans","Noto Sans CJK JP",sans-serif;fill:#20364c;font-size:16px}.h{font-size:29px;font-weight:bold}.sub{font-size:14px;fill:#51697b}.tag{font-size:13px;font-weight:bold}.tiny{font-size:12px;fill:#51697b}</style>']
    def t(x,y,s,cl=''): out.append(f'<text x="{x}" y="{y}" class="{cl}">{escape(str(s))}</text>')
    def xy(p):
        c,r=p; x=mx(c); y=my(r)
        return (80+(x if front else 95-x)*8,170+y*8)
    def path(pts,col,w=3,dash=''):
        ss=' '.join(f'{x:.1f},{y:.1f}' for x,y in [xy(p) for p in pts])
        out.append(f'<polyline points="{ss}" fill="none" stroke="{col}" stroke-width="{w}" stroke-linecap="round" stroke-linejoin="round"'+(f' stroke-dasharray="{dash}"' if dash else '')+'/>' )
    def circle(p,col,r=4):
        x,y=xy(p);out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{col}" stroke="#4d5e6d" stroke-width="0.8"/>')
    def rect_mm(x,y,w,h,fill,stroke='#61788d'):
        out.append(f'<rect x="{80+x*8}" y="{170+y*8}" width="{w*8}" height="{h*8}" fill="{fill}" stroke="{stroke}" rx="3"/>')

    t(40,47,f'Rev.9：{name}','h')
    t(40,78,'Bタイプ95×72mm・表面座標33列×24行。ADSは回路面を上、1×10ピン。裏面図は左右反転。','sub')
    t(40,103,'端子台6個は下端に横一列、挿入口は基板外向き。基板下端から15mm以上をケーブル引出し空間として確保する。','sub')
    out.append('<rect x="80" y="170" width="760" height="576" rx="4" fill="#f5ecd9" stroke="#aa9b79"/>')
    # Cable clearance zone is outside the physical board.
    out.append('<rect x="80" y="756" width="760" height="120" rx="5" fill="#e9f6ef" stroke="#5d9b73" stroke-dasharray="8 6"/>')
    t(300,855,'端子台前方：15mm以上のケーブル引出し空間（部品・壁・結束部品を置かない）','sub')
    for c in range(1,34):
        x,y=xy((c,1));t(x-5,161,c,'sub')
    for r in range(1,25):
        x,y=xy((1,r));t(93,y+5,r,'sub')
        for c in range(1,34): circle((c,r),'#ceb995',2.15)
    for X,Y in [(3,3),(92,3),(3,69),(92,69)]:
        x=80+X*8;y=170+Y*8;out.append(f'<circle cx="{x}" cy="{y}" r="12.8" fill="white" stroke="#9a8965"/>')

    if front:
        # ESP32 body and two socket rows.
        rect_mm(10.67,1.9,27.94,54.4,'#dceaf4');rect_mm(10.67,1.9,27.94,8.1,'#fff0ae')
        x,y=xy((3,3));t(x+34,y+43,'アンテナ','sub');t(x+20,y+160,'ESP32','h');t(x+50,y+354,'USB ↓','h')
        for c in (3,13):
            x,y=xy((c,3));out.append(f'<rect x="{x-6}" y="{y-6}" width="12" height="378" fill="#294257"/>')
            for r in range(3,22): circle((c,r),'#edf5fb',3)

        rect_mm(18,56.3,13,15.7,'#fff8e8','#caaa6b')
        t(225,729,'USBプラグ空間','tiny')
        # ADS board, circuit side up. Pin row is its upper edge after rotation.
        rect_mm(53.0,6.79,30.0,20.7,'#cfe8de')
        x,y=xy((20,2));t(x+30,y+70,'ADS1115','h');t(x+30,y+98,'回路面上・180°回転','sub')
        x1,y1=xy((20,2));x2,_=xy((29,2))
        out.append(f'<rect x="{x1-7}" y="{y1-7}" width="{x2-x1+14}" height="14" fill="#2b4d41"/>')
        for p,n in ads.items():
            circle(p,'white',3);x,y=xy(p)
            out.append(f'<text transform="translate({x-3},{y-12}) rotate(-90)" class="tiny">{escape(n)}</text>')

        # Terminal blocks and outward opening arrows.
        for title,c,r,ns in tb:
            x1,y=xy((c,r));x2,_=xy((c+len(ns)-1,r));left=min(x1,x2)-12;wid=abs(x2-x1)+24
            out.append(f'<rect x="{left}" y="{y-25}" width="{wid}" height="50" rx="3" fill="#87c699" stroke="#687f71"/>')
            t(left,y-34,title,'tiny')
            for j,n in enumerate(ns): circle((c+j,r),'white',4)
            mid=left+wid/2
            out.append(f'<path d="M {mid} {y+17} L {mid} {y+64} M {mid-7} {y+54} L {mid} {y+64} L {mid+7} {y+54}" stroke="#3d7c58" stroke-width="3" fill="none"/>')
    else:
        # Ghost component outlines on solder side.
        for c in (3,13):
            x1,y1=xy((c,3));x2,y2=xy((c,21));out.append(f'<rect x="{x1-7}" y="{y1-7}" width="14" height="{y2-y1+14}" fill="none" stroke="#9aaab5" stroke-dasharray="4 4"/>')
        x1,y=xy((20,2));x2,_=xy((29,2));out.append(f'<rect x="{min(x1,x2)-7}" y="{y-7}" width="{abs(x2-x1)+14}" height="14" fill="none" stroke="#9aaab5" stroke-dasharray="4 4"/>')
        for title,c,r,ns in tb:
            x1,y=xy((c,r));x2,_=xy((c+len(ns)-1,r));out.append(f'<rect x="{min(x1,x2)-12}" y="{y-25}" width="{abs(x2-x1)+24}" height="50" rx="3" fill="none" stroke="#9aaab5" stroke-width="1.5" stroke-dasharray="5 4"/>')
        for e in E:
            if e['stage'] not in stages: continue
            ps=[e['from'],*e['via'],e['to']];path(ps,'white',7);path(ps,COL[e['net']],5.0 if e['wire']=='0.5mmスズメッキ線' else 2.8, '' if e['wire']=='0.5mmスズメッキ線' else '8 3')
        for e in E:
            if e['stage'] not in stages: continue
            circle(e['from'],COL[e['net']],4);circle(e['to'],COL[e['net']],4)
            for p in e['taps']: circle(p,COL[e['net']],4)
        for p,n in ads.items():
            x,y=xy(p);out.append(f'<text transform="translate({x+4},{y-12}) rotate(-90)" class="tiny">{escape(n)}</text>')

    for ref,val,p,q,np,nq in parts:
        if not front and ref.startswith('C'):
            x1,y1=xy(p);x2,y2=xy(q);cx=(x1+x2)/2;cy=(y1+y2)/2
            vertical=abs(y2-y1)>abs(x2-x1)
            if vertical:
                lead1=(cx,cy-4);lead2=(cx,cy+4)
                plates=[(cx-7,cy-4,cx+7,cy-4),(cx-7,cy+4,cx+7,cy+4)]
            else:
                lead1=(cx+4,cy);lead2=(cx-4,cy)
                plates=[(cx+4,cy-7,cx+4,cy+7),(cx-4,cy-7,cx-4,cy+7)]
            for a,b in [((x1,y1),lead1),((x2,y2),lead2)]:
                out.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#86627d" stroke-width="2" stroke-dasharray="4 3"/>')
            for ax,ay,bx,by in plates:
                out.append(f'<line x1="{ax}" y1="{ay}" x2="{bx}" y2="{by}" stroke="#86627d" stroke-width="2"/>')
        else:
            path([p,q],'#86627d',5 if front else 2.2,'5 4' if not front else '')
        if front and ref == 'C6':
            x1,y1=xy(p);x2,y2=xy(q)
            out.append(f'<circle cx="{(x1+x2)/2}" cy="{(y1+y2)/2}" r="20" fill="#dce8ef" stroke="#52697b" stroke-width="2"/>')
        circle(p,'white',3);circle(q,'white',3)
        if ref != 'C6':
            x,y=xy(p);t(x-12,y-10,ref,'sub')
    x,y=xy((1,22));t(x+8,y-7,'C6＋','tiny');x,y=xy((1,24));t(x+8,y-6,'−','tiny')

    t(890,183,'作業の要点','h')
    note_map={
      '部品配置':['ADSを面内で180°回転。ピン列は行2、列20〜29。',
        '左→右：A3,A2,A1,A0,ALRT,ADDR,SDA,SCL,GND,VDD。',
        '電源・C6はESP下端の列1〜13へ。USB前方は空ける。',
        'R1/C1/R7は列22〜28、R2/C2/R8は列28〜32。',
        '受動部品の脚はすべて別穴。端子台は従来と同じ行24。'],
      '裏面① 電源・GND':['左下3V3：行22、列1〜13。GND：行24、列1〜13。',
        '両レールのタップは2列間隔（5.08mm）。',
        'C6＋(1,22)／−(1,24)は左下へ直接接続。',
        '端子台からの長い電源線も、図の経路へ分散する。',
        '右側の裸線はADS電源とペダル2 GNDの局所分配。'],
      '裏面② I²C':['SDA：列16、行8〜20。SCL：列19、行5〜20。',
        '裸線2本の間隔は7.62mm。接続点も縦に分散。',
        'ESP GPIO21/22、ADS、MPU1/2を同名点へ。',
        '交差するジャンパーには必ず被覆を残す。'],
      '裏面③ ペダル・ADC・GPIO27':['R1/R2は行12、C1/C2は行14、R7/R8は行16。',
        'ADC0：列26、ADC1：列32の縦裸線で分配。',
        'GPIO27の裸線は行21、列22〜29。',
        '同じ穴に部品脚2本を入れない。各部品を別穴へ。',
        'C1 GNDはADS側、C2 GNDはペダル2側へ接続。'],
      '裏面④ 全配線（①＋②＋③）':['基板裏面の左・中央・右を使った全配線。',
        '電源線はESPの裏側にも経路を分散する。',
        '太い実線＝裸線、細い破線＝被覆線。紫＝部品。',
        '白い切れ目の交差は非接続。曲がり角ははんだ点でない。',
        '裸線と被覆線の交差では被覆線を浮かせて通す。']}
    for i,s in enumerate(note_map[name]):t(890,223+31*i,s,'sub')
    t(890,445,'主な配置と穴番号','h')
    key=['ESP 3V3=(3,3)、GND=(13,3)、GPIO27=(3,13)',
         'GPIO21=(13,8)、GPIO22=(13,5)',
         'ADS VDD=(29,2)、GND=(28,2)、ADDR=(25,2)',
         'ADS SDA=(26,2)、SCL=(27,2)、A0=(23,2)、A1=(22,2)',
         '左下電源：3V3行22／GND行24（列1〜13）',
         'C6＝＋(1,22)、−(1,24)、外径5mm以下想定',
         'ADC0＝列26／ADC1＝列32（行12〜17）',
         '全端子台＝行24、挿入口は基板外向き']
    for i,s in enumerate(key):t(890,485+27*i,s,'sub')
    t(890,750,'実装上の注意','h')
    t(890,790,'ESP/ADSの裏面も配線に使用。モジュールは外して作業。','sub')
    t(890,820,'裸線を横切る被覆線は約1mm浮かせ、被覆を傷めない。','sub')
    t(890,850,'ソケット脚に沿わせず、交差部では被覆線を浮かせる。','sub')
    t(890,880,'ADSは裏返さない。回路面上のまま、表面ピン順を確認。','sub')
    t(80,1060,'太い実線＝0.5mmスズメッキ線10本。細い破線＝AWG24被覆線33本。紫の記号＝表面側の抵抗・コンデンサー位置。','sub')
    t(80,1090,'端子台へ入れるより線ははんだメッキしない。ケーブルは基板下端からまっすぐ外へ出し、15mm以上先で固定する。','sub')
    t(80,1120,'実物ADSのピッチが2.54mmであること、基板外形・ケース・ねじ回し空間を仮置きで確認してから固定する。','sub')
    t(80,1150,'電気ネット・穴位置・公称端子台外形は検査済み。実機の組立・導通・通電は未検証。','sub')
    out.append('</svg>')
    (P/(name+'.svg')).write_text('\n'.join(out))

svg('部品配置',[],True)
svg('裏面① 電源・GND',['rail_power','terminal_power','power'])
svg('裏面② I²C',['rail_i2c','terminal_i2c','i2c'])
svg('裏面③ ペダル・ADC・GPIO27',['analog','gpio'])
svg('裏面④ 全配線（①＋②＋③）',['rail_power','terminal_power','power','rail_i2c','terminal_i2c','i2c','analog','gpio'])
