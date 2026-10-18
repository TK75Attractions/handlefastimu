from pathlib import Path
from html import escape
from fractions import Fraction as F
import json, shutil

OUT=Path('hardware/Rev10');OUT.mkdir(exist_ok=True,parents=True)
IMU=OUT/'IMU_Cタイプ';IMU.mkdir(exist_ok=True)
COL={'3V3':'#d43c51','GND':'#293b4d','SDA':'#188849','SCL':'#2673c7'}
socket={(8+i,3):n for i,n in enumerate(['INT','AD0','XCL','XDA','SDA','SCL','GND','VCC'])}
terminal={(10,14):'3V3',(11,14):'GND',(13,14):'SCL',(14,14):'SDA'}
capacitor={(17,10):'3V3',(19,10):'GND'}
def edge(n,p,q,via=(),taps=(),bare=False):
 return {'net':n,'from':p,'to':q,'via':list(via),'taps':list(taps),'wire':'0.5mmスズメッキ線' if bare else 'AWG24被覆線'}
def wires(imu):
 return [edge('3V3',(15,3),(17,2),[(15,2)]),
 edge('3V3',(17,2),(17,13),taps=[(17,10)]+([(17,6)] if imu==2 else []),bare=True),
 edge('3V3',(17,13),(10,14),[(17,13.5),(10,13.5)]),
 edge('GND',(14,3),(19,4),[(14,1),(19,1)]),
 edge('GND',(19,4),(19,12),taps=[(19,10)]+([(19,5)] if imu==1 else []),bare=True),
 edge('GND',(19,12),(11,14),[(19,12.5),(11,12.5)]),
 edge('SDA',(12,3),(14,14),[(12,12),(14,12)]),
 edge('SCL',(13,3),(13,14),bare=True),
 edge('GND' if imu==1 else '3V3',(9,3),(19,5) if imu==1 else (17,6),[(9,5)] if imu==1 else [(9,6)])]

# Rev.8 is the authoritative main-board baseline. Do not use Rev.9 assets.
for src in Path('hardware/Rev8').iterdir():
 if not src.is_file() or src.suffix not in ['.svg','.json','.md']:continue
 if src.name in ['組立・はんだ付け工程表.md','旧版からの付け替え手順.md','図面レビュー結果.json']:continue
 text=src.read_text().replace('Rev.8','Rev.10')
 (OUT/src.name).write_text(text)
shutil.copyfile('/tmp/codex-remote-attachments/01a09915-7c78-7b13-a806-266f855656c9/04D610D6-B7DB-4149-9CAF-32E7473F8839/1-Photo-1.jpg',OUT/'参考写真_MPU6050.jpg')

def mx(c):return 5.52+(c-1)*2.54
def my(r):return 5.72+(r-1)*2.54
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def sub(a,b):return a[0]-b[0],a[1]-b[1]
def on(p,a,b):return cross(sub(p,a),sub(b,a))==0 and all(min(a[k],b[k])<=p[k]<=max(a[k],b[k]) for k in (0,1))
reports=[]
for imu in [1,2]:
 E=wires(imu);pins={};required={};parent={}
 def root(p):
  parent.setdefault(p,p)
  if parent[p]!=p:parent[p]=root(parent[p])
  return parent[p]
 def claim(p,n):
  assert pins.setdefault(tuple(p),n)==n,(imu,p,n,pins[tuple(p)])
  required.setdefault(n,[]).append(tuple(p))
 for p,n in terminal.items():claim(p,n)
 for p,n in capacitor.items():claim(p,n)
 smap={'VCC':'3V3','GND':'GND','SCL':'SCL','SDA':'SDA','AD0':'GND' if imu==1 else '3V3'}
 for p,n in socket.items():
  if n in smap:claim(p,smap[n])
 for e in E:
  pts=[e['from'],e['to'],*e['taps']]
  for p in pts:
   assert all(int(x)==x for x in p) and 1<=p[0]<=25 and 1<=p[1]<=15
   claim(p,e['net']);root(p)
  for p in pts[1:]:parent[root(p)]=root(pts[0])
 for n,ps in required.items():assert len({root(p) for p in ps})==1,(imu,n)
 assert len({root(ps[0]) for ps in required.values()})==4
 for p,n in socket.items():
  if n in ['INT','XCL','XDA']:assert p not in pins,(imu,n)
 segs=[]
 for i,e in enumerate(E,1):
  seq=[e['from'],*e['via'],e['to']]
  for a,b in zip(seq,seq[1:]):segs.append((i,e['net'],a,b,e['wire']))
 for k,(i,n,a,b,w) in enumerate(segs):
  for j,m,c,d,v in segs[k+1:]:
   if i==j:continue
   if cross(sub(b,a),sub(d,c))==0 and cross(sub(c,a),sub(b,a))==0:
    axis=0 if a[0]!=b[0] else 1
    assert max(min(a[axis],b[axis]),min(c[axis],d[axis]))>=min(max(a[axis],b[axis]),max(c[axis],d[axis])),(imu,'overlap',i,j)
  if w=='0.5mmスズメッキ線':
   for p,m in pins.items():assert not on(p,a,b) or m==n,(imu,'bare foreign pad',i,p,m)
   for p,label in socket.items():
    if label not in smap:assert not on(p,a,b),(imu,'unused pin',i,label)
 boxes=[('MPU',(21.2,8.8,43.2,26.8)),('C4/5',(mx(18)-2,my(10)-1,mx(18)+2,my(10)+1))]
 for c in [10,13]:
  cx=(mx(c)+mx(c+1))/2;cy=my(14)
  boxes.append(('terminal',(cx-2.77,cy-3.1,cx+2.77,cy+3.1)))
 for k,(name,a) in enumerate(boxes):
  assert 0<=a[0]<a[2]<=72 and 0<=a[1]<a[3]<=47
  for other,b in boxes[k+1:]:assert not(a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]),(name,other)
 reports.append({'imu':imu,'wires':9,'bare':3,'insulated':6,'nets':4,'collinear_overlaps':0,'unused_pins':['INT','XCL','XDA'],'AD0':'GND' if imu==1 else '3V3','mechanical_nominal_check':'pass','hardware_verified':False})

def draw(imu,front):
 W,H=1600,1120
 A=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">','<rect width="100%" height="100%" fill="white"/>',
 '<style>text{font-family:Arial,"Hiragino Sans",sans-serif;fill:#20364c;font-size:16px}.h{font-size:29px;font-weight:bold}.sub{font-size:14px;fill:#52697b}.pin{font-size:13px;font-weight:bold}.small{font-size:12px;fill:#52697b}</style>']
 def t(x,y,s,cls=''):A.append(f'<text x="{x}" y="{y}" class="{cls}">{escape(str(s))}</text>')
 def xy(p):
  x=mx(p[0]);return 70+(x if front else 72-x)*12,160+my(p[1])*12
 def path(ps,col,width=3,dash=''):
  coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,ps))
  A.append(f'<polyline points="{coords}" fill="none" stroke="{col}" stroke-width="{width}" stroke-linecap="round" stroke-linejoin="round"'+(f' stroke-dasharray="{dash}"' if dash else '')+'/>')
 def dot(p,fill,r=4):
  x,y=xy(p);A.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{fill}" stroke="#586b7c" stroke-width="1"/>')
 def boxmm(x,y,w,h,fill):A.append(f'<rect x="{70+x*12}" y="{160+y*12}" width="{w*12}" height="{h*12}" rx="4" fill="{fill}" stroke="#70869a"/>')
 t(40,48,f'Rev.10　IMU{imu}：Cタイプ基板 '+('部品配置' if front else '裏面全配線'),'h')
 t(40,80,'72×47mm・25列×15行・2.54mmピッチ。座標は部品面基準。裏面図は左右反転。','sub')
 t(40,108,f'写真の8ピン基板／回路面上、上側ピン列。AD0→'+('GND / 0x68' if imu==1 else '3V3 / 0x69'),'sub')
 boxmm(0,0,72,47,'#f4ead5')
 for r in range(1,16):
  _,y=xy((1,r));t(79,y+5,r,'small')
  for c in range(1,26):dot((c,r),'#ccb994',2)
 for c in range(1,26):x,_=xy((c,1));t(x-5,150,c,'small')
 for x,y in [(3,3),(69,3),(3,44),(69,44)]:
  A.append(f'<circle cx="{70+x*12}" cy="{160+y*12}" r="19.2" fill="white" stroke="#a59272"/>')
 if front:
  boxmm(21.2,8.8,22,18,'#bfd9e9')
  t(345,370,'MPU6050','h');t(335,400,'写真のモジュール','sub')
  for xx in [24,40]:
   A.append(f'<circle cx="{70+xx*12}" cy="{160+24*12}" r="15" fill="white" stroke="#879aac" stroke-dasharray="5 3"/>')
  t(335,485,'取付穴は現物写し','small')
  x,y=xy((8,3));A.append(f'<rect x="{x-8}" y="{y-8}" width="{7*2.54*12+16}" height="16" fill="#2d4657"/>')
 else:
  x,y=xy((15,3));A.append(f'<rect x="{x-8}" y="{y-8}" width="{7*2.54*12+16}" height="16" fill="none" stroke="#8295a6" stroke-dasharray="5 4"/>')
  for e in wires(imu):
   ps=[e['from'],*e['via'],e['to']];path(ps,'white',8);path(ps,COL[e['net']],5 if e['wire'].startswith('0.5') else 3,'' if e['wire'].startswith('0.5') else '9 4')
  for e in wires(imu):
   for p in [e['from'],e['to'],*e['taps']]:dot(p,COL[e['net']])
 for p,label in socket.items():
  dot(p,'white' if front or label in ['INT','XCL','XDA'] else COL[{'VCC':'3V3','AD0':'GND' if imu==1 else '3V3'}.get(label,label)])
  x,y=xy(p);t(x-13,y-16,label,'pin')
 for start,labels in [(10,['3V3','GND']),(13,['SCL','SDA'])]:
  x1,y=xy((start,14));x2,_=xy((start+1,14));left=min(x1,x2)-17
  A.append(f'<rect x="{left}" y="{y-37}" width="{abs(x2-x1)+34}" height="74" rx="4" fill="'+('#92c8a0' if front else 'none')+'" stroke="#668875"'+('' if front else ' stroke-dasharray="6 4"')+'/>')
  for j,n in enumerate(labels):
   p=(start+j,14);dot(p,'white' if front else COL[n]);x,y=xy(p);t(x-14,y-45,n,'pin')
  mid=(x1+x2)/2
  A.append(f'<path d="M {mid} {y+28} L {mid} 742 M {mid-8} 730 L {mid} 742 L {mid+8} 730" stroke="#43825b" stroke-width="3" fill="none"/>')
 A.append('<rect x="70" y="738" width="864" height="180" fill="#e9f6ef" stroke="#71a181" stroke-dasharray="8 6"/>')
 t(255,830,'端子台前方15mm以上はケーブル用に空ける','sub')
 # Capacitor is on component side. Symbol only on solder side.
 p,q=(17,10),(19,10);x1,y1=xy(p);x2,y2=xy(q)
 if front:
  path([p,q],'#83617e',5);dot(p,'white');dot(q,'white')
 else:
  cx=(x1+x2)/2
  for ax,bx in [(x1,cx+4),(x2,cx-4)]:A.append(f'<line x1="{ax}" y1="{y1}" x2="{bx}" y2="{y1}" stroke="#83617e" stroke-width="2" stroke-dasharray="4 3"/>')
  for x in [cx-4,cx+4]:A.append(f'<line x1="{x}" y1="{y1-9}" x2="{x}" y2="{y1+9}" stroke="#83617e" stroke-width="2"/>')
 t(min(x1,x2)-10,y1-19,'C4' if imu==1 else 'C5','pin')
 t(980,195,'ピン配置と接続','h')
 for i,(p,n) in enumerate(socket.items()):
  target={'INT':'未接続','XCL':'未接続','XDA':'未接続','AD0':'GND（0x68）' if imu==1 else '3V3（0x69）','VCC':'3V3','GND':'GND','SCL':'SCL','SDA':'SDA'}[n]
  t(980,237+i*30,f'{p}  {n} → {target}','sub')
 t(980,520,'実装・線材','h')
 notes=['1×8ピンソケット：行3、列8〜15。',f'C{4 if imu==1 else 5}=100nF：(17,10)―(19,10)。','端子台はRev.8と同じ行14・列10/11,13/14。','太い実線：0.5mmスズメッキ線3本。','細い破線：AWG24被覆線6本。','裸線は両端と指定タップだけはんだ付け。','白い切れ目の交差は非接続。被覆線を浮かせる。','12.5/13.5行は曲がり位置で、はんだ穴ではない。','INT/XCL/XDAへ裸線・はんだを触れさせない。','6ピン版用のADDRジャンパー手順は使わない。']
 for i,n in enumerate(notes):t(980,560+i*28,n,'sub')
 t(70,972,'写真のピン順を優先。モジュール外形22×18mm、ピッチ2.54mmは仮定しており、固定前に現物を仮置きする。','sub')
 t(70,1007,'取付穴の径・位置は実測し、合うナイロンねじとソケット高さに合うスペーサーを使う。モジュールに無理な力を掛けない。','sub')
 t(70,1042,'無通電でVCC/GND/SDA/SCL/AD0の導通と短絡なしを測定後、1個ずつ挿して0x68/0x69を確認する。','sub')
 A.append('</svg>')
 name=f'IMU{imu}_'+('部品配置' if front else '裏面配線')+'.svg'
 (IMU/name).write_text('\n'.join(A))
for imu in [1,2]:
 for front in [True,False]:draw(imu,front)
model={'socket':{f'{p[0]},{p[1]}':n for p,n in socket.items()},'terminals':{f'{p[0]},{p[1]}':n for p,n in terminal.items()},'capacitor':{f'{p[0]},{p[1]}':n for p,n in capacitor.items()},'IMU1':wires(1),'IMU2':wires(2)}
(IMU/'配線座標.json').write_text(json.dumps(model,ensure_ascii=False,indent=2))
(OUT/'図面レビュー結果.json').write_text(json.dumps({'base':'Rev.8','main_board_routes_unchanged':True,'IMU':reports},ensure_ascii=False,indent=2))

rows=['# Rev.10 IMU小基板：写真の8ピンMPU6050','','メイン基板はRev.8の配置・配線を継承。小基板はCタイプ72×47mm、25列×15行。写真のように回路面を上、ピン列を上側へ向ける。以下は部品面基準（列,行）で、裏面図は左右反転。','',
'## 写真から読み取ったピン順','','|穴|ピン|接続先|','|---|---|---|']
for p,n in socket.items():
 target={'VCC':'3V3','AD0':'IMU1=GND、IMU2=3V3','INT':'未接続','XCL':'未接続','XDA':'未接続'}.get(n,n)
 rows.append(f'|{p}|{n}|{target}|')
rows+=['','XDA/XCLは補助I²C用であり、今回ESP32へつなぐSDA/SCLとは別。INTも使わない。AD0は放置せず、IMU1/2で異なる電位へ接続する。写真の基板には旧Adafruit版の3Voピンはない。','',
'## 部品配置','','|部品|配置|','|---|---|','|1×8Pメスソケット|行3・列8〜15。モジュールのオスピン8本を下向きに挿す|','|C4/C5 100nF|3V3脚=(17,10)、GND脚=(19,10)。各1個、無極性|','|電源2P端子台|(10,14)=3V3、(11,14)=GND|','|信号2P端子台|(13,14)=SCL、(14,14)=SDA|','',
'端子台の前方は基板外へ向け、15mm以上空ける。端子の位置はRev.8から変更していない。ピン列とコンデンサーは移設する。','',
'## 全配線表','','太い実線は0.5mmスズメッキ線、細い破線はAWG24被覆線。裸線は両端と中間はんだ点だけを接合する。部品脚のある穴ではワイヤーを裏面ランド上へ寝かせて接合する。被覆線は両端の約1〜1.5mmのみ剥き、曲がり角・交差で途中を剥かない。交差では被覆を保ち、裸線やソケット脚から約1mm浮かせる。12.5/13.5行はランド間の曲がり位置。','']
for imu in [1,2]:
 rows+=[f'### IMU{imu}（0x{0x67+imu:02X}）','','|No.|ネット|線材|始点|中間はんだ点|終点|曲がり位置（はんだなし）|','|---:|---|---|---|---|---|---|']
 for i,e in enumerate(wires(imu),1):rows.append(f"|{i}|{e['net']}|{e['wire']}|{e['from']}|{','.join(map(str,e['taps'])) or '―'}|{e['to']}|{'→'.join(map(str,e['via'])) or '―'}|")
 rows.append('')
rows+=['## 4芯ケーブル','','|色|メイン基板側|IMU小基板側|','|---|---|---|','|赤|3V3|電源(10,14)=3V3|','|黒|GND|電源(11,14)=GND|','|緑|SDA|信号(14,14)=SDA|','|青|SCL|信号(13,14)=SCL|','',
'AD0の接続は小基板内の1本で完結し、メインとのケーブルは従来どおり4芯。信号端子の左右はメインと小基板で逆なので、必ず信号名・色でつなぐ。外部線はAWG24より線。ねじ端子に入れる素線ははんだメッキしない。','',
'## はんだ付けと確認の順番','','1. 無通電で実物を仮置きし、ピン順、寸法、取付穴、端子前方を確認する。外形22×18mmとピッチ2.54mmは写真に基づく配置の仮定。',
'2. 固定穴は実物の下側2穴から現物写しし、使用するナイロンねじに合う径で加工する。写真だけから穴径やM2.5適合は確定しない。部品を外して加工し、切粉を除去する。',
'3. モジュールにピンが未実装なら1×8オスピンを下向きにはんだ付けする。',
'4. C4/C5、1×8メスソケット、端子台を順に付ける。ソケットはモジュールで整列した後、モジュールを抜いて残りを接合する。',
'5. 裸線No.2/5/8を付ける。',
'6. 被覆線No.1/3/4/6/7を付け、電源とI²Cを検査する。',
'7. No.9でAD0を配線する。IMU1はGND、IMU2は3V3。ADDRジャンパーを探してはんだで短絡する旧手順は使わない。',
'8. ソケットからモジュールを外し、4ネットの導通、AD0の接続先、INT/XCL/XDA未接続、3V3―GNDとSDA―SCLの非短絡を測る。',
'9. ソケット高さに合うスペーサーでモジュールを固定する。1個ずつ挿して電源を切り替えながら、0x68/0x69を確認する。VCCは3V3へ給電する。',
'10. 写真だけではモジュール上のレギュレーター型番は確定できない。3.3V入力で通信しない場合はレギュレーター出力を測り、センサー本体のVDD動作範囲2.375〜3.46Vに入るか確認する。図のVCCを独断で5Vへ変更しない。','',
'## レビュー','','各基板9本（裸線3・被覆線6）、4ネットの導通、裸線と別ネットのランド接触、同一直線での配線重複、未使用ピンの分離、公称部品外形の干渉を図面上で検査済み。現物の寸法・配線高さ・導通・通電は未検証。','',
'## 出典','','ピン順・外観：ユーザー添付写真「Photo 1.jpg」。','AD0のLOW/HIGHと0x68/0x69、センサー本体電圧範囲：[TDK MPU-6000/MPU-6050 Product Specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf)。モジュールの型番をGY-521と断定してはいない。']
(IMU/'実装手順と全配線表.md').write_text('\n'.join(rows)+'\n')
(OUT/'Rev8からの変更点と部品差分.md').write_text('''# Rev.8からRev.10への変更

メインBタイプ基板はRev.8と同じ部品座標・37本の配線を使う。ADSの向きもRev.8のまま（行9、左VDD→右A3）。Rev.9の配置変更は取り込まない。

IMUモジュールは写真の8ピン基板へ変更する。旧6ピン用の小基板配線へ挿してはいけない。C基板のソケット・コンデンサー・裏面配線を作り直し、端子台は行14・列10/11・13/14のまま使える。

|項目|Rev.8|Rev.10|
|---|---|---|
|IMUソケット|1×6P×2|1×8P×2、2.54mmピッチ|
|IMUオスピン|1×6P×2|1×8P×2（モジュール実装済みなら不要）|
|IMU電源ピン|VIN|VCC=(15,3)|
|アドレス|裏面ADDRジャンパー|AD0=(9,3)：IMU1=GND、IMU2=3V3|
|C4/C5|列10/12・行10|列17/19・行10|
|小基板の配線|裸線6区間|各9本：裸線3、AWG24被覆線6|
|追加端子台・外部ケーブル|不要|従来の2P×2/基板、4芯/IMUを継続|
|固定ねじ・スペーサー|Adafruit寸法に基づくM2.5|写真のモジュール実物に合う径・高さを実測|

## 付け替え

1. USBと全ケーブルを外し、IMUをソケットから抜く。
2. C基板の旧裏面6区間、旧1×6ソケット、C4/C5を外す。端子台は残せる。
3. 下側取付穴と新しい上側ピン列を現物で仮合わせする。旧穴を無理に流用しない。
4. IMU図と配線表に従い、新ソケット・C4/C5・9本の配線を実装する。
5. AD0の接続先を個別に検査し、無通電の導通・短絡検査後、1台ずつ通電して0x68/0x69を確認する。

部品差分は1×8Pソケット2個、必要なら1×8Pオスピン2個、移設で脚が足りないC4/C5のみ。連結ピンソケット/ピンヘッダーを使う場合は8ピンずつ切り出す。基板・端子台・抵抗値の追加変更はない。
''')
(OUT/'旧版からの付け替え手順.md').write_text('# Rev.10への付け替え\n\nメインB基板はRev.8と同一。IMU小基板の付け替えは[変更点と部品差分](Rev8からの変更点と部品差分.md)と[新しいIMU配線表](IMU_Cタイプ/実装手順と全配線表.md)に従う。\n')
(OUT/'README.md').write_text('''# Rev.10 配線図一式

Rev.8をベースに、ユーザー添付写真の8ピンMPU6050へ合わせた版。メイン基板はRev.8と同じ37本の配線・部品座標・ADS取付方向を維持する。

|図・資料|内容|
|---|---|
|[メイン部品配置](部品配置.svg)|Bタイプの配置、Rev.8を継承|
|[メイン裏面全配線](裏面④%20全配線（①＋②＋③）.svg)|電源・I²C・ペダルをまとめた裏面図|
|[統合電気回路図](統合電気回路図.svg)|ESP32・ADS・ペダル2個・IMU2個、AD0も含む|
|[IMU1配置](IMU_Cタイプ/IMU1_部品配置.svg) / [裏面](IMU_Cタイプ/IMU1_裏面配線.svg)|Cタイプ、AD0=GND、0x68|
|[IMU2配置](IMU_Cタイプ/IMU2_部品配置.svg) / [裏面](IMU_Cタイプ/IMU2_裏面配線.svg)|Cタイプ、AD0=3V3、0x69|
|[IMU全配線表](IMU_Cタイプ/実装手順と全配線表.md)|各9本の線材・始点・終点・曲がり位置・はんだ点|
|[変更点・部品差分](Rev8からの変更点と部品差分.md)|ソケット8ピン化、固定部品の実測、付け替え手順|
|[工程表](組立・はんだ付け工程表.xlsx)|Rev.8の進捗を引継ぎ、IMUの工程・導通チェックを更新|

写真の向きで回路面を上にし、上側ピン列は左からINT,AD0,XCL,XDA,SDA,SCL,GND,VCC。VCCは3V3へ、INT/XCL/XDAは未接続。小基板の端子台と外部4芯ケーブルの位置・役割はRev.8と同じ。

SVGは拡大・印刷用、PNGは閲覧用。裏面図は部品面の穴番号を保持したまま左右反転。白い切れ目の交差は非接続。

小基板の電気ネット・配線重複・公称部品外形は図面上で確認済み。モジュール外形とピッチは仮定なので現物を仮置きし、固定穴・ねじ径・スペーサー高さを合わせる。実機の導通・絶縁・通信は実装後に確認する。
''')
print(json.dumps(reports,ensure_ascii=False))
