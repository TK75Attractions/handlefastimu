from pathlib import Path
from html import escape
import json
import shutil

P = Path('hardware/Rev8')
P.mkdir(parents=True, exist_ok=True)

COL = {
    '3V3': '#d43c51', 'GND': '#293b4d', 'SDA': '#188849',
    'SCL': '#2673c7', 'SIG1': '#b68215', 'SIG2': '#b26120',
    'ADC0': '#9a54b5', 'ADC1': '#ca5089', 'GPIO27': '#e79610'
}

# Component-side coordinates, 1-based.  The ADS module is installed circuit-side
# up.  Its photographed one-row header reads left-to-right as below.
ads_order = ['VDD', 'GND', 'SCL', 'SDA', 'ADDR', 'ALRT', 'A0', 'A1', 'A2', 'A3']
ads = {(20 + i, 9): name for i, name in enumerate(ads_order)}

# Every terminal block is on the lower board edge, with its cable opening facing
# down/outward.  One empty column separates neighbouring blocks.
tb = [
    ('MPU1電源', 14, 24, ['3V3', 'GND']),
    ('MPU1信号', 17, 24, ['SDA', 'SCL']),
    ('MPU2電源', 20, 24, ['3V3', 'GND']),
    ('MPU2信号', 23, 24, ['SDA', 'SCL']),
    ('ペダル1', 26, 24, ['3V3', 'SIG1', 'GND']),
    ('ペダル2', 30, 24, ['3V3', 'SIG2', 'GND']),
]

parts = [
    ('R1', '1kΩ', (25, 13), (28, 13), 'SIG1', 'ADC0'),
    ('C1', '100nF', (28, 13), (28, 15), 'ADC0', 'GND'),
    ('R2', '1kΩ', (25, 16), (28, 16), 'SIG2', 'ADC1'),
    ('C2', '100nF', (28, 16), (28, 18), 'ADC1', 'GND'),
    ('C3', '100nF', (20, 11), (21, 11), '3V3', 'GND'),
    ('C6', '10µF', (19, 20), (19, 22), '3V3', 'GND'),
    ('R7', '100kΩ', (17, 13), (20, 13), 'GPIO27', 'ADC0'),
    ('R8', '100kΩ', (17, 16), (20, 16), 'GPIO27', 'ADC1'),
]

E = []
def add(net, p, q, stage, bends=(), taps=()):
    E.append({'net': net, 'from': p, 'to': q, 'stage': stage,
              'via': list(bends), 'taps': list(taps)})

# Four straight distribution rails.  Only listed taps are soldered; all other
# bends in later insulated wires are routing hints, not electrical joints.
add('3V3', (14, 20), (30, 20), 'rail_power', taps=[(15, 20), (19, 20), (20, 20), (22, 20), (26, 20)])
add('GND', (15, 22), (32, 22), 'rail_power', taps=[(16, 22), (19, 22), (21, 22), (22, 22), (27, 22), (28, 22), (29, 22)])
add('SDA', (17, 18), (23, 18), 'rail_i2c')
add('SCL', (18, 19), (24, 19), 'rail_i2c')

# Edge terminal block drops to the rails.
for net, p, q in [
    ('3V3',(14,24),(14,20)), ('GND',(15,24),(15,22)),
    ('3V3',(20,24),(20,20)), ('GND',(21,24),(21,22)),
    ('3V3',(26,24),(26,20)), ('GND',(28,24),(28,22)),
    ('3V3',(30,24),(30,20)), ('GND',(32,24),(32,22)),
]: add(net,p,q,'terminal_power')
for net, p, q in [
    ('SDA',(17,24),(17,18)), ('SCL',(18,24),(18,19)),
    ('SDA',(23,24),(23,18)), ('SCL',(24,24),(24,19)),
]: add(net,p,q,'terminal_i2c')

# Power and ground.
add('3V3',(3,3),(15,20),'power',[(1,4),(1,21),(15,21)])
add('GND',(13,3),(22,22),'power',[(12,4),(12,23),(22,23)])
add('3V3',(20,9),(20,11),'power')
add('GND',(21,9),(21,11),'power')
add('3V3',(20,11),(22,20),'power',[(22,12)])
add('GND',(21,11),(16,22),'power',[(16,11)])
add('GND',(24,9),(21,9),'power',[(24,8),(21,8)])  # ADS ADDR=GND -> 0x48

# I2C.
add('SDA',(13,8),(17,18),'i2c',[(15,8),(15,18)])
add('SCL',(13,5),(18,19),'i2c',[(14,5),(14,19)])
add('SDA',(23,9),(23,18),'i2c')
add('SCL',(22,9),(24,19),'i2c',[(22,10),(24,10)])

# Pedal inputs and ADC nodes.
add('SIG1',(27,24),(25,13),'analog',[(27,23.5),(33,23.5),(33,14),(25,14)])
add('SIG2',(31,24),(25,16),'analog',[(31,23),(25,23)])
add('ADC0',(26,9),(28,13),'analog',[(26,10),(30,10),(30,13)])
add('ADC1',(27,9),(28,16),'analog',[(27,11),(31,11),(31,16)])
add('ADC0',(20,13),(28,13),'analog',[(20,12),(28,12)])
add('ADC1',(20,16),(28,16),'analog',[(20,17),(28,17)])
add('GND',(28,15),(29,22),'analog',[(29,15)])
add('GND',(28,18),(27,22),'analog',[(27,18)])

# GPIO27 common side of R7/R8.
add('GPIO27',(3,13),(17,13),'gpio',[(15,13)])
add('GPIO27',(17,13),(17,16),'gpio')

for e in E:
    e['wire'] = '0.5mmスズメッキ線' if (e['stage']=='rail_power' or (e['stage']=='terminal_power' and e['net']=='GND')) else 'AWG24被覆線'
assert len(E) == 37
assert sum(e['wire']=='0.5mmスズメッキ線' for e in E)==6

# --- Electrical and mechanical validation ---------------------------------
pins = {}
def claim(p, net):
    old = pins.setdefault(p, net)
    assert old == net, (p, old, net)

for p, n in [
    ((3,3),'3V3'), ((13,3),'GND'), ((13,8),'SDA'),
    ((13,5),'SCL'), ((3,13),'GPIO27'),
    ((20,9),'3V3'), ((21,9),'GND'), ((22,9),'SCL'),
    ((23,9),'SDA'), ((24,9),'GND'), ((26,9),'ADC0'), ((27,9),'ADC1')
]: claim(p,n)
for _, c, r, ns in tb:
    for j, n in enumerate(ns): claim((c+j,r),n)
for _,_,p,q,np,nq in parts: claim(p,np); claim(q,nq)
for e in E:
    claim(e['from'],e['net']); claim(e['to'],e['net'])
    for p in e['taps']: claim(p,e['net'])

parent = {}
def root(p):
    parent.setdefault(p,p)
    if parent[p] != p: parent[p] = root(parent[p])
    return parent[p]
for e in E:
    a0,b0=root(e['from']),root(e['to']); parent[a0]=b0
    for p in e['taps']: parent[root(p)] = root(e['from'])
groups = {}
for p,n in pins.items(): groups.setdefault(root(p),set()).add(n)
assert all(len(ns)==1 for ns in groups.values()), groups

expected = {
 '3V3':[(3,3),(20,9),(20,11),(19,20),(14,24),(20,24),(26,24),(30,24)],
 'GND':[(13,3),(21,9),(24,9),(21,11),(19,22),(15,24),(21,24),(28,24),(32,24)],
 'SDA':[(13,8),(23,9),(17,18),(23,18),(17,24),(23,24)],
 'SCL':[(13,5),(22,9),(18,19),(24,19),(18,24),(24,24)],
 'ADC0':[(26,9),(28,13),(20,13)],
 'ADC1':[(27,9),(28,16),(20,16)],
 'SIG1':[(27,24),(25,13)], 'SIG2':[(31,24),(25,16)],
 'GPIO27':[(3,13),(17,13),(17,16)]
}
for net, pts in expected.items(): assert len({root(p) for p in pts}) == 1, net
assert len({root(expected[n][0]) for n in expected}) == len(expected)

def mx(c): return 6.86+(c-1)*2.54
def my(r): return 6.79+(r-1)*2.54
def overlaps(a,b): return a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]

tb_boxes=[]
for title,c,r,ns in tb:
    cx=(mx(c)+mx(c+len(ns)-1))/2; cy=my(r)
    halfx=(5.54 if len(ns)==2 else 8.08)/2
    box=(cx-halfx,cy-3.1,cx+halfx,cy+3.1)
    assert box[0]>=0 and box[2]<=95 and box[1]>=0 and box[3]<=72, (title,box)
    tb_boxes.append((title,box))
for i,(ta,ba) in enumerate(tb_boxes):
    for tb_name,bb in tb_boxes[i+1:]: assert not overlaps(ba,bb),(ta,tb_name)

# Clear of the lower-right mounting hole (nominal centre 92,69 mm, radius 3.2 mm).
for title,(x1,y1,x2,y2) in tb_boxes:
    nx=min(max(92,x1),x2); ny=min(max(69,y1),y2)
    assert (nx-92)**2+(ny-69)**2 > 3.2**2,(title,'mounting-hole overlap')

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

    t(40,47,f'Rev.8：{name}','h')
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

        # ADS board, circuit side up.  Pin row is its lower edge.
        rect_mm(53.0,8.95,30.0,20.7,'#cfe8de')
        x,y=xy((20,9));t(x+36,y-83,'ADS1115','h');t(x+39,y-58,'回路面を上','sub')
        x1,y1=xy((20,9));x2,_=xy((29,9))
        out.append(f'<rect x="{x1-7}" y="{y1-7}" width="{x2-x1+14}" height="14" fill="#2b4d41"/>')
        for p,n in ads.items():
            circle(p,'white',3);x,y=xy(p)
            out.append(f'<text transform="translate({x-3},{y+34}) rotate(-90)" class="tiny">{escape(n)}</text>')

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
        x1,y=xy((20,9));x2,_=xy((29,9));out.append(f'<rect x="{min(x1,x2)-7}" y="{y-7}" width="{abs(x2-x1)+14}" height="14" fill="none" stroke="#9aaab5" stroke-dasharray="4 4"/>')
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
    x,y=xy((19,20));t(x+8,y-7,'C6＋','tiny');x,y=xy((19,22));t(x+8,y-6,'−','tiny')

    t(890,183,'作業の要点','h')
    note_map={
      '部品配置':[
        'ADS：行9、列20〜29。回路面を上にする。',
        'ADS左→右：VDD,GND,SCL,SDA,ADDR,ALRT,A0,A1,A2,A3。',
        '端子台はすべて行24。挿入口は下（基板外）向き。',
        '端子台同士は1列空け、前方15mm以上を空ける。',
        '1×10PメスソケットをB基板へ、オスピンをADSへ付ける。',
        'ALRT/A2/A3は未接続。ADDRはGNDへ接続し0x48。'],
      '裏面① 電源・GND':[
        '3V3横レール：行20、列14〜30。',
        'GND横レール：行22、列15〜32。行21は裸線なし。',
        '赤い端子電源線は被覆線。GND端子4本だけ裸の直線。',
        'C6＋(19,20)／−(19,22)はレールへ直接接続。',
        'ADS ADDR(24,9)をGND(21,9)へ接続。'],
      '裏面② I²C':[
        'SDA行18・列17〜23：AWG24被覆線（裸線を廃止）。',
        'SCL行19・列18〜24：AWG24被覆線（裸線を廃止）。',
        'ESP GPIO21とADS SDAをSDAレールへ。',
        'ESP GPIO22とADS SCLをSCLレールへ。',
        'MPU1/2信号端子は各レールへ短く接続。'],
      '裏面③ ペダル・ADC・GPIO27':[
        'P1 SIG(27,24)→R1→A0。P2 SIG(31,24)→R2→A1。',
        'C1/C2はADC側からGNDレールへ。',
        'R7/R8のGPIO27側だけを共通化。',
        'A0/A1の長い配線は列30/31側で分離。',
        '途中の曲がり点や交差にははんだ付けしない。'],
      '裏面④ 全配線（①＋②＋③）':[
        '全37本：裸線6本＋AWG24被覆線31本。',
        '太い実線＝裸線、細い破線＝被覆線。分岐を分散。',
        '交差部は白い切れ目で上下を表し、●がない交差は非接続。',
        '色はネット名を表す。端子台前方の空間には線を置かない。',
        '赤と黒の裸レールの間は1行空ける。交差には被覆を残す。']}
    for i,s in enumerate(note_map[name]): t(890,223+31*i,s,'sub')
    t(890,445,'主要な穴番号','h')
    key=[
      'ESP 3V3=(3,3)、GND=(13,3)', 'GPIO21=(13,8)、GPIO22=(13,5)',
      'GPIO27=(3,13)', 'ADS VDD=(20,9)、GND=(21,9)',
      'ADS SCL=(22,9)、SDA=(23,9)', 'ADS ADDR=(24,9)、A0=(26,9)、A1=(27,9)',
      '電源レール：3V3行20／GND行22（間隔5.08mm）', 'I²C分配線：SDA行18／SCL行19、両方被覆線',
      'C6＝＋(19,20)、−(19,22)。端子台は行24。']
    for i,s in enumerate(key): t(890,485+27*i,s,'sub')
    t(890,760,'配線の注意','h')
    t(890,795,'白い切れ目の交差は非接続。曲がり角ははんだ点ではない。','sub')
    t(890,825,'C6は脚を5.08mmに成形して2本のレールへ直接接続。','sub')
    t(890,855,'P1信号の水平線は行23と24の間。途中で穴へ差し込まない。','sub')
    t(80,1060,'太い実線＝0.5mmスズメッキ線6本。細い破線＝AWG24被覆線31本。紫の記号＝表面側の抵抗・コンデンサー位置。','sub')
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

# Documentation and coordinate tables are generated from the same net list.
rows=[
 '# Rev.8：電源分岐の分散・裸レール間隔の拡大 Bタイプ基板','',
 'Bタイプ95×72mmのメイン基板にESP32-DevKitC-32Eと、添付写真の1×10ピンADS1115を載せる。ADSは回路面を上にし、ピン列は表面から見て左から `VDD, GND, SCL, SDA, ADDR, ALRT, A0, A1, A2, A3`。MPU6050は従来どおりCタイプ小基板2枚へ載せる。','',
 '[裏面①〜③を重ねた全配線図](裏面④%20全配線（①＋②＋③）.svg)では、異なるネットが同じ直線区間を共有しないよう全経路を分離している。交差部に●がなければ非接続。','',
 '## 今回の修正','',
 '- GND裸レールを行21から行22へ移動し、3V3行20と5.08mm離した。行21に横向きの裸線を敷かない。',
 '- C6を＋(19,20)／−(19,22)へ移動し、両脚をレールへ直接接続する。旧(16,20)／(16,21)に部品を残さない。',
 '- ESP32の3V3は(15,20)、ADSの3V3は(22,20)へ接続。GNDもESP32=(22,22)、ADS=(16,22)へ分散した。',
 '- I²Cの横分配線2本は裸線からAWG24被覆線へ変更。3V3の隣に裸のSCL線を置かない。',
 '- GND端子からレールまでの4本は裸の直線5.08mmとし、短い被覆線4本の剥離作業をなくした。赤の3V3端子線4本はGNDレールを横切るため、必ず被覆を残す。',
 '- ペダル1信号の横区間は行23と24の中間へ移し、GNDレール上を走る重なりをなくした。半端な行番号は曲がり位置で、はんだ穴ではない。',
 '- ソケット・端子台・R1/R2/R7/R8/C1/C2/C3の穴位置はRev.7のまま。端子台前方15mmも確保。', '',
 '## ADS取付方向','',
 '回路面の部品と印字が見える状態を上とする。表面図の列20がVDD側、列29がA3側。B基板には1×10Pメスソケットを取り付け、ADSには短い側を基板へ通してはんだ付けした1×10Pオスピンを付け、長いピン側をソケットへ挿す。ピンピッチが2.54mmであることを現物で確認してからソケットを固定する。','',
 '## 端子台とケーブル空間','',
 'すべて行24に置き、挿入口を基板外側へ向ける。ケーブルは端子から下へまっすぐ出し、基板端から15mm以上離れた位置でストレインリリーフを設ける。端子の真正面にはスペーサー、ケース壁、結束バンド台座を置かない。端子台のねじは上から操作できる配置とする。','',
 '## はんだ付け順','',
 '1. ADS、ESP32、全端子台を無通電で仮置きし、ADSの左端がVDD、端子台の開口が基板外向きであることを確認する。','2. R1/R2/R7/R8、C1/C2/C3をはんだ付けする。','3. ESP32ソケット、ADSの1×10Pソケットを仮止めし、モジュールを一度挿して位置を確認してから全ピンを仕上げる。','4. 行24の端子台を左から順に付ける。最初の1脚だけ仮止めし、開口方向と直角を確認してから残りを付ける。','5. C6の脚を5.08mm間隔に成形し、＋(19,20)、−(19,22)へ付ける。根元に力をかけず、無理に引っ張らない。','6. 裏面①の3V3/GND横レール、端子電源線、ESP/ADS電源を配線する。','7. 裏面②のSDA/SCL横分配線を含め、I²CはすべてAWG24被覆線で配線する。','8. 裏面③のペダル、A0/A1、GPIO27を配線し、最後に裏面④全配線図と照合する。','9. ESP32/ADS/MPUをすべて外した状態で導通・短絡を検査する。','',
 '## 部品脚','', '|部品|片側|もう片側|','|---|---|---|']
for ref,val,p,q,np,nq in parts: rows.append(f'|{ref} {val}|{p} {np}|{q} {nq}|')
rows += ['', '## ADSソケット（表面、左から右）','', '|列・行|ピン|接続|','|---|---|---|']
ads_connect={'VDD':'3V3','GND':'GND','SCL':'ESP GPIO22・MPU SCL','SDA':'ESP GPIO21・MPU SDA','ADDR':'GND（0x48）','ALRT':'未接続','A0':'ADC0','A1':'ADC1','A2':'未接続','A3':'未接続'}
for p,n in ads.items(): rows.append(f'|{p}|{n}|{ads_connect[n]}|')
rows += ['', '## 端子台（すべて行24、挿入口は基板外向き）','', '|用途|表面から見た穴と順番|','|---|---|']
for title,c,r,ns in tb: rows.append(f'|{title}|行{r}：'+', '.join(f'列{c+j}={n}' for j,n in enumerate(ns))+'|')
rows += ['', '## 全配線表（表面座標、裏面作業）','', '|No.|段階|ネット|線材|はんだ付け元|途中のはんだ点|はんだ付け先|曲がり位置（はんだ付けしない）|','|---:|---|---|---|---|---|---|---|']
stage_names={'rail_power':'①電源横レール','rail_i2c':'②I²C横レール','terminal_power':'①端子台から電源レール','terminal_i2c':'②端子台からI²Cレール','power':'①電源/GND','i2c':'②I²C','analog':'③ペダル/ADC','gpio':'③GPIO27'}
for i,e in enumerate(E,1): rows.append(f"|{i}|{stage_names[e['stage']]}|{e['net']}|{e['wire']}|{e['from']}|{', '.join(map(str,e['taps'])) if e['taps'] else '―'}|{e['to']}|{' → '.join(map(str,e['via'])) if e['via'] else '―'}|")
rows += ['', '曲がり点、通過ランド、線の交差は接続点ではない。表の「途中のはんだ点」だけをレールの中継点としてはんだ付けする。被覆線が裸レールを横切る箇所は被覆を残し、線同士を密着させない。','',
 '## 電気的な検査結果','',
 f'ESP32、ADS、端子台、受動部品の全端点をネット照合した。{len(expected)}ネットは相互に短絡せず、同名点は配線表上で導通する。抵抗・コンデンサーは導通線として扱わず、計{len(E)}本（裸線6本＋被覆線31本）の端点を検査した。端子台は秋月2P/3P品の公称外形で互いに重ならず、右下取付穴とも重ならない。','',
 '## 部品変更','',
 '- 従来のADS用2列ソケットは使わない。2.54mmピッチ1×10Pメスソケット1本と、ADSへはんだ付けする1×10Pオスピンヘッダー1本を使う。','- メイン基板の端子台数は2P×4個、3P×2個のまま。すべて下端に置く。','- 電源の横レール2本とGND端子の縦線4本だけ0.5mmスズメッキ線、その他31本はAWG24被覆単線または同等品を使う。端子台から外へ出すケーブルはAWG24より線を使う。','',
 '## 通電前の必須確認','',
 '- ADS回路面が上で、列20=VDD、列21=GND、列22=SCL、列23=SDA、列24=ADDR、列26=A0、列27=A1になっている。','- 端子台6個の挿入口がすべて外向きで、前方15mm以上に障害物がない。','- 3V3―GNDが短絡していない。SDA―SCL、A0―A1も0Ωで短絡していない。','- ADS ADDR―GNDが導通し、ALRT/A2/A3は未接続。','- 段階通電でADS=0x48、IMU1=0x68、IMU2=0x69を確認する。']
(P/'実装手順と全配線表.md').write_text('\n'.join(rows))
(P/'配線座標.json').write_text(json.dumps(E,ensure_ascii=False,indent=2))

# IMU C boards are electrically and mechanically unchanged; copy the already
# reviewed drawings so Rev.8 is a self-contained deliverable.
src=Path('/Users/kenichirowada/Documents/Codex/2026-09-13/esp32/outputs/Rev7/IMU_Cタイプ'); dst=P/'IMU_Cタイプ'
if dst.exists(): shutil.rmtree(dst)
shutil.copytree(src,dst)
imu_doc=dst/'実装手順と全配線表.md'
imu_doc.write_text(imu_doc.read_text().replace('# Rev.7 IMU小基板：Cタイプ配線（配置はRev.6から変更なし）','# Rev.8 IMU小基板：Cタイプ配線（配置はRev.7から変更なし）'))

print(f'Rev.8 generated: {len(E)} insulated/rail connections; {len(expected)} net groups checked; all 6 terminal blocks edge-mounted.')
