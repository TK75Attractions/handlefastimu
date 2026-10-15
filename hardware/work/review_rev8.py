from pathlib import Path
from fractions import Fraction as F
from collections import defaultdict, Counter
import json, xml.etree.ElementTree as ET

out=Path('hardware/Rev8')
old=json.loads(Path('/Users/kenichirowada/Documents/Codex/2026-09-13/esp32/outputs/Rev7/配線座標.json').read_text())
edges=json.loads((out/'配線座標.json').read_text())
segs=[]; points=defaultdict(set); joints=Counter()
for i,e in enumerate(edges,1):
 ps=[e['from'],*e['via'],e['to']]
 for a,b in zip(ps,ps[1:]):
  assert a!=b,(i,a,b)
  segs.append((i,e['net'],tuple(map(F,a)),tuple(map(F,b)),e['wire']))
 for p in [e['from'],e['to'],*e['taps']]:
  assert all(int(t)==t for t in p) and 1<=p[0]<=33 and 1<=p[1]<=24,(i,p)
  points[tuple(p)].add(e['net']);joints[tuple(p)]+=1
 assert all(1<=p[0]<=33 and 1<=p[1]<=24 for p in ps)
assert all(len(n)==1 for n in points.values()),points

def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def sub(a,b):return a[0]-b[0],a[1]-b[1]
def on(p,a,b):
 return cross(sub(p,a),sub(b,a))==0 and all(min(a[k],b[k])<=p[k]<=max(a[k],b[k]) for k in (0,1))

# All orientations, including diagonals and the half-row signal route.
overlaps=[]
for k,(i,n,a,b,w) in enumerate(segs):
 for j,m,c,d,v in segs[k+1:]:
  if i==j:continue
  if cross(sub(b,a),sub(d,c)) or cross(sub(c,a),sub(b,a)):continue
  axis=0 if a[0]!=b[0] else 1
  if max(min(a[axis],b[axis]),min(c[axis],d[axis])) < min(max(a[axis],b[axis]),max(c[axis],d[axis])):
   overlaps.append((i,j,n,m))
assert not overlaps,overlaps

# A bare wire must never pass through a solder joint on another net.
for i,n,a,b,w in segs:
 if w!='0.5mmスズメッキ線':continue
 for p,ns in points.items():
  assert not on(tuple(map(F,p)),a,b) or ns=={n},(i,p,ns)
for k,(i,n,a,b,w) in enumerate(segs):
 if w!='0.5mmスズメッキ線':continue
 for j,m,c,d,v in segs[k+1:]:
  if v!='0.5mmスズメッキ線' or n==m:continue
  touch=any(on(p,a,b) for p in (c,d)) or any(on(p,c,d) for p in (a,b))
  u1,u2=cross(sub(b,a),sub(c,a)),cross(sub(b,a),sub(d,a))
  v1,v2=cross(sub(d,c),sub(a,c)),cross(sub(d,c),sub(b,c))
  assert not touch and not(u1*u2<0 and v1*v2<0),(i,j)

# Independently check all named electrical endpoints. Resistors and capacitors
# are deliberately excluded from the wire continuity graph.
parent={}
def root(p):
 p=tuple(p);parent.setdefault(p,p)
 if parent[p]!=p:parent[p]=root(parent[p])
 return parent[p]
for e in edges:
 a=root(e['from'])
 for p in [e['to'],*e['taps']]:parent[root(p)]=root(a)
req={
 '3V3':[(3,3),(20,9),(20,11),(19,20),(14,24),(20,24),(26,24),(30,24)],
 'GND':[(13,3),(21,9),(24,9),(21,11),(19,22),(28,15),(28,18),(15,24),(21,24),(28,24),(32,24)],
 'SDA':[(13,8),(23,9),(17,24),(23,24)],
 'SCL':[(13,5),(22,9),(18,24),(24,24)],
 'ADC0':[(26,9),(28,13),(20,13)], 'ADC1':[(27,9),(28,16),(20,16)],
 'SIG1':[(27,24),(25,13)],'SIG2':[(31,24),(25,16)],
 'GPIO27':[(3,13),(17,13),(17,16)]}
for n,ps in req.items():
 assert len({root(p) for p in ps})==1,n
 assert all(points[p]=={n} for p in ps),n
assert len({root(ps[0]) for ps in req.values()})==9
assert all(a['net']==b['net'] and a['stage']==b['stage'] for a,b in zip(old,edges))
assert (16,20) not in points and (16,21) not in points
power_joints={p:count for p,count in joints.items() if p[1] in (20,22) and p in points and points[p] <= {'3V3','GND'}}
assert max(power_joints.values())<=2,power_joints
for p in out.rglob('*.svg'):ET.parse(p)

def fmt(p):return f"({p[0]},{p[1]})"
def route(e):return ' → '.join(fmt(p) for p in [e['from'],*e['via'],e['to']])
changed=[(i,a,b) for i,(a,b) in enumerate(zip(old,edges),1) if any(a[k]!=b[k] for k in ('from','to','via','taps')) or (a['stage']=='rail_i2c' or a['stage']=='terminal_power' and a['net']=='GND')]
rows=['# Rev.7からRev.8への付け替え手順','','座標はすべて部品面基準の（列,行）。裏面図は左右を反転して表示する。USB、ESP32、ADS、IMU、ペダルケーブルを外して作業する。既存の3V3―GND短絡は、この変更だけで解消したと判断せず、工程ごとに実測する。','',
 '1. 旧C6を外す。旧行21のGND裸レール、SDA/SCL裸レール、表のNo.1以外の変更対象線を外す。No.1の3V3裸レールは位置を保ち、旧分岐を外してタップだけ変更する。元の穴は吸取線で余分なはんだを除き、隣のランドと切れていることを確認する。',
 '2. C6を＋(19,20)、−(19,22)へ移す。脚は5.08mm間隔に成形する。すでに短く切った脚が届かなければC6だけ新品に交換する。脚同士を裸線で延長して寄せ集めない。',
 '3. 3V3裸レールは行20、GND裸レールは行22に敷く。C6の2脚をレールへ直接接続する。旧(16,20)/(16,21)は分岐点として使わない。',
 '4. 端子台のGND4本を0.5mm裸線の直線で行22へつなぐ。長さは穴中心間5.08mm。両端ランド上に寝かせてはんだ付けする。3V3端子線はAWG24被覆線で、GNDを横切る部分の被覆を残す。',
 '5. ESP32/ADS電源とC1/C2のGNDを下表どおり付け替える。レールの各分岐穴は「裸レール＋1本の線または部品脚」だけとし、同じ穴へ線を寄せ集めない。',
 '6. SDA/SCLの横分配線をAWG24被覆線へ交換する。両端だけを剥き、途中の行19や行20の交差部分に裸線を残さない。その他のI²C線の端点は以前と同じ。',
 '7. ペダル1信号の水平区間を行23と行24の中間へ移す。曲がり位置(27,23.5)/(33,23.5)は被覆線を曲げる位置で、穴やはんだ点ではない。',
 '8. 被覆線の交差は被覆を残し、他のはんだ山や裸線に押し付けない。横レールに並走・接触させない。全配線図と照合し、3V3全点、GND全点、3V3―GND絶縁、SDA/SCL分離を確認してからモジュールを戻す。','',
 '## 変更対象の配線','','配線番号は全配線表と共通。No.1はレール位置を維持して下記はんだ点へ更新。それ以外は両端を外して経路・線材を作り直す。部分付け替えの場合も旧配線を残して2本の経路を作らない。','',
 '|No.|ネット|Rev.7の経路|Rev.8の経路|Rev.8の線材|','|---:|---|---|---|---|']
for i,a,b in changed:rows.append(f"|{i}|{b['net']}|{route(a)}|{route(b)}|{b['wire']}|")
rows+=['','## 裸線のはんだ点','','工程20では横レール2本の下記16点を付ける。端子GND縦線4本は工程21で付ける。●をすべて工程20で付けるわけではない。','',
 '|裸線|はんだ付けする穴|','|---|---|']
for e in edges[:2]:rows.append(f"|{e['net']}|"+'、'.join(fmt(p) for p in sorted([e['from'],e['to'],*e['taps']]))+'|')
rows+=['','工程21のGND縦線は(15,22)↔(15,24)、(21,22)↔(21,24)、(28,22)↔(28,24)、(32,22)↔(32,24)。途中の行23ははんだ付けしない。','',
 '## 図面上の確認','','- 9ネットの接続先は旧版と一致。抵抗・コンデンサーの回路構成は維持。',
 '- 全37配線について、斜め区間を含む同一直線区間の重複なし。',
 '- 裸線6本は異なるネットの裸線やはんだ点を通らない。3V3/GND横レール中心間隔5.08mm。',
 '- 電源横レールの1穴はレールと分岐1本（またはC6脚）に限定。',
 '- 端子台6個は行24外向き。C6は公称直径5mm以下を想定し、列19・行21付近で端子台前方を塞がない。実物の径と脚長は仮置きで確認。',
 '- 図面上の確認であり、実機の導通・絶縁・通電試験は未実施。',
 '- 工程表は見つかった入力済みAutoRecovered版の進捗・日付・メモを引き継ぐ。変更対象の完了工程18/20/21/22/23/24は「再作業」に変更し、その理由をメモへ追記。その他の進捗は引き継ぐ。']
(out/'旧版からの付け替え手順.md').write_text('\n'.join(rows))
report={'net_groups':9,'wires':37,'bare':6,'insulated':31,'overlapping_segments':0,'bare_foreign_joint_contacts':0,'power_rail_pitch_mm':5.08,'old_16_20_used':False,'modified_wire_numbers':[i for i,a,b in changed]}
(out/'図面レビュー結果.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
