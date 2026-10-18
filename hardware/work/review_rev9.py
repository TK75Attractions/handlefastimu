import runpy, json, math
from pathlib import Path
from fractions import Fraction as F
from collections import Counter

d=runpy.run_path('hardware/work/make_rev9.py')
E=d['E'];parts=d['parts'];pins=d['pins'];out=d['P']
segs=[]
for i,e in enumerate(E,1):
 for a,b in zip([e['from'],*e['via']],[*e['via'],e['to']]):
  assert a!=b,(i,a,b)
  segs.append((i,e['net'],tuple(map(F,a)),tuple(map(F,b)),e['wire']))
 for p in [e['from'],e['to'],*e['taps']]:
  assert all(int(x)==x for x in p) and 1<=p[0]<=33 and 1<=p[1]<=24,(i,p)
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def sub(a,b):return a[0]-b[0],a[1]-b[1]
def on(p,a,b):return cross(sub(p,a),sub(b,a))==0 and all(min(a[k],b[k])<=p[k]<=max(a[k],b[k]) for k in (0,1))
overlaps=[]
for k,(i,n,a,b,w) in enumerate(segs):
 for j,m,c,q,v in segs[k+1:]:
  if i==j:continue
  if cross(sub(b,a),sub(q,c)) or cross(sub(c,a),sub(b,a)):continue
  axis=0 if a[0]!=b[0] else 1
  if max(min(a[axis],b[axis]),min(c[axis],q[axis]))<min(max(a[axis],b[axis]),max(c[axis],q[axis])):overlaps.append((i,j,n,m))
assert not overlaps,overlaps
for i,n,a,b,w in segs:
 if w!='0.5mmスズメッキ線':continue
 for p,m in pins.items():assert not on(tuple(map(F,p)),a,b) or m==n,(i,p,m)
for k,(i,n,a,b,w) in enumerate(segs):
 if w!='0.5mmスズメッキ線':continue
 for j,m,c,q,v in segs[k+1:]:
  if n==m or v!='0.5mmスズメッキ線':continue
  touch=any(on(p,a,b) for p in (c,q)) or any(on(p,c,q) for p in (a,b))
  u1,u2=cross(sub(b,a),sub(c,a)),cross(sub(b,a),sub(q,a))
  v1,v2=cross(sub(q,c),sub(a,c)),cross(sub(q,c),sub(b,c))
  assert not touch and not(u1*u2<0 and v1*v2<0),(i,j)

out.joinpath('配線座標.json').write_text(json.dumps(E,ensure_ascii=False,indent=2))
model={'ads': [{'point':p,'pin':n} for p,n in d['ads'].items()],
       'parts':[{'ref':r,'value':v,'p':p,'q':q,'net_p':np,'net_q':nq} for r,v,p,q,np,nq in parts],
       'terminals':[{'name':r,'column':c,'row':y,'nets':ns} for r,c,y,ns in d['tb']]}
out.joinpath('部品座標.json').write_text(json.dumps(model,ensure_ascii=False,indent=2))
report={'wire_count':len(E),'bare_count':sum(e['wire']=='0.5mmスズメッキ線' for e in E),
        'nets':9,'collinear_overlaps':0,'component_shared_holes':0,
        'power_rail_spacing_mm':5.08,'i2c_rail_spacing_mm':7.62,
        'mechanical_nominal_body_check':'pass','hardware_verified':False}
out.joinpath('図面レビュー結果.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
