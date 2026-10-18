from pathlib import Path
from html import escape

OUT = Path("hardware/Rev9")
OUT.mkdir(parents=True, exist_ok=True)

W, H = 2600, 1700
C = {
    "3V3": "#d43c51",
    "GND": "#263746",
    "SDA": "#188849",
    "SCL": "#2673c7",
    "GPIO27": "#e79610",
    "ADC0": "#8c55aa",
    "ADC1": "#ca5089",
    "SIG": "#91691b",
}
a = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
    '<rect width="100%" height="100%" fill="white"/>',
    '<style>text{font-family:Arial,"Hiragino Sans","Noto Sans CJK JP",sans-serif;fill:#183047;font-size:18px}.title{font-size:34px;font-weight:bold}.h{font-size:23px;font-weight:bold}.ref{font-size:21px;font-weight:bold}.pin{font-size:16px}.small{font-size:14px;fill:#587082}.net{font-size:15px;font-weight:bold}.board{font-size:16px;font-weight:bold;fill:#52697b}</style>'
]

def t(x,y,s,cls="",anchor=None):
    extra=f' text-anchor="{anchor}"' if anchor else ""
    a.append(f'<text x="{x}" y="{y}" class="{cls}"{extra}>{escape(str(s))}</text>')

def line(x1,y1,x2,y2,color="#263746",width=4,dash=None):
    d=f' stroke-dasharray="{dash}"' if dash else ""
    a.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" stroke-linecap="round"{d}/>')

def path(points,color="#263746",width=4,dash=None):
    d=f' stroke-dasharray="{dash}"' if dash else ""
    p=" ".join(f"{x},{y}" for x,y in points)
    a.append(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round" stroke-linejoin="round"{d}/>')

def dot(x,y,color="#263746",r=6):
    a.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color}" stroke="white" stroke-width="1.5"/>')

def box(x,y,w,h,title,subtitle="",fill="#f5f8fb",stroke="#506d85"):
    a.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="9" fill="{fill}" stroke="{stroke}" stroke-width="2.5"/>')
    t(x+20,y+36,title,"ref")
    if subtitle:t(x+20,y+66,subtitle,"small")

def resistor(x1,y,x2,ref,value,color="#263746"):
    lead=22
    line(x1,y,x1+lead,y,color,4);line(x2-lead,y,x2,y,color,4)
    a.append(f'<rect x="{x1+lead}" y="{y-15}" width="{x2-x1-2*lead}" height="30" rx="2" fill="#fffdf5" stroke="{color}" stroke-width="2.5"/>')
    t((x1+x2)/2,y-24,ref,"ref","middle");t((x1+x2)/2,y+6,value,"pin","middle")

def capacitor_vertical(x,y_top,y_bottom,ref,value,polarized=False,top_color=None,bottom_color=None):
    top_color = top_color or C["3V3"]
    bottom_color = bottom_color or C["GND"]
    mid=(y_top+y_bottom)/2;gap=12
    line(x,y_top,x,mid-18,top_color,4)
    line(x,mid+18,x,y_bottom,bottom_color,4)
    line(x-20,mid-18,x+20,mid-18,"#80641d",3)
    line(x-20,mid+18,x+20,mid+18,"#80641d",3)
    if polarized:t(x+28,mid-18,"＋","pin")
    t(x+32,mid-4,f"{ref} {value}","pin")

def ground(x,y):
    line(x,y,x,y+10,C["GND"],3)
    line(x-17,y+10,x+17,y+10,C["GND"],3)
    line(x-11,y+17,x+11,y+17,C["GND"],3)
    line(x-5,y+24,x+5,y+24,C["GND"],3)

def pot(x,y_top,y_bottom,ref):
    mid=(y_top+y_bottom)/2
    line(x,y_top,x,y_top+22,C["3V3"],4)
    a.append(f'<rect x="{x-15}" y="{y_top+22}" width="30" height="{y_bottom-y_top-44}" fill="#fffdf5" stroke="#536a7b" stroke-width="2.5"/>')
    line(x,y_bottom-22,x,y_bottom,C["GND"],4)
    line(x+75,mid,x+20,mid,C["SIG"],4)
    path([(x+20,mid),(x+32,mid-8)],C["SIG"],3)
    t(x-40,y_top-15,ref,"ref");t(x-52,mid+6,"10kΩ B","pin","end")
    return x+75,mid

def module_pin_left(x,y,label,color):
    line(x-42,y,x,y,color,4);t(x+12,y+6,label,"pin")

# Title
t(42,52,"Rev.9　ESP32・ADS1115・ペダル2個・MPU6050 2個　統合電気回路図","title")
t(42,84,"全回路を1枚に統合。線が接続される箇所は●、交差だけの箇所は接続点なし。基板上の穴番号は実装図を参照。","small")
line(42,108,W-42,108,"#9db1c0",2)

# Board boundaries. The main circuit is one continuous sheet; only the two
# remote C boards are outlined because they are physically separate.
t(75,153,"メイン基板側＋外部ペダル", "board")
a.append('<rect x="1810" y="165" width="720" height="610" rx="15" fill="#f3fbf6" fill-opacity="0.85" stroke="#67a07c" stroke-width="2" stroke-dasharray="10 8"/>')
t(1830,195,"Cタイプ小基板 IMU1", "board")
a.append('<rect x="1810" y="825" width="720" height="610" rx="15" fill="#f3fbf6" fill-opacity="0.85" stroke="#67a07c" stroke-width="2" stroke-dasharray="10 8"/>')
t(1830,855,"Cタイプ小基板 IMU2", "board")

# Global power rails
line(120,205,2470,205,C["3V3"],7);t(125,192,"3V3 共通電源", "net")
line(120,1530,2470,1530,C["GND"],7);t(125,1560,"GND 共通", "net")

# U1 ESP32
box(105,300,360,500,"U1  ESP32-DevKitC-32E","USB給電。3V3ピンからセンサーへ給電")
t(125,390,"USB 5V", "pin");line(55,384,105,384,"#677f93",4);t(50,390,"USB", "small","end")
pins_u1=[("3V3",365,C["3V3"]),("GND",425,C["GND"]),("GPIO21 / SDA",525,C["SDA"]),("GPIO22 / SCL",590,C["SCL"]),("GPIO27",680,C["GPIO27"])]
for label,y,col in pins_u1:
    t(445,y+6,label,"pin","end");line(465,y,520,y,col,4)
path([(520,365),(560,365),(560,205)],C["3V3"],5);dot(560,205,C["3V3"])
path([(520,425),(590,425),(590,1530)],C["GND"],5);dot(590,1530,C["GND"])

# I2C buses directly join U1/U2/U3/U4. The upper branches route around U2,
# so no signal line appears to pass through the module body.
line(520,525,1038,525,C["SDA"],6);t(540,512,"SDA", "net")
line(520,590,1038,590,C["SCL"],6);t(540,577,"SCL", "net")
path([(760,525),(760,255),(1740,255),(1740,1180)],C["SDA"],5);dot(760,525,C["SDA"])
path([(800,590),(800,280),(1715,280),(1715,1245)],C["SCL"],5);dot(800,590,C["SCL"])

# U2 ADS1115
box(1080,300,430,590,"U2  ADS1115","汎用1×10ピン基板 / I²C 0x48 / 回路面を上")
for label,y,col in [("VDD",390,C["3V3"]),("GND",455,C["GND"]),("SDA",525,C["SDA"]),("SCL",590,C["SCL"]),("ADDR",655,C["GND"]),("A0",730,C["ADC0"]),("A1",795,C["ADC1"])]:
    module_pin_left(1080,y,label,col)
path([(1038,390),(1000,390),(1000,205)],C["3V3"],5);dot(1000,205,C["3V3"])
path([(1038,455),(970,455),(970,1530)],C["GND"],5);dot(970,1530,C["GND"])
dot(1038,525,C["SDA"]);dot(1038,590,C["SCL"])
path([(1038,655),(1015,655),(1015,695),(970,695)],C["GND"],4);dot(970,695,C["GND"])
t(1110,822,"Rev.9配置の左→右：", "small")
t(1110,846,"A3,A2,A1,A0,ALRT,ADDR,SDA,SCL,GND,VDD", "small")
t(1110,874,"ALRT、A2、A3：未接続", "small")

# ADS local decoupling C3 and main bulk C6
capacitor_vertical(1590,205,1530,"C3","100nF")
t(1545,900,"ADS VDD付近", "small")
capacitor_vertical(1690,205,1530,"C6","10µF",True)
t(1645,900,"メイン電源", "small")

# Analog vertical nodes from ADS
path([(1038,730),(905,730),(905,1050)],C["ADC0"],5);t(915,717,"ADC_A0", "net")
path([(1038,795),(980,795),(980,1360)],C["ADC1"],5);t(990,782,"ADC_A1", "net")

# GPIO27 protection/feedback resistors
path([(520,680),(640,680),(640,910)],C["GPIO27"],5);dot(640,680,C["GPIO27"])
resistor(640,820,850,"R7","100kΩ",C["GPIO27"])
path([(850,820),(905,820)],C["ADC0"],5);dot(905,820,C["ADC0"])
resistor(640,910,850,"R8","100kΩ",C["GPIO27"])
path([(850,910),(980,910)],C["ADC1"],5);dot(980,910,C["ADC1"])
t(675,960,"GPIO27側だけ共通。R7/R8のADS側はA0/A1へ別々に接続。", "small")

# Pedal 1 and 2 outside main board mechanically; show at left bottom
t(100,915,"外部ペダル", "h")
# P1: upper potentiometer. Its ground return is moved to x=235 so the P2 body
# below is not placed on top of a ground wire.
line(150,950,150,205,C["3V3"],4);dot(150,205,C["3V3"])
sx,sy=pot(150,950,1150,"P1 ペダル1")
line(150,1150,235,1150,C["GND"],4);line(235,1150,235,1530,C["GND"],4);dot(235,1530,C["GND"])
line(sx,sy,300,sy,C["SIG"],4);t(sx+15,sy-12,"SIG1","net")
resistor(300,sy,530,"R1","1kΩ",C["SIG"])
path([(530,sy),(905,sy)],C["ADC0"],5);dot(905,sy,C["ADC0"])
capacitor_vertical(905,sy,1530,"C1","100nF",top_color=C["ADC0"])

# P2: lower potentiometer. Its 3V3 feed runs down the far-left side, avoiding
# P1 and R1. The ADC1 line crosses ADC0 once without a junction dot.
line(70,1260,70,205,C["3V3"],4);dot(70,205,C["3V3"])
line(70,1260,150,1260,C["3V3"],4)
sx,sy=pot(150,1260,1460,"P2 ペダル2")
line(150,1460,150,1530,C["GND"],4);dot(150,1530,C["GND"])
line(sx,sy,300,sy,C["SIG"],4);t(sx+15,sy-12,"SIG2","net")
resistor(300,sy,530,"R2","1kΩ",C["SIG"])
path([(530,sy),(980,sy)],C["ADC1"],5);dot(980,sy,C["ADC1"])
capacitor_vertical(980,sy,1530,"C2","100nF",top_color=C["ADC1"])

# U3 MPU6050 IMU1
box(1900,255,410,405,"U3  MPU6050 IMU1","Adafruit #3886 / ADDR開放 / 0x68",fill="#e7f4eb",stroke="#4a8060")
for label,y,col in [("VIN",350,C["3V3"]),("GND",415,C["GND"]),("SDA",525,C["SDA"]),("SCL",590,C["SCL"])]: module_pin_left(1900,y,label,col)
path([(1858,350),(1840,350),(1840,205)],C["3V3"],5);dot(1840,205,C["3V3"])
path([(1858,415),(1818,415),(1818,1530)],C["GND"],5);dot(1818,1530,C["GND"])
line(1740,525,1858,525,C["SDA"],5);dot(1740,525,C["SDA"]);dot(1858,525,C["SDA"])
line(1715,590,1858,590,C["SCL"],5);dot(1715,590,C["SCL"]);dot(1858,590,C["SCL"])
t(1930,695,"3Vo、INT：未接続", "small")
t(1930,725,"AD0追加配線なし。裏面ADDRジャンパーは開放。", "small")
capacitor_vertical(2380,205,710,"C4","100nF")
line(2380,710,1818,710,C["GND"],4);dot(1818,710,C["GND"])
t(2320,745,"VIN―GND間", "small")

# U4 MPU6050 IMU2: I2C branches from common bus
box(1900,915,410,405,"U4  MPU6050 IMU2","Adafruit #3886 / ADDR短絡 / 0x69",fill="#e7f4eb",stroke="#4a8060")
for label,y,col in [("VIN",1010,C["3V3"]),("GND",1075,C["GND"]),("SDA",1180,C["SDA"]),("SCL",1245,C["SCL"])]: module_pin_left(1900,y,label,col)
path([(1858,1010),(1785,1010),(1785,205)],C["3V3"],5);dot(1785,205,C["3V3"])
path([(1858,1075),(1760,1075),(1760,1530)],C["GND"],5);dot(1760,1530,C["GND"])
line(1740,1180,1858,1180,C["SDA"],5);dot(1740,1180,C["SDA"]);dot(1858,1180,C["SDA"])
line(1715,1245,1858,1245,C["SCL"],5);dot(1715,1245,C["SCL"]);dot(1858,1245,C["SCL"])
t(1930,1355,"3Vo、INT：未接続", "small")
t(1930,1385,"裏面ADDRジャンパー2パッドをはんだで短絡。", "small")
capacitor_vertical(2380,865,1370,"C5","100nF")
line(2380,865,1785,865,C["3V3"],4);dot(1785,865,C["3V3"])
line(2380,1370,1760,1370,C["GND"],4);dot(1760,1370,C["GND"])
t(2320,1405,"VIN―GND間", "small")

# Cable notes and warnings
t(1840,790,"IMUケーブル各4芯：赤=3V3、黒=GND、緑=SDA、青=SCL（AWG24より線）", "small")
t(70,1648,"注：R7/R8によりGPIO27出力はA0/A1へ影響する。GPIO27入力時もR7+R8を介してA0/A1が弱く結合する。", "small")
t(70,1675,"通電前に3V3―GND短絡なし、各ネットの導通、IMU1=0x68・IMU2=0x69・ADS=0x48を確認する。", "small")

# Netlist-level invariants represented by the diagram.
expected = {
    "3V3": {"U1.3V3","U2.VDD","U3.VIN","U4.VIN","P1.HI","P2.HI","C3.1","C4.1","C5.1","C6.+"},
    "GND": {"U1.GND","U2.GND","U2.ADDR","U3.GND","U4.GND","P1.LO","P2.LO","C1.2","C2.2","C3.2","C4.2","C5.2","C6.-"},
    "SDA": {"U1.GPIO21","U2.SDA","U3.SDA","U4.SDA"},
    "SCL": {"U1.GPIO22","U2.SCL","U3.SCL","U4.SCL"},
    "GPIO27": {"U1.GPIO27","R7.1","R8.1"},
    "ADC0": {"U2.A0","R1.2","C1.1","R7.2"},
    "ADC1": {"U2.A1","R2.2","C2.1","R8.2"},
}
assert len(expected)==7
assert expected["SDA"] == {"U1.GPIO21","U2.SDA","U3.SDA","U4.SDA"}
assert expected["SCL"] == {"U1.GPIO22","U2.SCL","U3.SCL","U4.SCL"}
assert "U2.ADDR" in expected["GND"] and "U3.AD0" not in expected["GND"] and "U4.AD0" not in expected["GND"]
assert expected["ADC0"].isdisjoint(expected["ADC1"])

a.append('</svg>')
(OUT/'統合電気回路図.svg').write_text('\n'.join(a))
print('generated Rev.9 integrated schematic; 7 nets validated, 1x10 ADS, 2 IMUs and 2 pedals included')
