from pathlib import Path
import xml.etree.ElementTree as ET

ET.register_namespace('', 'http://www.w3.org/2000/svg')
p=Path('hardware/Rev8')
root=ET.parse(p/'裏面① 電源・GND.svg').getroot()
ns='{http://www.w3.org/2000/svg}'
root.set('width','1620');root.set('height','720');root.set('viewBox','110 535 540 240')
# Crop the generated power drawing, preserving its exact paths and junctions.
ET.SubElement(root,ns+'rect',{'x':'110','y':'535','width':'540','height':'42','fill':'white'})
def txt(x,y,text,size='12'):
 e=ET.SubElement(root,ns+'text',{'x':str(x),'y':str(y),'font-size':size,'style':f'font-size:{size}px;fill:#20364c'})
 e.text=text
txt(114,548,'Rev.8 電源部分の拡大：裏面（座標は部品面基準）','12')
for c in range(12,34):
 x=80+(95-(6.86+(c-1)*2.54))*8
 txt(x-5,571,str(c),'11')
for r in range(19,25):
 y=170+(6.79+(r-1)*2.54)*8
 txt(113,y+4,str(r),'11')
ET.SubElement(root,ns+'rect',{'x':'110','y':'720','width':'540','height':'55','fill':'white'})
txt(114,739,'太い実線＝0.5mmスズメッキ線／細い破線＝AWG24被覆線','12')
txt(114,760,'紫の記号＝C6（表側の部品）。＋(19,20)、−(19,22)。','12')
ET.ElementTree(root).write(p/'電源部分の拡大.svg',encoding='utf-8',xml_declaration=True)
