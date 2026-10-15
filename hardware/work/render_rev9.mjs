import fs from 'node:fs/promises';
import sharp from 'sharp';
for(const name of ['部品配置','裏面① 電源・GND','裏面② I²C','裏面③ ペダル・ADC・GPIO27','裏面④ 全配線（①＋②＋③）','統合電気回路図']){
 const file=`hardware/Rev9/${name}`;
 await sharp(`${file}.svg`,{density:144}).png().toFile(`${file}.png`);
 console.log(name);
}
