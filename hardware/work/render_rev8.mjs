import fs from 'node:fs/promises';
import sharp from 'sharp';

const dir='hardware/Rev8';
for (const name of ['部品配置','裏面① 電源・GND','裏面② I²C','裏面③ ペダル・ADC・GPIO27','裏面④ 全配線（①＋②＋③）','統合電気回路図','電源部分の拡大']) {
  const svg=`${dir}/${name}.svg`;
  try {
    await fs.access(svg);
    await sharp(svg,{density:144}).png().toFile(`${dir}/${name}.png`);
    console.log(`rendered ${name}.png`);
  } catch (e) {
    if (e?.code !== 'ENOENT') throw e;
  }
}
