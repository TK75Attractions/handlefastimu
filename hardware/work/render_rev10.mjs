import fs from 'node:fs/promises';
import sharp from 'sharp';
for(const dir of ['hardware/Rev10','hardware/Rev10/IMU_Cタイプ']){
 for(const name of await fs.readdir(dir)){
  if(!name.endsWith('.svg'))continue;
  await sharp(`${dir}/${name}`,{density:144}).png().toFile(`${dir}/${name.slice(0,-4)}.png`);
  console.log(name);
 }
}
