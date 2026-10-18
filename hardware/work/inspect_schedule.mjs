import fs from 'node:fs/promises';
import {FileBlob, SpreadsheetFile} from '@oai/artifact-tool';
const dir='/Users/kenichirowada/Documents/Codex/2026-09-13/esp32/outputs/Rev7';
const names=await fs.readdir(dir);
const file=names.find(n=>n.includes('AutoRecovered')&&n.endsWith('.xlsx'));
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(`${dir}/${file}`));
console.log((await wb.inspect({kind:'workbook,sheet,table',maxChars:2500,tableMaxRows:2,tableMaxCols:4})).ndjson);
console.log((await wb.inspect({kind:'table',range:'工程表!L24:N35',include:'values',maxChars:2500,tableMaxRows:12,tableMaxCols:3})).ndjson);
const out='hardware/work';
for(const [sheet,range,name] of [['工程表','C25:K33','schedule_before'],['導通チェック','A6:J13','checks_before']]){
 const img=await wb.render({sheetName:sheet,range,scale:1,format:'png'});
 await fs.writeFile(`${out}/${name}.png`,new Uint8Array(await img.arrayBuffer()));
}
console.log('source preview rendered');
