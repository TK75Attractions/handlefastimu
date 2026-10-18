import fs from 'node:fs/promises';
import {FileBlob,SpreadsheetFile} from '@oai/artifact-tool';

const src='/Users/kenichirowada/Documents/Codex/2026-09-13/esp32/outputs/Rev7';
const dir='hardware/Rev8';
const name=(await fs.readdir(src)).find(n=>n.includes('AutoRecovered')&&n.endsWith('.xlsx'));
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(`${src}/${name}`));
const s=wb.worksheets.getItem('工程表'),c=wb.worksheets.getItem('導通チェック');
const originalProgress=s.getRange('L9:N67').values;
const originalChecks=c.getRange('H6:J35').values;
const originalFormulas=s.getRange('A1:N67').formulas;

// Update version/path text only; preserve all formulas and other cell types.
for(const [sh,range] of [[s,'A1:N67'],[c,'A1:J35']]){
 const values=sh.getRange(range).values,formulas=sh.getRange(range).formulas;
 for(let r=0;r<values.length;r++)for(let col=0;col<values[r].length;col++){
  const v=values[r][col];if(typeof v!=='string'||formulas[r]?.[col])continue;
  const nv=v.replaceAll('Rev.7','Rev.8').replaceAll('outputs/Rev7/','hardware/Rev8/');
  if(nv!==v)sh.getCell(r,col).values=[[nv]];
 }
}
s.getRange('A3').values=[['Rev.8：Bタイプ基板の電源分岐を分散。3V3行20・GND行22、C6移設、I²C分配線を被覆線へ変更。入力済み進捗を引継ぎ、変更対象だけ再作業。']];
const changes={
 9:{D:'裸線と被覆線を準備',E:'横レール2本とGND縦線4本は0.5mmスズメッキ線。縦線は穴中心間5.08mm。残り31本はAWG24被覆線。図の経路より少し長く切り、両端だけ剥く。',J:'線材が図と一致。芯線に傷なし。被覆線の途中は裸にしない'},
 18:{E:'C6は＋(19,20)、−(19,22)。脚を5.08mm間隔に成形し、横レールへ直接接続する。旧(16,20)/(16,21)のC6を残さない。切った脚が届かなければ新品へ交換。',J:'＋/−が図と一致。レール間をリード線で短絡しない'},
 20:{D:'電源横レール2本を敷設',E:'3V3は行20・列14〜30、GNDは行22・列15〜32に0.5mm裸線を敷く。間隔5.08mm。配線表の16点だけをはんだ付け。行21に横向きの裸線を置かない。I²C分配線は工程23で付ける。',F:'裏面① 電源・GND.svg、旧版からの付け替え手順.md',J:'2レールが5.08mm離れ、指定16点で導通。相互非短絡'},
 21:{D:'電源分岐とGND縦線を配線',E:'ESP電源は3V3=(15,20)、GND=(22,22)。ADS/C3からは3V3=(22,20)、GND=(16,22)。端子GND4本は裸線で行22へ、端子3V3は被覆線で行20へ。交差部分は被覆を残す。',H:'AWG24被覆線、0.5mmスズメッキ線、こて',J:'レール各穴はレール＋分岐1本。3V3/GND共通化・相互非短絡'},
 22:{E:'USB、全モジュール、外部ケーブルを外す。3V3全点とGND全点の導通を確認。3V3―GNDは抵抗値の充電変化を待ち、低抵抗に固定されないことを確認する。付け替え後は再測定。'},
 23:{E:'SDA行18・列17〜23とSCL行19・列18〜24の横分配線もAWG24被覆線にする。ESP GPIO21/22、ADS SDA/SCL、MPU1/2信号端子を同名点へつなぐ。旧裸I²Cレールは外す。',J:'I²C全10本が被覆線。両端だけ裸にし、交差で短絡なし'},
 25:{E:'P1 SIG(27,24)は行23と24の中間を通し、R1→A0へ。P2 SIG→R2→A1。C1 GNDは(29,22)、C2 GNDは(27,22)。途中の曲がり位置23.5ははんだ穴ではない。'},
 27:{E:'裏面④全配線図と37本を照合。裸線6本・被覆線31本、C6極性、行20/22レール、端子台前方15mmを確認。導通・絶縁の結果を記録する。',J:'全37本と絶縁項目が合格。旧行21の裸GNDレールなし'},
};
for(const [no,fields] of Object.entries(changes))for(const [col,text] of Object.entries(fields))s.getRange(`${col}${Number(no)+8}`).values=[[text]];
const rework=[18,20,21,22,23,24];
for(const no of rework){
 const r=no+8;
 if(s.getRange(`L${r}`).values[0][0]==='完了')s.getRange(`L${r}`).values=[['再作業']];
 const memo=s.getRange(`N${r}`).values[0][0];
 s.getRange(`N${r}`).values=[[`${memo?memo+'／':''}Rev.8変更対象：付け替えまたは再検査後に完了へ更新`]];
}
// Fit only the changed long descriptions; leave all other layout intact.
for(const no of Object.keys(changes))s.getRange(`A${Number(no)+8}:N${Number(no)+8}`).format.rowHeight=82;

wb.recalculate();
const error=await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:100},maxChars:1200});
console.log(error.ndjson);
console.log((await wb.inspect({kind:'table',range:'工程表!A4:N4',include:'values,formulas',maxChars:2200,tableMaxCols:14})).ndjson);
const states=s.getRange('L9:N67').values;
for(let i=0;i<states.length;i++){
 if(!rework.includes(i+1)&&states[i][0]!==originalProgress[i][0])throw Error(`Unexpected progress change ${i+1}`);
 if(states[i][1]!==originalProgress[i][1])throw Error(`Date changed ${i+1}`);
 if(!rework.includes(i+1)&&states[i][2]!==originalProgress[i][2])throw Error(`Memo changed ${i+1}`);
}
if(JSON.stringify(c.getRange('H6:J35').values)!==JSON.stringify(originalChecks))throw Error('Check inputs changed');
if(JSON.stringify(s.getRange('A1:N67').formulas)!==JSON.stringify(originalFormulas))throw Error('Formula pattern changed');
for(const [range,file] of [['C25:K33','schedule_after'],['L24:N35','progress_after']]){
 const im=await wb.render({sheetName:'工程表',range,scale:1,format:'png'});
 await fs.writeFile(`hardware/work/${file}.png`,new Uint8Array(await im.arrayBuffer()));
}
const im=await wb.render({sheetName:'導通チェック',range:'A6:J13',scale:1,format:'png'});
await fs.writeFile('hardware/work/checks_after.png',new Uint8Array(await im.arrayBuffer()));
await(await SpreadsheetFile.exportXlsx(wb)).save(`${dir}/組立・はんだ付け工程表.xlsx`);

const rows=s.getRange('A9:N67').values;
const md=['# Rev.8 組立・はんだ付け工程表','','入力済み工程表の進捗を引き継いだ。変更対象工程18/20/21/22/23/24は、完了済みの場合「再作業」としている。付け替えは「旧版からの付け替え手順.md」に従う。','',
 '|No.|工程|対象|作業|手順|予定分|前提|合格条件|状態|メモ|','|---:|---|---|---|---|---:|---|---|---|---|'];
for(const r of rows)md.push('|'+[r[0],r[1],r[2],r[3],r[4],r[6],r[8],r[9],r[11],r[13]].map(v=>String(v??'').replaceAll('|','／')).join('|')+'|');
await fs.writeFile(`${dir}/組立・はんだ付け工程表.md`,md.join('\n'));
console.log(JSON.stringify({preservedInputs:true,reworkSteps:rework,completed:states.filter(r=>r[0]==='完了').length,rework:states.filter(r=>r[0]==='再作業').length,source:name}));
