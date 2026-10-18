import fs from 'node:fs/promises';
import {FileBlob,SpreadsheetFile} from '@oai/artifact-tool';

const src='hardware/Rev8/組立・はんだ付け工程表.xlsx',dir='hardware/Rev9';
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(src));
const s=wb.worksheets.getItem('工程表'),c=wb.worksheets.getItem('導通チェック');
if(process.argv.includes('--inspect')){
 for(const [sheetName,range,file] of [['工程表','C17:K24','rev9_source'],['導通チェック','A6:J10','rev9_source_checks']]){
  const im=await wb.render({sheetName,range,scale:1,format:'png'});
  await fs.writeFile(`hardware/work/${file}.png`,new Uint8Array(await im.arrayBuffer()));
 }
 console.log(JSON.stringify(s.getRange('A9:N35').values.map(r=>({no:r[0],task:r[3],description:r[4],status:r[11]}))));
 console.log(JSON.stringify(c.getRange('A6:J35').values));
 process.exit(0);
}
const originalProgress=s.getRange('L9:N67').values,originalChecks=c.getRange('H6:J35').values;
const originalFormulas=s.getRange('A1:N67').formulas;
const model=JSON.parse(await fs.readFile(`${dir}/部品座標.json`,'utf8'));
const edges=JSON.parse(await fs.readFile(`${dir}/配線座標.json`,'utf8'));
const pt=p=>`(${p.join(',')})`;
const part=ref=>{const p=model.parts.find(p=>p.ref===ref);return `${ref} ${p.value}：${pt(p.p)} ${p.net_p}―${pt(p.q)} ${p.net_q}`;};
const rework=[5,9,10,11,12,13,15,16,18,19,20,21,22,23,24,25,26,27,42,49];
for(const [sh,range] of [[s,'A1:N67'],[c,'A1:J35']]){
 const values=sh.getRange(range).values,formulas=sh.getRange(range).formulas;
 for(let r=0;r<values.length;r++)for(let col=0;col<values[r].length;col++){
  const v=values[r][col];if(typeof v!=='string'||formulas[r]?.[col])continue;
  const nv=v.replaceAll('Rev.8','Rev.9').replaceAll('hardware/Rev8/','hardware/Rev9/');
  if(nv!==v)sh.getCell(r,col).values=[[nv]];
 }
}
s.getRange('A3').values=[['Rev.9：ADSを回路面上のまま180°回転。ESP下端に電源、中央にI²C、右側にペダル回路を分散。変更対象の完了工程だけ再作業へ。']];
const changes={
 1:{E:'Rev.9の配置図・裏面①〜④・全配線表を揃える。既存基板の改造は「旧版からの付け替え手順.md」を先に読む。USBとケーブルを外す。'},
 5:{D:'モジュール・端子台を仮置き',E:'ESPソケット列3/13・行3〜21を維持。ADSは回路面上で180°回転し、行2・列20〜29へ。左A3→右VDD。端子台は行24・外向き。C6とUSB前方、ねじの空間を確認。',J:'基板外形とピンが実物と一致。USBプラグが入り、端子台前方15mm以上空く'},
 9:{D:'裸線と被覆線を準備',E:'0.5mmスズメッキ線10本とAWG24被覆線33本を用意する。経路に沿って少し余裕を持たせる。隣接ランド付近では両端の約1〜1.5mmだけ剥き、途中の被覆は残す。',J:'43本の線材が全配線表と一致。芯線・被覆に傷なし'},
 10:{E:'部品面基準（列,行）で位置を印す。裏面は左右反転。電源行22/24・列1〜13、I²C列16/19、ADS行2、抵抗行12/16、C1/C2行14、C3列33、C6列1。旧版の印と区別する。'},
 11:{E:`${part('R1')}。${part('R2')}。脚間10.16mmに成形し、本体を部品面へ置く。各2点をはんだ付けして余分な脚を切る。`,J:'抵抗の脚はそれぞれ別穴。隣のランドへはんだが流れていない'},
 12:{E:`${part('R7')}。${part('R8')}。脚間10.16mm。抵抗の両端を直結しない。`,J:'100kΩがそれぞれGPIO27とADC0/ADC1の間に入る'},
 13:{E:`${part('C1')}。${part('C2')}。${part('C3')}。抵抗の脚と穴を共有しない。脚間5.08mm。`,J:'3個とも100nF。ADC側/GND側が一致し、全脚が別穴'},
 15:{D:'ADSピンとソケットを仮固定',E:'ADSに1×10オスピンを垂直にはんだ付け（既存ピンは再使用）。メスソケットは行2・列20〜29。回路面上で180°回転、左A3,A2,A1,A0,ALRT,ADDR,SDA,SCL,GND,VDD。端の1点を仮止め。',J:'右端(29,2)がVDD。基板を裏返して取り付けていない'},
 16:{E:'ESP/ADSモジュールを抜く。ESP左右ソケットの未接合点と、行2のADSソケット残り9点をはんだ付けし、仮止めを仕上げる。隣接ランドのブリッジを確認。行9の旧ADSソケットは残さない。'},
 18:{E:`${part('C6')}。＋が(1,22)、−が(1,24)。脚間5.08mmに成形する。外径5mm以下の想定で仮合わせし、極性確認後にはんだ付けする。`,J:'極性が正しく、USBプラグ・取付ねじと干渉しない'},
 19:{E:'低い部品とソケット・端子台を目視確認。抵抗・コンデンサーの全16脚が別穴。裏面のはんだ山を整え、隣接ランドの短絡を除く。モジュールは外したまま。'},
 20:{D:'裏面①の電源裸線5本を敷設',E:'全配線表No.1/2/8/9/10を付ける。左下3V3=行22・GND=行24（列1〜13、間隔5.08mm）。ADS付近の局所電源とP2 GNDも指定穴のみ固定。I²C/ADC/GPIO27の裸線は後工程。',F:'裏面① 電源・GND.svg、実装手順と全配線表.md',J:'両端と指定中間点が導通。通過する未使用ランドまで接続していない'},
 21:{D:'電源の被覆線16本を配線',E:'裏面①No.11〜18/23〜30を順に付ける。ESP下も使い、電源分岐をレールの専用穴へ分散する。3V3/GND裸線の交差は被覆線を約1mm浮かせる。曲がり角でははんだ付けしない。',H:'AWG24被覆線、こて、全配線表',J:'16本の両端が表と一致。被覆がソケット脚や裸線で傷んでいない'},
 22:{E:'USB・全モジュール・外部ケーブルを外す。新しい3V3全点とGND全点の導通を測る。3V3―GNDは抵抗値の充電変化を待ち、低抵抗に固定されないことを確認。付け替え後は必ず再測定。'},
 23:{D:'裏面②のI²Cを配線',E:'SDA裸線=列16・行8〜20、SCL裸線=列19・行5〜20。中心間7.62mm。No.3/4と被覆線No.19〜22/31〜34を順に付ける。ESP GPIO21/22、ADS、MPU1/2端子を同名点へ。',H:'0.5mmスズメッキ線、AWG24線、こて',J:'裸線2本＋被覆線8本。SDA/SCLがそれぞれ導通し、相互非短絡'},
 24:{E:'モジュールを外し、GPIO21―SDA端子―ADS SDA、GPIO22―SCL端子―ADS SCLの導通を確認。SDA/SCLと3V3/GNDの短絡がないことを測る。'},
 25:{D:'ペダル入力とADCを配線',E:'裏面③No.5/6とNo.35〜40を付ける。ADC0裸線=列26・行12〜17、ADC1=列32・行12〜17。P1 SIG→R1→A0、P2 SIG→R2→A1。C1/C2のGNDも同図へ。',H:'0.5mmスズメッキ線、AWG24線、こて',J:'R1/R2を迂回する裸線なし。ADC0/ADC1の相互短絡なし'},
 26:{D:'GPIO27と100kΩを配線',E:'No.7裸線=行21・列22〜29。被覆線No.41〜43でESP GPIO27=(3,13)、R7=(22,16)、R8=(28,16)へ。R7/R8のADC側は別ネット。',H:'0.5mmスズメッキ線、AWG24線、こて',J:'GPIO27→R7/R8→ADCの間に100kΩ。GPIO27をA0/A1へ直接つながない'},
 27:{E:`裏面④と全${edges.length}本を照合（裸線10・被覆線33）。C6極性、ADS行2のピン順、抵抗両端、端子台外向き、USB前方の空間を確認。全導通チェックに記録する。`,J:'43本の経路と指定はんだ点が一致し、導通・絶縁項目が合格'},
 42:{E:'C6極性、ESP/MPU向き、ADSの回路面上・上側行2の右端(29,2)=VDDを確認。端子台6個が外向き、前方15mm空間、USBプラグ空間、MPU2ジャンパー、金属片の残留も確認する。'},
 49:{E:'電源を切り、ADSを回路面上・180°回転・上側行2の右端(29,2)=VDDで挿す。再通電し、ADS VDD電圧とI²Cスキャン0x48を確認する。'},
};
for(const [no,fields] of Object.entries(changes)){
 const r=Number(no)+8;
 for(const [col,text] of Object.entries(fields))s.getRange(`${col}${r}`).values=[[text]];
 s.getRange(`A${r}:N${r}`).format.rowHeight=92;
}
for(const no of rework){
 const r=no+8;
 if(s.getRange(`L${r}`).values[0][0]==='完了')s.getRange(`L${r}`).values=[['再作業']];
 const memo=originalProgress[no-1][2];
 s.getRange(`N${r}`).values=[[`${memo?memo+'／':''}Rev.9配置変更：付け替え・再検査後に完了へ`]];
}
c.getRange('D6').values=[['ADSの回路面・上側ピン列']];
c.getRange('E6').values=[['行2、左(20,2) A3→右(29,2) VDD']];
c.getRange('F6').values=[['回路面上のまま180°回転。ピン順と実物一致']];
c.getRange('E7').values=[['全6端子台の前方・USBプラグ・C6周辺']];
c.getRange('F7').values=[['端子台は外向き・前方15mm以上。USBとC6が干渉しない']];
c.getRange('A6:J7').format.rowHeight=68;
const invalidChecks=[];
for(let i=0;i<30;i++){
 const type=String(c.getRange(`A${i+6}`).values[0][0]);
 if(type.startsWith('M')||type.startsWith('P')||type.startsWith('I')||type.startsWith('A')){
  const status=String(originalChecks[i][0]??'');
  if(['合格','OK','完了','済'].includes(status)){
   c.getRange(`H${i+6}`).values=[['未確認']];
   const memo=originalChecks[i][2];
   c.getRange(`J${i+6}`).values=[[`${memo?memo+'／':''}Rev.9で経路変更。旧測定値を残し、再測定が必要`]];
   invalidChecks.push(i);
  }
 }
}
const states=s.getRange('L9:N67').values;
for(let i=0;i<states.length;i++){
 if(!rework.includes(i+1)&&JSON.stringify(states[i])!==JSON.stringify(originalProgress[i]))throw Error(`Progress changed ${i+1}`);
 if(states[i][1]!==originalProgress[i][1])throw Error(`Date changed ${i+1}`);
}
for(let i=0;i<30;i++){
 const row=c.getRange(`H${i+6}:J${i+6}`).values[0];
 if(!invalidChecks.includes(i)&&JSON.stringify(row)!==JSON.stringify(originalChecks[i]))throw Error(`Check changed ${i}`);
 if(row[1]!==originalChecks[i][1])throw Error(`Measurement changed ${i}`);
}
if(JSON.stringify(s.getRange('A1:N67').formulas)!==JSON.stringify(originalFormulas))throw Error('Formula pattern changed');
wb.recalculate();
const baseline=Number(s.getRange('H4').values[0][0]);
const testRow=states.findIndex(r=>r[0]==='未着手')+9;
const state=s.getRange(`L${testRow}`).values;
s.getRange(`L${testRow}`).values=[['完了']];wb.recalculate();
if(Number(s.getRange('H4').values[0][0])!==baseline+1)throw Error('Count formula did not recalculate');
s.getRange(`L${testRow}`).values=state;wb.recalculate();
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:100},maxChars:1200})).ndjson);
console.log((await wb.inspect({kind:'table',range:'工程表!A4:N4',include:'values,formulas',maxChars:2000,tableMaxCols:14})).ndjson);
for(const [sheetName,range,file] of [
 ['工程表','C17:K24','rev9_parts'],['工程表','C28:K35','rev9_wiring'],['工程表','L17:N35','rev9_progress'],['工程表','D50:K57','rev9_final_orientation'],['導通チェック','A6:J10','rev9_checks']]){
 const im=await wb.render({sheetName,range,scale:1,format:'png'});
 await fs.writeFile(`hardware/work/${file}.png`,new Uint8Array(await im.arrayBuffer()));
}
await(await SpreadsheetFile.exportXlsx(wb)).save(`${dir}/組立・はんだ付け工程表.xlsx`);
const rows=s.getRange('A9:N67').values;
const md=['# Rev.9 組立・はんだ付け工程表','','Rev.8の入力済み進捗を引き継いだ。配置・配線変更対象の完了済み工程は「再作業」としている。日付と元のメモを保持。旧基板の移設は「旧版からの付け替え手順.md」に従う。','',
 '|No.|工程|対象|作業|手順|予定分|前提|合格条件|状態|メモ|','|---:|---|---|---|---|---:|---|---|---|---|'];
for(const r of rows)md.push('|'+[r[0],r[1],r[2],r[3],r[4],r[6],r[8],r[9],r[11],r[13]].map(v=>String(v??'').replaceAll('|','／')).join('|')+'|');
await fs.writeFile(`${dir}/組立・はんだ付け工程表.md`,md.join('\n')+'\n');
console.log(JSON.stringify({source:src,sourceProgressRetained:true,reworkSteps:rework,completed:states.filter(r=>r[0]==='完了').length,rework:states.filter(r=>r[0]==='再作業').length,invalidChecks}));
