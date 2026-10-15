import fs from 'node:fs/promises';
import {FileBlob,SpreadsheetFile} from '@oai/artifact-tool';
const src='hardware/Rev8/組立・はんだ付け工程表.xlsx',dir='hardware/Rev10';
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(src));
const s=wb.worksheets.getItem('工程表'),c=wb.worksheets.getItem('導通チェック');
if(process.argv.includes('--inspect')){
 for(const [sheetName,range,file] of [['工程表','C36:K44','rev10_source_imu'],['導通チェック','A24:J31','rev10_source_checks']]){
  const im=await wb.render({sheetName,range,scale:1,format:'png'});
  await fs.writeFile(`hardware/work/${file}.png`,new Uint8Array(await im.arrayBuffer()));
 }
 console.log((await wb.inspect({kind:'table',range:'工程表!A4:N4',include:'values,formulas',maxChars:1600,tableMaxCols:14})).ndjson);
 console.log(JSON.stringify({progress:s.getRange('L9:N67').values,checks:c.getRange('H6:J35').values}));
 process.exit(0);
}
const oldProgress=s.getRange('L9:N67').values,oldChecks=c.getRange('H6:J35').values;
const oldFormulas=s.getRange('A1:N67').formulas;
// Keep user-entered dates/memos unchanged during global version and pin-name updates.
for(const [sheet,range] of [[s,'A1:K67'],[c,'A1:G35']]){
 const values=sheet.getRange(range).values,formulas=sheet.getRange(range).formulas;
 for(let r=0;r<values.length;r++)for(let col=0;col<values[r].length;col++){
  const v=values[r][col];if(typeof v!=='string'||formulas[r]?.[col])continue;
  let nv=v.replaceAll('Rev.8','Rev.10').replaceAll('hardware/Rev8/','hardware/Rev10/').replaceAll('1×6P','1×8P').replaceAll('IMU_電気接続.svg','統合電気回路図.svg');
  if(nv!==v)sheet.getCell(r,col).values=[[nv]];
 }
}
s.getRange('A3').values=[['Rev.10：メインはRev.8を継承。写真の8ピンMPU6050に合わせ、C基板ソケット・C4/C5・配線・AD0設定を変更。日付・元メモを保持。']];
const changes={
 1:{E:'Rev.10のメイン配置図・裏面①〜④、統合回路図、IMU1/2のC基板図を揃える。メインの配置・配線はRev.8を継承。IMUは写真の8ピン版。変更点と部品差分を先に読む。'},
 2:{E:'従来部品に加え、MPU用1×8Pメスソケット×2、必要なら1×8Pオス×2を確認。C4/C5は100nF各1個。ねじ径・スペーサー高さは写真の実物へ合わせる。基板・端子台・4芯ケーブルは従来どおり。',J:'写真の8ピン順・実物ピッチを確認。6ピンソケットと取り違えなし'},
 6:{E:'写真のように回路面上、ピン列上側。ソケット行3・列8〜15、左INT/AD0/XCL/XDA/SDA/SCL/GND/VCC。C4/C5は(17,10)/(19,10)。端子台行14・列10/11,13/14を仮置き。',J:'VCC=(15,3)、GND=(14,3)。取付穴と端子前方に余裕あり'},
 7:{E:'写真のモジュールの下側2穴をC基板へ現物写しする。穴径とねじ径は実物で選ぶ。部品を外して穴加工し、バリ・切粉を除去。旧Adafruit寸法やM2.5適合を前提にしない。',H:'現物に合うドリル、ナイロンねじ、保護具',J:'実物に合うナイロンねじが通る。銅箔めくれ・亀裂・導体接触なし'},
 8:{E:'ESP用・ADS用1×10Pオス/メスと、MPU用1×8Pオス/メスを必要長へ切る。モジュールにオスピンが実装済みなら重複購入・交換は不要。全ピンが無理なく入ることを確認。'},
 9:{E:'メインは従来の裸線6本＋AWG24被覆線31本。IMU小基板は各9本＝0.5mm裸線3＋AWG24被覆線6を準備。交差は被覆を残し、隣接ランド付近の端は約1〜1.5mmだけ剥く。'},
 28:{D:'1×8Pピンヘッダーを取り付け',E:'未実装の場合だけ写真のMPUへ1×8オスピンを下向きに付ける。メスソケットを治具にして、端1本→反対端→残りの順。回路面上で左INT→右VCCを確認。',J:'8ピンが平行で、ソケットへ無理なく挿入可能'},
 29:{E:'C4/C5各100nFを(17,10)=3V3―(19,10)=GNDへ付ける。脚間5.08mm、無極性。低い部品なのでソケット・端子台より先に接合する。',J:'C4/C5が正しい穴にあり、両脚の直結・隣接ブリッジなし'},
 30:{D:'1×8Pソケットをはんだ付け',E:'ソケットは行3・列8〜15。MPUを無通電で挿し、写真の向きで回路面上、右端VCC=(15,3)を確認。両端を仮止め、モジュールを抜いて残りを接合。',J:'左からINT/AD0/XCL/XDA/SDA/SCL/GND/VCC。VCC・GND一致'},
 31:{E:'行14に電源2P=(10,14)3V3/(11,14)GND、信号2P=(13,14)SCL/(14,14)SDA。開口は下辺外向き。前方15mm以上空け、1脚仮止め→整列→残りを接合。'},
 32:{D:'IMU1の電源・I²Cを8本配線',E:'IMU1裏面No.2/5/8の裸線3本→No.1/3/4/6/7の被覆線5本を付ける。C4の両脚を電源へ。12.5/13.5行は曲がり位置で、はんだ穴ではない。AD0 No.9は工程34。',H:'0.5mmスズメッキ線、AWG24線、こて',J:'4ネットが一致。INT/XCL/XDAは未接続。交差では被覆線を浮かせる'},
 33:{D:'IMU2の電源・I²Cを8本配線',E:'IMU2もNo.2/5/8の裸線3本→No.1/3/4/6/7の被覆線5本。C5=(17,10)/(19,10)。No.9のAD0経路だけIMU1と異なるので工程34へ。',H:'0.5mmスズメッキ線、AWG24線、こて',J:'電源・I²CはIMU1と同じ4ネット。INT/XCL/XDAは未接続'},
 34:{C:'IMU1/2 C基板',D:'AD0を配線してアドレス設定',E:'小基板の被覆線No.9を付ける。両方AD0=(9,3)。IMU1はGND=(19,5)へ、IMU2は3V3=(17,6)へ。旧版の裏面ADDRジャンパー短絡手順は使わない。',H:'AWG24被覆線、こて、テスター',J:'IMU1 AD0―GNDが導通。IMU2 AD0―3V3が導通。逆側電源へ短絡なし',K:'電源を切り、No.9の接続先を見直す'},
 35:{E:'8ピンソケットへ挿し、写真の下側2穴を実物に合うナイロンねじ・スペーサーで固定する。高さはソケットへ合わせる。基板を反らせず、ピンに横荷重を掛けない。',H:'実物に合うナイロンねじ・スペーサー、ドライバー'},
 36:{E:'MPUを抜いて、電源端子→VCC/GND、信号端子→SCL/SDA、IMU1 AD0→GND、IMU2 AD0→3V3を測る。INT/XCL/XDA未接続、3V3―GNDとSDA―SCLの非短絡を確認。',J:'両基板9本を照合し、AD0も含め全項目合格'},
 42:{E:'C6極性、ESP/MPU向き、ADS回路面上かつ列20=VDD、端子台外向き・前方15mm、IMU1 AD0=GND、IMU2 AD0=3V3、金属片の残留を確認。'},
 46:{E:'メイン端子から各MPUソケットVCC/GND/SDA/SCLまで導通し、4線相互短絡なし。IMU1 AD0―GNDとIMU2 AD0―3V3も確認。モジュールは外したまま測る。'},
 50:{E:'電源を切り、IMU1を回路面上・右VCCで挿す。再通電し0x48/0x68を確認。VCCが3.3V付近、AD0がGND付近か測る。通信しない場合はモジュールのレギュレーター出力も確認。',F:'統合電気回路図.svg',J:'0x48/0x68を検出。VCC約3.3V、AD0 LOW',K:'電源を切り、4芯ケーブルとAD0―GND配線を確認'},
 51:{E:'電源を切りIMU2を接続。再通電し0x48/0x68/0x69を確認。IMU2 VCCとAD0が3V3付近か測る。同じアドレスしか出なければNo.9の配線を確認。',F:'統合電気回路図.svg',J:'3アドレスを同時検出。IMU2 AD0 HIGH',K:'0x68だけなら電源を切り、IMU2 AD0―3V3配線を確認'},
};
const affected=Object.keys(changes).map(Number).filter(n=>n!==1);
for(const [no,fields] of Object.entries(changes)){
 const r=Number(no)+8;
 for(const [col,value] of Object.entries(fields))s.getRange(`${col}${r}`).values=[[value]];
 s.getRange(`A${r}:N${r}`).format.rowHeight=84;
}
for(const no of affected){
 const r=no+8;
 if(oldProgress[no-1][0]==='完了'){
  s.getRange(`L${r}`).values=[['再作業']];
  const memo=oldProgress[no-1][2];s.getRange(`N${r}`).values=[[`${memo?memo+'／':''}Rev.10 IMU変更：再確認後に完了へ`]];
 }
}
for(const r of [24,28])c.getRange(`E${r}`).values=[['MPU VCCソケット=(15,3)']];
for(const r of [27]){
 c.getRange(`D${r}`).values=[['MPU INT/XCL/XDAソケット']];
 c.getRange(`E${r}`).values=[['端子台4線']];
 c.getRange(`F${r}`).values=[['すべて非導通']];
}
c.getRange('D31').values=[['IMU1 AD0(9,3)―GND']];
c.getRange('E31').values=[['IMU2 AD0(9,3)―3V3']];
c.getRange('F31').values=[['両方導通。反対側電源へ短絡なし']];
c.getRange('G31').values=[['両MPUをソケットから外す']];
for(const r of [24,27,28,31])c.getRange(`A${r}:J${r}`).format.rowHeight=64;
const checkRows=[24,25,26,27,28,29,30,31,34,35];
for(const r of checkRows){
 if(['合格','完了','OK','済'].includes(String(oldChecks[r-6][0]))){
  c.getRange(`H${r}`).values=[['未確認']];
  const memo=oldChecks[r-6][2];c.getRange(`J${r}`).values=[[`${memo?memo+'／':''}Rev.10 IMU変更。旧測定値を保持し再測定`]];
 }
}
const states=s.getRange('L9:N67').values;
for(let i=0;i<states.length;i++){
 if(!affected.includes(i+1)&&JSON.stringify(states[i])!==JSON.stringify(oldProgress[i]))throw Error(`Unexpected progress change ${i+1}`);
 if(states[i][1]!==oldProgress[i][1])throw Error(`Date changed ${i+1}`);
}
if(JSON.stringify(s.getRange('A1:N67').formulas)!==JSON.stringify(oldFormulas))throw Error('Formula pattern changed');
const allText=JSON.stringify(s.getRange('A1:K67').values)+JSON.stringify(c.getRange('A1:G35').values);
for(const stale of ['1×6P','3Vo','ADDRジャンパーを短絡','MPU1開放','MPU2短絡','ADDR開放','IMU2 ADDRジャンパー','VIN','IMU_電気接続.svg'])if(allText.includes(stale))throw Error(`Stale MPU text: ${stale}`);
wb.recalculate();
const complete=states.filter(r=>r[0]==='完了').length;
if(Number(s.getRange('H4').values[0][0])!==complete)throw Error('Count mismatch');
const ri=states.findIndex(r=>r[0]==='未着手')+9,save=s.getRange(`L${ri}`).values;
s.getRange(`L${ri}`).values=[['完了']];wb.recalculate();
if(Number(s.getRange('H4').values[0][0])!==complete+1)throw Error('Count did not react');
s.getRange(`L${ri}`).values=save;wb.recalculate();
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:100},maxChars:1500})).ndjson);
for(const [sheetName,range,file] of [['工程表','C36:K44','rev10_imu'],['工程表','C14:K17','rev10_preparation'],['工程表','C58:K59','rev10_poweron'],['導通チェック','A24:J31','rev10_checks']]){
 const im=await wb.render({sheetName,range,scale:1,format:'png'});
 await fs.writeFile(`hardware/work/${file}.png`,new Uint8Array(await im.arrayBuffer()));
}
await(await SpreadsheetFile.exportXlsx(wb)).save(`${dir}/組立・はんだ付け工程表.xlsx`);
const md=['# Rev.10 組立・はんだ付け工程表','','Rev.8の進捗・日付・メモを引き継ぐ。IMU変更対象の完了済み工程だけ再作業に設定。メインの部品・配線はRev.8を維持。','',
 '|No.|工程|対象|作業|手順|予定分|前提|合格条件|状態|メモ|','|---:|---|---|---|---|---:|---|---|---|---|'];
for(const r of s.getRange('A9:N67').values)md.push('|'+[r[0],r[1],r[2],r[3],r[4],r[6],r[8],r[9],r[11],r[13]].map(v=>String(v??'').replaceAll('|','／')).join('|')+'|');
await fs.writeFile(`${dir}/組立・はんだ付け工程表.md`,md.join('\n')+'\n');
console.log(JSON.stringify({source:src,completed:complete,sourceDatesPreserved:true,mainProgressPreserved:true,affected}));
