/*　列車位置検出＆サーボモーター制御　Ver 18.0
 * 
 *　Serial1:19(RX),18(TX)=>RasPi 18Pinレベル変換要(未使用：GPIO）
 *　Servo1(HomeIn):3, Servo2(HomeOut):2
 *　GPIO:1=4,2=5,3=6,4=7,5=8,6=9,7=10,8=11,9=12,10=13(NG)
 *　GPIO:10=14,11=15,12=16,13=17, 14=20,15=21
 *  GPIOLED(red):H12=50,H13=51,H14=52,H15=53
 *  GPIOLED(Color):InSub(Red)=42,InSub(gren)=43,InMain(Red)=44,InMain(Gren)=45
 *  OutSub(Red)=46,OutSub(Gern)=47,OutMain(Red)=48,OutMain(Gern)=49
 *  PrarelOut:Bit0:38,Bit1:39,Bit2:40,Bit3:41（未使用）
 *  BitSelect:Bit0:36,Bit1:37（未使用）
 *  Bus_Location Servo 4個,  充電ステーション クランプ Servo 1個（未対応）
 *
 *　JROBO itoh junichi   2020/09/04
 *                       2020/09/08 // GPIO/Pin変更
 *                       2020/09/08 // Servo+
 *                       2020/09/28 // HoleSenser+1(15)
 *                       2020/09/28 // LED+(red,Color)
 *                       2020/10/10 // RasPi通信（GPIOに変更）
 *                       2020/10/12 // 位置情報のホールド機能追加
 *                       2020/10/13 // 上記機能アルゴリズム見直し
 *                       2020/10/15 // 上記機能アルゴリズム見直し
 *                       2020/10/20 // GPIO_OUT追加
 *                       2020/10/22 // 外周列車追尾システム追加(1)
 *                       2020/10/29 // 外周列車追尾システム追加(2)
 *                       2020/10/30 // 外周列車追尾システム追加(3)
 *                       2020/11/02 // 外周列車追尾システム追加(4)
 *                       2020/11/09 // HomePoji番号変更（列車番号と合致させる）
 *                       2020/11/10 // Home内位置番号変更（２０－２３）
 *                       2020/11/12 // Home入線ルーチン追加
 *                       2020/11/13 // Home入線ルーチン修正
 *                       2020/11/14 // 列車トレース法の変更（リング法）
 *                       2020/11/17 // 新リング法採用
 *                       Sub ={0,20,21,4,5,6,7,8,9,10,11,12,14,15}
 *                       Main={0,16,17,4,5,6,7,8,9,10,11,12,14,15}
 *                       2020/12/03 // 新リング法修正
 *                       2020/12/06 // I2C（20、21ピン）使用の為、ホールセンサNo移動（20->22,21->23）
 *                       2020/12/11 // USB(UART)に変更：Raspiリセット対応が可能となる
 *                       2020/12/15 // チャタリング対応（加算アルゴリズム変更）
 *                       2020/12/19 // SrevoOut制御（RasPi制御）
 *                       2021/01/15 // stand-by機能追加
 *                       2021/01/16 // stand-bySW用動作OK可能LED照明追加
 *                       2021/01/17 // 入線時判断＆入線ポイント切替判断
*/

#include <VarSpeedServo.h>

VarSpeedServo HomeIn;
VarSpeedServo HomeOut;

// サーボ本線・引込線移動角度パラメーター
int posInPSub   = 7;     // 0度相当（本線側）
int posInPMain  = 18;    // 15度相当（引込線側）
int posOutPMain = 6;     // 0度相当（引込線側）
int posOutPSub  = 19;    // 15度相当（本線側）
int speedMax = 0;        // 最高速
int speedMin = 7;        // (低速:1-高速:255)
int ServoInPoji = 0;     // Sub:1, Main:2
int Old_ServoInPoji = 0; // Sub:1, Main:2

int PointFlag3 = 0;
int PointFlag4 = 0;

// ホールセンサ位置（１－１５）
const int digitalPin[17]={0,4,5,6,7,8,9,10,11,12,14,15,16,17,22,23};
const int digitalMainPin[15]={0,16,17,4,5,6,7,8,9,10,11,12,14,15};
const int digitalSubPin[15] ={0,22,23,4,5,6,7,8,9,10,11,12,14,15};
const int digitalPinPS[4]={50,52,51,53};
const int digitalPinSev[8]={42,43,44,45,46,47,48,49};
int stand_by = 0;
int stanbySW = 0;
int fM2 = 0;
int fM1 = 0;
int fS2 = 0;
int fS1 = 0;
int INFI1 = 0;
int INFI2 = 0;
int INFI3 = 0;
int INFI4 = 0;
int StartFlag1 = 0;
int StartFlag2 = 0;
int StartFlag3 = 0;
int StartFlag4 = 0;
int Train[5] = {0,0,0,0,0};
int Loop1 = 1;
int Loop2 = 2;
int Loop3 = 1;
int Loop4 = 2;
int TrainPoji1 = 3;
int TrainPoji2 = 3;
int TrainPoji3 = 2;
int TrainPoji4 = 2;
int TrainNowFlag1 = 0;
int TrainNowFlag2 = 0;
int TrainNowFlag3 = 0;
int TrainNowFlag4 = 0;
int TrainNextFlag1 = 0;
int TrainNextFlag2 = 0;
int TrainNextFlag3 = 0;
int TrainNextFlag4 = 0;
String RaspiCommINT;
int TrainTrans[5]={0,0,0,0,0};

void setup() {
  Serial.begin(2000000);                         // 2020/12/11
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  for(int i=0; i < 4; i++){
    pinMode(digitalPinPS[i],  OUTPUT);
  }
  for(int i=1; i < 16; i++){
    pinMode(digitalPin[i],  INPUT_PULLUP);
  }
  while(RaspiCommINT != "I"){
    if(Serial.available() > 0){
      RaspiCommINT = Serial.readStringUntil('\n');
    }
    digitalWrite(25, LOW);
  }
  while(RaspiCommINT != "G"){
    if(Serial.available() > 0){
      RaspiCommINT = Serial.readStringUntil('\n');
    }
    HomeSignal();
  }
  for(int i=0; i < 8; i++){
    pinMode(digitalPinSev[i],  OUTPUT);
  }
  ServoContInit();
  HomeSignal();
  TrainConfirm();
}

void loop() {
  HomeSignal();
  Old_ServoInPoji = ServoInPoji;
  ServoInCont();
  TrainTrackingMain();
  TrainTrackingSub();
  Traintransformation();
  RaspiCommnd();
//  ServoOutCont();
//  Monitor();
}

void TrainConfirm(){
  fS2 = digitalRead(digitalSubPin[2]);
  fM2 = digitalRead(digitalMainPin[2]);
  fS1 = digitalRead(digitalSubPin[1]);
  fM1 = digitalRead(digitalMainPin[1]);
  if(fS2 == 1){
    Train[1] = 2;
    Loop1 = 1;
    TrainPoji1 = 3;
    }
  if(fM2 == 1){
    Train[2] = 2;
    Loop2 = 2;
    TrainPoji2 = 3;
    }
  if(fS1 == 1){
    Train[3] = 1;
    Loop3 = 1;
    TrainPoji3 = 2;
    }
  if(fM1 == 1){
    Train[4] = 1;
    Loop4 = 2;
    TrainPoji4 = 2;
    }
}

void RaspiCommnd(){
  String DataRequest;
  if(Serial.available() > 0){
    DataRequest = Serial.readStringUntil('\n');
    if(DataRequest == "R"){
      Serial.print(TrainTrans[1]);
//      Serial.print('\n');
      Serial.print(',');
      Serial.print(TrainTrans[2]);
//      Serial.print('\n');
      Serial.print(',');
      Serial.print(TrainTrans[3]);
//      Serial.print('\n');
      Serial.print(',');
      Serial.print(TrainTrans[4]);
//      Serial.print('\n');
      Serial.print(',');
      Serial.println(ServoInPoji);
//      Serial.print('\n');
    }
    if(DataRequest == "M"){
      HomeOut.attach(2);
      delay(700);
      HomeOut.write(posOutPMain, speedMax);
      digitalWrite(digitalPinSev[5], LOW);
      digitalWrite(digitalPinSev[6], LOW);
      digitalWrite(digitalPinSev[7], HIGH);
      digitalWrite(digitalPinSev[4], HIGH);
      HomeOut.wait();
      HomeOut.detach();
    }
    if(DataRequest == "S"){
      HomeOut.attach(2);
      delay(700);
      HomeOut.write(posOutPSub, speedMax);
      digitalWrite(digitalPinSev[4], LOW);
      digitalWrite(digitalPinSev[7], LOW);
      digitalWrite(digitalPinSev[5], HIGH);
      digitalWrite(digitalPinSev[6], HIGH);
      HomeOut.wait();
      HomeOut.detach();
    }
   }
  }

void Monitor(){
  Serial.print("1 : ");
  Serial.print(Train[1]);
  Serial.print("      2 : ");
  Serial.print(Train[2]);
  Serial.print("      3 : ");
  Serial.print(Train[3]);
  Serial.print("      4 : ");
  Serial.println(Train[4]);
  Serial.print("ServoInPoji: ");
  Serial.println(ServoInPoji);
  Serial.print("Loop1: ");
  Serial.print(Loop1);
  Serial.print("  Loop2: ");
  Serial.print(Loop2);
  Serial.print("  Loop3: ");
  Serial.print(Loop3);
  Serial.print("  Loop4: ");
  Serial.println(Loop4);
  Serial.println("");
  Serial.print("TrainTrans[1]: ");
  Serial.print(TrainTrans[1]);
  Serial.print("  TrainTrans[2]: ");
  Serial.print(TrainTrans[2]);
  Serial.print("  TrainTrans[3]: ");
  Serial.print(TrainTrans[3]);
  Serial.print("  TrainTrans[4]: ");
  Serial.print(TrainTrans[4]);
  Serial.println("");
  Serial.println("");
  delay(1000);
}

void TrainTrackingMain(){
  if(Loop1==1){
    if(Train[1]==1){
      TrainNowFlag1 = digitalRead(digitalSubPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalSubPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       TrainTrans[2]!=22 && TrainTrans[3]!=22 && TrainTrans[4]!=22){
      Train[1] = 2;
      TrainPoji1 = 3;
    }
   }
    if(Train[1]>1 && Train[1]<13){
      TrainNowFlag1 = digitalRead(digitalSubPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalSubPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       Train[2]!=TrainPoji1 && Train[3]!=TrainPoji1 && Train[4]!=TrainPoji1){
      Train[1] = Train[1] + 1;
      TrainPoji1 = TrainPoji1 + 1;
    }
   }
  if(Train[1]==13 && TrainTrans[2]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
      TrainNowFlag1 = digitalRead(digitalSubPin[13]);
      TrainNextFlag1 = digitalRead(digitalSubPin[1]);
        if(TrainNextFlag1==1 && TrainNowFlag1==0){
          Train[1] = 1;
          TrainPoji1 = 2;
        }
  }
  if(Train[1]==13  && TrainTrans[2]!=31 && TrainTrans[3]!=31
      && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop1 = 2; // ループ２に切替
      TrainNowFlag1 = digitalRead(digitalMainPin[13]);
      TrainNextFlag1 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
 }
  if(Loop1==2){
  if(Train[1]==1){
      TrainNowFlag1 = digitalRead(digitalMainPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalMainPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       TrainTrans[2]!=32 && TrainTrans[3]!=32 && TrainTrans[4]!=32){
      Train[1] = 2;
      TrainPoji1 = 3;
    }
   }
  if(Train[1]>1 && Train[1]<13){
      TrainNowFlag1 = digitalRead(digitalMainPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalMainPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       Train[2]!=TrainPoji1 && Train[3]!=TrainPoji1 && Train[4]!=TrainPoji1){
      Train[1] = Train[1] + 1;
      TrainPoji1 = TrainPoji1 + 1;
    }
   }
  if(Train[1]==13 && TrainTrans[2]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop1 = 1;
    TrainNowFlag1 = digitalRead(digitalSubPin[13]);
    TrainNextFlag1 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
  if(Train[1]==13 && TrainTrans[2]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag1 = digitalRead(digitalMainPin[13]);
      TrainNextFlag1 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
 }

  if(Loop2==1){
  if(Train[2]==1){
      TrainNowFlag2 = digitalRead(digitalSubPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalSubPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       TrainTrans[1]!=22 && TrainTrans[3]!=22 && TrainTrans[4]!=22){
      Train[2] = 2;
      TrainPoji2 = 3;
    }
   }
    if(Train[2]>1 && Train[2]<13){
      TrainNowFlag2 = digitalRead(digitalSubPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalSubPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       Train[1]!=TrainPoji2 && Train[3]!=TrainPoji2 && Train[4]!=TrainPoji2){
      Train[2] = Train[2] + 1;
      TrainPoji2 = TrainPoji2 + 1;
    }
   }
  if(Train[2]==13 && TrainTrans[1]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    TrainNowFlag2 = digitalRead(digitalSubPin[13]);
    TrainNextFlag2 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
  if(Train[2]==13 && TrainTrans[1]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop2 = 2;
      TrainNowFlag2 = digitalRead(digitalMainPin[13]);
      TrainNextFlag2 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
 }

  if(Loop2==2){
    if(Train[2]==1){
      TrainNowFlag2 = digitalRead(digitalMainPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalMainPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       TrainTrans[1]!=32 && TrainTrans[3]!=32 && TrainTrans[4]!=32){
      Train[2] = 2;
      TrainPoji2 = 3;
    }
   }
    if(Train[2]>1 && Train[2]<13){
      TrainNowFlag2 = digitalRead(digitalMainPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalMainPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       Train[1]!=TrainPoji2 && Train[3]!=TrainPoji2 && Train[4]!=TrainPoji2){
      Train[2] = Train[2] + 1;
      TrainPoji2 = TrainPoji2 + 1;
    }
   }
  if(Train[2]==13 && TrainTrans[1]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop2 = 1;
    TrainNowFlag2 = digitalRead(digitalSubPin[13]);
    TrainNextFlag2 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
  if(Train[2]==13 && TrainTrans[1]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag2 = digitalRead(digitalMainPin[13]);
      TrainNextFlag2 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
}
  if(Loop3==1){
    if(Train[3]==1){
      TrainNowFlag3 = digitalRead(digitalSubPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalSubPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       TrainTrans[1]!=22 && TrainTrans[2]!=22 && TrainTrans[4]!=22){
      Train[3] = 2;
      TrainPoji3 = 3;
    }
   }
    if(Train[3]>1 && Train[3]<13){
      TrainNowFlag3 = digitalRead(digitalSubPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalSubPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       Train[1]!=TrainPoji3 && Train[2]!=TrainPoji3 && Train[4]!=TrainPoji3){
      Train[3] = Train[3] + 1;
      TrainPoji3 = TrainPoji3 + 1;
    }
   }
  if(Train[3]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    TrainNowFlag3 = digitalRead(digitalSubPin[13]);
    TrainNextFlag3 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
  if(Train[3]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop3 = 2;
      TrainNowFlag3 = digitalRead(digitalMainPin[13]);
      TrainNextFlag3 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
 }
  if(Loop3==2){
    if(Train[3]==1){
      TrainNowFlag3 = digitalRead(digitalMainPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalMainPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       TrainTrans[1]!=32 && TrainTrans[2]!=32 && TrainTrans[4]!=32){
      Train[3] = 2;
      TrainPoji3 = 3;
    }
   }
    if(Train[3]>1 && Train[3]<13){
      TrainNowFlag3 = digitalRead(digitalMainPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalMainPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       Train[1]!=TrainPoji3 && Train[2]!=TrainPoji3 && Train[4]!=TrainPoji3){
      Train[3] = Train[3] + 1;
      TrainPoji3 = TrainPoji3 + 1;
    }
   }
  if(Train[3]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop3 = 1;
    TrainNowFlag3 = digitalRead(digitalSubPin[13]);
    TrainNextFlag3 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
  if(Train[3]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag3 = digitalRead(digitalMainPin[13]);
      TrainNextFlag3 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
}
  if(Loop4==1){
    if(Train[4]==1){
      TrainNowFlag4 = digitalRead(digitalSubPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalSubPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       TrainTrans[1]!=22 && TrainTrans[2]!=22 && TrainTrans[3]!=22){
      Train[4] = 2;
      TrainPoji4 = 3;
    }
   }
    if(Train[4]>1 && Train[4]<13){
      TrainNowFlag4 = digitalRead(digitalSubPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalSubPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       Train[1]!=TrainPoji4 && Train[2]!=TrainPoji4 && Train[3]!=TrainPoji4){
      Train[4] = Train[4] + 1;
      TrainPoji4 = TrainPoji4 + 1;
    }
   }
  if(Train[4]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[3]!=21 && Old_ServoInPoji==1){
    TrainNowFlag4 = digitalRead(digitalSubPin[13]);
    TrainNextFlag4 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
  if(Train[4]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[3]!=31 && Old_ServoInPoji==2){
      Loop4 = 2;
      TrainNowFlag4 = digitalRead(digitalMainPin[13]);
      TrainNextFlag4 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
 }
  if(Loop4==2){
    if(Train[4]==1){
      TrainNowFlag4 = digitalRead(digitalMainPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalMainPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       TrainTrans[1]!=32 && TrainTrans[2]!=32 && TrainTrans[3]!=32){
      Train[4] = 2;
      TrainPoji4 = 3;
    }
   }
    if(Train[4]>1 && Train[4]<13){
      TrainNowFlag4 = digitalRead(digitalMainPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalMainPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       Train[1]!=TrainPoji4 && Train[2]!=TrainPoji4 && Train[3]!=TrainPoji4){
      Train[4] = Train[4] + 1;
      TrainPoji4 = TrainPoji4 + 1;
    }
   }
  if(Train[4]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[3]!=21 && Old_ServoInPoji==1){
    Loop4 = 1;
    TrainNowFlag4 = digitalRead(digitalSubPin[13]);
    TrainNextFlag4 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
  if(Train[4]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[3]!=31 && Old_ServoInPoji==2){
      TrainNowFlag4 = digitalRead(digitalMainPin[13]);
      TrainNextFlag4 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
 }
}

void TrainTrackingSub(){
  if(Loop1==1){
    if(Train[1]==1){
      TrainNowFlag1 = digitalRead(digitalSubPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalSubPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       TrainTrans[2]!=22 && TrainTrans[3]!=22 && TrainTrans[4]!=22){
      Train[1] = 2;
      TrainPoji1 = 3;
    }
   }
    if(Train[1]>1 && Train[1]<13){
      TrainNowFlag1 = digitalRead(digitalSubPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalSubPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       Train[2]!=TrainPoji1 && Train[3]!=TrainPoji1 && Train[4]!=TrainPoji1){
      Train[1] = Train[1] + 1;
      TrainPoji1 = TrainPoji1 + 1;
    }
   }
  if(Train[1]==13 && TrainTrans[2]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
      TrainNowFlag1 = digitalRead(digitalSubPin[13]);
      TrainNextFlag1 = digitalRead(digitalSubPin[1]);
        if(TrainNextFlag1==1 && TrainNowFlag1==0){
          Train[1] = 1;
          TrainPoji1 = 2;
        }
  }
  if(Train[1]==13  && TrainTrans[2]!=31 && TrainTrans[3]!=31
      && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop1 = 2; // ループ２に切替
      TrainNowFlag1 = digitalRead(digitalMainPin[13]);
      TrainNextFlag1 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
 }
  if(Loop1==2){
  if(Train[1]==1){
      TrainNowFlag1 = digitalRead(digitalMainPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalMainPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       TrainTrans[2]!=32 && TrainTrans[3]!=32 && TrainTrans[4]!=32){
      Train[1] = 2;
      TrainPoji1 = 3;
    }
   }
  if(Train[1]>1 && Train[1]<13){
      TrainNowFlag1 = digitalRead(digitalMainPin[Train[1]]);
      TrainNextFlag1 = digitalRead(digitalMainPin[TrainPoji1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0 &&
       Train[2]!=TrainPoji1 && Train[3]!=TrainPoji1 && Train[4]!=TrainPoji1){
      Train[1] = Train[1] + 1;
      TrainPoji1 = TrainPoji1 + 1;
    }
   }
  if(Train[1]==13 && TrainTrans[2]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop1 = 1;
    TrainNowFlag1 = digitalRead(digitalSubPin[13]);
    TrainNextFlag1 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
  if(Train[1]==13 && TrainTrans[2]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag1 = digitalRead(digitalMainPin[13]);
      TrainNextFlag1 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag1==1 && TrainNowFlag1==0){
      Train[1] = 1;
      TrainPoji1 = 2;
    }
  }
 }

  if(Loop2==1){
  if(Train[2]==1){
      TrainNowFlag2 = digitalRead(digitalSubPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalSubPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       TrainTrans[1]!=22 && TrainTrans[3]!=22 && TrainTrans[4]!=22){
      Train[2] = 2;
      TrainPoji2 = 3;
    }
   }
    if(Train[2]>1 && Train[2]<13){
      TrainNowFlag2 = digitalRead(digitalSubPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalSubPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       Train[1]!=TrainPoji2 && Train[3]!=TrainPoji2 && Train[4]!=TrainPoji2){
      Train[2] = Train[2] + 1;
      TrainPoji2 = TrainPoji2 + 1;
    }
   }
  if(Train[2]==13 && TrainTrans[1]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    TrainNowFlag2 = digitalRead(digitalSubPin[13]);
    TrainNextFlag2 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
  if(Train[2]==13 && TrainTrans[1]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop2 = 2;
      TrainNowFlag2 = digitalRead(digitalMainPin[13]);
      TrainNextFlag2 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
 }

  if(Loop2==2){
    if(Train[2]==1){
      TrainNowFlag2 = digitalRead(digitalMainPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalMainPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       TrainTrans[1]!=32 && TrainTrans[3]!=32 && TrainTrans[4]!=32){
      Train[2] = 2;
      TrainPoji2 = 3;
    }
   }
    if(Train[2]>1 && Train[2]<13){
      TrainNowFlag2 = digitalRead(digitalMainPin[Train[2]]);
      TrainNextFlag2 = digitalRead(digitalMainPin[TrainPoji2]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0 &&
       Train[1]!=TrainPoji2 && Train[3]!=TrainPoji2 && Train[4]!=TrainPoji2){
      Train[2] = Train[2] + 1;
      TrainPoji2 = TrainPoji2 + 1;
    }
   }
  if(Train[2]==13 && TrainTrans[1]!=21 && TrainTrans[3]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop2 = 1;
    TrainNowFlag2 = digitalRead(digitalSubPin[13]);
    TrainNextFlag2 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
  if(Train[2]==13 && TrainTrans[1]!=31 && TrainTrans[3]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag2 = digitalRead(digitalMainPin[13]);
      TrainNextFlag2 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag2==1 && TrainNowFlag2==0){
      Train[2] = 1;
      TrainPoji2 = 2;
    }
  }
}
  if(Loop3==1){
    if(Train[3]==1){
      TrainNowFlag3 = digitalRead(digitalSubPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalSubPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       TrainTrans[1]!=22 && TrainTrans[2]!=22 && TrainTrans[4]!=22){
      Train[3] = 2;
      TrainPoji3 = 3;
    }
   }
    if(Train[3]>1 && Train[3]<13){
      TrainNowFlag3 = digitalRead(digitalSubPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalSubPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       Train[1]!=TrainPoji3 && Train[2]!=TrainPoji3 && Train[4]!=TrainPoji3){
      Train[3] = Train[3] + 1;
      TrainPoji3 = TrainPoji3 + 1;
    }
   }
  if(Train[3]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    TrainNowFlag3 = digitalRead(digitalSubPin[13]);
    TrainNextFlag3 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
  if(Train[3]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      Loop3 = 2;
      TrainNowFlag3 = digitalRead(digitalMainPin[13]);
      TrainNextFlag3 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
 }
  if(Loop3==2){
    if(Train[3]==1){
      TrainNowFlag3 = digitalRead(digitalMainPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalMainPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       TrainTrans[1]!=32 && TrainTrans[2]!=32 && TrainTrans[4]!=32){
      Train[3] = 2;
      TrainPoji3 = 3;
    }
   }
    if(Train[3]>1 && Train[3]<13){
      TrainNowFlag3 = digitalRead(digitalMainPin[Train[3]]);
      TrainNextFlag3 = digitalRead(digitalMainPin[TrainPoji3]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0 &&
       Train[1]!=TrainPoji3 && Train[2]!=TrainPoji3 && Train[4]!=TrainPoji3){
      Train[3] = Train[3] + 1;
      TrainPoji3 = TrainPoji3 + 1;
    }
   }
  if(Train[3]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[4]!=21 && Old_ServoInPoji==1){
    Loop3 = 1;
    TrainNowFlag3 = digitalRead(digitalSubPin[13]);
    TrainNextFlag3 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
  if(Train[3]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[4]!=31 && Old_ServoInPoji==2){
      TrainNowFlag3 = digitalRead(digitalMainPin[13]);
      TrainNextFlag3 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag3==1 && TrainNowFlag3==0){
      Train[3] = 1;
      TrainPoji3 = 2;
    }
  }
}
  if(Loop4==1){
    if(Train[4]==1){
      TrainNowFlag4 = digitalRead(digitalSubPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalSubPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       TrainTrans[1]!=22 && TrainTrans[2]!=22 && TrainTrans[3]!=22){
      Train[4] = 2;
      TrainPoji4 = 3;
    }
   }
    if(Train[4]>1 && Train[4]<13){
      TrainNowFlag4 = digitalRead(digitalSubPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalSubPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       Train[1]!=TrainPoji4 && Train[2]!=TrainPoji4 && Train[3]!=TrainPoji4){
      Train[4] = Train[4] + 1;
      TrainPoji4 = TrainPoji4 + 1;
    }
   }
  if(Train[4]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[3]!=21 && Old_ServoInPoji==1){
    TrainNowFlag4 = digitalRead(digitalSubPin[13]);
    TrainNextFlag4 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
  if(Train[4]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[3]!=31 && Old_ServoInPoji==2){
      Loop4 = 2;
      TrainNowFlag4 = digitalRead(digitalMainPin[13]);
      TrainNextFlag4 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
 }
  if(Loop4==2){
    if(Train[4]==1){
      TrainNowFlag4 = digitalRead(digitalMainPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalMainPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       TrainTrans[1]!=32 && TrainTrans[2]!=32 && TrainTrans[3]!=32){
      Train[4] = 2;
      TrainPoji4 = 3;
    }
   }
    if(Train[4]>1 && Train[4]<13){
      TrainNowFlag4 = digitalRead(digitalMainPin[Train[4]]);
      TrainNextFlag4 = digitalRead(digitalMainPin[TrainPoji4]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0 &&
       Train[1]!=TrainPoji4 && Train[2]!=TrainPoji4 && Train[3]!=TrainPoji4){
      Train[4] = Train[4] + 1;
      TrainPoji4 = TrainPoji4 + 1;
    }
   }
  if(Train[4]==13 && TrainTrans[1]!=21 && TrainTrans[2]!=21
    && TrainTrans[3]!=21 && Old_ServoInPoji==1){
    Loop4 = 1;
    TrainNowFlag4 = digitalRead(digitalSubPin[13]);
    TrainNextFlag4 = digitalRead(digitalSubPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
  if(Train[4]==13 && TrainTrans[1]!=31 && TrainTrans[2]!=31
    && TrainTrans[3]!=31 && Old_ServoInPoji==2){
      TrainNowFlag4 = digitalRead(digitalMainPin[13]);
      TrainNextFlag4 = digitalRead(digitalMainPin[1]);
    if(TrainNextFlag4==1 && TrainNowFlag4==0){
      Train[4] = 1;
      TrainPoji4 = 2;
    }
  }
 }
}

void Traintransformation(){
  if(Train[1] == 2 && Loop1 == 1){                //
    TrainTrans[1] = 22;                           //
  }
  if(Train[1] == 2 && Loop1 == 2){                //
    TrainTrans[1] = 32;                           //
  }
  if(Train[1] == 1 && Loop1 == 1){                //
    TrainTrans[1] = 21;                           //
  }
  if(Train[1] == 1 && Loop1 == 2){                //
    TrainTrans[1] = 31;                           //
  }
  if(Train[1] > 2 && Train[1] < 14){              //
    TrainTrans[1] = Train[1];                     //
  }
  if(Train[2] == 2 && Loop2 == 1){                //
    TrainTrans[2] = 22;                           //
  }
  if(Train[2] == 2 && Loop2 == 2){                //
    TrainTrans[2] = 32;                           //
  }
  if(Train[2] == 1 && Loop2 == 1){                //
    TrainTrans[2] = 21;                           //
  }
  if(Train[2] == 1 && Loop2 == 2){                //
    TrainTrans[2] = 31;                           //
  }
  if(Train[2] > 2 && Train[2] < 14){              //
    TrainTrans[2] = Train[2];                     //
  }                                               //
  if(Train[3] == 2 && Loop3 == 1){                //
    TrainTrans[3] = 22;                           //
  }
  if(Train[3] == 2 && Loop3 == 2){                //
    TrainTrans[3] = 32;                           //
  }
  if(Train[3] == 1 && Loop3 == 1){                //
    TrainTrans[3] = 21;                           //
  }                                               //
  if(Train[3] == 1 && Loop3 == 2){                //
    TrainTrans[3] = 31;                           //
  }
  if(Train[3] > 2 && Train[3] < 14){              //
    TrainTrans[3] = Train[3];                     //
  }                                               //
  if(Train[4] == 2 && Loop4 == 1){                //
    TrainTrans[4] = 22;                           //
  }
  if(Train[4] == 2 && Loop4 == 2){                //
    TrainTrans[4] = 32;                           //
  }
  if(Train[4] == 1 && Loop4 == 1){                //
    TrainTrans[4] = 21;                           //
  }
  if(Train[4] == 1 && Loop4 == 2){                //
    TrainTrans[4] = 31;                           //
  }
  if(Train[4] > 2 && Train[4] < 14){              //
    TrainTrans[4] = Train[4];                     //
  }
}

void ServoContInit(){
  HomeIn.attach(3);
  HomeOut.attach(2);
  delay(100);
//  HomeIn.write(posInPMain, speedMax);  // In（本線側）
//  digitalWrite(digitalPinSev[3], HIGH);// Pin45(MainGreen)
//  digitalWrite(digitalPinSev[0], HIGH);// Pin42(SubRed)
  HomeIn.write(posInPSub, speedMax);     // In（引込側）
  digitalWrite(digitalPinSev[1], HIGH);  // Pin43(SubGreen)
  digitalWrite(digitalPinSev[2], HIGH);  // Pin44(MainRed)
//  HomeOut.write(posOutPMain, speedMax);// Out(本線側)
//  digitalWrite(digitalPinSev[7], HIGH);// Pin49(MainGreen)
//  digitalWrite(digitalPinSev[4], HIGH);// Pin46(SubRed)
  HomeOut.write(posOutPSub, speedMax);   // Out(引込側）
  digitalWrite(digitalPinSev[5], HIGH);  // Pin47(SubGreen)
  digitalWrite(digitalPinSev[6], HIGH);  // Pin48(MainRed)
  HomeIn.wait();
  HomeOut.wait();
  HomeIn.detach();
  HomeOut.detach();
}

void ServoInCont(){
  HomeIn.attach(3);
  delay(30);
  if(((Train[1] == 1 && Loop1 == 1) || (Train[2] == 1 && Loop1 == 1) ||
      (Train[3] == 1 && Loop1 == 1) || (Train[4] == 1 && Loop1 == 1)) ||
      ((Train[1] == 1 && Loop1 == 2) || (Train[2] == 1 && Loop1 == 2) ||
      (Train[3] == 1 && Loop1 == 2) || (Train[4] == 1 && Loop1 == 2))){
     HomeIn.write(posInPMain, speedMax);
     ServoInPoji = 2; // Main
     digitalWrite(digitalPinSev[1], LOW); // Pin43(SubGreen)
     digitalWrite(digitalPinSev[2], LOW); // Pin44(MainRed)
     digitalWrite(digitalPinSev[3], HIGH);// Pin45(MainGreen)
     digitalWrite(digitalPinSev[0], HIGH);// Pin42(SubRed)
  }else{
    HomeIn.write(posInPSub, speedMax);
    ServoInPoji = 1;  // Sub
    digitalWrite(digitalPinSev[0], LOW); // Pin42(SubRed)
    digitalWrite(digitalPinSev[3], LOW); // Pin45(MainGreen)
    digitalWrite(digitalPinSev[1], HIGH);// Pin43(SubGreen)
    digitalWrite(digitalPinSev[2], HIGH);// Pin44(MainRed)
  }
  HomeIn.wait();
  HomeIn.detach();
}

void ServoOutCont(){
  String ServoOutComm;
  if(Serial.available() > 0){
    ServoOutComm = Serial.readStringUntil('\n');
//   }
    Serial.print(ServoOutComm);
    if(ServoOutComm=="S"){
//      Serial.print("S");
      HomeOut.attach(2);
      delay(500);
      HomeOut.write(posOutPSub, speedMax);   // Out(引込側）
      digitalWrite(digitalPinSev[4], LOW);   // Pin46(SubRed)
      digitalWrite(digitalPinSev[7], LOW);   // Pin49(MainGreen)
      digitalWrite(digitalPinSev[5], HIGH);  // Pin47(SubGreen)
      digitalWrite(digitalPinSev[6], HIGH);  // Pin48(MainRed)
    }
    if(ServoOutComm=="M"){
//      Serial.print("M");
      HomeOut.attach(2);
      delay(500);
      HomeOut.write(posOutPMain, speedMax);// Out(本線側)
      digitalWrite(digitalPinSev[5], LOW); // Pin47(SubGreen)
      digitalWrite(digitalPinSev[6], LOW); // Pin48(MainRed)
      digitalWrite(digitalPinSev[7], HIGH);// Pin49(MainGreen)
      digitalWrite(digitalPinSev[4], HIGH);// Pin46(SubRed)
    }
  }
   HomeOut.wait();
   HomeOut.detach();
}

void HomeSignal(){
  int SigSub1 = digitalRead(digitalSubPin[2]);
  if(SigSub1 == HIGH){
    digitalWrite(digitalPinPS[3], HIGH);
  }else{
    digitalWrite(digitalPinPS[3], LOW);
  }

  int SigMain1 = digitalRead(digitalMainPin[2]);
  if(SigMain1 == HIGH){
    digitalWrite(digitalPinPS[2], HIGH);
  }else{
    digitalWrite(digitalPinPS[2], LOW);
  }

  int SigSub2 = digitalRead(digitalSubPin[1]);
  if(SigSub2 == HIGH){
    digitalWrite(digitalPinPS[1], HIGH);
  }else{
    digitalWrite(digitalPinPS[1], LOW);
  }
  
  int SigMain2 = digitalRead(digitalMainPin[1]);
  if(SigMain2 == HIGH){
    digitalWrite(digitalPinPS[0], HIGH);
  }else{
    digitalWrite(digitalPinPS[0], LOW);
  }
}
