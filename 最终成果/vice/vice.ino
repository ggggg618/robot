/*
 * ===== 遥控板程序 ======
 *   读 4 个按键，通过串口把指令发给主控板。
 * 本程序不接舵机、不控制任何动作。
 * 主控板收到这些指令后，才去执行对应动作。
 */

// 11 个按键的引脚
const int k1=2;//循环
const int k2=3;//录制
const int k3=4;//播放
const int k4=5;//回中
const int D=6;//画直线
const int T=7;//画三角形
const int k5=8;//暂停/切换
const int k6=9;//取消
const int k7=10;//记录视教点
const int k8=11;//画五点折线
const int k9=12;//画五点曲线

void setup() {
  Serial.begin(9600);
  //按键设置
  pinMode(k1,INPUT_PULLUP);
  pinMode(k2,INPUT_PULLUP);
  pinMode(k3,INPUT_PULLUP);
  pinMode(k4,INPUT_PULLUP);
  pinMode(D,INPUT_PULLUP);
  pinMode(T,INPUT_PULLUP);
  pinMode(k5,INPUT_PULLUP);
  pinMode(k6,INPUT_PULLUP);
  pinMode(k7,INPUT_PULLUP);
  pinMode(k8,INPUT_PULLUP);
  pinMode(k9,INPUT_PULLUP);
}

void loop() {
  if(digitalRead(k1)==LOW){Serial.println("K1");delay(200);}
  if(digitalRead(k2)==LOW){Serial.println("K2");delay(200);}
  if(digitalRead(k3)==LOW){Serial.println("K3");delay(200);}
  if(digitalRead(k4)==LOW){Serial.println("K4");delay(200);}
  if(digitalRead(D)==LOW){Serial.println("D");delay(200);}
  if(digitalRead(T)==LOW){Serial.println("T");delay(200);}
  if(digitalRead(k5)==LOW){Serial.println("K5");delay(200);}
  if(digitalRead(k6)==LOW){Serial.println("K6");delay(200);}
  if(digitalRead(k7)==LOW){Serial.println("K7");delay(200);}
  if(digitalRead(k8)==LOW){Serial.println("K8");delay(200);}
  if(digitalRead(k9)==LOW){Serial.println("K9");delay(200);}
}
