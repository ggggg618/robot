/*
 * ===== 遥控板程序（任务三 · 第二块板）=====
 *
 * 本程序烧在"遥控板/自制板"上，只负责一件事：
 *   读 4 个按键，通过串口把指令发给主控板。
 *
 * 本程序不接舵机、不控制任何动作。
 * 主控板收到这些指令后，才去执行对应动作。
 */

// 4 个按键的引脚
const int k1=4;//循环
const int k2=5;//录制
const int k3=6;//播放
const int k4=7;//回中

void setup() {
  Serial.begin(9600);
  //按键设置
  pinMode(k1,INPUT_PULLUP);
  pinMode(k2,INPUT_PULLUP);
  pinMode(k3,INPUT_PULLUP);
  pinMode(k4,INPUT_PULLUP);
}

void loop() {
  if(digitalRead(k1)==LOW){Serial.println("K1");delay(200);}
  if(digitalRead(k2)==LOW){Serial.println("K2");delay(200);}
  if(digitalRead(k3)==LOW){Serial.println("K3");delay(200);}
  if(digitalRead(k4)==LOW){Serial.println("K4");delay(200);}
}
