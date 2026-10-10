#include<Servo.h>

/*
 * ===== 机械臂控制程序（算法方向）=====
 *
 * 本程序实现任务一的功能：
 *   1. 双摇杆实时操控 4 个关节（基础控制）
 *   2. 串口固定指令 O/S/H/L（固定指令通信）
 *   3. b/l/r/c 数字指令控制各关节（多舵机协同）
 *
 * 串口指令一览：
 *   J               → 切回摇杆模式
 *   O / S           → 夹爪张开 / 关闭
 *   H / L           → 加速 / 减速
 *   b90,l60,r30,c20 → 底座90°、肩60°、肘30°、夹爪20°
 *
 * 引脚：底座9、肩（左）8、肘（右）7、夹爪6
 */

//存放数据处--------------
//引脚
const int bottom_yijiao=9;
const int left_yijiao=8;
const int right_yijiao=7;
const int claw_yijiao=6;

//变量声明
int moveDelay;//每步移动的延时（越小越快），H/L 调速用
int angle;//角度
int cycle=0;//用来记录任务三按键1的点击次数
int number=0;//用来记录任务三按键2录制时位置个数
bool change=true;//模式开关：true=摇杆模式，false=串口指令模式
bool recording=false;//用来记录是否在录制中
Servo bottom;//底座
Servo left;//左边
Servo right;//右边
Servo claw;//夹爪
//数组变量声明（用来处理任务三中的录制和播放）
int bottomR[200];
int leftR[200];
int rightR[200];
int clawR[200];

/*为完成多舵机协同控制，并防止上位机输入数据的格式没有按顺序
用findangle函数来返回舵机所对应角度*/
int findangle(String s,char label){
  int num=s.indexOf(label);//找到编号的位置
  if(num==-1){
    return 0;
  }
  else return s.substring(++num).toInt();
}

// 从当前角度平滑移动到目标角度，stepDelay 控制速度
// 原理：不直接 write(目标)，而是每次加/减 1 度，每步 delay 一下
// stepDelay 越小 → 移动越快（H/L 调速的底层实现）
void moveServo(Servo &s, int target, int stepDelay) {
  int current = s.read();   // 读当前角度（上次 write 的值）
  // 目标比当前大 → 递增扫过去
  if (current < target) {
    for (int a = current; a <= target; a++) {
      s.write(a);           // 每步转 1 度
      delay(stepDelay);     // 每步停一下，控制速度
    }
  }
  // 目标比当前小 → 递减扫过去
  else if (current > target) {
    for (int a = current; a >= target; a--) {
      s.write(a);
      delay(stepDelay);
    }
  }
}
// 把 4 个舵机同时移动到指定角度
void moveTo(int b, int l, int r, int c, int spd) {
  moveServo(bottom, b, spd);   // 底座
  moveServo(left, l, spd);     // 肩
  moveServo(right, r, spd);    // 肘
  moveServo(claw, c, spd);     // 夹爪
}
//以下写任务二的三个动作
//A
void grabA(){
  moveTo(90,90,90,60,moveDelay);
}
//B
void grabB(){
  moveTo(90,90,90,60,moveDelay);
}
//C
void grabC(){
  moveTo(90,90,90,60,moveDelay);
}


void setup() {
  //接引脚
  bottom.attach(bottom_yijiao);
  left.attach(left_yijiao);
  right.attach(right_yijiao);
  claw.attach(claw_yijiao);
  //串口初始化
  Serial.begin(9600);
  //速度初始化
  moveDelay=10;
}

void loop() {
  // =====录制状态=====
  if(recording){
    int b=map(analogRead(A0),0,1023,0,180);
    int l=map(analogRead(A1),0,1023,0,180);
    int r=map(analogRead(A2),0,1023,0,180);
    int c=map(analogRead(A3),0,1023,0,180);
    bottom.write(b); left.write(l); right.write(r); claw.write(c);
    //存位置到数组中
    bottomR[number]=b;
    leftR[number]=l;
    rightR[number]=r;
    clawR[number]=c;
    number++;
    if(number>=200)number=200;
    delay(100);
  }
  // ===== 摇杆模式：实时操控（默认模式）=====
  if(change){
  int x1=analogRead(A0),//底座（摇杆1的X轴）
      x2=analogRead(A1),//肩（摇杆1的Y轴）
      x3=analogRead(A2),//肘（摇杆2的X轴）
      x4=analogRead(A3);//夹爪（摇杆2的Y轴）
  bottom.write(map(x1,0,1023,0,180));
  left.write(map(x2,0,1023,0,180));
  right.write(map(x3,0,1023,0,180));
  claw.write(map(x4,0,1023,0,180));
  }
  if(Serial.available()>0){
    //串口输入存储
    String input=Serial.readString();
    input.trim();
    //接收到J切回摇杆模式
    if(input=="J"){
      change=true;
      Serial.println("摇杆模式");
    }else{
      change=false;//切回串口模式
      Serial.println("串口模式");
    //接收O
    if(input=="O"){
      moveServo(claw, 30, moveDelay);
      Serial.println("open");
    }
    //接收S
    else if(input=="S"){
      moveServo(claw, 120, moveDelay);
      Serial.println("close");
    }
    //接收H
    else if(input=="H"){
      moveDelay-=2;
      if(moveDelay<2){
        moveDelay=2;
      }
      Serial.print("延时=");
      Serial.println(moveDelay);
    }
    //接收L
    else if(input=="L"){
      moveDelay+=2;
      if(moveDelay>50){
        moveDelay=50;
      }
      Serial.print("延时=");
      Serial.println(moveDelay);
    }
    //任务二 三个动作
    else if(input=="A"){
      grabA();
      Serial.println("A finish");
    }
    else if(input=="B"){
      grabB();
      Serial.println("B finish");
    }
    else if(input=="C"){
      grabC();
      Serial.println("C finish");
    }

    //任务三四个按键
    //循环
    else if(input=="K1"){
      cycle++;
      if(cycle>3)cycle=1;
      if(cycle==1){
        grabA();
      }else if(cycle==2){
        grabB();
      }else{
        grabC();
      }
      Serial.println("夹取完毕");
    }
    //录制
    else if(input=="K2"){
      recording=!recording;
      if(recording){
        number=0;
        Serial.println("开始录制");
      }else{
        Serial.println("停止录制");
      }
    }
    //播放
    else if(input=="K3"){
      Serial.println("开始播放");
      for(int i=0;i<number;i++){
        bottom.write(bottomR[i]);
        left.write(leftR[i]);
        right.write(rightR[i]);
        claw.write(clawR[i]);
        delay(100);
      }
      Serial.println("播放完毕");
    }
    //回中
    else if(input=="K4"){
      moveTo(90,90,90,60,moveDelay);
    }
    else{
    //实现舵机控制
    int b=findangle(input,'b');
    int l=findangle(input,'l');
    int r=findangle(input,'r');
    int c=findangle(input,'c');
    moveServo(bottom, b, moveDelay);
    moveServo(left, l, moveDelay);
    moveServo(right, r, moveDelay);
    moveServo(claw, c, moveDelay);
    }
    }
  }
}
