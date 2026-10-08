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
 * 引脚：底座9、肩8、肘7、夹爪6
 */

//存放数据处--------------
//引脚
const int bottom_yijiao=9;
const int left_yijiao=8;
const int right_yijiao=7;
const int claw_yijiao=6;

//变量声明
int speed;//用来控制机械臂的整体速度
int angle;//角度
bool change=true;//模式开关：true=摇杆模式，false=串口指令模式
Servo bottom;//底座
Servo left;//左边
Servo right;//右边
Servo claw;//夹爪

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
  moveTo(,,,,);
}
//B
void grabB(){
  moveTo(,,,,);
}
//C
void grabC(){
  moveTo(,,,,);
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
  speed=10;
}

void loop() {
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
      moveServo(claw, 120, speed);
      Serial.println("open");
    }
    //接收S
    else if(input=="S"){
      moveServo(claw, 30, speed);
      Serial.println("close");
    }
    //接收H
    else if(input=="H"){
      speed-=2;
      if(speed<2){
        speed=2;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
    //接收L
    else if(input=="L"){
      speed+=2;
      if(speed>50){
        speed=50;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
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
    else{
    //实现舵机控制
    int b=findangle(input,'b');
    int l=findangle(input,'l');
    int r=findangle(input,'r');
    int c=findangle(input,'c');
    moveServo(bottom, b, speed);
    moveServo(left, l, speed);
    moveServo(right, r, speed);
    moveServo(claw, c, speed);
    }
    }
  }
}
