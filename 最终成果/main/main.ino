#include<Servo.h>

/*
 * ===== 机械臂控制程序=====
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
//任务五数据
float L1=1;//底座到肩的长度
float L2=1;//后臂长
float L3=1;//前臂长
float L4=1;//夹爪尖到前臂尖长度

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
byte bottomR[120];
byte leftR[120];
byte rightR[120];
byte clawR[120];
//摇杆控制模式下记录当前舵机角度
int bottomAngle=90, leftAngle=90, rightAngle=90, clawAngle=90;

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
  // 目标比当前大 递增扫过去
  if (current < target) {
    for (int a = current; a <= target; a++) {
      s.write(a);           // 每步转 1 度
      delay(stepDelay);     // 每步停一下，控制速度
    }
  }
  // 目标比当前小 递减扫过去
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
  moveServo(left, 180-l, spd); // 肩（左舵机装反，取反）
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

/*任务五的逆解函数
a1--底座角度
a2--肩(左)的关节角
a3--肘(右)的关节角
*/

void backanswer(float x,float y,float z,float &a1,float &a2,float &a3){
  a1=atan2(y,x)*180/PI;//底座角度
  float d=sqrt(x*x+y*y)-L4;//水平距离
  float h=z-L1;//夹爪到肩的竖直距离
  float D=sqrt(d*d+h*h);//肩到夹爪的直线距离
  a3=acos((d*d+h*h-L2*L2-L3*L3)/(2*L2*L3))*180.0/PI;//肘(右)关节角
  float arph=atan2(h,d);//肩与夹爪连线与水平面夹角
  float beta=acos((L2*L2+D*D-L3*L3)/(2*L2*D));//肩与夹爪连线和肩与肘连线之间的夹角
  a2=(arph-beta)*180.0/PI;//肩（左）的关节角
}

/*任务五第一题：画一条直线
x1,y1--起点坐标
x2,y2--终点坐标
*/

void drawline(float x1,float y1,float x2,float y2){
  float z=1;//纸面高度
  int steps=50;//分50步来画这个线
  for(int i=0;i<=steps;i++){
    //算出当前点的位置
    float x=x1+(x2-x1)*i/steps;
    float y=y1+(y2-y1)*i/steps;
    //算出逆解
    float a1,a2,a3;
    backanswer(x,y,z,a1,a2,a3);
    //移动舵机
    bottom.write(a1);
    left.write((int)(180-a2));  // 左舵机装反，取反
    right.write(a3);
    delay(20);
  }
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
  bottom.write(90);
  left.write(90);
  right.write(90);
  claw.write(60);
}

void loop() {
  // =====录制状态=====
  if(recording){
    int b=map(analogRead(A0),0,1023,0,180);
    int l=map(analogRead(A1),0,1023,0,180);
    int r=map(analogRead(A2),0,1023,0,180);
    int c=map(analogRead(A3),0,1023,0,180);
    bottom.write(b); left.write(180-l); right.write(r); claw.write(c);
    //存位置到数组中
    bottomR[number]=b;
    leftR[number]=l;
    rightR[number]=r;
    clawR[number]=c;
    number++;
    if(number>=120)number=120;
    delay(100);
  }
  // ===== 摇杆模式：速度控制（像游戏摇杆，推得越远越快，松手停）=====
  if(change){
  int x1=analogRead(A0),//底座（摇杆1的X轴）
      x2=analogRead(A3),//肩（摇杆1的Y轴）
      x3=analogRead(A2),//肘（摇杆2的X轴）
      x4=analogRead(A1);//夹爪（摇杆2的Y轴）
  // 偏移量 = 摇杆位置 − 中间(512)，正负表示方向
  int dx1=x1-512, dx2=x2-512, dx3=x3-512, dx4=x4-512;
  // 死区 ±5：中心附近不动，防止抖动
  if(dx1<-5 || dx1>5) bottomAngle += dx1/100;
  if(dx2<-5 || dx2>5) leftAngle   += dx2/100;
  if(dx3<-5 || dx3>5) rightAngle  += dx3/100;
  if(dx4<-5 || dx4>5) clawAngle   += dx4/100;
  // 限制角度在 0~180
  bottomAngle=constrain(bottomAngle,0,180);
  leftAngle=constrain(leftAngle,0,180);
  rightAngle=constrain(rightAngle,0,180);
  clawAngle=constrain(clawAngle,0,180);
  // 写到舵机
  bottom.write(bottomAngle);
  left.write(180-leftAngle);
  right.write(rightAngle);
  claw.write(clawAngle);
  delay(20);
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
        left.write(180-leftR[i]);
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
    moveServo(left, 180-l, moveDelay);
    moveServo(right, r, moveDelay);
    moveServo(claw, c, moveDelay);
    }
    }
  }
}
