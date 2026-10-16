#include <Servo.h>

/*
 * =========机械臂控制程序（运动学版主控板）============
 *
 * 【硬件连接】
 *   舵机引脚：
 *     底座             D9
 *     前臂（左臂）      D8
 *     后臂（右臂）      D7
 *     夹爪             D6
 *   摇杆引脚：
 *     A0  摇杆1 X轴 -> 控制底座
 *     A1  摇杆2 Y轴 -> 控制夹爪
 *     A2  摇杆2 X轴 -> 控制肘
 *     A3  摇杆1 Y轴 -> 控制肩
 *
 * 【串口指令一览】
 *   基础控制
 *     J                   切回摇杆模式
 *     O / S               夹爪张开 / 关闭
 *     H / L               加速 / 减速
 *     b90,l60,r30,c20     多舵机协同（底座/肘/肩/夹爪的角度）
 *   夹取动作
 *     A / B / C           夹取物体A/B/C并放置
 *   遥控板按键
 *     K1                  循环夹取
 *     K2                  开始/停止录制动作
 *     K3                  播放录制的动作
 *     K4                  回中
 *   绘图
 *     D                   画直线
 *     T                   画三角形
 *     K5                  绘制过程中：暂停/继续切换
 *     K6                  绘制过程中：取消
 *     K7                  记录一个示教点（前5次记录，第6次清空）
 *     K8                  画五点折线
 *     K9                  画五点平滑曲线
 */

//========存放数据处=========
// 引脚
const int PIN_BASE=9;
const int PIN_ELBOW=8;
const int PIN_SHOULDER=7;
const int PIN_GRIPPER=6;

// 机械臂尺寸
const float L1=61.8;            //底座到肩的长度
const float L2=80.0;            //后臂长
const float L3=80.0;            //前臂长
const float L4=46.0;            //夹爪尖到前臂尖长度

// 绘图参数
const float drawstep=0.5;   //画线时每0.5mm取一个轨迹点，画得更细腻
const int JOINT_DELAY_MS=15;    //绘图时关节细分步的延时
const float PEN_LIFT_MM=6.0;    //抬笔高度
const int POINT_PAUSE_MS=150;   //绘制时到达顶点后的统一停顿时间

Servo bottom, left, right, claw;

//摇杆控制模式下记录当前舵机角度
float bottomAngle=90.0;
float leftAngle=115.0;
float rightAngle=80.0;
float currentX=0.0;
float currentY=0.0;
float currentZ=0.0;

bool stop=false;         //用来记录是否处于停止状态
bool cancel=false;       //用来确认是否取消绘制任务

// ========== 从 main 合并的功能变量 ==========
const int frequency=150;          //录制数组容量，降低内存占用
const int recordDelay=75;         //录制采样间隔
int moveDelay=20;                 //摇杆/手动移动速度
int cycle=0;                      //循环夹取计数
int number=0;                     //录制动作数量
int number1=0;                    //五点示教已记录数量
int clawAngle=100;             //夹爪当前角度
bool change=false;                //上电后先等待串口，不自动进入摇杆模式
bool recording=false;             //是否正在录制动作

//数组变量声明（用来处理录制和播放）
byte bottomR[frequency];
byte leftR[frequency];
byte rightR[frequency];
byte clawR[frequency];
float x[5],y[5],z[5];             //用来记录五个点的坐标
float b5[5],l5[5],r5[5];          //同时记录五个点的三个舵机角

// ==========函数库==========

/*
check函数用以在绘图任务中检查暂停/继续/取消
K5 = 暂停/继续切换，K6 = 取消
*/
void check(){
  if(Serial.available()>0){
    String s=Serial.readStringUntil('\n');
    s.trim();
    if(s=="K5"){
      stop=!stop;
      if(stop){
        Serial.println("已暂停");
      }else{
        Serial.println("已继续");
      }
    }else if(s=="K6"){
      cancel=true;
      Serial.println("已取消");
    }
  }
}

//浮点角度写舵机，用 writeMicroseconds 提高精度（小于1度也能动）
void writeServo(Servo &s,float angle){
  if(&s==&bottom)       angle=constrain(angle,0.0,180.0);
  else if(&s==&left) angle=constrain(angle,0.0,137.0);
  else if(&s==&right) angle=constrain(angle,5.0,165.0);
  else                     angle=constrain(angle,0.0,180.0);
  int us=(int)(544.0+angle*1856.0/180.0);
  s.writeMicroseconds(us);
}

/*
机械臂的正解
输入三个舵机角度，算出笔尖坐标 x/y/z
*/
void foreanswer(float b,float l,float r,float &x,float &y,float &z){
  float j1=radians(b);
  float j2=radians((18.0/17.0)*(r-165.0));
  float j3=radians(0.559*(115.0-l));
  float reach=L2*cos(j2)+L3*cos(j2+j3)+L4;
  x=reach*cos(j1);
  y=reach*sin(j1);
  z=L1+L2*sin(j2)+L3*sin(j2+j3);
}

/*
机械臂的逆解
输入笔尖坐标 x/y/z，算出三个舵机角度 b/l/r
目标点不可达时返回 false
*/
bool backanswer(float x,float y,float z,float &b,float &l,float &r){
  float R=sqrt(x*x+y*y);
  float Dh=R-L4;
  float h=z-L1;
  float D=sqrt(Dh*Dh+h*h);

  if(Dh<0.0 || D>158.0 || D<5.0) return false;

  float cosJ3=(D*D-L2*L2-L3*L3)/(2.0*L2*L3);
  if(cosJ3>1.0 || cosJ3<-1.0) return false;
  float j3=acos(cosJ3);

  float gamma=atan2(h,Dh);
  float cosBeta=(L2*L2+D*D-L3*L3)/(2.0*L2*D);
  if(cosBeta>1.0 || cosBeta<-1.0) return false;
  float beta=acos(cosBeta);
  float j2=gamma-beta;

  b=degrees(atan2(y,x));
  l=115.0-degrees(j3)/0.559;
  r=165.0+degrees(j2)*17.0/18.0;

  if(b<0.0 || b>180.0) return false;
  if(l<0.0 || l>137.0) return false;
  if(r<5.0 || r>165.0) return false;
  return true;
}

/*
三个运动舵机同步细分到目标角
先算笛卡尔点，再调用这里做关节角平滑，避免每个点之间突然跳舵机
*/
void moveJointsSmooth(float b,float l,float r){
  b=constrain(b,0.0,180.0);
  l=constrain(l,0.0,137.0);
  r=constrain(r,5.0,165.0);

  float b0=bottomAngle, l0=leftAngle, r0=rightAngle;
  float db=b-b0, dl=l-l0, dr=r-r0;
  float maxDelta=max(max(fabs(db),fabs(dl)),fabs(dr));
  int steps=max(1,(int)(maxDelta*2.0));

  for(int i=1;i<=steps;i++){
    check();
    if(cancel) return;
    while(stop && !cancel){
      check();
      delay(10);
    }
    writeServo(bottom,b0+db*i/steps);
    writeServo(left,l0+dl*i/steps);
    writeServo(right,r0+dr*i/steps);
    delay(JOINT_DELAY_MS);
  }

  bottomAngle=b;
  leftAngle=l;
  rightAngle=r;
}

/*
抬笔/落笔用关节空间移动，不经过笛卡尔路径，避免甩到极限位置
返回 false 表示目标点不可达
*/
bool movearm(float x,float y,float z){
  float b,l,r;
  if(!backanswer(x,y,z,b,l,r)){
    return false;
  }
  moveJointsSmooth(b,l,r);
  if(cancel) return false;
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
  return true;
}

//笛卡尔插值移动：x,y,z 线性变化，用于画线时保证轨迹平滑
void moveline(float x,float y,float z){
  float dist=sqrt((x-currentX)*(x-currentX)+(y-currentY)*(y-currentY)+(z-currentZ)*(z-currentZ));
  int steps=max(1,(int)(dist/drawstep));
  float b,l,r;

  for(int i=1;i<=steps;i++){
    float px=currentX+(x-currentX)*i/steps;
    float py=currentY+(y-currentY)*i/steps;
    float pz=currentZ+(z-currentZ)*i/steps;
    if(backanswer(px,py,pz,b,l,r)){
      moveJointsSmooth(b,l,r);
      if(cancel) return;
    }else{
      Serial.println("轨迹点不可达");
      return;
    }
  }

  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
}

/*
用运动学画一条直线
起点先抬笔，落笔后笛卡尔插值画到终点，最后抬笔
*/
void drawline(float x1,float y1,float z1,float x2,float y2,float z2){
  claw.write(100);
  clawAngle=100;
  if(!movearm(x1,y1,z1-PEN_LIFT_MM)){
    Serial.println("起点不可达");
    return;
  }
  if(cancel) return;
  if(!movearm(x1,y1,z1)){
    Serial.println("落笔点不可达");
    return;
  }
  if(cancel) return;
  moveline(x2,y2,z2);
  if(cancel) return;
  if(!movearm(x2,y2,z2-PEN_LIFT_MM)){
    Serial.println("抬笔点不可达");
    return;
  }
}

//用运动学绘制三角形
void triangle(){
  float x[3]={76.87,34.99,71.50};
  float y[3]={138.68,180.00,177.10};
  float z[3]={-42.45,-11.73,2.00};
  float aUpB=61.0, aUpL=76.0, aUpR=99.0;
  float aDownB=61.0, aDownL=56.0, aDownR=109.0;

  claw.write(100);
  clawAngle=100;
  moveJointsSmooth(aUpB,aUpL,aUpR);
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
  if(cancel) return;
  moveJointsSmooth(aDownB,aDownL,aDownR);
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
  if(cancel) return;

  moveline(x[1],y[1],z[1]);
  delay(POINT_PAUSE_MS);
  moveline(x[2],y[2],z[2]);
  delay(POINT_PAUSE_MS);
  moveline(x[0],y[0],z[0]);
  delay(POINT_PAUSE_MS);

  moveJointsSmooth(aUpB,aUpL,aUpR);
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
}

//回到初始位置
void home(){
  stop=false;
  cancel=false;
  claw.write(100);
  clawAngle=100;
  moveJointsSmooth(90.0,115.0,80.0);
  bottomAngle=90.0;
  leftAngle=115.0;
  rightAngle=80.0;
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
}

/*
打印当前笔尖坐标和四个舵机角度
测数据用：把机械臂摇到目标位置后，串口发 P 就能看到坐标和角度
*/
void printPose(){
  foreanswer(bottomAngle,leftAngle,rightAngle,currentX,currentY,currentZ);
  Serial.print("坐标 x=");
  Serial.print(currentX);
  Serial.print(" y=");
  Serial.print(currentY);
  Serial.print(" z=");
  Serial.println(currentZ);
  Serial.print("角度 底座=");
  Serial.print(bottomAngle);
  Serial.print(" 肘=");
  Serial.print(leftAngle);
  Serial.print(" 肩=");
  Serial.println(rightAngle);
  Serial.print(" 夹爪=");
  Serial.println(clawAngle);
}

/*
为完成多舵机协同控制，并防止上位机输入数据的格式没有按顺序
用findangle函数来返回舵机所对应角度
例如输入 "b90,l60,r30,c20"，findangle(s,'b') 返回 90
*/
int findangle(String s,char label){
  int num=s.indexOf(label);
  if(num==-1) return -1;
  return s.substring(++num).toInt();
}

/*
用moveServo函数来使机械臂活动流畅，不卡顿
从当前角度平滑移动到目标角度，stepDelay 控制速度
stepDelay 越小，移动越快
*/
void moveServo(Servo &s, int target, int stepDelay){
  int current=0;
  if(&s==&bottom) current=(int)bottomAngle;
  else if(&s==&left) current=(int)leftAngle;
  else if(&s==&right) current=(int)rightAngle;
  else current=clawAngle;

  if(current<target){
    for(int a=current;a<=target;a++){
      writeServo(s,(float)a);
      delay(stepDelay);
    }
  }else if(current>target){
    for(int a=current;a>=target;a--){
      writeServo(s,(float)a);
      delay(stepDelay);
    }
  }

  if(&s==&bottom) bottomAngle=target;
  else if(&s==&left) leftAngle=target;
  else if(&s==&right) rightAngle=target;
  else if(&s==&claw) clawAngle=target;
}

/*
用pole函数来完成摇杆控制
偏移量越远，角度变化越快
死区 ±5，中心附近不动作，防止抖动
*/
void pole(){
  int x1=analogRead(A0); //底座
  int x2=analogRead(A3); //肩
  int x3=analogRead(A2); //肘
  int x4=analogRead(A1); //夹爪
  int dx1=x1-512, dx2=x2-512, dx3=x3-512, dx4=x4-512;

  if(dx1<-5 || dx1>5) bottomAngle += dx1/100;
  if(dx2<-5 || dx2>5) leftAngle += dx2/100;
  if(dx3<-5 || dx3>5) rightAngle += dx3/100;
  if(dx4<-5 || dx4>5) clawAngle += dx4/100;

  bottomAngle=constrain(bottomAngle,0,180);
  leftAngle=constrain(leftAngle,0,137);
  rightAngle=constrain(rightAngle,5,165);
  clawAngle=constrain(clawAngle,0,100);

  writeServo(bottom,bottomAngle);
  writeServo(left,leftAngle);
  writeServo(right,rightAngle);
  claw.write(clawAngle);
}

/*
用moveTo函数把 4 个舵机同时移动到指定角度
其中 b=底座，l=肘，r=肩，c=夹爪，spd=延时
*/
void moveTo(int b,int l,int r,int c,int spd){
  int c1=(int)bottomAngle, c2=(int)leftAngle, c3=(int)rightAngle, c4=clawAngle;
  float maxDelta=max(max(fabs(b-c1),fabs(l-c2)),max(fabs(r-c3),fabs(c-c4)));
  int steps=max(1,(int)(maxDelta*2.0));

  for(int i=1;i<=steps;i++){
    float t=(float)i/steps;
    check();
    if(cancel) return;
    while(stop && !cancel){
      check();
      delay(10);
    }
    writeServo(bottom,c1+(b-c1)*t);
    writeServo(left,c2+(l-c2)*t);
    writeServo(right,c3+(r-c3)*t);
    claw.write((int)(c4+(c-c4)*t));
    delay(max(1,spd/2));
  }

  bottomAngle=b;
  leftAngle=l;
  rightAngle=r;
  clawAngle=c;
}

//按固定时长播放一帧录制动作，避免角度差大时忽快忽慢
void playFrame(int b,int l,int r,int c,int durationMs){
  int c1=(int)bottomAngle, c2=(int)leftAngle, c3=(int)rightAngle, c4=clawAngle;
  int stepMs=5;
  int steps=max(1,durationMs/stepMs);

  for(int i=1;i<=steps;i++){
    float t=(float)i/steps;
    check();
    if(cancel) return;
    while(stop && !cancel){
      check();
      delay(10);
    }
    writeServo(bottom,c1+(b-c1)*t);
    writeServo(left,c2+(l-c2)*t);
    writeServo(right,c3+(r-c3)*t);
    claw.write((int)(c4+(c-c4)*t));
    delay(stepMs);
  }

  bottomAngle=b;
  leftAngle=l;
  rightAngle=r;
  clawAngle=c;
}

/*
夹取动作 A/B/C
内部都是调用moveTo函数来完成移动
*/
void grabA(){
  moveTo(10,115,80,30,moveDelay);
  moveTo(10,49,156,30,moveDelay);
  moveTo(10,49,156,100,moveDelay);
  moveTo(10,115,80,100,moveDelay);
  moveTo(62,115,80,100,moveDelay);
  moveTo(62,49,156,100,moveDelay);
  moveTo(62,49,156,30,moveDelay);
  moveTo(62,115,80,30,moveDelay);
  home();
}

void grabB(){
  moveTo(168,137,91,30,moveDelay);
  moveTo(168,76,103,30,moveDelay);
  moveTo(168,76,103,100,moveDelay);
  moveTo(168,137,91,100,moveDelay);
  moveTo(77,137,91,100,moveDelay);
  moveTo(77,76,103,100,moveDelay);
  moveTo(77,76,103,30,moveDelay);
  moveTo(77,137,91,30,moveDelay);
  home();
}

void grabC(){
  moveTo(2,115,80,30,moveDelay);
  moveTo(2,23,148,30,moveDelay);
  moveTo(2,23,148,100,moveDelay);
  moveTo(2,115,80,100,moveDelay);
  moveTo(178,115,80,100,moveDelay);
  moveTo(178,23,148,100,moveDelay);
  moveTo(178,23,148,30,moveDelay);
  moveTo(178,115,80,30,moveDelay);
  home();
}

/*
用record函数来记录五点示教的坐标
按一次记录一个点，最多记录五个
按第六次会清空数据，重新开始记录
*/
void record(){
  if(number1<5){
    foreanswer(bottomAngle,leftAngle,rightAngle,x[number1],y[number1],z[number1]);
    b5[number1]=bottomAngle;
    l5[number1]=leftAngle;
    r5[number1]=rightAngle;
    Serial.println(x[number1]);
    Serial.println(y[number1]);
    Serial.println(z[number1]);
    number1++;
    Serial.print("已记录第");
    Serial.print(number1);
    Serial.println("个点");
    if(number1==5){
      Serial.println("已记录满5个点，再按一次清空");
    }
  }else{
    for(int i=0;i<5;i++){
      x[i]=0; y[i]=0; z[i]=0;
      b5[i]=0; l5[i]=0; r5[i]=0;
    }
    number1=0;
    Serial.println("已清空记录，可以重新示教");
  }
}

/*
根据某个示教点的三个舵机角，算出它的抬笔舵机角
抬笔时肘加一点、肩收一点，让笔先抬起来并往回收
*/
void fivePointUpAngles(int i,float &b,float &l,float &r){
  b=b5[i];
  l=constrain(l5[i]+20.0,0.0,137.0);
  r=constrain(r5[i]-10.0,5.0,165.0);
}

//在固定纸面高度 z0 上做一段 Catmull-Rom 平滑曲线
void curveSegment(int i0,int i1,int i2,int i3,float z0){
  int steps=40;
  for(int i=0;i<=steps && !cancel;i++){
    float t=(float)i/steps;
    float t2=t*t;
    float t3=t2*t;
    float px=0.5*((2*x[i1])+(-x[i0]+x[i2])*t+(2*x[i0]-5*x[i1]+4*x[i2]-x[i3])*t2+(-x[i0]+3*x[i1]-3*x[i2]+x[i3])*t3);
    float py=0.5*((2*y[i1])+(-y[i0]+y[i2])*t+(2*y[i0]-5*y[i1]+4*y[i2]-y[i3])*t2+(-y[i0]+3*y[i1]-3*y[i2]+y[i3])*t3);
    float b,l,r;
    if(!backanswer(px,py,z0,b,l,r)){
      Serial.println("曲线点不可达");
      return;
    }
    moveJointsSmooth(b,l,r);
    if(cancel) return;
  }
}

/*
五点示教--折线版
fiveline函数接收record记录到的点，并把它们两两连接
*/
void fiveline(){
  if(number1!=5){
    Serial.println("点的个数不够");
    return;
  }

  stop=false;
  cancel=false;
  float paperZ=z[0];   //用第一个点的纸面高度，避免绘图过程中笔尖上下漂移

  if(!movearm(x[0],y[0],paperZ-PEN_LIFT_MM)){
    Serial.println("起点不可达");
    return;
  }
  if(cancel) return;
  if(!movearm(x[0],y[0],paperZ)){
    Serial.println("落笔点不可达");
    return;
  }
  if(cancel) return;
  delay(POINT_PAUSE_MS);

  for(int i=1;i<number1 && !cancel;i++){
    Serial.print(F("折线第"));
    Serial.print(i);
    Serial.print(F("段：点"));
    Serial.print(i-1);
    Serial.print(F(" -> 点"));
    Serial.println(i);
    moveline(x[i],y[i],paperZ);
    delay(POINT_PAUSE_MS);
  }

  if(cancel){
    cancel=false;
    home();
    Serial.println("已取消并回中");
  }else{
    if(!movearm(x[4],y[4],paperZ-PEN_LIFT_MM)){
      Serial.println("抬笔点不可达");
    }
    home();
    Serial.println("折线已画好");
  }
}

/*
五点示教--曲线版
把5个点用 Catmull-Rom 曲线平滑地连起来，共4段
第一段和最后一段分别把首尾点重复一次，保证整条曲线平滑
*/
void fivecurve(){
  if(number1!=5){
    Serial.println("点的个数不够");
    return;
  }

  stop=false;
  cancel=false;
  float paperZ=z[0];

  if(!movearm(x[0],y[0],paperZ-PEN_LIFT_MM)){
    Serial.println("起点不可达");
    return;
  }
  if(cancel) return;
  if(!movearm(x[0],y[0],paperZ)){
    Serial.println("落笔点不可达");
    return;
  }
  if(cancel) return;
  if(!cancel) delay(POINT_PAUSE_MS);

  Serial.println(F("曲线第1段：点0 -> 点1"));
  curveSegment(0,0,1,2,paperZ);
  if(!cancel) delay(POINT_PAUSE_MS);
  Serial.println(F("曲线第2段：点1 -> 点2"));
  curveSegment(0,1,2,3,paperZ);
  if(!cancel) delay(POINT_PAUSE_MS);
  Serial.println(F("曲线第3段：点2 -> 点3"));
  curveSegment(1,2,3,4,paperZ);
  if(!cancel) delay(POINT_PAUSE_MS);
  Serial.println(F("曲线第4段：点3 -> 点4"));
  curveSegment(2,3,4,4,paperZ);
  if(!cancel) delay(POINT_PAUSE_MS);

  if(cancel){
    cancel=false;
    home();
    Serial.println("已取消并回中");
  }else{
    if(!movearm(x[4],y[4],paperZ-PEN_LIFT_MM)){
      Serial.println("抬笔点不可达");
    }
    home();
    Serial.println("曲线已画好");
  }
}

//=======初始化=========
void setup(){
  //串口初始化
  Serial.begin(9600);
  Serial.setTimeout(50);
  //接引脚
  bottom.attach(PIN_BASE);
  left.attach(PIN_ELBOW);
  right.attach(PIN_SHOULDER);
  claw.attach(PIN_GRIPPER);
  //回中并等待串口
  home();
  Serial.println(F("J摇杆 O/S夹爪 H/L速度 b/l/r/c手动 A/B/C夹取 K1循环 K2/K3录播 K4回中 D直线 T三角形 K5/K6暂停取消 K7记录 K8折线 K9曲线"));
}

//=========主循环========
void loop(){
  // =====录制状态=====
  if(recording){
    pole();
    //存位置到数组中，先判断数组有没有满，满了就不再写，防止越界
    if(number<frequency){
      bottomR[number]=(byte)bottomAngle;
      leftR[number]=(byte)leftAngle;
      rightR[number]=(byte)rightAngle;
      clawR[number]=(byte)clawAngle;
      number++;
    }else{
      recording=false;
      Serial.println("录制空间已满，自动停止");
    }
    delay(recordDelay);
  // =====摇杆模式=====
  }else if(change){
    pole();
    delay(20);
  }

  //======串口模式=======
  if(Serial.available()>0){
    //串口输入存储
    String cmd=Serial.readStringUntil('\n');
    cmd.trim();
    //接收到J切回摇杆模式
    if(cmd=="J"){
      change=true;
      Serial.println("摇杆模式");
    }else{
      //切回串口模式
      change=false;
      Serial.println("串口模式");

      //接收O-张开
      if(cmd=="O"){
        moveServo(claw,30,moveDelay);
        Serial.println("open");
      //接收S-关闭
      }else if(cmd=="S"){
        moveServo(claw,100,moveDelay);
        Serial.println("close");
      //接收H-加速
      }else if(cmd=="H"){
        moveDelay-=2;
        if(moveDelay<2) moveDelay=2;
        Serial.print("延时=");
        Serial.println(moveDelay);
      //接收L-减速
      }else if(cmd=="L"){
        moveDelay+=2;
        if(moveDelay>50) moveDelay=50;
        Serial.print("延时=");
        Serial.println(moveDelay);
      //夹取动作 A/B/C
      }else if(cmd=="A"){
        grabA();
        Serial.println("A finish");
      }else if(cmd=="B"){
        grabB();
        Serial.println("B finish");
      }else if(cmd=="C"){
        grabC();
        Serial.println("C finish");
      //循环夹取
      }else if(cmd=="K1"){
        cycle++;
        if(cycle>3) cycle=1;
        if(cycle==1) grabA();
        else if(cycle==2) grabB();
        else grabC();
        Serial.println("夹取完毕");
      //开始/停止录制
      }else if(cmd=="K2"){
        recording=!recording;
        if(recording){
          number=0;
          Serial.println("开始录制");
        }else{
          Serial.println("停止录制");
        }
      //播放录制的动作
      }else if(cmd=="K3"){
        stop=false;
        cancel=false;
        Serial.println("开始播放");
        for(int i=0;i<number && !cancel;i++){
          playFrame(bottomR[i],leftR[i],rightR[i],clawR[i],recordDelay);
        }
        Serial.println("播放完毕");
      //回中
      }else if(cmd=="K4"){
        change=true;
        stop=false;
        cancel=false;
        home();
        Serial.println("已回中");
      //画直线
      }else if(cmd=="D"){
        stop=false;
        cancel=false;
        drawline(0.0,191.90,8.16,0.0,157.66,-42.41);
        if(cancel){
          cancel=false;
          home();
          Serial.println("已取消并回中");
        }else{
          home();
          Serial.println("直线已画好");
        }
      //画三角形
      }else if(cmd=="T"){
        stop=false;
        cancel=false;
        triangle();
        if(cancel){
          cancel=false;
          home();
          Serial.println("已取消并回中");
        }else{
          home();
          Serial.println("三角形已画好");
        }
      //打印当前坐标和角度（测数据用）
      }else if(cmd=="P"){
        printPose();
        change=true;              //打印完切回摇杆，继续移动
        Serial.println("摇杆模式");
      //记录五点
      }else if(cmd=="K7"){
        record();
        change=true;              //记录完切回摇杆，继续移动笔尖
        Serial.println("摇杆模式");
      //画折线
      }else if(cmd=="K8"){
        fiveline();
      //画曲线
      }else if(cmd=="K9"){
        fivecurve();
      }else{
      //用 b/l/r/c 实现舵机控制
        int b=findangle(cmd,'b');
        int l=findangle(cmd,'l');
        int r=findangle(cmd,'r');
        int c=findangle(cmd,'c');
        if(b!=-1) moveServo(bottom,b,moveDelay);
        if(l!=-1) moveServo(left,l,moveDelay);
        if(r!=-1) moveServo(right,r,moveDelay);
        if(c!=-1) moveServo(claw,c,moveDelay);
      }
    }
  }
}
