#include<Servo.h>


/*
 * =========机械臂控制程序（主控板）============
 *
 * 【硬件连接】
 *   舵机引脚：
 *     底座             D9
 *     肩（左臂）        D8
 *     肘（右臂）        D7
 *     夹爪             D6
 *   摇杆引脚：
 *     A0  摇杆1 X轴 → 控制底座
 *     A1  摇杆2 Y轴 → 控制夹爪
 *     A2  摇杆2 X轴 → 控制肘
 *     A3  摇杆1 Y轴 → 控制肩
 *
 * 【串口指令一览】
 *   任务一·基础控制
 *     J                   切回摇杆模式
 *     O / S               夹爪张开 / 关闭
 *     H / L               加速 / 减速
 *     b90,l60,r30,c20     多舵机协同（底座/肩/肘/夹爪的角度）
 *   任务二·夹取动作
 *     A / B / C           夹取物体A/B/C并放置
 *   任务三·遥控板按键
 *     K1                  循环夹取
 *     K2                  开始/停止录制动作
 *     K3                  播放录制的动作
 *     K4                  回中
 *   任务五·绘图
 *     D                   画直线
 *     T                   画三角形
 *     K5                  绘制过程中：暂停/继续切换
 *     K6                  绘制过程中：取消
 *     K7                  记录一个示教点（前5次记录，第6次清空）
 *     K8                  画五点折线
 *     K9                  画五点平滑曲线
 */


//========存放数据处=========
//引脚
const int bottom_yijiao=9;  //底座
const int left_yijiao=8;    //左边
const int right_yijiao=7;   //右边
const int claw_yijiao=6;    //夹爪
const int frequency=175;    //用于任务3录制的数组
const int recordDelay=70;   //录制时每两个点之间的间隔（毫秒）
//任务五---机械臂数据
float L1=20;                //底座到肩的长度
float L2=80;                //后臂长
float L3=80;                //前臂长
float L4=40;                //夹爪尖到前臂尖长度
float inpaper=5;            //纸面高度
float outpaper=40;          //抬笔时高度


//========变量声明=======
int moveDelay;              //每步移动的延时（越小越快），H/L 调速用
int angle;                  //角度
int cycle=0;                //用来记录任务三按键1的点击次数
int number=0;               //用来记录任务三按键2录制时位置个数
int number1=0;              //用来记录五点视教已经记录的位置数
bool change=true;           //模式开关：true=摇杆模式，false=串口指令模式
bool recording=false;       //用来记录是否在录制中
bool stop=false;            //用来记录是否处于停止状态
bool cancel=false;          //用来确认是否取消绘制任务
Servo bottom;               //底座
Servo left;                 //左边
Servo right;                //右边
Servo claw;                 //夹爪
//数组变量声明（用来处理录制和播放）
byte bottomR[frequency];
byte leftR[frequency];
byte rightR[frequency];
byte clawR[frequency];
float x[5],y[5],z[5];//用来记录五个点的坐标
//摇杆控制模式下记录当前舵机角度
int bottomAngle=90, leftAngle=90, rightAngle=90, clawAngle=60;


//=========函数库=========

/*
为完成多舵机协同控制，并防止上位机输入数据的格式没有按顺序
用findangle函数来返回舵机所对应角度
例如输入 "b90,l60,r30,c20"，findangle(s,'b') 返回 90
*/
int findangle(String s,char label){
  int num=s.indexOf(label);//找到编号的位置
  if(num==-1){
    return -1;//没找到返回 -1，表示这个舵机不动
  }
  else return s.substring(++num).toInt();
}

/*
用moveServo函数来使机械臂活动流畅 不卡顿
从当前角度平滑移动到目标角度，stepDelay 控制速度
原理：不直接 write(目标)，而是每次加/减 1 度，每步 delay 一下
stepDelay 越小 移动越快
*/
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

/*
用pole函数来完成任务一要求的摇杆控制舵机
摇杆控制逻辑：偏移的越远，速度越快
原理为依赖偏移量 给角度加/减偏移量来实现速度控制
*/
void pole(){
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
  leftAngle=constrain(leftAngle,0,125);
  rightAngle=constrain(rightAngle,5,165);
  clawAngle=constrain(clawAngle,0,100);
  // 写到舵机
  bottom.write(bottomAngle);
  left.write(leftAngle);
  right.write(rightAngle);
  claw.write(clawAngle);
}

/*
用moveTo函数把 4 个舵机同时移动到指定角度
其中 b-底座 l-左边 r-右边 c-夹爪 spd-延时
*/
void moveTo(int b, int l, int r, int c, int spd) {
  moveServo(bottom, b, spd);
  moveServo(left, l, spd);
  moveServo(right, r, spd);
  moveServo(claw, c, spd);
}

/*
任务二夹取动作
A B C 函数使用逻辑都为调用moveTo函数来完成移动
*/
void grabA(){
  moveTo(90,90,80,100,moveDelay);
}
void grabB(){
  moveTo(90,90,80,100,moveDelay);
}

void grabC(){
  moveTo(90,90,80,100,moveDelay);
}

//      ========任务五·运动学=========
//a1--底座角度 a2--肩(左)的关节角 a3--肘(右)的关节角
/*
机械臂的正解
输入三个舵机角度，算出笔尖坐标 x/y/z
*/
void foreanswer(float a1,float a2,float a3,float &x,float &y,float &z){
  float a11=a1/180*PI;
  float a22=a2/180*PI;
  float a33=a3/180*PI;
  x=(L2*cos(a22)+L3*cos(a22+a33)+L4)*cos(a11);
  y=(L2*cos(a22)+L3*cos(a22+a33)+L4)*sin(a11);
  z=L1+L2*sin(a22)+L3*sin(a22+a33);
}

/*
机械臂的逆解
输入笔尖坐标 x/y/z，算出三个舵机角度 a1/a2/a3
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

//     ==========任务五·绘制==========
/*
check函数用以在子任务三中检查是否有暂停/继续/取消
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
      } else {
        Serial.println("已继续");
      }
    } else if(s=="K6"){
      cancel=true;
      Serial.println("已取消");
    }
  }
}

/*
用movearm函数把笔尖移动到指定坐标(x,y,z)
内部先做逆解，再把结果写到三个舵机
*/
void movearm(float x,float y,float z){
  float a1,a2,a3;
  backanswer(x,y,z,a1,a2,a3);
  bottom.write(a1);
  left.write(a2);
  right.write(a3);
  delay(60);
}

/*
用drawline函数来画一条直线
x1,y1为起点 x2,y2为终点
画的过程支持暂停/继续/取消
*/
void drawline(float x1,float y1,float x2,float y2){
  int steps=50;//分50步来画这个线
  for(int i=0;i<=steps&&cancel==false;i++){
    //当处于停止且没取消时反复检查
    while(stop==true&&cancel==false){
      check();
      delay(20);
    }
    check();//防止未进入while循环
    //算出当前点的位置
    float x=x1+(x2-x1)*i/steps;
    float y=y1+(y2-y1)*i/steps;
    movearm(x,y,inpaper);
  }
}

//绘制三角形
void triangle(float x1,float y1,float x2,float y2,float x3,float y3){
  movearm(x1,y1,outpaper);
  drawline(x1,y1,x2,y2);
  drawline(x2,y2,x3,y3);
  drawline(x3,y3,x1,y1);
  movearm(x1,y1,outpaper);
}

/*
用curveSegment函数来画一段平滑曲线（Catmull-Rom）
曲线在 p1 和 p2 之间，p0和p3用来保证曲线的平滑性
由于三次函数的平滑性较好 可设y=a*x*x*x+b*x*x+c*x+d
那么为了保证平滑性 需保证在p1处的导函数平行于p0,p2连线
p2处的导函数平行于p1,p3连线
据此解出 a,b,c,d
*/
void curveSegment(float p0x,float p0y,float p1x,float p1y,float p2x,float p2y,float p3x,float p3y){
  int steps=40;
  for(int i=0;i<=steps && cancel==false;i++){
    while(stop==true && cancel==false){
      check();
      delay(20);
    }
    check();
    float t=(float)i/steps;
    float t2=t*t;
    float t3=t2*t;
    float x=0.5*((2*p1x)+(-p0x+p2x)*t+(2*p0x-5*p1x+4*p2x-p3x)*t2+(-p0x+3*p1x-3*p2x+p3x)*t3);
    float y=0.5*((2*p1y)+(-p0y+p2y)*t+(2*p0y-5*p1y+4*p2y-p3y)*t2+(-p0y+3*p1y-3*p2y+p3y)*t3);
    movearm(x,y,inpaper);
  }
}

/*
用printPose函数打印当前笔尖坐标和四个舵机角度
测数据用：把机械臂摇到目标位置后，串口发 P 就能看到坐标和角度
*/
void printPose(){
  float px,py,pz;
  foreanswer(bottomAngle,leftAngle,rightAngle,px,py,pz);
  Serial.print("坐标 x=");
  Serial.print(px);
  Serial.print(" y=");
  Serial.print(py);
  Serial.print(" z=");
  Serial.println(pz);
  Serial.print("角度 底座=");
  Serial.print(bottomAngle);
  Serial.print(" 肩=");
  Serial.print(leftAngle);
  Serial.print(" 肘=");
  Serial.print(rightAngle);
  Serial.print(" 夹爪=");
  Serial.println(clawAngle);
}

/*
用record函数来记录五点示教的坐标
按一次记录一个坐标 最多记录五个 
若按第六次会清空数据，重新开始记录
*/
void record(){
  if(number1<5){
    foreanswer(bottomAngle,leftAngle,rightAngle,x[number1],y[number1],z[number1]);
    Serial.println(x[number1]);
    Serial.println(y[number1]);
    Serial.println(z[number1]);
    number1++;
    Serial.print("已记录第");
    Serial.print(number1);
    Serial.println("个点");
  if(number1==5){
      Serial.println("已记录满5个点再按一次清空");
    }
  }else{
    //已经满5个点，再点一次就清空，重新开始
    for(int i=0;i<5;i++){
      x[i]=0;y[i]=0;z[i]=0;
    }
    number1=0;
    Serial.println("已清空记录，可以重新示教");
  }
}

/*
五点示教--折线版
fiveline函数调用record函数
接收其记录的坐标 并将其两两连接
*/
void fiveline(){
  if(number1!=5){
    Serial.println("点的个数不够");
    return;
  }else{
    stop=false;
    cancel=false;
    //移动到正上方
    movearm(x[0],y[0],outpaper);
    //依次连线
    for(int i=0;i<number1-1&&cancel==false;i++){
      drawline(x[i],y[i],x[i+1],y[i+1]);
      delay(20);
    }
    //画完抬笔
    movearm(x[number1-1],y[number1-1],outpaper);
    Serial.println("折线已经画好");
  }
}

/*
五点示教--曲线版
把5个点用 Catmull-Rom 曲线平滑地连起来（共4段）
第一段和最后一段分别把首尾点重复一次，保证整条曲线平滑
*/
void fivecurve(){
  if(number1!=5){
    Serial.println("点的个数不够");
    return;
  }
  stop=false;
  cancel=false;
  movearm(x[0],y[0],outpaper);
  curveSegment(x[0],y[0], x[0],y[0], x[1],y[1], x[2],y[2]);
  curveSegment(x[0],y[0], x[1],y[1], x[2],y[2], x[3],y[3]);
  curveSegment(x[1],y[1], x[2],y[2], x[3],y[3], x[4],y[4]);
  curveSegment(x[2],y[2], x[3],y[3], x[4],y[4], x[4],y[4]);
  movearm(x[4],y[4],outpaper);
  Serial.println("曲线已画好");
}

//=======初始化=========
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
  right.write(80);
  claw.write(100);
}

//=========主循环========
void loop() {
  // =====录制状态=====
  if(recording){
    pole();
    //存位置到数组中，先判断数组有没有满，满了就不再写，防止越界
    if(number<frequency){
      bottomR[number]=bottomAngle;
      leftR[number]=leftAngle;
      rightR[number]=rightAngle;
      clawR[number]=clawAngle;
      number++;
    }
    else{
      recording=false;
      Serial.println("录制空间已满，自动停止");
    }
    delay(recordDelay);
  }
  // ===== 摇杆模式=====
  else if(change){
    pole();
    delay(20);
  }
  //======串口模式=======
  if(Serial.available()>0){
    //串口输入存储
    String input=Serial.readStringUntil('\n');
    input.trim();
    //接收到J切回摇杆模式
    if(input=="J"){
      change=true;
      Serial.println("摇杆模式");
    }else{
      change=false;//切回串口模式
      Serial.println("串口模式");
    //接收O-张开
    if(input=="O"){
      moveServo(claw, 30, moveDelay);
      Serial.println("open");
    }
    //接收S-关闭
    else if(input=="S"){
      moveServo(claw, 100, moveDelay);
      Serial.println("close");
    }
    //接收H-加速
    else if(input=="H"){
      moveDelay-=2;
      if(moveDelay<2){
        moveDelay=2;
      }
      Serial.print("延时=");
      Serial.println(moveDelay);
    }
    //接收L-减速
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

    //==========任务三==========
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
        delay(recordDelay);
      }
      Serial.println("播放完毕");
    }
    //回中
    else if(input=="K4"){
      moveTo(90,90,80,100,moveDelay);
    }
    //=======任务五========
    //画直线
    else if(input=="D"){
      stop=false;
      cancel=false;
      drawline(10,10,-60,-60);
      Serial.println("直线已画好");
    }
    //画三角形
    else if(input=="T"){
      stop=false;
      cancel=false;
      triangle(10,10,60,10,35,50);
      Serial.println("三角形已画好");
    }
    //打印当前坐标和角度（测数据用）
    else if(input=="P"){
      printPose();
      change=true;              //打印完切回摇杆，继续移动
      Serial.println("摇杆模式");
    }
    //记录五点
    else if(input=="K7"){
      record();
      change=true;              //记录完切回摇杆，继续移动笔尖
      Serial.println("摇杆模式");
    }
    //画折线
    else if(input=="K8"){
      fiveline();
    }
    //画曲线
    else if(input=="K9"){
      fivecurve();
    }
    else{
    //用 b/l/r/c 实现舵机控制
    int b=findangle(input,'b');
    int l=findangle(input,'l');
    int r=findangle(input,'r');
    int c=findangle(input,'c');
    if(b!=-1) moveServo(bottom, b, moveDelay);
    if(l!=-1) moveServo(left, l, moveDelay);
    if(r!=-1) moveServo(right, r, moveDelay);
    if(c!=-1) moveServo(claw, c, moveDelay);
    }
    }
  }
}
