#include<Servo.h>


/*
 * =========机械臂控制程序（主控板）============
 *
 * 【硬件连接】
 *   舵机引脚：
 *     底座             D9
 *     前臂（左臂）      D8
 *     后臂（右臂）      D7
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
const int frequency=210;    //用于任务3录制的数组
const int recordDelay=60;   //录制时每两个点之间的间隔（毫秒）
//任务五---机械臂数据
float L1=61.8;                //底座到肩的长度
float L2=80;                //后臂长
float L3=80;                //前臂长
float L4=46;                //夹爪尖到前臂尖长度
float inpaper=-46.25;            //夹爪尖落笔高度（笔尖贴纸时测得的 z）
float outpaper=-62.0;            //夹爪尖抬笔高度（笔尖抬离纸面）
const float drawstep=1.0;    //画线时每1mm取一个轨迹点
const int drawdelay=30;//画线时每个关节细分步的延时
const float thigh=114.0;    //三角形落笔高度，越小表示笔抬得越高
const float thighA=120.0;  //A点单独落笔高度，因为A处纸面更低
const float thighout=70.0;       //三角形抬笔高度，越小表示笔抬得越高
const int pointpause=200;        //绘制时到达顶点后的统一停顿时间


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
float b5[5],l5[5],r5[5];//同时记录五个点的三个舵机角
//摇杆控制模式下记录当前舵机角度
int bottomAngle=90, leftAngle=115, rightAngle=80, clawAngle=100;


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
  // 同步全局角度变量，防止之后切回摇杆时跳变
  if (&s == &bottom) bottomAngle = target;
  else if (&s == &left) leftAngle = target;
  else if (&s == &right) rightAngle = target;
  else if (&s == &claw) clawAngle = target;
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
  leftAngle=constrain(leftAngle,0,137);
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
  int c1=bottom.read(), c2=left.read(), c3=right.read(), c4=claw.read();
  //四个舵机同时逐度移动：谁的角度差大，就一起分多少步
  int steps=max(max(abs(b-c1), abs(l-c2)), max(abs(r-c3), abs(c-c4)));
  if(steps>0){
    for(int i=1;i<=steps;i++){
      bottom.write(c1+(b-c1)*i/steps);
      left.write(c2+(l-c2)*i/steps);
      right.write(c3+(r-c3)*i/steps);
      claw.write(c4+(c-c4)*i/steps);
      delay(spd);
    }
  }
  bottomAngle=b; leftAngle=l; rightAngle=r; clawAngle=c;
}

/*
任务二夹取动作
A B C 函数使用逻辑都为调用moveTo函数来完成移动
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
  moveTo(90,115,80,100,moveDelay);
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
  moveTo(90,115,80,100,moveDelay);
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
  moveTo(90,115,80,100,moveDelay);
}

//      ========任务五·运动学=========
//a1--底座角度 a2--肩(左)的关节角 a3--肘(右)的关节角
/*
机械臂的正解
输入三个舵机角度，算出笔尖坐标 x/y/z
*/
void foreanswer(float a1,float a2,float a3,float &x,float &y,float &z){
  //二连杆：a1=底座, a2=左舵机(肘), a3=右舵机(肩)
  //后臂角 j2，前臂相对后臂角 j3
  float j1=a1;                        //底座关节角（直接对应）
  float j2=(18.0/17.0)*(a3-165.0);    //后臂角（右=肩，向下为正，负=向上）
  float j3=0.559*(115.0-a2);          //前臂相对后臂角（左=肘）
  float a11=j1/180.0*PI;
  float aa=j2/180.0*PI;
  float bb=(j2+j3)/180.0*PI;
  x=(L2*cos(aa)+L3*cos(bb)+L4)*cos(a11);
  y=(L2*cos(aa)+L3*cos(bb)+L4)*sin(a11);
  z=L1+L2*sin(aa)+L3*sin(bb);
}

//浮点角度写舵机，用 writeMicroseconds 提高精度（小于1度也能动）
void writeServo(Servo &s, float angle){
  if(&s==&bottom)      angle=constrain(angle,0.0,180.0);
  else if(&s==&left)  angle=constrain(angle,0.0,137.0);
  else if(&s==&right) angle=constrain(angle,5.0,165.0);
  else                angle=constrain(angle,0.0,180.0);
  int us=(int)(544.0+angle*1856.0/180.0);
  s.writeMicroseconds(us);
}

/*
机械臂的逆解
输入笔尖坐标 x/y/z，算出三个舵机角度 a1/a2/a3
*/

void backanswer(float x,float y,float z,float &a1,float &a2,float &a3){
  //二连杆逆解
  float j1=atan2(y,x)*180.0/PI;//底座关节角
  float R=sqrt(x*x+y*y);//水平距离
  float D_h=R-L4;//夹爪到肩的水平投影
  float h=z-L1;//夹爪到肩的竖直投影
  float D=sqrt(D_h*D_h+h*h);//肩到夹爪距离
  float cosJ3=(D*D-L2*L2-L3*L3)/(2*L2*L3);
  cosJ3=constrain(cosJ3,-1.0,1.0);
  float j3=acos(cosJ3);//前臂相对后臂角（正，肘朝下折）
  float gamma=atan2(h,D_h);//合力方向
  float cosBeta=(L2*L2+D*D-L3*L3)/(2*L2*D);
  cosBeta=constrain(cosBeta,-1.0,1.0);
  float beta=acos(cosBeta);
  float j2=gamma-beta;//后臂角（肘朝上构型）
  //关节角转舵机角
  a1=j1;                        //底座舵机角
  a2=115.0-j3/0.559;            //左舵机角（肘）
  a3=165.0+j2*17.0/18.0;        //右舵机角（肩）
  a1=constrain(a1,0.0,180.0);
  a2=constrain(a2,0.0,137.0);
  a3=constrain(a3,5.0,165.0);
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
  float c1=bottom.read(), c2=left.read(), c3=right.read();
  //关节空间插值：舵机角线性，不会经过笛卡尔空间里的不可达点
  float steps=max(max(fabs(a1-c1),fabs(a2-c2)),fabs(a3-c3));
  if(steps<1.0) steps=1.0;
  for(int i=1;i<=(int)steps;i++){
    writeServo(bottom, c1+(a1-c1)*i/steps);
    writeServo(left, c2+(a2-c2)*i/steps);
    writeServo(right, c3+(a3-c3)*i/steps);
    delay(moveDelay);
  }
  bottomAngle=a1;
  leftAngle=a2;
  rightAngle=a3;
}

/*
把三个运动舵机同步细分到目标角。
先算笛卡尔点，再调用这里做关节角平滑，避免每个点之间突然跳舵机。
*/
void movetogether(float b,float l,float r){
  b=constrain(b,0.0,180.0);
  l=constrain(l,0.0,137.0);
  r=constrain(r,5.0,165.0);
  float b0=bottomAngle, l0=leftAngle, r0=rightAngle;
  float db=b-b0, dl=l-l0, dr=r-r0;
  int steps=max(1,(int)max(max(fabs(db),fabs(dl)),fabs(dr)));
  for(int i=1;i<=steps;i++){
    check();
    if(cancel) return;
    while(stop==true && cancel==false){
      check();
      delay(10);
    }
    writeServo(bottom, b0+db*i/steps);
    writeServo(left, l0+dl*i/steps);
    writeServo(right, r0+dr*i/steps);
    delay(drawdelay);
  }
  bottomAngle=b;
  leftAngle=l;
  rightAngle=r;
}

// ===== 三角形 =====
void changepolar(float a,float b,float &r,float &th){
  r=sqrt(a*a+b*b);
  if(r==0.0){
    th=0.0;
    return;
  }
  th=atan2(b,a);
}

bool tricos(float opp,float adj1,float adj2,float &theta){
  float den=2.0*adj1*adj2;
  if(den==0.0) return false;
  float c=(adj1*adj1+adj2*adj2-opp*opp)/den;
  if(c>1.0 || c<-1.0) return false;
  theta=acos(c);
  return true;
}

bool tbackanswer(float x,float y,float z,float &a0,float &a1,float &a2){
  float r,th0;
  changepolar(y,x,r,th0);
  r-=46.0;

  float R,angP;
  changepolar(r,z,R,angP);

  float B,C;
  if(!tricos(80.0,80.0,R,B)) return false;
  if(!tricos(R,80.0,80.0,C)) return false;

  a0=th0;
  a1=angP+B;
  a2=C+a1-PI;
  return true;
}

void tforeanswer(float a0,float a1,float a2,float &x,float &y,float &z){
  float u=80.0*cos(a1)+80.0*cos(a2)+46.0;
  float v=80.0*sin(a1)+80.0*sin(a2);
  y=u*cos(a0);
  x=u*sin(a0);
  z=v;
}

void tanswer(float a0,float a1,float a2,float &b,float &l,float &r){
  b=constrain(90.0-degrees(a0),0.0,180.0);
  r=constrain(165.0-degrees(a1)/1.8,5.0,165.0);
  l=constrain(115.0-degrees(a2)/0.559,0.0,137.0);
}

void triCurrent(float &x,float &y,float &z){
  float a0=radians(90.0-bottomAngle);
  float a1=radians((165.0-rightAngle)*1.8);
  float a2=radians((115.0-leftAngle)*0.559);
  tforeanswer(a0,a1,a2,x,y,z);
}

void tmoveto(float x,float y,float z){
  float cx,cy,cz;
  triCurrent(cx,cy,cz);
  float dx=x-cx, dy=y-cy, dz=z-cz;
  float dist=sqrt(dx*dx+dy*dy+dz*dz);
  int steps=max(1,(int)(dist/1.0));

  float a0,a1,a2;
  for(int i=1;i<=steps;i++){
    float px=cx+dx*i/steps;
    float py=cy+dy*i/steps;
    float pz=cz+dz*i/steps;
    if(tbackanswer(px,py,pz,a0,a1,a2)){
      float b,l,r;
      tanswer(a0,a1,a2,b,l,r);
      movetogether(b,l,r);
      if(cancel) return;
    }
  }
}

void drawt(){
  claw.write(100);
  tmoveto(78.0,78.0,thighout);     //抬笔到 A 上方
  tmoveto(78.0,78.0,thighA); //落到 A
  tmoveto(130.0,78.0,thigh);  //A -> B
  delay(pointpause);
  tmoveto(98.0,124.0,thigh);  //B -> C
  delay(pointpause);
  tmoveto(78.0,78.0,thighA); //C -> A
  delay(pointpause);
  tmoveto(78.0,78.0,thighout);     //抬笔
  moveTo(90,115,80,100,moveDelay);
}

//笛卡尔插值移动：x,y,z 线性变化，用于垂直抬笔/落笔，避免斜着拖纸
void moveline(float x,float y,float z){
  float cx,cy,cz;
  foreanswer(bottomAngle,leftAngle,rightAngle,cx,cy,cz);
  float dist=sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy)+(z-cz)*(z-cz));
  int steps=(int)(dist/1.0);
  if(steps<1) steps=1;
  float a1,a2,a3;
  for(int i=1;i<=steps;i++){
    float mx=cx+(x-cx)*i/steps;
    float my=cy+(y-cy)*i/steps;
    float mz=cz+(z-cz)*i/steps;
    backanswer(mx,my,mz,a1,a2,a3);
    movetogether(a1,a2,a3);
    if(cancel) return;
  }
  bottomAngle=a1;
  leftAngle=a2;
  rightAngle=a3;
}

/*
用drawline函数来画一条直线
x1,y1为起点 x2,y2为终点
画的过程支持暂停/继续/取消
*/
void drawline(float x1,float y1,float z1,float x2,float y2,float z2){
  float dist=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
  int steps=(int)(dist/drawstep);//每 1mm 一步
  if(steps<1) steps=1;
  float a1,a2,a3;
  for(int i=0;i<=steps&&cancel==false;i++){
    //当处于停止且没取消时反复检查
    while(stop==true&&cancel==false){
      check();
      delay(10);
    }
    check();//防止未进入while循环
    //算出当前点的位置
    float x=x1+(x2-x1)*i/steps;
    float y=y1+(y2-y1)*i/steps;
    float z=z1+(z2-z1)*i/steps;
    //逆解出目标舵机角，再用关节平滑插值写舵机
    backanswer(x,y,z,a1,a2,a3);
    movetogether(a1,a2,a3);
  }
  bottomAngle=a1;
  leftAngle=a2;
  rightAngle=a3;
}

//绘制三角形
void triangle(float x1,float y1,float x2,float y2,float x3,float y3){
  movearm(x1,y1,outpaper);
  movearm(x1,y1,inpaper);//落笔
  drawline(x1,y1,inpaper,x2,y2,inpaper);
  drawline(x2,y2,inpaper,x3,y3,inpaper);
  drawline(x3,y3,inpaper,x1,y1,inpaper);
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
void curveSegment(float p0x,float p0y,float p0z,float p1x,float p1y,float p1z,float p2x,float p2y,float p2z,float p3x,float p3y,float p3z){
  int steps=40;
  float a1,a2,a3;
  for(int i=0;i<=steps && cancel==false;i++){
    while(stop==true && cancel==false){
      check();
      delay(10);
    }
    check();
    float t=(float)i/steps;
    float t2=t*t;
    float t3=t2*t;
    float x=0.5*((2*p1x)+(-p0x+p2x)*t+(2*p0x-5*p1x+4*p2x-p3x)*t2+(-p0x+3*p1x-3*p2x+p3x)*t3);
    float y=0.5*((2*p1y)+(-p0y+p2y)*t+(2*p0y-5*p1y+4*p2y-p3y)*t2+(-p0y+3*p1y-3*p2y+p3y)*t3);
    float z=0.5*((2*p1z)+(-p0z+p2z)*t+(2*p0z-5*p1z+4*p2z-p3z)*t2+(-p0z+3*p1z-3*p2z+p3z)*t3);
    backanswer(x,y,z,a1,a2,a3);
    movetogether(a1,a2,a3);
    if(cancel) return;
  }
  bottomAngle=a1;
  leftAngle=a2;
  rightAngle=a3;
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
  Serial.print(rightAngle);
  Serial.print(" 肘=");
  Serial.print(leftAngle);
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
      Serial.println("已记录满5个点再按一次清空");
    }
  }else{
    //已经满5个点，再点一次就清空，重新开始
    for(int i=0;i<5;i++){
      x[i]=0;y[i]=0;z[i]=0;
      b5[i]=0;l5[i]=0;r5[i]=0;
    }
    number1=0;
    Serial.println("已清空记录，可以重新示教");
  }
}

/*
根据某个示教点的三个舵机角，算出它的抬笔舵机角。
抬笔时肘加一点、肩收一点，让笔先抬起来并往回收，不再伸到最远。
*/
void fivePointUpAngles(int i,float &b,float &l,float &r){
  b=b5[i];
  l=constrain(l5[i]+20.0,0.0,137.0);
  r=constrain(r5[i]-10.0,5.0,165.0);
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
    float ub,ul,ur;
    fivePointUpAngles(0,ub,ul,ur);
    moveTo((int)ub,(int)ul,(int)ur,100,moveDelay);       //第一个点抬笔
    moveTo((int)b5[0],(int)l5[0],(int)r5[0],100,moveDelay);//第一个点落笔
    delay(pointpause);
    //直接用记录到的舵机角依次连线，不再做逆解
    for(int i=0;i<number1-1&&cancel==false;i++){
      movetogether(b5[i+1],l5[i+1],r5[i+1]);
      delay(pointpause);
    }
    if(cancel){
      cancel=false;
      moveTo(90,115,80,100,moveDelay);
      Serial.println("已取消并回中");
    }else{
      //画完先垂直抬笔，再回中
      int last=number1-1;
      fivePointUpAngles(last,ub,ul,ur);
      moveTo((int)ub,(int)ul,(int)ur,100,moveDelay);
      moveTo(90,115,80,100,moveDelay);
      Serial.println("折线已经画好");
    }
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
  float ub,ul,ur;
  fivePointUpAngles(0,ub,ul,ur);
  moveTo((int)ub,(int)ul,(int)ur,100,moveDelay);       //第一个点抬笔
  moveTo((int)b5[0],(int)l5[0],(int)r5[0],100,moveDelay);//第一个点落笔
  curveSegment(x[0],y[0],z[0], x[0],y[0],z[0], x[1],y[1],z[1], x[2],y[2],z[2]);
  curveSegment(x[0],y[0],z[0], x[1],y[1],z[1], x[2],y[2],z[2], x[3],y[3],z[3]);
  curveSegment(x[1],y[1],z[1], x[2],y[2],z[2], x[3],y[3],z[3], x[4],y[4],z[4]);
  curveSegment(x[2],y[2],z[2], x[3],y[3],z[3], x[4],y[4],z[4], x[4],y[4],z[4]);
  if(cancel){
    cancel=false;
    moveTo(90,115,80,100,moveDelay);
    Serial.println("已取消并回中");
  }else{
    fivePointUpAngles(4,ub,ul,ur);
    moveTo((int)ub,(int)ul,(int)ur,100,moveDelay);
    moveTo(90,115,80,100,moveDelay);
    Serial.println("曲线已画好");
  }
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
  moveDelay=50;
  bottom.write(90);
  left.write(115);
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
      moveTo(90,115,80,100,moveDelay);
    }
    //=======任务五========
    //画直线
    else if(input=="D"){
      stop=false;
      cancel=false;
      int b1=90, l1=55, r1=118;             //竖线起点舵机角
      int l1Up=75, r1Up=120;                //起点抬笔：z=-31.5，比纸面高约10mm
      int b2=90;                            //竖线终点底座角也保持90
      int l2Up=79, r2Up=95;                 //终点抬笔：z=-79.9，比纸面高约11mm
      moveTo(b1,l1Up,r1Up,100,moveDelay);   //先到起点抬笔姿势
      moveTo(b1,l1,r1,100,moveDelay);       //落到竖线起点
      drawline(0.0,174.49,-21.62,0.0,115.22,-68.93); //底座固定90°，只靠肩肘画竖线
      if(cancel){
        cancel=false;
        moveTo(90,115,80,100,moveDelay);    //取消后回中
        Serial.println("已取消并回中");
      }else{
        moveTo(b2,l2Up,r2Up,100,moveDelay); //终点先往上抬笔
        moveTo(90,115,80,100,moveDelay);    //抬笔后再回中
        Serial.println("直线已画好");
      }
    }
    //画三角形
    else if(input=="T"){
      stop=false;
      cancel=false;
      drawt();
      if(cancel){
        cancel=false;
        moveTo(90,115,80,100,moveDelay);    //取消后回中
        Serial.println("已取消并回中");
      }else{
        Serial.println("三角形已画好");
      }
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
