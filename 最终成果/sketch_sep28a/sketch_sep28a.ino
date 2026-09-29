#include<Servo.h>

//存放数据处--------------
//引脚
const int bottom_yijiao=9;
const int left_yijiao=8;
const int right_yijiao=7;
const int claw_yijiao=6;

/*上位机给舵机的编号分别是
底部--'b'
左边--'l'
右边--'r'
*/

/*为完成多舵机协同控制，并防止上位机输入数据的格式没有按顺序
用findangle函数来返回舵机所对应角度*/
int findangle(String s,char label){
  int angle;
  int num=s.indexOf(label);//找到编号的位置
  if(num==-1){
    return 0;
  }
  else return s.substring(++num).toInt();
}

int speed;//用来控制机械臂的整体速度
int angle;//角度
Servo bottom;//底座
Servo left;//左边
Servo right;//右边
Servo claw;//夹爪

void setup() {
  //接引脚
  bottom.attach(bottom_yijiao);
  left.attach(left_yijiao);
  right.attach(right_yijiao);
  claw.attach(claw_yijiao);
  //串口初始化
  Serial.begin(9600);
  //速度初始化
  speed=1000;
}

void loop() {

  if(Serial.available()>0){
    //串口输入存储
    String input=Serial.readString();
    input.trim();
    //接收O
    if(input=="O"){
      claw.write(120);
      delay(speed);
      Serial.println("open");
    }
    //接收S
    else if(input=="S"){
      claw.write(30);
      delay(speed);
      Serial.println("close");
    }
    //接收H
    else if(input=="H"){
      speed-=100;
      if(speed<100){
        speed=100;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
    //接收L
    else if(input=="L"){
      speed+=100;
      if(speed>2000){
        speed=2000;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
    else{
    //实现舵机控制
    int b=findangle(input,'b');
    int l=findangle(input,'l');
    int r=findangle(input,'r');
    int c=findangle(input,'c');
    bottom.write(b);
    left.write(l);
    right.write(r);
    claw.write(c);
    }
  }
}
