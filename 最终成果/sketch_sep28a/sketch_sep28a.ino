#include<Servo.h>
int speed;//用来控制机械臂的整体速度
Servo bottom;//底座
Servo left;//左边
Servo right;//右边
Servo gripper;//夹爪
void setup() {
  //接引脚
  bottom.attach();
  left.attach();
  right.attach();
  gripper.attach();//引脚还没填 
  //串口初始化
  Serial.begin(9600);
  //速度初始化
  speed=1000;
}

void loop() {
  if(Serial.available()>0){
    //串口输入存储
    char input=Serial.read();
    //接收字符‘O’
    if(input=='O'){
      gripper.write(120);
      delay(speed);
      Serial.println("open");
    }
    //接收字符‘S'
    if(input=='S'){
      gripper.write(30);
      delay(speed);
      Serial.println("close")
    }
    //接收字符'H'
    if(input=='H'){
      speed-=100;
      if(speed<100){
        speed=100;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
    //接收字符‘L'
    if(input=='L'){
      speed+=100;
      if(speed>2000){
        speed=2000;
      }
      Serial.print("速度=");
      Serial.println(speed);
    }
  }
}
