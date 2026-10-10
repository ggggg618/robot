/*
 * ===== 遥控板程序（任务三 · 第二块板）=====
 *
 * 本程序烧在"遥控板/自制板"上，只负责一件事：
 *   读 4 个按键，通过串口把指令发给主控板。
 *
 * 本程序不接舵机、不控制任何动作。
 * 主控板收到这些指令后，才去执行对应动作。
 *
 * 串口接线（两块板之间）：
 *   遥控板 TX(1脚) -> 主控板 RX(0脚)
 *   遥控板 RX(0脚) <- 主控板 TX(1脚)
 *   遥控板 GND  ---  主控板 GND（共地）
 */

// 4 个按键的引脚（占位，待队友确认后修改）
const int K1_PIN = 2;   // 按键1 = 循环夹取
const int K2_PIN = 3;   // 按键2 = 录制
const int K3_PIN = 4;   // 按键3 = 播放
const int K4_PIN = 5;   // 按键4 = 回中

void setup() {
  Serial.begin(9600);            // 启动串口，发给主控
  pinMode(K1_PIN, INPUT_PULLUP); // 按键都用"内部上拉"
  pinMode(K2_PIN, INPUT_PULLUP);
  pinMode(K3_PIN, INPUT_PULLUP);
  pinMode(K4_PIN, INPUT_PULLUP);
}

void loop() {
  // 哪个按键按下，就发对应的指令字符串给主控
  if (digitalRead(K1_PIN) == LOW) {
    Serial.println("K1");
    delay(200);   // 消抖
  }
  if (digitalRead(K2_PIN) == LOW) {
    Serial.println("K2");
    delay(200);
  }
  if (digitalRead(K3_PIN) == LOW) {
    Serial.println("K3");
    delay(200);
  }
  if (digitalRead(K4_PIN) == LOW) {
    Serial.println("K4");
    delay(200);
  }
}
