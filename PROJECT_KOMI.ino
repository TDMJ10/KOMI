/*
  BUH&S SPRING 2025 AUTONOMOUS CONTROLLABLE MOTORIZED ROBOT PROJECT

  Written by Naim Iftekhar Rahman (TDMJ10)
  tdmj10@gmail.com
  naimiftekharrahman@gmail.com
  naim.iftekhar.rahman@g.bracu.ac.bd
  Student of Bangldesh BRAC University (24201190)
  
        CAUTION:
        ---------
        THIS CODE WILL CAUSE BLUE SCREENS OF DEATH (IQRL_NOT_LESS_OR_EQUAL) ON WINDOWS SYSTEMS
        DUE TO EXCESSIVE SERIAL PRINTS ON ARDUINO IDE DUE TO A BUG.
        (Arduino 2.3.4 As of 3/6/25)

        THIS CODE IS UNTESTED ON MAC OS. IT MAY CAUSE KERNEL PANICS OR SYSTEM CRASHES TOO.

        CLEAR YOUR SERIAL MONITOR REGULARLY TO MAKE SURE YOU DO NOT CRASH.

        WHEN ENTERING MANUAL MODE, DISCONNECTING THE ESP32 FROM YOUR COMPUTER
        IS HIGHLY RECCOMENDED AS SERIAL PRINTS WILL COME EXTREMELY QUICKLY

 To do:
==========
1: Buy two SSD1306 WHITES
2: Setup i2C with both SSD1306s and 16x2 LCD
3: 













================================================================================================

  ██████╗ ██████╗  ██████╗      ██╗███████╗ ██████╗████████╗    ██╗  ██╗ ██████╗ ███╗   ███╗██╗
  ██╔══██╗██╔══██╗██╔═══██╗     ██║██╔════╝██╔════╝╚══██╔══╝    ██║ ██╔╝██╔═══██╗████╗ ████║██║
  ██████╔╝██████╔╝██║   ██║     ██║█████╗  ██║        ██║       █████╔╝ ██║   ██║██╔████╔██║██║
  ██╔═══╝ ██╔══██╗██║   ██║██   ██║██╔══╝  ██║        ██║       ██╔═██╗ ██║   ██║██║╚██╔╝██║██║
  ██║     ██║  ██║╚██████╔╝╚█████╔╝███████╗╚██████╗   ██║       ██║  ██╗╚██████╔╝██║ ╚═╝ ██║██║
  ╚═╝     ╚═╝  ╚═╝ ╚═════╝  ╚════╝ ╚══════╝ ╚═════╝   ╚═╝       ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝╚═╝
                                                                                             
================================================================================================


    ========================
           PARTS USED:
    ========================                                               QTY: PRICE: (BDT)
    1: EXPRESSIF ESP32 AI-THINKER MICROCONTROLLER + OV2640 CAMERA (MASTER) |1| | 800
    2: EXPRESSIF ESP32 C340 DEVKIT-C MICROCONTROLLER (SLAVE)               |1| | 550
    3: ESP32 30P EXPANSION BOARD (SLAVE)                                   |1| | 450
    3: MPU6050 GYROSCOPE & ACCELEROMETER                                   |1| | 250
    3: ADAFRUIT SSD1306 0.96" 128x64 OLED DISPLAY                          |2| | 290
    4: ADAFRUIT 16x2 LIQUID CRYSTAL DISPLAY + I2C DISPLAY DRIVER           |1| | 340
    5: HC-SR04 ULTRASOUND SENSOR                                           |1| | 100
    4: SG90 MICRO-SERVO                                                    |8| | 135
    3: L293D MOTOR DRIVER                                                  |2| | 215
    5: N20 GEAR MOTOR                                                      |4| | 200
    7: N20 GEAR MOTOR WHEELS                                               |4| | 80
    8: LM2596S DC TO DC BUCK CONVERTER                                     |1| | 80
    6: 18650 LITHIUM ION BATTERY                                           |6| | 80
    9: 18650 BATTERY (x3) CELL HOLDER                                      |2| | 75

    TOTAL PROJECT COST: 800+550+450+250+290*2+340+100+135*8+2*215+4*200+80*4+6*80+75*2
                        = 6610 BDT
*/



//  ========================
//      GLOBAL VARIABLES:
//  ========================

// ESP NOW
String PREVIOUS_COMMAND = "ERROR";
String COMMAND = "ERROR";

// TROUBLESHOOTING
bool STARTUP_CHECK_COMPLETE = false;
int LOG_NO = 1;

//  ========================
//          PRE SETUP:
//  ========================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// ESP NOW 
uint8_t SLAVE_ESP_MAC[] = { 0x94, 0x54, 0xC5, 0xB5, 0xC6, 0x8C };

// HEAD ASSEMBLY SERVOS
Servo HEAD_1; // UP DOWN SERVO
Servo HEAD_2; // LEFT RIGHT SERVO

// DISPLAY EYES
#define LEFT_SCREEN_WIDTH 128
#define LEFT_SCREEN_HEIGHT 64
#define LEFT_OLED_RESET -1
#define LEFT_SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 LEFT_EYE(LEFT_SCREEN_WIDTH, LEFT_SCREEN_HEIGHT, &Wire, LEFT_OLED_RESET);

#define RIGHT_SCREEN_WIDTH 128
#define RIGHT_SCREEN_HEIGHT 64
#define RIGHT_OLED_RESET -1
#define RIGHT_SCREEN_ADDRESS  // SETUP MISSING
Adafruit_SSD1306 RIGHT_EYE(RIGHT_SCREEN_WIDTH, RIGHT_SCREEN_HEIGHT, &Wire, RIGHT_OLED_RESET);

// STATUS DISPLAY
LiquidCrystal_I2C STATUS_DISPLAY(0x27, 16,2);

//  ========================
//      PIN DEFINITIONS:
//  ========================

// I2C
const int SDA_PIN = 13;
const int SCL_PIN = 15;

// HEAD ASSEMBLY SERVOS
const int HEAD_1_PIN = 14; // UP DOWN SERVO
const int HEAD_2_PIN = 2; // LEFT RIGHT SERVO

//  ========================
//      LOGGING & ERRORS
//  ========================

void ERROR_FLASH() {
  digitalWrite(4, HIGH);
  delay(150);
  digitalWrite(4, LOW);
  delay(500);
  digitalWrite(4, HIGH);
  delay(150);
  digitalWrite(4, LOW);
  delay(100);
}

void PRINT_LOG(String TEXT) {
  Serial.println(LOG(TEXT));
}

String LOG(String TEXT) {
  return ("|" + String(LOG_NO++) + "|  | | MASTER> " + TEXT);
}

String TRANSMISSION_LOG(String COMMAND) {
  return ("|" + String(LOG_NO++) + "|  | | MASTER> SENT SLAVE: " + COMMAND);
}



//  # = General Error
//  ^ = Communication Error
//  X = Part Error

String ERR(String TEXT) {
  ERROR_FLASH();
  return ("|" + String(LOG_NO++) + "|  |#| MASTER> " + TEXT);
}

String TRANSMISSION_ERR(String COMMAND) {
  STATUS_DISPLAY.print("|^|" + COMMAND);
  ERROR_FLASH();
  return ("|" + String(LOG_NO++) + "|  |^| MASTER> COULD NOT SEND SLAVE: " + COMMAND);
}

String PART_ERR(String COMMAND) {
  STATUS_DISPLAY.print("|X|" + COMMAND);
  ERROR_FLASH();
  return ("|" + String(LOG_NO++) + "|  |X| MASTER> COULD NOT ALLOCATE: " + COMMAND);
}


//  ========================
//   ESP NOW COMMUNICATION:
//  ========================


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? TRANSMISSION_LOG(PREVIOUS_COMMAND) : TRANSMISSION_ERR(PREVIOUS_COMMAND));
  delay(10);
}

void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  COMMAND = String((char *)incomingData);
  if(COMMAND.equals("LIVE")) {
    STARTUP_CHECK_COMPLETE = true;
  }
  else {
    if(COMMAND.equals("REMOTE")) {
      REMOTE_MODE();
    }
    else {
      ERR("COMMAND NOT RECOGNIZED: " + COMMAND);
    }
  }

  delay(10);
}

void TRANSMIT(String COMMAND) {
  PREVIOUS_COMMAND = COMMAND;
  esp_now_send(SLAVE_ESP_MAC, (uint8_t *)COMMAND.c_str(), COMMAND.length() + 1);
  delay(10);
}






void STARTUP_SEQUENCE() {
  Serial.println("=============================================");
  Serial.println(" █▀█ █▀█ █▀█   █ █▀▀ █▀▀ ▀█▀ █▄▀ █▀█ █▀▄▀█ █");
  Serial.println(" █▀▀ █▀▄ █▄█ █▄█ ██▄ █▄▄  █  █ █ █▄█ █ ▀ █ █");
  Serial.println("=============================================");
  Serial.println();
  delay(1000);
  String Temp1 = "Hey There! Checking Back On This For Some Reason?";
  for (int Count = 0; Count<Temp1.length(); Count++) {
    Serial.print(Temp1.charAt(Count));
    delay(10);
  }
  delay(1000);
  Serial.println();
  String Temp2 = "Oh Boy What Errors Is It This Time....";
  for (int Count = 0; Count<Temp2.length(); Count++) {
    if(Temp2.charAt(Count)!='.') {
      delay(10);
      Serial.print(Temp2.charAt(Count));
    }
    else {
      delay(350);
      Serial.print(Temp2.charAt(Count));
    }
  }
  Serial.println();
  Serial.println();
  Serial.println();
  delay(2500);
  String Temp1 = "Oh Well Good Luck... BACK TO THE CONSOLE";
  for (int Count = 0; Count<Temp1.length(); Count++) {
    Serial.print(Temp1.charAt(Count));
    delay(10);
  }

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
}

void setup() {

  // SERIAL MONITOR
  Serial.begin(115200);
  Serial.println("=================================================================");
  Serial.println("Made By Naim Iftekhar Rahman (24201190) BRAC University For BUH&S");
  Serial.println("=================================================================");

                                        
//===============

  // I2C
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  delay(100);
  PRINT_LOG("I2C READY");

//===============

  // ESP NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("|" + String(LOG_NO++) + "|  |X| MASTER> ESP NOW INITIALIZATION FAILED");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, SLAVE_ESP_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("|" + String(LOG_NO++) + "|  |X| MASTER> ESP NOW ADDING PEER FAILED");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  PRINT_LOG("ESP NOW READY");
  delay(100);

//===============

  // REMOTE INTERRUPT
  attachInterrupt(digitalPinToInterrupt(2), activateMotor, FALLING);


//===============

  // DISPLAYS
  if (!LEFT_EYE.begin(SSD1306_SWITCHCAPVCC, LEFT_SCREEN_ADDRESS)) {
    Serial.println(PART_ERR("LEFT SSD1306"));
    while (1)
      ;
  }
  delay(100)
  LEFT_EYE.setCursor(0, 0);
  LEFT_EYE.clearDisplay();
  PRINT_LOG("LEFT EYE SSD1306 READY");

  delay(10);
  if (!RIGHT_EYE.begin(SSD1306_SWITCHCAPVCC, RIGHT_SCREEN_ADDRESS)) {
    Serial.println(PART_ERR("RIGHT SSD1306"));
    while (1)
      ;
  }
  delay(100);
  RIGHT_EYE.setCursor(0, 0);
  RIGHT_EYE.clearDisplay();
  PRINT_LOG("RIGHT EYE SSD1306 READY");

  delay(10);
  STATUS_DISPLAY.begin();
  PRINT_LOG("STATUS DISPLAY LCD READY");
  STATUS_DISPLAY.backlight();
  STATUS_DISPLAY.setCursor(0, 0);
  STATUS_DISPLAY.print("PROJECT KOMI");
  STATUS_DISPLAY.setCursor(0, 1);
  STATUS_DISPLAY.print("By Naim Iftekhar");
  delay(1000);

//===============

  // SERVOS
  HEAD_1.attach(HEAD_1_PIN); // UP DOWN SERVO
  HEAD_2.attach(HEAD_2_PIN); // LEFT RIGHT SERVO

//===============

  // SLAVE CHECK
  while(!STARTUP_CHECK_COMPLETE) {
    TRANSMIT("TEST");
    delay(3000);
  }
  PRINT_LOG("SLAVE READY");

  // 
  STARTUP_SEQUENCE();
}

void loop() {
  // put your main code here, to run repeatedly:

}
