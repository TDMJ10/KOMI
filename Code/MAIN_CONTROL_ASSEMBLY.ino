/*
  AUTONOMOUS CONTROLLABLE MOTORIZED ROBOT PROJECT

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
1: 
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
    ========================                                               QTY: PRICE: ADDITIONAL NOTES:
    1: EXPRESSIF ESP32 AI-THINKER MICROCONTROLLER + OV2640 CAMERA          |2| | 600  | OBJECT RECOGNITION + FEED CAMERA
    2: EXPRESSIF ESP32 C340 DEVKIT-C MICROCONTROLLER                       |3| | 550  | REMOTE + ACCESS POINT + MOTOR CONTROL
    3: ESP32 30P EXPANSION BOARD (SLAVE)                                   |1| | 450  | 
    4: MPU6050 GYROSCOPE & ACCELEROMETER                                   |1| | 250  |
    5: ADAFRUIT SSD1306 0.96" 128x64 OLED DISPLAY                          |2| | 290  |
    6: ADAFRUIT 16x2 LIQUID CRYSTAL DISPLAY + I2C DISPLAY DRIVER           |1| | 340  |
    7: HC-SR04 ULTRASOUND SENSOR                                           |1| | 100  |
    8: SG90 MICRO-SERVO                                                    |6| | 135  |
    9: L293D MOTOR DRIVER                                                  |2| | 215  |
    10: N20 GEAR MOTOR                                                     |4| | 200  |
    11: N20 GEAR MOTOR WHEELS                                              |4| | 80   |
    12: LM2596S DC TO DC BUCK CONVERTER                                    |1| | 80   |
    13: 3S 11.1V 2200mAh LIPO BATTERY                                      |1| | 2500 |
    14: FLAME SENSOR                                                       |1| | 70   | OPTIONAL
    15: MQ-2 FLAMMABLE GAS AND SMOKE SENSOR                                |1| | 150  | OPTIONAL
    16: AD20P 1230P BRUSHLESS WATER PUMP                                   |1| | 750  |
    17: STEEL BODY APPROXIMATE                                             |1| | 1000 |
    18: PCA9548A I2C MULTIPLEXER                                           |1| | 250  |

    TOTAL PROJECT COST: 600*2+550*3+450+250+290*2+340+100+135*6+215*2+200*4+80*4+2500+70+150+750+1000+250
                        = 11650 BDT
*/



//  ========================
//      GLOBAL VARIABLES:
//  ========================

// CEASE
bool CEASE = false;
const int CEASE_PIN; // PLACEHOLDER

// ESP NOW
String PREVIOUS_COMMAND = "ERROR";
String COMMAND = "ERROR";

// REMOTE CONTROL
String REMOTE_COMMAND = "VOID";
bool REMOTE_ACTIVE = false;
bool REMOTE_LOCK = false;


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
#include <Adafruit_PCF8574.h>
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
#define RIGHT_SCREEN_ADDRESS 0x3D // SETUP MISSING (PLACEHOLDER)
Adafruit_SSD1306 RIGHT_EYE(RIGHT_SCREEN_WIDTH, RIGHT_SCREEN_HEIGHT, &Wire, RIGHT_OLED_RESET);

// STATUS DISPLAY
LiquidCrystal_I2C STATUS_DISPLAY(0x27, 16,2);


//  ========================
//      PIN DEFINITIONS:
//  ========================

// I2C
const int SDA_PIN = 13;
const int SCL_PIN = 16;

// HEAD ASSEMBLY SERVOS
const int HEAD_1_PIN = 14; // UP DOWN SERVO
const int HEAD_2_PIN = 12; // LEFT RIGHT SERVO

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

void PRINT_IMPULSE_LOG(String TEXT) {
  Serial.println(LOG("IMPULSE> " + TEXT));
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
//  ~ = Edge Impulse Error

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

String IMPULSE_ERR(String TEXT) {
  STATUS_DISPLAY.print("|~| EDGE IMPULSE");
  ERROR_FLASH();
  return ("|" + String(LOG_NO++) + "|  |~| MASTER> IMPULSE> " + TEXT);
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
    if(COMMAND.equals("CEASE")) {
      CEASE = true;
    }
    else {
      if(COMMAND.equals("REMOTE ACTIVE")) {
        REMOTE_LOCK = true;
        REMOTE_MODE();
      }
      else {
        if(COMMAND.equals("REMOTE PASSIVE")) {
          REMOTE_MODE();
        }
        else {
          if(REMOTE_ACTIVE) {
            REMOTE_COMMAND = COMMAND;
          }
          else {
            ERR("COMMAND NOT RECOGNIZED: " + COMMAND);
          }
        }
      }
    }
  }

  delay(10);
}

void TRANSMIT(String COMMAND) {
  PREVIOUS_COMMAND = COMMAND;
  esp_now_send(SLAVE_ESP_MAC, (uint8_t *)COMMAND.c_str(), COMMAND.length() + 1);
  delay(10);
}


//  ========================
//       REMOTE CONTROL
//  ========================

void REMOTE_MODE() {
  REMOTE_ACTIVE = true;
  int Trigger_Time = millis()+5500;
  if(REMOTE_LOCK) {
    PRINT_LOG("ACTIVE REMOTE TRIGGERED");
    while(true) {
      if(REMOTE_COMMAND.equals("FORWARD")) {
        BODY_FORWARD();
      }
      else {
        if(REMOTE_COMMAND.equals("REVERSE")) {
          BODY_REVERSE();
        }
        else {
          if(REMOTE_COMMAND.equals("LEFT")) {
            BODY_LEFT();
          }
          else {
            if(REMOTE_COMMAND.equals("RIGHT")) {
              BODY_RIGHT();
            }
            else {
              if(REMOTE_COMMAND.equals("HEAD UP")) {
                HEAD_UP();
              }
              else {
                if(REMOTE_COMMAND.equals("HEAD DOWN")) {
                  HEAD_DOWN();
                }
                else {
                  if(REMOTE_COMMAND.equals("HEAD LEFT")) {
                    HEAD_LEFT();
                  }
                  else {
                    if(REMOTE_COMMAND.equals("HEAD RIGHT")) {
                      HEAD_RIGHT();
                    }
                    else {
                      if(REMOTE_COMMAND.equals("BREAK")) {
                        break;
                      }
                      else {
                        if(REMOTE_COMMAND.equals("LEFT ARM UP")) {
                          LEFT_ARM_UP();
                        }
                        else {
                          if(REMOTE_COMMAND.equals("RIGHT ARM UP")) {
                            RIGHT_ARM_UP();
                          }
                          else {
                            if(REMOTE_COMMAND.equals("LEFT ARM DOWN")) {
                              LEFT_ARM_UP();
                            }
                            else {
                              if(REMOTE_COMMAND.equals("RIGHT ARM DOWN")) {
                                RIGHT_ARM_DOWN();
                              }
                              else {
                                if(REMOTE_COMMAND.equals("VOID")) {
                                  // BLANK
                                }
                                else {
                                  ERR("MOVEMENT ERROR UNRECOGNIZED COMMAND: " + REMOTE_COMMAND);
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              } 
            }
          }
        }
      }
    }
    PRINT_LOG("ACTIVE DISENGAGED");
  }
  else {
    
    PRINT_LOG("PASSIVE REMOTE TRIGGERED");
    while(true) {
      if(millis()>Trigger_Time) {
        break;
      }
      if(REMOTE_COMMAND.equals("FORWARD")) {
        BODY_FORWARD();
      }
      else {
        if(REMOTE_COMMAND.equals("REVERSE")) {
          BODY_REVERSE();
        }
        else {
          if(REMOTE_COMMAND.equals("LEFT")) {
            BODY_LEFT();
          }
          else {
            if(REMOTE_COMMAND.equals("RIGHT")) {
              BODY_RIGHT();
            }
            else {
              if(REMOTE_COMMAND.equals("HEAD UP")) {
                HEAD_UP();
              }
              else {
                if(REMOTE_COMMAND.equals("HEAD DOWN")) {
                  HEAD_DOWN();
                }
                else {
                  if(REMOTE_COMMAND.equals("HEAD LEFT")) {
                    HEAD_LEFT();
                  }
                  else {
                    if(REMOTE_COMMAND.equals("HEAD RIGHT")) {
                      HEAD_RIGHT();
                    }
                    else {
                      if(REMOTE_COMMAND.equals("FREE_PLACEHOLDER")) {
                        // FREE PLACEHOLDER
                      }
                      else {
                        if(REMOTE_COMMAND.equals("LEFT ARM UP")) {
                          LEFT_ARM_UP();
                        }
                        else {
                          if(REMOTE_COMMAND.equals("RIGHT ARM UP")) {
                            RIGHT_ARM_UP();
                          }
                          else {
                            if(REMOTE_COMMAND.equals("LEFT ARM DOWN")) {
                              LEFT_ARM_DOWN();
                            }
                            else {
                              if(REMOTE_COMMAND.equals("RIGHT ARM DOWN")) {
                                RIGHT_ARM_DOWN();
                              }
                              else {
                                if(REMOTE_COMMAND.equals("VOID")) {
                                  // BLANK
                                }
                                else {
                                  ERR("MOVEMENT ERROR UNRECOGNIZED COMMAND: " + REMOTE_COMMAND);
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              } 
            }
          }
        }
      }
      REMOTE_COMMAND = "VOID";
    }
    PRINT_LOG("PASSIVE DISENGAGED");
  }
  REMOTE_ACTIVE = false;
}

void BODY_FORWARD() {
  // PLACEHOLDER
}

void BODY_REVERSE() {
  // PLACEHOLDER
}

void BODY_LEFT() {
  // PLACEHOLDER
}

void BODY_RIGHT() {
  // PLACEHOLDER
}

void HEAD_UP() {
  // PLACEHOLDER
}

void HEAD_DOWN() {
  // PLACEHOLDER
}

void HEAD_LEFT() {
  // PLACEHOLDER
}

void HEAD_RIGHT() {
  // PLACEHOLDER
}

void LEFT_ARM_UP() {
  // PLACEHOLDER
}

void LEFT_ARM_DOWN() {
  // PLACEHOLDER
}

void RIGHT_ARM_UP() {
  // PLACEHOLDER
}

void RIGHT_ARM_DOWN() {
  // PLACEHOLDER
}


//  ========================
//       INTELLIGENCE
//  ========================

void ARM_RAISE_BOTH() {
  // PLACEHOLDER
}

void ARM_RAISE_LEFT() {
  // PLACEHOLDER
}

void ARM_RAISE_RIGHT() {
  // PLACEHOLDER
}

void ARM_LOWER_BOTH() {
  // PLACEHOLDER
}

void ARM_LOWER_LEFT() {
  // PLACEHOLDER
}

void ARM_LOWER_RIGHT() {
  // PLACEHOLDER
}

void PASSIVE_INTELLIGENCE() {
  // PLACEHOLDER
}



//  ========================
//           OTHER
//  ========================

void BATTERY_CHECK() {
  int CEASE_TIMER = millis();
  while(true) {
    if(millis()>(CEASE_TIMER+5000)) {
      if(analogReadMilliVolts(CEASE_PIN)<2880) {
        CEASE = true;
        break;
      }
    }
  }
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
  String Temp3 = "Oh Well Good Luck... BACK TO THE CONSOLE";
  for (int Count = 0; Count<Temp3.length(); Count++) {
    Serial.print(Temp3.charAt(Count));
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

  BATTERY_CHECK();

  
  if(!CEASE) {

    // SERIAL MONITOR
    Serial.begin(115200);

    Serial.println("=================================================================");
    Serial.println("Made By Naim Iftekhar Rahman (24201190) BRAC University");
    Serial.println("=================================================================");

//=================

    // I2C
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();
    delay(100);
    PRINT_LOG("I2C READY");

//=================

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

//=================

    // DISPLAYS
    if (!LEFT_EYE.begin(SSD1306_SWITCHCAPVCC, LEFT_SCREEN_ADDRESS)) {
      Serial.println(PART_ERR("LEFT SSD1306"));
      while (1)
        ;
    }
    delay(100);
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

//=================

    // SERVOS
    HEAD_1.attach(HEAD_1_PIN); // UP DOWN SERVO
    HEAD_2.attach(HEAD_2_PIN); // LEFT RIGHT SERVO

//=================

    // SLAVE CHECK
    while(!STARTUP_CHECK_COMPLETE) {
      TRANSMIT("TEST");
      delay(3000);
    }
    PRINT_LOG("SLAVE READY");

//=================

    // BOOTUP SEQUENCE
    STARTUP_SEQUENCE();
  }
}

void loop() {
  if(!CEASE) {
    PASSIVE_INTELLIGENCE();
  }
}
