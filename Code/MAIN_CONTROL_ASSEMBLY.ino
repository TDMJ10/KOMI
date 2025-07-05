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
2:
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
int CEASE_PIN; // PLACEHOLDER

// ESP NOW
String PREVIOUS_COMMAND = "ERROR";
String COMMAND = "ERROR";

// REMOTE CONTROL
String REMOTE_COMMAND = "VOID";
bool REMOTE_ACTIVE = false;
bool REMOTE_LOCK = false;

// SERVOS
int CURRENT_PAN_ANGLE = 0;
int CURRENT_TILT_ANGLE = 0;

// STEPPERS

// PLACEHOLDER

// TROUBLESHOOTING
bool STARTUP_CHECK_COMPLETE = false;
int LOG_NO = 1;
bool DEBUG_MODE = false;

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
Servo HEAD_PAN; // LEFT RIGHT SERVO
Servo HEAD_TILT; // UP DOWN SERVO

// GPIO EXPANDERS
Adafruit_PCF8574 MOTORS;
Adafruit_PCF8574 STEPPERS;

//  ========================
//      PIN DEFINITIONS:
//  ========================

// I2C
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// HEAD ASSEMBLY SERVOS
const int PAN_SERVO_PIN = 13; // LEFT RIGHT SERVO
const int TILT_SERVO_PIN = 12; // UP DOWN SERVO

// BODY ASSEMBLY MOTORS
const int LEFT_IN1 = 1;
const int LEFT_IN2 = 2;
const int LEFT_IN3 = 3;
const int LEFT_IN4 = 4;

const int RIGHT_IN1 = 5;
const int RIGHT_IN2 = 6;
const int RIGHT_IN3 = 7;
const int RIGHT_IN4 = 8;



// TROUBLESHOOTING
const int TROUBLESHOOTING_PIN = 14; 

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
            if(ADVANCED_COMMAND(COMMAND)) {
              // PLACEHOLDER
            }
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
//      I2C MULTIPLEXING
//  ========================

void I2C_MULT(uint8_t BUS){
  Wire.beginTransmission(0x70);  // TCA9548A Address
  Wire.write(1 << BUS);
  Wire.endTransmission();
  Serial.print(BUS);
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
  if(CURRENT_TILT_ANGLE>=0 || CURRENT_TILT_ANGLE<180) {
    HEAD_PAN.write(++CURRENT_TILT_ANGLE);
  }
}

void HEAD_DOWN() {
  if(CURRENT_TILT_ANGLE>0 || CURRENT_TILT_ANGLE<=180) {
    HEAD_PAN.write(--CURRENT_TILT_ANGLE);
  }
}

void HEAD_LEFT() {
  if(CURRENT_PAN_ANGLE>0 || CURRENT_PAN_ANGLE<=180) {
    HEAD_PAN.write(--CURRENT_PAN_ANGLE);
  }
}

void HEAD_RIGHT() {
  if(CURRENT_PAN_ANGLE>=0 || CURRENT_PAN_ANGLE<180) {
    HEAD_PAN.write(++CURRENT_PAN_ANGLE);
  }
}

void HEAD_PAN_SET(int ANGLE) {
  if(ANGLE>=0 && ANGLE<=180) {
    HEAD_PAN.write(ANGLE);
  }
  else {
    ERR("INVALID PAN ANGLE: " + ANGLE);
  }
}

void HEAD_TILT_SET(int ANGLE) {
  if(ANGLE>=0 && ANGLE<=180) {
    HEAD_TILT.write(ANGLE);
  }
  else {
    ERR("INVALID TILT ANGLE: " + ANGLE);
  }
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

void GET_DISTANCE() {
  // PLACEHOLDER
}

bool VISUAL(String INPUT) {
  if(INPUT.equals("PERSON" || INPUT.equals("FACE"))) {
    if(INPUT.equals("PERSON")) {
      // GET CLOSER
    }
    if(INPUT.equals("FACE")) {
      // PLACEHOLDER
    }
  }
  else {
    return false;
  }
}

void CAM_FUNCS(String Visual) {
  String OBJECT_STR = "";
  String X_COORD_STR = "";
  String Y_COORD_STR = "";
  String WIDTH_STR = "";
  String HEIGHT_STR = "";
  bool Object_Found = false;
  bool X_Coord_Found = false;
  bool Y_Coord_Found = false;
  bool Width_Found = false;
  bool Height_Found = false;
  int X_COORD;
  int Y_COORD;
  int WIDTH;
  int HEIGHT;
  int DISTANCE;

  GET_DISTANCE();

  for(int Count = 0; Count<Visual.length; Count++) {
    if(Visual.charAt(Count)==' ') {
      if(Object_Found) {
        if(X_Coord_Found) {
          if(Y_Coord_Found) {
            if(Width_Found) {
              if(Height_Found) {
              }
              else {
                Height_Found = true;
              }
            }
            else {
              Width_Found = true;
            }
          }
          else {
            Y_Coord_Found = true;
          }
        }
        else {
          X_Coord_Found = true;
        }
      }
      else {
        Object_Found = true;
      }
    }
    else {
      if(!Object_Found) {
        OBJECT_STR += Visual.charAt(Count);
      }
      else {
        if(!X_Coord_Found) {
          X_COORD_STR += Visual.charAt(Count);
        }
        else {
          if(!Y_Coord_Found) {
            Y_COORD_STR += Visual.charAt(Count);
          }
          else {
            if(!Width_Found) {
              WIDTH_STR += Visual.charAt(Count);
            }
            else {
              HEIGHT_STR += Visual.charAt(Count);
            }
          }
        }
      }
    }
  }
  
  X_COORD = X_COORD_STR.toInt();
  Y_COORD = Y_COORD_STR.toInt();
  WIDTH = WIDTH_STR.toInt();
  HEIGHT = HEIGHT_STR.toInt();

  int X_SHIFT;
  int Y_SHIFT;

  if(X_COORD>48) {
    X_SHIFT = X_COORD-48;
  }
  else {
    X_SHIFT = 48-X_COORD;
  }
  if(Y_COORD>48) {
    Y_SHIFT = Y_COORD-48;
  }
  else {
    Y_SHIFT = 48-Y_COORD;
  }

  int X_SHIFT_ANGLE = (int)(atan(X_SHIFT/DISTANCE)*180.0*PI);
  int Y_SHIFT_ANGLE = (int)(atan(Y_SHIFT/DISTANCE)*180.0*PI);

  if(X_COORD>48) {
    CURRENT_PAN_ANGLE = CURRENT_PAN_ANGLE - X_SHIFT_ANGLE;
  }
  else {
    CURRENT_PAN_ANGLE = CURRENT_PAN_ANGLE + X_SHIFT_ANGLE;
  }

  if(Y_COORD>48) {
    CURRENT_TILT_ANGLE = CURRENT_TILT_ANGLE - Y_SHIFT_ANGLE;
  }
  else {
    CURRENT_TILT_ANGLE = CURRENT_TILT_ANGLE + Y_SHIFT_ANGLE;
  }

  HEAD_PAN.write(CURRENT_PAN_ANGLE);
  HEAD_TILT.write(CURRENT_TILT_ANGLE);
}

void PASSIVE_INTELLIGENCE() {
  TRANSMIT("CAM INFO");

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

bool ADVANCED_COMMAND(String INPUT) {

  // CAM -> Triggers CAM_FUNCS


  bool FAIL = false;
  String TAG = "";
  String RESULT = "":
  for(int Count = 0; Count<INPUT.length; Count++) {
    if(Count<3) {
      if(INPUT.charAt(Count)==' ') {
        FAIL = true;
        break;
      }
      else {
        TAG += INPUT.charAt(Count);
      }
    }
    if(Count==3) {
      if(INPUT.charAt(Count)!=' ') {
        FAIL = true;
      }
    }
    else {
      RESULT += INPUT.charAt(Count);
    }
  }
  if(RESULT == "") {
    FAIL = true;
  }
  if(FAIL) {
    ERR("INVALID ADVANCED COMMAND: " + INPUT);
  }
  else {
    if(TAG.equals("CAM")) {
      CAM_FUNCS(RESULT);
    }
  }

  return true;
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
  DISP_SEND
}







void setup() {

  BATTERY_CHECK();

  
  if(!CEASE) {

    // SERIAL MONITOR
    Serial.begin(115200);

    Serial.println("=================================================================");
    Serial.println("Made By Naim Iftekhar Rahman (24201190) BRAC University");
    Serial.println("=================================================================");
    delay(1500);

//=================
    // TROUBLESHOOTING BUTTON
    if(digitalRead(TROUBLESHOTING_PIN) == HIGH) {
      DEBUG_MODE = true;
    }
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

  // PGIO EXPANDERS
  if (!MOTORS.begin(0x20, &Wire)) {
    Serial.println(PART_ERR("MOTORS PCF"));
    while (1);
  }
  for (uint8_t PIN=0; PIN<8; PIN++) {
    MOTORS.pinMode(PIN, OUTPUT);
    MOTORS.digitalWrite(PIN, LOW);
    delay(10);
  }
  PRINT_LOG("MOTORS READY");
  delay(10);

  if (!STEPPERS.begin(0x20, &Wire)) {
    Serial.println(PART_ERR("STEPPERS PCF"));
    while (1);
  }
  for (uint8_t PIN=0; PIN<8; PIN++) {
    STEPPERS.pinMode(PIN, OUTPUT);
    STEPPERS.digitalWrite(PIN, LOW);
    delay(10);
  }
  PRINT_LOG("STEPPERS READY");
  delay(10);

//=================

    // SERVOS
    HEAD_PAN.attach(PAN_SERVO_PIN); // LEFT RIGHT SERVO
    HEAD_TILT.attach(TILT_SERVO_PIN); // UP DOWN SERVO

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
