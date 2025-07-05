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


================================================================================================

  ██████╗ ██████╗  ██████╗      ██╗███████╗ ██████╗████████╗    ██╗  ██╗ ██████╗ ███╗   ███╗██╗
  ██╔══██╗██╔══██╗██╔═══██╗     ██║██╔════╝██╔════╝╚══██╔══╝    ██║ ██╔╝██╔═══██╗████╗ ████║██║
  ██████╔╝██████╔╝██║   ██║     ██║█████╗  ██║        ██║       █████╔╝ ██║   ██║██╔████╔██║██║
  ██╔═══╝ ██╔══██╗██║   ██║██   ██║██╔══╝  ██║        ██║       ██╔═██╗ ██║   ██║██║╚██╔╝██║██║
  ██║     ██║  ██║╚██████╔╝╚█████╔╝███████╗╚██████╗   ██║       ██║  ██╗╚██████╔╝██║ ╚═╝ ██║██║
  ╚═╝     ╚═╝  ╚═╝ ╚═════╝  ╚════╝ ╚══════╝ ╚═════╝   ╚═╝       ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝╚═╝
                                                                                             
================================================================================================


*/

//  ========================
//      GLOBAL VARIABLES:
//  ========================

// ESP NOW
String PREVIOUS_COMMAND = "ERROR";
String COMMAND = "ERROR";
bool STARTUP = true;

//  ========================
//          PRE SETUP:
//  ========================

#include <esp_now.h>
#include <WiFi.h>

// ESP NOW ADDRESSES
uint8_t MASTER_CONTROL[] = { 0x94, 0x54, 0xC5, 0xB5, 0xC6, 0x8C };

// DISPLAY EYES
#define LEFT_SCREEN_WIDTH 128
#define LEFT_SCREEN_HEIGHT 64
#define LEFT_OLED_RESET -1
#define LEFT_SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 LEFT_EYE(LEFT_SCREEN_WIDTH, LEFT_SCREEN_HEIGHT, &Wire, LEFT_OLED_RESET);

#define RIGHT_SCREEN_WIDTH 128
#define RIGHT_SCREEN_HEIGHT 64
#define RIGHT_OLED_RESET -1
#define RIGHT_SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 RIGHT_EYE(RIGHT_SCREEN_WIDTH, RIGHT_SCREEN_HEIGHT, &Wire, RIGHT_OLED_RESET);

// STATUS DISPLAY
LiquidCrystal_I2C STATUS_DISPLAY(0x27, 16,2);

void I2C_MULT(uint8_t BUS){
  Wire.beginTransmission(0x70);  // TCA9548A Address
  Wire.write(1 << BUS);
  Wire.endTransmission();
  Serial.print(BUS);
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
  bool ForDISP = true;
  String DSP = "DSP ";
  for(int Count = 0; Count<4; Count++) {
    if(DSP.charAt(Count)!=COMMAND.charAt(Count)) {
      ForDISP = false;
    }
  }
  if(ForDISP) {
    String Temp = "";
    for(int Count = 4; Count<COMMAND.length(); Count++) {
      Temp += COMMAND.charAt(Count);
    }
    COMMAND = Temp;
  }
  DISPLAY_DECODER(COMMAND);

  delay(10);
}

void ERR_TRANSMIT(String COMMAND) {
  PREVIOUS_COMMAND = COMMAND;
  esp_now_send(MASTER_CONTROL, (uint8_t *)COMMAND.c_str(), COMMAND.length() + 1);
  delay(10);
}

//  ========================
//       DISPLAY CONTROL
//  ========================

void DISPLAY_DECODER(String INPUT) {
  String DISPLAYS[] = {}
  int Index;
  bool Found = false;
  for(int Count = 0; Count<DISPLAYS.length; Count++) {
    if(INPUT.equals(DISPLAYS[Count])) {
      Found = true;
      Serial.println("| | DISPLAY_CONTROLLER> DISPLAYING:" + INPUT);
      Index = Count;
      break;
    }
  }
  if(Index==1) {
    // PLACEHOLDER
  }
}

//  ========================
//          DISPLAYS
//  ========================

void setup() {

  // SERIAL MONITOR
    Serial.begin(115200);

    Serial.println("=================================================================");
    Serial.println("Made By Naim Iftekhar Rahman (24201190) BRAC University");
    Serial.println("=================================================================");
    delay(1500);

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

}

void loop() {
  // put your main code here, to run repeatedly:
  if(STARTUP) {
    // Loading Screen Update Frames
  }
}
