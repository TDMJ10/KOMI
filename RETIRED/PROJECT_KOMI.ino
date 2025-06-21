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
    ========================                                               QTY: PRICE: ADDITIONAL NOTES:
    1: EXPRESSIF ESP32 AI-THINKER MICROCONTROLLER + OV2640 CAMERA          |2| | 600 | OBJECT RECOGNITION + FEED CAMERA
    2: EXPRESSIF ESP32 C340 DEVKIT-C MICROCONTROLLER                       |3| | 550 | REMOTE + ACCESS POINT + MOTOR CONTROL
    3: ESP32 30P EXPANSION BOARD (SLAVE)                                   |1| | 450 | 
    3: MPU6050 GYROSCOPE & ACCELEROMETER                                   |1| | 250 |
    3: ADAFRUIT SSD1306 0.96" 128x64 OLED DISPLAY                          |2| | 290 |
    4: ADAFRUIT 16x2 LIQUID CRYSTAL DISPLAY + I2C DISPLAY DRIVER           |1| | 340 |
    5: HC-SR04 ULTRASOUND SENSOR                                           |1| | 100 |
    4: SG90 MICRO-SERVO                                                    |6| | 135 |
    3: L293D MOTOR DRIVER                                                  |2| | 215 |
    5: N20 GEAR MOTOR                                                      |4| | 200 |
    7: N20 GEAR MOTOR WHEELS                                               |4| | 80  |
    8: LM2596S DC TO DC BUCK CONVERTER                                     |1| | 80  |
    6: 18650 LITHIUM ION BATTERY                                           |6| | 80  |
    9: 18650 BATTERY (x3) CELL HOLDER                                      |2| | 75  |
    9: FLAME SENSOR                                                        |1| | 70  | OPTIONAL
    9: MQ-2 FLAMMABLE GAS AND SMOKE SENSOR                                 |1| | 150 | OPTIONAL
    9: AD20P 1230P BRUSHLESS WATER PUMP                                    |1| | 750 | OPTIONAL

    TOTAL PROJECT COST: 600*2+550*3+450+250+290*2+340+100+135*6+215*2+200*4+80*4+80+80*6+75*2+70+150+750
                        = 8610 BDT
*/



//  ========================
//      GLOBAL VARIABLES:
//  ========================

// ESP NOW
String PREVIOUS_COMMAND = "ERROR";
String COMMAND = "ERROR";

// EDGE IMPULSE IMAGE RECOGNITION;
String OBJECT = "ERROR";

// TROUBLESHOOTING
bool STARTUP_CHECK_COMPLETE = false;
int LOG_NO = 1;

//  ========================
//          PRE SETUP:
//  ========================

#include <PROJECT_KOMI_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
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

// EDGE IMPULSE
#define CAMERA_MODEL_AI_THINKER
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);


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

// EDGE IMPULSE - CAMERA
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

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


//  ========================
//     OBJECT RECOGNITION
//  ========================

bool ei_camera_init(void) {
  if (is_initialised) return true;
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    #endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      IMPULSE_ERR("CAMERA INITIALIZATION FAILED: 0x%x", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, 0);
    }

  #if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
  #endif

  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    IMPULSE_ERR("CAMERA DEINITIALIZATION FAILED");
    return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
      IMPULSE_ERR("CAMERA NOT INITIALIZED");
      return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        IMPULSE_ERR("CAMERA CAPTURE FAILED");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       IMPULSE_ERR("IMAGE CONVERSION FAILED");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
      out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
      out_ptr_ix++;
      pixel_ix+=3;
      pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "INVALID MODEL FOR CURRENT SENSOR"
#endif

void TRIGGER_IMPULSE() {
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  if(snapshot_buf == nullptr) {
      ei_printf("ERR: Failed to allocate snapshot buffer!\n");
      return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
      ei_printf("Failed to capture image\r\n");
      free(snapshot_buf);
      return;
  }

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }
  
  IMPULSE_DATA_LOG("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    free(snapshot_buf);
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

//===============

  // EDGE IMPULSE
  if (ei_camera_init() == false) {
        IMPULSE_ERR("CAMERA INITIALIZATION FAILED");
  }
  else {
      PRINT_IMPULSE_LOG("CAMERA READY");
  }

//===============

  // BOOTUP SEQUENCE
  STARTUP_SEQUENCE();
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/

void loop() {
  // put your main code here, to run repeatedly:

}
