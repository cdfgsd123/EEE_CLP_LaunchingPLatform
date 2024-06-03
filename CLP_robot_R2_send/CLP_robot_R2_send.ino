/*
  *@brief CLP robot firmware based on uPad_plant V1.4.0
  *@date 29 Feb 2024
  *@version 1.4.0
*/

//Components
#include "uPad_uart.h"
#include "uPad_i2c.h"
#include "uPad_ledc.h"
#include "uPad_crc.h"
#include "uPad_rmt.h"
#include "uPad_color.h"

//Using Udp for wireless communication
#include <WiFi.h>
#include <WiFiUdp.h>
const char *ssid = "Raymond";
const char *pwd = "raymondc129";
unsigned int localUdpPort = 8088;
WiFiUDP udp;

//Define

#define CMD_Starting 0x60

#define CMD_update_led 0x09
#define CMD_continue 0x0A
#define CMD_update_servo 0x0B
#define CMD_update_valve 0x0C
#define CMD_update_tof_en 0x0D
#define CMD_update_laser_en 0x0E

#define FB_update_led 0x99
#define FB_continue 0xAA
#define FB_update_servo 0xBB
#define FB_update_valve 0xCC
#define FB_update_tof_en 0xDD
#define FB_update_laser_en 0xEE

#define byte_to_send 26

#define error_invalid 0xF1
#define error_timeout 0xF2
#define error_crc 0xF3

#define tof_num 16
#define TOF_no_reading 0x9999
#define LASER_no_reading 0x9999

#define Servo_neutral_us 1500

#define max_fq_ms 50
#define timeout_ms 1000
typedef struct {
  uint8_t data[6];
} RX_ARRAY;
typedef struct {
  uint8_t data[byte_to_send];
} TX_ARRAY;
typedef struct {
  uint8_t tof[tof_num];
} TOF_ARRAY;
typedef struct {
  uint8_t type;
  uint32_t value;
  uPad_f state;
} CMD_STR;




//Queue
QueueHandle_t CmdMail;
QueueHandle_t TxQueue;
QueueHandle_t RxQueue;

QueueHandle_t RxQueue1;
QueueHandle_t RxQueue2;
QueueHandle_t TxQueue1;

QueueHandle_t SvoMail;
QueueHandle_t TofMail;
QueueHandle_t LasMail;
QueueHandle_t ValMail;
QueueHandle_t ImuMail;
//Semaphore
SemaphoreHandle_t UartMutex;
//Tasks
void UART0(void *pvNull);
void UART1(void *pvNull);
void I2C0(void *pvNull);
void I2C1(void *pvNull);
void SERVO(void *pvNull);  //LEDC0
void SORTER(void *pvNull);
void STACKER(void *pvNull);
void RGBLED(void *pvNull);
//Subtasks
void TOF(void *pvUART);    //UART0
void LASER(void *pvUART);  //UART0
void IMU(void *pvI2C);     //I2C0
void VALVE(void *pvI2C);   //I2C1
void UDP(void *pvNull);
//app_main
void setup() {
  // Create queue
  CmdMail = xQueueCreate(1, sizeof(CMD_STR));
  TxQueue = xQueueCreate(1, sizeof(TX_ARRAY));
  RxQueue = xQueueCreate(1, sizeof(RX_ARRAY));
  RxQueue1 = xQueueCreate(1, sizeof(RX_ARRAY));
  RxQueue2 = xQueueCreate(1, sizeof(RX_ARRAY));
  TxQueue1 = xQueueCreate(1, sizeof(TX_ARRAY));
  //Feedback
  SvoMail = xQueueCreate(1, sizeof(LEDC_SOURCE));
  TofMail = xQueueCreate(1, sizeof(TOF_ARRAY));
  LasMail = xQueueCreate(1, sizeof(uint16_t));
  ValMail = xQueueCreate(1, sizeof(I2C_SOURCE));
  ImuMail = xQueueCreate(1, sizeof(uint8_t));
  // Create tasks
  xTaskCreate(UART0, "Debug_serial", (1024) * 4, NULL, 1, NULL);
  xTaskCreate(SORTER, "Sorter", (1024) * 2, NULL, 3, NULL);
  xTaskCreate(STACKER, "Stacker", (1024) * 2, NULL, 3, NULL);
  xTaskCreate(UART1, "Communicate_serial", (1024) * 4, NULL, 1, NULL);
  xTaskCreate(I2C0, "Imu_i2c", (1024) * 4, NULL, 1, NULL);
  xTaskCreate(I2C1, "Valve_i2c", (1024) * 4, NULL, 1, NULL);
  xTaskCreate(SERVO, "Servo_driver", (1024) * 2, NULL, 2, NULL);
  xTaskCreate(RGBLED, "LED_indicator", (1024) * 4, NULL, 1, NULL);
  xTaskCreate(UDP, "Wireless_Serial", (1024) * 4, NULL, 1, NULL);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pwd);
  udp.begin(localUdpPort);
}

//idle
void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));
}
//Tasks
void UART0(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_UART uart0(UART_NUM_0, ESP_INTR_FLAG_LEVEL2, uPad_pin_uart0Txd, uPad_pin_uart0Rxd);
  UART_CONFIG config = { .tx_byte_limit = byte_to_send, .rx_byte_limit = 1 };
  RX_ARRAY from_user;
  TX_ARRAY to_user;
  //Initialize
  uint8_t count = 0;
  uart0.INIT(&config);
  //Infinite loop
  for (;;) {
    //Read
    while ((count < 5) && (uart0.GET() == uPad_DONE)) {
      if (from_user.data[0] == CMD_Starting) {
        uart0.EXP(&from_user.data[++count]);
      } else {
        uart0.EXP(&from_user.data[0]);
      }
    }

    if (count == 5) {
      xQueueSend(RxQueue, &from_user, portMAX_DELAY);
      
      from_user.data[0] = 0;
      count = 0;
    }

    //Write
    if (xQueueReceive(TxQueue, &to_user, 0) == pdPASS) {
      uart0.ENCODE(&to_user.data[0], &upad_null);
    }
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void SORTER(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  RX_ARRAY from_user;
  CMD_STR cmd;
  //Infinite loop
  for (;;) {
    //From Uart
    if (xQueueReceive(RxQueue2, &from_user, pdMS_TO_TICKS(timeout_ms)) != pdPASS) {
      cmd.type = error_timeout;
      cmd.state = uPad_ERROR;
    } else {
      //Decode
      cmd.type = from_user.data[1];
      cmd.value = from_user.data[2] << 8 * 3 | from_user.data[3] << 8 * 2 | from_user.data[4] << 8 * 1 | from_user.data[5] << 8 * 0;
      switch (cmd.type) {
        case CMD_continue:
          cmd.type = FB_continue;
          cmd.state = uPad_DONE;
          break;
        case CMD_update_led:  //set 1 to enable, from lowest bit
          cmd.state = uPad_BUSY;
          break;
        case CMD_update_servo:  //0-255, 4-channel, neutral at 128
          cmd.state = uPad_BUSY;
          break;
        case CMD_update_valve:  //set 1 to on, lowest byte
          cmd.state = uPad_BUSY;
          break;
        case CMD_update_tof_en:  //set 1 to enable, from lowest bit
          cmd.state = uPad_BUSY;
          break;
        case CMD_update_laser_en:  //set 1 to enable, lowest bit
          cmd.state = uPad_BUSY;
          break;
        default:
          cmd.type = error_invalid;
          cmd.state = uPad_ERROR;
      }
    }
    xQueueOverwrite(CmdMail, &cmd);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void STACKER(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  CMD_STR cmd;
  LEDC_SOURCE servo;
  I2C_SOURCE valve_state;
  TOF_ARRAY tof_dis;
  uint16_t laser_dis;
  TX_ARRAY to_uart;
  uint8_t roll;
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    if (cmd.state == uPad_DONE || cmd.state == uPad_ERROR) {
      //Feedback From Module
      xQueuePeek(SvoMail, &servo, 0);
      xQueuePeek(TofMail, &tof_dis, 0);
      xQueuePeek(LasMail, &laser_dis, 0);
      xQueuePeek(ValMail, &valve_state, 0);
      xQueuePeek(ImuMail, &roll, 0);
      //Encode
      uint8_t pos = 0;
      to_uart.data[pos++] = CMD_Starting;
      to_uart.data[pos++] = cmd.type;
      for (int i = 0; i < 4; i++) {
        to_uart.data[pos++] = servo.duty[i];
      }
      for (int i = 0; i < 16; i++) {
        to_uart.data[pos++] = tof_dis.tof[i];
      }
      to_uart.data[pos++] = (uint8_t)(laser_dis >> 8);
      to_uart.data[pos++] = (uint8_t)laser_dis;
      to_uart.data[pos++] = valve_state;
      to_uart.data[pos++] = roll;
      //To Uart
      xQueueSend(TxQueue, &to_uart, 0);
      xQueueSend(TxQueue1, &to_uart, 0);
      //End process
      cmd.state = uPad_BUSY;
      xQueueOverwrite(CmdMail, &cmd);
    }
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}


//Change
void UART1(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_UART uart1(UART_NUM_1, ESP_INTR_FLAG_LEVEL2, GPIO_NUM_40, GPIO_NUM_39);
  UART_CONFIG config = { .tx_byte_limit = 6, .rx_byte_limit = 1 };
  RX_ARRAY from_user;
  TX_ARRAY to_user;

  //Initialize
  uint8_t count = 0;
  uart1.INIT(&config);

  for (;;) {
    //Write

    if (xQueueReceive(RxQueue1, &from_user, pdMS_TO_TICKS(timeout_ms)) == pdPASS) {
      uart1.ENCODE(&from_user.data[0], &upad_null);
    }
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void I2C0(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_I2C i2c0(I2C_NUM_0, ESP_INTR_FLAG_LEVEL2, uPad_pin_i2c0Sda, uPad_pin_i2c0Scl);
  //Initialize
  i2c0.INIT();
  xTaskCreate(IMU, "IMU_driver", (1024) * 4, &i2c0, 1, NULL);
  //Infinite loop
  for (;;) {
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, portMAX_DELAY);
  }
  //Break out
  vTaskDelete(NULL);
}

void I2C1(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_I2C i2c1(I2C_NUM_1, ESP_INTR_FLAG_LEVEL2, uPad_pin_gpio35, uPad_pin_gpio36);
  //Initialize
  i2c1.INIT();
  xTaskCreate(VALVE, "Valve_driver", (1024) * 2, &i2c1, 1, NULL);
  //Infinite loop
  for (;;) {

    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, portMAX_DELAY);
  }
  //Break out
  vTaskDelete(NULL);
}

void SERVO(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  CMD_STR cmd;
  UPAD_LEDC ledc(LEDC_TIMER_0, ESP_INTR_FLAG_LEVEL1, uPad_pin_gpio13, uPad_pin_gpio14, uPad_pin_gpio15, uPad_pin_gpio16);
  LEDC_SOURCE servo;
  LEDC_CONFIG config = { .frequency_i = 50, .resolution_i = LEDC_TIMER_12_BIT, .fade_step_duty = 2 };
  //Initialize
  ledc.INIT(&config);
  for (int i = 0; i < 4; i++) servo.duty[i] = Servo_neutral_us * (1 << config.resolution_i) / 1000 * config.frequency_i / 1000;  //us to tick
  ledc.ENCODE(&servo, &upad_null);
  config.fade_en = 0;
  ledc.SYNC(&config, &upad_null);
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    //Timeout
    if (cmd.type == error_timeout) {
      for (int i = 0; i < 4; i++) servo.duty[i] = Servo_neutral_us * (1 << config.resolution_i) / 1000 * config.frequency_i / 1000;  //us to tick
      ledc.ENCODE(&servo, &upad_null);
    }
    //Write
    if (cmd.type == CMD_update_servo) {
      for (int i = 0; i < 4; i++) servo.duty[i] = (uint8_t)(cmd.value >> 8 * (3 - i)) + 179;  //byte to servo range tick
      ledc.ENCODE(&servo, &upad_null);
      cmd.state = uPad_DONE;
      cmd.type = FB_update_servo;
      xQueueOverwrite(CmdMail, &cmd);
    }
    //Read
    ledc.DECODE(&servo, &upad_null);
    for (int i = 0; i < 4; i++) {
      servo.duty[i] = servo.duty[i] - 179;  //servo range tick to byte
    }
    xQueueOverwrite(SvoMail, &servo);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void RGBLED(void *pvNull) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  CMD_STR cmd;
  UPAD_RMT rgbled(GPIO_NUM_48, GPIO_NUM_NC);
  RMT_CONFIG config = { .resolution_hz_i = 20 * 1000 * 1000 };
  RMT_SOURCE code_low = 1, code_high = 2, code_break = 4;
  RMT_SOURCE led_code[24];
  UPAD_COLOR hex2rgb;
  *hex2rgb.config = uPad_col_RGB;
  UPAD_COLOR hex2hsv;
  *hex2hsv.config = uPad_col_HSV;
  COLOR_SINK rgb_val, hsv_val = { 128, 255, 0 };
  //Initialize
  config.rmt_symbol[0] = {
    .duration0 = 0.4 * 20,
    .level0 = 1,
    .duration1 = 0.85 * 20,
    .level1 = 0,
  };
  config.rmt_symbol[1] = {
    .duration0 = 0.8 * 20,
    .level0 = 1,
    .duration1 = 0.45 * 20,
    .level1 = 0,
  };
  config.rmt_symbol[2] = {
    .duration0 = 0,
    .level0 = 1,
    .duration1 = 10 * 20,
    .level1 = 0,
  };
  rgbled.INIT(&config);
  int counter, step = 0;
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    if (counter == 0) {
      for (int lednum = 0; lednum < 8; lednum++) {
        if (cmd.type == error_timeout || cmd.type == error_invalid || cmd.type == error_timeout) {
          //Error Sign
          rgb_val.col_val[0] = (uint8_t)(0x80 - 0x08 * abs(step));
          rgb_val.col_val[1] = (uint8_t)(0x08 * abs(step));
          rgb_val.col_val[2] = (uint8_t)(0x08 * abs(step));
          rgb_val.col_val[3] = 0;
        } else {
          //HSV to RGB
          hsv_val.col_val[2] = (uint8_t)(0xF0 >> abs(abs(step) - lednum));
          hex2hsv.GET(&hsv_val);
          hex2hsv.EXP();
          *hex2rgb.source = *hex2hsv.source;
          hex2rgb.IMP();
          hex2rgb.SET(&rgb_val);
        }
        //RGB to Color_code
        for (int j = 0; j < 3; j++) {
          for (int i = 0; i < 8; i++) {
            if (rgb_val.col_val[(4 - j) % 3] & (1 << (8 - i))) {
              led_code[i + 8 * j] = code_high;
            } else {
              led_code[i + 8 * j] = code_low;
            }
          }
        }
        rgbled.IMP(led_code, 24);
        rgbled.IMP(&code_break);
        rgbled.SET();
      }
    }
    //Write
    if (cmd.type == CMD_update_led) {
      hsv_val.col_val[0] = (uint8_t)cmd.value;
      cmd.state = uPad_DONE;
      cmd.type = FB_update_led;
      xQueueOverwrite(CmdMail, &cmd);
    }
    //Enter blocked stage and wait
    counter++;
    if (counter > 50 / max_fq_ms) {  // 50 ms
      counter = 0;
      step++;
      if (step > 7) {
        step = -7;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

//Subtasks
void TOF(void *pvUART) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_UART *uart = (UPAD_UART *)pvUART;
  UPAD_CRC tof_crc;
  CRC_CONFIG crc_config = { .crc_byte_width_i = 2, .crc_poly_i = 0x8005, .init_val_i = 0xFFFF, .crc_reflect = true, .xor_out = 0x0000 };
  CRC_SINK crc;
  CMD_STR cmd;
  TOF_ARRAY tof_sel = { 0 };
  TOF_ARRAY tof_dis = { 0 };
  uint8_t tof_cmd[8] = { 0 };
  const uint8_t cmd_write_range[8] = { 0x00, 0x06, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00 };
  const uint8_t cmd_write_offset[8] = { 0x00, 0x06, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00 };
  const uint8_t cmd_read_dis[8] = { 0x00, 0x03, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00 };
  //^^^ToF modbus Protocol:        ^ID   ^Read ^Reg_H^Reg_L^D_H  ^D_L  ^CRC_L^CRC_H
  //Initialize
  tof_crc.INIT(&crc_config);
  xSemaphoreTake(UartMutex, portMAX_DELAY);
  //Write cmd
  uart->config->tx_byte_limit = 8;
  tof_crc.config->data_byte_length = 6;
  //Init range
  for (int tof_count = 0; tof_count < tof_num; tof_count++) {
    for (int i = 0; i < 8; i++) tof_cmd[i] = cmd_write_range[8];
    tof_cmd[0] = tof_count + 1;     //overwrite id
    tof_crc.ENCODE(tof_cmd, &crc);  //calculate CRC
    tof_cmd[6] = crc.crc_byte[0];
    tof_cmd[7] = crc.crc_byte[1];
    uart->ENCODE(tof_cmd, &upad_null);
    vTaskDelay(pdMS_TO_TICKS(5));  //Wait for TOF ready
  }
  //Init offset
  for (int tof_count = 0; tof_count < tof_num; tof_count++) {
    for (int i = 0; i < 8; i++) tof_cmd[i] = cmd_write_offset[8];
    tof_cmd[0] = tof_count + 1;     //overwrite id
    tof_crc.ENCODE(tof_cmd, &crc);  //calculate CRC
    tof_cmd[6] = crc.crc_byte[0];
    tof_cmd[7] = crc.crc_byte[1];
    uart->ENCODE(tof_cmd, &upad_null);
    vTaskDelay(pdMS_TO_TICKS(5));  //Wait for TOF ready
  }
  xSemaphoreGive(UartMutex);
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    //Update tof_enable_state
    if (cmd.type == CMD_update_tof_en) {
      for (int i = 0; i < tof_num; i++) {
        if (cmd.value & (1 << i)) {
          tof_sel.tof[i] = 1;
        } else {
          tof_sel.tof[i] = 0;
        }
      }
      //Update status
      cmd.state = uPad_DONE;
      cmd.type = FB_update_tof_en;
      xQueueOverwrite(CmdMail, &cmd);
    }
    xSemaphoreTake(UartMutex, portMAX_DELAY);
    xQueuePeek(TofMail, &tof_dis, 0);
    //Write cmd
    uart->config->tx_byte_limit = 8;
    tof_crc.config->data_byte_length = 6;
    for (int tof_count = 0; tof_count < tof_num; tof_count++) {
      if (tof_sel.tof[tof_count]) {
        for (int i = 0; i < 8; i++) tof_cmd[i] = cmd_read_dis[i];
        tof_cmd[0] = tof_count + 1;     //overwrite id
        tof_crc.ENCODE(tof_cmd, &crc);  //calculate CRC
        tof_cmd[6] = crc.crc_byte[0];
        tof_cmd[7] = crc.crc_byte[1];
        uart->ENCODE(tof_cmd, &upad_null);
        vTaskDelay(pdMS_TO_TICKS(5));             //Wait for TOF ready
      } else {                                    //Clear the unwanted
        tof_dis.tof[tof_count] = TOF_no_reading;  //Clear reading
      }
    }
    xQueueOverwrite(TofMail, &tof_dis);
    xSemaphoreGive(UartMutex);
    //Enter blocked stage and wait
    vTaskDelay(pdMS_TO_TICKS(10));
    //vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
  //Break out
  vTaskDelete(NULL);
}

void LASER(void *pvUART) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_UART *uart = (UPAD_UART *)pvUART;
  UPAD_CRC laser_crc;
  CRC_CONFIG crc_config = { .crc_byte_width_i = 2, .crc_poly_i = 0x8005, .init_val_i = 0xFFFF, .crc_reflect = true, .xor_out = 0x0000 };
  CRC_SINK crc;
  CMD_STR cmd;
  bool laser_en = 1;
  uint16_t laser_dis = 0;
  uint8_t laser_cmd[8] = { 0 };
  const uint8_t cmd_read_dis[8] = { 0x00, 0x03, 0x00, 0x0F, 0x00, 0x02, 0x00, 0x00 };
  //^^^ToF modbus Protocol:        ^ID   ^Read ^Reg_H^Reg_L^D_H  ^D_L  ^CRC_L^CRC_H
  //Initialize
  laser_crc.INIT(&crc_config);
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    //Update laser_enable_state
    if (cmd.type == CMD_update_laser_en) {
      if (cmd.value & (1)) {
        laser_en = 1;
      } else {
        laser_en = 0;
      }
      //Update status
      cmd.state = uPad_DONE;
      cmd.type = FB_update_laser_en;
      xQueueOverwrite(CmdMail, &cmd);
    }
    xSemaphoreTake(UartMutex, portMAX_DELAY);
    xQueuePeek(LasMail, &laser_dis, 0);
    //Write cmd
    uart->config->tx_byte_limit = 8;
    laser_crc.config->data_byte_length = 6;
    if (laser_en) {
      for (int i = 0; i < 8; i++) laser_cmd[i] = cmd_read_dis[i];
      laser_cmd[0] = 0x01;                //overwrite id
      laser_crc.ENCODE(laser_cmd, &crc);  //calculate CRC
      laser_cmd[6] = crc.crc_byte[0];
      laser_cmd[7] = crc.crc_byte[1];
      uart->ENCODE(laser_cmd, &upad_null);
      vTaskDelay(pdMS_TO_TICKS(5));  //Wait for TOF ready
    } else {                         //Clear the unwanted
      laser_dis = LASER_no_reading;  //Clear reading
    }
    xQueueOverwrite(LasMail, &laser_dis);
    xSemaphoreGive(UartMutex);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
  //Break out
  vTaskDelete(NULL);
}

void GET_VAL(void *pvUART) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  UPAD_UART *uart = (UPAD_UART *)pvUART;
  UPAD_CRC check;
  CRC_CONFIG crc_config = { .crc_byte_width_i = 2, .crc_poly_i = 0x8005, .init_val_i = 0xFFFF, .crc_reflect = true, .xor_out = 0x0000 };
  CRC_SINK crc;
  TOF_ARRAY tof_dis = { 0 };
  uint16_t laser_dis = 0;
  uint8_t fbk[10] = { 0 };
  //Initialize
  check.INIT(&crc_config);
  //Infinite loop
  for (;;) {
    xSemaphoreTake(UartMutex, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5));
    xQueuePeek(TofMail, &tof_dis, 0);
    xQueuePeek(LasMail, &laser_dis, 0);
    //Read cmd
    uart->config->rx_byte_limit = 1;
    while (uart->DECODE(&fbk[9], &upad_null) == uPad_DONE) {
      for (int i = 0; i < 9; i++) {  //Shift registor
        fbk[i] = fbk[i + 1];
      }
      check.config->data_byte_length = 5;
      check.ENCODE(fbk, &crc);
      if ((fbk[5] == crc.crc_byte[0]) && (fbk[6] == crc.crc_byte[1])) {  //ToF CRC
        if (fbk[3] != 0) {
          tof_dis.tof[fbk[0] - 1] = 0xff;
        } else {
          tof_dis.tof[fbk[0] - 1] = fbk[4];
        }
      }
      check.config->data_byte_length = 7;
      check.ENCODE(fbk, &crc);
      if ((fbk[7] == crc.crc_byte[0]) && (fbk[8] == crc.crc_byte[1])) {  //Laser CRC
        if (fbk[3] != 0 || fbk[4] != 0) {
          laser_dis = 0xffff;
        } else {
          laser_dis = (fbk[5] << 8 | fbk[6]);
        }
      }
    }
    xQueueOverwrite(TofMail, &tof_dis);
    xQueueOverwrite(LasMail, &laser_dis);
    xSemaphoreGive(UartMutex);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void VALVE(void *pvI2C) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  CMD_STR cmd;
  UPAD_I2C *i2c = (UPAD_I2C *)pvI2C;
  I2C_SOURCE i2c_byte[2] = { 0, 0 };
  I2C_CONFIG config = { .tx_byte = 2, .i2c_addr = I2c_status_addr };
  //Initialize
  i2c->INIT(&config);
  i2c_byte[0] = I2c_status_set_io;
  i2c_byte[1] = 0b00000000;
  i2c->ENCODE(i2c_byte, &upad_null);
  i2c_byte[0] = I2c_status_set_polar;
  i2c_byte[1] = 0b00000000;
  i2c->ENCODE(i2c_byte, &upad_null);
  i2c_byte[0] = I2c_status_set;
  i2c_byte[1] = 0b00000000;
  i2c->ENCODE(i2c_byte, &upad_null);
  //Infinite loop
  for (;;) {
    xQueuePeek(CmdMail, &cmd, 0);
    if (cmd.type == CMD_update_valve) {
      i2c_byte[0] = I2c_status_set;
      i2c_byte[1] = cmd.value & 0x000000ff;
      i2c->ENCODE(i2c_byte, &upad_null);
      //Update status
      cmd.state = uPad_DONE;
      cmd.type = FB_update_valve;
      xQueueOverwrite(CmdMail, &cmd);
    }
    xQueueOverwrite(ValMail, &i2c_byte[1]);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void IMU(void *pvI2C) {
  //Variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  CMD_STR cmd;
  UPAD_I2C *i2c = (UPAD_I2C *)pvI2C;
  I2C_SOURCE i2c_byte[6] = { 0, 0 };
  I2C_CONFIG config = { .tx_byte = 2, .i2c_addr = I2c_gyro_addr };
  float AccX = 0, AccY = 0, AccZ = 0;
  uint8_t roll = 0;
  //Initialize
  i2c->INIT(&config);
  i2c_byte[0] = I2c_gyro_set_pow1;
  i2c_byte[1] = 0b00000000;
  i2c->ENCODE(i2c_byte, &upad_null);
  i2c->config->tx_nonstop = 1;
  i2c->config->tx_byte = 1;
  i2c->config->rx_byte = 6;
  //Infinite loop
  for (;;) {
    i2c_byte[0] = I2c_gyro_get_accelXH;
    i2c->ENCODE(i2c_byte, &upad_null);
    i2c->DECODE(i2c_byte, &upad_null);
    AccX = AccX * 0.95 + (int16_t)(i2c_byte[0] << 8 | i2c_byte[1]) / 16384.0f * 0.05;
    AccY = AccY * 0.95 + (int16_t)(i2c_byte[2] << 8 | i2c_byte[3]) / 16384.0f * 0.05;
    AccZ = AccZ * 0.95 + (int16_t)(i2c_byte[4] << 8 | i2c_byte[5]) / 16384.0f * 0.05;
    if (abs(AccZ) > abs(AccX)) {
      roll = atan(AccX / AccZ) / 2 / 3.1415926 * 256;
      if (AccZ < 0) roll = roll + 256 / 2;
    } else {
      roll = 256 / 4 - (atan(AccZ / AccX) / 2 / 3.1415926) * 256;
      if (AccX < 0) roll = roll + 256 / 2;
    }
    roll = (uint8_t)round(roll);
    xQueueOverwrite(ImuMail, &roll);
    //Enter blocked stage and wait
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  //Break out
  vTaskDelete(NULL);
}

void UDP(void *pvNull) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  RX_ARRAY from_user;
  TX_ARRAY to_user;

  for (;;) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char incomingPacket[255];
      int len = udp.read(incomingPacket, 255);

      if (len > 0) {
        incomingPacket[len] = 0;

        if (len > sizeof(from_user.data)) {
          len = sizeof(from_user.data);
        }

        // Copy the data to from_user.data
        memcpy(from_user.data, incomingPacket, len);
      }
      xQueueSend(RxQueue1, &from_user, portMAX_DELAY);
      xQueueSend(RxQueue2, &from_user, portMAX_DELAY);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      if (xQueueReceive(TxQueue1, &to_user, 0) == pdPASS) {
        udp.write((const uint8_t*)to_user.data, sizeof(to_user.data));
      }

      udp.endPacket();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(max_fq_ms));
  }
  vTaskDelete(NULL);
}
