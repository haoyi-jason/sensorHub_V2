#ifndef _SENSOR_HUB_
#define _SENSOR_HUB_

#include "sysParam.h"
//#include "mmssocket.h"
#include "battery_mang.h"
//#include "socket.h"
#include "ff.h"

#define APP_USE_RSI_WIFI
#define APP_USE_RSI_BT

#ifdef APP_USE_RSI_BT
#include "rsi_bt_common.h"
#include "rsi_bt_config.h"
#endif

#if defined(APP_USE_RSI_WIFI) || defined(APP_USE_RSI_BT)
#define APP_USE_RSI
#endif


#define SW_VERSION_NUMBER       0x20032501
#define DATA_PATH_TRANMIT       0x01
#define DATA_PATH_LOGSD         0x02

#define BUFFER_SZ       1024

#define EV_ADXL_FIFO_FULL EVENT_MASK(0)

#define EV_EXT_INT              EVENT_MASK(0)
#define EV_TIMEOUT              EVENT_MASK(1)
#define EV_CMD_RUN              EVENT_MASK(2)
#define EV_CMD_STOP             EVENT_MASK(3)
#define EV_ADC_RESTART          EVENT_MASK(4)
#define EV_ACT_BATT             EVENT_MASK(5)
#define EV_CMD_SINGLE           EVENT_MASK(6)
#define EV_BMI_INT1             EVENT_MASK(7)
#define EV_BMI_INT2             EVENT_MASK(8)
#define EV_BUFFER_FULL          EVENT_MASK(9)
#define EV_P7_EXINT             EVENT_MASK(10)
#define EV_BT_EXTINT             EVENT_MASK(11)
#define EV_SAVE_PARAM           EVENT_MASK(12)
#define EV_LOAD_PARAM           EVENT_MASK(13)
#define EV_CMD_RESET           EVENT_MASK(14)
#define EV_CMD_EXTINT           EVENT_MASK(15)
#define EV_SDMMC_INSERT         EVENT_MASK(16)
#define EV_USER_BUTTON_0        EVENT_MASK(17)
#define EV_CMD_READ_BATT        EVENT_MASK(18)
#define EV_CMD_READ_ENV        EVENT_MASK(19)

#define EV_SD_WRITE_0           EVENT_MASK(20)
#define EV_SD_WRITE_1           EVENT_MASK(21)

#define EV_FSWRITE_DATA         EVENT_MASK(22)
#define EV_FSWRITE_DATA_HI         EVENT_MASK(23)
#define EV_FSWRITE_LOG         EVENT_MASK(24)
#define EV_CLIENT_CONNECT  EVENT_MASK(25)
#define EV_CLIENT_DISCONNECT  EVENT_MASK(26)
#define EV_CMD_RX               EVENT_MASK(27)
#define EV_PERIODIC             EVENT_MASK(28)
#define EV_BT_ERROR             EVENT_MASK(29)
#define EV_SYS_SAVE_PAIRED             EVENT_MASK(30)


#define POWER_IF_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_IF)
#define POWER_IF_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_IF)
#define POWER_P7_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_P7)
#define POWER_P7_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_P7)
#define POWER_P8_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_P8)
#define POWER_P8_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_P8)

typedef int8_t (*write_func)(uint8_t*,uint16_t n);
typedef int16_t (*stream_io)(uint8_t*, size_t);

typedef struct{
  uint16_t sampleNumber;
  uint16_t samplePeriodMs;
}time_domain_param_t;

typedef struct{
  uint8_t window;
  uint8_t overlap;
  uint16_t bins; 
}freq_domain_param_t;

enum {
  OP_STREAM,
  OP_VNODE,
  OP_FNODE,
  OP_OLED,
  OP_LOGSD,
};
enum {
  COMM_BT,
  COMM_WIFI,
  COMM_MBRTU_OVER_BT,
  COMM_MBRTU_OVER_SERIAL,
  COMM_LORA
};

#define ADXL_ENABLED    EVENT_MASK(0)
#define BMI160_ENABLED  EVENT_MASK(1)
#define SD_ENABLED      EVENT_MASK(2)




typedef struct {
  uint8_t opMode;
  uint8_t commType;
  uint8_t activeSensor;
}node_param_t;

typedef struct{
  uint8_t power;
  uint8_t odr;
  uint8_t range;
  uint8_t lpf;
}_triaxis_sensor_t;

typedef struct{
  _triaxis_sensor_t accel,gyro;
}imu_config_t;

typedef enum{
  SEN_IDLE,
  SEN_SINGLE,
  SEN_RUNNING
}sensor_state_t;


typedef struct{
  int32_t x;
  int32_t y;
  int32_t z;
}adxl_data_t;

typedef struct{
  ioportid_t port;
  uint16_t pad;
}io_config_t;

typedef struct{
  char data[64];
  char data_hr[64];
  char log[64];
}log_filename_t;

typedef struct{
  FIL f;
  char name[64];
  uint32_t lastSync;
}log_file_info_t;

enum {
  DATA,
  DATA_HIR,
  LOG
};

//typedef struct{
//  char path[256];
//  char fileName[64];
//  uint32_t offset;
//  uint32_t nofBytes;
//  uint32_t byteToRead;
//  uint8_t dirChanged;
//}file_op_t;



#define SD_BUFFER_SIZE  2048 // push CMD_STRUCT_SZ in the front of buffer


typedef struct{
  uint8_t *dptr;
  size_t sz;
}_fs_msg_t;


typedef void (*rsi_interrupt_cb)(void);
typedef struct{
  uint32_t rsi_app_async_event_map;
#ifdef APP_USE_RSI_BT
  rsi_bt_resp_get_local_name_t local_name;
  uint8_t local_dev_addr[RSI_DEV_ADDR_LEN];
#endif
  uint8_t str_conn_bd_addr[18];
}rsi_bt_data_t;

typedef struct{
  thread_t *rsi_bt;
  thread_t *rsi_wlan;
  thread_t *rsi_driver;
}rsi_handle_t;

typedef struct{
  uint16_t ms_on;
  uint16_t ms_off;
}_led_blink_t;

#ifndef BUFFER_SZ
#define BUFFER_SZ       320
#endif

typedef struct{
  int16_t sz;
  uint8_t buffer[BUFFER_SZ];
}buffer_t;

typedef struct{
  uint8_t savdSd;
  uint8_t prefix[16];
  uint32_t szConstrain;
  uint32_t capacity;
}sd_config_t;

typedef struct{
  uint8_t yy;
  uint8_t mm;
  uint8_t dd;
  uint8_t hh;
  uint8_t nn;
  uint8_t ss;
}rtc_config_t;

typedef struct{
  int16_t batv[2];
}battery_config_t;

typedef struct{
  float temp;
  float rh;
}trh_sensor_t;

//typedef struct{
//  uint8_t nofSensor;
//  trh_sensor_t sensor[5];
//};

typedef struct{
  _led_blink_t ledBlink;
  virtual_timer_t blinker;
  thread_t *mainThread;
  //thread_t *workingThread;
  //thread_t *serialThread;
  //thread_t *fSThread;
  //thread_t *transThread;
  thread_t *bufferThread;
  struct tm tim_now;
  //dev_type_t sensor[6];
  //log_filename_t fileName;
  //uint8_t pid[3];
  //file_op_t activeFile;
  uint8_t dataPath;
  int8_t tmp;
  //uint16_t flushSize;
  //buffer_t wbuf,rbuf;
  //uint16_t wrIndex;
  uint32_t ipAddr;
  uint32_t gwAddr;
  uint8_t macAddr[6];
  uint32_t runSecs;
  //int8_t maxCh;
  //int8_t chList[8];
  mutex_t mtx;
  mutex_t mtx_fs;
  //mutex_t mtx_uart;
  uint8_t singleRead;
  //batt_spec_t battery;
  double temp, humidity;
  int16_t battery_mv[2];
  trh_sensor_t trh[5];
  uint8_t user_press_inc;
  uint8_t user_pressed_sec;
  uint8_t user_cfg_mode;
  //size_t lig_file_size[3];
  //log_file_info_t data_file,data_file_hi,log_file;
  systime_t writeTime;
  //sd_buffer_t sdBuffer;
  uint8_t connState;
  uint32_t rate;
  rsi_interrupt_cb rsi_int;
  rsi_bt_data_t rsi_bt;
  rsi_handle_t rsi_handle;
  int32_t clientSocket;
  buffer_t rxBuf,txBuf;
  int16_t vbat_mv;
  //uint8_t linkKey[16];
  node_param_t node;
  adxl355_config_t adxl;
  time_domain_param_t time;
  freq_domain_param_t freq;
  imu_config_t imu;
//  bmi160_config_t bmicfg;
  stream_io writefcn;
  stream_io readfcn;
  sd_config_t sdConfig;
}app_param_t;


extern thread_t *mainThread;
extern app_param_t appParam;

void cmdSetup(BaseSequentialStream *chp, int argc, char *argv[]);
void cmdControl(BaseSequentialStream *chp, int argc, char *argv[]);
void cmdNull(BaseSequentialStream *chp, int argc, char *argv[]);

void app_loadDefault();
void sensorHubInit(void);
#endif