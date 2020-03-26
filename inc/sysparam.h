#ifndef _SYSPARAM_
#define _SYSPARAM_

#include "hal.h"
#include <time.h>
#include "ad7124_defs.h"
#include "adxl355_defs.h"
#include "bmi160_defs.h"



#define EEP_HEADING             0x45503032
#define EEP_BOOT_OFFSET         0x0
#define EEP_STORE_OFFSET        0x100
#define EEP_APP_OFFSET        0x200
#define EEP_USER_OFFSET        0x400

// mDNS  config
#define MDNS_SERVICE_1  "SensorHub-service-1._snn._tcp.local"
#define MDNS_SERVICE_2  "SensorHub-service-2._ipp._tcp.local"
#define MDNS_SERVICE_3  "SensorHub-service-3._ipp._tcp.local"

#define MDNS_TEXT_1     "dev=SENSORHub-VSS"
#define MDNS_TEXT_2     "dev=INERTIAL"
#define MDNS_TEXT_3     "vender=grididea.com.tw"

#define MDNS_PORT_1     5001
#define MDNS_PORT_2     600
#define MDNS_PORT_3     5353

#define TTLIVE             2000
#define UNIQUE_SERVICE  1
#define MAX_TEXT_LEN    120

typedef struct{
  uint8_t id[6];
  uint16_t offset;
  uint16_t sz;
  void *d;
}allocation_t;


typedef enum{
  SPAR_NONE,
  SPAR_EVEN,
  SPAR_ODD,
}com_parity_t;

typedef enum{
  SSTOP_1,
  SSTOP_1_5,
  SSTOP_2
}com_stopbit_t;

typedef enum{
  DATA_7 = 7,
  DATA_8
}com_databit_t;

enum baudrate_e{
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200
};

typedef struct{
  uint8_t slave_address;
  uint8_t baudrate_id;
  uint32_t baudrate_val;
  com_parity_t parity;
  com_stopbit_t stop;
  com_databit_t data;
}serial_setting_t;

typedef struct{
  uint8_t ip[6];
  uint8_t mask[6];
  uint8_t gateway[6];
  uint8_t macaddr[6];
}lan_setting_t;

typedef struct{
  uint32_t flag;
  uint8_t name[32];
  uint32_t verNum;
  uint32_t serialNum;
  uint8_t vender[32];
  uint8_t user[32];
}module_setting_t;

typedef struct{
  uint8_t wlan_mode;    // wifi-sta, wifi-ap, bt-slave, bt-master, etc..
  char prefix1[16];     // ap-mode ssid prefix, the ssid = ssid_prefix & serial number
  char passwd1[16];    // ap-mode password, default="53290921"
  char prefix2[16];     // sta-mode ssid
  char passwd2[16];    // sta-mode password, default="53290921"
  uint16_t connectionTimeout;   // timeout for searching available AP
  uint8_t secType;      // WEP-WPA2  
  uint8_t pairedInfo[32];
}wireless_param_t;



typedef enum{
  COMM_USE_BT = 0x40,
  COMM_USE_WIFI = 0x80
}comm_use_t;

typedef enum{
  BOOT_AP,
  BOOT_STATION,
}wifi_boot_t;


//
//typedef struct{
//  uint8_t active;
//  uint16_t capacity;
//  uint8_t soc_shutdown; 
//  uint16_t reportInterval;
//}battery_config_t;

typedef struct module_param_s{
  module_setting_t param;
  serial_setting_t serial;
  lan_setting_t lan;
  wireless_param_t wlan;
}module_params_t;

extern module_params_t moduleParam;

void sysSaveParams(uint8_t id);
uint8_t sysParamInit(void);

int8_t sys_read_param(uint8_t *name, void *d, uint16_t sz);
int8_t sys_write_param(uint8_t *name, void *d, uint16_t sz);

uint8_t sys_read_param_id(uint8_t id,uint8_t *d, uint16_t *sz);
uint8_t sys_write_param_id(uint8_t id,uint8_t *d, uint16_t sz);

#endif
