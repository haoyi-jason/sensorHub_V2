#include "ch.h"
#include "hal.h"
#include "sysParam.h"
#include "at24_eep.h"
#include "string.h"
#include "sensorhub.h"


module_params_t moduleParam;
const serial_setting_t serial_default = {
  1,
  BAUD_19200,
  19200,
  SPAR_NONE,
  SSTOP_1,
  DATA_8
};

const lan_setting_t lan_default = {
  {192,168,0,240,0,0},
  {255,255,255,0,0,0},
  {192,168,0,1,0,0},
  {0x70,0xb1,0xff,0xff,0xff,0xff}
};
const wireless_param_t wlan_default = {
  BOOT_AP,
  "SensorNode",
  "53290921",
  "Grididea.com.tw",
  "53290921",
  60,
  2
};


const module_setting_t module_default = {
  EEP_HEADING,
  "VSS-II",
  0x20200220, 
  0x00000001,
  "Grididea-F4",
  "USER"
};

const sd_config_t sd_default = {
  0,
  //{"VSS_DATA-","VSS_DATA_HIR-","VSS_LOG-"},
  //{10*1024*1024,10*1024*1024,10*1024*1024},
};




#define NOF_RECORD      4
static allocation_t param_alloc[]={
  {"MODULE",EEP_BOOT_OFFSET,sizeof(module_setting_t),&moduleParam.param},
  {"SERIAL",EEP_STORE_OFFSET,sizeof(serial_setting_t),&moduleParam.serial},
  {"LAN",EEP_STORE_OFFSET+32,sizeof(lan_setting_t),&moduleParam.lan},
  {"WLAN",EEP_STORE_OFFSET+64,sizeof(wireless_param_t),&moduleParam.wlan},
};

void defaultParams(void)
{
  memcpy((uint8_t*)&moduleParam.param,(uint8_t*)&module_default,sizeof(module_setting_t));
  memcpy((uint8_t*)&moduleParam.serial,(uint8_t*)&serial_default,sizeof(serial_setting_t));
  memcpy((uint8_t*)&moduleParam.lan,(uint8_t*)&lan_default,sizeof(lan_setting_t));
  memcpy((uint8_t*)&moduleParam.wlan,(uint8_t*)&wlan_default,sizeof(wireless_param_t)); 
   
}

void sysSaveParams(uint8_t id)
{
  if(id == 0xff){
    for(uint8_t i=0; i< NOF_RECORD;i++){
      sys_write_param(param_alloc[i].id,param_alloc[i].d,param_alloc[i].sz);
    }
  }else if(id < NOF_RECORD){
    sys_write_param(param_alloc[id].id,param_alloc[id].d,param_alloc[id].sz);
  }
}


uint8_t sysParamInit()
{
  for(uint8_t i=0; i< NOF_RECORD;i++){
    sys_read_param(param_alloc[i].id,param_alloc[i].d,param_alloc[i].sz);
  }
  
  if(moduleParam.param.flag != EEP_HEADING){
    defaultParams();
    sysSaveParams(0xff);
    return 1;
  }
  return 0;
}

int8_t sys_read_param(uint8_t *name, void *d, uint16_t sz)
{
  int8_t ret = -1; 
  for(uint8_t i=0;i<NOF_RECORD;i++){
    if(strncmp(name,param_alloc[i].id,strlen(name)) == 0){
      eepromRead(param_alloc[i].offset,sz,d);
      break;
    }
  }
  return ret;
}

int8_t sys_write_param(uint8_t *name, void *d, uint16_t sz)
{
  int8_t ret = -1; 
  for(uint8_t i=0;i<NOF_RECORD;i++){
    if(strncmp(name,param_alloc[i].id,strlen(name)) == 0){
      eepromWrite(param_alloc[i].offset,sz,d);
      ret = 0;
      break;
    }
  }
  return ret;
}

uint8_t sys_read_param_id(uint8_t id,uint8_t *d, uint16_t *sz)
{
  if(id < NOF_RECORD){
//    eepromRead(param_alloc[id].offset,param_alloc[id].sz,param_alloc[id].d);
    memcpy(d,param_alloc[id].d,param_alloc[id].sz);
    *sz = param_alloc[id].sz;
    return 1;
  }
  return 0;
}
uint8_t sys_write_param_id(uint8_t id,uint8_t *d, uint16_t sz)
{
  if(id < NOF_RECORD){
    if(id == 0){
      if((d[3] == 0x4A) && d[2] == 0x4A){
        d[3] = 0x45;
        d[2] = 0x50;
        memcpy(param_alloc[id].d,d,sz);  
//        *param_alloc[id].d[3] = 0x45;      
//        *param_alloc[id].d[2] = 0x50;      
      }else{
        memcpy(param_alloc[id].d,d,4);  // copy flag only
      }
    }else{
      memcpy(param_alloc[id].d,d,sz);
    }
    eepromWrite(param_alloc[id].offset,param_alloc[id].sz,param_alloc[id].d);
    return 1;
  }
  return 0;
}
