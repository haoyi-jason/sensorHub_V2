#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include "sensorhub.h"
#include "adxl355_dev.h"
#include "adxl355_config.h"
#include "adxl355_cmd.h"
#include "bmi160.h"
#include "bmi160_config.h"
#include "bmi160_cmd.h"

#include "filesys.h"
#include "sysParam.h"
#include "exti_cfg.h"
#include "htu2x.h"
#include "peripheral_if.h"
#include "ff.h"
#include "time.h"
//#include "comm_if.h"
#include "rsi_app_main.h"
#include "rsi_app_bt_spp_slave.h"

#include "bin_cmd.h"
#include "time_domain.h"

thread_reference_t sensorhub_trp = NULL;
app_param_t appParam;
uint8_t buf_a[SD_BUFFER_SIZE],buf_b[SD_BUFFER_SIZE];
//static ADXL355Driver adxl;

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};


static SPIConfig spicfg_bmi160 = {
  NULL,
  GPIOA,
  8,
  SPI_CR1_BR_1
};

static SPIConfig spicfg_adxl355 = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2
};
static SPIConfig spicfg_rsi = {
  NULL,
  GPIOA,
  GPIOA_SPI1_CS,
  SPI_CR1_BR_1 
};
static const node_param_t default_node = {OP_VNODE,COMM_USE_BT | 0x6};
static const adxl355_config_t default_adxl = {
  ADXL355_FS_2G,
  ADXL355_ODR_250,
  ADXL355_HPF_OFF,
  0x0,
  {0,0,0},
  {0.039,0.039,0.039},
  {0.,0.,0.},
};  

static const imu_config_t default_imu = {
  {BMI160_ACCEL_NORMAL_MODE,BMI160_ACCEL_ODR_100HZ,BMI160_ACCEL_RANGE_2G ,BMI160_ACCEL_BW_NORMAL_AVG4},
  {BMI160_GYRO_NORMAL_MODE,BMI160_GYRO_ODR_100HZ,BMI160_GYRO_RANGE_2000_DPS ,BMI160_GYRO_BW_NORMAL_MODE}
};  

static const time_domain_param_t default_time = {
  1000,1000
};

static const freq_domain_param_t default_freq = {
  0,0,512
};

static const sd_config_t default_sdconfig = {1,"VSS-DATA",1024*1024*10};

#define NOF_USER_CFG    6
#define USER_DATA_ID    6
#define TRH_SENSOR_ID   0x0c
#define BAT_VOLT_ID    0x0d
#define RTC_CFG_ID      0x0e
static const allocation_t param_alloc[]={
  {"NODE",EEP_APP_OFFSET,sizeof(node_param_t),&appParam.node},
  {"ADXL",EEP_APP_OFFSET+64,sizeof(adxl355_config_t),&appParam.adxl},
//  {"BMI160",EEP_APP_OFFSET+128,sizeof(bmi160_config_t),&appParam.bmicfg},
  {"IMU",EEP_APP_OFFSET+128,sizeof(imu_config_t),&appParam.imu},
  {"TIME",EEP_APP_OFFSET+226,sizeof(time_domain_param_t),&appParam.time},
  {"FREQ",EEP_APP_OFFSET+192,sizeof(freq_domain_param_t),&appParam.freq},
  {"SDC",EEP_APP_OFFSET+224,sizeof(sd_config_t),&appParam.sdConfig},
  {"USER",EEP_USER_OFFSET,256},
};
time_domain_t m_timeDomain;

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];

static void adccallback(ADCDriver *adcp, adcsample_t *buffer,size_t n)
{
  (void)adcp;
  uint16_t chSum[3] = {0,0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += samples[i*3];
    chSum[1] += samples[i*3+1];
    chSum[2] += samples[i*3+2];
  }
  chSum[0] >>= 2;
  chSum[1] >>= 2;
  chSum[2] >>= 2;
  // battery_mv for bat1, vbat_mv for bat2
  float ratio = (float)chSum[0]/4096.;
  appParam.battery_mv[0] = ratio*5440; // capacity is Vref...
  
  ratio = (float)chSum[1]/4096.;
  appParam.battery_mv[1] = ratio*5440; // capacity is Vref...
}
void cal_batv(){
  uint16_t chSum[3] = {0,0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += samples[i*3];
    chSum[1] += samples[i*3+1];
    chSum[2] += samples[i*3+2];
  }
  chSum[0] >>= 2;
  chSum[1] >>= 2;
  chSum[2] >>= 2;
  // battery_mv for bat1, vbat_mv for bat2
  float ratio = (float)chSum[0]/4096.;
  appParam.battery_mv[0] = ratio*2720; // capacity is Vref...
  
  ratio = (float)chSum[1]/4096.;
  appParam.battery_mv[1] = ratio*2720; // capacity is Vref...
}
static void adcerror(ADCDriver *adcp, adcerror_t err)
{
  (void)adcp;
  while(1){}
  
}

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  NULL,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)| ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12)
};

int16_t write_bt_stream(uint8_t *d, size_t sz)
{
#ifdef RSI_BT_ENABLE
  rsi_app_bt_send(0,d,sz);
#endif  
  return 0;
}

int16_t read_bt_stream(uint8_t *d, size_t SZ)
{
  int16_t sz = 0;
#ifdef RSI_BT_ENABLE
  sz = rsi_app_bt_read(0,d,SZ);
#endif
  return sz;  
}

int16_t write_wifi_stream(uint8_t *d, size_t sz)
{
#ifdef RSI_WLAN_ENABLE
  rsi_app_wlan_send(appParam.clientSocket,d,sz);
#endif
  return 0;
}

int16_t read_wifi_stream(uint8_t *d, size_t SZ)
{
  int16_t sz;
#ifdef RSI_WLAN_ENABLE
  sz = rsi_app_wlan_read(appParam.clientSocket,d,SZ);
#endif
  return sz;
}
void led_on(int id)
{
  switch(id){
  case 0:
    palClearPad(GPIOC,3);
    break;
  case 1:
    palClearPad(GPIOB,12);
    break;
  case 2:
    palClearPad(GPIOB,13);
    break;
  }
}

void led_off(int id)
{
  switch(id){
  case 0:
    palSetPad(GPIOC,3);
    break;
  case 1:
    palSetPad(GPIOB,12);
    break;
  case 2:
    palSetPad(GPIOB,13);
    break;
  }
}

void led_tog(int id)
{
  switch(id){
  case 0:
    if(palReadPad(GPIOC,3) == PAL_LOW)
      palSetPad(GPIOC,3);
    else
      palClearPad(GPIOC,3);
    break;
  case 1:
    if(palReadPad(GPIOB,12) == PAL_LOW)
      palSetPad(GPIOB,12);
    else
      palClearPad(GPIOB,12);
    break;
  case 2:
    if(palReadPad(GPIOB,13) == PAL_LOW)
      palSetPad(GPIOB,13);
    else
      palClearPad(GPIOB,13);
    break;
  }
}

uint8_t led_state(int id){
  uint8_t ret = 0;
  switch(id){
  case 0:
    ret = (palReadPad(GPIOC,3) == PAL_LOW)?1:0;
    break;
  case 1:
    ret = (palReadPad(GPIOB,12) == PAL_LOW)?1:0;
    break;
  case 2:
    ret = (palReadPad(GPIOB,13) == PAL_LOW)?1:0;
    break;
  }
  return ret;
}

#define PCA9548_BASE_ADDR       0x70
void pca_set_channel(uint8_t ch)
{
  msg_t ret;
  uint8_t ucTx = (1 << ch);
  uint8_t adr = 0;
  ret = i2cMasterTransmitTimeout(&I2CD1,PCA9548_BASE_ADDR + adr,&ucTx,1,NULL,0,MS2ST(10));  
}


uint8_t app_read_param(uint8_t id,uint8_t *d, uint16_t *sz)
{
  if(id & 0x40){
    return sys_read_param_id(id&0xf,d,sz);
  }
  else if(id & 0x10){
    eepromRead(param_alloc[USER_DATA_ID].offset,param_alloc[USER_DATA_ID].sz,d);
    *sz = param_alloc[USER_DATA_ID].sz;
  }
  else{
    switch(id){
    case TRH_SENSOR_ID:{
      trh_sensor_t *cfg = (trh_sensor_t*)d;
      for(uint8_t i=0;i<5;i++){
        cfg[i].temp = appParam.trh[i].temp;
        cfg[i].rh = appParam.trh[i].rh;
      }
      *sz = 40;
    }break;
    case BAT_VOLT_ID:{
      battery_config_t *cfg = (battery_config_t*)d;
      cfg->batv[0] = appParam.battery_mv[0];
      cfg->batv[1] = appParam.battery_mv[1];
      *sz = 4;
    }break;
    case RTC_CFG_ID:{
      rtc_config_t *cfg = (rtc_config_t*)d;
      cfg->yy = appParam.tim_now.tm_year;
      cfg->mm = appParam.tim_now.tm_mon+1;
      cfg->dd = appParam.tim_now.tm_mday;
      cfg->hh = appParam.tim_now.tm_hour;
      cfg->nn = appParam.tim_now.tm_min;
      cfg->ss = appParam.tim_now.tm_sec;
      *sz = 6;
    }break;
    default:
      if(id < NOF_USER_CFG){
        memcpy(d,param_alloc[id].d,param_alloc[id].sz);
        *sz = param_alloc[id].sz;
      }
    }
    return 1;
  }
  return 0;
}

uint8_t app_write_param(uint8_t id,uint8_t *d, uint16_t sz)
{
  i2cStart(&I2CD1,&i2ccfg);
  if(id & 0x40){
    return sys_write_param_id(id & 0xf,d,sz);
  }
  else if(id & 0x10){
    eepromWrite(param_alloc[USER_DATA_ID].offset,param_alloc[USER_DATA_ID].sz,d);
    return 1;
  }
  else  if(id == RTC_CFG_ID){
      rtc_config_t *cfg = (rtc_config_t*)d;
      RTCDateTime timespec;
      struct tm tim;
      tim.tm_year = cfg->yy;
      tim.tm_mon = cfg->mm-1;
      tim.tm_mday = cfg->dd;
      tim.tm_hour = cfg->hh;
      tim.tm_min = cfg->nn;
      tim.tm_sec = cfg->ss;
      rtcConvertStructTmToDateTime(&tim,0,&timespec);
      rtcSetTime(&RTCD1,&timespec);
      return 1;
  }
  else if(id < NOF_USER_CFG){
    memcpy(param_alloc[id].d,d,sz);
    eepromWrite(param_alloc[id].offset,param_alloc[id].sz,param_alloc[id].d);
    return 1;
  }
  i2cStop(&I2CD1);
  return 0;
}


void app_loadDefault()
{
  memcpy((uint8_t*)&appParam.node,(uint8_t*)&default_node,sizeof(node_param_t));
  memcpy((uint8_t*)&appParam.adxl,(uint8_t*)&default_adxl,sizeof(adxl355_config_t));
  memcpy((uint8_t*)&appParam.freq,(uint8_t*)&default_freq,sizeof(freq_domain_param_t));
  memcpy((uint8_t*)&appParam.time,(uint8_t*)&default_time,sizeof(time_domain_param_t));
  memcpy((uint8_t*)&appParam.imu,(uint8_t*)&default_imu,sizeof(imu_config_t));
  memcpy((uint8_t*)&appParam.sdConfig,(uint8_t*)&default_sdconfig,sizeof(sd_config_t));
}
void app_load_param(uint8_t id)
{
  i2cStart(&I2CD1,&i2ccfg);
  if(id == 0xff){
    if(sysParamInit()){
      app_loadDefault();
      for(uint8_t i=0;i<NOF_USER_CFG;i++){
        eepromWrite(param_alloc[i].offset,param_alloc[i].sz,param_alloc[i].d);
      }
    }else{
      for(uint8_t i=0;i<NOF_USER_CFG;i++){
        eepromRead(param_alloc[i].offset,param_alloc[i].sz,param_alloc[i].d);
      }
    }
  }else if(id < NOF_USER_CFG){
      eepromRead(param_alloc[id].offset,param_alloc[id].sz,param_alloc[id].d);
  }
  i2cStop(&I2CD1);
}

uint8_t app_save_param(uint8_t id)
{
  i2cStart(&I2CD1,&i2ccfg);
  if(id == 0x4f){
    eepromWrite(param_alloc[USER_DATA_ID].offset,param_alloc[USER_DATA_ID].sz,param_alloc[USER_DATA_ID].d);
    return 0;
  }
  else if(id & 0x40){
    sysSaveParams(id & 0xf);
    return 0;
  }
  else if(id < NOF_USER_CFG){
    eepromWrite(param_alloc[id].offset,param_alloc[id].sz,param_alloc[id].d);
    return 0;
  }
  i2cStop(&I2CD1);
  return 0;
}

enum{
  BT_INIT,
  BT_GET_MAC,
  BT_SET_NAME,
  BT_SET_BAUD
};


static virtual_timer_t vt;

static void timeout_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(appParam.mainThread,EV_ACT_BATT);
  chVTSetI(&vt,S2ST(1),timeout_cb,NULL);
  chSysUnlockFromISR();
}

static void blinker_cb(void *arg)
{
  chSysLockFromISR();
  if(appParam.ledBlink.ms_on == 0){
    led_off(0);
    chVTSetI(&appParam.blinker,MS2ST(1000),blinker_cb,NULL);
  }else if(appParam.ledBlink.ms_off == 0){
    led_on(0);
    chVTSetI(&appParam.blinker,MS2ST(1000),blinker_cb,NULL);
  }else{
    if(led_state(0)){
      led_off(0);
      chVTSetI(&appParam.blinker,MS2ST(appParam.ledBlink.ms_off),blinker_cb,NULL);
    }else{
      led_on(0);
      chVTSetI(&appParam.blinker,MS2ST(appParam.ledBlink.ms_on),blinker_cb,NULL);
    }
  }
  chSysUnlockFromISR();
}


uint32_t intCntr = 0;


enum{
  REPORT_NONE,
  REPORT_HT,
  REPORT_BATT
};

/**
  working thread
*/

static THD_WORKING_AREA(waBufferHandle,2048);
static THD_FUNCTION(procBufferHandler ,p)
{
  cmd_header_t *header;
  size_t sz;
  uint8_t buf[1024];
  uint16_t bufSz = 0;
  int32_t data[96];
  uint8_t *p_src,*p_dst;
  uint16_t bsz;
  systime_t t_start;
  uint8_t pktCount = 0;

//  resetObject(&m_timeDomain);
  
  bufSz = CMD_STRUCT_SZ;
  bool bStop = false;
  while(!bStop){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
//    DBG_IO_HIGH();
    if(evt & EV_ADXL_FIFO_FULL){
      // read fifo data, sz indicate the number of records, not bytes
      adxl355_get_fifo_size(&adxl355,&sz);
      sz /= 3;
      if(sz){
        switch(appParam.node.opMode){
        case OP_STREAM:
          bsz = sz*9; // read x/y/z combo
          adxl355.buffer = &buf[bufSz];
          adxl355_read_fifo(&adxl355,bsz); // each record has 9-bytes (x/y/z)*3
          bufSz += bsz;
          if(bufSz > 270){
            header = (cmd_header_t*)buf;
            header->magic1 = MAGIC1;
            header->magic2 = MAGIC2;
            header->type = MASK_DATA;
            header->len = bufSz;
            header->pid = pktCount++;
            header->chksum = cmd_checksum(buf,header->len);
            if(appParam.writefcn){
              appParam.writefcn(buf, header->len);
            }
            bufSz = CMD_STRUCT_SZ;        
          }
          break;
        case OP_VNODE:
          bsz = sz*9;
          adxl355.buffer = buf;
          adxl355_read_fifo(&adxl355,bsz); // each record has 9-bytes (x/y/z)*3
          bsz = sz*4;
          p_src = adxl355.buffer;
          p_dst = (uint8_t*)data;
          p_dst += 3;
          for(uint16_t j=0;j<bsz;j++){
            if((j%4)==3){
              *(p_dst--)=0;
              p_dst = (uint8_t*)data + j + 4;
            }else{
              *(p_dst--)=*(p_src++);
            }
          }
          if(feed_fifo32(&m_timeDomain,(uint8_t*)data,sz)==1){
            memcpy((void*)&appParam.rms,(void*)&m_timeDomain.rms,12);
            memcpy((void*)&appParam.crest,(void*)&m_timeDomain.crest,12);
            memcpy((void*)&appParam.velocity,(void*)&m_timeDomain.velocity,12);
            appParam.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
            appParam.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
            appParam.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
            resetObject(&m_timeDomain);
            // send data via WIFI/BT
            // prepare data transmit
            //appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
            header = (cmd_header_t*)buf;
            header->magic1 = MAGIC1;
            header->magic2 = MAGIC2;
            header->type = MASK_DATA;
            header->pid = pktCount++;
            uint8_t *ptr = &buf[CMD_STRUCT_SZ];
            memcpy(ptr,&appParam.peak,12);
            ptr += 12;
            memcpy(ptr,&appParam.rms,12);
            ptr += 12;
            memcpy(ptr,&appParam.crest,12);
            ptr += 12;
            memcpy(ptr,&appParam.velocity,12);
            ptr += 12;
            header->len = 48 + CMD_STRUCT_SZ;
            header->chksum = cmd_checksum(buf,header->len);
            if(appParam.writefcn)
              appParam.writefcn(buf,header->len);
          }
          break;
        case OP_LOGSD:
          bsz = sz*9; // read x/y/z combo
          adxl355.buffer = buf;
          adxl355_read_fifo(&adxl355,bsz); // each record has 9-bytes (x/y/z)*3
          sd_write(buf,bsz);
          break;
        }
      }
    }
    if(evt & EV_BMI_INT1){
      switch(appParam.node.opMode){
      case OP_STREAM:
        bmi160.fifo->data = &buf[CMD_STRUCT_SZ];
        bmi160.fifo->length = 300;
        bmi160_get_fifo_data(&bmi160);
        header = (cmd_header_t*)buf;
        header->magic1 = MAGIC1;
        header->magic2 = MAGIC2;
        header->type = MASK_DATA;
        header->len = bmi160.fifo->length + CMD_STRUCT_SZ;
        header->pid = pktCount++;
        header->chksum = cmd_checksum(buf,header->len);
        if(appParam.writefcn){
          appParam.writefcn(buf, header->len);
        }
        break;
      case OP_VNODE:
        sz = 300/12;
        bmi160.fifo->data = buf;
        bmi160.fifo->length = 300;
        bmi160_get_fifo_data(&bmi160);
          if(feed_fifo16_imu(&m_timeDomain,(uint8_t*)buf,sz)==1){
            memcpy((void*)&appParam.rms,(void*)&m_timeDomain.rms,12);
            memcpy((void*)&appParam.crest,(void*)&m_timeDomain.crest,12);
            memcpy((void*)&appParam.velocity,(void*)&m_timeDomain.velocity,12);
            appParam.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
            appParam.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
            appParam.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
            resetObject(&m_timeDomain);
            // send data via WIFI/BT
            // prepare data transmit
            //appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
            header = (cmd_header_t*)buf;
            header->magic1 = MAGIC1;
            header->magic2 = MAGIC2;
            header->type = MASK_DATA;
            header->pid = pktCount++;
            uint8_t *ptr = &buf[CMD_STRUCT_SZ];
            memcpy(ptr,&appParam.peak,12);
            ptr += 12;
            memcpy(ptr,&appParam.rms,12);
            ptr += 12;
            memcpy(ptr,&appParam.crest,12);
            ptr += 12;
            memcpy(ptr,&appParam.velocity,12);
            ptr += 12;
            header->len = 48 + CMD_STRUCT_SZ;
            header->chksum = cmd_checksum(buf,header->len);
            if(appParam.writefcn)
              appParam.writefcn(buf,header->len);
          }
        break;
      case OP_LOGSD:
        bmi160.fifo->data = buf;
        bmi160.fifo->length = 900;
        bmi160_get_fifo_data(&bmi160);
        sd_write(buf,bmi160.fifo->length);
        break;
      }
    }
    
    bStop = chThdShouldTerminateX();
  }
  chThdExit((msg_t)0);
}

static void startTransfer(void)
{

  if(appParam.node.opMode == OP_LOGSD){
    if(sd_buffer_init(appParam.sdConfig.prefix,appParam.sdConfig.szConstrain)<0){
      return;
    }
  }
  float rate=4000;
  appParam.bufferThread = chThdCreateStatic(waBufferHandle,sizeof(waBufferHandle),NORMALPRIO,procBufferHandler,NULL);
  if(appParam.node.activeSensor & ADXL_ENABLED){
    spiStart(&SPID3,&spicfg_adxl355);
    switch(appParam.adxl.fullscale){
    case ADXL355_FS_2G:
      adxl355.config->sensitivity[0] = 0.0000039;
      adxl355.config->sensitivity[1] = 0.0000039;
      adxl355.config->sensitivity[2] = 0.0000039;
      break;
    case ADXL355_FS_4G:
      adxl355.config->sensitivity[0] = 0.0000078;
      adxl355.config->sensitivity[1] = 0.0000078;
      adxl355.config->sensitivity[2] = 0.0000078;
      break;
    case ADXL355_FS_8G:
      adxl355.config->sensitivity[0] = 0.0000156;
      adxl355.config->sensitivity[1] = 0.0000156;
      adxl355.config->sensitivity[2] = 0.0000156;
      break;   
    }
    m_timeDomain.scale.x = adxl355.config->sensitivity[0];
    m_timeDomain.scale.y = adxl355.config->sensitivity[1];
    m_timeDomain.scale.z = adxl355.config->sensitivity[2];

    rate = 4000/(1 << adxl355.config->outputrate);
    m_timeDomain.nofSamples = (uint16_t)(rate * appParam.time.samplePeriodMs/1000);
    m_timeDomain.sampleTimesec = appParam.time.samplePeriodMs/1000.;
    adxl355_powerdown(&adxl355);
    adxl355.config->intmask = ADXL355_INT_FULL_EN1;
    adxl355_set_filter(&adxl355);
    adxl355_set_full_scale(&adxl355);
    adxl355_set_interrupt(&adxl355);
    extChannelEnable(&EXTD1,9);
    adxl355_powerup(&adxl355);
  }
  else if(appParam.node.activeSensor & BMI160_ENABLED){
    float gScale;
    if(appParam.imu.accel.range == 0x03){
      gScale = 4.0/65536.;
    }
    else if(appParam.imu.accel.range == 0x05){
      gScale = 8.0/65536.;
    }
    else if(appParam.imu.accel.range == 0x08){
      gScale = 16.0/65536.;
    }
    else{
      gScale = 32./65536.;
    }
    m_timeDomain.scale.x = gScale;
    m_timeDomain.scale.y = gScale;
    m_timeDomain.scale.z = gScale;
    spiStart(&SPID3,&spicfg_bmi160);
    bmi160.read = bmi160_read;
    bmi160.write = bmi160_write;
    bmi160.delay_ms = bmi160_delay;
    if(bmi160_cmd_init(&bmi160) == BMI160_OK){
      memcpy((void*)&bmi160.accel_cfg,(void*)&appParam.imu.accel,sizeof(struct bmi160_cfg));;
      memcpy((void*)&bmi160.gyro_cfg,(void*)&appParam.imu.gyro,sizeof(struct bmi160_cfg));;
      bmi160_reconfig_by_app(&bmi160);
      bmi160_cmd_testRead(&bmi160);
      bmi160_cmd_config_int(&bmi160);
      extChannelEnable(&EXTD1,11);
    }

//    spiStart(&SPID3,&spicfg_bmi160);
//    bmi160_reconfig_by_app(&bmi160);
//    bmi160_cmd_testRead(&bmi160);
//    bmi160_cmd_config_int(&bmi160);
//    extChannelEnable(&EXTD1,11);
  }
  appParam.ledBlink.ms_on = 250;
  appParam.ledBlink.ms_off = 250;
}

static void stopTransfer(void)
{
  if(appParam.bufferThread){
    chThdTerminate(appParam.bufferThread);
    chThdWait(appParam.bufferThread);
    appParam.bufferThread = NULL;
    if(appParam.node.activeSensor & ADXL_ENABLED ){
      extChannelDisable(&EXTD1,9);
      adxl355.config->intmask = 0;
      adxl355_set_interrupt(&adxl355);
      adxl355_powerdown(&adxl355);
      spiStop(&SPID3);
    }
    if(appParam.node.activeSensor & BMI160_ENABLED ){
      extChannelDisable(&EXTD1,11);
      bmi160_cmd_deconfig_int(&bmi160);
    }
    spiStop(&SPID3);
  }
  if(appParam.node.opMode == OP_LOGSD){
    sd_buffer_deinit();
  }
  appParam.ledBlink.ms_on = 1000;
  appParam.ledBlink.ms_off = 1000;
}


static THD_WORKING_AREA(waSensorHub,2048);
static THD_FUNCTION(procSensorHub ,p)
{
  cmd_header_t *header;
  uint8_t msg[64];
  uint16_t odr;
  
//  appParam.node.activeSensor = ADXL_ENABLED;
//  appParam.node.activeSensor = BMI160_ENABLED;
  if((appParam.node.activeSensor & 0x3) == 0x00){
    appParam.node.activeSensor = ADXL_ENABLED;
  }
  if(appParam.node.activeSensor & ADXL_ENABLED){
    spiStart(&SPID3,&spicfg_adxl355);
    if(adxl355_init(&adxl355) == ADXL355_OK){
      adxl355_powerup(&adxl355);
      adxl355_powerdown(&adxl355);
    }    
  }

  if(appParam.node.activeSensor & BMI160_ENABLED){
    spiStart(&SPID3,&spicfg_bmi160);
    bmi160.read = bmi160_read;
    bmi160.write = bmi160_write;
    bmi160.delay_ms = bmi160_delay;
    if(bmi160_cmd_init(&bmi160) == BMI160_OK){
      memcpy((void*)&bmi160.accel_cfg,(void*)&appParam.imu.accel,sizeof(struct bmi160_cfg));;
      memcpy((void*)&bmi160.gyro_cfg,(void*)&appParam.imu.gyro,sizeof(struct bmi160_cfg));;
      bmi160_reconfig_by_app(&bmi160);
      bmi160_cmd_testRead(&bmi160);
    }
  }
  chThdResume(&sensorhub_trp,MSG_OK);
  chVTObjectInit(&vt);
  chVTSet(&vt,S2ST(1),timeout_cb,NULL);
  chVTObjectInit(&appParam.blinker);
  //appParam.ledBlinkPeriod = 0xff;
  chVTSet(&appParam.blinker,MS2ST(1000),blinker_cb,NULL);
   extChannelEnable(&EXTD1,13);

  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);
  
  appParam.ledBlink.ms_off = 0;
  appParam.ledBlink.ms_on = 1000;
  if(appParam.node.commType & COMM_USE_BT){
    appParam.writefcn = write_bt_stream;
    appParam.readfcn = read_bt_stream;
  }
  else if(appParam.node.commType & COMM_USE_WIFI){
    appParam.writefcn = write_wifi_stream;
    appParam.readfcn = read_wifi_stream;
  }
  

  while(true){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_CMD_EXTINT){
      
    }
    if(evt & EV_ACT_BATT){
      RTCDateTime timespec;
      rtcGetTime(&RTCD1,&timespec);
      rtcConvertDateTimeToStructTm(&timespec,&appParam.tim_now,NULL);
      appParam.runSecs++;
      cal_batv();
      if(!appParam.bufferThread){
        i2cStart(&I2CD1,&i2ccfg);
        pca_set_channel(2);
        i2cStop(&I2CD1);
        htu2xinit(&I2CD1,(I2CConfig*)&i2ccfg);
        appParam.trh[0].temp = sen_htu2xx_read_temp();
        appParam.trh[0].rh = sen_htu2xx_read_humidity();
        //appParam.battery_mv = ads1015_read_data(&ads1015)*2;
      }

      //if(appParam.dataPath){
#if 0
        if(moduleParam.hub.ht_report_interval){
          if((appParam.runSecs % moduleParam.hub.ht_report_interval) == 0){
            report(REPORT_HT);
          }
        }
        if(moduleParam.battery.reportInterval){
          if((appParam.runSecs % moduleParam.battery.reportInterval) == 0){
            report(REPORT_BATT);
          }
        }
#endif
      //}
      if(palReadPad(GPIOC,13) == PAL_LOW){
        appParam.user_pressed_sec += appParam.user_press_inc;
        if(appParam.user_pressed_sec > 3){
          appParam.user_press_inc = 0;
          appParam.user_pressed_sec = 0;
          if(appParam.bufferThread)
            continue;     
          else{
            appParam.user_cfg_mode = (appParam.user_cfg_mode==0)?1:0;
//            if(appParam.user_cfg_mode == 0){ // leave wifi/bt config mode
//              sysSaveParams(); 
//            }
            led_off(0);
            appParam.ledBlink.ms_on = 250;
            appParam.ledBlink.ms_off = 250;
            chThdSleepMilliseconds(1700);
            appParam.ledBlink.ms_on = 0;
            appParam.ledBlink.ms_off = 0xffff;
            led_off(0);
          }
        }
      }else{
        appParam.user_press_inc = 0;
        appParam.user_pressed_sec = 0;
      }
    }
    
    if(evt & EV_CMD_READ_BATT){
      
      header = (cmd_header_t*)appParam.txBuf.buffer;
      header->magic1 = MAGIC1;
      header->magic2 = MAGIC2;
      header->type = MASK_DATA | DATA_BATTERY;
      header->len = CMD_STRUCT_SZ+4;
      header->pid = 0;
      appParam.txBuf.buffer[CMD_STRUCT_SZ+1] = appParam.battery_mv[0] >> 8;
      appParam.txBuf.buffer[CMD_STRUCT_SZ] = appParam.battery_mv[0] & 0xff;
      appParam.txBuf.buffer[CMD_STRUCT_SZ+3] = appParam.battery_mv[1] >> 8;
      appParam.txBuf.buffer[CMD_STRUCT_SZ+2] = appParam.battery_mv[1] & 0xff;
      header->chksum = cmd_checksum(appParam.txBuf.buffer,header->len);
      appParam.txBuf.sz = header->len;
//      if((moduleParam.hub.commType & COMM_USE_WIFI) == COMM_USE_WIFI){
//        rsi_app_wlan_send(appParam.clientSocket, appParam.txBuf.buffer, appParam.txBuf.sz);
//      }
//      else if((moduleParam.hub.commType & COMM_USE_BT) == COMM_USE_BT){
//        rsi_app_bt_sendcmd(0);
//      }
    }
    
    if(evt & EV_CMD_READ_ENV){
      uint8_t *ptr = appParam.txBuf.buffer;
      header = (cmd_header_t*)ptr;
      header->magic1 = MAGIC1;
      header->magic2 = MAGIC2;
      header->type = MASK_DATA | DATA_ENV;
      header->len = 16 + CMD_STRUCT_SZ;
      header->pid = 0;
      // send packet direct
      ptr += 8;
      memcpy(ptr,(uint8_t*)&appParam.temp,8);
      ptr +=8;
      memcpy(ptr,(uint8_t*)&appParam.humidity,8);
      header->chksum = cmd_checksum(msg,header->len);
      appParam.txBuf.sz = header->len;
//      if((moduleParam.hub.commType & COMM_USE_WIFI) == COMM_USE_WIFI){
//        rsi_app_wlan_send(appParam.clientSocket, appParam.txBuf.buffer, header->len);
//      }
//      else if((moduleParam.hub.commType & COMM_USE_BT) == COMM_USE_BT){
//        rsi_app_bt_sendcmd(0);
//      }
    }

//    if(evt & EV_CMD_SINGLE){
//      if(appParam.workingThread) return;
//      switch(appParam.singleRead){
//      case SENSOR_AD7124:
//        break;
//      case SENSOR_ADXL355:
//        break;
//      case SENSOR_BMI160:
//        bmi160_cmd_acquire_one(&bmi160,msg);
//        //sdWrite(&SD2,msg,12);
//        break;
//      case SENSOR_HTU21D:
//        break;
//      }
//    }
    if(evt & EV_USER_BUTTON_0){
      chThdSleepMilliseconds(500);
      if(palReadPad(GPIOC,13) == PAL_HIGH){
        if(appParam.user_cfg_mode){
          uint8_t n;
//          if((moduleParam.hub.commType & COMM_USE_BT) == COMM_USE_BT){
//            n = 2;
//            moduleParam.hub.commType = (moduleParam.hub.commType & 0x0f) | COMM_USE_WIFI;
//          }
//          else if((moduleParam.hub.commType & COMM_USE_WIFI) == COMM_USE_WIFI){
//            n = 1;
//            moduleParam.hub.commType = (moduleParam.hub.commType & 0x0f) | COMM_USE_BT;
//          }
          led_off(0);
          chThdSleepMilliseconds(200);
          appParam.ledBlink.ms_on = 500;
          appParam.ledBlink.ms_off = 500;
          chThdSleepMilliseconds(n*1100);
          appParam.ledBlink.ms_on = 0;
          appParam.ledBlink.ms_off = 0xffff;
          led_off(0);

          
        }
        else{
          if(appParam.bufferThread)
            evt |= EV_CMD_STOP;
          else{
            evt |= EV_CMD_RUN;
            appParam.dataPath = DATA_PATH_LOGSD;
            appParam.node.opMode = OP_LOGSD;
          }
        }
      }
      else{
        appParam.user_press_inc = 1;
      }
    }
    // run command received
    if(evt & EV_CMD_RUN){
      if(appParam.bufferThread)
        continue;
        startTransfer();
//        appParam.ledBlink.ms_on = 500;
//        appParam.ledBlink.ms_off = 500;
    }
    
    if(evt & EV_CMD_STOP){
      stopTransfer();
      appParam.ledBlink.ms_on = 0;
      appParam.ledBlink.ms_off = 0xffff;
      if(appParam.dataPath == DATA_PATH_TRANMIT){
        // send response packet
        cmd_header_t *h_resp = (cmd_header_t*)appParam.txBuf.buffer;
        h_resp->magic1 = MAGIC1;
        h_resp->magic2 = MAGIC2;
        h_resp->type = MASK_CMD_RET_OK;
        h_resp->pid = 0;
        h_resp->len = CMD_STRUCT_SZ;    
        h_resp->chksum = cmd_checksum(appParam.txBuf.buffer,h_resp->len);
        appParam.txBuf.sz = h_resp->len;
//        if((moduleParam.hub.commType & COMM_USE_WIFI) == COMM_USE_WIFI){
//          rsi_app_wlan_send(appParam.clientSocket, appParam.txBuf.buffer, appParam.txBuf.sz);
//        }
//        else if((moduleParam.hub.commType & COMM_USE_BT) == COMM_USE_BT){
//          rsi_app_bt_sendcmd(0);
//        }
      }
    }
    if(evt & EV_P7_EXINT){
    }
    
    if(evt & EV_SAVE_PARAM){
//      sysSaveParams();
      //sensorHubInit();
    }
    if(evt & EV_LOAD_PARAM){
      app_loadDefault();
      defaultParams();
//      sysSaveParams();
      //sensorHubInit();
    }
    if(evt & EV_BMI160_CMD_FOC){
      //bmi160_cmd_startfoc(&bmi160);
    }
    if(evt & EV_CMD_RESET){
      chSysDisable();
      NVIC_SystemReset();
    }

    if(evt & EV_SDMMC_INSERT){
      sdCheck();
      if(!appParam.sdConfig.capacity){
        appParam.ledBlink.ms_on = 100;
        appParam.ledBlink.ms_off = 1000;
      }
      extChannelEnable(&EXTD1,10);  
    }
    
    if(evt & EV_CLIENT_CONNECT){
      appParam.connState = 0x1;
      appParam.ledBlink.ms_on = 1000;
      appParam.ledBlink.ms_off = 1000;
    }
    
    if(evt & EV_CLIENT_DISCONNECT){
//      if(appParam.dataPath == DATA_PATH_TRANMIT){
        appParam.connState = 0x0;
        // stop working thread if any
        if(appParam.bufferThread && appParam.node.opMode != OP_LOGSD){
          stopTransfer();
        }
      appParam.ledBlink.ms_on = 1000;
      appParam.ledBlink.ms_off = 0;
//      }
    }
   
    if(evt & EV_CMD_RX){
      uint8_t cmd;
      size_t pktSz;
      appParam.rxBuf.sz = appParam.readfcn(appParam.rxBuf.buffer,256);
      cmd = commif_parse(appParam.rxBuf.buffer,appParam.txBuf.buffer,&pktSz);
      switch(cmd){
      case COMM_IF_CMD_START:
        if(appParam.bufferThread == NULL){
          if(pktSz && appParam.writefcn){
            appParam.writefcn(appParam.txBuf.buffer,pktSz);
          }
          startTransfer();
        }
        break;
      case COMM_IF_CMD_STOP:
        stopTransfer();
        cmd_header_t *h_resp = (cmd_header_t*)appParam.txBuf.buffer;
        h_resp->magic1 = MAGIC1;
        h_resp->magic2 = MAGIC2;
        h_resp->type = MASK_CMD_RET_OK;
        h_resp->pid = 0;
        h_resp->len = CMD_STRUCT_SZ;    
        h_resp->chksum = cmd_checksum(appParam.txBuf.buffer,h_resp->len);
        appParam.txBuf.sz = h_resp->len;
        if(appParam.writefcn){
          appParam.writefcn(appParam.txBuf.buffer,h_resp->len);
        }
        break;
      case COMM_IF_CMD_SINGLE:
        
        
        break;
      case COMM_IF_CMD_RESET:
        chSysDisable();
        NVIC_SystemReset();
        break;
      default:
        if(pktSz && appParam.writefcn){
          appParam.txBuf.sz = pktSz;
          if(appParam.bufferThread == NULL)
            appParam.writefcn(appParam.txBuf.buffer,pktSz);
        }
      }
    }
    if(evt & EV_SYS_SAVE_PAIRED){
      app_save_param(0x43);
    }
  }
  
}


int main(void)
{
  halInit();
  chSysInit();
  
  //i2cStart(&I2CD1,&i2ccfg);
  //chThdSleepMilliseconds(50);
  //i2cStart(&I2CD2,&i2ccfg);
  
 
  at24eep_init(&I2CD1,32,1024,0x50,2);
  app_load_param(0xff);
  
//  pca_set_channel(2);
//  htu2xinit(&I2CD1);
//  appParam.temp = sen_htu2xx_read_temp();
//  appParam.humidity = sen_htu2xx_read_humidity();
  
  //moduleParam.hub.commType = COMM_USE_WIFI;

  if(palReadPad(GPIOC,13) == PAL_LOW){
    uint8_t cntr = 0;
    for(cntr=0;cntr<100;cntr++){
      if(palReadPad(GPIOC,13) == PAL_HIGH){
        continue;
      }      
      chThdSleepMilliseconds(100);
    }
    if(cntr > 30){ // press > 3 sec
      defaultParams();
      //sysSaveParams();
      led_off(0);
      appParam.ledBlink.ms_on = 250;
      appParam.ledBlink.ms_off = 250;
      chThdSleepMilliseconds(1500);
      appParam.ledBlink.ms_on = 0;
      appParam.ledBlink.ms_off = 0xff;
      led_off(0);
      NVIC_SystemReset();
    }
  }
  
  
 
  moduleParam.param.verNum = SW_VERSION_NUMBER;
  
//  if(moduleParam.hub.passwdAP[0] == 0x0)
//    sprintf(moduleParam.hub.passwdAP,"53290921\0");
//  moduleParam.hub.ssidPrefix[15] = 0x0;
  extStart(&EXTD1,&extcfg);

  extChannelEnable(&EXTD1,2);  
  extChannelEnable(&EXTD1,10);  
  sdcStart(&SDCD1,NULL);
  sdCheck();
  //sdStop(&SD2);

  //sensorHubIfInit();
//  palSetPadMode(GPIOA,2,PAL_MODE_INPUT); // INT
//  palSetPadMode(GPIOA,3,PAL_MODE_OUTPUT_OPENDRAIN); // RST
//  palSetPadMode(GPIOA,0,PAL_MODE_INPUT); // LP_WAKEUP
//  palSetPadMode(GPIOA,1,PAL_MODE_INPUT); // BL_HOST
  //palClearPad(GPIOA,0);
  //palClearPad(GPIOA,1);
  spiStart(&SPID1,&spicfg_rsi);
  //rsi_spi_set_interface(&SPID1);
  rsi_app_init();

  //chThdSleepMilliseconds(100);
  //sensorHubInit();
  //appParam.stream = NULL;
//  appParam.activeFile.dirChanged = 0;
//  appParam.activeFile.path[0]='/';
//  appParam.activeFile.path[1] = 0x0;
//  appParam.dataPath = 0;
//  appParam.activeFile.dirChanged = 1;

  appParam.user_pressed_sec = 0;
  appParam.user_press_inc = 0;
  appParam.user_cfg_mode = 0;
  
  
  chMtxObjectInit(&appParam.mtx);
  chMtxObjectInit(&appParam.mtx_fs);
 
  appParam.mainThread = chThdCreateStatic(waSensorHub,sizeof(waSensorHub),NORMALPRIO,procSensorHub,NULL);

  //appParam.workingThread = chThdCreateStatic(waBufferHandle,sizeof(waBufferHandle),NORMALPRIO-1,procBufferHandler,NULL);
  
  // lock until thread finishing initial task
  chSysLock();
  chThdSuspendS(&sensorhub_trp);
  chSysUnlock();
  
  //commif_init(appParam.mainThread);
  while(1){
    chThdSleepMilliseconds(100);
  }
   
  
}

void adxl_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.bufferThread)
    chEvtSignalI(appParam.bufferThread, EV_ADXL_FIFO_FULL);
  chSysUnlockFromISR();
}

void bmi160_int1_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.bufferThread)
    chEvtSignalI(appParam.bufferThread,EV_BMI_INT1);
  chSysUnlockFromISR();  
}
void bmi160_int2_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.bufferThread)
    chEvtSignalI(appParam.bufferThread,EV_BMI_INT1);
  chSysUnlockFromISR();  
}


void adc_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  extChannelDisableI(&EXTD1,14);
  //LED_ON();
  if(appParam.bufferThread)
    chEvtSignalI(appParam.bufferThread,EV_P7_EXINT);
  chSysUnlockFromISR();
}

void adcEnableInterupt(void *p)
{
    extChannelEnable(&EXTD1,14);  
}




int8_t adxl355_conversion_done(struct ADXL355_dev *p)
{
  
  return 0;
}
int8_t adxl355_buffer_cb(struct ADXL355_dev *p)
{
 
  return 0;
}

void user_button_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  chEvtSignalI(appParam.mainThread,EV_USER_BUTTON_0);
  chSysUnlockFromISR();  
}
void sdmmc_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  extChannelDisableI(&EXTD1,10);
  chEvtSignalI(appParam.mainThread,EV_SDMMC_INSERT);
  chSysUnlockFromISR();
}

void gpioa2_int_handler(EXTDriver *extp, expchannel_t channel)
{
#ifdef APP_USE_RSI
  if(appParam.rsi_int)
    appParam.rsi_int();
#endif
#ifdef APP_USE_LORA
  if(appParam.lora_int)
    appParam.lora_int(NULL);
#endif
}


static int8_t ads_read(uint8_t sla, uint8_t *rx,uint16_t len)
{
  msg_t ret = MSG_OK;
  i2cAcquireBus(&I2CD1);
//  ret = i2cMasterTransmitTimeout(&I2CD1,
//                         sla,
//                         &regAddr,
//                         1,
//                         b,
//                         n,
//                         MS2ST(10));
  ret = i2cMasterReceiveTimeout(&I2CD1,sla,rx,len,MS2ST(50));
  i2cReleaseBus(&I2CD1);
  return 0;
}
static int8_t ads_write(uint8_t sla, uint8_t *tx,uint16_t len)
{
  msg_t ret = MSG_OK;
  i2cAcquireBus(&I2CD1);
  ret = i2cMasterTransmitTimeout(&I2CD1,
                         sla,
                         tx,
                         len,
                         0,
                         0,
                         MS2ST(10));
  i2cReleaseBus(&I2CD1);
  return 0;
}
