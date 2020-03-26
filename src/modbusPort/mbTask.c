#include "ch.h"
#include "hal.h"
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "mbTask.h"
#include "port.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "mbtcp.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "sysParam.h"
#include "userSlaveRegMap.h"
#include "modbus_gateway.h"

#define MB_TCP_UID      6
#define MB_TCP_LEN      4
#define MB_TCP_FUNC      7

#define MB_TCP_DEFAULT_PORT     502
#define MB_TCP_BUF_SIZE (256+7)

thread_t *thdModbus;
thread_t *thdMasterModbus;
thread_reference_t modbus_trp = NULL;

#define MB_RTU_EN   1
#define MB_TCP_EN   1
#define RANGE_VALID(x,y,z) ((x<=y) && (y<=z))

static THD_WORKING_AREA(T0, 512);
#define MB_SIZE 5
static MAILBOX_DECL(mb1,T0,MB_SIZE);

int16_t doorCtrlMap[7][17];
uint8_t doorCtrlRegAltered[7] = {0,0,0,0,0,0,0};
uint8_t levelDispPacket[7] = {0xfa,0x0,0x0,0x0,0x0,0x0,0xfb};
// decoder function declaration
int8_t levelDisplay(uint8_t,uint8_t*);
int8_t doorControlR(uint8_t,uint8_t*);
int8_t doorControlW(uint8_t,uint8_t*);


#define NOF_DECODER     3
modbus_decoder_t decoder[] = {
  {0,5,levelDisplay,levelDisplay},
  {101,117,doorControlR,doorControlW},
  {201,217,doorControlR,doorControlW}
};

uint16_t byteSwap(uint16_t v)
{
  uint16_t vret;
  vret = ((v&0xff)<<8) | (v >> 8);

  return vret;
}

static void mapMBWord(uint8_t *dptr, uint8_t *val)
{
  *dptr = *(val+1);
  *(dptr+1) = *val;
}

int8_t levelDisplay(uint8_t usAddr,uint8_t *dptr)
{
  uint8_t adr = usAddr+1;
  uint16_t uval;
  mapMBWord((uint8_t*)&uval,dptr);
  if((adr >0) && (adr < 6)){
    levelDispPacket[adr] = (uint8_t)uval;
    if(adr == 5)
      chEvtSignalI(thdGateway,EV_MBGW_LDISP_CMDRCVD);
  }
  return 1;
}

void levelDispGetPacket(uint8_t *p)
{
  p = levelDispPacket;
}

int8_t doorControlR(uint8_t usAddr,uint8_t *dptr)
{
  uint8_t id = usAddr / 100 - 1; // should be between 0 ~ 6
  uint8_t index = usAddr - id*100;
  uint16_t val = doorCtrlMap[id][index];
  
  mapMBWord(dptr,(uint8_t*)&val);
  
  return 1;
}

int8_t doorControlW(uint8_t usAddr,uint8_t *dptr)
{
  uint8_t id = usAddr / 100 - 1; // should be between 0 ~ 6
  uint8_t index = usAddr - id*100;
  uint16_t val = doorCtrlMap[id][index];
  
  mapMBWord((uint8_t*)&val,dptr);
  doorCtrlMap[id][index] = val;
  doorCtrlRegAltered[id] = 1;
  
  if(usAddr == 9){ // last register address write
    
  }
  return 1;
}


eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer,USHORT usAddress,USHORT usNRegs)
{
  return MB_ENOREG;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,USHORT usNRegs, eMBRegisterMode eMode)
{
  eMBErrorCode eStatus = MB_ENOERR;
  uint8_t i;
  uint16_t regVal;
  USHORT usAddr = usAddress - 1;
  uint8_t j;
  uint8_t index;
  uint8_t szRet;
  
  double dv;
  if(eMode == MB_REG_READ){
    for(i=0;i<usNRegs;i++){
      for(j=0;j<NOF_DECODER;j++){
        if(RANGE_VALID(decoder[i].loBound,usAddr,decoder[i].hiBound)){
          szRet = decoder[i].reader(usAddr,pucRegBuffer);
          usAddr += szRet;
          pucRegBuffer += (szRet<<1);
        }
      }
    }
  }else if(eMode == MB_REG_WRITE){
    for(i=0;i<usNRegs;i++){
      for(j=0;j<NOF_DECODER;j++){
        if(RANGE_VALID(decoder[i].loBound,usAddr,decoder[i].hiBound)){
          szRet = decoder[i].writer(usAddr,pucRegBuffer);
          usAddr += szRet;
          pucRegBuffer += (szRet<<1);
        }
      }
    }
  }
  
  return eStatus;
}
eMBErrorCode eMBMasterRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    mb_struct_t *mb = xMBGetActiveMb();
    uint8_t slaIndex = mb->address - 1;
    uint8_t i;
    uint8_t *dptr = (uint8_t*)pucRegBuffer;
    int16_t val;
    uint16_t addr = usAddress;
    switch(eMode){
    case MB_REG_READ: // read registers from slave
      for(i=0;i<usNRegs;i++){
        mapMBWord((uint8_t*)&val,(uint8_t*)dptr);
        doorCtrlMap[slaIndex][addr] = val;
        dptr += 2;
        addr++;
      }
      break;
    case MB_REG_WRITE: // write registers to slave
      for(i=0;i<usNRegs;i++){
        val = doorCtrlMap[slaIndex][addr];
        mapMBWord((uint8_t*)dptr,(uint8_t*)&val);
        dptr += 2;
        addr++;
      }
      break;
    }

    return eStatus;
}


static  mbMaster_request_t mbReq;
extern struct netconn *serverconn;
static THD_WORKING_AREA(waMBTask,2048);
static THD_FUNCTION(thMBTask ,p)
{
  mbRTUOpts_t *opt = (mbRTUOpts_t*)p;
  mb_struct_t *mbRTU;
  chRegSetThreadName("MODBUS_Thread");
  mb_struct_t *curMb;
  
  
  eMBInit(MB_SLAVE,MB_ASCII,1,2,9600,MB_PAR_NONE);
  
//  if(eMBTCPInit(MB_SLAVE,502) == MB_ENOERR){
//    
//  }
  
  /* Resumes the caller and goes to the final priority.*/
  chThdResume(&modbus_trp, MSG_OK);

  curMb = mbRoot;
  while(curMb){
    eMBEnable(curMb);
    curMb = curMb->next;
  }
  uint8_t cntr = 0;
  uint8_t regcnt = 1;
  msg_t ret;
  uint8_t sla = 1;
   mbReq.funcCode = 0x3;
   mbReq.nofReg = 1;
   mbReq.port = 1;
   mbReq.slaveAddr = sla;
   mbReq.startAddr = 0x2001;
  while(true){
    curMb = mbRoot;
    while(curMb){
      eMBPoll(curMb);
      curMb = curMb->next;
    }
    
  if(cntr++ > 200){
   if(regcnt == 10){
     regcnt = 1;
     sla = (sla==1)?1:1;
   }
    ret = chMBPost(&mb1,(msg_t)&mbReq,TIME_INFINITE);
    cntr = 0;
  }
    chThdSleepMilliseconds(5);
  }
  
}

static THD_WORKING_AREA(waMBMasterTask,2048);
static THD_FUNCTION(thMBMasterTask ,p)
{
  mbRTUOpts_t *opt = (mbRTUOpts_t*)p;
  mb_struct_t *mbRTU;
  mb_struct_t *curMb;
  chRegSetThreadName("MODBUSMaster_Thread");

  eMBInit(MB_MASTER,MB_ASCII,1,3,19200,MB_PAR_NONE);
  // eMBTCPInit(MB_MASTER,12345);
  /* Resumes the caller and goes to the final priority.*/
  chThdResume(&modbus_trp, MSG_OK);
  
  curMb = mbRoot;
  while(curMb){
    if(curMb->mode == MB_TCP){
      curMb->remoteIp[0] = 192;
      curMb->remoteIp[1] = 168;
      curMb->remoteIp[2] = 0;
      curMb->remoteIp[3] = 111;
      curMb->remoteport = 502;
      
    }
    eMBEnable(curMb);
    curMb = curMb->next;
  }
  uint8_t cntr = 0;
  uint8_t regcnt = 1;
  msg_t msg1,msg2;
  bool connected = false;
  while(true){
    curMb = mbRoot;
    if(!connected){
      while(curMb){
        if(curMb->mode == MB_TCP){
          if(curMb->connectionState == TCP_CLIENT_IDLE)
            eMBEnable(curMb);
          if(curMb->connectionState == TCP_CLIENT_CONNECTED)
            connected = true;
        }else{
            eMBEnable(curMb);
            connected = true;
        }
        curMb = curMb->next;
      }
    }else{
      while(curMb){
        if(curMb->role == MB_MASTER){
          eMBPoll(curMb);
          if(curMb->connectionState == TCP_CLIENT_DISCONNECTED){
            connected =false;
            curMb->connectionState = TCP_CLIENT_IDLE;
          }
        }
        curMb = curMb->next;
      }
      // check events
      //thread_t *tp = chMsgWait();
      //msg_t msg = chMsgGet(tp);
      msg_t msg = chMBFetch(&mb1,&msg1,MS2ST(10));
      if(msg != MSG_OK){
        continue;
      }
      mbMaster_request_t *req = (mbMaster_request_t*)msg1;
      curMb = mbRoot;
      while(curMb){
        if(curMb->role == MB_MASTER){
          eMBMasterReqReadHoldingRegister(curMb,req->slaveAddr,req->startAddr,req->nofReg,1000);
          break;
        }
        curMb = curMb->next;
      }
    }

    //chMsgRelease(tp,MSG_OK);
    chThdSleepMilliseconds(5);
  }
  
}

int mbPortInit(mbRTUOpts_t *opts)
{
  // initialize mailbox
  //chMBObjectInit(&mb1,(msg_t*)T0,MB_SIZE);
  
  
  thdModbus = chThdCreateStatic(waMBTask,sizeof(waMBTask),NORMALPRIO,thMBTask,opts);
  // lock until thread finishing initial task
  chSysLock();
  chThdSuspendS(&modbus_trp);
  chSysUnlock();
  
  thdMasterModbus = chThdCreateStatic(waMBMasterTask,sizeof(waMBMasterTask),NORMALPRIO,thMBMasterTask,opts);
  chSysLock();
  chThdSuspendS(&modbus_trp);
  chSysUnlock();

  return 0;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
