#include "ch.h"
#include "hal.h"
#include "simplelink.h"
#include "sl_common.h"

thread_t *tcpThread;


static THD_WORKING_AREA(waTcpServer,512);
static THD_FUNCTION(procTcpServer ,p)
{
  uint16_t *port = (uint16_t*)p;
  
  SlSockAddrIn_t addr;
  SlSockAddrIn_t LocalAddr;
  
  uint16_t idx = 0;
  uint16_t AddrSize = 0;
  uint16_t SockID = 0;
  uint32_t Status = 0;
  uint16_t newSockID = 0;
  uint16_t LoopCount = 0;
  int16_t recvSize = 0;
  
  LocalAddr.sin_family = SL_AF_INET;
  LocalAddr.sin_port = sl_Htons((_u16)*port);
  LocalAddr.sin_addr.s_addr = 0;
  
  SockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM,0);
  
  if(SockID < 0){
    chThdExit(MSG_RESET);
  }
  
  AddrSize = sizeof(SlSockAddrIn_t);
  Status = sl_Bind(SockID,(SlSockAddr_t*)&LocalAddr,AddrSize);
  if(Status < 0){
    sl_Close(SockID);
    chThdExit(MSG_RESET);
  }
  
  Status = sl_Listen(SockID,0);
  if(Status < 0){
    sl_Close(SockID);
    chThdExit(MSG_RESET);
  }
    
  
  
}

thread_t *tcpServer(uint16_t port)
{
  tcpThread = chThdCreateStatic(waTcpServer,sizeof(waTcpServer),NORMALPRIO,procTcpServer,&port);

  return tcpThread;
}
