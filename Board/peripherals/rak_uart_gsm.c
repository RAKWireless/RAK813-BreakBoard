/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "rak_uart_gsm.h"

#define  GSM_RXBUF_MAXSIZE           64//0

static uint16_t rxReadIndex  = 0;
static uint16_t rxWriteIndex = 0;
static uint16_t rxCount      = 0;
static uint8_t Gsm_RxBuf[GSM_RXBUF_MAXSIZE];

//extern uint8_t uart2_rx_data;
extern int R485_UART_TxBuf(uint8_t *buffer, int nbytes);
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  AppTaskGPRS ( void const * argument );

void delay_ms(uint32_t ms)
{
		nrf_delay_ms(ms);
}

int GSM_UART_TxBuf(uint8_t *buffer, int nbytes)
{
		uint32_t err_code;
		for (uint32_t i = 0; i < nbytes; i++)
		{
				do
				{
						err_code = app_uart_put(buffer[i]);
						if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
						{
								//NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
								APP_ERROR_CHECK(err_code);
						}
				} while (err_code == NRF_ERROR_BUSY);
		}
		return err_code;
}

void Gsm_RingBuf(uint8_t in_data)
{
		Gsm_RxBuf[rxWriteIndex]=in_data;
    rxWriteIndex++;
    rxCount++;
                      
    if (rxWriteIndex == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
    }
            
     /* Check for overflow */
    if (rxCount == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
        rxCount      = 0;
        rxReadIndex  = 0;
    }  
}

		
void Gsm_PowerUp(void)
{
	#if 1
    DPRINTF("GMS_PowerUp\r\n");
    GSM_PWR_OFF;
    delay_ms(200);
    /*Pwr en wait at least 30ms*/
    GSM_PWR_ON;
    delay_ms(200);
    /*Pwr key low to high at least 2S*/
    GSM_PWRKEY_LOW;
    delay_ms(2000); //2s
    GSM_PWRKEY_HIGH;
    delay_ms(1000);
	#endif
}

void Gsm_PowerDown(void)
{
	#if 0
    DPRINTF(LOG_DEBUG,"GMS_PowerDown\r\n");
    GSM_PWD_LOW;
    delay_ms(800); //800ms     600ms > t >1000ms  
    GSM_PWD_HIGH;
    delay_ms(12000); //12s
	  GSM_PWR_EN_DISABLE;
    delay_ms(2000);
	#endif
}

int Gsm_RxByte(void)
{
    int c = -1;

    __disable_irq();
    if (rxCount > 0)
    {
        c = Gsm_RxBuf[rxReadIndex];
				
        rxReadIndex++;
        if (rxReadIndex == GSM_RXBUF_MAXSIZE)
        {
            rxReadIndex = 0;
        }
        rxCount--;
    }
    __enable_irq();

    return c;
}

int Gsm_WaitRspOK(char *rsp_value,uint16_t timeout_ms,uint8_t is_rf)
{
    int retavl=-1,wait_len=0;
    uint16_t time_count=timeout_ms;
    char str_tmp[GSM_GENER_CMD_LEN];
    uint8_t i=0;
    
    memset(str_tmp,0,GSM_GENER_CMD_LEN);
    wait_len=is_rf?strlen(GSM_CMD_RSP_OK_RF):strlen(GSM_CMD_RSP_OK);
       
    do
    {
        int       c;
        c=Gsm_RxByte();
        if(c<0)
        {
            time_count--;
            delay_ms(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);
				//SEGGER_RTT_printf(0, "%c", c);
        str_tmp[i++]=(char)c;
        if(i>=wait_len)
        {
            char *cmp_p=NULL;
            if(is_rf)
                cmp_p=strstr(str_tmp,GSM_CMD_RSP_OK_RF);
            else
                cmp_p=strstr(str_tmp,GSM_CMD_RSP_OK);
            if(cmp_p)
            {
                if(i>wait_len&&rsp_value!=NULL)
                {
                    //printf("--%s  len=%d\r\n", str_tmp, (cmp_p-str_tmp)); 
                    memcpy(rsp_value,str_tmp,(cmp_p-str_tmp));
                }               
                retavl=0;
                break;
            }
                   
        }
    }while(time_count>0);
    
    //DPRINTF(LOG_DEBUG,"\r\n");
    return retavl;   
}

int Gsm_WaitSendAck(uint16_t timeout_ms)
{
    int retavl=-1;
    uint16_t time_count=timeout_ms;    
    do
    {
        int       c;
        c=Gsm_RxByte();
        if(c<0)
        {
            time_count--;
            delay_ms(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);       
        if((char)c=='>')
        {
            retavl=0;
            break;
        }   
    }while(time_count>0);
    
    //DPRINTF(LOG_DEBUG,"\r\n");
    return retavl;   
}

int Gsm_WaitNewLine(char *rsp_value,uint16_t timeout_ms)
{
    int retavl=-1;
		int i=0;
    uint16_t time_count=timeout_ms;   
		char new_line[3]={0};
		char str_tmp[GSM_GENER_CMD_LEN];
		char *cmp_p=NULL;
		memset(str_tmp, 0,GSM_GENER_CMD_LEN);
    do
    {
        int       c;
        c=Gsm_RxByte();
        if(c<0)
        {
            time_count--;
            delay_ms(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);
				SEGGER_RTT_printf(0, "%c", c);
				str_tmp[i++]=c;
				if(i>strlen("\r\n"))
				{
						if((cmp_p = strstr(str_tmp, "\r\n")) != NULL)
						{
								memcpy(rsp_value, str_tmp, (cmp_p-str_tmp));
								retavl=0;
								break;
						}   				
				}

    }while(time_count>0);
    
    //DPRINTF(LOG_DEBUG,"\r\n");
    return retavl;   
}

int Gsm_AutoBaud(void)
{
    int retavl=-1,rety_cunt=GSM_AUTO_CMD_NUM;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_AUTO_CMD_STR);
        do
        {
//            DPRINTF(LOG_DEBUG,"\r\n auto baud rety\r\n");
            GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
            retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,NULL);
            delay_ms(500);
            rety_cunt--;
        }while(retavl!=0&& rety_cunt>0);
        
        free(cmd);
    }
    return retavl;
}

int Gsm_FixBaudCmd(int baud)
{
    int retavl=-1;
    char *cmd;
    
    cmd=(char*)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s%d%s\r\n",GSM_FIXBAUD_CMD_STR,baud,"&W");
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);
        
        free(cmd);
    }
    return retavl;
}

//close cmd echo
int Gsm_SetEchoCmd(int flag)
{
    int retavl=-1;
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s%d\r\n",GSM_SETECHO_CMD_STR,flag);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);
        
        free(cmd);
    }
    return retavl;
}
//Check SIM Card Status
int Gsm_CheckSimCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_CHECKSIM_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(cmd,GSM_GENER_CMD_TIMEOUT,true);
        
        if(retavl>=0)
        {
            if(NULL!=strstr(cmd,GSM_CHECKSIM_RSP_OK))
            {
                retavl=0;
            }  
            else
            {
                retavl=-1;
            }
        }
        
        
        free(cmd);
    }
    
    return retavl;
}
//Check Network register Status
int Gsm_CheckNetworkCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_CHECKNETWORK_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(cmd,GSM_GENER_CMD_TIMEOUT,true);
        
        if(retavl>=0)
        {
            
            if (strstr(cmd,GSM_CHECKNETWORK_RSP_OK))
            {
                retavl = 0;
            } 
            else if (strstr(cmd,GSM_CHECKNETWORK_RSP_OK_5)) 
            {
               retavl = 0;
            } 
            else {
               retavl = -1;
            }
        }
        
        
        free(cmd);
    }
    return retavl;
}


//Check GPRS  Status
int Gsm_CheckGPRSCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_CHECKGPRS_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(cmd,GSM_GENER_CMD_TIMEOUT,true);
        
        if(retavl>=0)
        {
            if(!strstr(cmd,GSM_CHECKGPRS_RSP_OK))
            {
                retavl=-1;
            }  
        }
       
        free(cmd);
    }
    return retavl;
}

//Set context 0
int Gsm_SetContextCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_SETCONTEXT_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);       
       
        free(cmd);
    }
    return retavl;
}

//Set dns mode, if 1 domain, 
int Gsm_SetDnsModeCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_SETDNSMODE_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);       
       
        free(cmd);
    }
    return retavl;
}
//Check  ATS=1
int Gsm_SetAtsEnCmd(void)
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_ATS_ENABLE_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);       
       
        free(cmd);
    }
    return retavl;
}

//Check  Rssi
int Gsm_GetRssiCmd()
{
    int retavl=-1;
    //
    char *cmd;
    
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_RSSI_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(cmd,GSM_GENER_CMD_TIMEOUT,true);
        
        if(retavl>=0)
        {
            
            if(!strstr(cmd,GSM_GETRSSI_RSP_OK))
            {
                retavl=-1;
            }  
            else
            {
                
                retavl=atoi(cmd+2+strlen(GSM_GETRSSI_RSP_OK));
                if (retavl == 0) 
                {
                   retavl =53; //113
                }
                else if (retavl ==1) 
                {
                     retavl =111;
                }
                else if (retavl >=2 && retavl <= 30)
                {                   
                    retavl -=2;
                    retavl = 109- retavl*2;                  
                }
                else if (retavl >= 31)
                {
                   retavl =51;
                }         
            }
        }
       
        free(cmd);
    }
    return retavl;
}

void Gsm_CheckAutoBaud(void)
{
    uint8_t  is_auto = true, i=0;
    uint16_t time_count =0;
    uint8_t  str_tmp[64];
	
    delay_ms(800); 
    //check is AutoBaud 
    memset(str_tmp, 0, 64);
	
    do
    {
        int       c;
        c = Gsm_RxByte();
        if(c<=0)
        {
            time_count++;
            delay_ms(2);
            continue;
        }
				
        //R485_UART_TxBuf((uint8_t *)&c,1);		
				if(i<64) {
           str_tmp[i++]=(char)c;
				}
	
        if (i>3 && is_auto == true)
        {
            if(strstr((const char*)str_tmp, FIX_BAUD_URC))
            {
                is_auto = false;
                time_count = 800;  //Delay 400ms          
            }  
        }
    }while(time_count<1000);  //time out 2000ms
    
    if(is_auto==true)
    {
        Gsm_AutoBaud();
       
//        DPRINTF(LOG_DEBUG,"\r\n  Fix baud\r\n");
        Gsm_FixBaudCmd(GSM_FIX_BAUD);
    }  
}


int Gsm_WaitRspTcpConnt(char *rsp_value,uint16_t timeout_ms)
{
    int retavl=-1,wait_len=0;
    uint16_t time_count=timeout_ms;
    char str_tmp[GSM_GENER_CMD_LEN];
    uint8_t i=0;
    
    memset(str_tmp,0,GSM_GENER_CMD_LEN);
    
    wait_len=strlen(GSM_OPENSOCKET_OK_STR);
    do
    {
        int       c;
        c=Gsm_RxByte();
        if(c<0)
        {
            time_count--;
            delay_ms(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);
        str_tmp[i++]=(char)c;
        if(i>=wait_len)
        {
            char *cmp_ok_p=NULL,*cmp_fail_p=NULL;
            
            cmp_ok_p=strstr(str_tmp,GSM_OPENSOCKET_OK_STR);
            cmp_fail_p=strstr(str_tmp,GSM_OPENSOCKET_FAIL_STR);
            if(cmp_ok_p)
            {
                if(i>wait_len&&rsp_value!=NULL)
                {
                     memcpy(rsp_value,str_tmp,(cmp_ok_p-str_tmp));
                }               
                retavl=0;
                break;
            }
            else if(cmp_fail_p)
            {
                retavl=GSM_SOCKET_CONNECT_ERR;
                break;
            }
                      
        }
    }while(time_count>0);
    
//    DPRINTF(LOG_DEBUG,"\r\n");
    return retavl;   
}

//Open socket 
int Gsm_OpenSocketCmd(uint8_t SocketType,char *ip,uint16_t DestPort)
{
    int retavl=-1;
    //
    char *cmd;
      
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s%s,\"%s\",\"%d\"\r\n",GSM_OPENSOCKET_CMD_STR,
                        (SocketType==GSM_TCP_TYPE)?GSM_TCP_STR:GSM_UDP_STR,
                         ip,DestPort);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        memset(cmd,0,GSM_GENER_CMD_LEN);
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);       
       
        if(retavl>=0)
        {   
            //recv rsp msg...
            retavl=Gsm_WaitRspTcpConnt(NULL,GSM_OPENSOCKET_CMD_TIMEOUT);           
        }
        
        free(cmd);
    }
    return retavl;
}




//  send data 
int Gsm_SendDataCmd(char *data,uint16_t len)
{
    int retavl=-1;
    //
    char *cmd;
      
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s%d\r\n",GSM_SENDDATA_CMD_STR,len);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        retavl = Gsm_WaitSendAck(GSM_GENER_CMD_TIMEOUT*4);//500*4=2000ms
        if(retavl ==0)
        {
            GSM_UART_TxBuf((uint8_t *)data,len);
            retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true); 
            if(retavl==0)
            {
                retavl=len;
            } 
						else
						{
								//DPRINTF(LOG_INFO, "ret2=%d", retavl);
						}
        }  
				else
				{
						//DPRINTF(LOG_INFO, "ret1=%d", retavl);
				}
        free(cmd);
    }
    return retavl;
}


uint16_t Gsm_RecvData(char *recv_buf,uint16_t block_ms)
{
      int         c=0, i=0;      
      //OS_ERR      os_err; 
      //uint32_t    LocalTime=0, StartTime=0;
            
      int time_left = block_ms;
      
      do
      {
          c= Gsm_RxByte();
          if (c < 0) 
          {                 
               time_left--;
               delay_ms(1); 			 
               continue;
          }
          //R485_UART_TxBuf((uint8_t *)&c,1);
          recv_buf[i++] =(char)c;
          
      }while(time_left>0);
      
      return i;
}

//  close socket 
int Gsm_CloseSocketCmd(void)
{
    int retavl=-1;
    char *cmd;
      
    cmd=(char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd,0,GSM_GENER_CMD_LEN);
        cmd_len=sprintf(cmd,"%s\r\n",GSM_CLOSESOCKET_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd,cmd_len);
        
        retavl=Gsm_WaitRspOK(NULL,GSM_GENER_CMD_TIMEOUT,true);
        
        free(cmd);
    }
    return retavl;
}

/**
* @}
*/





