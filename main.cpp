#include "mbed.h"
#include "EthernetInterface.h"
#include "SNTPClient.h"
#include "TCPSocketConnection.h"
    #define FLAGSTARTSTOP   0x7e
    #define FLAGTRASPARENT  0x7d
    #define NOTMASKTRASPARENT 0x20
    #define XON            0x11
    #define XOFF           0x13
    #define W7500x_WZTOE_BASE   (0x46000000)
    #define SIPR                (0x6010)
    #define GAR                 (0x6008)
    #define SUBR                (0x600C)
    
  
DigitalOut HL4(PC_04); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut HL6(PC_05); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut HL5(PC_00); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE

/*
DigitalOut myledG(PC_04); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut myledB(PC_05); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut myledR(PC_00); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
*/

DigitalOut myledG(PC_08); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut myledB(PC_09); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
DigitalOut myledR(PC_12); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE


Serial pc(PC_10, PC_11); // tx, rx
//    pc.printf("Server IP Address is %s\r\n", eth.getIPAddress());
Serial puart(PC_2, PC_3); // tx, rx 
DigitalOut mGND(PA_2); ///PC_04=GREEN, PC_00=RED, PC_05=BLUE
Ticker step;
Ticker step9600;
Ticker estep9600;
EthernetInterface eth;
datetime ntptime;

const uint16_t  fcstab[256] = {
   0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
   0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
   0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
   0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
   0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
   0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
   0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
   0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
   0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
   0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
   0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
   0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
   0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
   0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
   0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
   0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
   0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
   0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
   0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
   0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
   0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
   0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
   0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
   0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
   0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
   0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
   0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
   0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
   0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
   0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
   0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
   0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
   };
char hex[]=  "0123456789ABCDEF";
char tms[] = "000.000.000.000";   
int ECHO_SERVER_PORT =  80;
char ip_addr[] = "192.168.001.142";
char subnet_mask[] = "255.255.255.000";
char gateway_addr[] = "192.168.022.001";
char sss[256],rsrv[128];   
int flgD=0,tmrT=0,SECi,SECm,SECd,DELTA=100;
uint8_t mac_addr[6] = {0x00, 0x08, 0xDC, 0x55, 0x51, 0x52}; 
char buffer[256];
uint8_t bufmet[256], TCPtimeout, MTtimeout=3, TimeZone, NTPperiod;
uint8_t Prefix,Version,CfgID[4],DH_CP,TMCOR=0;

int tmr00,flg=0,nn=2;
uint8_t sct,tx_in=0, tx_out=0, UART2TX[256],tx1_in=0, tx1_out=0, UART1TX[256];
uint8_t rx_ptr=0,buff[256];

char rx_buf[256],errr[]="noting";
uint16_t TCPSS_tmr, crc, TMSerr=0;
bool TCPSS_blk;
uint8_t flsh0[]={0x7E,0x50,0xFF,0xFF,0xFF,0xFE,0x02,0x10,0x10,0x61,0x01,0x00,0x00,0x00,0x66,0x6F,0x2B,0x7E};
uint8_t flsh1[]={0x7E,0x50,0xFF,0xFF,0xFF,0xFE,0x02,0x10,0x10,0x61,0x01,0x01,0x00,0x00,0x67,0x5D,0x26,0x7E};
uint8_t flsh2[]={0x7E,0x50,0xFF,0xFF,0xFF,0xFE,0x02,0x10,0x10,0x61,0x01,0x02,0x00,0x00,0x64,0x0B,0x31,0x7E};
uint8_t flsh3[]={0x7E,0x50,0xFF,0xFF,0xFF,0xFE,0x02,0x10,0x10,0x61,0x01,0x03,0x00,0x00,0x65,0x39,0x3C,0x7E};
uint8_t fl0[] = {0x10,0x10,0x61,0x01,0x00,0x00,0x00};
uint8_t timeR[] = {0x7E,0x50,0xFF,0xFF,0xFE,0xFF,0x66,0x10,0x10,0x07,0x00,0x00,0x00,0x00,0xFE,0xD0,0x7E};
uint8_t TCor[]={0x10,0x10,0x5C,0x02,0x00,0x05,0x00,0x00};//7E50FFFFFFFE 0B10105C0200050000 16C97E

uint8_t cnfg[] = {
0x10,0x10,0x60,0x41,
0x00,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,
00,0x00,0x00};

uint8_t stts[] = {
0x10,0x10,0x60,0x41,    
0x03,
0xda,0x3c,0x14, //[5...7]                                   //1-3
0x00,0x03,  //HW stts[8,9]                                  //4-5
0x01,0x03,  //FW stts[10,11]                                //6-7
0xCD,0xEF,0x12,0x34, //IP stts[12...15]
0x56,0x78,0xFF,0xAB, //MASK stts[16...19]
0xCD,0xEF,0x12,0x34, //GATEWAY stts[20...23]
0x77,0xAB,0x56,0x78,0xFF, 0xAB,//WORKTime stts[24,29]

0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,//[30...36]
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,//[37...44]
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,//45...52]
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,//53...60]
0xAB,0xCD,0xEF,0x12,0x34,0x56,0x78,0xFF,//61...68]          //57-63
00,0x00,0x00};//[69..71]                                    //64-66
// CRC [68,69]


uint8_t sample[] = {
0xda,0x3c,0xae,0x12,0xcd,0xd9,0x3a,0x57,
0x02,0x76,0x70,0x6e,0x69,0x2e,0x6b,0x79,
0x69,0x76,0x73,0x74,0x61,0x72,0x2e,0x6e,
0x65,0x74,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x04,0xa0};
void send_to_metter (uint8_t* buffer, uint8_t lenght);
void read_RTC ();                    
uint32_t hex2bcd (uint32_t x) 
{ 
    uint16_t y1,y2,y3;
    uint32_t y,y4;

    y4 = (x / 10000);
    y4=y4<< 16; 
    y4 = y4&0x0f0000;
    x = x % 10000;  
    
    y3 = (x / 1000) << 12; 
    y3 = y3&0x0f000;
    x = x % 1000;
    
    y2 = (x / 100) << 8; 
    y2 = y2&0x0f00;
    x = x % 100;
    
    y1 = (x / 10) << 4; 
    y1 = y1&0x0f0;
    x = x % 10;  
    
    y = y1|y2|y3|y4|x;
    return (y); 
}



uint16_t  fcs16(uint16_t  fcs, uint8_t  * cp, uint16_t  len)
{
   while (len--)
      {
      fcs = (fcs >> 8) ^ fcstab[(fcs ^ *cp++) & 0xff];
      }
   return (fcs);
}
void TMRTX1() 
{

// Loop to fill more than one character in UART's transmit FIFO buffer
// Stop if buffer empty
HL4=0;
    if ((puart.writeable()) && (tx1_in != tx1_out)) 
    {
        *(volatile uint32_t *)(0x4000d000)=UART1TX[tx1_out++];
  //      puart.putc(UART1TX[tx1_out++]);
    }
    if (tx1_in == tx1_out)     
    {
        estep9600.detach(); 
            rx_ptr=0; HL4=1;
        }

}
void TMRTX() 
{

// Loop to fill more than one character in UART's transmit FIFO buffer
// Stop if buffer empty
    if ((pc.writeable()) && (tx_in != tx_out)) 
    {
        *(volatile uint32_t *)(0x40006000)=UART2TX[tx_out++];
 //       pc.putc(UART2TX[tx_out++]);
    }
    if (tx_in == tx_out)     step9600.detach(); 

}
void printstr(char* str, uint8_t aa)
{
    for(int i = 0; i < aa; i++) 
    {
        UART2TX[tx_in++] = str[i];
    }
    step9600.attach(&TMRTX, 0.0015); 
}
void printstrmet(char* str, uint8_t aa)
{
    for(int i = 0; i < aa; i++) 
    {
        UART1TX[tx1_in++] = str[i];
    }
    estep9600.attach(&TMRTX1, 0.0015); 
}
void TMR() 
{

    tmr00++;
    tmrT++;
 
}
void rx_interruptUART() {
    char ch;
HL5=0;
    if(puart.readable())
    {  //Determine if there is a character available to read
        ch = puart.getc(); //Read char from terminal
        rx_buf[rx_ptr++]=ch;
        if ((ch==0x7e)&&(rx_ptr>2))
        {
        flg=1; HL5=1;
        }
    }
    return;
}




int main (void) 
{

    myledR=myledG=myledB=1;     mGND=0; myledR=0; HL4=HL5=HL6=1;
    step.attach(&TMR, 1);         
    puart.attach(&rx_interruptUART, Serial::RxIrq);

//=============================================START=============================    
        myledR=myledG=myledB=0; 
HL6=0;
        wait(3);
HL6=1;
        printstr(sss, sprintf (sss, "Programm is starting...\r\n"));  
        myledR=myledG=myledB=1;  myledR=0;
//=======read config======================
            wait(0.1);printstr(sss, sprintf (sss, "Read CONFIG from metter...\r\n"));   
            send_to_metter (fl0, sizeof fl0);
            tmr00=0;            flg=0;
            while ((tmr00<MTtimeout)&&(flg==0)) myledR=0;
            if (flg==0)
            {
                printstr(sss, sprintf (sss, "Metter do not respond, RESET...\r\n"));   
                wait(0.1);
                NVIC_SystemReset();
            }
            else
            {
                uint8_t j=1,i=0;
                while (rx_buf[j]!=0x7e) 
                {
                    if (rx_buf[j]!=0x7d)
                    {
                        bufmet[i++]=rx_buf[j++];
                    }
                    else 
                    {
                        j++; bufmet[i++]=rx_buf[j++]^0x20;
                    }
                
                }
//=========================bufmet - afther BS witch CRC            
                printstr(sss, sprintf (sss, "Afther byte-stuffing: \r\n"));   
                j=i;
                for (int i=0;i<j;i++) 
                {
                    UART2TX[tx_in++] = (hex[(bufmet[i]>>4)&0xf]);
                    UART2TX[tx_in++] = (hex[(bufmet[i])&0xf]);
                    if ((i&0x1f)==0x1f)
                    {
                        UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a); 
                    }
                }  
            UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);  
            }

            uint16_t crc0,crc= bufmet[72]<<8;
             crc = crc + bufmet[71];
            wait(0.1);printstr(sss, sprintf (sss, "CRC recieved= %d\r\n", crc));   
            crc0=fcs16(0xFFFF, bufmet+13,58) ^ 0xFFFF;
            wait(0.1);printstr(sss, sprintf (sss, "CRC calculated = %d\r\n", crc0));   
    if (crc==crc0)
    {
        for (crc=13;crc<48;crc++) rsrv[crc]=bufmet[crc];

            Prefix = bufmet[13];    
            Version = bufmet[14];   
            CfgID[0]= bufmet[15];   CfgID[1]= bufmet[16];    CfgID[2]= bufmet[17];  CfgID[3]= bufmet[18];
            mac_addr[0]=bufmet[19]; mac_addr[1]=bufmet[20]; mac_addr[2]=bufmet[21];
            mac_addr[3]=bufmet[22]; mac_addr[4]=bufmet[23]; mac_addr[5]=bufmet[24];    
              
            sprintf (ip_addr, "%03d",bufmet[25]);    ip_addr[3]='.';
            sprintf (ip_addr+4, "%03d",bufmet[26]);  ip_addr[7]='.';
            sprintf (ip_addr+8, "%03d",bufmet[27]);  ip_addr[11]='.';
            sprintf (ip_addr+12, "%03d",bufmet[28]);  ip_addr[15]=0;
            wait(0.1);printstr(sss, sprintf (sss, "IP Address from METTER is %s\r\n", ip_addr));   
            
            ECHO_SERVER_PORT = (bufmet[30]<<8)|bufmet[29];
            wait(0.1); printstr(sss, sprintf (sss, "ECHO_SERVER_PORT from METTER is %d\r\n", ECHO_SERVER_PORT ));   
            
            DH_CP = bufmet[31];
            

            sprintf (subnet_mask, "%03d",bufmet[32]);    subnet_mask[3]='.';
            sprintf (subnet_mask+4, "%03d",bufmet[33]);  subnet_mask[7]='.';
            sprintf (subnet_mask+8, "%03d",bufmet[34]);  subnet_mask[11]='.';
            sprintf (subnet_mask+12, "%03d",bufmet[35]);  subnet_mask[15]=0;
            wait(0.1);printstr(sss, sprintf (sss, "subnet_mask from METTER is %s\r\n", subnet_mask));  

            sprintf (gateway_addr, "%03d",bufmet[36]);    gateway_addr[3]='.';
            sprintf (gateway_addr+4, "%03d",bufmet[37]);  gateway_addr[7]='.';
            sprintf (gateway_addr+8, "%03d",bufmet[38]);  gateway_addr[11]='.';
            sprintf (gateway_addr+12, "%03d",bufmet[39]);  gateway_addr[15]=0;
            wait(0.1);printstr(sss, sprintf (sss, "gateway_addr from METTER is %s\r\n", gateway_addr));  

            sprintf (tms, "%d",bufmet[40]);    
            uint8_t i=0; while (tms[i]!=0)
            { 
                i++;
            }
                 tms[i++]='.';
            sprintf (tms+i, "%d",bufmet[41]);                          
            i=0; while (tms[i]!=0)
            { 
                i++;
            }
                 tms[i++]='.';
            sprintf (tms+i, "%d",bufmet[42]);  
            i=0; while (tms[i]!=0)
            { 
                i++;
            }
                 tms[i++]='.';
            sprintf (tms+i, "%d",bufmet[43]);  
                             
            wait(0.1);printstr(sss, sprintf (sss, "TMServer IP is %s\r\n", tms));  
            TCPtimeout = bufmet[44];
            wait(0.1);printstr(sss, sprintf (sss, "TCP timeout (sec) = %d\r\n", TCPtimeout));  
            MTtimeout = bufmet[45];
            wait(0.1);printstr(sss, sprintf (sss, "Metter timeout (sec) = %d\r\n", MTtimeout));              
            TimeZone = bufmet[46];
            wait(0.1);printstr(sss, sprintf (sss, "TimeZone is %d\r\n", TimeZone));  
            DELTA = bufmet[47]*60;
            wait(0.1);printstr(sss, sprintf (sss, "DELTA (sec) = %d\r\n", DELTA));  

            if (DH_CP==0xff)
            {
                 wait(0.1); printstr(sss, sprintf (sss, "Try to REGISTER in DHCP_MODE \r\n" ));   
            }
            else 
            {
                 wait(0.1); printstr(sss, sprintf (sss, "Try to REGISTER witch fix IP from METTER \r\n" ));   
            }

                EthernetInterface eth;
                if (DH_CP==0xff)   eth.init(mac_addr); //Use DHCP
                else     eth.init(mac_addr, ip_addr, subnet_mask, gateway_addr);
                eth.connect();
//    pc.printf("Server IP Address is %s\r\n", eth.getIPAddress());
                wait(0.1); printstr(sss, sprintf (sss, "Server IP Address is %s\r\n", eth.getIPAddress()));   wait(0.1);
                if (*(volatile uint32_t *)(W7500x_WZTOE_BASE+SIPR)==0) NVIC_SystemReset();
            }
    else 
    {
        wait(0.1); printstr(sss, sprintf (sss, "CRC is INCORRECT! Try to REGISTER in DHCP_MODE... \r\n" ));   
        eth.init(mac_addr); 
        eth.connect();
        wait(0.1); printstr(sss, sprintf (sss, "Server IP Address is %s\r\n", eth.getIPAddress()));   
        if (*(volatile uint32_t *)(W7500x_WZTOE_BASE+SIPR)==0) NVIC_SystemReset();
    }

  
    
    myledG=0;
    TCPSocketServer server;
    server.bind(ECHO_SERVER_PORT);
    server.listen();
//=========================================    
        read_RTC();        

            uint32_t stt = *(volatile uint32_t *)(W7500x_WZTOE_BASE+SIPR);
            stts[12] = (stt>>24)&0xff;   stts[13] = (stt>>16)&0xff;     stts[14] = (stt>>8)&0xff;   stts[15] = (stt)&0xff; //IP
            stt = *(volatile uint32_t *)(W7500x_WZTOE_BASE+SUBR);       
            stts[16] = (stt>>24)&0xff;   stts[17] = (stt>>16)&0xff;     stts[18] = (stt>>8)&0xff;   stts[19] = (stt)&0xff; //IP 
            stt = *(volatile uint32_t *)(W7500x_WZTOE_BASE+GAR); 
            stts[20] = (stt>>24)&0xff;   stts[21] = (stt>>16)&0xff;     stts[22] = (stt>>8)&0xff;   stts[23] = (stt)&0xff; //IP    
            stts[24] = bufmet[17]; stts[25] = bufmet[16]; stts[26] = bufmet[15]; stts[27] = bufmet[13]; stts[28] = bufmet[12]; stts[29] = bufmet[11];
            
                crc=fcs16(0xFFFF, stts+5,62) ^ 0xFFFF;
                stts[68]=crc & 0x00FF;//crcL;  
                stts[69]=(crc >> 8) & 0x00FF;//crcH;    
            
            wait(0.1);  send_to_metter (stts, sizeof stts);
            tmr00=0;    flg=0;
            while ((tmr00<MTtimeout)&&(flg==0)) myledR=0;
            if (flg==0)
            {
                wait(0.1); printstr(sss, sprintf (sss, "Metter do not respond afther WRITE STATUS, RESET...\r\n"));   
                NVIC_SystemReset();
            }
            else
            {
                wait(0.1); printstr(sss, sprintf (sss, "Status has saved in the METTER...\r\n"));   
            }   
                   
//=========================================                  
    
    
    while (true) 
    {
            myledR=myledG=myledB=1; myledB=0;  
            printstr(sss, sprintf (sss, "Wait for new connection...\r\n"));
            TCPSocketConnection client;

            printstr(sss, sprintf (sss, "TCPSocketConnection client complete\r\n"));
            client.set_blocking(false, 250); // Timeout after (15)s   
            TCPSS_tmr=250; TCPSS_blk=true;       

        while(1)
        {
            if ((tmrT&0x1)==1) 
            {
                myledG=0;myledR=1; HL6=1;
            }
            else
            {
                myledG=1;myledR=0; HL6=0;
            }
//===============================add check CONFIG=========================================        
            if (tmrT==30)
            {
            wait(0.1);printstr(sss, sprintf (sss, "Read CONFIG from metter...\r\n"));   
            send_to_metter (fl0, sizeof fl0);
            tmr00=0;            flg=0;
            while ((tmr00<MTtimeout)&&(flg==0)) myledR=0;
            if (flg==0)
            {
                printstr(sss, sprintf (sss, "Metter do not respond, RESET...\r\n"));   
                wait(0.1);
                NVIC_SystemReset();
            }
            else
            {
                uint8_t j=1,i=0;
                while (rx_buf[j]!=0x7e) 
                {
                    if (rx_buf[j]!=0x7d)
                    {
                        bufmet[i++]=rx_buf[j++];
                    }
                    else 
                    {
                        j++; bufmet[i++]=rx_buf[j++]^0x20;
                    }
                
                }                
                printstr(sss, sprintf (sss, "Afther byte-stuffing: \r\n"));   
                j=i;
                for (int i=0;i<j;i++) 
                {
                    UART2TX[tx_in++] = (hex[(bufmet[i]>>4)&0xf]);
                    UART2TX[tx_in++] = (hex[(bufmet[i])&0xf]);
                    if ((i&0x1f)==0x1f)
                    {
                        UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a); 
                    }
                }  
            UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);  
            }   
            crc= bufmet[72]<<8;
            crc = crc + bufmet[71];
            crc0=fcs16(0xFFFF, bufmet+13,58) ^ 0xFFFF;
            if (crc==crc0)      
            {
                crc0=0;
                wait(0.1);printstr(sss, sprintf (sss, "CRC OK!\r\n"));   
                for (crc=13;crc<48;crc++) 
                {
                    if (rsrv[crc]!=bufmet[crc]) crc0++;
                }
                if (crc0!=0)
                {
                    wait(0.1);printstr(sss, sprintf (sss, "STATUS was changer,RESTART!\r\n"));   
                    wait(0.1);
                    NVIC_SystemReset();
                }
                else 
                {
                    wait(0.1);printstr(sss, sprintf (sss, "STATUS the same \r\n"));   
                }
            }   
            
            
            
            
            }
//===============================add check CONFIG=========================================           
        if (tmrT>=60)
        {
            tmrT=0;

            wait(0.1); SNTPClient sntp(tms, TimeZone);   

            sntp.connect();
            if (sntp.getTime(&ntptime) != true)
            {
                        TMSerr++;
                        printstr(sss, sprintf (sss, "TMS do no response %d\r\n",TMSerr));
            }
            else    
            {
                printstr(sss, sprintf (sss, "Data&time is %d-%d-%d, %02d:%02d:%02d\r\n", ntptime.yy, ntptime.mo, ntptime.dd, ntptime.hh, ntptime.mm, ntptime.ss));
                if ((TMCOR==0)&&(ntptime.hh>2)&&(ntptime.hh<18))
                {
                    TMCOR=1;
                    read_RTC(); 
                    SECi = ntptime.hh*3600 + ntptime.mm*60 + ntptime.ss;
                    SECm = bufmet[13]*3600 + bufmet[12]*60 + bufmet[11];
                    wait(0.1); printstr(sss, sprintf (sss, "SECi=%d, SECm=%d\r\n",SECi,SECm));
                    SECd=SECi-SECm;
                    if ((SECd<DELTA)||(SECd>(65536-DELTA)))
                    {
                        TCor[4] = (SECd>>8)&0xff;
                        TCor[5] = (SECd)&0xff;
                        send_to_metter (TCor, sizeof TCor);
                        tmr00=0;            flg=0;
                        while ((tmr00<3)&&(flg==0)) myledR=0;
                        if (flg==0)
                        {
                            printstr(sss, sprintf (sss, "TIME&DATE can't SAVE, RESET...\r\n"));   
                            wait(1);
                            NVIC_SystemReset();
                        }
                    }
                }
            }

        }
        if (DH_CP==0xff)
        {
            if (!eth.link()) 
            {
                printstr(sss, sprintf (sss, "!eth.link do no response\r\n"));
                wait(1); 
                NVIC_SystemReset();
            }
        }

        if (!server.accept(client))   break;
    }
         
                
//================CONNECT==================================                
                
        client.set_blocking(false, 500); // Timeout after (15)s               
        printstr(sss, sprintf (sss, "Connection from: %s\r\n", client.get_address()));

        myledR=myledG=myledB=1; myledR=0;


        while (true) 
        {
            myledR=myledG=myledB=1; myledG=0; 

            int16_t addr =Sn_RX_RSR;
            uint8_t cb = (0x01+(sct<<2));

 //           printstr(sss, sprintf (sss, "in main socket = %d\n\r",sct));
            uint8_t bTCP=0,nnt=0;
            while ((bTCP==0)&&(nnt<TCPtimeout*2))
            {
                wait(0.5);
                 nnt++;
                bTCP= *(volatile uint8_t *)(W7500x_WZTOE_BASE + (uint32_t)((cb<<16)+addr));   

            }
            if (bTCP!=0) printstr(sss, sprintf (sss, "Data Lenght from ETHERNET = %d\n\r",bTCP));
            wait(0.1);
            if (nnt>(2*TCPtimeout-1)) break;
           
            int n = client.receive_all(buffer, sizeof(buffer));      
             if (n <= 0) break;                       
           n=1;
            while ((buffer[n]!=0x7e)&&(n<254))
            {
                n++;
                }
            printstr(sss, sprintf (sss, "Data: \r\n"));   
            for (int i=0;i<=n;i++) 
            {
                UART2TX[tx_in++] = (hex[(buffer[i]>>4)&0xf]);
                UART2TX[tx_in++] = (hex[(buffer[i])&0xf]);
            }
                UART2TX[tx_in++] = (0x0d); UART2TX[tx_in++] = (0x0a); step9600.attach(&TMRTX, 0.0015); 
                wait(0.5);
            printstr(sss, sprintf (sss, "Data for metter: \r\n"));               

           

         

            if ( ((buffer[1]==0x7d)&&(buffer[2]==0x33)) || buffer[1]==0x12|| buffer[1]==0x10  || ((buffer[1]==0x7d)&&(buffer[2]==0x31)) )
            {
                flgD=1;
                if (buffer[n-2]==0x7d) n--;
                if (buffer[n-3]==0x7d) n--;                
                if (buffer[n-4]==0x7d) n--;
                uint8_t i=0;
            bufmet[i++]=0x7e;
            bufmet[i++]=0x50;                        
            bufmet[i++]=0xff;   bufmet[i++]=0xff;      
            bufmet[i++]=0xff;   bufmet[i++]=0xfe;     
            bufmet[i++]=(nn++)&0x0f; 
            //now need ByteStuffing
            uint8_t j=1,aa=n&0xff;
            while (j<(aa-2)) 
            {
                if (buffer[j]!=0x7d)
                {
                    bufmet[i++]=buffer[j++];
                }
                else 
                {
                    j++; bufmet[i++]=buffer[j++]^0x20;
                }
                
            }
            //now need to calculate CRC
                uint16_t crc=fcs16(0xFFFF, bufmet+1,i-1) ^ 0xFFFF;
                bufmet[i++]=crc & 0x00FF;//crcL;  
                bufmet[i++]=(crc >> 8) & 0x00FF;//crcH;    

            //now need ByteStuffing
             uint8_t   tx3_ptrf=0;crc=1;
             buffer[tx3_ptrf++]=0x7E;
             while (crc<i)
            { 
                if ((bufmet[crc] == FLAGSTARTSTOP) || (bufmet[crc] == FLAGTRASPARENT) || 
                (bufmet[crc] == XON) || (bufmet[crc] == XOFF))
                {
                    buffer[tx3_ptrf++] = FLAGTRASPARENT;
                    buffer[tx3_ptrf++] = bufmet[crc++] ^ NOTMASKTRASPARENT;
                }
                else
                {
                    buffer[tx3_ptrf++] = bufmet[crc++];
                }
            }
            buffer[tx3_ptrf++]=0x7e;  
            //send to UARTs     
            for (int i=0;i<tx3_ptrf;i++) 
            {
                UART2TX[tx_in++] = (hex[(buffer[i]>>4)&0xf]);
                UART2TX[tx_in++] = (hex[(buffer[i])&0xf]);
 //               UART2TX[tx_in++] = (0x20);
            }
                UART2TX[tx_in++] = (0x0d); UART2TX[tx_in++] = (0x0a);
                //send to uarts
            step9600.attach(&TMRTX, 0.0015); 
            printstrmet(buffer,tx3_ptrf);   //    for (int i=0;i<tx3_ptrf;i++) puart.putc(buffer[i]);
            }
            else 
            {
                flgD=0;
                for (int i=0;i<=n&0xff;i++) 
                {
                UART2TX[tx_in++] = (hex[(buffer[i]>>4)&0xf]);
                UART2TX[tx_in++] = (hex[(buffer[i])&0xf]);
 //               UART2TX[tx_in++] = (0x20);
                }
                UART2TX[tx_in++] = (0x0d);                 UART2TX[tx_in++] = (0x0a);                 step9600.attach(&TMRTX, 0.0015); 
                printstrmet(buffer,n+1);  //for (int i=0;i<n;i++) puart.putc(buffer[i]);
            }
            
            
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            tmr00=0;   myledR=myledG=myledB=1; myledR=0;
                                                            while (tx_in == tx_out);

            while ((tmr00<MTtimeout)&&(myledR==0))
            {
                if (flg==1)
                {
                            wait(0.1);printstr(sss, sprintf (sss, "Data from metter: "));
                            wait(0.1);
                            for (int i=0;i<rx_ptr;i++) 
                                {
                                    UART2TX[tx_in++] = (hex[(rx_buf[i]>>4)&0xf]);
                                    UART2TX[tx_in++] = (hex[(rx_buf[i])&0xf]);
                                }  
                                UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);   
                     
                     
                     
                     
                     if (flgD==0) 
                     {

                            wait(0.1);printstr(sss, sprintf (sss, "Send to Ethernet : "));
                            wait(0.1);
                            for (int i=0;i<rx_ptr;i++) 
                                {
                                    UART2TX[tx_in++] = (hex[(rx_buf[i]>>4)&0xf]);
                                    UART2TX[tx_in++] = (hex[(rx_buf[i])&0xf]);
                                }  
                                UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);   
                            client.send_all(rx_buf,rx_ptr); 
                     }
                     else
                     {
                        uint8_t i=0,j=7;
                        bufmet[i++]=0x7e;
                        if (rx_buf[rx_ptr-3]==0x7d) rx_ptr--;
                        if (rx_buf[rx_ptr-4]==0x7d) rx_ptr--;
                        if (rx_buf[rx_ptr-5]==0x7d) rx_ptr--;
                        ///bstf
                        while (j<rx_ptr-3) 
                        {
                            if (rx_buf[j]!=0x7d)
                            {
                                 bufmet[i++]=rx_buf[j++];
                            }
                            else 
                            {
                            j++; bufmet[i++]=rx_buf[j++]^0x20;
                            }
                
                        }
                        ////bstf
                        //now need to calculate CRC
                        uint16_t crc=fcs16(0xFFFF, bufmet+1,i-1) ^ 0xFFFF;
                        bufmet[i++]=crc & 0x00FF;//crcL;  
                        bufmet[i++]=(crc >> 8) & 0x00FF;//crcH;    
                        //now need ByteStuffing
                        uint8_t   tx3_ptrf=0;crc=1;
                        buffer[tx3_ptrf++]=0x7E;
                        while (crc<i)
                        { 
                            if ((bufmet[crc] == FLAGSTARTSTOP) || (bufmet[crc] == FLAGTRASPARENT) || 
                                (bufmet[crc] == XON) || (bufmet[crc] == XOFF))
                            {
                                buffer[tx3_ptrf++] = FLAGTRASPARENT;
                                buffer[tx3_ptrf++] = bufmet[crc++] ^ NOTMASKTRASPARENT;
                            }
                            else
                            {
                                buffer[tx3_ptrf++] = bufmet[crc++];
                            }
                        }
                        buffer[tx3_ptrf++]=0x7e;  
                        client.send_all(buffer,tx3_ptrf); 
                                    //send to UARTs     
                        wait(0.1); printstr(sss, sprintf (sss, "Send to Ethernet (coded):%d\r\n",rx_ptr));
                        wait(0.1);
                        for (int i=0;i<tx3_ptrf;i++) 
                        {
                            UART2TX[tx_in++] = (hex[(buffer[i]>>4)&0xf]);
                            UART2TX[tx_in++] = (hex[(buffer[i])&0xf]);
       //                     UART2TX[tx_in++] = (0x20);
                        }
                            UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);   
                     }
                     
                     
    
                        rx_ptr=0;
                        flg=0;                             
                        myledR=myledG=myledB=1; myledB=0; 
                }
            }
          if (myledB!=0) 
            {
                printstr(sss, sprintf (sss, "Metter do NOT response:-((('%s'\r\n",rx_buf));
                for (int i=0;i<16;i++) rx_buf[i]=0;
                rx_ptr=0;             
//                client.send_all(errr,6); 
            }
            

        }

        myledG=0; 
        client.close();
    }
    
}

                        
 void send_to_metter (uint8_t* buffer, uint8_t lenght)
{              
            char sss[256];          
               uint8_t i=0,  bufmet[256];
            bufmet[i++]=0x7e;
            bufmet[i++]=0x50;                        
            bufmet[i++]=0xff;   bufmet[i++]=0xff;      
            bufmet[i++]=0xff;   bufmet[i++]=0xfe;     
            bufmet[i++]=(nn++)&0xf;   
                
            //now need ByteStuffing
            for (uint8_t j=0;j<lenght;j++)  bufmet[i++]=buffer[j];

            //now need to calculate CRC
                uint16_t crc=fcs16(0xFFFF, bufmet+1,i-1) ^ 0xFFFF;
                bufmet[i++]=crc & 0x00FF;//crcL;  
                bufmet[i++]=(crc >> 8) & 0x00FF;//crcH;    

            //now need ByteStuffing
             uint8_t   tx3_ptrf=0;crc=1;
             buff[tx3_ptrf++]=0x7E;
             while (crc<i)
            { 
                if ((bufmet[crc] == FLAGSTARTSTOP) || (bufmet[crc] == FLAGTRASPARENT) || 
                (bufmet[crc] == XON) || (bufmet[crc] == XOFF))
                {
                    buff[tx3_ptrf++] = FLAGTRASPARENT;
                    buff[tx3_ptrf++] = bufmet[crc++] ^ NOTMASKTRASPARENT;
                }
                else
                {
                    buff[tx3_ptrf++] = bufmet[crc++];
                }
            }
            buff[tx3_ptrf++]=0x7e;  
            //send to UARTs     
            wait(0.5);printstr(sss, sprintf (sss, "Send to metter: \r\n"));   
            for (int i=0;i<tx3_ptrf;i++) 
            {
                UART2TX[tx_in++] = (hex[(buff[i]>>4)&0xf]);
                UART2TX[tx_in++] = (hex[(buff[i])&0xf]);
            }
                UART2TX[tx_in++] = (0x0d); UART2TX[tx_in++] = (0x0a); step9600.attach(&TMRTX, 0.0015); 
                rx_ptr=0;
                for(int i = 0; i < tx3_ptrf; i++) 
                {
                    UART1TX[tx1_in++] = buff[i];
                }
                estep9600.attach(&TMRTX1, 0.0015); 
}                          
void read_RTC()
{                
            for(int i = 0; i < 17; i++) 
                {
                    UART1TX[tx1_in++] = timeR[i];
                }
            rx_ptr=0;
            estep9600.attach(&TMRTX1, 0.0015); 
            tmr00=0;            flg=0;

            while ((tmr00<MTtimeout)&&(flg==0)) myledR=0;
            if (flg==0)
            {
                printstr(sss, sprintf (sss, "Metter do not respond when read TIME&DATE, RESET...\r\n"));   
                wait(1);
                NVIC_SystemReset();
            }
            else
            {
                wait(0.1); printstr(sss, sprintf (sss, "TIME&DATA from metter:\r\n"));   
                uint8_t j=1,i=0;
                while (rx_buf[j]!=0x7e) 
                {
                    if (rx_buf[j]!=0x7d)
                    {
                        bufmet[i++]=rx_buf[j++];
                    }
                    else 
                    {
                        j++; bufmet[i++]=rx_buf[j++]^0x20;
                    }
                
                }
                j=i;
                for (int i=0;i<j;i++) 
                {
                    UART2TX[tx_in++] = (hex[(bufmet[i]>>4)&0xf]);
                    UART2TX[tx_in++] = (hex[(bufmet[i])&0xf]);
                }  
                UART2TX[tx_in++] = 0x0d; UART2TX[tx_in++] = (0x0a);  step9600.attach(&TMRTX, 0.0015);  
                wait(0.4);  printstr(sss, sprintf (sss, "Data&time is %d-%d-%d, %02d:%02d:%02d\r\n", bufmet[17], bufmet[16], bufmet[15], bufmet[13], bufmet[12], bufmet[11]));
            }
}