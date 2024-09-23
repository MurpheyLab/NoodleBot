// slightly modified code from https://github.com/MarvelmindRobotics/arduino_sample_uart/blob/main/hedgehog_sample_uart_v6/hedgehog_sample_uart_v6.ino 

#include <stdlib.h>
#include "global.h"

#define SERIAL_MONITOR_PRINT

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

////// moved these to global.h //////
// float hedgehog_x = 0.; // x coordinate of hedgehog
// float hedgehog_y = 0.;
// float hedgehog_z = 0.;
// bool hedgehog_pos_updated = false; // flag of new data from hedgehog received
// bool high_resolution_mode = false;
// unsigned int hedgehog_paired_heading = 0.; // diection of paired hedgehog
// int64_t hedgehog_pos_timestamp;

float hedgehog_xvel = 0.; // x velocity of hedgehog
float hedgehog_yvel = 0.;
float hedgehog_zvel = 0.;
bool hedgehog_vel_updated = false; // flag of new vel data from hedgehog received

//#define DISTANCE_SENSOR_ENABLED

#define HEDGEHOG_BUF_SIZE 60 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define NT_POSITION_DATAGRAM_HIGHRES_ID 0x0081
unsigned int hedgehog_data_id;

byte hedgehog_address;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;
typedef union {byte b[8];int64_t vi64;} uni_8x8_64;

void process_hedgehog() {
  int incoming_byte;
  int total_received_in_loop;
  int packet_received;
  bool good_byte;
  byte packet_size;
  uni_8x2_16 un16;
  uni_8x4_32 un32;
  uni_8x8_64 un64;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(Serial2.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial2.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte == 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte == 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID) ||
                       (hedgehog_data_id == NT_POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }

            case NT_POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= true;
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm 
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);

              hedgehog_address= hedgehog_serial_buf[16];
              un16.b[0]= hedgehog_serial_buf[17];
              un16.b[1]= hedgehog_serial_buf[18]&0x0f;
              hedgehog_paired_heading= un16.w;
              
              hedgehog_pos_updated= true;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            case NT_POSITION_DATAGRAM_HIGHRES_ID:
            {
              byte ofs;
              byte ofs_vel;
              if (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID) {
                un32.b[0]= hedgehog_serial_buf[5];
                un32.b[1]= hedgehog_serial_buf[6];
                un32.b[2]= hedgehog_serial_buf[7];
                un32.b[3]= hedgehog_serial_buf[8];
                hedgehog_pos_timestamp= un32.vi32;

                ofs= 9;
                ofs_vel=27;
              } else {
                un64.b[0]= hedgehog_serial_buf[5];
                un64.b[1]= hedgehog_serial_buf[6];
                un64.b[2]= hedgehog_serial_buf[7];
                un64.b[3]= hedgehog_serial_buf[8];
                un64.b[4]= hedgehog_serial_buf[9];
                un64.b[5]= hedgehog_serial_buf[10];
                un64.b[6]= hedgehog_serial_buf[11];
                un64.b[7]= hedgehog_serial_buf[12];
                hedgehog_pos_timestamp= un64.vi64;

                ofs= 13;
                ofs_vel=31;
              }
              
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[ofs+0];
              un32.b[1]= hedgehog_serial_buf[ofs+1];
              un32.b[2]= hedgehog_serial_buf[ofs+2];
              un32.b[3]= hedgehog_serial_buf[ofs+3];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[ofs+4];
              un32.b[1]= hedgehog_serial_buf[ofs+5];
              un32.b[2]= hedgehog_serial_buf[ofs+6];
              un32.b[3]= hedgehog_serial_buf[ofs+7];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[ofs+8];
              un32.b[1]= hedgehog_serial_buf[ofs+9];
              un32.b[2]= hedgehog_serial_buf[ofs+10];
              un32.b[3]= hedgehog_serial_buf[ofs+11];
              hedgehog_z= un32.vi32;

              hedgehog_address= hedgehog_serial_buf[ofs+13];
              un16.b[0]= hedgehog_serial_buf[ofs+14];
              un16.b[1]= hedgehog_serial_buf[ofs+15]&0x0f;
              hedgehog_paired_heading= un16.w;
              
              hedgehog_pos_updated= true;// flag of new data from hedgehog received
              high_resolution_mode= true;

              hedgehog_vel_updated= false; // flag of new data from hedgehog received

              byte subCmd= hedgehog_serial_buf[ofs_vel++];
              switch(subCmd) {
                  case 1: {
                      //only streamed if stream speed selected in interfaces menu of beacon
                      hedgehog_vel_updated= true; // flag of new data from hedgehog received

                      // vel of hedgehog (X,Y), mm/s 
                      un16.b[0]= hedgehog_serial_buf[ofs_vel+0];
                      un16.b[1]= hedgehog_serial_buf[ofs_vel+1];
                      hedgehog_xvel= long(un16.wi);

                      un16.b[0]= hedgehog_serial_buf[ofs_vel+2];
                      un16.b[1]= hedgehog_serial_buf[ofs_vel+3];
                      hedgehog_yvel= long(un16.wi);
                      
                      // // vel of hedgehog, mm/s 
                      un16.b[0]= hedgehog_serial_buf[ofs_vel+4];
                      un16.b[1]= hedgehog_serial_buf[ofs_vel+5];
                      hedgehog_zvel= long(un16.wi);

                      break;
                  }

                  default: {
                      break;
                  }
                }
            }
          }
        } 
    }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}//