#ifndef _PACKET_H_
#define _PACKET_H_
#include <Arduino.h>

#define PacketHead (char)0xff
#define Rate 115200
enum
{
    RevHead = 0,
    RevData = 1
};

class my_serial
{
public:
    int rev_status = RevHead;
    char rev_tpm;
    uint8_t command;

    my_serial();
    float rev_data();
    void send_data(uint8_t type);
};

my_serial::my_serial()
{
    command = 0;
    rev_status = RevHead;
    Serial.begin(Rate);
}

void my_serial::rev_data()
{
    if (Serial.available() > 0)
    {
        rev_tpm = (char)Serial.read();
          if (rev_status == RevHead)
          {
              if (rev_tpm == PacketHead)
              {
                  rev_status = RevData;
                  is_open = 0;
              }
          }
          else if (rev_status == RevData) //数据
          {
              command = *(uint8_t *)rev_tpm;
              rev_status = RevHead;
          }
    }
  
}


// void my_serial::send_data(uint8_t type)
// {
//     uint8_t send_buf[3];
//     int send_buf_index = 0;
//     send_buf[send_buf_index++] = PacketHead;
//     send_buf[send_buf_index++] = type;
//     send_buf[send_buf_index++] = PacketRear;
//     int i;
//     for(i=0;i<3;i++)
//     {
//         Serial.write(send_buf[i]);
//     }
// }

#endif
