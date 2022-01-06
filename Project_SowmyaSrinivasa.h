#ifndef Project_SowmyaSrinivasa_H_
#define Project_SowmyaSrinivasa_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void initMsg();
void sendRS485(uint8_t dstAddr, uint8_t cmd, uint8_t channel, uint8_t size,
               uint8_t data1[], bool ack);
void sendUImsg();
void send485Byte(void);
int CalculateRetransmitTime(void);


#endif
