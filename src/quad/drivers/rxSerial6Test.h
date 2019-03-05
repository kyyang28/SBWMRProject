#ifndef __RXSERIAL6TEST_H
#define __RXSERIAL6TEST_H

void rxSerial6TestInit(void);
//void gpsSetPrintfSerialPort(void);
void rxSerial6TestWrite(uint8_t ch);
uint8_t btSerial6Read(void);
void rxSerial6TestPrint(const char *str);

#endif	// __RXSERIALTEST_H
