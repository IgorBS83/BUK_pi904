#include <stm32f4xx.h>
void ALLBUS_send(unsigned char, unsigned char, unsigned short, unsigned char*, unsigned char*);
int ALLBUS_recieve(unsigned char*, unsigned char, unsigned char, int, int);
unsigned char Compare_counters(unsigned short, unsigned short, int, int);
