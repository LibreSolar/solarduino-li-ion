
#ifndef HELPER_H
#define HELPER_H

void format_double(double val, byte precision, char *buf, unsigned bufLen = 0xffff);
unsigned format_unsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, byte width = 0);

const char *byte2bin(int x);

long read_vcc();


#endif // HELPER_H
