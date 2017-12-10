#ifndef ringbufs_h 
#define ringbufs_h
 
#include <stdio.h>
#include <string.h>
typedef struct  ringBufS {
	unsigned char *buf;
	int size;
	int head;
	int tail;
	int count;
} ringBufS;

void  ringBufS_init  (ringBufS *_this, int size);
int   ringBufS_empty (ringBufS *_this);
int   ringBufS_full  (ringBufS *_this);
int   ringBufS_get   (ringBufS *_this);
void  ringBufS_put   (ringBufS *_this, const unsigned char c);
void  ringBufS_flush (ringBufS *_this, const int clearBuffer);
#endif
