#include <stdio.h>
#include <string.h>
#include "ringBufS.h"

unsigned int modulo_inc (const unsigned int value, const unsigned int modulus) {
	unsigned int my_value = value + 1;
	if (my_value >= modulus) 	{
		my_value  = 0;
	}
	return (my_value);
}

unsigned int modulo_dec (const unsigned int value, const unsigned int modulus) {
	unsigned int my_value = (0==value) ? (modulus - 1) : (value - 1);
	return (my_value);
}

void ringBufS_init (ringBufS *_this, int size) {
	/*****
	  The following clears:
	  -> buf
	  -> head
	  -> tail
	  -> count
	  and sets head = tail
	 ***/
	memset (_this, 0, sizeof (*_this));
	_this->size = size;
	_this->buf = (unsigned char *)malloc(sizeof(unsigned char)*size);
}
void ringBufs_delete (ringBufS *_this){
	if (_this->buf) free(_this->buf);
	_this->size = 0;
}

int ringBufS_empty (ringBufS *_this) {
	return (0==_this->count);
}

int ringBufS_full (ringBufS *_this) {
	return (_this->count>=_this->size);
}

void ringBufS_put (ringBufS *_this, const unsigned char c) {
	if (_this->count < _this->size) {
		_this->buf[_this->head] = c;
		_this->head = modulo_inc (_this->head, _this->size);
		++_this->count;
	}
}

int ringBufS_get (ringBufS *_this) {
	int c;
	if (_this->count>0) {
		c  = _this->buf[_this->tail];
		_this->tail = modulo_inc (_this->tail, _this->size);
		--_this->count;
	}
	else
		c = -1;

	return (c);
}
