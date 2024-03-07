#ifndef _ringbuffer_h
#define _ringbuffer_h
/* Philip Thrasher's Crazy Awesome Ring Buffer Macros!
 *
 * Below you will find some naughty macros for easy owning and manipulating
 * generic ring buffers. Yes, they are slightly evil in readability, but they
 * are really fast, and they work great.
 *
 * Example usage:
 *
 * #include <stdio.h>
 *
 * // So we can use this in any method, this gives us a typedef
 * // named 'intBuffer'.
 * ringBuffer_typedef(int, intBuffer);
 *
 * int main() {
 *   // Declare vars.
 *   intBuffer myBuffer;
 *
 *#ifdef USE_STATIC_MEMORY
 *   bufferInit(myBuffer,1024,int);
 *#else
	 static char bufMem[neededMemBytes(myBuffer, 1024)];
	 bufferInit(myBuffer,1024,int, bufMem);
 *#endif
 *   // We must have the pointer. All of the macros deal with the pointer.
 *   // (except for init.)
 *   intBuffer* myBuffer_ptr;
 *   myBuffer_ptr = &myBuffer;
 *
 *   // Write two values.
 *   bufferWrite(myBuffer_ptr,37);
 *   bufferWrite(myBuffer_ptr,72);
 *
 *   // Read a value into a local variable.
 *   int first;
 *   bufferRead(myBuffer_ptr,first);
 *   assert(first == 37); // true
 *
 *   int second;
 *   bufferRead(myBuffer_ptr,second);
 *   assert(second == 72); // true
 *
 *   return 0;
 * }
 *
 *
 *
 */

//example usafe
//#define SIM800_RX_RBUF_ITEM_TYPE char
//ringBuffer_typedef(SIM800_RX_RBUF_ITEM_TYPE, sim800RingBuf_t);
//
//sim800RingBuf_t  rx_rbuf;
//sim800RingBuf_t* pRx = &rx_rbuf; //used by macros
//SIM800_RX_RBUF_ITEM_TYPE rx_rbuf_mem[drv_rbuf_neededMemBytes(SIM800_RX_RBUF_ITEMS, SIM800_RX_RBUF_ITEM_TYPE)];
//
//drv_rbuf_init(pRx), SIM800_RX_RBUF_ITEMS, SIM800_RX_RBUF_ITEM_TYPE, (rx_rbuf_mem));
//drv_rbuf_write(pRx, char_item_var);
//drv_rbuf_read(pRx, &char_item_var));
//drv_rbuf_peekReadPtrWithOffset(pRx, pRead, i); //peek also previous char
//drv_rbuf_peekReadPtr(pRx, pRead);
//drv_rbuf_elements(pTx_buf, elemens); //NO PTR to elemens!

//#include "../lineReceiver/drv_lineReceiver.h"

#define ringBuffer_typedef(T, NAME) \
  typedef struct { \
    int size; \
    int start; \
    int end; \
    T* elems; \
  } NAME

//Get needed (static) memSize (useful when declaring static memory array)
//e.g. TYPE rx_rbuf_mem[drv_rbuf_neededMemBytes(ITEM_COUNT, TYPE)];
#define drv_rbuf_neededMemBytes(S,T) \
((S)*sizeof(T))

//BUF = pointer. All of the macros deal with the pointer.
// S = Size of buffer
// T = Type of buffer elements
// pM = pointer to allocated memory of type T
#define drv_rbuf_init(BUF, S, T, pM) \
   do { \
	  (BUF)->size = (S); \
	  (BUF)->start = 0;  /*aka read pointer*/ \
	  (BUF)->end = 0;    /*aka write pointer*/ \
	  (BUF)->elems = (T*) (pM);\
  } while(0)

#define nextStartIndex(BUF) (((BUF)->start + 1) % (BUF)->size)
#define nextEndIndex(BUF) (((BUF)->end + 1) % (BUF)->size)
#define isBufferEmpty(BUF) ((BUF)->end == (BUF)->start)
#define isBufferFull(BUF) (nextEndIndex(BUF) == (BUF)->start)
#define locStartIndex(BUF, LOC) (((BUF)->start + LOC) % (BUF)->size)

//Reset ringbuffer
#define drv_rbuf_reset(BUF) \
   do { \
	(BUF)->start = 0; \
	(BUF)->end = 0;\
	} while(0)

//Write element of type <T> into ringbuffer then advance write position +1
#define drv_rbuf_write(BUF, ELEM) \
    do { \
    	(BUF)->elems[(BUF)->end] = ELEM; \
    	(BUF)->end = ((BUF)->end + 1) % (BUF)->size; \
    	if (isBufferEmpty(BUF)) { \
    		(BUF)->start = nextStartIndex(BUF); \
    	}\
	} while(0)

//Get pointer of type <*T> from current write position then advance write position +1
#define drv_rbuf_getWritePtr(BUF, pELEM) \
    do { \
	pELEM = &((BUF)->elems[(BUF)->end]); \
	(BUF)->end = ((BUF)->end + 1) % (BUF)->size; \
	if (isBufferEmpty(BUF)) { \
		(BUF)->start = nextStartIndex(BUF); \
	}\
	} while(0)

//Get pointer of type <*T> from current write position
#define drv_rbuf_peekWritePtr(BUF, pELEM) \
   pELEM = &((BUF)->elems[(BUF)->end]);

//Advance write position +1
#define drv_rbuf_moveWritePos(BUF) \
  	do { \
  		(BUF)->end = ((BUF)->end + 1) % (BUF)->size; \
  		if (isBufferEmpty(BUF)) { \
  			(BUF)->start = nextStartIndex(BUF); \
  		}\
	} while(0)

//Read element of type <T> from ringbuffer then advance read position +1
#define drv_rbuf_read(BUF, pELEM) \
	do { \
		*pELEM = (BUF)->elems[(BUF)->start]; \
		(BUF)->start = nextStartIndex(BUF);\
	} while(0)

//Get pointer of type <*T> from current read position then advance read position +1
#define drv_rbuf_getReadPtr(BUF, pELEM) \
	do { \
	pELEM = &((BUF)->elems[(BUF)->start]); \
    (BUF)->start = nextStartIndex(BUF);\
    } while(0)

//Get pointer of type <*T> from current read position but don't advance read pointer!
#define drv_rbuf_peekReadPtr(BUF, pELEM) \
    pELEM = &((BUF)->elems[(BUF)->start]);

#define drv_rbuf_peekReadPtrWithOffset(BUF, pELEM, offset) \
    pELEM = &((BUF)->elems[locStartIndex(BUF, offset)]);

/*
//Get pointer of type <*T> from current read position + LOC
//can be iterated in a loop LOC[0...ELEMENT_COUNT] to count elements
//#define peekLocBufferReadPointer(BUF, pELEM, LOC) \
//		do { \
//			if(locStartIndex(BUF,LOC) == (BUF)->end) { \
//			pELEM = NULL; }\
//			else{\
//			pELEM = &((BUF)->elems[locStartIndex(BUF,LOC)]);\
//			}\
//		} while(0)
*/
#define drv_rbuf_elements(BUF, pNUM) \
		do { \
			(*(pNUM)) = 0;\
			while( locStartIndex(BUF, (*(pNUM))) != (BUF)->end) { \
				(*(pNUM))++;\
			}\
		} while(0)

//Standard Ringbuffer Types
ringBuffer_typedef(char, charRingBuf_t);


#endif


