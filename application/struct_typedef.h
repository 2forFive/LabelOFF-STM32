/**
  ******************************************************************************
  * @file       struct_typedef.h
	* @author			sxx
  * @brief      typedef of some common data types
  * @note       from DJI motor controlling demos
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-10-2021     1. done
	*
  ******************************************************************************
  */
	
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/* from C board demo struct_typedef.h */
typedef signed char 			int8_t;
typedef signed short int 	int16_t;
typedef signed int 				int32_t;
typedef signed long long 	int64_t;

typedef unsigned char 			uint8_t;
typedef unsigned short int 	uint16_t;
typedef unsigned int 				uint32_t;
typedef unsigned long long 	uint64_t;
typedef unsigned char 			bool_t;

typedef float 	fp32;
typedef double 	fp64;


/* from M2006 demo mytype.h */
typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32;

typedef volatile uint8_t 		vu8;
typedef volatile uint16_t 	vu16;
typedef volatile uint32_t 	vu32;

typedef volatile int8_t 	vs8;
typedef volatile int16_t 	vs16;
typedef volatile int32_t	vs32;

#endif



