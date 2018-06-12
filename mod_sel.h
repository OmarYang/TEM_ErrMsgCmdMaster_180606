#ifndef MOD_SEL_H_
#define MOD_SEL_H_

#define MT_MOD5270B		1		//module type
#define MT_MOD5282		2
#define MODULE_TYPE 	MT_MOD5270B
//#define MODULE_TYPE 	MT_MOD5282
//
#if MODULE_TYPE == MT_MOD5270B
#include <..\mod5270\system\sim5270.h>
#define SYSCLK			73728000
#define MODULE_NAME		"MOD5270B"
#define PITR_CH_OFS		36
#define PIT_VECTOR		0x2600
//
/*
#if MODULE_TYPE == MT_MOD5272
#include <..\mod5272\system\sim5272.h>
#define SYSCLK	62500000
#define MODULE_NAME		"MOD5272"
#define PITR_CH_OFS		36
#define PIT_VECTOR		0x2600
*/
//
#elif MODULE_TYPE == MT_MOD5282
#include <..\mod5282\system\sim5282.h>
#define SYSCLK			73728000
#define MODULE_NAME		"MOD5282"
#define PITR_CH_OFS		55
#define PIT_VECTOR		0x2200
#endif

#endif /*MOD_SEL_H_*/
