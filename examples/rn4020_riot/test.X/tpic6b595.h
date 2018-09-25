/* 
 * File:   tpic6b595.h
 * Author: root
 *
 * Created on September 18, 2018, 2:56 PM
 */

#ifndef TPIC6B595_H
#define	TPIC6B595_H

#ifdef	__cplusplus
extern "C" {
#endif

#define TPIC6B595_TEST_DATA	0b01010101
	
int tpic6b595_init(void);
int tpic6b595_testing(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TPIC6B595_H */

