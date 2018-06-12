/*
 * hv_ctrl.h
 *
 *  Created on: 2014/4/4
 *      Author: Robert Lee
 */

#ifndef HV_CTRL_H_
#define HV_CTRL_H_
//
int HV_DecipherResponse(char *p);
void HV_GetSN(void);
void HV_SetSN(char *p);
void HV_SetAcc(float fVolt);
void HV_SetBias(float fVolt);
void HV_SetFila(float fVolt);
void HV_SetOn(int nEnable);

#endif /* HV_CTRL_H_ */
