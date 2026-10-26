#ifndef __BALANCE_DIAG_H
#define __BALANCE_DIAG_H

#include "main.h"

/*
 * 双脚平放时的前后重心诊断。
 * 仅改变ID8/ID10，不修改正式直立、踏步或前进算法。
 */
uint8_t BalanceDiag_Start(uint8_t pattern);
uint8_t BalanceDiag_Return(void);
void BalanceDiag_Cancel(void);
const char *BalanceDiag_GetLastError(void);

#endif
