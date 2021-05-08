/*
 * useful macros from HAL
 */

#ifndef __HALPER_H__
#define __HALPER_H__

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLM1__, __PLLN1__, __PLLP1__, __PLLQ1__,__PLLR1__ ) \
                  do{ MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1) , ((__RCC_PLLSOURCE__) | ( (__PLLM1__) <<4U)));  \
                      WRITE_REG (RCC->PLL1DIVR , ( (((__PLLN1__) - 1U )& RCC_PLL1DIVR_N1) | ((((__PLLP1__) -1U ) << 9U) & RCC_PLL1DIVR_P1) | \
                                ((((__PLLQ1__) -1U) << 16U)& RCC_PLL1DIVR_Q1) | ((((__PLLR1__) - 1U) << 24U)& RCC_PLL1DIVR_R1))); \
                    } while(0)

#endif
