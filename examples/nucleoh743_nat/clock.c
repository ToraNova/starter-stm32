#include "stm32h7xx.h"
#include "stm32h743xx.h"

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLM1__, __PLLN1__, __PLLP1__, __PLLQ1__,__PLLR1__ ) \
                  do{ MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1) , ((__RCC_PLLSOURCE__) | ( (__PLLM1__) <<4U)));  \
                      WRITE_REG (RCC->PLL1DIVR , ( (((__PLLN1__) - 1U )& RCC_PLL1DIVR_N1) | ((((__PLLP1__) -1U ) << 9U) & RCC_PLL1DIVR_P1) | \
                                ((((__PLLQ1__) -1U) << 16U)& RCC_PLL1DIVR_Q1) | ((((__PLLR1__) - 1U) << 24U)& RCC_PLL1DIVR_R1))); \
                    } while(0)

uint32_t sysclk_pll_hse_freq(void){
	uint32_t pllp, pllm, pllfracen;
	float fracn1, pllvco;
	pllm = ((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1)>> 4)  ;
	pllfracen = ((RCC-> PLLCFGR & RCC_PLLCFGR_PLL1FRACEN)>>RCC_PLLCFGR_PLL1FRACEN_Pos);
	fracn1 = (float)(uint32_t)(pllfracen* ((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1)>> 3));
        pllvco = ((float)HSE_VALUE / (float)pllm) * ((float)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1/(float)0x2000) +(float)1 );
	pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >>9) + 1U ) ;
	return (uint32_t)(float)(pllvco/(float)pllp);
}

void __sysclk_pll_hse_init(
		uint32_t apb3clkdiv,
		uint32_t apb1clkdiv,
		uint32_t apb2clkdiv,
		uint32_t apb4clkdiv,
		uint32_t ahbclkdiv,
		uint32_t sysclkdiv,
		uint32_t flaten
){
	uint32_t common_system_clock;

	/* Increasing the CPU frequency */
	if( flaten > (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))){
        	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)flaten);
		while( (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)) != flaten );
	}
#if defined (RCC_D1CFGR_D1PPRE)
	if( apb3clkdiv > (RCC->D1CFGR & RCC_D1CFGR_D1PPRE)){
		MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE, apb3clkdiv); }
#else
#error please refer to stm32h7xx_hal_rcc.c line 921
#endif

#if defined (RCC_D2CFGR_D2PPRE1)
	if( apb1clkdiv > (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1)){
		MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, apb1clkdiv);}
#else
#error please refer to stm32h7xx_hal_rcc.c line 939
#endif

#if defined(RCC_D2CFGR_D2PPRE2)
	if( apb2clkdiv > (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2)) {
		MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, apb2clkdiv); }
#else
#error please refer to stm32h7xx_hal_rcc.c line 956
#endif

#if defined(RCC_D3CFGR_D3PPRE)
	if( apb4clkdiv > (RCC->D3CFGR & RCC_D3CFGR_D3PPRE)) {
		MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE, apb4clkdiv ); }
#else
#error please refer to stm32h7xx_hal_rcc.c line 974
#endif

#if defined (RCC_D1CFGR_HPRE)
	if( ahbclkdiv > (RCC->D1CFGR & RCC_D1CFGR_HPRE)) {
		MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE, ahbclkdiv); }
#else
#error please refer to stm32h7xx_hal_rcc.c line 992
#endif

#if defined(RCC_D1CFGR_D1CPRE)
	MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1CPRE, sysclkdiv);
#else
#error wip
#endif

	while( !(RCC->CR & RCC_CR_PLLRDY) ); //wait for PLL to be re-enabled
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL1); //PLL is clk source
	while( ((uint32_t)(RCC->CFGR & RCC_CFGR_SWS)) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos));

#if defined(RCC_D1CFGR_HPRE)
	if( ahbclkdiv < (RCC->D1CFGR & RCC_D1CFGR_HPRE)) {
		MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE, ahbclkdiv); }
#else
#error please refer to stm32h7xx_hal_rcc.c line 1014
#endif

	if( flaten < (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))){
		MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)flaten);
		while( (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)) != flaten );
	}

#if defined(RCC_D1CFGR_D1PPRE)
	if( apb3clkdiv < (RCC->D1CFGR & RCC_D1CFGR_D1PPRE)) {
		MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE, apb3clkdiv); }
#else
#endif

#if defined(RCC_D2CFGR_D2PPRE1)
	if( apb1clkdiv < (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1)) {
		MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, apb1clkdiv); }
#else
#error wip
#endif

#if defined (RCC_D2CFGR_D2PPRE2)
	if( apb2clkdiv < (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2)) {
		MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, apb2clkdiv); }
#else
#error wip
#endif

#if defined(RCC_D3CFGR_D3PPRE)
	if(apb4clkdiv < (RCC->D3CFGR & RCC_D3CFGR_D3PPRE)) {
		MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE, apb4clkdiv ); }
#else
#error wip
#endif

#if defined(RCC_D1CFGR_D1CPRE)
	common_system_clock = sysclk_pll_hse_freq() >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE)>> RCC_D1CFGR_D1CPRE_Pos]) & 0x1FU);
#else
#error wip
#endif

#if defined(RCC_D1CFGR_HPRE)
	SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)>> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
#else
	SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->CDCFGR1 & RCC_CDCFGR1_HPRE)>> RCC_CDCFGR1_HPRE_Pos]) & 0x1FU));
#endif

#if defined(DUAL_CORE) && defined(CORE_CM4)
	SystemCoreClock = SystemD2Clock;
#else
	SystemCoreClock = common_system_clock;
#endif /* DUAL_CORE && CORE_CM4 */
/* Update the SystemCoreClock global variable */
/* Configure the source of time base considering new system clocks settings*/
//halstatus = HAL_InitTick (uwTickPrio);
}

void sysclk_init(void){
	// RCC initialization according to tinyusb stm32h7nucleo board example
	// using high speed external oscillator (8MHz)
	RCC->CR |= RCC_CR_HSEON;
	while( !(RCC->CR & RCC_CR_HSERDY) ); //wait for HSE to be ready
	while( (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL1 );//ensure PLL1 is not sysclock
	CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON); //disable PLL1
	while( (RCC->CR & RCC_CR_PLLRDY) ); //wait for PLL to be disabled
	__HAL_RCC_PLL_CONFIG( RCC_CFGR_SW_HSE, HSE_VALUE/1000000, 336, 2, 7, 2);
	CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN); //stop frac
	MODIFY_REG(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACN1, 0 << RCC_PLL1FRACR_FRACN1_Pos);
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1RGE, RCC_PLLCFGR_PLL1RGE_0);
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1VCOSEL, RCC_PLLCFGR_PLL1VCOSEL);
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN);
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN);
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR1EN);
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN); //enable  frac
	SET_BIT(RCC->CR, RCC_CR_PLL1ON); //re-enable PLL
	while( !(RCC->CR & RCC_CR_PLLRDY) ); //wait for PLL to be re-enabled

	__sysclk_pll_hse_init(
		RCC_D1CFGR_D1PPRE_DIV2,
		RCC_D2CFGR_D2PPRE1_DIV2,
		RCC_D2CFGR_D2PPRE2_DIV2,
		RCC_D3CFGR_D3PPRE_DIV2,
		RCC_D1CFGR_HPRE_DIV1, //HCLK_DIV1
		RCC_D1CFGR_D1CPRE_DIV1, //SYSCLK_DIV1
		FLASH_ACR_LATENCY_4WS
	);
}
