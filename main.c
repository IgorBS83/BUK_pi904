#include <stm32f4xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "allbus.h"

#define true 1
#define false 0
	
#define PLL_M 24
#define PLL_N 336
#define PLL_P 2
#define PLL_Q 4

#define mGPIO_AF_FSMC0  0x0000000C
#define mGPIO_AF_FSMC1  0x000000C0
#define mGPIO_AF_FSMC2  0x00000C00
#define mGPIO_AF_FSMC3  0x0000C000
#define mGPIO_AF_FSMC4  0x000C0000
#define mGPIO_AF_FSMC5  0x00C00000
#define mGPIO_AF_FSMC6  0x0C000000
#define mGPIO_AF_FSMC7  0xC0000000
#define mGPIO_AF_FSMC8  0x0000000C
#define mGPIO_AF_FSMC9  0x000000C0
#define mGPIO_AF_FSMC10  0x00000C00
#define mGPIO_AF_FSMC11  0x0000C000
#define mGPIO_AF_FSMC12  0x000C0000
#define mGPIO_AF_FSMC13  0x00C00000
#define mGPIO_AF_FSMC14  0x0C000000
#define mGPIO_AF_FSMC15  0xC0000000

#define ALTERA_BASE      0x60000000

#define OU_ADDR 				1
#define PROTOCOL_SUADDR 1
#define MKIO_in_SIZE   7
#define MKIO_out_SIZE   8

uint32_t fl_mkio_new_in_data;
uint16_t *mkio_data_altera = (uint16_t*)(ALTERA_BASE + (PROTOCOL_SUADDR << (5 + 1)));
uint16_t *sensors_altera = (uint16_t*)(ALTERA_BASE + (0x410 << 1));
uint16_t mkio_data_iram[8], sensors_iram[8]; 

uint16_t SPI_RX[20], SPI_TX[20];
int fl_spi_busy_delay, fl_spi_transfer_done;
int fl_pfdz_ready, fl_biss_ready;

uint32_t corr_ugn = 0;//203571;
uint32_t corr_uvn = 0;//44877;
uint32_t ugn, uvn;

typedef struct
{
	uint32_t data_size: 5;
	uint32_t sub_addr: 5;
	uint32_t k: 1;
	uint32_t addr_ou: 5;
}MKIO_CONTROL_WORD_type;

MKIO_CONTROL_WORD_type MKIO_CONTROL_WORD;
uint32_t* MKIO_CONTROL_PATTERN = (uint32_t*)&MKIO_CONTROL_WORD;

void mPLL_init(void)
{
	int StartUpCounter = 0;
	
	RCC->CR |= RCC_CR_HSEON;
	while(((RCC->CR & RCC_CR_HSERDY) == 0) || (++StartUpCounter < 1000)){;} 
		
/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

		    /* HCLK = SYSCLK / 1*/
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | 
								RCC_CFGR_PPRE2_DIV2 | 
								RCC_CFGR_PPRE1_DIV4;
		
	RCC->PLLCFGR = 	RCC_PLLCFGR_PLLSRC_HSE	|//HSE oscillator clock selected as PLL
									(PLL_Q << 24) |//PLLQ
									(((PLL_P >> 1) -1) << 16) |//PLLP
									(PLL_N << 6) |//PLLN
									PLL_M;//PLLM
		
	RCC->CR |= RCC_CR_PLLON;
		
	while((RCC->CR & RCC_CR_PLLRDY) == 0){;} 	
		
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
		
    /* Select the main PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;	
		
	while((RCC->CFGR & RCC_CFGR_SWS_1) == 0){;} 	
}

void mTimer_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 &= ~TIM_CR1_CEN;
//	NVIC_EnableIRQ(TIM4_IRQn);
	

//	TIM4->CNT = 0;
	TIM4->PSC = 84 - 1;		//84MHz -> 1MHz
	TIM4->ARR = 100 - 1; //1MHz  -> 10KHz			
	//TIM4->ARR = 200 - 1; //1MHz  -> 5KHz
	//TIM4->ARR = 500 - 1; //1MHz  -> 2KHz
	//TIM4->ARR = 1000 - 1; //1MHz  -> 1KHz			
  TIM4->DIER = TIM_DIER_UIE;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;// | TIM_CR1_OPM;  
	TIM4->CR1 |= TIM_CR1_CEN;
//	TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;// | TIM_CR1_OPM;  
	NVIC_EnableIRQ(TIM4_IRQn);
}



void mDMA_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	DMA2_Stream0->CR = 
			(3 << 25) | //Channel 3
			(2 << 16) | //Priority level High
			(1 << 13) | //Memory data size half-word (16-bit)
			(1 << 11) | //Peripheral data size half-word (16-bit)
			(1 << 10) | //Memory address pointer is incremented after each data transfer
			(1 << 5);		//The peripheral is the flow controller
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);
	DMA2_Stream0->M0AR = (uint32_t)&SPI_RX;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	
	DMA2_Stream3->CR = 
			(3 << 25) | //Channel 3
			(3 << 16) | //Priority level Very High
			(1 << 13) | //Memory data size half-word (16-bit)
			(1 << 11) | //Peripheral data size half-word (16-bit)
			(1 << 10) | //Memory address pointer is incremented after each data transfer
			(1 << 6)  | //Memory-to-peripheral
			(1 << 5);		//The peripheral is the flow controller
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);
	DMA2_Stream3->M0AR = (uint32_t)&SPI_TX;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
}


void mEXTI_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0; //output
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	//EXTI enable
	EXTI->FTSR = EXTI_FTSR_TR0 | EXTI_FTSR_TR2 | EXTI_FTSR_TR3;
	EXTI->IMR = EXTI_IMR_MR0 | EXTI_IMR_MR2 | EXTI_IMR_MR3;
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI2_PC | SYSCFG_EXTICR1_EXTI3_PC;
	EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR2 | EXTI_PR_PR3;

	EXTI->RTSR = 0;
	EXTI->RTSR |= EXTI_FTSR_TR4;
	EXTI->IMR |= EXTI_IMR_MR4;
	SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI4_PA;
	EXTI->PR |= EXTI_PR_PR4;

	NVIC_EnableIRQ(EXTI0_IRQn);	
	NVIC_EnableIRQ(EXTI2_IRQn);	
	NVIC_EnableIRQ(EXTI3_IRQn);	
	NVIC_EnableIRQ(EXTI4_IRQn);	
}

void mSPI_init(void)
{
//PA4 - spi2 nss, PA5 - spi2 clk, PA6 - spi2 miso, PA7 - spi2 mosi 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
//	GPIOA->MODER &= ~(1 << 4);
	
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->AFR[0] |= (5 << (4*4)) | (5 << (5*4)) | (5 << (6*4)) | (5 << (7*4));

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = 0;
	SPI1->CR1 = 
		SPI_CR1_DFF;	//16-bit data frame format

//	SPI1->CR2 = 
//		SPI_CR2_RXDMAEN | //Rx Buffer DMA Enable
//		SPI_CR2_TXDMAEN;	//Tx buffer DMA enabled

	SPI1->CR2 = 
		SPI_CR2_RXNEIE | //Rx Buffer DMA Enable
		SPI_CR2_TXEIE;	//Tx buffer DMA enabled
	NVIC_EnableIRQ(SPI1_IRQn);	

	
	SPI1->CR1 |= SPI_CR1_SPE;//SPI Enable
	SPI1->DR = 0;
}
void mFSMC_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN 
							| RCC_AHB1ENR_GPIOEEN 
							| RCC_AHB1ENR_GPIOFEN 
							| RCC_AHB1ENR_GPIOGEN;
	
	GPIOD->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1	
							|	GPIO_MODER_MODER5_1
							|	GPIO_MODER_MODER7_1	
							|	GPIO_MODER_MODER8_1	
							|	GPIO_MODER_MODER9_1	
							|	GPIO_MODER_MODER10_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOD->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4	
							|	GPIO_OSPEEDER_OSPEEDR5
							|	GPIO_OSPEEDER_OSPEEDR7	
							|	GPIO_OSPEEDER_OSPEEDR8	
							|	GPIO_OSPEEDER_OSPEEDR9	
							|	GPIO_OSPEEDER_OSPEEDR10
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOD->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC3
							| mGPIO_AF_FSMC4
							| mGPIO_AF_FSMC5
							| mGPIO_AF_FSMC7;
	GPIOD->AFR[1] = mGPIO_AF_FSMC8 
							| mGPIO_AF_FSMC9 
							| mGPIO_AF_FSMC10
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOE->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER7_1	
							|	GPIO_MODER_MODER8_1	
							|	GPIO_MODER_MODER9_1
							|	GPIO_MODER_MODER10_1	
							|	GPIO_MODER_MODER11_1	
							|	GPIO_MODER_MODER12_1	
							|	GPIO_MODER_MODER13_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOE->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR7	
							|	GPIO_OSPEEDER_OSPEEDR8	
							|	GPIO_OSPEEDER_OSPEEDR9
							|	GPIO_OSPEEDER_OSPEEDR10	
							|	GPIO_OSPEEDER_OSPEEDR11	
							|	GPIO_OSPEEDER_OSPEEDR12	
							|	GPIO_OSPEEDER_OSPEEDR13
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOE->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC7;
	GPIOE->AFR[1] = mGPIO_AF_FSMC8 
							| mGPIO_AF_FSMC9 
							| mGPIO_AF_FSMC10 
							| mGPIO_AF_FSMC11
							| mGPIO_AF_FSMC12
							| mGPIO_AF_FSMC13
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOF->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER2_1	
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1
							|	GPIO_MODER_MODER5_1	
							|	GPIO_MODER_MODER12_1	
							|	GPIO_MODER_MODER13_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOF->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR2	
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4
							|	GPIO_OSPEEDER_OSPEEDR5	
							|	GPIO_OSPEEDER_OSPEEDR12	
							|	GPIO_OSPEEDER_OSPEEDR13
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOF->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC2 
							| mGPIO_AF_FSMC3 
							| mGPIO_AF_FSMC4
							| mGPIO_AF_FSMC5;
	GPIOF->AFR[1] = mGPIO_AF_FSMC12
							| mGPIO_AF_FSMC13
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOG->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER2_1	
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1;
							
	GPIOG->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR2	
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4;
								
	GPIOG->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC2 
							| mGPIO_AF_FSMC3 
							| mGPIO_AF_FSMC4;

	RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
	FSMC_Bank1->BTCR[0] = 0x00003011;//0x00003011;
	FSMC_Bank1->BTCR[1] = 0x021408F2;//0x021408F2;
	FSMC_Bank1E->BWTR[0] = 0x021F08F2;//0x021F08F2;
}



void EXTI0_IRQHandler()//mkio data
{
	if(EXTI->PR & EXTI_PR_PR0) 
	{
		EXTI->PR |= EXTI_PR_PR0;
		if(*(uint16_t*)(ALTERA_BASE + (0x400 << 1)) == *MKIO_CONTROL_PATTERN)
			fl_mkio_new_in_data = true;
	}	
}

void EXTI2_IRQHandler()//pfdz ready
{
	if(EXTI->PR & EXTI_PR_PR2) 
	{
		EXTI->PR |= EXTI_PR_PR2;
		fl_pfdz_ready = true;
	}	
}

void EXTI3_IRQHandler()//biss ready
{
	if(EXTI->PR & EXTI_PR_PR3) 
	{
		EXTI->PR |= EXTI_PR_PR3;
		fl_biss_ready = true;
	}	
}
int fl_spi_front_fall;
void EXTI4_IRQHandler()//biss ready
{
	if(EXTI->PR & EXTI_PR_PR4) 
	{
		EXTI->PR |= EXTI_PR_PR4;
		fl_spi_front_fall = true;
	}	
}

int spi_rx_cnt, spi_tx_cnt;
void SPI1_IRQHandler()//biss ready
{
	if(SPI1->SR & SPI_SR_RXNE) 
		SPI_RX[spi_rx_cnt++] = SPI1->DR;
	if(SPI1->SR & SPI_SR_TXE)
		SPI1->DR = SPI_TX[spi_tx_cnt++];
}
int a = 0;

void TIM4_IRQHandler()//timer for ask sensors 
{
	*(uint16_t*)(ALTERA_BASE + (0x419 << 1)) = 3;//launch sensors
	TIM4->SR = 0;
	a++;
}
				
int main()
{	
	int i;
	mPLL_init();
	mFSMC_init();
	mEXTI_init();
	mSPI_init();
//	mDMA_init();
	mTimer_init();

	MKIO_CONTROL_WORD.addr_ou = OU_ADDR;
	MKIO_CONTROL_WORD.sub_addr = PROTOCOL_SUADDR;
	MKIO_CONTROL_WORD.data_size = MKIO_in_SIZE;

//	*(uint16_t*)(ALTERA_BASE + (0x418 << 1)) = 1;//ask sensors themselves with period 1ms
	*(uint16_t*)(ALTERA_BASE + (0x418 << 1)) = 0;//ask sensors with enquire
	*(uint16_t*)(ALTERA_BASE + (0x419 << 1)) = 3;//launch sensors
	while(!fl_biss_ready);
	*(uint16_t*)(ALTERA_BASE + (0x420 << 1)) = 7;//switch on diodes

	while(1)
	{
		if(fl_mkio_new_in_data)
		{
			fl_mkio_new_in_data = false;
			mkio_data_iram[0] = 1;//new message flag
			for(i = 0; i < MKIO_in_SIZE; i++) mkio_data_iram[i + 1] = mkio_data_altera[i];
		}
		if(fl_pfdz_ready)
		{
			fl_pfdz_ready = false;
			for(i = 0; i < 4; i++) sensors_iram[i] = sensors_altera[i];
			ugn = (((uint32_t)sensors_iram[0]) << 16) | ((uint32_t)sensors_iram[1]);
			uvn = (((uint32_t)sensors_iram[2]) << 16) | ((uint32_t)sensors_iram[3]);
			ugn &= 0x3FFFF;
			uvn &= 0x3FFFF;
			ugn = ugn - corr_ugn;   //offset null   saa
			uvn = uvn - corr_uvn;   //offset null   saa
			ugn &= 0x3FFFF;         //mask on 18 bit  saa
			uvn &= 0x3FFFF;         //mask on 18 bit  saa
			ugn = ugn >> 2;         //transform       saa
			if(uvn < 0x364b8) uvn = (uvn * 67 / 960);   //transform saa
			else uvn = (0xFFFF-((0x3FFFF-uvn)*67/960));     //transform saa
			mkio_data_altera[1] = (uint16_t)ugn;
			mkio_data_altera[2] = (uint16_t)uvn;
		}
		if(fl_biss_ready)
		{
			fl_biss_ready = false;
			for(i = 4; i < 8; i++) sensors_iram[i] = sensors_altera[i];//bilo "<7"!!!!!!!!!!!!!!
		}
		if(spi_rx_cnt >= 20) {
			spi_rx_cnt= 0;
			if(SPI_RX[0] == 1) //test new messge bit 
			{
//MKV
				// saa 23112016
//				ugn = ugn - corr_ugn;   //offset null   saa
//				uvn = uvn - corr_uvn;   //offset null   saa
//				ugn &= 0x3FFFF;         //mask on 18 bit  saa
//				uvn &= 0x3FFFF;         //mask on 18 bit  saa
//				ugn = ugn >> 2;         //transform       saa
//				if(uvn < 0x364b8) uvn = (uvn * 67 / 960);   //transform saa
//				else uvn = (0xFFFF-((0x3FFFF-uvn)*67/960));     //transform saa
				mkio_data_altera[0] = SPI_RX[1];//translating protocol data from k3250 to mkio
//				mkio_data_altera[1] = (uint16_t)ugn;
//				mkio_data_altera[2] = (uint16_t)uvn;
				//end of saa 23112016
//MKV
				for(i = 3; i < MKIO_out_SIZE; i++) 	mkio_data_altera[i] = SPI_RX[i + 1];//translating protocol data from k3250 to mkio
			}
			if(SPI_RX[0] == 2)
			{
				corr_ugn = (((uint32_t)SPI_RX[2]) << 16) | ((uint32_t)SPI_RX[3]);
				corr_uvn = (((uint32_t)SPI_RX[4]) << 16) | ((uint32_t)SPI_RX[5]);
				corr_ugn &= 0x3FFFF;
				corr_uvn &= 0x3FFFF;
			}
			SPI_RX[0] = 0;			
		}
		if(spi_tx_cnt >= 20) {
			spi_tx_cnt= 0;
			//*(uint16_t*)(ALTERA_BASE + (0x419 << 1)) = 3;//launch sensors
			for(i = 0; i < 8; i++) //preparing data to k3250
			{
				SPI_TX[i] = sensors_iram[i];		
				SPI_TX[i + 8] = mkio_data_iram[i];		
			}
			mkio_data_iram[0] = 0;	
		}
		if(fl_spi_front_fall) 
		{
			fl_spi_front_fall = false;
			spi_rx_cnt = 0; spi_tx_cnt = 0;
		}
	}
	return 0;
}
