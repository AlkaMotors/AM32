/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"
#include "signal.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 4);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;
extern void processDshot(void);

#ifdef USE_TIMER_3_CHANNEL_1
#define Input_DMA_Ch  DMA1_Channel4
#endif
#ifdef USE_TIMER_15_CHANNEL_1  
#define Input_DMA_Ch  DMA1_Channel5
#endif


void runDshotCheck(){ 
     if((Input_DMA_Ch->CNDTR) < 63){
      if(armed){
        if ((IC_TIMER_REGISTER->CNT - dma_buffer[63-Input_DMA_Ch->CNDTR]) > (valid_packet_high<<1)){
          if(Input_DMA_Ch->CNDTR <= 32){
            DMA_start_bit = 32-Input_DMA_Ch->CNDTR;
            transfercomplete();
            EXTI->SWIER1 |= LL_EXTI_LINE_15;
          }
        
        LL_DMA_DisableChannel(DMA1, INPUT_DMA_CHANNEL);
        Input_DMA_Ch->CNDTR = 64;
        LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
        IC_TIMER_REGISTER->CNT = 0;
        DMA_start_bit = 0;
        }

    }else{
      if(Input_DMA_Ch->CNDTR <= 32){
      transfercomplete();
      processDshot();
 //     EXINT->swtrg = EXINT_LINE_15;
        LL_DMA_DisableChannel(DMA1, INPUT_DMA_CHANNEL);
        Input_DMA_Ch->CNDTR = 64;
        LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
        IC_TIMER_REGISTER->CNT = 0;
      }
    }
    } 
  
}



void receiveDshotDma()
{
    out_put = 0;
#ifdef USE_TIMER_3_CHANNEL_1
    RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
#endif
    IC_TIMER_REGISTER->CCMR1 = 0x21;
    IC_TIMER_REGISTER->CCER = 0xa;
    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 0xFFFF;
    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;

    IC_TIMER_REGISTER->CNT = 0;
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel4->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel4->CNDTR = buffersize;
    DMA1_Channel4->CCR = 0x989;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    DMA1_Channel5->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = buffersize;
    DMA1_Channel5->CCR = 0x989;
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma()
{
    out_put = 1;
#ifdef USE_TIMER_3_CHANNEL_1
    //          // de-init timer 2
    RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
#endif
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 115;

    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel4->CMAR = (uint32_t)&gcr;
    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel4->CNDTR = 23 + buffer_padding;
    DMA1_Channel4->CCR = 0x99b;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
    //(uint32_t)&gcr, (uint32_t)&IC_TIMER_REGISTER->CCR1,
    // LL_DMA_GetDataTransferDirection(DMA1,
    // INPUT_DMA_CHANNEL));
    DMA1_Channel5->CMAR = (uint32_t)&gcr;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = 23 + buffer_padding;
    DMA1_Channel5->CCR = 0x99b;
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState() { return (INPUT_PIN_PORT->IDR & INPUT_PIN); }

void setInputPolarityRising()
{
    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
        LL_TIM_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_DOWN);
}

void setInputPullUp()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_UP);
}

void enableHalfTransferInt() { LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL); }
void setInputPullNone()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_NO);
}
