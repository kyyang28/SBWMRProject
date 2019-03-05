
#include "timer.h"
#include "dma.h"
#include "target.h"

//typedef struct timerHardware_s {
//    TIM_TypeDef *tim;
//    ioTag_t tag;
//    uint8_t channel;
//    timerUsageFlag_e usageFlags;
//    uint8_t output;
//    uint8_t alternateFunction;
//#if defined(USE_DSHOT)
//    DMA_Stream_TypeDef *dmaStream;		//	for STM32F4
//    uint32_t dmaChannel;					// 	for STM32F1
//    uint8_t dmaIrqHandler;
//#endif
//} timerHardware_t;

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{	/* TIM2 PWM 2 for Motor 1 Encoder Phase A */
		.tim = TIM2,
		.tag = IO_TAG(PA0),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM2,
#ifdef USE_DSHOT
		.dmaStream = NULL,
//		.dmaChannel = 0,
		.dmaIrqHandler = 0
#endif
	},
	{	/* TIM2 PWM 3 for Motor 1 Encoder Phase B */
		.tim = TIM2,
		.tag = IO_TAG(PA1),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM2,
#ifdef USE_DSHOT
		.dmaStream = NULL,
//		.dmaChannel = 0,
		.dmaIrqHandler = 0
#endif
	},
	{	/* TIM2 PWM 2 for Motor 2 Encoder Phase A */
		.tim = TIM4,
		.tag = IO_TAG(PB6),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM4,
#ifdef USE_DSHOT
		.dmaStream = NULL,
//		.dmaChannel = 0,
		.dmaIrqHandler = 0
#endif
	},
	{	/* TIM2 PWM 2 for Motor 2 Encoder Phase B */
		.tim = TIM4,
		.tag = IO_TAG(PB7),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM4,
#ifdef USE_DSHOT
		.dmaStream = NULL,
//		.dmaChannel = 0,
		.dmaIrqHandler = 0
#endif
	},
	{	/* TIM3 PWM generator for DC Brushed MOTOR 1 */
		.tim = TIM3,
		.tag = IO_TAG(PB0),
		.channel = TIM_Channel_3,
		.usageFlags = TIM_USE_MOTOR,
		.output = TIMER_OUTPUT_STANDARD,
		.alternateFunction = GPIO_AF_TIM3,
#ifdef USE_DSHOT
		.dmaStream = DMA2_Stream6,					// DEF_TIM_DMA_STR_0__TIM1_CH1_STREAM = DMA2_ST6_STREAM = DMA2_Stream6
		.dmaChannel = DMA_Channel_0,
		.dmaIrqHandler = DMA2_ST6_HANDLER			// DMA2_ST6_HANDLER = 14
#endif
	},
	{	/* TIM3 PWM generator for DC Brushed MOTOR 2 */
		.tim = TIM3,
		.tag = IO_TAG(PB1),				// TIM1 PA9 (TIM_Channel_2) is not working for some reason
		.channel = TIM_Channel_4,
		.usageFlags = TIM_USE_MOTOR,
		.output = TIMER_OUTPUT_STANDARD,
		.alternateFunction = GPIO_AF_TIM3,
#ifdef USE_DSHOT
		.dmaStream = DMA2_Stream6,					// DEF_TIM_DMA_STR_0__TIM1_CH1_STREAM = DMA2_ST6_STREAM = DMA2_Stream6
		.dmaChannel = DMA_Channel_0,
		.dmaIrqHandler = DMA2_ST6_HANDLER			// DMA2_ST6_HANDLER = 14
#endif
	},
};
