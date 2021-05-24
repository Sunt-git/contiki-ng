#include "contiki.h"
#include "dev/leds.h"
#include "leds-arch.h"
#include "stm32f4xx_conf.h"

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  //GPIOF9,F10³õÊ¼»¯ÉèÖÃ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);//
	
  GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10ÉèÖÃ¸ß£¬µÆÃð
}

/*---------------------------------------------------------------------------*/
leds_mask_t
leds_arch_get(void)
{
  return 0;
}



