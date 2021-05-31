#include "contiki.h"
#include "leds-arch.h"
#include "usart.h"
#include "platform.h"
#include "ieee-addr.h"
#include "stm32f4-conf.h"
#include "linkaddr.h"

void
platform_init_stage_one(void)
{
	leds_arch_init();		  		//³õÊ¼»¯ÓëLEDÁ¬½ÓµÄÓ²¼þ½Ó¿Ú 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ÉèÖÃÏµÍ³ÖÐ¶ÏÓÅÏÈ¼¶·Ö×é2
}

void
platform_init_stage_two()
{
	uart_init(115200);	//´®¿Ú³õÊ¼»¯²¨ÌØÂÊÎª115200

  /* Populate linkaddr_node_addr */
  ieee_addr_cpy_to(linkaddr_node_addr.u8, LINKADDR_SIZE);
}

void
platform_init_stage_three()
{
        //uart_init(115200);      //´®¿Ú³õÊ¼»¯²¨ÌØÂÊÎª115200
}

/*---------------------------------------------------------------------------*/
void
platform_idle()
{
  //lpm_drop();
}

