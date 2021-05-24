#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(stm32fxx_led_demo_process, "stm32f4xx led demo");
AUTOSTART_PROCESSES(&stm32fxx_led_demo_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stm32fxx_led_demo_process, ev, data)
{
  
  PROCESS_BEGIN();
  etimer_set(&et, CLOCK_SECOND * 1);

  while(1) {
    leds_single_toggle(0);
	etimer_reset(&et);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */


