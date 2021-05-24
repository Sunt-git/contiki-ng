#include "contiki.h"
#include "leds-arch.h"

PROCESS(led, "led1");

AUTOSTART_PROCESSES(&led);

PROCESS_THREAD(led, ev, data)
{
	static struct etimer timer;

	PROCESS_BEGIN();

	etimer_set(&timer, CLOCK_SECOND * 0.5);

	while(1)
	{
		//for(int i=0;i < 10000; i++)
			//for(int j=0; j < 10000; j++)		
		LED0=!LED0;
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
		etimer_reset(&timer);
	}

	PROCESS_END();
}
