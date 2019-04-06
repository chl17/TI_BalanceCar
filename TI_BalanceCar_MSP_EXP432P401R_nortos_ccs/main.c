#include <./Headers/TI_BalanceCar.h>

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Hardware initialization */
    TI_BalanceCar_Init();

    /* Put your implementation here, in Protocol() or in the Timer32 IRQHandler */
    while(1)
    {
        if(newLineReceived)
            Protocol();

        //turnTest(20);
        //runTest(20);
        //printf("Pitchy: %f", Pitchy);
        //printf("speedN: %.2f %.2f\n", speedNLeft, speedNRight);
        //delay_ms(500);
    }
}
