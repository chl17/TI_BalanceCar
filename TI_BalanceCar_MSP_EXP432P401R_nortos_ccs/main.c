#include <./Headers/TI_BalanceCar.h>

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    TI_BalanceCar_Init();

    /* Sleeping when not in use */
    while(1)
    {
        //turnTest(20);
        //runTest(20);
        //printf("Pitchy: %f", Pitchy);
        //printf("speedN: %.2f %.2f\n", speedNLeft, speedNRight);
        //delay_ms(500);
    }
}
