// TODO 2 Intelligently include or don't include this based on target
#ifdef 0==1

/******************************************************************************/

extern uint32_t SystemCoreClock;

int lm3s811_uart0_read()
{
    int data;

    while (UART0_FR_R & UART_FR_RXFE)
    {
        continue;
    }

    data = UART0_DR_R & UART_DR_DATA_M;

    return data;
}

void lm3s811_uart0_write(int data)
{
    while (UART0_FR_R & UART_FR_TXFF)
    {
        continue;
    }

    UART0_DR_R = data;
}

void lm3s811_uart0_interrupt()
{
    UART0_ICR_R = UART_ICR_RXIC;

    if (!(UART0_FR_R & UART_FR_RXFE))
    {
	gdb_serial_interrupt(UART0_DR_R & UART_DR_DATA_M);
    }
}

void lm3s811_uart0_initialize(void)
{

    volatile unsigned int temp;
    unsigned int rate;

    gdb_initialize(lm3s811_uart0_read, lm3s811_uart0_write);

    // Enable UART0
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;
    temp = SYSCTL_RCGC1_R;

    // Enable GPIOA
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;
    temp = SYSCTL_RCGC2_R;

    // Enable GPIOA/AFSEL_UART0 bits
    GPIO_PORTA_AFSEL_R = (GPIO_PORTA_AFSEL_R  & ~0x03) | (0x03);

    // Enable/Set DIR
    GPIO_PORTA_DIR_R   = (GPIO_PORTA_DIR_R    & ~0x03) | (0x00);

#if 0
    /* Set 2mA drive */
    GPIO_PORTA_DR2R_R  = (GPIO_PORTA_DR2R_R   & ~0x03) | (0x03);
    GPIO_PORTA_DR4R_R  = (GPIO_PORTA_DR4R_R   & ~0x03) | (0x00);
    GPIO_PORTA_DR8R_R  = (GPIO_PORTA_DR8R_R   & ~0x03) | (0x00);
    GPIO_PORTA_SLR_R   = (GPIO_PORTA_SLR_R    & ~0x03) | (0x00);

    /* Standard pins */
    GPIO_PORTA_ODR_R   = (GPIO_PORTA_ODR_R    & ~0x03) | (0x00);
    GPIO_PORTA_PUR_R   = (GPIO_PORTA_PUR_R    & ~0x03) | (0x00);
    GPIO_PORTA_PDR_R   = (GPIO_PORTA_PDR_R    & ~0x03) | (0x00);
    GPIO_PORTA_DEN_R   = (GPIO_PORTA_DEN_R    & ~0x03) | (0x03);
#endif

    /* 9600, 8N1, RX interrupt enabled, no FIFO */

    /*
     * BRD = BRDI + BRDF = UARTSysClk / (16 * BaudRate)
     *
     * Scale by 64 to compute the fraction:
     *
     * BRD * 64 = (UARTSysClk * 64) / (16 * BaudRate)
     * BRD * 64 = (UARTSysClk * 4) / (BaudRate)
     * BRDI = (BRD * 64) / 64;
     * BRDF = (BRD * 64) % 64;
     *
     * Round by 0.5:
     *
     * BRD * 64 = (2 * ((UARTSysClk * 4) / (BaudRate)) + 1) / 2
     * BRD * 64 = (((UARTSysClk * 8) / (BaudRate)) + 1) / 2
     *
     * UARTSysClk = SystemCoreClock
     * BaudRate = 9600
     */

    rate = ((SystemCoreClock * 8) / 9600 + 1) >> 1;

    UART0_CTL_R  = 0;
    UART0_ECR_R  = ~0u;
    UART0_IBRD_R = rate >> 6;
    UART0_FBRD_R = rate & 63;
    UART0_LCRH_R = UART_LCRH_WLEN_8;
    UART0_IM_R   = UART_IM_RXIM;
    UART0_ICR_R  = ~0u;
    UART0_CTL_R  = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
}
#endif
