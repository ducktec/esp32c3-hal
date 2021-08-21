/// Peripheral input signals via GPIO Matrix (per TRM v0.3, Table 5-1)
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum InputSignal {
    SPIQ = 0,
    SPID = 1,
    SPIHD = 2,
    SPIWP = 3,
    U0RXD = 6,
    U0CTS = 7,
    U0DSR = 8,
    U1RXD = 9,
    U1CTS = 10,
    U1DSR = 11,
    I2S_MCLK = 12,
    I2SO_BCK = 13,
    I2SO_WS = 14,
    I2SI_SD = 15,
    I2SI_BCK = 16,
    I2SI_WS = 17,
    GPIO_BT_PRIORITY = 18,
    GPIO_BT_ACTIVE = 19,
    CPU_GPIO_0 = 28,
    CPU_GPIO_1 = 29,
    CPU_GPIO_2 = 30,
    CPU_GPIO_3 = 31,
    CPU_GPIO_4 = 32,
    CPU_GPIO_5 = 33,
    CPU_GPIO_6 = 34,
    CPU_GPIO_7 = 35,
    EXT_ADC_START = 45,
    RMT_SIG_0 = 51,
    RMT_SIG_1 = 52,
    I2CEXT0_SCL = 53,
    I2CEXT0_SDA = 54,
    FSPICLK = 63,
    FSPIQ = 64,
    FSPID = 65,
    FSPIHD = 66,
    FSPIWP = 67,
    FSPICS0 = 68,
    TWAI_RX = 74,
    SIG_FUNC_97 = 97,
    SIG_FUNC_98 = 98,
    SIG_FUNC_99 = 99,
    SIG_FUNC_100 = 100,
}

/// Peripheral output signals via GPIO Matrix (per TRM v0.3, Table 5-1)
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {
    SPIQ = 0,
    SPID = 1,
    SPIHD = 2,
    SPIWP = 3,
    SPICLK_MUX = 4,
    SPICS0 = 5,
    U0TXD = 6,
    U0RTS = 7,
    U0DTR = 8,
    U1TXD = 9,
    U1RTS = 10,
    U1DTR = 11,
    I2S_MCLK = 12,
    I2SO_BCK = 13,
    I2SO_WS = 14,
    I2SI_SD = 15,
    I2SI_BCK = 16,
    I2SI_WS = 17,
    GPIO_WLAN_PRIO = 18,
    GPIO_WLAN_ACTIVE = 19,
    CPU_GPIO_0 = 28,
    CPU_GPIO_1 = 29,
    CPU_GPIO_2 = 30,
    CPU_GPIO_3 = 31,
    CPU_GPIO_4 = 32,
    CPU_GPIO_5 = 33,
    CPU_GPIO_6 = 34,
    CPU_GPIO_7 = 35,
    USB_JTAG_TCK = 36,
    USB_JTAG_TMS = 37,
    USB_JTAG_TDI = 38,
    USB_JTAG_TDO = 39,
    LEDC_LS_SIG0 = 45,
    LEDC_LS_SIG1 = 46,
    LEDC_LS_SIG2 = 47,
    LEDC_LS_SIG3 = 48,
    LEDC_LS_SIG4 = 49,
    LEDC_LS_SIG5 = 50,
    RMT_SIG_0 = 51,
    RMT_SIG_1 = 52,
    I2CEXT0_SCL = 53,
    I2CEXT0_SDA = 54,
    GPIO_SD0 = 55,
    GPIO_SD1 = 56,
    GPIO_SD2 = 57,
    GPIO_SD3 = 58,
    I2SO_SD1 = 59,
    FSPICLK_MUX = 63,
    FSPIQ = 64,
    FSPID = 65,
    FSPIHD = 66,
    FSPIWP = 67,
    FSPICS0 = 68,
    FSPICS1 = 69,
    FSPICS3 = 70,
    FSPICS2 = 71,
    FSPICS4 = 72,
    FSPICS5 = 73,
    TWAI_TX = 74,
    TWAI_BUS_OFF_ON = 75,
    TWAI_CLKOUT = 76,
    ANT_SEL0 = 89,
    ANT_SEL1 = 90,
    ANT_SEL2 = 91,
    ANT_SEL3 = 92,
    ANT_SEL4 = 93,
    ANT_SEL5 = 94,
    ANT_SEL6 = 95,
    ANT_SEL7 = 96,
    SIG_FUNC_97 = 97,
    SIG_FUNC_98 = 98,
    SIG_FUNC_99 = 99,
    SIG_FUNC_100 = 100,
    CLK_OUT1 = 123,
    CLK_OUT2 = 124,
    CLK_OUT3 = 125,
    SPICS1 = 126,
    USB_JTAG_TRST = 127,
}
