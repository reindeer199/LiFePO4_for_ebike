/*  LiFePO Hardware Versions	*/

#define HW_version 3

/***************************/
#if (HW_version == 1)
/***************************/
    // digital PINs Hardware V01
    #define PWM_OUT 3
    #define MON_ADC 8
    #define MON_REQ 7
    #define MON_I2C 6
    #define MON_WDT 5
    #define BMS_OUT 9

    // analog PINs Hardware V01
    #define V_12 PIN_A2
    #define V_24 PIN_A3
    #define V_36 PIN_A6
    #define V_INT PIN_A7
    #define I_OUT PIN_A0
    #define I_IN PIN_A1
    
    // mA or mV @ ADC 1023
    #define FAKTOR_CURRENT_IN 2414
    #define FAKTOR_CURRENT_OUT 10735
    #define FAKTOR_VOLTAGE_12 14514
    #define FAKTOR_VOLTAGE_24 28776
    #define FAKTOR_VOLTAGE_36 43234
    #define FAKTOR_VOLTAGE_INT 52140
    // Offsets
    #define OFFSET_CURRENT_IN 28
    #define OFFSET_CURRENT_OUT 12
#endif    
#if (HW_version == 2)
/***************************/

/***************************/
    // digital PINs Hardware V02
    #define PWM_OUT 3
    #define MON_ADC 8
    #define MON_REQ 7
    #define MON_I2C 6
    #define MON_WDT 5
    #define BMS_OUT 9

    // analog PINs Hardware V02
    #define V_12 PIN_A2
    #define V_24 PIN_A3
    #define V_36 PIN_A6
    #define V_INT PIN_A7
    #define I_OUT PIN_A0
    #define I_IN PIN_A1
    
    // in mA or mV @ ADC 1023
    #define FAKTOR_CURRENT_IN 2394
    #define FAKTOR_CURRENT_OUT 9411
    #define FAKTOR_VOLTAGE_12 14514
    #define FAKTOR_VOLTAGE_24 28421
    #define FAKTOR_VOLTAGE_36 42513
    #define FAKTOR_VOLTAGE_INT 40100
    // Offsets in mA
    #define OFFSET_CURRENT_IN 45
    #define OFFSET_CURRENT_OUT 283
/***************************/
#endif
#if (HW_version == 3)
/***************************/
    // digital PINs Hardware V03
    #define PWM_OUT 3
    #define MON_ADC 8
    #define MON_REQ 7
    #define MON_I2C 6
    #define MON_WDT 5
    #define BMS_OUT 9
    #define LED_RED 31

    // analog PINs Hardware V03
    #define V_12 PIN_A0
    #define V_24 PIN_A1
    #define V_36 PIN_A2
    #define V_INT PIN_A3
    #define I_OUT PIN_A6
    #define I_IN PIN_A7
    
    // Scaling Hardware V03
    // in mA or mV @ ADC 1023
    #define FAKTOR_CURRENT_IN 2307
    #define FAKTOR_CURRENT_OUT 9382
    #define FAKTOR_VOLTAGE_12 14583
    #define FAKTOR_VOLTAGE_24 28520
    #define FAKTOR_VOLTAGE_36 42191
    #define FAKTOR_VOLTAGE_INT 39000
    // Offsets in mA
    #define OFFSET_CURRENT_IN 34
    #define OFFSET_CURRENT_OUT 191

#endif
