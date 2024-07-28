/*****************************************************************************
 * File           pinToPortMappingdemo
 * Author         Glaive software team
 * This version   V1
 * Date           28-July-2024
 * 
 *  This program shows how to get the GPIO port associated with an Arduino pin
 *  on an Arduino Giga R1 board with the STM32H747XI microcontroller.
 *  It then uses GPIO commands to blink the blue led four times.
 * 
 *  This program is Free Software and has ABSOLUTELY NO WARRANTY.
 */

// High nibble = port number (0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, 7=H, etc.)
// Low nibble  = pin number
// #define STM_PORT(X) (((uint32_t)(X) >> 4) & 0xF)
// #define STM_PIN(X)  ((uint32_t)(X) & 0xF)

void setup()
{
    Serial.begin(9600);
    while (!Serial) ;

    uint8_t pin;
    for (pin=0; pin<103; pin++)
    {
        PinName pinName = digitalPinToPinName(pin);
        int stmPort = STM_PORT(pinName);
        int stmPin  = STM_PIN(pinName);
        char portLetter = 'A' + stmPort;
        Serial.print("Arduino pin D");
        Serial.print(pin);
        Serial.print(" corresponds to GPIO pin ");
        Serial.print("P");
        Serial.print(portLetter);
        Serial.print("_");
        Serial.print(stmPin);
        Serial.print(" on port ");
        Serial.print(portLetter);
        Serial.println(".");
    }

    // Pin D88=PE_3=LEDB controls the blue LED.
    // Set this pin to be an output pin. Equivalent to pinMode(D88, OUTPUT).

    GPIO_TypeDef *GPIO_BLUE;
    GPIO_BLUE = getGPIOport(D88);
    GPIO_BLUE->MODER &= ~(0x3 << (2 * 2));    // Clear mode bits
    GPIO_BLUE->MODER |= (0x1 << (2 * 2));     // Set mode to output (01)
    
    // Get the bit masks needed to set the pin high and low.

    uint16_t bitnum = getSetBit(D88);
    uint32_t setBLUEbit = 1U << bitnum;       // bit mask to use to set pin HIGH
    uint32_t clrBLUEbit = 1U << (bitnum+16);  // bit mask to use to set pin LOW

    /*
     *    Set the pin LOW to turn on the blue LED.
     *    Set it HIGH to turn it off.
     *    Blink the blue LED 4 times.
     */

    for (int i=0; i<4; i++)
    {
        GPIO_BLUE->BSRR = clrBLUEbit;   // turn blue LED on
        delay(500);                     // wait half a second
        GPIO_BLUE->BSRR = setBLUEbit;   // turn blue LED off
        delay(500);                     // wait half a second
   }
}

// Get the port from the pin number

GPIO_TypeDef *getGPIOport(int pin)
{
    GPIO_TypeDef *ports[11] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};
    PinName pinName = digitalPinToPinName(pin);
    int stmPort = STM_PORT(pinName);
    int stmPin  = STM_PIN(pinName);
    char portLetter = 'A' + stmPort;
    GPIO_TypeDef *port = ports[stmPort];
    return port;
}

uint16_t getSetBit(int pin)
{
    PinName pinName = digitalPinToPinName(pin);
    int stmPin  = STM_PIN(pinName);
    return stmPin;
}

void loop()
{

}
