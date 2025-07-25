#include <p18cxxx.h>
#include <delays.h>

// === CONFIGURATION BITS for C18 ===
#pragma config OSC = HS         // HS oscillator
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF       // Internal/External Oscillator Switchover disabled
#pragma config PWRT = OFF       // Power-up Timer disabled
#pragma config BOREN = ON       // Brown-out Reset enabled
#pragma config BORV = 3         // Brown-out Reset Voltage bits
#pragma config WDT = OFF        // Watchdog Timer disabled
#pragma config WDTPS = 32768    // Watchdog Timer Postscale
#pragma config PBADEN = OFF     // PORTB<4:0> pins are configured as digital I/O
#pragma config LPT1OSC = OFF    // Timer1 configured for higher power operation
#pragma config MCLRE = ON       // MCLR pin enabled
#pragma config STVREN = ON      // Stack full/underflow will cause Reset
#pragma config LVP = OFF        // Single-Supply ICSP disabled
#pragma config XINST = OFF      // Instruction set extension disabled
#pragma config CP0 = OFF        // Code Protection disabled
#pragma config CP1 = OFF        // Code Protection disabled
#pragma config CPB = OFF        // Boot Block Code Protection disabled
#pragma config CPD = OFF        // Data EEPROM Code Protection disabled
#pragma config WRT0 = OFF       // Write Protection disabled
#pragma config WRT1 = OFF       // Write Protection disabled
#pragma config WRTC = OFF       // Configuration Register Write Protection disabled
#pragma config WRTB = OFF       // Boot Block Write Protection disabled
#pragma config WRTD = OFF       // Data EEPROM Write Protection disabled
#pragma config EBTR0 = OFF      // Table Read Protection disabled
#pragma config EBTR1 = OFF      // Table Read Protection disabled
#pragma config EBTRB = OFF      // Boot Block Table Read Protection disabled

#define _XTAL_FREQ 4000000UL  // 4 MHz

// === VARIABLES ===
volatile unsigned char zcA = 0, zcB = 0, zcC = 0;
volatile unsigned char scrIndexA = 0, scrIndexB = 0, scrIndexC = 0;
volatile unsigned char activeA = 0, activeB = 0, activeC = 0;
volatile unsigned int delayTicksA = 0, delayTicksB = 0, delayTicksC = 0;
volatile unsigned int lastTA = 0, lastTB = 0, lastTC = 0;

// === ISR for C18 compiler ===
#pragma interrupt high_isr
void high_isr(void) {
    if (INTCONbits.INT0IF) {  // Phase A ZC
        if (!activeA) zcA = 1;
        INTCONbits.INT0IF = 0;
    }
    if (INTCON3bits.INT1IF) {  // Phase B ZC
        if (!activeB) zcB = 1;
        INTCON3bits.INT1IF = 0;
    }
    if (INTCON3bits.INT2IF) {  // Phase C ZC
        if (!activeC) zcC = 1;
        INTCON3bits.INT2IF = 0;
    }
}

// Interrupt vector for C18
#pragma code high_vector = 0x08
void interrupt_at_high_vector(void) {
    _asm GOTO high_isr _endasm
}
#pragma code

// === ADC FUNCTION for C18 ===
int readADC(void) {
    ADCON0bits.CHS0 = 0;     // Select AN0
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS3 = 0;
    ADCON0bits.ADON = 1;     // Turn on ADC
    Delay10TCYx(5);          // Acquisition delay (C18 delay function)
    ADCON0bits.GO = 1;       // Start conversion
    while (ADCON0bits.GO);   // Wait for completion
    return ((int)ADRESH << 8) | ADRESL;
}

// === MAP ADC TO DELAY (up to 10ms = 5000 ticks @ 2us each) ===
int calculateDelayTicks(int adcVal) {
    return (adcVal * 5000) / 1023;
}

// === TIMER COMPARISON WITH OVERFLOW HANDLING ===
unsigned int timerDiff(unsigned int current, unsigned int previous) {
    if (current >= previous) {
        return current - previous;
    } else {
        // Handle Timer1 overflow (16-bit counter)
        return (0xFFFF - previous) + current + 1;
    }
}

// === READ TIMER1 16-BIT VALUE ===
unsigned int readTimer1(void) {
    unsigned char high1, low, high2;
    
    // Read twice to ensure consistency (avoid race condition)
    do {
        high1 = TMR1H;
        low = TMR1L;
        high2 = TMR1H;
    } while (high1 != high2);
    
    return ((unsigned int)high1 << 8) | low;
}

// === DOUBLE PULSE TRIGGER FUNCTION for C18 ===
void fireSCR(unsigned char bitIndex) {
    LATD = (1 << bitIndex);   // First pulse
    Delay1KTCYx(2);          // 2ms delay using C18 function
    LATD = 0;

    Delay1KTCYx(2);          // 2ms gap

    LATD = (1 << bitIndex);   // Second pulse
    Delay1KTCYx(2);          // 2ms delay
    LATD = 0;
}

// === INIT FUNCTION for C18 ===
void init(void) {
    // Port configuration
    TRISB = 0x07;           // RB0-RB2 as input (INT0–INT2)
    TRISA = 0x01;           // RA0 (AN0) input
    TRISD = 0x00;           // PORTD output
    LATD = 0;               // Clear PORTD

    // ADC Configuration for C18
    ADCON1 = 0x0E;          // Configure analog inputs (AN0 only)
    ADCON2 = 0x96;          // Right justified, 4 TAD, FOSC/64
    ADCON0 = 0x00;          // ADC off initially

    // Timer1: Fosc/4 = 1MHz, prescaler 1:8 = 2us/tick
    T1CONbits.TMR1CS = 0;   // Internal clock (Fosc/4)
    T1CONbits.T1CKPS0 = 1;  // Prescaler 1:8
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.TMR1ON = 1;   // Enable Timer1
    TMR1H = 0;              // Reset timer
    TMR1L = 0;

    // External Interrupts
    // INT0: RB0 – Phase A
    INTCON2bits.INTEDG0 = 0; // Falling edge
    INTCONbits.INT0IF = 0;   // Clear flag
    INTCONbits.INT0IE = 1;   // Enable interrupt

    // INT1: RB1 – Phase B
    INTCON2bits.INTEDG1 = 0; // Falling edge
    INTCON3bits.INT1IF = 0;  // Clear flag
    INTCON3bits.INT1IE = 1;  // Enable interrupt

    // INT2: RB2 – Phase C
    INTCON2bits.INTEDG2 = 0; // Falling edge
    INTCON3bits.INT2IF = 0;  // Clear flag
    INTCON3bits.INT2IE = 1;  // Enable interrupt

    // Enable global interrupts
    INTCONbits.GIEH = 1;     // Enable high priority interrupts
    INTCONbits.GIEL = 1;     // Enable low priority interrupts

    // Clear state variables
    zcA = zcB = zcC = 0;
    activeA = activeB = activeC = 0;
    scrIndexA = scrIndexB = scrIndexC = 0;
    delayTicksA = delayTicksB = delayTicksC = 0;
    lastTA = lastTB = lastTC = 0;
}

// === MAIN for C18 ===
void main(void) {
    init();

    while (1) {
        unsigned int now = readTimer1();

        // Phase A - Master phase (reads ADC)
        if (zcA && !activeA) {
            int adcVal = readADC();
            delayTicksA = calculateDelayTicks(adcVal);
            delayTicksB = delayTicksA;  // Sync all phases to same delay
            delayTicksC = delayTicksA;
            lastTA = now;
            activeA = 1; 
            scrIndexA = 0;
            zcA = 0;
        }
        if (activeA && (timerDiff(now, lastTA) >= delayTicksA)) {
            fireSCR(scrIndexA);  // RD0, RD1
            scrIndexA++;
            lastTA = now;  // Use same 'now' reading for consistency
            if (scrIndexA >= 2) activeA = 0;
        }

        // Phase B - Uses synchronized delay
        if (zcB && !activeB) {
            lastTB = now;
            activeB = 1; 
            scrIndexB = 2;  // Start at RD2
            zcB = 0;
        }
        if (activeB && (timerDiff(now, lastTB) >= delayTicksB)) {
            fireSCR(scrIndexB);  // RD2, RD3
            scrIndexB++;
            lastTB = now;  // Use same 'now' reading for consistency
            if (scrIndexB >= 4) activeB = 0;
        }

        // Phase C - Uses synchronized delay
        if (zcC && !activeC) {
            lastTC = now;
            activeC = 1; 
            scrIndexC = 4;  // Start at RD4
            zcC = 0;
        }
        if (activeC && (timerDiff(now, lastTC) >= delayTicksC)) {
            fireSCR(scrIndexC);  // RD4, RD5
            scrIndexC++;
            lastTC = now;  // Use same 'now' reading for consistency
            if (scrIndexC >= 6) activeC = 0;
        }
    }
}
