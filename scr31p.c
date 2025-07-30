#include <p18f4523.h>
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

#define _XTAL_FREQ 8000000UL  // 8 MHz (changed from 4 MHz)

// === VARIABLES ===
volatile unsigned char zcAP = 0, zcAN = 0, zcBP = 0, zcBN = 0, zcCP = 0, zcCN = 0;  // 6 separate zero-crossing flags
volatile unsigned char activeAP = 0, activeAN = 0, activeBP = 0, activeBN = 0, activeCP = 0, activeCN = 0;  // 6 active flags
volatile unsigned int delayTicks = 0;  // Single delay for all phases
volatile unsigned int lastTAP = 0, lastTAN = 0, lastTBP = 0, lastTBN = 0, lastTCP = 0, lastTCN = 0;  // 6 timers

// PORTB previous state for change detection
volatile unsigned char portb_old = 0;

// Individual pulse control for each SCR (6 separate pulse timers)
volatile unsigned char pulseActiveAP = 0, pulseActiveAN = 0, pulseActiveBP = 0;
volatile unsigned char pulseActiveBN = 0, pulseActiveCP = 0, pulseActiveCN = 0;
volatile unsigned int pulseStartAP = 0, pulseStartAN = 0, pulseStartBP = 0;
volatile unsigned int pulseStartBN = 0, pulseStartCP = 0, pulseStartCN = 0;

#define PULSE_WIDTH_TICKS 500  // 2ms = 500 ticks @ 4us/tick (adjusted for 8MHz)

// === ISR for C18 compiler ===
#pragma interrupt high_isr
void high_isr(void) {
    // INT0: RB0 – A+ trigger
    if (INTCONbits.INT0IF) {
        if (!activeAP) {
            zcAP = 1;
        }
        INTCONbits.INT0IF = 0;
    }
    
    // INT1: RB1 – A- trigger
    if (INTCON3bits.INT1IF) {
        if (!activeAN) {
            zcAN = 1;
        }
        INTCON3bits.INT1IF = 0;
    }
    
    // INT2: RB2 – B+ trigger
    if (INTCON3bits.INT2IF) {
        if (!activeBP) {
            zcBP = 1;
        }
        INTCON3bits.INT2IF = 0;
    }
    
    // PORTB Change Interrupt for RB3, RB4, RB5
    if (INTCONbits.RBIF) {
        unsigned char portb_new = PORTB;
        unsigned char portb_changed = portb_new ^ portb_old;
        
        // RB3 (B-) - rising edge detection
        if ((portb_changed & 0x08) && (portb_new & 0x08)) {
            if (!activeBN) {
                zcBN = 1;
            }
        }
        
        // RB4 (C+) - rising edge detection  
        if ((portb_changed & 0x10) && (portb_new & 0x10)) {
            if (!activeCP) {
                zcCP = 1;
            }
        }
        
        // RB5 (C-) - rising edge detection
        if ((portb_changed & 0x20) && (portb_new & 0x20)) {
            if (!activeCN) {
                zcCN = 1;
            }
        }
        
        portb_old = portb_new;  // Update previous state
        INTCONbits.RBIF = 0;    // Clear flag
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
    Delay10TCYx(10);         // Acquisition delay (doubled for 8MHz)
    ADCON0bits.GO = 1;       // Start conversion
    while (ADCON0bits.GO);   // Wait for completion
    
    // Debug: Test different values
    // Uncomment one line to test different delays:
    // return 0;        // Should give NO delay (immediate firing)
    // return 256;      // Should give ~2ms delay (2000 ticks)
    // return 512;      // Should give ~4ms delay (4000 ticks)
    // return 1023;     // Should give ~8ms delay (8000 ticks)
    
   return ((int)ADRESH << 8) | ADRESL;
}

// === MAP ADC TO DELAY (up to 8ms = 2000 ticks @ 4us each) ===
int calculateDelayTicks(int adcVal) {
    // Timer1: 8MHz/4/8 = 250kHz = 4us per tick (adjusted for 8MHz)
    // Max delay: 8ms = 2000 ticks
    return (adcVal * 2000) / 1023;  // 0-2000 ticks = 0-8ms delay
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

// === INDIVIDUAL SCR PULSE FUNCTIONS ===
void startSCRPulse(unsigned char bitIndex) {
    unsigned int now = readTimer1();
    
    switch(bitIndex) {
        case 0: // A+ (RD0)
            if (!pulseActiveAP) {
                LATD |= 0x01;  // Set RD0
                pulseStartAP = now;
                pulseActiveAP = 1;
            }
            break;
        case 1: // A- (RD1)
            if (!pulseActiveAN) {
                LATD |= 0x02;  // Set RD1
                pulseStartAN = now;
                pulseActiveAN = 1;
            }
            break;
        case 2: // B+ (RD2)
            if (!pulseActiveBP) {
                LATD |= 0x04;  // Set RD2
                pulseStartBP = now;
                pulseActiveBP = 1;
            }
            break;
        case 3: // B- (RD3)
            if (!pulseActiveBN) {
                LATD |= 0x08;  // Set RD3
                pulseStartBN = now;
                pulseActiveBN = 1;
            }
            break;
        case 4: // C+ (RD4)
            if (!pulseActiveCP) {
                LATD |= 0x10;  // Set RD4
                pulseStartCP = now;
                pulseActiveCP = 1;
            }
            break;
        case 5: // C- (RD5)
            if (!pulseActiveCN) {
                LATD |= 0x20;  // Set RD5
                pulseStartCN = now;
                pulseActiveCN = 1;
            }
            break;
    }
}

// === UPDATE ALL PULSE TIMERS ===
void updateSCRPulses(void) {
    unsigned int now = readTimer1();
    unsigned int elapsed;
    
    // Check A+ pulse
    if (pulseActiveAP) {
        elapsed = timerDiff(now, pulseStartAP);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x01;  // Clear RD0
            pulseActiveAP = 0;
        }
    }
    
    // Check A- pulse
    if (pulseActiveAN) {
        elapsed = timerDiff(now, pulseStartAN);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x02;  // Clear RD1
            pulseActiveAN = 0;
        }
    }
    
    // Check B+ pulse
    if (pulseActiveBP) {
        elapsed = timerDiff(now, pulseStartBP);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x04;  // Clear RD2
            pulseActiveBP = 0;
        }
    }
    
    // Check B- pulse
    if (pulseActiveBN) {
        elapsed = timerDiff(now, pulseStartBN);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x08;  // Clear RD3
            pulseActiveBN = 0;
        }
    }
    
    // Check C+ pulse
    if (pulseActiveCP) {
        elapsed = timerDiff(now, pulseStartCP);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x10;  // Clear RD4
            pulseActiveCP = 0;
        }
    }
    
    // Check C- pulse
    if (pulseActiveCN) {
        elapsed = timerDiff(now, pulseStartCN);
        if (elapsed >= PULSE_WIDTH_TICKS) {
            LATD &= ~0x20;  // Clear RD5
            pulseActiveCN = 0;
        }
    }
}

// === INIT FUNCTION for C18 ===
void init(void) {
    // Port configuration
    TRISB = 0x3F;           // RB0-RB5 as input (6 zero-crossing inputs)
    TRISA = 0x01;           // RA0 (AN0) input
    TRISD = 0x00;           // PORTD output
    LATD = 0;               // Clear PORTD

    // ADC Configuration for C18 (adjusted for 8MHz)
    ADCON1 = 0x0E;          // Configure analog inputs (AN0 only)
    ADCON2 = 0xBE;          // Right justified, 20 TAD, FOSC/64 (adjusted for 8MHz)
    ADCON0 = 0x00;          // ADC off initially
    
    // Alternative ADC configuration - try this if above doesn't work:
    // ADCON1 = 0x0F;       // All pins digital except AN0
    // ADCON2 = 0x96;       // Right justified, 4 TAD, FOSC/64

    // Timer1: 8MHz/4 = 2MHz, prescaler 1:8 = 250kHz = 4us/tick (adjusted for 8MHz)
    T1CONbits.TMR1CS = 0;   // Internal clock (Fosc/4)
    T1CONbits.T1CKPS0 = 1;  // Prescaler 1:8
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.TMR1ON = 1;   // Enable Timer1
    TMR1H = 0;              // Reset timer
    TMR1L = 0;

    // External Interrupts
    // INT0: RB0 – A+ input
    INTCON2bits.INTEDG0 = 1; // Rising edge
    INTCONbits.INT0IF = 0;   // Clear flag
    INTCONbits.INT0IE = 1;   // Enable interrupt

    // INT1: RB1 – A- input
    INTCON2bits.INTEDG1 = 1; // Rising edge
    INTCON3bits.INT1IF = 0;  // Clear flag
    INTCON3bits.INT1IE = 1;  // Enable interrupt

    // INT2: RB2 – B+ input
    INTCON2bits.INTEDG2 = 1; // Rising edge
    INTCON3bits.INT2IF = 0;  // Clear flag
    INTCON3bits.INT2IE = 1;  // Enable interrupt
    
    // PORTB Change Interrupt for RB3, RB4, RB5 (B-, C+, C-)
    portb_old = PORTB;           // Initialize previous state
    INTCONbits.RBIE = 1;         // Enable PORTB change interrupt
    INTCONbits.RBIF = 0;         // Clear flag

    // Enable global interrupts
    INTCONbits.GIEH = 1;     // Enable high priority interrupts
    INTCONbits.GIEL = 1;     // Enable low priority interrupts

    // Clear state variables
    zcAP = zcAN = zcBP = zcBN = zcCP = zcCN = 0;
    activeAP = activeAN = activeBP = activeBN = activeCP = activeCN = 0;
    delayTicks = 0;
    lastTAP = lastTAN = lastTBP = lastTBN = lastTCP = lastTCN = 0;
    
    // Clear individual pulse variables
    pulseActiveAP = pulseActiveAN = pulseActiveBP = 0;
    pulseActiveBN = pulseActiveCP = pulseActiveCN = 0;
    pulseStartAP = pulseStartAN = pulseStartBP = 0;
    pulseStartBN = pulseStartCP = pulseStartCN = 0;
}

// === MAIN for C18 ===
void main(void) {
    unsigned int now, elapsed;
    int adcVal;
    
    init();

    while (1) {
        now = readTimer1();

        // Update all individual pulse timers (must be called frequently)
        updateSCRPulses();

        // Read ADC for delay (done once per loop)
        adcVal = readADC();
        delayTicks = calculateDelayTicks(adcVal);

        // A+ (RD0) - triggered by RB0
        if (zcAP && !activeAP) {
            lastTAP = now;
            activeAP = 1;
            zcAP = 0;
        }
        if (activeAP) {
            elapsed = timerDiff(now, lastTAP);
            if (elapsed >= delayTicks) {
                startSCRPulse(0);  // Fire RD0 (A+)
                activeAP = 0;
            }
        }

        // A- (RD1) - triggered by RB1
        if (zcAN && !activeAN) {
            lastTAN = now;
            activeAN = 1;
            zcAN = 0;
        }
        if (activeAN) {
            elapsed = timerDiff(now, lastTAN);
            if (elapsed >= delayTicks) {
                startSCRPulse(1);  // Fire RD1 (A-)
                activeAN = 0;
            }
        }

        // B+ (RD2) - triggered by RB2
        if (zcBP && !activeBP) {
            lastTBP = now;
            activeBP = 1;
            zcBP = 0;
        }
        if (activeBP) {
            elapsed = timerDiff(now, lastTBP);
            if (elapsed >= delayTicks) {
                startSCRPulse(2);  // Fire RD2 (B+)
                activeBP = 0;
            }
        }

        // B- (RD3) - triggered by RB3
        if (zcBN && !activeBN) {
            lastTBN = now;
            activeBN = 1;
            zcBN = 0;
        }
        if (activeBN) {
            elapsed = timerDiff(now, lastTBN);
            if (elapsed >= delayTicks) {
                startSCRPulse(3);  // Fire RD3 (B-)
                activeBN = 0;
            }
        }

        // C+ (RD4) - triggered by RB4
        if (zcCP && !activeCP) {
            lastTCP = now;
            activeCP = 1;
            zcCP = 0;
        }
        if (activeCP) {
            elapsed = timerDiff(now, lastTCP);
            if (elapsed >= delayTicks) {
                startSCRPulse(4);  // Fire RD4 (C+)
                activeCP = 0;
            }
        }

        // C- (RD5) - triggered by RB5
        if (zcCN && !activeCN) {
            lastTCN = now;
            activeCN = 1;
            zcCN = 0;
        }
        if (activeCN) {
            elapsed = timerDiff(now, lastTCN);
            if (elapsed >= delayTicks) {
                startSCRPulse(5);  // Fire RD5 (C-)
                activeCN = 0;
            }
        }
    }
}
