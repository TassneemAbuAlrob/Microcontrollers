#define _XTAL_FREQ   4000000UL  // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#define STARTVALUE  3036

#ifndef XC_LCD_X8_H
#define _XTAL_FREQ   4000000UL 
#define LCD_TYPE 2
#define LCD_LINE_TWO 0x40
#define LCD_LINE_SIZE 16
#define lcd_output_enable(x) PORTEbits.RE1 = x
#define lcd_output_rs(x) PORTEbits.RE2 = x

struct lcd_pin_map {
    unsigned un1 : 1;
    unsigned rs : 1;
    unsigned rw : 1;
    unsigned enable : 1;
    unsigned data : 4;
} lcd __at(0xF83);
#endif

void delay_ms(unsigned int n);
void delay_cycles(unsigned char n);
void init_adc_no_lib(void);
float read_adc_voltage(unsigned char channel);
int read_adc_raw_no_lib(unsigned char channel);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);
void restartTimer0(void);
unsigned char LCD_INIT_STRING[4] = {0x08, 0xc, 1, 6};

unsigned short sndC = 0; //cooking
unsigned short minC = 0;
unsigned short hrC = 0;
unsigned short Mode = 0;
unsigned short oven = 0;
signed short total_time = 0;
unsigned short bzr = 0;

int state = 0;

unsigned short snd = 0; //clock
unsigned short min = 0;
unsigned short hr = 0;


float CT;
float sp;
short flagToggle = 0;

void delay_ms(unsigned int n) {
    int i;
    for (i = 0; i < n; i++) __delaywdt_ms(1);
}

void delay_cycles(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) CLRWDT();
}

void restartTimer0(void) {
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
}

void peeping(void) {
    PORTCbits.RC1 = 1;
    delay_ms(1000);
    PORTCbits.RC1 = 0;
    delay_ms(1000);
    PORTCbits.RC1 = 1;
    delay_ms(1000);
    PORTCbits.RC1 = 0;
    delay_ms(1000);
    PORTCbits.RC1 = 1;
    delay_ms(1000);
    PORTCbits.RC1 = 0;
    delay_ms(1000);
    PORTCbits.RC1 = 1;
    delay_ms(1000);
    PORTCbits.RC1 = 0;
    delay_ms(1000);

}

void going_down(void) {
    INTCONbits.TMR0IF = 0;
    total_time = sndC + 60 * minC + 60 * 60 * hrC;
    if (oven == 1 && total_time != 0) {
        total_time--;
        if (total_time == 0) {
            oven = 0;
            PORTCbits.RC5 = 0;
            bzr = 1;
        }
    }
    hrC = total_time / 3600;
    minC = (total_time - hrC * 3600) / 60;
    sndC = total_time - hrC * 3600 - minC * 60;

    restartTimer0();
}

void init_adc_no_lib(void) {
    ADCON0 = 0;
    ADCON0bits.ADON = 1;
    ADCON2 = 0b10001001;
}

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value;
    ADCON0bits.CHS = channel;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO) {
    };
    raw_value = ADRESH << 8 | ADRESL;
    return raw_value;
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
}

void lcd_send_nibble(unsigned char n) {
    lcd.data = n;
    delay_cycles(1);
    lcd_output_enable(1);
    __delaywdt_us(2);
    lcd_output_enable(0);
}

void lcd_send_byte(unsigned char cm_data, unsigned char n) {
    lcd_output_rs(cm_data);
    delay_cycles(1);
    delay_cycles(1);
    lcd_output_enable(0);
    lcd_send_nibble(n >> 4);
    lcd_send_nibble(n & 0x0f);
    if (cm_data) __delaywdt_us(200);
    else delay_ms(2);
}

void lcd_init(void) {
    unsigned char i;
    lcd_output_rs(0);
    lcd_output_enable(0);

    delay_ms(25);
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        delay_ms(6);
    }

    lcd_send_nibble(2);
    for (i = 0; i <= 3; ++i) lcd_send_byte(0, LCD_INIT_STRING[i]);
}

void lcd_gotoxy(unsigned char x, unsigned char y) {
    unsigned char address;

    switch (y) {
        case 1: address = 0x80;
            break;
        case 2: address = 0xc0;
            break;
        case 3: address = 0x80 + LCD_LINE_SIZE;
            break;
        case 4: address = 0xc0 + LCD_LINE_SIZE;
            break;
    }
    address += x - 1;
    lcd_send_byte(0, (unsigned char) (0x80 | address));
}

void lcd_putc(char c) {
    switch (c) {
        case '\f': lcd_send_byte(0, 1);
            delay_ms(2);
            break;
        case '\n': lcd_gotoxy(1, 2);
            break;
        case '\b': lcd_send_byte(0, 0x10);
            break;
        default: lcd_send_byte(1, c);
            break;
    }
}

void lcd_puts(char *s) {
    while (*s) {
        lcd_putc(*s);
        s++;
    }
}

void init(void) {

    
    ADCON0 = 0x00;
    ADCON1 = 0x0C; 
    TRISA = 0xFF; 
    TRISB = 0xFF; 
    TRISC = 0x80; 
    TRISD = 0x00; 
    TRISE = 0x00; 
    PORTD = 0;
    PIE1 = 0;

    
    INTCONbits.GIEH = 1; 
    INTCONbits.GIEL = 1; 
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON2bits.INTEDG0 = 1; 
    INTCON2bits.INTEDG1 = 1; 
    INTCON2bits.INTEDG2 = 1; 
    INTCON3bits.INT1IE = 1; 
    INTCON3bits.INT2IE = 1; 
    RCONbits.IPEN = 0; 
    INTCONbits.INT0IE = 1; 
    PORTCbits.RC5 = 0; 

    
    sndC = 0; 
    minC = 0;
    hrC = 0;
    Mode = 0;

  
    T0CON = 0;
    T0CONbits.T0PS0 = 1; 
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    T0CONbits.TMR0ON = 1;
    INTCONbits.TMR0IE = 1; 
    T0CONbits.TMR0ON = 1; 
    restartTimer0();

}

void our_modes(void) {
    INTCONbits.INT0IF = 0;
    if (Mode == 0) Mode = 1;
    else if (Mode == 1) Mode = 2;
    else if (Mode == 2) Mode = 3;
    else if (Mode == 3) Mode = 4;
    else Mode = 0;
}

void changeOnOff(void) {
    INTCON3bits.INT1IF = 0;
    if (flagToggle == 1) {
        oven = 1;
        flagToggle = 0;
    } else if (flagToggle == 0) {

        oven = 0;
        flagToggle = 1;
    }
}

void set_up(void) {
    if (state == 0) {
        if (Mode == 0) {
            total_time++;
        } else if (Mode == 1) {
            total_time += 10;
        } else if (Mode == 2) {
            total_time += 60;
        } else if (Mode == 3) {
            total_time += 600;
        } else if (Mode == 4) {
            total_time += 3600;
        }

        if (total_time > 36000) {
            total_time = 36000;
        }
    } else if (state == 1) {
        
        
        if (Mode == 0) {
            snd++;
           
        } else if (Mode == 1) {
            snd += 10;
            
        } else if (Mode == 2) {
            min += 1;
        } else if (Mode == 3) {
            min += 10;
        } else if (Mode == 4) {
            hr += 1;
        }

        if(snd >= 60){
            snd -=60;
            min++;
        }       
        if(min >= 60){
            min -= 60;
            hr++;
        }
        if(hr >= 23){
            hr -= 23;
            min=0;
            snd=0;
        }
        
        
        
        
    }
    hrC = total_time / 3600;
    minC = (total_time - hrC * 3600) / 60;
    sndC = total_time - hrC * 3600 - minC * 60;


}

void set_down(void) {
    if (Mode == 0) {
        total_time--;
    } else if (Mode == 1) {
        total_time -= 10;
    } else if (Mode == 2) {
        total_time -= 60;
    } else if (Mode == 3) {
        total_time -= 600;
    } else if (Mode == 4) {
        total_time -= 3600;
    }

    if (total_time < 0) {
        total_time = 0;
    }
    hrC = total_time / 3600;
    minC = (total_time - hrC * 3600) / 60;
    sndC = total_time - hrC * 3600 - minC * 60;


}

void control_cooking_temp(void) {
    if (oven == 1) {
        if (CT < (sp - 1)) {
            PORTCbits.RC5 = 1;
        } else if (CT > (sp + 1)) {
            PORTCbits.RC5 = 0;
        }
    }
}

void repeteTIMER0(void) {
    TMR0H = 0x0B;
    TMR0L = 0xDC;
    INTCONbits.TMR0IF = 0;
}

void __interrupt(high_priority) highIsr(void) {
    if (INTCONbits.TMR0IF) {
        going_down();

        snd++; 
        if (snd >= 60) {
            snd = 0;
            min++;
        }
        if (min >= 60) {
            min = 0;
            hr++;
        }
        if (hr >= 24) {
            hr = 0;
        }
        repeteTIMER0();
    } else if (INTCONbits.INT0IF) {
        our_modes();
    } else if (INTCON3bits.INT1IF) { //rb1
        changeOnOff();
    } else if (INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF = 0;
        oven = 0;
        PORTCbits.RC5 = 0;
    }
}

void main(void) {

    float analoggg0;
    float v;
    state = 2;



    init();




    char Buffer[32];


    lcd_init();
    init_adc_no_lib();
    lcd_putc('\f'); // Clears The Display

    while (1) {
        CLRWDT();

        if (PORTBbits.RB3 == 0) set_up();
        else if (PORTBbits.RB4 == 0) set_down();

        else if (PORTBbits.RB5 == 0) { //reset
            oven = 0;
            PORTCbits.RC5 = 0;
            sndC = 0;
            minC = 0;
            hrC = 0;
          

        } else if (PORTAbits.RA5 == 0)
        {
            if(state==2)// ideal state
            {   
                state=0;
                
            }
                
            else if (state == 0)
                state = 1; //clock
            else if (state == 1)
                state = 0;

            delay_ms(200);
        }

        analoggg0 = read_adc_voltage(0);
        sp = 40 * analoggg0;

        v = read_adc_voltage(2);
        CT = 200 * v;


        control_cooking_temp();
        
        if(state==2) //ideal
        {
            
            lcd_gotoxy(5, 1);
             sprintf(Buffer, "Set clock,   press RA 5");
             lcd_puts(Buffer);
        
        }
        if (state == 0) {
            lcd_gotoxy(1, 1);
            sprintf(Buffer, "Time: %02d:%02d:%02d", hrC, minC, sndC);
            lcd_puts(Buffer);


            lcd_gotoxy(1, 2);
            sprintf(Buffer, "CT:%4.1fC ", CT);
            lcd_puts(Buffer);

            lcd_gotoxy(11, 2);
            sprintf(Buffer, "CK:%s", oven == 1 ? " ON" : "OFF");
            lcd_puts(Buffer);

            lcd_gotoxy(1, 3);
            sprintf(Buffer, "SP:%4.1fC ", sp);
            lcd_puts(Buffer);

            lcd_gotoxy(11, 3);
            sprintf(Buffer, "HT:%s", PORTCbits.RC5 == 1 ? " ON" : "OFF");
            lcd_puts(Buffer);

            lcd_gotoxy(1, 4);
            sprintf(Buffer, "MD:%s", Mode == 0 ? "Sec  " : Mode == 1 ? "10Sec" : Mode == 2 ? "Min  " : Mode == 3 ? "10Min" : "HR   ");
            lcd_puts(Buffer);

            if (bzr == 1) {
                bzr = 0;
                peeping();
            }

        }
        if (state == 1) {
            lcd_gotoxy(5, 1);
            sprintf(Buffer, "%02d:%02d:  %02d", hr, min, snd);
            lcd_puts(Buffer);
            
            lcd_gotoxy(1, 4);
            sprintf(Buffer, "MD:%s", Mode == 0 ? "Sec  " : Mode == 1 ? "10Sec" : Mode == 2 ? "Min  " : Mode == 3 ? "10Min" : "HR   ");
            lcd_puts(Buffer);
        }
        delay_ms(200);


    }

}