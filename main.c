/*
 * File:   main.c
 * Author: 2020
 *
 * Created on December 9, 2022, 10:39 AM
 */

// CONFIG1H
#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency

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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#ifndef XC_LCD_X8_H
#define _XTAL_FREQ   4000000UL 

#define LCD_TYPE 2

#define LCD_LINE_TWO 0x40    // LCD RAM address for the second line
#define LCD_LINE_SIZE 16


struct lcd_pin_map {

    unsigned un1 : 1;
    unsigned rs : 1;           
    unsigned rw : 1;
    unsigned enable : 1; 
    unsigned data : 4; 

} lcd __at(0xF83);

#define lcd_output_enable(x) PORTEbits.RE1 = x 
#define lcd_output_rs(x) PORTEbits.RE2 = x 
void delay_cycles(unsigned char n);
void delay_ms(unsigned int n);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);

void init_adc_no_lib(void);
int read_adc_raw_no_lib(unsigned char channel);
float read_adc_voltage(unsigned char channel);

void init_pwm1(void);
void set_pwm1_raw(unsigned int raw_value);
void set_pwm1_percent(float value);// value 0
void set_pwm1_voltage(float value);// value 0--5V, 
void set_pwm1_general(float value, float min, float max);//


unsigned char LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};

void setupPorts(void);
void __interrupt(high_priority) highIsr(void); 
void initialization(void);
void reloadTimer3(void);
void pwd_Timer3(int limit);

#define STARTVALUE  3036
int our_modes = 0; 
int cTIMERTHREE = 0; 
int HS = 0;
float sp, roomT,vol;
bool H_ON = true;
bool C_ON = false;
int RPS_count = 0;
int CoolError;
#endif

void delay_ms(unsigned int n) {
    int x;
    for (x = 0; x <= n; x++) {
        __delaywdt_ms(1);
    }
}

void delay_cycles(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) {
        CLRWDT();
    }

}

void lcd_send_nibble(unsigned char n) {

    lcd.data = n;

    delay_cycles(1); //here 1111
    lcd_output_enable(1);
    __delaywdt_us(2); //delay_us(2);//20
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
    else
        delay_ms(2); //added by raed
}

void lcd_init(void) {

    unsigned char i;


    lcd_output_rs(0);
    lcd_output_enable(0);

    delay_ms(25); //   
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        delay_ms(6); //5
    }

    lcd_send_nibble(2);
    for (i = 0; i <= 3; ++i)
        lcd_send_byte(0, LCD_INIT_STRING[i]);
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

void Lcd_Shift_Right(void) {

    lcd_send_byte(0, 0x1C);
  
}

void Lcd_Shift_Left(void) {
    lcd_send_byte(0, 0x18);
    
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

    while (ADCON0bits.GO) {}; 

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

void init_pwm1(void)
{
    PR2 = 255; 
    T2CON = 0;
    CCP1CON = 0x0C; 
    T2CONbits.TMR2ON = 1;  
    TRISCbits.RC2 =0; 
}
void set_pwm1_raw(unsigned int raw_value)
{
    CCPR1L = (raw_value >> 2) & 0x00FF; 
    CCP1CONbits.DC1B = raw_value & 0x0003; 
    }
void set_pwm1_percent(float value)
{
    float tmp = value*1023.0/100.0;
    int raw_val = (int)(tmp +0.5); 
    if ( raw_val> 1023) raw_val = 1023;
    set_pwm1_raw(raw_val);
}
void set_pwm1_voltage(float value)
{
    float tmp = value*1023.0/5.0; 
    int raw_val = (int)(tmp +0.5); 
    if ( raw_val> 1023) raw_val = 1023; 
    set_pwm1_raw(raw_val);
    
}
void set_pwm1_general(float value, float min, float max)
{
    float tmp = (value - min)*1023.0/(max - min); 
    int raw_val = (int)(tmp +0.5);
    if ( raw_val> 1023) raw_val = 1023;
    set_pwm1_raw(raw_val);
    
}


void initTimers01(void) {
    T0CON = 0;
    
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; 
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    T1CONbits.TMR1CS = 1; 
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1;
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}

void reloadTimer3(void) {
    TMR3H = 0x9E;
    TMR3L = 0x58;
    PIR2bits.TMR3IF = 0;
}

void initialization(void) {
    PIR1 = 0;
    PIR2 = 0;
    PIE2bits.TMR3IE = 0;
    T3CON = 0;
    PIE2 = 0;
    PIE2bits.TMR3IE = 1; 
    T3CONbits.TMR3ON = 1;
    RCONbits.IPEN = 0;
    INTCON3 = 0;
    INTCON3bits.INT1E = 1;
    INTCON3bits.INT2E = 1;
    INTCON2 = 0;
    INTCON = 0;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.INT0E = 1;
    INTCONbits.TMR0IE = 1;
    INTCONbits.INT0F = 0;
}

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; 
    TRISB = 0xFF; 
    TRISC = 0x80; 
    TRISA = 0xFF; 
    TRISD = 0x00;
    TRISE = 0x00;
}
void modes(void)
{
if (our_modes == 0) our_modes = 1;
    else if (our_modes == 1) our_modes = 2;
    else if (our_modes == 2) our_modes = 0;
    

}
void __interrupt(high_priority) highIsr(void)
{
    if (PIR2bits.TMR3IF) {
        C_ON=0;
        H_ON=0;
        
        if(our_modes ==1)
        {
            H_ON=1;
        }
        else if(our_modes ==2)
        {
        if (CoolError > 0)
            C_ON=1;
        
        if (roomT < (sp - HS))
            H_ON=1;
        }
        else if(our_modes ==0)
        {
                    H_ON=0;
                    C_ON=0;
                 PORTCbits.RC2 = 0;
               PORTCbits.RC5 = 0; 



        }
        
        reloadTimer3();
       
        
    } 
    //RB0
    else if (INTCONbits.INT0IF) { 
        INTCONbits.INT0IF = 0;
        modes();
    }
    //RB1
    else if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
            if (HS != 0) {
                HS = (HS - 1) % 5;
            }
        delay_ms(100);
        
    } 
    //RB2
         else if (INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF = 0;

            if (HS != 5) {
                HS = (HS + 1) % 5;
            }
    } 
    //RB3
  
    //TIMER
         else if (INTCONbits.T0IF) {
        RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); 

        TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
        TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
        TMR1H = 0;
        TMR1L = 0;
        INTCONbits.T0IF = 0;
    }
}

void pwd_Timer3(int limit) {

    cTIMERTHREE = (cTIMERTHREE) % 5;
    if ((cTIMERTHREE <= limit) && (limit != 0)) {
        PORTCbits.RC5 = 1;
        H_ON = true;
        cTIMERTHREE++;

    } else {
        PORTCbits.RC5 = 0;
        cTIMERTHREE++;
    }
    if (limit == 0) {
        H_ON = false;
    }

}

void main(void) {
    char Buffer[32];
    float analogs[3];
    float voltage;
    initTimers01();
    setupPorts();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    reloadTimer3();
    initialization();
    lcd_putc('\f');
    TRISCbits.RC0 = 1; 
  
    

     
     
     
    while (1) {
        CLRWDT();
        
        
        if (PORTBbits.RB3 == 0)
         {
            our_modes=0;
            PORTCbits.RC2 = 0; 
                 PORTCbits.RC5 = 0; 
                 HS=0;  
                lcd_gotoxy(1, 4);
                sprintf(Buffer, "MD: OFF        ");
                lcd_puts(Buffer);
         }
        for (unsigned char channel = 0; channel < 3; channel++) {
            voltage = read_adc_voltage((unsigned char) channel);
            analogs[channel] = voltage * 100;
        }
        sp = analogs[0] / 5;
        roomT = analogs[2];
        lcd_gotoxy(1, 1);
        sprintf(Buffer, "RT:%03.02fC    H C", roomT);
        lcd_puts(Buffer);
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "SP:%03.02fC    ", sp);
        lcd_puts(Buffer);
                lcd_gotoxy(1, 3);
                sprintf(Buffer, "  HS=%d", HS);
        lcd_puts(Buffer);

        

        lcd_gotoxy(14, 2);
        if (H_ON)
            lcd_putc('Y');
        else
            lcd_putc('N');
        lcd_gotoxy(16, 2);
        
        if (C_ON)
            lcd_putc('Y');
        else
            lcd_putc('N');

        switch (our_modes) {
            case 1:
                C_ON = false;
                H_ON=true;
                PORTCbits.RC2 = 0; // fan off
                 PORTCbits.RC5 = 1; // heater on

                
                set_pwm1_raw(0);
                
                lcd_gotoxy(1, 4);
                sprintf(Buffer, "MD: Heat        ");
                lcd_puts(Buffer);
                break;

            case 2:
                  
                
                    lcd_gotoxy(1, 4);
                    sprintf(Buffer, "MD:ACooL");
                    lcd_puts(Buffer);
                     CoolError = roomT - sp;
                    if (CoolError > 0) {
                        int PWMPersent = CoolError * 10;
                        if (PWMPersent < 25) {
                            set_pwm1_percent(25);
                        } else if (PWMPersent > 100) {
                            set_pwm1_percent(100);
                        } else {
                            set_pwm1_percent(PWMPersent);
                        }
                        C_ON = true;
                        pwd_Timer3(0);

                    }
                    if (roomT < (sp - HS)) {
                        pwd_Timer3(4);

                        C_ON = false;
                        set_pwm1_percent(0);
                    }
                     break;
                     
            case 0:
                
                PORTCbits.RC2 = 0; // fan off
                 PORTCbits.RC5 = 0; // heater off
                 HS=0;  
                lcd_gotoxy(1, 4);
                sprintf(Buffer, "MD: OFF        ");
                lcd_puts(Buffer);
                break;
        }
        
    }
    return;
}