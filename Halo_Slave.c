/*
    3/1/2011
    Copyright Glen Rhodes� 2008
    
    RGB LED Spotlight
    Project Halo
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "USI_TWI_Slave.h"

#define RED_LED_MASK    0x10   //PB4
#define GREEN_LED_MASK  0x08   //PB3
#define BLUE_LED_MASK   0x04   //PB2
#define RGB_MASK        0x1C
#define RGB_PORT        PORTB
#define RGB_DDR         DDRB
#define RED_PWM         OCR1BL
#define GREEN_PWM       OCR1AL
#define BLUE_PWM        OCR0A
#define VIOLET_PORT		PORTD
#define VIOLET_LED_MASK 0x20   //PD5
#define DEVICE_DDR		DDRD
#define DEVICE_PORT 	PORTD
#define DEVICE_PORTIN	PIND
#define DEVICE_MASK		0x0F  // PD0-3


#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define MESSAGEBUF_SIZE       8
//Define functions
//======================
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay


volatile uint16_t timer_1k ;


ISR(TIMER0_OVF_vect)
{
    TIFR = (1<<TOV0); //Clear any interrupt flags on Timer2
    timer_1k = timer_1k + 1;
   
}
// const char color_ramp[16] = {0,
//                             1,
//                             5,
//                             10,
//                             18,
//                             28,
//                             41,
//                             56,
//                             73,
//                             92,
//                             113,
//                             137,
//                             163,
//                             192,
//                             222,
//                             255};
// 
// const uint8_t color_ramp_log[16] = {0,
//                                     64,
//                                     101,
//                                     128,
//                                     148,
//                                     165,
//                                     179,
//                                     191,
//                                     202,
//                                     212,
//                                     221,
//                                     229,
//                                     236,
//                                     243,
//                                     249,
//                                     255};
const char rainbow[18][3]= {{0xf,0,0},
                             {0xf,0,0x5},
                             {0xf,0,0xa},
                             {0xf,0,0xf},
                             {0xa,0,0xf},
                             {0x5,0,0xf},
                             {0,0,0xf},
                             {0,0x5,0xf},
                             {0,0xa,0xf},
                             {0,0xf,0xf},
                             {0,0xf,0xa},
                             {0,0xf,0x5},
                             {0,0xf,0},
                             {0x5,0xf,0},
                             {0xa,0xf,0},
                             {0xf,0xf,0},
                             {0xf,0xa,0},
                             {0xf,0x5,0}};

//======================


void LED_ChangeColor(uint8_t red,uint8_t green,uint8_t blue,uint8_t rate_ms){
	int timeout;
	timeout = 1000;
   /* while(( (temp_green != green)  || (temp_red != red) || (temp_blue != blue)) && timeout > 0){
        if(temp_blue > blue){
            temp_blue --;
        } 
        if (temp_blue < blue) {
            temp_blue ++;
        }
        if(temp_green > green){
            temp_green --;
        }
         if (temp_green < green){
            temp_green ++ ;
        }
        if(temp_red>red){
            temp_red --;
        }
        if (temp_red < red) {
            temp_red ++;
        }
        if(temp_green){
            TCCR1A |= (1<<COM0A1);
            while(GREEN_PWM != temp_green){
                if(GREEN_PWM > temp_green)
                    GREEN_PWM --;
                if(GREEN_PWM < temp_green)
                    GREEN_PWM ++;
            }
            
        }else {
            TCCR1A &= ~(1<<COM0A1);
        }
        if(temp_red){
            TCCR1A |= (1<<COM0B1);
            while(RED_PWM != temp_red){
                if(RED_PWM > temp_red)
                    RED_PWM --;
                if(RED_PWM < temp_red)
                    RED_PWM ++;
            }
            
        }else {
            TCCR1A &= ~(1<<COM0B1);
        }
        if(temp_blue){
            TCCR0A |= (1<<COM0A1);
            while(BLUE_PWM != temp_blue){
                if(BLUE_PWM > temp_blue)
                    BLUE_PWM --;
                if(BLUE_PWM < temp_blue)
                    BLUE_PWM ++;
            }
            
        }else {
            TCCR0A &= ~(1<<COM0A1);
        }

        delay_ms(rate_ms);
		timeout --;
    }
        */
	if(red){
		TCCR1A |= (1<<COM0B1);
		RED_PWM = red;
	}else{
		TCCR1A &= ~(1<<COM0B1);
	}
	if(green){
		TCCR1A |= (1<<COM0A1);
		GREEN_PWM = green;
	}else{
		TCCR1A &= ~(1<<COM0A1);	
	}
	if(blue){
		TCCR0A |= (1<<COM0A1);
		BLUE_PWM = blue;
	}else{
		TCCR0A &= ~(1<<COM0A1);	
	}


    

}

uint8_t checksum(uint8_t * array,int size){
  uint8_t i;
  uint8_t sum;
  
  sum = 0;
  for(i=0;i<size; i++){
    sum += array[i];
  }
  return sum;
}

 uint16_t color = 0;
int main (void)
{
    uint8_t TWI_slaveAddress,count;
    uint8_t messageBuf[MESSAGEBUF_SIZE];
    uint8_t rx_red, rx_green, rx_blue, rx_violet;
	unsigned char check_sum,command,color;
	//int rgb_value = 0;
    timer_1k = 0;
   
    ioinit();
    // Own TWI slave address
    TWI_slaveAddress = 0x8;
	TWI_slaveAddress = DEVICE_PORTIN & DEVICE_MASK;
	
    USI_TWI_Slave_Initialise( TWI_slaveAddress );
    while(USI_TWI_Data_In_Receive_Buffer()){
		USI_TWI_Receive_Byte();
	}
	LED_ChangeColor((0xff),(0xff),(0xff),50);
	delay_ms(1000);
	LED_ChangeColor((0xff),(0x00),(0xff),50);
	VIOLET_PORT |= VIOLET_LED_MASK;
	RGB_DDR &= ~(RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK);
	// delay_ms(250);
	RGB_DDR |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
	delay_ms(1000);
	VIOLET_PORT &= ~VIOLET_LED_MASK;
	LED_ChangeColor((0xff),(0x00),(0x00),50);
	
	color = 0;
	LED_ChangeColor(255,00,00,50);
	delay_ms(1000);
	LED_ChangeColor(0,255,00,50);
	delay_ms(1000);
	LED_ChangeColor(0,00,255,50);
	delay_ms(1000);
	LED_ChangeColor(0,00,0,50);
	// while(1){
	// 	
	// 	// LED_ChangeColor(255,255,255,50);
	// 	// 		delay_ms(500);
	// 	
	// 	color = DEVICE_PORT & DEVICE_MASK;
	// 	LED_ChangeColor((rainbow[color][0]*16),(rainbow[color][1]*16),(rainbow[color][2]*16),50);
	// 	delay_ms(200);
	// 	
	// }
    while(1)
	{
		
        
        //while(count < 4){
		command = USI_TWI_Receive_Byte();
            if (command == 0xaf )
            {
				messageBuf[4] =0;
				messageBuf[0] = 0xaf;
                messageBuf[1] = USI_TWI_Receive_Byte();
				messageBuf[2] = USI_TWI_Receive_Byte();
				messageBuf[3] = USI_TWI_Receive_Byte();
				messageBuf[4] = USI_TWI_Receive_Byte();
				messageBuf[5] = USI_TWI_Receive_Byte();
            	while(USI_TWI_Data_In_Receive_Buffer()){
             						USI_TWI_Receive_Byte();
             	}
				rx_red = messageBuf[1] ;
		        rx_green = messageBuf[2];
		        rx_blue  = messageBuf[3] ;
		        rx_violet = messageBuf[4];
				//check_sum = messageBuf[5];
				//check_sum = checksum(messageBuf,5);
				//USI_TWI_Transmit_Byte(check_sum);
		       // if(messageBuf[5] == check_sum){
		        	LED_ChangeColor(rx_blue,rx_red,rx_green,0);
					if(rx_violet){
						VIOLET_PORT |= VIOLET_LED_MASK;
					}else {
						VIOLET_PORT &= ~VIOLET_LED_MASK;
					}
				//}// else {
					// LED_ChangeColor((0xf),(0xF),(0xf),50);
					// 				delay_ms(250);
					// 				LED_ChangeColor((0x0),(0x0),(0x0),50);
					// 				delay_ms(250);
					// 
					// 			}   
            }else if(command == 0xae){
				while(USI_TWI_Data_In_Receive_Buffer()){
													USI_TWI_Receive_Byte();
												}
				USI_TWI_Transmit_Byte(rx_red);
				USI_TWI_Transmit_Byte(rx_green);
				USI_TWI_Transmit_Byte(rx_blue);
			}
        	
	        
       
        /*LED_ChangeColor(0x0,0xF,0,50);
        delay_ms(1000);
        LED_ChangeColor(0xF,0,0,50);
        delay_ms(1000);
        LED_ChangeColor(0,0,0xf,50);
       */
       
		//asm volatile ("sleep");
		//Sleep until a button wakes us up on interrupt
	}
	
    return(0);
}

void ioinit (void)
{
	//1 = Output, 0 = Input
	RGB_DDR |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    RGB_PORT = 0;
	DDRD |= VIOLET_LED_MASK;
    
    VIOLET_PORT &= ~VIOLET_LED_MASK; 

	//Look at page 48 of the datasheet for more info on this
	DEVICE_DDR &= ~DEVICE_MASK; //Make Device bits input
	DEVICE_PORT |= DEVICE_MASK; //Make device bits input
	MCUCR &= ~(0x80); //Enable internal pull up

	CLKPR = 0x80;
    CLKPR = 2;
    //Init Timer0 for delay_us
    TCCR0A |= (1<<COM0A1) /*|(1<<COM0B1)*/ |(1<<WGM01) |(1<<WGM00);
    TCCR0B = (1<<CS01);//(1<<CS02)|(1<<CS00); //Set Prescaler to No Prescaling (assume we are running at internal 1MHz). CS00=1 
    TIMSK = (1<<TOIE0); //enable Timer Overflow interrupt
    
    TCCR1A |= (1<<COM0A1) |(1<<COM0B1) |(1<<WGM00);
    TCCR1B |= (1<<WGM12)| (1<<CS01);
    OCR0A = 0x0;//rainbow[0][0];
    OCR1AL = 0;//rainbow[0][1];
    OCR1BL = 0;//rainbow[0][2];
    
    //WDTCSR = (1<<WDIE) | 0x6; //21 represents Bits 5, 2:0 – WDP3:0= 0x9 = 8seconds(page47 of UM)
	sei(); //Enable interrupts
}

//General short delays
void delay_ms(uint16_t x)
{
	uint16_t temp;
    temp= timer_1k + x;
        
    while (timer_1k!=temp) {}
}

