#define F_CPU 8000000
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
//#include <util/delay.h>

/***************MACROS***************/
//#define pwm_percent(dc_perc) dc_perc=((dc*255)/100)	//Macro para o valor de PWM em percentagem
#define TRIGGER_PIN1 /*PORTD|=(1<<PD2)*/PIND2	//Macro para o pino 0 do PORTD
#define ECHO_PIN1 /*PORTD|=(1<<PD3)*/ PIND3
#define TRIGGER_PIN2 PINC5
#define ECHO_PIN2 PINC4


/*************VARIABLES*******************/
unsigned int sonar_counter;
unsigned int intersect_counter;	//Counts the number of intersections in the path
volatile int cnt_led;
volatile int cnt=0;	//Counter for the LED Interrupt(should include volatile because its value could change unexpectedly within the interruptio routine)
int sonar1_parameters, sonar2_parameters;	//Set the parameters for the sonar
int front_sensor, right_sensor, left_sensor;
unsigned int time_flag;

void init(void)
{
	//DDRB |= (1 << PB3);
	DDRB  |= (1<<PB0);
	DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC6) | (1 << PC7);			//H Bridge Connections
	DDRD |=0b11000000;
	PORTC = 0b10000010;
	PORTD=0b11000000;
	
	TCCR0A = (1 << WGM01); //timer em modo CTC
	TIMSK0 = (1 << OCIE0A); // interrupt enable on compare with register
	OCR0A = 78; // 0.01 segundos	
	TCCR0B =(1 << CS02) | (1 << CS00); //prescaler 1024	
	
	TCCR2A = 0b10100011;   //set compare A for non-inv. PWM with 8 bit resolution
	TCCR2B=0b00000110;    //Prescaler 32, f=1000Hz
	TIMSK2=(1<<OCIE2A)|(1<<OCIE2B);    //Enables Timer 2 on Compare Match

	/*ADMUX = 0b01000000;
	ADCSRA = 0b10000110;
	ADCSRA |= 0b01000000;*/

	OCR2A=0;
	OCR2B=0;
	
	sei();
}

ISR(TIMER0_COMPA_vect)    //Interrup��o do TIMER0
{
	cnt++;
	if (cnt==50)
	{
		cnt=0;	//Resets timer
		PORTB ^= (1 << PORTB0);	//Toggles the bit XOR
	}

}
void motores(int esq, int dir)
{
	PORTC |= (1 << PC1) | (1 << PC7);   
	PORTC &=  ~(1<< PC0) & ~(1<< PC6);
	
	OCR2A = esq;
	OCR2B = dir;
}


void tcrt_read(void)
{
	
		if(PIND==0b00001000)
		{
		
			motores(70,70);	//Enables both motors, so that the car moves forward
				
		} 
		
		if (PIND==0b00001100)	//Turning right
		{	
					while (PIND != 0b00000000)
					{
						motores(0,70);
					}
					motores (30,70);	//Turns right, enables only the left motor			
		}
				
		if(PIND==0b00011000)	//Turning left
		{
			while (PIND !=0b00000000)
			{
				motores(70,0);	//Turns left, enables only the right motor
			}
			motores (70,35);
						
		}
				
		if(PIND==0b00011100)	//Intersection
		{
			//intersect_counter++;	//Increments the counter
			motores(70,70);
		}		
		
	

		 if (PIND==0b00000100)       //Direita ligado
		 {
			 motores (0,70);
		 }
		 
        if (PIND==0b00010000)
		{
			motores(70,0);
		}
		if (PIND == 0b00010100) //Direito e esquerdo, sem centro
		{
			motores(35,70);
		}
		
		if (PIND==0b00000000)
        {
            motores(0,0);
		}
}

/*************SONAR**************/
int sonar_read1(void)		//Reads the distance for the first sonar, positioned at PIND2 and PIND3
{
	PORTD &=(0<<TRIGGER_PIN1);	//Emittes the sign for the trigger
	_delay_us(2);		//2ms delay
	PORTD |=(1<<TRIGGER_PIN1);	//Switches on the trigger
	_delay_us(10);
	PORTD &=(0<<TRIGGER_PIN1); //AND cause if you do OR it still gets the value 1
	while (!(PIND & (1<<ECHO_PIN1)));  //While the echo is with the value zero
	TCCR1B=0b00000001;			//Initiates timer
	while(PIND & (1<<TRIGGER_PIN1)); //Waits for the echo to receive the value 0
	TCCR1B = 0b00000000;	//Disables the timer
	
	sonar_counter=TCNT1;
	int distance_centimeters_sonar1;
	distance_centimeters_sonar1=(sonar_counter/58)/8;	//Stores the distance in centimeters
	
	TCNT1=0;	//Resets the timer
	
	return distance_centimeters_sonar1;
}

int sonar_read2(void)		//Reads the distance for the second sonar, positioned at PINC4 and PINC5
{
    PORTD &=(0<<TRIGGER_PIN2);	//Emittes the sign for the trigger
	_delay_us(2);		//2ms delay
	PORTD |=(1<<TRIGGER_PIN2);	//Switches on the trigger
	_delay_us(10);
	PORTD &=(0<<TRIGGER_PIN2); //AND cause if you do OR it still gets the value 1
	while (!(PIND & (1<<ECHO_PIN2)));  //While the echo is with the value zero
	TCCR1B=0b00000001;			//Initiates timer
	while(PIND & (1<<TRIGGER_PIN2)); //Waits for the echo to receive the value 0
	TCCR1B = 0b00000000;	//Disables the timer
	
	sonar_counter=TCNT1;
	int distance_centimeters_sonar2;
	distance_centimeters_sonar2=(sonar_counter/58)/8;	//Stores the distance in centimeters
	
	TCNT1=0;	//Resets the timer
	
	return distance_centimeters_sonar2;
}


void sonar_led(void)
{
	PORTB |= (1<<PB2);	//Turns the LED on
}


int sonar_distance_difference(void) //Or should it receive like this int sonar_distance_difference(ECHO_PIN1,ECHO_PIN2)
{
	sonar1_parameters=sonar_read1();
	sonar2_parameters=sonar_read2();
	int parking_distance=(sonar2_parameters)-(sonar1_parameters);
	
	return parking_distance;
}

int main(void)
{
	init();
    int sonar1_verifier, sonar2_verifier;	//Reads the distance from sonar 1 and sonar 2
	int parking_distance_sonars=sonar_distance_difference();	//Stores the distance between the two sonar to verify if the distance is proper to park
	

    while (1) 
    {
		tcrt_read();	
		//motores(0,0);
        sonar1_verifier=sonar_read1();
        sonar2_verifier=sonar_read2();
	
        /****************VERIFYING DISTANCE**************/
		if (sonar1_verifier<=10 || sonar2_verifier<=10)
		{
			sonar_led();	//Turns the led on if the distance read by the two sonars is less than 10 centimeters
			motores(0,0);	//Stops the car
		} 
		else
		{
			motores(100,100);	//Proceeds with normal speed
		}
		
		/****************PARKING********************/
		
		if (sonar1_verifier=0 && sonar2_verifier=0)	//If the sonar1 and sonar2 don't detect a obstacle
			continue;	//Proceeds to the next line
		else
		{
			if (parking_distance_sonars<=15)
			{
					motores(100,100);
			} 
			else
				{
					if (parking_distance_sonars>=15)
					{
						PORTC &= (0<<3)(1<<2)(0<<1)(1<<0);	//Alters the rotation sense of the wheels in order to go backwards
						motores(40,40);
						_delay_ms(1000);
						motores(0,0);
					}
				}
				
			}
		}
		
	
    
	
	return 0;
}

