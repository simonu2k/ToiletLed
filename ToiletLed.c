/*
 * ToiletLed.c
 *
 * Created: 2015/08/01 16:29:54
 *  Author: sk
 */ 

#define F_CPU 1000000UL  //1MHz
#include <avr/io.h>
#include <util/delay.h>			// to enable delay
#include <avr/interrupt.h>		// to enable sei() function. wake up on IR sensors's input
#include <avr/sleep.h>			// to enable sleep mode

//max pulse width at PWM(the value of OCR0B)
uint8_t PulsWidth = 250;
int DurationSec = 0;

void InitPWM(){
	TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);  // 非反転出力, 高速PWM動作
	TCCR0B = (1 << WGM02);
	TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00); // 64分周
	_delay_ms(10);
}

void stopPWM(){
	TCCR0A = 0;
	TCCR0B = 0;
	_delay_ms(10);
}

void keeplighting(){
	for(int i=0;i<DurationSec;i++){	_delay_ms(1000);}
};

void FadeOut(void) //Making LED Fading Out with PWM
{
	InitPWM();
	
	OCR0B = PulsWidth;	//duty 127/255 =0.5 -> 2.5v 
	int count = PulsWidth;
	
	while(count > 0){
		OCR0B = (uint8_t)count;
		count--;
		_delay_ms(7);   //12*127 = 1524ms 		
	}
	OCR0B = 0x00;
	
	PORTB &= ~(1 << PINB1); //turn off light
	stopPWM();
}

void FadeIn(void) //Making LED Fading In with PWM
{
	InitPWM();
	
	OCR0B = 0x00;
	int count = 0;
	
	while(count < PulsWidth){
		OCR0B  = (uint8_t)count;
		count++;
		_delay_ms(7);
	}
	OCR0B = 0xFF;
	
	PORTB |= (1 << PINB1); //turn on light
	stopPWM();
}

void WakeUp() {       // here the interrupt is handled after wakeup
	_delay_ms(10);
	FadeIn();
	keeplighting();
	FadeOut();
	_delay_ms(100);
}

void SleepIn() {
	sleep_mode();									// here the device is actually put to sleep!!
}

void SetLightDuration(){
	
	DDRB |= (1 << DDB4);  //PB4 as OutPut
	PORTB |= (1 << DDB4); //PB4 as High
	
	//8bit分解能で十分なのを前提に設定を行っている。
	//デジタル入出力からの切り離し
	
	

	DIDR0	= (1<<ADC3D);	//Enable ADC at PB3
	ADMUX	= (0<<REFS0)	//Reference: 0でVCC参照, 1で1.1V内部電圧源
			| (1<<ADLAR)	//ADC Left Adjust Result
			| (1<<MUX1)
			| (1<<MUX0);	//select channel with ADMUX1, ADMUX0 11 ->ADC3

	ADCSRA	= (1<<ADEN)		//ADC enable
			| (1<<ADSC)		//ADC Start Conversion at the  value of 1
			| (0<<ADATE)	//ADC Auto Trigger Enable (for Continuous Conversion)
			| (0<<ADIF)		//ADC Interrupt Flag	
			| (0<<ADIE)		//ADC Interrupt enable　
			| (0b100);		//Clock Division: ADコンバータはクロック50-200kHzで性能がよい

	//分周は0のとき2、それ以上の時2^n分周	1.2MHz(Default)のとき16分周(100)で75kHz。9.6MHzのとき128分周(111)で75kHz
	
	loop_until_bit_is_set(ADCSRA,ADIF);	//ADIFビットが1になるまでムダループ

	if (ADCH>220)	{DurationSec = 3;	}			//set duration 3 sec when the ADC result is more than 4.3V. 
	else if(ADCH>0){DurationSec = 255 - (int)ADCH;}	//less voltage gives more duration. less voltage means high register at volume.
		
	//clear ADC settings
	ADCSRA =0;
	ADMUX =0;
	DIDR0 =0;
	
	DDRB |= (0 << DDB4);  //Change PB4 as Input
	PORTB |= (0 << DDB4); //Change PB4 as low
	
	_delay_ms(10);
}

ISR(PCINT0_vect){ //interruption script here.
	//PCINT0_vect: description which pin detects an interruption
	//AVR IC skips another interruption during old one in progress
	WakeUp();
}

int main(void)
{
	//Setting for I/O
	// Set PB1 as OutPut (PWM出力OC０A)
	DDRB = (1 << DDB1);		//Set PB1 as Output and PB0 as Input
	PORTB= (1 << DDB0);		//enable pull-up at PINB0
	//Setting for PWM 
	InitPWM();
	OCR0A  = 0xFF;  //Set PWM Period
	//OCR0B  = 0x00;	//set PWM Puls width 
	//setting for sleepmode and interruption
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);			// Selecting a sleep mode
	//Setting for interruption
	GIMSK = (1 << PCIE); //Set GeneralInterruptMaskRegister
	PCMSK = (1 << PCINT0);
	//read ADC and determine the duration on lighting
	SetLightDuration();
	
	sei();	// start detecting interruption
	
	_delay_ms(10); 
		
	while(1)
	{
		if (PINB & (1<<PINB0)){
			_delay_ms(100);		
		}else{
			SleepIn();
		}
	}
}