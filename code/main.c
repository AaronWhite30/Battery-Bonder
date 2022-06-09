#include <avr/io.h>
#include <stdio.h>

#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "LCD.h"

#define portbPinChangeInterrupt PCIE0
#define portdPinChangeInterrupt PCIE2

#define upSwitch PINB0
#define downSwitch PIND0
#define pulseSwitch PIND2

#define pulseOut PINB1

#define powerLED PIND7
#define powerGate PIND1
#define powerSwitch PIND3

int increaseRequest = 0;
int decreaseRequest = 0;
int pulseRequest = 0;

int clockCycleMaxPosition = 20;
int clockCycleMinPosition = 1;
int clockCyclePosition = 1;

static const int clockCycleTicks[20] =
{80, 158, 236, 314, 392, 470, 548, 626, 704, 782, 860, 938, 1016, 1094, 1172, 1250, 1328, 1406, 1484, 1562};

static const char clockCycleMillisecondsDisplay[20][10] =
{"   5ms", "  10ms", "  15ms", "  20ms", "  25ms", "  30ms", "  35ms", "  40ms", "  45ms", "  50ms", "  55ms", 
	"  60ms", "  65ms", "  70ms", "  75ms", "  80ms", "  85ms", "  90ms", "  95ms", " 100ms"};
			
void setup(){
	
	DDRD |= (1<<powerLED) | (1<<powerGate);
	DDRD &= ~(1<<powerSwitch);
	
	//Turn on the gate to hold power right away
	PORTD |= (1<<powerGate) | (1<<powerLED);
	
	lcd_init();
	lcd_clear_screen();
	
	//Let's pause if and while the power
	//button is still being held down
	while(~PIND & (1<<powerSwitch)){}
			
	//Activate port B and D for pin change interrupts
	PCICR |= (1<<portbPinChangeInterrupt)|(1<<portdPinChangeInterrupt);
	PCMSK0 |= (1<<upSwitch);
	PCMSK2 |= (1<<downSwitch);
	
	//All switches inputs
	DDRB &= ~(1<<upSwitch);
	DDRD &= ~(1<<downSwitch);
	DDRD &= ~(1<<pulseSwitch);

	//Pulse pin direction out
	DDRB |= (1<<PINB1);
	
	EICRA |= (1<<ISC01);
	EIMSK |= (1<<INT0);
}

void set_pulse_width(int cycles){
	OCR1A = 0xffff-(cycles-1);
}

void pulse_setup(int cycles){
	
	TCCR1B = 0; //Timer/Counter control register. Set to 0 inactivating timer
	TCNT1 = 0x0000; //Timer/Counter. Set to 0, start count from the bottom
	
	//OCR1A is the MATCH register
	ICR1 = 0; //ICR1 is the TOP register
	
	set_pulse_width(cycles);

	//COM1A0 COM1A1 OC1A Set on Match, clear on BOTTOM.
	TCCR1A = (1<<COM1A0) | (1<<COM1A1) | (1<<WGM11);
	
	//WGM11 WGM12 WGM13 ICR1 = TOP, mode 14 fast pwm
	//CS10 CS12 1024 prescaler
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS12); //Set TCCR1B to begin counting
}

ISR(INT0_vect){
	if(1<<pulseSwitch){
		pulseRequest = 1;
	}
}

ISR(PCINT0_vect){
	if(PINB & 1<<upSwitch){
		increaseRequest = 1;
	}
}

ISR(PCINT2_vect){
	if(PIND & 1<<downSwitch){
		decreaseRequest = 1;
	}
}

void updateEepromClockCyclePosition(){
	
	while(!eeprom_is_ready()){}
	eeprom_update_word((uint16_t*)11, clockCyclePosition);
}

void loadEepromClockCyclePosition(){
	
	while(!eeprom_is_ready()){}
	clockCyclePosition = eeprom_read_word((uint16_t*)11);
	
	if((clockCyclePosition > clockCycleMaxPosition) | (clockCyclePosition < clockCycleMinPosition)){
		clockCyclePosition = clockCycleMinPosition;
		while(!eeprom_is_ready()){}
		updateEepromClockCyclePosition();
	}
}

void pulse_set_and_fire(int cycles) {
	
	int pulseWidth = 0xffff-(cycles-1);
	OCR1A = pulseWidth;
	
	_delay_us(65);
	
	TCNT1 = pulseWidth-1;
}

void pulse_send(int cycles){
	pulse_set_and_fire(cycles);
}

void pollPowerSwitch(){
	
	//Has our power button been pressed?
	if(~PIND & (1<<powerSwitch)){
				
		//Let's pause if and while the power
		//button is still being held down
		while(~PIND & (1<<powerSwitch)){
					
			//Toggle LED at a faster rate to indicate shutdown
			PORTD ^= (1 << powerLED);
			_delay_ms(150);
		}
				
		//Release our power to shut down
		PORTD &= ~(1<<powerGate);
	}
}

void splash_screen(){
	
	lcd_clear_screen();
	
	lcd_set_cursor_1stLine();
	_delay_ms(3);
	lcd_display(" Battery");
	_delay_ms(3);
	lcd_set_cursor_2ndLine();
	_delay_ms(3);
	lcd_display(" Bonder");
	
	_delay_ms(2500);
	
	lcd_clear_screen();
}

void updateMainDisplay(){
	
	lcd_clear_screen();
	_delay_ms(3);
				
	lcd_set_cursor_1stLine();
	_delay_ms(3);
			
	lcd_display(clockCycleMillisecondsDisplay[clockCyclePosition-1]);
	_delay_ms(3);
			
	lcd_set_cursor_2ndLine();
	_delay_ms(3);
			
	lcd_display(" <    >");
	_delay_ms(3);
}

int main(){
	
	cli();
	
	setup();
	
	lcd_init();
	splash_screen();
	loadEepromClockCyclePosition();
	
	pulse_setup(clockCycleTicks[clockCyclePosition-1]);

	sei();
	
	updateMainDisplay();
	
	while(1)
	{
		
		if(increaseRequest){
			
			increaseRequest = 0;
			clockCyclePosition++;
			if(clockCyclePosition > clockCycleMaxPosition){
				clockCyclePosition = clockCycleMaxPosition;
			}
			
			updateEepromClockCyclePosition();
			
			updateMainDisplay();
					
		}else if(decreaseRequest){
			
			decreaseRequest = 0;
			clockCyclePosition--;
			if(clockCyclePosition < clockCycleMinPosition){
				clockCyclePosition = clockCycleMinPosition;
			}
			
			updateEepromClockCyclePosition();
			
			updateMainDisplay();
			
		}else if(pulseRequest){
			
			pulseRequest = 0;
			pulse_send(clockCycleTicks[clockCyclePosition-1]);
		}
		
		pollPowerSwitch();
		
		_delay_ms(100);
	}
	
}
