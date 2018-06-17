/*
* simistor.cpp
*
* Created: 01.06.2018 15:46:13
* Author : Григорий
*/

#include <avr/io.h>
#include <avr/interrupt.h>

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

volatile unsigned int amperaj;
volatile unsigned char inData[2];
volatile bool inOk = false;

ISR(TIMER2_COMPA_vect) {
	if (((PORTB & (1<<PORTB1)) != 0) || ((PORTB & (1<<PORTB2)) != 0))
	{
		PORTB |= (1<<PORTB3);
	}
}

ISR (USART_RX_vect) {
	static unsigned char cntidt = 0;
	if (cntidt < 2)
	{
		inData[cntidt] = UDR0;
		cntidt++;
		if (cntidt == 2)
		{
			cntidt = 0;
			UCSR0B &= ~(1<<RXCIE0);
			inOk = true;
		}
	}
}

ISR (USART_UDRE_vect) {
	static unsigned char cntodt = 0;
	if (cntodt < 3)
	{
		switch (cntodt) {
			case 0: UDR0 = amperaj >> 8; break;
			case 1: UDR0 = amperaj; break;
			case 2: UDR0 = inData[1]; break;
		}
		cntodt++;
		if (cntodt == 3)
		{
			cntodt = 0;
			UCSR0B &= ~(1<<UDRIE0);
			UCSR0B |= (1<<RXCIE0);
		}
	}
}

ISR(ANALOG_COMP_vect) {
	TCNT2 = 0x00;
	PORTB &= ~(1<<PORTB3);
	OCR2A = inData[0];
}

ISR(ADC_vect) {
	static unsigned char i = 0;
	static unsigned int amp = 0;
	amp += ADCW;
	if ((i > 0) || (i < 19))
	{
		amp += ADCW;
	}
	if (i <= 19)
	{
		i++;
		if (i == 20)
		{
			amperaj = amp;
			amp = 0;
			i = 0;
		}
	}
}

ISR (TIMER0_COMPA_vect) {

}

inline void initPereph() {
	CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 250.000 kHz
// Mode: CTC top=OCR0A
// OC0A output: Disconnected
// OC0B output: Disconnected
// Timer Period: 1 ms
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
TCNT0=0x00;
OCR0A=0xF9;
OCR0B=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

// ADC initialization
// ADC Clock frequency: 125.000 kHz
// ADC Voltage Reference: AVCC pin
// ADC Auto Trigger Source: Timer0 Compare Match
// Digital input buffers on ADC0: Off, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On
DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (1<<ADC0D);
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
ADCSRB=(0<<ADTS2) | (1<<ADTS1) | (1<<ADTS0);

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: 15.625 kHz
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	// Timer Period: 16.384 ms
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
	TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20);
	TCNT2=0x00;
	OCR2A=0x2E;
	OCR2B=0x00;

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);

	// Analog Comparator initialization
	// Analog Comparator: On
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	// Interrupt on Output Toggle
	// Analog Comparator Input Capture by Timer/Counter 1: Off
	ACSR=(0<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (1<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	// Digital input buffer on AIN0: Off
	// Digital input buffer on AIN1: Off
	DIDR1=(1<<AIN0D) | (1<<AIN1D);

	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART0 Mode: Asynchronous
	// USART Baud Rate: 38400
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H=0x00;
	UBRR0L=0x19;

	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3);
}

int main(void)
{
	initPereph();
	sei();
	/* Replace with your application code */
	while (1)
	{
		if (inOk)
		{
			for (unsigned char i = 0; i < 3; i++)
			{
				PORTB = ((inData[1] & (1<<i)) != 0) ? PORTB | (1<<i) : PORTB & ~(1<<i);
			}
			UCSR0B |= (1<<UDRIE0);
			inOk = false;
		}
	}
}

