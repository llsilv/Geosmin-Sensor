/*
 * Exp_in_teams_2.0.c
 *
 * Created: 06.12.2021 12:03:36
 * Author : luis
 */ 

#define F_CPU 16000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "SPI.h"
#include "lcd.h" //for testing with LCD
#include "i2cmaster.h"	//for testing with LCD


#define dvdt 0.2
#define DAC_REF 5.0
#define DESIRED_RESOLUTION_uv 615
#define V_REF 5.0


// #define V_OFFSET_IMPRT 2.5722
// #define V_OFFSET_REF 2.5863

#define ten_mins_in_millis 600000
#define large_number 4147200000 //48 hours in millis

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)


#define map_max_geo 50
#define max_outout_value 4096
#define mapping_percentage ( max_outout_value / map_max_geo )


#define V_reso (DAC_REF / 65535.0)
#define step_size (DESIRED_RESOLUTION_uv / (V_reso * 1e6))
#define maxValue (1.0 / V_reso)
#define bits_per_second (dvdt / V_reso)
#define time_between_incr (step_size / (bits_per_second * 0.001))

#define scope_voltage 0.02
#define scope_area ((scope_voltage * 65535.0) / V_REF)
#define array_size (scope_area / (step_size * 2))

// 1 -> represents the orange chemistry graph
#define geo_fun_offset_1 0.0159
#define geo_fun_slope_1 4.7667
// 2 -> represents the teal chemistry graph
#define geo_fun_offset_2 -0.0834
#define geo_fun_slope_2 7.1114



//functions
unsigned long millis();
void output_voltage(uint16_t output_data);
void slope_increment(int* current_value, int increament);
unsigned int adc_value(unsigned char adc_channe);

float get_voltage();
float get_current_imprt();
float get_current_ref();

void get_data(int num_of_meas/*, float* impr, float* ref*/);

//variables
volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;
unsigned long ten_mins_timer = 0; 
unsigned long slope_timer = 0;

int slope_current_value = 0;
int slope_increment_value = 0;
char up_down_condition = 0;
char counter = 10;

//float impr_data_array[(int)number_of_measurments];
float non_impr_data_array[1];
float difference_data_array[1];
unsigned measurment_counter = 0;
float V_OFFSET_IMPRT = 0;
float V_OFFSET_REF = 0;


float average_geosmin = 0;
int main(void)
{
	
	int number_of_measurments = ((((scope_area * 2))/step_size) + 1);
	//float impr_data_array[number_of_measurments];
	//float ref_data_array[number_of_measurments];
	ADMUX = (1<<REFS0);//set prescaler to 128 and turn on the ADC module
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

	
	DDRB |= 0b00101111; // first 00 might be something
	DDRC |= 0b00110000; // first 00 might be something else
	DDRD |= 0b11100000; // enabling button 1 (on PD0)
	PORTD |= 0b10000000;
	_delay_ms(500);
	
	PORTB |= 0x07; // disabling both CS and CS2 and disabling clear on slop generator
	//PORTD |= 0x01; // enabling button 1 (on PD0)
	
	//ICR1 = 732; //counter TOP value
	TCCR0B |= 0x03; //Sets prescaler for timer0 to 64
	TIMSK0 |= 0x01; //Enables interupt on timer0 overflow
	sei();

	PORTD &= 0x01111111;
	PORTD |= 0b01000000;
	_delay_ms(500);
	SPI_Init();
	//i2c_init();
	//LCD_init();
	//LCD_clear();
	uint16_t test = 100;
	
	V_OFFSET_IMPRT = (float)adc_value(1) *(V_REF / 1024);
	V_OFFSET_REF = (float)adc_value(2) * (V_REF / 1024);
	
	//LCD_set_cursor(0,0);
	//printf("%d",number_of_measurments);
	PORTD &= 0b10111111;
	PORTD |= 0b00100000;
	_delay_ms(500);
	PORTD &=0b11011111;
    while (1) 
    {
		if (!(PIND & (1<<PIND0))) // generates slope if button is pressed
		{
			counter = 0;
			output_voltage(test);
			test += 100;

		unsigned long temp_ten_min_millis = millis();
		if (ten_mins_timer + ten_mins_in_millis >= temp_ten_min_millis) // generates slope after every 10 sec
		{
			ten_mins_timer += ten_mins_in_millis;
			counter = 0;
		}
		
		if (counter != 5)
		{

			unsigned long temp_millis = millis();
			if (slope_timer + time_between_incr < temp_millis)
			{
				slope_timer = temp_millis;
				if (!up_down_condition) 
				{
					PORTD |= 0b10000000;
					PORTD &= ~0b01000000;
					slope_increment(&slope_current_value, step_size);
					if (slope_current_value >= maxValue) up_down_condition = 1;
				}
				else if (up_down_condition)
				{
					PORTD |= 0b01000000;
					PORTD &= ~0b10000000;
					slope_increment(&slope_current_value, -step_size);
					if (slope_current_value == 0)
					{
						 up_down_condition = 0;
						 counter ++;
					}
				}
				if ((slope_current_value > (maxValue - scope_area))) 
				{
					//get_data(number_of_measurments, &impr_data_array[measurment_counter], &ref_data_array[measurment_counter]);
					get_data(number_of_measurments);
					measurment_counter++;
				}
				
				
			}
		}
		measurment_counter = 0;
		
		if (timer0_millis >= large_number) // resets after 48 hours
		{
			timer0_overflow_count = 0;
			timer0_millis = 0;
			timer0_fract = 0;
		}
    }
}
}
ISR(TIMER0_OVF_vect) // timer0 overflow interrupt
{
	timer0_millis += MILLIS_INC;
	timer0_fract += FRACT_INC;
	if (timer0_fract >= FRACT_MAX) {
		timer0_fract -= FRACT_MAX;
		timer0_millis += 1;
	}
	timer0_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	sei();
	return m;
}

void output_voltage(uint16_t output_data)
{
	SPCR &= ~(1<<CPHA);
	char spi_data[2] = {0,0};
		
	//if (output_data > 4096) output_data = 4096;
	
	spi_data[0] = 0b01110000;
	spi_data[0] |= (char)(output_data >> 8);
	spi_data[1] =  (char)(output_data); //& 0x00FF);
	PORTB &= 0b11111101;
	SPI_Send(spi_data[0], spi_data[1], 0, 2);
}

void slope_increment(int* current_value, int increament)
{
	SPCR |= (1<<CPHA);
	char spi_data[3] ={0,0,0};
	*current_value += increament;
	spi_data[0] = 0x40;
	spi_data[0] |= *current_value >> 10;
	spi_data[1] = *current_value >> 2;
	spi_data[2] |= *current_value << 6;
	PORTB &= 0xFE;
	SPI_Send(spi_data[0], spi_data[1], spi_data[2], 3);
}

unsigned int adc_value(unsigned char adc_pin)
{
	ADMUX &= 0xf0; //clearing prev. channel
	ADMUX |= adc_pin; //setting channel
	ADCSRA |= (1<<ADSC); //  ------------------------------------------------------------------
	while ((ADCSRA & (1<<ADSC)))
	{
		//idle time till conversion is completed
	};
	return ADC;
}

float get_voltage()
{
	unsigned int temp = adc_value(0);

	float voltage = (float)temp * (V_REF/1024);
	return voltage;
}

float get_current_imprt()
{
	unsigned int temp = adc_value(1);
	float voltage = (float)temp * (V_REF/1024);
	float Vreal = voltage - V_OFFSET_IMPRT;
	float Ireal = -Vreal *20.0;
	return Ireal;
}

float get_current_ref()
{
	unsigned int temp = adc_value(2);
	float voltage = (float)temp * (V_REF/1024);
	float Vreal = voltage - V_OFFSET_REF;
	float Ireal = -Vreal *20.0;
	return Ireal;	
}

	
void get_data(int num_of_meas /*,float* impr, float* ref*/)
{

	uint16_t mapped_average_geosmin = 0;
	float i_impr = 0;
	float i_ref = 0;
	
	i_impr = get_current_imprt();
	i_ref = get_current_ref();
	
	average_geosmin += geo_fun_slope_1 / ( (i_impr - i_ref) + geo_fun_offset_1);
	
	if (measurment_counter == num_of_meas)
	{
		average_geosmin /= num_of_meas;
		mapped_average_geosmin = (uint16_t)(average_geosmin * mapping_percentage);
		output_voltage(mapped_average_geosmin);
		average_geosmin = 0;
	}
}
	