/*
 Servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 Copyright (c) 2009 Michael Margolis.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* 
 
 A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 The servos are pulsed in the background using the value most recently written using the write() method
 
 Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 
 The methods are:
 
 Servo - Class for manipulating servo motors connected to Arduino pins.
 
 attach(pin )  - Attaches a servo motor to an i/o pin.
 attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
 default min is 544, max is 2400  
 
 write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
 writeMicroseconds() - Sets the servo pulse width in microseconds 
 read()      - Gets the last written servo pulse width as an angle between 0 and 180. 
 readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
 attached()  - Returns true if there is a servo attached. 
 detach()    - Stops an attached servos from pulsing its i/o pin. 
 
*/

#if defined(__AVR__)

#include <avr/interrupt.h>
#include <Arduino.h> 

#include "Servo.h"

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

//#define NBR_TIMERS        (MAX_SERVOS / SERVOS_PER_TIMER)

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached servos


// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  if( Channel[timer] < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
  else{
    if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true )  
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,LOW); // pulse this channel low if activated   
  }

  Channel[timer]++;    // increment to the next channel
  if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
    *OCRnA = *TCNTn + SERVO(timer,Channel[timer]).ticks;
    if(SERVO(timer,Channel[timer]).Pin.isActive == true)     // check if activated
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high   
  }  
  else { 
    // finished all channels so wait for the refresh period to expire before starting over 
    if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);  
    else 
      *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
    Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino 
#if defined(_useTimer1)
SIGNAL (TIMER1_COMPA_vect) 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif

#if defined(_useTimer3)
SIGNAL (TIMER3_COMPA_vect) 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif

#if defined(_useTimer4)
SIGNAL (TIMER4_COMPA_vect) 
{
  handle_interrupts(_timer4, &TCNT4, &OCR4A); 
}
#endif

#if defined(_useTimer5)
SIGNAL (TIMER5_COMPA_vect) 
{
  handle_interrupts(_timer5, &TCNT5, &OCR5A); 
}
#endif

#elif defined WIRING
// Interrupt handlers for Wiring 
#if defined(_useTimer1)
void Timer1Service() 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif
#if defined(_useTimer3)
void Timer3Service() 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{  
#if defined (_useTimer1)
  if(timer == _timer1) {
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1 = 0;              // clear the timer count 
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF1A);      // clear any pending interrupts; 
    TIMSK |=  _BV(OCIE1A) ;  // enable the output compare interrupt  
#else
    // here if not ATmega8 or ATmega128
    TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt 
#endif    
#if defined(WIRING)       
    timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service); 
#endif	
  } 
#endif  

#if defined (_useTimer3)
  if(timer == _timer3) {
    TCCR3A = 0;             // normal counting mode 
    TCCR3B = _BV(CS31);     // set prescaler of 8  
    TCNT3 = 0;              // clear the timer count 
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);     // clear any pending interrupts;   
	ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt     
#else  
    TIFR3 = _BV(OCF3A);     // clear any pending interrupts; 
    TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt      
#endif
#if defined(WIRING)    
    timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only	
#endif  
  }
#endif

#if defined (_useTimer4)
  if(timer == _timer4) {
    TCCR4A = 0;             // normal counting mode 
    TCCR4B = _BV(CS41);     // set prescaler of 8  
    TCNT4 = 0;              // clear the timer count 
    TIFR4 = _BV(OCF4A);     // clear any pending interrupts; 
    TIMSK4 =  _BV(OCIE4A) ; // enable the output compare interrupt
  }    
#endif

#if defined (_useTimer5)
  if(timer == _timer5) {
    TCCR5A = 0;             // normal counting mode 
    TCCR5B = _BV(CS51);     // set prescaler of 8  
    TCNT5 = 0;              // clear the timer count 
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts; 
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt      
  }
#endif
} 

static void finISR(timer16_Sequence_t timer)
{
    //disable use of the given timer
#if defined WIRING   // Wiring
  if(timer == _timer1) {
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK1 &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
    #else 
    TIMSK &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt   
    #endif
    timerDetach(TIMER1OUTCOMPAREA_INT); 
  }
  else if(timer == _timer3) {     
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #else
    ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #endif
    timerDetach(TIMER3OUTCOMPAREA_INT);
  }
#else
    //For arduino - in future: call here to a currently undefined function to reset the timer
#endif
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}


/****************** end of static functions ******************************/

Servo::Servo()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos 
}

uint8_t Servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;  
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128 
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4; 
    // initialize the timer if it has not already been initialized 
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);    
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  } 
  return this->servoIndex ;
}

void Servo::detach()  
{
  servos[this->servoIndex].Pin.isActive = false;  
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void Servo::write(int value)
{  
  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());      
  }
  this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {  
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();   
    
  	value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    uint8_t oldSREG = SREG;
    cli();
    servos[channel].ticks = value;  
    SREG = oldSREG;   
  } 
}

int Servo::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);     
}

int Servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
  else 
    pulsewidth  = 0;

  return pulsewidth;   
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}



#elif defined(__arm__) && (defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))
// ******************************************************************************
// Teensy 3.0 implementation, using Programmable Delay Block
// ******************************************************************************

#include <Arduino.h> 
#include "Servo.h"

#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_CONT | PDB_SC_PRESCALER(2) | PDB_SC_MULT(0))
#define PDB_PRESCALE 4 // 
#define usToTicks(us)    ((us) * (F_BUS / 1000) / PDB_PRESCALE / 1000)
#define ticksToUs(ticks) ((ticks) * PDB_PRESCALE * 1000 / (F_BUS / 1000))

#if SERVOS_PER_TIMER <= 16
static uint16_t servo_active_mask = 0;
static uint16_t servo_allocated_mask = 0;
#else
static uint32_t servo_active_mask = 0;
static uint32_t servo_allocated_mask = 0;
#endif
static uint8_t servo_pin[MAX_SERVOS];
static uint16_t servo_ticks[MAX_SERVOS];

Servo::Servo()
{
	uint16_t mask;

	servoIndex = 0;
	for (mask=1; mask < (1<<MAX_SERVOS); mask <<= 1) {
		if (!(servo_allocated_mask & mask)) {
			servo_allocated_mask |= mask;
			servo_active_mask &= ~mask;
			return;
		}
		servoIndex++;
	}
	servoIndex = INVALID_SERVO;
}

uint8_t Servo::attach(int pin)
{
	return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int minimum, int maximum)
{
	if (servoIndex < MAX_SERVOS) {
		pinMode(pin, OUTPUT);
		servo_pin[servoIndex] = pin;
		servo_ticks[servoIndex] = usToTicks(DEFAULT_PULSE_WIDTH);
		servo_active_mask |= (1<<servoIndex);
		min_ticks = usToTicks(minimum);
		max_ticks = usToTicks(maximum);
		if (!(SIM_SCGC6 & SIM_SCGC6_PDB)) {
			SIM_SCGC6 |= SIM_SCGC6_PDB; // TODO: use bitband for atomic bitset
			PDB0_MOD = 0xFFFF;
			PDB0_CNT = 0;
			PDB0_IDLY = 0;
			PDB0_SC = PDB_CONFIG;
			// TODO: maybe this should be a higher priority than most
			// other interrupts (init all to some default?)
			PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
			NVIC_SET_PRIORITY(IRQ_PDB, 32);
		}
		NVIC_ENABLE_IRQ(IRQ_PDB);
	}
	return servoIndex;
}

void Servo::detach()  
{
	if (servoIndex >= MAX_SERVOS) return;
	servo_active_mask &= ~(1<<servoIndex);
	servo_allocated_mask &= ~(1<<servoIndex);
	if (servo_active_mask == 0) {
		NVIC_DISABLE_IRQ(IRQ_PDB);
	}
}

void Servo::write(int value)
{
	if (servoIndex >= MAX_SERVOS) return;
	if (value >= MIN_PULSE_WIDTH) {
		writeMicroseconds(value);
		return;
	} else if (value > 180) {
		value = 180;
	} else if (value < 0) {
		value = 0;
	}
	if (servoIndex >= MAX_SERVOS) return;
	servo_ticks[servoIndex] = map(value, 0, 180, min_ticks, max_ticks);
}

void Servo::writeMicroseconds(int value)
{
	value = usToTicks(value);
	if (value < min_ticks) {
		value = min_ticks;
	} else if (value > max_ticks) {
		value = max_ticks;
	}
	if (servoIndex >= MAX_SERVOS) return;
	servo_ticks[servoIndex] = value;
}

int Servo::read() // return the value as degrees
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return map(servo_ticks[servoIndex], min_ticks, max_ticks, 0, 180);     
}

int Servo::readMicroseconds()
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return ticksToUs(servo_ticks[servoIndex]);
}

bool Servo::attached()
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return servo_active_mask & (1<<servoIndex);
}

extern "C" void pdb_isr(void)
{
	static int8_t channel=0, channel_high=MAX_SERVOS;
	static uint32_t tick_accum=0;
	uint32_t ticks;
	int32_t wait_ticks;

	// first, if any channel was left high from the previous
	// run, now is the time to shut it off
	if (servo_active_mask & (1<<channel_high)) {
		digitalWrite(servo_pin[channel_high], LOW);
		channel_high = MAX_SERVOS;
	}
	// search for the next channel to turn on
	while (channel < MAX_SERVOS) {
		if (servo_active_mask & (1<<channel)) {
			digitalWrite(servo_pin[channel], HIGH);
			channel_high = channel;
			ticks = servo_ticks[channel];
			tick_accum += ticks;
			PDB0_IDLY += ticks;
			PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
			channel++;
			return;
		}
		channel++;
	}
	// when all channels have output, wait for the
	// minimum refresh interval
	wait_ticks = usToTicks(REFRESH_INTERVAL) - tick_accum;
	if (wait_ticks < usToTicks(100)) wait_ticks = usToTicks(100);
	else if (wait_ticks > 60000) wait_ticks = 60000;
	tick_accum += wait_ticks;
	PDB0_IDLY += wait_ticks;
	PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
	// if this wait is enough to satisfy the refresh
	// interval, next time begin again at channel zero 
	if (tick_accum >= usToTicks(REFRESH_INTERVAL)) {
		tick_accum = 0;
		channel = 0;
	}
}



#elif defined(__arm__) && defined(__MKL26Z64__)
// ******************************************************************************
// Teensy-LC implementation, using Low Power Timer
// ******************************************************************************

#include <Arduino.h> 
#include "Servo.h"

#define LPTMR_CONFIG     LPTMR_CSR_TIE | LPTMR_CSR_TFC | LPTMR_CSR_TEN
#define usToTicks(us)    ((us) * 8)
#define ticksToUs(ticks) ((ticks) / 8)

#if SERVOS_PER_TIMER <= 16
static uint16_t servo_active_mask = 0;
static uint16_t servo_allocated_mask = 0;
#else
static uint32_t servo_active_mask = 0;
static uint32_t servo_allocated_mask = 0;
#endif
static uint8_t servo_pin[MAX_SERVOS];
static uint16_t servo_ticks[MAX_SERVOS];

Servo::Servo()
{
	uint16_t mask;

	servoIndex = 0;
	for (mask=1; mask < (1<<MAX_SERVOS); mask <<= 1) {
		if (!(servo_allocated_mask & mask)) {
			servo_allocated_mask |= mask;
			servo_active_mask &= ~mask;
			return;
		}
		servoIndex++;
	}
	servoIndex = INVALID_SERVO;
}

uint8_t Servo::attach(int pin)
{
	return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int minimum, int maximum)
{
	if (servoIndex < MAX_SERVOS) {
		pinMode(pin, OUTPUT);
		servo_pin[servoIndex] = pin;
		servo_ticks[servoIndex] = usToTicks(DEFAULT_PULSE_WIDTH);
		servo_active_mask |= (1<<servoIndex);
		min_ticks = usToTicks(minimum);
		max_ticks = usToTicks(maximum);
		if (!(SIM_SCGC5 & SIM_SCGC5_LPTIMER)) {
			SIM_SCGC5 |= SIM_SCGC5_LPTIMER; // TODO: use BME
			OSC0_CR |= OSC_ERCLKEN;
			LPTMR0_CSR = 0;
			LPTMR0_PSR = LPTMR_PSR_PRESCALE(0) | LPTMR_PSR_PCS(3); // 8 MHz
			LPTMR0_CMR = 1;
			LPTMR0_CSR = LPTMR_CONFIG;
			NVIC_SET_PRIORITY(IRQ_LPTMR, 32);
		}
		NVIC_ENABLE_IRQ(IRQ_LPTMR);
	}
	return servoIndex;
}

void Servo::detach()  
{
	if (servoIndex >= MAX_SERVOS) return;
	servo_active_mask &= ~(1<<servoIndex);
	servo_allocated_mask &= ~(1<<servoIndex);
	if (servo_active_mask == 0) {
		NVIC_DISABLE_IRQ(IRQ_LPTMR);
	}
}

void Servo::write(int value)
{
	if (servoIndex >= MAX_SERVOS) return;
	if (value >= MIN_PULSE_WIDTH) {
		writeMicroseconds(value);
		return;
	} else if (value > 180) {
		value = 180;
	} else if (value < 0) {
		value = 0;
	}
	if (servoIndex >= MAX_SERVOS) return;
	servo_ticks[servoIndex] = map(value, 0, 180, min_ticks, max_ticks);
}

void Servo::writeMicroseconds(int value)
{
	value = usToTicks(value);
	if (value < min_ticks) {
		value = min_ticks;
	} else if (value > max_ticks) {
		value = max_ticks;
	}
	if (servoIndex >= MAX_SERVOS) return;
	servo_ticks[servoIndex] = value;
}

int Servo::read() // return the value as degrees
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return map(servo_ticks[servoIndex], min_ticks, max_ticks, 0, 180);     
}

int Servo::readMicroseconds()
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return ticksToUs(servo_ticks[servoIndex]);
}

bool Servo::attached()
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return servo_active_mask & (1<<servoIndex);
}

void lptmr_isr(void)
{
	static int8_t channel=0, channel_high=MAX_SERVOS;
	static uint32_t tick_accum=0;
	uint32_t ticks;
	int32_t wait_ticks;

	// first, if any channel was left high from the previous
	// run, now is the time to shut it off
	if (servo_active_mask & (1<<channel_high)) {
		digitalWrite(servo_pin[channel_high], LOW);
		channel_high = MAX_SERVOS;
	}
	// search for the next channel to turn on
	while (channel < MAX_SERVOS) {
		if (servo_active_mask & (1<<channel)) {
			digitalWrite(servo_pin[channel], HIGH);
			channel_high = channel;
			ticks = servo_ticks[channel];
			tick_accum += ticks;
			LPTMR0_CMR += ticks;
			LPTMR0_CSR = LPTMR_CONFIG | LPTMR_CSR_TCF;
			channel++;
			return;
		}
		channel++;
	}
	// when all channels have output, wait for the
	// minimum refresh interval
	wait_ticks = usToTicks(REFRESH_INTERVAL) - tick_accum;
	if (wait_ticks < usToTicks(100)) wait_ticks = usToTicks(100);
	else if (wait_ticks > 60000) wait_ticks = 60000;
	tick_accum += wait_ticks;
	LPTMR0_CMR += wait_ticks;
	LPTMR0_CSR = LPTMR_CONFIG | LPTMR_CSR_TCF;
	// if this wait is enough to satisfy the refresh
	// interval, next time begin again at channel zero 
	if (tick_accum >= usToTicks(REFRESH_INTERVAL)) {
		tick_accum = 0;
		channel = 0;
	}
}









#endif
