// Arduino Steuerung für Akkuersatz BL03
//
// Copyright (C) 2023 by Rudolf Herzog
//
// Diese Programm soll alle benötigten Botschaften vom Ersatzakku an die TranzX Steuerung senden

#include "Arduino.h"
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/
#include <Wire.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define Version 2.10
// PINs
#define PWM_OUT 3
#define MON_WDT 9
#define MON_ADC 10
#define MON_REQ 12
#define MON_I2C 13
// Anzahl der Messungen für einen Wert
#define ANZ_MESSUNGEN 4

#define I2C_DEV_ADDR 11
#define BATTERY_MODE             0x03
#define TEMPERATURE              0x08
#define VOLTAGE                  0x09
#define CURRENT                  0x0A
#define AVERAGE_CURRENT          0x0B
#define MAX_ERROR                0x0C
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define REMAINING_CAPACITY       0x0F
#define FULL_CHARGE_CAPACITY     0x10
#define RUNTIME_TO_EMPTY         0x11
#define TIME_TO_EMPTY            0x12
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define DESIGN_CAPACITY          0x18
#define DESIGN_VOLTAGE           0x19
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define SERIAL_NUM               0x1C
#define MFG_NAME                 0x20   // String
#define DEV_NAME                 0x21   // String
#define CELL_CHEM                0x22   // String
#define MFG_DATA                 0x23   // String
#define AUTHENTICATE             0x2F   // String
#define CELL4_VOLTAGE            0x3C   //
#define CELL3_VOLTAGE            0x3D
#define CELL2_VOLTAGE            0x3E
#define CELL1_VOLTAGE            0x3F
#define AFE_DATA                 0x45   // String
#define FET_CONTROL              0x46   // 1
#define STATE_OF_HEALTH          0x4F
#define OPERATION_STATUS         0x54   // 1
#define CHARGING_STATUS          0x55   // 1
#define PACK_VOLTAGE             0x5A   // mV
#define AVERAGE_VOLTAGE          0x5D   // mV

// *** Variables ***
// Operating System
uint32_t ontime;
uint32_t receiveTimeStmp;
enum T_states {
	ST_NEW,
	ST_INIT,
	ST_NACHLAUF,
	ST_SHUTDOWN,
};
T_states BL03state;
// Aquisition
#define NoOfAquisitionCycles 4
uint8_t cycle; // NoOfAquisitionCycles ADCvalues are collected and an average value is generated
// Raw values
volatile uint16_t Current_In_Raw,Current_Out_Raw,Voltage12_Raw,Voltage24_Raw,Voltage36_Raw,VoltageINT_Raw;
// Scaling
// mA or mV @ ADC 1023
#define FAKTOR_CURRENT_IN 2414
#define FAKTOR_CURRENT_OUT 10735
#define FAKTOR_VOLTAGE_12 14514
#define FAKTOR_VOLTAGE_24 28776
#define FAKTOR_VOLTAGE_36 43234
#define FAKTOR_VOLTAGE_INT 52140
// Offsets
#define OFFSET_CURRENT_IN 28
#define OFFSET_CURRENT_OUT 12

// Scaled values
// Currents in mA
uint16_t Current_In,Current_Out;
int16_t Current;
// Voltages in mV
uint16_t Voltage12,Voltage24,Voltage36,VoltageINT;

// Capacity in mA * h
uint16_t Capacity;
// Capacity prescaler
int16_t Capacity_Low;
// Value for 1 mA*h = 3600 mA*s; but cycle time is 0,88s. Therefore the value is 3600/0,88 =
#define VALUE_1mAxh 4091
// max. capacity of battery is 10Ah = 10000mAh
#define MAX_CAPACITY 10000

// BatteryStatus
uint16_t BatteryStatus;
// Bits within BatteryStatus
#define BATT_STAT_DISCHARGING    0x040
#define BATT_STAT_INIT           0x080
#define REMAINING_CAPACITY_ALARM 0x200


// data buffer
#define BUF_SIZE 16
uint8_t data[BUF_SIZE];
// welche Daten wurden angefordert
uint8_t command = 0;
volatile uint8_t flag_receive = false;

volatile uint16_t adcReading;
volatile boolean adcDone;

/* Zeitscheiben */
unsigned long t_ADC;

// EEPROM
#define EEPADR_CAPACITY 0x100	// 2 Bytes

/* Nachlauf */
uint8_t ctr_Nachlauf_I2C, ctr_Nachlauf_NoCurrent;

/* function declarations */
void ReadAllADCValues(void);
uint16_t ADCwhileSlp(uint8_t);
uint16_t ScalePhysical(uint16_t adc_value, uint16_t factor, uint16_t offset);


void onRequest(){
	uint16_t Loc_temp;
	int16_t Loc_stemp;
	union {
		uint16_t wrd;
		uint8_t byt[2];
	} LocValue;
	  // Blaue LED Einschalten
	  digitalWrite (MON_REQ, HIGH);
	  pinMode (MON_REQ, INPUT);

	switch(command)
    {
  case 0:
	  // keine Daten angefordert
	  break;
  case TEMPERATURE: // Temperature
	Loc_temp = (273 + 10)*10;
    Wire.write((uint8_t)Loc_temp); // Low byte; vorerst mal dummy Wert ausgeben
    Wire.write((uint8_t)(Loc_temp >> 8)); // High byte;
	Serial << command << F(":") << Loc_temp << F("°C")  << endl;
   break;
  case VOLTAGE: // Voltage 36000/3,34
	Loc_temp = (uint16_t)(Voltage36/3);
    Wire.write((uint8_t)Loc_temp); // Low byte;
    Wire.write((uint8_t)(Loc_temp >> 8)); // High byte;
	Serial << command << F(":") << Loc_temp << F("mV ")  << endl;
    break;
  case CURRENT: // Current
	Loc_stemp = Current_Out-Current_In;
    Wire.write((uint8_t)(Loc_stemp & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
    Wire.write((uint8_t)((Loc_stemp >> 8) & 0xFF)); // High byte;
	Serial << command << F(":") << Loc_stemp << F("mA ")  << endl;
    break;
  case AVERAGE_CURRENT: // average Current
	Loc_stemp = Current_Out-Current_In;
    Wire.write((uint8_t)(Loc_stemp & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
    Wire.write((uint8_t)((Loc_stemp >> 8) & 0xFF)); // High byte;
	Serial << command << F(":") << Loc_stemp << F("mA ")  << endl;
    break;
  case RELATIVE_SOC: // Relative state of charge
	LocValue.wrd = Capacity / (MAX_CAPACITY / 100); // Scale to %
    Wire.write(LocValue.byt[0]); // Low byte;
    Wire.write(LocValue.byt[1]); // High byte; sollte immer 0
	Serial << command << F(":") << LocValue.wrd << F("% ")  << endl;
    break;
  case BATTERY_STATUS: // BatteryStatus(0x16)
	 // This read-word function returns the status of the bq20z80A-based battery.
	 //            bit 7 bit 6 bit 5 bit 4 bit 3 bit 2 bit 1 bit 0
	 // Low Byte   INIT  DSG   FC    FD    EC3   EC2   EC1   EC0
	 // High Byte  OCA   TCA   RSVD  OTA   TDA   RSVD  RCA   RTA
	LocValue.wrd = BatteryStatus;
    Wire.write(LocValue.byt[0]); // Low byte;
    Wire.write(LocValue.byt[1]); // High byte
	Serial << command << F(":") << LocValue.wrd << F(" ")  << endl;
    break;
  case MFG_NAME:
    Wire.write((uint8_t)8);
    Wire.print(F("MFG_NAME"));
    Serial.print((uint8_t)8);
    Serial.print(F(" MFG_NAME"));
    break;
  default:
    Wire.print(data[0]);
    Wire.print(data[1]);
	Serial << command << F(":") << F("not sup ")  << endl;
  break;
  }
  Serial.println();
	// Blaue LED ausschalten
	digitalWrite (MON_REQ, LOW);
	pinMode (MON_REQ, OUTPUT);
}

void onReceive(int len){
  uint8_t i=0;
  // monitor I2C high
  pinMode (MON_I2C, OUTPUT);
  digitalWrite (MON_I2C, HIGH);
  flag_receive = true;

  //Serial.print(F("onReceive[]: "));
  //Serial.print(len);
  while(Wire.available()){
    data[i] = Wire.read();
    //Serial.print(F(" "));
    //Serial.print(data[i]);
    // limit index to BUF_SIZE-1
    if(i<BUF_SIZE-1)
      i++;
  }
  command = data[0];
  i = EEPROM.read(command);
  if(i!=254) // nicht endlos draufschreiben; zähler begrenzen
  {
	  i++;
	 // EEPROM.write(command,i);
	  //Serial.print(i);
  }
  //Serial.println();
  // monitor I2C low
  digitalWrite (MON_I2C, LOW);
  pinMode (MON_I2C, INPUT);
}

ISR(TIMER2_COMPA_vect)
{
  static boolean state = false;

  state = !state;  // toggle
  //digitalWrite (LED_GN, state ? HIGH : LOW);
}
// watchdog interrupt
ISR (WDT_vect)
{
	wdt_disable();  // disable watchdog
	// no I2C message received - increment timeout counter
	ctr_Nachlauf_I2C++;
	// monitor Pin high
	pinMode (MON_WDT, OUTPUT);
	digitalWrite (MON_WDT, HIGH);

}  // end of WDT_vect
// ADC complete ISR
ISR (ADC_vect)
{
  byte low, high;

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low = ADCL;
  high = ADCH;

  adcReading = (high << 8) | low;
  adcDone = true;
}  // end of ADC_vect


void setup() {
  Serial.begin(115200);
//  Startmeldung ausgeben
  Serial << F("BL03Ersatz V");
  Serial.println((float)Version);
  Serial.print(F("Dev_Addr: "));
  Serial.println(I2C_DEV_ADDR);
  delay(10);

  //start time counter
  ontime = 0;
  ctr_Nachlauf_I2C = 0;
  ctr_Nachlauf_NoCurrent = 0;

  // Interrupt routinen anmelden
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // I2C starten
  Wire.begin((uint8_t)I2C_DEV_ADDR) ;

  //pinMode (LED_GN, OUTPUT);
  // initialize data aquisition
  BL03state = ST_NEW;
  cycle = 0;
  Current_In_Raw = 0;
  Current_Out_Raw = 0;
  Voltage12_Raw = 0;
  Voltage24_Raw = 0;
  Voltage36_Raw = 0;
  Capacity_Low = 0;
  BatteryStatus = 0;

  // initialize EEPROM values
  EEPROM.get(EEPADR_CAPACITY, Capacity);
  // check plausibility
  if((Capacity < 100)||(Capacity > 11000))
  {
	  // if unplausible set to 90%
	  Capacity = MAX_CAPACITY / 100 * 90;
  }


#if 0  // set up Timer 1
  TCCR1A = 0;          // normal operation
  TCCR1B = bit(WGM12) | bit(CS10) | bit (CS12);   // CTC, scale to clock / 1024
  OCR1A =  1999;       // compare A register value (2000 * clock speed / 1024)
  TIMSK1 = bit (OCIE1A);             // interrupt on Compare A Match

  // set up Timer 2
  TCCR2A = bit(WGM21) ;          // normal operation, CTC
  TCCR2B = bit(CS20) | bit(CS21) | bit (CS22);   // scale to clock / 1024
  OCR2A =  255;       // compare A register value (256 * clock speed / 1024)
  TIMSK2 = bit (OCIE2A);             // interrupt on Compare A Match
#endif
  // PWM on Cntr2
  pinMode (PWM_OUT, OUTPUT);
  pinMode (11, OUTPUT);

  TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1) | bit (COM2A1); // fast PWM, clear OC2B on compare, set at bottom
  TCCR2B = bit (WGM22) | bit (CS21);         // fast PWM top=OCR2A, prescaler of 1
  OCR2A =  ((F_CPU / 8) / 50000) - 1;        // zero relative 50kHz
  OCR2B = ((OCR2A + 1) / 5) - 1;             // 20% duty cycle

  // write first timestamp
  receiveTimeStmp = millis();
}

void loop() {
	unsigned long Loc_Millis; //
	// something received on the I²C bus?
	if (flag_receive == true)
	{
		flag_receive = false;
		// check if it was the first or second request within a cycle
		Loc_Millis = millis() - receiveTimeStmp; // time since last received message
		if (Loc_Millis < 100)
		{
			// repeated request received
			// do ADC conversion
			ReadAllADCValues();
			cycle++;
		}
		else
		{
			// first request received
		}
		receiveTimeStmp += Loc_Millis;

		// give some time to handle I²C requests (done in interrupt service routines)
		//delay(20);
		// Restart counter
		ctr_Nachlauf_I2C = 0;
	}
	else
	{
		// check receive timeout
		Loc_Millis = millis() - receiveTimeStmp; // time since last received message
		if (Loc_Millis > 1000)
		{
			// do ADC conversion
			ReadAllADCValues();
			cycle++;
			receiveTimeStmp += Loc_Millis;
			// count seconds without message received
			ctr_Nachlauf_I2C++;
		}
	}

	// all values collected?
	if (cycle > NoOfAquisitionCycles-1)
	{
		// physical values are calculated on the average of NoOfAquisitionCycles
		Current_In = ScalePhysical(Current_In_Raw, FAKTOR_CURRENT_IN, OFFSET_CURRENT_IN);
		Current_Out = ScalePhysical(Current_Out_Raw, FAKTOR_CURRENT_OUT, OFFSET_CURRENT_OUT);
		Current = Current_In - Current_Out;

		Voltage12 = ScalePhysical(Voltage12_Raw, FAKTOR_VOLTAGE_12, 0);
		Voltage24 = ScalePhysical(Voltage24_Raw, FAKTOR_VOLTAGE_24, 0);
		Voltage36 = ScalePhysical(Voltage36_Raw, FAKTOR_VOLTAGE_36, 0);
		VoltageINT = ScalePhysical(VoltageINT_Raw, FAKTOR_VOLTAGE_INT, 0);

		cycle = 0; // restart
		Current_In_Raw = 0;
		Current_Out_Raw = 0;
		Voltage12_Raw = 0;
		Voltage24_Raw = 0;
		Voltage36_Raw = 0;
		VoltageINT_Raw = 0;

		//Show values on the USB interface for debuggin g
		Serial << Current_In << F(" ") << Current_Out << F("mA ") << Capacity << F("mAh");
	//	Serial << Voltage12 << F(" ") << Voltage24 << F("mV ");
		Serial << Voltage36 << F(" ") << VoltageINT << F("mV ") << OCR2A << endl;
		delay (2); // warten bis gesendet

		// Calculate Capacity
		Capacity_Low += Current_In;
		Capacity_Low -= Current_Out;
		// Check if overflow
		// was more than 1mA*h recharged into the battery?
		while (Capacity_Low >= VALUE_1mAxh)
		{
			// Transfer 1mA*h to Capacity
			Capacity++;
			Capacity_Low -= VALUE_1mAxh;
		}
		// was more than 1ma*h consumed from the battery?
		while (Capacity_Low <= -VALUE_1mAxh)
		{
			// Transfer 1mA*h to Capacity
			Capacity--;
			Capacity_Low += VALUE_1mAxh;
		}

		// adjust PWM duty cycle for switched power supply
		// if internal voltage < 7V
		if(VoltageINT < 7000)
		{
			// is cycle time longer than pulse time+1
			if(OCR2A > OCR2B + 10)
			{	// enlarge duty cycle by reducing cycle time
				OCR2A--;
			}
		}
		// if internal voltage > 7V
		if(VoltageINT > 7000)
		{
			// is cycle time shorter than maximum
			if(OCR2A < 127)
			{	// reduce duty cycle by enlarging cycle time
				OCR2A++;
			}
		}

		// checks for nachlauf
		// bike motor on?
		if (Current_Out > 100)
		{
		  ctr_Nachlauf_NoCurrent = 0;
		}
		// charging?
		else if (Current_In > 50)
		{
		  ctr_Nachlauf_NoCurrent = 0;
		}
		else
		{
			// neither motoron nor charging - increase timeout counter
		  ctr_Nachlauf_NoCurrent++;
		}

		// set INIT bit
		BatteryStatus |= BATT_STAT_INIT;
		// set / reset CHARGING mode
		if (Current > 0)
		{
			// charging mode, clear discharging bit
			BatteryStatus &= ~BATT_STAT_DISCHARGING;
		}
		else
		{
			// set discharging bit
			BatteryStatus |= BATT_STAT_DISCHARGING;
		}
	    BL03state = ST_INIT;
	}

	  // monitor Pin low
	  digitalWrite (MON_WDT, LOW);
	  pinMode (MON_WDT, INPUT);

	  // *********************
	  // *** Prepare Sleep ***
	  // *********************
	  // according to http://www.gammon.com.au/power
	  // disable ADC
//	  ADCSRA = 0;

	  // clear various "reset" flags
	  MCUSR = 0;
	  // allow changes, disable reset
//	  WDTCSR = bit (WDCE) | bit (WDE);
	  // set interrupt mode and an interval
//	  WDTCSR = bit (WDIE) | bit (WDP2);    // set WDIE, and 1/4 second delay
	//  wdt_reset();  // pat the dog

	  // Power save mode keeps Timer/Counter2 running!
//	  set_sleep_mode (SLEEP_MODE_PWR_SAVE);
	  noInterrupts ();           // timed sequence follows
//	  sleep_enable();

	  // turn off brown-out enable in software
//	  MCUCR = bit (BODS) | bit (BODSE);
//	  MCUCR = bit (BODS);
	  interrupts ();             // guarantees next instruction executed
//	  sleep_cpu ();

	  // *** after Wake ***
	  // cancel sleep as a precaution
	  sleep_disable();

	  // Nachlauf
	  if((ctr_Nachlauf_I2C == 255)||(ctr_Nachlauf_NoCurrent == 255))
	  {
		  // do only once
		  if(BL03state != ST_SHUTDOWN)
		  {
		    // Shutdown
		    BL03state = ST_SHUTDOWN;
		    // Save values in EEPROM
		    EEPROM.put(EEPADR_CAPACITY, Capacity);

		    // Spannungsversorgung ausschalten
		    TCCR2A = bit (WGM20) | bit (WGM21); // fast PWM, Normal Port Operation
		    // switch off power supply
		    digitalWrite (PWM_OUT, LOW);
		  }


	  }
	  else
	  {
	  }


}

/* Analogwert erfassen während die CPU in den Sleep Mode geht */
uint16_t ADCwhileSlp(uint8_t channel)
{
	unsigned long Loc_Micros = micros(); // starte Laufzeitmessung

	adcDone = false; // Flag löschen - wird durch die Interrupt service routine gesetzt wenn Wandlung fertig ist

	// for measuring execution time
	pinMode (MON_ADC, OUTPUT);
	digitalWrite (MON_ADC, HIGH);

	/* zuerst den Kanal welcher gelesen werden soll im Register eintragen */
    ADMUX = bit (REFS0) | ((channel-14) & 0x07);
    ADCSRA = bit (ADEN) | bit (ADIF);  // enable ADC, turn off any pending interrupt
    ADCSRA |= bit (ADPS2) | bit (ADPS1) | bit (ADPS0);   // prescaler of 128

    noInterrupts ();
 //   set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
 //   sleep_enable();
    // start the conversion
    ADCSRA |= bit (ADSC) | bit (ADIE);
    interrupts ();
   // sleep_cpu ();
   // sleep_disable ();

    // awake again, reading should be done, but better make sure
    // maybe the timer interrupt fired
    while (bit_is_set (ADCSRA, ADSC))
      { }

    // for measuring execution time
	digitalWrite (MON_ADC, LOW);
	pinMode (MON_ADC, INPUT);

	t_ADC = micros() - Loc_Micros;
	/* den Wert zurückgeben */
	return adcReading;
}

void ReadAllADCValues(void)
{
	// read ADC values and add up to get average
	Current_Out_Raw += ADCwhileSlp(PIN_A0);
	Current_In_Raw += ADCwhileSlp(PIN_A1);
	Voltage12_Raw += ADCwhileSlp(PIN_A2);
	Voltage24_Raw += ADCwhileSlp(PIN_A3);
	Voltage36_Raw += ADCwhileSlp(PIN_A6);
	VoltageINT_Raw += ADCwhileSlp(PIN_A7);
}

uint16_t ScalePhysical(uint16_t adc_value, uint16_t factor, uint16_t offset)
{
	uint32_t Loc_value;
	Loc_value = adc_value;
	Loc_value *= factor;
	Loc_value /= (1023*NoOfAquisitionCycles);
	if (offset > Loc_value)
		Loc_value = 0;
	else
		Loc_value -= offset;
	return (uint16_t) Loc_value;
}

// read a short value from the EEPROM
// There are 4 copies. The newest copy is fount by the counter.
// validity is checked with 8bit CRC
// Layout in EEPROM:
// copy0: Datalow, Datahigh, counter, CRC
//  ...
// copy3: Datalow, Datahigh, counter, CRC
uint16_t EepReadShortCRC (uint16_t address)
{
#define CAPACITY 0x100
union eep_block_t
{
	struct
	{
		union
		{
			uint8_t b[2];
			uint16_t w;
		}data;
		uint8_t cntr;
		uint8_t crc;
	}str;
	uint8_t buffer[4];
};
eep_block_t eep_copy;
uint8_t i;

    for (i=0; i<4; i++)
	{
		 eep_copy.buffer[i] = EEPROM.read(CAPACITY+i);
	}
	return eep_copy.str.data.w;
}

/**
		CRC result width: 8 bits
		Polynomial: 0x1D
		Initial value: 0xFF
		Input data reflected: No
		Result data reflected: No
		XOR value: 0xFF
		Check: 0x4B
		Magic check: 0xC4
	*/
uint8_t const crc8_SAE_J1850[256] =
	{
		0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
		0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
		0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
		0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
		0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
		0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
		0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
		0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
		0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
		0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
		0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
		0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
		0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
		0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
		0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
		0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4,
	};

/* $Log: BL03Ersatz.ino $
 * 24-01-06: erste Version
 */
