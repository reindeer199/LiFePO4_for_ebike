// Arduino Steuerung für Akkuersatz BL03
//
// Copyright (C) 2024 by Rudolf Herzog
//
// Diese Programm soll alle benötigten Botschaften vom Ersatzakku an die TranzX Steuerung senden

#include "Arduino.h"
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/
#include <Wire.h>
#include <EEPROM.h>

// PINs
#define PWM_OUT 3
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
uint32_t t_250ms;
uint32_t receiveTimeStmp;
enum T_states {
	ST_NEW,
	ST_INIT,
	ST_CHARGE,
	ST_DISCHARGE,
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

// Min/Max values
/* battery empty */
#define MIN_VOLTAGE		31000
/* battery full */
#define MAX_VOLTAGE		42000

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
// Value for 1 mA*h = 3600 mA*s;
#define VALUE_1mAxh 3600
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
volatile uint8_t flag_print_measurement;
enum {
	PRINT_NOTHING,
	PRINT_UNSIGNED,
	PRINT_SIGNED,
	PRINT_STRING,
}flag_print_commI2C;
uint16_t printValueU;
int16_t printValueS;
char * printValueStr;
const char str_unknown[] = "unknown";

volatile uint16_t adcReading;
volatile boolean adcDone;

/* Zeitscheiben */
unsigned long t_ADC;

// EEPROM address for CAPACITY
#define EEPADR_CAPACITY 0x100	// 2 Bytes

/* Nachlauf */
uint8_t ctr_Nachlauf_NoCurrent;

/* function declarations */
void ReadAllADCValues(void);
void CalcPhysicalValues(void);
void SetOutputs_1000ms (void);

uint16_t ADCwhileSlp(uint8_t);
uint16_t ScalePhysical(uint16_t adc_value, uint16_t factor, uint16_t offset);


void onRequest(){
	uint16_t Loc_temp;
	int16_t Loc_stemp;
	union {
		uint16_t wrd;
		uint8_t byt[2];
	} LocValue;
	// Request LED Einschalten
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
		// send to serial, but not in the ISR; set flag to send it in the background task
		flag_print_commI2C = PRINT_UNSIGNED;
		printValueU = Loc_temp;
		break;
	case VOLTAGE: // Voltage 36000/3,34
		Loc_temp = (uint16_t)(Voltage36/3);
		Wire.write((uint8_t)Loc_temp); // Low byte;
		Wire.write((uint8_t)(Loc_temp >> 8)); // High byte;
		// send to serial, but not in the ISR; set flag to send it in the background task
		flag_print_commI2C = PRINT_UNSIGNED;
		printValueU = Loc_temp;
		//	Serial << command << F(":") << Loc_temp << F("mV ");
		break;
	case CURRENT: // Current
		Loc_stemp = Current_Out-Current_In;
		Wire.write((uint8_t)(Loc_stemp & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
		Wire.write((uint8_t)((Loc_stemp >> 8) & 0xFF)); // High byte;
		// send to serial, but not in the ISR; set flag to send it in the background task
		flag_print_commI2C = PRINT_SIGNED;
		printValueS = Loc_stemp;
		//	Serial << command << F(":") << Loc_stemp << F("mA ");
		break;
	case AVERAGE_CURRENT: // average Current
		Loc_stemp = Current_Out-Current_In;
		Wire.write((uint8_t)(Loc_stemp & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
		Wire.write((uint8_t)((Loc_stemp >> 8) & 0xFF)); // High byte;
		flag_print_commI2C = PRINT_SIGNED;
		printValueS = Loc_stemp;
		//	Serial << command << F(":") << Loc_stemp << F("mA ");
		break;
	case RELATIVE_SOC: // Relative state of charge
		LocValue.wrd = Capacity / (MAX_CAPACITY / 100); // Scale to %
		if(LocValue.wrd > 100)
		{
			/* limit to 100% */
			LocValue.wrd = 100;
		}
		Wire.write(LocValue.byt[0]); // Low byte;
		Wire.write(LocValue.byt[1]); // High byte; sollte immer 0
		// send to serial, but not in the ISR; set flag to send it in the background task
		//flag_print_commI2C = PRINT_UNSIGNED;
		printValueU = LocValue.wrd;
		//	Serial << command << F(":") << LocValue.wrd << F("% ");
		break;
	case BATTERY_STATUS: // BatteryStatus(0x16)
		// This read-word function returns the status of the bq20z80A-based battery.
		//            bit 7 bit 6 bit 5 bit 4 bit 3 bit 2 bit 1 bit 0
		// Low Byte   INIT  DSG   FC    FD    EC3   EC2   EC1   EC0
		// High Byte  OCA   TCA   RSVD  OTA   TDA   RSVD  RCA   RTA
		LocValue.wrd = BatteryStatus;
		Wire.write(LocValue.byt[0]); // Low byte;
		Wire.write(LocValue.byt[1]); // High byte
		//flag_print_commI2C = PRINT_UNSIGNED;
		printValueU = LocValue.wrd;
		//	Serial << command << F(":") << LocValue.wrd << F(" ");
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
		flag_print_commI2C = PRINT_STRING;
		printValueStr = str_unknown;
		//	Serial << command << F(":") << F("not sup ");
		break;
    }
  // Request LED ausschalten
  digitalWrite (MON_REQ, LOW);
  pinMode (MON_REQ, OUTPUT);
}

void onReceive(int len){
  uint8_t i=0;
  // monitor I2C high
  pinMode (MON_I2C, OUTPUT);
  digitalWrite (MON_I2C, HIGH);

  while(Wire.available()){
    data[i] = Wire.read();
    // limit index to BUF_SIZE-1
    if(i<BUF_SIZE-1)
      i++;
  }
  command = data[0];
  // monitor I2C low
  digitalWrite (MON_I2C, LOW);
  pinMode (MON_I2C, INPUT);
}

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
	// PWM on Cntr2
	pinMode (PWM_OUT, OUTPUT);
	pinMode (11, OUTPUT);
	TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1) | bit (COM2A1); // fast PWM, clear OC2B on compare, set at bottom
	TCCR2B = bit (WGM22) | bit (CS21);         // fast PWM top=OCR2A, prescaler of 1
	OCR2A =  ((F_CPU / 8) / 50000) - 1;        // zero relative 50kHz
	OCR2B = ((OCR2A + 1) / 5) - 1;             // 20% duty cycle

	//  Startmeldung ausgeben
	Serial.begin(115200);
	Serial << F("BL03Ersatz V");
	Serial << __DATE__ << F(" ") << __TIME__ << endl;

  //start time counter
  ctr_Nachlauf_NoCurrent = 0;
  flag_print_measurement = false;

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
  Serial.print(F("Capacity: "));
  Serial.println(Capacity);


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
  // write first timestamp
  receiveTimeStmp = millis();
}

void loop() {
	/*********************/
	/* 250ms Zeitscheibe */
	/*********************/
	if(millis() - t_250ms >= 250)
	{
		t_250ms += 250;
		// do ADC conversions
		ReadAllADCValues();
		cycle++;
	}

	/**********************/
	/* 1000ms Zeitscheibe */
	/**********************/
	else if(cycle >= NoOfAquisitionCycles)
	{
		// calculate the physical values from the ADCs
		CalcPhysicalValues();
		// set flag to send measured values on the serial port
		flag_print_measurement = true;
		//
		SetOutputs_1000ms();
	}
	/*********************/
	/* background tasks  */
	/*********************/
	else
	{
		if (flag_print_measurement == true)
		{
			//Show values on the USB interface for debugging
			Serial << Current_In << F(";") << Current_Out << F(";") << Capacity << F(";");
		//	Serial << Voltage12 << F(" ") << Voltage24 << F("mV ");
			Serial << Voltage36 << F(";") << VoltageINT << F(";") << OCR2A << endl;
			flag_print_measurement = false;
		}
		else if (flag_print_commI2C != PRINT_NOTHING)
		{
			switch(flag_print_commI2C)
			{
			case PRINT_UNSIGNED:
				Serial << command << F(":") << printValueU << endl;
				break;
			case PRINT_SIGNED:
				Serial << command << F(":") << printValueS << endl;
				break;
			case PRINT_STRING:
				Serial << command << F(":") << printValueStr << endl;
				break;
			default:
				Serial << F("error:") << flag_print_commI2C << endl;
				break;
			}
			flag_print_commI2C = PRINT_NOTHING;
		}
	}

	  // *********************
	  // *** Prepare Sleep  NO SLEEP YET ***
	  // *********************

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
	Loc_value /= (1023*cycle);
	if (offset > Loc_value)
		Loc_value = 0;
	else
		Loc_value -= offset;
	return (uint16_t) Loc_value;
}

void CalcPhysicalValues(void)
{
	// physical values are calculated on the average of NoOfAquisitionCycles
	Current_In = ScalePhysical(Current_In_Raw, FAKTOR_CURRENT_IN, OFFSET_CURRENT_IN);
	Current_Out = ScalePhysical(Current_Out_Raw, FAKTOR_CURRENT_OUT, OFFSET_CURRENT_OUT);
	// resulting current
	Current = Current_In - Current_Out;

	Voltage12 = ScalePhysical(Voltage12_Raw, FAKTOR_VOLTAGE_12, 0);
	Voltage24 = ScalePhysical(Voltage24_Raw, FAKTOR_VOLTAGE_24, 0);
	Voltage36 = ScalePhysical(Voltage36_Raw, FAKTOR_VOLTAGE_36, 0);
	VoltageINT = ScalePhysical(VoltageINT_Raw, FAKTOR_VOLTAGE_INT, 0);

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

	// battery status
	// set INIT bit
	BatteryStatus |= BATT_STAT_INIT;
	// set / reset CHARGING mode
	if (Current > 10)
	{
		// charging mode, clear discharging bit
		BatteryStatus &= ~BATT_STAT_DISCHARGING;
		// set state to charging battery, only if not end
		if(BL03state < ST_NACHLAUF)
		{
		    BL03state = ST_CHARGE;
		}
	}
	else if (Current < -25)
	{
		// set discharging bit
		BatteryStatus |= BATT_STAT_DISCHARGING;
		// set state to discharging battery, only if not end
		if(BL03state < ST_NACHLAUF)
		{
		    BL03state = ST_DISCHARGE;
		}
		// check if empty
		if(Voltage36 < MIN_VOLTAGE)
		{
			/* set empty flag */
			BatteryStatus |= REMAINING_CAPACITY_ALARM;
			/* set capacity to min	(5%) */
	    	Capacity = MAX_CAPACITY / 20;
		}
	}

	if(Voltage36 > MIN_VOLTAGE + 1000)
	{
		/* Clear empty flag */
		BatteryStatus &= ~REMAINING_CAPACITY_ALARM;
	}

	cycle = 0; // prepare for next cycle
	Current_In_Raw = 0;
	Current_Out_Raw = 0;
	Voltage12_Raw = 0;
	Voltage24_Raw = 0;
	Voltage36_Raw = 0;
	VoltageINT_Raw = 0;

	// after first data calculated
	if(BL03state == ST_NEW)
	{
		// set state to initialized
	    BL03state = ST_INIT;
	}
}

void SetOutputs_1000ms (void)
{
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

	// Nachlauf
	// checks if ebike is on,
	if (Current_Out > 25)
	{
	  ctr_Nachlauf_NoCurrent = 0;
	}
	// or battery is charging?
	else if (Current_In > 25)
	{
	  ctr_Nachlauf_NoCurrent = 0;
	}
	else
	{
		// neither ebike on nor charging - increase timeout counter
	  ctr_Nachlauf_NoCurrent++;
	}

	// after 60 seconds inactivity -> shutdown
	if(ctr_Nachlauf_NoCurrent >= 60)
	{
		// do only once
		if(BL03state != ST_SHUTDOWN)
		{
			// Shutdown
		    BL03state = ST_SHUTDOWN;
		    // limit to max. value
		    if(Capacity > MAX_CAPACITY)
		    {
		    	Capacity = MAX_CAPACITY;
		    }
		    // Save values in EEPROM
		    EEPROM.put(EEPADR_CAPACITY, Capacity);
		    // message for debugging
			Serial << F("shutdown: ") << ctr_Nachlauf_NoCurrent;
			delay(1);

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




/* $Log: BL03Ersatz.ino $
 * 24-02-10: erste Version in Github
 * 24-02-12: PWM früher gestartet, debug Ausgaben überarbeitet
 * 24-02-13: Betriebssystem komplett überarbeitet; Reste von sleep mode entfernt
 */
