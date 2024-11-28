// Arduino Steuerung für Akkuersatz BL03
//
// Copyright (C) 2024 by Rudolf Herzog
//
// Diese Programm soll alle benötigten Botschaften vom Ersatzakku an die TranzX Steuerung senden
/****************************************
 * Version für LGT8F328 Microcontroller *
 ****************************************/
#include "Arduino.h"
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/
#include <Wire.h>
#include "LiFePO_HW_version.h"
#include "EEP_handler.h"
#include <WDT.h>
#include <PMU.h>

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

enum T_charge {
  SC_INIT,
  SC_CHARGE,
  SC_BMS,
  SC_FULL,
};
T_charge stateCharge;

// Aquisition
#define NoOfAquisitionCycles 4
uint8_t cycle; // NoOfAquisitionCycles ADCvalues are collected and an average value is generated
// Raw values
volatile uint16_t Current_In_Raw,Current_Out_Raw,Voltage12_Raw,Voltage24_Raw,Voltage36_Raw,VoltageINT_Raw;

// Min/Max values
/* battery empty */
#define MIN_VOLTAGE		31000
/* battery full */
#define MAX_VOLTAGE		42000
/* max test bench */
#define TEST_VOLTAGE  20000

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
#define MAX_CAPACITY 9000

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

/* Nachlauf */
uint8_t ctr_Nachlauf_NoCurrent;

/* function declarations */
void ReadAllADCValues(void);
void CalcPhysicalValues(void);
void SetOutputs_1000ms (void);

uint16_t ADCwhileSlp(uint8_t);
uint16_t ScalePhysical(uint16_t adc_value, uint16_t factor, uint16_t offset);

// Watchdog
volatile boolean wdtFlag;

// EEPROM
EEP_handler myEEP_hdl = EEP_handler();


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
		flag_print_commI2C = PRINT_UNSIGNED;
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
#if 1
ISR (WDT_vect)
{
  wdt_reset();  // 

  wdtFlag = true;

  wdt_ienable(WTO_256MS); // restart watchdog  
  // monitor Pin high
  pinMode (MON_WDT, OUTPUT);
  digitalWrite (MON_WDT, HIGH);

}  // end of WDT_vect
#endif

void setup() {
	// PWM on Cntr2
	pinMode (PWM_OUT, OUTPUT);
  digitalWrite (PWM_OUT, HIGH);
  // BMS not active
  pinMode (BMS_OUT, OUTPUT);
  digitalWrite (BMS_OUT, LOW);
  // Err LED enable
  DDRE |= (1<<DDE4);
  // Err LED on as lamp test
  PORTE |= (1<<DDE4);
  //pinMode (LED_RED, OUTPUT);
  //digitalWrite (LED_RED, LOW);

#if (HW_version == 1) // PWM
	pinMode (11, OUTPUT);
	TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1); // fast PWM, clear OC2B on compare, set at bottom, OC2A disconnected (will be used with SPI)
	TCCR2B = bit (WGM22) | bit (CS21);         // fast PWM top=OCR2A, prescaler of 1
	OCR2A =  ((F_CPU / 8) / 50000) - 1;        // zero relative 50kHz
	OCR2B = ((OCR2A + 1) / 5) - 1;             // 20% duty cycle
#endif
#if (HW_version == 3) // PWM
/*Um Energie zu sparen wird die 36V Betriebsspannung durch einen Schaltregler auf einen Wert von 10..6V gewandelt. Um auch bei Testbetrieb mit 12V
  starten zu können wird mit einem hohen duty cycle gestartet. Der Duty cycle wird dann solange reduziert bis die gewünschte interne 
  Betriebsspannung erreicht ist. */
  #if 0 // zum testen ohne PWM (0), sonst 1
  TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1); // fast PWM, clear OC2B on compare, set at bottom, OC2A disconnected (will be used with SPI)
  TCCR2B = bit (WGM22) | bit (CS20);         // fast PWM top=OCR2A, prescaler of 1
  OCR2A =  ((F_CPU / 1) / 16666) - 1;        // bei F_CPU 2MHz und 16,666kHz ist der Wert 119, passt also in 1 Byte
  OCR2B = ((OCR2A + 1) / 4)*3 - 1;           // 75% duty cycle
  #endif
#endif

	//  Startmeldung ausgeben
	Serial.begin(19200);
	Serial << F("BL03Ersatz V");
	Serial << __DATE__ << F(" ") << __TIME__ << endl;

  // EEPROM Initialisieren
  myEEP_hdl.Init();

  //start time counter
  ctr_Nachlauf_NoCurrent = 0;
  flag_print_measurement = false;

  // Interrupt routinen anmelden
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // I2C starten
  Wire.begin((uint8_t)I2C_DEV_ADDR) ;

  // for measuring execution time
  pinMode (MON_ADC, OUTPUT);
  digitalWrite (MON_ADC, HIGH);
  // initialize data aquisition
  BL03state = ST_NEW;
  cycle = 0;
  Current_In_Raw = 0;
  Current_Out_Raw = 0;
  Voltage12_Raw = 0;
  Voltage24_Raw = 0;
  Voltage36_Raw = 0;
  Capacity_Low = 0;
  // start up in discharging mode
  BatteryStatus = BATT_STAT_DISCHARGING;

  // initialize EEPROM values
  if (myEEP_hdl.EepRead(EEPADR_CAPACITY, &Capacity))
  {
    // copy Data to destination
    //  Capacity = (uint16_t)EEP.bufE2pRd_CV[1] * 256;
    //  Capacity += EEP.bufE2pRd_CV[0];
    //check plausibility
    if((Capacity > MAX_CAPACITY))
    {
      // if unplausible set to 90%
      Capacity = MAX_CAPACITY / 100 * 90;
    }
    
  }
  else
  {
	  // no valid data in EEP set to 90%
	  Capacity = MAX_CAPACITY / 100 * 90;
  }
  Serial.print(F("Capacity: "));
  Serial.println(Capacity);

  // for measuring execution time
  digitalWrite (MON_ADC, LOW);
  // write first timestamp
  receiveTimeStmp = millis();
  // Err LED off lamp test finished
  PORTE &= !(1<<DDE4);

  // start watchdog in interrupt mode
  wdt_ienable(WTO_256MS);
}

void loop() {
	/*********************/
	/* 250ms Zeitscheibe */
	/*********************/
  if (wdtFlag)
  {
    wdtFlag = false;
		// do ADC conversions
		ReadAllADCValues();
		cycle++;

	  /**********************/
	  /* 1000ms Zeitscheiben */
	  /**********************/
    switch (cycle)
    {
      case NoOfAquisitionCycles:
        // calculate the physical values from the ADCs
        CalcPhysicalValues();
        // process state machine
        StateMachine();
        // set flag to send measured values on the serial port
        flag_print_measurement = true;
        //
        SetOutputs_1000ms();

        cycle = 0; // prepare for next cycle
        break;
      /*********************/
      /* background tasks  */
      /*********************/
      case 1:
        if (flag_print_measurement == true)
        {
          //Show values on the USB interface for debugging
          Serial << Current_In << F(",") << Current_Out << F(",") << Capacity << F(",");
          Serial << Voltage12 << F(",") << Voltage24 << F(",");
          Serial << Voltage36 << F(",") << VoltageINT << F(",");
          Serial << lowByte(BatteryStatus) << F(",") << (BL03state * 10 + stateCharge) << F(",") /*<< OCR2B*/ << endl;
          flag_print_measurement = false;
        }
        break;
      case 2:
		    if (flag_print_commI2C != PRINT_NOTHING)
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
        break;
      case 3:
        // nothing to do
        break;
      default:
        cycle = 0; // prepare for next cycle
        break;
  	}
  }
  // *********************
  // *** Prepare Sleep ***
  // *********************
  Serial.flush();
  // monitor Pin low
    digitalWrite (MON_WDT, LOW);
    pinMode (MON_WDT, INPUT);

  //PMU.sleep(PM_IDLE,SLEEP_FOREVER);  
  
  while (wdtFlag == false){
    digitalWrite (BMS_OUT, HIGH);
  }
  digitalWrite (BMS_OUT, LOW);
}

/* Analogwert erfassen */
uint16_t ADCwhileSlp(uint8_t channel)
{
	//unsigned long Loc_Micros = micros(); // starte Laufzeitmessung

	// for measuring execution time
	pinMode (MON_ADC, OUTPUT);
	digitalWrite (MON_ADC, HIGH);

	 /* den ausgewählten Kanal lesen */
   adcReading = analogRead(channel);

    // for measuring execution time
	digitalWrite (MON_ADC, LOW);
	pinMode (MON_ADC, INPUT);

	//t_ADC = micros() - Loc_Micros;
	/* den Wert zurückgeben */
	return adcReading;
}

void ReadAllADCValues(void)
{
	// read ADC values and add up to get average
	Current_Out_Raw += ADCwhileSlp(I_OUT);
	Current_In_Raw += ADCwhileSlp(I_IN);
	Voltage12_Raw += ADCwhileSlp(V_12);
	Voltage24_Raw += ADCwhileSlp(V_24);
	Voltage36_Raw += ADCwhileSlp(V_36);
	VoltageINT_Raw += ADCwhileSlp(V_INT);
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
  // check if empty
  if((Voltage36 < MIN_VOLTAGE)&&(Voltage36 > TEST_VOLTAGE))
  {
    /* set empty flag */
    BatteryStatus |= REMAINING_CAPACITY_ALARM;
    /* set capacity to min  (5%) */
    //Capacity = MAX_CAPACITY / 20;
  }
	else if(Voltage36 > MIN_VOLTAGE + 1000)
	{
		/* Clear empty flag */
		BatteryStatus &= ~REMAINING_CAPACITY_ALARM;
	}

  // prepare vor next calculation
	Current_In_Raw = 0;
	Current_Out_Raw = 0;
	Voltage12_Raw = 0;
	Voltage24_Raw = 0;
	Voltage36_Raw = 0;
	VoltageINT_Raw = 0;
}

void StateMachine(void)
{
  switch(BL03state)
  {
    case ST_NEW:
      // after first data calculated
      // set state to initialized
      BL03state = ST_INIT;
      break;

    case ST_INIT:
      // set INIT bit
      BatteryStatus |= BATT_STAT_INIT;
      stateCharge = SC_INIT;
      
      if (Current > 20)
      {
        BL03state = ST_CHARGE;
        stateCharge = SC_CHARGE;
      }
      else //if (Current < -10)
      {
        BL03state = ST_DISCHARGE;
      }
      break;

    case ST_CHARGE:
      if (Current < -10)
      {
        BL03state = ST_DISCHARGE;
        // deactivate BMS charging aborted
        digitalWrite (BMS_OUT, LOW);
        stateCharge = SC_INIT;
      }
      else
      {
        // charging mode, clear discharging bit
        BatteryStatus &= ~BATT_STAT_DISCHARGING;

        switch(stateCharge)
        {
          case SC_INIT:
            stateCharge = SC_CHARGE;
            break;
          case SC_CHARGE:
            if(Current < 50)
            {
              //activate BMS to proceed charging of other moduls
              digitalWrite (BMS_OUT, HIGH);
              stateCharge = SC_BMS;
            }
            break;
          case SC_BMS:
            if(Current < 20)
            {
              // deactivate BMS battery is full
              digitalWrite (BMS_OUT, LOW);
              stateCharge = SC_FULL;
              // all modules filled?
              if( (Voltage12>13400)&
                 ((Voltage24-Voltage12)>13400)&
                 ((Voltage36-Voltage24)>13400))
                 {
                   // set capacity to max
                   Capacity = MAX_CAPACITY;
                 }
            }
            break;
          case SC_FULL:
            break;
          default:
            stateCharge = SC_INIT;
          break;
        }
      }
      break;

    case ST_DISCHARGE:
      if (Current > 30)
      {
        BL03state = ST_CHARGE;
      }
      else
      {
        // set discharging bit
        BatteryStatus |= BATT_STAT_DISCHARGING;
      }
      break;

    case ST_NACHLAUF:
      break;
    case ST_SHUTDOWN:
      break;
    default:
      break;
  }

  // Nachlauf
  // checks if ebike is on,
  if (Current_Out > 20)
  {
    ctr_Nachlauf_NoCurrent = 0;
  }
  // or battery is charging?
  else if (Current_In > 20)
  {
    ctr_Nachlauf_NoCurrent = 0;
  }
  // after 60 seconds inactivity -> shutdown
  else if(ctr_Nachlauf_NoCurrent >= 120)
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

      // copy data to write into buffer
      //EEP.bufE2pWr_CV[0] = Capacity;
      //EEP.bufE2pWr_CV[1] = Capacity>>8;
      // Save values in EEPROM
      myEEP_hdl.EepWrite(EEPADR_CAPACITY, &Capacity, 3);
        // message for debugging
      Serial << F("shutdown: ") << ctr_Nachlauf_NoCurrent;
      delay(1);

        // Spannungsversorgung ausschalten
        TCCR2A = bit (WGM20) | bit (WGM21); // fast PWM, Normal Port Operation
        // switch off power supply
        digitalWrite (PWM_OUT, LOW);
    }
  }
  // inactive 
  else
  {
    // if not in testbench
    if(Voltage36 > 20000)
    {
      // neither ebike on nor charging - increase timeout counter
      ctr_Nachlauf_NoCurrent++;
    }
    else
    {
      // in testbench
      ctr_Nachlauf_NoCurrent = 0;
    }
  }
}

void SetOutputs_1000ms (void)
{
	// adjust PWM duty cycle for switched power supply
	// if internal voltage < 7V
	if(VoltageINT < 7000)
	{
#if (HW_version == 1) // PWM
		// is cycle time longer than pulse time+1
		if(OCR2A > OCR2B + 10)
		{	// enlarge duty cycle by reducing cycle time
			OCR2A--;
		}
#endif
#if (HW_version == 3) // PWM period fix, increase duty
    // is cycle time longer than pulse time+1
    if(OCR2B < OCR2A - 1) // limit duty cycle
    { // enlarge duty cycle by 2 steps
      OCR2B++;
      OCR2B++;
    }
#endif
	}
	// if internal voltage > 7V
	if(VoltageINT > 7000)
	{
#if (HW_version == 1) // PWM
		// is cycle time shorter than maximum
		if(OCR2A < 127)
		{	// reduce duty cycle by enlarging cycle time
			OCR2A++;
		}
#endif
#if (HW_version == 3) // PWM period fix, decrease duty
    // min duty cycle 5%
    if(OCR2B > 3)
    { // reduce duty cycle
      OCR2B--;
      if(VoltageINT > 10000) // if >10V
      {
        OCR2B--; // reduce more
      }
    }
#endif
	}

}

/* $Log: LiFePO.ino $
 */
