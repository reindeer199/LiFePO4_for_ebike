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
//#include "lgt_LowPower.h"
#include "LiFePO_HW_version.h"
#include "EEP_handler.h"
#include <WDT.h>
// #include <PMU.h>

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

// feste Pulsweite von 4µs vorgeben:
#define PULSWITH_4MICROS ((4 * F_CPU) / (8 * 1000000))
#if (PULSWITH_4MICROS < 2)
#error "PULSWITH"
#endif

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
volatile uint16_t Current_In_Raw,Current_Out_Raw,Voltage36_Raw,VoltageINT_Raw,Current_In_Rawtemp,Current_Out_Rawtemp;
volatile uint16_t Voltage21_Raw,Voltage22_Raw,Voltage24_Raw,VoltageINT_temp;
// Min/Max values
/* battery empty */
#define MIN_VOLTAGE		31000
/* battery full */
#define MAX_VOLTAGE		42000
/* max test bench */
#define TEST_VOLTAGE  20000

// Scaled values
// Currents in mA
int16_t Current_In,Current_Out;
int16_t Current;
// Voltages in mV
uint16_t Voltage36,VoltageINT;
uint16_t Voltage21,Voltage22,Voltage24;

// Capacity in mA * h
uint16_t Capacity;
// Capacity prescaler
int16_t Capacity_Low;
// Value for 1 mA*h = 3600 mA*s;
#define VALUE_1mAxh 3600

// BatteryStatus
uint16_t BatteryStatus;
// Bits within BatteryStatus
#define BATT_STAT_DISCHARGING    0x040
#define BATT_STAT_INIT           0x080
#define REMAINING_CAPACITY_ALARM 0x200
#define FULLY_DISCHARGED         0x010
#define FULLY_CHARGED            0x020


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
volatile boolean i2cDone;

/* Zeitscheiben */
unsigned long t_ADC;

/* Nachlauf */
uint8_t ctr_Nachlauf_NoCurrent;

/* function declarations */
void ReadAllADCValues(void);
void CalcPhysicalValues(void);
void SetOutputs_1000ms (void);
void Adjust_DutyCycle (void);

uint16_t ADCwhileSlp(uint8_t);
int16_t ScalePhysicalS(uint16_t adc_value, uint16_t factor, uint16_t offset);
uint16_t ScalePhysicalU(uint16_t adc_value, uint16_t factor, uint16_t offset);

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
	case CURRENT: // Current  This read-word function returns a signed integer value of the measured current being supplied 
	              // (or accepted) by the battery in mA, with a range of –32,768 to 32,767. 
	              // A positive value indicates charge current and negative indicates discharge.
#if 0
    Wire.write((uint8_t)(Current & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
		Wire.write((uint8_t)((Current >> 8) & 0xFF)); // High byte;
		// send to serial, but not in the ISR; set flag to send it in the background task
		flag_print_commI2C = PRINT_SIGNED;
		printValueS = Current;
		//	Serial << command << F(":") << Loc_stemp << F("mA ");
		break;
#endif    
	case AVERAGE_CURRENT: // average Current
		Wire.write((uint8_t)(Current & 0xFF)); // Low byte; vorerst mal dummy Wert ausgeben
		Wire.write((uint8_t)((Current >> 8) & 0xFF)); // High byte;
		flag_print_commI2C = PRINT_SIGNED;
		printValueS = Current;
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
  digitalWrite (MON_ADC, LOW);

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

}  // end of WDT_vect
#endif

#if (HW_version == 21) // PWM
ISR(TIMER2_OVF_vect)
{
  //digitalWrite (MON_I2C, HIGH);
  //pinMode (MON_I2C, OUTPUT);
}
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
#if (HW_version == 21) // PWM
/*Um Energie zu sparen wird die 36V Betriebsspannung durch einen Schaltregler auf einen Wert von 10..6V gewandelt. Um auch bei Testbetrieb mit 12V
  starten zu können wird mit einem hohen duty cycle gestartet. Der Duty cycle wird dann solange reduziert bis die gewünschte interne 
  Betriebsspannung erreicht ist. */
  #if 1 // zum testen ohne PWM (0), sonst 1
  // Um eine geringe Stromaufnahme zu erzielen ist notwendig den Systemtakt möglichst niedrig zu wählen!
  //   Die minimale F_CPU mit der die I²C Communikation noch funktioniert beträgt 4 MHz!
  // Die minimale Pulsbreite die die Hardware ausgeben kann sind ca. 4µs!
  // Die maximale Periode um die Spannung von 40V auf 7V runter zu takten beträgt lt. Messung ca 140µs! Bei einem Prescaler von 1 erreiche
  //   ich maximal 0.25µs * 256 = 64µs -> mit Prescaler 8: 2µs * 256 = 512µs
  TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1); // fast PWM, clear OC2B on compare, set at bottom, OC2A disconnected (will be used with SPI)
  TCCR2B = bit (WGM22) | bit (CS21);         // fast PWM top=OCR2A, prescaler of 8 (mit prescaler 1 komme ich nicht tief genug)
  OCR2B = (uint8_t)PULSWITH_4MICROS - 1;
  // starte mit 50% duty, also Periode 8µs
  OCR2A = (2 * PULSWITH_4MICROS) - 1;
  #endif
  #if 0 // zum testen von normal mode
  TCCR2A = 0  ; // normal Mode, normal port operation
  TCCR2B = bit (CS20) | bit (CS21) | bit (CS22);         //  prescaler of 1024
  TCNT2 =   240;        // bei F_CPU 2MHz und prescaler 1024 8ms
  TIMSK2 = (1<<TOIE2); // interrupt when TCNT2 is overflowed
  #endif
#endif

	//  Startmeldung ausgeben
	Serial.begin(19200);
	Serial << F("BL03Ersatz V") << HW_version;
	Serial << __DATE__ << F(" ") << __TIME__ << OCR2A << OCR2B << endl;

  // EEPROM Initialisieren
  myEEP_hdl.Init();

  //start time counter
  ctr_Nachlauf_NoCurrent = 0;
  flag_print_measurement = false;
  // start up in discharging mode (before interrupt enabled)
  BatteryStatus = BATT_STAT_DISCHARGING;

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
  Voltage21_Raw = 0;
  Voltage22_Raw = 0;
  Voltage24_Raw = 0;
  Voltage36_Raw = 0;

  Capacity_Low = 0;

  // initialize EEPROM values
  if (myEEP_hdl.EepRead(EEPADR_CAPACITY, &Capacity))
  {
    // copy Data to destination
    //Capacity = (uint16_t)EEP.bufE2pRd_CV[1] * 256;
    //Capacity += EEP.bufE2pRd_CV[0];
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

  // Disable pullups of I2C pins because there is an external pullup
  // reduces current consuption when one of this pins goes low
  // switching regulator can be adjusted lower then
  digitalWrite (SCL, LOW);
  digitalWrite (SDA, LOW);

  // for measuring execution time
  digitalWrite (MON_ADC, LOW);
  // write first timestamp
//  receiveTimeStmp = millis();
  // Err LED off lamp test finished
  PORTE &= !(1<<DDE4);

#if PWM_INT
    pinMode(PWM_INT, INPUT);
    digitalWrite(PWM_INT, HIGH); 
#endif
  // start watchdog in interrupt mode
  wdt_ienable(WTO_256MS);
}

void loop() {
    pinMode (MON_WDT, OUTPUT);
    digitalWrite (MON_WDT, LOW);
	/*********************/
	/* 250ms Zeitscheibe */
	/*********************/
  digitalWrite (BMS_OUT, LOW);
  if (wdtFlag)
  {
    wdtFlag = false;
    // monitor Pin high
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

        cycle = 0; // prepare for next cycle
        break;
      /*********************/
      /* background tasks  */
      /*********************/
      case 1:
        if (flag_print_measurement == true)
        {
          //Show values on the USB interface for debugging
          Serial << endl;
          Serial << Current_In << F(",") << Current_Out << F(",") << Capacity << F(",");
          Serial << Voltage36 << F(",") << VoltageINT << F(",");
          Serial << lowByte(BatteryStatus) << F(",") << (BL03state * 10 + stateCharge) << F(",");
//          Serial << Current_In_Rawtemp << F(",") << Current_Out_Rawtemp << F(",") << ((int32_t)OFFSET_CURRENT_OUT2 * Current_In)<< F(","); // temporary
          Serial << OCR2A << F(",") /*<< OCR2B << F(",")*/;
          Serial.flush();
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
          Serial.flush();
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
  else
  {
    /* Background tasks */
    /* adjust duty cycle faster than 1 times per second */
    Adjust_DutyCycle();
  }
  // *********************
  // *** Prepare Sleep ***
  // *** Leider nicht geeignet um Strom zu sparen! Um einen nennenswerten Einspareffekt zu erhalten müsste der PM_POWERDOWN sleep mode
  // *** verwendet werden. Das aufwachen dauert dann aber mehrere Millisekunden weil der Oszillator erst wieder hochfahren muss. D.h. 
  // *** die I²C-Botschaft ist längst vorbei bis der Prozessor wieder wach ist und die Botschaft empfangen könnte.
  // *********************
}

/* Analogwert erfassen */
uint16_t ADCwhileSlp(uint8_t channel)
{
	//unsigned long Loc_Micros = micros(); // starte Laufzeitmessung
#if 0 //(HW_version == 21)
  bool Loc_State = HIGH; 
  // wait until PWM pulse or timeout
  while(TCNT2 > 2)
  {
    digitalWrite (MON_ADC, Loc_State);
    Loc_State = !Loc_State;
  }
#endif  
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

/* ADC Read syncronized with PWM of power switching to reduce ripple */
uint16_t ADCsynRead(uint8_t channel)
{
  
#if (HW_version == 21) // works only with HW version 21
  while(TCNT2 > 2) // wait for current pulse; will be finished until analogRead starts!
  {}
#endif
  pinMode (MON_ADC, OUTPUT);
  digitalWrite (MON_ADC, HIGH); // for scope measurement

   /* den ausgewählten Kanal lesen */
   adcReading = analogRead(channel);

    // for measuring execution time
  digitalWrite (MON_ADC, LOW);
  pinMode (MON_ADC, INPUT);

  return adcReading;
}

void ReadAllADCValues(void)
{
	// read ADC values and add up to get average
  analogReference(DEFAULT);
  analogReadResolution(10); // Resolution = 10, 11 or 12 Bit
	Current_Out_Raw += ADCsynRead(I_OUT);
	Current_In_Raw += ADCsynRead(I_IN);
	Voltage36_Raw += ADCsynRead(V_36);
	VoltageINT_Raw += ADCsynRead(V_INT);
  //ADCsynRead(V_INT);
}

uint16_t ScalePhysicalU(uint16_t adc_value, uint16_t factor, uint16_t offset)
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

int16_t ScalePhysicalS(uint16_t adc_value, uint16_t factor, int16_t offset)
{
  int32_t Loc_value;
  Loc_value = adc_value;
  Loc_value *= factor;
  Loc_value /= (1023*cycle);
  Loc_value -= offset;
  return (int16_t) Loc_value;
}

void CalcPhysicalValues(void)
{
  int16_t Loc_Offset;
	// physical values are calculated on the average of NoOfAquisitionCycles
	Current_In = ScalePhysicalS(Current_In_Raw, FAKTOR_CURRENT_IN, OFFSET_CURRENT_IN);
  // offset of Current_Out depends on Current_In!
  Loc_Offset = OFFSET_CURRENT_OUT1 + ((OFFSET_CURRENT_OUT2 * Current_In) / 1000);
	Current_Out = ScalePhysicalS(Current_Out_Raw, FAKTOR_CURRENT_OUT, Loc_Offset);
  Current_In_Rawtemp = Current_In_Raw;
  Current_Out_Rawtemp = Current_Out_Raw;
	// resulting current
  // A positive value indicates charge current and negative indicates discharge.
  Current = Current_In - Current_Out;

//Voltage24 = ScalePhysicalU(Voltage24_Raw, FAKTOR_VOLTAGE_24, 0);
  Voltage21 = Voltage21_Raw;
  Voltage22 = Voltage22_Raw;
  Voltage24 = Voltage24_Raw;

	Voltage36 = ScalePhysicalU(Voltage36_Raw, FAKTOR_VOLTAGE_36, 0);
	VoltageINT = ScalePhysicalU(VoltageINT_Raw, FAKTOR_VOLTAGE_INT, 0);

	// Calculate Capacity
	Capacity_Low += Current;
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
		// Transfer -1mA*h to Capacity
		Capacity--;
		Capacity_Low += VALUE_1mAxh;
	}

	// battery status
  // check if empty
  if((Voltage36 < MIN_VOLTAGE)||(digitalRead(BAT_EMPTY)))
  {
    /* set empty flag */
    BatteryStatus |= REMAINING_CAPACITY_ALARM | FULLY_DISCHARGED;
    /* set capacity to min  (5%) */
    //Capacity = MAX_CAPACITY / 20;
  }
	else if((Voltage36 > MIN_VOLTAGE + 1000)&&(digitalRead(BAT_EMPTY) == 0))
	{
		/* Clear empty flag */
		BatteryStatus &= ~(REMAINING_CAPACITY_ALARM | FULLY_DISCHARGED);
	}

  // prepare vor next calculation
	Current_In_Raw = 0;
	Current_Out_Raw = 0;
  Voltage21_Raw = 0;
  Voltage22_Raw = 0;
  Voltage24_Raw = 0;

	Voltage36_Raw = 0;
	VoltageINT_Raw = 0;
}

void StateMachine(void)
{
  switch(BL03state)
  {
    default:
    case ST_NEW:
      // after first data calculated
      // set state to initialized
      BL03state = ST_INIT;
      break;

    case ST_INIT:
      // set INIT bit
      BatteryStatus |= BATT_STAT_INIT;
      stateCharge = SC_INIT;
      ctr_Nachlauf_NoCurrent = 0;
      BL03state = ST_NACHLAUF;
      break;

    case ST_CHARGE:
      if (Current < 0)
      {
        BL03state = ST_NACHLAUF;
        // set discharging bit
        BatteryStatus |= BATT_STAT_DISCHARGING;
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
              if( Voltage36>40400)
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
      if (Current > -20)
      {
        BL03state = ST_NACHLAUF;
      }
      else
      {
        // set discharging bit
        BatteryStatus |= BATT_STAT_DISCHARGING;
      }
      break;

    case ST_NACHLAUF:
      // Nachlauf
      // checks if ebike is on,
      if (Current < -30)
      {
    	BL03state = ST_DISCHARGE;
    	ctr_Nachlauf_NoCurrent = 0;
      }
      // or battery is charging?
      else if (Current > 20)
      {
    	BL03state = ST_CHARGE;
     	ctr_Nachlauf_NoCurrent = 0;
      }
      // after inactivity > Nachlauf_time : shutdown
      else if(ctr_Nachlauf_NoCurrent >= 120)
      {
        // limit to max. value
        if(Capacity > MAX_CAPACITY)
        {
          Capacity = MAX_CAPACITY;
        }
        // Save values in EEPROM
        myEEP_hdl.EepWrite(EEPADR_CAPACITY, &Capacity, 3);
        // message for debugging
        Serial << F("shutdown: ") << ctr_Nachlauf_NoCurrent;
        delay(1);
   	    // Shutdown
   	    BL03state = ST_SHUTDOWN;
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
        else // in testbench
        {
          // no timeout
          ctr_Nachlauf_NoCurrent = 0;
        }
      }
      break;

    case ST_SHUTDOWN:
      // Spannungsversorgung ausschalten
      TCCR2A = bit (WGM20) | bit (WGM21); // fast PWM, Normal Port Operation
      // switch off power supply
      digitalWrite (PWM_OUT, LOW);
      break;
  }
}

void SetOutputs_1000ms (void)
{
	// adjust PWM duty cycle for switched power supply
	// if internal voltage < 8,5V
	if(VoltageINT < 8500)
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
#if (HW_version == 21) // PWM pulse with fix, decrease period
    // min. period not reached?
    if(OCR2A > ((2*OCR2B)+3) )
    {
      OCR2A--;    // decrease period
      // Risk of undervoltage?
      if(VoltageINT < 8000)
      {
        OCR2A--;    // decrease more
      }
   }
#endif
	}
	// if internal voltage > 8,7V then proceed with decreasing voltage
  // (with lower limits internal voltage can drop too much!!!)
	if(VoltageINT > 8700)
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
#if (HW_version == 21) //  PWM pulse with fix, increase period

#if 1
//Calculate Step
uint8_t Loc_Step;
      Loc_Step = 1;    // increase period
      if(VoltageINT > 30000) // if >30V
      {
        Loc_Step = 16; // increase more
      }
      else if(VoltageINT > 20000) // if >20V
      {
        Loc_Step = 8; // increase more
      }
      else if(VoltageINT > 15000) // if >15V
      {
        Loc_Step = 4; // increase more
      }
      else if(VoltageINT > 12500) // if >12,5V
      {
        Loc_Step = 2; // increase more
      }
    // max period not reached?
    if(OCR2A < (254-Loc_Step))
    {
      OCR2A += Loc_Step;
    }
#else
    if(OCR2A < 254)
    {
      OCR2A++;
    }
#endif    
#endif
	}

}

/* Adjusts Duty Cycle of switching regulator for internal voltage */
/* this is done by modifieing the period, ON time is kept constant */
/* if voltage is above specified level then period is increased */
/* else period is decreased */
void Adjust_DutyCycle (void)
{
#if (HW_version == 21) // this is only for HW 21

  adcReading = analogRead(V_INT);
  VoltageINT_temp = ScalePhysicalU(adcReading*NoOfAquisitionCycles, FAKTOR_VOLTAGE_INT, 0);
  // adjust PWM duty cycle for switched power supply
  // if internal voltage < 8,6V
  if(VoltageINT_temp < 8600)
  {
    // min. period not reached?
    if(OCR2A > ((2*OCR2B)+1) )
    {
      OCR2A--;    // decrease period
      Serial << F("-"); /* monitor show that period decreased */
      // Risk of undervoltage?
      if(VoltageINT < 8000)
      {
        OCR2A--;    // decrease more
        Serial << F("="); /* monitor show that period decreased double */
      }
    }
  }
  // if internal voltage > 8,8V then proceed with decreasing voltage
  // (with lower limits internal voltage can drop too much!!!)
  if(VoltageINT_temp > 8800)
  {
    if(OCR2A < 254)
    {
      OCR2A++;
      Serial << F("+"); /* monitor show that period increased */
    }
  }
#endif
}

/* $Log: LiFePO2_1.ino $
 */
