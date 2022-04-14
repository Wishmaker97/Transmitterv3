#include "Adafruit_SHTC3.h"
#include <Wire.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> 
#include <ArduinoJson.h>
#include <SPI.h>
#include <LoRa.h>
#include <Bounce.h>

// CONSTANTS
const int RX_GAIN = 0;
const int TX_POWER = 23;
const long LORA_FREQUENCY =  433E6;                   // LORA FREQUENCY set to 433MHz

const long LOAD_CELL_SAMPLING_RATE = 1000;            // Sampling rate for the load cell in milliseconds
const float LOAD_CELL_THRESHOLD = 20.0;               // threshold value in kilograms
const long LOAD_CELL_TIMEOUT = 1000;                  // time out when waiting for response from loadcelll

const long CHARGING_SAMPLE_RATE = 10000;              // Sampling rate for charing status in milliseconds
const int ENABLE_LOADCELL = 2; 
const String NODE_ID = "PUL056A";


// PINS
const int CHARGER_DETECT = 6;                         // pin for Charge charger detection ( 1=charger_running or 0=charger_not_running)
const int CHARGING_STATUS = 5;                        // pin for Charge charging status detection ( 1=charging or 0=not_Charging)
const int POWER_SOURCE_DETECT = 7;                    // Pin for source chosen by Power IC ( internal battery = 0 or external = 1)
const int DRIVER_ENABLE = 17;                         // Pin for Transmit enable for Half duplex RS485 connection

const int HE1_PIN = 3;                                // Pin for Hall Effect Sensor #1
const int HE2_PIN = 4;                                // Pin for Hall Effect Sensor #2

const int LORA_SLAVE_SELECT = 10;                     // Pin for slave Select for LORA module
const int LORA_RESET = 16;                            // Pin for LORA_LORA_LORA_LORA_LORA_RESET PIN for LORA module
const int LORA_INTERRUPT = 9;                         // Pin for Interupt PIN for LORA module


// VARIABLES
volatile long counter = 0;                            // Variable to keep track of counter
volatile double voltage = 0;                          // Variable to keep track of LiPo voltage
volatile double soc = 0;                              // Variable to keep track of LiPo state-of-charge (SOC)
volatile bool alert;                                  // Variable to keep track of whether alert has been triggered

volatile bool HE1_interupt = false;                   // Boolean flag for Hall effect sensor one
volatile bool HE2_interupt = false;                   // Boolean flag for Hall effect sensor two
volatile int direction_HE = 0;                        // Direction variable 0=no_direction, 1=positive, -1=negative

String incomming_data_from_gateway = "";
char c;

volatile bool charger_flag = false;
volatile bool charging_flag = false;

long start_time = 0;
float last_loadcell_value = 0.0;

long start_time_charging = 0;


// CREATE SHTC3 INSTANCE
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
                                                                                                            
// CREATE MAX17048 INSTANCE
SFE_MAX1704X lipo(MAX1704X_MAX17048);

StaticJsonDocument<300> BootUp;

void HEInteruptOne()            // ISR for HE1_Pin
{
    HE1_interupt = true;
}

void HEInteruptTwo()            // ISR for HE2_Pin
{
    HE2_interupt = true;
}


String getLoadCellValue()
{
	String loadcell_string = "";
    
    // sending requestion to loadcell via halfduplex RS485
    Serial1.flush();
    digitalWrite(DRIVER_ENABLE, HIGH);
    delayMicroseconds(500);
    Serial1.print("!001:SYS?\r");
    delay(12);
    digitalWrite(DRIVER_ENABLE, LOW);
    Serial1.flush();
    
    // delay(50); 
    
    // waiting for Loadcell to respond
    long start_time_wait_1 = millis();
    while(Serial1.available()==0 && ((millis()-start_time_wait_1)<LOAD_CELL_TIMEOUT)){
    
    }
    
    // read Serial buffer byte by byte
    long start_time_wait_2 = millis();
    while(Serial1.available()>0 && ((millis()-start_time_wait_2)<LOAD_CELL_TIMEOUT)){
        loadcell_string=Serial1.readStringUntil('\r');
    }

    loadcell_string.remove(0,1);

    return loadcell_string;       
}

void chargerPlugged()
{
    int charger_plugged_in = digitalRead(CHARGER_DETECT);
    String message = "";
    String preamble = "";
    if (!charger_plugged_in){      
        charger_flag=true;
        preamble="E";    
        message="Charger Plugged-In";  
    }
    else{
        charger_flag=false; 
        message="Charger Un-Plugged";
        preamble="F"; 
    }    
    
    StaticJsonDocument<60> chargerPinChange;
    chargerPinChange["Message"] = message;
    
    LoRa.beginPacket();
	LoRa.print(NODE_ID);
    LoRa.print(preamble);
    serializeJson(chargerPinChange, LoRa);
    LoRa.endPacket();    
}

void LoRa_rxMode(){
	LoRa.enableInvertIQ();                // active invert I and Q signals
	LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
	LoRa.idle();                          // set standby mode
	LoRa.disableInvertIQ();               // normal mode
}

void onReceive(int packetSize) {
	
	while (LoRa.available()) {
		incomming_data_from_gateway += (char)LoRa.read();
	}
}

void onTxDone() {
	Serial.println("\nTransmission to Gateway Complete");
	LoRa_rxMode();
}

void setup() {
	// DEBUG SERIAL/LOADCELL data 
	Serial.begin(9600);
	Serial1.begin(9600);

	StaticJsonDocument<300> BootUp;
	BootUp["Node"] = NODE_ID;

	// TEMPERATURE AND HUMIDITY SENSOR
	if (!shtc3.begin()) {
		BootUp["SHTC3"] = "SHTC3 not detected.";
	}
	else{
		BootUp["SHTC3"] = "Found SHTC3 sensor";
	}    

	// FUEL GAUGE SENSOR
	if (!lipo.begin()){
		BootUp["FuelGauge"] = "MAX17048 not detected.";
	}
	else{
		BootUp["FuelGauge"] = "Found MAX17048 sensor";
	}

	//LORA SETTINGS
	LoRa.setPins(LORA_SLAVE_SELECT, LORA_RESET, LORA_INTERRUPT);
	if (!LoRa.begin(LORA_FREQUENCY)) {
		BootUp["LORA"] = "Starting LoRa failed!";
	}
	else{
		BootUp["LORA"] = "Starting LoRa RFM96W Successful!";
	}

	//CHARGER AND PROGRAMMER SETTINGS
	pinMode(CHARGER_DETECT, INPUT_PULLUP);
	pinMode(CHARGING_STATUS, INPUT_PULLUP);

	pinMode(POWER_SOURCE_DETECT, INPUT_PULLUP);
	pinMode(DRIVER_ENABLE, OUTPUT);	

	//HALL EFFECT SENSOR SETTINGS    
	pinMode(HE1_PIN,INPUT); //pull down resistor in PCB
	pinMode(HE2_PIN,INPUT); //pull down resistor in PCB

	attachInterrupt ( HE1_PIN, HEInteruptOne, RISING );
	attachInterrupt ( HE2_PIN, HEInteruptTwo, RISING );

	attachInterrupt(digitalPinToInterrupt(CHARGER_DETECT), chargerPlugged, FALLING);

	// LoRa.setGain(6); 
	LoRa.setSyncWord(0xF3); 

	digitalWrite(DRIVER_ENABLE, LOW);

	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
  	LoRa_rxMode();

	LoRa_txMode();
	LoRa.beginPacket();
	LoRa.print(NODE_ID);
	LoRa.print("A");
	serializeJson(BootUp, LoRa);
	LoRa.endPacket(true);

	serializeJson(BootUp, Serial);
}

void loop() {

    if (millis() - start_time > LOAD_CELL_SAMPLING_RATE){

        String loadcell_string = getLoadCellValue();
        float current_loadcell_float = loadcell_string.toFloat()*1000000.0;

        if ((loadcell_string != "") && (abs(last_loadcell_value-current_loadcell_float) > LOAD_CELL_THRESHOLD*1000.0)){

            //Read Temp & Humidity Sensor
            sensors_event_t humidity1, temp1;  
            shtc3.getEvent(&humidity1, &temp1);       // populate temp and humidity objects with fresh data
            
            // Read Fuel Gauge Sensor
            soc = lipo.getSOC();   
            
            StaticJsonDocument<200> LC_Data; 
            LC_Data["LC"] = loadcell_string;
            LC_Data["SOC"] = soc;
            LC_Data["T"] = temp1.temperature;
            LC_Data["RH"] = humidity1.relative_humidity;

			Serial.println();
			serializeJson(LC_Data, Serial);    
            
			LoRa_txMode(); 
            LoRa.beginPacket();
			LoRa.print(NODE_ID);
            LoRa.print("G");
            serializeJson(LC_Data, LoRa);
            LoRa.endPacket(true);  
			       
        }

        last_loadcell_value=current_loadcell_float;        
        start_time = millis();               
    }   
  	if (incomming_data_from_gateway.length()) {

		char data_type = incomming_data_from_gateway.charAt(0);  
        incomming_data_from_gateway.remove(0,1);       
        
        DynamicJsonDocument doc(200);
        DeserializationError error = deserializeJson(doc, incomming_data_from_gateway);
        
        if (!error){
			if(data_type == 'R'){
				
				counter=0;  
				// Read Temp & Humidity Sensor
				sensors_event_t humidity, temp;  
				shtc3.getEvent(&humidity, &temp);

				// Read Fuel Gauge Sensor
				soc = lipo.getSOC();

				StaticJsonDocument<200> Data;

				Data["C"] = counter;
				Data["SOC"] = soc;
				Data["T"] = temp.temperature;
				Data["RH"] = humidity.relative_humidity;

				LoRa_txMode();  
				LoRa.beginPacket();
				LoRa.print(NODE_ID);
				LoRa.print("D");
				serializeJson(Data, LoRa);
				LoRa.endPacket(true);    

				Serial.println();
				serializeJson(Data, Serial);         
			}
			else if(data_type == 'T'){
				
				sensors_event_t humidity, temp;  
				shtc3.getEvent(&humidity, &temp);

				// Read Fuel Gauge Sensor
				soc = lipo.getSOC();

				StaticJsonDocument<200> Data;

				Data["SOC"] = soc;
				Data["T"] = temp.temperature;
				Data["RH"] = humidity.relative_humidity;

				LoRa_txMode();  
				LoRa.beginPacket();
				LoRa.print(NODE_ID);
				LoRa.print("T");
				serializeJson(Data, LoRa);
				LoRa.endPacket(true);             
			}
			else if(data_type == 'J'){
				Serial.println("Identity Requested by gateway");

				sensors_event_t humidity, temp;  
				shtc3.getEvent(&humidity, &temp);

				// Read Fuel Gauge Sensor
				soc = lipo.getSOC();

				StaticJsonDocument<200> Data;

				Data["Node"] = NODE_ID;
				Data["SOC"] = soc;
				Data["T"] = temp.temperature;
				Data["RH"] = humidity.relative_humidity;

				LoRa_txMode();  
				LoRa.beginPacket();
				LoRa.print(NODE_ID);
				LoRa.print("K");
				serializeJson(Data, LoRa);
				LoRa.endPacket(true);  
                  
        	}
        }
		incomming_data_from_gateway="";
    }

    if (charger_flag && (millis()-start_time_charging > CHARGING_SAMPLE_RATE)){
      
		int charging_state = digitalRead(CHARGING_STATUS);   

		// Read Temp & Humidity Sensor
		sensors_event_t humidity, temp;  
		shtc3.getEvent(&humidity, &temp);       // populate temp and humidity objects with fresh data

		// Read Voltage and state of charger
		voltage = lipo.getVoltage();            // lipo.getVoltage() returns a voltage value (e.g. 3.93)  
		soc = lipo.getSOC();  

		if (charging_state==0){ 

			StaticJsonDocument<60> charging; 
			charging["Message"] = "Charging";
			charging["Voltage"] = voltage;
			charging["SOC"] = soc;
			charging["T"] = temp.temperature;
			charging["RH"] = humidity.relative_humidity;

			// sending telemetry data while charging
			LoRa_txMode(); 
			LoRa.beginPacket();
			LoRa.print(NODE_ID);
			LoRa.print("B");
			serializeJson(charging, LoRa);
			LoRa.endPacket(true);
		}
		else{

			StaticJsonDocument<100> charged;
			charged["Message"] = "Charged";
			charged["Voltage"] = voltage;
			charged["T"] = temp.temperature;
			charged["RH"] = humidity.relative_humidity;          
			charged["SOC"] = soc;

			// sending telemetry data when charging is complete
			LoRa_txMode(); 
			LoRa.beginPacket();
			LoRa.print(NODE_ID);
			LoRa.print("C");
			serializeJson(charged, LoRa);
			LoRa.endPacket(true);
		} 
		start_time_charging = millis();  

	}    

	noInterrupts();
	bool HE1_interupt_temp = HE1_interupt;
	bool HE2_interupt_temp = HE2_interupt;
	interrupts();

	int direction_HE_temp = direction_HE;  

	if (HE1_interupt_temp && HE2_interupt_temp){

		// Read Temp & Humidity Sensor
		sensors_event_t humidity, temp;  
		shtc3.getEvent(&humidity, &temp);       // populate temp and humidity objects with fresh data

		// Read Fuel Gauge Sensor
		soc = lipo.getSOC();                    // lipo.getSOC() returns the estimated state of charge (e.g. 79%)

		StaticJsonDocument<200> Data;

		counter+=direction_HE_temp;

		Data["C"] = counter;
		Data["SOC"] = soc;
		Data["T"] = temp.temperature;
		Data["RH"] = humidity.relative_humidity;

		LoRa_txMode(); 
		LoRa.beginPacket();
		LoRa.print(NODE_ID);
		LoRa.print("D");
		serializeJson(Data, LoRa);
		LoRa.endPacket(true); 

		Serial.println();
		serializeJson(Data, Serial);

		HE1_interupt = HE2_interupt = false;      
		direction_HE = 0;  
	}
	else if (HE1_interupt_temp&&!HE2_interupt_temp){
		direction_HE = -1;        
	}
	else if (!HE1_interupt_temp&&HE2_interupt_temp){
		direction_HE = +1;
	}
}