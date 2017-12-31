/**
 * @file       CH_Gsm.h
 * @author     Viacheslav Komarov
 * @license    LGPL-3.0
 *
 * @date       DEC 2017
 */


#ifndef CH_Gsm_h

#define CH_Gsm_h
#define TINY_GSM_DEBUG Serial
#include <TinyGsmClientA6.h>

enum ProcessStatus {
  PRO_IDLE= 1,
  PRO_REQUEST= 2,
  PRO_RESPONSE= 3,
  PRO_TIMEOUT= 4,
  PRO_OK_ROAMING= 5,
  PRO_UNKNOWN= 0,
};

class CH_Gsm : public TinyGsm {
public:
	CH_Gsm (HardwareSerial& stream, long rate = 115200, HardwareSerial& gps = Serial1 ) // Stream& stream
		: TinyGsm(stream) 		
	{	
		
		serialAT = &stream;
		if (gps){
			serialGPS = &gps;
			gps.begin(9600); 
		}
		stream.begin(rate);	
		DBG("### begin: ", rate);
		funcTimeOut = funcResponse = &doNothing;
		status = PRO_IDLE;		
	}
		
	#if defined(PIN_PWRKEY)
		void pwrkeyOn(){
			pinMode (PIN_PWRKEY, OUTPUT);
			digitalWrite(PIN_PWRKEY, HIGH);
			DBG("### pwrkey on");
			funcTimeOut = &pwrkeyOff;
			waitTimeOut = 2100; // Power button, >1.9V more than 2s to boot;
			status = PRO_REQUEST;
		}
		void pwrkeyOff(){
			digitalWrite(PIN_PWRKEY, LOW);
			DBG("### pwrkey off");			
		}
	#endif
	void handler(){	
		int c;
		switch (status) {
			case PRO_REQUEST:
				if (shoutRequest()){
					startMillis = millis();
					status = PRO_RESPONSE;
				}
				break;	
				
			case PRO_RESPONSE: 
				if (listenResponce()){ 
					if (takeResponse()){
						status = PRO_IDLE;
					}
					else{
						startMillis = millis();						
					}
				}
				else if ((millis() - startMillis) > waitTimeOut){
					status = PRO_TIMEOUT;					
				}				
				break;
				
			case PRO_TIMEOUT:
				timeoutAction();
				status = PRO_IDLE;
				
			case PRO_IDLE:
				idleAction();
				break;
				
			default:
				break;
		}
		if (serialGPS){
			while (serialGPS->available()) {							
				Serial.write(serialGPS->read());				
			}			
		}		
	}
	bool shoutRequest(){
		DBG("### Request: ");
		return true;
	}
	
	int listenResponce(){		
		int index = waitResponse(0, data);
		if(index) {
			DBG("### response:", data);
			data = "";
		}
		return index;
	}
	
	bool takeResponse(){
		return true;
	}
	
	bool timeoutAction(){
		(this->*funcTimeOut)();
		funcTimeOut = &doNothing;
		return true;
	}
	
	void doNothing(){
		
	}
	
	bool idleAction(){
		int c;
		if (serialAT->available()) {
			c = serialAT->read();
			while (c != -1){
				Serial.write(c);
				c = serialAT->read();
			}			
		}	
	
		if (Serial.available()) {
			c = Serial.read();
			while (c != -1){			
				serialAT->write(c);
				c = Serial.read();
			}
		}
		return false;
	}

private:
	void (CH_Gsm::*funcTimeOut)();
	void (CH_Gsm::*funcResponse)();
	String data;
	unsigned long waitTimeOut; 
	unsigned long startMillis;
	ProcessStatus	status;
	HardwareSerial*	serialAT;
	HardwareSerial*	serialGPS;
};

#endif	//ndef CH_Gsm_h