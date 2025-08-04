// TYPE        // FUNCTION
uint8_t ledPin = PA8;   // d output    // to led    
uint8_t INT1 = PB5;     // d input     // for reed switch
uint8_t INT2 = PB12;    // d input     // for opto
uint8_t on_off = PA15;  // d output    // to gps module input
uint8_t gps_v_en = PA0; // d output    // to mosfet gate
uint8_t vbat_en = PA1;  // d output    // to bjt base
uint8_t adc4 = PA10;    // a input     // for battery voltage divider 
uint8_t adc1 = PB3;     // a input     // for analog sensor
uint8_t sen_v_en = PA9; // d output    // to mosfet gate


void setup() {


    pinMode(ledPin, OUTPUT);
    pinMode(INT1, INPUT);
    pinMode(INT2, INPUT);

    pinMode(on_off, OUTPUT);
    pinMode(gps_v_en, OUTPUT);
    pinMode(vbat_en, OUTPUT);
    pinMode(sen_v_en, OUTPUT);

    

    Serial.begin(9600);
    Serial1.begin(9600);

    analogReadResolution(12);

////////////////////////////////////////////////////////////////////////// TO TURN ON THE GPS    

    digitalWrite(gps_v_en, HIGH); // turn on gps voltage with LOW
    digitalWrite(on_off, LOW);	// turn off gps with LOW
    delay(1000);

    digitalWrite(gps_v_en, LOW); // turn on gps voltage with LOW
    digitalWrite(on_off, LOW);	// turn off gps with LOW
    delay(1000);

    digitalWrite(gps_v_en, LOW); // turn on gps voltage with LOW
    digitalWrite(on_off, HIGH);	// turn on gps with LOW
    delay(1000);   


    // digitalWrite(vbat_en, LOW);	 //turn off batery reading with LOW
    // digitalWrite(sen_v_en, HIGH); // turn on  sensor voltage with LOW

    // delay(500);

    ////////////////////////////////////////////////////////////////////////////////


    // ////TURN GPS OFF
    // digitalWrite(gps_v_en, HIGH); // turn on gps voltage with LOW
    // digitalWrite(on_off, LOW);	// turn off gps with LOW
    // ///


    digitalWrite(ledPin, HIGH);	// LED turn on when input pin value is LOW
    delay(200);
    digitalWrite(ledPin, LOW);	// LED turn off when input pin value is HIGH
    delay(200);

    digitalWrite(ledPin, HIGH);	// LED turn on when input pin value is LOW
    delay(200);
    digitalWrite(ledPin, LOW);	// LED turn off when input pin value is HIGH
    delay(200);

    Serial1.println("$PCAS10,0*1C");//HOT START
    delay(200);
    Serial1.println("$PCAS10,0*1C");
    delay(200);

    Serial1.println("$PCAS04,1*18");//GPS ONLY
    delay(200);
    Serial1.println("$PCAS04,1*18");
    delay(200);

    Serial1.println("$PCAS03,1,1,1,1,1,1,1,1*02");//ALL COMMANDS DISPLAYED
    delay(200);
    Serial1.println("$PCAS03,1,1,1,1,1,1,1,1*02");
    //Serial1.println("$PCAS03,1,0,1,0,0,0,0,0*02");//ONLY GGA AND GSA
    delay(200);

}

void loop() {
  // if (Serial.available()) {        // If anything comes in Serial (USB), try not to send a thing here just yet
  //   Serial1.write(Serial.read());  // read it and send it out Serial1 (pins 0 & 1)
  // }

  if (Serial1.available()) {       // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(Serial1.read());  // read it and send it out Serial (USB)
  }


// Serial.println(analogRead(adc4));
// delay(100);
// digitalWrite(vbat_en, HIGH);	 // turn on vbatery reading with HIGH, 3.7v to 414, 3.35 to 375 , 4.2 470
// delay(2000);

// Serial.println(analogRead(adc4));
// delay(100);
// digitalWrite(vbat_en, LOW);	 // turn off vbatery reading with LOW
// delay(2000);  




  // Serial.println(analogRead(adc1));
  // delay(100);
  // digitalWrite(sen_v_en, HIGH);	 // turn on vbatery reading with HIGH
  // delay(2000);

  // Serial.println(analogRead(adc1));
  // delay(100);
  // digitalWrite(sen_v_en, LOW);	 // turn on vbatery reading with HIGH
  // delay(2000); 

    // delay(100);
    // Serial.println("(Wait 10 seconds or Press any key to wakeup)");
    // api.system.sleep.all(10000);


}