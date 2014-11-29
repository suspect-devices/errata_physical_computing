title: 'Errata for Physical Computing'
---

Chapter 4: The Microcontroller {.page-title itemprop="headline"}
==============================

“Hello World!” is the Hard Part
--------------------------------

I don't actually subscribe to this supposition, but that's ok.

### Identifying the Pins on the Microcontroller (p51)

![Atmega168PinMap2][]

### Hello (p57)

The Basic STAMP-like environments described in this section can be replaced
by the Arduino IDE. After downloading and running the software according to
the docs at arduino.cc, you should select the correct board and serial port
from the `Tools` menu and then select `File → Examples → 01.Basics → Blink`

    /*
      Blink
      Turns on an LED on for one second, then off for one second, repeatedly.

      This example code is in the public domain.
     */

    // Pin 13 has an LED connected on most Arduino boards, including the Leonardo.
    // Give it a name:
    int led = 13;

    // The setup routine runs once when you press reset:
    void setup() {
      // initialize the digital pin as an output.
      pinMode(led, OUTPUT);
    }

    // The loop routine runs over and over again forever:
    void loop() {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);               // wait for a second
    }


Chapter 5: Programming
=====================

Chapter 5 has a lot of good starting material, but it's
all written in dialects of Basic. I will explain as well as translate the
code examples.

Loops (p66)
-----------

The event loop that the book refers to is hidden by the Arduino IDE, but
the event loop used by the Arduino core version 1.0.3 uses a
“for” loop.

    for (;;) {
      loop();
      if (serialEventRun) serialEventRun();
    }

This can also be written as:

    while(true)
      loop();
      if (serialEventRun) serialEventRun();
    }

If Statements (p67)
-------------------

    if (in6 == 1) {
       /*Statement1*/;
    }
    /*Statement2*/

Please note the == to indicate testing for equality. The statement

    if (in6 = 1)

would set the value of *in6* to 1 and return 1 which would be true.
If only one statement is part of the condition, the brackets are optional
(as you can see in the code examples above). However, they
remove any potential ambiguity and I highly suggest you use them.

Variables (p69)
----------------

In C, variables must be declared with a type. They can be initialized and
declared at the same time. In standard C, they must be declared at the
top of a function or, if global, outside of the function. In C++, which
the Arduino uses, they can be inserted as needed.

    int Date=12;
    int ticketValue = 250;
    int Fare=125

Variable type sizes can be different between machines. In general,
an `int` is a short int which is usually 16 bits and a `char` or character is 8 bits,
but this is not guaranteed. Integers can be either signed or unsigned. **EXPLAIN WHAT SIGNED INTEGERS ARE??**
The Arduino provides some types that help you specify exactly what you
mean.

    uint8_t Sensor; // an 8 bit unsigned variable
    uint8_t Ticket;
    uint16_t Bigger; // this is a 16 bit unsigned variable

And the code on page 71 could be written thusly:

    uint8_t ticketsSubmitted=0;
    uint8_t ticketSensor=0;

    loop {
      if (ticketSensor==1) {
           ticketsSubmitted++;
      }
      if (ticketSubmitted ==3) {
           OpenGate();
           ticketsSubmitted=0;
      } 
    }

## Subroutines

In C, subroutines are functions which return void as a result. The
Arduino core provides a rich set of built-in functions to interact
easily with pins. In addition, the underlying GCC **EXPLAIN** provides defines
that allow you to interact directly with the processors registers.

    digitalWrite(13,HIGH);
    PORTB |= _BV(0);

Both lines above set pin 0 of port B on the Arduino to high.

## Constants (p73)

Constants in C are handled by the preprocessor which replaces the value **PHRASING IS UNCLEAR**
before compiling the code. Unlike using variables, using preprocessor
macroes does not cost you any memory to store the values. C convention
has macros in all caps.

    #define MIN_PULSE 100
    PulseWidth = MIN_PULSE + angle;

Which would expand to

    PulseWidth = 100 + angle;

### Example (p74)

    #define MY_FAVORITE_PIN_NO 14
    #define MY_FAVORITE_PINS PINB
    #define MY_FAVORITE_PIN PINB0
    printState=digitalRead(MY_FAVORITE_PIN_NO);
    printState=MY_FAVORITE_PINS & _BV(MY_FAVORITE_PIN); // does more or less the same as the above.

C values are passed to routines through a stack, become local to
the function, and go away when it exits. To get a function to modify a
value like the examples at the bottom of page 74, you would pass the
address of the value that needs to be changed. This is called “pass by
reference”

    int sensor;
    rctime(5,1,&sensor); // the and here means "the memory address of sensor"

Homemade Routines
------------------

In C you cannot use a function until you have declared it. The Arduino
hides this from you by adding the declarations for you. For this reason
you can write.

    #define THANK_YOU_LIGHT 13
    void setup() {
    // your setup goes here.
    }

    void loop() {
       if (theySneezeOnYou) {
         myThankYouRoutine();
       }
       if (theySneezeOnYou) {
         myThankYouRoutine();
       }
       if (theySneezeOnYou) {
         myThankYouRoutine();
       }
    }

    void myThankYouRoutine() {
    digitalWrite(THANK_YOU_LIGHT, HIGH);
       delay(1000);
       digitalWrite(THANK_YOU_LIGHT, LOW);
       delay(1000);
    };

However, it is good form to declare functions.

    #define THANK_YOU_LIGHT 13
    // routines defined in this file.
    void myThankYouRoutine();
    void setup();
    void loop();

    void setup() {
       ...

### Advanced Loops:

The code below is the main program file from Arduino 22.

    #include <Wprogram.h>

    int main(void)
    {
        init();
        setup();
        for (;;)
            loop();  
        return 0;
    }

It uses a form of C’s for loop. This could have been done with a
while(true) statement as well.

#### While-End Do-While (p77)

    while(digitalRead(5)==0) {
        digitalWrite(6,HIGH);
        delay(250);
        digitalWrite(6,LOW);
        delay(250);
    }

C also supports a do-until loop structure.

    do {
        //.... stuff to do here ...
    } until (someThinIsTrue);

#### For-Next (p78)

The Basic code on page 78 would look like this in Arduino.

    digitalWrite(5,HIGH);
    delay(1000);
    digitalWrite(6,HIGH);
    delay(1000);
    digitalWrite(7,HIGH);
    delay(1000);

Which in a C for loop would look more like this.

    //... fragment ...
    uint8_t counter;
    //... later ...
    for (counter=0;counter<=15;counter++) {
       digitalWrite(counter+5,HIGH);
       delay(1000);
    }

The for loop in C is a litte more flexible than the Basic for-next as
we have already seen.

Its basic form is this:

    for( initialexpression; testexpression; iterateexpression)
    {
       /*stuff to do*/;
    }

First, `initialexpression` is executed before the loop, then `testexpression`
is evaluated to see if it is true. If the _stuff to do_ is done,
`iterateexpression` is executed last.

*Note: Any or all of these expressions can be omitted. If
the test expression is omitted, then it evaluates to true.
If all expressions are ommitted, it becomes a while loop as
seen at the beginning of the chapter.*

## Comments (p81)

Comments in C come in the traditional form `/\* comment \*/` and the newer
C++ style `//`

    /*
     * This is a multi line comment.
     * The next line is starts the main "loop"
     */
    void loop() {
      if (ticketValue > 0){ // check the tickets value
         takeFare()
      } // endif
    } /* end of main loop */

## Debugging (p82)

There is a common convention in the Arduino to assume that all debugging
information should be printed with `Serial.print()` and
`Serial.println()`. This is a bad habit that you will see in almost every
Arduino program.

    serial.println("Hello World");
    Serial.println("start of routine");
    Serial.print("fare = ");
    Serial.println(fare, DEC);

With a little work, a more flexible system can be worked out that not
only distinguishes between printing and debugging but also allows you to
point your debugging messages where you want (or even turn them off).

    #include <stdarg.h>
    void SerialPrintFormatted(char *fmt, ... ){
            char tmp[128]; // resulting string limited to 128 chars
            va_list args;
            va_start (args, fmt );
            vsnprintf(tmp, 128, fmt, args);
            va_end (args);
            Serial.println(tmp);
    }
    #define DEBUG(...) SerialPrintFormatted(__VA_ARGS__);

Then you can write

    serial.println("Hello World");
    DEBUG("start of routine");
    DEBUG("fare = %d", fare);


Chapter 6: The “Big Four”: Schematics, Programs, and Transducers
===============================================================

## Example (p92)

    /* ... include debug code from previous chapter ...*/

    #define INPUT_PIN 7
    uint8_t x;
    //declare a global variable called x
    void setup {
       Serial.begin(9600); //for debug
       pinMode(INPUT_PIN_NO,INPUT);
    }

    void loop {
       x=digitalRead(INPUT_PIN);
       DEBUG("x = %d", x);
    };

## Example (p94)

    #define INPUT1 2
    #define INPUT1 4
    #define INPUT1 6
    #define ALL_SWITCHES_OFF 7
    #define ANY_SWITCHES_ON 8
    #define ON HIGH
    #define OFF LOW

    void setup() {
      pinMode(INPUT1,INPUT);
      pinMode(INPUT2,INPUT);
      pinMode(INPUT3,INPUT);
      pinMode(ALL_SWITCHES_OFF,OUTPUT);
      pinMode(ANY_SWITCHES_ON,OUTPUT);
    }

    void loop() {
      digitalWrite(ANY_SWITCHES_ON,OFF);
      if(digitalRead(INPUT1)&&digitalRead(INPUT1)&&digitalRead(INPUT1)) {
         digitalWrite(ALL_SWITCHES_OFF,ON);
         //digitalWrite(ANY_SWITCHES_ON,OFF);//already done above!
         delay(100);
         digitalWrite(ALL_SWITCHES_OFF,OFF);
         delay(100);
         digitalWrite(ALL_SWITCHES_OFF,ON);
         delay(100);
         digitalWrite(ALL_SWITCHES_OFF,OFF);
      } else {
         digitalWrite(ANY_SWITCHES_ON,ON);
         digitalWrite(ALL_SWITCHES_OFF,OFF);
      }
    }

## Example (p96)

    void loop(){
       digitalWrite(25,0);
       digitalWrite(26,1);
       delay(200);
       digitalWrite(25,0);
       digitalWrite(26,1);
       delay(200);
    }

## Example (p102)

    void loop(){
       digitalWrite(5,1);
       delay(300);
       digitalWrite(5,0);
       delay(300);
    }

Analog-to-Digital Converters (p108)
------------------------------------
![Photo on 2-5-13 at 5.48 PM \#2][]

    /*... setup and debug here ...*/

    int ADCValue;
    uint8_t OutputValue;

    void loop() {
    /* read the ADC on pin a0 */
    ADCValue=analogRead(0);
    DEBUG("ACDValue = %d", ADCValue);
    }

#### Rctime (p110)

If we really want to engage this 80s era technique, then here
is the Arduino documentation for RCTime.
 <http://arduino.cc/en/Tutorial/RCtime>

    /* RCtime
     *   Duplicates the functionality of the Basic Stamp's RCtime
     *   Allows digital pins to be used to read resistive analog sensors
     *   One advantage of this technique is that is can be used to read very wide ranging inputs.
     *   (The equivalent of 16 or 18 bit A/D)
     *

     Schematic
                      +5V
                       |
                       |
                      ___
                      ___    Sensing Cap
                       |      .001 ufd  (change to suit for required resolution)
                       |      (102) pfd
                       |
    sPin ---\/\/\/-----.
           220 - 1K    |
                       |
                       \   
                       /     Variable Resistive Sensor
                       \     Photocell, phototransistor, FSR etc.
                       /
                       |
                       |
                       |
                     _____ 
                      ___
                       _

    */

    int sensorPin = 4;              // 220 or 1k resistor connected to this pin
    long result = 0;
    void setup()                    // run once, when the sketch starts
    {
       Serial.begin(9600);
       Serial.println("start");      // a personal quirk
    }
    void loop()                     // run over and over again
    {

       Serial.println( RCtime(sensorPin) );
       delay(10);

    }

    long RCtime(int sensPin){
       long result = 0;
       pinMode(sensPin, OUTPUT);       // make pin OUTPUT
       digitalWrite(sensPin, HIGH);    // make pin HIGH to discharge capacitor - study the schematic
       delay(1);                       // wait a  ms to make sure cap is discharged

       pinMode(sensPin, INPUT);        // turn pin into an input and time till pin goes low
       digitalWrite(sensPin, LOW);     // turn pullups off - or it won't work
       while(digitalRead(sensPin)){    // wait for pin to go low
          result++;
       }

       return result;                   // report results
    }

At this point, if its important to actually emulate RCTime, you
should replace the function above with something more accurate and
appropriate for your current platform.

#### Pulse Width Modulation for Input (p111)

For what it's worth, the Arduino also supports `pulseIn()`

### LED Dimming (p114)

The LED dimming and motor speed control examples are included in David
Mellis’s LED fade example code which is included with the Arduino. If you
cut the bottom half of the loop where he walks the value back from
bright to off. ** FRAGMENT ** See Also <http://arduino.cc/en/Tutorial/Fade>

    int ledPin = 13;	// LED connected to digital pin 9

    void setup() {
    	// nothing happens in setup
    }

    void loop() { 
    	// fade in from min to max in increments of 5 points:
    	for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
    	    // sets the value (range from 0 to 255):
    	    analogWrite(ledPin, fadeValue);
    	    // wait for 30 milliseconds to see the dimming effect
    	    delay(30);
    	}
    }

For pins which have a \~ mark next to them on the Arduino, this is handled
quite nicely in hardware. For other pins, this is done using timer
interrupts.

### Generating Tones (p117)

    #define SPEAKER 9
    const int note[] = {
    262, // C
    277, // C#
    294, // D
    311, // D#
    330, // E
    349, // F
    370, // F#
    392, // G
    415, // G#
    440, // A
    466, // A#
    494, // B
    523  // C next octave
    };

    void setup() {
    }
    int thisNote;
    void loop() {
    	for(thisNote=0; thisNote<=9;thisNote++) {
    	    tone(SPEAKER,note[thisNote]);
    	    delay(1000);
    	    //noTone(SPEAKER);
    	}
    }

### RC Servo Motors (p121)

    #include <Servo.h> 

    #define MIN_ANGLE 0
    #define MAX_ANGLE 180
    #define SERVO_PIN

    Servo myservo;		// create servo object to control a servo 
    int pos = MIN_ANGLE;	// variable to store the servo position 

    void setup() {
    	myservo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object 
    }

    void loop() { 
    	for(pos = MIN_ANGLE; pos < MAX_ANGLE; pos += 1) // goes from 0 degrees to 180 degrees 
    	{   // in steps of 1 degree 
    	    myservo.write(pos); // tell servo to go to position in variable 'pos'
    	    delay(15);		// waits 15ms for the servo to reach the position
    	}
    }

### Scaling Functions (code, p130)

The Arduino provides an extremely handy function called map that makes
the example below fairly trivial.

    #include <Servo.h>

    #define SERVO_PIN 9
    #define SENSOR_PIN 0

    #define MAX_SENSOR_READING 130#define MIN_SENSOR_READING 5
    #define MIN_ANGLE 0
    #define MAX_ANGLE 180

    Servo myservo; 		// create servo object to control a servo 
    int pos = MIN_ANGLE; 	// variable to store the servo position 
    int sensor = MIN_ANGLE; 	// variable to store the sensor reading 

    void setup() 
    { 
    	    myservo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object 
    } 

    void loop() { sensor=analogRead(SENSOR_PIN);
    	    if (sensor<MIN_SENSOR_READING) sensor=MIN_SENSOR_READING;
    	    if (sensor>MAX_SENSOR_READING) sensor=MAX_SENSOR_READING;
    	    myservo.write(map(MIN_SENSOR_READING,MAX_SENSOR_READING,MIN_ANGLE,MAX_ANGLE);
    	    delay(20);		// waits 15ms for the servo to reach the position 
    }


Chapter 8: Physical Interaction Design
=======================================

### Multitasking (p191)

The defines below allow you to normalize the way you build the circuit.
Generally, it is easier to pull current down for LEDs than it is to source it.
Since the AVR has built in pull-up resistors, you can do a normally open switch
to ground with fewer components.

    #define INPUT_PIN 7
    #define LED_PIN 8
    #define LED_ON LOW
    #define LED_OFF HIGH
    #define SWITCH_ON LOW
    #define SWITCH_OFF HIGH

    bool needFlashingFlag;
    uint8_t timesFlashed;
    bool ledState;
    uint16_t counter =1;

    void setup() {
    	pinMode (INPUT_PIN, INPUT);
    	digitalWrite(INPUT_PIN, HIGH); // enable pullup.
    	pinMode (LED_PIN, OUTPUT);
    	digitalWrite(LED_PIN, LED_OFF);
    }

    void loop(){
    	if (digitalRead(INPUT_PIN)==SWITCH_ON) {
    		needFlashingFlag=true;
    	}
    	if (needFlashingFlag) {
    		counter-=1;
    		if (!counter){
    		counter=1000;
    		if (timesFlashed < 3) {
    			if (ledState==LED_OFF) {
    				timesFlashed += 1;
    				ledState=LED_ON;
    				digitalWrite(LED_PIN,ledState);
    			} else {
    				ledState=LED_OFF;
    				digitalWrite(LED_PIN,ledState);
    			}
    		} else {
    			timesFlashed=0;
    			needFlashingFlag=false;
    			counter=1;
    			ledState=LED_OFF;
    			digitalWrite(LED_PIN,ledState);
    			}
    		}
    	}
    }

## Edge Detection (p195):

    #define INPUT_PIN 7
    #define SWITCH_ON LOW
    #define SWITCH_OFF HIGH
    #include "debug.h"

    bool buttonState;
    bool lastButtonState;
    uint8_t buttonCount;

    void setup(){
    	pinMode(INPUT_PIN, INPUT);;
    	digitalWrite(INPUT_PIN, HIGH);
    }

    void loop (){
    	buttonState=digitalRead(INPUT_PIN);
    	if (buttonState != lastButtonState) {
    		if (buttonState == SWITCH_ON) {
    			buttonCount++;
    			DEBUG("Button is pressed");
    		} else {
    			DEBUG("Button is nos pressed");
    			DEBUG("Button hits: %d", buttonCount);
    		}
    		lastButtonState=buttonState;
    	}
    }

## Finding Peaks in an Analog Signal (p202)

![Photo on 2-9-13 at 11.33 PM][]

    #define ANALOG_PIN 0
    #include "debug.h"

    uint16_t peakValue=0;
    uint16_t noise=7;
    uint16_t sensorValue;
    uint16_t lastSensorValue=0;
    uint16_t threshold=300;

    void setup(){
    	INIT_DEBUG();
    }

    void loop (){
    	sensorValue=analogRead(ANALOG_PIN);
    	DEBUG("%d",sensorValue);
    	if (sensorValue>=threshold+noise){
    		if (sensorValue>=lastSensorValue+noise){
    			peakValue=sensorValue;
    		}
    	} else {
    		if(peakValue >= threshold){
    			DEBUG("peak reading: %d", peakValue);
    		}
    	peakValue = 0;
    	}
    	lastSensorValue=sensorValue;
    }

## Button Debouncing (p205)

    #define SWITCH_ON LOW
    void setup () {
    	pinMode(SWITCH_PIN, INPUT);
    	digitalWrite(SWITCH_PIN, HIGH);
    }

    void loop(){
    	if(digitalRead(SWITCH_PIN) == SWITCH_ON){
    		delay(10);
    		if(digitalRead(SWITCH_PIN) == SWITCH_ON) {
    			switchOn=true;
    		} else {
    			switchOn=false;
    		}
    	}
    }


## Smoothing, Sampling, and Averaging (p207)

```
    #define ANALOG_PIN		0
    #define HISTORY_SIZE	4
    #include "debug.h"

    uint8_t positionInPastArray=HISTORY_SIZE;
    uint16_t past[HISTORY_SIZE];
    uint16_t sortedPast[HISTORY_SIZE];
    uint16_t averageArray();
    uint16_t medianArray();

    void setup(){
    	INIT_DEBUG();
    	delay(500);
    }

    void loop (){
    	uint16_t temp;
    	uint16_t average;
    	uint16_t median;
    	temp=analogRead(ANALOG_PIN);
    	if (++positionInPastArray >= HISTORY_SIZE) {
    		positionInPastArray=0;
    	}
    	past[positionInPastArray]=temp;
    	average=averageArray();
    	median=medianArray();
    	DEBUG("Average = %d Median = %d", average,median);
    }

    uint16_t averageArray() {
    	long int total = 0;
    	int i;
    	for (i=0;i<=HISTORY_SIZE;i++){
    		total+=past[i];
    	}
    	return (uint16_t)(total/HISTORY_SIZE);
    	    }

    uint16_t medianArray() {
    	int total = 0;
    	int i,j;
    	int finger=-1;
    	for (i=0;i<=HISTORY_SIZE;i++){
    		for (i=0;j<=HISTORY_SIZE;j++){
    			if (past[i]>=past[j]){
    				finger++;
    			}
    		}
    		sortedPast[finger]=past[i];
    	}
    	for (i=0;i<=HISTORY_SIZE;i++){
    		DEBUG("sortedPast[%d]=%d",i,sortedPast[i]);
    	}
    	return (uint16_t)sortedPast[HISTORY_SIZE/2];
    	}
```


  [Atmega168PinMap2]: http://farm8.staticflickr.com/7101/7329403498_47d10925f3_b.jpg "Arduino Leonardo pin reference"
  [physical-computing]: http://thingadayforever.files.wordpress.com/2013/02/physical-computing.jpg?w=238
  [![physical-computing][]]: http://thingadayforever.files.wordpress.com/2013/02/physical-computing.jpg
  [Homemade Routines]: http://thingadayforever.files.wordpress.com/2013/02/0203132245.jpg?w=300
  [![Homemade Routines][]]: http://thingadayforever.files.wordpress.com/2013/02/0203132245.jpg
  [1]: /blahg/workshops/errata-for-physical-computing/chapter-5-programming/#respond
  [The Big Four]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-4-13-at-6-56-pm.jpg?w=300
  [![The Big Four][]]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-4-13-at-6-56-pm.jpg
  [Photo on 2-5-13 at 5.48 PM \#2]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-5-13-at-5-48-pm-2.jpg?w=150
  [![Photo on 2-5-13 at 5.48 PM \#2][]]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-5-13-at-5-48-pm-2.jpg
  [2]: /blahg/workshops/errata-for-physical-computing/chapter-6-the-big-four-schematics-programs-and-transducers/#respond
  [Photo on 2-9-13 at 11.33 PM]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-9-13-at-11-33-pm.jpg?w=150
  [![Photo on 2-9-13 at 11.33 PM][]]: http://thingadayforever.files.wordpress.com/2013/02/photo-on-2-9-13-at-11-33-pm.jpg
  [3]: /blahg/workshops/errata-for-physical-computing/chapter-8-physical-interaction-design/#respond
