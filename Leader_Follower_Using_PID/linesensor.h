// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

# define EMIT_PIN    11    // Documentation says 11.
# define LS_LEFT_PIN 12   // Complete for DN1 pin
# define LS_MIDLEFT_PIN A0   // Complete for DN2 pin
# define LS_MIDDLE_PIN A2   // Complete for DN3 pin
# define LS_MIDRIGHT_PIN A3   // Complete for DN4 pin
# define LS_RIGHT_PIN A4   // Complete for DN5 pin

// Store our pin numbers into an array, which means
// we can conveniently select one later.
// ls(line sensor)_pin
int ls_pins[5] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN };
// Class to operate the linesensor(s).
class LineSensor_c {
  public:
    
    // Constructor, must exist.
    LineSensor_c() {

    } 

  void init(){

    // Set some initial pin modes and states
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( 12, INPUT );     // Set line sensor pin to input
  pinMode( A0, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A4, INPUT );

  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");



   
  }

  int readLineSensor(int ls_serial){

    if( ls_serial < 0 ) {
        return ('garbage');
    }
    if( ls_serial > 4 ) {
        return ('garbage');
    }
    else
    {
    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );

    // In this line, we retrieve the pin value
    // stored in the array "ls_pins" at location
    // "number".  So it is like a look-up table.
    // We can think of ls_pins in memory like:
    // Index 0, Index 1, Index 2, Index 3, Index 4
    //[  DN1  ][   DN2 ][  DN3  ][  DN4  ][  DN5  ]
    pinMode( ls_pins[ ls_serial ], OUTPUT );

    digitalWrite( ls_pins[ ls_serial ], HIGH );
    delayMicroseconds( 10 );

    pinMode( ls_pins[ ls_serial ], INPUT );
    
    unsigned long start_time = micros();

    while( digitalRead( ls_pins[ ls_serial ] ) == HIGH ) {
        // Do nothing here (waiting).
    }

    unsigned long end_time = micros();

    pinMode( EMIT_PIN, INPUT );

    unsigned long elapsed_time = end_time - start_time;

    // Give the result back to wherever this
    // function was called from.
    return (float)elapsed_time;
    }
  }
};



#endif
