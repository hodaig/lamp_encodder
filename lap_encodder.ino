
/***************************
 * Setup configurationd
 ***************************/

// Rotary Encoder Inputs
#define ENC_PIN_A  2
#define ENC_PIN_B  3
#define ENC_SW_PIN 4

// Led light output
#define OUTPUT_PIN 5


/***************************
 * Encodder reading
 ***************************/
// Settings
#define ENC_MAX_COUNT 30
#define ENC_MIN_COUNT 0

// >> 00 10 11 01 00
// << 00 01 11 10 00
int last_state = 0b00;
int counter = 0;
unsigned long lastButtonPress = 0;

void encoderSetup() {
        
  // Set encoder pins as inputs
  pinMode(ENC_PIN_A,INPUT_PULLUP);
  pinMode(ENC_PIN_B,INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(0, encoderChange, CHANGE); // ENC_PIN_A
  attachInterrupt(1, encoderChange, CHANGE); // DT

  // Read the initial state
  last_state = digitalRead(ENC_PIN_A) << 1 | digitalRead(ENC_PIN_B);
}

void encoderLoop() {
  // TODO - reduce counter to avoid reading errors accumulate
}

void encoderChange() {
  int cur_state = digitalRead(ENC_PIN_A) << 1 | digitalRead(ENC_PIN_B);
  int dir = 0;

  switch (last_state) {
    case 0b00:
      if (cur_state == 0b10) {
        dir = +1;
      } else if (cur_state == 0b01) {
        dir = -1;
      }
    break;
    case 0b10:
      if (cur_state == 0b11) {
        dir = +1;
      } else if (cur_state == 0b00) {
        dir = -1;
      }
    break;
    case 0b11:
      if (cur_state == 0b01) {
        dir = +1;
      } else if (cur_state == 0b10) {
        dir = -1;
      }
    break;
    case 0b01:
      if (cur_state == 0b00) {
        dir = +1;
      } else if (cur_state == 0b11) {
        dir = -1;
      }
    break;
  }


  int tmp = counter + dir;
  if (ENC_MIN_COUNT <= tmp && tmp <= ENC_MAX_COUNT) {
    counter = tmp;
  }

#if DEBUG
  if (last_state != cur_state) {
    Serial.print(counter);
    Serial.print(" - ");
    Serial.println(cur_state);
  } else {
    // TODO assert
  }
#endif

  last_state = cur_state;

    // Read the button state
  int btnState = digitalRead(ENC_SW_PIN);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println("Button pressed!");
    }

    // Remember last button press event
    lastButtonPress = millis();
  }
  
}


/***************************
 * Output calculation - TODO
 ***************************/
// #define TOTAL_STEPS 50
// #define MAX_SPEED_STEPS_4_SEC 10000
// #define MIN_SPEED_STEPS_4_SEC 100
// #define MAX_SPEED_CHANGE_4_STEP 100
// #define MIN_SPEED_CHANGE_4_STEP 1

// int rotationsToDimSteps(int rotations, int d_ms) {
// 	int speed_sec = 1000 * steps / d_ms;
// 	if (speed_sec > MAX_SPEED_STEPS_4_SEC) { speed_sec = MAX_SPEED_STEPS_4_SEC; }
// 	if (speed_sec < MIN_SPEED_STEPS_4_SEC) { speed_sec = MIN_SPEED_STEPS_4_SEC; }
	
// 	int factor = my_map(speed_sec, MIN_SPEED_STEPS_4_SEC, MAX_SPEED_STEPS_4_SEC, MIN_SPEED_CHANGE_4_STEP, MAX_SPEED_CHANGE_4_STEP);
// 	int change = factor * steps;
	
// 	return change;
// }

void outputSetup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  analogWrite(OUTPUT_PIN, 0);

#if DEBUG
  pinMode(12, OUTPUT);
  analogWrite(lightPin, 0);
  pinMode(13, OUTPUT);
  analogWrite(lightPin, 0);
#endif
}

void outputLoop() {
  int scaled_counter;
    
  if (counter < 10) {
    scaled_counter = counter;
  } else {
    scaled_counter = map(counter, 10, ENC_MAX_COUNT, 10, 255);
  }

#if DEBUG
  digitalWrite(12, scaled_counter == 0);
  digitalWrite(13, scaled_counter == 255);
#endif

  analogWrite(OUTPUT_PIN, 1);
}

/***************************
 * Encodder button task - TODO
 ***************************/

/***************************
 * Main
 ***************************/
void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);

  encoderSetup();
  encoderSetup();
}

void loop() {

  encoderLoop();
  outputLoop();

  delay(1);
}
