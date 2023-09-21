
/***************************
 * Setup configurationd
 ***************************/

// Rotary Encoder Inputs
#define ENC_PIN_A  2
#define ENC_PIN_B  3
#define ENC_BTN_PIN 4

// Led light output
#define OUTPUT_PIN 5

// Debug mode
// #define DEBUG

#ifdef DEBUG
#  define DEBUG_PRINTLN(_x) Serial.println(_x)
#  define DEBUG_PRINT(_x)   Serial.print(_x)
#else
#  define DEBUG_PRINTLN(_x) do {;} while(0)
#  define DEBUG_PRINT(_x)   do {;} while(0)
#endif

/***************************
 * Globals
 ***************************/
int counter = 0;

/***************************
 * Encodder reading
 ***************************/

// >> 00 10 11 01 00
// << 00 01 11 10 00
int last_enc_state = 0b00;

void encoderSetup() {
        
  // Set encoder pins as inputs
  pinMode(ENC_PIN_A,INPUT_PULLUP);
  pinMode(ENC_PIN_B,INPUT_PULLUP);
  attachInterrupt(0, encoderChange, CHANGE); // ENC_PIN_A
  attachInterrupt(1, encoderChange, CHANGE); // DT

  // Read the initial state
  last_enc_state = digitalRead(ENC_PIN_A) << 1 | digitalRead(ENC_PIN_B);
}

void encoderLoop() {
  // TODO - reduce counter to avoid reading errors accumulate
}

void encoderChange() {
  int cur_enc_state = digitalRead(ENC_PIN_A) << 1 | digitalRead(ENC_PIN_B);
  int dir = 0;

  switch (last_enc_state) {
    case 0b00:
      if (cur_enc_state == 0b10) {
        dir = +1;
      } else if (cur_enc_state == 0b01) {
        dir = -1;
      }
    break;
    case 0b10:
      if (cur_enc_state == 0b11) {
        dir = +1;
      } else if (cur_enc_state == 0b00) {
        dir = -1;
      }
    break;
    case 0b11:
      if (cur_enc_state == 0b01) {
        dir = +1;
      } else if (cur_enc_state == 0b10) {
        dir = -1;
      }
    break;
    case 0b01:
      if (cur_enc_state == 0b00) {
        dir = +1;
      } else if (cur_enc_state == 0b11) {
        dir = -1;
      }
    break;
  }


  counter += dir;

  if (last_enc_state != cur_enc_state) {
    DEBUG_PRINT(counter);
    DEBUG_PRINT(" - ");
    DEBUG_PRINTLN(cur_enc_state);
  } else {
    // TODO assert
  }

  last_enc_state = cur_enc_state;
  
}


/***************************
 * Output calculation
 ***************************/
#define OUTPUT_REFRESH_RATE_MS 100

#define MAX_SPEED_STEPS_4_SEC 50
#define MIN_SPEED_STEPS_4_SEC 10
#define MAX_SPEED_CHANGE_4_STEP 30
#define MIN_SPEED_CHANGE_4_STEP 1

unsigned long lastOutUpdate;
float         OutValue;

float rotationsToDimSteps(int rotations, int d_ms) {
  if (0 == d_ms) {
    d_ms = 1;
  }
	long speed_sec = abs(rotations * (1000/ d_ms));
	if (speed_sec > MAX_SPEED_STEPS_4_SEC) { speed_sec = MAX_SPEED_STEPS_4_SEC; }
	if (speed_sec < MIN_SPEED_STEPS_4_SEC) { speed_sec = MIN_SPEED_STEPS_4_SEC; }
	
	int factor = map(speed_sec, MIN_SPEED_STEPS_4_SEC, MAX_SPEED_STEPS_4_SEC, MIN_SPEED_CHANGE_4_STEP, MAX_SPEED_CHANGE_4_STEP);
	float change = factor * rotations;
	
	return change;
}

void outputSetup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  analogWrite(OUTPUT_PIN, 0);

  lastOutUpdate = millis();
  OutValue      = 0;

#ifdef DEBUG
  pinMode(12, OUTPUT);
  analogWrite(12, 0);
  pinMode(13, OUTPUT);
  analogWrite(13, 0);
#endif
}

void outputLoop() {
#ifdef LEGACY_OUT
  if (counter < 10) {
    OutValue = counter;
  } else {
    OutValue = map(counter, 10, ENC_MAX_COUNT, 10, 255);
  }
#else
  unsigned long d_ms      = (millis() - lastOutUpdate);
  // TODO - consider to fix the "encoder andvace by 2" in the encodder algorithem
  int           t_counter = counter / 2; // each physical encodder step produce 2 countable steps
  if (t_counter && d_ms > OUTPUT_REFRESH_RATE_MS) {
    lastOutUpdate = millis();
    counter = 0;
    DEBUG_PRINT("[");
    DEBUG_PRINT(OutValue);
    DEBUG_PRINT("] d_ms = ");
    DEBUG_PRINT(d_ms);
    DEBUG_PRINT(" t_counter = ");
    DEBUG_PRINT(t_counter);
    DEBUG_PRINT(" rot = ");
    DEBUG_PRINT(rotationsToDimSteps(t_counter, d_ms));
    OutValue += rotationsToDimSteps(t_counter, d_ms);
    DEBUG_PRINT("[");
    DEBUG_PRINT(OutValue);
    DEBUG_PRINTLN("]");
    if (OutValue < 0) {
      OutValue = 0;
    } else if (OutValue > 255) {
      OutValue = 255;
    }
  }
#endif

#ifdef DEBUG
  digitalWrite(12, OutValue == 0);
  digitalWrite(13, OutValue == 255);
#endif

  analogWrite(OUTPUT_PIN, OutValue);
}


/***************************
 * Encodder button task
 ***************************/
unsigned long lastBtnPress;
int           lastBtnRead;
int           lastBtnState;

void btnSetup() {
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  // Read the initial state
  lastBtnPress = millis();
  lastBtnRead  = digitalRead(ENC_BTN_PIN);
  lastBtnState = digitalRead(ENC_BTN_PIN);
}

void btnLoop() {
  // Read the button state
  int btnState = digitalRead(ENC_BTN_PIN);

  if (btnState != lastBtnRead) {
    // Remember last button event
    lastBtnPress = millis();
    lastBtnRead = btnState;
  }

  if (millis() - lastBtnPress < 50) {
    // reed state may be unstable wait to see continuos signal 
    return;
  }

  if (lastBtnState != btnState) {
    // button state has changed
    if (btnState == LOW) {
      // pressed
      DEBUG_PRINTLN("Button pressed!");
    } else {
      // Released
      DEBUG_PRINTLN("Button released!");
    }

    lastBtnState = btnState;
  }
}

/***************************
 * Main
 ***************************/
void setup() {
#ifdef DEBUG
  // Setup Serial Monitor
  // Serial.begin(9600);
  Serial.begin(115200);
#endif

  encoderSetup();
  outputSetup();
  btnSetup();
}

void loop() {

  encoderLoop();
  outputLoop();
  btnLoop();

  delay(1);
}
