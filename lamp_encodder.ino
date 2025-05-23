
/***************************
 * Setup configurationd
 ***************************/

// Rotary Encoder Inputs
#define ENC_PIN_A   2
#define ENC_PIN_B   3
#define ENC_BTN_PIN 4

// Led light output
#define OUTPUT_PIN 9

// use timer directly (instead of analogWrite()) to get 16bit resolution PWM
// Only pin 9 and 10 support 16bit
#define USE_16BIT_PWM 1

#ifdef USE_16BIT_PWM
#  define ANALOGWRITE(_pin, _val) analogWrite16(_pin, _val);
#else // USE_16BIT_PWM
#  define ANALOGWRITE(_pin, _val) analogWrite(_pin, _val);
#endif // USE_16BIT_PWM

// Debug mode
#define DEBUG

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

#define BTN_STATE_MACHINE_MAKE(_gpio, _duration) ((BtnState)(_gpio | (_duration << 1)))
#define BTN_GET_GPIO(_state)                      ((_state & 0x1))
#define BTN_GET_DURATION_SEC(_state)              (((_state >> 1) & 0xFF))
typedef enum BtnState {
  BTN_STATE_PRESS                           ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_PRESS, 0),  
  BTN_STATE_RELEASE                         ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_RELEASE, 0),
  BTN_STATE_LONGPRESS                       ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_PRESS, 1),
  BTN_STATE_LONGPRESS_RELEASE               ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_RELEASE, 1),
  BTN_STATE_LONGPRESS_10_SEC                ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_PRESS, 10),
  BTN_STATE_LONGPRESS_10_SEC_RELEASE        ,//= BTN_STATE_MACHINE_MAKE(BTN_GPIO_RELEASE, 10),
} BtnState;

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
  // TODO - Optional, reduce counter to avoid reading errors accumulate
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
//     DEBUG_PRINT(counter);
//     DEBUG_PRINT(" - ");
//     DEBUG_PRINTLN(cur_enc_state);
  } else {
    // TODO assert
  }

  last_enc_state = cur_enc_state;
  
}


/***************************
 * Output calculation
 ***************************/
#define OUTPUT_REFRESH_RATE_MS  200
#ifdef USE_16BIT_PWM
#define OUTPUT_MAX_VALUE        255*100L //0x7FFF
#else // USE_16BIT_PWM
#define OUTPUT_MAX_VALUE        255
#endif // USE_16BIT_PWM
#define OUTPUT_MIN_VALUE        0

#define MAX_SPEED_STEPS_4_SEC 60
#define MIN_SPEED_STEPS_4_SEC 5
#ifdef USE_16BIT_PWM
#define MAX_SPEED_CHANGE_4_STEP 1000
#define MIN_SPEED_CHANGE_4_STEP 1
#else // USE_16BIT_PWM
#define MAX_SPEED_CHANGE_4_STEP 30
#define MIN_SPEED_CHANGE_4_STEP 1
#endif // USE_16BIT_PWM

unsigned long lastOutUpdate;
long          outValue;

long rotationsToDimSteps(int rotations, int d_ms) {
  if (0 == d_ms) {
    d_ms = 1;
  }
	long speed_sec = abs(rotations * (1000/ d_ms));
	if (speed_sec > MAX_SPEED_STEPS_4_SEC) { speed_sec = MAX_SPEED_STEPS_4_SEC; }
	if (speed_sec < MIN_SPEED_STEPS_4_SEC) { speed_sec = MIN_SPEED_STEPS_4_SEC; }
	
	long factor = map(speed_sec, MIN_SPEED_STEPS_4_SEC, MAX_SPEED_STEPS_4_SEC, MIN_SPEED_CHANGE_4_STEP, MAX_SPEED_CHANGE_4_STEP);
	long change = factor * rotations;
	
	return change;
}

void outputSetup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  ANALOGWRITE(OUTPUT_PIN, 0);

  lastOutUpdate = millis();
  outValue      = 0;

#ifdef DEBUG
  pinMode(12, OUTPUT);
  analogWrite(12, 0);
  pinMode(13, OUTPUT);
  analogWrite(13, 0);
#endif
}

long safeAdd(long val1, long val2, long minVal, long maxVal) {
  if ((maxVal - val1) < val2) {
    return maxVal;
  } else if (val2 < (minVal - val1)) {
    return minVal;
  } else {
    return val1 + val2;
  }
}

void outputLoop() {
#ifdef LEGACY_OUT
  if (counter < 10) {
    outValue = counter;
  } else {
    outValue = map(counter, 10, ENC_MAX_COUNT, 10, 255);
  }
#else
  unsigned long d_ms      = (millis() - lastOutUpdate);
  // TODO - consider to fix the "encoder andvace by 2" in the encodder algorithem
  int           t_counter = counter / 2; // each physical encodder step produce 2 countable steps
  if (t_counter && d_ms > OUTPUT_REFRESH_RATE_MS) {
    long stepsToChange = rotationsToDimSteps(t_counter, d_ms);
    
    lastOutUpdate = millis();
    counter = 0;

    DEBUG_PRINT("[");
    DEBUG_PRINT(outValue);
    DEBUG_PRINT("] d_ms = ");
    DEBUG_PRINT(d_ms);
    DEBUG_PRINT(" t_counter = ");
    DEBUG_PRINT(t_counter);
    DEBUG_PRINT(" rot = ");
    DEBUG_PRINT(stepsToChange);
    outValue = safeAdd(outValue, stepsToChange, OUTPUT_MIN_VALUE, OUTPUT_MAX_VALUE);
    DEBUG_PRINT("[");
    DEBUG_PRINT(outValue);
    DEBUG_PRINTLN("]");
    if (outValue < OUTPUT_MIN_VALUE) {
      outValue = OUTPUT_MIN_VALUE;
    } else if (outValue > OUTPUT_MAX_VALUE) {
      outValue = OUTPUT_MAX_VALUE;
    }
  }
#endif

#ifdef DEBUG
  digitalWrite(12, outValue == OUTPUT_MIN_VALUE);
  digitalWrite(13, outValue == OUTPUT_MAX_VALUE);
#endif

  ANALOGWRITE(OUTPUT_PIN, outValue);
}


/***************************
 * Encodder button task
 ***************************/
unsigned long lastBtnPress;
int           lastBtnRead;
int           lastBtnState;
int           btnFuncVal;

#ifdef USE_16BIT_PWM
#define BTN_FUNC_INTERVAL_MS      (100L)
#else // USE_16BIT_PWM
#define BTN_FUNC_INTERVAL_MS      (10000)
#endif // USE_16BIT_PWM

#define BTN_FUNC_FULL_2_OFF_TIME_MS     (1*60*1000L)
#define BTN_FUNC_ITER_SIZE              ((OUTPUT_MAX_VALUE * BTN_FUNC_INTERVAL_MS) / BTN_FUNC_FULL_2_OFF_TIME_MS)
#define BTN_DEBOUNCE_PERIOD_MS          (50)
#define BTN_1_SECOND_IN_MS              (1000)
#define BTN_GPIO_PRESS                  (0 /*LOW*/)
#define BTN_GPIO_RELEASE                (!BTN_GPIO_PRESS)

enum {
  BTN_FUNC_UNINIT,
  BTN_FUNC_READY,
  BTN_FUNC_ACTIVE,
} btnFuncStateMachine;

void btnStateChange(BtnState btnState) {

  switch (btnState) {
    case BTN_STATE_PRESS:
      DEBUG_PRINTLN("Button pressed!");
      break;
    case BTN_STATE_RELEASE:
      DEBUG_PRINTLN("Button short press released!");
      DEBUG_PRINTLN(btnFuncStateMachine);
      if (BTN_FUNC_READY == btnFuncStateMachine) {
        // Start function
        btnFuncStateMachine = BTN_FUNC_ACTIVE;
      } else if (BTN_FUNC_ACTIVE == btnFuncStateMachine) {
        // Pause function
        btnFuncStateMachine = BTN_FUNC_READY;
      } else {
        // undefine - skeep
      }
      break;
    case BTN_STATE_LONGPRESS:
    case BTN_STATE_LONGPRESS_10_SEC:
      DEBUG_PRINTLN("Button long press!");
      // Long press feedback
      ANALOGWRITE(OUTPUT_PIN, OUTPUT_MAX_VALUE/2);
      delay(100);
      ANALOGWRITE(OUTPUT_PIN, OUTPUT_MIN_VALUE);
      delay(100);
      break;
    case BTN_STATE_LONGPRESS_RELEASE:
      DEBUG_PRINTLN("Button long press released!");
      // record low state
      btnFuncVal = outValue;
    break;
    case BTN_STATE_LONGPRESS_10_SEC_RELEASE:
      DEBUG_PRINTLN("Button 10 long press released!");
      // TODO
    break;
    default:
      break;
  }
}

/**
 * Botton function: 
 * Long press  -> save current state as the target state (can't be zero)
 * Short press -> start slowly change the state to the terget state
 */
void btnFuncLoop(void) {
  static unsigned long lastIterTime = 0;

  if (millis() - lastIterTime < BTN_FUNC_INTERVAL_MS){
    return;
  }
    lastIterTime = millis();

  if (BTN_FUNC_ACTIVE == btnFuncStateMachine) {
    // perfotm botton single func step
    if (btnFuncVal < outValue) {
      DEBUG_PRINTLN("BTN --");
      outValue = safeAdd(outValue, -BTN_FUNC_ITER_SIZE, btnFuncVal, OUTPUT_MAX_VALUE);
      // outValue -= BTN_FUNC_ITER_SIZE;
    } else if (outValue < btnFuncVal) {
      DEBUG_PRINTLN("BTN ++");
      outValue = safeAdd(outValue, +BTN_FUNC_ITER_SIZE, OUTPUT_MIN_VALUE, btnFuncVal);
      // outValue += BTN_FUNC_ITER_SIZE;
    } else {
      DEBUG_PRINTLN("BTN done");
      // target achived -> exit botton function mode
      btnFuncStateMachine = BTN_FUNC_READY;
    }
  }
}

void btnSetup() {
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  // Read the initial state
  lastBtnPress        = millis();
  lastBtnRead         = digitalRead(ENC_BTN_PIN);
  lastBtnState        = digitalRead(ENC_BTN_PIN);

  btnFuncStateMachine = BTN_FUNC_READY;
  btnFuncVal          = 1;
}

void btnLoop() {
  static BtnState btnStateMachine = BTN_STATE_RELEASE;
  bool stateChanged = false;

  // execute the relevant mode
  btnFuncLoop();
  
  // Read the button state and avoid debouncing
  int btnGpioState = digitalRead(ENC_BTN_PIN);

  if (btnGpioState != lastBtnRead) {
    // Reset debounce counter
    lastBtnPress = millis();
    lastBtnRead = btnGpioState;
  }

  unsigned long diff = millis() - lastBtnPress;
  if (diff < BTN_DEBOUNCE_PERIOD_MS) {
    // Debounce - reed state may be unstable wait to see continuos signal 
    return;
  }

  if (BTN_GET_GPIO(btnStateMachine) != btnGpioState) {
    btnStateMachine = BTN_STATE_MACHINE_MAKE(btnGpioState, BTN_GET_DURATION_SEC(btnStateMachine));
    btnStateChange(btnStateMachine);
    btnStateMachine = BTN_STATE_MACHINE_MAKE(btnGpioState, 0);
  }
  else if (BTN_GPIO_PRESS == BTN_GET_GPIO(btnStateMachine) &&
           BTN_GPIO_PRESS == btnGpioState) {
    if (diff > 10 * BTN_1_SECOND_IN_MS && 10 > BTN_GET_DURATION_SEC(btnStateMachine)) {
      btnStateMachine = BTN_STATE_LONGPRESS_10_SEC;
      btnStateChange(btnStateMachine);
    } else if (diff > BTN_1_SECOND_IN_MS && 0 == BTN_GET_DURATION_SEC(btnStateMachine)) {
      btnStateMachine = BTN_STATE_LONGPRESS;
      btnStateChange(btnStateMachine);
    } 
  }

}

/***************************
 * Main
 ***************************/
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
#ifdef USE_16BIT_PWM
  setupPWM16();
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
