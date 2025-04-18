//https://github.com/br3ttb/Arduino-PID-Library
//install using library manager - search for PID
#include <PID_v1.h>

//https://github.com/pfeerick/elapsedMillis/wiki
//install using library manager - search for elapsedMillis
#include <elapsedMillis.h>

#include "tusb.h"

const bool analog_driver = true;
#define VERSION 2


const float mega = 1000000.f;
const float micro = 0.000001f;
#define NUM_CMD_DATA 4
#define CHAR_BUF_SIZE 128



typedef struct {
  char cmd;
  float data[NUM_CMD_DATA];
} CommandT;

const double min_setpoint = 0;
const double max_setpoint = 1023;

const int analog_zero_out = 512;

double setpoint = 512;
double measurement = 0;
double output = 0;

double pwm_freq = 100;

int num_meas_to_avg = 1000;
double meas_accum = 0;
int num_meas = 0;
int num_cycles_since_compute = 0;

//must specify values
const int input_pin = 26; //ADC0
const int output_pin_a = 16;
const int output_pin_b = 17;
const int analog_out_pin = 16;

const int num_debug_pins = 4;
const int debug_pin[num_debug_pins] = {-1,-1,-1,-1}; //fill in with actual pin numbers to use

int report_interval = 0;

//create a pid object that connects to setpoint, measurement, output; default feedback is direct and all Ks set to 0 initially
PID myPID(&measurement, &output, &setpoint, 0, 0, 0, DIRECT);

elapsedMillis time_since_report;

void setup() {
  // put your setup code here, to run once:
  if (analog_driver) {
    pwm_freq = 200000;
  }

  myPID.SetMode(0);
  myPID.SetOutputLimits(-1023, 1023);
  myPID.SetSampleTime(10);
  pinMode(input_pin, INPUT);
  pinMode(output_pin_a, OUTPUT);
  pinMode(output_pin_b, OUTPUT);
  analogWriteResolution(10); //10 bits 0 to 1023
  analogReadResolution(10); //10 bits 0 to 1023
  analogWriteFreq(pwm_freq);
  for (int j = 0; j < num_debug_pins; ++j){
    if (debug_pin[j] > 0) {
      pinMode(debug_pin[j], OUTPUT);
    }
  }
}

inline void debug_out(int j, bool value){
  if (debug_pin[j] > 0) {
      digitalWrite(debug_pin[j], value);
    } 
}

void loop() {
  for (int j = 0; j < num_debug_pins; ++j) {
    debug_out(j,0);
  }
  debug_out(0,HIGH);
  pollSerial();
  debug_out(1,HIGH);
  measure_loop();
  debug_out(2,HIGH);
  if (myPID.Compute()) {
    num_meas_to_avg = (int) (num_cycles_since_compute * 0.9 + 1); 
    num_cycles_since_compute = 0;
  } else {
    ++num_cycles_since_compute;
  }
  debug_out(3,HIGH);
  setOutput(output);

}

//sets output value from -1023 (full reverse) to 1023 (full forward)
void setOutput(double value) {
  if (analog_driver) {
    //input -1023 -> 1023, input 1023 -> 0 (typical); input 0 -> analog_output_zero
      int outvalue = (int) (-value/2.0 + analog_zero_out + 0.5);
      outvalue = outvalue > 0 ? outvalue : 0;
      outvalue = outvalue < 1023 ? outvalue : 1023;
      analogWrite(analog_out_pin, outvalue);
      return;
  }
  if (value > 0) {
    digitalWrite(output_pin_b, LOW);
    analogWrite(output_pin_a, (int) (value + 0.5));
  } else if (value < 0) {
    digitalWrite(output_pin_a, LOW);
    analogWrite(output_pin_b, (int) (-value - 0.5));
  } else {
    digitalWrite(output_pin_a, LOW);
    digitalWrite(output_pin_b, LOW);
  }

}

//average multiple measurements to reduce noise
//number of measurements to average is controlled by num_meas_to_avg
void measure_loop() {
  if (num_meas >= num_meas_to_avg) {
    measurement = meas_accum / num_meas;
    num_meas = 0;
    meas_accum = 0;
  }
  meas_accum += analogRead(input_pin);
  num_meas++;
}
int readLineSerial(char buff[], int buffersize, unsigned int timeout) {
  int i = 0;
  if (!Serial || !tud_cdc_connected()) {
    return 0;
  }
  if (!Serial.available()) {
    return 0;
  }
  elapsedMicros t0;
  while (i < buffersize - 1 && (Serial.available() || t0 < timeout)) {
    if (!Serial.available()) {
      continue;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (i > 0) { //discard newline characters at beginning of string (in case of \n\r)
        buff[i + 1] = '\0';
        return i; //positive return value
      }
    } else {
      buff[i++] = c;
    }
  }
  return -i; //0 if nothing read, negative value if read was incomplete
}
void pollSerial() {
  if (!Serial) {
    return;
  }
  processSerialLine();
   if (report_interval > 0 && time_since_report > report_interval){
    time_since_report -= report_interval;
    sendReport();
  }
}

void processSerialLine() {
  char buff[CHAR_BUF_SIZE];
  int rv;
  rv = readLineSerial(buff, CHAR_BUF_SIZE, 500);  //changed from 500 to 500k for debugging
  // if (rv < 0) {
  //   sendMessage("line reading failed", 1);
  //   return;
  // }
  if (rv == 0) {
    return;
  }
  //  restarted = false; //received a command
  int wsoff = 0;
  for (wsoff = 0; isspace(buff[wsoff]) && wsoff < rv; ++wsoff)
    ;  //get rid of leading whitespace
  CommandT c;
  c.cmd = 'X';
  c.data[0] = c.data[1] = c.data[2] = c.data[3] = 0;
  sscanf(buff + wsoff, "%c %f %f %f %f", &c.cmd, c.data, c.data + 1, c.data + 2, c.data + 3);  //change if num_data_bytes changes
  parseCommand(c);
}

void parseCommand(CommandT c) {
  digitalWrite(LED_BUILTIN, LOW);
  switch (toupper(c.cmd)) {
    case 'P':  //set proportional
      myPID.SetTunings(c.data[0], myPID.GetKi(), myPID.GetKd());
      return;
    case 'I':  //set integral
      myPID.SetTunings(myPID.GetKp(), c.data[0], myPID.GetKd());
      return;
    case 'D':  //set derivative
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), c.data[0]);
      return;
    case 'E':  //enable or disable
      myPID.SetMode(c.data[0]);
       return;
    case 'F':  //set feedback direction  >= 0 -- direct, <0 is reverse
      myPID.SetControllerDirection((int)(c.data[0] < 0));
      return;
    case 'S':  //set setpoint
      setpoint = c.data[0];
      setpoint = setpoint < min_setpoint ? min_setpoint : setpoint;
      setpoint = setpoint > max_setpoint ? max_setpoint : setpoint;
      return;
    case 'O':  //set output
      output = c.data[0];
      return;
    case 'R':  //get report or report every xx seconds
      sendReport();
      report_interval = c.data[0];
      time_since_report = 0;
      return;
    case 'Y': //set pwm frequency
      pwm_freq = c.data[0];
      analogWriteFreq(pwm_freq);
    case 'C': // set computation period in ms
      if (c.data[0] > 0) {
        myPID.SetSampleTime(c.data[0]);
      }
    default:
      digitalWrite(LED_BUILTIN, HIGH);
  }
}

void sendReport(){
    char buff[1024];
    double mult = myPID.GetDirection() > 0 ? -1 : 1;
    sprintf(buff, "MEAS: %.1f\tOUT: %.1f\tSET: %.1f\tKp: %.1f\tKi: %.1f\tKd: %.1f\tVER: %d", measurement, output, setpoint, mult*myPID.GetKp(), mult*myPID.GetKi(), mult*myPID.GetKd(), VERSION);
    Serial.println(buff);
}
