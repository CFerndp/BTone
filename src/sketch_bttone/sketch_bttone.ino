#include "BluetoothA2DPSource.h"
#include <math.h> 

/*
 * PARAMETERS TO CHANGE: Note, Tone length and BT Speaker Name
*/

#define DEFAULT_FREQUENCY 440 // 440 -> A note
#define PIN_BUTTON 15
const long toneInterval = 1000; // Timeout (ms). One second of note, one second of silence
const long connectionInterval = 1000; //Timeout (ms) to check if BT is connected
char BT_DEVICE[] = "Redmi AirDots_R";

/*
 * INTERNAL PARAMETERS
*/

BluetoothA2DPSource a2dp_source;

unsigned long previousMillisConnection = 0;
unsigned long previousMillisTone = 0;
int previousValButton = -1;

int c3_frequency = DEFAULT_FREQUENCY;

// Note callback
int32_t get_data_channels(Channels *channels, int32_t channel_len) {
    static double m_time = 0.0;
    double m_amplitude = 10000.0;  // -32,768 to 32,767
    double m_deltaTime = 1.0 / 44100.0;
    double m_phase = 0.0;
    double double_Pi = PI * 2.0;

    // fill the channel data
    for (int sample = 0; sample < channel_len; ++sample) {
         double angle = double_Pi * c3_frequency * m_time + m_phase;
         channels[sample].channel1 = m_amplitude * sin(angle);
         channels[sample].channel2 = channels[sample].channel1;
         m_time += m_deltaTime;
    } 
    
    return channel_len;
}

void activate_sound() {
  c3_frequency = DEFAULT_FREQUENCY;
  Serial.println("Sound Activated");
}

void deactivate_sound() {
  c3_frequency = 0;
  Serial.println("Sound Deactivated");
}

void toggle_sound() {
  if(c3_frequency == 0) {
      activate_sound();
    } else {
      deactivate_sound();
    }
}

void setup() {
  Serial.begin(115200); 
  pinMode(PIN_BUTTON, INPUT);
  a2dp_source.start(BT_DEVICE, get_data_channels);  
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisConnection >= connectionInterval) {
    previousMillisConnection = currentMillis;

    if(a2dp_source.isConnected()){
      Serial.print("Connected to ");
      Serial.println(BT_DEVICE);
    } else {
      Serial.println("Not Connected");
    }    
  }

  if(a2dp_source.isConnected()){
    int pinValue = digitalRead(PIN_BUTTON);

    if(previousValButton != pinValue) {
      if(val == HIGH) {
        activate_sound();
      } else {
        deactivate_sound();
      }
      
      previousValButton = pinValue;
    }
   }
}
