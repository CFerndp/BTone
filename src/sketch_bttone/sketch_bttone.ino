#include "BluetoothA2DPSource.h"
#include <math.h>

/*
 * PARAMETERS TO CHANGE: Note, Tone length and BT Speaker Name
*/

#define DEFAULT_FREQUENCY 440 // 440 -> A note
#define PIN_BUTTON 15

#define WAIT_TIMEOUT_INTERVAL_MS 5 * 1000000  // Meaning of first operator: secs
#define SOUND_TIMEOUT_INTERVAL_MS 2 * 1000000 // Meaning of first operator: secs

const long connectionInterval = 1000; //Timeout (ms) to check if BT is connected
char BT_DEVICE[] = "Redmi AirDots_R";

/*
 * INTERNAL PARAMETERS
*/

#define BOARD_FREC 240
BluetoothA2DPSource a2dp_source;

unsigned long previousMillisConnection = 0;
int previousValButton = -1;
bool previousConnectedStatus = true;

int c3_frequency = DEFAULT_FREQUENCY;

//Wait Timer
volatile int waitInterruptCounter = 0;
hw_timer_t *waitTimer = NULL;
portMUX_TYPE waitTimerMux = portMUX_INITIALIZER_UNLOCKED;

//Sound Timer
volatile int soundInterruptCounter = 0;
hw_timer_t *soundTimer = NULL;
portMUX_TYPE soundTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Note callback
int32_t get_data_channels(Channels *channels, int32_t channel_len)
{
  static double m_time = 0.0;
  double m_amplitude = 10000.0; // -32,768 to 32,767
  double m_deltaTime = 1.0 / 44100.0;
  double m_phase = 0.0;
  double double_Pi = PI * 2.0;

  // fill the channel data
  for (int sample = 0; sample < channel_len; ++sample)
  {
    double angle = double_Pi * c3_frequency * m_time + m_phase;
    channels[sample].channel1 = m_amplitude * sin(angle);
    channels[sample].channel2 = channels[sample].channel1;
    m_time += m_deltaTime;
  }

  return channel_len;
}

void activate_sound()
{
  c3_frequency = DEFAULT_FREQUENCY;
  Serial.println("Sound Activated");
}

void deactivate_sound()
{
  c3_frequency = 0;
  Serial.println("Sound Deactivated");
}

void IRAM_ATTR onWaitTimer()
{
  portENTER_CRITICAL_ISR(&waitTimerMux);
  waitInterruptCounter++;
  portEXIT_CRITICAL_ISR(&waitTimerMux);
  Serial.println(millis());
}

void IRAM_ATTR onSoundTimer()
{
  portENTER_CRITICAL_ISR(&soundTimerMux);
  soundInterruptCounter++;
  portEXIT_CRITICAL_ISR(&soundTimerMux);

  Serial.println(millis());
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);
  a2dp_source.start(BT_DEVICE, get_data_channels);

  //Wait Timer config
  waitTimer = timerBegin(0, BOARD_FREC, false);                // Timer 0, if we divide by BOARD_FREC -> Timer will be increased 1M/s. Count up
  timerAttachInterrupt(waitTimer, &onWaitTimer, true);         //Attach callback
  timerAlarmWrite(waitTimer, WAIT_TIMEOUT_INTERVAL_MS, false); // third parameter means if it's periodical

  //Sound Timer config
  soundTimer = timerBegin(2, BOARD_FREC, true);                  // Timer 1, if we divide by BOARD_FREC -> Timer will be increased 1M/s. Count up
  timerAttachInterrupt(soundTimer, &onSoundTimer, true);         //Attach callback
  timerAlarmWrite(soundTimer, SOUND_TIMEOUT_INTERVAL_MS, false); // third parameter means if it's periodical

  deactivate_sound();
}

void loop()
{

  //Timer use. Handle by Mutex
  if (waitInterruptCounter > 0)
  {

    portENTER_CRITICAL(&waitTimerMux);
    waitInterruptCounter--;
    portEXIT_CRITICAL(&waitTimerMux);

    Serial.println("Enabling Sound Timer and sound");
    activate_sound();
    timerAlarmEnable(soundTimer);
  }

  if (soundInterruptCounter > 0)
  {

    portENTER_CRITICAL(&soundTimerMux);
    soundInterruptCounter--;
    portEXIT_CRITICAL(&soundTimerMux);

    Serial.println("Deactivating sound");
    deactivate_sound();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisConnection >= connectionInterval)
  {
    previousMillisConnection = currentMillis;
    bool isBTConnected = a2dp_source.isConnected();

    if (previousConnectedStatus != isBTConnected)
    {
      if (isBTConnected)
      {
        Serial.print("Connected to ");
        Serial.println(BT_DEVICE);
      }
      else
      {
        Serial.println("Not Connected");
      }
    }

    previousConnectedStatus = isBTConnected;
  }

  int pinValue = digitalRead(PIN_BUTTON);

  if (previousValButton != pinValue)
  {
    if (pinValue == HIGH)
    {
      if (a2dp_source.isConnected())
      {
        Serial.println(millis());
        timerAlarmEnable(waitTimer);
        Serial.println("Wait on!");
      }
      else
      {
        Serial.println("BT NOT CONNECTED");
      }
    }
  }

  previousValButton = pinValue;
}
