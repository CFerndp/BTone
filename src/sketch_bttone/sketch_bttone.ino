#include "BluetoothA2DPSource.h"
#include <math.h>

/*
 * PARAMETERS TO CHANGE: Note, Tone length and BT Speaker Name
*/

#define DEFAULT_FREQUENCY 440 // 440 -> A note

const uint64_t WAIT_TIMEOUT_INTERVAL_MS = 1 * 1000000;  // Meaning of first operator: secs
const uint64_t SOUND_TIMEOUT_INTERVAL_MS = 2 * 1000000; // Meaning of first operator: secs

const long connectionInterval = 1000; //Timeout (ms) to check if BT is connected
char BT_DEVICE[] = "Mi True Wireless EBs Basic 2";

/*
 * INTERNAL PARAMETERS
*/

#define TOUCH_GPIO 15
const int TOUCH_THRESHOLD = 10;

const uint16_t BOARD_FREC = 80;
BluetoothA2DPSource a2dp_source;

unsigned long previousMillisConnection = 0;
bool previousTouchValue = -1;
bool previousConnectedStatus = true;

int c3_frequency = DEFAULT_FREQUENCY;

//Wait Timer
volatile int waitInterruptCounter = 0;
hw_timer_t *waitTimer = NULL;
portMUX_TYPE waitTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int firstWaitTimer = 0; // When you activate a timer, the callbacks is executed just before activation. This is in order to fix it, skips first iteration.

//Sound Timer
volatile int soundInterruptCounter = 0;
hw_timer_t *soundTimer = NULL;
portMUX_TYPE soundTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int firstSoundTimer = 0; // When you activate a timer, the callbacks is executed just before activation. This is in order to fix it, skips first iteration.

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
}

void IRAM_ATTR onSoundTimer()
{
  portENTER_CRITICAL_ISR(&soundTimerMux);
  soundInterruptCounter++;
  portEXIT_CRITICAL_ISR(&soundTimerMux);
}

bool getTouchStatus()
{
  return touchRead(TOUCH_GPIO) < TOUCH_THRESHOLD;
}

void setup()
{
  Serial.begin(115200);
  a2dp_source.start(BT_DEVICE, get_data_channels);

  //Wait Timer config
  waitTimer = timerBegin(0, BOARD_FREC, true);                // Timer 0, if we divide by BOARD_FREC -> Timer will be increased 1M/s. Count up
  timerAttachInterrupt(waitTimer, &onWaitTimer, true);        //Attach callback
  timerAlarmWrite(waitTimer, WAIT_TIMEOUT_INTERVAL_MS, true); // third parameter means if it's periodical

  //Sound Timer config
  soundTimer = timerBegin(1, BOARD_FREC, true);                 // Timer 1, if we divide by BOARD_FREC -> Timer will be increased 1M/s. Count up
  timerAttachInterrupt(soundTimer, &onSoundTimer, true);        //Attach callback
  timerAlarmWrite(soundTimer, SOUND_TIMEOUT_INTERVAL_MS, true); // third parameter means if it's periodical

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

    if (firstWaitTimer > 0)
    {
      Serial.println("Enabling Sound Timer and sound");
      activate_sound();
      timerAlarmDisable(waitTimer);
      firstWaitTimer = 0;
      timerAlarmEnable(soundTimer);
    }
    else
    {
      firstWaitTimer++;
    }
  }

  if (soundInterruptCounter > 0)
  {

    portENTER_CRITICAL(&soundTimerMux);
    soundInterruptCounter--;
    portEXIT_CRITICAL(&soundTimerMux);

    if (firstSoundTimer > 0)
    {
      Serial.println("Deactivating sound");
      deactivate_sound();
      firstSoundTimer = 0;
      timerAlarmDisable(soundTimer);
    }
    else
    {
      firstSoundTimer++;
    }
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

  bool touchValue = getTouchStatus();

  if (previousTouchValue != touchValue)
  {
    if (touchValue)
    {
      if (a2dp_source.isConnected())
      {
        if (!timerAlarmEnabled(waitTimer) && !timerAlarmEnabled(soundTimer))
        {

          timerAlarmEnable(waitTimer);
          Serial.println("Wait on!");
        }
      }
      else
      {
        Serial.println("BT NOT CONNECTED");
      }
    }
  }

  previousTouchValue = touchValue;
}
