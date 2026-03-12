#include "globals.h"

static constexpr uint32_t TRAJ_SAMPLES = 5000u;
static constexpr float SAMPLE_RATE_HZ = 1000.0f;
static constexpr float AMP_DEG = 45.0f;
bool playing = false;
static void buildCircularSineTrajectory()
{
    trajectory.reset();

    const float dt = 1.0f / SAMPLE_RATE_HZ;
    const float twoPi = 6.28318530718f;

    for (uint32_t n = 0; n < TRAJ_SAMPLES; ++n)
    {
        const float t = n * dt;
        float row[NUM_AXIS];

        for (uint32_t axis = 0; axis < NUM_AXIS; ++axis)
        {
            const float f = static_cast<float>(axis + 1u); // 1..6 Hz
            row[axis] = AMP_DEG * sinf(twoPi * f * t);
        }

        if (!trajectory.pushRow(row))
        {   while(1)
    
            {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(500);
                digitalWrite(LED_BUILTIN, LOW);
                delay(500);
                Serial.println("ERR:TRAJECTORY_BUFFER_FULL");
            }
        }
    }

}

void setup()
{   fro.setFeedrate(0.0f);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setTimeout(0);
    while (!Serial)
        yield();

    buildCircularSineTrajectory();
    delay(1000);
    // Serial.println("System Initialized.");
    // Serial.printf("OK:TRAJECTORY_BUILT samples=%lu\n", static_cast<unsigned long>(trajectory.length()));
    // delay(1000);
    myTimer.begin(timerISR, 1000); // 1kHz
}


void play()
{
    fro.setFeedrate(1.0f);
    playing = true;
}

void pause()
{
    fro.setFeedrate(0.0f);
    playing = false;
}



void serialEvent()
{
    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());
        if (c == 'p')
        {
            if (playing)
                pause();
            else
                play();
        }
        // Ignore all other incoming chars.
    }
}

void loop()
{
    serialEvent();
}