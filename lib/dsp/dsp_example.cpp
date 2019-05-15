// example how to use the dsp lib on Nrf52
#include <Arduino.h>
#include <dsp.h>


// callback for AudioSampler
void AudioFftCallback(float Sample_Data[SAMPLES_COUNT],float MagnitudeData[FFT_COUNT])
{
    Serial.print("FFT:");
    for (size_t i = 0; i < FFT_COUNT; i++)
    {
        Serial.print(Sample_Data[i]);
        Serial.print(",");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    initSampler();
}