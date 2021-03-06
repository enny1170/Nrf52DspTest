#include <Arduino.h>
#include <nrf.h>
#include <dsp.h>

// callback for Sampler
void FftCallback(float Sample_Data[SAMPLES_COUNT],float MagnitudeData[FFT_COUNT])
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
    Serial.println("Init");
    pinMode(A0,INPUT);
    pinMode(PIN_LED1,OUTPUT);
    digitalWrite(PIN_LED1,HIGH);
    delay(2000);
    digitalWrite(PIN_LED1,LOW);
    Serial.println("Init Sampler");
    initSampler();
    startSampler();
}

void loop()
{
    //uint32_t test=analogRead(A0);
}