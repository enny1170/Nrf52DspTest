
/* lib to read spectra informations from Analog input and compute with FFT */

#define ARM_MATH_CM4


#include <nrf.h>
#include <Arduino.h>

#define ANALOG_PIN  A0
#define SAMPLE_RATE  9000 // HZ
#define FFT_COUNT       256
#define SAMPLES_COUNT   FFT_COUNT*2

typedef void (*AudioSampleCallback) (float Sample_Data[SAMPLES_COUNT],float MagnitudeData[FFT_COUNT]);

// to rceive the Callback in your Application, you have to set the Valiable below to an Method with fallowing Signature
// void AudioCallback(float Sample_Data[SAMPLES_COUNT],float MagnitudeData[FFT_COUNT])

AudioSampleCallback SampleCallback;
float Samples[SAMPLES_COUNT]={0};
float Magnitudes[FFT_COUNT]={0};
int SampleIdx=0;

// Start Timer for Sampling
void startTimer1()
{
    // enable Interrupt
    NVIC_EnableIRQ(TIMER1_IRQn);
    // start Timer
    NRF_TIMER1->TASKS_START=1;
}

// Stop Timer and disable Interrupts
void stopTimer1()
{
    //Disable Interrupt
    NVIC_DisableIRQ(TIMER1_IRQn);
    //stopping Tasks
    NRF_TIMER1->TASKS_STOP=1;
    while(NRF_TIMER1->TASKS_STOP!=0)
    {

    };
}

//define Timer1 Interrupt Handler. this must be marked for c-style linkage to work
#ifdef __cplusplus
extern "C" {
#endif

#include <arm_math.h>

// The Interrupt Handler needs fallowing signature:
void TIMER1_IRQHandler(void)
{
    // Check which timer Value is reached
    if(NRF_TIMER1->EVENTS_COMPARE[0] != 1)
    {
        // Reset the Compare Flag
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        // do your stuff here
        // Serial.println("Timer Interrupt reached-");
        // this compare is used to trigger reading of A0
        if(SampleIdx<SAMPLES_COUNT && SampleIdx+1<SAMPLES_COUNT)
        {
            Samples[SampleIdx]=analogRead(ANALOG_PIN);
            SampleIdx++;
            // Complex FFT functions require a coefficient for the imaginary part of the input.
            // Since we only have real data, set this coefficient to zero.
            Samples[SampleIdx] = 0.0;
            SampleIdx++;
        }
        else
        {
            stopTimer1();
            // Compute Samples with FFT here
            arm_cfft_radix4_instance_f32 fft_inst;
            arm_cfft_radix4_init_f32(&fft_inst, FFT_COUNT, 0, 1);
            arm_cfft_radix4_f32(&fft_inst, Samples);
            // Calculate magnitude of complex numbers output by the FFT.
            arm_cmplx_mag_f32(Samples, Magnitudes, FFT_COUNT);
            // call Callback Method to notify App about Finished Sampling and Calculation
            SampleCallback(Samples,Magnitudes);
            // restart Sampler 
            SampleIdx=0;
            startTimer1();
        }
    }
}

#ifdef __cplusplus
}
#endif

// Setup and start a timer that ticks all time_us microseconds
void initTimer1(uint32_t time_us)
{
    #define TIME_PRESCALER 4UL
    //Disable Interrupt
    NVIC_DisableIRQ(TIMER1_IRQn);
    //stopping Tasks
    NRF_TIMER1->TASKS_STOP=1;
    while(NRF_TIMER1->TASKS_STOP!=0)
    {

    };
    //reset Compare Flag
    NRF_TIMER1->EVENTS_COMPARE[0]=0;   

    NRF_TIMER1->MODE=TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER1->BITMODE= TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos; //24-bit Mode
    NRF_TIMER1->PRESCALER=TIME_PRESCALER << TIMER_PRESCALER_PRESCALER_Pos; // 1us resolution
    NRF_TIMER1->TASKS_CLEAR=1;

    //setup ShortCuts for single run
    //NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_STOP_Enabled << TIMER_SHORTS_COMPARE1_STOP_Pos |
    //                      TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos );
    //time ((crystal-freq / (2 ^ Prescaler))*seconds

    NRF_TIMER1->CC[0]=  time_us; //value in microseconds 
    //if you need a interrupt for a second time you can add a additional time here
    //NRF_TIMER1->CC[1]=  time_us*7;

    //enable interrupt for CC[0]
    NRF_TIMER1->INTENSET=(TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    // Setup Shortcust for self running
    NRF_TIMER1->SHORTS=(TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    // Start Timer
    // startTimer1();
}

// init a Sampler by SEMPLE_RATE define
void initSampler()
{
    pinMode(ANALOG_PIN,INPUT);
    initTimer1((uint32_t)1000000/SAMPLE_RATE);
}

// init a Sampler by Parameter
void initSampler(uint32_t SampleRate)
{
    pinMode(ANALOG_PIN,INPUT);
    initTimer1((uint32_t)1000000/SampleRate);
}

// start a Sampler
void startSampler()
{
    startTimer1();
}

// stop a Sampler
void stopSampler()
{
    stopTimer1();
}
