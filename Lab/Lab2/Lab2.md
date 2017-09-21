# Lab 2- Analog Circuitry and FFTs

## Premise
The goal of this lab is to use signal processing for treasure-detecting sensors and a microphone sensor. We used the Open Music Lab FFT library for Arduino to implement a digital filter. One sub-team, composed of Shanee, Daniel, and David, attached a microphone to distinguish a 660 Hz audible start signal; the other sub-team, composed of Aasta, Erika, and Ben, attached an IR sensor to detect treasures emitting IR light at 7 kHz.

## Required Materials
Acoustic Team:

- Arduino Uno
- Electret microphone
- 1 µF capacitor
- 300 Ω resistors
- ~3 kΩ resistor

Optical Team:

- Arduino Uno [IDE](https://www.arduino.cc/en/Main/Software)
- IR receiver
- 300 Ω resistors
- Treasure board 
- Analog filter circuit
- Breadboard

## Procedure

## Acoustic Team
First and foremost, we installed the Open Music Labs FFT library [here](http://wiki.openmusiclabs.com/wiki/ArduinoFFT).

Before implementing the microphone circuit, we looked over the Open Music Labs Arduino FFT library documentation and the ADC on the Arduino microcontroller (ATmega328). The [datasheet](http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf) (beginning section 28, page 305) indicates that:

-The ADC has 10-bit resolution

-At a resolution of 10 bits, an ADC conversion takes 13 clock cycles.

-The ADC clock is preset at a speed of 125 kHz. Therefore, the preset ADC sampling frequency is 125 kHz / 13 = 9.6 kHz.

-The ADC sampling frequency of 9.6 kHz is using the default prescaler of 128, which can be set using the last 3 bits of the ADCSRA register. Max sampling frequency is 615 kHz at a prescaler of 2.

-For maximum resolution, the Arduino takes an input clock frequency of 50 to 200 kHz.

-ADC initialization takes 15 clock cycles.

-The output is stored in ADCL and ADCH and is 1024*(analog input voltage)/5V.

-The ADC can be set to Free Running mode to continuously update the ADCL and ADCH values. 

The analogRead function has a sampling frequency of 10 kHz, which is sufficient for detecting an audio signal of 660 Hz, but faces limitations in terms of available analog pins. As a result we decided to use ADC directly.

### Using fft_adc_serial
