# STM32F334R8_digital_filtration
Digital filtration of analog signal from onboard adc.

The project implements digital filtering (FIR) of an analog signal (from ADC). The filtered signal is passed to the DAC.

The filter coefficients correspond to a 300-order filter and perform band-stop filtering of a 50hz signal.

Coefficients were determined using Matlab tools.
