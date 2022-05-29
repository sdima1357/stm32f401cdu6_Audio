# stm32f401ccAudioNative
stm32 green pill usb sound card
last one: 
https://www.youtube.com/watch?v=KB1A08Pj6nc


https://www.youtube.com/watch?v=0MmWp3HdV2A

https://www.youtube.com/watch?v=GbiTxVYopDI

https://www.youtube.com/watch?v=TnEBuS5ONsY




High quality, low noise  DAC based on 2 PWM timer channels with virtual software Sigma Delta ADC between stream from usb and PWM output.
There is implemented "sigma-delta floating point encoder" workaround of native stm32f401 limit 10.5 bits on 44100 Hz (1904 levels=84MHz/44.1KHz )
So, we can have for only  $3 ,very low noise , high sound quality solution, which better then most onboard sound cards !

There is implemented virtual software second order sigma delta adc for the shift quantization noise to high frequency.
see here (it [provide useful links too at the end of document):

https://www.analog.com/media/en/training-seminars/tutorials/MT-022.pdf

But instead one bit ( two levels ) i use more bits (0-MAX_LEVELS) Same technology can be used for esp32 high quality sound rendering.

14.04.2022 :

Added second order sigma-delta 

number of LCD from zero to 2

minor bugs fixed

optional external i2s module


21.04.2022 :
-- Change to open drain timer1 PWM outputs( PA8 PA9 ) config for better noise supression.

27.04.2022

-- pll bug fix

-- up pwm freq to 384KHz - significant sound quality improvment 

-- add tty output

30.04.2022

-- cheeper!!! green pill (stm32f401cdu6)version. Can buy here ( US $2.63 ) https://www.aliexpress.com/item/4001062944589.html

02.05.2022

-- detected problems with new STM32Cube_FW_F4_V1.27.0 audio driver. please build with <=STM32Cube_FW_F4_V1.26.0 . WIP


![image2](https://github.com/sdima1357/stm32f401ccAudioNative/blob/main/images/schematic1.png?raw=true)


![image1](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/P1010093.JPG?raw=true)

![image3](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220524_193907.jpg?raw=true)

![image4](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220524_193927_3.jpg?raw=true)

![image5](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220524_193935_3.jpg?raw=true)

![image6](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220524_193944_2.jpg?raw=true)

![image7](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220524_194003_3.jpg?raw=true)

![image8](https://github.com/sdima1357/stm32f401cdu6_Audio/blob/main/images/IMG_20220529_131254_3.jpg?raw=true)







