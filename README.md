bell-temple
==================

This Arduino project analyizes a sound input from a microphone, converts it to frequency bins using FHT, and visualizes it using a strip of LED NeoPixels.


## Key Hardware

* Arduino Uno
* Electret microphone
* [NeoPixel Strip](http://www.adafruit.com/products/1376)


## A Slightly More Detailed Description

A digital signal is captured from a electet microphone connnected to an Arudino and then processed using a Fast Hartley Transform. Because the frequency range of the the bells we're targeting are relatively low, we're using a low pass filter to filter out frequencies higher than 1,000 Hz. The frequencies of the bells roughly range from 200 Hz to about 1000 Hz. The FHT produces 256 bins which contain strength data on a small silver of the audio signal. When using a sample rate of 5000, each bin represents a range of about 19.5 Hz. So, the first 50 bins are the most relevent for this particular project. Each loop, we determine which bins have been a signal and assign it to a section of the LED strip.
