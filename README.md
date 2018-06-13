# RaspberryPi-NHD-1.8-TFT
C executable for the Raspberry Pi to output an area of the framebuffer to the NHD-1.8-128160EF TFT (ILI9163, 8 bit parallel)

This is a driver for a Newhaven 1.8" TFT display (http://www.newhavendisplay.com/tfts-18-tfts-c-1_589.html). It maps the framebuffer of the main display (/dev/fb0) to memory, extracts a 160x128 pixel sized area and sends this to the TFT display via the 8 bit parallel interface.

![TFT showing 160x128 area](/images/raspberry-desktop-160x128.jpg)

*A 160x128 area of the Raspberry Pi's desktop rendered on the TFT*

The driver init code sets up the 8 bit parallel interface to the color mode 18 bit RGB with 6/6/6 bits for the red/green/blue pixel values.

The pixel data in the framebuffer is 32 bits per pixel, 8/8/8/8 bits for the alpha/red/green/blue pixel values. This 32 bit data is split into 3x 8 bytes (only the topmost 6 bits of each byte are used internally by the driver IC) and sent to the TFT via the GPIOs with a frame rate of around 34 fps.

## Physical setup

**Part list:**

* Raspberry Pi 2 or 3
* NHD-1.8-128160EF TFT display with ILI9163 driver IC
* Breakout-Board (at least 24 pins / 0.8mm pitch or 0.5mm pitch for the -F variants of the TFT)


### Connecting the NHD TFT to the Raspberry Pi

Connect the **TFT Pins** from the breakout to the **Raspberry's GPIO** (I used simple jumper wires) like this:  
```
          TFT   ARTY  
    ---------   ---------   
       GND  1   GND  
     IOVDD  2   3.3v (typical 2.8V, max 3.3V)    
       VDD  3   3.3v      
       /CS  4   GND (= CS is always enabled)
      /RES  5   PIN 11 / GPIO17     
       D/C  6   PIN 07 / GPIO04   
       /WR  7   PIN 12 / GPIO18   
       /RD  8   3.3v (the read function is not used)   
       DB0  9   PIN 31 / GPIO06  
       DB1 10   PIN 32 / GPIO12   
       DB2 11   PIN 33 / GPIO13   
       DB3 12   PIN 35 / GPIO19   
       DB4 13   PIN 36 / GPIO16   
       DB5 14   PIN 37 / GPIO26   
       DB6 15   PIN 38 / GPIO20   
       DB7 16   PIN 40 / GPIO21 
     LED-A 17   3.3v  
    LED-K1 18   GND   
    LED-K2 19   GND  
       GND 20   GND  
     NC/YU 21   - 
     NC/XL 22   - 
     NC/YD 23   - 
     NC/XR 24   -    
```

## Compiling and running the executable

* Download the file **NHD_18_128160_Framebuffer2GPIO.c** from this repository to your Raspberry Pi's home folder
* Open a terminal on the Raspberry Pi and enter this line:  
`gcc -o NHD_18 NHD_18_128160_Framebuffer2GPIO.c -lrt -O3`  
This will generate the executable **NHD_18** in the same folder as the .c file
* To run the executable, enter  
`sudo ./NHD_18`  
into the terminal
* `CTRL-C` stops the executable

## Offsetting the TFT display area

The 160x128 pixel area that is output to the TFT has its origin by default at the upper left corner (0/0) of the display. If you want to move the area to some other location within the display's resolution, you can change the values of the #defines `X_START` and `Y_START` in the .c file, lines 24 and 25.


