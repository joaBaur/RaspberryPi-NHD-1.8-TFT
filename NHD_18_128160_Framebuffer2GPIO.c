/*******************************************************************************
 * NHD_24_240320_Framebuffer2GPIO.c
 * for NHD-1.8-128160EF TFTs with driver IC ILI9163 using the 8 bit parallel interface
 * 
 * copies the contents of the display framebuffer into a memory region
 * and outputs a 160x128 area to the TFT display via bit banging GPIOs
 *
 * compile with:   gcc -o NHD_18 NHD_18_128160_Framebuffer2GPIO.c -lrt -O3
 * start with:     sudo ./NHD_18
 *
 * framebuffer code adapted from
 * https://gist.github.com/Darfk/5790622
 *
 * framebuffer info
 * http://www.ummon.eu/Linux/API/Devices/framebuffer.html
 *
 * GPIO code adapted from
 * https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access
 *
 *******************************************************************************/

#define PRINT_INFO          1 /* change to 0 to surpress all printf output */

#define X_START             0 /* horizontal start position from where the 320x240 area is read */
#define Y_START             0 /* vertical start position from where the 320x240 area is read */

#define BCM2708_PERI_BASE   0x3F000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)  /* GPIO controller */

// define interface connections to GPIOs
// /CS is tied to GND (= enabled)
// /RD is tied to 3V3 (= disabled)
#define NHD_DC   4 /*  Pin 07 -> GPIO 04  */
#define NHD_WR  18 /*  Pin 12 -> GPIO 18  */
#define NHD_RES 17 /*  Pin 11 -> GPIO 17  */

#define NHD_DB0 6 /*  Pin 31 -> GPIO 06  */
#define NHD_DB1 12 /*  Pin 32 -> GPIO 12  */
#define NHD_DB2 13 /*  Pin 33 -> GPIO 13  */
#define NHD_DB3 19 /*  Pin 35 -> GPIO 19  */
#define NHD_DB4 16 /*  Pin 36 -> GPIO 16  */
#define NHD_DB5 26 /*  Pin 37 -> GPIO 26  */
#define NHD_DB6 20 /*  Pin 38 -> GPIO 20  */
#define NHD_DB7 21 /*  Pin 40 -> GPIO 21  */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <string.h>
#include <stdint.h>

#include <time.h>
#include <sys/time.h>

#define BLOCK_SIZE (4*1024)
 
int  mem_fd;
void *gpio_map;

// delay nanosecondsNoSleep
long int start_time;
long int time_difference;
struct timespec gettime_now;
 
// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

void Setup_GPIO();
void TFT_18_9163_Write_8Bit(int8_t);
void TFT_18_9163_Write_Command(int8_t);
void TFT_18_9163_Write_Data(int8_t);
void TFT_18_9163_Init();
void DelayNanosecondsNoSleep (int);

int main (void) {
    
    if (PRINT_INFO == 1) printf("Starting NHD_24_240320_Framebuffer2GPIO\n");

    /*************************
     *
     * Framebuffer init
     *
     *************************/

    /* framebuffer file descriptor */
    int fbfd;

    /* structures for framebuffer information */
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    
    /* screen size in bytes */
    /* x * y * bpp / 8 */
    unsigned long int screensize;

    /* row length in pixels */
    unsigned int row_length;

    /* Framebuffer pointer */
    int *fbp;

    /* Open framebuffer */
    fbfd = open("/dev/fb0", O_RDWR);
    if(fbfd == -1) {
        printf("Error: cannot open framebuffer device");
        exit(1);
    }

    /* Get fixed screen information */
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        printf("Error reading fixed information");
        exit(2);
    }

    /* Get variable screen information */
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        printf("Error reading variable information");
        exit(3);
    }

    if (PRINT_INFO == 1) {
        printf("%s\n", finfo.id);
        printf("linelength %d\n", finfo.line_length);
        printf("%d x %d, %d bpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
        printf("ALPHA offset %d length %d\n", vinfo.transp.offset, vinfo.transp.length);
        printf("RED   offset %d length %d\n", vinfo.red.offset,    vinfo.red.length);
        printf("GREEN offset %d length %d\n", vinfo.green.offset,  vinfo.green.length);
        printf("BLUE  offset %d length %d\n", vinfo.blue.offset,   vinfo.blue.length);
    }
    
    row_length = vinfo.xres;
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8 ;
    fbp = mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);

    if (*(fbp) == -1) {
        printf("Error: failed to map framebuffer device to memory\n");
        exit(4);
    }

    /*************************
     *
     * GPIO init
     *
     *************************/

    Setup_GPIO();

    /*************************
     *
     * TFT driver init
     *
     *************************/

    TFT_18_9163_Init();

    /*************************
     *
     * Pixel processing
     *
     *************************/

    int row, col, color32, red, green, blue;

    // fps measurement
    struct timeval t1, t2;
    long long elapsedTime;

    while(1) {

        // Start frame memory write
        // 9.1.22 RAMWR (2Ch): Memory Write page 195
        TFT_18_9163_Write_Command(0x2C);

        // Start fps measurement
        gettimeofday(&t1, NULL);

        // switch columns and rows (TFT expects data in portrait orientation)
        for (col=0; col<160; col++) {
            for (row=0; row<128; row++) {

                // read 32 bit pixel data from framebuffer memory map
                color32 = fbp[(row + Y_START)*row_length + (col + X_START)];

                // color32 (RGB 8/8/8 pixel data from framebuffer, bits C24....C31 = alpha, not shown)
                // C23 C22 C21 C20 C19 C18 C17 C16  C15 C14 C13 C12 C11 C10 C09 C08  C07 C06 C05 C04 C03 C02 C01 C00
                // R07 R06 R05 R04 R03 R02 R01 R00  G07 G06 G05 G04 G03 G02 G01 G00  B07 B06 B05 B04 B03 B02 B01 B00

                red   = (color32 >> 16) & 0xFF; // shift down 16 bits, then mask 1 byte
                TFT_18_9163_Write_Data(red);

                green = (color32 >>  8) & 0xFF; // shift down  8 bits, then mask 1 byte
                TFT_18_9163_Write_Data(green);

                blue  = (color32 & 0xFF);       // imask 1 byte
                TFT_18_9163_Write_Data(blue);
            }
        }

        // display on command (stop memory write at end of frame)
        TFT_18_9163_Write_Command(0x29);

        // fps measurement
        gettimeofday(&t2, NULL);
        elapsedTime = ((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec);
        if (PRINT_INFO == 1) printf("FPS: %lld , %lld ms\n", 1000/(elapsedTime/1000), elapsedTime/1000);

    }

    /* Unmap the memory and release all the files */
    munmap(fbp, screensize);
    close(fbfd);

    return 0;

}

//
// Set up a memory regions to access GPIO
//
void Setup_GPIO() {

    // Set up gpio pointer for direct register access

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
       printf("Error: Can't open /dev/mem \n");
       exit(-1);
    }
    
    /* mmap GPIO */
    gpio_map = mmap(
       NULL,                    // Any adddress in our space will do
       BLOCK_SIZE,              // Map length
       PROT_READ|PROT_WRITE,    // Enable reading & writting to mapped memory
       MAP_SHARED,              // Shared with other processes
       mem_fd,                  // File to map
       GPIO_BASE                // Offset to GPIO peripheral
    );
    
    close(mem_fd);              //No need to keep mem_fd open after mmap
    
    if (gpio_map == MAP_FAILED) {
       printf("Error: mmap failed %d\n", (int)gpio_map);
       exit(-1);
    }
    
    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;

    // Set up the pins as output
    // Always use INP_GPIO(x) before using OUT_GPIO(x)

    INP_GPIO(NHD_DC);
    OUT_GPIO(NHD_DC);
    INP_GPIO(NHD_WR);
    OUT_GPIO(NHD_WR);
    INP_GPIO(NHD_RES);
    OUT_GPIO(NHD_RES);

    INP_GPIO(NHD_DB0);
    OUT_GPIO(NHD_DB0);
    INP_GPIO(NHD_DB1);
    OUT_GPIO(NHD_DB1);
    INP_GPIO(NHD_DB2);
    OUT_GPIO(NHD_DB2);
    INP_GPIO(NHD_DB3);
    OUT_GPIO(NHD_DB3);
    INP_GPIO(NHD_DB4);
    OUT_GPIO(NHD_DB4);
    INP_GPIO(NHD_DB5);
    OUT_GPIO(NHD_DB5);
    INP_GPIO(NHD_DB6);
    OUT_GPIO(NHD_DB6);
    INP_GPIO(NHD_DB7);
    OUT_GPIO(NHD_DB7);
}

//
// Writes a 16 bit value (command or data) to the TFTs driver IC
//
void TFT_18_9163_Write_8Bit(int8_t command) {

    if (command & 0x01 ) {
        GPIO_SET = 1 << NHD_DB0;
    } else {
        GPIO_CLR = 1 << NHD_DB0;
    }
    if (command & 0x02 ) {
        GPIO_SET = 1 << NHD_DB1;
    } else {
        GPIO_CLR = 1 << NHD_DB1;
    }
    if (command & 0x04 ) {
        GPIO_SET = 1 << NHD_DB2;
    } else {
        GPIO_CLR = 1 << NHD_DB2;
    }
    if (command & 0x08 ) {
        GPIO_SET = 1 << NHD_DB3;
    } else {
        GPIO_CLR = 1 << NHD_DB3;
    }
    if (command & 0x10 ) {
        GPIO_SET = 1 << NHD_DB4;
    } else {
        GPIO_CLR = 1 << NHD_DB4;
    }
    if (command & 0x20 ) {
        GPIO_SET = 1 << NHD_DB5;
    } else {
        GPIO_CLR = 1 << NHD_DB5;
    }
    if (command & 0x40 ) {
        GPIO_SET = 1 << NHD_DB6;
    } else {
        GPIO_CLR = 1 << NHD_DB6;
    }
    if (command & 0x80 ) {
        GPIO_SET = 1 << NHD_DB7;
    } else {
        GPIO_CLR = 1 << NHD_DB7;
    }

}

//
// Sends a 8 or 16 bit command to the TFTs driver IC
//
void TFT_18_9163_Write_Command(int8_t command) {

    // D/C = Low -> Sending command
    GPIO_CLR = 1 << NHD_DC;

    TFT_18_9163_Write_8Bit(command);

    GPIO_CLR = 1 << NHD_WR; // create rising edge for WR

    DelayNanosecondsNoSleep(10);
    GPIO_SET = 1 << NHD_WR;

}

//
// Sends a 8 or 16 bit data value to the TFTs driver IC
//
void TFT_18_9163_Write_Data(int8_t data) {

    // D/C = High -> Sending data
    GPIO_SET = 1 << NHD_DC;

    TFT_18_9163_Write_8Bit(data);

    GPIO_CLR = 1 << NHD_WR; // create rising edge for WR

    DelayNanosecondsNoSleep(10);
    GPIO_SET = 1 << NHD_WR;

}

//
// Initialize the TFT's driver IC after power up or reset
//
void TFT_18_9163_Init() {

    // init control wires (GPIO_SET -> HIGH, GPIO_CLR -> LOW)
    GPIO_SET = 1 << NHD_DC;   // Data/Command wire
    GPIO_SET = 1 << NHD_WR;   // Write wire

    GPIO_SET = 1 << NHD_RES;  // Reset signal (ACTIVE = LOW) disabled
    DelayNanosecondsNoSleep(120*1000); // delay 120ms after powerup before reset

    GPIO_CLR = 1 << NHD_RES; // Reset is active
    DelayNanosecondsNoSleep(30); // hold reset active for 30 ns

    GPIO_SET = 1 << NHD_RES; // Reset is done
    DelayNanosecondsNoSleep(120*1000); // Delay 120ms after reset

    // exit sleep mode
    TFT_18_9163_Write_Command(0x11);
    DelayNanosecondsNoSleep(100*1000); // Delay 100ms after exit sleep

    // display OFF
    TFT_18_9163_Write_Command(0x28);

    // MADCTL: memory data access control
    TFT_18_9163_Write_Command(0x36);
    TFT_18_9163_Write_Data(0x48);

    // COLMOD: Interface Pixel format
    TFT_18_9163_Write_Command(0x3A);
    TFT_18_9163_Write_Data(0x66);

    // GAMSET: Select gamma curve
    TFT_18_9163_Write_Command(0x26);
    TFT_18_9163_Write_Data(0x0C4);

    // VCOM Control 1
    TFT_18_9163_Write_Command(0xC5);
    TFT_18_9163_Write_Data(0x2F);
    TFT_18_9163_Write_Data(0x3E);

    // VCOM Offset
    TFT_18_9163_Write_Command(0xC7);
    TFT_18_9163_Write_Data(0x40);

    // FRMCTRL: Frame Rate Control
    TFT_18_9163_Write_Command(0xB1);
    TFT_18_9163_Write_Data(0x0A);
    TFT_18_9163_Write_Data(0x14);

    // PWRCTL1: Power Control 1
    TFT_18_9163_Write_Command(0xC0);
    TFT_18_9163_Write_Data(0x0A);
    TFT_18_9163_Write_Data(0x00);

    // PWRCTL2: Power Control 2
    TFT_18_9163_Write_Command(0xC1);
    TFT_18_9163_Write_Data(0x02);
    
    // X address set                
    TFT_18_9163_Write_Command(0x2A);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x7F);

    // Y address set
    TFT_18_9163_Write_Command(0x2B);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x00);
    TFT_18_9163_Write_Data(0x9F);

    // display on
    // 9.1.19 DISPON (29h): Display On page 189
    // This command is used to recover from DISPLAY OFF mode.
    // Output from the Frame Memory is enabled.
    TFT_18_9163_Write_Command(0x29);

    DelayNanosecondsNoSleep(10*1000);
}

//
// Delay for x nanoseconds without using nanosleep();
// nanosleep can cause a additional delay of up to 10 ms
//
void DelayNanosecondsNoSleep (int delay_ns) {
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    start_time = gettime_now.tv_nsec;       // Get nS value
    while (1) {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        time_difference = gettime_now.tv_nsec - start_time;
        if (time_difference < 0)
            time_difference += 1000000000;  // (Rolls over every 1 second)
        if (time_difference > (delay_ns))       // Delay for # nS reached
            break;
    }
}

