This is a simple serial logger for the ESP32, originally intended
for longterm logging of NMEA sentances from a GPS module with a
LOLIN32 PRO board.

Data is read from UART2, buffered into larger blocks, and then written 
out to the SD card. Because the data is buffered it is possible to 
remove the SD card, copy the data to another device, then insert the
SD card again and maintain a continious log of data. 

The length of time that the SD card can be removed depends on the data
rate, but the default allows for buffering of up to 32k of data.

To avoid data corruption, the LED on 'WARN_GPIO' pin (set to pin 5)
will be on for a few seconds before the data is written - to avoid 
data corruption do not insert or remove the SD card while the LED is
on - just wait for a few seconds for it to go out.

Here are most of the important settings that you may wish to customize:

    /* Parameters for the SD card */
    #define PIN_NUM_MISO 2
    #define PIN_NUM_MOSI 15
    #define PIN_NUM_CLK  14
    #define PIN_NUM_CS   13

    /* Parameters for the data buffers */
    #define N_BUFFERS   (4)
    #define BUFFER_SIZE (8192)

    /* GPIO for the warning LED  - active low*/
    #define WARN_GPIO 5

    /* Which UART is the GPS module on? */
    #define GPS_UART_NUM      UART_NUM_2
    #define GPS_UART_RX_PIN  (22)
    #define GPS_UART_TX_PIN  (23)
    #define GPS_UART_RTS_PIN (21)
    #define GPS_UART_CTS_PIN (35)

The serial port parameters are set using constants for configuring the port:

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

The file name is set with:

    #define FILENAME "gps_log.txt" 

Hope it may be of some use to you.
