/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "gps_logger";

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
#define UART_BUF_SIZE    (128)
#define PATTERN_CHR_NUM  (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/


         static uint8_t buffers[N_BUFFERS][BUFFER_SIZE];
volatile static uint8_t current_buffer         = 0;
volatile static uint8_t next_buffer_to_write   = 0;
volatile static uint8_t seconds_to_write = 0;
volatile static uint32_t used_in_current_buffer = 0;
static QueueHandle_t uart0_queue;


void warning_led_set() {
	    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
	     *        muxed to GPIO on reset already, but some default to other
	     *               functions and need to be switched to GPIO. Consult the
	     *                      Technical Reference for a list of pads and their default
	     *                             functions.)
	     *                                 */
  gpio_pad_select_gpio(WARN_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(WARN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(WARN_GPIO, 0);
}

void warning_led_unset() {
  gpio_set_level(WARN_GPIO, 1);
  gpio_set_direction(WARN_GPIO, GPIO_MODE_INPUT);
}

int write_out_any_buffers(void)
{ 
  ESP_LOGI(TAG, "Initializing SD card");

  ESP_LOGI(TAG, "Using SDMMC peripheral");
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
  gpio_set_pull_mode( 2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
  gpio_set_pull_mode( 4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
  gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
  gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
  };

  sdmmc_card_t* card;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount filesystem. "
           "If you want the card to be formatted, set format_if_mount_failed = true.");
    } else {
      ESP_LOGE(TAG, "Failed to initialize the card (%s). "
          "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    }
    return 0;
  }

  FILE* f = fopen("/sdcard/gpslog.txt", "a");
  if(f == NULL) {
    FILE* f = fopen("/sdcard/gpslog.txt", "w");
    if (f == NULL) {
      ESP_LOGE(TAG, "Failed to open file for writing");
      return 0;
    }
    ESP_LOGI(TAG, "Created new file");
  } else {
    ESP_LOGI(TAG, "Opening file");
  }
  /* Attempt to write out any full buffers */
  while(next_buffer_to_write != current_buffer) {
    if(fwrite(buffers[next_buffer_to_write], BUFFER_SIZE, 1, f)!=1) {
      ESP_LOGI(TAG, "Write error");
      break;
    }
    next_buffer_to_write++;
    if(next_buffer_to_write == N_BUFFERS) {
      next_buffer_to_write = 0;
    }
  }
  ESP_LOGI(TAG, "Closing file");
  fclose(f);

  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdmmc_unmount();
  return 1;
}

static void uart_event_task(void *pvParameters)
{
    int left_to_read;
    uart_event_t event;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
		    left_to_read = event.size;
		    while(left_to_read > 0) {
		       int amount_to_read = BUFFER_SIZE - used_in_current_buffer;
		       if(amount_to_read > left_to_read)
			  amount_to_read = left_to_read;
                       uart_read_bytes(GPS_UART_NUM, buffers[current_buffer]+used_in_current_buffer, amount_to_read, portMAX_DELAY);
		       left_to_read -= amount_to_read;

                       used_in_current_buffer += amount_to_read;

		       /* Is the current buffer full */
                       if(used_in_current_buffer == BUFFER_SIZE) {
			 int next_buffer;
                         /* Move onto next buffer */
			 next_buffer = current_buffer;
			 next_buffer++;
                         if(next_buffer == N_BUFFERS)
                           next_buffer = 0;

			 if(next_buffer == next_buffer_to_write) {
                           ESP_LOGI(TAG, "Overrun the buffers - please insert an SD card");
			 } else {
                           current_buffer = next_buffer;
			 }
                         used_in_current_buffer = 0;
                       }
		    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(GPS_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(GPS_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED]");
                    uart_flush_input(GPS_UART_NUM);
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    /* Should never get here */
    vTaskDelete(NULL);
}

static void config_gps_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART_NUM, &uart_config);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(GPS_UART_NUM, GPS_UART_RX_PIN, GPS_UART_TX_PIN, GPS_UART_RTS_PIN, GPS_UART_CTS_PIN);
    //Install UART driver, and get the queue.
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart0_queue, 0);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void app_main(void)
{
  ESP_LOGI(TAG, "Configuring UART");
  config_gps_uart();

  ESP_LOGI(TAG, "Starting collecting data");
  while(1) {
    sleep(1);
    /* We use new lines from the GPS to act as a rough timer */
    if(seconds_to_write == 1) {
      if(write_out_any_buffers()) {
        seconds_to_write = 0;
      } else {
        seconds_to_write = 20;
      }
      warning_led_unset();
    } else if(seconds_to_write > 0) {
      if(seconds_to_write == 5) 
        warning_led_set();
      seconds_to_write--;
    }
    /* Do we need to set up for the next write */
    if(current_buffer != next_buffer_to_write && seconds_to_write == 0)
      seconds_to_write = 10;
  }
}

