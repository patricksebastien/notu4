// TODO
// CHOOSE THE RIGHT GPIO and update
// CHECK IP ADDRESS...
// SEND TO PD
// KEEP MIDI UART

#include <ableton/Link.hpp>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include "driver/uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "esp_sleep.h"

// UPD
#include <string.h>
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define HOST_IP_ADDR "192.168.2.101"
#define PORT 3333

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;
struct sockaddr_in dest_addr;
int sock;

#define LED GPIO_NUM_2
#define PRINT_LINK_STATE false

// Serial midi
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define BUF_SIZE (1024)
#define MIDI_TIMING_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_SONG_POSITION_POINTER 0xF2

// Global
static void periodic_timer_callback(void* arg);
esp_timer_handle_t periodic_timer;
bool startStopCB = false;
bool startStopState = false;
double curr_beat_time;
double prev_beat_time;

// gpio
extern "C" {
  
  // BUTTONS
  #define GPIO_INPUT_IO_0     GPIO_NUM_36
  #define GPIO_INPUT_IO_1     GPIO_NUM_39
  #define GPIO_INPUT_IO_2     GPIO_NUM_34
  #define GPIO_INPUT_IO_3     GPIO_NUM_35
  #define GPIO_INPUT_IO_4     GPIO_NUM_32
  #define GPIO_INPUT_IO_5     GPIO_NUM_33
  #define GPIO_INPUT_IO_6     GPIO_NUM_25
  #define GPIO_INPUT_IO_7     GPIO_NUM_26
  #define GPIO_INPUT_IO_8     GPIO_NUM_27
  #define GPIO_INPUT_IO_9     GPIO_NUM_14
  
  #define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4) | (1ULL<<GPIO_INPUT_IO_5) | (1ULL<<GPIO_INPUT_IO_6) | (1ULL<<GPIO_INPUT_IO_7) | (1ULL<<GPIO_INPUT_IO_8) | (1ULL<<GPIO_INPUT_IO_9))

  // LEDS
  #define GPIO_OUTPUT_IO_0    GPIO_NUM_13
  #define GPIO_OUTPUT_IO_1    GPIO_NUM_9
  #define GPIO_OUTPUT_IO_2    GPIO_NUM_10
  #define GPIO_OUTPUT_IO_3    GPIO_NUM_11
  
  #define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3))
  #define ESP_INTR_FLAG_DEFAULT 0

  static xQueueHandle gpio_evt_queue = NULL;

  static void IRAM_ATTR gpio_isr_handler(void* arg)
  {
      uint32_t gpio_num = (uint32_t) arg;
      xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
  }

  static void gpio_task_example(void* arg)
  {
      uint32_t io_num;
      for(;;) {
          if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
              
              // check gpio_input for low
              // debounce in pd...
              if(io_num == GPIO_NUM_36 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "0";
              } else if(io_num == GPIO_NUM_39 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "1";
              } else if(io_num == GPIO_NUM_34 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "2";
              } else if(io_num == GPIO_NUM_35 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "3";
              } else if(io_num == GPIO_NUM_32 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "4";
              } else if(io_num == GPIO_NUM_33 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "5";
              } else if(io_num == GPIO_NUM_25 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "6";
              } else if(io_num == GPIO_NUM_26 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "7";
              } else if(io_num == GPIO_NUM_27 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "8";
              } else if(io_num == GPIO_NUM_14 && gpio_get_level((gpio_num_t)io_num) == 0) {
                payload = "9";
              }

              int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                  if (err < 0) {
                      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                      break;
                  }
                  ESP_LOGI(TAG, "Message sent");
                
                printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
          }
      }
  }
}


unsigned int if_nametoindex(const char* ifName)
{
  return 0;
}

char* if_indextoname(unsigned int ifIndex, char* ifName)
{
  return nullptr;
}

void IRAM_ATTR timer_group0_isr(void* userParam)
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[0].config.alarm_en = 1;

  xSemaphoreGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

void timerGroup0Init(int timerPeriodUS, void* userParam)
{
  timer_config_t config = {.alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80};

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timerPeriodUS);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_group0_isr, userParam, 0, nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void printTask(void* userParam)
{
  auto link = static_cast<ableton::Link*>(userParam);
  const auto quantum = 4.0;

  while (true)
  {
    const auto sessionState = link->captureAppSessionState();
    const auto numPeers = link->numPeers();
    const auto time = link->clock().micros();
    const auto beats = sessionState.beatAtTime(time, quantum);
    std::cout << std::defaultfloat << "| peers: " << numPeers << " | "
              << "tempo: " << sessionState.tempo() << " | " << std::fixed
              << "beats: " << beats << " |" << std::endl;
    vTaskDelay(800 / portTICK_PERIOD_MS);
  }
}

// callbacks
void tempoChanged(double tempo) {
    double midiClockMicroSecond = ((60000 / tempo) / 24) * 1000;
    esp_timer_handle_t periodic_timer_handle = (esp_timer_handle_t) periodic_timer;
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_handle, midiClockMicroSecond));
}

void startStopChanged(bool state) {
  // received as soon as sent
  // need to wait for phase to be 0 (and deal with latency...)
  startStopCB = state;
}

void tickTask(void* userParam)
{
  // connect link
  ableton::Link link(120.0f);
  link.enable(true);
  link.enableStartStopSync(true); // if not no callback for start/stop

  // callbacks
  link.setTempoCallback(tempoChanged);
  link.setStartStopCallback(startStopChanged);

  // debug
  if (PRINT_LINK_STATE)
  {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }
  gpio_set_direction(LED, GPIO_MODE_OUTPUT);

  // phase
  while (true)
  { 
    xSemaphoreTake(userParam, portMAX_DELAY);

    const auto state = link.captureAudioSessionState();
    const auto phase = state.phaseAtTime(link.clock().micros(), 0.25);
    gpio_set_level(LED, fmodf(phase, 1.) < 0.1);
   
    curr_beat_time = state.beatAtTime(link.clock().micros(), 4);
    const double curr_phase = fmod(curr_beat_time, 4);
    if (curr_beat_time > prev_beat_time) {
      const double prev_phase = fmod(prev_beat_time, 4);
      const double prev_step = floor(prev_phase * 1);
      const double curr_step = floor(curr_phase * 1);
      if (prev_phase - curr_phase > 4 / 2 || prev_step != curr_step) {
        
        // only if playing
        if(startStopCB) {
        // show step with leds
          if(curr_step == 0) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 1);
          } else {
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
          }
          if(curr_step == 1) {
            gpio_set_level(GPIO_OUTPUT_IO_1, 1);
          } else {
            gpio_set_level(GPIO_OUTPUT_IO_1, 0);
          }
          if(curr_step == 2) {
            gpio_set_level(GPIO_OUTPUT_IO_2, 1);
          } else {
            gpio_set_level(GPIO_OUTPUT_IO_2, 0);
          }
          if(curr_step == 3) {
            gpio_set_level(GPIO_OUTPUT_IO_3, 1);
          } else {
            gpio_set_level(GPIO_OUTPUT_IO_3, 0);
          }
        }

        if(curr_step == 0 && startStopState != startStopCB) {
          char zedata[] = { MIDI_START };
          uart_write_bytes(UART_NUM_1, zedata, 1);
          uart_write_bytes(UART_NUM_1, 0, 1);
        } else {
          char zedata[] = { MIDI_STOP };
          uart_write_bytes(UART_NUM_1, zedata, 1);
          uart_write_bytes(UART_NUM_1, 0, 1);
        }
        startStopState = startStopCB;
        
      }
    }
    prev_beat_time = curr_beat_time;

    portYIELD();
  }
}

static void periodic_timer_callback(void* arg)
{
    char zedata[] = { MIDI_TIMING_CLOCK };
    uart_write_bytes(UART_NUM_1, zedata, 1);
}



// UPD
static void udp_client_task(void *pvParameters)
{
    while (1) {

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  // serial
  uart_config_t uart_config = {
    .baud_rate = 31250, // midi speed
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

  // upd init + timer
  dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
  sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
  if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

  // link timer - phase
  SemaphoreHandle_t tickSemphr = xSemaphoreCreateBinary();
  timerGroup0Init(1000, tickSemphr);
  xTaskCreate(tickTask, "tick", 8192, tickSemphr, 1, nullptr);

  // midi clock
  const esp_timer_create_args_t periodic_timer_args = {
          .callback = &periodic_timer_callback,
          .name = "periodic"
  };
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 20833.333333333)); // 120 bpm by default


/*
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);*/

  // gpio
  gpio_config_t io_conf;   // GPIO

      //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_NEGEDGE; 
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;  //bit mask of the pins, use GPIO4
  io_conf.mode = GPIO_MODE_INPUT;  //set as input mode   
  io_conf.pull_down_en = (gpio_pulldown_t)0; // 0 //disable pull-down mode
  io_conf.pull_up_en = (gpio_pullup_t)1; //enable pull-up mode
  gpio_config(&io_conf);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); //create a queue to handle gpio event from isr
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL); //start gpio task

      //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}
