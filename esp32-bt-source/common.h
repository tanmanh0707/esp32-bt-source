#pragma once

/* C++ */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <vector>

/* ESP */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include <mbedtls/md.h>
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp32-hal-log.h"
#include "esp32-hal-bt.h"

/* Arduino */
#include <FS.h>
#include <SPIFFS.h>

#define TEST_CRASH_LEVEL              10

/* Application */
#include "bt_a2dp_source.h"

#define MEMCMP_EQUAL                  0

/**
 * @brief Utility structure that can be used to split a int32_t up into 2 separate channels with int16_t data.
 * @author Phil Schatzmann
 * @copyright Apache License Version 2
 */
struct __attribute__((packed)) Frame {
  int16_t channel1;
  int16_t channel2;

  Frame(int v=0){
    channel1 = channel2 = v;
  }
  
  Frame(int ch1, int ch2){
    channel1 = ch1;
    channel2 = ch2;
  }

};

// support for legacy name;
using Channels = Frame;

int32_t get_data_sinwave(uint8_t *data, int32_t channel_len);