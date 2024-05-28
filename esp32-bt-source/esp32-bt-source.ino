#include "common.h"

#define LOG_MAIN                        "MAIN"
#define DB_DEVICE_PATH                 "/test_bt_headset"

#if (TEST_CRASH_LEVEL > 3)
static uint8_t _headset_selected = -1;
static app_av_state_t _prev_state = APP_AV_STATE_IDLE;
static e_ConnectionType _connection_type = E_CONNECTION_MANUAL;
static st_bluetooth_client _stored_dev = {{0}, {0}, 0};
#endif

#if (TEST_CRASH_LEVEL > 1)
void DB_Init();
#endif

#if (TEST_CRASH_LEVEL > 2)
void SERIAL_Task();
#endif

#if (TEST_CRASH_LEVEL > 0)
void HELP_Show();
#endif

#if (TEST_CRASH_LEVEL > 6)
void _clearBtSelect(void)
{
  memset(&_stored_dev, 0, sizeof(st_bluetooth_client));
  DB_WriteFile(DB_DEVICE_PATH, (uint8_t *)&_stored_dev, sizeof(st_bluetooth_client));
}
#endif

void setup()
{
  Serial.begin(115200);
  delay(2000);

  ESP_LOGI(LOG_MAIN, "Headset Bluetooth Started!");
  ESP_LOGI(LOG_MAIN, "Test Crash Level: %d", TEST_CRASH_LEVEL);

#if (TEST_CRASH_LEVEL > 3)
  (void)_headset_selected;
  (void)_prev_state;
  (void)_connection_type;
  (void)_stored_dev;
#endif

#if (TEST_CRASH_LEVEL > 1)
  DB_Init();
#endif

#if (TEST_CRASH_LEVEL > 6)
  BT_Init();
  BT_ListBondedDevices();
#endif

#if (TEST_CRASH_LEVEL > 0)
  HELP_Show();
#endif
}

void loop()
{
#if (TEST_CRASH_LEVEL > 2)
  SERIAL_Task();
#endif

#if (TEST_CRASH_LEVEL > 6)
  if (_prev_state == APP_AV_STATE_DISCOVERING) {
    if (BT_GetState() == APP_AV_STATE_DISCOVERED) {
      _prev_state = APP_AV_STATE_DISCOVERED;

      vlist_bt_client_t list_client = BT_GetDeviceList();
      ESP_LOGI(LOG_MAIN, "Headset num: %d", list_client.size());
      for (int i = 0; i < list_client.size(); i++) {
        ESP_LOGI(LOG_MAIN, "  (%d) %s - %s", i+1, list_client[i].name, bda_str(list_client[i].peer_bda));
      }
      ESP_LOGI(LOG_MAIN, "Press 1 ~ 9 to select device!");
    }
  } else if (_prev_state == APP_AV_STATE_CONNECTING) {
    if (BT_GetState() == APP_AV_STATE_CONNECTED) {
      _prev_state = APP_AV_STATE_CONNECTED;
      if (_headset_selected < BT_DeviceListSize()) {
        memcpy(&_stored_dev, &BT_GetDeviceList()[_headset_selected], sizeof(st_bluetooth_client));
        DB_WriteFile(DB_DEVICE_PATH, (uint8_t *)&_stored_dev, sizeof(st_bluetooth_client));
      }
      ESP_LOGI(LOG_MAIN, "Headset connected!");
    } else if (BT_GetState() == APP_AV_STATE_UNCONNECTED) {
      _prev_state = APP_AV_STATE_UNCONNECTED;
      ESP_LOGI(LOG_MAIN, "Headset disconnected!");
    }
  } else if (_prev_state == APP_AV_STATE_DISCONNECTING) {
    if (BT_GetState() == APP_AV_STATE_UNCONNECTED) {
      _prev_state = APP_AV_STATE_UNCONNECTED;
      ESP_LOGI(LOG_MAIN, "Headset disconnected!");
    }
  }
#endif
}

#if (TEST_CRASH_LEVEL > 0)
void HELP_Show()
{
  ESP_LOGI("", "Command list:");
  ESP_LOGI("", "  S           Scan");
  ESP_LOGI("", "  1 ~ 9       Select device number after scanning");
  ESP_LOGI("", "  C           Connect to selected device");
  ESP_LOGI("", "  D           Disconnect");
  ESP_LOGI("", "  A           Enable Auto connect mode");
  ESP_LOGI("", "  M           Enable Manual connect mode");
  ESP_LOGI("", "  L           List bonded devices");
  ESP_LOGI("", "  W           Wipe out all bonded devices");
}
#endif

#if (TEST_CRASH_LEVEL > 2)
void SERIAL_Task()
{
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'S': {
#if (TEST_CRASH_LEVEL > 6)
        app_av_state_t state = BT_GetState();
        _headset_selected = -1;
        if (state == APP_AV_STATE_IDLE || state == APP_AV_STATE_DISCOVERED || state == APP_AV_STATE_UNCONNECTED) {
          BT_Scan();
          _prev_state = APP_AV_STATE_DISCOVERING;
        } else { ESP_LOGI(LOG_MAIN, "Scan IGNORED in state: %s", state_str(state)); }
#endif
        break;
      }

      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
      {
#if (TEST_CRASH_LEVEL > 7)
        app_av_state_t state = BT_GetState();
        uint8_t selection = c - '0';
        size_t device_num = BT_DeviceListSize();
        if (state == APP_AV_STATE_DISCOVERED) {
          if (selection <= device_num && selection > 0) {
            _headset_selected = selection - 1;
            ESP_LOGI(LOG_MAIN, "Headset number select: %d - %s", selection, BT_GetDeviceList()[_headset_selected].name);
            ESP_LOGI(LOG_MAIN, "Press C to connect...");
          } else { ESP_LOGE(LOG_MAIN, "Headset number (%d) is invalid. Size = %d", selection, device_num); }
        } else { ESP_LOGE(LOG_MAIN, "Select headset IGNORED in state: %s", state_str(state)); }
#endif
        break;
      }

      case 'C': {
#if (TEST_CRASH_LEVEL > 8)
        if (_headset_selected < BT_DeviceListSize()) {
          ESP_LOGI(LOG_MAIN, "Connecting to %s", bda_str(BT_GetDeviceList()[_headset_selected].peer_bda));
          BT_Connect(&BT_GetDeviceList()[_headset_selected]);
        } else if (BT_ValidateMac(_stored_dev.peer_bda)) {
          _headset_selected = -1;
          ESP_LOGI(LOG_MAIN, "Connecting to stored device %s", bda_str(_stored_dev.peer_bda));
          BT_Connect(&_stored_dev);
        } else {
          ESP_LOGE(LOG_MAIN, "Invalid device!");
          break;
        }

        _prev_state = APP_AV_STATE_CONNECTING;
#endif
        break;
      }

      case 'D': {
#if (TEST_CRASH_LEVEL > 8)
        if (BT_GetState() == APP_AV_STATE_CONNECTED) {
          _prev_state = APP_AV_STATE_DISCONNECTING;
          BT_Disconnect();
          ESP_LOGI(LOG_MAIN, "Disconnecting...");
        } else { ESP_LOGE(LOG_MAIN, "Disconnect IGNORED in state: %s", state_str(BT_GetState())); }
#endif
        break;
      }

      case 'A': {
#if (TEST_CRASH_LEVEL > 4)
        _connection_type = E_CONNECTION_AUTO;
#if (TEST_CRASH_LEVEL > 8)
        BT_SetAutoConnect(true);
#endif
#endif
        break;
      }

      case 'M': {
#if (TEST_CRASH_LEVEL > 4)
        _connection_type = E_CONNECTION_MANUAL;
#if (TEST_CRASH_LEVEL > 8)
        BT_SetAutoConnect(false);
#endif
#endif
        break;
      }

      case 'L': {
#if (TEST_CRASH_LEVEL > 8)
        BT_ListBondedDevices();
#endif
        break;
      }

      case 'W': {
#if (TEST_CRASH_LEVEL > 8)
        BT_WipeBondedDevices();
        _clearBtSelect();
#endif
        break;
      }

      default: break;
    }
  }
}
#endif

#if (TEST_CRASH_LEVEL > 1)
bool DB_ReadFile(const char *path, uint8_t *data, size_t size) {
  bool result = true;

  File file = SPIFFS.open(path);
  if (!file) {
    ESP_LOGE(LOG_MAIN, "- failed to open file for reading %s", path);
    return false;
  }

  if (file.isDirectory()) {
    file.close();
    ESP_LOGE(LOG_MAIN, "- Cannot read directory %s", path);
    return false;
  }  

  if (file.available()) {
    int readSize = file.readBytes((char *)data, size);
    if (readSize < size) {
      ESP_LOGE(LOG_MAIN, " - read size error %d - expected %d", readSize, size);
      result = false;
    }
  } else {
    ESP_LOGE(LOG_MAIN, " - file not available. read error");
    result = false;
  }
  file.close();

  return result;
}

bool DB_WriteFile(const char *path, uint8_t *data, size_t size) {
  bool result = true;

  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    ESP_LOGE(LOG_MAIN, "- failed to open file for writing %s", path);
    return false;
  }

  int writeSize = file.write(data, size);
  if (writeSize < size) {
    ESP_LOGE(LOG_MAIN, " - write size error %d - expected %d", writeSize, size);
    result = false;
  }
  file.close();

  return result;
}

bool DB_FileExists(const char *path)
{
  return SPIFFS.exists(path);
}

void DB_Init()
{
  ESP_LOGI(LOG_MAIN, "Database initializing...");
  if (!SPIFFS.begin()) {
    ESP_LOGE(LOG_MAIN, "SPIFFS FAILED to begin!");
  } else {
    if (!DB_FileExists(DB_DEVICE_PATH)) {
      ESP_LOGE(LOG_MAIN, "%s does not exist!", DB_DEVICE_PATH);
#if (TEST_CRASH_LEVEL > 6)
      _clearBtSelect();
#endif
    }

#if (TEST_CRASH_LEVEL > 6)
    DB_ReadFile(DB_DEVICE_PATH, (uint8_t *)&_stored_dev, sizeof(st_bluetooth_client));

    if (BT_ValidateMac(_stored_dev.peer_bda)) {
      ESP_LOGI(LOG_MAIN, "Stored headset: %s - %s", _stored_dev.name, bda_str(_stored_dev.peer_bda));
    } else {
      ESP_LOGE(LOG_MAIN, "Stored headset is empty");
    }
#endif
  }

  ESP_LOGI(LOG_MAIN, "Database init done!");
}
#endif

#if (TEST_CRASH_LEVEL > 6)
#define c3_frequency  130.81
int32_t get_data_sinwave(uint8_t *data, int32_t len)
{
  static double m_time = 0.0;
  static int count = 0;
  double m_amplitude = 10000.0;  // -32,768 to 32,767
  double m_deltaTime = 1.0 / 44100.0;
  double m_phase = 0.0;
  double double_Pi = PI * 2.0;
  Frame *frame = (Frame*)data;
  int32_t channel_len = len >> 2;
  // fill the channel data
  for (int sample = 0; sample < channel_len; ++sample) {
      double angle = double_Pi * c3_frequency * m_time + m_phase;
      frame[sample].channel1 = m_amplitude * sin(angle);
      frame[sample].channel2 = frame[sample].channel1;
      m_time += m_deltaTime;
  }

  count++;
  if (count > 200) {
    count = 0;
  }
  return len;
}
#endif
