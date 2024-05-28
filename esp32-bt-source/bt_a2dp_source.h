#pragma once

#include "common.h"

#if (TEST_CRASH_LEVEL > 3)
#define SIZE_NAME_CLIENT_MAX              248
#define BT_NAME_LEN_MAX                   (SIZE_NAME_CLIENT_MAX + 1)
#define BONDED_DEVICES_MAX                20

enum {
  APP_AV_MEDIA_STATE_IDLE,
  APP_AV_MEDIA_STATE_STARTING,
  APP_AV_MEDIA_STATE_STARTED,
  APP_AV_MEDIA_STATE_STOPPING,
};

/* A2DP global state */
typedef enum {
  APP_AV_STATE_IDLE,
  APP_AV_STATE_DISCOVERING,
  APP_AV_STATE_DISCOVERED,
  APP_AV_STATE_UNCONNECTED,
  APP_AV_STATE_CONNECTING,
  APP_AV_STATE_CONNECTED,
  APP_AV_STATE_DISCONNECTING,
  APP_AV_STATE_ENDING,
  APP_AV_STATE_MAX
} app_av_state_t;

typedef enum {
  E_CONNECTION_MANUAL = (0),
  E_CONNECTION_AUTO
} e_ConnectionType;

typedef struct
{
  char name[BT_NAME_LEN_MAX];
  esp_bd_addr_t peer_bda;
  int rssi;
} st_bluetooth_client;

typedef std::vector<st_bluetooth_client> vlist_bt_client_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if (TEST_CRASH_LEVEL > 6)
void BT_Init();
void BT_End();
bool BT_Scan();
bool BT_Connect(st_bluetooth_client *client);
bool BT_Disconnect();
void BT_ClearDeviceList();
bool BT_ValidateMac(uint8_t *mac);
bool BT_SetAutoConnect(bool autoconnect);
bool BT_IsBonded(esp_bd_addr_t addr);
bool BT_RemoveBonded(esp_bd_addr_t addr);
void BT_WipeBondedDevices(void);
void BT_ListBondedDevices();
app_av_state_t BT_GetState(void);
uint8_t BT_DeviceListSize();
std::vector<st_bluetooth_client> &BT_GetDeviceList();
const char* state_str(app_av_state_t state);
const char* bda_str(esp_bd_addr_t bda);
#endif
#ifdef __cplusplus
}
#endif
