#include "bt_a2dp_source.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (TEST_CRASH_LEVEL > 5)
#define BT_AV_TAG                                  "BT"

/* device name */
#define LOCAL_DEVICE_NAME                           "Smart Connect"
#define BDA_STR_LEN                                 18

enum {
  BT_APP_STACK_UP_EVT   = 0x0000,    /* event for stack up */
  BT_APP_REQUEST_SCAN,
  BT_APP_REQUEST_CONNECT,
  BT_APP_REQUEST_DISCONNECT,
  BT_APP_REQUEST_AUTO_CONNECT,
  BT_APP_REQUEST_NON_AUTO_CONNECT,
  BT_APP_REQUEST_END,
  BT_APP_HEART_BEAT_EVT = 0xff00,    /* event for heart beat */
  BT_APP_MEDIA_CHECK_SRC,
};

/* signal for dispatcher */
#define BT_APP_SIG_WORK_DISPATCH    (0x01)

/* AVRCP used transaction label */
#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_RN_VOLUME_CHANGE    (1)

/**
 * @brief    handler for the dispatched work
 *
 * @param [in] event  message event id
 * @param [in] param  pointer to the parameter
 */
typedef void (* bt_app_cb_t) (uint16_t event, void *param);

/* message to be sent */
typedef struct {
  uint16_t             sig;      /*!< signal to bt_app_task */
  uint16_t             event;    /*!< message event id */
  bt_app_cb_t          cb;       /*!< context switch callback */
  void                 *param;   /*!< parameter area needs to be last */
} bt_app_msg_t;

/**
 * @brief    parameter deep-copy function to be customized
 *
 * @param [in] p_dest  pointer to the destination
 * @param [in] p_src   pointer to the source
 * @param [in] len     data length in byte
 */
typedef void (* bt_app_copy_cb_td) (void *p_dest, void *p_src, int len);

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/* avrc controller event handler */
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/* callback function for A2DP source */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/* callback function for A2DP source audio data stream */
// static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);

/* callback function for AVRCP controller */
static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/* handler for heart beat timer */
#if 0
static void bt_app_a2d_heart_beat(TimerHandle_t arg);
#endif

/* A2DP application state machine */
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

/* utils for transfer BLuetooth Deveice Address into string form */
static char *bda2str(esp_bd_addr_t bda, char *str, size_t size);

static void bt_app_av_media_proc(uint16_t event, void *param);
/* A2DP application state machine handler for each state */
static void bt_app_av_state_idle_hdlr(uint16_t event, void *param);
static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param);
static bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_td p_copy_cback);
/*********************************
 * STATIC VARIABLE DEFINITIONS
 ********************************/

static esp_bd_addr_t s_peer_bda = {0};                        /* Bluetooth Device Address of peer device*/
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];  /* Bluetooth Device Name of peer device*/
static app_av_state_t s_a2d_state = APP_AV_STATE_IDLE;        /* A2DP global state */
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;           /* sub states of APP_AV_STATE_CONNECTED */
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;         /* AVRC target notification event capability bit mask */
#if 0
static TimerHandle_t s_tmr;                                   /* handle of heart beat timer */
#endif
static QueueHandle_t s_bt_app_task_queue = NULL;
static TaskHandle_t s_bt_app_task_handle = NULL;
static std::vector<st_bluetooth_client> _deviceList;

#define changeState(newState)               _changeState(newState, __LINE__)

const char* connection_str(esp_a2d_connection_state_t state) {
  const char *conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};

  if (state < 4) {
    return conn_state_str[state];
  } else {
    return "Unknown";
  }
}

const char* audio_str(esp_a2d_audio_state_t state) {
  const char *audio_state_str[3] = {"Suspended", "Stopped", "Started"};

  if (state < 3) {
    return audio_state_str[state];
  } else {
    return "Unknown";
  }
}

const char* state_str(app_av_state_t state) {
  const char *app_state_str[APP_AV_STATE_MAX] = {"IDLE", "DISCOVERING", "DISCOVERED", "UNCONNECTED", "CONNECTING", "CONNECTED", "DISCONNECTING", "ENDING"};
  
  if (state < APP_AV_STATE_MAX) {
    return app_state_str[state];
  } else {
    return "Unknown";
  }
}

const char* bda_str(esp_bd_addr_t bda){
  static char bda_str[BDA_STR_LEN] = {0};
  sprintf(bda_str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return (const char*)bda_str;
}

static void _changeState(app_av_state_t newState, int line) {
  ESP_LOGI(BT_AV_TAG, "[%04d] App state from %s to %s", line, state_str(s_a2d_state), state_str(newState));
  s_a2d_state = newState;
}

bool BT_ValidateMac(uint8_t *mac)
{
  uint8_t mac_all0[6] = {0u, 0u, 0u, 0u, 0u, 0u};
  uint8_t mac_allF[6] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

  if (!memcmp(mac, mac_all0, 6) || !memcmp(mac, mac_allF, 6))
  {
    return false;
  }

  return true;
}

static bool validate_name(unsigned char *name)
{
  if (name == NULL || strlen((char*)name) == 0) {
    return false;
  }

  return true;
}

app_av_state_t BT_GetState(void) {
  return s_a2d_state;
}

void BT_ClearDeviceList() {
  _deviceList.clear();
}

uint8_t BT_DeviceListSize() {
  return _deviceList.size();
}

std::vector<st_bluetooth_client> &BT_GetDeviceList() {
  return _deviceList;
}

bool BT_Scan() {
  return bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_REQUEST_SCAN, NULL, 0, NULL);
}

bool BT_Connect(st_bluetooth_client *client) {
  return bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_REQUEST_CONNECT, (void*)client, sizeof(st_bluetooth_client), NULL);
}

bool BT_Disconnect() {
  return bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_REQUEST_DISCONNECT, NULL, 0, NULL);
}

bool BT_SetAutoConnect(bool autoconnect) {
  if (autoconnect) {
    return bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_REQUEST_AUTO_CONNECT, NULL, 0, NULL);
  } else {
    return bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_REQUEST_NON_AUTO_CONNECT, NULL, 0, NULL);
  }
}

bool BT_IsBonded(esp_bd_addr_t addr)
{
  bool res = false;
  esp_bd_addr_t bondedDevicesAddr[BONDED_DEVICES_MAX];
  int count = esp_bt_gap_get_bond_device_num();

  if (count) {

  } else {
    ESP_LOGE(BT_AV_TAG, "No bonded devices found!");
    return false;
  }

  esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, bondedDevicesAddr);
  if (ESP_OK == tError) {
    for(int i = 0; i < count; i++) {
      if (memcmp(addr, bondedDevicesAddr[i], ESP_BD_ADDR_LEN) == MEMCMP_EQUAL) {
        ESP_LOGI(BT_AV_TAG, "Device %s is bonded!", bda_str(bondedDevicesAddr[i]));
        res = true;
        break;
      }
    }
  }

  return res;
}

bool BT_RemoveBonded(esp_bd_addr_t addr)
{
  bool res = false;

  if (BT_IsBonded(addr)) {
    esp_err_t tError = esp_bt_gap_remove_bond_device(addr);
    if(ESP_OK == tError) {
      ESP_LOGI(BT_AV_TAG, "Bonded device %s has been removed!", bda_str(addr));
      res = true;
    } else {
      ESP_LOGD(BT_AV_TAG, "Failed to remove bonded device %s", bda_str(addr));
    }
  }

  return res;
}

void BT_ListBondedDevices()
{
  char bda_str[BDA_STR_LEN];
  esp_bd_addr_t bondedDevicesAddr[BONDED_DEVICES_MAX];

  int count = esp_bt_gap_get_bond_device_num();
  if (!count) {
    ESP_LOGE(BT_AV_TAG, "No bonded devices found!");
    return;
  }

  ESP_LOGI(BT_AV_TAG, "Number of bonded devices: %d", count);
  if (BONDED_DEVICES_MAX < count) {
    count = BONDED_DEVICES_MAX; 
  }

  esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, bondedDevicesAddr);
  if (ESP_OK == tError) {
    for(int i = 0; i < count; i++) {
      ESP_LOGI(BT_AV_TAG, "  #%d -> %s", i, bda2str(bondedDevicesAddr[i], bda_str, BDA_STR_LEN));
    }
  } else {
    ESP_LOGE(BT_AV_TAG, "Cannot get bonded device list! (%d)", tError);
  }
}

void BT_WipeBondedDevices(void)
{
  char bda_str[BDA_STR_LEN];
  esp_bd_addr_t bondedDevicesAddr[BONDED_DEVICES_MAX];

  int count = esp_bt_gap_get_bond_device_num();
  if (!count) {
    ESP_LOGE(BT_AV_TAG, "No bonded devices found!");
    return;
  }

  ESP_LOGI(BT_AV_TAG, "Number of bonded devices: %d", count);
  if (BONDED_DEVICES_MAX < count) {
    count = BONDED_DEVICES_MAX; 
  }

  esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, bondedDevicesAddr);
  if (ESP_OK == tError) {
    for(int i = 0; i < count; i++) {
      ESP_LOGI(BT_AV_TAG, "  #%d -> %s", i, bda2str(bondedDevicesAddr[i], bda_str, BDA_STR_LEN));
      esp_err_t tError = esp_bt_gap_remove_bond_device(bondedDevicesAddr[i]);
      if(ESP_OK == tError) {
        ESP_LOGI(BT_AV_TAG, "Removed bonded device #%d", i); 
      } else {
        ESP_LOGE(BT_AV_TAG, "Failed to remove bonded device #%d", i);
      }
    }
  }
}

static void set_scan_mode_connectable(esp_bt_connection_mode_t connectMode, esp_bt_discovery_mode_t discoverMode)
{
  ESP_LOGI(BT_AV_TAG, "connectable: %s - discoverable: %s",
            connectMode ==  ESP_BT_CONNECTABLE? "yes" : "no",
            discoverMode == ESP_BT_GENERAL_DISCOVERABLE? "yes" : "no");

  if (esp_bt_gap_set_scan_mode(connectMode, discoverMode) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "esp_bt_gap_set_scan_mode failed!");
  }
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < BDA_STR_LEN) {
    return NULL;
  }

  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
  if (msg->cb) {
    msg->cb(msg->event, msg->param);
  }
}

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
  if (msg == NULL) {
    return false;
  }

  if (pdTRUE != xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_PERIOD_MS)) {
    ESP_LOGE(BT_AV_TAG, "xQueue send failed");
    return false;
  }

  return true;
}

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_td p_copy_cback)
{
    ESP_LOGD(BT_AV_TAG, "event: 0x%x, param len: %d", event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(msg.param, p_params, param_len);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

static void bt_app_task_handler(void *arg)
{
  bt_app_msg_t msg;

  for (;;) {
    /* receive message from work queue and handle it */
    if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (TickType_t)portMAX_DELAY)) {
      ESP_LOGD(BT_AV_TAG, "signal: 0x%x, event: 0x%x", msg.sig, msg.event);

      switch (msg.sig) {
      case BT_APP_SIG_WORK_DISPATCH:
        bt_app_work_dispatched(&msg);
        break;
      default:
        ESP_LOGW(BT_AV_TAG, "unhandled signal: %d", msg.sig);
        break;
      }

      if (msg.param) {
        free(msg.param);
      }
    }
  }
}

static void bt_app_task_start_up(void)
{
  s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
  xTaskCreate(bt_app_task_handler, "BtAppTask", 2048, NULL, 10, &s_bt_app_task_handle);
}

void bt_app_task_shut_down(void)
{
  if (s_bt_app_task_handle) {
    vTaskDelete(s_bt_app_task_handle);
    s_bt_app_task_handle = NULL;
  }
  if (s_bt_app_task_queue) {
    vQueueDelete(s_bt_app_task_queue);
    s_bt_app_task_queue = NULL;
  }
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
  uint8_t *rmt_bdname = NULL;
  uint8_t rmt_bdname_len = 0;

  if (!eir) {
    return false;
  }

  rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
  if (!rmt_bdname) {
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
  }

  if (rmt_bdname) {
    if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
      rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
    }

    if (bdname) {
      memcpy(bdname, rmt_bdname, rmt_bdname_len);
      bdname[rmt_bdname_len] = '\0';
    }
    if (bdname_len) {
      *bdname_len = rmt_bdname_len;
    }
    return true;
  }

  return false;
}

static void add_device(unsigned char *name, esp_bd_addr_t bd_addr, int rssi)
{
  st_bluetooth_client bt_client;
  char bdastr[BDA_STR_LEN] = {0};

  bda2str(bd_addr, bdastr, BDA_STR_LEN);
  ESP_LOGD(BT_AV_TAG, "  - Name: %s, Address: %s", name, bdastr);

  memset(&bt_client, 0, sizeof(st_bluetooth_client));
  strncpy(bt_client.name, (char*)name, ESP_BT_GAP_MAX_BDNAME_LEN);
  memcpy(bt_client.peer_bda, bd_addr, ESP_BD_ADDR_LEN);
  bt_client.rssi = rssi;
  _deviceList.push_back(bt_client);
}

static bool check_device_exist(unsigned char *name, esp_bd_addr_t bd_addr)
{
  for (auto it = _deviceList.rbegin(); it != _deviceList.rend(); ++it) {
    if (!strcmp((char*)name, it->name) && !memcmp(bd_addr, it->peer_bda, ESP_BD_ADDR_LEN)) {
      return true;
    }
  }

  return false;
}

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
  uint32_t cod = 0;
  int32_t rssi = -129; /* invalid value */
  uint8_t *eir = NULL;
  unsigned char bt_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
  esp_bt_gap_dev_prop_t *p;
	
  for (int i = 0; i < param->disc_res.num_prop; i++) {
    p = param->disc_res.prop + i;
    switch (p->type) {
    case ESP_BT_GAP_DEV_PROP_COD:
      cod = *(uint32_t *)(p->val);
      break;
    case ESP_BT_GAP_DEV_PROP_RSSI:
      rssi = *(int8_t *)(p->val);
      break;
    case ESP_BT_GAP_DEV_PROP_EIR:
      eir = (uint8_t *)(p->val);
      break;
    case ESP_BT_GAP_DEV_PROP_BDNAME:
    default:
      break;
    }
  }

  ESP_LOGD(BT_AV_TAG, "Scanned device: %s, COD: 0x%x, RSSI: %d", bda_str(param->disc_res.bda), cod, rssi);
  /* search for device with MAJOR DEVICE class as "Audio/Visual" in COD */
  if ((esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_AV))
	{
    if (!esp_bt_gap_is_valid_cod(cod) || !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_AUDIO)) {
      return;
    }

		/* search for device name in its extended inqury response */
		if (eir)
		{
      memset(bt_name, 0x00, sizeof(bt_name));
			get_name_from_eir(eir, bt_name, NULL);
			

      if (!validate_name(bt_name) || !BT_ValidateMac(param->disc_res.bda)) {
        return;
      }

      if (!check_device_exist(bt_name, param->disc_res.bda)) {
        add_device(bt_name, param->disc_res.bda, rssi);
      }
		}
		else { ESP_LOGD(BT_AV_TAG, "--Compatiblity: Scanned Device not Compatible"); }
	} else{ ESP_LOGD(BT_AV_TAG, "--Compatiblity: Scanned Device not Compatible."); }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  switch (event) {
  case ESP_BT_GAP_DISC_RES_EVT: {
    if (s_a2d_state == APP_AV_STATE_DISCOVERING) {
      filter_inquiry_scan_result(param);
    }
    break;
  }

  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      ESP_LOGI(BT_AV_TAG, "Discovery Stopped!");
      if (s_a2d_state == APP_AV_STATE_DISCOVERING) {
        changeState(APP_AV_STATE_DISCOVERED);
      }
    } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
      ESP_LOGI(BT_AV_TAG, "Discovery Started!");
    }
    break;
  }
  /* when authentication completed, this event comes */
  case ESP_BT_GAP_AUTH_CMPL_EVT: {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
      esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
    } else {
      ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
    }
    break;
  }
  /* when Legacy Pairing pin code requested, this event comes */
  case ESP_BT_GAP_PIN_REQ_EVT: {
      ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit: %d", param->pin_req.min_16_digit);
      if (param->pin_req.min_16_digit) {
        ESP_LOGI(BT_AV_TAG, "Input pin code: 0000 0000 0000 0000");
        esp_bt_pin_code_t pin_code = {0};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
      } else {
        ESP_LOGI(BT_AV_TAG, "Input pin code: 1234");
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '1';
        pin_code[1] = '2';
        pin_code[2] = '3';
        pin_code[3] = '4';
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
      }
      break;
  }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
  /* when Security Simple Pairing user confirmation requested, this event comes */
  case ESP_BT_GAP_CFM_REQ_EVT:
    ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    break;
  /* when Security Simple Pairing passkey notified, this event comes */
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %"PRIu32, param->key_notif.passkey);
    break;
  /* when Security Simple Pairing passkey requested, this event comes */
  case ESP_BT_GAP_KEY_REQ_EVT:
    ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
#endif

  /* when GAP mode changed, this event comes */
  case ESP_BT_GAP_MODE_CHG_EVT:
    ESP_LOGD(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
    break;

  /* other */
  default: {
    ESP_LOGD(BT_AV_TAG, "event: %d", event);
    break;
  }
  }

  return;
}

static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
  switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_METADATA_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT: {
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default: {
        ESP_LOGE(BT_AV_TAG, "Invalid AVRC event: %d", event);
        break;
    }
  }
}

static void bt_av_volume_changed(void)
{
  if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                          ESP_AVRC_RN_VOLUME_CHANGE)) {
    ESP_LOGI(BT_AV_TAG, "esp_avrc_ct_send_register_notification_cmd");
    esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, ESP_AVRC_RN_VOLUME_CHANGE, 0);
  }
}

void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
  switch (event_id) {
    /* when volume changed locally on target, this event comes */
    case ESP_AVRC_RN_VOLUME_CHANGE: {
      ESP_LOGI(BT_AV_TAG, "Volume changed: %d", event_parameter->volume);
      ESP_LOGI(BT_AV_TAG, "Set absolute volume: volume %d", event_parameter->volume + 5);
      esp_avrc_ct_send_set_absolute_volume_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, event_parameter->volume + 5);
      bt_av_volume_changed();
      break;
    }
    /* other */
    default:
      break;
  }
}

const char* media_ack_str(esp_a2d_media_ctrl_ack_t state) {
  const char *media_ack_text[APP_AV_STATE_MAX] = {"SUCCESS", "FAILURE", "BUSY"};
  
  if (state < ESP_A2D_MEDIA_CTRL_ACK_BUSY) {
    return media_ack_text[state];
  } else {
    return "Unknown";
  }
}

const char* media_cmd_str(esp_a2d_media_ctrl_t state) {
  const char *media_cmd_text[APP_AV_STATE_MAX] = {"NONE", "CHECK_SRC_READY", "START", "SUSPEND", "STOP"};
  
  if (state < ESP_A2D_MEDIA_CTRL_STOP) {
    return media_cmd_text[state];
  } else {
    return "Unknown";
  }
}

static void _print_av_sm(uint16_t event, void *param){
  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);

  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
      ESP_LOGI(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_EVT: %s", connection_str(a2d->conn_stat.state));
      break;

    case ESP_A2D_AUDIO_STATE_EVT:
      ESP_LOGI(BT_AV_TAG, "ESP_A2D_AUDIO_STATE_EVT: %s", audio_str(a2d->audio_stat.state));
      break;
    
    case ESP_A2D_PROF_STATE_EVT:
      ESP_LOGI(BT_AV_TAG, "ESP_A2D_PROF_STATE_EVT: %s", (a2d->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS)?
                                                        "INIT_SUCCESS":"DEINIT_SUCCESS");
      break;

    case ESP_A2D_MEDIA_CTRL_ACK_EVT: {
      ESP_LOGI(BT_AV_TAG, "ESP_A2D_MEDIA_CTRL_ACK_EVT: cmd: %s - status: %s",
                          media_cmd_str(a2d->media_ctrl_stat.cmd),
                          media_ack_str(a2d->media_ctrl_stat.status));
      }
      break;

    default:
      break;
  }
}

static void bt_app_av_sm_hdlr(uint16_t event, void *param)
{
  _print_av_sm(event, param);

    /* select handler according to different states */
  switch (s_a2d_state) {
    case APP_AV_STATE_IDLE:
      bt_app_av_state_idle_hdlr(event, param);
      break;
    case APP_AV_STATE_DISCOVERING:
    case APP_AV_STATE_DISCOVERED:
      break;
    case APP_AV_STATE_UNCONNECTED:
      bt_app_av_state_unconnected_hdlr(event, param);
      break;
    case APP_AV_STATE_CONNECTING:
      bt_app_av_state_connecting_hdlr(event, param);
      break;
    case APP_AV_STATE_CONNECTED:
      bt_app_av_state_connected_hdlr(event, param);
      break;
    case APP_AV_STATE_DISCONNECTING:
      bt_app_av_state_disconnecting_hdlr(event, param);
      break;
    default:
      ESP_LOGD(BT_AV_TAG, "Invalid state: %d", s_a2d_state);
      break;
  }
}

static void bt_app_av_state_idle_hdlr(uint16_t event, void *param)
{
  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);

  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
      switch(a2d->conn_stat.state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
          changeState(APP_AV_STATE_CONNECTING);
          break;
        
        default: break;
      }
      break;
    }

    default: break;
  }
}

static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);
    /* handle the events of interest in unconnected state */
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
      switch(a2d->conn_stat.state){
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
          changeState(APP_AV_STATE_CONNECTED);
          break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
          changeState(APP_AV_STATE_CONNECTING);
          break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
          break;
        default:
          break;
      }
      break;
    } 
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
      break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
      break;
    default: {
      break;
    }
    }
}

static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;

    /* handle the events of interest in connecting state */
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
          changeState(APP_AV_STATE_CONNECTED);
          s_media_state = APP_AV_MEDIA_STATE_IDLE;
          ESP_LOGI(BT_AV_TAG, "A2DP connected");
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
          ESP_LOGI(BT_AV_TAG, "A2DP disconnected");
          changeState(APP_AV_STATE_UNCONNECTED);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    default:
        ESP_LOGE(BT_AV_TAG, "Unhandled event: %d", event);
        break;
    }
}

static void bt_app_av_media_proc(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;

    switch (s_media_state) {
      case APP_AV_MEDIA_STATE_IDLE: {
        if (event == BT_APP_MEDIA_CHECK_SRC) {
          ESP_LOGI(BT_AV_TAG, "a2dp media ready checking ...");
          esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
          a2d = (esp_a2d_cb_param_t *)(param);
          if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                  a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "a2dp media ready, starting ...");
            s_media_state = APP_AV_MEDIA_STATE_STARTING;
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
          }
        }
        break;
      }
      case APP_AV_MEDIA_STATE_STARTING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
          a2d = (esp_a2d_cb_param_t *)(param);
          if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
                  a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "A2DP Media Started!");
            s_media_state = APP_AV_MEDIA_STATE_STARTED;
          } else {
            /* not started successfully, transfer to idle state */
            ESP_LOGE(BT_AV_TAG, "A2DP Media Failed to start!");
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
          }
        }
        break;
      }
      case APP_AV_MEDIA_STATE_STARTED: {
        break;
      }
      case APP_AV_MEDIA_STATE_STOPPING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
          a2d = (esp_a2d_cb_param_t *)(param);
          if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_STOP &&
                  a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "a2dp media stopped successfully, disconnecting...");
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
            changeState(APP_AV_STATE_DISCONNECTING);
            esp_a2d_source_disconnect(s_peer_bda);
          } else {
            ESP_LOGI(BT_AV_TAG, "a2dp media stopping...");
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
          }
        }
        break;
      }
      default: {
        break;
      }
    }
}

static void bt_app_av_state_connected_hdlr(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;

    /* handle the events of interest in connected state */
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(param);
      if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        ESP_LOGI(BT_AV_TAG, "A2DP disconnected reason: %s", (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL)? "Normal":"Abnormal");
        changeState(APP_AV_STATE_UNCONNECTED);
        BT_Disconnect();
      }
      break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(param);
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
      } else if (ESP_A2D_AUDIO_STATE_STOPPED == a2d->audio_stat.state) {
        ESP_LOGI(BT_AV_TAG, "A2DP Media Stopped!");
        s_media_state = APP_AV_MEDIA_STATE_IDLE;
      }
      break;
    }
    case ESP_A2D_AUDIO_CFG_EVT:
      // not supposed to occur for A2DP source
      break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        bt_app_av_media_proc(event, param);
        break;

    default: {
        ESP_LOGD(BT_AV_TAG, "Unhandled event: %d", event);
        break;
    }
    }
}

static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;

    /* handle the events of interest in disconnecing state */
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
          ESP_LOGI(BT_AV_TAG, "A2DP disconnected");
          changeState(APP_AV_STATE_UNCONNECTED);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    default: {
        ESP_LOGE(BT_AV_TAG, "Unhandled event: %d", event);
        break;
    }
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

#if 0
static void bt_app_a2d_heart_beat(TimerHandle_t arg)
{
  bt_app_work_dispatch(bt_app_av_sm_hdlr, BT_APP_HEART_BEAT_EVT, NULL, 0, NULL);
}
#endif

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
  ESP_LOGI(BT_AV_TAG, "evt %d", event);
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

  switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
      uint8_t *bda = rc->conn_stat.remote_bda;
      ESP_LOGI(BT_AV_TAG, "AVRC: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
                rc->conn_stat.connected? "connected":"disconnected",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

      if (rc->conn_stat.connected) {
        esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
      } else {
        s_avrc_peer_rn_cap.bits = 0;
      }
      break;
    }
    /* when passthrough responded, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
      ESP_LOGD(BT_AV_TAG, "AVRC passthrough response: key_code 0x%x, key_state %d, rsp_code %d", rc->psth_rsp.key_code,
                  rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
      break;
    }
    /* when metadata responded, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
      ESP_LOGD(BT_AV_TAG, "AVRC metadata response: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
      free(rc->meta_rsp.attr_text);
      break;
    }
    /* when notification changed, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
      ESP_LOGI(BT_AV_TAG, "AVRC event notification: %d", rc->change_ntf.event_id);
      bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
      break;
    }
    /* when indicate feature of remote device, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
      ESP_LOGI(BT_AV_TAG, "AVRC remote features %" PRIx32 ", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
      ESP_LOGI(BT_AV_TAG, "AVRC Force starting media...");
      s_media_state = APP_AV_MEDIA_STATE_IDLE;
      bt_app_av_media_proc(BT_APP_MEDIA_CHECK_SRC, NULL);
      break;
    }
    /* when get supported notification events capability of peer device, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
      ESP_LOGI(BT_AV_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                rc->get_rn_caps_rsp.evt_set.bits);
      s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;

      bt_av_volume_changed();
      break;
    }
    /* when set absolute volume responded, this event comes */
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT: {
      ESP_LOGI(BT_AV_TAG, "Set absolute volume response: volume %d", rc->set_volume_rsp.volume);
      break;
    }
    /* other */
    default: {
      ESP_LOGE(BT_AV_TAG, "Unhandled event: %d", event);
      break;
    }
  }
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "Event: %d", event);

  switch (event) {
    /* when stack up worked, this event comes */
    case BT_APP_STACK_UP_EVT: {
      // char *dev_name = LOCAL_DEVICE_NAME;
      esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME);
      esp_bt_gap_register_callback(bt_app_gap_cb);

      esp_avrc_ct_init();
      esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

      esp_avrc_rn_evt_cap_mask_t evt_set = {0};
      esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
      ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

      esp_a2d_source_init();
      esp_a2d_register_callback(&bt_app_a2d_cb);
      esp_a2d_source_register_data_callback(get_data_sinwave);

      /* Avoid the state error of s_a2d_state caused by the connection initiated by the peer device. */
      set_scan_mode_connectable(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

      ESP_LOGI(BT_AV_TAG, "Stacked up!");

#if 0
      /* create and start heart beat timer */
      do {
        int tmr_id = 0;
        s_tmr = xTimerCreate("connTmr", (10000 / portTICK_PERIOD_MS),
                              pdTRUE, (void *) &tmr_id, bt_app_a2d_heart_beat);
        xTimerStart(s_tmr, portMAX_DELAY);
      } while (0);
#endif
      break;
    }

    case BT_APP_REQUEST_SCAN: {
      if (s_a2d_state == APP_AV_STATE_IDLE || s_a2d_state == APP_AV_STATE_UNCONNECTED || s_a2d_state == APP_AV_STATE_DISCOVERED) {
        ESP_LOGI(BT_AV_TAG, "Scanning...");
        changeState(APP_AV_STATE_DISCOVERING);
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
      } else { ESP_LOGE(BT_AV_TAG, "Scanning ignored in state %s", state_str(s_a2d_state)); }

      break;
    }

    case BT_APP_REQUEST_CONNECT: {
      char str_addr[BDA_STR_LEN] = {0};
      st_bluetooth_client *client = (st_bluetooth_client *)p_param;

      memset(s_peer_bdname, 0, ESP_BT_GAP_MAX_BDNAME_LEN);
      strncpy((char*)s_peer_bdname, client->name, ESP_BT_GAP_MAX_BDNAME_LEN);
      memcpy(s_peer_bda, client->peer_bda, ESP_BD_ADDR_LEN);

      if (s_a2d_state == APP_AV_STATE_DISCOVERED || s_a2d_state == APP_AV_STATE_UNCONNECTED || s_a2d_state == APP_AV_STATE_IDLE) {
        if (validate_name(s_peer_bdname) && BT_ValidateMac(s_peer_bda)) {
          changeState(APP_AV_STATE_CONNECTING);
          esp_a2d_source_connect(s_peer_bda);
          ESP_LOGI(BT_AV_TAG, "Connecting to %s - %s", s_peer_bdname, bda_str(s_peer_bda));
        } else { ESP_LOGE(BT_AV_TAG, "Wrong peer! %s - %s", s_peer_bdname, bda2str(s_peer_bda, str_addr, BDA_STR_LEN)); }
      } else { ESP_LOGE(BT_AV_TAG, "Request (%d) ignored in state %s", event, state_str(s_a2d_state)); }

      break;
    }

    case BT_APP_REQUEST_DISCONNECT: {
      if (s_media_state == APP_AV_MEDIA_STATE_STARTED) {
        ESP_LOGI(BT_AV_TAG, "Disconnect requested. Stop media first...");
        s_media_state = APP_AV_MEDIA_STATE_STOPPING;
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
      } else {
        ESP_LOGI(BT_AV_TAG, "Disconnect requested. Disconnecting...");
        if (s_a2d_state == APP_AV_STATE_CONNECTED) {
          changeState(APP_AV_STATE_DISCONNECTING);
        } else {
          changeState(APP_AV_STATE_UNCONNECTED);
        }
        esp_a2d_source_disconnect(s_peer_bda);
      }
      break;
    }

    case BT_APP_REQUEST_AUTO_CONNECT: {
      ESP_LOGI(BT_AV_TAG, "Enabling Auto Connect...");
      set_scan_mode_connectable(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      break;
    }

    case BT_APP_REQUEST_NON_AUTO_CONNECT: {
      ESP_LOGI(BT_AV_TAG, "Disabling Auto Connect...");
      set_scan_mode_connectable(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
      break;
    }

    /* other */
    default: {
      ESP_LOGD(BT_AV_TAG, "Unhandled event: %d", event);
      break;
    }
  }
}

void BT_Init()
{
  esp_err_t ret = ESP_FAIL;

  ESP_LOGI(BT_AV_TAG, "Bluetooth initializing...");

  if ((ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE)) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "BLE failed to release: %s", esp_err_to_name(ret));
  }

  if (!btStart()) {
    ESP_LOGE(BT_AV_TAG,"Failed to initialize controller");
    return;
  }

  if ((ret = esp_bluedroid_init()) != ESP_OK) {
      ESP_LOGE(BT_AV_TAG, "Initialize bluedroid failed: %s", esp_err_to_name(ret));
      return;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
      ESP_LOGE(BT_AV_TAG, "Enable bluedroid failed");
      return;
  }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
  /* set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

  /*
    * Set default parameters for Legacy Pairing
    * Use variable pin, input pin code when pairing
    */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  esp_bt_pin_code_t pin_code;
  esp_bt_gap_set_pin(pin_type, 0, pin_code);

  bt_app_task_start_up();
  /* Bluetooth device name, connection mode and profile set up */
  bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_STACK_UP_EVT, NULL, 0, NULL);

  ESP_LOGI(BT_AV_TAG, "Bluetooth init done!");
}

void BT_End() {}

#endif

#ifdef __cplusplus
}
#endif