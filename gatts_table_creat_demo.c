#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define I2C_MASTER_SCL_IO           19      
#define I2C_MASTER_SDA_IO           21      
#define I2C_MASTER_FREQ_HZ          100000

#define TCS34725_ADDR               0x29    
#define TCS34725_COMMAND_BIT        0x80
#define TCS34725_CMD_AUTO_INC       0x20
#define TCS34725_ENABLE             0x00
#define TCS34725_ENABLE_PON         0x01
#define TCS34725_ENABLE_AEN         0x02
#define TCS34725_ATIME              0x01
#define TCS34725_WTIME              0x03
#define TCS34725_CONTROL            0x0F
#define TCS34725_ID                 0x12
#define TCS34725_STATUS             0x13
#define TCS34725_CDATAL             0x14

#define TCS34725_STATUS_AVALID      0x01
#define TCS34725_GAIN_4X            0x01
#define TCS34725_DEFAULT_ATIME      0xEB
#define TCS34725_DEFAULT_WTIME      0xFF
#define TCS34725_CLEAR_MIN          80

typedef enum {
    UNKNOWN = 0,
    RED, GREEN,
    BLUE,
} color_state_t;

color_state_t color_state = UNKNOWN;

i2c_master_dev_handle_t color_sensor_handle; 
static uint8_t color_sensor_ready = 0;
static uint8_t fan_vibration_on = 1;

typedef enum {
    DRIVE_NONE = 0,
    DRIVE_FORWARD,
    DRIVE_LEFT,
    DRIVE_RIGHT,
} drive_action_t;

typedef enum {
    COLOR_OVERRIDE_NONE = 0,
    COLOR_OVERRIDE_RED,
    COLOR_OVERRIDE_GREEN_PAUSE,
    COLOR_OVERRIDE_GREEN_TURN,
} color_override_t;

static drive_action_t last_ble_drive = DRIVE_NONE;
static color_override_t color_override = COLOR_OVERRIDE_NONE;
static uint8_t fan_vibration_saved = 1;
static int64_t green_pause_start_ms = 0;

typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} tcs34725_rgbc_t;

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "my_esp32"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static void turn_right_action(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 665);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void turn_left_action(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 563);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void drive_forward_action(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 614);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void set_fan_vibration(uint8_t on)
{
    fan_vibration_on = on ? 1 : 0;
    gpio_set_level(GPIO_NUM_23, fan_vibration_on);
}

static void apply_drive_action(drive_action_t action)
{
    switch (action) {
        case DRIVE_FORWARD:
            drive_forward_action();
            break;
        case DRIVE_LEFT:
            turn_left_action();
            break;
        case DRIVE_RIGHT:
            turn_right_action();
            break;
        default:
            break;
    }
}

static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, 
    .max_interval        = 0x0010, 
    .appearance          = 0x00,
    .manufacturer_len    = 0,    
    .p_manufacturer_data = NULL, 
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL, 
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       
    },
};

static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},[IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d", param->update_conn_params.status);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_OFFSET;
    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }
    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_ble_gap_config_adv_data(&scan_rsp_data);
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, value :");
                ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);
                if(param->write.value[0]=='s'){
                    gpio_set_level(GPIO_NUM_22, 0);
                    set_fan_vibration(0);
                    last_ble_drive = DRIVE_NONE;
                } else if(param->write.value[0]=='e'){
                    gpio_set_level(GPIO_NUM_22, 0);
                    set_fan_vibration(1);
                    last_ble_drive = DRIVE_NONE;
                } else if(param->write.value[0]=='f'){
                    gpio_set_level(GPIO_NUM_22, 1);
                    drive_forward_action();
                    last_ble_drive = DRIVE_FORWARD;
                } else if(param->write.value[0]=='l'){
                    gpio_set_level(GPIO_NUM_22, 1);
                    turn_left_action();
                    last_ble_drive = DRIVE_LEFT;
                } else if(param->write.value[0]=='r'){
                    gpio_set_level(GPIO_NUM_22, 1);
                    turn_right_action();
                    last_ble_drive = DRIVE_RIGHT;
                } 

                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT");
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    
            conn_params.min_int = 0x10;    
            conn_params.timeout = 400;    
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status == ESP_GATT_OK){
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            }
            break;
        }
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void pwm_config(void){
    ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_TIMER_13_BIT,
    .freq_hz          = 50,
    .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    {ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0, 
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 4,
        .duty           = 614, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));}
} 

static esp_err_t i2c_master_init(void) {
    //初始化I2C主机总线
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS34725_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &color_sensor_handle);
}

static esp_err_t tcs34725_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {TCS34725_COMMAND_BIT | reg, value};
    return i2c_master_transmit(color_sensor_handle, data, sizeof(data), -1);
}

static esp_err_t tcs34725_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    return i2c_master_transmit_receive(color_sensor_handle, &cmd, 1, value, 1, -1);
}

static esp_err_t tcs34725_read_rgbc(tcs34725_rgbc_t *rgbc)
{
    uint8_t cmd = TCS34725_COMMAND_BIT | TCS34725_CMD_AUTO_INC | TCS34725_CDATAL;
    uint8_t data[8] = {0};
    esp_err_t err = i2c_master_transmit_receive(color_sensor_handle, &cmd, 1, data, sizeof(data), -1);
    if (err != ESP_OK) {
        return err;
    }

    rgbc->clear = (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
    rgbc->red = (uint16_t)(((uint16_t)data[3] << 8) | data[2]);
    rgbc->green = (uint16_t)(((uint16_t)data[5] << 8) | data[4]);
    rgbc->blue = (uint16_t)(((uint16_t)data[7] << 8) | data[6]);

    return ESP_OK;
}

static color_state_t classify_color(const tcs34725_rgbc_t *rgbc)
{
    if (rgbc->clear < TCS34725_CLEAR_MIN) {
        return UNKNOWN;
    }

    uint32_t r_pct = ((uint32_t)rgbc->red * 100U) / rgbc->clear;
    uint32_t g_pct = ((uint32_t)rgbc->green * 100U) / rgbc->clear;
    uint32_t b_pct = ((uint32_t)rgbc->blue * 100U) / rgbc->clear;

    if ((r_pct > g_pct + 8U) && (r_pct > b_pct + 8U) && (r_pct >= 38U)) {
        return RED;
    }
    if ((g_pct > r_pct + 8U) && (g_pct > b_pct + 8U) && (g_pct >= 34U)) {
        return GREEN;
    }
    if ((b_pct > r_pct + 8U) && (b_pct > g_pct + 8U) && (b_pct >= 30U)) {
        return BLUE;
    }

    return UNKNOWN;
}

static esp_err_t color_sensor_init(void)
{
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(tcs34725_read_reg(TCS34725_ID, &chip_id), GATTS_TABLE_TAG, "read sensor id failed");
    if ((chip_id != 0x44) && (chip_id != 0x4D)) {
        ESP_LOGE(GATTS_TABLE_TAG, "unexpected TCS34725 chip id: 0x%02X", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(tcs34725_write_reg(TCS34725_ATIME, TCS34725_DEFAULT_ATIME), GATTS_TABLE_TAG, "set ATIME failed");
    ESP_RETURN_ON_ERROR(tcs34725_write_reg(TCS34725_WTIME, TCS34725_DEFAULT_WTIME), GATTS_TABLE_TAG, "set WTIME failed");
    ESP_RETURN_ON_ERROR(tcs34725_write_reg(TCS34725_CONTROL, TCS34725_GAIN_4X), GATTS_TABLE_TAG, "set gain failed");
    ESP_RETURN_ON_ERROR(tcs34725_write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON), GATTS_TABLE_TAG, "enable PON failed");

    vTaskDelay(pdMS_TO_TICKS(3));

    ESP_RETURN_ON_ERROR(tcs34725_write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN), GATTS_TABLE_TAG, "enable AEN failed");
    vTaskDelay(pdMS_TO_TICKS(60));

    ESP_LOGI(GATTS_TABLE_TAG, "TCS34725 ready, chip id: 0x%02X", chip_id);
    return ESP_OK;
}

static color_state_t updateColor(void)
{
    uint8_t status = 0;
    tcs34725_rgbc_t rgbc = {0};

    if (tcs34725_read_reg(TCS34725_STATUS, &status) != ESP_OK) {
        return UNKNOWN;
    }

    if ((status & TCS34725_STATUS_AVALID) == 0) {
        return UNKNOWN;
    }

    if (tcs34725_read_rgbc(&rgbc) != ESP_OK) {
        return UNKNOWN;
    }

    ESP_LOGD(GATTS_TABLE_TAG, "C:%u R:%u G:%u B:%u", rgbc.clear, rgbc.red, rgbc.green, rgbc.blue);
    return classify_color(&rgbc);
}

static esp_err_t try_init_color_sensor(void)
{
    esp_err_t ret;

    if (color_sensor_handle == NULL) {
        ret = i2c_master_init();
        if (ret != ESP_OK) {
            ESP_LOGE(GATTS_TABLE_TAG, "i2c_master_init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ret = color_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "color_sensor_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    color_sensor_ready = 1;
    ESP_LOGI(GATTS_TABLE_TAG, "Color sensor init success");
    return ESP_OK;
}
// =====================================================================

void app_main(void)
{
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << GPIO_NUM_22;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
    }
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << GPIO_NUM_23;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
    }
    gpio_set_level(GPIO_NUM_22, 0);
    set_fan_vibration(1);

    pwm_config();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

    // 传感器初始化失败不影响 BLE，后台重试即可。
    if (try_init_color_sensor() != ESP_OK) {
        ESP_LOGW(GATTS_TABLE_TAG, "TCS34725 not ready, BLE stays available and sensor will retry");
    }

    uint8_t retry_counter = 0;
    ESP_LOGI(GATTS_TABLE_TAG, "Started BLE + Sensing Task...");

    //扫描颜色状态
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500)); 

        if (!color_sensor_ready) {
            retry_counter++;
            if (retry_counter >= 6) {
                retry_counter = 0;
                try_init_color_sensor();
            }
            continue;
        }
        
        color_state = updateColor();
        switch(color_state) {
            case RED:
                if (color_override == COLOR_OVERRIDE_GREEN_PAUSE && fan_vibration_saved) {
                    set_fan_vibration(1);
                }
                color_override = COLOR_OVERRIDE_RED;
                turn_right_action();
                ESP_LOGI(GATTS_TABLE_TAG, "Color: RED");
                break;
            case GREEN:
                if (color_override != COLOR_OVERRIDE_GREEN_PAUSE && color_override != COLOR_OVERRIDE_GREEN_TURN) {
                    color_override = COLOR_OVERRIDE_GREEN_PAUSE;
                    fan_vibration_saved = fan_vibration_on;
                    set_fan_vibration(0);
                    green_pause_start_ms = esp_timer_get_time() / 1000;
                }

                if (color_override == COLOR_OVERRIDE_GREEN_PAUSE) {
                    int64_t now_ms = esp_timer_get_time() / 1000;
                    if ((now_ms - green_pause_start_ms) >= 2000) {
                        if (fan_vibration_saved) {
                            set_fan_vibration(1);
                        }
                        color_override = COLOR_OVERRIDE_GREEN_TURN;
                    } else {
                        break;
                    }
                }

                if (color_override == COLOR_OVERRIDE_GREEN_TURN) {
                    turn_right_action();
                }

                ESP_LOGI(GATTS_TABLE_TAG, "Color: GREEN");
                break;
            case BLUE:
                ESP_LOGI(GATTS_TABLE_TAG, "Color: BLUE");
                break;
            default:
                if (color_override != COLOR_OVERRIDE_NONE) {
                    if (color_override == COLOR_OVERRIDE_GREEN_PAUSE && fan_vibration_saved) {
                        set_fan_vibration(1);
                    }
                    color_override = COLOR_OVERRIDE_NONE;
                    apply_drive_action(last_ble_drive);
                }
                break;
        }
    }
}