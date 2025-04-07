#include "tesla_ble_vehicle.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h" 
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/scheduler.h" // Added for App.scheduler

// Include necessary component headers for logging macros and types
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h" // Needed for switch_::log_switch
#include "esphome/components/lock/lock.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

#include <vector>
#include <string>
#include <algorithm> // for std::min
#include <cstring> // Needed for memcpy

// Library includes
#include <client.h>
#include <signatures.pb.h>
#include <universal_message.pb.h>
#include <peer.h>

namespace esphome {
namespace tesla_ble_vehicle {

static const char *const TAG = "tesla_ble_vehicle";

using namespace esphome::esp32_ble_client;
using namespace esphome::esp32_ble_tracker;

// Helper function to replace esphome::esp32_ble_tracker::bda_to_string
std::string bda_to_string_helper(const esp_bd_addr_t bda) {
    char str[18];
    snprintf(str, sizeof(str), "%02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return std::string(str);
}

TeslaBLEVehicle::TeslaBLEVehicle() {
    // Initialize UUIDs directly using esp_bt_uuid_t
    this->service_uuid_.len = ESP_UUID_LEN_128;
    uint8_t service_uuid_128[16] = {0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, 0xf0, 0x43, 0xd1, 0xb2, 0x11, 0x02, 0x00, 0x00};
    memcpy(this->service_uuid_.uuid.uuid128, service_uuid_128, 16);

    this->read_uuid_.len = ESP_UUID_LEN_128;
    uint8_t read_uuid_128[16] =    {0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, 0xf0, 0x43, 0xd1, 0xb2, 0x12, 0x02, 0x00, 0x00};
    memcpy(this->read_uuid_.uuid.uuid128, read_uuid_128, 16);

    this->write_uuid_.len = ESP_UUID_LEN_128;
    uint8_t write_uuid_128[16] =   {0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, 0xf0, 0x43, 0xd1, 0xb2, 0x13, 0x02, 0x00, 0x00};
    memcpy(this->write_uuid_.uuid.uuid128, write_uuid_128, 16);

    // Logging requires a helper to convert esp_bt_uuid_t to string, or use ESPHome ESPBTUUID for logging only
    ESP_LOGD(TAG, "Service UUID initialized (raw)");
    ESP_LOGD(TAG, "Read UUID initialized (raw)");
    ESP_LOGD(TAG, "Write UUID initialized (raw)");

    this->last_rx_time_ = 0;
    this->gattc_if = 0;
    this->conn_id = 0;
}

float TeslaBLEVehicle::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH; }

void TeslaBLEVehicle::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Tesla BLE Vehicle '%s'...", this->vin_.c_str());

    // TODO: Implement Base64 decoding for private key without tb_utils.h
    // For now, assume private_key_ is filled directly if needed, or handle error
    /*
    if (!tb_utils::decode_base64(this->private_key_b64_, this->private_key_) || this->private_key_.empty()) {
        ESP_LOGE(TAG, "Failed to decode private key (DECODING DISABLED)!");
        this->mark_failed();
        return;
    }
    */
    if (this->private_key_b64_.empty()) { // Check if base64 string is provided
         ESP_LOGE(TAG, "Private key (Base64) is not configured!");
         this->mark_failed();
         return;
    } else {
        ESP_LOGW(TAG, "Private key decoding not implemented yet! Assuming raw key needed if decoding fails.");
        // Placeholder: maybe the library can handle base64?
        // Or we need mbedtls_base64_decode or similar here.
        // Forcing a failure for now as decoding is missing.
        ESP_LOGE(TAG, "Marking as failed because private key decoding is missing.");
        this->mark_failed();
        return; 
    }

    this->client_ = std::make_unique<TeslaBLE::Client>();

    // This likely expects the RAW key, not base64
    int load_status = this->client_->loadPrivateKey(this->private_key_.data(), this->private_key_.size()); 
    if (load_status != 0) {
        ESP_LOGE(TAG, "Failed to load private key into TeslaBLE::Client! Error code: %d (Key size: %zu)", load_status, this->private_key_.size());
        this->mark_failed();
        return;
    }
    ESP_LOGD(TAG, "Private key loaded successfully.");

    this->client_->setVIN(this->vin_.c_str());
    ESP_LOGD(TAG, "VIN set in TeslaBLE::Client.");

    auto *ble_tracker = esphome::esp32_ble_tracker::global_esp32_ble_tracker;
    if (!ble_tracker) {
        ESP_LOGE(TAG, "BLE Tracker not found!");
        this->mark_failed();
        return;
    }
    ESP_LOGD(TAG, "BLE Tracker found.");

    // --- Setup Callbacks --- 
    if (this->wake_up_button_) this->wake_up_button_->add_on_press_callback([this]() {
        ESP_LOGD(TAG, "Wake button pressed - enqueueing command");
        this->queue_command(UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY, "wake_up");
    });
    if (this->honk_button_) this->honk_button_->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Honk button pressed - enqueueing command");
         this->queue_command(UniversalMessage_Domain_DOMAIN_INFOTAINMENT, "honk_horn");
    });
    
    if (this->climate_switch_) {
        this->climate_switch_->add_on_state_callback([this](bool state) {
            ESP_LOGD(TAG, "Climate switch -> %s - enqueueing command", state ? "ON" : "OFF");
            this->queue_command(UniversalMessage_Domain_DOMAIN_INFOTAINMENT, state ? "climate_on" : "climate_off");
        });
    }

    ESP_LOGCONFIG(TAG, "Tesla BLE Vehicle setup complete.");
    this->scan_ = true;
    ESP_LOGD(TAG, "Initial scan enabled.");
}

void TeslaBLEVehicle::loop() {
    this->process_ble_read_queue();
    this->process_response_queue();
    this->process_command_queue();
    this->process_ble_write_queue();
}

void TeslaBLEVehicle::dump_config() {
    ESP_LOGCONFIG(TAG, "Tesla BLE Vehicle:");
    ESP_LOGCONFIG(TAG, "  VIN: %s", this->vin_.c_str());
    ESP_LOGCONFIG(TAG, "  Private Key: %s", this->private_key_b64_.empty() ? "(Not Set)" : "(Set)");
#ifdef USE_WEB_SERVER
    ESP_LOGCONFIG(TAG, "  HTTP Proxy Enabled: %s", YESNO(this->http_proxy_enabled_));
    ESP_LOGCONFIG(TAG, "  HTTP Proxy Port: %u", this->http_proxy_port_);
#endif

    LOG_LOCK("  ", "Door Lock", this->door_lock_);
    LOG_BUTTON("  ", "Honk Button", this->honk_button_);
    esphome::switch_::log_switch(TAG, "  ", "Climate Switch", this->climate_switch_); 
}

bool TeslaBLEVehicle::parse_device(const esphome::esp32_ble_tracker::ESPBTDevice &device) {
    ESP_LOGV(TAG, "parse_device: %s RSSI=%d", device.address_str().c_str(), device.get_rssi());

    bool service_found = false;
    for (auto &uuid : device.get_service_uuids()) {
        // Compare espbt::ESPBTUUID with our esp_bt_uuid_t
        // Convert our UUID to ESPBTUUID for comparison or implement direct compare
        // Using ESPBTUUID for comparison for simplicity
        if (uuid == esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->service_uuid_)) {
            ESP_LOGI(TAG, "Found matching service UUID: %s", uuid.to_string().c_str());
            service_found = true;
            break;
        }
    }

    if (!service_found) {
        return false;
    }

    ESP_LOGI(TAG, "Found Tesla vehicle candidate: %s", device.address_str().c_str());
    return true; 
}

void TeslaBLEVehicle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    ESP_LOGV(TAG, "TeslaBLEVehicle::gattc_event_handler - Event: %s (%d), IF: %d (node_if: %d)", 
             gattc_event_to_string(event), event, gattc_if, this->gattc_if);

    if (this->client_ == nullptr && event != ESP_GATTC_REG_EVT) { 
        ESP_LOGV(TAG, "Ignoring event %d because client_ is null", event);
        return; 
    }
    
    if (this->gattc_if != 0 && gattc_if != ESP_GATT_IF_NONE && gattc_if != this->gattc_if) {
        ESP_LOGV(TAG, "Ignoring event for different gattc_if %d (expected %d)", gattc_if, this->gattc_if);
        return;
    }

    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGD(TAG, "ESP_GATTC_REG_EVT: status=%s (%d), app_id=%d", esp_err_to_name(param->reg.status), param->reg.status, param->reg.app_id);
            if (param->reg.status == ESP_GATT_OK) {
                this->gattc_if = gattc_if; 
                ESP_LOGI(TAG, "GATT client registered successfully (if=%d)", this->gattc_if);
            } else {
                ESP_LOGE(TAG, "GATT client registration failed");
                this->node_state = espbt::ClientState::IDLE; 
                this->mark_failed(); 
            }
            break;

        case ESP_GATTC_OPEN_EVT:
            ESP_LOGD(TAG, "ESP_GATTC_OPEN_EVT: conn_id=%d, status=%s (%d), mtu=%d, addr=%s", 
                 param->open.conn_id, esp_err_to_name(param->open.status), param->open.status, 
                 param->open.mtu, bda_to_string_helper(param->open.remote_bda).c_str());
                 
            if (memcmp(param->open.remote_bda, this->parent_->get_remote_bda(), ESP_BD_ADDR_LEN) != 0) { 
                 ESP_LOGW(TAG, "Connected to unexpected device? %s (expected %s)", 
                     bda_to_string_helper(param->open.remote_bda).c_str(),
                     this->parent_->address_str().c_str());
                 esp_ble_gattc_close(this->gattc_if, param->open.conn_id);
                 break;
             }

            if (param->open.status == ESP_GATT_OK) {
                this->conn_id = param->open.conn_id;
                this->node_state = espbt::ClientState::CONNECTING; 
                ESP_LOGI(TAG, "Connected successfully (conn_id=%d), starting service discovery...", this->conn_id);
                esp_err_t search_status = esp_ble_gattc_search_service(this->gattc_if, this->conn_id, &this->service_uuid_);
                if (search_status != ESP_OK) {
                     ESP_LOGE(TAG, "esp_ble_gattc_search_service failed: %s", esp_err_to_name(search_status));
                     this->parent_->disconnect(); 
                 }
            } else {
                ESP_LOGW(TAG, "Connection failed, status=%s (%d)", esp_err_to_name(param->open.status), param->open.status);
                this->node_state = espbt::ClientState::IDLE;
                this->scan_ = true; 
            }
            break;
        
        case ESP_GATTC_SEARCH_RES_EVT:
             ESP_LOGV(TAG, "ESP_GATTC_SEARCH_RES_EVT: conn_id=%d", param->search_res.conn_id);
             if (param->search_res.conn_id == this->conn_id) {
                 // Convert found esp_bt_uuid_t to ESPBTUUID for comparison
                 esphome::esp32_ble_tracker::ESPBTUUID found_uuid = esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(param->search_res.srvc_id.uuid);
                 // Compare with our service UUID (also converted)
                 if (found_uuid == esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->service_uuid_)) {
                     ESP_LOGD(TAG, "  Found matching service UUID: %s (start_handle=0x%x, end_handle=0x%x)", 
                              found_uuid.to_string().c_str(), param->search_res.start_handle, param->search_res.end_handle);
                 } else {
                     ESP_LOGV(TAG, "  Found other service UUID: %s", found_uuid.to_string().c_str());
                 }
             }
            break;

        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGD(TAG, "ESP_GATTC_SEARCH_CMPL_EVT: conn_id=%d, status=%s (%d)", 
                 param->search_cmpl.conn_id, esp_err_to_name(param->search_cmpl.status), param->search_cmpl.status);
            if (param->search_cmpl.conn_id == this->conn_id && param->search_cmpl.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Service search completed. Discovering characteristics...");
                auto* read_char = this->parent_->get_characteristic(esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->service_uuid_), 
                                                                  esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->read_uuid_));
                auto* write_char = this->parent_->get_characteristic(esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->service_uuid_),
                                                                   esphome::esp32_ble_tracker::ESPBTUUID::from_uuid(this->write_uuid_));

                if (read_char && write_char) {
                    this->read_handle_ = read_char->handle;
                    this->write_handle_ = write_char->handle;
                    ESP_LOGI(TAG, "Found Read handle: 0x%x, Write handle: 0x%x", this->read_handle_, this->write_handle_);
                    
                    esp_err_t notify_status = esp_ble_gattc_register_for_notify(this->gattc_if, this->parent_->get_remote_bda(), this->read_handle_);
                     if (notify_status == ESP_OK) {
                         ESP_LOGI(TAG, "Successfully requested registration for notifications on handle 0x%x", this->read_handle_);
                     } else {
                         ESP_LOGE(TAG, "esp_ble_gattc_register_for_notify failed: %s", esp_err_to_name(notify_status));
                         this->parent_->disconnect(); 
                     }
                } else {
                     ESP_LOGE(TAG, "Could not find required characteristics after service search!");
                     // Logging requires converting esp_bt_uuid_t to string
                     // if (!read_char) ESP_LOGE(TAG, "  Read characteristic %s not found", /* Log read_uuid_ */);
                     // if (!write_char) ESP_LOGE(TAG, "  Write characteristic %s not found", /* Log write_uuid_ */ );
                     this->parent_->disconnect(); 
                 }
            } else {
                ESP_LOGE(TAG, "Service search failed or for wrong connection: status=%s (%d)", 
                     esp_err_to_name(param->search_cmpl.status), param->search_cmpl.status);
                this->parent_->disconnect();
            }
            break;

        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            ESP_LOGD(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT: status=%s (%d), handle=0x%x", 
                 esp_err_to_name(param->reg_for_notify.status), param->reg_for_notify.status, param->reg_for_notify.handle);
             if (param->reg_for_notify.status == ESP_GATT_OK && param->reg_for_notify.handle == this->read_handle_) {
                 ESP_LOGI(TAG, "Notification registration confirmed. Connection fully established.");
                 this->node_state = espbt::ClientState::CONNECTED; 
                 this->setSensors(true); 
                 ESP_LOGI(TAG, "Queueing initial VCSEC information request.");
                 this->enqueueVCSECInformationRequest(true); 
             } else {
                 ESP_LOGE(TAG, "Failed to register for notifications: status=%s (%d), handle=0x%x (expected 0x%x)", 
                          esp_err_to_name(param->reg_for_notify.status), param->reg_for_notify.status, 
                          param->reg_for_notify.handle, this->read_handle_);
                 this->parent_->disconnect(); 
             }
            break;

        case ESP_GATTC_NOTIFY_EVT:
             if (param->notify.conn_id == this->conn_id && param->notify.handle == this->read_handle_) {
                 ESP_LOGV(TAG, "RX Data (handle 0x%x, len %d): %s", 
                     param->notify.handle, param->notify.value_len, 
                     format_hex_pretty(param->notify.value, param->notify.value_len).c_str());
                 std::vector<unsigned char> data_chunk(param->notify.value, param->notify.value + param->notify.value_len);
                 this->ble_read_queue_.emplace(data_chunk);
                 App.feed_wdt(); 
             } else {
                 ESP_LOGV(TAG, "Ignored notify for handle 0x%x or conn_id %d", param->notify.handle, param->notify.conn_id);
             }
            break;

        case ESP_GATTC_WRITE_CHAR_EVT:
             if (param->write.conn_id == this->conn_id && param->write.handle == this->write_handle_) {
                 ESP_LOGV(TAG, "ESP_GATTC_WRITE_CHAR_EVT: status=%s (%d), handle=0x%x", 
                      esp_err_to_name(param->write.status), param->write.status, param->write.handle);
                 if (param->write.status == ESP_GATT_OK) {
                     ESP_LOGD(TAG, "Write chunk successful.");
                     if (!this->ble_write_queue_.empty()) {
                         this->ble_write_queue_.pop(); 
                         if (!this->ble_write_queue_.empty()) {
                             BLETXChunk& next_chunk = this->ble_write_queue_.front();
                             ESP_LOGV(TAG, "Writing next chunk (%zu bytes)", next_chunk.data.size());
                             esp_err_t write_status = esp_ble_gattc_write_char(
                                 this->gattc_if,
                                 this->conn_id,
                                 this->write_handle_,
                                 next_chunk.data.size(),
                                 next_chunk.data.data(),
                                 next_chunk.write_type,
                                 next_chunk.auth_req
                             );
                             if (write_status != ESP_OK) {
                                 ESP_LOGE(TAG, "writeBLE: esp_ble_gattc_write_char failed for next chunk: %s", esp_err_to_name(write_status));
                                 while (!this->ble_write_queue_.empty()) this->ble_write_queue_.pop();
                             } else {
                                 ESP_LOGV(TAG, "writeBLE: Next chunk write initiated successfully.");
                             }
                         } else {
                             ESP_LOGD(TAG, "Finished writing message sequence.");
                             if (!this->command_queue_.empty()) {
                                 BLECommand& current_command = this->command_queue_.front();
                                 if (current_command.state == BLECommandState::SENDING) {
                                     ESP_LOGI(TAG, "Command '%s' sent, now WAITING_FOR_RESPONSE", current_command.execute_name.c_str());
                                     current_command.state = BLECommandState::WAITING_FOR_RESPONSE;
                                     current_command.started_at = millis();
                                 } else {
                                     ESP_LOGW(TAG, "Write sequence finished, but front command state was %d (expected SENDING)", static_cast<int>(current_command.state));
                                 }
                             }
                         }
                     } else {
                         ESP_LOGW(TAG, "Write successful but write queue was empty?");
                     }
                 } else {
                     ESP_LOGE(TAG, "Write chunk failed, status=%s (%d)", esp_err_to_name(param->write.status), param->write.status);
                     while (!this->ble_write_queue_.empty()) this->ble_write_queue_.pop();
                 }
             } else {
                  ESP_LOGV(TAG, "Ignored write event for handle 0x%x or conn_id %d", param->write.handle, param->write.conn_id);
             }
            break;

        case ESP_GATTC_DISCONNECT_EVT:
        case ESP_GATTC_CLOSE_EVT:     
             {
                 uint16_t conn_id_event = (event == ESP_GATTC_DISCONNECT_EVT ? param->disconnect.conn_id : param->close.conn_id);
                 uint16_t reason_or_status = (event == ESP_GATTC_DISCONNECT_EVT ? param->disconnect.reason : param->close.status); 
                 const char* event_name = (event == ESP_GATTC_DISCONNECT_EVT ? "DISCONNECT" : "CLOSE");
                 const char* remote_bda_str = (event == ESP_GATTC_DISCONNECT_EVT ? bda_to_string_helper(param->disconnect.remote_bda).c_str() : bda_to_string_helper(param->close.remote_bda).c_str());

                 ESP_LOGD(TAG, "ESP_GATTC_%s_EVT: conn_id=%d, status/reason=0x%x (%s), addr=%s", 
                     event_name, conn_id_event, reason_or_status,
                     (event == ESP_GATTC_CLOSE_EVT ? esp_err_to_name(reason_or_status) : "N/A"),
                     remote_bda_str);
                 
                  if (conn_id_event == this->conn_id) {
                     ESP_LOGW(TAG, "Connection closed/disconnected (conn_id=%d).", this->conn_id);
                     this->conn_id = 0; 
                     this->read_handle_ = 0;
                     this->write_handle_ = 0;
                     this->gattc_if = 0; 
                     this->node_state = espbt::ClientState::IDLE; 
                     this->setSensors(false); 
                     while (!this->ble_read_queue_.empty()) this->ble_read_queue_.pop();
                     while (!this->response_queue_.empty()) this->response_queue_.pop();
                     while (!this->ble_write_queue_.empty()) this->ble_write_queue_.pop();
                     while (!this->command_queue_.empty()) this->command_queue_.pop();
                     ESP_LOGI(TAG, "Cleared all queues due to disconnect.");
                     ESP_LOGI(TAG, "Scheduling rescan in 5s...");
                     App.scheduler.set_timeout(this, "rescan", 5000, [this](){ 
                        this->scan_ = true; 
                     }); 
                  } else if (conn_id_event != 0) {
                       ESP_LOGD(TAG, "Ignoring %s event for inactive conn_id %d (current active: %d)", event_name, conn_id_event, this->conn_id);
                  }
             }
            break;
        
        default:
            ESP_LOGV(TAG, "Unhandled GATTC Event: %s (%d)", gattc_event_to_string(event), event);
            break;
    }
}

void TeslaBLEVehicle::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    BLEClientNode::gap_event_handler(event, param);

     ESP_LOGV(TAG, "TeslaBLEVehicle::gap_event_handler - Event: %d", event);
    if (event == ESP_GAP_BLE_AUTH_CMPL_EVT) {
        ESP_LOGD(TAG, "GAP Authentication Complete: status=%d, addr=%s",
                  param->ble_security.auth_cmpl.status, bda_to_string_helper(param->ble_security.auth_cmpl.bd_addr).c_str());
    }
}


const char* TeslaBLEVehicle::gattc_event_to_string(esp_gattc_cb_event_t event) {
    switch (event) {
        case ESP_GATTC_REG_EVT: return "REG_EVT";
        case ESP_GATTC_UNREG_EVT: return "UNREG_EVT";
        case ESP_GATTC_OPEN_EVT: return "OPEN_EVT";
        case ESP_GATTC_READ_CHAR_EVT: return "READ_CHAR_EVT";
        case ESP_GATTC_WRITE_CHAR_EVT: return "WRITE_CHAR_EVT";
        case ESP_GATTC_CLOSE_EVT: return "CLOSE_EVT";
        case ESP_GATTC_SEARCH_CMPL_EVT: return "SEARCH_CMPL_EVT";
        case ESP_GATTC_SEARCH_RES_EVT: return "SEARCH_RES_EVT";
        case ESP_GATTC_READ_DESCR_EVT: return "READ_DESCR_EVT";
        case ESP_GATTC_WRITE_DESCR_EVT: return "WRITE_DESCR_EVT";
        case ESP_GATTC_NOTIFY_EVT: return "NOTIFY_EVT";
        case ESP_GATTC_PREP_WRITE_EVT: return "PREP_WRITE_EVT";
        case ESP_GATTC_EXEC_EVT: return "EXEC_EVT";
        case ESP_GATTC_ACL_EVT: return "ACL_EVT";
        case ESP_GATTC_CANCEL_OPEN_EVT: return "CANCEL_OPEN_EVT";
        case ESP_GATTC_SRVC_CHG_EVT: return "SRVC_CHG_EVT";
        case ESP_GATTC_ENC_CMPL_CB_EVT: return "ENC_CMPL_CB_EVT";
        case ESP_GATTC_CFG_MTU_EVT: return "CFG_MTU_EVT";
        case ESP_GATTC_ADV_DATA_EVT: return "ADV_DATA_EVT";
        case ESP_GATTC_MULT_ADV_ENB_EVT: return "MULT_ADV_ENB_EVT";
        case ESP_GATTC_MULT_ADV_UPD_EVT: return "MULT_ADV_UPD_EVT";
        case ESP_GATTC_MULT_ADV_DATA_EVT: return "MULT_ADV_DATA_EVT";
        case ESP_GATTC_MULT_ADV_DIS_EVT: return "MULT_ADV_DIS_EVT";
        case ESP_GATTC_CONGEST_EVT: return "CONGEST_EVT";
        case ESP_GATTC_SCAN_FLT_CFG_EVT: return "SCAN_FLT_CFG_EVT";
        case ESP_GATTC_SCAN_FLT_PARAM_EVT: return "SCAN_FLT_PARAM_EVT";
        case ESP_GATTC_SCAN_FLT_STATUS_EVT: return "SCAN_FLT_STATUS_EVT";
        case ESP_GATTC_ADV_VSC_EVT: return "ADV_VSC_EVT";
        case ESP_GATTC_REG_FOR_NOTIFY_EVT: return "REG_FOR_NOTIFY_EVT";
        case ESP_GATTC_UNREG_FOR_NOTIFY_EVT: return "UNREG_FOR_NOTIFY_EVT";
        case ESP_GATTC_CONNECT_EVT: return "CONNECT_EVT"; 
        case ESP_GATTC_DISCONNECT_EVT: return "DISCONNECT_EVT";
        case ESP_GATTC_READ_MULTIPLE_EVT: return "READ_MULTIPLE_EVT";
        case ESP_GATTC_QUEUE_FULL_EVT: return "QUEUE_FULL_EVT";
        case ESP_GATTC_SET_ASSOC_EVT: return "SET_ASSOC_EVT";
        case ESP_GATTC_GET_ADDR_LIST_EVT: return "GET_ADDR_LIST_EVT";
        case ESP_GATTC_DIS_SRVC_CMPL_EVT: return "DIS_SRVC_CMPL_EVT";
        default: return "UNKNOWN";
    }
}

void TeslaBLEVehicle::process_ble_read_queue() {
    uint32_t now = millis();

    if (!this->ble_read_buffer_.empty() && 
        (this->last_rx_time_ != 0) && 
        (now - this->last_rx_time_ > RX_TIMEOUT)) {
        ESP_LOGW(TAG, "Clearing stale RX buffer (no data received for %lu ms), size was %zu", 
                 (unsigned long)(now - this->last_rx_time_), this->ble_read_buffer_.size());
        ESP_LOGD(TAG, "Stale data: %s", format_hex_pretty(this->ble_read_buffer_.data(), this->ble_read_buffer_.size()).c_str());
        this->ble_read_buffer_.clear();
        this->last_rx_time_ = 0;
    }

    while (!this->ble_read_queue_.empty()) {
        ESP_LOGV(TAG, "process_ble_read_queue: Processing chunk...");
        BLERXChunk& chunk = this->ble_read_queue_.front();

        this->ble_read_buffer_.insert(this->ble_read_buffer_.end(), chunk.buffer.begin(), chunk.buffer.end());
        this->last_rx_time_ = now;
        ESP_LOGV(TAG, "  Appended %zu bytes, buffer size is now %zu", chunk.buffer.size(), this->ble_read_buffer_.size());

        this->ble_read_queue_.pop();

        if (!this->ble_read_buffer_.empty()) {
            ESP_LOGW(TAG, "process_ble_read_queue: Parsing logic disabled due to potential library issues.");
        }
    }
}

void TeslaBLEVehicle::process_response_queue() {
    if (!this->response_queue_.empty()) {
        ESP_LOGW(TAG, "process_response_queue: Handling disabled due to potential library issues.");
        while (!this->response_queue_.empty()) this->response_queue_.pop();
    }
}

void TeslaBLEVehicle::process_command_queue() {
     if (this->command_queue_.empty()) {
         return;
     }
 
     bool is_connected = (this->node_state == espbt::ClientState::CONNECTED);
     bool write_idle = this->ble_write_queue_.empty();
     bool command_slot_available = true; 
     
      if (!this->command_queue_.empty()) {
          BLECommandState front_state = this->command_queue_.front().state;
          if (front_state == BLECommandState::BUILDING || 
              front_state == BLECommandState::SENDING || 
              front_state == BLECommandState::WAITING_FOR_RESPONSE) {
              command_slot_available = false;
              ESP_LOGV(TAG, "process_command_queue: Command slot not available (front state: %d)", front_state);
          }
      }
 
     if (!is_connected) {
         ESP_LOGV(TAG, "process_command_queue: Cannot process command, not connected.");
         return;
     }
 
     if (!write_idle) {
          ESP_LOGV(TAG, "process_command_queue: Cannot process command, write queue busy.");
          return;
     }
 
     if (!command_slot_available) {
          ESP_LOGV(TAG, "process_command_queue: Cannot process command, another command active.");
         return;
     }
 
     BLECommand& command = this->command_queue_.front();
     uint32_t now = millis();
 
     if ((now - command.started_at) > COMMAND_TIMEOUT && 
         (command.state == BLECommandState::SENDING || command.state == BLECommandState::WAITING_FOR_RESPONSE)) { 
         ESP_LOGW(TAG, "Command '%s' timed out after %lu ms", command.execute_name.c_str(), (unsigned long)(now - command.started_at));
         command.state = BLECommandState::TIMEOUT;
         command.retry_count++;
         if (command.retry_count > MAX_RETRIES) {
             ESP_LOGE(TAG, "Command '%s' failed after %d retries (timeout).", command.execute_name.c_str(), MAX_RETRIES);
             command.state = BLECommandState::FAILED; 
             this->command_queue_.pop();
         } else {
              ESP_LOGW(TAG, "Command '%s' timed out, retry %d/%d pending... (Not implemented, failing for now)", 
                      command.execute_name.c_str(), command.retry_count, MAX_RETRIES);
              command.state = BLECommandState::FAILED; 
              this->command_queue_.pop(); 
         }
         return; 
     }
 
     if (command.state == BLECommandState::PENDING) {
         ESP_LOGI(TAG, "Processing command: '%s' (Domain: %d)", command.execute_name.c_str(), command.domain);
         command.state = BLECommandState::BUILDING;
         command.started_at = now; 
 
         ESP_LOGW(TAG, "process_command_queue: Building/Sending logic disabled due to potential library issues.");
         ESP_LOGE(TAG, "Failing command '%s' immediately.", command.execute_name.c_str());
         command.state = BLECommandState::FAILED;
         this->command_queue_.pop();
     }
}

void TeslaBLEVehicle::process_ble_write_queue() {
     if (!this->ble_write_queue_.empty()) {
         ESP_LOGV(TAG, "process_ble_write_queue: %zu chunks waiting to be written", this->ble_write_queue_.size());
     }
}

int TeslaBLEVehicle::writeBLE(const unsigned char *message_buffer, size_t message_length, 
                              esp_gatt_write_type_t write_type, esp_gatt_auth_req_t auth_req) {
    ESP_LOGD(TAG, "writeBLE: Preparing to send %zu bytes to handle 0x%x", message_length, this->write_handle_);
    
    if (this->node_state != espbt::ClientState::CONNECTED) {
        ESP_LOGE(TAG, "writeBLE: Not connected, cannot write.");
        return -1; 
    }

    if (this->write_handle_ == 0) {
        ESP_LOGE(TAG, "writeBLE: Write handle is invalid (0).");
        return -2;
    }

    bool write_in_progress = !this->ble_write_queue_.empty();
    ESP_LOGV(TAG, "writeBLE: Write queue empty before adding? %s", write_in_progress ? "false" : "true");

    size_t offset = 0;
    while (offset < message_length) {
        size_t chunk_size = std::min((size_t)BLOCK_LENGTH, message_length - offset);
        std::vector<unsigned char> chunk_data(message_buffer + offset, message_buffer + offset + chunk_size);
        this->ble_write_queue_.emplace(chunk_data, write_type, auth_req);
        ESP_LOGV(TAG, "  Enqueued chunk: size=%zu", chunk_size);
        offset += chunk_size;
    }
    ESP_LOGD(TAG, "writeBLE: Enqueued %zu chunks for message.", this->ble_write_queue_.size());

    if (!write_in_progress && !this->ble_write_queue_.empty()) {
        ESP_LOGD(TAG, "writeBLE: Initiating write sequence...");
        BLETXChunk& first_chunk = this->ble_write_queue_.front();
        esp_err_t write_status = esp_ble_gattc_write_char(
            this->gattc_if,
            this->conn_id,
            this->write_handle_,
            first_chunk.data.size(),
            first_chunk.data.data(),
            first_chunk.write_type,
            first_chunk.auth_req
        );
        if (write_status != ESP_OK) {
            ESP_LOGE(TAG, "writeBLE: esp_ble_gattc_write_char failed for first chunk: %s", esp_err_to_name(write_status));
            while (!this->ble_write_queue_.empty()) this->ble_write_queue_.pop();
            return -3; 
        } else {
            ESP_LOGV(TAG, "writeBLE: First chunk write initiated successfully.");
        }
    } else {
        ESP_LOGD(TAG, "writeBLE: Write already in progress or queue empty after adding, new chunks queued.");
    }

    return 0;
}

void TeslaBLEVehicle::set_vin(const char *vin) {
    this->vin_ = vin;
    ESP_LOGD(TAG, "VIN set to: %s", this->vin_.c_str());
}

void TeslaBLEVehicle::queue_command(UniversalMessage_Domain domain, const std::string& command_name) {
    ESP_LOGD(TAG, "Queueing command '%s' for domain %d", command_name.c_str(), domain);
    this->command_queue_.emplace(domain, command_name);
}

void TeslaBLEVehicle::queue_rke_action(VCSEC_RKEAction_E action, const std::string& command_name) {
    ESP_LOGD(TAG, "Queueing RKE action %d (%s)", action, command_name.c_str());
    this->command_queue_.emplace(UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY, command_name, action);
}

void TeslaBLEVehicle::invalidateSession(UniversalMessage_Domain domain) { ESP_LOGW(TAG, "invalidateSession not implemented"); }
int TeslaBLEVehicle::nvs_save_session_info(const Signatures_SessionInfo &session_info, const UniversalMessage_Domain domain) { ESP_LOGW(TAG, "nvs_save_session_info not implemented"); return -1; }
int TeslaBLEVehicle::nvs_load_session_info(Signatures_SessionInfo *session_info, const UniversalMessage_Domain domain) { ESP_LOGW(TAG, "nvs_load_session_info not implemented"); return -1; }
int TeslaBLEVehicle::nvs_initialize_private_key() { ESP_LOGW(TAG, "nvs_initialize_private_key not implemented"); return -1; }
int TeslaBLEVehicle::handleSessionInfoUpdate(const UniversalMessage_RoutableMessage& message, UniversalMessage_Domain domain) { ESP_LOGW(TAG, "handleSessionInfoUpdate not implemented"); return -1; }
int TeslaBLEVehicle::handleVCSECVehicleStatus(const VCSEC_VehicleStatus& vehicleStatus) { ESP_LOGW(TAG, "handleVCSECVehicleStatus not implemented"); return -1; }
int TeslaBLEVehicle::wakeVehicle(void) { ESP_LOGW(TAG, "wakeVehicle not implemented"); return -1; }
int TeslaBLEVehicle::sendVCSECActionMessage(VCSEC_RKEAction_E action) { ESP_LOGW(TAG, "sendVCSECActionMessage not implemented"); return -1; }
int TeslaBLEVehicle::sendSessionInfoRequest(UniversalMessage_Domain domain) { ESP_LOGW(TAG, "sendSessionInfoRequest not implemented"); return -1; }
int TeslaBLEVehicle::sendVCSECInformationRequest(void) { ESP_LOGW(TAG, "sendVCSECInformationRequest not implemented"); return -1; }
void TeslaBLEVehicle::enqueueVCSECInformationRequest(bool force) { ESP_LOGW(TAG, "enqueueVCSECInformationRequest not implemented"); }
void TeslaBLEVehicle::regenerateKey() { ESP_LOGW(TAG, "regenerateKey not implemented"); }
int TeslaBLEVehicle::startPair(void) { ESP_LOGW(TAG, "startPair not implemented"); return -1; }

void TeslaBLEVehicle::update() {
    // This method is required by PollingComponent.
    // Add any periodic tasks here if needed.
    // Currently empty.
}

} // namespace tesla_ble_vehicle
} // namespace esphome
