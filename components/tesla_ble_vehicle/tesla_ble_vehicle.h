#pragma once

// ===== BEGINN WICHTIGE INCLUDES =====
// ESP-IDF Header (MUSS zuerst kommen für esp_gattc_cb_param_t)
#include <esp_gattc_api.h>
#include <esp_gap_ble_api.h> // For esp_gap_ble_cb_event_t
// #include <client.h> // Removed, will be included in .cpp if needed there

// TeslaBLE Bibliotheks-Header (ALLE relevanten .pb.h und .h Dateien!)
// Change include path - remove "TeslaBLE/" prefix
#include <common.pb.h>
#include <keys.pb.h>
#include <vcsec.pb.h>
#include <vehicle.pb.h>
// #include <errors.pb.h> // errors.pb.h existiert wahrscheinlich nicht, auskommentiert
#include <universal_message.pb.h>
#include <signatures.pb.h>
#include <car_server.pb.h> 
// Die Haupt-Header der Bibliothek:
#include <errors.h> // Definition von TeslaBLE::Error
#include <client.h> 
#include <peer.h>   
// Füge hier weitere .h Dateien aus TeslaBLE/include hinzu, falls vorhanden/benötigt
// ===== ENDE WICHTIGE INCLUDES =====

// Standard C++ Libraries (können nach den kritischen Headern kommen)
#include <algorithm>
#include <cstring>
#include <iterator>
#include <vector>
#include <queue>
#include <functional>
#include <memory> // For std::unique_ptr

// ESPHome Core and Component Headers
#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include <esphome/components/binary_sensor/binary_sensor.h>
#include <esphome/components/ble_client/ble_client.h>
#include <esphome/components/esp32_ble_tracker/esp32_ble_tracker.h>
#include <esphome/components/sensor/sensor.h>
// Include other necessary ESPHome components if used (e.g., lock, switch, button, text_sensor, number)
#include <esphome/components/lock/lock.h>
#include <esphome/components/switch/switch.h>
#include <esphome/components/button/button.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <esphome/components/number/number.h>
#include <esphome/core/scheduler.h> // Needed for App.scheduler
#include <esphome/core/automation.h>

// ==== Lokale Header (GLEICHES Verzeichnis) ====
#include "log.h" // Assuming log.h is local
// Change include to use quotes for local file
#include "custom_binary_sensor.h"
// ==== Ende Lokale Header ====

// Forward Deklaration für WebServer (REMOVED AsyncWebServer)
#ifdef USE_WEB_SERVER
// Includes and definitions for esp_http_server will be needed later in .cpp
#endif

namespace esphome {
namespace tesla_ble_vehicle {

namespace espbt = esphome::esp32_ble_tracker;

// REMOVE TAG definition from header
// static const char *const TAG = "tesla_ble_vehicle"; // Removed from header
static const char *nvs_key_infotainment = "tk_infotainment";
static const char *nvs_key_vcsec = "tk_vcsec";

// Diese UUIDs scheinen spezifisch für diese Implementierung zu sein,
// nicht die offiziellen von yoziru/tesla-ble? Behalten wir sie vorerst bei.
static const char *const SERVICE_UUID = "00000211-b2d1-43f0-9b88-960cebf8b91e";
static const char *const READ_UUID = "00000213-b2d1-43f0-9b88-960cebf8b91e";
static const char *const WRITE_UUID = "00000212-b2d1-43f0-9b88-960cebf8b91e";

static const int PRIVATE_KEY_SIZE = 228;
static const int PUBLIC_KEY_SIZE = 65;
static const int MAX_BLE_MESSAGE_SIZE = 1024; // Max size of a BLE message
static const int RX_TIMEOUT = 1 * 1000;       // Timeout interval between receiving chunks of a message (1s)
static const int MAX_LATENCY = 4 * 1000;      // Max allowed error when syncing vehicle clock (4s)
static const int BLOCK_LENGTH = 20;           // BLE MTU is 23 bytes, so we need to split the message into chunks (20 bytes as in vehicle_command)
static const int MAX_RETRIES = 5;             // Max number of retries for a command
static const int COMMAND_TIMEOUT = 30 * 1000; // Overall timeout for a command (30s)

enum class BLECommandState
{
    PENDING,               // Waiting in the queue to be processed
    BUILDING,              // Currently being built into a message
    SENDING,               // Chunks are actively being sent via writeBLE / WRITE_CHAR_EVT
    WAITING_FOR_RESPONSE,  // Message sent, waiting for reply from vehicle
    COMPLETED,             // Successfully completed (response received)
    FAILED,                // Failed due to build error, write error, or negative response
    TIMEOUT                // Timed out waiting for response
};

struct BLECommand
{
    // Use global namespace types from .pb.h
    UniversalMessage_Domain domain;
    std::string execute_name;
    VCSEC_RKEAction_E rke_action;
    // --- State Management ---
    BLECommandState state;
    uint32_t started_at;
    uint32_t last_tx_at;
    uint8_t retry_count;

    // Constructor
    BLECommand(UniversalMessage_Domain d, std::string n)
        : domain(d), execute_name(n),
          state(BLECommandState::PENDING), started_at(millis()), last_tx_at(0), retry_count(0) {}

    // Constructor RKE actions
     BLECommand(UniversalMessage_Domain d, std::string n, VCSEC_RKEAction_E action)
        : domain(d), execute_name(n), rke_action(action),
          state(BLECommandState::PENDING), started_at(millis()), last_tx_at(0), retry_count(0) {}
};

struct BLETXChunk
{
    std::vector<unsigned char> data;
    esp_gatt_write_type_t write_type;
    esp_gatt_auth_req_t auth_req;
    uint32_t sent_at = millis();
    uint8_t retry_count = 0;

    BLETXChunk(std::vector<unsigned char> d, esp_gatt_write_type_t wt, esp_gatt_auth_req_t ar)
        : data(d), write_type(wt), auth_req(ar) {}
};

struct BLERXChunk
{
    std::vector<unsigned char> buffer;
    uint32_t received_at = millis();

    BLERXChunk(std::vector<unsigned char> b)
        : buffer(b) {}
};

struct BLEResponse
{
    // Use global namespace types from .pb.h
    UniversalMessage_RoutableMessage message;
    uint32_t received_at;

    BLEResponse(const UniversalMessage_RoutableMessage& m)
        : message(m), received_at(millis()) {} 
};

class TeslaBLEVehicle : public PollingComponent,
                        public ble_client::BLEClientNode
{
public:
    TeslaBLEVehicle();
    void setup() override;
    void loop() override;
    void update() override;
    void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                             esp_ble_gattc_cb_param_t *param) override;
    void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;
    void dump_config() override;
    float get_setup_priority() const override;
    void set_vin(const char *vin);
    void process_command_queue();
    void process_response_queue();
    void process_ble_read_queue();
    void process_ble_write_queue();
    void invalidateSession(UniversalMessage_Domain domain);

    void regenerateKey();
    int startPair(void);
    int nvs_save_session_info(const Signatures_SessionInfo &session_info, const UniversalMessage_Domain domain);
    int nvs_load_session_info(Signatures_SessionInfo *session_info, const UniversalMessage_Domain domain);
    int nvs_initialize_private_key();

    int handleSessionInfoUpdate(const UniversalMessage_RoutableMessage& message, UniversalMessage_Domain domain);
    int handleVCSECVehicleStatus(const VCSEC_VehicleStatus& vehicleStatus);

    int wakeVehicle(void);
    int sendVCSECActionMessage(VCSEC_RKEAction_E action);
    int sendSessionInfoRequest(UniversalMessage_Domain domain);
    int sendVCSECInformationRequest(void);
    void enqueueVCSECInformationRequest(bool force = false);

    int writeBLE(const unsigned char *message_buffer, size_t message_length,
                 esp_gatt_write_type_t write_type, esp_gatt_auth_req_t auth_req);

    // sensors
    void set_binary_sensor_is_asleep(binary_sensor::BinarySensor *s) { isAsleepSensor = static_cast<binary_sensor::CustomBinarySensor *>(s); }
    void updateIsAsleep(bool asleep)
    {
        if (isAsleepSensor) isAsleepSensor->publish_state(asleep);
    }
    void set_binary_sensor_is_unlocked(binary_sensor::BinarySensor *s) { isUnlockedSensor = static_cast<binary_sensor::CustomBinarySensor *>(s); }
    void updateisUnlocked(bool unlocked)
    {
         if (isUnlockedSensor) isUnlockedSensor->publish_state(unlocked);
    }
    void set_binary_sensor_is_user_present(binary_sensor::BinarySensor *s) { isUserPresentSensor = static_cast<binary_sensor::CustomBinarySensor *>(s); }
    void updateIsUserPresent(bool present)
    {
         if (isUserPresentSensor) isUserPresentSensor->publish_state(present);
    }
    void set_binary_sensor_is_charge_flap_open(binary_sensor::BinarySensor *s) { isChargeFlapOpenSensor = static_cast<binary_sensor::CustomBinarySensor *>(s); }
    void updateIsChargeFlapOpen(bool open)
    {
         if (isChargeFlapOpenSensor) isChargeFlapOpenSensor->publish_state(open);
    }
    void setChargeFlapHasState(bool has_state)
    {
         if (isChargeFlapOpenSensor) isChargeFlapOpenSensor->set_has_state(has_state);
    }

    void setSensors(bool has_state)
    {
         if (isAsleepSensor) isAsleepSensor->set_has_state(has_state);
         if (isUnlockedSensor) isUnlockedSensor->set_has_state(has_state);
         if (isUserPresentSensor) isUserPresentSensor->set_has_state(has_state);
         // isChargeFlapOpenSensor wird separat behandelt
    }

    // --- Configuration Setters --- 
    void set_private_key_b64(const std::string &key) { this->private_key_b64_ = key; }
    void set_public_key_str(const std::string &key) { this->public_key_str_ = key; }
#ifdef USE_WEB_SERVER
    void set_http_proxy_enabled(bool enabled) { this->http_proxy_enabled_ = enabled; }
    void set_http_proxy_port(uint16_t port) { this->http_proxy_port_ = port; }
#endif
    // --- Entity Setters (called from __init__.py) --- 
    void set_lock_door(lock::Lock *l) { door_lock_ = l; }
    void set_button_honk(button::Button *b) { honk_button_ = b; }
    void set_button_wake_up(button::Button *b) { wake_up_button_ = b; }
    void set_switch_climate(switch_::Switch *sw) { climate_switch_ = sw; }
    // Add setters for other entities as needed...

    // --- Public Command Methods --- (To be called by buttons, proxy etc.)
    void queue_command(UniversalMessage_Domain domain, const std::string& command_name);
    void queue_rke_action(VCSEC_RKEAction_E action, const std::string& command_name);

protected:
    void set_state(espbt::ClientState new_state);

    std::queue<BLERXChunk> ble_read_queue_;
    std::queue<BLEResponse> response_queue_;
    std::queue<BLETXChunk> ble_write_queue_;
    std::queue<BLECommand> command_queue_;

    std::unique_ptr<TeslaBLE::Client> client_{nullptr};
    uint32_t storage_handle_{0};
    uint16_t handle_{0};
    uint16_t read_handle_{0};
    uint16_t write_handle_{0};

    esp_bt_uuid_t service_uuid_;
    esp_bt_uuid_t read_uuid_;
    esp_bt_uuid_t write_uuid_;

    binary_sensor::CustomBinarySensor *isAsleepSensor{nullptr};
    binary_sensor::CustomBinarySensor *isUnlockedSensor{nullptr};
    binary_sensor::CustomBinarySensor *isUserPresentSensor{nullptr};
    binary_sensor::CustomBinarySensor *isChargeFlapOpenSensor{nullptr};

    std::vector<unsigned char> ble_read_buffer_;
    uint32_t last_rx_time_{0};

    std::vector<uint8_t> private_key_;
    std::string vin_;
    std::string private_key_b64_;
    std::string public_key_str_;

    lock::Lock *door_lock_{nullptr};
    button::Button *honk_button_{nullptr};
    button::Button *wake_up_button_{nullptr};
    switch_::Switch *climate_switch_{nullptr};

    void initializeFlash();
    void openNVSHandle();
    void initializePrivateKey();
    void loadSessionInfo();
    void loadDomainSessionInfo(UniversalMessage_Domain domain);

#ifdef USE_WEB_SERVER
    bool http_proxy_enabled_{false};
    uint16_t http_proxy_port_{8088};
#endif

    esp_gatt_if_t gattc_if{0};
    uint16_t conn_id{0};
    bool parse_device(const espbt::ESPBTDevice &device);
    const char* gattc_event_to_string(esp_gattc_cb_event_t event);
    bool connect_to_vehicle();
    void disconnect_from_vehicle();
    void start_scan();
    void stop_scan();
    bool scan_{false};
};

} // namespace tesla_ble_vehicle
} // namespace esphome
