#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"
#include "mqtt_client.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"


/******************************************* BEGIN 阿里云移植代码 BEGIN *******************************************/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include <esp_http_client.h>

#define   Aliyun_hostname       "iot-06z00godbmkjb42.mqtt.iothub.aliyuncs.com" //或称mqttHostUrl、Broker Address
#define   Aliyun_port       1883
#define   Aliyun_client_id  "k1r45F1HfgR.Cat_butler|securemode=2,signmethod=hmacsha256,timestamp=1742002977452|"
#define   Aliyun_username   "Cat_butler&k1r45F1HfgR"
#define   Aliyun_password   "798bb01ae1c4d9305a4062c9f2415c5815d4352ca93aeac8946fe1937a351513"

#define   AliyunPublishTopic_user_update    "/k1r45F1HfgR/Cat_butler/user/butlertx"
#define   AliyunSubscribeTopic_user_get     "/k1r45F1HfgR/Cat_butler/user/butlerrx"

// 阿里云语音合成服务配置
#define ALIYUN_TTS_ENDPOINT "nls-gateway.aliyuncs.com"
#define ALIYUN_TTS_PATH "/stream/v1/tts"
#define ALIYUN_ACCESS_KEY_ID "LTAI5t6WcQpQ6Jdry8vRSQ7F"
#define ALIYUN_ACCESS_KEY_SECRET "rMOwLOeEruf4jyjJJVjA50soaRa9Yd"

static esp_mqtt_client_handle_t ali_mqtt_client = nullptr;

static void ali_mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(ali_mqtt_client, AliyunSubscribeTopic_user_get, 0);
            ESP_LOGI(TAG, "Connected to Aliyun IoT Platform");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected from Aliyun IoT");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Received data: %.*s", event->data_len, event->data);
            Application::GetInstance().ProcessHealthData(event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error: %d", event->error_handle->error_type);
            break;
        default:
            break;
    }
}

void Application::SynthesizeAndPlaySpeech(const std::string& text) {
    esp_http_client_config_t config = {
       .host = ALIYUN_TTS_ENDPOINT,
       .path = ALIYUN_TTS_PATH,
       .method = HTTP_METHOD_POST,
       .transport_type = HTTP_TRANSPORT_OVER_TCP
    };

    ESP_LOGI(TAG, "Initializing HTTP client to connect to Aliyun TTS endpoint: %s%s", ALIYUN_TTS_ENDPOINT, ALIYUN_TTS_PATH);
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // 构建请求体
    std::string request_body = "{\"appkey\":\"Ok3AlAfU52fxSSpl\",\"text\":\"" + text + "\",\"format\":\"wav\",\"sample_rate\":16000}";
    ESP_LOGI(TAG, "TTS request body: %s", request_body.c_str());

    esp_http_client_set_post_field(client, request_body.c_str(), request_body.length());
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", "APPCODE your_appcode");

    ESP_LOGI(TAG, "Performing HTTP request to Aliyun TTS");
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        // 增加状态码检查
        int status_code = esp_http_client_get_status_code(client);
        if (status_code != 200) {
            ESP_LOGE(TAG, "HTTP请求失败，状态码: %d", status_code);
            esp_http_client_cleanup(client);
            return;
        }

        // 改用动态获取内容长度
        int content_length = esp_http_client_get_content_length(client);
        if (content_length <= 0) {
            ESP_LOGE(TAG, "无效的音频内容长度: %d", content_length);
            esp_http_client_cleanup(client);
            return;
        }

        std::vector<uint8_t> audio_data;
        audio_data.reserve(content_length);
        
        // 分块读取数据
        int total_read = 0;
        char buffer[256];
        while (total_read < content_length) {
            int read_len = esp_http_client_read(client, buffer, sizeof(buffer));
            if (read_len <= 0) break;
            audio_data.insert(audio_data.end(), buffer, buffer + read_len);
            total_read += read_len;
        }

        // 校验实际读取长度
        if (total_read != content_length) {
            ESP_LOGE(TAG, "音频数据不完整，预期:%d 实际:%d", content_length, total_read);
            esp_http_client_cleanup(client);
            return;
        }

        // 增加音频格式校验
        if (audio_data.size() < 44 || 
            memcmp(audio_data.data(), "RIFF", 4) != 0 || 
            memcmp(audio_data.data() + 8, "WAVE", 4) != 0) {
            ESP_LOGE(TAG, "无效的WAV文件头");
            esp_http_client_cleanup(client);
            return;
        }

        Schedule([this, audio_data = std::move(audio_data)]() {
            Board::GetInstance().GetAudioCodec()->EnableOutput(true);
            PlayLocalFile(reinterpret_cast<const char*>(audio_data.data()), audio_data.size());
        });
    } else {
        ESP_LOGE(TAG, "HTTP请求失败: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void Application::InitAliyunIoT() {
    if (ali_mqtt_initialized_) {
        return;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.broker.address.hostname = Aliyun_hostname;
    mqtt_cfg.broker.address.port = Aliyun_port;
    mqtt_cfg.credentials.client_id = Aliyun_client_id;
    mqtt_cfg.credentials.username = Aliyun_username;
    mqtt_cfg.credentials.authentication.password = Aliyun_password;

    ali_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(ali_mqtt_client, MQTT_EVENT_ANY, ali_mqtt_event_handler, NULL);
    esp_mqtt_client_start(ali_mqtt_client);
    ali_mqtt_initialized_ = true;
}

void Application::ProcessHealthData(const char* data, size_t len) {
    // {
    //     "days": [
    //         {
    //             "date": "2024-03-15",
    //             "metrics": {
    //                 "water_ml": 280,
    //                 "food_g": 150,
    //                 "excretion": 3
    //             }
    //         },
    //         {
    //             "date": "2024-03-16",
    //             "metrics": {
    //                 "water_ml": 320,
    //                 "food_g": 170,
    //                 "excretion": 2
    //             }
    //         },
    //         {
    //             "date": "2024-03-17",
    //             "metrics": {
    //                 "water_ml": 240,
    //                 "food_g": 190,
    //                 "excretion": 4
    //             }
    //         }
    //     ]
    // }

    cJSON* root = cJSON_Parse(data);
    if (!root) {
        ESP_LOGE(TAG, "健康数据解析失败");
        return;
    }

    // 清空历史数据
    health_data_.clear();
    health_dates_.clear();

    // 解析数据层级
    cJSON* days = cJSON_GetObjectItem(root, "days");
    if (days && cJSON_IsArray(days)) {
        int day_count = cJSON_GetArraySize(days);
        
        // 严格校验数据条目
        if (day_count != 3) {
            ESP_LOGE(TAG, "无效数据天数：%d（要求3天）", day_count);
            cJSON_Delete(root);
            return;
        }

        // 遍历处理每天数据
        for (int i = 0; i < day_count; ++i) {
            cJSON* day = cJSON_GetArrayItem(days, i);
            cJSON* date = cJSON_GetObjectItem(day, "date");
            cJSON* metrics = cJSON_GetObjectItem(day, "metrics");

            // 字段完整性校验
            if (!date || !metrics) {
                ESP_LOGE(TAG, "第%d天数据字段缺失", i+1);
                continue;
            }

            // 指标提取
            std::map<std::string, int> day_data;
            day_data["water"] = cJSON_GetObjectItem(metrics, "water_ml")->valueint;
            day_data["food"] = cJSON_GetObjectItem(metrics, "food_g")->valueint;
            day_data["excretion"] = cJSON_GetObjectItem(metrics, "excretion")->valueint;

            // 数据存储
            health_dates_.push_back(date->valuestring);
            health_data_.push_back(day_data);
        }

        // 发送分析请求（通过合法接口）
        if (health_data_.size() == 3) {
            
            cJSON* analysis_req = cJSON_CreateObject();
            cJSON_AddStringToObject(analysis_req, "type", "health_report");
            
            cJSON* data_array = cJSON_AddArrayToObject(analysis_req, "data");
            for (int i = 0; i < 3; ++i) {
                cJSON* day_obj = cJSON_CreateObject();
                cJSON_AddStringToObject(day_obj, "date", health_dates_[i].c_str());
                cJSON_AddNumberToObject(day_obj, "water", health_data_[i]["water"]);
                cJSON_AddNumberToObject(day_obj, "food", health_data_[i]["food"]);
                cJSON_AddNumberToObject(day_obj, "excretion", health_data_[i]["excretion"]);
                cJSON_AddItemToArray(data_array, day_obj);
            }
            
            char* json_str = cJSON_PrintUnformatted(analysis_req);
            ESP_LOGI(TAG, "待发送的 JSON 内容: %s", json_str);  // 打印原始 JSON 字符串
            
            protocol_->SendCommand(std::string(json_str));  // 通过公共接口发送
            cJSON_Delete(analysis_req);
            free(json_str);
            
            ESP_LOGI(TAG, "已发送健康分析请求");
            
            std::string speech_content = "";
            for(int i=0; i<3; i++){
                speech_content += health_dates_[i] + 
                                "，饮水" + std::to_string(health_data_[i]["water"]) + "毫升，" +
                                "进食" + std::to_string(health_data_[i]["food"]) + "克，" +
                                "排泄" + std::to_string(health_data_[i]["excretion"]) + "次。";
            }

            SynthesizeAndPlaySpeech(speech_content);
            Schedule([this]() {
                SetDeviceState(kDeviceStateSpeaking);
                Board::GetInstance().GetAudioCodec()->EnableOutput(true);
            });
        }
    }
    cJSON_Delete(root);
}

/******************************************* END 阿里云移植代码 END *******************************************/

static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "fatal_error",
    "invalid_state"
};

extern const char p3_err_reg_start[] asm("_binary_err_reg_p3_start");
extern const char p3_err_reg_end[] asm("_binary_err_reg_p3_end");
extern const char p3_err_pin_start[] asm("_binary_err_pin_p3_start");
extern const char p3_err_pin_end[] asm("_binary_err_pin_p3_end");
extern const char p3_wificonfig_start[] asm("_binary_wificonfig_p3_start");
extern const char p3_wificonfig_end[] asm("_binary_wificonfig_p3_end");
extern const char p3_upgrade_start[] asm("_binary_upgrade_p3_start");
extern const char p3_upgrade_end[] asm("_binary_upgrade_p3_end");
extern const char p3_output_start[] asm("_binary_output_p3_start");
extern const char p3_output_end[] asm("_binary_output_p3_end");
extern const char p3_cat_start[] asm("_binary_cat_p3_start");
extern const char p3_cat_end[] asm("_binary_cat_p3_end");
Application::Application() {
    event_group_ = xEventGroupCreate();
    background_task_ = new BackgroundTask(4096 * 8);

    ota_.SetCheckVersionUrl(CONFIG_OTA_VERSION_URL);
    ota_.SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
}

Application::~Application() {
    if (background_task_ != nullptr) {
        delete background_task_;
    }
    vEventGroupDelete(event_group_);
}

// void Application::CheckNewVersion() {
//     auto& board = Board::GetInstance();
//     auto display = board.GetDisplay();
//     // Check if there is a new firmware version available
//     ota_.SetPostData(board.GetJson());

//     while (true) {
//         if (ota_.CheckVersion()) {
//             if (ota_.HasNewVersion()) {
//                 Alert("Info", "正在升级固件");
//                 // Wait for the chat state to be idle
//                 do {
//                     vTaskDelay(pdMS_TO_TICKS(3000));
//                 } while (GetDeviceState() != kDeviceStateIdle);

//                 // Use main task to do the upgrade, not cancelable
//                 Schedule([this, &board, display]() {
//                     SetDeviceState(kDeviceStateUpgrading);
                    
//                     display->SetIcon(FONT_AWESOME_DOWNLOAD);
//                     display->SetStatus("新版本 " + ota_.GetFirmwareVersion());

//                     board.SetPowerSaveMode(false);
// #if CONFIG_USE_AUDIO_PROCESSING
//                     wake_word_detect_.StopDetection();
// #endif
//                     // 预先关闭音频输出，避免升级过程有音频操作
//                     board.GetAudioCodec()->EnableOutput(false);
//                     {
//                         std::lock_guard<std::mutex> lock(mutex_);
//                         audio_decode_queue_.clear();
//                     }
//                     background_task_->WaitForCompletion();
//                     delete background_task_;
//                     background_task_ = nullptr;
//                     vTaskDelay(pdMS_TO_TICKS(1000));

//                     ota_.StartUpgrade([display](int progress, size_t speed) {
//                         char buffer[64];
//                         snprintf(buffer, sizeof(buffer), "%d%% %zuKB/s", progress, speed / 1024);
//                         display->SetStatus(buffer);
//                     });

//                     // If upgrade success, the device will reboot and never reach here
//                     display->SetStatus("更新失败");
//                     ESP_LOGI(TAG, "Firmware upgrade failed...");
//                     vTaskDelay(pdMS_TO_TICKS(3000));
//                     esp_restart();
//                 });
//             } else {
//                 ota_.MarkCurrentVersionValid();
//                 display->ShowNotification("版本 " + ota_.GetCurrentVersion());
//             }
//             return;
//         }

//         // Check again in 60 seconds
//         vTaskDelay(pdMS_TO_TICKS(60000));
//     }
// }

void Application::Alert(const std::string& title, const std::string& message) {
    ESP_LOGW(TAG, "Alert: %s, %s", title.c_str(), message.c_str());
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification(message);

    if (message == "进入配网模式") {
        PlayLocalFile(p3_wificonfig_start, p3_wificonfig_end - p3_wificonfig_start);
    } else if (message == "正在升级固件") {
        PlayLocalFile(p3_upgrade_start, p3_upgrade_end - p3_upgrade_start);
    } else if (message == "请插入SIM卡") {
        PlayLocalFile(p3_err_pin_start, p3_err_pin_end - p3_err_pin_start);
    } else if (message == "无法接入网络，请检查流量卡状态") {
        PlayLocalFile(p3_err_reg_start, p3_err_reg_end - p3_err_reg_start);
    }
}

void Application::PlayLocalFile(const char* data, size_t size) {
    ESP_LOGI(TAG, "PlayLocalFile: %zu bytes", size);
    SetDecodeSampleRate(16000);
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        std::vector<uint8_t> opus;
        opus.resize(payload_size);
        memcpy(opus.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(opus));
    }
}

void Application::ToggleChatState() {
    Schedule([this]() {
        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }

        if (device_state_ == kDeviceStateIdle) {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                Alert("Error", "Failed to open audio channel");
                SetDeviceState(kDeviceStateIdle);
                return;
            }

            keep_listening_ = true;
            protocol_->SendStartListening(kListeningModeAutoStop);
            SetDeviceState(kDeviceStateListening);
        } else if (device_state_ == kDeviceStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
        } else if (device_state_ == kDeviceStateListening) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::StartListening() {
    Schedule([this]() {
        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }
        
        keep_listening_ = false;
        if (device_state_ == kDeviceStateIdle) {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    SetDeviceState(kDeviceStateIdle);
                    Alert("Error", "Failed to open audio channel");
                    return;
                }
            }
            protocol_->SendStartListening(kListeningModeManualStop);
            SetDeviceState(kDeviceStateListening);
        } else if (device_state_ == kDeviceStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
            protocol_->SendStartListening(kListeningModeManualStop);
            // FIXME: Wait for the speaker to empty the buffer
            vTaskDelay(pdMS_TO_TICKS(120));
            SetDeviceState(kDeviceStateListening);
        }
    });
}

void Application::StopListening() {
    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {
    PlayLocalFile(p3_cat_start, p3_cat_end - p3_cat_start);
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();
    opus_decode_sample_rate_ = codec->output_sample_rate();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(opus_decode_sample_rate_, 1);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    // For ML307 boards, we use complexity 5 to save bandwidth
    // For other boards, we use complexity 3 to save CPU
    if (board.GetBoardType() == "ml307") {
        ESP_LOGI(TAG, "ML307 board detected, setting opus encoder complexity to 5");
        opus_encoder_->SetComplexity(5);
    } else {
        ESP_LOGI(TAG, "WiFi board detected, setting opus encoder complexity to 3");
        opus_encoder_->SetComplexity(3);
    }

    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    codec->OnInputReady([this, codec]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_INPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->OnOutputReady([this]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_OUTPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->Start();

    /* Start the main loop */
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->MainLoop();
        vTaskDelete(NULL);
    }, "main_loop", 4096 * 2, this, 2, nullptr);

    /* Wait for the network to be ready */
    board.StartNetwork();

    /******************************************* BEGIN 阿里云连接代码调用 BEGIN *******************************************/
    InitAliyunIoT();  // 新增阿里云IoT初始化
    /******************************************* END 阿里云连接代码调用  END *******************************************/

    // Initialize the protocol
    display->SetStatus("初始化协议");
#ifdef CONFIG_CONNECTION_TYPE_WEBSOCKET
    protocol_ = std::make_unique<WebsocketProtocol>();
#else
    protocol_ = std::make_unique<MqttProtocol>();
#endif
    protocol_->OnNetworkError([this](const std::string& message) {
        Alert("Error", std::move(message));
    });
    protocol_->OnIncomingAudio([this](std::vector<uint8_t>&& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_state_ == kDeviceStateSpeaking) {
            audio_decode_queue_.emplace_back(std::move(data));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "服务器的音频采样率 %d 与设备输出的采样率 %d 不一致，重采样后可能会失真",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
        SetDecodeSampleRate(protocol_->server_sample_rate());
        // 物联网设备描述符
        last_iot_states_.clear();
        auto& thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        ESP_LOGI(TAG, "type: %s", type->valuestring);
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (device_state_ == kDeviceStateSpeaking) {
                        background_task_->WaitForCompletion();
                        if (keep_listening_) {
                            protocol_->SendStartListening(kListeningModeAutoStop);
                            SetDeviceState(kDeviceStateListening);
                        } else {
                            SetDeviceState(kDeviceStateIdle);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (text != NULL) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message);
                    });
                }
            }
        } 
        else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (text != NULL) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                std::string message = text->valuestring;
                
                // 新增指令检测-厕所冲水
                if (message.find("厕所冲水") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了厕所冲水指令");
                    const char* payload = "{\"push_water\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }

                // 新增指令检测-开启风扇
                if (message.find("开启风扇") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了开启风扇指令");
                    const char* payload = "{\"fan\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }

                // 新增指令检测-关闭风扇
                if (message.find("关闭风扇") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了关闭风扇指令");
                    const char* payload = "{\"fan\":0}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-开始喂猫
                if (message.find("开始喂猫") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了开始喂猫指令");
                    const char* payload = "{\"food_door\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-停止喂猫
                if (message.find("停止喂猫") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了停止喂猫指令");
                    const char* payload = "{\"food_door\":0}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-开启水泵
                if (message.find("开启水泵") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了开启水泵指令");
                    const char* payload = "{\"Bump\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }

                // 新增指令检测-关闭水泵
                if (message.find("关闭水泵") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了关闭水泵指令");
                    const char* payload = "{\"Bump\":0}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-开始自动清理
                if (message.find("开始自动清理") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了自动清理指令");
                    const char* payload = "{\"basin_clean\":1}";;
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-清理猫砂盆
                if (message.find("清理猫砂盆") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了清理猫砂盆指令");
                    const char* payload = "{\"clean_feces\":1}";;
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-开启厕所灯
                if (message.find("开启厕所灯") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了开启厕所灯指令");
                    const char* payload = "{\"lamp\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }

                // 新增指令检测-关闭厕所灯
                if (message.find("关闭厕所灯") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了关闭厕所灯指令");
                    const char* payload = "{\"lamp\":0}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                }
                
                // 新增指令检测-猫咪健康检测
                if (message.find("猫咪健康检测") != std::string::npos) {
                    ESP_LOGI(TAG, "[指令检测] 用户发出了猫咪健康检测指令");
                    const char* payload = "{\"send_me_data\":1}";
                    int msg_id = esp_mqtt_client_publish(ali_mqtt_client, AliyunPublishTopic_user_update, payload, strlen(payload), 1, 0);
                    ESP_LOGI(TAG, "Sent auto_clean command, msg_id=%d", msg_id);
                    if (protocol_) {
                        msg_id = esp_mqtt_client_subscribe(ali_mqtt_client, AliyunSubscribeTopic_user_get, 0);
                        ESP_LOGI(TAG, "execute subscribe event, msg_id=%d", msg_id);
                    }
                    // 更新用户界面
                    Schedule([this]() {
                        auto display = Board::GetInstance().GetDisplay();
                        display->SetStatus("正在获取近三日数据...");
                        display->SetEmotion("loading");
                    });
                }
                // 更新用户界面
                Schedule([this, display, message = std::move(message)]() {
                    display->SetChatMessage("user", message);
                });
            }
        }
        else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (emotion != NULL) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str);
                });
            }
        } else if (strcmp(type->valuestring, "iot") == 0) {
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (commands != NULL) {
                auto& thing_manager = iot::ThingManager::GetInstance();
                for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
                    auto command = cJSON_GetArrayItem(commands, i);
                    thing_manager.Invoke(command);
                }
            }
        }
    });

    // Check for new firmware version or get the MQTT broker address
    // xTaskCreate([](void* arg) {
    //     Application* app = (Application*)arg;
    //     app->CheckNewVersion();
    //     vTaskDelete(NULL);
    // }, "check_new_version", 4096 * 2, this, 1, nullptr);


#if CONFIG_USE_AUDIO_PROCESSING
    audio_processor_.Initialize(codec->input_channels(), codec->input_reference());
    audio_processor_.OnOutput([this](std::vector<int16_t>&& data) {
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    });

    wake_word_detect_.Initialize(codec->input_channels(), codec->input_reference());
    wake_word_detect_.OnVadStateChange([this](bool speaking) {
        Schedule([this, speaking]() {
            if (device_state_ == kDeviceStateListening) {
                if (speaking) {
                    voice_detected_ = true;
                } else {
                    voice_detected_ = false;
                }
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        });
    });

    wake_word_detect_.OnWakeWordDetected([this](const std::string& wake_word) {
        Schedule([this, &wake_word]() { // 原代码使用引用捕获（存在潜在问题）
            if (device_state_ == kDeviceStateIdle) {
                SetDeviceState(kDeviceStateConnecting);
                wake_word_detect_.EncodeWakeWordData();

                if (!protocol_->OpenAudioChannel()) {
                    ESP_LOGE(TAG, "Failed to open audio channel");
                    SetDeviceState(kDeviceStateIdle);
                    wake_word_detect_.StartDetection();
                    return;
                }
                
                std::vector<uint8_t> opus;
                // 编码并发送唤醒词数据
                while (wake_word_detect_.GetWakeWordOpus(opus)) {
                    protocol_->SendAudio(opus);
                }
                // 通知服务器唤醒词已检测
                protocol_->SendWakeWordDetected(wake_word);
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                keep_listening_ = true; // 原值：true，需修改为 false
                SetDeviceState(kDeviceStateListening);
            } else if (device_state_ == kDeviceStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            }

            // 恢复唤醒词检测
            wake_word_detect_.StartDetection();
        });
    });
    wake_word_detect_.StartDetection();
#endif

    SetDeviceState(kDeviceStateIdle);
}

void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// The Main Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainLoop() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_,
            SCHEDULE_EVENT | AUDIO_INPUT_READY_EVENT | AUDIO_OUTPUT_READY_EVENT,
            pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & AUDIO_INPUT_READY_EVENT) {
            InputAudio();
        }
        if (bits & AUDIO_OUTPUT_READY_EVENT) {
            OutputAudio();
        }
        if (bits & SCHEDULE_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void Application::ResetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    last_output_time_ = std::chrono::steady_clock::now();
    Board::GetInstance().GetAudioCodec()->EnableOutput(true);
}

void Application::OutputAudio() {
    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
        // Disable the output if there is no audio data for a long time
        if (device_state_ == kDeviceStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                codec->EnableOutput(false);
            }
        }
        return;
    }

    if (device_state_ == kDeviceStateListening) {
        audio_decode_queue_.clear();
        return;
    }

    last_output_time_ = now;
    auto opus = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();

    background_task_->Schedule([this, codec, opus = std::move(opus)]() mutable {
        if (aborted_) {
            return;
        }

        std::vector<int16_t> pcm;
        if (!opus_decoder_->Decode(std::move(opus), pcm)) {
            return;
        }

        // Resample if the sample rate is different
        if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(pcm.size());
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), pcm.size(), resampled.data());
            pcm = std::move(resampled);
        }
        
        codec->OutputData(pcm);
    });
}

void Application::InputAudio() {
    auto codec = Board::GetInstance().GetAudioCodec();
    std::vector<int16_t> data;
    if (!codec->InputData(data)) {
        return;
    }

    if (codec->input_sample_rate() != 16000) {
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        } else {
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    }
    
#if CONFIG_USE_AUDIO_PROCESSING
    if (audio_processor_.IsRunning()) {
        audio_processor_.Input(data);
    }
    if (wake_word_detect_.IsDetectionRunning()) {
        wake_word_detect_.Feed(data);
    }
#else
    if (device_state_ == kDeviceStateListening) {
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    }
#endif
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);
    // The state is changed, wait for all background tasks to finish
    background_task_->WaitForCompletion();

    auto display = Board::GetInstance().GetDisplay();
    auto led = Board::GetInstance().GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus("待命");
            display->SetEmotion("neutral");
#ifdef CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Stop();
#endif
            break;
        case kDeviceStateConnecting:
            display->SetStatus("连接中...");
            break;
        case kDeviceStateListening:
            display->SetStatus("倾听中...");
            display->SetEmotion("neutral");
            ResetDecoder();
            opus_encoder_->ResetState();
#if CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Start();
#endif
            UpdateIotStates();
            break;
        case kDeviceStateSpeaking:
            display->SetStatus("说话中...");
            ResetDecoder();
#if CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Stop();
#endif
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::SetDecodeSampleRate(int sample_rate) {
    if (opus_decode_sample_rate_ == sample_rate) {
        return;
    }

    opus_decode_sample_rate_ = sample_rate;
    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(opus_decode_sample_rate_, 1);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decode_sample_rate_, codec->output_sample_rate());
        output_resampler_.Configure(opus_decode_sample_rate_, codec->output_sample_rate());
    }
}

void Application::UpdateIotStates() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    auto states = thing_manager.GetStatesJson();
    if (states != last_iot_states_) {
        last_iot_states_ = states;
        protocol_->SendIotStates(states);
    }
}
