const uint8_t CMD_LOG_SIZE = 12;
String cmdLogEntries[CMD_LOG_SIZE];
uint8_t cmdLogHead = 0;
uint8_t cmdLogCount = 0;
unsigned long serialRxByteCount = 0;
unsigned long serialRxLineCount = 0;
unsigned long serialRxLastMs = 0;
String serialRxLastChars = "";
String serialRxCurrentLine = "";
const uint8_t SERIAL_RX_PREVIEW_LIMIT = 160;

String trimCmdLogRaw(const String &input) {
  String value = input;
  value.trim();
  if (value.length() > 180) {
    value = value.substring(0, 180) + "...";
  }
  return value;
}

String printableSerialChar(char value) {
  if (value == '\n') {
    return "\\n";
  }
  if (value == '\r') {
    return "\\r";
  }
  if (value == '\t') {
    return "\\t";
  }
  if (value >= 32 && value <= 126) {
    return String(value);
  }

  char buffer[5];
  snprintf(buffer, sizeof(buffer), "\\x%02X", (uint8_t)value);
  return String(buffer);
}

void recordSerialRxChar(char value, const String &lineSoFar) {
  serialRxByteCount++;
  serialRxLastMs = millis();
  serialRxLastChars += printableSerialChar(value);
  if (serialRxLastChars.length() > SERIAL_RX_PREVIEW_LIMIT) {
    serialRxLastChars = serialRxLastChars.substring(serialRxLastChars.length() - SERIAL_RX_PREVIEW_LIMIT);
  }

  serialRxCurrentLine = trimCmdLogRaw(lineSoFar);
}

void addCmdLog(const char *source, const String &raw, const char *status, int cmdType, bool hasCmdType) {
  StaticJsonDocument<384> doc;
  doc["ms"] = millis();
  doc["src"] = source;
  doc["status"] = status;
  if (hasCmdType) {
    doc["T"] = cmdType;
  }
  doc["raw"] = trimCmdLogRaw(raw);

  String entry;
  serializeJson(doc, entry);
  cmdLogEntries[cmdLogHead] = entry;
  cmdLogHead = (cmdLogHead + 1) % CMD_LOG_SIZE;
  if (cmdLogCount < CMD_LOG_SIZE) {
    cmdLogCount++;
  }
}

void addParsedCmdLog(const char *source, const String &raw) {
  bool hasCmdType = jsonCmdReceive.containsKey("T");
  int cmdType = hasCmdType ? jsonCmdReceive["T"].as<int>() : 0;
  addCmdLog(source, raw, hasCmdType ? "OK" : "NO_T", cmdType, hasCmdType);
}

void addParseErrorCmdLog(const char *source, const String &raw, DeserializationError err) {
  addCmdLog(source, raw, err.c_str(), 0, false);
}

String getCmdLogJson() {
  String output = "[";
  for (uint8_t i = 0; i < cmdLogCount; i++) {
    uint8_t index = (cmdLogHead + CMD_LOG_SIZE - cmdLogCount + i) % CMD_LOG_SIZE;
    if (i > 0) {
      output += ",";
    }
    output += cmdLogEntries[index];
  }
  output += "]";
  return output;
}

String getSerialStatsJson() {
  StaticJsonDocument<768> doc;
  doc["rx_bytes"] = serialRxByteCount;
  doc["rx_lines"] = serialRxLineCount;
  doc["last_ms"] = serialRxLastMs;
  doc["idle_ms"] = serialRxLastMs == 0 ? -1 : (long)(millis() - serialRxLastMs);
  doc["last_chars"] = serialRxLastChars;
  doc["current_line"] = serialRxCurrentLine;

  String output;
  serializeJson(doc, output);
  return output;
}

void serialParseErrorFeedback(DeserializationError err) {
  StaticJsonDocument<128> doc;
  doc["T"] = 40000;
  doc["err"] = err.c_str();
  serializeJson(doc, Serial);
  Serial.println();
}

// Periodic "board alive" liveness heartbeat — sent every
// HEARTBEAT_INTERVAL_MS regardless of host activity.
void uartHeartbeat() {
  unsigned long now = millis();
  if (now - last_heartbeat_ms < HEARTBEAT_INTERVAL_MS) {
    return;
  }
  last_heartbeat_ms = now;
  heartbeat_count++;

  StaticJsonDocument<128> doc;
  doc["T"] = FB_HEARTBEAT;
  doc["hb"] = heartbeat_count;
  doc["up"] = now;
  serializeJson(doc, Serial);
  Serial.println();
}

void jsonCmdReceiveHandler(){
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch(cmdType){
	case CMD_DDSM_STOP:
                ddsm_stop(
								jsonCmdReceive["id"]);break;
	case CMD_DDSM_CTRL:
                ddsm_ctrl(
								jsonCmdReceive["id"],
								jsonCmdReceive["cmd"],
								jsonCmdReceive["act"]);break;
  case CMD_DDSM_CHANGE_ID:
                ddsm_change_id(
                jsonCmdReceive["id"]);break;
  case CMD_CHANGE_MODE:
                ddsm_change_mode(
                jsonCmdReceive["id"],
                jsonCmdReceive["mode"]);break;
  case CMD_DDSM_ID_CHECK:
                ddsm_id_check();break;
  case CMD_DDSM_INFO:
                ddsm_get_info(
                jsonCmdReceive["id"]);break;
	case CMD_HEARTBEAT_TIME:
                set_heartbeat_time(
								jsonCmdReceive["time"]);break;
  case CMD_TYPE:
                set_ddsm_type(
                jsonCmdReceive["type"]);break;


  // === === === wifi settings. === === ===
  // wifi settings.
  case CMD_WIFI_ON_BOOT: 
                        configWifiModeOnBoot(
                        jsonCmdReceive["cmd"]
                        );break;
  case CMD_SET_AP:      wifiModeAP(
                        jsonCmdReceive["ssid"],
                        jsonCmdReceive["password"]
                        );break;
  case CMD_SET_STA:     wifiModeSTA(
                        jsonCmdReceive["ssid"],
                        jsonCmdReceive["password"]
                        );break;
  case CMD_WIFI_APSTA:  wifiModeAPSTA(
                        jsonCmdReceive["ap_ssid"],
                        jsonCmdReceive["ap_password"],
                        jsonCmdReceive["sta_ssid"],
                        jsonCmdReceive["sta_password"]
                        );break;
  case CMD_WIFI_INFO:   wifiStatusFeedback();break;
  case CMD_WIFI_CONFIG_CREATE_BY_STATUS: 
                        createWifiConfigFileByStatus();break;
  case CMD_WIFI_CONFIG_CREATE_BY_INPUT: 
                        createWifiConfigFileByInput(
                        jsonCmdReceive["mode"],
                        jsonCmdReceive["ap_ssid"],
                        jsonCmdReceive["ap_password"],
                        jsonCmdReceive["sta_ssid"],
                        jsonCmdReceive["sta_password"]
                        );break;
  case CMD_WIFI_STOP:   wifiStop();break;


  // esp-32 dev ctrl.
  case CMD_REBOOT:      esp_restart();break;
  case CMD_FREE_FLASH_SPACE:
                        freeFlashSpace();break;
  case CMD_RESET_WIFI_SETTINGS:
                        deleteFile("wifiConfig.json");break;
  case CMD_NVS_CLEAR:   nvs_flash_erase();
                        delay(100);
                        nvs_flash_init();
                        break;
	}
}

void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;
    recordSerialRxChar(receivedChar, receivedData);

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      serialRxLineCount++;
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
        addParsedCmdLog("SERIAL", receivedData);
      	prev_time = millis();
      	if (stop_flag) {
      		stop_flag = false;
      	}
      	clear_ddsm_buffer();
        jsonCmdReceiveHandler();
      } else {
        addParseErrorCmdLog("SERIAL", receivedData, err);
        serialParseErrorFeedback(err);
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    }
  }
}


void ddsm210_fb() {
  if (Serial1.available() >= 10) {
    uint8_t data[10];
    Serial1.readBytes(data, 10);

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
      crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]){
      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["crc"] = 0;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      return;
    }

    int feedback_type = data[1];
    uint8_t ID = data[0];

    if (feedback_type == 0x64) {
      int speed_data = (data[2] << 8) | data[3];
      if (speed_data & 0x8000) {
        speed_data = -(0x10000 - speed_data);
      }

      int current = (data[4] << 8) | data[5];
      if (current & 0x8000) {
        current = -(0x10000 - current);
      }

      int acceleration_time = data[6];
      int temperature = data[7];
      int fault_code = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["id"]  = ID;
      jsonInfoSend["typ"] = 210;
      jsonInfoSend["spd"] = speed_data;
      jsonInfoSend["crt"] = current;
      jsonInfoSend["act"] = acceleration_time;
      jsonInfoSend["tep"] = temperature;
      jsonInfoSend["err"] = fault_code;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
    } else if (feedback_type == 0x74) {
      int32_t mileage = (int32_t)((uint32_t)data[2] << 24 | (uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5]);
      int ddsm_pos = (data[6] << 8) | data[7];
      int fault_code = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_INFO;
      jsonInfoSend["id"]  = ID;
      jsonInfoSend["typ"] = 210;
      jsonInfoSend["mil"] = mileage;
      jsonInfoSend["pos"] = ddsm_pos;
      jsonInfoSend["err"] = fault_code;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
    }
  }
}

void ddsm115_fb() {
  if (Serial1.available() >= 10) {
    uint8_t data[10];
    Serial1.readBytes(data, 10);

    uint8_t ddsm_id = data[0];

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
      crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]){
      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["crc"] = 0;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      return;
    }

    int ddsm_mode = data[1];

    int ddsm_torque = (data[2] << 8) | data[3];
    if (ddsm_torque & 0x8000) {
      ddsm_torque = -(0x10000 - ddsm_torque);
    }

    int ddsm_spd = (data[4] << 8) | data[5];
    if (ddsm_spd & 0x8000) {
      ddsm_spd = -(0x10000 - ddsm_spd);
    }

    if (get_info_flag) {
      get_info_flag = false;
      int ddsm_temp = data[6];
      int ddsm_u8 = data[7];

      int ddsm_error = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["id"] = ddsm_id;
      jsonInfoSend["typ"] = 115;
      jsonInfoSend["mode"] = ddsm_mode;
      jsonInfoSend["tor"] = ddsm_torque;
      jsonInfoSend["spd"] = ddsm_spd;
      jsonInfoSend["temp"] = ddsm_temp;
      jsonInfoSend["u8"] = ddsm_u8;
      jsonInfoSend["err"] = ddsm_error;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      print_packet(data, 10);
    } else {
      int ddsm_pos = (data[6] << 8) | data[7];
      // if (ddsm_pos & 0x8000) {
      //   ddsm_pos = -(0x10000 - ddsm_pos);
      // }

      int ddsm_error = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["id"] = ddsm_id;
      jsonInfoSend["typ"] = 115;
      jsonInfoSend["mode"] = ddsm_mode;
      jsonInfoSend["tor"] = ddsm_torque;
      jsonInfoSend["spd"] = ddsm_spd;
      jsonInfoSend["pos"] = ddsm_pos;
      jsonInfoSend["err"] = ddsm_error;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      print_packet(data, 10);
    }
  }
}


void ddsm_fb() {
  if (ddsm_type == TYPE_DDSM115) {
    ddsm115_fb();
  } else if (ddsm_type == TYPE_DDSM210) {
    ddsm210_fb();
  }
}
