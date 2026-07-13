// http server funcs.

#include "web_page.h"

// Create AsyncWebServer object on port 80
WebServer server(80);

void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
}

void webCtrlServer(){
  server.on("/", handleRoot);

  server.on("/js", [](){
    String jsonCmdWebString = server.hasArg("json") ? server.arg("json") : server.arg(0);
    DeserializationError err = deserializeJson(jsonCmdReceive, jsonCmdWebString);
    String jsonResponseString;

    if (err == DeserializationError::Ok) {
      addParsedCmdLog("WEB", jsonCmdWebString);
      jsonCmdReceiveHandler();
      serializeJson(jsonInfoSend, jsonResponseString);
    } else {
      addParseErrorCmdLog("WEB", jsonCmdWebString, err);
      jsonInfoSend.clear();
      jsonInfoSend["T"] = 40000;
      jsonInfoSend["err"] = err.c_str();
      serializeJson(jsonInfoSend, jsonResponseString);
    }

    server.send(200, "text/plain", jsonResponseString);
    jsonInfoSend.clear();
    jsonCmdReceive.clear();
  });

  server.on("/cmdlog", [](){
    server.send(200, "application/json", getCmdLogJson());
  });

  server.on("/serialstats", [](){
    server.send(200, "application/json", getSerialStatsJson());
  });

  server.on("/serialtest", [](){
    Serial.println("{\"T\":49999,\"src\":\"WEB_SERIAL_TEST\"}");
    addCmdLog("WEB", "serialtest", "SERIAL_PRINT", 0, false);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}

void initHttpWebServer(){
  webCtrlServer();
}
