#include <Arduino.h>
#include "server.h"
#include "debug.h"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

#include <ESP8266mDNS.h>

#define STA_MAXTRIES 10

ESP8266WebServer httpServer (80); // create a webserver object that listens for HTTP using port 80
WebSocketsServer wsServer = WebSocketsServer(81); // create a websocket server on port 81

//
// Setup //
//

// This function setup station mode to connect to wifi
void setupSTA(char* ssid, char* password) {
    DEBUG("Connecting to STA");
    //switching to station mode to use wifi
    WiFi.begin(ssid, password);

    int tries = 0;
    
   // This loop tries connecting to WIFI. if wait too long, timeout and give up
    while (WiFi.status() != WL_CONNECTED) {
        if (tries++ > STA_MAXTRIES) {
            DEBUG("  giving up.");
            return;
        }
        delay(500);
        DEBUG("  ... waiting");
    }
   
    IPAddress myIP = WiFi.localIP(); // get ip address of the arduino
    DEBUG("STA IP address: ");
    DEBUG(myIP.toString());
}

// This fucntion set up a soft access poitnt to establish a wifi network for others connect
// In this way, ESP8266 behaves like a host server
void setupAP(char* ssid, char* password) {
    WiFi.softAP(ssid, password); //set up id and password

    IPAddress myIP = WiFi.softAPIP(); // get IP address of arduino ap
    DEBUG("AP IP address: ");
    DEBUG(myIP.toString());
}

/* make the page availble on the url and write conntect to the page 
 */
void registerPage(const char* url, const char* type, String &content) {
    httpServer.on(url,  [&type, &content]() { httpServer.send(200, type, content); }); 
}

// Initialize HTTP server
void setupHTTP() {
    httpServer.begin();
}

// Initialize Websockets and set an event driven mode on socket
void setupWS(ws_callback_t callback) {
    // start webSocket server
    wsServer.begin();
    wsServer.onEvent(callback);
}

// set up short cut name to the IP address of Arduino' access point. So we can connect by typing string instead of 
// IP address on url
void setupMDNS(char* name) {
    if(MDNS.begin(name)) {
        // Add services to mDNS
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 81);
        DEBUG("mDNS responder started");
    } else {
        DEBUG("mDNS failed\n");
    }
}

// listen for HTTP requests from clients 
void httpLoop() {
	httpServer.handleClient();                                                       
}

// constantly check for websocket events
void wsLoop() {
	wsServer.loop();
}

// send text from ESP8266 to client
void wsSend(int id, char* txt) {
    wsServer.sendTXT(id, txt);
}
