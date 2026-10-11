/*
 * WebServerLedToggle - switch the onboard LED from a button on a web page.
 *
 * The Pico gets an IP address from your DHCP server, prints it on the serial
 * port and serves a page with a "Toggle LED" button. Open http://<that address>/
 * in a browser on the same network.
 *
 *   GET  /        the page
 *   POST /toggle  toggles the LED, then redirects back to the page
 *   GET  /status  {"led":true,"gpio":1,"uptime_ms":1234,"ip":"192.168.1.50"}
 *
 * Needs the arduino-pico core (lwIP and WebServer come with it). Wiring and
 * pins: see ENC28J60Config in PicoENC28J60.h (SCK=GP2, MOSI=GP3, MISO=GP4,
 * CS=GP5, RST=GP7 by default).
 *
 * There is no authentication: run it on a network you trust.
 */
#include <PicoENC28J60lwIP.h>
#include <WebServer.h>

PicoENC28J60lwIP eth;  // default pins
WebServer server(80);

static bool ledOn = false;

static const char PAGE[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pico LED</title>
<style>
 body{font-family:system-ui,sans-serif;max-width:22rem;margin:3rem auto;padding:0 1rem;text-align:center}
 .led{width:5rem;height:5rem;border-radius:50%;margin:1.5rem auto;border:4px solid #888;background:#ddd}
 .led.on{background:#2e7d32;border-color:#1b5e20;box-shadow:0 0 1.5rem #66bb6a}
 button{font-size:1.4rem;padding:.8rem 2rem;border:0;border-radius:.6rem;background:#1565c0;color:#fff;cursor:pointer}
 small{color:#666}
</style></head><body>
<h1>Pico + ENC28J60</h1>
<div class="led %CLASS%"></div>
<p>The onboard LED is <strong>%STATE%</strong></p>
<form method="post" action="/toggle"><button>Toggle LED</button></form>
<p><small>Served over Ethernet by PicoENC28J60 &middot; <a href="/status">status</a></small></p>
</body></html>
)HTML";

static void setLed(bool on)
{
    ledOn = on;
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

static void handleRoot()
{
    String html = PAGE;
    html.replace("%CLASS%", ledOn ? "on" : "");
    html.replace("%STATE%", ledOn ? "ON" : "OFF");
    server.send(200, "text/html", html);
}

static void handleToggle()
{
    setLed(!ledOn);
    Serial.printf("[HTTP] LED %s\n", ledOn ? "ON" : "OFF");
    server.sendHeader("Location", "/");
    server.send(303);  // See Other: reloading the page does not toggle again
}

static void handleStatus()
{
    String json = "{\"led\":";
    json += ledOn ? "true" : "false";
    json += ",\"gpio\":";
    json += digitalRead(LED_BUILTIN);  // what the pin really does
    json += ",\"uptime_ms\":";
    json += millis();
    json += ",\"ip\":\"";
    json += eth.localIP().toString();
    json += "\"}";
    server.send(200, "application/json", json);
}

void setup()
{
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && millis() - t < 3000;) {
        delay(10);  // give a serial monitor a moment, but do not depend on one
    }

    pinMode(LED_BUILTIN, OUTPUT);
    setLed(false);

    Serial.println("Starting Ethernet...");
    if (!eth.begin()) {
        Serial.println("No ENC28J60 found. Check the wiring and the 3.3 V supply.");
        while (true) delay(1000);
    }

    Serial.print("Waiting for an address from DHCP");
    uint32_t start = millis();
    while (!eth.connected()) {
        Serial.print('.');
        delay(500);
        if (millis() - start > 30000) {
            Serial.println("\nStill no address: is the cable plugged in, and is there a DHCP server?");
            start = millis();
        }
    }
    Serial.printf("\nOpen http://%s/\n", eth.localIP().toString().c_str());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/toggle", HTTP_POST, handleToggle);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();
}

void loop()
{
    server.handleClient();
}
