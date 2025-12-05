#include <FastLED.h>
#include <WiFiS3.h>   // for UNO R4 WiFi; adjust if you use a different lib

// ====== WiFi / backend config ======
const char* WIFI_SSID     = "r4Uno";
const char* WIFI_PASSWORD = "1234";

// Example: expose a JSON endpoint from your Apps Script without JSONP
// e.g. https://script.google.com/macros/s/....../exec?device=arduino
const char* SCRIPT_HOST = "script.google.com";   // or script.googleusercontent.com depending on deployment
const int   SCRIPT_PORT = 443;                    // use 443 with TLS-capable client if needed
const char* SCRIPT_PATH = "/macros/s/AKfycbzN-oyQ6uG5GwO0eZrXvlvBVAEMYoSS589ZuOGnhdd7zMa2Exa83ntu8NlEXzts7p9l/exec?device=arduino";
//https://script.google.com/macros/s/AKfycbzN-oyQ6uG5GwO0eZrXvlvBVAEMYoSS589ZuOGnhdd7zMa2Exa83ntu8NlEXzts7p9l/exec

WiFiSSLClient client;   // use TLS   // For HTTP. For HTTPS you'd need a TLS client with proper config.

// ====== NeoPixel / FastLED config ======
#define NUM_LEDS_PER_LAMP 12

// Change these to the actual pins you wired!
#define PIN_EW_RED     6
#define PIN_EW_YELLOW  7
#define PIN_EW_GREEN   8

CRGB ewRed[NUM_LEDS_PER_LAMP];
CRGB ewYellow[NUM_LEDS_PER_LAMP];
CRGB ewGreen[NUM_LEDS_PER_LAMP];

// ====== Shared timing / phase logic ======
const unsigned long T_EW_GREEN   = 10000;
const unsigned long T_EW_YELLOW  = 3000;
const unsigned long T_ALL_RED_1  = 1000;
const unsigned long T_NS_GREEN   = 10000;
const unsigned long T_NS_YELLOW  = 3000;
const unsigned long T_ALL_RED_2  = 1000;
const unsigned long PEDESTRIAN_ACTIVE_MS = 7000;

enum Color { RED, YELLOW, GREEN };
enum Phase {
  PH_EW_GREEN,
  PH_EW_YELLOW,
  PH_ALL_RED_1,
  PH_NS_GREEN,
  PH_NS_YELLOW,
  PH_ALL_RED_2,
  PH_PEDESTRIAN,
  PH_BLINK_YELLOW
};

struct ViewState {
  Phase phase;
  Color ew;
  Color ns;
};

struct RemoteState {
  // Mirrors what your Sheet / Apps Script provides:
  unsigned long cycle_ms;
  long offset_A_ms;
  long offset_B_ms;
  bool ped_request;
  bool blink_mode;
  unsigned long base_epoch_ms;        // epoch of base in ms
  unsigned long ped_active_until_ms;  // 0 if none
};

RemoteState remoteState = {
  26000,   // cycle_ms
  0,       // offset_A_ms
  0,       // offset_B_ms
  false,   // ped_request
  false,   // blink_mode
  0,       // base_epoch_ms
  0        // ped_active_until_ms
};

// NOTE: you can hardcode this to "site A" or "site B"
const char SITE_ID = 'A';

ViewState phaseFromElapsed(unsigned long elapsed, unsigned long cycle) {
  unsigned long t = elapsed % cycle;
  if (t < T_EW_GREEN) return { PH_EW_GREEN, GREEN, RED };
  if (t < T_EW_GREEN + T_EW_YELLOW) return { PH_EW_YELLOW, YELLOW, RED };
  if (t < T_EW_GREEN + T_EW_YELLOW + T_ALL_RED_1) return { PH_ALL_RED_1, RED, RED };
  if (t < T_EW_GREEN + T_EW_YELLOW + T_ALL_RED_1 + T_NS_GREEN) return { PH_NS_GREEN, RED, GREEN };
  if (t < T_EW_GREEN + T_EW_YELLOW + T_ALL_RED_1 + T_NS_GREEN + T_NS_YELLOW) return { PH_NS_YELLOW, RED, YELLOW };
  return { PH_ALL_RED_2, RED, RED };
}

// ====== Helpers ======

  void fillLamp(CRGB* lamp, int count, const CRGB& color) {
    for (int i = 0; i < count; i++) {
      lamp[i] = color;
    }
  }

  void clearAllLamps() {
    fillLamp(ewRed,    NUM_LEDS_PER_LAMP, CRGB::Black);
    fillLamp(ewYellow, NUM_LEDS_PER_LAMP, CRGB::Black);
    fillLamp(ewGreen,  NUM_LEDS_PER_LAMP, CRGB::Black);
  }

  void setNeoPixelColor(Color c) {
    clearAllLamps();

    switch (c) {
      case RED:
        fillLamp(ewRed, NUM_LEDS_PER_LAMP, CRGB::Red);
        break;

      case YELLOW:
        fillLamp(ewYellow, NUM_LEDS_PER_LAMP, CRGB::Yellow);
        break;

      case GREEN:
        fillLamp(ewGreen, NUM_LEDS_PER_LAMP, CRGB::Green);
        break;
    }

    FastLED.show();
  }


void sendNSColorToUno(Color c) {
  const char* txt = "RED";
  if (c == YELLOW) txt = "YELLOW";
  else if (c == GREEN) txt = "GREEN";

  Serial1.print("NS:");
  Serial1.print(txt);
  Serial1.print("\n");
}

// Very minimalistic string search inside HTTP response.
// You can replace with ArduinoJson parsing if your endpoint returns pure JSON.
bool extractField(const String &json, const char *key, String &out) {
  int idx = json.indexOf(key);
  if (idx < 0) return false;

  idx = json.indexOf(":", idx);
  if (idx < 0) return false;
  idx++;

  // Skip spaces
  while (idx < json.length() && (json[idx] == ' ')) idx++;

  // Detect quoted or unquoted JSON value
  if (json[idx] == '"') {
    int end = json.indexOf('"', idx + 1);
    if (end < 0) return false;
    out = json.substring(idx + 1, end);
    return true;
  } else {
    int end = idx;
    while (end < json.length() && json[end] != ',' && json[end] != '}') end++;
    out = json.substring(idx, end);
    out.trim();
    return true;
  }
}

bool fetchStateFromSheets(RemoteState &st) {
  if (!client.connect(SCRIPT_HOST, SCRIPT_PORT)) {
    return false;
  }

  client.print(String("GET ") + SCRIPT_PATH + " HTTP/1.1\r\n" +
               "Host: " + SCRIPT_HOST + "\r\n" +
               "Connection: close\r\n\r\n");

  // Wait for headers
  unsigned long start = millis();
  while (client.connected() && !client.available()) {
    if (millis() - start > 3000) {
      client.stop();
      return false;
    }
  }

  String payload;
  bool inBody = false;

  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (!inBody) {
      if (line == "\r") inBody = true;
    } else {
      payload += line;
    }
  }

  client.stop();

  // --- Parse JSON fields ---
  String tmp;

  if (extractField(payload, "cycle_ms", tmp)) st.cycle_ms = tmp.toInt();
  if (extractField(payload, "offset_A_ms", tmp)) st.offset_A_ms = tmp.toInt();
  if (extractField(payload, "offset_B_ms", tmp)) st.offset_B_ms = tmp.toInt();
  if (extractField(payload, "blink_mode", tmp)) st.blink_mode = (tmp == "true");
  if (extractField(payload, "ped_request", tmp)) st.ped_request = (tmp == "true");
  if (extractField(payload, "base_epoch_ms", tmp)) st.base_epoch_ms = tmp.toInt();
  if (extractField(payload, "ped_active_until_ms", tmp)) st.ped_active_until_ms = tmp.toInt();

  return true;
}


void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ====== Setup & loop ======

unsigned long lastPoll = 0;
RemoteState remoteState;

void setup() {
  FastLED.addLeds<NEOPIXEL, PIN_EW_RED>(ewRed, NUM_LEDS_PER_LAMP);
  FastLED.addLeds<NEOPIXEL, PIN_EW_YELLOW>(ewYellow, NUM_LEDS_PER_LAMP);
  FastLED.addLeds<NEOPIXEL, PIN_EW_GREEN>(ewGreen, NUM_LEDS_PER_LAMP);
  setNeoPixelColor(RED);

  Serial.begin(115200);   // debug over USB
  Serial1.begin(9600);    // link to UNO

  connectWiFi();
}

void loop() {
  unsigned long now = millis();

  // Poll Sheets/backend every 500 ms
  if (now - lastPoll > 500) {
    lastPoll = now;
    RemoteState st;
    if (fetchStateFromSheets(st)) {
      remoteState = st;
    }
  }

  // Compute "view" based on remoteState and local time
  unsigned long cycle = remoteState.cycle_ms > 0 ? remoteState.cycle_ms : 26000;
  long offset = (SITE_ID == 'A') ? remoteState.offset_A_ms : remoteState.offset_B_ms;

  unsigned long base = remoteState.base_epoch_ms;   // if 0, we just use millis as relative
  unsigned long elapsed;
  if (base == 0) {
    // fallback: impressive enough for demo; won't be globally synced but okay
    elapsed = (now + cycle * 10) % cycle;
  } else {
    // if you keep base_epoch in ms in Sheets, use that with a real-time clock or millis offset
    elapsed = (now + cycle * 10) % cycle;
  }

  ViewState view;

  if (remoteState.blink_mode) {
    bool on = (now / 500) % 2 == 0;
    view.phase = PH_BLINK_YELLOW;
    view.ew = on ? YELLOW : RED;
    view.ns = on ? YELLOW : RED;
  } else if (remoteState.ped_active_until_ms != 0 && now < remoteState.ped_active_until_ms) {
    view.phase = PH_PEDESTRIAN;
    view.ew = RED;
    view.ns = GREEN;
  } else {
    view = phaseFromElapsed(elapsed, cycle);
  }

  // Drive EW NeoPixels and send NS to UNO
  setNeoPixelColor(view.ew);
  sendNSColorToUno(view.ns);

  delay(20); // small delay for stability
}
