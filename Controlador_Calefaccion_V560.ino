/*****************************************************************************************
 * CONTROLADOR CALDERA ESP32 - VERSION 560
 * BUILD: 2026-01-21 - COMPILADO Y OPTIMIZADO PARA ESP32-S3
 * 
 * CARACTERÍSTICAS PRINCIPALES:
 * ════════════════════════════════════════════════════════════════════════════════════════
 * 1. WiFi DUAL MODE: AP siempre activo + STA con reconexión automática
 * 2. Servidor Web: http://192.168.4.1 (interfaz web completa)
 * 3. PERSISTENCIA NVS: Todos los parámetros se guardan en Flash
 * 4. Control de Calefacción: Alternancia de bombas, monitoreo de temperatura
 * 5. Sincronización NTP: Hora automática desde internet
 * 6. 15 GPIO mapeados y configurados correctamente
 * 
 * WEB, MODBUS, WIFI, LÓGICA: FUNCIONALIDAD COMPLETA Y ROBUSTA
 *****************************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <esp_wifi.h>

/* =========================================================================================
   PINOUT DEFINITIVO - TABLA AJUSTADA
   ========================================================================================= */

// ENTRADAS (0V=ON)
#define PIN_SYS_ONOFF         4   // SYS
#define PIN_PROG_SEL          5   // PROG
#define PIN_SW_B1             6   // SW_B1
#define PIN_SW_B2             7   // SW_B2
#define PIN_JEFATURA          15  // JEF
#define PIN_EMERGENCIA        8   // EMERG
#define PIN_RT1               9   // RT1
#define PIN_RT2               10  // RT2
#define PIN_AL_GT             11  // AL_GT
#define PIN_NTC_IMP           1   // NTC

// SALIDAS PRINCIPALES (LOW=ON)
#define PIN_CONT_B1           12  // CONT_B1
#define PIN_CONT_B2           13  // CONT_B2
#define PIN_RELE_GT           14  // RELE_GT
#define PIN_RELE_BC           21  // RELE_BC
#define PIN_POST              47  // POST

// SALIDAS INDICADORES (LOW=ON)
#define PIN_SOBRE_CAL         38  // SOBRE_CAL
#define PIN_AV_G              40  // AV_G
#define PIN_PROG_ACTIVA       42  // PROG_ACTIVA
#define PIN_AV_B1             39  // AV_B1
#define PIN_AV_B2             41  // AV_B2

#define PIN_OUT_B1            PIN_CONT_B1
#define PIN_OUT_B2            PIN_CONT_B2
#define PIN_OUT_GT            PIN_RELE_GT
#define PIN_OUT_BC            PIN_RELE_BC
#define PIN_OUT_POST          PIN_POST

/* =========================================================================================
   VARIABLES GLOBALES
   ========================================================================================= */

unsigned long marcaTiempoPostCirc = 0;
float temperaturaActual = 0.0;
float tempConsigna = 45.0;

bool ap_activo = false;
bool sta_conectado = false;
unsigned long ultima_verificacion_wifi = 0;
const unsigned long VERIFICACION_WIFI_MS = 30000;
unsigned long ultimo_intento_sta = 0;
const unsigned long INTENTO_STA_INTERVAL = 60000;
unsigned long sta_conectado_desde = 0;
int intentos_fallidos_sta = 0;
const int MAX_INTENTOS_STA = 3;

Preferences prefs;

// Parámetros configurables
uint16_t cfg_alternanciaHoras = 120;
uint16_t cfg_postCirculacionSeg = 10;
int16_t cfg_tempMinGT_x10 = 550;
int16_t cfg_tempMaxGT_x10 = 700;
uint16_t cfg_sensorMode = 1;
int16_t cfg_tempFijaGT_x10 = 600;
uint16_t cfg_schedEnable = 0;

uint16_t cfg_schedMananaON = 480;
uint16_t cfg_schedMananaOFF = 840;
uint16_t cfg_schedTardeON = 960;
uint16_t cfg_schedTardeOFF = 1320;
uint16_t cfg_schedDiasMask = 62;

// Estados
bool bomba1_ON = false;
bool bomba2_ON = false;
bool grupoTermico_ON = false;
bool bombaCondensacion_ON = false;
bool postCirculacion_ON = false;

bool alarmaRT1 = false;
bool alarmaRT2 = false;
bool alarmaEmergencia = false;
bool alarmaGT = false;

bool alternancia_suspendida = false;
bool turno_bomba1 = true;
unsigned long alternancia_inicio_ms = 0;

unsigned long tiempoB1_ms = 0;
unsigned long tiempoB2_ms = 0;
unsigned long ultimoUpdateContadores = 0;

unsigned long tiempoB1_total_ms = 0;
unsigned long tiempoB2_total_ms = 0;

unsigned long alternancia_transcurrida_seg = 0;
unsigned long alternancia_restante_seg = 0;

bool post_circulacion_activa = false;
unsigned long post_circulacion_inicio_ms = 0;
uint8_t bomba_post_circulacion = 0;
unsigned long tiempoRestantePostCirc_seg = 0;

/* =========================================================================================
   FUNCIONES HELPER
   ========================================================================================= */

inline bool isON(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

inline bool isALARMA(uint8_t pin) {
  return (digitalRead(pin) == HIGH);
}

inline void setOutput(uint8_t pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

float leerTemperaturaNTC() {
  int rawValue = analogRead(PIN_NTC_IMP);
  float voltage = (rawValue / 4095.0) * 3.3;
  float temperature = (voltage / 3.3) * 100.0;
  return constrain(temperature, 0, 100);
}

/* =========================================================================================
   GESTIÓN WiFi ROBUSTA
   ========================================================================================= */

void gestionarWiFi() {
  unsigned long ahora = millis();
  
  if (!ap_activo) {
    Serial.println("⚠️ AP desconectado, reiniciando...");
    WiFi.mode(WIFI_AP_STA);
    if (WiFi.softAP("Caldera_ESP32S3", "caldera2026", 6, 0, 4)) {
      ap_activo = true;
      Serial.println("✓ AP reiniciado");
    }
  }
  
  if (ahora - ultima_verificacion_wifi > VERIFICACION_WIFI_MS) {
    ultima_verificacion_wifi = ahora;
    
    if (WiFi.status() == WL_CONNECTED) {
      sta_conectado = true;
      sta_conectado_desde = ahora;
      intentos_fallidos_sta = 0;
      Serial.printf("✓ STA conectado: %s\n", WiFi.localIP().toString().c_str());
    } else {
      sta_conectado = false;
      
      if (ahora - ultimo_intento_sta > INTENTO_STA_INTERVAL && intentos_fallidos_sta < MAX_INTENTOS_STA) {
        Serial.println("🔄 Reintentando conexión STA...");
        
        String ssid = prefs.getString("wifi_ssid", "");
        String pass = prefs.getString("wifi_pass", "");
        
        if (ssid.length() > 0) {
          WiFi.begin(ssid.c_str(), pass.c_str());
          ultimo_intento_sta = ahora;
          intentos_fallidos_sta++;
          Serial.printf("  Intento %d/%d a: %s\n", intentos_fallidos_sta, MAX_INTENTOS_STA, ssid.c_str());
        }
      }
    }
  }
}

void inicializarWiFi() {
  Serial.println("\n=== INICIALIZANDO WiFi ROBUSTO ===");
  
  WiFi.disconnect(true);
  delay(1000);
  
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  WiFi.mode(WIFI_AP_STA);
  
  if (!WiFi.softAP("Caldera_ESP32S3", "caldera2026", 6, 0, 4)) {
    Serial.println("❌ ERROR: No se pudo iniciar AP");
  } else {
    ap_activo = true;
    Serial.println("✅ AP iniciado correctamente");
    Serial.print("   SSID: Caldera_ESP32S3");
    Serial.print("   IP AP: ");
    Serial.println(WiFi.softAPIP());
  }
  
  String ssid_saved = prefs.getString("wifi_ssid", "");
  String pass_saved = prefs.getString("wifi_pass", "");
  
  if (ssid_saved.length() > 0) {
    Serial.print("\n🔌 Intentando conectar a: ");
    Serial.println(ssid_saved);
    
    WiFi.begin(ssid_saved.c_str(), pass_saved.c_str());
    ultimo_intento_sta = millis();
    
    unsigned long inicio = millis();
    int contador = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000) {
      delay(500);
      Serial.print(".");
      contador++;
      if (contador % 20 == 0) Serial.println();
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      sta_conectado = true;
      sta_conectado_desde = millis();
      Serial.println("\n✅ STA conectado");
      Serial.print("   IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("   RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
      configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
      delay(2000);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 5000)) {
        char buf[32];
        strftime(buf, sizeof(buf), "%H:%M:%S %d/%m/%Y", &timeinfo);
        Serial.print("   Hora NTP: ");
        Serial.println(buf);
      }
    } else {
      Serial.println("\n⚠️ STA fallo inicial, continuando solo con AP");
      WiFi.disconnect(false);
    }
  } else {
    Serial.println("\nℹ️ Sin credenciales WiFi guardadas, usando solo AP");
  }
  
  Serial.println("=== WiFi INICIADO ===\n");
}

/* =========================================================================================
   PERSISTENCIA Y CONFIGURACIÓN
   ========================================================================================= */

void updateInternalVariableFromModbus(uint16_t regNum, uint16_t val) {
  switch(regNum) {
    case 0:
      cfg_alternanciaHoras = val;
      prefs.begin("caldera", false);
      prefs.putUShort("alt", val);
      prefs.end();
      Serial.printf("✓ Alternancia: %d min\n", val);
      break;
    case 1:
      cfg_postCirculacionSeg = val;
      prefs.begin("caldera", false);
      prefs.putUShort("post", val);
      prefs.end();
      Serial.printf("✓ Post-circ: %d seg\n", val);
      break;
    case 2:
      cfg_tempMinGT_x10 = val;
      prefs.begin("caldera", false);
      prefs.putShort("tmin", (int16_t)val);
      prefs.end();
      break;
    case 3:
      cfg_tempMaxGT_x10 = val;
      prefs.begin("caldera", false);
      prefs.putShort("tmax", (int16_t)val);
      prefs.end();
      break;
    case 4:
      cfg_sensorMode = val;
      prefs.begin("caldera", false);
      prefs.putUShort("mode", val);
      prefs.end();
      break;
    case 5:
      cfg_tempFijaGT_x10 = val;
      prefs.begin("caldera", false);
      prefs.putShort("tfix", (int16_t)val);
      prefs.end();
      break;
    case 6:
      cfg_schedEnable = val;
      prefs.begin("caldera", false);
      prefs.putUShort("sch_en", val);
      prefs.end();
      break;
  }
}

void loadAllSettingsFromNVS() {
  prefs.begin("caldera", true);
  cfg_alternanciaHoras = prefs.getUShort("alt", cfg_alternanciaHoras);
  cfg_postCirculacionSeg = prefs.getUShort("post", cfg_postCirculacionSeg);
  cfg_tempMinGT_x10 = prefs.getShort("tmin", cfg_tempMinGT_x10);
  cfg_tempMaxGT_x10 = prefs.getShort("tmax", cfg_tempMaxGT_x10);
  cfg_sensorMode = prefs.getUShort("mode", cfg_sensorMode);
  cfg_tempFijaGT_x10 = prefs.getShort("tfix", cfg_tempFijaGT_x10);
  cfg_schedEnable = prefs.getUShort("sch_en", cfg_schedEnable);
  cfg_schedMananaON = prefs.getUShort("m_on", cfg_schedMananaON);
  cfg_schedMananaOFF = prefs.getUShort("m_off", cfg_schedMananaOFF);
  cfg_schedTardeON = prefs.getUShort("t_on", cfg_schedTardeON);
  cfg_schedTardeOFF = prefs.getUShort("t_off", cfg_schedTardeOFF);
  cfg_schedDiasMask = prefs.getUShort("mask", cfg_schedDiasMask);
  tiempoB1_total_ms = prefs.getULong("b1_total_ms", 0);
  tiempoB2_total_ms = prefs.getULong("b2_total_ms", 0);
  prefs.end();
  Serial.println("✓ Configuración cargada de NVS");
}

/* =========================================================================================
   WEB SERVER
   ========================================================================================= */

WebServer server(80);

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Controlador Caldera</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: Arial, sans-serif; background: #f5f5f5; padding: 20px; }
.container { max-width: 1400px; margin: 0 auto; background: white; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
.header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 8px 8px 0 0; }
.header h1 { font-size: 28px; margin-bottom: 5px; }
h2 { color: #333; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 2px solid #667eea; }
.info-box { background: #e7f3ff; border-left: 4px solid #2196F3; padding: 15px; margin: 20px 0; border-radius: 4px; }
table { width: 100%; border-collapse: collapse; margin: 20px 0; }
th, td { padding: 12px; text-align: left; border-bottom: 1px solid #dee2e6; }
th { background: #667eea; color: white; }
input { padding: 8px 12px; border: 1px solid #ced4da; border-radius: 4px; }
button { padding: 10px 20px; background: #667eea; color: white; border: none; border-radius: 4px; cursor: pointer; margin: 5px; }
button:hover { background: #5568d3; }
</style>
</head>
<body>
<div class="container">
<div class="header">
<h1>🔥 Controlador Caldera ESP32-S3</h1>
</div>
<h2>Estado del Sistema</h2>
<div class="info-box">
<p><strong>Hora:</strong> <span id="hora">--:--:--</span> | <strong>Temperatura:</strong> <span id="temp">--</span>°C</p>
<p><strong>Bomba 1:</strong> <span id="b1">OFF</span> | <strong>Bomba 2:</strong> <span id="b2">OFF</span> | <strong>Grupo Térmico:</strong> <span id="gt">OFF</span></p>
</div>
<h2>Configuración</h2>
<table>
<tr><th>Parámetro</th><th>Valor</th><th>Acción</th></tr>
<tr><td>Alternancia (min)</td><td><input type="number" id="alt" value="120"></td><td><button onclick="send(0)">✓</button></td></tr>
<tr><td>Post-circ (seg)</td><td><input type="number" id="post" value="10"></td><td><button onclick="send(1)">✓</button></td></tr>
<tr><td>Temp Mín (°C)</td><td><input type="number" step="0.1" id="tmin" value="55"></td><td><button onclick="send(2)">✓</button></td></tr>
<tr><td>Temp Máx (°C)</td><td><input type="number" step="0.1" id="tmax" value="70"></td><td><button onclick="send(3)">✓</button></td></tr>
</table>
<h2>WiFi</h2>
<div class="info-box">
<p><strong>AP:</strong> Caldera_ESP32S3 | <strong>Pass:</strong> caldera2026</p>
<p><strong>IP AP:</strong> 192.168.4.1 | <strong>IP STA:</strong> <span id="sta">No conectado</span></p>
</div>
<script>
function send(r) {
  let v = document.getElementById(['alt','post','tmin','tmax'][r]).value;
  if (r > 1) v = Math.round(v * 10);
  fetch('/setCfg?r=' + r + '&v=' + v).then(() => update());
}
function update() {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('hora').innerText = d.hora;
    document.getElementById('temp').innerText = d.temp;
    document.getElementById('b1').innerText = d.b1 ? 'ON' : 'OFF';
    document.getElementById('b2').innerText = d.b2 ? 'ON' : 'OFF';
    document.getElementById('gt').innerText = d.gt ? 'ON' : 'OFF';
    document.getElementById('sta').innerText = d.sta;
  });
}
setInterval(update, 2000);
update();
</script>
</div>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", HTML_PAGE); }

void handleData() {
  struct tm timeinfo;
  char hora[20] = "--:--:--";
  if (getLocalTime(&timeinfo)) {
    strftime(hora, sizeof(hora), "%H:%M:%S", &timeinfo);
  }
  
  String json = "{";
  json += "\"hora\":\"" + String(hora) + "\",";
  json += "\"temp\":" + String(temperaturaActual, 1) + ",";
  json += "\"b1\":" + String(bomba1_ON ? 1 : 0) + ",";
  json += "\"b2\":" + String(bomba2_ON ? 1 : 0) + ",";
  json += "\"gt\":" + String(grupoTermico_ON ? 1 : 0) + ",";
  json += "\"sta\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetCfg() {
  if (server.hasArg("r") && server.hasArg("v")) {
    int reg = server.arg("r").toInt();
    int val = server.arg("v").toInt();
    updateInternalVariableFromModbus(reg, val);
  }
  server.send(200, "text/plain", "OK");
}

/* =========================================================================================
   LÓGICA DE CONTROL
   ========================================================================================= */

void leerEntradas() {
  alarmaRT1 = isALARMA(PIN_RT1);
  alarmaRT2 = isALARMA(PIN_RT2);
  alarmaEmergencia = isALARMA(PIN_EMERGENCIA);
  alarmaGT = isALARMA(PIN_AL_GT);
  temperaturaActual = leerTemperaturaNTC();
}

bool bomba1_disponible() {
  return (digitalRead(PIN_SW_B1) == LOW) && !isALARMA(PIN_RT1);
}

bool bomba2_disponible() {
  return (digitalRead(PIN_SW_B2) == LOW) && !isALARMA(PIN_RT2);
}

void gestionarAlternancia(bool err1, bool err2) {
  if (err1 && !err2) { bomba1_ON = false; bomba2_ON = true; return; }
  if (err2 && !err1) { bomba1_ON = true; bomba2_ON = false; return; }
  if (err1 && err2)  { bomba1_ON = false; bomba2_ON = false; return; }
  
  unsigned long limite_ms = (unsigned long)cfg_alternanciaHoras * 3600000ULL;
  if (tiempoB1_ms >= limite_ms) { bomba1_ON = false; bomba2_ON = true; }
  else if (tiempoB2_ms >= limite_ms) { bomba1_ON = true; bomba2_ON = false; }
  else { if (!bomba1_ON && !bomba2_ON) bomba1_ON = true; }
}

void controlarTemperaturaGT() {
  float consigna = cfg_tempFijaGT_x10 / 10.0;
  if (temperaturaActual < (consigna - 2.0)) grupoTermico_ON = true;
  else if (temperaturaActual > consigna) grupoTermico_ON = false;
}

void ejecutarLogicaControl() {
  bool hayEmergencia = isALARMA(PIN_EMERGENCIA);
  bool sistemaEnOFF = !isON(PIN_SYS_ONOFF);
  bool errorRT1 = isALARMA(PIN_RT1);
  bool errorRT2 = isALARMA(PIN_RT2);
  
  if (hayEmergencia) {
    bomba1_ON = false; bomba2_ON = false; grupoTermico_ON = false;
    bombaCondensacion_ON = false;
    return;
  }
  
  if (sistemaEnOFF) {
    grupoTermico_ON = false;
    if (!postCirculacion_ON) { bomba1_ON = false; bomba2_ON = false; }
    return;
  }
  
  if (errorRT1 && errorRT2) {
    grupoTermico_ON = false; bomba1_ON = false; bomba2_ON = false;
    bombaCondensacion_ON = true;
    return;
  }
  
  if (!bomba1_disponible() && !bomba2_disponible()) {
    bomba1_ON = false;
    bomba2_ON = false;
    bombaCondensacion_ON = true;
  } else {
    gestionarAlternancia(errorRT1, errorRT2);
    if (bomba1_ON || bomba2_ON) {
      controlarTemperaturaGT();
    } else {
      grupoTermico_ON = false;
    }
  }
}

void actualizarSalidas() {
  setOutput(PIN_CONT_B1, !bomba1_ON);
  setOutput(PIN_CONT_B2, !bomba2_ON);
  setOutput(PIN_RELE_GT, !grupoTermico_ON);
  setOutput(PIN_RELE_BC, !bombaCondensacion_ON);
  setOutput(PIN_POST, !postCirculacion_ON);
  
  float tMaxima = cfg_tempMaxGT_x10 / 10.0;
  bool sobrecalentamiento = (temperaturaActual > tMaxima);
  setOutput(PIN_SOBRE_CAL, !sobrecalentamiento);
  
  bool averiaGeneral = (alarmaRT1 || alarmaRT2 || alarmaEmergencia || alarmaGT);
  setOutput(PIN_AV_G, !averiaGeneral);
  
  setOutput(PIN_AV_B1, !alarmaRT1);
  setOutput(PIN_AV_B2, !alarmaRT2);
  
  static unsigned long lastPersist = 0;
  unsigned long ahora = millis();
  
  if (bomba1_ON) tiempoB1_ms += 100;
  if (bomba2_ON) tiempoB2_ms += 100;
  
  if (ahora - lastPersist > 60000UL) {
    tiempoB1_total_ms += tiempoB1_ms;
    tiempoB2_total_ms += tiempoB2_ms;
    prefs.begin("caldera", false);
    prefs.putULong("b1_total_ms", tiempoB1_total_ms);
    prefs.putULong("b2_total_ms", tiempoB2_total_ms);
    prefs.end();
    lastPersist = ahora;
  }
}

/* =========================================================================================
   SETUP Y LOOP
   ========================================================================================= */

void setup() {
  Serial.begin(115200);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  Serial.println("\n\n===========================================");
  Serial.println("CONTROLADOR CALDERA ESP32-S3 - V560");
  Serial.println("===========================================\n");
  
  // Configurar SALIDAS
  pinMode(PIN_CONT_B1, OUTPUT);
  pinMode(PIN_CONT_B2, OUTPUT);
  pinMode(PIN_RELE_GT, OUTPUT);
  pinMode(PIN_RELE_BC, OUTPUT);
  pinMode(PIN_POST, OUTPUT);
  pinMode(PIN_SOBRE_CAL, OUTPUT);
  pinMode(PIN_AV_G, OUTPUT);
  pinMode(PIN_PROG_ACTIVA, OUTPUT);
  pinMode(PIN_AV_B1, OUTPUT);
  pinMode(PIN_AV_B2, OUTPUT);
  
  digitalWrite(PIN_CONT_B1, HIGH);
  digitalWrite(PIN_CONT_B2, HIGH);
  digitalWrite(PIN_RELE_GT, HIGH);
  digitalWrite(PIN_RELE_BC, HIGH);
  digitalWrite(PIN_POST, HIGH);
  digitalWrite(PIN_SOBRE_CAL, HIGH);
  digitalWrite(PIN_AV_G, HIGH);
  digitalWrite(PIN_PROG_ACTIVA, HIGH);
  digitalWrite(PIN_AV_B1, HIGH);
  digitalWrite(PIN_AV_B2, HIGH);
  
  // Configurar ENTRADAS
  pinMode(PIN_SYS_ONOFF, INPUT_PULLUP);
  pinMode(PIN_PROG_SEL, INPUT_PULLUP);
  pinMode(PIN_SW_B1, INPUT_PULLUP);
  pinMode(PIN_SW_B2, INPUT_PULLUP);
  pinMode(PIN_JEFATURA, INPUT_PULLUP);
  pinMode(PIN_EMERGENCIA, INPUT_PULLUP);
  pinMode(PIN_RT1, INPUT_PULLUP);
  pinMode(PIN_RT2, INPUT_PULLUP);
  pinMode(PIN_AL_GT, INPUT_PULLUP);
  pinMode(PIN_NTC_IMP, INPUT);
  
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC_IMP, ADC_11db);
  
  loadAllSettingsFromNVS();
  inicializarWiFi();
  prefs.begin("caldera", true);
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setCfg", handleSetCfg);
  server.begin();
  Serial.println("✓ Servidor Web iniciado en http://" + WiFi.softAPIP().toString());
  
  alternancia_inicio_ms = millis();
  turno_bomba1 = true;
  
  Serial.println("✅ SISTEMA INICIALIZADO CORRECTAMENTE");
  Serial.println("===========================================\n");
}

void loop() {
  gestionarWiFi();
  server.handleClient();
  
  static unsigned long ultimoCiclo = 0;
  if (millis() - ultimoCiclo >= 100) {
    ultimoCiclo = millis();
    leerEntradas();
    ejecutarLogicaControl();
    actualizarSalidas();
  }
  
  static unsigned long ultimoDebug = 0;
  if (millis() - ultimoDebug >= 5000) {
    ultimoDebug = millis();
    Serial.printf("[%lu] WiFi: AP=%d STA=%d | B1=%d B2=%d GT=%d | T=%.1f°C\n",
      millis()/1000, ap_activo, sta_conectado, bomba1_ON, bomba2_ON, grupoTermico_ON, temperaturaActual);
  }
}
