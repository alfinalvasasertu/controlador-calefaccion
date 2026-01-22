/*
  Controlador de Calefacción ESP32-S3 - V607_GitHub
  
  HARDWARE: ESP32-S3 Dev Module con PSRAM
  LIBRERÍA MODBUS: modbus-esp8266 v4.1.0
  
  === MAPEO DE PINES ===
  ENTRADAS DIGITALES (0V = ON):
  - PIN 4:  GT (Generador de Temperatura)
  - PIN 5:  GR (Generador Reserva)
  - PIN 6:  GE (Generador Emergencia)
  - PIN 7:  SISTEMA (ON/OFF general)
  - PIN 8:  JEFATURA (Mando prioritario)
  - PIN 9:  PROG (Programación horaria)
  - PIN 10: VERIF_B1 (Verificación Bomba 1)
  - PIN 11: VERIF_B2 (Verificación Bomba 2)
  - PIN 15: NIVEL_OK
  - PIN 16: TERMOSTATO_ACS
  - PIN 17: TERMOSTATO_AMB
  - PIN 18: MARCHA (ON/OFF físico)
  
  ENTRADAS DE ALARMA (3.3V = ALARMA):
  - PIN 12: ALARMA_1 (Generador)
  - PIN 13: ALARMA_2 (Bomba 1)
  - PIN 14: ALARMA_3 (Bomba 2)
  - PIN 21: ALARMA_4 (General)
  
  ENTRADA ANALÓGICA:
  - PIN 1 (ADC1_CH0): NTC (Sensor temperatura)
  
  SALIDAS RELÉ (LOW = ON):
  - PIN 47: BOMBA_1
  - PIN 38: BOMBA_2
  - PIN 39: VÁLVULA_ACS
  - PIN 40: VÁLVULA_AMB
  - PIN 41: CONTACTO_AUX
  
  SALIDAS LED:
  - PIN 42: LED_MARCHA
  - PIN 43: LED_ALARMA
  - PIN 44: LED_WIFI
  - PIN 45: LED_STATUS_1
  - PIN 46: LED_STATUS_2
  
  === REGISTROS MODBUS TCP ===
  40001-40250 (Holding Registers)
  - Temperaturas en formato x10 (ej: 70°C = 700)
  - Configuraciones
  - Estados
  
  === WiFi ===
  - Modo AP: SSID "Caldera_ESP32S3", IP 192.168.4.1
  - Modo STA: Configurable, DHCP
  - Servidor Web: Puerto 80
  - Servidor Modbus TCP: Puerto 502
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <ModbusIP_ESP8266.h>

// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================

// Entradas digitales (0V = ON)
#define PIN_GT              4
#define PIN_GR              5
#define PIN_GE              6
#define PIN_SISTEMA         7
#define PIN_JEFATURA        8
#define PIN_PROG            9
#define PIN_VERIF_B1        10
#define PIN_VERIF_B2        11
#define PIN_NIVEL_OK        15
#define PIN_TERMOSTATO_ACS  16
#define PIN_TERMOSTATO_AMB  17
#define PIN_MARCHA          18

// Entradas de alarma (3.3V = ALARMA)
#define PIN_ALARMA_1        12
#define PIN_ALARMA_2        13
#define PIN_ALARMA_3        14
#define PIN_ALARMA_4        21

// Entrada analógica (NTC)
#define PIN_NTC             1

// Salidas relé (LOW = ON)
#define PIN_BOMBA_1         47
#define PIN_BOMBA_2         38
#define PIN_VALVULA_ACS     39
#define PIN_VALVULA_AMB     40
#define PIN_CONTACTO_AUX    41

// Salidas LED
#define PIN_LED_MARCHA      42
#define PIN_LED_ALARMA      43
#define PIN_LED_WIFI        44
#define PIN_LED_STATUS_1    45
#define PIN_LED_STATUS_2    46

// ============================================================================
// VARIABLES GLOBALES - ESTADOS DE ENTRADAS
// ============================================================================

// Estados actuales
bool estado_GT = false;
bool estado_GR = false;
bool estado_GE = false;
bool estado_SISTEMA = false;
bool estado_JEFATURA = false;
bool estado_PROG = false;
bool estado_VERIF_B1 = false;
bool estado_VERIF_B2 = false;
bool estado_NIVEL_OK = false;
bool estado_TERMOSTATO_ACS = false;
bool estado_TERMOSTATO_AMB = false;
bool estado_MARCHA = false;

// Alarmas
bool alarma_1 = false;
bool alarma_2 = false;
bool alarma_3 = false;
bool alarma_4 = false;

// Estados anteriores para detección de flancos
bool prev_GT = false;
bool prev_SISTEMA = false;
bool prev_JEFATURA = false;
bool prev_VERIF_B1 = false;
bool prev_VERIF_B2 = false;

// ============================================================================
// VARIABLES GLOBALES - ESTADOS DE SALIDAS
// ============================================================================

bool salida_BOMBA_1 = false;
bool salida_BOMBA_2 = false;
bool salida_VALVULA_ACS = false;
bool salida_VALVULA_AMB = false;
bool salida_CONTACTO_AUX = false;

// ============================================================================
// VARIABLES GLOBALES - LÓGICA DE CONTROL
// ============================================================================

// Post-circulación
bool postCirculacion_activa = false;
unsigned long postCirculacion_inicio = 0;
unsigned int postCirculacion_duracion = 180; // segundos (40002)
enum MotivoPostCirc { NINGUNO, GT_OFF, SISTEMA_OFF, JEFATURA_OFF, DOBLE_AVERIA };
MotivoPostCirc motivoPostCirc = NINGUNO;

// Alternancia de bombas
bool alternancia_activa = true;
bool bomba_principal_es_B1 = true;
unsigned long alternancia_ultimo_cambio = 0;
unsigned int alternancia_intervalo = 168; // horas (40001)
bool alternancia_pausada = false;

// Bloqueo post Jefatura
bool bloqueo_post_jefatura = false;

// Temperatura
float temperatura_actual = 0.0;

// ============================================================================
// VARIABLES GLOBALES - CONFIGURACIÓN MODBUS
// ============================================================================

#define MODBUS_BASE_ADDRESS 40001
#define NUM_MODBUS_REGISTERS 250

uint16_t modbusRegisters[NUM_MODBUS_REGISTERS];

// ============================================================================
// VARIABLES GLOBALES - WiFi Y WEB
// ============================================================================

WebServer server(80);
Preferences preferences;

// Configuración WiFi
String wifi_ssid_sta = "";
String wifi_password_sta = "";
String wifi_ssid_ap = "Caldera_ESP32S3";
String wifi_password_ap = "12345678";
bool wifi_modo_sta = false;

// NTP
bool ntp_auto = false;
String ntp_server = "pool.ntp.org";
long gmt_offset_sec = 3600;  // GMT+1
int daylight_offset_sec = 3600;

// ============================================================================
// VARIABLES GLOBALES - SERVIDOR MODBUS TCP
// ============================================================================

ModbusIP mb;  // Instancia ModbusIP

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Controlador Calefacción V607_GitHub ===");
  Serial.println("ESP32-S3 con ArduinoModbus");
  
  // Inicializar NVS
  preferences.begin("caldera", false);
  cargarConfiguracion();
  
  // Configurar pines de entrada
  pinMode(PIN_GT, INPUT_PULLUP);
  pinMode(PIN_GR, INPUT_PULLUP);
  pinMode(PIN_GE, INPUT_PULLUP);
  pinMode(PIN_SISTEMA, INPUT_PULLUP);
  pinMode(PIN_JEFATURA, INPUT_PULLUP);
  pinMode(PIN_PROG, INPUT_PULLUP);
  pinMode(PIN_VERIF_B1, INPUT_PULLUP);
  pinMode(PIN_VERIF_B2, INPUT_PULLUP);
  pinMode(PIN_NIVEL_OK, INPUT_PULLUP);
  pinMode(PIN_TERMOSTATO_ACS, INPUT_PULLUP);
  pinMode(PIN_TERMOSTATO_AMB, INPUT_PULLUP);
  pinMode(PIN_MARCHA, INPUT_PULLUP);
  
  // Pines de alarma (sin pull-up, entrada directa)
  pinMode(PIN_ALARMA_1, INPUT);
  pinMode(PIN_ALARMA_2, INPUT);
  pinMode(PIN_ALARMA_3, INPUT);
  pinMode(PIN_ALARMA_4, INPUT);
  
  // Configurar pines de salida (iniciar en OFF = HIGH)
  pinMode(PIN_BOMBA_1, OUTPUT);
  pinMode(PIN_BOMBA_2, OUTPUT);
  pinMode(PIN_VALVULA_ACS, OUTPUT);
  pinMode(PIN_VALVULA_AMB, OUTPUT);
  pinMode(PIN_CONTACTO_AUX, OUTPUT);
  
  digitalWrite(PIN_BOMBA_1, HIGH);
  digitalWrite(PIN_BOMBA_2, HIGH);
  digitalWrite(PIN_VALVULA_ACS, HIGH);
  digitalWrite(PIN_VALVULA_AMB, HIGH);
  digitalWrite(PIN_CONTACTO_AUX, HIGH);
  
  // Configurar LEDs
  pinMode(PIN_LED_MARCHA, OUTPUT);
  pinMode(PIN_LED_ALARMA, OUTPUT);
  pinMode(PIN_LED_WIFI, OUTPUT);
  pinMode(PIN_LED_STATUS_1, OUTPUT);
  pinMode(PIN_LED_STATUS_2, OUTPUT);
  
  digitalWrite(PIN_LED_MARCHA, LOW);
  digitalWrite(PIN_LED_ALARMA, LOW);
  digitalWrite(PIN_LED_WIFI, LOW);
  digitalWrite(PIN_LED_STATUS_1, LOW);
  digitalWrite(PIN_LED_STATUS_2, LOW);
  
  // Inicializar registros Modbus
  inicializarModbus();
  
  // Inicializar WiFi
  inicializarWiFi();
  
  // Inicializar Servidor Web
  inicializarServidor();
  
  // Inicializar Modbus TCP
  mb.server();
  Serial.println("Servidor Modbus TCP iniciado en puerto 502");
  
  // Configurar registros Holding (40001-40250)
  for (int i = 0; i < NUM_MODBUS_REGISTERS; i++) {
    mb.addHreg(MODBUS_BASE_ADDRESS + i, 0);
  }
  
  // Sincronizar hora NTP si está habilitado
  if (ntp_auto) {
    sincronizarNTP();
  }
  
  Serial.println("=== Setup completado ===\n");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  // Leer entradas físicas
  leerEntradas();
  
  // Detectar flancos y eventos
  detectarFlancos();
  
  // Ejecutar lógica de control
  ejecutarLogicaControl();
  
  // Actualizar salidas físicas
  actualizarSalidas();
  
  // Actualizar LEDs
  actualizarLEDs();
  
  // Actualizar registros Modbus
  actualizarRegistrosModbus();
  
  // Procesar servidor Modbus TCP
  mb.task();
  
  // Aplicar cambios desde Modbus a variables
  aplicarCambiosModbus();
  
  // Servidor Web
  server.handleClient();
  
  // Actualizar NTP si está en auto
  if (ntp_auto) {
    static unsigned long ultimo_ntp = 0;
    if (millis() - ultimo_ntp > 3600000) { // cada hora
      sincronizarNTP();
      ultimo_ntp = millis();
    }
  }
  
  delay(100); // Ciclo de 100ms
}

// ============================================================================
// FUNCIONES - LEER ENTRADAS
// ============================================================================

void leerEntradas() {
  // Entradas digitales (lógica invertida: 0V = ON)
  estado_GT = !digitalRead(PIN_GT);
  estado_GR = !digitalRead(PIN_GR);
  estado_GE = !digitalRead(PIN_GE);
  estado_SISTEMA = !digitalRead(PIN_SISTEMA);
  estado_JEFATURA = !digitalRead(PIN_JEFATURA);
  estado_PROG = !digitalRead(PIN_PROG);
  estado_VERIF_B1 = !digitalRead(PIN_VERIF_B1);
  estado_VERIF_B2 = !digitalRead(PIN_VERIF_B2);
  estado_NIVEL_OK = !digitalRead(PIN_NIVEL_OK);
  estado_TERMOSTATO_ACS = !digitalRead(PIN_TERMOSTATO_ACS);
  estado_TERMOSTATO_AMB = !digitalRead(PIN_TERMOSTATO_AMB);
  estado_MARCHA = !digitalRead(PIN_MARCHA);
  
  // Alarmas (lógica directa: 3.3V = ALARMA)
  alarma_1 = digitalRead(PIN_ALARMA_1);
  alarma_2 = digitalRead(PIN_ALARMA_2);
  alarma_3 = digitalRead(PIN_ALARMA_3);
  alarma_4 = digitalRead(PIN_ALARMA_4);
  
  // Leer temperatura NTC
  leerTemperatura();
}

void leerTemperatura() {
  int adc_value = analogRead(PIN_NTC);
  // Conversión simplificada NTC 10k
  // TODO: Implementar tabla de conversión precisa
  float voltage = (adc_value / 4095.0) * 3.3;
  float resistance = 10000.0 * voltage / (3.3 - voltage);
  
  // Ecuación Steinhart-Hart simplificada
  float temp_k = 1.0 / (0.001129148 + (0.000234125 * log(resistance)) + (0.0000000876741 * pow(log(resistance), 3)));
  temperatura_actual = temp_k - 273.15;
  
  // Limitar rango
  if (temperatura_actual < -50) temperatura_actual = -50;
  if (temperatura_actual > 150) temperatura_actual = 150;
}

// ============================================================================
// FUNCIONES - DETECTAR FLANCOS
// ============================================================================

void detectarFlancos() {
  // Flanco descendente GT (GT pasa de ON a OFF)
  if (prev_GT && !estado_GT) {
    Serial.println("[FLANCO] GT: ON -> OFF");
    // Si hay demanda activa, iniciar post-circulación
    if ((estado_TERMOSTATO_ACS || estado_TERMOSTATO_AMB) && !postCirculacion_activa) {
      iniciarPostCirculacion(GT_OFF);
    }
  }
  
  // Flanco descendente SISTEMA (SISTEMA pasa de ON a OFF)
  if (prev_SISTEMA && !estado_SISTEMA) {
    Serial.println("[FLANCO] SISTEMA: ON -> OFF");
    if (!postCirculacion_activa) {
      iniciarPostCirculacion(SISTEMA_OFF);
    }
  }
  
  // Flanco descendente JEFATURA (JEFATURA pasa de ON a OFF)
  if (prev_JEFATURA && !estado_JEFATURA) {
    Serial.println("[FLANCO] JEFATURA: ON -> OFF");
    if (!postCirculacion_activa) {
      iniciarPostCirculacion(JEFATURA_OFF);
      // Activar bloqueo
      bloqueo_post_jefatura = true;
    }
  }
  
  // Flanco ascendente JEFATURA (JEFATURA pasa de OFF a ON)
  if (!prev_JEFATURA && estado_JEFATURA) {
    Serial.println("[FLANCO] JEFATURA: OFF -> ON");
    // Desactivar bloqueo
    bloqueo_post_jefatura = false;
  }
  
  // Detectar doble avería (bomba activa con verificación OFF)
  if (salida_BOMBA_1 && !estado_VERIF_B1 && prev_VERIF_B1) {
    Serial.println("[FLANCO] Avería Bomba 1 detectada");
    if (!postCirculacion_activa) {
      iniciarPostCirculacion(DOBLE_AVERIA);
    }
  }
  
  if (salida_BOMBA_2 && !estado_VERIF_B2 && prev_VERIF_B2) {
    Serial.println("[FLANCO] Avería Bomba 2 detectada");
    if (!postCirculacion_activa) {
      iniciarPostCirculacion(DOBLE_AVERIA);
    }
  }
  
  // Actualizar estados previos
  prev_GT = estado_GT;
  prev_SISTEMA = estado_SISTEMA;
  prev_JEFATURA = estado_JEFATURA;
  prev_VERIF_B1 = estado_VERIF_B1;
  prev_VERIF_B2 = estado_VERIF_B2;
}

// ============================================================================
// FUNCIONES - POST-CIRCULACIÓN
// ============================================================================

void iniciarPostCirculacion(MotivoPostCirc motivo) {
  postCirculacion_activa = true;
  postCirculacion_inicio = millis();
  motivoPostCirc = motivo;
  
  // Pausar alternancia durante post-circulación
  alternancia_pausada = true;
  
  Serial.print("[POST-CIRC] Iniciada por motivo: ");
  switch(motivo) {
    case GT_OFF: Serial.println("GT_OFF"); break;
    case SISTEMA_OFF: Serial.println("SISTEMA_OFF"); break;
    case JEFATURA_OFF: Serial.println("JEFATURA_OFF"); break;
    case DOBLE_AVERIA: Serial.println("DOBLE_AVERIA"); break;
    default: Serial.println("DESCONOCIDO"); break;
  }
}

void verificarPostCirculacion() {
  if (!postCirculacion_activa) return;
  
  unsigned long tiempo_transcurrido = (millis() - postCirculacion_inicio) / 1000;
  
  // Verificar si se cumplió el tiempo
  if (tiempo_transcurrido >= postCirculacion_duracion) {
    Serial.println("[POST-CIRC] Finalizada por tiempo");
    finalizarPostCirculacion();
    return;
  }
  
  // Cancelación si SISTEMA o JEFATURA se activan (excepto si post-circ fue por JEFATURA_OFF)
  if ((estado_SISTEMA || estado_JEFATURA) && motivoPostCirc != JEFATURA_OFF) {
    Serial.println("[POST-CIRC] Cancelada por activación SISTEMA/JEFATURA");
    finalizarPostCirculacion();
    return;
  }
  
  // Si GT vuelve a ON, cancelar post-circulación
  if (estado_GT && motivoPostCirc == GT_OFF) {
    Serial.println("[POST-CIRC] Cancelada por GT ON");
    finalizarPostCirculacion();
    return;
  }
}

void finalizarPostCirculacion() {
  postCirculacion_activa = false;
  motivoPostCirc = NINGUNO;
  
  // Reanudar alternancia
  alternancia_pausada = false;
  
  Serial.println("[POST-CIRC] Finalizada");
}

// ============================================================================
// FUNCIONES - ALTERNANCIA DE BOMBAS
// ============================================================================

void verificarAlternancia() {
  if (!alternancia_activa || alternancia_pausada) return;
  
  unsigned long tiempo_transcurrido = (millis() - alternancia_ultimo_cambio) / 1000 / 3600; // horas
  
  if (tiempo_transcurrido >= alternancia_intervalo) {
    // Cambiar bomba principal
    bomba_principal_es_B1 = !bomba_principal_es_B1;
    alternancia_ultimo_cambio = millis();
    
    Serial.print("[ALTERNANCIA] Cambio a bomba principal: ");
    Serial.println(bomba_principal_es_B1 ? "B1" : "B2");
  }
}

// ============================================================================
// FUNCIONES - LÓGICA DE CONTROL PRINCIPAL
// ============================================================================

void ejecutarLogicaControl() {
  // Verificar post-circulación
  verificarPostCirculacion();
  
  // Verificar alternancia
  verificarAlternancia();
  
  // Inicializar todas las salidas a OFF
  salida_BOMBA_1 = false;
  salida_BOMBA_2 = false;
  salida_VALVULA_ACS = false;
  salida_VALVULA_AMB = false;
  salida_CONTACTO_AUX = false;
  
  // === NUEVA LÓGICA V607 ===
  // 1. Si está en post-circulación, mantener bombas según configuración
  if (postCirculacion_activa) {
    salida_BOMBA_1 = bomba_principal_es_B1;
    salida_BOMBA_2 = !bomba_principal_es_B1;
    // Válvulas abiertas según demanda
    salida_VALVULA_ACS = estado_TERMOSTATO_ACS;
    salida_VALVULA_AMB = estado_TERMOSTATO_AMB;
    return;
  }
  
  // 2. Si hay bloqueo post-Jefatura, no activar nada
  if (bloqueo_post_jefatura) {
    Serial.println("[LÓGICA] Sistema bloqueado post-Jefatura");
    return;
  }
  
  // 3. Prioridad máxima: JEFATURA
  if (estado_JEFATURA && estado_MARCHA && estado_NIVEL_OK) {
    Serial.println("[LÓGICA] Modo JEFATURA activo");
    salida_BOMBA_1 = bomba_principal_es_B1 && estado_VERIF_B1;
    salida_BOMBA_2 = !bomba_principal_es_B1 && estado_VERIF_B2;
    salida_VALVULA_ACS = estado_TERMOSTATO_ACS;
    salida_VALVULA_AMB = estado_TERMOSTATO_AMB;
    salida_CONTACTO_AUX = true;
    return;
  }
  
  // 4. Prioridad media: SISTEMA
  if (estado_SISTEMA && estado_MARCHA && estado_NIVEL_OK) {
    Serial.println("[LÓGICA] Modo SISTEMA activo");
    
    // Si GT está ON, activar bombas
    if (estado_GT) {
      salida_BOMBA_1 = bomba_principal_es_B1 && estado_VERIF_B1;
      salida_BOMBA_2 = !bomba_principal_es_B1 && estado_VERIF_B2;
      salida_VALVULA_ACS = estado_TERMOSTATO_ACS;
      salida_VALVULA_AMB = estado_TERMOSTATO_AMB;
      salida_CONTACTO_AUX = true;
    }
    // Si GT está OFF pero hay demanda, iniciar post-circulación ya manejada arriba
    
    return;
  }
  
  // 5. Si no hay SISTEMA ni JEFATURA, todo OFF
  Serial.println("[LÓGICA] Sin activación (SISTEMA y JEFATURA OFF)");
}

// ============================================================================
// FUNCIONES - ACTUALIZAR SALIDAS FÍSICAS
// ============================================================================

void actualizarSalidas() {
  // Aplicar lógica invertida: LOW = ON
  digitalWrite(PIN_BOMBA_1, salida_BOMBA_1 ? LOW : HIGH);
  digitalWrite(PIN_BOMBA_2, salida_BOMBA_2 ? LOW : HIGH);
  digitalWrite(PIN_VALVULA_ACS, salida_VALVULA_ACS ? LOW : HIGH);
  digitalWrite(PIN_VALVULA_AMB, salida_VALVULA_AMB ? LOW : HIGH);
  digitalWrite(PIN_CONTACTO_AUX, salida_CONTACTO_AUX ? LOW : HIGH);
}

void actualizarLEDs() {
  // LED Marcha: ON si MARCHA activo
  digitalWrite(PIN_LED_MARCHA, estado_MARCHA ? HIGH : LOW);
  
  // LED Alarma: ON si hay cualquier alarma
  bool hay_alarma = alarma_1 || alarma_2 || alarma_3 || alarma_4;
  digitalWrite(PIN_LED_ALARMA, hay_alarma ? HIGH : LOW);
  
  // LED WiFi: ON si está conectado
  bool wifi_conectado = (WiFi.status() == WL_CONNECTED) || (WiFi.softAPgetStationNum() > 0);
  digitalWrite(PIN_LED_WIFI, wifi_conectado ? HIGH : LOW);
  
  // LED Status 1: Bomba 1 activa
  digitalWrite(PIN_LED_STATUS_1, salida_BOMBA_1 ? HIGH : LOW);
  
  // LED Status 2: Bomba 2 activa
  digitalWrite(PIN_LED_STATUS_2, salida_BOMBA_2 ? HIGH : LOW);
}

// ============================================================================
// FUNCIONES - MODBUS
// ============================================================================

void inicializarModbus() {
  // Inicializar todos los registros a 0
  for (int i = 0; i < NUM_MODBUS_REGISTERS; i++) {
    modbusRegisters[i] = 0;
  }
  
  // Cargar valores desde NVS
  alternancia_intervalo = preferences.getUInt("alt_int", 168);
  postCirculacion_duracion = preferences.getUInt("post_dur", 180);
  
  // Asignar a registros Modbus (índices base 0)
  modbusRegisters[0] = alternancia_intervalo;      // 40001
  modbusRegisters[1] = postCirculacion_duracion;   // 40002
  
  Serial.println("[MODBUS] Registros inicializados");
}

void actualizarRegistrosModbus() {
  // Escribir estados al servidor Modbus TCP
  // Temperatura (x10)
  mb.Hreg(MODBUS_BASE_ADDRESS + 10, (int)(temperatura_actual * 10));
  
  // Estados de entradas (registros 40011-40025)
  mb.Hreg(MODBUS_BASE_ADDRESS + 11, estado_GT ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 12, estado_GR ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 13, estado_GE ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 14, estado_SISTEMA ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 15, estado_JEFATURA ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 16, estado_PROG ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 17, estado_VERIF_B1 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 18, estado_VERIF_B2 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 19, estado_NIVEL_OK ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 20, estado_TERMOSTATO_ACS ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 21, estado_TERMOSTATO_AMB ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 22, estado_MARCHA ? 1 : 0);
  
  // Alarmas (registros 40026-40029)
  mb.Hreg(MODBUS_BASE_ADDRESS + 25, alarma_1 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 26, alarma_2 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 27, alarma_3 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 28, alarma_4 ? 1 : 0);
  
  // Estados de salidas (registros 40030-40034)
  mb.Hreg(MODBUS_BASE_ADDRESS + 29, salida_BOMBA_1 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 30, salida_BOMBA_2 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 31, salida_VALVULA_ACS ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 32, salida_VALVULA_AMB ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 33, salida_CONTACTO_AUX ? 1 : 0);
  
  // Post-circulación (registros 40040-40042)
  mb.Hreg(MODBUS_BASE_ADDRESS + 39, postCirculacion_activa ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 40, (uint16_t)motivoPostCirc);
  
  // Alternancia (registros 40050-40052)
  mb.Hreg(MODBUS_BASE_ADDRESS + 49, alternancia_activa ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 50, bomba_principal_es_B1 ? 1 : 0);
  mb.Hreg(MODBUS_BASE_ADDRESS + 51, alternancia_pausada ? 1 : 0);
}

void aplicarCambiosModbus() {
  // Leer cambios desde Modbus TCP
  uint16_t reg_40001 = mb.Hreg(MODBUS_BASE_ADDRESS);
  uint16_t reg_40002 = mb.Hreg(MODBUS_BASE_ADDRESS + 1);
  
  if (reg_40001 > 0 && reg_40001 != alternancia_intervalo) {
    alternancia_intervalo = reg_40001;
    preferences.putUInt("alt_int", alternancia_intervalo);
    Serial.print("[MODBUS] Alternancia actualizada: ");
    Serial.println(alternancia_intervalo);
  }
  
  if (reg_40002 > 0 && reg_40002 != postCirculacion_duracion) {
    postCirculacion_duracion = reg_40002;
    preferences.putUInt("post_dur", postCirculacion_duracion);
    Serial.print("[MODBUS] Post-circ duración actualizada: ");
    Serial.println(postCirculacion_duracion);
  }
}

// ============================================================================
// FUNCIONES - WiFi
// ============================================================================

void inicializarWiFi() {
  if (wifi_modo_sta && wifi_ssid_sta.length() > 0) {
    // Modo Station
    Serial.print("Conectando a WiFi: ");
    Serial.println(wifi_ssid_sta);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid_sta.c_str(), wifi_password_sta.c_str());
    
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 20) {
      delay(500);
      Serial.print(".");
      intentos++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConectado a WiFi!");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      digitalWrite(PIN_LED_WIFI, HIGH);
    } else {
      Serial.println("\nNo se pudo conectar. Activando AP...");
      iniciarAP();
    }
  } else {
    // Modo Access Point
    iniciarAP();
  }
}

void iniciarAP() {
  Serial.println("Iniciando Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(wifi_ssid_ap.c_str(), wifi_password_ap.c_str());
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);
  digitalWrite(PIN_LED_WIFI, HIGH);
}

void sincronizarNTP() {
  Serial.println("[NTP] Sincronizando...");
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server.c_str());
  
  // Esperar hasta 5 segundos
  int intentos = 0;
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo) && intentos < 10) {
    delay(500);
    intentos++;
  }
  
  if (getLocalTime(&timeinfo)) {
    Serial.println("[NTP] Sincronización exitosa");
  } else {
    Serial.println("[NTP] Sincronización fallida");
  }
}

// ============================================================================
// FUNCIONES - CONFIGURACIÓN NVS
// ============================================================================

void cargarConfiguracion() {
  wifi_ssid_sta = preferences.getString("wifi_ssid", "");
  wifi_password_sta = preferences.getString("wifi_pass", "");
  wifi_ssid_ap = preferences.getString("ap_ssid", "Caldera_ESP32S3");
  wifi_password_ap = preferences.getString("ap_pass", "12345678");
  wifi_modo_sta = preferences.getBool("wifi_sta", false);
  
  ntp_auto = preferences.getBool("ntp_auto", false);
  ntp_server = preferences.getString("ntp_server", "pool.ntp.org");
  gmt_offset_sec = preferences.getLong("gmt_offset", 3600);
  daylight_offset_sec = preferences.getInt("dst_offset", 3600);
  
  alternancia_intervalo = preferences.getUInt("alt_int", 168);
  postCirculacion_duracion = preferences.getUInt("post_dur", 180);
  alternancia_activa = preferences.getBool("alt_activa", true);
  
  Serial.println("[NVS] Configuración cargada");
}

void guardarConfiguracion() {
  preferences.putString("wifi_ssid", wifi_ssid_sta);
  preferences.putString("wifi_pass", wifi_password_sta);
  preferences.putString("ap_ssid", wifi_ssid_ap);
  preferences.putString("ap_pass", wifi_password_ap);
  preferences.putBool("wifi_sta", wifi_modo_sta);
  
  preferences.putBool("ntp_auto", ntp_auto);
  preferences.putString("ntp_server", ntp_server);
  preferences.putLong("gmt_offset", gmt_offset_sec);
  preferences.putInt("dst_offset", daylight_offset_sec);
  
  preferences.putUInt("alt_int", alternancia_intervalo);
  preferences.putUInt("post_dur", postCirculacion_duracion);
  preferences.putBool("alt_activa", alternancia_activa);
  
  Serial.println("[NVS] Configuración guardada");
}

// ============================================================================
// SERVIDOR WEB - INICIALIZACIÓN Y RUTAS
// ============================================================================

void inicializarServidor() {
  // Página principal
  server.on("/", HTTP_GET, handleRoot);
  
  // API endpoints
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/config", HTTP_GET, handleAPIConfig);
  server.on("/api/config", HTTP_POST, handleAPIConfigPost);
  server.on("/api/wifi", HTTP_GET, handleAPIWiFi);
  server.on("/api/wifi", HTTP_POST, handleAPIWiFiPost);
  server.on("/api/wifi/scan", HTTP_GET, handleAPIWiFiScan);
  server.on("/api/time", HTTP_POST, handleAPITimePost);
  server.on("/api/alarmas", HTTP_GET, handleAPIAlarmas);
  
  server.begin();
  Serial.println("Servidor Web iniciado en puerto 80");
}

// ============================================================================
// SERVIDOR WEB - HANDLERS
// ============================================================================

void handleRoot() {
  String html = getHTMLPage();
  server.send(200, "text/html", html);
}

void handleAPIStatus() {
  struct tm timeinfo;
  char timeString[64];
  if (getLocalTime(&timeinfo)) {
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  } else {
    strcpy(timeString, "No sincronizado");
  }
  
  String json = "{";
  json += "\"temperatura\":" + String(temperatura_actual, 1) + ",";
  json += "\"GT\":" + String(estado_GT ? "true" : "false") + ",";
  json += "\"GR\":" + String(estado_GR ? "true" : "false") + ",";
  json += "\"GE\":" + String(estado_GE ? "true" : "false") + ",";
  json += "\"SISTEMA\":" + String(estado_SISTEMA ? "true" : "false") + ",";
  json += "\"JEFATURA\":" + String(estado_JEFATURA ? "true" : "false") + ",";
  json += "\"PROG\":" + String(estado_PROG ? "true" : "false") + ",";
  json += "\"VERIF_B1\":" + String(estado_VERIF_B1 ? "true" : "false") + ",";
  json += "\"VERIF_B2\":" + String(estado_VERIF_B2 ? "true" : "false") + ",";
  json += "\"NIVEL_OK\":" + String(estado_NIVEL_OK ? "true" : "false") + ",";
  json += "\"TERMOSTATO_ACS\":" + String(estado_TERMOSTATO_ACS ? "true" : "false") + ",";
  json += "\"TERMOSTATO_AMB\":" + String(estado_TERMOSTATO_AMB ? "true" : "false") + ",";
  json += "\"MARCHA\":" + String(estado_MARCHA ? "true" : "false") + ",";
  json += "\"BOMBA_1\":" + String(salida_BOMBA_1 ? "true" : "false") + ",";
  json += "\"BOMBA_2\":" + String(salida_BOMBA_2 ? "true" : "false") + ",";
  json += "\"VALVULA_ACS\":" + String(salida_VALVULA_ACS ? "true" : "false") + ",";
  json += "\"VALVULA_AMB\":" + String(salida_VALVULA_AMB ? "true" : "false") + ",";
  json += "\"CONTACTO_AUX\":" + String(salida_CONTACTO_AUX ? "true" : "false") + ",";
  json += "\"postCirc\":" + String(postCirculacion_activa ? "true" : "false") + ",";
  json += "\"bombaPrincipal\":\"" + String(bomba_principal_es_B1 ? "B1" : "B2") + "\",";
  json += "\"tiempo\":\"" + String(timeString) + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleAPIConfig() {
  String json = "{";
  json += "\"alt_intervalo\":" + String(alternancia_intervalo) + ",";
  json += "\"post_duracion\":" + String(postCirculacion_duracion) + ",";
  json += "\"alt_activa\":" + String(alternancia_activa ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleAPIConfigPost() {
  if (server.hasArg("alt_intervalo")) {
    alternancia_intervalo = server.arg("alt_intervalo").toInt();
  }
  if (server.hasArg("post_duracion")) {
    postCirculacion_duracion = server.arg("post_duracion").toInt();
  }
  if (server.hasArg("alt_activa")) {
    alternancia_activa = server.arg("alt_activa") == "true";
  }
  
  guardarConfiguracion();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleAPIWiFi() {
  String json = "{";
  json += "\"ssid_sta\":\"" + wifi_ssid_sta + "\",";
  json += "\"ssid_ap\":\"" + wifi_ssid_ap + "\",";
  json += "\"modo_sta\":" + String(wifi_modo_sta ? "true" : "false") + ",";
  json += "\"conectado\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ip\":\"" + (wifi_modo_sta ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleAPIWiFiPost() {
  if (server.hasArg("ssid_sta")) {
    wifi_ssid_sta = server.arg("ssid_sta");
  }
  if (server.hasArg("pass_sta")) {
    wifi_password_sta = server.arg("pass_sta");
  }
  if (server.hasArg("ssid_ap")) {
    wifi_ssid_ap = server.arg("ssid_ap");
  }
  if (server.hasArg("pass_ap")) {
    wifi_password_ap = server.arg("pass_ap");
  }
  if (server.hasArg("modo_sta")) {
    wifi_modo_sta = server.arg("modo_sta") == "true";
  }
  
  guardarConfiguracion();
  server.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Reinicie para aplicar cambios\"}");
}

void handleAPIWiFiScan() {
  Serial.println("[WiFi] Escaneando redes...");
  int n = WiFi.scanNetworks();
  
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + ",\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
  }
  json += "]";
  
  server.send(200, "application/json", json);
}

void handleAPITimePost() {
  if (server.hasArg("sync") && server.arg("sync") == "manual") {
    // Sincronización manual NTP
    sincronizarNTP();
    server.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Sincronización manual realizada\"}");
  } else if (server.hasArg("ntp_auto")) {
    // Activar/desactivar auto NTP
    ntp_auto = server.arg("ntp_auto") == "true";
    guardarConfiguracion();
    if (ntp_auto) {
      sincronizarNTP();
    }
    server.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"NTP auto " + String(ntp_auto ? "activado" : "desactivado") + "\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"msg\":\"Parámetros inválidos\"}");
  }
}

void handleAPIAlarmas() {
  String json = "{";
  json += "\"alarma_1\":" + String(alarma_1 ? "true" : "false") + ",";
  json += "\"alarma_2\":" + String(alarma_2 ? "true" : "false") + ",";
  json += "\"alarma_3\":" + String(alarma_3 ? "true" : "false") + ",";
  json += "\"alarma_4\":" + String(alarma_4 ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

// ============================================================================
// HTML EMBEBIDO
// ============================================================================

String getHTMLPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Controlador Calefacción V607</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        
        header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }
        
        header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
        }
        
        header p {
            opacity: 0.9;
            font-size: 1.1em;
        }
        
        .tabs {
            display: flex;
            background: #f5f5f5;
            border-bottom: 2px solid #ddd;
            overflow-x: auto;
        }
        
        .tab {
            padding: 15px 30px;
            cursor: pointer;
            border: none;
            background: none;
            font-size: 1em;
            color: #666;
            transition: all 0.3s;
            white-space: nowrap;
        }
        
        .tab:hover {
            background: #e0e0e0;
        }
        
        .tab.active {
            background: white;
            color: #667eea;
            border-bottom: 3px solid #667eea;
        }
        
        .tab-content {
            display: none;
            padding: 30px;
            animation: fadeIn 0.3s;
        }
        
        .tab-content.active {
            display: block;
        }
        
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .status-card {
            background: #f9f9f9;
            border-radius: 10px;
            padding: 20px;
            border-left: 4px solid #667eea;
        }
        
        .status-card h3 {
            color: #333;
            margin-bottom: 15px;
            font-size: 1.1em;
        }
        
        .status-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px 0;
            border-bottom: 1px solid #eee;
        }
        
        .status-item:last-child {
            border-bottom: none;
        }
        
        .status-label {
            color: #666;
            font-weight: 500;
        }
        
        .status-value {
            font-weight: bold;
            color: #333;
        }
        
        .status-value.on {
            color: #4CAF50;
        }
        
        .status-value.off {
            color: #f44336;
        }
        
        .temp-display {
            font-size: 3em;
            text-align: center;
            color: #667eea;
            margin: 20px 0;
            font-weight: bold;
        }
        
        .temp-status {
            text-align: center;
            padding: 15px;
            background: #f0f0f0;
            border-radius: 8px;
            margin: 20px 0;
            font-size: 1.2em;
        }
        
        .temp-status.normal {
            background: #e8f5e9;
            color: #2e7d32;
        }
        
        .temp-status.warning {
            background: #fff3e0;
            color: #ef6c00;
        }
        
        .temp-status.critical {
            background: #ffebee;
            color: #c62828;
        }
        
        .form-group {
            margin-bottom: 20px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            color: #333;
            font-weight: 500;
        }
        
        .form-group input,
        .form-group select {
            width: 100%;
            padding: 12px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 1em;
            transition: border 0.3s;
        }
        
        .form-group input:focus,
        .form-group select:focus {
            outline: none;
            border-color: #667eea;
        }
        
        .value-pair {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 5px;
        }
        
        .value-box {
            padding: 10px;
            background: #f5f5f5;
            border-radius: 5px;
            text-align: center;
        }
        
        .value-box label {
            display: block;
            font-size: 0.85em;
            color: #666;
            margin-bottom: 5px;
        }
        
        .value-box input {
            width: 100%;
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
            text-align: center;
        }
        
        .checkbox-group {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .checkbox-group input[type="checkbox"] {
            width: 20px;
            height: 20px;
        }
        
        button {
            padding: 12px 30px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s;
            margin-right: 10px;
            margin-top: 10px;
        }
        
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        
        button:active {
            transform: translateY(0);
        }
        
        button.secondary {
            background: #6c757d;
        }
        
        .alert {
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        
        .alert-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        
        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        
        .alarm-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        
        .alarm-indicator.active {
            background: #f44336;
            box-shadow: 0 0 10px #f44336;
            animation: blink 1s infinite;
        }
        
        .alarm-indicator.inactive {
            background: #4CAF50;
        }
        
        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }
        
        #tiempo-actual {
            font-size: 1.2em;
            text-align: center;
            padding: 10px;
            background: #f5f5f5;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        
        @media (max-width: 768px) {
            header h1 {
                font-size: 1.8em;
            }
            
            .status-grid {
                grid-template-columns: 1fr;
            }
            
            .tabs {
                flex-wrap: wrap;
            }
            
            .tab {
                flex: 1;
                min-width: 120px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🔥 Controlador de Calefacción</h1>
            <p>ESP32-S3 - Versión 607 GitHub</p>
            <div id="tiempo-actual">Cargando hora...</div>
        </header>
        
        <div class="tabs">
            <button class="tab active" onclick="openTab(event, 'tab-principal')">Principal</button>
            <button class="tab" onclick="openTab(event, 'tab-config')">Configuración</button>
            <button class="tab" onclick="openTab(event, 'tab-wifi')">WiFi</button>
            <button class="tab" onclick="openTab(event, 'tab-alarmas')">Alarmas</button>
            <button class="tab" onclick="openTab(event, 'tab-prog')">Programación</button>
        </div>
        
        <!-- TAB PRINCIPAL -->
        <div id="tab-principal" class="tab-content active">
            <div class="temp-display" id="temp-display">-- °C</div>
            <div class="temp-status normal" id="temp-status">Estado de temperatura normal</div>
            
            <div class="status-grid">
                <div class="status-card">
                    <h3>Entradas Generadores</h3>
                    <div class="status-item">
                        <span class="status-label">GT (Principal)</span>
                        <span class="status-value" id="st-gt">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">GR (Reserva)</span>
                        <span class="status-value" id="st-gr">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">GE (Emergencia)</span>
                        <span class="status-value" id="st-ge">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <h3>Mandos y Control</h3>
                    <div class="status-item">
                        <span class="status-label">SISTEMA</span>
                        <span class="status-value" id="st-sistema">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">JEFATURA</span>
                        <span class="status-value" id="st-jefatura">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">MARCHA</span>
                        <span class="status-value" id="st-marcha">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <h3>Termostatos</h3>
                    <div class="status-item">
                        <span class="status-label">ACS (Agua Sanitaria)</span>
                        <span class="status-value" id="st-t-acs">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">AMB (Ambiente)</span>
                        <span class="status-value" id="st-t-amb">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Nivel OK</span>
                        <span class="status-value" id="st-nivel">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <h3>Salidas Activas</h3>
                    <div class="status-item">
                        <span class="status-label">Bomba 1</span>
                        <span class="status-value" id="st-b1">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Bomba 2</span>
                        <span class="status-value" id="st-b2">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Válvula ACS</span>
                        <span class="status-value" id="st-v-acs">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Válvula AMB</span>
                        <span class="status-value" id="st-v-amb">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <h3>Sistema</h3>
                    <div class="status-item">
                        <span class="status-label">Post-Circulación</span>
                        <span class="status-value" id="st-postcirc">--</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Bomba Principal</span>
                        <span class="status-value" id="st-bomba-princ">--</span>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- TAB CONFIGURACIÓN -->
        <div id="tab-config" class="tab-content">
            <h2 style="margin-bottom: 20px;">Parámetros del Sistema</h2>
            
            <div class="form-group">
                <label>Intervalo de Alternancia de Bombas (horas)</label>
                <div class="value-pair">
                    <div class="value-box">
                        <label>Valor actual</label>
                        <input type="number" id="alt-int-read" readonly>
                    </div>
                    <div class="value-box">
                        <label>Nuevo valor</label>
                        <input type="number" id="alt-int-write" min="1" max="1000">
                    </div>
                </div>
            </div>
            
            <div class="form-group">
                <label>Duración Post-Circulación (segundos)</label>
                <div class="value-pair">
                    <div class="value-box">
                        <label>Valor actual</label>
                        <input type="number" id="post-dur-read" readonly>
                    </div>
                    <div class="value-box">
                        <label>Nuevo valor</label>
                        <input type="number" id="post-dur-write" min="0" max="3600">
                    </div>
                </div>
            </div>
            
            <div class="form-group checkbox-group">
                <input type="checkbox" id="alt-activa">
                <label for="alt-activa">Alternancia Activa</label>
            </div>
            
            <button onclick="guardarConfig()">Guardar Configuración</button>
            <div id="msg-config"></div>
        </div>
        
        <!-- TAB WIFI -->
        <div id="tab-wifi" class="tab-content">
            <h2 style="margin-bottom: 20px;">Configuración WiFi</h2>
            
            <div class="form-group">
                <label>Modo de Operación</label>
                <select id="wifi-modo">
                    <option value="false">Access Point (AP)</option>
                    <option value="true">Station (STA)</option>
                </select>
            </div>
            
            <div class="form-group">
                <label>SSID Station (Conectar a red existente)</label>
                <select id="wifi-ssid-sta">
                    <option value="">-- Seleccionar red --</option>
                </select>
                <button class="secondary" onclick="escanearRedes()" style="margin-top: 10px;">Escanear Redes</button>
            </div>
            
            <div class="form-group">
                <label>Contraseña Station</label>
                <input type="password" id="wifi-pass-sta">
            </div>
            
            <div class="form-group">
                <label>SSID Access Point</label>
                <input type="text" id="wifi-ssid-ap">
            </div>
            
            <div class="form-group">
                <label>Contraseña Access Point</label>
                <input type="password" id="wifi-pass-ap">
            </div>
            
            <h3 style="margin-top: 30px; margin-bottom: 15px;">Sincronización Horaria</h3>
            
            <div class="form-group checkbox-group">
                <input type="checkbox" id="ntp-auto">
                <label for="ntp-auto">Actualizar hora automáticamente (NTP)</label>
            </div>
            
            <button onclick="sincronizarHoraManual()">Sincronizar Hora Manualmente</button>
            <button onclick="guardarWiFi()">Guardar WiFi</button>
            <div id="msg-wifi"></div>
        </div>
        
        <!-- TAB ALARMAS -->
        <div id="tab-alarmas" class="tab-content">
            <h2 style="margin-bottom: 20px;">Estado de Alarmas</h2>
            
            <div class="status-grid">
                <div class="status-card">
                    <div class="status-item">
                        <span class="status-label">
                            <span class="alarm-indicator" id="alarm-ind-1"></span>
                            Alarma 1 (Generador)
                        </span>
                        <span class="status-value" id="alarm-1">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <div class="status-item">
                        <span class="status-label">
                            <span class="alarm-indicator" id="alarm-ind-2"></span>
                            Alarma 2 (Bomba 1)
                        </span>
                        <span class="status-value" id="alarm-2">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <div class="status-item">
                        <span class="status-label">
                            <span class="alarm-indicator" id="alarm-ind-3"></span>
                            Alarma 3 (Bomba 2)
                        </span>
                        <span class="status-value" id="alarm-3">--</span>
                    </div>
                </div>
                
                <div class="status-card">
                    <div class="status-item">
                        <span class="status-label">
                            <span class="alarm-indicator" id="alarm-ind-4"></span>
                            Alarma 4 (General)
                        </span>
                        <span class="status-value" id="alarm-4">--</span>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- TAB PROGRAMACIÓN -->
        <div id="tab-prog" class="tab-content">
            <h2 style="margin-bottom: 20px;">Programación Horaria</h2>
            
            <div class="alert alert-success">
                <strong>Nota:</strong> La programación horaria se activa mediante el selector físico PROG.
            </div>
            
            <p>Funcionalidad de programación horaria en desarrollo.</p>
            <p>Configure franjas horarias para activación automática del sistema.</p>
        </div>
    </div>
    
    <script>
        // Actualizar estado cada 2 segundos
        setInterval(actualizarEstado, 2000);
        actualizarEstado();
        
        // Actualizar configuración al cargar
        actualizarConfig();
        actualizarWiFiInfo();
        actualizarAlarmas();
        
        function openTab(evt, tabName) {
            var i, tabcontent, tablinks;
            tabcontent = document.getElementsByClassName("tab-content");
            for (i = 0; i < tabcontent.length; i++) {
                tabcontent[i].classList.remove("active");
            }
            tablinks = document.getElementsByClassName("tab");
            for (i = 0; i < tablinks.length; i++) {
                tablinks[i].classList.remove("active");
            }
            document.getElementById(tabName).classList.add("active");
            evt.currentTarget.classList.add("active");
        }
        
        function actualizarEstado() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Temperatura
                    document.getElementById('temp-display').textContent = data.temperatura.toFixed(1) + ' °C';
                    
                    // Estado de temperatura
                    let tempStatus = document.getElementById('temp-status');
                    if (data.temperatura < 50) {
                        tempStatus.className = 'temp-status normal';
                        tempStatus.textContent = '✓ Temperatura normal';
                    } else if (data.temperatura < 80) {
                        tempStatus.className = 'temp-status warning';
                        tempStatus.textContent = '⚠ Temperatura elevada';
                    } else {
                        tempStatus.className = 'temp-status critical';
                        tempStatus.textContent = '🔥 Temperatura crítica';
                    }
                    
                    // Tiempo
                    document.getElementById('tiempo-actual').textContent = '🕐 ' + data.tiempo;
                    
                    // Estados
                    updateStatus('st-gt', data.GT);
                    updateStatus('st-gr', data.GR);
                    updateStatus('st-ge', data.GE);
                    updateStatus('st-sistema', data.SISTEMA);
                    updateStatus('st-jefatura', data.JEFATURA);
                    updateStatus('st-marcha', data.MARCHA);
                    updateStatus('st-t-acs', data.TERMOSTATO_ACS);
                    updateStatus('st-t-amb', data.TERMOSTATO_AMB);
                    updateStatus('st-nivel', data.NIVEL_OK);
                    updateStatus('st-b1', data.BOMBA_1);
                    updateStatus('st-b2', data.BOMBA_2);
                    updateStatus('st-v-acs', data.VALVULA_ACS);
                    updateStatus('st-v-amb', data.VALVULA_AMB);
                    updateStatus('st-postcirc', data.postCirc);
                    document.getElementById('st-bomba-princ').textContent = data.bombaPrincipal;
                });
        }
        
        function updateStatus(id, value) {
            let elem = document.getElementById(id);
            elem.textContent = value ? 'ON' : 'OFF';
            elem.className = 'status-value ' + (value ? 'on' : 'off');
        }
        
        function actualizarConfig() {
            fetch('/api/config')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('alt-int-read').value = data.alt_intervalo;
                    document.getElementById('alt-int-write').value = data.alt_intervalo;
                    document.getElementById('post-dur-read').value = data.post_duracion;
                    document.getElementById('post-dur-write').value = data.post_duracion;
                    document.getElementById('alt-activa').checked = data.alt_activa;
                });
        }
        
        function guardarConfig() {
            let params = new URLSearchParams();
            params.append('alt_intervalo', document.getElementById('alt-int-write').value);
            params.append('post_duracion', document.getElementById('post-dur-write').value);
            params.append('alt_activa', document.getElementById('alt-activa').checked);
            
            fetch('/api/config', {
                method: 'POST',
                body: params
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('msg-config').innerHTML = '<div class="alert alert-success">Configuración guardada correctamente</div>';
                setTimeout(() => actualizarConfig(), 1000);
            });
        }
        
        function actualizarWiFiInfo() {
            fetch('/api/wifi')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('wifi-modo').value = data.modo_sta.toString();
                    document.getElementById('wifi-ssid-ap').value = data.ssid_ap;
                });
        }
        
        function escanearRedes() {
            document.getElementById('msg-wifi').innerHTML = '<div class="alert alert-success">Escaneando redes...</div>';
            
            fetch('/api/wifi/scan')
                .then(response => response.json())
                .then(data => {
                    let select = document.getElementById('wifi-ssid-sta');
                    select.innerHTML = '<option value="">-- Seleccionar red --</option>';
                    data.forEach(red => {
                        let option = document.createElement('option');
                        option.value = red.ssid;
                        option.textContent = red.ssid + ' (' + red.rssi + ' dBm)' + (red.secure ? ' 🔒' : '');
                        select.appendChild(option);
                    });
                    document.getElementById('msg-wifi').innerHTML = '<div class="alert alert-success">Escaneo completado: ' + data.length + ' redes encontradas</div>';
                });
        }
        
        function sincronizarHoraManual() {
            let params = new URLSearchParams();
            params.append('sync', 'manual');
            
            fetch('/api/time', {
                method: 'POST',
                body: params
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('msg-wifi').innerHTML = '<div class="alert alert-success">' + data.msg + '</div>';
            });
        }
        
        function guardarWiFi() {
            let params = new URLSearchParams();
            params.append('ssid_sta', document.getElementById('wifi-ssid-sta').value);
            params.append('pass_sta', document.getElementById('wifi-pass-sta').value);
            params.append('ssid_ap', document.getElementById('wifi-ssid-ap').value);
            params.append('pass_ap', document.getElementById('wifi-pass-ap').value);
            params.append('modo_sta', document.getElementById('wifi-modo').value);
            
            fetch('/api/wifi', {
                method: 'POST',
                body: params
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('msg-wifi').innerHTML = '<div class="alert alert-success">' + data.msg + '</div>';
            });
            
            // Guardar configuración NTP
            let paramsTime = new URLSearchParams();
            paramsTime.append('ntp_auto', document.getElementById('ntp-auto').checked);
            
            fetch('/api/time', {
                method: 'POST',
                body: paramsTime
            });
        }
        
        function actualizarAlarmas() {
            fetch('/api/alarmas')
                .then(response => response.json())
                .then(data => {
                    updateAlarm('alarm-1', 'alarm-ind-1', data.alarma_1);
                    updateAlarm('alarm-2', 'alarm-ind-2', data.alarma_2);
                    updateAlarm('alarm-3', 'alarm-ind-3', data.alarma_3);
                    updateAlarm('alarm-4', 'alarm-ind-4', data.alarma_4);
                });
        }
        
        function updateAlarm(textId, indId, active) {
            let text = document.getElementById(textId);
            let ind = document.getElementById(indId);
            text.textContent = active ? 'ACTIVA' : 'Normal';
            text.className = 'status-value ' + (active ? 'off' : 'on');
            ind.className = 'alarm-indicator ' + (active ? 'active' : 'inactive');
        }
        
        setInterval(actualizarAlarmas, 3000);
    </script>
</body>
</html>
)rawliteral";
}
