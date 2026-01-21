# WiFi Robusto - Mejoras Implementadas

## 🔧 Problemas Solucionados

1. **AP inestable** → Ahora se verifica cada 30s y se reinicia si cae
2. **Conexión STA que se pierde** → Reintentos automáticos cada 60s
3. **Pérdida de conexión tras reinicio** → Se mantiene el AP siempre activo
4. **Sin reconexión automática** → Máximo 3 reintentos antes de usar solo AP

## 📋 Cambios Clave

### 1. **Variables de Control Mejoradas**
- `ap_activo`: Verifica si AP está disponible
- `sta_conectado`: Estado actual de conexión STA
- `ultima_verificacion_wifi`: Control de tiempo entre chequeos
- `intentos_fallidos_sta`: Contador de reintentos (máx 3)

### 2. **Función `gestionarWiFi()`**
Se ejecuta en cada ciclo del loop() para:
- Verificar que AP siga activo (cada 30s)
- Reiniciar AP si se desconecta
- Intentar reconectar STA cada 60s
- Mostrar estado en Serial

### 3. **Función `inicializarWiFi()`**
Ejecutada en setup():
- Limpia conexiones previas
- Inicia AP PRIMERO (siempre activo)
- Intenta STA (15s timeout)
- Sincroniza NTP si STA conecta
- Muestra estado detallado

### 4. **Loop Mejorado**
```cpp
void loop() {
  gestionarWiFi();  // ← PRIMERO: Gestionar WiFi
  mb.task();
  server.handleClient();
  // ... resto de lógica
}
```

## 🎯 Comportamiento Esperado

**Escenario 1: Con WiFi disponible**
1. AP inicia → 192.168.4.1 (siempre disponible)
2. STA conecta a red guardada (15s)
3. NTP sincroniza hora
4. Ambos activos simultáneamente

**Escenario 2: Sin WiFi disponible**
1. AP inicia y funciona normalmente
2. STA falla tras 15s
3. Se reintenta cada 60s (máx 3 veces)
4. AP sigue disponible para conexión manual

**Escenario 3: Reinicio ESP32**
1. AP se reinicia automáticamente
2. STA se reconecta a red guardada
3. Sin pérdida de funcionalidad

**Escenario 4: WiFi cae mientras funciona**
1. Verificación cada 30s detecta caída
2. Reintentos automáticos cada 60s
3. AP sigue disponible

## 📊 Diagnóstico en Serial

Ver estado con:
```
[segundos] WiFi: AP=1 STA=1 | B1=0 B2=0 GT=0 | T=45.2°C
          ^----^  ^---^  ^---^
          Tiempo  AP OK  STA OK
```

## ⚙️ Configuración Fine-tuning

Si necesitas cambiar tiempos:

```cpp
// En línea ~189:
const unsigned long VERIFICACION_WIFI_MS = 30000;  // Verificar cada 30s
const unsigned long INTENTO_STA_INTERVAL = 60000;  // Reintentar cada 60s
const int MAX_INTENTOS_STA = 3;                    // Máximo 3 reintentos
```

## ✅ Checklist Pre-Presentación

- [ ] Reinicia ESP32 varias veces
- [ ] Comprueba que AP sigue disponible en 192.168.4.1
- [ ] Comprueba conexión STA en Serial
- [ ] Prueba web en http://192.168.4.1
- [ ] Prueba Modbus TCP (puerto 502)
- [ ] Quita WiFi 5 segundos y verifica reconexión
- [ ] Verifica Serial para diagnosticar

---
**Estado: LISTA PARA PRESENTACIÓN**
