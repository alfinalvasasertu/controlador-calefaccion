# 🔥 Firmware Controlador de Calefacción ESP32-S3

**Versión:** 1.0  
**Fecha:** 21 de enero de 2026  
**Estado:** ✅ Compilado y listo para cargar

---

## 📋 Contenido

- `firmware.bin` - Firmware principal (791 KB)
- `bootloader.bin` - Bootloader (15 KB)
- `partitions.bin` - Tabla de particiones (3 KB)
- `CARGAR_FIRMWARE.bat` - Script automático para Windows

---

## 🚀 Instrucciones de Carga (Windows)

### Opción 1: Script Automático (RECOMENDADO)

1. **Abre PowerShell o CMD** en la carpeta con estos archivos
2. **Ejecuta:**
   ```
   CARGAR_FIRMWARE.bat
   ```
3. **Ingresa el puerto COM** (ej: COM17)
4. **Sigue las instrucciones en pantalla**

### Opción 2: Manual con esptool

1. **Instala esptool** (si no lo tienes):
   ```powershell
   pip install esptool
   ```

2. **Carga el firmware:**
   ```powershell
   esptool.py --chip esp32s3 --port COM17 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
   ```
   
   **Reemplaza `COM17` con tu puerto real**

---

## ⚙️ Configuración Inicial

### WiFi

**Punto de Acceso (AP):**
- SSID: `Caldera_ESP32S3`
- Contraseña: `caldera2026`
- IP: `192.168.4.1`

**Conexión a Tu Red (Opcional):**
- En la web: http://192.168.4.1/settings
- Ingresa SSID y contraseña de tu WiFi

### Web Interface

```
http://192.168.4.1/
```

**Endpoints disponibles:**
- `GET /` - Página principal
- `GET /status` - Estado actual
- `POST /control` - Controlar sistema
- `GET /settings` - Ver configuración
- `POST /setCfg` - Modificar configuración

---

## 🔧 Especificaciones

- **Microcontrolador:** ESP32-S3 (240 MHz, 320 KB RAM)
- **Flash:** 8 MB
- **Memoria usado:** 24.2% (809 KB)
- **RAM usado:** 14% (46 KB)

---

## 📊 Características

✅ **WiFi Dual Mode:**
- AP activo permanentemente
- STA con reconexión automática (60s)
- Sincronización NTP

✅ **Control de Calefacción:**
- Alternancia automática de bombas
- Monitoreo de temperatura (NTC)
- Alarmas y protecciones

✅ **Persistencia:**
- Configuración guardada en Flash (NVS)
- Recuperación automática al reiniciar

✅ **GPIO Mapeados (15 total):**
- SYS(4), PROG(5), SW_B1(6), SW_B2(7)
- JEF(15), EMERG(8), RT1(9), RT2(10)
- AL_GT(11), NTC(1), CONT_B1(12), CONT_B2(13)
- RELE_GT(14), RELE_BC(21), POST(47)
- SOBRE_CAL(38), AV_G(40), PROG_ACTIVA(42)
- AV_B1(39), AV_B2(41)

---

## 🐛 Troubleshooting

### "Failed to connect to ESP32-S3"

1. Verifica que el ESP32 esté conectado al USB
2. Intenta presionar **BOOT** mientras esptool se conecta
3. Prueba con otro cable USB
4. Verifica el puerto COM correcto

### "Permission denied"

En Linux/Mac, necesitas permisos:
```bash
sudo chmod 666 /dev/ttyUSB0  # Linux
```

### WiFi no conecta

1. Reinicia el ESP32
2. Espera 30 segundos para que levante el AP
3. Conecta a `Caldera_ESP32S3`
4. Abre http://192.168.4.1

---

## 📝 Notas de Versión

- ✅ WiFi estable (AP + STA con auto-reconexión)
- ✅ Web server funcional
- ✅ Persistencia de configuración
- ✅ NTP sincronización
- ✅ Control de calefacción con bombas alternadas
- ✅ Compilación optimizada (PlatformIO)

---

## 🔗 URLs Útiles

- **Documentación ESP32:** https://docs.espressif.com/
- **esptool en GitHub:** https://github.com/espressif/esptool
- **Arduino Core para ESP32:** https://github.com/espressif/arduino-esp32

---

**¿Problemas?** Revisa los logs de la consola serial a 115200 baud después de cargar.

Éxito con tu presentación 🚀
