# 📝 Cargar Firmware con Arduino IDE

## Opción 1: Usando el archivo .ino (Arduino IDE)

### Paso 1: Instalar Arduino IDE
- Descarga desde: https://www.arduino.cc/en/software
- Instala normalmente

### Paso 2: Instalar ESP32
En Arduino IDE:
1. **File** → **Preferences**
2. **Additional Boards Manager URLs** (copiar el link siguiente):
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Haz clic en el ícono de carpeta y pega el link
4. **OK**

### Paso 3: Instalar el board ESP32-S3
1. **Tools** → **Board** → **Boards Manager**
2. Busca: `ESP32`
3. Haz clic en **ESP32 by Espressif Systems**
4. Selecciona versión **3.0.0** o superior
5. Haz clic en **Install**
6. Espera 5 minutos (es grande)

### Paso 4: Configurar el Board
1. **Tools** → **Board** → **ESP32** → **ESP32-S3 Dev Module**
2. **Tools** → **Upload Speed** → **921600**
3. **Tools** → **Flash Size** → **8MB**
4. **Tools** → **Partition Scheme** → **Huge APP (3MB No OTA/1MB SPIFFS)**

### Paso 5: Seleccionar Puerto Serial
1. **Tools** → **Port** → **COM17** (tu puerto)

### Paso 6: Cargar el .ino
1. Abre el archivo: `Controlador_Calefaccion_V560.ino`
2. Haz clic en **Upload** (flecha →)
3. Espera a que termine (30-60 segundos)

### Éxito si ves:
```
Leaving...
Hard resetting via RTS pin...
✓ Firmware cargado correctamente
```

---

## Opción 2: Usar el firmware.bin compilado (más rápido)

Si ya tienes **esptool** instalado:

```powershell
esptool.py --chip esp32s3 --port COM17 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

---

## 🎯 Después de Cargar

1. **Reinicia el ESP32** (desconecta y conecta USB)
2. **Busca WiFi:** `Caldera_ESP32S3`
3. **Contraseña:** `caldera2026`
4. **Abre el navegador:** `http://192.168.4.1`

---

## ⚠️ Problemas Comunes

### "No se detecta el puerto"
- Instala CH340 drivers: https://github.com/WCHSoftware/CH341SER/releases

### "Failed to connect to ESP32-S3"
- Presiona y mantén **BOOT** mientras esptool se conecta
- Prueba con otro cable USB

### "Compilation error"
- Arduino IDE no tiene todos los archivos
- Usa mejor el `.bin` compilado con esptool

---

**¡Tienes el código listo para personalizar!** 🚀
