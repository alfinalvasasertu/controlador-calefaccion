@echo off
REM Script para cargar el firmware en ESP32-S3
REM Autor: Controlador Calefaccion
REM Fecha: 2026-01-21

echo ====================================================
echo CARGADOR DE FIRMWARE ESP32-S3
echo Controlador de Calefaccion v1.0
echo ====================================================
echo.

REM Verificar si esptool esta instalado
where esptool.py >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: esptool.py no esta instalado
    echo Instala con: pip install esptool
    echo.
    pause
    exit /b 1
)

echo Puertos seriales disponibles:
echo.
wmic logicaldisk get name
echo.

set /p PORT="Ingresa el puerto (ej: COM17): "

if "%PORT%"=="" (
    echo ERROR: Debes especificar un puerto
    pause
    exit /b 1
)

echo.
echo ====================================================
echo ATENCION: Conecta el ESP32-S3 y presiona BOOT si es necesario
echo Cargando firmware en puerto: %PORT%
echo ====================================================
echo.

REM Cargar el firmware completo
esptool.py --chip esp32s3 --port %PORT% --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ====================================================
    echo EXITO: Firmware cargado correctamente
    echo ====================================================
    echo.
    echo El ESP32-S3 se reiniciara en 3 segundos...
    timeout /t 3
    echo.
    echo Conectate a: http://192.168.4.1
    echo SSID: Caldera_ESP32S3
    echo Pass: caldera2026
    echo.
) else (
    echo.
    echo ERROR: Fallo la carga del firmware
    echo Verifica:
    echo - El puerto sea correcto
    echo - El ESP32 este conectado
    echo - Presiona BOOT mientras se conecta
    echo.
)

pause
