import serial
import time
from datetime import datetime

# Configuração da porta serial UART no Raspberry Pi
# Verifique a porta correta para o seu modelo de Raspberry Pi.
# Geralmente é /dev/ttyS0 ou /dev/ttyAMA0.
# Certifique-se de desabilitar o console serial se estiver habilitado na UART.
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

# String constante para enviar
MESSAGE_TO_SEND = 'string teste\n' # Adiciona uma nova linha para facilitar a leitura no receptor

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    while True:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        ser.write(MESSAGE_TO_SEND.encode('utf-8'))
        print(f"{timestamp}: string enviada: '{MESSAGE_TO_SEND.strip()}'")
        time.sleep(2)

except serial.SerialException as e:
    print(f"Erro ao abrir ou usar a porta serial: {e}")
    print("Verifique se a porta serial está correta e se o console serial está desabilitado (se necessário).")
except KeyboardInterrupt:
    print("Transmissão interrompida pelo usuário.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Porta serial fechada.")