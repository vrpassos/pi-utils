import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime

# --- Configuração GPIO Raspberry Pi (INPUTS) ---
GPIO.setmode(GPIO.BOARD) # Usando a numeração BOARD para os pinos físicos
# Mapeamento dos pinos físicos que serão ENTRADAS
# Estes pinos devem estar conectados às GPIOs de SAÍDA da Toradex
PIN_35_BOARD = 35 # Entrada que receberá o sinal do GPIO 27 da Toradex
PIN_36_BOARD = 36 # Entrada que receberá o sinal do GPIO 28 da Toradex
PIN_37_BOARD = 37 # Entrada que receberá o sinal do GPIO 29 da Toradex
PIN_38_BOARD = 38 # Entrada que receberá o sinal do GPIO 30 da Toradex

INPUT_PINS = [PIN_35_BOARD, PIN_36_BOARD, PIN_37_BOARD, PIN_38_BOARD]

# Configura os pinos como entrada
# Se as saídas da Toradex são ativas HIGH, você pode usar PUD_DOWN para garantir um estado LOW quando desconectado.
# Se as saídas da Toradex são ativas LOW, use PUD_UP. Ou não use pull-up/down se as saídas da Toradex forem robustas.
for pin in INPUT_PINS:
    GPIO.setup(pin, GPIO.IN) # Sem pull-up/down, espera um sinal claro da Toradex

# --- Configuração UART Raspberry Pi ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

# String constante para enviar (apenas para inicialização, o conteúdo será dinâmico)
MESSAGE_TO_SEND = 'string teste\n'

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    while True:
        # Lê o estado das suas próprias GPIOs de entrada
        pin_states = []
        for pin in INPUT_PINS:
            state = GPIO.input(pin) # Retorna 0 (LOW) ou 1 (HIGH)
            pin_states.append(str(state))

        # Converte o estado dos pinos para um byte
        # A ordem dos bits: P35 (MSB) P36 P37 P38 (LSB)
        # Isso significa que o estado de P35 será o bit 3, P36 o bit 2, etc.
        bit_string = "".join(pin_states)
        byte_to_send = int(bit_string, 2).to_bytes(1, 'big')

        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"{timestamp}: Raspberry Pi GPIOs (35-38) lidas: {' '.join(pin_states)}")
        
        ser.write(byte_to_send)
        time.sleep(0.5) # Envia o estado a cada 0.5 segundos

except serial.SerialException as e:
    print(f"Erro ao abrir ou usar a porta serial: {e}")
    print("Verifique se a porta serial está correta e se o console serial está desabilitado (se necessário).")
except KeyboardInterrupt:
    print("Transmissão interrompida pelo usuário.")
except Exception as e:
    print(f"Ocorreu um erro: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Porta serial fechada.")
    GPIO.cleanup() # Limpa as configurações GPIO
    print("GPIOs limpas.")