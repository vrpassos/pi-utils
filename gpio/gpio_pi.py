import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime

# --- Configuração GPIO Raspberry Pi ---
GPIO.setmode(GPIO.BOARD) # <-- ATENÇÃO: Mudado para BOARD numbering
# Mapeamento dos pinos físicos (BOARD) que você especificou
PIN_35_BOARD = 35
PIN_36_BOARD = 36
PIN_37_BOARD = 37
PIN_38_BOARD = 38

INPUT_PINS = [PIN_35_BOARD, PIN_36_BOARD, PIN_37_BOARD, PIN_38_BOARD]

# Configura os pinos como entrada com pull-up interno (se não houver resistores externos)
# IMPORTANTE: Se seus sensores/botões já possuem pull-ups/downs externos,
# ou se a lógica for ativo baixo (pino conectado ao GND quando pressionado),
# você pode precisar ajustar GPIO.PUD_UP para GPIO.PUD_DOWN ou remover o pull-up/down.
for pin in INPUT_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Exemplo: pull-up. Ajuste conforme seu hardware

# --- Configuração UART Raspberry Pi ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    while True:
        # Lê o estado dos pinos
        pin_states = []
        for pin in INPUT_PINS:
            # RPi.GPIO.input(pin) retorna 0 (LOW) ou 1 (HIGH)
            # Se você usou GPIO.PUD_UP, o pino estará HIGH por padrão e LOW quando conectado ao GND.
            # Se LOW = 0 (ativo) e HIGH = 1 (inativo), então é direto:
            state = GPIO.input(pin)
            # Se precisar inverter a lógica (ex: 0=Ativo, 1=Inativo, mas quer 1=Ativo, 0=Inativo na string):
            # state = 1 if GPIO.input(pin) == GPIO.LOW else 0 # Exemplo para ativo baixo
            pin_states.append(str(state))

        # Converte o estado dos pinos para um byte
        # A ordem dos bits na string importa para a Toradex.
        # Estamos mantendo a sequência P35 (bit mais significativo) até P38 (bit menos significativo)
        bit_string = "".join(pin_states)
        byte_to_send = int(bit_string, 2).to_bytes(1, 'big')

        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"{timestamp}: pin 35-38: {' '.join(pin_states)}")
        
        ser.write(byte_to_send)
        time.sleep(2)

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