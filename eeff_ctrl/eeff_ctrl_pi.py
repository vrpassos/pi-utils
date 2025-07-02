import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime
import argparse

# --- Configuração GPIO Raspberry Pi ---
GPIO.setmode(GPIO.BOARD)

# --- Mapeamento dos Pinos Físicos do Raspberry Pi (ENTRADAS) ---
PIN_TOOL_CHANGER_IN = 35
PIN_VAC_INFERIOR_IN = 36
PIN_CILINDRO_IN = 37
PIN_VAC_SUPERIOR_IN = 38

INPUT_PINS = [PIN_TOOL_CHANGER_IN, PIN_VAC_INFERIOR_IN, PIN_CILINDRO_IN, PIN_VAC_SUPERIOR_IN]

for pin in INPUT_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# --- Configuração UART Raspberry Pi ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

# --- Configurações de Simulação padrão ---
DEFAULT_SIMULATION_DELAY_SECONDS = 2
DEFAULT_UPDATE_INTERVAL_SECONDS = 1
DEFAULT_SENSOR_FAIL_IDS = []
DEFAULT_REC_ENABLED = False # Alterado de DEX_ENABLED para REC_ENABLED

# Mapeamento de IDs de sensor para seus respectivos pinos e índices de bit na string de saída
SENSOR_MAPPING = {
    1: {'pin': PIN_VAC_INFERIOR_IN, 'name': 'Vácuo Inferior', 'bit_index': 1},
    2: {'pin': PIN_CILINDRO_IN, 'name': 'Cilindro', 'bit_index': 2},
    3: {'pin': PIN_VAC_SUPERIOR_IN, 'name': 'Vácuo Superior', 'bit_index': 3}
}

# --- Parsing de Argumentos de Linha de Comando ---
parser = argparse.ArgumentParser(description="Script de controle de atuadores Raspberry Pi com simulação de falha de sensor.")
parser.add_argument('-t', '--time', type=float, default=DEFAULT_SIMULATION_DELAY_SECONDS,
                    help=f"Tempo em segundos para os sensores responderem à mudança de 0 para 1 (padrão: {DEFAULT_SIMULATION_DELAY_SECONDS}s).")
parser.add_argument('-s', '--sensor_fail', type=int, nargs='*', default=DEFAULT_SENSOR_FAIL_IDS,
                    help="ID(s) do sensor que terá(ão) problema em responder (1: Vácuo Inferior, 2: Cilindro, 3: Vácuo Superior). Pode ser um ou múltiplos IDs separados por espaço.")
parser.add_argument('-rec', action='store_true', default=DEFAULT_REC_ENABLED, # Alterado de -dex para -rec
                    help="Se presente, após a primeira tentativa falha, o sensor volta a funcionar após 10 segundos.")

args = parser.parse_args()

SIMULATION_DELAY_SECONDS = args.time
SENSOR_FAIL_IDS = args.sensor_fail
REC_ENABLED = args.rec # Alterado de DEX_ENABLED para REC_ENABLED

# Variáveis de estado para simulação de delay
simulation_states = {
    PIN_VAC_INFERIOR_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None, 'failed_once': False, 'rec_recovery_time': None}, # Alterado de dex_recovery_time para rec_recovery_time
    PIN_CILINDRO_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None, 'failed_once': False, 'rec_recovery_time': None},    # Alterado de dex_recovery_time para rec_recovery_time
    PIN_VAC_SUPERIOR_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None, 'failed_once': False, 'rec_recovery_time': None}  # Alterado de dex_recovery_time para rec_recovery_time
}

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    last_update_time = time.time()

    # Imprime as configurações de simulação ativas
    print("\n--- Configurações de Simulação ---")
    print(f"Delay de Resposta do Sensor: {SIMULATION_DELAY_SECONDS}s")
    if SENSOR_FAIL_IDS:
        failed_sensors_names = [SENSOR_MAPPING[sid]['name'] for sid in SENSOR_FAIL_IDS if sid in SENSOR_MAPPING]
        print(f"Sensores com Falha (não respondem): {', '.join(failed_sensors_names)}")
    else:
        print("Todos os sensores funcionando normalmente.")
    print(f"Recuperação 'rec' habilitada: {'Sim' if REC_ENABLED else 'Não'}") # Alterado de 'dex' para 'rec'
    print("----------------------------------\n")


    while True:
        current_time = time.time()

        # --- Leitura das ENTRADAS GPIO do Raspberry Pi (que recebem da Toradex) ---
        tc_command_signal = GPIO.input(PIN_TOOL_CHANGER_IN)
        vi_command_signal = GPIO.input(PIN_VAC_INFERIOR_IN)
        cil_command_signal = GPIO.input(PIN_CILINDRO_IN)
        vs_command_signal = GPIO.input(PIN_VAC_SUPERIOR_IN)

        # --- Processa a Lógica de Simulação para o Feedback ---

        # 1. Tool Changer (Pino 35) - Sem Sensoriamento/Delay
        tool_changer_report_state = tc_command_signal

        # Iterar sobre os pinos que possuem simulação de delay e/ou falha
        simulated_pins_info = [
            (PIN_VAC_INFERIOR_IN, vi_command_signal, 1),
            (PIN_CILINDRO_IN, cil_command_signal, 2),
            (PIN_VAC_SUPERIOR_IN, vs_command_signal, 3)
        ]

        for pin_id, command_signal_from_toradex, sensor_id in simulated_pins_info:
            state_info = simulation_states[pin_id]

            # Lógica de recuperação 'rec'
            if REC_ENABLED and state_info['failed_once'] and state_info['rec_recovery_time'] is not None: # Alterado de dex_recovery_time para rec_recovery_time
                if current_time - state_info['rec_recovery_time'] >= 10: # 10 segundos para recuperação # Alterado de dex_recovery_time para rec_recovery_time
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Sensor {SENSOR_MAPPING[sensor_id]['name']} recuperado (rec).") # Alterado de dex para rec
                    if sensor_id in SENSOR_FAIL_IDS:
                        SENSOR_FAIL_IDS.remove(sensor_id)
                    state_info['rec_recovery_time'] = None # Alterado de dex_recovery_time para rec_recovery_time
                    state_info['failed_once'] = False

            # Lógica de simulação de falha
            if sensor_id in SENSOR_FAIL_IDS and not REC_ENABLED: # Alterado de DEX_ENABLED para REC_ENABLED
                state_info['simulated_output_state'] = GPIO.LOW
                state_info['start_time'] = None
                continue

            if command_signal_from_toradex == GPIO.HIGH:
                if state_info['simulated_output_state'] == GPIO.LOW:
                    if state_info['start_time'] is None:
                        state_info['start_time'] = current_time

                    if (current_time - state_info['start_time']) >= SIMULATION_DELAY_SECONDS:
                        if REC_ENABLED and sensor_id in SENSOR_FAIL_IDS and not state_info['failed_once']: # Alterado de DEX_ENABLED para REC_ENABLED
                            state_info['failed_once'] = True
                            state_info['rec_recovery_time'] = current_time # Alterado de dex_recovery_time para rec_recovery_time
                            state_info['simulated_output_state'] = GPIO.LOW
                        else:
                            state_info['simulated_output_state'] = GPIO.HIGH
                            state_info['start_time'] = None

            elif command_signal_from_toradex == GPIO.LOW:
                state_info['simulated_output_state'] = GPIO.LOW
                state_info['start_time'] = None
                state_info['failed_once'] = False
                state_info['rec_recovery_time'] = None # Alterado de dex_recovery_time para rec_recovery_time


        # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
        if (current_time - last_update_time) >= DEFAULT_UPDATE_INTERVAL_SECONDS:
            state_tc = str(tool_changer_report_state)
            state_vi = str(simulation_states[PIN_VAC_INFERIOR_IN]['simulated_output_state'])
            state_cil = str(simulation_states[PIN_CILINDRO_IN]['simulated_output_state'])
            state_vs = str(simulation_states[PIN_VAC_SUPERIOR_IN]['simulated_output_state'])

            output_bit_string = state_tc + state_vi + state_cil + state_vs
            byte_to_send = int(output_bit_string, 2).to_bytes(1, 'big')

            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"{timestamp}: Saídas da Toradex: {output_bit_string[0]} {output_bit_string[1]} {output_bit_string[2]} {output_bit_string[3]}")
            ser.write(byte_to_send)

            last_update_time = current_time

        time.sleep(0.01)

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
    GPIO.cleanup()
    print("GPIOs limpas.")