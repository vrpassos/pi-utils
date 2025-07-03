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

# Configura os pinos como entrada com pull-down para garantir LOW se não conectados
for pin in INPUT_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# --- Configuração UART Raspberry Pi ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

# --- Configurações de Simulação padrão ---
DEFAULT_SIMULATION_DELAY_SECONDS = 2
DEFAULT_UPDATE_INTERVAL_SECONDS = 1

# --- Mapeamento de IDs de sensor para seus respectivos pinos e índices de bit na string de saída ---
SENSOR_MAPPING = {
    0: {'pin': PIN_TOOL_CHANGER_IN, 'name': 'Tool Changer', 'bit_index': 0},
    1: {'pin': PIN_VAC_INFERIOR_IN, 'name': 'Vácuo Inferior', 'bit_index': 1},
    2: {'pin': PIN_CILINDRO_IN, 'name': 'Cilindro', 'bit_index': 2},
    3: {'pin': PIN_VAC_SUPERIOR_IN, 'name': 'Vácuo Superior', 'bit_index': 3}
}

# --- Parsing de Argumentos de Linha de Comando ---
parser = argparse.ArgumentParser(description="Script de controle de atuadores Raspberry Pi com simulação de falha de sensor.")
parser.add_argument('-t', '--time', type=float, default=DEFAULT_SIMULATION_DELAY_SECONDS,
                    help=f"Tempo em segundos para os sensores responderem à mudança de 0 para 1 (padrão: {DEFAULT_SIMULATION_DELAY_SECONDS}s).")
parser.add_argument('-s', '--sensor_fail', type=int, nargs='*', default=[],
                    help="ID(s) do sensor que terá(ão) problema em responder (1: Vácuo Inferior, 2: Cilindro, 3: Vácuo Superior). Pode ser um ou múltiplos IDs separados por espaço.")
parser.add_argument('-rec', action='store_true', default=False,
                    help="Se presente, após a 1a tentativa falha, o sensor volta a funcionar após 10 segundos.")

args = parser.parse_args()

SIMULATION_DELAY_SECONDS = args.time
INITIAL_SENSOR_FAIL_IDS = sorted(list(set(args.sensor_fail)))
REC_ENABLED = args.rec

# --- Estrutura de Estado para Cada Sensor ---
sensor_states = {}
for sensor_id_val, sensor_info in SENSOR_MAPPING.items():
    pin = sensor_info['pin']
    sensor_states[pin] = {
        'simulated_output_state': GPIO.LOW, # O estado que a RPi envia pela UART
        'activation_start_time': None, # Quando o Pi REGISTROU um comando HIGH para iniciar a contagem do delay
        'is_pending_response': False, # NOVO: Indica que o Pi está aguardando seu próprio delay para responder HIGH
        'is_failing_first_attempt_complete': False, 
        'recovery_timer_start_time': None, 
        'is_ready_for_recovery_attempt': False, 
        'last_toradex_command_signal': GPIO.LOW, 
        'is_currently_failing': False, 
        'last_simulated_output_state': GPIO.LOW 
    }
    if sensor_id_val in INITIAL_SENSOR_FAIL_IDS:
        sensor_states[pin]['is_currently_failing'] = True 
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG INIT: Sensor {sensor_info['name']} (ID {sensor_id_val}) marcado como is_currently_failing=True.")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    last_update_time = time.time()

    print("\n--- Configurações de Simulação ---")
    print(f"Delay de Resposta do Sensor: {SIMULATION_DELAY_SECONDS}s")
    if INITIAL_SENSOR_FAIL_IDS:
        failed_sensors_names = [SENSOR_MAPPING[sid]['name'] for sid in INITIAL_SENSOR_FAIL_IDS if sid in SENSOR_MAPPING]
        print(f"Sensores com Falha (não respondem): {', '.join(failed_sensors_names)}")
    else:
        print("Todos os sensores funcionando normalmente.")
    print(f"Recuperação 'rec' habilitada: {'Sim' if REC_ENABLED else 'Não'}")
    print("----------------------------------\n")


    while True:
        current_time = time.time()

        for sensor_id_val, sensor_info in SENSOR_MAPPING.items():
            pin = sensor_info['pin']
            state = sensor_states[pin]

            # Lê o estado atual do pino de entrada (vindo da Toradex)
            command_signal_from_toradex = GPIO.input(pin)
            
            # Detecta bordas de subida e descida
            command_signal_changed_to_high = (command_signal_from_toradex == GPIO.HIGH and
                                              state['last_toradex_command_signal'] == GPIO.LOW)
            command_signal_changed_to_low = (command_signal_from_toradex == GPIO.LOW and
                                             state['last_toradex_command_signal'] == GPIO.HIGH)

            # DEBUG: Printa o estado das flags e inputs se houver mudança
            if (command_signal_from_toradex != state['last_toradex_command_signal'] or
                state['simulated_output_state'] != state['last_simulated_output_state']):
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] DEBUG STATE: Sensor {sensor_info['name']} (ID {sensor_id_val}) - Cmd: {command_signal_from_toradex} (PrevCmd: {state['last_toradex_command_signal']}), SimOutput: {state['simulated_output_state']}, Failing: {state['is_currently_failing']}, FirstFailCpl: {state['is_failing_first_attempt_complete']}, ReadyRec: {state['is_ready_for_recovery_attempt']}, PendingResp: {state['is_pending_response']}, ActStart: {state['activation_start_time'] is not None}, RecStart: {state['recovery_timer_start_time'] is not None}")
            
            # Atualiza o último estado de saída simulada para a próxima comparação de debug
            state['last_simulated_output_state'] = state['simulated_output_state']

            # Atualiza o último estado do comando para a próxima iteração
            state['last_toradex_command_signal'] = command_signal_from_toradex


            # --- Tratamento especial para Tool Changer (ID 0) ---
            if sensor_id_val == 0:
                state['simulated_output_state'] = command_signal_from_toradex
                # Reseta todas as flags de recuperação para garantir que não haja interferência
                state['activation_start_time'] = None
                state['is_pending_response'] = False # NOVO
                state['is_failing_first_attempt_complete'] = False
                state['recovery_timer_start_time'] = None
                state['is_ready_for_recovery_attempt'] = False 
                state['is_currently_failing'] = False 
                continue # Pula para o próximo sensor no loop


            # --- Lógica do Timer de Recuperação (10 segundos) ---
            if REC_ENABLED and state['is_failing_first_attempt_complete'] and state['recovery_timer_start_time'] is not None:
                if current_time - state['recovery_timer_start_time'] >= 10:
                    if not state['is_ready_for_recovery_attempt']: 
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG REC TIMER: Sensor {sensor_info['name']}: Tempo de recuperação (rec) transcorrido. Aguardando novo acionamento.")
                    state['is_ready_for_recovery_attempt'] = True 
                    state['recovery_timer_start_time'] = None


            # --- Lógica Principal de Simulação do Sensor (Comandos HIGH/LOW) ---

            # Etapa 1: Detectar a borda de subida e iniciar um novo ciclo de resposta/falha
            if command_signal_changed_to_high:
                # Se o sensor está atualmente falhando E é a primeira tentativa, ele falha IMEDIATAMENTE.
                if state['is_currently_failing'] and not state['is_failing_first_attempt_complete']:
                    state['simulated_output_state'] = GPIO.LOW # O sensor falha, saída permanece LOW
                    if REC_ENABLED: 
                        state['is_failing_first_attempt_complete'] = True # Marca que a primeira falha ocorreu
                        state['recovery_timer_start_time'] = current_time # Inicia o timer de 10s de recuperação
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {sensor_info['name']} falhou na 1a tentativa e iniciou timer REC.")
                    
                    # Nenhuma resposta HIGH é esperada, então não setamos is_pending_response para TRUE aqui.
                    # O processo para este comando HIGH é "concluído" com a falha registrada.
                    state['activation_start_time'] = None # Clear activation time, as no delayed response is coming
                    state['is_pending_response'] = False # Ensure this is false

                # Se o sensor NÃO está falhando OU está pronto para recuperação, inicia o processo de resposta
                elif not state['is_currently_failing'] or (REC_ENABLED and state['is_currently_failing'] and state['is_ready_for_recovery_attempt']):
                    state['activation_start_time'] = current_time # Registra o momento do HIGH
                    state['is_pending_response'] = True # Marcamos que estamos esperando para responder HIGH
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND: Sensor {sensor_info['name']}: Comando HIGH detectado (borda de subida). Iniciando contagem de delay para resposta.")
                    # Se for o cenário de recuperação, limpa ReadyRec aqui (pois estamos iniciando a tentativa de recuperação)
                    if REC_ENABLED and state['is_currently_failing'] and state['is_ready_for_recovery_attempt']:
                        state['is_ready_for_recovery_attempt'] = False # Consumimos o estado de "pronto para recuperar"


            # Etapa 2: Checar se algum sensor pendente deve responder HIGH
            # Esta lógica agora é independente do estado atual do pino de entrada (Cmd)
            if state['is_pending_response'] and state['activation_start_time'] is not None:
                if (current_time - state['activation_start_time']) >= SIMULATION_DELAY_SECONDS:
                    state['simulated_output_state'] = GPIO.HIGH # O sensor finalmente responde HIGH
                    
                    # Concluímos o ciclo de resposta HIGH
                    state['is_pending_response'] = False
                    state['activation_start_time'] = None 

                    # Se a resposta foi bem-sucedida para um sensor que estava falhando (REC)
                    if state['is_currently_failing']: # means REC was enabled and it just responded
                        state['is_currently_failing'] = False 
                        state['is_failing_first_attempt_complete'] = False
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Sensor {sensor_info['name']} recuperado (rec) e respondendo ao comando.")
                    else: # Resposta normal
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {sensor_info['name']} respondendo HIGH normalmente.")


            # Etapa 3: Comando LOW da Toradex (sempre reseta a saída simulada imediatamente)
            if command_signal_changed_to_low:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND: Sensor {sensor_info['name']}: Comando LOW detectado (borda de descida).")
                state['simulated_output_state'] = GPIO.LOW # A saída simulada vai para LOW imediatamente
                
                # IMPORTANTE: NÃO reseta is_pending_response ou activation_start_time AQUI.
                # Se o Pi já começou a contar o delay para responder HIGH, ele deve continuar contando
                # e responder HIGH quando o delay expirar, MESMO que a Toradex tenha desligado o pino.
                # Isso simula o sensor concluindo seu ciclo.
                
                # Lógica de reset de flags de recuperação (APENAS se não estiver em um ciclo REC ativo)
                if not REC_ENABLED: 
                    state['is_failing_first_attempt_complete'] = False
                    state['recovery_timer_start_time'] = None
                    state['is_ready_for_recovery_attempt'] = False
                elif REC_ENABLED:
                    # Se estiver em processo de falha/recuperação REC, não resetar as flags de recuperação
                    if state['is_currently_failing'] or state['is_failing_first_attempt_complete'] or state['is_ready_for_recovery_attempt']:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD REC: Sensor {sensor_info['name']}: LOW enquanto falhando/recuperando. Flags de recuperação NÃO resetadas.")
                    else: # Se REC está habilitado mas não está em um ciclo de falha/recuperação ativo, pode resetar
                        state['is_failing_first_attempt_complete'] = False
                        state['recovery_timer_start_time'] = None
                        state['is_ready_for_recovery_attempt'] = False


        # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
        if (current_time - last_update_time) >= DEFAULT_UPDATE_INTERVAL_SECONDS:
            output_bit_string = ""
            for sensor_id_val in range(len(SENSOR_MAPPING)):
                pin = SENSOR_MAPPING[sensor_id_val]['pin']
                output_bit_string += str(sensor_states[pin]['simulated_output_state'])

            byte_to_send = int(output_bit_string, 2).to_bytes(1, 'big')

            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"{timestamp}: Saídas da Toradex: {output_bit_string[0]} {output_bit_string[1]} {output_bit_string[2]} {output_bit_string[3]}")
            ser.write(byte_to_send)

            last_update_time = current_time

        time.sleep(0.01) # Pequena pausa para evitar alto uso de CPU

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