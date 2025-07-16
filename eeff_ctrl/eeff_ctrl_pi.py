import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime
import argparse

# --- Configuração GPIO Raspberry Pi ---
GPIO.setmode(GPIO.BOARD)

# Limpeza inicial das GPIOs. O RuntimeWarning pode aparecer, mas é inofensivo aqui.
GPIO.cleanup() 

# --- Mapeamento dos Pinos Físicos do Raspberry Pi (ENTRADAS) ---
PIN_TOOL_CHANGER_IN = 35
PIN_VAC_INFERIOR_IN = 36
PIN_CILINDRO_IN = 37
PIN_VAC_SUPERIOR_IN = 38

# Mapeamento de Pinos de Entrada para seus IDs de Sensor correspondentes
INPUT_PINS_MAP = {
    PIN_TOOL_CHANGER_IN: 0,
    PIN_VAC_INFERIOR_IN: 1,
    PIN_CILINDRO_IN: 2,
    PIN_VAC_SUPERIOR_IN: 3
}

# Configura os pinos como entrada com pull-down
for pin in INPUT_PINS_MAP.keys():
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

# --- Debounce Time para detecção manual ---
DEBOUNCE_TIME_SECONDS = 0.5 # AUMENTADO para 500ms (era 0.3)

# --- Estrutura de Estado para Cada Sensor ---
sensor_states = {}
for pin_gpio, sensor_id_val in INPUT_PINS_MAP.items(): # Itera sobre os pinos GPIO para inicializar estados
    sensor_info = SENSOR_MAPPING[sensor_id_val] # Obtém info do sensor pelo ID
    sensor_states[pin_gpio] = { # Chave é o PINO GPIO
        'id': sensor_id_val, 
        'name': sensor_info['name'], 
        'simulated_output_state': GPIO.LOW, 
        'activation_start_time': None, 
        'is_pending_response': False, 
        'is_failing_first_attempt_complete': False, 
        'recovery_timer_start_time': None, 
        'is_ready_for_recovery_attempt': False, 
        'last_toradex_command_signal': GPIO.LOW, # Último estado lido do pino físico
        'is_currently_failing': False, 
        'last_simulated_output_state': GPIO.LOW,
        'last_transition_time': 0 # Timestamp da última transição (HIGH ou LOW) válida
    }
    # O Tool Changer (ID 0) NUNCA deve ser marcado como falha inicial.
    # Apenas sensores com ID em INITIAL_SENSOR_FAIL_IDS (1, 2, 3) podem falhar.
    if sensor_id_val != 0 and sensor_id_val in INITIAL_SENSOR_FAIL_IDS: 
        sensor_states[pin_gpio]['is_currently_failing'] = True 
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG INIT: Sensor {sensor_info['name']} (ID {sensor_id_val}) marcado como is_currently_failing=True.")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    last_update_time = time.time()

    print("\n--- Configurações de Simulação ---")
    print(f"Delay de Resposta do Sensor: {SIMULATION_DELAY_SECONDS}s")
    if INITIAL_SENSOR_FAIL_IDS:
        failed_sensors_names = [SENSOR_MAPPING[sid]['name'] for sid in INITIAL_SENSOR_FAIL_IDS if sid in SENSOR_MAPPING and sid != 0] # Exclui Tool Changer
        if failed_sensors_names:
            print(f"Sensores com Falha (não respondem): {', '.join(failed_sensors_names)}")
        else:
            print("Todos os sensores funcionando normalmente (nenhum sensor de atraso/falha selecionado).")
    else:
        print("Todos os sensores funcionando normalmente.")
    print(f"Recuperação 'rec' habilitada: {'Sim' if REC_ENABLED else 'Não'}")
    print("----------------------------------\n")


    while True:
        current_time = time.time()
        # DEBUG: Imprime o current_time no início de cada iteração
        # print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] DEBUG PI Loop: current_time={current_time:.3f}")

        for pin_gpio in INPUT_PINS_MAP.keys(): # Itera sobre os pinos GPIO lidos
            state = sensor_states[pin_gpio]
            
            # Lê o estado atual do pino de entrada (vindo da Toradex)
            current_command_signal = GPIO.input(pin_gpio)
            
            # Lógica de Debounce Manual e Detecção de Borda
            command_signal_changed = (current_command_signal != state['last_toradex_command_signal'])

            if command_signal_changed:
                if (current_time - state['last_transition_time']) > DEBOUNCE_TIME_SECONDS:
                    # Transição válida (não é ruído)
                    state['last_transition_time'] = current_time # Atualiza o tempo da última transição válida
                    
                    # --- LÓGICA ESPECÍFICA PARA O TOOL CHANGER (ID 0) ---
                    if pin_gpio == PIN_TOOL_CHANGER_IN:
                        # O Tool Changer espelha o input imediatamente, sem atraso ou lógica de falha
                        state['simulated_output_state'] = current_command_signal
                        state['is_pending_response'] = False # Não há pendência para o TC
                        state['activation_start_time'] = None # Não há timer para o TC
                        state['is_currently_failing'] = False # Não aplicável ao TC
                        state['is_failing_first_attempt_complete'] = False # Não aplicável ao TC
                        state['recovery_timer_start_time'] = None # Não aplicável ao TC
                        state['is_ready_for_recovery_attempt'] = False # Não aplicável ao TC
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Tool Changer input ({'HIGH' if current_command_signal == GPIO.HIGH else 'LOW'}) directly mirrored. Simulated output: {'HIGH' if state['simulated_output_state'] == GPIO.HIGH else 'LOW'}.")
                    # --- LÓGICA PARA OS OUTROS SENSORES (VÁCUOS E CILINDRO) ---
                    else: 
                        if current_command_signal == GPIO.HIGH: # Borda de Subida (LOW para HIGH)
                            # Cenário 1: Sensor está configurado para falhar E é a primeira tentativa, ele falha IMEDIATAMENTE.
                            if state['is_currently_failing'] and not state['is_failing_first_attempt_complete']:
                                state['simulated_output_state'] = GPIO.LOW # O sensor falha, saída permanece LOW
                                if REC_ENABLED: 
                                    state['is_failing_first_attempt_complete'] = True # Marca que a primeira falha ocorreu
                                    state['recovery_timer_start_time'] = current_time # Inicia o timer de 10s de recuperação
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} falhou na 1a tentativa e iniciou timer REC.")
                                
                                state['activation_start_time'] = None # Não há delay de resposta a ser esperado aqui
                                state['is_pending_response'] = False # Não está pendente de uma resposta HIGH
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOGIC: Sensor {state['name']}: Cenário 1 - Primeira falha registrada.")

                            # Cenário 2/3: Sensor está pronto para recuperar OU não está falhando. Inicia processo de resposta normal.
                            elif not state['is_currently_failing'] or (REC_ENABLED and state['is_currently_failing'] and state['is_ready_for_recovery_attempt']):
                                state['activation_start_time'] = current_time # Registra o momento do HIGH
                                state['is_pending_response'] = True # Marcamos que estamos esperando para responder HIGH
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND: Sensor {state['name']}: Comando HIGH detectado (borda de subida). Iniciando contagem de delay para resposta.")
                                
                                if REC_ENABLED and state['is_currently_failing'] and state['is_ready_for_recovery_attempt']:
                                    state['is_ready_for_recovery_attempt'] = False # Consumimos o estado de "pronto para recuperar"
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOGIC: Sensor {state['name']}: Cenário 2 - Tentativa de recuperação iniciada.")
                                else:
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOGIC: Sensor {state['name']}: Cenário 3 - Resposta normal iniciada.")

                        elif current_command_signal == GPIO.LOW: # Borda de Descida (HIGH para LOW)
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND: Sensor {state['name']}: Comando LOW detectado (borda de descida).")
                            state['simulated_output_state'] = GPIO.LOW # A saída simulada vai para LOW imediatamente
                            
                            # Lógica de reset de flags de recuperação (APENAS se não estiver em um ciclo REC ativo)
                            if not REC_ENABLED: 
                                state['is_failing_first_attempt_complete'] = False
                                state['recovery_timer_start_time'] = None
                                state['is_ready_for_recovery_attempt'] = False
                            elif REC_ENABLED:
                                if state['is_currently_failing'] or state['is_failing_first_attempt_complete'] or state['is_ready_for_recovery_attempt']:
                                    # Se o sensor estava falhando/recuperando e recebeu um comando LOW,
                                    # ele deve parar de "esperar para responder" mas NÃO resetar o estado de falha/recuperação
                                    # até que seja ativado novamente e se recupere (ou continue a falhar).
                                    # No entanto, se o comando LOW encerrou a tentativa de ativação, a flag pending deve ser limpa.
                                    state['is_pending_response'] = False
                                    state['activation_start_time'] = None
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD REC: Sensor {state['name']}: LOW enquanto falhando/recuperando. Pending response limpa. Flags REC NÃO resetadas.")
                                else: # Se REC está habilitado mas não está em um ciclo de falha/recuperação ativo, pode resetar
                                    state['is_failing_first_attempt_complete'] = False
                                    state['recovery_timer_start_time'] = None
                                    state['is_ready_for_recovery_attempt'] = False
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD NO REC: Sensor {state['name']}: LOW. Flags REC resetadas normalmente.")


            # Atualiza o last_toradex_command_signal para a próxima iteração
            state['last_toradex_command_signal'] = current_command_signal


            # --- Lógica do Timer de Recuperação (10 segundos) ---
            # Esta lógica só se aplica a sensores que podem falhar (IDs 1, 2, 3)
            if pin_gpio != PIN_TOOL_CHANGER_IN: 
                if REC_ENABLED and state['is_failing_first_attempt_complete'] and state['recovery_timer_start_time'] is not None:
                    if current_time - state['recovery_timer_start_time'] >= 10:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG REC TIMER: Sensor {state['name']}: Tempo de recuperação (rec) transcorrido. Aguardando novo acionamento.")
                        state['is_ready_for_recovery_attempt'] = True 
                        state['recovery_timer_start_time'] = None


            # --- Checar se algum sensor pendente deve responder HIGH (independente do estado atual do pino de entrada) ---
            # Esta lógica só se aplica a sensores que podem ter delay (IDs 1, 2, 3)
            if pin_gpio != PIN_TOOL_CHANGER_IN: 
                if state['is_pending_response'] and state['activation_start_time'] is not None:
                    # DEBUG: Tempo decorrido desde o comando
                    # delta_time_pending = current_time - state['activation_start_time']
                    # print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] DEBUG PI Delay Calc: Sensor {state['name']}. Delta: {delta_time_pending:.3f}s / Delay: {SIMULATION_DELAY_SECONDS}s")

                    if (current_time - state['activation_start_time']) >= SIMULATION_DELAY_SECONDS:
                        state['simulated_output_state'] = GPIO.HIGH # O sensor finalmente responde HIGH
                        
                        # Concluímos o ciclo de resposta HIGH
                        state['is_pending_response'] = False
                        state['activation_start_time'] = None 

                        # Se a resposta foi bem-sucedida para um sensor que estava falhando (REC)
                        if REC_ENABLED and state['is_currently_failing']: # means REC was enabled and it just responded
                            state['is_currently_failing'] = False 
                            state['is_failing_first_attempt_complete'] = False
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] Sensor {state['name']} recuperado (rec) e respondendo ao comando.")
                        else: # Resposta normal
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} respondendo HIGH normalmente.")


        # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
        if (current_time - last_update_time) >= DEFAULT_UPDATE_INTERVAL_SECONDS:
            output_bit_string = ""
            debug_states_list = [] # Para coletar estados simulados para debug
            for sensor_id_val in range(len(SENSOR_MAPPING)): # Itera pelos IDs para garantir a ordem dos bits
                pin = SENSOR_MAPPING[sensor_id_val]['pin'] # Obtém o pino correspondente ao ID
                bit_val = str(sensor_states[pin]['simulated_output_state'])
                output_bit_string += bit_val
                debug_states_list.append(f"{SENSOR_MAPPING[sensor_id_val]['name']}:{bit_val}")

            byte_to_send = int(output_bit_string, 2).to_bytes(1, 'big')

            timestamp_ms = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp_ms}] PI SENDING (UART): Bits: {output_bit_string} (Byte: {byte_to_send.hex()}) - Current Simulated States: {', '.join(debug_states_list)}")
            ser.write(byte_to_send)

            last_update_time = current_time

        time.sleep(0.05) # Aumentado para 50ms para reduzir carga da CPU. Era 0.01

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