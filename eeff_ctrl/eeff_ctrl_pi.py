import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime
import argparse
import threading
import sys

# DEBUG: Print para verificar se o script inicia
print("DEBUG: Script started and imports complete.") 

# --- Configuração GPIO Raspberry Pi ---
GPIO.setmode(GPIO.BOARD)

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

# Configura os pinos como entrada (sem pull-up/down, conforme seu teste gpio_pi.py)
for pin in INPUT_PINS_MAP.keys():
    GPIO.setup(pin, GPIO.IN)

# DEBUG: Print para verificar se o setup das GPIOs foi concluído
print("DEBUG: GPIO setup complete.") 

# --- Configuração UART Raspberry Pi ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 9600

# --- Configurações de Simulação (Constantes) ---
DEFAULT_SIMULATION_DELAY_SECONDS = 2
DEFAULT_UPDATE_INTERVAL_SECONDS = 1
SENSOR_RECOVERY_TIME_SECONDS = 10.0 # Tempo para o sensor se recuperar

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

try:
    args = parser.parse_args()
except SystemExit as e:
    print(f"ERROR: Argument parsing failed. Exit code: {e.code}", file=sys.stderr)
    print("Please check script usage: python3 eeff_ctrl_pi.py -t <delay> [-s <ids>] [-rec]", file=sys.stderr)
    sys.exit(e.code)


SIMULATION_DELAY_SECONDS = args.time
INITIAL_SENSOR_FAIL_IDS = sorted(list(set(args.sensor_fail)))
REC_ENABLED = args.rec

# DEBUG: Print para verificar se o parsing de argumentos foi concluído
print("DEBUG: Argument parsing complete.") 

# --- Debounce Time para Polling Manual EFETIVO (em segundos) ---
# O sinal deve ser estável por este tempo para ser considerado uma transição válida.
POLLING_DEBOUNCE_SECONDS = 0.05 # 50ms de estabilidade. Ajuste se o ruído for muito persistente.

# --- Estrutura de Estado para Cada Sensor ---
sensor_states = {}
for pin_gpio, sensor_id_val in INPUT_PINS_MAP.items():
    sensor_info = SENSOR_MAPPING[sensor_id_val]
    sensor_states[pin_gpio] = {
        'id': sensor_id_val, 
        'name': sensor_info['name'], 
        'current_output_bit': '0',         # O bit que será enviado na UART (0 ou 1)
        'is_pending_high_response': False, # True se detectou HIGH e está aguardando delay para mudar 'current_output_bit' para '1'
        'response_timer': None,            # threading.Timer para agendar a mudança para HIGH
        'is_currently_failing': False,     # Se o sensor está em estado de falha (não responde HIGH na 1a tentativa)
        'has_failed_once': False,          # True se já falhou uma vez e está aguardando recuperação
        'recovery_timer': None,            # threading.Timer para agendar a recuperação após falha
        'last_stable_signal': GPIO.LOW,    # Último sinal considerado estável após debounce
        'raw_signal_at_transition_start': GPIO.LOW, # Sinal lido no início de uma possível transição
        'signal_stable_start_time': 0      # Tempo em que o 'raw_signal_at_transition_start' começou
    }
    # Marca sensores que devem falhar inicialmente
    if sensor_id_val != 0 and sensor_id_val in INITIAL_SENSOR_FAIL_IDS: 
        sensor_states[pin_gpio]['is_currently_failing'] = True 
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG INIT: Sensor {sensor_info['name']} (ID {sensor_id_val}) marcado para falhar.")

# --- UART Serial Port Object ---
ser = None # Inicializa como None para ser aberto no try/finally

# --- Funções de Callback para Sensores de Atuadores (Vácuos e Cilindro) ---
def set_sensor_output_high(pin_gpio):
    """Função chamada após o delay para setar o bit de saída do sensor para HIGH."""
    state = sensor_states[pin_gpio]
    # SÓ MUDA PARA HIGH SE AINDA ESTÁ ESPERANDO UMA RESPOSTA HIGH PENDENTE
    if state['is_pending_high_response']: 
        state['current_output_bit'] = '1'
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} (ID {state['id']}) respondendo HIGH.")
    else: # Se não está mais pendente, um HIGH agendado foi cancelado ou já foi resetado por um LOW
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} (ID {state['id']}) - HIGH agendado ignorado (não mais pendente).")

    state['is_pending_high_response'] = False # Resposta entregue ou ignorada, não está mais pendente de agendamento


    # Se estava falhando e agora respondeu (o 'if is_pending_high_response' acima), ele se recupera
    if REC_ENABLED and state['is_currently_failing'] and state['current_output_bit'] == '1': # Só recupera se realmente ligou
        state['is_currently_failing'] = False
        state['has_failed_once'] = False
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Sensor {state['name']} (ID {state['id']}) recuperado e respondendo ao comando.")


def recover_sensor_from_failure(pin_gpio):
    """Função chamada após o timer de recuperação para resetar a flag de falha."""
    state = sensor_states[pin_gpio]
    state['is_currently_failing'] = False
    state['has_failed_once'] = False # Reseta a flag de "já falhou uma vez"
    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG REC TIMER: Sensor {state['name']} (ID {state['id']}): Tempo de recuperação transcorrido. Pronto para nova tentativa.")


# --- Main Setup and Loop ---
def main():
    global ser # Declara ser como global para ser acessível no finally
    try:
        # Zera todos os current_output_bit dos sensores ao iniciar o script.
        for pin_gpio in INPUT_PINS_MAP.keys():
            state = sensor_states[pin_gpio]
            state['current_output_bit'] = '0'
            state['last_stable_signal'] = GPIO.input(pin_gpio) # Lê o estado inicial real como estável
            state['raw_signal_at_transition_start'] = state['last_stable_signal']
            state['signal_stable_start_time'] = time.time() # Inicia o timer de estabilidade

            # Cancela e limpa quaisquer timers que possam ter ficado pendentes de uma execução anterior
            if state['response_timer'] and state['response_timer'].is_alive():
                state['response_timer'].cancel()
            state['response_timer'] = None 
            state['is_pending_high_response'] = False 
            
            if state['recovery_timer'] and state['recovery_timer'].is_alive():
                state['recovery_timer'].cancel()
            state['recovery_timer'] = None 

            # Reinicia o estado de falha para o estado inicial configurado por linha de comando
            state['is_currently_failing'] = (state['id'] in INITIAL_SENSOR_FAIL_IDS)
            state['has_failed_once'] = False 


        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # Timeout pequeno para não bloquear
        print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

        last_update_time = time.time()

        print("\n--- Configurações de Simulação ---")
        print(f"Delay de Resposta do Sensor: {SIMULATION_DELAY_SECONDS}s")
        if INITIAL_SENSOR_FAIL_IDS:
            failed_sensors_names = [SENSOR_MAPPING[sid]['name'] for sid in INITIAL_SENSOR_FAIL_IDS if sid in SENSOR_MAPPING and sid != 0]
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

            # --- Lógica de Polling com Debounce EFETIVO para TODOS os Pinos ---
            for pin_gpio in INPUT_PINS_MAP.keys():
                state = sensor_states[pin_gpio]
                current_raw_signal = GPIO.input(pin_gpio) # Lê o estado atual RAW do pino

                # --- Lógica de Debounce por Tempo ---
                # Se o sinal RAW mudou desde a última vez que o verificamos (para o debounce)
                if current_raw_signal != state['raw_signal_at_transition_start']:
                    state['raw_signal_at_transition_start'] = current_raw_signal # Registra o novo sinal RAW
                    state['signal_stable_start_time'] = current_time # Reinicia o timer de estabilidade
                
                # Se o sinal RAW é o mesmo de 'raw_signal_at_transition_start' E já passou o tempo de debounce,
                # E este sinal ainda não foi reconhecido como 'last_stable_signal'
                elif current_raw_signal == state['raw_signal_at_transition_start'] and \
                     (current_time - state['signal_stable_start_time']) >= POLLING_DEBOUNCE_SECONDS and \
                     current_raw_signal != state['last_stable_signal']:
                    
                    # O sinal está ESTÁVEL e é uma transição VÁLIDA!
                    stable_signal = current_raw_signal
                    state['last_stable_signal'] = stable_signal # Atualiza o último sinal estável
                    
                    # --- Lógica de Processamento do Comando ESTÁVEL (copiada de handle_gpio_event) ---
                    # --- LÓGICA ESPECÍFICA PARA O TOOL CHANGER (ID 0) ---
                    if pin_gpio == PIN_TOOL_CHANGER_IN:
                        state['current_output_bit'] = '1' if stable_signal == GPIO.HIGH else '0' # Espelha diretamente
                        if state['response_timer'] and state['response_timer'].is_alive():
                            state['response_timer'].cancel()
                            state['response_timer'] = None 
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Tool Changer input ({'HIGH' if stable_signal == GPIO.HIGH else 'LOW'}) directly mirrored. Simulated output: {state['current_output_bit']}.")
                    # --- LÓGICA PARA OS OUTROS SENSORES (VÁCUOS E CILINDRO) ---
                    else: 
                        if stable_signal == GPIO.HIGH: # Borda de Subida ESTÁVEL (LOW para HIGH)
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando HIGH detectado.")

                            # Cancela qualquer timer de resposta ou recuperação ativo ao receber um novo comando
                            if state['response_timer'] and state['response_timer'].is_alive():
                                state['response_timer'].cancel()
                                state['response_timer'] = None
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} - Timer de resposta anterior cancelado.")
                            if state['recovery_timer'] and state['recovery_timer'].is_alive():
                                state['recovery_timer'].cancel()
                                state['recovery_timer'] = None
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} - Timer de recuperação anterior cancelado.")

                            # Se o sensor está configurado para falhar E é a primeira tentativa, ele falha IMEDIATAMENTE.
                            if state['is_currently_failing'] and not state['has_failed_once']:
                                state['current_output_bit'] = '0' # A saída permanece LOW (falha)
                                state['has_failed_once'] = True # Marca que a 1a falha ocorreu
                                if REC_ENABLED:
                                    state['recovery_timer'] = threading.Timer(SENSOR_RECOVERY_TIME_SECONDS, recover_sensor_from_failure, args=[pin_gpio])
                                    state['recovery_timer'].start()
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) falhou na 1a tentativa. Início timer REC.")
                                else:
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) falhou (REC desabilitado).")
                                
                            # Se não está falhando ou já se recuperou (REC habilitado e não falhou antes nesta tentativa)
                            elif not state['is_currently_failing'] or (REC_ENABLED and state['has_failed_once'] and not state['is_currently_failing']):
                                state['is_pending_high_response'] = True # MARCA que está aguardando para responder HIGH (VERIFICADO NO CALLBACK set_sensor_output_high)
                                state['response_timer'] = threading.Timer(SIMULATION_DELAY_SECONDS, set_sensor_output_high, args=[pin_gpio])
                                state['response_timer'].start()
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) - Resposta HIGH agendada para {SIMULATION_DELAY_SECONDS}s.")

                        elif stable_signal == GPIO.LOW: # Borda de Descida ESTÁVEL (HIGH para LOW)
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando LOW detectado.")
                            state['current_output_bit'] = '0' # Seta a saída para LOW imediatamente
                            # Se um timer HIGH estava agendado, ele será ignorado pela lógica dentro de set_sensor_output_high
                            state['is_pending_high_response'] = False # Cancela explicitamente qualquer HIGH pendente

                            # Lógica de reset de flags de recuperação
                            if REC_ENABLED and state['is_currently_failing']: # Se REC habilitado e está em ciclo de falha ativa
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD REC (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando LOW enquanto em ciclo de falha. Estado de falha mantido.")
                            else: # Não em ciclo de falha ativa, reseta flags de falha
                                state['is_currently_failing'] = (state['id'] in INITIAL_SENSOR_FAIL_IDS) # Reverte ao estado inicial de falha
                                state['has_failed_once'] = False
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD NO REC (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando LOW. Flags de falha resetadas normalmente.")
                    
                    # state['last_polled_signal'] é agora last_stable_signal, atualizado acima
                    # state['last_transition_time'] também é atualizado no início do debounce
                
                # --- Opcional: Para debugar oscilações menores que o debounce ---
                # if current_raw_signal != state['last_polled_signal'] and \
                #    (current_time - state['last_transition_time']) <= POLLING_DEBOUNCE_SECONDS:
                #     # Isso é ruído que o debounce está filtrando
                #     print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] DEBUG PI: Noise on {state['name']}: {current_raw_signal} (raw) vs {state['last_polled_signal']} (last stable)")

            # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
            if (current_time - last_update_time) >= DEFAULT_UPDATE_INTERVAL_SECONDS:
                output_bit_string = ""
                debug_states_list = [] # Para coletar estados simulados para debug
                for sensor_id_val in range(len(SENSOR_MAPPING)): # Itera pelos IDs para garantir a ordem dos bits
                    pin = SENSOR_MAPPING[sensor_id_val]['pin'] # Obtém o pino correspondente ao ID
                    # O bit de saída é lido diretamente do estado do sensor
                    bit_val = sensor_states[pin]['current_output_bit'] 
                    output_bit_string += bit_val
                    debug_states_list.append(f"{SENSOR_MAPPING[sensor_id_val]['name']}:{bit_val}")

                byte_to_send = int(output_bit_string, 2).to_bytes(1, 'big')

                timestamp_ms = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp_ms}] PI SENDING (UART): Bits: {output_bit_string} (Byte: {byte_to_send.hex()}) - Current Simulated States: {', '.join(debug_states_list)}")
                ser.write(byte_to_send)

                last_update_time = current_time

            time.sleep(0.01) # Pequena pausa para evitar alto uso de CPU

    except serial.SerialException as e:
        print(f"Erro ao abrir ou usar a porta serial: {e}")
        print("Verifique se a porta serial está correta e se o console serial está desabilitado (se necessário).")
    except Exception as e:
        print(f"Ocorreu um erro: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Porta serial fechada.")
        GPIO.cleanup() # Limpa as configurações GPIO APENAS no final
        print("GPIOs limpas.")

if __name__ == "__main__":
    main()