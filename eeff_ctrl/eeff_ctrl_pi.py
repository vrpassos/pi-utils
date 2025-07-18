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
DEFAULT_SIMULATION_DELAY_SECONDS = 1.0 # Reduzido para 1 segundo
DEFAULT_UPDATE_INTERVAL_SECONDS = 1.0 # Reduzido para 0.1s para envio mais rápido
SENSOR_RECOVERY_TIME_SECONDS = 1.0 # Tempo de recuperação é 1 segundo

# --- Mapeamento de IDs de sensor para seus respectivos pinos e índices de bit na string de saída ---
# Os IDs aqui são internos da Pi (0, 1, 2, 3) e correspondem à posição do bit.
SENSOR_MAPPING = {
    0: {'pin': PIN_TOOL_CHANGER_IN, 'name': 'Tool Changer', 'bit_index': 0},
    1: {'pin': PIN_VAC_INFERIOR_IN, 'name': 'Vácuo Inferior', 'bit_index': 1},
    2: {'pin': PIN_CILINDRO_IN, 'name': 'Cilindro', 'bit_index': 2},
    3: {'pin': PIN_VAC_SUPERIOR_IN, 'name': 'Vácuo Superior', 'bit_index': 3}
}

# --- Mapeamento do Número de Comando da Toradex para o ID Interno do Sensor da Pi ---
# Isso permite que o usuário use os números de comando da Toradex (2, 3, 4) para -s.
CMD_NUM_TO_SENSOR_ID = {
    2: 1, # Comando '2' (Vácuo Inferior) -> ID interno do sensor '1'
    3: 2, # Comando '3' (Cilindro)      -> ID interno do sensor '2'
    4: 3  # Comando '4' (Vácuo Superior) -> ID interno do sensor '3'
}

# --- Parsing de Argumentos de Linha de Comando ---
parser = argparse.ArgumentParser(description="Script de controle de atuadores Raspberry Pi com simulação de falha de sensor.")
parser.add_argument('-t', '--time', type=float, default=DEFAULT_SIMULATION_DELAY_SECONDS,
                    help=f"Tempo em segundos para os sensores responderem à mudança de 0 para 1 (padrão: {DEFAULT_SIMULATION_DELAY_SECONDS}s).")
parser.add_argument('-s', '--sensor_fail', type=int, nargs='*', default=[],
                    help=f"Número(s) do comando da Toradex para o(s) sensor(es) que terá(ão) problema em responder (Vácuo Inferior: 2, Cilindro: 3, Vácuo Superior: 4). Pode ser um ou múltiplos números separados por espaço.")
parser.add_argument('-rec', action='store_true', default=False,
                    help="Se presente, após a 1a tentativa falha, o sensor volta a funcionar após 10 segundos.")

try:
    args = parser.parse_args()
except SystemExit as e:
    print(f"ERROR: Argument parsing failed. Exit code: {e.code}", file=sys.stderr)
    print("Please check script usage: python3 eeff_ctrl_pi.py -t <delay> [-s <cmd_numbers>] [-rec]", file=sys.stderr)
    sys.exit(e.code)


SIMULATION_DELAY_SECONDS = args.time
REC_ENABLED = args.rec

# Processa os números de comando fornecidos para -s e os traduz para IDs de sensor internos
FAILED_SENSOR_IDS_INTERNAL = []
for cmd_num_arg in args.sensor_fail:
    if cmd_num_arg == 1:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] WARNING: Tool Changer (comando 1) não pode ser configurado para falhar. Ignorando -s 1.", file=sys.stderr)
    elif cmd_num_arg in CMD_NUM_TO_SENSOR_ID:
        FAILED_SENSOR_IDS_INTERNAL.append(CMD_NUM_TO_SENSOR_ID[cmd_num_arg])
    else:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] WARNING: Número de comando de sensor inválido para falha: {cmd_num_arg}. Ignorando.", file=sys.stderr)

# DEBUG: Print para verificar se o parsing de argumentos foi concluído
print("DEBUG: Argument parsing complete.")

# --- Debounce Time para Polling Manual EFETIVO (em segundos) ---
# O sinal deve ser estável por este tempo para ser considerado uma transição válida.
POLLING_DEBOUNCE_SECONDS = 0.2 # 50ms de estabilidade. Ajuste se o ruído for muito persistente.

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
        'has_failed_once_in_cycle': False, # True se já falhou uma vez neste ciclo de comando HIGH/LOW
        'permanently_failed_this_run': False, # True se já falhou uma vez nesta execução do script
        'recovery_timer': None,            # threading.Timer para agendar a recuperação após falha
        'last_stable_signal': GPIO.LOW,    # Último sinal considerado estável após debounce
        'raw_signal_at_transition_start': GPIO.LOW, # Sinal lido no início de uma possível transição
        'signal_stable_start_time': 0      # Tempo em que o 'raw_signal_at_transition_start' começou
    }
    # Marca sensores que devem falhar inicialmente (usando os IDs internos processados)
    if sensor_id_val in FAILED_SENSOR_IDS_INTERNAL:
        sensor_states[pin_gpio]['is_currently_failing'] = True # Estado inicial de falha
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG INIT: Sensor {sensor_info['name']} (ID {sensor_id_val}) marcado para falhar.")

# --- UART Serial Port Object ---
ser = None # Inicializa como None para ser aberto no try/finally

# --- Funções de Callback para Sensores de Atuadores (Vácuos e Cilindro) ---
def set_sensor_output_high(pin_gpio):
    """Função chamada após o delay para setar o bit de saída do sensor para HIGH."""
    global last_update_time # Adicionado para forçar update da UART
    state = sensor_states[pin_gpio]
    # SÓ MUDA PARA HIGH SE AINDA ESTÁ ESPERANDO UMA RESPOSTA HIGH PENDENTE
    if state['is_pending_high_response']:
        state['current_output_bit'] = '1'
        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} (ID {state['id']}) respondendo HIGH. Bit simulado atualizado para '1'.")
        last_update_time = time.time() # FORÇA UM ENVIO IMEDIATO NO PRÓXIMO CICLO DO LOOP PRINCIPAL

    state['is_pending_high_response'] = False # Resposta entregue ou ignorada, não está mais pendente de agendamento


def recover_sensor_from_failure(pin_gpio):
    """Função chamada após o timer de recuperação para resetar a flag de falha.
    Esta função apenas prepara o sensor para a próxima tentativa, se o LOW de reset não tiver chegado antes."""
    state = sensor_states[pin_gpio]
    state['is_currently_failing'] = False # O sensor está pronto para tentar novamente
    state['recovery_timer'] = None # Limpa a referência ao timer

    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG REC TIMER: Sensor {state['name']} (ID {state['id']}): Tempo de recuperação transcorrido. Pronto para nova tentativa.")


# --- Main Setup and Loop ---
def main():
    global ser
    global last_update_time # <-- MOVIDO PARA O TOPO DA FUNÇÃO MAIN
    try:
        # Zera todos os current_output_bit dos sensores ao iniciar o script.
        for pin_gpio in INPUT_PINS_MAP.keys():
            state = sensor_states[pin_gpio]
            state['current_output_bit'] = '0'
            state['last_stable_signal'] = GPIO.input(pin_gpio) # Lê o estado inicial real como estável
            state['raw_signal_at_transition_start'] = state['last_stable_signal'] # Inicializa com o estado real lido
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
            state['is_currently_failing'] = (state['id'] in FAILED_SENSOR_IDS_INTERNAL)
            state['has_failed_once_in_cycle'] = False # Zera para o novo ciclo HIGH/LOW
            state['permanently_failed_this_run'] = False # Garante que a flag de falha única seja resetada por execução

        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # Timeout pequeno para não bloquear
        print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

        last_update_time = time.time() # Inicializa last_update_time AQUI, APÓS O GLOBAL

        print("\n--- Configurações de Simulação ---")
        print(f"Delay de Resposta do Sensor: {SIMULATION_DELAY_SECONDS}s")
        if FAILED_SENSOR_IDS_INTERNAL: # Verifica a lista já processada
            failed_sensors_names = [SENSOR_MAPPING[sid]['name'] for sid in FAILED_SENSOR_IDS_INTERNAL if sid in SENSOR_MAPPING]
            if failed_sensors_names:
                print(f"Sensores com Falha (não respondem): {', '.join(failed_sensors_names)}")
            else:
                # Caso o usuário passe -s 1 que é ignorado, ou números inválidos.
                print("Todos os sensores funcionando normalmente (nenhum sensor de falha válido selecionado).")
        else:
            print("Todos os sensores funcionando normalmente.")
        print(f"Recuperação 'rec' habilitada: {'Sim' if REC_ENABLED else 'Não'}")
        print("----------------------------------\n")

        while True:
            current_time = time.time()

            # --- Lógica de Polling com Debounce EFETIVO para TODOS os Pinos ---
            for pin_gpio in INPUT_PINS_MAP.keys(): # Itera sobre TODOS os pinos de entrada
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

                    # --- Lógica de Processamento do Comando ESTÁVEL ---
                    # --- LÓGICA ESPECÍFICA PARA O TOOL CHANGER (ID 0) ---
                    if pin_gpio == PIN_TOOL_CHANGER_IN:
                        state['current_output_bit'] = '1' if stable_signal == GPIO.HIGH else '0' # Espelha diretamente
                        # Cancela qualquer timer para garantir que ele não se comporte como atuador
                        if state['response_timer'] and state['response_timer'].is_alive():
                            state['response_timer'].cancel()
                            state['response_timer'] = None # Limpa a referência
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Tool Changer input ({'HIGH' if stable_signal == GPIO.HIGH else 'LOW'}) directly mirrored. Simulated output: {state['current_output_bit']}.")
                        # Força envio imediato para TC (já que não tem delay)
                        # last_update_time = time.time() # Já está global e será atualizado no bloco LOW se mudar para 0
                    # --- LÓGICA PARA OS OUTROS SENSORES (VÁCUOS E CILINDRO) ---
                    else:
                        if stable_signal == GPIO.HIGH: # Borda de Subida ESTÁVEL (LOW para HIGH)
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando HIGH detectado.")

                            # Cancela APENAS o timer de resposta (se estiver ativo) pois um novo HIGH foi detectado.
                            if state['response_timer'] and state['response_timer'].is_alive():
                                state['response_timer'].cancel()
                                state['response_timer'] = None # Limpa a referência
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} - Timer de resposta anterior cancelado.")

                            # --- Lógica de Falha Única por Execução ---
                            # Se REC_ENABLED está ativado, o sensor está na lista de falha, E AINDA NÃO FALHOU NESTA EXECUÇÃO:
                            if REC_ENABLED and (state['id'] in FAILED_SENSOR_IDS_INTERNAL) and not state['permanently_failed_this_run']:
                                state['current_output_bit'] = '0' # A saída permanece LOW (falha)
                                state['has_failed_once_in_cycle'] = True # Marca que falhou neste ciclo HIGH/LOW
                                state['is_currently_failing'] = True # Mantém o estado de falha até o reset/recuperação
                                state['permanently_failed_this_run'] = True # MARCA QUE FALHOU PELA PRIMEIRA E ÚNICA VEZ NESTA EXECUÇÃO

                                # Inicia o timer de recuperação APENAS SE AINDA NÃO ESTIVER RODANDO
                                if not (state['recovery_timer'] and state['recovery_timer'].is_alive()):
                                    state['recovery_timer'] = threading.Timer(SENSOR_RECOVERY_TIME_SECONDS, recover_sensor_from_failure, args=[pin_gpio])
                                    state['recovery_timer'].start()
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) falhou na 1a tentativa. Início timer REC.")
                                else:
                                    print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) falhou na 1a tentativa, timer REC já ativo.")
                            # --- Lógica de Resposta Normal (quando o sensor não está falhando ativamente ou já falhou uma vez e recuperou) ---
                            elif not state['is_currently_failing']:
                                state['is_pending_high_response'] = True # MARCA que está aguardando para responder HIGH
                                state['response_timer'] = threading.Timer(SIMULATION_DELAY_SECONDS, set_sensor_output_high, args=[pin_gpio])
                                state['response_timer'].start()
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) - Resposta HIGH agendada para {SIMULATION_DELAY_SECONDS}s.")
                            else:
                                # Este else só será atingido se:
                                # 1. REC_ENABLED é False E o sensor está em FAILED_SENSOR_IDS_INTERNAL (falha permanente sem recuperação).
                                # 2. REC_ENABLED é True, o sensor está em FAILED_SENSOR_IDS_INTERNAL, já falhou permanentemente,
                                #    E AINDA ESTÁ is_currently_failing (o timer de 1s ou LOW ainda não o recuperou para este ciclo de reenvio).
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Polling-Stable): Sensor {state['name']} (ID {state['id']}) - Ainda em estado de falha ou já falhou uma vez, não responderá HIGH.")


                        elif stable_signal == GPIO.LOW: # Borda de Descida ESTÁVEL (HIGH para LOW)
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG COMMAND (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando LOW detectado.")
                            state['current_output_bit'] = '0' # Seta a saída para LOW imediatamente
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI: Sensor {state['name']} (ID {state['id']}) Bit simulado atualizado para '0'.")
                            #last_update_time = time.time() # Este ajuste só para Tool Changer, aqui a mudança já é imediata
                            state['is_pending_high_response'] = False # Cancela explicitamente qualquer HIGH pendente

                            # Lógica para limpar o estado de falha quando o comando LOW chega
                            # Só recupera por LOW se REC_ENABLED e o sensor *pode* falhar e *já falhou* neste ciclo (has_failed_once_in_cycle)
                            if (state['id'] in FAILED_SENSOR_IDS_INTERNAL) and REC_ENABLED and state['has_failed_once_in_cycle']:
                                state['is_currently_failing'] = False # Reseta o estado de falha para que possa responder na próxima HIGH
                                state['has_failed_once_in_cycle'] = False # Zera a flag para o próximo ciclo de comando HIGH/LOW
                                if state['recovery_timer'] and state['recovery_timer'].is_alive():
                                    state['recovery_timer'].cancel()
                                    state['recovery_timer'] = None
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG PI (Recovery by LOW): Sensor {state['name']} (ID {state['id']}): Recuperado por comando LOW. Pronto para próxima tentativa.")
                            else:
                                # Para sensores que não falham, ou REC_ENABLED não está ativo, ou não falhou neste ciclo.
                                if (state['id'] in FAILED_SENSOR_IDS_INTERNAL) and not REC_ENABLED: # Sensor configurado para falhar sempre, sem REC
                                    state['is_currently_failing'] = True # Mantém a falha permanente
                                else: # Sensor normal ou sensor que já passou pela falha única
                                    state['is_currently_failing'] = False # Garante que ele está normal
                                state['has_failed_once_in_cycle'] = False # Garante reset da flag do ciclo
                                if state['recovery_timer'] and state['recovery_timer'].is_alive():
                                    state['recovery_timer'].cancel()
                                    state['recovery_timer'] = None
                                print(f"[{datetime.now().strftime('%H:%M:%S')}] DEBUG LOW CMD NO REC (Polling-Stable): Sensor {state['name']} (ID {state['id']}): Comando LOW. Flags de falha resetadas normalmente.")


            # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
            # Sempre envia no mínimo a cada DEFAULT_UPDATE_INTERVAL_SECONDS
            # Mas também é forçado a enviar se last_update_time for atualizado explicitamente
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

# This block should be outside the main function, at the top level of the script.
if __name__ == "__main__":
    main()