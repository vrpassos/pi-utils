import serial
import RPi.GPIO as GPIO
import time
from datetime import datetime

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

# --- Configurações de Simulação ---
SIMULATION_DELAY_SECONDS = 2 # Delay para simular a transição de 0 para 1 (ligar/avançar)
UPDATE_INTERVAL_SECONDS = 1 # Intervalo de atualização da UART para 1 segundo

# Variáveis de estado para simulação de delay
simulation_states = {
    PIN_VAC_INFERIOR_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None},
    PIN_CILINDRO_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None},
    PIN_VAC_SUPERIOR_IN: {'simulated_output_state': GPIO.LOW, 'start_time': None}
}

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) 
    print(f"UART configurada e aberta na porta {SERIAL_PORT} com baud rate {BAUD_RATE}")

    last_update_time = time.time() 

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
        
        # Iterar sobre os pinos que possuem simulação de delay
        simulated_pins_info = [
            (PIN_VAC_INFERIOR_IN, vi_command_signal),
            (PIN_CILINDRO_IN, cil_command_signal),
            (PIN_VAC_SUPERIOR_IN, vs_command_signal)
        ]

        for pin_id, command_signal_from_toradex in simulated_pins_info:
            state_info = simulation_states[pin_id]

            if command_signal_from_toradex == GPIO.HIGH: # Comando para LIGAR/AVANÇAR
                if state_info['simulated_output_state'] == GPIO.LOW: # Se o status reportado ainda não é HIGH
                    if state_info['start_time'] is None: # Se o timer não foi iniciado
                        state_info['start_time'] = current_time # Inicia o timer
                    
                    # Se o delay já passou, mude o estado reportado para HIGH
                    if state_info['start_time'] is not None and (current_time - state_info['start_time']) >= SIMULATION_DELAY_SECONDS:
                        state_info['simulated_output_state'] = GPIO.HIGH
                        state_info['start_time'] = None # Reseta o timer após atingir o estado final
                # else (simulated_output_state já é HIGH): Não faz nada, já está no estado final
                # Não há 'else' aqui, pois se já está HIGH e o comando é HIGH, a simulação já concluiu.
                # O start_time não deve ser resetado aqui, pois ele já foi resetado na conclusão do delay.
                # Se o comando muda de LOW para HIGH e depois para HIGH novamente,
                # a condição simulated_output_state == GPIO.LOW impedirá que o start_time seja indevidamente resetado.
            
            elif command_signal_from_toradex == GPIO.LOW: # Comando para DESLIGAR/RETORNAR
                state_info['simulated_output_state'] = GPIO.LOW # Muda para LOW IMEDIATAMENTE
                state_info['start_time'] = None # Reseta o timer (cancela qualquer delay de ativação)

        # --- Envia o Status Atual (Simulado) para a Toradex via UART ---
        if (current_time - last_update_time) >= UPDATE_INTERVAL_SECONDS:
            state_tc = str(tool_changer_report_state)
            state_vi = str(simulation_states[PIN_VAC_INFERIOR_IN]['simulated_output_state'])
            state_cil = str(simulation_states[PIN_CILINDRO_IN]['simulated_output_state'])
            state_vs = str(simulation_states[PIN_VAC_SUPERIOR_IN]['simulated_output_state'])

            output_bit_string = state_tc + state_vi + state_cil + state_vs
            byte_to_send = int(output_bit_string, 2).to_bytes(1, 'big')

            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"{timestamp}: Saídas da Toradex: {output_bit_string[0]} {output_bit_string[1]} {output_bit_string[2]} {output_bit_string[3]}") # String ALTERADA AQUI
            ser.write(byte_to_send)
            
            last_update_time = current_time 
        
        time.sleep(0.01) # Pequena pausa para evitar CPU alta no loop

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