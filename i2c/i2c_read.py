import time
import smbus2

# --- Configurações do ADS1115 ---
# Endereço I2C do ADS1115. Verifique o seu hardware.
# 0x48 é o padrão (ADDR/A0 conectado ao GND)
# 0x49 se ADDR/A0 estiver conectado ao VDD
# 0x4A se ADDR/A0 estiver conectado ao SDA
# 0x4B se ADDR/A0 estiver conectado ao SCL
ADS1115_ADDRESS = 0x48

# Barramento I2C na Raspberry Pi (geralmente 1 para modelos recentes)
I2C_BUS = 1

# Registradores do ADS1115
CONVERSION_REGISTER = 0x00  # Onde o resultado da conversão é armazenado
CONFIG_REGISTER = 0x01      # Onde as configurações são escritas

# --- Configurações para a palavra de configuração (CONFIG_REGISTER) ---
# Os valores abaixo são bits que serão combinados para formar a palavra de configuração.
# Consulte o datasheet do ADS1115 para mais detalhes.

# OS (Operational Status) / Iniciar conversão:
# 1 = Inicia uma única conversão (single-shot)
OS_SINGLE_SHOT = 0x8000 # Bit 15

# MUX (Multiplexer Configuration) - Seleção do Canal de Entrada
# Estes bits selecionam qual entrada analógica será lida.
# Exemplo: AIN0 = 0x4000, AIN1 = 0x5000, etc.
# Usaremos uma máscara e deslocamento para definir isso na função.

# PGA (Programmable Gain Amplifier) - Faixa de Leitura (FSR - Full Scale Range)
# Importante! Define a voltagem máxima que pode ser lida.
# É uma troca entre resolução e faixa.
FSR_6_144V = 0x0000  # ±6.144V  (Ganho 2/3) - Maior faixa, menor resolução
FSR_4_096V = 0x0200  # ±4.096V  (Ganho 1)   - Bom equilíbrio
FSR_2_048V = 0x0400  # ±2.048V  (Ganho 2)
FSR_1_024V = 0x0600  # ±1.024V  (Ganho 4)
FSR_0_512V = 0x0800  # ±0.512V  (Ganho 8)
FSR_0_256V = 0x0A00  # ±0.256V  (Ganho 16)

# MODE (Operating Mode)
# 0 = Single-shot (conversão única, desliga após a leitura)
# 1 = Continuous (conversões contínuas)
MODE_SINGLE_SHOT = 0x0100 # Bit 8

# DR (Data Rate) - Taxa de Amostragem (SPS - Samples Per Second)
# Taxa em que as conversões são realizadas.
DR_8_SPS   = 0x0000
DR_16_SPS  = 0x0020
DR_32_SPS  = 0x0040
DR_64_SPS  = 0x0060
DR_128_SPS = 0x0080 # Padrão
DR_250_SPS = 0x00A0
DR_475_SPS = 0x00C0
DR_860_SPS = 0x00E0

# COMP_MODE, COMP_POL, COMP_LAT, COMP_QUE - Configurações do Comparador (geralmente não usadas para leitura simples)
# Desabilitar comparador para leitura simples
COMP_DISABLE = 0x0003 # Bits [3:0] - Desativa o comparador

# --- Inicializar o barramento I2C ---
try:
    bus = smbus2.SMBus(I2C_BUS)
    print(f"DEBUG: Barramento I2C {I2C_BUS} inicializado com sucesso.")
except Exception as e:
    print(f"ERROR: Erro ao inicializar o barramento I2C: {e}")
    exit(1) # Sai com código de erro se o barramento não puder ser inicializado

def read_ads1115(channel, fsr_volts=4.096, sample_rate=DR_860_SPS):
    """
    Lê a voltagem de um canal específico do ADS1115.

    Args:
        channel (int): O canal a ser lido (0 para AIN0, 1 para AIN1, etc.).
                       Pode ser 0, 1, 2, 3 para entradas single-ended.
        fsr_volts (float): A Faixa de Escala Total (Full Scale Range) em Volts.
                           Valores comuns: 0.256, 0.512, 1.024, 2.048, 4.096, 6.144.
        sample_rate (int): Taxa de amostragem (use as constantes DR_xxx_SPS).

    Returns:
        float: A voltagem lida no canal especificado, ou float('nan') em caso de erro.
    """
    # Mapear o canal para os bits MUX
    if channel == 0:
        mux_bits = 0x4000  # AIN0
    elif channel == 1:
        mux_bits = 0x5000  # AIN1
    elif channel == 2:
        mux_bits = 0x6000  # AIN2
    elif channel == 3:
        mux_bits = 0x7000  # AIN3
    else:
        print(f"ERROR: Canal inválido: {channel}. Use 0, 1, 2 ou 3.")
        return float('nan')

    # Mapear FSR para os bits PGA
    pga_bits = 0x0000 # Padrão para 6.144V
    if fsr_volts == 0.256:
        pga_bits = FSR_0_256V
        fsr_value = 0.256 # Valor real do FSR
        max_raw = 8192 # 2^13, pois 16-bit com sinal, mas FSR é menor
    elif fsr_volts == 0.512:
        pga_bits = FSR_0_512V
        fsr_value = 0.512
        max_raw = 16384
    elif fsr_volts == 1.024:
        pga_bits = FSR_1_024V
        fsr_value = 1.024
        max_raw = 32768
    elif fsr_volts == 2.048:
        pga_bits = FSR_2_048V
        fsr_value = 2.048
        max_raw = 32768
    elif fsr_volts == 4.096:
        pga_bits = FSR_4_096V
        fsr_value = 4.096
        max_raw = 32768
    elif fsr_volts == 6.144:
        pga_bits = FSR_6_144V
        fsr_value = 6.144
        max_raw = 32768
    else:
        print(f"ERROR: FSR inválido: {fsr_volts}V. Usando 4.096V.")
        pga_bits = FSR_4_096V
        fsr_value = 4.096
        max_raw = 32768

    # Construir a palavra de configuração
    # OS_SINGLE_SHOT: Inicia uma conversão
    # mux_bits: Seleciona o canal
    # pga_bits: Define a faixa de ganho
    # MODE_SINGLE_SHOT: Modo single-shot (desliga após a leitura)
    # sample_rate: Define a taxa de amostragem
    # COMP_DISABLE: Desabilita o comparador
    config_word = (OS_SINGLE_SHOT | mux_bits | pga_bits |
                   MODE_SINGLE_SHOT | sample_rate | COMP_DISABLE)

    # Converter a palavra de configuração para dois bytes (MSB primeiro)
    config_bytes = [(config_word >> 8) & 0xFF, config_word & 0xFF]

    try:
        # Escrever a palavra de configuração no ADS1115
        bus.write_i2c_block_data(ADS1115_ADDRESS, CONFIG_REGISTER, config_bytes)
    except Exception as e:
        print(f"ERROR: Erro ao escrever configuração para o ADS1115 (canal {channel}): {e}")
        return float('nan')

    # Pequena pausa para permitir que a conversão single-shot ocorra.
    # A duração depende da taxa de amostragem (SPS). Para 860SPS, 10ms é mais que suficiente.
    time.sleep(0.01)

    try:
        # Ler 2 bytes do registrador de conversão
        data = bus.read_i2c_block_data(ADS1115_ADDRESS, CONVERSION_REGISTER, 2)
        raw_adc = (data[0] << 8) | data[1]

        # Converter para número com sinal (complemento de dois)
        if raw_adc & 0x8000:  # Se o bit mais significativo (bit 15) for 1, é negativo
            raw_adc -= 65536 # Equivalente a ~raw_adc + 1 para 16 bits

        # Calcular a voltagem
        # A resolução é FSR / (2^15) porque é 16 bits com sinal (-32768 a +32767)
        voltage = raw_adc * (fsr_value / 32768.0)

    except Exception as e:
        print(f"ERROR: Erro ao ler dados do ADS1115 (canal {channel}): {e}")
        return float('nan')

    return voltage

# --- Loop principal para leitura dos canais ---
try:
    print("\nIniciando leituras do ADS1115...")
    fsr_volts = 2.048
    while True:
        # Exemplo de leitura dos Canais A0, A1 e A2 com FSR de 4.096V
        voltage_ch0 = read_ads1115(0, fsr_volts, sample_rate=DR_860_SPS)
        voltage_ch1 = read_ads1115(1, fsr_volts, sample_rate=DR_860_SPS)
        voltage_ch2 = read_ads1115(2, fsr_volts, sample_rate=DR_860_SPS)

        current_time = time.strftime("%H:%M:%S", time.localtime())
        print(f"{current_time}")
        print(f"  AIN0: {voltage_ch0:.3f} V (FSR: {fsr_volts} V)")
        print(f"  AIN1: {voltage_ch1:.3f} V (FSR: {fsr_volts} V)")
        print(f"  AIN2: {voltage_ch2:.3f} V (FSR: {fsr_volts} V)")
        print("---")
        time.sleep(1) # Aguarda 1 segundo antes da próxima leitura

except KeyboardInterrupt:
    print("\nPrograma encerrado pelo usuário (Ctrl+C)")
except Exception as e:
    print(f"\nOcorreu um erro inesperado: {e}")
finally:
    # Garante que o barramento I2C seja fechado ao sair
    if 'bus' in locals() and bus:
        bus.close()
        print("Barramento I2C fechado.")
