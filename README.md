# Sistema de Coleta de dados veiculares (MobLab)

Projeto baseado na API do ESPRESSIF IDF versao 6.0.

==> Resumo do projeto <==

O codigo foi projetado para rodar no ESP32 S3 ligado ao módulo BNO055(9DOF) via I2C e ao módulo ELM327 (OBD-II) via USB.

## LED RGB

<!-- descrever comportamento do led e suas indicações -->

## WIFI

<!-- descrever como configurar o access point padrão ou customizar as configs de wifi -->

## I2C (Acelerometro, giroscopio e magnetometro)

![BNO055](./.assets/BNO055.webp)

- Porta: I2C_NUM_0

- Frequencia: 400kHz

- Pinagem:
   - SDA: GPIO 1
   - SCL: GPIO 2

## USB (ELM327)

![ELM327](./.assets/ELM327.webp)

Configurado para a classe CDC com Virtual COM Port (VCP) e drivers especiais para CH34X (interface com ELM327 acontece atraves de um CH340).

- Parametros de linha
    - 38400 bauds/s
    - 8N1

- Pinagem:
    - D+: GPIO 20 (Fio Verde)
    - D-: GPIO 19 (Fio Branco)

## Codigo

- Dependencias externas
