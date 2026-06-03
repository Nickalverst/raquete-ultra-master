# Telemetria da raquete no PC

Este código roda no computador. Ele recebe dados pela porta serial criada pelo rádio/USB/Bluetooth e mostra:

- yaw em tempo real;
- roll / ângulo X;
- pitch / ângulo Y;
- aceleração X, Y e Z;
- heatmap 3x3 dos impactos dos piezos;
- visualização 3D parecida com uma raquete de ping-pong.

## Protocolos adotados

### IMU

```text
$RAQ,RAQ01,t_ms,yaw_deg,roll_deg,pitch_deg,acc_x_mg,acc_y_mg,acc_z_mg
```

Exemplo:

```text
$RAQ,RAQ01,1250,3,12,-4,30,-21,985
```

### Impacto / heatmap

Formato curto:

```text
$HIT,RAQ01,t_ms,regiao,valor_pico
```

Formato recomendado, com os contadores do heatmap:

```text
$HIT,RAQ01,t_ms,regiao,valor_pico,h0,h1,h2,h3,h4,h5,h6,h7,h8
```

Exemplo:

```text
$HIT,RAQ01,3250,4,1890,0,0,0,0,1,0,0,0,0
```

Regiões do heatmap:

```text
0 1 2
3 4 5
6 7 8
```

O Python ignora qualquer outra linha do terminal, incluindo os prints humanos.

## Instalar dependências

```bash
pip install -r requirements.txt
```

## Testar sem a raquete

```bash
python telemetria_raquete.py --mock
```

ou:

```bash
python telemetria_raquete.py --simulate
```

Isso gera dados falsos `$RAQ` e `$HIT` e abre o painel em tempo real.

## Ver linhas falsas no terminal

```bash
python mock_linhas_raquete.py
```

## Testar o parser

```bash
python test_parser.py
```

## Rodar com a raquete real

Descubra a porta:

```bash
python -m serial.tools.list_ports
```

Depois rode, por exemplo:

```bash
python telemetria_raquete.py --port COM5 --baud 115200
```

No Linux/macOS, a porta costuma ser algo como `/dev/ttyUSB0`, `/dev/ttyACM0` ou `/dev/cu.usbserial-XXXX`.

## Salvar CSV

```bash
python telemetria_raquete.py --port COM5 --baud 115200 --csv dados_raquete.csv
```
