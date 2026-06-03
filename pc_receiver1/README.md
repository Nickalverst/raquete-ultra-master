# Telemetria da raquete no PC

Este código roda **no computador**, separado do firmware do STM32.

Ele abre a porta serial criada pelo rádio/Bluetooth/USB serial, recebe os dados da raquete e mostra:

- yaw em tempo real;
- ângulo X / roll;
- ângulo Y / pitch;
- aceleração X, Y e Z;
- visualização 3D aproximada da orientação da raquete.

## 1. Instalar dependências

No terminal, dentro desta pasta:

```bash
pip install -r requirements.txt
```

## 2. Descobrir a porta serial

No Windows:

```bash
python -m serial.tools.list_ports
```

Procure algo como `COM3`, `COM4`, `COM5` etc.

No Linux/macOS, normalmente será algo como:

```bash
/dev/ttyUSB0
/dev/ttyACM0
/dev/cu.usbserial-XXXX
```

## 3. Rodar com a raquete

Exemplo no Windows:

```bash
python telemetria_raquete.py --port COM5 --baud 115200
```

Exemplo no Linux:

```bash
python telemetria_raquete.py --port /dev/ttyUSB0 --baud 115200
```

Para gravar os dados recebidos em CSV:

```bash
python telemetria_raquete.py --port COM5 --baud 115200 --csv dados_raquete.csv
```

## 4. Testar sem a raquete

```bash
python telemetria_raquete.py --simulate
```

## Formato recomendado do firmware

O ideal é o STM32 enviar uma linha por amostra:

```text
TEL,t_ms,yaw_deg,ang_x_deg,ang_y_deg,acc_x_mg,acc_y_mg,acc_z_mg
```

Exemplo:

```text
TEL,1250,3,12,-4,30,-21,985
```

No código atual, `ang_x` é o `roll` e `ang_y` é o `pitch`.

O script também tenta ler o formato antigo, com linhas do tipo:

```text
Accel : X=   30mg  Y=  -21mg  Z=  985mg
Roll  :   12deg   Pitch:   -4deg   Yaw:    3deg
```

Mas o formato `TEL,...` é mais confiável para rádio serial e para processamento em Python.
