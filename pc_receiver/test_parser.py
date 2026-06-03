from telemetria_raquete import TelemetryParser

parser = TelemetryParser(expected_id="RAQ01")

linhas = [
    "IMU | t=1250 ms | yaw=3 roll=12 pitch=-4 | acc=[30,-21,985] mg | PLANO",
    "$RAQ,RAQ01,1250,3,12,-4,30,-21,985",
    "$HIT,RAQ01,3250,4,1890",
    "$HIT,RAQ01,5000,8,2100,0,0,0,0,1,0,0,0,1",
    "$RAQ,OUTRA,1250,99,99,99,99,99,99",
]

for linha in linhas:
    print(linha, "=>", parser.parse_line(linha))
