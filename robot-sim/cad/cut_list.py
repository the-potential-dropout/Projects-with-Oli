"""Harness cut list for Hexapod Mk1 from the CAD routes (build_report.json).

    python cut_list.py            # prints the Markdown table used in docs/WIRING.md

Cut length = routed length x 1.15 (bends, service loops) + 20 mm (terminations),
rounded up to 10 mm.  Leg cables marked x6 are cut six times.
"""
import json
import math
import os

B = chr(92)
HERE = os.path.dirname(os.path.abspath(__file__))
L = lambda p: (B + B + '?' + B + os.path.abspath(p)) if os.name == 'nt' else p

# name prefix: (from, to, conductors / gauge, colours, end A, end B, current A cont / peak)
SPEC = [
    ('Battery lead', 'battery', 'XT60 pair', '2 x 12 AWG (pack lead)', 'red / black', 'XT60 (pack)', 'XT60 male', '6 / 25'),
    ('XT60 to fuse', 'XT60 pair', 'fuse holder', '14 AWG silicone', 'red', 'XT60 female', 'ATC holder', '6 / 25'),
    ('Fuse to ACS724', 'fuse holder', 'ACS724 IP+', '14 AWG silicone', 'red', 'ATC holder', 'screw terminal', '6 / 25'),
    ('ACS724 to switch', 'ACS724 IP-', 'MOSFET switch VIN', '14 AWG silicone', 'red', 'screw terminal', 'solder', '6 / 25'),
    ('Charge lead', 'rear panel XT60', 'battery node', '14 AWG silicone pair', 'red / black', 'XT60 female (panel)', 'solder at XT60 pair', '2 (charging)'),
    ('Balance extension', 'pack balance lead', 'rear panel', '4 x 24 AWG', 'pack colours', 'JST-XH 4 female', 'JST-XH 4 panel', '< 0.5'),
    ('Switch control', 'rocker', 'MOSFET switch CTRL', '2 x 26 AWG', 'grey', 'solder (rocker)', 'solder', '< 0.01'),
    ('Bus trunk', 'switch output / bridge bus port', 'side hub', '2 x 18 AWG + 1 x 26 AWG', 'red / black / yellow', 'solder (switch) + 5264 (data)', 'solder (hub bus bars) + 5264', '3 / 12.5'),
    ('Bus coxa', 'side hub (3 A PTC)', 'coxa servo', '3 x 22 AWG', 'red / black / yellow', 'Molex 5264-3', 'Molex 5264-3', '1 / 4'),
    ('Bus coxa servo-hub', 'coxa servo 2nd socket', 'coxa hub', '3 x 22 AWG', 'red / black / yellow', 'Molex 5264-3', 'Molex 5264-3', '0.8 / 3'),
    ('Bus hub-femur', 'coxa hub', 'femur servo', '3 x 22 AWG', 'red / black / yellow', 'Molex 5264-3', 'Molex 5264-3', '0.5 / 2'),
    ('Bus hub-tibia', 'coxa hub', 'tibia servo', '3 x 22 AWG', 'red / black / yellow', 'Molex 5264-3', 'Molex 5264-3', '0.3 / 1.5'),
    ('J1 bus+power', 'right hub + 5 V buck', 'J1 (receiver)', '2 x 20 AWG + 4 x 26 AWG', 'red / black / orange / yellow', 'solder / 5264', 'JST-XH 8 female', '1.5 / 5'),
    ('J1 aux', 'Feather A2/A3', 'J1 AUX1/AUX2', '2 x 28 AWG', 'white / green', 'solder', 'JST-XH crimp (J1)', '< 0.01'),
    ('VBAT to driver', 'switch output', 'bridge DC jack', '2 x 22 AWG', 'red / black', 'solder', '5.5 x 2.1 plug', '0.2'),
    ('VBAT to buck', 'switch output', 'D36V28F5 VIN', '2 x 22 AWG', 'red / black', 'solder', 'solder', '0.3 / 1'),
    ('VBAT sense', 'switch output', 'bridge GPIO34 (100k/22k)', '26 AWG', 'red/white', 'solder', 'solder + heat-shrink', '< 0.001'),
    ('ACS724 signal', 'ACS724 VOUT', 'bridge GPIO35 (10k/20k)', '26 AWG', 'blue', 'solder', 'solder', '< 0.001'),
    ('Feather 5V', 'D36V28F5 VOUT', 'Feather USB pin (via Schottky)', '2 x 26 AWG', 'orange / black', 'solder', 'solder', '0.3'),
    ('Bridge UART', 'bridge GPIO21/22 + GND', 'Feather RX/TX + GND', '3 x 28 AWG', 'white / green / black', 'JST-SH 4 (header)', 'solder', '< 0.01'),
    ('IMU lead', 'BNO085 STEMMA QT', 'Feather STEMMA QT', '4 x 28 AWG (STEMMA QT cable, 200 mm)', 'red / black / blue / yellow', 'JST-SH 4', 'JST-SH 4', '0.02'),
    ('ToF sensor', 'VL53L1X', 'Feather I2C (daisy)', '4 x 28 AWG', 'red / black / blue / yellow', 'solder', 'JST-SH 4', '0.02'),
    ('Sonar', 'HC-SR04', 'Feather A0/A1 + 5 V', '4 x 26 AWG', 'orange / black / white / green', 'Dupont 4', 'solder', '0.02'),
    ('Receiver CRSF', 'RP1 V2', 'Feather D11/D12 + 5 V', '4 x 28 AWG (RP1 lead)', 'red / black / white / green', 'RP1 pads', 'solder', '0.06'),
    ('Receiver antenna', 'RP1 UFL', 'lid post', 'UFL T-antenna coax', 'stock', 'UFL', '-', '-'),
    ('Foot lead plate', 'pod side wall', 'Feather foot pin + GND', '2 x 28 AWG twisted', 'white / black', 'solder', '(continues as Foot lead A)', '< 0.001'),
    ('Foot lead', 'D2F switch (NO, COM)', 'over the plate to the pod', '2 x 28 AWG twisted silicone', 'white / black', 'solder + heat-shrink', 'one piece with "plate"', '< 0.001'),
    ('Brain power', 'J1 plug VBAT/GND', 'D36V50F5 VIN (470 uF)', '2 x 20 AWG', 'red / black', 'JST-XH 8 male', 'solder', '1.6'),
    ('Brain UART', 'J1 plug AUX1/AUX2', 'Pi GPIO14/15 (pins 8/10)', '2 x 28 AWG', 'white / green', 'JST-XH 8 male', 'Dupont 2', '< 0.01'),
    ('Brain 5V Pi', 'D36V50F5 VOUT', 'Pi GPIO 5 V (pins 2, 4) / GND (6, 9)', '2 + 2 x 20 AWG', 'red / black', 'solder', 'Dupont 2 + 2', '2.3 / 3.5'),
    ('OAK USB', 'Pi USB 3', 'OAK-D Lite USB-C', 'USB-C cable, right-angle, 150 mm', '-', 'USB-A right-angle', 'USB-C', '0.9'),
    ('LD19 lead', 'LD19 ZH1.5-4', 'Pi GPIO5 (UART2 RX) + 5 V', '4 x 28 AWG (stock lead)', 'stock', 'ZH 1.5-4', 'Dupont', '0.18'),
]


def cut(mm):
    return int(math.ceil((mm * 1.15 + 20) / 10.0) * 10)


def rows():
    r = json.load(open(L(os.path.join(HERE, 'build_report.json')), encoding='utf-8'))
    out = []
    for h in r['harness']:
        n = h['cable']
        spec = next((s for s in SPEC if n.startswith(s[0])), None)
        qty = 6 if '(x6)' in n else 1
        out.append((n.replace(' (x6)', ''), qty, h['length_mm'], cut(h['length_mm']), spec))
    return out


def markdown():
    lines = ['| Cable | Qty | From → to | Wire | Colours | End A | End B | A (cont / peak) | Routed (mm) | **Cut (mm)** |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    total = {}
    for n, q, L_, c, s in rows():
        if s:
            lines.append(f'| {n} | {q} | {s[1]} → {s[2]} | {s[3]} | {s[4]} | {s[5]} | {s[6]} | {s[7]} | {L_} | **{c}** |')
            total[s[3]] = total.get(s[3], 0) + q * c
        else:
            lines.append(f'| {n} | {q} | | | | | | | {L_} | **{c}** |')
    lines.append('')
    lines.append('Wire to buy (sum of cut lengths, before 20 % spare): ' + '; '.join(f'{k}: {v / 1000:.2f} m' for k, v in sorted(total.items())))
    return '\n'.join(lines)


if __name__ == '__main__':
    print(markdown())
