"""Download the manufacturers' STEP models used by the Hexapod Mk1 generator.

Only official manufacturer / distributor URLs, no logins, no executables.  The
files are not committed to the repo (their licences don't grant
redistribution); this script puts them in this folder, where
hexapod_mk1_fusion.py picks them up.  See SOURCES.md.

    python fetch_vendor_cad.py
"""
import io
import os
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
LONG = '\\\\?\\'   # Windows extended-length path prefix

FILES = [
    # (saved name, url, member inside the zip or None)
    ('ST3215.step', 'https://files.waveshare.com/upload/5/59/ST3215-3D.zip', 'ST3215.step'),
    ('ST3215.pdf', 'https://files.waveshare.com/upload/0/08/ST3215-2D.zip', 'ST3215.pdf'),
    ('Servo Driver with ESP32.step', 'https://files.waveshare.com/upload/7/73/Servo_Driver_with_ESP32_3D.zip', 'Servo Driver with ESP32.step'),
    ('d36v28fx-step-down-voltage-regulators.step', 'https://www.pololu.com/file/0J1660/d36v28fx-step-down-voltage-regulators.step', None),
    ('big-mosfet-slide-switch-with-reverse-voltage-protection.step', 'https://www.pololu.com/file/0J1406/big-mosfet-slide-switch-with-reverse-voltage-protection.step', None),
    ('acs724-current-sensor-carrier.step', 'https://www.pololu.com/file/0J1985/acs724-current-sensor-carrier.step', None),
]


def fetch(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (hexapod-mk1 CAD fetch)'})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def main():
    for name, url, member in FILES:
        dest = os.path.join(HERE, name)
        if os.name == 'nt' and not dest.startswith(LONG):
            dest = LONG + dest            # this repo lives in a very long path
        if os.path.exists(dest):
            print('have', name)
            continue
        data = fetch(url)
        if member:
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                data = z.read(member)
        if name.endswith('.step') and not data.startswith(b'ISO-10303-21'):
            sys.exit('%s does not look like a STEP file - not saved' % name)
        with open(dest, 'wb') as f:
            f.write(data)
        print('saved', name, len(data), 'bytes')


if __name__ == '__main__':
    main()
