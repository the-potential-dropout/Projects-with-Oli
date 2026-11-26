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
    # rev B2 control / autonomy parts
    ('d36v50fx-step-down-voltage-regulator.step', 'https://www.pololu.com/file/0J1733/d36v50fx-step-down-voltage-regulator.step', None),
    ('vl53l1x-carrier.step', 'https://www.pololu.com/file/0J1195/vl53l0x-vl53l1x-vl53l3cx-vl53l4cd-carrier-model.step', None),
    # Adafruit_CAD_Parts (official Adafruit GitHub, MIT licence)
    ('4754 BNO085 STEMMA QT.step', 'https://raw.githubusercontent.com/adafruit/Adafruit_CAD_Parts/main/4754%20BNO085%20STEMMA%20QT/4754%20BNO085%20STEMMA%20QT.step', None),
    ('5323 Feather ESP32-S3.step', 'https://raw.githubusercontent.com/adafruit/Adafruit_CAD_Parts/main/5323%20Feather%20ESP32-S3/5323%20Feather%20ESP32-S3.step', None),
    # Luxonis (linked from github.com/luxonis/depthai-hardware, MIT licence)
    ('DM9095_enclosure.stp', 'https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-Lite/DM9095_enclosure.stp', None),
    # Raspberry Pi product information portal (public, no login)
    ('rpi5.step', 'https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-010083-CA-1-rpi-5%203D%20STEP%20-%20No%20Graphics%20small%20file.zip', '*.step'),
]


def fetch(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (hexapod-mk1 CAD fetch)'})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def main():
    failed = []
    for name, url, member in FILES:
        dest = os.path.join(HERE, name)
        if os.name == 'nt' and not dest.startswith(LONG):
            dest = LONG + dest            # this repo lives in a very long path
        if os.path.exists(dest):
            print('have', name)
            continue
        try:
            data = fetch(url)
        except Exception as e:           # keep going: the generator falls back to envelopes
            print('FAILED', name, e)
            failed.append(name)
            continue
        if member:
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                if member == '*.step':           # first STEP file in the archive
                    member = next(n for n in z.namelist() if n.lower().endswith(('.step', '.stp')))
                data = z.read(member)
        if name.endswith(('.step', '.stp')) and not data.startswith(b'ISO-10303-21'):
            print('%s does not look like a STEP file - not saved' % name)
            failed.append(name)
            continue
        with open(dest, 'wb') as f:
            f.write(data)
        print('saved', name, len(data), 'bytes')
    # the official Pi 5 STEP stalls Fusion's importer: make the slim copy the generator uses
    full, slim = os.path.join(HERE, 'rpi5.step'), os.path.join(HERE, 'rpi5_slim.step')
    if os.name == 'nt':
        full, slim = LONG + full, LONG + slim
    if os.path.exists(full) and not os.path.exists(slim):
        import subprocess
        tmp = os.path.join(os.environ.get('TEMP', '.'), 'rpi5_in.step')
        out = os.path.join(os.environ.get('TEMP', '.'), 'rpi5_slim.step')
        open(tmp, 'wb').write(open(full, 'rb').read())
        r = subprocess.run([sys.executable, os.path.join(HERE, 'slim_pi5.py'), tmp, out, '2.5', '12'])
        if r.returncode == 0:
            open(slim, 'wb').write(open(out, 'rb').read())
            print('saved rpi5_slim.step')
        else:
            print('slim_pi5.py failed (pip install cadquery-ocp); the generator falls back to the Pi envelope')
    if failed:
        sys.exit('failed: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
