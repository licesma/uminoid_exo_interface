# Control Inspire hand from host — quick notes

## On the G1 (over SSH)

```bash
ssh unitree@192.168.123.164
# password: 123
```

If the host key changed and ssh complains:
```bash
ssh-keygen -f "/home/esteban/.ssh/known_hosts" -R "192.168.123.164"
```

Once inside the G1, start the DDS↔serial bridge:

```bash
cd ~/repos/uminoid_exo_interface/cpp/build
sudo ./dfx_inspire_service/inspire_g1
```

It should print:
```
 --- Unitree Robotics ---
  Inspire Hand Controller
```
and then just sit there (that's correct — it's waiting for DDS commands).

Hand is on `/dev/ttyUSB0` (right hand). Left hand is disabled in code.

## On the host

Make sure the USB-ethernet to the G1 is UP:
```bash
ip -br link show | grep enx
```
(should say UP, not DOWN)

Then run the test:
```bash
cd ~/repos/uminoid_exo_interface/cpp/build
./inspire_g1_test enx00e04c6803fc
```

That sweeps the right-hand **index finger** open↔closed via DDS.

## Flow (just so I remember)

```
host: inspire_g1_test  --DDS rt/inspire/cmd-->  G1: inspire_g1  --serial-->  Inspire hand
                       <--DDS rt/inspire/state--                <--serial--
```

Bridge on the G1 must be running or nothing happens.


~/micromamba/envs/vision/bin/python ~/realsense_raw_rgb.py