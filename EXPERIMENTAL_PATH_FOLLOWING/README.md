
# EXPERIMENTAL SOFTWARE FOR THE UGV LOCATED AT THE ITESM CAMPUS PUEBLA
Are you ready for experimenting with the real UGV, you should consider all the safety requirements before anything. Always verify the batteries voltage are over 48 volts, otherwise the vehicle will be stocked in the middle of nowhere. 

- First of all turn on the vehicle. NEVER PLUG USB CABLES TO YOUR LAPTOP BEFORE TURNING ON THE VEHICLE. it might cause damage to your USB.
- Once everything is on, plug in the 3 wires to your laptop (Xbox receiver, IMU-GPS Arduino, UGV Arduino).
- Wait around 15 seconds before running any program, the Arduinos take some time before your serial ports are available.
- Verify that the UGV Arduino is on serial port /dev/ttyACM1, otherwise you will have to change the AGV_INTERFACE_1.py program.
- Run the AGV_INTERFACE_1.py. If it breaks, try several times. Sometimes it does not work on the first try. 

python AGV_INTERFACE_1.py
