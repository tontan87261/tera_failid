import serial.tools.list_ports as port_list
import serial
#ublox_dev_name = "u-blox GNSS receiver"
#arduino = 'Arduino'

def search_port(device):
    ports = list(port_list.comports())
    for p in ports:
        print(p.manufacturer)
        print(p.description)
        print("--------------")
        
        # Find U-Blox
        if device in p.description:
            print(f"GPS is in port: {p.device}")
            return p.device

        # Find Arduino
        elif device in p.manufacturer:
            print(f"Arduino is in port {p.device}")
            return p.device

