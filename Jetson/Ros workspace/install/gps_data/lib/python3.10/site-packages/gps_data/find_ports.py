import serial.tools.list_ports as port_list

find = 'u-blox GNSS receiver'

def find_ports(name):
    
    ports = list(port_list.comports())
    for p in ports:
        print(name)
        #Edge case for arduino, It has no name in the list.
        if name == 'u-blox GNSS receiver':
            if p.product == None:
                auk = p.device
                return auk
            
        elif p.product == name:
            auk = p.device
            return auk
        
          

if __name__ == '__main__':
    try:
        print(find_ports(find))
    except KeyboardInterrupt:
        pass
