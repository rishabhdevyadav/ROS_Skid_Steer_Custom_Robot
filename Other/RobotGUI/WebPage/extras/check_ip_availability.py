import os
hostnames = ["192.168.1.109"]  
# hostnames = ["192.168.1.10" + str(i) for i in range(1,10)]
lut = {0:'Active', 256:'Inactive'}  
botStatus = [os.system("ping -c 1 " + i) for i in hostnames]
print botStatus
botStatus = [lut[i] for i in botStatus]
print botStatus