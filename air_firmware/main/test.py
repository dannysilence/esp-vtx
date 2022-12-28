from serial import Serial
import time

serial=Serial(port="com4",baudrate=115200)




serial.setDTR(False)  # IO0=HIGH, done
serial.setRTS(False)   # EN=LOW, chip in reset

bytes=b""
print("OK")

t1=time.time()

time.sleep(1)

serial.setRTS(True)
serial.setDTR(True)

t2=t1
while t2-t1<6:
    #
    while(serial.in_waiting>0):
        bytes+=serial.read(serial.in_waiting)
        print(bytes)
    t2=time.time()


serial.write("a".encode())
bytes=serial.read(4)
length=int.from_bytes(bytes,byteorder='little')
print(length)
exit(0)
bytes=serial.read(length)

print(bytes)

#with open("test.jpg","wb") as f:
    #f.write(bytes)
exit(0)