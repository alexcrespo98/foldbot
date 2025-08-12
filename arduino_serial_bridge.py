import serial
import socket
import threading

SERIAL_PORT = 'COM9'
BAUD_RATE = 9600
TCP_IP = '127.0.0.1'
TCP_PORT = 5005

def serial_to_socket(ser, conn):
    while True:
        line = ser.readline()
        if line:
            conn.sendall(line)

def socket_to_serial(ser, conn):
    while True:
        data = conn.recv(1024)
        if data:
            ser.write(data)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print(f"Listening for TCP connection on {TCP_IP}:{TCP_PORT}...")
    conn, addr = s.accept()
    print(f"Connection from {addr}")

    t1 = threading.Thread(target=serial_to_socket, args=(ser, conn))
    t2 = threading.Thread(target=socket_to_serial, args=(ser, conn))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()