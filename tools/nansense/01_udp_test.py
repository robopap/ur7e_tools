import socket
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 33333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP {UDP_IP}:{UDP_PORT} ...")

count = 0
t0 = time.time()
printed_packet = False

while True:
    data, addr = sock.recvfrom(65535)

    count += 1

    if not printed_packet:
        print("\nFirst packet")
        print("Source:", addr)
        print("Bytes:", len(data))
        print("-" * 60)

        try:
            print(data.decode("utf-8", errors="replace")[:4000])
        except Exception:
            print(data[:500])

        print("-" * 60)
        printed_packet = True

    now = time.time()

    if now - t0 >= 1.0:
        print(f"Rate: {count / (now - t0):.1f} packets/s")
        count = 0
        t0 = now