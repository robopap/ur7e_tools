import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 33333))

print("Waiting for one NANSENSE frame...")

data, addr = sock.recvfrom(65535)

text = data.decode("utf-8", errors="ignore")

print(f"\nSource: {addr}\n")

for line in text.splitlines():
    parts = line.strip().split(",")

    if len(parts) < 2:
        continue

    name = parts[0].replace("mixamorig:", "")
    parent = parts[1].replace("mixamorig:", "")

    if name.lower() == "displacement":
        continue

    print(f"{name:25s} <- {parent}")

sock.close()