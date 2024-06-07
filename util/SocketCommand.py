import socket

def send_command(command):
    host = 'localhost'  # The IP where Unity listens
    port = 12345        # The port on which Unity listens
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(command.encode())

# Example usage to move camera
# send_command("MoveCamera,69.8000031,152.5,-46.2000008")  # Move camera to (10, 20, 30)
send_command("Screenshot,/home/david/Documents/Research/ASSIST-U/output/testUnity.png,Normal")
send_command("Screenshot,/home/david/Documents/Research/ASSIST-U/output/testUnityDepth.png,Depth")