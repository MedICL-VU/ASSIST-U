import socket

def send_command(command):
    host = 'localhost'  # The IP where Unity listens
    port = 12345        # The port on which Unity listens
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(command.encode())

# Example usage to move camera
# send_command("MoveCamera,69.8000031,152.5,-46.2000008")  # Move camera to (10, 20, 30)
# send_command("Screenshot,/home/david/Documents/Research/ASSIST-U/output/testUnity.png,Normal")
# send_command("Screenshot,/home/david/Documents/Research/ASSIST-U/output/testUnityDepth.png,Depth")

    # if rotation is not None:
    #     send_command(f'RotateCamera,{rotation[0]},{rotation[1]},{rotation[2]}')
    # else:
    #     send_command(f'RotateCameraLook,{-focal[0]},{focal[1]},{focal[2]}')
if __name__=='__main__':
    send_command(f'MoveCamera,{-11.30071532},{39.94956768},{30.84415942}')
    send_command(f"RotateCameraLook,{-11.13278016},{ 40.34419754},{29.94079623}")  # Move camera to (10, 20, 30)



# debugging params
# position [-11.30071532  39.94956768  30.84415942]
# focal [-11.13278016  40.34419754  29.94079623]
# roll pitch yaw [-246.94776979093007, 154.6037267846497, -59.23476880065692]
# name '002526_soft 3 med wet_manual_crop.png'