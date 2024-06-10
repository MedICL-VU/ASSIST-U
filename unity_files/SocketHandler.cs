using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.IO;

public class CameraController : MonoBehaviour
{
    public Camera controlledCamera;
    public Camera depthCamera;
    private TcpListener listener;
    private const int port = 12345;

    private void Start()
    {
        listener = new TcpListener(IPAddress.Parse("127.0.0.1"), port);
        listener.Start();
        StartCoroutine(HandleIncomingConnections());
    }
    
    public void TakeScreenshot(string filepath, string cameraType="Normal")
    {
    	Camera camera = controlledCamera;
    	if (cameraType == "Depth") {
    	     camera = depthCamera;
    	}

        
        //string path = Path.Combine(Application.persistentDataPath, filename);
        Debug.Log("Writing to " + filepath);
        StartCoroutine(CaptureScreenshot(filepath, camera));
    }

    private IEnumerator CaptureScreenshot(string path, Camera camera)
    {
        // Wait for the end of the frame to ensure all rendering is done
        yield return new WaitForEndOfFrame();

        RenderTexture renderTexture = new RenderTexture(1080, 1080, 24);
        camera.targetTexture = renderTexture;  // Set the camera's target texture
        camera.Render();

        RenderTexture.active = renderTexture;
        Texture2D screenshot = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        screenshot.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        screenshot.Apply();

        byte[] bytes = screenshot.EncodeToPNG();
        File.WriteAllBytes(path, bytes);

        camera.targetTexture = null;
        RenderTexture.active = null;  // Reset the active render texture
        Destroy(renderTexture);  // Clean up
    }

    private IEnumerator HandleIncomingConnections()
    {
        while (true)
        {
            if (listener.Pending())
            {
                TcpClient client = listener.AcceptTcpClient();
                NetworkStream stream = client.GetStream();
                byte[] buffer = new byte[client.ReceiveBufferSize];
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                string command = Encoding.ASCII.GetString(buffer, 0, bytesRead);
                ExecuteCommand(command);
                client.Close();
            }
            yield return null;
        }
    }

    private void ExecuteCommand(string command)
    {
    	Debug.Log(command);
        string[] parts = command.Split(',');
        if ((parts.Length == 2 || parts.Length == 3) && parts[0] == "Screenshot") 
        {
            if (parts.Length ==2) {
            	TakeScreenshot(parts[1]);
            }
            else {
            	TakeScreenshot(parts[1], parts[2]);
            }
            
        }
        else if (parts.Length == 4 && parts[0] == "MoveCamera")
        {
            float x = float.Parse(parts[1]);
            float y = float.Parse(parts[2]);
            float z = float.Parse(parts[3]);
            controlledCamera.transform.position = new Vector3(x, y, z);
        }
        else if (parts[0] == "RotateCamera" && parts.Length == 4)
        {
            // Rotation change command
            float x = float.Parse(parts[1]);
            float y = float.Parse(parts[2]);
            float z = float.Parse(parts[3]);
            controlledCamera.transform.eulerAngles = new Vector3(x, y, z);
        }
        else if (parts[0] == "RotateCameraLook" && parts.Length == 4)
        {
            // Rotation change command
            float x = float.Parse(parts[1]);
            float y = float.Parse(parts[2]);
            float z = float.Parse(parts[3]);
            Vector3 focalPoint = new Vector3(x, y, z);
            Vector3 forwardDirection = focalPoint - transform.position;
            Quaternion rotation = Quaternion.LookRotation(forwardDirection, Vector3.up);
            
            transform.rotation = rotation;
        }
        // Add more commands as needed
    }

    private void OnApplicationQuit()
    {
        listener.Stop();
    }
}

