# Unity Setup

---

ASSIST-U can communicate with Unity to create renders instead of vtk. As long as a Unity scene is running, ASSIST-U will attempt to communicate and send commands over a local socket.

The following setup have been tested with Unity 2022.3.31f1 LTS version. It may work with other versions but has not been tested. Unity6 does not support built-in render pipeline so this setup will not work with Unity 6.

## Mesh details

---

Unity does not like rendering the insides of surface meshes with surface normals pointed outward. We have written two scripts to convert an stl into an obj for moving into Unity and flipping the normals. Simply run stl2obj.py and then FlipNormals.py witht he correct filenames to fix this issue.

All meshes should be loaded with position and rotation of (0,0,0). The camera will automatically be moved by ASSIST-U.

[//]: # (### Quick Setup)

[//]: # ()
[//]: # (---)

[//]: # ()
[//]: # (The fastest way to set up the scene is to import the prefabs in _unity_files_.)

[//]: # ()
[//]: # (You should import the mesh twice, assign one to a layer named `Mesh` and the other named `Depth`.)

[//]: # (The `Tissue Cam` prefab should be imported and added to the scene. Be sure that the top layer Tissue Cam is also assigned to Mesh and has Culling Mask set to Mesh. Similarly, the Depth Cam should have the corresponding layer and culling masks set.)

[//]: # ()
[//]: # (If the Camera Controller Script does not exist at the bottom of Tissue Cam as shown in the image below, see Manual Setup.)

![Unity Scene](./images/unity_scene_setup.png)

### Manual Scene Setup

---
Start a new `3D` project in Unity Hub using the `3D Built-In Render Pipeline`.

Create a new scene or reuse the startup scene in Unity and delete the main camera and light gameobjects on the left.

On the top bar, go to Window-> Rendering->Lighting. Under Environment->Skybox Material, select None to disable the skybox. Then under Environment->Ambient Mode select Color and set the color to black.

Import all unity files in [ASSIST-U/unity_files](../unity_files) into the Project/Assets.

Create two cameras and a spotlight from under the top GameObject tab.
One camera should be the parent of the other two as shown in the picture above. The spotlight should also be a child of the parent camera. After moving, you will likely need to reset the transform (position and rotation) of the child camera and spotlight to (0,0,0). Find this in the Inspector on the right when selecting the object in the Hierarchy on the left.

Drag your surface mesh into a project folder in the Unity assets. If you only have an stl file from segmentation, we have provided a simple stl to obj file converter under the [util folder](../util/stl2obj.py). Click on the imported model in the bottom file explorer and find the defaultMat under Inspector->Materials. Click on the circle to the right of `None (Material)` and select the [NewOrgan](../unity_files/NewOrgan.mat) file you have recently imported.
Change the color of your surface mesh by adjusting the Albedo of `NewOrgan`.

Make sure that after importing JetMaterial under the Assets folder, the shader category points to [JetShader](../unity_files/JetShader.shader). Find this by clicking on JetMaterial and looking under Inspector->Shader. If it does not, click on the dropdown and select Custom/JetShader.
Modify the depth maps by adjusting the Monochrome or JetColor methods of JetShader. They are called in the frag method.

Import your surface mesh with flipped surface normals (see Mesh Details above) twice into the Unity scene. This can be done by dragging the same file into the scene two times. It is easier to manipulate these objects if you make one the child of the other.
Under Layers in the inspector of any object, add 2 layers, named `Mesh` and `Depth`.
Assign the parent camera, the spotlight, and one surface mesh to `Mesh`. Assign the child camera and the other surface mesh to Depth. Their names are not important so rename as convenient.

Add the [CameraOperate](../unity_files/CameraOperate.cs) and [Socket Handler](../unity_files/SocketHandler.cs) to the parent Camera. This can be done by scrolling to the bottom of the inspector and dragging the scripts in, or by pressing Add Component. The former script allows manipulation of the camera view when the scene is running and the latter allows communication with ASSIST-U.
Point controlled camera to the parent camera (tagged Mesh) and the depth camera to the child camera (tagged Depth).

Click on the depth camera and add the [Depth Script](../unity_files/depth_script.cs). Set the material from the Depth Script variables to [JetMaterial](../unity_files/JetMaterial.mat) in the inspector. Under the Depth Camera settings, set Depth to -1.
### Running the Scene
---

Before running the scene, a few settings under the Camera components must be changed. In the inspector for the parent camera, set Clear Flags to Don't Clear, Culling Mask to Mesh, and Projection to Perspective. In the inspector for the depth camera, set Clear Flags to don't Clear, Culling Mask to Depth, and Projection to Perspective.

Remember to adjust the target display of the child camera to Display 2.

Adjust the FOV of both cameras to match the intrinsics of your camera. This can be done by changing the FOV value in the inspector of each camera. Adjust the spotlight intensity and range to properly illuminate the surface mesh. Clipping planes should also be adjusted to ensure the entire mesh is rendered while not distorting the scale of the output depth maps. You want to have the far plane far enough to always capture rendered objects but not too far that depth precision is lost. The near plane should be sent to around 0.01 or less. Remember to adjust both depth and main cameras equally.

The spotlight color should be adjusted as needed and mode should be set to baked. Make sure the range is set to a value of similar size to your farthest clipping plane or larger (e.g. 200).

Hit the play button at the top center of the Unity window to start the scene. A new window should pop up showing the depth map render. ASSIST-U should now be able to communicate with Unity when running. Display 1 will show the main camera view while Display 2 will show the depth map.

Running ASSIST-U will generate commands to move the camera and capture depth maps. Make sure that Unity is running before starting ASSIST-U.

### Add-ons
---

For quality of life, we added the reset-transform and provided a tissuecam and toolmesh prefab. The Tissue Cam has the CameraOperate and SocketHandler scripts already attached. The ToolMesh prefab is a simple mesh with a standard material that can be used to represent surgical tools in the scene. Both prefabs can be found in the [unity_files](../unity_files) folder. ToolMeshes should be placed under the main camera in the hierarchy to move with the camera.

A ResetTransform script is also provided to quickly reset the position and rotation of any object in the scene. This can be added to any object by selecting the object and pressing Add Component in the inspector. Search for ResetTransform and add it. When the scene is running, select the object and press R to reset its transform.

For additional control over the camera, the Unity Asset Store has a several packages that can be imported to allow for mouse and keyboard control of the camera when the scene is running. The CameraOperate script was imported from the H4CameraOperate package. This package was modified to work with our setup and allow for easier camera movement when the scene is running. Similar packages can be added to quickly extend boilerplate functionality.

