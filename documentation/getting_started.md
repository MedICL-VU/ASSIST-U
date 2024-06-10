# Getting Started

--- 

## Libraries

---

ASSIST-U was built on Python 3.8 with conda and should work for any later version. 
A requirements.txt has been created and can be installed via 

```conda create --name <env> --file requirements.txt```

Alternatively, you can customize your install with the following core libraries:

```
vtk 
open3d
opencv-python
skeletor
fastremap
numpy
```



## Unity

---

ASSIST-U supports rendering in Unity with its Universal Rendering Pipeline. To set up Unity, see this [Unity Setup](./unity_setup.md).



## Running ASSIST-U

---

ASSIST-U will run with default arguments with a direct call of `python main.py`. 

Arguments can be found in _util/argparser.py_.

