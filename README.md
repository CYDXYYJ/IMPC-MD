# IMPC-MD
Code for article: Multi-UAV Deployment in Obstacle-cluttered Environments with LOS connectivity


Left: Find a topology that deploys as few asagents to reach some targets. 
Right: A trajectory planning method that make the underlying UAVs fly to the targets while maintain connectivity.

&nbsp;
<div align=center>
<img src="./images/fig-1.jpg" width=300>
<img src="./images/fig-2.gif" width=300>
</div>
&nbsp;


## 1. General Setup

We have tested our code with:
`ubuntu 20.0.4`
in python3

This APP need following dependencies:
```
numpy          1.21.2
scipy          1.6.3
cvxopt         1.3.0
matplotlib     3.4.3
mayavi         4.8.0
```

## 2. Essential file 'opengjkc.so' generation Setup

IMPC-MD depends on the package [openGJK](https://github.com/MattiaMontanari/openGJK#getting-started) to apply polytope-related calculation. As openGJK is written in C++, a conversion from C++ to python is needed. This can be done by generating a Python-Binding file called `opengjkc.so`.

PS: This file must be generated specifically for each different running environment. The one provided in IMPC-OB source code is generated based on python 3.9, thus it might not be available for importing in your environment. Be sure to manually generate it yourself!

The process is shown below:
### 1.1 Copy the source code  
    git clone https://github.com/MattiaMontanari/openGJK  
### 1.2 Build the file  
    cmake -E make_directory build  
    cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Release ..   
    cmake --build build  
    cmake -E chdir build/examples/c ./example_lib_opengjk_ce  
### 1.3 File location adjustment  
Move the 'openGJK.c' file from the root directory to the 'src' directory, then open it and delete all keywords named `restrict`.  
### 1.4 File generation  
cd to the 'examples/python' directory and run ```source build.sh```.  
This should generate a file called `opengjkc.cpython-38-x86_64-linux-gnu.so`.  
Rename it as `opengjkc.so` and move it to the 'IMPC-MD/scripts/geometry/opengjk' directory to replace the existing opengjkc.so file.  


## Running Program
```
cd scripts
python3 test.py
```
