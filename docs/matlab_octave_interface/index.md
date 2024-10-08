# Matlab + Simulink and Octave Interface

In order to use `acados` from Octave or Matlab, you need to create the `acados` shared libraries using either the `CMake` or `Make` build system, as described [on the installation page](../installation/index.md).

## Getting started
Check out the examples [`new_minimal_example_ocp.m`](https://github.com/acados/acados/tree/master/examples/acados_matlab_octave/getting_started/new_minimal_example_sim.m) and [`new_minimal_example_sim.m`](https://github.com/acados/acados/tree/master/examples/acados_matlab_octave/getting_started/new_minimal_example_sim.m) to get started with the Matlab interface of `acados`.
Note that `acados` currently supports both an old Matlab interface (< v0.4.0) as well as the new one (>= v0.4.0).
Unfortunately, not all MATLAB examples have been ported to the new interface yet.
If you are new to `acados` please start with [those examples](https://github.com/acados/acados/issues/1196#issuecomment-2311822122) that use the new interface already.


The examples require an installation of `CasADi` to generate the model functions.
The `getting_started` example offers the option to attempt to automatically download the correct version in the recommended folder.
Detailed instructions for a manual installation can be found in the last section of this page [Setup CasADi](#setup-casadi).

The problem formulation is stated in [this PDF](https://github.com/acados/acados/tree/master/docs/problem_formulation/problem_formulation_ocp_mex.pdf).



## Export environment variables
In order to run the examples, some environment variables need to be exported.
Instead of running the scripts below, you can modify an `rc` file, like `.bashrc` when launching MATLAB from bash,
[`.matlab7rc.sh`](https://discourse.acados.org/t/matlab-mex-more-elegant-way-to-setup-env-sh/62/4) or `startup.m` to always have those environment variables defined when starting `Matlab`.

### Linux / macOS
Navigate into the folder of the example you want to run and execute the following command:
```
source env.sh # Which can be found in the folder of one of the examples
```

If you want to run an `acados` example from another folder, you need to export the environment variable `ACADOS_INSTALL_DIR` properly.
In the `env.sh` file it is assumed that `ACADOS_INSTALL_DIR` is two folders above the directory, in which the example is located.

Afterwards, launch `Matlab` or `Octave` from the same shell.

If you want to run the examples in a different folder, please close the current shell and open a new one to repeat the procedure: this ensures the correct setting of the environment variables.

### Windows
1. Open `Matlab` and navigate into [`<acados_root>/examples/acados_matlab_octave`](https://github.com/acados/acados/blob/master/examples/acados_matlab_octave).
2. Run [`acados_env_variables_windows`](https://github.com/acados/acados/blob/master/examples/acados_matlab_octave/acados_env_variables_windows.m) to export the environment variable `ACADOS_INSTALL_DIR`.
3. Navigate into [`<acados_root>/examples/acados_matlab_octave/getting_started`](https://github.com/acados/acados/tree/master/examples/acados_matlab_octave/getting_started) and run one of the examples.


## Interface structure
The interface allows one to conveniently and compactly formulate an OCP (or IVP) and specify solver options.
The nonlinear problem functions can be formulated using CasADi symbolics which are generated as C code with the required derivatives using automatic differentiation.
The whole problem description is written to a json-file which is then used to render different templates, via the `Tera` renderer.
These are the same templates as in the Python interface (see [`Python interface`](../python_interface/index.md)).
In addition to a `MEX` wrapper it contains all the `C` code that is needed for embedded deployment.
These templates can be found in [`<acados_root>/interfaces/acados_template/acados_template/c_templates_tera`](https://github.com/acados/acados/tree/master/interfaces/acados_template/acados_template/c_templates_tera).

## Options documentation
For the template based part of the `Matlab` interface, we refer to [the docstring based documentation of the Python interface](../python_interface/index.md).

## Simulink
The templates mentioned [above](#templates) also contain templated S-functions and corresponding make functions for Matlab for both the OCP solver and the acados integrator.

A basic Simulink example can be found in [`<acados_root>/examples/acados_python/getting_started/simulink_example.m`](https://github.com/acados/acados/blob/master/examples/acados_matlab_octave/getting_started/simulink_example.m)

A more advanced Simulink example which showcases how to customize the inputs and outputs of the Simulink block corrsponding to the solver can be found in [`<acados_root>/examples/acados_python/getting_started/simulink_example.m`](https://github.com/acados/acados/blob/master/examples/acados_matlab_octave/getting_started/simulink_example_advanced.m)

If you want a more advanced interaction with the `acados` solver via Simulink, feel free to edit the corresponding templates in [`<acados_root>/interfaces/acados_template/acados_template/c_templates_tera`](https://github.com/acados/acados/tree/master/interfaces/acados_template/acados_template/c_templates_tera) to add more inputs or outputs.

### S-function Mask
The Simulink S-functions interface can be improved using the following mask commands, which shows the names of input and output ports, facilitating debugging operations.
These commands use global variables that are automatically generated by the `make_sfun` command.
1. OCP S-function mask
```
global sfun_input_names sfun_output_names
for i = 1:length(sfun_input_names)
	port_label('input', i, sfun_input_names{i})
end
for i = 1:length(sfun_output_names)
	port_label('output', i, sfun_output_names{i})
end
```
2. SIM S-function mask
```
global sfun_sim_input_names sfun_sim_output_names
for i = 1:length(sfun_sim_input_names)
	port_label('input', i, sfun_sim_input_names{i})
end
for i = 1:length(sfun_sim_output_names)
	port_label('output', i, sfun_sim_output_names{i})
end
```
To use the mask command just copy-paste it in the "icon drawing commands" field, accessible by right clicking on the S-function block - Mask - Edit Mask.

## Setup CasADi
To create external function for your problem, we suggest to use `CasADi` from the folder `<acados_root_folder>/external`.
Depending on the environment you want to use to generate `CasADi` functions from, proceed with the corresponding paragraph (Matlab, Octave).

Any CasADi version between 3.4.0 and 3.6.5 should work.
If you don't have CasADi yet, you can install it as described below.

### **Matlab**
Download and extract the `CasADi` binaries into `<acados_root_folder>/external/casadi-matlab`:
```
cd external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.4.0/casadi-linux-matlabR2014b-v3.4.0.tar.gz
mkdir -p casadi-matlab
tar -xf casadi-linux-matlabR2014b-v3.4.0.tar.gz -C casadi-matlab
cd ..
```

### **Octave version 4.4 or later**
Download and extract the `CasADi` binaries into `<acados_root_folder>/external/casadi-octave`:
```
cd external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.4.5/casadi-linux-octave-4.4.1-v3.4.5.tar.gz
mkdir -p casadi-octave
tar -xf casadi-linux-octave-4.4.1-v3.4.5.tar.gz -C casadi-octave
```

### **Octave version 4.2 or earliear**
Download and extract the `CasADi` binaries into `<acados_root_folder>/external/casadi-octave`:
```
cd external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.4.0/casadi-linux-octave-v3.4.0.tar.gz
mkdir -p casadi-octave
tar -xf casadi-linux-octave-v3.4.0.tar.gz -C casadi-octave
cd ..
```
