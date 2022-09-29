# LFMC-CVal

A C++ based deployment repository
for executing policies trained with [```LFMC-Gym```](https://github.com/ori-drs/lfmc_gym).
This git repository contains
two branches. The ```master``` branch provides an execution
example that requires [Raisim](https://raisim.com/) while
the ```library``` branch contains the minimal implementation
of the controller. 

#### Prerequisites
The deployment code depnds on [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
and [YAML-CPP](https://github.com/jbeder/yaml-cpp). These can be installed in Ubuntu like so:
```bash
sudo apt-get install libeigen3-dev libyaml-cpp-dev
```

#### Clone
To clone ```lfmc_cval```, use the following command. Note that, 
this repository depends upon a neural network implementation
written in C++ called [```networks_minimal```](https://github.com/gsiddhant/networks_minimal) 
and is included as a submodule. Ensure you
use ```--recurse-submodule``` flag while cloning the repository.

```bash
git clone --recurse-submodules git@github.com:ori-drs/lfmc_cval.git
```

#### Build
Upon cloning the repository, we build the library in the
```build``` folder. 
```bash
cd lfmc_cval
mkdir build && cd build
```

Note that, for ```CMake``` to be able to find Raisim, you need to ensure
that ```$LOCAL_INSTALL``` is an accessible environment variable and 
directs to the Raisim install location. You can either execute the
following command in your current shell or add it to your ```.bashrc```.
```bash
export LOCAL_INSTALL=<raisim-install-directory>
```

Assuming, Raisim has been correctly set up, you can then build the package.
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### Code Structure
    └── dependencies                    # Packages required by LFMC-CVal
        ├── networks_minimal            # C++ based implementation of MLP and GRU networks
    └── include                         # C++ header files
        ├── lfmc
            └── Controller.hpp          # LFMC controller interface class declaration
    └── src                             # C++ source files
        ├── Controller.cpp              # LFMC controller interface class function definitions
    └── CMakeLists.txt                  # C++ build utility

### Author(s)
[Siddhant Gangapurwala](mailto:siddhant@robots.ox.ac.uk)
