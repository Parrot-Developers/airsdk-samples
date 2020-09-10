
# Samples flight mission

## Files

.
├── build/                     Alchemy and build tools
├── out/                       Output build
│
├── packages/                  Source files
│   └── airsdk-doc/            Samples
│
├── products/                  Build configuration
└── sdk/
    ├── classic/               Parrot AirSDK for drone build
    │   ├── ...
    │   └── atom.mk
    └── pc/                    Parrot AirSDK for simulator build
        ├── ...
        └── atom.mk

## Build

Before building, Parrot AirSDK must be added to your work repository. It
contains all necessary headers as well as prebuilt libraries. It also contains
generated files used by our build system (`alchemy`) to specify include
directories required for each library.

The images should be extracted in the following directories:
- ./sdk/pc/ for the simulator image
- ./sdk/classic/ for the drone image

Then:
```bash
# Simulator build
./build.sh -p pc -t build -j

# Drone build
./build.sh -p classic -t build -j
```
**TODO: Image generation once implemented**

To add new dependencies or source files refer to the `atom.mk` and the alchemy
documentation.

