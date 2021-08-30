
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

Before building, you need to get AirSDK in your work repository with the
following command:

```bash
# Simulator build
./build.sh -p pc -t download-base-sdk

# Drone build
./build.sh -p classic -t download-base-sdk
```

Then, you can build with the following command:

```bash
# Simulator build
./build.sh -p hello-pc -t all -j

# Drone build
./build.sh -p hello-classic -t all -j
```

To add new dependencies or source files refer to the `atom.mk` and the alchemy
documentation.

## Mission usage

```bash
# Simulator upload
./build.sh -p hello-pc -t sync --reboot

# Drone upload (and set mission to default)
./build.sh -p hello-classic -t sync --is-default --reboot
```

