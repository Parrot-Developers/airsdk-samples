
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
following command (Ubuntu 18.04 LTS):

```bash
# Drone build
./build.sh -p classic -t download-base-sdk

# Simulator build
./build.sh -p pc -t download-base-sdk
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
curl -X PUT "http://anafi2.local/api/v1/mission/missions/?allow_overwrite=yes" --data-binary @"out/hello-pc/images/com.parrot.missions.samples.hello.tar.gz"

# Drone upload (and set mission to default)
curl -X PUT "http://anafi2.local/api/v1/mission/missions/?allow_overwrite=yes&is_default=yes" --data-binary @"out/hello-classic/images/com.parrot.missions.samples.hello.tar.gz"
```

