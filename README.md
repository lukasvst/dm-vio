<h1 align="center">DM-VIO: Delayed Marginalization<br/> Visual-Inertial Odometry </h1>

<p align="center">
<a href="https://vision.in.tum.de/_media/research/vslam/dm-vio/dm-vio.pdf">Paper</a> | <a href="https://youtu.be/7iep3BvcJPU">Video</a> | <a href="http://vision.in.tum.de/dm-vio">Project Page</a>
</p>

When using this project in academic work, please consider citing:

    @article{stumberg22dmvio,
      author = {L. von Stumberg and D. Cremers},
      title = {{DM-VIO}: Delayed Marginalization Visual-Inertial Odometry},
      journal = {{IEEE} Robotics and Automation Letters ({RA-L})},
      year = {2022},
      volume = {7},
      number = {2},
      pages = {1408-1415},
      doi = {10.1109/LRA.2021.3140129}
    }

### 1. Related Papers
* **[DM-VIO: Delayed Marginalization Visual-Inertial Odometry](https://vision.in.tum.de/dm-vio)**, L. von Stumberg and D. Cremers, In IEEE Robotics and Automation Letters (RA-L), volume 7, 2022
* **[Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization](https://vision.in.tum.de/vi-dso)**, L. von Stumberg, V. Usenko and D. Cremers, In International Conference on Robotics and Automation (ICRA), 2018
* **[Direct Sparse Odometry](https://vision.in.tum.de/dso)**, *J. Engel, V. Koltun, D. Cremers*, In  TPAMI, vol. 40, 2018

### 2. Installation from source
This branch is the Windows port of DM-VIO. No Windows specific APIs were introduced during porting, only the standard library equivalents of the \*NIX APIs were used. It might still compile under Linux or MacOS, but I haven't verified either. It was only tested with Windows 10, Visual Studio 2019.

All dependecies (except for Pangolin) are gathered via Conan. They are listed in the conanfile.txt and all of them are available from the default conancenter repository. All following command line instructions are to be executed in Git Bash which comes with the official Windows Git installer. To install the dependencies, invoke conan from within the DM-VIO folder:

	git clone https://github.com/AltVanguard/dm-vio.git -b windows_support
	cd dm-vio
	mkdir conan
	cd conan
	conan install ..

##### Pangolin
Like for DSO, [Pangolin](https://github.com/stevenlovegrove/Pangolin) is used for the GUI. Checkout Pangolin v0.6 to a separate folder (outside DM-VIO). Same process for conan dependencies:

	git clone https://github.com/AltVanguard/Pangolin.git -b v0.6
	cd Pangolin
	mkdir conan
	cd conan
	conan install ..

#### 2.3 Build
To build DM-VIO, you have to select installation folders for Pangolin and DM-VIO, then subsitute the folders into the following commands. I recommend not leaving the install folders as the default, and not installing the libraries under system folders or within the source repo folders. This avoids interference and you can easily restart from scratch if something goes wrong.

Build and install Pangolin first:

	cd Pangolin
    mkdir build
    cd build
    cmake .. \
		-DBUILD_EXAMPLES=OFF \
		-DBUILD_TESTS=OFF \
		-DBUILD_TOOLS=OFF \
		-DDISPLAY_X11=OFF \
		-DMSVC_USE_STATIC_CRT=OFF \
		-DCMAKE_INSTALL_PREFIX=<pangolin install folder> \
		-DCMAKE_MODULE_PATH=<pangolin repo folder>/conan/
    # Here open the build/*.sln file, and build and install the solution from Visual Studio
	
Build DM-VIO:

    cd dm-vio
    mkdir build
    cd build
    cmake .. \
		-DCMAKE_INSTALL_PREFIX=<dm-vio install folder> \
		-DCMAKE_PREFIX_PATH=<pangolin install folder> \
		-DCMAKE_MODULE_PATH=<dm-vio repo folder>/conan/
    # Here open the build/*.sln file, and build the solution from Visual Studio

This compiles `dmvio_dataset` to run DM-VIO on datasets (needs both OpenCV and Pangolin installed).
It also compiles the DM-VIO static library, which other projects can link to.

### 3 Running
Download a TUM-VI sequence (download in the format `Euroc / DSO 512x512`) at https://vision.in.tum.de/data/datasets/visual-inertial-dataset

    bin/dmvio_dataset
        files=XXXX/datasetXXXX/dso/cam0/images              
        vignette=XXXX/datasetXXXX/dso/cam0/vignette.png
        imuFile=XXXX/datasetXXXX/dso/imu.txt
        gtFile=XXXX/datasetXXXX/dso/gt_imu.csv
        calib=PATH_TO_DMVIO/configs/tumvi_calib/camera02.txt
        gamma=PATH_TO_DMVIO/configs/tumvi_calib/pcalib.txt
        imuCalib=PATH_TO_DMVIO/configs/tumvi_calib/camchain.yaml
        mode=0
        use16Bit=1
        preset=0                                                        # use 1 for realtime
        nogui=0                                                         # use 1 to enable GUI
        resultsPrefix=/PATH_TO_RESULTS/
        settingsFile=PATH_TO_DMVIO/configs/tumvi.yaml
        start=2                                                         

Instead of typing this long command you can use the [python tools](https://github.com/lukasvst/dm-vio-python-tools).

#### Running on EuRoC, 4Seasons, reproducing paper results, etc.
We **strongly recommend** using the python-dm-vio tools published at: https://github.com/lukasvst/dm-vio-python-tools

They can be used to
* prepare the EuRoC and 4Seasons sequences for usage with DM-VIO.
* run on all (or some) sequences of EuRoC, TUM-VI and 4Seasons and gather the results.
* create a Python evaluation script for inspecting the results and generating the plots shown in the paper.

#### Commandline arguments
There are two types of commandline arguments:
1. Main arguments defined `in main_dmvio_dataset.cpp` (see `parseArgument`). Most of these are derived from 
DSO, so you can read [src/dso/README.md](src/dso/README.md) for documentation on them. 
2. Lots of additional settings are defined using the `SettingsUtil`. They can be set either using comandline
or by placing them in the yaml file defined with the commandline argument `settingsFile`.
All of them are printed to commandline when the program starts (and also into the file `usedSettingsdso.txt`).
Most of these are documented in the header file they are defined in 
(see `src/IMU/IMUSettings.h`, `src/IMUInitialization/IMUInitSettings.h`).

#### Running on your own datasets
To run on your own dataset you need
* to pass the folder containing files with `files=...`
* an accurate camera calibration! For tips on calibration and the format of camera.txt see 
[src/dso/README.md](src/dso/README.md).
* to set the `mode=1` unless you have a photometric calibration (vignette.png and pcalib.txt).
* a file times.txt which contains **exactly** one timestamp for each image in the image folder.

When enabling IMU data you also need

* IMU calibration (transformation between camera and IMU) as a `camchain.yaml`. Note that only the field `cam0/T_cam_imu`
and optionally the noise values are read from this file.
* a file containing IMU data. For each image it **must** contain an IMU 'measurement' with exactly the same timestamp. 
If the sensor does not output this, a fake measurement with this timestamp has to be interpolated in advance.
    The [DM-VIO python tools](https://github.com/lukasvst/dm-vio-python-tools) contain a script to do this.
* You should also set the IMU noise values (see `configs/tumvi.yaml`, `configs/euroc.yaml`, and `configs/4seasons.yaml`).
You can read them from an Allan-Variance plot (either computed yourself or taken from datasheet of IMU). 
Note that often times these values are too small in practice and should be inflated by a large factor for optimal results.

You can first set `useimu=0` to try the visual-only system (basically DSO). If this does not work well for 
comparably slow motions, there is likely a problem with camera calibration which should be addressed first.


### 4 License
DM-VIO is based on Direct Sparse Odometry (DSO), which was developed by Jakob Engel 
at the Technical University of Munich and Intel.
Like DSO, DM-VIO is licensed under the GNU General Public License
Version 3 (GPLv3).
