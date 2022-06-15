## Live Demo for Realsense T265 Camera

There is a live version of the code which runs on Realsense cameras (currently only T265 is officially supported).

### Installation

The live version will automatically be compiled together with the main code if Librealsense is installed.

You can download and install it from: https://github.com/IntelRealSense/librealsense

On Ubuntu you can alternatively use the apt binaries provided by the
project [Basalt](https://github.com/VladyslavUsenko/basalt-mirror):

    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 0AD9A3000D97B6C9
    sudo sh -c 'echo "deb [arch=amd64] http://packages.usenko.eu/ubuntu $(lsb_release -sc) $(lsb_release -sc)/main" > /etc/apt/sources.list.d/basalt.list'
    sudo apt-get update
    sudo apt install librealsense2-dev

On Mac you can install it with Homebrew

    brew install librealsense

Afterwards you will need to rebuild DM-VIO.

### Running:

Run with

    bin/dmvio_t265 
        useimu=1 
        mode=0 
        preset=1 
        nogui=0 
        quiet=1 
        start=2
        settingsFile=PATH_TO_DMVIO/configs/t265_noise_tumvi.yaml
        resultsPrefix=/PATH_TO_RESULTS/
        calibSavePath=/FACTORY_CALIBRATION_WILL_BE_SAVED_HERE.txt
        vignette=PATH_TO_DMVIO/configs/realsense/vignette_t265.png
        gamma=PATH_TO_DMVIO/configs/pcalib_linear_8bit.txt

#### Notes and useful commandline parameters:

* The T265 has a fisheye lens so we **highly** recommend calibrating the vignette. You can start with the provided
  vignette (as in the command above), but for better results we recommend calibration the vignette yourself, e.g. using
  the [Basalt tools](https://github.com/VladyslavUsenko/basalt-mirror/blob/master/doc/Realsense.md) or
  the [TUM-mono dataset tools](https://github.com/tum-vision/mono_dataset_code)
* By default, the code will use the (geometric) factory calibration. However, it can be better to calibrate the camera
  yourself (e.g. using Kalibr or Basalt) and supply your own calibration. This works by
  passing `calib=PATH_TO_YOUR/camera.txt` as a commandline argument.

### Adjusting the config file

There are some settings which can be tuned based on your application. See our provided default
config `configs/t265_noise_tumvi.yaml` for reference.

* Noise values: We recommend to start with the noise values tuned for the TUM-VI dataset and only use custom noise
  values if they seem to improve the performance.
* Frame skipping settings: If the system is slower than framerate it can be useful to skip frames. You can control this
  separately for different parts of the system: The visual initializer is still unchanged from DSO which means that
  it is comparably slow. For the live demo we address this by not skipping frames during this stage to improve stability.
  However, this also means that some delay will be accumulated during this stage, which will be caught up after (visual)
  initialization is finished. To control this behaviour you can adjust the following settings.

      maxSkipFramesVisualOnlyMode: 1      # This many frames can be skipped at a time while in visual-only mode.
      maxSkipFramesVisualInertial: 2      # This many frames can be skipped at a time while in visual-inertial mode.
      skipFramesVisualOnlyDelay: 30       # After visual initializer finished, wait this amount if frames before switching to the visualOnly threshold. This is useful because during the first frames the system might be less stable than later.
      minQueueSizeForSkipping: 2          # Don't skip frames if the image queue is smaller than this.
      maxTimeBetweenInitFrames: 1.0       # Reset the visual initializer if this time has passed between the first and last frame.
  You might especially want to adjust the `maxTimeBetweenInitFrames` depending on how fast you move the camera initially
  and how much delay is allowed.
* The following settings make the system more robust against bad visual initialization. You might want to adjust the
  `init_requestFullResetNormalizedErrorThreshold` to your use-case. For this you can have a look at the commandline
  output lines starting with `CoarseIMUInit normalized error: ` and then set the threshold accordingly.

      init_pgba_skipFirstKFs: 1
      init_requestFullResetNormalizedErrorThreshold: 0.1  # If the error after the CoarseIMUInit is larger than this we assume the visual system failed and reset the full system.
* If you know that the camera will not be pointed at clouds (e.g. for indoor demos) you can add the following setting
  which will include points with large depth.

      setting_minIdepth: 0.0
* For automotive demos I highly recommend adding the following settings taken from the `4seasons.yaml`. (The former two
  only work if the camera is mounted approximately looking forward.)

      setting_weightZeroPriorDSOInitY: 5e09
      setting_weightZeroPriorDSOInitX: 5e09
      setting_forceNoKFTranslationThresh: 0.01
* Advanced: If you have a good guess for the initial scale of the scene it might be better to immediately initialize VIO
  with this approximate scale instead of waiting until it becomes observable. For this you can have a look at the
  setting `init_disableVIOUntilFirstInit` but at the moment it is not officially supported, so you will need to adjust
  the code for it.

### Recording and playing back a dataset

You can also record the images and IMU data to file while running the live demo. This works by providing the
argument `saveDatasetPath=/DATASET_WILL_BE_SAVED_HERE`
Below are sample scripts demonstrating how to use it in practice.

#### live_demo_with_saving.sh

    savefolder=/FOLDER/TO/DATASETS/$1                                                                                                                                                                                                       
    dm-vio/cmake-build-relwithdebinfo/bin/dmvio_t265 useimu=1 mode=0 preset=1 nogui=0 quiet=1 start=2 settingsFile=dm-vio/configs/t265_noise_tumvi.yaml resultsPrefix=./runResults/ calibSavePath=./RealsenseCalibration/factory_calib.txt camchainSavePath=./RealsenseCalibration/factory_camchain.yaml vignette=dm-vio/configs/realsense/vignette_t265.png gamma=dm-vio/configs/pcalib_linear_8bit.txt saveDatasetPath=$savefolder
    python3 dm-vio-python-tools/interpolate_imu_file.py --input $savefolder/imu_orig.txt --times $savefolder/times.txt --output $savefolder/imu.txt

#### run_on_dataset.sh

    savefolder=/FOLDER/TO/DATASETS/$1                                                                                                                                                                                                       
    dm-vio/cmake-build-relwithdebinfo/bin/dmvio_dataset useimu=1 mode=0 preset=0 nogui=0 quiet=1 start=2 settingsFile=dm-vio/configs/t265_noise_tumvi.yaml resultsPrefix=./runResults/ vignette=dm-vio/configs/realsense/vignette_t265.png gamma=dm-vio/configs/pcalib_linear_8bit.txt files=$savefolder/cam0 imuCalib=./RealsenseCalibration/factory_camchain.yaml calib=./RealsenseCalibration/factory_calib.txt

To make this work you will need to adjust the paths for the arguments.

Then you can simply run:

    ./live_demo_with_saving seq00
    ./run_on_dataset seq00

Note: It seems like the setting `init_requestFullResetNormalizedErrorThreshold` needs to be significantly (~factor 10)
larger when running on a recorded dataset compared to running live. If in doubt you can run with a large value first and
have a look at the commandline output lines starting with `CoarseIMUInit normalized error: ` and then set the threshold
accordingly.
