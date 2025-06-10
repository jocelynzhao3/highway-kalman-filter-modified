# Unscented Kalman Filter for Highway

## 1, Overview
This project implements an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy *lidar* and *radar* measurements. We'll then evaluate the quality of our filter using RMSE values.

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has its own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so we're only tracking along the X/Y axis in this project.


## 2. Table of Contents
- [Project Instructions](#build)
- [UKF Implementation](#implementation)
- [Acknowledgements](#acknowledgements)


## 3. Project Instructions <a name="build"></a>
The main program can be built and ran by doing the following from the project top directory.

1. Clone this repo with LFS, which can be done in two ways:
  1. `git lfs clone https://github.com/moorissa/lidar-obstacle-detector.git` OR
  2. Alternatively:
  ```bash
    git clone https://github.com/moorissa/lidar-obstacle-detector.git
    cd lidar-obstacle-detector  # ensure no duplicated names in the same directory
    git lfs pull
  ```
  If LFS continues causing (submission) issues:
   - Upload PCD files to a cloud service (Google Drive, Dropbox) and include download links
   - Use smaller sample PCD files that don't require LFS
   - Compress the PCD files if possible
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make -j`
4. Run it: `./ukf_highway`

<img src="media/ukf_highway.png" width="700" height="400" />

#### Dependencies
* cmake >= 3.10
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 3.8
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* PCL >= 1.10
  * [Documentation](https://pointclouds.org/downloads/)

#### Currently used (2025 version):
* MacOS: Sequoia 15.5
* cmake: 3.31.7
* GNU make: 3.81
* gcc: 
  ```
  Target: arm64-apple-darwin24.5.0
  Thread model: posix
  InstalledDir: /Library/Developer/CommandLineTools/usr/bin
  ```
* pcl: stable 1.15.0

#### Generating Additional Data
If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment and performing clustering.


## UKF Implementation <a name="implementation"></a>

### 1. Parameters
In the `highway.h`, there are a number of parameters we can modify for debugging purpose.
- `trackCars` list can toggle on/off cars for UKF object to track
- `projectedTime` and `projectedSteps` controls the visualization of predicted position in the future
- `visualize_pcd` sets the visualization of Lidar point cloud data

```c++
// Set which cars to track with UKF
std::vector<bool> trackCars = {true,true,true};
// Visualize sensor measurements
bool visualize_lidar = true;
bool visualize_radar = true;
bool visualize_pcd = false;
// Predict path in the future using UKF
double projectedTime = 0;
int projectedSteps = 0;
```

### 2. Code Walkthrough



## Ackowledgements <a name="acknowledgements"></a>
* [Udacity Sensor Fusion Program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

For any questions or feedback, feel free to email [moorissa.tjokro@columbia.edu](mailto:moorissa.tjokro@columbia.edu).


