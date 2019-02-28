# FieldSAFE - Dataset for Obstacle Detection in Agriculture

This repository contains the necessary software for utilizing the FieldSAFE dataset.
Software and example usage scripts are available in the folder "ros".
Further, ground truth GPS annotations for all static and dynamic obstacles are contained in the folder "ground_truth".

For more information, visit the FieldSAFE website: [https://vision.eng.au.dk/fieldsafe/](https://vision.eng.au.dk/fieldsafe/)

## Citation
If you use this dataset in your research or elsewhere, please cite/reference the following paper:

[FieldSAFE: Dataset for Obstacle Detection in Agriculture](https://arxiv.org/abs/1709.03526)

```sh
@article{kragh2017fieldsafe,
  title={FieldSAFE: Dataset for Obstacle Detection in Agriculture},
  author={Kragh, Mikkel Fly and Christiansen, Peter and Laursen, Morten Stigaard and Larsen, Morten and Steen, Kim Arild and Green, Ole and Karstoft, Henrik and J{\o}rgensen, Rasmus Nyholm},
  journal={arXiv preprint arXiv:1709.03526},
  year={2017}
}
```

## Installation Instructions
The FieldSAFE dataset and software has been tested with Ubuntu 16.04 and ROS Kinetic, but may work with other Linux distributions and newer ROS distributions.
Below, installations instructions for all necessary dependencies are given.

* Install ROS Kinetic on Ubuntu 16.04 (Desktop-Full Install)

    http://wiki.ros.org/kinetic/Installation/Ubuntu

* Install the following additional packages:
    ```sh
    sudo apt-get install ros-kinetic-robot-localization 
    sudo apt-get install ros-kinetic-geographic-msgs
    sudo apt-get install libpcap-dev
    ```
* Clone and build this repository
    ```sh
    git clone https://github.com/mikkelkh/FieldSAFE
    cd FieldSAFE
    git submodule update --init --recursive
    cd ros
    catkin_make
    ```
* Environment Setup
    ```sh
    source devel/setup.bash
    ```
* Download a 1 minute example bag with sensor data: 

    [2016-10-25-11-41-21_example.bag](https://vision.eng.au.dk/data/FieldSAFE/2016-10-25-11-41-21_example.bag)

* Run the original demo
    ```sh
    roslaunch demo demo.launch file:=/path/to/2016-10-25-11-41-21_example.bag
    ```
    or this updated demo by [@tambetm](https://github.com/tambetm) including visualization of ground truth obstacles:
    ```sh
    roslaunch demo demo_markers.launch file:=/path/to/2016-10-25-11-41-21_example.bag
    ```
* Download more data from: 

    [https://vision.eng.au.dk/fieldsafe/](https://vision.eng.au.dk/fieldsafe/)
