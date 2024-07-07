# ROV Workstation Setup

## Overview
This section provides guidelines for setting up the workstation environment necessary to run and control the ROV.

## Table of Contents
- [Gstreamer Setup](#gstreamer-setup)
- [GPU Setup](#gpu-setup)



___

## Gstreamer Setup
Follow the steps in this [link](https://galaktyk.medium.com/how-to-build-opencv-with-gstreamer-b11668fa09c).

___

## GPU Setup
1. **Install Nvidia driver**: search for Additional Drivers in Ubuntu applications and choose:
<p align="center">
  <img src="images/nvidia_driver.png" width="500" height="300">
</p>

If the driver is not installed, you can install it manually. Follow this [link](https://www.nvidia.com/download/index.aspx).


2. **install cudnn**: Follow this [link](https://developer.nvidia.com/cudnn-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local).


3. **Install cuda toolkit**: Follow this [link](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local).

4. **Install anaconda**: Follow this [link](https://www.anaconda.com/download/).

5. Create a new environment named GPU.
    ```
    conda create -n gpu python=3.9
    ```

6. Activate this environment.
    ```
    conda activate gpu
    ```

7. Install pytorch gpu.
    ```
    conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia
    ```

8. Install ultralytics.
    ```
    pip install ultralytics
    ```

9. Install tensorflow gpu (optional).
    ```
    conda install -c anaconda tensorflow-gpu
    ```
    ```
    conda install -c conda-forge keras
    ```
10. Notebook support.
    ```
    conda install -c anaconda ipython
    ```
    ```
    conda install ipykernel
    ```
    ```
    conda install nb_conda_kernels
    ```
    ```
    pip install --upgrade nbconvert
    ```

11. Useful utils.
    ```
    pip3 install opencv-python
    ```
    ```
    conda install -c conda-forge matplotlib
    ```
    ```
    conda install -c anaconda pillow
    ```
    ```
    conda update pillow
    ```
    ```
    pip3 install numpy==1.23
    ```
    ```
    conda install -c anaconda scikit-learn
    ```
    ```
    conda install -c anaconda pandas
    ```
