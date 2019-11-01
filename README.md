# System requirements
- Ubuntu 16.04 or 18.04

# Installation
- Clone the repo:
```
git clone https://github.com/dut09/inverse_cad
```
- Install CUDA 10.0: Open https://developer.nvidia.com/cuda-10.0-download-archive and select your system to download the installer. Follow the instructions to install CUDA 10.0. Make sure you can see CUDA 10.0 by running `nvcc --version` before you proceed.
- Install miniconda:
```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
```
- Create a conda environment:
```
conda env create -f environment.yml
```
- Activate the conda environment:
```
conda activate sim_playground
```
- Run `install.sh` from the root folder:
```
./install.sh
```

# How to run
TODO
