# System requirements
- Ubuntu 18.04

# Installation
- Clone the repo:
```
git clone https://github.com/dut09/inverse_cad
```
- Run `install.sh` from the root folder:
```
./install.sh
```

# How to run
## Create Armando's sofa:
```
cd build
./inverse_cad_demo
extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +
extrude -0.8 0.2 1 0.8 0.2 1 0.8 0.8 1 -0.8 0.8 1 0 0 -0.5 -
save sofa.nef3
save sofa.off
exit
```
Here `sofa.nef3` is the b-rep file and `sofa.off` has the mesh information. You can then use `meshlab sofa.off` to visualize the shape.

## Reconstruct Armando's sofa:
```
cd build
./inverse_cad_demo
load_target sofa.nef3
ls v
ls e
ls f
extrude_ref f2 0 v0 v6 +
extrude_ref f16 0 v14 v11 -
save sofa_reconstructed.off
```
You can then compare `sofa.off` and `sofa_reconstructed.off` in Meshlab. They should be identical.