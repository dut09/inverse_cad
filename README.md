## System requirements
- Ubuntu 18.04

## Installation
- Clone the repo:
```
git clone https://github.com/dut09/inverse_cad
```
- Create a conda environment and activate it:
```
conda env create -f environment.yml
conda activate inverse_cad
```
- Run `install.sh` from the root folder:
```
./install.sh
```

## How to run
`./inverse_cad_demo` is an interactive session: it maintains a polyhedron and you type in a few commands to modify it. The polyhedron is empty at the beginning. Below are the detailed explanations of each command:
### Loading and saving
`.nef3` is the proprietary file format designed by CGAL for handling 3D solids and boolean operations. We use it as the b-rep file in our project. A `.nef3` file includes the full information of a polyhedron. You can use the `load` command to load an `.nef3` file to the interactive session. For example:
```
>>> load sofa.nef3
```
will overwrite the polyhedron in the canvas-so-far with the one defined in `sofa.nef3`.

The `save` command can take as input either a `.nef3` or `.off` file. For example, you can do
```
>>> save sofa.nef3
```
or
```
>>> save sofa.off
```
to save the polyhedron in the canvas-so-far into either a `.nef3` file which can be reloaded later or a `.off` file which can be used for visualization.

We also present a `load_target` command to load a `.nef3` file describing the target polyhedron, i.e., the one that we want our polyhedron to eventually match after applying a finite number of commands. For example,
```
>>> load_target sofa.nef3
```
will read the full description of the polyhedron in `sofa.nef3` and set it as our target polyhedron. This command won't modify the polyhedron in the canvas-so-far.

### Exploring topological information
We provide `ls v`, `ls e`, and `ls f` to display the vertex, edge, and facet information of the polyhedron in the canvas-so-far respectively. Additionally, you can add the `--target` flag (for example, `ls v --target`) to display the same information of the *target* polyhedron.

The `ls v` command starts with a line of `Vertex number N` where `N` is the number of vertices of the polyhedron in the canvas-so-far. It is followed by `N` lines, one for each vertex, describing the name of each vertex and its 3D coordinates. Here is a sample output:
```
>>> ls v
Vertex number 22
v0      1       0       0
v1      1       1       0
v2      -1      1       0
...
v21     0       0.2     0.5
```
You can assume the name of a vertex always starts with a `v` followed by an index number increasing from `0` to `N - 1`.

The `ls e` command starts with a line of `Edge number N` where `N` is the number of *half edges*. For each edge connecting vertices `vi` and `vj`, two half edges are created, one pointing from `vi` to `vj` and the other from `vj` to `vi`. Each half edge is described by a line starting with its name, the source vertex, the target vertex, and its twin half edge connecting the same vertices in the opposite direction. Here is a sample output:
```
>>> ls e
Edge number 66
e0      v0      v5      twin    e16
e1      v0      v6      twin    e18
...
e16     v5      v0      twin    e0
...
e18     v6      v0      twin    e1
...
e65     v21     v19     twin    e58
```
As in the case of vertices, you can assume the name of a half edge always starts with an `e` followed by an index number increasing from `0`.

Finally, the `ls f` command shows information about the half facets. For each facet of the polyhedron in the canvas-so-far, two half facets are created with opposite orientations. A half facet has a unique name, a twin half facet, and most importantly, a list of vertex cycles that defines the boundary of this facet. Note that a facet can have multiple vertex cycles if it contains one or more holes. Here is a sample output:
```
>>> ls f
Face number 26
f0      1
v9      v17     v13     v12     v11     v18     v10     v6      v7      v8
twin    f1
f1      1
v9      v8      v7      v6      v10     v18     v11     v12     v13     v17
twin    f0
...
```
Here the number `N` after the facet name shows how many vertex cycles this facet contains. It is then immediately followed by `N` lines, one for each vertex cycle, showing the vertex names in that cycle.

### Modeling
We present two commands `extrude` and `extrude_ref` to model the shape. `extrude` takes as input a list of 3D points that define a polygon on a plane, a 3D direction, and a `+` or `-` indicating whether the extrusion should be merged into or subtracted from the polyhedron in the canvas-so-far. For example:
```
extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +
```
This command first defines a polygon with 6 vertices: `(1, 0, 0)`, `(1, 1, 0)`, `(-1, 1, 0)`, `(-1, 0.5, 0)`, `(0, 0.5, 0)`, and `(0, 0, 0)` on the plane `z = 0`. It then defines an extrusion direction `(0, 0.1, 1)`. The final `+` means that the newly created extrusion will be merged into the polyhedron in the canvas-so-far.

The `extrude_ref` command is similar but replaces all the numerical numbers with references to elements of the polyhedron in the canvas-so-far. For example:
```
>>> extrude_ref f2 0 v0 v6 +
```
Here `f2` is the facet you pick, `0` is the vertex cycle index to use in that facet, and `v0` and `v6` define an extrusion direction (from vertex `v0` to vertex `v6`). Putting it together, this command extrudes the first vertex cycle in facet `f2` along the direction `v6 - v0` and merges the result into the current polyhedron.

Just as the `ls` command above, we also provide the `--target` flag for `extrude_ref`. This flag allows you to use the elements from the target polyhedron instead of the polyhedron in the canvas-so-far.

### Generating random polygon
We present a command `rand_poly` to generate a random polygon from a selected surface. This command takes as input a half facet name, two probabilities that control the complexity of the generated polygons (larger probabilities means simpler polygons), and an optional `--target` command. For example:
```
>>> rand_poly f2 0.5 0.5 --target
```
The command will first pick the half facet from either the target or the polyhedron in the canvas-so-far and check if the facet contains holes (by checking the number of vertex cycles). It will terminate if the facet contains a hole. The command will then create a random polygon on the facet and return a string `x1 y1 z1 x2 y2 z2 ...` describing the vertices of the generated polygon. The order of the vertices is guaranteed to be the same as the order of the vertex cycle in the half facet. The random polygon is guaranteed to be valid but does not need to be convex.

### Converting between `.nef3` and `.off`
In this project, `.off` files are used only to visualize a polyhedron (in the form of a triangle mesh). Modeling, including extrusion and boolean operations for now, should work with `.nef3` files. We present a `convert` command to convert between these two file formats. For example:
```
>>> convert sofa.nef3 sofa.off
>>> convert sofa.off sofa.nef3
```
In both cases, the command reads data from the first file and write it to the second file after conversion.

### Exit
Finally, you can exit the interactive session by typing `exit`:
```
>>> exit
```

## Examples
### Creating Armando's sofa
```
cd build
./inverse_cad_demo
>>> extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +
>>> extrude -0.8 0.2 1 0.8 0.2 1 0.8 0.8 1 -0.8 0.8 1 0 0 -0.5 -
>>> save sofa.nef3
>>> save sofa.off
>>> exit
```
Here `sofa.nef3` is the b-rep file and `sofa.off` has the mesh information. You can then use `meshlab sofa.off` to visualize the shape.

### Reconstructing Armando's sofa
```
cd build
./inverse_cad_demo
>>> load_target sofa.nef3
>>> ls v --target
>>> ls e --target
>>> ls f --target
>>> extrude_ref f2 0 v0 v6 + --target
>>> ls v
>>> ls e
>>> ls f
>>> extrude_ref f16 0 v14 v11 - --target
>>> save sofa_reconstructed.nef3
>>> save sofa_reconstructed.off
>>> exit
```
You can then compare `sofa.off` and `sofa_reconstructed.off` in Meshlab. They should be volumetrically identical.

## Python bindings
You can read and run `python test_binding.py` from the root folder to understand how to use the Python class. Bindings complement of `swig`, which in theory means that they should exactly mirror the C++ implementation with regards to naming etc.