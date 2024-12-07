# VoxelizeGeometry 
## A.K.A. Cube-a-Saurus

**Cube-a-Saurus** provides tools to voxelize 3D geometries into uniformly sized cubic Breps (voxels) and evaluate their spatial relationship with reference geometries and points. The component is ideal for applications in 3D modeling, architecture, computational design, and manufacturing, where discrete representations of complex geometries are needed.

This work is an ongoing PhD research project at the University of Michigan. This component was initially used to translate FEA load data to color-informed voxels that were then converted into 3D printing toolpaths. 
![Cube-a-Saurus](https://github.com/ChristopherVoltl/VoxelizeGeometry/blob/master/assets/cube-a-sarous.jpg)

What does this component do? 

## Features
1. **Voxelization:**
  - Generates cubic Breps (voxels) with a specified size to represent the input Brep geometry.
2. **Containment Analysis:**
  - Tests each voxel to determine whether it is entirely inside, intersecting, or outside a reference Brep.
3. **Boolean Splitting:**
  - Splits voxelized Breps using the input Brep as a splitting boundary.
  - Filters resulting Breps to include only those inside or on the surface of the reference Brep.
4. **Point Influence:**
  - Accepts a list of points to influence the color or categorization of the generated voxels.
5. **Interior Voxel Extraction:**
  - Separates and outputs voxels that are fully contained within the reference Brep.
6. **Customizability:**
  - Includes parameters for voxel size, Brep input, and influencing points for flexibility in various design scenarios.

**Inputs**
1. **Brep (B):**
  - The geometry to be voxelized.
2. **Voxel Size (V):**
  - The uniform size of the cubic voxels.
3. **Points (P):**
  - A list of points used to influence voxel properties (e.g., color).

**Outputs**
1. **All Voxels (V):**
  - A list of all generated voxels as cubic Breps.
2. **Colors (C):**
  - Corresponding colors for each voxel based on point influence or other criteria.
3. **Split Breps (SB):**
  - Voxels that have been split and filtered using the reference Brep.
4. **Interior Voxels (SB):**
  - Voxels entirely contained within the input Brep geometry.

**Internal Logic**

1. **Voxel Generation:**

  - A bounding box is computed for the input Brep.
  - Based on the specified size, the bounding box is divided into a grid of uniform cubes (voxels).
  - Each voxel is checked for intersection or containment within the input Brep.

2. **Boolean Splitting:**

  - Uses Rhino’s Brep.CreateBooleanSplit method to split voxels that intersect the input Brep.
  - Filters the resulting Breps to include only those entirely within or on the boundary of the input Brep.

3. **Interior Filtering:**

  - Extracts voxels that are fully inside the Brep using a robust containment check (IsFullyInside).

4. Containment Check:

  - Evaluates Breps based on their vertices, centroid, and optionally sampled surface points to ensure comprehensive containment analysis.

![Component Features](https://github.com/ChristopherVoltl/VoxelizeGeometry/blob/master/assets/1Artboard%201-100.jpg)

