using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace VoxelizeGeometry
{
    public class VoxelizeGeometryComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        
        public VoxelizeGeometryComponent()
          : base("Cube-a-saurus", "CAS",
            "A prehistoric beast that only eats geometry and spits out voxels",
            "FGAM", "Cube-a-saurus")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "Input Brep", GH_ParamAccess.item);
            pManager.AddNumberParameter("Voxel Size", "V", "Size of each voxel", GH_ParamAccess.item, 2.0);
            pManager.AddPointParameter("Points", "P", "Points to influence voxel colors", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBoxParameter("All Voxels", "V", "Generated voxels", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Colors for each voxel", GH_ParamAccess.list);
            pManager.AddBrepParameter("Split Breps", "SB", "Resulting split Breps", GH_ParamAccess.list);
            pManager.AddBrepParameter("Interior Voxels", "SB", "Voxels Inside Brep Geometry", GH_ParamAccess.list);
        }
    

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Inputs
            Brep brep = null;
            double voxelSize = 2.0;
            List<Point3d> points = new List<Point3d>();
            List<Brep> splittingBreps = new List<Brep>();

            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref voxelSize)) return;
            if (!DA.GetDataList(2, points)) return;


            // Voxelization logic as before...
            BoundingBox bbox = brep.GetBoundingBox(true);
            Point3d minPt = bbox.Min;
            Point3d maxPt = bbox.Max;

            double xSpan = maxPt.X - minPt.X;
            double ySpan = maxPt.Y - minPt.Y;
            double zSpan = maxPt.Z - minPt.Z;

            int xVoxels = (int)(xSpan / voxelSize) + 1;
            int yVoxels = (int)(ySpan / voxelSize) + 1;
            int zVoxels = (int)(zSpan / voxelSize) + 1;

            List<Brep> voxels = new List<Brep>();
            List<System.Drawing.Color> colors = new List<System.Drawing.Color>();

            for (int i = 0; i < xVoxels; i++)
            {
                for (int j = 0; j < yVoxels; j++)
                {
                    for (int k = 0; k < zVoxels; k++)
                    {
                        double x = minPt.X + i * voxelSize;
                        double y = minPt.Y + j * voxelSize;
                        double z = minPt.Z + k * voxelSize;

                        Box voxel = new Box(new BoundingBox(
                            new Point3d(x, y, z),
                            new Point3d(x + voxelSize, y + voxelSize, z + voxelSize)
                        ));

                        // Check if voxel intersects Brep
                        if (voxel.ToBrep().IsPointInside(brep.ClosestPoint(voxel.Center), 0.001, true))
                        {
                            voxels.Add(voxel.ToBrep());
                            colors.Add(System.Drawing.Color.White); // Default color
                        }
                        if (brep.IsPointInside(voxel.Center, 0.001, true))
                        {
                            voxels.Add(voxel.ToBrep());
                            colors.Add(System.Drawing.Color.White); // Default color
                        }

                    }
                }
            }

            // Split Breps with Breps
            List<Brep> splitBreps = BooleanSplitAndFilterBrepsAccurate(voxels, brep);
            List<Brep> interiorBreps = GetInteriorVoxels(voxels, brep);


            // Display the breps inside the geometry


            // Outputs
            DA.SetDataList(0, voxels);
            DA.SetDataList(1, colors);
            DA.SetDataList(2, splitBreps);
            DA.SetDataList(3, interiorBreps);

        }

        public List<Brep> GetInteriorVoxels(List<Brep> voxels, Brep brep)
        {
            List<Brep> interiorBreps = new List<Brep>();

            foreach (Brep voxel in voxels)
            {
                //Check for interior Voxels
                if (IsFullyInside(voxel, brep))
                {
                    interiorBreps.Add(voxel);

                }
            }
            return interiorBreps;
        }

            public List<Brep> BooleanSplitAndFilterBrepsAccurate(List<Brep> brepsToSplit, Brep splittingBrep)
        {
            List<Brep> filteredBreps = new List<Brep>();

            foreach (Brep brepToSplit in brepsToSplit)
            {
                // Perform Boolean Split
                Brep[] splitPieces = Brep.CreateBooleanSplit(brepToSplit, splittingBrep, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                if (splitPieces != null && splitPieces.Length > 0)
                {
                    // Check each split piece
                    foreach (Brep piece in splitPieces)
                    {
                        if (IsFullyInsideOrOnSurface(piece, splittingBrep))
                        {
                            filteredBreps.Add(piece);
                        }
                    }
                }
                else
                {
                    // If no split occurred, check if the original brepToSplit is fully inside or on the surface
                    if (IsFullyInsideOrOnSurface(brepToSplit, splittingBrep))
                    {
                        filteredBreps.Add(brepToSplit);
                    }
                }
            }

            return filteredBreps;
        }
        private bool IsFullyInside(Brep testBrep, Brep containerBrep)
        {
            if (testBrep == null || containerBrep == null || !containerBrep.IsSolid)
                return false; // Early exit for invalid inputs or non-closed container Breps

            // Tolerance
            double tolerance = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;

            // Collect points to test: vertices and centroid
            List<Point3d> pointsToTest = new List<Point3d>();

            // Add vertices of the test Brep
            foreach (BrepVertex vertex in testBrep.Vertices)
            {
                pointsToTest.Add(vertex.Location);
            }

            // Add the centroid of the test Brep
            Point3d centroid = AreaMassProperties.Compute(testBrep)?.Centroid ?? Point3d.Unset;
            if (centroid.IsValid)
                pointsToTest.Add(centroid);

            // Test each point for containment
            foreach (Point3d point in pointsToTest)
            {
                if (!containerBrep.IsPointInside(point, tolerance, true)) // true includes "on-surface"
                {
                    return false; // If any point is outside, the test Brep is not completely inside
                }
            }

            return true; // All points are inside or on the surface
        }
        private bool IsFullyInsideOrOnSurface(Brep testBrep, Brep containerBrep)
        {
            // Get vertices of the test Brep
            List<Point3d> testPoints = new List<Point3d>();
            foreach (BrepVertex vertex in testBrep.Vertices)
            {
                testPoints.Add(vertex.Location);
            }

            // Add centroid for additional accuracy
            Point3d centroid = AreaMassProperties.Compute(testBrep)?.Centroid ?? Point3d.Unset;
            if (centroid.IsValid) testPoints.Add(centroid);

            // Optionally, add surface-sampled points for additional accuracy
            // Uncomment the following lines to sample surface points:
            // testPoints.AddRange(SampleSurfacePoints(testBrep, 10));

            // Check if all points are inside or on the surface of the container Brep
            foreach (Point3d point in testPoints)
            {
                if (!IsPointInsideOrOnSurface(point, containerBrep))
                {
                    return false; // If any point is not inside or on the surface, the Brep is excluded
                }
            }

            return true; // All points are inside or on the surface
        }

        private bool IsPointInsideOrOnSurface(Point3d point, Brep containerBrep)
        {
            // Check if the point is inside the Brep or on its surface within tolerance
            bool isInside = containerBrep.IsPointInside(point, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, true);

            if (isInside)
                return true;

            // If not inside, check if the point is close to the Brep surface
            Point3d closestPoint = containerBrep.ClosestPoint(point);
            double distance = point.DistanceTo(closestPoint);

            return distance <= Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {             get
            {
                return Properties.Resources.voxelizeGeo_icon;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("8fd0b8df-6cae-4448-bd6f-5ee776abc51f");
    }
}