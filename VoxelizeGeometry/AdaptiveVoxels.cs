using Grasshopper.Kernel;
using Rhino.DocObjects;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Rhino.UI;
using Grasshopper.Kernel.Types;

namespace SpatialGeneration
{
    public class AdaptiveVoxelsComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>

        public AdaptiveVoxelsComponent()
          : base("AdaptiveVoxels", "AV",
            "Take some lines and build an army",
            "FGAM", "Divide-and-Conquer")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "Input Brep", GH_ParamAccess.item);
            pManager.AddGenericParameter("Points", "P", "List of Points for Adaptive Meshing", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Resolution", "R", "Base resolution (larger = coarser)", GH_ParamAccess.item, 6);
            pManager.AddNumberParameter("Threshold", "T", "Density threshold for refinement", GH_ParamAccess.item, 0.2);
            pManager.AddIntegerParameter("MaxDepth", "D", "Maximum octree depth", GH_ParamAccess.item, 4);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBoxParameter("AdaptiveMesh", "AM", "Generated Split Curves", GH_ParamAccess.list);
            pManager.AddCurveParameter("AdaptiveMeshCurves", "AMC", "Generated ", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Density-based voxel colors", GH_ParamAccess.list);

        }

        // Method to split a curve if it exceeds max length

        public class GradedVoxel
        {
            public Box Box;
            public double Density;
            public int Depth;
            public bool IsLeaf;
            public List<GradedVoxel> Children = new List<GradedVoxel>();

            public GradedVoxel(Box box, double density, int depth)
            {
                Box = box;
                Density = density;
                Depth = depth;
                IsLeaf = true;
            }

            public void Subdivide(double densityThreshold, int maxDepth)
            {
                if (Depth >= maxDepth)
                    return;

                if (Depth == 0 || Density >= densityThreshold)
                {
                    IsLeaf = false;
                    var subBoxes = SubdivideBox(Box);

                    foreach (var sub in subBoxes)
                    {
                        var center = sub.Center;
                        double newDensity = GradedVoxelGrid.NearestDensity(center);
                        var child = new GradedVoxel(sub, newDensity, Depth + 1);
                        child.Subdivide(densityThreshold, maxDepth);
                        Children.Add(child);
                    }
                }
            }

            private List<Box> SubdivideBox(Box box)
            {
                var boxes = new List<Box>();

                Plane plane = box.Plane;
                Vector3d xDir = plane.XAxis;
                Vector3d yDir = plane.YAxis;
                Vector3d zDir = plane.ZAxis;
                xDir.Unitize(); yDir.Unitize(); zDir.Unitize();

                double halfX = box.X.Length / 2;
                double halfY = box.Y.Length / 2;
                double halfZ = box.Z.Length / 2;

                Point3d basePt = box.Plane.Origin;

                for (int xi = 0; xi < 2; xi++)
                    for (int yi = 0; yi < 2; yi++)
                        for (int zi = 0; zi < 2; zi++)
                        {
                            Point3d corner = basePt + xi * halfX * xDir + yi * halfY * yDir + zi * halfZ * zDir;
                            Box subBox = new Box(
                                new Plane(corner, xDir, yDir),
                                new Interval(0, halfX),
                                new Interval(0, halfY),
                                new Interval(0, halfZ)
                            );
                            boxes.Add(subBox);
                        }
                return boxes;
            }
        }


        public static class GradedVoxelGrid
        {
            public static Dictionary<Point3d, double> DensityField = new Dictionary<Point3d, double>();
            private static List<GradedVoxel> leaves = new List<GradedVoxel>();

            public static double NearestDensity(Point3d pt)
            {
                double minDist = double.MaxValue;
                double density = 0;
                foreach (var kvp in DensityField)
                {
                    double dist = pt.DistanceTo(kvp.Key);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        density = kvp.Value;
                    }
                }
                return density;
            }

            public static List<(Box box, Color color)> BuildGradedVoxels(Brep brep, int resolution, double threshold, int maxDepth)
            {
                leaves.Clear();
                BoundingBox bounds = brep.GetBoundingBox(true);

                double sizeX = bounds.Max.X - bounds.Min.X;
                double sizeY = bounds.Max.Y - bounds.Min.Y;
                double sizeZ = bounds.Max.Z - bounds.Min.Z;

                Box baseBox = new Box(
                    new Plane(bounds.Min, Vector3d.XAxis, Vector3d.YAxis),
                    new Interval(0, sizeX),
                    new Interval(0, sizeY),
                    new Interval(0, sizeZ)
                );

                GradedVoxel root = new GradedVoxel(baseBox, NearestDensity(bounds.Center), 0);
                root.Subdivide(threshold, maxDepth);

                List<(Box, Color)> coloredBoxes = new List<(Box, Color)>();
                CollectLeaves(root);
                foreach (var v in leaves)
                {
                    if (brep.IsPointInside(v.Box.Center, 0.01, true))
                    {
                        int r = (int)(v.Density * 255);
                        Color color = Color.FromArgb(r, 0, 255 - r); // Red to Cyan gradient
                        coloredBoxes.Add((v.Box, color));
                    }
                }
                return coloredBoxes;
            }

            private static void CollectLeaves(GradedVoxel voxel)
            {
                if (voxel.IsLeaf)
                    leaves.Add(voxel);
                else
                    foreach (var child in voxel.Children)
                        CollectLeaves(child);
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<IGH_GeometricGoo> geoInputs = new List<IGH_GeometricGoo>();
            Brep brep = null;
            int resolution = 6;
            double threshold = 0.2;
            int maxDepth = 4;

            if (!DA.GetDataList(1, geoInputs)) return;
            if (!DA.GetData(0, ref brep)) return;
            DA.GetData(2, ref resolution);
            DA.GetData(3, ref threshold);
            DA.GetData(4, ref maxDepth);

            List<Point3d> points = new List<Point3d>();
            List<Color> colors = new List<Color>();

            foreach (var ggoo in geoInputs)
            {
                if (ggoo is IGH_GeometricGoo geo && geo.IsReferencedGeometry)
                {
                    Point3d pt;
                    if (geo.CastTo(out pt))
                    {
                        Guid id = geo.ReferenceID;
                        RhinoObject rhObj = RhinoDoc.ActiveDoc.Objects.Find(id);
                        if (rhObj != null)
                        {
                            points.Add(pt);
                            Color color;
                            if (rhObj.Attributes.ColorSource == ObjectColorSource.ColorFromObject)
                            {
                                color = rhObj.Attributes.ObjectColor;
                            }
                            else
                            {
                                int layerIndex = rhObj.Attributes.LayerIndex;
                                Layer layer = RhinoDoc.ActiveDoc.Layers[layerIndex];
                                color = layer.Color;
                            }
                            colors.Add(color);
                        }
                    }
                }
            }

            if (points.Count != colors.Count || points.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input must reference valid Rhino document points with colors.");
                return;
            }

            GradedVoxelGrid.DensityField.Clear();
            for (int i = 0; i < points.Count; i++)
            {
                double density = colors[i].R / 255.0;
                GradedVoxelGrid.DensityField[points[i]] = density;
            }

            var coloredBoxes = GradedVoxelGrid.BuildGradedVoxels(brep, resolution, threshold, maxDepth);
            var outputBoxes = new List<Box>();
            var outputColors = new List<Color>();

            foreach (var (box, color) in coloredBoxes)
            {
                outputBoxes.Add(box);
                outputColors.Add(color);
            }

            DA.SetDataList(0, outputBoxes);
            DA.SetDataList(2, outputColors);
        }



    /// <summary>
    /// Provides an Icon for every component that will be visible in the User Interface.
    /// Icons need to be 24x24 pixels.
    /// You can add image files to your project resources and access them like this:
    /// return Resources.IconForThisComponent;
    /// </summary>
    protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.voxelizeGeo_icon;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("3a0c4762-918d-421c-baee-42339e89437f");
    }
}