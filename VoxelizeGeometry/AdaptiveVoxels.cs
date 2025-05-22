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
            pManager.AddBoxParameter("AdaptiveVoxel", "AV", "Generated Voxels", GH_ParamAccess.list);
            pManager.AddCurveParameter("AdaptiveMeshCurves", "AMC", "Generated ", GH_ParamAccess.list);
            pManager.AddMeshParameter("AdaptiveMesh", "AM", "Generated Mesh", GH_ParamAccess.list);
            pManager.AddMeshParameter("AdaptiveTetrahedra", "AT", "Generated Tetrahedra", GH_ParamAccess.list);
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

            public void Subdivide(int maxDepth)
            {
                // Prevent all-zero subdivision
                int targetDepth = Math.Max(1, (int)(Density * maxDepth));

                if (Depth >= targetDepth || Depth >= maxDepth)
                    return;

                IsLeaf = false;
                var subBoxes = SubdivideBox(Box);

                foreach (var sub in subBoxes)
                {
                    Point3d center = sub.Center;

                    double newDensity = GradedVoxelGrid.SampleDensitySmooth(center);
                    var child = new GradedVoxel(sub, newDensity, Depth + 1);
                    child.Subdivide(maxDepth);
                    Children.Add(child);
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

                double hx = box.X.Length / 2;
                double hy = box.Y.Length / 2;
                double hz = box.Z.Length / 2;

                Point3d basePt = box.Center - 0.5 * box.X.Length * xDir
                                              - 0.5 * box.Y.Length * yDir
                                              - 0.5 * box.Z.Length * zDir;

                for (int xi = 0; xi < 2; xi++)
                    for (int yi = 0; yi < 2; yi++)
                        for (int zi = 0; zi < 2; zi++)
                        {
                            var corner = basePt + new Vector3d(hx * xi, hy * yi, hz * zi);
                            var subBox = new Box(
                                new Plane(corner, xDir, yDir),
                                new Interval(0, hx),
                                new Interval(0, hy),
                                new Interval(0, hz)
                            );
                            boxes.Add(subBox);
                        }

                return boxes;
            }
        }


        public static class GradedVoxelGrid
        {
            public static Dictionary<Point3d, double> DensityField = new Dictionary<Point3d, double>();

            private static readonly List<(Box box, System.Drawing.Color color)> output = new List<(Box box, System.Drawing.Color color)>();

            public static List<(Box box, System.Drawing.Color color)> BuildGradedVoxels(Brep brep, int resolution, int maxDepth)
            {
                output.Clear();

                BoundingBox bounds = brep.GetBoundingBox(true);
                double size = bounds.Diagonal.Length / resolution;

                // Create root voxel as a box from Brep bounds
                Point3d basePt = bounds.Min;
                var rootBox = new Box(
                    new Plane(basePt, Vector3d.XAxis, Vector3d.YAxis),
                    new Interval(0, bounds.Max.X - basePt.X),
                    new Interval(0, bounds.Max.Y - basePt.Y),
                    new Interval(0, bounds.Max.Z - basePt.Z)
                );

                var root = new GradedVoxel(rootBox, SampleDensitySmooth(rootBox.Center), 0);
                root.Subdivide(maxDepth);

                CollectLeaves(root, brep);
                return output;
            }

            private static void CollectLeaves(GradedVoxel voxel, Brep brep)
            {
                if (voxel.IsLeaf && brep.IsPointInside(voxel.Box.Center, 0.001, false))
                {
                    // Optional: color gradient for preview
                    double density = voxel.Density;
                    int r = (int)(255 * (1.0 - density));
                    int g = (int)(255 * density);
                    int b = (int)(255 * (1.0 - Math.Abs(density - 0.5) * 2));
                    var color = System.Drawing.Color.FromArgb(r, g, b);
                    output.Add((voxel.Box, color));
                }
                else
                {
                    foreach (var child in voxel.Children)
                        CollectLeaves(child, brep);
                }

            }

            public static List<Box> SubdivideBox(Box box)
            {
                var boxes = new List<Box>();

                Plane plane = box.Plane;
                Vector3d xDir = plane.XAxis;
                Vector3d yDir = plane.YAxis;
                Vector3d zDir = plane.ZAxis;
                xDir.Unitize(); yDir.Unitize(); zDir.Unitize();

                double hx = box.X.Length / 2;
                double hy = box.Y.Length / 2;
                double hz = box.Z.Length / 2;

                Point3d basePt = box.Center - 0.5 * box.X.Length * xDir
                                              - 0.5 * box.Y.Length * yDir
                                              - 0.5 * box.Z.Length * zDir;

                for (int xi = 0; xi < 2; xi++)
                    for (int yi = 0; yi < 2; yi++)
                        for (int zi = 0; zi < 2; zi++)
                        {
                            var corner = basePt + new Vector3d(hx * xi, hy * yi, hz * zi);
                            var subBox = new Box(
                                new Plane(corner, xDir, yDir),
                                new Interval(0, hx),
                                new Interval(0, hy),
                                new Interval(0, hz)
                            );
                            boxes.Add(subBox);
                        }

                return boxes;
            }

            public static double NearestDensity(Point3d pt)
            {
                double minDist = double.MaxValue;
                double best = 0;

                foreach (var kvp in DensityField)
                {
                    double d = pt.DistanceTo(kvp.Key);
                    if (d < minDist)
                    {
                        minDist = d;
                        best = kvp.Value;
                    }
                }
                return best;
            }

            public static double SampleDensitySmooth(Point3d pt)
            {
                double radius = 10.0; // influence radius
                double totalWeight = 0;
                double weightedSum = 0;

                foreach (var kvp in DensityField)
                {
                    double dist = pt.DistanceTo(kvp.Key);
                    if (dist < radius && dist > 1e-6)
                    {
                        double w = 1.0 / (dist * dist);
                        weightedSum += kvp.Value * w;
                        totalWeight += w;
                    }
                }

                if (totalWeight > 0)
                    return weightedSum / totalWeight;

                return NearestDensity(pt); // fallback
            }

            private static bool BoxIntersectsBrep(Box box, Brep brep)
            {
                foreach (var corner in box.GetCorners())
                {
                    if (brep.IsPointInside(corner, 0.001, false))
                        return true;
                }
                return false;
            }

            //Convert Box to Mesh
            public static Mesh BoxToMesh(Box box)
            {
                var corners = box.GetCorners();
                if (corners == null || corners.Length != 8)
                    return null;

                var mesh = new Mesh();

                foreach (var pt in corners)
                    mesh.Vertices.Add(pt);
                

                // Define 6 quad faces (use correct winding)
                mesh.Faces.AddFace(0, 1, 2, 3); // bottom
                mesh.Faces.AddFace(4, 5, 6, 7); // top
                mesh.Faces.AddFace(0, 1, 5, 4); // front
                mesh.Faces.AddFace(1, 5, 6, 2); // right
                mesh.Faces.AddFace(3, 2, 6, 7); // back
                mesh.Faces.AddFace(0, 4, 7, 3); // left

                mesh.Normals.ComputeNormals();
                mesh.Compact();
                return mesh;
            }

            //convert box to tetrahedra
            public static List<Mesh> BoxToTetrahedra(Box box)
            {
                var corners = box.GetCorners();
                if (corners == null || corners.Length != 8)
                    return new List<Mesh>();

                var tets = new List<Mesh>();

                // Use your validated ordering:
                // 0: bottom-front-left
                // 1: bottom-front-right
                // 2: bottom-back-left
                // 3: bottom-back-right
                // 4: top-front-left


                // 5-tet split (balanced)
                int[][] tetIndices = new int[][]
                {
                    new int[] { 0, 1, 2, 5 },
                    new int[] { 2, 3, 0, 7 },
                    new int[] { 2, 0, 7, 5 },
                    new int[] { 0, 4, 7, 5 },
                    new int[] { 2, 6, 7, 5 }
                };

                foreach (var tet in tetIndices)
                {
                    var mesh = new Mesh();
                    foreach (int i in tet)
                        mesh.Vertices.Add(corners[i]);

                    mesh.Faces.AddFace(0, 1, 2);
                    mesh.Faces.AddFace(0, 1, 3);
                    mesh.Faces.AddFace(1, 2, 3);
                    mesh.Faces.AddFace(2, 0, 3);
                    mesh.Normals.ComputeNormals();
                    mesh.Compact();
                    tets.Add(mesh);
                }

                return tets;
            }

            //debug the mesh face order
            public static void PreviewBoxCornerIndices(Box box)
            {
                var corners = box.GetCorners();

                if (corners == null || corners.Length != 8)
                    return;

                for (int i = 0; i < corners.Length; i++)
                {
                    string label = $"Pt {i}";
                    Rhino.RhinoDoc.ActiveDoc.Objects.AddTextDot(label, corners[i]);
                }
            }

        }
         
        


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<IGH_GeometricGoo> geoInputs = new List<IGH_GeometricGoo>();
            Brep brep = null;
            int resolution = 6;
            double threshold = 0.2;
            int maxDepth = 4;
            var outputMeshes = new List<Mesh>();
            var tetMeshes = new List<Mesh>();



            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetDataList(1, geoInputs)) return;
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
                double r = colors[i].R;
                double g = colors[i].G;
                double b = colors[i].B;
                double raw = (0.3 * r + 0.59 * g + 0.11 * b) / 255.0;
                double density = Math.Max(0.05, Math.Min(1.0, raw));  // avoid zeros
                GradedVoxelGrid.DensityField[points[i]] = density;
            }

            var coloredBoxes = GradedVoxelGrid.BuildGradedVoxels(brep, resolution, maxDepth);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Generated {coloredBoxes.Count} voxels.");
            var outputBoxes = new List<Box>();
            var outputColors = new List<Color>();

            foreach (var (box, color) in coloredBoxes)
            {
                outputBoxes.Add(box);
                outputColors.Add(color);
                var mesh = GradedVoxelGrid.BoxToMesh(box);
                if (mesh != null)
                    outputMeshes.Add(mesh);

                var tets = GradedVoxelGrid.BoxToTetrahedra(box);
                tetMeshes.AddRange(tets);
            }

            if (coloredBoxes.Count > 0)
            {
                var (firstBox, _) = coloredBoxes[0];
                GradedVoxelGrid.PreviewBoxCornerIndices(firstBox);
            }

            DA.SetDataList(0, outputBoxes);
            DA.SetDataList(2, outputMeshes);
            DA.SetDataList(3, tetMeshes);
            DA.SetDataList(4, outputColors);
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