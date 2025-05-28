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
using Rhino.Geometry.Intersect;
using Eto.Forms;

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
            pManager.AddNumberParameter("MinSize", "Min", "Minimum voxel size", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("MaxSize", "Max", "Maximum voxel size", GH_ParamAccess.item, 100.0);
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

            public void Subdivide(int maxDepth, double minSize)
            {
                // Always allow at least 1 subdivision at root
                if (Depth >= maxDepth)
                    return;

                if (Box.X.Length <= minSize)
                    return;

                // If density is low, avoid refining unless we're still above min size
                int targetDepth = Math.Max(1, (int)(Density * maxDepth));
                if (Depth >= targetDepth)
                    return;

                IsLeaf = false;
                var subBoxes = SubdivideBox(Box);

                foreach (var sub in subBoxes)
                {
                    var center = sub.Center;
                    double newDensity = GradedVoxelGrid.SampleDensitySmooth(center);

                    var child = new GradedVoxel(sub, newDensity, Depth + 1);
                    child.Subdivide(maxDepth, minSize);
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

            public static List<(Box box, System.Drawing.Color color)> BuildGradedVoxels(
            Brep brep,
            int resolution,
            int maxDepth,
            double minSize,
            double maxSize)
            {
                output.Clear();

                BoundingBox bounds = brep.GetBoundingBox(true);

                var rootBoxes = new List<(Box, double)>();

                // Create initial grid of max-size root boxes
                for (double x = bounds.Min.X; x < bounds.Max.X; x += maxSize)
                    for (double y = bounds.Min.Y; y < bounds.Max.Y; y += maxSize)
                        for (double z = bounds.Min.Z; z < bounds.Max.Z; z += maxSize)
                        {
                            double dx = Math.Min(maxSize, bounds.Max.X - x);
                            double dy = Math.Min(maxSize, bounds.Max.Y - y);
                            double dz = Math.Min(maxSize, bounds.Max.Z - z);

                            var plane = new Plane(new Point3d(x, y, z), Vector3d.XAxis, Vector3d.YAxis);
                            var box = new Box(plane, new Interval(0, dx), new Interval(0, dy), new Interval(0, dz));
                            double density = SampleDensitySmooth(box.Center);

                            rootBoxes.Add((box, density));
                        }

                // Subdivide and collect voxels
                foreach (var (box, density) in rootBoxes)
                {
                    var root = new GradedVoxel(box, density, 0);
                    root.Subdivide(maxDepth, minSize);
                    CollectLeaves(root, brep);
                }

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

                /*for (int i = 0; i < corners.Length; i++)
                {
                    string label = $"Pt {i}";
                    Rhino.RhinoDoc.ActiveDoc.Objects.AddTextDot(label, corners[i]);
                }*/
            }

            public static List<Line> ExtractUniqueTetEdges(List<Mesh> tetMeshes)
            {
                var edgeSet = new HashSet<(Point3d, Point3d)>(new UndirectedEdgeComparer());

                foreach (var mesh in tetMeshes)
                {
                    var verts = mesh.Vertices;
                    if (verts.Count != 4) continue;

                    var v = new Point3d[]
                    {
                        verts[0],
                        verts[1],
                        verts[2],
                        verts[3]
                    };

                    // 6 edges of a tetrahedron
                    var edges = new (Point3d, Point3d)[]
                    {
                        (v[0], v[1]),
                        (v[0], v[2]),
                        (v[0], v[3]),
                        (v[1], v[2]),
                        (v[1], v[3]),
                        (v[2], v[3])
                    };

                    foreach (var (a, b) in edges)
                    {
                        edgeSet.Add((a, b));
                    }
                }

                return edgeSet.Select(e => new Line(e.Item1, e.Item2)).ToList();
            }

            // Undirected point comparer
            class UndirectedEdgeComparer : IEqualityComparer<(Point3d, Point3d)>
            {
                public bool Equals((Point3d, Point3d) e1, (Point3d, Point3d) e2)
                {
                    return (e1.Item1.EpsilonEquals(e2.Item1, 1e-6) && e1.Item2.EpsilonEquals(e2.Item2, 1e-6)) ||
                           (e1.Item1.EpsilonEquals(e2.Item2, 1e-6) && e1.Item2.EpsilonEquals(e2.Item1, 1e-6));
                }

                public int GetHashCode((Point3d, Point3d) edge)
                {
                    // Order-independent hash
                    unchecked
                    {
                        int h1 = edge.Item1.GetHashCode();
                        int h2 = edge.Item2.GetHashCode();
                        return h1 ^ h2;
                    }
                }
            }

            public static List<Line> RemoveNearMidpointDuplicates(List<Line> lines, double threshold = 1e-6)
            {
                var kept = new List<Line>();
                var used = new bool[lines.Count];

                for (int i = 0; i < lines.Count; i++)
                {
                    if (used[i]) continue;

                    var midA = lines[i].PointAt(0.5);
                    bool isDuplicate = false;

                    for (int j = i + 1; j < lines.Count; j++)
                    {
                        if (used[j]) continue;

                        var midB = lines[j].PointAt(0.5);

                        if (midA.DistanceToSquared(midB) < threshold * threshold)
                        {
                            // Optional: check angle similarity
                            Vector3d dirA = lines[i].Direction;
                            Vector3d dirB = lines[j].Direction;

                            if (Vector3d.VectorAngle(dirA, dirB) < 0.1 || Vector3d.VectorAngle(dirA, -dirB) < 0.1)
                            {
                                used[j] = true; // drop the second one
                                isDuplicate = true;
                            }
                        }
                    }

                    if (!isDuplicate)
                        kept.Add(lines[i]);
                }

                return kept;
            }
            public static List<Line> DeleteLongerIntersectingLines(List<Line> curves, double tol)
            {
                var toRemove = new HashSet<int>(); // indexes of curves to remove

                for (int i = 0; i < curves.Count; i++)
                {
                    for (int j = i + 1; j < curves.Count; j++)
                    {
                        if (toRemove.Contains(i) || toRemove.Contains(j)) continue;

                        Curve c1 = curves[i].ToNurbsCurve();
                        Curve c2 = curves[j].ToNurbsCurve();
                        var events = Rhino.Geometry.Intersect.Intersection.CurveCurve(c1, c2, tol, tol);

                        if (events != null && events.Count > 0)
                        {
                            foreach (var ccx in events)
                            {
                                Point3d pt = ccx.PointA;

                                bool onEnd1 = pt.DistanceTo(c1.PointAtStart) < tol || pt.DistanceTo(c1.PointAtEnd) < tol;
                                bool onEnd2 = pt.DistanceTo(c2.PointAtStart) < tol || pt.DistanceTo(c2.PointAtEnd) < tol;

                                // Skip if intersection is only at endpoints
                                if (onEnd1 || onEnd2)
                                    continue;

                                // Mark one curve for removal
                                double len1 = c1.GetLength();
                                double len2 = c2.GetLength();

                                if (len1 > len2)
                                    toRemove.Add(i);
                                else
                                    toRemove.Add(j);

                                break; // Only need to find one non-endpoint intersection
                            }
                        }
                    }
                }

                // Build result list
                var kept = new List<Line>();
                for (int i = 0; i < curves.Count; i++)
                {
                    if (!toRemove.Contains(i))
                        kept.Add(curves[i]);
                }

                return kept;
            }

            private static bool BoundingBoxesIntersect(BoundingBox a, BoundingBox b)
            {
                return (a.Max.X >= b.Min.X && a.Min.X <= b.Max.X) &&
                       (a.Max.Y >= b.Min.Y && a.Min.Y <= b.Max.Y) &&
                       (a.Max.Z >= b.Min.Z && a.Min.Z <= b.Max.Z);
            }

            public static List<Curve> FilterIntersectingCurves(List<Curve> curves, double tolerance)
            {
                int n = curves.Count;
                var toRemove = new HashSet<int>();

                // Precompute bounding boxes
                BoundingBox[] bboxes = new BoundingBox[n];
                for (int i = 0; i < n; i++)
                    bboxes[i] = curves[i].GetBoundingBox(true);

                for (int i = 0; i < n; i++)
                {
                    if (toRemove.Contains(i))
                        continue;

                    Curve c1 = curves[i];

                    for (int j = i + 1; j < n; j++)
                    {
                        if (toRemove.Contains(j))
                            continue;

                        // Bounding box filter
                        if (!BoundingBoxesIntersect(bboxes[i], bboxes[j]))
                            continue;

                        Curve c2 = curves[j];

                        // Use Rhino's intersection API
                        var events = Intersection.CurveCurve(c1, c2, tolerance, tolerance);

                        if (events == null && events.Count == 0)
                            continue;

                        foreach (var x in events)
                        {
                            Point3d pt = x.PointA;

                            // Tolerance-aware endpoint checks
                            bool isEndOnC1 = pt.DistanceTo(c1.PointAtStart) < tolerance || pt.DistanceTo(c1.PointAtEnd) < tolerance;
                            bool isEndOnC2 = pt.DistanceTo(c2.PointAtStart) < tolerance || pt.DistanceTo(c2.PointAtEnd) < tolerance;

                            if (isEndOnC1 && isEndOnC2)
                                continue; // Skip pure endpoint intersection

                            // Decide which one to remove
                            double len1 = c1.GetLength();
                            double len2 = c2.GetLength();

                            if (len1 > len2)
                                toRemove.Add(i);
                            else
                                toRemove.Add(j);

                            break; // Only process one valid intersection per pair
                        }
                    }
                }

                // Return only the kept curves
                var result = new List<Curve>();
                for (int i = 0; i < n; i++)
                {
                    if (!toRemove.Contains(i))
                        result.Add(curves[i]);
                }

                return result;
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

            double minSize = 1.0;
            double maxSize = 100.0;
            



            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetDataList(1, geoInputs)) return;
            DA.GetData(2, ref resolution);
            DA.GetData(3, ref threshold);
            DA.GetData(4, ref maxDepth);
            DA.GetData(5, ref minSize);
            DA.GetData(6, ref maxSize);

            

            List<Point3d> points = new List<Point3d>();
            List<Color> colors = new List<Color>();

            //int maxDepth = (int)Math.Ceiling(Math.Log(maxSize / minSize, 2));


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

            var coloredBoxes = GradedVoxelGrid.BuildGradedVoxels(brep, resolution, maxDepth, minSize, maxSize);
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

            var uniqueEdges = GradedVoxelGrid.ExtractUniqueTetEdges(tetMeshes);
            var cleanEdges = GradedVoxelGrid.RemoveNearMidpointDuplicates(uniqueEdges, 0.01);

            List<Curve> curves = new List<Curve>();

            // Convert lines to curves
            foreach (var line in cleanEdges)
            {
                curves.Add(line.ToNurbsCurve()); // or line.ToCurve() if using newer RhinoCommon
            }
            List<Curve> cleaned = GradedVoxelGrid.FilterIntersectingCurves(curves, 0.01);


            DA.SetDataList(0, outputBoxes);
            DA.SetDataList(1, cleaned);
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