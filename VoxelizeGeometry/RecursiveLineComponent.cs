﻿using Grasshopper;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using HelixToolkit.SharpDX.Core.Model.Scene;
using HelixToolkit.SharpDX.Core.Utilities;
using System.Numerics;
using System.Linq;

namespace SpatialGeneration
{
    public class RecursiveLineComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>

        public RecursiveLineComponent()
          : base("Divide-and-Conquer", "DAC",
            "Take some lines and build an army",
            "FGAM", "Divide-and-Conquer")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "C", "Input Curves", GH_ParamAccess.list);
            pManager.AddNumberParameter("Maximum Edge Length", "MEL", "Max Length of Edge", GH_ParamAccess.item, 6.0);
            pManager.AddNumberParameter("Voxel Size", "VS", "Size of Voxel", GH_ParamAccess.item, .0);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("All Curves", "C", "Generated Split Curves", GH_ParamAccess.list);
            pManager.AddBoxParameter("Voxel", "V", "Generated Approximate Voxel", GH_ParamAccess.list);

        }

        // Method to split a curve if it exceeds max length
        Curve[] SplitCurveIfTooLong(Curve curve, double maxLength, out Point3d splitPt)
        {
            var splitCurves = new List<Curve>();
            splitPt = Point3d.Unset;

            if (curve.GetLength() > maxLength)
            {
                double t = (curve.Domain.T0 + curve.Domain.T1) / 2.0;
                splitPt = curve.PointAt(t);
                var splitResult = curve.Split(t);
                if (splitResult != null)
                    splitCurves.AddRange(splitResult);
                else
                    splitCurves.Add(curve);
            }
            else
                splitCurves.Add(curve);

            return splitCurves.ToArray();
        }

        // Method to find the closest point
        Point3d ClosestPoint(Point3d pt, List<Point3d> pts)
        {
            Point3d closest = Point3d.Unset;
            double minDist = double.MaxValue;

            foreach (Point3d p in pts)
            {
                double dist = pt.DistanceTo(p);
                if (dist < minDist && dist > 0)
                {
                    minDist = dist;
                    closest = p;
                }
            }
            return closest;
        }
        // Main logic to process curves
        List<Curve> ProcessCurves(List<Curve> curves, double maxLength)
        {
            var splitCurves = new List<Curve>();
            var finalCurves = new List<Curve>();
            var splitPoints = new List<Point3d>();

            // Initial splitting
            foreach (var curve in curves)
            {
                if (curve.GetLength() > maxLength)
                {
                    Point3d splitPt;
                    var split = SplitCurveIfTooLong(curve, maxLength, out splitPt);
                    splitCurves.AddRange(split);
                    if (splitPt != Point3d.Unset)
                        splitPoints.Add(splitPt);
                }
                else
                {
                    finalCurves.Add(curve);
                }
            }

            // Iterative splitting and connecting
            int iteration = 0;
            while (splitCurves.Exists(c => c.GetLength() > maxLength) && iteration < 100)
            {
                var nextGenCurves = new List<Curve>();
                var newSplitPoints = new List<Point3d>();

                foreach (var c in splitCurves)
                {
                    if (c.GetLength() > maxLength)
                    {
                        Point3d splitPt;
                        var split = SplitCurveIfTooLong(c, maxLength, out splitPt);
                        nextGenCurves.AddRange(split);
                        if (splitPt != Point3d.Unset)
                            newSplitPoints.Add(splitPt);
                    }
                    else
                    {
                        finalCurves.Add(c);
                    }
                }

                // Pairing split points correctly (unique pairs only)
                var pointsToPair = new List<Point3d>(newSplitPoints);
                var pairedPoints = new HashSet<Point3d>();

                foreach (var pt in newSplitPoints)
                {
                    if (pairedPoints.Contains(pt)) continue;

                    var potentialMatches = pointsToPair.Where(p => p != pt && !pairedPoints.Contains(p)).ToList();
                    var closest = ClosestPoint(pt, potentialMatches);

                    if (closest != Point3d.Unset)
                    {
                        nextGenCurves.Add(new Line(pt, closest).ToNurbsCurve());
                        pairedPoints.Add(pt);
                        pairedPoints.Add(closest);
                    }
                }

                // Prepare for next iteration
                splitCurves = nextGenCurves;
                iteration++;
            }

            finalCurves.AddRange(splitCurves);
            return finalCurves;
        }

        public static float ToSingle(double value)
        {
            return (float)value;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Input initialization
            List<Curve> crvs = new List<Curve>();
            double max_edge_length = 6.0;
            // Create a voxel grid
            double voxelSize = 1.0;

            // Check and retrieve input
            if (!DA.GetDataList(0, crvs))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curves failed to load. Please provide valid line-like curves.");
                return;
            }

            if (!DA.GetData(1, ref max_edge_length)) return;
            if (!DA.GetData(2, ref voxelSize)) return;

            var result = ProcessCurves(crvs, max_edge_length);


            List<Point3d> pts = new List<Point3d>();

            // Sample points densely from curves
            foreach (Curve crv in result)
            {
                double length = crv.GetLength();
                int divisions = 10; //Math.Max(5, (int)(length / 0.1)); // Adjust the sampling density as needed
                Point3d[] sampledPts;
                crv.DivideByCount(divisions, true, out sampledPts);
                pts.AddRange(sampledPts);
            }
        
            
            /*voxelSize = ToSingle(voxelSize);

            var voxel = new List<Box>();
            var grid = new HashSet<Vector3>();
            foreach (var pt in pts)
            {
                
                Rhino.Geometry.Plane plane = new Rhino.Geometry.Plane(pt, Vector3d.ZAxis);
                var xInterval = new Interval(-voxelSize / 2, voxelSize / 2);
                var yInterval = new Interval(-voxelSize / 2, voxelSize / 2);
                var zInterval = new Interval(-voxelSize / 2, voxelSize / 2);
                var box = new Box(plane, xInterval, yInterval, zInterval);
                voxel.Add(box);
              
            }*/
            // Compute convex hull
            //Mesh convexHullMesh = Mesh.CreateConvexHull3D(pts, out hullFacets, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);


            DA.SetDataList(0, result);
            //DA.SetDataList(1, voxel);



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
        public override Guid ComponentGuid => new Guid("3dfee11d-2069-4e1e-b451-56c4d7c6a669");
    }
}