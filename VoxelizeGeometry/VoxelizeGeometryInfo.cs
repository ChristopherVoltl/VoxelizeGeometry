using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace VoxelizeGeometry
{
    public class VoxelizeGeometryInfo : GH_AssemblyInfo
    {
        public override string Name => "VoxelizeGeometry";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("ef5d6457-011e-4d9c-b10d-67c0c178a840");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}