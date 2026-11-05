// Port of: src/ModelingData/TKGeomBase/GTests/Extrema_ExtPC_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Extrema_ExtPC_Test
{
    // Test for bug OCC24945
    // Extrema_ExtPElC::Perform does not consider angular tolerance when calculates angle between two
    // vectors
    [Fact]
    public void Bug24945_CylinderParameterNormalization()
    {
        // Point to project
        Point3D aP3D = new(-1725.97, 843.257, -4.22741e-013);

        // Circle for projection
        Frame3D aAxis = new(new Point3D(0, 843.257, 0), global::Kerf.Engine.Geometry.Constants.DY().Reversed(), global::Kerf.Engine.Geometry.Constants.DX());
        CircleCurve3D aCircle = new(aAxis, 1725.9708621929999);
        CurveAdaptor aC3D = new(aCircle);

        // Project point onto circle
        CurvePointExtrema aExtPC = new(aP3D, aC3D);
        Assert.True(aExtPC.IsDone);
        Assert.True(aExtPC.ExtCount > 0);

        Point3D aProj = aExtPC.Point(1).Value();

        // Check projected point coordinates
        Assert.Equal(-1725.97, aProj.X, 1.0e-2);
        Assert.Equal(843.257, aProj.Y, 1.0e-2);
        Assert.Equal(0.0, aProj.Z, 1.0e-10);

        // Compute parameters on cylinder
        Frame3D aCylAxis = new(new Point3D(0, 2103.87, 0), global::Kerf.Engine.Geometry.Constants.DY().Reversed(), global::Kerf.Engine.Geometry.Constants.DX().Reversed());
        Cylinder3D aCylinder = new(new CoordinateSystem(aCylAxis), 1890.0);

        global::Kerf.Engine.Geometry.ElSLib.Parameters(aCylinder, aProj, out double aU, out double aV);

        // Check parameters
        // U should be normalized to [0, 2*PI) - the test previously expected 6.2832 (2*PI)
        // but the improved normalization now consistently returns 0.0 for the seam
        Assert.Equal(0.0, aU, 1.0e-4);
        Assert.Equal(1260.613, aV, 1.0e-2);
    }
}
