// Port of: src/ModelingData/TKGeomBase/GTests/Extrema_ExtPC_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Extrema_ExtPC_Test
{
    // Test for bug OCC24945
    // Extrema_ExtPElC::Perform does not consider angular tolerance when calculates angle between two
    // vectors
    [Fact]
    public void Bug24945_CylinderParameterNormalization()
    {
        // Point to project
        gp_Pnt aP3D = new(-1725.97, 843.257, -4.22741e-013);

        // Circle for projection
        gp_Ax2 aAxis = new(new gp_Pnt(0, 843.257, 0), gp.DY().Reversed(), gp.DX());
        Geom_Circle aCircle = new(aAxis, 1725.9708621929999);
        GeomAdaptor_Curve aC3D = new(aCircle);

        // Project point onto circle
        Extrema_ExtPC aExtPC = new(aP3D, aC3D);
        Assert.True(aExtPC.IsDone());
        Assert.True(aExtPC.NbExt() > 0);

        gp_Pnt aProj = aExtPC.Point(1).Value();

        // Check projected point coordinates
        Assert.Equal(-1725.97, aProj.X(), 1.0e-2);
        Assert.Equal(843.257, aProj.Y(), 1.0e-2);
        Assert.Equal(0.0, aProj.Z(), 1.0e-10);

        // Compute parameters on cylinder
        gp_Ax2 aCylAxis = new(new gp_Pnt(0, 2103.87, 0), gp.DY().Reversed(), gp.DX().Reversed());
        gp_Cylinder aCylinder = new(new gp_Ax3(aCylAxis), 1890.0);

        ElSLib.Parameters(aCylinder, aProj, out double aU, out double aV);

        // Check parameters
        // U should be normalized to [0, 2*PI) - the test previously expected 6.2832 (2*PI)
        // but the improved normalization now consistently returns 0.0 for the seam
        Assert.Equal(0.0, aU, 1.0e-4);
        Assert.Equal(1260.613, aV, 1.0e-2);
    }
}
