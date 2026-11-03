// Port of: src/ModelingData/TKG3d/GTests/GeomAPI_Interpolate_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomAPI_Interpolate GTest (in TKG3d's GTests in V8_0_1).</summary>
public class GeomAPI_Interpolate_Test
{
    // Test BUC60902: GeomAPI_Interpolate tangent computation
    [Fact]
    public void BUC60902_TangentPreservation()
    {
        // Create 5 points along a sine wave
        NCollection_HArray1<Point3D> aPnts = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPnts.SetValue(i, new Point3D((i - 1) * 1.57, Math.Sin((i - 1) * 1.57), 0.0));
        }

        // First interpolation without tangents
        GeomAPI_Interpolate anInterpolater = new(aPnts, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpolater.Perform();
        Assert.True(anInterpolater.IsDone(), "First interpolation should succeed");

        Geom_BSplineCurve aCur = anInterpolater.Curve();
        Assert.NotNull(aCur);

        aCur.D1(aCur.FirstParameter(), out _, out Vector3D aFirstTang);
        aCur.D1(aCur.LastParameter(), out _, out Vector3D aLastTang);

        // Second interpolation with the same tangents explicitly provided
        GeomAPI_Interpolate anInterpolater1 = new(aPnts, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpolater1.Load(aFirstTang, aLastTang, false);
        anInterpolater1.Perform();
        Assert.True(anInterpolater1.IsDone(), "Second interpolation should succeed");

        aCur = anInterpolater1.Curve();
        Assert.NotNull(aCur);

        aCur.D1(aCur.FirstParameter(), out _, out Vector3D aFirstTang1);
        aCur.D1(aCur.LastParameter(), out _, out Vector3D aLastTang1);

        Assert.True(aFirstTang.IsEqual(aFirstTang1, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Angular()),
                    "First tangent should be preserved after recomputation");
        Assert.True(aLastTang.IsEqual(aLastTang1, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Angular()),
                    "Last tangent should be preserved after recomputation");
    }
}
