// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dAPI_PointsToBSpline_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's Geom2dAPI_PointsToBSpline GTest.</summary>
public class Geom2dAPI_PointsToBSpline_Test
{
    [Fact]
    public void DegenerateXRangeFallsBackToUniformParameters()
    {
        NCollection_Array1<double> aYValues = new(1, 3);
        aYValues[1] = 0.0;
        aYValues[2] = 1.0;
        aYValues[3] = 0.0;

        Geom2dAPI_PointsToBSpline anApprox = new();
        anApprox.Init(aYValues, -2.0, 1.0, 3, 5, C2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(anApprox.IsDone());

        Geom2d_BSplineCurve aCurve = anApprox.Curve();
        Assert.NotNull(aCurve);
    }

    [Fact]
    public void DegenerateExplicitParametersResetDoneState()
    {
        NCollection_Array1<Point2D> aPoints = new(1, 3);
        aPoints[1] = new Point2D(0.0, 0.0);
        aPoints[2] = new Point2D(1.0, 1.0);
        aPoints[3] = new Point2D(2.0, 0.0);

        NCollection_Array1<double> aParameters = new(1, 3);
        aParameters[1] = 5.0;
        aParameters[2] = 5.0;
        aParameters[3] = 5.0;

        Geom2dAPI_PointsToBSpline anApprox = new(aPoints, 3, 5, C2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(anApprox.IsDone());

        anApprox.Init(aPoints, aParameters, 3, 5, C2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(anApprox.IsDone());
    }
}
