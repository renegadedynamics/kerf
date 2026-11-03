// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomAPI_PointsToBSpline_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomAPI_PointsToBSpline GTest.</summary>
public class GeomAPI_PointsToBSpline_Test
{
    [Fact]
    public void DegenerateExplicitParametersResetDoneState()
    {
        NCollection_Array1<Point3D> aPoints = new(1, 3);
        aPoints[1] = new Point3D(0.0, 0.0, 0.0);
        aPoints[2] = new Point3D(1.0, 0.5, 0.0);
        aPoints[3] = new Point3D(2.0, 0.0, 0.0);

        NCollection_Array1<double> aParameters = new(1, 3);
        aParameters[1] = 2.0;
        aParameters[2] = 2.0;
        aParameters[3] = 2.0;

        PointsToBSpline anApprox = new(aPoints, 3, 5, Continuity.C2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(anApprox.IsDone);

        Assert.Null(Record.Exception(() =>
            anApprox.Init(aPoints, aParameters, 3, 5, Continuity.C2, global::Kerf.Engine.Foundation.Precision.Confusion())));
        Assert.False(anApprox.IsDone);
    }
}
