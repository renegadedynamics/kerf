// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomFill_NSections_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomFill_NSections GTest (suite GeomFill_NSectionsTest).</summary>
public class GeomFill_NSections_Test
{
    // Builds a simple degree-1 BSpline line segment from (0,0,0) to (1,0,0).
    private static Geom_BSplineCurve makeSimpleLinearCurve()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 2);
        aPoles[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[2] = new gp_Pnt(1.0, 0.0, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;

        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
    }

    // Tests that GeomFill_NSections with a single-curve sequence does not throw.
    // The resulting BSplineSurface may be null for a degenerate single-section case,
    // but no exception should be raised (OCC27875).
    [Fact]
    public void OCC27875_SingleCurveDoesNotThrow()
    {
        Geom_BSplineCurve aCurve = makeSimpleLinearCurve();
        Assert.NotNull(aCurve);

        // The NCollection_IncAllocator argument of OCCT's sequence is dropped (translation rules 9).
        NCollection_Sequence<Geom_Curve> aNC = new();
        aNC.Append(aCurve);

        // Must not throw even though result may be degenerate
        var anException = Record.Exception(() => new GeomFill_NSections(aNC));
        Assert.Null(anException);
    }
}
