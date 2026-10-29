// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_OffsetCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingData.Tests.GeomBndLibSurfaceTestUtils;

namespace OcctNet.ModelingData.Tests;

public class GeomBndLib_OffsetCurveTest
{
    private static readonly double Conf = Precision.Confusion();

    /// <summary>Helper: verify box contains all sampled points of the offset curve.</summary>
    private static void ExpectContainsCurve(in Bnd_Box theBox, Geom_OffsetCurve theCurve, double theU1, double theU2, int theNbSamples)
    {
        double aStep = (theU2 - theU1) / theNbSamples;
        theBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        for (int i = 0; i <= theNbSamples; ++i)
        {
            double aU = theU1 + i * aStep;
            gp_Pnt aPnt = theCurve.Value(aU);
            string aWhere = $"Point outside box at U={aU}";
            ExpectGE(aPnt.X(), aXmin - Conf, aWhere);
            ExpectLE(aPnt.X(), aXmax + Conf, aWhere);
            ExpectGE(aPnt.Y(), aYmin - Conf, aWhere);
            ExpectLE(aPnt.Y(), aYmax + Conf, aWhere);
            ExpectGE(aPnt.Z(), aZmin - Conf, aWhere);
            ExpectLE(aPnt.Z(), aZmax + Conf, aWhere);
        }
    }

    /// <summary>GeomBndLib_Curve.Add and BndLib_Add3dCurve.Add (both whole range), then the GTest's checks.</summary>
    private static void CheckAdd(Geom_OffsetCurve theOff, double theU1, double theU2, int theNbSamples)
    {
        GeomAdaptor_Curve anAdaptor = new(theOff);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Curve(theOff).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsCurve(aNewBox, theOff, theU1, theU2, theNbSamples);
    }

    /// <summary>As <see cref="CheckAdd"/>, on the arc [theU1, theU2].</summary>
    private static void CheckAddArc(Geom_OffsetCurve theOff, double theU1, double theU2, int theNbSamples)
    {
        GeomAdaptor_Curve anAdaptor = new(theOff);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Curve(theOff).Add(theU1, theU2, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_Add3dCurve.Add(anAdaptor, theU1, theU2, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsCurve(aNewBox, theOff, theU1, theU2, theNbSamples);
    }

    /// <summary>As <see cref="CheckAdd"/>, with AddOptimal on both sides.</summary>
    private static void CheckAddOptimal(Geom_OffsetCurve theOff, double theU1, double theU2, int theNbSamples)
    {
        GeomAdaptor_Curve anAdaptor = new(theOff);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Curve(theOff).AddOptimal(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_Add3dCurve.AddOptimal(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsCurve(aNewBox, theOff, theU1, theU2, theNbSamples);
    }

    // =========================================================================
    // Circle offset in Z direction (displacement in XY plane)
    // =========================================================================

    [Fact]
    public void CircleOffsetZ_Full_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 5.0);
        CheckAdd(new Geom_OffsetCurve(aCircle, 2.0, gp.DZ()), 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void CircleOffsetZ_Arc_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 5.0);
        CheckAddArc(new Geom_OffsetCurve(aCircle, 3.0, gp.DZ()), 0.0, M_PI / 2.0, 50);
    }

    // =========================================================================
    // Circle offset in X direction (displacement in YZ plane)
    // =========================================================================

    [Fact]
    public void CircleOffsetX_Full_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 7.0);
        CheckAdd(new Geom_OffsetCurve(aCircle, 2.0, gp.DX()), 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void CircleOffsetX_Arc_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 7.0);
        CheckAddArc(new Geom_OffsetCurve(aCircle, 4.0, gp.DX()), M_PI / 4.0, 3.0 * M_PI / 4.0, 50);
    }

    // =========================================================================
    // Circle offset in Y direction - degenerate case
    // Offset direction Y is in the circle plane (XOY), so T(t) x V can be zero.
    // Old BndLib throws "GeomAdaptor_Curve::EvalD0: evaluation failed".
    // Test only containment for our implementation.
    // =========================================================================

    [Fact]
    public void CircleOffsetY_Full_Containment()
    {
        Geom_Circle aCircle = new(gp.XOY(), 3.0);
        Geom_OffsetCurve anOff = new(aCircle, 1.5, gp.DY());

        Bnd_Box aNewBox = new();
        new GeomBndLib_Curve(anOff).Add(Conf, ref aNewBox);

        Assert.False(aNewBox.IsVoid());
    }

    // =========================================================================
    // Circle offset in diagonal direction
    // =========================================================================

    [Fact]
    public void CircleOffsetDiagonal_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 5.0);
        CheckAddOptimal(new Geom_OffsetCurve(aCircle, 2.0, new gp_Dir(1.0, 1.0, 1.0)), 0.0, 2.0 * M_PI, 100);
    }

    // =========================================================================
    // Negative offset
    // =========================================================================

    [Fact]
    public void CircleNegativeOffset_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 10.0);
        CheckAdd(new Geom_OffsetCurve(aCircle, -3.0, gp.DZ()), 0.0, 2.0 * M_PI, 100);
    }

    // =========================================================================
    // Ellipse offset
    // =========================================================================

    [Fact]
    public void EllipseOffsetZ_Full_CompareWithBndLib()
    {
        Geom_Ellipse anEllipse = new(gp.XOY(), 8.0, 3.0);
        CheckAdd(new Geom_OffsetCurve(anEllipse, 2.0, gp.DZ()), 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void EllipseOffsetX_Arc_CompareWithBndLib()
    {
        Geom_Ellipse anEllipse = new(gp.XOY(), 8.0, 3.0);
        CheckAddArc(new Geom_OffsetCurve(anEllipse, 1.5, gp.DX()), 0.0, M_PI, 50);
    }

    // =========================================================================
    // Line offset
    // =========================================================================

    [Fact]
    public void LineOffset_CompareWithBndLib()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        CheckAddArc(new Geom_OffsetCurve(aLine, 3.0, gp.DZ()), 0.0, 10.0, 50);
    }

    // =========================================================================
    // BSpline offset
    // =========================================================================

    [Fact]
    public void BSplineOffset_CompareWithBndLib()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(3.0, 5.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt(7.0, -2.0, 2.0));
        aPoles.SetValue(4, new gp_Pnt(10.0, 0.0, 0.0));

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);

        NCollection_Array1<int> aMults = new(1, 3);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 1);
        aMults.SetValue(3, 3);

        Geom_BSplineCurve aBSpl = new(aPoles, aKnots, aMults, 2);
        CheckAddOptimal(new Geom_OffsetCurve(aBSpl, 1.0, gp.DZ()), 0.0, 1.0, 50);
    }

    // =========================================================================
    // Large offset
    // =========================================================================

    [Fact]
    public void LargeOffset_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(gp.XOY(), 2.0);
        CheckAdd(new Geom_OffsetCurve(aCircle, 20.0, gp.DZ()), 0.0, 2.0 * M_PI, 100);
    }

    // =========================================================================
    // Non-planar circle with offset
    // =========================================================================

    [Fact]
    public void TiltedCircle_CompareWithBndLib()
    {
        // Circle in a tilted plane
        gp_Ax2 anAx2 = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(1.0, 1.0, 1.0));
        Geom_Circle aCircle = new(anAx2, 4.0);
        CheckAdd(new Geom_OffsetCurve(aCircle, 1.5, new gp_Dir(1.0, 1.0, 1.0)), 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void InfiniteRange_OpennessPreserved()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        Geom_OffsetCurve anOff = new(aLine, 3.0, gp.DZ());
        GeomAdaptor_Curve anAdaptor = new(anOff);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Curve(anOff).Add(-Precision.Infinite(), 10.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_Add3dCurve.Add(anAdaptor, -Precision.Infinite(), 10.0, Conf, ref anOldBox);

        Assert.False(aNewBox.IsVoid());
        Assert.True(aNewBox.IsOpenXmin());
        Assert.False(aNewBox.IsOpenXmax());
        Assert.False(aNewBox.IsOpenYmin());
        Assert.False(aNewBox.IsOpenYmax());
        Assert.False(aNewBox.IsOpenZmin());
        Assert.False(aNewBox.IsOpenZmax());
        // BndLib now delegates to GeomBndLib, so it also correctly computes the open X min.
        Assert.True(anOldBox.IsOpenXmin());
    }
}
