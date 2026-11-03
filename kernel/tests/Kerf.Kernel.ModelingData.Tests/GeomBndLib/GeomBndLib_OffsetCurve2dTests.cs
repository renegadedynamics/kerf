// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_OffsetCurve2d_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomBndLib_OffsetCurve2dTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static void ExpectNoLarger2d(in Box2d theNew, in Box2d theOld, double theTol)
        => GeomBndLibCurve2dTestUtils.ExpectNoLarger2d(theNew, theOld, theTol);

    /// <summary>Helper: verify 2D box contains all sampled points of the offset curve.</summary>
    private static void ExpectContainsCurve2d(in Box2d theBox, OffsetCurve2D theCurve, double theU1, double theU2, int theNbSamples)
    {
        double aStep = (theU2 - theU1) / theNbSamples;
        theBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
        for (int i = 0; i <= theNbSamples; ++i)
        {
            double aU = theU1 + i * aStep;
            Point2D aPnt = theCurve.Value(aU);
            string aWhere = $"Point outside box at U={aU}";
            GeomBndLibCurve2dTestUtils.ExpectGE(aPnt.X(), aXmin - Conf, aWhere);
            GeomBndLibCurve2dTestUtils.ExpectLE(aPnt.X(), aXmax + Conf, aWhere);
            GeomBndLibCurve2dTestUtils.ExpectGE(aPnt.Y(), aYmin - Conf, aWhere);
            GeomBndLibCurve2dTestUtils.ExpectLE(aPnt.Y(), aYmax + Conf, aWhere);
        }
    }

    private static CircleCurve2D Circle(double theX, double theY, double theR)
        => new(new Axis2D(new Point2D(theX, theY), new Direction2D(1.0, 0.0)), theR);

    private static EllipseCurve2D Ellipse83()
        => new(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0), new Direction2D(0.0, 1.0)), 8.0, 3.0);

    // =========================================================================
    // Circle offset (positive)
    // =========================================================================

    [Fact]
    public void Circle_PositiveOffset_Full_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(0.0, 0.0, 5.0), 2.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void Circle_PositiveOffset_Arc_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(0.0, 0.0, 5.0), 2.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(0.0, M_PI / 2.0, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, 0.0, M_PI / 2.0, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, M_PI / 2.0, 50);
    }

    // =========================================================================
    // Circle offset (negative)
    // =========================================================================

    [Fact]
    public void Circle_NegativeOffset_Full_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(0.0, 0.0, 10.0), -3.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void Circle_NegativeOffset_Arc_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(0.0, 0.0, 10.0), -3.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(M_PI / 4.0, M_PI, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, M_PI / 4.0, M_PI, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, M_PI / 4.0, M_PI, 50);
    }

    // =========================================================================
    // Ellipse offset
    // =========================================================================

    [Fact]
    public void Ellipse_Full_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Ellipse83(), 1.5);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 2.0 * M_PI, 100);
    }

    [Fact]
    public void Ellipse_Arc_OptimalCompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Ellipse83(), 1.5);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).AddOptimal(0.0, M_PI, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.AddOptimal(anOff, 0.0, M_PI, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, M_PI, 50);
    }

    // =========================================================================
    // Line offset
    // =========================================================================

    [Fact]
    public void Line_CompareWithBndLib()
    {
        LineCurve2D aLine = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        OffsetCurve2D anOff = new(aLine, 3.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(0.0, 10.0, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 10.0, 50);
    }

    // =========================================================================
    // BSpline offset
    // =========================================================================

    [Fact]
    public void BSpline_OptimalCompareWithBndLib()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(3.0, 5.0));
        aPoles.SetValue(3, new Point2D(7.0, -2.0));
        aPoles.SetValue(4, new Point2D(10.0, 0.0));

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);

        NCollection_Array1<int> aMults = new(1, 3);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 1);
        aMults.SetValue(3, 3);

        BSplineCurve2D aBSpl = new(aPoles, aKnots, aMults, 2);
        OffsetCurve2D anOff = new(aBSpl, 1.0);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).AddOptimal(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.AddOptimal(anOff, 0.0, 1.0, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 1.0, 50);
    }

    // =========================================================================
    // Large offset
    // =========================================================================

    [Fact]
    public void LargeOffset_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(0.0, 0.0, 2.0), 20.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 2.0 * M_PI, 100);
    }

    // =========================================================================
    // Off-center circle with offset
    // =========================================================================

    [Fact]
    public void OffCenterCircle_CompareWithBndLib()
    {
        OffsetCurve2D anOff = new(Circle(5.0, 3.0, 4.0), 1.0);
        CurveAdaptor2D anAdaptor = new(anOff);

        Box2d aNewBox = new();
        new Curve2dBounds(anOff).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger2d(aNewBox, anOldBox, Conf);
        ExpectContainsCurve2d(aNewBox, anOff, 0.0, 2.0 * M_PI, 100);
    }
}
