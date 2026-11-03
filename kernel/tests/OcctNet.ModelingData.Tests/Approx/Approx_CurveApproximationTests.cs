// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of Approx_Curve2d, Approx_Curve3d, Approx_CurveOnSurface and Approx_SameParameter (OCCT has no
/// GTests of them): known answers on conics, planes and cylinders.
/// </summary>
public class Approx_CurveApproximationTests
{
    private static Point3D P(double theX, double theY, double theZ = 0.0) => new(theX, theY, theZ);

    private static Geom_CylindricalSurface Cylinder(double theR) => new(new CoordinateSystem(P(0, 0, 0), new Direction3D(0, 0, 1)), theR);

    // A 2D B-spline (degree 2) from (theU1, theV) to (theU2, theV) whose U grows non-linearly with the parameter
    // on [theT1, theT2].
    private static Geom2d_BSplineCurve WarpedLine(double theU1, double theU2, double theV, double theT1, double theT2)
    {
        double aMid = theU1 + 0.8 * (theU2 - theU1);
        NCollection_Array1<Point2D> aPoles = NCollection_Array1<Point2D>.From([
            new Point2D(theU1, theV), new Point2D(aMid, theV), new Point2D(theU2, theV)]);
        return new Geom2d_BSplineCurve(aPoles, NCollection_Array1<double>.From([theT1, theT2]),
                                       NCollection_Array1<int>.From([3, 3]), 2);
    }

    [Fact]
    public void Curve3d_ApproximatesAnEllipseArc()
    {
        Geom_Ellipse anEllipse = new(new Frame3D(P(1, 2, 3), new Direction3D(0, 0, 1)), 5.0, 2.0);
        GeomAdaptor_Curve anAdaptor = new(anEllipse, 0.3, 4.0);
        Approx_Curve3d anApprox = new(anAdaptor, 1e-6, C2, 50, 12);
        Assert.True(anApprox.IsDone());
        Assert.True(anApprox.HasResult());
        Assert.True(anApprox.MaxError() < 1e-6);
        Geom_BSplineCurve aCurve = anApprox.Curve()!;
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, anApprox.Curve());
        Assert.Equal(0.3, aCurve.FirstParameter(), 1e-14);
        Assert.Equal(4.0, aCurve.LastParameter(), 1e-14);
        for (int i = 0; i <= 40; i++)
        {
            double t = 0.3 + 3.7 * i / 40.0;
            Assert.True(aCurve.Value(t).Distance(anEllipse.Value(t)) < 1e-6);
        }
        Assert.Contains("NbSegments", anApprox.ToString());
    }

    [Fact]
    public void Curve2d_ApproximatesACircleArc_WithSeparateTolerances()
    {
        Geom2d_Circle aCircle = new(new Axis2D(new Point2D(1, -1), new Direction2D(1, 0)), 3.0);
        Geom2dAdaptor_Curve anAdaptor = new(aCircle);
        Approx_Curve2d anApprox = new(anAdaptor, 0.5, 2.5, 1e-7, 1e-5, C1, 10, 20);
        Assert.True(anApprox.IsDone());
        Assert.True(anApprox.MaxError2dU() < 1e-7);
        Assert.True(anApprox.MaxError2dV() < 1e-5);
        Geom2d_BSplineCurve aCurve = anApprox.Curve()!;
        Assert.True(aCurve.IsFrozen);
        for (int i = 0; i <= 20; i++)
        {
            double t = 0.5 + 2.0 * i / 20.0;
            Point2D aP = aCurve.Value(t), aQ = aCircle.Value(t);
            Assert.True(Math.Abs(aP.X() - aQ.X()) < 1e-7);
            Assert.True(Math.Abs(aP.Y() - aQ.Y()) < 1e-5);
        }
    }

    [Fact]
    public void CurveOnSurface_HelixOnCylinder_Builds3dAnd2dCurves()
    {
        Geom_CylindricalSurface aCyl = Cylinder(2.0);
        Geom2d_Line aLine = new(new Point2D(0, 0), new Direction2D(1, 0.5));
        Geom2dAdaptor_Curve aC2d = new(aLine);
        GeomAdaptor_Surface aS = new(aCyl);
        Approx_CurveOnSurface anApprox = new(aC2d, aS, 0.0, 6.0, 1e-6);
        anApprox.Perform(30, 12, C2);
        Assert.True(anApprox.IsDone());
        Geom_BSplineCurve aC3d = anApprox.Curve3d()!;
        Geom2d_BSplineCurve aPC = anApprox.Curve2d()!;
        Assert.True(anApprox.MaxError3d() < 1e-6);
        for (int i = 0; i <= 30; i++)
        {
            double t = 6.0 * i / 30.0;
            Point2D anUV = aLine.Value(t);
            Assert.True(aC3d.Value(t).Distance(aCyl.Value(anUV.X(), anUV.Y())) < 1e-6);
            Assert.True(aPC.Value(t).Distance(anUV) < 1e-6);
        }

        anApprox.Perform(30, 12, C2, false, true);
        Assert.True(anApprox.HasResult());
        Assert.NotNull(anApprox.Curve2d());
        Assert.Equal(0.0, anApprox.MaxError3d());

        Assert.Throws<ConstructionErrorException>(() => anApprox.Perform(30, 12, C2, true, true));
    }

    [Fact]
    public void CurveOnSurface_IsolineOnCylinder_TakesTheSurfaceIso()
    {
        Geom_CylindricalSurface aCyl = Cylinder(2.0);
        // U = 1 const, V going down from 3 to -1: a reversed generator
        Geom2d_Line aLine = new(new Point2D(1.0, 3.0), new Direction2D(0, -1));
        Approx_CurveOnSurface anApprox = new(new Geom2dAdaptor_Curve(aLine), new GeomAdaptor_Surface(aCyl), 0.0, 4.0, 1e-7);
        anApprox.Perform(10, 5, C1, true);
        Assert.True(anApprox.IsDone());
        Geom_BSplineCurve aC3d = anApprox.Curve3d()!;
        Assert.Equal(1, aC3d.Degree());
        Assert.Equal(0.0, aC3d.FirstParameter(), 1e-14);
        Assert.Equal(4.0, aC3d.LastParameter(), 1e-14);
        Assert.True(aC3d.Value(0.0).Distance(aCyl.Value(1.0, 3.0)) < 1e-12);
        Assert.True(aC3d.Value(4.0).Distance(aCyl.Value(1.0, -1.0)) < 1e-12);
        Assert.True(anApprox.MaxError3d() < 1e-12);
        Assert.Null(anApprox.Curve2d());
    }

    [Fact]
    public void SameParameter_AlreadySameParameter_DoesNothing()
    {
        Geom_CylindricalSurface aCyl = Cylinder(3.0);
        Geom_Circle aCircle = new(new Frame3D(P(0, 0, 1.5), new Direction3D(0, 0, 1)), 3.0);
        Geom_TrimmedCurve aC3d = new(aCircle, 0.5, 2.5);
        Geom2d_Line aPC = new(new Point2D(0, 1.5), new Direction2D(1, 0));
        Approx_SameParameter aSP = new(new GeomAdaptor_Curve(aC3d), new Geom2dAdaptor_Curve(aPC, 0.5, 2.5),
                                       new GeomAdaptor_Surface(aCyl), 1e-7);
        Assert.True(aSP.IsSameParameter());
        Assert.True(aSP.IsDone());
        Assert.Null(aSP.Curve2d());
        Assert.True(aSP.TolReached() <= 1e-6);
    }

    [Fact]
    public void SameParameter_WarpedPCurveOnCylinder_IsReparametrised()
    {
        Geom_CylindricalSurface aCyl = Cylinder(3.0);
        Geom_Circle aCircle = new(new Frame3D(P(0, 0, 1.5), new Direction3D(0, 0, 1)), 3.0);
        Geom_TrimmedCurve aC3d = new(aCircle, 0.5, 2.5);
        Geom2d_BSplineCurve aPC = WarpedLine(0.5, 2.5, 1.5, 0.5, 2.5);
        Approx_SameParameter aSP = new(aC3d, aPC, aCyl, 1e-7);
        Assert.False(aSP.IsSameParameter());
        Assert.True(aSP.IsDone());
        Geom2d_Curve aNew = aSP.Curve2d()!;
        Assert.True(aNew.IsFrozen);
        Assert.True(aSP.TolReached() < 1e-5);
        Assert.NotNull(aSP.CurveOnSurface());
        for (int i = 0; i <= 40; i++)
        {
            double t = 0.5 + 2.0 * i / 40.0;
            Point2D anUV = aNew.Value(t);
            Assert.True(aCyl.Value(anUV.X(), anUV.Y()).Distance(aC3d.Value(t)) <= aSP.TolReached());
        }
        // The caller's pcurve isn't touched
        Assert.False(aPC.IsFrozen);
    }

    [Fact]
    public void SameParameter_RationalCircleOnPlane_IsReparametrised()
    {
        // 3D circle parametrised by angle; the pcurve is the rational B-spline circle (another parametrisation)
        Geom_Plane aPlane = new(new CoordinateSystem(P(0, 0, 0), new Direction3D(0, 0, 1)));
        Geom_Circle aCircle = new(new Frame3D(P(1, 1, 0), new Direction3D(0, 0, 1)), 2.0);
        Geom2d_Circle aCircle2d = new(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 2.0);
        Geom2d_BSplineCurve aPC = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aCircle2d, 0.2, 2.8),
                                                                    RationalC1);
        NCollection_Array1<double> aKnots = new(aPC.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.2, 2.8, aKnots);
        aPC.SetKnots(aKnots);
        Geom_TrimmedCurve aC3d = new(aCircle, 0.2, 2.8);

        Approx_SameParameter aSP = new(aC3d, aPC, aPlane, 1e-7);
        Assert.False(aSP.IsSameParameter());
        Assert.True(aSP.IsDone());
        Geom2d_Curve aNew = aSP.Curve2d()!;
        for (int i = 0; i <= 40; i++)
        {
            double t = 0.2 + 2.6 * i / 40.0;
            Point2D anUV = aNew.Value(t);
            Assert.True(aPlane.Value(anUV.X(), anUV.Y()).Distance(aC3d.Value(t)) <= aSP.TolReached());
        }
        Assert.True(aSP.TolReached() < 1e-5);
    }
    [Fact]
    public void SameParameter_C0PolylinePCurve_UsesTheC1Intervals()
    {
        // A 3D line by length and a degree 1 pcurve along it with three spans of different speeds (C0)
        Geom_Plane aPlane = new(new CoordinateSystem(P(0, 0, 0), new Direction3D(0, 0, 1)));
        Geom_Line aLine = new(P(0, 0, 0), new Direction3D(1, 0, 0));
        NCollection_Array1<Point2D> aPoles = NCollection_Array1<Point2D>.From([
            new Point2D(0, 0), new Point2D(2.5, 0), new Point2D(3.0, 0), new Point2D(6.0, 0)]);
        Geom2d_BSplineCurve aPC = new(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 4.0, 6.0]),
                                      NCollection_Array1<int>.From([2, 1, 1, 2]), 1);
        Assert.Equal(C0, aPC.Continuity());
        Approx_SameParameter aSP = new(new GeomAdaptor_Curve(aLine, 0.0, 6.0), aPC, new GeomAdaptor_Surface(aPlane), 1e-7);
        Assert.False(aSP.IsSameParameter());
        Assert.True(aSP.IsDone());
        Geom2d_Curve aNew = aSP.Curve2d()!;
        for (int i = 0; i <= 60; i++)
        {
            double t = 6.0 * i / 60.0;
            Point2D anUV = aNew.Value(t);
            Assert.True(aPlane.Value(anUV.X(), anUV.Y()).Distance(aLine.Value(t)) <= aSP.TolReached());
        }
    }
}
