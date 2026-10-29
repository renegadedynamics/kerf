// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// A minimal adaptor over a Geom curve, for tests until GeomAdaptor_Curve (unit 2A) is ported: evaluation through the
/// curve, the elementary types and the B-spline and Bezier queries. Continuity and intervals are the curve's
/// whole range (one interval).
/// </summary>
internal sealed class TestCurveAdaptor(Geom_Curve theCurve, double theFirst, double theLast) : Adaptor3d_Curve
{
    public TestCurveAdaptor(Geom_Curve theCurve) : this(theCurve, theCurve.FirstParameter(), theCurve.LastParameter()) { }

    public Geom_Curve Curve() => theCurve;

    public override Adaptor3d_Curve ShallowCopy() => new TestCurveAdaptor(theCurve, theFirst, theLast);
    public override double FirstParameter() => theFirst;
    public override double LastParameter() => theLast;
    public override GeomAbs_Shape Continuity() => theCurve.Continuity();
    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = theFirst;
        T[T.Lower() + 1] = theLast;
    }

    public override Adaptor3d_Curve Trim(double First, double Last, double Tol) => new TestCurveAdaptor(theCurve, First, Last);
    public override bool IsClosed() => theCurve.IsClosed();
    public override bool IsPeriodic() => theCurve.IsPeriodic();
    public override double Period() => theCurve.Period();
    public override double Resolution(double R3d) => Precision.Parametric(R3d);

    public override GeomAbs_CurveType GetType() => theCurve switch
    {
        Geom_Line => GeomAbs_Line,
        Geom_Circle => GeomAbs_Circle,
        Geom_Ellipse => GeomAbs_Ellipse,
        Geom_Hyperbola => GeomAbs_Hyperbola,
        Geom_Parabola => GeomAbs_Parabola,
        Geom_BezierCurve => GeomAbs_BezierCurve,
        Geom_BSplineCurve => GeomAbs_BSplineCurve,
        Geom_OffsetCurve => GeomAbs_OffsetCurve,
        _ => GeomAbs_OtherCurve,
    };

    public override gp_Lin Line() => ((Geom_Line)theCurve).Lin();
    public override gp_Circ Circle() => ((Geom_Circle)theCurve).Circ();
    public override gp_Elips Ellipse() => ((Geom_Ellipse)theCurve).Elips();
    public override gp_Hypr Hyperbola() => ((Geom_Hyperbola)theCurve).Hypr();
    public override gp_Parab Parabola() => ((Geom_Parabola)theCurve).Parab();
    public override int Degree() => theCurve is Geom_BezierCurve B ? B.Degree() : ((Geom_BSplineCurve)theCurve).Degree();
    public override bool IsRational() => theCurve is Geom_BezierCurve B ? B.IsRational() : theCurve is Geom_BSplineCurve S && S.IsRational();
    public override int NbPoles() => theCurve is Geom_BezierCurve B ? B.NbPoles() : ((Geom_BSplineCurve)theCurve).NbPoles();
    public override int NbKnots() => ((Geom_BSplineCurve)theCurve).NbKnots();
    public override Geom_BezierCurve Bezier() => (Geom_BezierCurve)theCurve;
    public override Geom_BSplineCurve BSpline() => (Geom_BSplineCurve)theCurve;
    public override gp_Pnt EvalD0(double theU) => theCurve.EvalD0(theU);
    public override Geom_Curve.ResD1 EvalD1(double theU) => theCurve.EvalD1(theU);
    public override Geom_Curve.ResD2 EvalD2(double theU) => theCurve.EvalD2(theU);
    public override Geom_Curve.ResD3 EvalD3(double theU) => theCurve.EvalD3(theU);
    public override gp_Vec EvalDN(double theU, int theN) => theCurve.EvalDN(theU, theN);
}

/// <summary>
/// A minimal adaptor over a Geom surface between given bounds, for tests until GeomAdaptor_Surface (unit 2A) is
/// ported: evaluation through the surface, the type queries and the elementary, swept, Bezier and B-spline data.
/// Continuity and intervals are the whole range (one interval in each direction).
/// </summary>
internal sealed class TestSurfaceAdaptor : Adaptor3d_Surface
{
    private readonly Geom_Surface mySurf;
    private readonly double myU1, myU2, myV1, myV2;

    public TestSurfaceAdaptor(Geom_Surface theSurf, double theU1, double theU2, double theV1, double theV2)
    {
        mySurf = theSurf;
        myU1 = theU1;
        myU2 = theU2;
        myV1 = theV1;
        myV2 = theV2;
    }

    public TestSurfaceAdaptor(Geom_Surface theSurf)
    {
        mySurf = theSurf;
        theSurf.Bounds(out myU1, out myU2, out myV1, out myV2);
    }

    public Geom_Surface Surface() => mySurf;

    public override Adaptor3d_Surface ShallowCopy() => new TestSurfaceAdaptor(mySurf, myU1, myU2, myV1, myV2);
    public override double FirstUParameter() => myU1;
    public override double LastUParameter() => myU2;
    public override double FirstVParameter() => myV1;
    public override double LastVParameter() => myV2;
    public override GeomAbs_Shape UContinuity() => mySurf.Continuity();
    public override GeomAbs_Shape VContinuity() => mySurf.Continuity();
    public override int NbUIntervals(GeomAbs_Shape S) => 1;
    public override int NbVIntervals(GeomAbs_Shape S) => 1;

    public override void UIntervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = myU1;
        T[T.Lower() + 1] = myU2;
    }

    public override void VIntervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = myV1;
        T[T.Lower() + 1] = myV2;
    }

    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, First, Last, myV1, myV2);
    public override Adaptor3d_Surface VTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, myU1, myU2, First, Last);
    public override bool IsUClosed() => mySurf.IsUClosed();
    public override bool IsVClosed() => mySurf.IsVClosed();
    public override bool IsUPeriodic() => mySurf.IsUPeriodic();
    public override double UPeriod() => mySurf.UPeriod();
    public override bool IsVPeriodic() => mySurf.IsVPeriodic();
    public override double VPeriod() => mySurf.VPeriod();
    public override double UResolution(double R3d) => Precision.Parametric(R3d);
    public override double VResolution(double R3d) => Precision.Parametric(R3d);

    public override GeomAbs_SurfaceType GetType() => mySurf switch
    {
        Geom_Plane => GeomAbs_Plane,
        Geom_CylindricalSurface => GeomAbs_Cylinder,
        Geom_ConicalSurface => GeomAbs_Cone,
        Geom_SphericalSurface => GeomAbs_Sphere,
        Geom_ToroidalSurface => GeomAbs_Torus,
        Geom_BezierSurface => GeomAbs_BezierSurface,
        Geom_BSplineSurface => GeomAbs_BSplineSurface,
        Geom_SurfaceOfRevolution => GeomAbs_SurfaceOfRevolution,
        Geom_SurfaceOfLinearExtrusion => GeomAbs_SurfaceOfExtrusion,
        _ => GeomAbs_OtherSurface,
    };

    public override gp_Pln Plane() => ((Geom_Plane)mySurf).Pln();
    public override gp_Cylinder Cylinder() => ((Geom_CylindricalSurface)mySurf).Cylinder();
    public override gp_Cone Cone() => ((Geom_ConicalSurface)mySurf).Cone();
    public override gp_Sphere Sphere() => ((Geom_SphericalSurface)mySurf).Sphere();
    public override gp_Torus Torus() => ((Geom_ToroidalSurface)mySurf).Torus();
    public override int UDegree() => mySurf is Geom_BezierSurface B ? B.UDegree() : ((Geom_BSplineSurface)mySurf).UDegree();
    public override int VDegree() => mySurf is Geom_BezierSurface B ? B.VDegree() : ((Geom_BSplineSurface)mySurf).VDegree();
    public override int NbUPoles() => mySurf is Geom_BezierSurface B ? B.NbUPoles() : ((Geom_BSplineSurface)mySurf).NbUPoles();
    public override int NbVPoles() => mySurf is Geom_BezierSurface B ? B.NbVPoles() : ((Geom_BSplineSurface)mySurf).NbVPoles();
    public override int NbUKnots() => ((Geom_BSplineSurface)mySurf).NbUKnots();
    public override int NbVKnots() => ((Geom_BSplineSurface)mySurf).NbVKnots();
    public override bool IsURational() => mySurf is Geom_BezierSurface B ? B.IsURational() : ((Geom_BSplineSurface)mySurf).IsURational();
    public override bool IsVRational() => mySurf is Geom_BezierSurface B ? B.IsVRational() : ((Geom_BSplineSurface)mySurf).IsVRational();
    public override Geom_BezierSurface Bezier() => (Geom_BezierSurface)mySurf;
    public override Geom_BSplineSurface BSpline() => (Geom_BSplineSurface)mySurf;
    public override gp_Ax1 AxeOfRevolution() => ((Geom_SurfaceOfRevolution)mySurf).Axis();
    public override gp_Dir Direction() => ((Geom_SweptSurface)mySurf).Direction();
    public override Adaptor3d_Curve BasisCurve() => new TestCurveAdaptor(((Geom_SweptSurface)mySurf).BasisCurve());
    public override gp_Pnt EvalD0(double theU, double theV) => mySurf.EvalD0(theU, theV);
    public override Geom_Surface.ResD1 EvalD1(double theU, double theV) => mySurf.EvalD1(theU, theV);
    public override Geom_Surface.ResD2 EvalD2(double theU, double theV) => mySurf.EvalD2(theU, theV);
    public override Geom_Surface.ResD3 EvalD3(double theU, double theV) => mySurf.EvalD3(theU, theV);
    public override gp_Vec EvalDN(double theU, double theV, int theNu, int theNv) => mySurf.EvalDN(theU, theV, theNu, theNv);
}
