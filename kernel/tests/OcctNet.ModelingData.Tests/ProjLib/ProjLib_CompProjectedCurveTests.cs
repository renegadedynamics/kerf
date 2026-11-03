// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// ProjLib_CompProjectedCurve, ProjLib_PrjResolve and ProjLib_PrjFunc: known projections on planes, cylinders,
/// spheres and B-spline surfaces, isoparametric and punctual parts, several parts, the approximations of
/// <c>Perform</c>, the adaptor's queries, and the OCCT issue it reproduces.
/// </summary>
public class ProjLib_CompProjectedCurveTests
{
    private const double Tol = 1.0e-7;

    private static SurfaceAdaptor UnitSquare(double theZ = 0.0)
    {
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        return new SurfaceAdaptor(aPlane, 0.0, 1.0, 0.0, 1.0);
    }

    private static CurveAdaptor Circle(in Point3D theCenter, double theRadius, double theFirst, double theLast)
    {
        CircleCurve3D aCircle = new(new Frame3D(theCenter, new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), theRadius);
        return new CurveAdaptor(aCircle, theFirst, theLast);
    }

    // The 2D point is the foot of the perpendicular from C(t): S(P) - C(t) is normal to the surface.
    private static void AssertOrthogonal(SurfaceAdaptorBase theS, CurveAdaptorBase theC, double theT, in Point2D theP)
    {
        theS.D1(theP.X(), theP.Y(), out Point3D aS, out Vector3D aDU, out Vector3D aDV);
        Vector3D aV = new(theC.Value(theT), aS);
        Assert.Equal(0.0, aV * aDU / aDU.Magnitude(), 1.0e-6);
        Assert.Equal(0.0, aV * aDV / aDV.Magnitude(), 1.0e-6);
    }

    [Fact]
    public void Plane_Circle_ProjectsToCircle()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        SurfaceAdaptor aS = new(aPlane, -5, 5, -5, 5);
        CurveAdaptor aC = Circle(new Point3D(0, 0, 3), 2.0, 0.0, 2 * M_PI);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);
        aProj.Bounds(1, out double aF, out double aL);
        Assert.Equal(0.0, aF, 1.0e-12);
        Assert.Equal(2 * M_PI, aL, 1.0e-12);
        Assert.False(aProj.IsSinglePnt(1, out _));
        Assert.False(aProj.IsUIso(1, out _));
        Assert.False(aProj.IsVIso(1, out _));
        Assert.Equal(3.0, aProj.MaxDistance(1), 1.0e-9);
        for (int i = 0; i <= 20; i++)
        {
            double t = 2 * M_PI * i / 20;
            Point2D aP = aProj.Value(t);
            Assert.Equal(2 * Math.Cos(t), aP.X(), 1.0e-6);
            Assert.Equal(2 * Math.Sin(t), aP.Y(), 1.0e-6);
            aProj.D1(t, out _, out Vector2D aD1);
            Assert.Equal(-2 * Math.Sin(t), aD1.X(), 1.0e-6);
            Assert.Equal(2 * Math.Cos(t), aD1.Y(), 1.0e-6);
            aProj.D2(t, out _, out _, out Vector2D aD2);
            Assert.Equal(-2 * Math.Cos(t), aD2.X(), 1.0e-6);
            Assert.Equal(-2 * Math.Sin(t), aD2.Y(), 1.0e-6);
        }
        Assert.Equal(CurveKind.OtherCurve, aProj.GetType());
    }

    [Fact]
    public void Cylinder_Generatrix_IsUIso()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        SurfaceAdaptor aS = new(aCyl, 0, 2 * M_PI, -10, 10);
        LineCurve3D aLine = new(new Point3D(0, 3, 0), new Direction3D(0, 0, 1));
        CurveAdaptor aC = new(aLine, 0.0, 5.0);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);
        Assert.True(aProj.IsUIso(1, out double aU));
        Assert.Equal(M_PI / 2, aU, 1.0e-6);
        Assert.False(aProj.IsVIso(1, out _));
        Assert.Equal(2.0, aProj.MaxDistance(1), 1.0e-9);

        aProj.Perform();
        Assert.False(aProj.ResultIsPoint(1));
        Curve2D? aC2d = aProj.GetResult2dC(1);
        Assert.NotNull(aC2d);
        Assert.True(aC2d.IsFrozen);
        Assert.Same(aC2d, aProj.GetResult2dC(1));
        Assert.IsType<TrimmedCurve2D>(aC2d);
        // The line (UIso, 0) along Y, trimmed to the V range.
        Assert.Equal(0.0, aC2d.FirstParameter(), 1.0e-6);
        Assert.Equal(5.0, aC2d.LastParameter(), 1.0e-6);
        Point2D aP = aC2d.Value(2.5);
        Assert.Equal(M_PI / 2, aP.X(), 1.0e-6);
        Assert.Equal(2.5, aP.Y(), 1.0e-6);
    }

    [Fact]
    public void Cylinder_Circle_IsVIso_And3dCurve()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        SurfaceAdaptor aS = new(aCyl, 0, 2 * M_PI, -10, 10);
        CurveAdaptor aC = Circle(new Point3D(0, 0, 2), 3.0, 0.5, 2.5);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);
        Assert.True(aProj.IsVIso(1, out double aV));
        Assert.Equal(2.0, aV, 1.0e-6);
        Assert.False(aProj.IsUIso(1, out _));
        for (int i = 0; i <= 10; i++)
        {
            double t = 0.5 + 2.0 * i / 10;
            Point2D aP = aProj.Value(t);
            Assert.Equal(t, aP.X(), 1.0e-6);
            Assert.Equal(2.0, aP.Y(), 1.0e-6);
        }

        aProj.SetProj3d(true);
        aProj.Perform();
        Curve2D? aC2d = aProj.GetResult2dC(1);
        Assert.NotNull(aC2d);
        Assert.Equal(1.0, aC2d.Value(1.0).X(), 1.0e-6); // the V-iso line (0, V) along X
        Curve3D? aC3d = aProj.GetResult3dC(1);
        Assert.NotNull(aC3d);
        Assert.True(aC3d.IsFrozen);
        Assert.Same(aC3d, aProj.GetResult3dC(1));
        for (int i = 0; i <= 10; i++)
        {
            double t = aC3d.FirstParameter() + (aC3d.LastParameter() - aC3d.FirstParameter()) * i / 10;
            Point3D aP = aC3d.Value(t);
            Assert.Equal(1.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1.0e-5);
            Assert.Equal(2.0, aP.Z(), 1.0e-5);
        }
    }

    [Fact]
    public void Sphere_Parallel_IsVIso()
    {
        SphericalSurface aSph = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        SurfaceAdaptor aS = new(aSph);
        CurveAdaptor aC = Circle(new Point3D(0, 0, 1), 2.0, 0.5, 2.5);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);
        Assert.True(aProj.IsVIso(1, out double aV));
        Assert.Equal(Math.Atan2(1.0, 2.0), aV, 1.0e-6);
        for (int i = 0; i <= 10; i++)
        {
            double t = 0.5 + 2.0 * i / 10;
            Point2D aP = aProj.Value(t);
            Assert.Equal(t, aP.X(), 1.0e-6);
            AssertOrthogonal(aS, aC, t, aP);
        }
    }

    [Fact]
    public void BSplineSurface_BSplineCurve_PointsAreFeetAndApproximationFits()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double z = ((i == 2 || i == 3) && (j == 2 || j == 3)) ? 0.5 : 0.0;
                aPoles[i, j] = new Point3D((i - 1) / 3.0 * 4, (j - 1) / 3.0 * 4, z);
            }
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        BSplineSurface aSurf = new(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
        SurfaceAdaptor aS = new(aSurf);

        NCollection_Array1<Point3D> aCPoles = new(1, 4);
        aCPoles[1] = new Point3D(0.5, 0.5, 1.0);
        aCPoles[2] = new Point3D(1.5, 3.0, 1.5);
        aCPoles[3] = new Point3D(2.5, 1.0, 1.2);
        aCPoles[4] = new Point3D(3.5, 3.5, 1.0);
        NCollection_Array1<double> aCKnots = new(1, 2);
        aCKnots[1] = 0;
        aCKnots[2] = 1;
        NCollection_Array1<int> aCMults = new(1, 2);
        aCMults[1] = 4;
        aCMults[2] = 4;
        BSplineCurve3D aCurve = new(aCPoles, aCKnots, aCMults, 3);
        CurveAdaptor aC = new(aCurve);

        CompProjectedCurve aProj = new(1.0e-6, aS, aC);
        aProj.GetTolerance(out double aTolU, out double aTolV);
        Assert.True(aTolU >= global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.True(aTolV >= global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(1, aProj.CurveCount);
        for (int i = 0; i <= 20; i++)
        {
            double t = i / 20.0;
            AssertOrthogonal(aS, aC, t, aProj.Value(t));
        }

        aProj.Perform();
        Assert.True(aProj.GetResult2dUApproxError(1) < 1.0e-4);
        Assert.True(aProj.GetResult2dVApproxError(1) < 1.0e-4);
        Curve2D? aC2d = aProj.GetResult2dC(1);
        Assert.NotNull(aC2d);
        Assert.IsType<BSplineCurve2D>(aC2d);
        for (int i = 0; i <= 20; i++)
        {
            double t = i / 20.0;
            Assert.True(aC2d.Value(t).Distance(aProj.Value(t)) < 1.0e-4);
        }
    }

    [Fact]
    public void Plane_NormalLine_IsSinglePoint()
    {
        SurfaceAdaptor aS = UnitSquare();
        LineCurve3D aLine = new(new Point3D(0.3, 0.4, 0), new Direction3D(0, 0, 1));
        CurveAdaptor aC = new(aLine, 1.0, 2.0);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);
        Assert.True(aProj.IsSinglePnt(1, out Point2D aP));
        Assert.Equal(0.3, aP.X(), 1.0e-9);
        Assert.Equal(0.4, aP.Y(), 1.0e-9);
        Assert.False(aProj.IsUIso(1, out _));
        Assert.False(aProj.IsVIso(1, out _));

        aProj.SetProj3d(true);
        aProj.Perform();
        Assert.True(aProj.ResultIsPoint(1));
        Assert.Equal(0.3, aProj.GetResult2dP(1).X(), 1.0e-9);
        Assert.Equal(0.4, aProj.GetResult3dP(1).Y(), 1.0e-9);
        Assert.Null(aProj.GetResult2dC(1));
        Assert.Null(aProj.GetResult3dC(1));
    }

    [Fact]
    public void Square_Circle_FourParts_IntervalsAndMaxDist()
    {
        SurfaceAdaptor aS = UnitSquare();
        CurveAdaptor aC = Circle(new Point3D(0.5, 0.5, 1), 0.6, 0.0, 2 * M_PI);

        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(4, aProj.CurveCount);
        double aPrevLast = -1;
        for (int i = 1; i <= 4; i++)
        {
            aProj.Bounds(i, out double aF, out double aL);
            Assert.True(aF > aPrevLast);
            Assert.True(aL > aF);
            aPrevLast = aL;
            Assert.False(aProj.IsUIso(i, out _));
            Assert.False(aProj.IsVIso(i, out _));
            // Each part ends on the square's boundary.
            foreach (double t in new[] { aF, aL })
            {
                Point2D aP = aProj.Value(t);
                double aDist = Min(Min(Math.Abs(aP.X()), Math.Abs(aP.X() - 1)), Min(Math.Abs(aP.Y()), Math.Abs(aP.Y() - 1)));
                Assert.True(aDist < 1.0e-5, $"part {i}, t = {t}: {aP}");
            }
        }
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aProj.Value(0.0));

        // The intervals are the parts' bounds (a circle has one interval and a plane none).
        int aNb = aProj.NbIntervals(Continuity.C1);
        Assert.Equal(7, aNb);
        NCollection_Array1<double> aT = new(1, aNb + 1);
        aProj.Intervals(aT, Continuity.C1);
        for (int i = 1; i <= 4; i++)
        {
            aProj.Bounds(i, out double aF, out double aL);
            Assert.Equal(aF, aT[2 * i - 1]);
            Assert.Equal(aL, aT[2 * i]);
        }

        // MaxDist: the circle is 1 above the plane.
        Assert.Equal(0, new CompProjectedCurve(aS, aC, Tol, Tol, 0.5).CurveCount);
        Assert.Equal(4, new CompProjectedCurve(aS, aC, Tol, Tol, 2.0).CurveCount);

        // 3D approximations lie on the projected circle.
        aProj.SetProj2d(false);
        aProj.SetProj3d(true);
        aProj.Perform();
        for (int i = 1; i <= 4; i++)
        {
            Curve3D? aC3d = aProj.GetResult3dC(i);
            Assert.NotNull(aC3d);
            Assert.True(aProj.GetResult3dApproxError(i) < 1.0e-5);
            for (int k = 0; k <= 10; k++)
            {
                Point3D aP = aC3d.Value(aC3d.FirstParameter() + (aC3d.LastParameter() - aC3d.FirstParameter()) * k / 10);
                Assert.Equal(0.0, aP.Z(), 1.0e-6);
                Assert.Equal(0.6, aP.Distance(new Point3D(0.5, 0.5, 0)), 1.0e-5);
            }
        }
    }

    [Fact]
    public void Perform_IsoThenFreePart_ReusesIsoLine_OcctBug()
    {
        // A polyline above the unit square: along V = 0.5 out of the square, then back in diagonally.
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0.1, 0.5, 1);
        aPoles[2] = new Point3D(1.5, 0.5, 1);
        aPoles[3] = new Point3D(0.6, 0.9, 1);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 2;
        aMults[2] = 1;
        aMults[3] = 2;
        CurveAdaptor aC = new(new BSplineCurve3D(aPoles, aKnots, aMults, 1));

        CompProjectedCurve aProj = new(UnitSquare(), aC, Tol, Tol);
        Assert.Equal(2, aProj.CurveCount);
        Assert.True(aProj.IsVIso(1, out double aV));
        Assert.Equal(0.5, aV, 1.0e-9);
        Assert.False(aProj.IsVIso(2, out _));
        Assert.False(aProj.IsUIso(2, out _));

        aProj.Perform();
        // OCCT: the iso part clears approx2d for the following parts, which get the iso line again.
        Assert.Same(aProj.GetResult2dC(1), aProj.GetResult2dC(2));
    }

    [Fact]
    public void TrimAndShallowCopy_ShareTheParts()
    {
        SurfaceAdaptor aS = UnitSquare();
        CurveAdaptor aC = Circle(new Point3D(0.5, 0.5, 1), 0.3, 0.0, 2 * M_PI);
        CompProjectedCurve aProj = new(aS, aC, Tol, Tol);
        Assert.Equal(1, aProj.CurveCount);

        CompProjectedCurve aCopy = Assert.IsType<CompProjectedCurve>(aProj.ShallowCopy());
        Assert.Equal(1, aCopy.CurveCount);
        Assert.NotSame(aS, aCopy.GetSurface());
        Assert.NotSame(aC, aCopy.GetCurve());
        Assert.Equal(aProj.Value(1.0).X(), aCopy.Value(1.0).X(), 1.0e-12);

        CompProjectedCurve aTrim = Assert.IsType<CompProjectedCurve>(aProj.Trim(1.0, 2.0, 1.0e-9));
        Assert.Equal(1.0, aTrim.FirstParameter());
        Assert.Equal(2.0, aTrim.LastParameter());
        Assert.Same(aS, aTrim.GetSurface());
        Assert.Equal(aProj.Value(1.5).Y(), aTrim.Value(1.5).Y(), 1.0e-12);
        Assert.Equal(0.0, aProj.FirstParameter());
    }

    [Fact]
    public void GetSequence_IsACopy()
    {
        CompProjectedCurve aProj = new(UnitSquare(), Circle(new Point3D(0.5, 0.5, 1), 0.3, 0.0, 2 * M_PI), Tol, Tol);
        NCollection_HSequence<NCollection_HSequence<Point3D>> aSeq = aProj.GetSequence();
        Assert.Equal(1, aSeq.Length());
        aProj.Bounds(1, out double aF, out double aL);
        Assert.Equal(aF, aSeq.Value(1).First().X());
        Assert.Equal(aL, aSeq.Value(1).Last().X());
        aSeq.Value(1).Clear();
        aSeq.Clear();
        aProj.Bounds(1, out double aF2, out double aL2);
        Assert.Equal(aF, aF2);
        Assert.Equal(aL, aL2);
    }

    [Fact]
    public void Errors()
    {
        CompProjectedCurve aProj = new(UnitSquare(), Circle(new Point3D(0.5, 0.5, 1), 0.3, 0.0, 2 * M_PI), Tol, Tol);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aProj.Bounds(0, out _, out _));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aProj.IsUIso(2, out _));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => aProj.DN(1.0, 0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aProj.DN(1.0, 3));
        aProj.D1(1.0, out _, out Vector2D aV);
        Assert.Equal(aV, aProj.DN(1.0, 1));
        aProj.Perform();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.TypeMismatchException>(() => aProj.GetResult2dP(1));

        CompProjectedCurve anEmpty = new();
        Assert.Equal(0, anEmpty.CurveCount);
        Assert.Null(anEmpty.GetSurface());
        Assert.Null(anEmpty.GetCurve());
        Assert.Equal(0, Assert.IsType<CompProjectedCurve>(anEmpty.ShallowCopy()).CurveCount);
        anEmpty.Perform(); // nothing to do
    }

    [Fact]
    public void Setters_KeepValidValues()
    {
        CompProjectedCurve aProj = new();
        aProj.SetMaxDegree(0);
        aProj.SetMaxSeg(0);
        aProj.SetProj2d(false);
        aProj.SetProj3d(true);
        Assert.False(aProj.GetProj2d());
        Assert.True(aProj.GetProj3d());
    }

    [Fact]
    public void PrjResolve_FootOnPlane()
    {
        SurfaceAdaptor aS = UnitSquare();
        CurveAdaptor aC = Circle(new Point3D(0.5, 0.5, 1), 0.3, 0.0, 2 * M_PI);
        PrjResolve aSolver = new(aC, aS, 1);
        aSolver.Perform(0.0, 0.5, 0.5, new Point2D(Tol, Tol), new Point2D(0, 0), new Point2D(1, 1), 1.0e-10);
        Assert.True(aSolver.IsDone);
        Assert.Equal(0.8, aSolver.Solution().X(), 1.0e-9);
        Assert.Equal(0.5, aSolver.Solution().Y(), 1.0e-9);

        // Outside the bounds: not done.
        aSolver.Perform(0.0, 0.5, 0.5, new Point2D(Tol, Tol), new Point2D(0, 0), new Point2D(0.6, 1), 1.0e-10);
        Assert.False(aSolver.IsDone);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Solution());

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new PrjResolve(aC, aS, 0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new PrjResolve(aC, aS, 4));
    }

    [Fact]
    public void PrjFunc_ZeroAtTheFoot()
    {
        SurfaceAdaptor aS = UnitSquare();
        CurveAdaptor aC = Circle(new Point3D(0.5, 0.5, 1), 0.3, 0.0, 2 * M_PI);
        // Fix 2: u fixed at 0.8, the variables (t, v); the foot of C(0) is (0.8, 0.5).
        PrjFunc aF = new(aC, 0.8, aS, 2);
        Assert.Equal(2, aF.NbVariables());
        Assert.Equal(2, aF.NbEquations());
        math_Vector aX = new(1, 2);
        aX[1] = 0.0;
        aX[2] = 0.5;
        math_Vector aVal = new(1, 2);
        Matrix aD = new(1, 2, 1, 2);
        Assert.True(aF.Values(aX, aVal, aD));
        Assert.Equal(0.0, aVal[1], 1.0e-15);
        Assert.Equal(0.0, aVal[2], 1.0e-15);
        Assert.Equal(new Point2D(0.0, 0.5), aF.Solution());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new PrjFunc(aC, 0.0, aS, 0));
    }
}
