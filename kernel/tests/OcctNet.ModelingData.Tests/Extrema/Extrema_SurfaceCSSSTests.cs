// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Unit tests of the curve/surface and surface/surface extrema of unit 4A (Extrema_GlobOptFuncCS, ConicS, CQuadric,
/// Extrema_FuncExtCS, GenLocateExtCS, GenExtCS, Extrema_FuncExtSS, GenLocateExtSS, GenExtSS, ExtSS). OCCT has no
/// GTests for them: results are checked against known answers, finite differences and dense sampling.
/// </summary>
public class Extrema_SurfaceCSSSTests
{
    // A bicubic B-spline surface on [0, 3] x [0, 2], a wavy sheet near z = 0.
    private static BSplineSurface MakeBSplineSurface(double theZ = 0.0)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 6, 1, 5);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 5; j++)
            {
                aPoles[i, j] = new Point3D(i, j, theZ + 0.4 * Math.Sin(i * 1.3 + j * 0.7));
            }
        }
        NCollection_Array1<double> aUKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0, 3.0 });
        NCollection_Array1<int> aUMults = NCollection_Array1<int>.From(new[] { 4, 1, 1, 4 });
        NCollection_Array1<double> aVKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0 });
        NCollection_Array1<int> aVMults = NCollection_Array1<int>.From(new[] { 4, 1, 4 });
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
    }

    // A bicubic B-spline bump on [0, 4] x [0, 4] (x and y from 1 to 7), with its extreme near the middle: up for
    // theSign = 1, down for -1.
    private static BSplineSurface MakeBump(double theSign, double theZ)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 7, 1, 7);
        for (int i = 1; i <= 7; i++)
        {
            for (int j = 1; j <= 7; j++)
            {
                double r2 = (i - 4.1) * (i - 4.1) + (j - 3.9) * (j - 3.9);
                aPoles[i, j] = new Point3D(i, j, theZ + theSign * 2.0 * Math.Exp(-r2 / 3.0));
            }
        }
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0, 3.0, 4.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 1, 1, 4 });
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    // A cubic B-spline curve above the sheet.
    private static BSplineCurve3D MakeBSplineCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        aPoles[1] = new Point3D(1.2, 1.1, 2.0);
        aPoles[2] = new Point3D(2.0, 2.5, 1.2);
        aPoles[3] = new Point3D(3.0, 1.5, 0.9);
        aPoles[4] = new Point3D(4.0, 3.5, 1.5);
        aPoles[5] = new Point3D(5.0, 2.0, 1.1);
        aPoles[6] = new Point3D(5.5, 4.5, 2.2);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0, 3.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 1, 4 });
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    // The minimum square distance between a curve and a surface over a dense grid.
    private static double SampledMinCS(CurveAdaptorBase theC, SurfaceAdaptorBase theS, int theN = 80)
    {
        double aMin = double.MaxValue;
        Point3D[] aSPnts = new Point3D[(theN + 1) * (theN + 1)];
        for (int i = 0; i <= theN; i++)
        {
            double u = theS.FirstUParameter() + (theS.LastUParameter() - theS.FirstUParameter()) * i / theN;
            for (int j = 0; j <= theN; j++)
            {
                double v = theS.FirstVParameter() + (theS.LastVParameter() - theS.FirstVParameter()) * j / theN;
                aSPnts[i * (theN + 1) + j] = theS.Value(u, v);
            }
        }
        for (int k = 0; k <= 4 * theN; k++)
        {
            Point3D aP = theC.Value(theC.FirstParameter() + (theC.LastParameter() - theC.FirstParameter()) * k / (4 * theN));
            foreach (Point3D aQ in aSPnts)
            {
                aMin = Math.Min(aMin, aP.SquareDistance(aQ));
            }
        }
        return aMin;
    }

    private static double SampledMinSS(SurfaceAdaptorBase theS1, SurfaceAdaptorBase theS2, int theN = 40)
    {
        Point3D[] Grid(SurfaceAdaptorBase theS)
        {
            Point3D[] aPnts = new Point3D[(theN + 1) * (theN + 1)];
            for (int i = 0; i <= theN; i++)
            {
                double u = theS.FirstUParameter() + (theS.LastUParameter() - theS.FirstUParameter()) * i / theN;
                for (int j = 0; j <= theN; j++)
                {
                    double v = theS.FirstVParameter() + (theS.LastVParameter() - theS.FirstVParameter()) * j / theN;
                    aPnts[i * (theN + 1) + j] = theS.Value(u, v);
                }
            }
            return aPnts;
        }
        Point3D[] aG1 = Grid(theS1), aG2 = Grid(theS2);
        double aMin = double.MaxValue;
        foreach (Point3D aP in aG1)
        {
            foreach (Point3D aQ in aG2)
            {
                aMin = Math.Min(aMin, aP.SquareDistance(aQ));
            }
        }
        return aMin;
    }

    // The extremum (t, u, v) is a stationary point of the distance: (C - S) is normal to C', Su and Sv.
    private static void AssertStationaryCS(CurveAdaptorBase theC, SurfaceAdaptorBase theS, POnCurv thePC,
        POnSurf thePS, double theTol = 1e-6)
    {
        theC.D1(thePC.Parameter(), out Point3D aPC, out Vector3D aDT);
        thePS.Parameter(out double u, out double v);
        theS.D1(u, v, out Point3D aPS, out Vector3D aDU, out Vector3D aDV);
        Vector3D aD = new(aPS, aPC);
        if (aD.Magnitude() < 1e-7)
        {
            return;
        }
        // At the bounds a component may be non-zero; the tests choose interior extrema.
        Assert.True(Math.Abs(aD.Dot(aDT)) < theTol * aDT.Magnitude() * Math.Max(1.0, aD.Magnitude()), "C' not normal");
        Assert.True(Math.Abs(aD.Dot(aDU)) < theTol * aDU.Magnitude() * Math.Max(1.0, aD.Magnitude()), "Su not normal");
        Assert.True(Math.Abs(aD.Dot(aDV)) < theTol * aDV.Magnitude() * Math.Max(1.0, aD.Magnitude()), "Sv not normal");
    }

    [Fact]
    public void GlobOptFuncCS_GradientAndHessian_MatchFiniteDifferences()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(1.5, 1.0, 2.0), new Direction3D(0.2, 0.3, 1.0)), 0.8));
        SurfaceAdaptor aS = new(MakeBSplineSurface());
        GlobOptFuncCS aF = new(aC, aS);
        Assert.Equal(3, aF.VariableCount);

        math_Vector X = new(1, 3);
        X[1] = 1.1;
        X[2] = 1.3;
        X[3] = 0.7;
        Assert.True(aF.Value(X, out double F0));
        Assert.Equal(aC.Value(1.1).SquareDistance(aS.Value(1.3, 0.7)), F0, 1e-14);

        math_Vector G = new(1, 3);
        Matrix H = new(1, 3, 1, 3);
        Assert.True(aF.Values(X, out double F1, G, H));
        Assert.Equal(F0, F1);

        const double h = 1e-6;
        for (int k = 1; k <= 3; k++)
        {
            math_Vector Xp = new(X), Xm = new(X);
            Xp[k] += h;
            Xm[k] -= h;
            aF.Value(Xp, out double Fp);
            aF.Value(Xm, out double Fm);
            // OCCT's gradient and hessian are those of F / 2
            Assert.Equal(0.5 * (Fp - Fm) / (2 * h), G[k], 1e-6);

            math_Vector Gp = new(1, 3), Gm = new(1, 3);
            aF.Gradient(Xp, Gp);
            aF.Gradient(Xm, Gm);
            for (int l = 1; l <= 3; l++)
            {
                Assert.Equal((Gp[l] - Gm[l]) / (2 * h), H[l, k], 1e-5);
            }
        }

        // Outside the ranges
        X[2] = 10.0;
        Assert.False(aF.Value(X, out _));
        Assert.False(aF.Gradient(X, G));
    }

    [Fact]
    public void GlobOptFuncConicS_ValueAndConicParameter()
    {
        // A line along X at height 2 above the plane z = 0, on [-1, 1]
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, 2), new Direction3D(1, 0, 0)), -1.0, 1.0);
        SurfaceAdaptor aS = new(new PlaneSurface(new CoordinateSystem()), -5, 5, -5, 5);
        GlobOptFuncConicS aF = new(aC, aS);
        Assert.Equal(2, aF.VariableCount);

        math_Vector UV = new(1, 2);
        UV[1] = 0.5;
        UV[2] = 1.0;
        Assert.True(aF.Value(UV, out double F));
        // Projection of (0.5, 1, 0) on the line is (0.5, 0, 2), inside the range
        Assert.Equal(1.0 + 4.0, F, 1e-14);
        Assert.Equal(0.5, aF.ConicParameter(UV), 1e-14);

        // Beyond the line's range: the nearest end point
        UV[1] = 3.0;
        Assert.True(aF.Value(UV, out F));
        Assert.Equal(4.0 + 1.0 + 4.0, F, 1e-14);
        Assert.Equal(1.0, aF.ConicParameter(UV), 1e-14);

        // Outside the surface range
        UV[1] = 6.0;
        Assert.False(aF.Value(UV, out _));

        // Circle: parameters are shifted into the range when it goes past 2 Pi
        CurveAdaptor aCirc = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 1.0), 4.0, 8.0);
        GlobOptFuncConicS aFC = new(aS);
        aFC.LoadConic(aCirc, 4.0, 8.0);
        UV[1] = Math.Cos(7.0) * 3;
        UV[2] = Math.Sin(7.0) * 3;
        Assert.True(aFC.Value(UV, out F));
        Assert.Equal(4.0 + 1.0, F, 1e-12);
        Assert.Equal(7.0, aFC.ConicParameter(UV), 1e-12);
    }

    [Fact]
    public void GlobOptFuncCQuadric_ValueAndQuadricParameters()
    {
        // A sphere of radius 1 at the origin; a line along X at height 3
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, 3), new Direction3D(1, 0, 0)), -2.0, 2.0);
        SurfaceAdaptor aS = new(new SphericalSurface(new CoordinateSystem(), 1.0));
        GlobOptFuncCQuadric aF = new(aC, aS);
        Assert.Equal(1, aF.VariableCount);

        math_Vector T = new(1, 1);
        T[1] = 0.0;
        Assert.True(aF.Value(T, out double F));
        Assert.Equal(4.0, F, 1e-12); // (3 - 1)^2

        math_Vector UV = new(1, 2);
        aF.QuadricParameters(T, UV);
        Assert.Equal(M_PI / 2, UV[2], 1e-12); // the north pole

        T[1] = 1.5;
        Assert.True(aF.Value(T, out F));
        double d = Math.Sqrt(1.5 * 1.5 + 9.0) - 1.0;
        Assert.Equal(d * d, F, 1e-12);

        T[1] = 3.0;
        Assert.False(aF.Value(T, out _));

        // Line vs a bounded plane: the projection when inside, the nearest corner otherwise
        SurfaceAdaptor aP = new(new PlaneSurface(new CoordinateSystem()), 0, 1, 0, 1);
        GlobOptFuncCQuadric aFP = new(aC, -2.0, 2.0);
        aFP.LoadQuad(aP, 0, 1, 0, 1);
        T[1] = 0.5;
        Assert.True(aFP.Value(T, out F));
        Assert.Equal(9.0, F, 1e-14);
        T[1] = 2.0;
        Assert.True(aFP.Value(T, out F));
        Assert.Equal(1.0 + 9.0, F, 1e-14); // corner (1, 0, 0)
        aFP.QuadricParameters(T, UV);
        Assert.Equal(1.0, UV[1]);
        Assert.Equal(0.0, UV[2]);
    }

    [Fact]
    public void FuncExtCS_DerivativesMatchFiniteDifferences()
    {
        CurveAdaptor aC = new(MakeBSplineCurve());
        SurfaceAdaptor aS = new(MakeBSplineSurface());
        FuncExtCS aF = new(aC, aS);
        Assert.Equal(3, aF.NbVariables());
        Assert.Equal(3, aF.NbEquations());

        math_Vector X = new(1, 3);
        X[1] = 1.3;
        X[2] = 1.7;
        X[3] = 0.6;
        math_Vector F = new(1, 3);
        Matrix D = new(1, 3, 1, 3);
        Assert.True(aF.Values(X, F, D));
        const double h = 1e-6;
        for (int k = 1; k <= 3; k++)
        {
            math_Vector Xp = new(X), Xm = new(X), Fp = new(1, 3), Fm = new(1, 3);
            Xp[k] += h;
            Xm[k] -= h;
            aF.Value(Xp, Fp);
            aF.Value(Xm, Fm);
            for (int l = 1; l <= 3; l++)
            {
                Assert.Equal((Fp[l] - Fm[l]) / (2 * h), D[l, k], 1e-5);
            }
        }

        // GetStateNumber stores a solution once per curve parameter
        aF.Value(X, F);
        aF.GetStateNumber();
        aF.GetStateNumber();
        Assert.Equal(1, aF.ExtCount);
        Assert.Equal(aC.Value(1.3).SquareDistance(aS.Value(1.7, 0.6)), aF.SquareDistance(1), 1e-14);
        Assert.Equal(1.3, aF.PointOnCurve(1).Parameter());

        Assert.Throws<TypeMismatchException>(() => new FuncExtCS().Value(X, F));
    }

    [Fact]
    public void FuncExtSS_DerivativesMatchFiniteDifferences()
    {
        SurfaceAdaptor aS1 = new(MakeBSplineSurface());
        SurfaceAdaptor aS2 = new(new SphericalSurface(new CoordinateSystem(new Point3D(2, 1, 3), new Direction3D(0, 0, 1)), 1.0));
        FuncExtSS aF = new(aS1, aS2);
        Assert.Equal(4, aF.NbVariables());
        Assert.Equal(4, aF.NbEquations());

        math_Vector X = new(1, 4);
        X[1] = 1.3;
        X[2] = 0.8;
        X[3] = 2.0;
        X[4] = -0.5;
        math_Vector F = new(1, 4);
        Matrix D = new(1, 4, 1, 4);
        Assert.True(aF.Values(X, F, D));
        const double h = 1e-6;
        for (int k = 1; k <= 4; k++)
        {
            math_Vector Xp = new(X), Xm = new(X), Fp = new(1, 4), Fm = new(1, 4);
            Xp[k] += h;
            Xm[k] -= h;
            aF.Value(Xp, Fp);
            aF.Value(Xm, Fm);
            for (int l = 1; l <= 4; l++)
            {
                Assert.Equal((Fp[l] - Fm[l]) / (2 * h), D[l, k], 1e-5);
            }
        }
    }

    [Fact]
    public void GenExtCS_LineSphere_QuadricPath()
    {
        // A line at distance 5 from the centre of a sphere of radius 2: the minimum is 3
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(-10, 5, 0), new Direction3D(1, 0, 0)), 0.0, 20.0);
        SurfaceAdaptor aS = new(new SphericalSurface(new CoordinateSystem(), 2.0));
        GenExtCS anExt = new(aC, aS, 10, 10, 10, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aMin = double.MaxValue;
        int aIMin = 0;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            if (anExt.SquareDistance(i) < aMin)
            {
                aMin = anExt.SquareDistance(i);
                aIMin = i;
            }
        }
        Assert.Equal(9.0, aMin, 1e-8);
        Assert.Equal(10.0, anExt.PointOnCurve(aIMin).Parameter(), 1e-5);
        Point3D aPS = anExt.PointOnSurface(aIMin).Value();
        Assert.Equal(0.0, aPS.X(), 1e-5);
        Assert.Equal(2.0, aPS.Y(), 1e-5);
    }

    [Fact]
    public void GenExtCS_CircleBSpline_ConicPath()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(3.5, 3.0, 1.5), new Direction3D(0.3, -0.2, 1.0)), 0.7));
        SurfaceAdaptor aS = new(MakeBSplineSurface());
        GenExtCS anExt = new(aC, aS, 20, 20, 20, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aSampled = SampledMinCS(aC, aS);
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
            AssertStationaryCS(aC, aS, anExt.PointOnCurve(i), anExt.PointOnSurface(i));
            Assert.Equal(anExt.PointOnCurve(i).Value().SquareDistance(anExt.PointOnSurface(i).Value()),
                anExt.SquareDistance(i), 1e-12);
        }
        Assert.True(aMin <= aSampled + 1e-9, $"{aMin} > sampled {aSampled}");
        Assert.True(Math.Sqrt(aMin) > Math.Sqrt(aSampled) - 0.05);
    }

    [Fact]
    public void GenExtCS_BSplineBSpline_GeneralPath()
    {
        CurveAdaptor aC = new(MakeBSplineCurve());
        SurfaceAdaptor aS = new(MakeBSplineSurface());
        GenExtCS anExt = new(aC, aS, 20, 20, 20, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aSampled = SampledMinCS(aC, aS);
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
        }
        Assert.True(aMin <= aSampled + 1e-9, $"{aMin} > sampled {aSampled}");
        Assert.True(Math.Sqrt(aMin) > Math.Sqrt(aSampled) - 0.05);

        Assert.Throws<OutOfRangeException>(() => anExt.SquareDistance(anExt.ExtCount + 1));
        Assert.Throws<NotDoneException>(() => new GenExtCS().ExtCount);
    }

    [Fact]
    public void GenExtCS_LinePlane_InitializeThenPerform()
    {
        // A line crossing a bounded plane: the minimum is 0 at the crossing
        SurfaceAdaptor aS = new(new PlaneSurface(new CoordinateSystem()), -2, 2, -2, 2);
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0.3, 0.2, -1), new Direction3D(0.1, 0.2, 1.0)), 0.0, 3.0);
        GenExtCS anExt = new();
        anExt.Initialize(aS, 10, 10, 1e-9);
        anExt.Perform(aC, 10, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        Assert.Equal(0.0, anExt.SquareDistance(1), 1e-12);
        Assert.Equal(0.0, anExt.PointOnSurface(1).Value().Z(), 1e-7);
    }

    [Fact]
    public void GenLocateExtCS_LineCylinder()
    {
        // A line parallel to the X axis at y = 4, cylinder of radius 1 around Z: the minimum is 3 at (0, 1, z)
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(-5, 4, 0.5), new Direction3D(1, 0, 0)), 0.0, 10.0);
        SurfaceAdaptor aS = new(new CylindricalSurface(new CoordinateSystem(), 1.0), 0, 2 * M_PI, -2, 2);
        GenLocateExtCS anExt = new(aC, aS, 4.5, 1.4, 0.3, 1e-10, 1e-10);
        Assert.True(anExt.IsDone);
        Assert.Equal(9.0, anExt.SquareDistance(), 1e-10);
        Assert.Equal(5.0, anExt.PointOnCurve().Parameter(), 1e-7);
        anExt.PointOnSurface().Parameter(out double u, out double v);
        Assert.Equal(M_PI / 2, u, 1e-7);
        Assert.Equal(0.5, v, 1e-7);

        Assert.Throws<NotDoneException>(() => new GenLocateExtCS().SquareDistance());
    }

    [Fact]
    public void GenLocateExtSS_SphereSphere()
    {
        SurfaceAdaptor aS1 = new(new SphericalSurface(new CoordinateSystem(), 1.0));
        SurfaceAdaptor aS2 = new(new SphericalSurface(new CoordinateSystem(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), 2.0));
        GenLocateExtSS anExt = new(aS1, aS2, 0.2, 0.1, M_PI - 0.2, -0.1, 1e-10, 1e-10);
        Assert.True(anExt.IsDone);
        Assert.Equal(4.0, anExt.SquareDistance(), 1e-10);
        Assert.Equal(1.0, anExt.PointOnS1().Value().X(), 1e-7);
        Assert.Equal(3.0, anExt.PointOnS2().Value().X(), 1e-7);
    }

    [Fact]
    public void GenExtSS_SphereSphere_MinAndMax()
    {
        SurfaceAdaptor aS1 = new(new SphericalSurface(new CoordinateSystem(), 1.0));
        SurfaceAdaptor aS2 = new(new SphericalSurface(new CoordinateSystem(new Point3D(3, 4, 0), new Direction3D(0, 0, 1)), 2.0));
        GenExtSS anExt = new(aS1, aS2, 20, 20, 1e-10, 1e-10);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 2);
        double aMin = double.MaxValue, aMax = 0.0;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
            aMax = Math.Max(aMax, anExt.SquareDistance(i));
            Assert.Equal(anExt.PointOnS1(i).Value().SquareDistance(anExt.PointOnS2(i).Value()), anExt.SquareDistance(i),
                1e-10);
        }
        Assert.Equal(4.0, aMin, 1e-7); // (5 - 1 - 2)^2
        Assert.Equal(64.0, aMax, 1e-7); // (5 + 1 + 2)^2
    }

    [Fact]
    public void ExtSS_ParallelPlanes()
    {
        SurfaceAdaptor aS1 = new(new PlaneSurface(new CoordinateSystem()), -1, 1, -1, 1);
        SurfaceAdaptor aS2 = new(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 2.5), new Direction3D(0, 0, -1))), -1, 1, -1, 1);
        ExtSS anExt = new(aS1, aS2, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.IsParallel);
        Assert.Equal(1, anExt.ExtCount);
        Assert.Equal(6.25, anExt.SquareDistance(1), 1e-14);
        // OCCT stores no points for parallel planes
        Assert.ThrowsAny<FailureException>(() => anExt.Points(1, out _, out _));
    }

    [Fact]
    public void ExtSS_IntersectingPlanes_NoExtrema()
    {
        SurfaceAdaptor aS1 = new(new PlaneSurface(new CoordinateSystem()), -1, 1, -1, 1);
        SurfaceAdaptor aS2 = new(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))), -1, 1, -1, 1);
        ExtSS anExt = new(aS1, aS2, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.False(anExt.IsParallel);
        Assert.Equal(0, anExt.ExtCount);
    }

    [Fact]
    public void ExtSS_PlaneBSpline_AgainstSampling()
    {
        SurfaceAdaptor aS1 = new(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 3), new Direction3D(0.05, 0.02, 1))), -10, 10,
            -10, 10);
        SurfaceAdaptor aS2 = new(MakeBump(1.0, 0.0));
        ExtSS anExt = new(aS1, aS2, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
            anExt.Points(i, out POnSurf aP1, out POnSurf aP2);
            Assert.Equal(aP1.Value().SquareDistance(aP2.Value()), anExt.SquareDistance(i), 1e-9);
        }
        double aSampled = SampledMinSS(aS1, aS2);
        Assert.True(aMin <= aSampled + 1e-9, $"{aMin} > sampled {aSampled}");
        Assert.True(Math.Sqrt(aMin) > Math.Sqrt(aSampled) - 0.05);
    }

    [Fact]
    public void ExtSS_BSplineBSpline_AgainstSampling()
    {
        SurfaceAdaptor aS1 = new(MakeBump(1.0, 0.0));
        SurfaceAdaptor aS2 = new((Surface)MakeBump(-1.0, 4.5).Rotated(new Axis3D(new Point3D(4, 4, 0), new Direction3D(0, 0, 1)), 0.4));
        ExtSS anExt = new(aS1, aS2, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
        }
        double aSampled = SampledMinSS(aS1, aS2);
        Assert.True(aMin <= aSampled + 1e-9, $"{aMin} > sampled {aSampled}");
        Assert.True(Math.Sqrt(aMin) > Math.Sqrt(aSampled) - 0.05);
    }
}
