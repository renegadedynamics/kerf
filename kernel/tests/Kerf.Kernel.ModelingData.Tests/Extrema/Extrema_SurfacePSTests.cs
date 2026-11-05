// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Extrema.ElementType;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Unit tests of the point/surface extrema of unit 4A (Extrema_FuncPSNorm, FuncPSDist, GenLocateExtPS, GenExtPS,
/// ExtPExtS, ExtPRevS, ExtPS, POnSurfParams). Results are checked against known answers, the elementary solver and
/// dense sampling; the oracle tests compare them with native OCCT.
/// </summary>
public class Extrema_SurfacePSTests
{
    // A bicubic B-spline bump on [0, 4] x [0, 4] (x and y from 1 to 7), its top near (4.1, 3.9, 2).
    private static BSplineSurface MakeBump()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 7, 1, 7);
        for (int i = 1; i <= 7; i++)
        {
            for (int j = 1; j <= 7; j++)
            {
                double r2 = (i - 4.1) * (i - 4.1) + (j - 3.9) * (j - 3.9);
                aPoles[i, j] = new Point3D(i, j, 2.0 * Math.Exp(-r2 / 3.0));
            }
        }
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0, 3.0, 4.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 1, 1, 4 });
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    private static double SampledMin(SurfaceAdaptorBase theS, in Point3D theP, int theN = 200)
        => SampledMin(theS, theP, theS.FirstUParameter(), theS.LastUParameter(), theS.FirstVParameter(),
            theS.LastVParameter(), theN);

    private static double SampledMin(SurfaceAdaptorBase theS, in Point3D theP, double theU1, double theU2, double theV1,
        double theV2, int theN = 200)
    {
        double aMin = double.MaxValue;
        for (int i = 0; i <= theN; i++)
        {
            double u = theU1 + (theU2 - theU1) * i / theN;
            for (int j = 0; j <= theN; j++)
            {
                double v = theV1 + (theV2 - theV1) * j / theN;
                aMin = Math.Min(aMin, theP.SquareDistanceTo(theS.Value(u, v)));
            }
        }
        return aMin;
    }

    private static double MinSquareDistance(ExtPS theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    // Each extremum is a stationary point of the distance: (S - P) is normal to the surface, or P is on it.
    private static void AssertStationary(SurfaceAdaptorBase theS, in Point3D theP, POnSurf thePS, double theTol = 1e-6)
    {
        thePS.Parameter(out double u, out double v);
        theS.D1(u, v, out Point3D aS, out Vector3D aDu, out Vector3D aDv);
        Assert.True(aS.DistanceTo(thePS.Value()) < 1e-9);
        Vector3D aPS = new(theP, aS);
        if (aPS.Magnitude < 1e-7)
        {
            return;
        }
        Assert.True(Math.Abs(aPS.Dot(aDu)) / (aPS.Magnitude * aDu.Magnitude) < theTol, "not normal along U");
        Assert.True(Math.Abs(aPS.Dot(aDv)) / (aPS.Magnitude * aDv.Magnitude) < theTol, "not normal along V");
    }

    /// <summary>
    /// OCCT's GeomAPI_ProjectPointOnSurf GTest (Bug OCC867), on the Extrema_ExtPS path GeomAPI_ProjectPointOnSurf
    /// takes: Init() with UV bounds tighter than the trimmed surface's, then Perform(), must not throw.
    /// </summary>
    [Fact]
    public void Bug867_InitWithTightBoundsNoException()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(Constants.XOY()), 10.0);
        TrimmedSurface aTrimSurf = new(aCyl, 0.0, 4.0, 0.0, 2.0);
        Point3D aPoint = new(30.0, 30.0, 30.0);

        // GeomAPI_ProjectPointOnSurf::Init(Surface, 0, 3, 0, 2) and Perform(aPoint)
        double aTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        SurfaceAdaptor anAdaptor = new();
        ExtPS anExtPS = new();
        Exception? anError = Record.Exception(() =>
        {
            anAdaptor.Load(aTrimSurf, 0.0, 3.0, 0.0, 2.0);
            anExtPS.SetAlgo(ExtAlgo_Grad);
            anExtPS.Initialize(anAdaptor, 0.0, 3.0, 0.0, 2.0, aTol, aTol);
            anExtPS.Perform(aPoint);
        });
        Assert.Null(anError);
    }

    [Fact]
    public void ExtPS_Elementary_MatchExtPElS()
    {
        Point3D aP = new(3.0, -2.0, 5.0);
        Surface[] aSurfaces =
        [
            new PlaneSurface(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1))),
            new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 2.0),
            new SphericalSurface(new CoordinateSystem(Constants.XOY()), 2.0),
            new ToroidalSurface(new CoordinateSystem(Constants.XOY()), 5.0, 1.0),
            new ConicalSurface(new CoordinateSystem(Constants.XOY()), 0.3, 1.0),
        ];
        foreach (Surface aS in aSurfaces)
        {
            SurfaceAdaptor anAdaptor = new(aS);
            ExtPS anExt = new(aP, anAdaptor, 1e-9, 1e-9);
            Assert.True(anExt.IsDone);
            Assert.True(anExt.ExtCount > 0);
            for (int i = 1; i <= anExt.ExtCount; i++)
            {
                AssertStationary(anAdaptor, aP, anExt.Point(i));
                Assert.Equal(aP.SquareDistanceTo(anExt.Point(i).Value()), anExt.SquareDistance(i), 1e-9);
            }
        }

        // The sphere: the nearest point is on the line to the centre
        SurfaceAdaptor aSphere = new(new SphericalSurface(new CoordinateSystem(Constants.XOY()), 2.0));
        ExtPS aSphereExt = new(aP, aSphere, 1e-9, 1e-9);
        Assert.Equal(Math.Pow(Math.Sqrt(9 + 4 + 25) - 2.0, 2), MinSquareDistance(aSphereExt), 1e-10);
    }

    [Fact]
    public void ExtPS_BSplineBump_FindsTheNearestPoint()
    {
        SurfaceAdaptor aBump = new(MakeBump());
        Point3D[] aPoints = [new(4.1, 3.9, 6.0), new(2.0, 2.5, 3.0), new(6.0, 6.5, -1.0), new(3.3, 4.4, 0.5)];
        foreach (Point3D aP in aPoints)
        {
            foreach (ExtAlgo anAlgo in new[] { ExtAlgo_Grad, ExtAlgo_Tree })
            {
                ExtPS anExt = new(aP, aBump, 1e-9, 1e-9, ExtFlag_MIN, anAlgo);
                Assert.True(anExt.IsDone);
                double aSampled = SampledMin(aBump, aP);
                Assert.True(MinSquareDistance(anExt) <= aSampled + 1e-9, $"{aP} {anAlgo}");
                Assert.True(MinSquareDistance(anExt) > aSampled - 1e-3, $"{aP} {anAlgo}");
                for (int i = 1; i <= anExt.ExtCount; i++)
                {
                    AssertStationary(aBump, aP, anExt.Point(i), 1e-5);
                }
            }
        }
    }

    [Fact]
    public void GenExtPS_MaxFlag_FindsTheFarthestInteriorPoint()
    {
        // Below the bump's top: the farthest interior stationary point of the distance is the top
        SurfaceAdaptor aBump = new(MakeBump());
        Point3D aP = new(4.0, 4.0, -10.0);
        GenExtPS anExt = new(aP, aBump, 20, 20, 1e-9, 1e-9, ExtFlag_MAX);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);
        double aMax = 0.0;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            aMax = Math.Max(aMax, anExt.SquareDistance(i));
            AssertStationary(aBump, aP, anExt.Point(i), 1e-5);
        }
        Assert.True(aMax > 130.0);

        // Reusing the grid for another point gives the same results as a new algorithm
        Point3D aQ = new(3.0, 5.0, -8.0);
        anExt.Perform(aQ);
        GenExtPS aNew = new(aQ, aBump, 20, 20, 1e-9, 1e-9, ExtFlag_MAX);
        Assert.Equal(aNew.ExtCount, anExt.ExtCount);
        for (int i = 1; i <= aNew.ExtCount; i++)
        {
            Assert.Equal(aNew.SquareDistance(i), anExt.SquareDistance(i));
            Assert.Equal(aNew.Point(i), anExt.Point(i));
        }
    }

    /// <summary>
    /// OCCT's tree search for the maximum starts its selector at a maximal distance of 0, which Bnd_Sphere::IsOut reads
    /// as "reject every sphere farther than 0": nothing is selected, and the solution is refined from the first grid
    /// node instead of the farthest one (occt-issues.md). The gradient search finds the farthest point.
    /// </summary>
    [Fact]
    public void GenExtPS_TreeMax_StartsFromTheFirstNode_AsInOcct()
    {
        SurfaceAdaptor aBump = new(MakeBump());
        Point3D aP = new(4.0, 4.0, -10.0);
        GenExtPS aGrad = new(aP, aBump, 20, 20, 1e-9, 1e-9, ExtFlag_MAX, ExtAlgo_Grad);
        GenExtPS aTree = new(aP, aBump, 20, 20, 1e-9, 1e-9, ExtFlag_MAX, ExtAlgo_Tree);
        Assert.True(aGrad.IsDone && aTree.IsDone);
        double aGradMax = Enumerable.Range(1, aGrad.ExtCount).Max(aGrad.SquareDistance);
        double aTreeMax = aTree.ExtCount == 0 ? 0.0 : Enumerable.Range(1, aTree.ExtCount).Max(aTree.SquareDistance);
        Assert.True(aTreeMax < aGradMax - 1.0, $"tree {aTreeMax}, gradient {aGradMax}");
    }

    [Fact]
    public void GenExtPS_Errors()
    {
        SurfaceAdaptor aBump = new(MakeBump());
        GenExtPS anExt = new();
        Assert.False(anExt.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anExt.ExtCount);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anExt.Initialize(aBump, 1, 10, 1e-9, 1e-9));
    }

    [Fact]
    public void ExtPExtS_Analytic_ExtrusionOfCircle()
    {
        // A circle of radius 2 in XOY extruded along an oblique direction: the analytic path
        CircleCurve3D aCircle = new(Constants.XOY(), 2.0);
        Direction3D aDir = new(0.3, 0.1, 1.0);
        SurfaceOfLinearExtrusion aS = new(new CurveAdaptor(aCircle), aDir);
        Point3D aP = new(5.0, 1.0, 3.0);
        ExtPExtS anExt = new(aP, aS, 0.0, 2 * Math.PI, -10.0, 10.0, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.NbExt() >= 2);
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            AssertStationary(aS, aP, anExt.Point(i), 1e-6);
        }
        Assert.Equal(SampledMin(aS, aP, 0.0, 2 * Math.PI, -10.0, 10.0, 400), Enumerable.Range(1, anExt.NbExt()).Min(anExt.SquareDistance), 1e-2);

        // Straight extrusion: the simple case, distances in the section plane
        SurfaceOfLinearExtrusion aStraight = new(new CurveAdaptor(aCircle), new Direction3D(0, 0, 1));
        ExtPExtS aStraightExt = new(aP, aStraight, 0.0, 2 * Math.PI, -10.0, 10.0, 1e-9, 1e-9);
        Assert.True(aStraightExt.IsDone);
        Assert.Equal(2, aStraightExt.NbExt());
        double aR = Math.Sqrt(26.0);
        Assert.Equal((aR - 2.0) * (aR - 2.0), Math.Min(aStraightExt.SquareDistance(1), aStraightExt.SquareDistance(2)), 1e-12);
        Assert.Equal((aR + 2.0) * (aR + 2.0), Math.Max(aStraightExt.SquareDistance(1), aStraightExt.SquareDistance(2)), 1e-12);
    }

    [Fact]
    public void ExtPExtS_General_ExtrusionOfBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, -1, 0);
        aPoles[4] = new Point3D(4, 1, 0);
        BezierCurve3D aCurve = new(aPoles);
        SurfaceOfLinearExtrusion aS = new(new CurveAdaptor(aCurve), new Direction3D(0, 0, 1));
        Point3D aP = new(2.0, 3.0, 1.0);
        ExtPExtS anExt = new(aP, aS, 0.0, 1.0, -5.0, 5.0, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.NbExt() >= 1);
        Assert.Equal(SampledMin(aS, aP, 0.0, 1.0, -5.0, 5.0, 400), Enumerable.Range(1, anExt.NbExt()).Min(anExt.SquareDistance), 1e-2);
    }

    [Fact]
    public void ExtPRevS_Analytic_RevolutionOfLine()
    {
        // A line parallel to Z at distance 3, revolved about Z: a cylinder of radius 3
        LineCurve3D aLine = new(new Axis3D(new Point3D(3, 0, 0), new Direction3D(0, 0, 1)));
        RevolutionSurfaceAdaptor aS = new(new CurveAdaptor(aLine, -10, 10), new Axis3D(Constants.Origin(), new Direction3D(0, 0, 1)));
        Point3D aP = new(1.0, 7.0, 2.0);
        ExtPRevS anExt = new(aP, aS, 0.0, 2 * Math.PI, -10.0, 10.0, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.Equal(2, anExt.ExtCount);
        double aR = Math.Sqrt(50.0);
        Assert.Equal((aR - 3.0) * (aR - 3.0), Math.Min(anExt.SquareDistance(1), anExt.SquareDistance(2)), 1e-10);
        Assert.Equal((aR + 3.0) * (aR + 3.0), Math.Max(anExt.SquareDistance(1), anExt.SquareDistance(2)), 1e-10);
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            AssertStationary(aS, aP, anExt.Point(i), 1e-9);
        }

        // A point on the axis: not done, as in OCCT
        ExtPRevS anOnAxis = new(new Point3D(0, 0, 4), aS, 0.0, 2 * Math.PI, -10.0, 10.0, 1e-9, 1e-9);
        Assert.False(anOnAxis.IsDone);
    }

    [Fact]
    public void ExtPS_SweptSurfaces_ThroughExtPS()
    {
        // Revolution of a B-spline curve (the general path through Extrema_GenExtPS) and a periodic solution put in
        // the range
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(2, 0, 0);
        aPoles[2] = new Point3D(3, 0, 1);
        aPoles[3] = new Point3D(2.5, 0, 2);
        aPoles[4] = new Point3D(3.5, 0, 3);
        RevolutionSurface aRev = new(new BezierCurve3D(aPoles), new Axis3D(Constants.Origin(), new Direction3D(0, 0, 1)));
        SurfaceAdaptor anAdaptor = new(aRev);
        Point3D aP = new(-5.0, -1.0, 1.5);
        ExtPS anExt = new(aP, anAdaptor, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.Equal(SampledMin(anAdaptor, aP, 300), MinSquareDistance(anExt), 1e-2);
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            anExt.Point(i).Parameter(out double u, out double v);
            Assert.InRange(u, -1e-9, 2 * Math.PI + 1e-9);
            Assert.InRange(v, -1e-9, 1.0 + 1e-9);
        }

        // Another point with the same algorithm reuses its ExtPRevS
        Point3D aQ = new(1.0, 4.0, 2.5);
        anExt.Perform(aQ);
        Assert.True(anExt.IsDone);
        Assert.Equal(SampledMin(anAdaptor, aQ, 300), MinSquareDistance(anExt), 1e-2);
    }

    [Fact]
    public void ExtPS_TrimmedSquareDistances_AreNeverComputed()
    {
        ExtPS anExt = new();
        anExt.TrimmedSquareDistances(out double d11, out double d12, out double d21, out double d22, out Point3D p11,
            out _, out _, out _);
        Assert.Equal(0.0, d11 + d12 + d21 + d22);
        Assert.Equal(new Point3D(), p11);
        Assert.False(anExt.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anExt.ExtCount);
    }

    [Fact]
    public void GenLocateExtPS_BothCriteria()
    {
        SurfaceAdaptor aBump = new(MakeBump());
        Point3D aP = new(4.1, 3.9, 6.0);
        GenLocateExtPS aLocate = new(aBump, 1e-9, 1e-9);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aLocate.SquareDistance());

        aLocate.Perform(aP, 2.0, 2.0);
        Assert.True(aLocate.IsDone);
        AssertStationary(aBump, aP, aLocate.Point(), 1e-6);
        double aNormal = aLocate.SquareDistance();

        aLocate.Perform(aP, 2.0, 2.0, true);
        Assert.True(aLocate.IsDone);
        Assert.Equal(aNormal, aLocate.SquareDistance(), 1e-6);
        aLocate.Point().Parameter(out double u, out double v);
        Assert.True(GenLocateExtPS.IsMinDist(aP, aBump, u, v));
        Assert.False(GenLocateExtPS.IsMinDist(aP, aBump, 0.5, 0.5));
    }

    [Fact]
    public void FuncPSNormAndDist_DerivativesMatchFiniteDifferences()
    {
        SurfaceAdaptor aBump = new(MakeBump());
        Point3D aP = new(2.0, 5.0, 3.0);
        FuncPSNorm aNorm = new(aP, aBump);
        FuncPSDist aDist = new(aBump, aP);
        math_Vector X = new(1, 2);
        X[1] = 1.3;
        X[2] = 2.6;
        math_Vector F = new(1, 2), G = new(1, 2);
        Matrix D = new(1, 2, 1, 2);
        Assert.True(aNorm.Values(X, F, D));
        Assert.True(aDist.Values(X, out double aF, G));
        const double h = 1e-6;
        for (int k = 1; k <= 2; k++)
        {
            math_Vector Xp = new(X), Xm = new(X);
            Xp[k] += h;
            Xm[k] -= h;
            math_Vector Fp = new(1, 2), Fm = new(1, 2);
            aNorm.Value(Xp, Fp);
            aNorm.Value(Xm, Fm);
            for (int i = 1; i <= 2; i++)
            {
                Assert.Equal((Fp[i] - Fm[i]) / (2 * h), D[i, k], 1e-5);
            }
            aDist.Value(Xp, out double aFp);
            aDist.Value(Xm, out double aFm);
            // OCCT's gradient is that of F / 2, with the sign of (P - S)
            Assert.Equal(-(aFp - aFm) / (4 * h), G[k], 1e-5);
            // FuncPSNorm's F is (S - P).Su, FuncPSDist's gradient (P - S).Su
            Assert.Equal(-F[k], G[k], 1e-12);
        }
        Assert.Equal(aP.SquareDistanceTo(aBump.Value(1.3, 2.6)), aF);

        // Outside the range, FuncPSDist returns false
        X[1] = -1.0;
        Assert.False(aDist.Value(X, out _));
        Assert.False(aDist.Gradient(X, G));

        // GetStateNumber stores each new root once
        X[1] = 1.3;
        aNorm.Value(X, F);
        aNorm.GetStateNumber();
        aNorm.GetStateNumber();
        Assert.Equal(1, aNorm.ExtCount);
        aNorm.SetPoint(aP);
        Assert.Equal(0, aNorm.ExtCount);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.TypeMismatchException>(() => new FuncPSNorm().Value(X, F));
    }

    [Fact]
    public void POnSurfParams_DefaultsAndTwins()
    {
        POnSurfParams aDefault = new();
        Assert.Equal(new POnSurf(), aDefault.POnSurf());
        Assert.Equal(0.0, aDefault.GetSqrDistance());
        Assert.Equal(Node, aDefault.GetElementType());
        aDefault.GetIndices(out int iu, out int iv);
        Assert.Equal((0, 0), (iu, iv));

        POnSurfParams aP = new POnSurfParams(1.0, 2.0, new Point3D(1, 2, 3))
            .WithSqrDistance(4.0).WithElementType(Face).WithIndices(3, 5);
        aP.Parameter(out double u, out double v);
        Assert.Equal((1.0, 2.0), (u, v));
        Assert.Equal(new Point3D(1, 2, 3), aP.Value());
        Assert.Equal(4.0, aP.GetSqrDistance());
        Assert.Equal(Face, aP.GetElementType());
        aP.GetIndices(out iu, out iv);
        Assert.Equal((3, 5), (iu, iv));
        Assert.NotEqual(aDefault, aP);
        Assert.Equal(aP, aP.WithIndices(3, 5));
        Assert.Contains(", Face, ", aP.ToString(), StringComparison.Ordinal);
    }
}
