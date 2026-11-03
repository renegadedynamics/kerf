// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Adaptor3d_IsoCurve (no OCCT GTest): evaluation against the surface evaluated directly, the type and conic
/// queries on the elementary and swept surfaces, intervals, periodic adjustment, trimming and copies.
/// </summary>
public class Adaptor3d_IsoCurveTests
{
    private static readonly CoordinateSystem TheAx3 = new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0.2, 0.3, 1.0), new Direction3D(1, 0, -0.2));

    private static BSplineCurve3D MakeBSplineCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(2, 0, 0);
        aPoles[2] = new Point3D(3, 0, 1);
        aPoles[3] = new Point3D(2.5, 0, 2);
        aPoles[4] = new Point3D(4, 0, 3);
        aPoles[5] = new Point3D(3, 0, 4);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 0.5;
        aKnots[3] = 1;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    private static BSplineSurface MakeBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, Math.Sin(i * 0.7 + j * 1.3));
            }
        }
        NCollection_Array1<double> aUK = new(1, 3);
        aUK[1] = 0;
        aUK[2] = 0.4;
        aUK[3] = 1;
        NCollection_Array1<int> aUM = new(1, 3);
        aUM[1] = 4;
        aUM[2] = 1;
        aUM[3] = 4;
        NCollection_Array1<double> aVK = new(1, 2);
        aVK[1] = 0;
        aVK[2] = 2;
        NCollection_Array1<int> aVM = new(1, 2);
        aVM[1] = 4;
        aVM[2] = 4;
        return new BSplineSurface(aPoles, aUK, aVK, aUM, aVM, 3, 3);
    }

    public static TheoryData<string> SurfaceNames() =>
        new() { "plane", "cylinder", "cone", "sphere", "torus", "bspline", "revolution", "extrusion" };

    private static TestSurfaceAdaptor MakeSurface(string theName) => theName switch
    {
        "plane" => new TestSurfaceAdaptor(new PlaneSurface(TheAx3), -5, 5, -4, 4),
        "cylinder" => new TestSurfaceAdaptor(new CylindricalSurface(TheAx3, 2.0), 0, 2 * Math.PI, -3, 3),
        "cone" => new TestSurfaceAdaptor(new ConicalSurface(TheAx3, 0.4, 2.0), 0, 2 * Math.PI, -3, 3),
        "sphere" => new TestSurfaceAdaptor(new SphericalSurface(TheAx3, 2.0)),
        "torus" => new TestSurfaceAdaptor(new ToroidalSurface(TheAx3, 5.0, 1.5)),
        "bspline" => new TestSurfaceAdaptor(MakeBSplineSurface()),
        "revolution" => new TestSurfaceAdaptor(new RevolutionSurface(MakeBSplineCurve(), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)))),
        _ => new TestSurfaceAdaptor(new ExtrusionSurface(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0),
                                                                  new Direction3D(0.1, 0.2, 1.0)), 0, 2 * Math.PI, -2, 2),
    };

    private static void SameVec(in Vector3D theExpected, in Vector3D theActual, string theWhat)
        => Assert.True(theExpected.IsEqual(theActual, 1e-12, 1e-12) || (theExpected - theActual).Magnitude == 0.0,
                       $"{theWhat}: {theExpected} {theActual}");

    [Theory]
    [MemberData(nameof(SurfaceNames))]
    public void Evaluation_MatchesSurface(string theName)
    {
        TestSurfaceAdaptor aS = MakeSurface(theName);
        Surface aGS = aS.Surface();
        foreach (global::Kerf.Engine.Geometry.IsoType anIso in new[] { IsoU, IsoV })
        {
            double aParam = anIso == IsoU
                                ? aS.FirstUParameter() + 0.37 * (aS.LastUParameter() - aS.FirstUParameter())
                                : aS.FirstVParameter() + 0.61 * (aS.LastVParameter() - aS.FirstVParameter());
            IsoCurve anIsoC = new IsoCurve(aS, anIso, aParam);
            Assert.Equal(anIso, anIsoC.Iso());
            Assert.Equal(aParam, anIsoC.Parameter());
            Assert.Same(aS, anIsoC.Surface());
            double aF = anIsoC.FirstParameter(), aL = anIsoC.LastParameter();
            Assert.Equal(anIso == IsoU ? aS.FirstVParameter() : aS.FirstUParameter(), aF);
            Assert.Equal(anIso == IsoU ? aS.LastVParameter() : aS.LastUParameter(), aL);
            for (int k = 0; k <= 10; k++)
            {
                double t = aF + (aL - aF) * k / 10.0;
                double u = anIso == IsoU ? aParam : t;
                double v = anIso == IsoU ? t : aParam;
                aGS.D3(u, v, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV,
                       out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV);
                anIsoC.D3(t, out Point3D CP, out Vector3D C1, out Vector3D C2, out Vector3D C3);
                Assert.True(P.IsEqual(CP, 0.0), $"{theName} {anIso} D0 at {t}");
                SameVec(anIso == IsoU ? D1V : D1U, C1, $"{theName} {anIso} D1");
                SameVec(anIso == IsoU ? D2V : D2U, C2, $"{theName} {anIso} D2");
                SameVec(anIso == IsoU ? D3V : D3U, C3, $"{theName} {anIso} D3");
                // EvalD0 and EvalD3 of a B-spline surface can differ in the last bit
                Assert.True(anIsoC.Value(t).IsEqual(P, 1e-12));
                anIsoC.D1(t, out CP, out C1);
                SameVec(anIso == IsoU ? D1V : D1U, C1, $"{theName} {anIso} D1 alone");
                anIsoC.D2(t, out CP, out C1, out C2);
                SameVec(anIso == IsoU ? D2V : D2U, C2, $"{theName} {anIso} D2 alone");
                SameVec(anIso == IsoU ? aGS.DN(u, v, 0, 2) : aGS.DN(u, v, 2, 0), anIsoC.DN(t, 2), $"{theName} DN");
            }

            // The shallow copy and the trimmed copy evaluate the same
            CurveAdaptorBase aCopy = anIsoC.ShallowCopy();
            double aMid = 0.5 * (aF + aL);
            Assert.True(aCopy.Value(aMid).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.True(aGS.Value(anIso == IsoU ? aParam : aMid, anIso == IsoU ? aMid : aParam).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.NotSame(aS, ((IsoCurve)aCopy).Surface());
            CurveAdaptorBase aTrim = anIsoC.Trim(aF + 0.25 * (aL - aF), aL - 0.25 * (aL - aF), 1e-7);
            Assert.Equal(aF + 0.25 * (aL - aF), aTrim.FirstParameter());
            Assert.Equal(aL - 0.25 * (aL - aF), aTrim.LastParameter());
            Assert.Same(aS, ((IsoCurve)aTrim).Surface());
            Assert.True(aTrim.Value(aMid).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Parametric(0.1), anIsoC.Resolution(0.1));
        }
    }

    [Fact]
    public void Types_AndConics()
    {
        foreach ((string Name, CurveKind U, CurveKind V) in new[]
                 {
                     ("plane", Line, Line), ("cylinder", Line, Circle),
                     ("cone", Line, Circle), ("sphere", Circle, Circle),
                     ("torus", Circle, Circle), ("bspline", BSplineCurve, BSplineCurve),
                     ("revolution", BSplineCurve, Circle), ("extrusion", Line, Circle),
                 })
        {
            TestSurfaceAdaptor aS = MakeSurface(Name);
            IsoCurve aUIso = new IsoCurve(aS, IsoU, 0.3 * aS.LastUParameter() + 0.7 * aS.FirstUParameter());
            IsoCurve aVIso = new IsoCurve(aS, IsoV, 0.6 * aS.LastVParameter() + 0.4 * aS.FirstVParameter());
            Assert.Equal(U, aUIso.GetType());
            Assert.Equal(V, aVIso.GetType());
            foreach (IsoCurve anIsoC in new[] { aUIso, aVIso })
            {
                double aF = anIsoC.FirstParameter(), aL = anIsoC.LastParameter();
                if (anIsoC.GetType() == Circle)
                {
                    // Every point of the iso lies on the circle
                    Circle3D aC = anIsoC.Circle();
                    for (int k = 0; k <= 8; k++)
                    {
                        Point3D P = anIsoC.Value(aF + (aL - aF) * k / 8.0);
                        Assert.True(aC.DistanceTo(P) < 1e-9, $"{Name} {anIsoC.Iso()} circle at {k}: {aC.DistanceTo(P)}");
                    }
                }
                else if (anIsoC.GetType() == Line)
                {
                    Line3D aLin = anIsoC.Line();
                    for (int k = 0; k <= 8; k++)
                    {
                        Assert.True(aLin.DistanceTo(anIsoC.Value(aF + (aL - aF) * k / 8.0)) < 1e-9);
                    }
                }
            }
        }
    }

    [Fact]
    public void Queries_OnBSplineAndSweptSurfaces()
    {
        TestSurfaceAdaptor aBS = MakeSurface("bspline");
        IsoCurve aU = new IsoCurve(aBS, IsoU, 0.2);
        IsoCurve aV = new IsoCurve(aBS, IsoV, 1.0);
        Assert.Equal(3, aU.Degree());
        Assert.Equal(4, aU.PoleCount);
        Assert.Equal(5, aV.PoleCount);
        Assert.Equal(2, aU.KnotCount);
        Assert.Equal(3, aV.KnotCount);
        Assert.False(aU.IsRational);
        Assert.False(aU.IsPeriodic);
        Assert.False(aU.IsClosed);
        Assert.Equal(C2, aU.Continuity());
        // The iso-curve's B-spline is the surface's own UIso, and evaluates as the adaptor does
        BSplineCurve3D anIso = aU.BSpline();
        Assert.Equal(4, anIso.PoleCount);
        for (int k = 0; k <= 8; k++)
        {
            double t = aU.FirstParameter() + (aU.LastParameter() - aU.FirstParameter()) * k / 8.0;
            Assert.True(anIso.Value(t).IsEqual(aU.Value(t), 1e-12));
        }

        // Revolution: the U iso is the basis curve rotated; the V iso a circle
        TestSurfaceAdaptor aRev = MakeSurface("revolution");
        IsoCurve aRU = new IsoCurve(aRev, IsoU, 1.1);
        Assert.Equal(3, aRU.Degree());
        Assert.Equal(5, aRU.PoleCount);
        Assert.Equal(3, aRU.KnotCount);
        Assert.False(aRU.IsRational);
        BSplineCurve3D aRot = aRU.BSpline();
        for (int k = 0; k <= 8; k++)
        {
            double t = k / 8.0;
            Assert.True(aRot.Value(t).IsEqual(aRU.Value(t), 1e-12));
        }
        IsoCurve aRV = new IsoCurve(aRev, IsoV, 0.5);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aRV.Degree());
        Assert.True(aRV.IsPeriodic);
        Assert.Equal(2 * Math.PI, aRV.Period());

        // Extrusion: the V iso is the basis curve translated
        TestSurfaceAdaptor anExt = MakeSurface("extrusion");
        IsoCurve anEV = new IsoCurve(anExt, IsoV, 1.5);
        Circle3D aC = anEV.Circle();
        Assert.True(aC.Location.IsEqual(new Point3D(0, 0, 0).Translated(1.5 * new Vector3D(new Direction3D(0.1, 0.2, 1.0))), 1e-12));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anEV.Hyperbola());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anEV.Parabola());
    }

    [Fact]
    public void NoneIso_Throws()
    {
        TestSurfaceAdaptor aS = MakeSurface("plane");
        IsoCurve anIsoC = new IsoCurve(aS);
        Assert.Equal(NoneIso, anIsoC.Iso());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anIsoC.Value(0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anIsoC.Continuity());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anIsoC.NbIntervals(C1));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anIsoC.IsClosed);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anIsoC.Load(NoneIso, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => new IsoCurve(MakeSurface("cylinder")).GetType());

        // Loading a surface resets the iso
        anIsoC.Load(IsoU, 1.0);
        Assert.Equal(IsoU, anIsoC.Iso());
        anIsoC.Load(aS);
        Assert.Equal(NoneIso, anIsoC.Iso());
    }

    [Fact]
    public void Load_ClampsRange_AndAdjustsPeriodicParameters()
    {
        TestSurfaceAdaptor aS = MakeSurface("plane");
        IsoCurve anIsoC = new IsoCurve(aS, IsoU, 1.0, -10.0, 2.0);
        Assert.Equal(-4.0, anIsoC.FirstParameter());
        Assert.Equal(2.0, anIsoC.LastParameter());

        // Cylinder (U periodic): a U iso's parameter, or a V iso's range, is brought into [0, 2 Pi)
        TestSurfaceAdaptor aCyl = new TestSurfaceAdaptor(new CylindricalSurface(TheAx3, 2.0), -10, 10, -3, 3);
        IsoCurve aUIso = new IsoCurve(aCyl, IsoU, 2 * Math.PI + 1.0);
        double aExpected = 2 * Math.PI + 1.0;
        double aDummy = aExpected;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-10, -10 + 2 * Math.PI, global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion()), ref aExpected, ref aDummy);
        Assert.Equal(aExpected, aUIso.Parameter());
        Assert.True(aUIso.Parameter() >= -10 && aUIso.Parameter() < -10 + 2 * Math.PI);
        Assert.True(aUIso.Value(0.5).IsEqual(aCyl.Value(2 * Math.PI + 1.0, 0.5), 1e-12));

        IsoCurve aVIso = new IsoCurve(aCyl, IsoV, 0.5, 1.0, 4.0);
        double aF = 1.0, aL = 4.0;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-10, -10 + 2 * Math.PI, global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion()), ref aF, ref aL);
        Assert.Equal(aF, aVIso.FirstParameter());
        Assert.Equal(aL, aVIso.LastParameter());
    }

    [Fact]
    public void Intervals_RestrictedToTheRange()
    {
        // A surface with V interval bounds at -1, 0 and 2 inside [-4, 4]
        TestSurfaceAdaptor aS = new TestSurfaceAdaptor(new PlaneSurface(TheAx3), -5, 5, -4, 4) { VBreaks = [-1.0, 0.0, 2.0] };
        IsoCurve anIsoC = new IsoCurve(aS, IsoU, 1.0, -2.0, 1.0);
        Assert.Equal(3, anIsoC.NbIntervals(C2));
        NCollection_Array1<double> T = new(1, 4);
        anIsoC.Intervals(T, C2);
        Assert.Equal(new[] { -2.0, -1.0, 0.0, 1.0 }, new[] { T[1], T[2], T[3], T[4] });

        // A range on an interval bound
        IsoCurve anOnBound = new IsoCurve(aS, IsoU, 1.0, -1.0, 2.0);
        Assert.Equal(2, anOnBound.NbIntervals(C2));
        NCollection_Array1<double> T2 = new(1, 3);
        anOnBound.Intervals(T2, C2);
        Assert.Equal(new[] { -1.0, 0.0, 2.0 }, new[] { T2[1], T2[2], T2[3] });

        // One interval: the range
        IsoCurve aVIso = new IsoCurve(aS, IsoV, 1.0, -2.0, 3.0);
        Assert.Equal(1, aVIso.NbIntervals(C2));
        NCollection_Array1<double> T3 = new(1, 2);
        aVIso.Intervals(T3, C2);
        Assert.Equal(new[] { -2.0, 3.0 }, new[] { T3[1], T3[2] });
    }
}
