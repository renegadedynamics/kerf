// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Adaptor3d_IsoCurve (no OCCT GTest): evaluation against the surface evaluated directly, the type and conic
/// queries on the elementary and swept surfaces, intervals, periodic adjustment, trimming and copies.
/// </summary>
public class Adaptor3d_IsoCurveTests
{
    private static readonly gp_Ax3 TheAx3 = new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0.2, 0.3, 1.0), new gp_Dir(1, 0, -0.2));

    private static Geom_BSplineCurve MakeBSplineCurve()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 5);
        aPoles[1] = new gp_Pnt(2, 0, 0);
        aPoles[2] = new gp_Pnt(3, 0, 1);
        aPoles[3] = new gp_Pnt(2.5, 0, 2);
        aPoles[4] = new gp_Pnt(4, 0, 3);
        aPoles[5] = new gp_Pnt(3, 0, 4);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 0.5;
        aKnots[3] = 1;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    private static Geom_BSplineSurface MakeBSplineSurface()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new gp_Pnt(i, j, Math.Sin(i * 0.7 + j * 1.3));
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
        return new Geom_BSplineSurface(aPoles, aUK, aVK, aUM, aVM, 3, 3);
    }

    public static TheoryData<string> SurfaceNames() =>
        new() { "plane", "cylinder", "cone", "sphere", "torus", "bspline", "revolution", "extrusion" };

    private static TestSurfaceAdaptor MakeSurface(string theName) => theName switch
    {
        "plane" => new TestSurfaceAdaptor(new Geom_Plane(TheAx3), -5, 5, -4, 4),
        "cylinder" => new TestSurfaceAdaptor(new Geom_CylindricalSurface(TheAx3, 2.0), 0, 2 * Math.PI, -3, 3),
        "cone" => new TestSurfaceAdaptor(new Geom_ConicalSurface(TheAx3, 0.4, 2.0), 0, 2 * Math.PI, -3, 3),
        "sphere" => new TestSurfaceAdaptor(new Geom_SphericalSurface(TheAx3, 2.0)),
        "torus" => new TestSurfaceAdaptor(new Geom_ToroidalSurface(TheAx3, 5.0, 1.5)),
        "bspline" => new TestSurfaceAdaptor(MakeBSplineSurface()),
        "revolution" => new TestSurfaceAdaptor(new Geom_SurfaceOfRevolution(MakeBSplineCurve(), new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)))),
        _ => new TestSurfaceAdaptor(new Geom_SurfaceOfLinearExtrusion(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0),
                                                                  new gp_Dir(0.1, 0.2, 1.0)), 0, 2 * Math.PI, -2, 2),
    };

    private static void SameVec(in gp_Vec theExpected, in gp_Vec theActual, string theWhat)
        => Assert.True(theExpected.IsEqual(theActual, 1e-12, 1e-12) || (theExpected - theActual).Magnitude() == 0.0,
                       $"{theWhat}: {theExpected} {theActual}");

    [Theory]
    [MemberData(nameof(SurfaceNames))]
    public void Evaluation_MatchesSurface(string theName)
    {
        TestSurfaceAdaptor aS = MakeSurface(theName);
        Geom_Surface aGS = aS.Surface();
        foreach (GeomAbs_IsoType anIso in new[] { GeomAbs_IsoU, GeomAbs_IsoV })
        {
            double aParam = anIso == GeomAbs_IsoU
                                ? aS.FirstUParameter() + 0.37 * (aS.LastUParameter() - aS.FirstUParameter())
                                : aS.FirstVParameter() + 0.61 * (aS.LastVParameter() - aS.FirstVParameter());
            Adaptor3d_IsoCurve anIsoC = new Adaptor3d_IsoCurve(aS, anIso, aParam);
            Assert.Equal(anIso, anIsoC.Iso());
            Assert.Equal(aParam, anIsoC.Parameter());
            Assert.Same(aS, anIsoC.Surface());
            double aF = anIsoC.FirstParameter(), aL = anIsoC.LastParameter();
            Assert.Equal(anIso == GeomAbs_IsoU ? aS.FirstVParameter() : aS.FirstUParameter(), aF);
            Assert.Equal(anIso == GeomAbs_IsoU ? aS.LastVParameter() : aS.LastUParameter(), aL);
            for (int k = 0; k <= 10; k++)
            {
                double t = aF + (aL - aF) * k / 10.0;
                double u = anIso == GeomAbs_IsoU ? aParam : t;
                double v = anIso == GeomAbs_IsoU ? t : aParam;
                aGS.D3(u, v, out gp_Pnt P, out gp_Vec D1U, out gp_Vec D1V, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV,
                       out gp_Vec D3U, out gp_Vec D3V, out gp_Vec D3UUV, out gp_Vec D3UVV);
                anIsoC.D3(t, out gp_Pnt CP, out gp_Vec C1, out gp_Vec C2, out gp_Vec C3);
                Assert.True(P.IsEqual(CP, 0.0), $"{theName} {anIso} D0 at {t}");
                SameVec(anIso == GeomAbs_IsoU ? D1V : D1U, C1, $"{theName} {anIso} D1");
                SameVec(anIso == GeomAbs_IsoU ? D2V : D2U, C2, $"{theName} {anIso} D2");
                SameVec(anIso == GeomAbs_IsoU ? D3V : D3U, C3, $"{theName} {anIso} D3");
                // EvalD0 and EvalD3 of a B-spline surface can differ in the last bit
                Assert.True(anIsoC.Value(t).IsEqual(P, 1e-12));
                anIsoC.D1(t, out CP, out C1);
                SameVec(anIso == GeomAbs_IsoU ? D1V : D1U, C1, $"{theName} {anIso} D1 alone");
                anIsoC.D2(t, out CP, out C1, out C2);
                SameVec(anIso == GeomAbs_IsoU ? D2V : D2U, C2, $"{theName} {anIso} D2 alone");
                SameVec(anIso == GeomAbs_IsoU ? aGS.DN(u, v, 0, 2) : aGS.DN(u, v, 2, 0), anIsoC.DN(t, 2), $"{theName} DN");
            }

            // The shallow copy and the trimmed copy evaluate the same
            Adaptor3d_Curve aCopy = anIsoC.ShallowCopy();
            double aMid = 0.5 * (aF + aL);
            Assert.True(aCopy.Value(aMid).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.True(aGS.Value(anIso == GeomAbs_IsoU ? aParam : aMid, anIso == GeomAbs_IsoU ? aMid : aParam).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.NotSame(aS, ((Adaptor3d_IsoCurve)aCopy).Surface());
            Adaptor3d_Curve aTrim = anIsoC.Trim(aF + 0.25 * (aL - aF), aL - 0.25 * (aL - aF), 1e-7);
            Assert.Equal(aF + 0.25 * (aL - aF), aTrim.FirstParameter());
            Assert.Equal(aL - 0.25 * (aL - aF), aTrim.LastParameter());
            Assert.Same(aS, ((Adaptor3d_IsoCurve)aTrim).Surface());
            Assert.True(aTrim.Value(aMid).IsEqual(anIsoC.Value(aMid), 0.0));
            Assert.Equal(Precision.Parametric(0.1), anIsoC.Resolution(0.1));
        }
    }

    [Fact]
    public void Types_AndConics()
    {
        foreach ((string Name, GeomAbs_CurveType U, GeomAbs_CurveType V) in new[]
                 {
                     ("plane", GeomAbs_Line, GeomAbs_Line), ("cylinder", GeomAbs_Line, GeomAbs_Circle),
                     ("cone", GeomAbs_Line, GeomAbs_Circle), ("sphere", GeomAbs_Circle, GeomAbs_Circle),
                     ("torus", GeomAbs_Circle, GeomAbs_Circle), ("bspline", GeomAbs_BSplineCurve, GeomAbs_BSplineCurve),
                     ("revolution", GeomAbs_BSplineCurve, GeomAbs_Circle), ("extrusion", GeomAbs_Line, GeomAbs_Circle),
                 })
        {
            TestSurfaceAdaptor aS = MakeSurface(Name);
            Adaptor3d_IsoCurve aUIso = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoU, 0.3 * aS.LastUParameter() + 0.7 * aS.FirstUParameter());
            Adaptor3d_IsoCurve aVIso = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoV, 0.6 * aS.LastVParameter() + 0.4 * aS.FirstVParameter());
            Assert.Equal(U, aUIso.GetType());
            Assert.Equal(V, aVIso.GetType());
            foreach (Adaptor3d_IsoCurve anIsoC in new[] { aUIso, aVIso })
            {
                double aF = anIsoC.FirstParameter(), aL = anIsoC.LastParameter();
                if (anIsoC.GetType() == GeomAbs_Circle)
                {
                    // Every point of the iso lies on the circle
                    gp_Circ aC = anIsoC.Circle();
                    for (int k = 0; k <= 8; k++)
                    {
                        gp_Pnt P = anIsoC.Value(aF + (aL - aF) * k / 8.0);
                        Assert.True(aC.Distance(P) < 1e-9, $"{Name} {anIsoC.Iso()} circle at {k}: {aC.Distance(P)}");
                    }
                }
                else if (anIsoC.GetType() == GeomAbs_Line)
                {
                    gp_Lin aLin = anIsoC.Line();
                    for (int k = 0; k <= 8; k++)
                    {
                        Assert.True(aLin.Distance(anIsoC.Value(aF + (aL - aF) * k / 8.0)) < 1e-9);
                    }
                }
            }
        }
    }

    [Fact]
    public void Queries_OnBSplineAndSweptSurfaces()
    {
        TestSurfaceAdaptor aBS = MakeSurface("bspline");
        Adaptor3d_IsoCurve aU = new Adaptor3d_IsoCurve(aBS, GeomAbs_IsoU, 0.2);
        Adaptor3d_IsoCurve aV = new Adaptor3d_IsoCurve(aBS, GeomAbs_IsoV, 1.0);
        Assert.Equal(3, aU.Degree());
        Assert.Equal(4, aU.NbPoles());
        Assert.Equal(5, aV.NbPoles());
        Assert.Equal(2, aU.NbKnots());
        Assert.Equal(3, aV.NbKnots());
        Assert.False(aU.IsRational());
        Assert.False(aU.IsPeriodic());
        Assert.False(aU.IsClosed());
        Assert.Equal(GeomAbs_C2, aU.Continuity());
        // Geom_BSplineSurface.UIso waits for unit 1A's Geom_BSplineCurve (decision 169)
        Assert.ThrowsAny<Standard_NotImplemented>(() => aU.BSpline());

        // Revolution: the U iso is the basis curve rotated; the V iso a circle
        TestSurfaceAdaptor aRev = MakeSurface("revolution");
        Adaptor3d_IsoCurve aRU = new Adaptor3d_IsoCurve(aRev, GeomAbs_IsoU, 1.1);
        Assert.Equal(3, aRU.Degree());
        Assert.Equal(5, aRU.NbPoles());
        Assert.Equal(3, aRU.NbKnots());
        Assert.False(aRU.IsRational());
        Geom_BSplineCurve aRot = aRU.BSpline();
        for (int k = 0; k <= 8; k++)
        {
            double t = k / 8.0;
            Assert.True(aRot.Value(t).IsEqual(aRU.Value(t), 1e-12));
        }
        Adaptor3d_IsoCurve aRV = new Adaptor3d_IsoCurve(aRev, GeomAbs_IsoV, 0.5);
        Assert.ThrowsAny<Standard_NoSuchObject>(() => aRV.Degree());
        Assert.True(aRV.IsPeriodic());
        Assert.Equal(2 * Math.PI, aRV.Period());

        // Extrusion: the V iso is the basis curve translated
        TestSurfaceAdaptor anExt = MakeSurface("extrusion");
        Adaptor3d_IsoCurve anEV = new Adaptor3d_IsoCurve(anExt, GeomAbs_IsoV, 1.5);
        gp_Circ aC = anEV.Circle();
        Assert.True(aC.Location().IsEqual(new gp_Pnt(0, 0, 0).Translated(1.5 * new gp_Vec(new gp_Dir(0.1, 0.2, 1.0))), 1e-12));
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anEV.Hyperbola());
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anEV.Parabola());
    }

    [Fact]
    public void NoneIso_Throws()
    {
        TestSurfaceAdaptor aS = MakeSurface("plane");
        Adaptor3d_IsoCurve anIsoC = new Adaptor3d_IsoCurve(aS);
        Assert.Equal(GeomAbs_NoneIso, anIsoC.Iso());
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anIsoC.Value(0.0));
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anIsoC.Continuity());
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anIsoC.NbIntervals(GeomAbs_C1));
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anIsoC.IsClosed());
        Assert.ThrowsAny<Standard_NoSuchObject>(() => anIsoC.Load(GeomAbs_NoneIso, 0.0));
        Assert.ThrowsAny<Standard_NoSuchObject>(() => new Adaptor3d_IsoCurve(MakeSurface("cylinder")).GetType());

        // Loading a surface resets the iso
        anIsoC.Load(GeomAbs_IsoU, 1.0);
        Assert.Equal(GeomAbs_IsoU, anIsoC.Iso());
        anIsoC.Load(aS);
        Assert.Equal(GeomAbs_NoneIso, anIsoC.Iso());
    }

    [Fact]
    public void Load_ClampsRange_AndAdjustsPeriodicParameters()
    {
        TestSurfaceAdaptor aS = MakeSurface("plane");
        Adaptor3d_IsoCurve anIsoC = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoU, 1.0, -10.0, 2.0);
        Assert.Equal(-4.0, anIsoC.FirstParameter());
        Assert.Equal(2.0, anIsoC.LastParameter());

        // Cylinder (U periodic): a U iso's parameter, or a V iso's range, is brought into [0, 2 Pi)
        TestSurfaceAdaptor aCyl = new TestSurfaceAdaptor(new Geom_CylindricalSurface(TheAx3, 2.0), -10, 10, -3, 3);
        Adaptor3d_IsoCurve aUIso = new Adaptor3d_IsoCurve(aCyl, GeomAbs_IsoU, 2 * Math.PI + 1.0);
        double aExpected = 2 * Math.PI + 1.0;
        double aDummy = aExpected;
        ElCLib.AdjustPeriodic(-10, -10 + 2 * Math.PI, Precision.Parametric(Precision.Confusion()), ref aExpected, ref aDummy);
        Assert.Equal(aExpected, aUIso.Parameter());
        Assert.True(aUIso.Parameter() >= -10 && aUIso.Parameter() < -10 + 2 * Math.PI);
        Assert.True(aUIso.Value(0.5).IsEqual(aCyl.Value(2 * Math.PI + 1.0, 0.5), 1e-12));

        Adaptor3d_IsoCurve aVIso = new Adaptor3d_IsoCurve(aCyl, GeomAbs_IsoV, 0.5, 1.0, 4.0);
        double aF = 1.0, aL = 4.0;
        ElCLib.AdjustPeriodic(-10, -10 + 2 * Math.PI, Precision.Parametric(Precision.Confusion()), ref aF, ref aL);
        Assert.Equal(aF, aVIso.FirstParameter());
        Assert.Equal(aL, aVIso.LastParameter());
    }

    [Fact]
    public void Intervals_RestrictedToTheRange()
    {
        // A surface with V interval bounds at -1, 0 and 2 inside [-4, 4]
        TestSurfaceAdaptor aS = new TestSurfaceAdaptor(new Geom_Plane(TheAx3), -5, 5, -4, 4) { VBreaks = [-1.0, 0.0, 2.0] };
        Adaptor3d_IsoCurve anIsoC = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoU, 1.0, -2.0, 1.0);
        Assert.Equal(3, anIsoC.NbIntervals(GeomAbs_C2));
        NCollection_Array1<double> T = new(1, 4);
        anIsoC.Intervals(T, GeomAbs_C2);
        Assert.Equal(new[] { -2.0, -1.0, 0.0, 1.0 }, new[] { T[1], T[2], T[3], T[4] });

        // A range on an interval bound
        Adaptor3d_IsoCurve anOnBound = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoU, 1.0, -1.0, 2.0);
        Assert.Equal(2, anOnBound.NbIntervals(GeomAbs_C2));
        NCollection_Array1<double> T2 = new(1, 3);
        anOnBound.Intervals(T2, GeomAbs_C2);
        Assert.Equal(new[] { -1.0, 0.0, 2.0 }, new[] { T2[1], T2[2], T2[3] });

        // One interval: the range
        Adaptor3d_IsoCurve aVIso = new Adaptor3d_IsoCurve(aS, GeomAbs_IsoV, 1.0, -2.0, 3.0);
        Assert.Equal(1, aVIso.NbIntervals(GeomAbs_C2));
        NCollection_Array1<double> T3 = new(1, 2);
        aVIso.Intervals(T3, GeomAbs_C2);
        Assert.Equal(new[] { -2.0, 3.0 }, new[] { T3[1], T3[2] });
    }
}
