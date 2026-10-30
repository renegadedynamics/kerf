// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_ImpPrmIntersection: a plane, a cylinder, a sphere and a cone against a B-spline patch and a surface of
/// linear extrusion. Every point of every walking line lies on both surfaces (each at its own parameters), the lines
/// have their end vertices, and the result is the same, bit for bit, on every run and from parallel threads
/// (decision 490). The bit-for-bit comparison with native OCCT is 5G's (GeomAPI_IntSS).
/// </summary>
public class IntPatchImpPrmIntersectionTests
{
    private const double TolArc = 1e-7;
    private const double TolTang = 1e-6;
    private const double Fleche = 0.01;
    private const double Pas = 0.05;

    private static readonly gp_Ax3 Axis = new(new gp_Pnt(0.5, 0.5, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0));

    // A bicubic patch over [0, 1] x [0, 1], z = 0.2 + 0.1 x with a bump in the middle.
    private static GeomAdaptor_Surface Patch()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) / 3.0, y = (j - 1) / 3.0;
                double aBump = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.05 : 0.0;
                aPoles[i, j] = new gp_Pnt(x, y, 0.2 + 0.1 * x + aBump);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new GeomAdaptor_Surface(new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    // A wavy wall: a cubic B-spline curve in the plane z = 0 extruded along Z, v in [-1, 1].
    private static GeomAdaptor_Surface Wall()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(-1, 0.3, 0);
        aPoles[2] = new gp_Pnt(0, 0.7, 0);
        aPoles[3] = new gp_Pnt(1, 0.2, 0);
        aPoles[4] = new gp_Pnt(2, 0.6, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        var aCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        return new GeomAdaptor_Surface(new Geom_SurfaceOfLinearExtrusion(aCurve, new gp_Dir(0, 0, 1)), 0, 1, -1, 1);
    }

    private static GeomAdaptor_Surface Plane(double z) =>
        new(new Geom_Plane(new gp_Pnt(0, 0, z), new gp_Dir(0, 0, 1)), -2, 2, -2, 2);

    private static GeomAdaptor_Surface Cylinder() => new(new Geom_CylindricalSurface(Axis, 0.35), 0, 2 * M_PI, -1, 2);

    private static GeomAdaptor_Surface Sphere() =>
        new(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0.5, 0.5, 0.25), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)),
                                      0.3));

    private static GeomAdaptor_Surface Cone() => new(new Geom_ConicalSurface(Axis, M_PI / 6, 0.2), 0, 2 * M_PI, -1, 2);

    private static IntPatch_ImpPrmIntersection Run(GeomAdaptor_Surface theS1, GeomAdaptor_Surface theS2) =>
        new(theS1, new Adaptor3d_TopolTool(theS1), theS2, new Adaptor3d_TopolTool(theS2), TolArc, TolTang, Fleche, Pas);

    // Every point of every walking line on both surfaces; end vertices present.
    private static void CheckLines(IntPatch_ImpPrmIntersection theInt, GeomAdaptor_Surface theS1,
                                   GeomAdaptor_Surface theS2, double theTol)
    {
        Assert.True(theInt.IsDone());
        for (int i = 1; i <= theInt.NbLines(); i++)
        {
            IntPatch_Line aLine = theInt.Line(i);
            Assert.True(aLine.ArcType() == IntPatch_Walking || aLine.ArcType() == IntPatch_Restriction);
            if (aLine is not IntPatch_WLine aWL)
            {
                continue;
            }
            Assert.True(aWL.NbPnts() >= 2);
            Assert.True(aWL.NbVertex() >= 2);
            Assert.True(aWL.HasFirstPoint() && aWL.HasLastPoint());
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                IntSurf_PntOn2S aP = aWL.Point(k);
                aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.NbVertex(); k++)
            {
                IntPatch_Point aV = aWL.Vertex(k);
                double aPrm = aV.ParameterOnLine();
                Assert.InRange(aPrm, 1.0, aWL.NbPnts());
            }
        }
    }

    // All the doubles of the result, as bits, for the determinism tests.
    private static List<long> Bits(IntPatch_ImpPrmIntersection theInt)
    {
        List<long> aBits = [theInt.NbLines()];
        for (int i = 1; i <= theInt.NbLines(); i++)
        {
            IntPatch_Line aLine = theInt.Line(i);
            aBits.Add((long)aLine.ArcType());
            if (aLine is IntPatch_PointLine aPL)
            {
                for (int k = 1; k <= aPL.NbPnts(); k++)
                {
                    aPL.Point(k).Parameters(out double u1, out double v1, out double u2, out double v2);
                    gp_Pnt aP = aPL.Point(k).Value();
                    foreach (double d in new[] { aP.X(), aP.Y(), aP.Z(), u1, v1, u2, v2 })
                    {
                        aBits.Add(BitConverter.DoubleToInt64Bits(d));
                    }
                }
                for (int k = 1; k <= aPL.NbVertex(); k++)
                {
                    aBits.Add(BitConverter.DoubleToInt64Bits(aPL.Vertex(k).ParameterOnLine()));
                }
            }
        }
        return aBits;
    }

    [Fact]
    public void FarPlane_IsEmpty()
    {
        // No restriction point and no interior point: the walking does not run.
        GeomAdaptor_Surface aPln = Plane(5.0), aPatch = Patch();
        IntPatch_ImpPrmIntersection anInt = Run(aPln, aPatch);
        Assert.True(anInt.IsDone());
        Assert.True(anInt.IsEmpty());
        Assert.Equal(0, anInt.NbLines());
        Assert.Equal(0, anInt.NbPnts());
    }

    [Fact]
    public void NoQuadric_Throws()
    {
        GeomAdaptor_Surface aTorus = new(new Geom_ToroidalSurface(Axis, 1.0, 0.2));
        IntPatch_ImpPrmIntersection anInt = new();
        Assert.Throws<Standard_ConstructionError>(() =>
            anInt.Perform(aTorus, new Adaptor3d_TopolTool(aTorus), Patch(), new Adaptor3d_TopolTool(Patch()), TolArc,
                          TolTang, Fleche, Pas));
        Assert.False(anInt.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anInt.NbLines());
    }

    [Fact]
    public void Plane_BSpline_OpenLine()
    {
        GeomAdaptor_Surface aPln = Plane(0.26), aPatch = Patch();
        IntPatch_ImpPrmIntersection anInt = Run(aPln, aPatch);
        CheckLines(anInt, aPln, aPatch, 1e-5);
        Assert.False(anInt.IsEmpty());
        Assert.True(anInt.NbLines() >= 1);
    }

    [Fact]
    public void BSpline_Plane_Reversed()
    {
        // The quadric second: the lines' first parameters are the patch's.
        GeomAdaptor_Surface aPln = Plane(0.26), aPatch = Patch();
        IntPatch_ImpPrmIntersection anInt = Run(aPatch, aPln);
        CheckLines(anInt, aPatch, aPln, 1e-5);
        Assert.True(anInt.NbLines() >= 1);
    }

    [Fact]
    public void Cylinder_BSpline_ClosedLine()
    {
        GeomAdaptor_Surface aCyl = Cylinder(), aPatch = Patch();
        IntPatch_ImpPrmIntersection anInt = Run(aCyl, aPatch);
        CheckLines(anInt, aCyl, aPatch, 1e-5);
        Assert.True(anInt.NbLines() >= 1);
        for (int i = 1; i <= anInt.NbLines(); i++)
        {
            IntPatch_WLine aWL = Assert.IsType<IntPatch_WLine>(anInt.Line(i));
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                gp_Pnt aP = aWL.Point(k).Value();
                double r = Math.Sqrt((aP.X() - 0.5) * (aP.X() - 0.5) + (aP.Y() - 0.5) * (aP.Y() - 0.5));
                Assert.Equal(0.35, r, 6);
            }
        }
    }

    [Fact]
    public void Sphere_BSpline()
    {
        GeomAdaptor_Surface aSph = Sphere(), aPatch = Patch();
        IntPatch_ImpPrmIntersection anInt = Run(aSph, aPatch);
        CheckLines(anInt, aSph, aPatch, 1e-5);
        Assert.True(anInt.NbLines() >= 1);
    }

    [Fact]
    public void Cone_Extrusion()
    {
        GeomAdaptor_Surface aCone = Cone(), aWall = Wall();
        IntPatch_ImpPrmIntersection anInt = Run(aCone, aWall);
        CheckLines(anInt, aCone, aWall, 1e-5);
        Assert.True(anInt.NbLines() >= 1);
    }

    [Fact]
    public void SameResult_EveryRun_AndInParallel()
    {
        List<long> aRef = Bits(Run(Cylinder(), Patch()));
        Assert.True(aRef.Count > 1);
        Assert.Equal(aRef, Bits(Run(Cylinder(), Patch())));

        var aResults = new List<long>[8];
        Parallel.For(0, aResults.Length, i => aResults[i] = Bits(Run(Cylinder(), Patch())));
        foreach (List<long> aBits in aResults)
        {
            Assert.Equal(aRef, aBits);
        }
    }
}
