// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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

    private static readonly CoordinateSystem Axis = new(new Point3D(0.5, 0.5, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    // A bicubic patch over [0, 1] x [0, 1], z = 0.2 + 0.1 x with a bump in the middle.
    private static SurfaceAdaptor Patch()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) / 3.0, y = (j - 1) / 3.0;
                double aBump = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.05 : 0.0;
                aPoles[i, j] = new Point3D(x, y, 0.2 + 0.1 * x + aBump);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new SurfaceAdaptor(new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    // A wavy wall: a cubic B-spline curve in the plane z = 0 extruded along Z, v in [-1, 1].
    private static SurfaceAdaptor Wall()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(-1, 0.3, 0);
        aPoles[2] = new Point3D(0, 0.7, 0);
        aPoles[3] = new Point3D(1, 0.2, 0);
        aPoles[4] = new Point3D(2, 0.6, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        var aCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        return new SurfaceAdaptor(new ExtrusionSurface(aCurve, new Direction3D(0, 0, 1)), 0, 1, -1, 1);
    }

    private static SurfaceAdaptor Plane(double z) =>
        new(new PlaneSurface(new Point3D(0, 0, z), new Direction3D(0, 0, 1)), -2, 2, -2, 2);

    private static SurfaceAdaptor Cylinder() => new(new CylindricalSurface(Axis, 0.35), 0, 2 * M_PI, -1, 2);

    private static SurfaceAdaptor Sphere() =>
        new(new SphericalSurface(new CoordinateSystem(new Point3D(0.5, 0.5, 0.25), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)),
                                      0.3));

    private static SurfaceAdaptor Cone() => new(new ConicalSurface(Axis, M_PI / 6, 0.2), 0, 2 * M_PI, -1, 2);

    private static ImpPrmIntersection Run(SurfaceAdaptor theS1, SurfaceAdaptor theS2) =>
        new(theS1, new TopolTool(theS1), theS2, new TopolTool(theS2), TolArc, TolTang, Fleche, Pas);

    // Every point of every walking line on both surfaces; end vertices present.
    private static void CheckLines(ImpPrmIntersection theInt, SurfaceAdaptor theS1,
                                   SurfaceAdaptor theS2, double theTol)
    {
        Assert.True(theInt.IsDone);
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            PatchLine aLine = theInt.Line(i);
            Assert.True(aLine.ArcType() == PatchLineType.Walking || aLine.ArcType() == PatchLineType.Restriction);
            if (aLine is not WLine aWL)
            {
                continue;
            }
            Assert.True(aWL.NbPnts() >= 2);
            Assert.True(aWL.VertexCount >= 2);
            Assert.True(aWL.HasFirstPoint && aWL.HasLastPoint);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                PntOn2S aP = aWL.Point(k);
                aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).DistanceTo(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).DistanceTo(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.VertexCount; k++)
            {
                PatchPoint aV = aWL.Vertex(k);
                double aPrm = aV.ParameterOnLine();
                Assert.InRange(aPrm, 1.0, aWL.NbPnts());
            }
        }
    }

    // All the doubles of the result, as bits, for the determinism tests.
    private static List<long> Bits(ImpPrmIntersection theInt)
    {
        List<long> aBits = [theInt.LineCount];
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            PatchLine aLine = theInt.Line(i);
            aBits.Add((long)aLine.ArcType());
            if (aLine is PointLine aPL)
            {
                for (int k = 1; k <= aPL.NbPnts(); k++)
                {
                    aPL.Point(k).Parameters(out double u1, out double v1, out double u2, out double v2);
                    Point3D aP = aPL.Point(k).Value();
                    foreach (double d in new[] { aP.X, aP.Y, aP.Z, u1, v1, u2, v2 })
                    {
                        aBits.Add(BitConverter.DoubleToInt64Bits(d));
                    }
                }
                for (int k = 1; k <= aPL.VertexCount; k++)
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
        SurfaceAdaptor aPln = Plane(5.0), aPatch = Patch();
        ImpPrmIntersection anInt = Run(aPln, aPatch);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.IsEmpty);
        Assert.Equal(0, anInt.LineCount);
        Assert.Equal(0, anInt.PntCount);
    }

    [Fact]
    public void NoQuadric_Throws()
    {
        SurfaceAdaptor aTorus = new(new ToroidalSurface(Axis, 1.0, 0.2));
        ImpPrmIntersection anInt = new();
        EngineAssert.Throws<ConstructionErrorException>(() =>
            anInt.Perform(aTorus, new TopolTool(aTorus), Patch(), new TopolTool(Patch()), TolArc,
                          TolTang, Fleche, Pas));
        Assert.False(anInt.IsDone);
        EngineAssert.Throws<NotDoneException>(() => anInt.LineCount);
    }

    [Fact]
    public void Plane_BSpline_OpenLine()
    {
        SurfaceAdaptor aPln = Plane(0.26), aPatch = Patch();
        ImpPrmIntersection anInt = Run(aPln, aPatch);
        CheckLines(anInt, aPln, aPatch, 1e-5);
        Assert.False(anInt.IsEmpty);
        Assert.True(anInt.LineCount >= 1);
    }

    [Fact]
    public void BSpline_Plane_Reversed()
    {
        // The quadric second: the lines' first parameters are the patch's.
        SurfaceAdaptor aPln = Plane(0.26), aPatch = Patch();
        ImpPrmIntersection anInt = Run(aPatch, aPln);
        CheckLines(anInt, aPatch, aPln, 1e-5);
        Assert.True(anInt.LineCount >= 1);
    }

    [Fact]
    public void Cylinder_BSpline_ClosedLine()
    {
        SurfaceAdaptor aCyl = Cylinder(), aPatch = Patch();
        ImpPrmIntersection anInt = Run(aCyl, aPatch);
        CheckLines(anInt, aCyl, aPatch, 1e-5);
        Assert.True(anInt.LineCount >= 1);
        for (int i = 1; i <= anInt.LineCount; i++)
        {
            WLine aWL = Assert.IsType<WLine>(anInt.Line(i));
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                Point3D aP = aWL.Point(k).Value();
                double r = Math.Sqrt((aP.X - 0.5) * (aP.X - 0.5) + (aP.Y - 0.5) * (aP.Y - 0.5));
                Assert.Equal(0.35, r, 6);
            }
        }
    }

    [Fact]
    public void Sphere_BSpline()
    {
        SurfaceAdaptor aSph = Sphere(), aPatch = Patch();
        ImpPrmIntersection anInt = Run(aSph, aPatch);
        CheckLines(anInt, aSph, aPatch, 1e-5);
        Assert.True(anInt.LineCount >= 1);
    }

    [Fact]
    public void Cone_Extrusion()
    {
        SurfaceAdaptor aCone = Cone(), aWall = Wall();
        ImpPrmIntersection anInt = Run(aCone, aWall);
        CheckLines(anInt, aCone, aWall, 1e-5);
        Assert.True(anInt.LineCount >= 1);
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
