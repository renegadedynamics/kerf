// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of GeomAdaptor (which has no GTests for these): the surfaces of revolution and extrusion
/// over curve adaptors, GeomAdaptor's MakeCurve and MakeSurface, Adaptor3d_HSurfaceTool, and the adaptors as
/// working objects (shallow copies with their own caches, reloading).
/// </summary>
public class GeomAdaptor_PortTests
{
    private static readonly Axis3D ZAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

    private static Geom_BSplineCurve Cubic()
    {
        var aPoles = NCollection_Array1<Point3D>.From([new Point3D(1, 0, 0), new Point3D(2, 0, 1), new Point3D(1.5, 0, 2),
                                                      new Point3D(3, 0, 3), new Point3D(2, 0, 4)]);
        var aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        var aMults = NCollection_Array1<int>.From([4, 1, 4]);
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    private static void AssertSameEval(Adaptor3d_Surface theA, Geom_Surface theS, double U, double V)
    {
        theA.D2(U, V, out Point3D aP, out Vector3D aDU, out Vector3D aDV, out Vector3D aDUU, out Vector3D aDVV, out Vector3D aDUV);
        theS.D2(U, V, out Point3D aSP, out Vector3D aSDU, out Vector3D aSDV, out Vector3D aSDUU, out Vector3D aSDVV, out Vector3D aSDUV);
        Assert.True(aP.IsEqual(aSP, 1e-12));
        Assert.True(aDU.IsEqual(aSDU, 1e-12, 1e-12));
        Assert.True(aDV.IsEqual(aSDV, 1e-12, 1e-12));
        Assert.True(aDUU.IsEqual(aSDUU, 1e-12, 1e-12));
        Assert.True(aDVV.IsEqual(aSDVV, 1e-12, 1e-12));
        Assert.True(aDUV.IsEqual(aSDUV, 1e-12, 1e-12));
    }

    [Fact]
    public void SurfaceOfRevolution_Recognises_Elementary_Surfaces()
    {
        // a line parallel to the axis: a cylinder
        var aParallel = new GeomAdaptor_Curve(new Geom_Line(new Axis3D(new Point3D(3, 0, 0), new Direction3D(0, 0, 1))), -2, 2);
        var aCyl = new GeomAdaptor_SurfaceOfRevolution(aParallel, ZAxis);
        Assert.Equal(SurfaceKind.Cylinder, aCyl.GetType());
        Assert.Equal(3.0, aCyl.Cylinder().Radius(), 12);

        // a line normal to the axis: a plane
        var aNormal = new GeomAdaptor_Curve(new Geom_Line(new Axis3D(new Point3D(1, 0, 2), new Direction3D(1, 0, 0))), 0, 5);
        var aPln = new GeomAdaptor_SurfaceOfRevolution(aNormal, ZAxis);
        Assert.Equal(SurfaceKind.Plane, aPln.GetType());
        Assert.Equal(2.0, aPln.Plane().Location().Z(), 12);

        // a line crossing the axis: a cone
        var aSlant = new GeomAdaptor_Curve(new Geom_Line(new Axis3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 1))), 0.5, 3);
        var aCone = new GeomAdaptor_SurfaceOfRevolution(aSlant, ZAxis);
        Assert.Equal(SurfaceKind.Cone, aCone.GetType());
        Assert.Equal(Math.PI / 4, Math.Abs(aCone.Cone().SemiAngle()), 12);

        // a circle centred on the axis, in a plane containing it: a sphere; off the axis: a torus
        var aMeridian = new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0));
        var aSph = new GeomAdaptor_SurfaceOfRevolution(new GeomAdaptor_Curve(new Geom_Circle(aMeridian, 2)), ZAxis);
        Assert.Equal(SurfaceKind.Sphere, aSph.GetType());
        Assert.Equal(2.0, aSph.Sphere().Radius(), 12);
        var aTorusMeridian = new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0));
        var aTor = new GeomAdaptor_SurfaceOfRevolution(new GeomAdaptor_Curve(new Geom_Circle(aTorusMeridian, 1)), ZAxis);
        Assert.Equal(SurfaceKind.Torus, aTor.GetType());
        Assert.Equal(5.0, aTor.Torus().MajorRadius(), 12);

        // a B-spline curve: a general surface of revolution, evaluated as Geom_SurfaceOfRevolution
        var aGen = new GeomAdaptor_SurfaceOfRevolution(new GeomAdaptor_Curve(Cubic()), ZAxis);
        Assert.Equal(SurfaceKind.SurfaceOfRevolution, aGen.GetType());
        Geom_SurfaceOfRevolution aGeom = new(Cubic(), ZAxis);
        AssertSameEval(aGen, aGeom, 1.2, 0.3);
        AssertSameEval(aGen, aGeom, 4.0, 0.75);
        Assert.Equal(0.0, aGen.FirstUParameter());
        Assert.Equal(2 * Math.PI, aGen.LastUParameter());
        Assert.Equal(1, aGen.NbVIntervals(Continuity.C2)); // C2 at its single knot
        Assert.Equal(2, aGen.NbVIntervals(Continuity.C3));
        Assert.Throws<OutOfRangeException>(() => aGen.UTrim(0.5, 1.0, 1e-7));
        Adaptor3d_Surface aVTrim = aGen.VTrim(0.2, 0.6, 1e-7);
        Assert.Equal(0.6, aVTrim.LastVParameter());
        Assert.Throws<NoSuchObjectException>(() => aGen.BSpline());
    }

    [Fact]
    public void SurfaceOfLinearExtrusion_Recognises_Planes_And_Cylinders()
    {
        var aCircle = new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aCyl = new GeomAdaptor_SurfaceOfLinearExtrusion(new GeomAdaptor_Curve(aCircle), new Direction3D(0, 0, -1));
        Assert.Equal(SurfaceKind.Cylinder, aCyl.GetType());
        Assert.Equal(-1.0, aCyl.Cylinder().Position().Direction().Z(), 12); // reversed to the direction

        var aLine = new Geom_Line(new Axis3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        var aPln = new GeomAdaptor_SurfaceOfLinearExtrusion(new GeomAdaptor_Curve(aLine), new Direction3D(0, 1, 0));
        Assert.Equal(SurfaceKind.Plane, aPln.GetType());
        Assert.Equal(1.0, Math.Abs(aPln.Plane().Axis().Direction().Z()), 12);

        var aGen = new GeomAdaptor_SurfaceOfLinearExtrusion(new GeomAdaptor_Curve(Cubic()), new Direction3D(0, 1, 1));
        Assert.Equal(SurfaceKind.SurfaceOfExtrusion, aGen.GetType());
        Geom_SurfaceOfLinearExtrusion aGeom = new(Cubic(), new Direction3D(0, 1, 1));
        AssertSameEval(aGen, aGeom, 0.3, -2.0);
        AssertSameEval(aGen, aGeom, 0.5, 7.0);
        Assert.Equal(RealFirst(), aGen.FirstVParameter());
        Assert.Throws<DomainErrorException>(() => aGen.VPeriod());
        Assert.Equal(3, aGen.UDegree());

        // OCCT (occt-issues.md): VTrim trims the curve, which carries U, and UTrim trims nothing
        Adaptor3d_Surface aVTrim = aGen.VTrim(0.2, 0.6, 1e-7);
        Assert.Equal((0.2, 0.6), (aVTrim.FirstUParameter(), aVTrim.LastUParameter()));
        Assert.Equal(RealLast(), aVTrim.LastVParameter());
        Adaptor3d_Surface aUTrim = aGen.UTrim(0.2, 0.6, 1e-7);
        Assert.Equal((0.0, 1.0), (aUTrim.FirstUParameter(), aUTrim.LastUParameter()));
    }

    [Fact]
    public void MakeCurve_And_MakeSurface_Rebuild_Trimmed_Geometry()
    {
        Geom_BSplineCurve aC = Cubic();
        Geom_Curve aWhole = GeomAdaptor.MakeCurve(new GeomAdaptor_Curve(aC));
        Geom_BSplineCurve aCopy = Assert.IsType<Geom_BSplineCurve>(aWhole);
        Assert.NotSame(aC, aCopy);
        Geom_TrimmedCurve aTrim = Assert.IsType<Geom_TrimmedCurve>(GeomAdaptor.MakeCurve(new GeomAdaptor_Curve(aC, 0.2, 0.7)));
        Assert.Equal(0.2, aTrim.FirstParameter());
        Assert.Equal(0.7, aTrim.LastParameter());

        var aCyl = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 2);
        Geom_Surface aS = GeomAdaptor.MakeSurface(new GeomAdaptor_Surface(aCyl, 0, 1, -1, 1));
        Geom_RectangularTrimmedSurface aT = Assert.IsType<Geom_RectangularTrimmedSurface>(aS);
        Assert.IsType<Geom_CylindricalSurface>(aT.BasisSurface());
        Assert.IsType<Geom_CylindricalSurface>(GeomAdaptor.MakeSurface(new GeomAdaptor_Surface(aCyl, 0, 1, -1, 1), false));

        var anOff = new Geom_OffsetSurface(aCyl, 0.5);
        // an offset of the trimmed basis, whose bounds are already the adaptor's
        var anOffS = Assert.IsType<Geom_OffsetSurface>(GeomAdaptor.MakeSurface(new GeomAdaptor_Surface(anOff, 0, 1, -1, 1)));
        Assert.IsType<Geom_RectangularTrimmedSurface>(anOffS.BasisSurface());
        Assert.True(anOffS.Value(0.5, 0.5).IsEqual(anOff.Value(0.5, 0.5), 1e-12));

        Assert.Throws<DomainErrorException>(() => GeomAdaptor.MakeCurve(new GeomAdaptor_Curve(new Geom_OffsetCurve(aC, 1, new Direction3D(0, 1, 0)))));
    }

    [Fact]
    public void HSurfaceTool_Samples_And_G1()
    {
        var aPlane = new GeomAdaptor_Surface(new Geom_Plane(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1))), 0, 1, 0, 1);
        Assert.Equal(2, Adaptor3d_HSurfaceTool.NbSamplesU(aPlane));
        var aTorus = new GeomAdaptor_Surface(new Geom_ToroidalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 5, 1));
        Assert.Equal(20, Adaptor3d_HSurfaceTool.NbSamplesU(aTorus));
        Assert.Equal(15, Adaptor3d_HSurfaceTool.NbSamplesV(aTorus));
        Assert.Equal(5, Adaptor3d_HSurfaceTool.NbSamplesU(aTorus, 0, Math.PI)); // 20 * (int)0.5 = 0, raised to 5
        Assert.Equal(SurfaceKind.Torus, Adaptor3d_HSurfaceTool.GetType(aTorus));

        // a revolution of a B-spline curve with a G1 corner is G1 along V, and not along U (no B-spline there)
        var aRev = new GeomAdaptor_Surface(new Geom_SurfaceOfRevolution(Cubic(), ZAxis));
        Assert.True(Adaptor3d_HSurfaceTool.IsSurfG1(aRev, false));
        Assert.False(Adaptor3d_HSurfaceTool.IsSurfG1(aRev, true));
        Assert.False(Adaptor3d_HSurfaceTool.IsSurfG1(aPlane, true));
    }

    [Fact]
    public void ShallowCopy_Has_Its_Own_Cache_And_Reload_Drops_It()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, (i * j) % 3);
            }
        }
        var aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        var aMults = NCollection_Array1<int>.From([3, 1, 3]);
        Geom_BSplineSurface aS = new(aPoles, aKnots, aKnots, aMults, aMults, 2, 2);
        var anA = new GeomAdaptor_Surface(aS);
        Point3D aP1 = anA.Value(0.2, 0.3);
        var aCopy = (GeomAdaptor_Surface)anA.ShallowCopy();
        Assert.Same(anA.Surface(), aCopy.Surface());
        Assert.True(aCopy.Value(0.8, 0.9).IsEqual(aS.Value(0.8, 0.9), 1e-12));
        Assert.True(anA.Value(0.2, 0.3).IsEqual(aP1, 0.0));

        // the same surface reloaded with other bounds: the cache is rebuilt; another surface replaces it
        anA.Load(aS, 0.1, 0.9, 0.1, 0.9);
        Assert.True(anA.Value(0.75, 0.25).IsEqual(aS.Value(0.75, 0.25), 1e-12));
        anA.Load(new Geom_Plane(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1))));
        Assert.Equal(SurfaceKind.Plane, anA.GetType());

        var aCurve = new GeomAdaptor_Curve(Cubic());
        aCurve.Reset();
        Assert.Equal(CurveKind.OtherCurve, aCurve.GetType());
        Assert.Null(aCurve.Curve());
    }
}
