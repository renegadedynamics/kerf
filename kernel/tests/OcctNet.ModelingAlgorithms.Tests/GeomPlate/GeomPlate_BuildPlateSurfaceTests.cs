// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTest GeomPlate_BuildPlateSurface (names kept) and unit tests of the builder.</summary>
public class GeomPlate_BuildPlateSurfaceTests
{
    // ---- OCCT's GTests (src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomPlate_BuildPlateSurface_Test.cxx) ----

    [Fact]
    public void OCC525_PerformWithoutConstraints()
    {
        var aBuilder = new BuildPlateSurface();
        aBuilder.Perform();

        // OCCT's TODO: IsDone() returns true due to Plate_Plate::Init() setting OK=true.
        Assert.True(aBuilder.IsDone);
        Assert.Null(aBuilder.Surface());
    }

    [Fact]
    public void Perform_ClearsStaleResult()
    {
        var aBuilder = new BuildPlateSurface(3, 10, 3, 1.0e-5, 1.0e-4, 0.01, 0.1, false);

        aBuilder.Add(new PointConstraint(new Point3D(0, 0, 0), 0));
        aBuilder.Add(new PointConstraint(new Point3D(1, 0, 0), 0));
        aBuilder.Add(new PointConstraint(new Point3D(0, 1, 0), 0));
        aBuilder.Add(new PointConstraint(new Point3D(1, 1, 0.1), 0));
        aBuilder.Perform();

        aBuilder.Init();
        aBuilder.Perform();
        Assert.True(aBuilder.IsDone);
        Assert.Null(aBuilder.Surface());
    }

    // ---- unit tests ----

    private static CurveAdaptorBase Segment(in Point3D P1, in Point3D P2)
    {
        var aLine = new MakeSegment(P1, P2).Value()!;
        return new CurveAdaptor(aLine);
    }

    // A closed contour of four curves: three segments and an arc bulging up, around the unit square.
    private static CurveAdaptorBase[] Contour()
    {
        var p1 = new Point3D(0, 0, 0);
        var p2 = new Point3D(10, 0, 0);
        var p3 = new Point3D(10, 10, 0);
        var p4 = new Point3D(0, 10, 0);
        var anArc = new MakeArcOfCircle(p3, new Point3D(5, 10, 3), p4).Value()!;
        return [Segment(p1, p2), Segment(p2, p3), new CurveAdaptor(anArc), Segment(p4, p1)];
    }

    [Fact]
    public void Perform_on_a_contour_passes_through_its_curves()
    {
        var aBuilder = new BuildPlateSurface(3, 15, 2);
        foreach (CurveAdaptorBase aCurve in Contour())
        {
            aBuilder.Add(new CurveConstraint(aCurve, 0));
        }
        aBuilder.Perform();
        Assert.True(aBuilder.IsDone);
        PlateSurface aSurf = aBuilder.Surface()!;
        Assert.True(aSurf.IsFrozen);
        Assert.True(aBuilder.G0Error() < 1.0e-2, $"G0 {aBuilder.G0Error()}");
        Assert.Equal(0.0, aBuilder.G1Error());

        // the 2d curves map onto the 3d curves through the plate surface
        NCollection_HArray1<Curve2D?> aCurves2d = aBuilder.Curves2d();
        CurveAdaptorBase[] aContour = Contour();
        NCollection_HArray1<int> anOrder = aBuilder.Order();
        Assert.Equal(4, aCurves2d.Length());
        for (int i = 1; i <= 4; i++)
        {
            Curve2D aC2d = aCurves2d.Value(i)!;
            Assert.True(aC2d.IsFrozen);
            CurveAdaptorBase aC3d = aContour[i - 1];
            for (int k = 0; k <= 10; k++)
            {
                double u = aC3d.FirstParameter() + (aC3d.LastParameter() - aC3d.FirstParameter()) * k / 10;
                Point2D aUV = aC2d.Value(u);
                Assert.True(aSurf.Value(aUV.X(), aUV.Y()).Distance(aC3d.Value(u)) < 2.0e-2);
            }
            Assert.InRange(anOrder.Value(i), 1, 4);
        }

        // G0Error(Index) and the approximation
        // OCCT's bug: G0Error(Index) sizes its arrays by NbPtsOnCur (15), but the curves have more points now
        Assert.True(aBuilder.CurveConstraint(1).PointCount > 15);
        Assert.Throws<OutOfRangeException>(() => aBuilder.G0Error(1));
        var aSeq2d = new NCollection_Sequence<Xy>();
        var aSeq3d = new NCollection_Sequence<Xyz>();
        aBuilder.Disc2dContour(4, aSeq2d);
        aBuilder.Disc3dContour(4, 0, aSeq3d);
        Assert.Equal(aSeq2d.Length(), aSeq3d.Length());
        var aCrit = new PlateG0Criterion(aSeq2d, aSeq3d, 1.0e-3);
        var anApprox = new MakeApprox(aSurf, aCrit, 1.0e-4, 9, 8);
        BSplineSurface aBSpl = anApprox.Surface()!;
        Assert.True(aBSpl.IsFrozen);
        Assert.True(anApprox.ApproxError() < 5.0e-2, $"{anApprox.ApproxError()}");
        Assert.True(anApprox.CriterionError() < 1.0e-2, $"{anApprox.CriterionError()}");
        for (int i = 1; i <= aSeq2d.Length(); i++)
        {
            Xy aUV = aSeq2d.Value(i);
            Assert.True(aBSpl.Value(aUV.X(), aUV.Y()).Distance(new Point3D(aSeq3d.Value(i))) < 2.0e-2);
        }
    }

    [Fact]
    public void Perform_with_points_only_interpolates_them()
    {
        var aBuilder = new BuildPlateSurface(3, 10, 3);
        Point3D[] aPoints = [new(0, 0, 0), new(4, 0, 1), new(0, 4, -1), new(4, 4, 0.5), new(2, 2, 2)];
        foreach (Point3D aP in aPoints)
        {
            aBuilder.Add(new PointConstraint(aP, 0));
        }
        aBuilder.Perform();
        Assert.True(aBuilder.IsDone);
        PlateSurface aSurf = aBuilder.Surface()!;
        for (int i = 1; i <= aPoints.Length; i++)
        {
            Point2D aUV = aBuilder.PointConstraint(i).Pnt2dOnSurf();
            Assert.True(aSurf.Value(aUV.X(), aUV.Y()).Distance(aPoints[i - 1]) < 1.0e-6);
        }
        // the initial surface is the average plane, trimmed
        Assert.IsType<TrimmedSurface>(aBuilder.SurfInit());
    }

    [Fact]
    public void Perform_with_a_given_initial_surface_keeps_it()
    {
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        var aBuilder = new BuildPlateSurface(aPlane, 3, 10, 3);
        foreach (CurveAdaptorBase aCurve in Contour())
        {
            aBuilder.Add(new CurveConstraint(aCurve, 0));
        }
        aBuilder.Perform();
        Assert.True(aBuilder.IsDone);
        Assert.Same(aPlane, aBuilder.SurfInit());
        Assert.Same(aPlane, aBuilder.Surface()!.CallSurfinit());
        Assert.True(aBuilder.G0Error() < 1.0e-2);
        NCollection_HArray1<int> aSense = aBuilder.Sense();
        Assert.Equal(4, aSense.Length());
    }

    [Fact]
    public void The_old_constructor_checks_its_arguments()
    {
        var aCurves = new NCollection_HArray1<CurveAdaptorBase>(1, 1);
        aCurves.SetValue(1, Contour()[0]);
        var aTang = new NCollection_HArray1<int>(1, 1, 0);
        var aNoPoints = new NCollection_HArray1<int>(1, 1, 0);
        Assert.Throws<ConstructionErrorException>(() => new BuildPlateSurface(aNoPoints, aCurves, aTang, 3));
        var aPoints = new NCollection_HArray1<int>(1, 1, 10);
        Assert.Throws<ConstructionErrorException>(() => new BuildPlateSurface(aPoints, aCurves, aTang, 1));
        Assert.Throws<ConstructionErrorException>(() => new BuildPlateSurface(aPoints, aCurves, aTang, 3, 0));
        Assert.Throws<ConstructionErrorException>(() => new BuildPlateSurface(3, 10, 0));
        Assert.Throws<ConstructionErrorException>(() => new BuildPlateSurface(1));
        var aBuilder = new BuildPlateSurface(aPoints, aCurves, aTang, 3);
        Assert.Equal(10, aBuilder.CurveConstraint(1).PointCount);
    }

    [Fact]
    public void Perform_with_tangency_to_a_cylinder_meets_it_G1()
    {
        // a cylinder's half seen from above; the filling closes the strip between its two straight edges with G1
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 5.0);
        var aSurf = new SurfaceAdaptor(aCyl);
        // the iso v = 0 on u in [0, pi] of the cylinder z = 5 cos, y = 5 sin: the edge at x = 0 and x = 10
        var aBuilder = new BuildPlateSurface(3, 15, 3);
        var aL1 = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(1, 0));
        var aC1 = new CurveOnSurface(new CurveAdaptor2D(aL1, 0.0, M_PI), aSurf);
        var aL2 = new LineCurve2D(new Point2D(M_PI, 10.0), new Direction2D(-1, 0));
        var aC2 = new CurveOnSurface(new CurveAdaptor2D(aL2, 0.0, M_PI), aSurf);
        aBuilder.Add(new CurveConstraint(aC1, 1));
        aBuilder.Add(new CurveConstraint(Segment(aC1.Value(M_PI), aC2.Value(0.0)), 0));
        aBuilder.Add(new CurveConstraint(aC2, 1));
        aBuilder.Add(new CurveConstraint(Segment(aC2.Value(M_PI), aC1.Value(0.0)), 0));
        aBuilder.Perform();
        Assert.True(aBuilder.IsDone);
        Assert.True(aBuilder.G0Error() < 1.0e-2, $"G0 {aBuilder.G0Error()}");
        Assert.True(aBuilder.G1Error() < 0.1, $"G1 {aBuilder.G1Error()}");
    }
}
