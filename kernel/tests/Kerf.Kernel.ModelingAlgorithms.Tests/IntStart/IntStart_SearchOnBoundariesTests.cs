// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntStart's boundary search (IntPatch_TheSOnBounds) on the lateral faces of a cylinder (radius 1, height 2) and a
/// sphere (radius 2) cut by planes: the points it finds on the arcs (new points or vertices of the face), the arcs
/// that lie on the plane (segments), and the transitions that IntPatch_ImpPrmIntersection computes from them with
/// IntSurf.MakeTransition (the line's tangent from IntPatch_TheSurfFunction, the arc's tangent, the surface normal).
/// </summary>
/// <remarks>
/// The arcs are the face's edges in wire order: for the cylinder, the top circle (0, 2) to (2π, 2), the seam at u = 2π,
/// the bottom circle (0, 0) to (2π, 0) and the seam at u = 0 (both seams' pcurves go up); for the sphere, the north
/// pole's degenerated edge, the seam at u = 2π, the south pole's and the seam at u = 0, the meridians parametrised by
/// v + 2π. A point found on an arc is new (tolerance 1000 × TolBoundary) or a vertex of the face (tolerance
/// TolBoundary). OCCT's own GTests don't cover IntStart; these values follow OCCT's algorithm (oracle: indirect, 5G).
/// </remarks>
public class IntStart_SearchOnBoundariesTests
{
    private const double TolBoundary = 1e-7;
    private const double TolTangency = 1e-6;

    private sealed record Search(TheSOnBounds Result, FaceSurfaceAdaptor Surface, BrepTopolTool Domain,
                                 global::Kerf.Engine.Intersection.Walking.SurfaceQuadric Quadric, List<Curve2DAdaptorBase> Arcs);

    private static Search Run(Face theFace, Plane3D thePlane)
    {
        FaceSurfaceAdaptor aSurf = new(theFace);
        BrepTopolTool aDomain = new(aSurf);
        global::Kerf.Engine.Intersection.Walking.SurfaceQuadric aQuad = new(thePlane);
        PatchArcFunction aFunc = new();
        aFunc.SetQuadric(aQuad);
        aFunc.Set(aSurf);
        TheSOnBounds aSearch = new();
        aSearch.Perform(aFunc, aDomain, TolBoundary, TolTangency);
        Assert.True(aSearch.IsDone);

        List<Curve2DAdaptorBase> anArcs = [];
        aDomain.Init();
        while (aDomain.More())
        {
            anArcs.Add(aDomain.Value());
            aDomain.Next();
        }
        return new Search(aSearch, aSurf, aDomain, aQuad, anArcs);
    }

    private static Face Cylinder() => new ApiMakeCylinder(1, 2).Face();

    private static Face Sphere() => new MakeSphere(2).Face();

    private static Plane3D Plane(double x, double y, double z, double nx, double ny, double nz) =>
        new(new Point3D(x, y, z), new Direction3D(nx, ny, nz));

    // (arc rank in wire order, parameter, is new, tolerance) of every point, in the order found
    private static (int Arc, double Param, bool IsNew, double Tol)[] Points(Search theSearch)
        => Enumerable.Range(1, theSearch.Result.PointCount)
                     .Select(i => theSearch.Result.Point(i))
                     .Select(p => (theSearch.Arcs.FindIndex(a => ReferenceEquals(a, p.Arc())) + 1, p.Parameter(), p.IsNew(),
                                   p.Tolerance()))
                     .ToArray();

    private static void OnBoth(Search theSearch, in ThePathPointOfTheSOnBounds theP, double theTol)
    {
        Point2D aUV = theP.Arc().Value(theP.Parameter());
        Point3D anOnSurf = theSearch.Surface.Value(aUV.X(), aUV.Y());
        Assert.True(anOnSurf.Distance(theP.Value()) <= theTol, $"{theP} is not at its arc parameter");
        Assert.True(Math.Abs(theSearch.Quadric.Distance(theP.Value())) <= theTol, $"{theP} is not on the plane");
    }

    // The transitions of the intersection line and of the arc at a point, as IntPatch_ImpPrmIntersection makes them.
    private static (global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans Line, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans Arc) Transitions(Search theSearch,
                                                                                in ThePathPointOfTheSOnBounds theP)
    {
        theP.Arc().D1(theP.Parameter(), out Point2D aUV, out Vector2D aD2d);
        theSearch.Surface.D1(aUV.X(), aUV.Y(), out _, out Vector3D aD1u, out Vector3D aD1v);

        TheSurfFunction aFunc = new(theSearch.Surface, theSearch.Quadric);
        math_Vector aX = new(1, 2);
        aX[1] = aUV.X();
        aX[2] = aUV.Y();
        math_Vector aF = new(1, 1);
        Matrix aD = new(1, 1, 1, 2);
        aFunc.Values(aX, aF, aD);
        Assert.False(aFunc.IsTangent());
        Vector3D aTgLine = aFunc.Direction3d();

        Vector3D aTgRst = new();
        aTgRst.SetLinearForm(aD2d.X(), aD1u, aD2d.Y(), aD1v);
        Direction3D aNormal = new(aD1u.Crossed(aD1v));
        global::Kerf.Engine.Intersection.Walking.WalkingTransition aTLine = new(), aTArc = new();
        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(aTgLine, aTgRst, aNormal, ref aTLine, ref aTArc);
        return (aTLine.TransitionType(), aTArc.TransitionType());
    }

    [Fact]
    public void Cylinder_cut_across_meets_both_seams_at_new_points()
    {
        Search aS = Run(Cylinder(), Plane(0, 0, 1, 0, 0, 1));
        Assert.False(aS.Result.AllArcSolution());
        Assert.Equal(0, aS.Result.SegmentCount);
        Assert.Equal(new[] { (2, 1.0, true, 1e-4), (4, 1.0, true, 1e-4) },
                     Points(aS).Select(p => (p.Arc, Math.Round(p.Param, 12), p.IsNew, Math.Round(p.Tol, 12))).ToArray());
        for (int i = 1; i <= 2; i++)
        {
            OnBoth(aS, aS.Result.Point(i), 1e-12);
        }

        // The circle z = 1 goes the way of u; it leaves the arc (both seam pcurves go up), which the face uses once
        // forward and once reversed.
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), Transitions(aS, aS.Result.Point(1)));
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), Transitions(aS, aS.Result.Point(2)));
        Assert.NotEqual(aS.Domain.Orientation(aS.Arcs[1]), aS.Domain.Orientation(aS.Arcs[3]));
    }

    [Fact]
    public void Cylinder_cut_along_its_axis_meets_both_circles_twice()
    {
        // x = 0.5: the lines u = π/3 and u = 5π/3, crossing the top (arc 1) and bottom (arc 3) circles.
        Search aS = Run(Cylinder(), Plane(0.5, 0, 0, 1, 0, 0));
        Assert.Equal(0, aS.Result.SegmentCount);
        var aPoints = Points(aS);
        Assert.Equal(new[] { 1, 1, 3, 3 }, aPoints.Select(p => p.Arc).ToArray());
        Assert.All(aPoints, p => Assert.True(p.IsNew));
        double[] anExpected = [Math.PI / 3, 5 * Math.PI / 3, Math.PI / 3, 5 * Math.PI / 3];
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(anExpected[i], aPoints[i].Param, 1e-12);
            OnBoth(aS, aS.Result.Point(i + 1), 1e-12);
        }

        // At u = π/3 the line goes up (the way of v), entering the side of the circle the normal and the arc point
        // to; at u = 5π/3 it goes down. Same on both circles, whose pcurves go the way of u.
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out), Transitions(aS, aS.Result.Point(1)));
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), Transitions(aS, aS.Result.Point(2)));
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out), Transitions(aS, aS.Result.Point(3)));
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), Transitions(aS, aS.Result.Point(4)));
    }

    [Fact]
    public void Cylinder_on_the_plane_of_its_base_gives_the_base_circle_as_a_segment()
    {
        Search aS = Run(Cylinder(), Plane(0, 0, 0, 0, 0, 1));
        Assert.False(aS.Result.AllArcSolution());

        // FindVertex adds the face's vertex on each arc that ends at z = 0 (tolerance TolBoundary), at that end.
        Assert.Equal(new[] { (2, 0.0, false), (3, 0.0, false), (3, 2 * Math.PI, false), (4, 0.0, false) },
                     Points(aS).Select(p => (p.Arc, p.Param, p.IsNew)).ToArray());
        Assert.All(Points(aS), p => Assert.Equal(TolBoundary, p.Tol));

        // The bottom circle is on the plane from end to end; its limits are the vertex points already found.
        Assert.Equal(1, aS.Result.SegmentCount);
        TheSegmentOfTheSOnBounds aSeg = aS.Result.Segment(1);
        Assert.Same(aS.Arcs[2], aSeg.Curve());
        Assert.True(aSeg.HasFirstPoint && aSeg.HasLastPoint);
        Assert.Equal(aS.Result.Point(2), aSeg.FirstPoint());
        Assert.Equal(aS.Result.Point(3), aSeg.LastPoint());
        Assert.Same(aS.Result.Point(2).Vertex(), aSeg.FirstPoint().Vertex());
    }

    [Fact]
    public void Cylinder_tangent_to_a_plane_along_its_seam_gives_both_seams_as_segments()
    {
        Search aS = Run(Cylinder(), Plane(1, 0, 0, 1, 0, 0));
        Assert.False(aS.Result.AllArcSolution());
        Assert.All(Points(aS), p => Assert.False(p.IsNew));
        Assert.Equal(new[] { 1, 1, 2, 2, 3, 3, 4, 4 }, Points(aS).Select(p => p.Arc).ToArray());

        Assert.Equal(2, aS.Result.SegmentCount);
        for (int i = 1; i <= 2; i++)
        {
            TheSegmentOfTheSOnBounds aSeg = aS.Result.Segment(i);
            Assert.Same(aS.Arcs[2 * i - 1], aSeg.Curve()); // the seams, arcs 2 and 4
            Assert.Equal(0.0, aSeg.FirstPoint().Parameter());
            Assert.Equal(2.0, aSeg.LastPoint().Parameter());
            Assert.Equal(0.0, new Point3D(1, 0, 2).Distance(aSeg.LastPoint().Value()), 1e-15);
        }
    }

    [Fact]
    public void Sphere_cut_by_a_horizontal_plane_meets_both_seams()
    {
        Search aS = Run(Sphere(), Plane(0, 0, 1, 0, 0, 1));
        Assert.Equal(0, aS.Result.SegmentCount);
        var aPoints = Points(aS);
        Assert.Equal(new[] { 2, 4 }, aPoints.Select(p => p.Arc).ToArray());
        foreach (var p in aPoints)
        {
            Assert.True(p.IsNew);
            Assert.Equal(Math.PI / 6 + 2 * Math.PI, p.Param, 1e-9); // the meridians are parametrised by v + 2π
        }
        for (int i = 1; i <= 2; i++)
        {
            OnBoth(aS, aS.Result.Point(i), 1e-9);
            Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), Transitions(aS, aS.Result.Point(i)));
        }
    }

    [Fact]
    public void Sphere_touching_a_plane_at_its_pole_gives_the_pole_vertex()
    {
        Search aS = Run(Sphere(), Plane(0, 0, 2, 0, 0, 1));
        var aPoints = Points(aS);
        Assert.All(aPoints, p => Assert.False(p.IsNew));
        // the north pole's vertex on the degenerated edge (both ends) and at the top of both meridians
        Assert.Equal(new[] { 1, 1, 2, 4 }, aPoints.Select(p => p.Arc).ToArray());
        Assert.Equal(5 * Math.PI / 2, aPoints[2].Param, 1e-15);
        // the degenerated edge is on the plane from end to end
        Assert.Equal(1, aS.Result.SegmentCount);
        Assert.Same(aS.Arcs[0], aS.Result.Segment(1).Curve());
    }

    [Fact]
    public void Sphere_cut_through_its_seams_has_every_arc_on_the_plane()
    {
        // y = 0 contains both meridians (the seam) and the poles: every arc is a solution.
        Search aS = Run(Sphere(), Plane(0, 0, 0, 0, 1, 0));
        Assert.True(aS.Result.AllArcSolution());
        Assert.Equal(4, aS.Result.SegmentCount);
        Assert.All(Points(aS), p => Assert.False(p.IsNew));
    }

    [Fact]
    public void Results_are_not_readable_before_Perform()
    {
        TheSOnBounds aSearch = new();
        Assert.False(aSearch.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => aSearch.PointCount);
        Assert.ThrowsAny<NotDoneException>(() => aSearch.AllArcSolution());
        Assert.ThrowsAny<DomainErrorException>(() => new ThePathPointOfTheSOnBounds().Vertex());
        Assert.ThrowsAny<DomainErrorException>(() => new TheSegmentOfTheSOnBounds().FirstPoint());
    }

    [Fact]
    public void Domain_orientations_of_the_seams_differ()
    {
        Search aS = Run(Cylinder(), Plane(0, 0, 1, 0, 0, 1));
        Assert.Contains(aS.Domain.Orientation(aS.Arcs[1]), new[] { Forward, Reversed });
    }
}

/// <summary>
/// IntStart's interior search (IntPatch_TheSearchInside over IntImp_ZerImpFunc), IntImp_Int2S over two surfaces and
/// IntImp_IntCS over a curve on a surface (IntPatch_CurvIntSurf).
/// </summary>
public class IntStart_SearchInsideTests
{
    [Fact]
    public void SearchInside_finds_points_of_the_circle_inside_a_cylinder_face()
    {
        Face aFace = new ApiMakeCylinder(1, 2).Face();
        FaceSurfaceAdaptor aSurf = new(aFace);
        BrepTopolTool aDomain = new(aSurf);
        global::Kerf.Engine.Intersection.Walking.SurfaceQuadric aQuad = new(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        TheSurfFunction aFunc = new(aQuad);
        aFunc.Set(global::Kerf.Engine.Intersection.Walking.QuadricTool.Tolerance(aQuad));

        PatchTheSearchInside aSearch = new(aFunc, aSurf, aDomain, 1e-6);
        Assert.True(aSearch.IsDone);
        Assert.True(aSearch.PointCount > 0);
        for (int i = 1; i <= aSearch.PointCount; i++)
        {
            global::Kerf.Engine.Intersection.Walking.InteriorPoint aP = aSearch.Value(i);
            Point3D aPnt = aP.Value();
            Assert.Equal(1.0, aPnt.Z(), 1e-9);
            Assert.Equal(1.0, Math.Sqrt(aPnt.X() * aPnt.X() + aPnt.Y() * aPnt.Y()), 1e-9);
            Assert.True(aP.UParameter() > 0 && aP.UParameter() < 2 * Math.PI && Math.Abs(aP.VParameter() - 1) < 1e-9);
            // the tangent of the circle: horizontal, perpendicular to the radius; in (u, v), along u
            Assert.Equal(0.0, aP.Direction().Z(), 1e-12);
            Assert.Equal(0.0, aP.Direction().Dot(new Vector3D(aPnt.X(), aPnt.Y(), 0)), 1e-12);
            Assert.Equal(0.0, aP.Direction2d().Y(), 1e-12);
            for (int j = 1; j < i; j++)
            {
                Assert.True(aSearch.Value(j).Value().Distance(aPnt) > 1e-6, "a point found twice");
            }
        }

        // From a start point: one point, the nearest on the circle.
        PatchTheSearchInside aOne = new();
        aOne.Perform(aFunc, aSurf, 1.0, 0.9);
        Assert.Equal(1, aOne.PointCount);
        Assert.Equal(1.0, aOne.Value(1).UParameter(), 1e-12);
        Assert.Equal(1.0, aOne.Value(1).VParameter(), 1e-9);
        // Out of the surface's bounds: none.
        aOne.Perform(aFunc, aSurf, 1.0, 3.0);
        Assert.Equal(0, aOne.PointCount);
    }

    [Fact]
    public void ZerImpFunc_is_tangent_where_the_surfaces_are()
    {
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 1));
        TheSurfFunction aFunc = new(aCyl, new global::Kerf.Engine.Intersection.Walking.SurfaceQuadric(new Plane3D(new Point3D(1, 0, 0), new Direction3D(1, 0, 0))));
        math_Vector aX = new(1, 2);
        math_Vector aF = new(1, 1);
        Matrix aD = new(1, 1, 1, 2);
        aX[1] = 0.0;
        aX[2] = 0.5;
        aFunc.Values(aX, aF, aD);
        Assert.Equal(0.0, aFunc.Root(), 1e-15);
        Assert.True(aFunc.IsTangent()); // the plane x = 1 touches the cylinder along u = 0
        Assert.ThrowsAny<UndefinedDerivativeException>(() => aFunc.Direction3d());

        aX[1] = 0.3;
        aFunc.Value(aX, aF); // Value alone: IsTangent evaluates the derivatives itself
        Assert.Equal(Math.Cos(0.3) - 1, aFunc.Root(), 1e-15);
        Assert.Same(aCyl, aFunc.PSurface());
    }

    [Fact]
    public void Int2S_finds_a_point_of_two_surfaces_and_the_tangent_there()
    {
        SurfaceAdaptor aPln = new(new PlaneSurface(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1))), -5, 5, -5, 5);
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 1), 0, 2 * Math.PI, -5, 5);
        Int2S<SurfaceAdaptorBase, TheHSurfaceTool> anInt = new(aPln, aCyl, 1e-6);
        FunctionSetRoot aRsnld = new(anInt.Function());

        // near (cos 0.7, sin 0.7, 1): u1, v1 on the plane, u2, v2 on the cylinder
        NCollection_Array1<double> aParam = new(1, 4);
        aParam[1] = 0.75;
        aParam[2] = 0.65;
        aParam[3] = 0.72;
        aParam[4] = 1.05;
        anInt.Perform(aParam, aRsnld);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.False(anInt.IsTangent);
        Point3D aP = anInt.Point().Value();
        Assert.Equal(1.0, aP.Z(), 1e-9);
        Assert.Equal(1.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-9);
        anInt.Point().Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal(aP.X(), u1, 1e-9);
        Assert.Equal(aP.Y(), v1, 1e-9);
        Assert.Equal(Math.Atan2(aP.Y(), aP.X()), u2, 1e-9);
        Assert.Equal(1.0, v2, 1e-9);
        Direction3D aT = anInt.Direction();
        Assert.Equal(0.0, aT.Z(), 1e-9);
        Assert.Equal(0.0, aT.X() * aP.X() + aT.Y() * aP.Y(), 1e-9);
        Assert.Equal(0.0, anInt.DirectionOnS2().Y(), 1e-9);

        ref global::Kerf.Engine.Intersection.Walking.PntOn2S aChange = ref anInt.ChangePoint();
        aChange.SetValue(new Point3D(9, 9, 9));
        Assert.Equal(new Point3D(9, 9, 9), anInt.Point().Value());

        // the constructor that searches at once
        Int2S<SurfaceAdaptorBase, TheHSurfaceTool> anAtOnce = new(aParam, aPln, aCyl, 1e-6);
        Assert.False(anAtOnce.IsEmpty);
        Assert.Equal(0.0, aP.Distance(anAtOnce.Point().Value()), 1e-9);
    }

    [Fact]
    public void Int2S_reports_parallel_planes_as_tangent()
    {
        SurfaceAdaptor aP1 = new(new PlaneSurface(new Plane3D()), -5, 5, -5, 5);
        SurfaceAdaptor aP2 = new(new PlaneSurface(new Plane3D()), -5, 5, -5, 5);
        Int2S<SurfaceAdaptorBase, TheHSurfaceTool> anInt = new(aP1, aP2, 1e-6);
        NCollection_Array1<double> aParam = new(1, 4);
        aParam[1] = 1;
        aParam[2] = 1;
        aParam[3] = 1;
        aParam[4] = 1;
        anInt.Perform(aParam, new FunctionSetRoot(anInt.Function()));
        Assert.True(anInt.IsEmpty); // ComputeTangence: the surfaces are tangent, no search
        Assert.ThrowsAny<DomainErrorException>(() => anInt.Point());
    }

    [Fact]
    public void CurvIntSurf_intersects_a_curve_on_a_surface_with_another_surface()
    {
        SurfaceAdaptor aPln = new(new PlaneSurface(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1))), -5, 5, -5, 5);
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 1));
        // the line u = 0.3 on the cylinder, v from -5 to 5
        CurveAdaptor2D anArc = new(new LineCurve2D(new Point2D(0.3, 0), new Direction2D(0, 1)), -5, 5);
        PatchCSFunction aFunc = new(aPln, anArc, aCyl);
        CurvIntSurf anInt = new(0.0, 0.0, 0.0, aFunc, 1e-9);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.Equal(1.0, anInt.ParameterOnCurve(), 1e-9);
        anInt.ParameterOnSurface(out double u, out double v);
        Assert.Equal(Math.Cos(0.3), u, 1e-9);
        Assert.Equal(Math.Sin(0.3), v, 1e-9);
        Assert.Equal(0.0, anInt.Point().Distance(new Point3D(Math.Cos(0.3), Math.Sin(0.3), 1)), 1e-9);
        Assert.Same(aFunc, anInt.Function());
    }
}

/// <summary>IntPatch_HInterTool's sampling of surfaces and arcs.</summary>
public class IntPatch_HInterToolTests
{
    [Fact]
    public void Sample_points_follow_OCCT_s_grid_and_its_perturbations()
    {
        // A cylinder on u in [0, 2π], v in [0, 3]: 10 samples in u, 15 in v, so a grid of 5 x 8 interior points.
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 1), 0, 2 * Math.PI, 0, 3);
        HInterTool aTool = new();
        Assert.Equal(6 * 8, aTool.NbSamplePoints(aCyl));
        double du = 2 * Math.PI / 6, dv = 3.0 / 9;

        aTool.SamplePoint(aCyl, 1, out double u, out double v); // NU = NV = 0: no perturbation
        Assert.Equal((du, dv), (u, v));
        aTool.SamplePoint(aCyl, 2, out u, out v); // NU = 1, NV = 0: perturbation 1, dv scaled by 1.001 then 0.999 (OCCT)
        Assert.Equal(2 * du, u);
        Assert.Equal(dv * 1.001 * 0.999, v);
        aTool.SamplePoint(aCyl, 3, out u, out v); // perturbation 2: both scaled by 1.001
        Assert.Equal(du * 1.001 * 3, u);
        Assert.Equal(dv * 1.001, v);

        // A plane has a 2 x 2 grid only: the five fixed points.
        SurfaceAdaptor aPln = new(new PlaneSurface(new Plane3D()), 0, 10, 0, 10);
        Assert.Equal(4, aTool.NbSamplePoints(aPln));
        aTool.SamplePoint(aPln, 1, out u, out v);
        Assert.Equal((0.24 * 10, 0.26 * 10), (u, v));

        Assert.Equal(2, HInterTool.NbSamplesOnArc(new CurveAdaptor2D(new LineCurve2D(new Point2D(), new Direction2D(1, 0)))));
        Assert.Equal(10, HInterTool.NbSamplesOnArc(new CurveAdaptor2D(new CircleCurve2D(new Axis2D(), 1))));
        Assert.Equal(2, PatchHCurve2dTool.NbSamples(new CurveAdaptor2D(new LineCurve2D(new Point2D(), new Direction2D(1, 0))), 0, 1));
    }
}
