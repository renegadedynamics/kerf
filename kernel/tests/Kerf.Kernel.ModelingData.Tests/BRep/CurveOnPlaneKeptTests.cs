// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// <c>Tool.CurveOnPlane</c> keeps its latest answers for frozen curves and planes: the same question gives a frozen copy
/// of the same curve, and any change to the question, or a plane that can still change, gives a new answer. An edge's curve is always frozen.
/// </summary>
public class CurveOnPlaneKeptTests
{
    private static Edge EdgeOn(CircleCurve3D theCurve, double theFirst = 0.5, double theLast = 2.5)
    {
        var aBuilder = new BrepBuilder();
        Edge anEdge = new();
        aBuilder.MakeEdge(ref anEdge, theCurve, 1e-7);
        aBuilder.Range(anEdge, theFirst, theLast, true);
        return anEdge;
    }

    private static CircleCurve3D Circle(double theRadius = 2.0)
        => new(new Frame3D(new Point3D(1, 2, 0), new Direction3D(0, 0, 1)), theRadius);

    private static PlaneSurface Plane(double theZ = 0.0) => new(new CoordinateSystem(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1)));

    private static double RadiusOf(Curve2D? theCurve) => Assert.IsType<CircleCurve2D>(theCurve).Radius;

    [Fact]
    public void TheSameQuestion_OfFrozenGeometry_GivesAFrozenCopy()
    {
        CircleCurve3D aCurve = Circle();
        aCurve.Freeze();
        PlaneSurface aPlane = Plane();
        aPlane.Freeze();
        Edge anEdge = EdgeOn(aCurve);

        Curve2D? aFirst = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aF1, out double aL1);
        Curve2D? aSecond = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aF2, out double aL2);

        Assert.NotNull(aFirst);
        Assert.True(aFirst.IsFrozen);
        Assert.NotNull(aSecond);
        Assert.True(aSecond.IsFrozen);
        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(RadiusOf(aFirst), RadiusOf(aSecond));
        Assert.Equal(aF1, aF2);
        Assert.Equal(aL1, aL2);
    }

    [Fact]
    public void AnotherRange_GivesItsOwnRange()
    {
        CircleCurve3D aCurve = Circle();
        aCurve.Freeze();
        PlaneSurface aPlane = Plane();
        aPlane.Freeze();

        Tool.CurveOnPlane(EdgeOn(aCurve, 0.5, 2.5), aPlane, new Placement(), out double aF1, out double aL1);
        Tool.CurveOnPlane(EdgeOn(aCurve, 1.0, 2.0), aPlane, new Placement(), out double aF2, out double aL2);

        Assert.Equal((0.5, 2.5), (aF1, aL1));
        Assert.Equal((1.0, 2.0), (aF2, aL2));
    }

    [Fact]
    public void AnotherLocation_GivesItsOwnCurve()
    {
        CircleCurve3D aCurve = Circle();
        aCurve.Freeze();
        PlaneSurface aPlane = Plane();
        aPlane.Freeze();
        Edge anEdge = EdgeOn(aCurve);
        Transform3D aMove = Transform3D.Translation(new Vector3D(10, 0, 0));

        var aCircle1 = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out _, out _));
        var aCircle2 = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane, new Placement(aMove), out _, out _));

        Assert.Equal(1.0, aCircle1.Location.X, 12);
        Assert.Equal(-9.0, aCircle2.Location.X, 12);
    }

    [Fact]
    public void AnotherPlane_GivesItsOwnCurve()
    {
        CircleCurve3D aCurve = Circle();
        aCurve.Freeze();
        PlaneSurface aPlane1 = Plane();
        aPlane1.Freeze();
        var aPlane2 = new PlaneSurface(new CoordinateSystem(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)));
        aPlane2.Freeze();
        Edge anEdge = EdgeOn(aCurve);

        var aCircle1 = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane1, new Placement(), out _, out _));
        var aCircle2 = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane2, new Placement(), out _, out _));

        Assert.Equal(1.0, aCircle1.Location.X, 12);
        Assert.Equal(-4.0, aCircle2.Location.X, 12);
    }

    [Fact]
    public void APlaneThatCanStillChange_IsNotKept()
    {
        CircleCurve3D aCurve = Circle();
        PlaneSurface aPlane = Plane();
        Edge anEdge = EdgeOn(aCurve);

        var aFirst = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out _, out _));
        Assert.Equal(1.0, aFirst.Location.X, 12);

        aPlane.SetPosition(new CoordinateSystem(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)));
        var aSecond = Assert.IsType<CircleCurve2D>(Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out _, out _));
        Assert.Equal(-4.0, aSecond.Location.X, 12);
    }

    [Fact]
    public async Task AnotherThread_MakesItsOwn()
    {
        CircleCurve3D aCurve = Circle();
        aCurve.Freeze();
        PlaneSurface aPlane = Plane();
        aPlane.Freeze();
        Edge anEdge = EdgeOn(aCurve);

        Curve2D? aHere = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out _, out _);
        Curve2D? aThere = await Task.Factory.StartNew(() => Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out _, out _),
                                                TaskCreationOptions.LongRunning);

        Assert.NotSame(aHere, aThere);
        Assert.Equal(RadiusOf(aHere), RadiusOf(aThere));
    }
}
