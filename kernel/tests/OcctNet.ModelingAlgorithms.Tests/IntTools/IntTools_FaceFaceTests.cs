// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_FaceFace and IntTools_WLineTool: the API's contract (frozen results, the parameters), and OCCT's
/// behaviours the port keeps (occt-issues.md, unit 6D). Native agreement is IntTools_FaceFaceNativeTests'.
/// </summary>
public class IntTools_FaceFaceTests
{
    private static Face Face(in Shape theShape, int theIndex = 1)
    {
        NCollection_IndexedMap<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, global::Kerf.Topology.ShapeType.Face, aMap);
        return ShapeCasts.Face(aMap.FindKey(theIndex));
    }

    private static Face PlaneFace(Plane3D thePln, double theSize)
        => new BRepBuilderAPI_MakeFace(thePln, -theSize, theSize, -theSize, theSize).Face();

    [Fact]
    public void Lines_are_frozen_InternalLines_are_not()
    {
        Face aCyl = Face(new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape());
        Face aPln = PlaneFace(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0.2, 0, 1)), 3);
        IntTools_FaceFace aFF = new();
        aFF.Perform(aPln, aCyl);
        Assert.True(aFF.IsDone());
        IntTools_Curve anInternal = aFF.InternalLines().Value(1);
        Assert.False(anInternal.Curve()!.IsFrozen);
        NCollection_Sequence<IntTools_Curve> aLines = aFF.Lines();
        Assert.True(aLines.Length() >= 1);
        Assert.True(aLines.Value(1).Curve()!.IsFrozen);
        Assert.True(aLines.Value(1).FirstCurve2d()!.IsFrozen);
        // a copy: clearing it leaves the result
        aLines.Clear();
        Assert.True(aFF.Lines().Length() >= 1);
        // the tolerances: at least the faces' (plus half the fuzzy value each), tangential at least the larger
        IntTools_Curve aIC = aFF.Lines().Value(1);
        Assert.True(aIC.TangentialTolerance() >= Tool.Tolerance(aCyl) + 0.5 * global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Lines_throw_when_not_done()
    {
        IntTools_FaceFace aFF = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aFF.Lines());
    }

    [Fact]
    public void Parameters_and_fuzzy_value()
    {
        IntTools_FaceFace aFF = new();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aFF.FuzzyValue());
        aFF.SetFuzzyValue(0.0);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aFF.FuzzyValue());
        aFF.SetFuzzyValue(1.0e-3);
        Assert.Equal(1.0e-3, aFF.FuzzyValue());
        Assert.Null(aFF.Context());
        Face aF1 = PlaneFace(new Plane3D(), 1), aF2 = PlaneFace(new Plane3D(new Point3D(), new Direction3D(1, 0, 0)), 1);
        aFF.Perform(aF1, aF2);
        Assert.NotNull(aFF.Context());
        Assert.True(aFF.Face1().IsSame(aF1) && aFF.Face2().IsSame(aF2));
        Assert.Equal(1, aFF.Lines().Length());
        // the fuzzy value enters the curve's tolerance: the larger face tolerance plus half of it
        Assert.Equal(Tool.Tolerance(aF1) + 0.5e-3, aFF.Lines().Value(1).Tolerance(), 1.0e-15);
    }

    [Fact]
    public void Plane_plane_tangent_and_reversed_order()
    {
        Face aF1 = PlaneFace(new Plane3D(), 2), aF2 = PlaneFace(new Plane3D(), 1);
        IntTools_FaceFace aFF = new();
        aFF.Perform(aF1, aF2);
        Assert.True(aFF.IsDone());
        Assert.True(aFF.TangentFaces());
        Assert.Equal(0, aFF.Lines().Length());

        // a plane before a cylinder is sorted after it (the higher IndexType first); the 2D curves come back in the
        // caller's order
        Face aCyl = Face(new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape());
        Face aPln = PlaneFace(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 3);
        IntTools_FaceFace aFF12 = new(), aFF21 = new();
        aFF12.Perform(aPln, aCyl);
        aFF21.Perform(aCyl, aPln);
        Assert.Equal(aFF12.Lines().Length(), aFF21.Lines().Length());
        Assert.True(aFF12.Face1().IsSame(aCyl) && aFF21.Face1().IsSame(aCyl)); // the faces as sorted
        Curve2D aOnPln = aFF12.Lines().Value(1).FirstCurve2d()!;
        Curve2D aOnPln21 = aFF21.Lines().Value(1).SecondCurve2d()!;
        double t = 0.5 * (aOnPln.FirstParameter() + aOnPln.LastParameter());
        Assert.Equal(0.0, aOnPln.Value(t).Distance(aOnPln21.Value(t)), 12);
    }

    // OCCT: PerformPlanes clears the first 2D curve where it means the second, so a plane/plane line computed with
    // the first 2D curve and without the second has neither.
    [Fact]
    public void OCCT_plane_plane_without_second_pcurve_loses_the_first()
    {
        Face aF1 = PlaneFace(new Plane3D(), 1), aF2 = PlaneFace(new Plane3D(new Point3D(), new Direction3D(1, 0, 0)), 1);
        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, false, 1.0e-7);
        aFF.Perform(aF1, aF2);
        IntTools_Curve aIC = aFF.Lines().Value(1);
        Assert.NotNull(aIC.Curve());
        Assert.Null(aIC.FirstCurve2d());
        Assert.Null(aIC.SecondCurve2d());
    }

    // OCCT: Perform never clears the points, so a second Perform on the same object appends to the first's.
    [Fact]
    public void OCCT_points_accumulate_over_Performs()
    {
        Face aSph = Face(new BRepPrimAPI_MakeSphere(new Point3D(0.3, 0.2, 1.0), 1.0).Shape());
        Face aPln = PlaneFace(new Plane3D(), 3);
        IntTools_FaceFace aFF = new();
        aFF.Perform(aSph, aPln);
        Assert.Equal(0, aFF.Lines().Length());
        Assert.Equal(1, aFF.Points().Length());
        IntTools_PntOn2Faces aP = aFF.Points().Value(1);
        Assert.True(aP.P1().Face().IsSame(aSph)); // in the caller's order
        Assert.Equal(0.0, aP.P1().Pnt().Distance(new Point3D(0.3, 0.2, 0.0)), 6);
        aFF.Perform(aSph, aPln);
        Assert.Equal(2, aFF.Points().Length());
    }

    // OCCT: two unbounded planar faces (no wires) give PerformPlanes infinite bounds, and trimming the line at the
    // resulting parameters throws (native throws Standard_ConstructionError too; the recordings have such pairs).
    [Fact]
    public void OCCT_unbounded_planes_throw()
    {
        BrepBuilder aBB = new();
        Face aF1 = new(), aF2 = new();
        aBB.MakeFace(ref aF1, new PlaneSurface(new CoordinateSystem(new Point3D(0, -0.2, 0.3), new Direction3D(0, 1, 0.3))), 1.0e-7);
        aBB.MakeFace(ref aF2, new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0.5))), 1.0e-7);
        IntTools_FaceFace aFF = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aFF.Perform(aF1, aF2));
    }

    [Fact]
    public void PrepareLines3D_splits_closed_curves()
    {
        Face aCyl = Face(new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape());
        Face aPln = PlaneFace(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 3);
        IntTools_FaceFace aFF = new();
        aFF.Perform(aPln, aCyl);
        int aBefore = aFF.Lines().Length();
        aFF.PrepareLines3D(false);
        Assert.Equal(aBefore, aFF.Lines().Length());
        aFF.PrepareLines3D(true);
        Assert.True(aFF.Lines().Length() > aBefore); // the full circle in two halves
    }

    [Fact]
    public void WLineTool_NotUseSurfacesForApprox_at_a_pole()
    {
        Surface aSph = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new SphericalSurface(new CoordinateSystem(), 1.0));
        Face aF = new BRepBuilderAPI_MakeFace(aSph, 1.0e-7).Face();
        aSph.Bounds(out double u1, out _, out double v1, out double v2);
        IntSurf_LineOn2S aLine = new();
        IntSurf_PntOn2S aP = new();
        // at the pole (V = the upper bound, a degenerated row of poles)
        aP.SetValue(aSph.Value(0.3, v2), 0.3, v2, 0.3, v2);
        aLine.Add(aP);
        aP.SetValue(aSph.Value(0.4, 0.5 * (v1 + v2)), 0.4, 0.5 * (v1 + v2), 0.4, 0.5 * (v1 + v2));
        aLine.Add(aP);
        IntPatch_WLine aWL = new(aLine, false);
        Assert.True(IntTools_WLineTool.NotUseSurfacesForApprox(aF, aF, aWL, 1, 2));
        // away from the boundaries
        Assert.False(IntTools_WLineTool.NotUseSurfacesForApprox(aF, aF, aWL, 2, 2));
        _ = u1;
    }

    [Fact]
    public void WLineTool_DecompositionOfWLine_without_boundary_points_is_false()
    {
        Surface aS1 = new PlaneSurface(new CoordinateSystem());
        Surface aS2 = new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(1, 0, 0)));
        Face aF1 = PlaneFace(new Plane3D(), 2), aF2 = PlaneFace(new Plane3D(new Point3D(), new Direction3D(1, 0, 0)), 2);
        IntSurf_LineOn2S aLine = new();
        for (int i = 0; i < 4; i++)
        {
            IntSurf_PntOn2S aP = new();
            aP.SetValue(new Point3D(0, -1 + 0.5 * i, 0), 0, -1 + 0.5 * i, -1 + 0.5 * i, 0);
            aLine.Add(aP);
        }
        IntPatch_WLine aWL = new(aLine, false);
        NCollection_Sequence<IntPatch_Line> aNew = new();
        Assert.False(IntTools_WLineTool.DecompositionOfWLine(aWL, new SurfaceAdaptor(aS1),
                                                             new SurfaceAdaptor(aS2), aF1, aF2,
                                                             new GeomInt_LineConstructor(), true, 1.0e-7, aNew,
                                                             new IntTools_Context()));
        Assert.Equal(0, aNew.Length());
    }
}
