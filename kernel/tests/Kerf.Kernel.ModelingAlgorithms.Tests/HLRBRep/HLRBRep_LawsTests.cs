// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Hidden lines by what they measure (decision 19801): what a solid shows from a given direction is known exactly: a
/// sphere's outline is its great circle, a cylinder from the side its two rulings, and a box in front of another hides what
/// lies behind it. The exact and the meshed algorithms must both come to those lengths.
/// </summary>
public class HLRBRep_LawsTests
{
    private static readonly AlgorithmProjector Top = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    // Looking along the x axis, with the z axis up the page
    private static readonly AlgorithmProjector Side = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)));

    private static readonly AlgorithmProjector Oblique = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2)));

    private static double Length(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0;
        }
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static HiddenLineAlgo Exact(AlgorithmProjector theView, int theIsos, params Shape[] theShapes)
    {
        HiddenLineAlgo anAlgo = new();
        foreach (Shape aS in theShapes)
        {
            anAlgo.Add(aS, theIsos);
        }
        anAlgo.Projector(theView);
        anAlgo.Update();
        anAlgo.Hide();
        return anAlgo;
    }

    private static PolyHlrToShape Meshed(AlgorithmProjector theView, double theDeflection, params Shape[] theShapes)
    {
        PolyAlgo anAlgo = new();
        foreach (Shape aS in theShapes)
        {
            anAlgo.Load(new IncrementalMesh(aS, theDeflection, false, 0.1).Shape());
        }
        anAlgo.Projector(theView);
        anAlgo.Update();
        PolyHlrToShape aToShape = new();
        aToShape.Update(anAlgo);
        return aToShape;
    }

    [Fact]
    public void A_sphere_outlines_its_great_circle()
    {
        Shape aSphere = new MakeSphere(5).Shape();
        var aToShape = new HlrToShape(Exact(Top, 0, aSphere));
        Assert.Equal(2 * Math.PI * 5, Length(aToShape.OutLineVCompound()), 3);
        Assert.Equal(2 * Math.PI * 5, Length(aToShape.OutLineVCompound(aSphere)), 3);
        Assert.Equal(2 * Math.PI * 5, Length(aToShape.OutLineVCompound3d()), 3);
        Assert.Equal(0, Length(aToShape.OutLineHCompound()), 6);
    }

    [Fact]
    public void A_sphere_seen_from_anywhere_outlines_the_same_length()
    {
        Shape aSphere = new MakeSphere(new Point3D(2, 3, 4), 5).Shape();
        var aToShape = new HlrToShape(Exact(Oblique, 0, aSphere));
        Assert.Equal(2 * Math.PI * 5, Length(aToShape.OutLineVCompound()), 3);
    }

    [Fact]
    public void A_torus_seen_along_its_axis_shows_two_circles()
    {
        // the inner is an outline; the outer is where the surface's seam runs, so it's an edge
        Shape aTorus = new MakeTorus(6, 2).Shape();
        var aToShape = new HlrToShape(Exact(Top, 0, aTorus));
        Assert.Equal(2 * Math.PI * 4, Length(aToShape.OutLineVCompound()), 3);
        Assert.Equal(2 * Math.PI * 8, Length(aToShape.VCompound()), 3);
        Assert.Equal(0, Length(aToShape.OutLineHCompound()), 6);
    }

    [Fact]
    public void A_torus_seen_from_the_side_hides_half_of_its_outline()
    {
        Shape aTorus = new MakeTorus(6, 2).Shape();
        var aToShape = new HlrToShape(Exact(Side, 0, aTorus));
        Assert.Equal(Length(aToShape.OutLineVCompound()), Length(aToShape.OutLineHCompound()), 0.01);
    }

    [Fact]
    public void A_cylinder_from_the_side_outlines_two_rulings()
    {
        Shape aCylinder = new ApiMakeCylinder(3, 10).Shape();
        var aToShape = new HlrToShape(Exact(Side, 0, aCylinder));
        Assert.Equal(2 * 10, Length(aToShape.OutLineVCompound()), 5);
        // the end circles are seen edge on, as lines the width of the cylinder: half of each is behind the other half
        Assert.Equal(2 * 6, Length(aToShape.VCompound()), 5);
        Assert.Equal(2 * 6, Length(aToShape.HCompound()), 5);
    }

    [Fact]
    public void A_cone_from_the_top_shows_its_end_circles()
    {
        Shape aCone = new ApiMakeCone(5, 3, 10).Shape();
        var aToShape = new HlrToShape(Exact(Top, 0, aCone));
        Assert.Equal(2 * Math.PI * (5 + 3), Length(aToShape.VCompound()), 3);
    }

    // A box 10 x 10 x 1 under another moved by 5 along x: from above, its top's edges under the other are hidden

    private static (Shape Lower, Shape Upper) TwoBoxes() =>
        (new MakeBox(10, 10, 1).Shape(), new MakeBox(new Point3D(5, 0, 5), 10, 10, 1).Shape());

    [Fact]
    public void A_box_above_another_hides_what_lies_beneath_it()
    {
        (Shape aLower, Shape anUpper) = TwoBoxes();
        var aToShape = new HlrToShape(Exact(Top, 0, aLower, anUpper));
        // all of the upper's perimeter, and of the lower's only the part not beneath the upper; the edges along y = 0
        // and y = 10 coincide with the upper's and neither hides the other, so they count from x = 0 to 5 alone
        Assert.Equal(40, Length(aToShape.VCompound(anUpper)), 5);
        Assert.Equal(5 + 5 + 10, Length(aToShape.VCompound(aLower)), 5);
    }

    [Fact]
    public void Selecting_one_shape_hides_it_by_the_others_as_well()
    {
        (Shape aLower, Shape anUpper) = TwoBoxes();
        HiddenLineAlgo anAlgo = new();
        anAlgo.Add(aLower);
        anAlgo.Add(anUpper);
        anAlgo.Projector(Top);
        anAlgo.Update();
        anAlgo.Select(1);
        anAlgo.Hide(1);
        Assert.True(Length(new HlrToShape(anAlgo).VCompound()) > 0);
        anAlgo.Select();
        anAlgo.Hide(1, 2);
        Assert.Equal(5 + 5 + 10, Length(new HlrToShape(anAlgo).VCompound()), 5);
        anAlgo.SelectEdge(1);
        anAlgo.SelectFace(2);
        anAlgo.Select();
        anAlgo.ShowAll(2);
        anAlgo.HideAll(2);
        anAlgo.InitEdgeStatus();
        anAlgo.Remove(2);
        Assert.Equal(1, anAlgo.ShapeCount);
        anAlgo.Update();
        anAlgo.Hide();
        Assert.Equal(40, Length(new HlrToShape(anAlgo).VCompound()), 5);
    }

    [Fact]
    public void Another_view_needs_the_old_outlines_forgotten()
    {
        Shape aSphere = new MakeSphere(5).Shape();
        HiddenLineAlgo anAlgo = Exact(Top, 0, aSphere);
        // the outliner keeps its outlines, so the old great circle is seen from the side, flat, and half of it behind
        anAlgo.Projector(Side);
        anAlgo.Update();
        anAlgo.Hide();
        Assert.Equal(0, Length(new HlrToShape(anAlgo).OutLineVCompound()), 6);
        anAlgo.OutLinedShapeNullify();
        anAlgo.Update();
        anAlgo.Hide();
        Assert.Equal(2 * Math.PI * 5, Length(new HlrToShape(anAlgo).OutLineVCompound()), 3);
    }

    private static BSplineSurface Bump()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double aH = i is 2 or 3 && j is 2 or 3 ? 4 : 0;
                aPoles[i, j] = new Point3D((i - 1) * 4, (j - 1) * 4, aH);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    private static NCollection_Array2<Point3D> BumpPoles() => Bump().InternalPoles();

    [Fact]
    public void A_free_form_patch_from_above_shows_its_four_edges()
    {
        foreach (Surface aSurface in new Surface[] { Bump(), new BezierSurface(BumpPoles()) })
        {
            Shape aFace = new MakeFace(aSurface, 1e-7).Shape();
            var aToShape = new HlrToShape(Exact(Top, 2, aFace));
            Assert.Equal(4 * 12, Length(aToShape.VCompound()), 3);
            Assert.True(Length(aToShape.IsoLineVCompound()) > 0);
        }
    }

    [Fact]
    public void A_free_form_patch_seen_along_its_side_outlines_its_hump()
    {
        foreach (Surface aSurface in new Surface[] { Bump(), new BezierSurface(BumpPoles()) })
        {
            Shape aFace = new MakeFace(aSurface, 1e-7).Shape();
            var aToShape = new HlrToShape(Exact(Side, 2, aFace));
            Assert.True(Length(aToShape.OutLineVCompound()) >= 12, Length(aToShape.OutLineVCompound()).ToString());
            Assert.True(Length(aToShape.OutLineVCompound()) < 3 * 12);
        }
    }

    [Theory]
    [InlineData(20.0)]
    [InlineData(50.0)]
    [InlineData(200.0)]
    [InlineData(10000.0)]
    public void Perspective_keeps_a_sphere_outline_a_circle_that_is_larger_when_closer(double theFocus)
    {
        var aView = new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), theFocus);
        var aToShape = new HlrToShape(Exact(aView, 0, new MakeSphere(5).Shape()));
        double aLength = Length(aToShape.OutLineVCompound());
        // the rim seen from a distance D is of apparent radius 5 / sqrt(1 - 25 / D^2). The outline is walked, not analytic,
        // and comes up to 1% short (occt-issues.md)
        double anExpected = 2 * Math.PI * 5 / Math.Sqrt(1 - (25 / (theFocus * theFocus)));
        Assert.InRange(aLength, 0.99 * anExpected, 1.001 * anExpected);
        Assert.Equal(0, Length(aToShape.OutLineHCompound()), 6);
    }

    // The meshed algorithm

    [Fact]
    public void A_meshed_box_shows_its_top()
    {
        PolyHlrToShape aToShape = Meshed(Oblique, 0.2, new MakeBox(10, 10, 10).Shape());
        // from above and to the side, three faces are in view, so 9 of the 12 edges, the other 3 hidden, and each edge
        // along an axis projects to sqrt(2 / 3) of its length
        double anEdge = 10 * Math.Sqrt(2.0 / 3);
        Assert.Equal(9 * anEdge, Length(aToShape.VCompound()), 3);
        Assert.Equal(3 * anEdge, Length(aToShape.HCompound()), 3);
    }

    [Fact]
    public void A_meshed_box_above_another_hides_what_lies_beneath_it()
    {
        (Shape aLower, Shape anUpper) = TwoBoxes();
        Shape aMeshLower = new IncrementalMesh(aLower, 0.2).Shape(), aMeshUpper = new IncrementalMesh(anUpper, 0.2).Shape();
        PolyAlgo anAlgo = new();
        anAlgo.Load(aMeshLower);
        anAlgo.Load(aMeshUpper);
        anAlgo.Projector(Top);
        anAlgo.Update();
        PolyHlrToShape aToShape = new();
        aToShape.Update(anAlgo);
        Assert.Equal(40, Length(aToShape.VCompound(aMeshUpper)), 3);
        Assert.InRange(Length(aToShape.VCompound(aMeshLower)), 15, 30);
        Assert.True(Length(aToShape.HCompound(aMeshLower)) > 20);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void A_meshed_curved_solid_outlines_close_to_the_exact_length(int theKind)
    {
        (Shape aShape, double anExpected, AlgorithmProjector aView) = theKind switch
        {
            0 => (new MakeSphere(5).Shape(), 2 * Math.PI * 5, Top),
            1 => (new MakeTorus(6, 2).Shape(), 2 * Math.PI * 4, Top), // the outer circle is the seam's edge
            _ => (new ApiMakeCylinder(3, 10).Shape(), 20.0, Side),
        };
        PolyHlrToShape aToShape = Meshed(aView, 0.02, aShape);
        double aLength = Length(aToShape.OutLineVCompound());
        Assert.InRange(aLength, 0.9 * anExpected, 1.15 * anExpected);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Curved_solids_hiding_each_other_in_a_mesh_stay_within_what_they_show_alone(int theKind)
    {
        // a sphere over a cylinder over a box: every kind of crossing between hiding and hidden segments
        Shape aFront = new MakeSphere(new Point3D(2, 2, 30), 5).Shape();
        Shape aMiddle = new ApiMakeCylinder(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 4, 6).Shape();
        Shape aBack = new MakeBox(new Point3D(-3, -3, 0), 6, 6, 2).Shape();
        AlgorithmProjector aView = theKind switch { 0 => Top, 1 => Oblique, _ => Side };
        double aHidden = Length(Meshed(aView, 0.05, aFront, aMiddle, aBack).HCompound());
        double aShown = Length(Meshed(aView, 0.05, aFront, aMiddle, aBack).VCompound());
        double anAlone = Length(Meshed(aView, 0.05, aFront).VCompound()) + Length(Meshed(aView, 0.05, aMiddle).VCompound()) + Length(Meshed(aView, 0.05, aBack).VCompound());
        Assert.True(aShown <= anAlone + 1e-6, $"{aShown} shown, {anAlone} alone");
        Assert.True(aHidden >= 0 && aShown > 0);
    }

    [Fact]
    public void A_meshed_free_form_patch_keeps_its_four_edges_in_view_from_above()
    {
        Shape aFace = new MakeFace(Bump(), 1e-7).Shape();
        PolyHlrToShape aToShape = Meshed(Top, 0.02, aFace);
        Assert.InRange(Length(aToShape.VCompound()), 48, 60);
    }

    [Fact]
    public void Smooth_edges_are_sorted_by_their_regularity()
    {
        // a box with a rounded edge: the fillet's boundaries are G1, not sharp
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aFillet = new Kerf.Engine.Fillets.Api.MakeFillet(aBox);
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
            Point3D aP1 = Tool.Pnt(aV1), aP2 = Tool.Pnt(aV2);
            if (Math.Abs(aP1.X - 10) < 1e-9 && Math.Abs(aP2.X - 10) < 1e-9 && Math.Abs(aP1.Y - 10) < 1e-9 && Math.Abs(aP2.Y - 10) < 1e-9)
            {
                aFillet.Add(2, anEdge);
            }
        }
        Shape aRounded = aFillet.Shape();
        var anExact = new HlrToShape(Exact(Oblique, 0, aRounded));
        Assert.True(Length(anExact.Rg1LineVCompound()) > 0);
        Assert.Equal(0, Length(anExact.RgNLineVCompound()), 6);
        PolyHlrToShape aMeshed = Meshed(Oblique, 0.05, aRounded);
        Assert.True(Length(aMeshed.Rg1LineVCompound()) > 0);
    }
}
