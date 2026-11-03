// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.HiddenLines.TypeOfResultingEdge;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the hidden-line removal algorithms (unit 7Q2): HLRBRep_Algo and HLRBRep_InternalAlgo's shapes,
/// selections and hiding, HLRBRep_HLRToShape's compounds (frozen, of a shape and of a sub-shape of a frozen shape),
/// HLRBRep_PolyAlgo and HLRBRep_PolyHLRToShape on meshed shapes, HLRAppli_ReflectLines, the package's edge builders and
/// angles, the value classes, frozen inputs, and runs on several threads. The native comparisons are
/// <see cref="HLRBRepAlgoNativeTests"/>.
/// </summary>
public class HLRBRepAlgoTests
{
    private static readonly AlgorithmProjector Top = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static readonly AlgorithmProjector Axo =
        new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2)));

    private static string BRepText(Shape theShape)
    {
        StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static int NbEdges(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0;
        }
        NCollection_IndexedMap<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, ShapeType.Edge, aMap);
        return aMap.Extent();
    }

    private static Shape Frozen(Shape theShape)
    {
        theShape.TShape()!.FreezeGraph();
        return theShape;
    }

    private static HiddenLineAlgo Hide(AlgorithmProjector P, int nbIso, params Shape[] theShapes)
    {
        HiddenLineAlgo anAlgo = new();
        foreach (Shape aS in theShapes)
        {
            anAlgo.Add(aS, nbIso);
        }
        anAlgo.Projector(P);
        anAlgo.Update();
        anAlgo.Hide();
        return anAlgo;
    }

    [Fact]
    public void Algo_box_from_the_top_shows_its_top_and_hides_its_bottom()
    {
        Shape aBox = Frozen(new MakeBox(10.0, 20.0, 30.0).Shape());
        string aBefore = BRepText(aBox);
        HiddenLineAlgo anAlgo = Hide(Top, 0, aBox);
        HlrToShape aToShape = new(anAlgo);

        // the vertical edges project to points; the top face's four edges are visible, the bottom's hidden under them
        Shape aV = aToShape.VCompound();
        Shape aH = aToShape.HCompound();
        Assert.Equal(4, NbEdges(aV));
        Assert.Equal(4, NbEdges(aH));
        Assert.True(aV.TShape()!.IsFrozen);
        Assert.True(aH.TShape()!.IsFrozen);
        Assert.True(aToShape.OutLineVCompound().IsNull); // no outline on planes
        Assert.True(aToShape.IsoLineVCompound().IsNull);
        // 2D edges: on the XOY plane; their total length is the top face's perimeter
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(aV, aProps);
        Assert.Equal(60.0, aProps.Mass(), 1.0e-9);
        // the 3D edges are on the box's edges, at z = 30
        Shape aV3d = aToShape.CompoundOfEdges(TypeOfResultingEdge.Sharp, true, true);
        Box aBnd = new();
        Bounds.Add(aV3d, ref aBnd);
        aBnd.Get(out _, out _, out double aZmin, out _, out _, out double aZmax);
        Assert.Equal(30.0, aZmin, 1.0e-6);
        Assert.Equal(30.0, aZmax, 1.0e-6);
        // the caller's box is unchanged (the outliner works on a copy, decision 1095)
        Assert.Equal(aBefore, BRepText(aBox));
    }

    [Fact]
    public void Algo_keeps_its_shapes_and_finds_them_by_their_original_shape()
    {
        Shape aBox = Frozen(new MakeBox(10.0, 20.0, 30.0).Shape());
        Shape aSph = Frozen(new MakeSphere(new Point3D(5, 10, 40), 6.0).Shape());
        HiddenLineAlgo anAlgo = Hide(Axo, 0, aBox, aSph);
        Assert.Equal(2, anAlgo.ShapeCount);
        Assert.Equal(1, anAlgo.Index(aBox));
        Assert.Equal(2, anAlgo.Index(aSph));
        Assert.Equal(0, anAlgo.Index(new MakeBox(1.0, 1.0, 1.0).Shape()));
        Assert.NotNull(anAlgo.DataStructure());
        // the data structure holds both shapes, the bounds of each
        anAlgo.ShapeBounds(2).Bounds(out _, out _, out int e1, out int e2, out int f1, out int f2);
        Assert.Equal(anAlgo.DataStructure()!.EdgeCount, e2);
        Assert.Equal(13, e1); // after the box's twelve edges
        Assert.True(f2 >= f1);

        // a shape's results by the shape, and a face's by exploring (the face of a frozen shape: found in the copy)
        HlrToShape aToShape = new(anAlgo);
        Assert.Equal(NbEdges(aToShape.OutLineVCompound()), NbEdges(aToShape.OutLineVCompound(aSph)));
        // an edge's results (explored: the edge of a frozen shape is found in the outliner's copy), as for an
        // unfrozen box, which the outlining works on in place
        Shape anUnfrozen = new MakeBox(10.0, 20.0, 30.0).Shape();
        HlrToShape anUnfrozenToShape = new(Hide(Axo, 0, anUnfrozen, aSph));
        int anEdgeResults = 0;
        Explorer anExp = new(aBox, ShapeType.Edge);
        Explorer anUnfrozenExp = new(anUnfrozen, ShapeType.Edge);
        for (; anExp.More(); anExp.Next(), anUnfrozenExp.Next())
        {
            int aNb = NbEdges(aToShape.VCompound(anExp.Current())) + NbEdges(aToShape.HCompound(anExp.Current()));
            Assert.Equal(NbEdges(anUnfrozenToShape.VCompound(anUnfrozenExp.Current()))
                         + NbEdges(anUnfrozenToShape.HCompound(anUnfrozenExp.Current())), aNb);
            anEdgeResults += aNb;
        }
        Assert.True(anEdgeResults >= 12);

        // OCCT's copy shares the data structure
        HiddenLineAlgo aCopy = new(anAlgo);
        Assert.Same(anAlgo.DataStructure(), aCopy.DataStructure());
        Assert.Equal(2, aCopy.ShapeCount);

        // remove: the data structure is gone until the next update
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anAlgo.Remove(3));
        anAlgo.Remove(1);
        Assert.Null(anAlgo.DataStructure());
        Assert.Equal(1, anAlgo.Index(aSph));
        anAlgo.Update();
        anAlgo.Hide();
        Assert.True(NbEdges(new HlrToShape(anAlgo).OutLineVCompound()) > 0);

        // the outlined shapes are rebuilt after OutLinedShapeNullify
        anAlgo.OutLinedShapeNullify();
        Assert.True(anAlgo.ShapeBounds(1).Shape()!.OutLinedShape().IsNull);
        anAlgo.Update();
        anAlgo.Hide();
        Assert.True(NbEdges(new HlrToShape(anAlgo).OutLineVCompound()) > 0);
    }

    [Fact]
    public void InternalAlgo_shows_and_hides_all_and_hides_partially()
    {
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        Shape aBox2 = new MakeBox(new Point3D(-2, -2, 40), 4.0, 4.0, 4.0).Shape();
        HiddenLineAlgo anAlgo = new();
        anAlgo.Add(aBox);
        anAlgo.Add(aBox2);
        anAlgo.Projector(Top);
        anAlgo.Update();
        anAlgo.Select();
        anAlgo.ShowAll();
        Assert.Equal(16, NbEdges(new HlrToShape(anAlgo).VCompound())); // not hidden: both tops and bottoms
        anAlgo.HideAll();
        Assert.True(new HlrToShape(anAlgo).VCompound().IsNull);
        anAlgo.ShowAll(1);
        anAlgo.Select();
        Assert.Equal(8, NbEdges(new HlrToShape(anAlgo).VCompound()));
        // each by itself: the small box doesn't hide the big one's top
        anAlgo.PartialHide();
        int aPartial = NbEdges(new HlrToShape(anAlgo).HCompound());
        anAlgo.Hide();
        int aTotal = NbEdges(new HlrToShape(anAlgo).HCompound());
        Assert.True(aTotal > aPartial, $"{aTotal} > {aPartial}");
        Assert.False(anAlgo.Debug());
        anAlgo.Debug(true);
        Assert.True(anAlgo.Debug());
    }

    [Fact]
    public void Isolines_and_smooth_edges_of_a_cylinder()
    {
        Shape aCyl = Frozen(new ApiMakeCylinder(10.0, 30.0).Shape());
        HiddenLineAlgo anAlgo = Hide(Axo, 3, aCyl);
        HlrToShape aToShape = new(anAlgo);
        Assert.True(NbEdges(aToShape.IsoLineVCompound()) > 0);
        Assert.True(NbEdges(aToShape.OutLineVCompound()) >= 2); // the two silhouettes (split at the isolines)
        Shape aOut3d = aToShape.OutLineVCompound3d();
        Assert.True(NbEdges(aOut3d) >= 2);
        Assert.True(aOut3d.TShape()!.IsFrozen);
    }

    [Fact]
    public void ReflectLines_gives_the_outlines_in_3d_and_2d()
    {
        Shape aCyl = Frozen(new ApiMakeCylinder(10.0, 30.0).Shape());
        string aBefore = BRepText(aCyl);
        ReflectLines aReflector = new(aCyl);
        aReflector.SetAxes(1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0);
        aReflector.Perform();
        Shape aResult = aReflector.GetResult();
        Assert.True(NbEdges(aResult) >= 2);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.True(NbEdges(aReflector.GetCompoundOf3dEdges(TypeOfResultingEdge.Sharp, true, false)) > 0);
        Assert.Equal(aBefore, BRepText(aCyl));
    }

    [Fact]
    public void PolyAlgo_hides_meshed_shapes_and_skips_unmeshed_ones()
    {
        // without a mesh: nothing to hide
        PolyAlgo aNoMesh = new(new MakeBox(10.0, 20.0, 30.0).Shape());
        aNoMesh.Projector(Top);
        aNoMesh.Update();
        aNoMesh.InitHide();
        Assert.False(aNoMesh.MoreHide());

        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        aBox = Frozen(new IncrementalMesh(aBox, 0.5).Shape());
        string aBefore = BRepText(aBox);
        PolyAlgo anAlgo = new();
        anAlgo.Load(aBox);
        Assert.Equal(1, anAlgo.ShapeCount);
        Assert.Equal(1, anAlgo.Index(aBox));
        Assert.Equal(0.1, anAlgo.TolCoef());
        Assert.Equal(0.001, anAlgo.TolAngular());
        anAlgo.Projector(Top);
        anAlgo.Update();

        PolyHlrToShape aToShape = new();
        aToShape.Update(anAlgo);
        Shape aV = aToShape.VCompound();
        Assert.True(aV.TShape()!.IsFrozen);
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(aV, aProps);
        Assert.Equal(60.0, aProps.Mass(), 1.0e-6); // the top face's perimeter, in segments
        Assert.True(NbEdges(aToShape.HCompound()) > 0);
        Assert.True(NbEdges(aToShape.VCompound(aBox)) > 0);
        Assert.True(aToShape.VCompound(new MakeBox(1.0, 1.0, 1.0).Shape()).IsNull);
        // shown without hiding: all the segments, the bottom's too
        aToShape.Show();
        Assert.True(NbEdges(aToShape.VCompound()) > NbEdges(aV));
        aToShape.Hide();
        Assert.Equal(NbEdges(aV), NbEdges(aToShape.VCompound()));

        // the segments come from the caller's own edges
        EdgeStatus aStatus = new();
        anAlgo.InitHide();
        Assert.True(anAlgo.MoreHide());
        anAlgo.Hide(aStatus, out Shape anEdge, out _, out _, out _, out _);
        NCollection_IndexedMap<Shape> anEdges = new(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Edge, anEdges);
        Assert.True(anEdges.Contains(anEdge));

        Assert.Equal(aBefore, BRepText(aBox));
        // OCCT's copy shares the algorithm
        PolyAlgo aCopy = new(anAlgo);
        Assert.Same(anAlgo.Algo(), aCopy.Algo());
        Assert.Equal(1, aCopy.ShapeCount);
        anAlgo.TolCoef(0.2);
        Assert.Equal(0.2, anAlgo.TolCoef());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anAlgo.Remove(2));
        anAlgo.Remove(1);
        Assert.Equal(0, anAlgo.ShapeCount);
    }

    [Fact]
    public void PolyAlgo_outlined_shape_holds_the_shape_and_its_internal_outlines()
    {
        Shape aSph = new IncrementalMesh(new MakeSphere(10.0).Shape(), 0.2).Shape();
        PolyAlgo anAlgo = new(aSph);
        anAlgo.Projector(Axo);
        Assert.Equal(1, NbChildren(anAlgo.OutLinedShape(aSph))); // before Update: the shape alone
        anAlgo.Update();
        Shape anOutLined = anAlgo.OutLinedShape(aSph);
        Assert.True(NbChildren(anOutLined) >= 1);
        Assert.True(anOutLined.TShape()!.IsFrozen);
        Assert.True(anAlgo.OutLinedShape(new Shape()).IsNull);
    }

    private static int NbChildren(in Shape theShape) => theShape.ChildCount;

    [Fact]
    public void Exact_hiding_on_several_threads_gives_the_serial_results()
    {
        Shape aTorus = Frozen(new MakeTorus(30.0, 10.0).Shape());
        string Run() => BRepText(new HlrToShape(Hide(Axo, 2, aTorus)).OutLineVCompound())
                        + BRepText(new HlrToShape(Hide(Axo, 2, aTorus)).HCompound());
        string aSerial = Run();
        string[] aParallel = new string[8];
        Parallel.For(0, aParallel.Length, i => aParallel[i] = Run());
        Assert.All(aParallel, t => Assert.Equal(aSerial, t));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(0.1)]
    [InlineData(1.0)]
    public void PolyHLRAngleAndDeflection_as_OCCT(double theAngle)
    {
        HlrbRep.PolyHLRAngleAndDeflection(theAngle, out double anAngle, out double aDefl);
        double aMin = 1 * Math.PI / 180, aLim = 5 * Math.PI / 180, aMax = 35 * Math.PI / 180;
        double anIn = Math.Clamp(theAngle, aMin, aMax);
        double anExpected = aLim + Math.Sqrt((anIn - aMin) * (aMax - aLim) * (aMax - aLim) / (aMax - aMin));
        Assert.Equal(anExpected, anAngle);
        Assert.Equal(anExpected * anExpected * 0.5, aDefl);
    }

    [Fact]
    public void BiPoints_are_values()
    {
        Shape aBox = new MakeBox(1.0, 1.0, 1.0).Shape();
        BiPnt2D a = new(0.0, 0.0, 1.0, 1.0, aBox, true, false, false, false);
        BiPnt2D b = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0), aBox, true, false, false, false);
        Assert.Equal(a, b);
        Assert.True(a.Rg1Line());
        Assert.NotEqual(a, a.WithOutLine(true));
        Assert.True(a.WithIntLine(true).IntLine());
        BiPoint p = new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0, aBox, false, true, false, true);
        Assert.Equal(new Point3D(1.0, 2.0, 3.0), p.P2());
        Assert.True(p.RgNLine() && p.IntLine());
        Assert.False(p.WithRgNLine(false).RgNLine());
        Assert.True(new BiPoint().Shape().IsNull);
    }

    [Fact]
    public void MakeEdge_builds_the_projected_part_of_an_edge()
    {
        // a circle seen from the top: a 2D circle; its 3D edge shares the curve
        Edge anEdge = new MakeEdge(new Circle3D(Constants.XOY(), 5.0)).Edge();
        HiddenLineCurve aCurve = new();
        aCurve.Projector(new System.Runtime.CompilerServices.StrongBox<AlgorithmProjector>(Top));
        aCurve.Curve(anEdge);
        Span<double> aMin = stackalloc double[16];
        Span<double> aMax = stackalloc double[16];
        HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), aMin, aMax);
        aCurve.Update(aMin, aMax);
        Edge a2d = HlrbRep.MakeEdge(aCurve, 0.0, Math.PI);
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(a2d, aProps);
        Assert.Equal(5.0 * Math.PI, aProps.Mass(), 1.0e-9);
        Edge a3d = HlrbRep.MakeEdge3d(aCurve, 0.0, Math.PI);
        Tool.Range(a3d, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(Math.PI, l);
        Exploration.Vertices(a3d, out Vertex V1, out _);
        Exploration.Vertices(anEdge, out Vertex E1, out _);
        Assert.True(V1.IsSame(E1)); // shared at the start of the edge
    }
}
