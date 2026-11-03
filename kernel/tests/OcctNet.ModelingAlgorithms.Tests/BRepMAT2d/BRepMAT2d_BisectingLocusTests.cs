// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Engine.Offsets.MedialAxis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

using static Kerf.Geometry.JoinKind;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The bisecting locus of simple faces: the graph's counts (the same as native's, tools/matrec), the structure of
/// the tree inside a closed contour, equidistance of the nodes, the links to the topology, and that the face given is
/// never written.
/// </summary>
public class BRepMAT2d_BisectingLocusTests(ITestOutputHelper theOutput)
{
    private const string Square = "1 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L";
    private const string Rectangle = "1 W 1 4 0 0 L 20 0 L 20 5 L 0 5 L";
    private const string Triangle = "1 W 1 3 0 0 L 12 0 L 3 9 L";
    private const string LShape = "1 W 1 6 0 0 L 10 0 L 10 4 L 4 4 L 4 10 L 0 10 L";
    private const string Stadium = "1 W 1 4 0 0 L 10 0 A 13 3 10 6 L 0 6 A -3 3";
    private const string Disc = "1 W 1 2 5 0 A 0 5 -5 0 A 0 -5";
    private const string SquareWithHole = "2 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L W 1 4 3 3 L 3 7 L 7 7 L 7 3 L";
    private const string SquareWithRoundHole = "2 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L W 1 2 7 5 A 5 3 3 5 A 5 7";
    private const string Bow = "1 W 1 4 0 0 S 3 -2 7 -2 10 0 L 10 10 L 0 10 L";
    private const string OpenPolyline = "1 W 0 3 0 0 L 10 0 L 10 10 -";

    private static (MedialAxisExplorer, BisectingLocus, Face) Compute(string theFace,
        MedialSide theSide = MedialSide.Left, JoinKind theJoin = Arc, bool theOpen = false)
    {
        Face aFace = MatRecordings.Face("M L 0 0 " + theFace);
        MedialAxisExplorer anExplo = new(aFace);
        BisectingLocus aLocus = new();
        aLocus.Compute(anExplo, 1, theSide, theJoin, theOpen);
        return (anExplo, aLocus, aFace);
    }

    [Theory]
    [InlineData(Square, 4, 5, 4, 0)]
    [InlineData(Rectangle, 5, 6, 4, 0)]
    [InlineData(Triangle, 3, 4, 3, 0)]
    [InlineData(LShape, 11, 12, 7, 0)]
    [InlineData(Stadium, 5, 6, 4, 0)]
    [InlineData(Disc, 2, 4, 2, 2)]
    [InlineData(SquareWithHole, 24, 24, 12, 0)]
    [InlineData(SquareWithRoundHole, 12, 12, 6, 0)]
    [InlineData(Bow, 6, 7, 5, 0)]
    public void GraphCounts_Inside(string theFace, int theNbArcs, int theNbNodes, int theNbElts, int theNbInfinite)
    {
        (_, BisectingLocus aLocus, _) = Compute(theFace);
        Assert.True(aLocus.IsDone);
        MedialGraph aG = aLocus.Graph();
        Assert.Equal(theNbArcs, aG.NumberOfArcs());
        Assert.Equal(theNbNodes, aG.NumberOfNodes());
        Assert.Equal(theNbElts, aG.NumberOfBasicElts());
        Assert.Equal(theNbInfinite, aG.NumberOfInfiniteNodes());
    }

    [Fact]
    public void GraphCounts_OpenWire()
    {
        (_, BisectingLocus aLocus, _) = Compute(OpenPolyline);
        MedialGraph aG = aLocus.Graph();
        Assert.Equal((8, 13, 7, 5),
            (aG.NumberOfArcs(), aG.NumberOfNodes(), aG.NumberOfBasicElts(), aG.NumberOfInfiniteNodes()));
    }

    // Inside a simply connected contour, the locus is a tree: one node more than arcs, every arc's nodes and
    // elements are in the graph, the leaves are on the contour (distance 0) and each node is as far from the
    // elements around it as its distance says.
    [Theory]
    [InlineData(Square)]
    [InlineData(Rectangle)]
    [InlineData(Triangle)]
    [InlineData(LShape)]
    [InlineData(Stadium)]
    [InlineData(Bow)]
    public void Inside_IsATree_WithEquidistantNodes(string theFace)
    {
        (_, BisectingLocus aLocus, _) = Compute(theFace);
        MedialGraph aG = aLocus.Graph();
        Assert.Equal(aG.NumberOfArcs() + 1, aG.NumberOfNodes());
        for (int i = 1; i <= aG.NumberOfArcs(); i++)
        {
            MedialArc anArc = aG.Arc(i);
            Assert.Equal(i, anArc.Index());
            Assert.NotSame(anArc.FirstElement(), anArc.SecondElement());
            Assert.Same(anArc.FirstNode(), aG.Node(anArc.FirstNode().Index()));
            Assert.Same(anArc.SecondNode(), aG.Node(anArc.SecondNode().Index()));
            Assert.Same(anArc.FirstNode(), anArc.TheOtherNode(anArc.SecondNode()));
        }
        for (int i = 1; i <= aG.NumberOfNodes(); i++)
        {
            MedialNode aN = aG.Node(i);
            Assert.False(aN.Infinite());
            Assert.Equal(aN.PendingNode(), aN.OnBasicElt());
            Point2D aP = aLocus.GeomElt(aN);
            NCollection_Sequence<MedialBasicElt> aNear = new();
            aN.NearElts(aNear);
            foreach (MedialBasicElt aBE in aNear)
            {
                Assert.Equal(aN.Distance(), DistanceTo(aLocus.GeomElt(aBE), aP), 1.0e-6);
            }
        }
    }

    private static double DistanceTo(Geometry2D theElt, Point2D theP)
    {
        if (theElt is CartesianPoint2 aPnt)
        {
            return aPnt.Pnt2d().Distance(theP);
        }
        Curve2D aC = (Curve2D)theElt;
        CurveAdaptor2D anAd = new(aC);
        ExtPC2d anExt = new(theP, anAd);
        double aMin = Math.Min(aC.Value(aC.FirstParameter()).Distance(theP), aC.Value(aC.LastParameter()).Distance(theP));
        for (int i = 1; anExt.IsDone && i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, Math.Sqrt(anExt.SquareDistance(i)));
        }
        return aMin;
    }

    [Fact]
    public void Square_ArcsMeetAtTheCentre()
    {
        (_, BisectingLocus aLocus, _) = Compute(Square);
        MedialGraph aG = aLocus.Graph();
        int aNbCentre = 0;
        for (int i = 1; i <= aG.NumberOfNodes(); i++)
        {
            Point2D aP = aLocus.GeomElt(aG.Node(i));
            if (aP.Distance(new Point2D(5, 5)) < 1.0e-9)
            {
                aNbCentre++;
                Assert.Equal(5.0, aG.Node(i).Distance(), 1.0e-9);
                NCollection_Sequence<MedialArc> anArcs = new();
                aG.Node(i).LinkedArcs(anArcs);
                Assert.Equal(4, anArcs.Length());
            }
        }
        Assert.Equal(1, aNbCentre);
        for (int i = 1; i <= aG.NumberOfArcs(); i++)
        {
            Bisec aB = aLocus.GeomBis(aG.Arc(i), out bool aReverse);
            Assert.IsType<BisecAna>(aB.Value().ChangeBasisCurve());
            Point2D aStart = aB.Value().Value(aReverse ? aB.Value().LastParameter() : aB.Value().FirstParameter());
            Assert.Equal(0.0, aStart.Distance(aLocus.GeomElt(aG.Arc(i).FirstNode())), 1.0e-9);
        }
    }

    [Fact]
    public void Outside_HasInfiniteNodes()
    {
        (_, BisectingLocus aLocus, _) = Compute(Square, MedialSide.Right);
        MedialGraph aG = aLocus.Graph();
        Assert.Equal((8, 16, 8, 8),
            (aG.NumberOfArcs(), aG.NumberOfNodes(), aG.NumberOfBasicElts(), aG.NumberOfInfiniteNodes()));
        int aNbInfinite = 0;
        for (int i = 1; i <= aG.NumberOfNodes(); i++)
        {
            if (aG.Node(i).Infinite())
            {
                aNbInfinite++;
                Assert.Equal(0, aG.Node(i).GeomIndex());
            }
        }
        Assert.Equal(8, aNbInfinite);
    }

    [Fact]
    public void Contours_ElementsAndSections()
    {
        (MedialAxisExplorer anExplo, BisectingLocus aLocus, _) = Compute(SquareWithHole);
        Assert.Equal(2, anExplo.NumberOfContours());
        Assert.Equal(2, aLocus.NumberOfContours());
        int aSum = 0;
        for (int ic = 1; ic <= 2; ic++)
        {
            Assert.Equal(4, anExplo.NumberOfCurves(ic));
            for (int j = 1; j <= 4; j++)
            {
                Assert.Equal(1, aLocus.NumberOfSections(ic, j));
            }
            aSum += aLocus.NumberOfElts(ic);
            for (int ie = 1; ie <= aLocus.NumberOfElts(ic); ie++)
            {
                Assert.Same(aLocus.Graph().BasicElt(aLocus.BasicElt(ic, ie).Index()), aLocus.BasicElt(ic, ie));
            }
        }
        Assert.Equal(aLocus.Graph().NumberOfBasicElts(), aSum);
    }

    [Fact]
    public void BezierSides_AreCutAtTheirCurvatureExtrema()
    {
        (_, BisectingLocus aLocus, _) = Compute(Bow);
        Assert.True(aLocus.NumberOfSections(1, 1) > 1);
        bool aHasCC = false;
        MedialGraph aG = aLocus.Graph();
        for (int i = 1; i <= aG.NumberOfArcs(); i++)
        {
            aHasCC |= aLocus.GeomBis(aG.Arc(i), out _).Value().ChangeBasisCurve() is BisecCC or BisecPC;
        }
        Assert.True(aHasCC);
    }

    [Fact]
    public void LinkTopoBilo_EveryEdgeHasItsElements()
    {
        (MedialAxisExplorer anExplo, BisectingLocus aLocus, Face aFace) = Compute(LShape);
        LinkTopoBilo aLink = new(anExplo, aLocus);
        NCollection_IndexedMap<Shape> anEdges = new(ShapeHasher.Instance);
        Exploration.MapShapes(aFace, ShapeType.Edge, anEdges);
        int aNb = 0;
        for (int k = 1; k <= anEdges.Extent(); k++)
        {
            int aNbOnEdge = 0;
            for (aLink.Init(anEdges.FindKey(k)); aLink.More(); aLink.Next())
            {
                aNbOnEdge++;
                Assert.True(aLink.GeneratingShape(aLink.Value()).IsSame(anEdges.FindKey(k)));
            }
            Assert.Equal(1, aNbOnEdge);
            aNb += aNbOnEdge;
        }
        // The L's reflex corner is a point element, linked to its vertex.
        Assert.Equal(aLocus.Graph().NumberOfBasicElts(), aNb + 1);
        aLink.Init(new Shape());
        Assert.False(aLink.More());
    }

    [Fact]
    public void LinkTopoBilo_RequiresAFace()
    {
        // An explorer that read nothing has a null shape: OCCT dereferences its null TShape, the port throws
        // Standard_NullObject (TopoDS_Shape.ShapeType).
        MedialAxisExplorer anExplo = new();
        Assert.Throws<NullObjectException>(() => new LinkTopoBilo(anExplo, new BisectingLocus()));
    }

    [Fact]
    public void FrozenFace_IsNotWritten_AndGivesTheSameLocus()
    {
        Face aFace = MatRecordings.Face("M L 0 0 " + Stadium);
        Assert.True(aFace.TShape()!.IsFrozen);
        string aBefore = Brep(aFace);
        List<string> aFirst = MatRecordings.Run("M L 0 0 " + Stadium);
        MedialAxisExplorer anExplo = new(aFace);
        Assert.True(anExplo.IsModified(aFace));
        Shape aNew = anExplo.ModifiedShape(aFace);
        Assert.False(aNew.IsSame(aFace));
        Assert.Equal(aBefore, Brep(aFace));
        Assert.Equal(aFirst, MatRecordings.Run("M L 0 0 " + Stadium));
    }

    private static string Brep(in Shape theS)
    {
        StringWriter aW = new();
        BrepTools.Write(theS, aW);
        return aW.ToString();
    }

    // Where native OCCT crashes in Compute (an access violation, recorded as !SEH by tools/matrec), the port throws a
    // .NET exception at the same place (occt-issues.md, unit 5M): a face bounded by one full circle (MAT2d_Mat2d
    // takes the roots from the list of a bisector never built), and a holed face with Bezier sides.
    [Theory]
    [InlineData("1 O 0 0 5")]
    [InlineData("2 W 1 4 0 0 S 4 -2 8 2 12 0 L 12 12 L 0 12 L W 1 4 4 4 L 4 8 L 8 8 S 7 6 9 5 8 4 L")]
    public void Throws_WhereNativeCrashes(string theFace)
    {
        Face aFace = MatRecordings.Face("M L 0 0 " + theFace);
        MedialAxisExplorer anExplo = new(aFace);
        BisectingLocus aLocus = new();
        Exception anE = Assert.ThrowsAny<Exception>(() => aLocus.Compute(anExplo));
        theOutput.WriteLine(global::Kerf.Engine.Foundation.OcctTypeNames.Of(anE.GetType()) + ": " + anE.Message);
        theOutput.WriteLine(anE.StackTrace);
        Assert.IsNotAssignableFrom<FailureException>(anE);
    }
}
