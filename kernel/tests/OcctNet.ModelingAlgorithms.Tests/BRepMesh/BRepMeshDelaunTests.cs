// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Property tests of the Delaunay core (unit 4B): BRepMesh_Delaun on random and degenerate point sets (grids,
/// collinear runs, duplicates, near-cocircular points) and on constrained polygons, driven as unit 4D's
/// DelaunayBaseMeshAlgo drives it; checks empty circumcircles, Euler's formula, orientation, constraints kept and the
/// same result on every run. The oracle is indirect (unit 4D's whole meshes).
/// </summary>
public class BRepMeshDelaunTests
{
    // ---- Helpers

    /// <summary>The triangles of the domain as node triples, in the structure's order.</summary>
    internal static List<(int, int, int)> Triangles(BRepMesh_DataStructureOfDelaun theStructure)
    {
        List<(int, int, int)> aResult = [];
        Span<int> aNodes = stackalloc int[3];
        for (NCollection_PackedMapOfInteger.Iterator anIt = new(theStructure.ElementsOfDomain()); anIt.More(); anIt.Next())
        {
            theStructure.ElementNodes(theStructure.GetElement(anIt.Key()), aNodes);
            aResult.Add((aNodes[0], aNodes[1], aNodes[2]));
        }

        return aResult;
    }

    private static double SignedArea(BRepMesh_DataStructureOfDelaun theStructure, (int A, int B, int C) theTriangle)
    {
        gp_XY a = theStructure.GetNode(theTriangle.A).Coord();
        gp_XY b = theStructure.GetNode(theTriangle.B).Coord();
        gp_XY c = theStructure.GetNode(theTriangle.C).Coord();
        return 0.5 * ((b - a) ^ (c - a));
    }

    /// <summary>
    /// Builds a structure of free nodes and triangulates it as DelaunayBaseMeshAlgo does: the vertex order 1..n, cells
    /// (-1, -1), then MeshTool.EraseFreeLinks.
    /// </summary>
    internal static BRepMesh_DataStructureOfDelaun TriangulateFree(IReadOnlyList<gp_XY> thePoints)
    {
        BRepMesh_DataStructureOfDelaun aStructure = new();
        for (int i = 0; i < thePoints.Count; ++i)
        {
            aStructure.AddNode(new BRepMesh_Vertex(thePoints[i], i + 1, BRepMesh_Free));
        }

        NCollection_DynamicArray<int> anOrder = new(aStructure.NbNodes());
        for (int i = 1; i <= aStructure.NbNodes(); ++i)
        {
            anOrder.Append(i);
        }

        _ = new BRepMesh_Delaun(aStructure, anOrder, -1, -1);
        new BRepMesh_MeshTool(aStructure).EraseFreeLinks();
        return aStructure;
    }

    /// <summary>Checks the triangulation's invariants; returns the triangles.</summary>
    internal static List<(int, int, int)> CheckTriangulation(BRepMesh_DataStructureOfDelaun theStructure,
                                                            int theNbHoles = 0,
                                                            bool theCheckEmptyCircles = true)
    {
        List<(int, int, int)> aTriangles = Triangles(theStructure);
        Assert.NotEmpty(aTriangles);

        // Orientation: every triangle counter-clockwise and not degenerate.
        foreach ((int, int, int) aTriangle in aTriangles)
        {
            Assert.True(SignedArea(theStructure, aTriangle) > 0.0, $"triangle {aTriangle} is not counter-clockwise");
        }

        // Links: each used by one or two triangles, once in each direction.
        Dictionary<(int, int), int> aDirected = [];
        HashSet<(int, int)> anUndirected = [];
        HashSet<int> aNodes = [];
        foreach ((int a, int b, int c) in aTriangles)
        {
            foreach ((int p, int q) in new[] { (a, b), (b, c), (c, a) })
            {
                aDirected[(p, q)] = aDirected.GetValueOrDefault((p, q)) + 1;
                anUndirected.Add((Math.Min(p, q), Math.Max(p, q)));
            }

            aNodes.Add(a);
            aNodes.Add(b);
            aNodes.Add(c);
        }

        Assert.All(aDirected.Values, n => Assert.Equal(1, n));

        // Euler: V - E + F = 1 - holes for a connected planar triangulation.
        Assert.Equal(1 - theNbHoles, aNodes.Count - anUndirected.Count + aTriangles.Count);

        if (theCheckEmptyCircles)
        {
            CheckEmptyCircles(theStructure, aTriangles, aNodes);
        }

        return aTriangles;
    }

    private static void CheckEmptyCircles(BRepMesh_DataStructureOfDelaun theStructure,
                                          List<(int, int, int)> theTriangles,
                                          HashSet<int> theNodes)
    {
        foreach ((int a, int b, int c) in theTriangles)
        {
            Assert.True(BRepMesh_CircleTool.MakeCircle(theStructure.GetNode(a).Coord(),
                                                       theStructure.GetNode(b).Coord(),
                                                       theStructure.GetNode(c).Coord(),
                                                       out gp_XY aCenter,
                                                       out double aRadius));
            double aTol = 1e-9 * Math.Max(1.0, aRadius * aRadius);
            foreach (int n in theNodes)
            {
                if (n == a || n == b || n == c)
                {
                    continue;
                }

                double aSqDist = (theStructure.GetNode(n).Coord() - aCenter).SquareModulus();
                Assert.True(aSqDist >= aRadius * aRadius - aTol,
                            $"node {n} is inside the circumcircle of ({a}, {b}, {c}) by {aRadius * aRadius - aSqDist}");
            }
        }
    }

    private static List<gp_XY> RandomPoints(int theSeed, int theCount, double theScale = 1.0)
    {
        Random aRandom = new(theSeed);
        List<gp_XY> aPoints = [];
        for (int i = 0; i < theCount; ++i)
        {
            aPoints.Add(new gp_XY(theScale * aRandom.NextDouble(), theScale * aRandom.NextDouble()));
        }

        return aPoints;
    }

    private static List<gp_XY> Grid(int theNx, int theNy, double theStep)
    {
        List<gp_XY> aPoints = [];
        for (int j = 0; j < theNy; ++j)
        {
            for (int i = 0; i < theNx; ++i)
            {
                aPoints.Add(new gp_XY(theStep * i, theStep * j));
            }
        }

        return aPoints;
    }

    // ---- Free point sets

    [Theory]
    [InlineData(1, 10)]
    [InlineData(2, 50)]
    [InlineData(3, 200)]
    [InlineData(4, 1000)]
    public void Random_points_give_a_Delaunay_triangulation(int theSeed, int theCount)
    {
        BRepMesh_DataStructureOfDelaun aStructure = TriangulateFree(RandomPoints(theSeed, theCount, 10.0));
        CheckTriangulation(aStructure);
    }

    [Theory]
    [InlineData(5, 5, 1.0)]
    [InlineData(12, 7, 0.25)]
    [InlineData(30, 30, 0.1)]
    public void Grids_give_a_Delaunay_triangulation(int theNx, int theNy, double theStep)
    {
        // Free nodes only: Delaun's cleanup removes the triangles near the hull that no frontier holds.
        CheckTriangulation(TriangulateFree(Grid(theNx, theNy, theStep)));
    }

    [Theory]
    [InlineData(5, 5, 1.0)]
    [InlineData(12, 7, 0.25)]
    [InlineData(30, 30, 0.1)]
    public void Grids_inside_their_frontier_are_split_cell_by_cell(int theNx, int theNy, double theStep)
    {
        // As a face's mesh: the boundary nodes and links are the frontier (counter-clockwise), the others free.
        List<gp_XY> aBoundary = [];
        for (int i = 0; i < theNx - 1; ++i)
        {
            aBoundary.Add(new gp_XY(theStep * i, 0.0));
        }

        for (int j = 0; j < theNy - 1; ++j)
        {
            aBoundary.Add(new gp_XY(theStep * (theNx - 1), theStep * j));
        }

        for (int i = theNx - 1; i > 0; --i)
        {
            aBoundary.Add(new gp_XY(theStep * i, theStep * (theNy - 1)));
        }

        for (int j = theNy - 1; j > 0; --j)
        {
            aBoundary.Add(new gp_XY(0.0, theStep * j));
        }

        List<gp_XY> anInterior = [];
        for (int j = 1; j < theNy - 1; ++j)
        {
            for (int i = 1; i < theNx - 1; ++i)
            {
                anInterior.Add(new gp_XY(theStep * i, theStep * j));
            }
        }

        BRepMesh_DataStructureOfDelaun aStructure =
            TriangulateConstrained([aBoundary], anInterior, null, out List<int> aFrontier, out _);
        List<(int, int, int)> aTriangles = CheckTriangulation(aStructure);
        CheckFrontier(aStructure, aFrontier);

        // A grid's cells are cocircular: every cell is split in two, so the triangles cover the rectangle.
        double anArea = 0.0;
        foreach ((int, int, int) aTriangle in aTriangles)
        {
            anArea += SignedArea(aStructure, aTriangle);
        }

        Assert.Equal((theNx - 1) * theStep * (theNy - 1) * theStep, anArea, 1e-9);
        Assert.Equal(2 * (theNx - 1) * (theNy - 1), aTriangles.Count);
    }

    [Fact]
    public void Collinear_runs_on_the_frontier()
    {
        // A right triangle whose two legs are runs of 20 collinear frontier nodes, and free nodes inside.
        List<gp_XY> aBoundary = [];
        for (int i = 0; i < 20; ++i)
        {
            aBoundary.Add(new gp_XY(0.5 * i, 0.0));
        }

        aBoundary.Add(new gp_XY(10.0, 0.0));
        for (int j = 19; j > 0; --j)
        {
            aBoundary.Add(new gp_XY(0.0, 0.5 * j + 0.5));
        }

        aBoundary.Add(new gp_XY(0.0, 0.5));
        List<gp_XY> anInterior = [new(3.0, 2.0), new(1.5, 1.5), new(5.0, 1.0), new(1.0, 6.0)];
        BRepMesh_DataStructureOfDelaun aStructure =
            TriangulateConstrained([aBoundary], anInterior, null, out List<int> aFrontier, out _);
        List<(int, int, int)> aTriangles = CheckTriangulation(aStructure, theCheckEmptyCircles: false);
        CheckFrontier(aStructure, aFrontier);

        double anArea = 0.0;
        foreach ((int, int, int) aTriangle in aTriangles)
        {
            anArea += SignedArea(aStructure, aTriangle);
        }

        Assert.Equal(50.0, anArea, 1e-9);
    }

    [Fact]
    public void Duplicates_are_merged_into_one_node()
    {
        List<gp_XY> aPoints = RandomPoints(7, 100, 5.0);
        List<gp_XY> aWithDuplicates = [.. aPoints, .. aPoints.Take(30), new gp_XY(aPoints[0].X() + 1e-9, aPoints[0].Y())];
        BRepMesh_DataStructureOfDelaun aStructure = TriangulateFree(aWithDuplicates);
        BRepMesh_DataStructureOfDelaun aReference = TriangulateFree(aPoints);

        Assert.Equal(aReference.NbNodes(), aStructure.NbNodes());
        Assert.Equal(Triangles(aReference), Triangles(aStructure));
        CheckTriangulation(aStructure);
    }

    [Fact]
    public void Near_cocircular_points()
    {
        List<gp_XY> aPoints = [];
        Random aRandom = new(11);
        for (int i = 0; i < 64; ++i)
        {
            double anAngle = 2 * Math.PI * i / 64;
            double aRadius = 1.0 + 1e-12 * (aRandom.NextDouble() - 0.5);
            aPoints.Add(new gp_XY(aRadius * Math.Cos(anAngle), aRadius * Math.Sin(anAngle)));
        }

        aPoints.Add(new gp_XY(0.0, 0.0));
        aPoints.Add(new gp_XY(0.1, 0.05));
        CheckTriangulation(TriangulateFree(aPoints));
    }

    [Fact]
    public void The_same_input_gives_the_same_triangulation_every_time()
    {
        List<gp_XY> aPoints = [.. RandomPoints(21, 300, 3.0), .. Grid(10, 10, 0.3)];
        List<(int, int, int)> aFirst = Triangles(TriangulateFree(aPoints));
        for (int aRun = 0; aRun < 3; ++aRun)
        {
            Assert.Equal(aFirst, Triangles(TriangulateFree(aPoints)));
        }

        // And in parallel, each run on its own structure (decision 490).
        List<(int, int, int)>[] aResults = new List<(int, int, int)>[8];
        Parallel.For(0, aResults.Length, i => aResults[i] = Triangles(TriangulateFree(aPoints)));
        Assert.All(aResults, r => Assert.Equal(aFirst, r));
    }

    [Fact]
    public void Three_points_and_fewer()
    {
        // With its links as the frontier, a triangle is meshed as itself.
        BRepMesh_DataStructureOfDelaun aTriangle =
            TriangulateConstrained([[new gp_XY(0, 0), new gp_XY(1, 0), new gp_XY(0, 1)]], [], null, out List<int> aFrontier, out _);
        Assert.Single(CheckTriangulation(aTriangle));
        CheckFrontier(aTriangle, aFrontier);

        // Free nodes alone (a property of OCCT's algorithm, not a bug): the cleanup after the frontier adjustment removes
        // every triangle with a free link whose nodes no frontier holds, here the only one.
        Assert.True(TriangulateFree([new gp_XY(0, 0), new gp_XY(1, 0), new gp_XY(0, 1)]).ElementsOfDomain().IsEmpty());

        BRepMesh_DataStructureOfDelaun aTwo = TriangulateFree([new gp_XY(0, 0), new gp_XY(1, 0)]);
        Assert.True(aTwo.ElementsOfDomain().IsEmpty());
        Assert.Equal(0, aTwo.NbLinks());
    }

    // ---- Constraints

    /// <summary>
    /// A structure with closed frontier polygons (each counter-clockwise for material on the left; holes clockwise),
    /// free interior points, and optional fixed internal links; triangulated as DelaunayBaseMeshAlgo does.
    /// </summary>
    internal static BRepMesh_DataStructureOfDelaun TriangulateConstrained(IReadOnlyList<IReadOnlyList<gp_XY>> theWires,
                                                                         IReadOnlyList<gp_XY> theInterior,
                                                                         IReadOnlyList<(gp_XY, gp_XY)>? theFixed,
                                                                         out List<int> theFrontierLinks,
                                                                         out List<int> theFixedLinks)
    {
        BRepMesh_DataStructureOfDelaun aStructure = new();
        theFrontierLinks = [];
        theFixedLinks = [];
        int aLocation = 0;
        foreach (IReadOnlyList<gp_XY> aWire in theWires)
        {
            List<int> aNodes = [];
            foreach (gp_XY aPoint in aWire)
            {
                aNodes.Add(aStructure.AddNode(new BRepMesh_Vertex(aPoint, ++aLocation, BRepMesh_Frontier)));
            }

            for (int i = 0; i < aNodes.Count; ++i)
            {
                theFrontierLinks.Add(aStructure.AddLink(new BRepMesh_Edge(aNodes[i], aNodes[(i + 1) % aNodes.Count], BRepMesh_Frontier)));
            }
        }

        foreach ((gp_XY, gp_XY) aFixed in theFixed ?? [])
        {
            int aNode1 = aStructure.AddNode(new BRepMesh_Vertex(aFixed.Item1, ++aLocation, BRepMesh_Fixed));
            int aNode2 = aStructure.AddNode(new BRepMesh_Vertex(aFixed.Item2, ++aLocation, BRepMesh_Fixed));
            theFixedLinks.Add(aStructure.AddLink(new BRepMesh_Edge(aNode1, aNode2, BRepMesh_Fixed)));
        }

        foreach (gp_XY aPoint in theInterior)
        {
            aStructure.AddNode(new BRepMesh_Vertex(aPoint, ++aLocation, BRepMesh_Free));
        }

        NCollection_DynamicArray<int> anOrder = new(aStructure.NbNodes());
        for (int i = 1; i <= aStructure.NbNodes(); ++i)
        {
            anOrder.Append(i);
        }

        _ = new BRepMesh_Delaun(aStructure, anOrder, -1, -1);
        new BRepMesh_MeshTool(aStructure).EraseFreeLinks();
        return aStructure;
    }

    private static List<gp_XY> Square(double theMin, double theMax, int thePointsPerSide, bool theIsCounterClockwise)
    {
        List<gp_XY> aPoints = [];
        double aStep = (theMax - theMin) / thePointsPerSide;
        for (int i = 0; i < thePointsPerSide; ++i)
        {
            aPoints.Add(new gp_XY(theMin + i * aStep, theMin));
        }

        for (int i = 0; i < thePointsPerSide; ++i)
        {
            aPoints.Add(new gp_XY(theMax, theMin + i * aStep));
        }

        for (int i = 0; i < thePointsPerSide; ++i)
        {
            aPoints.Add(new gp_XY(theMax - i * aStep, theMax));
        }

        for (int i = 0; i < thePointsPerSide; ++i)
        {
            aPoints.Add(new gp_XY(theMin, theMax - i * aStep));
        }

        if (!theIsCounterClockwise)
        {
            aPoints.Reverse();
        }

        return aPoints;
    }

    /// <summary>Every frontier link has exactly one triangle, on its left (the material side).</summary>
    private static void CheckFrontier(BRepMesh_DataStructureOfDelaun theStructure, List<int> theFrontierLinks)
    {
        foreach (int aLinkInfo in theFrontierLinks)
        {
            int aLinkId = Math.Abs(aLinkInfo);
            BRepMesh_Edge aLink = theStructure.GetLink(aLinkId);
            Assert.Equal(BRepMesh_Frontier, aLink.Movability());
            BRepMesh_PairOfIndex aPair = theStructure.ElementsConnectedTo(aLinkId);
            Assert.Equal(1, aPair.Extent());
            BRepMesh_Triangle aTriangle = theStructure.GetElement(aPair.FirstIndex());
            bool isFound = false;
            for (int i = 0; i < 3; ++i)
            {
                if (aTriangle.Edge(i) == aLinkId)
                {
                    Assert.True(aTriangle.Orientation(i) == (aLinkInfo > 0), $"frontier link {aLinkId} has its triangle on the right");
                    isFound = true;
                }
            }

            Assert.True(isFound);
        }
    }

    [Fact]
    public void Square_with_a_square_hole()
    {
        // The Delaunay part of BRepMesh_DelaunTest.MeshPlanarFaceWithHole: the outer square 10 x 10 and the hole 2 x 2.
        List<gp_XY> anInterior = [];
        foreach (gp_XY aPoint in Grid(21, 21, 0.5))
        {
            gp_XY aShifted = new(aPoint.X() - 5.0 + 0.0123, aPoint.Y() - 5.0 + 0.0456);
            if (Math.Abs(aShifted.X()) < 4.9 && Math.Abs(aShifted.Y()) < 4.9
                && (Math.Abs(aShifted.X()) > 1.1 || Math.Abs(aShifted.Y()) > 1.1))
            {
                anInterior.Add(aShifted);
            }
        }

        BRepMesh_DataStructureOfDelaun aStructure = TriangulateConstrained(
            [Square(-5.0, 5.0, 10, true), Square(-1.0, 1.0, 4, false)], anInterior, null,
            out List<int> aFrontier, out _);

        List<(int, int, int)> aTriangles = CheckTriangulation(aStructure, theNbHoles: 1, theCheckEmptyCircles: false);
        CheckFrontier(aStructure, aFrontier);

        double anArea = 0.0;
        foreach ((int, int, int) aTriangle in aTriangles)
        {
            anArea += SignedArea(aStructure, aTriangle);
        }

        Assert.Equal(100.0 - 4.0, anArea, 1e-9);
    }

    [Fact]
    public void Non_convex_polygon_with_a_fixed_internal_link()
    {
        // An L-shaped polygon (counter-clockwise), random interior points away from its frontier, and a fixed link.
        List<gp_XY> anL =
        [
            new(0, 0), new(1, 0), new(2, 0), new(3, 0), new(3, 1), new(2, 1), new(1, 1), new(1, 2), new(1, 3),
            new(0, 3), new(0, 2), new(0, 1),
        ];
        Random aRandom = new(5);
        List<gp_XY> anInterior = [];
        while (anInterior.Count < 80)
        {
            gp_XY aPoint = new(0.05 + 2.9 * aRandom.NextDouble(), 0.05 + 2.9 * aRandom.NextDouble());
            bool isInside = (aPoint.Y() < 0.95) || (aPoint.X() < 0.95);
            bool isOffFixed = Math.Abs(aPoint.Y() - 0.5) > 0.03;
            if (isInside && isOffFixed)
            {
                anInterior.Add(aPoint);
            }
        }

        BRepMesh_DataStructureOfDelaun aStructure = TriangulateConstrained(
            [anL], anInterior, [(new gp_XY(0.4, 0.5), new gp_XY(2.6, 0.5))],
            out List<int> aFrontier, out List<int> aFixed);

        List<(int, int, int)> aTriangles = CheckTriangulation(aStructure, theCheckEmptyCircles: false);
        CheckFrontier(aStructure, aFrontier);

        // The fixed link is kept, with a triangle on each side.
        int aFixedId = Math.Abs(aFixed[0]);
        Assert.Equal(BRepMesh_Fixed, aStructure.GetLink(aFixedId).Movability());
        Assert.Equal(2, aStructure.ElementsConnectedTo(aFixedId).Extent());

        double anArea = 0.0;
        foreach ((int, int, int) aTriangle in aTriangles)
        {
            anArea += SignedArea(aStructure, aTriangle);
        }

        Assert.Equal(5.0, anArea, 1e-9);
    }

    [Fact]
    public void Constrained_triangulation_is_the_same_every_time()
    {
        List<gp_XY> anInterior = [.. RandomPoints(3, 200, 8.0).Select(p => new gp_XY(p.X() - 4.0, p.Y() - 4.0))
                                                             .Where(p => Math.Abs(p.X()) > 1.05 || Math.Abs(p.Y()) > 1.05)];
        List<(int, int, int)> aFirst = Triangles(TriangulateConstrained(
            [Square(-5.0, 5.0, 7, true), Square(-1.0, 1.0, 3, false)], anInterior, null, out _, out _));
        for (int aRun = 0; aRun < 3; ++aRun)
        {
            Assert.Equal(aFirst, Triangles(TriangulateConstrained(
                [Square(-5.0, 5.0, 7, true), Square(-1.0, 1.0, 3, false)], anInterior, null, out _, out _)));
        }
    }

    // ---- Insertion after the base triangulation (DelaunayNodeInsertionMeshAlgo's AddVertices)

    [Fact]
    public void AddVertices_inserts_nodes_into_an_existing_triangulation()
    {
        List<gp_XY> aBase = Square(0.0, 4.0, 4, true);
        BRepMesh_DataStructureOfDelaun aStructure = TriangulateConstrained([aBase], [], null, out List<int> aFrontier, out _);
        BRepMesh_Delaun aMesher = new(aStructure, -1, -1, true);

        Random aRandom = new(9);
        NCollection_DynamicArray<int> aNew = new();
        for (int i = 0; i < 60; ++i)
        {
            // Away from the frontier: a free node within 1/5 of a frontier link's length of it is skipped (below).
            gp_XY aPoint = new(0.25 + 3.5 * aRandom.NextDouble(), 0.25 + 3.5 * aRandom.NextDouble());
            aNew.Append(aStructure.AddNode(new BRepMesh_Vertex(aPoint, 1000 + i, BRepMesh_Free)));
        }

        aMesher.AddVertices(aNew);
        new BRepMesh_MeshTool(aStructure).EraseFreeLinks();

        List<(int, int, int)> aTriangles = CheckTriangulation(aStructure, theCheckEmptyCircles: false);
        CheckFrontier(aStructure, aFrontier);
        HashSet<int> aUsed = [.. aTriangles.SelectMany(t => new[] { t.Item1, t.Item2, t.Item3 })];
        for (int i = 0; i < aNew.Length(); ++i)
        {
            Assert.Contains(aNew[i], aUsed);
        }
    }

    [Fact]
    public void AddVertices_skips_a_free_node_too_close_to_the_frontier()
    {
        // OCCT's Contains reports the frontier link when the node is within 1/5 of its length, and
        // createTrianglesOnNewVertices then skips a free node (a frontier or fixed one would be inserted).
        BRepMesh_DataStructureOfDelaun aStructure =
            TriangulateConstrained([Square(0.0, 4.0, 4, true)], [], null, out _, out _);
        BRepMesh_Delaun aMesher = new(aStructure, -1, -1, true);

        NCollection_DynamicArray<int> aNew = new();
        int aNear = aStructure.AddNode(new BRepMesh_Vertex(new gp_XY(0.15, 1.5), 100, BRepMesh_Free));
        int aFar = aStructure.AddNode(new BRepMesh_Vertex(new gp_XY(0.25, 2.5), 101, BRepMesh_Free));
        aNew.Append(aNear);
        aNew.Append(aFar);
        aMesher.AddVertices(aNew);

        HashSet<int> aUsed = [.. Triangles(aStructure).SelectMany(t => new[] { t.Item1, t.Item2, t.Item3 })];
        Assert.DoesNotContain(aNear, aUsed);
        Assert.Contains(aFar, aUsed);
    }
}
