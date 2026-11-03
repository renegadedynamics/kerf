// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Watson (BRepMesh_Delaun) against Delabella on the same constrained point sets, each driven as unit 4D drives it:
/// DelaunayBaseMeshAlgo for Watson; DelabellaBaseMeshAlgo's base triangulation then CustomBaseMeshAlgo's constraint
/// processing for Delabella. Both must cover the same nodes and the same region.
/// </summary>
public class BRepMeshWatsonDelabellaTests
{
    // Builds the frontier wires and interior nodes into a fresh structure; returns it and the frontier links.
    private static DataStructureOfDelaun Build(IReadOnlyList<IReadOnlyList<Xy>> theWires,
                                                        IReadOnlyList<Xy> theInterior,
                                                        out List<int> theFrontier)
    {
        DataStructureOfDelaun aStructure = new();
        theFrontier = [];
        int aLocation = 0;
        foreach (IReadOnlyList<Xy> aWire in theWires)
        {
            List<int> aNodes = [.. aWire.Select(p => aStructure.AddNode(new MeshVertex(p, ++aLocation, DegreeOfFreedom.Frontier)))];
            for (int i = 0; i < aNodes.Count; ++i)
            {
                theFrontier.Add(aStructure.AddLink(new MeshEdge(aNodes[i], aNodes[(i + 1) % aNodes.Count], DegreeOfFreedom.Frontier)));
            }
        }

        foreach (Xy aPoint in theInterior)
        {
            aStructure.AddNode(new MeshVertex(aPoint, ++aLocation, DegreeOfFreedom.Free));
        }

        return aStructure;
    }

    // BRepMesh_DelabellaBaseMeshAlgo::buildBaseTriangulation, as unit 4D will port it.
    private static void buildBaseTriangulation(DataStructureOfDelaun aStructure)
    {
        B2d aBox = new();
        int aNodesNb = aStructure.NodeCount;
        double[] aPoints = new double[2 * (aNodesNb + 4)];
        for (int aNodeIt = 0; aNodeIt < aNodesNb; ++aNodeIt)
        {
            MeshVertex aVertex = aStructure.GetNode(aNodeIt + 1);
            aPoints[2 * aNodeIt + 0] = aVertex.Coord().X();
            aPoints[2 * aNodeIt + 1] = aVertex.Coord().Y();
            aBox.Add(new Point2D(aVertex.Coord()));
        }

        aBox.Enlarge(0.1 * (aBox.CornerMax() - aBox.CornerMin()).Modulus());
        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();
        (double, double)[] aCorners = [(aMin.X(), aMin.Y()), (aMax.X(), aMin.Y()), (aMax.X(), aMax.Y()), (aMin.X(), aMax.Y())];
        for (int k = 0; k < 4; ++k)
        {
            aPoints[2 * aNodesNb + 2 * k + 0] = aCorners[k].Item1;
            aPoints[2 * aNodesNb + 2 * k + 1] = aCorners[k].Item2;
            aStructure.AddNode(new MeshVertex(aCorners[k].Item1, aCorners[k].Item2, DegreeOfFreedom.Free));
        }

        double aDiffX = (aMax.X() - aMin.X());
        double aDiffY = (aMax.Y() - aMin.Y());
        for (int i = 0; i < aPoints.Length; i += 2)
        {
            aPoints[i + 0] = (aPoints[i + 0] - aMin.X()) / aDiffX - 0.5;
            aPoints[i + 1] = (aPoints[i + 1] - aMin.Y()) / aDiffY - 0.5;
        }

        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aTriangulator = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        int aVerticesNb = aTriangulator.Triangulate(aPoints.Length / 2, aPoints.AsSpan(0), aPoints.AsSpan(1), 2 * sizeof(double));
        Assert.True(aVerticesNb > 0);
        Span<int> aEdges = stackalloc int[3];
        Span<bool> aOrientations = stackalloc bool[3];
        for (global::Kerf.Engine.Meshing.Delaunay.DelaunayTriangle? aTrianglePtr = aTriangulator.GetFirstDelaunayTriangle(); aTrianglePtr is not null; aTrianglePtr = aTrianglePtr.next)
        {
            Span<int> aNodes = [aTrianglePtr.v[0]!.i + 1, aTrianglePtr.v[2]!.i + 1, aTrianglePtr.v[1]!.i + 1];
            for (int k = 0; k < 3; ++k)
            {
                int aLinkInfo = aStructure.AddLink(new MeshEdge(aNodes[k], aNodes[(k + 1) % 3], DegreeOfFreedom.Free));
                aEdges[k] = Math.Abs(aLinkInfo);
                aOrientations[k] = aLinkInfo > 0;
            }

            aStructure.AddElement(new Triangle(aEdges, aOrientations, DegreeOfFreedom.Free));
        }

        aTriangulator.Destroy();
    }

    // BRepMesh_CustomBaseMeshAlgo::generateMesh with the Delabella base, cells (-1, -1).
    private static void DelabellaMesh(DataStructureOfDelaun aStructure)
    {
        int aNodesNb = aStructure.NodeCount;
        buildBaseTriangulation(aStructure);

        Delaun aMesher = new(aStructure, -1, -1, false);
        int aNewNodesNb = aStructure.NodeCount;
        bool isRemoveAux = aNewNodesNb > aNodesNb;
        if (isRemoveAux)
        {
            NCollection_DynamicArray<int> aAuxVertices = new(aNewNodesNb - aNodesNb);
            for (int aExtNodesIt = aNodesNb + 1; aExtNodesIt <= aNewNodesNb; ++aExtNodesIt)
            {
                aAuxVertices.Append(aExtNodesIt);
            }

            aMesher.SetAuxVertices(aAuxVertices);
        }

        aMesher.ProcessConstraints();
        if (isRemoveAux)
        {
            aMesher.RemoveAuxElements();
        }

        new MeshTool(aStructure).EraseFreeLinks();
    }

    // BRepMesh_DelaunayBaseMeshAlgo::generateMesh, cells (-1, -1).
    private static void WatsonMesh(DataStructureOfDelaun aStructure)
    {
        NCollection_DynamicArray<int> aVerticesOrder = new(aStructure.NodeCount);
        for (int i = 1; i <= aStructure.NodeCount; ++i)
        {
            aVerticesOrder.Append(i);
        }

        _ = new Delaun(aStructure, aVerticesOrder, -1, -1);
        new MeshTool(aStructure).EraseFreeLinks();
    }

    private static (HashSet<int> Nodes, double Area) Coverage(DataStructureOfDelaun theStructure)
    {
        HashSet<int> aNodes = [];
        double anArea = 0.0;
        foreach ((int a, int b, int c) in BRepMeshDelaunTests.Triangles(theStructure))
        {
            aNodes.UnionWith([a, b, c]);
            Xy pa = theStructure.GetNode(a).Coord();
            anArea += 0.5 * ((theStructure.GetNode(b).Coord() - pa) ^ (theStructure.GetNode(c).Coord() - pa));
        }

        return (aNodes, anArea);
    }

    public static TheoryData<int> Seeds => [1, 2, 3];

    [Theory]
    [MemberData(nameof(Seeds))]
    public void Both_cover_a_square_with_a_hole(int theSeed)
    {
        List<Xy> anOuter = [];
        List<Xy> aHole = [];
        for (int i = 0; i < 40; ++i)
        {
            double t = 2 * Math.PI * i / 40;
            anOuter.Add(new Xy(5 * Math.Cos(t), 4 * Math.Sin(t)));
            if (i % 4 == 0)
            {
                aHole.Add(new Xy(1.0 + Math.Cos(-t), 0.5 + Math.Sin(-t)));
            }
        }

        Random aRandom = new(theSeed);
        List<Xy> anInterior = [];
        while (anInterior.Count < 150)
        {
            Xy p = new(10 * aRandom.NextDouble() - 5, 8 * aRandom.NextDouble() - 4);
            double anEllipse = p.X() * p.X() / 25 + p.Y() * p.Y() / 16;
            double aHoleDist = Math.Sqrt((p.X() - 1.0) * (p.X() - 1.0) + (p.Y() - 0.5) * (p.Y() - 0.5));
            if (anEllipse < 0.8 && aHoleDist > 1.3)
            {
                anInterior.Add(p);
            }
        }

        DataStructureOfDelaun aWatson = Build([anOuter, aHole], anInterior, out List<int> aWatsonFrontier);
        WatsonMesh(aWatson);
        DataStructureOfDelaun aDelabella = Build([anOuter, aHole], anInterior, out List<int> aDelabellaFrontier);
        int aNbInput = aDelabella.NodeCount;
        DelabellaMesh(aDelabella);

        BRepMeshDelaunTests.CheckTriangulation(aWatson, theNbHoles: 1, theCheckEmptyCircles: false);
        BRepMeshDelaunTests.CheckTriangulation(aDelabella, theNbHoles: 1, theCheckEmptyCircles: false);

        (HashSet<int> aWatsonNodes, double aWatsonArea) = Coverage(aWatson);
        (HashSet<int> aDelabellaNodes, double aDelabellaArea) = Coverage(aDelabella);
        Assert.Equal(Enumerable.Range(1, aNbInput).ToHashSet(), aWatsonNodes);
        Assert.Equal(aWatsonNodes, aDelabellaNodes);
        Assert.Equal(aWatsonArea, aDelabellaArea, 1e-9);
        Assert.Equal(aWatsonFrontier.Count, aDelabellaFrontier.Count);
    }
}
