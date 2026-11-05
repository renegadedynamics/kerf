// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Auxiliary operations on a <see cref="DataStructureOfDelaun"/>: adding triangles and legalizing them by
/// Delaunay's criterion, and erasing triangles and free links.
/// </summary>
/// <remarks>A working object, as OCCT's: it writes into the structure it is given.</remarks>
internal sealed class MeshTool : Transient
{
    /// <summary>Classifies nodes against the line of a constraint link.</summary>
    public sealed class NodeClassifier
    {
        private readonly DataStructureOfDelaun _structure;
        private Line2D _constraint;
        private readonly bool _sign;

        /// <summary>A classifier against the line of the link, oriented from its first node to its last.</summary>
        public NodeClassifier(in MeshEdge constraint, DataStructureOfDelaun structure)
        {
            _structure = structure;
            _constraint = new Line2D();
            MeshVertex vertex1 = _structure.GetNode(constraint.FirstNode());
            MeshVertex vertex2 = _structure.GetNode(constraint.LastNode());

            _constraint = _constraint.WithLocation(new Point2D(vertex1.Coord()));
            _constraint = _constraint.WithDirection(new Direction2D(new Vector2D(new Point2D(vertex1.Coord()), new Point2D(vertex2.Coord()))));
            _sign = _constraint.Direction.X > 0;
        }

        /// <summary>Whether the node is strictly above the line (in the sense of its direction's X).</summary>
        public bool IsAbove(int nodeIndex)
        {
            MeshVertex vertex = _structure.GetNode(nodeIndex);
            Vector2D nodeVec = new(_constraint.Location, new Point2D(vertex.Coord()));
            if (nodeVec.SquareMagnitude > Constants.Resolution())
            {
                double cross = nodeVec.Crossed(new Vector2D(_constraint.Direction));
                if (Math.Abs(cross) > Constants.Resolution())
                {
                    return _sign ? cross < 0.0 : cross > 0.0;
                }
            }

            return false;
        }
    }

    private readonly DataStructureOfDelaun _structure;

    /// <summary>A tool over the given structure.</summary>
    public MeshTool(DataStructureOfDelaun structure) => _structure = structure;

    /// <summary>The structure.</summary>
    public DataStructureOfDelaun GetStructure() => _structure;

    /// <summary>
    /// Writes the triangles of the domain (only those in <paramref name="triangles"/>, if given) as planar faces of a
    /// compound to a BRep file.
    /// </summary>
    public void DumpTriangles(string fileName, NCollection_PackedMapOfInteger? triangles)
    {
        BrepBuilder builder = new();
        Compound result = new();
        builder.MakeCompound(ref result);

        NCollection_PackedMapOfInteger aTriangles = _structure.ElementsOfDomain();
        Span<int> nodes = stackalloc int[3];
        for (NCollection_PackedMapOfInteger.Iterator it = new(aTriangles); it.More(); it.Next())
        {
            if (triangles is not null && !triangles.Contains(it.Key()))
            {
                continue;
            }

            Triangle tri = _structure.GetElement(it.Key());
            _structure.ElementNodes(tri, nodes);

            Xy v1 = _structure.GetNode(nodes[0]).Coord();
            Xy v2 = _structure.GetNode(nodes[1]).Coord();
            Xy v3 = _structure.GetNode(nodes[2]).Coord();

            MakePolygon poly = new(new Point3D(v1.X(), v1.Y(), 0.0),
                                                   new Point3D(v2.X(), v2.Y(), 0.0),
                                                   new Point3D(v3.X(), v3.Y(), 0.0),
                                                   true);

            MakeFace faceBuilder = new(new Plane3D(new CoordinateSystem(Constants.XOY())), poly.Wire());
            builder.Add(result, faceBuilder.Shape());
        }

        BrepTools.Write(result, fileName);
    }

    /// <summary>Adds the triangle of the three nodes and legalizes its three links.</summary>
    public void AddAndLegalizeTriangle(int point1, int point2, int point3)
    {
        Span<int> edges = stackalloc int[3];
        AddTriangle(point1, point2, point3, edges);

        Legalize(edges[0]);
        Legalize(edges[1]);
        Legalize(edges[2]);
    }

    /// <summary>Adds the triangle of the three nodes (and its links); returns its link indices in a span of three.</summary>
    public void AddTriangle(int point1, int point2, int point3, Span<int> edges)
    {
        Span<bool> ori = stackalloc bool[3];
        AddLink(point1, point2, out edges[0], out ori[0]);
        AddLink(point2, point3, out edges[1], out ori[1]);
        AddLink(point3, point1, out edges[2], out ori[2]);

        _structure.AddElement(new Triangle(edges, ori, DegreeOfFreedom.Free));
    }

    /// <summary>Adds the free link of the two nodes; returns its index and whether it has this orientation.</summary>
    public void AddLink(int firstNode, int lastNode, out int linkIndex, out bool linkOri)
    {
        int linkIt = _structure.AddLink(new MeshEdge(firstNode, lastNode, DegreeOfFreedom.Free));

        linkIndex = Math.Abs(linkIt);
        linkOri = (linkIt > 0);
    }

    /// <summary>Flips the link and the links around it, recursively, until they satisfy Delaunay's criterion.</summary>
    public void Legalize(int linkIndex)
    {
        Stack<int> stack = new();
        stack.Push(linkIndex);

        NCollection_PackedMapOfInteger usedLinks = new();
        Span<int> nodes0 = stackalloc int[3];
        Span<int> nodes1 = stackalloc int[3];
        while (stack.Count != 0)
        {
            int aLinkIndex = stack.Pop();

            usedLinks.Add(aLinkIndex);
            MeshEdge link = _structure.GetLink(aLinkIndex);
            if (link.Movability() != DegreeOfFreedom.Frontier)
            {
                PairOfIndex pair = _structure.ElementsConnectedTo(aLinkIndex);
                if (pair.Extent() == 2)
                {
                    Triangle triangle1 = _structure.GetElement(pair.FirstIndex());
                    Triangle triangle2 = _structure.GetElement(pair.LastIndex());

                    _structure.ElementNodes(triangle1, nodes0);
                    _structure.ElementNodes(triangle2, nodes1);

                    int apexIndex0 = FindApexIndex(nodes0, link);
                    int apexIndex1 = FindApexIndex(nodes1, link);

                    if (CheckCircle(nodes0, nodes1[apexIndex1])
                        || CheckCircle(nodes1, nodes0[apexIndex0]))
                    {
                        _structure.RemoveElement(pair.FirstIndex());
                        _structure.RemoveElement(pair.LastIndex());
                        _structure.RemoveLink(aLinkIndex);

                        AddTriangleAndUpdateStack(nodes0[apexIndex0],
                                                  nodes0[(apexIndex0 + 1) % 3],
                                                  nodes1[apexIndex1],
                                                  usedLinks,
                                                  stack);

                        AddTriangleAndUpdateStack(nodes1[apexIndex1],
                                                  nodes1[(apexIndex1 + 1) % 3],
                                                  nodes0[apexIndex0],
                                                  usedLinks,
                                                  stack);
                    }
                }
            }
        }
    }

    /// <summary>Erases the triangles around the node, the free links they leave, and the node (if free).</summary>
    public void EraseItemsConnectedTo(int nodeIndex)
    {
        SelectorOfDataStructureOfDelaun selector = new(_structure);
        selector.NeighboursOfNode(nodeIndex);

        NCollection_IntDataMap<bool> loopEdges = new(1);
        EraseTriangles(selector.Elements(), loopEdges);
        EraseFreeLinks(loopEdges);
        _structure.RemoveNode(nodeIndex);
    }

    /// <summary>
    /// Erases the triangles outside the frontier links (on their right) and the triangles reached from them through free
    /// links around the frontier's nodes, and the free links they leave.
    /// </summary>
    public void CleanFrontierLinks()
    {
        NCollection_PackedMapOfInteger trianglesToErase = new();
        NCollection_IntDataMap<bool> loopEdges = new(1);

        NCollection_PackedMapOfInteger frontier = GetEdgesByType(DegreeOfFreedom.Frontier);
        NCollection_PackedMapOfInteger.Iterator frontierIt = new(frontier);
        for (; frontierIt.More(); frontierIt.Next())
        {
            int frontierId = frontierIt.Key();
            MeshEdge link = _structure.GetLink(frontierId);

            bool isTriangleFound = false;
            PairOfIndex pair = _structure.ElementsConnectedTo(frontierId);
            for (int elemIt = 1; elemIt <= pair.Extent() && !isTriangleFound; ++elemIt)
            {
                int priorElemId = pair.Index(elemIt);
                Triangle element = _structure.GetElement(priorElemId);
                Triangle.Int3 e = element._edges;
                Triangle.Bool3 o = element._orientations;

                for (int n = 0; n < 3 && !isTriangleFound; ++n)
                {
                    if (frontierId == e[n] && !o[n])
                    {
                        // Destruction of external triangles on boundary edges
                        isTriangleFound = true;
                        trianglesToErase.Add(priorElemId);

                        CollectTrianglesOnFreeLinksAroundNodesOf(link, e[(n + 1) % 3], trianglesToErase);
                        CollectTrianglesOnFreeLinksAroundNodesOf(link, e[(n + 2) % 3], trianglesToErase);
                    }
                }
            }
        }

        EraseTriangles(trianglesToErase, loopEdges);
        EraseFreeLinks(loopEdges);
    }

    /// <summary>Erases the triangles; their links go into <paramref name="loopEdges"/> (see <see cref="EraseTriangle"/>).</summary>
    public void EraseTriangles(NCollection_PackedMapOfInteger triangles, NCollection_IntDataMap<bool> loopEdges)
    {
        NCollection_PackedMapOfInteger.Iterator freeTriangles = new(triangles);
        for (; freeTriangles.More(); freeTriangles.Next())
        {
            EraseTriangle(freeTriangles.Key(), loopEdges);
        }
    }

    /// <summary>
    /// Erases the triangle. Each of its links is added to <paramref name="loopEdges"/> with its orientation; a link
    /// already there (shared with another erased triangle) is taken out and removed.
    /// </summary>
    public void EraseTriangle(int triangleIndex, NCollection_IntDataMap<bool> loopEdges)
    {
        Triangle element = _structure.GetElement(triangleIndex);
        Triangle.Int3 e = element._edges;
        Triangle.Bool3 o = element._orientations;

        _structure.RemoveElement(triangleIndex);

        for (int i = 0; i < 3; ++i)
        {
            if (!loopEdges.Bind(e[i], o[i]))
            {
                loopEdges.UnBind(e[i]);
                _structure.RemoveLink(e[i]);
            }
        }
    }

    /// <summary>Removes every link without triangles (frontier and fixed ones included, made free first).</summary>
    public void EraseFreeLinks()
    {
        for (int i = 1; i <= _structure.LinkCount; i++)
        {
            if (_structure.ElementsConnectedTo(i).IsEmpty)
            {
                ref MeshEdge edge = ref _structure.ChangeLink(i);
                if (edge.Movability() == DegreeOfFreedom.Deleted)
                {
                    continue;
                }

                edge.SetMovability(DegreeOfFreedom.Free);
                _structure.RemoveLink(i);
            }
        }
    }

    /// <summary>Removes the free links of the map that have no triangles.</summary>
    public void EraseFreeLinks(NCollection_IntDataMap<bool> links)
    {
        NCollection_IntDataMap<bool>.Iterator freeEdges = new(links);
        for (; freeEdges.More(); freeEdges.Next())
        {
            if (_structure.ElementsConnectedTo(freeEdges.Key()).IsEmpty)
            {
                _structure.RemoveLink(freeEdges.Key());
            }
        }
    }

    /// <summary>The links of the domain with the given movability.</summary>
    public NCollection_PackedMapOfInteger GetEdgesByType(DegreeOfFreedom edgeType)
    {
        NCollection_PackedMapOfInteger result = new();
        NCollection_PackedMapOfInteger.Iterator edgeIt = new(_structure.LinksOfDomain());

        for (; edgeIt.More(); edgeIt.Next())
        {
            MeshEdge edge = _structure.GetLink(edgeIt.Key());
            if (edge.Movability() == edgeType)
            {
                result.Add(edgeIt.Key());
            }
        }

        return result;
    }

    // Returns index of triangle node opposite to the given link.
    private static int FindApexIndex(ReadOnlySpan<int> aNodes, in MeshEdge link)
    {
        int i = 0;
        for (; i < 3; ++i)
        {
            if (aNodes[i] != link.FirstNode() && aNodes[i] != link.LastNode())
            {
                break;
            }
        }

        return i;
    }

    private bool CheckCircle(ReadOnlySpan<int> aNodes, int point)
    {
        MeshVertex vertex0 = _structure.GetNode(aNodes[0]);
        MeshVertex vertex1 = _structure.GetNode(aNodes[1]);
        MeshVertex vertex2 = _structure.GetNode(aNodes[2]);

        bool isOk = CircleTool.MakeCircle(vertex0.Coord(),
                                                   vertex1.Coord(),
                                                   vertex2.Coord(),
                                                   out Xy location,
                                                   out double radius);

        if (isOk)
        {
            MeshVertex vertex = _structure.GetNode(point);
            double dist = (vertex.Coord() - location).SquareModulus() - (radius * radius);
            return (dist < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        }

        return false;
    }

    private void AddTriangleAndUpdateStack(int node0,
                                           int node1,
                                           int node2,
                                           NCollection_PackedMapOfInteger usedLinks,
                                           Stack<int> stack)
    {
        Span<int> edges = stackalloc int[3];
        AddTriangle(node0, node1, node2, edges);

        for (int i = 0; i < 3; ++i)
        {
            if (!usedLinks.Contains(edges[i]))
            {
                stack.Push(edges[i]);
            }
        }
    }

    private void CollectTrianglesOnFreeLinksAroundNodesOf(in MeshEdge constraint,
                                                          int startLink,
                                                          NCollection_PackedMapOfInteger triangles)
    {
        NCollection_PackedMapOfInteger usedLinks = new();
        Stack<int> stack = new();
        stack.Push(startLink);
        usedLinks.Add(startLink);

        while (stack.Count != 0)
        {
            int linkIndex = stack.Pop();

            MeshEdge link = _structure.GetLink(linkIndex);
            if (link.Movability() == DegreeOfFreedom.Free
                && (link.FirstNode() == constraint.FirstNode()
                    || link.LastNode() == constraint.FirstNode()
                    || link.FirstNode() == constraint.LastNode()
                    || link.LastNode() == constraint.LastNode()))
            {
                PairOfIndex pair = _structure.ElementsConnectedTo(linkIndex);
                for (int elemIt = 1; elemIt <= pair.Extent(); ++elemIt)
                {
                    int index = pair.Index(elemIt);
                    triangles.Add(index);

                    Triangle element = _structure.GetElement(index);
                    Triangle.Int3 edges = element._edges;

                    for (int i = 0; i < 3; ++i)
                    {
                        if (edges[i] != linkIndex && !usedLinks.Contains(edges[i]))
                        {
                            usedLinks.Add(edges[i]);
                            stack.Push(edges[i]);
                        }
                    }
                }
            }
        }
    }
}
