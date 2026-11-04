// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangulator.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangulator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Triangulates a planar polygon with holes given as wires of indices into a list of 3D points: the points are
/// projected on the plane of the given normal through the origin and triangulated by <see cref="Delaun"/>
/// with the wires as frontier links. Wires of three or four points are triangulated directly.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The triangles' node indices are the points' indices plus 1. Failures are reported to
/// the messenger (<see cref="Message.DefaultMessenger"/> unless one is set), which sends nothing by default
/// (decision 493). <c>MoniTool_TimerSentry</c> (under <c>CHECK_PERF</c>) is left out.
/// </remarks>
internal sealed class Triangulator
{
    private readonly List<Xyz> _xyZs;
    private readonly NCollection_List<NCollection_Sequence<int>> _wires;
    private readonly Plane3D _plane;
    private Messenger? _mess;

    private DataStructureOfDelaun? _meshStructure;
    private readonly Dictionary<int, int> _tmpMap = new();
    private List<int>? _indices;

    /// <summary>A triangulation of the given triangles over the given nodes.</summary>
    public static Triangulation ToPolyTriangulation(NCollection_Array1<Point3D> nodes,
                                                         NCollection_List<Triangle1> polyTriangles)
    {
        NCollection_Array1<Triangle1> triangles = new(1, polyTriangles.Extent());
        NCollection_List<Triangle1>.Iterator itT = new(polyTriangles);
        for (int i = 1; itT.More(); itT.Next(), i++)
        {
            triangles.SetValue(i, itT.Value());
        }

        return new Triangulation(nodes, triangles);
    }

    /// <summary>
    /// A triangulator of the wires (sequences of indices, from 0, into <paramref name="xyZs"/>) in the plane of the
    /// given normal. The point list and the wires are kept by reference, as OCCT keeps references.
    /// </summary>
    public Triangulator(List<Xyz> xyZs,
                                 NCollection_List<NCollection_Sequence<int>> wires,
                                 in Direction3D norm)
    {
        _xyZs = xyZs;
        _wires = wires;
        _plane = new Plane3D(Constants.Origin(), norm);
    }

    /// <summary>Appends the triangles to <paramref name="polyTriangles"/>; false on failure.</summary>
    public bool Perform(NCollection_List<Triangle1> polyTriangles)
    {
        _mess ??= global::Kerf.Engine.Foundation.Message.DefaultMessenger();

        if (_wires.Extent() == 1)
        {
            NCollection_Sequence<int> tmpWire = _wires.First();
            if (tmpWire.Length() < 5)
            {
                // prepare triangles from simple wire (3 or 4 points)
                AddTriange34(tmpWire, polyTriangles);
                return true;
            }
        }

        if (!PrepareMeshStructure())
        {
            return false;
        }

        if (!Triangulate(polyTriangles))
        {
            return false;
        }

        return true;
    }

    /// <summary>Sets the messenger that failures are reported to.</summary>
    public void SetMessenger(Messenger? mess) => _mess = mess;

    private static int WireNodesNb(NCollection_List<NCollection_Sequence<int>> wires)
    {
        int nbNodes = 0;
        NCollection_List<NCollection_Sequence<int>>.Iterator itW = new(wires);
        for (int i = 1; itW.More(); itW.Next(), i++)
        {
            nbNodes += itW.Value().Length();
        }

        return nbNodes;
    }

    private static void AppendTriangle(int node1,
                                       int node2,
                                       int node3,
                                       NCollection_Sequence<int> w,
                                       NCollection_List<Triangle1> polyTriangles)
    {
        Triangle1 t1 = new(w[node1] + 1, w[node2] + 1, w[node3] + 1);
        polyTriangles.Append(t1);
    }

    // auxiliary for makeTrianglesUsingBRepMesh
    private void AddTriange34(NCollection_Sequence<int> w, NCollection_List<Triangle1> polyTriangles)
    {
        // prepare triangles from simple wire (3 or 4 points)
        if (w.Length() == 3)
        {
            Triangle1 t = new(w[1] + 1, w[2] + 1, w[3] + 1);
            polyTriangles.Append(t);
        }
        else if (w.Length() == 4)
        {
            double d13 = (_xyZs[w[1]] - _xyZs[w[3]]).SquareModulus();
            double d24 = (_xyZs[w[2]] - _xyZs[w[4]]).SquareModulus();
            bool use13 = true;
            if (d24 < d13)
            {
                // additional check for inner corner
                ReadOnlySpan<int> pivotNodes = [2, 4, 1, 3];
                use13 = CheckCondition(pivotNodes, w);
            }
            else
            {
                ReadOnlySpan<int> pivotNodes = [1, 3, 2, 4];
                use13 = !CheckCondition(pivotNodes, w);
            }

            if (use13)
            {
                AppendTriangle(1, 2, 3, w, polyTriangles);
                AppendTriangle(3, 4, 1, w, polyTriangles);
            }
            else
            {
                AppendTriangle(1, 2, 4, w, polyTriangles);
                AppendTriangle(2, 3, 4, w, polyTriangles);
            }
        }
    }

    // auxiliary for addTriange34
    private bool CheckCondition(ReadOnlySpan<int> nodes, NCollection_Sequence<int> w)
    {
        Xyz v0 = _xyZs[w[nodes[1]]] - _xyZs[w[nodes[0]]];
        Xyz v1 = _xyZs[w[nodes[2]]] - _xyZs[w[nodes[0]]];
        Xyz v2 = _xyZs[w[nodes[3]]] - _xyZs[w[nodes[0]]];

        Xyz cross1 = v0.Crossed(v1);
        Xyz cross2 = v0.Crossed(v2);
        return (cross1.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || cross2.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || new Direction3D(cross1).IsEqual(new Direction3D(cross2), 0.01));
    }

    // performs initialization of mesh data structure.
    private bool PrepareMeshStructure()
    {
        _indices = new List<int>(WireNodesNb(_wires));
        _meshStructure = new DataStructureOfDelaun();

        // fill this structure created BRepMesh_Vertexes using 2d points received
        // by projection initial 3d point on plane.
        try
        {
            int numNode = 0;
            NCollection_List<NCollection_Sequence<int>>.Iterator itW = new(_wires);
            for (int i = 1; itW.More(); itW.Next(), i++)
            {
                NCollection_Sequence<int> w = itW.Value();
                for (int nn = 1; nn <= w.Length(); ++nn, ++numNode)
                {
                    Point2D p2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(_plane, new Point3D(_xyZs[w[nn]]));
                    MeshVertex vertex = new(p2d.XY(), numNode, DegreeOfFreedom.Frontier);
                    int nnn = _meshStructure.AddNode(vertex);
                    _indices.SetAt(numNode, nnn);
                    _tmpMap[numNode + 1] = w[nn] + 1;
                }

                int offset = numNode - w.Length();
                for (int nn = 1; nn <= w.Length(); ++nn)
                {
                    MeshEdge edge = new(_indices[offset + (nn - 1)],
                                               _indices[offset + (nn % w.Length())],
                                               DegreeOfFreedom.Frontier);
                    _meshStructure.AddLink(edge);
                }
            }
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception) || exception is FailureException)
        {
            // OCCT catches everything (catch (...)); the port catches what decision 529 maps OCCT's failures to.
            _mess!.Send("makeTrianglesUsingBRepMesh: Exception raised during filling of "
                         + "BRepMesh_DataStructureOfDelaun",
                         Fail);
            return false;
        }

        return true;
    }

    // auxiliary for triangulation
    private bool Triangulate(NCollection_List<Triangle1> polyTriangles)
    {
        try
        {
            // perform triangulation used created aMeshStructure
            _ = new Delaun(_meshStructure!, _indices!);
            NCollection_PackedMapOfInteger triangles = _meshStructure!.ElementsOfDomain();
            if (triangles.Extent() < 1)
            {
                return false;
            }

            // prepare Poly_Triangles from result triangles and add to returned list
            Span<int> tri2d = stackalloc int[3];
            for (NCollection_PackedMapOfInteger.Iterator triIter = new(triangles); triIter.More(); triIter.Next())
            {
                int triangleId = triIter.Key();
                Triangle triangle = _meshStructure.GetElement(triangleId);
                if (triangle.Movability() == DegreeOfFreedom.Deleted)
                {
                    continue;
                }

                _meshStructure.ElementNodes(triangle, tri2d);

                Triangle1 t = new(_tmpMap.GetValue(tri2d[0]),
                                       _tmpMap.GetValue(tri2d[1]),
                                       _tmpMap.GetValue(tri2d[2]));
                polyTriangles.Append(t);
            }
        }
        catch (FailureException failure)
        {
            string str = "makeTrianglesUsingBRepMesh: Exception raised during polygon triangulation: ";
            str += failure.Message;
            _mess!.Send(str, Fail);
            return false;
        }

        return true;
    }
}
