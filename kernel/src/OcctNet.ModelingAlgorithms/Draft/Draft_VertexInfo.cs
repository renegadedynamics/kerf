// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_VertexInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_VertexInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

/// <summary>What the draft modification knows of a vertex: its new point and its parameters on its edges.</summary>
/// <remarks>
/// A value type in C++; a class in the port (decision 1033), as <see cref="Draft_EdgeInfo"/>. OCCT's two lists and
/// their iterator are two lists and an index.
/// </remarks>
public sealed class Draft_VertexInfo
{
    private gp_Pnt myGeom = new();
    private readonly List<TopoDS_Shape> myEdges = [];
    private readonly List<double> myParams = [];
    private int myItEd; // OCCT's NCollection_List<TopoDS_Shape>::Iterator

    public Draft_VertexInfo()
    {
    }

    public void Add(in TopoDS_Edge E)
    {
        for (myItEd = 0; myItEd < myEdges.Count; myItEd++)
        {
            if (E.IsSame(myEdges[myItEd]))
            {
                break;
            }
        }
        if (myItEd >= myEdges.Count)
        {
            myEdges.Add(E);
            myParams.Add(RealLast());
        }
    }

    public gp_Pnt Geometry() => myGeom;

    public ref gp_Pnt ChangeGeometry() => ref myGeom;

    /// <exception cref="Standard_DomainError">The edge isn't one of the vertex's.</exception>
    public double Parameter(in TopoDS_Edge E) => ChangeParameter(E);

    /// <exception cref="Standard_DomainError">The edge isn't one of the vertex's.</exception>
    public ref double ChangeParameter(in TopoDS_Edge E)
    {
        for (myItEd = 0; myItEd < myEdges.Count; myItEd++)
        {
            if (myEdges[myItEd].IsSame(E))
            {
                return ref CollectionsMarshal.AsSpan(myParams)[myItEd];
            }
        }
        throw new Standard_DomainError();
    }

    public void InitEdgeIterator() => myItEd = 0;

    public TopoDS_Edge Edge() => TopoDS.Edge(myEdges[myItEd]);

    public void NextEdge() => myItEd++;

    public bool MoreEdge() => myItEd < myEdges.Count;
}
