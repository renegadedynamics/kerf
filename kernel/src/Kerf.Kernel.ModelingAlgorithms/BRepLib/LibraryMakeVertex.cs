// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Library;

/// <summary>Makes a vertex at a point, with the default precision (<see cref="BrepLibrary.Precision()"/>).</summary>
internal class LibraryMakeVertex : LibraryMakeShape
{
    public LibraryMakeVertex(in Point3D p)
    {
        var b = new BrepBuilder();
        Vertex v = new();
        b.MakeVertex(ref v, p, BrepLibrary.Precision());
        _shape = v;
        Done();
    }

    /// <summary>The vertex (frozen).</summary>
    public Vertex Vertex()
    {
        return ShapeCasts.Vertex(Shape());
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex()
    {
        return ShapeCasts.Vertex(InternalShape());
    }

    /// <summary>The vertex (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Vertex(LibraryMakeVertex maker) => maker.Vertex();
}
