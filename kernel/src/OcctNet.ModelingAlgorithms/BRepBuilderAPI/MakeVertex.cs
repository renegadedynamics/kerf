// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a vertex from a 3D point, with the default precision (<c>Precision::Confusion()</c>).</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeVertex"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class MakeVertex : MakeShape
{
    private readonly LibraryMakeVertex _makeVertex;

    // OCCT's "if (myMakeVertex.IsDone()) { Done(); myShape = myMakeVertex.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeVertex.IsDone)
        {
            Done();
            _shape = _makeVertex.InternalShape();
        }
    }

    public MakeVertex(in Point3D p)
    {
        _makeVertex = new LibraryMakeVertex(p);
        TakeResult();
    }


    /// <summary>The vertex (frozen).</summary>
    public Vertex Vertex()
    {
        return _makeVertex.Vertex();
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex()
    {
        return _makeVertex.InternalVertex();
    }

    /// <summary>The vertex (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Vertex(MakeVertex maker) => maker.Vertex();
}
