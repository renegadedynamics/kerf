// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Builds a vertex from a 3D point, with the default precision (<c>Precision::Confusion()</c>).</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeVertex"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakeVertex : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeVertex myMakeVertex;

    // OCCT's "if (myMakeVertex.IsDone()) { Done(); myShape = myMakeVertex.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeVertex.IsDone())
        {
            Done();
            myShape = myMakeVertex.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeVertex(in Point3D P)
    {
        myMakeVertex = new BRepLib_MakeVertex(P);
        TakeResult();
    }


    /// <summary>The vertex (frozen).</summary>
    public TopoDS_Vertex Vertex()
    {
        return myMakeVertex.Vertex();
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex()
    {
        return myMakeVertex.InternalVertex();
    }

    /// <summary>The vertex (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Vertex(BRepBuilderAPI_MakeVertex theMaker) => theMaker.Vertex();
}
