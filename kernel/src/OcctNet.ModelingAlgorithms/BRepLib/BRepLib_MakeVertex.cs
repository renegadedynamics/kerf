// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Makes a vertex at a point, with the default precision (<see cref="BRepLib.Precision()"/>).</summary>
public class BRepLib_MakeVertex : BRepLib_MakeShape
{
    public BRepLib_MakeVertex(in Point3D P)
    {
        var B = new BRep_Builder();
        TopoDS_Vertex V = new();
        B.MakeVertex(ref V, P, BRepLib.Precision());
        myShape = V;
        Done();
    }

    /// <summary>The vertex (frozen).</summary>
    public TopoDS_Vertex Vertex()
    {
        return TopoDS.Vertex(Shape());
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex()
    {
        return TopoDS.Vertex(InternalShape());
    }

    /// <summary>The vertex (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Vertex(BRepLib_MakeVertex theMaker) => theMaker.Vertex();
}
