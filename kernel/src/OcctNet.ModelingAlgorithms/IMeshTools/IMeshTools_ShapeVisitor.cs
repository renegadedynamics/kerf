// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeVisitor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeVisitor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A visitor of the faces and edges of a shape.</summary>
public abstract class IMeshTools_ShapeVisitor : Transient
{
    /// <summary>A visitor.</summary>
    protected IMeshTools_ShapeVisitor()
    {
    }

    /// <summary>Handles a face.</summary>
    public abstract void Visit(in TopoDS_Face theFace);

    /// <summary>Handles an edge.</summary>
    public abstract void Visit(in TopoDS_Edge theEdge);
}
