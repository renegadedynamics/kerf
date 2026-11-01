// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// SKELETON (unit 7E): the signatures the other BRepOffset files compile against; the porter of BRepOffset_Offset.cxx
// replaces the bodies, keeping these signatures.

using static Occt.GeomAbs_JoinType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>Builds the offset of a face, or the pipe or sphere joining offsets along an edge or at a vertex.</summary>
public sealed class BRepOffset_Offset
{
    private TopoDS_Shape myShape = new();
    private BRepOffset_Status myStatus = BRepOffset_Status.BRepOffset_Unknown;
    private TopoDS_Face myFace = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);

    public BRepOffset_Offset()
    {
    }

    public BRepOffset_Offset(in TopoDS_Face Face, double Offset, bool OffsetOutside = true,
                             GeomAbs_JoinType JoinType = GeomAbs_Arc) =>
        Init(Face, Offset, OffsetOutside, JoinType);

    public BRepOffset_Offset(in TopoDS_Face Face, double Offset, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Created,
                             bool OffsetOutside = true, GeomAbs_JoinType JoinType = GeomAbs_Arc) =>
        Init(Face, Offset, Created, OffsetOutside, JoinType);

    public BRepOffset_Offset(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, GeomAbs_Shape Conti = GeomAbs_C1) =>
        Init(Path, Edge1, Edge2, Offset, Polynomial, Tol, Conti);

    public BRepOffset_Offset(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                             in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge, bool Polynomial = false,
                             double Tol = 1.0e-4, GeomAbs_Shape Conti = GeomAbs_C1) =>
        Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, Polynomial, Tol, Conti);

    public BRepOffset_Offset(in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> LEdge, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, GeomAbs_Shape Conti = GeomAbs_C1) =>
        Init(Vertex, LEdge, Offset, Polynomial, Tol, Conti);

    public void Init(in TopoDS_Face Face, double Offset, bool OffsetOutside = true,
                     GeomAbs_JoinType JoinType = GeomAbs_Arc) =>
        throw new NotImplementedException();

    public void Init(in TopoDS_Face Face, double Offset, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Created,
                     bool OffsetOutside = true, GeomAbs_JoinType JoinType = GeomAbs_Arc) =>
        throw new NotImplementedException();

    public void Init(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, GeomAbs_Shape Conti = GeomAbs_C1) =>
        throw new NotImplementedException();

    public void Init(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                     in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge, bool Polynomial = false, double Tol = 1.0e-4,
                     GeomAbs_Shape Conti = GeomAbs_C1) =>
        throw new NotImplementedException();

    public void Init(in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> LEdge, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, GeomAbs_Shape Conti = GeomAbs_C1) =>
        throw new NotImplementedException();

    public void Init(in TopoDS_Edge Edge, double Offset) => throw new NotImplementedException();

    /// <summary>The shape the offset was built from.</summary>
    public TopoDS_Shape InitialShape() => myShape;

    /// <summary>The offset face (frozen).</summary>
    public TopoDS_Face Face() => throw new NotImplementedException();

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal TopoDS_Face InternalFace() => myFace;

    /// <summary>The shape generated from a sub-shape of the initial shape (frozen).</summary>
    public TopoDS_Shape Generated(in TopoDS_Shape Shape) => throw new NotImplementedException();

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalGenerated(in TopoDS_Shape Shape) => throw new NotImplementedException();

    /// <summary>The status of the offset surface.</summary>
    public BRepOffset_Status Status() => myStatus;
}
