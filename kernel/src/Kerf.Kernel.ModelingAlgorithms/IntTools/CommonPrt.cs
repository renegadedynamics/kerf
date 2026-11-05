// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CommonPrt.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CommonPrt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>A common part of two edges in 3D space: a vertex or an edge, with the ranges on each edge.</summary>
/// <remarks>
/// A class, a working object of the edge intersectors (plan section 6), because it holds a sequence of ranges: OCCT's
/// by-value copies are the copy constructor and <see cref="Assign"/>, which ported code writes out.
/// </remarks>
internal sealed class CommonPrt
{
    private Edge _edge1 = new();
    private Edge _edge2 = new();
    private ShapeType _type;
    private ParameterRange _range1 = new();
    private double _vertPar1;
    private double _vertPar2;
    private readonly NCollection_Sequence<ParameterRange> _ranges2 = new();
    private bool _allNullFlag;
    private Point3D _pnt1;
    private Point3D _pnt2;

    /// <summary>An empty common part, of type <c>Shape</c>.</summary>
    public CommonPrt()
    {
        _type = ShapeType.Shape;
        _allNullFlag = false;
        //
        _pnt1 = _pnt1.WithCoord(0.0, 0.0, 0.0);
        _pnt2 = _pnt2.WithCoord(0.0, 0.0, 0.0);
        //
        _vertPar1 = 0.0;
        _vertPar2 = 0.0;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's copy constructor).</summary>
    public CommonPrt(CommonPrt other)
    {
        _edge1 = other._edge1;
        _edge2 = other._edge2;
        _type = other._type;
        _range1 = other._range1;
        _vertPar1 = other._vertPar1;
        _vertPar2 = other._vertPar2;
        _allNullFlag = false; // OCCT: the copy constructor doesn't copy the flag (occt-issues.md, unit 6A)
        //
        _pnt1 = other._pnt1;
        _pnt2 = other._pnt2;
        int i, nb = other._ranges2.Length();
        for (i = 1; i <= nb; i++)
        {
            _ranges2.Append(other._ranges2.Value(i));
        }
    }

    /// <summary>OCCT's assignment operator.</summary>
    internal CommonPrt Assign(CommonPrt other)
    {
        _edge1 = other._edge1;
        _edge2 = other._edge2;
        _type = other._type;
        _range1 = other._range1;
        _vertPar1 = other._vertPar1;
        _vertPar2 = other._vertPar2;
        //
        _pnt1 = other._pnt1;
        _pnt2 = other._pnt2;
        //
        // OCCT: the second edge's ranges are appended to the ones already here, not replacing them
        // (occt-issues.md, unit 6A)
        int i, nb = other._ranges2.Length();
        for (i = 1; i <= nb; i++)
        {
            _ranges2.Append(other._ranges2.Value(i));
        }
        _allNullFlag = other._allNullFlag;
        return this;
    }

    internal void SetEdge1(in Edge anEdge) => _edge1 = anEdge;

    internal void SetEdge2(in Edge anEdge) => _edge2 = anEdge;

    /// <summary>Sets the type of the common part: vertex or edge.</summary>
    internal void SetType(ShapeType aType) => _type = aType;

    internal void SetRange1(in ParameterRange aRange) => _range1 = aRange;

    internal void SetRange1(double tf, double tl)
    {
        _range1.SetFirst(tf);
        _range1.SetLast(tl);
    }

    internal void AppendRange2(in ParameterRange aRange) => _ranges2.Append(aRange);

    internal void AppendRange2(double tf, double tl)
    {
        ParameterRange range = new(tf, tl);
        _ranges2.Append(range);
    }

    internal void SetVertexParameter1(double tV) => _vertPar1 = tV;

    internal void SetVertexParameter2(double tV) => _vertPar2 = tV;

    public Edge Edge1() => _edge1;

    public Edge Edge2() => _edge2;

    /// <summary>The type of the common part: vertex or edge.</summary>
    public ShapeType Type() => _type;

    public ParameterRange Range1() => _range1;

    public void Range1(out double tf, out double tl)
    {
        tf = _range1.First();
        tl = _range1.Last();
    }

    /// <summary>The ranges on the second edge (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<ParameterRange> Ranges2() => _ranges2;

    internal NCollection_Sequence<ParameterRange> ChangeRanges2() => _ranges2;

    public double VertexParameter1() => _vertPar1;

    public double VertexParameter2() => _vertPar2;

    /// <summary>Copies this into <paramref name="aCP"/> (not the flag nor the bounding points, as in OCCT).</summary>
    internal void Copy(CommonPrt aCP)
    {
        aCP.SetEdge1(Edge1());
        aCP.SetEdge2(Edge2());
        aCP.SetType(Type());
        aCP.SetRange1(Range1());
        aCP.SetVertexParameter1(_vertPar1);
        aCP.SetVertexParameter2(_vertPar2);
        int i, nb;
        nb = _ranges2.Length();
        for (i = 1; i <= nb; i++)
        {
            aCP.AppendRange2(_ranges2.Value(i));
        }
    }

    public bool AllNullFlag() => _allNullFlag;

    internal void SetAllNullFlag(bool aFlag) => _allNullFlag = aFlag;

    internal void SetBoundingPoints(in Point3D aP1, in Point3D aP2)
    {
        _pnt1 = aP1;
        _pnt2 = aP2;
    }

    public void BoundingPoints(out Point3D aP1, out Point3D aP2)
    {
        aP1 = _pnt1;
        aP2 = _pnt2;
    }
}
