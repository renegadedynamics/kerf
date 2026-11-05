// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.lxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>
/// An indexed edge or vertex: the directing shape of a sweep. For an edge, the index is the number of vertices (0, 1
/// or 2), with flags for a closed edge and an edge infinite at its beginning or end; for a vertex, the index is its
/// rank on the edge (1 or 2) and the closed flag is the edge's.
/// </summary>
/// <remarks>A value type (OCCT's class used by value). <c>default</c> has the type TopAbs_COMPOUND (0): ported code writes
/// <c>new Sweep_NumShape()</c>, OCCT's dummy shape of type TopAbs_SHAPE.</remarks>
internal struct NumShape : IEquatable<NumShape>
{
    private ShapeType _type;
    private int _index;
    private bool _closed;
    private bool _begInf;
    private bool _endInf;

    /// <summary>A dummy indexed edge.</summary>
    public NumShape()
    {
        _type = ShapeType.Shape;
        _index = 0;
        _closed = false;
        _begInf = false;
        _endInf = false;
    }

    /// <summary>A simple indexed edge or vertex.</summary>
    public NumShape(int index, ShapeType type, bool closed = false, bool begInf = false, bool endInf = false)
    {
        _type = type;
        _index = index;
        _closed = closed;
        _begInf = begInf;
        _endInf = endInf;
    }

    /// <summary>Reinitializes the indexed edge or vertex.</summary>
    internal void Init(int index, ShapeType type, bool closed = false, bool begInf = false, bool endInf = false)
    {
        _index = index;
        _type = type;
        _closed = closed;
        _begInf = begInf;
        _endInf = endInf;
    }

    public readonly int Index() => _index;

    public readonly ShapeType Type() => _type;

    public readonly bool Closed() => _closed;

    public readonly bool BegInfinite() => _begInf;

    public readonly bool EndInfinite() => _endInf;

    public readonly Orientation Orientation()
    {
        if (_type == ShapeType.Edge)
        {
            return Forward;
        }
        else
        {
            if (_index == 2)
            {
                return Forward;
            }
            else
            {
                return Reversed;
            }
        }
    }

    public readonly bool Equals(NumShape other) =>
        _type == other._type && _index == other._index && _closed == other._closed && _begInf == other._begInf
        && _endInf == other._endInf;

    public override readonly bool Equals(object? obj) => obj is NumShape other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_type, _index, _closed, _begInf, _endInf);

    public static bool operator ==(NumShape left, NumShape right) => left.Equals(right);

    public static bool operator !=(NumShape left, NumShape right) => !left.Equals(right);
}
