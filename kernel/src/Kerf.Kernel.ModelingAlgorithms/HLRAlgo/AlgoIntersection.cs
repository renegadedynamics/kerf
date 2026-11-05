// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// An intersection of an edge with the boundary of a face, for the hidden-line removal: its orientation, level,
/// segment and edge indices, parameter, tolerance and state.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the orientation and the state unset; here they are the enums' first values). The setters (OCCT's overloads of the
/// getters' names) are internal.
/// </remarks>
internal partial struct AlgoIntersection : IEquatable<AlgoIntersection>
{
    private Orientation _orien;
    private int _segIndex;
    private int _index;
    private int _level;
    private double _param;
    private float _toler;
    private Containment _state;

    public AlgoIntersection()
    {
        _segIndex = 0;
        _index = 0;
        _level = 0;
        _param = 0.0;
        _toler = 0.0f;
    }

    public AlgoIntersection(Orientation ori, int lev, int segInd, int ind, double p, float tol,
                                Containment s)
    {
        _orien = ori;
        _segIndex = segInd;
        _index = ind;
        _level = lev;
        _param = p;
        _toler = tol;
        _state = s;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Orientation(Orientation ori) => _orien = ori;

    internal void Level(int lev) => _level = lev;

    internal void SegIndex(int segInd) => _segIndex = segInd;

    internal void Index(int ind) => _index = ind;

    internal void Parameter(double p) => _param = p;

    internal void Tolerance(float t) => _toler = t;

    internal void State(Containment st) => _state = st;

    // Queries

    public readonly Orientation Orientation() => _orien;

    public readonly int Level() => _level;

    public readonly int SegIndex() => _segIndex;

    public readonly int Index() => _index;

    public readonly double Parameter() => _param;

    public readonly float Tolerance() => _toler;

    public readonly Containment State() => _state;

    public readonly AlgoIntersection WithOrientation(Orientation ori)
    {
        AlgoIntersection copy = this;
        copy.Orientation(ori);
        return copy;
    }

    public readonly AlgoIntersection WithLevel(int lev)
    {
        AlgoIntersection copy = this;
        copy.Level(lev);
        return copy;
    }

    public readonly AlgoIntersection WithSegIndex(int segInd)
    {
        AlgoIntersection copy = this;
        copy.SegIndex(segInd);
        return copy;
    }

    public readonly AlgoIntersection WithIndex(int ind)
    {
        AlgoIntersection copy = this;
        copy.Index(ind);
        return copy;
    }

    public readonly AlgoIntersection WithParameter(double p)
    {
        AlgoIntersection copy = this;
        copy.Parameter(p);
        return copy;
    }

    public readonly AlgoIntersection WithTolerance(float t)
    {
        AlgoIntersection copy = this;
        copy.Tolerance(t);
        return copy;
    }

    public readonly AlgoIntersection WithState(Containment st)
    {
        AlgoIntersection copy = this;
        copy.State(st);
        return copy;
    }

    public readonly bool Equals(AlgoIntersection other)
        => _orien == other._orien && _segIndex == other._segIndex && _index == other._index
           && _level == other._level && _param.Equals(other._param) && _toler.Equals(other._toler)
           && _state == other._state;

    public override readonly bool Equals(object? obj) => obj is AlgoIntersection other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_orien, _segIndex, _index, _level, _param, _toler, _state);

    public static bool operator ==(in AlgoIntersection left, in AlgoIntersection right)
        => left.Equals(right);

    public static bool operator !=(in AlgoIntersection left, in AlgoIntersection right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("HLRAlgo_Intersection", _level, _segIndex, _index, _param, _toler)} {_orien} {_state}";
}
