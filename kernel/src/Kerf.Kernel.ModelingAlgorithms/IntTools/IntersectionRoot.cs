// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Root.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// A root of a function of one variable, for the edge/edge and edge/surface algorithms: its value, its type, the
/// states before and after it, and the interval [t1, t2] it was found in with the function's values f(t1), f(t2).
/// </summary>
/// <remarks>A struct, as OCCT's by-value class; the setters are internal (translation rules, section 3).</remarks>
internal partial struct IntersectionRoot : IEquatable<IntersectionRoot>
{
    private double _root;
    private int _type;
    private double _layerHeight;
    private Containment _stateBefore;
    private Containment _stateAfter;
    private double _myt1;
    private double _myt2;
    private double _myf1;
    private double _myf2;

    /// <summary>An empty root: value 0, type -1, states unknown.</summary>
    /// <remarks><c>default(IntTools_Root)</c> differs: type 0 and states IN (the enum's zero).</remarks>
    public IntersectionRoot()
    {
        _root = 0.0;
        _type = -1;
        _layerHeight = 0.0;
        _stateBefore = Unknown;
        _stateAfter = Unknown;
        _myt1 = 0.0;
        _myt2 = 0.0;
        _myf1 = 0.0;
        _myf2 = 0.0;
    }

    /// <summary>A root with its value and type.</summary>
    public IntersectionRoot(double aRoot, int aType)
    {
        _layerHeight = 0.0;
        _stateBefore = Unknown;
        _stateAfter = Unknown;
        _myt1 = 0.0;
        _myt2 = 0.0;
        _myf1 = 0.0;
        _myf2 = 0.0;
        _root = aRoot;
        _type = aType;
    }

    internal void SetRoot(double aRoot) => _root = aRoot;

    internal void SetType(int aType) => _type = aType;

    internal void SetStateBefore(Containment aState) => _stateBefore = aState;

    internal void SetStateAfter(Containment aState) => _stateAfter = aState;

    /// <summary>Not used in the edge/edge algorithm.</summary>
    internal void SetLayerHeight(double aHeight) => _layerHeight = aHeight;

    /// <summary>Sets the interval [t1, t2] the root was found in and the function's values on its bounds.</summary>
    internal void SetInterval(double t1, double t2, double f1, double f2)
    {
        _myt1 = t1;
        _myt2 = t2;
        _myf1 = f1;
        _myf2 = f2;
    }

    /// <summary>The interval [t1, t2] the root was found in and the function's values f(t1), f(t2).</summary>
    public readonly void Interval(out double t1, out double t2, out double f1, out double f2)
    {
        t1 = _myt1;
        t2 = _myt2;
        f1 = _myf1;
        f2 = _myf2;
    }

    /// <summary>The root's value.</summary>
    public readonly double Root() => _root;

    /// <summary>
    /// The type of the root: 0 simple (found by bisection), 1 pure (zero on all of [t1, t2]), 2 smart (f1 = 0 and
    /// f2 != 0 or the reverse, found by the Fibonacci method).
    /// </summary>
    public readonly int Type() => _type;

    public readonly Containment StateBefore() => _stateBefore;

    public readonly Containment StateAfter() => _stateAfter;

    /// <summary>Not used in the edge/edge algorithm.</summary>
    public readonly double LayerHeight() => _layerHeight;

    /// <summary>True for the state pairs (before, after) OUT/IN, OUT/ON, ON/OUT and IN/OUT.</summary>
    public readonly bool IsValid()
    {
        if (_stateBefore == Out && _stateAfter == In)
        {
            return true;
        }
        if (_stateBefore == Out && _stateAfter == On)
        {
            return true;
        }
        if (_stateBefore == On && _stateAfter == Out)
        {
            return true;
        }
        if (_stateBefore == In && _stateAfter == Out)
        {
            return true;
        }
        return false;
    }

    // ---- OCCT.NET additions

    public readonly IntersectionRoot WithRoot(double aRoot)
    {
        IntersectionRoot copy = this;
        copy.SetRoot(aRoot);
        return copy;
    }

    public readonly IntersectionRoot WithType(int aType)
    {
        IntersectionRoot copy = this;
        copy.SetType(aType);
        return copy;
    }

    public readonly IntersectionRoot WithStateBefore(Containment aState)
    {
        IntersectionRoot copy = this;
        copy.SetStateBefore(aState);
        return copy;
    }

    public readonly IntersectionRoot WithStateAfter(Containment aState)
    {
        IntersectionRoot copy = this;
        copy.SetStateAfter(aState);
        return copy;
    }

    public readonly IntersectionRoot WithLayerHeight(double aHeight)
    {
        IntersectionRoot copy = this;
        copy.SetLayerHeight(aHeight);
        return copy;
    }

    public readonly IntersectionRoot WithInterval(double t1, double t2, double f1, double f2)
    {
        IntersectionRoot copy = this;
        copy.SetInterval(t1, t2, f1, f2);
        return copy;
    }

    public readonly bool Equals(IntersectionRoot other) =>
        _root.Equals(other._root) && _type == other._type && _layerHeight.Equals(other._layerHeight)
        && _stateBefore == other._stateBefore && _stateAfter == other._stateAfter && _myt1.Equals(other._myt1)
        && _myt2.Equals(other._myt2) && _myf1.Equals(other._myf1) && _myf2.Equals(other._myf2);

    public override readonly bool Equals(object? obj) => obj is IntersectionRoot other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(_root);
        hash.Add(_type);
        hash.Add(_layerHeight);
        hash.Add(_stateBefore);
        hash.Add(_stateAfter);
        hash.Add(_myt1);
        hash.Add(_myt2);
        hash.Add(_myf1);
        hash.Add(_myf2);
        return hash.ToHashCode();
    }

    public static bool operator ==(IntersectionRoot left, IntersectionRoot right) => left.Equals(right);

    public static bool operator !=(IntersectionRoot left, IntersectionRoot right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"IntTools_Root({Constants.Format("", _root, _myt1, _myt2, _myf1, _myf2).Trim('(', ')')}; type {_type}, {_stateBefore}/{_stateAfter})";
}
