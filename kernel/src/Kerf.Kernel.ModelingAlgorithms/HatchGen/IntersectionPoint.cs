// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using System.Globalization;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// An intersection point of a hatcher: the index of its supporting curve, the parameter on it, its position on it
/// (FORWARD at the start, INTERNAL, REVERSED at the end), the states before and after it, and whether it begins or
/// ends a segment of intersection.
/// </summary>
/// <remarks>
/// OCCT's value classes HatchGen_PointOnElement and HatchGen_PointOnHatching derive from this one, so they are
/// classes with copy constructors, copied where OCCT copies them (decision 664). The setters are internal: the
/// hatcher fills the points it builds, and callers only read them.
/// </remarks>
public abstract class IntersectionPoint
{
    protected int _index;
    protected double _param;
    protected Orientation _posit;
    protected Containment _before;
    protected Containment _after;
    protected bool _segBeg;
    protected bool _segEnd;

    /// <summary>An empty intersection point (parameter RealLast(), position INTERNAL, states UNKNOWN).</summary>
    protected IntersectionPoint()
    {
        _index = 0;
        _param = RealLast();
        _posit = Internal;
        _before = Unknown;
        _after = Unknown;
        _segBeg = false;
        _segEnd = false;
    }

    /// <summary>A copy of <paramref name="other"/>'s fields (C++'s copy).</summary>
    protected IntersectionPoint(IntersectionPoint other)
    {
        _index = other._index;
        _param = other._param;
        _posit = other._posit;
        _before = other._before;
        _after = other._after;
        _segBeg = other._segBeg;
        _segEnd = other._segEnd;
    }

    /// <summary>Sets the index of the supporting curve.</summary>
    internal void SetIndex(int index)
    {
        _index = index;
    }

    /// <summary>The index of the supporting curve.</summary>
    public int Index() => _index;

    /// <summary>Sets the parameter on the curve.</summary>
    internal void SetParameter(double parameter)
    {
        _param = parameter;
    }

    /// <summary>The parameter on the curve.</summary>
    public double Parameter() => _param;

    /// <summary>Sets the position of the point on the curve.</summary>
    internal void SetPosition(Orientation position)
    {
        _posit = position;
    }

    /// <summary>The position of the point on the curve.</summary>
    public Orientation Position() => _posit;

    /// <summary>Sets the transition state before the intersection.</summary>
    internal void SetStateBefore(Containment state)
    {
        _before = state;
    }

    /// <summary>The transition state before the intersection.</summary>
    public Containment StateBefore() => _before;

    /// <summary>Sets the transition state after the intersection.</summary>
    internal void SetStateAfter(Containment state)
    {
        _after = state;
    }

    /// <summary>The transition state after the intersection.</summary>
    public Containment StateAfter() => _after;

    /// <summary>Sets the flag that the point is the beginning of a segment.</summary>
    internal void SetSegmentBeginning(bool state = true)
    {
        _segBeg = state;
    }

    /// <summary>True if the point is the beginning of a segment.</summary>
    public bool SegmentBeginning() => _segBeg;

    /// <summary>Sets the flag that the point is the end of a segment.</summary>
    internal void SetSegmentEnd(bool state = true)
    {
        _segEnd = state;
    }

    /// <summary>True if the point is the end of a segment.</summary>
    public bool SegmentEnd() => _segEnd;

    /// <summary>Writes the point to <paramref name="os"/> (OCCT writes to std::cout).</summary>
    public abstract void Dump(TextWriter os, int index = 0);

    /// <summary>The dump of the point.</summary>
    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    // The names Dump writes for a position and a state (as OCCT's switches).
    internal static string PositionName(Orientation posit) => posit switch
    {
        Forward => "FORWARD  (i.e. BEGIN  )",
        Internal => "INTERNAL (i.e. MIDDLE )",
        Reversed => "REVERSED (i.e. END    )",
        _ => "EXTERNAL (i.e. UNKNOWN)",
    };

    internal static string StateName(Containment state) => state switch
    {
        In => "IN",
        Out => "OUT",
        On => "ON",
        _ => "UNKNOWN",
    };

    // std::cout's default formatting of a double (6 significant digits).
    internal static string Number(double value) => value.ToString("G6", CultureInfo.InvariantCulture);
}
