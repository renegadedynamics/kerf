// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// An intersection point of a hatcher: the index of its supporting curve, the parameter on it, its position on it
/// (FORWARD at the start, INTERNAL, REVERSED at the end), the states before and after it, and whether it begins or
/// ends a segment of intersection.
/// </summary>
/// <remarks>
/// OCCT's value classes HatchGen_PointOnElement and HatchGen_PointOnHatching derive from this one, so they are
/// classes with copy constructors, copied where OCCT copies them (decision 5802). The setters are internal: the
/// hatcher fills the points it builds, and callers only read them.
/// </remarks>
public abstract class HatchGen_IntersectionPoint
{
    protected int myIndex;
    protected double myParam;
    protected TopAbs_Orientation myPosit;
    protected TopAbs_State myBefore;
    protected TopAbs_State myAfter;
    protected bool mySegBeg;
    protected bool mySegEnd;

    /// <summary>An empty intersection point (parameter RealLast(), position INTERNAL, states UNKNOWN).</summary>
    protected HatchGen_IntersectionPoint()
    {
        myIndex = 0;
        myParam = RealLast();
        myPosit = TopAbs_INTERNAL;
        myBefore = TopAbs_UNKNOWN;
        myAfter = TopAbs_UNKNOWN;
        mySegBeg = false;
        mySegEnd = false;
    }

    /// <summary>A copy of <paramref name="Other"/>'s fields (C++'s copy).</summary>
    protected HatchGen_IntersectionPoint(HatchGen_IntersectionPoint Other)
    {
        myIndex = Other.myIndex;
        myParam = Other.myParam;
        myPosit = Other.myPosit;
        myBefore = Other.myBefore;
        myAfter = Other.myAfter;
        mySegBeg = Other.mySegBeg;
        mySegEnd = Other.mySegEnd;
    }

    /// <summary>Sets the index of the supporting curve.</summary>
    internal void SetIndex(int Index)
    {
        myIndex = Index;
    }

    /// <summary>The index of the supporting curve.</summary>
    public int Index() => myIndex;

    /// <summary>Sets the parameter on the curve.</summary>
    internal void SetParameter(double Parameter)
    {
        myParam = Parameter;
    }

    /// <summary>The parameter on the curve.</summary>
    public double Parameter() => myParam;

    /// <summary>Sets the position of the point on the curve.</summary>
    internal void SetPosition(TopAbs_Orientation Position)
    {
        myPosit = Position;
    }

    /// <summary>The position of the point on the curve.</summary>
    public TopAbs_Orientation Position() => myPosit;

    /// <summary>Sets the transition state before the intersection.</summary>
    internal void SetStateBefore(TopAbs_State State)
    {
        myBefore = State;
    }

    /// <summary>The transition state before the intersection.</summary>
    public TopAbs_State StateBefore() => myBefore;

    /// <summary>Sets the transition state after the intersection.</summary>
    internal void SetStateAfter(TopAbs_State State)
    {
        myAfter = State;
    }

    /// <summary>The transition state after the intersection.</summary>
    public TopAbs_State StateAfter() => myAfter;

    /// <summary>Sets the flag that the point is the beginning of a segment.</summary>
    internal void SetSegmentBeginning(bool State = true)
    {
        mySegBeg = State;
    }

    /// <summary>True if the point is the beginning of a segment.</summary>
    public bool SegmentBeginning() => mySegBeg;

    /// <summary>Sets the flag that the point is the end of a segment.</summary>
    internal void SetSegmentEnd(bool State = true)
    {
        mySegEnd = State;
    }

    /// <summary>True if the point is the end of a segment.</summary>
    public bool SegmentEnd() => mySegEnd;

    /// <summary>Writes the point to <paramref name="theOS"/> (OCCT writes to std::cout).</summary>
    public abstract void Dump(TextWriter theOS, int Index = 0);

    /// <summary>The dump of the point.</summary>
    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    // The names Dump writes for a position and a state (as OCCT's switches).
    internal static string PositionName(TopAbs_Orientation thePosit) => thePosit switch
    {
        TopAbs_FORWARD => "FORWARD  (i.e. BEGIN  )",
        TopAbs_INTERNAL => "INTERNAL (i.e. MIDDLE )",
        TopAbs_REVERSED => "REVERSED (i.e. END    )",
        _ => "EXTERNAL (i.e. UNKNOWN)",
    };

    internal static string StateName(TopAbs_State theState) => theState switch
    {
        TopAbs_IN => "IN",
        TopAbs_OUT => "OUT",
        TopAbs_ON => "ON",
        _ => "UNKNOWN",
    };

    // std::cout's default formatting of a double (6 significant digits).
    internal static string Number(double theValue) => theValue.ToString("G6", CultureInfo.InvariantCulture);
}
