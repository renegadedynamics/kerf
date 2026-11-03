// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Root.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// A root of a function of one variable, for the edge/edge and edge/surface algorithms: its value, its type, the
/// states before and after it, and the interval [t1, t2] it was found in with the function's values f(t1), f(t2).
/// </summary>
/// <remarks>A struct, as OCCT's by-value class; the setters are internal (translation rules, section 3).</remarks>
public partial struct IntTools_Root : IEquatable<IntTools_Root>
{
    private double myRoot;
    private int myType;
    private double myLayerHeight;
    private TopAbs_State myStateBefore;
    private TopAbs_State myStateAfter;
    private double myt1;
    private double myt2;
    private double myf1;
    private double myf2;

    /// <summary>An empty root: value 0, type -1, states unknown.</summary>
    /// <remarks><c>default(IntTools_Root)</c> differs: type 0 and states IN (the enum's zero).</remarks>
    public IntTools_Root()
    {
        myRoot = 0.0;
        myType = -1;
        myLayerHeight = 0.0;
        myStateBefore = TopAbs_UNKNOWN;
        myStateAfter = TopAbs_UNKNOWN;
        myt1 = 0.0;
        myt2 = 0.0;
        myf1 = 0.0;
        myf2 = 0.0;
    }

    /// <summary>A root with its value and type.</summary>
    public IntTools_Root(double aRoot, int aType)
    {
        myLayerHeight = 0.0;
        myStateBefore = TopAbs_UNKNOWN;
        myStateAfter = TopAbs_UNKNOWN;
        myt1 = 0.0;
        myt2 = 0.0;
        myf1 = 0.0;
        myf2 = 0.0;
        myRoot = aRoot;
        myType = aType;
    }

    internal void SetRoot(double aRoot) => myRoot = aRoot;

    internal void SetType(int aType) => myType = aType;

    internal void SetStateBefore(TopAbs_State aState) => myStateBefore = aState;

    internal void SetStateAfter(TopAbs_State aState) => myStateAfter = aState;

    /// <summary>Not used in the edge/edge algorithm.</summary>
    internal void SetLayerHeight(double aHeight) => myLayerHeight = aHeight;

    /// <summary>Sets the interval [t1, t2] the root was found in and the function's values on its bounds.</summary>
    internal void SetInterval(double t1, double t2, double f1, double f2)
    {
        myt1 = t1;
        myt2 = t2;
        myf1 = f1;
        myf2 = f2;
    }

    /// <summary>The interval [t1, t2] the root was found in and the function's values f(t1), f(t2).</summary>
    public readonly void Interval(out double t1, out double t2, out double f1, out double f2)
    {
        t1 = myt1;
        t2 = myt2;
        f1 = myf1;
        f2 = myf2;
    }

    /// <summary>The root's value.</summary>
    public readonly double Root() => myRoot;

    /// <summary>
    /// The type of the root: 0 simple (found by bisection), 1 pure (zero on all of [t1, t2]), 2 smart (f1 = 0 and
    /// f2 != 0 or the reverse, found by the Fibonacci method).
    /// </summary>
    public readonly int Type() => myType;

    public readonly TopAbs_State StateBefore() => myStateBefore;

    public readonly TopAbs_State StateAfter() => myStateAfter;

    /// <summary>Not used in the edge/edge algorithm.</summary>
    public readonly double LayerHeight() => myLayerHeight;

    /// <summary>True for the state pairs (before, after) OUT/IN, OUT/ON, ON/OUT and IN/OUT.</summary>
    public readonly bool IsValid()
    {
        if (myStateBefore == TopAbs_OUT && myStateAfter == TopAbs_IN)
        {
            return true;
        }
        if (myStateBefore == TopAbs_OUT && myStateAfter == TopAbs_ON)
        {
            return true;
        }
        if (myStateBefore == TopAbs_ON && myStateAfter == TopAbs_OUT)
        {
            return true;
        }
        if (myStateBefore == TopAbs_IN && myStateAfter == TopAbs_OUT)
        {
            return true;
        }
        return false;
    }

    // ---- OCCT.NET additions

    public readonly IntTools_Root WithRoot(double aRoot)
    {
        IntTools_Root aCopy = this;
        aCopy.SetRoot(aRoot);
        return aCopy;
    }

    public readonly IntTools_Root WithType(int aType)
    {
        IntTools_Root aCopy = this;
        aCopy.SetType(aType);
        return aCopy;
    }

    public readonly IntTools_Root WithStateBefore(TopAbs_State aState)
    {
        IntTools_Root aCopy = this;
        aCopy.SetStateBefore(aState);
        return aCopy;
    }

    public readonly IntTools_Root WithStateAfter(TopAbs_State aState)
    {
        IntTools_Root aCopy = this;
        aCopy.SetStateAfter(aState);
        return aCopy;
    }

    public readonly IntTools_Root WithLayerHeight(double aHeight)
    {
        IntTools_Root aCopy = this;
        aCopy.SetLayerHeight(aHeight);
        return aCopy;
    }

    public readonly IntTools_Root WithInterval(double t1, double t2, double f1, double f2)
    {
        IntTools_Root aCopy = this;
        aCopy.SetInterval(t1, t2, f1, f2);
        return aCopy;
    }

    public readonly bool Equals(IntTools_Root theOther) =>
        myRoot.Equals(theOther.myRoot) && myType == theOther.myType && myLayerHeight.Equals(theOther.myLayerHeight)
        && myStateBefore == theOther.myStateBefore && myStateAfter == theOther.myStateAfter && myt1.Equals(theOther.myt1)
        && myt2.Equals(theOther.myt2) && myf1.Equals(theOther.myf1) && myf2.Equals(theOther.myf2);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_Root anOther && Equals(anOther);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(myRoot);
        aHash.Add(myType);
        aHash.Add(myLayerHeight);
        aHash.Add(myStateBefore);
        aHash.Add(myStateAfter);
        aHash.Add(myt1);
        aHash.Add(myt2);
        aHash.Add(myf1);
        aHash.Add(myf2);
        return aHash.ToHashCode();
    }

    public static bool operator ==(IntTools_Root theLeft, IntTools_Root theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntTools_Root theLeft, IntTools_Root theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        $"IntTools_Root({Constants.Format("", myRoot, myt1, myt2, myf1, myf2).Trim('(', ')')}; type {myType}, {myStateBefore}/{myStateAfter})";
}
