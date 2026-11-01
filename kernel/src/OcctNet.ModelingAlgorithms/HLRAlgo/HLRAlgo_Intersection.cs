// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection of an edge with the boundary of a face, for the hidden-line removal: its orientation, level,
/// segment and edge indices, parameter, tolerance and state.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the orientation and the state unset; here they are the enums' first values). The setters (OCCT's overloads of the
/// getters' names) are internal.
/// </remarks>
public partial struct HLRAlgo_Intersection : IEquatable<HLRAlgo_Intersection>
{
    private TopAbs_Orientation myOrien;
    private int mySegIndex;
    private int myIndex;
    private int myLevel;
    private double myParam;
    private float myToler;
    private TopAbs_State myState;

    public HLRAlgo_Intersection()
    {
        mySegIndex = 0;
        myIndex = 0;
        myLevel = 0;
        myParam = 0.0;
        myToler = 0.0f;
    }

    public HLRAlgo_Intersection(TopAbs_Orientation Ori, int Lev, int SegInd, int Ind, double P, float Tol,
                                TopAbs_State S)
    {
        myOrien = Ori;
        mySegIndex = SegInd;
        myIndex = Ind;
        myLevel = Lev;
        myParam = P;
        myToler = Tol;
        myState = S;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Orientation(TopAbs_Orientation Ori) => myOrien = Ori;

    internal void Level(int Lev) => myLevel = Lev;

    internal void SegIndex(int SegInd) => mySegIndex = SegInd;

    internal void Index(int Ind) => myIndex = Ind;

    internal void Parameter(double P) => myParam = P;

    internal void Tolerance(float T) => myToler = T;

    internal void State(TopAbs_State St) => myState = St;

    // Queries

    public readonly TopAbs_Orientation Orientation() => myOrien;

    public readonly int Level() => myLevel;

    public readonly int SegIndex() => mySegIndex;

    public readonly int Index() => myIndex;

    public readonly double Parameter() => myParam;

    public readonly float Tolerance() => myToler;

    public readonly TopAbs_State State() => myState;

    public readonly HLRAlgo_Intersection WithOrientation(TopAbs_Orientation Ori)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.Orientation(Ori);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithLevel(int Lev)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.Level(Lev);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithSegIndex(int SegInd)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.SegIndex(SegInd);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithIndex(int Ind)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.Index(Ind);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithParameter(double P)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.Parameter(P);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithTolerance(float T)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.Tolerance(T);
        return aCopy;
    }

    public readonly HLRAlgo_Intersection WithState(TopAbs_State St)
    {
        HLRAlgo_Intersection aCopy = this;
        aCopy.State(St);
        return aCopy;
    }

    public readonly bool Equals(HLRAlgo_Intersection other)
        => myOrien == other.myOrien && mySegIndex == other.mySegIndex && myIndex == other.myIndex
           && myLevel == other.myLevel && myParam.Equals(other.myParam) && myToler.Equals(other.myToler)
           && myState == other.myState;

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_Intersection other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myOrien, mySegIndex, myIndex, myLevel, myParam, myToler, myState);

    public static bool operator ==(in HLRAlgo_Intersection theLeft, in HLRAlgo_Intersection theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_Intersection theLeft, in HLRAlgo_Intersection theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("HLRAlgo_Intersection", myLevel, mySegIndex, myIndex, myParam, myToler)} {myOrien} {myState}";
}
