// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Coincidence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The coincidence of an edge with the boundary of a face, for the hidden-line removal: in 2D, the face's edge and
/// the parameter on it; in 3D, the states before and after.
/// </summary>
/// <remarks>A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same. The setters are internal.</remarks>
public partial struct HLRAlgo_Coincidence : IEquatable<HLRAlgo_Coincidence>
{
    private int myFE;
    private double myParam;
    private Containment myStBef;
    private Containment myStAft;

    public HLRAlgo_Coincidence()
    {
        myFE = 0;
        myParam = 0.0;
        myStBef = In;
        myStAft = In;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set2D(int FE, double Param)
    {
        myFE = FE;
        myParam = Param;
    }

    internal void SetState3D(Containment stbef, Containment staft)
    {
        myStBef = stbef;
        myStAft = staft;
    }

    // Queries

    public readonly void Value2D(out int FE, out double Param)
    {
        FE = myFE;
        Param = myParam;
    }

    public readonly void State3D(out Containment stbef, out Containment staft)
    {
        stbef = myStBef;
        staft = myStAft;
    }

    public readonly HLRAlgo_Coincidence With2D(int FE, double Param)
    {
        HLRAlgo_Coincidence aCopy = this;
        aCopy.Set2D(FE, Param);
        return aCopy;
    }

    public readonly HLRAlgo_Coincidence WithState3D(Containment stbef, Containment staft)
    {
        HLRAlgo_Coincidence aCopy = this;
        aCopy.SetState3D(stbef, staft);
        return aCopy;
    }

    public readonly bool Equals(HLRAlgo_Coincidence other)
        => myFE == other.myFE && myParam.Equals(other.myParam) && myStBef == other.myStBef
           && myStAft == other.myStAft;

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_Coincidence other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myFE, myParam, myStBef, myStAft);

    public static bool operator ==(in HLRAlgo_Coincidence theLeft, in HLRAlgo_Coincidence theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_Coincidence theLeft, in HLRAlgo_Coincidence theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{Constants.Format("HLRAlgo_Coincidence", myFE, myParam)} {myStBef} {myStAft}";
}
