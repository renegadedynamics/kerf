// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Parameter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Parameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection on a line of the <see cref="Hatch_Hatcher"/>: a parameter, and a flag true when the parameter
/// starts an interval; with the index of the trimming element and the parameter on it.
/// </summary>
/// <remarks>
/// A value type (decision 663). Its fields are internal, as OCCT's friends Hatch_Line and Hatch_Hatcher read them;
/// nothing writes them after construction. <c>default</c> equals OCCT's default constructor.
/// </remarks>
public readonly struct Hatch_Parameter : IEquatable<Hatch_Parameter>
{
    internal readonly double myPar1;
    internal readonly bool myStart;
    internal readonly int myIndex;
    internal readonly double myPar2;

    /// <summary>A parameter 0, not starting an interval.</summary>
    public Hatch_Parameter()
    {
        myPar1 = 0.0;
        myStart = false;
        myIndex = 0;
        myPar2 = 0.0;
    }

    /// <summary>The parameter <paramref name="Par1"/> on the line, starting an interval if <paramref name="Start"/>.</summary>
    public Hatch_Parameter(double Par1, bool Start, int Index = 0, double Par2 = 0)
    {
        myPar1 = Par1;
        myStart = Start;
        myIndex = Index;
        myPar2 = Par2;
    }

    public bool Equals(Hatch_Parameter other) =>
        myPar1.Equals(other.myPar1) && myStart == other.myStart && myIndex == other.myIndex
        && myPar2.Equals(other.myPar2);

    public override bool Equals(object? obj) => obj is Hatch_Parameter other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(myPar1, myStart, myIndex, myPar2);

    public static bool operator ==(Hatch_Parameter left, Hatch_Parameter right) => left.Equals(right);

    public static bool operator !=(Hatch_Parameter left, Hatch_Parameter right) => !left.Equals(right);

    public override string ToString() =>
        $"{gp.Format("Hatch_Parameter", myPar1, myPar2)} start={myStart} index={myIndex}";
}
