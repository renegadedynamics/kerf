// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CircSection.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CircSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A section of a fillet: a circle or a line, with its parameter range.</summary>
/// <remarks>A C++ value class, so a struct. <c>new()</c> and <c>default</c> differ only in the default circle and line.</remarks>
public partial struct ChFiDS_CircSection : IEquatable<ChFiDS_CircSection>
{
    private gp_Circ myCirc;
    private gp_Lin myLin;
    private double myF;
    private double myL;

    public ChFiDS_CircSection()
    {
        myCirc = new gp_Circ();
        myLin = new gp_Lin();
        myF = 0.0;
        myL = 0.0;
    }

    internal void Set(in gp_Circ C, double F, double L)
    {
        myCirc = C;
        myF = F;
        myL = L;
    }

    internal void Set(in gp_Lin C, double F, double L)
    {
        myLin = C;
        myF = F;
        myL = L;
    }

    /// <summary>A copy with the circle <paramref name="C"/> on [<paramref name="F"/>, <paramref name="L"/>].</summary>
    public readonly ChFiDS_CircSection WithCircle(in gp_Circ C, double F, double L)
    {
        ChFiDS_CircSection aCopy = this;
        aCopy.Set(C, F, L);
        return aCopy;
    }

    /// <summary>A copy with the line <paramref name="C"/> on [<paramref name="F"/>, <paramref name="L"/>].</summary>
    public readonly ChFiDS_CircSection WithLine(in gp_Lin C, double F, double L)
    {
        ChFiDS_CircSection aCopy = this;
        aCopy.Set(C, F, L);
        return aCopy;
    }

    public readonly void Get(out gp_Circ C, out double F, out double L)
    {
        C = myCirc;
        F = myF;
        L = myL;
    }

    public readonly void Get(out gp_Lin C, out double F, out double L)
    {
        C = myLin;
        F = myF;
        L = myL;
    }

    public readonly bool Equals(ChFiDS_CircSection other)
        => myCirc.Equals(other.myCirc) && myLin.Equals(other.myLin) && myF.Equals(other.myF) && myL.Equals(other.myL);

    public override readonly bool Equals(object? obj) => obj is ChFiDS_CircSection other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myCirc, myLin, myF, myL);

    public static bool operator ==(in ChFiDS_CircSection theLeft, in ChFiDS_CircSection theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in ChFiDS_CircSection theLeft, in ChFiDS_CircSection theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("ChFiDS_CircSection", myF, myL);
}
