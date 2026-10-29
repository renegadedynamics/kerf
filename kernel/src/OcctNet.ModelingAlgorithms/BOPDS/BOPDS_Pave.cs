// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>A vertex on an edge: the vertex's index in the data structure and its parameter on the edge.</summary>
/// <remarks>
/// <para>
/// OCCT's two comparisons are kept as OCCT defines them, and they are not consistent with each other:
/// <see cref="IsLess"/> (C++'s <c>operator&lt;</c>) compares parameters only, <see cref="IsEqual"/> (C++'s
/// <c>operator==</c>, and <see cref="Equals(BOPDS_Pave)"/> here) compares index and parameter. Sorts of paves use
/// <see cref="LessThan"/> with <see cref="std_sort"/>, so that paves with equal parameters stay in MSVC's order.
/// </para>
/// <para>The default pave has the index -1 and the parameter 99, as OCCT's.</para>
/// </remarks>
public partial struct BOPDS_Pave : IEquatable<BOPDS_Pave>
{
    private int myIndex;
    private double myParameter;

    /// <summary>The pave (-1, 99).</summary>
    public BOPDS_Pave()
    {
        myIndex = -1;
        myParameter = 99.0;
    }

    /// <summary>The pave of the vertex <paramref name="theIndex"/> at <paramref name="theParameter"/>.</summary>
    public BOPDS_Pave(int theIndex, double theParameter)
    {
        myIndex = theIndex;
        myParameter = theParameter;
    }

    internal void SetIndex(int theIndex) => myIndex = theIndex;

    /// <summary>The index of the vertex.</summary>
    public readonly int Index() => myIndex;

    internal void SetParameter(double theParameter) => myParameter = theParameter;

    /// <summary>The parameter of the vertex on the edge.</summary>
    public readonly double Parameter() => myParameter;

    /// <summary>The index and the parameter.</summary>
    public readonly void Contents(out int theIndex, out double theParameter)
    {
        theIndex = myIndex;
        theParameter = myParameter;
    }

    /// <summary>True if this pave's parameter is less than the other's (C++'s <c>operator&lt;</c>).</summary>
    public readonly bool IsLess(in BOPDS_Pave theOther) => myParameter < theOther.myParameter;

    /// <summary>True if the index and the parameter are equal (C++'s <c>operator==</c>).</summary>
    public readonly bool IsEqual(in BOPDS_Pave theOther) => myIndex == theOther.myIndex && myParameter == theOther.myParameter;

    /// <summary>Writes the pave as OCCT's <c>Dump</c> prints it: <c> { index parameter}</c>.</summary>
    public readonly void Dump(TextWriter theOS)
    {
        theOS.Write(string.Format(CultureInfo.InvariantCulture, " {{ {0} {1,-7:F3}}}", myIndex, myParameter));
    }

    // ---- OCCT.NET additions

    /// <summary>C++'s <c>operator&lt;</c> for <see cref="std_sort"/>.</summary>
    public readonly struct LessThan : std_heap.ILess<BOPDS_Pave>
    {
        public bool Less(BOPDS_Pave theLeft, BOPDS_Pave theRight) => theLeft.IsLess(theRight);
    }

    public readonly BOPDS_Pave WithIndex(int theIndex)
    {
        BOPDS_Pave aCopy = this;
        aCopy.SetIndex(theIndex);
        return aCopy;
    }

    public readonly BOPDS_Pave WithParameter(double theParameter)
    {
        BOPDS_Pave aCopy = this;
        aCopy.SetParameter(theParameter);
        return aCopy;
    }

    public readonly bool Equals(BOPDS_Pave theOther) => IsEqual(theOther);

    public override readonly bool Equals(object? theObj) => theObj is BOPDS_Pave anOther && IsEqual(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myIndex, myParameter == 0.0 ? 0.0 : myParameter);

    public static bool operator ==(BOPDS_Pave theLeft, BOPDS_Pave theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(BOPDS_Pave theLeft, BOPDS_Pave theRight) => !theLeft.IsEqual(theRight);

    public override readonly string ToString() => gp.Format("BOPDS_Pave", myIndex, myParameter);
}
