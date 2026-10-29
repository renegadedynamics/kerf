// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Couple.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Couple.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A couple of indices with an analyzed flag and an angle. In IntPolyh_MaillageAffinage, a couple of interfering
/// triangles with the cosine of the angle between them.
/// </summary>
/// <remarks>
/// OCCT's default couple is (-1, -1), not analyzed, angle -2; <c>default(IntPolyh_Couple)</c> is all zeros, so ported
/// code writes <c>new IntPolyh_Couple()</c>. <see cref="Equals(IntPolyh_Couple)"/> and <c>==</c> compare the fields
/// exactly; OCCT's <c>operator==</c> (the same two indices in either order) is <see cref="IsEqual"/>, and maps in ported
/// code take <see cref="Hasher"/>.
/// </remarks>
public partial struct IntPolyh_Couple : IEquatable<IntPolyh_Couple>
{
    private int myIndex1;
    private int myIndex2;
    private bool myAnalyzed;
    private double myAngle;

    public IntPolyh_Couple()
    {
        myIndex1 = -1;
        myIndex2 = -1;
        myAnalyzed = false;
        myAngle = -2.0;
    }

    public IntPolyh_Couple(int theTriangle1, int theTriangle2, double theAngle = -2.0)
    {
        myIndex1 = theTriangle1;
        myIndex2 = theTriangle2;
        myAnalyzed = false;
        myAngle = theAngle;
    }

    public readonly int FirstValue() => myIndex1;

    public readonly int SecondValue() => myIndex2;

    /// <summary>True if the couple has been analyzed.</summary>
    public readonly bool IsAnalyzed() => myAnalyzed;

    public readonly double Angle() => myAngle;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoupleValue(int theInd1, int theInd2)
    {
        myIndex1 = theInd1;
        myIndex2 = theInd2;
    }

    internal void SetAnalyzed(bool theAnalyzed) => myAnalyzed = theAnalyzed;

    internal void SetAngle(double theAngle) => myAngle = theAngle;

    public readonly IntPolyh_Couple WithCoupleValue(int theInd1, int theInd2)
    {
        IntPolyh_Couple aCopy = this;
        aCopy.SetCoupleValue(theInd1, theInd2);
        return aCopy;
    }

    public readonly IntPolyh_Couple WithAnalyzed(bool theAnalyzed)
    {
        IntPolyh_Couple aCopy = this;
        aCopy.SetAnalyzed(theAnalyzed);
        return aCopy;
    }

    public readonly IntPolyh_Couple WithAngle(double theAngle)
    {
        IntPolyh_Couple aCopy = this;
        aCopy.SetAngle(theAngle);
        return aCopy;
    }

    /// <summary>OCCT's equality: the same two indices, in either order.</summary>
    public readonly bool IsEqual(in IntPolyh_Couple theOther)
        => (myIndex1 == theOther.myIndex1 && myIndex2 == theOther.myIndex2)
           || (myIndex1 == theOther.myIndex2 && myIndex2 == theOther.myIndex1);

    /// <summary>
    /// OCCT's <c>std::hash</c> and <c>operator==</c>: <see cref="IsEqual"/>, and a hash of the ordered pair (OCCT hashes
    /// its bytes; the value differs, which IntPolyh never iterates on).
    /// </summary>
    public sealed class Hasher : IEqualityComparer<IntPolyh_Couple>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(IntPolyh_Couple theC1, IntPolyh_Couple theC2) => theC1.IsEqual(theC2);

        public int GetHashCode(IntPolyh_Couple theCouple)
        {
            int a0 = theCouple.FirstValue(), a1 = theCouple.SecondValue();
            return a0 > a1 ? HashCode.Combine(a1, a0) : HashCode.Combine(a0, a1);
        }
    }

    /// <summary>Writes the couple with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter theStream)
    {
        theStream.Write($"\nCouple({IntPolyh_Format.D(i, 3)}) : {IntPolyh_Format.D(myIndex1, 5)} "
                        + $"{IntPolyh_Format.D(myIndex2, 5)} {IntPolyh_Format.D(myAnalyzed ? 1 : 0, 3)} "
                        + $"{IntPolyh_Format.F(myAngle, 5)}\n");
    }

    public readonly bool Equals(IntPolyh_Couple other)
        => myIndex1 == other.myIndex1 && myIndex2 == other.myIndex2 && myAnalyzed == other.myAnalyzed
           && myAngle.Equals(other.myAngle);

    public override readonly bool Equals(object? obj) => obj is IntPolyh_Couple other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myIndex1, myIndex2, myAnalyzed, myAngle);

    public static bool operator ==(in IntPolyh_Couple theLeft, in IntPolyh_Couple theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_Couple theLeft, in IntPolyh_Couple theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntPolyh_Couple({myIndex1}, {myIndex2}, {(myAnalyzed ? "analyzed" : "not analyzed")}, {gp.Format("angle", myAngle)})";
}
