// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_SectionAttribute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The flags of the face/face intersection: approximation of the curves, p-curves on each face.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class. The setters, which OCCT overloads with the getters' names
/// (<c>Approximation(bool)</c>), are internal (translation rules, section 3); the public twins are the
/// <c>With</c> forms. The parameterless constructor sets every flag (OCCT's default); <c>default</c> clears them.
/// </remarks>
public partial struct BOPAlgo_SectionAttribute : IEquatable<BOPAlgo_SectionAttribute>
{
    private bool myApproximation;
    private bool myPCurve1;
    private bool myPCurve2;

    /// <summary>All flags set.</summary>
    public BOPAlgo_SectionAttribute()
    {
        myApproximation = true;
        myPCurve1 = true;
        myPCurve2 = true;
    }

    /// <summary>The given flags.</summary>
    public BOPAlgo_SectionAttribute(bool theAproximation, bool thePCurveOnS1, bool thePCurveOnS2)
    {
        myApproximation = theAproximation;
        myPCurve1 = thePCurveOnS1;
        myPCurve2 = thePCurveOnS2;
    }

    /// <summary>Sets the approximation flag.</summary>
    internal void Approximation(bool theApprox) => myApproximation = theApprox;

    /// <summary>Sets the flag of p-curves on the first face.</summary>
    internal void PCurveOnS1(bool thePCurveOnS1) => myPCurve1 = thePCurveOnS1;

    /// <summary>Sets the flag of p-curves on the second face.</summary>
    internal void PCurveOnS2(bool thePCurveOnS2) => myPCurve2 = thePCurveOnS2;

    /// <summary>The approximation flag.</summary>
    public readonly bool Approximation() => myApproximation;

    /// <summary>The flag of p-curves on the first face.</summary>
    public readonly bool PCurveOnS1() => myPCurve1;

    /// <summary>The flag of p-curves on the second face.</summary>
    public readonly bool PCurveOnS2() => myPCurve2;

    // ---- OCCT.NET additions

    public readonly BOPAlgo_SectionAttribute WithApproximation(bool theApprox)
    {
        BOPAlgo_SectionAttribute aCopy = this;
        aCopy.Approximation(theApprox);
        return aCopy;
    }

    public readonly BOPAlgo_SectionAttribute WithPCurveOnS1(bool thePCurveOnS1)
    {
        BOPAlgo_SectionAttribute aCopy = this;
        aCopy.PCurveOnS1(thePCurveOnS1);
        return aCopy;
    }

    public readonly BOPAlgo_SectionAttribute WithPCurveOnS2(bool thePCurveOnS2)
    {
        BOPAlgo_SectionAttribute aCopy = this;
        aCopy.PCurveOnS2(thePCurveOnS2);
        return aCopy;
    }

    public readonly bool Equals(BOPAlgo_SectionAttribute theOther) =>
        myApproximation == theOther.myApproximation && myPCurve1 == theOther.myPCurve1 && myPCurve2 == theOther.myPCurve2;

    public override readonly bool Equals(object? theObj) => theObj is BOPAlgo_SectionAttribute anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myApproximation, myPCurve1, myPCurve2);

    public static bool operator ==(BOPAlgo_SectionAttribute theLeft, BOPAlgo_SectionAttribute theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BOPAlgo_SectionAttribute theLeft, BOPAlgo_SectionAttribute theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        $"BOPAlgo_SectionAttribute(Approximation={myApproximation}, PCurveOnS1={myPCurve1}, PCurveOnS2={myPCurve2})";
}
