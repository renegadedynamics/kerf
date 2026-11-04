// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_SectionAttribute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans;

/// <summary>The flags of the face/face intersection: approximation of the curves, p-curves on each face.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class. The setters, which OCCT overloads with the getters' names
/// (<c>Approximation(bool)</c>), are internal (translation rules, section 3); the public twins are the
/// <c>With</c> forms. The parameterless constructor sets every flag (OCCT's default); <c>default</c> clears them.
/// </remarks>
internal partial struct SectionAttribute : IEquatable<SectionAttribute>
{
    private bool _approximation;
    private bool _pCurve1;
    private bool _pCurve2;

    /// <summary>All flags set.</summary>
    public SectionAttribute()
    {
        _approximation = true;
        _pCurve1 = true;
        _pCurve2 = true;
    }

    /// <summary>The given flags.</summary>
    public SectionAttribute(bool aproximation, bool pCurveOnS1, bool pCurveOnS2)
    {
        _approximation = aproximation;
        _pCurve1 = pCurveOnS1;
        _pCurve2 = pCurveOnS2;
    }

    /// <summary>Sets the approximation flag.</summary>
    internal void Approximation(bool approx) => _approximation = approx;

    /// <summary>Sets the flag of p-curves on the first face.</summary>
    internal void PCurveOnS1(bool pCurveOnS1) => _pCurve1 = pCurveOnS1;

    /// <summary>Sets the flag of p-curves on the second face.</summary>
    internal void PCurveOnS2(bool pCurveOnS2) => _pCurve2 = pCurveOnS2;

    /// <summary>The approximation flag.</summary>
    public readonly bool Approximation() => _approximation;

    /// <summary>The flag of p-curves on the first face.</summary>
    public readonly bool PCurveOnS1() => _pCurve1;

    /// <summary>The flag of p-curves on the second face.</summary>
    public readonly bool PCurveOnS2() => _pCurve2;

    // ---- OCCT.NET additions

    public readonly SectionAttribute WithApproximation(bool approx)
    {
        SectionAttribute copy = this;
        copy.Approximation(approx);
        return copy;
    }

    public readonly SectionAttribute WithPCurveOnS1(bool pCurveOnS1)
    {
        SectionAttribute copy = this;
        copy.PCurveOnS1(pCurveOnS1);
        return copy;
    }

    public readonly SectionAttribute WithPCurveOnS2(bool pCurveOnS2)
    {
        SectionAttribute copy = this;
        copy.PCurveOnS2(pCurveOnS2);
        return copy;
    }

    public readonly bool Equals(SectionAttribute other) =>
        _approximation == other._approximation && _pCurve1 == other._pCurve1 && _pCurve2 == other._pCurve2;

    public override readonly bool Equals(object? obj) => obj is SectionAttribute other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_approximation, _pCurve1, _pCurve2);

    public static bool operator ==(SectionAttribute left, SectionAttribute right) => left.Equals(right);

    public static bool operator !=(SectionAttribute left, SectionAttribute right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"BOPAlgo_SectionAttribute(Approximation={_approximation}, PCurveOnS1={_pCurve1}, PCurveOnS2={_pCurve2})";
}
