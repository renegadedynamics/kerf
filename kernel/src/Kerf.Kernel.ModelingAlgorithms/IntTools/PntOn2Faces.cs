// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOn2Faces.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOn2Faces.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>Two points on faces and a validity flag.</summary>
/// <remarks>A struct, as OCCT's by-value class; the setters are internal.</remarks>
internal partial struct PntOn2Faces : IEquatable<PntOn2Faces>
{
    private bool _isValid;
    private PntOnFace _pnt1;
    private PntOnFace _pnt2;

    /// <summary>Two default points, not valid.</summary>
    public PntOn2Faces()
    {
        _isValid = false;
        _pnt1 = new PntOnFace();
        _pnt2 = new PntOnFace();
    }

    /// <summary>The two points, not valid.</summary>
    public PntOn2Faces(in PntOnFace aP1, in PntOnFace aP2)
    {
        _isValid = false;
        _pnt1 = aP1;
        _pnt2 = aP2;
    }

    internal void SetP1(in PntOnFace aP) => _pnt1 = aP;

    internal void SetP2(in PntOnFace aP) => _pnt2 = aP;

    internal void SetValid(bool bF) => _isValid = bF;

    public readonly PntOnFace P1() => _pnt1;

    public readonly PntOnFace P2() => _pnt2;

    public readonly bool IsValid => _isValid;

    // ---- Kerf additions

    public readonly PntOn2Faces WithP1(in PntOnFace aP)
    {
        PntOn2Faces copy = this;
        copy.SetP1(aP);
        return copy;
    }

    public readonly PntOn2Faces WithP2(in PntOnFace aP)
    {
        PntOn2Faces copy = this;
        copy.SetP2(aP);
        return copy;
    }

    public readonly PntOn2Faces WithValid(bool bF)
    {
        PntOn2Faces copy = this;
        copy.SetValid(bF);
        return copy;
    }

    public readonly bool Equals(PntOn2Faces other) =>
        _isValid == other._isValid && _pnt1.Equals(other._pnt1) && _pnt2.Equals(other._pnt2);

    public override readonly bool Equals(object? obj) => obj is PntOn2Faces other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_isValid, _pnt1, _pnt2);

    public static bool operator ==(PntOn2Faces left, PntOn2Faces right) => left.Equals(right);

    public static bool operator !=(PntOn2Faces left, PntOn2Faces right) => !left.Equals(right);

    public override readonly string ToString() => $"IntTools_PntOn2Faces({_pnt1}, {_pnt2}, valid {_isValid})";
}
