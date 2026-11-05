// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CornerState.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CornerState.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The state of a corner of the contour to fill: the gap between the boundaries, the angle between their tangents,
/// whether both are constrained and the angle between their normals, and whether the Coons compatibility
/// conditions hold (else the corner is to be killed, with a scale).
/// </summary>
/// <remarks>
/// A value type (translation rules, section 3): OCCT copies it by value (<c>stcor[3] = stcor[2]</c>). Its setters
/// (<c>Gap(G)</c>, <c>TgtAng(Ang)</c>, <c>Constraint()</c>, <c>NorAng(Ang)</c>, <c>DoKill(Scal)</c>) are internal,
/// with the public twins <see cref="WithGap"/>, <see cref="WithTgtAng"/>, <see cref="Constrained"/>,
/// <see cref="WithNorAng"/> and <see cref="Killed"/>. <c>new GeomFill_CornerState()</c> is OCCT's default (an
/// infinite gap, a scale of 1, the Coons conditions met); <c>default</c> is all zeros.
/// </remarks>
internal partial struct CornerState : IEquatable<CornerState>
{
    private double _gap;
    private double _tgtang;
    private bool _isconstrained;
    private double _norang;
    private double _scal;
    private bool _coonscnd;

    public CornerState()
    {
        _gap = RealLast();
        _tgtang = 0.0;
        _isconstrained = false;
        _norang = 0.0;
        _scal = 1.0;
        _coonscnd = true;
    }

    public readonly double Gap() => _gap;

    internal void Gap(double g)
    {
        _gap = g;
    }

    public readonly double TgtAng() => _tgtang;

    internal void TgtAng(double ang)
    {
        _tgtang = ang;
    }

    public readonly bool HasConstraint => _isconstrained;

    internal void Constraint()
    {
        _isconstrained = true;
    }

    public readonly double NorAng() => _norang;

    internal void NorAng(double ang)
    {
        _norang = ang;
    }

    /// <summary>Whether the corner is constrained and the Coons conditions fail; Scal is the kill factor.</summary>
    public readonly bool IsToKill(out double scal)
    {
        scal = _scal;
        if (!_isconstrained)
        {
            return false;
        }
        return !_coonscnd;
    }

    internal void DoKill(double scal)
    {
        _scal = scal;
        _coonscnd = false;
    }

    /// <summary>A copy with the gap <paramref name="g"/> (the twin of <c>Gap(G)</c>).</summary>
    public readonly CornerState WithGap(double g)
    {
        CornerState copy = this;
        copy.Gap(g);
        return copy;
    }

    /// <summary>A copy with the tangents' angle <paramref name="ang"/> (the twin of <c>TgtAng(Ang)</c>).</summary>
    public readonly CornerState WithTgtAng(double ang)
    {
        CornerState copy = this;
        copy.TgtAng(ang);
        return copy;
    }

    /// <summary>A constrained copy (the twin of <c>Constraint()</c>).</summary>
    public readonly CornerState Constrained()
    {
        CornerState copy = this;
        copy.Constraint();
        return copy;
    }

    /// <summary>A copy with the normals' angle <paramref name="ang"/> (the twin of <c>NorAng(Ang)</c>).</summary>
    public readonly CornerState WithNorAng(double ang)
    {
        CornerState copy = this;
        copy.NorAng(ang);
        return copy;
    }

    /// <summary>A killed copy with the factor <paramref name="scal"/> (the twin of <c>DoKill(Scal)</c>).</summary>
    public readonly CornerState Killed(double scal)
    {
        CornerState copy = this;
        copy.DoKill(scal);
        return copy;
    }

    public readonly bool Equals(CornerState other) =>
        _gap.Equals(other._gap) && _tgtang.Equals(other._tgtang) && _isconstrained == other._isconstrained
        && _norang.Equals(other._norang) && _scal.Equals(other._scal) && _coonscnd == other._coonscnd;

    public override readonly bool Equals(object? obj) => obj is CornerState other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_gap, _tgtang, _isconstrained, _norang, _scal, _coonscnd);

    public static bool operator ==(CornerState left, CornerState right) => left.Equals(right);

    public static bool operator !=(CornerState left, CornerState right) => !left.Equals(right);

    public override readonly string ToString() =>
        Constants.Format("GeomFill_CornerState", _gap, _tgtang, _isconstrained ? 1.0 : 0.0, _norang, _scal, _coonscnd ? 1.0 : 0.0);
}
