// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CornerState.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CornerState.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

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
public partial struct GeomFill_CornerState : IEquatable<GeomFill_CornerState>
{
    private double gap;
    private double tgtang;
    private bool isconstrained;
    private double norang;
    private double scal;
    private bool coonscnd;

    public GeomFill_CornerState()
    {
        gap = RealLast();
        tgtang = 0.0;
        isconstrained = false;
        norang = 0.0;
        scal = 1.0;
        coonscnd = true;
    }

    public readonly double Gap() => gap;

    internal void Gap(double G)
    {
        gap = G;
    }

    public readonly double TgtAng() => tgtang;

    internal void TgtAng(double Ang)
    {
        tgtang = Ang;
    }

    public readonly bool HasConstraint() => isconstrained;

    internal void Constraint()
    {
        isconstrained = true;
    }

    public readonly double NorAng() => norang;

    internal void NorAng(double Ang)
    {
        norang = Ang;
    }

    /// <summary>Whether the corner is constrained and the Coons conditions fail; Scal is the kill factor.</summary>
    public readonly bool IsToKill(out double Scal)
    {
        Scal = scal;
        if (!isconstrained)
        {
            return false;
        }
        return !coonscnd;
    }

    internal void DoKill(double Scal)
    {
        scal = Scal;
        coonscnd = false;
    }

    /// <summary>A copy with the gap <paramref name="G"/> (the twin of <c>Gap(G)</c>).</summary>
    public readonly GeomFill_CornerState WithGap(double G)
    {
        GeomFill_CornerState aCopy = this;
        aCopy.Gap(G);
        return aCopy;
    }

    /// <summary>A copy with the tangents' angle <paramref name="Ang"/> (the twin of <c>TgtAng(Ang)</c>).</summary>
    public readonly GeomFill_CornerState WithTgtAng(double Ang)
    {
        GeomFill_CornerState aCopy = this;
        aCopy.TgtAng(Ang);
        return aCopy;
    }

    /// <summary>A constrained copy (the twin of <c>Constraint()</c>).</summary>
    public readonly GeomFill_CornerState Constrained()
    {
        GeomFill_CornerState aCopy = this;
        aCopy.Constraint();
        return aCopy;
    }

    /// <summary>A copy with the normals' angle <paramref name="Ang"/> (the twin of <c>NorAng(Ang)</c>).</summary>
    public readonly GeomFill_CornerState WithNorAng(double Ang)
    {
        GeomFill_CornerState aCopy = this;
        aCopy.NorAng(Ang);
        return aCopy;
    }

    /// <summary>A killed copy with the factor <paramref name="Scal"/> (the twin of <c>DoKill(Scal)</c>).</summary>
    public readonly GeomFill_CornerState Killed(double Scal)
    {
        GeomFill_CornerState aCopy = this;
        aCopy.DoKill(Scal);
        return aCopy;
    }

    public readonly bool Equals(GeomFill_CornerState other) =>
        gap.Equals(other.gap) && tgtang.Equals(other.tgtang) && isconstrained == other.isconstrained
        && norang.Equals(other.norang) && scal.Equals(other.scal) && coonscnd == other.coonscnd;

    public override readonly bool Equals(object? obj) => obj is GeomFill_CornerState other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(gap, tgtang, isconstrained, norang, scal, coonscnd);

    public static bool operator ==(GeomFill_CornerState left, GeomFill_CornerState right) => left.Equals(right);

    public static bool operator !=(GeomFill_CornerState left, GeomFill_CornerState right) => !left.Equals(right);

    public override readonly string ToString() =>
        Constants.Format("GeomFill_CornerState", gap, tgtang, isconstrained ? 1.0 : 0.0, norang, scal, coonscnd ? 1.0 : 0.0);
}
