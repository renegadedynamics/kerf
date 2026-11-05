// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Where a walking line meets an arc of restriction: the arc, the parameter on it and the transitions of the line
/// and of the arc at the point.
/// </summary>
/// <remarks>A C++ value class, so a struct. <c>SetArc</c> is an internal mutator with a public <c>WithArc</c> twin.</remarks>
internal partial struct PointOnRst : IEquatable<PointOnRst>
{
    private Curve2DAdaptorBase? _arc;
    private WalkingTransition _traline;
    private WalkingTransition _traarc;
    private double _prm;

    public PointOnRst()
    {
        _traline = new WalkingTransition();
        _traarc = new WalkingTransition();
        _prm = 0.0;
    }

    /// <summary>A point at parameter <paramref name="param"/> on the arc <paramref name="a"/>.</summary>
    public PointOnRst(Curve2DAdaptorBase a, double param, in WalkingTransition tLine,
                                in WalkingTransition tArc)
    {
        _arc = a;
        _traline = tLine;
        _traarc = tArc;
        _prm = param;
    }

    internal void SetArc(Curve2DAdaptorBase a, double param, in WalkingTransition tLine, in WalkingTransition tArc)
    {
        _arc = a;
        _prm = param;
        _traline = tLine;
        _traarc = tArc;
    }

    public readonly PointOnRst WithArc(Curve2DAdaptorBase a, double param, in WalkingTransition tLine,
                                                 in WalkingTransition tArc)
    {
        PointOnRst copy = this;
        copy.SetArc(a, param, tLine, tArc);
        return copy;
    }

    public readonly Curve2DAdaptorBase Arc() => _arc!;

    public readonly WalkingTransition TransitionOnLine() => _traline;

    public readonly WalkingTransition TransitionOnArc() => _traarc;

    public readonly double ParameterOnArc() => _prm;

    public readonly bool Equals(PointOnRst other)
        => ReferenceEquals(_arc, other._arc) && _traline.Equals(other._traline) && _traarc.Equals(other._traarc)
           && _prm.Equals(other._prm);

    public override readonly bool Equals(object? obj) => obj is PointOnRst other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_arc, _traline, _traarc, _prm);

    public static bool operator ==(in PointOnRst left, in PointOnRst right)
        => left.Equals(right);

    public static bool operator !=(in PointOnRst left, in PointOnRst right)
        => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("BRepBlend_PointOnRst", _prm);
}
