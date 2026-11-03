// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_PointOnRst.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>
/// Where a walking line meets an arc of restriction: the arc, the parameter on it and the transitions of the line
/// and of the arc at the point.
/// </summary>
/// <remarks>A C++ value class, so a struct. <c>SetArc</c> is an internal mutator with a public <c>WithArc</c> twin.</remarks>
public partial struct BRepBlend_PointOnRst : IEquatable<BRepBlend_PointOnRst>
{
    private Curve2DAdaptorBase? arc;
    private IntSurf_Transition traline;
    private IntSurf_Transition traarc;
    private double prm;

    public BRepBlend_PointOnRst()
    {
        traline = new IntSurf_Transition();
        traarc = new IntSurf_Transition();
        prm = 0.0;
    }

    /// <summary>A point at parameter <paramref name="Param"/> on the arc <paramref name="A"/>.</summary>
    public BRepBlend_PointOnRst(Curve2DAdaptorBase A, double Param, in IntSurf_Transition TLine,
                                in IntSurf_Transition TArc)
    {
        arc = A;
        traline = TLine;
        traarc = TArc;
        prm = Param;
    }

    internal void SetArc(Curve2DAdaptorBase A, double Param, in IntSurf_Transition TLine, in IntSurf_Transition TArc)
    {
        arc = A;
        prm = Param;
        traline = TLine;
        traarc = TArc;
    }

    public readonly BRepBlend_PointOnRst WithArc(Curve2DAdaptorBase A, double Param, in IntSurf_Transition TLine,
                                                 in IntSurf_Transition TArc)
    {
        BRepBlend_PointOnRst aCopy = this;
        aCopy.SetArc(A, Param, TLine, TArc);
        return aCopy;
    }

    public readonly Curve2DAdaptorBase Arc() => arc!;

    public readonly IntSurf_Transition TransitionOnLine() => traline;

    public readonly IntSurf_Transition TransitionOnArc() => traarc;

    public readonly double ParameterOnArc() => prm;

    public readonly bool Equals(BRepBlend_PointOnRst other)
        => ReferenceEquals(arc, other.arc) && traline.Equals(other.traline) && traarc.Equals(other.traarc)
           && prm.Equals(other.prm);

    public override readonly bool Equals(object? obj) => obj is BRepBlend_PointOnRst other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(arc, traline, traarc, prm);

    public static bool operator ==(in BRepBlend_PointOnRst theLeft, in BRepBlend_PointOnRst theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in BRepBlend_PointOnRst theLeft, in BRepBlend_PointOnRst theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("BRepBlend_PointOnRst", prm);
}
