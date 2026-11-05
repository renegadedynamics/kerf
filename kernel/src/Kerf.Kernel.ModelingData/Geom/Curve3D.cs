// Port of: src/ModelingData/TKG3d/Geom/Geom_Curve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// The common behaviour of parametric curves in 3D space: evaluation of the point and derivatives at a
/// parameter, continuity, closure, periodicity, parameter bounds, and how the parameter changes when the curve
/// is transformed or reversed. Every curve is at least C0.
/// </summary>
public abstract partial class Curve3D : Geometry3D
{
    /// <summary>Only the kernel derives from it (unit 10L1).</summary>
    private protected Curve3D()
    {
    }

    // Results with public fields, written through out parameters across the engine: 10L replaces them (geometry.md,
    // CurveDerivatives), so KERF0003 is off for them until then.
#pragma warning disable KERF0003
    /// <summary>Result of D1 evaluation: point and first derivative.</summary>
    internal struct ResD1
    {
        public Point3D Point;
        public Vector3D D1;
    }

    /// <summary>Result of D2 evaluation: point and first two derivatives.</summary>
    internal struct ResD2
    {
        public Point3D Point;
        public Vector3D D1;
        public Vector3D D2;
    }

    /// <summary>Result of D3 evaluation: point and first three derivatives.</summary>
    internal struct ResD3
    {
        public Point3D Point;
        public Vector3D D1;
        public Vector3D D2;
        public Vector3D D3;
    }
#pragma warning restore KERF0003

    /// <summary>
    /// Reverses the direction of parametrization in place. The first and last parameters don't change; the start
    /// and end points of a bounded curve swap.
    /// </summary>
    protected internal abstract void Reverse();

    /// <summary>The parameter on the reversed curve of the point of parameter U on this one.</summary>
    public abstract double ReversedParameter(double u);

    /// <summary>
    /// The parameter on the curve transformed by T of the transform of the point of parameter U: U itself here,
    /// redefined by some curves (lines, for instance).
    /// </summary>
    internal virtual double TransformedParameter(double u, in Transform3D t) => u;

    /// <summary>
    /// The coefficient to apply to a parameter of this curve to get the parameter of the transformed point on the
    /// curve transformed by T: 1 here, redefined by some curves.
    /// </summary>
    internal virtual double ParametricTransformation(in Transform3D t) => 1.0;

    /// <summary>A reversed copy of this curve.</summary>
    public Curve3D Reversed()
    {
        Curve3D c = (Copy() as Curve3D)!;
        c.Reverse();
        return c;
    }

    /// <summary>The first parameter (may be <c>RealFirst()</c> for an infinite curve).</summary>
    public abstract double FirstParameter { get; }

    /// <summary>The last parameter (may be <c>RealLast()</c> for an infinite curve).</summary>
    public abstract double LastParameter { get; }

    /// <summary>True if the curve is closed (its end points are within <c>gp::Resolution()</c>).</summary>
    public abstract bool IsClosed { get; }

    /// <summary>True if the parametrization is periodic.</summary>
    public abstract bool IsPeriodic { get; }

    /// <summary>The period of the curve.</summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    public virtual double Period
    {
        get
        {
            Refusal.StateIf(!IsPeriodic, FailureKind.NoSuchObject, "Geom_Curve::Period");
            return (LastParameter - FirstParameter);
        }
    }

    /// <summary>The global continuity of the curve.</summary>
    public abstract Continuity Continuity { get; }

    /// <summary>True if the curve is at least CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal abstract bool IsCN(int n);

    /// <summary>The point of parameter U. Throws on failure (an offset curve at a singular point, for instance).</summary>
    internal abstract Point3D EvalD0(double u);

    /// <summary>The point and first derivative at U. Throws if the curve is not C1.</summary>
    internal abstract ResD1 EvalD1(double u);

    /// <summary>The point and first two derivatives at U. Throws if the curve is not C2.</summary>
    internal abstract ResD2 EvalD2(double u);

    /// <summary>The point and first three derivatives at U. Throws if the curve is not C3.</summary>
    internal abstract ResD3 EvalD3(double u);

    /// <summary>The Nth derivative at U. Throws if the curve is not CN, or N &lt; 1.</summary>
    internal abstract Vector3D EvalDN(double u, int n);

    /// <summary>The point of parameter U.</summary>
    internal void D0(double u, out Point3D p) => p = EvalD0(u);

    /// <summary>The point of parameter U and the first derivative.</summary>
    internal void D1(double u, out Point3D p, out Vector3D v1)
    {
        ResD1 r = EvalD1(u);
        p = r.Point;
        v1 = r.D1;
    }

    /// <summary>The point of parameter U and the first two derivatives.</summary>
    internal void D2(double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        ResD2 r = EvalD2(u);
        p = r.Point;
        v1 = r.D1;
        v2 = r.D2;
    }

    /// <summary>The point of parameter U and the first three derivatives.</summary>
    internal void D3(double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        ResD3 r = EvalD3(u);
        p = r.Point;
        v1 = r.D1;
        v2 = r.D2;
        v3 = r.D3;
    }

    /// <summary>The derivative of order N at U.</summary>
    internal Vector3D DN(double u, int n) => EvalDN(u, n);

    /// <summary>The point of parameter U.</summary>
    internal Point3D Value(double u) => EvalD0(u);
}
