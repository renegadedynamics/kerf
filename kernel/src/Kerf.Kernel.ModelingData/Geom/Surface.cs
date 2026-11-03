// Port of: src/ModelingData/TKG3d/Geom/Geom_Surface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// The common behaviour of parametric surfaces in 3D space: evaluation of the point and partial derivatives at
/// (u, v), continuity, closure and periodicity in each direction, parameter bounds, iso-curves, and how the
/// parameters change when the surface is transformed or reversed. Every surface is at least C0.
/// </summary>
public abstract class Surface : Geometry3D
{
    /// <summary>Result of D1 evaluation: point and first partial derivatives.</summary>
    public struct ResD1
    {
        public Point3D Point;
        public Vector3D D1U;
        public Vector3D D1V;
    }

    /// <summary>Result of D2 evaluation: point and partial derivatives up to the second order.</summary>
    public struct ResD2
    {
        public Point3D Point;
        public Vector3D D1U;
        public Vector3D D1V;
        public Vector3D D2U;
        public Vector3D D2V;
        public Vector3D D2UV;
    }

    /// <summary>Result of D3 evaluation: point and partial derivatives up to the third order.</summary>
    public struct ResD3
    {
        public Point3D Point;
        public Vector3D D1U;
        public Vector3D D1V;
        public Vector3D D2U;
        public Vector3D D2V;
        public Vector3D D2UV;
        public Vector3D D3U;
        public Vector3D D3V;
        public Vector3D D3UUV;
        public Vector3D D3UVV;
    }

    /// <summary>Reverses the U direction of parametrization in place; the bounds don't change.</summary>
    protected internal abstract void UReverse();

    /// <summary>A copy with the U direction reversed.</summary>
    public Surface UReversed()
    {
        Surface s = (Copy() as Surface)!;
        s.UReverse();
        return s;
    }

    /// <summary>The U parameter on the U-reversed surface of the point of parameter U on this one.</summary>
    public abstract double UReversedParameter(double u);

    /// <summary>Reverses the V direction of parametrization in place; the bounds don't change.</summary>
    protected internal abstract void VReverse();

    /// <summary>A copy with the V direction reversed.</summary>
    public Surface VReversed()
    {
        Surface s = (Copy() as Surface)!;
        s.VReverse();
        return s;
    }

    /// <summary>The V parameter on the V-reversed surface of the point of parameter V on this one.</summary>
    public abstract double VReversedParameter(double v);

    /// <summary>
    /// Changes (U, V) into the parameters, on the surface transformed by T, of the transform of the point (U, V).
    /// Leaves them unchanged here; redefined by planes, cylinders, cones, revolved and extruded surfaces.
    /// </summary>
    public virtual void TransformParameters(ref double u, ref double v, in Transform3D t) { }

    /// <summary>
    /// The 2D transformation of (U, V) giving the parameters of the transformed point on the surface transformed
    /// by T: the identity here, redefined by some surfaces.
    /// </summary>
    public virtual GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        GeneralTransform2D dummy = new GeneralTransform2D();
        return dummy;
    }

    /// <summary>The parametric bounds; may be <c>Precision::Infinite()</c> for an infinite surface.</summary>
    public abstract void Bounds(out double u1, out double u2, out double v1, out double v2);

    /// <summary>True if the surface is closed in the U direction.</summary>
    public abstract bool IsUClosed { get; }

    /// <summary>True if the surface is closed in the V direction.</summary>
    public abstract bool IsVClosed { get; }

    /// <summary>True if the surface is periodic in the U direction.</summary>
    public abstract bool IsUPeriodic { get; }

    /// <summary>The period in the U direction.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not U-periodic.</exception>
    public virtual double UPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsUPeriodic, "Geom_Surface::UPeriod");
        double u1, u2, v1, v2;
        Bounds(out u1, out u2, out v1, out v2);
        return (u2 - u1);
    }

    /// <summary>True if the surface is periodic in the V direction.</summary>
    public abstract bool IsVPeriodic { get; }

    /// <summary>The period in the V direction.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not V-periodic.</exception>
    public virtual double VPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsVPeriodic, "Geom_Surface::VPeriod");
        double u1, u2, v1, v2;
        Bounds(out u1, out u2, out v1, out v2);
        return (v2 - v1);
    }

    /// <summary>The U iso-parametric curve.</summary>
    public abstract Curve3D UIso(double u);

    /// <summary>The V iso-parametric curve.</summary>
    public abstract Curve3D VIso(double v);

    /// <summary>The global continuity of the surface (the lower of the U and V continuities).</summary>
    public abstract Continuity Continuity();

    /// <summary>True if the surface is at least CN in the U direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public abstract bool IsCNu(int n);

    /// <summary>True if the surface is at least CN in the V direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public abstract bool IsCNv(int n);

    /// <summary>The point of parameters (U, V). Throws on failure.</summary>
    public abstract Point3D EvalD0(double u, double v);

    /// <summary>The point and first partial derivatives at (U, V). Throws if the surface is not C1.</summary>
    public abstract ResD1 EvalD1(double u, double v);

    /// <summary>The point and partial derivatives up to the second order. Throws if the surface is not C2.</summary>
    public abstract ResD2 EvalD2(double u, double v);

    /// <summary>The point and partial derivatives up to the third order. Throws if the surface is not C3.</summary>
    public abstract ResD3 EvalD3(double u, double v);

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V). Throws on failure.</summary>
    public abstract Vector3D EvalDN(double u, double v, int nu, int nv);

    /// <summary>The point of parameters (U, V).</summary>
    public void D0(double u, double v, out Point3D p) => p = EvalD0(u, v);

    /// <summary>The point and first partial derivatives.</summary>
    public void D1(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        ResD1 r = EvalD1(u, v);
        p = r.Point;
        d1U = r.D1U;
        d1V = r.D1V;
    }

    /// <summary>The point and partial derivatives up to the second order.</summary>
    public void D2(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                   out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV)
    {
        ResD2 r = EvalD2(u, v);
        p = r.Point;
        d1U = r.D1U;
        d1V = r.D1V;
        d2U = r.D2U;
        d2V = r.D2V;
        d2UV = r.D2UV;
    }

    /// <summary>The point and partial derivatives up to the third order.</summary>
    public void D3(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                   out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV,
                   out Vector3D d3U, out Vector3D d3V, out Vector3D d3UUV, out Vector3D d3UVV)
    {
        ResD3 r = EvalD3(u, v);
        p = r.Point;
        d1U = r.D1U;
        d1V = r.D1V;
        d2U = r.D2U;
        d2V = r.D2V;
        d2UV = r.D2UV;
        d3U = r.D3U;
        d3V = r.D3V;
        d3UUV = r.D3UUV;
        d3UVV = r.D3UVV;
    }

    /// <summary>The derivative of order Nu in U and Nv in V.</summary>
    public Vector3D DN(double u, double v, int nu, int nv) => EvalDN(u, v, nu, nv);

    /// <summary>The point of parameters (U, V).</summary>
    public Point3D Value(double u, double v) => EvalD0(u, v);
}
