// Port of: src/ModelingData/TKG3d/Geom/Geom_Surface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The common behaviour of parametric surfaces in 3D space: evaluation of the point and partial derivatives at
/// (u, v), continuity, closure and periodicity in each direction, parameter bounds, iso-curves, and how the
/// parameters change when the surface is transformed or reversed. Every surface is at least C0.
/// </summary>
public abstract class Geom_Surface : Geom_Geometry
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
    public Geom_Surface UReversed()
    {
        Geom_Surface S = (Copy() as Geom_Surface)!;
        S.UReverse();
        return S;
    }

    /// <summary>The U parameter on the U-reversed surface of the point of parameter U on this one.</summary>
    public abstract double UReversedParameter(double U);

    /// <summary>Reverses the V direction of parametrization in place; the bounds don't change.</summary>
    protected internal abstract void VReverse();

    /// <summary>A copy with the V direction reversed.</summary>
    public Geom_Surface VReversed()
    {
        Geom_Surface S = (Copy() as Geom_Surface)!;
        S.VReverse();
        return S;
    }

    /// <summary>The V parameter on the V-reversed surface of the point of parameter V on this one.</summary>
    public abstract double VReversedParameter(double V);

    /// <summary>
    /// Changes (U, V) into the parameters, on the surface transformed by T, of the transform of the point (U, V).
    /// Leaves them unchanged here; redefined by planes, cylinders, cones, revolved and extruded surfaces.
    /// </summary>
    public virtual void TransformParameters(ref double U, ref double V, in Transform3D T) { }

    /// <summary>
    /// The 2D transformation of (U, V) giving the parameters of the transformed point on the surface transformed
    /// by T: the identity here, redefined by some surfaces.
    /// </summary>
    public virtual GeneralTransform2D ParametricTransformation(in Transform3D T)
    {
        GeneralTransform2D dummy = new GeneralTransform2D();
        return dummy;
    }

    /// <summary>The parametric bounds; may be <c>Precision::Infinite()</c> for an infinite surface.</summary>
    public abstract void Bounds(out double U1, out double U2, out double V1, out double V2);

    /// <summary>True if the surface is closed in the U direction.</summary>
    public abstract bool IsUClosed();

    /// <summary>True if the surface is closed in the V direction.</summary>
    public abstract bool IsVClosed();

    /// <summary>True if the surface is periodic in the U direction.</summary>
    public abstract bool IsUPeriodic();

    /// <summary>The period in the U direction.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not U-periodic.</exception>
    public virtual double UPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsUPeriodic(), "Geom_Surface::UPeriod");
        double U1, U2, V1, V2;
        Bounds(out U1, out U2, out V1, out V2);
        return (U2 - U1);
    }

    /// <summary>True if the surface is periodic in the V direction.</summary>
    public abstract bool IsVPeriodic();

    /// <summary>The period in the V direction.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not V-periodic.</exception>
    public virtual double VPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsVPeriodic(), "Geom_Surface::VPeriod");
        double U1, U2, V1, V2;
        Bounds(out U1, out U2, out V1, out V2);
        return (V2 - V1);
    }

    /// <summary>The U iso-parametric curve.</summary>
    public abstract Geom_Curve UIso(double U);

    /// <summary>The V iso-parametric curve.</summary>
    public abstract Geom_Curve VIso(double V);

    /// <summary>The global continuity of the surface (the lower of the U and V continuities).</summary>
    public abstract Continuity Continuity();

    /// <summary>True if the surface is at least CN in the U direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public abstract bool IsCNu(int N);

    /// <summary>True if the surface is at least CN in the V direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public abstract bool IsCNv(int N);

    /// <summary>The point of parameters (U, V). Throws on failure.</summary>
    public abstract Point3D EvalD0(double U, double V);

    /// <summary>The point and first partial derivatives at (U, V). Throws if the surface is not C1.</summary>
    public abstract ResD1 EvalD1(double U, double V);

    /// <summary>The point and partial derivatives up to the second order. Throws if the surface is not C2.</summary>
    public abstract ResD2 EvalD2(double U, double V);

    /// <summary>The point and partial derivatives up to the third order. Throws if the surface is not C3.</summary>
    public abstract ResD3 EvalD3(double U, double V);

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V). Throws on failure.</summary>
    public abstract Vector3D EvalDN(double U, double V, int Nu, int Nv);

    /// <summary>The point of parameters (U, V).</summary>
    public void D0(double U, double V, out Point3D P) => P = EvalD0(U, V);

    /// <summary>The point and first partial derivatives.</summary>
    public void D1(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        ResD1 aR = EvalD1(U, V);
        P = aR.Point;
        D1U = aR.D1U;
        D1V = aR.D1V;
    }

    /// <summary>The point and partial derivatives up to the second order.</summary>
    public void D2(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                   out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV)
    {
        ResD2 aR = EvalD2(U, V);
        P = aR.Point;
        D1U = aR.D1U;
        D1V = aR.D1V;
        D2U = aR.D2U;
        D2V = aR.D2V;
        D2UV = aR.D2UV;
    }

    /// <summary>The point and partial derivatives up to the third order.</summary>
    public void D3(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                   out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV,
                   out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV)
    {
        ResD3 aR = EvalD3(U, V);
        P = aR.Point;
        D1U = aR.D1U;
        D1V = aR.D1V;
        D2U = aR.D2U;
        D2V = aR.D2V;
        D2UV = aR.D2UV;
        D3U = aR.D3U;
        D3V = aR.D3V;
        D3UUV = aR.D3UUV;
        D3UVV = aR.D3UVV;
    }

    /// <summary>The derivative of order Nu in U and Nv in V.</summary>
    public Vector3D DN(double U, double V, int Nu, int Nv) => EvalDN(U, V, Nu, Nv);

    /// <summary>The point of parameters (U, V).</summary>
    public Point3D Value(double U, double V) => EvalD0(U, V);
}
