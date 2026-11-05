// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Surface.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The root of surfaces on which geometric algorithms work: an interface between the services a surface
/// provides and those algorithms need. GeomAdaptor_Surface adapts a Geom surface. The surface can be divided
/// into intervals of any continuity in U and V.
/// </summary>
/// <remarks>
/// Every virtual method throws <see cref="MethodNotImplementedException"/> here, so a derived class need only redefine
/// what its algorithms use. Evaluation goes through the <c>Eval</c> methods; <see cref="D0"/> to
/// <see cref="DN"/> and <see cref="Value"/> call them. B-spline coefficients are cached for evaluation, so
/// evaluating one adaptor from several threads at once is not safe.
/// </remarks>
internal class SurfaceAdaptorBase : Transient
{
    /// <summary>A shallow copy of the adaptor.</summary>
    public virtual SurfaceAdaptorBase ShallowCopy() => throw new MethodNotImplementedException("Adaptor3d_Surface::ShallowCopy");

    public virtual double FirstUParameter() => throw new MethodNotImplementedException("Adaptor3d_Surface::FirstUParameter");

    public virtual double LastUParameter() => throw new MethodNotImplementedException("Adaptor3d_Surface::LastUParameter");

    public virtual double FirstVParameter() => throw new MethodNotImplementedException("Adaptor3d_Surface::FirstVParameter");

    public virtual double LastVParameter() => throw new MethodNotImplementedException("Adaptor3d_Surface::LastVParameter");

    public virtual Continuity UContinuity() => throw new MethodNotImplementedException("Adaptor3d_Surface::UContinuity");

    public virtual Continuity VContinuity() => throw new MethodNotImplementedException("Adaptor3d_Surface::VContinuity");

    /// <summary>The number of U intervals of continuity S; may be one if UContinuity() is at least S.</summary>
    public virtual int NbUIntervals(Continuity s) => throw new MethodNotImplementedException("Adaptor3d_Surface::NbUIntervals");

    /// <summary>The number of V intervals of continuity S; may be one if VContinuity() is at least S.</summary>
    public virtual int NbVIntervals(Continuity s) => throw new MethodNotImplementedException("Adaptor3d_Surface::NbVIntervals");

    /// <summary>The bounds of the U intervals of continuity S.</summary>
    public virtual void UIntervals(NCollection_Array1<double> t, Continuity s) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::UIntervals");

    /// <summary>The bounds of the V intervals of continuity S.</summary>
    public virtual void VIntervals(NCollection_Array1<double> t, Continuity s) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::VIntervals");

    /// <summary>The surface trimmed in U between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual SurfaceAdaptorBase UTrim(double first, double last, double tol) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::UTrim");

    /// <summary>The surface trimmed in V between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual SurfaceAdaptorBase VTrim(double first, double last, double tol) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::VTrim");

    public virtual bool IsUClosed => throw new MethodNotImplementedException("Adaptor3d_Surface::IsUClosed");

    public virtual bool IsVClosed => throw new MethodNotImplementedException("Adaptor3d_Surface::IsVClosed");

    public virtual bool IsUPeriodic => throw new MethodNotImplementedException("Adaptor3d_Surface::IsUPeriodic");

    public virtual double UPeriod() => throw new MethodNotImplementedException("Adaptor3d_Surface::UPeriod");

    public virtual bool IsVPeriodic => throw new MethodNotImplementedException("Adaptor3d_Surface::IsVPeriodic");

    public virtual double VPeriod() => throw new MethodNotImplementedException("Adaptor3d_Surface::VPeriod");

    /// <summary>The point of parameters (U, V). (GeomLib::NormEstim computes the normal.)</summary>
    public Point3D Value(double u, double v) => EvalD0(u, v);

    /// <summary>The point of parameters (U, V).</summary>
    public void D0(double u, double v, out Point3D p) => p = EvalD0(u, v);

    /// <summary>The point and first derivatives. Throws if the current intervals are not C1.</summary>
    public void D1(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        Surface.ResD1 res = EvalD1(u, v);
        p = res.Point;
        d1U = res.D1U;
        d1V = res.D1V;
    }

    /// <summary>The point and the first and second derivatives. Throws if the current intervals are not C2.</summary>
    public void D2(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                   out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV)
    {
        Surface.ResD2 res = EvalD2(u, v);
        p = res.Point;
        d1U = res.D1U;
        d1V = res.D1V;
        d2U = res.D2U;
        d2V = res.D2V;
        d2UV = res.D2UV;
    }

    /// <summary>The point and the derivatives up to the third order. Throws if the current intervals are not C3.</summary>
    public void D3(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                   out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV,
                   out Vector3D d3U, out Vector3D d3V, out Vector3D d3UUV, out Vector3D d3UVV)
    {
        Surface.ResD3 res = EvalD3(u, v);
        p = res.Point;
        d1U = res.D1U;
        d1V = res.D1V;
        d2U = res.D2U;
        d2V = res.D2V;
        d2UV = res.D2UV;
        d3U = res.D3U;
        d3V = res.D3V;
        d3UUV = res.D3UUV;
        d3UVV = res.D3UVV;
    }

    /// <summary>
    /// The derivative of order Nu in U and Nv in V at (U, V). Throws if the current intervals are not CNu and CNv,
    /// or if Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.
    /// </summary>
    public Vector3D DN(double u, double v, int nu, int nv) => EvalDN(u, v, nu, nv);

    /// <summary>The U resolution corresponding to the real space resolution R3d.</summary>
    public virtual double UResolution(double r3d) => throw new MethodNotImplementedException("Adaptor3d_Surface::UResolution");

    /// <summary>The V resolution corresponding to the real space resolution R3d.</summary>
    public virtual double VResolution(double r3d) => throw new MethodNotImplementedException("Adaptor3d_Surface::VResolution");

    /// <summary>The type of the surface.</summary>
    /// <remarks>Hides <see cref="object.GetType"/>; use <see cref="Transient.DynamicType"/> for the .NET type.</remarks>
    public new virtual SurfaceKind GetType() => throw new MethodNotImplementedException("Adaptor3d_Surface::GetType");

    public virtual Plane3D Plane() => throw new MethodNotImplementedException("Adaptor3d_Surface::Plane");

    public virtual Cylinder3D Cylinder() => throw new MethodNotImplementedException("Adaptor3d_Surface::Cylinder");

    public virtual Cone3D Cone() => throw new MethodNotImplementedException("Adaptor3d_Surface::Cone");

    public virtual Sphere3D Sphere() => throw new MethodNotImplementedException("Adaptor3d_Surface::Sphere");

    public virtual Torus3D Torus() => throw new MethodNotImplementedException("Adaptor3d_Surface::Torus");

    public virtual int UDegree() => throw new MethodNotImplementedException("Adaptor3d_Surface::UDegree");

    public virtual int UPoleCount => throw new MethodNotImplementedException("Adaptor3d_Surface::NbUPoles");

    public virtual int VDegree() => throw new MethodNotImplementedException("Adaptor3d_Surface::VDegree");

    public virtual int VPoleCount => throw new MethodNotImplementedException("Adaptor3d_Surface::NbVPoles");

    public virtual int UKnotCount => throw new MethodNotImplementedException("Adaptor3d_Surface::NbUKnots");

    public virtual int VKnotCount => throw new MethodNotImplementedException("Adaptor3d_Surface::NbVKnots");

    public virtual bool IsURational => throw new MethodNotImplementedException("Adaptor3d_Surface::IsURational");

    public virtual bool IsVRational => throw new MethodNotImplementedException("Adaptor3d_Surface::IsVRational");

    public virtual BezierSurface Bezier() => throw new MethodNotImplementedException("Adaptor3d_Surface::Bezier");

    public virtual BSplineSurface BSpline() => throw new MethodNotImplementedException("Adaptor3d_Surface::BSpline");

    public virtual Axis3D AxeOfRevolution() => throw new MethodNotImplementedException("Adaptor3d_Surface::AxeOfRevolution");

    public virtual Direction3D Direction() => throw new MethodNotImplementedException("Adaptor3d_Surface::Direction");

    public virtual CurveAdaptorBase BasisCurve() => throw new MethodNotImplementedException("Adaptor3d_Surface::BasisCurve");

    public virtual SurfaceAdaptorBase BasisSurface() => throw new MethodNotImplementedException("Adaptor3d_Surface::BasisSurface");

    public virtual double OffsetValue() => throw new MethodNotImplementedException("Adaptor3d_Surface::OffsetValue");

    /// <summary>The point of parameters (U, V). Throws on failure.</summary>
    public virtual Point3D EvalD0(double u, double v) => throw new MethodNotImplementedException("Adaptor3d_Surface::EvalD0");

    /// <summary>The point and first partial derivatives at (U, V). Throws on failure.</summary>
    internal virtual Surface.ResD1 EvalD1(double u, double v) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::EvalD1");

    /// <summary>The point and partial derivatives up to the second order at (U, V). Throws on failure.</summary>
    internal virtual Surface.ResD2 EvalD2(double u, double v) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::EvalD2");

    /// <summary>The point and partial derivatives up to the third order at (U, V). Throws on failure.</summary>
    internal virtual Surface.ResD3 EvalD3(double u, double v) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::EvalD3");

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V). Throws on failure.</summary>
    public virtual Vector3D EvalDN(double u, double v, int nu, int nv) =>
        throw new MethodNotImplementedException("Adaptor3d_Surface::EvalDN");
}
