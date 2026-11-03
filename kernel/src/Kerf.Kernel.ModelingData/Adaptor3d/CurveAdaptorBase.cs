// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Curve.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The root of 3D curves on which geometric algorithms work: an interface between the services a curve provides
/// and those algorithms need. GeomAdaptor_Curve adapts a Geom curve, Adaptor3d_CurveOnSurface a curve lying on a
/// surface.
/// </summary>
/// <remarks>
/// Every virtual method throws <see cref="MethodNotImplementedException"/> here, so a derived class need only redefine
/// what its algorithms use. Evaluation goes through the <c>Eval</c> methods; <see cref="D0"/> to
/// <see cref="DN"/> and <see cref="Value"/> call them. B-spline coefficients are cached for evaluation, so
/// evaluating one adaptor from several threads at once is not safe.
/// </remarks>
public class CurveAdaptorBase : Transient
{
    /// <summary>A shallow copy of the adaptor.</summary>
    public virtual CurveAdaptorBase ShallowCopy() => throw new MethodNotImplementedException("Adaptor3d_Curve::ShallowCopy");

    public virtual double FirstParameter() => throw new MethodNotImplementedException("Adaptor3d_Curve::FirstParameter");

    public virtual double LastParameter() => throw new MethodNotImplementedException("Adaptor3d_Curve::LastParameter");

    public virtual Continuity Continuity() => throw new MethodNotImplementedException("Adaptor3d_Curve::Continuity");

    /// <summary>The number of intervals of continuity S; may be one if Continuity() is at least S.</summary>
    public virtual int NbIntervals(Continuity s) => throw new MethodNotImplementedException("Adaptor3d_Curve::NbIntervals");

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public virtual void Intervals(NCollection_Array1<double> t, Continuity s) =>
        throw new MethodNotImplementedException("Adaptor3d_Curve::Intervals");

    /// <summary>The curve between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual CurveAdaptorBase Trim(double first, double last, double tol) =>
        throw new MethodNotImplementedException("Adaptor3d_Curve::Trim");

    public virtual bool IsClosed => throw new MethodNotImplementedException("Adaptor3d_Curve::IsClosed");

    public virtual bool IsPeriodic => throw new MethodNotImplementedException("Adaptor3d_Curve::IsPeriodic");

    public virtual double Period() => throw new MethodNotImplementedException("Adaptor3d_Curve::Period");

    /// <summary>The point of parameter U.</summary>
    public Point3D Value(double u) => EvalD0(u);

    /// <summary>The point of parameter U.</summary>
    public void D0(double u, out Point3D p) => p = EvalD0(u);

    /// <summary>The point of parameter U and the first derivative. Throws if the current interval is not C1.</summary>
    public void D1(double u, out Point3D p, out Vector3D v)
    {
        Curve3D.ResD1 res = EvalD1(u);
        p = res.Point;
        v = res.D1;
    }

    /// <summary>The point of parameter U and the first two derivatives. Throws if the current interval is not C2.</summary>
    public void D2(double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        Curve3D.ResD2 res = EvalD2(u);
        p = res.Point;
        v1 = res.D1;
        v2 = res.D2;
    }

    /// <summary>The point of parameter U and the first three derivatives. Throws if the current interval is not C3.</summary>
    public void D3(double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        Curve3D.ResD3 res = EvalD3(u);
        p = res.Point;
        v1 = res.D1;
        v2 = res.D2;
        v3 = res.D3;
    }

    /// <summary>The derivative of order N at U. Throws if the current interval is not CN, or N &lt; 1.</summary>
    public Vector3D DN(double u, int n) => EvalDN(u, n);

    /// <summary>The parametric resolution corresponding to the real space resolution R3d.</summary>
    public virtual double Resolution(double r3d) => throw new MethodNotImplementedException("Adaptor3d_Curve::Resolution");

    /// <summary>The type of the curve in the current interval.</summary>
    /// <remarks>Hides <see cref="object.GetType"/>; use <see cref="Transient.DynamicType"/> for the .NET type.</remarks>
    public new virtual CurveKind GetType() => throw new MethodNotImplementedException("Adaptor3d_Curve::GetType");

    public virtual Line3D Line() => throw new MethodNotImplementedException("Adaptor3d_Curve::Line");

    public virtual Circle3D Circle() => throw new MethodNotImplementedException("Adaptor3d_Curve::Circle");

    public virtual Ellipse3D Ellipse() => throw new MethodNotImplementedException("Adaptor3d_Curve::Ellipse");

    public virtual Hyperbola3D Hyperbola() => throw new MethodNotImplementedException("Adaptor3d_Curve::Hyperbola");

    public virtual Parabola3D Parabola() => throw new MethodNotImplementedException("Adaptor3d_Curve::Parabola");

    public virtual int Degree() => throw new MethodNotImplementedException("Adaptor3d_Curve::Degree");

    public virtual bool IsRational => throw new MethodNotImplementedException("Adaptor3d_Curve::IsRational");

    public virtual int PoleCount => throw new MethodNotImplementedException("Adaptor3d_Curve::NbPoles");

    public virtual int KnotCount => throw new MethodNotImplementedException("Adaptor3d_Curve::NbKnots");

    public virtual BezierCurve3D Bezier() => throw new MethodNotImplementedException("Adaptor3d_Curve::Bezier");

    public virtual BSplineCurve3D BSpline() => throw new MethodNotImplementedException("Adaptor3d_Curve::BSpline");

    public virtual OffsetCurve3D OffsetCurve() => throw new MethodNotImplementedException("Adaptor3d_Curve::OffsetCurve");

    /// <summary>The point of parameter U. Throws on failure.</summary>
    public virtual Point3D EvalD0(double u) => throw new MethodNotImplementedException("Adaptor3d_Curve::EvalD0");

    /// <summary>The point and first derivative at U. Throws on failure.</summary>
    internal virtual Curve3D.ResD1 EvalD1(double u) => throw new MethodNotImplementedException("Adaptor3d_Curve::EvalD1");

    /// <summary>The point and first two derivatives at U. Throws on failure.</summary>
    internal virtual Curve3D.ResD2 EvalD2(double u) => throw new MethodNotImplementedException("Adaptor3d_Curve::EvalD2");

    /// <summary>The point and first three derivatives at U. Throws on failure.</summary>
    internal virtual Curve3D.ResD3 EvalD3(double u) => throw new MethodNotImplementedException("Adaptor3d_Curve::EvalD3");

    /// <summary>The derivative of order N at U. Throws on failure.</summary>
    public virtual Vector3D EvalDN(double u, int n) => throw new MethodNotImplementedException("Adaptor3d_Curve::EvalDN");
}
