// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Curve.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of 3D curves on which geometric algorithms work: an interface between the services a curve provides
/// and those algorithms need. GeomAdaptor_Curve adapts a Geom curve, Adaptor3d_CurveOnSurface a curve lying on a
/// surface.
/// </summary>
/// <remarks>
/// Every virtual method throws <see cref="Standard_NotImplemented"/> here, so a derived class need only redefine
/// what its algorithms use. Evaluation goes through the <c>Eval</c> methods; <see cref="D0"/> to
/// <see cref="DN"/> and <see cref="Value"/> call them. B-spline coefficients are cached for evaluation, so
/// evaluating one adaptor from several threads at once is not safe.
/// </remarks>
public class Adaptor3d_Curve : Standard_Transient
{
    /// <summary>A shallow copy of the adaptor.</summary>
    public virtual Adaptor3d_Curve ShallowCopy() => throw new Standard_NotImplemented("Adaptor3d_Curve::ShallowCopy");

    public virtual double FirstParameter() => throw new Standard_NotImplemented("Adaptor3d_Curve::FirstParameter");

    public virtual double LastParameter() => throw new Standard_NotImplemented("Adaptor3d_Curve::LastParameter");

    public virtual GeomAbs_Shape Continuity() => throw new Standard_NotImplemented("Adaptor3d_Curve::Continuity");

    /// <summary>The number of intervals of continuity S; may be one if Continuity() is at least S.</summary>
    public virtual int NbIntervals(GeomAbs_Shape S) => throw new Standard_NotImplemented("Adaptor3d_Curve::NbIntervals");

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public virtual void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S) =>
        throw new Standard_NotImplemented("Adaptor3d_Curve::Intervals");

    /// <summary>The curve between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual Adaptor3d_Curve Trim(double First, double Last, double Tol) =>
        throw new Standard_NotImplemented("Adaptor3d_Curve::Trim");

    public virtual bool IsClosed() => throw new Standard_NotImplemented("Adaptor3d_Curve::IsClosed");

    public virtual bool IsPeriodic() => throw new Standard_NotImplemented("Adaptor3d_Curve::IsPeriodic");

    public virtual double Period() => throw new Standard_NotImplemented("Adaptor3d_Curve::Period");

    /// <summary>The point of parameter U.</summary>
    public gp_Pnt Value(double theU) => EvalD0(theU);

    /// <summary>The point of parameter U.</summary>
    public void D0(double theU, out gp_Pnt theP) => theP = EvalD0(theU);

    /// <summary>The point of parameter U and the first derivative. Throws if the current interval is not C1.</summary>
    public void D1(double theU, out gp_Pnt theP, out gp_Vec theV)
    {
        Geom_Curve.ResD1 aRes = EvalD1(theU);
        theP = aRes.Point;
        theV = aRes.D1;
    }

    /// <summary>The point of parameter U and the first two derivatives. Throws if the current interval is not C2.</summary>
    public void D2(double theU, out gp_Pnt theP, out gp_Vec theV1, out gp_Vec theV2)
    {
        Geom_Curve.ResD2 aRes = EvalD2(theU);
        theP = aRes.Point;
        theV1 = aRes.D1;
        theV2 = aRes.D2;
    }

    /// <summary>The point of parameter U and the first three derivatives. Throws if the current interval is not C3.</summary>
    public void D3(double theU, out gp_Pnt theP, out gp_Vec theV1, out gp_Vec theV2, out gp_Vec theV3)
    {
        Geom_Curve.ResD3 aRes = EvalD3(theU);
        theP = aRes.Point;
        theV1 = aRes.D1;
        theV2 = aRes.D2;
        theV3 = aRes.D3;
    }

    /// <summary>The derivative of order N at U. Throws if the current interval is not CN, or N &lt; 1.</summary>
    public gp_Vec DN(double theU, int theN) => EvalDN(theU, theN);

    /// <summary>The parametric resolution corresponding to the real space resolution R3d.</summary>
    public virtual double Resolution(double R3d) => throw new Standard_NotImplemented("Adaptor3d_Curve::Resolution");

    /// <summary>The type of the curve in the current interval.</summary>
    /// <remarks>Hides <see cref="object.GetType"/>; use <see cref="Standard_Transient.DynamicType"/> for the .NET type.</remarks>
    public new virtual GeomAbs_CurveType GetType() => throw new Standard_NotImplemented("Adaptor3d_Curve::GetType");

    public virtual gp_Lin Line() => throw new Standard_NotImplemented("Adaptor3d_Curve::Line");

    public virtual gp_Circ Circle() => throw new Standard_NotImplemented("Adaptor3d_Curve::Circle");

    public virtual gp_Elips Ellipse() => throw new Standard_NotImplemented("Adaptor3d_Curve::Ellipse");

    public virtual gp_Hypr Hyperbola() => throw new Standard_NotImplemented("Adaptor3d_Curve::Hyperbola");

    public virtual gp_Parab Parabola() => throw new Standard_NotImplemented("Adaptor3d_Curve::Parabola");

    public virtual int Degree() => throw new Standard_NotImplemented("Adaptor3d_Curve::Degree");

    public virtual bool IsRational() => throw new Standard_NotImplemented("Adaptor3d_Curve::IsRational");

    public virtual int NbPoles() => throw new Standard_NotImplemented("Adaptor3d_Curve::NbPoles");

    public virtual int NbKnots() => throw new Standard_NotImplemented("Adaptor3d_Curve::NbKnots");

    public virtual Geom_BezierCurve Bezier() => throw new Standard_NotImplemented("Adaptor3d_Curve::Bezier");

    public virtual Geom_BSplineCurve BSpline() => throw new Standard_NotImplemented("Adaptor3d_Curve::BSpline");

    public virtual Geom_OffsetCurve OffsetCurve() => throw new Standard_NotImplemented("Adaptor3d_Curve::OffsetCurve");

    /// <summary>The point of parameter U. Throws on failure.</summary>
    public virtual gp_Pnt EvalD0(double theU) => throw new Standard_NotImplemented("Adaptor3d_Curve::EvalD0");

    /// <summary>The point and first derivative at U. Throws on failure.</summary>
    public virtual Geom_Curve.ResD1 EvalD1(double theU) => throw new Standard_NotImplemented("Adaptor3d_Curve::EvalD1");

    /// <summary>The point and first two derivatives at U. Throws on failure.</summary>
    public virtual Geom_Curve.ResD2 EvalD2(double theU) => throw new Standard_NotImplemented("Adaptor3d_Curve::EvalD2");

    /// <summary>The point and first three derivatives at U. Throws on failure.</summary>
    public virtual Geom_Curve.ResD3 EvalD3(double theU) => throw new Standard_NotImplemented("Adaptor3d_Curve::EvalD3");

    /// <summary>The derivative of order N at U. Throws on failure.</summary>
    public virtual gp_Vec EvalDN(double theU, int theN) => throw new Standard_NotImplemented("Adaptor3d_Curve::EvalDN");
}
