// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Curve2d.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The root of 2D curves on which geometric algorithms work: an interface between the services a curve
/// provides and those algorithms need. Geom2dAdaptor_Curve adapts a Geom2d curve.
/// </summary>
/// <remarks>
/// Every method is virtual and, here, throws <see cref="MethodNotImplementedException"/> (except
/// <see cref="NbSamples"/>, and the <c>Eval</c> methods, which call D0 to DN), so a derived class need only
/// redefine what its algorithms use. B-spline coefficients are cached for evaluation, so evaluating one adaptor
/// from several threads at once is not safe.
/// </remarks>
public class Adaptor2d_Curve2d : Transient
{
    /// <summary>A shallow copy of the adaptor.</summary>
    public virtual Adaptor2d_Curve2d ShallowCopy() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::ShallowCopy");

    public virtual double FirstParameter() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::FirstParameter");

    public virtual double LastParameter() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::LastParameter");

    public virtual Continuity Continuity() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Continuity");

    /// <summary>The number of intervals of continuity S (breaking the curve if necessary).</summary>
    public virtual int NbIntervals(Continuity S) => throw new MethodNotImplementedException("Adaptor2d_Curve2d::NbIntervals");

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public virtual void Intervals(NCollection_Array1<double> T, Continuity S) =>
        throw new MethodNotImplementedException("Adaptor2d_Curve2d::Intervals");

    /// <summary>The curve between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual Adaptor2d_Curve2d Trim(double First, double Last, double Tol) =>
        throw new MethodNotImplementedException("Adaptor2d_Curve2d::Trim");

    public virtual bool IsClosed() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::IsClosed");

    public virtual bool IsPeriodic() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::IsPeriodic");

    public virtual double Period() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Period");

    /// <summary>The point of parameter U.</summary>
    public virtual Point2D Value(double U) => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Value");

    /// <summary>The point of parameter U.</summary>
    public virtual void D0(double U, out Point2D P) => throw new MethodNotImplementedException("Adaptor2d_Curve2d::D0");

    /// <summary>The point of parameter U and the first derivative. Throws if the current interval is not C1.</summary>
    public virtual void D1(double U, out Point2D P, out Vector2D V) =>
        throw new MethodNotImplementedException("Adaptor2d_Curve2d::D1");

    /// <summary>The point of parameter U and the first two derivatives. Throws if the current interval is not C2.</summary>
    public virtual void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2) =>
        throw new MethodNotImplementedException("Adaptor2d_Curve2d::D2");

    /// <summary>The point of parameter U and the first three derivatives. Throws if the current interval is not C3.</summary>
    public virtual void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3) =>
        throw new MethodNotImplementedException("Adaptor2d_Curve2d::D3");

    /// <summary>The derivative of order N at U. Throws if the current interval is not CN, or N &lt; 1.</summary>
    public virtual Vector2D DN(double U, int N) => throw new MethodNotImplementedException("Adaptor2d_Curve2d::DN");

    /// <summary>The parametric resolution corresponding to the real space resolution R3d.</summary>
    public virtual double Resolution(double R3d) => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Resolution");

    /// <summary>The type of the curve in the current interval.</summary>
    /// <remarks>Hides <see cref="object.GetType"/>; use <see cref="Transient.DynamicType"/> for the .NET type.</remarks>
    public new virtual CurveKind GetType() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::GetType");

    public virtual Line2D Line() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Line");

    public virtual Circle2D Circle() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Circle");

    public virtual Ellipse2D Ellipse() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Ellipse");

    public virtual Hyperbola2D Hyperbola() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Hyperbola");

    public virtual Parabola2D Parabola() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Parabola");

    public virtual int Degree() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Degree");

    public virtual bool IsRational() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::IsRational");

    // (sic) OCCT's message says "NbPole".
    public virtual int NbPoles() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::NbPole");

    public virtual int NbKnots() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::NbKnots");

    /// <summary>The number of samples algorithms should take on the curve: 20 here.</summary>
    public virtual int NbSamples() => 20;

    public virtual Geom2d_BezierCurve Bezier() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::Bezier");

    public virtual Geom2d_BSplineCurve BSpline() => throw new MethodNotImplementedException("Adaptor2d_Curve2d::BSpline");

    /// <summary>The point of parameter U (calls <see cref="D0"/> unless redefined).</summary>
    public virtual Point2D EvalD0(double theU)
    {
        Point2D aP;
        D0(theU, out aP);
        return aP;
    }

    /// <summary>The point and first derivative at U (calls <see cref="D1"/> unless redefined).</summary>
    public virtual Geom2d_Curve.ResD1 EvalD1(double theU)
    {
        Geom2d_Curve.ResD1 aResult;
        D1(theU, out aResult.Point, out aResult.D1);
        return aResult;
    }

    /// <summary>The point and first two derivatives at U (calls <see cref="D2"/> unless redefined).</summary>
    public virtual Geom2d_Curve.ResD2 EvalD2(double theU)
    {
        Geom2d_Curve.ResD2 aResult;
        D2(theU, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    /// <summary>The point and first three derivatives at U (calls <see cref="D3"/> unless redefined).</summary>
    public virtual Geom2d_Curve.ResD3 EvalD3(double theU)
    {
        Geom2d_Curve.ResD3 aResult;
        D3(theU, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <summary>The derivative of order N at U (calls <see cref="DN"/> unless redefined).</summary>
    public virtual Vector2D EvalDN(double theU, int theN) => DN(theU, theN);
}
