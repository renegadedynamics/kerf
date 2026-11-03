// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Curve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The common behaviour of parametric curves in the plane: evaluation of the point and derivatives at a
/// parameter, continuity, closure, periodicity, parameter bounds, and how the parameter changes when the curve
/// is transformed or reversed.
/// </summary>
public abstract class Geom2d_Curve : Geom2d_Geometry
{
    /// <summary>Result of D1 evaluation: point and first derivative.</summary>
    public struct ResD1
    {
        public Point2D Point;
        public Vector2D D1;
    }

    /// <summary>Result of D2 evaluation: point and first two derivatives.</summary>
    public struct ResD2
    {
        public Point2D Point;
        public Vector2D D1;
        public Vector2D D2;
    }

    /// <summary>Result of D3 evaluation: point and first three derivatives.</summary>
    public struct ResD3
    {
        public Point2D Point;
        public Vector2D D1;
        public Vector2D D2;
        public Vector2D D3;
    }

    /// <summary>
    /// Reverses the direction of parametrization in place. The first and last parameters don't change; the start
    /// and end points of a bounded curve swap.
    /// </summary>
    protected internal abstract void Reverse();

    /// <summary>The parameter on the reversed curve of the point of parameter U on this one.</summary>
    public abstract double ReversedParameter(double U);

    /// <summary>
    /// The parameter on the curve transformed by T of the transform of the point of parameter U: U here, redefined
    /// by some curves (lines, for instance).
    /// </summary>
    public virtual double TransformedParameter(double U, in Transform2D T) => U;

    /// <summary>
    /// The ratio between a parameter on this curve and the parameter of the transformed point on the curve
    /// transformed by T: 1 here, redefined by some curves.
    /// </summary>
    public virtual double ParametricTransformation(in Transform2D T) => 1.0;

    /// <summary>A reversed copy of this curve.</summary>
    public Geom2d_Curve Reversed()
    {
        Geom2d_Curve C = (Copy() as Geom2d_Curve)!;
        C.Reverse();
        return C;
    }

    /// <summary>The first parameter (may be <c>RealFirst()</c> for an infinite curve).</summary>
    public abstract double FirstParameter();

    /// <summary>The last parameter (may be <c>RealLast()</c> for an infinite curve).</summary>
    public abstract double LastParameter();

    /// <summary>True if the curve is closed (its end points are within <c>gp::Resolution()</c>).</summary>
    public abstract bool IsClosed();

    /// <summary>True if the parametrization is periodic.</summary>
    public abstract bool IsPeriodic();

    /// <summary>The period of the curve.</summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    public virtual double Period()
    {
        Standard_NoSuchObject_Raise_if(!IsPeriodic(), "Geom2d_Curve::Period");
        return (LastParameter() - FirstParameter());
    }

    /// <summary>The global continuity of the curve.</summary>
    public abstract Continuity Continuity();

    /// <summary>True if the curve is at least CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public abstract bool IsCN(int N);

    /// <summary>The point of parameter U. Throws on failure.</summary>
    public abstract Point2D EvalD0(double U);

    /// <summary>The point and first derivative at U. Throws if the curve is not C1.</summary>
    public abstract ResD1 EvalD1(double U);

    /// <summary>The point and first two derivatives at U. Throws if the curve is not C2.</summary>
    public abstract ResD2 EvalD2(double U);

    /// <summary>The point and first three derivatives at U. Throws if the curve is not C3.</summary>
    public abstract ResD3 EvalD3(double U);

    /// <summary>The Nth derivative at U. Throws if the curve is not CN, or N &lt; 1.</summary>
    public abstract Vector2D EvalDN(double U, int N);

    /// <summary>The point of parameter U.</summary>
    public void D0(double U, out Point2D P) => P = EvalD0(U);

    /// <summary>The point of parameter U and the first derivative.</summary>
    public void D1(double U, out Point2D P, out Vector2D V1)
    {
        ResD1 aR = EvalD1(U);
        P = aR.Point;
        V1 = aR.D1;
    }

    /// <summary>The point of parameter U and the first two derivatives.</summary>
    public void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        ResD2 aR = EvalD2(U);
        P = aR.Point;
        V1 = aR.D1;
        V2 = aR.D2;
    }

    /// <summary>The point of parameter U and the first three derivatives.</summary>
    public void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3)
    {
        ResD3 aR = EvalD3(U);
        P = aR.Point;
        V1 = aR.D1;
        V2 = aR.D2;
        V3 = aR.D3;
    }

    /// <summary>The derivative of order N at U.</summary>
    public Vector2D DN(double U, int N) => EvalDN(U, N);

    /// <summary>The point of parameter U.</summary>
    public Point2D Value(double U)
    {
        Point2D P;
        D0(U, out P);
        return P;
    }
}
