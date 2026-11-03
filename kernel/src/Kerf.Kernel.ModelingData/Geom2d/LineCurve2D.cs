// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Line.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// An infinite line in the plane, placed by an axis (gp_Ax2d): its origin is the origin of the parameter and its
/// unit direction the positive sense, so the parameter of a point is its distance along the line from the origin.
/// The parameter range is ]-infinite, +infinite[.
/// </summary>
public sealed class LineCurve2D : Curve2D
{
    private Axis2D _pos;

    /// <summary>A new, unfrozen line equal to this one.</summary>
    public sealed override Geometry2D Copy()
    {
        LineCurve2D l;
        l = new LineCurve2D(_pos);
        return l;
    }

    /// <summary>The line on the axis A: the origin and direction of A give the parametrization.</summary>
    public LineCurve2D(in Axis2D a) => _pos = a;

    /// <summary>The line equal to L.</summary>
    public LineCurve2D(in Line2D l) => _pos = l.Position;

    /// <summary>The line through P with the direction V.</summary>
    public LineCurve2D(in Point2D p, in Direction2D v) => _pos = new Axis2D(p, v);

    internal void SetDirection(in Direction2D v)
    {
        CheckMutable();
        _pos = _pos.WithDirection(v);
    }

    /// <summary>The unit direction of the line.</summary>
    public Direction2D Direction => _pos.Direction;

    internal void SetLin2d(in Line2D l)
    {
        CheckMutable();
        _pos = l.Position;
    }

    internal void SetLocation(in Point2D p)
    {
        CheckMutable();
        _pos = _pos.WithLocation(p);
    }

    /// <summary>The origin of the line.</summary>
    public Point2D Location => _pos.Location;

    internal void SetPosition(in Axis2D a)
    {
        CheckMutable();
        _pos = a;
    }

    /// <summary>The axis of the line.</summary>
    public Axis2D Position => _pos;

    /// <summary>The line as a gp_Lin2d.</summary>
    public Line2D Line => new Line2D(_pos);

    /// <summary>Reverses the direction of the line.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        _pos = _pos.Reversed();
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double u) => (-u);

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>False.</summary>
    public sealed override bool IsClosed => false;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic => false;

    /// <summary>CN.</summary>
    public sealed override Continuity Continuity => CN;

    /// <summary>The distance between P and the line.</summary>
    public double Distance(in Point2D p)
    {
        Line2D l = new Line2D(_pos);
        return l.DistanceTo(p);
    }

    /// <summary>True.</summary>
    internal sealed override bool IsCN(int n) => true;

    internal sealed override Point2D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.LineValue(u, _pos);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        result.D2 = result.D2.WithCoord(0.0, 0.0);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        result.D2 = result.D2.WithCoord(0.0, 0.0);
        result.D3 = result.D3.WithCoord(0.0, 0.0);
        return result;
    }

    /// <summary>The direction for N = 1, a null vector above.</summary>
    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException();
        }
        if (n == 1)
        {
            return new Vector2D(_pos.Direction);
        }
        else
        {
            return new Vector2D(0.0, 0.0);
        }
    }

    /// <summary>Applies T to the line's axis.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        _pos = _pos.Transformed(t);
    }

    /// <summary>U * |T.ScaleFactor()|; an infinite U is returned unchanged.</summary>
    internal sealed override double TransformedParameter(double u, in Transform2D t)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
        {
            return u;
        }
        return u * Math.Abs(t.ScaleFactor);
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    internal sealed override double ParametricTransformation(in Transform2D t) => Math.Abs(t.ScaleFactor);

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to L.</summary>
    public LineCurve2D WithLin2d(in Line2D l)
    {
        LineCurve2D copy = (LineCurve2D)Copy();
        copy.SetLin2d(l);
        return copy;
    }

    /// <summary>A copy with the direction V.</summary>
    public LineCurve2D WithDirection(in Direction2D v)
    {
        LineCurve2D copy = (LineCurve2D)Copy();
        copy.SetDirection(v);
        return copy;
    }

    /// <summary>A copy with the origin P.</summary>
    public LineCurve2D WithLocation(in Point2D p)
    {
        LineCurve2D copy = (LineCurve2D)Copy();
        copy.SetLocation(p);
        return copy;
    }

    /// <summary>A copy on the axis A.</summary>
    public LineCurve2D WithPosition(in Axis2D a)
    {
        LineCurve2D copy = (LineCurve2D)Copy();
        copy.SetPosition(a);
        return copy;
    }
}
