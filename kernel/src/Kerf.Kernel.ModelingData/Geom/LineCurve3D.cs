// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// An infinite line, defined by an axis (gp_Ax1): its origin is the origin of the parameter, its unit vector
/// the direction of the parametrization, P(u) = O + u * Dir. The parameter range is ]-infinite, +infinite[.
/// </summary>
public sealed class LineCurve3D : Curve3D
{
    private Axis3D _pos;

    /// <summary>A line on the axis A1.</summary>
    public LineCurve3D(in Axis3D a) => _pos = a;

    /// <summary>A line equal to L.</summary>
    public LineCurve3D(in Line3D l) => _pos = l.Position;

    /// <summary>A line through P with direction V.</summary>
    public LineCurve3D(in Point3D p, in Direction3D v) => _pos = new Axis3D(p, v);

    /// <summary>A new, unfrozen line equal to this one.</summary>
    public sealed override Geometry3D Copy()
    {
        LineCurve3D l;
        l = new LineCurve3D(_pos);
        return l;
    }

    /// <summary>Reverses the direction of the line.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        _pos = _pos.Reversed();
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double u) => (-u);

    internal void SetDirection(in Direction3D v)
    {
        CheckMutable();
        _pos = _pos.WithDirection(v);
    }

    internal void SetLin(in Line3D l)
    {
        CheckMutable();
        _pos = l.Position;
    }

    internal void SetLocation(in Point3D p)
    {
        CheckMutable();
        _pos = _pos.WithLocation(p);
    }

    internal void SetPosition(in Axis3D a1)
    {
        CheckMutable();
        _pos = a1;
    }

    /// <summary>The axis of the line.</summary>
    public Axis3D Position => _pos;

    public sealed override bool IsClosed => false;

    public sealed override bool IsPeriodic => false;

    public sealed override Continuity Continuity => CN;

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The non-persistent line equal to this one.</summary>
    public Line3D Line => new Line3D(_pos);

    internal sealed override bool IsCN(int n) => true;

    /// <summary>Applies T to this line.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _pos = _pos.Transformed(t);
    }

    internal sealed override Point3D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.LineValue(u, _pos);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        result.D2 = new Vector3D(0.0, 0.0, 0.0);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(u, _pos, out result.Point, out result.D1);
        result.D2 = new Vector3D(0.0, 0.0, 0.0);
        result.D3 = new Vector3D(0.0, 0.0, 0.0);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        if (n == 1)
        {
            return new Vector3D(_pos.Direction);
        }
        else
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    internal sealed override double TransformedParameter(double u, in Transform3D t)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
        {
            return u;
        }
        return u * Math.Abs(t.ScaleFactor);
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    internal sealed override double ParametricTransformation(in Transform3D t) => Math.Abs(t.ScaleFactor);

    // Non-mutating twins of the Set methods (Kerf)

    public LineCurve3D WithDirection(in Direction3D v)
    {
        LineCurve3D copy = (LineCurve3D)Copy();
        copy.SetDirection(v);
        return copy;
    }

    public LineCurve3D WithLin(in Line3D l)
    {
        LineCurve3D copy = (LineCurve3D)Copy();
        copy.SetLin(l);
        return copy;
    }

    public LineCurve3D WithLocation(in Point3D p)
    {
        LineCurve3D copy = (LineCurve3D)Copy();
        copy.SetLocation(p);
        return copy;
    }

    public LineCurve3D WithPosition(in Axis3D a1)
    {
        LineCurve3D copy = (LineCurve3D)Copy();
        copy.SetPosition(a1);
        return copy;
    }
}
