// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Circle.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A circle in the plane, placed by a local coordinate system whose origin is the centre. The parameter is the
/// angle from the X axis, in [0, 2*PI[, in the sense of the coordinate system: P(U) = O + R*cos(U)*XDir +
/// R*sin(U)*YDir. The curve is closed and periodic, of period 2*PI.
/// </summary>
public class CircleCurve2D : ConicCurve2D
{
    private double _radius;

    /// <summary>A new, unfrozen circle equal to this one.</summary>
    public sealed override Geometry2D Copy()
    {
        CircleCurve2D c;
        c = new CircleCurve2D(_pos, _radius);
        return c;
    }

    /// <summary>The circle equal to C.</summary>
    public CircleCurve2D(in Circle2D c)
    {
        _radius = c.Radius;
        _pos = c.Axis;
    }

    /// <summary>
    /// The circle of centre A's origin and radius Radius; A is the X axis, and Sense the orientation (true:
    /// counterclockwise).
    /// </summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public CircleCurve2D(in Axis2D a, double radius, bool sense = true)
    {
        _radius = radius;
        if (radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = new Frame2D(a, sense);
    }

    /// <summary>The circle of radius Radius in the coordinate system A.</summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public CircleCurve2D(in Frame2D a, double radius)
    {
        _radius = radius;
        if (radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = a;
    }

    internal void SetCirc2d(in Circle2D c)
    {
        CheckMutable();
        _radius = c.Radius;
        _pos = c.Axis;
    }

    internal void SetRadius(double r)
    {
        CheckMutable();
        if (r < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _radius = r;
    }

    /// <summary>The radius.</summary>
    public double Radius => _radius;

    /// <summary>The circle as a gp_Circ2d.</summary>
    public Circle2D Circle => new Circle2D(_pos, _radius);

    /// <summary>2*PI - U.</summary>
    public sealed override double ReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>0.</summary>
    public sealed override double Eccentricity => 0.0;

    /// <summary>0.</summary>
    public sealed override double FirstParameter => 0.0;

    /// <summary>2*PI.</summary>
    public sealed override double LastParameter => 2.0 * M_PI;

    /// <summary>True.</summary>
    public sealed override bool IsClosed => true;

    /// <summary>True.</summary>
    public sealed override bool IsPeriodic => true;

    internal sealed override Point2D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.CircleValue(u, _pos, _radius);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElCLib.CircleD1(u, _pos, _radius, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElCLib.CircleD2(u, _pos, _radius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result;
        global::Kerf.Engine.Geometry.ElCLib.CircleD3(u, _pos, _radius, out result.Point, out result.D1, out result.D2, out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException();
        }
        return global::Kerf.Engine.Geometry.ElCLib.CircleDN(u, _pos, _radius, n);
    }

    /// <summary>Applies T to the circle: the radius is multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        _radius = _radius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to C.</summary>
    public CircleCurve2D WithCirc2d(in Circle2D c)
    {
        CircleCurve2D copy = (CircleCurve2D)Copy();
        copy.SetCirc2d(c);
        return copy;
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public CircleCurve2D WithRadius(double r)
    {
        CircleCurve2D copy = (CircleCurve2D)Copy();
        copy.SetRadius(r);
        return copy;
    }
}
