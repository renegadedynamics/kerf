// Port of: src/ModelingData/TKG3d/Geom/Geom_Circle.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A circle in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its centre:
/// P(u) = O + R * cos(u) * XDir + R * sin(u) * YDir, u in [0, 2 * PI[, periodic.
/// </summary>
public sealed class CircleCurve3D : ConicCurve3D
{
    private double _radius;

    /// <summary>A circle equal to C.</summary>
    public CircleCurve3D(in Circle3D c)
    {
        _radius = c.Radius;
        _pos = c.Position;
    }

    /// <summary>A circle of radius R placed by A2.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public CircleCurve3D(in Frame3D a2, double r)
    {
        _radius = r;
        if (r < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = a2;
    }

    /// <summary>A new, unfrozen circle equal to this one.</summary>
    public sealed override Geometry3D Copy()
    {
        CircleCurve3D c;
        c = new CircleCurve3D(_pos, _radius);
        return c;
    }

    public sealed override bool IsClosed => true;

    public sealed override bool IsPeriodic => true;

    /// <summary>2 * PI - U.</summary>
    public sealed override double ReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>0.</summary>
    public sealed override double Eccentricity => 0.0;

    public sealed override double FirstParameter => 0.0;

    public sealed override double LastParameter => 2.0 * M_PI;

    /// <summary>The non-persistent circle equal to this one.</summary>
    public Circle3D Circle => new Circle3D(_pos, _radius);

    internal void SetCirc(in Circle3D c)
    {
        CheckMutable();
        _radius = c.Radius;
        _pos = c.Position;
    }

    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    internal void SetRadius(double r)
    {
        CheckMutable();
        if (r < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _radius = r;
    }

    public double Radius => _radius;

    internal sealed override Point3D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.CircleValue(u, _pos, _radius);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.CircleD1(u, _pos, _radius, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.CircleD2(u, _pos, _radius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.CircleD3(u, _pos, _radius, out result.Point, out result.D1, out result.D2, out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        return global::Kerf.Engine.Geometry.ElCLib.CircleDN(u, _pos, _radius, n);
    }

    /// <summary>Applies T to this circle (the radius is scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _radius = _radius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public CircleCurve3D WithCirc(in Circle3D c)
    {
        CircleCurve3D copy = (CircleCurve3D)Copy();
        copy.SetCirc(c);
        return copy;
    }

    public CircleCurve3D WithRadius(double r)
    {
        CircleCurve3D copy = (CircleCurve3D)Copy();
        copy.SetRadius(r);
        return copy;
    }
}
