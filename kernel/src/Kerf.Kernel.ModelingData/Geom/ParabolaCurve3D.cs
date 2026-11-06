// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;

namespace Kerf.Geometry;

/// <summary>
/// A parabola in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its apex and
/// whose X direction is its axis of symmetry: P(u) = O + u² / (4 * F) * XDir + u * YDir, u in
/// ]-infinite, +infinite[. The focal length F may be 0.
/// </summary>
public sealed class ParabolaCurve3D : ConicCurve3D
{
    private double _focalLength;

    /// <summary>A parabola equal to Prb.</summary>
    public ParabolaCurve3D(in Parabola3D parabola)
    {
        _focalLength = parabola.Focal;
        _pos = parabola.Position;
    }

    /// <summary>A parabola placed by A2 with focal length Focal.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public ParabolaCurve3D(in Frame3D frame, double focal)
    {
        _focalLength = focal;
        if (!(focal >= 0.0))
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = frame;
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public ParabolaCurve3D(in Axis3D directrix, in Point3D focus)
    {
        Parabola3D prb = new Parabola3D(directrix, focus);
        _pos = prb.Position;
        _focalLength = prb.Focal;
    }

    /// <summary>A new, unfrozen parabola equal to this one.</summary>
    public sealed override Geometry3D Copy()
    {
        ParabolaCurve3D prb;
        prb = new ParabolaCurve3D(_pos, _focalLength);
        return prb;
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double u) => (-u);

    /// <summary>Always false.</summary>
    public sealed override bool IsClosed => false;

    /// <summary>Always false.</summary>
    public sealed override bool IsPeriodic => false;

    /// <summary>1.</summary>
    public sealed override double Eccentricity => 1.0;

    /// <summary>Effectively minus infinity: -2e100, since the curve has no start.</summary>
    public sealed override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The focal distance: from the apex to the focus.</summary>
    public double Focal => _focalLength;

    /// <summary>Effectively infinity: 2e100, since the curve has no end.</summary>
    public sealed override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The parameter of the parabola: twice the focal length.</summary>
    public double Parameter => 2.0 * _focalLength;

    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    internal void SetFocal(double focal)
    {
        CheckMutable();
        if (!(focal >= 0.0))
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _focalLength = focal;
    }

    internal void SetParab(in Parabola3D prb)
    {
        CheckMutable();
        _focalLength = prb.Focal;
        _pos = prb.Position;
    }

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative X side.</summary>
    public Axis3D Directrix
    {
        get
        {
            Parabola3D prb = new Parabola3D(_pos, _focalLength);
            return prb.Directrix;
        }
    }

    internal sealed override Point3D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(u, _pos, _focalLength);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(u, _pos, _focalLength, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(u, _pos, _focalLength, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(u, _pos, _focalLength, out result.Point, out result.D1, out result.D2);
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
        return global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(u, _pos, _focalLength, n);
    }

    /// <summary>The focus.</summary>
    public Point3D Focus
    {
        get
        {
            double xp, yp, zp, xd, yd, zd;
            _pos.Location.Coord(out xp, out yp, out zp);
            _pos.XDirection.Coord(out xd, out yd, out zd);
            return new Point3D(xp + _focalLength * xd, yp + _focalLength * yd, zp + _focalLength * zd);
        }
    }

    /// <summary>The non-persistent parabola equal to this one.</summary>
    public Parabola3D Parabola => new Parabola3D(_pos, _focalLength);

    /// <summary>Applies T to this parabola (the focal length is scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D transform)
    {
        CheckMutable();
        _focalLength *= Math.Abs(transform.ScaleFactor);
        _pos = _pos.Transformed(transform);
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    internal sealed override double TransformedParameter(double u, in Transform3D transform)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
        {
            return u;
        }
        return u * Math.Abs(transform.ScaleFactor);
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    internal sealed override double ParametricTransformation(in Transform3D transform) => Math.Abs(transform.ScaleFactor);

    // Non-mutating twins of the Set methods (Kerf)

    /// <summary>A copy with this focal length; this curve is unchanged.</summary>
    /// <exception cref="ArgumentException"><paramref name="focal"/> is negative.</exception>
    public ParabolaCurve3D WithFocal(double focal)
    {
        ParabolaCurve3D copy = (ParabolaCurve3D)Copy();
        copy.SetFocal(focal);
        return copy;
    }

    /// <summary>A copy that takes its placement and focal length from <paramref name="parabola"/>; this curve is unchanged.</summary>
    public ParabolaCurve3D WithParabola(in Parabola3D parabola)
    {
        ParabolaCurve3D copy = (ParabolaCurve3D)Copy();
        copy.SetParab(parabola);
        return copy;
    }
}
