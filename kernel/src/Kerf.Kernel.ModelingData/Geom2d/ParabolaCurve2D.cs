// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// A parabola in the plane, placed by a local coordinate system whose origin is the apex and whose X axis is the
/// axis of symmetry. P(U) = O + U^2/(4*F)*XDir + U*YDir, where F is the focal length; U in ]-infinite,
/// +infinite[. A focal length of 0 is accepted.
/// </summary>
public sealed class ParabolaCurve2D : ConicCurve2D
{
    private double _focalLength;

    /// <summary>A new, unfrozen parabola equal to this one.</summary>
    public sealed override Geometry2D Copy()
    {
        ParabolaCurve2D prb;
        prb = new ParabolaCurve2D(_pos, _focalLength);
        return prb;
    }

    /// <summary>The parabola equal to Prb.</summary>
    public ParabolaCurve2D(in Parabola2D parabola)
    {
        _focalLength = parabola.Focal;
        _pos = parabola.Axis;
    }

    /// <summary>The parabola whose axis of symmetry is MirrorAxis; Sense gives the orientation.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public ParabolaCurve2D(in Axis2D mirrorAxis, double focal, bool sense = true)
    {
        _focalLength = focal;
        if (focal < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = new Frame2D(mirrorAxis, sense);
    }

    /// <summary>The parabola of focal length Focal in the coordinate system Axis.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public ParabolaCurve2D(in Frame2D axis, double focal)
    {
        _focalLength = focal;
        if (focal < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = axis;
    }

    /// <summary>The parabola of directrix D and focus F.</summary>
    public ParabolaCurve2D(in Axis2D directrix, in Point2D focus)
    {
        Parabola2D prb = new Parabola2D(directrix, focus);
        _pos = prb.Axis;
        _focalLength = prb.Focal;
    }

    internal void SetFocal(double focal)
    {
        CheckMutable();
        if (focal < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _focalLength = focal;
    }

    internal void SetParab2d(in Parabola2D prb)
    {
        CheckMutable();
        _focalLength = prb.Focal;
        _pos = prb.Axis;
    }

    /// <summary>The parabola as a gp_Parab2d.</summary>
    public Parabola2D Parabola => new Parabola2D(_pos, _focalLength);

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

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative side of the apex.</summary>
    public Axis2D Directrix
    {
        get
        {
            Parabola2D prb = new Parabola2D(_pos, _focalLength);
            return prb.Directrix;
        }
    }

    /// <summary>1.</summary>
    public sealed override double Eccentricity => 1.0;

    /// <summary>The focus.</summary>
    public Point2D Focus
    {
        get
        {
            Xy pxy = _pos.XDirection.XY();
            pxy.Multiply(_focalLength);
            pxy.Add(_pos.Location.XY());
            return new Point2D(pxy);
        }
    }

    /// <summary>The focal length: the distance from the apex to the focus.</summary>
    public double Focal => _focalLength;

    /// <summary>The parameter: the distance from the focus to the directrix, 2 * Focal().</summary>
    public double Parameter => 2.0 * _focalLength;

    internal sealed override Point2D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(u, _pos, _focalLength);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(u, _pos, _focalLength, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(u, _pos, _focalLength, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(u, _pos, _focalLength, out result.Point, out result.D1, out result.D2);
        result.D3 = result.D3.WithCoord(0.0, 0.0);
        return result;
    }

    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative2DException());
        }
        return global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(u, _pos, _focalLength, n);
    }

    /// <summary>Applies T to the parabola: the focal length is multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D transform)
    {
        CheckMutable();
        _focalLength *= Math.Abs(transform.ScaleFactor);
        _pos = _pos.Transformed(transform);
    }

    /// <summary>U * |T.ScaleFactor()|; an infinite U is returned unchanged.</summary>
    internal sealed override double TransformedParameter(double u, in Transform2D transform)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
        {
            return u;
        }
        return u * Math.Abs(transform.ScaleFactor);
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    internal sealed override double ParametricTransformation(in Transform2D transform) => Math.Abs(transform.ScaleFactor);

    // Non-mutating twins of the Set methods (Kerf)

    /// <summary>A copy of focal length Focal.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public ParabolaCurve2D WithFocal(double focal)
    {
        ParabolaCurve2D copy = (ParabolaCurve2D)Copy();
        copy.SetFocal(focal);
        return copy;
    }

    /// <summary>A copy equal to Prb.</summary>
    public ParabolaCurve2D WithParabola(in Parabola2D parabola)
    {
        ParabolaCurve2D copy = (ParabolaCurve2D)Copy();
        copy.SetParab2d(parabola);
        return copy;
    }
}
