// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A parabola in the plane, placed by a local coordinate system whose origin is the apex and whose X axis is the
/// axis of symmetry. P(U) = O + U^2/(4*F)*XDir + U*YDir, where F is the focal length; U in ]-infinite,
/// +infinite[. A focal length of 0 is accepted.
/// </summary>
public class Geom2d_Parabola : Geom2d_Conic
{
    private double focalLength;

    /// <summary>A new, unfrozen parabola equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_Parabola Prb;
        Prb = new Geom2d_Parabola(pos, focalLength);
        return Prb;
    }

    /// <summary>The parabola equal to Prb.</summary>
    public Geom2d_Parabola(in Parabola2D Prb)
    {
        focalLength = Prb.Focal();
        pos = Prb.Axis();
    }

    /// <summary>The parabola whose axis of symmetry is MirrorAxis; Sense gives the orientation.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public Geom2d_Parabola(in Axis2D MirrorAxis, double Focal, bool Sense = true)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = new Frame2D(MirrorAxis, Sense);
    }

    /// <summary>The parabola of focal length Focal in the coordinate system Axis.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public Geom2d_Parabola(in Frame2D Axis, double Focal)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = Axis;
    }

    /// <summary>The parabola of directrix D and focus F.</summary>
    public Geom2d_Parabola(in Axis2D D, in Point2D F)
    {
        Parabola2D Prb = new Parabola2D(D, F);
        pos = Prb.Axis();
        focalLength = Prb.Focal();
    }

    internal void SetFocal(double Focal)
    {
        CheckMutable();
        if (Focal < 0.0)
        {
            throw new ConstructionErrorException();
        }
        focalLength = Focal;
    }

    internal void SetParab2d(in Parabola2D Prb)
    {
        CheckMutable();
        focalLength = Prb.Focal();
        pos = Prb.Axis();
    }

    /// <summary>The parabola as a gp_Parab2d.</summary>
    public Parabola2D Parab2d() => new Parabola2D(pos, focalLength);

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>False.</summary>
    public sealed override bool IsClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic() => false;

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative side of the apex.</summary>
    public Axis2D Directrix()
    {
        Parabola2D Prb = new Parabola2D(pos, focalLength);
        return Prb.Directrix();
    }

    /// <summary>1.</summary>
    public sealed override double Eccentricity() => 1.0;

    /// <summary>The focus.</summary>
    public Point2D Focus()
    {
        Xy Pxy = pos.XDirection().XY();
        Pxy.Multiply(focalLength);
        Pxy.Add(pos.Location().XY());
        return new Point2D(Pxy);
    }

    /// <summary>The focal length: the distance from the apex to the focus.</summary>
    public double Focal() => focalLength;

    /// <summary>The parameter: the distance from the focus to the directrix, 2 * Focal().</summary>
    public double Parameter() => 2.0 * focalLength;

    public sealed override Point2D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(U, pos, focalLength);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(U, pos, focalLength, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        aResult.D3.SetCoord(0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector2D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        return global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(U, pos, focalLength, N);
    }

    /// <summary>Applies T to the parabola: the focal length is multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D T)
    {
        CheckMutable();
        focalLength *= Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>U * |T.ScaleFactor()|; an infinite U is returned unchanged.</summary>
    public sealed override double TransformedParameter(double U, in Transform2D T)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in Transform2D T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy of focal length Focal.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public Geom2d_Parabola WithFocal(double Focal)
    {
        Geom2d_Parabola aCopy = (Geom2d_Parabola)Copy();
        aCopy.SetFocal(Focal);
        return aCopy;
    }

    /// <summary>A copy equal to Prb.</summary>
    public Geom2d_Parabola WithParab2d(in Parabola2D Prb)
    {
        Geom2d_Parabola aCopy = (Geom2d_Parabola)Copy();
        aCopy.SetParab2d(Prb);
        return aCopy;
    }
}
