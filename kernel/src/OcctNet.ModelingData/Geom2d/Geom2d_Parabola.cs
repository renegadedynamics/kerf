// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public Geom2d_Parabola(in gp_Parab2d Prb)
    {
        focalLength = Prb.Focal();
        pos = Prb.Axis();
    }

    /// <summary>The parabola whose axis of symmetry is MirrorAxis; Sense gives the orientation.</summary>
    /// <exception cref="Standard_ConstructionError">Focal &lt; 0.</exception>
    public Geom2d_Parabola(in gp_Ax2d MirrorAxis, double Focal, bool Sense = true)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = new gp_Ax22d(MirrorAxis, Sense);
    }

    /// <summary>The parabola of focal length Focal in the coordinate system Axis.</summary>
    /// <exception cref="Standard_ConstructionError">Focal &lt; 0.</exception>
    public Geom2d_Parabola(in gp_Ax22d Axis, double Focal)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = Axis;
    }

    /// <summary>The parabola of directrix D and focus F.</summary>
    public Geom2d_Parabola(in gp_Ax2d D, in gp_Pnt2d F)
    {
        gp_Parab2d Prb = new gp_Parab2d(D, F);
        pos = Prb.Axis();
        focalLength = Prb.Focal();
    }

    internal void SetFocal(double Focal)
    {
        CheckMutable();
        if (Focal < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        focalLength = Focal;
    }

    internal void SetParab2d(in gp_Parab2d Prb)
    {
        CheckMutable();
        focalLength = Prb.Focal();
        pos = Prb.Axis();
    }

    /// <summary>The parabola as a gp_Parab2d.</summary>
    public gp_Parab2d Parab2d() => new gp_Parab2d(pos, focalLength);

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => Precision.Infinite();

    /// <summary>False.</summary>
    public sealed override bool IsClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic() => false;

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative side of the apex.</summary>
    public gp_Ax2d Directrix()
    {
        gp_Parab2d Prb = new gp_Parab2d(pos, focalLength);
        return Prb.Directrix();
    }

    /// <summary>1.</summary>
    public sealed override double Eccentricity() => 1.0;

    /// <summary>The focus.</summary>
    public gp_Pnt2d Focus()
    {
        gp_XY Pxy = pos.XDirection().XY();
        Pxy.Multiply(focalLength);
        Pxy.Add(pos.Location().XY());
        return new gp_Pnt2d(Pxy);
    }

    /// <summary>The focal length: the distance from the apex to the focus.</summary>
    public double Focal() => focalLength;

    /// <summary>The parameter: the distance from the focus to the directrix, 2 * Focal().</summary>
    public double Parameter() => 2.0 * focalLength;

    public sealed override gp_Pnt2d EvalD0(double U) => ElCLib.ParabolaValue(U, pos, focalLength);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        ElCLib.ParabolaD1(U, pos, focalLength, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        aResult.D3.SetCoord(0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        return ElCLib.ParabolaDN(U, pos, focalLength, N);
    }

    /// <summary>Applies T to the parabola: the focal length is multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        focalLength *= Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>U * |T.ScaleFactor()|; an infinite U is returned unchanged.</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf2d T)
    {
        if (Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf2d T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy of focal length Focal.</summary>
    /// <exception cref="Standard_ConstructionError">Focal &lt; 0.</exception>
    public Geom2d_Parabola WithFocal(double Focal)
    {
        Geom2d_Parabola aCopy = (Geom2d_Parabola)Copy();
        aCopy.SetFocal(Focal);
        return aCopy;
    }

    /// <summary>A copy equal to Prb.</summary>
    public Geom2d_Parabola WithParab2d(in gp_Parab2d Prb)
    {
        Geom2d_Parabola aCopy = (Geom2d_Parabola)Copy();
        aCopy.SetParab2d(Prb);
        return aCopy;
    }
}
