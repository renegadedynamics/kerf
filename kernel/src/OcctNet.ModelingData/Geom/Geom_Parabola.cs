// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A parabola in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its apex and
/// whose X direction is its axis of symmetry: P(u) = O + u² / (4 * F) * XDir + u * YDir, u in
/// ]-infinite, +infinite[. The focal length F may be 0.
/// </summary>
public class Geom_Parabola : Geom_Conic
{
    private double focalLength;

    /// <summary>A parabola equal to Prb.</summary>
    public Geom_Parabola(in gp_Parab Prb)
    {
        focalLength = Prb.Focal();
        pos = Prb.Position();
    }

    /// <summary>A parabola placed by A2 with focal length Focal.</summary>
    /// <exception cref="Standard_ConstructionError">Focal &lt; 0.</exception>
    public Geom_Parabola(in gp_Ax2 A2, double Focal)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = A2;
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public Geom_Parabola(in gp_Ax1 D, in gp_Pnt F)
    {
        gp_Parab Prb = new gp_Parab(D, F);
        pos = Prb.Position();
        focalLength = Prb.Focal();
    }

    /// <summary>A new, unfrozen parabola equal to this one.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_Parabola Prb;
        Prb = new Geom_Parabola(pos, focalLength);
        return Prb;
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    public sealed override bool IsClosed() => false;

    public sealed override bool IsPeriodic() => false;

    /// <summary>1.</summary>
    public sealed override double Eccentricity() => 1.0;

    public sealed override double FirstParameter() => -Precision.Infinite();

    /// <summary>The focal distance: from the apex to the focus.</summary>
    public double Focal() => focalLength;

    public sealed override double LastParameter() => Precision.Infinite();

    /// <summary>The parameter of the parabola: twice the focal length.</summary>
    public double Parameter() => 2.0 * focalLength;

    /// <exception cref="Standard_ConstructionError">Focal &lt; 0.</exception>
    internal void SetFocal(double Focal)
    {
        CheckMutable();
        if (Focal < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        focalLength = Focal;
    }

    internal void SetParab(in gp_Parab Prb)
    {
        CheckMutable();
        focalLength = Prb.Focal();
        pos = Prb.Position();
    }

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative X side.</summary>
    public gp_Ax1 Directrix()
    {
        gp_Parab Prb = new gp_Parab(pos, focalLength);
        return Prb.Directrix();
    }

    public sealed override gp_Pnt EvalD0(double U) => ElCLib.ParabolaValue(U, pos, focalLength);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.ParabolaD1(U, pos, focalLength, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        aResult.D3 = new gp_Vec(0.0, 0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return ElCLib.ParabolaDN(U, pos, focalLength, N);
    }

    /// <summary>The focus.</summary>
    public gp_Pnt Focus()
    {
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new gp_Pnt(Xp + focalLength * Xd, Yp + focalLength * Yd, Zp + focalLength * Zd);
    }

    /// <summary>The non-persistent parabola equal to this one.</summary>
    public gp_Parab Parab() => new gp_Parab(pos, focalLength);

    /// <summary>Applies T to this parabola (the focal length is scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        focalLength *= Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf T)
    {
        if (Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Parabola WithFocal(double Focal)
    {
        Geom_Parabola aCopy = (Geom_Parabola)Copy();
        aCopy.SetFocal(Focal);
        return aCopy;
    }

    public Geom_Parabola WithParab(in gp_Parab Prb)
    {
        Geom_Parabola aCopy = (Geom_Parabola)Copy();
        aCopy.SetParab(Prb);
        return aCopy;
    }
}
