// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

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
    public Geom_Parabola(in Parabola3D Prb)
    {
        focalLength = Prb.Focal();
        pos = Prb.Position();
    }

    /// <summary>A parabola placed by A2 with focal length Focal.</summary>
    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    public Geom_Parabola(in Frame3D A2, double Focal)
    {
        focalLength = Focal;
        if (Focal < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = A2;
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public Geom_Parabola(in Axis3D D, in Point3D F)
    {
        Parabola3D Prb = new Parabola3D(D, F);
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

    public sealed override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The focal distance: from the apex to the focus.</summary>
    public double Focal() => focalLength;

    public sealed override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The parameter of the parabola: twice the focal length.</summary>
    public double Parameter() => 2.0 * focalLength;

    /// <exception cref="ConstructionErrorException">Focal &lt; 0.</exception>
    internal void SetFocal(double Focal)
    {
        CheckMutable();
        if (Focal < 0.0)
        {
            throw new ConstructionErrorException();
        }
        focalLength = Focal;
    }

    internal void SetParab(in Parabola3D Prb)
    {
        CheckMutable();
        focalLength = Prb.Focal();
        pos = Prb.Position();
    }

    /// <summary>The directrix: parallel to the Y axis, at the focal distance on the negative X side.</summary>
    public Axis3D Directrix()
    {
        Parabola3D Prb = new Parabola3D(pos, focalLength);
        return Prb.Directrix();
    }

    public sealed override Point3D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(U, pos, focalLength);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(U, pos, focalLength, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(U, pos, focalLength, out aResult.Point, out aResult.D1, out aResult.D2);
        aResult.D3 = new Vector3D(0.0, 0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(U, pos, focalLength, N);
    }

    /// <summary>The focus.</summary>
    public Point3D Focus()
    {
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new Point3D(Xp + focalLength * Xd, Yp + focalLength * Yd, Zp + focalLength * Zd);
    }

    /// <summary>The non-persistent parabola equal to this one.</summary>
    public Parabola3D Parab() => new Parabola3D(pos, focalLength);

    /// <summary>Applies T to this parabola (the focal length is scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        focalLength *= Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    public sealed override double TransformedParameter(double U, in Transform3D T)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in Transform3D T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Parabola WithFocal(double Focal)
    {
        Geom_Parabola aCopy = (Geom_Parabola)Copy();
        aCopy.SetFocal(Focal);
        return aCopy;
    }

    public Geom_Parabola WithParab(in Parabola3D Prb)
    {
        Geom_Parabola aCopy = (Geom_Parabola)Copy();
        aCopy.SetParab(Prb);
        return aCopy;
    }
}
