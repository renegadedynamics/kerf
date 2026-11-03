// Port of: src/ModelingData/TKG3d/Geom/Geom_Hyperbola.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The positive branch of a hyperbola in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose
/// origin is its centre and whose X direction is its major axis: P(u) = O + MajRad * cosh(u) * XDir +
/// MinRad * sinh(u) * YDir, u in ]-infinite, +infinite[. The major radius may be less than the minor radius.
/// </summary>
public class Geom_Hyperbola : Geom_Conic
{
    private double majorRadius;
    private double minorRadius;

    /// <summary>A hyperbola equal to H.</summary>
    public Geom_Hyperbola(in Hyperbola3D H)
    {
        majorRadius = H.MajorRadius();
        minorRadius = H.MinorRadius();
        pos = H.Position();
    }

    /// <summary>A hyperbola placed by A with the given radii.</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Geom_Hyperbola(in Frame3D A, double MajorRadius, double MinorRadius)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = A;
    }

    /// <summary>A new, unfrozen hyperbola equal to this one.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_Hyperbola H;
        H = new Geom_Hyperbola(pos, majorRadius, minorRadius);
        return H;
    }

    public sealed override bool IsClosed() => false;

    public sealed override bool IsPeriodic() => false;

    public sealed override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    public sealed override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    public double MajorRadius() => majorRadius;

    public double MinorRadius() => minorRadius;

    internal void SetHypr(in Hyperbola3D H)
    {
        CheckMutable();
        majorRadius = H.MajorRadius();
        minorRadius = H.MinorRadius();
        pos = H.Position();
    }

    /// <exception cref="ConstructionErrorException">MajorRadius &lt; 0.</exception>
    internal void SetMajorRadius(double MajorRadius)
    {
        CheckMutable();
        if (MajorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        else
        {
            majorRadius = MajorRadius;
        }
    }

    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0.</exception>
    internal void SetMinorRadius(double MinorRadius)
    {
        CheckMutable();
        if (MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        else
        {
            minorRadius = MinorRadius;
        }
    }

    /// <summary>The non-persistent hyperbola equal to this one.</summary>
    public Hyperbola3D Hypr() => new Hyperbola3D(pos, majorRadius, minorRadius);

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    /// <summary>The first asymptote (see gp_Hypr).</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public Axis3D Asymptote1()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.Asymptote1();
    }

    /// <summary>The second asymptote (see gp_Hypr).</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public Axis3D Asymptote2()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.Asymptote2();
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public Hyperbola3D ConjugateBranch1()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.ConjugateBranch1();
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public Hyperbola3D ConjugateBranch2()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.ConjugateBranch2();
    }

    /// <summary>The first directrix (see gp_Hypr).</summary>
    public Axis3D Directrix1()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.Directrix1();
    }

    /// <summary>The second directrix (see gp_Hypr).</summary>
    public Axis3D Directrix2()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.Directrix2();
    }

    public sealed override Point3D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(U, pos, majorRadius, minorRadius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD3(U,
                           pos,
                           majorRadius,
                           minorRadius,
                           out aResult.Point,
                           out aResult.D1,
                           out aResult.D2,
                           out aResult.D3);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return global::Kerf.Engine.Geometry.ElCLib.HyperbolaDN(U, pos, majorRadius, minorRadius, N);
    }

    /// <summary>sqrt(a² + b²) / a.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public sealed override double Eccentricity()
    {
        Standard_ConstructionError_Raise_if(majorRadius == 0.0, " ");
        return (Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius)) / majorRadius;
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public Point3D Focus1()
    {
        double C = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new Point3D(Xp + C * Xd, Yp + C * Yd, Zp + C * Zd);
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point3D Focus2()
    {
        double C = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new Point3D(Xp - C * Xd, Yp - C * Yd, Zp - C * Zd);
    }

    /// <summary>The branch symmetric to this one about the Y axis.</summary>
    public Hyperbola3D OtherBranch()
    {
        Hyperbola3D Hv = new Hyperbola3D(pos, majorRadius, minorRadius);
        return Hv.OtherBranch();
    }

    /// <summary>The parameter of the hyperbola: b² / a.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public double Parameter()
    {
        Standard_ConstructionError_Raise_if(majorRadius == 0.0, " ");
        return (minorRadius * minorRadius) / majorRadius;
    }

    /// <summary>Applies T to this hyperbola (the radii are scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        majorRadius = majorRadius * Math.Abs(T.ScaleFactor());
        minorRadius = minorRadius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Hyperbola WithHypr(in Hyperbola3D H)
    {
        Geom_Hyperbola aCopy = (Geom_Hyperbola)Copy();
        aCopy.SetHypr(H);
        return aCopy;
    }

    public Geom_Hyperbola WithMajorRadius(double MajorRadius)
    {
        Geom_Hyperbola aCopy = (Geom_Hyperbola)Copy();
        aCopy.SetMajorRadius(MajorRadius);
        return aCopy;
    }

    public Geom_Hyperbola WithMinorRadius(double MinorRadius)
    {
        Geom_Hyperbola aCopy = (Geom_Hyperbola)Copy();
        aCopy.SetMinorRadius(MinorRadius);
        return aCopy;
    }
}
