// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Hyperbola.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The main branch of a hyperbola in the plane, placed by a local coordinate system whose origin is the centre
/// and whose X axis is the major axis. P(U) = O + MajorRadius*cosh(U)*XDir + MinorRadius*sinh(U)*YDir, U in
/// ]-infinite, +infinite[. The radii may be 0, and the major radius may be less than the minor one.
/// </summary>
public class Geom2d_Hyperbola : Geom2d_Conic
{
    private double majorRadius;
    private double minorRadius;

    /// <summary>A new, unfrozen hyperbola equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_Hyperbola H;
        H = new Geom2d_Hyperbola(pos, majorRadius, minorRadius);
        return H;
    }

    /// <summary>The hyperbola equal to H.</summary>
    public Geom2d_Hyperbola(in Hyperbola2D H)
    {
        majorRadius = H.MajorRadius();
        minorRadius = H.MinorRadius();
        pos = H.Axis();
    }

    /// <summary>The hyperbola whose major axis is MajorAxis; Sense gives the orientation (true: counterclockwise).</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Geom2d_Hyperbola(in Axis2D MajorAxis, double MajorRadius, double MinorRadius, bool Sense = true)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = new Frame2D(MajorAxis, Sense);
    }

    /// <summary>The hyperbola in the coordinate system Axis, whose X axis is the major axis.</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Geom2d_Hyperbola(in Frame2D Axis, double MajorRadius, double MinorRadius)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = Axis;
    }

    internal void SetHypr2d(in Hyperbola2D H)
    {
        CheckMutable();
        majorRadius = H.MajorRadius();
        minorRadius = H.MinorRadius();
        pos = H.Axis();
    }

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

    /// <summary>The hyperbola as a gp_Hypr2d.</summary>
    public Hyperbola2D Hypr2d() => new Hyperbola2D(pos, majorRadius, minorRadius);

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

    /// <summary>The first asymptote (see gp_Hypr2d::Asymptote1, which has an OCCT bug: occt-issues.md).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius is 0.</exception>
    public Axis2D Asymptote1()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.Asymptote1();
    }

    /// <summary>The second asymptote (see gp_Hypr2d::Asymptote2, which has an OCCT bug: occt-issues.md).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius is 0.</exception>
    public Axis2D Asymptote2()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.Asymptote2();
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public Hyperbola2D ConjugateBranch1()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.ConjugateBranch1();
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public Hyperbola2D ConjugateBranch2()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.ConjugateBranch2();
    }

    /// <summary>The directrix on the positive side of the X axis.</summary>
    public Axis2D Directrix1()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.Directrix1();
    }

    /// <summary>The directrix on the negative side of the X axis (see gp_Hypr2d::Directrix2, occt-issues.md).</summary>
    public Axis2D Directrix2()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.Directrix2();
    }

    /// <summary>sqrt(MajorRadius^2 + MinorRadius^2) / MajorRadius.</summary>
    /// <exception cref="DomainErrorException">MajorRadius is 0.</exception>
    public sealed override double Eccentricity()
    {
        Standard_DomainError_Raise_if(majorRadius <= Constants.Resolution(), " ");
        return (Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius)) / majorRadius;
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public Point2D Focus1()
    {
        double C = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        Xy Pxy = pos.XDirection().XY();
        Pxy.Multiply(C);
        Pxy.Add(pos.Location().XY());
        return new Point2D(Pxy);
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point2D Focus2()
    {
        double C = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        Xy Pxy = pos.XDirection().XY();
        Pxy.Multiply(-C);
        Pxy.Add(pos.Location().XY());
        return new Point2D(Pxy);
    }

    /// <summary>The major radius.</summary>
    public double MajorRadius() => majorRadius;

    /// <summary>The minor radius.</summary>
    public double MinorRadius() => minorRadius;

    /// <summary>The other branch, symmetric about the Y axis.</summary>
    public Hyperbola2D OtherBranch()
    {
        Hyperbola2D Hv = new Hyperbola2D(pos, majorRadius, minorRadius);
        return Hv.OtherBranch();
    }

    /// <summary>MinorRadius^2 / MajorRadius.</summary>
    /// <exception cref="DomainErrorException">MajorRadius is 0.</exception>
    public double Parameter()
    {
        Standard_DomainError_Raise_if(majorRadius <= Constants.Resolution(), " ");
        return (minorRadius * minorRadius) / majorRadius;
    }

    public sealed override Point2D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(U, pos, majorRadius, minorRadius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
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

    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector2D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        return global::Kerf.Engine.Geometry.ElCLib.HyperbolaDN(U, pos, majorRadius, minorRadius, N);
    }

    /// <summary>Applies T to the hyperbola: the radii are multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D T)
    {
        CheckMutable();
        majorRadius = majorRadius * Math.Abs(T.ScaleFactor());
        minorRadius = minorRadius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to H.</summary>
    public Geom2d_Hyperbola WithHypr2d(in Hyperbola2D H)
    {
        Geom2d_Hyperbola aCopy = (Geom2d_Hyperbola)Copy();
        aCopy.SetHypr2d(H);
        return aCopy;
    }

    /// <summary>A copy with the major radius MajorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; 0.</exception>
    public Geom2d_Hyperbola WithMajorRadius(double MajorRadius)
    {
        Geom2d_Hyperbola aCopy = (Geom2d_Hyperbola)Copy();
        aCopy.SetMajorRadius(MajorRadius);
        return aCopy;
    }

    /// <summary>A copy with the minor radius MinorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0.</exception>
    public Geom2d_Hyperbola WithMinorRadius(double MinorRadius)
    {
        Geom2d_Hyperbola aCopy = (Geom2d_Hyperbola)Copy();
        aCopy.SetMinorRadius(MinorRadius);
        return aCopy;
    }
}
