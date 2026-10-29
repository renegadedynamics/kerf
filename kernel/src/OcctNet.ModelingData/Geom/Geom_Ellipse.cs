// Port of: src/ModelingData/TKG3d/Geom/Geom_Ellipse.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An ellipse in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its centre and
/// whose X direction is its major axis: P(u) = O + MajorRad * cos(u) * XDir + MinorRad * sin(u) * YDir,
/// u in [0, 2 * PI[, periodic. The major radius may equal the minor radius.
/// </summary>
public class Geom_Ellipse : Geom_Conic
{
    private double majorRadius;
    private double minorRadius;

    /// <summary>An ellipse equal to E.</summary>
    public Geom_Ellipse(in gp_Elips E)
    {
        majorRadius = E.MajorRadius();
        minorRadius = E.MinorRadius();
        pos = E.Position();
    }

    /// <summary>An ellipse placed by A with the given radii.</summary>
    /// <exception cref="Standard_ConstructionError">MajorRadius &lt; MinorRadius, or MinorRadius &lt; 0.</exception>
    public Geom_Ellipse(in gp_Ax2 A, double MajorRadius, double MinorRadius)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < MinorRadius || MinorRadius < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = A;
    }

    /// <summary>A new, unfrozen ellipse equal to this one.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_Ellipse E;
        E = new Geom_Ellipse(pos, majorRadius, minorRadius);
        return E;
    }

    public sealed override bool IsClosed() => true;

    public sealed override bool IsPeriodic() => true;

    public sealed override double FirstParameter() => 0.0;

    public sealed override double LastParameter() => 2.0 * M_PI;

    public double MajorRadius() => majorRadius;

    public double MinorRadius() => minorRadius;

    internal void SetElips(in gp_Elips E)
    {
        CheckMutable();
        majorRadius = E.MajorRadius();
        minorRadius = E.MinorRadius();
        pos = E.Position();
    }

    /// <exception cref="Standard_ConstructionError">MajorRadius &lt; the minor radius.</exception>
    internal void SetMajorRadius(double MajorRadius)
    {
        CheckMutable();
        if (MajorRadius < minorRadius)
        {
            throw new Standard_ConstructionError();
        }
        else
        {
            majorRadius = MajorRadius;
        }
    }

    /// <exception cref="Standard_ConstructionError">MinorRadius &lt; 0 or more than the major radius.</exception>
    internal void SetMinorRadius(double MinorRadius)
    {
        CheckMutable();
        if (MinorRadius < 0 || majorRadius < MinorRadius)
        {
            throw new Standard_ConstructionError();
        }
        else
        {
            minorRadius = MinorRadius;
        }
    }

    /// <summary>The non-persistent ellipse equal to this one.</summary>
    public gp_Elips Elips() => new gp_Elips(pos, majorRadius, minorRadius);

    /// <summary>2 * PI - U.</summary>
    public sealed override double ReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>The first directrix (see gp_Elips).</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is 0.</exception>
    public gp_Ax1 Directrix1()
    {
        gp_Elips Ev = new gp_Elips(pos, majorRadius, minorRadius);
        return Ev.Directrix1();
    }

    /// <summary>The second directrix, symmetric to the first about the main axis.</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is 0.</exception>
    public gp_Ax1 Directrix2()
    {
        gp_Elips Ev = new gp_Elips(pos, majorRadius, minorRadius);
        return Ev.Directrix2();
    }

    public sealed override gp_Pnt EvalD0(double U) => ElCLib.EllipseValue(U, pos, majorRadius, minorRadius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.EllipseD1(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.EllipseD2(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.EllipseD3(U,
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
    public sealed override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return ElCLib.EllipseDN(U, pos, majorRadius, minorRadius, N);
    }

    /// <summary>sqrt(a² - b²) / a, 0 when the major radius is 0.</summary>
    public sealed override double Eccentricity()
    {
        if (majorRadius == 0.0)
        {
            return 0.0;
        }
        else
        {
            return (Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius)) / majorRadius;
        }
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public gp_Pnt Focus1()
    {
        double C = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new gp_Pnt(Xp + C * Xd, Yp + C * Yd, Zp + C * Zd);
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public gp_Pnt Focus2()
    {
        double C = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        double Xp, Yp, Zp, Xd, Yd, Zd;
        pos.Location().Coord(out Xp, out Yp, out Zp);
        pos.XDirection().Coord(out Xd, out Yd, out Zd);
        return new gp_Pnt(Xp - C * Xd, Yp - C * Yd, Zp - C * Zd);
    }

    /// <summary>The parameter of the ellipse: b² / a (0 when a is 0).</summary>
    public double Parameter()
    {
        if (majorRadius == 0.0)
        {
            return 0.0;
        }
        else
        {
            return (minorRadius * minorRadius) / majorRadius;
        }
    }

    /// <summary>Applies T to this ellipse (the radii are scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        majorRadius = majorRadius * Math.Abs(T.ScaleFactor());
        minorRadius = minorRadius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Ellipse WithElips(in gp_Elips E)
    {
        Geom_Ellipse aCopy = (Geom_Ellipse)Copy();
        aCopy.SetElips(E);
        return aCopy;
    }

    public Geom_Ellipse WithMajorRadius(double MajorRadius)
    {
        Geom_Ellipse aCopy = (Geom_Ellipse)Copy();
        aCopy.SetMajorRadius(MajorRadius);
        return aCopy;
    }

    public Geom_Ellipse WithMinorRadius(double MinorRadius)
    {
        Geom_Ellipse aCopy = (Geom_Ellipse)Copy();
        aCopy.SetMinorRadius(MinorRadius);
        return aCopy;
    }
}
