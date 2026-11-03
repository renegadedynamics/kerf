// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Ellipse.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An ellipse in the plane, placed by a local coordinate system whose origin is the centre and whose X axis is
/// the major axis. P(U) = O + MajorRadius*cos(U)*XDir + MinorRadius*sin(U)*YDir, U in [0, 2*PI[; the curve is
/// closed and periodic. The major radius may equal the minor radius (a circle) and the minor radius may be 0.
/// </summary>
public class Geom2d_Ellipse : Geom2d_Conic
{
    private double majorRadius;
    private double minorRadius;

    /// <summary>A new, unfrozen ellipse equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_Ellipse E;
        E = new Geom2d_Ellipse(pos, majorRadius, minorRadius);
        return E;
    }

    /// <summary>The ellipse equal to E.</summary>
    public Geom2d_Ellipse(in Ellipse2D E)
    {
        majorRadius = E.MajorRadius();
        minorRadius = E.MinorRadius();
        pos = E.Axis();
    }

    /// <summary>The ellipse whose major axis is MajorAxis; Sense gives the orientation (true: counterclockwise).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius or MinorRadius &lt; 0.</exception>
    public Geom2d_Ellipse(in Axis2D MajorAxis, double MajorRadius, double MinorRadius, bool Sense = true)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < MinorRadius || MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = new Frame2D(MajorAxis, Sense);
    }

    /// <summary>The ellipse in the coordinate system Axis, whose X axis is the major axis.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius or MinorRadius &lt; 0.</exception>
    public Geom2d_Ellipse(in Frame2D Axis, double MajorRadius, double MinorRadius)
    {
        majorRadius = MajorRadius;
        minorRadius = MinorRadius;
        if (MajorRadius < MinorRadius || MinorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = Axis;
    }

    internal void SetElips2d(in Ellipse2D E)
    {
        CheckMutable();
        majorRadius = E.MajorRadius();
        minorRadius = E.MinorRadius();
        pos = E.Axis();
    }

    internal void SetMajorRadius(double MajorRadius)
    {
        CheckMutable();
        if (MajorRadius < minorRadius)
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
        if (MinorRadius < 0 || majorRadius < MinorRadius)
        {
            throw new ConstructionErrorException();
        }
        else
        {
            minorRadius = MinorRadius;
        }
    }

    /// <summary>The ellipse as a gp_Elips2d.</summary>
    public Ellipse2D Elips2d() => new Ellipse2D(pos, majorRadius, minorRadius);

    /// <summary>2*PI - U.</summary>
    public sealed override double ReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>The first directrix: parallel to the Y axis at distance MajorRadius/e from the centre, on the positive X side.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0 (a circle).</exception>
    public Axis2D Directrix1()
    {
        Ellipse2D Ev = new Ellipse2D(pos, majorRadius, minorRadius);
        return Ev.Directrix1();
    }

    /// <summary>The second directrix, the symmetric of the first about the Y axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0 (a circle).</exception>
    public Axis2D Directrix2()
    {
        Ellipse2D Ev = new Ellipse2D(pos, majorRadius, minorRadius);
        return Ev.Directrix2();
    }

    /// <summary>sqrt(MajorRadius^2 - MinorRadius^2) / MajorRadius, or 0 if MajorRadius is 0.</summary>
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
    public Point2D Focus1()
    {
        double C = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        return new Point2D(pos.Location().X() + C * pos.XDirection().X(),
                            pos.Location().Y() + C * pos.XDirection().Y());
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point2D Focus2()
    {
        double C = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        return new Point2D(pos.Location().X() - C * pos.XDirection().X(),
                            pos.Location().Y() - C * pos.XDirection().Y());
    }

    /// <summary>The major radius.</summary>
    public double MajorRadius() => majorRadius;

    /// <summary>The minor radius.</summary>
    public double MinorRadius() => minorRadius;

    /// <summary>The parameter of the ellipse (MinorRadius^2 / MajorRadius), or 0 if MajorRadius is 0.</summary>
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

    /// <summary>0.</summary>
    public sealed override double FirstParameter() => 0.0;

    /// <summary>2*PI.</summary>
    public sealed override double LastParameter() => 2.0 * M_PI;

    /// <summary>True.</summary>
    public sealed override bool IsClosed() => true;

    /// <summary>True.</summary>
    public sealed override bool IsPeriodic() => true;

    public sealed override Point2D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.EllipseValue(U, pos, majorRadius, minorRadius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElCLib.EllipseD1(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElCLib.EllipseD2(U, pos, majorRadius, minorRadius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
        global::Kerf.Engine.Geometry.ElCLib.EllipseD3(U,
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
        return global::Kerf.Engine.Geometry.ElCLib.EllipseDN(U, pos, majorRadius, minorRadius, N);
    }

    /// <summary>Applies T to the ellipse: the radii are multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D T)
    {
        CheckMutable();
        majorRadius = majorRadius * Math.Abs(T.ScaleFactor());
        minorRadius = minorRadius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to E.</summary>
    public Geom2d_Ellipse WithElips2d(in Ellipse2D E)
    {
        Geom2d_Ellipse aCopy = (Geom2d_Ellipse)Copy();
        aCopy.SetElips2d(E);
        return aCopy;
    }

    /// <summary>A copy with the major radius MajorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius().</exception>
    public Geom2d_Ellipse WithMajorRadius(double MajorRadius)
    {
        Geom2d_Ellipse aCopy = (Geom2d_Ellipse)Copy();
        aCopy.SetMajorRadius(MajorRadius);
        return aCopy;
    }

    /// <summary>A copy with the minor radius MinorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0 or MajorRadius() &lt; MinorRadius.</exception>
    public Geom2d_Ellipse WithMinorRadius(double MinorRadius)
    {
        Geom2d_Ellipse aCopy = (Geom2d_Ellipse)Copy();
        aCopy.SetMinorRadius(MinorRadius);
        return aCopy;
    }
}
