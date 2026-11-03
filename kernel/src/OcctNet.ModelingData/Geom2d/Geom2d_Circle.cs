// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Circle.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A circle in the plane, placed by a local coordinate system whose origin is the centre. The parameter is the
/// angle from the X axis, in [0, 2*PI[, in the sense of the coordinate system: P(U) = O + R*cos(U)*XDir +
/// R*sin(U)*YDir. The curve is closed and periodic, of period 2*PI.
/// </summary>
public class Geom2d_Circle : Geom2d_Conic
{
    private double radius;

    /// <summary>A new, unfrozen circle equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_Circle C;
        C = new Geom2d_Circle(pos, radius);
        return C;
    }

    /// <summary>The circle equal to C.</summary>
    public Geom2d_Circle(in Circle2D C)
    {
        radius = C.Radius();
        pos = C.Axis();
    }

    /// <summary>
    /// The circle of centre A's origin and radius Radius; A is the X axis, and Sense the orientation (true:
    /// counterclockwise).
    /// </summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public Geom2d_Circle(in Axis2D A, double Radius, bool Sense = true)
    {
        radius = Radius;
        if (Radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = new Frame2D(A, Sense);
    }

    /// <summary>The circle of radius Radius in the coordinate system A.</summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public Geom2d_Circle(in Frame2D A, double Radius)
    {
        radius = Radius;
        if (Radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = A;
    }

    internal void SetCirc2d(in Circle2D C)
    {
        CheckMutable();
        radius = C.Radius();
        pos = C.Axis();
    }

    internal void SetRadius(double R)
    {
        CheckMutable();
        if (R < 0.0)
        {
            throw new ConstructionErrorException();
        }
        radius = R;
    }

    /// <summary>The radius.</summary>
    public double Radius() => radius;

    /// <summary>The circle as a gp_Circ2d.</summary>
    public Circle2D Circ2d() => new Circle2D(pos, radius);

    /// <summary>2*PI - U.</summary>
    public sealed override double ReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>0.</summary>
    public sealed override double Eccentricity() => 0.0;

    /// <summary>0.</summary>
    public sealed override double FirstParameter() => 0.0;

    /// <summary>2*PI.</summary>
    public sealed override double LastParameter() => 2.0 * M_PI;

    /// <summary>True.</summary>
    public sealed override bool IsClosed() => true;

    /// <summary>True.</summary>
    public sealed override bool IsPeriodic() => true;

    public sealed override Point2D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.CircleValue(U, pos, radius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElCLib.CircleD1(U, pos, radius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElCLib.CircleD2(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
        global::Kerf.Engine.Geometry.ElCLib.CircleD3(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector2D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        return global::Kerf.Engine.Geometry.ElCLib.CircleDN(U, pos, radius, N);
    }

    /// <summary>Applies T to the circle: the radius is multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to C.</summary>
    public Geom2d_Circle WithCirc2d(in Circle2D C)
    {
        Geom2d_Circle aCopy = (Geom2d_Circle)Copy();
        aCopy.SetCirc2d(C);
        return aCopy;
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public Geom2d_Circle WithRadius(double R)
    {
        Geom2d_Circle aCopy = (Geom2d_Circle)Copy();
        aCopy.SetRadius(R);
        return aCopy;
    }
}
