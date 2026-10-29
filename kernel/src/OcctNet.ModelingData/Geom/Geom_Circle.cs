// Port of: src/ModelingData/TKG3d/Geom/Geom_Circle.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A circle in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its centre:
/// P(u) = O + R * cos(u) * XDir + R * sin(u) * YDir, u in [0, 2 * PI[, periodic.
/// </summary>
public class Geom_Circle : Geom_Conic
{
    private double radius;

    /// <summary>A circle equal to C.</summary>
    public Geom_Circle(in gp_Circ C)
    {
        radius = C.Radius();
        pos = C.Position();
    }

    /// <summary>A circle of radius R placed by A2.</summary>
    /// <exception cref="Standard_ConstructionError">R &lt; 0.</exception>
    public Geom_Circle(in gp_Ax2 A2, double R)
    {
        radius = R;
        if (R < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = A2;
    }

    /// <summary>A new, unfrozen circle equal to this one.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_Circle C;
        C = new Geom_Circle(pos, radius);
        return C;
    }

    public sealed override bool IsClosed() => true;

    public sealed override bool IsPeriodic() => true;

    /// <summary>2 * PI - U.</summary>
    public sealed override double ReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>0.</summary>
    public sealed override double Eccentricity() => 0.0;

    public sealed override double FirstParameter() => 0.0;

    public sealed override double LastParameter() => 2.0 * M_PI;

    /// <summary>The non-persistent circle equal to this one.</summary>
    public gp_Circ Circ() => new gp_Circ(pos, radius);

    internal void SetCirc(in gp_Circ C)
    {
        CheckMutable();
        radius = C.Radius();
        pos = C.Position();
    }

    /// <exception cref="Standard_ConstructionError">R &lt; 0.</exception>
    internal void SetRadius(double R)
    {
        CheckMutable();
        if (R < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        radius = R;
    }

    public double Radius() => radius;

    public sealed override gp_Pnt EvalD0(double U) => ElCLib.CircleValue(U, pos, radius);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.CircleD1(U, pos, radius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.CircleD2(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.CircleD3(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return ElCLib.CircleDN(U, pos, radius, N);
    }

    /// <summary>Applies T to this circle (the radius is scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Circle WithCirc(in gp_Circ C)
    {
        Geom_Circle aCopy = (Geom_Circle)Copy();
        aCopy.SetCirc(C);
        return aCopy;
    }

    public Geom_Circle WithRadius(double R)
    {
        Geom_Circle aCopy = (Geom_Circle)Copy();
        aCopy.SetRadius(R);
        return aCopy;
    }
}
