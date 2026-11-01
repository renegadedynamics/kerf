// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfRstFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfRstFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a blending surface between a surface and a restriction (a curve on another surface) along a
/// guide line. Its variables are U, V on the surface and the parameter on the restriction.
/// </summary>
public abstract class Blend_SurfRstFunction : Blend_AppFunction
{
    /// <summary>Not implemented: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new Standard_NotImplemented("Blend_SurfRstFunction::GetMinimalDistance");
    }

    public override gp_Pnt Pnt1()
    {
        return PointOnS();
    }

    public override gp_Pnt Pnt2()
    {
        return PointOnRst();
    }

    /// <summary>The point on the surface.</summary>
    public abstract gp_Pnt PointOnS();

    /// <summary>The point on the restriction.</summary>
    public abstract gp_Pnt PointOnRst();

    /// <summary>The U, V coordinates of the point on the surface.</summary>
    public abstract gp_Pnt2d Pnt2dOnS();

    /// <summary>The U, V coordinates of the point on the restriction, on its surface.</summary>
    public abstract gp_Pnt2d Pnt2dOnRst();

    /// <summary>The parameter of the point on the restriction.</summary>
    public abstract double ParameterOnRst();

    /// <summary>True when the tangent vectors cannot be computed.</summary>
    public abstract bool IsTangencyPoint();

    public abstract gp_Vec TangentOnS();

    public abstract gp_Vec2d Tangent2dOnS();

    public abstract gp_Vec TangentOnRst();

    public abstract gp_Vec2d Tangent2dOnRst();

    /// <summary>Whether the solution <paramref name="Sol"/> leaves the restriction's surface.</summary>
    public abstract bool Decroch(math_Vector Sol, out gp_Vec NS, out gp_Vec TgS);
}
