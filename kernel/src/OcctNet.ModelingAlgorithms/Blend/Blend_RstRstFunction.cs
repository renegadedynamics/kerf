// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_RstRstFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_RstRstFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a blending surface between two restrictions (curves on surfaces) along a guide line. Its
/// variables are the parameters on the two restrictions.
/// </summary>
public abstract class Blend_RstRstFunction : Blend_AppFunction
{
    /// <summary>Not implemented: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new Standard_NotImplemented("Blend_RstRstFunction::GetMinimalDistance");
    }

    public override gp_Pnt Pnt1()
    {
        return PointOnRst1();
    }

    public override gp_Pnt Pnt2()
    {
        return PointOnRst2();
    }

    /// <summary>The point on the first restriction.</summary>
    public abstract gp_Pnt PointOnRst1();

    /// <summary>The point on the second restriction.</summary>
    public abstract gp_Pnt PointOnRst2();

    /// <summary>The U, V coordinates of the point on the first restriction, on its surface.</summary>
    public abstract gp_Pnt2d Pnt2dOnRst1();

    /// <summary>The U, V coordinates of the point on the second restriction, on its surface.</summary>
    public abstract gp_Pnt2d Pnt2dOnRst2();

    /// <summary>The parameter of the point on the first restriction.</summary>
    public abstract double ParameterOnRst1();

    /// <summary>The parameter of the point on the second restriction.</summary>
    public abstract double ParameterOnRst2();

    /// <summary>True when the tangent vectors cannot be computed.</summary>
    public abstract bool IsTangencyPoint();

    public abstract gp_Vec TangentOnRst1();

    public abstract gp_Vec2d Tangent2dOnRst1();

    public abstract gp_Vec TangentOnRst2();

    public abstract gp_Vec2d Tangent2dOnRst2();

    /// <summary>Whether the solution <paramref name="Sol"/> leaves the restrictions' surfaces.</summary>
    public abstract Blend_DecrochStatus Decroch(math_Vector Sol, out gp_Vec NRst1, out gp_Vec TgRst1,
                                                out gp_Vec NRst2, out gp_Vec TgRst2);
}
