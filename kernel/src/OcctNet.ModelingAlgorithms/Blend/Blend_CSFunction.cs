// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CSFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CSFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a blending surface between a surface and a curve along a guide line. Its variables are U, V, W:
/// the parametric coordinates of the section's extremities on the surface and on the curve.
/// </summary>
public abstract class Blend_CSFunction : Blend_AppFunction
{
    /// <summary>Returns 3 (the default).</summary>
    public override int NbVariables()
    {
        return 3;
    }

    /// <summary>Not implemented: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new Standard_NotImplemented("Blend_CSFunction::GetMinimalDistance");
    }

    /// <summary>The point on the curve.</summary>
    public override gp_Pnt Pnt1()
    {
        return PointOnC();
    }

    /// <summary>The point on the surface.</summary>
    public override gp_Pnt Pnt2()
    {
        return PointOnS();
    }

    /// <summary>The point on the surface.</summary>
    public abstract gp_Pnt PointOnS();

    /// <summary>The point on the curve.</summary>
    public abstract gp_Pnt PointOnC();

    /// <summary>The U, V coordinates of the point on the surface.</summary>
    public abstract gp_Pnt2d Pnt2d();

    /// <summary>The parameter of the point on the curve.</summary>
    public abstract double ParameterOnC();

    /// <summary>True when the tangent vectors at PointOnS and/or PointOnC cannot be computed.</summary>
    public abstract bool IsTangencyPoint();

    /// <summary>The tangent vector at PointOnS, in 3d space.</summary>
    public abstract gp_Vec TangentOnS();

    /// <summary>The tangent vector at PointOnS, in the parametric space of the surface.</summary>
    public abstract gp_Vec2d Tangent2d();

    /// <summary>The tangent vector at PointOnC, in 3d space.</summary>
    public abstract gp_Vec TangentOnC();

    /// <summary>The tangent of the section at its point on the surface, and the surface's normal there.</summary>
    public abstract void Tangent(double U, double V, out gp_Vec TgS, out gp_Vec NormS);

    /// <summary>Not computed by default: returns false.</summary>
    public override bool Section(in Blend_Point P, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<gp_Vec> DPoles,
                                 NCollection_Array1<gp_Vec> D2Poles, NCollection_Array1<gp_Pnt2d> Poles2d,
                                 NCollection_Array1<gp_Vec2d> DPoles2d, NCollection_Array1<gp_Vec2d> D2Poles2d,
                                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths,
                                 NCollection_Array1<double> D2Weigths)
    {
        return false;
    }
}
