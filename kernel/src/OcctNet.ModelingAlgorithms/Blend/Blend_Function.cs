// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Function.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Function.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a blending surface between two surfaces along a guide line. Its variables are the parametric
/// coordinates U1, V1, U2, V2 of the section's extremities on the two surfaces.
/// </summary>
public abstract class Blend_Function : Blend_AppFunction
{
    /// <summary>Returns 4.</summary>
    public override int NbVariables()
    {
        return 4;
    }

    public override gp_Pnt Pnt1()
    {
        return PointOnS1();
    }

    public override gp_Pnt Pnt2()
    {
        return PointOnS2();
    }

    /// <summary>The point on the first surface, at the current guide parameter.</summary>
    public abstract gp_Pnt PointOnS1();

    /// <summary>The point on the second surface, at the current guide parameter.</summary>
    public abstract gp_Pnt PointOnS2();

    /// <summary>True when the tangent vectors at PointOnS1 and/or PointOnS2 cannot be computed.</summary>
    public abstract bool IsTangencyPoint();

    /// <summary>The tangent vector at PointOnS1, in 3d space.</summary>
    public abstract gp_Vec TangentOnS1();

    /// <summary>The tangent vector at PointOnS1, in the parametric space of the first surface.</summary>
    public abstract gp_Vec2d Tangent2dOnS1();

    /// <summary>The tangent vector at PointOnS2, in 3d space.</summary>
    public abstract gp_Vec TangentOnS2();

    /// <summary>The tangent vector at PointOnS2, in the parametric space of the second surface.</summary>
    public abstract gp_Vec2d Tangent2dOnS2();

    /// <summary>The tangents at the beginning and the end of the section, and the surfaces' normals there.</summary>
    public abstract void Tangent(double U1, double V1, double U2, double V2, out gp_Vec TgFirst, out gp_Vec TgLast,
                                 out gp_Vec NormFirst, out gp_Vec NormLast);

    /// <summary>False by default.</summary>
    public virtual bool TwistOnS1()
    {
        return false;
    }

    /// <summary>False by default.</summary>
    public virtual bool TwistOnS2()
    {
        return false;
    }

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
