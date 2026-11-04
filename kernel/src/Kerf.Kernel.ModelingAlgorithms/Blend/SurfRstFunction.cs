// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfRstFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfRstFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between a surface and a restriction (a curve on another surface) along a
/// guide line. Its variables are U, V on the surface and the parameter on the restriction.
/// </summary>
internal abstract class SurfRstFunction : AppFunction
{
    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new MethodNotImplementedException("Blend_SurfRstFunction::GetMinimalDistance");
    }

    public override Point3D Pnt1()
    {
        return PointOnS();
    }

    public override Point3D Pnt2()
    {
        return PointOnRst();
    }

    /// <summary>The point on the surface.</summary>
    public abstract Point3D PointOnS();

    /// <summary>The point on the restriction.</summary>
    public abstract Point3D PointOnRst();

    /// <summary>The U, V coordinates of the point on the surface.</summary>
    public abstract Point2D Pnt2dOnS();

    /// <summary>The U, V coordinates of the point on the restriction, on its surface.</summary>
    public abstract Point2D Pnt2dOnRst();

    /// <summary>The parameter of the point on the restriction.</summary>
    public abstract double ParameterOnRst();

    /// <summary>True when the tangent vectors cannot be computed.</summary>
    public abstract bool IsTangencyPoint { get; }

    public abstract Vector3D TangentOnS();

    public abstract Vector2D Tangent2dOnS();

    public abstract Vector3D TangentOnRst();

    public abstract Vector2D Tangent2dOnRst();

    /// <summary>Whether the solution <paramref name="sol"/> leaves the restriction's surface.</summary>
    public abstract bool Decroch(math_Vector sol, out Vector3D ns, out Vector3D tgS);
}
