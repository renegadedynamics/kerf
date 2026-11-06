// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_RstRstFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_RstRstFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between two restrictions (curves on surfaces) along a guide line. Its
/// variables are the parameters on the two restrictions.
/// </summary>
internal abstract class RstRstFunction : AppFunction
{
    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new MethodNotImplementedException("Blend_RstRstFunction::GetMinimalDistance");
    }

    public override Point3D Pnt1()
    {
        return PointOnRst1();
    }

    public override Point3D Pnt2()
    {
        return PointOnRst2();
    }

    /// <summary>The point on the first restriction.</summary>
    public abstract Point3D PointOnRst1();

    /// <summary>The point on the second restriction.</summary>
    public abstract Point3D PointOnRst2();

    /// <summary>The U, V coordinates of the point on the first restriction, on its surface.</summary>
    public abstract Point2D Pnt2dOnRst1();

    /// <summary>The U, V coordinates of the point on the second restriction, on its surface.</summary>
    public abstract Point2D Pnt2dOnRst2();

    /// <summary>The parameter of the point on the first restriction.</summary>
    public abstract double ParameterOnRst1();

    /// <summary>The parameter of the point on the second restriction.</summary>
    public abstract double ParameterOnRst2();

    /// <summary>True when the tangent vectors cannot be computed.</summary>
    public abstract bool IsTangencyPoint { get; }

    public abstract Vector3D TangentOnRst1();

    public abstract Vector2D Tangent2dOnRst1();

    public abstract Vector3D TangentOnRst2();

    public abstract Vector2D Tangent2dOnRst2();

    /// <summary>Whether the solution <paramref name="sol"/> leaves the restrictions' surfaces.</summary>
    public abstract DecrochStatus Decroch(math_Vector sol, out Vector3D nRst1, out Vector3D tgRst1,
                                                out Vector3D nRst2, out Vector3D tgRst2);
}
