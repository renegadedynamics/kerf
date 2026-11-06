// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CSFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CSFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between a surface and a curve along a guide line. Its variables are U, V, W:
/// the parametric coordinates of the section's extremities on the surface and on the curve.
/// </summary>
internal abstract class CSFunction : AppFunction
{
    /// <summary>Returns 3 (the default).</summary>
    public override int NbVariables()
    {
        return 3;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetMinimalDistance()
    {
        throw new MethodNotImplementedException("Blend_CSFunction::GetMinimalDistance");
    }

    /// <summary>The point on the curve.</summary>
    public override Point3D Pnt1()
    {
        return PointOnC();
    }

    /// <summary>The point on the surface.</summary>
    public override Point3D Pnt2()
    {
        return PointOnS();
    }

    /// <summary>The point on the surface.</summary>
    public abstract Point3D PointOnS();

    /// <summary>The point on the curve.</summary>
    public abstract Point3D PointOnC();

    /// <summary>The U, V coordinates of the point on the surface.</summary>
    public abstract Point2D Pnt2d();

    /// <summary>The parameter of the point on the curve.</summary>
    public abstract double ParameterOnC();

    /// <summary>True when the tangent vectors at PointOnS and/or PointOnC cannot be computed.</summary>
    public abstract bool IsTangencyPoint { get; }

    /// <summary>The tangent vector at PointOnS, in 3d space.</summary>
    public abstract Vector3D TangentOnS();

    /// <summary>The tangent vector at PointOnS, in the parametric space of the surface.</summary>
    public abstract Vector2D Tangent2d();

    /// <summary>The tangent vector at PointOnC, in 3d space.</summary>
    public abstract Vector3D TangentOnC();

    /// <summary>The tangent of the section at its point on the surface, and the surface's normal there.</summary>
    public abstract void Tangent(double u, double v, out Vector3D tgS, out Vector3D normS);

    /// <summary>Not computed by default: returns false.</summary>
    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths,
                                 NCollection_Array1<double> d2Weigths)
    {
        return false;
    }
}
