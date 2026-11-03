// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Function.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Function.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between two surfaces along a guide line. Its variables are the parametric
/// coordinates U1, V1, U2, V2 of the section's extremities on the two surfaces.
/// </summary>
public abstract class BlendFunction : AppFunction
{
    /// <summary>Returns 4.</summary>
    public override int NbVariables()
    {
        return 4;
    }

    public override Point3D Pnt1()
    {
        return PointOnS1();
    }

    public override Point3D Pnt2()
    {
        return PointOnS2();
    }

    /// <summary>The point on the first surface, at the current guide parameter.</summary>
    public abstract Point3D PointOnS1();

    /// <summary>The point on the second surface, at the current guide parameter.</summary>
    public abstract Point3D PointOnS2();

    /// <summary>True when the tangent vectors at PointOnS1 and/or PointOnS2 cannot be computed.</summary>
    public abstract bool IsTangencyPoint { get; }

    /// <summary>The tangent vector at PointOnS1, in 3d space.</summary>
    public abstract Vector3D TangentOnS1();

    /// <summary>The tangent vector at PointOnS1, in the parametric space of the first surface.</summary>
    public abstract Vector2D Tangent2dOnS1();

    /// <summary>The tangent vector at PointOnS2, in 3d space.</summary>
    public abstract Vector3D TangentOnS2();

    /// <summary>The tangent vector at PointOnS2, in the parametric space of the second surface.</summary>
    public abstract Vector2D Tangent2dOnS2();

    /// <summary>The tangents at the beginning and the end of the section, and the surfaces' normals there.</summary>
    public abstract void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgFirst, out Vector3D tgLast,
                                 out Vector3D normFirst, out Vector3D normLast);

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
    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths,
                                 NCollection_Array1<double> d2Weigths)
    {
        return false;
    }
}
