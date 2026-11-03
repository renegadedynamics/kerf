// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurveUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The computations shared by the curve local-property classes (OCCT's namespace <c>LProp_CurveUtils</c>): curvature,
/// normal and centre of curvature from the derivatives, in 3D and 2D.
/// </summary>
/// <remarks>
/// OCCT's templates over the point, vector and curve types are ported per instantiation (rules, section 9): these
/// helpers as 3D and 2D overloads; the access policies (<c>DirectAccess</c>, <c>ToolAccess</c>), the derivative
/// evaluation and the wrappers that take the properties object are written out in each instantiation
/// (<see cref="CLProps"/>, <see cref="CLProps2d"/>, <see cref="CLProps3d"/>), where they
/// read the fields that OCCT passes by reference.
/// </remarks>
internal static class CurveUtils
{
    /// <summary>|D1 x D2| / |D1|^3, or 0 if the derivatives are collinear or D2 is null.</summary>
    internal static double ComputeCurvature(in Vector3D d1, in Vector3D d2, double tolSq)
    {
        double dd1 = d1.SquareMagnitude();
        double dd2 = d2.SquareMagnitude();

        if (dd2 <= tolSq)
        {
            return 0.0;
        }

        double n = d1.CrossSquareMagnitude(d2);
        double t = n / dd1 / dd2;
        if (t <= tolSq)
        {
            return 0.0;
        }

        return Math.Sqrt(n) / dd1 / Math.Sqrt(dd1);
    }

    /// <inheritdoc cref="ComputeCurvature(in Vector3D, in Vector3D, double)"/>
    internal static double ComputeCurvature(in Vector2D d1, in Vector2D d2, double tolSq)
    {
        double dd1 = d1.SquareMagnitude();
        double dd2 = d2.SquareMagnitude();

        if (dd2 <= tolSq)
        {
            return 0.0;
        }

        double n = d1.CrossSquareMagnitude(d2);
        double t = n / dd1 / dd2;
        if (t <= tolSq)
        {
            return 0.0;
        }

        return Math.Sqrt(n) / dd1 / Math.Sqrt(dd1);
    }

    /// <summary>The normal direction D2*(D1.D1) - D1*(D1.D2) (the vector triple product).</summary>
    internal static void ComputeNormal(in Vector3D d1, in Vector3D d2, out Direction3D dir)
    {
        Vector3D norm = d2 * (d1 * d1) - d1 * (d1 * d2);
        dir = new Direction3D(norm);
    }

    /// <inheritdoc cref="ComputeNormal(in Vector3D, in Vector3D, out Direction3D)"/>
    internal static void ComputeNormal(in Vector2D d1, in Vector2D d2, out Direction2D dir)
    {
        Vector2D norm = d2 * (d1 * d1) - d1 * (d1 * d2);
        dir = new Direction2D(norm);
    }

    /// <summary>The centre of curvature: the point plus the unit normal divided by the curvature.</summary>
    internal static void ComputeCentreOfCurvature(in Point3D pnt, in Vector3D d1, in Vector3D d2, double curvature,
                                                  out Point3D centre)
    {
        Vector3D norm = d2 * (d1 * d1) - d1 * (d1 * d2);
        norm = norm.Normalized();
        norm = norm.Divided(curvature);
        centre = pnt.Translated(norm);
    }

    /// <inheritdoc cref="ComputeCentreOfCurvature(in Point3D, in Vector3D, in Vector3D, double, out Point3D)"/>
    internal static void ComputeCentreOfCurvature(in Point2D pnt, in Vector2D d1, in Vector2D d2,
                                                  double curvature, out Point2D centre)
    {
        Vector2D norm = d2 * (d1 * d1) - d1 * (d1 * d2);
        norm = norm.Normalized();
        norm = norm.Divided(curvature);
        centre = pnt.Translated(norm);
    }
}
