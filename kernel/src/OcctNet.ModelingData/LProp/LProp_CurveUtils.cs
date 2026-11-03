// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurveUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The computations shared by the curve local-property classes (OCCT's namespace <c>LProp_CurveUtils</c>): curvature,
/// normal and centre of curvature from the derivatives, in 3D and 2D.
/// </summary>
/// <remarks>
/// OCCT's templates over the point, vector and curve types are ported per instantiation (rules, section 9): these
/// helpers as 3D and 2D overloads; the access policies (<c>DirectAccess</c>, <c>ToolAccess</c>), the derivative
/// evaluation and the wrappers that take the properties object are written out in each instantiation
/// (<see cref="GeomLProp_CLProps"/>, <see cref="GeomLProp_CLProps2d"/>, <see cref="LProp_CLProps3d"/>), where they
/// read the fields that OCCT passes by reference.
/// </remarks>
internal static class LProp_CurveUtils
{
    /// <summary>|D1 x D2| / |D1|^3, or 0 if the derivatives are collinear or D2 is null.</summary>
    internal static double ComputeCurvature(in Vector3D theD1, in Vector3D theD2, double theTolSq)
    {
        double aDD1 = theD1.SquareMagnitude();
        double aDD2 = theD2.SquareMagnitude();

        if (aDD2 <= theTolSq)
        {
            return 0.0;
        }

        double aN = theD1.CrossSquareMagnitude(theD2);
        double aT = aN / aDD1 / aDD2;
        if (aT <= theTolSq)
        {
            return 0.0;
        }

        return Math.Sqrt(aN) / aDD1 / Math.Sqrt(aDD1);
    }

    /// <inheritdoc cref="ComputeCurvature(in Vector3D, in Vector3D, double)"/>
    internal static double ComputeCurvature(in Vector2D theD1, in Vector2D theD2, double theTolSq)
    {
        double aDD1 = theD1.SquareMagnitude();
        double aDD2 = theD2.SquareMagnitude();

        if (aDD2 <= theTolSq)
        {
            return 0.0;
        }

        double aN = theD1.CrossSquareMagnitude(theD2);
        double aT = aN / aDD1 / aDD2;
        if (aT <= theTolSq)
        {
            return 0.0;
        }

        return Math.Sqrt(aN) / aDD1 / Math.Sqrt(aDD1);
    }

    /// <summary>The normal direction D2*(D1.D1) - D1*(D1.D2) (the vector triple product).</summary>
    internal static void ComputeNormal(in Vector3D theD1, in Vector3D theD2, out Direction3D theDir)
    {
        Vector3D aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        theDir = new Direction3D(aNorm);
    }

    /// <inheritdoc cref="ComputeNormal(in Vector3D, in Vector3D, out Direction3D)"/>
    internal static void ComputeNormal(in Vector2D theD1, in Vector2D theD2, out Direction2D theDir)
    {
        Vector2D aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        theDir = new Direction2D(aNorm);
    }

    /// <summary>The centre of curvature: the point plus the unit normal divided by the curvature.</summary>
    internal static void ComputeCentreOfCurvature(in Point3D thePnt, in Vector3D theD1, in Vector3D theD2, double theCurvature,
                                                  out Point3D theCentre)
    {
        Vector3D aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        aNorm.Normalize();
        aNorm.Divide(theCurvature);
        theCentre = thePnt.Translated(aNorm);
    }

    /// <inheritdoc cref="ComputeCentreOfCurvature(in Point3D, in Vector3D, in Vector3D, double, out Point3D)"/>
    internal static void ComputeCentreOfCurvature(in Point2D thePnt, in Vector2D theD1, in Vector2D theD2,
                                                  double theCurvature, out Point2D theCentre)
    {
        Vector2D aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        aNorm.Normalize();
        aNorm.Divide(theCurvature);
        theCentre = thePnt.Translated(aNorm);
    }
}
