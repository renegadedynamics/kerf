// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurveUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    internal static double ComputeCurvature(in gp_Vec theD1, in gp_Vec theD2, double theTolSq)
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

    /// <inheritdoc cref="ComputeCurvature(in gp_Vec, in gp_Vec, double)"/>
    internal static double ComputeCurvature(in gp_Vec2d theD1, in gp_Vec2d theD2, double theTolSq)
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
    internal static void ComputeNormal(in gp_Vec theD1, in gp_Vec theD2, out gp_Dir theDir)
    {
        gp_Vec aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        theDir = new gp_Dir(aNorm);
    }

    /// <inheritdoc cref="ComputeNormal(in gp_Vec, in gp_Vec, out gp_Dir)"/>
    internal static void ComputeNormal(in gp_Vec2d theD1, in gp_Vec2d theD2, out gp_Dir2d theDir)
    {
        gp_Vec2d aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        theDir = new gp_Dir2d(aNorm);
    }

    /// <summary>The centre of curvature: the point plus the unit normal divided by the curvature.</summary>
    internal static void ComputeCentreOfCurvature(in gp_Pnt thePnt, in gp_Vec theD1, in gp_Vec theD2, double theCurvature,
                                                  out gp_Pnt theCentre)
    {
        gp_Vec aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        aNorm.Normalize();
        aNorm.Divide(theCurvature);
        theCentre = thePnt.Translated(aNorm);
    }

    /// <inheritdoc cref="ComputeCentreOfCurvature(in gp_Pnt, in gp_Vec, in gp_Vec, double, out gp_Pnt)"/>
    internal static void ComputeCentreOfCurvature(in gp_Pnt2d thePnt, in gp_Vec2d theD1, in gp_Vec2d theD2,
                                                  double theCurvature, out gp_Pnt2d theCentre)
    {
        gp_Vec2d aNorm = theD2 * (theD1 * theD1) - theD1 * (theD1 * theD2);
        aNorm.Normalize();
        aNorm.Divide(theCurvature);
        theCentre = thePnt.Translated(aNorm);
    }
}
