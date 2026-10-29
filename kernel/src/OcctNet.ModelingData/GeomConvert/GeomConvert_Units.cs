// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_Units.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_Units.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Conversions of 2D curves (pcurves) between angle units (radians and degrees) and length units, according to
/// the parameterisation of the surface they lie on. Each function returns a new curve; the input is not modified.
/// </summary>
public static class GeomConvert_Units
{
    // TODO(4C merge): Geom2dConvert is ported by another part of unit 4C. Replace the body with OCCT's
    // `return Geom2dConvert.CurveToBSplineCurve(theCurve);` once it is merged.
    private static Geom2d_BSplineCurve CurveToBSplineCurve2d(Geom2d_Curve theCurve)
        => throw new Standard_NotImplemented("GeomConvert_Units: Geom2dConvert::CurveToBSplineCurve not merged yet");

    /// <summary>
    /// The pcurve theCurve2d on theSurf converted from radians to degrees (theFactorRadianDegree, for example
    /// 180 / Pi) on the angular parameters and scaled by 1 / theLengthFactor on the linear ones. Circles and
    /// ellipses on a periodic surface become B-splines; types that can't be transformed are returned as copies.
    /// </summary>
    public static Geom2d_Curve RadianToDegree(Geom2d_Curve theCurve2d,
                                              Geom_Surface theSurf,
                                              double theLengthFactor,
                                              double theFactorRadianDegree)
    {
        Geom2d_Curve aCurve2d = (Geom2d_Curve)theCurve2d.Copy();
        double uFact = 1.0;
        double vFact = 1.0;
        double LengthFact = 1.0 / theLengthFactor;
        double AngleFact = theFactorRadianDegree; // 180./PI;  pilotable

        var Pt1 = new gp_Pnt2d();
        gp_XY pXY;
        var tMatu = new gp_GTrsf2d();
        var tMatv = new gp_GTrsf2d();

        //  theSurf is a CylindricalSurface or a ConicalSurface or
        //             a ToroidalSurface or a SphericalSurface or
        //             a SurfaceOfRevolution
        if (theSurf is Geom_SphericalSurface
            || theSurf is Geom_ToroidalSurface)
        {
            uFact = vFact = AngleFact;
        }
        else if (theSurf is Geom_CylindricalSurface)
        {
            uFact = AngleFact;
            vFact = LengthFact;
        }
        else if (theSurf is Geom_SurfaceOfRevolution)
        {
            uFact = AngleFact;
        }
        else if (theSurf is Geom_ConicalSurface)
        {
            Geom_ConicalSurface conicS = (Geom_ConicalSurface)theSurf;
            double semAng = conicS.SemiAngle();
            uFact = AngleFact;
            vFact = LengthFact * Math.Cos(semAng);
        }
        else if (theSurf is Geom_Plane)
        {
            uFact = vFact = LengthFact;
            if (aCurve2d is Geom2d_Circle
                || aCurve2d is Geom2d_Ellipse)
            {
                var aT = new gp_Trsf2d();
                aT.SetScale(gp.Origin2d(), LengthFact);
                aCurve2d.Transform(aT);
                return aCurve2d;
            }
        }
        else
        {
            return aCurve2d;
        }

        if (aCurve2d is Geom2d_Line)
        {
            Geom2d_Line aLine2d = (Geom2d_Line)aCurve2d;
            gp_Pnt2d myLoc = aLine2d.Location();
            gp_Dir2d myDir = aLine2d.Direction();
            var myNewLoc = new gp_Pnt2d();
            myNewLoc.SetCoord(myLoc.X() * uFact, myLoc.Y() * vFact);
            var myNewDir = new gp_Dir2d();
            myNewDir.SetCoord(myDir.X() * uFact, myDir.Y() * vFact);
            Geom2d_Line myNewLine2d = (Geom2d_Line)aLine2d.Copy();
            myNewLine2d.SetLocation(myNewLoc);
            myNewLine2d.SetDirection(myNewDir);
            return myNewLine2d;
        }
        else if (aCurve2d is Geom2d_Conic)
        {
            if (aCurve2d is Geom2d_Circle
                || aCurve2d is Geom2d_Ellipse)
            {
                Geom2d_BSplineCurve aBSpline2d = CurveToBSplineCurve2d(aCurve2d);
                aCurve2d = aBSpline2d;
            }

            // Parabolas and hyperbolas are left as they are (OCCT only reports them in debug builds)
        }

        // Compute affinity
        tMatu.SetAffinity(gp.OY2d(), uFact);
        tMatv.SetAffinity(gp.OX2d(), vFact);
        if (aCurve2d is Geom2d_BoundedCurve)
        {
            if (aCurve2d is Geom2d_BSplineCurve)
            {
                Geom2d_BSplineCurve aBSpline2d = (Geom2d_BSplineCurve)aCurve2d;
                Geom2d_BSplineCurve myNewBSpline2d =
                    (Geom2d_BSplineCurve)aBSpline2d.Copy();
                int nbPol = aBSpline2d.NbPoles();
                for (int i = 1; i <= nbPol; i++)
                {
                    pXY = aBSpline2d.Pole(i).XY();
                    tMatu.Transforms(ref pXY);
                    tMatv.Transforms(ref pXY);
                    Pt1.SetXY(pXY);
                    myNewBSpline2d.SetPole(i, Pt1);
                }

                return myNewBSpline2d;
            }

            // Other bounded curves are left as they are (OCCT only reports them in debug builds)
        }

        return aCurve2d;
    }

    /// <summary>
    /// The pcurve thePcurve on theSurface converted from degrees to radians (theFactorRadianDegree, for example
    /// Pi / 180) on the angular parameters and scaled by theLengthFactor on the linear ones (STEP's cylinders,
    /// tori, cones and spheres are parameterised in degrees). Circles and ellipses on a periodic surface become
    /// B-splines; types that can't be transformed are returned as copies.
    /// </summary>
    public static Geom2d_Curve DegreeToRadian(Geom2d_Curve thePcurve,
                                              Geom_Surface theSurface,
                                              double theLengthFactor,
                                              double theFactorRadianDegree)
    {
        Geom2d_Curve aPcurve = (Geom2d_Curve)thePcurve.Copy();
        double uFact = 1.0;
        double vFact = 1.0;
        double LengthFact = theLengthFactor;
        double AngleFact = theFactorRadianDegree; // PI/180.;  pilotable

        var Pt1 = new gp_Pnt2d();
        gp_XY pXY;
        var tMatu = new gp_GTrsf2d();
        var tMatv = new gp_GTrsf2d();

        // What to change ??

        if (theSurface is Geom_SphericalSurface
            || theSurface is Geom_ToroidalSurface)
        {
            uFact = vFact = AngleFact;
        }
        else if (theSurface is Geom_CylindricalSurface)
        {
            uFact = AngleFact;
            vFact = LengthFact;
        }
        else if (theSurface is Geom_SurfaceOfRevolution)
        {
            uFact = AngleFact;
        }
        else if (theSurface is Geom_ConicalSurface)
        {
            Geom_ConicalSurface conicS = (Geom_ConicalSurface)theSurface;
            double semAng = conicS.SemiAngle();
            uFact = AngleFact;
            vFact = LengthFact / Math.Cos(semAng);
        }
        else if (theSurface is Geom_Plane)
        {
            uFact = vFact = LengthFact;
            if (aPcurve is Geom2d_Circle
                || aPcurve is Geom2d_Ellipse)
            {
                var aT = new gp_Trsf2d();
                aT.SetScale(gp.Origin2d(), LengthFact);
                aPcurve.Transform(aT);
                return aPcurve;
            }
        }
        else
        {
            return aPcurve;
        }

        if (aPcurve is Geom2d_Conic)
        {
            if (aPcurve is Geom2d_Circle
                || aPcurve is Geom2d_Ellipse)
            {
                Geom2d_BSplineCurve aBSpline2d = CurveToBSplineCurve2d(aPcurve);
                aPcurve = aBSpline2d;
            }
            else if (aPcurve is Geom2d_Parabola)
            {
                // Parameters not yet transformed according to LengthUnit
                return aPcurve;
            }
            else if (aPcurve is Geom2d_Hyperbola)
            {
                // Parameters not yet transformed according to LengthUnit
                return aPcurve;
            }
        }

        // Compute affinity

        tMatu.SetAffinity(gp.OY2d(), uFact);
        tMatv.SetAffinity(gp.OX2d(), vFact);

        if (aPcurve is Geom2d_Line)
        {
            Geom2d_Line aLine2d = (Geom2d_Line)aPcurve;

            gp_Pnt2d myLoc = aLine2d.Location();
            gp_Dir2d myDir = aLine2d.Direction();

            var myNewLoc = new gp_Pnt2d();
            myNewLoc.SetCoord(myLoc.X() * uFact, myLoc.Y() * vFact);

            var myNewDir = new gp_Dir2d();
            myNewDir.SetCoord(myDir.X() * uFact, myDir.Y() * vFact);

            aLine2d.SetLocation(myNewLoc);
            aLine2d.SetDirection(myNewDir);

            aPcurve = aLine2d;
        }
        else if (aPcurve is Geom2d_BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline2d = (Geom2d_BSplineCurve)aPcurve;

            // transform the Poles of the BSplineCurve according to AngleFact and LengthFact

            int nbPol = aBSpline2d.NbPoles();
            for (int i = 1; i <= nbPol; i++)
            {
                pXY = aBSpline2d.Pole(i).XY();
                tMatu.Transforms(ref pXY);
                tMatv.Transforms(ref pXY);
                Pt1.SetXY(pXY);
                aBSpline2d.SetPole(i, Pt1);
            }

            aPcurve = aBSpline2d;
        }

        // Other types are not yet implemented (OCCT only reports them in debug builds)
        return aPcurve;
    }

    /// <summary>A copy of theCurve mirrored about the U axis (V becomes -V).</summary>
    public static Geom2d_Curve MirrorPCurve(Geom2d_Curve theCurve)
    {
        Geom2d_Curve theMirrored = (Geom2d_Curve)theCurve.Copy();
        var T = new gp_Trsf2d();
        var Loc = new gp_Pnt2d(0.0, 0.0);
        var Dir = new gp_Dir2d(gp_Dir2d.D.X);
        var ax2 = new gp_Ax2d(Loc, Dir);
        T.SetMirror(ax2);
        theMirrored.Transform(T);
        return theMirrored;
    }
}
