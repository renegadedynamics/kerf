// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_Units.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_Units.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Conversions of 2D curves (pcurves) between angle units (radians and degrees) and length units, according to
/// the parameterisation of the surface they lie on. Each function returns a new curve; the input is not modified.
/// </summary>
internal static class Units
{
    /// <summary>
    /// The pcurve theCurve2d on theSurf converted from radians to degrees (theFactorRadianDegree, for example
    /// 180 / Pi) on the angular parameters and scaled by 1 / theLengthFactor on the linear ones. Circles and
    /// ellipses on a periodic surface become B-splines; types that can't be transformed are returned as copies.
    /// </summary>
    public static Curve2D RadianToDegree(Curve2D curve2d,
                                              Surface surf,
                                              double lengthFactor,
                                              double factorRadianDegree)
    {
        Curve2D aCurve2d = (Curve2D)curve2d.Copy();
        double uFact = 1.0;
        double vFact = 1.0;
        double lengthFact = 1.0 / lengthFactor;
        double angleFact = factorRadianDegree; // 180./PI;  pilotable

        var pt1 = new Point2D();
        Xy pXY;
        var tMatu = new GeneralTransform2D();
        var tMatv = new GeneralTransform2D();

        //  theSurf is a CylindricalSurface or a ConicalSurface or
        //             a ToroidalSurface or a SphericalSurface or
        //             a SurfaceOfRevolution
        if (surf is SphericalSurface
            || surf is ToroidalSurface)
        {
            uFact = vFact = angleFact;
        }
        else if (surf is CylindricalSurface)
        {
            uFact = angleFact;
            vFact = lengthFact;
        }
        else if (surf is RevolutionSurface)
        {
            uFact = angleFact;
        }
        else if (surf is ConicalSurface)
        {
            ConicalSurface conicS = (ConicalSurface)surf;
            double semAng = conicS.SemiAngle;
            uFact = angleFact;
            vFact = lengthFact * Math.Cos(semAng);
        }
        else if (surf is PlaneSurface)
        {
            uFact = vFact = lengthFact;
            if (aCurve2d is CircleCurve2D
                || aCurve2d is EllipseCurve2D)
            {
                var t = new Transform2D();
                t = t.WithScale(Constants.Origin2d(), lengthFact);
                aCurve2d.Transform(t);
                return aCurve2d;
            }
        }
        else
        {
            return aCurve2d;
        }

        if (aCurve2d is LineCurve2D)
        {
            LineCurve2D line2d = (LineCurve2D)aCurve2d;
            Point2D myLoc = line2d.Location;
            Direction2D myDir = line2d.Direction;
            var myNewLoc = new Point2D();
            myNewLoc = myNewLoc.WithCoord(myLoc.X * uFact, myLoc.Y * vFact);
            var myNewDir = new Direction2D();
            myNewDir = myNewDir.WithCoord(myDir.X * uFact, myDir.Y * vFact);
            LineCurve2D myNewLine2d = (LineCurve2D)line2d.Copy();
            myNewLine2d.SetLocation(myNewLoc);
            myNewLine2d.SetDirection(myNewDir);
            return myNewLine2d;
        }
        else if (aCurve2d is ConicCurve2D)
        {
            if (aCurve2d is CircleCurve2D
                || aCurve2d is EllipseCurve2D)
            {
                BSplineCurve2D bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(aCurve2d);
                aCurve2d = bSpline2d;
            }

            // Parabolas and hyperbolas are left as they are (OCCT only reports them in debug builds)
        }

        // Compute affinity
        tMatu = tMatu.WithAffinity(Constants.OY2d(), uFact);
        tMatv = tMatv.WithAffinity(Constants.OX2d(), vFact);
        if (aCurve2d is BoundedCurve2D)
        {
            if (aCurve2d is BSplineCurve2D)
            {
                BSplineCurve2D bSpline2d = (BSplineCurve2D)aCurve2d;
                BSplineCurve2D myNewBSpline2d =
                    (BSplineCurve2D)bSpline2d.Copy();
                int nbPol = bSpline2d.PoleCount;
                for (int i = 1; i <= nbPol; i++)
                {
                    pXY = bSpline2d.Pole(i).XY();
                    tMatu.Transforms(ref pXY);
                    tMatv.Transforms(ref pXY);
                    pt1 = pt1.WithXY(pXY);
                    myNewBSpline2d.SetPole(i, pt1);
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
    public static Curve2D DegreeToRadian(Curve2D pcurve,
                                              Surface surface,
                                              double lengthFactor,
                                              double factorRadianDegree)
    {
        Curve2D aPcurve = (Curve2D)pcurve.Copy();
        double uFact = 1.0;
        double vFact = 1.0;
        double lengthFact = lengthFactor;
        double angleFact = factorRadianDegree; // PI/180.;  pilotable

        var pt1 = new Point2D();
        Xy pXY;
        var tMatu = new GeneralTransform2D();
        var tMatv = new GeneralTransform2D();

        // What to change ??

        if (surface is SphericalSurface
            || surface is ToroidalSurface)
        {
            uFact = vFact = angleFact;
        }
        else if (surface is CylindricalSurface)
        {
            uFact = angleFact;
            vFact = lengthFact;
        }
        else if (surface is RevolutionSurface)
        {
            uFact = angleFact;
        }
        else if (surface is ConicalSurface)
        {
            ConicalSurface conicS = (ConicalSurface)surface;
            double semAng = conicS.SemiAngle;
            uFact = angleFact;
            vFact = lengthFact / Math.Cos(semAng);
        }
        else if (surface is PlaneSurface)
        {
            uFact = vFact = lengthFact;
            if (aPcurve is CircleCurve2D
                || aPcurve is EllipseCurve2D)
            {
                var t = new Transform2D();
                t = t.WithScale(Constants.Origin2d(), lengthFact);
                aPcurve.Transform(t);
                return aPcurve;
            }
        }
        else
        {
            return aPcurve;
        }

        if (aPcurve is ConicCurve2D)
        {
            if (aPcurve is CircleCurve2D
                || aPcurve is EllipseCurve2D)
            {
                BSplineCurve2D bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(aPcurve);
                aPcurve = bSpline2d;
            }
            else if (aPcurve is ParabolaCurve2D)
            {
                // Parameters not yet transformed according to LengthUnit
                return aPcurve;
            }
            else if (aPcurve is HyperbolaCurve2D)
            {
                // Parameters not yet transformed according to LengthUnit
                return aPcurve;
            }
        }

        // Compute affinity

        tMatu = tMatu.WithAffinity(Constants.OY2d(), uFact);
        tMatv = tMatv.WithAffinity(Constants.OX2d(), vFact);

        if (aPcurve is LineCurve2D)
        {
            LineCurve2D line2d = (LineCurve2D)aPcurve;

            Point2D myLoc = line2d.Location;
            Direction2D myDir = line2d.Direction;

            var myNewLoc = new Point2D();
            myNewLoc = myNewLoc.WithCoord(myLoc.X * uFact, myLoc.Y * vFact);

            var myNewDir = new Direction2D();
            myNewDir = myNewDir.WithCoord(myDir.X * uFact, myDir.Y * vFact);

            line2d.SetLocation(myNewLoc);
            line2d.SetDirection(myNewDir);

            aPcurve = line2d;
        }
        else if (aPcurve is BSplineCurve2D)
        {
            BSplineCurve2D bSpline2d = (BSplineCurve2D)aPcurve;

            // transform the Poles of the BSplineCurve according to AngleFact and LengthFact

            int nbPol = bSpline2d.PoleCount;
            for (int i = 1; i <= nbPol; i++)
            {
                pXY = bSpline2d.Pole(i).XY();
                tMatu.Transforms(ref pXY);
                tMatv.Transforms(ref pXY);
                pt1 = pt1.WithXY(pXY);
                bSpline2d.SetPole(i, pt1);
            }

            aPcurve = bSpline2d;
        }

        // Other types are not yet implemented (OCCT only reports them in debug builds)
        return aPcurve;
    }

    /// <summary>A copy of theCurve mirrored about the U axis (V becomes -V).</summary>
    public static Curve2D MirrorPCurve(Curve2D curve)
    {
        Curve2D theMirrored = (Curve2D)curve.Copy();
        var t = new Transform2D();
        var loc = new Point2D(0.0, 0.0);
        var dir = new Direction2D(Direction2D.D.X);
        var ax2 = new Axis2D(loc, dir);
        t = t.WithMirror(ax2);
        theMirrored.Transform(t);
        return theMirrored;
    }
}
