// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_SurfToAnaSurf.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_SurfToAnaSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.GeomConvert_ConvType;
using static Occt.IntAna_ResultType;

namespace Occt;

/// <summary>
/// Converts a surface to an analytical form (plane, cylinder, cone, sphere or torus) within a tolerance. The
/// conversion is made for B-spline and Bezier surfaces, and for other surfaces that some analytical surface
/// approximates within that tolerance. The surfaces returned are new, and the caller owns them.
/// </summary>
public sealed class GeomConvert_SurfToAnaSurf
{
    private Geom_Surface? mySurf;
    private double myGap;
    private GeomConvert_ConvType myConvType;
    private GeomAbs_SurfaceType myTarget;

    public GeomConvert_SurfToAnaSurf()
    {
        myGap = -1.0;
        myConvType = GeomConvert_Simplest;
        myTarget = GeomAbs_Plane;
    }

    /// <summary>A converter for S (kept, never changed).</summary>
    public GeomConvert_SurfToAnaSurf(Geom_Surface S)
    {
        myGap = -1.0;
        myConvType = GeomConvert_Simplest;
        myTarget = GeomAbs_Plane;
        Init(S);
    }

    /// <summary>Sets the surface to convert (kept, never changed).</summary>
    public void Init(Geom_Surface S)
    {
        mySurf = S;
    }

    /// <summary>Sets how the result is chosen: the target type, the simplest type or the smallest gap.</summary>
    public void SetConvType(GeomConvert_ConvType theConvType = GeomConvert_Simplest)
    {
        myConvType = theConvType;
    }

    /// <summary>Sets the target type, for <see cref="GeomConvert_ConvType.GeomConvert_Target"/>.</summary>
    public void SetTarget(GeomAbs_SurfaceType theSurfType = GeomAbs_Plane)
    {
        myTarget = theSurfType;
    }

    /// <summary>The maximal deviation of the surface converted by the last call to ConvertToAnalytical.</summary>
    public double Gap() => myGap;

    //=================================================================================================
    private static void CheckVTrimForRevSurf(Geom_SurfaceOfRevolution aRevSurf, ref double V1, ref double V2)
    {
        Geom_Curve aBC = aRevSurf.BasisCurve();
        Geom_Line? aLine = aBC as Geom_Line;
        if (aLine is null)
        {
            return;
        }
        gp_Ax1 anAxis = aRevSurf.Axis();

        gp_Lin anALin = new(anAxis);
        Extrema_ExtElC anExtLL = new(aLine.Lin(), anALin, Precision.Angular());
        if (!anExtLL.IsDone() || anExtLL.IsParallel())
        {
            return;
        }
        int aNbExt = anExtLL.NbExt();
        if (aNbExt == 0)
        {
            return;
        }

        int i;
        int imin = 0;
        for (i = 1; i <= aNbExt; ++i)
        {
            if (anExtLL.SquareDistance(i) < Precision.SquareConfusion())
            {
                imin = i;
                break;
            }
        }
        if (imin == 0)
        {
            return;
        }

        anExtLL.Points(imin, out Extrema_POnCurv aP1, out Extrema_POnCurv _);
        double aVExt = aP1.Parameter();
        if (aVExt <= V1 || aVExt >= V2)
        {
            return;
        }

        if (aVExt - V1 > V2 - aVExt)
        {
            V2 = aVExt;
        }
        else
        {
            V1 = aVExt;
        }
    }

    //=================================================================================================
    private static Geom_Surface? TryCylinerCone(Geom_Surface theSurf,
                                                bool theVCase,
                                                Geom_Curve theUmidiso,
                                                Geom_Curve theVmidiso,
                                                double theU1,
                                                double theU2,
                                                double theV1,
                                                double theV2,
                                                double theToler)
    {
        Geom_Surface? aNewSurf = null;
        double param1, param2, cf1, cf2, cl1, cl2, aGap1 = 0.0, aGap2 = 0.0;
        Geom_Curve firstiso, lastiso;
        Geom_Circle? firstisocirc, lastisocirc, midisocirc;
        gp_Dir isoline;
        if (theVCase)
        {
            param1 = theU1;
            param2 = theU2;
            firstiso = theSurf.VIso(theV1);
            lastiso = theSurf.VIso(theV2);
            midisocirc = theVmidiso as Geom_Circle;
            isoline = ((Geom_Line)theUmidiso).Lin().Direction();
        }
        else
        {
            param1 = theV1;
            param2 = theV2;
            firstiso = theSurf.UIso(theU1);
            lastiso = theSurf.UIso(theU2);
            midisocirc = theUmidiso as Geom_Circle;
            isoline = ((Geom_Line)theVmidiso).Lin().Direction();
        }
        firstisocirc = GeomConvert_CurveToAnaCurve.ComputeCurve(firstiso,
                                                                theToler,
                                                                param1,
                                                                param2,
                                                                out cf1,
                                                                out cl1,
                                                                ref aGap1,
                                                                GeomConvert_Target,
                                                                GeomAbs_Circle) as Geom_Circle;
        lastisocirc = GeomConvert_CurveToAnaCurve.ComputeCurve(lastiso,
                                                               theToler,
                                                               param1,
                                                               param2,
                                                               out cf2,
                                                               out cl2,
                                                               ref aGap2,
                                                               GeomConvert_Target,
                                                               GeomAbs_Circle) as Geom_Circle;
        if (firstisocirc is not null || lastisocirc is not null)
        {
            double R1, R2, R3;
            gp_Pnt P1, P2, P3;
            if (firstisocirc is not null)
            {
                R1 = firstisocirc.Circ().Radius();
                P1 = firstisocirc.Circ().Location();
            }
            else
            {
                R1 = 0;
                // OCCT: half the parameter range, not its middle (occt-issues.md)
                P1 = firstiso.Value((firstiso.LastParameter() - firstiso.FirstParameter()) / 2);
            }
            R2 = midisocirc!.Circ().Radius();
            P2 = midisocirc.Circ().Location();
            if (lastisocirc is not null)
            {
                R3 = lastisocirc.Circ().Radius();
                P3 = lastisocirc.Circ().Location();
            }
            else
            {
                R3 = 0;
                // OCCT: half the parameter range, not its middle (occt-issues.md)
                P3 = lastiso.Value((lastiso.LastParameter() - lastiso.FirstParameter()) / 2);
            }
            // cylinder
            if (((Math.Abs(R2 - R1)) < theToler) && ((Math.Abs(R3 - R1)) < theToler)
                && ((Math.Abs(R3 - R2)) < theToler))
            {
                gp_Ax3 Axes = new(P1, new gp_Dir(new gp_Vec(P1, P3)));
                aNewSurf = new Geom_CylindricalSurface(Axes, R1);
            }
            // cone
            else if ((((Math.Abs(R1)) > (Math.Abs(R2))) && ((Math.Abs(R2)) > (Math.Abs(R3))))
                     || (((Math.Abs(R3)) > (Math.Abs(R2))) && ((Math.Abs(R2)) > (Math.Abs(R1)))))
            {
                double radius;
                gp_Ax3 Axes;
                double semiangle = new gp_Vec(isoline).Angle(new gp_Vec(P3, P1));
                if (semiangle > M_PI / 2)
                {
                    semiangle = M_PI - semiangle;
                }
                if (R1 > R3)
                {
                    radius = R3;
                    Axes = new gp_Ax3(P3, new gp_Dir(new gp_Vec(P3, P1)));
                }
                else
                {
                    radius = R1;
                    Axes = new gp_Ax3(P1, new gp_Dir(new gp_Vec(P1, P3)));
                }
                aNewSurf = new Geom_ConicalSurface(Axes, semiangle, radius);
            }
        }
        return aNewSurf;
    }

    //=================================================================================================

    private static void GetLSGap(NCollection_HArray1<gp_XYZ> thePoints, in gp_Ax3 thePos, double theR, out double theGap)
    {
        theGap = 0.0;
        int i;
        gp_XYZ aLoc = thePos.Location().XYZ();
        gp_Dir aDir = thePos.Direction();
        for (i = thePoints.Lower(); i <= thePoints.Upper(); ++i)
        {
            gp_Vec aD = new(thePoints.Value(i) - aLoc);
            aD.Cross(new gp_Vec(aDir));
            theGap = Max(theGap, Math.Abs((aD.Magnitude() - theR)));
        }
    }

    private static bool GetCylByLS(NCollection_HArray1<gp_XYZ> thePoints,
                                   double theTol,
                                   ref gp_Ax3 thePos,
                                   ref double theR,
                                   out double theGap)
    {
        GetLSGap(thePoints, thePos, theR, out theGap);
        if (theGap <= Precision.Confusion())
        {
            return true;
        }

        int i;

        int aNbVar = 4;

        math_Vector aFBnd = new(1, aNbVar), aLBnd = new(1, aNbVar), aStartPoint = new(1, aNbVar);

        double aRelDev = 0.2; // Customer can set parameters of sample surface
                              //  with relative precision about aRelDev.
                              //  For example, if radius of sample surface is R,
                              //  it means, that "exact" value is in interav
                              //[R - aRelDev*R, R + aRelDev*R]. This interval is set
                              //  for R as boundary values for optimization algo.

        aStartPoint[1] = thePos.Location().X();
        aStartPoint[2] = thePos.Location().Y();
        aStartPoint[3] = thePos.Location().Z();
        aStartPoint[4] = theR;
        double aDR = aRelDev * theR;
        double aDXYZ = aDR;
        for (i = 1; i <= 3; ++i)
        {
            aFBnd[i] = aStartPoint[i] - aDXYZ;
            aLBnd[i] = aStartPoint[i] + aDXYZ;
        }
        aFBnd[4] = aStartPoint[4] - aDR;
        aLBnd[4] = aStartPoint[4] + aDR;

        //
        double aTol = Precision.Confusion();
        math_MultipleVarFunction aPFunc;
        GeomConvert_FuncCylinderLSDist aFuncCyl = new(thePoints, thePos.Direction());
        aPFunc = aFuncCyl;
        //
        math_Vector aSteps = new(1, aNbVar);
        int aNbInt = 10;
        for (i = 1; i <= aNbVar; ++i)
        {
            aSteps[i] = (aLBnd[i] - aFBnd[i]) / aNbInt;
        }
        math_PSO aGlobSolver = new(aPFunc, aFBnd, aLBnd, aSteps);
        double aLSDist;
        aGlobSolver.Perform(aSteps, out aLSDist, aStartPoint);
        //
        gp_Pnt aLoc = new(aStartPoint[1], aStartPoint[2], aStartPoint[3]);
        thePos.SetLocation(aLoc);
        theR = aStartPoint[4];

        GetLSGap(thePoints, thePos, theR, out theGap);
        if (theGap <= aTol)
        {
            return true;
        }
        //
        math_Matrix aDirMatrix = new(1, aNbVar, 1, aNbVar, 0.0);
        for (i = 1; i <= aNbVar; i++)
        {
            aDirMatrix[i, i] = 1.0;
        }

        // Set search direction for location to be perpendicular to axis to avoid
        // searching along axis
        gp_Dir aDir = thePos.Direction();
        gp_Pln aPln = new(thePos.Location(), aDir);
        gp_Dir aUDir = aPln.Position().XDirection();
        gp_Dir aVDir = aPln.Position().YDirection();
        for (i = 1; i <= 3; ++i)
        {
            aDirMatrix[i, 1] = aUDir.Coord(i);
            aDirMatrix[i, 2] = aVDir.Coord(i);
            gp_Dir aUVDir = new(aUDir.XYZ() + aVDir.XYZ());
            aDirMatrix[i, 3] = aUVDir.Coord(i);
        }

        math_Powell aSolver = new(aPFunc, aTol);
        aSolver.Perform(aPFunc, aStartPoint, aDirMatrix);

        if (aSolver.IsDone())
        {
            gp_Ax3 aPos2 = thePos;
            aSolver.Location(aStartPoint);
            aLoc.SetCoord(aStartPoint[1], aStartPoint[2], aStartPoint[3]);
            aPos2.SetLocation(aLoc);
            double anR2 = aStartPoint[4], aGap2;
            //
            GetLSGap(thePoints, aPos2, anR2, out aGap2);
            //
            if (aGap2 < theGap)
            {
                theGap = aGap2;
                thePos = aPos2;
                theR = anR2;
            }
        }
        if (theGap <= theTol)
        {
            return true;
        }
        return false;
    }

    //=================================================================================================

    private static Geom_Surface? TryCylinderByGaussField(Geom_Surface theSurf,
                                                         double theU1,
                                                         double theU2,
                                                         double theV1,
                                                         double theV2,
                                                         double theToler,
                                                         int theNbU = 20,
                                                         int theNbV = 20,
                                                         bool theLeastSquare = false)
    {
        Geom_Surface? aNewSurf = null;
        double du = (theU2 - theU1) / theNbU, dv = (theV2 - theV1) / theNbV;
        double aSigmaR = 0.0;
        double aTol = 1.0e3 * theToler;
        NCollection_Array1<double> anRs = new(1, theNbU * theNbV);
        NCollection_HArray1<gp_XYZ>? aPoints = null;
        if (theLeastSquare)
        {
            // OCCT: sized NbU * NbU (occt-issues.md); its only caller passes NbU == NbV
            aPoints = new NCollection_HArray1<gp_XYZ>(1, theNbU * theNbU);
        }
        //
        GeomLProp_SLProps aProps = new(theSurf, 2, Precision.Confusion());
        double anAvMaxCurv = 0.0, anAvMinCurv = 0.0, anAvR = 0, aSign = 1.0;
        gp_XYZ anAvDir = new();
        gp_Dir aMinD, aMaxD;
        int i, j, n = 0;
        double anU, aV;
        for (i = 1, anU = theU1 + du / 2.0; i <= theNbU; ++i, anU += du)
        {
            for (j = 1, aV = theV1 + dv / 2.0; j <= theNbV; ++j, aV += dv)
            {
                aProps.SetParameters(anU, aV);
                if (!aProps.IsCurvatureDefined())
                {
                    return aNewSurf;
                }
                if (aProps.IsUmbilic())
                {
                    return aNewSurf;
                }
                ++n;
                double aMinCurv = aProps.MinCurvature();
                double aMaxCurv = aProps.MaxCurvature();
                double aGaussCurv = Math.Abs(aProps.GaussianCurvature());
                double aK1 = Math.Sqrt(aGaussCurv);
                if (aK1 > theToler)
                {
                    return aNewSurf;
                }
                gp_XYZ aD;
                aProps.CurvatureDirections(out aMaxD, out aMinD);
                aMinCurv = Math.Abs(aMinCurv);
                aMaxCurv = Math.Abs(aMaxCurv);
                if (aMinCurv > aMaxCurv)
                {
                    // aMinCurv < 0;
                    aSign = -1.0;
                    (aMinCurv, aMaxCurv) = (aMaxCurv, aMinCurv);
                    gp_Dir aDummy = aMaxD;
                    aMaxD = aMinD;
                    aMinD = aDummy;
                }
                double anR = 1.0 / aMaxCurv;
                double anR2 = anR * anR;
                anRs[n] = anR;
                //
                if (n > 1)
                {
                    if (Math.Abs(aMaxCurv - anAvMaxCurv / (n - 1)) > aTol / anR2)
                    {
                        return aNewSurf;
                    }
                    if (Math.Abs(aMinCurv - anAvMinCurv / (n - 1)) > aTol)
                    {
                        return aNewSurf;
                    }
                }
                aD = aMinD.XYZ();
                anAvR += anR;
                anAvDir += aD;
                anAvMaxCurv += aMaxCurv;
                anAvMinCurv += aMinCurv;
                if (theLeastSquare)
                {
                    aPoints!.SetValue(n, aProps.Value().XYZ());
                }
            }
        }
        anAvMaxCurv /= n;
        anAvMinCurv /= n;
        anAvR /= n;
        anAvDir /= n;
        //
        if (Math.Abs(anAvMinCurv) > theToler)
        {
            return aNewSurf;
        }
        //
        for (i = 1; i <= n; ++i)
        {
            double d = (anRs[i] - anAvR);
            aSigmaR += d * d;
        }
        aSigmaR = Math.Sqrt(aSigmaR / n);
        aSigmaR = 3.0 * aSigmaR / Math.Sqrt(n);
        if (aSigmaR > aTol)
        {
            return aNewSurf;
        }
        aProps.SetParameters(theU1, theV1);
        if (!aProps.IsCurvatureDefined())
        {
            return aNewSurf;
        }
        gp_Dir aNorm = aProps.Normal();
        gp_Pnt aLoc = aProps.Value();
        gp_Dir anAxD = new(anAvDir);
        gp_Vec aT = new(aSign * anAvR * aNorm.XYZ());
        aLoc.Translate(aT);
        gp_Ax1 anAx1 = new(aLoc, anAxD);
        gp_Cylinder aCyl = new();
        aCyl.SetAxis(anAx1);
        aCyl.SetRadius(anAvR);

        if (theLeastSquare)
        {
            gp_Ax3 aPos = aCyl.Position();
            double anR = aCyl.Radius();
            bool IsDone = GetCylByLS(aPoints!, theToler, ref aPos, ref anR, out double _);
            if (IsDone)
            {
                aCyl.SetPosition(aPos);
                aCyl.SetRadius(anR);
            }
        }

        aNewSurf = new Geom_CylindricalSurface(aCyl);

        return aNewSurf;
    }

    //=================================================================================================

    private static Geom_Surface? TryTorusSphere(Geom_Surface theSurf,
                                                Geom_Circle circle,
                                                Geom_Circle otherCircle,
                                                double Param1,
                                                double Param2,
                                                double aParam1ToCrv,
                                                double aParam2ToCrv,
                                                double toler,
                                                bool isTryUMajor)
    {
        Geom_Surface? newSurface = null;
        double cf, cl;
        Geom_Curve IsoCrv1;
        Geom_Curve IsoCrv2;
        double aGap1 = 0.0, aGap2 = 0.0;
        // initial radius
        double R = circle.Circ().Radius();
        // iso lines

        if (isTryUMajor)
        {
            IsoCrv1 = theSurf.VIso(Param1 + ((Param2 - Param1) / 3.0));
            IsoCrv2 = theSurf.VIso(Param1 + ((Param2 - Param1) * 2.0 / 3));
        }
        else
        {
            IsoCrv1 = theSurf.UIso(Param1 + ((Param2 - Param1) / 3.0));
            IsoCrv2 = theSurf.UIso(Param1 + ((Param2 - Param1) * 2.0 / 3));
        }

        Geom_Curve? Crv1 = GeomConvert_CurveToAnaCurve.ComputeCurve(IsoCrv1,
                                                                    toler,
                                                                    aParam1ToCrv,
                                                                    aParam2ToCrv,
                                                                    out cf,
                                                                    out cl,
                                                                    ref aGap1,
                                                                    GeomConvert_Target,
                                                                    GeomAbs_Circle);
        Geom_Curve? Crv2 = GeomConvert_CurveToAnaCurve.ComputeCurve(IsoCrv2,
                                                                    toler,
                                                                    aParam1ToCrv,
                                                                    aParam2ToCrv,
                                                                    out cf,
                                                                    out cl,
                                                                    ref aGap2,
                                                                    GeomConvert_Target,
                                                                    GeomAbs_Circle);
        if (Crv1 is not Geom_Circle aCircle1 || Crv2 is not Geom_Circle aCircle2)
        {
            return newSurface;
        }

        double R1 = aCircle1.Circ().Radius();
        double R2 = aCircle2.Circ().Radius();

        // check radiuses
        if ((Math.Abs(R - R1) > toler) || (Math.Abs(R - R2) > toler))
        {
            return newSurface;
        }

        // get centers of the major radius
        gp_Pnt aPnt1, aPnt2, aPnt3;
        aPnt1 = circle.Circ().Location();
        aPnt2 = aCircle1.Circ().Location();
        aPnt3 = aCircle2.Circ().Location();

        // double eps = 1.e-09;  // angular resolution
        double d0 = aPnt1.Distance(aPnt2);
        double d1 = aPnt1.Distance(aPnt3);
        gp_Circ circ = new();

        if (d0 < toler || d1 < toler)
        {
            // compute sphere
            gp_Dir MainDir = otherCircle.Circ().Axis().Direction();
            gp_Ax3 Axes = new(circle.Circ().Location(), MainDir);
            Geom_SphericalSurface anObject = new(Axes, R);
            newSurface = anObject;

            return newSurface;
        }

        if (!GeomConvert_CurveToAnaCurve.GetCircle(ref circ, aPnt1, aPnt2, aPnt3) /*, d0, d1, eps)*/)
        {
            return newSurface;
        }

        double aMajorR = circ.Radius();
        gp_Pnt aCenter = circ.Location();
        gp_Dir aDir = new((aPnt1.XYZ() - aCenter.XYZ()) ^ (aPnt3.XYZ() - aCenter.XYZ()));
        gp_Ax3 anAx3 = new(aCenter, aDir);
        newSurface = new Geom_ToroidalSurface(anAx3, aMajorR, R);
        return newSurface;
    }

    //=================================================================================================

    private static double ComputeGap(Geom_Surface theSurf,
                                     double theU1,
                                     double theU2,
                                     double theV1,
                                     double theV2,
                                     Geom_Surface theNewSurf,
                                     double theTol = double.MaxValue) // RealLast()
    {
        GeomAdaptor_Surface aGAS = new(theNewSurf);
        GeomAbs_SurfaceType aSType = aGAS.GetType();
        gp_Pln aPln = new();
        gp_Cylinder aCyl = new();
        gp_Cone aCon = new();
        gp_Sphere aSphere = new();
        gp_Torus aTor = new();
        switch (aSType)
        {
            case GeomAbs_Plane:
                aPln = aGAS.Plane();
                break;
            case GeomAbs_Cylinder:
                aCyl = aGAS.Cylinder();
                break;
            case GeomAbs_Cone:
                aCon = aGAS.Cone();
                break;
            case GeomAbs_Sphere:
                aSphere = aGAS.Sphere();
                break;
            case GeomAbs_Torus:
                aTor = aGAS.Torus();
                break;
            default:
                return Precision.Infinite();
        }

        double aGap = 0.0;
        bool onSurface = true;

        double S, T;
        gp_Pnt P3d, P3d2;

        const int NP = 21;
        double DU, DV;
        int j, i;
        DU = (theU2 - theU1) / (NP - 1);
        DV = (theV2 - theV1) / (NP - 1);
        double DU2 = DU / 2.0, DV2 = DV / 2.0;
        for (j = 1; (j < NP) && onSurface; j++)
        {
            double V = theV1 + DV * (j - 1) + DV2;
            for (i = 1; i < NP; i++)
            {
                double U = theU1 + DU * (i - 1) + DU2;
                theSurf.D0(U, V, out P3d);

                switch (aSType)
                {
                    case GeomAbs_Plane:
                    {
                        ElSLib.Parameters(aPln, P3d, out S, out T);
                        P3d2 = ElSLib.Value(S, T, aPln);
                        break;
                    }
                    case GeomAbs_Cylinder:
                    {
                        ElSLib.Parameters(aCyl, P3d, out S, out T);
                        P3d2 = ElSLib.Value(S, T, aCyl);
                        break;
                    }
                    case GeomAbs_Cone:
                    {
                        ElSLib.Parameters(aCon, P3d, out S, out T);
                        P3d2 = ElSLib.Value(S, T, aCon);
                        break;
                    }
                    case GeomAbs_Sphere:
                    {
                        ElSLib.Parameters(aSphere, P3d, out S, out T);
                        P3d2 = ElSLib.Value(S, T, aSphere);
                        break;
                    }
                    case GeomAbs_Torus:
                    {
                        ElSLib.Parameters(aTor, P3d, out S, out T);
                        P3d2 = ElSLib.Value(S, T, aTor);
                        break;
                    }
                    default:
                        S = 0.0;
                        T = 0.0;
                        theNewSurf.D0(S, T, out P3d2);
                        break;
                }

                double dis = P3d.Distance(P3d2);
                if (dis > aGap)
                {
                    aGap = dis;
                }

                if (aGap > theTol)
                {
                    onSurface = false;
                    break;
                }
            }
        }
        return aGap;
    }

    //=================================================================================================

    /// <summary>
    /// Tries to convert the surface to an analytical form within InitialToler, on its whole domain (infinite ranges
    /// are taken as [-1, 1]). Returns null on failure.
    /// </summary>
    public Geom_Surface? ConvertToAnalytical(double InitialToler)
    {
        double U1, U2, V1, V2;
        mySurf!.Bounds(out U1, out U2, out V1, out V2);
        if (Precision.IsInfinite(U1) && Precision.IsInfinite(U2))
        {
            U1 = -1.0;
            U2 = 1.0;
        }
        if (Precision.IsInfinite(V1) && Precision.IsInfinite(V2))
        {
            V1 = -1.0;
            V2 = 1.0;
            if (mySurf is Geom_SurfaceOfRevolution aRevSurf)
            {
                CheckVTrimForRevSurf(aRevSurf, ref V1, ref V2);
            }
        }
        return ConvertToAnalytical(InitialToler, U1, U2, V1, V2);
    }

    //=================================================================================================

    /// <summary>
    /// Tries to convert the surface, on the given parameter range, to an analytical form within InitialToler.
    /// Returns null on failure, and when the range is outside the surface's.
    /// </summary>
    public Geom_Surface? ConvertToAnalytical(double InitialToler, double Umin, double Umax, double Vmin, double Vmax)
    {
        //
        GeomAdaptor_Surface aGAS = new(mySurf!);
        GeomAbs_SurfaceType aSType = aGAS.GetType();
        switch (aSType)
        {
            case GeomAbs_Plane:
            {
                myGap = 0.0;
                return new Geom_Plane(aGAS.Plane());
            }
            case GeomAbs_Cylinder:
            {
                myGap = 0.0;
                return new Geom_CylindricalSurface(aGAS.Cylinder());
            }
            case GeomAbs_Cone:
            {
                myGap = 0.0;
                return new Geom_ConicalSurface(aGAS.Cone());
            }
            case GeomAbs_Sphere:
            {
                myGap = 0.0;
                return new Geom_SphericalSurface(aGAS.Sphere());
            }
            case GeomAbs_Torus:
            {
                myGap = 0.0;
                return new Geom_ToroidalSurface(aGAS.Torus());
            }
            default:
                break;
        }
        //
        double toler = InitialToler;
        Geom_Surface?[] newSurf = new Geom_Surface?[5];
        Span<double> dd = [RealLast(), RealLast(), RealLast(), RealLast(), RealLast()];
        ReadOnlySpan<GeomAbs_SurfaceType> aSTypes =
            [GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus];

        // Check boundaries
        double U1, U2, V1, V2;
        mySurf!.Bounds(out U1, out U2, out V1, out V2);
        bool aDoSegment = false;
        double aTolBnd = Precision.PConfusion();
        int isurf = 0;
        if (Umin < U1 || Umax > U2 || Vmin < V1 || Vmax > V2)
        {
            return newSurf[isurf];
        }
        else
        {
            if (Umin - U1 > aTolBnd)
            {
                U1 = Umin;
                aDoSegment = true;
            }
            if (U2 - Umax > aTolBnd)
            {
                U2 = Umax;
                aDoSegment = true;
            }
            if (Vmin - V1 > aTolBnd)
            {
                V1 = Vmin;
                aDoSegment = true;
            }
            if (V2 - Vmax > aTolBnd)
            {
                V2 = Vmax;
                aDoSegment = true;
            }
        }

        bool IsBz = aSType == GeomAbs_BezierSurface;
        bool IsBs = aSType == GeomAbs_BSplineSurface;

        Geom_Surface aTempS = mySurf;
        if (IsBs)
        {
            // A copy (unfrozen), segmented: the caller's surface is never changed.
            Geom_BSplineSurface aBs = (Geom_BSplineSurface)mySurf.Copy();
            if (aDoSegment)
            {
                aBs.Segment(U1, U2, V1, V2);
            }
            aTempS = aBs;
        }
        else if (IsBz)
        {
            Geom_BezierSurface aBz = (Geom_BezierSurface)mySurf.Copy();
            if (aDoSegment)
            {
                aBz.Segment(U1, U2, V1, V2);
            }
            aTempS = aBz;
        }
        // check the planarity first
        if (!IsBs && !IsBz)
        {
            aTempS = new Geom_RectangularTrimmedSurface(aTempS, U1, U2, V1, V2);
        }
        isurf = 0; // set plane
        GeomLib_IsPlanarSurface GeomIsPlanar = new(aTempS, toler);
        if (GeomIsPlanar.IsPlanar())
        {
            gp_Pln newPln = GeomIsPlanar.Plan();
            newSurf[isurf] = new Geom_Plane(newPln);
            dd[isurf] = ComputeGap(aTempS, U1, U2, V1, V2, newSurf[isurf]!);
            if (myConvType == GeomConvert_Simplest
                || (myConvType == GeomConvert_Target && myTarget == GeomAbs_Plane))
            {
                myGap = dd[isurf];
                return newSurf[isurf];
            }
        }
        else
        {
            if (myConvType == GeomConvert_Target && myTarget == GeomAbs_Plane)
            {
                myGap = dd[isurf];
                return newSurf[isurf];
            }
        }

        // OCCT: evaluates at U1 + U2, not at the middle (U1 + U2) / 2 (occt-issues.md)
        double diagonal = mySurf.Value(U1, V1).Distance(mySurf.Value((U1 + U2), (V1 + V2) / 2));
        double twist = 1000;
        if (toler > diagonal / twist)
        {
            toler = diagonal / twist;
        }

        isurf = 1; // set cylinder
        bool aCylinderConus = false;
        bool aToroidSphere = false;

        // convert middle uiso and viso to canonical representation
        double VMid = 0.5 * (V1 + V2);
        double UMid = 0.5 * (U1 + U2);
        // occ::handle<Geom_Surface> TrSurf = aTempS;

        Geom_Curve UIso = aTempS.UIso(UMid);
        Geom_Curve VIso = aTempS.VIso(VMid);

        double cuf, cul, cvf, cvl, aGap1 = 0.0, aGap2 = 0.0;
        bool aLineIso = false;
        Geom_Curve? umidiso = GeomConvert_CurveToAnaCurve.ComputeCurve(UIso,
                                                                       toler,
                                                                       V1,
                                                                       V2,
                                                                       out cuf,
                                                                       out cul,
                                                                       ref aGap1,
                                                                       GeomConvert_Simplest);
        if (umidiso is not null)
        {
            aLineIso = umidiso is Geom_Line;
        }
        Geom_Curve? vmidiso = GeomConvert_CurveToAnaCurve.ComputeCurve(VIso,
                                                                       toler,
                                                                       U1,
                                                                       U2,
                                                                       out cvf,
                                                                       out cvl,
                                                                       ref aGap2,
                                                                       GeomConvert_Simplest);
        if (vmidiso is not null && !aLineIso)
        {
            aLineIso = vmidiso is Geom_Line;
        }
        if (umidiso is not null && vmidiso is not null)
        {
            //
            bool VCase = false;

            if (umidiso is Geom_Circle && vmidiso is Geom_Circle)
            {
                aToroidSphere = true;
                if (myConvType == GeomConvert_Target
                    && (myTarget == GeomAbs_Cylinder || myTarget == GeomAbs_Cone))
                {
                    isurf = 1;
                    myGap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 3; // set sphere
            }
            else if (umidiso is Geom_Line && vmidiso is Geom_Circle)
            {
                aCylinderConus = true;
                VCase = true;
                if (myConvType == GeomConvert_Target
                    && (myTarget == GeomAbs_Sphere || myTarget == GeomAbs_Torus))
                {
                    isurf = 3;
                    myGap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 1; // set cylinder
            }
            else if (umidiso is Geom_Circle && vmidiso is Geom_Line)
            {
                aCylinderConus = true;
                if (myConvType == GeomConvert_Target
                    && (myTarget == GeomAbs_Sphere || myTarget == GeomAbs_Torus))
                {
                    isurf = 3;
                    myGap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 1; // set cylinder
            }

            // case of torus-sphere
            if (aToroidSphere)
            {
                isurf = 3; // Set spherical surface

                Geom_Circle Ucircle = (Geom_Circle)umidiso;
                Geom_Circle Vcircle = (Geom_Circle)vmidiso;
                // torus
                // try when V isolines is with same radius
                Geom_Surface? anObject = TryTorusSphere(mySurf, Vcircle, Ucircle, V1, V2, U1, U2, toler, true);
                if (anObject is null)
                { // try when U isolines is with same radius
                    anObject = TryTorusSphere(mySurf, Ucircle, Vcircle, U1, U2, V1, V2, toler, false);
                }

                if (anObject is not null)
                {
                    if (anObject is Geom_ToroidalSurface)
                    {
                        isurf = 4; // set torus
                    }
                    newSurf[isurf] = anObject;
                    if (myConvType == GeomConvert_Target && (myTarget != aSTypes[isurf]))
                    {
                        myGap = RealLast();
                        return null;
                    }
                }
                else
                {
                    myGap = dd[isurf];
                }
            }
            // case of cone - cylinder
            else if (aCylinderConus)
            {
                isurf = 1; // set cylindrical  surface
                Geom_Surface? anObject =
                    TryCylinerCone(aTempS, VCase, umidiso, vmidiso, U1, U2, V1, V2, toler);
                if (anObject is not null)
                {
                    if (anObject is Geom_ConicalSurface)
                    {
                        isurf = 2; // set conical surface
                    }
                    if (myConvType == GeomConvert_Target && (myTarget != aSTypes[isurf]))
                    {
                        myGap = RealLast();
                        return null;
                    }
                    newSurf[isurf] = anObject;
                }
                else
                {
                    aCylinderConus = false;
                    myGap = dd[isurf];
                }
            }
        }
        // Additional checking for case of cylinder
        if (!aCylinderConus && !aToroidSphere && aLineIso)
        {
            // Try cylinder using Gauss field
            int aNbU = 7, aNbV = 7;
            bool aLeastSquare = true;
            Geom_Surface? anObject =
                TryCylinderByGaussField(aTempS, U1, U2, V1, V2, toler, aNbU, aNbV, aLeastSquare);
            if (anObject is not null)
            {
                isurf = 1;
                newSurf[isurf] = anObject;
            }
        }

        //
        //---------------------------------------------------------------------
        //                 verification
        //---------------------------------------------------------------------
        int imin = -1;
        double aDmin = RealLast();
        for (isurf = 0; isurf < 5; ++isurf)
        {
            if (newSurf[isurf] is null)
            {
                continue;
            }
            dd[isurf] = ComputeGap(aTempS, U1, U2, V1, V2, newSurf[isurf]!, toler);
            if (dd[isurf] <= toler)
            {
                if (myConvType == GeomConvert_Simplest
                    || (myConvType == GeomConvert_Target && myTarget == aSTypes[isurf]))
                {
                    myGap = dd[isurf];
                    return newSurf[isurf];
                }
                else if (myConvType == GeomConvert_MinGap)
                {
                    if (dd[isurf] < aDmin)
                    {
                        aDmin = dd[isurf];
                        imin = isurf;
                    }
                }
            }
        }
        //
        if (imin >= 0)
        {
            myGap = dd[imin];
            return newSurf[imin];
        }

        return null;
    }

    //=================================================================================================

    /// <summary>Whether two elementary surfaces are the same within tol.</summary>
    public static bool IsSame(Geom_Surface S1, Geom_Surface S2, double tol)
    {
        // only elementary surfaces are processed
        if (S1 is not Geom_ElementarySurface || S2 is not Geom_ElementarySurface)
        {
            return false;
        }

        GeomAdaptor_Surface anAdaptor1 = new(S1);
        GeomAdaptor_Surface anAdaptor2 = new(S2);

        GeomAbs_SurfaceType aST1 = anAdaptor1.GetType();
        GeomAbs_SurfaceType aST2 = anAdaptor2.GetType();

        if (aST1 != aST2)
        {
            return false;
        }

        IntAna_QuadQuadGeo interii = new();
        if (aST1 == GeomAbs_Plane)
        {
            interii.Perform(anAdaptor1.Plane(), anAdaptor2.Plane(), tol, tol);
        }
        else if (aST1 == GeomAbs_Cylinder)
        {
            interii.Perform(anAdaptor1.Cylinder(), anAdaptor2.Cylinder(), tol);
        }
        else if (aST1 == GeomAbs_Cone)
        {
            interii.Perform(anAdaptor1.Cone(), anAdaptor2.Cone(), tol);
        }
        else if (aST1 == GeomAbs_Sphere)
        {
            interii.Perform(anAdaptor1.Sphere(), anAdaptor2.Sphere(), tol);
        }
        else if (aST1 == GeomAbs_Torus)
        {
            interii.Perform(anAdaptor1.Torus(), anAdaptor2.Torus(), tol);
        }

        if (!interii.IsDone())
        {
            return false;
        }

        IntAna_ResultType aTypeRes = interii.TypeInter();

        return aTypeRes == IntAna_Same;
    }

    //=================================================================================================

    /// <summary>Whether S is a plane, cylinder, cone, sphere or torus.</summary>
    public static bool IsCanonical(Geom_Surface? S)
    {
        if (S is null)
        {
            return false;
        }

        if (S is Geom_Plane || S is Geom_CylindricalSurface
            || S is Geom_ConicalSurface
            || S is Geom_SphericalSurface
            || S is Geom_ToroidalSurface)
        {
            return true;
        }

        return false;
    }
}
