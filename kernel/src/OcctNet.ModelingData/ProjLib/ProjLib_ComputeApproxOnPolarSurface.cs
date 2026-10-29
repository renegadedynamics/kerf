// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApproxOnPolarSurface.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApproxOnPolarSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;
using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Approximates the projection of a 3D curve on a polar surface (a surface that may have singular points: sphere,
/// cone, torus, B-spline, Bezier or offset surfaces). The result is a 2D B-spline curve; its points are computed
/// from the extrema between the 3D curve's points and the surface, starting from a rough initial 2D curve.
/// </summary>
/// <remarks>
/// <para>The tolerance is the largest allowed 3D deviation of the projected curve from the exact projection; the 2D
/// tolerance is derived from it through the surface's U and V resolutions.</para>
/// <para>A working object (decision 21): the setters, <c>Perform</c>, <see cref="BuildInitialCurve2d"/> and
/// <see cref="ProjectUsingInitialCurve2d"/> are public. It keeps references to the caller's adaptors only during a
/// call. <see cref="BSpline"/> and <see cref="Curve2d"/> freeze the stored curves and return the same objects at each
/// call; the public <c>Perform</c> overload that returns a curve and <see cref="ProjectUsingInitialCurve2d"/> return
/// new curves the caller owns, unfrozen.</para>
/// </remarks>
public class ProjLib_ComputeApproxOnPolarSurface
{
    private bool myProjIsDone;
    private double myTolerance;
    private Geom2d_BSplineCurve? myBSpline;
    private Geom2d_Curve? my2ndCurve;
    private double myTolReached;
    private int myDegMin;
    private int myDegMax;
    private int myMaxSegments;
    private double myMaxDist;
    private AppParCurves_Constraint myBndPnt;
    private double myDist;

    /// <summary>OCCT's file-local struct aFuncStruct.</summary>
    private sealed class aFuncStruct
    {
        public Adaptor3d_Surface mySurf = null!; // Surface where to project.
        public Adaptor3d_Curve myCurve = null!; // Curve to project.
        public Adaptor2d_Curve2d myInitCurve2d = null!; // Initial 2dcurve projection.
        public double mySqProjOrtTol; // Used to filter non-orthogonal projected point.
        public double myTolU;
        public double myTolV;
        public readonly double[] myPeriod = new double[2]; // U and V period correspondingly.
    }

    private static void computePeriodicity(Adaptor3d_Surface theSurf, out double theUPeriod, out double theVPeriod)
    {
        theUPeriod = 0.0;
        theVPeriod = 0.0;

        // Compute once information about periodicity.
        // Param space may be reduced in case of rectangular trimmed surface,
        // in this case really trimmed bounds should be set as unperiodic.
        double aTrimF, aTrimL, aBaseF, aBaseL, aDummyF, aDummyL;
        Geom_Surface aS = GeomAdaptor.MakeSurface(theSurf, false); // Not trim.
        // U param space.
        if (theSurf.IsUPeriodic())
        {
            theUPeriod = theSurf.UPeriod();
        }
        else if (theSurf.IsUClosed())
        {
            theUPeriod = theSurf.LastUParameter() - theSurf.FirstUParameter();
        }
        if (Math.Abs(theUPeriod) > Precision.PConfusion())
        {
            aTrimF = theSurf.FirstUParameter(); // Trimmed first
            aTrimL = theSurf.LastUParameter(); // Trimmed last
            aS.Bounds(out aBaseF, out aBaseL, out aDummyF, out aDummyL); // Non-trimmed values.
            if (Math.Abs(aBaseF - aTrimF) + Math.Abs(aBaseL - aTrimL) > Precision.PConfusion())
            {
                // Param space reduced.
                theUPeriod = 0.0;
            }
        }

        // V param space.
        if (theSurf.IsVPeriodic())
        {
            theVPeriod = theSurf.VPeriod();
        }
        else if (theSurf.IsVClosed())
        {
            theVPeriod = theSurf.LastVParameter() - theSurf.FirstVParameter();
        }
        if (Math.Abs(theVPeriod) > Precision.PConfusion())
        {
            aTrimF = theSurf.FirstVParameter(); // Trimmed first
            aTrimL = theSurf.LastVParameter(); // Trimmed last
            aS.Bounds(out aDummyF, out aDummyL, out aBaseF, out aBaseL); // Non-trimmed values.
            if (Math.Abs(aBaseF - aTrimF) + Math.Abs(aBaseL - aTrimL) > Precision.PConfusion())
            {
                // Param space reduced.
                theVPeriod = 0.0;
            }
        }
    }

    private static double anOrthogSqValue(in gp_Pnt aBasePnt, Adaptor3d_Surface Surf, double theU, double theV)
    {
        // Since find projection, formula is:
        // F1 = Dot(S_U, Vec(aBasePnt, aProjPnt))
        // F2 = Dot(S_V, Vec(aBasePnt, aProjPnt))

        gp_Pnt aProjPnt;
        gp_Vec aSu, aSv;

        Surf.D1(theU, theV, out aProjPnt, out aSu, out aSv);
        gp_Vec aBaseVec = new(aBasePnt, aProjPnt);

        if (aSu.SquareMagnitude() > Precision.SquareConfusion())
        {
            aSu.Normalize();
        }

        if (aSv.SquareMagnitude() > Precision.SquareConfusion())
        {
            aSv.Normalize();
        }

        double aFirstPart = aSu.Dot(aBaseVec);
        double aSecondPart = aSv.Dot(aBaseVec);
        return (aFirstPart * aFirstPart + aSecondPart * aSecondPart);
    }

    private static gp_Pnt2d Function_Value(double theU, aFuncStruct theData)
    {
        gp_Pnt2d p2d = theData.myInitCurve2d.Value(theU);
        gp_Pnt p = theData.myCurve.Value(theU);
        gp_Pnt aSurfPnt = theData.mySurf.Value(p2d.X(), p2d.Y());
        double aSurfPntDist = aSurfPnt.SquareDistance(p);

        double Uinf, Usup, Vinf, Vsup;
        Uinf = theData.mySurf.FirstUParameter();
        Usup = theData.mySurf.LastUParameter();
        Vinf = theData.mySurf.FirstVParameter();
        Vsup = theData.mySurf.LastVParameter();

        // Check case when curve is close to co-parametrized isoline on surf.
        if (Math.Abs(p2d.X() - Uinf) < Precision.PConfusion()
            || Math.Abs(p2d.X() - Usup) < Precision.PConfusion())
        {
            // V isoline.
            gp_Pnt aPnt;
            theData.mySurf.D0(p2d.X(), theU, out aPnt);
            if (aPnt.SquareDistance(p) < aSurfPntDist)
            {
                p2d.SetY(theU);
            }
        }

        if (Math.Abs(p2d.Y() - Vinf) < Precision.PConfusion()
            || Math.Abs(p2d.Y() - Vsup) < Precision.PConfusion())
        {
            // U isoline.
            gp_Pnt aPnt;
            theData.mySurf.D0(theU, p2d.Y(), out aPnt);
            if (aPnt.SquareDistance(p) < aSurfPntDist)
            {
                p2d.SetX(theU);
            }
        }

        int decalU = 0, decalV = 0;
        double U0 = p2d.X(), V0 = p2d.Y();

        GeomAbs_SurfaceType Type = theData.mySurf.GetType();
        if ((Type != GeomAbs_BSplineSurface) && (Type != GeomAbs_BezierSurface)
            && (Type != GeomAbs_OffsetSurface))
        {
            // Analytical cases.
            double S = 0.0, T = 0.0;
            switch (Type)
            {
                case GeomAbs_Cylinder:
                {
                    gp_Cylinder Cylinder = theData.mySurf.Cylinder();
                    ElSLib.Parameters(Cylinder, p, out S, out T);
                    if (U0 < Uinf)
                    {
                        decalU = -(int)((Uinf - U0) / (2 * M_PI)) - 1;
                    }
                    if (U0 > Usup)
                    {
                        decalU = (int)((U0 - Usup) / (2 * M_PI)) + 1;
                    }
                    S += decalU * 2 * M_PI;
                    break;
                }
                case GeomAbs_Cone:
                {
                    gp_Cone Cone = theData.mySurf.Cone();
                    ElSLib.Parameters(Cone, p, out S, out T);
                    if (U0 < Uinf)
                    {
                        decalU = -(int)((Uinf - U0) / (2 * M_PI)) - 1;
                    }
                    if (U0 > Usup)
                    {
                        decalU = (int)((U0 - Usup) / (2 * M_PI)) + 1;
                    }
                    S += decalU * 2 * M_PI;
                    break;
                }
                case GeomAbs_Sphere:
                {
                    gp_Sphere Sphere = theData.mySurf.Sphere();
                    ElSLib.Parameters(Sphere, p, out S, out T);
                    if (U0 < Uinf)
                    {
                        decalU = -(int)((Uinf - U0) / (2 * M_PI)) - 1;
                    }
                    if (U0 > Usup)
                    {
                        decalU = (int)((U0 - Usup) / (2 * M_PI)) + 1;
                    }
                    S += decalU * 2 * M_PI;
                    if (V0 < Vinf)
                    {
                        decalV = -(int)((Vinf - V0) / (2 * M_PI)) - 1;
                    }
                    if (V0 > (Vsup + (Vsup - Vinf)))
                    {
                        decalV = (int)((V0 - Vsup + (Vsup - Vinf)) / (2 * M_PI)) + 1;
                    }
                    T += decalV * 2 * M_PI;
                    if (0.4 * M_PI < Math.Abs(U0 - S) && Math.Abs(U0 - S) < 1.6 * M_PI)
                    {
                        T = M_PI - T;
                        if (U0 < S)
                        {
                            S -= M_PI;
                        }
                        else
                        {
                            S += M_PI;
                        }
                    }
                    break;
                }
                case GeomAbs_Torus:
                {
                    gp_Torus Torus = theData.mySurf.Torus();
                    ElSLib.Parameters(Torus, p, out S, out T);
                    if (U0 < Uinf)
                    {
                        decalU = -(int)((Uinf - U0) / (2 * M_PI)) - 1;
                    }
                    if (U0 > Usup)
                    {
                        decalU = (int)((U0 - Usup) / (2 * M_PI)) + 1;
                    }
                    if (V0 < Vinf)
                    {
                        decalV = -(int)((Vinf - V0) / (2 * M_PI)) - 1;
                    }
                    if (V0 > Vsup)
                    {
                        decalV = (int)((V0 - Vsup) / (2 * M_PI)) + 1;
                    }
                    S += decalU * 2 * M_PI;
                    T += decalV * 2 * M_PI;
                    break;
                }
                default:
                    throw new Standard_NoSuchObject("ProjLib_ComputeApproxOnPolarSurface::Value");
            }
            return new gp_Pnt2d(S, T);
        }

        // Non-analytical case.
        double Dist2Min = RealLast();
        double uperiod = theData.myPeriod[0], vperiod = theData.myPeriod[1], u, v;

        // U0 and V0 are the points within the initialized period.
        if (U0 < Uinf)
        {
            if (uperiod == 0.0)
            {
                U0 = Uinf;
            }
            else
            {
                decalU = (int)((Uinf - U0) / uperiod) + 1;
                U0 += decalU * uperiod;
            }
        }
        if (U0 > Usup)
        {
            if (uperiod == 0.0)
            {
                U0 = Usup;
            }
            else
            {
                decalU = -((int)((U0 - Usup) / uperiod) + 1);
                U0 += decalU * uperiod;
            }
        }
        if (V0 < Vinf)
        {
            if (vperiod == 0.0)
            {
                V0 = Vinf;
            }
            else
            {
                decalV = (int)((Vinf - V0) / vperiod) + 1;
                V0 += decalV * vperiod;
            }
        }
        if (V0 > Vsup)
        {
            if (vperiod == 0.0)
            {
                V0 = Vsup;
            }
            else
            {
                decalV = -(int)((V0 - Vsup) / vperiod) - 1;
                V0 += decalV * vperiod;
            }
        }

        // The surface around (U0,V0) is reduced.
        double uLittle = (Usup - Uinf) / 10, vLittle = (Vsup - Vinf) / 10;
        double uInfLi = 0, vInfLi = 0, uSupLi = 0, vSupLi = 0;
        if ((U0 - Uinf) > uLittle)
        {
            uInfLi = U0 - uLittle;
        }
        else
        {
            uInfLi = Uinf;
        }
        if ((V0 - Vinf) > vLittle)
        {
            vInfLi = V0 - vLittle;
        }
        else
        {
            vInfLi = Vinf;
        }
        if ((Usup - U0) > uLittle)
        {
            uSupLi = U0 + uLittle;
        }
        else
        {
            uSupLi = Usup;
        }
        if ((Vsup - V0) > vLittle)
        {
            vSupLi = V0 + vLittle;
        }
        else
        {
            vSupLi = Vsup;
        }

        GeomAdaptor_Surface SurfLittle = new();
        if (Type == GeomAbs_BSplineSurface)
        {
            Geom_Surface GBSS = theData.mySurf.BSpline();
            SurfLittle.Load(GBSS, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else if (Type == GeomAbs_BezierSurface)
        {
            Geom_Surface GS = theData.mySurf.Bezier();
            SurfLittle.Load(GS, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else if (Type == GeomAbs_OffsetSurface)
        {
            Geom_Surface GS = GeomAdaptor.MakeSurface(theData.mySurf);
            SurfLittle.Load(GS, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else
        {
            throw new Standard_NoSuchObject(
                "ProjLib_ComputeApproxOnPolarSurface::ProjectUsingInitialCurve2d() - unknown surface type");
        }

        // Try to run simple search with initial point (U0, V0).
        Extrema_GenLocateExtPS locext = new(SurfLittle, theData.myTolU, theData.myTolV);
        locext.Perform(p, U0, V0);
        if (locext.IsDone())
        {
            locext.Point().Parameter(out u, out v);
            Dist2Min = anOrthogSqValue(p, theData.mySurf, u, v);
            if (Dist2Min < theData.mySqProjOrtTol && // Point is projection.
                locext.SquareDistance()
                < aSurfPntDist + Precision.SquareConfusion()) // Point better than initial.
            {
                gp_Pnt2d pnt = new(u - decalU * uperiod, v - decalV * vperiod);
                return pnt;
            }
        }

        // Perform whole param space search.
        Extrema_ExtPS ext = new(p, SurfLittle, theData.myTolU, theData.myTolV, Extrema_ExtFlag_MIN);
        if (ext.IsDone() && ext.NbExt() >= 1)
        {
            Dist2Min = ext.SquareDistance(1);
            int GoodValue = 1;
            for (int i = 2; i <= ext.NbExt(); i++)
            {
                if (Dist2Min > ext.SquareDistance(i))
                {
                    Dist2Min = ext.SquareDistance(i);
                    GoodValue = i;
                }
            }
            ext.Point(GoodValue).Parameter(out u, out v);
            Dist2Min = anOrthogSqValue(p, theData.mySurf, u, v);
            if (Dist2Min < theData.mySqProjOrtTol && // Point is projection.
                ext.SquareDistance(GoodValue)
                < aSurfPntDist + Precision.SquareConfusion()) // Point better than initial.
            {
                gp_Pnt2d pnt = new(u - decalU * uperiod, v - decalV * vperiod);
                return pnt;
            }
        }

        // Both searches return bad values, use point from initial 2dcurve.
        return p2d;
    }

    /// <summary>OCCT's file-local class ProjLib_PolarFunction: the 2D point of the projection at a parameter.</summary>
    private sealed class ProjLib_PolarFunction : AppCont_Function
    {
        private readonly aFuncStruct myStruct = new();

        public ProjLib_PolarFunction(Adaptor3d_Curve C,
                                     Adaptor3d_Surface Surf,
                                     Adaptor2d_Curve2d InitialCurve2d,
                                     double Tol3d)
        {
            myNbPnt = 0;
            myNbPnt2d = 1;

            computePeriodicity(Surf, out myStruct.myPeriod[0], out myStruct.myPeriod[1]);

            myStruct.myCurve = C;
            myStruct.myInitCurve2d = InitialCurve2d;
            myStruct.mySurf = Surf;
            myStruct.mySqProjOrtTol = 10000.0 * Tol3d * Tol3d;
            myStruct.myTolU = Surf.UResolution(Tol3d);
            myStruct.myTolV = Surf.VResolution(Tol3d);
        }

        public override double FirstParameter() => myStruct.myCurve.FirstParameter();

        public override double LastParameter() => myStruct.myCurve.LastParameter();

        public gp_Pnt2d Value(double t) => Function_Value(t, myStruct);

        public override bool Value(double theT, NCollection_Array1<gp_Pnt2d> thePnt2d, NCollection_Array1<gp_Pnt> thePnt)
        {
            thePnt2d[1] = Function_Value(theT, myStruct);
            return true;
        }

        public override bool D1(double theT, NCollection_Array1<gp_Vec2d> theVec2d, NCollection_Array1<gp_Vec> theVec)
        {
            return false;
        }
    }

    /// <summary>Sets the initial values: tolerance Precision::Approximation(), OCCT's defaults for the rest.</summary>
    public ProjLib_ComputeApproxOnPolarSurface()
    {
        myProjIsDone = false;
        myTolerance = Precision.Approximation();
        myTolReached = -1.0;
        myDegMin = -1;
        myDegMax = -1;
        myMaxSegments = -1;
        myMaxDist = -1.0;
        myBndPnt = AppParCurves_TangencyPoint;
        myDist = 0.0;
    }

    /// <summary>
    /// Projects, using the initial curve <paramref name="theInitialCurve2d"/>, a rough approximation of the result.
    /// <paramref name="theTolerance3D"/> is the 3D tolerance of the approximation.
    /// </summary>
    public ProjLib_ComputeApproxOnPolarSurface(Adaptor2d_Curve2d? theInitialCurve2d,
                                               Adaptor3d_Curve theCurve,
                                               Adaptor3d_Surface theSurface,
                                               double theTolerance3D)
        : this()
    {
        myTolerance = theTolerance3D;
        myBSpline = Perform(theInitialCurve2d, theCurve, theSurface);
    }

    /// <summary>Projects <paramref name="theCurve"/> on <paramref name="theSurface"/>.</summary>
    public ProjLib_ComputeApproxOnPolarSurface(Adaptor3d_Curve theCurve,
                                               Adaptor3d_Surface theSurface,
                                               double theTolerance3D = 1.0e-4)
        : this()
    {
        myTolerance = theTolerance3D;
        Adaptor2d_Curve2d? anInitCurve2d = null;
        myBSpline = Perform(anInitCurve2d, theCurve, theSurface);
    }

    /// <summary>
    /// Projects using two initial curves, rough approximations of the two pcurves of a seam edge; the result is in
    /// <see cref="BSpline"/> and <see cref="Curve2d"/>.
    /// </summary>
    public ProjLib_ComputeApproxOnPolarSurface(Adaptor2d_Curve2d theInitialCurve2d,
                                               Adaptor2d_Curve2d theInitialCurve2dBis,
                                               Adaptor3d_Curve theCurve,
                                               Adaptor3d_Surface theSurface,
                                               double theTolerance3D)
        : this()
    {
        myTolerance = theTolerance3D;
        // InitialCurve2d and InitialCurve2dBis are two pcurves of the sewing
        Geom2d_BSplineCurve? bsc = Perform(theInitialCurve2d, theCurve, theSurface);

        if (myProjIsDone)
        {
            gp_Pnt2d P2dproj, P2d, P2dBis;
            P2dproj = bsc!.StartPoint();
            P2d = theInitialCurve2d.Value(theInitialCurve2d.FirstParameter());
            P2dBis = theInitialCurve2dBis.Value(theInitialCurve2dBis.FirstParameter());

            double Dist, DistBis;
            Dist = P2dproj.Distance(P2d);
            DistBis = P2dproj.Distance(P2dBis);
            if (Dist < DistBis)
            {
                // myBSpline2d is the pcurve that is found. It is translated to obtain myCurve2d
                myBSpline = bsc;
                Geom2d_Geometry GG = myBSpline.Translated(P2d, P2dBis);
                my2ndCurve = GG as Geom2d_Curve;
            }
            else
            {
                my2ndCurve = bsc;
                Geom2d_Geometry GG = my2ndCurve.Translated(P2dBis, P2d);
                myBSpline = GG as Geom2d_BSplineCurve;
            }
        }
    }

    // Raises the degree of C1 or C2 in place: both are curves the algorithm has just built.
    private static Geom2d_BSplineCurve Concat(Geom2d_BSplineCurve C1,
                                              Geom2d_BSplineCurve C2,
                                              double theUJump,
                                              double theVJump)
    {
        int deg, deg1, deg2;
        deg1 = C1.Degree();
        deg2 = C2.Degree();

        if (deg1 < deg2)
        {
            C1.IncreaseDegree(deg2);
            deg = deg2;
        }
        else if (deg2 < deg1)
        {
            C2.IncreaseDegree(deg1);
            deg = deg1;
        }
        else
        {
            deg = deg1;
        }

        int np1, np2, nk1, nk2, np, nk;
        np1 = C1.NbPoles();
        nk1 = C1.NbKnots();
        np2 = C2.NbPoles();
        nk2 = C2.NbKnots();
        nk = nk1 + nk2 - 1;
        np = np1 + np2 - 1;

        NCollection_Array1<double> K1 = C1.InternalKnots();
        NCollection_Array1<int> M1 = C1.InternalMultiplicities();
        NCollection_Array1<gp_Pnt2d> P1 = C1.InternalPoles();
        NCollection_Array1<double> K2 = C2.InternalKnots();
        NCollection_Array1<int> M2 = C2.InternalMultiplicities();
        NCollection_Array1<gp_Pnt2d> P2 = C2.InternalPoles();

        // Compute the new BSplineCurve
        NCollection_Array1<double> K = new(1, nk);
        NCollection_Array1<int> M = new(1, nk);
        NCollection_Array1<gp_Pnt2d> P = new(1, np);

        int i, count = 0;
        // Set Knots and Mults
        for (i = 1; i <= nk1; i++)
        {
            count++;
            K[count] = K1[i];
            M[count] = M1[i];
        }
        M[count] = deg;
        for (i = 2; i <= nk2; i++)
        {
            count++;
            K[count] = K2[i];
            M[count] = M2[i];
        }
        // Set the Poles
        count = 0;
        for (i = 1; i <= np1; i++)
        {
            count++;
            P[count] = P1[i];
        }
        for (i = 2; i <= np2; i++)
        {
            count++;
            P[count].SetX(P2[i].X() + theUJump);
            P[count].SetY(P2[i].Y() + theVJump);
        }

        Geom2d_BSplineCurve BS = new(P, K, M, deg);
        return BS;
    }

    /// <summary>
    /// Projects with the parameters set before (or their defaults); the result is in <see cref="BSpline"/>.
    /// </summary>
    public void Perform(Adaptor3d_Curve Curve, Adaptor3d_Surface S)
    {
        Adaptor2d_Curve2d? anInitCurve2d = null;
        myBSpline = Perform(anInitCurve2d, Curve, S);
    }

    /// <summary>
    /// Projects with the parameters set before, starting from <paramref name="InitialCurve2d"/>, a rough estimation
    /// of the result (null: built by <see cref="BuildInitialCurve2d"/>). Returns a new curve, or null on failure;
    /// the result is not stored (<see cref="BSpline"/> may then be the initial curve that was built).
    /// </summary>
    public Geom2d_BSplineCurve? Perform(Adaptor2d_Curve2d? InitialCurve2d, Adaptor3d_Curve Curve, Adaptor3d_Surface S)
    {
        // OCC217
        double Tol3d = myTolerance;
        double ParamTol = Precision.PApproximation();

        Adaptor2d_Curve2d? AHC2d = InitialCurve2d;
        Adaptor3d_Curve AHC = Curve;

        // if the curve 3d is a BSpline with degree C0, it is cut into sections with degree C1
        // -> bug cts18237
        GeomAbs_CurveType typeCurve = Curve.GetType();
        if (typeCurve == GeomAbs_BSplineCurve)
        {
            NCollection_List<Standard_Transient> LOfBSpline2d = new();
            Geom_BSplineCurve BSC = Curve.BSpline();
            int nbInter = Curve.NbIntervals(GeomAbs_C1);
            if (nbInter > 1)
            {
                int i, j;
                Geom_TrimmedCurve GTC;
                Geom2d_TrimmedCurve? G2dTC = null;
                NCollection_Array1<double> Inter = new(1, nbInter + 1);
                Curve.Intervals(Inter, GeomAbs_C1);
                double firstinter = Inter.Value(1), secondinter = Inter.Value(2);
                // initialization 3d
                GTC = new Geom_TrimmedCurve(BSC, firstinter, secondinter);
                AHC = new GeomAdaptor_Curve(GTC);

                // if there is an initialization curve:
                // - either this is a BSpline C0, with discontinuity at the same parameters of nodes
                // and the sections C1 are taken
                // - or this is a curve C1 and the sections of interest are taken otherwise the curve is
                // created.

                // initialization 2d
                int nbInter2d;
                bool C2dIsToCompute;
                C2dIsToCompute = InitialCurve2d is null;
                Geom2d_BSplineCurve? BSC2d;
                Geom2d_Curve? G2dC = null;

                if (!C2dIsToCompute)
                {
                    nbInter2d = InitialCurve2d!.NbIntervals(GeomAbs_C1);
                    NCollection_Array1<double> Inter2d = new(1, nbInter2d + 1);
                    InitialCurve2d.Intervals(Inter2d, GeomAbs_C1);
                    j = 1;
                    for (i = 1, j = 1; i <= nbInter; i++)
                    {
                        // OCCT: j reaches nbInter2d + 2 when the 2D intervals end before the 3D ones, and Inter2d(j)
                        // then reads past the array (the port throws there).
                        if (Math.Abs(Inter.Value(i) - Inter2d.Value(j)) < ParamTol)
                        {
                            if (j > nbInter2d)
                            {
                                break;
                            }
                            j++;
                        }
                    }
                    if (j != (nbInter2d + 1))
                    {
                        C2dIsToCompute = true;
                    }
                }

                if (C2dIsToCompute)
                {
                    AHC2d = BuildInitialCurve2d(AHC, S);
                }
                else
                {
                    typeCurve = InitialCurve2d!.GetType();
                    switch (typeCurve)
                    {
                        case GeomAbs_Line:
                        {
                            G2dC = new Geom2d_Line(InitialCurve2d.Line());
                            break;
                        }
                        case GeomAbs_Circle:
                        {
                            G2dC = new Geom2d_Circle(InitialCurve2d.Circle());
                            break;
                        }
                        case GeomAbs_Ellipse:
                        {
                            G2dC = new Geom2d_Ellipse(InitialCurve2d.Ellipse());
                            break;
                        }
                        case GeomAbs_Hyperbola:
                        {
                            G2dC = new Geom2d_Hyperbola(InitialCurve2d.Hyperbola());
                            break;
                        }
                        case GeomAbs_Parabola:
                        {
                            G2dC = new Geom2d_Parabola(InitialCurve2d.Parabola());
                            break;
                        }
                        case GeomAbs_BezierCurve:
                        {
                            G2dC = InitialCurve2d.Bezier();
                            break;
                        }
                        case GeomAbs_BSplineCurve:
                        {
                            G2dC = InitialCurve2d.BSpline();
                            break;
                        }
                        case GeomAbs_OtherCurve:
                        default:
                            break;
                    }
                    // OCCT: an initial curve of type OtherCurve leaves G2dC null and dereferences it (the port throws
                    // NullReferenceException).
                    gp_Pnt2d fp2d = G2dC!.Value(firstinter), lp2d = G2dC.Value(secondinter);
                    gp_Pnt fps, lps, fpc, lpc;
                    S.D0(fp2d.X(), fp2d.Y(), out fps);
                    S.D0(lp2d.X(), lp2d.Y(), out lps);
                    Curve.D0(firstinter, out fpc);
                    Curve.D0(secondinter, out lpc);
                    // OCC217
                    if ((fps.IsEqual(fpc, Tol3d)) && (lps.IsEqual(lpc, Tol3d)))
                    {
                        G2dTC = new Geom2d_TrimmedCurve(G2dC, firstinter, secondinter);
                        AHC2d = new Geom2dAdaptor_Curve(G2dTC);
                        myProjIsDone = true;
                    }
                    else
                    {
                        AHC2d = BuildInitialCurve2d(AHC, S);
                        C2dIsToCompute = true;
                    }
                }

                if (myProjIsDone)
                {
                    BSC2d = ProjectUsingInitialCurve2d(AHC, S, AHC2d!);
                    if (BSC2d is null)
                    {
                        return null;
                    }
                    LOfBSpline2d.Append(BSC2d);
                }
                else
                {
                    return null;
                }

                int nbK2d = BSC2d.NbKnots();
                int deg = BSC2d.Degree();

                for (i = 2; i <= nbInter; i++)
                {
                    double iinter = Inter.Value(i);
                    double ip1inter = Inter.Value(i + 1);
                    // general case 3d
                    GTC.SetTrim(iinter, ip1inter);
                    AHC = new GeomAdaptor_Curve(GTC);

                    // general case 2d
                    if (C2dIsToCompute)
                    {
                        AHC2d = BuildInitialCurve2d(AHC, S);
                    }
                    else
                    {
                        gp_Pnt2d fp2d = G2dC!.Value(iinter), lp2d = G2dC.Value(ip1inter);
                        gp_Pnt fps, lps, fpc, lpc;
                        S.D0(fp2d.X(), fp2d.Y(), out fps);
                        S.D0(lp2d.X(), lp2d.Y(), out lps);
                        Curve.D0(iinter, out fpc);
                        Curve.D0(ip1inter, out lpc);
                        // OCC217
                        if ((fps.IsEqual(fpc, Tol3d)) && (lps.IsEqual(lpc, Tol3d)))
                        {
                            G2dTC!.SetTrim(iinter, ip1inter);
                            AHC2d = new Geom2dAdaptor_Curve(G2dTC);
                            myProjIsDone = true;
                        }
                        else
                        {
                            AHC2d = BuildInitialCurve2d(AHC, S);
                        }
                    }
                    if (myProjIsDone)
                    {
                        BSC2d = ProjectUsingInitialCurve2d(AHC, S, AHC2d!);
                        if (BSC2d is null)
                        {
                            return null;
                        }
                        LOfBSpline2d.Append(BSC2d);

                        nbK2d += BSC2d.NbKnots() - 1; // unused but set for debug
                        deg = Max(deg, BSC2d.Degree());
                    }
                    else
                    {
                        return null;
                    }
                }
                _ = nbK2d;

                double anUPeriod, anVPeriod;
                computePeriodicity(S, out anUPeriod, out anVPeriod);
                int NbC = LOfBSpline2d.Extent();
                Geom2d_BSplineCurve CurBS;
                CurBS = (Geom2d_BSplineCurve)LOfBSpline2d.First();
                LOfBSpline2d.RemoveFirst();
                for (int ii = 2; ii <= NbC; ii++)
                {
                    Geom2d_BSplineCurve BS = (Geom2d_BSplineCurve)LOfBSpline2d.First();

                    // Check for period jump in point of contact.
                    gp_Pnt2d aC1End = CurBS.Pole(CurBS.NbPoles()); // End of C1.
                    gp_Pnt2d aC2Beg = BS.Pole(1); // Beginning of C2.
                    double anUJump = 0.0, anVJump = 0.0;

                    if (anUPeriod > 0.0 && Math.Abs(aC1End.X() - aC2Beg.X()) > (anUPeriod) / 2.01)
                    {
                        double aMultCoeff = aC2Beg.X() < aC1End.X() ? 1.0 : -1.0;
                        anUJump = (anUPeriod) * aMultCoeff;
                    }

                    if (anVPeriod != 0.0 && Math.Abs(aC1End.Y() - aC2Beg.Y()) > (anVPeriod) / 2.01)
                    {
                        double aMultCoeff = aC2Beg.Y() < aC1End.Y() ? 1.0 : -1.0;
                        anVJump = (anVPeriod) * aMultCoeff;
                    }

                    CurBS = Concat(CurBS, BS, anUJump, anVJump);
                    LOfBSpline2d.RemoveFirst();
                }
                return CurBS;
            }
        }

        if (InitialCurve2d is null)
        {
            AHC2d = BuildInitialCurve2d(Curve, S);
            if (!myProjIsDone)
            {
                return null;
            }
        }
        return ProjectUsingInitialCurve2d(AHC, S, AHC2d!);
    }

    /// <summary>
    /// Builds an initial 2D curve, a B-spline of degree 1 through the projections (by extrema) of points of the
    /// curve. Returns null on failure (<see cref="IsDone"/> false).
    /// </summary>
    /// <remarks>The curve is also stored as <see cref="BSpline"/>, and the adaptor returned reads it.</remarks>
    public Adaptor2d_Curve2d? BuildInitialCurve2d(Adaptor3d_Curve Curve, Adaptor3d_Surface Surf)
    {
        //  discretize the Curve with quasiuniform deflection
        //  density at least NbOfPnts points
        myProjIsDone = false;

        // OCC217
        double Tol3d = myTolerance;
        double TolU = Surf.UResolution(Tol3d), TolV = Surf.VResolution(Tol3d);
        double DistTol3d = 100.0 * Tol3d;
        if (myMaxDist > 0.0)
        {
            DistTol3d = myMaxDist;
        }
        double DistTol3d2 = DistTol3d * DistTol3d;
        double uperiod, vperiod;
        computePeriodicity(Surf, out uperiod, out vperiod);

        // NO myTol is Tol2d !!!!

        int NbOfPnts = 61;
        GCPnts_QuasiUniformAbscissa QUA = new(Curve, NbOfPnts);
        NbOfPnts = QUA.NbPoints();
        NCollection_Array1<gp_Pnt> Pts = new(1, NbOfPnts);
        NCollection_Array1<double> Param = new(1, NbOfPnts);
        int i, j;
        for (i = 1; i <= NbOfPnts; i++)
        {
            Param[i] = QUA.Parameter(i);
            Pts[i] = Curve.Value(Param[i]);
        }

        NCollection_Array1<gp_Pnt2d> Pts2d = new(1, NbOfPnts);
        NCollection_Array1<int> Mult = new(1, NbOfPnts);
        Mult.Init(1);
        Mult[1] = Mult[NbOfPnts] = 2;

        double Uinf, Usup, Vinf, Vsup;
        Uinf = Surf.FirstUParameter();
        Usup = Surf.LastUParameter();
        Vinf = Surf.FirstVParameter();
        Vsup = Surf.LastVParameter();
        GeomAbs_SurfaceType Type = Surf.GetType();
        if ((Type != GeomAbs_BSplineSurface) && (Type != GeomAbs_BezierSurface)
            && (Type != GeomAbs_OffsetSurface))
        {
            double S, T;
            // to know the position relatively to the period
            switch (Type)
            {
                case GeomAbs_Cylinder:
                {
                    double Sloc;
                    int usens = 0;
                    gp_Cylinder Cylinder = Surf.Cylinder();
                    ElSLib.Parameters(Cylinder, Pts[1], out S, out T);
                    Pts2d[1].SetCoord(S, T);
                    for (i = 2; i <= NbOfPnts; i++)
                    {
                        Sloc = S;
                        ElSLib.Parameters(Cylinder, Pts[i], out S, out T);
                        if (Math.Abs(Sloc - S) > M_PI)
                        {
                            if (Sloc > S)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        Pts2d[i].SetCoord(S + usens * 2 * M_PI, T);
                    }
                    myProjIsDone = true;
                    break;
                }
                case GeomAbs_Cone:
                {
                    double Sloc;
                    int usens = 0;
                    gp_Cone Cone = Surf.Cone();
                    ElSLib.Parameters(Cone, Pts[1], out S, out T);
                    Pts2d[1].SetCoord(S, T);
                    for (i = 2; i <= NbOfPnts; i++)
                    {
                        Sloc = S;
                        ElSLib.Parameters(Cone, Pts[i], out S, out T);
                        if (Math.Abs(Sloc - S) > M_PI)
                        {
                            if (Sloc > S)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        Pts2d[i].SetCoord(S + usens * 2 * M_PI, T);
                    }
                    myProjIsDone = true;
                    break;
                }
                case GeomAbs_Sphere:
                {
                    double Sloc, Tloc;
                    int usens = 0, vsens = 0; // usens steps by half-period
                    bool vparit = false;
                    gp_Sphere Sphere = Surf.Sphere();
                    ElSLib.Parameters(Sphere, Pts[1], out S, out T);
                    Pts2d[1].SetCoord(S, T);
                    for (i = 2; i <= NbOfPnts; i++)
                    {
                        Sloc = S;
                        Tloc = T;
                        ElSLib.Parameters(Sphere, Pts[i], out S, out T);
                        if (1.6 * M_PI < Math.Abs(Sloc - S))
                        {
                            if (Sloc > S)
                            {
                                usens += 2;
                            }
                            else
                            {
                                usens -= 2;
                            }
                        }
                        if (1.6 * M_PI > Math.Abs(Sloc - S) && Math.Abs(Sloc - S) > 0.4 * M_PI)
                        {
                            vparit = !vparit;
                            if (Sloc > S)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                            if (Math.Abs(Tloc - Vsup) < (Vsup - Vinf) / 5)
                            {
                                vsens++;
                            }
                            else
                            {
                                vsens--;
                            }
                        }
                        if (vparit)
                        {
                            Pts2d[i].SetCoord(S + usens * M_PI, (M_PI - T) * (vsens - 1));
                        }
                        else
                        {
                            Pts2d[i].SetCoord(S + usens * M_PI, T + vsens * M_PI);
                        }
                    }
                    myProjIsDone = true;
                    break;
                }
                case GeomAbs_Torus:
                {
                    double Sloc, Tloc;
                    int usens = 0, vsens = 0;
                    gp_Torus Torus = Surf.Torus();
                    ElSLib.Parameters(Torus, Pts[1], out S, out T);
                    Pts2d[1].SetCoord(S, T);
                    for (i = 2; i <= NbOfPnts; i++)
                    {
                        Sloc = S;
                        Tloc = T;
                        ElSLib.Parameters(Torus, Pts[i], out S, out T);
                        if (Math.Abs(Sloc - S) > M_PI)
                        {
                            if (Sloc > S)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        if (Math.Abs(Tloc - T) > M_PI)
                        {
                            if (Tloc > T)
                            {
                                vsens++;
                            }
                            else
                            {
                                vsens--;
                            }
                        }
                        Pts2d[i].SetCoord(S + usens * 2 * M_PI, T + vsens * 2 * M_PI);
                    }
                    myProjIsDone = true;
                    break;
                }
                default:
                    throw new Standard_NoSuchObject("ProjLib_ComputeApproxOnPolarSurface::BuildInitialCurve2d");
            }
        }
        else
        {
            myProjIsDone = false;
            double Dist2Min = 1.0e+200, u = 0.0, v = 0.0;
            myDist = 0.0;
            gp_Pnt pntproj;

            NCollection_Sequence<gp_Pnt2d> Sols = new();
            bool areManyZeros = false;

            pntproj = Pts[1];
            Extrema_ExtPS aExtPS = new(pntproj, Surf, TolU, TolV, Extrema_ExtFlag_MIN);
            double aMinSqDist = RealLast();
            if (aExtPS.IsDone())
            {
                for (i = 1; i <= aExtPS.NbExt(); i++)
                {
                    double aSqDist = aExtPS.SquareDistance(i);
                    if (aSqDist < aMinSqDist)
                    {
                        aMinSqDist = aSqDist;
                    }
                }
            }
            if (aMinSqDist > DistTol3d2) // try to project with less tolerance
            {
                TolU = Min(TolU, Precision.PConfusion());
                TolV = Min(TolV, Precision.PConfusion());
                aExtPS.Initialize(Surf,
                                  Surf.FirstUParameter(),
                                  Surf.LastUParameter(),
                                  Surf.FirstVParameter(),
                                  Surf.LastVParameter(),
                                  TolU,
                                  TolV);
                aExtPS.Perform(pntproj);
            }

            if (aExtPS.IsDone() && aExtPS.NbExt() >= 1)
            {
                int GoodValue = 1;

                for (i = 1; i <= aExtPS.NbExt(); i++)
                {
                    if (aExtPS.SquareDistance(i) < DistTol3d2)
                    {
                        if (aExtPS.SquareDistance(i) <= 1.0e-18)
                        {
                            aExtPS.Point(i).Parameter(out u, out v);
                            gp_Pnt2d p2d = new(u, v);
                            bool isSame = false;
                            for (j = 1; j <= Sols.Length(); j++)
                            {
                                if (p2d.SquareDistance(Sols.Value(j)) <= 1.0e-18)
                                {
                                    isSame = true;
                                    break;
                                }
                            }
                            if (!isSame)
                            {
                                Sols.Append(p2d);
                            }
                        }
                        if (Dist2Min > aExtPS.SquareDistance(i))
                        {
                            Dist2Min = aExtPS.SquareDistance(i);
                            GoodValue = i;
                        }
                    }
                }

                if (Sols.Length() > 1)
                {
                    areManyZeros = true;
                }

                if (Dist2Min <= DistTol3d2)
                {
                    if (!areManyZeros)
                    {
                        aExtPS.Point(GoodValue).Parameter(out u, out v);
                        Pts2d[1].SetCoord(u, v);
                        myProjIsDone = true;
                    }
                    else
                    {
                        int nbSols = Sols.Length();
                        double Dist2Max = -1.0e+200;
                        for (i = 1; i <= nbSols; i++)
                        {
                            gp_Pnt2d aP1 = Sols.Value(i);
                            for (j = i + 1; j <= nbSols; j++)
                            {
                                gp_Pnt2d aP2 = Sols.Value(j);
                                double aDist2 = aP1.SquareDistance(aP2);
                                if (aDist2 > Dist2Max)
                                {
                                    Dist2Max = aDist2;
                                }
                            }
                        }
                        double aMaxT2 = Max(TolU, TolV);
                        aMaxT2 *= aMaxT2;
                        if (Dist2Max > aMaxT2)
                        {
                            int tPp = 0;
                            for (i = 1; i <= 5; i++)
                            {
                                int nbExtOk = 0;
                                int indExt = 0;
                                int iT = 1 + (NbOfPnts - 1) / 5 * i;
                                pntproj = Pts[iT];
                                Extrema_ExtPS aTPS = new(pntproj, Surf, TolU, TolV, Extrema_ExtFlag_MIN);
                                Dist2Min = 1.0e+200;
                                if (aTPS.IsDone() && aTPS.NbExt() >= 1)
                                {
                                    for (j = 1; j <= aTPS.NbExt(); j++)
                                    {
                                        if (aTPS.SquareDistance(j) < DistTol3d2)
                                        {
                                            nbExtOk++;
                                            if (aTPS.SquareDistance(j) < Dist2Min)
                                            {
                                                Dist2Min = aTPS.SquareDistance(j);
                                                indExt = j;
                                            }
                                        }
                                    }
                                }
                                if (nbExtOk == 1)
                                {
                                    tPp = iT;
                                    aTPS.Point(indExt).Parameter(out u, out v);
                                    break;
                                }
                            }

                            if (tPp != 0 && tPp != NbOfPnts)
                            {
                                gp_Pnt2d aPp = new(u, v);
                                gp_Pnt2d aPn = new();
                                bool isFound = false;
                                for (j = tPp + 1; j <= NbOfPnts; ++j)
                                {
                                    pntproj = Pts[j];
                                    Extrema_ExtPS aTPS = new(pntproj, Surf, TolU, TolV, Extrema_ExtFlag_MIN);
                                    Dist2Min = RealLast();
                                    if (aTPS.IsDone() && aTPS.NbExt() >= 1)
                                    {
                                        int indExt = 0;
                                        for (i = 1; i <= aTPS.NbExt(); i++)
                                        {
                                            if (aTPS.SquareDistance(i) < DistTol3d2 && aTPS.SquareDistance(i) < Dist2Min)
                                            {
                                                Dist2Min = aTPS.SquareDistance(i);
                                                indExt = i;
                                            }
                                        }
                                        if (indExt > 0)
                                        {
                                            aTPS.Point(indExt).Parameter(out u, out v);
                                            aPn = new gp_Pnt2d(u, v);
                                            isFound = true;
                                            break;
                                        }
                                    }
                                }

                                if (isFound)
                                {
                                    gp_Dir2d atV = new(new gp_Vec2d(aPp, aPn));
                                    bool isChosen = false;
                                    for (i = 1; i <= nbSols; i++)
                                    {
                                        gp_Pnt2d aP1 = Sols.Value(i);
                                        gp_Dir2d asV = new(new gp_Vec2d(aP1, aPp));
                                        if (asV.Dot(atV) > 0.0)
                                        {
                                            isChosen = true;
                                            u = aP1.X();
                                            v = aP1.Y();
                                            Pts2d[1].SetCoord(u, v);
                                            myProjIsDone = true;
                                            break;
                                        }
                                    }
                                    if (!isChosen)
                                    {
                                        aExtPS.Point(GoodValue).Parameter(out u, out v);
                                        Pts2d[1].SetCoord(u, v);
                                        myProjIsDone = true;
                                    }
                                }
                                else
                                {
                                    aExtPS.Point(GoodValue).Parameter(out u, out v);
                                    Pts2d[1].SetCoord(u, v);
                                    myProjIsDone = true;
                                }
                            }
                            else
                            {
                                aExtPS.Point(GoodValue).Parameter(out u, out v);
                                Pts2d[1].SetCoord(u, v);
                                myProjIsDone = true;
                            }
                        }
                        else
                        {
                            aExtPS.Point(GoodValue).Parameter(out u, out v);
                            Pts2d[1].SetCoord(u, v);
                            myProjIsDone = true;
                        }
                    }
                }

                //  calculate the following points with GenLocate_ExtPS
                // (and store the result and each parameter in a sequence)
                int usens = 0, vsens = 0;
                // to know the position relatively to the period
                double U0 = u, V0 = v, U1 = u, V1 = v;
                // U0 and V0 are the points in the initialized period
                // (period with u and v),
                // U1 and V1 are the points for construction of poles
                myDist = Dist2Min;
                // OCCT's fixed arrays aUpar[3] and aVpar[3] of the (i == 2) block below, hoisted out of the loop.
                Span<double> aUpar = stackalloc double[3];
                Span<double> aVpar = stackalloc double[3];
                for (i = 2; i <= NbOfPnts; i++)
                {
                    if (myProjIsDone)
                    {
                        myProjIsDone = false;
                        Dist2Min = RealLast();
                        pntproj = Pts[i];
                        Extrema_GenLocateExtPS aLocateExtPS = new(Surf, TolU, TolV);
                        aLocateExtPS.Perform(pntproj, U0, V0);

                        if (aLocateExtPS.IsDone())
                        {
                            if (aLocateExtPS.SquareDistance() < DistTol3d2)
                            { // OCC217
                                if (aLocateExtPS.SquareDistance() > myDist)
                                {
                                    myDist = aLocateExtPS.SquareDistance();
                                }
                                (aLocateExtPS.Point()).Parameter(out U0, out V0);
                                U1 = U0 + usens * uperiod;
                                V1 = V0 + vsens * vperiod;
                                Pts2d[i].SetCoord(U1, V1);
                                myProjIsDone = true;
                            }
                            else
                            {
                                Extrema_ExtPS aGlobalExtr = new(pntproj, Surf, TolU, TolV, Extrema_ExtFlag_MIN);
                                if (aGlobalExtr.IsDone())
                                {
                                    double LocalMinSqDist = RealLast();
                                    int imin = 0;
                                    for (int isol = 1; isol <= aGlobalExtr.NbExt(); isol++)
                                    {
                                        double aSqDist = aGlobalExtr.SquareDistance(isol);
                                        if (aSqDist < LocalMinSqDist)
                                        {
                                            LocalMinSqDist = aSqDist;
                                            imin = isol;
                                        }
                                    }
                                    if (LocalMinSqDist < DistTol3d2)
                                    {
                                        if (LocalMinSqDist > myDist)
                                        {
                                            myDist = LocalMinSqDist;
                                        }
                                        double LocalU, LocalV;
                                        aGlobalExtr.Point(imin).Parameter(out LocalU, out LocalV);
                                        if (uperiod > 0.0 && Math.Abs(U0 - LocalU) >= uperiod / 2.0)
                                        {
                                            if (LocalU > U0)
                                            {
                                                usens = -1;
                                            }
                                            else
                                            {
                                                usens = 1;
                                            }
                                        }
                                        if (vperiod > 0.0 && Math.Abs(V0 - LocalV) >= vperiod / 2.0)
                                        {
                                            if (LocalV > V0)
                                            {
                                                vsens = -1;
                                            }
                                            else
                                            {
                                                vsens = 1;
                                            }
                                        }
                                        U0 = LocalU;
                                        V0 = LocalV;
                                        U1 = U0 + usens * uperiod;
                                        V1 = V0 + vsens * vperiod;
                                        Pts2d[i].SetCoord(U1, V1);
                                        myProjIsDone = true;

                                        if ((i == 2) && (!IsEqual(uperiod, 0.0) || !IsEqual(vperiod, 0.0)))
                                        { // Make 1st point more precise for periodic surfaces
                                            const int aSize = 3;
                                            gp_Pnt2d aP = Pts2d[2];
                                            Pts2d[1].Coord(out aUpar[1], out aVpar[1]);
                                            aUpar[0] = aUpar[1] - uperiod;
                                            aUpar[2] = aUpar[1] + uperiod;
                                            aVpar[0] = aVpar[1] - vperiod;
                                            aVpar[2] = aVpar[1] + vperiod;

                                            double aSQdistMin = RealLast();
                                            int aBestUInd = 1, aBestVInd = 1;
                                            int aSizeU = IsEqual(uperiod, 0.0) ? 1 : aSize,
                                                aSizeV = IsEqual(vperiod, 0.0) ? 1 : aSize;
                                            for (int uInd = 0; uInd < aSizeU; uInd++)
                                            {
                                                for (int vInd = 0; vInd < aSizeV; vInd++)
                                                {
                                                    double aSQdist = aP.SquareDistance(new gp_Pnt2d(aUpar[uInd], aVpar[vInd]));
                                                    if (aSQdist < aSQdistMin)
                                                    {
                                                        aSQdistMin = aSQdist;
                                                        aBestUInd = uInd;
                                                        aBestVInd = vInd;
                                                    }
                                                }
                                            }

                                            Pts2d[1].SetCoord(aUpar[aBestUInd], aVpar[aBestVInd]);
                                        } // if(i == 2) condition
                                    }
                                }
                            }
                        }
                        if (!myProjIsDone && uperiod != 0.0)
                        {
                            double aUinf, aUsup, Uaux;
                            aUinf = Surf.FirstUParameter();
                            aUsup = Surf.LastUParameter();
                            if ((aUsup - U0) > (U0 - aUinf))
                            {
                                Uaux = 2 * aUinf - U0 + uperiod;
                            }
                            else
                            {
                                Uaux = 2 * aUsup - U0 - uperiod;
                            }

                            Extrema_GenLocateExtPS locext = new(Surf, TolU, TolV);
                            locext.Perform(pntproj, Uaux, V0);

                            if (locext.IsDone())
                            {
                                if (locext.SquareDistance() < DistTol3d2)
                                { // OCC217
                                    if (locext.SquareDistance() > myDist)
                                    {
                                        myDist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((aUsup - U0) > (U0 - aUinf))
                                    {
                                        usens--;
                                    }
                                    else
                                    {
                                        usens++;
                                    }
                                    U0 = u;
                                    V0 = v;
                                    U1 = U0 + usens * uperiod;
                                    V1 = V0 + vsens * vperiod;
                                    Pts2d[i].SetCoord(U1, V1);
                                    myProjIsDone = true;
                                }
                            }
                        }
                        if (!myProjIsDone && vperiod != 0.0)
                        {
                            double aVinf, aVsup, Vaux;
                            aVinf = Surf.FirstVParameter();
                            aVsup = Surf.LastVParameter();
                            if ((aVsup - V0) > (V0 - aVinf))
                            {
                                Vaux = 2 * aVinf - V0 + vperiod;
                            }
                            else
                            {
                                Vaux = 2 * aVsup - V0 - vperiod;
                            }

                            Extrema_GenLocateExtPS locext = new(Surf, TolU, TolV);
                            locext.Perform(pntproj, U0, Vaux);

                            if (locext.IsDone())
                            {
                                if (locext.SquareDistance() < DistTol3d2)
                                { // OCC217
                                    if (locext.SquareDistance() > myDist)
                                    {
                                        myDist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((aVsup - V0) > (V0 - aVinf))
                                    {
                                        vsens--;
                                    }
                                    else
                                    {
                                        vsens++;
                                    }
                                    U0 = u;
                                    V0 = v;
                                    U1 = U0 + usens * uperiod;
                                    V1 = V0 + vsens * vperiod;
                                    Pts2d[i].SetCoord(U1, V1);
                                    myProjIsDone = true;
                                }
                            }
                        }
                        if (!myProjIsDone && uperiod != 0.0 && vperiod != 0.0)
                        {
                            double Uaux, Vaux;
                            if ((Usup - U0) > (U0 - Uinf))
                            {
                                Uaux = 2 * Uinf - U0 + uperiod;
                            }
                            else
                            {
                                Uaux = 2 * Usup - U0 - uperiod;
                            }
                            if ((Vsup - V0) > (V0 - Vinf))
                            {
                                Vaux = 2 * Vinf - V0 + vperiod;
                            }
                            else
                            {
                                Vaux = 2 * Vsup - V0 - vperiod;
                            }

                            Extrema_GenLocateExtPS locext = new(Surf, TolU, TolV);
                            locext.Perform(pntproj, Uaux, Vaux);

                            if (locext.IsDone())
                            {
                                if (locext.SquareDistance() < DistTol3d2)
                                {
                                    if (locext.SquareDistance() > myDist)
                                    {
                                        myDist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((Usup - U0) > (U0 - Uinf))
                                    {
                                        usens--;
                                    }
                                    else
                                    {
                                        usens++;
                                    }
                                    if ((Vsup - V0) > (V0 - Vinf))
                                    {
                                        vsens--;
                                    }
                                    else
                                    {
                                        vsens++;
                                    }
                                    U0 = u;
                                    V0 = v;
                                    U1 = U0 + usens * uperiod;
                                    V1 = V0 + vsens * vperiod;
                                    Pts2d[i].SetCoord(U1, V1);
                                    myProjIsDone = true;
                                }
                            }
                        }
                        if (!myProjIsDone)
                        {
                            Extrema_ExtPS ext = new(pntproj, Surf, TolU, TolV, Extrema_ExtFlag_MIN);
                            if (ext.IsDone())
                            {
                                // OCCT: reads the first extremum without checking NbExt() > 0 (the port throws when
                                // there is none).
                                Dist2Min = ext.SquareDistance(1);
                                int aGoodValue = 1;
                                for (j = 2; j <= ext.NbExt(); j++)
                                {
                                    if (Dist2Min > ext.SquareDistance(j))
                                    {
                                        Dist2Min = ext.SquareDistance(j);
                                        aGoodValue = j;
                                    }
                                }
                                if (Dist2Min < DistTol3d2)
                                {
                                    if (Dist2Min > myDist)
                                    {
                                        myDist = Dist2Min;
                                    }
                                    (ext.Point(aGoodValue)).Parameter(out u, out v);
                                    if (uperiod != 0.0)
                                    {
                                        if ((U0 - u) > (2 * uperiod / 3))
                                        {
                                            usens++;
                                        }
                                        else if ((u - U0) > (2 * uperiod / 3))
                                        {
                                            usens--;
                                        }
                                    }
                                    if (vperiod != 0.0)
                                    {
                                        if ((V0 - v) > (vperiod / 2))
                                        {
                                            vsens++;
                                        }
                                        else if ((v - V0) > (vperiod / 2))
                                        {
                                            vsens--;
                                        }
                                    }
                                    U0 = u;
                                    V0 = v;
                                    U1 = U0 + usens * uperiod;
                                    V1 = V0 + vsens * vperiod;
                                    Pts2d[i].SetCoord(U1, V1);
                                    myProjIsDone = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        // -- Pnts2d is transformed into Geom2d_BSplineCurve, with the help of Param and Mult
        if (myProjIsDone)
        {
            Geom2d_BSplineCurve aBSpline = new(Pts2d, Param, Mult, 1);
            myBSpline = aBSpline;
            // jgv: put the curve into parametric range
            gp_Pnt2d MidPoint =
                aBSpline.Value(0.5 * (aBSpline.FirstParameter() + aBSpline.LastParameter()));
            double TestU = MidPoint.X(), TestV = MidPoint.Y();
            double sense = 0.0;
            if (uperiod != 0.0)
            {
                if (TestU < Uinf - TolU)
                {
                    sense = 1.0;
                }
                else if (TestU > Usup + TolU)
                {
                    sense = -1;
                }
                while (TestU < Uinf - TolU || TestU > Usup + TolU)
                {
                    TestU += sense * uperiod;
                }
            }
            if (vperiod != 0.0)
            {
                sense = 0.0;
                if (TestV < Vinf - TolV)
                {
                    sense = 1.0;
                }
                else if (TestV > Vsup + TolV)
                {
                    sense = -1.0;
                }
                while (TestV < Vinf - TolV || TestV > Vsup + TolV)
                {
                    TestV += sense * vperiod;
                }
            }
            gp_Vec2d Offset = new(TestU - MidPoint.X(), TestV - MidPoint.Y());
            if (Math.Abs(Offset.X()) > gp.Resolution() || Math.Abs(Offset.Y()) > gp.Resolution())
            {
                aBSpline.Translate(Offset);
            }
            //////////////////////////////////////////
            Adaptor2d_Curve2d IC2d = new Geom2dAdaptor_Curve(aBSpline);
            return IC2d;
        }
        else
        {
            return null;
        }
    }

    /// <summary>
    /// Projects, starting from <paramref name="InitCurve2d"/>. Returns a new curve the caller owns, or null when the
    /// approximation fails.
    /// </summary>
    public Geom2d_BSplineCurve? ProjectUsingInitialCurve2d(Adaptor3d_Curve Curve,
                                                           Adaptor3d_Surface Surf,
                                                           Adaptor2d_Curve2d InitCurve2d)
    {
        // OCC217
        double Tol3d = myTolerance;
        double DistTol3d = 100.0 * Tol3d;
        if (myMaxDist > 0.0)
        {
            DistTol3d = myMaxDist;
        }
        double DistTol3d2 = DistTol3d * DistTol3d;
        double TolU = Surf.UResolution(Tol3d), TolV = Surf.VResolution(Tol3d);
        double Tol2d = Max(Math.Sqrt(TolU * TolU + TolV * TolV), Precision.PConfusion());

        int i;
        GeomAbs_SurfaceType TheTypeS = Surf.GetType();
        GeomAbs_CurveType TheTypeC = Curve.GetType();
        if (TheTypeS == GeomAbs_Plane)
        {
            double S, T;
            gp_Pln Plane = Surf.Plane();
            if (TheTypeC == GeomAbs_BSplineCurve)
            {
                myTolReached = Precision.Confusion();
                Geom_BSplineCurve BSC = Curve.BSpline();
                NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                for (i = 1; i <= Curve.NbPoles(); i++)
                {
                    ElSLib.Parameters(Plane, BSC.Pole(i), out S, out T);
                    Poles2d[i].SetCoord(S, T);
                }
                NCollection_Array1<double> Knots = BSC.InternalKnots();
                NCollection_Array1<int> Mults = BSC.InternalMultiplicities();
                if (BSC.IsRational())
                {
                    NCollection_Array1<double>? pWeights = BSC.InternalWeights();
                    if (pWeights is not null)
                    {
                        return new Geom2d_BSplineCurve(Poles2d,
                                                       pWeights,
                                                       Knots,
                                                       Mults,
                                                       BSC.Degree(),
                                                       BSC.IsPeriodic());
                    }
                }
                return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BSC.Degree(), BSC.IsPeriodic());
            }
            if (TheTypeC == GeomAbs_BezierCurve)
            {
                myTolReached = Precision.Confusion();
                Geom_BezierCurve BC = Curve.Bezier();
                NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                for (i = 1; i <= Curve.NbPoles(); i++)
                {
                    ElSLib.Parameters(Plane, BC.Pole(i), out S, out T);
                    Poles2d[i].SetCoord(S, T);
                }
                NCollection_Array1<double> Knots = new(1, 2);
                Knots.SetValue(1, 0.0);
                Knots.SetValue(2, 1.0);
                NCollection_Array1<int> Mults = new(1, 2);
                Mults.Init(BC.NbPoles());
                if (BC.IsRational())
                {
                    NCollection_Array1<double>? pWeights = BC.InternalWeights();
                    if (pWeights is not null)
                    {
                        return new Geom2d_BSplineCurve(Poles2d,
                                                       pWeights,
                                                       Knots,
                                                       Mults,
                                                       BC.Degree(),
                                                       BC.IsPeriodic());
                    }
                }
                return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BC.Degree(), BC.IsPeriodic());
            }
        }
        if (TheTypeS == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface BSS = Surf.BSpline();
            // OCCT: MaxDegree() is the static maximal degree of B-spline surfaces (25), not this surface's degree, so
            // this condition is never true and the bilinear-surface branch is dead code.
            if ((Geom_BSplineSurface.MaxDegree() == 1) && (BSS.NbUPoles() == 2) && (BSS.NbVPoles() == 2))
            {
                gp_Pnt p11 = BSS.Pole(1, 1);
                gp_Pnt p12 = BSS.Pole(1, 2);
                gp_Pnt p21 = BSS.Pole(2, 1);
                gp_Pnt p22 = BSS.Pole(2, 2);
                gp_Vec V1 = new(p11, p12);
                gp_Vec V2 = new(p21, p22);
                if (V1.IsEqual(V2, Tol3d, Tol3d / (p11.Distance(p12) * 180 / M_PI)))
                {
                    // OCCT: the square distances are truncated to int (Dist2Min is an int), and the extrema start from
                    // the mean of the corners' X and Y coordinates, not from parameters.
                    int Dist2Min = IntegerLast();
                    double u, v;
                    if (TheTypeC == GeomAbs_BSplineCurve)
                    {
                        myTolReached = Tol3d;
                        Geom_BSplineCurve BSC = Curve.BSpline();
                        NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                        for (i = 1; i <= Curve.NbPoles(); i++)
                        {
                            myProjIsDone = false;
                            Dist2Min = IntegerLast();

                            Extrema_GenLocateExtPS extrloc = new(Surf, TolU, TolV);
                            extrloc.Perform(BSC.Pole(i), (p11.X() + p22.X()) / 2, (p11.Y() + p22.Y()) / 2);

                            if (extrloc.IsDone())
                            {
                                Dist2Min = (int)extrloc.SquareDistance();
                                if (Dist2Min < DistTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    Poles2d[i].SetCoord(u, v);
                                    myProjIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            if (!myProjIsDone)
                            {
                                break;
                            }
                        }
                        if (myProjIsDone)
                        {
                            NCollection_Array1<double> Knots = BSC.InternalKnots();
                            NCollection_Array1<int> Mults = BSC.InternalMultiplicities();
                            if (BSC.IsRational())
                            {
                                return new Geom2d_BSplineCurve(Poles2d,
                                                               BSC.InternalWeightsArray(),
                                                               Knots,
                                                               Mults,
                                                               BSC.Degree(),
                                                               BSC.IsPeriodic());
                            }
                            return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BSC.Degree(), BSC.IsPeriodic());
                        }
                    }
                    if (TheTypeC == GeomAbs_BezierCurve)
                    {
                        myTolReached = Tol3d;
                        Geom_BezierCurve BC = Curve.Bezier();
                        NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                        for (i = 1; i <= Curve.NbPoles(); i++)
                        {
                            Dist2Min = IntegerLast();

                            Extrema_GenLocateExtPS extrloc = new(Surf, TolU, TolV);
                            extrloc.Perform(BC.Pole(i), 0.5, 0.5);

                            if (extrloc.IsDone())
                            {
                                Dist2Min = (int)extrloc.SquareDistance();
                                if (Dist2Min < DistTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    Poles2d[i].SetCoord(u, v);
                                    myProjIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            // OCCT: the flag is reset after each pole, so the curve below is never built (with at
                            // least one pole); the projection falls through to the approximation.
                            if (myProjIsDone)
                            {
                                myProjIsDone = false;
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (myProjIsDone)
                        {
                            NCollection_Array1<double> Knots = new(1, 2);
                            Knots.SetValue(1, 0.0);
                            Knots.SetValue(2, 1.0);
                            NCollection_Array1<int> Mults = new(1, 2);
                            Mults.Init(BC.NbPoles());
                            if (BC.IsRational())
                            {
                                return new Geom2d_BSplineCurve(Poles2d,
                                                               BC.InternalWeightsArray(),
                                                               Knots,
                                                               Mults,
                                                               BC.Degree(),
                                                               BC.IsPeriodic());
                            }
                            return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BC.Degree(), BC.IsPeriodic());
                        }
                    }
                }
            }
        }
        else if (TheTypeS == GeomAbs_BezierSurface)
        {
            Geom_BezierSurface BS = Surf.Bezier();
            // OCCT: as above, MaxDegree() is static (25): this branch is dead code.
            if ((Geom_BezierSurface.MaxDegree() == 1) && (BS.NbUPoles() == 2) && (BS.NbVPoles() == 2))
            {
                gp_Pnt p11 = BS.Pole(1, 1);
                gp_Pnt p12 = BS.Pole(1, 2);
                gp_Pnt p21 = BS.Pole(2, 1);
                gp_Pnt p22 = BS.Pole(2, 2);
                gp_Vec V1 = new(p11, p12);
                gp_Vec V2 = new(p21, p22);
                if (V1.IsEqual(V2, Tol3d, Tol3d / (p11.Distance(p12) * 180 / M_PI)))
                {
                    // OCCT: as for the B-spline surface above.
                    int Dist2Min = IntegerLast();
                    double u, v;

                    if (TheTypeC == GeomAbs_BSplineCurve)
                    {
                        myTolReached = Tol3d;
                        Geom_BSplineCurve BSC = Curve.BSpline();
                        NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                        for (i = 1; i <= Curve.NbPoles(); i++)
                        {
                            myProjIsDone = false;
                            Dist2Min = IntegerLast();

                            Extrema_GenLocateExtPS extrloc = new(Surf, TolU, TolV);
                            extrloc.Perform(BSC.Pole(i), (p11.X() + p22.X()) / 2, (p11.Y() + p22.Y()) / 2);

                            if (extrloc.IsDone())
                            {
                                Dist2Min = (int)extrloc.SquareDistance();
                                if (Dist2Min < DistTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    Poles2d[i].SetCoord(u, v);
                                    myProjIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            if (!myProjIsDone)
                            {
                                break;
                            }
                        }
                        if (myProjIsDone)
                        {
                            NCollection_Array1<double> Knots = BSC.InternalKnots();
                            NCollection_Array1<int> Mults = BSC.InternalMultiplicities();
                            if (BSC.IsRational())
                            {
                                return new Geom2d_BSplineCurve(Poles2d,
                                                               BSC.InternalWeightsArray(),
                                                               Knots,
                                                               Mults,
                                                               BSC.Degree(),
                                                               BSC.IsPeriodic());
                            }
                            return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BSC.Degree(), BSC.IsPeriodic());
                        }
                    }
                    if (TheTypeC == GeomAbs_BezierCurve)
                    {
                        myTolReached = Tol3d;
                        Geom_BezierCurve BC = Curve.Bezier();
                        NCollection_Array1<gp_Pnt2d> Poles2d = new(1, Curve.NbPoles());
                        for (i = 1; i <= Curve.NbPoles(); i++)
                        {
                            Dist2Min = IntegerLast();

                            Extrema_GenLocateExtPS extrloc = new(Surf, TolU, TolV);
                            extrloc.Perform(BC.Pole(i), 0.5, 0.5);

                            if (extrloc.IsDone())
                            {
                                Dist2Min = (int)extrloc.SquareDistance();
                                if (Dist2Min < DistTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    Poles2d[i].SetCoord(u, v);
                                    myProjIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            // OCCT: as for the B-spline surface above, the curve below is never built.
                            if (myProjIsDone)
                            {
                                myProjIsDone = false;
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (myProjIsDone)
                        {
                            NCollection_Array1<double> Knots = new(1, 2);
                            Knots.SetValue(1, 0.0);
                            Knots.SetValue(2, 1.0);
                            NCollection_Array1<int> Mults = new(1, 2);
                            Mults.Init(BC.NbPoles());
                            if (BC.IsRational())
                            {
                                return new Geom2d_BSplineCurve(Poles2d,
                                                               BC.InternalWeightsArray(),
                                                               Knots,
                                                               Mults,
                                                               BC.Degree(),
                                                               BC.IsPeriodic());
                            }
                            return new Geom2d_BSplineCurve(Poles2d, Knots, Mults, BC.Degree(), BC.IsPeriodic());
                        }
                    }
                }
            }
        }

        ProjLib_PolarFunction F = new(Curve, Surf, InitCurve2d, Tol3d);

        int Deg1, Deg2;
        Deg1 = 2;
        Deg2 = 8;
        if (myDegMin > 0)
        {
            Deg1 = myDegMin;
        }
        if (myDegMax > 0)
        {
            Deg2 = myDegMax;
        }
        int aMaxSegments = 1000;
        if (myMaxSegments > 0)
        {
            aMaxSegments = myMaxSegments;
        }
        AppParCurves_Constraint aFistC = AppParCurves_TangencyPoint, aLastC = AppParCurves_TangencyPoint;
        if (myBndPnt != AppParCurves_TangencyPoint)
        {
            aFistC = myBndPnt;
            aLastC = myBndPnt;
        }

        if (myDist > 10.0 * Tol3d)
        {
            aFistC = AppParCurves_PassPoint;
            aLastC = AppParCurves_PassPoint;
        }

        Approx_FitAndDivide2d Fit = new(Deg1, Deg2, Tol3d, Tol2d, true, aFistC, aLastC);
        Fit.SetMaxSegments(aMaxSegments);
        if (InitCurve2d.GetType() == GeomAbs_Line)
        {
            Fit.SetInvOrder(false);
        }
        Fit.Perform(F);

        double anOldTol2d = Tol2d;
        double aNewTol2d = 0;
        if (Fit.IsAllApproximated())
        {
            int j;
            int NbCurves = Fit.NbMultiCurves();
            int MaxDeg = 0;
            // To transform the MultiCurve into BSpline, it is required that all
            // Bezier constituing it have the same degree -> Calculation of MaxDeg
            int NbPoles = 1;
            for (j = 1; j <= NbCurves; j++)
            {
                int Deg = Fit.Value(j).Degree();
                MaxDeg = Max(MaxDeg, Deg);
                Fit.Error(j, out Tol3d, out Tol2d);
                aNewTol2d = Max(aNewTol2d, Tol2d);
            }
            //
            myTolReached = Max(myTolReached, myTolerance * (aNewTol2d / anOldTol2d));
            //
            NbPoles = MaxDeg * NbCurves + 1; // Tops on the BSpline
            NCollection_Array1<gp_Pnt2d> Poles = new(1, NbPoles);

            NCollection_Array1<gp_Pnt2d> TempPoles = new(1, MaxDeg + 1); // to augment the degree

            NCollection_Array1<double> Knots = new(1, NbCurves + 1); // Nodes of the BSpline

            int Compt = 1;
            for (i = 1; i <= NbCurves; i++)
            {
                Fit.Parameters(i, out Knots[i], out Knots[i + 1]);
                AppParCurves_MultiCurve MC = Fit.Value(i); // Load the Ith Curve
                NCollection_Array1<gp_Pnt2d> Poles2d = new(1, MC.Degree() + 1); // Retrieve the tops
                MC.Curve(1, Poles2d);

                // Eventual augmentation of the degree
                int Inc = MaxDeg - MC.Degree();
                if (Inc > 0)
                {
                    BSplCLib.IncreaseDegree(MaxDeg,
                                            Poles2d,
                                            BSplCLib.NoWeights(),
                                            TempPoles,
                                            BSplCLib.NoWeights());
                    // update of tops of the PCurve
                    for (int k = 1; k <= MaxDeg + 1; k++)
                    {
                        Poles.SetValue(Compt, TempPoles[k]);
                        Compt++;
                    }
                }
                else
                {
                    // update of tops of the PCurve
                    for (int k = 1; k <= MaxDeg + 1; k++)
                    {
                        Poles.SetValue(Compt, Poles2d[k]);
                        Compt++;
                    }
                }

                Compt--;
            }

            // update of fields of  ProjLib_Approx
            int NbKnots = NbCurves + 1;

            NCollection_Array1<int> Mults = new(1, NbKnots);
            Mults.Init(MaxDeg);
            Mults.SetValue(1, MaxDeg + 1);
            Mults.SetValue(NbKnots, MaxDeg + 1);
            myProjIsDone = true;
            Geom2d_BSplineCurve Dummy = new(Poles, Knots, Mults, MaxDeg);

            // try to smoother the Curve GeomAbs_C1.

            bool OK = true;
            double aSmoothTol = Max(Precision.Confusion(), aNewTol2d);
            if (myBndPnt == AppParCurves_PassPoint)
            {
                aSmoothTol *= 10.0;
            }
            for (int ij = 2; ij < NbKnots; ij++)
            {
                OK = OK && Dummy.RemoveKnot(ij, MaxDeg - 1, aSmoothTol);
            }
            return Dummy;
        }
        return null;
    }

    /// <summary>The result curve, frozen (the same object at each call); null when the projection failed.</summary>
    public Geom2d_BSplineCurve? BSpline()
    {
        myBSpline?.Freeze();
        return myBSpline;
    }

    /// <summary>The result curve, unfrozen, for ported code that discards the algorithm (ProjLib_ProjectedCurve).</summary>
    internal Geom2d_BSplineCurve? InternalBSpline() => myBSpline;

    /// <summary>The second 2D curve (of the seam-edge constructor), frozen; null for the other constructors.</summary>
    /// <exception cref="Standard_NoSuchObject">The projection is not done.</exception>
    public Geom2d_Curve? Curve2d()
    {
        Standard_NoSuchObject_Raise_if(!myProjIsDone, "ProjLib_ComputeApproxOnPolarSurface:2ndCurve2d");
        my2ndCurve?.Freeze();
        return my2ndCurve;
    }

    /// <summary>The second 2D curve, unfrozen, for ported code that discards the algorithm.</summary>
    internal Geom2d_Curve? InternalCurve2d()
    {
        Standard_NoSuchObject_Raise_if(!myProjIsDone, "ProjLib_ComputeApproxOnPolarSurface:2ndCurve2d");
        return my2ndCurve;
    }

    public bool IsDone() => myProjIsDone;

    /// <summary>Sets the tolerance of the projection (default Precision::Approximation()).</summary>
    public void SetTolerance(double theTol)
    {
        myTolerance = theTol;
    }

    /// <summary>Sets the degrees of the approximation; values below 1 mean the defaults 2 and 8.</summary>
    public void SetDegree(int theDegMin, int theDegMax)
    {
        myDegMin = theDegMin;
        myDegMax = theDegMax;
    }

    /// <summary>Sets the largest number of segments of the approximation; below 1 means 1000.</summary>
    public void SetMaxSegments(int theMaxSegments)
    {
        myMaxSegments = theMaxSegments;
    }

    /// <summary>Sets the boundary condition between segments: PassPoint or TangencyPoint (the default).</summary>
    public void SetBndPnt(AppParCurves_Constraint theBndPnt)
    {
        myBndPnt = theBndPnt;
    }

    /// <summary>
    /// Sets the largest distance between the curve and a non-analytical surface; below 0 means 100 * tolerance.
    /// </summary>
    public void SetMaxDist(double theMaxDist)
    {
        myMaxDist = theMaxDist;
    }

    /// <summary>The tolerance reached.</summary>
    public double Tolerance() => myTolReached;
}
