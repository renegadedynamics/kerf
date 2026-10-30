// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx (the file-static functions)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

// OCCT's file also defines a global MakeBSpline(WL, ideb, ifin) that nothing calls (MakeCurve calls
// GeomInt_IntSS::MakeBSpline); it isn't ported.
public partial class IntTools_FaceFace
{
    private static void Parameters(GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, in gp_Pnt Ptref,
                                   out double U1, out double V1, out double U2, out double V2)
    {

        IntSurf_Quadric quad1 = new(), quad2 = new();
        GeomAbs_SurfaceType typs = HS1.GetType();

        switch (typs)
        {
            case GeomAbs_Plane:
                quad1.SetValue(HS1.Plane());
                break;
            case GeomAbs_Cylinder:
                quad1.SetValue(HS1.Cylinder());
                break;
            case GeomAbs_Cone:
                quad1.SetValue(HS1.Cone());
                break;
            case GeomAbs_Sphere:
                quad1.SetValue(HS1.Sphere());
                break;
            case GeomAbs_Torus:
                quad1.SetValue(HS1.Torus());
                break;
            default:
                throw new Standard_ConstructionError("GeomInt_IntSS::MakeCurve");
        }

        typs = HS2.GetType();
        switch (typs)
        {
            case GeomAbs_Plane:
                quad2.SetValue(HS2.Plane());
                break;
            case GeomAbs_Cylinder:
                quad2.SetValue(HS2.Cylinder());
                break;
            case GeomAbs_Cone:
                quad2.SetValue(HS2.Cone());
                break;
            case GeomAbs_Sphere:
                quad2.SetValue(HS2.Sphere());
                break;
            case GeomAbs_Torus:
                quad2.SetValue(HS2.Torus());
                break;
            default:
                throw new Standard_ConstructionError("GeomInt_IntSS::MakeCurve");
        }

        U1 = V1 = U2 = V2 = 0.0;
        quad1.Parameters(Ptref, ref U1, ref V1);
        quad2.Parameters(Ptref, ref U2, ref V2);
    }

    private static void CorrectSurfaceBoundaries(in TopoDS_Face theFace, double theTolerance, ref double theumin,
                                                 ref double theumax, ref double thevmin, ref double thevmax)
    {
        bool enlarge, isuperiodic, isvperiodic;
        double uinf, usup, vinf, vsup, delta;
        GeomAbs_SurfaceType aType;
        Geom_Surface aSurface;
        //
        aSurface = BRep_Tool.Surface(theFace)!;
        aSurface.Bounds(out uinf, out usup, out vinf, out vsup);
        delta = theTolerance;
        enlarge = false;
        //
        GeomAdaptor_Surface anAdaptorSurface = new(aSurface);
        //
        if (aSurface is Geom_RectangularTrimmedSurface aTrimmed)
        {
            Geom_Surface aBasisSurface = aTrimmed.BasisSurface();

            if (aBasisSurface is Geom_RectangularTrimmedSurface || aBasisSurface is Geom_OffsetSurface)
            {
                return;
            }
        }
        //
        if (aSurface is Geom_OffsetSurface anOffset)
        {
            Geom_Surface aBasisSurface = anOffset.BasisSurface();

            if (aBasisSurface is Geom_RectangularTrimmedSurface || aBasisSurface is Geom_OffsetSurface)
            {
                return;
            }
        }
        //
        isuperiodic = anAdaptorSurface.IsUPeriodic();
        isvperiodic = anAdaptorSurface.IsVPeriodic();
        //
        aType = anAdaptorSurface.GetType();
        if ((aType == GeomAbs_BezierSurface) || (aType == GeomAbs_BSplineSurface)
            || (aType == GeomAbs_SurfaceOfExtrusion) || (aType == GeomAbs_SurfaceOfRevolution)
            || (aType == GeomAbs_Cylinder))
        {
            enlarge = true;
        }
        //

        if (!isuperiodic && enlarge)
        {

            if (!Precision.IsInfinite(theumin) && ((theumin - uinf) > delta))
            {
                theumin -= delta;
            }
            else
            {
                theumin = uinf;
            }

            if (!Precision.IsInfinite(theumax) && ((usup - theumax) > delta))
            {
                theumax += delta;
            }
            else
            {
                theumax = usup;
            }
        }

        //
        if (!isvperiodic && enlarge)
        {
            if (!Precision.IsInfinite(thevmin) && ((thevmin - vinf) > delta))
            {
                thevmin -= delta;
            }
            else
            {
                thevmin = vinf;
            }
            if (!Precision.IsInfinite(thevmax) && ((vsup - thevmax) > delta))
            {
                thevmax += delta;
            }
            else
            {
                thevmax = vsup;
            }
        }

        //
        if (isuperiodic || isvperiodic)
        {
            bool correct = false;
            bool correctU = false;
            bool correctV = false;
            Bnd_Box2d aBox = new();
            TopExp_Explorer anExp = new();

            for (anExp.Init(theFace, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                if (BRep_Tool.IsClosed(TopoDS.Edge(anExp.Current()), theFace))
                {
                    correct = true;
                    double f, l;
                    TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

                    for (int i = 0; i < 2; i++)
                    {
                        if (i == 0)
                        {
                            anEdge.Orientation(TopAbs_FORWARD);
                        }
                        else
                        {
                            anEdge.Orientation(TopAbs_REVERSED);
                        }
                        Geom2d_Curve? aCurve = BRep_Tool.CurveOnSurface(anEdge, theFace, out f, out l);

                        if (aCurve is null)
                        {
                            correct = false;
                            break;
                        }
                        Geom2d_Line? aLine = aCurve as Geom2d_Line;

                        if (aLine is null)
                        {
                            correct = false;
                            break;
                        }
                        gp_Dir2d anUDir = new(gp_Dir2d.D.X);
                        gp_Dir2d aVDir = new(gp_Dir2d.D.Y);
                        double anAngularTolerance = Precision.Angular();

                        correctU = correctU || aLine.Position().Direction().IsParallel(aVDir, anAngularTolerance);
                        correctV = correctV || aLine.Position().Direction().IsParallel(anUDir, anAngularTolerance);

                        gp_Pnt2d pp1 = aCurve.Value(f);
                        aBox.Add(pp1);
                        gp_Pnt2d pp2 = aCurve.Value(l);
                        aBox.Add(pp2);
                    }
                    if (!correct)
                    {
                        break;
                    }
                }
            }

            if (correct)
            {
                aBox.Get(out double umin, out double vmin, out double umax, out double vmax);

                if (isuperiodic && correctU)
                {
                    if (theumin < umin)
                    {
                        theumin = umin;
                    }
                    if (theumax > umax)
                    {
                        theumax = umax;
                    }
                }
                if (isvperiodic && correctV)
                {
                    if (thevmin < vmin)
                    {
                        thevmin = vmin;
                    }
                    if (thevmax > vmax)
                    {
                        thevmax = vmax;
                    }
                }
            }
        }
    }

    // Computes a new parameter for given curve. The corresponding 2d points
    // does not lay on any boundary of given faces
    private static bool ParameterOutOfBoundary(double theParameter, Geom_Curve theCurve, in TopoDS_Face theFace1,
                                               in TopoDS_Face theFace2, double theOtherParameter, bool bIncreasePar,
                                               double theTol, out double theNewParameter, IntTools_Context aContext)
    {
        bool bIsComputed = false;
        theNewParameter = theParameter;

        double acurpar = theParameter;
        TopAbs_State aState = TopAbs_ON;
        int iter = 0;
        double asumtol = theTol;
        double adelta = asumtol * 0.1;
        adelta = (adelta < Precision.Confusion()) ? Precision.Confusion() : adelta;
        Geom_Surface aSurf1 = BRep_Tool.Surface(theFace1)!;
        Geom_Surface aSurf2 = BRep_Tool.Surface(theFace2)!;

        double u1, u2, v1, v2;

        GeomAPI_ProjectPointOnSurf aPrj1 = new();
        aSurf1.Bounds(out u1, out u2, out v1, out v2);
        aPrj1.Init(aSurf1, u1, u2, v1, v2);

        GeomAPI_ProjectPointOnSurf aPrj2 = new();
        aSurf2.Bounds(out u1, out u2, out v1, out v2);
        aPrj2.Init(aSurf2, u1, u2, v1, v2);

        while (aState == TopAbs_ON)
        {
            if (bIncreasePar)
            {
                acurpar += adelta;
            }
            else
            {
                acurpar -= adelta;
            }
            gp_Pnt aPCurrent = theCurve.Value(acurpar);
            aPrj1.Perform(aPCurrent);
            double U = 0.0, V = 0.0;

            if (aPrj1.IsDone())
            {
                aPrj1.LowerDistanceParameters(out U, out V);
                aState = aContext.StatePointFace(theFace1, new gp_Pnt2d(U, V));
            }

            if (aState != TopAbs_ON)
            {
                aPrj2.Perform(aPCurrent);

                if (aPrj2.IsDone())
                {
                    aPrj2.LowerDistanceParameters(out U, out V);
                    aState = aContext.StatePointFace(theFace2, new gp_Pnt2d(U, V));
                }
            }

            if (iter > 11)
            {
                break;
            }
            iter++;
        }

        if (iter <= 11)
        {
            theNewParameter = acurpar;
            bIsComputed = true;

            if (bIncreasePar)
            {
                if (acurpar >= theOtherParameter)
                {
                    theNewParameter = theOtherParameter;
                }
            }
            else
            {
                if (acurpar <= theOtherParameter)
                {
                    theNewParameter = theOtherParameter;
                }
            }
        }
        return bIsComputed;
    }

    private static bool IsCurveValid(Geom2d_Curve? thePCurve)
    {
        if (thePCurve is null)
        {
            return false;
        }

        double tolint = 1.0e-10;
        Geom2dAdaptor_Curve PCA = new();
        IntRes2d_Domain PCD = new();
        Geom2dInt_GInter PCI = new();

        double pf = 0.0, pl = 0.0;
        gp_Pnt2d pntf, pntl;

        if (!thePCurve.IsClosed() && !thePCurve.IsPeriodic())
        {
            pf = thePCurve.FirstParameter();
            pl = thePCurve.LastParameter();
            pntf = thePCurve.Value(pf);
            pntl = thePCurve.Value(pl);
            PCA.Load(thePCurve);
            if (!PCA.IsPeriodic())
            {
                if (PCA.FirstParameter() > pf)
                {
                    pf = PCA.FirstParameter();
                }
                if (PCA.LastParameter() < pl)
                {
                    pl = PCA.LastParameter();
                }
            }
            PCD.SetValues(pntf, pf, tolint, pntl, pl, tolint);
            PCI.Perform(PCA, PCD, tolint, tolint);
            if (PCI.IsDone())
            {
                if (PCI.NbPoints() > 0)
                {
                    return false;
                }
            }
        }

        return true;
    }

    private static bool ApproxWithPCurves(in gp_Cylinder theCyl, in gp_Sphere theSph)
    {
        bool bRes = true;
        double R1 = theCyl.Radius(), R2 = theSph.Radius();
        //
        {
            double aD2, aRc2, aEps;
            gp_Pnt aApexSph = new();
            //
            aEps = 1.0E-7;
            aRc2 = R1 * R1;
            //
            gp_Ax3 aAx3Sph = theSph.Position();
            gp_Pnt aLocSph = aAx3Sph.Location();
            gp_Dir aDirSph = aAx3Sph.Direction();
            //
            gp_Ax1 aAx1Cyl = theCyl.Axis();
            gp_Lin aLinCyl = new(aAx1Cyl);
            //
            aApexSph.SetXYZ(aLocSph.XYZ() + R2 * aDirSph.XYZ());
            aD2 = aLinCyl.SquareDistance(aApexSph);
            if (Math.Abs(aD2 - aRc2) < aEps)
            {
                return !bRes;
            }
            //
            aApexSph.SetXYZ(aLocSph.XYZ() - R2 * aDirSph.XYZ());
            aD2 = aLinCyl.SquareDistance(aApexSph);
            if (Math.Abs(aD2 - aRc2) < aEps)
            {
                return !bRes;
            }
        }
        //

        if (R1 < 2.0 * R2)
        {
            return bRes;
        }
        gp_Lin anCylAx = new(theCyl.Axis());

        double aDist = anCylAx.Distance(theSph.Location());
        double aDRel = Math.Abs(aDist - R1) / R2;

        if (aDRel > 0.2)
        {
            return bRes;
        }

        double par = ElCLib.Parameter(anCylAx, theSph.Location());
        gp_Pnt aP = ElCLib.Value(par, anCylAx);
        gp_Vec aV = new(aP, theSph.Location());

        double dd = aV.Dot(new gp_Vec(theSph.Position().XDirection()));

        if (aDist < R1 && dd > 0.0)
        {
            return false;
        }
        if (aDist > R1 && dd < 0.0)
        {
            return false;
        }

        return bRes;
    }

    private static void PerformPlanes(GeomAdaptor_Surface theS1, GeomAdaptor_Surface theS2, double TolF1,
                                      double TolF2, double TolAng, double TolTang, bool theApprox1, bool theApprox2,
                                      NCollection_Sequence<IntTools_Curve> theSeqOfCurve, out bool theTangentFaces)
    {

        gp_Pln aPln1 = theS1.Plane();
        gp_Pln aPln2 = theS2.Plane();

        IntAna_QuadQuadGeo aPlnInter = new(aPln1, aPln2, TolAng, TolTang);

        if (!aPlnInter.IsDone())
        {
            theTangentFaces = false;
            return;
        }

        IntAna_ResultType aResType = aPlnInter.TypeInter();

        if (aResType == IntAna_ResultType.IntAna_Same)
        {
            theTangentFaces = true;
            return;
        }

        theTangentFaces = false;

        if (aResType == IntAna_ResultType.IntAna_Empty)
        {
            return;
        }

        gp_Lin aLin = aPlnInter.Line(1);

        ProjLib_Plane aProj = new();

        aProj.Init(aPln1);
        aProj.Project(aLin);
        gp_Lin2d aLin2d1 = aProj.Line();
        //
        aProj.Init(aPln2);
        aProj.Project(aLin);
        gp_Lin2d aLin2d2 = aProj.Line();
        //
        // classify line2d1 relatively first plane
        double P11, P12;
        bool IsCrossed = ClassifyLin2d(theS1, aLin2d1, TolTang, out P11, out P12);
        if (!IsCrossed)
        {
            return;
        }
        // classify line2d2 relatively second plane
        double P21, P22;
        IsCrossed = ClassifyLin2d(theS2, aLin2d2, TolTang, out P21, out P22);
        if (!IsCrossed)
        {
            return;
        }

        // Analysis of parametric intervals: must have common part

        if (P21 >= P12)
        {
            return;
        }
        if (P22 <= P11)
        {
            return;
        }

        double pmin, pmax;
        pmin = Max(P11, P21);
        pmax = Min(P12, P22);

        if (pmax - pmin <= TolTang)
        {
            return;
        }

        Geom_Line aGLin = new(aLin);

        IntTools_Curve aCurve = new();
        // OCCT: on unbounded planar faces (no wires) pmin and pmax come from the infinite bounds and the trimming
        // throws Standard_ConstructionError
        Geom_TrimmedCurve aGTLin = new(aGLin, pmin, pmax);

        aCurve.SetCurve(aGTLin);

        if (theApprox1)
        {
            Geom2d_Line C2d = new(aLin2d1);
            aCurve.SetFirstCurve2d(new Geom2d_TrimmedCurve(C2d, pmin, pmax));
        }
        else
        {
            Geom2d_Curve? H1 = null;
            aCurve.SetFirstCurve2d(H1);
        }
        if (theApprox2)
        {
            Geom2d_Line C2d = new(aLin2d2);
            aCurve.SetSecondCurve2d(new Geom2d_TrimmedCurve(C2d, pmin, pmax));
        }
        else
        {
            // OCCT clears the first 2D curve here, not the second: without a second 2D curve the first is lost
            Geom2d_Curve? H1 = null;
            aCurve.SetFirstCurve2d(H1);
        }
        //
        // Valid tolerance for the intersection curve between planar faces
        // is the maximal tolerance between tolerances of faces
        double aTolC = Max(TolF1, TolF2);
        aCurve.SetTolerance(aTolC);
        //
        // Computation of the tangential tolerance
        double anAngle, aDt;
        gp_Dir aD1, aD2;
        //
        aD1 = aPln1.Position().Direction();
        aD2 = aPln2.Position().Direction();
        anAngle = aD1.Angle(aD2);
        //
        aDt = IntTools_Tools.ComputeIntRange(TolF1, TolF2, anAngle);
        double aTangTol = Math.Sqrt(aDt * aDt + TolF1 * TolF1);
        //
        aCurve.SetTangentialTolerance(aTangTol);
        //
        theSeqOfCurve.Append(aCurve);
    }

    private static bool INTER(double d1, double d2, double tol)
        => (d1 > tol && d2 < -tol) || (d1 < -tol && d2 > tol)
           || ((d1 <= tol && d1 >= -tol) && (d2 > tol || d2 < -tol))
           || ((d2 <= tol && d2 >= -tol) && (d1 > tol || d1 < -tol));

    private static bool COINC(double d1, double d2, double tol)
        => (d1 <= tol && d1 >= -tol) && (d2 <= tol && d2 >= -tol);

    private static bool ClassifyLin2d(GeomAdaptor_Surface theS, in gp_Lin2d theLin2d, double theTol,
                                      out double theP1, out double theP2)
    {
        double xmin, xmax, ymin, ymax, d1, d2, A, B, C;
        Span<double> par = stackalloc double[2];
        int nbi = 0;

        // OCCT leaves theP1 and theP2 unwritten when the line misses the domain; its callers return then.
        theP1 = theP2 = 0.0;

        xmin = theS.FirstUParameter();
        xmax = theS.LastUParameter();
        ymin = theS.FirstVParameter();
        ymax = theS.LastVParameter();

        theLin2d.Coefficients(out A, out B, out C);

        // xmin, ymin <-> xmin, ymax
        d1 = A * xmin + B * ymin + C;
        d2 = A * xmin + B * ymax + C;

        if (INTER(d1, d2, theTol))
        {
            // Intersection with boundary
            double y = -(C + A * xmin) / B;
            par[nbi] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmin, y));
            nbi++;
        }
        else if (COINC(d1, d2, theTol))
        {
            // Coincidence with boundary
            par[0] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmin, ymin));
            par[1] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmin, ymax));
            nbi = 2;
        }

        if (nbi == 2)
        {

            if (Math.Abs(par[0] - par[1]) > theTol)
            {
                theP1 = Min(par[0], par[1]);
                theP2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmin, ymax <-> xmax, ymax
        d1 = d2;
        d2 = A * xmax + B * ymax + C;

        if (d1 > theTol || d1 < -theTol)
        { // to avoid checking of
            // coincidence with the same point
            if (INTER(d1, d2, theTol))
            {
                double x = -(C + B * ymax) / A;
                par[nbi] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(x, ymax));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmin, ymax));
                par[1] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmax, ymax));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {

            if (Math.Abs(par[0] - par[1]) > theTol)
            {
                theP1 = Min(par[0], par[1]);
                theP2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmax, ymax <-> xmax, ymin
        d1 = d2;
        d2 = A * xmax + B * ymin + C;

        if (d1 > theTol || d1 < -theTol)
        {
            if (INTER(d1, d2, theTol))
            {
                double y = -(C + A * xmax) / B;
                par[nbi] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmax, y));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmax, ymax));
                par[1] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmax, ymin));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {
            if (Math.Abs(par[0] - par[1]) > theTol)
            {
                theP1 = Min(par[0], par[1]);
                theP2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmax, ymin <-> xmin, ymin
        d1 = d2;
        d2 = A * xmin + B * ymin + C;

        if (d1 > theTol || d1 < -theTol)
        {
            if (INTER(d1, d2, theTol))
            {
                double x = -(C + B * ymin) / A;
                par[nbi] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(x, ymin));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmax, ymin));
                par[1] = ElCLib.Parameter(theLin2d, new gp_Pnt2d(xmin, ymin));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {
            if (Math.Abs(par[0] - par[1]) > theTol)
            {
                theP1 = Min(par[0], par[1]);
                theP2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    //
    private static void ApproxParameters(GeomAdaptor_Surface aHS1, GeomAdaptor_Surface aHS2, out int iDegMin,
                                         out int iDegMax, out int iNbIter)

    {
        GeomAbs_SurfaceType aTS1, aTS2;

        //
        iNbIter = 0;
        iDegMin = 4;
        iDegMax = 8;
        //
        aTS1 = aHS1.GetType();
        aTS2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((aTS1 == GeomAbs_Cylinder && aTS2 == GeomAbs_Torus) || (aTS2 == GeomAbs_Cylinder && aTS1 == GeomAbs_Torus))
        {
            double aRC, aRT, dR, aPC;
            gp_Cylinder aCylinder;
            gp_Torus aTorus;
            //
            aPC = Precision.Confusion();
            //
            aCylinder = (aTS1 == GeomAbs_Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            aTorus = (aTS1 == GeomAbs_Torus) ? aHS1.Torus() : aHS2.Torus();
            //
            aRC = aCylinder.Radius();
            aRT = aTorus.MinorRadius();
            dR = aRC - aRT;
            if (dR < 0.0)
            {
                dR = -dR;
            }
            //
            if (dR < aPC)
            {
                iDegMax = 6;
            }
        }
        if (aTS1 == GeomAbs_Cylinder && aTS2 == GeomAbs_Cylinder)
        {
            iNbIter = 1;
        }
    }

    private static void Tolerances(GeomAdaptor_Surface aHS1, GeomAdaptor_Surface aHS2, ref double aTolTang)
    {
        GeomAbs_SurfaceType aTS1, aTS2;
        //
        aTS1 = aHS1.GetType();
        aTS2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((aTS1 == GeomAbs_Cylinder && aTS2 == GeomAbs_Torus) || (aTS2 == GeomAbs_Cylinder && aTS1 == GeomAbs_Torus))
        {
            double aRC, aRT, dR, aPC;
            gp_Cylinder aCylinder;
            gp_Torus aTorus;
            //
            aPC = Precision.Confusion();
            //
            aCylinder = (aTS1 == GeomAbs_Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            aTorus = (aTS1 == GeomAbs_Torus) ? aHS1.Torus() : aHS2.Torus();
            //
            aRC = aCylinder.Radius();
            aRT = aTorus.MinorRadius();
            dR = aRC - aRT;
            if (dR < 0.0)
            {
                dR = -dR;
            }
            //
            if (dR < aPC)
            {
                aTolTang = 0.1 * aTolTang;
            }
        }
    }

    private static bool SortTypes(GeomAbs_SurfaceType aType1, GeomAbs_SurfaceType aType2)
    {
        bool bRet;
        int aI1, aI2;
        //
        bRet = false;
        //
        aI1 = IndexType(aType1);
        aI2 = IndexType(aType2);
        if (aI1 < aI2)
        {
            bRet = !bRet;
        }
        return bRet;
    }

    private static int IndexType(GeomAbs_SurfaceType aType)
    {
        int aIndex;
        //
        aIndex = 11;
        //
        if (aType == GeomAbs_Plane)
        {
            aIndex = 0;
        }
        else if (aType == GeomAbs_Cylinder)
        {
            aIndex = 1;
        }
        else if (aType == GeomAbs_Cone)
        {
            aIndex = 2;
        }
        else if (aType == GeomAbs_Sphere)
        {
            aIndex = 3;
        }
        else if (aType == GeomAbs_Torus)
        {
            aIndex = 4;
        }
        else if (aType == GeomAbs_BezierSurface)
        {
            aIndex = 5;
        }
        else if (aType == GeomAbs_BSplineSurface)
        {
            aIndex = 6;
        }
        else if (aType == GeomAbs_SurfaceOfRevolution)
        {
            aIndex = 7;
        }
        else if (aType == GeomAbs_SurfaceOfExtrusion)
        {
            aIndex = 8;
        }
        else if (aType == GeomAbs_OffsetSurface)
        {
            aIndex = 9;
        }
        else if (aType == GeomAbs_OtherSurface)
        {
            aIndex = 10;
        }
        return aIndex;
    }

    private static double FindMaxDistance(Geom_Curve theCurve, double theFirst, double theLast, in TopoDS_Face theFace,
                                          IntTools_Context theContext)
    {
        int aNbS;
        double aT1, aT2, aDt, aD, aDMax, anEps;
        //
        aNbS = 11;
        aDt = (theLast - theFirst) / aNbS;
        aDMax = 0.0;
        anEps = 1.0e-4 * aDt;
        //
        GeomAPI_ProjectPointOnSurf aProjPS = theContext.ProjPS(theFace);
        aT2 = theFirst;
        for (;;)
        {
            aT1 = aT2;
            aT2 += aDt;
            //
            if (aT2 > theLast)
            {
                break;
            }
            //
            aD = FindMaxDistance(theCurve, aT1, aT2, aProjPS, anEps);
            if (aD > aDMax)
            {
                aDMax = aD;
            }
        }
        //
        return aDMax;
    }

    private static double FindMaxDistance(Geom_Curve theC, double theFirst, double theLast,
                                          GeomAPI_ProjectPointOnSurf theProjPS, double theEps)
    {
        double aA, aB, aCf, aX, aX1, aX2, aF1, aF2, aF;
        //
        aCf = 0.61803398874989484820458683436564; //(sqrt(5.)-1)/2.;
        aA = theFirst;
        aB = theLast;
        //
        aX1 = aB - aCf * (aB - aA);
        aF1 = MaxDistance(theC, aX1, theProjPS);
        aX2 = aA + aCf * (aB - aA);
        aF2 = MaxDistance(theC, aX2, theProjPS);

        while (Math.Abs(aX1 - aX2) > theEps)
        {
            if (aF1 > aF2)
            {
                aB = aX2;
                aX2 = aX1;
                aF2 = aF1;
                aX1 = aB - aCf * (aB - aA);
                aF1 = MaxDistance(theC, aX1, theProjPS);
            }
            else
            {
                aA = aX1;
                aX1 = aX2;
                aF1 = aF2;
                aX2 = aA + aCf * (aB - aA);
                aF2 = MaxDistance(theC, aX2, theProjPS);
            }
        }
        //
        aX = 0.5 * (aA + aB);
        aF = MaxDistance(theC, aX, theProjPS);
        //
        if (aF1 > aF)
        {
            aF = aF1;
        }
        //
        if (aF2 > aF)
        {
            aF = aF2;
        }
        //
        return aF;
    }

    private static double MaxDistance(Geom_Curve theC, double aT, GeomAPI_ProjectPointOnSurf theProjPS)
    {
        double aD;
        gp_Pnt aP;
        //
        theC.D0(aT, out aP);
        theProjPS.Perform(aP);
        aD = theProjPS.NbPoints() != 0 ? theProjPS.LowerDistance() : 0.0;
        //
        return aD;
    }

    // Checks if points of the pcurve are out of the face bounds.
    private static bool CheckPCurve(Geom2d_Curve aPC, in TopoDS_Face aFace, IntTools_Context theCtx)
    {
        const int NPoints = 23;
        int i;
        double umin, umax, vmin, vmax;

        theCtx.UVBounds(aFace, out umin, out umax, out vmin, out vmax);
        double tolU = Max((umax - umin) * 0.01, Precision.Confusion());
        double tolV = Max((vmax - vmin) * 0.01, Precision.Confusion());
        double fp = aPC.FirstParameter();
        double lp = aPC.LastParameter();

        // adjust domain for periodic surfaces
        Geom_Surface aSurf = BRep_Tool.Surface(aFace, out _)!;
        if (aSurf is Geom_RectangularTrimmedSurface aTrimmed)
        {
            aSurf = aTrimmed.BasisSurface();
        }
        gp_Pnt2d pnt = aPC.Value((fp + lp) / 2);
        double u, v;
        pnt.Coord(out u, out v);
        //
        if (aSurf.IsUPeriodic())
        {
            double aPer = aSurf.UPeriod();
            int nshift = (int)((u - umin) / aPer);
            if (u < umin + aPer * nshift)
            {
                nshift--;
            }
            umin += aPer * nshift;
            umax += aPer * nshift;
        }
        if (aSurf.IsVPeriodic())
        {
            double aPer = aSurf.VPeriod();
            int nshift = (int)((v - vmin) / aPer);
            if (v < vmin + aPer * nshift)
            {
                nshift--;
            }
            vmin += aPer * nshift;
            vmax += aPer * nshift;
        }
        //
        //--------------------------------------------------------
        bool bRet;
        int j, aNbIntervals;
        double aT, dT;
        gp_Pnt2d aP2D;
        //
        Geom2dAdaptor_Curve aGAC = new(aPC);
        aNbIntervals = aGAC.NbIntervals(GeomAbs_CN);
        //
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        aGAC.Intervals(aTI, GeomAbs_CN);
        //
        bRet = false;
        //
        // OCCT steps (NPoints - 1) * dT per interval from the first parameter on: with several intervals the samples
        // drift from the intervals and never reach the last one's end
        aT = aGAC.FirstParameter();
        for (j = 1; j <= aNbIntervals; ++j)
        {
            dT = (aTI[j + 1] - aTI[j]) / NPoints;
            //
            for (i = 1; i < NPoints; i++)
            {
                aT = aT + dT;
                aGAC.D0(aT, out aP2D);
                aP2D.Coord(out u, out v);
                if (umin - u > tolU || u - umax > tolU || vmin - v > tolV || v - vmax > tolV)
                {
                    return bRet;
                }
            }
        }
        return !bRet;
    }

    private static void CorrectPlaneBoundaries(ref double aUmin, ref double aUmax, ref double aVmin, ref double aVmax)
    {
        if (!(Precision.IsInfinite(aUmin) || Precision.IsInfinite(aUmax)))
        {
            double dU;
            //
            dU = 0.1 * (aUmax - aUmin);
            aUmin = aUmin - dU;
            aUmax = aUmax + dU;
        }
        if (!(Precision.IsInfinite(aVmin) || Precision.IsInfinite(aVmax)))
        {
            double dV;
            //
            dV = 0.1 * (aVmax - aVmin);
            aVmin = aVmin - dV;
            aVmax = aVmax + dV;
        }
    }
}
