// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx (the file-static functions)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

// OCCT's file also defines a global MakeBSpline(WL, ideb, ifin) that nothing calls (MakeCurve calls
// GeomInt_IntSS::MakeBSpline); it isn't ported.
public partial class IntTools_FaceFace
{
    private static void Parameters(SurfaceAdaptor HS1, SurfaceAdaptor HS2, in Point3D Ptref,
                                   out double U1, out double V1, out double U2, out double V2)
    {

        IntSurf_Quadric quad1 = new(), quad2 = new();
        SurfaceKind typs = HS1.GetType();

        switch (typs)
        {
            case Plane:
                quad1.SetValue(HS1.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad1.SetValue(HS1.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad1.SetValue(HS1.Cone());
                break;
            case SurfaceKind.Sphere:
                quad1.SetValue(HS1.Sphere());
                break;
            case SurfaceKind.Torus:
                quad1.SetValue(HS1.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_IntSS::MakeCurve");
        }

        typs = HS2.GetType();
        switch (typs)
        {
            case Plane:
                quad2.SetValue(HS2.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad2.SetValue(HS2.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad2.SetValue(HS2.Cone());
                break;
            case SurfaceKind.Sphere:
                quad2.SetValue(HS2.Sphere());
                break;
            case SurfaceKind.Torus:
                quad2.SetValue(HS2.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_IntSS::MakeCurve");
        }

        U1 = V1 = U2 = V2 = 0.0;
        quad1.Parameters(Ptref, ref U1, ref V1);
        quad2.Parameters(Ptref, ref U2, ref V2);
    }

    private static void CorrectSurfaceBoundaries(in Face theFace, double theTolerance, ref double theumin,
                                                 ref double theumax, ref double thevmin, ref double thevmax)
    {
        bool enlarge, isuperiodic, isvperiodic;
        double uinf, usup, vinf, vsup, delta;
        SurfaceKind aType;
        Surface aSurface;
        //
        aSurface = Tool.Surface(theFace)!;
        aSurface.Bounds(out uinf, out usup, out vinf, out vsup);
        delta = theTolerance;
        enlarge = false;
        //
        SurfaceAdaptor anAdaptorSurface = new(aSurface);
        //
        if (aSurface is TrimmedSurface aTrimmed)
        {
            Surface aBasisSurface = aTrimmed.BasisSurface();

            if (aBasisSurface is TrimmedSurface || aBasisSurface is global::Kerf.Geometry.OffsetSurface)
            {
                return;
            }
        }
        //
        if (aSurface is global::Kerf.Geometry.OffsetSurface anOffset)
        {
            Surface aBasisSurface = anOffset.BasisSurface();

            if (aBasisSurface is TrimmedSurface || aBasisSurface is global::Kerf.Geometry.OffsetSurface)
            {
                return;
            }
        }
        //
        isuperiodic = anAdaptorSurface.IsUPeriodic;
        isvperiodic = anAdaptorSurface.IsVPeriodic;
        //
        aType = anAdaptorSurface.GetType();
        if ((aType == SurfaceKind.BezierSurface) || (aType == SurfaceKind.BSplineSurface)
            || (aType == SurfaceOfExtrusion) || (aType == SurfaceOfRevolution)
            || (aType == SurfaceKind.Cylinder))
        {
            enlarge = true;
        }
        //

        if (!isuperiodic && enlarge)
        {

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theumin) && ((theumin - uinf) > delta))
            {
                theumin -= delta;
            }
            else
            {
                theumin = uinf;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theumax) && ((usup - theumax) > delta))
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
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(thevmin) && ((thevmin - vinf) > delta))
            {
                thevmin -= delta;
            }
            else
            {
                thevmin = vinf;
            }
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(thevmax) && ((vsup - thevmax) > delta))
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
            Box2d aBox = new();
            Explorer anExp = new();

            for (anExp.Init(theFace, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                if (Tool.IsClosed(ShapeCasts.Edge(anExp.Current()), theFace))
                {
                    correct = true;
                    double f, l;
                    Edge anEdge = ShapeCasts.Edge(anExp.Current());

                    for (int i = 0; i < 2; i++)
                    {
                        if (i == 0)
                        {
                            anEdge.Orientation(Forward);
                        }
                        else
                        {
                            anEdge.Orientation(Reversed);
                        }
                        Curve2D? aCurve = Tool.CurveOnSurface(anEdge, theFace, out f, out l);

                        if (aCurve is null)
                        {
                            correct = false;
                            break;
                        }
                        LineCurve2D? aLine = aCurve as LineCurve2D;

                        if (aLine is null)
                        {
                            correct = false;
                            break;
                        }
                        Direction2D anUDir = new(Direction2D.D.X);
                        Direction2D aVDir = new(Direction2D.D.Y);
                        double anAngularTolerance = global::Kerf.Engine.Foundation.Precision.Angular();

                        correctU = correctU || aLine.Position().Direction().IsParallel(aVDir, anAngularTolerance);
                        correctV = correctV || aLine.Position().Direction().IsParallel(anUDir, anAngularTolerance);

                        Point2D pp1 = aCurve.Value(f);
                        aBox.Add(pp1);
                        Point2D pp2 = aCurve.Value(l);
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
    private static bool ParameterOutOfBoundary(double theParameter, Curve3D theCurve, in Face theFace1,
                                               in Face theFace2, double theOtherParameter, bool bIncreasePar,
                                               double theTol, out double theNewParameter, IntTools_Context aContext)
    {
        bool bIsComputed = false;
        theNewParameter = theParameter;

        double acurpar = theParameter;
        Containment aState = On;
        int iter = 0;
        double asumtol = theTol;
        double adelta = asumtol * 0.1;
        adelta = (adelta < global::Kerf.Engine.Foundation.Precision.Confusion()) ? global::Kerf.Engine.Foundation.Precision.Confusion() : adelta;
        Surface aSurf1 = Tool.Surface(theFace1)!;
        Surface aSurf2 = Tool.Surface(theFace2)!;

        double u1, u2, v1, v2;

        GeomAPI_ProjectPointOnSurf aPrj1 = new();
        aSurf1.Bounds(out u1, out u2, out v1, out v2);
        aPrj1.Init(aSurf1, u1, u2, v1, v2);

        GeomAPI_ProjectPointOnSurf aPrj2 = new();
        aSurf2.Bounds(out u1, out u2, out v1, out v2);
        aPrj2.Init(aSurf2, u1, u2, v1, v2);

        while (aState == On)
        {
            if (bIncreasePar)
            {
                acurpar += adelta;
            }
            else
            {
                acurpar -= adelta;
            }
            Point3D aPCurrent = theCurve.Value(acurpar);
            aPrj1.Perform(aPCurrent);
            double U = 0.0, V = 0.0;

            if (aPrj1.IsDone())
            {
                aPrj1.LowerDistanceParameters(out U, out V);
                aState = aContext.StatePointFace(theFace1, new Point2D(U, V));
            }

            if (aState != On)
            {
                aPrj2.Perform(aPCurrent);

                if (aPrj2.IsDone())
                {
                    aPrj2.LowerDistanceParameters(out U, out V);
                    aState = aContext.StatePointFace(theFace2, new Point2D(U, V));
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

    private static bool IsCurveValid(Curve2D? thePCurve)
    {
        if (thePCurve is null)
        {
            return false;
        }

        double tolint = 1.0e-10;
        CurveAdaptor2D PCA = new();
        IntRes2d_Domain PCD = new();
        Geom2dInt_GInter PCI = new();

        double pf = 0.0, pl = 0.0;
        Point2D pntf, pntl;

        if (!thePCurve.IsClosed && !thePCurve.IsPeriodic)
        {
            pf = thePCurve.FirstParameter();
            pl = thePCurve.LastParameter();
            pntf = thePCurve.Value(pf);
            pntl = thePCurve.Value(pl);
            PCA.Load(thePCurve);
            if (!PCA.IsPeriodic)
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

    private static bool ApproxWithPCurves(in Cylinder theCyl, in Sphere theSph)
    {
        bool bRes = true;
        double R1 = theCyl.Radius(), R2 = theSph.Radius();
        //
        {
            double aD2, aRc2, aEps;
            Point3D aApexSph = new();
            //
            aEps = 1.0E-7;
            aRc2 = R1 * R1;
            //
            CoordinateSystem aAx3Sph = theSph.Position();
            Point3D aLocSph = aAx3Sph.Location();
            Direction3D aDirSph = aAx3Sph.Direction();
            //
            Axis3D aAx1Cyl = theCyl.Axis();
            Line3D aLinCyl = new(aAx1Cyl);
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
        Line3D anCylAx = new(theCyl.Axis());

        double aDist = anCylAx.Distance(theSph.Location());
        double aDRel = Math.Abs(aDist - R1) / R2;

        if (aDRel > 0.2)
        {
            return bRes;
        }

        double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anCylAx, theSph.Location());
        Point3D aP = global::Kerf.Engine.Geometry.ElCLib.Value(par, anCylAx);
        Vector3D aV = new(aP, theSph.Location());

        double dd = aV.Dot(new Vector3D(theSph.Position().XDirection()));

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

    private static void PerformPlanes(SurfaceAdaptor theS1, SurfaceAdaptor theS2, double TolF1,
                                      double TolF2, double TolAng, double TolTang, bool theApprox1, bool theApprox2,
                                      NCollection_Sequence<IntTools_Curve> theSeqOfCurve, out bool theTangentFaces)
    {

        Plane3D aPln1 = theS1.Plane();
        Plane3D aPln2 = theS2.Plane();

        QuadQuadGeo aPlnInter = new(aPln1, aPln2, TolAng, TolTang);

        if (!aPlnInter.IsDone)
        {
            theTangentFaces = false;
            return;
        }

        ResultType aResType = aPlnInter.TypeInter();

        if (aResType == ResultType.Same)
        {
            theTangentFaces = true;
            return;
        }

        theTangentFaces = false;

        if (aResType == ResultType.Empty)
        {
            return;
        }

        Line3D aLin = aPlnInter.Line(1);

        PlaneProjector aProj = new();

        aProj.Init(aPln1);
        aProj.Project(aLin);
        Line2D aLin2d1 = aProj.Line();
        //
        aProj.Init(aPln2);
        aProj.Project(aLin);
        Line2D aLin2d2 = aProj.Line();
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

        LineCurve3D aGLin = new(aLin);

        IntTools_Curve aCurve = new();
        // OCCT: on unbounded planar faces (no wires) pmin and pmax come from the infinite bounds and the trimming
        // throws Standard_ConstructionError
        TrimmedCurve3D aGTLin = new(aGLin, pmin, pmax);

        aCurve.SetCurve(aGTLin);

        if (theApprox1)
        {
            LineCurve2D C2d = new(aLin2d1);
            aCurve.SetFirstCurve2d(new TrimmedCurve2D(C2d, pmin, pmax));
        }
        else
        {
            Curve2D? H1 = null;
            aCurve.SetFirstCurve2d(H1);
        }
        if (theApprox2)
        {
            LineCurve2D C2d = new(aLin2d2);
            aCurve.SetSecondCurve2d(new TrimmedCurve2D(C2d, pmin, pmax));
        }
        else
        {
            // OCCT clears the first 2D curve here, not the second: without a second 2D curve the first is lost
            Curve2D? H1 = null;
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
        Direction3D aD1, aD2;
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

    private static bool ClassifyLin2d(SurfaceAdaptor theS, in Line2D theLin2d, double theTol,
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
            par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmin, y));
            nbi++;
        }
        else if (COINC(d1, d2, theTol))
        {
            // Coincidence with boundary
            par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmin, ymin));
            par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmin, ymax));
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
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(x, ymax));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmin, ymax));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmax, ymax));
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
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmax, y));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmax, ymax));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmax, ymin));
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
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(x, ymin));
                nbi++;
            }
            else if (COINC(d1, d2, theTol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmax, ymin));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin2d, new Point2D(xmin, ymin));
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
    private static void ApproxParameters(SurfaceAdaptor aHS1, SurfaceAdaptor aHS2, out int iDegMin,
                                         out int iDegMax, out int iNbIter)

    {
        SurfaceKind aTS1, aTS2;

        //
        iNbIter = 0;
        iDegMin = 4;
        iDegMax = 8;
        //
        aTS1 = aHS1.GetType();
        aTS2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((aTS1 == SurfaceKind.Cylinder && aTS2 == SurfaceKind.Torus) || (aTS2 == SurfaceKind.Cylinder && aTS1 == SurfaceKind.Torus))
        {
            double aRC, aRT, dR, aPC;
            Cylinder aCylinder;
            Torus aTorus;
            //
            aPC = global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            aCylinder = (aTS1 == SurfaceKind.Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            aTorus = (aTS1 == SurfaceKind.Torus) ? aHS1.Torus() : aHS2.Torus();
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
        if (aTS1 == SurfaceKind.Cylinder && aTS2 == SurfaceKind.Cylinder)
        {
            iNbIter = 1;
        }
    }

    private static void Tolerances(SurfaceAdaptor aHS1, SurfaceAdaptor aHS2, ref double aTolTang)
    {
        SurfaceKind aTS1, aTS2;
        //
        aTS1 = aHS1.GetType();
        aTS2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((aTS1 == SurfaceKind.Cylinder && aTS2 == SurfaceKind.Torus) || (aTS2 == SurfaceKind.Cylinder && aTS1 == SurfaceKind.Torus))
        {
            double aRC, aRT, dR, aPC;
            Cylinder aCylinder;
            Torus aTorus;
            //
            aPC = global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            aCylinder = (aTS1 == SurfaceKind.Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            aTorus = (aTS1 == SurfaceKind.Torus) ? aHS1.Torus() : aHS2.Torus();
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

    private static bool SortTypes(SurfaceKind aType1, SurfaceKind aType2)
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

    private static int IndexType(SurfaceKind aType)
    {
        int aIndex;
        //
        aIndex = 11;
        //
        if (aType == Plane)
        {
            aIndex = 0;
        }
        else if (aType == SurfaceKind.Cylinder)
        {
            aIndex = 1;
        }
        else if (aType == SurfaceKind.Cone)
        {
            aIndex = 2;
        }
        else if (aType == SurfaceKind.Sphere)
        {
            aIndex = 3;
        }
        else if (aType == SurfaceKind.Torus)
        {
            aIndex = 4;
        }
        else if (aType == SurfaceKind.BezierSurface)
        {
            aIndex = 5;
        }
        else if (aType == SurfaceKind.BSplineSurface)
        {
            aIndex = 6;
        }
        else if (aType == SurfaceOfRevolution)
        {
            aIndex = 7;
        }
        else if (aType == SurfaceOfExtrusion)
        {
            aIndex = 8;
        }
        else if (aType == SurfaceKind.OffsetSurface)
        {
            aIndex = 9;
        }
        else if (aType == OtherSurface)
        {
            aIndex = 10;
        }
        return aIndex;
    }

    private static double FindMaxDistance(Curve3D theCurve, double theFirst, double theLast, in Face theFace,
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

    private static double FindMaxDistance(Curve3D theC, double theFirst, double theLast,
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

    private static double MaxDistance(Curve3D theC, double aT, GeomAPI_ProjectPointOnSurf theProjPS)
    {
        double aD;
        Point3D aP;
        //
        theC.D0(aT, out aP);
        theProjPS.Perform(aP);
        aD = theProjPS.NbPoints() != 0 ? theProjPS.LowerDistance() : 0.0;
        //
        return aD;
    }

    // Checks if points of the pcurve are out of the face bounds.
    private static bool CheckPCurve(Curve2D aPC, in Face aFace, IntTools_Context theCtx)
    {
        const int NPoints = 23;
        int i;
        double umin, umax, vmin, vmax;

        theCtx.UVBounds(aFace, out umin, out umax, out vmin, out vmax);
        double tolU = Max((umax - umin) * 0.01, global::Kerf.Engine.Foundation.Precision.Confusion());
        double tolV = Max((vmax - vmin) * 0.01, global::Kerf.Engine.Foundation.Precision.Confusion());
        double fp = aPC.FirstParameter();
        double lp = aPC.LastParameter();

        // adjust domain for periodic surfaces
        Surface aSurf = Tool.Surface(aFace, out _)!;
        if (aSurf is TrimmedSurface aTrimmed)
        {
            aSurf = aTrimmed.BasisSurface();
        }
        Point2D pnt = aPC.Value((fp + lp) / 2);
        double u, v;
        pnt.Coord(out u, out v);
        //
        if (aSurf.IsUPeriodic)
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
        if (aSurf.IsVPeriodic)
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
        Point2D aP2D;
        //
        CurveAdaptor2D aGAC = new(aPC);
        aNbIntervals = aGAC.NbIntervals(CN);
        //
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        aGAC.Intervals(aTI, CN);
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
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(aUmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aUmax)))
        {
            double dU;
            //
            dU = 0.1 * (aUmax - aUmin);
            aUmin = aUmin - dU;
            aUmax = aUmax + dU;
        }
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(aVmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aVmax)))
        {
            double dV;
            //
            dV = 0.1 * (aVmax - aVmin);
            aVmin = aVmin - dV;
            aVmax = aVmax + dV;
        }
    }
}
