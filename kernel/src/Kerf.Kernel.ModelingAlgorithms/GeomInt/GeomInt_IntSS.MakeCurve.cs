// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Surfaces;

internal partial class IntSS
{
    /// <summary>std::sort's operator&lt; on doubles (TrimILineOnSurfBoundaries).</summary>
    private readonly struct DoubleLess : Heap.ILess<double>
    {
        public bool Less(double left, double right) => left < right;
    }

    private static void AdjustUPeriodic(Surface aS, Curve2D? aC2D)
    {
        if (aC2D is null || !aS.IsUPeriodic)
        {
            return;
        }
        //
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion(); // 1.e-9
        double epsilon = Epsilon(10.0); // 1.77e-15
        //
        aS.ParameterBounds(out double umin, out double umax, out _, out _);
        double period = aS.UPeriod;

        double t1 = aC2D.FirstParameter;
        double t2 = aC2D.LastParameter;
        double tx = t1 + 0.467 * (t2 - t1);
        Point2D px = aC2D.Value(tx);
        //
        double ux = px.X;
        if (Math.Abs(ux) < epsilon)
        {
            ux = 0.0;
        }
        if (Math.Abs(ux - period) < epsilon)
        {
            ux = period;
        }
        //
        double dU = 0.0;
        while (ux < (umin - eps))
        {
            ux += period;
            dU += period;
        }
        while (ux > (umax + eps))
        {
            ux -= period;
            dU -= period;
        }
        //
        if (dU != 0.0)
        {
            Vector2D v2D = new(dU, 0.0);
            aC2D.Translate(v2D);
        }
    }

    private static void GetQuadric(SurfaceAdaptor hs1, ref SurfaceQuadric quad1)
    {
        switch (hs1.GetType())
        {
            case Plane:
                quad1.SetValue(hs1.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad1.SetValue(hs1.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad1.SetValue(hs1.Cone());
                break;
            case SurfaceKind.Sphere:
                quad1.SetValue(hs1.Sphere());
                break;
            case SurfaceKind.Torus:
                quad1.SetValue(hs1.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_IntSS::MakeCurve");
        }
    }

    private static void Parameters(SurfaceAdaptor hs1, SurfaceAdaptor hs2, in Point3D ptref, out double u1,
                                   out double v1, out double u2, out double v2)
    {
        SurfaceQuadric quad1 = new(), quad2 = new();
        //
        GetQuadric(hs1, ref quad1);
        GetQuadric(hs2, ref quad2);
        //
        u1 = v1 = u2 = v2 = 0.0;
        quad1.Parameters(ptref, ref u1, ref v1);
        quad2.Parameters(ptref, ref u2, ref v2);
    }

    private static bool ParametersOfNearestPointOnSurface(ExtPS extr, ref double u, ref double v)
    {
        if (!extr.IsDone || extr.ExtCount == 0)
        {
            return false;
        }

        int index = 1;
        double minSQDist = extr.SquareDistance(index);
        for (int i = 2; i <= extr.ExtCount; i++)
        {
            double sqd = extr.SquareDistance(i);
            if (sqd < minSQDist)
            {
                minSQDist = sqd;
                index = i;
            }
        }

        extr.Point(index).Parameter(out u, out v);

        return true;
    }

    private static void GetSegmentBoundary(in IntersectionSegment2D segm, Curve2D curve,
                                           List<double> arrayOfParameters)
    {
        double u1 = curve.FirstParameter, u2 = curve.LastParameter;

        if (segm.HasFirstPoint)
        {
            IntersectionPoint2D ipf = segm.FirstPoint();
            u1 = ipf.ParamOnFirst();
        }

        if (segm.HasLastPoint)
        {
            IntersectionPoint2D ipl = segm.LastPoint();
            u2 = ipl.ParamOnFirst();
        }

        arrayOfParameters.Add(u1);
        arrayOfParameters.Add(u2);
    }

    private static void IntersectCurveAndBoundary(Curve2D? c2d, Curve2D?[] arrBounds,
                                                  int numberOfCurves, double tol,
                                                  List<double> arrayOfParameters)
    {
        if (c2d is null)
        {
            return;
        }

        CurveAdaptor2D ac1 = new(c2d);
        for (int curID = 0; curID < numberOfCurves; curID++)
        {
            if (arrBounds[curID] is null)
            {
                continue;
            }

            CurveAdaptor2D ac2 = new(arrBounds[curID]!);
            GInter intCC2d = new(ac1, ac2, tol, tol);

            if (!intCC2d.IsDone || intCC2d.IsEmpty)
            {
                continue;
            }

            for (int pntID = 1; pntID <= intCC2d.PointCount; pntID++)
            {
                double param = intCC2d.Point(pntID).ParamOnFirst();
                arrayOfParameters.Add(param);
            }

            for (int segmID = 1; segmID <= intCC2d.SegmentCount; segmID++)
            {
                GetSegmentBoundary(intCC2d.Segment(segmID), c2d, arrayOfParameters);
            }
        }
    }

    // Check if theAHC2d corresponds to a degenerated edge.
    private static bool IsDegenerated(SurfaceAdaptor gahs, Curve2DAdaptorBase ahc2d, double firstPar,
                                      double lastPar)
    {
        double sqTol = global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
        Point2D p2d;
        Point3D p1, p2;

        ahc2d.D0(firstPar, out p2d);
        gahs.D0(p2d.X, p2d.Y, out p1);

        ahc2d.D0(lastPar, out p2d);
        gahs.D0(p2d.X, p2d.Y, out p2);

        if (p1.SquareDistanceTo(p2) > sqTol)
        {
            return false;
        }

        ahc2d.D0(0.5 * (firstPar + lastPar), out p2d);
        gahs.D0(p2d.X, p2d.Y, out p2);

        return p1.SquareDistanceTo(p2) <= sqTol;
    }

    /// <summary>Makes the Geom curves of the line of index Index of the intersector (with their 2D curves on
    /// request) and appends them to the results.</summary>
    protected void MakeCurve(int index, TopolTool dom1, TopolTool dom2, double tol, bool approx,
                             bool approxS1, bool approxS2)
    {
        bool myApprox1, myApprox2, myApprox;
        double tolpc, myTolApprox;
        PatchLineType typl;
        BSplineCurve2D? H1 = null;
        Surface s1, s2;
        //
        tolpc = tol;
        myApprox = approx;
        myApprox1 = approxS1;
        myApprox2 = approxS2;
        myTolApprox = 0.0000001;
        //
        SurfaceAdaptor myHS1 = this._hs1!;
        SurfaceAdaptor myHS2 = this._hs2!;
        s1 = myHS1.Surface()!;
        s2 = myHS2.Surface()!;
        //
        PatchLine l = _intersector.Line(index);
        typl = l.ArcType();
        //
        if (typl == PatchLineType.Walking)
        {
            WLine? wLine = l as WLine;
            if (wLine is null)
            {
                return;
            }
            l = wLine;
        }
        //
        // Line Constructor
        _lConstruct.Perform(l);
        if (!_lConstruct.IsDone || _lConstruct.PartCount <= 0)
        {
            return;
        }
        // Do the Curve
        bool ok;
        int i, j, nbParts;
        double fprm, lprm;
        Curve3D? newc = null;

        switch (typl)
        {
            // ########################################
            //  Line, Parabola, Hyperbola
            // ########################################
            case PatchLineType.Lin:
            case PatchLineType.Parabola:
            case PatchLineType.Hyperbola:
            {
                if (typl == PatchLineType.Lin)
                {
                    newc = new LineCurve3D((l as GLine)!.Line());
                }

                else if (typl == PatchLineType.Parabola)
                {
                    newc = new ParabolaCurve3D((l as GLine)!.Parabola());
                }

                else if (typl == PatchLineType.Hyperbola)
                {
                    newc = new HyperbolaCurve3D((l as GLine)!.Hyperbola());
                }
                //
                nbParts = _lConstruct.PartCount;
                for (i = 1; i <= nbParts; i++)
                {
                    _lConstruct.Part(i, out fprm, out lprm);

                    if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fprm) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lprm))
                    {
                        TrimmedCurve3D ct3D = new(newc!, fprm, lprm);
                        _sline.Append(ct3D);
                        //
                        if (myApprox1)
                        {
                            Curve2D? c2d = null;
                            BuildPCurves(fprm, lprm, ref tolpc, myHS1.Surface(), newc!, ref c2d);
                            if (tolpc > _tolReached2d || _tolReached2d == 0.0)
                            {
                                _tolReached2d = tolpc;
                            }
                            _slineS1.Append(new TrimmedCurve2D(c2d!, fprm, lprm));
                        }
                        else
                        {
                            _slineS1.Append(H1);
                        }
                        //
                        if (myApprox2)
                        {
                            Curve2D? c2d = null;
                            BuildPCurves(fprm, lprm, ref tolpc, myHS2.Surface(), newc!, ref c2d);
                            if (tolpc > _tolReached2d || _tolReached2d == 0.0)
                            {
                                _tolReached2d = tolpc;
                            }
                            //
                            _slineS2.Append(new TrimmedCurve2D(c2d!, fprm, lprm));
                        }
                        else
                        {
                            _slineS2.Append(H1);
                        }
                    } // if (!Precision::IsNegativeInfinite(fprm) &&  !Precision::IsPositiveInfinite(lprm))

                    else
                    {
                        SurfaceKind typS1 = myHS1.GetType();
                        SurfaceKind typS2 = myHS2.GetType();
                        if (typS1 == SurfaceOfExtrusion || typS1 == SurfaceKind.OffsetSurface
                            || typS1 == SurfaceOfRevolution || typS2 == SurfaceOfExtrusion
                            || typS2 == SurfaceKind.OffsetSurface || typS2 == SurfaceOfRevolution)
                        {
                            _sline.Append(newc!);
                            _slineS1.Append(H1);
                            _slineS2.Append(H1);
                            continue;
                        }
                        bool bFNIt, bLPIt;
                        double testPrm, dT = 100.0;
                        double u1, v1, u2, v2, tolX;
                        //
                        bFNIt = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fprm);
                        bLPIt = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lprm);

                        testPrm = 0.0;

                        if (bFNIt && !bLPIt)
                        {
                            testPrm = lprm - dT;
                        }
                        else if (!bFNIt && bLPIt)
                        {
                            testPrm = fprm + dT;
                        }
                        //
                        Point3D ptref = newc!.Value(testPrm);
                        //
                        tolX = global::Kerf.Engine.Foundation.Precision.Confusion();
                        Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                        ok = (dom1.Classify(new Point2D(u1, v1), tolX) != Out);
                        if (ok)
                        {
                            ok = (dom2.Classify(new Point2D(u2, v2), tolX) != Out);
                        }
                        if (ok)
                        {
                            _sline.Append(newc);
                            _slineS1.Append(H1);
                            _slineS2.Append(H1);
                        }
                    }
                } // end of for (i=1; i<=myLConstruct.NbParts(); i++)
            } // case IntPatch_Lin:  case IntPatch_Parabola:  case IntPatch_Hyperbola:
            break;

            // ########################################
            //  Circle and Ellipse
            // ########################################
            case PatchLineType.Circle:
            case PatchLineType.Ellipse:
            {

                if (typl == PatchLineType.Circle)
                {
                    newc = new CircleCurve3D((l as GLine)!.Circle());
                }
                else
                {
                    newc = new EllipseCurve3D((l as GLine)!.Ellipse());
                }
                //
                double period, realEpsilon;
                //
                realEpsilon = RealEpsilon();
                period = M_PI + M_PI;
                //
                nbParts = _lConstruct.PartCount;
                //
                for (i = 1; i <= nbParts; i++)
                {
                    _lConstruct.Part(i, out fprm, out lprm);
                    //
                    if (Math.Abs(fprm) > realEpsilon || Math.Abs(lprm - period) > realEpsilon)
                    {
                        //==============================================
                        TrimmedCurve3D tc3D = new(newc, fprm, lprm);
                        //
                        _sline.Append(tc3D);
                        //
                        fprm = tc3D.FirstParameter;
                        lprm = tc3D.LastParameter;
                        ////
                        if (myApprox1)
                        {
                            Curve2D? c2d = null;
                            BuildPCurves(fprm, lprm, ref tolpc, myHS1.Surface(), newc, ref c2d);
                            if (tolpc > _tolReached2d || _tolReached2d == 0.0)
                            {
                                _tolReached2d = tolpc;
                            }
                            _slineS1.Append(c2d);
                        }
                        else
                        { ////
                            _slineS1.Append(H1);
                        }
                        //
                        if (myApprox2)
                        {
                            Curve2D? c2d = null;
                            BuildPCurves(fprm, lprm, ref tolpc, myHS2.Surface(), newc, ref c2d);
                            if (tolpc > _tolReached2d || _tolReached2d == 0)
                            {
                                _tolReached2d = tolpc;
                            }
                            _slineS2.Append(c2d);
                        }
                        else
                        {
                            _slineS2.Append(H1);
                        }
                        //==============================================
                    } // if (std::abs(fprm) > RealEpsilon() || std::abs(lprm-2.*M_PI) > RealEpsilon())
                    //
                    else
                    { //  on regarde si on garde
                        //
                        if (nbParts == 1)
                        {
                            if (Math.Abs(fprm) < RealEpsilon() && Math.Abs(lprm - 2.0 * M_PI) < RealEpsilon())
                            {
                                TrimmedCurve3D tc3D = new(newc, fprm, lprm);
                                //
                                _sline.Append(tc3D);
                                fprm = tc3D.FirstParameter;
                                lprm = tc3D.LastParameter;

                                if (myApprox1)
                                {
                                    Curve2D? c2d = null;
                                    BuildPCurves(fprm, lprm, ref tolpc, myHS1.Surface(), newc, ref c2d);
                                    if (tolpc > _tolReached2d || _tolReached2d == 0)
                                    {
                                        _tolReached2d = tolpc;
                                    }
                                    _slineS1.Append(c2d);
                                }
                                else
                                { ////
                                    _slineS1.Append(H1);
                                }

                                if (myApprox2)
                                {
                                    Curve2D? c2d = null;
                                    BuildPCurves(fprm, lprm, ref tolpc, myHS2.Surface(), newc, ref c2d);
                                    if (tolpc > _tolReached2d || _tolReached2d == 0)
                                    {
                                        _tolReached2d = tolpc;
                                    }
                                    _slineS2.Append(c2d);
                                }
                                else
                                {
                                    _slineS2.Append(H1);
                                }
                                break;
                            }
                        }
                        //
                        double twoPIdiv17, u1, v1, u2, v2, tolX;
                        //
                        twoPIdiv17 = 2.0 * M_PI / 17.0;
                        //
                        for (j = 0; j <= 17; j++)
                        {
                            Point3D ptref = newc.Value(j * twoPIdiv17);
                            tolX = global::Kerf.Engine.Foundation.Precision.Confusion();

                            Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                            ok = (dom1.Classify(new Point2D(u1, v1), tolX) != Out);
                            if (ok)
                            {
                                ok = (dom2.Classify(new Point2D(u2, v2), tolX) != Out);
                            }
                            if (ok)
                            {
                                _sline.Append(newc);
                                //==============================================
                                if (myApprox1)
                                {
                                    Curve2D? c2d = null;
                                    BuildPCurves(fprm, lprm, ref tolpc, myHS1.Surface(), newc, ref c2d);
                                    if (tolpc > _tolReached2d || _tolReached2d == 0)
                                    {
                                        _tolReached2d = tolpc;
                                    }
                                    _slineS1.Append(c2d);
                                }
                                else
                                {
                                    _slineS1.Append(H1);
                                }

                                if (myApprox2)
                                {
                                    Curve2D? c2d = null;
                                    BuildPCurves(fprm, lprm, ref tolpc, myHS2.Surface(), newc, ref c2d);
                                    if (tolpc > _tolReached2d || _tolReached2d == 0)
                                    {
                                        _tolReached2d = tolpc;
                                    }
                                    _slineS2.Append(c2d);
                                }
                                else
                                {
                                    _slineS2.Append(H1);
                                }
                                break;
                            } //  end of if (ok) {
                        } //  end of for (int j=0; j<=17; j++)
                    } //  end of else { on regarde si on garde
                } // for (i=1; i<=myLConstruct.NbParts(); i++)
            } // IntPatch_Circle: IntPatch_Ellipse
            break;

            // ########################################
            //  Analytic
            // ########################################
            case PatchLineType.Analytic:
                // This case was processed earlier (in IntPatch_Intersection)

                break;

            // ########################################
            //  Walking
            // ########################################
            case PatchLineType.Walking:
            {
                WLine wl = (l as WLine)!;

                //
                int ifprm, ilprm;
                //
                if (!myApprox)
                {
                    nbParts = _lConstruct.PartCount;
                    for (i = 1; i <= nbParts; i++)
                    {
                        _lConstruct.Part(i, out fprm, out lprm);
                        ifprm = (int)fprm;
                        ilprm = (int)lprm;
                        //
                        BSplineCurve2D? h1 = null, h2 = null;

                        if (myApprox1)
                        {
                            h1 = MakeBSpline2d(wl, ifprm, ilprm, true);
                        }
                        if (myApprox2)
                        {
                            h2 = MakeBSpline2d(wl, ifprm, ilprm, false);
                        }
                        //
                        Curve3D bSp = MakeBSpline(wl, ifprm, ilprm);
                        //
                        _sline.Append(bSp);
                        _slineS1.Append(h1);
                        _slineS2.Append(h2);
                    }
                }
                //
                else
                {
                    bool bIsDecomposited;
                    int nbiter, nbSeqOfL;
                    WLApprox theapp3d = new();
                    NCollection_Sequence<PatchLine> seqOfL = new();
                    double tol2d, tolSS;
                    //
                    tol2d = myTolApprox;
                    tolSS = 2.0e-7;
                    theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, myHS1 != myHS2);
                    //
                    bIsDecomposited = LineTool.DecompositionOfWLine(wl, myHS1, myHS2, tolSS, _lConstruct,
                                                                            seqOfL);
                    //
                    nbParts = _lConstruct.PartCount;
                    nbSeqOfL = seqOfL.Length();
                    //
                    nbiter = (bIsDecomposited) ? nbSeqOfL : nbParts;
                    //
                    for (i = 1; i <= nbiter; i++)
                    {
                        if (bIsDecomposited)
                        {
                            wl = (seqOfL.Value(i) as WLine)!;
                            ifprm = 1;
                            ilprm = wl.NbPnts();
                        }
                        else
                        {
                            _lConstruct.Part(i, out fprm, out lprm);
                            ifprm = (int)fprm;
                            ilprm = (int)lprm;
                        }

                        bool anApprox = myApprox;
                        bool approx1 = myApprox1;
                        bool approx2 = myApprox2;
                        SurfaceKind typs1, typs2;
                        typs1 = myHS1.GetType();
                        typs2 = myHS2.GetType();

                        if (typs1 == Plane)
                        {
                            anApprox = false;
                            approx1 = true;
                        }
                        else if (typs2 == Plane)
                        {
                            anApprox = false;
                            approx2 = true;
                        }

                        global::Kerf.Geometry.ParametrizationType parType =
                            KnotTools.DefineParType(wl, ifprm, ilprm, anApprox, approx1, approx2);

                        theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, myHS1 != myHS2, parType);

                        //-- lbr :
                        //-- Si une des surfaces est un plan , on approxime en 2d
                        //-- sur cette surface et on remonte les points 2d en 3d.
                        //
                        if (typs1 == Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, wl, false, true, myApprox2, ifprm, ilprm);
                        }
                        else if (typs2 == Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, wl, false, myApprox1, true, ifprm, ilprm);
                        }
                        else
                        {
                            //
                            if (myHS1 != myHS2)
                            {
                                if ((typs1 == SurfaceKind.BezierSurface || typs1 == SurfaceKind.BSplineSurface)
                                    && (typs2 == SurfaceKind.BezierSurface || typs2 == SurfaceKind.BSplineSurface))
                                {

                                    theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, true, parType);
                                }
                            }
                            //
                            theapp3d.Perform(myHS1, myHS2, wl, true, myApprox1, myApprox2, ifprm, ilprm);
                        }

                        if (!theapp3d.IsDone())
                        {
                            //
                            BSplineCurve2D? h1 = null, h2 = null;
                            //
                            Curve3D bSp = MakeBSpline(wl, ifprm, ilprm);
                            if (myApprox1)
                            {
                                h1 = MakeBSpline2d(wl, ifprm, ilprm, true);
                            }
                            if (myApprox2)
                            {
                                h2 = MakeBSpline2d(wl, ifprm, ilprm, false);
                            }
                            //
                            _sline.Append(bSp);
                            _slineS1.Append(h1);
                            _slineS2.Append(h2);
                        } // if (!theapp3d.IsDone())

                        else
                        {
                            if (myApprox1 || myApprox2 || (typs1 == Plane || typs2 == Plane))
                            {
                                if (theapp3d.TolReached2d() > _tolReached2d || _tolReached2d == 0.0)
                                {
                                    _tolReached2d = theapp3d.TolReached2d();
                                }
                            }
                            if (typs1 == Plane || typs2 == Plane)
                            {
                                _tolReached3d = _tolReached2d;
                            }
                            else if (theapp3d.TolReached3d() > _tolReached3d || _tolReached3d == 0.0)
                            {
                                _tolReached3d = theapp3d.TolReached3d();
                            }

                            int nbMultiCurves, nbpoles;
                            //
                            nbMultiCurves = theapp3d.MultiCurveCount;
                            for (j = 1; j <= nbMultiCurves; j++)
                            {
                                if (typs1 == Plane)
                                {
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);

                                    mbspc.Curve(1, tpoles2d);
                                    Plane3D pln = myHS1.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik,
                                                        global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X, tpoles2d.Value(ik).Y, pln));
                                    }
                                    //
                                    BSplineCurve3D bs = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, _tolCheck, _tolAngCheck);
                                    check.FixTangent(true, true);
                                    //
                                    _sline.Append(bs);
                                    //
                                    if (myApprox1)
                                    {
                                        BSplineCurve2D bs1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check1 = new(bs1, _tolCheck, _tolAngCheck);
                                        check1.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(s1, bs1);
                                        //
                                        _slineS1.Append(bs1);
                                    }
                                    else
                                    {
                                        _slineS1.Append(H1);
                                    }

                                    if (myApprox2)
                                    {
                                        mbspc.Curve(2, tpoles2d);

                                        BSplineCurve2D bs2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve newCheck = new(bs2, _tolCheck, _tolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(s2, bs2);
                                        //
                                        _slineS2.Append(bs2);
                                    }
                                    else
                                    {
                                        _slineS2.Append(H1);
                                    }
                                } // if(typs1 == GeomAbs_Plane)
                                //
                                else if (typs2 == Plane)
                                {
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve((myApprox1) ? 2 : 1, tpoles2d);
                                    Plane3D pln = myHS2.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik,
                                                        global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X, tpoles2d.Value(ik).Y, pln));
                                    }
                                    //
                                    BSplineCurve3D bs = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, _tolCheck, _tolAngCheck);
                                    check.FixTangent(true, true);
                                    //
                                    _sline.Append(bs);
                                    //
                                    if (myApprox2)
                                    {
                                        BSplineCurve2D bs1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check1 = new(bs1, _tolCheck, _tolAngCheck);
                                        check1.FixTangent(true, true);
                                        //
                                        //
                                        AdjustUPeriodic(s2, bs1);
                                        //
                                        _slineS2.Append(bs1);
                                    }
                                    else
                                    {
                                        _slineS2.Append(H1);
                                    }

                                    if (myApprox1)
                                    {
                                        mbspc.Curve(1, tpoles2d);
                                        BSplineCurve2D bs2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check2 = new(bs2, _tolCheck, _tolAngCheck);
                                        check2.FixTangent(true, true);
                                        //
                                        //
                                        AdjustUPeriodic(s1, bs2);
                                        //
                                        _slineS1.Append(bs2);
                                    }
                                    else
                                    {
                                        _slineS1.Append(H1);
                                    }
                                } // else if(typs2 == GeomAbs_Plane)
                                //
                                else
                                { // typs1!=GeomAbs_Plane && typs2!=GeomAbs_Plane
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve(1, tpoles);
                                    BSplineCurve3D bs = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, _tolCheck, _tolAngCheck);
                                    check.FixTangent(true, true);
                                    //
                                    // Check IsClosed()
                                    double dist = Max(bs.StartPoint.XYZ().SquareModulus(),
                                                       bs.EndPoint.XYZ().SquareModulus());
                                    double eps = Epsilon(dist);
                                    if (bs.StartPoint.SquareDistanceTo(bs.EndPoint) < 2.0 * eps)
                                    {
                                        // Avoid creating B-splines containing two coincident poles only
                                        if (mbspc.Degree() == 1 && nbpoles == 2)
                                        {
                                            continue;
                                        }

                                        if (!bs.IsClosed && !bs.IsPeriodic)
                                        {
                                            // force Closed()
                                            Point3D pm = new((bs.Pole(1).XYZ() + bs.Pole(bs.PoleCount).XYZ()) / 2.0);
                                            bs.SetPole(1, pm);
                                            bs.SetPole(bs.PoleCount, pm);
                                        }
                                    }
                                    _sline.Append(bs);

                                    if (myApprox1)
                                    {
                                        NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                        mbspc.Curve(2, tpoles2d);
                                        BSplineCurve2D bs1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve newCheck = new(bs1, _tolCheck, _tolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(s1, bs1);
                                        //
                                        _slineS1.Append(bs1);
                                    }
                                    else
                                    {
                                        _slineS1.Append(H1);
                                    }
                                    if (myApprox2)
                                    {
                                        NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                        mbspc.Curve((myApprox1) ? 3 : 2, tpoles2d);
                                        BSplineCurve2D bs2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve newCheck = new(bs2, _tolCheck, _tolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(s2, bs2);
                                        //
                                        _slineS2.Append(bs2);
                                    }
                                    else
                                    {
                                        _slineS2.Append(H1);
                                    }
                                } // else { // typs1!=GeomAbs_Plane && typs2!=GeomAbs_Plane
                            } // for (j=1; j<=aNbMultiCurves; j++
                        }
                    }
                }
            }
            break;

            case PatchLineType.Restriction:
            {
                RLine rl = (l as RLine)!;
                Curve3D? c3d = null;
                Curve2D? c2d1 = null, c2d2 = null;
                double tolReached = 0.0;
                TreatRLine(rl, myHS1, myHS2, ref c3d, ref c2d1, ref c2d2, ref tolReached);

                if (c3d is null)
                {
                    break;
                }

                Box2d box1 = new(), box2 = new();

                double u1f = myHS1.FirstUParameter(), v1f = myHS1.FirstVParameter(), u1l = myHS1.LastUParameter(),
                       v1l = myHS1.LastVParameter();
                double u2f = myHS2.FirstUParameter(), v2f = myHS2.FirstVParameter(), u2l = myHS2.LastUParameter(),
                       v2l = myHS2.LastVParameter();

                box1.Add(new Point2D(u1f, v1f));
                box1.Add(new Point2D(u1l, v1l));
                box2.Add(new Point2D(u2f, v2f));
                box2.Add(new Point2D(u2l, v2l));

                List<double> arrayOfParameters = new();

                // We consider here that the intersection line is same-parameter-line
                arrayOfParameters.Add(c3d.FirstParameter);
                arrayOfParameters.Add(c3d.LastParameter);

                TrimILineOnSurfBoundaries(c2d1, c2d2, box1, box2, arrayOfParameters);

                int nbIntersSolutionsm1 = arrayOfParameters.Count - 1;

                // Trim RLine found.
                for (int ind = 0; ind < nbIntersSolutionsm1; ind++)
                {
                    double parF = arrayOfParameters[ind], parL = arrayOfParameters[ind + 1];

                    if ((parL - parF) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }

                    double par = 0.5 * (parF + parL);
                    Point2D pt;

                    Curve2D? curv2d1 = null, curv2d2 = null;
                    if (c2d1 is not null)
                    {
                        c2d1.D0(par, out pt);

                        if (box1.IsOut(pt))
                        {
                            continue;
                        }

                        if (myApprox1)
                        {
                            curv2d1 = new TrimmedCurve2D(c2d1, parF, parL);
                        }
                    }

                    if (c2d2 is not null)
                    {
                        c2d2.D0(par, out pt);

                        if (box2.IsOut(pt))
                        {
                            continue;
                        }

                        if (myApprox2)
                        {
                            curv2d2 = new TrimmedCurve2D(c2d2, parF, parL);
                        }
                    }

                    Curve3D curv3d = new TrimmedCurve3D(c3d, parF, parL);

                    _sline.Append(curv3d);
                    _slineS1.Append(curv2d1);
                    _slineS2.Append(curv2d2);
                }
            }
            break;
        }
    }

    /// <summary>
    /// Makes the 3D curve of a restriction line as the approximation of its arc on its surface (degree up to 8, 1000
    /// segments), and the 2D curves on both surfaces (the arc itself, trimmed, and the projection on the other
    /// surface). theC3d stays null if the arc is degenerate or the approximation fails. The curves are new and
    /// unfrozen.
    /// </summary>
    public static void TreatRLine(RLine rl, SurfaceAdaptor hs1, SurfaceAdaptor hs2,
                                  ref Curve3D? c3d, ref Curve2D? c2d1, ref Curve2D? c2d2,
                                  ref double tolReached)
    {
        SurfaceAdaptor gahs;
        Curve2DAdaptorBase ahc2d;
        double tf, tl;
        // It is assumed that 2d curve is 2d line (rectangular surface domain)
        if (rl.IsArcOnS1)
        {
            gahs = hs1;
            ahc2d = rl.ArcOnS1()!;
            rl.ParamOnS1(out tf, out tl);
            c2d1 = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(ahc2d);
            tf = Max(tf, c2d1.FirstParameter);
            tl = Min(tl, c2d1.LastParameter);
            c2d1 = new TrimmedCurve2D(c2d1, tf, tl);
        }
        else if (rl.IsArcOnS2)
        {
            gahs = hs2;
            ahc2d = rl.ArcOnS2()!;
            rl.ParamOnS2(out tf, out tl);
            c2d2 = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(ahc2d);
            tf = Max(tf, c2d2.FirstParameter);
            tl = Min(tl, c2d2.LastParameter);
            c2d2 = new TrimmedCurve2D(c2d2, tf, tl);
        }
        else
        {
            return;
        }

        // Restriction line can correspond to a degenerated edge.
        // In this case we return null-curve.
        if (IsDegenerated(gahs, ahc2d, tf, tl))
        {
            return;
        }

        //
        // To provide sameparameter it is necessary to get 3d curve as
        // approximation of curve on surface.
        int maxDeg = 8;
        int maxSeg = 1000;
        CurveOnSurfaceApproximation app = new(ahc2d, gahs, tf, tl, global::Kerf.Engine.Foundation.Precision.Confusion());
        app.Perform(maxSeg, maxDeg, C1, true, false);
        if (!app.HasResult)
        {
            return;
        }

        // The approximation is discarded: its curve is taken unfrozen.
        c3d = app.InternalCurve3d();
        tolReached = app.MaxError3d();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (rl.IsArcOnS1)
        {
            Surface s = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeSurface(hs2);
            BuildPCurves(tf, tl, ref tol, s, c3d!, ref c2d2);
        }
        if (rl.IsArcOnS2)
        {
            Surface s = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeSurface(hs1);
            BuildPCurves(tf, tl, ref tol, s, c3d!, ref c2d1);
        }
        tolReached = Max(tolReached, tol);
    }

    /// <summary>
    /// Projects theCurve between theFirst and theLast on theSurface bounded by [theUmin, theUmax] × [theVmin,
    /// theVmax] into theCurve2d (a 2-pole segment on a range shorter than 2e-9), unless theCurve2d is already set or
    /// theSurface is null; on a U-periodic surface the result is moved into the U range. theTol is the projection's
    /// tolerance in and out.
    /// </summary>
    public static void BuildPCurves(double first, double last, double umin, double umax, double vmin,
                                    double vmax, ref double tol, Surface? surface, Curve3D curve,
                                    ref Curve2D? curve2d)
    {
        if (curve2d is not null || surface is null)
        {
            return;
        }
        //
        // in class ProjLib_Function the range of parameters is shrank by 1.e-09
        if ((last - first) > 2.0e-09)
        {
            curve2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve, first, last, surface, umin, umax, vmin,
                                             vmax, ref tol);
            if (curve2d is null)
            {
                // proj. a circle that goes through the pole on a sphere to the sphere
                tol += global::Kerf.Engine.Foundation.Precision.Confusion();
                curve2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve, first, last, surface, ref tol);
            }
            // OCCT dereferences the projection without checking that the second one succeeded (a null 2D curve
            // here is an access violation in OCCT, a NullReferenceException in the port).
            if (curve2d!.GetType() == typeof(BSplineCurve2D))
            {
                // Check first, last knots to avoid problems with trimming
                // First, last knots can differ from f, l because of numerical error
                // of projection and approximation
                // The same checking as in Geom2d_TrimmedCurve
                if ((curve2d.FirstParameter - first > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    || (last - curve2d.LastParameter > global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    BSplineCurve2D bspl = (curve2d as BSplineCurve2D)!;
                    NCollection_Array1<double> aKnots = new(bspl.InternalKnots());
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(first, last, aKnots);
                    bspl.SetKnots(aKnots);
                }
            }
        }
        else
        {
            if ((last - first) > Epsilon(Math.Abs(first)))
            {
                // The domain of C2d is [Epsilon(std::abs(f)), 2.e-09]
                // On this small range C2d can be considered as segment
                // of line.

                double u = 0.0, v = 0.0;
                SurfaceAdaptor anAS = new();
                anAS.Load(surface);
                ExtPS extr = new();
                Point3D p3d1 = curve.Value(first);
                Point3D p3d2 = curve.Value(last);

                extr.SetAlgo(ExtAlgo.ExtAlgo_Grad);
                extr.Initialize(anAS, umin, umax, vmin, vmax, global::Kerf.Engine.Foundation.Precision.Confusion(),
                                  global::Kerf.Engine.Foundation.Precision.Confusion());
                extr.Perform(p3d1);

                if (ParametersOfNearestPointOnSurface(extr, ref u, ref v))
                {
                    Point2D p2d1 = new(u, v);

                    extr.Perform(p3d2);

                    if (ParametersOfNearestPointOnSurface(extr, ref u, ref v))
                    {
                        Point2D p2d2 = new(u, v);

                        if (p2d1.DistanceTo(p2d2) > Constants.Resolution())
                        {
                            NCollection_Array1<Point2D> poles = new(1, 2);
                            NCollection_Array1<double> knots = new(1, 2);
                            NCollection_Array1<int> mults = new(1, 2);
                            poles[1] = p2d1;
                            poles[2] = p2d2;
                            knots[1] = first;
                            knots[2] = last;
                            mults[1] = mults[2] = 2;

                            curve2d = new BSplineCurve2D(poles, knots, mults, 1);

                            // Check same parameter in middle point .begin
                            Point3D pMid = curve.Value(0.5 * (first + last));
                            Point2D pmidcurve2d = new(0.5 * (p2d1.XY() + p2d2.XY()));
                            Point3D pc = anAS.Value(pmidcurve2d.X, pmidcurve2d.Y);
                            double dist = pMid.DistanceTo(pc);
                            tol = Max(dist, tol);
                            // Check same parameter in middle point .end
                        }
                    }
                }
            }
        }
        //
        if (surface.IsUPeriodic && curve2d is not null)
        {
            // Recadre dans le domaine UV de la face
            double tm, u0, eps, period, du;
            bool bAdjust;
            //
            eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
            period = surface.UPeriod;
            //
            tm = 0.5 * (first + last);
            Point2D pm = curve2d.Value(tm);
            u0 = pm.X;
            //
            bAdjust = global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u0, umin, umax, period, out _, out du, eps);
            if (bAdjust)
            {
                Vector2D t1 = new(du, 0.0);
                curve2d.Translate(t1);
            }
        }
    }

    /// <summary>
    /// Projects C between f and l on S (over S's bounds) into C2d, unless C2d is already set or S is null (see
    /// <see cref="BuildPCurves(double, double, double, double, double, double, ref double, Surface?, Curve3D,
    /// ref Curve2D?)"/>).
    /// </summary>
    public static void BuildPCurves(double f, double l, ref double tol, Surface? s, Curve3D c,
                                    ref Curve2D? c2d)
    {
        if (c2d is not null || s is null)
        {
            return;
        }
        //
        s.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);

        BuildPCurves(f, l, umin, umax, vmin, vmax, ref tol, s, c, ref c2d);
    }

    /// <summary>
    /// Appends to theArrayOfParameters the parameters on theC2d1 and theC2d2 of their intersections with the sides of
    /// the boxes theBound1 and theBound2 (the surfaces' rectangular domains), then sorts the array.
    /// </summary>
    public static void TrimILineOnSurfBoundaries(Curve2D? c2d1, Curve2D? c2d2, in Box2d bound1,
                                                 in Box2d bound2,
                                                 List<double> arrayOfParameters)
    {
        // Rectangular boundaries of two surfaces: [0]:U=Ufirst, [1]:U=Ulast,
        //                                         [2]:V=Vfirst, [3]:V=Vlast
        const int numberOfCurves = 4;
        Curve2D?[] curS1Bounds = new Curve2D?[numberOfCurves];
        Curve2D?[] curS2Bounds = new Curve2D?[numberOfCurves];

        bound1.Get(out double u1f, out double v1f, out double u1l, out double v1l);
        bound2.Get(out double u2f, out double v2f, out double u2l, out double v2l);

        double delta = v1l - v1f;
        if (Math.Abs(delta) > RealSmall())
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u1f))
            {
                curS1Bounds[0] = new LineCurve2D(new Point2D(u1f, v1f), new Direction2D(Direction2D.D.Y));

                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS1Bounds[0] = new TrimmedCurve2D(curS1Bounds[0]!, 0, delta);
                }
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u1l))
            {
                curS1Bounds[1] = new LineCurve2D(new Point2D(u1l, v1f), new Direction2D(Direction2D.D.Y));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS1Bounds[1] = new TrimmedCurve2D(curS1Bounds[1]!, 0, delta);
                }
            }
        }

        delta = u1l - u1f;
        if (Math.Abs(delta) > RealSmall())
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v1f))
            {
                curS1Bounds[2] = new LineCurve2D(new Point2D(u1f, v1f), new Direction2D(Direction2D.D.X));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS1Bounds[2] = new TrimmedCurve2D(curS1Bounds[2]!, 0, delta);
                }
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v1l))
            {
                curS1Bounds[3] = new LineCurve2D(new Point2D(u1f, v1l), new Direction2D(Direction2D.D.X));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS1Bounds[3] = new TrimmedCurve2D(curS1Bounds[3]!, 0, delta);
                }
            }
        }

        delta = v2l - v2f;
        if (Math.Abs(delta) > RealSmall())
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u2f))
            {
                curS2Bounds[0] = new LineCurve2D(new Point2D(u2f, v2f), new Direction2D(Direction2D.D.Y));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS2Bounds[0] = new TrimmedCurve2D(curS2Bounds[0]!, 0, delta);
                }
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u2l))
            {
                curS2Bounds[1] = new LineCurve2D(new Point2D(u2l, v2f), new Direction2D(Direction2D.D.Y));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS2Bounds[1] = new TrimmedCurve2D(curS2Bounds[1]!, 0, delta);
                }
            }
        }

        delta = u2l - u2f;
        if (Math.Abs(delta) > RealSmall())
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v2f))
            {
                curS2Bounds[2] = new LineCurve2D(new Point2D(u2f, v2f), new Direction2D(Direction2D.D.X));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS2Bounds[2] = new TrimmedCurve2D(curS2Bounds[2]!, 0, delta);
                }
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v2l))
            {
                curS2Bounds[3] = new LineCurve2D(new Point2D(u2f, v2l), new Direction2D(Direction2D.D.X));
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(delta))
                {
                    curS2Bounds[3] = new TrimmedCurve2D(curS2Bounds[3]!, 0, delta);
                }
            }
        }

        double intTol = 10.0 * global::Kerf.Engine.Foundation.Precision.Confusion();

        IntersectCurveAndBoundary(c2d1, curS1Bounds, numberOfCurves, intTol, arrayOfParameters);

        IntersectCurveAndBoundary(c2d2, curS2Bounds, numberOfCurves, intTol, arrayOfParameters);

        // MSVC's order (decision 621): only 0.0 and -0.0 tie.
        Sorting.Sort(CollectionsMarshal.AsSpan(arrayOfParameters), new DoubleLess());
    }

    /// <summary>The degree-1 B-spline through the points ideb to ifin of the walking line (knots 0, 1, 2, ...).</summary>
    public static Curve3D MakeBSpline(WLine wl, int ideb, int ifin)
    {
        int nbpnt = ifin - ideb + 1;
        NCollection_Array1<Point3D> poles = new(1, nbpnt);
        NCollection_Array1<double> knots = new(1, nbpnt);
        NCollection_Array1<int> mults = new(1, nbpnt);
        int i = 1, ipidebm1 = ideb;
        for (; i <= nbpnt; ipidebm1++, i++)
        {
            poles[i] = wl.Point(ipidebm1).Value();
            mults[i] = 1;
            knots[i] = i - 1;
        }
        mults[1] = mults[nbpnt] = 2;
        return new BSplineCurve3D(poles, knots, mults, 1);
    }

    /// <summary>The degree-1 2D B-spline through the parameters on the first (onFirst) or second surface of the
    /// points ideb to ifin of the walking line.</summary>
    public static BSplineCurve2D MakeBSpline2d(WLine wLine, int ideb, int ifin, bool onFirst)
    {
        int nbpnt = ifin - ideb + 1;
        NCollection_Array1<Point2D> poles = new(1, nbpnt);
        NCollection_Array1<double> knots = new(1, nbpnt);
        NCollection_Array1<int> mults = new(1, nbpnt);
        int i = 1, ipidebm1 = ideb;
        for (; i <= nbpnt; ipidebm1++, i++)
        {
            double u, v;
            if (onFirst)
            {
                wLine.Point(ipidebm1).ParametersOnS1(out u, out v);
            }
            else
            {
                wLine.Point(ipidebm1).ParametersOnS2(out u, out v);
            }
            poles[i] = poles[i].WithCoord(u, v);
            mults[i] = 1;
            knots[i] = i - 1;
        }

        mults[1] = mults[nbpnt] = 2;
        return new BSplineCurve2D(poles, knots, mults, 1);
    }
}
