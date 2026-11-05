// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx (MakeCurve)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using System.Runtime.InteropServices;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

internal partial class FaceFace
{
    /// <summary>
    /// Makes the curves of the intersection line of index Index and appends them to the result: trimmed lines,
    /// conics and circles with their projected 2D curves, approximated (or polygonal) B-splines for walking lines,
    /// approximated restriction lines trimmed on the surfaces' domains.
    /// </summary>
    protected void MakeCurve(int index, TopolTool dom1, TopolTool dom2, double toler)
    {
        bool bDone, rejectSurface, reApprox, bAvoidLineConstructor;
        bool ok, bPCurvesOk;
        int i, j, nbParts;
        double fprm, lprm;
        double tolpc;
        PatchLine l;
        PatchLineType typl;
        Curve3D newc;
        //
        const double tolcheck = 1.0e-7;
        const double tolangcheck = 1.0e-6;
        //
        rejectSurface = false;
        reApprox = false;
        //
        bPCurvesOk = true;

    reapprox:;

        tolpc = _tolApprox;
        bAvoidLineConstructor = false;
        l = _intersector.Line(index);
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

            int nbp = wLine.NbPnts();
            PntOn2S p1 = wLine.Point(1);
            PntOn2S p2 = wLine.Point(nbp);

            Point3D P1 = p1.Value();
            Point3D P2 = p2.Value();

            if (P1.SquareDistanceTo(P2) < 1.0e-14)
            {
                bAvoidLineConstructor = false;
            }
        }

        typl = l.ArcType();

        if (typl == PatchLineType.Restriction)
        {
            bAvoidLineConstructor = true;
        }

        //
        // Line Constructor
        if (!bAvoidLineConstructor)
        {
            _lConstruct.Perform(l);
            //
            bDone = _lConstruct.IsDone;
            if (!bDone)
            {
                return;
            }

            if (typl != PatchLineType.Restriction)
            {
                nbParts = _lConstruct.PartCount;
                if (nbParts <= 0)
                {
                    return;
                }
            }
        }
        // Do the Curve

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

                else
                {
                    newc = new HyperbolaCurve3D((l as GLine)!.Hyperbola());
                }
                //
                nbParts = _lConstruct.PartCount;
                for (i = 1; i <= nbParts; i++)
                {
                    bool bFNIt, bLPIt;
                    //
                    _lConstruct.Part(i, out fprm, out lprm);
                    //
                    bFNIt = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fprm);
                    bLPIt = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lprm);
                    //
                    if (!bFNIt && !bLPIt)
                    {
                        //
                        IntersectionCurve curve = new();
                        //
                        TrimmedCurve3D ct3D = new(newc, fprm, lprm);
                        curve.SetCurve(ct3D);
                        if (typl == PatchLineType.Parabola)
                        {
                            double tolC = IntersectionTools.CurveTolerance(ct3D, _tol);
                            curve.SetTolerance(tolC);
                        }
                        //
                        if (_approx1)
                        {
                            Curve2D? c2d = null;
                            IntSS.BuildPCurves(fprm, lprm, ref tolpc, _hs1.Surface(), newc, ref c2d);

                            // OCCT: when both projections fail BuildPCurves dereferences null first (unit 5G)
                            if (c2d is null)
                            {
                                continue;
                            }

                            curve.SetFirstCurve2d(new TrimmedCurve2D(c2d, fprm, lprm));
                        }
                        //
                        if (_approx2)
                        {
                            Curve2D? c2d = null;
                            IntSS.BuildPCurves(fprm, lprm, ref tolpc, _hs2.Surface(), newc, ref c2d);

                            if (c2d is null)
                            {
                                continue;
                            }

                            curve.SetSecondCurve2d(new TrimmedCurve2D(c2d, fprm, lprm));
                        }
                        //
                        _seqOfCurve.Append(curve);
                    } // if (!bFNIt && !bLPIt) {
                    else
                    {
                        //  on regarde si on garde
                        //
                        double testPrm, dT = 100.0;
                        //
                        testPrm = 0.0;
                        if (bFNIt && !bLPIt)
                        {
                            testPrm = lprm - dT;
                        }
                        else if (!bFNIt && bLPIt)
                        {
                            testPrm = fprm + dT;
                        }
                        else
                        {
                            // i.e, if (bFNIt && bLPIt)
                            testPrm = IntersectionTools.IntermediatePoint(-dT, dT);
                        }
                        //
                        Point3D ptref = newc.Value(testPrm);
                        //
                        SurfaceKind typS1 = _hs1.GetType();
                        SurfaceKind typS2 = _hs2.GetType();
                        if (typS1 == SurfaceOfExtrusion || typS1 == SurfaceKind.OffsetSurface
                            || typS1 == SurfaceOfRevolution || typS2 == SurfaceOfExtrusion
                            || typS2 == SurfaceKind.OffsetSurface || typS2 == SurfaceOfRevolution)
                        {
                            BSplineCurve2D? h1 = null;
                            _seqOfCurve.Append(new IntersectionCurve(newc, h1, h1));
                            continue;
                        }

                        double u1, v1, u2, v2, Tol;

                        Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                        Parameters(_hs1, _hs2, ptref, out u1, out v1, out u2, out v2);
                        ok = (dom1.Classify(new Point2D(u1, v1), Tol) != Out);
                        if (ok)
                        {
                            ok = (dom2.Classify(new Point2D(u2, v2), Tol) != Out);
                        }
                        if (ok)
                        {
                            BSplineCurve2D? h1 = null;
                            _seqOfCurve.Append(new IntersectionCurve(newc, h1, h1));
                        }
                    }
                } // for (i=1; i<=aNbParts; i++) {
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
                { // IntPatch_Ellipse
                    newc = new EllipseCurve3D((l as GLine)!.Ellipse());
                }
                //
                nbParts = _lConstruct.PartCount;
                //
                double period, nul;
                NCollection_Sequence<double> seqFprm = new(), seqLprm = new();

                nul = 0.0;
                period = M_PI + M_PI;

                for (i = 1; i <= nbParts; i++)
                {
                    _lConstruct.Part(i, out fprm, out lprm);

                    if (fprm < nul && lprm > nul)
                    {
                        // interval that goes through 0. is divided on two intervals;
                        while (fprm < nul || fprm > period)
                        {
                            fprm = fprm + period;
                        }
                        while (lprm < nul || lprm > period)
                        {
                            lprm = lprm + period;
                        }
                        //
                        if ((period - fprm) > tolpc)
                        {
                            seqFprm.Append(fprm);
                            seqLprm.Append(period);
                        }
                        else
                        {
                            Point3D P1 = newc.Value(fprm);
                            Point3D P2 = newc.Value(period);

                            if (P1.DistanceTo(P2) > _tol)
                            {
                                double anewpar = fprm;

                                if (ParameterOutOfBoundary(fprm, newc, _face1, _face2, lprm, false, _tol,
                                                           out anewpar, _context!))
                                {
                                    fprm = anewpar;
                                }
                                seqFprm.Append(fprm);
                                seqLprm.Append(period);
                            }
                        }

                        //
                        if ((lprm - nul) > tolpc)
                        {
                            seqFprm.Append(nul);
                            seqLprm.Append(lprm);
                        }
                        else
                        {
                            Point3D P1 = newc.Value(nul);
                            Point3D P2 = newc.Value(lprm);

                            if (P1.DistanceTo(P2) > _tol)
                            {
                                double anewpar = lprm;

                                if (ParameterOutOfBoundary(lprm, newc, _face1, _face2, fprm, true, _tol,
                                                           out anewpar, _context!))
                                {
                                    lprm = anewpar;
                                }
                                seqFprm.Append(nul);
                                seqLprm.Append(lprm);
                            }
                        }
                    }
                    else
                    {
                        // usual interval
                        seqFprm.Append(fprm);
                        seqLprm.Append(lprm);
                    }
                }
                //
                nbParts = seqFprm.Length();
                for (i = 1; i <= nbParts; i++)
                {
                    fprm = seqFprm.Value(i);
                    lprm = seqLprm.Value(i);
                    //
                    double realEpsilon = RealEpsilon();
                    if (Math.Abs(fprm) > realEpsilon || Math.Abs(lprm - 2.0 * M_PI) > realEpsilon)
                    {
                        //==============================================
                        ////
                        IntersectionCurve curve = new();
                        TrimmedCurve3D tc3D = new(newc, fprm, lprm);
                        curve.SetCurve(tc3D);
                        fprm = tc3D.FirstParameter;
                        lprm = tc3D.LastParameter;
                        ////
                        if (typl == PatchLineType.Circle || typl == PatchLineType.Ellipse)
                        { ////
                            if (_approx1)
                            {
                                Curve2D? c2d = null;
                                IntSS.BuildPCurves(fprm, lprm, _hs1.FirstUParameter(),
                                                           _hs1.LastUParameter(), _hs1.FirstVParameter(),
                                                           _hs1.LastVParameter(), ref tolpc, _hs1.Surface(), newc,
                                                           ref c2d);
                                curve.SetFirstCurve2d(c2d);
                            }

                            if (_approx2)
                            {
                                Curve2D? c2d = null;
                                IntSS.BuildPCurves(fprm, lprm, _hs2.FirstUParameter(),
                                                           _hs2.LastUParameter(), _hs2.FirstVParameter(),
                                                           _hs2.LastVParameter(), ref tolpc, _hs2.Surface(), newc,
                                                           ref c2d);
                                curve.SetSecondCurve2d(c2d);
                            }
                        }
                        //
                        _seqOfCurve.Append(curve);
                        //==============================================
                    } // if (std::abs(fprm) > RealEpsilon() || std::abs(lprm-2.*M_PI) > RealEpsilon())

                    else
                    {
                        //  on regarde si on garde
                        //
                        if (nbParts == 1)
                        {
                            //           if (std::abs(fprm) < RealEpsilon() &&  std::abs(lprm-2.*M_PI) <
                            //           RealEpsilon()) {
                            if (Math.Abs(fprm) <= realEpsilon && Math.Abs(lprm - 2.0 * M_PI) <= realEpsilon)
                            {
                                IntersectionCurve curve = new();
                                TrimmedCurve3D tc3D = new(newc, fprm, lprm);
                                curve.SetCurve(tc3D);
                                fprm = tc3D.FirstParameter;
                                lprm = tc3D.LastParameter;

                                if (_approx1)
                                {
                                    Curve2D? c2d = null;
                                    IntSS.BuildPCurves(fprm, lprm, ref tolpc, _hs1.Surface(), newc, ref c2d);
                                    curve.SetFirstCurve2d(c2d);
                                }

                                if (_approx2)
                                {
                                    Curve2D? c2d = null;
                                    IntSS.BuildPCurves(fprm, lprm, ref tolpc, _hs2.Surface(), newc, ref c2d);
                                    curve.SetSecondCurve2d(c2d);
                                }
                                //
                                _seqOfCurve.Append(curve);
                                break;
                            }
                        }
                        //
                        double twoPIdiv17, u1, v1, u2, v2, Tol;

                        twoPIdiv17 = 2.0 * M_PI / 17.0;

                        for (j = 0; j <= 17; j++)
                        {
                            Point3D ptref = newc.Value(j * twoPIdiv17);
                            Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

                            Parameters(_hs1, _hs2, ptref, out u1, out v1, out u2, out v2);
                            ok = (dom1.Classify(new Point2D(u1, v1), Tol) != Out);
                            if (ok)
                            {
                                ok = (dom2.Classify(new Point2D(u2, v2), Tol) != Out);
                            }
                            if (ok)
                            {
                                IntersectionCurve curve = new();
                                curve.SetCurve(newc);
                                //==============================================
                                if (typl == PatchLineType.Circle || typl == PatchLineType.Ellipse)
                                {

                                    if (_approx1)
                                    {
                                        Curve2D? c2d = null;
                                        IntSS.BuildPCurves(fprm, lprm, _hs1.FirstUParameter(),
                                                                   _hs1.LastUParameter(), _hs1.FirstVParameter(),
                                                                   _hs1.LastVParameter(), ref tolpc,
                                                                   _hs1.Surface(), newc, ref c2d);
                                        curve.SetFirstCurve2d(c2d);
                                    }

                                    if (_approx2)
                                    {
                                        Curve2D? c2d = null;
                                        IntSS.BuildPCurves(fprm, lprm, _hs2.FirstUParameter(),
                                                                   _hs2.LastUParameter(), _hs2.FirstVParameter(),
                                                                   _hs2.LastVParameter(), ref tolpc,
                                                                   _hs2.Surface(), newc, ref c2d);
                                        curve.SetSecondCurve2d(c2d);
                                    }
                                } //  end of if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
                                //==============================================
                                //
                                _seqOfCurve.Append(curve);
                                break;

                            } //  end of if (ok) {
                        } //  end of for (int j=0; j<=17; j++)
                    } //  end of else { on regarde si on garde
                } // for (i=1; i<=myLConstruct.NbParts(); i++)
            } // IntPatch_Circle: IntPatch_Ellipse:
            break;

            case PatchLineType.Analytic:
                // This case was processed earlier (in IntPatch_Intersection)
                break;

            case PatchLineType.Walking:
            {
                WLine wl = (l as WLine)!;

                // INTTOOLS_FACEFACE_DEBUG's WL->Dump(0) goes.

                //
                int ifprm, ilprm;
                //
                if (!_approx)
                {
                    nbParts = 1;
                    if (!bAvoidLineConstructor)
                    {
                        nbParts = _lConstruct.PartCount;
                    }
                    for (i = 1; i <= nbParts; ++i)
                    {
                        BSplineCurve2D? h1 = null, h2 = null;
                        Curve3D bSp;
                        //
                        if (bAvoidLineConstructor)
                        {
                            ifprm = 1;
                            ilprm = wl.NbPnts();
                        }
                        else
                        {
                            _lConstruct.Part(i, out fprm, out lprm);
                            ifprm = (int)fprm;
                            ilprm = (int)lprm;
                        }
                        //
                        if (_approx1)
                        {
                            h1 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, true);
                        }
                        //
                        if (_approx2)
                        {
                            h2 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, false);
                        }
                        //
                        bSp = IntSS.MakeBSpline(wl, ifprm, ilprm);
                        IntersectionCurve ic = new(bSp, h1, h2);
                        _seqOfCurve.Append(ic);
                    } // for (i=1; i<=aNbParts; ++i) {
                } // if (!myApprox) {
                //
                else
                { // X
                    bool bIsDecomposited;
                    int nbiter, nbSeqOfL;
                    double tol2d, tolApproxImp;
                    NCollection_Sequence<PatchLine> seqOfL = new();
                    WLApprox theapp3d = new();
                    global::Kerf.Geometry.ParametrizationType parType = global::Kerf.Geometry.ParametrizationType.ChordLength;
                    //
                    bool approx1 = _approx1;
                    bool approx2 = _approx2;
                    //
                    tolApproxImp = 1.0e-5;
                    tol2d = _tolApprox;

                    SurfaceKind typs1, typs2;
                    typs1 = _hs1.GetType();
                    typs2 = _hs2.GetType();
                    bool withPC = true;

                    if (typs1 == SurfaceKind.Cylinder && typs2 == SurfaceKind.Sphere)
                    {
                        withPC = ApproxWithPCurves(_hs1.Cylinder(), _hs2.Sphere());
                    }
                    else if (typs1 == SurfaceKind.Sphere && typs2 == SurfaceKind.Cylinder)
                    {
                        withPC = ApproxWithPCurves(_hs2.Cylinder(), _hs1.Sphere());
                    }
                    //
                    if (!withPC)
                    {
                        // OCCT overwrites the member: 1e-5 stays for the following lines and Performs
                        _tolApprox = tolApproxImp; // 1.e-5;
                        approx1 = false;
                        approx2 = false;
                        //
                        tol2d = _tolApprox;
                    }

                    bIsDecomposited = ToolWLineTool.DecompositionOfWLine(wl, _hs1, _hs2, _face1, _face2,
                                                                              _lConstruct, bAvoidLineConstructor,
                                                                              _tol, seqOfL, _context!);
                    //
                    nbSeqOfL = seqOfL.Length();
                    //
                    double tolC = 0.0;
                    if (bIsDecomposited)
                    {
                        nbiter = nbSeqOfL;
                        tolC = global::Kerf.Engine.Foundation.Precision.Confusion();
                    }
                    else
                    {
                        nbiter = 1;
                        nbParts = 1;
                        if (!bAvoidLineConstructor)
                        {
                            nbParts = _lConstruct.PartCount;
                            nbiter = nbParts;
                        }
                    }
                    //
                    for (i = 1; i <= nbiter; ++i)
                    {
                        if (bIsDecomposited)
                        {
                            wl = (seqOfL.Value(i) as WLine)!;
                            ifprm = 1;
                            ilprm = wl.NbPnts();
                        }
                        else
                        {
                            if (bAvoidLineConstructor)
                            {
                                ifprm = 1;
                                ilprm = wl.NbPnts();
                            }
                            else
                            {
                                _lConstruct.Part(i, out fprm, out lprm);
                                ifprm = (int)fprm;
                                ilprm = (int)lprm;
                            }
                        }

                        bool approx = _approx;
                        if (typs1 == Plane)
                        {
                            approx = false;
                            approx1 = true;
                        }
                        else if (typs2 == Plane)
                        {
                            approx = false;
                            approx2 = true;
                        }

                        parType = KnotTools.DefineParType(wl, ifprm, ilprm, approx, approx1, approx2);
                        if (_hs1 == _hs2)
                        {
                            theapp3d.SetParameters(_tolApprox, tol2d, 4, 8, 0, 30, false, parType);
                            rejectSurface = true;
                        }
                        else
                        {
                            if (reApprox && !rejectSurface)
                            {
                                theapp3d.SetParameters(_tolApprox, tol2d, 4, 8, 0, 30, false, parType);
                            }
                            else
                            {
                                int iDegMax, iDegMin, iNbIter;
                                //
                                ApproxParameters(_hs1, _hs2, out iDegMin, out iDegMax, out iNbIter);
                                theapp3d.SetParameters(_tolApprox, tol2d, iDegMin, iDegMax, iNbIter, 30, true, parType);
                            }
                        }

                        //-- lbr :
                        //-- Si une des surfaces est un plan , on approxime en 2d
                        //-- sur cette surface et on remonte les points 2d en 3d.
                        if (typs1 == Plane)
                        {
                            theapp3d.Perform(_hs1, _hs2, wl, false, true, _approx2, ifprm, ilprm);
                        }
                        else if (typs2 == Plane)
                        {
                            theapp3d.Perform(_hs1, _hs2, wl, false, _approx1, true, ifprm, ilprm);
                        }
                        else
                        {
                            //
                            if (_hs1 != _hs2)
                            {
                                if ((typs1 == SurfaceKind.BezierSurface || typs1 == SurfaceKind.BSplineSurface)
                                    && (typs2 == SurfaceKind.BezierSurface || typs2 == SurfaceKind.BSplineSurface))
                                {

                                    theapp3d.SetParameters(_tolApprox, tol2d, 4, 8, 0, 30, true, parType);

                                    bool bUseSurfaces;
                                    bUseSurfaces = ToolWLineTool.NotUseSurfacesForApprox(_face1, _face2, wl,
                                                                                              ifprm, ilprm);
                                    if (bUseSurfaces)
                                    {
                                        // ######
                                        rejectSurface = true;
                                        // ######
                                        theapp3d.SetParameters(_tolApprox, tol2d, 4, 8, 0, 30, false, parType);
                                    }
                                }
                            }
                            //
                            theapp3d.Perform(_hs1, _hs2, wl, true, approx1, approx2, ifprm, ilprm);
                        }
                        //
                        if (!theapp3d.IsDone())
                        {
                            BSplineCurve2D? h1 = null;
                            BSplineCurve2D? h2 = null;
                            //
                            Curve3D bSp = IntSS.MakeBSpline(wl, ifprm, ilprm);
                            //
                            if (_approx1)
                            {
                                h1 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, true);
                            }
                            //
                            if (_approx2)
                            {
                                h2 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, false);
                            }
                            //
                            IntersectionCurve ic = new(bSp, h1, h2);
                            _seqOfCurve.Append(ic);
                        }
                        else
                        {
                            if (typs1 == Plane || typs2 == Plane)
                            {
                                //
                                if (typs1 == SurfaceKind.Torus || typs2 == SurfaceKind.Torus)
                                {
                                    if (tolC < 1.0e-6)
                                    {
                                        tolC = 1.0e-6;
                                    }
                                }
                            }
                            //
                            int nbMultiCurves, nbpoles;
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
                                    Plane3D pln = _hs1.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik, global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X, tpoles2d.Value(ik).Y,
                                                                         pln));
                                    }
                                    //
                                    BSplineCurve3D bs = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, tolcheck, tolangcheck);
                                    check.FixTangent(true, true);
                                    //
                                    IntersectionCurve curve = new();
                                    curve.SetCurve(bs);

                                    if (_approx1)
                                    {
                                        BSplineCurve2D bs1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check1 = new(bs1, tolcheck, tolangcheck);
                                        check1.FixTangent(true, true);
                                        //
                                        // ############################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(bs1);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ############################################
                                        curve.SetFirstCurve2d(bs1);
                                    }

                                    if (_approx2)
                                    {
                                        mbspc.Curve(2, tpoles2d);

                                        BSplineCurve2D bs2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve newCheck = new(bs2, tolcheck, tolangcheck);
                                        newCheck.FixTangent(true, true);

                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(bs2);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        //
                                        curve.SetSecondCurve2d(bs2);
                                    }
                                    //
                                    curve.SetTolerance(tolC);
                                    //
                                    _seqOfCurve.Append(curve);

                                } // if(typs1 == GeomAbs_Plane) {

                                else if (typs2 == Plane)
                                {
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve((_approx1) ? 2 : 1, tpoles2d);
                                    Plane3D pln = _hs2.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik, global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X, tpoles2d.Value(ik).Y,
                                                                         pln));
                                    }
                                    //
                                    BSplineCurve3D bs = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, tolcheck, tolangcheck);
                                    check.FixTangent(true, true);
                                    //
                                    IntersectionCurve curve = new();
                                    curve.SetCurve(bs);
                                    curve.SetTolerance(tolC);

                                    if (_approx2)
                                    {
                                        BSplineCurve2D bs1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check1 = new(bs1, tolcheck, tolangcheck);
                                        check1.FixTangent(true, true);
                                        //
                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(bs1);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        bPCurvesOk = CheckPCurve(bs1, _face2, _context!);
                                        curve.SetSecondCurve2d(bs1);
                                    }

                                    if (_approx1)
                                    {
                                        mbspc.Curve(1, tpoles2d);
                                        BSplineCurve2D bs2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve check2 = new(bs2, tolcheck, tolangcheck);
                                        check2.FixTangent(true, true);
                                        //
                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(bs2);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        bPCurvesOk = bPCurvesOk && CheckPCurve(bs2, _face1, _context!);
                                        curve.SetFirstCurve2d(bs2);
                                    }
                                    //
                                    // if points of the pcurves are out of the faces bounds
                                    // create 3d and 2d curves without approximation
                                    if (!bPCurvesOk)
                                    {
                                        BSplineCurve2D? h1 = null, h2 = null;
                                        bPCurvesOk = true;
                                        //
                                        Curve3D bSp = IntSS.MakeBSpline(wl, ifprm, ilprm);

                                        if (_approx1)
                                        {
                                            h1 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, true);
                                            bPCurvesOk = CheckPCurve(h1, _face1, _context!);
                                        }

                                        if (_approx2)
                                        {
                                            h2 = IntSS.MakeBSpline2d(wl, ifprm, ilprm, false);
                                            bPCurvesOk = bPCurvesOk && CheckPCurve(h2, _face2, _context!);
                                        }
                                        //
                                        // if pcurves created without approximation are out of the
                                        // faces bounds, use approximated 3d and 2d curves
                                        if (bPCurvesOk)
                                        {
                                            IntersectionCurve ic = new(bSp, h1, h2, tolC);
                                            _seqOfCurve.Append(ic);
                                        }
                                        else
                                        {
                                            _seqOfCurve.Append(curve);
                                        }
                                    }
                                    else
                                    {
                                        _seqOfCurve.Append(curve);
                                    }

                                } // else if(typs2 == GeomAbs_Plane)
                                //
                                else
                                { // typs2 != GeomAbs_Plane && typs1 != GeomAbs_Plane
                                    bool bIsValid1, bIsValid2;
                                    BSplineCurve3D bs;
                                    IntersectionCurve curve = new();
                                    //
                                    bIsValid1 = true;
                                    bIsValid2 = true;
                                    //
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve(1, tpoles);
                                    bs = new BSplineCurve3D(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve check = new(bs, tolcheck, tolangcheck);
                                    check.FixTangent(true, true);
                                    //
                                    curve.SetCurve(bs);
                                    curve.SetTolerance(tolC);
                                    //
                                    if (_approx1)
                                    {
                                        if (approx1)
                                        {
                                            BSplineCurve2D bs1;
                                            NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                            mbspc.Curve(2, tpoles2d);
                                            //
                                            bs1 = new BSplineCurve2D(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                          mbspc.Degree());
                                            Check2dBSplineCurve newCheck = new(bs1, tolcheck, tolangcheck);
                                            newCheck.FixTangent(true, true);
                                            //
                                            if (!reApprox)
                                            {
                                                bIsValid1 = CheckPCurve(bs1, _face1, _context!);
                                            }
                                            //
                                            curve.SetFirstCurve2d(bs1);
                                        }
                                        else
                                        {
                                            BSplineCurve2D? bs1;
                                            fprm = bs.FirstParameter;
                                            lprm = bs.LastParameter;

                                            Curve2D? c2d = null;
                                            double tol = _tolApprox;
                                            IntSS.BuildPCurves(fprm, lprm, ref tol, _hs1.Surface(), bs, ref c2d);
                                            bs1 = c2d as BSplineCurve2D;
                                            curve.SetFirstCurve2d(bs1);
                                        }
                                    } // if(myApprox1) {
                                    //
                                    if (_approx2)
                                    {
                                        if (approx2)
                                        {
                                            BSplineCurve2D bs2;
                                            NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                            mbspc.Curve((_approx1) ? 3 : 2, tpoles2d);
                                            bs2 = new BSplineCurve2D(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                          mbspc.Degree());
                                            Check2dBSplineCurve newCheck = new(bs2, tolcheck, tolangcheck);
                                            newCheck.FixTangent(true, true);
                                            //
                                            if (!reApprox)
                                            {
                                                bIsValid2 = CheckPCurve(bs2, _face2, _context!);
                                            }
                                            curve.SetSecondCurve2d(bs2);
                                        }
                                        else
                                        {
                                            BSplineCurve2D? bs2;
                                            fprm = bs.FirstParameter;
                                            lprm = bs.LastParameter;

                                            Curve2D? c2d = null;
                                            double tol = _tolApprox;
                                            IntSS.BuildPCurves(fprm, lprm, ref tol, _hs2.Surface(), bs, ref c2d);
                                            bs2 = c2d as BSplineCurve2D;
                                            curve.SetSecondCurve2d(bs2);
                                        }
                                    } // if(myApprox2) {
                                    if (!bIsValid1 || !bIsValid2)
                                    {
                                        // OCCT overwrites the member (it stays 1e-5), and restarts the whole line
                                        // although its earlier parts are already appended (they are appended again)
                                        _tolApprox = tolApproxImp; // 1.e-5;
                                        tol2d = _tolApprox;
                                        reApprox = true;
                                        goto reapprox;
                                    }
                                    //
                                    _seqOfCurve.Append(curve);
                                }
                            }
                        }
                    }
                } // else { // X
            } // case IntPatch_Walking:{
            break;

            case PatchLineType.Restriction:
            {
                RLine rl = (l as RLine)!;

                // INTTOOLS_FACEFACE_DEBUG's RL->Dump(0) goes.

                Curve3D? c3d = null;
                Curve2D? c2d1 = null, c2d2 = null;
                double tolReached = 0.0;
                IntSS.TreatRLine(rl, _hs1, _hs2, ref c3d, ref c2d1, ref c2d2, ref tolReached);

                if (c3d is null)
                {
                    break;
                }

                Box2d box1 = new(), box2 = new();

                double u1f = _hs1.FirstUParameter(), v1f = _hs1.FirstVParameter(),
                       u1l = _hs1.LastUParameter(), v1l = _hs1.LastVParameter();
                double u2f = _hs2.FirstUParameter(), v2f = _hs2.FirstVParameter(),
                       u2l = _hs2.LastUParameter(), v2l = _hs2.LastVParameter();

                box1.Add(new Point2D(u1f, v1f));
                box1.Add(new Point2D(u1l, v1l));
                box2.Add(new Point2D(u2f, v2f));
                box2.Add(new Point2D(u2l, v2l));

                List<double> arrayOfParameters = new();

                // We consider here that the intersection line is same-parameter-line
                arrayOfParameters.Add(c3d.FirstParameter);
                arrayOfParameters.Add(c3d.LastParameter);

                IntSS.TrimILineOnSurfBoundaries(c2d1, c2d2, box1, box2, arrayOfParameters);

                // Intersect with true boundaries. After that, enlarge bounding-boxes in order to
                // correct definition, if point on curve is inscribed in the box.
                box1.Enlarge(toler);
                box2.Enlarge(toler);

                int nbIntersSolutionsm1 = arrayOfParameters.Count - 1;

                // Trim RLine found.
                for (int ind = 0; ind < nbIntersSolutionsm1; ind++)
                {
                    ref double parF = ref CollectionsMarshal.AsSpan(arrayOfParameters)[ind];
                    ref double parL = ref CollectionsMarshal.AsSpan(arrayOfParameters)[ind + 1];

                    if ((parL - parF) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // In order to more precise extending to the boundaries of source curves.
                        if (ind < nbIntersSolutionsm1 - 1)
                        {
                            parL = parF;
                        }

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

                        if (_approx1)
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

                        if (_approx2)
                        {
                            curv2d2 = new TrimmedCurve2D(c2d2, parF, parL);
                        }
                    }

                    Curve3D curv3d = new TrimmedCurve3D(c3d, parF, parL);

                    IntersectionCurve ic = new(curv3d, curv2d1, curv2d2);
                    _seqOfCurve.Append(ic);
                }
            }
            break;
            default:
                break;
        }
    }
}
