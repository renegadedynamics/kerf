// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>The 2D tools of the Boolean component: pcurves of edges on faces, their adjustment to periodic surfaces.</summary>
/// <remarks>
/// <para>
/// A context argument may be null, as in OCCT: the functions then use a surface adaptor of their own.
/// </para>
/// <para>
/// <see cref="BuildPCurveForEdgeOnFace"/> and <see cref="AttachExistingPCurve"/> write into the edges they are given
/// (pcurves, tolerances, vertices' tolerances), as in OCCT: ported callers give them edges they built or copied
/// (plan 2.3). A frozen edge throws at the first write.
/// </para>
/// </remarks>
internal static class AlgoTools2D
{
    /// <summary>
    /// Computes the edge's pcurve on the face and stores it in the edge (with the larger of the edge's tolerance and the
    /// tolerance reached), unless the edge already has one on the face. Writes into <paramref name="aE"/>.
    /// </summary>
    public static void BuildPCurveForEdgeOnFace(in Edge aE, in Face aF, IntersectionContext? context = null)
    {
        BrepBuilder bb = new();
        Curve2D? c2D = null;
        double tolPC, tolFact, tolEdge;

        bool hasOld;
        hasOld = HasCurveOnSurface(aE, aF, ref c2D, out _, out _, out tolEdge);
        if (hasOld)
        {
            return;
        }

        CurveOnSurface(aE, aF, out c2D, out tolPC, context);

        tolEdge = Tool.Tolerance(aE);

        tolFact = Max(tolEdge, tolPC);

        bb.UpdateEdge(aE, c2D, aF, tolFact);
    }

    /// <summary>
    /// The unit tangent of the edge's 3D curve at the parameter, reversed for a REVERSED edge. False (and
    /// <paramref name="aTau"/> unchanged or unnormalised) for a degenerated edge or a null derivative.
    /// </summary>
    public static bool EdgeTangent(in Edge anEdge, double aT, ref Vector3D aTau)
    {
        bool isdgE;

        isdgE = Tool.Degenerated(anEdge);
        if (isdgE)
        {
            return false;
        }

        Curve3D c = Tool.Curve(anEdge, out _, out _)!;
        c.D1(aT, out _, out aTau);
        double mod = aTau.Magnitude;
        if (mod > Constants.Resolution())
        {
            aTau /= mod;
        }
        else
        {
            return false;
        }
        // aTau.Normalize();
        if (anEdge.Orientation == Reversed)
        {
            aTau = aTau.Reversed();
        }
        return true;
    }

    /// <summary>The UV point of the edge's pcurve on the face (made if missing) at the parameter.</summary>
    public static void PointOnSurface(in Edge aE, in Face aF, double aParameter, out double u, out double v,
                                      IntersectionContext? context = null)
    {
        Point2D p2D;

        CurveOnSurface(aE, aF, out Curve2D? c2D, out _, out _, out _, context);
        c2D!.D0(aParameter, out p2D);
        u = p2D.X;
        v = p2D.Y;
    }

    /// <summary>The edge's pcurve on the face, made by projection if the edge has none (not stored in the edge).</summary>
    public static void CurveOnSurface(in Edge aE, in Face aF, out Curve2D? aC2D, out double aToler,
                                      IntersectionContext? context = null)
    {
        //
        CurveOnSurface(aE, aF, out aC2D, out _, out _, out aToler, context);
        //
    }

    /// <inheritdoc cref="CurveOnSurface(in Edge, in Face, out Curve2D?, out double, IntersectionContext?)"/>
    public static void CurveOnSurface(in Edge aE, in Face aF, out Curve2D? aC2D, out double aFirst,
                                      out double aLast, out double aToler, IntersectionContext? context = null)
    {
        bool hasOld;
        Curve2D? c2D = null;

        hasOld = HasCurveOnSurface(aE, aF, ref c2D, out aFirst, out aLast, out aToler);
        if (hasOld)
        {
            aC2D = c2D;
            return;
        }

        Make2D(aE, aF, out c2D, out aFirst, out aLast, ref aToler, context);
        aC2D = c2D;
    }

    /// <summary>
    /// True if the edge has a pcurve on the face, returned with its range and the edge's tolerance. A range shorter
    /// than PConfusion gives false with <paramref name="aC2D"/> unchanged.
    /// </summary>
    public static bool HasCurveOnSurface(in Edge aE, in Face aF, ref Curve2D? aC2D, out double aFirst,
                                         out double aLast, out double aToler)
    {
        bool hasOld;

        aToler = Tool.Tolerance(aE);
        Tool.Range(aE, out aFirst, out aLast);

        if ((aLast - aFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        aC2D = Tool.CurveOnSurface(aE, aF, out aFirst, out aLast);
        hasOld = aC2D is not null;
        return hasOld;
    }

    /// <summary>True if the edge (with a range longer than PConfusion) has a pcurve on the face.</summary>
    public static bool HasCurveOnSurface(in Edge aE, in Face aF)
    {
        bool bHasOld;
        Curve2D? c2D;
        //
        Tool.Range(aE, out double first, out double last);
        //
        if ((last - first) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        //
        c2D = Tool.CurveOnSurface(aE, aF, out _, out _);
        bHasOld = c2D is not null;
        //
        return bHasOld;
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that it lies in the face's UV bounds (over the 3D curve's range).
    /// </summary>
    public static void AdjustPCurveOnFace(in Face f, Curve3D c3D, Curve2D c2D, out Curve2D? c2DA,
                                          IntersectionContext? context = null)
    {
        double t1 = c3D.FirstParameter;
        double t2 = c3D.LastParameter;
        //
        AdjustPCurveOnFace(f, t1, t2, c2D, out c2DA, context);
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that it lies in the face's UV bounds (over the given range).
    /// </summary>
    public static void AdjustPCurveOnFace(in Face f, double first, double last, Curve2D c2D,
                                          out Curve2D? c2DA, IntersectionContext? context = null)
    {
        FaceSurfaceAdaptor pBAS;
        if (context is not null)
        {
            pBAS = context.SurfaceAdaptor(f);
        }
        else
        {
            pBAS = new FaceSurfaceAdaptor();
            pBAS.Initialize(f, true);
        }
        //
        AdjustPCurveOnSurf(pBAS, first, last, c2D, out c2DA);
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that its point at mid-range lies in the adaptor's UV bounds (checked
    /// with a face classifier where the bounds exceed a period). Returns the pcurve itself or a translated copy.
    /// </summary>
    public static void AdjustPCurveOnSurf(FaceSurfaceAdaptor aBAS, double aFirst, double aLast, Curve2D aC2D,
                                          out Curve2D? aC2DA)
    {
        bool mincond, maxcond;
        double uMin, uMax, vMin, vMax, t, u2, v2, du, dv, delta;
        double uPeriod;
        //
        Face f = aBAS.Face();
        uMin = aBAS.FirstUParameter();
        uMax = aBAS.LastUParameter();
        vMin = aBAS.FirstVParameter();
        vMax = aBAS.LastVParameter();
        //
        delta = global::Kerf.Engine.Foundation.Precision.PConfusion();

        t = .5 * (aFirst + aLast);

        aC2D.D0(t, out Point2D pC2D);

        u2 = pC2D.X;
        v2 = pC2D.Y;
        //
        // du
        du = 0.0;
        if (aBAS.IsUPeriodic)
        {
            uPeriod = aBAS.UPeriod();

            //
            // a. try to clarify u2 using the precision (aDelta)
            if (Math.Abs(u2 - uMin) < delta)
            {
                u2 = uMin;
            }
            else if (Math.Abs(u2 - uMin - uPeriod) < delta)
            {
                u2 = uMin + uPeriod;
            }
            // b. compute du again using clarified value of u2
            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u2, uMin, uMax, uPeriod, out u2, out du, 0.0);
            //
            if (du == 0.0)
            {
                if (aBAS.GetType() == SurfaceKind.Cylinder)
                {
                    double r, dFi, tol;
                    //
                    Cylinder3D cylinder = aBAS.Cylinder();
                    r = cylinder.Radius;
                    tol = MaxToleranceEdge(f);
                    dFi = tol / r;
                    if (dFi < delta)
                    {
                        dFi = delta;
                    }
                    //
                    mincond = (uMin - u2 > dFi);
                    maxcond = (u2 - uMax > dFi);
                    if (mincond || maxcond)
                    {
                        du = (mincond) ? uPeriod : -uPeriod;
                    }
                }
            }
        }

        // dv
        dv = 0.0;
        if (aBAS.IsVPeriodic)
        {
            double vPeriod, vm, vr, vmid, dVm, dVr;
            //
            vPeriod = aBAS.VPeriod();
            mincond = (vMin - v2 > delta);
            maxcond = (v2 - vMax > delta);
            //
            if (mincond || maxcond)
            {
                dv = (mincond) ? vPeriod : -vPeriod;
            }
            //
            if ((vMax - vMin < vPeriod) && dv != 0.0)
            {
                vm = v2;
                vr = v2 + dv;
                vmid = 0.5 * (vMin + vMax);
                dVm = Math.Abs(vm - vmid);
                dVr = Math.Abs(vr - vmid);
                if (dVm < dVr)
                {
                    dv = 0.0;
                }
            }
        }
        //
        {
            // check the point with classifier
            double u, v;
            u = u2 + du;
            v = v2 + dv;
            if (aBAS.IsUPeriodic)
            {
                uPeriod = aBAS.UPeriod();
                if ((uMax - uMin - 2 * delta) > uPeriod)
                {
                    if ((u > (uMin + delta + uPeriod)) || (u < (uMax - delta - uPeriod)))
                    {
                        FaceClassifier classifier = new();
                        classifier.Perform(f, new Point2D(u, v), delta);
                        Containment status = classifier.State();
                        if (status == Out)
                        {
                            du += (u > (uMin + delta + uPeriod)) ? -uPeriod : uPeriod;
                        }
                    }
                }
            }
            //
            u = u2 + du;
            if (aBAS.IsVPeriodic)
            {
                double vPeriod = aBAS.VPeriod();
                if ((vMax - vMin - 2 * delta) > vPeriod)
                {
                    if ((v > (vMin + delta + vPeriod)) || (v < (vMax - delta - vPeriod)))
                    {
                        FaceClassifier classifier = new();
                        classifier.Perform(f, new Point2D(u, v), delta);
                        Containment status = classifier.State();
                        if (status == Out)
                        {
                            dv += (v > (vMin + delta + vPeriod)) ? -vPeriod : vPeriod;
                        }
                    }
                }
            }
        }
        // Translation if necessary
        Curve2D c2Dx = aC2D;

        if (du != 0.0 || dv != 0.0)
        {
            Curve2D pct = (Curve2D)c2Dx.Copy();
            Vector2D v2D = new(du, dv);
            pct.Translate(v2D);
            c2Dx = pct;
        }

        aC2DA = c2Dx;
    }

    /// <summary>A parameter inside [aFirst, aLast], at the fraction 10 e^-pi (0.43213918) of the range.</summary>
    public static double IntermediatePoint(double aFirst, double aLast)
    {
        // define parameter division number as 10*e^(-PI) = 0.43213918
        const double PAR_T = 0.43213918;
        double parm;
        parm = (1.0 - PAR_T) * aFirst + PAR_T * aLast;
        return parm;
    }

    /// <summary>The intermediate parameter of the edge's 3D curve range (of its range if it has no 3D curve).</summary>
    public static double IntermediatePoint(in Edge aE)
    {
        double t, t1, t2;

        Curve3D? c1 = Tool.Curve(aE, out t1, out t2);
        if (c1 is null)
        {
            Tool.Range(aE, out t1, out t2);
        }

        t = IntermediatePoint(t1, t2);
        return t;
    }

    /// <summary>
    /// The edge's pcurve on the face, made by projecting its 3D curve if it has none. <paramref name="aToler"/> is set
    /// to the tolerance reached only when the pcurve is made; otherwise it is left unchanged, as in OCCT.
    /// </summary>
    public static void Make2D(in Edge aE, in Face aF, out Curve2D? aC2D, out double aFirst, out double aLast,
                              ref double aToler, IntersectionContext? context = null)
    {
        bool locIdentity;
        double f3d, l3d;
        Placement loc;

        Curve2D? c2D;

        c2D = Tool.CurveOnSurface(aE, aF, out aFirst, out aLast);

        if (c2D is not null)
        {
            aC2D = c2D;
            return;
        }

        Curve3D? c3D2, c3D;
        c3D = Tool.Curve(aE, out loc, out f3d, out l3d);
        //
        if (c3D is null)
        {
            // aE has no 3D curve, so nothing is done
        }
        //
        locIdentity = loc.IsIdentity;

        if (locIdentity)
        {
            c3D2 = c3D;
        }
        else
        {
            c3D2 = (Curve3D)c3D!.Transformed(loc.Transformation);
        }

        //
        aToler = Tool.Tolerance(aE);
        MakePCurveOnFace(aF, c3D2!, f3d, l3d, out aC2D, ref aToler, context);
        //
        aFirst = f3d;
        aLast = l3d;
    }

    /// <summary>The pcurve of the 3D curve (over its whole range) on the face, by projection, and the tolerance reached.</summary>
    public static void MakePCurveOnFace(in Face aF, Curve3D aC3D, out Curve2D? aC2D, out double tolReached2d,
                                        IntersectionContext? context = null)
    {
        double first, last;

        first = aC3D.FirstParameter;
        last = aC3D.LastParameter;
        //
        tolReached2d = 0.0;
        //
        MakePCurveOnFace(aF, aC3D, first, last, out aC2D, ref tolReached2d, context);
    }

    /// <summary>
    /// The pcurve of the 3D curve over [aT1, aT2] on the face, by projection, adjusted to the face's periods and
    /// reparametrised to the range. <paramref name="tolReached2d"/> is read first (a tolerance the projection may use)
    /// and then set to the tolerance reached. Throws Standard_ConstructionError if no pcurve can be made.
    /// </summary>
    public static void MakePCurveOnFace(in Face aF, Curve3D aC3D, double aT1, double aT2, out Curve2D? aC2D,
                                        ref double tolReached2d, IntersectionContext? context = null)
    {
        FaceSurfaceAdaptor pBAS;
        if (context is not null)
        {
            pBAS = context.SurfaceAdaptor(aF);
        }
        else
        {
            pBAS = new FaceSurfaceAdaptor();
            pBAS.Initialize(aF, true);
        }
        //
        // OCCT copies the adaptor (new BRepAdaptor_Surface(*pBAS)); the port's copy is ShallowCopy.
        FaceSurfaceAdaptor bahs = (FaceSurfaceAdaptor)pBAS.ShallowCopy();
        CurveAdaptor bahc = new(aC3D, aT1, aT2);
        //
        double tolR;
        double tr = global::Kerf.Engine.Foundation.Precision.Confusion(); // 1.e-7;
        double maxTol = 1.0e3 * tr;       // 0.0001
        bool isAnaSurf = global::Kerf.Engine.Geometry.Projection.ProjLib.IsAnaSurf(bahs);

        // when the type of surface is GeomAbs_SurfaceOfRevolution
        if (pBAS.GetType() == SurfaceOfRevolution)
        {
            if (tolReached2d > tr)
            {
                tr = tolReached2d;
            }
            //
            ProjectedCurve proj1 = new(bahs, bahc, tr);
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(proj1, out aC2D);
            tolR = proj1.GetTolerance();
        }
        else
        {
            ProjectedCurve projCurv = new(bahs);
            int degMin = -1, degMax = -1, maxSegments = -1;
            double maxDist = -1;
            Constraint bndPnt = TangencyPoint;
            bool isExtendSurf = false;
            if ((tolReached2d >= 10.0 * tr) && (tolReached2d <= maxTol || isAnaSurf))
            {
                tr = Min(maxTol, 0.1 * tolReached2d);
                maxSegments = 100;
                maxDist = 1.0e3 * tolReached2d;
                if (!isAnaSurf || tolReached2d > 1.0)
                {
                    bndPnt = PassPoint;
                }
                isExtendSurf = true;
            }
            else if (tolReached2d > maxTol)
            {
                tr = Min(tolReached2d, 1.0e3 * maxTol);
                maxDist = 1.0e2 * tr;
                maxSegments = 100;
                isExtendSurf = true;
            }
            if (isExtendSurf)
            {
                SurfaceAdaptorBase? a3dSurf = null;
                double dt = (bahs.LastUParameter() - bahs.FirstUParameter());
                if (!bahs.IsUPeriodic || Math.Abs(dt - bahs.UPeriod()) > 0.01 * dt)
                {
                    dt *= 0.01;
                    a3dSurf = bahs.UTrim(bahs.FirstUParameter() - dt, bahs.LastUParameter() + dt, 0.0);
                }
                dt = (bahs.LastVParameter() - bahs.FirstVParameter());
                if (!bahs.IsVPeriodic || Math.Abs(dt - bahs.VPeriod()) > 0.01 * dt)
                {
                    dt *= 0.01;
                    a3dSurf = bahs.VTrim(bahs.FirstVParameter() - dt, bahs.LastVParameter() + dt, 0.0);
                }
                if (a3dSurf is not null)
                {
                    projCurv.Load(a3dSurf);
                }
            }
            projCurv.Load(tr);
            projCurv.SetDegree(degMin, degMax);
            projCurv.SetMaxSegments(maxSegments);
            projCurv.SetBndPnt(bndPnt);
            projCurv.SetMaxDist(maxDist);
            projCurv.Perform(bahc);
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(projCurv, out aC2D);
            tolR = projCurv.GetTolerance();
        }
        //
        if (aC2D is null && (tr < maxTol || tr < tolReached2d))
        {
            tr = Max(tolReached2d, maxTol);
            ProjectedCurve projCurvAgain = new(bahs, bahc, tr); // 2
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(projCurvAgain, out aC2D);
            tolR = projCurvAgain.GetTolerance();
        }
        //
        if (aC2D is null)
        {
            throw new ConstructionErrorException("BOPTools_AlgoTools2D::MakePCurveOnFace : PCurve is Null");
        }
        //
        tolReached2d = tolR;

        // Adjust curve for periodic surface
        AdjustPCurveOnSurf(pBAS, aT1, aT2, aC2D, out Curve2D? c2DA);
        aC2D = c2DA!;

        // Make sure that the range of the 2D curve is sufficient for representation of the 3D curve.
        double tcFirst = aC2D.FirstParameter;
        double tcLast = aC2D.LastParameter;
        if ((tcFirst - aT1) > global::Kerf.Engine.Foundation.Precision.PConfusion() || (aT2 - tcLast) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (tcFirst < aT1)
            {
                tcFirst = aT1;
            }
            if (tcLast > aT2)
            {
                tcLast = aT2;
            }

            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), aC2D, tcFirst, tcLast, aT1, aT2, ref aC2D);
        }

        // compute the appropriate tolerance for the edge
        Surface s = pBAS.GeomSurfaceTransformed()!;
        //
        if (IntersectionTools.ComputeTolerance(aC3D, aC2D!, s, aT1, aT2, out tolR, out _))
        {
            if (tolR > tolReached2d)
            {
                tolReached2d = tolR;
            }
        }
    }

    // The largest tolerance of the face's edges.
    private static double MaxToleranceEdge(in Face aF)
    {
        double tol, tolMax;
        Explorer exp = new();
        //
        tolMax = 0.0;
        exp.Init(aF, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            tol = Tool.Tolerance(e);
            if (tol > tolMax)
            {
                tolMax = tol;
            }
        }
        return tolMax;
    }

    /// <summary>
    /// Whether the edge's pcurve on the face is a U-isoline (tangent along V) or a V-isoline (tangent along U), within
    /// the angular precision, at its mid parameter.
    /// </summary>
    public static void IsEdgeIsoline(in Edge e, in Face f, out bool isTheUIso, out bool isTheVIso)
    {
        isTheUIso = isTheVIso = false;

        Curve2D pc = Tool.CurveOnSurface(e, f, out double first, out double last)!;

        pc.D1(0.5 * (first + last), out _, out Vector2D t);

        double sqMagn = t.SquareMagnitude;
        if (sqMagn <= Constants.Resolution())
        {
            return;
        }

        // Normalize aT
        t /= Math.Sqrt(sqMagn);

        // sin(da) ~ da, when da->0.
        double tol = global::Kerf.Engine.Foundation.Precision.Angular();
        Vector2D refVDir = new(0.0, 1.0), refUDir = new(1.0, 0.0);

        double dPv = t.CrossMagnitude(refVDir), dPu = t.CrossMagnitude(refUDir);

        isTheUIso = (dPv <= tol);
        isTheVIso = (dPu <= tol);
    }

    // ---------------------------------------------------------------- BOPTools_AlgoTools2D_1.cxx

    /// <summary>
    /// Gives the new edge <paramref name="e1"/> the pcurve of the old edge <paramref name="e2"/> on the face
    /// (reversed if the new edge is reversed relative to the old one, reparametrised, made same-parameter through a
    /// temporary edge, doubled for a seam). Returns 0 on success, 1 if the old edge has no pcurve, 2 if the
    /// reparametrisation fails, 3 if the tolerance can't be computed, 4 if it would be too large, 5 if the seam's
    /// second pcurve can't be made, 6 if same-parameter fails. Writes into <paramref name="e1"/> (its pcurves and
    /// tolerance, by BRep_Builder.Transfert) and its vertices (their tolerances).
    /// </summary>
    public static int AttachExistingPCurve(in Edge e2, // old
                                           in Edge e1, // new
                                           in Face f,
                                           IntersectionContext? aCtx)
    {
        bool bIsToReverse, bIsClosed, bComp;
        int iRet;
        double tol, t11, t12, t21, t22, tolPPC;
        double tolSP;
        Curve2D? c2Dold;
        Curve2D c2DoldC;
        Curve2D? c2DT;
        BrepBuilder bb = new();
        //
        iRet = 0;
        //
        Face aF = f;
        aF.Orient(Forward);
        Edge aE1 = e1;
        aE1.Orient(Forward);
        Edge aE2 = e2;
        aE2.Orient(Forward);
        //
        c2Dold = Tool.CurveOnSurface(aE2, aF, out t21, out t22);
        if (c2Dold is null)
        {
            iRet = 1;
            return iRet;
        }
        //
        c2DoldC = (Curve2D)c2Dold.Copy();
        //
        bIsToReverse = AlgoTools.IsSplitToReverse(aE1, aE2, aCtx);
        if (bIsToReverse)
        {
            double t21r, t22r;
            //
            c2DoldC.Reverse();
            //
            t21r = c2DoldC.ReversedParameter(t21);
            t22r = c2DoldC.ReversedParameter(t22);
            t21 = t22r;
            t22 = t21r;
        }
        //
        c2DT = new TrimmedCurve2D(c2DoldC, t21, t22);
        //
        tolPPC = global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        Curve3D ce1 = Tool.Curve(aE1, out t11, out t12)!;
        //
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolPPC, c2DT, t21, t22, t11, t12, ref c2DT);
        //
        if (c2DT is null)
        {
            iRet = 2;
            return iRet;
        }
        //
        // check the curves on same parameter to prevent
        // big tolerance increasing
        Surface sf = Tool.Surface(aF)!;
        //
        bComp = IntersectionTools.ComputeTolerance(ce1, c2DT, sf, t11, t12, out tolSP, out _, tolPPC);
        if (!bComp)
        {
            iRet = 3;
            return iRet;
        }
        //
        tol = Tool.Tolerance(aE1);
        //
        if ((tolSP > 10.0 * tol) && tolSP > 0.1)
        {
            iRet = 4;
            return iRet;
        }
        //
        // create a temporary edge to make same parameter pcurve
        Edge e1T = new();
        bb.MakeEdge(ref e1T, ce1, tol);
        bb.Range(e1T, t11, t12);
        bb.SameRange(e1T, false);
        bb.SameParameter(e1T, false);
        //
        bb.UpdateEdge(e1T, c2DT, aF, tol);
        try
        {
            BrepLibrary.SameParameter(e1T);
            BrepLibrary.SameRange(e1T);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            iRet = 6;
            return iRet;
        }
        //
        bIsClosed = IsClosed(aE2, aF);
        if (bIsClosed)
        {
            iRet = UpdateClosedPCurve(aE2, e1T, aF, aCtx);
            if (iRet != 0)
            {
                iRet = 5;
                return iRet;
            }
        }
        // transfer pcurve(s) from the temporary edge to the new edge
        bb.Transfert(e1T, aE1);
        // update tolerance of vertices
        double newTol = Tool.Tolerance(e1T);
        Iterator it = new(aE1);
        for (; it.More(); it.Next())
        {
            bb.UpdateVertex(ShapeCasts.Vertex(it.Value()), newTol);
        }
        return iRet;
    }

    // Builds the second pcurve of the new edge on a face where the old one is a seam. Writes into aEnew (the temporary
    // edge of AttachExistingPCurve). OCCT dereferences the context unchecked.
    private static int UpdateClosedPCurve(in Edge aEold, in Edge aEnew, in Face aF, IntersectionContext? aCtx)
    {
        bool bUClosed, bRevOrder;
        int nbPoints, iRet;
        double ts1, ts2, ts, scPr, us1, vs1, us2, vs2, t, u, v;
        double t1, t2, tol;
        Point2D p2DS1, p2DS2, p2D;
        Vector2D v2DT, v2D, v2DS1, v2DS2;
        Point3D p;
        Curve2D c2D, c2DS1, c2DS2, c2Dnew, c2DoldCT;
        TrimmedCurve2D c2DTnew;
        Surface s;
        Edge es;
        BrepBuilder bb = new();
        //
        iRet = 0;
        tol = Tool.Tolerance(aEnew);
        //
        // aC2DoldCT is alone p-curve of aEnew that we've built
        // The task is to build closed p-curves for aEnew
        c2DoldCT = Tool.CurveOnSurface(aEnew, aF, out t1, out t2)!;
        //
        // aC2Dold is p-curve of aEold
        _ = Tool.CurveOnSurface(aEold, aF, out t1, out t2);
        //
        // As aEold is closed on aF, it is possible to retrieve
        // the two p-curves:
        //  aC2DS1 -first p-curve
        //  aC2DS2 -second p-curve
        es = aEold;
        es.Orient(Forward);
        c2DS1 = Tool.CurveOnSurface(es, aF, out ts1, out ts2)!;
        //
        es.Orient(Reversed);
        c2DS2 = Tool.CurveOnSurface(es, aF, out ts1, out ts2)!;
        //
        ts = IntermediatePoint(ts1, ts2);
        //
        c2DS1.D1(ts, out p2DS1, out v2DS1);
        c2DS2.D1(ts, out p2DS2, out v2DS2);
        //
        // aV2DS12 - translation vector
        Vector2D v2DS12 = new(p2DS1, p2DS2);
        Direction2D d2DS12 = new(v2DS12);
        Direction2D d2DX = Constants.DX2d();
        //
        // Directoion of closeness: U-Closed or V-Closed
        scPr = d2DS12 * d2DX;
        bUClosed = true;
        if (Math.Abs(scPr) < tol)
        {
            bUClosed = !bUClosed;
        }
        //
        p2DS1.Coord(out us1, out vs1);
        p2DS2.Coord(out us2, out vs2);
        //
        // aP - some 3D-point on seam edge of the surface aS
        s = Tool.Surface(aF)!;
        s.D0(us1, vs1, out p);
        //
        ProjectPointOnCurve projPC = aCtx!.ProjPC(aEnew);
        //
        projPC.Perform(p);
        nbPoints = projPC.NbPoints();
        if (nbPoints == 0)
        {
            iRet = 1;
            return iRet;
        }
        //
        // aT - parameter for aP on the curves of aEnew
        t = projPC.LowerDistanceParameter();
        //
        c2D = c2DoldCT;
        c2D.D1(t, out p2D, out v2D);
        p2D.Coord(out u, out v);
        //
        c2Dnew = (Curve2D)c2D.Copy();
        c2DTnew = new TrimmedCurve2D(c2Dnew, t1, t2);
        //
        v2DT = v2DS12;
        if (!bUClosed)
        { // V Closed
            if (Math.Abs(v - vs2) < tol)
            {
                v2DT = v2DT.Reversed();
            }
        }
        else
        { // U Closed
            if (Math.Abs(u - us2) < tol)
            {
                v2DT = v2DT.Reversed();
            }
        }
        //
        // Translate aC2DTnew
        c2DTnew.Translate(v2DT);
        //
        // 4 Order the 2D curves
        bRevOrder = false;
        scPr = v2D * v2DS1;
        if (scPr < 0.0)
        {
            bRevOrder = !bRevOrder;
        }
        //
        if (!bRevOrder)
        {
            bb.UpdateEdge(aEnew, c2D, c2DTnew, aF, tol);
        }
        else
        {
            bb.UpdateEdge(aEnew, c2DTnew, c2D, aF, tol);
        }
        return iRet;
    }

    // True if the edge is closed on the face and found twice in it. OCCT: the loop condition (More() || iCnt == 2)
    // doesn't stop once both occurrences are counted at the end of the exploration: Next() on an exhausted explorer
    // restarts it (TopExp_Explorer::Next with an empty stack pushes the shape's iterator again), so the count reaches 4
    // and a true seam gives false (occt-issues.md). Kept; internal for the test that reproduces it.
    internal static bool IsClosed(in Edge aE, in Face aF)
    {
        bool bRet;
        //
        bRet = Tool.IsClosed(aE, aF);
        if (bRet)
        {
            int iCnt;
            //
            iCnt = 0;
            Explorer exp = new(aF, ShapeType.Edge);
            for (; (exp.More() || iCnt == 2); exp.Next()) // OCCT
            {
                Shape ex = exp.Current();
                if (ex.IsSame(aE))
                {
                    ++iCnt;
                }
            }
            bRet = (iCnt == 2);
        }
        return bRet;
    }
}
