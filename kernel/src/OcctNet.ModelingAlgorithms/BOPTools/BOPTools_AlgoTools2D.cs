// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

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
public static class BOPTools_AlgoTools2D
{
    /// <summary>
    /// Computes the edge's pcurve on the face and stores it in the edge (with the larger of the edge's tolerance and the
    /// tolerance reached), unless the edge already has one on the face. Writes into <paramref name="aE"/>.
    /// </summary>
    public static void BuildPCurveForEdgeOnFace(in Edge aE, in Face aF, IntTools_Context? theContext = null)
    {
        BrepBuilder aBB = new();
        Curve2D? aC2D = null;
        double aTolPC, aTolFact, aTolEdge;

        bool aHasOld;
        aHasOld = HasCurveOnSurface(aE, aF, ref aC2D, out _, out _, out aTolEdge);
        if (aHasOld)
        {
            return;
        }

        CurveOnSurface(aE, aF, out aC2D, out aTolPC, theContext);

        aTolEdge = Tool.Tolerance(aE);

        aTolFact = Max(aTolEdge, aTolPC);

        aBB.UpdateEdge(aE, aC2D, aF, aTolFact);
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

        Curve3D aC = Tool.Curve(anEdge, out _, out _)!;
        aC.D1(aT, out _, out aTau);
        double mod = aTau.Magnitude();
        if (mod > Constants.Resolution())
        {
            aTau /= mod;
        }
        else
        {
            return false;
        }
        // aTau.Normalize();
        if (anEdge.Orientation() == Reversed)
        {
            aTau.Reverse();
        }
        return true;
    }

    /// <summary>The UV point of the edge's pcurve on the face (made if missing) at the parameter.</summary>
    public static void PointOnSurface(in Edge aE, in Face aF, double aParameter, out double U, out double V,
                                      IntTools_Context? theContext = null)
    {
        Point2D aP2D;

        CurveOnSurface(aE, aF, out Curve2D? aC2D, out _, out _, out _, theContext);
        aC2D!.D0(aParameter, out aP2D);
        U = aP2D.X();
        V = aP2D.Y();
    }

    /// <summary>The edge's pcurve on the face, made by projection if the edge has none (not stored in the edge).</summary>
    public static void CurveOnSurface(in Edge aE, in Face aF, out Curve2D? aC2D, out double aToler,
                                      IntTools_Context? theContext = null)
    {
        //
        CurveOnSurface(aE, aF, out aC2D, out _, out _, out aToler, theContext);
        //
    }

    /// <inheritdoc cref="CurveOnSurface(in Edge, in Face, out Curve2D?, out double, IntTools_Context?)"/>
    public static void CurveOnSurface(in Edge aE, in Face aF, out Curve2D? aC2D, out double aFirst,
                                      out double aLast, out double aToler, IntTools_Context? theContext = null)
    {
        bool aHasOld;
        Curve2D? C2D = null;

        aHasOld = HasCurveOnSurface(aE, aF, ref C2D, out aFirst, out aLast, out aToler);
        if (aHasOld)
        {
            aC2D = C2D;
            return;
        }

        Make2D(aE, aF, out C2D, out aFirst, out aLast, ref aToler, theContext);
        aC2D = C2D;
    }

    /// <summary>
    /// True if the edge has a pcurve on the face, returned with its range and the edge's tolerance. A range shorter
    /// than PConfusion gives false with <paramref name="aC2D"/> unchanged.
    /// </summary>
    public static bool HasCurveOnSurface(in Edge aE, in Face aF, ref Curve2D? aC2D, out double aFirst,
                                         out double aLast, out double aToler)
    {
        bool aHasOld;

        aToler = Tool.Tolerance(aE);
        Tool.Range(aE, out aFirst, out aLast);

        if ((aLast - aFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        aC2D = Tool.CurveOnSurface(aE, aF, out aFirst, out aLast);
        aHasOld = aC2D is not null;
        return aHasOld;
    }

    /// <summary>True if the edge (with a range longer than PConfusion) has a pcurve on the face.</summary>
    public static bool HasCurveOnSurface(in Edge aE, in Face aF)
    {
        bool bHasOld;
        Curve2D? aC2D;
        //
        Tool.Range(aE, out double aFirst, out double aLast);
        //
        if ((aLast - aFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        //
        aC2D = Tool.CurveOnSurface(aE, aF, out _, out _);
        bHasOld = aC2D is not null;
        //
        return bHasOld;
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that it lies in the face's UV bounds (over the 3D curve's range).
    /// </summary>
    public static void AdjustPCurveOnFace(in Face theF, Curve3D theC3D, Curve2D theC2D, out Curve2D? theC2DA,
                                          IntTools_Context? theContext = null)
    {
        double aT1 = theC3D.FirstParameter();
        double aT2 = theC3D.LastParameter();
        //
        AdjustPCurveOnFace(theF, aT1, aT2, theC2D, out theC2DA, theContext);
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that it lies in the face's UV bounds (over the given range).
    /// </summary>
    public static void AdjustPCurveOnFace(in Face theF, double theFirst, double theLast, Curve2D theC2D,
                                          out Curve2D? theC2DA, IntTools_Context? theContext = null)
    {
        FaceSurfaceAdaptor pBAS;
        if (theContext is not null)
        {
            pBAS = theContext.SurfaceAdaptor(theF);
        }
        else
        {
            pBAS = new FaceSurfaceAdaptor();
            pBAS.Initialize(theF, true);
        }
        //
        AdjustPCurveOnSurf(pBAS, theFirst, theLast, theC2D, out theC2DA);
    }

    /// <summary>
    /// Translates the pcurve by whole periods so that its point at mid-range lies in the adaptor's UV bounds (checked
    /// with a face classifier where the bounds exceed a period). Returns the pcurve itself or a translated copy.
    /// </summary>
    public static void AdjustPCurveOnSurf(FaceSurfaceAdaptor aBAS, double aFirst, double aLast, Curve2D aC2D,
                                          out Curve2D? aC2DA)
    {
        bool mincond, maxcond;
        double UMin, UMax, VMin, VMax, aT, u2, v2, du, dv, aDelta;
        double aUPeriod;
        //
        Face aF = aBAS.Face();
        UMin = aBAS.FirstUParameter();
        UMax = aBAS.LastUParameter();
        VMin = aBAS.FirstVParameter();
        VMax = aBAS.LastVParameter();
        //
        aDelta = global::Kerf.Engine.Foundation.Precision.PConfusion();

        aT = .5 * (aFirst + aLast);

        aC2D.D0(aT, out Point2D pC2D);

        u2 = pC2D.X();
        v2 = pC2D.Y();
        //
        // du
        du = 0.0;
        if (aBAS.IsUPeriodic)
        {
            aUPeriod = aBAS.UPeriod();

            //
            // a. try to clarify u2 using the precision (aDelta)
            if (Math.Abs(u2 - UMin) < aDelta)
            {
                u2 = UMin;
            }
            else if (Math.Abs(u2 - UMin - aUPeriod) < aDelta)
            {
                u2 = UMin + aUPeriod;
            }
            // b. compute du again using clarified value of u2
            GeomInt.AdjustPeriodic(u2, UMin, UMax, aUPeriod, out u2, out du, 0.0);
            //
            if (du == 0.0)
            {
                if (aBAS.GetType() == SurfaceKind.Cylinder)
                {
                    double aR, dFi, aTol;
                    //
                    Cylinder aCylinder = aBAS.Cylinder();
                    aR = aCylinder.Radius();
                    aTol = MaxToleranceEdge(aF);
                    dFi = aTol / aR;
                    if (dFi < aDelta)
                    {
                        dFi = aDelta;
                    }
                    //
                    mincond = (UMin - u2 > dFi);
                    maxcond = (u2 - UMax > dFi);
                    if (mincond || maxcond)
                    {
                        du = (mincond) ? aUPeriod : -aUPeriod;
                    }
                }
            }
        }

        // dv
        dv = 0.0;
        if (aBAS.IsVPeriodic)
        {
            double aVPeriod, aVm, aVr, aVmid, dVm, dVr;
            //
            aVPeriod = aBAS.VPeriod();
            mincond = (VMin - v2 > aDelta);
            maxcond = (v2 - VMax > aDelta);
            //
            if (mincond || maxcond)
            {
                dv = (mincond) ? aVPeriod : -aVPeriod;
            }
            //
            if ((VMax - VMin < aVPeriod) && dv != 0.0)
            {
                aVm = v2;
                aVr = v2 + dv;
                aVmid = 0.5 * (VMin + VMax);
                dVm = Math.Abs(aVm - aVmid);
                dVr = Math.Abs(aVr - aVmid);
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
                aUPeriod = aBAS.UPeriod();
                if ((UMax - UMin - 2 * aDelta) > aUPeriod)
                {
                    if ((u > (UMin + aDelta + aUPeriod)) || (u < (UMax - aDelta - aUPeriod)))
                    {
                        BRepClass_FaceClassifier aClassifier = new();
                        aClassifier.Perform(aF, new Point2D(u, v), aDelta);
                        Containment Status = aClassifier.State();
                        if (Status == Out)
                        {
                            du += (u > (UMin + aDelta + aUPeriod)) ? -aUPeriod : aUPeriod;
                        }
                    }
                }
            }
            //
            u = u2 + du;
            if (aBAS.IsVPeriodic)
            {
                double aVPeriod = aBAS.VPeriod();
                if ((VMax - VMin - 2 * aDelta) > aVPeriod)
                {
                    if ((v > (VMin + aDelta + aVPeriod)) || (v < (VMax - aDelta - aVPeriod)))
                    {
                        BRepClass_FaceClassifier aClassifier = new();
                        aClassifier.Perform(aF, new Point2D(u, v), aDelta);
                        Containment Status = aClassifier.State();
                        if (Status == Out)
                        {
                            dv += (v > (VMin + aDelta + aVPeriod)) ? -aVPeriod : aVPeriod;
                        }
                    }
                }
            }
        }
        // Translation if necessary
        Curve2D aC2Dx = aC2D;

        if (du != 0.0 || dv != 0.0)
        {
            Curve2D PCT = (Curve2D)aC2Dx.Copy();
            Vector2D aV2D = new(du, dv);
            PCT.Translate(aV2D);
            aC2Dx = PCT;
        }

        aC2DA = aC2Dx;
    }

    /// <summary>A parameter inside [aFirst, aLast], at the fraction 10 e^-pi (0.43213918) of the range.</summary>
    public static double IntermediatePoint(double aFirst, double aLast)
    {
        // define parameter division number as 10*e^(-PI) = 0.43213918
        const double PAR_T = 0.43213918;
        double aParm;
        aParm = (1.0 - PAR_T) * aFirst + PAR_T * aLast;
        return aParm;
    }

    /// <summary>The intermediate parameter of the edge's 3D curve range (of its range if it has no 3D curve).</summary>
    public static double IntermediatePoint(in Edge aE)
    {
        double aT, aT1, aT2;

        Curve3D? aC1 = Tool.Curve(aE, out aT1, out aT2);
        if (aC1 is null)
        {
            Tool.Range(aE, out aT1, out aT2);
        }

        aT = IntermediatePoint(aT1, aT2);
        return aT;
    }

    /// <summary>
    /// The edge's pcurve on the face, made by projecting its 3D curve if it has none. <paramref name="aToler"/> is set
    /// to the tolerance reached only when the pcurve is made; otherwise it is left unchanged, as in OCCT.
    /// </summary>
    public static void Make2D(in Edge aE, in Face aF, out Curve2D? aC2D, out double aFirst, out double aLast,
                              ref double aToler, IntTools_Context? theContext = null)
    {
        bool aLocIdentity;
        double f3d, l3d;
        Placement aLoc;

        Curve2D? C2D;

        C2D = Tool.CurveOnSurface(aE, aF, out aFirst, out aLast);

        if (C2D is not null)
        {
            aC2D = C2D;
            return;
        }

        Curve3D? C3D2, C3D;
        C3D = Tool.Curve(aE, out aLoc, out f3d, out l3d);
        //
        if (C3D is null)
        {
            // aE has no 3D curve, so nothing is done
        }
        //
        aLocIdentity = aLoc.IsIdentity;

        if (aLocIdentity)
        {
            C3D2 = C3D;
        }
        else
        {
            C3D2 = (Curve3D)C3D!.Transformed(aLoc.Transformation());
        }

        //
        aToler = Tool.Tolerance(aE);
        MakePCurveOnFace(aF, C3D2!, f3d, l3d, out aC2D, ref aToler, theContext);
        //
        aFirst = f3d;
        aLast = l3d;
    }

    /// <summary>The pcurve of the 3D curve (over its whole range) on the face, by projection, and the tolerance reached.</summary>
    public static void MakePCurveOnFace(in Face aF, Curve3D aC3D, out Curve2D? aC2D, out double TolReached2d,
                                        IntTools_Context? theContext = null)
    {
        double aFirst, aLast;

        aFirst = aC3D.FirstParameter();
        aLast = aC3D.LastParameter();
        //
        TolReached2d = 0.0;
        //
        MakePCurveOnFace(aF, aC3D, aFirst, aLast, out aC2D, ref TolReached2d, theContext);
    }

    /// <summary>
    /// The pcurve of the 3D curve over [aT1, aT2] on the face, by projection, adjusted to the face's periods and
    /// reparametrised to the range. <paramref name="TolReached2d"/> is read first (a tolerance the projection may use)
    /// and then set to the tolerance reached. Throws Standard_ConstructionError if no pcurve can be made.
    /// </summary>
    public static void MakePCurveOnFace(in Face aF, Curve3D aC3D, double aT1, double aT2, out Curve2D? aC2D,
                                        ref double TolReached2d, IntTools_Context? theContext = null)
    {
        FaceSurfaceAdaptor pBAS;
        if (theContext is not null)
        {
            pBAS = theContext.SurfaceAdaptor(aF);
        }
        else
        {
            pBAS = new FaceSurfaceAdaptor();
            pBAS.Initialize(aF, true);
        }
        //
        // OCCT copies the adaptor (new BRepAdaptor_Surface(*pBAS)); the port's copy is ShallowCopy.
        FaceSurfaceAdaptor aBAHS = (FaceSurfaceAdaptor)pBAS.ShallowCopy();
        CurveAdaptor aBAHC = new(aC3D, aT1, aT2);
        //
        double aTolR;
        double aTR = global::Kerf.Engine.Foundation.Precision.Confusion(); // 1.e-7;
        double aMaxTol = 1.0e3 * aTR;       // 0.0001
        bool isAnaSurf = global::Kerf.Engine.Geometry.Projection.ProjLib.IsAnaSurf(aBAHS);

        // when the type of surface is GeomAbs_SurfaceOfRevolution
        if (pBAS.GetType() == SurfaceOfRevolution)
        {
            if (TolReached2d > aTR)
            {
                aTR = TolReached2d;
            }
            //
            ProjectedCurve aProj1 = new(aBAHS, aBAHC, aTR);
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(aProj1, out aC2D);
            aTolR = aProj1.GetTolerance();
        }
        else
        {
            ProjectedCurve aProjCurv = new(aBAHS);
            int aDegMin = -1, aDegMax = -1, aMaxSegments = -1;
            double aMaxDist = -1;
            Constraint aBndPnt = TangencyPoint;
            bool isExtendSurf = false;
            if ((TolReached2d >= 10.0 * aTR) && (TolReached2d <= aMaxTol || isAnaSurf))
            {
                aTR = Min(aMaxTol, 0.1 * TolReached2d);
                aMaxSegments = 100;
                aMaxDist = 1.0e3 * TolReached2d;
                if (!isAnaSurf || TolReached2d > 1.0)
                {
                    aBndPnt = PassPoint;
                }
                isExtendSurf = true;
            }
            else if (TolReached2d > aMaxTol)
            {
                aTR = Min(TolReached2d, 1.0e3 * aMaxTol);
                aMaxDist = 1.0e2 * aTR;
                aMaxSegments = 100;
                isExtendSurf = true;
            }
            if (isExtendSurf)
            {
                SurfaceAdaptorBase? anA3dSurf = null;
                double dt = (aBAHS.LastUParameter() - aBAHS.FirstUParameter());
                if (!aBAHS.IsUPeriodic || Math.Abs(dt - aBAHS.UPeriod()) > 0.01 * dt)
                {
                    dt *= 0.01;
                    anA3dSurf = aBAHS.UTrim(aBAHS.FirstUParameter() - dt, aBAHS.LastUParameter() + dt, 0.0);
                }
                dt = (aBAHS.LastVParameter() - aBAHS.FirstVParameter());
                if (!aBAHS.IsVPeriodic || Math.Abs(dt - aBAHS.VPeriod()) > 0.01 * dt)
                {
                    dt *= 0.01;
                    anA3dSurf = aBAHS.VTrim(aBAHS.FirstVParameter() - dt, aBAHS.LastVParameter() + dt, 0.0);
                }
                if (anA3dSurf is not null)
                {
                    aProjCurv.Load(anA3dSurf);
                }
            }
            aProjCurv.Load(aTR);
            aProjCurv.SetDegree(aDegMin, aDegMax);
            aProjCurv.SetMaxSegments(aMaxSegments);
            aProjCurv.SetBndPnt(aBndPnt);
            aProjCurv.SetMaxDist(aMaxDist);
            aProjCurv.Perform(aBAHC);
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(aProjCurv, out aC2D);
            aTolR = aProjCurv.GetTolerance();
        }
        //
        if (aC2D is null && (aTR < aMaxTol || aTR < TolReached2d))
        {
            aTR = Max(TolReached2d, aMaxTol);
            ProjectedCurve aProjCurvAgain = new(aBAHS, aBAHC, aTR); // 2
            global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(aProjCurvAgain, out aC2D);
            aTolR = aProjCurvAgain.GetTolerance();
        }
        //
        if (aC2D is null)
        {
            throw new ConstructionErrorException("BOPTools_AlgoTools2D::MakePCurveOnFace : PCurve is Null");
        }
        //
        TolReached2d = aTolR;

        // Adjust curve for periodic surface
        AdjustPCurveOnSurf(pBAS, aT1, aT2, aC2D, out Curve2D? aC2DA);
        aC2D = aC2DA!;

        // Make sure that the range of the 2D curve is sufficient for representation of the 3D curve.
        double aTCFirst = aC2D.FirstParameter();
        double aTCLast = aC2D.LastParameter();
        if ((aTCFirst - aT1) > global::Kerf.Engine.Foundation.Precision.PConfusion() || (aT2 - aTCLast) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (aTCFirst < aT1)
            {
                aTCFirst = aT1;
            }
            if (aTCLast > aT2)
            {
                aTCLast = aT2;
            }

            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), aC2D, aTCFirst, aTCLast, aT1, aT2, ref aC2D);
        }

        // compute the appropriate tolerance for the edge
        Surface aS = pBAS.GeomSurfaceTransformed()!;
        //
        if (IntTools_Tools.ComputeTolerance(aC3D, aC2D!, aS, aT1, aT2, out aTolR, out _))
        {
            if (aTolR > TolReached2d)
            {
                TolReached2d = aTolR;
            }
        }
    }

    // The largest tolerance of the face's edges.
    private static double MaxToleranceEdge(in Face aF)
    {
        double aTol, aTolMax;
        Explorer aExp = new();
        //
        aTolMax = 0.0;
        aExp.Init(aF, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
            aTol = Tool.Tolerance(aE);
            if (aTol > aTolMax)
            {
                aTolMax = aTol;
            }
        }
        return aTolMax;
    }

    /// <summary>
    /// Whether the edge's pcurve on the face is a U-isoline (tangent along V) or a V-isoline (tangent along U), within
    /// the angular precision, at its mid parameter.
    /// </summary>
    public static void IsEdgeIsoline(in Edge theE, in Face theF, out bool isTheUIso, out bool isTheVIso)
    {
        isTheUIso = isTheVIso = false;

        Curve2D aPC = Tool.CurveOnSurface(theE, theF, out double aFirst, out double aLast)!;

        aPC.D1(0.5 * (aFirst + aLast), out _, out Vector2D aT);

        double aSqMagn = aT.SquareMagnitude();
        if (aSqMagn <= Constants.Resolution())
        {
            return;
        }

        // Normalize aT
        aT /= Math.Sqrt(aSqMagn);

        // sin(da) ~ da, when da->0.
        double aTol = global::Kerf.Engine.Foundation.Precision.Angular();
        Vector2D aRefVDir = new(0.0, 1.0), aRefUDir = new(1.0, 0.0);

        double aDPv = aT.CrossMagnitude(aRefVDir), aDPu = aT.CrossMagnitude(aRefUDir);

        isTheUIso = (aDPv <= aTol);
        isTheVIso = (aDPu <= aTol);
    }

    // ---------------------------------------------------------------- BOPTools_AlgoTools2D_1.cxx

    /// <summary>
    /// Gives the new edge <paramref name="theE1"/> the pcurve of the old edge <paramref name="theE2"/> on the face
    /// (reversed if the new edge is reversed relative to the old one, reparametrised, made same-parameter through a
    /// temporary edge, doubled for a seam). Returns 0 on success, 1 if the old edge has no pcurve, 2 if the
    /// reparametrisation fails, 3 if the tolerance can't be computed, 4 if it would be too large, 5 if the seam's
    /// second pcurve can't be made, 6 if same-parameter fails. Writes into <paramref name="theE1"/> (its pcurves and
    /// tolerance, by BRep_Builder.Transfert) and its vertices (their tolerances).
    /// </summary>
    public static int AttachExistingPCurve(in Edge theE2, // old
                                           in Edge theE1, // new
                                           in Face theF,
                                           IntTools_Context? aCtx)
    {
        bool bIsToReverse, bIsClosed, bComp;
        int iRet;
        double aTol, aT11, aT12, aT21, aT22, aTolPPC;
        double aTolSP;
        Curve2D? aC2Dold;
        Curve2D aC2DoldC;
        Curve2D? aC2DT;
        BrepBuilder aBB = new();
        //
        iRet = 0;
        //
        Face aF = theF;
        aF.Orientation(Forward);
        Edge aE1 = theE1;
        aE1.Orientation(Forward);
        Edge aE2 = theE2;
        aE2.Orientation(Forward);
        //
        aC2Dold = Tool.CurveOnSurface(aE2, aF, out aT21, out aT22);
        if (aC2Dold is null)
        {
            iRet = 1;
            return iRet;
        }
        //
        aC2DoldC = (Curve2D)aC2Dold.Copy();
        //
        bIsToReverse = BOPTools_AlgoTools.IsSplitToReverse(aE1, aE2, aCtx);
        if (bIsToReverse)
        {
            double aT21r, aT22r;
            //
            aC2DoldC.Reverse();
            //
            aT21r = aC2DoldC.ReversedParameter(aT21);
            aT22r = aC2DoldC.ReversedParameter(aT22);
            aT21 = aT22r;
            aT22 = aT21r;
        }
        //
        aC2DT = new TrimmedCurve2D(aC2DoldC, aT21, aT22);
        //
        aTolPPC = global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        Curve3D aCE1 = Tool.Curve(aE1, out aT11, out aT12)!;
        //
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(aTolPPC, aC2DT, aT21, aT22, aT11, aT12, ref aC2DT);
        //
        if (aC2DT is null)
        {
            iRet = 2;
            return iRet;
        }
        //
        // check the curves on same parameter to prevent
        // big tolerance increasing
        Surface aSF = Tool.Surface(aF)!;
        //
        bComp = IntTools_Tools.ComputeTolerance(aCE1, aC2DT, aSF, aT11, aT12, out aTolSP, out _, aTolPPC);
        if (!bComp)
        {
            iRet = 3;
            return iRet;
        }
        //
        aTol = Tool.Tolerance(aE1);
        //
        if ((aTolSP > 10.0 * aTol) && aTolSP > 0.1)
        {
            iRet = 4;
            return iRet;
        }
        //
        // create a temporary edge to make same parameter pcurve
        Edge aE1T = new();
        aBB.MakeEdge(ref aE1T, aCE1, aTol);
        aBB.Range(aE1T, aT11, aT12);
        aBB.SameRange(aE1T, false);
        aBB.SameParameter(aE1T, false);
        //
        aBB.UpdateEdge(aE1T, aC2DT, aF, aTol);
        try
        {
            BRepLib.SameParameter(aE1T);
            BRepLib.SameRange(aE1T);
        }
        catch (FailureException)
        {
            iRet = 6;
            return iRet;
        }
        //
        bIsClosed = IsClosed(aE2, aF);
        if (bIsClosed)
        {
            iRet = UpdateClosedPCurve(aE2, aE1T, aF, aCtx);
            if (iRet != 0)
            {
                iRet = 5;
                return iRet;
            }
        }
        // transfer pcurve(s) from the temporary edge to the new edge
        aBB.Transfert(aE1T, aE1);
        // update tolerance of vertices
        double aNewTol = Tool.Tolerance(aE1T);
        Iterator it = new(aE1);
        for (; it.More(); it.Next())
        {
            aBB.UpdateVertex(ShapeCasts.Vertex(it.Value()), aNewTol);
        }
        return iRet;
    }

    // Builds the second pcurve of the new edge on a face where the old one is a seam. Writes into aEnew (the temporary
    // edge of AttachExistingPCurve). OCCT dereferences the context unchecked.
    private static int UpdateClosedPCurve(in Edge aEold, in Edge aEnew, in Face aF, IntTools_Context? aCtx)
    {
        bool bUClosed, bRevOrder;
        int aNbPoints, iRet;
        double aTS1, aTS2, aTS, aScPr, aUS1, aVS1, aUS2, aVS2, aT, aU, aV;
        double aT1, aT2, aTol;
        Point2D aP2DS1, aP2DS2, aP2D;
        Vector2D aV2DT, aV2D, aV2DS1, aV2DS2;
        Point3D aP;
        Curve2D aC2D, aC2DS1, aC2DS2, aC2Dnew, aC2DoldCT;
        TrimmedCurve2D aC2DTnew;
        Surface aS;
        Edge aES;
        BrepBuilder aBB = new();
        //
        iRet = 0;
        aTol = Tool.Tolerance(aEnew);
        //
        // aC2DoldCT is alone p-curve of aEnew that we've built
        // The task is to build closed p-curves for aEnew
        aC2DoldCT = Tool.CurveOnSurface(aEnew, aF, out aT1, out aT2)!;
        //
        // aC2Dold is p-curve of aEold
        _ = Tool.CurveOnSurface(aEold, aF, out aT1, out aT2);
        //
        // As aEold is closed on aF, it is possible to retrieve
        // the two p-curves:
        //  aC2DS1 -first p-curve
        //  aC2DS2 -second p-curve
        aES = aEold;
        aES.Orientation(Forward);
        aC2DS1 = Tool.CurveOnSurface(aES, aF, out aTS1, out aTS2)!;
        //
        aES.Orientation(Reversed);
        aC2DS2 = Tool.CurveOnSurface(aES, aF, out aTS1, out aTS2)!;
        //
        aTS = IntermediatePoint(aTS1, aTS2);
        //
        aC2DS1.D1(aTS, out aP2DS1, out aV2DS1);
        aC2DS2.D1(aTS, out aP2DS2, out aV2DS2);
        //
        // aV2DS12 - translation vector
        Vector2D aV2DS12 = new(aP2DS1, aP2DS2);
        Direction2D aD2DS12 = new(aV2DS12);
        Direction2D aD2DX = Constants.DX2d();
        //
        // Directoion of closeness: U-Closed or V-Closed
        aScPr = aD2DS12 * aD2DX;
        bUClosed = true;
        if (Math.Abs(aScPr) < aTol)
        {
            bUClosed = !bUClosed;
        }
        //
        aP2DS1.Coord(out aUS1, out aVS1);
        aP2DS2.Coord(out aUS2, out aVS2);
        //
        // aP - some 3D-point on seam edge of the surface aS
        aS = Tool.Surface(aF)!;
        aS.D0(aUS1, aVS1, out aP);
        //
        GeomAPI_ProjectPointOnCurve aProjPC = aCtx!.ProjPC(aEnew);
        //
        aProjPC.Perform(aP);
        aNbPoints = aProjPC.NbPoints();
        if (aNbPoints == 0)
        {
            iRet = 1;
            return iRet;
        }
        //
        // aT - parameter for aP on the curves of aEnew
        aT = aProjPC.LowerDistanceParameter();
        //
        aC2D = aC2DoldCT;
        aC2D.D1(aT, out aP2D, out aV2D);
        aP2D.Coord(out aU, out aV);
        //
        aC2Dnew = (Curve2D)aC2D.Copy();
        aC2DTnew = new TrimmedCurve2D(aC2Dnew, aT1, aT2);
        //
        aV2DT = aV2DS12;
        if (!bUClosed)
        { // V Closed
            if (Math.Abs(aV - aVS2) < aTol)
            {
                aV2DT.Reverse();
            }
        }
        else
        { // U Closed
            if (Math.Abs(aU - aUS2) < aTol)
            {
                aV2DT.Reverse();
            }
        }
        //
        // Translate aC2DTnew
        aC2DTnew.Translate(aV2DT);
        //
        // 4 Order the 2D curves
        bRevOrder = false;
        aScPr = aV2D * aV2DS1;
        if (aScPr < 0.0)
        {
            bRevOrder = !bRevOrder;
        }
        //
        if (!bRevOrder)
        {
            aBB.UpdateEdge(aEnew, aC2D, aC2DTnew, aF, aTol);
        }
        else
        {
            aBB.UpdateEdge(aEnew, aC2DTnew, aC2D, aF, aTol);
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
            Explorer aExp = new(aF, ShapeType.Edge);
            for (; (aExp.More() || iCnt == 2); aExp.Next()) // OCCT
            {
                Shape aEx = aExp.Current();
                if (aEx.IsSame(aE))
                {
                    ++iCnt;
                }
            }
            bRet = (iCnt == 2);
        }
        return bRet;
    }
}
