// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_ProjectCurveOnSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_ProjectCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Computes a pcurve by projecting a 3D curve onto a surface. The projection uses 23 points or more (for B-splines,
/// at least the number of spans times degree + 1, growing by 22 from 23). Isoparametric cases (the curve on a
/// U = const or V = const line) are recognised with the given precision.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The projector keeps a cache of the last pcurve's end points, so projecting the
/// edges of a wire one after the other keeps their pcurves connected across a seam, as in OCCT: one projector per
/// loop body in parallel code.
/// </para>
/// <para>
/// <see cref="Perform"/> freezes the pcurve it hands out (docs/phase3-design.md, section 4.2); ported code (ShapeFix)
/// calls the internal, virtual <see cref="InternalPerform"/>, which doesn't freeze what it builds: the pcurve is new
/// and unfrozen, but for a projection on a plane, where (as OCCT) it is the basis of ProjLib's trimmed result, which
/// <c>BasisCurve()</c> has frozen; ported code copies a pcurve before editing it in place. The 3D curve and the surface
/// are only read.
/// </para>
/// </remarks>
public class ShapeConstruct_ProjectCurveOnSurface : Transient
{
    //! Default number of control points for discretization.
    private const int THE_NCONTROL = 23;

    private ShapeAnalysis_Surface? mySurf; // Surface to project on
    private double myPreci; // Current precision
    private int myStatus; // Operation status
    private int myAdjustOverDegen; // Seam adjustment flag

    // Cached 3D/2D point pairs for projection optimization (0-based, OCCT's CacheArray)
    private (Point3D first, Point2D second)[] myCache = [];

    /// <summary>An empty projector: Init or SetSurface, then Perform.</summary>
    public ShapeConstruct_ProjectCurveOnSurface()
    {
        myPreci = global::Kerf.Engine.Foundation.Precision.Confusion();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myAdjustOverDegen = 1;
    }

    /// <summary>Sets the surface to project on and the precision.</summary>
    public virtual void Init(Geom_Surface theSurf, double thePreci)
    {
        Init(new ShapeAnalysis_Surface(theSurf), thePreci);
    }

    /// <summary>Sets the surface (as a ShapeAnalysis_Surface, shared) to project on and the precision.</summary>
    public virtual void Init(ShapeAnalysis_Surface theSurf, double thePreci)
    {
        SetSurface(theSurf);
        SetPrecision(thePreci);
    }

    /// <summary>Sets the surface to project on.</summary>
    public void SetSurface(Geom_Surface theSurf)
    {
        SetSurface(new ShapeAnalysis_Surface(theSurf));
    }

    /// <summary>Sets the surface (as a ShapeAnalysis_Surface, shared) to project on; clears the end-point cache.</summary>
    public void SetSurface(ShapeAnalysis_Surface theSurf)
    {
        if (ReferenceEquals(mySurf, theSurf))
        {
            return;
        }
        mySurf = theSurf;
        myCache = [];
    }

    /// <summary>Sets the precision.</summary>
    public void SetPrecision(double thePreci)
    {
        myPreci = thePreci;
    }

    /// <summary>
    /// The side of the parametric space to which a part of the pcurve lying on the seam of a closed surface with a
    /// singularity (a sphere) is adjusted: 1 (default) the left side (U = 0 on a sphere), 0 the right side, -1 none.
    /// OCCT's modifiable <c>int&amp;</c>.
    /// </summary>
    public ref int AdjustOverDegenMode() => ref myAdjustOverDegen;

    /// <summary>True if the status of the last Perform has the bit <paramref name="theStatus"/>.</summary>
    public bool Status(ShapeExtend_Status theStatus) => ShapeExtend.DecodeStatus(myStatus, theStatus);

    /// <summary>
    /// Projects the 3D curve on [<paramref name="theFirst"/>, <paramref name="theLast"/>] onto the surface. The pcurve
    /// (frozen) is same-parameter with the 3D curve on that range; a straight pcurve may be an infinite line.
    /// Returns false if the projection fails.
    /// </summary>
    public bool Perform(Geom_Curve? theC3D, double theFirst, double theLast, out Geom2d_Curve? theC2D,
                        double theTolFirst = Precision_Confusion, double theTolLast = Precision_Confusion)
    {
        bool aResult = InternalPerform(theC3D, theFirst, theLast, out theC2D, theTolFirst, theTolLast);
        theC2D?.Freeze();
        return aResult;
    }

    // Precision::Confusion() as a constant default argument.
    private const double Precision_Confusion = 1.0e-7;

    /// <summary>The projection, as <see cref="Perform"/>, without freezing the pcurve (ported code; OCCT's virtual).</summary>
    internal virtual bool InternalPerform(Geom_Curve? theC3D, double theFirst, double theLast, out Geom2d_Curve? theC2D,
                                          double theTolFirst = Precision_Confusion, double theTolLast = Precision_Confusion)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        if (mySurf is null || theC3D is null)
        {
            theC2D = null;
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        // Try analytical projection first
        Geom_Curve aCurve3DTrim = theC3D;
        if (theC3D is not Geom_BoundedCurve)
        {
            aCurve3DTrim = new Geom_TrimmedCurve(theC3D, theFirst, theLast);
        }

        theC2D = projectAnalytic(aCurve3DTrim);
        if (theC2D is not null)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return true;
        }

        // Handle problematic B-spline curves with uneven parameterization
        if (isBSplineCurveInvalid(theC3D, theFirst, theLast, out _))
        {
            // Use ProjLib for curves with highly uneven parameterization speed
            if (PerformByProjLib(theC3D, theFirst, theLast, out theC2D))
            {
                return Status(ShapeExtend_DONE);
            }
        }

        // Generate curve points
        NCollection_Array1<Point3D> aPoints = new();
        NCollection_Array1<double> aParams = new();
        int aNbPnt = generateCurvePoints(theC3D, theFirst, theLast, THE_NCONTROL, aPoints, aParams);

        // Approximate pcurve
        NCollection_Array1<Point2D> aPoints2d = new(1, aNbPnt);
        for (int i = 1; i <= aNbPnt; i++)
        {
            aPoints2d.SetValue(i, new Point2D(0.0, 0.0));
        }

        approxPCurve(aNbPnt, theC3D, theTolFirst, theTolLast, ref aPoints, ref aParams, ref aPoints2d, out theC2D);

        int aNbPini = aPoints.Length();
        if (theC2D is not null)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            return true;
        }

        // Interpolate the result
        theC2D = interpolatePCurve(aNbPini, aPoints2d, aParams);

        myStatus |= ShapeExtend.EncodeStatus(theC2D is null ? ShapeExtend_FAIL1 : ShapeExtend_DONE2);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>
    /// Projects the 3D curve with ProjLib's standard projector. Returns false if it fails, raises an exception or cuts
    /// the curve by the surface's bounds.
    /// </summary>
    protected bool PerformByProjLib(Geom_Curve theC3D, double theFirst, double theLast, out Geom2d_Curve? theC2D)
    {
        theC2D = null;
        if (mySurf is null)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        try
        {
            GeomAdaptor_Surface? aGAS = mySurf.Adaptor3d();
            GeomAdaptor_Curve aGAC = new(theC3D, theFirst, theLast);
            ProjLib_ProjectedCurve aProjector = new(aGAS!, aGAC);

            switch (aProjector.GetType())
            {
                case Line:
                    theC2D = new Geom2d_Line(aProjector.Line());
                    break;
                case Circle:
                    theC2D = new Geom2d_Circle(aProjector.Circle());
                    break;
                case Ellipse:
                    theC2D = new Geom2d_Ellipse(aProjector.Ellipse());
                    break;
                case Parabola:
                    theC2D = new Geom2d_Parabola(aProjector.Parabola());
                    break;
                case Hyperbola:
                    theC2D = new Geom2d_Hyperbola(aProjector.Hyperbola());
                    break;
                case BSplineCurve:
                    theC2D = aProjector.BSpline();
                    break;
                default:
                    break;
            }

            if (theC2D is null)
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }
            else
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                return true;
            }
        }
        catch (FailureException)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            theC2D = null;
        }
        return false;
    }

    /// <summary>The analytic projection on a plane (also trimmed or offset), or null.</summary>
    protected Geom2d_Curve? projectAnalytic(Geom_Curve theC3D)
    {
        Geom2d_Curve? aResult = null;

        Geom_Surface? aSurf = mySurf!.Surface();
        Geom_Plane? aPlane = aSurf as Geom_Plane;

        if (aPlane is null)
        {
            if (aSurf is Geom_RectangularTrimmedSurface aRTS)
            {
                aPlane = aRTS.BasisSurface() as Geom_Plane;
            }
            else
            {
                if (aSurf is Geom_OffsetSurface anOS)
                {
                    aPlane = anOS.BasisSurface() as Geom_Plane;
                }
            }
        }

        if (aPlane is not null)
        {
            Geom_Curve? aProjOnPlane = GeomProjLib.ProjectOnPlane(theC3D, aPlane, aPlane.Position().Direction(), true);
            GeomAdaptor_Curve aHC = new(aProjOnPlane!);
            ProjLib_ProjectedCurve aProj = new(mySurf.Adaptor3d()!, aHC);

            aResult = Geom2dAdaptor.MakeCurve(aProj);
            if (aResult is null)
            {
                return aResult;
            }

            if (aResult is Geom2d_TrimmedCurve aTC)
            {
                aResult = aTC.BasisCurve();
            }

            return aResult;
        }

        return aResult;
    }

    // OCCT's projectPoint lambda of getLine: projects a point, first through the end-point cache.
    private void projectPoint(SurfaceProjectorWithCache aProjector, in Point3D thePoint, out Point2D theResult, int theIndex,
                              double aTol2, double aTolWorking, ref int aSavedPointNum, ref Point2D aSavedPoint,
                              ref bool theIsFromCache)
    {
        // Try existing endpoint cache first
        int aNbCache = myCache.Length;
        for (int j = 0; j < aNbCache; ++j)
        {
            (Point3D first, Point2D second) aCachePnt = myCache[j];
            if (aCachePnt.first.SquareDistance(thePoint) < aTol2)
            {
                theResult = aProjector.NextValueOfUV(aCachePnt.second, thePoint, aTolWorking, aTol2, aTolWorking);
                aSavedPointNum = theIndex;
                aSavedPoint = aCachePnt.second;
                if (theIndex == 0)
                {
                    theIsFromCache = true;
                }
                return;
            }
        }

        // Fall back to full projection (with B-spline pole cache optimization inside)
        theResult = aProjector.ValueOfUV(thePoint, aTolWorking, aTol2);
    }

    /// <summary>
    /// Tries to represent the pcurve by a Geom2d_Line or a degree-1 B-spline within <paramref name="theTol"/>; null
    /// if the 3D points aren't on such a line of the parametric space.
    /// </summary>
    protected Geom2d_Curve? getLine(NCollection_Array1<Point3D> thePoints, NCollection_Array1<double> theParams,
                                    NCollection_Array1<Point2D> thePoints2d, double theTol, ref bool theIsRecompute,
                                    ref bool theIsFromCache)
    {
        int aNb = thePoints.Length();
        Span<Point3D> aP = stackalloc Point3D[4];
        aP[0] = thePoints[1];
        aP[1] = thePoints[2];
        aP[2] = thePoints[aNb - 1];
        aP[3] = thePoints[aNb];

        Span<Point2D> aP2d = stackalloc Point2D[4];
        int i = 0;

        double aTol2 = theTol * theTol;
        double aTolWorking = theTol;
        bool isPeriodicU = mySurf!.Surface()!.IsUPeriodic();
        bool isPeriodicV = mySurf.Surface()!.IsVPeriodic();

        // Protection against bad tolerance shapes
        if (aTol2 > 1.0)
        {
            aTolWorking = global::Kerf.Engine.Foundation.Precision.Confusion();
            aTol2 = aTolWorking * aTolWorking;
        }
        if (aTol2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            aTol2 = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }
        double anOldTol2 = aTol2;

        int aSavedPointNum = -1;
        Point2D aSavedPoint = new();

        // Create projector with B-spline surface pole cache optimization
        SurfaceProjectorWithCache aProjector = new(mySurf);

        // Project first and last points
        for (; i < 4; i += 3)
        {
            projectPoint(aProjector, aP[i], out aP2d[i], i, aTol2, aTolWorking, ref aSavedPointNum, ref aSavedPoint,
                         ref theIsFromCache);

            double aDist = aProjector.Gap();
            double aCurDist = aDist * aDist;
            if (aTol2 < aCurDist)
            {
                aTol2 = aCurDist;
            }
        }

        if (isPeriodicU || isPeriodicV)
        {
            // Compute second and last but one c2d points
            for (i = 1; i < 3; i++)
            {
                projectPoint(aProjector, aP[i], out aP2d[i], i, aTol2, aTolWorking, ref aSavedPointNum, ref aSavedPoint,
                             ref theIsFromCache);

                double aDist = aProjector.Gap();
                double aCurDist = aDist * aDist;
                if (aTol2 < aCurDist)
                {
                    aTol2 = aCurDist;
                }
            }

            if (isPeriodicU)
            {
                theIsRecompute = fixPeriodicityTroubles(aP2d, 1, mySurf.Surface()!.UPeriod(), aSavedPointNum,
                                                        aSavedPoint.X());
            }

            if (isPeriodicV)
            {
                theIsRecompute = fixPeriodicityTroubles(aP2d, 2, mySurf.Surface()!.VPeriod(), aSavedPointNum,
                                                        aSavedPoint.Y());
            }
        }

        if (theIsRecompute && mySurf.Surface() is Geom_SphericalSurface)
        {
            return null;
        }

        thePoints2d.SetValue(1, aP2d[0]);
        thePoints2d.SetValue(aNb, aP2d[3]);

        // Restore old tolerance
        aTol2 = anOldTol2;

        double dPar = theParams[aNb] - theParams[1];
        if (Math.Abs(dPar) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return null;
        }

        Vector2D aVec0 = new(aP2d[0], aP2d[3]);
        Vector2D aVec = aVec0 / dPar;
        Geom_Surface aSurf = mySurf.Surface()!;
        bool isNormalCheck = aSurf.IsCNu(1) && aSurf.IsCNv(1);

        if (isNormalCheck)
        {
            for (i = 1; i <= aNb; i++)
            {
                Xy aCurPoint = aP2d[0].XY() + aVec.XY() * (theParams[i] - theParams[1]);
                aSurf.D1(aCurPoint.X(), aCurPoint.Y(), out Point3D aCurP, out Vector3D aDu, out Vector3D aDv);
                Vector3D aNormalVec = aDu ^ aDv;
                if (aNormalVec.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    isNormalCheck = false;
                    break;
                }
                Line3D aNormalLine = new(aCurP, new Direction3D(aNormalVec));
                double aDist = aNormalLine.Distance(thePoints[i]);
                if (aDist > aTolWorking)
                {
                    return null;
                }
            }
        }

        if (!isNormalCheck)
        {
            double aFirstPointDist = mySurf.Surface()!.Value(aP2d[0].X(), aP2d[0].Y()).SquareDistance(thePoints[1]);
            aTol2 = Max(aTol2, aTol2 * 2 * aFirstPointDist);
            for (i = 2; i < aNb; i++)
            {
                Xy aCurPoint = aP2d[0].XY() + aVec.XY() * (theParams[i] - theParams[1]);
                aSurf.D0(aCurPoint.X(), aCurPoint.Y(), out Point3D aCurP);
                double aDist1 = aCurP.SquareDistance(thePoints[i]);

                if (Math.Abs(aFirstPointDist - aDist1) > aTol2)
                {
                    return null;
                }
            }
        }

        // Check if pcurve can be represented by Geom2d_Line
        double aLLength = aVec0.Magnitude();
        if (Math.Abs(aLLength - dPar) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Xy aDirL = aVec0.XY() / aLLength;
            Point2D aPL = new(aP2d[0].XY() - theParams[1] * aDirL);
            return new Geom2d_Line(aPL, new Direction2D(aDirL));
        }

        // Create straight bspline
        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles[1] = aP2d[0];
        aPoles[2] = aP2d[3];

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = theParams[1];
        aKnots[2] = theParams[theParams.Length()];

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;

        return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1);
    }

    /// <summary>
    /// The main approximation of the pcurve: projects the points (detecting isoparametric curves and seams), and
    /// returns a straight pcurve in <paramref name="theC2D"/> when <c>getLine</c> finds one (else null, the points in
    /// <paramref name="thePoints2d"/>). The arrays may grow by inserted points.
    /// </summary>
    protected bool approxPCurve(int theNbPnt, Geom_Curve theC3D, double theTolFirst, double theTolLast,
                                ref NCollection_Array1<Point3D> thePoints, ref NCollection_Array1<double> theParams,
                                ref NCollection_Array1<Point2D> thePoints2d, out Geom2d_Curve? theC2D)
    {
        // For performance, first try to handle typical case when pcurve is straight
        bool isRecompute = false;
        bool isFromCacheLine = false;

        theC2D = getLine(thePoints, theParams, thePoints2d, myPreci, ref isRecompute, ref isFromCacheLine);
        if (theC2D is not null)
        {
            // Fill cache
            bool aChangeCycle1 = false;
            if (myCache.Length != 0
                && myCache[0].first.Distance(thePoints[1]) > myCache[0].first.Distance(thePoints[theNbPnt])
                && myCache[0].first.Distance(thePoints[theNbPnt]) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aChangeCycle1 = true;
            }

            myCache = new (Point3D, Point2D)[2];
            if (aChangeCycle1)
            {
                myCache[0] = (thePoints[1], thePoints2d[1]);
                myCache[1] = (thePoints[theNbPnt], thePoints2d[theNbPnt]);
            }
            else
            {
                myCache[0] = (thePoints[theNbPnt], thePoints2d[theNbPnt]);
                myCache[1] = (thePoints[1], thePoints2d[1]);
            }
            return true;
        }

        bool isDone = true;

        // Test if the curve 3d is a boundary of the surface
        bool isoParam, isoTypeU = false, p1OnIso = false, p2OnIso = false, isoClosed;
        // OCCT leaves these unset when isAnIsoparametric doesn't write them; they're read only where it did
        bool isoPar2d3d = false;
        Point2D valueP1 = new(), valueP2 = new();
        Geom_Curve? cIso = null;
        double t1 = 0.0, t2 = 0.0;

        Geom_Surface aSurface = mySurf!.Surface()!;
        bool isAnalytic = true;
        if (aSurface.GetType() == typeof(Geom_BezierSurface) || aSurface.GetType() == typeof(Geom_BSplineSurface))
        {
            isAnalytic = false;
        }

        aSurface.Bounds(out double uf, out double ul, out double vf, out double vl);
        isoClosed = false;

        NCollection_Array1<double> pout = new(1, theNbPnt);
        for (int i = 1; i <= theNbPnt; i++)
        {
            pout.SetValue(i, 0.0);
        }

        isoParam = isAnIsoparametric(theNbPnt, thePoints, theParams, ref isoTypeU, ref p1OnIso, ref valueP1, ref p2OnIso,
                                     ref valueP2, ref isoPar2d3d, ref cIso, ref t1, ref t2, pout);

        // Projection of the points on surfaces
        Point3D p3d;
        Point2D p2d = new();
        double isoValue = 0.0, isoPar1 = 0.0, isoPar2 = 0.0, tPar = 0.0, tdeb, tfin;
        double Cf, Cl, parf, parl;

        if (isoParam)
        {
            if (isoTypeU)
            {
                isoValue = valueP1.X();
                isoPar1 = valueP1.Y();
                isoPar2 = valueP2.Y();
                isoClosed = mySurf.IsVClosed(myPreci);
                parf = vf;
                parl = vl;
            }
            else
            {
                isoValue = valueP1.Y();
                isoPar1 = valueP1.X();
                isoPar2 = valueP2.X();
                isoClosed = mySurf.IsUClosed(myPreci);
                parf = uf;
                parl = ul;
            }
            if (!isoPar2d3d && !isAnalytic)
            {
                Cf = cIso!.FirstParameter();
                Cl = cIso.LastParameter();
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cf))
                {
                    Cf = -1000;
                }
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cl))
                {
                    Cl = +1000;
                }

                tdeb = pout[2];

                if (isoClosed && (isoPar1 == parf || isoPar1 == parl))
                {
                    if (Math.Abs(tdeb - parf) < Math.Abs(tdeb - parl))
                    {
                        isoPar1 = parf;
                    }
                    else
                    {
                        isoPar1 = parl;
                    }
                    if (isoTypeU)
                    {
                        valueP1.SetY(isoPar1);
                    }
                    else
                    {
                        valueP1.SetX(isoPar1);
                    }
                }
                if (isoClosed && (isoPar2 == parf || isoPar2 == parl))
                {
                    tfin = pout[theNbPnt - 1];
                    if (Math.Abs(tfin - parf) < Math.Abs(tfin - parl))
                    {
                        isoPar2 = parf;
                    }
                    else
                    {
                        isoPar2 = parl;
                    }
                    if (isoTypeU)
                    {
                        valueP2.SetY(isoPar2);
                    }
                    else
                    {
                        valueP2.SetX(isoPar2);
                    }
                }

                if (!isoClosed)
                {
                    if ((Math.Abs(tdeb - isoPar1) > Math.Abs(tdeb - isoPar2))
                        && (Math.Abs(pout[theNbPnt - 1] - isoPar2) > Math.Abs(pout[theNbPnt - 1] - isoPar1)))
                    {
                        Point2D valueTmp = valueP1;
                        valueP1 = valueP2;
                        valueP2 = valueTmp;
                        if (isoTypeU)
                        {
                            isoValue = valueP1.X();
                            isoPar1 = valueP1.Y();
                            isoPar2 = valueP2.Y();
                        }
                        else
                        {
                            isoValue = valueP1.Y();
                            isoPar1 = valueP1.X();
                            isoPar2 = valueP2.X();
                        }
                    }
                }
            }
        }

        double Up = ul - uf;
        double Vp = vl - vf;
        double gap = myPreci;
        bool aChangeCycle = false;

        bool isFromCache = false;
        Point2D aSavedPoint = new();

        if (myCache.Length != 0
            && myCache[0].first.Distance(thePoints[1]) > myCache[0].first.Distance(thePoints[theNbPnt]))
        {
            if (myCache[0].first.Distance(thePoints[theNbPnt]) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aChangeCycle = true;
            }
        }

        bool needResolveUJump = false;
        bool needResolveVJump = false;
        Point3D prevP3d = new();
        Point2D prevP2d = new();

        for (int ii = 1; ii <= theNbPnt; ii++)
        {
            int aPntIndex = aChangeCycle ? (theNbPnt - ii + 1) : ii;
            p3d = thePoints[aPntIndex];

            if (isoParam)
            {
                if (isoPar2d3d)
                {
                    if (isoPar2 > isoPar1)
                    {
                        tPar = theParams[aPntIndex];
                    }
                    else
                    {
                        tPar = t1 + t2 - theParams[aPntIndex];
                    }
                }
                else if (!isAnalytic)
                {
                    if (aPntIndex == 1)
                    {
                        tPar = isoPar1;
                    }
                    else if (aPntIndex == theNbPnt)
                    {
                        tPar = isoPar2;
                    }
                    else
                    {
                        tPar = pout[aPntIndex];
                    }
                }

                if (!isoPar2d3d && isAnalytic)
                {
                    if (aPntIndex == 1)
                    {
                        p2d = valueP1;
                    }
                    else if (aPntIndex == theNbPnt)
                    {
                        p2d = valueP2;
                    }
                    else
                    {
                        p2d = mySurf.NextValueOfUV(p2d, p3d, myPreci, global::Kerf.Engine.Foundation.Precision.Confusion() + 1000 * gap);
                        gap = mySurf.Gap();
                    }
                }
                else
                {
                    if (isoTypeU)
                    {
                        p2d.SetX(isoValue);
                        p2d.SetY(tPar);
                    }
                    else
                    {
                        p2d.SetX(tPar);
                        p2d.SetY(isoValue);
                    }
                }
            }
            else
            {
                if ((aPntIndex == 1) && p1OnIso)
                {
                    p2d = valueP1;
                }
                else if ((aPntIndex == theNbPnt) && p2OnIso)
                {
                    p2d = valueP2;
                }
                else
                {
                    if (aPntIndex == 1 || aPntIndex == theNbPnt)
                    {
                        if (!isRecompute)
                        {
                            p2d = thePoints2d[aPntIndex];
                            gap = mySurf.Gap();
                            if (aPntIndex == 1)
                            {
                                isFromCache = isFromCacheLine;
                                aSavedPoint = p2d;
                            }
                            continue;
                        }
                        else
                        {
                            int j = 0;
                            int aNbCache = myCache.Length;
                            for (; j < aNbCache; ++j)
                            {
                                (Point3D first, Point2D second) aCachePnt = myCache[j];
                                if (aCachePnt.first.SquareDistance(p3d) < myPreci * myPreci)
                                {
                                    p2d = mySurf.NextValueOfUV(aCachePnt.second, p3d, myPreci, global::Kerf.Engine.Foundation.Precision.Confusion() + gap);
                                    if (aPntIndex == 1)
                                    {
                                        isFromCache = true;
                                        aSavedPoint = aCachePnt.second;
                                    }
                                    break;
                                }
                            }
                            if (j >= aNbCache)
                            {
                                p2d = mySurf.ValueOfUV(p3d, myPreci);
                            }
                        }
                    }
                    else
                    {
                        p2d = mySurf.NextValueOfUV(p2d, p3d, myPreci, global::Kerf.Engine.Foundation.Precision.Confusion() + 1000 * gap);
                    }
                    gap = mySurf.Gap();
                }
            }
            thePoints2d.SetValue(aPntIndex, p2d);

            if (theNbPnt > 23 && ii > 2 && ii < theNbPnt)
            {
                if (Math.Abs(p2d.X() - prevP2d.X()) > 0.95 * Up && prevP3d.Distance(p3d) < myPreci
                    && !mySurf.IsUClosed(myPreci) && mySurf.NbSingularities(myPreci) > 0
                    && aSurface is Geom_BSplineSurface)
                {
                    needResolveUJump = true;
                }
                if (Math.Abs(p2d.Y() - prevP2d.Y()) > 0.95 * Vp && prevP3d.Distance(p3d) < myPreci
                    && !mySurf.IsVClosed(myPreci) && mySurf.NbSingularities(myPreci) > 0
                    && aSurface is Geom_BSplineSurface)
                {
                    needResolveVJump = true;
                }
            }
            prevP3d = p3d;
            prevP2d = p2d;
            if (ii > 1)
            {
                if (aChangeCycle)
                {
                    p2d.SetXY(2.0 * p2d.XY() - thePoints2d[aPntIndex + 1].XY());
                }
                else
                {
                    p2d.SetXY(2.0 * p2d.XY() - thePoints2d[aPntIndex - 1].XY());
                }
            }
        }

        if (!isoPar2d3d)
        {
            projectDegeneratedPoints(mySurf, theNbPnt, thePoints, thePoints2d, myPreci, true);
            projectDegeneratedPoints(mySurf, theNbPnt, thePoints, thePoints2d, myPreci, false);
        }

        // Check extremities for singularities
        Point3D aPointFirst = thePoints.First();
        Point3D aPointLast = thePoints.Last();
        double aTolFirst = (theTolFirst < 0) ? global::Kerf.Engine.Foundation.Precision.Confusion() : theTolFirst;
        double aTolLast = (theTolLast < 0) ? global::Kerf.Engine.Foundation.Precision.Confusion() : theTolLast;

        for (int i = 1;; i++)
        {
            double aPreci = 0.0, aFirstPar = 0.0, aLastPar = 0.0;
            Point3D aP3d = new();
            Point2D aFirstP2d = new(), aLastP2d = new();
            bool IsUiso = false;
            if (!mySurf.Singularity(i, ref aPreci, ref aP3d, ref aFirstP2d, ref aLastP2d, ref aFirstPar, ref aLastPar,
                                    ref IsUiso))
            {
                break;
            }
            if (aPreci <= global::Kerf.Engine.Foundation.Precision.Confusion() && aPointFirst.Distance(aP3d) <= aTolFirst)
            {
                correctExtremity(theC3D, theParams, thePoints2d, true, aFirstP2d, IsUiso);
            }
            if (aPreci <= global::Kerf.Engine.Foundation.Precision.Confusion() && aPointLast.Distance(aP3d) <= aTolLast)
            {
                correctExtremity(theC3D, theParams, thePoints2d, false, aFirstP2d, IsUiso);
            }
        }

        // Handle U-closed surfaces
        double dist2d;
        double TolOnUPeriod = global::Kerf.Engine.Foundation.Precision.Confusion() * Up;
        double TolOnVPeriod = global::Kerf.Engine.Foundation.Precision.Confusion() * Vp;

        if (mySurf.IsUClosed(myPreci) || needResolveUJump)
        {
            double prevX, firstX = thePoints2d[1].X();
            if (!isFromCache)
            {
                while (firstX < uf)
                {
                    firstX += Up;
                    thePoints2d[1].SetX(firstX);
                }
                while (firstX > ul)
                {
                    firstX -= Up;
                    thePoints2d[1].SetX(firstX);
                }
            }

            if (aSurface.IsUPeriodic() && isFromCache)
            {
                double aMinParam = uf, aMaxParam = ul;
                while (aMinParam > aSavedPoint.X())
                {
                    aMinParam -= Up;
                    aMaxParam -= Up;
                }
                while (aMaxParam < aSavedPoint.X())
                {
                    aMinParam += Up;
                    aMaxParam += Up;
                }
                double aShift = ShapeAnalysis.AdjustToPeriod(firstX, aMinParam, aMaxParam);
                firstX += aShift;
                thePoints2d[1].SetX(firstX);
            }
            prevX = firstX;

            double minX = firstX, maxX = firstX;
            bool ToAdjust = false;

            for (int aPntIter = 2; aPntIter <= thePoints2d.Length(); ++aPntIter)
            {
                double CurX = thePoints2d[aPntIter].X();
                dist2d = Math.Abs(CurX - prevX);
                if (dist2d > (Up / 2))
                {
                    insertAdditionalPointOrAdjust(ref ToAdjust, 1, Up, TolOnUPeriod, ref CurX, prevX, theC3D, ref aPntIter,
                                                  ref thePoints, ref theParams, ref thePoints2d);
                }
                prevX = CurX;
                if (minX > CurX)
                {
                    minX = CurX;
                }
                else if (maxX < CurX)
                {
                    maxX = CurX;
                }
            }

            if (!isFromCache)
            {
                double midX = 0.5 * (minX + maxX);
                double shiftX = 0.0;
                if (midX > ul)
                {
                    shiftX = -Up;
                }
                else if (midX < uf)
                {
                    shiftX = Up;
                }
                if (shiftX != 0.0)
                {
                    for (int aPntIter = 1; aPntIter <= thePoints2d.Length(); ++aPntIter)
                    {
                        thePoints2d[aPntIter].SetX(thePoints2d[aPntIter].X() + shiftX);
                    }
                }
            }
        }

        // Handle V-closed surfaces
        if (mySurf.IsVClosed(myPreci) || needResolveVJump || aSurface is Geom_SphericalSurface)
        {
            double prevY, firstY = thePoints2d[1].Y();
            if (!isFromCache)
            {
                while (firstY < vf)
                {
                    firstY += Vp;
                    thePoints2d[1].SetY(firstY);
                }
                while (firstY > vl)
                {
                    firstY -= Vp;
                    thePoints2d[1].SetY(firstY);
                }
            }

            if (aSurface.IsVPeriodic() && isFromCache)
            {
                double aMinParam = vf, aMaxParam = vl;
                while (aMinParam > aSavedPoint.Y())
                {
                    aMinParam -= Vp;
                    aMaxParam -= Vp;
                }
                while (aMaxParam < aSavedPoint.Y())
                {
                    aMinParam += Vp;
                    aMaxParam += Vp;
                }
                double aShift = ShapeAnalysis.AdjustToPeriod(firstY, aMinParam, aMaxParam);
                firstY += aShift;
                thePoints2d[1].SetY(firstY);
            }
            prevY = firstY;

            double minY = firstY, maxY = firstY;
            bool ToAdjust = false;

            for (int aPntIter = 2; aPntIter <= thePoints2d.Length(); ++aPntIter)
            {
                double CurY = thePoints2d[aPntIter].Y();
                dist2d = Math.Abs(CurY - prevY);
                if (dist2d > (Vp / 2))
                {
                    insertAdditionalPointOrAdjust(ref ToAdjust, 2, Vp, TolOnVPeriod, ref CurY, prevY, theC3D, ref aPntIter,
                                                  ref thePoints, ref theParams, ref thePoints2d);
                }
                prevY = CurY;
                if (minY > CurY)
                {
                    minY = CurY;
                }
                else if (maxY < CurY)
                {
                    maxY = CurY;
                }
            }

            if (!isFromCache)
            {
                double midY = 0.5 * (minY + maxY);
                double shiftY = 0.0;
                if (midY > vl)
                {
                    shiftY = -Vp;
                }
                else if (midY < vf)
                {
                    shiftY = Vp;
                }
                if (shiftY != 0.0)
                {
                    for (int aPntIter = 1; aPntIter <= thePoints2d.Length(); ++aPntIter)
                    {
                        thePoints2d[aPntIter].SetY(thePoints2d[aPntIter].Y() + shiftY);
                    }
                }
            }
        }

        // Handle V-closed seam adjustment
        if (mySurf.IsVClosed(myPreci) || aSurface is Geom_SphericalSurface)
        {
            for (int aPntIter = 2; aPntIter <= thePoints2d.Length(); ++aPntIter)
            {
                dist2d = Math.Abs(thePoints2d[aPntIter].Y() - thePoints2d[aPntIter - 1].Y());
                if (dist2d > (Vp / 2))
                {
                    bool prevOnFirst = false;
                    bool prevOnLast = false;
                    bool currOnFirst = false;
                    bool currOnLast = false;

                    double distPrevVF = Math.Abs(thePoints2d[aPntIter - 1].Y() - vf);
                    double distPrevVL = Math.Abs(thePoints2d[aPntIter - 1].Y() - vl);
                    double distCurrVF = Math.Abs(thePoints2d[aPntIter].Y() - vf);
                    double distCurrVL = Math.Abs(thePoints2d[aPntIter].Y() - vl);

                    double theMin = distPrevVF;
                    prevOnFirst = true;
                    if (distPrevVL < theMin)
                    {
                        theMin = distPrevVL;
                        prevOnFirst = false;
                        prevOnLast = true;
                    }
                    if (distCurrVF < theMin)
                    {
                        theMin = distCurrVF;
                        prevOnFirst = false;
                        prevOnLast = false;
                        currOnFirst = true;
                    }
                    if (distCurrVL < theMin)
                    {
                        prevOnFirst = false;
                        prevOnLast = false;
                        currOnFirst = false;
                        currOnLast = true;
                    }

                    if (prevOnFirst)
                    {
                        thePoints2d[aPntIter - 1].SetY(vf);
                    }
                    else if (prevOnLast)
                    {
                        thePoints2d[aPntIter - 1].SetY(vl);
                    }
                    else if (currOnFirst)
                    {
                        thePoints2d[aPntIter].SetY(vf);
                    }
                    else if (currOnLast)
                    {
                        thePoints2d[aPntIter].SetY(vl);
                    }
                }
            }
        }

        // Handle AdjustOverDegen
        if (myAdjustOverDegen != -1)
        {
            if (mySurf.IsUClosed(myPreci))
            {
                mySurf.IsDegenerated(new Point3D(0, 0, 0), myPreci);
                if (mySurf.NbSingularities(myPreci) > 0)
                {
                    double PrevX = 0.0;
                    int OnBound = 0, PrevOnBound = 0;
                    int ind;
                    bool start = true;
                    for (ind = 1; ind <= thePoints2d.Length(); ind++)
                    {
                        double CurX = thePoints2d[ind].X();
                        if (mySurf.IsDegenerated(thePoints[ind], global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            continue;
                        }
                        OnBound = (Math.Abs(Math.Abs(CurX - 0.5 * (ul + uf)) - Up / 2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) ? 1 : 0;
                        if (!start && Math.Abs(Math.Abs(CurX - PrevX) - Up / 2) <= 0.01 * Up)
                        {
                            break;
                        }
                        start = false;
                        PrevX = CurX;
                        PrevOnBound = OnBound;
                    }
                    if (ind <= thePoints2d.Length())
                    {
                        PrevX = (myAdjustOverDegen != 0 ? uf : ul);
                        double dU = Up / 2 + global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (PrevOnBound != 0)
                        {
                            thePoints2d[ind - 1].SetX(PrevX);
                            for (int j = ind - 2; j > 0; j--)
                            {
                                double CurX = thePoints2d[j].X();
                                while (CurX < PrevX - dU)
                                {
                                    CurX += Up;
                                    thePoints2d[j].SetX(CurX);
                                }
                                while (CurX > PrevX + dU)
                                {
                                    CurX -= Up;
                                    thePoints2d[j].SetX(CurX);
                                }
                            }
                        }
                        else if (OnBound != 0)
                        {
                            thePoints2d[ind].SetX(PrevX);
                            for (int j = ind + 1; j <= thePoints2d.Length(); j++)
                            {
                                double CurX = thePoints2d[j].X();
                                while (CurX < PrevX - dU)
                                {
                                    CurX += Up;
                                    thePoints2d[j].SetX(CurX);
                                }
                                while (CurX > PrevX + dU)
                                {
                                    CurX -= Up;
                                    thePoints2d[j].SetX(CurX);
                                }
                            }
                        }
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                    }
                }
            }
            else if (mySurf.IsVClosed(myPreci))
            {
                mySurf.IsDegenerated(new Point3D(0, 0, 0), myPreci);
                if (mySurf.NbSingularities(myPreci) > 0)
                {
                    double PrevY = 0.0;
                    int OnBound = 0, PrevOnBound = 0;
                    int ind;
                    bool start = true;
                    for (ind = 1; ind <= thePoints2d.Length(); ind++)
                    {
                        double CurY = thePoints2d[ind].Y();
                        if (mySurf.IsDegenerated(thePoints[ind], global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            continue;
                        }
                        OnBound = (Math.Abs(Math.Abs(CurY - 0.5 * (vl + vf)) - Vp / 2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) ? 1 : 0;
                        if (!start && Math.Abs(Math.Abs(CurY - PrevY) - Vp / 2) <= 0.01 * Vp)
                        {
                            break;
                        }
                        start = false;
                        PrevY = CurY;
                        PrevOnBound = OnBound;
                    }
                    if (ind <= thePoints2d.Length())
                    {
                        PrevY = (myAdjustOverDegen != 0 ? vf : vl);
                        double dV = Vp / 2 + global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (PrevOnBound != 0)
                        {
                            thePoints2d[ind - 1].SetY(PrevY);
                            for (int j = ind - 2; j > 0; j--)
                            {
                                double CurY = thePoints2d[j].Y();
                                while (CurY < PrevY - dV)
                                {
                                    CurY += Vp;
                                    thePoints2d[j].SetY(CurY);
                                }
                                while (CurY > PrevY + dV)
                                {
                                    CurY -= Vp;
                                    thePoints2d[j].SetY(CurY);
                                }
                            }
                        }
                        else if (OnBound != 0)
                        {
                            thePoints2d[ind].SetY(PrevY);
                            for (int j = ind + 1; j <= thePoints2d.Length(); j++)
                            {
                                double CurY = thePoints2d[j].Y();
                                while (CurY < PrevY - dV)
                                {
                                    CurY += Vp;
                                    thePoints2d[j].SetY(CurY);
                                }
                                while (CurY > PrevY + dV)
                                {
                                    CurY -= Vp;
                                    thePoints2d[j].SetY(CurY);
                                }
                            }
                        }
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                    }
                }
            }
        }

        // Fill cache
        myCache = new (Point3D, Point2D)[2];
        if (aChangeCycle)
        {
            myCache[0] = (thePoints[1], thePoints2d[1]);
            myCache[1] = (thePoints.Last(), thePoints2d.Last());
        }
        else
        {
            myCache[0] = (thePoints.Last(), thePoints2d.Last());
            myCache[1] = (thePoints[1], thePoints2d[1]);
        }

        return isDone;
    }

    /// <summary>Corrects an end point of the pcurve near a singularity of the surface.</summary>
    protected void correctExtremity(Geom_Curve theC3D, NCollection_Array1<double> theParams,
                                    NCollection_Array1<Point2D> thePoints2d, bool theIsFirstPoint,
                                    in Point2D thePointOnIsoLine, bool theIsUIso)
    {
        int NbPnt = thePoints2d.Length();
        int IndCoord = (theIsUIso) ? 2 : 1;
        double SingularityCoord = thePointOnIsoLine.Coord(3 - IndCoord);
        Point2D EndPoint = (theIsFirstPoint) ? thePoints2d[1] : thePoints2d[NbPnt];
        double FinishCoord = EndPoint.Coord(3 - IndCoord);

        Direction2D aDir = (theIsUIso) ? Constants.DY2d() : Constants.DX2d();
        Line2D anIsoLine = new(EndPoint, aDir);
        IntRes2d_Domain Dom1 = new(), Dom2 = new();

        bool IsPeriodic = (theIsUIso) ? mySurf!.Surface()!.IsVPeriodic() : mySurf!.Surface()!.IsUPeriodic();

        Point2D FirstPointOfLine, SecondPointOfLine;
        double FinishParam, FirstParam, SecondParam;

        if (theIsFirstPoint)
        {
            FirstPointOfLine = thePoints2d[3];
            SecondPointOfLine = thePoints2d[2];
            FinishParam = theParams[1];
            FirstParam = theParams[3];
            SecondParam = theParams[2];
        }
        else
        {
            FirstPointOfLine = thePoints2d[NbPnt - 2];
            SecondPointOfLine = thePoints2d[NbPnt - 1];
            FinishParam = theParams[NbPnt];
            FirstParam = theParams[NbPnt - 2];
            SecondParam = theParams[NbPnt - 1];
        }

        if (SingularityCoord > FinishCoord && SecondPointOfLine.Coord(3 - IndCoord) > FinishCoord)
        {
            return;
        }
        if (SingularityCoord < FinishCoord && SecondPointOfLine.Coord(3 - IndCoord) < FinishCoord)
        {
            return;
        }

        {
            double aPrevDist = Math.Abs(SecondPointOfLine.Coord(IndCoord) - FirstPointOfLine.Coord(IndCoord));
            double aCurDist = Math.Abs(EndPoint.Coord(IndCoord) - SecondPointOfLine.Coord(IndCoord));
            if (aCurDist <= 2 * aPrevDist)
            {
                return;
            }
        }

        Point2D FinishPoint = (theIsUIso) ? new Point2D(FinishCoord, SecondPointOfLine.Y())
                                           : new Point2D(SecondPointOfLine.X(), FinishCoord);

        for (;;)
        {
            if (Math.Abs(SecondPointOfLine.Coord(3 - IndCoord) - FinishCoord) <= 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }

            Vector2D aVec = new(FirstPointOfLine, SecondPointOfLine);
            double aSqMagnitude = aVec.SquareMagnitude();
            if (aSqMagnitude <= 1.0e-32)
            {
                break;
            }
            aDir.SetCoord(aVec.X(), aVec.Y());

            Line2D aLine = new(FirstPointOfLine, aDir);
            IntCurve_IntConicConic Intersector = new(anIsoLine, Dom1, aLine, Dom2, 1.0e-10, 1.0e-10);
            if (Intersector.IsDone() && !Intersector.IsEmpty())
            {
                IntRes2d_IntersectionPoint IntPoint = Intersector.Point(1);
                FinishPoint = IntPoint.Value();
            }
            else
            {
                FinishPoint = (theIsUIso) ? new Point2D(FinishCoord, SecondPointOfLine.Y())
                                          : new Point2D(SecondPointOfLine.X(), FinishCoord);
            }

            Point2D PrevPoint = FirstPointOfLine;
            FirstPointOfLine = SecondPointOfLine;
            FirstParam = SecondParam;
            SecondParam = (FirstParam + FinishParam) / 2;
            if (Math.Abs(SecondParam - FirstParam) <= 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            theC3D.D0(SecondParam, out Point3D aP3d);
            SecondPointOfLine = mySurf.NextValueOfUV(FirstPointOfLine, aP3d, myPreci, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (IsPeriodic)
            {
                adjustSecondToFirstPoint(FirstPointOfLine, ref SecondPointOfLine, mySurf.Surface()!);
            }

            double aPrevDist = Math.Abs(FirstPointOfLine.Coord(IndCoord) - PrevPoint.Coord(IndCoord));
            double aCurDist = Math.Abs(SecondPointOfLine.Coord(IndCoord) - FirstPointOfLine.Coord(IndCoord));
            if (aCurDist > 2 * aPrevDist)
            {
                break;
            }
        }

        if (theIsFirstPoint)
        {
            thePoints2d[1] = FinishPoint;
        }
        else
        {
            thePoints2d[NbPnt] = FinishPoint;
        }
    }

    /// <summary>
    /// At a jump of a period between two points: inserts a point between them (the arrays are replaced by longer
    /// ones) or adjusts the current coordinate by the period.
    /// </summary>
    protected void insertAdditionalPointOrAdjust(ref bool theToAdjust, int theIndCoord, double thePeriod,
                                                 double theTolOnPeriod, ref double theCurCoord, double thePrevCoord,
                                                 Geom_Curve theC3D, ref int theIndex,
                                                 ref NCollection_Array1<Point3D> thePoints,
                                                 ref NCollection_Array1<double> theParams,
                                                 ref NCollection_Array1<Point2D> thePoints2d)
    {
        double CorrectedCurCoord = global::Kerf.Engine.Geometry.ElCLib.InPeriod(theCurCoord, thePrevCoord - thePeriod / 2,
                                                   thePrevCoord + thePeriod / 2);

        if (!theToAdjust)
        {
            double CurPar = theParams[theIndex];
            double PrevPar = theParams[theIndex - 1];
            double MidPar = (PrevPar + CurPar) / 2;
            theC3D.D0(MidPar, out Point3D MidP3d);
            Point2D MidP2d = mySurf!.ValueOfUV(MidP3d, myPreci);
            double MidCoord = MidP2d.Coord(theIndCoord);
            MidCoord = global::Kerf.Engine.Geometry.ElCLib.InPeriod(MidCoord, thePrevCoord - thePeriod / 2, thePrevCoord + thePeriod / 2);
            double FirstCoord = thePrevCoord, LastCoord = CorrectedCurCoord;
            if (LastCoord < FirstCoord)
            {
                double tmp = FirstCoord;
                FirstCoord = LastCoord;
                LastCoord = tmp;
            }
            if (LastCoord - FirstCoord <= theTolOnPeriod)
            {
                theToAdjust = true;
            }
            else if (FirstCoord <= MidCoord && MidCoord <= LastCoord)
            {
                theToAdjust = true;
            }
            else
            {
                bool Success = true;
                double FirstT = PrevPar;
                double LastT = CurPar;
                MidCoord = MidP2d.Coord(theIndCoord);
                while (Math.Abs(MidCoord - thePrevCoord) >= thePeriod / 2 - theTolOnPeriod
                       || Math.Abs(theCurCoord - MidCoord) >= thePeriod / 2 - theTolOnPeriod)
                {
                    if (MidPar - FirstT <= global::Kerf.Engine.Foundation.Precision.PConfusion() || LastT - MidPar <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Success = false;
                        break;
                    }
                    if (Math.Abs(MidCoord - thePrevCoord) >= thePeriod / 2 - theTolOnPeriod)
                    {
                        LastT = (FirstT + LastT) / 2;
                    }
                    else
                    {
                        FirstT = (FirstT + LastT) / 2;
                    }
                    MidPar = (FirstT + LastT) / 2;
                    theC3D.D0(MidPar, out MidP3d);
                    MidP2d = mySurf.ValueOfUV(MidP3d, myPreci);
                    MidCoord = MidP2d.Coord(theIndCoord);
                }
                if (Success)
                {
                    // Insert additional point - need to resize arrays
                    int aNewLength = thePoints.Length() + 1;
                    NCollection_Array1<Point3D> aNewPoints = new(1, aNewLength);
                    NCollection_Array1<double> aNewParams = new(1, aNewLength);
                    NCollection_Array1<Point2D> aNewPoints2d = new(1, aNewLength);

                    for (int i = 1; i < theIndex; i++)
                    {
                        aNewPoints.SetValue(i, thePoints[i]);
                        aNewParams.SetValue(i, theParams[i]);
                        aNewPoints2d.SetValue(i, thePoints2d[i]);
                    }
                    aNewPoints.SetValue(theIndex, MidP3d);
                    aNewParams.SetValue(theIndex, MidPar);
                    aNewPoints2d.SetValue(theIndex, MidP2d);
                    for (int i = theIndex; i <= thePoints.Length(); i++)
                    {
                        aNewPoints.SetValue(i + 1, thePoints[i]);
                        aNewParams.SetValue(i + 1, theParams[i]);
                        aNewPoints2d.SetValue(i + 1, thePoints2d[i]);
                    }

                    thePoints = aNewPoints;
                    theParams = aNewParams;
                    thePoints2d = aNewPoints2d;
                    theIndex++;
                }
                else
                {
                    theToAdjust = true;
                }
            }
        }
        if (theToAdjust)
        {
            theCurCoord = CorrectedCurCoord;
            thePoints2d[theIndex].SetCoord(theIndCoord, theCurCoord);
        }
    }

    /// <summary>Interpolates the pcurve through the points (Geom2dAPI_Interpolate), or null on failure.</summary>
    protected Geom2d_Curve? interpolatePCurve(int theNbPnt, NCollection_Array1<Point2D> thePoints2d,
                                              NCollection_Array1<double> theParams)
    {
        Geom2d_Curve? aC2D = null;
        double theTolerance2d = myPreci / (100 * theNbPnt);

        try
        {
            // Convert to HArrays for Geom2dAPI_Interpolate
            NCollection_HArray1<Point2D> aPnts2d = new(1, theNbPnt);
            NCollection_HArray1<double> aParams = new(1, theNbPnt);

            for (int i = 1; i <= theNbPnt; i++)
            {
                aPnts2d.SetValue(i, thePoints2d[i]);
                aParams.SetValue(i, theParams[i]);
            }

            // Check for coincident points
            NCollection_Array1<Point2D> aTmpPnts2d = new(thePoints2d);
            NCollection_Array1<double> aTmpParams = new(theParams);
            double aPreci = theTolerance2d;
            checkPoints2d(ref aTmpPnts2d, ref aTmpParams, ref aPreci);

            if (aTmpPnts2d.Length() < 2)
            {
                return aC2D;
            }

            // Rebuild HArrays if points were removed
            if (aTmpPnts2d.Length() != theNbPnt)
            {
                aPnts2d = new NCollection_HArray1<Point2D>(1, aTmpPnts2d.Length());
                aParams = new NCollection_HArray1<double>(1, aTmpParams.Length());
                for (int i = 1; i <= aTmpPnts2d.Length(); i++)
                {
                    aPnts2d.SetValue(i, aTmpPnts2d[i]);
                    aParams.SetValue(i, aTmpParams[i]);
                }
            }

            Geom2dAPI_Interpolate myInterPol2d = new(aPnts2d, aParams, false, aPreci);
            myInterPol2d.Perform();
            if (myInterPol2d.IsDone())
            {
                aC2D = myInterPol2d.InternalCurve();
            }
        }
        catch (FailureException)
        {
            aC2D = null;
        }
        return aC2D;
    }

    /// <summary>Approximates the pcurve through the points (GeomAPI_PointsToBSpline), or null on failure.</summary>
    protected Geom2d_Curve? approximatePCurve(NCollection_Array1<Point2D> thePoints2d, NCollection_Array1<double> theParams)
    {
        double theTolerance2d = myPreci;
        Geom2d_Curve? aC2D;

        try
        {
            NCollection_Array1<Point2D> aTmpPnts2d = new(thePoints2d);
            NCollection_Array1<double> aTmpParams = new(theParams);
            double aPreci = theTolerance2d;
            checkPoints2d(ref aTmpPnts2d, ref aTmpParams, ref aPreci);

            int numberPnt = aTmpPnts2d.Length();
            if (numberPnt < 2)
            {
                return null;
            }

            NCollection_Array1<Point3D> points3d = new(1, numberPnt);
            NCollection_Array1<double> @params = new(1, numberPnt);

            for (int i = 1; i <= numberPnt; i++)
            {
                Point2D pnt2d = aTmpPnts2d[i];
                points3d[i].SetCoord(pnt2d.X(), pnt2d.Y(), 0);
                @params[i] = aTmpParams[i];
            }

            GeomAPI_PointsToBSpline appr = new(points3d, @params, 1, 10, C1, theTolerance2d);
            Geom_BSplineCurve crv3d = appr.Curve();

            int NbPoles = crv3d.NbPoles();
            NCollection_Array1<Point3D> poles3d = crv3d.InternalPoles();
            NCollection_Array1<Point2D> poles2d = new(1, NbPoles);

            for (int i = 1; i <= NbPoles; i++)
            {
                poles2d[i].SetCoord(poles3d[i].X(), poles3d[i].Y());
            }

            NCollection_Array1<double> weights = crv3d.InternalWeightsArray();
            NCollection_Array1<double> knots = crv3d.InternalKnots();
            NCollection_Array1<int> multiplicities = crv3d.InternalMultiplicities();

            aC2D = new Geom2d_BSplineCurve(poles2d, weights, knots, multiplicities, crv3d.Degree(), crv3d.IsPeriodic());
            return aC2D;
        }
        catch (FailureException)
        {
            aC2D = null;
        }
        return aC2D;
    }

    /// <summary>Removes coincident 3D points (with their parameters); sets the precision from the smallest gap.</summary>
    protected void checkPoints(ref NCollection_Array1<Point3D> thePoints, ref NCollection_Array1<double> theParams,
                               ref double thePreci)
    {
        int firstElem = thePoints.Lower();
        int lastElem = thePoints.Upper();

        int nbPntDropped = 0;
        int lastValid = firstElem;

        NCollection_Array1<int> tmpParam = new(firstElem, lastElem);
        for (int i = firstElem; i <= lastElem; i++)
        {
            tmpParam.SetValue(i, 1);
        }

        double DistMin2 = RealLast();
        Point3D Prev = thePoints.Value(lastValid);

        for (int i = firstElem + 1; i <= lastElem; i++)
        {
            Point3D Curr = thePoints.Value(i);
            double CurDist2 = Prev.SquareDistance(Curr);
            if (CurDist2 < Constants.Resolution())
            {
                nbPntDropped++;
                if (i == lastElem)
                {
                    tmpParam.SetValue(lastValid, 0);
                }
                else
                {
                    tmpParam.SetValue(i, 0);
                }
            }
            else
            {
                if (CurDist2 < DistMin2)
                {
                    DistMin2 = CurDist2;
                }
                lastValid = i;
                Prev = Curr;
            }
        }

        if (DistMin2 < RealLast())
        {
            thePreci = 0.9 * Math.Sqrt(DistMin2);
        }

        if (nbPntDropped == 0)
        {
            return;
        }

        int newLast = lastElem - nbPntDropped;
        if ((newLast - firstElem + 1) < 2)
        {
            return;
        }

        NCollection_Array1<Point3D> newPnts = new(firstElem, newLast);
        NCollection_Array1<double> newParams = new(firstElem, newLast);
        int newCurr = firstElem;

        for (int i = firstElem; i <= lastElem; i++)
        {
            if (tmpParam.Value(i) == 1)
            {
                newPnts.SetValue(newCurr, thePoints.Value(i));
                newParams.SetValue(newCurr, theParams.Value(i));
                newCurr++;
            }
        }

        thePoints = newPnts;
        theParams = newParams;
    }

    /// <summary>
    /// Removes coincident 2D points (with their parameters); sets the precision from the smallest gap. If fewer than two
    /// points would remain, keeps the ends, the last moved by the precision.
    /// </summary>
    protected void checkPoints2d(ref NCollection_Array1<Point2D> thePoints2d, ref NCollection_Array1<double> theParams,
                                 ref double thePreci)
    {
        int firstElem = thePoints2d.Lower();
        int lastElem = thePoints2d.Upper();

        int nbPntDropped = 0;
        int lastValid = firstElem;

        NCollection_Array1<int> tmpParam = new(firstElem, lastElem);
        for (int i = firstElem; i <= lastElem; i++)
        {
            tmpParam.SetValue(i, 1);
        }

        double DistMin2 = RealLast();
        Point2D Prev = thePoints2d.Value(lastValid);

        for (int i = firstElem + 1; i <= lastElem; i++)
        {
            Point2D Curr = thePoints2d.Value(i);
            double CurDist2 = Prev.SquareDistance(Curr);
            if (CurDist2 < Constants.Resolution())
            {
                nbPntDropped++;
                if (i == lastElem)
                {
                    tmpParam.SetValue(lastValid, 0);
                }
                else
                {
                    tmpParam.SetValue(i, 0);
                }
            }
            else
            {
                if (CurDist2 < DistMin2)
                {
                    DistMin2 = CurDist2;
                }
                lastValid = i;
                Prev = Curr;
            }
        }

        if (DistMin2 < RealLast())
        {
            thePreci = 0.9 * Math.Sqrt(DistMin2);
        }

        if (nbPntDropped == 0)
        {
            return;
        }

        int newLast = lastElem - nbPntDropped;
        if ((newLast - firstElem + 1) < 2)
        {
            // Create minimal length pcurve
            tmpParam.SetValue(firstElem, 1);
            tmpParam.SetValue(lastElem, 1);
            Xy lastPnt = thePoints2d.Value(lastElem).XY();
            lastPnt.Add(new Xy(thePreci, thePreci));
            thePoints2d[lastElem].SetXY(lastPnt);
            newLast = firstElem + 1;
        }

        NCollection_Array1<Point2D> newPnts = new(firstElem, newLast);
        NCollection_Array1<double> newParams = new(firstElem, newLast);
        int newCurr = firstElem;

        for (int i = firstElem; i <= lastElem; i++)
        {
            if (tmpParam.Value(i) == 1)
            {
                newPnts.SetValue(newCurr, thePoints2d.Value(i));
                newParams.SetValue(newCurr, theParams.Value(i));
                newCurr++;
            }
        }

        thePoints2d = newPnts;
        theParams = newParams;
    }

    /// <summary>
    /// True if the curve's points lie on an isoparametric line of the surface's bounds (U = const if
    /// <paramref name="theIsTypeU"/>), with the end points' parameters, the iso-curve and the points' parameters on it.
    /// The outputs are <c>ref</c>: OCCT leaves some of them unset on some paths.
    /// </summary>
    protected bool isAnIsoparametric(int theNbPnt, NCollection_Array1<Point3D> thePoints, NCollection_Array1<double> theParams,
                                     ref bool theIsTypeU, ref bool theP1OnIso, ref Point2D theValueP1,
                                     ref bool theP2OnIso, ref Point2D theValueP2, ref bool theIsoPar2d3d,
                                     ref Geom_Curve? theCIso, ref double theT1, ref double theT2,
                                     NCollection_Array1<double> theParamsOut)
    {
        try
        {
            double prec = global::Kerf.Engine.Foundation.Precision.Confusion();

            bool isoParam = false;
            theIsoPar2d3d = false;

            mySurf!.Bounds(out double U1, out double U2, out double V1, out double V2);

            if (mySurf.Surface() is Geom_RectangularTrimmedSurface sTrim)
            {
                sTrim.Bounds(out U1, out U2, out V1, out V2);
            }

            Point3D pt;
            Span<int> mpt = stackalloc int[2];
            mpt[0] = mpt[1] = 0;
            double t;
            Span<double> tpar = stackalloc double[2] { 0.0, 0.0 };
            double isoValue = 0.0;
            double mindist2;
            Span<double> mind2 = stackalloc double[2];
            mindist2 = mind2[0] = mind2[1] = 4 * prec * prec;

            theP1OnIso = false;
            theP2OnIso = false;
            Box aBox = new();
            Span<double> currd2 = stackalloc double[2];
            Span<double> tp = stackalloc double[2];
            Span<int> mp = stackalloc int[2];

            for (int j = 1; j <= 4; j++)
            {
                double isoVal = 0.0;
                bool isoU = false;
                Geom_Curve? cI = null;
                double tt1, tt2;

                if (j == 1)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U1))
                    {
                        continue;
                    }
                    cI = mySurf.UIso(U1);
                    isoU = true;
                    isoVal = U1;
                    aBox = mySurf.GetBoxUF();
                }
                else if (j == 2)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U2))
                    {
                        continue;
                    }
                    cI = mySurf.UIso(U2);
                    isoU = true;
                    isoVal = U2;
                    aBox = mySurf.GetBoxUL();
                }
                else if (j == 3)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V1))
                    {
                        continue;
                    }
                    cI = mySurf.VIso(V1);
                    isoU = false;
                    isoVal = V1;
                    aBox = mySurf.GetBoxVF();
                }
                else if (j == 4)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V2))
                    {
                        continue;
                    }
                    cI = mySurf.VIso(V2);
                    isoU = false;
                    isoVal = V2;
                    aBox = mySurf.GetBoxVL();
                }
                if (cI is null)
                {
                    continue;
                }

                if (isoU)
                {
                    tt1 = V1;
                    tt2 = V2;
                }
                else
                {
                    tt1 = U1;
                    tt2 = U2;
                }

                cI.D0(tt1, out Point3D ext1);
                cI.D0(tt2, out Point3D ext2);

                cI.D0((tt1 + tt2) / 2, out Point3D extmi);
                if (ext1.IsEqual(ext2, prec) && ext1.IsEqual(extmi, prec))
                {
                    continue;
                }

                bool PtEQext1 = false;
                bool PtEQext2 = false;

                tp[0] = tp[1] = 0;

                for (int i = 0; i < 2; i++)
                {
                    mp[i] = 0;
                    int k = (i == 0 ? 1 : theNbPnt);

                    currd2[i] = thePoints[k].SquareDistance(ext1);
                    if (currd2[i] <= prec * prec && !PtEQext1)
                    {
                        mp[i] = 1;
                        tp[i] = tt1;
                        PtEQext1 = true;
                        continue;
                    }

                    currd2[i] = thePoints[k].SquareDistance(ext2);
                    if (currd2[i] <= prec * prec && !PtEQext2)
                    {
                        mp[i] = 2;
                        tp[i] = tt2;
                        PtEQext2 = true;
                        continue;
                    }

                    if (mySurf.Surface() is Geom_SphericalSurface && !isoU)
                    {
                        continue;
                    }

                    if (aBox.IsOut(thePoints[k]))
                    {
                        continue;
                    }

                    double Cf1 = cI.FirstParameter();
                    double Cl1 = cI.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cf1))
                    {
                        Cf1 = -1000;
                    }
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cl1))
                    {
                        Cl1 = +1000;
                    }

                    ShapeAnalysis_Curve sac1 = new();
                    double dist1 = sac1.Project(cI, thePoints[k], prec, out pt, out t, Cf1, Cl1);
                    currd2[i] = dist1 * dist1;
                    if ((dist1 <= prec) && (t >= Cf1) && (t <= Cl1))
                    {
                        mp[i] = 3;
                        tp[i] = t;
                    }
                }

                if (mp[0] > 0 && mp[1] > 0 && Math.Abs(tp[0] - tp[1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue;
                }

                if (mp[0] > 0 && (!theP1OnIso || currd2[0] < mind2[0]))
                {
                    theP1OnIso = true;
                    mind2[0] = currd2[0];
                    if (isoU)
                    {
                        theValueP1.SetCoord(isoVal, tp[0]);
                    }
                    else
                    {
                        theValueP1.SetCoord(tp[0], isoVal);
                    }
                }

                if (mp[1] > 0 && (!theP2OnIso || currd2[1] < mind2[1]))
                {
                    theP2OnIso = true;
                    mind2[1] = currd2[1];
                    if (isoU)
                    {
                        theValueP2.SetCoord(isoVal, tp[1]);
                    }
                    else
                    {
                        theValueP2.SetCoord(tp[1], isoVal);
                    }
                }

                if (mp[0] <= 0 || mp[1] <= 0)
                {
                    continue;
                }

                double md2 = currd2[0] + currd2[1];
                if (mindist2 <= md2)
                {
                    continue;
                }

                mindist2 = md2;
                mpt[0] = mp[0];
                mpt[1] = mp[1];
                tpar[0] = tp[0];
                tpar[1] = tp[1];
                theIsTypeU = isoU;
                isoValue = isoVal;
                theCIso = cI;
                theT1 = tt1;
                theT2 = tt2;
            }

            if (mpt[0] > 0 && mpt[1] > 0)
            {
                theP1OnIso = theP2OnIso = true;
                if (theIsTypeU)
                {
                    theValueP1.SetCoord(isoValue, tpar[0]);
                    theValueP2.SetCoord(isoValue, tpar[1]);
                }
                else
                {
                    theValueP1.SetCoord(tpar[0], isoValue);
                    theValueP2.SetCoord(tpar[1], isoValue);
                }

                if (mpt[0] != 3 && mpt[1] != 3)
                {
                    theIsoPar2d3d = true;
                    for (int i = 2; i < theNbPnt && theIsoPar2d3d; i++)
                    {
                        if (tpar[1] > tpar[0])
                        {
                            t = theParams[i];
                        }
                        else
                        {
                            t = theT1 + theT2 - theParams[i];
                        }
                        theCIso!.D0(t, out pt);
                        if (!thePoints[i].IsEqual(pt, prec))
                        {
                            theIsoPar2d3d = false;
                        }
                    }
                }

                if (theIsoPar2d3d)
                {
                    isoParam = true;
                }
                else
                {
                    double prevParam = tpar[0];
                    double Cf, Cl;
                    bool isoByDistance = true;
                    Cf = theCIso!.FirstParameter();
                    Cl = theCIso.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cf))
                    {
                        Cf = -1000;
                    }
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Cl))
                    {
                        Cl = +1000;
                    }

                    ShapeAnalysis_Curve sac = new();
                    for (int i = 2; i < theNbPnt && isoByDistance; i++)
                    {
                        double dist = sac.NextProject(prevParam, theCIso, thePoints[i], prec, out pt, out t, Cf, Cl, false);
                        prevParam = t;
                        theParamsOut[i] = t;
                        if ((dist > prec) || (t < Cf) || (t > Cl))
                        {
                            isoByDistance = false;
                        }
                    }
                    if (isoByDistance)
                    {
                        isoParam = true;
                    }
                }
            }
            return isoParam;
        }
        catch (FailureException)
        {
            return false;
        }
    }

    //=================================================================================================
    // OCCT's anonymous-namespace helpers
    //=================================================================================================

    // Utility class for projecting points onto a surface with B-spline corner cache optimization.
    // For clamped B-spline surfaces, caches corner pole positions and their exact UV parameters
    // to avoid expensive ValueOfUV calls when projected points coincide with surface corners.
    private sealed class SurfaceProjectorWithCache
    {
        private readonly ShapeAnalysis_Surface? mySurf; // Surface to project on
        private readonly List<Point3D> myCorners3d = []; // 3D positions of B-spline surface corners
        private readonly List<Point2D> myCorners2d = []; // UV parameters of B-spline surface corners

        public SurfaceProjectorWithCache(ShapeAnalysis_Surface? theSurf)
        {
            mySurf = theSurf;
            if (theSurf is null)
            {
                return;
            }

            if (theSurf.Surface() is Geom_BSplineSurface aBSplineSurf)
            {
                buildCornerCache(aBSplineSurf);
            }
        }

        // Projects a 3D point onto the surface: first the B-spline corner cache, then ValueOfUV.
        public Point2D ValueOfUV(in Point3D thePoint, double theTol, double theTolSq)
        {
            if (findInCornerCache(thePoint, theTolSq, out Point2D aResult))
            {
                // Corner UV is exact, but refine for numerical stability
                return mySurf!.NextValueOfUV(aResult, thePoint, theTol, theTol);
            }
            return mySurf!.ValueOfUV(thePoint, theTol);
        }

        // Projects a 3D point with a hint: first the B-spline corner cache, then NextValueOfUV.
        public Point2D NextValueOfUV(in Point2D theHint, in Point3D thePoint, double theTol, double theTolSq, double theStep)
        {
            if (findInCornerCache(thePoint, theTolSq, out Point2D aResult))
            {
                // Corner UV is exact, but refine for numerical stability
                return mySurf!.NextValueOfUV(aResult, thePoint, theTol, theTol);
            }
            return mySurf!.NextValueOfUV(theHint, thePoint, theTol, theStep);
        }

        // Returns the gap from the last projection
        public double Gap() => mySurf!.Gap();

        // Builds the cache from the B-spline surface's corner poles (where it is clamped).
        private void buildCornerCache(Geom_BSplineSurface theSurface)
        {
            int aNbPolesU = theSurface.NbUPoles();
            int aNbPolesV = theSurface.NbVPoles();
            int aDegreeU = theSurface.UDegree();
            int aDegreeV = theSurface.VDegree();

            // Check if surface is clamped (end multiplicities = degree + 1)
            int aFirstUMult = theSurface.UMultiplicity(1);
            int aLastUMult = theSurface.UMultiplicity(theSurface.NbUKnots());
            int aFirstVMult = theSurface.VMultiplicity(1);
            int aLastVMult = theSurface.VMultiplicity(theSurface.NbVKnots());

            bool isUFirstClamped = (aFirstUMult >= aDegreeU + 1);
            bool isULastClamped = (aLastUMult >= aDegreeU + 1);
            bool isVFirstClamped = (aFirstVMult >= aDegreeV + 1);
            bool isVLastClamped = (aLastVMult >= aDegreeV + 1);

            // Get parameter bounds
            double aUFirst = theSurface.UKnot(1);
            double aULast = theSurface.UKnot(theSurface.NbUKnots());
            double aVFirst = theSurface.VKnot(1);
            double aVLast = theSurface.VKnot(theSurface.NbVKnots());

            // Corner (1, 1) - UFirst, VFirst
            if (isUFirstClamped && isVFirstClamped)
            {
                myCorners3d.Add(theSurface.Pole(1, 1));
                myCorners2d.Add(new Point2D(aUFirst, aVFirst));
            }

            // Corner (NbPolesU, 1) - ULast, VFirst
            if (isULastClamped && isVFirstClamped)
            {
                myCorners3d.Add(theSurface.Pole(aNbPolesU, 1));
                myCorners2d.Add(new Point2D(aULast, aVFirst));
            }

            // Corner (1, NbPolesV) - UFirst, VLast
            if (isUFirstClamped && isVLastClamped)
            {
                myCorners3d.Add(theSurface.Pole(1, aNbPolesV));
                myCorners2d.Add(new Point2D(aUFirst, aVLast));
            }

            // Corner (NbPolesU, NbPolesV) - ULast, VLast
            if (isULastClamped && isVLastClamped)
            {
                myCorners3d.Add(theSurface.Pole(aNbPolesU, aNbPolesV));
                myCorners2d.Add(new Point2D(aULast, aVLast));
            }
        }

        // Finds the first corner within tolerance of the point.
        private bool findInCornerCache(in Point3D thePoint, double theTolSq, out Point2D theUV)
        {
            for (int i = 0; i < myCorners3d.Count; ++i)
            {
                if (myCorners3d[i].SquareDistance(thePoint) < theTolSq)
                {
                    theUV = myCorners2d[i];
                    return true;
                }
            }
            theUV = new Point2D();
            return false;
        }
    }

    // Extracts the B-spline basis of a possibly nested trimmed curve, or null.
    private static Geom_BSplineCurve? extractBSplineCurve(Geom_Curve? theCurve)
    {
        Geom_Curve? aCurve = theCurve;

        // Recursively unwrap trimmed curves
        while (aCurve is Geom_TrimmedCurve aTrimmed)
        {
            aCurve = aTrimmed.BasisCurve();
        }

        return aCurve as Geom_BSplineCurve;
    }

    // Adjusts the second point to the first within half a period of a periodic surface.
    private static void adjustSecondToFirstPoint(in Point2D theFirstPoint, ref Point2D theSecondPoint, Geom_Surface theSurf)
    {
        if (theSurf.IsUPeriodic())
        {
            double anUPeriod = theSurf.UPeriod();
            double aNewU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(theSecondPoint.X(), theFirstPoint.X() - anUPeriod / 2,
                                           theFirstPoint.X() + anUPeriod / 2);
            theSecondPoint.SetX(aNewU);
        }
        if (theSurf.IsVPeriodic())
        {
            double aVPeriod = theSurf.VPeriod();
            double aNewV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(theSecondPoint.Y(), theFirstPoint.Y() - aVPeriod / 2,
                                           theFirstPoint.Y() + aVPeriod / 2);
            theSecondPoint.SetY(aNewV);
        }
    }

    // Fixes possible period jumps in an array of 4 points. Returns true if a period jump was found and fixed.
    private static bool fixPeriodicityTroubles(Span<Point2D> thePnt, int theIdx, double thePeriod, int theSavedPoint,
                                               double theSavedParam)
    {
        double aSavedParam;
        int aSavedPoint;
        double aMinParam = 0.0, aMaxParam = thePeriod;

        if (theSavedPoint < 0)
        {
            aSavedParam = 0.5 * thePeriod;
            aSavedPoint = 0;
        }
        else
        {
            aSavedParam = theSavedParam;
            aSavedPoint = theSavedPoint;
            while (aMinParam > aSavedParam)
            {
                aMinParam -= thePeriod;
                aMaxParam -= thePeriod;
            }
            while (aMaxParam < aSavedParam)
            {
                aMinParam += thePeriod;
                aMaxParam += thePeriod;
            }
        }

        double aFixIsoParam = aMinParam;
        bool isIsoLine = false;
        if (aMaxParam - aSavedParam < global::Kerf.Engine.Foundation.Precision.PConfusion() || aSavedParam - aMinParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            aFixIsoParam = aSavedParam;
            isIsoLine = true;
        }

        // Normalize all coordinates to [aMinParam, aMaxParam)
        for (int i = 0; i < 4; i++)
        {
            double aParam = thePnt[i].Coord(theIdx);
            double aShift = ShapeAnalysis.AdjustToPeriod(aParam, aMinParam, aMaxParam);
            aParam += aShift;

            if (isIsoLine)
            {
                if (aMaxParam - aParam < global::Kerf.Engine.Foundation.Precision.PConfusion() || aParam - aMinParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aParam = aFixIsoParam;
                }
            }
            else
            {
                if (aMaxParam - aParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aParam = aMaxParam;
                }
                if (aParam - aMinParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aParam = aMinParam;
                }
            }

            thePnt[i].SetCoord(theIdx, aParam);
        }

        // Find possible period jump and increasing or decreasing coordinates vector
        bool isJump = false;
        double aPrevDiff = 0.0;
        double aSumDiff = 1.0;
        for (int i = 0; i < 3; i++)
        {
            double aDiff = thePnt[i + 1].Coord(theIdx) - thePnt[i].Coord(theIdx);
            if (aDiff < -global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aSumDiff *= -1.0;
            }
            if (aDiff * aPrevDiff < -global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isJump = true;
            }
            aPrevDiff = aDiff;
        }

        if (!isJump)
        {
            return false;
        }

        if (aSumDiff > 0)
        {
            for (int i = aSavedPoint; i > 0; i--)
            {
                if (thePnt[i].Coord(theIdx) > thePnt[i - 1].Coord(theIdx))
                {
                    thePnt[i - 1].SetCoord(theIdx, thePnt[i - 1].Coord(theIdx) + thePeriod);
                }
            }
            for (int i = aSavedPoint; i < 3; i++)
            {
                if (thePnt[i].Coord(theIdx) < thePnt[i + 1].Coord(theIdx))
                {
                    thePnt[i + 1].SetCoord(theIdx, thePnt[i + 1].Coord(theIdx) - thePeriod);
                }
            }
        }
        else
        {
            for (int i = aSavedPoint; i > 0; i--)
            {
                if (thePnt[i].Coord(theIdx) < thePnt[i - 1].Coord(theIdx))
                {
                    thePnt[i - 1].SetCoord(theIdx, thePnt[i - 1].Coord(theIdx) - thePeriod);
                }
            }
            for (int i = aSavedPoint; i < 3; i++)
            {
                if (thePnt[i].Coord(theIdx) > thePnt[i + 1].Coord(theIdx))
                {
                    thePnt[i + 1].SetCoord(theIdx, thePnt[i + 1].Coord(theIdx) + thePeriod);
                }
            }
        }

        return true;
    }

    // True if the B-spline curve (possibly trimmed) has a parametrisation so uneven that ProjLib must project it.
    private static bool isBSplineCurveInvalid(Geom_Curve theCurve, double theFirst, double theLast,
                                              out Geom_BSplineCurve? theBSpline)
    {
        theBSpline = extractBSplineCurve(theCurve);
        if (theBSpline is null)
        {
            return false;
        }

        // Compute parametrization speed on each knot interval inside [theFirst, theLast].
        // If quotient = (MaxSpeed / MinSpeed) >= aMaxQuotientCoeff then use PerformByProjLib.
        double aFirstParam = theFirst;
        double aLastParam;

        // Find first knot index
        int anIdx = 1;
        for (; anIdx <= theBSpline.NbKnots() && aFirstParam < theLast; anIdx++)
        {
            if (theBSpline.Knot(anIdx) > theFirst)
            {
                break;
            }
        }

        GeomAdaptor_Curve aC3DAdaptor = new(theCurve);
        double aMinParSpeed = global::Kerf.Engine.Foundation.Precision.Infinite();
        NCollection_Sequence<double> aKnotCoeffs = new();

        for (; anIdx <= theBSpline.NbKnots() && aFirstParam < theLast; anIdx++)
        {
            // Fill current knot interval
            aLastParam = Min(theLast, theBSpline.Knot(anIdx));
            int aNbIntPnts = THE_NCONTROL;

            // Adapt number of inner points according to the length of the interval
            // to avoid a lot of calculations on small range of parameters.
            if (anIdx > 1)
            {
                const double aLenThres = 1.0e-2;
                double aLenRatio = (aLastParam - aFirstParam) / (theBSpline.Knot(anIdx) - theBSpline.Knot(anIdx - 1));
                if (aLenRatio < aLenThres)
                {
                    aNbIntPnts = (int)(aLenRatio / aLenThres * aNbIntPnts);
                    if (aNbIntPnts < 2)
                    {
                        aNbIntPnts = 2;
                    }
                }
            }

            double aStep = (aLastParam - aFirstParam) / (aNbIntPnts - 1);

            // Start filling from first point
            aC3DAdaptor.D0(aFirstParam, out Point3D p3d1);

            double aLength3d = 0.0;
            for (int anIntIdx = 1; anIntIdx < aNbIntPnts; anIntIdx++)
            {
                double aParam = aFirstParam + aStep * anIntIdx;
                aC3DAdaptor.D0(aParam, out Point3D p3d2);
                double aDist = p3d2.Distance(p3d1);

                aLength3d += aDist;
                p3d1 = p3d2;

                aMinParSpeed = Min(aMinParSpeed, aDist / aStep);
            }

            double aCoeff = aLength3d / (aLastParam - aFirstParam);
            if (Math.Abs(aCoeff) > Constants.Resolution())
            {
                aKnotCoeffs.Append(aCoeff);
            }
            aFirstParam = aLastParam;
        }

        double anEvenlyCoeff = 0;
        if (aKnotCoeffs.Size() > 0)
        {
            // std::max_element / std::min_element: the first of equal elements, as a plain scan
            double aMax = aKnotCoeffs.Value(1), aMin = aKnotCoeffs.Value(1);
            for (int i = 2; i <= aKnotCoeffs.Size(); i++)
            {
                if (aMax < aKnotCoeffs.Value(i))
                {
                    aMax = aKnotCoeffs.Value(i);
                }
                if (aKnotCoeffs.Value(i) < aMin)
                {
                    aMin = aKnotCoeffs.Value(i);
                }
            }
            anEvenlyCoeff = aMax / aMin;
        }

        const double aMaxQuotientCoeff = 1500.0;
        return (anEvenlyCoeff > aMaxQuotientCoeff && aMinParSpeed > global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Generates the discretization points of a curve: uniform, at least theNbControlPoints, more for B-splines with
    // many spans. Resizes the arrays and returns the number of points.
    private static int generateCurvePoints(Geom_Curve theCurve, double theFirst, double theLast, int theNbControlPoints,
                                           NCollection_Array1<Point3D> thePoints, NCollection_Array1<double> theParams)
    {
        Geom_BSplineCurve? aBSpline = extractBSplineCurve(theCurve);
        int aNbPini = theNbControlPoints;

        if (aBSpline is not null)
        {
            int aUsedKnots = 0;
            for (int i = 1; i < aBSpline.NbKnots(); i++)
            {
                if (aBSpline.Knot(i + 1) > theFirst && aBSpline.Knot(i) < theLast)
                {
                    aUsedKnots++;
                }
            }
            int aMinPnt = aUsedKnots * (aBSpline.Degree() + 1);
            while (aNbPini < aMinPnt)
            {
                aNbPini += THE_NCONTROL - 1;
            }
        }

        thePoints.Resize(1, aNbPini, false);
        theParams.Resize(1, aNbPini, false);

        double aDeltaParam = (theLast - theFirst) / (aNbPini - 1);

        for (int i = 1; i <= aNbPini; ++i)
        {
            double aParam;
            if (i == 1)
            {
                aParam = theFirst;
            }
            else if (i == aNbPini)
            {
                aParam = theLast;
            }
            else
            {
                aParam = theFirst + (i - 1) * aDeltaParam;
            }

            theCurve.D0(aParam, out Point3D aPoint);
            thePoints.SetValue(i, aPoint);
            theParams.SetValue(i, aParam);
        }

        return aNbPini;
    }

    // ShapeAnalysis_Surface::ProjectDegenerated over the arrays (it takes sequences).
    private static void projectDegeneratedPoints(ShapeAnalysis_Surface theSurf, int theNbPnt, NCollection_Array1<Point3D> thePoints,
                                                 NCollection_Array1<Point2D> thePoints2d, double thePreci, bool theDirect)
    {
        // Convert arrays to sequences for ShapeAnalysis_Surface::ProjectDegenerated
        NCollection_Sequence<Point3D> aPoints3d = new();
        NCollection_Sequence<Point2D> aPoints2d = new();

        for (int i = 1; i <= theNbPnt; ++i)
        {
            aPoints3d.Append(thePoints[i]);
            aPoints2d.Append(thePoints2d[i]);
        }

        // Call the method that expects sequences
        theSurf.ProjectDegenerated(theNbPnt, aPoints3d, aPoints2d, thePreci, theDirect);

        // Copy results back to array
        for (int i = 1; i <= theNbPnt; ++i)
        {
            thePoints2d.SetValue(i, aPoints2d.Value(i));
        }
    }
}
