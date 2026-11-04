// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The edge/face intersection algorithm: the common parts of an edge and a face in 3D space, vertices (intersection
/// or touching) or the edge (the edge lies in the face).
/// </summary>
/// <remarks>
/// A working object (plan section 6), derived from by the pave filler's <c>BOPAlgo_EdgeFace</c> (unit 6E). It uses
/// the context it is given (or a new one) for the face's adaptor, projector and classifier and for the localisation
/// data of <see cref="BeanFaceIntersector"/>, which gives every run the result a fresh context would give
/// (decision 753). The common parts are the algorithm's own <see cref="CommonPrt"/> objects.
/// </remarks>
internal class EdgeFace
{
    protected Edge _edge = new();
    protected Face _face = new();
    protected double _fuzzyValue;
    protected EdgeCurveAdaptor _c = new();
    protected FaceSurfaceAdaptor _s = new();
    protected double _criteria;
    protected bool _isDone;
    protected int _errorStatus;
    protected IntersectionContext? _context;
    protected readonly NCollection_Sequence<CommonPrt> _seqOfCommonPrts = new();
    protected ParameterRange _range;
    protected bool _quickCoincidenceCheck;

    /// <summary>The minimal distance found.</summary>
    protected double _minDistance;

    /// <summary>An empty algorithm.</summary>
    public EdgeFace()
    {
        _fuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isDone = false;
        _errorStatus = 1;
        _quickCoincidenceCheck = false;
        _minDistance = RealLast();
    }

    /// <summary>Sets the edge.</summary>
    public void SetEdge(in Edge edge) => _edge = edge;

    /// <summary>The edge.</summary>
    public Edge Edge() => _edge;

    /// <summary>Sets the face.</summary>
    public void SetFace(in Face face) => _face = face;

    /// <summary>The face.</summary>
    public Face Face() => _face;

    /// <summary>Sets the range of the edge processed by the algorithm.</summary>
    public void SetRange(in ParameterRange range) => _range = range;

    /// <summary>Sets the range of the edge processed by the algorithm.</summary>
    public void SetRange(double first, double last)
    {
        _range.SetFirst(first);
        _range.SetLast(last);
    }

    /// <summary>The range of the edge.</summary>
    public ParameterRange Range() => _range;

    /// <summary>Sets the intersection context.</summary>
    public void SetContext(IntersectionContext? context) => _context = context;

    /// <summary>The intersection context.</summary>
    public IntersectionContext? Context() => _context;

    /// <summary>Sets the fuzzy value (at least <see cref="Precision.Confusion"/>).</summary>
    public void SetFuzzyValue(double fuzz) => _fuzzyValue = Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => _fuzzyValue;

    /// <summary>
    /// Sets the flag of the quick coincidence check, safe only if the edge's vertices lie on the face and the edge
    /// doesn't cross the face's boundaries on the range.
    /// </summary>
    public void UseQuickCoincidenceCheck(bool flag) => _quickCoincidenceCheck = flag;

    /// <summary>The flag of the quick coincidence check.</summary>
    public bool IsCoincidenceCheckedQuickly => _quickCoincidenceCheck;

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>
    /// The completion code: 0 success, 1 not started, 2 and 3 invalid data (degenerated or non-geometric edge), 4 a
    /// projection failed.
    /// </summary>
    public int ErrorStatus() => _errorStatus;

    /// <summary>The common parts (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<CommonPrt> CommonParts() => _seqOfCommonPrts;

    /// <summary>The minimal distance found between the edge and the face.</summary>
    public double MinimalDistance() => _minDistance;

    /// <summary>Checks if the edge really lies in the face.</summary>
    protected bool IsCoincident()
    {
        int i, iCnt;
        double dT, t, d, t1, t2, u, v;

        Point3D p;
        Containment state;
        Point2D p2d = new();
        //
        ProjectPointOnSurface projector = _context!.ProjPS(_face);

        int nbSeg = 23;
        if (_c.GetType() == Line && _s.GetType() == Plane)
        {
            nbSeg = 2; // Check only three points for Line/Plane intersection
        }

        double tresh = 0.5;
        int treshIdxF = RealToInt((nbSeg + 1) * 0.25),
            treshIdxL = RealToInt((nbSeg + 1) * 0.75);

        t1 = _range.First();
        t2 = _range.Last();
        double bndShift = 0.01 * (t2 - t1);
        // Shifting first and last curve points in order to avoid projection
        // on surface boundary and rejection projection point with minimal distance
        t1 += bndShift;
        t2 -= bndShift;
        dT = (t2 - t1) / nbSeg;
        //
        bool isClassified = false;
        iCnt = 0;
        for (i = 0; i <= nbSeg; ++i)
        {
            t = t1 + i * dT;
            p = _c.Value(t);
            //
            projector.Perform(p);
            if (!projector.IsDone)
            {
                continue;
            }
            //

            d = projector.LowerDistance();
            if (d > _criteria)
            {
                if (d > 100.0 * _criteria)
                {
                    return false;
                }
                else
                {
                    continue;
                }
            }
            //

            ++iCnt;

            // We classify only three points: in the begin, in the
            // end and in the middle of the edge.
            // However, exact middle point (when i == (aNbSeg + 1)/2)
            // can be unprojectable. Therefore, it will not be able to
            // be classified. Therefore, points with indexes in
            //[aTreshIdxF, aTreshIdxL] range are made available
            // for classification.
            // isClassified == TRUE if MIDDLE point has been chosen and
            // classified correctly.

            if (((0 < i) && (i < treshIdxF)) || ((treshIdxL < i) && (i < nbSeg)))
            {
                continue;
            }

            if (isClassified && (i != nbSeg))
            {
                continue;
            }

            projector.LowerDistanceParameters(out u, out v);
            p2d = p2d.WithX(u);
            p2d = p2d.WithY(v);

            FaceClassifier2D class2d = _context.FClass2d(_face);
            state = class2d.Perform(p2d);

            if (state == Out)
            {
                return false;
            }

            if (i != 0)
            {
                isClassified = true;
            }
        }
        //
        double coeff = (double)iCnt / ((double)nbSeg + 1);
        return coeff > tresh;
    }

    /// <summary>Checks the data.</summary>
    protected void CheckData()
    {
        if (Tool.Degenerated(_edge))
        {
            _errorStatus = 2;
        }
        if (!Tool.IsGeometric(_edge))
        {
            _errorStatus = 3;
        }
    }

    /// <summary>True if the point of the edge at <paramref name="aT"/> projects onto the face within the criteria.</summary>
    protected bool IsProjectable(double aT)
    {
        bool bFlag;
        Point3D pc;
        //
        _c.D0(aT, out pc);
        bFlag = _context!.IsValidPointForFace(pc, _face, _criteria);
        //
        return bFlag;
    }

    /// <summary>The distance from the edge's point at <paramref name="t"/> to the face's surface, less the criteria.</summary>
    protected double DistanceFunction(double t)
    {
        double d = 0.0;

        //
        Point3D p;
        _c.D0(t, out p);
        //
        bool bIsEqDistance;

        bIsEqDistance = EdgeFace.IsEqDistance(p, _s, 1.0e-7, ref d);
        if (bIsEqDistance)
        {
            d = d - _criteria;
            return d;
        }

        //
        bool bFlag = false;

        ProjectPointOnSurface locProj = _context!.ProjPS(_face);
        locProj.Perform(p);
        bFlag = locProj.IsDone;

        if (bFlag)
        {
            d = locProj.LowerDistance();
        }
        //

        if (!bFlag)
        {
            _errorStatus = 4;
            return 99.0;
        }

        //
        //   aD=aProjector.LowerDistance();
        //
        d = d - _criteria;
        return d;
    }

    /// <summary>
    /// True, with the distance in <paramref name="aD"/>, if the point is on the axis of a cylinder or cone or on the
    /// spine of a torus (all points of the surface's circle are then at the same distance).
    /// </summary>
    protected static bool IsEqDistance(in Point3D aP, FaceSurfaceAdaptor aBAS, double aTol, ref double aD)
    {
        bool bRetFlag = true;

        SurfaceKind surfType = aBAS.GetType();

        if (surfType == SurfaceKind.Cylinder)
        {
            Cylinder cyl = aBAS.Cylinder();
            Axis3D ax1 = cyl.Axis;
            Line3D linAxis = new(ax1);
            double dc, radius = cyl.Radius;
            dc = linAxis.DistanceTo(aP);
            if (dc < aTol)
            {
                aD = radius;
                return bRetFlag;
            }
        }

        if (surfType == SurfaceKind.Cone)
        {
            Cone cone = aBAS.Cone();
            Axis3D ax1 = cone.Axis;
            Line3D linAxis = new(ax1);
            double dc, radius, ds, semiAngle;
            dc = linAxis.DistanceTo(aP);
            if (dc < aTol)
            {
                Point3D apex = cone.Apex;
                semiAngle = cone.SemiAngle;
                ds = aP.DistanceTo(apex);

                radius = ds * Math.Tan(semiAngle);
                aD = radius;
                return bRetFlag;
            }
        }

        if (surfType == SurfaceKind.Torus)
        {
            double majorRadius, minorRadius, dc;

            Torus torus = aBAS.Torus();
            Point3D pLoc = torus.Location;
            majorRadius = torus.MajorRadius;

            dc = Math.Abs(pLoc.DistanceTo(aP) - majorRadius);
            if (dc < aTol)
            {
                minorRadius = torus.MinorRadius;
                aD = minorRadius;
                return bRetFlag;
            }
        }
        return !bRetFlag;
    }

    /// <summary>Sets the type of the common part (edge or vertex) and its vertex parameter.</summary>
    protected int MakeType(CommonPrt aCommonPrt)
    {
        double af1, al1;
        double df1, tm = 0.0;
        bool bAllNullFlag;
        //
        bAllNullFlag = aCommonPrt.AllNullFlag();
        if (bAllNullFlag)
        {
            aCommonPrt.SetType(global::Kerf.Topology.ShapeType.Edge);
            return 0;
        }
        //
        aCommonPrt.Range1(out af1, out al1);

        {
            Point3D pf, pl;
            _c.D0(af1, out pf);
            _c.D0(al1, out pl);
            df1 = pf.DistanceTo(pl);
            bool isWholeRange = false;

            if ((Math.Abs(af1 - _range.First()) < _c.Resolution(_criteria))
                && (Math.Abs(al1 - _range.Last()) < _c.Resolution(_criteria)))
            {
                isWholeRange = true;
            }

            if ((df1 > _criteria * 2.0) && isWholeRange)
            {
                aCommonPrt.SetType(global::Kerf.Topology.ShapeType.Edge);
            }
            else
            {
                if (isWholeRange)
                {
                    tm = (af1 + al1) * 0.5;

                    if (pf.DistanceTo(_c.Value(tm)) > _criteria * 2.0)
                    {
                        aCommonPrt.SetType(global::Kerf.Topology.ShapeType.Edge);
                        return 0;
                    }
                }

                if (!CheckTouch(aCommonPrt, ref tm))
                {
                    tm = (af1 + al1) * 0.5;
                }
                aCommonPrt.SetType(ShapeType.Vertex);
                aCommonPrt.SetVertexParameter1(tm);
                aCommonPrt.SetRange1(af1, al1);
            }
        }
        return 0;
    }

    /// <summary>True, with the touching parameter in <paramref name="aTx"/>, if the edge touches the face on the part.</summary>
    protected bool CheckTouch(CommonPrt aCP, ref double aTx)
    {
        double tf, tl, tol, u1f, u1l, v1f, v1l, dist2, minDist2;
        bool theflag = false;
        int nbExt, iLower;

        aCP.Range1(out tf, out tl);

        //
        double cr;
        cr = _c.Resolution(_criteria);
        if ((Math.Abs(tf - _range.First()) < cr) && (Math.Abs(tl - _range.Last()) < cr))
        {
            return theflag; // EDGE
        }
        //

        tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        Curve3D curve = Tool.Curve(_c.Edge(), out _, out _)!;
        Surface surface = Tool.Surface(_s.Face())!;
        //   Surface->Bounds(U1f,U1l,V1f,V1l);
        u1f = _s.FirstUParameter();
        u1l = _s.LastUParameter();
        v1f = _s.FirstVParameter();
        v1l = _s.LastVParameter();

        CurveAdaptor theCurve = new(curve, tf, tl);
        SurfaceAdaptor theSurface = new(surface, u1f, u1l, v1f, v1l);

        ExtCS extrema = new(theCurve, theSurface, tol, tol);

        dist2 = 1.0e100;

        if (extrema.IsDone)
        {
            minDist2 = dist2;

            if (!extrema.IsParallel)
            {
                nbExt = extrema.ExtCount;

                if (nbExt > 0)
                {
                    iLower = 1;
                    for (int i = 1; i <= nbExt; i++)
                    {
                        dist2 = extrema.SquareDistance(i);
                        if (dist2 < minDist2)
                        {
                            minDist2 = dist2;
                            iLower = i;
                        }
                    }
                    dist2 = extrema.SquareDistance(iLower);
                    extrema.Points(iLower, out POnCurv pOnC, out _);
                    aTx = pOnC.Parameter();
                }
                else
                {
                    // modified by NIZHNY-MKK  Thu Jul 21 11:35:32 2005.BEGIN
                    HInter exactIntersector = new();

                    // OCCT copies the adaptors into handles; they aren't used afterwards, so the port passes them.
                    exactIntersector.Perform(theCurve, theSurface);

                    if (exactIntersector.IsDone)
                    {
                        for (int i = 1; i <= exactIntersector.PointCount; i++)
                        {
                            CurveSurfaceIntersectionPoint point = exactIntersector.Point(i);

                            if ((point.W() >= tf) && (point.W() <= tl))
                            {
                                dist2 = 0.0;
                                aTx = point.W();
                            }
                        }
                    }
                    // modified by NIZHNY-MKK  Thu Jul 21 11:35:40 2005.END
                }
            }
            else
            {
                return theflag;
            }
        }

        double boundaryDist;

        boundaryDist = DistanceFunction(tf) + _criteria;
        if (boundaryDist * boundaryDist < dist2)
        {
            dist2 = boundaryDist * boundaryDist;
            aTx = tf;
        }

        boundaryDist = DistanceFunction(tl) + _criteria;
        if (boundaryDist * boundaryDist < dist2)
        {
            dist2 = boundaryDist * boundaryDist;
            aTx = tl;
        }

        double parameter = (tf + tl) * 0.5;
        boundaryDist = DistanceFunction(parameter) + _criteria;
        if (boundaryDist * boundaryDist < dist2)
        {
            dist2 = boundaryDist * boundaryDist;
            aTx = parameter;
        }

        if (dist2 > _criteria * _criteria)
        {
            return theflag;
        }

        if (Math.Abs(aTx - tf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return !theflag;
        }

        if (Math.Abs(aTx - tl) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return !theflag;
        }

        if (aTx > tf && aTx < tl)
        {
            return !theflag;
        }

        return theflag;
    }

    /// <summary>Launches the process.</summary>
    public void Perform()
    {
        int i, nb;
        CommonPrt commonPrt = new();
        //
        commonPrt.SetEdge1(_edge);
        //
        _errorStatus = 0;
        CheckData();
        if (_errorStatus != 0)
        {
            return;
        }
        //
        _context ??= new IntersectionContext();
        //
        _isDone = false;
        _c.Initialize(_edge);
        CurveKind curveType;
        curveType = _c.GetType();
        //
        // Prepare myCriteria
        double fuzz = _fuzzyValue / 2.0;
        double tolF = Tool.Tolerance(_face) + fuzz;
        double tolE = Tool.Tolerance(_edge) + fuzz;
        if (curveType == BSplineCurve || curveType == BezierCurve)
        {
            //--- 5112
            double diff1 = (tolE / tolF);
            double diff2 = (tolF / tolE);
            if (diff1 > 100 || diff2 > 100)
            {
                _criteria = Max(tolE, tolF);
            }
            else
            { //--- 5112
                _criteria = 1.5 * tolE + tolF;
            }
        }
        else
        {
            _criteria = tolE + tolF;
        }

        // OCCT copies the context's adaptor; the port shares it (adaptors are only read here).
        _s = _context.SurfaceAdaptor(_face);

        if (_quickCoincidenceCheck)
        {
            if (IsCoincident())
            {
                commonPrt.SetType(global::Kerf.Topology.ShapeType.Edge);
                commonPrt.SetRange1(_range.First(), _range.Last());
                _seqOfCommonPrts.Append(new CommonPrt(commonPrt));
                _isDone = true;
                return;
            }
        }
        //
        BeanFaceIntersector intersector = new(_c, _s, tolE, tolF);
        intersector.SetBeanParameters(_range.First(), _range.Last());
        //
        intersector.SetContext(_context);
        //
        intersector.Perform();

        if (intersector.MinimalSquareDistance() < RealLast())
        {
            _minDistance = Math.Sqrt(intersector.MinimalSquareDistance());
        }

        if (!intersector.IsDone)
        {
            return;
        }

        for (int r = 1; r <= intersector.Result().Length(); r++)
        {
            ParameterRange range = intersector.Result().Value(r);

            if (IsProjectable(IntersectionTools.IntermediatePoint(range.First(), range.Last())))
            {
                commonPrt.SetRange1(range.First(), range.Last());
                _seqOfCommonPrts.Append(new CommonPrt(commonPrt));
            }
        }

        nb = _seqOfCommonPrts.Length();

        for (i = 1; i <= nb; i++)
        {
            CommonPrt cp = _seqOfCommonPrts.Value(i);
            //
            double tx1, tx2;
            Point3D px1, px2;
            //
            cp.Range1(out tx1, out tx2);
            _c.D0(tx1, out px1);
            _c.D0(tx2, out px2);
            cp.SetBoundingPoints(px1, px2);
            //
            MakeType(cp);
        }
        {
            // Line\Cylinder's Common Parts treatment
            CurveKind cType;
            SurfaceKind sType;
            ShapeType type;
            bool bIsTouch;
            double tx = 0.0;

            cType = _c.GetType();
            sType = _s.GetType();

            if (cType == Line && sType == SurfaceKind.Cylinder)
            {
                for (i = 1; i <= nb; i++)
                {
                    CommonPrt cp = _seqOfCommonPrts.Value(i);
                    type = cp.Type();
                    if (type == global::Kerf.Topology.ShapeType.Edge)
                    {
                        bIsTouch = CheckTouch(cp, ref tx);
                        if (bIsTouch)
                        {
                            cp.SetType(ShapeType.Vertex);
                            cp.SetVertexParameter1(tx);
                            // aCP.SetRange1 (aTx, aTx);
                        }
                    }
                    else if (type == ShapeType.Vertex)
                    {
                        bIsTouch = CheckTouchVertex(cp, ref tx);
                        if (bIsTouch)
                        {
                            cp.SetVertexParameter1(tx);
                            // aCP.SetRange1 (aTx, aTx);
                        }
                    }
                }
            }

            // Circle\Plane's Common Parts treatment

            if (cType == Circle && sType == Plane)
            {
                bool bIsCoplanar, bIsRadius;
                bIsCoplanar = IsCoplanar(_c, _s);
                bIsRadius = IsRadius(_c, _s, _criteria);
                if (!bIsCoplanar && !bIsRadius)
                {
                    for (i = 1; i <= nb; i++)
                    {
                        CommonPrt cp = _seqOfCommonPrts.Value(i);
                        type = cp.Type();
                        if (type == global::Kerf.Topology.ShapeType.Edge)
                        {
                            bIsTouch = CheckTouch(cp, ref tx);
                            if (bIsTouch)
                            {
                                cp.SetType(ShapeType.Vertex);
                                cp.SetVertexParameter1(tx);
                                // aCP.SetRange1 (aTx, aTx);
                            }
                        }
                        else if (type == ShapeType.Vertex)
                        {
                            bIsTouch = CheckTouchVertex(cp, ref tx);
                            if (bIsTouch)
                            {
                                cp.SetVertexParameter1(tx);
                                // aCP.SetRange1 (aTx, aTx);
                            }
                        }
                    }
                }
            }
        }
        _isDone = true;
    }

    /// <summary>True, with the touching parameter in <paramref name="aTx"/>, if a vertex common part is a touch.</summary>
    protected bool CheckTouchVertex(CommonPrt aCP, ref double aTx)
    {
        double tf, tl, tol, u1f, u1l, v1f, v1l;
        double epsT, dist2, minDist2, tm, dist2New;
        bool theflag = false;
        int nbExt, i, iLower;
        CurveKind type;
        //
        aCP.Range1(out tf, out tl);
        type = _c.GetType();
        //
        epsT = 8.0e-5;
        if (type == Line)
        {
            epsT = 9.0e-5;
        }
        //
        tm = 0.5 * (tf + tl);
        dist2 = DistanceFunction(tm);
        dist2 *= dist2;

        tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        Curve3D curve = Tool.Curve(_c.Edge(), out _, out _)!;
        Surface surface = Tool.Surface(_s.Face())!;

        surface.ParameterBounds(out u1f, out u1l, out v1f, out v1l);

        CurveAdaptor theCurve = new(curve, tf, tl);
        SurfaceAdaptor theSurface = new(surface, u1f, u1l, v1f, v1l);

        ExtCS extrema = new(theCurve, theSurface, tol, tol);

        if (!extrema.IsDone)
        {
            return theflag;
        }
        if (extrema.IsParallel)
        {
            return theflag;
        }

        nbExt = extrema.ExtCount;
        if (nbExt == 0)
        {
            return theflag;
        }

        iLower = 1;
        minDist2 = 1.0e100;
        for (i = 1; i <= nbExt; ++i)
        {
            dist2 = extrema.SquareDistance(i);
            if (dist2 < minDist2)
            {
                minDist2 = dist2;
                iLower = i;
            }
        }

        dist2New = extrema.SquareDistance(iLower);

        if (dist2New > dist2)
        {
            aTx = tm;
            return !theflag;
        }

        if (dist2New > _criteria * _criteria)
        {
            return theflag;
        }

        extrema.Points(iLower, out POnCurv pOnC, out _);

        aTx = pOnC.Parameter();
        //
        if (Math.Abs(aTx - tf) < epsT)
        {
            return theflag;
        }

        if (Math.Abs(aTx - tl) < epsT)
        {
            return theflag;
        }

        if (aTx > tf && aTx < tl)
        {
            return !theflag;
        }

        return theflag;
    }

    // ---- file-static functions of IntTools_EdgeFace.cxx

    private static bool IsCoplanar(EdgeCurveAdaptor aCurve, FaceSurfaceAdaptor aSurface)
    {
        bool bFlag = false;

        CurveKind cType;
        SurfaceKind sType;

        cType = aCurve.GetType();
        sType = aSurface.GetType();

        if (cType == Circle && sType == Plane)
        {
            Circle3D circ = aCurve.Circle();
            Axis3D ax1 = circ.Axis;
            Direction3D dirAx1 = ax1.Direction;

            Plane3D pln = aSurface.Plane();
            Axis3D ax = pln.Axis;
            Direction3D dirPln = ax.Direction;

            bFlag = IntersectionTools.IsDirsCoinside(dirAx1, dirPln);
        }
        return bFlag;
    }

    private static bool IsRadius(EdgeCurveAdaptor aCurve, FaceSurfaceAdaptor aSurface, double aCriteria)
    {
        bool bFlag = false;

        CurveKind cType;
        SurfaceKind sType;

        cType = aCurve.GetType();
        sType = aSurface.GetType();

        if (cType == Circle && sType == Plane)
        {
            Circle3D circ = aCurve.Circle();
            Point3D center = circ.Location;
            double r = circ.Radius;
            Plane3D pln = aSurface.Plane();
            double d = pln.DistanceTo(center);
            if (Math.Abs(d - r) < aCriteria)
            {
                return !bFlag;
            }
        }
        return bFlag;
    }

    /// <summary>
    /// OCCT's file-level <c>AdaptiveDiscret</c>: the number of discretisation points for an edge on a cylinder (at least
    /// <paramref name="iDiscret"/>). Not called in OCCT; ported for completeness.
    /// </summary>
    internal static int AdaptiveDiscret(int iDiscret, EdgeCurveAdaptor aCurve, FaceSurfaceAdaptor aSurface)
    {
        int iDiscretNew;

        iDiscretNew = iDiscret;

        SurfaceKind sType;

        sType = aSurface.GetType();

        if (sType == SurfaceKind.Cylinder)
        {
            double eLength, radius, dLR;

            eLength = global::Kerf.Engine.Intersection.Tools.IntTools.Length(aCurve.Edge());

            Cylinder cylinder = aSurface.Cylinder();
            radius = cylinder.Radius;
            dLR = 2 * radius;

            iDiscretNew = (int)(eLength / dLR);

            if (iDiscretNew < iDiscret)
            {
                iDiscretNew = iDiscret;
            }
        }
        return iDiscretNew;
    }
}
