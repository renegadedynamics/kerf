// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The intersection context: caches of the tools the Boolean component builds per shape (point classifiers,
/// projectors, solid classifiers, hatchers, surface adaptors, boxes), so that each is built once per algorithm run.
/// </summary>
/// <remarks>
/// <para>
/// A working object, not a value (plan section 6). Each thread of a parallel loop has its own context
/// (<see cref="BooleanParallel"/>, decision 580), so a context is used by one thread at a time and needs no lock.
/// The tools it hands out are its own cached objects, as OCCT's references are: callers run them
/// (<c>ProjPS(F).Perform(P)</c>) and read their results before the next call on the same context.
/// </para>
/// <para>
/// The cache audit (plan section 2.4, decision 714): every cached tool but one gives the same answers whatever the
/// context computed before. The exception is OCCT's <see cref="SurfaceData(in Face)"/>, whose boxes
/// IntTools_BeanFaceIntersector keys by sample index and enlarges by the first edge's tolerance (occt-issues.md, unit
/// 6C). The port's intersector uses the internal keyed overload instead, so that it gives every run the result of a
/// fresh context (decision 753).
/// </para>
/// <para>
/// The maps are keyed by shape with <see cref="ShapeHasher"/> (orientation ignored), and by curve with
/// reference equality (OCCT's handle hasher). The allocator goes (translation rules, section 9).
/// </para>
/// </remarks>
internal class IntersectionContext : Transient
{
    private readonly Dictionary<Shape, FaceClassifier2D> _fClass2dMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, ProjectPointOnSurface> _projPSMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, ProjectPointOnCurve> _projPCMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, SolidClassifier> _sClassMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Curve3D, ProjectPointOnCurve> _projPTMap = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<Shape, Hatcher2D> _hatcherMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, SurfaceRangeLocalizeData> _projSDataMap = new(ShapeHasher.Instance);
    // Kerf (decision 753): the localisation data keyed by face and by what the intersector's boxes depend on
    private readonly Dictionary<Shape, Dictionary<SurfaceDataKey, SurfaceRangeLocalizeData>> _sDataKeyedMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Box> _bndBoxDataMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, FaceSurfaceAdaptor> _surfAdaptorMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Obb> _obbMap = new(ShapeHasher.Instance);
    private int _createFlag;
    private double _pOnSTolerance;

    /// <summary>An empty context.</summary>
    public IntersectionContext()
    {
        _createFlag = 0;
        _pOnSTolerance = 1.0e-12;
    }

    /// <summary>
    /// The bounding box of the shape (<see cref="Bounds.Add"/>), computed on the first call. OCCT returns a
    /// reference into its cache; the port returns the box by value.
    /// </summary>
    public Box BndBox(in Shape aS)
    {
        if (!_bndBoxDataMap.TryGetValue(aS, out Box box))
        {
            box = new Box();
            Bounds.Add(aS, ref box);
            _bndBoxDataMap[aS] = box;
        }
        return box;
    }

    /// <summary>True if the face's bounding box is open in some direction (an infinite face).</summary>
    public bool IsInfiniteFace(in Face aFace)
    {
        Box box = BndBox(aFace);
        return box.IsOpenXmax || box.IsOpenXmin || box.IsOpenYmax || box.IsOpenYmin
               || box.IsOpenZmax || box.IsOpenZmin;
    }

    /// <summary>The 2D point classifier of the face (taken FORWARD, with the face's tolerance), built on the first call.</summary>
    public FaceClassifier2D FClass2d(in Face aF)
    {
        if (!_fClass2dMap.TryGetValue(aF, out FaceClassifier2D? pFClass2d))
        {
            double tolF;
            Face ff;
            //
            ff = aF;
            ff.Orient(Forward);
            tolF = Tool.Tolerance(ff);
            //
            pFClass2d = new FaceClassifier2D(ff, tolF);
            //
            _fClass2dMap[ff] = pFClass2d;
        }
        return pFClass2d!;
    }

    /// <summary>
    /// The point projector of the face's surface, bounded by the face's UV bounds, with the context's tolerance and
    /// the minimum-only flag, built on the first call.
    /// </summary>
    public ProjectPointOnSurface ProjPS(in Face aF)
    {
        if (!_projPSMap.TryGetValue(aF, out ProjectPointOnSurface? pProjPS))
        {
            UVBounds(aF, out double umin, out double usup, out double vmin, out double vsup);
            Surface s = Tool.Surface(aF)!;
            //
            pProjPS = new ProjectPointOnSurface();
            pProjPS.Init(s, umin, usup, vmin, vsup, _pOnSTolerance);
            pProjPS.SetExtremaFlag(ExtFlag_MIN);
            //
            _projPSMap[aF] = pProjPS;
        }
        return pProjPS!;
    }

    /// <summary>The point projector of the edge's 3D curve over the edge's range, built on the first call.</summary>
    public ProjectPointOnCurve ProjPC(in Edge aE)
    {
        if (!_projPCMap.TryGetValue(aE, out ProjectPointOnCurve? pProjPC))
        {
            Curve3D? c3D = Tool.Curve(aE, out double f, out double l);
            //
            pProjPC = new ProjectPointOnCurve();
            pProjPC.Init(c3D!, f, l);
            //
            _projPCMap[aE] = pProjPC;
        }
        return pProjPC!;
    }

    /// <summary>The point projector of the curve over its own range, built on the first call (keyed by the curve object).</summary>
    public ProjectPointOnCurve ProjPT(Curve3D aC3D)
    {
        if (!_projPTMap.TryGetValue(aC3D, out ProjectPointOnCurve? pProjPT))
        {
            double f, l;
            f = aC3D.FirstParameter;
            l = aC3D.LastParameter;
            //
            pProjPT = new ProjectPointOnCurve();
            pProjPT.Init(aC3D, f, l);
            //
            _projPTMap[aC3D] = pProjPT;
        }
        return pProjPT!;
    }

    /// <summary>The 3D point classifier of the solid, built on the first call.</summary>
    public SolidClassifier SolidClassifier(in Solid aSolid)
    {
        if (!_sClassMap.TryGetValue(aSolid, out SolidClassifier? pSC))
        {
            pSC = new SolidClassifier(aSolid);
            //
            _sClassMap[aSolid] = pSC;
        }
        return pSC!;
    }

    /// <summary>The surface adaptor of the face (restricted to the face's UV bounds), built on the first call.</summary>
    public FaceSurfaceAdaptor SurfaceAdaptor(in Face face)
    {
        if (!_surfAdaptorMap.TryGetValue(face, out FaceSurfaceAdaptor? pBAS))
        {
            //
            pBAS = new FaceSurfaceAdaptor(face, true);
            //
            _surfAdaptorMap[face] = pBAS;
        }
        return pBAS!;
    }

    /// <summary>
    /// The 2D hatcher of the face (taken FORWARD), loaded with the face's pcurves trimmed to their edges' ranges, built
    /// on the first call.
    /// </summary>
    public Hatcher2D Hatcher(in Face aF)
    {
        if (!_hatcherMap.TryGetValue(aF, out Hatcher2D? pHatcher))
        {
            double tolArcIntr, tolTangfIntr, tolHatch2D, tolHatch3D;
            double epsT;
            Orientation orE;
            Curve2D? c2D;
            TrimmedCurve2D ct2D;
            Face ff;
            Explorer exp = new();
            //
            tolHatch2D = 1.0e-8;
            tolHatch3D = 1.0e-8;
            tolArcIntr = 1.0e-10;
            tolTangfIntr = 1.0e-10;
            epsT = global::Kerf.Engine.Foundation.Precision.PConfusion();
            //
            Intersector2D intr = new(tolArcIntr, tolTangfIntr);
            pHatcher = new Hatcher2D(intr, tolHatch2D, tolHatch3D, true, false);
            //
            ff = aF;
            ff.Orient(Forward);
            _ = Tool.Surface(ff);

            exp.Init(ff, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                orE = e.Orientation;
                //
                c2D = Tool.CurveOnSurface(e, ff, out double u1, out double u2);
                if (c2D is null)
                {
                    continue;
                }
                if (Math.Abs(u1 - u2) < epsT)
                {
                    continue;
                }
                //
                ct2D = new TrimmedCurve2D(c2D, u1, u2);
                CurveAdaptor2D gac = new(ct2D);
                pHatcher.AddElement(gac, orE);
            } // for (; aExp.More() ; aExp.Next()) {
            //
            _hatcherMap[ff] = pHatcher;
        }
        return pHatcher!;
    }

    /// <summary>
    /// The oriented bounding box of the shape (<see cref="Bounds.AddOBB"/>) enlarged by the gap (the fuzzy value,
    /// <see cref="Precision.Confusion"/> by default), computed on the first call. As in OCCT, a box cached with one gap
    /// is returned for any later gap.
    /// </summary>
    public Obb OBB(in Shape aS) => OBB(aS, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="OBB(in Shape)"/>
    public Obb OBB(in Shape aS, double gap)
    {
        if (!_obbMap.TryGetValue(aS, out Obb box))
        {
            box = new Obb();
            Bounds.AddOBB(aS, ref box);
            box.Enlarge(gap);
            _obbMap[aS] = box;
        }
        return box;
    }

    /// <summary>
    /// The surface localization data of the face (3 by 3 samples, minimal ranges 10 PConfusion), made on the first
    /// call: the cached object itself, which IntTools_BeanFaceIntersector fills and reads.
    /// </summary>
    public SurfaceRangeLocalizeData SurfaceData(in Face aF)
    {
        if (!_projSDataMap.TryGetValue(aF, out SurfaceRangeLocalizeData? pSData))
        {
            pSData = new SurfaceRangeLocalizeData(3,
                                                           3,
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
            //
            _projSDataMap[aF] = pSData;
        }
        return pSData!;
    }

    /// <summary>
    /// Kerf (decision 753): the surface localisation data of the face for one set of box parameters (the
    /// intersector's criteria, the face's tolerance and the surface's parameter ranges), made as
    /// <see cref="SurfaceData(in Face)"/> on the first call with these values. The boxes
    /// IntTools_BeanFaceIntersector caches in it are then the boxes it would compute, whichever edge computed them.
    /// </summary>
    internal SurfaceRangeLocalizeData SurfaceData(in Face aF,
                                                           double criteria,
                                                           double faceTolerance,
                                                           double uMin,
                                                           double uMax,
                                                           double vMin,
                                                           double vMax)
    {
        if (!_sDataKeyedMap.TryGetValue(aF, out Dictionary<SurfaceDataKey, SurfaceRangeLocalizeData>? map))
        {
            map = [];
            _sDataKeyedMap[aF] = map;
        }
        SurfaceDataKey key = new(criteria, faceTolerance, uMin, uMax, vMin, vMax);
        if (!map!.TryGetValue(key, out SurfaceRangeLocalizeData? pSData))
        {
            pSData = new SurfaceRangeLocalizeData(3,
                                                           3,
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
            map.Add(key, pSData);
        }
        return pSData;
    }

    // The key of the localisation data: doubles compared by double.Equals, as the options structs of decision 640.
    private readonly record struct SurfaceDataKey(double Criteria, double FaceTolerance, double UMin, double UMax,
                                                  double VMin, double VMax);

    /// <summary>
    /// Computes the parameter of the point on the edge. Returns 0 if the distance is within the sum of the point's
    /// and the edge's tolerances (or the point is near a vertex when the projection fails), -2 if the edge has no
    /// geometry, -3 if the projection fails and no vertex is near, -4 if the point is too far.
    /// </summary>
    public int ComputePE(in Point3D aP1, double aTolP1, in Edge aE2, ref double aT, ref double aDist)
    {
        if (!Tool.IsGeometric(aE2))
        {
            return -2;
        }
        double tolE2, tolSum;
        int nbProj;
        //
        ProjectPointOnCurve projector = ProjPC(aE2);
        projector.Perform(aP1);

        nbProj = projector.NbPoints();
        if (nbProj != 0)
        {
            // point falls on the curve
            aDist = projector.LowerDistance();
            //
            tolE2 = Tool.Tolerance(aE2);
            tolSum = aTolP1 + tolE2 + global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            aT = projector.LowerDistanceParameter();
            if (aDist > tolSum)
            {
                return -4;
            }
        }
        else
        {
            // point falls out of the curve, check distance to vertices
            Edge eFwd = ShapeCasts.Edge(aE2.Oriented(Forward));
            Iterator itV = new(eFwd);
            aDist = RealLast();
            for (; itV.More(); itV.Next())
            {
                Vertex v = ShapeCasts.Vertex(itV.Value());
                if (v.Orientation == Forward || v.Orientation == Reversed)
                {
                    Point3D pv = Tool.Pnt(v);
                    tolSum = aTolP1 + Tool.Tolerance(v) + global::Kerf.Engine.Foundation.Precision.Confusion();
                    double dist1 = aP1.DistanceTo(pv);
                    if (dist1 < aDist && dist1 < tolSum)
                    {
                        aDist = dist1;
                        aT = Tool.Parameter(v, eFwd);
                    }
                }
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aDist))
            {
                return -3;
            }
        }
        return 0;
    }

    /// <summary>
    /// Computes the parameter of the vertex on the edge and the tolerance the vertex would need there. Returns 0 if
    /// the distance is within the sum of the tolerances and the fuzzy value, -1 for a degenerated edge, -2 for an edge
    /// without geometry, -3 if the projection fails, -4 if the vertex is too far.
    /// </summary>
    public int ComputeVE(in Vertex v, in Edge e, ref double t, ref double tol) =>
        ComputeVE(v, e, ref t, ref tol, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVE(in Vertex, in Edge, ref double, ref double)"/>
    public int ComputeVE(in Vertex v, in Edge e, ref double t, ref double tol, double fuzz)
    {
        if (Tool.Degenerated(e))
        {
            return -1;
        }
        if (!Tool.IsGeometric(e))
        {
            return -2;
        }
        double dist, tolV, tolE, tolSum;
        int nbProj;
        Point3D p;
        //
        p = Tool.Pnt(v);
        //
        ProjectPointOnCurve projector = ProjPC(e);
        projector.Perform(p);

        nbProj = projector.NbPoints();
        if (nbProj == 0)
        {
            return -3;
        }
        //
        dist = projector.LowerDistance();
        //
        tolV = Tool.Tolerance(v);
        tolE = Tool.Tolerance(e);
        tolSum = tolV + tolE + Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        tol = dist + tolE;
        t = projector.LowerDistanceParameter();
        if (dist > tolSum)
        {
            return -4;
        }
        return 0;
    }

    /// <summary>
    /// Computes the UV parameters of the vertex on the face and the tolerance the vertex would need there. Returns 0
    /// if the distance is within the sum of the tolerances and the fuzzy value and the projection is inside the face,
    /// -1 if the projection fails, -2 if the vertex is too far, -3 if the projection is out of or on the face's
    /// boundary.
    /// </summary>
    public int ComputeVF(in Vertex vertex, in Face face, ref double u, ref double v, ref double tol) =>
        ComputeVF(vertex, face, ref u, ref v, ref tol, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVF(in Vertex, in Face, ref double, ref double, ref double)"/>
    public int ComputeVF(in Vertex vertex, in Face face, ref double u, ref double v, ref double tol,
                         double fuzz)
    {
        double tolV, tolF, tolSum, dist;
        Point3D p;

        p = Tool.Pnt(vertex);
        //
        // 1. Check if the point is projectable on the surface
        ProjectPointOnSurface projector = ProjPS(face);
        projector.Perform(p);
        //
        if (!projector.IsDone)
        { // the point is not  projectable on the surface
            return -1;
        }
        //
        // 2. Check the distance between the projection point and
        //    the original point
        dist = projector.LowerDistance();
        //
        tolV = Tool.Tolerance(vertex);
        tolF = Tool.Tolerance(face);
        //
        tolSum = tolV + tolF + Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());
        tol = dist + tolF;
        projector.LowerDistanceParameters(out u, out v);
        //
        if (dist > tolSum)
        {
            // the distance is too large
            return -2;
        }
        //
        Point2D p2d = new(u, v);
        bool pri = IsPointInFace(face, p2d);
        if (!pri)
        { //  the point lays on the surface but out of the face
            return -3;
        }
        return 0;
    }

    /// <summary>The state of the 2D point relative to the face (<see cref="FaceClassifier2D.Perform"/>).</summary>
    public Containment StatePointFace(in Face aF, in Point2D aP2d)
    {
        Containment state;
        FaceClassifier2D class2d = FClass2d(aF);
        state = class2d.Perform(aP2d);
        return state;
    }

    /// <summary>True if the 2D point is inside the face's boundaries (not OUT, not ON).</summary>
    public bool IsPointInFace(in Face aF, in Point2D aP2d)
    {
        Containment state = StatePointFace(aF, aP2d);
        return state != Out && state != On;
    }

    /// <summary>True if the point projects on the face's surface closer than the tolerance, inside the face.</summary>
    public bool IsPointInFace(in Point3D aP, in Face aF, double aTol)
    {
        bool bIn = false;
        double dist;
        //
        ProjectPointOnSurface projector = ProjPS(aF);
        projector.Perform(aP);
        //
        bool bDone = projector.IsDone;
        if (bDone)
        {
            dist = projector.LowerDistance();
            if (dist < aTol)
            {
                projector.LowerDistanceParameters(out double u, out double v);
                Point2D p2D = new(u, v);
                bIn = IsPointInFace(aF, p2D);
            }
        }
        //
        return bIn;
    }

    /// <summary>True if the 2D point is inside or on the face's boundaries.</summary>
    public bool IsPointInOnFace(in Face aF, in Point2D aP2d)
    {
        Containment state = StatePointFace(aF, aP2d);
        return state != Out;
    }

    /// <summary>
    /// True if the point projects on the face's surface within the tolerance, inside or on the face's boundaries.
    /// </summary>
    public bool IsValidPointForFace(in Point3D aP, in Face aF, double aTol)
    {
        bool bFlag;
        double umin;

        ProjectPointOnSurface projector = ProjPS(aF);
        projector.Perform(aP);

        bFlag = projector.IsDone;
        if (bFlag)
        {

            umin = projector.LowerDistance();
            // if (Umin > 1.e-3) { // it was
            if (umin > aTol)
            {
                return !bFlag;
            }
            //
            projector.LowerDistanceParameters(out double u, out double v);
            Point2D p2D = new(u, v);
            bFlag = IsPointInOnFace(aF, p2D);
        }
        return bFlag;
    }

    /// <summary>True if <see cref="IsValidPointForFace"/> holds for both faces.</summary>
    public bool IsValidPointForFaces(in Point3D aP, in Face aF1, in Face aF2, double aTol)
    {
        bool bFlag1, bFlag2;

        bFlag1 = IsValidPointForFace(aP, aF1, aTol);
        if (!bFlag1)
        {
            return bFlag1;
        }
        bFlag2 = IsValidPointForFace(aP, aF2, aTol);
        return bFlag2;
    }

    /// <summary>True if <see cref="IsValidPointForFace"/> holds for the curve's intermediate point between the parameters.</summary>
    public bool IsValidBlockForFace(double aT1, double aT2, in IntersectionCurve aC, in Face aF, double aTol)
    {
        bool bFlag;
        double tInterm;
        Point3D pInterm;

        tInterm = IntersectionTools.IntermediatePoint(aT1, aT2);

        Curve3D c3D = aC.Curve()!;
        // point 3D
        c3D.D0(tInterm, out pInterm);
        //
        bFlag = IsValidPointForFace(pInterm, aF, aTol);
        return bFlag;
    }

    /// <summary>
    /// True if the curve's intermediate point between the parameters is valid for both faces: through the pcurve where
    /// the curve has one for the face, through projection otherwise.
    /// </summary>
    public bool IsValidBlockForFaces(double t1, double t2, in IntersectionCurve c, in Face f1, in Face f2,
                                     double tol)
    {
        const int nbElem = 2;
        Curve2D? pc1 = c.FirstCurve2d();
        Curve2D? pc2 = c.SecondCurve2d();
        Curve3D c3D = c.Curve()!;

        Curve2D?[] arrPC = [pc1, pc2];
        Face[] arrF = [f1, f2];

        double midPar = IntersectionTools.IntermediatePoint(t1, t2);
        Point3D p = c3D.Value(midPar);

        bool bFlag = true;
        Point2D pnt2D;

        for (int i = 0; (i < nbElem) && bFlag; ++i)
        {
            Curve2D? pc = arrPC[i];
            Face f = arrF[i];

            if (pc is not null)
            {
                pc.D0(midPar, out pnt2D);
                bFlag = IsPointInOnFace(f, pnt2D);
            }
            else
            {
                bFlag = IsValidPointForFace(p, f, tol);
            }
        }

        return bFlag;
    }

    /// <summary>
    /// Computes the parameter of the vertex on the curve. True if the distance is within the sum of the vertex's
    /// tolerance and <paramref name="aTolC"/> (doubled, at least 1e-5 on B-spline and Bézier curves, 1e-6 on others);
    /// otherwise, or if the projection fails, false (and <paramref name="aT"/> is meaningless).
    /// </summary>
    public bool IsVertexOnLine(in Vertex aV, in IntersectionCurve aC, double aTolC, ref double aT)
    {
        bool bRet;
        double tolV;
        //
        tolV = Tool.Tolerance(aV);
        bRet = IsVertexOnLine(aV, tolV, aC, aTolC, ref aT);
        //
        return bRet;
    }

    /// <inheritdoc cref="IsVertexOnLine(in Vertex, in IntersectionCurve, double, ref double)"/>
    public bool IsVertexOnLine(in Vertex aV, double aTolV, in IntersectionCurve aC, double aTolC, ref double aT)
    {
        double first, last, dist, tolSum;
        int nbProj;
        Point3D pv;

        pv = Tool.Pnt(aV);

        Curve3D c3D = aC.Curve()!;

        tolSum = aTolV + aTolC;
        //
        CurveAdaptor gac = new(c3D);
        CurveKind type = gac.GetType();
        if (type == BSplineCurve || type == BezierCurve)
        {
            tolSum = 2.0 * tolSum;
            if (tolSum < 1.0e-5)
            {
                tolSum = 1.0e-5;
            }
        }
        else
        {
            tolSum = 2.0 * tolSum; // xft
            if (tolSum < 1.0e-6)
            {
                tolSum = 1.0e-6;
            }
        }
        //
        first = c3D.FirstParameter;
        last = c3D.LastParameter;
        //
        // Checking extremities first
        // It is necessary to chose the closest bound to the point
        bool bFirstValid = false;
        double firstDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        //
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(first))
        {
            Point3D pcFirst = c3D.Value(first);
            firstDist = pv.DistanceTo(pcFirst);
            if (firstDist < tolSum)
            {
                bFirstValid = true;
                aT = first;
                //
                if (firstDist > aTolV)
                {
                    LocateExtPC ext = new(pv, gac, first, 1.0e-10);

                    if (ext.IsDone)
                    {
                        POnCurv pOncurve = ext.Point();
                        aT = pOncurve.Parameter();

                        if ((aT > (last + first) * 0.5) || (pv.DistanceTo(pOncurve.Value()) > tolSum)
                            || (pcFirst.DistanceTo(pOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            aT = first;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        CurvePointExtrema ext2 = new(pv, gac, 1.0e-10);
                        double minDist = RealLast();
                        int minIdx = -1;
                        if (ext2.IsDone)
                        {
                            for (int idx = 1; idx <= ext2.ExtCount; idx++)
                            {
                                if (ext2.IsMin(idx) && ext2.SquareDistance(idx) < minDist)
                                {
                                    minDist = ext2.SquareDistance(idx);
                                    minIdx = idx;
                                }
                            }
                        }
                        if (minIdx != -1)
                        {
                            POnCurv pOncurve = ext2.Point(minIdx);
                            aT = pOncurve.Parameter();

                            if ((aT > (last + first) * 0.5) || (pv.DistanceTo(pOncurve.Value()) > tolSum)
                                || (pcFirst.DistanceTo(pOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                aT = first;
                            }
                        }
                    }
                }
            }
        }
        //
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(last))
        {
            Point3D pcLast = c3D.Value(last);
            dist = pv.DistanceTo(pcLast);
            if (bFirstValid && (firstDist < dist))
            {
                return true;
            }
            //
            if (dist < tolSum)
            {
                aT = last;
                //
                if (dist > aTolV)
                {
                    LocateExtPC ext = new(pv, gac, last, 1.0e-10);

                    if (ext.IsDone)
                    {
                        POnCurv pOncurve = ext.Point();
                        aT = pOncurve.Parameter();

                        if ((aT < (last + first) * 0.5) || (pv.DistanceTo(pOncurve.Value()) > tolSum)
                            || (pcLast.DistanceTo(pOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            aT = last;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        CurvePointExtrema ext2 = new(pv, gac, 1.0e-10);
                        double minDist = RealLast();
                        int minIdx = -1;
                        if (ext2.IsDone)
                        {
                            for (int idx = 1; idx <= ext2.ExtCount; idx++)
                            {
                                if (ext2.IsMin(idx) && ext2.SquareDistance(idx) < minDist)
                                {
                                    minDist = ext2.SquareDistance(idx);
                                    minIdx = idx;
                                }
                            }
                        }
                        if (minIdx != -1)
                        {
                            POnCurv pOncurve = ext2.Point(minIdx);
                            aT = pOncurve.Parameter();

                            if ((aT < (last + first) * 0.5) || (pv.DistanceTo(pOncurve.Value()) > tolSum)
                                || (pcLast.DistanceTo(pOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                aT = last;
                            }
                        }
                    }
                }
                //
                return true;
            }
        }
        else if (bFirstValid)
        {
            return true;
        }
        //
        ProjectPointOnCurve projector = ProjPT(c3D);
        projector.Perform(pv);

        nbProj = projector.NbPoints();
        if (nbProj == 0)
        {
            if (c3D is BoundedCurve3D bc)
            {
                Point3D pStart = bc.StartPoint;
                Point3D pEnd = bc.EndPoint;

                dist = pv.DistanceTo(pStart);
                if (dist < tolSum)
                {
                    aT = first;
                    return true;
                }

                dist = pv.DistanceTo(pEnd);
                if (dist < tolSum)
                {
                    aT = last;
                    return true;
                }
            }

            return false;
        }

        dist = projector.LowerDistance();

        if (dist > tolSum)
        {
            return false;
        }

        aT = projector.LowerDistanceParameter();

        return true;
    }

    /// <summary>Computes the parameter of the point's projection on the edge; false if the projection fails.</summary>
    public bool ProjectPointOnEdge(in Point3D aP, in Edge anEdge, ref double aT)
    {
        int nbPoints;

        ProjectPointOnCurve projector = ProjPC(anEdge);
        projector.Perform(aP);

        nbPoints = projector.NbPoints();
        if (nbPoints != 0)
        {
            aT = projector.LowerDistanceParameter();
            return true;
        }
        return false;
    }

    /// <summary>
    /// Sets the tolerance used to project points on surfaces (1e-12 by default), and clears the cached surface
    /// projectors so that every later one uses it.
    /// </summary>
    public void SetPOnSProjectionTolerance(double value)
    {
        _pOnSTolerance = value;
        ClearCachedPOnSProjectors();
    }

    /// <summary>Clears the map of cached surface projectors.</summary>
    private void ClearCachedPOnSProjectors() => _projPSMap.Clear();

    /// <summary>The UV bounds of the face, from its cached surface adaptor.</summary>
    public void UVBounds(in Face face, out double uMin, out double uMax, out double vMin, out double vMax)
    {
        FaceSurfaceAdaptor bas = SurfaceAdaptor(face);
        uMin = bas.FirstUParameter();
        uMax = bas.LastUParameter();
        vMin = bas.FirstVParameter();
        vMax = bas.LastVParameter();
    }

    // myCreateFlag tells OCCT whether the context was given its allocator; the allocator goes, the flag stays.
    internal int CreateFlag() => _createFlag;

    internal double POnSProjectionTolerance() => _pOnSTolerance;
}
