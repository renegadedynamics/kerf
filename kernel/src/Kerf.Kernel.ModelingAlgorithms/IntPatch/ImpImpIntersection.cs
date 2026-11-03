// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The intersection of two implicit surfaces (planes, cylinders, cones, spheres and tori) bounded by their domains:
/// lines (conics, analytic curves, restrictions or, for two cylinders without a geometric solution, walking lines)
/// and isolated points.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's class. OCCT's .cxx is split into partial files by subject: this one
/// (the class and Perform), Boundaries and Restrictions (the points and segments of the domains put on the lines),
/// Plane (plane/quadric pairs), CyCy (cylinder/cylinder, with its walking lines) and Quadrics (the other pairs and
/// the tori). <see cref="Point"/> returns a copy (OCCT's const reference).
/// </remarks>
public sealed partial class ImpImpIntersection
{
    /// <summary>The status of an intersection.</summary>
    public enum IntStatus
    {
        /// <summary>OK. Good intersection result.</summary>
        IntStatus_OK,

        /// <summary>
        /// The intersection curve is too long (as in OCCT's bug 26894): its value and derivatives can't be computed
        /// precisely enough in IEEE 754 doubles, so OCCT's algorithms can't work with it correctly.
        /// </summary>
        IntStatus_InfiniteSectionCurve,

        /// <summary>The algorithm cannot finish correctly.</summary>
        IntStatus_Fail,
    }

    private IntStatus _done;
    private bool _empt;
    private bool _tgte;
    private bool _oppo;
    private readonly NCollection_Sequence<PatchPoint> _spnt = new();
    private readonly NCollection_Sequence<PatchLine> _slin = new();
    private readonly TheSOnBounds _solrst = new();

    /// <summary>An empty intersection (not done).</summary>
    public ImpImpIntersection()
    {
        _done = IntStatus.IntStatus_Fail;
        _empt = true;
        _tgte = false;
        _oppo = false;
    }

    /// <summary>
    /// Intersects <paramref name="s1"/> bounded by <paramref name="d1"/> with <paramref name="s2"/> bounded by
    /// <paramref name="d2"/>. With <paramref name="isReqToKeepRLine"/> (kept for TopOpeBRep), a restriction line
    /// is kept beside another line even where they coincide.
    /// </summary>
    public ImpImpIntersection(SurfaceAdaptorBase s1,
                                       TopolTool d1,
                                       SurfaceAdaptorBase s2,
                                       TopolTool d2,
                                       double tolArc,
                                       double tolTang,
                                       bool isReqToKeepRLine = false)
    {
        Perform(s1, d1, s2, d2, tolArc, tolTang, isReqToKeepRLine);
    }

    /// <summary>
    /// Intersects <paramref name="s1"/> bounded by <paramref name="d1"/> with <paramref name="s2"/> bounded by
    /// <paramref name="d2"/>; see the constructor.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A surface is not a plane, cylinder, cone, sphere or torus.</exception>
    public void Perform(SurfaceAdaptorBase s1,
                        TopolTool d1,
                        SurfaceAdaptorBase s2,
                        TopolTool d2,
                        double tolArc,
                        double tolTang,
                        bool isReqToKeepRLine = false)
    {
        _done = IntStatus.IntStatus_Fail;
        _spnt.Clear();
        _slin.Clear();

        bool isPostProcessingRequired = true;

        _empt = true;
        _tgte = false;
        _oppo = false;

        bool all1 = false;
        bool all2 = false;
        bool sameSurf = false;
        bool multpoint = false;

        bool nosolonS1 = false;
        // indique s il y a des points sur restriction du carreau 1
        bool nosolonS2 = false;
        // indique s il y a des points sur restriction du carreau 2
        int i, nbpt, nbseg;
        NCollection_Sequence<TheSegmentOfTheSOnBounds> edg1 = new(), edg2 = new();
        NCollection_Sequence<ThePathPointOfTheSOnBounds> pnt1 = new(), pnt2 = new();
        //
        // On commence par intersecter les supports des surfaces
        SurfaceQuadric quad1 = new(), quad2 = new();
        PatchArcFunction aFunc = new();
        const double tolang = 1.0e-8;
        SurfaceKind typs1, typs2;
        bool bEmpty = false;
        //
        int iT1 = SetQuad(s1, out typs1, ref quad1);
        int iT2 = SetQuad(s2, out typs2, ref quad2);
        //
        if (iT1 == 0 || iT2 == 0)
        {
            throw new ConstructionErrorException();
        }
        //
        bool bReverse = iT1 > iT2;
        int iTT = iT1 * 10 + iT2;
        //
        switch (iTT)
        {
            case 11:
                { // Plane/Plane
                    if (!IntPP(quad1, quad2, tolang, tolTang, ref sameSurf, _slin))
                    {
                        return;
                    }
                    break;
                }
            //
            case 12:
            case 21:
                { // Plane/Cylinder
                    double vMin, vMax, h;
                    //
                    SurfaceAdaptorBase sCyl = bReverse ? s1 : s2;
                    vMin = sCyl.FirstVParameter();
                    vMax = sCyl.LastVParameter();
                    h = (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
                          ? 0
                          : (vMax - vMin);
                    //
                    if (!IntPCy(quad1, quad2, tolang, tolTang, bReverse, ref _empt, _slin, h))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 13:
            case 31:
                { // Plane/Cone
                    if (!IntPCo(quad1, quad2, tolang, tolTang, bReverse, ref _empt, ref multpoint, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 14:
            case 41:
                { // Plane/Sphere
                    if (!IntPSp(quad1, quad2, tolang, tolTang, bReverse, ref _empt, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 15:
            case 51:
                { // Plane/Torus
                    if (!IntPTo(quad1, quad2, tolTang, bReverse, ref _empt, _slin))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 22:
                { // Cylinder/Cylinder
                    Box2d box1 = new(), box2 = new();

                    double u1f = s1.FirstUParameter();
                    double u1l = s1.LastUParameter();
                    double u2f = s2.FirstUParameter();
                    double u2l = s2.LastUParameter();

                    const double uperiod = 2.0 * M_PI;

                    if (u1l - u1f > uperiod)
                    {
                        u1l = u1f + uperiod;
                    }

                    if (u2l - u2f > uperiod)
                    {
                        u2l = u2f + uperiod;
                    }

                    box1.Add(new Point2D(u1f, s1.FirstVParameter()));
                    box1.Add(new Point2D(u1l, s1.LastVParameter()));
                    box2.Add(new Point2D(u2f, s2.FirstVParameter()));
                    box2.Add(new Point2D(u2l, s2.LastVParameter()));

                    // Resolution is too big if the cylinder radius is
                    // too small. Therefore, we shall bind its value above.
                    // Here, we use simple constant.
                    double a2DTol = Min(1.0e-4, Min(s1.UResolution(tolTang), s2.UResolution(tolTang)));

                    _done = IntCyCy(quad1, quad2, tolTang, a2DTol, box1, box2, ref _empt, ref sameSurf,
                                     ref multpoint, _slin, _spnt);

                    if (_done == IntStatus.IntStatus_Fail)
                    {
                        return;
                    }

                    bEmpty = _empt;
                    if (!_slin.IsEmpty())
                    {
                        if (_slin.Value(1) is WLine)
                        { // No geometric solution
                            isPostProcessingRequired = false;
                        }
                    }

                    break;
                }
            //
            case 23:
            case 32:
                { // Cylinder/Cone
                    if (!IntCyCo(quad1, quad2, tolTang, bReverse, ref _empt, ref multpoint, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 24:
            case 42:
                { // Cylinder/Sphere
                    if (!IntCySp(quad1, quad2, tolTang, bReverse, ref _empt, ref multpoint, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 25:
            case 52:
                { // Cylinder/Torus
                    if (!IntCyTo(quad1, quad2, tolTang, bReverse, ref _empt, _slin))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 33:
                { // Cone/Cone
                    if (!IntCoCo(quad1, quad2, tolTang, ref _empt, ref sameSurf, ref multpoint, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 34:
            case 43:
                { // Cone/Sphere
                    if (!IntCoSp(quad1, quad2, tolTang, bReverse, ref _empt, ref multpoint, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 35:
            case 53:
                { // Cone/Torus
                    if (!IntCoTo(quad1, quad2, tolTang, bReverse, ref _empt, _slin))
                    {
                        return;
                    }
                    // OCCT: bEmpty is not set from empt here, as it is for every other pair, so an empty cone/torus
                    // intersection goes on to the boundary processing.
                    break;
                }
            //
            case 44:
                { // Sphere/Sphere
                    if (!IntSpSp(quad1, quad2, tolTang, ref _empt, ref sameSurf, _slin, _spnt))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 45:
            case 54:
                { // Sphere/Torus
                    if (!IntSpTo(quad1, quad2, tolTang, bReverse, ref _empt, _slin))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            case 55:
                { // Torus/Torus
                    if (!IntToTo(quad1, quad2, tolTang, ref sameSurf, ref _empt, _slin))
                    {
                        return;
                    }
                    bEmpty = _empt;
                    break;
                }
            //
            default:
                {
                    throw new ConstructionErrorException();
                }
        }
        //
        if (bEmpty)
        {
            if (_done == IntStatus.IntStatus_Fail)
            {
                _done = IntStatus.IntStatus_OK;
            }

            return;
        }
        //

        if (isPostProcessingRequired)
        {
            if (!sameSurf)
            {
                aFunc.SetQuadric(quad2);
                aFunc.Set(s1);

                _solrst.Perform(aFunc, d1, tolArc, tolTang);
                if (!_solrst.IsDone)
                {
                    return;
                }

                if (_solrst.AllArcSolution() && typs1 == typs2)
                {
                    all1 = true;
                }
                nbpt = _solrst.PointCount;
                nbseg = _solrst.SegmentCount;
                for (i = 1; i <= nbpt; i++)
                {
                    pnt1.Append(_solrst.Point(i));
                }
                for (i = 1; i <= nbseg; i++)
                {
                    edg1.Append(_solrst.Segment(i));
                }
                nosolonS1 = (nbpt == 0) && (nbseg == 0);

                if (nosolonS1 && all1)
                { // cas de face sans restrictions
                    all1 = false;
                }
            } // if (!SameSurf) {
            else
            {
                nosolonS1 = true;
            }

            if (!sameSurf)
            {
                aFunc.SetQuadric(quad1);
                aFunc.Set(s2);

                _solrst.Perform(aFunc, d2, tolArc, tolTang);
                if (!_solrst.IsDone)
                {
                    return;
                }

                if (_solrst.AllArcSolution() && typs1 == typs2)
                {
                    all2 = true;
                }

                nbpt = _solrst.PointCount;
                nbseg = _solrst.SegmentCount;
                for (i = 1; i <= nbpt; i++)
                {
                    pnt2.Append(_solrst.Point(i));
                }

                for (i = 1; i <= nbseg; i++)
                {
                    edg2.Append(_solrst.Segment(i));
                }

                nosolonS2 = (nbpt == 0) && (nbseg == 0);

                if (nosolonS2 && all2)
                { // cas de face sans restrictions
                    all2 = false;
                }
            } // if (!SameSurf) {
            else
            {
                nosolonS2 = true;
            }
            //
            if (sameSurf || (all1 && all2))
            {
                // faces "paralleles" parfaites
                _empt = false;
                _tgte = true;
                _slin.Clear();
                _spnt.Clear();

                Point3D ptreference = new();

                switch (typs1)
                {
                    case Plane:
                        {
                            ptreference = (s1.Plane()).Location;
                        }
                        break;
                    case SurfaceKind.Cylinder:
                        {
                            ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 0.0, s1.Cylinder());
                        }
                        break;
                    case SurfaceKind.Sphere:
                        {
                            ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(M_PI / 4.0, M_PI / 4.0, s1.Sphere());
                        }
                        break;
                    case SurfaceKind.Cone:
                        {
                            ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 10.0, s1.Cone());
                        }
                        break;
                    case SurfaceKind.Torus:
                        {
                            ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 0.0, s1.Torus());
                        }
                        break;
                    default:
                        break;
                }
                //
                _oppo = quad1.Normale(ptreference).Dot(quad2.Normale(ptreference)) < 0.0;
                _done = IntStatus.IntStatus_OK;
                return;
            } // if (SameSurf || (all1 && all2)) {

            if (!nosolonS1 || !nosolonS2)
            {
                _empt = false;
                // C est la qu il faut commencer a bosser...
                PutPointsOnLine(s1, s2, pnt1, _slin, true, d1, quad1, quad2, multpoint, tolArc);

                PutPointsOnLine(s1, s2, pnt2, _slin, false, d2, quad2, quad1, multpoint, tolArc);

                if (edg1.Length() != 0)
                {
                    ProcessSegments(edg1, _slin, quad1, quad2, true, tolArc);
                }

                if (edg2.Length() != 0)
                {
                    ProcessSegments(edg2, _slin, quad1, quad2, false, tolArc);
                }

                if (edg1.Length() != 0 || edg2.Length() != 0)
                {
                    ProcessRLine(_slin, quad1, quad2, tolArc, isReqToKeepRLine);
                }
            } // if (!nosolonS1 || !nosolonS2) {
            else
            {
                _empt = ((_slin.Length() == 0) && (_spnt.Length() == 0));
            }
        }

        int nblin = _slin.Length(), nbPnt = _spnt.Length();
        //
        // modified by NIZNHY-PKV Tue Sep 06 10:03:35 2011f
        if (nbPnt != 0)
        {
            NCollection_Sequence<PatchPoint> sip = new();
            //
            for (i = 1; i <= nbPnt; ++i)
            {
                Point2D p2D;
                Containment state1, state2;
                //
                PatchPoint ip = _spnt[i];
                ip.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
                //
                p2D = new Point2D(aU1, aV1);
                state1 = d1.Classify(p2D, tolArc);
                //
                p2D = new Point2D(aU2, aV2);
                state2 = d2.Classify(p2D, tolArc);
                //
                if (state1 != Out && state2 != Out)
                {
                    sip.Append(ip);
                }
            }
            //
            _spnt.Clear();
            //
            nbPnt = sip.Length();
            for (i = 1; i <= nbPnt; ++i)
            {
                _spnt.Append(sip[i]);
            }
            //
        } //  if (aNbPnt) {
        // modified by NIZNHY-PKV Tue Sep 06 10:18:20 2011t
        //
        for (i = 1; i <= nblin; i++)
        {
            PatchLineType thetype = _slin.Value(i).ArcType();
            if ((thetype == PatchLineType.Ellipse) || (thetype == PatchLineType.Circle) || (thetype == PatchLineType.Lin)
                || (thetype == PatchLineType.Parabola) || (thetype == PatchLineType.Hyperbola))
            {
                ((GLine)_slin.Value(i)).ComputeVertexParameters(tolArc);
            }
            else if (thetype == PatchLineType.Analytic)
            {
                ((ALine)_slin.Value(i)).ComputeVertexParameters(tolArc);
            }
            else if (thetype == PatchLineType.Restriction)
            {
                ((RLine)_slin.Value(i)).ComputeVertexParameters(tolArc);
            }
        }
        //
        //----------------------------------------------------------------
        //-- On place 2 vertex sur les courbes de GLine qui n en
        //-- contiennent pas.
        for (i = 1; i <= nblin; i++)
        {
            Point3D p;
            PatchPoint point = new();
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            if (_slin.Value(i).ArcType() == PatchLineType.Circle)
            {
                GLine glin = (GLine)_slin.Value(i);
                if (glin.VertexCount == 0)
                {
                    Circle3D circ = glin.Circle();
                    p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ);
                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);
                    point.SetValue(p, tolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(0.0);
                    glin.AddVertex(point);

                    p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ);
                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);
                    point.SetValue(p, tolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(M_PI + M_PI);
                    glin.AddVertex(point);
                }
            }

            else if (_slin.Value(i).ArcType() == PatchLineType.Ellipse)
            {
                GLine glin = (GLine)_slin.Value(i);
                if (glin.VertexCount == 0)
                {
                    Ellipse3D elips = glin.Ellipse();
                    p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elips);
                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);
                    point.SetValue(p, tolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(0.0);
                    glin.AddVertex(point);

                    p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elips);
                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);
                    point.SetValue(p, tolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(M_PI + M_PI);
                    glin.AddVertex(point);
                }
            }
        }
        _done = IntStatus.IntStatus_OK;
    }

    /// <summary>Returns true if the computation was successful.</summary>
    public bool IsDone => (GetStatus() != IntStatus.IntStatus_Fail);

    /// <summary>Returns the status.</summary>
    public IntStatus GetStatus() => _done;

    /// <summary>Returns true if there is no intersection.</summary>
    public bool IsEmpty
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _empt;
        }
    }

    /// <summary>
    /// Returns true if the two patches are entirely tangent: every restriction arc of one patch is inside the
    /// geometric base of the other.
    /// </summary>
    public bool TangentFaces()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _tgte;
    }

    /// <summary>
    /// Returns true when <see cref="TangentFaces"/> is true and the normals at a point of the two surfaces are
    /// opposite.
    /// </summary>
    /// <exception cref="DomainErrorException"><see cref="TangentFaces"/> is false.</exception>
    public bool OppositeFaces()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        if (!_tgte)
        {
            throw new DomainErrorException();
        }
        return _oppo;
    }

    /// <summary>Returns the number of "single" points.</summary>
    public int PntCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _spnt.Length();
        }
    }

    /// <summary>Returns the point of range <paramref name="index"/>.</summary>
    public PatchPoint Point(int index)
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _spnt.Value(index);
    }

    /// <summary>Returns the number of intersection lines.</summary>
    public int LineCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _slin.Length();
        }
    }

    /// <summary>Returns the line of range <paramref name="index"/>.</summary>
    public PatchLine Line(int index)
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _slin.Value(index);
    }

    //=================================================================================================

    private static int SetQuad(SurfaceAdaptorBase s, out SurfaceKind ts, ref SurfaceQuadric quad)
    {
        ts = s.GetType();
        int iRet = 0;
        switch (ts)
        {
            case Plane:
                quad.SetValue(s.Plane());
                iRet = 1;
                break;
            case SurfaceKind.Cylinder:
                quad.SetValue(s.Cylinder());
                iRet = 2;
                break;
            case SurfaceKind.Cone:
                quad.SetValue(s.Cone());
                iRet = 3;
                break;
            case SurfaceKind.Sphere:
                quad.SetValue(s.Sphere());
                iRet = 4;
                break;
            case SurfaceKind.Torus:
                quad.SetValue(s.Torus());
                iRet = 5;
                break;
            default:
                break;
        }
        //
        return iRet;
    }
}
