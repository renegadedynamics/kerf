// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidExplorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>
/// The exploration of a shape for the classification of points in it: the lines from the point towards its faces,
/// the shells and their faces with an intersector each, and a tree of the boxes of its edges and vertices.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape. Its arguments that OCCT may leave unwritten are
/// <c>ref</c> (decision 57): the point, parameters and derivatives of the FindAPointInTheFace and PointInTheFace
/// overloads, and the line and parameter of <see cref="Segment"/> and <see cref="OtherSegment"/>.
/// </remarks>
internal class SolidExplorer
{
    private Shape _shape = new();
    private bool _reject;
    private int _firstFace;
    private double _paramOnEdge;
    private Explorer _shellExplorer = new();
    private Explorer _faceExplorer = new();
    private readonly Dictionary<Shape, CurveSurfaceIntersector?> _mapOfInter =
        new(ShapeHasher.Instance);
    private readonly SpatialTree<int, Box> _tree = new();
    private readonly IndexedSet<Shape> _mapEV = new(ShapeHasher.Instance);
    // Kerf: whether the edges and vertices and their tree are filled; see InitShape
    private bool _treeFilled;
    // Kerf: the faces of each edge of a frozen shape, which every point classified asks for (unit 12Y); see GetMapEF
    private IndexedMap<Shape, NCollection_List<Shape>>? _mapEF;
    // Kerf: the faces whose intersectors OtherSegment replaced by unrestricted ones, with the one each had (unit 15C2)
    private Dictionary<Shape, CurveSurfaceIntersector?>? _restricted;
    // Kerf (unit 15E17): the shape's box, and whether it bounds a finite region; see IsOutsideFiniteBounds
    private Box? _bounds;
    private bool? _finite;
    // Kerf (unit 15E17): for a shape placed with a scale, an explorer of it without its placement; see Unscaled
    private SolidExplorer? _unscaled;
    private bool _unscaledAsked;

    /// <summary>
    /// A point <paramref name="APoint_"/> in the face <paramref name="_face"/>: <paramref name="param_"/>, in ]0, 1[,
    /// initialises the search; different values give different points.
    /// </summary>
    public static bool FindAPointInTheFace(in Face _face, ref Point3D APoint_, ref double param_)
    {
        double u = 0.0, v = 0.0;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u, ref v, ref param_);
        return r;
    }

    /// <summary>A point <paramref name="APoint_"/> in the face, with its parameters (see the other overloads).</summary>
    public static bool FindAPointInTheFace(in Face _face,
                                           ref Point3D APoint_,
                                           ref double u_,
                                           ref double v_,
                                           ref double param_)
    {
        Vector3D vecD1U = new(), vecD1V = new();
        return FindAPointInTheFace(_face, ref APoint_, ref u_, ref v_, ref param_, ref vecD1U, ref vecD1V);
    }

    /// <summary>
    /// A point <paramref name="APoint_"/> in the face, with its parameters and the surface's first derivatives there:
    /// a point a little inside an edge, moved towards the nearest other edge, where the surface isn't singular. The
    /// outputs are unchanged when none is found.
    /// </summary>
    public static bool FindAPointInTheFace(in Face _face,
                                           ref Point3D APoint_,
                                           ref double u_,
                                           ref double v_,
                                           ref double param_,
                                           ref Vector3D vecD1U,
                                           ref Vector3D vecD1V)
    {
        Face face = _face;
        face.Orient(Forward);

        Vector2D t;
        Point2D p;

        for (var faceexplorer = new Explorer(face, ShapeType.Edge); faceexplorer.More(); faceexplorer.Next())
        {
            Edge edge = ShapeCasts.Edge(faceexplorer.Current());
            var c = new EdgeCurve2DAdaptor(edge, face);
            if (c.Curve() is null)
            {
                continue;
            }

            c.D1((c.LastParameter() - c.FirstParameter()) * param_ + c.FirstParameter(), out p, out t);

            double x = t.X;
            double y = t.Y;
            if (edge.Orientation == Forward)
            {
                t = t.WithCoord(-y, x);
            }
            else
            {
                t = t.WithCoord(y, -x);
            }

            double paramInit = global::Kerf.Engine.Foundation.Precision.Infinite();
            double tolInit = 0.00001;
            bool aPointExist = false;

            var fClassifier = new FacePassiveClassifier();

            t = t.Normalized();
            p = p.WithCoord(p.X + tolInit * t.X, p.Y + tolInit * t.Y);
            fClassifier.Reset(new Line2D(p, new Direction2D(t)), paramInit, RealEpsilon()); //-- Length and Tolerance #######

            var otherfaceexplorer = new Explorer();
            int nbEdges = 0;
            for (otherfaceexplorer.Init(face, global::Kerf.Topology.ShapeType.Edge); otherfaceexplorer.More();
                 otherfaceexplorer.Next(), ++nbEdges)
            {
                Edge otherEdge = ShapeCasts.Edge(otherfaceexplorer.Current());
                if (otherEdge.Orientation != External && otherEdge != edge)
                {
                    var aEdge = new ClassifierEdge(otherEdge, face);
                    fClassifier.Compare(aEdge, otherEdge.Orientation);
                    if (fClassifier.ClosestIntersection() != 0)
                    {
                        if (paramInit > fClassifier.Parameter())
                        {
                            paramInit = fClassifier.Parameter();
                            aPointExist = true;
                        }
                    }
                }
            }

            if (nbEdges == 1)
            {
                var aEdge = new ClassifierEdge(edge, face);
                fClassifier.Compare(aEdge, edge.Orientation);
                if (fClassifier.ClosestIntersection() != 0)
                {
                    if (paramInit > fClassifier.Parameter())
                    {
                        paramInit = fClassifier.Parameter();
                        aPointExist = true;
                    }
                }
            }

            while (aPointExist)
            {
                paramInit *= 0.41234;
                u_ = p.X + paramInit * t.X;
                v_ = p.Y + paramInit * t.Y;

                // Additional check
                var classifier = new FClass2d(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                Point2D pnt2d = new(u_, v_);
                Containment stateOfResultingPoint = classifier.Perform(pnt2d);
                if (stateOfResultingPoint != In)
                {
                    return false;
                }

                var s = new FaceSurfaceAdaptor();
                s.Initialize(face, false);
                s.D1(u_, v_, out APoint_, out vecD1U, out vecD1V);

                if (vecD1U.CrossMagnitude(vecD1V) > Constants.Resolution())
                {
                    return true;
                }

                if (paramInit < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
        }

        return false;
    }

    /// <summary>A point in the face, searched on a grid of the given UV bounds of <paramref name="surf"/> (see the overload with derivatives).</summary>
    public bool PointInTheFace(in Face face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int indexPoint,
                               FaceSurfaceAdaptor surf,
                               double u1,
                               double v1,
                               double u2,
                               double v2)
    {
        Vector3D vecD1U = new(), vecD1V = new();
        return PointInTheFace(face,
                              ref APoint_,
                              ref u_,
                              ref v_,
                              ref param_,
                              ref indexPoint,
                              surf,
                              u1,
                              v1,
                              u2,
                              v2,
                              ref vecD1U,
                              ref vecD1V);
    }

    /// <summary>The state of a UV point in a face: ON within the tolerance of an edge or vertex, otherwise the intersector's.</summary>
    protected Containment ClassifyUVPoint(CurveSurfaceIntersector intersector,
                                           FaceSurfaceAdaptor surf,
                                           in Point2D p2d)
    {
        // first find if the point is near an edge/vertex
        Point3D p3d = surf.Value(p2d.X, p2d.Y);
        var selectorPoint = new BndBoxTreeSelectorPoint(GetMapEV());
        selectorPoint.SetCurrentPoint(p3d);
        int selsVE = GetTree().Select(selectorPoint);
        if (selsVE > 0)
        {
            // The point is inside the tolerance area of vertices/edges => return ON state.
            return On;
        }

        return intersector.ClassifyUVPoint(p2d);
    }

    /// <summary>
    /// A point in the face, searched on a grid of the given UV bounds of <paramref name="surf"/> from the point of
    /// index <paramref name="indexPoint"/>, which returns the index of the point found; then by
    /// <see cref="FindAPointInTheFace(in Face, ref Point3D, ref double, ref double, ref double, ref Vector3D, ref Vector3D)"/>.
    /// </summary>
    public bool PointInTheFace(in Face face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int indexPoint,
                               FaceSurfaceAdaptor surf,
                               double u1,
                               double v1,
                               double u2,
                               double v2,
                               ref Vector3D vecD1U,
                               ref Vector3D vecD1V)
    {
        double u, du = (u2 - u1) / 6.0;
        double v, dv = (v2 - v1) / 6.0;
        if (du < 1e-12)
        {
            du = 1e-12;
        }

        if (dv < 1e-12)
        {
            dv = 1e-12;
        }

        bool isNotUper = !surf.IsUPeriodic, isNotVper = !surf.IsVPeriodic;
        int nbPntCalc = 0;
        if (_mapOfInter.ContainsKey(face))
        {
            CurveSurfaceIntersector? ptr = _mapOfInter.GetValue(face);
            bool isInside = true;
            if (isNotUper)
            {
                isInside = (u_ >= u1) && (u_ <= u2);
            }

            if (isNotVper)
            {
                isInside &= (v_ >= v1) && (v_ <= v2);
            }

            if (ptr is not null)
            {
                CurveSurfaceIntersector theIntersector = ptr;
                // Check if the point is already in the face
                if (isInside && (ClassifyUVPoint(theIntersector, surf, new Point2D(u_, v_)) == In))
                {
                    surf.D1(u_, v_, out Point3D pnt, out vecD1U, out vecD1V);
                    if (pnt.SquareDistanceTo(APoint_) < global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return true;
                    }
                }

                //-- Take 4 points in each Quarter of surface
                //-- -> Index : 1 -> 16
                //--
                //--
                //--  Then take a matrix of points on a tight grid
                //--
                for (u = du + (u1 + u2) * 0.5; u < u2; u = StepLimit.Moved(u, u + du))
                {
                    //--  0  X    u increases
                    for (v = dv + (v1 + v2) * 0.5; v < v2; v = StepLimit.Moved(v, v + dv))
                    {
                        //--  0  0    v increases
                        if (++nbPntCalc >= indexPoint)
                        {
                            if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                                indexPoint = nbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = -du + (u1 + u2) * 0.5; u > u1; u = StepLimit.Moved(u, u - du))
                {
                    //--  0  0    u decreases
                    for (v = -dv + (v1 + v2) * 0.5; v > v1; v = StepLimit.Moved(v, v - dv))
                    {
                        //--  X  0    v decreases
                        if (++nbPntCalc >= indexPoint)
                        {
                            if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                                indexPoint = nbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = -du + (u1 + u2) * 0.5; u > u1; u = StepLimit.Moved(u, u - du))
                {
                    //--  X  0    u decreases
                    for (v = dv + (v1 + v2) * 0.5; v < v2; v = StepLimit.Moved(v, v + dv))
                    {
                        //--  0  0    v increases
                        if (++nbPntCalc >= indexPoint)
                        {
                            if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                                indexPoint = nbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = du + (u1 + u2) * 0.5; u < u2; u = StepLimit.Moved(u, u + du))
                {
                    //--  0  0     u increases
                    for (v = -dv + (v1 + v2) * 0.5; v > v1; v = StepLimit.Moved(v, v - dv))
                    {
                        //--  0  X     v decreases
                        if (++nbPntCalc >= indexPoint)
                        {
                            if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                                indexPoint = nbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                //-- the remainder
                du = (u2 - u1) / 37.0;
                dv = (v2 - v1) / 37.0;
                if (du < 1e-12)
                {
                    du = 1e-12;
                }

                if (dv < 1e-12)
                {
                    dv = 1e-12;
                }

                for (u = du + u1; u < u2; u = StepLimit.Moved(u, u + du))
                {
                    for (v = dv + v1; v < v2; v = StepLimit.Moved(v, v + dv))
                    {
                        if (++nbPntCalc >= indexPoint)
                        {
                            if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                                indexPoint = nbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                u = (u1 + u2) * 0.5;
                v = (v1 + v2) * 0.5;
                if (++nbPntCalc >= indexPoint)
                {
                    if (ClassifyUVPoint(theIntersector, surf, new Point2D(u, v)) == In)
                    {
                        u_ = u;
                        v_ = v;
                        surf.D1(u, v, out APoint_, out vecD1U, out vecD1V);
                        indexPoint = nbPntCalc;
                        return (true);
                    }
                }
            }

            indexPoint = nbPntCalc;
        }
        else
        {
            // printf("BRepClass3d_SolidExplorer Face not found ds the map \n");
        }

        return FindAPointInTheFace(face,
                                   ref APoint_,
                                   ref u_,
                                   ref v_,
                                   ref param_,
                                   ref vecD1U,
                                   ref vecD1V);
    }

    private static void LimitInfiniteUV(ref double u1, ref double v1, ref double u2, ref double v2)
    {
        bool infU1 = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1), infV1 = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(v1),
             infU2 = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2), infV2 = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(v2);

        if (infU1)
        {
            u1 = -1e10;
        }

        if (infV1)
        {
            v1 = -1e10;
        }

        if (infU2)
        {
            u2 = 1e10;
        }

        if (infV2)
        {
            v2 = 1e10;
        }
    }

    private static int IsInfiniteUV(double u1, double v1, double u2, double v2)
    {
        int val = 0;

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
        {
            val |= 1;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
        {
            val |= 2;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
        {
            val |= 4;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
        {
            val |= 8;
        }

        return val;
    }

    /// <summary>
    /// A line <paramref name="l"/> from <paramref name="p"/> that meets the shape's boundary, with in
    /// <paramref name="_Par"/> the parameter up to which to intersect: the first call aims at a point of the first
    /// face, the next at the second face, and so on. Returns 0 for a line, 1 if the point is on a face (or the face
    /// is infinite), 2 for a degenerate face (from a bad shell), 3 if the point is on the surface but outside the face.
    /// </summary>
    public int OtherSegment(in Point3D p, ref Line3D l, ref double _Par)
    {
        double tolU = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double tolV = tolU;

        Face face;
        var faceexplorer = new Explorer();
        Point3D aPoint = new();
        Vector3D vecD1U = new(), vecD1V = new();
        double maxscal = 0;
        bool ptfound = false;
        double par;
        double _u, _v;
        int indexPoint = 0;
        int nbPointsOK = 0;
        int nbFacesInSolid = 0;
        bool restr = true;
        bool testInvert = false;

        for (;;)
        {
            _firstFace++;
            faceexplorer.Init(_shape, ShapeType.Face);
            // look for point on face starting from myFirstFace
            //  Modified by skv - Thu Sep  4 14:31:12 2003 OCC578 Begin
            //     while (faceexplorer.More()) {
            nbFacesInSolid = 0;
            for (; faceexplorer.More(); faceexplorer.Next())
            {
                //  Modified by skv - Thu Sep  4 14:31:12 2003 OCC578 End
                nbFacesInSolid++;
                if (_firstFace > nbFacesInSolid)
                {
                    continue;
                }

                face = ShapeCasts.Face(faceexplorer.Current());

                var surf = new FaceSurfaceAdaptor();
                if (testInvert)
                {
                    var aClass = new FClass2d(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (aClass.PerformInfinitePoint() == In)
                    {
                        restr = false;
                        if (_mapOfInter.TryGetValue(face, out CurveSurfaceIntersector? restricted))
                        {
                            // Kerf: the first kept, for the next point (ForANewPoint)
                            (_restricted ??= new(ShapeHasher.Instance)).TryAdd(face, restricted);
                            _mapOfInter.Remove(face);
                            var ptr = new CurveSurfaceIntersector(face, global::Kerf.Engine.Foundation.Precision.Confusion(), restr, false);
                            _mapOfInter[face] = ptr;
                        }
                    }
                    else
                    {
                        restr = true;
                    }
                }

                surf.Initialize(face, restr);
                double u1, v1, u2, v2;
                u1 = surf.FirstUParameter();
                v1 = surf.FirstVParameter();
                u2 = surf.LastUParameter();
                v2 = surf.LastVParameter();
                face.Orient(Forward);
                //
                // avoid process faces from uncorrected shells
                double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
                double epsU = Max(eps * Max(Math.Abs(u2), Math.Abs(u1)), eps);
                double epsV = Max(eps * Max(Math.Abs(v2), Math.Abs(v1)), eps);
                if (Math.Abs(u2 - u1) < epsU || Math.Abs(v2 - v1) < epsV)
                {
                    return 2;
                }

                //
                double svmyparam = _paramOnEdge;
                //
                // Check if the point is on the face or the face is infinite.
                int infFlag = IsInfiniteUV(u1, v1, u2, v2);
                // default values
                _u = (u1 + u2) * 0.5;
                _v = (v1 + v2) * 0.5;

                var ga = new SurfaceAdaptor(Tool.Surface(face)!);
                var ext = new ExtPS(p, ga, tolU, tolV, ExtFlag_MIN);
                //
                if (ext.IsDone && ext.ExtCount > 0)
                {
                    int i, iNear, iEnd;
                    double ux, vx, dist2, dist2Min;
                    POnSurf px;
                    //
                    iNear = 1;
                    dist2Min = ext.SquareDistance(1);
                    iEnd = ext.ExtCount;
                    for (i = 2; i <= iEnd; i++)
                    {
                        px = ext.Point(i);
                        px.Parameter(out ux, out vx);
                        if (ux >= u1 && ux <= u2 && vx >= v1 && vx <= v2)
                        {
                            dist2 = ext.SquareDistance(i);
                            if (dist2 < dist2Min)
                            {
                                dist2Min = dist2;
                                iNear = i;
                            }
                        }
                    }

                    //
                    double dist2Tresh = 1.0e-24;
                    //
                    if (dist2Min < dist2Tresh)
                    {
                        if (infFlag != 0)
                        {
                            return 1;
                        }
                        else
                        {
                            var classifier2d = new FaceClassifier();
                            (ext.Point(iNear)).Parameter(out double u, out double aV);

                            Point2D puv = new(u, aV);

                            classifier2d.Perform(face, puv, global::Kerf.Engine.Foundation.Precision.PConfusion());

                            Containment state = classifier2d.State();

                            if (state == In || state == On)
                            {
                                return 1;
                            }
                            else
                            {
                                return 3; // skv - the point is on surface but outside face.
                            }
                        }
                    }

                    if (infFlag != 0)
                    {
                        aPoint = (ext.Point(iNear)).Value();
                        Vector3D v = new(p, aPoint);
                        _Par = v.Magnitude;
                        l = new Line3D(p, new Direction3D(v));
                        ptfound = true;
                        return 0;
                    }

                    // set the parameters found by extrema
                    px = ext.Point(iNear);
                    px.Parameter(out _u, out _v);
                    aPoint = px.Value();
                }

                // The point is not ON the face or surface. The face is restricted.
                //  find point in a face not too far from a projection of P on face
                do
                {
                    ++indexPoint;
                    if (PointInTheFace(face,
                                       ref aPoint,
                                       ref _u,
                                       ref _v,
                                       ref _paramOnEdge,
                                       ref indexPoint,
                                       surf,
                                       u1,
                                       v1,
                                       u2,
                                       v2,
                                       ref vecD1U,
                                       ref vecD1V))
                    {
                        ++nbPointsOK;
                        Vector3D v = new(p, aPoint);
                        par = v.Magnitude;
                        if (par > Constants.Resolution() && vecD1U.Magnitude > Constants.Resolution()
                            && vecD1V.Magnitude > Constants.Resolution())
                        {
                            Vector3D norm = vecD1U.Crossed(vecD1V);
                            double tt = norm.Magnitude;
                            if (tt > Constants.Resolution())
                            {
                                tt = Math.Abs(norm.Dot(v)) / (tt * par);
                                if (tt > maxscal)
                                {
                                    maxscal = tt;
                                    l = new Line3D(p, new Direction3D(v));
                                    _Par = par;
                                    ptfound = true;
                                    if (maxscal > 0.2)
                                    {
                                        _paramOnEdge = svmyparam;
                                        return 0;
                                    }
                                }
                            }
                        }
                    }
                } while (indexPoint < 200 && nbPointsOK < 16);

                _paramOnEdge = svmyparam;
                if (maxscal > 0.2)
                {
                    return 0;
                }

                //  Modified by skv - Thu Sep  4 14:32:14 2003 OCC578 Begin
                // Next is done in the main for(..) loop.
                //       faceexplorer.Next();
                //  Modified by skv - Thu Sep  4 14:32:14 2003 OCC578 End
                indexPoint = 0;

                bool encoreuneface = faceexplorer.More();
                if (!ptfound && !encoreuneface)
                {
                    if (_paramOnEdge < 0.0001)
                    {
                        //-- This case takes place when the point is on the solid
                        //-- and this solid is reduced to a face
                        Point3D pBidon = new(p.X + 1.0, p.Y, p.Z);
                        Vector3D v = new(p, pBidon);
                        par = 1.0;
                        _Par = par;
                        l = new Line3D(p, new Direction3D(v));
                        return 0;
                    }
                }
            } //-- Exploration of the faces

            if (nbFacesInSolid == 0)
            {
                _Par = 0.0;
                _reject = true;
                return 0;
            }

            if (ptfound)
            {
                return 0;
            }

            _firstFace = 0;
            if (_paramOnEdge == 0.512345)
            {
                _paramOnEdge = 0.4;
            }
            else if (_paramOnEdge == 0.4)
            {
                _paramOnEdge = 0.6;
            }
            else if (_paramOnEdge == 0.6)
            {
                _paramOnEdge = 0.3;
            }
            else if (_paramOnEdge == 0.3)
            {
                _paramOnEdge = 0.7;
            }
            else if (_paramOnEdge == 0.7)
            {
                _paramOnEdge = 0.2;
            }
            else if (_paramOnEdge == 0.2)
            {
                _paramOnEdge = 0.8;
            }
            else if (_paramOnEdge == 0.8)
            {
                _paramOnEdge = 0.1;
            }
            else if (_paramOnEdge == 0.1)
            {
                _paramOnEdge = 0.9;
                //
            }
            else
            {
                _paramOnEdge *= 0.5;
                if (_paramOnEdge < 0.0001)
                {
                    Point3D pBidon = new(p.X + 1.0, p.Y, p.Z);
                    Vector3D v = new(p, pBidon);
                    par = 1.0;
                    _Par = par;
                    l = new Line3D(p, new Direction3D(v));
                    return 0;
                }
            }

            testInvert = true;
        } //-- for(;;) { ...  }
    }

    //  Modified by skv - Thu Sep  4 12:30:14 2003 OCC578 Begin

    /// <summary>The index of the face the last line was computed for.</summary>
    public int GetFaceSegmentIndex() => _firstFace;

    //  Modified by skv - Thu Sep  4 12:30:14 2003 OCC578 End

    /// <summary>A point in the face, searched on a grid of its whole surface (infinite bounds limited to 1e10).</summary>
    public bool PointInTheFace(in Face _face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int indexPoint)
    {
        Face face = _face;
        face.Orient(Forward);
        var surf = new FaceSurfaceAdaptor();
        surf.Initialize(face);
        double u1, v1, u2, v2; //,u,v;
        u1 = surf.FirstUParameter();
        v1 = surf.FirstVParameter();
        u2 = surf.LastUParameter();
        v2 = surf.LastVParameter();
        LimitInfiniteUV(ref u1, ref v1, ref u2, ref v2);
        return (PointInTheFace(face, ref APoint_, ref u_, ref v_, ref param_, ref indexPoint, surf, u1, v1, u2, v2));
    }

    /// <summary>A point in the face with its parameters, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in Face _face, ref Point3D APoint_, ref double u_, ref double v_)
    {
        double param = 0.1;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u_, ref v_, ref param);
        return r;
    }

    /// <summary>A point in the face, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in Face _face, ref Point3D APoint_)
    {
        double u = 0.0, v = 0.0;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u, ref v);
        return r;
    }

    /// <summary>The parameters of a point in the face, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in Face _face, ref double u_, ref double v_)
    {
        Point3D aPoint = new();
        bool r = FindAPointInTheFace(_face, ref aPoint, ref u_, ref v_);
        return r;
    }

    /// <summary>An empty explorer.</summary>
    public SolidExplorer()
    {
        _reject = true;
        _firstFace = 0;
        _paramOnEdge = 0.0;
    }

    /// <summary>An explorer of the shape <paramref name="s"/>.</summary>
    public SolidExplorer(in Shape s)
    {
        InitShape(s);
    }

    /// <summary>
    /// An explorer of <paramref name="s"/> for a classifier that only classifies the point at infinity, which reads
    /// only the faces' intersectors: the edges and vertices and their tree are left until something asks for them.
    /// </summary>
    /// <remarks>
    /// Kerf: they were half of a shell's orientation in shape healing (performance.md, "The perf search"). Filled then, they
    /// are what they would have been, as long as the shape hasn't changed since, which is why only a classifier that's used
    /// at once and dropped is made this way (<see cref="SolidClassifier.InfinitePointState"/>).
    /// </remarks>
    internal SolidExplorer(in Shape s, bool fillTree)
    {
        InitShape(s, fillTree);
    }

    /// <summary>Forgets the faces' intersectors (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy()
    {
        _mapOfInter.Clear();
        _restricted?.Clear();
    }

    /// <summary>
    /// Loads the shape <paramref name="s"/>: an intersector for each face, and the tree of the boxes of the edges
    /// and vertices of its FORWARD and REVERSED faces and edges (degenerated edges left out).
    /// </summary>
    public void InitShape(in Shape s) => InitShape(s, true);

    private void InitShape(in Shape s, bool fillTree)
    {
        _mapEV.Clear();
        _tree.Clear();
        _treeFilled = false;
        _mapEF = null;

        _shape = s;
        _firstFace = 0;
        _paramOnEdge = 0.512345;
        //-- Exploring of the Map and removal of allocated objects

        _mapOfInter.Clear();
        _restricted?.Clear();
        _bounds = null;
        _finite = null;
        _unscaled = null;
        _unscaledAsked = false;

        _reject = true; //-- case of infinite solid (without any face)

        var expl = new Explorer();
        for (expl.Init(s, ShapeType.Face); expl.More(); expl.Next())
        {
            Face face = ShapeCasts.Face(expl.Current());
            var ptr = new CurveSurfaceIntersector(face, global::Kerf.Engine.Foundation.Precision.Confusion(), true, false);
            _mapOfInter[face] = ptr;
            _reject = false; //-- at least one face in the solid
        }

        if (fillTree)
        {
            FillTree();
        }
    }

    // The map of the edges and vertices and the tree of their boxes, as OCCT's InitShape fills them
    private void FillTree()
    {
        _treeFilled = true;

        // since the internal/external parts should be avoided in tree filler,
        // there is no need to add these parts in the EV map as well
        var expF = new Explorer(_shape, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Shape f = expF.Current();
            //
            Orientation orF = f.Orientation;
            if (orF == Internal || orF == External)
            {
                continue;
            }

            //
            var expE = new Explorer(f, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Shape e = expE.Current();
                //
                Orientation orE = e.Orientation;
                if (orE == Internal || orE == External)
                {
                    continue;
                }

                //
                if (Tool.Degenerated(ShapeCasts.Edge(e)))
                {
                    continue;
                }

                // Kerf (unit 15AR): an edge already mapped, from the face on its other side, is there with its vertices, as the
                // map compares shapes by IsSame: mapping it again adds nothing
                if (_mapEV.Contains(e))
                {
                    continue;
                }

                //
                Exploration.MapShapes(e, _mapEV);
            }
        }

        //
        // Fill mapEV with vertices and edges from shape
        var treeFiller = new SpatialTreeFiller<int, Box>(_tree);
        //
        int i, nbEV = _mapEV.Count;
        for (i = 1; i <= nbEV; ++i)
        {
            Shape aS = _mapEV[i - 1];
            //
            Box box = new();
            Bounds.Add(aS, ref box);
            treeFiller.Add(i, box);
        }

        treeFiller.Fill();
    }

    /// <summary>True if the point is outside a bounding volume of the shape: true only for a shape without faces.</summary>
    public virtual bool Reject(in Point3D p) => (_reject); // case of solid without face

    /// <summary>Starts an exploration of the shells.</summary>
    public void InitShell() => _shellExplorer.Init(_shape, ShapeType.Shell);

    /// <summary>True if there is a current shell.</summary>
    public bool MoreShell() => (_shellExplorer.More());

    /// <summary>Moves to the next shell.</summary>
    public void NextShell() => _shellExplorer.Next();

    /// <summary>The current shell.</summary>
    public Shell CurrentShell() => (ShapeCasts.Shell(_shellExplorer.Current()));

    /// <summary>True if the shell is rejected: always false.</summary>
    public virtual bool RejectShell(in Line3D l) => (false);

    /// <summary>Starts an exploration of the faces of the current shell.</summary>
    public void InitFace() => _faceExplorer.Init(ShapeCasts.Shell(_shellExplorer.Current()), ShapeType.Face);

    /// <summary>True if there is a current face in the current shell.</summary>
    public bool MoreFace() => (_faceExplorer.More());

    /// <summary>Moves to the next face of the current shell.</summary>
    public void NextFace() => _faceExplorer.Next();

    /// <summary>The current face.</summary>
    public Face CurrentFace() => (ShapeCasts.Face(_faceExplorer.Current()));

    /// <summary>True if the face is rejected: always false.</summary>
    public virtual bool RejectFace(in Line3D l) => (false);

    /// <summary>A line from <paramref name="p"/> to the boundary, starting again from the first face (see <see cref="OtherSegment"/>).</summary>
    public int Segment(in Point3D p, ref Line3D l, ref double par)
    {
        int bRetFlag;
        _firstFace = 0;
        bRetFlag = OtherSegment(p, ref l, ref par);
        return bRetFlag;
    }

    /// <summary>
    /// Kerf (units 15C and 15C2): the choice of face and of where on an edge to aim set back to what
    /// <see cref="InitShape(in Shape)"/> leaves, each face's intersector that <see cref="OtherSegment"/> replaced by an
    /// unrestricted one put back, and each intersector's flag of a line found parallel set back to a new one's: all a
    /// classification leaves in the explorer that the next one can read before setting it itself. Every classification of a
    /// point starts here (<see cref="SClassifier.Perform(SolidExplorer, in Point3D, double)"/>), so it gives what a new
    /// classifier would; OCCT carries them over.
    /// </summary>
    internal void ForANewPoint()
    {
        _firstFace = 0;
        _paramOnEdge = 0.512345;
        if (_restricted is { Count: > 0 })
        {
            foreach ((Shape face, CurveSurfaceIntersector? restricted) in _restricted)
            {
                _mapOfInter[face] = restricted;
            }
            _restricted.Clear();
        }
        foreach (CurveSurfaceIntersector? intersector in _mapOfInter.Values)
        {
            intersector?.ForgetParallel();
        }
    }

    /// <summary>The intersector of the face <paramref name="f"/>.</summary>
    /// <exception cref="NoSuchObjectException">The face is not in the shape.</exception>
    public CurveSurfaceIntersector Intersector(in Face f)
    {
        CurveSurfaceIntersector? ptr = _mapOfInter.GetValue(f);
        CurveSurfaceIntersector curr = ptr!;
        return curr;
    }

    /// <summary>Does nothing (OCCT's debug output).</summary>
    public virtual void DumpSegment(in Point3D p, in Line3D l, double par, Containment s)
    {
    }

    /// <summary>The shape.</summary>
    public Shape GetShape() => (_shape);

    /// <summary>
    /// Kerf (unit 15E17, decision 35008): for a shape whose own placement has a scale, and nothing placed with a scale below
    /// it, an explorer of the shape without that placement, in which a point is classified by its image under the
    /// placement's inverse; null for any other shape. The ray's crossings aren't the geometry's under a scale: a point
    /// inside a scaled box by its corner read OUT. Made when first asked, and kept.
    /// </summary>
    internal SolidExplorer? Unscaled(out Placement placement)
    {
        placement = _shape.Placement;
        if (!_unscaledAsked)
        {
            _unscaledAsked = true;
            if (placement.Transformation.ScaleFactor != 1.0)
            {
                Shape inner = _shape.Located(new Placement());
                if (!IsScaled(inner))
                {
                    _unscaled = new SolidExplorer(inner);
                }
            }
        }

        return _unscaled;
    }

    // Whether a face of the shape is placed with a scale
    private static bool IsScaled(in Shape shape)
    {
        for (var exp = new Explorer(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().Placement.Transformation.ScaleFactor != 1.0)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Kerf (unit 15E17, decision 35003): true if <paramref name="p"/> is farther than <paramref name="tol"/> outside the
    /// shape's box and the shape bounds a finite region (it isn't reversed, its shells are closed, no face is placed with a
    /// scale, and its point at infinity
    /// is OUT): then the point is outside it. The box and the region's finiteness are made when first asked, and kept.
    /// </summary>
    internal bool IsOutsideFiniteBounds(in Point3D p, double tol)
    {
        if (_reject)
        {
            return false;
        }

        if (_bounds is null)
        {
            Box box = new();
            Bounds.Add(_shape, ref box, false);
            _bounds = box;
        }

        if (_bounds.Value.IsVoid || !_bounds.Value.Enlarged(tol).IsOut(p))
        {
            return false;
        }

        // A box flat in a direction isn't a solid's: OCCT's bug26609_z, an invalid solid of five faces whose box is 2e-7
        // thick, holds the point its test asks for, by the ray, 30 below that box
        _bounds.Value.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        double flat = 10 * Kerf.Engine.Foundation.Precision.Confusion() + 2 * tol;
        if (xmax - xmin <= flat || ymax - ymin <= flat || zmax - zmin <= flat)
        {
            return false;
        }

        _finite ??= _shape.Orientation != Reversed && IsClosedUnscaled(_shape) && SolidClassifier.InfinitePointState(_shape, tol) == Out;
        return _finite.Value;
    }

    // Whether every shell of the shape is closed and no face is placed with a scale. An open shell, a half-space's face
    // among them, bounds no finite region, whatever its point at infinity reads. A shape placed with a scale is classified
    // in its unscaled frame (Unscaled); one with a scale below its own placement is left to the ray
    private static bool IsClosedUnscaled(in Shape shape)
    {
        bool hasShell = false;
        for (var exp = new Explorer(shape, ShapeType.Shell); exp.More(); exp.Next())
        {
            hasShell = true;
            if (!Tool.IsClosed(exp.Current()))
            {
                return false;
            }
        }

        if (!hasShell)
        {
            return false;
        }

        return !IsScaled(shape);
    }

    /// <summary>The tree of the boxes of the edges and vertices (objects are indices in <see cref="GetMapEV"/>).</summary>
    public SpatialTree<int, Box> GetTree()
    {
        if (!_treeFilled)
        {
            FillTree();
        }

        return _tree;
    }

    /// <summary>
    /// The faces of each edge of the shape, as <c>BRepClass3d_SClassifier::Perform</c> maps them for each point. A frozen
    /// shape's are kept (unit 12Y): its TShapes never change again, so the map made again would be the same, in the same
    /// order. Made afresh for a shape that isn't frozen, which an algorithm may change between points, and inside an open
    /// working overlay, where a frozen TShape can read as its working copy; and never kept from there.
    /// </summary>
    public IndexedMap<Shape, NCollection_List<Shape>> GetMapEF()
    {
        bool keep = _shape.TShape() is { IsFrozen: true } && Kerf.Engine.Topology.Tools.WorkingOverlay.Current is null;
        if (keep && Volatile.Read(ref _mapEF) is { } kept)
        {
            return kept;
        }

        var mapEF = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, mapEF);
        if (keep)
        {
            Volatile.Write(ref _mapEF, mapEF);
        }

        return mapEF;
    }

    /// <summary>The edges and vertices of the shape.</summary>
    public IndexedSet<Shape> GetMapEV()
    {
        if (!_treeFilled)
        {
            FillTree();
        }

        return _mapEV;
    }
}
