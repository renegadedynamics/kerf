// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FindSurface.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FindSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Finds a surface through the edges of a shape: a surface on which every edge has a pcurve (<see cref="Existed"/>),
/// or else a plane fitted by least squares to points sampled along the edges, accepted if no point is farther than
/// the tolerance.
/// </summary>
/// <remarks>
/// <para>The tolerance is the larger of the given one and the edges' largest; a negative tolerance accepts a plane up
/// to five times the edges' tolerance. A plane through a closed wire is oriented so that the wire bounds a finite
/// area.</para>
/// <para>A working object; it only reads the shape. <see cref="Surface"/> freezes the surface it returns.</para>
/// </remarks>
internal class FindSurface
{
    private Surface? _surface;
    private double _tolerance;
    private double _tolReached;
    private bool _isExisted;
    private Placement _location = new();

    private static double Controle(NCollection_Sequence<Point3D> points, PlaneSurface plane)
    {
        double dfMaxDist = 0.0;
        double dist;
        int ii;
        plane.Coefficients(out double a, out double b, out double c, out double d);
        for (ii = 1; ii <= points.Length(); ii++)
        {
            Xyz xyz = points[ii].XYZ();
            dist = Math.Abs(a * xyz.X() + b * xyz.Y() + c * xyz.Z() + d);
            if (dist > dfMaxDist)
            {
                dfMaxDist = dist;
            }
        }

        return dfMaxDist;
    }

    // Return true if the last vertex of theEdge1 coincides with
    // the first vertex of theEdge2 in parametric space of theFace
    private static bool Is2DConnected(in Edge edge1,
                                      in Edge edge2,
                                      Surface surface,
                                      in Placement location)
    {
        double f, l;
        Curve2D? curve;
        Point2D p1, p2;

        // get 2D points
        curve = Tool.CurveOnSurface(edge1, surface, location, out f, out l);
        p1 = curve!.Value(edge1.Orientation == Forward ? l : f);
        curve = Tool.CurveOnSurface(edge2, surface, location, out f, out l);
        p2 = curve!.Value(edge2.Orientation == Forward ? f : l);

        // compare 2D points
        var aSurface = new SurfaceAdaptor(surface);
        Vertex v = Exploration.FirstVertex(edge2, true);
        double tol3D = Tool.Tolerance(v);
        double tol2D = aSurface.UResolution(tol3D) + aSurface.VResolution(tol3D);
        double dist2 = p1.SquareDistanceTo(p2);
        return dist2 < tol2D * tol2D;
    }

    // Return true if edges of theShape form a closed wire in
    // parametric space of theSurface
    private static bool Is2DClosed(in Shape shape, Surface surface, in Placement location)
    {
        try
        {
            // get a wire theShape
            var wireExp = new Explorer(shape, ShapeType.Wire);
            if (!wireExp.More())
            {
                return false;
            }
            Wire wire = ShapeCasts.Wire(wireExp.Current());
            // a tmp face
            Face tmpFace = new LibraryMakeFace(surface, global::Kerf.Engine.Foundation.Precision.PConfusion()).InternalFace();

            // check topological closeness using wire explorer, if the wire is not closed
            // the 1st and the last vertices of wire are different
            var wireExplorer = new WireExplorer(wire, tmpFace);
            if (!wireExplorer.More())
            {
                return false;
            }
            // remember the 1st and the last edges of aWire
            Edge fisrtEdge = wireExplorer.Current(), lastEdge = fisrtEdge;
            // check if edges connected topologically (that is assured by BRepTools_WireExplorer)
            // are connected in 2D
            Edge prevEdge = fisrtEdge;
            for (wireExplorer.Next(); wireExplorer.More(); wireExplorer.Next())
            {
                lastEdge = wireExplorer.Current();
                if (!Is2DConnected(prevEdge, lastEdge, surface, location))
                {
                    return false;
                }
                prevEdge = lastEdge;
            }
            // wire is closed if ( 1st vertex of aFisrtEdge ) ==
            // ( last vertex of aLastEdge ) in 2D
            Vertex v1 = Exploration.FirstVertex(fisrtEdge, true);
            Vertex v2 = Exploration.LastVertex(lastEdge, true);
            return (v1.IsSame(v2) && Is2DConnected(lastEdge, fisrtEdge, surface, location));
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return false;
        }
        catch (NullReferenceException)
        {
            // OCCT dereferences a null pcurve under OCC_CATCH_SIGNALS, which turns the access violation into a
            // Standard_Failure caught here.
            return false;
        }
    }

    /// <summary>Creates an empty finder.</summary>
    public FindSurface()
    {
        _tolerance = 0.0;
        _tolReached = 0.0;
        _isExisted = false;
    }

    /// <summary>Finds a surface through the edges of <paramref name="s"/> (see <see cref="Init"/>).</summary>
    public FindSurface(in Shape s, double tol = -1, bool onlyPlane = false, bool onlyClosed = false)
    {
        Init(s, tol, onlyPlane, onlyClosed);
    }

    private static void FillParams(NCollection_Array1<double> knots,
                                   int degree,
                                   double parMin,
                                   double parMax,
                                   List<double> theParams)
    {
        double prevPar = parMin;
        theParams.Add(prevPar);

        int nbP = Max(degree, 1);

        for (int i = 1; (i < knots.Length()) && (knots[i] < (parMax - global::Kerf.Engine.Foundation.Precision.PConfusion())); ++i)
        {
            if (knots[i + 1] < parMin + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            double step = (knots[i + 1] - knots[i]) / nbP;
            for (int k = 1; k <= nbP; ++k)
            {
                double par = knots[i] + k * step;
                if (par > parMax - global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    break;
                }

                if (par > prevPar + global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    theParams.Add(par);
                    prevPar = par;
                }
            }
        }
        theParams.Add(parMax);
    }

    private static void FillPoints(EdgeCurveAdaptor curve,
                                   List<double> theParams,
                                   NCollection_Sequence<Point3D> points,
                                   NCollection_Sequence<double> weights)
    {
        double distPrev = 0.0, distNext;
        Point3D pPrev = curve.Value(theParams[0]), pNext = new();

        for (int iP = 1; iP <= theParams.Count; ++iP)
        {
            if (iP < theParams.Count)
            {
                double param = theParams[iP];
                pNext = curve.Value(param);
                distNext = pPrev.DistanceTo(pNext);
            }
            else
            {
                distNext = 0.0;
            }

            points.Append(pPrev);
            weights.Append(distPrev + distNext);
            distPrev = distNext;
            pPrev = pNext;
        }
    }

    /// <summary>
    /// Finds a surface through the edges of <paramref name="s"/>: an existing surface of the first edge's pcurves that
    /// every edge has a pcurve on (with <paramref name="onlyPlane"/>, only a plane; with <paramref name="onlyClosed"/>,
    /// only one on which the shape's wire is closed in 2D), otherwise a least-squares plane within the tolerance.
    /// </summary>
    public void Init(in Shape s, double tol = -1, bool onlyPlane = false, bool onlyClosed = false)
    {
        _tolerance = tol;
        _tolReached = 0.0;
        _isExisted = false;
        _location = new Placement();
        _surface = null;

        // compute the tolerance
        var ex = new Explorer();

        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            double t = Tool.Tolerance(ShapeCasts.Edge(ex.Current()));
            if (t > _tolerance)
            {
                _tolerance = t;
            }
        }

        // search an existing surface
        ex.Init(s, ShapeType.Edge);
        if (!ex.More())
        {
            return; // no edges ....
        }

        Edge E = ShapeCasts.Edge(ex.Current());
        double f = 0, l = 0, ff = 0, ll = 0;
        Curve2D? pc = null, ppc = null;
        Surface? ss = null;
        var L = new Placement();
        int i = 0, j;

        // iterate on the surfaces of the first edge
        for (;;)
        {
            i++;
            Tool.CurveOnSurface(E, ref pc, ref _surface, ref _location, ref f, ref l, i);
            if (_surface is null)
            {
                break;
            }
            // check the other edges
            for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
            {
                if (!E.IsSame(ex.Current()))
                {
                    j = 0;
                    for (;;)
                    {
                        j++;
                        Tool.CurveOnSurface(ShapeCasts.Edge(ex.Current()), ref ppc, ref ss, ref L, ref ff, ref ll, j);
                        if (ss is null)
                        {
                            break;
                        }
                        if (ReferenceEquals(ss, _surface) && (L.IsEqual(_location)))
                        {
                            break;
                        }
                        ss = null;
                    }

                    if (ss is null)
                    {
                        _surface = null;
                        break;
                    }
                }
            }

            // if OnlyPlane, eval if mySurface is a plane.
            if (onlyPlane && _surface is not null)
            {
                if (_surface is TrimmedSurface rts)
                {
                    _surface = rts.BasisSurface;
                }
                _surface = _surface as PlaneSurface;
            }

            if (_surface is not null)
            {
                // if S is e.g. the bottom face of a cylinder, mySurface can be the
                // lateral (cylindrical) face of the cylinder; reject an improper mySurface
                if (!onlyClosed || Is2DClosed(s, _surface, _location))
                {
                    break;
                }
            }
        }

        if (_surface is not null)
        {
            _isExisted = true;
            return;
        }
        //
        // no existing surface, search a plane
        // 07/02/02 akm vvv : (OCC157) changed algorithm
        //                    1. Collect the points along all edges of the shape
        //                       For each point calculate the WEIGHT = sum of
        //                       distances from neighboring points (_only_ same edge)
        //                    2. Minimizing the weighed sum of squared deviations
        //                       compute coefficients of the sought plane.

        var points = new NCollection_Sequence<Point3D>();
        var weight = new NCollection_Sequence<double>();

        // ======================= Step #1
        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            var c = new EdgeCurveAdaptor(ShapeCasts.Edge(ex.Current()));

            double dfUf = c.FirstParameter();
            double dfUl = c.LastParameter();
            if (IsEqual(dfUf, dfUl))
            {
                // Degenerate
                continue;
            }
            int iNbPoints = 0;

            // Fill the parameters of the sampling points
            var aParams = new List<double>();
            switch (c.GetType())
            {
                case BezierCurve:
                {
                    BezierCurve3D gc = c.Bezier();
                    var knots = new NCollection_Array1<double>(1, 2);
                    knots.SetValue(1, gc.FirstParameter);
                    knots.SetValue(2, gc.LastParameter);

                    FillParams(knots, gc.Degree, dfUf, dfUl, aParams);
                    break;
                }
                case BSplineCurve:
                {
                    BSplineCurve3D gc = c.BSpline();
                    FillParams(gc.InternalKnots(), gc.Degree, dfUf, dfUl, aParams);
                    break;
                }
                case Line:
                {
                    // Two points on a straight segment
                    aParams.Add(dfUf);
                    aParams.Add(dfUl);
                    break;
                }
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                default:
                {
                    // Four points on other analytical curves
                    if (c.GetType() is Circle or Ellipse or Hyperbola or Parabola)
                    {
                        iNbPoints = 4;
                    }
                    // Put some points on other curves
                    if (iNbPoints == 0)
                    {
                        iNbPoints = 15 + c.NbIntervals(Continuity.C3);
                    }

                    var bounds = new NCollection_Array1<double>(1, 2);
                    bounds.SetValue(1, dfUf);
                    bounds.SetValue(2, dfUl);

                    FillParams(bounds, iNbPoints - 1, dfUf, dfUl, aParams);
                    break;
                }
            }

            // Add the points with weights to the sequences
            FillPoints(c, aParams, points, weight);
        }

        if (points.Length() < 3)
        {
            return;
        }

        // ======================= Step #2
        _location = new Placement();
        int iPoint;
        var mat = new Matrix(1, 3, 1, 3, 0.0);
        var vec = new math_Vector(1, 3, 0.0);
        // Find the barycenter and normalize weights
        double dfMaxWeight = 0.0;
        var baryCenter = new Xyz(0.0, 0.0, 0.0);
        double dfSumWeight = 0.0;
        for (iPoint = 1; iPoint <= points.Length(); iPoint++)
        {
            double dfW = weight[iPoint];
            baryCenter += dfW * points[iPoint].XYZ();
            dfSumWeight += dfW;
            if (dfW > dfMaxWeight)
            {
                dfMaxWeight = dfW;
            }
        }
        baryCenter /= dfSumWeight;

        // Fill the matrix and the right vector
        for (iPoint = 1; iPoint <= points.Length(); iPoint++)
        {
            Xyz p = points[iPoint].XYZ() - baryCenter;
            double w = weight[iPoint] / dfMaxWeight;
            mat[1, 1] += w * p.X() * p.X();
            mat[1, 2] += w * p.X() * p.Y();
            mat[1, 3] += w * p.X() * p.Z();
            //
            mat[2, 2] += w * p.Y() * p.Y();
            mat[2, 3] += w * p.Y() * p.Z();
            //
            mat[3, 3] += w * p.Z() * p.Z();
        }
        mat[2, 1] = mat[1, 2];
        mat[3, 1] = mat[1, 3];
        mat[3, 2] = mat[2, 3];
        //
        var eignval = new Jacobi(mat);
        math_Vector eVals;
        bool isSolved = eignval.IsDone;
        int isol = 0;
        if (isSolved)
        {
            eVals = eignval.Values();
            // We need vector with eigenvalue ~ 0.
            double eMin = RealLast();
            double eMax = -eMin;
            for (i = 1; i <= 3; ++i)
            {
                double e = Math.Abs(eVals[i]);
                if (eMin > e)
                {
                    eMin = e;
                    isol = i;
                }
                if (eMax < e)
                {
                    eMax = e;
                }
            }

            if (isol == 0)
            {
                isSolved = false;
            }
            else
            {
                double eps = Epsilon(eMax);
                if (eMin <= eps)
                {
                    eignval.Vector(isol, vec);
                }
                else
                {
                    // try using vector product of other axes
                    Span<int> ind = [0, 0];
                    for (i = 1; i <= 3; ++i)
                    {
                        if (i == isol)
                        {
                            continue;
                        }
                        if (ind[0] == 0)
                        {
                            ind[0] = i;
                            continue;
                        }
                        if (ind[1] == 0)
                        {
                            ind[1] = i;
                            continue;
                        }
                    }
                    var vec1 = new math_Vector(1, 3, 0.0);
                    var vec2 = new math_Vector(1, 3, 0.0);
                    eignval.Vector(ind[0], vec1);
                    eignval.Vector(ind[1], vec2);
                    var v1 = new Vector3D(vec1[1], vec1[2], vec1[3]);
                    var v2 = new Vector3D(vec2[1], vec2[2], vec2[3]);
                    Vector3D n0 = v1 ^ v2;
                    vec[1] = n0.X;
                    vec[2] = n0.Y;
                    vec[3] = n0.Z;
                }
                if (vec.Norm2() < Constants.Resolution())
                {
                    isSolved = false;
                }
            }
        }

        if (!isSolved)
        {
            return;
        }
        // Removing very small values
        double maxV = Max(Math.Abs(vec[1]), Max(Math.Abs(vec[2]), Math.Abs(vec[3])));
        double eps2 = Epsilon(maxV);
        for (i = 1; i <= 3; ++i)
        {
            if (Math.Abs(vec[i]) <= eps2)
            {
                vec[i] = 0.0;
            }
        }
        var n = new Vector3D(vec[1], vec[2], vec[3]);
        var plane = new PlaneSurface(new Point3D(baryCenter), new Direction3D(n));
        _tolReached = Controle(points, plane);
        const double weakness = 5.0;
        if (_tolReached <= _tolerance || (tol < 0 && _tolReached < _tolerance * weakness))
        {
            _surface = plane;
            // If S is wire, try to orient surface according to orientation of wire.
            if (s.Type == ShapeType.Wire && s.ClosedFlag)
            {
                Wire aW = ShapeCasts.Wire(s);
                Face tmpFace = new LibraryMakeFace(_surface, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
                var bb = new BrepBuilder();
                bb.Add(tmpFace, aW);
                var fClass = new FClass2d(tmpFace, 0.0);
                if (fClass.PerformInfinitePoint() == In)
                {
                    Direction3D norm = plane.Position.Direction;
                    norm = norm.Reversed();
                    _surface = new PlaneSurface(plane.Position.Location, norm);
                }
            }
        }
    }

    /// <summary>True if a surface was found.</summary>
    public bool Found()
    {
        return _surface is not null;
    }

    /// <summary>The surface found (frozen), or null.</summary>
    public Surface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary><see cref="Surface"/> without freezing, for ported code that edits it (BRepFill_Sweep).</summary>
    internal Surface? InternalSurface() => _surface;

    /// <summary>The tolerance: the larger of the given one and the edges' largest.</summary>
    public double Tolerance()
    {
        return _tolerance;
    }

    /// <summary>The largest distance of the sampled points to the fitted plane (0 for an existing surface).</summary>
    public double ToleranceReached()
    {
        return _tolReached;
    }

    /// <summary>True if the surface is one of the edges' existing surfaces.</summary>
    public bool Existed()
    {
        return _isExisted;
    }

    /// <summary>The location of the surface (of an existing surface; identity for a fitted plane).</summary>
    public Placement Location()
    {
        return _location;
    }
}
