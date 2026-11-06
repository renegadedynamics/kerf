// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_NormalProjection.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_NormalProjection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Algo;

// OCCT_DEBUG and OCCT_DEBUG_CHRONO blocks (traces and chronometers) are left out.

/// <summary>
/// The normal projection of edges and wires on a shape: each edge projected along the normals of each face, as edges
/// with their pcurves on the faces, trimmed by the faces' bounds; punctual projections are vertices.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shapes it is given: every edge of the result is new (or a
/// section's). <see cref="Projection"/> and <see cref="Generated"/> are publication points and freeze what they
/// return; <see cref="InternalProjection"/> and <see cref="InternalGenerated"/> are the unfrozen forms for ported code.
/// </remarks>
internal sealed class NormalProjection
{
    private Shape _shape = new();
    private bool _isDone;
    private double _tol3d;
    private double _maxDist;
    private bool _with3d;
    private Continuity _continuity;
    private int _maxDegree;
    private int _maxSeg;
    private bool _faceBounds;
    private readonly Shape _toProj;
    private readonly Dictionary<Shape, Shape> _ancestorMap = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Shape> _corresp = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _descendants =
        new(ShapeHasher.Instance);
    private Shape _res = new();

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape to project on.</summary>
    public NormalProjection()
    {
        _isDone = false;
        _maxDist = -1.0;
        _with3d = true;
        _faceBounds = true;
        BrepBuilder bb = new();
        Compound toProj = new();
        bb.MakeCompound(ref toProj);
        _toProj = toProj;
        SetDefaultParams();
    }

    //=================================================================================================

    /// <summary>The projection on the shape <paramref name="s"/>.</summary>
    public NormalProjection(in Shape s) : this()
    {
        Init(s);
    }

    //=================================================================================================

    /// <summary>Sets the shape to project on.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
    }

    //=================================================================================================

    /// <summary>Adds an edge or a wire to project.</summary>
    public void Add(in Shape toProj)
    {
        BrepBuilder bb = new();
        bb.Add(_toProj, toProj);
    }

    //=================================================================================================

    /// <summary>
    /// Sets the parameters: the tolerances between the projected 3D curve and its 2D curve, the continuity of the
    /// approximation, its maximum degree and number of segments.
    /// </summary>
    public void SetParams(double tol3D, double tol2D, Continuity internalContinuity, int maxDegree, int maxSeg)
    {
        _tol3d = tol3D;
        _continuity = internalContinuity;
        _maxDegree = maxDegree;
        _maxSeg = maxSeg;
    }

    //=================================================================================================

    /// <summary>Sets the default parameters (1e-4, its power 2/3, C1, 14, 16).</summary>
    public void SetDefaultParams()
    {
        _tol3d = 1.0e-4;
        _continuity = C1;
        _maxDegree = 14;
        _maxSeg = 16;
    }

    //=================================================================================================

    /// <summary>Whether the projected edges are trimmed by the faces' bounds (true by default).</summary>
    public void SetLimit(bool faceBounds = true)
    {
        _faceBounds = faceBounds;
    }

    //=================================================================================================

    /// <summary>The maximum distance between the shape and what is projected (none if negative, the default).</summary>
    public void SetMaxDistance(double maxDist)
    {
        _maxDist = maxDist;
    }

    //=================================================================================================

    /// <summary>False keeps the initial 3D curves for the projected edges where the projection is close enough.</summary>
    public void Compute3d(bool with3d = true)
    {
        _with3d = with3d;
    }

    //=================================================================================================

    /// <summary>Builds the result as a compound.</summary>
    public void Build()
    {
        _isDone = false;

        NCollection_HSequence<Shape> edges = new();
        NCollection_HSequence<Shape> faces = new();
        NCollection_List<Shape> descenList = new();
        int nbEdges = 0, nbFaces = 0, i, j, k;
        Explorer expOfWire = new(), expOfShape = new();
        double udeb, ufin;
        Shape VertexRes;
        bool only3d, only2d, elementary;

        // for isoparametric cases
        NCollection_Array1<Point2D> poles = new(1, 2);
        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> mults = new(1, 2);
        int deg;
        deg = 1;
        mults[1] = deg + 1;
        mults[2] = deg + 1;
        //

        for (expOfWire.Init(_toProj, ShapeType.Edge); expOfWire.More(); expOfWire.Next(), nbEdges++)
        {
            edges.Append(expOfWire.Current());
        }

        for (expOfShape.Init(_shape, ShapeType.Face); expOfShape.More(); expOfShape.Next(), nbFaces++)
        {
            faces.Append(expOfShape.Current());
        }

        BrepBuilder bb = new();
        Compound res = new(), vertexRes = new();
        bb.MakeCompound(ref res);
        bb.MakeCompound(ref vertexRes);
        _res = res;
        VertexRes = vertexRes;
        bool yaVertexRes = false;

        for (i = 1; i <= nbEdges; i++)
        {
            descenList.Clear();
            EdgeCurveAdaptor hcur = new(ShapeCasts.Edge(edges.Value(i)));
            elementary = IsElementary(hcur);
            for (j = 1; j <= nbFaces; j++)
            {
                FaceSurfaceAdaptor hsur = new(ShapeCasts.Face(faces.Value(j)));

                // computation of  TolU and TolV

                double tolU, tolV;

                tolU = hsur.UResolution(_tol3d) / 20;
                tolV = hsur.VResolution(_tol3d) / 20;
                // Projection
                CompProjectedCurve hProjector = new(hsur, hcur, tolU, tolV, _maxDist);
                //
                Shape prj = new();
                bool degenerated = false;
                Point3D p;

                Curve2DAdaptorBase hpCur;
                Curve2D? pCur2d = null; // Only for isoparametric projection

                for (k = 1; k <= hProjector.CurveCount; k++)
                {
                    if (hProjector.IsSinglePnt(k, out Point2D p2d))
                    {
                        hProjector.GetSurface()!.D0(p2d.X, p2d.Y, out p);
                        prj = new LibraryMakeVertex(p).InternalShape();
                        descenList.Append(prj);
                        bb.Add(VertexRes, prj);
                        yaVertexRes = true;

                        _ancestorMap[prj] = edges.Value(i);
                    }
                    else
                    {
                        only2d = only3d = false;
                        hProjector.Bounds(k, out udeb, out ufin);

                        /**************************************************************/
                        if (hProjector.IsUIso(k, out double uIso))
                        {
                            hProjector.D0(udeb, out Point2D pdeb);
                            hProjector.D0(ufin, out Point2D pfin);
                            poles[1] = pdeb;
                            poles[2] = pfin;
                            knots[1] = udeb;
                            knots[2] = ufin;
                            BSplineCurve2D bs2d = new(poles, knots, mults, deg);
                            pCur2d = new TrimmedCurve2D(bs2d, udeb, ufin);
                            hpCur = new CurveAdaptor2D(pCur2d);
                            only3d = true;
                        }
                        else if (hProjector.IsVIso(k, out double vIso))
                        {
                            hProjector.D0(udeb, out Point2D pdeb);
                            hProjector.D0(ufin, out Point2D pfin);
                            poles[1] = pdeb;
                            poles[2] = pfin;
                            knots[1] = udeb;
                            knots[2] = ufin;
                            BSplineCurve2D bs2d = new(poles, knots, mults, deg);
                            pCur2d = new TrimmedCurve2D(bs2d, udeb, ufin);
                            hpCur = new CurveAdaptor2D(pCur2d);
                            only3d = true;
                        }
                        else
                        {
                            hpCur = hProjector;
                        }

                        if ((!_with3d || elementary) && (hProjector.MaxDistance(k) <= _tol3d))
                        {
                            only2d = true;
                        }

                        if (only2d && only3d)
                        {
                            LibraryMakeEdge mKed = new(global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(hcur), udeb, ufin);
                            prj = mKed.InternalEdge();
                            bb.UpdateEdge(ShapeCasts.Edge(prj), pCur2d, ShapeCasts.Face(faces.Value(j)), _tol3d);
                            bb.UpdateVertex(Exploration.FirstVertex(ShapeCasts.Edge(prj)), _tol3d);
                            bb.UpdateVertex(Exploration.LastVertex(ShapeCasts.Edge(prj)), _tol3d);
                        }
                        else
                        {
                            CurveOnSurfaceApproximation appr = new(hpCur, hsur, udeb, ufin, _tol3d);
                            appr.Perform(_maxSeg, _maxDegree, _continuity, only3d, only2d);

                            if (appr.MaxError3d() > 1.0e3 * _tol3d)
                            {
                                continue;
                            }

                            if (!only3d)
                            {
                                pCur2d = appr.Curve2d();
                            }
                            if (only2d)
                            {
                                LibraryMakeEdge mKed = new(global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(hcur), udeb, ufin);
                                prj = mKed.InternalEdge();
                            }
                            else
                            {
                                // It is tested if the solution is not degenerated to set the
                                // flag on edge, one takes several points, checks if the cloud of
                                // points has less diameter than the tolerance 3D
                                degenerated = true;
                                double dist;
                                BSplineCurve3D bs3d = appr.Curve3d()!;
                                Point3D p1 = new(0.0, 0.0, 0.0), pp; // skl : I change "P" to "PP"
                                int nbPoint, ii; // skl : I change "i" to "ii"
                                double par, dPar;
                                // start from 3 points to reject non degenerated edges
                                // very fast
                                nbPoint = 3;
                                dPar = (bs3d.LastParameter - bs3d.FirstParameter) / (nbPoint - 1);
                                for (ii = 0; ii < nbPoint; ii++)
                                {
                                    par = bs3d.FirstParameter + ii * dPar;
                                    pp = bs3d.Value(par);
                                    p1 = p1.WithXYZ(p1.XYZ() + pp.XYZ() / nbPoint);
                                }
                                for (ii = 0; ii < nbPoint && degenerated; ii++)
                                {
                                    par = bs3d.FirstParameter + ii * dPar;
                                    pp = bs3d.Value(par);
                                    dist = p1.DistanceTo(pp);
                                    if (dist > _tol3d)
                                    {
                                        degenerated = false;
                                        break;
                                    }
                                }
                                // if the test passes a more exact test with 10 points
                                if (degenerated)
                                {
                                    p1 = p1.WithCoord(0.0, 0.0, 0.0);
                                    nbPoint = 10;
                                    dPar = (bs3d.LastParameter - bs3d.FirstParameter) / (nbPoint - 1);
                                    for (ii = 0; ii < nbPoint; ii++)
                                    {
                                        par = bs3d.FirstParameter + ii * dPar;
                                        pp = bs3d.Value(par);
                                        p1 = p1.WithXYZ(p1.XYZ() + pp.XYZ() / nbPoint);
                                    }
                                    for (ii = 0; ii < nbPoint && degenerated; ii++)
                                    {
                                        par = bs3d.FirstParameter + ii * dPar;
                                        pp = bs3d.Value(par);
                                        dist = p1.DistanceTo(pp);
                                        if (dist > _tol3d)
                                        {
                                            degenerated = false;
                                            break;
                                        }
                                    }
                                }
                                if (degenerated)
                                {
                                    Vertex vv = new();
                                    bb.MakeVertex(ref vv);
                                    bb.UpdateVertex(vv, p1, _tol3d);
                                    Edge aPrj = new();
                                    bb.MakeEdge(ref aPrj);
                                    bb.Add(aPrj, vv.Oriented(Forward));
                                    bb.Add(aPrj, vv.Oriented(Reversed));
                                    bb.Degenerated(aPrj, true);
                                    prj = aPrj;
                                }
                                else
                                {
                                    prj = new LibraryMakeEdge(bs3d).InternalEdge();
                                }
                            }

                            bb.UpdateEdge(ShapeCasts.Edge(prj), pCur2d, ShapeCasts.Face(faces.Value(j)), appr.MaxError3d());
                            bb.UpdateVertex(Exploration.FirstVertex(ShapeCasts.Edge(prj)), appr.MaxError3d());
                            bb.UpdateVertex(Exploration.LastVertex(ShapeCasts.Edge(prj)), appr.MaxError3d());
                            if (degenerated)
                            {
                                bb.Range(ShapeCasts.Edge(prj), ShapeCasts.Face(faces.Value(j)), udeb, ufin);
                            }
                        }

                        if (_faceBounds)
                        {
                            // Trimming edges by face bounds
                            // if the solution is degenerated, use of BoolTool is avoided
                            if (!degenerated)
                            {
                                // Perform Boolean COMMON operation to get parts of projected edge
                                // inside the face
                                SectionOperation section = new(faces.Value(j), prj);
                                if (section.IsDone)
                                {
                                    Shape rc = section.InternalShape();
                                    //
                                    Explorer expE = new(rc, ShapeType.Edge);
                                    for (; expE.More(); expE.Next())
                                    {
                                        Shape e = expE.Current();
                                        bb.Add(_res, e);
                                        _ancestorMap[e] = edges.Value(i);
                                        _corresp[e] = faces.Value(j);
                                    }
                                }
                                else
                                {
                                    // if the common operation has failed, try to classify the part
                                    FClass2d classifier = new(ShapeCasts.Face(faces.Value(j)), global::Kerf.Engine.Foundation.Precision.Confusion());
                                    double f = pCur2d!.FirstParameter;
                                    double l = pCur2d.LastParameter;
                                    double pmil = (f + l) / 2;
                                    pCur2d.D0(pmil, out Point2D puv);
                                    Containment state;
                                    state = classifier.Perform(puv);
                                    if (state == In || state == On)
                                    {
                                        bb.Add(_res, prj);
                                        descenList.Append(prj);
                                        _ancestorMap[prj] = edges.Value(i);
                                        _corresp[prj] = faces.Value(j);
                                    }
                                }
                            }
                            else
                            {
                                FClass2d classifier = new(ShapeCasts.Face(faces.Value(j)), global::Kerf.Engine.Foundation.Precision.Confusion());
                                double f = pCur2d!.FirstParameter;
                                double l = pCur2d.LastParameter;
                                double pmil = (f + l) / 2;
                                pCur2d.D0(pmil, out Point2D puv);
                                Containment state;
                                state = classifier.Perform(puv);
                                if (state == In || state == On)
                                {
                                    bb.Add(_res, prj);
                                    descenList.Append(prj);
                                    _ancestorMap[prj] = edges.Value(i);
                                    _corresp[prj] = faces.Value(j);
                                }
                            }
                        }
                        else
                        {
                            bb.Add(_res, prj);
                            descenList.Append(prj);
                            _ancestorMap[prj] = edges.Value(i);
                            _corresp[prj] = faces.Value(j);
                        }
                    }
                }
            }
            // OCCT binds the list by value (a copy), then clears it for the next edge.
            _descendants[edges.Value(i)] = new NCollection_List<Shape>(descenList);
        }
        // JPI : eventual wire creation is reported in a specific method
        //       BuilWire that can be called by the user. Otherwise, the
        //       relations of map myAncestorMap, myCorresp will be lost.

        if (yaVertexRes)
        {
            bb.Add(_res, VertexRes);
        }

        _isDone = true;
    }

    //=================================================================================================

    /// <summary>True if <see cref="Build"/> was run.</summary>
    public bool IsDone => _isDone;

    //=================================================================================================

    /// <summary>The result, a compound of edges (and a compound of vertices), frozen.</summary>
    public Shape Projection()
    {
        _res.TShape()?.FreezeGraph();
        return _res;
    }

    /// <summary><see cref="Projection"/> without freezing, for ported code.</summary>
    internal Shape InternalProjection() => _res;

    //=================================================================================================

    /// <summary>The edge or wire's edge projected into the edge (or vertex) <paramref name="e"/> of the result.</summary>
    /// <exception cref="NoSuchObjectException">E isn't in the result.</exception>
    public Shape Ancestor(in Edge e) => _ancestorMap.GetValue(e);

    //=================================================================================================

    /// <summary>The face on which the edge <paramref name="e"/> of the result lies.</summary>
    /// <exception cref="NoSuchObjectException">E isn't in the result.</exception>
    public Shape Couple(in Edge e) => _corresp.GetValue(e);

    //=================================================================================================

    /// <summary>The shapes of the result generated from the edge <paramref name="s"/>, frozen.</summary>
    /// <exception cref="NoSuchObjectException">S wasn't projected.</exception>
    public IReadOnlyList<Shape> Generated(in Shape s)
    {
        NCollection_List<Shape> list = InternalGenerated(s);
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return ImmutableArrays.Own<Shape>(shapes);
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape s) => _descendants.GetValue(s);

    //=================================================================================================

    /// <summary>True if the curve is a line or a conic.</summary>
    public bool IsElementary(CurveAdaptorBase c)
    {
        CurveKind type;
        type = c.GetType();
        switch (type)
        {
            case Line:
            case Circle:
            case Ellipse:
            case Hyperbola:
            case Parabola:
                return true;
            default:
                return false;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Builds the result as a wire if its edges make one, appended to <paramref name="listOfWire"/>; false if they
    /// make several wires or there is no edge.
    /// </summary>
    public bool BuildWire(NCollection_List<Shape> listOfWire)
    {
        Explorer expOfShape = new();
        bool isWire = false;
        expOfShape.Init(_res, ShapeType.Edge);
        if (expOfShape.More())
        {
            NCollection_List<Shape> list = new();

            for (; expOfShape.More(); expOfShape.Next())
            {
                Shape curE = expOfShape.Current();
                list.Append(curE);
            }
            LibraryMakeWire mw = new();
            mw.Add(list);
            if (mw.IsDone)
            {
                Shape wire = mw.InternalShape();
                // If the resulting wire contains the same edge as at the beginning OK
                // otherwise the result really consists of several wires.
                Explorer exp2 = new(wire, ShapeType.Edge);
                int nbEdges = 0;
                for (; exp2.More(); exp2.Next())
                {
                    nbEdges++;
                }
                if (nbEdges == list.Extent())
                {
                    listOfWire.Append(wire);
                    isWire = true;
                }
            }
        }
        return isWire;
    }
}
