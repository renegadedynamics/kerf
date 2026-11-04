// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// N-sided filling: a face bounded by edges (each possibly on a support face it must meet with G0, G1 or G2
/// continuity), passing through further curve and point constraints, built on a plate surface
/// (GeomPlate_BuildPlateSurface) approximated by a B-spline surface.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: constraints are added, then <see cref="Build"/>. It never writes
/// into the caller's shapes: the result's edges are new (empty copies of the boundary edges) on new vertices.
/// <see cref="Face"/> freezes the face it returns, and <see cref="Generated"/> the shapes it lists; ported code
/// reads <see cref="InternalFace"/> and <see cref="InternalGenerated"/>. As in OCCT, each <see cref="Build"/>
/// starts a new plate from the constraints, but keeps what earlier builds recorded: a boundary edge is then rebuilt
/// from the previous build's edge, and the bounds found for free constraints stay in the boundary.
/// </remarks>
internal class SurfaceFilling
{
    private BuildPlateSurface? _builder;
    private readonly NCollection_Sequence<EdgeFaceAndOrder> _boundary = new();
    private readonly NCollection_Sequence<EdgeFaceAndOrder> _constraints = new();
    private readonly NCollection_Sequence<FaceAndOrder> _freeConstraints = new();
    private readonly NCollection_Sequence<PointConstraint> _points = new();
    private readonly Dictionary<Shape, Shape> _oldNewMap = new(ShapeHasher.Instance);
    // OCCT.NET: frozen. The boundary's vertices BuildWires' MakeWire replaced by its copies for write (see BuildWires).
    private readonly Dictionary<Shape, Shape> _vertexCopies = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _generated = new();
    private Face _face = new();
    private Face _initFace = new();
    private double _tol2d;
    private double _tol3d;
    private double _tolAng;
    private double _tolCurv;
    private int _maxDeg;
    private int _maxSegments;
    private int _degree;
    private int _nbPtsOnCur;
    private int _nbIter;
    private bool _anisotropie;
    private bool _isInitFaceGiven;
    private bool _isDone;

    private static Vector3D MakeFinVec(in Wire aWire, in Vertex aVertex)
    {
        Vertex vfirst, vlast, origin = new Vertex();
        WireExplorer explo = new WireExplorer(aWire);
        for (; explo.More(); explo.Next())
        {
            Exploration.Vertices(explo.Current(), out vfirst, out vlast);
            if (vfirst.IsSame(aVertex))
            {
                origin = vlast;
                break;
            }
            if (vlast.IsSame(aVertex))
            {
                origin = vfirst;
                break;
            }
        }
        return new Vector3D(Tool.Pnt(origin), Tool.Pnt(aVertex));
    }

    private static Wire WireFromList(NCollection_List<Shape> edges)
    {
        BrepBuilder bb = new BrepBuilder();
        Wire wire = new Wire();
        bb.MakeWire(ref wire);
        Edge edge = ShapeCasts.Edge(edges.First());
        bb.Add(wire, edge);
        edges.RemoveFirst();

        Vertex v1, v2;
        Exploration.Vertices(edge, out v1, out v2, true); // with orientation

        while (!edges.IsEmpty())
        {
            NCollection_List<Shape>.Iterator itl = new NCollection_List<Shape>.Iterator(edges);
            for (; itl.More(); itl.Next())
            {
                edge = ShapeCasts.Edge(itl.Value());
                Vertex v3, v4;
                Exploration.Vertices(edge, out v3, out v4, true); // with orientation
                if (v1.IsSame(v3) || v1.IsSame(v4) || v2.IsSame(v3) || v2.IsSame(v4))
                {
                    if (v1.IsSame(v3))
                    {
                        edge.Reverse();
                        v1 = v4;
                    }
                    else if (v1.IsSame(v4))
                    {
                        v1 = v3;
                    }
                    else if (v2.IsSame(v3))
                    {
                        v2 = v4;
                    }
                    else
                    {
                        edge.Reverse();
                        v2 = v3;
                    }
                    break;
                }
            }
            if (!itl.More())
            {
                global::Kerf.Engine.Foundation.Message.SendWarning("Warning: WireFromList: can't find the next edge. The wire is not "
                                    + "complete, some edges are lost.");
                break;
            }
            bb.Add(wire, edge);
            edges.Remove(itl);
        }

        wire.Closed(true);
        return wire;
    }

    public SurfaceFilling(int degree = 3, int nbPtsOnCur = 15, int nbIter = 2, bool anisotropie = false,
                            double tol2d = 0.00001, double tol3d = 0.0001, double tolAng = 0.01, double tolCurv = 0.1,
                            int maxDeg = 8, int maxSegments = 9)
    {
        _degree = degree;
        _nbPtsOnCur = nbPtsOnCur;
        _nbIter = nbIter;
        _anisotropie = anisotropie;

        _tol2d = tol2d;
        _tol3d = tol3d;
        _tolAng = tolAng;
        _tolCurv = tolCurv;

        _maxDeg = maxDeg;
        _maxSegments = maxSegments;

        _isInitFaceGiven = false;

        _isDone = false;
    }

    /// <summary>Sets the tolerances of the constraints.</summary>
    public void SetConstrParam(double tol2d = 0.00001, double tol3d = 0.0001, double tolAng = 0.01, double tolCurv = 0.1)
    {
        _tol2d = tol2d;
        _tol3d = tol3d;
        _tolAng = tolAng;
        _tolCurv = tolCurv;
    }

    /// <summary>Sets the parameters of the plate's resolution.</summary>
    public void SetResolParam(int degree = 3, int nbPtsOnCur = 15, int nbIter = 2, bool anisotropie = false)
    {
        _degree = degree;
        _nbPtsOnCur = nbPtsOnCur;
        _nbIter = nbIter;
        _anisotropie = anisotropie;
    }

    /// <summary>Sets the parameters of the approximation of the plate surface.</summary>
    public void SetApproxParam(int maxDeg = 8, int maxSegments = 9)
    {
        _maxDeg = maxDeg;
        _maxSegments = maxSegments;
    }

    /// <summary>Loads the face whose surface the filling starts from.</summary>
    public void LoadInitSurface(in Face aFace)
    {
        _initFace = aFace;
        _isInitFaceGiven = true;
    }

    /// <summary>Adds a boundary edge (or, with <paramref name="isBound"/> false, a free edge) of order G0 (no face).</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge anEdge, Continuity order, bool isBound = true)
    {
        Face nullFace = new Face();
        EdgeFaceAndOrder edgeFaceAndOrder = new EdgeFaceAndOrder(anEdge, nullFace, order);
        if (isBound)
        {
            _boundary.Append(edgeFaceAndOrder);
            return _boundary.Length();
        }
        else
        {
            _constraints.Append(edgeFaceAndOrder);
            return (_boundary.Length() + _freeConstraints.Length() + _constraints.Length());
        }
    }

    /// <summary>Adds an edge with its support face as a constraint of order <paramref name="order"/>.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge anEdge, in Face support, Continuity order, bool isBound = true)
    {
        EdgeFaceAndOrder edgeFaceAndOrder = new EdgeFaceAndOrder(anEdge, support, order);
        if (isBound)
        {
            _boundary.Append(edgeFaceAndOrder);
            return _boundary.Length();
        }
        else
        {
            _constraints.Append(edgeFaceAndOrder);
            return (_boundary.Length() + _freeConstraints.Length() + _constraints.Length());
        }
    }

    /// <summary>Adds a "free constraint": a face that bounds the filling where its boundary has a gap.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Face support, Continuity order)
    {
        FaceAndOrder faceAndOrder = new FaceAndOrder(support, order);
        _freeConstraints.Append(faceAndOrder);
        return (_boundary.Length() + _freeConstraints.Length());
    }

    /// <summary>Adds a point the filling passes through.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Point3D point)
    {
        PointConstraint pc = new PointConstraint(point, (int)C0, _tol3d);
        _points.Append(pc);
        return (_boundary.Length() + _freeConstraints.Length() + _constraints.Length() + _points.Length());
    }

    /// <summary>Adds a point constraint on a face, of order <paramref name="order"/>.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(double u, double v, in Face support, Continuity order)
    {
        FaceSurfaceAdaptor hSurf = new FaceSurfaceAdaptor();
        hSurf.Initialize(support);
        PointConstraint pc = new PointConstraint(u, v, Tool.Surface(hSurf.Face())!, PlateOrder(order),
                                                                      _tol3d, _tolAng, _tolCurv);
        _points.Append(pc);
        return (_boundary.Length() + _freeConstraints.Length() + _constraints.Length() + _points.Length());
    }

    // OCCT.NET: OCCT's sweeps of an edge (MakePrism, MakeRevol, ...) write the new face's pcurve into the caller's edge,
    // which the face shares; the port sweeps a frozen edge as a copy (decision 468), so the caller's edge has no pcurve
    // on the face the sweep made, and the constraint would read a null pcurve (OCCT's comment below). For a frozen edge
    // without a pcurve on the support, the support's edge on the same 3D curve, location and range stands for it, as it
    // is OCCT's edge itself (decision 1103).
    private static Edge EdgeOnSupport(in Edge edge, in Face support)
    {
        if (!edge.TShape()!.IsFrozen || Tool.CurveOnSurface(edge, support, out _, out _) is not null)
        {
            return edge;
        }
        Curve3D? curve = Tool.Curve(edge, out Placement loc, out double first, out double last);
        if (curve is null)
        {
            return edge;
        }
        for (Explorer exp = new(support, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(exp.Current());
            Curve3D? c = Tool.Curve(anEdge, out Placement l, out double f, out double la);
            if (ReferenceEquals(c, curve) && l.IsEqual(loc) && f == first && la == last)
            {
                return ShapeCasts.Edge(anEdge.Oriented(edge.Orientation));
            }
        }
        return edge;
    }

    // The plate's order of a continuity: 0 for C0, 1 for G1 and C1, 2 for G2 and above. OCCT passed the enum's value,
    // so C1 (2) gave a G2 constraint and G2 (3) threw (decision 1434).
    private static int PlateOrder(Continuity order) => order switch
    {
        C0 => 0,
        G1 or C1 => 1,
        _ => 2,
    };

    private void AddConstraints(NCollection_Sequence<EdgeFaceAndOrder> seqOfConstraints)
    {
        Edge curEdge;
        Face curFace;
        Continuity curOrder;

        CurveConstraint constr;
        int i;
        for (i = 1; i <= seqOfConstraints.Length(); i++)
        {
            curEdge = seqOfConstraints.Value(i)._edge;
            curFace = seqOfConstraints.Value(i)._face;
            curOrder = seqOfConstraints.Value(i)._order;

            if (curFace.IsNull)
            {
                if (curOrder == C0)
                {
                    EdgeCurveAdaptor HCurve = new EdgeCurveAdaptor();
                    HCurve.Initialize(curEdge);
                    CurveAdaptorBase hCurve = HCurve; // to avoid ambiguity
                    constr = new FillingCurveConstraint(hCurve, PlateOrder(curOrder), _nbPtsOnCur, _tol3d);
                }
                else
                { // Pas de representation Topologique
                  // On prend une representation Geometrique : au pif !
                    Surface? surface;
                    Curve2D? c2d;
                    Placement loc;
                    double f, l;
                    Tool.CurveOnSurface(curEdge, out c2d, out surface, out loc, out f, out l);
                    if (surface is null)
                    {
                        throw new FailureException("Add");
                    }
                    surface = (Surface)surface.Copy();
                    surface.Transform(loc.Transformation);
                    SurfaceAdaptor surf = new SurfaceAdaptor(surface);
                    CurveAdaptor2D curve2d = new CurveAdaptor2D(c2d!);

                    CurveOnSurface curvOnSurf = new CurveOnSurface(curve2d, surf);
                    CurveOnSurface hCurvOnSurf = curvOnSurf;

                    constr = new CurveConstraint(hCurvOnSurf, PlateOrder(curOrder), _nbPtsOnCur, _tol3d, _tolAng,
                                                           _tolCurv);
                }
            }
            else
            {
                FaceSurfaceAdaptor surf = new FaceSurfaceAdaptor();
                surf.Initialize(curFace);
                EdgeCurve2DAdaptor curve2d = new EdgeCurve2DAdaptor();
                curve2d.Initialize(EdgeOnSupport(curEdge, curFace), curFace);
                // If CurEdge has no 2d representation on CurFace,
                // there will be exception "Attempt to access to null object"
                // in this initialization (null pcurve).
                CurveOnSurface curvOnSurf = new CurveOnSurface(curve2d, surf);
                CurveOnSurface hCurvOnSurf = curvOnSurf;

                constr = new FillingCurveConstraint(hCurvOnSurf, PlateOrder(curOrder), _nbPtsOnCur, _tol3d, _tolAng,
                                                      _tolCurv);
            }
            if (_isInitFaceGiven)
            {
                Curve2D? curve2d;
                double firstPar, lastPar;
                curve2d = Tool.CurveOnSurface(curEdge, _initFace, out firstPar, out lastPar);
                if (curve2d is not null)
                {
                    curve2d = new TrimmedCurve2D(curve2d, firstPar, lastPar);
                    constr.SetCurve2dOnSurf(curve2d);
                }
            }
            _builder!.Add(constr);
        }
    }

    private void BuildWires(NCollection_List<Shape> edgeList, NCollection_List<Shape> wireList)
    {
        NCollection_List<Shape>.Iterator itl = new NCollection_List<Shape>.Iterator();
        int i, j;

        while (!edgeList.IsEmpty())
        {
            LibraryMakeWire mw = new LibraryMakeWire();
            Edge firstEdge = ShapeCasts.Edge(edgeList.First());
            mw.Add(firstEdge);
            var added = new List<(Edge, Edge)> { (firstEdge, firstEdge) }; // OCCT.NET: frozen
            edgeList.RemoveFirst();
            Vertex[] V_wire = new Vertex[2], V_edge = new Vertex[2];

            for (; ; )
            {
                Wire curWire = mw.InternalWire();
                Exploration.Vertices(curWire, out V_wire[0], out V_wire[1]);
                bool found = false;
                for (itl.Initialize(edgeList); itl.More(); itl.Next())
                {
                    Edge curEdge = ShapeCasts.Edge(itl.Value());
                    Exploration.Vertices(curEdge, out V_edge[0], out V_edge[1]);
                    for (i = 0; i < 2; i++)
                    {
                        for (j = 0; j < 2; j++)
                        {
                            if (V_wire[i].IsSame(V_edge[j]))
                            {
                                mw.Add(curEdge);
                                added.Add((curEdge, curEdge));
                                edgeList.Remove(itl);
                                found = true;
                                break;
                            }
                        }
                        if (found)
                        {
                            break;
                        }
                    }
                    if (found)
                    {
                        break;
                    }
                }
                if (!found) // try to find geometric coincidence
                {
                    Point3D[] P_wire = new Point3D[2];
                    P_wire[0] = Tool.Pnt(V_wire[0]);
                    P_wire[1] = Tool.Pnt(V_wire[1]);
                    for (itl.Initialize(edgeList); itl.More(); itl.Next())
                    {
                        Edge curEdge = ShapeCasts.Edge(itl.Value());
                        Exploration.Vertices(curEdge, out V_edge[0], out V_edge[1]);
                        for (i = 0; i < 2; i++)
                        {
                            for (j = 0; j < 2; j++)
                            {
                                double dist = P_wire[i].DistanceTo(Tool.Pnt(V_edge[j]));
                                if (dist < Tool.Tolerance(V_wire[i]) && dist < Tool.Tolerance(V_edge[j]))
                                {
                                    mw.Add(curEdge);
                                    Edge newEdge = mw.InternalEdge();
                                    _oldNewMap[curEdge.Oriented(Forward)] = newEdge.Oriented(Forward);
                                    added.Add((curEdge, newEdge));
                                    edgeList.Remove(itl);
                                    found = true;
                                    break;
                                }
                            }
                            if (found)
                            {
                                break;
                            }
                        }
                        if (found)
                        {
                            break;
                        }
                    }
                }
                if (!found) // end of current wire, begin next wire
                {
                    wireList.Append(mw.InternalWire());
                    BindCopiesForWrite(mw, added);
                    break;
                }
            } // end of for (;;)
        } // end of while (! EdgeList.IsEmpty())
    }

    // OCCT.NET: frozen. Merging an edge's vertex onto a wire vertex that must move or grow, OCCT's MakeWire changes that
    // vertex in place, so the edges added before it (the boundary's own edges, or the copies bound in myOldNewMap) end at
    // the moved vertex, and the final edges share it. The port's MakeWire goes on with a copy for write of the wire built
    // so far when the vertex is frozen (BRepLib_MakeWire's remarks): each boundary edge is bound here to the edge
    // standing for it in the finished wire, and each of its vertices that was copied is recorded for Generated.
    private void BindCopiesForWrite(LibraryMakeWire mw, List<(Edge, Edge)> added)
    {
        History? aHistory = mw.InternalHistory();
        if (aHistory is null)
        {
            return;
        }

        static Shape Final(History history, Shape s)
        {
            for (int guard = 0; guard < 100; guard++)
            {
                IReadOnlyList<Shape> modified = history.Modified(s);
                if (modified.Count == 0 || modified[0].IsSame(s))
                {
                    break;
                }
                s = modified[0].Oriented(s.Orientation);
            }
            return s;
        }

        foreach ((Edge original, Edge inWire) in added)
        {
            Shape final = Final(aHistory, inWire.Oriented(Forward));
            if (!final.IsSame(inWire))
            {
                _oldNewMap[original.Oriented(Forward)] = final;
            }
            for (var it = new Iterator(original.Oriented(Forward)); it.More(); it.Next())
            {
                Shape v = it.Value().Oriented(Forward);
                Shape finalV = Final(aHistory, v);
                if (!finalV.IsSame(v))
                {
                    _vertexCopies[v] = finalV;
                }
            }
        }
    }

    private void FindExtremitiesOfHoles(NCollection_List<Shape> wireList, NCollection_Sequence<Shape> verSeq)
    {
        NCollection_Sequence<Shape> wireSeq = new NCollection_Sequence<Shape>();
        NCollection_List<Shape>.Iterator itl = new NCollection_List<Shape>.Iterator(wireList);
        for (; itl.More(); itl.Next())
        {
            wireSeq.Append(itl.Value());
        }

        Wire theWire;
        theWire = ShapeCasts.Wire(wireSeq.Value(1));
        wireSeq.Remove(1);

        if (Tool.IsClosed(theWire))
        {
            return;
        }

        Vertex vfirst, vlast;
        Exploration.Vertices(theWire, out vfirst, out vlast);

        if (vfirst.IsSame(vlast))
        {
            // The Wire is closed indeed despite its
            // being not detected earlier.

            return;
        }

        Vector3D finVec = MakeFinVec(theWire, vlast);
        Vertex theVertex = vlast;
        verSeq.Append(vlast);

        while (!wireSeq.IsEmpty())
        {
            Vertex minVtx = new Vertex();
            int i, minInd = 1;
            bool isLast = false;
            double minAngle = M_PI;

            for (i = 1; i <= wireSeq.Length(); i++)
            {
                Wire curWire = ShapeCasts.Wire(wireSeq.Value(i));
                Exploration.Vertices(curWire, out vfirst, out vlast);

                double angle = finVec.AngleTo(new Vector3D(Tool.Pnt(theVertex), Tool.Pnt(vfirst)));
                if (angle < minAngle)
                {
                    minAngle = angle;
                    minVtx = vfirst;
                    minInd = i;
                    isLast = true;
                }
                angle = finVec.AngleTo(new Vector3D(Tool.Pnt(theVertex), Tool.Pnt(vlast)));
                if (angle < minAngle)
                {
                    minAngle = angle;
                    minVtx = vlast;
                    minInd = i;
                    isLast = false;
                }
            }
            verSeq.Append(minVtx);
            Wire minWire = ShapeCasts.Wire(wireSeq.Value(minInd));
            Exploration.Vertices(minWire, out vfirst, out vlast);
            if (isLast)
            {
                finVec = MakeFinVec(minWire, vlast);
                theVertex = vlast;
            }
            else
            {
                finVec = MakeFinVec(minWire, vfirst);
                theVertex = vfirst;
            }
            verSeq.Append(theVertex);
            wireSeq.Remove(minInd);
        }
        Exploration.Vertices(theWire, out vfirst, out vlast);
        verSeq.Append(vfirst);
    }

    /// <summary>Builds the filling.</summary>
    public void Build()
    {
        _builder = new BuildPlateSurface(_degree, _nbPtsOnCur, _nbIter, _tol2d, _tol3d, _tolAng, _tolCurv,
                                                    _anisotropie);

        if (_boundary.IsEmpty())
        {
            _isDone = false;
            return;
        }

        Face curFace;
        int i, j;

        // Creating array of vertices: extremities of wires
        NCollection_Sequence<Shape> verSeq = new NCollection_Sequence<Shape>();

        // Building missing bounds
        NCollection_List<Shape> edgeList = new NCollection_List<Shape>(),
                                       wireList = new NCollection_List<Shape>();
        for (i = 1; i <= _boundary.Length(); i++)
        {
            edgeList.Append(_boundary.Value(i)._edge);
        }

        BuildWires(edgeList, wireList);
        FindExtremitiesOfHoles(wireList, verSeq);

        // Searching for surfaces for missing bounds
        for (j = 1; j <= _freeConstraints.Length(); j++)
        {
            ProjectPointOnSurface projector = new ProjectPointOnSurface();
            double u1, v1, u2, v2;

            curFace = _freeConstraints.Value(j)._face;
            FaceSurfaceAdaptor hSurf = new FaceSurfaceAdaptor();
            hSurf.Initialize(curFace);
            Surface curSurface = Tool.Surface(hSurf.Face())!;

            for (i = 1; i <= verSeq.Length(); i += 2)
            {
                Vertex firstVtx = ShapeCasts.Vertex(verSeq.Value(i));
                Vertex lastVtx = ShapeCasts.Vertex(verSeq.Value(i + 1));

                Point3D firstPnt = Tool.Pnt(firstVtx);
                projector.Init(firstPnt, curSurface);
                if (projector.LowerDistance() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                projector.LowerDistanceParameters(out u1, out v1);

                Point3D lastPnt = Tool.Pnt(lastVtx);
                projector.Init(lastPnt, curSurface);
                if (projector.LowerDistance() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                projector.LowerDistanceParameters(out u2, out v2);

                // Making the constraint
                NCollection_Array1<Point2D> points = new NCollection_Array1<Point2D>(1, 2);
                points[1] = new Point2D(u1, v1);
                points[2] = new Point2D(u2, v2);
                BezierCurve2D line2d = new BezierCurve2D(points);
                Edge e = new LibraryMakeEdge(line2d, curSurface, firstVtx, lastVtx).InternalEdge();
                Add(e, curFace, _freeConstraints.Value(j)._order);
                verSeq.Remove(i, i + 1);
                break;
            } // for (i = 1; i <= VerSeq.Length(); i += 2)
        } // for (j = 1; j <= myFreeConstraints.Length(); j++)

        // Load initial surface to myBuilder if it is given
        if (_isInitFaceGiven)
        {
            FaceSurfaceAdaptor hSurfInit = new FaceSurfaceAdaptor();
            hSurfInit.Initialize(_initFace);
            _builder.LoadInitSurface(Tool.Surface(hSurfInit.Face())!);
        }

        // Adding constraints to myBuilder
        AddConstraints(_boundary);
        _builder.SetNbBounds(_boundary.Length());
        AddConstraints(_constraints);
        for (i = 1; i <= _points.Length(); i++)
        {
            _builder.Add(_points.Value(i));
        }

        _builder.Perform();
        if (_builder.IsDone)
        {
            _isDone = true;
        }
        else
        {
            _isDone = false;
            return;
        }

        PlateSurface gPlate = _builder.Surface()!;
        BSplineSurface surface;
        // Approximation
        double dmax = 1.1 * _builder.G0Error(); //???????????
        if (!_isInitFaceGiven)
        {
            double seuil; //?????

            NCollection_Sequence<Xy> s2d = new NCollection_Sequence<Xy>();
            NCollection_Sequence<Xyz> s3d = new NCollection_Sequence<Xyz>();
            _builder.Disc2dContour(4, s2d);
            _builder.Disc3dContour(4, 0, s3d);
            seuil = Max(_tol3d, 10 * _builder.G0Error()); //????????
            PlateG0Criterion criterion = new PlateG0Criterion(s2d, s3d, seuil);
            MakeApprox approx = new MakeApprox(gPlate, criterion, _tol3d, _maxSegments, _maxDeg);
            surface = approx.Surface()!;
        }
        else
        {
            MakeApprox approx = new MakeApprox(gPlate, _tol3d, _maxSegments, _maxDeg, dmax, 0); //?????????????
            surface = approx.Surface()!;
        }

        // Build the final wire and final face
        NCollection_List<Shape> finalEdges = new NCollection_List<Shape>();
        NCollection_HArray1<Curve2D?> curvesOnPlate = _builder.Curves2d();
        BrepBuilder bb = new BrepBuilder();
        for (i = 1; i <= _boundary.Length(); i++)
        {
            Edge initEdge = _boundary.Value(i)._edge;
            Edge edge = initEdge;
            edge.Orient(Forward);
            if (_oldNewMap.ContainsKey(edge))
            {
                edge = ShapeCasts.Edge(_oldNewMap.GetValue(edge));
            }

            Curve2D? curveOnPlate = curvesOnPlate.Value(i);

            Edge newEdge = ShapeCasts.Edge(edge.EmptyCopied());

            Vertex v1, v2, newV1, newV2;
            Exploration.Vertices(edge, out v1, out v2);

            if (_oldNewMap.ContainsKey(v1))
            {
                newV1 = ShapeCasts.Vertex(_oldNewMap.GetValue(v1));
            }
            else
            {
                Point3D pnt = Tool.Pnt(v1);
                newV1 = new LibraryMakeVertex(pnt).InternalVertex();
                bb.UpdateVertex(newV1, dmax);
                _oldNewMap[v1.Oriented(Forward)] = newV1;
            }

            if (_oldNewMap.ContainsKey(v2))
            {
                newV2 = ShapeCasts.Vertex(_oldNewMap.GetValue(v2));
            }
            else
            {
                Point3D pnt = Tool.Pnt(v2);
                newV2 = new LibraryMakeVertex(pnt).InternalVertex();
                bb.UpdateVertex(newV2, dmax);
                _oldNewMap[v2.Oriented(Forward)] = newV2;
            }

            newV1.Orient(Forward);
            bb.Add(newEdge, newV1);
            newV2.Orient(Reversed);
            bb.Add(newEdge, newV2);
            Placement loc = new Placement();
            bb.UpdateEdge(newEdge, curveOnPlate, surface, loc, dmax);
            BrepLibrary.SameParameter(newEdge, dmax, true);
            finalEdges.Append(newEdge);
            _oldNewMap[initEdge.Oriented(Forward)] = newEdge.Oriented(Forward);
        }

        Wire finalWire = WireFromList(finalEdges);

        _face = new LibraryMakeFace(surface, finalWire).InternalFace();
    }

    public bool IsDone => _isDone;

    /// <summary>The filling face (frozen; a null face if the filling wasn't built).</summary>
    public Face Face()
    {
        _face.TShape()?.FreezeGraph();
        return _face;
    }

    /// <summary>The filling face, not frozen, for ported code (BRepOffsetAPI_MakeFilling).</summary>
    internal Face InternalFace() => _face;

    /// <summary>
    /// The new edge or vertex made from the boundary edge or vertex <paramref name="s"/> (a list of one shape, or
    /// empty; frozen, with the face).
    /// </summary>
    public IReadOnlyList<Shape> Generated(in Shape s)
    {
        NCollection_List<Shape> list = InternalGenerated(s);
        _face.TShape()?.FreezeGraph();
        if (list.IsEmpty())
        {
            return Array.Empty<Shape>();
        }
        Shape[] shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return ImmutableArrays.Own(shapes);
    }

    /// <summary><see cref="Generated"/>: the filling's own list, unfrozen (OCCT's reference), for ported code.</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();

        if (_oldNewMap.ContainsKey(s))
        {
            _generated.Append(_oldNewMap.GetValue(s));
        }
        // OCCT.NET: frozen. A boundary vertex BuildWires' MakeWire copied (OCCT moves it in place): its copy's
        else if (_vertexCopies.TryGetValue(s, out Shape copy) && _oldNewMap.ContainsKey(copy))
        {
            _generated.Append(_oldNewMap.GetValue(copy));
        }

        return _generated;
    }

    /// <summary>The plate builder of the last <see cref="Build"/>, for ported code and tests.</summary>
    internal BuildPlateSurface? InternalBuilder() => _builder;

    /// <summary>The largest distance from the boundary to the surface.</summary>
    public double G0Error() => _builder!.G0Error();

    /// <summary>The largest angle between the surface and the support faces at the boundary.</summary>
    public double G1Error() => _builder!.G1Error();

    /// <summary>The largest curvature gap between the surface and the support faces at the boundary.</summary>
    public double G2Error() => _builder!.G2Error();

    /// <summary>The largest distance between the constraint <paramref name="index"/> and the surface.</summary>
    public double G0Error(int index) => _builder!.G0Error(index);

    /// <summary>The largest angle between the constraint <paramref name="index"/> and the surface.</summary>
    public double G1Error(int index) => _builder!.G1Error(index);

    /// <summary>The largest curvature gap between the constraint <paramref name="index"/> and the surface.</summary>
    public double G2Error(int index) => _builder!.G2Error(index);
}
