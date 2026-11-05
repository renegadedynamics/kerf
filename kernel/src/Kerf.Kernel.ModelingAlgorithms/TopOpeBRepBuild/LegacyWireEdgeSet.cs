// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// The shape set of a face to rebuild: wires (bounds) kept as they are, edges (bound elements) to start the
/// reconstruction from, and the edges incident to each vertex. Neighbours through a vertex shared by closing edges
/// (seams) are chosen by the turn of their pcurves' tangents.
/// </summary>
/// <remarks>
/// The edges must have a pcurve on the face's surface (or the surface be a plane). OCCT's constructor takes an unused
/// <c>void*</c>; the port drops it. The debugging members (<c>DumpSS</c>, <c>SName</c>, <c>SNameori</c>,
/// <c>SNameVEE</c>, <c>SNameVEL</c>) are left out (decision 493).
/// </remarks>
internal class LegacyWireEdgeSet : BuildShapeSet
{
    private readonly Face _face;

    /// <summary>A wire/edge set on the face <paramref name="f"/>.</summary>
    public LegacyWireEdgeSet(in Shape f)
        : base(ShapeType.Vertex)
    {
        _face = ShapeCasts.Face(f);
    }

    /// <summary>
    /// Adds the wire <paramref name="s"/> (not checked on a face closed in U or V; checking is off anyway, see the base
    /// class).
    /// </summary>
    public override void AddShape(in Shape s)
    {
        bool tocheck = true;
        bool iswire = s.Type == ShapeType.Wire;
        if (iswire)
        {
            FaceSurfaceAdaptor bas = new(_face, false);
            bool uc = bas.IsUClosed;
            bool vc = bas.IsVClosed;
            if (uc || vc)
            {
                tocheck = false;
            }
        }
        bool chk = true;
        if (tocheck)
        {
            chk = CheckShape(s);
        }

        if (!chk)
        {
            return;
        }

        ProcessAddShape(s);
    }

    /// <summary>Adds the start edge <paramref name="s"/> (BSpline and Bezier edges are not checked).</summary>
    public override void AddStartElement(in Shape s)
    {
        bool tocheck = true;
        bool isedge = s.Type == ShapeType.Edge;
        if (isedge)
        {
            EdgeCurveAdaptor cac = new(ShapeCasts.Edge(s));
            CurveKind t = cac.GetType();
            bool b = t == BSplineCurve || t == BezierCurve;
            tocheck = !b;
        }
        bool chk = true;
        if (tocheck)
        {
            chk = CheckShape(s);
        }

        if (!chk)
        {
            return;
        }

        ProcessAddStartElement(s);
    }

    /// <inheritdoc/>
    public override void AddElement(in Shape s)
    {
        base.AddElement(s);
    }

    /// <summary>The face.</summary>
    public Face Face() => _face;

    /// <inheritdoc/>
    public override void InitNeighbours(in Shape e)
    {
        _subShapeExplorer.Init(e, _subShapeType);
        _currentShape = e;
        FindNeighbours();
    }

    /// <inheritdoc/>
    public override void FindNeighbours()
    {
        while (_subShapeExplorer.More())
        {
            // l = list of edges neighbour of edge myCurrentShape through
            // the vertex mySubShapeExplorer.Current(), which is a vertex of the
            // edge myCurrentShape.
            Shape v = _subShapeExplorer.Current();
            NCollection_List<Shape> l = MakeNeighboursList(_currentShape, v);

            // myIncidentShapesIter iterates on the neighbour edges of the edge
            // given as InitNeighbours() argument (this edge has been stored
            // in the field myCurrentShape).

            _incidentShapesIter.Initialize(l);
            if (_incidentShapesIter.More())
            {
                break;
            }
            else
            {
                _subShapeExplorer.Next();
            }
        }
    }

    /// <summary>
    /// The edges incident to the vertex <paramref name="varg"/> of <paramref name="earg"/>; where closing edges meet
    /// there, only the closing edges connected to Earg and on the right side of its tangent.
    /// </summary>
    public override NCollection_List<Shape> MakeNeighboursList(in Shape earg, in Shape varg)
    {
        Edge e = ShapeCasts.Edge(earg);
        Vertex v = ShapeCasts.Vertex(varg);

        NCollection_List<Shape> l = _subShapeMap.GetValue(v);

        int nclosing = NbClosingShapes(l);
        if (nclosing != 0)
        {
            // build myCurrentShapeNeighbours =
            // edge list made of connected shapes to Earg through Varg

            _currentShapeNeighbours.Clear();

            for (NCollection_List<Shape>.Iterator it = new(l); it.More(); it.Next())
            {
                Shape curn = it.Value(); // current neighbour
                bool k = VertexConnectsEdgesClosing(v, e, curn);
                if (k)
                {
                    _currentShapeNeighbours.Append(curn);
                }
            }

            int newn = NbClosingShapes(_currentShapeNeighbours);
            if (newn >= 2)
            {
                Face f = _face;

                // plusieurs aretes de couture connexes a E par V et telle que :
                // orientation de V dans E # orientation de V dans ces aretes.
                // on ne garde,parmi les aretes de couture connexes,
                // que l'arete A qui verifie tg(E) ^ tg(A) > 0

                Vector2D d1E;
                Point2D pE;
                double parE = Tool.Parameter(v, e);
                bool trim3d = true;
                Curve2D? pce = Legacy2D.FC2D_CurveOnSurface(e, f, out _, out _, out _, trim3d);
                if (pce is not null)
                {
                    pce.D1(parE, out pE, out d1E);
                }
                else
                {
                    LocalD1(f, e, v, out pE, out d1E);
                }

                Orientation eori = e.Orientation;
                if (eori == Reversed)
                {
                    d1E = d1E.Reversed();
                }

                NCollection_List<Shape>.Iterator lclo = new(_currentShapeNeighbours);
                while (lclo.More())
                {
                    if (!IsClosed(lclo.Value()))
                    {
                        lclo.Next();
                        continue;
                    }

                    Edge ee = ShapeCasts.Edge(lclo.Value());
                    Vector2D d1EE;
                    Point2D pEE;
                    double parEE = Tool.Parameter(v, ee);
                    Curve2D? pcee = Legacy2D.FC2D_CurveOnSurface(ee, f, out _, out _, out _, trim3d);
                    if (pcee is not null)
                    {
                        pcee.D1(parEE, out pEE, out d1EE);
                    }
                    else
                    {
                        LocalD1(f, ee, v, out pEE, out d1EE);
                    }

                    Orientation eEori = ee.Orientation;
                    if (eEori == Reversed)
                    {
                        d1EE = d1EE.Reversed();
                    }

                    double cross = d1E.Crossed(d1EE);
                    VertexConnectsEdges(v, e, ee, out Orientation oVE, out _);

                    bool t2 = ((cross > 0) && oVE == Reversed) || ((cross < 0) && oVE == Forward);

                    if (t2)
                    { //-- t1
                        // c'est la bonne IsClosed,on ne touche pas a la liste
                        lclo.Next();
                    }
                    else
                    {
                        // c'est la mauvaise IsClosed,on l'enleve de la liste
                        _currentShapeNeighbours.Remove(lclo);
                    }
                }
            }
            return _currentShapeNeighbours;
        }
        else
        {
            return l;
        }
    } // MakeNeighoursList

    // Whether V is shared by E1 and E2 with different orientations; the orientations of V on E1 and E2 if shared.
    // OCCT leaves o1 and o2 unwritten when V isn't shared, and MakeNeighboursList then reads its oVE uninitialised;
    // the port sets them to FORWARD first.
    private bool VertexConnectsEdges(in Shape v,
                                     in Shape e1,
                                     in Shape e2,
                                     out Orientation o1,
                                     out Orientation o2)
    {
        o1 = Forward;
        o2 = Forward;
        ToolShapeExplorer ex1 = new(), ex2 = new();
        for (ex1.Init(e1, ShapeType.Vertex); ex1.More(); ex1.Next())
        {
            if (v.IsSame(ex1.Current()))
            {
                for (ex2.Init(e2, ShapeType.Vertex); ex2.More(); ex2.Next())
                {
                    if (v.IsSame(ex2.Current()))
                    {
                        o1 = ex1.Current().Orientation;
                        o2 = ex2.Current().Orientation;
                        if (o1 != o2)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    private bool VertexConnectsEdgesClosing(in Shape v, in Shape e1, in Shape e2)
    {
        // Let S the set of incident edges on vertex V.
        // S contains at least one closed edge on the periodic face to build.
        // (the face implied in closing test of edge is myFace)
        // E1,E2 are S shapes (sharing V).
        //
        // if E1 and E2 are not closed : edges are NOT connected
        // if E1 or E2 is/are closed :
        //   if V is shared by E1 and E2 with different orientations : edges are connected
        //   else : edges are NOT connected

        bool c1 = IsClosed(e1);
        bool c2 = IsClosed(e2);

        bool testconnect = c1 || c2;
        bool resu = false;
        if (c1 && c2)
        {
            bool u1 = c1 ? IsUClosed(e1) : false;
            bool v1 = c1 ? IsVClosed(e1) : false;
            bool u2 = c2 ? IsUClosed(e2) : false;
            bool v2 = c2 ? IsVClosed(e2) : false;
            bool uvdiff = (u1 && v2) || (u2 && v1);
            testconnect = uvdiff;
        }

        if (testconnect)
        {
            resu = VertexConnectsEdges(v, e1, e2, out _, out _);
        }
        else
        {
            // cto 012 O2 arete de couture de face cylindrique
            // chaque arete de couture est en double dans le WES
            // (une FORWARD et une REVERSED)
            Orientation oe1 = e1.Orientation;
            Orientation oe2 = e2.Orientation;
            bool iseq = e1.IsEqual(e2);
            if ((c1 && c2) && (oe1 == oe2) && (!iseq))
            {
                resu = VertexConnectsEdges(v, e1, e2, out _, out _);
            }
        }
        return resu;
    }

    private int NbClosingShapes(NCollection_List<Shape> l)
    {
        int n = 0;
        for (NCollection_List<Shape>.Iterator it = new(l); it.More(); it.Next())
        {
            Shape s = it.Value();
            if (IsClosed(s))
            {
                n++;
            }
        }
        return n;
    }

    // The point and first derivative at V of the projection of E's 3D curve on F's surface
    private void LocalD1(in Shape sf, in Shape se, in Shape sv, out Point2D pE, out Vector2D d1E)
    {
        Face f = ShapeCasts.Face(sf);
        Edge e = ShapeCasts.Edge(se);
        Vertex V = ShapeCasts.Vertex(sv);
        double parE = Tool.Parameter(V, e);

        Curve3D ce = Tool.Curve(e, out Placement loc, out _, out _)!;
        ce = (Curve3D)ce.Transformed(loc.Transformation);

        ce.D1(parE, out Point3D p3dE, out Vector3D d3dE);

        Surface s = Tool.Surface(f)!;
        ProjectPointOnSurface proj = new(p3dE, s);
        proj.LowerDistanceParameters(out double u, out double v);
        pE = new Point2D(u, v);
        s.D1(u, v, out Point3D _, out Vector3D d1u, out Vector3D d1v);
        u = d3dE.Dot(d1u);
        v = d3dE.Dot(d1v);
        d1E = new Vector2D(u, v);
    }

    // Whether E is a closing edge of myFace
    private bool IsClosed(in Shape e)
    {
        Edge ee = ShapeCasts.Edge(e);
        bool closed = Tool.IsClosed(ee, _face);
        return closed;
    }

    /// <summary>Whether the pcurve of <paramref name="e"/> on <paramref name="f"/> is a U or V isoline.</summary>
    /// <exception cref="ProgramErrorException">The edge has no pcurve on the face.</exception>
    public static void IsUVISO(in Edge e, in Face f, out bool uiso, out bool viso)
    {
        uiso = viso = false;
        Curve2D? pc;
        bool trim3d = true;
        pc = Legacy2D.FC2D_CurveOnSurface(e, f, out _, out _, out _, trim3d);
        if (pc is null)
        {
            throw new ProgramErrorException("TopOpeBRepBuild_WireEdgeSet::IsUVISO");
        }

        if (pc.GetType() == typeof(LineCurve2D))
        {
            LineCurve2D hl = (LineCurve2D)pc;
            Direction2D d = hl.Direction;
            double tol = global::Kerf.Engine.Foundation.Precision.Angular();

            if (d.IsParallel(new Direction2D(Direction2D.D.Y), tol))
            {
                uiso = true;
            }
            else if (d.IsParallel(new Direction2D(Direction2D.D.X), tol))
            {
                viso = true;
            }
        }
    }

    // Whether E is a closing edge on U of myFace
    private bool IsUClosed(in Shape e)
    {
        Edge ee = ShapeCasts.Edge(e);
        IsUVISO(ee, _face, out bool closed, out _);
        return closed;
    }

    // Whether E is a closing edge on V of myFace
    private bool IsVClosed(in Shape e)
    {
        Edge ee = ShapeCasts.Edge(e);
        IsUVISO(ee, _face, out _, out bool closed);
        return closed;
    }
}
