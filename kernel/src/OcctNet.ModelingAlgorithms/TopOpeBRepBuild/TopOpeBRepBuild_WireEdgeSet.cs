// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
public class TopOpeBRepBuild_WireEdgeSet : TopOpeBRepBuild_ShapeSet
{
    private readonly Face myFace;

    /// <summary>A wire/edge set on the face <paramref name="F"/>.</summary>
    public TopOpeBRepBuild_WireEdgeSet(in Shape F)
        : base(ShapeType.Vertex)
    {
        myFace = ShapeCasts.Face(F);
    }

    /// <summary>
    /// Adds the wire <paramref name="S"/> (not checked on a face closed in U or V; checking is off anyway, see the base
    /// class).
    /// </summary>
    public override void AddShape(in Shape S)
    {
        bool tocheck = true;
        bool iswire = S.ShapeType() == ShapeType.Wire;
        if (iswire)
        {
            FaceSurfaceAdaptor bas = new(myFace, false);
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
            chk = CheckShape(S);
        }

        if (!chk)
        {
            return;
        }

        ProcessAddShape(S);
    }

    /// <summary>Adds the start edge <paramref name="S"/> (BSpline and Bezier edges are not checked).</summary>
    public override void AddStartElement(in Shape S)
    {
        bool tocheck = true;
        bool isedge = S.ShapeType() == ShapeType.Edge;
        if (isedge)
        {
            EdgeCurveAdaptor cac = new(ShapeCasts.Edge(S));
            CurveKind t = cac.GetType();
            bool b = t == BSplineCurve || t == BezierCurve;
            tocheck = !b;
        }
        bool chk = true;
        if (tocheck)
        {
            chk = CheckShape(S);
        }

        if (!chk)
        {
            return;
        }

        ProcessAddStartElement(S);
    }

    /// <inheritdoc/>
    public override void AddElement(in Shape S)
    {
        base.AddElement(S);
    }

    /// <summary>The face.</summary>
    public Face Face() => myFace;

    /// <inheritdoc/>
    public override void InitNeighbours(in Shape E)
    {
        mySubShapeExplorer.Init(E, mySubShapeType);
        myCurrentShape = E;
        FindNeighbours();
    }

    /// <inheritdoc/>
    public override void FindNeighbours()
    {
        while (mySubShapeExplorer.More())
        {
            // l = list of edges neighbour of edge myCurrentShape through
            // the vertex mySubShapeExplorer.Current(), which is a vertex of the
            // edge myCurrentShape.
            Shape V = mySubShapeExplorer.Current();
            NCollection_List<Shape> l = MakeNeighboursList(myCurrentShape, V);

            // myIncidentShapesIter iterates on the neighbour edges of the edge
            // given as InitNeighbours() argument (this edge has been stored
            // in the field myCurrentShape).

            myIncidentShapesIter.Initialize(l);
            if (myIncidentShapesIter.More())
            {
                break;
            }
            else
            {
                mySubShapeExplorer.Next();
            }
        }
    }

    /// <summary>
    /// The edges incident to the vertex <paramref name="Varg"/> of <paramref name="Earg"/>; where closing edges meet
    /// there, only the closing edges connected to Earg and on the right side of its tangent.
    /// </summary>
    public override NCollection_List<Shape> MakeNeighboursList(in Shape Earg, in Shape Varg)
    {
        Edge E = ShapeCasts.Edge(Earg);
        Vertex V = ShapeCasts.Vertex(Varg);

        NCollection_List<Shape> l = mySubShapeMap.FindFromKey(V);

        int nclosing = NbClosingShapes(l);
        if (nclosing != 0)
        {
            // build myCurrentShapeNeighbours =
            // edge list made of connected shapes to Earg through Varg

            myCurrentShapeNeighbours.Clear();

            for (NCollection_List<Shape>.Iterator it = new(l); it.More(); it.Next())
            {
                Shape curn = it.Value(); // current neighbour
                bool k = VertexConnectsEdgesClosing(V, E, curn);
                if (k)
                {
                    myCurrentShapeNeighbours.Append(curn);
                }
            }

            int newn = NbClosingShapes(myCurrentShapeNeighbours);
            if (newn >= 2)
            {
                Face F = myFace;

                // plusieurs aretes de couture connexes a E par V et telle que :
                // orientation de V dans E # orientation de V dans ces aretes.
                // on ne garde,parmi les aretes de couture connexes,
                // que l'arete A qui verifie tg(E) ^ tg(A) > 0

                Vector2D d1E;
                Point2D pE;
                double parE = Tool.Parameter(V, E);
                bool trim3d = true;
                Curve2D? PCE = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out _, trim3d);
                if (PCE is not null)
                {
                    PCE.D1(parE, out pE, out d1E);
                }
                else
                {
                    LocalD1(F, E, V, out pE, out d1E);
                }

                Orientation Eori = E.Orientation();
                if (Eori == Reversed)
                {
                    d1E.Reverse();
                }

                NCollection_List<Shape>.Iterator lclo = new(myCurrentShapeNeighbours);
                while (lclo.More())
                {
                    if (!IsClosed(lclo.Value()))
                    {
                        lclo.Next();
                        continue;
                    }

                    Edge EE = ShapeCasts.Edge(lclo.Value());
                    Vector2D d1EE;
                    Point2D pEE;
                    double parEE = Tool.Parameter(V, EE);
                    Curve2D? PCEE = TopOpeBRepTool_2d.FC2D_CurveOnSurface(EE, F, out _, out _, out _, trim3d);
                    if (PCEE is not null)
                    {
                        PCEE.D1(parEE, out pEE, out d1EE);
                    }
                    else
                    {
                        LocalD1(F, EE, V, out pEE, out d1EE);
                    }

                    Orientation EEori = EE.Orientation();
                    if (EEori == Reversed)
                    {
                        d1EE.Reverse();
                    }

                    double cross = d1E.Crossed(d1EE);
                    VertexConnectsEdges(V, E, EE, out Orientation oVE, out _);

                    bool t2 = ((cross > 0) && oVE == Reversed) || ((cross < 0) && oVE == Forward);

                    if (t2)
                    { //-- t1
                        // c'est la bonne IsClosed,on ne touche pas a la liste
                        lclo.Next();
                    }
                    else
                    {
                        // c'est la mauvaise IsClosed,on l'enleve de la liste
                        myCurrentShapeNeighbours.Remove(lclo);
                    }
                }
            }
            return myCurrentShapeNeighbours;
        }
        else
        {
            return l;
        }
    } // MakeNeighoursList

    // Whether V is shared by E1 and E2 with different orientations; the orientations of V on E1 and E2 if shared.
    // OCCT leaves o1 and o2 unwritten when V isn't shared, and MakeNeighboursList then reads its oVE uninitialised;
    // the port sets them to FORWARD first.
    private bool VertexConnectsEdges(in Shape V,
                                     in Shape E1,
                                     in Shape E2,
                                     out Orientation o1,
                                     out Orientation o2)
    {
        o1 = Forward;
        o2 = Forward;
        TopOpeBRepTool_ShapeExplorer ex1 = new(), ex2 = new();
        for (ex1.Init(E1, ShapeType.Vertex); ex1.More(); ex1.Next())
        {
            if (V.IsSame(ex1.Current()))
            {
                for (ex2.Init(E2, ShapeType.Vertex); ex2.More(); ex2.Next())
                {
                    if (V.IsSame(ex2.Current()))
                    {
                        o1 = ex1.Current().Orientation();
                        o2 = ex2.Current().Orientation();
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

    private bool VertexConnectsEdgesClosing(in Shape V, in Shape E1, in Shape E2)
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

        bool c1 = IsClosed(E1);
        bool c2 = IsClosed(E2);

        bool testconnect = c1 || c2;
        bool resu = false;
        if (c1 && c2)
        {
            bool u1 = c1 ? IsUClosed(E1) : false;
            bool v1 = c1 ? IsVClosed(E1) : false;
            bool u2 = c2 ? IsUClosed(E2) : false;
            bool v2 = c2 ? IsVClosed(E2) : false;
            bool uvdiff = (u1 && v2) || (u2 && v1);
            testconnect = uvdiff;
        }

        if (testconnect)
        {
            resu = VertexConnectsEdges(V, E1, E2, out _, out _);
        }
        else
        {
            // cto 012 O2 arete de couture de face cylindrique
            // chaque arete de couture est en double dans le WES
            // (une FORWARD et une REVERSED)
            Orientation oe1 = E1.Orientation();
            Orientation oe2 = E2.Orientation();
            bool iseq = E1.IsEqual(E2);
            if ((c1 && c2) && (oe1 == oe2) && (!iseq))
            {
                resu = VertexConnectsEdges(V, E1, E2, out _, out _);
            }
        }
        return resu;
    }

    private int NbClosingShapes(NCollection_List<Shape> L)
    {
        int n = 0;
        for (NCollection_List<Shape>.Iterator it = new(L); it.More(); it.Next())
        {
            Shape S = it.Value();
            if (IsClosed(S))
            {
                n++;
            }
        }
        return n;
    }

    // The point and first derivative at V of the projection of E's 3D curve on F's surface
    private void LocalD1(in Shape SF, in Shape SE, in Shape SV, out Point2D pE, out Vector2D d1E)
    {
        Face F = ShapeCasts.Face(SF);
        Edge E = ShapeCasts.Edge(SE);
        Vertex V = ShapeCasts.Vertex(SV);
        double parE = Tool.Parameter(V, E);

        Curve3D CE = Tool.Curve(E, out Placement Loc, out _, out _)!;
        CE = (Curve3D)CE.Transformed(Loc.Transformation());

        CE.D1(parE, out Point3D p3dE, out Vector3D d3dE);

        Surface S = Tool.Surface(F)!;
        GeomAPI_ProjectPointOnSurf proj = new(p3dE, S);
        proj.LowerDistanceParameters(out double u, out double v);
        pE = new Point2D(u, v);
        S.D1(u, v, out Point3D _, out Vector3D d1u, out Vector3D d1v);
        u = d3dE.Dot(d1u);
        v = d3dE.Dot(d1v);
        d1E = new Vector2D(u, v);
    }

    // Whether E is a closing edge of myFace
    private bool IsClosed(in Shape E)
    {
        Edge EE = ShapeCasts.Edge(E);
        bool closed = Tool.IsClosed(EE, myFace);
        return closed;
    }

    /// <summary>Whether the pcurve of <paramref name="E"/> on <paramref name="F"/> is a U or V isoline.</summary>
    /// <exception cref="ProgramErrorException">The edge has no pcurve on the face.</exception>
    public static void IsUVISO(in Edge E, in Face F, out bool uiso, out bool viso)
    {
        uiso = viso = false;
        Curve2D? PC;
        bool trim3d = true;
        PC = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out _, trim3d);
        if (PC is null)
        {
            throw new ProgramErrorException("TopOpeBRepBuild_WireEdgeSet::IsUVISO");
        }

        if (PC.GetType() == typeof(LineCurve2D))
        {
            LineCurve2D HL = (LineCurve2D)PC;
            Direction2D D = HL.Direction();
            double tol = global::Kerf.Engine.Foundation.Precision.Angular();

            if (D.IsParallel(new Direction2D(Direction2D.D.Y), tol))
            {
                uiso = true;
            }
            else if (D.IsParallel(new Direction2D(Direction2D.D.X), tol))
            {
                viso = true;
            }
        }
    }

    // Whether E is a closing edge on U of myFace
    private bool IsUClosed(in Shape E)
    {
        Edge EE = ShapeCasts.Edge(E);
        IsUVISO(EE, myFace, out bool closed, out _);
        return closed;
    }

    // Whether E is a closing edge on V of myFace
    private bool IsVClosed(in Shape E)
    {
        Edge EE = ShapeCasts.Edge(E);
        IsUVISO(EE, myFace, out _, out bool closed);
        return closed;
    }
}
