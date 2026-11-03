// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    private readonly TopoDS_Face myFace;

    /// <summary>A wire/edge set on the face <paramref name="F"/>.</summary>
    public TopOpeBRepBuild_WireEdgeSet(in TopoDS_Shape F)
        : base(TopAbs_VERTEX)
    {
        myFace = TopoDS.Face(F);
    }

    /// <summary>
    /// Adds the wire <paramref name="S"/> (not checked on a face closed in U or V; checking is off anyway, see the base
    /// class).
    /// </summary>
    public override void AddShape(in TopoDS_Shape S)
    {
        bool tocheck = true;
        bool iswire = S.ShapeType() == TopAbs_WIRE;
        if (iswire)
        {
            BRepAdaptor_Surface bas = new(myFace, false);
            bool uc = bas.IsUClosed();
            bool vc = bas.IsVClosed();
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
    public override void AddStartElement(in TopoDS_Shape S)
    {
        bool tocheck = true;
        bool isedge = S.ShapeType() == TopAbs_EDGE;
        if (isedge)
        {
            BRepAdaptor_Curve cac = new(TopoDS.Edge(S));
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
    public override void AddElement(in TopoDS_Shape S)
    {
        base.AddElement(S);
    }

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => myFace;

    /// <inheritdoc/>
    public override void InitNeighbours(in TopoDS_Shape E)
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
            TopoDS_Shape V = mySubShapeExplorer.Current();
            NCollection_List<TopoDS_Shape> l = MakeNeighboursList(myCurrentShape, V);

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
    public override NCollection_List<TopoDS_Shape> MakeNeighboursList(in TopoDS_Shape Earg, in TopoDS_Shape Varg)
    {
        TopoDS_Edge E = TopoDS.Edge(Earg);
        TopoDS_Vertex V = TopoDS.Vertex(Varg);

        NCollection_List<TopoDS_Shape> l = mySubShapeMap.FindFromKey(V);

        int nclosing = NbClosingShapes(l);
        if (nclosing != 0)
        {
            // build myCurrentShapeNeighbours =
            // edge list made of connected shapes to Earg through Varg

            myCurrentShapeNeighbours.Clear();

            for (NCollection_List<TopoDS_Shape>.Iterator it = new(l); it.More(); it.Next())
            {
                TopoDS_Shape curn = it.Value(); // current neighbour
                bool k = VertexConnectsEdgesClosing(V, E, curn);
                if (k)
                {
                    myCurrentShapeNeighbours.Append(curn);
                }
            }

            int newn = NbClosingShapes(myCurrentShapeNeighbours);
            if (newn >= 2)
            {
                TopoDS_Face F = myFace;

                // plusieurs aretes de couture connexes a E par V et telle que :
                // orientation de V dans E # orientation de V dans ces aretes.
                // on ne garde,parmi les aretes de couture connexes,
                // que l'arete A qui verifie tg(E) ^ tg(A) > 0

                Vector2D d1E;
                Point2D pE;
                double parE = BRep_Tool.Parameter(V, E);
                bool trim3d = true;
                Geom2d_Curve? PCE = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out _, trim3d);
                if (PCE is not null)
                {
                    PCE.D1(parE, out pE, out d1E);
                }
                else
                {
                    LocalD1(F, E, V, out pE, out d1E);
                }

                TopAbs_Orientation Eori = E.Orientation();
                if (Eori == TopAbs_REVERSED)
                {
                    d1E.Reverse();
                }

                NCollection_List<TopoDS_Shape>.Iterator lclo = new(myCurrentShapeNeighbours);
                while (lclo.More())
                {
                    if (!IsClosed(lclo.Value()))
                    {
                        lclo.Next();
                        continue;
                    }

                    TopoDS_Edge EE = TopoDS.Edge(lclo.Value());
                    Vector2D d1EE;
                    Point2D pEE;
                    double parEE = BRep_Tool.Parameter(V, EE);
                    Geom2d_Curve? PCEE = TopOpeBRepTool_2d.FC2D_CurveOnSurface(EE, F, out _, out _, out _, trim3d);
                    if (PCEE is not null)
                    {
                        PCEE.D1(parEE, out pEE, out d1EE);
                    }
                    else
                    {
                        LocalD1(F, EE, V, out pEE, out d1EE);
                    }

                    TopAbs_Orientation EEori = EE.Orientation();
                    if (EEori == TopAbs_REVERSED)
                    {
                        d1EE.Reverse();
                    }

                    double cross = d1E.Crossed(d1EE);
                    VertexConnectsEdges(V, E, EE, out TopAbs_Orientation oVE, out _);

                    bool t2 = ((cross > 0) && oVE == TopAbs_REVERSED) || ((cross < 0) && oVE == TopAbs_FORWARD);

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
    private bool VertexConnectsEdges(in TopoDS_Shape V,
                                     in TopoDS_Shape E1,
                                     in TopoDS_Shape E2,
                                     out TopAbs_Orientation o1,
                                     out TopAbs_Orientation o2)
    {
        o1 = TopAbs_FORWARD;
        o2 = TopAbs_FORWARD;
        TopOpeBRepTool_ShapeExplorer ex1 = new(), ex2 = new();
        for (ex1.Init(E1, TopAbs_VERTEX); ex1.More(); ex1.Next())
        {
            if (V.IsSame(ex1.Current()))
            {
                for (ex2.Init(E2, TopAbs_VERTEX); ex2.More(); ex2.Next())
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

    private bool VertexConnectsEdgesClosing(in TopoDS_Shape V, in TopoDS_Shape E1, in TopoDS_Shape E2)
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
            TopAbs_Orientation oe1 = E1.Orientation();
            TopAbs_Orientation oe2 = E2.Orientation();
            bool iseq = E1.IsEqual(E2);
            if ((c1 && c2) && (oe1 == oe2) && (!iseq))
            {
                resu = VertexConnectsEdges(V, E1, E2, out _, out _);
            }
        }
        return resu;
    }

    private int NbClosingShapes(NCollection_List<TopoDS_Shape> L)
    {
        int n = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(L); it.More(); it.Next())
        {
            TopoDS_Shape S = it.Value();
            if (IsClosed(S))
            {
                n++;
            }
        }
        return n;
    }

    // The point and first derivative at V of the projection of E's 3D curve on F's surface
    private void LocalD1(in TopoDS_Shape SF, in TopoDS_Shape SE, in TopoDS_Shape SV, out Point2D pE, out Vector2D d1E)
    {
        TopoDS_Face F = TopoDS.Face(SF);
        TopoDS_Edge E = TopoDS.Edge(SE);
        TopoDS_Vertex V = TopoDS.Vertex(SV);
        double parE = BRep_Tool.Parameter(V, E);

        Geom_Curve CE = BRep_Tool.Curve(E, out Placement Loc, out _, out _)!;
        CE = (Geom_Curve)CE.Transformed(Loc.Transformation());

        CE.D1(parE, out Point3D p3dE, out Vector3D d3dE);

        Geom_Surface S = BRep_Tool.Surface(F)!;
        GeomAPI_ProjectPointOnSurf proj = new(p3dE, S);
        proj.LowerDistanceParameters(out double u, out double v);
        pE = new Point2D(u, v);
        S.D1(u, v, out Point3D _, out Vector3D d1u, out Vector3D d1v);
        u = d3dE.Dot(d1u);
        v = d3dE.Dot(d1v);
        d1E = new Vector2D(u, v);
    }

    // Whether E is a closing edge of myFace
    private bool IsClosed(in TopoDS_Shape E)
    {
        TopoDS_Edge EE = TopoDS.Edge(E);
        bool closed = BRep_Tool.IsClosed(EE, myFace);
        return closed;
    }

    /// <summary>Whether the pcurve of <paramref name="E"/> on <paramref name="F"/> is a U or V isoline.</summary>
    /// <exception cref="ProgramErrorException">The edge has no pcurve on the face.</exception>
    public static void IsUVISO(in TopoDS_Edge E, in TopoDS_Face F, out bool uiso, out bool viso)
    {
        uiso = viso = false;
        Geom2d_Curve? PC;
        bool trim3d = true;
        PC = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out _, trim3d);
        if (PC is null)
        {
            throw new ProgramErrorException("TopOpeBRepBuild_WireEdgeSet::IsUVISO");
        }

        if (PC.GetType() == typeof(Geom2d_Line))
        {
            Geom2d_Line HL = (Geom2d_Line)PC;
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
    private bool IsUClosed(in TopoDS_Shape E)
    {
        TopoDS_Edge EE = TopoDS.Edge(E);
        IsUVISO(EE, myFace, out bool closed, out _);
        return closed;
    }

    // Whether E is a closing edge on V of myFace
    private bool IsVClosed(in TopoDS_Shape E)
    {
        TopoDS_Edge EE = TopoDS.Edge(E);
        IsUVISO(EE, myFace, out _, out bool closed);
        return closed;
    }
}
