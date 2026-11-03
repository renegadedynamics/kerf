// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Classifies wires and edges of a face against each other, in the face's parametric space: a point of a first edge
/// against the other loop's edges (a passive face classifier), or, between two wires that classification couldn't
/// settle, a point of an edge of one in the face bounded by the other.
/// </summary>
/// <remarks>
/// A working object. Where an edge has no stored pcurve on the face, <see cref="ResetElement"/>,
/// <see cref="CompareElement"/> and <see cref="LoopToShape"/> compute one and store it on the edge, as in OCCT: the
/// edges must then be unfrozen (on a plane, BRep_Tool computes the pcurve and nothing is written).
/// </remarks>
public class TopOpeBRepBuild_WireEdgeClassifier : TopOpeBRepBuild_CompositeClassifier
{
    private bool myFirstCompare;
    private Point2D myPoint2d = new();
    private BRepClass_Edge myBCEdge = new();
    private readonly BRepClass_FacePassiveClassifier myFPC = new();
    private TopoDS_Shape myShape = new();

    /// <summary>A classifier of the wires and edges of the face <paramref name="F"/>, the blocks being in <paramref name="BB"/>.</summary>
    public TopOpeBRepBuild_WireEdgeClassifier(in TopoDS_Shape F, TopOpeBRepBuild_BlockBuilder BB)
        : base(BB)
    {
        myBCEdge.ChangeFace() = TopoDS.Face(F);
    }

    /// <inheritdoc/>
    public override TopAbs_State Compare(TopOpeBRepBuild_Loop L1, TopOpeBRepBuild_Loop L2)
    {
        TopAbs_State state = TopAbs_UNKNOWN;

        bool isshape1 = L1.IsShape();
        bool isshape2 = L2.IsShape();

        if (isshape2 && isshape1)
        { // L1 is Shape , L2 is Shape
            TopoDS_Shape s1 = L1.Shape();
            TopoDS_Shape s2 = L2.Shape();
            state = CompareShapes(s1, s2);
        }
        else if (isshape2 && !isshape1)
        { // L1 is Block , L2 is Shape
            TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
            Bit1.Initialize();
            bool yena1 = Bit1.More();
            while (yena1)
            {
                TopoDS_Shape s1 = myBlockBuilder.Element(Bit1);
                TopoDS_Shape s2 = L2.Shape();
                state = CompareElementToShape(s1, s2);
                yena1 = false;
                if (state == TopAbs_UNKNOWN)
                {
                    if (Bit1.More())
                    {
                        Bit1.Next();
                    }
                    yena1 = Bit1.More();
                }
            }
        }
        else if (!isshape2 && isshape1)
        { // L1 is Shape , L2 is Block
            TopoDS_Shape s1 = L1.Shape();
            ResetShape(s1);
            TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
            for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
            {
                TopoDS_Shape s2 = myBlockBuilder.Element(Bit2);
                CompareElement(s2);
            }
            state = State();
        }
        else if (!isshape2 && !isshape1)
        { // L1 is Block , L2 is Block
            if (state == TopAbs_UNKNOWN)
            {
                TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
                Bit1.Initialize();
                bool yena1 = Bit1.More();
                while (yena1)
                {
                    TopoDS_Shape s1 = myBlockBuilder.Element(Bit1);
                    ResetElement(s1);
                    TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
                    for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
                    {
                        TopoDS_Shape s2 = myBlockBuilder.Element(Bit2);
                        CompareElement(s2);
                    }
                    state = State();
                    yena1 = false;
                    if (state == TopAbs_UNKNOWN)
                    {
                        if (Bit1.More())
                        {
                            Bit1.Next();
                        }
                        yena1 = Bit1.More();
                    }
                }
            } // UNKNOWN

            if (state == TopAbs_UNKNOWN)
            {
                TopoDS_Shape s1 = LoopToShape(L1);
                if (s1.IsNull())
                {
                    return state;
                }
                TopoDS_Shape s2 = LoopToShape(L2);
                if (s2.IsNull())
                {
                    return state;
                }
                TopOpeBRepTool_ShapeClassifier SC = TopOpeBRepTool_SC.FSC_GetPSC();
                int samedomain = SC.SameDomain();
                SC.SameDomain(1);
                SC.SetReference(s2);
                TopoDS_Shape AvS = s2;
                state = SC.StateShapeReference(s1, AvS);
                SC.SameDomain(samedomain);
            } // UNKNOWN
        }

        return state;
    }

    /// <summary>
    /// A face on the classifier's surface bounded by a wire of the block's edges (null for an empty block). Edges
    /// without a pcurve on the face get one.
    /// </summary>
    public TopoDS_Shape LoopToShape(TopOpeBRepBuild_Loop L)
    {
        myShape.Nullify();
        TopOpeBRepBuild_BlockIterator Bit = L.BlockIterator();
        Bit.Initialize();
        if (!Bit.More())
        {
            return myShape;
        }

        TopoDS_Shape aLocalShape = myBCEdge.Face();
        TopoDS_Face F1 = TopoDS.Face(aLocalShape);
        aLocalShape = F1.EmptyCopied();
        TopoDS_Face F = TopoDS.Face(aLocalShape);
        BRep_Builder BB = new();
        TopoDS_Wire W = new();
        BB.MakeWire(ref W);
        for (; Bit.More(); Bit.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(myBlockBuilder.Element(Bit));
            double tolE;
            tolE = BRep_Tool.Tolerance(E);
            bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F);
            if (!haspc)
            {
                Geom2d_Curve? C2D;
                C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out double tolpc);
                if (C2D is not null)
                {
                    double tol = Max(tolpc, tolE);
                    BB.UpdateEdge(E, C2D, F, tol);
                }
            }
            BB.Add(W, E);
        }
        BB.Add(F, W);
        myShape = F;
        return myShape;
    }

    // The unit tangent at v of e, pointing into the edge
    private static Vector3D FUN_tgINE(in TopoDS_Vertex v, in TopoDS_Vertex vl, in TopoDS_Edge e)
    {
        // tg oriented INSIDE 1d(e)
        // vl : last vertex of e
        double par = BRep_Tool.Parameter(v, e);
        bool ok = TopOpeBRepTool_TOOL.TggeomE(par, e, out Vector3D tg);
        if (!ok)
        {
            return new Vector3D(0.0, 0.0, 0.0); // NYIRAISE
        }
        if (v.IsSame(vl))
        {
            tg.Reverse();
        }
        return tg;
    }

    /// <summary>
    /// Classifies the wire <paramref name="B1"/> with the wire <paramref name="B2"/>: an edge of B1 against B2's edges;
    /// if undecided, a point of an edge of B2 that isn't in B1, in the face bounded by B1.
    /// </summary>
    public override TopAbs_State CompareShapes(in TopoDS_Shape B1, in TopoDS_Shape B2)
    {
        TopAbs_State state = TopAbs_UNKNOWN;

        TopExp_Explorer ex1 = new(B1, TopAbs_EDGE);
        if (!ex1.More())
        {
            return state; // NYIRAISE
        }
        for (; ex1.More(); ex1.Next())
        {
            TopoDS_Edge e1 = TopoDS.Edge(ex1.Current());
            TopExp.Vertices(e1, out TopoDS_Vertex vf1, out TopoDS_Vertex vl1); // xpu101198
            bool e1clo = vf1.IsSame(vl1);
            NCollection_IndexedMap<TopoDS_Shape> mapv1 = new(TopTools_ShapeMapHasher.Instance);
            mapv1.Add(vf1);
            mapv1.Add(vl1);

            ResetShape(e1);
            bool indy = false;
            TopExp_Explorer Ex = new();
            for (Ex.Init(B2, TopAbs_EDGE); Ex.More(); Ex.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(Ex.Current());
                if (E.IsSame(e1))
                {
                    state = TopAbs_UNKNOWN;
                    break;
                } // eap occ416
                TopExp.Vertices(E, out TopoDS_Vertex vf, out TopoDS_Vertex vl); // xpu101198
                bool Eclo = vf.IsSame(vl);                                        // xpu101198
                bool hasf = mapv1.Contains(vf);                                   // xpu101198
                bool hasl = mapv1.Contains(vl);                                   // xpu101198
                bool filter = (hasf || hasl) && (!e1clo) && (!Eclo);              // nyi : Eclo || e1clo
                if (filter)
                { // xpu101198
                    TopoDS_Vertex vshared = new();
                    if (hasf)
                    {
                        vshared = vf;
                    }
                    if (hasl)
                    {
                        vshared = vl;
                    }
                    Vector3D tg1 = FUN_tgINE(vshared, vl1, e1);
                    Vector3D tg = FUN_tgINE(vshared, vl, E);
                    double dot = tg1.Dot(tg);
                    double tol = global::Kerf.Engine.Foundation.Precision.Angular() * 1.0e4; // nyixpu
                    bool undecided = Math.Abs(1 + dot) < tol;
                    if (undecided)
                    {
                        indy = true;
                    }
                } // xpu101198
                if (indy)
                {
                    state = TopAbs_UNKNOWN;
                    break;
                }
                CompareElement(E);
                state = State();
            } // ex(B2,EDGE)
            if (state != TopAbs_UNKNOWN)
            {
                break;
            }
        } // ex1

        bool resta = state == TopAbs_UNKNOWN;
        resta = resta && (B2.ShapeType() == TopAbs_WIRE) && (B1.ShapeType() == TopAbs_WIRE);
        if (resta)
        {
            // classifying first point of B2's first edge not in B1's edges, in face bounded by B1
            NCollection_IndexedMap<TopoDS_Shape> mape1 = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(B1, TopAbs_EDGE, mape1);
            TopExp_Explorer ex2 = new(B2, TopAbs_EDGE);
            for (; ex2.More(); ex2.Next())
            {
                TopoDS_Edge E2 = TopoDS.Edge(ex2.Current());
                if (mape1.Contains(E2))
                {
                    continue;
                }

                TopoDS_Face theFace = myBCEdge.Face();
                BRep_Builder BB = new();
                TopoDS_Shape aLocalShape = theFace.Oriented(TopAbs_FORWARD);
                TopoDS_Face ftmp = TopoDS.Face(aLocalShape);
                aLocalShape = ftmp.EmptyCopied();
                TopoDS_Face F2 = TopoDS.Face(aLocalShape);
                BB.Add(F2, TopoDS.Wire(B2));
                BRepAdaptor_Curve2d BC2d = new(E2, F2);
                TopOpeBRepTool_PROJECT.FUN_tool_bounds(E2, out double f, out double l);
                double x = 0.45678;
                double p2 = (1 - x) * l + x * f;
                Point2D p2d = BC2d.Value(p2);

                aLocalShape = ftmp.EmptyCopied();
                TopoDS_Face F1 = TopoDS.Face(aLocalShape);
                BB.Add(F1, TopoDS.Wire(B1));
                double tolF1 = BRep_Tool.Tolerance(F1);
                BRepClass_FaceClassifier Fclass = new(F1, p2d, tolF1);
                state = Fclass.State();
                return state;
            } // ex2
        }

        return state;
    }

    /// <summary>Classifies the edge <paramref name="EE"/> with the wire <paramref name="B"/>.</summary>
    public override TopAbs_State CompareElementToShape(in TopoDS_Shape EE, in TopoDS_Shape B)
    {
        // isvalid edges : ~ (closed edge && ~reachable B)
        ResetElement(EE);
        TopExp_Explorer Ex = new();
        for (Ex.Init(B, TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            TopoDS_Shape E = Ex.Current();
            CompareElement(E);
        }
        TopAbs_State state = State();
        return state;
    }

    /// <summary>Prepares a classification involving the wire (or edge) <paramref name="B"/>: ResetElement on its first edge.</summary>
    public override void ResetShape(in TopoDS_Shape B)
    {
        if (B.ShapeType() == TopAbs_EDGE)
        {
            ResetElement(B);
        }
        else
        {
            TopExp_Explorer ex = new(B, TopAbs_EDGE);
            if (ex.More())
            {
                TopoDS_Shape E = ex.Current();
                ResetElement(E);
            }
        }
    }

    /// <summary>
    /// Prepares a classification involving the edge <paramref name="EE"/>: the 2D point to classify is on its pcurve
    /// (computed and stored on the edge if it has none).
    /// </summary>
    /// <exception cref="ProgramErrorException">No pcurve could be found.</exception>
    public override void ResetElement(in TopoDS_Shape EE)
    {
        TopoDS_Edge E = TopoDS.Edge(EE);
        TopoDS_Face F = myBCEdge.Face();

        // initialize myPoint2d with first point of edge E
        Geom2d_Curve? C2D;                                                // jyl980406+
        bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F); // jyl980406+
        if (!haspc)
        { // jyl980406+
            bool trim3d = true;
            C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = BRep_Tool.Tolerance(E);                                                       // jyl980406+
            double tol = Max(tolE, tolpc0);                                                             // jyl980406+
            BRep_Builder BB = new();
            BB.UpdateEdge(E, C2D, F, tol); // jyl980406+
        } // jyl980406+

        C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out double f2, out double l2, out _);
        if (C2D is null)
        {
            throw new ProgramErrorException("WEC : ResetElement");
        }

        double t = 0.397891143689;
        double par = (1 - t) * f2 + t * l2;
        myPoint2d = C2D.Value(par);

        myFirstCompare = true;
    }

    /// <summary>
    /// Adds the edge <paramref name="EE"/> to the edges the 2D point is classified against (its pcurve computed and
    /// stored on the edge if it has none). Always true.
    /// </summary>
    public override bool CompareElement(in TopoDS_Shape EE)
    {
        bool bRet = true;
        TopoDS_Edge E = TopoDS.Edge(EE);
        TopoDS_Face F = myBCEdge.Face();

        Geom2d_Curve? C2D;                                                // jyl980402+
        bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F); // jyl980402+
        if (!haspc)
        { // jyl980402+
            bool trim3d = true;
            C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = BRep_Tool.Tolerance(E);                                                       // jyl980402+
            double tol = Max(tolE, tolpc0);                                                             // jyl980402+
            BRep_Builder BB = new();
            BB.UpdateEdge(E, C2D, F, tol); // jyl980402+
        } // jyl980402+

        if (myFirstCompare)
        {
            C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out double f2, out double l2, out _);
            double t = 0.33334567;
            double par = (1 - t) * f2 + t * l2;
            Point2D p2d = C2D!.Value(par);

            Vector2D v2d = new(myPoint2d, p2d);
            Line2D l2d = new(myPoint2d, new Direction2D(v2d));
            double dist = myPoint2d.Distance(p2d);
            double tol2d = global::Kerf.Engine.Foundation.Precision.PConfusion(); // NYI : a voir
            myFPC.Reset(l2d, dist, tol2d);
            myFirstCompare = false;
        }

        myBCEdge.ChangeEdge() = E;
        TopAbs_Orientation Eori = E.Orientation();
        myFPC.Compare(myBCEdge, Eori);
        return bRet;
    }

    /// <summary>The state of the 2D point against the edges added.</summary>
    public override TopAbs_State State()
    {
        TopAbs_State state = myFPC.State();
        return state;
    }
}
