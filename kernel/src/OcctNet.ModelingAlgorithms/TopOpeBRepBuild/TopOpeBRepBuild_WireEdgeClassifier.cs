// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private Shape myShape = new();

    /// <summary>A classifier of the wires and edges of the face <paramref name="F"/>, the blocks being in <paramref name="BB"/>.</summary>
    public TopOpeBRepBuild_WireEdgeClassifier(in Shape F, TopOpeBRepBuild_BlockBuilder BB)
        : base(BB)
    {
        myBCEdge.ChangeFace() = ShapeCasts.Face(F);
    }

    /// <inheritdoc/>
    public override Containment Compare(TopOpeBRepBuild_Loop L1, TopOpeBRepBuild_Loop L2)
    {
        Containment state = Unknown;

        bool isshape1 = L1.IsShape();
        bool isshape2 = L2.IsShape();

        if (isshape2 && isshape1)
        { // L1 is Shape , L2 is Shape
            Shape s1 = L1.Shape();
            Shape s2 = L2.Shape();
            state = CompareShapes(s1, s2);
        }
        else if (isshape2 && !isshape1)
        { // L1 is Block , L2 is Shape
            TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
            Bit1.Initialize();
            bool yena1 = Bit1.More();
            while (yena1)
            {
                Shape s1 = myBlockBuilder.Element(Bit1);
                Shape s2 = L2.Shape();
                state = CompareElementToShape(s1, s2);
                yena1 = false;
                if (state == Unknown)
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
            Shape s1 = L1.Shape();
            ResetShape(s1);
            TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
            for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
            {
                Shape s2 = myBlockBuilder.Element(Bit2);
                CompareElement(s2);
            }
            state = State();
        }
        else if (!isshape2 && !isshape1)
        { // L1 is Block , L2 is Block
            if (state == Unknown)
            {
                TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
                Bit1.Initialize();
                bool yena1 = Bit1.More();
                while (yena1)
                {
                    Shape s1 = myBlockBuilder.Element(Bit1);
                    ResetElement(s1);
                    TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
                    for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
                    {
                        Shape s2 = myBlockBuilder.Element(Bit2);
                        CompareElement(s2);
                    }
                    state = State();
                    yena1 = false;
                    if (state == Unknown)
                    {
                        if (Bit1.More())
                        {
                            Bit1.Next();
                        }
                        yena1 = Bit1.More();
                    }
                }
            } // UNKNOWN

            if (state == Unknown)
            {
                Shape s1 = LoopToShape(L1);
                if (s1.IsNull)
                {
                    return state;
                }
                Shape s2 = LoopToShape(L2);
                if (s2.IsNull)
                {
                    return state;
                }
                TopOpeBRepTool_ShapeClassifier SC = TopOpeBRepTool_SC.FSC_GetPSC();
                int samedomain = SC.SameDomain();
                SC.SameDomain(1);
                SC.SetReference(s2);
                Shape AvS = s2;
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
    public Shape LoopToShape(TopOpeBRepBuild_Loop L)
    {
        myShape.Nullify();
        TopOpeBRepBuild_BlockIterator Bit = L.BlockIterator();
        Bit.Initialize();
        if (!Bit.More())
        {
            return myShape;
        }

        Shape aLocalShape = myBCEdge.Face();
        Face F1 = ShapeCasts.Face(aLocalShape);
        aLocalShape = F1.EmptyCopied();
        Face F = ShapeCasts.Face(aLocalShape);
        BrepBuilder BB = new();
        Wire W = new();
        BB.MakeWire(ref W);
        for (; Bit.More(); Bit.Next())
        {
            Edge E = ShapeCasts.Edge(myBlockBuilder.Element(Bit));
            double tolE;
            tolE = Tool.Tolerance(E);
            bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F);
            if (!haspc)
            {
                Curve2D? C2D;
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
    private static Vector3D FUN_tgINE(in Vertex v, in Vertex vl, in Edge e)
    {
        // tg oriented INSIDE 1d(e)
        // vl : last vertex of e
        double par = Tool.Parameter(v, e);
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
    public override Containment CompareShapes(in Shape B1, in Shape B2)
    {
        Containment state = Unknown;

        Explorer ex1 = new(B1, ShapeType.Edge);
        if (!ex1.More())
        {
            return state; // NYIRAISE
        }
        for (; ex1.More(); ex1.Next())
        {
            Edge e1 = ShapeCasts.Edge(ex1.Current());
            Exploration.Vertices(e1, out Vertex vf1, out Vertex vl1); // xpu101198
            bool e1clo = vf1.IsSame(vl1);
            NCollection_IndexedMap<Shape> mapv1 = new(ShapeHasher.Instance);
            mapv1.Add(vf1);
            mapv1.Add(vl1);

            ResetShape(e1);
            bool indy = false;
            Explorer Ex = new();
            for (Ex.Init(B2, ShapeType.Edge); Ex.More(); Ex.Next())
            {
                Edge E = ShapeCasts.Edge(Ex.Current());
                if (E.IsSame(e1))
                {
                    state = Unknown;
                    break;
                } // eap occ416
                Exploration.Vertices(E, out Vertex vf, out Vertex vl); // xpu101198
                bool Eclo = vf.IsSame(vl);                                        // xpu101198
                bool hasf = mapv1.Contains(vf);                                   // xpu101198
                bool hasl = mapv1.Contains(vl);                                   // xpu101198
                bool filter = (hasf || hasl) && (!e1clo) && (!Eclo);              // nyi : Eclo || e1clo
                if (filter)
                { // xpu101198
                    Vertex vshared = new();
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
                    state = Unknown;
                    break;
                }
                CompareElement(E);
                state = State();
            } // ex(B2,EDGE)
            if (state != Unknown)
            {
                break;
            }
        } // ex1

        bool resta = state == Unknown;
        resta = resta && (B2.ShapeType() == ShapeType.Wire) && (B1.ShapeType() == ShapeType.Wire);
        if (resta)
        {
            // classifying first point of B2's first edge not in B1's edges, in face bounded by B1
            NCollection_IndexedMap<Shape> mape1 = new(ShapeHasher.Instance);
            Exploration.MapShapes(B1, ShapeType.Edge, mape1);
            Explorer ex2 = new(B2, ShapeType.Edge);
            for (; ex2.More(); ex2.Next())
            {
                Edge E2 = ShapeCasts.Edge(ex2.Current());
                if (mape1.Contains(E2))
                {
                    continue;
                }

                Face theFace = myBCEdge.Face();
                BrepBuilder BB = new();
                Shape aLocalShape = theFace.Oriented(Forward);
                Face ftmp = ShapeCasts.Face(aLocalShape);
                aLocalShape = ftmp.EmptyCopied();
                Face F2 = ShapeCasts.Face(aLocalShape);
                BB.Add(F2, ShapeCasts.Wire(B2));
                EdgeCurve2DAdaptor BC2d = new(E2, F2);
                TopOpeBRepTool_PROJECT.FUN_tool_bounds(E2, out double f, out double l);
                double x = 0.45678;
                double p2 = (1 - x) * l + x * f;
                Point2D p2d = BC2d.Value(p2);

                aLocalShape = ftmp.EmptyCopied();
                Face F1 = ShapeCasts.Face(aLocalShape);
                BB.Add(F1, ShapeCasts.Wire(B1));
                double tolF1 = Tool.Tolerance(F1);
                BRepClass_FaceClassifier Fclass = new(F1, p2d, tolF1);
                state = Fclass.State();
                return state;
            } // ex2
        }

        return state;
    }

    /// <summary>Classifies the edge <paramref name="EE"/> with the wire <paramref name="B"/>.</summary>
    public override Containment CompareElementToShape(in Shape EE, in Shape B)
    {
        // isvalid edges : ~ (closed edge && ~reachable B)
        ResetElement(EE);
        Explorer Ex = new();
        for (Ex.Init(B, ShapeType.Edge); Ex.More(); Ex.Next())
        {
            Shape E = Ex.Current();
            CompareElement(E);
        }
        Containment state = State();
        return state;
    }

    /// <summary>Prepares a classification involving the wire (or edge) <paramref name="B"/>: ResetElement on its first edge.</summary>
    public override void ResetShape(in Shape B)
    {
        if (B.ShapeType() == ShapeType.Edge)
        {
            ResetElement(B);
        }
        else
        {
            Explorer ex = new(B, ShapeType.Edge);
            if (ex.More())
            {
                Shape E = ex.Current();
                ResetElement(E);
            }
        }
    }

    /// <summary>
    /// Prepares a classification involving the edge <paramref name="EE"/>: the 2D point to classify is on its pcurve
    /// (computed and stored on the edge if it has none).
    /// </summary>
    /// <exception cref="ProgramErrorException">No pcurve could be found.</exception>
    public override void ResetElement(in Shape EE)
    {
        Edge E = ShapeCasts.Edge(EE);
        Face F = myBCEdge.Face();

        // initialize myPoint2d with first point of edge E
        Curve2D? C2D;                                                // jyl980406+
        bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F); // jyl980406+
        if (!haspc)
        { // jyl980406+
            bool trim3d = true;
            C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = Tool.Tolerance(E);                                                       // jyl980406+
            double tol = Max(tolE, tolpc0);                                                             // jyl980406+
            BrepBuilder BB = new();
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
    public override bool CompareElement(in Shape EE)
    {
        bool bRet = true;
        Edge E = ShapeCasts.Edge(EE);
        Face F = myBCEdge.Face();

        Curve2D? C2D;                                                // jyl980402+
        bool haspc = TopOpeBRepTool_2d.FC2D_HasCurveOnSurface(E, F); // jyl980402+
        if (!haspc)
        { // jyl980402+
            bool trim3d = true;
            C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = Tool.Tolerance(E);                                                       // jyl980402+
            double tol = Max(tolE, tolpc0);                                                             // jyl980402+
            BrepBuilder BB = new();
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
        Orientation Eori = E.Orientation();
        myFPC.Compare(myBCEdge, Eori);
        return bRet;
    }

    /// <summary>The state of the 2D point against the edges added.</summary>
    public override Containment State()
    {
        Containment state = myFPC.State();
        return state;
    }
}
