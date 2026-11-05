// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireEdgeClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Build;

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
internal class WireEdgeClassifier : CompositeClassifier
{
    private bool _firstCompare;
    private Point2D _point2d = new();
    private ClassifierEdge _bcEdge = new();
    private readonly FacePassiveClassifier _fpc = new();
    private Shape _shape = new();

    /// <summary>A classifier of the wires and edges of the face <paramref name="f"/>, the blocks being in <paramref name="bb"/>.</summary>
    public WireEdgeClassifier(in Shape f, BlockBuilder bb)
        : base(bb)
    {
        _bcEdge.ChangeFace() = ShapeCasts.Face(f);
    }

    /// <inheritdoc/>
    public override Containment Compare(BuildLoop l1, BuildLoop l2)
    {
        Containment state = Unknown;

        bool isshape1 = l1.IsShape;
        bool isshape2 = l2.IsShape;

        if (isshape2 && isshape1)
        { // L1 is Shape , L2 is Shape
            Shape s1 = l1.Shape();
            Shape s2 = l2.Shape();
            state = CompareShapes(s1, s2);
        }
        else if (isshape2 && !isshape1)
        { // L1 is Block , L2 is Shape
            BlockIterator bit1 = l1.BlockIterator();
            bit1.Initialize();
            bool yena1 = bit1.More();
            while (yena1)
            {
                Shape s1 = _blockBuilder.Element(bit1);
                Shape s2 = l2.Shape();
                state = CompareElementToShape(s1, s2);
                yena1 = false;
                if (state == Unknown)
                {
                    if (bit1.More())
                    {
                        bit1.Next();
                    }
                    yena1 = bit1.More();
                }
            }
        }
        else if (!isshape2 && isshape1)
        { // L1 is Shape , L2 is Block
            Shape s1 = l1.Shape();
            ResetShape(s1);
            BlockIterator bit2 = l2.BlockIterator();
            for (bit2.Initialize(); bit2.More(); bit2.Next())
            {
                Shape s2 = _blockBuilder.Element(bit2);
                CompareElement(s2);
            }
            state = State();
        }
        else if (!isshape2 && !isshape1)
        { // L1 is Block , L2 is Block
            if (state == Unknown)
            {
                BlockIterator bit1 = l1.BlockIterator();
                bit1.Initialize();
                bool yena1 = bit1.More();
                while (yena1)
                {
                    Shape s1 = _blockBuilder.Element(bit1);
                    ResetElement(s1);
                    BlockIterator bit2 = l2.BlockIterator();
                    for (bit2.Initialize(); bit2.More(); bit2.Next())
                    {
                        Shape s2 = _blockBuilder.Element(bit2);
                        CompareElement(s2);
                    }
                    state = State();
                    yena1 = false;
                    if (state == Unknown)
                    {
                        if (bit1.More())
                        {
                            bit1.Next();
                        }
                        yena1 = bit1.More();
                    }
                }
            } // UNKNOWN

            if (state == Unknown)
            {
                Shape s1 = LoopToShape(l1);
                if (s1.IsNull)
                {
                    return state;
                }
                Shape s2 = LoopToShape(l2);
                if (s2.IsNull)
                {
                    return state;
                }
                ShapeClassifier sc = new();
                int samedomain = sc.SameDomain();
                sc.SameDomain(1);
                sc.SetReference(s2);
                Shape avS = s2;
                state = sc.StateShapeReference(s1, avS);
                sc.SameDomain(samedomain);
            } // UNKNOWN
        }

        return state;
    }

    /// <summary>
    /// A face on the classifier's surface bounded by a wire of the block's edges (null for an empty block). Edges
    /// without a pcurve on the face get one.
    /// </summary>
    public Shape LoopToShape(BuildLoop l)
    {
        _shape.Nullify();
        BlockIterator bit = l.BlockIterator();
        bit.Initialize();
        if (!bit.More())
        {
            return _shape;
        }

        Shape localShape = _bcEdge.Face();
        Face f1 = ShapeCasts.Face(localShape);
        localShape = f1.EmptyCopied();
        Face f = ShapeCasts.Face(localShape);
        BrepBuilder bb = new();
        Wire w = new();
        bb.MakeWire(ref w);
        for (; bit.More(); bit.Next())
        {
            Edge e = ShapeCasts.Edge(_blockBuilder.Element(bit));
            double tolE;
            tolE = Tool.Tolerance(e);
            bool haspc = Legacy2D.FC2D_HasCurveOnSurface(e, f);
            if (!haspc)
            {
                Curve2D? c2D;
                c2D = Legacy2D.FC2D_CurveOnSurface(e, f, out _, out _, out double tolpc);
                if (c2D is not null)
                {
                    double tol = Max(tolpc, tolE);
                    bb.UpdateEdge(e, c2D, f, tol);
                }
            }
            bb.Add(w, e);
        }
        bb.Add(f, w);
        _shape = f;
        return _shape;
    }

    // The unit tangent at v of e, pointing into the edge
    private static Vector3D FUN_tgINE(in Vertex v, in Vertex vl, in Edge e)
    {
        // tg oriented INSIDE 1d(e)
        // vl : last vertex of e
        double par = Tool.Parameter(v, e);
        bool ok = LegacyTool.TggeomE(par, e, out Vector3D tg);
        if (!ok)
        {
            return new Vector3D(0.0, 0.0, 0.0); // NYIRAISE
        }
        if (v.IsSame(vl))
        {
            tg = tg.Reversed();
        }
        return tg;
    }

    /// <summary>
    /// Classifies the wire <paramref name="b1"/> with the wire <paramref name="b2"/>: an edge of B1 against B2's edges;
    /// if undecided, a point of an edge of B2 that isn't in B1, in the face bounded by B1.
    /// </summary>
    public override Containment CompareShapes(in Shape b1, in Shape b2)
    {
        Containment state = Unknown;

        Explorer ex1 = new(b1, ShapeType.Edge);
        if (!ex1.More())
        {
            return state; // NYIRAISE
        }
        for (; ex1.More(); ex1.Next())
        {
            Edge e1 = ShapeCasts.Edge(ex1.Current());
            Exploration.Vertices(e1, out Vertex vf1, out Vertex vl1); // xpu101198
            bool e1clo = vf1.IsSame(vl1);
            IndexedSet<Shape> mapv1 = new(ShapeHasher.Instance);
            mapv1.Add(vf1);
            mapv1.Add(vl1);

            ResetShape(e1);
            bool indy = false;
            Explorer ex = new();
            for (ex.Init(b2, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge e = ShapeCasts.Edge(ex.Current());
                if (e.IsSame(e1))
                {
                    state = Unknown;
                    break;
                } // eap occ416
                Exploration.Vertices(e, out Vertex vf, out Vertex vl); // xpu101198
                bool eclo = vf.IsSame(vl);                                        // xpu101198
                bool hasf = mapv1.Contains(vf);                                   // xpu101198
                bool hasl = mapv1.Contains(vl);                                   // xpu101198
                bool filter = (hasf || hasl) && (!e1clo) && (!eclo);              // nyi : Eclo || e1clo
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
                    Vector3D tg = FUN_tgINE(vshared, vl, e);
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
                CompareElement(e);
                state = State();
            } // ex(B2,EDGE)
            if (state != Unknown)
            {
                break;
            }
        } // ex1

        bool resta = state == Unknown;
        resta = resta && (b2.Type == ShapeType.Wire) && (b1.Type == ShapeType.Wire);
        if (resta)
        {
            // classifying first point of B2's first edge not in B1's edges, in face bounded by B1
            IndexedSet<Shape> mape1 = new(ShapeHasher.Instance);
            Exploration.MapShapes(b1, ShapeType.Edge, mape1);
            Explorer ex2 = new(b2, ShapeType.Edge);
            for (; ex2.More(); ex2.Next())
            {
                Edge e2 = ShapeCasts.Edge(ex2.Current());
                if (mape1.Contains(e2))
                {
                    continue;
                }

                Face theFace = _bcEdge.Face();
                BrepBuilder bb = new();
                Shape localShape = theFace.Oriented(Forward);
                Face ftmp = ShapeCasts.Face(localShape);
                localShape = ftmp.EmptyCopied();
                Face f2 = ShapeCasts.Face(localShape);
                bb.Add(f2, ShapeCasts.Wire(b2));
                EdgeCurve2DAdaptor bc2d = new(e2, f2);
                Project.FUN_tool_bounds(e2, out double f, out double l);
                double x = 0.45678;
                double p2 = (1 - x) * l + x * f;
                Point2D p2d = bc2d.Value(p2);

                localShape = ftmp.EmptyCopied();
                Face f1 = ShapeCasts.Face(localShape);
                bb.Add(f1, ShapeCasts.Wire(b1));
                double tolF1 = Tool.Tolerance(f1);
                FaceClassifier fclass = new(f1, p2d, tolF1);
                state = fclass.State();
                return state;
            } // ex2
        }

        return state;
    }

    /// <summary>Classifies the edge <paramref name="ee"/> with the wire <paramref name="b"/>.</summary>
    public override Containment CompareElementToShape(in Shape ee, in Shape b)
    {
        // isvalid edges : ~ (closed edge && ~reachable B)
        ResetElement(ee);
        Explorer ex = new();
        for (ex.Init(b, ShapeType.Edge); ex.More(); ex.Next())
        {
            Shape e = ex.Current();
            CompareElement(e);
        }
        Containment state = State();
        return state;
    }

    /// <summary>Prepares a classification involving the wire (or edge) <paramref name="b"/>: ResetElement on its first edge.</summary>
    public override void ResetShape(in Shape b)
    {
        if (b.Type == ShapeType.Edge)
        {
            ResetElement(b);
        }
        else
        {
            Explorer ex = new(b, ShapeType.Edge);
            if (ex.More())
            {
                Shape e = ex.Current();
                ResetElement(e);
            }
        }
    }

    /// <summary>
    /// Prepares a classification involving the edge <paramref name="ee"/>: the 2D point to classify is on its pcurve
    /// (computed and stored on the edge if it has none).
    /// </summary>
    /// <exception cref="ProgramErrorException">No pcurve could be found.</exception>
    public override void ResetElement(in Shape ee)
    {
        Edge e = ShapeCasts.Edge(ee);
        Face f = _bcEdge.Face();

        // initialize myPoint2d with first point of edge E
        Curve2D? c2D;                                                // jyl980406+
        bool haspc = Legacy2D.FC2D_HasCurveOnSurface(e, f); // jyl980406+
        if (!haspc)
        { // jyl980406+
            bool trim3d = true;
            c2D = Legacy2D.FC2D_CurveOnSurface(e, f, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = Tool.Tolerance(e);                                                       // jyl980406+
            double tol = Max(tolE, tolpc0);                                                             // jyl980406+
            BrepBuilder bb = new();
            bb.UpdateEdge(e, c2D, f, tol); // jyl980406+
        } // jyl980406+

        c2D = Legacy2D.FC2D_CurveOnSurface(e, f, out double f2, out double l2, out _);
        if (c2D is null)
        {
            throw new ProgramErrorException("WEC : ResetElement");
        }

        double t = 0.397891143689;
        double par = (1 - t) * f2 + t * l2;
        _point2d = c2D.Value(par);

        _firstCompare = true;
    }

    /// <summary>
    /// Adds the edge <paramref name="ee"/> to the edges the 2D point is classified against (its pcurve computed and
    /// stored on the edge if it has none). Always true.
    /// </summary>
    public override bool CompareElement(in Shape ee)
    {
        bool bRet = true;
        Edge e = ShapeCasts.Edge(ee);
        Face f = _bcEdge.Face();

        Curve2D? c2D;                                                // jyl980402+
        bool haspc = Legacy2D.FC2D_HasCurveOnSurface(e, f); // jyl980402+
        if (!haspc)
        { // jyl980402+
            bool trim3d = true;
            c2D = Legacy2D.FC2D_CurveOnSurface(e, f, out _, out _, out double tolpc0, trim3d); // jyl980406+
            double tolE = Tool.Tolerance(e);                                                       // jyl980402+
            double tol = Max(tolE, tolpc0);                                                             // jyl980402+
            BrepBuilder bb = new();
            bb.UpdateEdge(e, c2D, f, tol); // jyl980402+
        } // jyl980402+

        if (_firstCompare)
        {
            c2D = Legacy2D.FC2D_CurveOnSurface(e, f, out double f2, out double l2, out _);
            double t = 0.33334567;
            double par = (1 - t) * f2 + t * l2;
            Point2D p2d = c2D!.Value(par);

            Vector2D v2d = new(_point2d, p2d);
            Line2D l2d = new(_point2d, new Direction2D(v2d));
            double dist = _point2d.DistanceTo(p2d);
            double tol2d = global::Kerf.Engine.Foundation.Precision.PConfusion(); // NYI : a voir
            _fpc.Reset(l2d, dist, tol2d);
            _firstCompare = false;
        }

        _bcEdge.ChangeEdge() = e;
        Orientation eori = e.Orientation;
        _fpc.Compare(_bcEdge, eori);
        return bRet;
    }

    /// <summary>The state of the 2D point against the edges added.</summary>
    public override Containment State()
    {
        Containment state = _fpc.State();
        return state;
    }
}
