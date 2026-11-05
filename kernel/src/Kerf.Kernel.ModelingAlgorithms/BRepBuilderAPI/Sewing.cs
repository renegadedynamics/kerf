// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing;

/// <summary>
/// Identifies the possible contiguous boundaries of shapes (for a later control of continuity) and assembles the
/// contiguous shapes into one shape. Only manifold shapes are found, and nothing is sewn along multiple edges unless the
/// non-manifold mode is set.
/// </summary>
/// <remarks>
/// <para>To sew: create the object (default tolerance 1e-6, face analysis, sewing and cutting on), add the shapes with
/// <see cref="Add"/>, call <see cref="Perform"/>, then read the sewn shape, the free, contiguous, multiple and
/// degenerated edges and the modifications.</para>
/// <para>A working object (decision 21). What it hands out is frozen (docs/phase3-design.md, section 4.2):
/// <see cref="SewedShape"/>, the edges, faces and lists of the queries and <see cref="Modified"/> and
/// <see cref="ModifiedSubShape"/>. The internal forms (<see cref="InternalSewedShape"/>, ...) don't freeze, for ported
/// code that goes on building.</para>
/// <para>Sewing writes into the shapes it is given (OCCT works in place on its inputs and records the replacements in
/// its context). <see cref="Perform"/> runs on the inputs themselves in a working overlay (Kerf unit 10H4; decision
/// 583), which takes the writes into frozen shapes; afterwards the result and every answer hold new TShapes for what
/// sewing changed and the caller's own sub-shapes for the rest. The queries take the caller's shapes or the shapes they
/// handed out. The context (<see cref="GetContext"/>) records the caller's sub-shapes, as OCCT's records its inputs'.
/// Inside another operation's overlay, sewing runs in place in that one.</para>
/// </remarks>
internal class Sewing : Transient
{
    protected double _tolerance;
    protected bool _sewing;
    protected bool _analysis;
    protected bool _cutting;
    protected bool _nonmanifold;
    protected IndexedMap<Shape, Shape> _oldShapes = new(ShapeHasher.Instance);
    protected Shape _sewedShape = new();
    protected IndexedSet<Shape> _degenerated = new(ShapeHasher.Instance);
    protected IndexedSet<Shape> _freeEdges = new(ShapeHasher.Instance);
    protected IndexedSet<Shape> _multipleEdges = new(ShapeHasher.Instance);
    protected IndexedMap<Shape, NCollection_List<Shape>> _contigousEdges =
        new(ShapeHasher.Instance);
    protected Dictionary<Shape, Shape> _contigSecBound = new(ShapeHasher.Instance);
    protected int _nbShapes;
    protected int _nbVertices;
    protected int _nbEdges;
    protected IndexedMap<Shape, NCollection_List<Shape>> _boundFaces =
        new(ShapeHasher.Instance);
    protected Dictionary<Shape, NCollection_List<Shape>> _boundSections =
        new(ShapeHasher.Instance);
    protected Dictionary<Shape, Shape> _sectionBound = new(ShapeHasher.Instance);
    protected IndexedMap<Shape, Shape> _vertexNode = new(ShapeHasher.Instance);
    protected IndexedMap<Shape, Shape> _vertexNodeFree = new(ShapeHasher.Instance);
    protected Dictionary<Shape, NCollection_List<Shape>> _nodeSections =
        new(ShapeHasher.Instance);
    protected Dictionary<Shape, NCollection_List<Shape>> _cuttingNode =
        new(ShapeHasher.Instance);
    protected IndexedSet<Shape> _littleFace = new(ShapeHasher.Instance);
    protected Shape _shape = new();
    protected ReShape _reShape;

    private bool _faceMode;
    private bool _floatingEdgesMode;
    private bool _sameParameterMode;
    private bool _localToleranceMode;
    private double _minTolerance;
    private double _maxTolerance;
    private readonly HashSet<Shape> _mergedEdges = new(ShapeHasher.Instance);

    // Kerf: the working overlay of the last Perform (unit 10H4; decision 583), finished; null when sewing ran in place
    // (inside another operation's overlay) or before Perform. The answers map what sewing worked with through it.
    private WorkingOverlay? _overlay;

    /// <summary>
    /// Creates an object with the tolerance of connexity <paramref name="tolerance"/> and the options for sewing
    /// (<paramref name="option1"/>; if false, only control), analysis of degenerated shapes (<paramref name="option2"/>),
    /// cutting of free edges (<paramref name="option3"/>) and non-manifold processing (<paramref name="option4"/>).
    /// </summary>
    public Sewing(double tolerance = 1.0e-06,
                                 bool option1 = true,
                                 bool option2 = true,
                                 bool option3 = true,
                                 bool option4 = false)
    {
        _reShape = new ReShape();
        Init(tolerance, option1, option2, option3, option4);
    }

    //=================================================================================================
    // BRepBuilderAPI_Sewing.lxx

    /// <summary>Gets the same parameter mode.</summary>
    public bool SameParameterMode() => _sameParameterMode;

    /// <summary>Sets the same parameter mode.</summary>
    public void SetSameParameterMode(bool sameParameterMode) => _sameParameterMode = sameParameterMode;

    /// <summary>The tolerance set.</summary>
    public double Tolerance() => _tolerance;

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double toler) => _tolerance = toler;

    /// <summary>The min tolerance set.</summary>
    public double MinTolerance() => _minTolerance;

    /// <summary>Sets the min tolerance.</summary>
    public void SetMinTolerance(double minToler) => _minTolerance = minToler;

    /// <summary>The max tolerance set.</summary>
    public double MaxTolerance() => _maxTolerance;

    /// <summary>Sets the max tolerance.</summary>
    public void SetMaxTolerance(double maxToler) => _maxTolerance = maxToler;

    /// <summary>The mode for sewing faces (true by default).</summary>
    public bool FaceMode() => _faceMode;

    /// <summary>Sets the mode for sewing faces (true by default).</summary>
    public void SetFaceMode(bool faceMode) => _faceMode = faceMode;

    /// <summary>The mode for sewing floating edges (false by default).</summary>
    public bool FloatingEdgesMode() => _floatingEdgesMode;

    /// <summary>Sets the mode for sewing floating edges (false by default).</summary>
    public void SetFloatingEdgesMode(bool floatingEdgesMode) => _floatingEdgesMode = floatingEdgesMode;

    /// <summary>The mode for accounting of local tolerances of edges and vertices during merging.</summary>
    public bool LocalTolerancesMode() => _localToleranceMode;

    /// <summary>
    /// Sets the mode for accounting of local tolerances of edges and vertices during merging; then the working
    /// tolerance is myTolerance + tolEdge1 + tolEdge2.
    /// </summary>
    public void SetLocalTolerancesMode(bool localTolerancesMode) => _localToleranceMode = localTolerancesMode;

    /// <summary>Sets the mode for non-manifold sewing.</summary>
    public void SetNonManifoldMode(bool nonManifoldMode) => _nonmanifold = nonManifoldMode;

    /// <summary>The mode for non-manifold sewing.</summary>
    public bool NonManifoldMode() => _nonmanifold;

    //=================================================================================================
    // OCCT.NET: publication and the working overlay (decision 583; Kerf unit 10H4).

    // The shape sewing worked with for a shape a caller passes: one it handed out stands for the shape it replaced.
    private Shape ToWork(in Shape shape) => _overlay is null || shape.IsNull ? shape : _overlay.Initial(shape);

    // The shape a caller sees for a shape sewing worked with: what stands for it after the overlay.
    private Shape FromWork(in Shape shape) => _overlay is null || shape.IsNull ? shape : _overlay.Final(shape);

    // A working shape handed to a caller: the caller's view of it (fromWork), frozen.
    private Shape Published(in Shape shape)
    {
        Shape aShape = FromWork(shape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    // Every shape the answers may hand out, for the overlay to assemble: the inputs' images, the edges and faces of the
    // queries and the context's replacements.
    private List<Shape> Answers()
    {
        var shapes = new List<Shape>();
        for (int i = 0; i < _oldShapes.Count; i++)
        {
            shapes.Add(_oldShapes.ValueAt(i));
        }
        for (int i = 0; i < _degenerated.Count; i++)
        {
            shapes.Add(_degenerated[i]);
        }
        for (int i = 0; i < _freeEdges.Count; i++)
        {
            shapes.Add(_freeEdges[i]);
        }
        for (int i = 0; i < _multipleEdges.Count; i++)
        {
            shapes.Add(_multipleEdges[i]);
        }
        for (int i = 0; i < _contigousEdges.Count; i++)
        {
            shapes.Add(_contigousEdges.KeyAt(i));
            for (var it = new NCollection_List<Shape>.Iterator(_contigousEdges.ValueAt(i)); it.More(); it.Next())
            {
                shapes.Add(it.Value());
            }
        }
        foreach (KeyValuePair<Shape, Shape> pair in _contigSecBound)
        {
            shapes.Add(pair.Value);
        }
        for (int i = 0; i < _boundFaces.Count; i++)
        {
            for (var it = new NCollection_List<Shape>.Iterator(_boundFaces.ValueAt(i)); it.More(); it.Next())
            {
                shapes.Add(it.Value());
            }
        }
        for (int i = 0; i < _littleFace.Count; i++)
        {
            shapes.Add(_littleFace[i]);
        }
        shapes.AddRange(_reShape.InternalReplacements());
        return shapes;
    }

    //=================================================================================================

    /// <summary>The pcurve given the range RequestedFirst to RequestedLast (GeomLib.SameRange); null on failure.</summary>
    /// <remarks>Called from SameParameterEdge only.</remarks>
    protected virtual Curve2D? SameRange(Curve2D? curvePtr,
                                              double firstOnCurve,
                                              double lastOnCurve,
                                              double requestedFirst,
                                              double requestedLast)
    {
        Curve2D? newCurvePtr = null;
        try
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                              curvePtr,
                              firstOnCurve,
                              lastOnCurve,
                              requestedFirst,
                              requestedLast,
                              ref newCurvePtr);
        }
        catch (FailureException)
        {
        }
        return newCurvePtr;
    }

    //=======================================================================
    // function : WhichFace
    // purpose  : Give the face whose edge is the border
    //=======================================================================

    /// <summary>The <paramref name="index"/>-th face whose border is the edge (or the edge's bound), frozen; a null face if none.</summary>
    public Face WhichFace(in Edge edg, int index = 1) =>
        ShapeCasts.Face(Published(InternalWhichFace(ShapeCasts.Edge(ToWork(edg)), index)));

    /// <summary><see cref="WhichFace"/> on the working shapes, without freezing (ported code).</summary>
    internal Face InternalWhichFace(in Edge edg, int index = 1)
    {
        Shape bound = edg;
        if (_sectionBound.ContainsKey(bound))
        {
            bound = _sectionBound.GetValue(bound);
        }
        if (_boundFaces.ContainsKey(bound))
        {
            int i = 1;
            var itf = new NCollection_List<Shape>.Iterator(_boundFaces.GetValue(bound));
            for (; itf.More(); itf.Next(), i++)
            {
                if (i == index)
                {
                    return ShapeCasts.Face(itf.Value());
                }
            }
        }
        return new Face();
    }

    //=================================================================================================

    private static bool IsClosedShape(in Shape theshape, in Shape v1, in Shape v2)
    {
        double totLength = 0.0;
        var aexp = new Explorer();
        for (aexp.Init(theshape, ShapeType.Edge); aexp.More(); aexp.Next())
        {
            Edge aedge = ShapeCasts.Edge(aexp.Current());
            if (aedge.IsNull)
            {
                continue;
            }
            Exploration.Vertices(aedge, out Vertex ve1, out Vertex ve2);
            if (!ve1.IsSame(v1) && !ve1.IsSame(v2))
            {
                continue;
            }
            if (Tool.Degenerated(aedge))
            {
                continue;
            }
            Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(aedge), out double first, out double last);
            if (c3d is not null)
            {
                var cAdapt = new CurveAdaptor(c3d);
                double length = AbscissaPoint.Length(cAdapt, first, last);
                totLength += length;
                if (ve2.IsSame(v1) || ve2.IsSame(v2))
                {
                    break;
                }
            }
        }
        if (totLength > 0.0)
        {
            Point3D p1 = Tool.Pnt(ShapeCasts.Vertex(v1));
            Point3D p2 = Tool.Pnt(ShapeCasts.Vertex(v2));
            return (p1.DistanceTo(p2) < totLength / (1.2 * M_PI));
        }
        return false;
    }

    //=================================================================================================

    private static bool IsClosedByIsos(Surface thesurf,
                                       Curve2D acrv2d,
                                       double f2d,
                                       double l2d,
                                       bool isUIsos)
    {
        bool isClosed = false;

        Point2D psurf1 = (acrv2d.IsPeriodic ? acrv2d.Value(f2d)
                                               : acrv2d.Value(Max(f2d, acrv2d.FirstParameter)));
        Point2D psurf2 = (acrv2d.IsPeriodic ? acrv2d.Value(l2d)
                                               : acrv2d.Value(Min(l2d, acrv2d.LastParameter)));
        Curve3D crv1;
        Curve3D crv2;
        if (isUIsos)
        {
            crv1 = thesurf.UIso(psurf1.X);
            crv2 = thesurf.UIso(psurf2.X);
        }
        else
        {
            crv1 = thesurf.VIso(psurf1.Y);
            crv2 = thesurf.VIso(psurf2.Y);
        }
        Point3D p11 = new(), p1m = new(), p12 = new(), p21 = new(), p2m = new(), p22 = new();
        double af1 = crv1.FirstParameter;
        double al1 = crv1.LastParameter;
        double af2 = crv2.FirstParameter;
        double al2 = crv2.LastParameter;
        crv1.D0(af1, out p11);
        crv1.D0((af1 + al1) * 0.5, out p1m);
        crv1.D0(al1, out p12);
        crv2.D0(af2, out p21);
        crv2.D0((af2 + al2) * 0.5, out p2m);
        crv2.D0(al2, out p22);
        isClosed = (((p11.XYZ() - p12.XYZ()).Modulus()
                     < (p11.XYZ() - p1m.XYZ()).Modulus() - global::Kerf.Engine.Foundation.Precision.Confusion())
                    && ((p21.XYZ() - p22.XYZ()).Modulus()
                        < (p21.XYZ() - p2m.XYZ()).Modulus() - global::Kerf.Engine.Foundation.Precision.Confusion()));
        return isClosed;
    }

    //=================================================================================================

    /// <summary>Defines if the surface is U-closed (along the edge).</summary>
    protected virtual bool IsUClosedSurface(Surface surf, in Shape edge, in Placement theloc)
    {
        Surface tmpsurf = surf;
        if (tmpsurf is TrimmedSurface rts)
        {
            tmpsurf = rts.BasisSurface;
        }
        else if (tmpsurf is OffsetSurface os)
        {
            tmpsurf = os.BasisSurface;
        }
        else
        {
            bool isClosed = tmpsurf.IsUClosed;
            if (!isClosed)
            {
                Curve2D? acrv2d =
                    Tool.CurveOnSurface(ShapeCasts.Edge(edge), surf, theloc, out double f2d, out double l2d);
                if (acrv2d is not null)
                {
                    isClosed = IsClosedByIsos(tmpsurf, acrv2d, f2d, l2d, false);
                }
            }
            return isClosed;
        }
        return IsUClosedSurface(tmpsurf, edge, theloc);
    }

    //=================================================================================================

    /// <summary>Defines if the surface is V-closed (along the edge).</summary>
    protected virtual bool IsVClosedSurface(Surface surf, in Shape edge, in Placement theloc)
    {
        Surface tmpsurf = surf;
        if (tmpsurf is TrimmedSurface rts)
        {
            tmpsurf = rts.BasisSurface;
        }
        else if (tmpsurf is OffsetSurface os)
        {
            tmpsurf = os.BasisSurface;
        }
        else
        {
            bool isClosed = tmpsurf.IsVClosed;
            if (!isClosed)
            {
                Curve2D? acrv2d =
                    Tool.CurveOnSurface(ShapeCasts.Edge(edge), surf, theloc, out double f2d, out double l2d);
                if (acrv2d is not null)
                {
                    isClosed = IsClosedByIsos(tmpsurf, acrv2d, f2d, l2d, true);
                }
            }
            return isClosed;
        }
        return IsVClosedSurface(tmpsurf, edge, theloc);
    }

    //=================================================================================================

    /// <summary>BRepLib.SameParameter on the edge, failures ignored. Called from SameParameterEdge only.</summary>
    protected virtual void SameParameter(in Edge edge)
    {
        try
        {
            BrepLibrary.SameParameter(edge);
        }
        catch (FailureException)
        {
        }
    }

    //=======================================================================
    // function : SameParameterEdge
    // purpose  : internal use
    //           Merge the Sequence Of Section on one edge.
    //           This function keep the curve3d,curve2d,range and parametrization
    //           from the first section, and report and made sameparameter the
    //           pcurves of the other function.
    //           This function works when the are not more than two Pcurves
    //           on a same face.
    //=======================================================================

    /// <summary>Merges the sections <paramref name="seqEdges"/> with <paramref name="edge"/>. Called from Merging only.</summary>
    protected virtual Edge SameParameterEdge(in Shape edge,
                                                    NCollection_Sequence<Shape> seqEdges,
                                                    NCollection_Sequence<bool> seqForward,
                                                    HashSet<Shape> mapMerged,
                                                    ReShape locReShape)
    {
        // Retrieve reference section
        Shape tmpShape = _reShape.InternalApply(edge); // for porting
        Edge edge1 = ShapeCasts.Edge(tmpShape);
        tmpShape = locReShape.InternalApply(edge1);
        if (locReShape != _reShape)
        {
            edge1 = ShapeCasts.Edge(tmpShape);
        }
        bool isDone = false;

        // Create data structures for temporary merged edges
        var listFaces1 = new NCollection_List<Shape>();
        var mergedFaces = new HashSet<Shape>(ShapeHasher.Instance);

        if (_sewing)
        {
            // Fill MergedFaces with faces of Edge1
            Shape bnd1 = edge;
            if (_sectionBound.ContainsKey(bnd1))
            {
                bnd1 = _sectionBound.GetValue(bnd1);
            }
            if (_boundFaces.ContainsKey(bnd1))
            {
                var itf = new NCollection_List<Shape>.Iterator(_boundFaces.GetValue(bnd1));
                for (; itf.More(); itf.Next())
                {
                    if (mergedFaces.Add(itf.Value()))
                    {
                        listFaces1.Append(itf.Value());
                    }
                }
            }
        }
        else
        {
            // Create presentation edge
            Exploration.Vertices(edge1, out Vertex v1, out Vertex v2);
            if (_vertexNode.ContainsKey(v1))
            {
                v1 = ShapeCasts.Vertex(_vertexNode.GetValue(v1));
            }
            if (_vertexNode.ContainsKey(v2))
            {
                v2 = ShapeCasts.Vertex(_vertexNode.GetValue(v2));
            }

            Edge newEdge = edge1;
            newEdge.EmptyCopy();

            // Add the vertices
            var builder = new BrepBuilder();
            Shape anEdge = newEdge.Oriented(Forward);
            builder.Add(anEdge, v1.Oriented(Forward));
            builder.Add(anEdge, v2.Oriented(Reversed));

            edge1 = newEdge;
        }

        bool isForward = true;

        // Merge candidate sections
        for (int i = 1; i <= seqEdges.Length(); i++)
        {
            // Retrieve candidate section
            Shape oedge2 = seqEdges[i];

            if (_sewing)
            {
                tmpShape = _reShape.InternalApply(oedge2); // for porting
                Edge edge2 = ShapeCasts.Edge(tmpShape);
                tmpShape = locReShape.InternalApply(edge2);
                if (locReShape != _reShape)
                {
                    edge2 = ShapeCasts.Edge(tmpShape);
                }

                // Calculate relative orientation
                bool orientation = seqForward[i];
                if (!isForward)
                {
                    orientation = !orientation;
                }

                // Retrieve faces information for the second edge
                Shape bnd2 = oedge2;
                if (_sectionBound.ContainsKey(bnd2))
                {
                    bnd2 = _sectionBound.GetValue(bnd2);
                }
                if (!_boundFaces.ContainsKey(bnd2))
                {
                    continue; // Skip floating edge
                }
                NCollection_List<Shape> listFaces2 = _boundFaces.GetValue(bnd2);

                int whichSec = 1; // Indicates on which edge the pCurve has been reported
                Edge newEdge =
                    SameParameterEdge(edge1, edge2, listFaces1, listFaces2, orientation, ref whichSec);
                if (newEdge.IsNull)
                {
                    continue;
                }

                // Record faces information for the temporary merged edge
                var itf = new NCollection_List<Shape>.Iterator(listFaces2);
                for (; itf.More(); itf.Next())
                {
                    if (mergedFaces.Add(itf.Value()))
                    {
                        listFaces1.Append(itf.Value());
                    }
                }

                // Record merged section orientation
                if (!orientation && whichSec != 1)
                {
                    isForward = !isForward;
                }
                edge1 = newEdge;
            }

            // Append actually merged edge
            mapMerged.Add(oedge2);
            isDone = true;

            if (!_nonmanifold)
            {
                break;
            }
        }

        if (isDone)
        {
            // Change result orientation
            edge1.Orient(isForward ? Forward : Reversed);
        }
        else
        {
            edge1.Nullify();
        }

        return edge1;
    }

    //=================================================================================================

    private static bool FindNMVertices(in Edge edge,
                                       NCollection_Sequence<Shape> seqNMVert,
                                       NCollection_Sequence<double> seqPars)
    {
        var itV = new Iterator(edge, false);
        for (; itV.More(); itV.Next())
        {
            if (itV.Value().Orientation == Internal
                || itV.Value().Orientation == External)
            {
                seqNMVert.Append(itV.Value());
            }
        }
        int nbV = seqNMVert.Length();
        if (nbV == 0)
        {
            return false;
        }
        Curve3D? c3d = Tool.Curve(edge, out double first, out double last);
        var gac = new CurveAdaptor(c3d!);
        var locProj = new CurvePointExtrema();
        locProj.Initialize(gac, first, last);
        Point3D pfirst = gac.Value(first), plast = gac.Value(last);

        for (int i = 1; i <= nbV; i++)
        {
            Vertex v = ShapeCasts.Vertex(seqNMVert.Value(i));
            Point3D pt = Tool.Pnt(v);

            double distF2 = pfirst.SquareDistanceTo(pt);
            double distL2 = plast.SquareDistanceTo(pt);
            double apar = (distF2 > distL2 ? last : first);
            // Project current point on curve
            locProj.Perform(pt);
            if (locProj.IsDone && locProj.ExtCount > 0)
            {
                double dist2Min = Min(distF2, distL2);
                int ind, indMin = 0;
                for (ind = 1; ind <= locProj.ExtCount; ind++)
                {
                    double dProj2 = locProj.SquareDistance(ind);
                    if (dProj2 < dist2Min)
                    {
                        indMin = ind;
                        dist2Min = dProj2;
                    }
                }
                if (indMin != 0)
                {
                    apar = locProj.Point(indMin).Parameter();
                }

                seqPars.Append(apar);
            }
        }
        return true;
    }

    private static void ComputeToleranceVertex(Vertex v1, Vertex v2, ref Vertex newV)
    {
        int m, n;
        double dR, d, eps;
        Span<double> r = stackalloc double[2];
        var v = new Vertex[2];
        var p = new Point3D[2];
        var bb = new BrepBuilder();
        //
        eps = RealEpsilon();
        v[0] = v1;
        v[1] = v2;
        for (m = 0; m < 2; ++m)
        {
            p[m] = Tool.Pnt(v[m]);
            r[m] = Tool.Tolerance(v[m]);
        }
        //
        m = 0; // max R
        n = 1; // min R
        if (r[0] < r[1])
        {
            m = 1;
            n = 0;
        }
        //
        dR = r[m] - r[n]; // dR >= 0.
        var vd = new Vector3D(p[m], p[n]);
        d = vd.Magnitude;
        //
        if (d <= dR || d < eps)
        {
            bb.MakeVertex(ref newV, p[m], r[m]);
        }
        else
        {
            double rr;
            Xyz xyZr;
            var pr = new Point3D();
            //
            rr = 0.5 * (r[m] + r[n] + d);
            xyZr = 0.5 * (p[m].XYZ() + p[n].XYZ() - vd.XYZ() * (dR / d));
            pr = pr.WithXYZ(xyZr);
            //
            bb.MakeVertex(ref newV, pr, rr);
        }
    }

    private static void ComputeToleranceVertex(Vertex v1,
                                               Vertex v2,
                                               Vertex v3,
                                               ref Vertex newV)
    {
        double di, dmax;
        var center = new Point3D();
        var p = new Point3D[3];
        Span<double> r = stackalloc double[3];
        var v = new Vertex[3];
        var xyz = new Xyz(0.0, 0.0, 0.0);
        v[0] = v1;
        v[1] = v2;
        v[2] = v3;
        for (int i = 0; i < 3; ++i)
        {
            p[i] = Tool.Pnt(v[i]);
            r[i] = Tool.Tolerance(v[i]);
            xyz = xyz + p[i].XYZ();
        }
        //
        xyz.Divide(3.0);
        center = center.WithXYZ(xyz);
        //
        dmax = -1.0;
        for (int i = 0; i < 3; ++i)
        {
            di = center.DistanceTo(p[i]);
            di += r[i];
            if (di > dmax)
            {
                dmax = di;
            }
        }

        var bb = new BrepBuilder();
        bb.MakeVertex(ref newV, center, dmax);
    }

    /// <summary>
    /// Merges two sections into a new edge keeping the 3D curve, range and parametrisation of one of them (the longer
    /// one at the first call; <paramref name="whichSec"/> tells which) and the pcurves of both. Called from Merging only.
    /// </summary>
    protected virtual Edge SameParameterEdge(in Edge edgeFirst,
                                                    in Edge edgeLast,
                                                    NCollection_List<Shape> listFacesFirst,
                                                    NCollection_List<Shape> listFacesLast,
                                                    bool secForward,
                                                    ref int whichSec,
                                                    bool firstCall = true)
    {
        // Do not process floating edges
        if (listFacesFirst.Extent() == 0 || listFacesLast.Extent() == 0)
        {
            return new Edge();
        }

        // Sort input edges
        Edge edge1 = new(), edge2 = new();
        if (firstCall)
        {
            // Take the longest edge as first
            Curve3D? c3d1 = Tool.Curve(ShapeCasts.Edge(edgeFirst), out double f, out double l);
            var cAdapt1 = new CurveAdaptor(c3d1!);
            double len1 = AbscissaPoint.Length(cAdapt1, f, l);
            Curve3D? c3d2 = Tool.Curve(ShapeCasts.Edge(edgeLast), out f, out l);
            var cAdapt2 = new CurveAdaptor(c3d2!);
            double len2 = AbscissaPoint.Length(cAdapt2, f, l);
            if (len1 < len2)
            {
                edge1 = edgeLast;
                edge2 = edgeFirst;
                whichSec = 2;
            }
            else
            {
                edge1 = edgeFirst;
                edge2 = edgeLast;
                whichSec = 1;
            }
        }
        else
        {
            if (whichSec == 1)
            {
                edge1 = edgeLast;
                edge2 = edgeFirst;
                whichSec = 2;
            }
            else
            {
                edge1 = edgeFirst;
                edge2 = edgeLast;
                whichSec = 1;
            }
        }

        Tool.Range(edge1, out double first, out double last);
        var builder = new BrepBuilder();

        // To keep NM vertices on edge
        var seqNMVert = new NCollection_Sequence<Shape>();
        var seqNMPars = new NCollection_Sequence<double>();
        FindNMVertices(edge1, seqNMVert, seqNMPars);
        FindNMVertices(edge2, seqNMVert, seqNMPars);

        // Create new edge
        var edge = new Edge();
        builder.MakeEdge(ref edge);
        edge.Orient(edge1.Orientation);

        // Retrieve edge curve
        Curve3D? c3d = Tool.Curve(edge1, out Placement loc3d, out double first3d, out double last3d);
        if (!loc3d.IsIdentity)
        {
            c3d = (Curve3D)c3d!.Copy();
            c3d.Transform(loc3d.Transformation);
        }
        builder.UpdateEdge(edge, c3d, Tool.Tolerance(edge1));
        builder.Range(edge, first, last);
        builder.SameRange(edge, false); // true
        builder.SameParameter(edge, false);
        // Create and add new vertices
        {
            Vertex v1New = new(), v2New = new();

            // Retrieve original vertices from edges
            Exploration.Vertices(edge1, out Vertex v11, out Vertex v12);
            Exploration.Vertices(edge2, out Vertex v21, out Vertex v22);

            // check that edges merged valid way (for edges having length less than specified
            // tolerance
            //  Check if edges are closed
            bool isClosed1 = v11.IsSame(v12);
            bool isClosed2 = v21.IsSame(v22);
            if (!isClosed1 && !isClosed2)
            {
                if (secForward)
                {
                    if (v11.IsSame(v22) || v12.IsSame(v21))
                    {
                        return new Edge();
                    }
                }
                else
                {
                    if (v11.IsSame(v21) || v12.IsSame(v22))
                    {
                        return new Edge();
                    }
                }
            }

            // szv: do not reshape here!!!

            if (isClosed1 || isClosed2)
            {
                // at least one of the edges is closed
                if (isClosed1 && isClosed2)
                {
                    // both edges are closed
                    ComputeToleranceVertex(v11, v21, ref v1New);
                }
                else if (isClosed1)
                {
                    // only first edge is closed
                    ComputeToleranceVertex(v22, v21, v11, ref v1New);
                }
                else
                {
                    // only second edge is closed
                    ComputeToleranceVertex(v11, v12, v21, ref v1New);
                }
                v2New = v1New;
            }
            else
            {
                // both edges are open
                bool isOldFirst = (secForward ? v11.IsSame(v21) : v11.IsSame(v22));
                bool isOldLast = (secForward ? v12.IsSame(v22) : v12.IsSame(v21));
                if (secForward)
                {
                    // case if vertices already sewed
                    if (!isOldFirst)
                    {
                        ComputeToleranceVertex(v11, v21, ref v1New);
                    }
                    if (!isOldLast)
                    {
                        ComputeToleranceVertex(v12, v22, ref v2New);
                    }
                }
                else
                {
                    if (!isOldFirst)
                    {
                        ComputeToleranceVertex(v11, v22, ref v1New);
                    }
                    if (!isOldLast)
                    {
                        ComputeToleranceVertex(v12, v21, ref v2New);
                    }
                }
                if (isOldFirst)
                {
                    v1New = v11;
                }

                if (isOldLast)
                {
                    v2New = v12;
                }
            }
            // Add the vertices in the good sense
            Shape anEdge = edge.Oriented(Forward);
            Shape localEdge = v1New.Oriented(Forward); //(listNode.First()).Oriented(TopAbs_FORWARD);
            builder.Add(anEdge, localEdge);
            localEdge = v2New.Oriented(Reversed); //(listNode.Last()).Oriented(TopAbs_REVERSED);
            builder.Add(anEdge, localEdge);

            int k = 1;
            for (; k <= seqNMVert.Length(); k++)
            {
                builder.Add(anEdge, seqNMVert.Value(k));
            }
        }

        // Retrieve second PCurves
        var loc2 = new Placement();
        Surface? surf2 = null;

        var itf2 = new NCollection_List<Shape>.Iterator();
        if (whichSec == 1)
        {
            itf2.Initialize(listFacesLast);
        }
        else
        {
            itf2.Initialize(listFacesFirst);
        }
        bool isResEdge = false;
        var fac2 = new Face();
        for (; itf2.More(); itf2.Next())
        {
            Curve2D? c2d2 = null, c2d21 = null;
            double firstOld, lastOld;
            fac2 = ShapeCasts.Face(itf2.Value());

            surf2 = Tool.Surface(fac2, out loc2);
            bool isSeam2 = ((IsUClosedSurface(surf2!, edge2, loc2) || IsVClosedSurface(surf2!, edge2, loc2))
                            && Tool.IsClosed(ShapeCasts.Edge(edge2), fac2));
            if (isSeam2)
            {
                if (!_nonmanifold)
                {
                    return new Edge();
                }
                Shape tmpShape = edge2.Reversed(); // for porting
                c2d21 = Tool.CurveOnSurface(ShapeCasts.Edge(tmpShape), fac2, out firstOld, out lastOld);
            }
            c2d2 = Tool.CurveOnSurface(edge2, fac2, out firstOld, out lastOld);
            if (c2d2 is null && c2d21 is null)
            {
                continue;
            }

            if (c2d21 is not null)
            {
                c2d21 = (Curve2D)c2d21.Copy();
                if (!secForward)
                {
                    if (c2d21 is LineCurve2D)
                    {
                        c2d21 = new TrimmedCurve2D(c2d21, firstOld, lastOld);
                    }
                    double first2d = firstOld; // c2dTmp->FirstParameter(); BUG USA60321
                    double last2d = lastOld;   // c2dTmp->LastParameter();
                    firstOld = c2d21.ReversedParameter(last2d);
                    lastOld = c2d21.ReversedParameter(first2d);
                    c2d21.Reverse();
                }
                c2d21 = SameRange(c2d21, firstOld, lastOld, first, last);
            }

            // Make second PCurve sameRange with the 3d curve
            c2d2 = (Curve2D)c2d2!.Copy(); // OCCT: null when only the seam's reversed pcurve exists

            if (!secForward)
            {
                if (c2d2 is LineCurve2D)
                {
                    c2d2 = new TrimmedCurve2D(c2d2, firstOld, lastOld);
                }
                double first2d = firstOld;
                double last2d = lastOld;
                firstOld = c2d2.ReversedParameter(last2d);
                lastOld = c2d2.ReversedParameter(first2d);
                c2d2.Reverse();
            }

            c2d2 = SameRange(c2d2, firstOld, lastOld, first, last);
            if (c2d2 is null)
            {
                continue;
            }

            // Add second PCurve
            bool isSeam = false;
            Orientation ori = Forward;

            var itf1 = new NCollection_List<Shape>.Iterator();
            if (whichSec == 1)
            {
                itf1.Initialize(listFacesFirst);
            }
            else
            {
                itf1.Initialize(listFacesLast);
            }
            for (; itf1.More() && !isSeam; itf1.Next())
            {
                Curve2D? c2d1 = null, c2d11 = null;
                Face fac1 = ShapeCasts.Face(itf1.Value());

                Surface? surf1 = Tool.Surface(fac1, out Placement loc1);

                double first2d, last2d;
                bool isSeam1 = ((IsUClosedSurface(surf1!, edge1, loc1) || IsVClosedSurface(surf1!, edge1, loc1))
                                && Tool.IsClosed(ShapeCasts.Edge(edge1), fac1));
                c2d1 = Tool.CurveOnSurface(edge1, fac1, out first2d, out last2d);
                ori = edge1.Orientation;
                if (fac1.Orientation == Reversed)
                {
                    ori = global::Kerf.Engine.Topology.TopAbs.Reverse(ori);
                }

                if (isSeam1)
                {
                    if (!_nonmanifold)
                    {
                        return new Edge();
                    }
                    Shape tmpShape = edge1.Reversed(); // for porting
                    c2d11 = Tool.CurveOnSurface(ShapeCasts.Edge(tmpShape), fac1, out first2d, out last2d);
                    // if(fac1.Orientation() == TopAbs_REVERSED) //
                    if (ori == Forward)
                    {
                        builder.UpdateEdge(edge, c2d1, c2d11, fac1, 0);
                    }
                    else
                    {
                        builder.UpdateEdge(edge, c2d11, c2d1, fac1, 0);
                    }
                }
                else
                {
                    builder.UpdateEdge(edge, c2d1, fac1, 0);
                }

                if (c2d1 is null && c2d11 is null)
                {
                    continue;
                }

                if (ReferenceEquals(surf2, surf1))
                {
                    // Merge sections which are on the same face
                    if (!loc2.IsDifferent(loc1))
                    {
                        bool uclosed = IsUClosedSurface(surf2!, edge2, loc2);
                        bool vclosed = IsVClosedSurface(surf2!, edge2, loc2);
                        if (uclosed || vclosed)
                        {
                            double pf = c2d1!.FirstParameter;
                            Point2D p1n = c2d1.Value(Max(first, pf));
                            Point2D p21n = c2d2.Value(Max(first, c2d2.FirstParameter));
                            Point2D p22n = c2d2.Value(Min(last, c2d2.LastParameter));
                            double aDist = Min(p1n.DistanceTo(p21n), p1n.DistanceTo(p22n));
                            surf2!.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                            isSeam = ((uclosed && aDist > 0.75 * (Math.Abs(u2 - u1)))
                                      || (vclosed && aDist > 0.75 * (Math.Abs(v2 - v1))));
                            if (!isSeam && Tool.IsClosed(ShapeCasts.Edge(edge), fac1))
                            {
                                continue;
                            }
                        }
                    }
                }

                isResEdge = true;
                if (isSeam)
                {
                    if (ori == Forward)
                    {
                        builder.UpdateEdge(edge, c2d1, c2d2, surf2, loc2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    else
                    {
                        builder.UpdateEdge(edge, c2d2, c2d1, surf2, loc2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }
                else if (isSeam2)
                {
                    Orientation initOri = edge2.Orientation;
                    Orientation secOri = edge.Orientation;
                    if (fac2.Orientation == Reversed)
                    {
                        initOri = global::Kerf.Engine.Topology.TopAbs.Reverse(initOri);
                        secOri = global::Kerf.Engine.Topology.TopAbs.Reverse(secOri);
                    }
                    if (!secForward)
                    {
                        initOri = global::Kerf.Engine.Topology.TopAbs.Reverse(initOri);
                    }

                    if (initOri == Forward)
                    {
                        builder.UpdateEdge(edge, c2d2, c2d21, surf2, loc2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    else
                    {
                        builder.UpdateEdge(edge, c2d21, c2d2, surf2, loc2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }
                else
                {
                    builder.UpdateEdge(edge, c2d2, surf2, loc2, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
            }
        }
        double tolReached = global::Kerf.Engine.Foundation.Precision.Infinite();
        bool isSamePar = false;
        try
        {
            if (isResEdge)
            {
                SameParameter(edge);
            }

            if (Tool.SameParameter(edge))
            {
                isSamePar = true;
                tolReached = Tool.Tolerance(edge);
            }
        }
        catch (FailureException)
        {
            isSamePar = false;
        }

        if (firstCall && (!isResEdge || !isSamePar || tolReached > _tolerance))
        {
            int whichSecn = whichSec;
            // Try to merge on the second section
            bool second_ok = false;
            Edge s_edge = SameParameterEdge(edgeFirst,
                                                   edgeLast,
                                                   listFacesFirst,
                                                   listFacesLast,
                                                   secForward,
                                                   ref whichSecn,
                                                   false);
            if (!s_edge.IsNull)
            {
                double tolReached_2 = Tool.Tolerance(s_edge);
                second_ok = (Tool.SameParameter(s_edge) && tolReached_2 < tolReached);
                if (second_ok)
                {
                    edge = s_edge;
                    whichSec = whichSecn;
                    tolReached = tolReached_2;
                }
            }

            if (!second_ok && !edge.IsNull)
            {
                var c3dAdapt = new CurveAdaptor(c3d!);

                // Discretize edge curve
                int i, j, nbp = 23;
                double deltaT = (last3d - first3d) / (nbp - 1);
                var c3dpnt = new NCollection_Array1<Point3D>(1, nbp);
                for (i = 1; i <= nbp; i++)
                {
                    c3dpnt[i] = c3dAdapt.Value(first3d + (i - 1) * deltaT);
                }

                double dist = 0.0, maxTol = -1.0;
                bool more = true;

                for (j = 1; more; j++)
                {
                    Curve2D? c2d2 = null;
                    Tool.CurveOnSurface(edge, ref c2d2, ref surf2, ref loc2, ref first, ref last, j);

                    more = c2d2 is not null;
                    if (more)
                    {
                        Surface s = surf2!;
                        if (!loc2.IsIdentity)
                        {
                            s = (Surface)surf2!.Transformed(loc2.Transformation);
                        }

                        double dist2 = 0.0;
                        deltaT = (last - first) / (nbp - 1);
                        for (i = 1; i <= nbp; i++)
                        {
                            Point2D p2d = c2d2!.Value(first + (i - 1) * deltaT);
                            s.D0(p2d.X, p2d.Y, out Point3D p2);
                            Point3D p1 = c3dpnt[i];
                            dist = p2.SquareDistanceTo(p1);
                            if (dist > dist2)
                            {
                                dist2 = dist;
                            }
                        }
                        maxTol = Max(Math.Sqrt(dist2) * (1.0 + 1e-7), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }
                if (maxTol >= 0.0 && maxTol < tolReached)
                {
                    if (tolReached > MaxTolerance())
                    {
                        // Set tolerance directly to overwrite too large tolerance
                        ((BrepTEdge)edge.TShape()!).Tolerance(maxTol);
                    }
                    else
                    {
                        // just update tolerance with computed distance
                        builder.UpdateEdge(edge, maxTol);
                    }
                }
                builder.SameParameter(edge, true);
            }
        }

        double tolEdge1 = Tool.Tolerance(edge);
        if (tolEdge1 > MaxTolerance())
        {
            edge.Nullify();
        }
        return edge;
    }

    //=================================================================================================

    /// <summary>Evaluates the mean angles between the normals along the sections. Called from MergingOfSections only.</summary>
    protected virtual void EvaluateAngulars(NCollection_Sequence<Shape> sequenceSec,
                                            NCollection_Array1<bool> secForward,
                                            NCollection_Array1<double> tabAng,
                                            int indRef)
    {
        tabAng.Init(-1.0);

        int i, j, npt = 4, lengSec = sequenceSec.Length();

        Edge edge;
        Face face;
        Placement loc;
        double first = 0.0, last = 0.0;
        Curve3D? c3d;
        Curve2D? c2d = null;
        Surface? surf = null;
        var normRef = new NCollection_Array1<Vector3D>(1, npt);

        for (i = indRef; i <= lengSec; i++)
        {
            edge = ShapeCasts.Edge(sequenceSec[i]);

            Shape bnd = edge;
            if (_sectionBound.ContainsKey(bnd))
            {
                bnd = _sectionBound.GetValue(bnd);
            }
            if (_boundFaces.ContainsKey(bnd))
            {
                face = ShapeCasts.Face(_boundFaces.GetValue(bnd).First());
                surf = Tool.Surface(face, out loc);
                if (!loc.IsIdentity)
                {
                    surf = (Surface)surf!.Copy();
                    surf.Transform(loc.Transformation);
                }
                c2d = Tool.CurveOnSurface(edge, face, out first, out last);
            }
            else if (i == indRef)
            {
                return;
            }
            else
            {
                continue;
            }

            c3d = Tool.Curve(edge, out loc, out first, out last);
            if (!loc.IsIdentity)
            {
                c3d = (Curve3D)c3d!.Copy();
                c3d.Transform(loc.Transformation);
            }

            var adapt = new CurveAdaptor(c3d!);
            var uniAbs = new UniformAbscissa(adapt, npt, first, last);

            double cumulateAngular = 0.0;
            int nbComputedAngle = 0;

            for (j = 1; j <= npt; j++)
            {
                c2d!.D0(uniAbs.Parameter((secForward[i] || i == indRef) ? j : npt - j + 1), out Point2D p);
                surf!.D1(p.X, p.Y, out _, out Vector3D w1, out Vector3D w2);
                Vector3D n = w1 ^ w2; // Compute the normal vector
                if (i == indRef)
                {
                    normRef[j] = n;
                }
                else if ((n.Magnitude > Constants.Resolution()) && (normRef[j].Magnitude > Constants.Resolution()))
                {
                    nbComputedAngle++;
                    double angular = n.AngleTo(normRef[j]);
                    if (angular > M_PI / 2.0)
                    {
                        angular = M_PI - angular;
                    }
                    cumulateAngular += angular;
                }
            }

            if (nbComputedAngle != 0)
            {
                tabAng[i] = cumulateAngular / ((double)nbComputedAngle);
            }
        }
    }

    //=======================================================================
    // function : EvaluateDistances
    // purpose  : internal use
    // Evaluate distance between edges with indice indRef and the following edges in the list
    // Remarks (lengSec - indRef) must be >= 1
    //=======================================================================

    /// <summary>
    /// Evaluates the distances between the section <paramref name="indRef"/> and the following ones, with their mutual
    /// orientations, lengths and least distances. Called from MergingOfSections only.
    /// </summary>
    protected virtual void EvaluateDistances(NCollection_Sequence<Shape> sequenceSec,
                                             NCollection_Array1<bool> secForward,
                                             NCollection_Array1<double> tabDst,
                                             NCollection_Array1<double> arrLen,
                                             NCollection_Array1<double> tabMinDist,
                                             int indRef)
    {
        secForward.Init(true);
        tabDst.Init(-1.0);
        arrLen.Init(0.0);
        tabMinDist.Init(global::Kerf.Engine.Foundation.Precision.Infinite());
        const int npt = 8; // Number of points for curve discretization
        var ptsRef = new NCollection_Array1<Point3D>(1, npt);
        var ptsSec = new NCollection_Array1<Point3D>(1, npt);

        int i, j, lengSec = sequenceSec.Length();

        Curve3D? c3dRef = null;
        double firstRef = 0.0, lastRef = 0.0;

        for (i = indRef; i <= lengSec; i++)
        {
            // reading of the edge (attention for the first one: reference)
            Edge sec = ShapeCasts.Edge(sequenceSec[i]);

            Curve3D? c3d = Tool.Curve(sec, out Placement loc, out double first, out double last);
            if (c3d is null)
            {
                continue;
            }
            if (!loc.IsIdentity)
            {
                c3d = (Curve3D)c3d.Copy();
                c3d.Transform(loc.Transformation);
            }

            if (i == indRef)
            {
                c3dRef = c3d;
                firstRef = first;
                lastRef = last;
            }

            double dist = global::Kerf.Engine.Foundation.Precision.Infinite(), distFor = -1.0, distRev = -1.0;
            double minDist = global::Kerf.Engine.Foundation.Precision.Infinite();

            double t, deltaT = (last - first) / (npt - 1);
            double lenSec2 = 0.0;

            int nbFound = 0;
            for (j = 1; j <= npt; j++)
            {
                // Uniform parameter on curve
                if (j == 1)
                {
                    t = first;
                }
                else if (j == npt)
                {
                    t = last;
                }
                else
                {
                    t = first + (j - 1) * deltaT;
                }

                // Take point on curve
                Point3D pt = c3d.Value(t);

                if (i == indRef)
                {
                    ptsRef[j] = pt;
                    if (j > 1)
                    {
                        lenSec2 += pt.SquareDistanceTo(ptsRef[j - 1]);
                    }
                }
                else
                {
                    ptsSec[j] = pt;
                    // protection to avoid merging with small sections
                    if (j > 1)
                    {
                        lenSec2 += pt.SquareDistanceTo(ptsSec[j - 1]);
                    }
                    // To evaluate mutual orientation and distance
                    dist = pt.DistanceTo(ptsRef[j]);
                    if (minDist > dist)
                    {
                        minDist = dist;
                    }
                    if (distFor < dist)
                    {
                        distFor = dist;
                    }
                    dist = pt.DistanceTo(ptsRef[npt - j + 1]);

                    if (minDist > dist)
                    {
                        minDist = dist;
                    }
                    if (distRev < dist)
                    {
                        distRev = dist;
                    }

                    // Check that point lays between vertices of reference curve
                    Point3D p11 = ptsRef[1];
                    Point3D p12 = ptsRef[npt];
                    var vec1 = new Vector3D(pt, p11);
                    var vec2 = new Vector3D(pt, p12);
                    var vecRef = new Vector3D(p11, p12);
                    if ((vecRef * vec1) * (vecRef * vec2) < 0.0)
                    {
                        nbFound++;
                    }
                }
            }

            double lenSec = Math.Sqrt(lenSec2);

            // if(aLenSec < myMinTolerance )
            //  continue;
            arrLen.SetValue(i, lenSec);
            // Record mutual orientation
            bool isForward = (distFor < distRev); // szv debug: <=
            secForward[i] = isForward;

            dist = (isForward ? distFor : distRev);
            if (i == indRef || (dist < _tolerance && nbFound >= npt * 0.5))
            {
                tabDst[i] = dist;
                tabMinDist[i] = minDist;
            }
            else
            {
                nbFound = 0;
                minDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                dist = -1;
                var arrProj = new NCollection_Array1<Point3D>(1, npt);
                var arrDist = new NCollection_Array1<double>(1, npt);
                var arrPara = new NCollection_Array1<double>(1, npt);
                if (arrLen[indRef] >= arrLen[i])
                {
                    ProjectPointsOnCurve(ptsSec, c3dRef, firstRef, lastRef, arrDist, arrPara, arrProj, false);
                }
                else
                {
                    ProjectPointsOnCurve(ptsRef, c3d, first, last, arrDist, arrPara, arrProj, false);
                }
                for (j = 1; j <= npt; j++)
                {
                    if (arrDist[j] < 0.0)
                    {
                        continue;
                    }
                    if (dist < arrDist[j])
                    {
                        dist = arrDist[j];
                    }
                    if (minDist > arrDist[j])
                    {
                        minDist = arrDist[j];
                    }
                    nbFound++;
                }
                if (nbFound > 1)
                {
                    tabDst[i] = dist;
                    tabMinDist[i] = minDist;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the two edges, on a closed surface of the face, would make a merged closed edge.</summary>
    protected bool IsMergedClosed(in Edge edge1, in Edge edge2, in Face face)
    {
        // Check for closed surface
        Surface? surf = Tool.Surface(face, out Placement loc);
        bool isUClosed = IsUClosedSurface(surf!, edge1, loc);
        bool isVClosed = IsVClosedSurface(surf!, edge1, loc);
        if (!isUClosed && !isVClosed)
        {
            return false;
        }
        // Check condition on closed surface
        Curve2D? C2d1 = Tool.CurveOnSurface(edge1, face, out double first2d1, out double last2d1);
        Curve2D? C2d2 = Tool.CurveOnSurface(edge2, face, out double first2d2, out double last2d2);
        if (C2d1 is null || C2d2 is null)
        {
            return false;
        }
        bool isULongC1, isULongC2, isVLongC1, isVLongC2;
        double sUmin, sUmax, sVmin, sVmax;
        double c1Umin, c1Vmin, c1Umax, c1Vmax;
        double c2Umin, c2Vmin, c2Umax, c2Vmax;
        { // szv: Use brackets to destroy local variables
            Box2d b1 = new(), b2 = new();
            var c2d1 = new CurveAdaptor2D(C2d1);
            var c2d2 = new CurveAdaptor2D(C2d2);
            Add2dCurve.Add(c2d1, first2d1, last2d1, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref b1);
            Add2dCurve.Add(c2d2, first2d2, last2d2, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref b2);
            b1.Get(out c1Umin, out c1Vmin, out c1Umax, out c1Vmax);
            b2.Get(out c2Umin, out c2Vmin, out c2Umax, out c2Vmax);
            double du, dv;
            du = (c1Umax - c1Umin);
            dv = (c1Vmax - c1Vmin);
            isULongC1 = (dv <= du);
            isVLongC1 = (du <= dv);
            du = (c2Umax - c2Umin);
            dv = (c2Vmax - c2Vmin);
            isULongC2 = (dv <= du);
            isVLongC2 = (du <= dv);
            surf!.ParameterBounds(out sUmin, out sUmax, out sVmin, out sVmax);
        }
        if (isUClosed && isVLongC1 && isVLongC2)
        {
            // Do not merge if not overlapped by V
            double dist = Max((c2Vmin - c1Vmax), (c1Vmin - c2Vmax));
            if (dist < 0.0)
            {
                double distInner = Max((c2Umin - c1Umax), (c1Umin - c2Umax));
                double distOuter = (sUmax - sUmin) - Max((c2Umax - c1Umin), (c1Umax - c2Umin));
                if (distOuter <= distInner)
                {
                    return true;
                }
            }
        }
        if (isVClosed && isULongC1 && isULongC2)
        {
            // Do not merge if not overlapped by U
            double dist = Max((c2Umin - c1Umax), (c1Umin - c2Umax));
            if (dist < 0.0)
            {
                double distInner = Max((c2Vmin - c1Vmax), (c1Vmin - c2Vmax));
                double distOuter = (sVmax - sVmin) - Max((c2Vmax - c1Vmin), (c1Vmax - c2Vmin));
                if (distOuter <= distInner)
                {
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    /// <summary>Removes from the candidates the sections that have nearer sections on the work face.</summary>
    protected void AnalysisNearestEdges(NCollection_Sequence<Shape> sequenceSec,
                                        NCollection_Sequence<int> seqIndCandidate,
                                        NCollection_Sequence<bool> seqOrientations,
                                        bool evalDist = true)
    {
        int workIndex = seqIndCandidate.First();
        Shape workedge = sequenceSec.Value(workIndex);
        Shape bnd = workedge;
        var workfaces = new NCollection_List<Shape>();
        if (_sectionBound.ContainsKey(bnd))
        {
            bnd = _sectionBound.GetValue(bnd);
        }
        if (_boundFaces.ContainsKey(bnd))
        {
            workfaces.Assign(_boundFaces.GetValue(bnd));
        }
        if (workfaces.IsEmpty())
        {
            return;
        }
        var mapFaces = new HashSet<Shape>(ShapeHasher.Instance);
        var lIt = new NCollection_List<Shape>.Iterator();
        for (lIt.Initialize(workfaces); lIt.More(); lIt.Next())
        {
            mapFaces.Add(lIt.Value());
        }
        var seqNotCandidate = new NCollection_Sequence<int>();
        // Separates edges belonging the same face as work edge
        // for exception of edges belonging closed faces

        seqNotCandidate.Append(workIndex);
        for (int i = 1; i <= seqIndCandidate.Length();)
        {
            int index = seqIndCandidate.Value(i);
            bool isRemove = false;
            if (index == workIndex)
            {
                seqIndCandidate.Remove(i);
                seqOrientations.Remove(i);
                isRemove = true;
            }
            if (!isRemove)
            {
                Shape bnd2 = sequenceSec.Value(index);
                if (_sectionBound.ContainsKey(bnd2))
                {
                    bnd2 = _sectionBound.GetValue(bnd2);
                }

                if (_boundFaces.ContainsKey(bnd2))
                {
                    NCollection_List<Shape> listfaces = _boundFaces.GetValue(bnd2);
                    bool isMerged = true;
                    for (lIt.Initialize(listfaces); lIt.More() && isMerged; lIt.Next())
                    {
                        if (mapFaces.Contains(lIt.Value()))
                        {
                            Surface? surf = Tool.Surface(ShapeCasts.Face(lIt.Value()), out Placement loc);
                            isMerged = ((IsUClosedSurface(surf!, bnd2, loc) || IsVClosedSurface(surf!, bnd2, loc))
                                        && IsMergedClosed(ShapeCasts.Edge(sequenceSec.Value(index)),
                                                          ShapeCasts.Edge(workedge),
                                                          ShapeCasts.Face(lIt.Value())));
                        }
                    }
                    if (!isMerged)
                    {
                        seqNotCandidate.Append(index);
                        seqIndCandidate.Remove(i);
                        seqOrientations.Remove(i);
                        isRemove = true;
                    }
                }
                else
                {
                    seqIndCandidate.Remove(i);
                    seqOrientations.Remove(i);
                    isRemove = true;
                }
            }
            if (!isRemove)
            {
                i++;
            }
        }
        if (seqIndCandidate.Length() == 0 || seqNotCandidate.Length() == 1)
        {
            return;
        }
        if (!evalDist)
        {
            return;
        }
        var totTabDist = new NCollection_Array2<double>(1, seqNotCandidate.Length(), 1, seqIndCandidate.Length());
        var mapIndex = new HashSet<int>();
        var seqForward = new NCollection_Sequence<int>();

        // Definition and removing edges which are not candidate for work edge
        // (they have other nearest edges belonging to the work face)
        for (int k = 1; k <= seqNotCandidate.Length(); k++)
        {
            int index1 = seqNotCandidate.Value(k);
            Shape edge = sequenceSec.Value(index1);
            var tmpSeq = new NCollection_Sequence<Shape>();
            tmpSeq.Append(edge);
            for (int kk = 1; kk <= seqIndCandidate.Length(); kk++)
            {
                tmpSeq.Append(sequenceSec.Value(seqIndCandidate.Value(kk)));
            }

            int lengSec = tmpSeq.Length();
            var tabForward = new NCollection_Array1<bool>(1, lengSec);
            var tabDist = new NCollection_Array1<double>(1, lengSec);
            var arrLen = new NCollection_Array1<double>(1, lengSec);
            var tabMinDist = new NCollection_Array1<double>(1, lengSec);
            for (int i1 = 1; i1 <= lengSec; i1++)
            {
                tabDist[i1] = -1;
            }

            EvaluateDistances(tmpSeq, tabForward, tabDist, arrLen, tabMinDist, 1);
            if (k == 1)
            {
                for (int n = 1; n < lengSec; n++)
                {
                    if (tabDist[n + 1] == -1 || tabDist[n + 1] > _tolerance)
                    {
                        mapIndex.Add(n);
                        continue;
                    }
                    totTabDist[k, n] = tabDist[n + 1];
                    seqForward.Append(tabForward[n + 1] ? 1 : 0);
                }
            }
            else
            {
                for (int n = 1; n < lengSec; n++)
                {
                    if (tabDist[n] == -1 || tabDist[n] > _tolerance)
                    {
                        continue;
                    }
                    if (tabDist[n + 1] < totTabDist[1, n])
                    {
                        mapIndex.Add(n);
                    }
                }
            }
        }

        int i2 = seqIndCandidate.Length();
        for (; i2 >= 1; i2--)
        {
            if (mapIndex.Contains(i2))
            {
                seqIndCandidate.Remove(i2);
                seqOrientations.Remove(i2);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Finds the sections that can be merged with the last reference of <paramref name="mapReference"/>, sorted by
    /// distance, with their orientations.
    /// </summary>
    protected bool FindCandidates(NCollection_Sequence<Shape> seqSections,
                                  IndexedSet<int> mapReference,
                                  NCollection_Sequence<int> seqCandidates,
                                  NCollection_Sequence<bool> seqOrientations)
    {
        int i, nbSections = seqSections.Length();
        if (nbSections <= 1)
        {
            return false;
        }
        // Retrieve last reference index
        int indReference = mapReference[mapReference.Count - 1];
        int nbCandidates = 0;
        var faces1 = new HashSet<Shape>(ShapeHasher.Instance);
        // if (nbSections > 1) {

        Edge edge1 = ShapeCasts.Edge(seqSections[indReference]);

        // Retrieve faces for reference section

        { // szv: Use brackets to destroy local variables
            Shape bnd = edge1;
            if (_sectionBound.ContainsKey(bnd))
            {
                bnd = _sectionBound.GetValue(bnd);
            }
            if (_boundFaces.ContainsKey(bnd))
            {
                var itf1 = new NCollection_List<Shape>.Iterator(_boundFaces.GetValue(bnd));
                for (; itf1.More(); itf1.Next())
                {
                    faces1.Add(itf1.Value());
                }
            }
        }

        // Check merging conditions for candidates and remove unsatisfactory
        var seqSectionsNew = new NCollection_Sequence<Shape>();
        var seqCandidatesNew = new NCollection_Sequence<int>();
        for (i = 1; i <= nbSections; i++)
        {
            if (i == indReference)
            {
                seqSectionsNew.Prepend(edge1);
                seqCandidatesNew.Prepend(i);
            }
            else
            {
                Edge edge2 = ShapeCasts.Edge(seqSections[i]);
                seqSectionsNew.Append(edge2);
                seqCandidatesNew.Append(i);
            }
        }

        int nbSectionsNew = seqSectionsNew.Length();
        if (nbSectionsNew > 1)
        {
            // Evaluate distances between reference and other sections
            var arrForward = new NCollection_Array1<bool>(1, nbSectionsNew);
            var arrDistance = new NCollection_Array1<double>(1, nbSectionsNew);
            var arrLen = new NCollection_Array1<double>(1, nbSectionsNew);
            var arrMinDist = new NCollection_Array1<double>(1, nbSectionsNew);
            EvaluateDistances(seqSectionsNew, arrForward, arrDistance, arrLen, arrMinDist, 1);

            // Fill sequence of candidate indices sorted by distance
            for (i = 2; i <= nbSectionsNew; i++)
            {
                double maxDist = arrDistance[i];
                if (maxDist >= 0.0 && maxDist <= _tolerance && arrLen[i] > _minTolerance)
                {
                    // Reference section is connected to section #i
                    bool isInserted = false;
                    bool ori = arrForward[i];
                    for (int j = 1; (j <= seqCandidates.Length()) && !isInserted; j++)
                    {
                        double delta = arrDistance[i] - arrDistance[seqCandidates.Value(j)];

                        if (delta < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            if (Math.Abs(delta) > RealSmall() || arrMinDist[i] < arrMinDist[seqCandidates.Value(j)])
                            {
                                seqCandidates.InsertBefore(j, i);
                                seqOrientations.InsertBefore(j, ori);
                                isInserted = true;
                            }
                        }
                    }
                    if (!isInserted)
                    {
                        seqCandidates.Append(i);
                        seqOrientations.Append(ori);
                    }
                }
            }

            nbCandidates = seqCandidates.Length();
            if (nbCandidates == 0)
            {
                return false; // Section has no candidates to merge
            }

            // Replace candidate indices

            for (i = 1; i <= nbCandidates; i++)
            {
                seqCandidates[i] = seqCandidatesNew[seqCandidates[i]];
            }
        }

        if (nbCandidates == 0)
        {
            return false; // Section has no candidates to merge
        }

        if (_nonmanifold && nbCandidates > 1)
        {
            var seqNewCandidates = new NCollection_Sequence<int>();
            var seqOrientationsNew = new NCollection_Sequence<bool>();
            seqCandidates.Prepend(1);
            seqOrientations.Prepend(true);
            for (int k = 1; k <= seqSections.Length() && seqCandidates.Length() > 1; k++)
            {
                AnalysisNearestEdges(seqSections, seqCandidates, seqOrientations, (k == 1));
                if (k == 1 && seqCandidates.Length() == 0)
                {
                    return false;
                }
                if (seqCandidates.Length() != 0)
                {
                    seqNewCandidates.Append(seqCandidates.First());
                    seqOrientationsNew.Append(seqOrientations.First());
                }
            }
            seqCandidates.Prepend(seqNewCandidates);
            seqOrientations.Prepend(seqOrientationsNew);
            return true;
        }
        else
        {
            // For manifold case leave only one candidate from equidistant candidates
            // (OCCT's commented-out equidistant-candidate analysis is left out.)

            // Find the best approved candidate
            while (nbCandidates != 0)
            {
                // Retrieve first candidate
                int indCandidate = seqCandidates.First();
                // Candidate is approved if it is in the map
                if (mapReference.Contains(indCandidate))
                {
                    break;
                }
                // Find candidates for candidate #indCandidate
                mapReference.Add(indCandidate); // Push candidate in the map
                var seqCandidates1 = new NCollection_Sequence<int>();
                var seqOrientations1 = new NCollection_Sequence<bool>();
                bool isFound = FindCandidates(seqSections, mapReference, seqCandidates1, seqOrientations1);
                mapReference.RemoveLast(); // Pop candidate from the map
                if (isFound)
                {
                    isFound = (seqCandidates1.Length() > 0);
                }
                if (isFound)
                {
                    int indCandidate1 = seqCandidates1.First();
                    // If indReference is the best candidate for indCandidate
                    // then indCandidate is the best candidate for indReference
                    if (indCandidate1 == indReference)
                    {
                        break;
                    }
                    // If some other reference in the map is the best candidate for indCandidate
                    // then assume that reference is the best candidate for indReference
                    if (mapReference.Contains(indCandidate1))
                    {
                        seqCandidates.Prepend(indCandidate1);
                        nbCandidates++;
                        break;
                    }
                    isFound = false;
                }
                if (!isFound)
                {
                    // Remove candidate #1
                    seqCandidates.Remove(1);
                    seqOrientations.Remove(1);
                    nbCandidates--;
                }
            }
        }
        // gka
        if (nbCandidates > 0)
        {
            int ind = seqCandidates.Value(1);
            Edge edge2 = ShapeCasts.Edge(seqSections[ind]);
            Shape bnd = edge2;
            if (_sectionBound.ContainsKey(bnd))
            {
                bnd = _sectionBound.GetValue(bnd);
            }
            // gka
            if (_boundFaces.ContainsKey(bnd))
            {
                bool isOK = true;
                var itf2 = new NCollection_List<Shape>.Iterator(_boundFaces.GetValue(bnd));
                for (; itf2.More() && isOK; itf2.Next())
                {
                    Face face2 = ShapeCasts.Face(itf2.Value());
                    // Check whether condition is satisfied
                    isOK = !faces1.Contains(face2);
                    if (!isOK)
                    {
                        isOK = IsMergedClosed(edge1, edge2, face2);
                    }
                }
                if (!isOK)
                {
                    return false;
                }
            }
        }
        return (nbCandidates > 0);
    }

    //=======================================================================
    // function : Init
    // purpose  : Initialise Talerance, and options sewing, faceAnalysis and cutting
    //=======================================================================

    /// <summary>Initializes the tolerance and the options (as the constructor), and clears the inputs and results.</summary>
    public void Init(double tolerance = 1.0e-06,
                     bool optionSewing = true,
                     bool optionAnalysis = true,
                     bool optionCutting = true,
                     bool optionNonmanifold = false)
    {
        // Set tolerance and Perform options
        _tolerance = Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        _sewing = optionSewing;
        _analysis = optionAnalysis;
        _cutting = optionCutting;
        _nonmanifold = optionNonmanifold;
        // Set min and max tolerances
        _minTolerance = _tolerance * 1e-4; // szv: proposal
        if (_minTolerance < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _minTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        _maxTolerance = global::Kerf.Engine.Foundation.Precision.Infinite();
        // Set other modes
        _faceMode = true;
        _floatingEdgesMode = false;
        // myCuttingFloatingEdgesMode = false; //gka
        _sameParameterMode = true;
        _localToleranceMode = false;
        _sewedShape.Nullify();
        // Load empty shape
        Load(new Shape());
    }

    //=================================================================================================

    /// <summary>Loads the context shape (and clears the inputs and results).</summary>
    public void Load(in Shape shape)
    {
        _reShape.Clear();
        if (shape.IsNull)
        {
            _shape.Nullify();
        }
        else
        {
            _shape = _reShape.InternalApply(shape);
        }
        _sewedShape.Nullify();
        // Nullify flags and counters
        _nbShapes = _nbEdges = _nbVertices = 0;
        // Clear all maps
        _oldShapes.Clear();
        // myOldFaces.Clear();
        _degenerated.Clear();
        _freeEdges.Clear();
        _multipleEdges.Clear();
        _contigousEdges.Clear();
        _contigSecBound.Clear();
        _boundFaces.Clear();
        _boundSections.Clear();
        _vertexNode.Clear();
        _vertexNodeFree.Clear();
        _nodeSections.Clear();
        _cuttingNode.Clear();
        _sectionBound.Clear();
        _littleFace.Clear();
        // Kerf: forget the last working overlay.
        _overlay = null;
    }

    //=================================================================================================

    /// <summary>Adds a shape to be sewn or controlled.</summary>
    public void Add(in Shape aShape)
    {
        if (aShape.IsNull)
        {
            return;
        }
        Shape oShape = _reShape.InternalApply(aShape);
        _oldShapes.Add(aShape, oShape);
        _nbShapes = _oldShapes.Count;
    }

    //=================================================================================================

    /// <summary>Computes the sewing (or only the analysis, if the sewing option is off).</summary>
    /// <remarks>
    /// The computation runs on the inputs in a working overlay (Kerf unit 10H4), which takes the writes into frozen
    /// shapes: the result and the answers hold new TShapes for what changed and the caller's own sub-shapes for the
    /// rest (decision 583). The inputs are never written.
    /// </remarks>
    public void Perform(ProgressRange progress = default)
    {
        // Kerf: the working overlay (unit 10H4; decision 583), over no input: sewing keeps no history over its inputs, and
        // what it hands out goes through Finish
        _overlay = null;
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape());
        PerformSewing(progress);
        if (overlay is not null)
        {
            _sewedShape = overlay.Finish(_sewedShape, Answers());
            _overlay = overlay;
        }
    }

    // OCCT's Perform.
    private void PerformSewing(ProgressRange progress)
    {
        int numberOfStages = (_analysis ? 1 : 0) + (_cutting ? 1 : 0) + (_sewing ? 1 : 0) + 2;
        using var ps = new ProgressScope(progress, "Sewing", numberOfStages);

        // face analysis
        if (_analysis)
        {
            FaceAnalysis(ps.Next());
            if (!ps.More())
            {
                return;
            }
        }

        if (_nbShapes != 0 || !_shape.IsNull)
        {
            FindFreeBoundaries();

            if (_boundFaces.Count != 0)
            {
                VerticesAssembling(ps.Next());
                if (!ps.More())
                {
                    return;
                }
                if (_cutting)
                {
                    Cutting(ps.Next());
                    if (!ps.More())
                    {
                        return;
                    }
                }
                Merging(true, ps.Next());
                if (!ps.More())
                {
                    return;
                }
            }
            else
            {
                ps.Next();
                if (_cutting)
                {
                    ps.Next();
                }
                ps.Next();
                if (!ps.More())
                {
                    return;
                }
            }

            if (_sewing)
            {
                // examine the multiple edges if any and process sameparameter for edges if necessary
                EdgeProcessing(ps.Next());
                if (!ps.More())
                {
                    return;
                }
                CreateSewedShape();
                if (!ps.More())
                {
                    _sewedShape.Nullify();
                    return;
                }

                EdgeRegularity(ps.Next());

                if (_sameParameterMode && _faceMode)
                {
                    SameParameterShape();
                }
                if (!ps.More())
                {
                    _sewedShape.Nullify();
                    return;
                }
            }

            // create edge information for output
            CreateOutputInformations();
            if (!ps.More())
            {
                _sewedShape.Nullify();
                return;
            }
        }
    }

    //=======================================================================
    // function : SewedShape
    // purpose  : give the sewed shape
    //           if a null shape, reasons:
    //             -- no useable input shapes : all input shapes are degenerated
    //             -- has multiple edges
    //=======================================================================

    /// <summary>
    /// The sewn shape, frozen: a face, a shell, a solid or a compound; a null shape if nothing was built (all inputs
    /// degenerated).
    /// </summary>
    public Shape SewedShape() => Published(_sewedShape);

    /// <summary>The sewn shape, as <see cref="SewedShape"/>, without freezing it (ported code).</summary>
    internal Shape InternalSewedShape() => FromWork(_sewedShape);

    //=================================================================================================

    /// <summary>The number of free edges (edges shared by one face).</summary>
    public int FreeEdgeCount => _freeEdges.Count;

    /// <summary>The <paramref name="index"/>-th free edge, frozen.</summary>
    public Edge FreeEdge(int index) => ShapeCasts.Edge(Published(InternalFreeEdge(index)));

    /// <summary>The <paramref name="index"/>-th free edge, without freezing it (ported code).</summary>
    internal Edge InternalFreeEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > FreeEdgeCount, "BRepBuilderAPI_Sewing::FreeEdge");
        return ShapeCasts.Edge(FromWork(_freeEdges[index - 1]));
    }

    //=================================================================================================

    /// <summary>The number of multiple edges (edges shared by more than two faces).</summary>
    public int MultipleEdgeCount => _multipleEdges.Count;

    /// <summary>The <paramref name="index"/>-th multiple edge, frozen.</summary>
    public Edge MultipleEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > MultipleEdgeCount, "BRepBuilderAPI_Sewing::MultipleEdge");
        return ShapeCasts.Edge(Published(_multipleEdges[index - 1]));
    }

    //=================================================================================================

    /// <summary>The number of contiguous edges (edges shared by two faces).</summary>
    public int ContigousEdgeCount => _contigousEdges.Count;

    /// <summary>The <paramref name="index"/>-th contiguous edge, frozen.</summary>
    public Edge ContigousEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > ContigousEdgeCount, "BRepBuilderAPI_Sewing::ContigousEdge");
        return ShapeCasts.Edge(Published(_contigousEdges.KeyAt(index - 1)));
    }

    //=================================================================================================

    /// <summary>The sections (edges) of the <paramref name="index"/>-th contiguous edge, frozen, in a new list.</summary>
    public IReadOnlyList<Shape> ContigousEdgeCouple(int index)
    {
        NCollection_List<Shape> list = InternalContigousEdgeCouple(index);
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (var it = new NCollection_List<Shape>.Iterator(list); it.More(); it.Next())
        {
            shapes[i++] = Published(it.Value());
        }
        return ImmutableArrays.Own(shapes);
    }

    /// <summary>The sections of the <paramref name="index"/>-th contiguous edge: the algorithm's own list (ported code).</summary>
    internal NCollection_List<Shape> InternalContigousEdgeCouple(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > ContigousEdgeCount,
                                     "BRepBuilderAPI_Sewing::ContigousEdgeCouple");
        return _contigousEdges.ValueAt(index - 1);
    }

    //=================================================================================================

    /// <summary>True if the section is bound to a boundary (see <see cref="SectionToBoundary"/>).</summary>
    public bool IsSectionBound(in Edge section) => _contigSecBound.ContainsKey(ToWork(section));

    //=================================================================================================

    /// <summary>
    /// The original edge (free boundary) that became the section, frozen. Sections make the common edges; the original
    /// edge gives the surface the section is attached to.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The section isn't bound.</exception>
    public Edge SectionToBoundary(in Edge section)
    {
        Standard_NoSuchObject_Raise_if(!IsSectionBound(section), "BRepBuilderAPI_Sewing::SectionToBoundary");
        return ShapeCasts.Edge(Published(_contigSecBound.GetValue(ToWork(section))));
    }

    //=================================================================================================

    /// <summary>The number of deleted faces (faces smaller than the tolerance).</summary>
    public int DeletedFaceCount => _littleFace.Count;

    /// <summary>The <paramref name="index"/>-th deleted face, frozen.</summary>
    public Face DeletedFace(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > DeletedFaceCount, "BRepBuilderAPI_Sewing::DeletedFace");
        return ShapeCasts.Face(Published(_littleFace[index - 1]));
    }

    //=================================================================================================

    /// <summary>The number of degenerated shapes.</summary>
    public int DegeneratedShapeCount => _degenerated.Count;

    /// <summary>The <paramref name="index"/>-th degenerated shape, frozen.</summary>
    public Shape DegeneratedShape(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > DegeneratedShapeCount,
                                     "BRepBuilderAPI_Sewing::DegereratedShape");
        return Published(_degenerated[index - 1]);
    }

    //=================================================================================================

    /// <summary>True if the input shape is degenerated.</summary>
    public bool IsDegenerated(in Shape aShape)
    {
        Shape newShape = _reShape.InternalApply(ToWork(aShape));
        // Degenerated face
        if (aShape.Type == ShapeType.Face)
        {
            return newShape.IsNull;
        }
        if (newShape.IsNull)
        {
            return false;
        }
        // Degenerated edge
        if (newShape.Type == ShapeType.Edge)
        {
            return Tool.Degenerated(ShapeCasts.Edge(newShape));
        }
        // Degenerated wire
        if (newShape.Type == ShapeType.Wire)
        {
            bool isDegenerated = true;
            for (var it = new Iterator(newShape); it.More() && isDegenerated; it.Next())
            {
                isDegenerated = Tool.Degenerated(ShapeCasts.Edge(it.Value()));
            }
            return isDegenerated;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>True if the input shape has been modified.</summary>
    public bool IsModified(in Shape aShape)
    {
        Shape newShape = aShape;
        if (_oldShapes.ContainsKey(aShape))
        {
            newShape = FromWork(_oldShapes.GetValue(aShape));
        }
        if (!newShape.IsSame(aShape))
        {
            return true;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>The modified input shape, frozen (the shape itself if it isn't an input).</summary>
    public Shape Modified(in Shape aShape)
    {
        if (_oldShapes.ContainsKey(aShape))
        {
            return Published(_oldShapes.GetValue(aShape));
        }
        // if (myOldFaces.Contains(aShape)) return myOldFaces.FindFromKey(aShape);
        return aShape;
    }

    /// <summary>The modified input shape, as <see cref="Modified"/>, without freezing it (ported code).</summary>
    internal Shape InternalModified(in Shape aShape)
    {
        if (_oldShapes.ContainsKey(aShape))
        {
            return FromWork(_oldShapes.GetValue(aShape));
        }
        return aShape;
    }

    //=================================================================================================

    /// <summary>True if the input sub-shape has been modified.</summary>
    public bool IsModifiedSubShape(in Shape aShape)
    {
        Shape newShape = FromWork(_reShape.InternalApply(ToWork(aShape)));
        return !newShape.IsSame(aShape);
    }

    //=================================================================================================

    /// <summary>The modified sub-shape, frozen (the sub-shape itself if it isn't modified).</summary>
    public Shape ModifiedSubShape(in Shape aShape) =>
        Published(_reShape.InternalApply(ToWork(aShape)));

    /// <summary>The modified sub-shape, as <see cref="ModifiedSubShape"/>, without freezing it (ported code).</summary>
    internal Shape InternalModifiedSubShape(in Shape aShape) =>
        FromWork(_reShape.InternalApply(ToWork(aShape)));

    //=================================================================================================

    /// <summary>Prints the information on the console.</summary>
    public void Dump() => Dump(Console.Out);

    /// <summary>Prints the information (OCCT writes it to <c>std::cout</c>).</summary>
    public void Dump(TextWriter os)
    {
        int i, nbBounds = _boundFaces.Count, nbSections = 0;
        var mapVertices = new HashSet<Shape>(ShapeHasher.Instance);
        var mapEdges = new HashSet<Shape>(ShapeHasher.Instance);
        for (i = 1; i <= nbBounds; i++)
        {
            Shape bound = _boundFaces.KeyAt(i - 1);
            if (_boundSections.ContainsKey(bound))
            {
                nbSections += _boundSections.GetValue(bound).Extent();
            }
            else
            {
                nbSections++;
            }
            var exp = new Explorer(_reShape.InternalApply(bound), ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                mapEdges.Add(e);
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                mapVertices.Add(v1);
                mapVertices.Add(v2);
            }
        }
        os.Write(" " + '\n');
        os.Write("                        Information                         " + '\n');
        os.Write(" ===========================================================" + '\n');
        os.Write(" " + '\n');
        os.Write(" Number of input shapes      : " + _oldShapes.Count + '\n');
        os.Write(" Number of actual shapes     : " + _nbShapes + '\n');
        os.Write(" Number of Bounds            : " + nbBounds + '\n');
        os.Write(" Number of Sections          : " + nbSections + '\n');
        os.Write(" Number of Edges             : " + mapEdges.Count + '\n');
        os.Write(" Number of Vertices          : " + _nbVertices + '\n');
        os.Write(" Number of Nodes             : " + mapVertices.Count + '\n');
        os.Write(" Number of Free Edges        : " + _freeEdges.Count + '\n');
        os.Write(" Number of Contiguous Edges  : " + _contigousEdges.Count + '\n');
        os.Write(" Number of Multiple Edges    : " + _multipleEdges.Count + '\n');
        os.Write(" Number of Degenerated Edges : " + _degenerated.Count + '\n');
        os.Write(" ===========================================================" + '\n');
        os.Write(" " + '\n');
    }

    //=======================================================================
    // function : FaceAnalysis
    // purpose  : Remove
    //	     Modifies:
    //                      myNbShapes
    //                      myOldShapes
    //
    //           Constructs:
    //                      myDegenerated
    //=======================================================================

    /// <summary>Makes small edges degenerated and removes small faces. Called from Perform only.</summary>
    protected virtual void FaceAnalysis(ProgressRange progress = default)
    {
        if (!_shape.IsNull && _oldShapes.Count == 0)
        {
            Add(_shape);
            _shape.Nullify();
        }

        var b = new BrepBuilder();
        var smallEdges = new HashSet<Shape>(ShapeHasher.Instance);
        var gluedVertices = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int i = 1;
        using var ps = new ProgressScope(progress, "Shape analysis", _oldShapes.Count);
        for (i = 1; i <= _oldShapes.Count && ps.More(); i++, ps.Next())
        {
            for (var fexp = new Explorer(_oldShapes.ValueAt(i - 1), ShapeType.Face); fexp.More(); fexp.Next())
            {
                // Retrieve current face
                Shape tmpShape = fexp.Current(); // for porting
                Face face = ShapeCasts.Face(tmpShape);
                int nbEdges = 0, nbSmall = 0;

                // Build replacing face
                tmpShape = face.EmptyCopied().Oriented(Forward); // for porting
                Face nface = ShapeCasts.Face(tmpShape);
                bool isFaceChanged = false;

                var witer = new Iterator(face.Oriented(Forward));
                for (; witer.More(); witer.Next())
                {
                    // Retrieve current wire
                    tmpShape = witer.Value(); // for porting
                    if (tmpShape.Type != ShapeType.Wire)
                    {
                        continue;
                    }
                    Wire wire = ShapeCasts.Wire(tmpShape);

                    // Build replacing wire
                    tmpShape = wire.EmptyCopied().Oriented(Forward);
                    Wire nwire = ShapeCasts.Wire(tmpShape);
                    bool isWireChanged = false;

                    var eiter = new Iterator(wire.Oriented(Forward));
                    for (; eiter.More(); eiter.Next())
                    {
                        // Retrieve current edge
                        tmpShape = eiter.Value(); // for porting
                        Edge edge = ShapeCasts.Edge(tmpShape);
                        nbEdges++;

                        // Process degenerated edge
                        if (Tool.Degenerated(edge))
                        {
                            b.Add(nwire, edge); // Old edge kept
                            _degenerated.Add(edge);
                            nbSmall++;
                            continue;
                        }

                        bool isSmall = smallEdges.Contains(edge);
                        if (!isSmall)
                        {
                            // Check for small edge
                            Curve3D? c3d = Tool.Curve(edge, out double first, out double last);
                            if (c3d is null)
                            {
                                // Warning: Possibly small edge can be sewed: No 3D curve
                            }
                            else
                            {
                                // Evaluate curve compactness
                                const int npt = 5;
                                var cp = new Point3D((c3d.Value(first).XYZ() + c3d.Value(last).XYZ()) * 0.5);
                                double dist, maxdist = 0.0;
                                double delta = (last - first) / (npt - 1);
                                for (int idx = 0; idx < npt; idx++)
                                {
                                    dist = cp.DistanceTo(c3d.Value(first + idx * delta));
                                    if (maxdist < dist)
                                    {
                                        maxdist = dist;
                                    }
                                }
                                isSmall = (2.0 * maxdist <= MinTolerance());
                            }

                            if (isSmall)
                            {
                                // Store small edge in the map
                                smallEdges.Add(edge);

                                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                                Shape nv1 = _reShape.InternalApply(v1), nv2 = _reShape.InternalApply(v2);

                                // Store glued vertices
                                if (!nv1.IsSame(v1))
                                {
                                    NCollection_List<Shape> vlist1 = gluedVertices.GetValue(nv1);
                                    // First vertex was already glued
                                    if (!nv2.IsSame(v2))
                                    {
                                        // Merge lists of glued vertices
                                        if (!nv1.IsSame(nv2))
                                        {
                                            var liter = new NCollection_List<Shape>.Iterator(gluedVertices.GetValue(nv2));
                                            for (; liter.More(); liter.Next())
                                            {
                                                Shape v = liter.Value();
                                                _reShape.Replace(v, nv1.Oriented(v.Orientation));
                                                vlist1.Append(v);
                                            }
                                            gluedVertices.Remove(nv2);
                                        }
                                    }
                                    else
                                    {
                                        // Add second vertex to the existing list
                                        vlist1.Append(v2);
                                        _reShape.Replace(v2, nv1.Oriented(v2.Orientation));
                                    }
                                }
                                else if (!nv2.IsSame(v2))
                                {
                                    // Add first vertex to the existing list
                                    gluedVertices.GetValue(nv2).Append(v1);
                                    _reShape.Replace(v1, nv2.Oriented(v1.Orientation));
                                }
                                else if (!v1.IsSame(v2))
                                {
                                    // Record new glued vertices
                                    var nv = new Vertex();
                                    b.MakeVertex(ref nv);
                                    var vlist = new NCollection_List<Shape>();
                                    vlist.Append(v1);
                                    vlist.Append(v2);
                                    gluedVertices.Add(nv, vlist);
                                    _reShape.Replace(v1, nv.Oriented(v1.Orientation));
                                    _reShape.Replace(v2, nv.Oriented(v2.Orientation));
                                }
                            }
                        }

                        // Replace small edge
                        if (isSmall)
                        {
                            nbSmall++;
                            // Create new degenerated edge
                            tmpShape = edge.Oriented(Forward);
                            Edge fedge = ShapeCasts.Edge(tmpShape);
                            Curve2D? c2d = Tool.CurveOnSurface(fedge, face, out double pfirst, out double plast);
                            if (c2d is not null)
                            {
                                var nedge = new Edge();
                                b.MakeEdge(ref nedge);
                                b.UpdateEdge(nedge, c2d, face, global::Kerf.Engine.Foundation.Precision.Confusion());
                                b.Range(nedge, pfirst, plast);
                                b.Degenerated(nedge, true);
                                Exploration.Vertices(fedge, out Vertex v1, out Vertex v2);
                                b.Add(nedge, _reShape.InternalApply(v1).Oriented(v1.Orientation));
                                b.Add(nedge, _reShape.InternalApply(v2).Oriented(v2.Orientation));
                                b.Add(nwire, nedge.Oriented(edge.Orientation));
                                _degenerated.Add(nedge);
                            }
                            isWireChanged = true;
                        }
                        else
                        {
                            b.Add(nwire, edge); // Old edge kept
                        }
                    }

                    // Record wire in the new face
                    if (isWireChanged)
                    {
                        b.Add(nface, nwire.Oriented(wire.Orientation));
                        isFaceChanged = true;
                    }
                    else
                    {
                        b.Add(nface, wire);
                    }
                }

                // Remove small face
                if (nbSmall == nbEdges)
                {
                    _littleFace.Add(face);
                    _reShape.Remove(face);
                }
                else if (isFaceChanged)
                {
                    _reShape.Replace(face, nface.Oriented(face.Orientation));
                }
            }
        }

        // Update glued vertices
        for (int mIter = 0; mIter < gluedVertices.Count; mIter++)
        {
            Vertex vnew = ShapeCasts.Vertex(gluedVertices.KeyAt(mIter));
            var coord = new Xyz(0.0, 0.0, 0.0);
            int nbPoints = 0;
            NCollection_List<Shape> vlist = gluedVertices.ValueAt(mIter);
            var liter1 = new NCollection_List<Shape>.Iterator(vlist);
            for (; liter1.More(); liter1.Next())
            {
                coord += Tool.Pnt(ShapeCasts.Vertex(liter1.Value())).XYZ();
                nbPoints++;
            }
            if (nbPoints != 0)
            {
                var vp = new Point3D(coord / nbPoints);
                double tol = 0.0, mtol = 0.0;
                var liter2 = new NCollection_List<Shape>.Iterator(vlist);
                for (; liter2.More(); liter2.Next())
                {
                    double vtol = Tool.Tolerance(ShapeCasts.Vertex(liter2.Value()));
                    if (mtol < vtol)
                    {
                        mtol = vtol;
                    }
                    vtol = vp.DistanceTo(Tool.Pnt(ShapeCasts.Vertex(liter2.Value())));
                    if (tol < vtol)
                    {
                        tol = vtol;
                    }
                }
                b.UpdateVertex(vnew, vp, tol + mtol);
            }
        }

        // Update input shapes
        for (i = 1; i <= _oldShapes.Count; i++)
        {
            _oldShapes.ValueAt(i - 1) = _reShape.InternalApply(_oldShapes.ValueAt(i - 1));
        }
    }

    //=======================================================================
    // function : FindFreeBoundaries
    // purpose  : Constructs :
    //                      myBoundFaces     (bound = list of faces) - REFERENCE
    //                      myVertexNode     (vertex = node)
    //                      myVertexNodeFree (floating vertex = node)
    //
    //=======================================================================

    /// <summary>Finds the free boundaries of the faces and their vertices. Called from Perform only.</summary>
    protected virtual void FindFreeBoundaries()
    {
        // Take into account the context shape if needed
        var newShapes = new IndexedSet<Shape>(ShapeHasher.Instance);
        if (!_shape.IsNull)
        {
            if (_oldShapes.Count == 0)
            {
                Add(_shape);
                _shape.Nullify();
            }
            else
            {
                Shape newShape = _reShape.InternalApply(_shape);
                if (!newShape.IsNull)
                {
                    newShapes.Add(newShape);
                }
            }
        }
        // Create map Edge -> Faces
        var edgeFaces = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int i, nbShapes = _oldShapes.Count;
        for (i = 1; i <= nbShapes; i++)
        {
            // Retrieve new shape
            Shape shape = _oldShapes.ValueAt(i - 1);
            if (shape.IsNull)
            {
                continue;
            }
            newShapes.Add(shape);
            // Explore shape to find all boundaries
            for (var eExp = new Explorer(shape, ShapeType.Edge); eExp.More(); eExp.Next())
            {
                Shape edge = eExp.Current();
                if (!edgeFaces.ContainsKey(edge))
                {
                    var listFaces = new NCollection_List<Shape>();
                    edgeFaces.Add(edge, listFaces);
                }
            }
        }
        // Fill map Edge -> Faces
        nbShapes = newShapes.Count;
        var mapFaces = new HashSet<Shape>(ShapeHasher.Instance);
        for (i = 1; i <= nbShapes; i++)
        {
            // Explore shape to find all faces
            var fExp = new Explorer(newShapes[i - 1], ShapeType.Face);
            for (; fExp.More(); fExp.Next())
            {
                Shape face = fExp.Current();
                if (mapFaces.Contains(face))
                {
                    continue;
                }
                else
                {
                    mapFaces.Add(face);
                }
                // Explore face to find all boundaries
                for (var iw = new Iterator(face); iw.More(); iw.Next())
                {
                    if (iw.Value().Type != ShapeType.Wire)
                    {
                        continue;
                    }
                    for (var iIe = new Iterator(iw.Value()); iIe.More(); iIe.Next())
                    {
                        Shape edge = iIe.Value();

                        if (edgeFaces.ContainsKey(edge))
                        {
                            edgeFaces.GetValue(edge).Append(face);
                        }
                    }
                }
            }
        }
        // Find free boundaries
        for (int iterEF = 0; iterEF < edgeFaces.Count; iterEF++)
        {
            NCollection_List<Shape> listFaces = edgeFaces.ValueAt(iterEF);
            int nbFaces = listFaces.Extent();
            Shape edge = edgeFaces.KeyAt(iterEF);
            if (edge.Orientation == Internal)
            {
                continue;
            }
            bool isSeam = false;
            if (nbFaces == 1)
            {
                Face face = ShapeCasts.Face(listFaces.First());
                isSeam = Tool.IsClosed(ShapeCasts.Edge(edge), face);
                if (isSeam)
                {
                    var aB = new BrepBuilder();
                    Shape anewEdge = edge.EmptyCopied();
                    var itV = new Iterator(edge);
                    for (; itV.More(); itV.Next())
                    {
                        aB.Add(anewEdge, itV.Value());
                    }

                    Curve2D? c2dold =
                        Tool.CurveOnSurface(ShapeCasts.Edge(edge),
                                                 ShapeCasts.Face(listFaces.First()),
                                                 out double first2d,
                                                 out double last2d);

                    Curve2D? c2d = null;
                    var b = new BrepBuilder();
                    b.UpdateEdge(ShapeCasts.Edge(anewEdge), c2d, c2d, ShapeCasts.Face(listFaces.First()), 0);
                    b.UpdateEdge(ShapeCasts.Edge(anewEdge), c2dold, ShapeCasts.Face(listFaces.First()), 0);

                    Tool.Range(ShapeCasts.Edge(edge), out double first, out double last);
                    aB.Range(ShapeCasts.Edge(anewEdge), first, last);
                    aB.Range(ShapeCasts.Edge(anewEdge), ShapeCasts.Face(listFaces.First()), first2d, last2d);
                    _reShape.Replace(edge, anewEdge);
                    edge = anewEdge;

                    isSeam = false;
                }
            }
            bool isBoundFloat = (_floatingEdgesMode && nbFaces == 0);
            bool isBound = (_faceMode && ((_nonmanifold && nbFaces != 0) || (nbFaces == 1 && !isSeam)));
            if (isBound || isBoundFloat)
            {
                // Ignore degenerated edge
                if (Tool.Degenerated(ShapeCasts.Edge(edge)))
                {
                    continue;
                }
                // Add to BoundFaces
                var listFacesCopy = new NCollection_List<Shape>();
                listFacesCopy.Append(listFaces);
                _boundFaces.Add(edge, listFacesCopy);
                // Process edge vertices
                Exploration.Vertices(ShapeCasts.Edge(edge), out Vertex vFirst, out Vertex vLast);
                if (vFirst.IsNull || vLast.IsNull)
                {
                    continue;
                }
                if (vFirst.Orientation == Internal || vLast.Orientation == Internal)
                {
                    continue;
                }
                if (isBound)
                {
                    // Add to VertexNode
                    if (!_vertexNode.ContainsKey(vFirst))
                    {
                        _vertexNode.Add(vFirst, vFirst);
                    }
                    if (!_vertexNode.ContainsKey(vLast))
                    {
                        _vertexNode.Add(vLast, vLast);
                    }
                }
                else
                {
                    // Add to VertexNodeFree
                    if (!_vertexNodeFree.ContainsKey(vFirst))
                    {
                        _vertexNodeFree.Add(vFirst, vFirst);
                    }
                    if (!_vertexNodeFree.ContainsKey(vLast))
                    {
                        _vertexNodeFree.Add(vLast, vLast);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : VerticesAssembling
    // purpose  : Modifies :
    //                      myVertexNode     (nodes glued)
    //                      myVertexNodeFree (nodes glued)
    //                      myNodeSections   (lists of sections merged for glued nodes)
    //
    //=======================================================================

    private static bool CreateNewNodes(IndexedMap<Shape, Shape> nodeNearestNode,
                                       IndexedMap<Shape, NCollection_List<Shape>> nodeVertices,
                                       IndexedMap<Shape, Shape> aVertexNode,
                                       Dictionary<Shape, NCollection_List<Shape>> aNodeEdges)
    {
        // Create new nodes
        var b = new BrepBuilder();
        var oldNodeNewNode = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var newNodeOldNodes = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        for (int iter = 0; iter < nodeNearestNode.Count; iter++)
        {
            // Retrieve a pair of nodes to merge
            Shape oldnode1 = nodeNearestNode.KeyAt(iter);
            Shape oldnode2 = nodeNearestNode.ValueAt(iter);
            // Second node should also be in the map
            if (!nodeNearestNode.ContainsKey(oldnode2))
            {
                continue;
            }
            // Get new node for old node #1
            if (oldNodeNewNode.ContainsKey(oldnode1))
            {
                Shape newnode1 = oldNodeNewNode.GetValue(oldnode1);
                if (oldNodeNewNode.ContainsKey(oldnode2))
                {
                    Shape newnode2 = oldNodeNewNode.GetValue(oldnode2);
                    if (!newnode1.IsSame(newnode2))
                    {
                        // Change data for new node #2
                        NCollection_List<Shape> lnode1 = newNodeOldNodes.GetValue(newnode1);
                        var itn = new NCollection_List<Shape>.Iterator(newNodeOldNodes.GetValue(newnode2));
                        for (; itn.More(); itn.Next())
                        {
                            Shape node2 = itn.Value();
                            lnode1.Append(node2);
                            oldNodeNewNode.GetValueRef(node2) = newnode1;
                        }
                        newNodeOldNodes.Remove(newnode2);
                    }
                }
                else
                {
                    // Old node #2 is not bound - add to old node #1
                    oldNodeNewNode[oldnode2] = newnode1;
                    newNodeOldNodes.GetValue(newnode1).Append(oldnode2);
                }
            }
            else
            {
                if (oldNodeNewNode.ContainsKey(oldnode2))
                {
                    // Old node #1 is not bound - add to old node #2
                    Shape newnode2 = oldNodeNewNode.GetValue(oldnode2);
                    oldNodeNewNode[oldnode1] = newnode2;
                    newNodeOldNodes.GetValue(newnode2).Append(oldnode1);
                }
                else
                {
                    // Nodes are not bound - create new node
                    var newnode = new Vertex();
                    b.MakeVertex(ref newnode);
                    oldNodeNewNode[oldnode1] = newnode;
                    oldNodeNewNode[oldnode2] = newnode;
                    var lnodes = new NCollection_List<Shape>();
                    lnodes.Append(oldnode1);
                    lnodes.Append(oldnode2);
                    newNodeOldNodes.Add(newnode, lnodes);
                }
            }
        }

        // Stop if no new nodes created
        if (newNodeOldNodes.Count == 0)
        {
            return false;
        }

        for (int iter1 = 0; iter1 < newNodeOldNodes.Count; iter1++)
        {
            Vertex newnode = ShapeCasts.Vertex(newNodeOldNodes.KeyAt(iter1));
            // Calculate new node center point
            var theCoordinates = new Xyz(0.0, 0.0, 0.0);
            var lvert = new NCollection_List<Shape>(); // Accumulate node vertices
            var medge = new HashSet<Shape>(ShapeHasher.Instance);
            var ledge = new NCollection_List<Shape>(); // Accumulate node edges
            // Iterate on old nodes
            var itn = new NCollection_List<Shape>.Iterator(newNodeOldNodes.ValueAt(iter1));
            for (; itn.More(); itn.Next())
            {
                Shape oldnode = itn.Value();
                // Iterate on node vertices
                var itv1 = new NCollection_List<Shape>.Iterator(nodeVertices.GetValue(oldnode));
                for (; itv1.More(); itv1.Next())
                {
                    Vertex vertex = ShapeCasts.Vertex(itv1.Value());
                    // Change node for vertex
                    aVertexNode.GetValueRef(vertex) = newnode;
                    // Accumulate coordinates
                    theCoordinates += Tool.Pnt(vertex).XYZ();
                    lvert.Append(vertex);
                }
                // Iterate on node edges
                NCollection_List<Shape> edges = aNodeEdges.GetValue(oldnode);
                var ite = new NCollection_List<Shape>.Iterator(edges);
                for (; ite.More(); ite.Next())
                {
                    Shape edge = ite.Value();
                    if (!medge.Contains(edge))
                    {
                        medge.Add(edge);
                        ledge.Append(edge);
                    }
                }
                // Unbind old node edges
                aNodeEdges.Remove(oldnode);
            }
            // Bind new node edges
            aNodeEdges[newnode] = ledge;
            var center = new Point3D(theCoordinates / lvert.Extent());
            // Calculate new node tolerance
            double toler = 0.0;
            var itv = new NCollection_List<Shape>.Iterator(lvert);
            for (; itv.More(); itv.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(itv.Value());
                double t = center.DistanceTo(Tool.Pnt(vertex)) + Tool.Tolerance(vertex);
                if (toler < t)
                {
                    toler = t;
                }
            }
            // Update new node parameters
            b.UpdateVertex(newnode, center, toler);
        }

        return true;
    }

    private static int IsMergedVertices(in Shape face1,
                                        in Shape e1,
                                        in Shape e2,
                                        in Shape vtx1,
                                        in Shape vtx2)
    {
        // Case of floating edges
        if (face1.IsNull)
        {
            return (!IsClosedShape(e1, vtx1, vtx2)) ? 1 : 0;
        }

        // Find wires containing given edges
        Shape wire1 = new(), wire2 = new();
        var itw = new Explorer(face1, ShapeType.Wire);
        for (; itw.More() && (wire1.IsNull || wire2.IsNull); itw.Next())
        {
            var ite = new Iterator(itw.Current(), false);
            for (; ite.More() && (wire1.IsNull || wire2.IsNull); ite.Next())
            {
                if (wire1.IsNull && e1.IsSame(ite.Value()))
                {
                    wire1 = itw.Current();
                }
                if (wire2.IsNull && e2.IsSame(ite.Value()))
                {
                    wire2 = itw.Current();
                }
            }
        }
        int status = 0;
        if (!wire1.IsNull && !wire2.IsNull)
        {
            if (wire1.IsSame(wire2))
            {
                for (var aIte = new Iterator(wire1, false); aIte.More(); aIte.Next())
                {
                    Exploration.Vertices(ShapeCasts.Edge(aIte.Value()), out Vertex ve1, out Vertex ve2);
                    if ((ve1.IsSame(vtx1) && ve2.IsSame(vtx2)) || (ve2.IsSame(vtx1) && ve1.IsSame(vtx2)))
                    {
                        return (IsClosedShape(aIte.Value(), vtx1, vtx2) ? 0 : 1);
                    }
                }
                if (IsClosedShape(wire1, vtx1, vtx2))
                {
                    Exploration.Vertices(ShapeCasts.Wire(wire1), out Vertex v1, out Vertex v2);
                    bool isEndVertex =
                        ((v1.IsSame(vtx1) && v2.IsSame(vtx2)) || (v2.IsSame(vtx1) && v1.IsSame(vtx2)));
                    if (!isEndVertex)
                    {
                        status = 1;
                    }
                }
                else
                {
                    status = 1;
                }
            }
            else
            {
                status = -1;
            }
        }
        return status;
    }

    private static bool GlueVertices(IndexedMap<Shape, Shape> aVertexNode,
                                     Dictionary<Shape, NCollection_List<Shape>> aNodeEdges,
                                     IndexedMap<Shape, NCollection_List<Shape>> aBoundFaces,
                                     double tolerance,
                                     ProgressRange progress)
    {
        // Create map of node -> vertices
        var nodeVertices = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var filter = new CellFilter<VertexInspector, int, Xyz>(tolerance);
        var inspector = new VertexInspector(tolerance);
        for (int anIter1 = 0; anIter1 < aVertexNode.Count; anIter1++)
        {
            Shape vertex = aVertexNode.KeyAt(anIter1);
            Vertex node = ShapeCasts.Vertex(aVertexNode.ValueAt(anIter1));
            if (nodeVertices.ContainsKey(node))
            {
                nodeVertices.GetValue(node).Append(vertex);
            }
            else
            {
                var vlist = new NCollection_List<Shape>();
                vlist.Append(vertex);
                nodeVertices.Add(node, vlist);
                Point3D pnt = Tool.Pnt(ShapeCasts.Vertex(node));
                filter.Add(nodeVertices.IndexOf(node) + 1, pnt.XYZ());
                inspector.Add(pnt.XYZ());
            }
        }
        int nbNodes = nodeVertices.Count;
        // Merge nearest nodes
        var nodeNearestNode = new IndexedMap<Shape, Shape>(ShapeHasher.Instance);
        using var ps = new ProgressScope(progress, "Glueing nodes", nbNodes, true);
        for (int i = 1; i <= nbNodes && ps.More(); i++, ps.Next())
        {
            Vertex node1 = ShapeCasts.Vertex(nodeVertices.KeyAt(i - 1));
            // Find near nodes
            Point3D pt1 = Tool.Pnt(node1);
            inspector.SetCurrent(pt1.XYZ());
            Xyz pntMin = VertexInspector.Shift(pt1.XYZ(), -tolerance);
            Xyz pntMax = VertexInspector.Shift(pt1.XYZ(), tolerance);
            filter.Inspect(pntMin, pntMax, ref inspector);
            if (inspector.ResInd().IsEmpty())
            {
                continue;
            }
            // Retrieve list of edges for the first node
            NCollection_List<Shape> ledges1 = aNodeEdges.GetValue(node1);
            // Explore list of near nodes and fill the sequence of glued nodes
            var seqNodes = new NCollection_Sequence<Shape>();
            var listNodesSameEdge = new NCollection_List<Shape>();
            var iter1 = new NCollection_List<int>.Iterator(inspector.ResInd());
            for (; iter1.More(); iter1.Next())
            {
                Vertex node2 = ShapeCasts.Vertex(nodeVertices.KeyAt(iter1.Value() - 1));
                if (node1 == node2)
                {
                    continue;
                }
                // Retrieve list of edges for the second node
                NCollection_List<Shape> ledges2 = aNodeEdges.GetValue(node2);
                // Check merging condition for the pair of nodes
                int status = 0;
                bool isSameEdge = false;
                // Explore edges of the first node
                var ie1 = new NCollection_List<Shape>.Iterator(ledges1);
                for (; ie1.More() && status == 0 && !isSameEdge; ie1.Next())
                {
                    Shape e1 = ie1.Value();
                    // Obtain real vertex from edge
                    Shape v1 = node1;
                    { // szv: Use brackets to destroy local variables
                        Exploration.Vertices(ShapeCasts.Edge(e1), out Vertex ov1, out Vertex ov2);
                        if (aVertexNode.ContainsKey(ov1))
                        {
                            if (node1.IsSame(aVertexNode.GetValue(ov1)))
                            {
                                v1 = ov1;
                            }
                        }
                        if (aVertexNode.ContainsKey(ov2))
                        {
                            if (node1.IsSame(aVertexNode.GetValue(ov2)))
                            {
                                v1 = ov2;
                            }
                        }
                    }
                    // Create map of faces for e1
                    var faces1 = new HashSet<Shape>(ShapeHasher.Instance);
                    NCollection_List<Shape> lfac1 = aBoundFaces.GetValue(e1);
                    if (lfac1.Extent() != 0)
                    {
                        var itf = new NCollection_List<Shape>.Iterator(lfac1);
                        for (; itf.More(); itf.Next())
                        {
                            if (!itf.Value().IsNull)
                            {
                                faces1.Add(itf.Value());
                            }
                        }
                    }
                    // Explore edges of the second node
                    var ie2 = new NCollection_List<Shape>.Iterator(ledges2);
                    for (; ie2.More() && status == 0 && !isSameEdge; ie2.Next())
                    {
                        Shape e2 = ie2.Value();
                        // Obtain real vertex from edge
                        Shape v2 = node2;
                        { // szv: Use brackets to destroy local variables
                            Exploration.Vertices(ShapeCasts.Edge(e2), out Vertex ov1, out Vertex ov2);
                            if (aVertexNode.ContainsKey(ov1))
                            {
                                if (node2.IsSame(aVertexNode.GetValue(ov1)))
                                {
                                    v2 = ov1;
                                }
                            }
                            if (aVertexNode.ContainsKey(ov2))
                            {
                                if (node2.IsSame(aVertexNode.GetValue(ov2)))
                                {
                                    v2 = ov2;
                                }
                            }
                        }
                        // Explore faces for e2
                        NCollection_List<Shape> lfac2 = aBoundFaces.GetValue(e2);
                        if (lfac2.Extent() != 0)
                        {
                            var itf = new NCollection_List<Shape>.Iterator(lfac2);
                            for (; itf.More() && status == 0 && !isSameEdge; itf.Next())
                            {
                                // Check merging conditions for the same face
                                if (faces1.Contains(itf.Value()))
                                {
                                    int stat = IsMergedVertices(itf.Value(), e1, e2, v1, v2);
                                    if (stat == 1)
                                    {
                                        isSameEdge = true;
                                    }
                                    else
                                    {
                                        status = stat;
                                    }
                                }
                            }
                        }
                        else if (faces1.Count == 0 && e1 == e2)
                        {
                            int stat = IsMergedVertices(new Face(), e1, e1, v1, v2);
                            if (stat == 1)
                            {
                                isSameEdge = true;
                            }
                            else
                            {
                                status = stat;
                            }
                            break;
                        }
                    }
                }
                if (status != 0)
                {
                    continue;
                }
                if (isSameEdge)
                {
                    listNodesSameEdge.Append(node2);
                }
                // Append near node to the sequence
                Point3D pt2 = Tool.Pnt(node2);
                double dist = pt1.DistanceTo(pt2);
                if (dist < tolerance)
                {
                    bool isIns = false;
                    for (int kk = 1; kk <= seqNodes.Length() && !isIns; kk++)
                    {
                        Point3D pt = Tool.Pnt(ShapeCasts.Vertex(seqNodes.Value(kk)));
                        if (dist < pt1.DistanceTo(pt))
                        {
                            seqNodes.InsertBefore(kk, node2);
                            isIns = true;
                        }
                    }
                    if (!isIns)
                    {
                        seqNodes.Append(node2);
                    }
                }
            }
            if (seqNodes.Length() != 0)
            {
                // Remove nodes near to some other from the same edge
                if (listNodesSameEdge.Extent() != 0)
                {
                    var lInt = new NCollection_List<Shape>.Iterator(listNodesSameEdge);
                    for (; lInt.More(); lInt.Next())
                    {
                        Vertex n2 = ShapeCasts.Vertex(lInt.Value());
                        Point3D p2 = Tool.Pnt(n2);
                        for (int k = 1; k <= seqNodes.Length();)
                        {
                            Vertex n1 = ShapeCasts.Vertex(seqNodes.Value(k));
                            if (n1 != n2)
                            {
                                Point3D p1 = Tool.Pnt(n1);
                                if (p2.DistanceTo(p1) >= pt1.DistanceTo(p1))
                                {
                                    k++;
                                    continue;
                                }
                            }
                            seqNodes.Remove(k);
                        }
                    }
                }
                // Bind nearest node if at least one exists
                if (seqNodes.Length() != 0)
                {
                    nodeNearestNode.Add(node1, seqNodes.First());
                }
            }
            inspector.ClearResList();
        }

        // Create new nodes for chained nearest nodes
        if (nodeNearestNode.Count == 0)
        {
            return false;
        }

        return CreateNewNodes(nodeNearestNode, nodeVertices, aVertexNode, aNodeEdges);
    }

    /// <summary>Glues the vertices of the bounds within the tolerance into nodes. Called from Perform only.</summary>
    protected virtual void VerticesAssembling(ProgressRange progress = default)
    {
        int nbVert = _vertexNode.Count;
        int nbVertFree = _vertexNodeFree.Count;
        using var ps = new ProgressScope(progress, "Vertices assembling", 2);
        if (nbVert != 0 || nbVertFree != 0)
        {
            // Fill map node -> sections
            int i;
            for (i = 1; i <= _boundFaces.Count; i++)
            {
                Shape bound = _boundFaces.KeyAt(i - 1);
                for (var itv = new Iterator(bound, false); itv.More(); itv.Next())
                {
                    Shape node = itv.Value();
                    if (_nodeSections.ContainsKey(node))
                    {
                        _nodeSections.GetValue(node).Append(bound);
                    }
                    else
                    {
                        var lbnd = new NCollection_List<Shape>();
                        lbnd.Append(bound);
                        _nodeSections[node] = lbnd;
                    }
                }
            }
            // Glue vertices
            if (nbVert != 0)
            {
                while (GlueVertices(_vertexNode, _nodeSections, _boundFaces, _tolerance, ps.Next()))
                {
                }
            }
            if (!ps.More())
            {
                return;
            }
            if (nbVertFree != 0)
            {
                while (GlueVertices(_vertexNodeFree, _nodeSections, _boundFaces, _tolerance, ps.Next()))
                {
                }
            }
        }
    }

    //=======================================================================
    // function : replaceNMVertices
    // purpose  : internal use (static)
    //=======================================================================
    private static void ReplaceNMVertices(in Edge edge,
                                          in Vertex v1,
                                          in Vertex v2,
                                          ReShape reShape)
    {
        // To keep NM vertices on edge
        var seqNMVert = new NCollection_Sequence<Shape>();
        var seqNMPars = new NCollection_Sequence<double>();
        bool hasNMVert = FindNMVertices(edge, seqNMVert, seqNMPars);
        if (!hasNMVert)
        {
            return;
        }
        Tool.Range(edge, out double first, out double last);
        Curve3D? c3d = Tool.Curve(edge, out Placement loc, out first, out last);
        if (c3d is null)
        {
            return;
        }
        var edVert = new NCollection_Sequence<Shape>();
        var edParams = new NCollection_Sequence<double>();
        int i = 1, nb = seqNMPars.Length();

        for (; i <= nb; i++)
        {
            double apar = seqNMPars.Value(i);
            if (Math.Abs(apar - first) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                reShape.Replace(seqNMVert.Value(i), v1);
                continue;
            }
            if (Math.Abs(apar - last) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                reShape.Replace(seqNMVert.Value(i), v2);
                continue;
            }
            Shape v = seqNMVert.Value(i);
            int j = 1;
            for (; j <= edParams.Length(); j++)
            {
                double apar2 = edParams.Value(j);
                if (Math.Abs(apar - apar2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    reShape.Replace(v, edVert.Value(j));
                    break;
                }
                else if (apar < apar2)
                {
                    Shape anewV = v.EmptyCopied();
                    edVert.InsertBefore(j, anewV);
                    edParams.InsertBefore(j, apar);
                    NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();
                    var pRep = new PointOnCurve(apar, c3d, loc);
                    alistrep.Append(pRep);
                    reShape.Replace(v, anewV);
                    break;
                }
            }
            if (j > edParams.Length())
            {
                Shape anewV = v.EmptyCopied();
                edVert.Append(anewV);
                edParams.Append(apar);
                NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();
                var pRep = new PointOnCurve(apar, c3d, loc);
                alistrep.Append(pRep);
                reShape.Replace(v, anewV);
            }
        }

        int newnb = edParams.Length();
        if (newnb < nb)
        {
            Shape anewEdge = edge.EmptyCopied();
            Orientation ori = edge.Orientation;
            anewEdge.Orient(Forward);
            var b = new BrepBuilder();
            b.Add(anewEdge, v1);
            b.Add(anewEdge, v2);

            for (i = 1; i <= edVert.Length(); i++)
            {
                b.Add(anewEdge, edVert.Value(i));
            }
            anewEdge.Orient(ori);
            reShape.Replace(edge, anewEdge);
        }
    }

    //=======================================================================
    // function : ReplaceEdge
    // purpose  : internal use (static)
    //=======================================================================

    private static void ReplaceEdge(in Shape oldEdge, in Shape theNewShape, ReShape aReShape)
    {
        Shape oldShape = aReShape.InternalApply(oldEdge);
        Shape newShape = aReShape.InternalApply(theNewShape);
        if (oldShape.IsSame(newShape) || aReShape.IsRecorded(newShape))
        {
            return;
        }

        aReShape.Replace(oldShape, newShape);
        Vertex v1new = new(), v2new = new();
        Exploration.Vertices(ShapeCasts.Edge(oldShape), out Vertex v1old, out Vertex v2old);
        Orientation orold = oldShape.Orientation;
        Orientation ornew = orold;
        if (newShape.Type == ShapeType.Edge)
        {
            Edge en = ShapeCasts.Edge(newShape);
            Exploration.Vertices(en, out v1new, out v2new);
            ornew = en.Orientation;
            ReplaceNMVertices(en, v1new, v2new, aReShape);
        }
        else if (newShape.Type == ShapeType.Wire)
        {
            for (var aex = new Explorer(newShape, ShapeType.Edge); aex.More(); aex.Next())
            {
                Edge ed = ShapeCasts.Edge(aex.Current());
                ornew = ed.Orientation;
                Exploration.Vertices(ed, out Vertex v1, out Vertex v2);
                ReplaceNMVertices(ed, v1, v2, aReShape);
                if (v1new.IsNull)
                {
                    v1new = v1;
                }
                v2new = v2;
            }
        }

        v1new.Orient(v1old.Orientation);
        v2new.Orient(v2old.Orientation);
        if (v1old.IsSame(v2old) && !v1old.IsSame(v1new) && !aReShape.IsRecorded(v1new))
        {
            aReShape.Replace(v1old, v1new);
            return;
        }
        if (orold == ornew)
        {
            v1new.Orient(v1old.Orientation);
            v2new.Orient(v2old.Orientation);
            if (!v1old.IsSame(v1new) && !v1old.IsSame(v2new) && !aReShape.IsRecorded(v1new))
            {
                aReShape.Replace(v1old, v1new);
            }
            if (!v2old.IsSame(v2new) && !v2old.IsSame(v1new) && !aReShape.IsRecorded(v2new))
            {
                aReShape.Replace(v2old, v2new);
            }
        }
        else
        {
            v1new.Orient(v2old.Orientation);
            v2new.Orient(v1old.Orientation);
            if (!v1old.IsSame(v2new) && !v1old.IsSame(v1new) && !aReShape.IsRecorded(v2new))
            {
                aReShape.Replace(v1old, v2new);
            }
            if (!v2old.IsSame(v2new) && !v2old.IsSame(v1new) && !aReShape.IsRecorded(v1new))
            {
                aReShape.Replace(v2old, v1new);
            }
        }
    }

    //=======================================================================
    // function : Merging
    // purpose  : Modifies :
    //                   myHasFreeBound
    //
    //=======================================================================

    /// <summary>Merges the bounds (or their cutting sections) with their nearest edges.</summary>
    protected void Merging(bool passage, ProgressRange progress = default)
    {
        _ = passage; // OCCT: unused (firstTime)
        var b = new BrepBuilder();
        //  NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> MergedEdges;
        using var ps = new ProgressScope(progress, "Merging bounds", _boundFaces.Count);
        for (int iterB = 1; iterB <= _boundFaces.Count && ps.More(); iterB++, ps.Next())
        {
            Shape bound = _boundFaces.KeyAt(iterB - 1);

            // If bound was already merged - continue
            if (_mergedEdges.Contains(bound))
            {
                continue;
            }

            if (_boundFaces.ValueAt(iterB - 1).Extent() == 0)
            {
                // Merge free edge - only vertices
                Exploration.Vertices(ShapeCasts.Edge(bound), out Vertex no1, out Vertex no2);
                Shape nno1 = no1, nno2 = no2;
                if (_vertexNodeFree.ContainsKey(no1))
                {
                    nno1 = _vertexNodeFree.GetValue(no1);
                }
                if (_vertexNodeFree.ContainsKey(no2))
                {
                    nno2 = _vertexNodeFree.GetValue(no2);
                }
                if (!no1.IsSame(nno1))
                {
                    nno1.Orient(no1.Orientation);
                    _reShape.Replace(no1, nno1);
                }
                if (!no2.IsSame(nno2))
                {
                    nno2.Orient(no2.Orientation);
                    _reShape.Replace(no2, nno2);
                }
                _mergedEdges.Add(bound);
                continue;
            }

            // Check for previous splitting, build replacing wire
            var boundWire = new Wire();
            bool isPrevSplit = false;
            bool hasCuttingSections = _boundSections.ContainsKey(bound);
            if (hasCuttingSections)
            {
                b.MakeWire(ref boundWire);
                boundWire.Orient(bound.Orientation);
                // Iterate on cutting sections
                var its = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(bound));
                for (; its.More(); its.Next())
                {
                    Shape section = its.Value();
                    b.Add(boundWire, section);
                    if (_mergedEdges.Contains(section))
                    {
                        isPrevSplit = true;
                    }
                }
            }

            // Merge with bound
            var mergedWithBound = new IndexedMap<Shape, Shape>(ShapeHasher.Instance);
            if (!isPrevSplit)
            {
                // Obtain sequence of edges merged with bound
                var seqMergedWithBound = new NCollection_Sequence<Shape>();
                var seqMergedWithBoundOri = new NCollection_Sequence<bool>();
                if (MergedNearestEdges(bound, seqMergedWithBound, seqMergedWithBoundOri))
                {
                    // Store bound in the map
                    mergedWithBound.Add(bound, bound);
                    // Iterate on edges merged with bound
                    int ii = 1;
                    while (ii <= seqMergedWithBound.Length())
                    {
                        Shape iedge = seqMergedWithBound.Value(ii);
                        // Remove edge if recorded as merged
                        bool isRejected = (_mergedEdges.Contains(iedge) || mergedWithBound.ContainsKey(iedge));
                        if (!isRejected)
                        {
                            if (_boundSections.ContainsKey(iedge))
                            {
                                // Edge is split - check sections
                                var lit = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(iedge));
                                for (; lit.More() && !isRejected; lit.Next())
                                {
                                    Shape sec = lit.Value();
                                    // Remove edge (bound) if at least one of its sections already merged
                                    isRejected = (_mergedEdges.Contains(sec) || mergedWithBound.ContainsKey(sec));
                                }
                            }
                            if (!isRejected)
                            {
                                if (_sectionBound.ContainsKey(iedge))
                                {
                                    // Edge is a section - check bound
                                    Shape bnd = _sectionBound.GetValue(iedge);
                                    // Remove edge (section) if its bound already merged
                                    isRejected = (_mergedEdges.Contains(bnd) || mergedWithBound.ContainsKey(bnd));
                                }
                            }
                        }
                        // To the next merged edge
                        if (isRejected)
                        {
                            // Remove rejected edge
                            seqMergedWithBound.Remove(ii);
                            seqMergedWithBoundOri.Remove(ii);
                        }
                        else
                        {
                            // Process accepted edge
                            mergedWithBound.Add(iedge, iedge);
                            ii++;
                        }
                    }
                    int nbMerged = seqMergedWithBound.Length();
                    if (nbMerged != 0)
                    {
                        // Create same parameter edge
                        var actuallyMerged = new HashSet<Shape>(ShapeHasher.Instance);
                        Edge mergedEdge = SameParameterEdge(bound,
                                                                   seqMergedWithBound,
                                                                   seqMergedWithBoundOri,
                                                                   actuallyMerged,
                                                                   _reShape);
                        bool isForward = false;
                        if (!mergedEdge.IsNull)
                        {
                            isForward = (mergedEdge.Orientation == Forward);
                        }
                        // Process actually merged edges
                        int nbActuallyMerged = 0;
                        for (ii = 1; ii <= nbMerged; ii++)
                        {
                            Shape iedge = seqMergedWithBound[ii];
                            if (actuallyMerged.Contains(iedge))
                            {
                                nbActuallyMerged++;
                                // Record merged edge in the map
                                Orientation orient = iedge.Orientation;
                                if (!isForward)
                                {
                                    orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                                }
                                if (!seqMergedWithBoundOri[ii])
                                {
                                    orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                                }
                                mergedWithBound.GetValueRef(iedge) = mergedEdge.Oriented(orient);
                            }
                            else
                            {
                                mergedWithBound.Remove(iedge);
                            }
                        }
                        if (nbActuallyMerged != 0)
                        {
                            // Record merged bound in the map
                            Orientation orient = bound.Orientation;
                            if (!isForward)
                            {
                                orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                            }
                            mergedWithBound.GetValueRef(bound) = mergedEdge.Oriented(orient);
                        }
                        nbMerged = nbActuallyMerged;
                    }
                    // Remove bound from the map if not finally merged
                    if (nbMerged == 0)
                    {
                        mergedWithBound.Remove(bound);
                    }
                }
            }
            bool isMerged = mergedWithBound.Count != 0;

            // Merge with cutting sections
            var sectionsReShape = new ReShape();
            var mergedWithSections = new IndexedMap<Shape, Shape>(ShapeHasher.Instance);
            if (hasCuttingSections)
            {
                // Iterate on cutting sections
                var its = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(bound));
                for (; its.More(); its.Next())
                {
                    // Retrieve cutting section
                    Shape section = its.Value();
                    // Skip section if already merged
                    if (_mergedEdges.Contains(section))
                    {
                        continue;
                    }
                    // Merge cutting section
                    var seqMergedWithSection = new NCollection_Sequence<Shape>();
                    var seqMergedWithSectionOri = new NCollection_Sequence<bool>();
                    if (MergedNearestEdges(section, seqMergedWithSection, seqMergedWithSectionOri))
                    {
                        // Store section in the map
                        mergedWithSections.Add(section, section);
                        // Iterate on edges merged with section
                        int ii = 1;
                        while (ii <= seqMergedWithSection.Length())
                        {
                            Shape iedge = seqMergedWithSection.Value(ii);
                            // Remove edge if recorded as merged
                            bool isRejected = (_mergedEdges.Contains(iedge) || mergedWithSections.ContainsKey(iedge));
                            if (!isRejected)
                            {
                                if (_boundSections.ContainsKey(iedge))
                                {
                                    // Edge is split - check sections
                                    var lit = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(iedge));
                                    for (; lit.More() && !isRejected; lit.Next())
                                    {
                                        Shape sec = lit.Value();
                                        // Remove edge (bound) if at least one of its sections already merged
                                        isRejected = (_mergedEdges.Contains(sec) || mergedWithSections.ContainsKey(sec));
                                    }
                                }
                                if (!isRejected)
                                {
                                    if (_sectionBound.ContainsKey(iedge))
                                    {
                                        // Edge is a section - check bound
                                        Shape bnd = _sectionBound.GetValue(iedge);
                                        // Remove edge (section) if its bound already merged
                                        isRejected = (_mergedEdges.Contains(bnd) || mergedWithSections.ContainsKey(bnd));
                                    }
                                }
                            }
                            // To the next merged edge
                            if (isRejected)
                            {
                                // Remove rejected edge
                                seqMergedWithSection.Remove(ii);
                                seqMergedWithSectionOri.Remove(ii);
                            }
                            else
                            {
                                // Process accepted edge
                                mergedWithSections.Add(iedge, iedge);
                                ii++;
                            }
                        }
                        int nbMerged = seqMergedWithSection.Length();
                        if (nbMerged != 0)
                        {
                            // Create same parameter edge
                            var actuallyMerged = new HashSet<Shape>(ShapeHasher.Instance);
                            Edge mergedEdge = SameParameterEdge(section,
                                                                       seqMergedWithSection,
                                                                       seqMergedWithSectionOri,
                                                                       actuallyMerged,
                                                                       sectionsReShape);
                            bool isForward = false;
                            if (!mergedEdge.IsNull)
                            {
                                isForward = (mergedEdge.Orientation == Forward);
                            }
                            // Process actually merged edges
                            int nbActuallyMerged = 0;
                            for (ii = 1; ii <= nbMerged; ii++)
                            {
                                Shape iedge = seqMergedWithSection[ii];
                                if (actuallyMerged.Contains(iedge))
                                {
                                    nbActuallyMerged++;
                                    // Record merged edge in the map
                                    Orientation orient = iedge.Orientation;
                                    if (!isForward)
                                    {
                                        orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                                    }
                                    if (!seqMergedWithSectionOri[ii])
                                    {
                                        orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                                    }
                                    Shape oedge = mergedEdge.Oriented(orient);
                                    mergedWithSections.GetValueRef(iedge) = oedge;
                                    ReplaceEdge(_reShape.InternalApply(iedge), oedge, sectionsReShape);
                                }
                                else
                                {
                                    mergedWithSections.Remove(iedge);
                                }
                            }
                            if (nbActuallyMerged != 0)
                            {
                                // Record merged section in the map
                                Orientation orient = section.Orientation;
                                if (!isForward)
                                {
                                    orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                                }
                                Shape oedge = mergedEdge.Oriented(orient);
                                mergedWithSections.GetValueRef(section) = oedge;
                                ReplaceEdge(_reShape.InternalApply(section), oedge, sectionsReShape);
                            }
                            nbMerged = nbActuallyMerged;
                        }
                        // Remove section from the map if not finally merged
                        if (nbMerged == 0)
                        {
                            mergedWithSections.Remove(section);
                        }
                    }
                    else if (isMerged)
                    {
                        // Reject merging of sections
                        mergedWithSections.Clear();
                        break;
                    }
                }
            }
            bool isMergedSplit = mergedWithSections.Count != 0;

            if (!isMerged && !isMergedSplit)
            {
                // Nothing was merged in this iteration
                if (isPrevSplit)
                {
                    // Replace previously split bound
                    _reShape.Replace(_reShape.InternalApply(bound), _reShape.InternalApply(boundWire));
                }
                continue;
            }

            // Set splitting flag
            bool isSplitted = ((!isMerged && isMergedSplit) || isPrevSplit);

            // Choose between bound and sections merging
            if (isMerged && isMergedSplit && !isPrevSplit)
            {
                // Fill map of merged cutting sections
                var mapSplitEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
                for (int itm = 0; itm < mergedWithSections.Count; itm++)
                {
                    Shape edge = mergedWithSections.KeyAt(itm);
                    mapSplitEdges.Add(edge);
                }
                // Iterate on edges merged with bound
                for (int itm1 = 0; itm1 < mergedWithBound.Count; itm1++)
                {
                    // Retrieve edge merged with bound
                    Shape edge = mergedWithBound.KeyAt(itm1);
                    // Remove edge from the map
                    if (mapSplitEdges.Contains(edge))
                    {
                        mapSplitEdges.Remove(edge);
                    }
                    if (_boundSections.ContainsKey(edge))
                    {
                        // Edge has cutting sections
                        var its = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(edge));
                        for (; its.More(); its.Next())
                        {
                            Shape sec = its.Value();
                            // Remove section from the map
                            if (mapSplitEdges.Contains(sec))
                            {
                                mapSplitEdges.Remove(sec);
                            }
                        }
                    }
                }
                // Calculate section merging tolerance
                double minSplitTol = RealLast();
                for (int ii = 0; ii < mapSplitEdges.Count; ii++)
                {
                    Edge edge = ShapeCasts.Edge(mergedWithSections.GetValue(mapSplitEdges[ii]));
                    minSplitTol = Min(minSplitTol, Tool.Tolerance(edge));
                }
                // Calculate bound merging tolerance
                Edge boundEdge = ShapeCasts.Edge(mergedWithBound.GetValue(bound));
                double boundEdgeTol = Tool.Tolerance(boundEdge);
                isSplitted = ((minSplitTol < boundEdgeTol + MinTolerance()) || _nonmanifold);
                isSplitted = (mapSplitEdges.Count != 0 && isSplitted);
            }

            if (isSplitted)
            {
                // Merging of cutting sections
                // myMergedEdges.Add(bound);
                _reShape.Replace(_reShape.InternalApply(bound), _reShape.InternalApply(boundWire));
                for (int itm = 0; itm < mergedWithSections.Count; itm++)
                {
                    Shape oldedge = mergedWithSections.KeyAt(itm);
                    Shape newedge = sectionsReShape.InternalApply(mergedWithSections.ValueAt(itm));
                    ReplaceEdge(_reShape.InternalApply(oldedge), newedge, _reShape);
                    _mergedEdges.Add(oldedge);
                    if (_boundSections.ContainsKey(oldedge))
                    {
                        _boundSections.Remove(oldedge);
                    }
                }
            }
            else
            {
                // Merging of initial bound
                // myMergedEdges.Add(bound);
                for (int itm = 0; itm < mergedWithBound.Count; itm++)
                {
                    Shape oldedge = mergedWithBound.KeyAt(itm);
                    Shape newedge = mergedWithBound.ValueAt(itm);
                    ReplaceEdge(_reShape.InternalApply(oldedge), newedge, _reShape);
                    _mergedEdges.Add(oldedge);
                    if (_boundSections.ContainsKey(oldedge))
                    {
                        _boundSections.Remove(oldedge);
                    }
                }
                if (_boundSections.ContainsKey(bound))
                {
                    _boundSections.Remove(bound);
                }
                if (!_mergedEdges.Contains(bound))
                {
                    _mergedEdges.Add(bound);
                }
            }
        }

        _nbVertices = _vertexNode.Count + _vertexNodeFree.Count;
        _nodeSections.Clear();
        _vertexNode.Clear();
        _vertexNodeFree.Clear();
        _cuttingNode.Clear();
    }

    //=================================================================================================

    /// <summary>Finds the edges to merge with the edge (the sections between the same nodes), with their orientations.</summary>
    protected bool MergedNearestEdges(in Shape edge,
                                      NCollection_Sequence<Shape> seqMergedEdge,
                                      NCollection_Sequence<bool> seqMergedOri)
    {
        // Retrieve edge nodes
        Exploration.Vertices(ShapeCasts.Edge(edge), out Vertex no1, out Vertex no2);
        Shape nno1 = no1, nno2 = no2;
        bool isNode1 = _vertexNode.ContainsKey(no1);
        bool isNode2 = _vertexNode.ContainsKey(no2);
        if (isNode1)
        {
            nno1 = _vertexNode.GetValue(no1);
        }
        if (isNode2)
        {
            nno2 = _vertexNode.GetValue(no2);
        }

        // Fill map of nodes connected to the node #1
        var mapVert1 = new IndexedSet<Shape>(ShapeHasher.Instance);
        mapVert1.Add(nno1);
        if (_cuttingNode.ContainsKey(nno1))
        {
            var ilv = new NCollection_List<Shape>.Iterator(_cuttingNode.GetValue(nno1));
            for (; ilv.More(); ilv.Next())
            {
                Shape v1 = ilv.Value();
                mapVert1.Add(v1);
                if (!isNode1 && _cuttingNode.ContainsKey(v1))
                {
                    var ilvn = new NCollection_List<Shape>.Iterator(_cuttingNode.GetValue(v1));
                    for (; ilvn.More(); ilvn.Next())
                    {
                        Shape vn = ilvn.Value();
                        mapVert1.Add(vn);
                    }
                }
            }
        }

        // Fill map of nodes connected to the node #2
        var mapVert2 = new HashSet<Shape>(ShapeHasher.Instance);
        mapVert2.Add(nno2);
        if (_cuttingNode.ContainsKey(nno2))
        {
            var ilv = new NCollection_List<Shape>.Iterator(_cuttingNode.GetValue(nno2));
            for (; ilv.More(); ilv.Next())
            {
                Shape v1 = ilv.Value();
                mapVert2.Add(v1);
                if (!isNode2 && _cuttingNode.ContainsKey(v1))
                {
                    var ilvn = new NCollection_List<Shape>.Iterator(_cuttingNode.GetValue(v1));
                    for (; ilvn.More(); ilvn.Next())
                    {
                        Shape vn = ilvn.Value();
                        mapVert2.Add(vn);
                    }
                }
            }
        }

        // Find all possible contiguous edges
        var seqEdges = new NCollection_Sequence<Shape>();
        seqEdges.Append(edge);
        var mapEdges = new HashSet<Shape>(ShapeHasher.Instance);
        mapEdges.Add(edge);
        for (int i = 0; i < mapVert1.Count; i++)
        {
            Shape node1 = mapVert1[i];
            if (!_nodeSections.ContainsKey(node1))
            {
                continue;
            }
            var ilsec = new NCollection_List<Shape>.Iterator(_nodeSections.GetValue(node1));
            for (; ilsec.More(); ilsec.Next())
            {
                Shape sec = ilsec.Value();
                if (sec.IsSame(edge))
                {
                    continue;
                }
                // Retrieve section nodes
                Exploration.Vertices(ShapeCasts.Edge(sec), out Vertex vs1, out Vertex vs2);
                Shape vs1n = vs1, vs2n = vs2;
                if (_vertexNode.ContainsKey(vs1))
                {
                    vs1n = _vertexNode.GetValue(vs1);
                }
                if (_vertexNode.ContainsKey(vs2))
                {
                    vs2n = _vertexNode.GetValue(vs2);
                }
                if ((mapVert1.Contains(vs1n) && mapVert2.Contains(vs2n))
                    || (mapVert1.Contains(vs2n) && mapVert2.Contains(vs1n)))
                {
                    if (mapEdges.Add(sec))
                    {
                        // Check for rejected cutting
                        bool isRejected = _mergedEdges.Contains(sec);
                        if (!isRejected && _boundSections.ContainsKey(sec))
                        {
                            var its = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(sec));
                            for (; its.More() && !isRejected; its.Next())
                            {
                                Shape section = its.Value();

                                if (_mergedEdges.Contains(section))
                                {
                                    isRejected = true;
                                }
                            }
                        }
                        if (!isRejected && _sectionBound.ContainsKey(sec))
                        {
                            Shape bnd = _sectionBound.GetValue(sec);
                            isRejected = (!_boundSections.ContainsKey(bnd) || _mergedEdges.Contains(bnd));
                        }

                        if (!isRejected)
                        {
                            seqEdges.Append(sec);
                        }
                    }
                }
            }
        }

        mapEdges.Clear();

        bool success = false;

        int nbSection = seqEdges.Length();
        if (nbSection > 1)
        {
            // Find the longest edge CCI60011
            int i, indRef = 1;
            if (_nonmanifold)
            {
                double lenRef = 0.0;
                for (i = 1; i <= nbSection; i++)
                {
                    Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(seqEdges[i]), out double f, out double l);
                    var cAdapt = new CurveAdaptor(c3d!);
                    double len = AbscissaPoint.Length(cAdapt, f, l);
                    if (len > lenRef)
                    {
                        indRef = i;
                        lenRef = len;
                    }
                }
                if (indRef != 1)
                {
                    Shape longEdge = seqEdges[indRef];
                    seqEdges[indRef] = seqEdges[1];
                    seqEdges[1] = longEdge;
                }
            }

            // Find merging candidates
            var seqForward = new NCollection_Sequence<bool>();
            var seqCandidates = new NCollection_Sequence<int>();
            var mapReference = new IndexedSet<int>();
            mapReference.Add(indRef); // Add index of reference section
            if (FindCandidates(seqEdges, mapReference, seqCandidates, seqForward))
            {
                int nbCandidates = seqCandidates.Length();
                // Record candidate sections
                for (i = 1; i <= nbCandidates; i++)
                {
                    // Retrieve merged edge
                    Shape iedge = seqEdges[seqCandidates[i]];
                    bool ori = seqForward[i];
                    seqMergedEdge.Append(iedge);
                    seqMergedOri.Append(ori);
                    if (!_nonmanifold)
                    {
                        break;
                    }
                }
                success = (nbCandidates != 0);
            }
        }

        return success;
    }

    //=======================================================================
    // function : Cutting
    // purpose  : Modifies :
    //                     myBoundSections
    //                     myNodeSections
    //                     myCuttingNode
    //=======================================================================

    /// <summary>Cuts the bounds at the projections of the nodes near them.</summary>
    protected void Cutting(ProgressRange progress = default)
    {
        int i, nbVertices = _vertexNode.Count;
        if (nbVertices == 0)
        {
            return;
        }
        // Create a box tree with vertices
        double eps = _tolerance * 0.5;
        var tree = new SpatialTree<int, Box>();
        var selector = new BndBoxTreeSelector();
        using (var treeFiller = new SpatialTreeFiller<int, Box>(tree))
        {
            for (i = 1; i <= nbVertices; i++)
            {
                Point3D pt = Tool.Pnt(ShapeCasts.Vertex(_vertexNode.KeyAt(i - 1)));
                var box = new Box();
                box.Set(pt);
                box.Enlarge(eps);
                treeFiller.Add(i, box);
            }
            treeFiller.Fill();
        }

        Curve3D? c3d;
        Placement loc;
        double first, last;
        // Iterate on all boundaries
        int nbBounds = _boundFaces.Count;
        using var ps = new ProgressScope(progress, "Cutting bounds", nbBounds);
        for (int iterB = 1; iterB <= _boundFaces.Count && ps.More(); iterB++, ps.Next())
        {
            Edge bound = ShapeCasts.Edge(_boundFaces.KeyAt(iterB - 1));
            // Do not cut floating edges
            if (_boundFaces.ValueAt(iterB - 1).Extent() == 0)
            {
                continue;
            }
            // Obtain bound curve
            c3d = Tool.Curve(bound, out loc, out first, out last);
            if (c3d is null)
            {
                continue;
            }
            if (!loc.IsIdentity)
            {
                c3d = (Curve3D)c3d.Copy();
                c3d.Transform(loc.Transformation);
            }
            // Create cutting sections
            var listSections = new NCollection_List<Shape>();
            { // szv: Use brackets to destroy local variables
                // Obtain candidate vertices
                Vertex v1 = new(), v2 = new();
                var candidateVertices = new IndexedSet<Shape>(ShapeHasher.Instance);
                { // szv: Use brackets to destroy local variables
                    // Create bounding box around curve
                    var globalBox = new Box();
                    var adptC = new CurveAdaptor(c3d, first, last);
                    Add3dCurve.Add(adptC, _tolerance, ref globalBox);
                    // Sort vertices to find candidates
                    selector.SetCurrent(globalBox);
                    tree.Select(selector);
                    // Skip bound if no node is in the boundind box
                    if (selector.ResInd().Extent() == 0)
                    {
                        continue;
                    }
                    // Retrieve bound nodes
                    Exploration.Vertices(bound, out v1, out v2);
                    Shape node1 = _vertexNode.GetValue(v1);
                    Shape node2 = _vertexNode.GetValue(v2);
                    // Fill map of candidate vertices
                    var itl = new NCollection_List<int>.Iterator(selector.ResInd());
                    for (; itl.More(); itl.Next())
                    {
                        int index = itl.Value();
                        Shape node = _vertexNode.ValueAt(index - 1);
                        if (!node.IsSame(node1) && !node.IsSame(node2))
                        {
                            Shape vertex = _vertexNode.KeyAt(index - 1);
                            candidateVertices.Add(vertex);
                        }
                    }
                    selector.ClearResList();
                }
                int nbCandidates = candidateVertices.Count;
                if (nbCandidates == 0)
                {
                    continue;
                }
                // Project vertices on curve
                var arrPara = new NCollection_Array1<double>(1, nbCandidates);
                var arrDist = new NCollection_Array1<double>(1, nbCandidates);
                var arrPnt = new NCollection_Array1<Point3D>(1, nbCandidates);
                var arrProj = new NCollection_Array1<Point3D>(1, nbCandidates);
                for (int j = 1; j <= nbCandidates; j++)
                {
                    arrPnt[j] = Tool.Pnt(ShapeCasts.Vertex(candidateVertices[j - 1]));
                }
                ProjectPointsOnCurve(arrPnt, c3d, first, last, arrDist, arrPara, arrProj, true);
                // Create cutting nodes
                var seqNode = new NCollection_Sequence<Shape>();
                var seqPara = new NCollection_Sequence<double>();
                CreateCuttingNodes(candidateVertices,
                                   bound,
                                   v1,
                                   v2,
                                   arrDist,
                                   arrPara,
                                   arrProj,
                                   seqNode,
                                   seqPara);
                if (seqPara.Length() == 0)
                {
                    continue;
                }
                // Create cutting sections
                CreateSections(bound, seqNode, seqPara, listSections);
            }
            if (listSections.Extent() > 1)
            {
                // modification of maps:
                //                     myBoundSections
                var its = new NCollection_List<Shape>.Iterator(listSections);
                for (; its.More(); its.Next())
                {
                    Shape section = its.Value();
                    // Iterate on section vertices
                    for (var itv = new Iterator(section); itv.More(); itv.Next())
                    {
                        Shape vertex = itv.Value();
                        // Convert vertex to node
                        if (_vertexNode.ContainsKey(vertex))
                        {
                            vertex = ShapeCasts.Vertex(_vertexNode.GetValue(vertex));
                        }
                        // Update node sections
                        if (_nodeSections.ContainsKey(vertex))
                        {
                            _nodeSections.GetValue(vertex).Append(section);
                        }
                        else
                        {
                            var lsec = new NCollection_List<Shape>();
                            lsec.Append(section);
                            _nodeSections[vertex] = lsec;
                        }
                    }
                    // Store bound for section
                    _sectionBound[section] = bound;
                }
                // Store split bound
                _boundSections[bound] = listSections;
            }
        }
    }

    //=================================================================================================

    private static void GetSeqEdges(in Shape edge,
                                    NCollection_Sequence<Shape> seqEdges,
                                    Dictionary<Shape, NCollection_List<Shape>> vertEdge)
    {
        int numV = 0;
        for (var iv = new Iterator(edge, false); iv.More(); iv.Next())
        {
            Vertex v1 = ShapeCasts.Vertex(iv.Value());
            numV++;
            if (vertEdge.ContainsKey(v1))
            {
                NCollection_List<Shape> listEdges = vertEdge.GetValue(v1);
                for (var lIt = new NCollection_List<Shape>.Iterator(listEdges); lIt.More(); lIt.Next())
                {
                    Shape edge1 = lIt.Value();
                    if (edge1.IsSame(edge))
                    {
                        continue;
                    }
                    bool isContained = false;
                    int i, index = 1;
                    for (i = 1; i <= seqEdges.Length() && !isContained; i++)
                    {
                        isContained = seqEdges.Value(i).IsSame(edge1);
                        if (!isContained && seqEdges.Value(i).IsSame(edge))
                        {
                            index = i;
                        }
                    }
                    if (!isContained)
                    {
                        if (numV == 1)
                        {
                            seqEdges.InsertBefore(index, edge1);
                        }
                        else
                        {
                            seqEdges.InsertAfter(index, edge1);
                        }
                        GetSeqEdges(edge1, seqEdges, vertEdge);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Chains the free edges into wires. Called from EdgeProcessing only.</summary>
    protected virtual void GetFreeWires(IndexedSet<Shape> mapFreeEdges,
                                        NCollection_Sequence<Shape> seqWires)
    {
        var vertEdge = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var seqFreeEdges = new NCollection_Sequence<Shape>();
        for (int k = 0; k < mapFreeEdges.Count; k++)
        {
            Shape edge = mapFreeEdges[k];
            seqFreeEdges.Append(edge);
            for (var iv = new Iterator(edge, false); iv.More(); iv.Next())
            {
                Vertex v1 = ShapeCasts.Vertex(iv.Value());
                if (vertEdge.ContainsKey(v1))
                {
                    vertEdge.GetValue(v1).Append(edge);
                }
                else
                {
                    var ls = new NCollection_List<Shape>();
                    ls.Append(edge);
                    vertEdge[v1] = ls;
                }
            }
        }
        var b = new BrepBuilder();
        int i, j;
        for (i = 1; i <= seqFreeEdges.Length(); i++)
        {
            var seqEdges = new NCollection_Sequence<Shape>();
            Shape edge = seqFreeEdges.Value(i);
            if (!mapFreeEdges.Contains(edge))
            {
                continue;
            }
            seqEdges.Append(edge);
            GetSeqEdges(edge, seqEdges, vertEdge);
            var wire = new Wire();
            b.MakeWire(ref wire);
            for (j = 1; j <= seqEdges.Length(); j++)
            {
                b.Add(wire, seqEdges.Value(j));
                mapFreeEdges.Remove(seqEdges.Value(j));
            }
            seqWires.Append(wire);
            if (mapFreeEdges.Count == 0)
            {
                break;
            }
        }
    }

    //=================================================================================================

    private static bool IsDegeneratedWire(in Shape wire)
    {
        if (wire.Type != ShapeType.Wire)
        {
            return false;
        }
        // Get maximal vertices tolerance
        Vertex v1 = new(), v2 = new();
        double wireLength = 0.0;
        int nume = 0;
        int isSmall = 0;
        for (var it = new Iterator(wire, false); it.More(); it.Next())
        {
            nume++;
            Shape edge = it.Value();
            Exploration.Vertices(ShapeCasts.Edge(edge), out Vertex ve1, out Vertex ve2);
            if (nume == 1)
            {
                v1 = ve1;
                v2 = ve2;
            }
            else
            {
                if (ve1.IsSame(v1))
                {
                    v1 = ve2;
                }
                else if (ve1.IsSame(v2))
                {
                    v2 = ve2;
                }
                if (ve2.IsSame(v1))
                {
                    v1 = ve1;
                }
                else if (ve2.IsSame(v2))
                {
                    v2 = ve1;
                }
            }
            Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(it.Value()), out Placement loc, out double first, out double last);
            if (c3d is not null)
            {
                c3d = (Curve3D)c3d.Copy();
                if (!loc.IsIdentity)
                {
                    c3d.Transform(loc.Transformation);
                }
                Point3D pfirst = c3d.Value(first);
                Point3D plast = c3d.Value(last);
                Point3D pmid = c3d.Value((first + last) * 0.5);
                double length = 0;
                if (pfirst.DistanceTo(plast) > pfirst.DistanceTo(pmid))
                {
                    length = pfirst.DistanceTo(plast);
                }
                else
                {
                    var cAdapt = new CurveAdaptor(c3d);
                    length = AbscissaPoint.Length(cAdapt, first, last);
                }
                double tole = Tool.Tolerance(ve1) + Tool.Tolerance(ve2);
                if (length <= tole)
                {
                    isSmall++;
                }
                wireLength += length;
            }
        }
        if (isSmall == nume)
        {
            return true;
        }
        double tol = Tool.Tolerance(v1) + Tool.Tolerance(v2); // Max(BRep_Tool::Tolerance(V1),BRep_Tool::Tolerance(V2));
        return wireLength <= tol;
    }

    //=======================================================================
    // function :  DegeneratedSection
    // purpose  :  internal use
    //            create a new degenerated edge if the section is degenerated
    //=======================================================================

    private static Edge DegeneratedSection(in Shape section, in Shape face)
    {
        // Return if section is already degenerated
        if (Tool.Degenerated(ShapeCasts.Edge(section)))
        {
            return ShapeCasts.Edge(section);
        }

        // Retrieve edge curve
        Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(section), out Placement loc, out double first, out double last);
        if (c3d is null)
        { // gka
            var b = new BrepBuilder();
            Edge edge1 = ShapeCasts.Edge(section);
            b.Degenerated(edge1, true);
            return edge1;
        }
        if (!loc.IsIdentity)
        {
            c3d = (Curve3D)c3d.Copy();
            c3d.Transform(loc.Transformation);
        }

        // Test if the new edge is degenerated
        Exploration.Vertices(ShapeCasts.Edge(section), out Vertex v1, out Vertex v2);

        Point3D p1, p2, p3;
        p1 = Tool.Pnt(v1);
        p3 = Tool.Pnt(v2);
        c3d.D0(0.5 * (first + last), out p2);

        Edge edge;

        // processing
        var builder = new BrepBuilder();
        edge = ShapeCasts.Edge(section);
        edge.EmptyCopy();
        if (v1.IsSame(v2))
        {
            Shape anEdge1 = edge.Oriented(Forward);
            builder.Add(anEdge1, v1.Oriented(Forward));
            builder.Add(anEdge1, v2.Oriented(Reversed));
        }
        else
        {
            var newVertex = new Vertex();
            if (p1.DistanceTo(p3) < Tool.Tolerance(v1))
            {
                newVertex = v1;
            }
            else if (p1.DistanceTo(p3) < Tool.Tolerance(v2))
            {
                newVertex = v2;
            }
            else
            {
                double d1 = Tool.Tolerance(v1) + p2.DistanceTo(p1);
                double d2 = Tool.Tolerance(v2) + p2.DistanceTo(p3);
                double newTolerance = Max(d1, d2);
                builder.MakeVertex(ref newVertex, p2, newTolerance);
            }
            Shape anEdge1 = edge.Oriented(Forward);
            builder.Add(anEdge1, newVertex.Oriented(Forward));
            builder.Add(anEdge1, newVertex.Oriented(Reversed));
        }

        Tool.Range(ShapeCasts.Edge(section), out first, out last);
        Shape anEdge = edge.Oriented(Forward);
        builder.Range(ShapeCasts.Edge(anEdge), first, last);
        builder.Degenerated(edge, true);
        Curve3D? c3dNew = null;
        if (!face.IsNull)
        {
            Curve2D? c2dt =
                Tool.CurveOnSurface(ShapeCasts.Edge(section), ShapeCasts.Face(face), out double af, out double al);
            builder.UpdateEdge(edge, c3dNew, 0);
            Curve2D? c2dn = Tool.CurveOnSurface(edge, ShapeCasts.Face(face), out af, out al);
            if (c2dn is null)
            {
                builder.UpdateEdge(edge, c2dt, ShapeCasts.Face(face), 0);
            }
        }

        return edge;
    }

    //=======================================================================
    // function : EdgeProcessing
    // purpose  : modifies :
    //                       myNbEdges
    //                       myHasMultipleEdge
    //                       myHasFreeBound
    //           . if multiple edge
    //              - length < 100.*myTolerance -> several free edge
    //           . if no multiple edge
    //              - make the contiguous edges sameparameter
    //=======================================================================

    /// <summary>Makes the free wires that are degenerated into degenerated edges.</summary>
    protected void EdgeProcessing(ProgressRange progress = default)
    {
        // constructs sectionEdge
        var mapFreeEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        var edgeFace = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        using var ps = new ProgressScope(progress, "Edge processing", _boundFaces.Count);
        for (int iterB = 1; iterB <= _boundFaces.Count && ps.More(); iterB++, ps.Next())
        {
            Shape bound = _boundFaces.KeyAt(iterB - 1);
            NCollection_List<Shape> listFaces = _boundFaces.ValueAt(iterB - 1);
            if (listFaces.Extent() == 1)
            {
                if (_boundSections.ContainsKey(bound))
                {
                    var liter = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(bound));
                    for (; liter.More(); liter.Next())
                    {
                        if (!_mergedEdges.Contains(liter.Value()))
                        { // myReShape->IsRecorded(liter.Value())) {
                            Shape edge = _reShape.InternalApply(liter.Value());
                            if (!mapFreeEdges.Contains(edge))
                            {
                                Shape face = listFaces.First();
                                edgeFace[edge] = face;
                                mapFreeEdges.Add(edge);
                            }
                        }
                    }
                }
                else
                {
                    if (!_mergedEdges.Contains(bound))
                    {
                        Shape edge = _reShape.InternalApply(bound);
                        if (!mapFreeEdges.Contains(edge))
                        {
                            Shape face = listFaces.First();
                            edgeFace[edge] = face;
                            mapFreeEdges.Add(edge);
                        }
                    }
                }
            }
        }

        if (mapFreeEdges.Count != 0)
        {
            var seqWires = new NCollection_Sequence<Shape>();
            GetFreeWires(mapFreeEdges, seqWires);
            for (int j = 1; j <= seqWires.Length(); j++)
            {
                Wire wire = ShapeCasts.Wire(seqWires.Value(j));
                if (!IsDegeneratedWire(wire))
                {
                    continue;
                }
                for (var ie = new Iterator(wire, false); ie.More(); ie.Next())
                {
                    Shape tmpShape = _reShape.InternalApply(ie.Value()); // for porting
                    Edge edge = ShapeCasts.Edge(tmpShape);
                    var face = new Shape();
                    if (edgeFace.ContainsKey(edge))
                    {
                        face = edgeFace.GetValue(edge);
                    }
                    Shape degedge = DegeneratedSection(edge, face);
                    if (degedge.IsNull)
                    {
                        continue;
                    }
                    if (!degedge.IsSame(edge))
                    {
                        ReplaceEdge(edge, degedge, _reShape);
                    }
                    if (Tool.Degenerated(ShapeCasts.Edge(degedge)))
                    {
                        _degenerated.Add(degedge);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : EdgeRegularity
    // purpose  : update Continuity flag on newly created edges
    //=======================================================================

    /// <summary>Recomputes the regularity of the merged edges.</summary>
    protected void EdgeRegularity(ProgressRange progress = default)
    {
        var mapEF = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_sewedShape, ShapeType.Edge, ShapeType.Face, mapEF);

        using var ps = new ProgressScope(progress, "Encode edge regularity", _mergedEdges.Count);
        // OCCT iterates its map in address order; the port in insertion order (decision 18). Each edge is encoded on
        // its own, so the order doesn't change the result.
        foreach (Shape mergedEdge in _mergedEdges)
        {
            if (!ps.More())
            {
                break;
            }
            Edge edge = ShapeCasts.Edge(_reShape.InternalApply(mergedEdge));
            // encode regularity if and only if edges is shared by two faces
            if (mapEF.TryGetValue(edge, out NCollection_List<Shape> faces) && faces.Extent() == 2)
            {
                BrepLibrary.EncodeRegularity(edge,
                                         ShapeCasts.Face(faces.First()),
                                         ShapeCasts.Face(faces.Last()));
            }
            ps.Next();
        }

        _mergedEdges.Clear(); // OCCT: CreateOutputInformations reads it afterwards, so no section is bound after sewing
    }

    //=================================================================================================

    /// <summary>Builds the sewn shape from the reshaped inputs. Called from Perform only.</summary>
    protected virtual void CreateSewedShape()
    {
        // ---------------------
        // create the new shapes
        // ---------------------
        var quilt = new Quilt();
        bool isLocal = !_shape.IsNull;
        if (isLocal)
        {
            // Local sewing
            Shape ns = _reShape.InternalApply(_shape);
            quilt.Add(ns);
        }
        for (int iter = 0; iter < _oldShapes.Count; iter++)
        {
            Shape sh = _oldShapes.ValueAt(iter);
            if (!sh.IsNull)
            {
                sh = _reShape.InternalApply(sh);
                _oldShapes.ValueAt(iter) = sh;
                if (!isLocal)
                {
                    quilt.Add(sh);
                }
            }
        }
        Shape newShape = quilt.InternalShells();
        int numsh = 0;

        var oldShells = new IndexedSet<Shape>(ShapeHasher.Instance);

        var b = new BrepBuilder();
        var comp = new Compound();
        b.MakeCompound(ref comp);
        for (var expSh = new Iterator(newShape, false); expSh.More(); expSh.Next())
        {
            Shape sh = expSh.Value();
            bool hasEdges = false;
            if (sh.Type == ShapeType.Shell)
            {
                if (_nonmanifold)
                {
                    hasEdges = !oldShells.Contains(sh);
                }
                else
                {
                    var face = new Shape();
                    int numf = 0;
                    for (var expF = new Explorer(sh, ShapeType.Face); expF.More() && (numf < 2); expF.Next())
                    {
                        face = expF.Current();
                        numf++;
                    }
                    if (numf == 1)
                    {
                        b.Add(comp, face);
                    }
                    else if (numf > 1)
                    {
                        b.Add(comp, sh);
                    }
                    if (numf != 0)
                    {
                        numsh++;
                    }
                }
            }
            else if (sh.Type == ShapeType.Face)
            {
                if (_nonmanifold)
                {
                    var ss = new Shell();
                    b.MakeShell(ref ss);
                    b.Add(ss, sh);
                    sh = ss;
                    hasEdges = true;
                }
                else
                {
                    b.Add(comp, sh);
                    numsh++;
                }
            }
            else
            {
                b.Add(comp, sh);
                numsh++;
            }
            if (hasEdges)
            {
                oldShells.Add(sh);
            }
        }
        // Process collected shells
        if (_nonmanifold)
        {
            int nbOldShells = oldShells.Count;
            if (nbOldShells == 1)
            {
                // Single shell - check for single face
                Shape sh = oldShells[0];
                var face = new Shape();
                int numf = 0;
                for (var expF = new Explorer(sh, ShapeType.Face); expF.More() && (numf < 2); expF.Next())
                {
                    face = expF.Current();
                    numf++;
                }
                if (numf == 1)
                {
                    b.Add(comp, face);
                }
                else if (numf > 1)
                {
                    b.Add(comp, sh);
                }
                if (numf != 0)
                {
                    numsh++;
                }
            }
            else if (nbOldShells != 0)
            {
                // Several shells should be merged
                var indexMerged = new HashSet<int>();
                while (indexMerged.Count < nbOldShells)
                {
                    var newShell = new Shell();
                    var newEdges = new HashSet<Shape>(ShapeHasher.Instance);
                    for (int i = 1; i <= nbOldShells; i++)
                    {
                        if (indexMerged.Contains(i))
                        {
                            continue;
                        }
                        Shell shell = ShapeCasts.Shell(oldShells[i - 1]);
                        if (newShell.IsNull)
                        {
                            b.MakeShell(ref newShell);
                            var itSS = new Iterator(shell);
                            for (; itSS.More(); itSS.Next())
                            {
                                b.Add(newShell, itSS.Value());
                            }
                            // Fill map of edges
                            for (var eexp = new Explorer(shell, ShapeType.Edge); eexp.More(); eexp.Next())
                            {
                                Shape edge = eexp.Current();
                                newEdges.Add(edge);
                            }
                            indexMerged.Add(i);
                        }
                        else
                        {
                            bool hasSharedEdge = false;
                            var eexp = new Explorer(shell, ShapeType.Edge);
                            for (; eexp.More() && !hasSharedEdge; eexp.Next())
                            {
                                hasSharedEdge = newEdges.Contains(eexp.Current());
                            }
                            if (hasSharedEdge)
                            {
                                // Add edges to the map
                                for (var eexp1 = new Explorer(shell, ShapeType.Edge); eexp1.More(); eexp1.Next())
                                {
                                    Shape edge = eexp1.Current();
                                    newEdges.Add(edge);
                                }
                                // Add faces to the shell
                                for (var fexp = new Explorer(shell, ShapeType.Face); fexp.More(); fexp.Next())
                                {
                                    Shape face = fexp.Current();
                                    b.Add(newShell, face);
                                }
                                indexMerged.Add(i);
                            }
                        }
                    }
                    // Process new shell
                    var face1 = new Shape();
                    int numf = 0;
                    var expF = new Explorer(newShell, ShapeType.Face);
                    for (; expF.More() && (numf < 2); expF.Next())
                    {
                        face1 = expF.Current();
                        numf++;
                    }
                    if (numf == 1)
                    {
                        b.Add(comp, face1);
                    }
                    else if (numf > 1)
                    {
                        b.Add(comp, newShell);
                    }
                    if (numf != 0)
                    {
                        numsh++;
                    }
                }
            }
        }
        if (numsh == 1)
        {
            // Extract single component
            var it = new Iterator(comp, false);
            _sewedShape = it.Value();
        }
        else
        {
            _sewedShape = comp;
        }
    }

    //=======================================================================
    // function : CreateOutputInformations
    // purpose  : constructs :
    //                       myEdgeSections
    //                       mySectionBound
    //                       myNbFreeEdges
    //                       myNbContigousEdges
    //                       myNbMultipleEdges
    //                       myNbDegenerated
    //=======================================================================

    /// <summary>Fills the maps of free, contiguous, multiple and degenerated edges.</summary>
    protected void CreateOutputInformations()
    {
        // Construct edgeSections
        int i;
        // use index map for regulating free edges
        var edgeSections = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        for (i = 1; i <= _boundFaces.Count; i++)
        {
            Shape bound = _boundFaces.KeyAt(i - 1);
            var lsect = new NCollection_List<Shape>();
            if (_boundSections.ContainsKey(bound))
            {
                lsect.Assign(_boundSections.GetValue(bound));
            }
            var exp = new Explorer(_reShape.InternalApply(bound), ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape sec = bound;
                Shape edge = exp.Current();
                var aI = new NCollection_List<Shape>.Iterator(lsect);
                for (; aI.More(); aI.Next())
                {
                    Shape section = aI.Value();
                    if (edge.IsSame(_reShape.InternalApply(section)))
                    {
                        sec = section;
                        break;
                    }
                }
                if (edgeSections.ContainsKey(edge))
                {
                    edgeSections.GetValue(edge).Append(sec);
                }
                else
                {
                    var listSec = new NCollection_List<Shape>();
                    listSec.Append(sec);
                    edgeSections.Add(edge, listSec);
                }
            }
        }

        // Fill maps of Free, Contiguous and Multiple edges
        for (int anIter = 0; anIter < edgeSections.Count; anIter++)
        {
            Shape edge = edgeSections.KeyAt(anIter);
            NCollection_List<Shape> listSection = edgeSections.ValueAt(anIter);
            if (listSection.Extent() == 1)
            {
                if (Tool.Degenerated(ShapeCasts.Edge(edge)))
                {
                    _degenerated.Add(edge);
                }
                else
                {
                    _freeEdges.Add(edge);
                }
            }
            else if (listSection.Extent() == 2)
            {
                _contigousEdges.Add(edge, listSection);
            }
            else
            {
                _multipleEdges.Add(edge);
            }
        }

        // constructs myContigSectBound
        for (i = 1; i <= _boundFaces.Count; i++)
        {
            Shape bound = _boundFaces.KeyAt(i - 1);
            if (_boundSections.ContainsKey(bound))
            {
                var iter = new NCollection_List<Shape>.Iterator(_boundSections.GetValue(bound));
                for (; iter.More(); iter.Next())
                {
                    Shape section = iter.Value();
                    if (!_mergedEdges.Contains(section)) // OCCT: empty after EdgeRegularity when sewing
                    {
                        continue;
                    }
                    // if (!myReShape->IsRecorded(section)) continue; // section is free
                    Shape nedge = _reShape.InternalApply(section);
                    if (nedge.IsNull)
                    {
                        continue; // szv debug
                    }
                    if (!bound.IsSame(section))
                    {
                        if (_contigousEdges.ContainsKey(nedge))
                        {
                            _contigSecBound[section] = bound;
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Projects the points on the curve: distances (-1 where a point isn't projected), parameters and projections.
    /// Called from Cutting only.
    /// </summary>
    protected void ProjectPointsOnCurve(NCollection_Array1<Point3D> arrPnt,
                                        Curve3D? c3d,
                                        double first,
                                        double last,
                                        NCollection_Array1<double> arrDist,
                                        NCollection_Array1<double> arrPara,
                                        NCollection_Array1<Point3D> arrProj,
                                        bool isConsiderEnds)
    {
        arrDist.Init(-1.0);

        var gac = new CurveAdaptor(c3d!);
        var locProj = new CurvePointExtrema();
        locProj.Initialize(gac, first, last);
        Point3D pfirst = gac.Value(first), plast = gac.Value(last);
        int find = 1;              //(isConsiderEnds ? 1 : 2);
        int lind = arrPnt.Length(); //(isConsiderEnds ? arrPnt.Length() : arrPnt.Length() -1);

        for (int i1 = find; i1 <= lind; i1++)
        {
            Point3D pt = arrPnt[i1];
            double worktol = _tolerance;
            double distF2 = pfirst.SquareDistanceTo(pt);
            double distL2 = plast.SquareDistanceTo(pt);
            bool isProjected = false;
            try
            {
                // Project current point on curve
                locProj.Perform(pt);
                if (locProj.IsDone && locProj.ExtCount > 0)
                {
                    double dist2Min = (isConsiderEnds || i1 == find || i1 == lind ? Min(distF2, distL2)
                                                                                 : global::Kerf.Engine.Foundation.Precision.Infinite());
                    int ind, indMin = 0;
                    for (ind = 1; ind <= locProj.ExtCount; ind++)
                    {
                        double dProj2 = locProj.SquareDistance(ind);
                        if (dProj2 < dist2Min)
                        {
                            indMin = ind;
                            dist2Min = dProj2;
                        }
                    }
                    if (indMin != 0)
                    {
                        isProjected = true;
                        POnCurv pOnC = locProj.Point(indMin);
                        double paramProj = pOnC.Parameter();
                        Point3D ptProj = gac.Value(paramProj);
                        double distProj2 = ptProj.SquareDistanceTo(pt);
                        if (!locProj.IsMin(indMin))
                        {
                            if (Min(distF2, distL2) < dist2Min)
                            {
                                if (distF2 < distL2)
                                {
                                    paramProj = first;
                                    distProj2 = distF2;
                                    ptProj = pfirst;
                                }
                                else
                                {
                                    paramProj = last;
                                    distProj2 = distL2;
                                    ptProj = plast;
                                }
                            }
                        }
                        if (distProj2 < worktol * worktol || !isConsiderEnds)
                        {
                            arrDist[i1] = Math.Sqrt(distProj2);
                            arrPara[i1] = paramProj;
                            arrProj[i1] = ptProj;
                        }
                    }
                }
            }
            catch (FailureException)
            {
                worktol = MinTolerance();
            }
            if (!isProjected && isConsiderEnds)
            {
                if (Min(distF2, distL2) < worktol * worktol)
                {
                    if (distF2 < distL2)
                    {
                        arrDist[i1] = Math.Sqrt(distF2);
                        arrPara[i1] = first;
                        arrProj[i1] = pfirst;
                    }
                    else
                    {
                        arrDist[i1] = Math.Sqrt(distL2);
                        arrPara[i1] = last;
                        arrProj[i1] = plast;
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Creates the cutting vertices at the projections of the nodes on a bound. Called from Cutting only.</summary>
    protected virtual void CreateCuttingNodes(IndexedSet<Shape> mapVert,
                                              in Shape bound,
                                              in Shape vfirst,
                                              in Shape vlast,
                                              NCollection_Array1<double> arrDist,
                                              NCollection_Array1<double> arrPara,
                                              NCollection_Array1<Point3D> arrPnt,
                                              NCollection_Sequence<Shape> seqVert,
                                              NCollection_Sequence<double> seqPara)
    {
        int i, j, nbProj = mapVert.Count;

        // Reorder projections by distance
        var seqOrderedIndex = new NCollection_Sequence<int>();
        { // szv: Use brackets to destroy local variables
            var seqOrderedDistance = new NCollection_Sequence<double>();
            for (i = 1; i <= nbProj; i++)
            {
                double distProj = arrDist[i];
                if (distProj < 0.0)
                {
                    continue; // Skip vertex if not projected
                }
                bool isInserted = false;
                for (j = 1; j <= seqOrderedIndex.Length() && !isInserted; j++)
                {
                    isInserted = (distProj < seqOrderedDistance[j]);
                    if (isInserted)
                    {
                        seqOrderedIndex.InsertBefore(j, i);
                        seqOrderedDistance.InsertBefore(j, distProj);
                    }
                }
                if (!isInserted)
                {
                    seqOrderedIndex.Append(i);
                    seqOrderedDistance.Append(distProj);
                }
            }
        }
        nbProj = seqOrderedIndex.Length();
        if (nbProj == 0)
        {
            return;
        }

        var builder = new BrepBuilder();

        // Insert two initial vertices (to be removed later)
        var seqDist = new NCollection_Sequence<double>();
        var seqPnt = new NCollection_Sequence<Point3D>();
        { // szv: Use brackets to destroy local variables
            // Retrieve bound curve
            Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(bound), out Placement loc, out double first, out double last);
            if (!loc.IsIdentity)
            {
                c3d = (Curve3D)c3d!.Copy();
                c3d.Transform(loc.Transformation);
            }
            var gac = new CurveAdaptor(c3d!);
            seqVert.Prepend(vfirst);
            seqVert.Append(vlast);
            seqPara.Prepend(first);
            seqPara.Append(last);
            seqDist.Prepend(-1.0);
            seqDist.Append(-1.0);
            seqPnt.Prepend(gac.Value(first));
            seqPnt.Append(gac.Value(last));
        }

        var nodeCuttingVertex = new IndexedMap<Shape, Shape>(ShapeHasher.Instance);
        for (i = 1; i <= nbProj; i++)
        {
            int index = seqOrderedIndex[i];
            double disProj = arrDist[index];
            Point3D pntProj = arrPnt[index];

            // Skip node if already bound to cutting vertex
            Shape node = _vertexNode.GetValue(mapVert[index - 1]);
            if (nodeCuttingVertex.ContainsKey(node))
            {
                continue;
            }

            // Find the closest vertex
            int indexMin = 1;
            double dist, distMin = pntProj.DistanceTo(seqPnt[1]);
            for (j = 2; j <= seqPnt.Length(); j++)
            {
                dist = pntProj.DistanceTo(seqPnt[j]);
                if (dist < distMin)
                {
                    distMin = dist;
                    indexMin = j;
                }
            }

            // Check if current point is close to one of the existent
            if (distMin <= Max(disProj * 0.1, MinTolerance()))
            {
                // Check distance if close
                double jdist = seqDist.Value(indexMin);
                if (jdist < 0.0)
                {
                    // Bind new cutting node (end vertex only)
                    seqDist.SetValue(indexMin, disProj);
                    Shape cvertex = seqVert.Value(indexMin);
                    nodeCuttingVertex.Add(node, cvertex);
                }
                else
                {
                    // Bind secondary cutting nodes
                    nodeCuttingVertex.Add(node, new Vertex());
                }
            }
            else
            {
                // Build new cutting vertex
                var cvertex = new Vertex();
                builder.MakeVertex(ref cvertex, pntProj, global::Kerf.Engine.Foundation.Precision.Confusion());
                // Bind new cutting vertex
                nodeCuttingVertex.Add(node, cvertex);
                // Insert cutting vertex in the sequences
                double parProj = arrPara[index];
                for (j = 2; j <= seqPara.Length(); j++)
                {
                    if (parProj <= seqPara.Value(j))
                    {
                        seqVert.InsertBefore(j, cvertex);
                        seqPara.InsertBefore(j, parProj);
                        seqDist.InsertBefore(j, disProj);
                        seqPnt.InsertBefore(j, pntProj);
                        break;
                    }
                }
            }
        }

        // filling map for cutting nodes
        for (int mIt = 0; mIt < nodeCuttingVertex.Count; mIt++)
        {
            Shape cnode = nodeCuttingVertex.ValueAt(mIt);
            // Skip secondary nodes
            if (cnode.IsNull)
            {
                continue;
            }
            // Obtain vertex node
            Shape node = nodeCuttingVertex.KeyAt(mIt);
            if (_vertexNode.ContainsKey(cnode))
            {
                // This is an end vertex
                cnode = _vertexNode.GetValue(cnode);
            }
            else
            {
                // Create link: cutting vertex -> node
                var ls1 = new NCollection_List<Shape>();
                ls1.Append(node);
                _cuttingNode[cnode] = ls1;
            }
            // Create link: node -> cutting vertex
            if (_cuttingNode.ContainsKey(node))
            {
                _cuttingNode.GetValue(node).Append(cnode);
            }
            else
            {
                var ls = new NCollection_List<Shape>();
                ls.Append(cnode);
                _cuttingNode[node] = ls;
            }
        }

        // Remove two initial vertices
        seqVert.Remove(1);
        seqVert.Remove(seqVert.Length());
        seqPara.Remove(1);
        seqPara.Remove(seqPara.Length());
    }

    //=================================================================================================

    /// <summary>Cuts the bound at the nodes into sections with their pcurves. Called from Cutting only.</summary>
    protected virtual void CreateSections(in Shape section,
                                          NCollection_Sequence<Shape> seqNode,
                                          NCollection_Sequence<double> seqPara,
                                          NCollection_List<Shape> listEdge)
    {
        Edge sec = ShapeCasts.Edge(section);
        //  TopAbs_Orientation aInitOr = sec.Orientation();

        // To keep NM vertices on edge
        var seqNMVert = new NCollection_Sequence<Shape>();
        var seqNMPars = new NCollection_Sequence<double>();
        FindNMVertices(sec, seqNMVert, seqNMPars);

        var builder = new BrepBuilder();

        Tool.Range(sec, out double first, out double last);

        // Create cutting sections
        double par1, par2;
        Shape v1, v2;
        int i, len = seqPara.Length() + 1;
        for (i = 1; i <= len; i++)
        {
            Edge edge = sec;
            edge.EmptyCopy();

            if (i == 1)
            {
                par1 = first;
                par2 = seqPara[i];
                v1 = Exploration.FirstVertex(sec);
                v2 = seqNode[i];
            }
            else if (i == len)
            {
                par1 = seqPara[i - 1];
                par2 = last;
                v1 = seqNode[i - 1];
                v2 = Exploration.LastVertex(sec);
            }
            else
            {
                par1 = seqPara[i - 1];
                par2 = seqPara[i];
                v1 = seqNode[i - 1];
                v2 = seqNode[i];
            }

            Shape tmpShape = edge.Oriented(Forward);
            Edge tmpEdge = ShapeCasts.Edge(tmpShape); // for porting
            tmpShape = v1.Oriented(Forward);
            builder.Add(tmpEdge, tmpShape);
            tmpShape = v2.Oriented(Reversed);
            builder.Add(tmpEdge, tmpShape);
            builder.Range(tmpEdge, par1, par2);

            int k = 1;
            for (; k <= seqNMPars.Length(); k++)
            {
                double apar = seqNMPars.Value(k);
                if (apar >= par1 && apar <= par2)
                {
                    builder.Add(tmpEdge, seqNMVert.Value(k));
                    seqNMVert.Remove(k);
                    seqNMPars.Remove(k);
                    k--;
                }
            }
            listEdge.Append(edge);
        }

        NCollection_List<Shape> listFaces = _boundFaces.GetValue(sec);
        if (listFaces.Extent() == 0)
        {
            return;
        }

        double tolEdge = Tool.Tolerance(sec);

        // Add cutting pcurves
        var itf = new NCollection_List<Shape>.Iterator(listFaces);
        for (; itf.More(); itf.Next())
        {
            Face fac = ShapeCasts.Face(itf.Value());

            // Retrieve curve on surface
            double first2d1 = 0, last2d1 = 0.0;
            Curve2D? c2d = Tool.CurveOnSurface(sec, fac, out double first2d, out double last2d);
            if (c2d is null)
            {
                continue;
            }
            Curve2D? c2d1 = null;
            bool isSeam = Tool.IsClosed(sec, fac);

            // gka  - Convert to BSpline was commented because
            // it is not necessary to create BSpline instead of Lines or cIrcles.
            // Besides after conversion circles to BSpline
            // it is necessary to recompute parameters of cutting because parameterization of created
            // BSpline curve differs from parametrization of circle.

            // gka fix for bug OCC12203 21.04.06 addition second curve for seam edges

            if (isSeam)
            {
                Edge secRev = ShapeCasts.Edge(sec.Reversed());

                c2d1 = Tool.CurveOnSurface(secRev, fac, out first2d1, out last2d1);
                if (c2d1 is null)
                {
                    continue;
                }
            }

            // Update cutting sections
            Curve2D? c2dNew = null, c2d1New = null;
            var ite = new NCollection_List<Shape>.Iterator(listEdge);
            for (; ite.More(); ite.Next())
            {
                // Retrieve cutting section
                Edge edge = ShapeCasts.Edge(ite.Value());
                Tool.Range(edge, out par1, out par2);

                // Cut BSpline pcurve
                c2dNew = (Curve2D)c2d.Copy();
                if (c2d1 is not null)
                {
                    c2d1New = (Curve2D)c2d1.Copy();
                }

                if (!isSeam && c2d1New is null)
                {
                    builder.UpdateEdge(edge, c2dNew, fac, tolEdge);
                }
                else
                {
                    Orientation ori = edge.Orientation;
                    if (fac.Orientation == Reversed)
                    {
                        ori = global::Kerf.Engine.Topology.TopAbs.Reverse(ori);
                    }

                    if (ori == Forward)
                    {
                        builder.UpdateEdge(edge, c2dNew, c2d1New, fac, tolEdge);
                    }
                    else
                    {
                        builder.UpdateEdge(edge, c2d1New, c2dNew, fac, tolEdge);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Makes all the edges of the sewn shape same parameter if the same parameter mode is on. Called from Perform only.
    /// </summary>
    protected virtual void SameParameterShape()
    {
        if (!_sameParameterMode)
        {
            return;
        }
        var exp = new Explorer(_sewedShape, ShapeType.Edge);
        // Le flag sameparameter est a false pour chaque edge cousue
        for (; exp.More(); exp.Next())
        {
            Edge sec = ShapeCasts.Edge(exp.Current());
            try
            {
                BrepLibrary.SameParameter(sec, Tool.Tolerance(sec));
            }
            catch (FailureException)
            {
                continue;
            }
        }
    }

    //=================================================================================================

    /// <summary>The context, which records the replacements made by sewing.</summary>
    /// <remarks>When the inputs were copied (they held frozen shapes), it records the working copies' sub-shapes.</remarks>
    public ReShape GetContext() => _reShape;

    //=================================================================================================

    /// <summary>Sets the context.</summary>
    public void SetContext(ReShape context) => _reShape = context;
}
