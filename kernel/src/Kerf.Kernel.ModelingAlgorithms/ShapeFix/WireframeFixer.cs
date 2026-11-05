// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes the wireframe of a shape: the gaps between the edges of its wires (in 3d and on the faces) and its small
/// edges (merged with a neighbour, or dropped with <see cref="ModeDropSmallEdges"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Load"/> only records the shape. The fixes edit the shape they work on in
/// place, as OCCT's, in a working overlay (Kerf unit 10H4) that takes the writes into frozen shapes (any shape a caller
/// holds), so the caller's shape never changes: this tool's own from the first fix to <see cref="Shape"/>, or, inside
/// another operation's overlay (ShapeFix_Shape's, ShapeProcess's), that one. Set the context before the first fix.
/// <see cref="Shape"/> ends the work: the result holds new TShapes for what changed and the caller's own sub-shapes
/// for the rest, the context records the sub-shapes changed in place (it answers for the caller's shapes), and the
/// result is frozen. A fix after <see cref="Shape"/> starts a new overlay.
/// </remarks>
internal class WireframeFixer : RootFixer
{
    /// <summary>The shape fixed (OCCT's own field, hiding the root's).</summary>
    protected new Shape _shape;

    private bool   _modeDrop;
    private double _limitAngle;
    private int    _statusWireGaps;
    private int    _statusSmallEdges;

    // Kerf: the working overlay, from the first fix after a load (or a public result) to the next public result (unit
    // 10H4): this tool's own (detached) or the current operation's, which it doesn't finish.
    private WorkingOverlay? _overlay;
    private bool                   _ownsOverlay;
    private bool                   _toEnter = true;
    // True when the first fix made the context (OCCT makes it in the fix, and then skips applying it).
    private bool                   _isNewContext;

    /// <summary>Empty constructor.</summary>
    public WireframeFixer()
    {
        _shape = new Shape();
        ClearStatuses();
        _modeDrop   = false;
        _limitAngle = -1;
    }

    /// <summary>Initializes by a shape.</summary>
    public WireframeFixer(in Shape shape)
    {
        ClearStatuses();
        _shape      = shape;
        _modeDrop   = false;
        _limitAngle = -1;
    }

    /// <summary>Clears the statuses.</summary>
    public virtual void ClearStatuses()
    {
        int emptyStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        _statusWireGaps   = emptyStatus;
        _statusSmallEdges = emptyStatus;
    }

    /// <summary>Loads a shape, resets the statuses.</summary>
    public void Load(in Shape shape)
    {
        ClearStatuses();
        _shape   = shape;
        _overlay = null;
        _toEnter = true;
    }

    /// <summary>
    /// Fixes the gaps between the edges of the wires (ShapeFix_Wire's FixGaps3d and, on faces, FixGaps2d), then makes the
    /// edges SameParameter and fixes the self-intersections and vertex tolerances of the wires. Returns true if a gap
    /// was fixed.
    /// </summary>
    public bool FixWireGaps()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixWireGapsCore();
    }

    private bool FixWireGapsCore()
    {
        _statusWireGaps = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (_shape.IsNull)
        {
            return false;
        }

        if (Context() is null || _isNewContext)
        {
            _isNewContext = false;
            if (Context() is null)
            {
                SetContext(new ReShapeBuilder());
            }
        }
        else
        {
            Shape shape = _shape;
            _shape.Nullify();
            _shape = Context()!.InternalApply(shape);
        }
        ReShapeBuilder context = Context()!;

        double prec = (Precision() > 0.0) ? Precision() : Kerf.Engine.Foundation.Precision.Confusion();
        var    cont = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        if (_shape.Type == ShapeType.Compound)
        {
            bool locModified = false;
            var  c           = new Compound();
            var  b           = new BrepBuilder();
            b.MakeCompound(ref c);
            Shape savShape = _shape;
            for (var it = new Iterator(savShape); it.More(); it.Next())
            {
                Shape    shape1 = it.Value();
                Placement l = shape1.Placement, nullLoc = new();
                shape1.Location(nullLoc);
                Shape res;
                if (cont.ContainsKey(shape1))
                {
                    res = cont.GetValue(shape1).Oriented(shape1.Orientation);
                }
                else
                {
                    _shape = shape1;
                    FixWireGapsCore();
                    res = _shape;
                    cont[_shape] = res;
                }
                if (!res.IsSame(shape1))
                {
                    locModified = true;
                }
                res.Location(l);
                b.Add(c, res);
            }
            if (locModified)
            {
                c.Orient(savShape.Orientation);
                context.Replace(savShape, c);
            }
            _shape = context.InternalApply(savShape);
            return StatusWireGaps(ExtendStatus.Done);
        }
        var sfw = new WireFixer();
        sfw.SetContext(context);
        sfw.SetPrecision(prec);

        Face face;
        for (var expf1 = new Explorer(_shape, ShapeType.Face); expf1.More(); expf1.Next())
        {
            // smh#8
            Shape tmpF = context.InternalApply(expf1.Current());
            face              = ShapeCasts.Face(tmpF);
            if (face.Orientation == Reversed)
            {
                face.Orient(Forward);
            }
            for (var itw = new Iterator(face); itw.More(); itw.Next())
            {
                if (itw.Value().Type != ShapeType.Wire)
                {
                    continue;
                }
                // smh#8
                Shape tmpW = context.InternalApply(itw.Value());
                sfw.Init(ShapeCasts.Wire(tmpW), face, prec);
                sfw.FixReorder();
                sfw.FixGaps3d();
                if (sfw.StatusGaps3d(ExtendStatus.Done))
                {
                    _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                if (sfw.StatusGaps3d(ExtendStatus.Fail))
                {
                    _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
                sfw.FixGaps2d();
                if (sfw.StatusGaps2d(ExtendStatus.Done))
                {
                    _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
                if (sfw.StatusGaps2d(ExtendStatus.Fail))
                {
                    _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
                if (sfw.StatusGaps3d(ExtendStatus.Done) || sfw.StatusGaps2d(ExtendStatus.Done))
                {
                    SendWarning(itw.Value(), new Msg("FixWireframe.FixFixWireGaps.MSG0"));
                }
            }
        }

        //============================================================
        // Author :  enk
        // Purpose:  This block fixing a 3d wire which not lie on plane
        // Part 1
        //============================================================
        for (var expw = new Explorer(_shape, ShapeType.Wire, ShapeType.Face); expw.More(); expw.Next())
        {
            Shape tmpW = context.InternalApply(expw.Current());
            sfw.Load(ShapeCasts.Wire(tmpW));
            sfw.SetPrecision(prec);
            sfw.FixReorder();
            sfw.FixGaps3d();
            if (sfw.StatusGaps3d(ExtendStatus.Done))
            {
                _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
            if (sfw.StatusGaps3d(ExtendStatus.Fail))
            {
                _statusWireGaps |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }
            if (sfw.StatusGaps3d(ExtendStatus.Done))
            {
                SendWarning(expw.Current(), new Msg("FixWireframe.FixFixWireGaps.MSG0"));
            }
        }
        // End Part1========================================================

        if (StatusWireGaps(ExtendStatus.Done))
        {
            _shape = context.InternalApply(_shape);

            ShapeFixFixer.SameParameter(_shape, false);

            Wire wire;
            var         sfe = new EdgeFixer();
            for (var expf2 = new Explorer(_shape, ShapeType.Face); expf2.More(); expf2.Next())
            {
                face = ShapeCasts.Face(expf2.Current());
                if (face.Orientation == Reversed)
                {
                    face.Orient(Forward);
                }
                for (var itw = new Iterator(face); itw.More(); itw.Next())
                {
                    if (itw.Value().Type != ShapeType.Wire)
                    {
                        continue;
                    }
                    wire = ShapeCasts.Wire(itw.Value());
                    sfw.Init(wire, face, prec);
                    sfw.FixReorder();
                    sfw.FixSelfIntersection();
                    for (var ite = new Iterator(wire); ite.More(); ite.Next())
                    {
                        sfe.FixVertexTolerance(ShapeCasts.Edge(ite.Value()));
                    }
                }
            }

            // enk Part 2
            for (var expw2 = new Explorer(_shape, ShapeType.Wire, ShapeType.Face); expw2.More(); expw2.Next())
            {
                wire = ShapeCasts.Wire(expw2.Current());
                sfw.Load(wire);
                sfw.SetPrecision(prec);
                sfw.FixReorder();
                sfw.FixSelfIntersection();
                for (var ite = new Iterator(wire); ite.More(); ite.Next())
                {
                    sfe.FixVertexTolerance(ShapeCasts.Edge(ite.Value()));
                }
            }
            // End Part 2

            return true;
        }

        return false;
    }

    //=======================================================================
    // function : JoinEdges (static)
    // purpose  : used in FixSmallEdges
    //=======================================================================
    private static bool JoinEdges(in Edge e1, in Edge e2, out Edge e3, NCollection_List<Shape> faces)
    {
        bool replaceFirst = true;
        var  sae          = new EdgeAnalysis();
        var  newedge      = new Edge();
        var  newedge1     = new Edge();
        e3 = newedge1;
        Vertex v11 = sae.FirstVertex(e1);
        Vertex v12 = sae.LastVertex(e1);
        Vertex v21 = sae.FirstVertex(e2);
        Vertex v22 = sae.LastVertex(e2);

        bool isSame = (v11.IsSame(v12) || v22.IsSame(v21));
        var  b      = new BrepBuilder();
        b.MakeEdge(ref newedge);
        double first1, first2, last1, last2;
        newedge.Orient(Forward);
        try
        {
            if (!sae.Curve3d(e1, out Curve3D? c3d1, out double cf1, out double cl1, false))
            {
                return replaceFirst;
            }
            if (!sae.Curve3d(e2, out Curve3D? c3d2, out double cf2, out double cl2, false))
            {
                return replaceFirst;
            }

            b.Add(newedge, v11.Oriented(Forward));
            b.Add(newedge, v22.Oriented(Reversed));

            //     double newf,newl;
            if (!global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalJoinCurves(c3d1!, c3d2!, e1.Orientation, e2.Orientation, ref cf1, ref cl1, ref cf2,
                                                   ref cl2, out Curve3D? cRes, out bool isRev1, out bool isRev2))
            {
                return replaceFirst;
            }
            //    if(isRev1 || isRev2)
            if (!isSame && (isRev1 || isRev2))
            {
                return replaceFirst;
            }
            replaceFirst = (!isRev1);

            double             newf        = cf1;
            double             newl        = cl1 + cl2 - cf2;
            Orientation orEdge1     = e1.Orientation;
            Orientation orEdge2     = e2.Orientation;
            bool               ismanifold  = (orEdge1 == Forward || orEdge1 == Reversed);
            bool               ismanifold2 = (orEdge2 == Forward || orEdge2 == Reversed);
            if (ismanifold != ismanifold2)
            {
                return replaceFirst;
            }

            if (ismanifold)
            {
                orEdge1 = ((!isRev1 && e1.Orientation == Forward) || (isRev1 && e1.Orientation == Reversed)
                               ? Forward
                               : Reversed);
                orEdge2 = ((!isRev2 && e2.Orientation == Forward) || (isRev2 && e2.Orientation == Reversed)
                               ? Forward
                               : Reversed);
            }
            b.UpdateEdge(newedge, cRes, Math.Max(Tool.Tolerance(e1), Tool.Tolerance(e2)));
            double fp = cRes!.FirstParameter;
            double lp = cRes.LastParameter;
            if (fp > newf)
            {
                newf = fp;
            }
            if (lp < newl)
            {
                newl = lp;
            }
            b.Range(newedge, newf, newl);

            // merging pcurves
            for (var iter = new NCollection_List<Shape>.Iterator(faces); iter.More(); iter.Next())
            {
                Face face = ShapeCasts.Face(iter.Value());
                if (!sae.PCurve(e1, face, out Curve2D? c2d1, out first1, out last1, false))
                {
                    return replaceFirst;
                }
                if (!sae.PCurve(e2, face, out Curve2D? c2d2, out first2, out last2, false))
                {
                    return replaceFirst;
                }

                if (!global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalJoinCurves(c2d1!, c2d2!, orEdge1, orEdge2, ref first1, ref last1, ref first2,
                                                       ref last2, out Curve2D? c2dRes, out bool isRev12, out bool isRev22,
                                                       isSame))
                {
                    return replaceFirst;
                }

                if (ismanifold && (!isSame && (isRev12 || isRev22)))
                {
                    return replaceFirst;
                }

                if (isRev12)
                {
                    replaceFirst = false;
                }
                double fp2d = c2dRes!.FirstParameter;
                double lp2d = c2dRes.LastParameter;
                // B.UpdateEdge(newedge,C2dRes,face,0);
                double newf1 = first1;
                double newl1 = last1 + (last2 - first2);
                if (fp2d > newf1)
                {
                    newf1 = fp2d;
                }
                if (lp2d < newl1)
                {
                    newl1 = lp2d;
                }

                // dealing with seams: the same again
                if (sae.IsSeam(e1, face) && sae.IsSeam(e2, face))
                {
                    // smh#8
                    Shape tmpE1 = e1.Reversed(), tmpE2 = e2.Reversed();
                    Edge  e1t   = ShapeCasts.Edge(tmpE1);
                    Edge  e2t   = ShapeCasts.Edge(tmpE2);
                    sae.PCurve(e1t, face, out Curve2D? c2d12, out first1, out last1, false);
                    sae.PCurve(e2t, face, out Curve2D? c2d22, out first2, out last2, false);

                    if (!global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalJoinCurves(c2d12!, c2d22!, orEdge1, orEdge2, ref first1, ref last1, ref first2,
                                                           ref last2, out Curve2D? c2dRes2, out isRev12, out isRev22, isSame))
                    {
                        return replaceFirst;
                    }
                    if (!isSame && (isRev1 || isRev2))
                    {
                        return replaceFirst;
                    }
                    b.UpdateEdge(newedge, c2dRes, c2dRes2, face, 0);
                }
                else if (sae.IsSeam(e1, face) || sae.IsSeam(e2, face))
                {
                    return replaceFirst;
                }
                else if (!sae.IsSeam(e1, face) && !sae.IsSeam(e2, face))
                {
                    b.UpdateEdge(newedge, c2dRes, face, 0);
                }
                b.Range(newedge, face, newf1, newl1);
                if (!ismanifold)
                {
                    newedge.Orient(replaceFirst ? orEdge1 : orEdge2);
                }
            }
            b.SameRange(newedge, false);

            e3 = newedge;
            return replaceFirst;
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return replaceFirst;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Fixes the small edges of the shape (shorter than the precision): each is merged with a neighbour on the same
    /// faces (unless the angle between them exceeds <see cref="LimitAngle"/>), or dropped with
    /// <see cref="ModeDropSmallEdges"/>. Returns true if something was done.
    /// </summary>
    public bool FixSmallEdges()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixSmallEdgesCore();
    }

    private bool FixSmallEdgesCore()
    {
        _statusSmallEdges = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (_shape.IsNull)
        {
            return false;
        }

        if (Context() is null || _isNewContext)
        {
            _isNewContext = false;
            if (Context() is null)
            {
                SetContext(new ReShapeBuilder());
            }
        }
        else
        {
            Shape shape = _shape;
            _shape.Nullify();
            _shape = Context()!.InternalApply(shape);
        }
        ReShapeBuilder context = Context()!;
        var                cont     = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        if (_shape.Type == ShapeType.Compound)
        {
            bool locModified = false;
            var  c           = new Compound();
            var  b           = new BrepBuilder();
            b.MakeCompound(ref c);
            Shape savShape = _shape;
            for (var it = new Iterator(savShape); it.More(); it.Next())
            {
                Shape    shape1 = it.Value();
                Placement l = shape1.Placement, nullLoc = new();
                shape1.Location(nullLoc);
                Shape res;
                if (cont.ContainsKey(shape1))
                {
                    res = cont.GetValue(shape1).Oriented(shape1.Orientation);
                }
                else
                {
                    _shape = shape1;
                    FixSmallEdgesCore();
                    res = _shape;
                    cont[_shape] = res;
                }
                if (!res.IsSame(shape1))
                {
                    locModified = true;
                }

                // check if resulting shape if not empty
                if (res.IsNull)
                {
                    continue;
                }

                res.Location(l);
                b.Add(c, res);
            }
            if (locModified)
            {
                c.Orient(savShape.Orientation);
                context.Replace(savShape, c);
            }
            _shape = context.InternalApply(savShape);
            return StatusSmallEdges(ExtendStatus.Done);
        }
        var theSmallEdges    = new HashSet<Shape>(ShapeHasher.Instance);
        var theMultyEdges    = new HashSet<Shape>(ShapeHasher.Instance);
        var theEdgeToFaces   = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var theFaceWithSmall = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        CheckSmallEdgesCore(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
        MergeSmallEdgesCore(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
        return StatusSmallEdges(ExtendStatus.Done);
    }

    //=================================================================================================

    /// <summary>
    /// Finds the small edges of the shape, the faces of each edge, the faces holding small edges and the edges met
    /// more than once in a wire (not seams). Returns true if a small edge was found.
    /// </summary>
    public bool CheckSmallEdges(HashSet<Shape> smallEdges,
                                Dictionary<Shape, NCollection_List<Shape>> edgeToFaces,
                                Dictionary<Shape, NCollection_List<Shape>> faceWithSmall,
                                HashSet<Shape> multyEdges)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return CheckSmallEdgesCore(smallEdges, edgeToFaces, faceWithSmall, multyEdges);
    }

    private bool CheckSmallEdgesCore(HashSet<Shape> smallEdges,
                                 Dictionary<Shape, NCollection_List<Shape>> edgeToFaces,
                                 Dictionary<Shape, NCollection_List<Shape>> faceWithSmall,
                                 HashSet<Shape> multyEdges)
    {
        Face face;
        Edge edge;
        var         saw = new WireAnalysis();

        for (var expf1 = new Explorer(_shape, ShapeType.Face); expf1.More(); expf1.Next())
        {
            var         theEdgeList = new NCollection_List<Shape>();
            Face facet       = ShapeCasts.Face(expf1.Current());
            face = facet;
            if (facet.Orientation == Reversed)
            {
                face = ShapeCasts.Face(facet.Oriented(Forward));
            }
            for (var itw = new Iterator(face); itw.More(); itw.Next())
            {
                if (itw.Value().Type != ShapeType.Wire)
                {
                    continue;
                }
                Wire w   = ShapeCasts.Wire(itw.Value());
                var         aswd = new ExtendWireData(w, true, false);
                saw.Init(aswd, face, Precision());
                // pnd protection on seam edges
                var edgeMap = new Dictionary<Shape, int>(ShapeHasher.Instance);
                int i;
                for (i = 1; i <= saw.EdgeCount; i++)
                {
                    edge = saw.WireData()!.Edge(i);
                    if (edgeMap.ContainsKey(edge))
                    {
                        edgeMap.GetValueRef(edge)++;
                    }
                    else
                    {
                        edgeMap[edge] = 1;
                    }
                }

                for (i = 1; i <= saw.EdgeCount; i++)
                {
                    edge = saw.WireData()!.Edge(i);
                    if (edgeMap.GetValue(edge) != 1)
                    {
                        if (!saw.WireData()!.IsSeam(i))
                        {
                            multyEdges.Add(edge);
                        }
                        continue;
                    }
                    // Append current face to the list
                    if (edgeToFaces.ContainsKey(edge))
                    {
                        edgeToFaces.GetValue(edge).Append(facet);
                    }
                    else
                    {
                        var theFaceList = new NCollection_List<Shape>();
                        theFaceList.Append(facet);
                        edgeToFaces[edge] = theFaceList;
                    }
                    // Check if current edge is small
                    if (smallEdges.Contains(edge))
                    {
                        theEdgeList.Append(edge);
                    }
                    else if (saw.CheckSmall(i, Precision()))
                    {
                        smallEdges.Add(edge);
                        theEdgeList.Append(edge);
                    }
                }
            }
            // Add current face to the map if has small edges
            if (theEdgeList.Extent() != 0)
            {
                faceWithSmall[facet] = theEdgeList;
            }
        }

        //=========================================================================
        // Author : enk
        // Purpose: Analyzing of shape for small edges, if edge doesn't lie on face
        //=========================================================================
        for (var expw1 = new Explorer(_shape, ShapeType.Wire, ShapeType.Face); expw1.More(); expw1.Next())
        {
            saw.SetPrecision(Precision());
            var         edgeMap = new Dictionary<Shape, int>(ShapeHasher.Instance);
            int         i;
            Wire theWire     = ShapeCasts.Wire(expw1.Current());
            var         theEdgeList = new NCollection_List<Shape>();
            saw.Load(theWire);
            if (!saw.IsLoaded)
            {
                return false;
            }
            for (i = 1; i <= saw.EdgeCount; i++)
            {
                edge = saw.WireData()!.Edge(i);
                if (edgeMap.ContainsKey(edge))
                {
                    edgeMap.GetValueRef(edge)++;
                }
                else
                {
                    edgeMap[edge] = 1;
                }
            }

            for (i = 1; i <= saw.EdgeCount; i++)
            {
                edge = saw.WireData()!.Edge(i);
                if (edgeMap.GetValue(edge) != 1)
                {
                    if (!saw.WireData()!.IsSeam(i))
                    {
                        multyEdges.Add(edge);
                    }
                    continue;
                }

                // Check if current edge is small
                if (smallEdges.Contains(edge))
                {
                    theEdgeList.Append(edge);
                }
                else if (saw.CheckSmall(i, Precision()))
                {
                    smallEdges.Add(edge);
                    theEdgeList.Append(edge);
                }
            }
        }
        return (smallEdges.Count != 0);
    }

    //=================================================================================================

    /// <summary>
    /// Merges (or drops) the small edges found by <see cref="CheckSmallEdges"/>, as <see cref="FixSmallEdges"/>
    /// describes; <paramref name="modeDrop"/> and <paramref name="limitAngle"/> add to the tool's own settings.
    /// Returns true if something was done.
    /// </summary>
    public bool MergeSmallEdges(HashSet<Shape> smallEdges,
                                Dictionary<Shape, NCollection_List<Shape>> edgeToFaces,
                                Dictionary<Shape, NCollection_List<Shape>> faceWithSmall,
                                HashSet<Shape> multyEdges,
                                bool modeDrop = false,
                                double limitAngle = -1)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return MergeSmallEdgesCore(smallEdges, edgeToFaces, faceWithSmall, multyEdges, modeDrop, limitAngle);
    }

    private bool MergeSmallEdgesCore(HashSet<Shape> smallEdges,
                                 Dictionary<Shape, NCollection_List<Shape>> edgeToFaces,
                                 Dictionary<Shape, NCollection_List<Shape>> faceWithSmall,
                                 HashSet<Shape> multyEdges,
                                 bool modeDrop = false,
                                 double limitAngle = -1)
    {
        bool   modLimitAngle = (limitAngle > -1.0 || _limitAngle > -1.0);
        double aLimitAngle    = Math.Max(limitAngle, _limitAngle);

        bool aModeDrop      = modeDrop || _modeDrop;
        var  theNewVertices = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        if (smallEdges.Count != 0)
        {
            ReShapeBuilder context = Context()!;
            var                sfw      = new WireFixer();
            sfw.SetContext(context);
            var         SAE = new EdgeAnalysis();
            Edge edge1, edge2, edge3;
            // Iterate on map of faces with small edges
            var expf2 = new Explorer(_shape, ShapeType.Face);
            for (; expf2.More(); expf2.Next())
            {
                if (faceWithSmall.ContainsKey(expf2.Current()))
                {
                    if (faceWithSmall.GetValue(expf2.Current()).Extent() != 0)
                    {
                        // smh#8
                        Shape tmpShape = context.InternalApply(expf2.Current());
                        Face  facet    = ShapeCasts.Face(tmpShape);
                        if (!facet.IsSame(expf2.Current()))
                        { // gka
                            var expEdge = new Explorer(expf2.Current(), ShapeType.Edge);
                            for (; expEdge.More(); expEdge.Next())
                            {
                                int stat = context.InternalStatus(expEdge.Current(), out Shape newEdge, true);
                                if (stat > 0)
                                {
                                    if (smallEdges.Contains(expEdge.Current()))
                                    {
                                        smallEdges.Remove(expEdge.Current());
                                        smallEdges.Add(newEdge);
                                    }
                                    if (edgeToFaces.ContainsKey(expEdge.Current()))
                                    {
                                        NCollection_List<Shape> listFaces = edgeToFaces.GetValue(expEdge.Current());
                                        edgeToFaces.Remove(expEdge.Current());
                                        edgeToFaces[newEdge] = listFaces;
                                    }
                                }
                            }
                        }
                        Face face = facet;
                        // if (face.Orientation()==TopAbs_REVERSED)
                        //   face = TopoDS::Face(facet.Oriented(TopAbs_FORWARD));
                        for (var itw = new Iterator(face); itw.More(); itw.Next())
                        {
                            if (itw.Value().Type != ShapeType.Wire)
                            {
                                continue;
                            }
                            Wire wire = ShapeCasts.Wire(itw.Value());
                            if (face.Orientation == Reversed)
                            {
                                face = ShapeCasts.Face(facet.Oriented(Forward));
                            }
                            var aswd = new ExtendWireData(wire, true, false);
                            // SFW->Load(aWire);
                            sfw.Load(aswd);
                            sfw.FixReorder();
                            int prev, next, index = 1;

                            while (index <= sfw.WireData()!.EdgeCount && sfw.NbEdges() > 1)
                            {
                                prev  = (index == 1) ? sfw.WireData()!.EdgeCount : index - 1;
                                next  = (index == sfw.WireData()!.EdgeCount) ? 1 : index + 1;
                                edge1 = sfw.WireData()!.Edge(prev);
                                edge2 = sfw.WireData()!.Edge(index);
                                edge3 = sfw.WireData()!.Edge(next);

                                // gka protection against joining seem edge
                                if (edge2.IsSame(edge1) || edge2.IsSame(edge3))
                                {
                                    // if(BRep_Tool::IsClosed(edge2,face)) {
                                    index++;
                                    continue;
                                }

                                bool isSeam  = sfw.WireData()!.IsSeam(index);
                                bool isSeam1 = sfw.WireData()!.IsSeam(prev);
                                bool isSeam2 = sfw.WireData()!.IsSeam(next);
                                if (smallEdges.Contains(edge2))
                                {
                                    // Middle edge is small - choose a pair of edges to join
                                    bool   isAnyJoin  = (edge1.IsSame(edge3));
                                    bool   take_next  = isAnyJoin; // false;
                                    bool   isLimAngle = false;
                                    double ang1 = 0.0, ang2 = 0.0;
                                    if (SAE.Curve3d(edge1, out Curve3D? c1, out _, out double last1)
                                        && SAE.Curve3d(edge2, out Curve3D? c2, out double first2, out double last2)
                                        && SAE.Curve3d(edge3, out Curve3D? c3, out double first3, out _))
                                    {
                                        // Compare angles between edges
                                        // double Ang1, Ang2;
                                        c1!.D1(last1, out _, out Vector3D vec1);
                                        c2!.D1(first2, out _, out Vector3D vec2);
                                        if (edge1.Orientation == Reversed)
                                        {
                                            vec1 = vec1.Reversed();
                                        }
                                        if (edge2.Orientation == Reversed)
                                        {
                                            vec2 = vec2.Reversed();
                                        }
                                        double tol2 = Kerf.Engine.Foundation.Precision.SquareConfusion();
                                        if (vec1.SquareMagnitude < tol2 || vec2.SquareMagnitude < tol2)
                                        {
                                            ang1 = Math.PI / 2.0;
                                        }
                                        else
                                        {
                                            ang1 = Math.Abs(vec1.AngleTo(vec2));
                                        }
                                        c2.D1(last2, out _, out vec1);
                                        c3!.D1(first3, out _, out vec2);
                                        if (edge2.Orientation == Reversed)
                                        {
                                            vec1 = vec1.Reversed();
                                        }
                                        if (edge3.Orientation == Reversed)
                                        {
                                            vec2 = vec2.Reversed();
                                        }
                                        if (vec1.SquareMagnitude < tol2 || vec2.SquareMagnitude < tol2)
                                        {
                                            ang2 = Math.PI / 2.0;
                                        }
                                        else
                                        {
                                            ang2 = Math.Abs(vec1.AngleTo(vec2));
                                        }
                                        // isLimAngle = (theLimitAngle != -1 && std::min(Ang1,Ang2) > theLimitAngle);
                                        // take_next = (Ang2<Ang1);
                                        // if (take_next) { edge1 = edge2; edge2 = edge3; }
                                    }
                                    // if(theLimitAngle != -1 && Ang1 > theLimitAngle && Ang2 >theLimitAngle) {
                                    //   index++; continue;
                                    // }

                                    // Check if edges lay on the same faces
                                    if (multyEdges.Contains(edge1) || multyEdges.Contains(edge2))
                                    { //??????
                                        index++;
                                        continue;
                                    }
                                    var theList1 = new NCollection_List<Shape>();
                                    var theList2 = new NCollection_List<Shape>();
                                    var theList3 = new NCollection_List<Shape>();
                                    if (edgeToFaces.ContainsKey(edge1))
                                    {
                                        theList1 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge1));
                                    }
                                    if (edgeToFaces.ContainsKey(edge2))
                                    {
                                        theList2 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge2));
                                    }
                                    if (edgeToFaces.ContainsKey(edge3))
                                    {
                                        theList3 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge3));
                                    }
                                    bool same_set = false;

                                    // gka protection against joining seem edges with other edges
                                    bool same_set1 = (theList1.Extent() == theList2.Extent()
                                                      && ((!isSeam && !isSeam1) || (isSeam && isSeam1))); // gka
                                    bool same_set2 = (theList3.Extent() == theList2.Extent()
                                                      && ((!isSeam && !isSeam2) || (isSeam && isSeam2)));
                                    var theSetOfFaces = new HashSet<Shape>(ShapeHasher.Instance);
                                    for (var itf1 = new NCollection_List<Shape>.Iterator(theList2); itf1.More(); itf1.Next())
                                    {
                                        theSetOfFaces.Add(itf1.Value());
                                    }
                                    if (same_set1)
                                    {
                                        // Add all faces of the first edge to the current set
                                        for (var itf2 = new NCollection_List<Shape>.Iterator(theList1); (itf2.More() && same_set1);
                                             itf2.Next())
                                        {
                                            same_set1 = theSetOfFaces.Contains(itf2.Value());
                                        }
                                    }
                                    if (same_set2)
                                    {
                                        // Add all faces of the first edge to the current set
                                        for (var itf2 = new NCollection_List<Shape>.Iterator(theList3); (itf2.More() && same_set2);
                                             itf2.Next())
                                        {
                                            same_set2 = theSetOfFaces.Contains(itf2.Value());
                                        }
                                    }
                                    if (same_set1 && same_set2)
                                    {
                                        same_set = true;
                                        if (Math.Abs(ang2 - ang1) > Kerf.Engine.Foundation.Precision.Angular())
                                        {
                                            take_next = (ang2 < ang1);
                                        }
                                        if (take_next)
                                        {
                                            edge1 = edge2;
                                            edge2 = edge3;
                                        }
                                        isLimAngle = (modLimitAngle && Math.Min(ang1, ang2) > aLimitAngle);
                                    }
                                    else if (same_set1 && !same_set2)
                                    {
                                        isLimAngle = (modLimitAngle && ang1 > aLimitAngle);
                                        same_set   = true;
                                    }
                                    else if (!same_set1 && same_set2)
                                    {
                                        same_set   = true;
                                        isLimAngle = (modLimitAngle && ang2 > aLimitAngle);
                                        edge1      = edge2;
                                        edge2      = edge3;
                                        take_next  = true;
                                    }
                                    if (same_set && !isLimAngle)
                                    {
                                        // Merge current pair of edges
                                        // gka protection against crossing seem on second face
                                        bool isNeedJoin = true; // false;
                                        for (var itF = new NCollection_List<Shape>.Iterator(theList2); itF.More() && isNeedJoin;
                                             itF.Next())
                                        {
                                            if (itF.Value().IsSame(expf2.Current()))
                                            {
                                                continue;
                                            }
                                            Shape f = context.InternalApply(itF.Value());
                                            // aF = aF.Oriented(TopAbs_FORWARD);
                                            for (var iw = new Iterator(f); iw.More(); iw.Next())
                                            {
                                                if (iw.Value().Type != ShapeType.Wire)
                                                {
                                                    continue;
                                                }
                                                Wire wt   = ShapeCasts.Wire(iw.Value());
                                                var         sfw1 = new WireFixer();
                                                sfw1.Load(wt);
                                                sfw1.FixReorder();
                                                ExtendWireData atmpswd = sfw1.WireData()!;
                                                int                  ind1    = atmpswd.Index(edge1);
                                                int                  ind2    = atmpswd.Index(edge2);
                                                if (ind1 != 0 && ind2 != 0)
                                                {
                                                    isNeedJoin = ((ind1 - ind2) == 1 || (ind2 == atmpswd.EdgeCount && ind1 == 1));
                                                    break;
                                                }
                                            }
                                        }
                                        bool replaceFirst = true;
                                        if (isNeedJoin)
                                        {
                                            var listF = new NCollection_List<Shape>();
                                            for (var itlF = new NCollection_List<Shape>.Iterator(theList2); itlF.More(); itlF.Next())
                                            {
                                                Shape tmpF = context.InternalApply(itlF.Value());
                                                listF.Append(tmpF);
                                            }
                                            replaceFirst = JoinEdges(edge1, edge2, out edge3, listF);
                                        }
                                        else
                                        {
                                            edge3 = new Edge();
                                        }
                                        if (edge3.IsNull)
                                        {
                                            index++;
                                            _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                                        }
                                        else
                                        {
                                            // Record vertex replacements in the map
                                            Vertex oldV1 = SAE.FirstVertex(edge3), oldV2 = SAE.LastVertex(edge3);
                                            if (!theNewVertices.ContainsKey(oldV1))
                                            // smh#8
                                            {
                                                Shape emptyCopiedV1 = oldV1.EmptyCopied();
                                                theNewVertices[oldV1] = ShapeCasts.Vertex(emptyCopiedV1);
                                            }
                                            if (!oldV1.IsSame(oldV2))
                                            {
                                                if (!theNewVertices.ContainsKey(oldV2))
                                                // smh#8
                                                {
                                                    Shape emptyCopiedV2 = oldV2.EmptyCopied();
                                                    theNewVertices[oldV2] = ShapeCasts.Vertex(emptyCopiedV2);
                                                }
                                            }

                                            // To keep NM vertices belonging initial edges
                                            var aItv = new Iterator(edge1, false);
                                            for (; aItv.More(); aItv.Next())
                                            {
                                                if (aItv.Value().Orientation == Internal
                                                    || aItv.Value().Orientation == External)
                                                {
                                                    Vertex oldV = ShapeCasts.Vertex(aItv.Value());
                                                    Vertex anewV =
                                                        TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, edge3, edge1);
                                                    var b = new BrepBuilder();
                                                    b.Add(edge3, anewV);
                                                    context.Replace(oldV, anewV);
                                                }
                                            }

                                            for (aItv.Initialize(edge2, false); aItv.More(); aItv.Next())
                                            {
                                                if (aItv.Value().Orientation == Internal
                                                    || aItv.Value().Orientation == External)
                                                {
                                                    var           b    = new BrepBuilder();
                                                    Vertex oldV = ShapeCasts.Vertex(aItv.Value());
                                                    Vertex anewV =
                                                        TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, edge3, edge2);
                                                    b.Add(edge3, anewV);
                                                    context.Replace(oldV, anewV);
                                                }
                                            }

                                            // Check for small resulting edge
                                            bool newsmall = false;
                                            var  saw      = new WireAnalysis();
                                            saw.Init(sfw.WireData(), face, Precision());
                                            // Make changes in WireData and Context
                                            if (replaceFirst)
                                            {
                                                context.Replace(edge1, edge3);
                                                context.Remove(edge2);
                                                SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            }
                                            else
                                            {
                                                context.Replace(edge2, edge3);
                                                context.Remove(edge1);
                                                SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            }
                                            if (take_next)
                                            {
                                                sfw.WireData()!.Set(edge3, next);
                                                newsmall = saw.CheckSmall(next, Precision());
                                            }
                                            else
                                            {
                                                sfw.WireData()!.Set(edge3, prev);
                                                newsmall = saw.CheckSmall(prev, Precision());
                                            }
                                            sfw.WireData()!.Remove(index);
                                            // Process changes in maps
                                            var theList = new NCollection_List<Shape>();
                                            theList.Append(theList2);
                                            edgeToFaces.Remove(edge1);
                                            edgeToFaces.Remove(edge2);
                                            edgeToFaces[edge3] = theList;
                                            if (smallEdges.Contains(edge1))
                                            {
                                                smallEdges.Remove(edge1);
                                            }
                                            if (smallEdges.Contains(edge2))
                                            {
                                                smallEdges.Remove(edge2);
                                            }
                                            if (newsmall)
                                            {
                                                smallEdges.Add(edge3);
                                            }
                                            for (var itlf = new NCollection_List<Shape>.Iterator(theList); itlf.More(); itlf.Next())
                                            {
                                                Shape curface = itlf.Value();
                                                if (faceWithSmall.ContainsKey(curface))
                                                {
                                                    NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                                    if (newsmall)
                                                    {
                                                        theEdges.Append(edge3);
                                                    }
                                                    var ite = new NCollection_List<Shape>.Iterator(theEdges);
                                                    while (ite.More())
                                                    {
                                                        Shape iedge = ite.Value();
                                                        if (iedge.IsSame(edge1) || iedge.IsSame(edge2))
                                                        {
                                                            theEdges.Remove(ite);
                                                        }
                                                        else
                                                        {
                                                            ite.Next();
                                                        }
                                                    }
                                                    // Remove face without small edges from the map
                                                    if (theEdges.Extent() == 0)
                                                    {
                                                        faceWithSmall.Remove(curface);
                                                    }
                                                }
                                            }
                                            _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                                        }
                                    }
                                    else if (aModeDrop)
                                    { // gka
                                        var tempWire = new ExtendWireData();
                                        var tempSaw  = new WireAnalysis();
                                        tempWire.Add(sfw.InternalWire());
                                        Edge remedge;
                                        if (take_next)
                                        {
                                            remedge = edge1;
                                        }
                                        else
                                        {
                                            remedge = edge2;
                                        }
                                        tempWire.Remove(index);
                                        tempSaw.Load(tempWire);
                                        int newindex = (index <= tempSaw.EdgeCount ? index : 1);
                                        tempSaw.CheckConnected(newindex, Precision());
                                        if (!tempSaw.LastCheckStatus(ExtendStatus.Fail))
                                        {
                                            sfw.WireData()!.Remove(index);
                                            Edge tmpedge1 = tempWire.Edge(newindex);
                                            Edge tmpedge2 = tempWire.Edge(newindex == 1 ? tempSaw.EdgeCount : (newindex - 1));
                                            var         aL1      = new NCollection_List<Shape>();
                                            if (edgeToFaces.ContainsKey(tmpedge1))
                                            {
                                                aL1 = new NCollection_List<Shape>(edgeToFaces.GetValue(tmpedge1));
                                            }
                                            var l2 = new NCollection_List<Shape>();
                                            if (edgeToFaces.ContainsKey(tmpedge2))
                                            {
                                                l2 = new NCollection_List<Shape>(edgeToFaces.GetValue(tmpedge2));
                                            }
                                            sfw.FixConnected(newindex <= sfw.NbEdges() ? newindex : 1, Precision());
                                            sfw.FixDegenerated(newindex <= sfw.NbEdges() ? newindex : 1);
                                            Shape aTmpShape = context.InternalApply(tmpedge1); // for porting
                                            Edge  anewedge1 = ShapeCasts.Edge(aTmpShape);
                                            aTmpShape = context.InternalApply(tmpedge2);
                                            Edge anewedge2 = ShapeCasts.Edge(aTmpShape);
                                            context.Remove(remedge);
                                            SendWarning(remedge, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            if (smallEdges.Contains(remedge))
                                            {
                                                smallEdges.Remove(remedge);
                                            }
                                            edgeToFaces.Remove(remedge);
                                            edgeToFaces.Remove(tmpedge1);
                                            edgeToFaces.Remove(tmpedge2);
                                            edgeToFaces[anewedge1] = aL1;
                                            edgeToFaces[anewedge2] = l2;
                                            if (smallEdges.Contains(tmpedge1))
                                            {
                                                smallEdges.Remove(tmpedge1);
                                                smallEdges.Add(anewedge1);
                                                for (var itlf = new NCollection_List<Shape>.Iterator(aL1); itlf.More(); itlf.Next())
                                                {
                                                    Shape                   curface  = itlf.Value();
                                                    NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                                    var                            ite      = new NCollection_List<Shape>.Iterator(theEdges);
                                                    while (ite.More())
                                                    {
                                                        Shape iedge = ite.Value();
                                                        if (iedge.IsSame(tmpedge1))
                                                        {
                                                            theEdges.Remove(ite);
                                                            theEdges.Append(anewedge1);
                                                        }
                                                        else
                                                        {
                                                            ite.Next();
                                                        }
                                                    }
                                                }
                                            }
                                            if (smallEdges.Contains(tmpedge2))
                                            {
                                                smallEdges.Remove(tmpedge2);
                                                smallEdges.Add(anewedge2);
                                                for (var itlf = new NCollection_List<Shape>.Iterator(l2); itlf.More(); itlf.Next())
                                                {
                                                    Shape                   curface  = itlf.Value();
                                                    NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                                    var                            ite      = new NCollection_List<Shape>.Iterator(theEdges);
                                                    while (ite.More())
                                                    {
                                                        Shape iedge = ite.Value();
                                                        if (iedge.IsSame(tmpedge2))
                                                        {
                                                            theEdges.Remove(ite);
                                                            theEdges.Append(anewedge2);
                                                        }
                                                        else
                                                        {
                                                            ite.Next();
                                                        }
                                                    }
                                                }
                                            }
                                            _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                                        }
                                        else
                                        {
                                            index++;
                                        }
                                    }
                                    else
                                    {
                                        // gka protection against removing circles
                                        Edge ed  = (take_next ? edge1 : edge2);
                                        var         sae = new EdgeAnalysis();
                                        if (sae.Curve3d(ed, out Curve3D? c3d, out double f1, out double l1, false))
                                        {
                                            c3d!.D0(f1, out Point3D p1);
                                            c3d.D0(l1, out Point3D p2);
                                            c3d.D0((f1 + l1) * 0.5, out Point3D p3);

                                            if (p1.DistanceTo(p3) > p1.DistanceTo(p2))
                                            {
                                                index++;
                                                continue;
                                            }
                                        }
                                        if (take_next && theList2.Extent() == 1)
                                        { // gka
                                            Vertex v1 = SAE.FirstVertex(edge1), v2 = SAE.LastVertex(edge1);
                                            if (v1.IsSame(v2))
                                            {
                                                sfw.WireData()!.Remove(index);
                                                context.Remove(edge1);
                                                SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                                if (smallEdges.Contains(edge1))
                                                {
                                                    smallEdges.Remove(edge1);
                                                }
                                                edgeToFaces.Remove(edge1);
                                                _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                            }
                                            else
                                            {
                                                index++;
                                            }
                                        }
                                        else if (!take_next && theList2.Extent() == 1)
                                        {
                                            Vertex v1 = SAE.FirstVertex(edge2), v2 = SAE.LastVertex(edge2);
                                            if (v1.IsSame(v2))
                                            {
                                                sfw.WireData()!.Remove(index);
                                                context.Remove(edge2);
                                                SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                                if (smallEdges.Contains(edge2))
                                                {
                                                    smallEdges.Remove(edge2);
                                                }
                                                edgeToFaces.Remove(edge2);
                                                _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                            }
                                            else
                                            {
                                                index++;
                                            }
                                        }
                                        else
                                        {
                                            index++;
                                        }
                                    }
                                }
                                else
                                {
                                    index++;
                                }
                            }
                            if (sfw.NbEdges() == 1 && aModeDrop)
                            {
                                edge1 = sfw.WireData()!.Edge(1);
                                if (smallEdges.Contains(edge1))
                                {
                                    sfw.WireData()!.Remove(1);
                                    context.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    smallEdges.Remove(edge1);
                                    edgeToFaces.Remove(edge1);
                                    context.Remove(wire);
                                    SendWarning(wire, new Msg("FixWireframe.FixSmallEdges.MSG1"));
                                    _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                }
                            }
                            else
                            {
                                sfw.FixConnected();
                                context.Replace(wire, sfw.InternalWire());
                            }
                        }
                        face.Orient(facet.Orientation);
                        Shape anewShape = context.InternalApply(face);
                        var          iter     = new Iterator(anewShape);
                        if (!iter.More())
                        {
                            context.Remove(anewShape);
                            SendWarning(face, new Msg("FixWireframe.FixSmallEdges.MSG2"));
                        }
                    }
                }
            }

            // enk block
            // Iterate on map of wires which not lie on faces
            for (var expw1 = new Explorer(_shape, ShapeType.Wire, ShapeType.Face); expw1.More(); expw1.Next())
            {
                Wire wire = ShapeCasts.Wire(expw1.Current());
                sfw.Load(wire);
                sfw.FixReorder();
                int prev, next, index = 1;
                while (index <= sfw.NbEdges() && sfw.NbEdges() > 1)
                {
                    prev  = (index == 1) ? sfw.NbEdges() : index - 1;
                    next  = (index == sfw.NbEdges()) ? 1 : index + 1;
                    edge1 = sfw.WireData()!.Edge(prev);
                    edge2 = sfw.WireData()!.Edge(index);
                    edge3 = sfw.WireData()!.Edge(next);

                    // gka protection against joining seem edge
                    if (edge2.IsSame(edge1) || edge2.IsSame(edge3))
                    {
                        // if(BRep_Tool::IsClosed(edge2,face)) {
                        index++;
                        continue;
                    }

                    bool isSeam  = sfw.WireData()!.IsSeam(index);
                    bool isSeam1 = sfw.WireData()!.IsSeam(prev);
                    bool isSeam2 = sfw.WireData()!.IsSeam(next);
                    if (smallEdges.Contains(edge2))
                    {
                        // Middle edge is small - choose a pair of edges to join
                        bool   isAnyJoin  = (edge1.IsSame(edge3));
                        bool   take_next  = isAnyJoin; // false;
                        bool   isLimAngle = false;
                        double ang1 = 0.0, ang2 = 0.0;
                        if (SAE.Curve3d(edge1, out Curve3D? c1, out _, out double last1)
                            && SAE.Curve3d(edge2, out Curve3D? c2, out double first2, out double last2)
                            && SAE.Curve3d(edge3, out Curve3D? c3, out double first3, out _))
                        {
                            // Compare angles between edges
                            // double Ang1, Ang2;
                            c1!.D1(last1, out _, out Vector3D vec1);
                            c2!.D1(first2, out _, out Vector3D vec2);
                            if (edge1.Orientation == Reversed)
                            {
                                vec1 = vec1.Reversed();
                            }
                            if (edge2.Orientation == Reversed)
                            {
                                vec2 = vec2.Reversed();
                            }
                            double tol2 = Kerf.Engine.Foundation.Precision.SquareConfusion();
                            if (vec1.SquareMagnitude < tol2 || vec2.SquareMagnitude < tol2)
                            {
                                ang1 = Math.PI / 2.0;
                            }
                            else
                            {
                                ang1 = Math.Abs(vec1.AngleTo(vec2));
                            }
                            c2.D1(last2, out _, out vec1);
                            c3!.D1(first3, out _, out vec2);
                            if (edge2.Orientation == Reversed)
                            {
                                vec1 = vec1.Reversed();
                            }
                            if (edge3.Orientation == Reversed)
                            {
                                vec2 = vec2.Reversed();
                            }
                            if (vec1.SquareMagnitude < tol2 || vec2.SquareMagnitude < tol2)
                            {
                                ang2 = Math.PI / 2.0;
                            }
                            else
                            {
                                ang2 = Math.Abs(vec1.AngleTo(vec2));
                            }
                            // isLimAngle = (theLimitAngle != -1 && std::min(Ang1,Ang2) > theLimitAngle);
                            // take_next = (Ang2<Ang1);
                            // if (take_next) { edge1 = edge2; edge2 = edge3; }
                        }
                        // if(theLimitAngle != -1 && Ang1 > theLimitAngle && Ang2 >theLimitAngle) {
                        //   index++; continue;
                        // }

                        // Check if edges lay on the same faces
                        if (multyEdges.Contains(edge1) || multyEdges.Contains(edge2))
                        { //??????
                            index++;
                            continue;
                        }
                        var theList1 = new NCollection_List<Shape>();
                        var theList2 = new NCollection_List<Shape>();
                        var theList3 = new NCollection_List<Shape>();
                        if (edgeToFaces.ContainsKey(edge1))
                        {
                            theList1 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge1));
                        }
                        if (edgeToFaces.ContainsKey(edge2))
                        {
                            theList2 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge2));
                        }
                        if (edgeToFaces.ContainsKey(edge3))
                        {
                            theList3 = new NCollection_List<Shape>(edgeToFaces.GetValue(edge3));
                        }
                        bool same_set = false;

                        // gka protection against joining seem edges with other edges
                        bool same_set1 = (theList1.Extent() == theList2.Extent()
                                          && ((!isSeam && !isSeam1) || (isSeam && isSeam1))); // gka
                        bool same_set2 = (theList3.Extent() == theList2.Extent()
                                          && ((!isSeam && !isSeam2) || (isSeam && isSeam2)));
                        var theSetOfFaces = new HashSet<Shape>(ShapeHasher.Instance);
                        for (var itf1 = new NCollection_List<Shape>.Iterator(theList2); itf1.More(); itf1.Next())
                        {
                            theSetOfFaces.Add(itf1.Value());
                        }
                        if (same_set1)
                        {
                            // Add all faces of the first edge to the current set
                            for (var itf2 = new NCollection_List<Shape>.Iterator(theList1); (itf2.More() && same_set1); itf2.Next())
                            {
                                same_set1 = theSetOfFaces.Contains(itf2.Value());
                            }
                        }
                        if (same_set2)
                        {
                            // Add all faces of the first edge to the current set
                            for (var itf2 = new NCollection_List<Shape>.Iterator(theList3); (itf2.More() && same_set2); itf2.Next())
                            {
                                same_set2 = theSetOfFaces.Contains(itf2.Value());
                            }
                        }
                        if (same_set1 && same_set2)
                        {
                            same_set = true;
                            if (Math.Abs(ang2 - ang1) > Kerf.Engine.Foundation.Precision.Angular())
                            {
                                take_next = (ang2 < ang1);
                            }
                            if (take_next)
                            {
                                edge1 = edge2;
                                edge2 = edge3;
                            }
                            isLimAngle = (modLimitAngle && Math.Min(ang1, ang2) > aLimitAngle);
                        }
                        else if (same_set1 && !same_set2)
                        {
                            isLimAngle = (modLimitAngle && ang1 > aLimitAngle);
                            same_set   = true;
                        }
                        else if (!same_set1 && same_set2)
                        {
                            same_set   = true;
                            isLimAngle = (modLimitAngle && ang2 > aLimitAngle);
                            edge1      = edge2;
                            edge2      = edge3;
                            take_next  = true;
                        }
                        if (same_set && !isLimAngle)
                        {
                            // Merge current pair of edges
                            // gka protection against crossing seem on second face
                            bool isNeedJoin = true; // false;
                            for (var itF = new NCollection_List<Shape>.Iterator(theList2); itF.More() && isNeedJoin; itF.Next())
                            {
                                // OCCT: anExpf2 has explored all the faces here: its Current() is the explored shape
                                if (itF.Value().IsSame(expf2.Current()))
                                {
                                    continue;
                                }
                                Shape f = context.InternalApply(itF.Value());
                                // aF = aF.Oriented(TopAbs_FORWARD);
                                for (var iw = new Iterator(f); iw.More(); iw.Next())
                                {
                                    if (iw.Value().Type != ShapeType.Wire)
                                    {
                                        continue;
                                    }
                                    Wire wt   = ShapeCasts.Wire(iw.Value());
                                    var         sfw1 = new WireFixer();
                                    sfw1.Load(wt);
                                    sfw1.FixReorder();
                                    ExtendWireData atmpswd = sfw1.WireData()!;
                                    int                  ind1    = atmpswd.Index(edge1);
                                    int                  ind2    = atmpswd.Index(edge2);
                                    if (ind1 != 0 && ind2 != 0)
                                    {
                                        isNeedJoin = ((ind1 - ind2) == 1 || (ind2 == atmpswd.EdgeCount && ind1 == 1));
                                        break;
                                    }
                                }
                            }
                            bool replaceFirst = true;
                            if (isNeedJoin)
                            {
                                var listF = new NCollection_List<Shape>();
                                for (var itlF = new NCollection_List<Shape>.Iterator(theList2); itlF.More(); itlF.Next())
                                {
                                    Shape tmpF = context.InternalApply(itlF.Value());
                                    listF.Append(tmpF);
                                }
                                replaceFirst = JoinEdges(edge1, edge2, out edge3, listF);
                            }
                            else
                            {
                                edge3 = new Edge();
                            }
                            if (edge3.IsNull)
                            {
                                index++;
                                _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                            }
                            else
                            {
                                // Record vertex replacements in the map
                                Vertex oldV1 = SAE.FirstVertex(edge3), oldV2 = SAE.LastVertex(edge3);
                                if (!theNewVertices.ContainsKey(oldV1))
                                // smh#8
                                {
                                    Shape emptyCopiedV1 = oldV1.EmptyCopied();
                                    theNewVertices[oldV1] = ShapeCasts.Vertex(emptyCopiedV1);
                                }
                                if (!oldV1.IsSame(oldV2))
                                {
                                    if (!theNewVertices.ContainsKey(oldV2))
                                    // smh#8
                                    {
                                        Shape emptyCopiedV2 = oldV2.EmptyCopied();
                                        theNewVertices[oldV2] = ShapeCasts.Vertex(emptyCopiedV2);
                                    }
                                }
                                // To keep NM vertices belonging initial edges
                                var aItv = new Iterator(edge1, false);
                                for (; aItv.More(); aItv.Next())
                                {
                                    if (aItv.Value().Orientation == Internal
                                        || aItv.Value().Orientation == External)
                                    {
                                        var           b    = new BrepBuilder();
                                        Vertex oldV = ShapeCasts.Vertex(aItv.Value());
                                        Vertex anewV =
                                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, edge3, edge1);
                                        b.Add(edge3, anewV);
                                        context.Replace(oldV, anewV);
                                    }
                                }

                                for (aItv.Initialize(edge2, false); aItv.More(); aItv.Next())
                                {
                                    if (aItv.Value().Orientation == Internal
                                        || aItv.Value().Orientation == External)
                                    {
                                        var           b    = new BrepBuilder();
                                        Vertex oldV = ShapeCasts.Vertex(aItv.Value());
                                        Vertex anewV =
                                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, edge3, edge2);
                                        b.Add(edge3, anewV);
                                        context.Replace(oldV, anewV);
                                    }
                                }
                                // Check for small resulting edge
                                bool newsmall = false;
                                var  saw      = new WireAnalysis();
                                saw.Load(sfw.WireData());
                                saw.SetPrecision(Precision());
                                // Make changes in WireData and Context
                                if (replaceFirst)
                                {
                                    context.Replace(edge1, edge3);
                                    context.Remove(edge2);
                                    SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                }
                                else
                                {
                                    context.Replace(edge2, edge3);
                                    context.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                }
                                if (take_next)
                                {
                                    sfw.WireData()!.Set(edge3, next);
                                    newsmall = saw.CheckSmall(next, Precision());
                                }
                                else
                                {
                                    sfw.WireData()!.Set(edge3, prev);
                                    newsmall = saw.CheckSmall(prev, Precision());
                                }
                                sfw.WireData()!.Remove(index);
                                // Process changes in maps
                                var theList = new NCollection_List<Shape>();
                                theList.Append(theList2);
                                edgeToFaces.Remove(edge1);
                                edgeToFaces.Remove(edge2);
                                edgeToFaces[edge3] = theList;
                                if (smallEdges.Contains(edge1))
                                {
                                    smallEdges.Remove(edge1);
                                }
                                if (smallEdges.Contains(edge2))
                                {
                                    smallEdges.Remove(edge2);
                                }
                                if (newsmall)
                                {
                                    smallEdges.Add(edge3);
                                }
                                for (var itlf = new NCollection_List<Shape>.Iterator(theList); itlf.More(); itlf.Next())
                                {
                                    Shape curface = itlf.Value();
                                    if (faceWithSmall.ContainsKey(curface))
                                    {
                                        NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                        if (newsmall)
                                        {
                                            theEdges.Append(edge3);
                                        }
                                        var ite = new NCollection_List<Shape>.Iterator(theEdges);
                                        while (ite.More())
                                        {
                                            Shape iedge = ite.Value();
                                            if (iedge.IsSame(edge1) || iedge.IsSame(edge2))
                                            {
                                                theEdges.Remove(ite);
                                            }
                                            else
                                            {
                                                ite.Next();
                                            }
                                        }
                                        // Remove face without small edges from the map
                                        if (theEdges.Extent() == 0)
                                        {
                                            faceWithSmall.Remove(curface);
                                        }
                                    }
                                }
                                _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                            }
                        }
                        else if (aModeDrop)
                        { // gka
                            var tempWire = new ExtendWireData();
                            var tempSaw  = new WireAnalysis();
                            tempWire.Add(sfw.InternalWire());
                            Edge remedge;
                            if (take_next)
                            {
                                remedge = edge1;
                            }
                            else
                            {
                                remedge = edge2;
                            }
                            tempWire.Remove(index);
                            tempSaw.Load(tempWire);
                            int newindex = (index <= tempSaw.EdgeCount ? index : 1);
                            tempSaw.CheckConnected(newindex, Precision());
                            if (!tempSaw.LastCheckStatus(ExtendStatus.Fail))
                            {
                                sfw.WireData()!.Remove(index);
                                Edge tmpedge1 = tempWire.Edge(newindex);
                                Edge tmpedge2 = tempWire.Edge(newindex == 1 ? tempSaw.EdgeCount : (newindex - 1));
                                var         aL1      = new NCollection_List<Shape>();
                                if (edgeToFaces.ContainsKey(tmpedge1))
                                {
                                    aL1 = new NCollection_List<Shape>(edgeToFaces.GetValue(tmpedge1));
                                }
                                var l2 = new NCollection_List<Shape>();
                                if (edgeToFaces.ContainsKey(tmpedge2))
                                {
                                    l2 = new NCollection_List<Shape>(edgeToFaces.GetValue(tmpedge2));
                                }
                                sfw.FixConnected(newindex <= sfw.NbEdges() ? newindex : 1, Precision());
                                sfw.FixDegenerated(newindex <= sfw.NbEdges() ? newindex : 1);
                                Shape aTmpShape = context.InternalApply(tmpedge1); // for porting
                                Edge  anewedge1 = ShapeCasts.Edge(aTmpShape);
                                aTmpShape = context.InternalApply(tmpedge2);
                                Edge anewedge2 = ShapeCasts.Edge(aTmpShape);
                                context.Remove(remedge);
                                SendWarning(remedge, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                if (smallEdges.Contains(remedge))
                                {
                                    smallEdges.Remove(remedge);
                                }
                                edgeToFaces.Remove(remedge);
                                edgeToFaces.Remove(tmpedge1);
                                edgeToFaces.Remove(tmpedge2);
                                edgeToFaces[anewedge1] = aL1;
                                edgeToFaces[anewedge2] = l2;
                                if (smallEdges.Contains(tmpedge1))
                                {
                                    smallEdges.Remove(tmpedge1);
                                    smallEdges.Add(anewedge1);
                                    for (var itlf = new NCollection_List<Shape>.Iterator(aL1); itlf.More(); itlf.Next())
                                    {
                                        Shape                   curface  = itlf.Value();
                                        NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                        var                            ite      = new NCollection_List<Shape>.Iterator(theEdges);
                                        while (ite.More())
                                        {
                                            Shape iedge = ite.Value();
                                            if (iedge.IsSame(tmpedge1))
                                            {
                                                theEdges.Remove(ite);
                                                theEdges.Append(anewedge1);
                                            }
                                            else
                                            {
                                                ite.Next();
                                            }
                                        }
                                    }
                                }
                                if (smallEdges.Contains(tmpedge2))
                                {
                                    smallEdges.Remove(tmpedge2);
                                    smallEdges.Add(anewedge2);
                                    for (var itlf = new NCollection_List<Shape>.Iterator(l2); itlf.More(); itlf.Next())
                                    {
                                        Shape                   curface  = itlf.Value();
                                        NCollection_List<Shape> theEdges = faceWithSmall.GetValue(curface);
                                        var                            ite      = new NCollection_List<Shape>.Iterator(theEdges);
                                        while (ite.More())
                                        {
                                            Shape iedge = ite.Value();
                                            if (iedge.IsSame(tmpedge2))
                                            {
                                                theEdges.Remove(ite);
                                                theEdges.Append(anewedge2);
                                            }
                                            else
                                            {
                                                ite.Next();
                                            }
                                        }
                                    }
                                }
                                _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                            }
                            else
                            {
                                index++;
                            }
                        }
                        else
                        {
                            // gka protection against removing circles
                            Edge ed  = (take_next ? edge1 : edge2);
                            var         sae = new EdgeAnalysis();
                            if (sae.Curve3d(ed, out Curve3D? c3d, out double f1, out double l1, false))
                            {
                                c3d!.D0(f1, out Point3D p1);
                                c3d.D0(l1, out Point3D p2);
                                c3d.D0((f1 + l1) * 0.5, out Point3D p3);

                                if (p1.DistanceTo(p3) > p1.DistanceTo(p2))
                                {
                                    index++;
                                    continue;
                                }
                            }
                            if (take_next && theList2.Extent() == 1)
                            { // gka
                                Vertex v1 = SAE.FirstVertex(edge1), v2 = SAE.LastVertex(edge1);
                                if (v1.IsSame(v2))
                                {
                                    sfw.WireData()!.Remove(index);
                                    context.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    if (smallEdges.Contains(edge1))
                                    {
                                        smallEdges.Remove(edge1);
                                    }
                                    edgeToFaces.Remove(edge1);
                                    _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                }
                                else
                                {
                                    index++;
                                }
                            }
                            else if (!take_next && theList2.Extent() == 1)
                            {
                                Vertex v1 = SAE.FirstVertex(edge2), v2 = SAE.LastVertex(edge2);
                                if (v1.IsSame(v2))
                                {
                                    sfw.WireData()!.Remove(index);
                                    context.Remove(edge2);
                                    SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    if (smallEdges.Contains(edge2))
                                    {
                                        smallEdges.Remove(edge2);
                                    }
                                    edgeToFaces.Remove(edge2);
                                    _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                }
                                else
                                {
                                    index++;
                                }
                            }
                            else
                            {
                                index++;
                            }
                        }
                    }
                    else
                    {
                        index++;
                    }
                }
                if (sfw.NbEdges() == 1 && aModeDrop)
                {
                    edge1 = sfw.WireData()!.Edge(1);
                    if (smallEdges.Contains(edge1))
                    {
                        sfw.WireData()!.Remove(1);
                        context.Remove(edge1);
                        SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                        smallEdges.Remove(edge1);
                        edgeToFaces.Remove(edge1);
                        context.Remove(wire);
                        SendWarning(wire, new Msg("FixWireframe.FixSmallEdges.MSG1"));
                        _statusSmallEdges |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                    }
                }
                else
                {
                    sfw.FixConnected();
                    context.Replace(wire, sfw.InternalWire());
                }
            }
            // end enk block
            // Record vertex replacements in context
            foreach (KeyValuePair<Shape, Shape> itv in theNewVertices)
            {
                context.Replace(itv.Key, itv.Value);
            }

            Shape shape = _shape;
            _shape.Nullify();
            _shape = context.InternalApply(shape);

            ShapeFixFixer.SameParameter(_shape, false);

            return StatusSmallEdges(ExtendStatus.Done);
        }

        return false;
    }

    /// <summary>Decodes the status of the last <see cref="FixWireGaps"/>.</summary>
    public bool StatusWireGaps(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusWireGaps, status);

    /// <summary>Decodes the status of the last <see cref="FixSmallEdges"/>.</summary>
    public bool StatusSmallEdges(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusSmallEdges, status);

    /// <summary>
    /// Returns the resulting shape, frozen: new TShapes for what the fixes changed and the caller's own sub-shapes for
    /// the rest; the context records the ones changed in place.
    /// </summary>
    public Shape Shape()
    {
        EndWork();
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The resulting shape, unfrozen, for ported callers fixing a shape they own.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>The mode of dropping small edges that can't be merged (false by default).</summary>
    public ref bool ModeDropSmallEdges() => ref _modeDrop;

    /// <summary>Sets the limit angle between two edges above which a small edge isn't merged (-1: no limit).</summary>
    public void SetLimitAngle(double limitAngle)
    {
        _limitAngle = limitAngle;
    }

    /// <summary>The limit angle (-1: no limit).</summary>
    public double LimitAngle() => _limitAngle;

    // Kerf: the first fix after a load or a public result takes the overlay the fixes work in; each fix enters it.
    private WorkingOverlay.Scope EnterWork()
    {
        if (_toEnter)
        {
            _toEnter     = false;
            _overlay     = null;
            _ownsOverlay = false;
            if (!_shape.IsNull)
            {
                if (Context() is null)
                {
                    // OCCT makes the context in the fix; it is made here so that the overlay records into it
                    SetContext(new ReShapeBuilder());
                    _isNewContext = true;
                }
                _overlay = WorkingOverlay.ForTool(_shape, Context(), out _ownsOverlay);
            }
        }
        return _overlay?.Enter() ?? default;
    }

    // Kerf: before a public result, finishes this tool's overlay.
    private void EndWork()
    {
        _toEnter = true;
        WorkingOverlay? overlay = _ownsOverlay ? _overlay : null;
        _overlay     = null;
        _ownsOverlay = false;
        if (overlay is not null)
        {
            _shape = overlay.Finish(_shape);
        }
    }
}
