// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wireframe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Fixes the wireframe of a shape: the gaps between the edges of its wires (in 3d and on the faces) and its small
/// edges (merged with a neighbour, or dropped with <see cref="ModeDropSmallEdges"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Load"/> only records the shape. The fixes edit the shape they work on in
/// place, as OCCT's: a shape holding frozen shapes (any shape a caller holds) is copied at the first fix
/// (<see cref="CopyAtEntry"/>, decision 583), so the caller's shape never changes; set the context before the
/// first fix. <see cref="Shape"/> ends the work: the copies the fixes left unchanged are the caller's own again, in the
/// result and in the context (which answers for the caller's shapes), and the result is frozen. A fix after
/// <see cref="Shape"/> copies again.
/// </remarks>
public class ShapeFix_Wireframe : ShapeFix_Root
{
    /// <summary>The shape fixed (OCCT's own field, hiding the root's).</summary>
    protected new Shape myShape;

    private bool   myModeDrop;
    private double myLimitAngle;
    private int    myStatusWireGaps;
    private int    myStatusSmallEdges;

    // The copy at the entry: from the first fix after a load (or a public result) to the next public result.
    private CopyAtEntry? myEntry;
    private bool                   myToEnter = true;
    // True when the first fix made the context (OCCT makes it in the fix, and then skips applying it).
    private bool                   myIsNewContext;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_Wireframe()
    {
        myShape = new Shape();
        ClearStatuses();
        myModeDrop   = false;
        myLimitAngle = -1;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeFix_Wireframe(in Shape shape)
    {
        ClearStatuses();
        myShape      = shape;
        myModeDrop   = false;
        myLimitAngle = -1;
    }

    /// <summary>Clears the statuses.</summary>
    public virtual void ClearStatuses()
    {
        int emptyStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        myStatusWireGaps   = emptyStatus;
        myStatusSmallEdges = emptyStatus;
    }

    /// <summary>Loads a shape, resets the statuses.</summary>
    public void Load(in Shape shape)
    {
        ClearStatuses();
        myShape   = shape;
        myEntry   = null;
        myToEnter = true;
    }

    /// <summary>
    /// Fixes the gaps between the edges of the wires (ShapeFix_Wire's FixGaps3d and, on faces, FixGaps2d), then makes the
    /// edges SameParameter and fixes the self-intersections and vertex tolerances of the wires. Returns true if a gap
    /// was fixed.
    /// </summary>
    public bool FixWireGaps()
    {
        enterWork();
        return fixWireGaps();
    }

    private bool fixWireGaps()
    {
        myStatusWireGaps = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (myShape.IsNull)
        {
            return false;
        }

        if (Context() is null || myIsNewContext)
        {
            myIsNewContext = false;
            if (Context() is null)
            {
                SetContext(new ShapeBuild_ReShape());
            }
        }
        else
        {
            Shape shape = myShape;
            myShape.Nullify();
            myShape = Context()!.InternalApply(shape);
        }
        ShapeBuild_ReShape aContext = Context()!;

        double prec = (Precision() > 0.0) ? Precision() : Kerf.Engine.Foundation.Precision.Confusion();
        var    cont = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        if (myShape.ShapeType() == ShapeType.Compound)
        {
            bool locModified = false;
            var  C           = new Compound();
            var  B           = new BrepBuilder();
            B.MakeCompound(ref C);
            Shape savShape = myShape;
            for (var it = new Iterator(savShape); it.More(); it.Next())
            {
                Shape    shape1 = it.Value();
                Placement L = shape1.Location(), nullLoc = new();
                shape1.Location(nullLoc);
                Shape res;
                if (cont.IsBound(shape1))
                {
                    res = cont.Find(shape1).Oriented(shape1.Orientation());
                }
                else
                {
                    myShape = shape1;
                    fixWireGaps();
                    res = myShape;
                    cont.Bind(myShape, res);
                }
                if (!res.IsSame(shape1))
                {
                    locModified = true;
                }
                res.Location(L);
                B.Add(C, res);
            }
            if (locModified)
            {
                C.Orientation(savShape.Orientation());
                aContext.Replace(savShape, C);
            }
            myShape = aContext.InternalApply(savShape);
            return StatusWireGaps(ShapeExtend_DONE);
        }
        var sfw = new ShapeFix_Wire();
        sfw.SetContext(aContext);
        sfw.SetPrecision(prec);

        Face face;
        for (var anExpf1 = new Explorer(myShape, ShapeType.Face); anExpf1.More(); anExpf1.Next())
        {
            // smh#8
            Shape tmpF = aContext.InternalApply(anExpf1.Current());
            face              = ShapeCasts.Face(tmpF);
            if (face.Orientation() == Reversed)
            {
                face.Orientation(Forward);
            }
            for (var itw = new Iterator(face); itw.More(); itw.Next())
            {
                if (itw.Value().ShapeType() != ShapeType.Wire)
                {
                    continue;
                }
                // smh#8
                Shape tmpW = aContext.InternalApply(itw.Value());
                sfw.Init(ShapeCasts.Wire(tmpW), face, prec);
                sfw.FixReorder();
                sfw.FixGaps3d();
                if (sfw.StatusGaps3d(ShapeExtend_DONE))
                {
                    myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                if (sfw.StatusGaps3d(ShapeExtend_FAIL))
                {
                    myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
                sfw.FixGaps2d();
                if (sfw.StatusGaps2d(ShapeExtend_DONE))
                {
                    myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
                if (sfw.StatusGaps2d(ShapeExtend_FAIL))
                {
                    myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
                if (sfw.StatusGaps3d(ShapeExtend_DONE) || sfw.StatusGaps2d(ShapeExtend_DONE))
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
        for (var expw = new Explorer(myShape, ShapeType.Wire, ShapeType.Face); expw.More(); expw.Next())
        {
            Shape tmpW = aContext.InternalApply(expw.Current());
            sfw.Load(ShapeCasts.Wire(tmpW));
            sfw.SetPrecision(prec);
            sfw.FixReorder();
            sfw.FixGaps3d();
            if (sfw.StatusGaps3d(ShapeExtend_DONE))
            {
                myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
            if (sfw.StatusGaps3d(ShapeExtend_FAIL))
            {
                myStatusWireGaps |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }
            if (sfw.StatusGaps3d(ShapeExtend_DONE))
            {
                SendWarning(expw.Current(), new Msg("FixWireframe.FixFixWireGaps.MSG0"));
            }
        }
        // End Part1========================================================

        if (StatusWireGaps(ShapeExtend_DONE))
        {
            myShape = aContext.InternalApply(myShape);

            ShapeFix.SameParameter(myShape, false);

            Wire wire;
            var         sfe = new ShapeFix_Edge();
            for (var anExpf2 = new Explorer(myShape, ShapeType.Face); anExpf2.More(); anExpf2.Next())
            {
                face = ShapeCasts.Face(anExpf2.Current());
                if (face.Orientation() == Reversed)
                {
                    face.Orientation(Forward);
                }
                for (var itw = new Iterator(face); itw.More(); itw.Next())
                {
                    if (itw.Value().ShapeType() != ShapeType.Wire)
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
            for (var expw2 = new Explorer(myShape, ShapeType.Wire, ShapeType.Face); expw2.More(); expw2.Next())
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
    private static bool JoinEdges(in Edge E1, in Edge E2, out Edge E3, NCollection_List<Shape> faces)
    {
        bool ReplaceFirst = true;
        var  sae          = new ShapeAnalysis_Edge();
        var  newedge      = new Edge();
        var  newedge1     = new Edge();
        E3 = newedge1;
        Vertex V11 = sae.FirstVertex(E1);
        Vertex V12 = sae.LastVertex(E1);
        Vertex V21 = sae.FirstVertex(E2);
        Vertex V22 = sae.LastVertex(E2);

        bool isSame = (V11.IsSame(V12) || V22.IsSame(V21));
        var  B      = new BrepBuilder();
        B.MakeEdge(ref newedge);
        double first1, first2, last1, last2;
        newedge.Orientation(Forward);
        try
        {
            if (!sae.Curve3d(E1, out Curve3D? c3d1, out double cf1, out double cl1, false))
            {
                return ReplaceFirst;
            }
            if (!sae.Curve3d(E2, out Curve3D? c3d2, out double cf2, out double cl2, false))
            {
                return ReplaceFirst;
            }

            B.Add(newedge, V11.Oriented(Forward));
            B.Add(newedge, V22.Oriented(Reversed));

            //     double newf,newl;
            if (!ShapeConstruct.InternalJoinCurves(c3d1!, c3d2!, E1.Orientation(), E2.Orientation(), ref cf1, ref cl1, ref cf2,
                                                   ref cl2, out Curve3D? CRes, out bool isRev1, out bool isRev2))
            {
                return ReplaceFirst;
            }
            //    if(isRev1 || isRev2)
            if (!isSame && (isRev1 || isRev2))
            {
                return ReplaceFirst;
            }
            ReplaceFirst = (!isRev1);

            double             newf        = cf1;
            double             newl        = cl1 + cl2 - cf2;
            Orientation OrEdge1     = E1.Orientation();
            Orientation OrEdge2     = E2.Orientation();
            bool               ismanifold  = (OrEdge1 == Forward || OrEdge1 == Reversed);
            bool               ismanifold2 = (OrEdge2 == Forward || OrEdge2 == Reversed);
            if (ismanifold != ismanifold2)
            {
                return ReplaceFirst;
            }

            if (ismanifold)
            {
                OrEdge1 = ((!isRev1 && E1.Orientation() == Forward) || (isRev1 && E1.Orientation() == Reversed)
                               ? Forward
                               : Reversed);
                OrEdge2 = ((!isRev2 && E2.Orientation() == Forward) || (isRev2 && E2.Orientation() == Reversed)
                               ? Forward
                               : Reversed);
            }
            B.UpdateEdge(newedge, CRes, Math.Max(Tool.Tolerance(E1), Tool.Tolerance(E2)));
            double fp = CRes!.FirstParameter();
            double lp = CRes.LastParameter();
            if (fp > newf)
            {
                newf = fp;
            }
            if (lp < newl)
            {
                newl = lp;
            }
            B.Range(newedge, newf, newl);

            // merging pcurves
            for (var iter = new NCollection_List<Shape>.Iterator(faces); iter.More(); iter.Next())
            {
                Face face = ShapeCasts.Face(iter.Value());
                if (!sae.PCurve(E1, face, out Curve2D? c2d1, out first1, out last1, false))
                {
                    return ReplaceFirst;
                }
                if (!sae.PCurve(E2, face, out Curve2D? c2d2, out first2, out last2, false))
                {
                    return ReplaceFirst;
                }

                if (!ShapeConstruct.InternalJoinCurves(c2d1!, c2d2!, OrEdge1, OrEdge2, ref first1, ref last1, ref first2,
                                                       ref last2, out Curve2D? C2dRes, out bool isRev12, out bool isRev22,
                                                       isSame))
                {
                    return ReplaceFirst;
                }

                if (ismanifold && (!isSame && (isRev12 || isRev22)))
                {
                    return ReplaceFirst;
                }

                if (isRev12)
                {
                    ReplaceFirst = false;
                }
                double fp2d = C2dRes!.FirstParameter();
                double lp2d = C2dRes.LastParameter();
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
                if (sae.IsSeam(E1, face) && sae.IsSeam(E2, face))
                {
                    // smh#8
                    Shape tmpE1 = E1.Reversed(), tmpE2 = E2.Reversed();
                    Edge  E1t   = ShapeCasts.Edge(tmpE1);
                    Edge  E2t   = ShapeCasts.Edge(tmpE2);
                    sae.PCurve(E1t, face, out Curve2D? c2d12, out first1, out last1, false);
                    sae.PCurve(E2t, face, out Curve2D? c2d22, out first2, out last2, false);

                    if (!ShapeConstruct.InternalJoinCurves(c2d12!, c2d22!, OrEdge1, OrEdge2, ref first1, ref last1, ref first2,
                                                           ref last2, out Curve2D? C2dRes2, out isRev12, out isRev22, isSame))
                    {
                        return ReplaceFirst;
                    }
                    if (!isSame && (isRev1 || isRev2))
                    {
                        return ReplaceFirst;
                    }
                    B.UpdateEdge(newedge, C2dRes, C2dRes2, face, 0);
                }
                else if (sae.IsSeam(E1, face) || sae.IsSeam(E2, face))
                {
                    return ReplaceFirst;
                }
                else if (!sae.IsSeam(E1, face) && !sae.IsSeam(E2, face))
                {
                    B.UpdateEdge(newedge, C2dRes, face, 0);
                }
                B.Range(newedge, face, newf1, newl1);
                if (!ismanifold)
                {
                    newedge.Orientation(ReplaceFirst ? OrEdge1 : OrEdge2);
                }
            }
            B.SameRange(newedge, false);

            E3 = newedge;
            return ReplaceFirst;
        }
        catch (FailureException)
        {
            return ReplaceFirst;
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
        enterWork();
        return fixSmallEdges();
    }

    private bool fixSmallEdges()
    {
        myStatusSmallEdges = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (myShape.IsNull)
        {
            return false;
        }

        if (Context() is null || myIsNewContext)
        {
            myIsNewContext = false;
            if (Context() is null)
            {
                SetContext(new ShapeBuild_ReShape());
            }
        }
        else
        {
            Shape shape = myShape;
            myShape.Nullify();
            myShape = Context()!.InternalApply(shape);
        }
        ShapeBuild_ReShape aContext = Context()!;
        var                cont     = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        if (myShape.ShapeType() == ShapeType.Compound)
        {
            bool locModified = false;
            var  C           = new Compound();
            var  B           = new BrepBuilder();
            B.MakeCompound(ref C);
            Shape savShape = myShape;
            for (var it = new Iterator(savShape); it.More(); it.Next())
            {
                Shape    shape1 = it.Value();
                Placement L = shape1.Location(), nullLoc = new();
                shape1.Location(nullLoc);
                Shape res;
                if (cont.IsBound(shape1))
                {
                    res = cont.Find(shape1).Oriented(shape1.Orientation());
                }
                else
                {
                    myShape = shape1;
                    fixSmallEdges();
                    res = myShape;
                    cont.Bind(myShape, res);
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

                res.Location(L);
                B.Add(C, res);
            }
            if (locModified)
            {
                C.Orientation(savShape.Orientation());
                aContext.Replace(savShape, C);
            }
            myShape = aContext.InternalApply(savShape);
            return StatusSmallEdges(ShapeExtend_DONE);
        }
        var theSmallEdges    = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var theMultyEdges    = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var theEdgeToFaces   = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var theFaceWithSmall = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        checkSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
        mergeSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
        return StatusSmallEdges(ShapeExtend_DONE);
    }

    //=================================================================================================

    /// <summary>
    /// Finds the small edges of the shape, the faces of each edge, the faces holding small edges and the edges met
    /// more than once in a wire (not seams). Returns true if a small edge was found.
    /// </summary>
    public bool CheckSmallEdges(NCollection_Map<Shape> theSmallEdges,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgeToFaces,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theFaceWithSmall,
                                NCollection_Map<Shape> theMultyEdges)
    {
        enterWork();
        return checkSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
    }

    private bool checkSmallEdges(NCollection_Map<Shape> theSmallEdges,
                                 NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgeToFaces,
                                 NCollection_DataMap<Shape, NCollection_List<Shape>> theFaceWithSmall,
                                 NCollection_Map<Shape> theMultyEdges)
    {
        Face face;
        Edge edge;
        var         SAW = new ShapeAnalysis_Wire();

        for (var anExpf1 = new Explorer(myShape, ShapeType.Face); anExpf1.More(); anExpf1.Next())
        {
            var         theEdgeList = new NCollection_List<Shape>();
            Face facet       = ShapeCasts.Face(anExpf1.Current());
            face = facet;
            if (facet.Orientation() == Reversed)
            {
                face = ShapeCasts.Face(facet.Oriented(Forward));
            }
            for (var itw = new Iterator(face); itw.More(); itw.Next())
            {
                if (itw.Value().ShapeType() != ShapeType.Wire)
                {
                    continue;
                }
                Wire aW   = ShapeCasts.Wire(itw.Value());
                var         aswd = new ShapeExtend_WireData(aW, true, false);
                SAW.Init(aswd, face, Precision());
                // pnd protection on seam edges
                var EdgeMap = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
                int i;
                for (i = 1; i <= SAW.NbEdges(); i++)
                {
                    edge = SAW.WireData()!.Edge(i);
                    if (EdgeMap.IsBound(edge))
                    {
                        EdgeMap.ChangeFind(edge)++;
                    }
                    else
                    {
                        EdgeMap.Bind(edge, 1);
                    }
                }

                for (i = 1; i <= SAW.NbEdges(); i++)
                {
                    edge = SAW.WireData()!.Edge(i);
                    if (EdgeMap.Find(edge) != 1)
                    {
                        if (!SAW.WireData()!.IsSeam(i))
                        {
                            theMultyEdges.Add(edge);
                        }
                        continue;
                    }
                    // Append current face to the list
                    if (theEdgeToFaces.IsBound(edge))
                    {
                        theEdgeToFaces[edge].Append(facet);
                    }
                    else
                    {
                        var theFaceList = new NCollection_List<Shape>();
                        theFaceList.Append(facet);
                        theEdgeToFaces.Bind(edge, theFaceList);
                    }
                    // Check if current edge is small
                    if (theSmallEdges.Contains(edge))
                    {
                        theEdgeList.Append(edge);
                    }
                    else if (SAW.CheckSmall(i, Precision()))
                    {
                        theSmallEdges.Add(edge);
                        theEdgeList.Append(edge);
                    }
                }
            }
            // Add current face to the map if has small edges
            if (theEdgeList.Extent() != 0)
            {
                theFaceWithSmall.Bind(facet, theEdgeList);
            }
        }

        //=========================================================================
        // Author : enk
        // Purpose: Analyzing of shape for small edges, if edge doesn't lie on face
        //=========================================================================
        for (var expw1 = new Explorer(myShape, ShapeType.Wire, ShapeType.Face); expw1.More(); expw1.Next())
        {
            SAW.SetPrecision(Precision());
            var         EdgeMap = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
            int         i;
            Wire theWire     = ShapeCasts.Wire(expw1.Current());
            var         theEdgeList = new NCollection_List<Shape>();
            SAW.Load(theWire);
            if (!SAW.IsLoaded())
            {
                return false;
            }
            for (i = 1; i <= SAW.NbEdges(); i++)
            {
                edge = SAW.WireData()!.Edge(i);
                if (EdgeMap.IsBound(edge))
                {
                    EdgeMap.ChangeFind(edge)++;
                }
                else
                {
                    EdgeMap.Bind(edge, 1);
                }
            }

            for (i = 1; i <= SAW.NbEdges(); i++)
            {
                edge = SAW.WireData()!.Edge(i);
                if (EdgeMap.Find(edge) != 1)
                {
                    if (!SAW.WireData()!.IsSeam(i))
                    {
                        theMultyEdges.Add(edge);
                    }
                    continue;
                }

                // Check if current edge is small
                if (theSmallEdges.Contains(edge))
                {
                    theEdgeList.Append(edge);
                }
                else if (SAW.CheckSmall(i, Precision()))
                {
                    theSmallEdges.Add(edge);
                    theEdgeList.Append(edge);
                }
            }
        }
        return (!theSmallEdges.IsEmpty());
    }

    //=================================================================================================

    /// <summary>
    /// Merges (or drops) the small edges found by <see cref="CheckSmallEdges"/>, as <see cref="FixSmallEdges"/>
    /// describes; <paramref name="theModeDrop"/> and <paramref name="theLimitAngle"/> add to the tool's own settings.
    /// Returns true if something was done.
    /// </summary>
    public bool MergeSmallEdges(NCollection_Map<Shape> theSmallEdges,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgeToFaces,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theFaceWithSmall,
                                NCollection_Map<Shape> theMultyEdges,
                                bool theModeDrop = false,
                                double theLimitAngle = -1)
    {
        enterWork();
        return mergeSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges, theModeDrop, theLimitAngle);
    }

    private bool mergeSmallEdges(NCollection_Map<Shape> theSmallEdges,
                                 NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgeToFaces,
                                 NCollection_DataMap<Shape, NCollection_List<Shape>> theFaceWithSmall,
                                 NCollection_Map<Shape> theMultyEdges,
                                 bool theModeDrop = false,
                                 double theLimitAngle = -1)
    {
        bool   aModLimitAngle = (theLimitAngle > -1.0 || myLimitAngle > -1.0);
        double aLimitAngle    = Math.Max(theLimitAngle, myLimitAngle);

        bool aModeDrop      = theModeDrop || myModeDrop;
        var  theNewVertices = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        if (!theSmallEdges.IsEmpty())
        {
            ShapeBuild_ReShape aContext = Context()!;
            var                SFW      = new ShapeFix_Wire();
            SFW.SetContext(aContext);
            var         SAE = new ShapeAnalysis_Edge();
            Edge edge1, edge2, edge3;
            // Iterate on map of faces with small edges
            var anExpf2 = new Explorer(myShape, ShapeType.Face);
            for (; anExpf2.More(); anExpf2.Next())
            {
                if (theFaceWithSmall.IsBound(anExpf2.Current()))
                {
                    if (theFaceWithSmall[anExpf2.Current()].Extent() != 0)
                    {
                        // smh#8
                        Shape tmpShape = aContext.InternalApply(anExpf2.Current());
                        Face  facet    = ShapeCasts.Face(tmpShape);
                        if (!facet.IsSame(anExpf2.Current()))
                        { // gka
                            var aExpEdge = new Explorer(anExpf2.Current(), ShapeType.Edge);
                            for (; aExpEdge.More(); aExpEdge.Next())
                            {
                                int stat = aContext.InternalStatus(aExpEdge.Current(), out Shape newEdge, true);
                                if (stat > 0)
                                {
                                    if (theSmallEdges.Contains(aExpEdge.Current()))
                                    {
                                        theSmallEdges.Remove(aExpEdge.Current());
                                        theSmallEdges.Add(newEdge);
                                    }
                                    if (theEdgeToFaces.IsBound(aExpEdge.Current()))
                                    {
                                        NCollection_List<Shape> aListFaces = theEdgeToFaces.Find(aExpEdge.Current());
                                        theEdgeToFaces.UnBind(aExpEdge.Current());
                                        theEdgeToFaces.Bind(newEdge, aListFaces);
                                    }
                                }
                            }
                        }
                        Face face = facet;
                        // if (face.Orientation()==TopAbs_REVERSED)
                        //   face = TopoDS::Face(facet.Oriented(TopAbs_FORWARD));
                        for (var itw = new Iterator(face); itw.More(); itw.Next())
                        {
                            if (itw.Value().ShapeType() != ShapeType.Wire)
                            {
                                continue;
                            }
                            Wire aWire = ShapeCasts.Wire(itw.Value());
                            if (face.Orientation() == Reversed)
                            {
                                face = ShapeCasts.Face(facet.Oriented(Forward));
                            }
                            var aswd = new ShapeExtend_WireData(aWire, true, false);
                            // SFW->Load(aWire);
                            SFW.Load(aswd);
                            SFW.FixReorder();
                            int prev, next, index = 1;

                            while (index <= SFW.WireData()!.NbEdges() && SFW.NbEdges() > 1)
                            {
                                prev  = (index == 1) ? SFW.WireData()!.NbEdges() : index - 1;
                                next  = (index == SFW.WireData()!.NbEdges()) ? 1 : index + 1;
                                edge1 = SFW.WireData()!.Edge(prev);
                                edge2 = SFW.WireData()!.Edge(index);
                                edge3 = SFW.WireData()!.Edge(next);

                                // gka protection against joining seem edge
                                if (edge2.IsSame(edge1) || edge2.IsSame(edge3))
                                {
                                    // if(BRep_Tool::IsClosed(edge2,face)) {
                                    index++;
                                    continue;
                                }

                                bool isSeam  = SFW.WireData()!.IsSeam(index);
                                bool isSeam1 = SFW.WireData()!.IsSeam(prev);
                                bool isSeam2 = SFW.WireData()!.IsSeam(next);
                                if (theSmallEdges.Contains(edge2))
                                {
                                    // Middle edge is small - choose a pair of edges to join
                                    bool   IsAnyJoin  = (edge1.IsSame(edge3));
                                    bool   take_next  = IsAnyJoin; // false;
                                    bool   isLimAngle = false;
                                    double Ang1 = 0.0, Ang2 = 0.0;
                                    if (SAE.Curve3d(edge1, out Curve3D? C1, out _, out double last1)
                                        && SAE.Curve3d(edge2, out Curve3D? C2, out double first2, out double last2)
                                        && SAE.Curve3d(edge3, out Curve3D? C3, out double first3, out _))
                                    {
                                        // Compare angles between edges
                                        // double Ang1, Ang2;
                                        C1!.D1(last1, out _, out Vector3D Vec1);
                                        C2!.D1(first2, out _, out Vector3D Vec2);
                                        if (edge1.Orientation() == Reversed)
                                        {
                                            Vec1.Reverse();
                                        }
                                        if (edge2.Orientation() == Reversed)
                                        {
                                            Vec2.Reverse();
                                        }
                                        double tol2 = Kerf.Engine.Foundation.Precision.SquareConfusion();
                                        if (Vec1.SquareMagnitude() < tol2 || Vec2.SquareMagnitude() < tol2)
                                        {
                                            Ang1 = Math.PI / 2.0;
                                        }
                                        else
                                        {
                                            Ang1 = Math.Abs(Vec1.Angle(Vec2));
                                        }
                                        C2.D1(last2, out _, out Vec1);
                                        C3!.D1(first3, out _, out Vec2);
                                        if (edge2.Orientation() == Reversed)
                                        {
                                            Vec1.Reverse();
                                        }
                                        if (edge3.Orientation() == Reversed)
                                        {
                                            Vec2.Reverse();
                                        }
                                        if (Vec1.SquareMagnitude() < tol2 || Vec2.SquareMagnitude() < tol2)
                                        {
                                            Ang2 = Math.PI / 2.0;
                                        }
                                        else
                                        {
                                            Ang2 = Math.Abs(Vec1.Angle(Vec2));
                                        }
                                        // isLimAngle = (theLimitAngle != -1 && std::min(Ang1,Ang2) > theLimitAngle);
                                        // take_next = (Ang2<Ang1);
                                        // if (take_next) { edge1 = edge2; edge2 = edge3; }
                                    }
                                    // if(theLimitAngle != -1 && Ang1 > theLimitAngle && Ang2 >theLimitAngle) {
                                    //   index++; continue;
                                    // }

                                    // Check if edges lay on the same faces
                                    if (theMultyEdges.Contains(edge1) || theMultyEdges.Contains(edge2))
                                    { //??????
                                        index++;
                                        continue;
                                    }
                                    var theList1 = new NCollection_List<Shape>();
                                    var theList2 = new NCollection_List<Shape>();
                                    var theList3 = new NCollection_List<Shape>();
                                    if (theEdgeToFaces.IsBound(edge1))
                                    {
                                        theList1 = new NCollection_List<Shape>(theEdgeToFaces[edge1]);
                                    }
                                    if (theEdgeToFaces.IsBound(edge2))
                                    {
                                        theList2 = new NCollection_List<Shape>(theEdgeToFaces[edge2]);
                                    }
                                    if (theEdgeToFaces.IsBound(edge3))
                                    {
                                        theList3 = new NCollection_List<Shape>(theEdgeToFaces[edge3]);
                                    }
                                    bool same_set = false;

                                    // gka protection against joining seem edges with other edges
                                    bool same_set1 = (theList1.Extent() == theList2.Extent()
                                                      && ((!isSeam && !isSeam1) || (isSeam && isSeam1))); // gka
                                    bool same_set2 = (theList3.Extent() == theList2.Extent()
                                                      && ((!isSeam && !isSeam2) || (isSeam && isSeam2)));
                                    var theSetOfFaces = new NCollection_Map<Shape>(ShapeHasher.Instance);
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
                                        if (Math.Abs(Ang2 - Ang1) > Kerf.Engine.Foundation.Precision.Angular())
                                        {
                                            take_next = (Ang2 < Ang1);
                                        }
                                        if (take_next)
                                        {
                                            edge1 = edge2;
                                            edge2 = edge3;
                                        }
                                        isLimAngle = (aModLimitAngle && Math.Min(Ang1, Ang2) > aLimitAngle);
                                    }
                                    else if (same_set1 && !same_set2)
                                    {
                                        isLimAngle = (aModLimitAngle && Ang1 > aLimitAngle);
                                        same_set   = true;
                                    }
                                    else if (!same_set1 && same_set2)
                                    {
                                        same_set   = true;
                                        isLimAngle = (aModLimitAngle && Ang2 > aLimitAngle);
                                        edge1      = edge2;
                                        edge2      = edge3;
                                        take_next  = true;
                                    }
                                    if (same_set && !isLimAngle)
                                    {
                                        // Merge current pair of edges
                                        // gka protection against crossing seem on second face
                                        bool isNeedJoin = true; // false;
                                        for (var aItF = new NCollection_List<Shape>.Iterator(theList2); aItF.More() && isNeedJoin;
                                             aItF.Next())
                                        {
                                            if (aItF.Value().IsSame(anExpf2.Current()))
                                            {
                                                continue;
                                            }
                                            Shape aF = aContext.InternalApply(aItF.Value());
                                            // aF = aF.Oriented(TopAbs_FORWARD);
                                            for (var aIw = new Iterator(aF); aIw.More(); aIw.Next())
                                            {
                                                if (aIw.Value().ShapeType() != ShapeType.Wire)
                                                {
                                                    continue;
                                                }
                                                Wire wt   = ShapeCasts.Wire(aIw.Value());
                                                var         SFW1 = new ShapeFix_Wire();
                                                SFW1.Load(wt);
                                                SFW1.FixReorder();
                                                ShapeExtend_WireData atmpswd = SFW1.WireData()!;
                                                int                  ind1    = atmpswd.Index(edge1);
                                                int                  ind2    = atmpswd.Index(edge2);
                                                if (ind1 != 0 && ind2 != 0)
                                                {
                                                    isNeedJoin = ((ind1 - ind2) == 1 || (ind2 == atmpswd.NbEdges() && ind1 == 1));
                                                    break;
                                                }
                                            }
                                        }
                                        bool ReplaceFirst = true;
                                        if (isNeedJoin)
                                        {
                                            var aListF = new NCollection_List<Shape>();
                                            for (var aItlF = new NCollection_List<Shape>.Iterator(theList2); aItlF.More(); aItlF.Next())
                                            {
                                                Shape tmpF = aContext.InternalApply(aItlF.Value());
                                                aListF.Append(tmpF);
                                            }
                                            ReplaceFirst = JoinEdges(edge1, edge2, out edge3, aListF);
                                        }
                                        else
                                        {
                                            edge3 = new Edge();
                                        }
                                        if (edge3.IsNull)
                                        {
                                            index++;
                                            myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                                        }
                                        else
                                        {
                                            // Record vertex replacements in the map
                                            Vertex oldV1 = SAE.FirstVertex(edge3), oldV2 = SAE.LastVertex(edge3);
                                            if (!theNewVertices.IsBound(oldV1))
                                            // smh#8
                                            {
                                                Shape emptyCopiedV1 = oldV1.EmptyCopied();
                                                theNewVertices.Bind(oldV1, ShapeCasts.Vertex(emptyCopiedV1));
                                            }
                                            if (!oldV1.IsSame(oldV2))
                                            {
                                                if (!theNewVertices.IsBound(oldV2))
                                                // smh#8
                                                {
                                                    Shape emptyCopiedV2 = oldV2.EmptyCopied();
                                                    theNewVertices.Bind(oldV2, ShapeCasts.Vertex(emptyCopiedV2));
                                                }
                                            }

                                            // To keep NM vertices belonging initial edges
                                            var aItv = new Iterator(edge1, false);
                                            for (; aItv.More(); aItv.Next())
                                            {
                                                if (aItv.Value().Orientation() == Internal
                                                    || aItv.Value().Orientation() == External)
                                                {
                                                    Vertex aOldV = ShapeCasts.Vertex(aItv.Value());
                                                    Vertex anewV =
                                                        ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, edge3, edge1);
                                                    var aB = new BrepBuilder();
                                                    aB.Add(edge3, anewV);
                                                    aContext.Replace(aOldV, anewV);
                                                }
                                            }

                                            for (aItv.Initialize(edge2, false); aItv.More(); aItv.Next())
                                            {
                                                if (aItv.Value().Orientation() == Internal
                                                    || aItv.Value().Orientation() == External)
                                                {
                                                    var           aB    = new BrepBuilder();
                                                    Vertex aOldV = ShapeCasts.Vertex(aItv.Value());
                                                    Vertex anewV =
                                                        ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, edge3, edge2);
                                                    aB.Add(edge3, anewV);
                                                    aContext.Replace(aOldV, anewV);
                                                }
                                            }

                                            // Check for small resulting edge
                                            bool newsmall = false;
                                            var  SAW      = new ShapeAnalysis_Wire();
                                            SAW.Init(SFW.WireData(), face, Precision());
                                            // Make changes in WireData and Context
                                            if (ReplaceFirst)
                                            {
                                                aContext.Replace(edge1, edge3);
                                                aContext.Remove(edge2);
                                                SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            }
                                            else
                                            {
                                                aContext.Replace(edge2, edge3);
                                                aContext.Remove(edge1);
                                                SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            }
                                            if (take_next)
                                            {
                                                SFW.WireData()!.Set(edge3, next);
                                                newsmall = SAW.CheckSmall(next, Precision());
                                            }
                                            else
                                            {
                                                SFW.WireData()!.Set(edge3, prev);
                                                newsmall = SAW.CheckSmall(prev, Precision());
                                            }
                                            SFW.WireData()!.Remove(index);
                                            // Process changes in maps
                                            var theList = new NCollection_List<Shape>();
                                            theList.Append(theList2);
                                            theEdgeToFaces.UnBind(edge1);
                                            theEdgeToFaces.UnBind(edge2);
                                            theEdgeToFaces.Bind(edge3, theList);
                                            if (theSmallEdges.Contains(edge1))
                                            {
                                                theSmallEdges.Remove(edge1);
                                            }
                                            if (theSmallEdges.Contains(edge2))
                                            {
                                                theSmallEdges.Remove(edge2);
                                            }
                                            if (newsmall)
                                            {
                                                theSmallEdges.Add(edge3);
                                            }
                                            for (var itlf = new NCollection_List<Shape>.Iterator(theList); itlf.More(); itlf.Next())
                                            {
                                                Shape curface = itlf.Value();
                                                if (theFaceWithSmall.IsBound(curface))
                                                {
                                                    NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                                        theFaceWithSmall.UnBind(curface);
                                                    }
                                                }
                                            }
                                            myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                                        }
                                    }
                                    else if (aModeDrop)
                                    { // gka
                                        var tempWire = new ShapeExtend_WireData();
                                        var tempSaw  = new ShapeAnalysis_Wire();
                                        tempWire.Add(SFW.InternalWire());
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
                                        int newindex = (index <= tempSaw.NbEdges() ? index : 1);
                                        tempSaw.CheckConnected(newindex, Precision());
                                        if (!tempSaw.LastCheckStatus(ShapeExtend_FAIL))
                                        {
                                            SFW.WireData()!.Remove(index);
                                            Edge tmpedge1 = tempWire.Edge(newindex);
                                            Edge tmpedge2 = tempWire.Edge(newindex == 1 ? tempSaw.NbEdges() : (newindex - 1));
                                            var         aL1      = new NCollection_List<Shape>();
                                            if (theEdgeToFaces.IsBound(tmpedge1))
                                            {
                                                aL1 = new NCollection_List<Shape>(theEdgeToFaces.Find(tmpedge1));
                                            }
                                            var aL2 = new NCollection_List<Shape>();
                                            if (theEdgeToFaces.IsBound(tmpedge2))
                                            {
                                                aL2 = new NCollection_List<Shape>(theEdgeToFaces.Find(tmpedge2));
                                            }
                                            SFW.FixConnected(newindex <= SFW.NbEdges() ? newindex : 1, Precision());
                                            SFW.FixDegenerated(newindex <= SFW.NbEdges() ? newindex : 1);
                                            Shape aTmpShape = aContext.InternalApply(tmpedge1); // for porting
                                            Edge  anewedge1 = ShapeCasts.Edge(aTmpShape);
                                            aTmpShape = aContext.InternalApply(tmpedge2);
                                            Edge anewedge2 = ShapeCasts.Edge(aTmpShape);
                                            aContext.Remove(remedge);
                                            SendWarning(remedge, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                            if (theSmallEdges.Contains(remedge))
                                            {
                                                theSmallEdges.Remove(remedge);
                                            }
                                            theEdgeToFaces.UnBind(remedge);
                                            theEdgeToFaces.UnBind(tmpedge1);
                                            theEdgeToFaces.UnBind(tmpedge2);
                                            theEdgeToFaces.Bind(anewedge1, aL1);
                                            theEdgeToFaces.Bind(anewedge2, aL2);
                                            if (theSmallEdges.Contains(tmpedge1))
                                            {
                                                theSmallEdges.Remove(tmpedge1);
                                                theSmallEdges.Add(anewedge1);
                                                for (var itlf = new NCollection_List<Shape>.Iterator(aL1); itlf.More(); itlf.Next())
                                                {
                                                    Shape                   curface  = itlf.Value();
                                                    NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                            if (theSmallEdges.Contains(tmpedge2))
                                            {
                                                theSmallEdges.Remove(tmpedge2);
                                                theSmallEdges.Add(anewedge2);
                                                for (var itlf = new NCollection_List<Shape>.Iterator(aL2); itlf.More(); itlf.Next())
                                                {
                                                    Shape                   curface  = itlf.Value();
                                                    NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                            myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
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
                                        var         sae = new ShapeAnalysis_Edge();
                                        if (sae.Curve3d(ed, out Curve3D? c3d, out double f1, out double l1, false))
                                        {
                                            c3d!.D0(f1, out Point3D p1);
                                            c3d.D0(l1, out Point3D p2);
                                            c3d.D0((f1 + l1) * 0.5, out Point3D p3);

                                            if (p1.Distance(p3) > p1.Distance(p2))
                                            {
                                                index++;
                                                continue;
                                            }
                                        }
                                        if (take_next && theList2.Extent() == 1)
                                        { // gka
                                            Vertex V1 = SAE.FirstVertex(edge1), V2 = SAE.LastVertex(edge1);
                                            if (V1.IsSame(V2))
                                            {
                                                SFW.WireData()!.Remove(index);
                                                aContext.Remove(edge1);
                                                SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                                if (theSmallEdges.Contains(edge1))
                                                {
                                                    theSmallEdges.Remove(edge1);
                                                }
                                                theEdgeToFaces.UnBind(edge1);
                                                myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                                            }
                                            else
                                            {
                                                index++;
                                            }
                                        }
                                        else if (!take_next && theList2.Extent() == 1)
                                        {
                                            Vertex V1 = SAE.FirstVertex(edge2), V2 = SAE.LastVertex(edge2);
                                            if (V1.IsSame(V2))
                                            {
                                                SFW.WireData()!.Remove(index);
                                                aContext.Remove(edge2);
                                                SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                                if (theSmallEdges.Contains(edge2))
                                                {
                                                    theSmallEdges.Remove(edge2);
                                                }
                                                theEdgeToFaces.UnBind(edge2);
                                                myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
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
                            if (SFW.NbEdges() == 1 && aModeDrop)
                            {
                                edge1 = SFW.WireData()!.Edge(1);
                                if (theSmallEdges.Contains(edge1))
                                {
                                    SFW.WireData()!.Remove(1);
                                    aContext.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    theSmallEdges.Remove(edge1);
                                    theEdgeToFaces.UnBind(edge1);
                                    aContext.Remove(aWire);
                                    SendWarning(aWire, new Msg("FixWireframe.FixSmallEdges.MSG1"));
                                    myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                                }
                            }
                            else
                            {
                                SFW.FixConnected();
                                aContext.Replace(aWire, SFW.InternalWire());
                            }
                        }
                        face.Orientation(facet.Orientation());
                        Shape anewShape = aContext.InternalApply(face);
                        var          aIter     = new Iterator(anewShape);
                        if (!aIter.More())
                        {
                            aContext.Remove(anewShape);
                            SendWarning(face, new Msg("FixWireframe.FixSmallEdges.MSG2"));
                        }
                    }
                }
            }

            // enk block
            // Iterate on map of wires which not lie on faces
            for (var expw1 = new Explorer(myShape, ShapeType.Wire, ShapeType.Face); expw1.More(); expw1.Next())
            {
                Wire aWire = ShapeCasts.Wire(expw1.Current());
                SFW.Load(aWire);
                SFW.FixReorder();
                int prev, next, index = 1;
                while (index <= SFW.NbEdges() && SFW.NbEdges() > 1)
                {
                    prev  = (index == 1) ? SFW.NbEdges() : index - 1;
                    next  = (index == SFW.NbEdges()) ? 1 : index + 1;
                    edge1 = SFW.WireData()!.Edge(prev);
                    edge2 = SFW.WireData()!.Edge(index);
                    edge3 = SFW.WireData()!.Edge(next);

                    // gka protection against joining seem edge
                    if (edge2.IsSame(edge1) || edge2.IsSame(edge3))
                    {
                        // if(BRep_Tool::IsClosed(edge2,face)) {
                        index++;
                        continue;
                    }

                    bool isSeam  = SFW.WireData()!.IsSeam(index);
                    bool isSeam1 = SFW.WireData()!.IsSeam(prev);
                    bool isSeam2 = SFW.WireData()!.IsSeam(next);
                    if (theSmallEdges.Contains(edge2))
                    {
                        // Middle edge is small - choose a pair of edges to join
                        bool   IsAnyJoin  = (edge1.IsSame(edge3));
                        bool   take_next  = IsAnyJoin; // false;
                        bool   isLimAngle = false;
                        double Ang1 = 0.0, Ang2 = 0.0;
                        if (SAE.Curve3d(edge1, out Curve3D? C1, out _, out double last1)
                            && SAE.Curve3d(edge2, out Curve3D? C2, out double first2, out double last2)
                            && SAE.Curve3d(edge3, out Curve3D? C3, out double first3, out _))
                        {
                            // Compare angles between edges
                            // double Ang1, Ang2;
                            C1!.D1(last1, out _, out Vector3D Vec1);
                            C2!.D1(first2, out _, out Vector3D Vec2);
                            if (edge1.Orientation() == Reversed)
                            {
                                Vec1.Reverse();
                            }
                            if (edge2.Orientation() == Reversed)
                            {
                                Vec2.Reverse();
                            }
                            double tol2 = Kerf.Engine.Foundation.Precision.SquareConfusion();
                            if (Vec1.SquareMagnitude() < tol2 || Vec2.SquareMagnitude() < tol2)
                            {
                                Ang1 = Math.PI / 2.0;
                            }
                            else
                            {
                                Ang1 = Math.Abs(Vec1.Angle(Vec2));
                            }
                            C2.D1(last2, out _, out Vec1);
                            C3!.D1(first3, out _, out Vec2);
                            if (edge2.Orientation() == Reversed)
                            {
                                Vec1.Reverse();
                            }
                            if (edge3.Orientation() == Reversed)
                            {
                                Vec2.Reverse();
                            }
                            if (Vec1.SquareMagnitude() < tol2 || Vec2.SquareMagnitude() < tol2)
                            {
                                Ang2 = Math.PI / 2.0;
                            }
                            else
                            {
                                Ang2 = Math.Abs(Vec1.Angle(Vec2));
                            }
                            // isLimAngle = (theLimitAngle != -1 && std::min(Ang1,Ang2) > theLimitAngle);
                            // take_next = (Ang2<Ang1);
                            // if (take_next) { edge1 = edge2; edge2 = edge3; }
                        }
                        // if(theLimitAngle != -1 && Ang1 > theLimitAngle && Ang2 >theLimitAngle) {
                        //   index++; continue;
                        // }

                        // Check if edges lay on the same faces
                        if (theMultyEdges.Contains(edge1) || theMultyEdges.Contains(edge2))
                        { //??????
                            index++;
                            continue;
                        }
                        var theList1 = new NCollection_List<Shape>();
                        var theList2 = new NCollection_List<Shape>();
                        var theList3 = new NCollection_List<Shape>();
                        if (theEdgeToFaces.IsBound(edge1))
                        {
                            theList1 = new NCollection_List<Shape>(theEdgeToFaces[edge1]);
                        }
                        if (theEdgeToFaces.IsBound(edge2))
                        {
                            theList2 = new NCollection_List<Shape>(theEdgeToFaces[edge2]);
                        }
                        if (theEdgeToFaces.IsBound(edge3))
                        {
                            theList3 = new NCollection_List<Shape>(theEdgeToFaces[edge3]);
                        }
                        bool same_set = false;

                        // gka protection against joining seem edges with other edges
                        bool same_set1 = (theList1.Extent() == theList2.Extent()
                                          && ((!isSeam && !isSeam1) || (isSeam && isSeam1))); // gka
                        bool same_set2 = (theList3.Extent() == theList2.Extent()
                                          && ((!isSeam && !isSeam2) || (isSeam && isSeam2)));
                        var theSetOfFaces = new NCollection_Map<Shape>(ShapeHasher.Instance);
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
                            if (Math.Abs(Ang2 - Ang1) > Kerf.Engine.Foundation.Precision.Angular())
                            {
                                take_next = (Ang2 < Ang1);
                            }
                            if (take_next)
                            {
                                edge1 = edge2;
                                edge2 = edge3;
                            }
                            isLimAngle = (aModLimitAngle && Math.Min(Ang1, Ang2) > aLimitAngle);
                        }
                        else if (same_set1 && !same_set2)
                        {
                            isLimAngle = (aModLimitAngle && Ang1 > aLimitAngle);
                            same_set   = true;
                        }
                        else if (!same_set1 && same_set2)
                        {
                            same_set   = true;
                            isLimAngle = (aModLimitAngle && Ang2 > aLimitAngle);
                            edge1      = edge2;
                            edge2      = edge3;
                            take_next  = true;
                        }
                        if (same_set && !isLimAngle)
                        {
                            // Merge current pair of edges
                            // gka protection against crossing seem on second face
                            bool isNeedJoin = true; // false;
                            for (var aItF = new NCollection_List<Shape>.Iterator(theList2); aItF.More() && isNeedJoin; aItF.Next())
                            {
                                // OCCT: anExpf2 has explored all the faces here: its Current() is the explored shape
                                if (aItF.Value().IsSame(anExpf2.Current()))
                                {
                                    continue;
                                }
                                Shape aF = aContext.InternalApply(aItF.Value());
                                // aF = aF.Oriented(TopAbs_FORWARD);
                                for (var aIw = new Iterator(aF); aIw.More(); aIw.Next())
                                {
                                    if (aIw.Value().ShapeType() != ShapeType.Wire)
                                    {
                                        continue;
                                    }
                                    Wire wt   = ShapeCasts.Wire(aIw.Value());
                                    var         SFW1 = new ShapeFix_Wire();
                                    SFW1.Load(wt);
                                    SFW1.FixReorder();
                                    ShapeExtend_WireData atmpswd = SFW1.WireData()!;
                                    int                  ind1    = atmpswd.Index(edge1);
                                    int                  ind2    = atmpswd.Index(edge2);
                                    if (ind1 != 0 && ind2 != 0)
                                    {
                                        isNeedJoin = ((ind1 - ind2) == 1 || (ind2 == atmpswd.NbEdges() && ind1 == 1));
                                        break;
                                    }
                                }
                            }
                            bool ReplaceFirst = true;
                            if (isNeedJoin)
                            {
                                var aListF = new NCollection_List<Shape>();
                                for (var aItlF = new NCollection_List<Shape>.Iterator(theList2); aItlF.More(); aItlF.Next())
                                {
                                    Shape tmpF = aContext.InternalApply(aItlF.Value());
                                    aListF.Append(tmpF);
                                }
                                ReplaceFirst = JoinEdges(edge1, edge2, out edge3, aListF);
                            }
                            else
                            {
                                edge3 = new Edge();
                            }
                            if (edge3.IsNull)
                            {
                                index++;
                                myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                            }
                            else
                            {
                                // Record vertex replacements in the map
                                Vertex oldV1 = SAE.FirstVertex(edge3), oldV2 = SAE.LastVertex(edge3);
                                if (!theNewVertices.IsBound(oldV1))
                                // smh#8
                                {
                                    Shape emptyCopiedV1 = oldV1.EmptyCopied();
                                    theNewVertices.Bind(oldV1, ShapeCasts.Vertex(emptyCopiedV1));
                                }
                                if (!oldV1.IsSame(oldV2))
                                {
                                    if (!theNewVertices.IsBound(oldV2))
                                    // smh#8
                                    {
                                        Shape emptyCopiedV2 = oldV2.EmptyCopied();
                                        theNewVertices.Bind(oldV2, ShapeCasts.Vertex(emptyCopiedV2));
                                    }
                                }
                                // To keep NM vertices belonging initial edges
                                var aItv = new Iterator(edge1, false);
                                for (; aItv.More(); aItv.Next())
                                {
                                    if (aItv.Value().Orientation() == Internal
                                        || aItv.Value().Orientation() == External)
                                    {
                                        var           aB    = new BrepBuilder();
                                        Vertex aOldV = ShapeCasts.Vertex(aItv.Value());
                                        Vertex anewV =
                                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, edge3, edge1);
                                        aB.Add(edge3, anewV);
                                        aContext.Replace(aOldV, anewV);
                                    }
                                }

                                for (aItv.Initialize(edge2, false); aItv.More(); aItv.Next())
                                {
                                    if (aItv.Value().Orientation() == Internal
                                        || aItv.Value().Orientation() == External)
                                    {
                                        var           aB    = new BrepBuilder();
                                        Vertex aOldV = ShapeCasts.Vertex(aItv.Value());
                                        Vertex anewV =
                                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, edge3, edge2);
                                        aB.Add(edge3, anewV);
                                        aContext.Replace(aOldV, anewV);
                                    }
                                }
                                // Check for small resulting edge
                                bool newsmall = false;
                                var  SAW      = new ShapeAnalysis_Wire();
                                SAW.Load(SFW.WireData());
                                SAW.SetPrecision(Precision());
                                // Make changes in WireData and Context
                                if (ReplaceFirst)
                                {
                                    aContext.Replace(edge1, edge3);
                                    aContext.Remove(edge2);
                                    SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                }
                                else
                                {
                                    aContext.Replace(edge2, edge3);
                                    aContext.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                }
                                if (take_next)
                                {
                                    SFW.WireData()!.Set(edge3, next);
                                    newsmall = SAW.CheckSmall(next, Precision());
                                }
                                else
                                {
                                    SFW.WireData()!.Set(edge3, prev);
                                    newsmall = SAW.CheckSmall(prev, Precision());
                                }
                                SFW.WireData()!.Remove(index);
                                // Process changes in maps
                                var theList = new NCollection_List<Shape>();
                                theList.Append(theList2);
                                theEdgeToFaces.UnBind(edge1);
                                theEdgeToFaces.UnBind(edge2);
                                theEdgeToFaces.Bind(edge3, theList);
                                if (theSmallEdges.Contains(edge1))
                                {
                                    theSmallEdges.Remove(edge1);
                                }
                                if (theSmallEdges.Contains(edge2))
                                {
                                    theSmallEdges.Remove(edge2);
                                }
                                if (newsmall)
                                {
                                    theSmallEdges.Add(edge3);
                                }
                                for (var itlf = new NCollection_List<Shape>.Iterator(theList); itlf.More(); itlf.Next())
                                {
                                    Shape curface = itlf.Value();
                                    if (theFaceWithSmall.IsBound(curface))
                                    {
                                        NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                            theFaceWithSmall.UnBind(curface);
                                        }
                                    }
                                }
                                myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            }
                        }
                        else if (aModeDrop)
                        { // gka
                            var tempWire = new ShapeExtend_WireData();
                            var tempSaw  = new ShapeAnalysis_Wire();
                            tempWire.Add(SFW.InternalWire());
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
                            int newindex = (index <= tempSaw.NbEdges() ? index : 1);
                            tempSaw.CheckConnected(newindex, Precision());
                            if (!tempSaw.LastCheckStatus(ShapeExtend_FAIL))
                            {
                                SFW.WireData()!.Remove(index);
                                Edge tmpedge1 = tempWire.Edge(newindex);
                                Edge tmpedge2 = tempWire.Edge(newindex == 1 ? tempSaw.NbEdges() : (newindex - 1));
                                var         aL1      = new NCollection_List<Shape>();
                                if (theEdgeToFaces.IsBound(tmpedge1))
                                {
                                    aL1 = new NCollection_List<Shape>(theEdgeToFaces.Find(tmpedge1));
                                }
                                var aL2 = new NCollection_List<Shape>();
                                if (theEdgeToFaces.IsBound(tmpedge2))
                                {
                                    aL2 = new NCollection_List<Shape>(theEdgeToFaces.Find(tmpedge2));
                                }
                                SFW.FixConnected(newindex <= SFW.NbEdges() ? newindex : 1, Precision());
                                SFW.FixDegenerated(newindex <= SFW.NbEdges() ? newindex : 1);
                                Shape aTmpShape = aContext.InternalApply(tmpedge1); // for porting
                                Edge  anewedge1 = ShapeCasts.Edge(aTmpShape);
                                aTmpShape = aContext.InternalApply(tmpedge2);
                                Edge anewedge2 = ShapeCasts.Edge(aTmpShape);
                                aContext.Remove(remedge);
                                SendWarning(remedge, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                if (theSmallEdges.Contains(remedge))
                                {
                                    theSmallEdges.Remove(remedge);
                                }
                                theEdgeToFaces.UnBind(remedge);
                                theEdgeToFaces.UnBind(tmpedge1);
                                theEdgeToFaces.UnBind(tmpedge2);
                                theEdgeToFaces.Bind(anewedge1, aL1);
                                theEdgeToFaces.Bind(anewedge2, aL2);
                                if (theSmallEdges.Contains(tmpedge1))
                                {
                                    theSmallEdges.Remove(tmpedge1);
                                    theSmallEdges.Add(anewedge1);
                                    for (var itlf = new NCollection_List<Shape>.Iterator(aL1); itlf.More(); itlf.Next())
                                    {
                                        Shape                   curface  = itlf.Value();
                                        NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                if (theSmallEdges.Contains(tmpedge2))
                                {
                                    theSmallEdges.Remove(tmpedge2);
                                    theSmallEdges.Add(anewedge2);
                                    for (var itlf = new NCollection_List<Shape>.Iterator(aL2); itlf.More(); itlf.Next())
                                    {
                                        Shape                   curface  = itlf.Value();
                                        NCollection_List<Shape> theEdges = theFaceWithSmall[curface];
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
                                myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
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
                            var         sae = new ShapeAnalysis_Edge();
                            if (sae.Curve3d(ed, out Curve3D? c3d, out double f1, out double l1, false))
                            {
                                c3d!.D0(f1, out Point3D p1);
                                c3d.D0(l1, out Point3D p2);
                                c3d.D0((f1 + l1) * 0.5, out Point3D p3);

                                if (p1.Distance(p3) > p1.Distance(p2))
                                {
                                    index++;
                                    continue;
                                }
                            }
                            if (take_next && theList2.Extent() == 1)
                            { // gka
                                Vertex V1 = SAE.FirstVertex(edge1), V2 = SAE.LastVertex(edge1);
                                if (V1.IsSame(V2))
                                {
                                    SFW.WireData()!.Remove(index);
                                    aContext.Remove(edge1);
                                    SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    if (theSmallEdges.Contains(edge1))
                                    {
                                        theSmallEdges.Remove(edge1);
                                    }
                                    theEdgeToFaces.UnBind(edge1);
                                    myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                                }
                                else
                                {
                                    index++;
                                }
                            }
                            else if (!take_next && theList2.Extent() == 1)
                            {
                                Vertex V1 = SAE.FirstVertex(edge2), V2 = SAE.LastVertex(edge2);
                                if (V1.IsSame(V2))
                                {
                                    SFW.WireData()!.Remove(index);
                                    aContext.Remove(edge2);
                                    SendWarning(edge2, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                                    if (theSmallEdges.Contains(edge2))
                                    {
                                        theSmallEdges.Remove(edge2);
                                    }
                                    theEdgeToFaces.UnBind(edge2);
                                    myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
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
                if (SFW.NbEdges() == 1 && aModeDrop)
                {
                    edge1 = SFW.WireData()!.Edge(1);
                    if (theSmallEdges.Contains(edge1))
                    {
                        SFW.WireData()!.Remove(1);
                        aContext.Remove(edge1);
                        SendWarning(edge1, new Msg("FixWireframe.FixSmallEdges.MSG0"));
                        theSmallEdges.Remove(edge1);
                        theEdgeToFaces.UnBind(edge1);
                        aContext.Remove(aWire);
                        SendWarning(aWire, new Msg("FixWireframe.FixSmallEdges.MSG1"));
                        myStatusSmallEdges |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                    }
                }
                else
                {
                    SFW.FixConnected();
                    aContext.Replace(aWire, SFW.InternalWire());
                }
            }
            // end enk block
            // Record vertex replacements in context
            foreach (KeyValuePair<Shape, Shape> itv in theNewVertices)
            {
                aContext.Replace(itv.Key, itv.Value);
            }

            Shape shape = myShape;
            myShape.Nullify();
            myShape = aContext.InternalApply(shape);

            ShapeFix.SameParameter(myShape, false);

            return StatusSmallEdges(ShapeExtend_DONE);
        }

        return false;
    }

    /// <summary>Decodes the status of the last <see cref="FixWireGaps"/>.</summary>
    public bool StatusWireGaps(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusWireGaps, status);

    /// <summary>Decodes the status of the last <see cref="FixSmallEdges"/>.</summary>
    public bool StatusSmallEdges(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusSmallEdges, status);

    /// <summary>
    /// Returns the resulting shape, frozen; where the loaded shape was frozen, the sub-shapes the fixes left unchanged
    /// are the caller's own, and the context records the ones they changed in place.
    /// </summary>
    public Shape Shape()
    {
        endWork();
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, unfrozen, for ported callers fixing a shape they own.</summary>
    internal Shape InternalShape() => myShape;

    /// <summary>The mode of dropping small edges that can't be merged (false by default).</summary>
    public ref bool ModeDropSmallEdges() => ref myModeDrop;

    /// <summary>Sets the limit angle between two edges above which a small edge isn't merged (-1: no limit).</summary>
    public void SetLimitAngle(double theLimitAngle)
    {
        myLimitAngle = theLimitAngle;
    }

    /// <summary>The limit angle (-1: no limit).</summary>
    public double LimitAngle() => myLimitAngle;

    // The first fix after a load or a public result: copies the loaded shape when it holds frozen shapes.
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry   = null;
        if (myShape.IsNull)
        {
            return;
        }
        if (Context() is null)
        {
            // OCCT makes the context in the fix; it is made here so that the copy records into it
            SetContext(new ShapeBuild_ReShape());
            myIsNewContext = true;
        }
        if (BrepTools.HoldsFrozen(myShape))
        {
            myEntry = new CopyAtEntry(myShape, Context());
            myShape = myEntry.Shape();
        }
    }

    // Before a public result: the copies the fixes left unchanged are the caller's own again.
    private void endWork()
    {
        myToEnter = true;
        CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is not null)
        {
            myShape = anEntry.Finish(myShape);
        }
    }
}
