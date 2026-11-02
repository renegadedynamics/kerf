// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Schematiq.Collections;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
/// its context). When an input holds a frozen shape, <see cref="Perform"/> runs on an exact copy of the inputs
/// (<c>BRepTools.ExactCopyForWrite</c>), and afterwards every sub-shape of the copy that sewing left unchanged is the
/// caller's own again, in the result and in every answer (decision 583). The queries take the caller's shapes or the
/// shapes they handed out. The context (<see cref="GetContext"/>) then records the working copies' sub-shapes.</para>
/// </remarks>
public class BRepBuilderAPI_Sewing : Standard_Transient
{
    protected double myTolerance;
    protected bool mySewing;
    protected bool myAnalysis;
    protected bool myCutting;
    protected bool myNonmanifold;
    protected NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myOldShapes = new(TopTools_ShapeMapHasher.Instance);
    protected TopoDS_Shape mySewedShape = new();
    protected NCollection_IndexedMap<TopoDS_Shape> myDegenerated = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedMap<TopoDS_Shape> myFreeEdges = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedMap<TopoDS_Shape> myMultipleEdges = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myContigousEdges =
        new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myContigSecBound = new(TopTools_ShapeMapHasher.Instance);
    protected int myNbShapes;
    protected int myNbVertices;
    protected int myNbEdges;
    protected NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myBoundFaces =
        new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myBoundSections =
        new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> mySectionBound = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myVertexNode = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myVertexNodeFree = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myNodeSections =
        new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myCuttingNode =
        new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_IndexedMap<TopoDS_Shape> myLittleFace = new(TopTools_ShapeMapHasher.Instance);
    protected TopoDS_Shape myShape = new();
    protected BRepTools_ReShape myReShape;

    private bool myFaceMode;
    private bool myFloatingEdgesMode;
    private bool mySameParameterMode;
    private bool myLocalToleranceMode;
    private double myMinTolerance;
    private double myMaxTolerance;
    private readonly NCollection_Map<TopoDS_Shape> myMergedEdges = new(TopTools_ShapeMapHasher.Instance);

    // OCCT.NET: copy at entry (decision 583). When an input holds a frozen shape, Perform works on an exact copy:
    // sewing (the original where sewing left the copy unchanged, the copy otherwise). Both null otherwise.
    private Dictionary<TopoDS_TShape, TopoDS_TShape>? myWorkCopies;
    private Dictionary<TopoDS_TShape, TopoDS_TShape>? myWorkReplaced;

    /// <summary>
    /// Creates an object with the tolerance of connexity <paramref name="tolerance"/> and the options for sewing
    /// (<paramref name="option1"/>; if false, only control), analysis of degenerated shapes (<paramref name="option2"/>),
    /// cutting of free edges (<paramref name="option3"/>) and non-manifold processing (<paramref name="option4"/>).
    /// </summary>
    public BRepBuilderAPI_Sewing(double tolerance = 1.0e-06,
                                 bool option1 = true,
                                 bool option2 = true,
                                 bool option3 = true,
                                 bool option4 = false)
    {
        myReShape = new BRepTools_ReShape();
        Init(tolerance, option1, option2, option3, option4);
    }

    //=================================================================================================
    // BRepBuilderAPI_Sewing.lxx

    /// <summary>Gets the same parameter mode.</summary>
    public bool SameParameterMode() => mySameParameterMode;

    /// <summary>Sets the same parameter mode.</summary>
    public void SetSameParameterMode(bool SameParameterMode) => mySameParameterMode = SameParameterMode;

    /// <summary>The tolerance set.</summary>
    public double Tolerance() => myTolerance;

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double theToler) => myTolerance = theToler;

    /// <summary>The min tolerance set.</summary>
    public double MinTolerance() => myMinTolerance;

    /// <summary>Sets the min tolerance.</summary>
    public void SetMinTolerance(double theMinToler) => myMinTolerance = theMinToler;

    /// <summary>The max tolerance set.</summary>
    public double MaxTolerance() => myMaxTolerance;

    /// <summary>Sets the max tolerance.</summary>
    public void SetMaxTolerance(double theMaxToler) => myMaxTolerance = theMaxToler;

    /// <summary>The mode for sewing faces (true by default).</summary>
    public bool FaceMode() => myFaceMode;

    /// <summary>Sets the mode for sewing faces (true by default).</summary>
    public void SetFaceMode(bool theFaceMode) => myFaceMode = theFaceMode;

    /// <summary>The mode for sewing floating edges (false by default).</summary>
    public bool FloatingEdgesMode() => myFloatingEdgesMode;

    /// <summary>Sets the mode for sewing floating edges (false by default).</summary>
    public void SetFloatingEdgesMode(bool theFloatingEdgesMode) => myFloatingEdgesMode = theFloatingEdgesMode;

    /// <summary>The mode for accounting of local tolerances of edges and vertices during merging.</summary>
    public bool LocalTolerancesMode() => myLocalToleranceMode;

    /// <summary>
    /// Sets the mode for accounting of local tolerances of edges and vertices during merging; then the working
    /// tolerance is myTolerance + tolEdge1 + tolEdge2.
    /// </summary>
    public void SetLocalTolerancesMode(bool theLocalTolerancesMode) => myLocalToleranceMode = theLocalTolerancesMode;

    /// <summary>Sets the mode for non-manifold sewing.</summary>
    public void SetNonManifoldMode(bool theNonManifoldMode) => myNonmanifold = theNonManifoldMode;

    /// <summary>The mode for non-manifold sewing.</summary>
    public bool NonManifoldMode() => myNonmanifold;

    //=================================================================================================
    // OCCT.NET: publication and the copy at entry (decision 583).

    // The shape the algorithm works on for a shape a caller passes: its working copy when the inputs were copied.
    private TopoDS_Shape toWork(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = theShape;
        if (myWorkCopies is not null && !aShape.IsNull()
            && myWorkCopies.TryGetValue(aShape.TShape()!, out TopoDS_TShape? aCopy))
        {
            aShape.TShape(aCopy);
        }
        return aShape;
    }

    // The shape a caller sees for a working shape: each copy that sewing left unchanged is the caller's original again,
    // in the shape's unfrozen graph (as BRepTools.KeepUnchangedIn does for the result) and at its top.
    private TopoDS_Shape fromWork(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = theShape;
        if (myWorkReplaced is null || myWorkReplaced.Count == 0 || aShape.IsNull())
        {
            return aShape;
        }
        if (myWorkReplaced.TryGetValue(aShape.TShape()!, out TopoDS_TShape? anOriginal))
        {
            aShape.TShape(anOriginal);
            return aShape;
        }
        substituteTShapes(aShape.TShape()!, myWorkReplaced, new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance));
        return aShape;
    }

    // Replaces, in the unfrozen TShapes reachable from theTShape, each child whose TShape is a key of the map by the
    // map's value, keeping its place, location and orientation (BRepTools.KeepUnchangedIn's walk).
    private static void substituteTShapes(TopoDS_TShape theTShape, Dictionary<TopoDS_TShape, TopoDS_TShape> theMap,
                                          HashSet<TopoDS_TShape> theVisited)
    {
        if (theTShape.IsFrozen() || !theVisited.Add(theTShape))
        {
            return;
        }

        RuntimeHelpers.EnsureSufficientExecutionStack();
        var aChildren = new List<TopoDS_Shape>();
        bool isChanged = false;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theTShape.myShapes); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aChild = anIt.Value();
            if (theMap.TryGetValue(aChild.TShape()!, out TopoDS_TShape? aNew))
            {
                aChild.TShape(aNew);
                isChanged = true;
            }
            else
            {
                substituteTShapes(aChild.TShape()!, theMap, theVisited);
            }
            aChildren.Add(aChild);
        }

        if (isChanged)
        {
            theTShape.myShapes.Clear();
            foreach (TopoDS_Shape aChild in aChildren)
            {
                theTShape.myShapes.Append(aChild);
            }
        }
    }

    // A working shape handed to a caller: the caller's view of it (fromWork), frozen.
    private TopoDS_Shape published(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = fromWork(theShape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    // Before Perform: when an input holds a frozen shape, replaces the inputs by an exact copy of them all (one copy,
    // so that sub-shapes shared between inputs stay shared). Returns the compound of the inputs and its copy.
    private bool beginWork(out TopoDS_Shape theOriginals, out TopoDS_Shape theCopy,
                           out Dictionary<TopoDS_TShape, TopoDS_TShape>? theCopies)
    {
        myWorkCopies = null;
        myWorkReplaced = null;
        theCopy = new TopoDS_Shape();
        theCopies = null;

        var B = new BRep_Builder();
        var aComp = new TopoDS_Compound();
        B.MakeCompound(ref aComp);
        if (!myShape.IsNull())
        {
            B.Add(aComp, myShape);
        }
        for (int i = 1; i <= myOldShapes.Extent(); i++)
        {
            if (!myOldShapes[i].IsNull())
            {
                B.Add(aComp, myOldShapes[i]);
            }
        }
        theOriginals = aComp;
        if (!BRepTools.HoldsFrozen(aComp))
        {
            return false;
        }

        theCopy = BRepTools.ExactCopyForWrite(aComp, out _, out Dictionary<TopoDS_TShape, TopoDS_TShape> aCopies);
        theCopies = aCopies;
        myWorkCopies = aCopies;
        var anIt = new TopoDS_Iterator(theCopy);
        if (!myShape.IsNull())
        {
            myShape = anIt.Value();
            anIt.Next();
        }
        for (int i = 1; i <= myOldShapes.Extent(); i++)
        {
            if (!myOldShapes[i].IsNull())
            {
                myOldShapes[i] = anIt.Value();
                anIt.Next();
            }
        }
        return true;
    }

    // After Perform on copies: gives back the caller's sub-shapes wherever sewing left the copy unchanged, in the
    // sewn shape (BRepTools.KeepUnchangedIn) and, through fromWork, in every answer.
    private void endWork(in TopoDS_Shape theOriginals, in TopoDS_Shape theCopy,
                         Dictionary<TopoDS_TShape, TopoDS_TShape> theCopies)
    {
        TopoDS_Shape aResult = mySewedShape;
        Dictionary<TopoDS_TShape, TopoDS_TShape> aFinal =
            BRepTools.KeepUnchangedIn(theOriginals, theCopy, theCopies, ref aResult, out _);
        mySewedShape = aResult;
        myWorkReplaced = new Dictionary<TopoDS_TShape, TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        foreach (KeyValuePair<TopoDS_TShape, TopoDS_TShape> aCF in aFinal)
        {
            if (!ReferenceEquals(aCF.Key, aCF.Value))
            {
                myWorkReplaced[aCF.Key] = aCF.Value;
            }
        }
    }

    //=================================================================================================

    /// <summary>The pcurve given the range RequestedFirst to RequestedLast (GeomLib.SameRange); null on failure.</summary>
    /// <remarks>Called from SameParameterEdge only.</remarks>
    protected virtual Geom2d_Curve? SameRange(Geom2d_Curve? CurvePtr,
                                              double FirstOnCurve,
                                              double LastOnCurve,
                                              double RequestedFirst,
                                              double RequestedLast)
    {
        Geom2d_Curve? NewCurvePtr = null;
        try
        {
            GeomLib.SameRange(Precision.PConfusion(),
                              CurvePtr,
                              FirstOnCurve,
                              LastOnCurve,
                              RequestedFirst,
                              RequestedLast,
                              ref NewCurvePtr);
        }
        catch (Standard_Failure)
        {
        }
        return NewCurvePtr;
    }

    //=======================================================================
    // function : WhichFace
    // purpose  : Give the face whose edge is the border
    //=======================================================================

    /// <summary>The <paramref name="index"/>-th face whose border is the edge (or the edge's bound), frozen; a null face if none.</summary>
    public TopoDS_Face WhichFace(in TopoDS_Edge theEdg, int index = 1) =>
        TopoDS.Face(published(InternalWhichFace(TopoDS.Edge(toWork(theEdg)), index)));

    /// <summary><see cref="WhichFace"/> on the working shapes, without freezing (ported code).</summary>
    internal TopoDS_Face InternalWhichFace(in TopoDS_Edge theEdg, int index = 1)
    {
        TopoDS_Shape bound = theEdg;
        if (mySectionBound.IsBound(bound))
        {
            bound = mySectionBound[bound];
        }
        if (myBoundFaces.Contains(bound))
        {
            int i = 1;
            var itf = new NCollection_List<TopoDS_Shape>.Iterator(myBoundFaces.FindFromKey(bound));
            for (; itf.More(); itf.Next(), i++)
            {
                if (i == index)
                {
                    return TopoDS.Face(itf.Value());
                }
            }
        }
        return new TopoDS_Face();
    }

    //=================================================================================================

    private static bool IsClosedShape(in TopoDS_Shape theshape, in TopoDS_Shape v1, in TopoDS_Shape v2)
    {
        double TotLength = 0.0;
        var aexp = new TopExp_Explorer();
        for (aexp.Init(theshape, TopAbs_EDGE); aexp.More(); aexp.Next())
        {
            TopoDS_Edge aedge = TopoDS.Edge(aexp.Current());
            if (aedge.IsNull())
            {
                continue;
            }
            TopExp.Vertices(aedge, out TopoDS_Vertex ve1, out TopoDS_Vertex ve2);
            if (!ve1.IsSame(v1) && !ve1.IsSame(v2))
            {
                continue;
            }
            if (BRep_Tool.Degenerated(aedge))
            {
                continue;
            }
            Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(aedge), out double first, out double last);
            if (c3d is not null)
            {
                var cAdapt = new GeomAdaptor_Curve(c3d);
                double length = GCPnts_AbscissaPoint.Length(cAdapt, first, last);
                TotLength += length;
                if (ve2.IsSame(v1) || ve2.IsSame(v2))
                {
                    break;
                }
            }
        }
        if (TotLength > 0.0)
        {
            gp_Pnt p1 = BRep_Tool.Pnt(TopoDS.Vertex(v1));
            gp_Pnt p2 = BRep_Tool.Pnt(TopoDS.Vertex(v2));
            return (p1.Distance(p2) < TotLength / (1.2 * M_PI));
        }
        return false;
    }

    //=================================================================================================

    private static bool IsClosedByIsos(Geom_Surface thesurf,
                                       Geom2d_Curve acrv2d,
                                       double f2d,
                                       double l2d,
                                       bool isUIsos)
    {
        bool isClosed = false;

        gp_Pnt2d psurf1 = (acrv2d.IsPeriodic() ? acrv2d.Value(f2d)
                                               : acrv2d.Value(Max(f2d, acrv2d.FirstParameter())));
        gp_Pnt2d psurf2 = (acrv2d.IsPeriodic() ? acrv2d.Value(l2d)
                                               : acrv2d.Value(Min(l2d, acrv2d.LastParameter())));
        Geom_Curve aCrv1;
        Geom_Curve aCrv2;
        if (isUIsos)
        {
            aCrv1 = thesurf.UIso(psurf1.X());
            aCrv2 = thesurf.UIso(psurf2.X());
        }
        else
        {
            aCrv1 = thesurf.VIso(psurf1.Y());
            aCrv2 = thesurf.VIso(psurf2.Y());
        }
        gp_Pnt p11 = new(), p1m = new(), p12 = new(), p21 = new(), p2m = new(), p22 = new();
        double af1 = aCrv1.FirstParameter();
        double al1 = aCrv1.LastParameter();
        double af2 = aCrv2.FirstParameter();
        double al2 = aCrv2.LastParameter();
        aCrv1.D0(af1, out p11);
        aCrv1.D0((af1 + al1) * 0.5, out p1m);
        aCrv1.D0(al1, out p12);
        aCrv2.D0(af2, out p21);
        aCrv2.D0((af2 + al2) * 0.5, out p2m);
        aCrv2.D0(al2, out p22);
        isClosed = (((p11.XYZ() - p12.XYZ()).Modulus()
                     < (p11.XYZ() - p1m.XYZ()).Modulus() - Precision.Confusion())
                    && ((p21.XYZ() - p22.XYZ()).Modulus()
                        < (p21.XYZ() - p2m.XYZ()).Modulus() - Precision.Confusion()));
        return isClosed;
    }

    //=================================================================================================

    /// <summary>Defines if the surface is U-closed (along the edge).</summary>
    protected virtual bool IsUClosedSurface(Geom_Surface surf, in TopoDS_Shape theEdge, in TopLoc_Location theloc)
    {
        Geom_Surface tmpsurf = surf;
        if (tmpsurf is Geom_RectangularTrimmedSurface aRTS)
        {
            tmpsurf = aRTS.BasisSurface();
        }
        else if (tmpsurf is Geom_OffsetSurface anOS)
        {
            tmpsurf = anOS.BasisSurface();
        }
        else
        {
            bool isClosed = tmpsurf.IsUClosed();
            if (!isClosed)
            {
                Geom2d_Curve? acrv2d =
                    BRep_Tool.CurveOnSurface(TopoDS.Edge(theEdge), surf, theloc, out double f2d, out double l2d);
                if (acrv2d is not null)
                {
                    isClosed = IsClosedByIsos(tmpsurf, acrv2d, f2d, l2d, false);
                }
            }
            return isClosed;
        }
        return IsUClosedSurface(tmpsurf, theEdge, theloc);
    }

    //=================================================================================================

    /// <summary>Defines if the surface is V-closed (along the edge).</summary>
    protected virtual bool IsVClosedSurface(Geom_Surface surf, in TopoDS_Shape theEdge, in TopLoc_Location theloc)
    {
        Geom_Surface tmpsurf = surf;
        if (tmpsurf is Geom_RectangularTrimmedSurface aRTS)
        {
            tmpsurf = aRTS.BasisSurface();
        }
        else if (tmpsurf is Geom_OffsetSurface anOS)
        {
            tmpsurf = anOS.BasisSurface();
        }
        else
        {
            bool isClosed = tmpsurf.IsVClosed();
            if (!isClosed)
            {
                Geom2d_Curve? acrv2d =
                    BRep_Tool.CurveOnSurface(TopoDS.Edge(theEdge), surf, theloc, out double f2d, out double l2d);
                if (acrv2d is not null)
                {
                    isClosed = IsClosedByIsos(tmpsurf, acrv2d, f2d, l2d, true);
                }
            }
            return isClosed;
        }
        return IsVClosedSurface(tmpsurf, theEdge, theloc);
    }

    //=================================================================================================

    /// <summary>BRepLib.SameParameter on the edge, failures ignored. Called from SameParameterEdge only.</summary>
    protected virtual void SameParameter(in TopoDS_Edge edge)
    {
        try
        {
            BRepLib.SameParameter(edge);
        }
        catch (Standard_Failure)
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
    protected virtual TopoDS_Edge SameParameterEdge(in TopoDS_Shape edge,
                                                    NCollection_Sequence<TopoDS_Shape> seqEdges,
                                                    NCollection_Sequence<bool> seqForward,
                                                    NCollection_Map<TopoDS_Shape> mapMerged,
                                                    BRepTools_ReShape locReShape)
    {
        // Retrieve reference section
        TopoDS_Shape aTmpShape = myReShape.InternalApply(edge); // for porting
        TopoDS_Edge Edge1 = TopoDS.Edge(aTmpShape);
        aTmpShape = locReShape.InternalApply(Edge1);
        if (locReShape != myReShape)
        {
            Edge1 = TopoDS.Edge(aTmpShape);
        }
        bool isDone = false;

        // Create data structures for temporary merged edges
        var listFaces1 = new NCollection_List<TopoDS_Shape>();
        var MergedFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        if (mySewing)
        {
            // Fill MergedFaces with faces of Edge1
            TopoDS_Shape bnd1 = edge;
            if (mySectionBound.IsBound(bnd1))
            {
                bnd1 = mySectionBound[bnd1];
            }
            if (myBoundFaces.Contains(bnd1))
            {
                var itf = new NCollection_List<TopoDS_Shape>.Iterator(myBoundFaces.FindFromKey(bnd1));
                for (; itf.More(); itf.Next())
                {
                    if (MergedFaces.Add(itf.Value()))
                    {
                        listFaces1.Append(itf.Value());
                    }
                }
            }
        }
        else
        {
            // Create presentation edge
            TopExp.Vertices(Edge1, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (myVertexNode.Contains(V1))
            {
                V1 = TopoDS.Vertex(myVertexNode.FindFromKey(V1));
            }
            if (myVertexNode.Contains(V2))
            {
                V2 = TopoDS.Vertex(myVertexNode.FindFromKey(V2));
            }

            TopoDS_Edge NewEdge = Edge1;
            NewEdge.EmptyCopy();

            // Add the vertices
            var aBuilder = new BRep_Builder();
            TopoDS_Shape anEdge = NewEdge.Oriented(TopAbs_FORWARD);
            aBuilder.Add(anEdge, V1.Oriented(TopAbs_FORWARD));
            aBuilder.Add(anEdge, V2.Oriented(TopAbs_REVERSED));

            Edge1 = NewEdge;
        }

        bool isForward = true;

        // Merge candidate sections
        for (int i = 1; i <= seqEdges.Length(); i++)
        {
            // Retrieve candidate section
            TopoDS_Shape oedge2 = seqEdges[i];

            if (mySewing)
            {
                aTmpShape = myReShape.InternalApply(oedge2); // for porting
                TopoDS_Edge Edge2 = TopoDS.Edge(aTmpShape);
                aTmpShape = locReShape.InternalApply(Edge2);
                if (locReShape != myReShape)
                {
                    Edge2 = TopoDS.Edge(aTmpShape);
                }

                // Calculate relative orientation
                bool Orientation = seqForward[i];
                if (!isForward)
                {
                    Orientation = !Orientation;
                }

                // Retrieve faces information for the second edge
                TopoDS_Shape bnd2 = oedge2;
                if (mySectionBound.IsBound(bnd2))
                {
                    bnd2 = mySectionBound[bnd2];
                }
                if (!myBoundFaces.Contains(bnd2))
                {
                    continue; // Skip floating edge
                }
                NCollection_List<TopoDS_Shape> listFaces2 = myBoundFaces.FindFromKey(bnd2);

                int whichSec = 1; // Indicates on which edge the pCurve has been reported
                TopoDS_Edge NewEdge =
                    SameParameterEdge(Edge1, Edge2, listFaces1, listFaces2, Orientation, ref whichSec);
                if (NewEdge.IsNull())
                {
                    continue;
                }

                // Record faces information for the temporary merged edge
                var itf = new NCollection_List<TopoDS_Shape>.Iterator(listFaces2);
                for (; itf.More(); itf.Next())
                {
                    if (MergedFaces.Add(itf.Value()))
                    {
                        listFaces1.Append(itf.Value());
                    }
                }

                // Record merged section orientation
                if (!Orientation && whichSec != 1)
                {
                    isForward = !isForward;
                }
                Edge1 = NewEdge;
            }

            // Append actually merged edge
            mapMerged.Add(oedge2);
            isDone = true;

            if (!myNonmanifold)
            {
                break;
            }
        }

        if (isDone)
        {
            // Change result orientation
            Edge1.Orientation(isForward ? TopAbs_FORWARD : TopAbs_REVERSED);
        }
        else
        {
            Edge1.Nullify();
        }

        return Edge1;
    }

    //=================================================================================================

    private static bool findNMVertices(in TopoDS_Edge theEdge,
                                       NCollection_Sequence<TopoDS_Shape> theSeqNMVert,
                                       NCollection_Sequence<double> theSeqPars)
    {
        var aItV = new TopoDS_Iterator(theEdge, false);
        for (; aItV.More(); aItV.Next())
        {
            if (aItV.Value().Orientation() == TopAbs_INTERNAL
                || aItV.Value().Orientation() == TopAbs_EXTERNAL)
            {
                theSeqNMVert.Append(aItV.Value());
            }
        }
        int nbV = theSeqNMVert.Length();
        if (nbV == 0)
        {
            return false;
        }
        Geom_Curve? c3d = BRep_Tool.Curve(theEdge, out double first, out double last);
        var GAC = new GeomAdaptor_Curve(c3d!);
        var locProj = new Extrema_ExtPC();
        locProj.Initialize(GAC, first, last);
        gp_Pnt pfirst = GAC.Value(first), plast = GAC.Value(last);

        for (int i = 1; i <= nbV; i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(theSeqNMVert.Value(i));
            gp_Pnt pt = BRep_Tool.Pnt(aV);

            double distF2 = pfirst.SquareDistance(pt);
            double distL2 = plast.SquareDistance(pt);
            double apar = (distF2 > distL2 ? last : first);
            // Project current point on curve
            locProj.Perform(pt);
            if (locProj.IsDone() && locProj.NbExt() > 0)
            {
                double dist2Min = Min(distF2, distL2);
                int ind, indMin = 0;
                for (ind = 1; ind <= locProj.NbExt(); ind++)
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

                theSeqPars.Append(apar);
            }
        }
        return true;
    }

    private static void ComputeToleranceVertex(TopoDS_Vertex theV1, TopoDS_Vertex theV2, ref TopoDS_Vertex theNewV)
    {
        int m, n;
        double dR, aD, aEps;
        Span<double> aR = stackalloc double[2];
        var aV = new TopoDS_Vertex[2];
        var aP = new gp_Pnt[2];
        var aBB = new BRep_Builder();
        //
        aEps = RealEpsilon();
        aV[0] = theV1;
        aV[1] = theV2;
        for (m = 0; m < 2; ++m)
        {
            aP[m] = BRep_Tool.Pnt(aV[m]);
            aR[m] = BRep_Tool.Tolerance(aV[m]);
        }
        //
        m = 0; // max R
        n = 1; // min R
        if (aR[0] < aR[1])
        {
            m = 1;
            n = 0;
        }
        //
        dR = aR[m] - aR[n]; // dR >= 0.
        var aVD = new gp_Vec(aP[m], aP[n]);
        aD = aVD.Magnitude();
        //
        if (aD <= dR || aD < aEps)
        {
            aBB.MakeVertex(ref theNewV, aP[m], aR[m]);
        }
        else
        {
            double aRr;
            gp_XYZ aXYZr;
            var aPr = new gp_Pnt();
            //
            aRr = 0.5 * (aR[m] + aR[n] + aD);
            aXYZr = 0.5 * (aP[m].XYZ() + aP[n].XYZ() - aVD.XYZ() * (dR / aD));
            aPr.SetXYZ(aXYZr);
            //
            aBB.MakeVertex(ref theNewV, aPr, aRr);
        }
    }

    private static void ComputeToleranceVertex(TopoDS_Vertex theV1,
                                               TopoDS_Vertex theV2,
                                               TopoDS_Vertex theV3,
                                               ref TopoDS_Vertex theNewV)
    {
        double aDi, aDmax;
        var aCenter = new gp_Pnt();
        var aP = new gp_Pnt[3];
        Span<double> aR = stackalloc double[3];
        var aV = new TopoDS_Vertex[3];
        var aXYZ = new gp_XYZ(0.0, 0.0, 0.0);
        aV[0] = theV1;
        aV[1] = theV2;
        aV[2] = theV3;
        for (int i = 0; i < 3; ++i)
        {
            aP[i] = BRep_Tool.Pnt(aV[i]);
            aR[i] = BRep_Tool.Tolerance(aV[i]);
            aXYZ = aXYZ + aP[i].XYZ();
        }
        //
        aXYZ.Divide(3.0);
        aCenter.SetXYZ(aXYZ);
        //
        aDmax = -1.0;
        for (int i = 0; i < 3; ++i)
        {
            aDi = aCenter.Distance(aP[i]);
            aDi += aR[i];
            if (aDi > aDmax)
            {
                aDmax = aDi;
            }
        }

        var aBB = new BRep_Builder();
        aBB.MakeVertex(ref theNewV, aCenter, aDmax);
    }

    /// <summary>
    /// Merges two sections into a new edge keeping the 3D curve, range and parametrisation of one of them (the longer
    /// one at the first call; <paramref name="whichSec"/> tells which) and the pcurves of both. Called from Merging only.
    /// </summary>
    protected virtual TopoDS_Edge SameParameterEdge(in TopoDS_Edge edgeFirst,
                                                    in TopoDS_Edge edgeLast,
                                                    NCollection_List<TopoDS_Shape> listFacesFirst,
                                                    NCollection_List<TopoDS_Shape> listFacesLast,
                                                    bool secForward,
                                                    ref int whichSec,
                                                    bool firstCall = true)
    {
        // Do not process floating edges
        if (listFacesFirst.Extent() == 0 || listFacesLast.Extent() == 0)
        {
            return new TopoDS_Edge();
        }

        // Sort input edges
        TopoDS_Edge edge1 = new(), edge2 = new();
        if (firstCall)
        {
            // Take the longest edge as first
            Geom_Curve? c3d1 = BRep_Tool.Curve(TopoDS.Edge(edgeFirst), out double f, out double l);
            var cAdapt1 = new GeomAdaptor_Curve(c3d1!);
            double len1 = GCPnts_AbscissaPoint.Length(cAdapt1, f, l);
            Geom_Curve? c3d2 = BRep_Tool.Curve(TopoDS.Edge(edgeLast), out f, out l);
            var cAdapt2 = new GeomAdaptor_Curve(c3d2!);
            double len2 = GCPnts_AbscissaPoint.Length(cAdapt2, f, l);
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

        BRep_Tool.Range(edge1, out double first, out double last);
        var aBuilder = new BRep_Builder();

        // To keep NM vertices on edge
        var aSeqNMVert = new NCollection_Sequence<TopoDS_Shape>();
        var aSeqNMPars = new NCollection_Sequence<double>();
        findNMVertices(edge1, aSeqNMVert, aSeqNMPars);
        findNMVertices(edge2, aSeqNMVert, aSeqNMPars);

        // Create new edge
        var edge = new TopoDS_Edge();
        aBuilder.MakeEdge(ref edge);
        edge.Orientation(edge1.Orientation());

        // Retrieve edge curve
        Geom_Curve? c3d = BRep_Tool.Curve(edge1, out TopLoc_Location loc3d, out double first3d, out double last3d);
        if (!loc3d.IsIdentity())
        {
            c3d = (Geom_Curve)c3d!.Copy();
            c3d.Transform(loc3d.Transformation());
        }
        aBuilder.UpdateEdge(edge, c3d, BRep_Tool.Tolerance(edge1));
        aBuilder.Range(edge, first, last);
        aBuilder.SameRange(edge, false); // true
        aBuilder.SameParameter(edge, false);
        // Create and add new vertices
        {
            TopoDS_Vertex V1New = new(), V2New = new();

            // Retrieve original vertices from edges
            TopExp.Vertices(edge1, out TopoDS_Vertex V11, out TopoDS_Vertex V12);
            TopExp.Vertices(edge2, out TopoDS_Vertex V21, out TopoDS_Vertex V22);

            // check that edges merged valid way (for edges having length less than specified
            // tolerance
            //  Check if edges are closed
            bool isClosed1 = V11.IsSame(V12);
            bool isClosed2 = V21.IsSame(V22);
            if (!isClosed1 && !isClosed2)
            {
                if (secForward)
                {
                    if (V11.IsSame(V22) || V12.IsSame(V21))
                    {
                        return new TopoDS_Edge();
                    }
                }
                else
                {
                    if (V11.IsSame(V21) || V12.IsSame(V22))
                    {
                        return new TopoDS_Edge();
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
                    ComputeToleranceVertex(V11, V21, ref V1New);
                }
                else if (isClosed1)
                {
                    // only first edge is closed
                    ComputeToleranceVertex(V22, V21, V11, ref V1New);
                }
                else
                {
                    // only second edge is closed
                    ComputeToleranceVertex(V11, V12, V21, ref V1New);
                }
                V2New = V1New;
            }
            else
            {
                // both edges are open
                bool isOldFirst = (secForward ? V11.IsSame(V21) : V11.IsSame(V22));
                bool isOldLast = (secForward ? V12.IsSame(V22) : V12.IsSame(V21));
                if (secForward)
                {
                    // case if vertices already sewed
                    if (!isOldFirst)
                    {
                        ComputeToleranceVertex(V11, V21, ref V1New);
                    }
                    if (!isOldLast)
                    {
                        ComputeToleranceVertex(V12, V22, ref V2New);
                    }
                }
                else
                {
                    if (!isOldFirst)
                    {
                        ComputeToleranceVertex(V11, V22, ref V1New);
                    }
                    if (!isOldLast)
                    {
                        ComputeToleranceVertex(V12, V21, ref V2New);
                    }
                }
                if (isOldFirst)
                {
                    V1New = V11;
                }

                if (isOldLast)
                {
                    V2New = V12;
                }
            }
            // Add the vertices in the good sense
            TopoDS_Shape anEdge = edge.Oriented(TopAbs_FORWARD);
            TopoDS_Shape aLocalEdge = V1New.Oriented(TopAbs_FORWARD); //(listNode.First()).Oriented(TopAbs_FORWARD);
            aBuilder.Add(anEdge, aLocalEdge);
            aLocalEdge = V2New.Oriented(TopAbs_REVERSED); //(listNode.Last()).Oriented(TopAbs_REVERSED);
            aBuilder.Add(anEdge, aLocalEdge);

            int k = 1;
            for (; k <= aSeqNMVert.Length(); k++)
            {
                aBuilder.Add(anEdge, aSeqNMVert.Value(k));
            }
        }

        // Retrieve second PCurves
        var loc2 = new TopLoc_Location();
        Geom_Surface? surf2 = null;

        var itf2 = new NCollection_List<TopoDS_Shape>.Iterator();
        if (whichSec == 1)
        {
            itf2.Initialize(listFacesLast);
        }
        else
        {
            itf2.Initialize(listFacesFirst);
        }
        bool isResEdge = false;
        var fac2 = new TopoDS_Face();
        for (; itf2.More(); itf2.Next())
        {
            Geom2d_Curve? c2d2 = null, c2d21 = null;
            double firstOld, lastOld;
            fac2 = TopoDS.Face(itf2.Value());

            surf2 = BRep_Tool.Surface(fac2, out loc2);
            bool isSeam2 = ((IsUClosedSurface(surf2!, edge2, loc2) || IsVClosedSurface(surf2!, edge2, loc2))
                            && BRep_Tool.IsClosed(TopoDS.Edge(edge2), fac2));
            if (isSeam2)
            {
                if (!myNonmanifold)
                {
                    return new TopoDS_Edge();
                }
                TopoDS_Shape aTmpShape = edge2.Reversed(); // for porting
                c2d21 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aTmpShape), fac2, out firstOld, out lastOld);
            }
            c2d2 = BRep_Tool.CurveOnSurface(edge2, fac2, out firstOld, out lastOld);
            if (c2d2 is null && c2d21 is null)
            {
                continue;
            }

            if (c2d21 is not null)
            {
                c2d21 = (Geom2d_Curve)c2d21.Copy();
                if (!secForward)
                {
                    if (c2d21 is Geom2d_Line)
                    {
                        c2d21 = new Geom2d_TrimmedCurve(c2d21, firstOld, lastOld);
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
            c2d2 = (Geom2d_Curve)c2d2!.Copy(); // OCCT: null when only the seam's reversed pcurve exists

            if (!secForward)
            {
                if (c2d2 is Geom2d_Line)
                {
                    c2d2 = new Geom2d_TrimmedCurve(c2d2, firstOld, lastOld);
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
            TopAbs_Orientation Ori = TopAbs_FORWARD;

            var itf1 = new NCollection_List<TopoDS_Shape>.Iterator();
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
                Geom2d_Curve? c2d1 = null, c2d11 = null;
                TopoDS_Face fac1 = TopoDS.Face(itf1.Value());

                Geom_Surface? surf1 = BRep_Tool.Surface(fac1, out TopLoc_Location loc1);

                double first2d, last2d;
                bool isSeam1 = ((IsUClosedSurface(surf1!, edge1, loc1) || IsVClosedSurface(surf1!, edge1, loc1))
                                && BRep_Tool.IsClosed(TopoDS.Edge(edge1), fac1));
                c2d1 = BRep_Tool.CurveOnSurface(edge1, fac1, out first2d, out last2d);
                Ori = edge1.Orientation();
                if (fac1.Orientation() == TopAbs_REVERSED)
                {
                    Ori = TopAbs.Reverse(Ori);
                }

                if (isSeam1)
                {
                    if (!myNonmanifold)
                    {
                        return new TopoDS_Edge();
                    }
                    TopoDS_Shape aTmpShape = edge1.Reversed(); // for porting
                    c2d11 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aTmpShape), fac1, out first2d, out last2d);
                    // if(fac1.Orientation() == TopAbs_REVERSED) //
                    if (Ori == TopAbs_FORWARD)
                    {
                        aBuilder.UpdateEdge(edge, c2d1, c2d11, fac1, 0);
                    }
                    else
                    {
                        aBuilder.UpdateEdge(edge, c2d11, c2d1, fac1, 0);
                    }
                }
                else
                {
                    aBuilder.UpdateEdge(edge, c2d1, fac1, 0);
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
                            double pf = c2d1!.FirstParameter();
                            gp_Pnt2d p1n = c2d1.Value(Max(first, pf));
                            gp_Pnt2d p21n = c2d2.Value(Max(first, c2d2.FirstParameter()));
                            gp_Pnt2d p22n = c2d2.Value(Min(last, c2d2.LastParameter()));
                            double aDist = Min(p1n.Distance(p21n), p1n.Distance(p22n));
                            surf2!.Bounds(out double U1, out double U2, out double V1, out double V2);
                            isSeam = ((uclosed && aDist > 0.75 * (Math.Abs(U2 - U1)))
                                      || (vclosed && aDist > 0.75 * (Math.Abs(V2 - V1))));
                            if (!isSeam && BRep_Tool.IsClosed(TopoDS.Edge(edge), fac1))
                            {
                                continue;
                            }
                        }
                    }
                }

                isResEdge = true;
                if (isSeam)
                {
                    if (Ori == TopAbs_FORWARD)
                    {
                        aBuilder.UpdateEdge(edge, c2d1, c2d2, surf2, loc2, Precision.Confusion());
                    }
                    else
                    {
                        aBuilder.UpdateEdge(edge, c2d2, c2d1, surf2, loc2, Precision.Confusion());
                    }
                }
                else if (isSeam2)
                {
                    TopAbs_Orientation InitOri = edge2.Orientation();
                    TopAbs_Orientation SecOri = edge.Orientation();
                    if (fac2.Orientation() == TopAbs_REVERSED)
                    {
                        InitOri = TopAbs.Reverse(InitOri);
                        SecOri = TopAbs.Reverse(SecOri);
                    }
                    if (!secForward)
                    {
                        InitOri = TopAbs.Reverse(InitOri);
                    }

                    if (InitOri == TopAbs_FORWARD)
                    {
                        aBuilder.UpdateEdge(edge, c2d2, c2d21, surf2, loc2, Precision.Confusion());
                    }
                    else
                    {
                        aBuilder.UpdateEdge(edge, c2d21, c2d2, surf2, loc2, Precision.Confusion());
                    }
                }
                else
                {
                    aBuilder.UpdateEdge(edge, c2d2, surf2, loc2, Precision.Confusion());
                }
            }
        }
        double tolReached = Precision.Infinite();
        bool isSamePar = false;
        try
        {
            if (isResEdge)
            {
                SameParameter(edge);
            }

            if (BRep_Tool.SameParameter(edge))
            {
                isSamePar = true;
                tolReached = BRep_Tool.Tolerance(edge);
            }
        }
        catch (Standard_Failure)
        {
            isSamePar = false;
        }

        if (firstCall && (!isResEdge || !isSamePar || tolReached > myTolerance))
        {
            int whichSecn = whichSec;
            // Try to merge on the second section
            bool second_ok = false;
            TopoDS_Edge s_edge = SameParameterEdge(edgeFirst,
                                                   edgeLast,
                                                   listFacesFirst,
                                                   listFacesLast,
                                                   secForward,
                                                   ref whichSecn,
                                                   false);
            if (!s_edge.IsNull())
            {
                double tolReached_2 = BRep_Tool.Tolerance(s_edge);
                second_ok = (BRep_Tool.SameParameter(s_edge) && tolReached_2 < tolReached);
                if (second_ok)
                {
                    edge = s_edge;
                    whichSec = whichSecn;
                    tolReached = tolReached_2;
                }
            }

            if (!second_ok && !edge.IsNull())
            {
                var c3dAdapt = new GeomAdaptor_Curve(c3d!);

                // Discretize edge curve
                int i, j, nbp = 23;
                double deltaT = (last3d - first3d) / (nbp - 1);
                var c3dpnt = new NCollection_Array1<gp_Pnt>(1, nbp);
                for (i = 1; i <= nbp; i++)
                {
                    c3dpnt[i] = c3dAdapt.Value(first3d + (i - 1) * deltaT);
                }

                double dist = 0.0, maxTol = -1.0;
                bool more = true;

                for (j = 1; more; j++)
                {
                    Geom2d_Curve? c2d2 = null;
                    BRep_Tool.CurveOnSurface(edge, ref c2d2, ref surf2, ref loc2, ref first, ref last, j);

                    more = c2d2 is not null;
                    if (more)
                    {
                        Geom_Surface aS = surf2!;
                        if (!loc2.IsIdentity())
                        {
                            aS = (Geom_Surface)surf2!.Transformed(loc2.Transformation());
                        }

                        double dist2 = 0.0;
                        deltaT = (last - first) / (nbp - 1);
                        for (i = 1; i <= nbp; i++)
                        {
                            gp_Pnt2d aP2d = c2d2!.Value(first + (i - 1) * deltaT);
                            aS.D0(aP2d.X(), aP2d.Y(), out gp_Pnt aP2);
                            gp_Pnt aP1 = c3dpnt[i];
                            dist = aP2.SquareDistance(aP1);
                            if (dist > dist2)
                            {
                                dist2 = dist;
                            }
                        }
                        maxTol = Max(Math.Sqrt(dist2) * (1.0 + 1e-7), Precision.Confusion());
                    }
                }
                if (maxTol >= 0.0 && maxTol < tolReached)
                {
                    if (tolReached > MaxTolerance())
                    {
                        // Set tolerance directly to overwrite too large tolerance
                        ((BRep_TEdge)edge.TShape()!).Tolerance(maxTol);
                    }
                    else
                    {
                        // just update tolerance with computed distance
                        aBuilder.UpdateEdge(edge, maxTol);
                    }
                }
                aBuilder.SameParameter(edge, true);
            }
        }

        double tolEdge1 = BRep_Tool.Tolerance(edge);
        if (tolEdge1 > MaxTolerance())
        {
            edge.Nullify();
        }
        return edge;
    }

    //=================================================================================================

    /// <summary>Evaluates the mean angles between the normals along the sections. Called from MergingOfSections only.</summary>
    protected virtual void EvaluateAngulars(NCollection_Sequence<TopoDS_Shape> sequenceSec,
                                            NCollection_Array1<bool> secForward,
                                            NCollection_Array1<double> tabAng,
                                            int indRef)
    {
        tabAng.Init(-1.0);

        int i, j, npt = 4, lengSec = sequenceSec.Length();

        TopoDS_Edge edge;
        TopoDS_Face face;
        TopLoc_Location loc;
        double first = 0.0, last = 0.0;
        Geom_Curve? c3d;
        Geom2d_Curve? c2d = null;
        Geom_Surface? surf = null;
        var normRef = new NCollection_Array1<gp_Vec>(1, npt);

        for (i = indRef; i <= lengSec; i++)
        {
            edge = TopoDS.Edge(sequenceSec[i]);

            TopoDS_Shape bnd = edge;
            if (mySectionBound.IsBound(bnd))
            {
                bnd = mySectionBound[bnd];
            }
            if (myBoundFaces.Contains(bnd))
            {
                face = TopoDS.Face(myBoundFaces.FindFromKey(bnd).First());
                surf = BRep_Tool.Surface(face, out loc);
                if (!loc.IsIdentity())
                {
                    surf = (Geom_Surface)surf!.Copy();
                    surf.Transform(loc.Transformation());
                }
                c2d = BRep_Tool.CurveOnSurface(edge, face, out first, out last);
            }
            else if (i == indRef)
            {
                return;
            }
            else
            {
                continue;
            }

            c3d = BRep_Tool.Curve(edge, out loc, out first, out last);
            if (!loc.IsIdentity())
            {
                c3d = (Geom_Curve)c3d!.Copy();
                c3d.Transform(loc.Transformation());
            }

            var adapt = new GeomAdaptor_Curve(c3d!);
            var uniAbs = new GCPnts_UniformAbscissa(adapt, npt, first, last);

            double cumulateAngular = 0.0;
            int nbComputedAngle = 0;

            for (j = 1; j <= npt; j++)
            {
                c2d!.D0(uniAbs.Parameter((secForward[i] || i == indRef) ? j : npt - j + 1), out gp_Pnt2d P);
                surf!.D1(P.X(), P.Y(), out _, out gp_Vec w1, out gp_Vec w2);
                gp_Vec n = w1 ^ w2; // Compute the normal vector
                if (i == indRef)
                {
                    normRef[j] = n;
                }
                else if ((n.Magnitude() > gp.Resolution()) && (normRef[j].Magnitude() > gp.Resolution()))
                {
                    nbComputedAngle++;
                    double angular = n.Angle(normRef[j]);
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
    protected virtual void EvaluateDistances(NCollection_Sequence<TopoDS_Shape> sequenceSec,
                                             NCollection_Array1<bool> secForward,
                                             NCollection_Array1<double> tabDst,
                                             NCollection_Array1<double> arrLen,
                                             NCollection_Array1<double> tabMinDist,
                                             int indRef)
    {
        secForward.Init(true);
        tabDst.Init(-1.0);
        arrLen.Init(0.0);
        tabMinDist.Init(Precision.Infinite());
        const int npt = 8; // Number of points for curve discretization
        var ptsRef = new NCollection_Array1<gp_Pnt>(1, npt);
        var ptsSec = new NCollection_Array1<gp_Pnt>(1, npt);

        int i, j, lengSec = sequenceSec.Length();

        Geom_Curve? c3dRef = null;
        double firstRef = 0.0, lastRef = 0.0;

        for (i = indRef; i <= lengSec; i++)
        {
            // reading of the edge (attention for the first one: reference)
            TopoDS_Edge sec = TopoDS.Edge(sequenceSec[i]);

            Geom_Curve? c3d = BRep_Tool.Curve(sec, out TopLoc_Location loc, out double first, out double last);
            if (c3d is null)
            {
                continue;
            }
            if (!loc.IsIdentity())
            {
                c3d = (Geom_Curve)c3d.Copy();
                c3d.Transform(loc.Transformation());
            }

            if (i == indRef)
            {
                c3dRef = c3d;
                firstRef = first;
                lastRef = last;
            }

            double dist = Precision.Infinite(), distFor = -1.0, distRev = -1.0;
            double aMinDist = Precision.Infinite();

            double T, deltaT = (last - first) / (npt - 1);
            double aLenSec2 = 0.0;

            int nbFound = 0;
            for (j = 1; j <= npt; j++)
            {
                // Uniform parameter on curve
                if (j == 1)
                {
                    T = first;
                }
                else if (j == npt)
                {
                    T = last;
                }
                else
                {
                    T = first + (j - 1) * deltaT;
                }

                // Take point on curve
                gp_Pnt pt = c3d.Value(T);

                if (i == indRef)
                {
                    ptsRef[j] = pt;
                    if (j > 1)
                    {
                        aLenSec2 += pt.SquareDistance(ptsRef[j - 1]);
                    }
                }
                else
                {
                    ptsSec[j] = pt;
                    // protection to avoid merging with small sections
                    if (j > 1)
                    {
                        aLenSec2 += pt.SquareDistance(ptsSec[j - 1]);
                    }
                    // To evaluate mutual orientation and distance
                    dist = pt.Distance(ptsRef[j]);
                    if (aMinDist > dist)
                    {
                        aMinDist = dist;
                    }
                    if (distFor < dist)
                    {
                        distFor = dist;
                    }
                    dist = pt.Distance(ptsRef[npt - j + 1]);

                    if (aMinDist > dist)
                    {
                        aMinDist = dist;
                    }
                    if (distRev < dist)
                    {
                        distRev = dist;
                    }

                    // Check that point lays between vertices of reference curve
                    gp_Pnt p11 = ptsRef[1];
                    gp_Pnt p12 = ptsRef[npt];
                    var aVec1 = new gp_Vec(pt, p11);
                    var aVec2 = new gp_Vec(pt, p12);
                    var aVecRef = new gp_Vec(p11, p12);
                    if ((aVecRef * aVec1) * (aVecRef * aVec2) < 0.0)
                    {
                        nbFound++;
                    }
                }
            }

            double aLenSec = Math.Sqrt(aLenSec2);

            // if(aLenSec < myMinTolerance )
            //  continue;
            arrLen.SetValue(i, aLenSec);
            // Record mutual orientation
            bool isForward = (distFor < distRev); // szv debug: <=
            secForward[i] = isForward;

            dist = (isForward ? distFor : distRev);
            if (i == indRef || (dist < myTolerance && nbFound >= npt * 0.5))
            {
                tabDst[i] = dist;
                tabMinDist[i] = aMinDist;
            }
            else
            {
                nbFound = 0;
                aMinDist = Precision.Infinite();
                dist = -1;
                var arrProj = new NCollection_Array1<gp_Pnt>(1, npt);
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
                    if (aMinDist > arrDist[j])
                    {
                        aMinDist = arrDist[j];
                    }
                    nbFound++;
                }
                if (nbFound > 1)
                {
                    tabDst[i] = dist;
                    tabMinDist[i] = aMinDist;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the two edges, on a closed surface of the face, would make a merged closed edge.</summary>
    protected bool IsMergedClosed(in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, in TopoDS_Face face)
    {
        // Check for closed surface
        Geom_Surface? surf = BRep_Tool.Surface(face, out TopLoc_Location loc);
        bool isUClosed = IsUClosedSurface(surf!, Edge1, loc);
        bool isVClosed = IsVClosedSurface(surf!, Edge1, loc);
        if (!isUClosed && !isVClosed)
        {
            return false;
        }
        // Check condition on closed surface
        Geom2d_Curve? C2d1 = BRep_Tool.CurveOnSurface(Edge1, face, out double first2d1, out double last2d1);
        Geom2d_Curve? C2d2 = BRep_Tool.CurveOnSurface(Edge2, face, out double first2d2, out double last2d2);
        if (C2d1 is null || C2d2 is null)
        {
            return false;
        }
        bool isULongC1, isULongC2, isVLongC1, isVLongC2;
        double SUmin, SUmax, SVmin, SVmax;
        double C1Umin, C1Vmin, C1Umax, C1Vmax;
        double C2Umin, C2Vmin, C2Umax, C2Vmax;
        { // szv: Use brackets to destroy local variables
            Bnd_Box2d B1 = new(), B2 = new();
            var aC2d1 = new Geom2dAdaptor_Curve(C2d1);
            var aC2d2 = new Geom2dAdaptor_Curve(C2d2);
            BndLib_Add2dCurve.Add(aC2d1, first2d1, last2d1, Precision.PConfusion(), ref B1);
            BndLib_Add2dCurve.Add(aC2d2, first2d2, last2d2, Precision.PConfusion(), ref B2);
            B1.Get(out C1Umin, out C1Vmin, out C1Umax, out C1Vmax);
            B2.Get(out C2Umin, out C2Vmin, out C2Umax, out C2Vmax);
            double du, dv;
            du = (C1Umax - C1Umin);
            dv = (C1Vmax - C1Vmin);
            isULongC1 = (dv <= du);
            isVLongC1 = (du <= dv);
            du = (C2Umax - C2Umin);
            dv = (C2Vmax - C2Vmin);
            isULongC2 = (dv <= du);
            isVLongC2 = (du <= dv);
            surf!.Bounds(out SUmin, out SUmax, out SVmin, out SVmax);
        }
        if (isUClosed && isVLongC1 && isVLongC2)
        {
            // Do not merge if not overlapped by V
            double dist = Max((C2Vmin - C1Vmax), (C1Vmin - C2Vmax));
            if (dist < 0.0)
            {
                double distInner = Max((C2Umin - C1Umax), (C1Umin - C2Umax));
                double distOuter = (SUmax - SUmin) - Max((C2Umax - C1Umin), (C1Umax - C2Umin));
                if (distOuter <= distInner)
                {
                    return true;
                }
            }
        }
        if (isVClosed && isULongC1 && isULongC2)
        {
            // Do not merge if not overlapped by U
            double dist = Max((C2Umin - C1Umax), (C1Umin - C2Umax));
            if (dist < 0.0)
            {
                double distInner = Max((C2Vmin - C1Vmax), (C1Vmin - C2Vmax));
                double distOuter = (SVmax - SVmin) - Max((C2Vmax - C1Vmin), (C1Vmax - C2Vmin));
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
    protected void AnalysisNearestEdges(NCollection_Sequence<TopoDS_Shape> sequenceSec,
                                        NCollection_Sequence<int> seqIndCandidate,
                                        NCollection_Sequence<bool> seqOrientations,
                                        bool evalDist = true)
    {
        int workIndex = seqIndCandidate.First();
        TopoDS_Shape workedge = sequenceSec.Value(workIndex);
        TopoDS_Shape bnd = workedge;
        var workfaces = new NCollection_List<TopoDS_Shape>();
        if (mySectionBound.IsBound(bnd))
        {
            bnd = mySectionBound[bnd];
        }
        if (myBoundFaces.Contains(bnd))
        {
            workfaces.Assign(myBoundFaces.FindFromKey(bnd));
        }
        if (workfaces.IsEmpty())
        {
            return;
        }
        var mapFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var lIt = new NCollection_List<TopoDS_Shape>.Iterator();
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
                TopoDS_Shape bnd2 = sequenceSec.Value(index);
                if (mySectionBound.IsBound(bnd2))
                {
                    bnd2 = mySectionBound[bnd2];
                }

                if (myBoundFaces.Contains(bnd2))
                {
                    NCollection_List<TopoDS_Shape> listfaces = myBoundFaces.FindFromKey(bnd2);
                    bool isMerged = true;
                    for (lIt.Initialize(listfaces); lIt.More() && isMerged; lIt.Next())
                    {
                        if (mapFaces.Contains(lIt.Value()))
                        {
                            Geom_Surface? surf = BRep_Tool.Surface(TopoDS.Face(lIt.Value()), out TopLoc_Location loc);
                            isMerged = ((IsUClosedSurface(surf!, bnd2, loc) || IsVClosedSurface(surf!, bnd2, loc))
                                        && IsMergedClosed(TopoDS.Edge(sequenceSec.Value(index)),
                                                          TopoDS.Edge(workedge),
                                                          TopoDS.Face(lIt.Value())));
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
        var TotTabDist = new NCollection_Array2<double>(1, seqNotCandidate.Length(), 1, seqIndCandidate.Length());
        var MapIndex = new NCollection_Map<int>();
        var seqForward = new NCollection_Sequence<int>();

        // Definition and removing edges which are not candidate for work edge
        // (they have other nearest edges belonging to the work face)
        for (int k = 1; k <= seqNotCandidate.Length(); k++)
        {
            int index1 = seqNotCandidate.Value(k);
            TopoDS_Shape edge = sequenceSec.Value(index1);
            var tmpSeq = new NCollection_Sequence<TopoDS_Shape>();
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
                    if (tabDist[n + 1] == -1 || tabDist[n + 1] > myTolerance)
                    {
                        MapIndex.Add(n);
                        continue;
                    }
                    TotTabDist[k, n] = tabDist[n + 1];
                    seqForward.Append(tabForward[n + 1] ? 1 : 0);
                }
            }
            else
            {
                for (int n = 1; n < lengSec; n++)
                {
                    if (tabDist[n] == -1 || tabDist[n] > myTolerance)
                    {
                        continue;
                    }
                    if (tabDist[n + 1] < TotTabDist[1, n])
                    {
                        MapIndex.Add(n);
                    }
                }
            }
        }

        int i2 = seqIndCandidate.Length();
        for (; i2 >= 1; i2--)
        {
            if (MapIndex.Contains(i2))
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
    protected bool FindCandidates(NCollection_Sequence<TopoDS_Shape> seqSections,
                                  NCollection_IndexedMap<int> mapReference,
                                  NCollection_Sequence<int> seqCandidates,
                                  NCollection_Sequence<bool> seqOrientations)
    {
        int i, nbSections = seqSections.Length();
        if (nbSections <= 1)
        {
            return false;
        }
        // Retrieve last reference index
        int indReference = mapReference[mapReference.Extent()];
        int nbCandidates = 0;
        var Faces1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        // if (nbSections > 1) {

        TopoDS_Edge Edge1 = TopoDS.Edge(seqSections[indReference]);

        // Retrieve faces for reference section

        { // szv: Use brackets to destroy local variables
            TopoDS_Shape bnd = Edge1;
            if (mySectionBound.IsBound(bnd))
            {
                bnd = mySectionBound[bnd];
            }
            if (myBoundFaces.Contains(bnd))
            {
                var itf1 = new NCollection_List<TopoDS_Shape>.Iterator(myBoundFaces.FindFromKey(bnd));
                for (; itf1.More(); itf1.Next())
                {
                    Faces1.Add(itf1.Value());
                }
            }
        }

        // Check merging conditions for candidates and remove unsatisfactory
        var seqSectionsNew = new NCollection_Sequence<TopoDS_Shape>();
        var seqCandidatesNew = new NCollection_Sequence<int>();
        for (i = 1; i <= nbSections; i++)
        {
            if (i == indReference)
            {
                seqSectionsNew.Prepend(Edge1);
                seqCandidatesNew.Prepend(i);
            }
            else
            {
                TopoDS_Edge Edge2 = TopoDS.Edge(seqSections[i]);
                seqSectionsNew.Append(Edge2);
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
                double aMaxDist = arrDistance[i];
                if (aMaxDist >= 0.0 && aMaxDist <= myTolerance && arrLen[i] > myMinTolerance)
                {
                    // Reference section is connected to section #i
                    bool isInserted = false;
                    bool ori = arrForward[i];
                    for (int j = 1; (j <= seqCandidates.Length()) && !isInserted; j++)
                    {
                        double aDelta = arrDistance[i] - arrDistance[seqCandidates.Value(j)];

                        if (aDelta < Precision.Confusion())
                        {
                            if (Math.Abs(aDelta) > RealSmall() || arrMinDist[i] < arrMinDist[seqCandidates.Value(j)])
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

        if (myNonmanifold && nbCandidates > 1)
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
            int anInd = seqCandidates.Value(1);
            TopoDS_Edge Edge2 = TopoDS.Edge(seqSections[anInd]);
            TopoDS_Shape bnd = Edge2;
            if (mySectionBound.IsBound(bnd))
            {
                bnd = mySectionBound[bnd];
            }
            // gka
            if (myBoundFaces.Contains(bnd))
            {
                bool isOK = true;
                var itf2 = new NCollection_List<TopoDS_Shape>.Iterator(myBoundFaces.FindFromKey(bnd));
                for (; itf2.More() && isOK; itf2.Next())
                {
                    TopoDS_Face Face2 = TopoDS.Face(itf2.Value());
                    // Check whether condition is satisfied
                    isOK = !Faces1.Contains(Face2);
                    if (!isOK)
                    {
                        isOK = IsMergedClosed(Edge1, Edge2, Face2);
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
        myTolerance = Max(tolerance, Precision.Confusion());
        mySewing = optionSewing;
        myAnalysis = optionAnalysis;
        myCutting = optionCutting;
        myNonmanifold = optionNonmanifold;
        // Set min and max tolerances
        myMinTolerance = myTolerance * 1e-4; // szv: proposal
        if (myMinTolerance < Precision.Confusion())
        {
            myMinTolerance = Precision.Confusion();
        }
        myMaxTolerance = Precision.Infinite();
        // Set other modes
        myFaceMode = true;
        myFloatingEdgesMode = false;
        // myCuttingFloatingEdgesMode = false; //gka
        mySameParameterMode = true;
        myLocalToleranceMode = false;
        mySewedShape.Nullify();
        // Load empty shape
        Load(new TopoDS_Shape());
    }

    //=================================================================================================

    /// <summary>Loads the context shape (and clears the inputs and results).</summary>
    public void Load(in TopoDS_Shape theShape)
    {
        myReShape.Clear();
        if (theShape.IsNull())
        {
            myShape.Nullify();
        }
        else
        {
            myShape = myReShape.InternalApply(theShape);
        }
        mySewedShape.Nullify();
        // Nullify flags and counters
        myNbShapes = myNbEdges = myNbVertices = 0;
        // Clear all maps
        myOldShapes.Clear();
        // myOldFaces.Clear();
        myDegenerated.Clear();
        myFreeEdges.Clear();
        myMultipleEdges.Clear();
        myContigousEdges.Clear();
        myContigSecBound.Clear();
        myBoundFaces.Clear();
        myBoundSections.Clear();
        myVertexNode.Clear();
        myVertexNodeFree.Clear();
        myNodeSections.Clear();
        myCuttingNode.Clear();
        mySectionBound.Clear();
        myLittleFace.Clear();
        // OCCT.NET: forget a copy at entry.
        myWorkCopies = null;
        myWorkReplaced = null;
    }

    //=================================================================================================

    /// <summary>Adds a shape to be sewn or controlled.</summary>
    public void Add(in TopoDS_Shape aShape)
    {
        if (aShape.IsNull())
        {
            return;
        }
        TopoDS_Shape oShape = myReShape.InternalApply(aShape);
        myOldShapes.Add(aShape, oShape);
        myNbShapes = myOldShapes.Extent();
    }

    //=================================================================================================

    /// <summary>Computes the sewing (or only the analysis, if the sewing option is off).</summary>
    /// <remarks>
    /// When an input holds a frozen shape, the computation runs on an exact copy of the inputs, and the caller's own
    /// sub-shapes are given back wherever sewing left them unchanged (decision 583). The inputs are never written.
    /// </remarks>
    public void Perform(Message_ProgressRange theProgress = default)
    {
        // OCCT.NET: copy at entry (decision 583).
        bool isCopied = beginWork(out TopoDS_Shape anOriginals, out TopoDS_Shape aCopy,
                                  out Dictionary<TopoDS_TShape, TopoDS_TShape>? aCopies);
        performSewing(theProgress);
        if (isCopied)
        {
            endWork(anOriginals, aCopy, aCopies!);
        }
    }

    // OCCT's Perform.
    private void performSewing(Message_ProgressRange theProgress)
    {
        int aNumberOfStages = (myAnalysis ? 1 : 0) + (myCutting ? 1 : 0) + (mySewing ? 1 : 0) + 2;
        using var aPS = new Message_ProgressScope(theProgress, "Sewing", aNumberOfStages);

        // face analysis
        if (myAnalysis)
        {
            FaceAnalysis(aPS.Next());
            if (!aPS.More())
            {
                return;
            }
        }

        if (myNbShapes != 0 || !myShape.IsNull())
        {
            FindFreeBoundaries();

            if (myBoundFaces.Extent() != 0)
            {
                VerticesAssembling(aPS.Next());
                if (!aPS.More())
                {
                    return;
                }
                if (myCutting)
                {
                    Cutting(aPS.Next());
                    if (!aPS.More())
                    {
                        return;
                    }
                }
                Merging(true, aPS.Next());
                if (!aPS.More())
                {
                    return;
                }
            }
            else
            {
                aPS.Next();
                if (myCutting)
                {
                    aPS.Next();
                }
                aPS.Next();
                if (!aPS.More())
                {
                    return;
                }
            }

            if (mySewing)
            {
                // examine the multiple edges if any and process sameparameter for edges if necessary
                EdgeProcessing(aPS.Next());
                if (!aPS.More())
                {
                    return;
                }
                CreateSewedShape();
                if (!aPS.More())
                {
                    mySewedShape.Nullify();
                    return;
                }

                EdgeRegularity(aPS.Next());

                if (mySameParameterMode && myFaceMode)
                {
                    SameParameterShape();
                }
                if (!aPS.More())
                {
                    mySewedShape.Nullify();
                    return;
                }
            }

            // create edge information for output
            CreateOutputInformations();
            if (!aPS.More())
            {
                mySewedShape.Nullify();
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
    public TopoDS_Shape SewedShape() => published(mySewedShape);

    /// <summary>The sewn shape, as <see cref="SewedShape"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalSewedShape() => fromWork(mySewedShape);

    //=================================================================================================

    /// <summary>The number of free edges (edges shared by one face).</summary>
    public int NbFreeEdges() => myFreeEdges.Extent();

    /// <summary>The <paramref name="index"/>-th free edge, frozen.</summary>
    public TopoDS_Edge FreeEdge(int index) => TopoDS.Edge(published(InternalFreeEdge(index)));

    /// <summary>The <paramref name="index"/>-th free edge, without freezing it (ported code).</summary>
    internal TopoDS_Edge InternalFreeEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbFreeEdges(), "BRepBuilderAPI_Sewing::FreeEdge");
        return TopoDS.Edge(fromWork(myFreeEdges[index]));
    }

    //=================================================================================================

    /// <summary>The number of multiple edges (edges shared by more than two faces).</summary>
    public int NbMultipleEdges() => myMultipleEdges.Extent();

    /// <summary>The <paramref name="index"/>-th multiple edge, frozen.</summary>
    public TopoDS_Edge MultipleEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbMultipleEdges(), "BRepBuilderAPI_Sewing::MultipleEdge");
        return TopoDS.Edge(published(myMultipleEdges[index]));
    }

    //=================================================================================================

    /// <summary>The number of contiguous edges (edges shared by two faces).</summary>
    public int NbContigousEdges() => myContigousEdges.Extent();

    /// <summary>The <paramref name="index"/>-th contiguous edge, frozen.</summary>
    public TopoDS_Edge ContigousEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbContigousEdges(), "BRepBuilderAPI_Sewing::ContigousEdge");
        return TopoDS.Edge(published(myContigousEdges.FindKey(index)));
    }

    //=================================================================================================

    /// <summary>The sections (edges) of the <paramref name="index"/>-th contiguous edge, frozen, in a new list.</summary>
    public ISchematiqList<TopoDS_Shape> ContigousEdgeCouple(int index)
    {
        NCollection_List<TopoDS_Shape> aList = InternalContigousEdgeCouple(index);
        var aShapes = new TopoDS_Shape[aList.Extent()];
        int i = 0;
        for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(aList); anIt.More(); anIt.Next())
        {
            aShapes[i++] = published(anIt.Value());
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>The sections of the <paramref name="index"/>-th contiguous edge: the algorithm's own list (ported code).</summary>
    internal NCollection_List<TopoDS_Shape> InternalContigousEdgeCouple(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbContigousEdges(),
                                     "BRepBuilderAPI_Sewing::ContigousEdgeCouple");
        return myContigousEdges[index];
    }

    //=================================================================================================

    /// <summary>True if the section is bound to a boundary (see <see cref="SectionToBoundary"/>).</summary>
    public bool IsSectionBound(in TopoDS_Edge section) => myContigSecBound.IsBound(toWork(section));

    //=================================================================================================

    /// <summary>
    /// The original edge (free boundary) that became the section, frozen. Sections make the common edges; the original
    /// edge gives the surface the section is attached to.
    /// </summary>
    /// <exception cref="Standard_NoSuchObject">The section isn't bound.</exception>
    public TopoDS_Edge SectionToBoundary(in TopoDS_Edge section)
    {
        Standard_NoSuchObject_Raise_if(!IsSectionBound(section), "BRepBuilderAPI_Sewing::SectionToBoundary");
        return TopoDS.Edge(published(myContigSecBound[toWork(section)]));
    }

    //=================================================================================================

    /// <summary>The number of deleted faces (faces smaller than the tolerance).</summary>
    public int NbDeletedFaces() => myLittleFace.Extent();

    /// <summary>The <paramref name="index"/>-th deleted face, frozen.</summary>
    public TopoDS_Face DeletedFace(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbDeletedFaces(), "BRepBuilderAPI_Sewing::DeletedFace");
        return TopoDS.Face(published(myLittleFace[index]));
    }

    //=================================================================================================

    /// <summary>The number of degenerated shapes.</summary>
    public int NbDegeneratedShapes() => myDegenerated.Extent();

    /// <summary>The <paramref name="index"/>-th degenerated shape, frozen.</summary>
    public TopoDS_Shape DegeneratedShape(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbDegeneratedShapes(),
                                     "BRepBuilderAPI_Sewing::DegereratedShape");
        return published(myDegenerated[index]);
    }

    //=================================================================================================

    /// <summary>True if the input shape is degenerated.</summary>
    public bool IsDegenerated(in TopoDS_Shape aShape)
    {
        TopoDS_Shape NewShape = myReShape.InternalApply(toWork(aShape));
        // Degenerated face
        if (aShape.ShapeType() == TopAbs_FACE)
        {
            return NewShape.IsNull();
        }
        if (NewShape.IsNull())
        {
            return false;
        }
        // Degenerated edge
        if (NewShape.ShapeType() == TopAbs_EDGE)
        {
            return BRep_Tool.Degenerated(TopoDS.Edge(NewShape));
        }
        // Degenerated wire
        if (NewShape.ShapeType() == TopAbs_WIRE)
        {
            bool isDegenerated = true;
            for (var aIt = new TopoDS_Iterator(NewShape); aIt.More() && isDegenerated; aIt.Next())
            {
                isDegenerated = BRep_Tool.Degenerated(TopoDS.Edge(aIt.Value()));
            }
            return isDegenerated;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>True if the input shape has been modified.</summary>
    public bool IsModified(in TopoDS_Shape aShape)
    {
        TopoDS_Shape NewShape = aShape;
        if (myOldShapes.Contains(aShape))
        {
            NewShape = fromWork(myOldShapes.FindFromKey(aShape));
        }
        if (!NewShape.IsSame(aShape))
        {
            return true;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>The modified input shape, frozen (the shape itself if it isn't an input).</summary>
    public TopoDS_Shape Modified(in TopoDS_Shape aShape)
    {
        if (myOldShapes.Contains(aShape))
        {
            return published(myOldShapes.FindFromKey(aShape));
        }
        // if (myOldFaces.Contains(aShape)) return myOldFaces.FindFromKey(aShape);
        return aShape;
    }

    /// <summary>The modified input shape, as <see cref="Modified"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalModified(in TopoDS_Shape aShape)
    {
        if (myOldShapes.Contains(aShape))
        {
            return fromWork(myOldShapes.FindFromKey(aShape));
        }
        return aShape;
    }

    //=================================================================================================

    /// <summary>True if the input sub-shape has been modified.</summary>
    public bool IsModifiedSubShape(in TopoDS_Shape aShape)
    {
        TopoDS_Shape NewShape = fromWork(myReShape.InternalApply(toWork(aShape)));
        return !NewShape.IsSame(aShape);
    }

    //=================================================================================================

    /// <summary>The modified sub-shape, frozen (the sub-shape itself if it isn't modified).</summary>
    public TopoDS_Shape ModifiedSubShape(in TopoDS_Shape aShape) =>
        published(myReShape.InternalApply(toWork(aShape)));

    /// <summary>The modified sub-shape, as <see cref="ModifiedSubShape"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalModifiedSubShape(in TopoDS_Shape aShape) =>
        fromWork(myReShape.InternalApply(toWork(aShape)));

    //=================================================================================================

    /// <summary>Prints the information on the console.</summary>
    public void Dump() => Dump(Console.Out);

    /// <summary>Prints the information (OCCT writes it to <c>std::cout</c>).</summary>
    public void Dump(TextWriter theOS)
    {
        int i, NbBounds = myBoundFaces.Extent(), NbSections = 0;
        var mapVertices = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var mapEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= NbBounds; i++)
        {
            TopoDS_Shape bound = myBoundFaces.FindKey(i);
            if (myBoundSections.IsBound(bound))
            {
                NbSections += myBoundSections[bound].Extent();
            }
            else
            {
                NbSections++;
            }
            var aExp = new TopExp_Explorer(myReShape.InternalApply(bound), TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(aExp.Current());
                mapEdges.Add(E);
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                mapVertices.Add(V1);
                mapVertices.Add(V2);
            }
        }
        theOS.Write(" " + '\n');
        theOS.Write("                        Information                         " + '\n');
        theOS.Write(" ===========================================================" + '\n');
        theOS.Write(" " + '\n');
        theOS.Write(" Number of input shapes      : " + myOldShapes.Extent() + '\n');
        theOS.Write(" Number of actual shapes     : " + myNbShapes + '\n');
        theOS.Write(" Number of Bounds            : " + NbBounds + '\n');
        theOS.Write(" Number of Sections          : " + NbSections + '\n');
        theOS.Write(" Number of Edges             : " + mapEdges.Extent() + '\n');
        theOS.Write(" Number of Vertices          : " + myNbVertices + '\n');
        theOS.Write(" Number of Nodes             : " + mapVertices.Extent() + '\n');
        theOS.Write(" Number of Free Edges        : " + myFreeEdges.Extent() + '\n');
        theOS.Write(" Number of Contiguous Edges  : " + myContigousEdges.Extent() + '\n');
        theOS.Write(" Number of Multiple Edges    : " + myMultipleEdges.Extent() + '\n');
        theOS.Write(" Number of Degenerated Edges : " + myDegenerated.Extent() + '\n');
        theOS.Write(" ===========================================================" + '\n');
        theOS.Write(" " + '\n');
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
    protected virtual void FaceAnalysis(Message_ProgressRange theProgress = default)
    {
        if (!myShape.IsNull() && myOldShapes.IsEmpty())
        {
            Add(myShape);
            myShape.Nullify();
        }

        var B = new BRep_Builder();
        var SmallEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var GluedVertices = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        int i = 1;
        using var aPS = new Message_ProgressScope(theProgress, "Shape analysis", myOldShapes.Extent());
        for (i = 1; i <= myOldShapes.Extent() && aPS.More(); i++, aPS.Next())
        {
            for (var fexp = new TopExp_Explorer(myOldShapes[i], TopAbs_FACE); fexp.More(); fexp.Next())
            {
                // Retrieve current face
                TopoDS_Shape aTmpShape = fexp.Current(); // for porting
                TopoDS_Face face = TopoDS.Face(aTmpShape);
                int nbEdges = 0, nbSmall = 0;

                // Build replacing face
                aTmpShape = face.EmptyCopied().Oriented(TopAbs_FORWARD); // for porting
                TopoDS_Face nface = TopoDS.Face(aTmpShape);
                bool isFaceChanged = false;

                var witer = new TopoDS_Iterator(face.Oriented(TopAbs_FORWARD));
                for (; witer.More(); witer.Next())
                {
                    // Retrieve current wire
                    aTmpShape = witer.Value(); // for porting
                    if (aTmpShape.ShapeType() != TopAbs_WIRE)
                    {
                        continue;
                    }
                    TopoDS_Wire wire = TopoDS.Wire(aTmpShape);

                    // Build replacing wire
                    aTmpShape = wire.EmptyCopied().Oriented(TopAbs_FORWARD);
                    TopoDS_Wire nwire = TopoDS.Wire(aTmpShape);
                    bool isWireChanged = false;

                    var eiter = new TopoDS_Iterator(wire.Oriented(TopAbs_FORWARD));
                    for (; eiter.More(); eiter.Next())
                    {
                        // Retrieve current edge
                        aTmpShape = eiter.Value(); // for porting
                        TopoDS_Edge edge = TopoDS.Edge(aTmpShape);
                        nbEdges++;

                        // Process degenerated edge
                        if (BRep_Tool.Degenerated(edge))
                        {
                            B.Add(nwire, edge); // Old edge kept
                            myDegenerated.Add(edge);
                            nbSmall++;
                            continue;
                        }

                        bool isSmall = SmallEdges.Contains(edge);
                        if (!isSmall)
                        {
                            // Check for small edge
                            Geom_Curve? c3d = BRep_Tool.Curve(edge, out double first, out double last);
                            if (c3d is null)
                            {
                                // Warning: Possibly small edge can be sewed: No 3D curve
                            }
                            else
                            {
                                // Evaluate curve compactness
                                const int npt = 5;
                                var cp = new gp_Pnt((c3d.Value(first).XYZ() + c3d.Value(last).XYZ()) * 0.5);
                                double dist, maxdist = 0.0;
                                double delta = (last - first) / (npt - 1);
                                for (int idx = 0; idx < npt; idx++)
                                {
                                    dist = cp.Distance(c3d.Value(first + idx * delta));
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
                                SmallEdges.Add(edge);

                                TopExp.Vertices(edge, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
                                TopoDS_Shape nv1 = myReShape.InternalApply(v1), nv2 = myReShape.InternalApply(v2);

                                // Store glued vertices
                                if (!nv1.IsSame(v1))
                                {
                                    NCollection_List<TopoDS_Shape> vlist1 = GluedVertices.ChangeFromKey(nv1);
                                    // First vertex was already glued
                                    if (!nv2.IsSame(v2))
                                    {
                                        // Merge lists of glued vertices
                                        if (!nv1.IsSame(nv2))
                                        {
                                            var liter = new NCollection_List<TopoDS_Shape>.Iterator(GluedVertices.FindFromKey(nv2));
                                            for (; liter.More(); liter.Next())
                                            {
                                                TopoDS_Shape v = liter.Value();
                                                myReShape.Replace(v, nv1.Oriented(v.Orientation()));
                                                vlist1.Append(v);
                                            }
                                            GluedVertices.RemoveKey(nv2);
                                        }
                                    }
                                    else
                                    {
                                        // Add second vertex to the existing list
                                        vlist1.Append(v2);
                                        myReShape.Replace(v2, nv1.Oriented(v2.Orientation()));
                                    }
                                }
                                else if (!nv2.IsSame(v2))
                                {
                                    // Add first vertex to the existing list
                                    GluedVertices.ChangeFromKey(nv2).Append(v1);
                                    myReShape.Replace(v1, nv2.Oriented(v1.Orientation()));
                                }
                                else if (!v1.IsSame(v2))
                                {
                                    // Record new glued vertices
                                    var nv = new TopoDS_Vertex();
                                    B.MakeVertex(ref nv);
                                    var vlist = new NCollection_List<TopoDS_Shape>();
                                    vlist.Append(v1);
                                    vlist.Append(v2);
                                    GluedVertices.Add(nv, vlist);
                                    myReShape.Replace(v1, nv.Oriented(v1.Orientation()));
                                    myReShape.Replace(v2, nv.Oriented(v2.Orientation()));
                                }
                            }
                        }

                        // Replace small edge
                        if (isSmall)
                        {
                            nbSmall++;
                            // Create new degenerated edge
                            aTmpShape = edge.Oriented(TopAbs_FORWARD);
                            TopoDS_Edge fedge = TopoDS.Edge(aTmpShape);
                            Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(fedge, face, out double pfirst, out double plast);
                            if (c2d is not null)
                            {
                                var nedge = new TopoDS_Edge();
                                B.MakeEdge(ref nedge);
                                B.UpdateEdge(nedge, c2d, face, Precision.Confusion());
                                B.Range(nedge, pfirst, plast);
                                B.Degenerated(nedge, true);
                                TopExp.Vertices(fedge, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
                                B.Add(nedge, myReShape.InternalApply(v1).Oriented(v1.Orientation()));
                                B.Add(nedge, myReShape.InternalApply(v2).Oriented(v2.Orientation()));
                                B.Add(nwire, nedge.Oriented(edge.Orientation()));
                                myDegenerated.Add(nedge);
                            }
                            isWireChanged = true;
                        }
                        else
                        {
                            B.Add(nwire, edge); // Old edge kept
                        }
                    }

                    // Record wire in the new face
                    if (isWireChanged)
                    {
                        B.Add(nface, nwire.Oriented(wire.Orientation()));
                        isFaceChanged = true;
                    }
                    else
                    {
                        B.Add(nface, wire);
                    }
                }

                // Remove small face
                if (nbSmall == nbEdges)
                {
                    myLittleFace.Add(face);
                    myReShape.Remove(face);
                }
                else if (isFaceChanged)
                {
                    myReShape.Replace(face, nface.Oriented(face.Orientation()));
                }
            }
        }

        // Update glued vertices
        for (int aMIter = 1; aMIter <= GluedVertices.Extent(); aMIter++)
        {
            TopoDS_Vertex vnew = TopoDS.Vertex(GluedVertices.FindKey(aMIter));
            var coord = new gp_XYZ(0.0, 0.0, 0.0);
            int nbPoints = 0;
            NCollection_List<TopoDS_Shape> vlist = GluedVertices[aMIter];
            var liter1 = new NCollection_List<TopoDS_Shape>.Iterator(vlist);
            for (; liter1.More(); liter1.Next())
            {
                coord += BRep_Tool.Pnt(TopoDS.Vertex(liter1.Value())).XYZ();
                nbPoints++;
            }
            if (nbPoints != 0)
            {
                var vp = new gp_Pnt(coord / nbPoints);
                double tol = 0.0, mtol = 0.0;
                var liter2 = new NCollection_List<TopoDS_Shape>.Iterator(vlist);
                for (; liter2.More(); liter2.Next())
                {
                    double vtol = BRep_Tool.Tolerance(TopoDS.Vertex(liter2.Value()));
                    if (mtol < vtol)
                    {
                        mtol = vtol;
                    }
                    vtol = vp.Distance(BRep_Tool.Pnt(TopoDS.Vertex(liter2.Value())));
                    if (tol < vtol)
                    {
                        tol = vtol;
                    }
                }
                B.UpdateVertex(vnew, vp, tol + mtol);
            }
        }

        // Update input shapes
        for (i = 1; i <= myOldShapes.Extent(); i++)
        {
            myOldShapes[i] = myReShape.InternalApply(myOldShapes[i]);
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
        var NewShapes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        if (!myShape.IsNull())
        {
            if (myOldShapes.IsEmpty())
            {
                Add(myShape);
                myShape.Nullify();
            }
            else
            {
                TopoDS_Shape newShape = myReShape.InternalApply(myShape);
                if (!newShape.IsNull())
                {
                    NewShapes.Add(newShape);
                }
            }
        }
        // Create map Edge -> Faces
        var EdgeFaces = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        int i, nbShapes = myOldShapes.Extent();
        for (i = 1; i <= nbShapes; i++)
        {
            // Retrieve new shape
            TopoDS_Shape shape = myOldShapes[i];
            if (shape.IsNull())
            {
                continue;
            }
            NewShapes.Add(shape);
            // Explore shape to find all boundaries
            for (var eExp = new TopExp_Explorer(shape, TopAbs_EDGE); eExp.More(); eExp.Next())
            {
                TopoDS_Shape edge = eExp.Current();
                if (!EdgeFaces.Contains(edge))
                {
                    var listFaces = new NCollection_List<TopoDS_Shape>();
                    EdgeFaces.Add(edge, listFaces);
                }
            }
        }
        // Fill map Edge -> Faces
        nbShapes = NewShapes.Extent();
        var mapFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= nbShapes; i++)
        {
            // Explore shape to find all faces
            var fExp = new TopExp_Explorer(NewShapes.FindKey(i), TopAbs_FACE);
            for (; fExp.More(); fExp.Next())
            {
                TopoDS_Shape face = fExp.Current();
                if (mapFaces.Contains(face))
                {
                    continue;
                }
                else
                {
                    mapFaces.Add(face);
                }
                // Explore face to find all boundaries
                for (var aIw = new TopoDS_Iterator(face); aIw.More(); aIw.Next())
                {
                    if (aIw.Value().ShapeType() != TopAbs_WIRE)
                    {
                        continue;
                    }
                    for (var aIIe = new TopoDS_Iterator(aIw.Value()); aIIe.More(); aIIe.Next())
                    {
                        TopoDS_Shape edge = aIIe.Value();

                        if (EdgeFaces.Contains(edge))
                        {
                            EdgeFaces.ChangeFromKey(edge).Append(face);
                        }
                    }
                }
            }
        }
        // Find free boundaries
        for (int anIterEF = 1; anIterEF <= EdgeFaces.Extent(); anIterEF++)
        {
            NCollection_List<TopoDS_Shape> listFaces = EdgeFaces[anIterEF];
            int nbFaces = listFaces.Extent();
            TopoDS_Shape edge = EdgeFaces.FindKey(anIterEF);
            if (edge.Orientation() == TopAbs_INTERNAL)
            {
                continue;
            }
            bool isSeam = false;
            if (nbFaces == 1)
            {
                TopoDS_Face face = TopoDS.Face(listFaces.First());
                isSeam = BRep_Tool.IsClosed(TopoDS.Edge(edge), face);
                if (isSeam)
                {
                    var aB = new BRep_Builder();
                    TopoDS_Shape anewEdge = edge.EmptyCopied();
                    var aItV = new TopoDS_Iterator(edge);
                    for (; aItV.More(); aItV.Next())
                    {
                        aB.Add(anewEdge, aItV.Value());
                    }

                    Geom2d_Curve? c2dold =
                        BRep_Tool.CurveOnSurface(TopoDS.Edge(edge),
                                                 TopoDS.Face(listFaces.First()),
                                                 out double first2d,
                                                 out double last2d);

                    Geom2d_Curve? c2d = null;
                    var B = new BRep_Builder();
                    B.UpdateEdge(TopoDS.Edge(anewEdge), c2d, c2d, TopoDS.Face(listFaces.First()), 0);
                    B.UpdateEdge(TopoDS.Edge(anewEdge), c2dold, TopoDS.Face(listFaces.First()), 0);

                    BRep_Tool.Range(TopoDS.Edge(edge), out double aFirst, out double aLast);
                    aB.Range(TopoDS.Edge(anewEdge), aFirst, aLast);
                    aB.Range(TopoDS.Edge(anewEdge), TopoDS.Face(listFaces.First()), first2d, last2d);
                    myReShape.Replace(edge, anewEdge);
                    edge = anewEdge;

                    isSeam = false;
                }
            }
            bool isBoundFloat = (myFloatingEdgesMode && nbFaces == 0);
            bool isBound = (myFaceMode && ((myNonmanifold && nbFaces != 0) || (nbFaces == 1 && !isSeam)));
            if (isBound || isBoundFloat)
            {
                // Ignore degenerated edge
                if (BRep_Tool.Degenerated(TopoDS.Edge(edge)))
                {
                    continue;
                }
                // Add to BoundFaces
                var listFacesCopy = new NCollection_List<TopoDS_Shape>();
                listFacesCopy.Append(listFaces);
                myBoundFaces.Add(edge, listFacesCopy);
                // Process edge vertices
                TopExp.Vertices(TopoDS.Edge(edge), out TopoDS_Vertex vFirst, out TopoDS_Vertex vLast);
                if (vFirst.IsNull() || vLast.IsNull())
                {
                    continue;
                }
                if (vFirst.Orientation() == TopAbs_INTERNAL || vLast.Orientation() == TopAbs_INTERNAL)
                {
                    continue;
                }
                if (isBound)
                {
                    // Add to VertexNode
                    if (!myVertexNode.Contains(vFirst))
                    {
                        myVertexNode.Add(vFirst, vFirst);
                    }
                    if (!myVertexNode.Contains(vLast))
                    {
                        myVertexNode.Add(vLast, vLast);
                    }
                }
                else
                {
                    // Add to VertexNodeFree
                    if (!myVertexNodeFree.Contains(vFirst))
                    {
                        myVertexNodeFree.Add(vFirst, vFirst);
                    }
                    if (!myVertexNodeFree.Contains(vLast))
                    {
                        myVertexNodeFree.Add(vLast, vLast);
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

    private static bool CreateNewNodes(NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> NodeNearestNode,
                                       NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> NodeVertices,
                                       NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> aVertexNode,
                                       NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aNodeEdges)
    {
        // Create new nodes
        var B = new BRep_Builder();
        var OldNodeNewNode = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var NewNodeOldNodes = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        for (int anIter = 1; anIter <= NodeNearestNode.Extent(); anIter++)
        {
            // Retrieve a pair of nodes to merge
            TopoDS_Shape oldnode1 = NodeNearestNode.FindKey(anIter);
            TopoDS_Shape oldnode2 = NodeNearestNode[anIter];
            // Second node should also be in the map
            if (!NodeNearestNode.Contains(oldnode2))
            {
                continue;
            }
            // Get new node for old node #1
            if (OldNodeNewNode.IsBound(oldnode1))
            {
                TopoDS_Shape newnode1 = OldNodeNewNode[oldnode1];
                if (OldNodeNewNode.IsBound(oldnode2))
                {
                    TopoDS_Shape newnode2 = OldNodeNewNode[oldnode2];
                    if (!newnode1.IsSame(newnode2))
                    {
                        // Change data for new node #2
                        NCollection_List<TopoDS_Shape> lnode1 = NewNodeOldNodes.ChangeFromKey(newnode1);
                        var itn = new NCollection_List<TopoDS_Shape>.Iterator(NewNodeOldNodes.FindFromKey(newnode2));
                        for (; itn.More(); itn.Next())
                        {
                            TopoDS_Shape node2 = itn.Value();
                            lnode1.Append(node2);
                            OldNodeNewNode[node2] = newnode1;
                        }
                        NewNodeOldNodes.RemoveKey(newnode2);
                    }
                }
                else
                {
                    // Old node #2 is not bound - add to old node #1
                    OldNodeNewNode.Bind(oldnode2, newnode1);
                    NewNodeOldNodes.ChangeFromKey(newnode1).Append(oldnode2);
                }
            }
            else
            {
                if (OldNodeNewNode.IsBound(oldnode2))
                {
                    // Old node #1 is not bound - add to old node #2
                    TopoDS_Shape newnode2 = OldNodeNewNode[oldnode2];
                    OldNodeNewNode.Bind(oldnode1, newnode2);
                    NewNodeOldNodes.ChangeFromKey(newnode2).Append(oldnode1);
                }
                else
                {
                    // Nodes are not bound - create new node
                    var newnode = new TopoDS_Vertex();
                    B.MakeVertex(ref newnode);
                    OldNodeNewNode.Bind(oldnode1, newnode);
                    OldNodeNewNode.Bind(oldnode2, newnode);
                    var lnodes = new NCollection_List<TopoDS_Shape>();
                    lnodes.Append(oldnode1);
                    lnodes.Append(oldnode2);
                    NewNodeOldNodes.Add(newnode, lnodes);
                }
            }
        }

        // Stop if no new nodes created
        if (NewNodeOldNodes.Extent() == 0)
        {
            return false;
        }

        for (int anIter1 = 1; anIter1 <= NewNodeOldNodes.Extent(); anIter1++)
        {
            TopoDS_Vertex newnode = TopoDS.Vertex(NewNodeOldNodes.FindKey(anIter1));
            // Calculate new node center point
            var theCoordinates = new gp_XYZ(0.0, 0.0, 0.0);
            var lvert = new NCollection_List<TopoDS_Shape>(); // Accumulate node vertices
            var medge = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var ledge = new NCollection_List<TopoDS_Shape>(); // Accumulate node edges
            // Iterate on old nodes
            var itn = new NCollection_List<TopoDS_Shape>.Iterator(NewNodeOldNodes[anIter1]);
            for (; itn.More(); itn.Next())
            {
                TopoDS_Shape oldnode = itn.Value();
                // Iterate on node vertices
                var itv1 = new NCollection_List<TopoDS_Shape>.Iterator(NodeVertices.FindFromKey(oldnode));
                for (; itv1.More(); itv1.Next())
                {
                    TopoDS_Vertex vertex = TopoDS.Vertex(itv1.Value());
                    // Change node for vertex
                    aVertexNode.ChangeFromKey(vertex) = newnode;
                    // Accumulate coordinates
                    theCoordinates += BRep_Tool.Pnt(vertex).XYZ();
                    lvert.Append(vertex);
                }
                // Iterate on node edges
                NCollection_List<TopoDS_Shape> edges = aNodeEdges[oldnode];
                var ite = new NCollection_List<TopoDS_Shape>.Iterator(edges);
                for (; ite.More(); ite.Next())
                {
                    TopoDS_Shape edge = ite.Value();
                    if (!medge.Contains(edge))
                    {
                        medge.Add(edge);
                        ledge.Append(edge);
                    }
                }
                // Unbind old node edges
                aNodeEdges.UnBind(oldnode);
            }
            // Bind new node edges
            aNodeEdges.Bind(newnode, ledge);
            var center = new gp_Pnt(theCoordinates / lvert.Extent());
            // Calculate new node tolerance
            double toler = 0.0;
            var itv = new NCollection_List<TopoDS_Shape>.Iterator(lvert);
            for (; itv.More(); itv.Next())
            {
                TopoDS_Vertex vertex = TopoDS.Vertex(itv.Value());
                double t = center.Distance(BRep_Tool.Pnt(vertex)) + BRep_Tool.Tolerance(vertex);
                if (toler < t)
                {
                    toler = t;
                }
            }
            // Update new node parameters
            B.UpdateVertex(newnode, center, toler);
        }

        return true;
    }

    private static int IsMergedVertices(in TopoDS_Shape face1,
                                        in TopoDS_Shape e1,
                                        in TopoDS_Shape e2,
                                        in TopoDS_Shape vtx1,
                                        in TopoDS_Shape vtx2)
    {
        // Case of floating edges
        if (face1.IsNull())
        {
            return (!IsClosedShape(e1, vtx1, vtx2)) ? 1 : 0;
        }

        // Find wires containing given edges
        TopoDS_Shape wire1 = new(), wire2 = new();
        var itw = new TopExp_Explorer(face1, TopAbs_WIRE);
        for (; itw.More() && (wire1.IsNull() || wire2.IsNull()); itw.Next())
        {
            var ite = new TopoDS_Iterator(itw.Current(), false);
            for (; ite.More() && (wire1.IsNull() || wire2.IsNull()); ite.Next())
            {
                if (wire1.IsNull() && e1.IsSame(ite.Value()))
                {
                    wire1 = itw.Current();
                }
                if (wire2.IsNull() && e2.IsSame(ite.Value()))
                {
                    wire2 = itw.Current();
                }
            }
        }
        int Status = 0;
        if (!wire1.IsNull() && !wire2.IsNull())
        {
            if (wire1.IsSame(wire2))
            {
                for (var aIte = new TopoDS_Iterator(wire1, false); aIte.More(); aIte.Next())
                {
                    TopExp.Vertices(TopoDS.Edge(aIte.Value()), out TopoDS_Vertex ve1, out TopoDS_Vertex ve2);
                    if ((ve1.IsSame(vtx1) && ve2.IsSame(vtx2)) || (ve2.IsSame(vtx1) && ve1.IsSame(vtx2)))
                    {
                        return (IsClosedShape(aIte.Value(), vtx1, vtx2) ? 0 : 1);
                    }
                }
                if (IsClosedShape(wire1, vtx1, vtx2))
                {
                    TopExp.Vertices(TopoDS.Wire(wire1), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    bool isEndVertex =
                        ((V1.IsSame(vtx1) && V2.IsSame(vtx2)) || (V2.IsSame(vtx1) && V1.IsSame(vtx2)));
                    if (!isEndVertex)
                    {
                        Status = 1;
                    }
                }
                else
                {
                    Status = 1;
                }
            }
            else
            {
                Status = -1;
            }
        }
        return Status;
    }

    private static bool GlueVertices(NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> aVertexNode,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aNodeEdges,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aBoundFaces,
                                     double Tolerance,
                                     Message_ProgressRange theProgress)
    {
        // Create map of node -> vertices
        var NodeVertices = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var aFilter = new NCollection_CellFilter<BRepBuilderAPI_VertexInspector, int, gp_XYZ>(Tolerance);
        var anInspector = new BRepBuilderAPI_VertexInspector(Tolerance);
        for (int anIter1 = 1; anIter1 <= aVertexNode.Extent(); anIter1++)
        {
            TopoDS_Shape vertex = aVertexNode.FindKey(anIter1);
            TopoDS_Vertex node = TopoDS.Vertex(aVertexNode[anIter1]);
            if (NodeVertices.Contains(node))
            {
                NodeVertices.ChangeFromKey(node).Append(vertex);
            }
            else
            {
                var vlist = new NCollection_List<TopoDS_Shape>();
                vlist.Append(vertex);
                NodeVertices.Add(node, vlist);
                gp_Pnt aPnt = BRep_Tool.Pnt(TopoDS.Vertex(node));
                aFilter.Add(NodeVertices.FindIndex(node), aPnt.XYZ());
                anInspector.Add(aPnt.XYZ());
            }
        }
        int nbNodes = NodeVertices.Extent();
        // Merge nearest nodes
        var NodeNearestNode = new NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        using var aPS = new Message_ProgressScope(theProgress, "Glueing nodes", nbNodes, true);
        for (int i = 1; i <= nbNodes && aPS.More(); i++, aPS.Next())
        {
            TopoDS_Vertex node1 = TopoDS.Vertex(NodeVertices.FindKey(i));
            // Find near nodes
            gp_Pnt pt1 = BRep_Tool.Pnt(node1);
            anInspector.SetCurrent(pt1.XYZ());
            gp_XYZ aPntMin = BRepBuilderAPI_VertexInspector.Shift(pt1.XYZ(), -Tolerance);
            gp_XYZ aPntMax = BRepBuilderAPI_VertexInspector.Shift(pt1.XYZ(), Tolerance);
            aFilter.Inspect(aPntMin, aPntMax, ref anInspector);
            if (anInspector.ResInd().IsEmpty())
            {
                continue;
            }
            // Retrieve list of edges for the first node
            NCollection_List<TopoDS_Shape> ledges1 = aNodeEdges[node1];
            // Explore list of near nodes and fill the sequence of glued nodes
            var SeqNodes = new NCollection_Sequence<TopoDS_Shape>();
            var listNodesSameEdge = new NCollection_List<TopoDS_Shape>();
            var iter1 = new NCollection_List<int>.Iterator(anInspector.ResInd());
            for (; iter1.More(); iter1.Next())
            {
                TopoDS_Vertex node2 = TopoDS.Vertex(NodeVertices.FindKey(iter1.Value()));
                if (node1 == node2)
                {
                    continue;
                }
                // Retrieve list of edges for the second node
                NCollection_List<TopoDS_Shape> ledges2 = aNodeEdges[node2];
                // Check merging condition for the pair of nodes
                int Status = 0;
                bool isSameEdge = false;
                // Explore edges of the first node
                var Ie1 = new NCollection_List<TopoDS_Shape>.Iterator(ledges1);
                for (; Ie1.More() && Status == 0 && !isSameEdge; Ie1.Next())
                {
                    TopoDS_Shape e1 = Ie1.Value();
                    // Obtain real vertex from edge
                    TopoDS_Shape v1 = node1;
                    { // szv: Use brackets to destroy local variables
                        TopExp.Vertices(TopoDS.Edge(e1), out TopoDS_Vertex ov1, out TopoDS_Vertex ov2);
                        if (aVertexNode.Contains(ov1))
                        {
                            if (node1.IsSame(aVertexNode.FindFromKey(ov1)))
                            {
                                v1 = ov1;
                            }
                        }
                        if (aVertexNode.Contains(ov2))
                        {
                            if (node1.IsSame(aVertexNode.FindFromKey(ov2)))
                            {
                                v1 = ov2;
                            }
                        }
                    }
                    // Create map of faces for e1
                    var Faces1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    NCollection_List<TopoDS_Shape> lfac1 = aBoundFaces.FindFromKey(e1);
                    if (lfac1.Extent() != 0)
                    {
                        var itf = new NCollection_List<TopoDS_Shape>.Iterator(lfac1);
                        for (; itf.More(); itf.Next())
                        {
                            if (!itf.Value().IsNull())
                            {
                                Faces1.Add(itf.Value());
                            }
                        }
                    }
                    // Explore edges of the second node
                    var Ie2 = new NCollection_List<TopoDS_Shape>.Iterator(ledges2);
                    for (; Ie2.More() && Status == 0 && !isSameEdge; Ie2.Next())
                    {
                        TopoDS_Shape e2 = Ie2.Value();
                        // Obtain real vertex from edge
                        TopoDS_Shape v2 = node2;
                        { // szv: Use brackets to destroy local variables
                            TopExp.Vertices(TopoDS.Edge(e2), out TopoDS_Vertex ov1, out TopoDS_Vertex ov2);
                            if (aVertexNode.Contains(ov1))
                            {
                                if (node2.IsSame(aVertexNode.FindFromKey(ov1)))
                                {
                                    v2 = ov1;
                                }
                            }
                            if (aVertexNode.Contains(ov2))
                            {
                                if (node2.IsSame(aVertexNode.FindFromKey(ov2)))
                                {
                                    v2 = ov2;
                                }
                            }
                        }
                        // Explore faces for e2
                        NCollection_List<TopoDS_Shape> lfac2 = aBoundFaces.FindFromKey(e2);
                        if (lfac2.Extent() != 0)
                        {
                            var itf = new NCollection_List<TopoDS_Shape>.Iterator(lfac2);
                            for (; itf.More() && Status == 0 && !isSameEdge; itf.Next())
                            {
                                // Check merging conditions for the same face
                                if (Faces1.Contains(itf.Value()))
                                {
                                    int stat = IsMergedVertices(itf.Value(), e1, e2, v1, v2);
                                    if (stat == 1)
                                    {
                                        isSameEdge = true;
                                    }
                                    else
                                    {
                                        Status = stat;
                                    }
                                }
                            }
                        }
                        else if (Faces1.IsEmpty() && e1 == e2)
                        {
                            int stat = IsMergedVertices(new TopoDS_Face(), e1, e1, v1, v2);
                            if (stat == 1)
                            {
                                isSameEdge = true;
                            }
                            else
                            {
                                Status = stat;
                            }
                            break;
                        }
                    }
                }
                if (Status != 0)
                {
                    continue;
                }
                if (isSameEdge)
                {
                    listNodesSameEdge.Append(node2);
                }
                // Append near node to the sequence
                gp_Pnt pt2 = BRep_Tool.Pnt(node2);
                double dist = pt1.Distance(pt2);
                if (dist < Tolerance)
                {
                    bool isIns = false;
                    for (int kk = 1; kk <= SeqNodes.Length() && !isIns; kk++)
                    {
                        gp_Pnt pt = BRep_Tool.Pnt(TopoDS.Vertex(SeqNodes.Value(kk)));
                        if (dist < pt1.Distance(pt))
                        {
                            SeqNodes.InsertBefore(kk, node2);
                            isIns = true;
                        }
                    }
                    if (!isIns)
                    {
                        SeqNodes.Append(node2);
                    }
                }
            }
            if (SeqNodes.Length() != 0)
            {
                // Remove nodes near to some other from the same edge
                if (listNodesSameEdge.Extent() != 0)
                {
                    var lInt = new NCollection_List<TopoDS_Shape>.Iterator(listNodesSameEdge);
                    for (; lInt.More(); lInt.Next())
                    {
                        TopoDS_Vertex n2 = TopoDS.Vertex(lInt.Value());
                        gp_Pnt p2 = BRep_Tool.Pnt(n2);
                        for (int k = 1; k <= SeqNodes.Length();)
                        {
                            TopoDS_Vertex n1 = TopoDS.Vertex(SeqNodes.Value(k));
                            if (n1 != n2)
                            {
                                gp_Pnt p1 = BRep_Tool.Pnt(n1);
                                if (p2.Distance(p1) >= pt1.Distance(p1))
                                {
                                    k++;
                                    continue;
                                }
                            }
                            SeqNodes.Remove(k);
                        }
                    }
                }
                // Bind nearest node if at least one exists
                if (SeqNodes.Length() != 0)
                {
                    NodeNearestNode.Add(node1, SeqNodes.First());
                }
            }
            anInspector.ClearResList();
        }

        // Create new nodes for chained nearest nodes
        if (NodeNearestNode.IsEmpty())
        {
            return false;
        }

        return CreateNewNodes(NodeNearestNode, NodeVertices, aVertexNode, aNodeEdges);
    }

    /// <summary>Glues the vertices of the bounds within the tolerance into nodes. Called from Perform only.</summary>
    protected virtual void VerticesAssembling(Message_ProgressRange theProgress = default)
    {
        int nbVert = myVertexNode.Extent();
        int nbVertFree = myVertexNodeFree.Extent();
        using var aPS = new Message_ProgressScope(theProgress, "Vertices assembling", 2);
        if (nbVert != 0 || nbVertFree != 0)
        {
            // Fill map node -> sections
            int i;
            for (i = 1; i <= myBoundFaces.Extent(); i++)
            {
                TopoDS_Shape bound = myBoundFaces.FindKey(i);
                for (var itv = new TopoDS_Iterator(bound, false); itv.More(); itv.Next())
                {
                    TopoDS_Shape node = itv.Value();
                    if (myNodeSections.IsBound(node))
                    {
                        myNodeSections[node].Append(bound);
                    }
                    else
                    {
                        var lbnd = new NCollection_List<TopoDS_Shape>();
                        lbnd.Append(bound);
                        myNodeSections.Bind(node, lbnd);
                    }
                }
            }
            // Glue vertices
            if (nbVert != 0)
            {
                while (GlueVertices(myVertexNode, myNodeSections, myBoundFaces, myTolerance, aPS.Next()))
                {
                }
            }
            if (!aPS.More())
            {
                return;
            }
            if (nbVertFree != 0)
            {
                while (GlueVertices(myVertexNodeFree, myNodeSections, myBoundFaces, myTolerance, aPS.Next()))
                {
                }
            }
        }
    }

    //=======================================================================
    // function : replaceNMVertices
    // purpose  : internal use (static)
    //=======================================================================
    private static void replaceNMVertices(in TopoDS_Edge theEdge,
                                          in TopoDS_Vertex theV1,
                                          in TopoDS_Vertex theV2,
                                          BRepTools_ReShape theReShape)
    {
        // To keep NM vertices on edge
        var aSeqNMVert = new NCollection_Sequence<TopoDS_Shape>();
        var aSeqNMPars = new NCollection_Sequence<double>();
        bool hasNMVert = findNMVertices(theEdge, aSeqNMVert, aSeqNMPars);
        if (!hasNMVert)
        {
            return;
        }
        BRep_Tool.Range(theEdge, out double first, out double last);
        Geom_Curve? c3d = BRep_Tool.Curve(theEdge, out TopLoc_Location aLoc, out first, out last);
        if (c3d is null)
        {
            return;
        }
        var aEdVert = new NCollection_Sequence<TopoDS_Shape>();
        var aEdParams = new NCollection_Sequence<double>();
        int i = 1, nb = aSeqNMPars.Length();

        for (; i <= nb; i++)
        {
            double apar = aSeqNMPars.Value(i);
            if (Math.Abs(apar - first) <= Precision.PConfusion())
            {
                theReShape.Replace(aSeqNMVert.Value(i), theV1);
                continue;
            }
            if (Math.Abs(apar - last) <= Precision.PConfusion())
            {
                theReShape.Replace(aSeqNMVert.Value(i), theV2);
                continue;
            }
            TopoDS_Shape aV = aSeqNMVert.Value(i);
            int j = 1;
            for (; j <= aEdParams.Length(); j++)
            {
                double apar2 = aEdParams.Value(j);
                if (Math.Abs(apar - apar2) <= Precision.PConfusion())
                {
                    theReShape.Replace(aV, aEdVert.Value(j));
                    break;
                }
                else if (apar < apar2)
                {
                    TopoDS_Shape anewV = aV.EmptyCopied();
                    aEdVert.InsertBefore(j, anewV);
                    aEdParams.InsertBefore(j, apar);
                    NCollection_List<BRep_PointRepresentation> alistrep = ((BRep_TVertex)anewV.TShape()!).ChangePoints();
                    var aPRep = new BRep_PointOnCurve(apar, c3d, aLoc);
                    alistrep.Append(aPRep);
                    theReShape.Replace(aV, anewV);
                    break;
                }
            }
            if (j > aEdParams.Length())
            {
                TopoDS_Shape anewV = aV.EmptyCopied();
                aEdVert.Append(anewV);
                aEdParams.Append(apar);
                NCollection_List<BRep_PointRepresentation> alistrep = ((BRep_TVertex)anewV.TShape()!).ChangePoints();
                var aPRep = new BRep_PointOnCurve(apar, c3d, aLoc);
                alistrep.Append(aPRep);
                theReShape.Replace(aV, anewV);
            }
        }

        int newnb = aEdParams.Length();
        if (newnb < nb)
        {
            TopoDS_Shape anewEdge = theEdge.EmptyCopied();
            TopAbs_Orientation anOri = theEdge.Orientation();
            anewEdge.Orientation(TopAbs_FORWARD);
            var aB = new BRep_Builder();
            aB.Add(anewEdge, theV1);
            aB.Add(anewEdge, theV2);

            for (i = 1; i <= aEdVert.Length(); i++)
            {
                aB.Add(anewEdge, aEdVert.Value(i));
            }
            anewEdge.Orientation(anOri);
            theReShape.Replace(theEdge, anewEdge);
        }
    }

    //=======================================================================
    // function : ReplaceEdge
    // purpose  : internal use (static)
    //=======================================================================

    private static void ReplaceEdge(in TopoDS_Shape oldEdge, in TopoDS_Shape theNewShape, BRepTools_ReShape aReShape)
    {
        TopoDS_Shape oldShape = aReShape.InternalApply(oldEdge);
        TopoDS_Shape newShape = aReShape.InternalApply(theNewShape);
        if (oldShape.IsSame(newShape) || aReShape.IsRecorded(newShape))
        {
            return;
        }

        aReShape.Replace(oldShape, newShape);
        TopoDS_Vertex V1new = new(), V2new = new();
        TopExp.Vertices(TopoDS.Edge(oldShape), out TopoDS_Vertex V1old, out TopoDS_Vertex V2old);
        TopAbs_Orientation Orold = oldShape.Orientation();
        TopAbs_Orientation Ornew = Orold;
        if (newShape.ShapeType() == TopAbs_EDGE)
        {
            TopoDS_Edge aEn = TopoDS.Edge(newShape);
            TopExp.Vertices(aEn, out V1new, out V2new);
            Ornew = aEn.Orientation();
            replaceNMVertices(aEn, V1new, V2new, aReShape);
        }
        else if (newShape.ShapeType() == TopAbs_WIRE)
        {
            for (var aex = new TopExp_Explorer(newShape, TopAbs_EDGE); aex.More(); aex.Next())
            {
                TopoDS_Edge ed = TopoDS.Edge(aex.Current());
                Ornew = ed.Orientation();
                TopExp.Vertices(ed, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                replaceNMVertices(ed, aV1, aV2, aReShape);
                if (V1new.IsNull())
                {
                    V1new = aV1;
                }
                V2new = aV2;
            }
        }

        V1new.Orientation(V1old.Orientation());
        V2new.Orientation(V2old.Orientation());
        if (V1old.IsSame(V2old) && !V1old.IsSame(V1new) && !aReShape.IsRecorded(V1new))
        {
            aReShape.Replace(V1old, V1new);
            return;
        }
        if (Orold == Ornew)
        {
            V1new.Orientation(V1old.Orientation());
            V2new.Orientation(V2old.Orientation());
            if (!V1old.IsSame(V1new) && !V1old.IsSame(V2new) && !aReShape.IsRecorded(V1new))
            {
                aReShape.Replace(V1old, V1new);
            }
            if (!V2old.IsSame(V2new) && !V2old.IsSame(V1new) && !aReShape.IsRecorded(V2new))
            {
                aReShape.Replace(V2old, V2new);
            }
        }
        else
        {
            V1new.Orientation(V2old.Orientation());
            V2new.Orientation(V1old.Orientation());
            if (!V1old.IsSame(V2new) && !V1old.IsSame(V1new) && !aReShape.IsRecorded(V2new))
            {
                aReShape.Replace(V1old, V2new);
            }
            if (!V2old.IsSame(V2new) && !V2old.IsSame(V1new) && !aReShape.IsRecorded(V1new))
            {
                aReShape.Replace(V2old, V1new);
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
    protected void Merging(bool passage, Message_ProgressRange theProgress = default)
    {
        _ = passage; // OCCT: unused (firstTime)
        var B = new BRep_Builder();
        //  NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> MergedEdges;
        using var aPS = new Message_ProgressScope(theProgress, "Merging bounds", myBoundFaces.Extent());
        for (int anIterB = 1; anIterB <= myBoundFaces.Extent() && aPS.More(); anIterB++, aPS.Next())
        {
            TopoDS_Shape bound = myBoundFaces.FindKey(anIterB);

            // If bound was already merged - continue
            if (myMergedEdges.Contains(bound))
            {
                continue;
            }

            if (myBoundFaces[anIterB].Extent() == 0)
            {
                // Merge free edge - only vertices
                TopExp.Vertices(TopoDS.Edge(bound), out TopoDS_Vertex no1, out TopoDS_Vertex no2);
                TopoDS_Shape nno1 = no1, nno2 = no2;
                if (myVertexNodeFree.Contains(no1))
                {
                    nno1 = myVertexNodeFree.FindFromKey(no1);
                }
                if (myVertexNodeFree.Contains(no2))
                {
                    nno2 = myVertexNodeFree.FindFromKey(no2);
                }
                if (!no1.IsSame(nno1))
                {
                    nno1.Orientation(no1.Orientation());
                    myReShape.Replace(no1, nno1);
                }
                if (!no2.IsSame(nno2))
                {
                    nno2.Orientation(no2.Orientation());
                    myReShape.Replace(no2, nno2);
                }
                myMergedEdges.Add(bound);
                continue;
            }

            // Check for previous splitting, build replacing wire
            var BoundWire = new TopoDS_Wire();
            bool isPrevSplit = false;
            bool hasCuttingSections = myBoundSections.IsBound(bound);
            if (hasCuttingSections)
            {
                B.MakeWire(ref BoundWire);
                BoundWire.Orientation(bound.Orientation());
                // Iterate on cutting sections
                var its = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[bound]);
                for (; its.More(); its.Next())
                {
                    TopoDS_Shape section = its.Value();
                    B.Add(BoundWire, section);
                    if (myMergedEdges.Contains(section))
                    {
                        isPrevSplit = true;
                    }
                }
            }

            // Merge with bound
            var MergedWithBound = new NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            if (!isPrevSplit)
            {
                // Obtain sequence of edges merged with bound
                var seqMergedWithBound = new NCollection_Sequence<TopoDS_Shape>();
                var seqMergedWithBoundOri = new NCollection_Sequence<bool>();
                if (MergedNearestEdges(bound, seqMergedWithBound, seqMergedWithBoundOri))
                {
                    // Store bound in the map
                    MergedWithBound.Add(bound, bound);
                    // Iterate on edges merged with bound
                    int ii = 1;
                    while (ii <= seqMergedWithBound.Length())
                    {
                        TopoDS_Shape iedge = seqMergedWithBound.Value(ii);
                        // Remove edge if recorded as merged
                        bool isRejected = (myMergedEdges.Contains(iedge) || MergedWithBound.Contains(iedge));
                        if (!isRejected)
                        {
                            if (myBoundSections.IsBound(iedge))
                            {
                                // Edge is split - check sections
                                var lit = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[iedge]);
                                for (; lit.More() && !isRejected; lit.Next())
                                {
                                    TopoDS_Shape sec = lit.Value();
                                    // Remove edge (bound) if at least one of its sections already merged
                                    isRejected = (myMergedEdges.Contains(sec) || MergedWithBound.Contains(sec));
                                }
                            }
                            if (!isRejected)
                            {
                                if (mySectionBound.IsBound(iedge))
                                {
                                    // Edge is a section - check bound
                                    TopoDS_Shape bnd = mySectionBound[iedge];
                                    // Remove edge (section) if its bound already merged
                                    isRejected = (myMergedEdges.Contains(bnd) || MergedWithBound.Contains(bnd));
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
                            MergedWithBound.Add(iedge, iedge);
                            ii++;
                        }
                    }
                    int nbMerged = seqMergedWithBound.Length();
                    if (nbMerged != 0)
                    {
                        // Create same parameter edge
                        var ActuallyMerged = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                        TopoDS_Edge MergedEdge = SameParameterEdge(bound,
                                                                   seqMergedWithBound,
                                                                   seqMergedWithBoundOri,
                                                                   ActuallyMerged,
                                                                   myReShape);
                        bool isForward = false;
                        if (!MergedEdge.IsNull())
                        {
                            isForward = (MergedEdge.Orientation() == TopAbs_FORWARD);
                        }
                        // Process actually merged edges
                        int nbActuallyMerged = 0;
                        for (ii = 1; ii <= nbMerged; ii++)
                        {
                            TopoDS_Shape iedge = seqMergedWithBound[ii];
                            if (ActuallyMerged.Contains(iedge))
                            {
                                nbActuallyMerged++;
                                // Record merged edge in the map
                                TopAbs_Orientation orient = iedge.Orientation();
                                if (!isForward)
                                {
                                    orient = TopAbs.Reverse(orient);
                                }
                                if (!seqMergedWithBoundOri[ii])
                                {
                                    orient = TopAbs.Reverse(orient);
                                }
                                MergedWithBound.ChangeFromKey(iedge) = MergedEdge.Oriented(orient);
                            }
                            else
                            {
                                MergedWithBound.RemoveKey(iedge);
                            }
                        }
                        if (nbActuallyMerged != 0)
                        {
                            // Record merged bound in the map
                            TopAbs_Orientation orient = bound.Orientation();
                            if (!isForward)
                            {
                                orient = TopAbs.Reverse(orient);
                            }
                            MergedWithBound.ChangeFromKey(bound) = MergedEdge.Oriented(orient);
                        }
                        nbMerged = nbActuallyMerged;
                    }
                    // Remove bound from the map if not finally merged
                    if (nbMerged == 0)
                    {
                        MergedWithBound.RemoveKey(bound);
                    }
                }
            }
            bool isMerged = !MergedWithBound.IsEmpty();

            // Merge with cutting sections
            var SectionsReShape = new BRepTools_ReShape();
            var MergedWithSections = new NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            if (hasCuttingSections)
            {
                // Iterate on cutting sections
                var its = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[bound]);
                for (; its.More(); its.Next())
                {
                    // Retrieve cutting section
                    TopoDS_Shape section = its.Value();
                    // Skip section if already merged
                    if (myMergedEdges.Contains(section))
                    {
                        continue;
                    }
                    // Merge cutting section
                    var seqMergedWithSection = new NCollection_Sequence<TopoDS_Shape>();
                    var seqMergedWithSectionOri = new NCollection_Sequence<bool>();
                    if (MergedNearestEdges(section, seqMergedWithSection, seqMergedWithSectionOri))
                    {
                        // Store section in the map
                        MergedWithSections.Add(section, section);
                        // Iterate on edges merged with section
                        int ii = 1;
                        while (ii <= seqMergedWithSection.Length())
                        {
                            TopoDS_Shape iedge = seqMergedWithSection.Value(ii);
                            // Remove edge if recorded as merged
                            bool isRejected = (myMergedEdges.Contains(iedge) || MergedWithSections.Contains(iedge));
                            if (!isRejected)
                            {
                                if (myBoundSections.IsBound(iedge))
                                {
                                    // Edge is split - check sections
                                    var lit = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[iedge]);
                                    for (; lit.More() && !isRejected; lit.Next())
                                    {
                                        TopoDS_Shape sec = lit.Value();
                                        // Remove edge (bound) if at least one of its sections already merged
                                        isRejected = (myMergedEdges.Contains(sec) || MergedWithSections.Contains(sec));
                                    }
                                }
                                if (!isRejected)
                                {
                                    if (mySectionBound.IsBound(iedge))
                                    {
                                        // Edge is a section - check bound
                                        TopoDS_Shape bnd = mySectionBound[iedge];
                                        // Remove edge (section) if its bound already merged
                                        isRejected = (myMergedEdges.Contains(bnd) || MergedWithSections.Contains(bnd));
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
                                MergedWithSections.Add(iedge, iedge);
                                ii++;
                            }
                        }
                        int nbMerged = seqMergedWithSection.Length();
                        if (nbMerged != 0)
                        {
                            // Create same parameter edge
                            var ActuallyMerged = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                            TopoDS_Edge MergedEdge = SameParameterEdge(section,
                                                                       seqMergedWithSection,
                                                                       seqMergedWithSectionOri,
                                                                       ActuallyMerged,
                                                                       SectionsReShape);
                            bool isForward = false;
                            if (!MergedEdge.IsNull())
                            {
                                isForward = (MergedEdge.Orientation() == TopAbs_FORWARD);
                            }
                            // Process actually merged edges
                            int nbActuallyMerged = 0;
                            for (ii = 1; ii <= nbMerged; ii++)
                            {
                                TopoDS_Shape iedge = seqMergedWithSection[ii];
                                if (ActuallyMerged.Contains(iedge))
                                {
                                    nbActuallyMerged++;
                                    // Record merged edge in the map
                                    TopAbs_Orientation orient = iedge.Orientation();
                                    if (!isForward)
                                    {
                                        orient = TopAbs.Reverse(orient);
                                    }
                                    if (!seqMergedWithSectionOri[ii])
                                    {
                                        orient = TopAbs.Reverse(orient);
                                    }
                                    TopoDS_Shape oedge = MergedEdge.Oriented(orient);
                                    MergedWithSections.ChangeFromKey(iedge) = oedge;
                                    ReplaceEdge(myReShape.InternalApply(iedge), oedge, SectionsReShape);
                                }
                                else
                                {
                                    MergedWithSections.RemoveKey(iedge);
                                }
                            }
                            if (nbActuallyMerged != 0)
                            {
                                // Record merged section in the map
                                TopAbs_Orientation orient = section.Orientation();
                                if (!isForward)
                                {
                                    orient = TopAbs.Reverse(orient);
                                }
                                TopoDS_Shape oedge = MergedEdge.Oriented(orient);
                                MergedWithSections.ChangeFromKey(section) = oedge;
                                ReplaceEdge(myReShape.InternalApply(section), oedge, SectionsReShape);
                            }
                            nbMerged = nbActuallyMerged;
                        }
                        // Remove section from the map if not finally merged
                        if (nbMerged == 0)
                        {
                            MergedWithSections.RemoveKey(section);
                        }
                    }
                    else if (isMerged)
                    {
                        // Reject merging of sections
                        MergedWithSections.Clear();
                        break;
                    }
                }
            }
            bool isMergedSplit = !MergedWithSections.IsEmpty();

            if (!isMerged && !isMergedSplit)
            {
                // Nothing was merged in this iteration
                if (isPrevSplit)
                {
                    // Replace previously split bound
                    myReShape.Replace(myReShape.InternalApply(bound), myReShape.InternalApply(BoundWire));
                }
                continue;
            }

            // Set splitting flag
            bool isSplitted = ((!isMerged && isMergedSplit) || isPrevSplit);

            // Choose between bound and sections merging
            if (isMerged && isMergedSplit && !isPrevSplit)
            {
                // Fill map of merged cutting sections
                var MapSplitEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                for (int anItm = 1; anItm <= MergedWithSections.Extent(); anItm++)
                {
                    TopoDS_Shape edge = MergedWithSections.FindKey(anItm);
                    MapSplitEdges.Add(edge);
                }
                // Iterate on edges merged with bound
                for (int anItm1 = 1; anItm1 <= MergedWithBound.Extent(); anItm1++)
                {
                    // Retrieve edge merged with bound
                    TopoDS_Shape edge = MergedWithBound.FindKey(anItm1);
                    // Remove edge from the map
                    if (MapSplitEdges.Contains(edge))
                    {
                        MapSplitEdges.RemoveKey(edge);
                    }
                    if (myBoundSections.IsBound(edge))
                    {
                        // Edge has cutting sections
                        var its = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[edge]);
                        for (; its.More(); its.Next())
                        {
                            TopoDS_Shape sec = its.Value();
                            // Remove section from the map
                            if (MapSplitEdges.Contains(sec))
                            {
                                MapSplitEdges.RemoveKey(sec);
                            }
                        }
                    }
                }
                // Calculate section merging tolerance
                double MinSplitTol = RealLast();
                for (int ii = 1; ii <= MapSplitEdges.Extent(); ii++)
                {
                    TopoDS_Edge edge = TopoDS.Edge(MergedWithSections.FindFromKey(MapSplitEdges.FindKey(ii)));
                    MinSplitTol = Min(MinSplitTol, BRep_Tool.Tolerance(edge));
                }
                // Calculate bound merging tolerance
                TopoDS_Edge BoundEdge = TopoDS.Edge(MergedWithBound.FindFromKey(bound));
                double BoundEdgeTol = BRep_Tool.Tolerance(BoundEdge);
                isSplitted = ((MinSplitTol < BoundEdgeTol + MinTolerance()) || myNonmanifold);
                isSplitted = (!MapSplitEdges.IsEmpty() && isSplitted);
            }

            if (isSplitted)
            {
                // Merging of cutting sections
                // myMergedEdges.Add(bound);
                myReShape.Replace(myReShape.InternalApply(bound), myReShape.InternalApply(BoundWire));
                for (int anItm = 1; anItm <= MergedWithSections.Extent(); anItm++)
                {
                    TopoDS_Shape oldedge = MergedWithSections.FindKey(anItm);
                    TopoDS_Shape newedge = SectionsReShape.InternalApply(MergedWithSections[anItm]);
                    ReplaceEdge(myReShape.InternalApply(oldedge), newedge, myReShape);
                    myMergedEdges.Add(oldedge);
                    if (myBoundSections.IsBound(oldedge))
                    {
                        myBoundSections.UnBind(oldedge);
                    }
                }
            }
            else
            {
                // Merging of initial bound
                // myMergedEdges.Add(bound);
                for (int anItm = 1; anItm <= MergedWithBound.Extent(); anItm++)
                {
                    TopoDS_Shape oldedge = MergedWithBound.FindKey(anItm);
                    TopoDS_Shape newedge = MergedWithBound[anItm];
                    ReplaceEdge(myReShape.InternalApply(oldedge), newedge, myReShape);
                    myMergedEdges.Add(oldedge);
                    if (myBoundSections.IsBound(oldedge))
                    {
                        myBoundSections.UnBind(oldedge);
                    }
                }
                if (myBoundSections.IsBound(bound))
                {
                    myBoundSections.UnBind(bound);
                }
                if (!myMergedEdges.Contains(bound))
                {
                    myMergedEdges.Add(bound);
                }
            }
        }

        myNbVertices = myVertexNode.Extent() + myVertexNodeFree.Extent();
        myNodeSections.Clear();
        myVertexNode.Clear();
        myVertexNodeFree.Clear();
        myCuttingNode.Clear();
    }

    //=================================================================================================

    /// <summary>Finds the edges to merge with the edge (the sections between the same nodes), with their orientations.</summary>
    protected bool MergedNearestEdges(in TopoDS_Shape edge,
                                      NCollection_Sequence<TopoDS_Shape> SeqMergedEdge,
                                      NCollection_Sequence<bool> SeqMergedOri)
    {
        // Retrieve edge nodes
        TopExp.Vertices(TopoDS.Edge(edge), out TopoDS_Vertex no1, out TopoDS_Vertex no2);
        TopoDS_Shape nno1 = no1, nno2 = no2;
        bool isNode1 = myVertexNode.Contains(no1);
        bool isNode2 = myVertexNode.Contains(no2);
        if (isNode1)
        {
            nno1 = myVertexNode.FindFromKey(no1);
        }
        if (isNode2)
        {
            nno2 = myVertexNode.FindFromKey(no2);
        }

        // Fill map of nodes connected to the node #1
        var mapVert1 = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        mapVert1.Add(nno1);
        if (myCuttingNode.IsBound(nno1))
        {
            var ilv = new NCollection_List<TopoDS_Shape>.Iterator(myCuttingNode[nno1]);
            for (; ilv.More(); ilv.Next())
            {
                TopoDS_Shape v1 = ilv.Value();
                mapVert1.Add(v1);
                if (!isNode1 && myCuttingNode.IsBound(v1))
                {
                    var ilvn = new NCollection_List<TopoDS_Shape>.Iterator(myCuttingNode[v1]);
                    for (; ilvn.More(); ilvn.Next())
                    {
                        TopoDS_Shape vn = ilvn.Value();
                        mapVert1.Add(vn);
                    }
                }
            }
        }

        // Fill map of nodes connected to the node #2
        var mapVert2 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        mapVert2.Add(nno2);
        if (myCuttingNode.IsBound(nno2))
        {
            var ilv = new NCollection_List<TopoDS_Shape>.Iterator(myCuttingNode[nno2]);
            for (; ilv.More(); ilv.Next())
            {
                TopoDS_Shape v1 = ilv.Value();
                mapVert2.Add(v1);
                if (!isNode2 && myCuttingNode.IsBound(v1))
                {
                    var ilvn = new NCollection_List<TopoDS_Shape>.Iterator(myCuttingNode[v1]);
                    for (; ilvn.More(); ilvn.Next())
                    {
                        TopoDS_Shape vn = ilvn.Value();
                        mapVert2.Add(vn);
                    }
                }
            }
        }

        // Find all possible contiguous edges
        var seqEdges = new NCollection_Sequence<TopoDS_Shape>();
        seqEdges.Append(edge);
        var mapEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        mapEdges.Add(edge);
        for (int i = 1; i <= mapVert1.Extent(); i++)
        {
            TopoDS_Shape node1 = mapVert1.FindKey(i);
            if (!myNodeSections.IsBound(node1))
            {
                continue;
            }
            var ilsec = new NCollection_List<TopoDS_Shape>.Iterator(myNodeSections[node1]);
            for (; ilsec.More(); ilsec.Next())
            {
                TopoDS_Shape sec = ilsec.Value();
                if (sec.IsSame(edge))
                {
                    continue;
                }
                // Retrieve section nodes
                TopExp.Vertices(TopoDS.Edge(sec), out TopoDS_Vertex vs1, out TopoDS_Vertex vs2);
                TopoDS_Shape vs1n = vs1, vs2n = vs2;
                if (myVertexNode.Contains(vs1))
                {
                    vs1n = myVertexNode.FindFromKey(vs1);
                }
                if (myVertexNode.Contains(vs2))
                {
                    vs2n = myVertexNode.FindFromKey(vs2);
                }
                if ((mapVert1.Contains(vs1n) && mapVert2.Contains(vs2n))
                    || (mapVert1.Contains(vs2n) && mapVert2.Contains(vs1n)))
                {
                    if (mapEdges.Add(sec))
                    {
                        // Check for rejected cutting
                        bool isRejected = myMergedEdges.Contains(sec);
                        if (!isRejected && myBoundSections.IsBound(sec))
                        {
                            var its = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[sec]);
                            for (; its.More() && !isRejected; its.Next())
                            {
                                TopoDS_Shape section = its.Value();

                                if (myMergedEdges.Contains(section))
                                {
                                    isRejected = true;
                                }
                            }
                        }
                        if (!isRejected && mySectionBound.IsBound(sec))
                        {
                            TopoDS_Shape bnd = mySectionBound[sec];
                            isRejected = (!myBoundSections.IsBound(bnd) || myMergedEdges.Contains(bnd));
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
            if (myNonmanifold)
            {
                double lenRef = 0.0;
                for (i = 1; i <= nbSection; i++)
                {
                    Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(seqEdges[i]), out double f, out double l);
                    var cAdapt = new GeomAdaptor_Curve(c3d!);
                    double len = GCPnts_AbscissaPoint.Length(cAdapt, f, l);
                    if (len > lenRef)
                    {
                        indRef = i;
                        lenRef = len;
                    }
                }
                if (indRef != 1)
                {
                    TopoDS_Shape longEdge = seqEdges[indRef];
                    seqEdges[indRef] = seqEdges[1];
                    seqEdges[1] = longEdge;
                }
            }

            // Find merging candidates
            var seqForward = new NCollection_Sequence<bool>();
            var seqCandidates = new NCollection_Sequence<int>();
            var mapReference = new NCollection_IndexedMap<int>();
            mapReference.Add(indRef); // Add index of reference section
            if (FindCandidates(seqEdges, mapReference, seqCandidates, seqForward))
            {
                int nbCandidates = seqCandidates.Length();
                // Record candidate sections
                for (i = 1; i <= nbCandidates; i++)
                {
                    // Retrieve merged edge
                    TopoDS_Shape iedge = seqEdges[seqCandidates[i]];
                    bool ori = seqForward[i];
                    SeqMergedEdge.Append(iedge);
                    SeqMergedOri.Append(ori);
                    if (!myNonmanifold)
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
    protected void Cutting(Message_ProgressRange theProgress = default)
    {
        int i, nbVertices = myVertexNode.Extent();
        if (nbVertices == 0)
        {
            return;
        }
        // Create a box tree with vertices
        double eps = myTolerance * 0.5;
        var aTree = new NCollection_UBTree<int, Bnd_Box>();
        var aSelector = new BRepBuilderAPI_BndBoxTreeSelector();
        using (var aTreeFiller = new NCollection_UBTreeFiller<int, Bnd_Box>(aTree))
        {
            for (i = 1; i <= nbVertices; i++)
            {
                gp_Pnt pt = BRep_Tool.Pnt(TopoDS.Vertex(myVertexNode.FindKey(i)));
                var aBox = new Bnd_Box();
                aBox.Set(pt);
                aBox.Enlarge(eps);
                aTreeFiller.Add(i, aBox);
            }
            aTreeFiller.Fill();
        }

        Geom_Curve? c3d;
        TopLoc_Location loc;
        double first, last;
        // Iterate on all boundaries
        int nbBounds = myBoundFaces.Extent();
        using var aPS = new Message_ProgressScope(theProgress, "Cutting bounds", nbBounds);
        for (int anIterB = 1; anIterB <= myBoundFaces.Extent() && aPS.More(); anIterB++, aPS.Next())
        {
            TopoDS_Edge bound = TopoDS.Edge(myBoundFaces.FindKey(anIterB));
            // Do not cut floating edges
            if (myBoundFaces[anIterB].Extent() == 0)
            {
                continue;
            }
            // Obtain bound curve
            c3d = BRep_Tool.Curve(bound, out loc, out first, out last);
            if (c3d is null)
            {
                continue;
            }
            if (!loc.IsIdentity())
            {
                c3d = (Geom_Curve)c3d.Copy();
                c3d.Transform(loc.Transformation());
            }
            // Create cutting sections
            var listSections = new NCollection_List<TopoDS_Shape>();
            { // szv: Use brackets to destroy local variables
                // Obtain candidate vertices
                TopoDS_Vertex V1 = new(), V2 = new();
                var CandidateVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                { // szv: Use brackets to destroy local variables
                    // Create bounding box around curve
                    var aGlobalBox = new Bnd_Box();
                    var adptC = new GeomAdaptor_Curve(c3d, first, last);
                    BndLib_Add3dCurve.Add(adptC, myTolerance, ref aGlobalBox);
                    // Sort vertices to find candidates
                    aSelector.SetCurrent(aGlobalBox);
                    aTree.Select(aSelector);
                    // Skip bound if no node is in the boundind box
                    if (aSelector.ResInd().Extent() == 0)
                    {
                        continue;
                    }
                    // Retrieve bound nodes
                    TopExp.Vertices(bound, out V1, out V2);
                    TopoDS_Shape Node1 = myVertexNode.FindFromKey(V1);
                    TopoDS_Shape Node2 = myVertexNode.FindFromKey(V2);
                    // Fill map of candidate vertices
                    var itl = new NCollection_List<int>.Iterator(aSelector.ResInd());
                    for (; itl.More(); itl.Next())
                    {
                        int index = itl.Value();
                        TopoDS_Shape Node = myVertexNode.FindFromIndex(index);
                        if (!Node.IsSame(Node1) && !Node.IsSame(Node2))
                        {
                            TopoDS_Shape vertex = myVertexNode.FindKey(index);
                            CandidateVertices.Add(vertex);
                        }
                    }
                    aSelector.ClearResList();
                }
                int nbCandidates = CandidateVertices.Extent();
                if (nbCandidates == 0)
                {
                    continue;
                }
                // Project vertices on curve
                var arrPara = new NCollection_Array1<double>(1, nbCandidates);
                var arrDist = new NCollection_Array1<double>(1, nbCandidates);
                var arrPnt = new NCollection_Array1<gp_Pnt>(1, nbCandidates);
                var arrProj = new NCollection_Array1<gp_Pnt>(1, nbCandidates);
                for (int j = 1; j <= nbCandidates; j++)
                {
                    arrPnt[j] = BRep_Tool.Pnt(TopoDS.Vertex(CandidateVertices[j]));
                }
                ProjectPointsOnCurve(arrPnt, c3d, first, last, arrDist, arrPara, arrProj, true);
                // Create cutting nodes
                var seqNode = new NCollection_Sequence<TopoDS_Shape>();
                var seqPara = new NCollection_Sequence<double>();
                CreateCuttingNodes(CandidateVertices,
                                   bound,
                                   V1,
                                   V2,
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
                var its = new NCollection_List<TopoDS_Shape>.Iterator(listSections);
                for (; its.More(); its.Next())
                {
                    TopoDS_Shape section = its.Value();
                    // Iterate on section vertices
                    for (var itv = new TopoDS_Iterator(section); itv.More(); itv.Next())
                    {
                        TopoDS_Shape vertex = itv.Value();
                        // Convert vertex to node
                        if (myVertexNode.Contains(vertex))
                        {
                            vertex = TopoDS.Vertex(myVertexNode.FindFromKey(vertex));
                        }
                        // Update node sections
                        if (myNodeSections.IsBound(vertex))
                        {
                            myNodeSections.ChangeFind(vertex).Append(section);
                        }
                        else
                        {
                            var lsec = new NCollection_List<TopoDS_Shape>();
                            lsec.Append(section);
                            myNodeSections.Bind(vertex, lsec);
                        }
                    }
                    // Store bound for section
                    mySectionBound.Bind(section, bound);
                }
                // Store split bound
                myBoundSections.Bind(bound, listSections);
            }
        }
    }

    //=================================================================================================

    private static void GetSeqEdges(in TopoDS_Shape edge,
                                    NCollection_Sequence<TopoDS_Shape> seqEdges,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VertEdge)
    {
        int numV = 0;
        for (var Iv = new TopoDS_Iterator(edge, false); Iv.More(); Iv.Next())
        {
            TopoDS_Vertex V1 = TopoDS.Vertex(Iv.Value());
            numV++;
            if (VertEdge.IsBound(V1))
            {
                NCollection_List<TopoDS_Shape> listEdges = VertEdge.Find(V1);
                for (var lIt = new NCollection_List<TopoDS_Shape>.Iterator(listEdges); lIt.More(); lIt.Next())
                {
                    TopoDS_Shape edge1 = lIt.Value();
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
                        GetSeqEdges(edge1, seqEdges, VertEdge);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Chains the free edges into wires. Called from EdgeProcessing only.</summary>
    protected virtual void GetFreeWires(NCollection_IndexedMap<TopoDS_Shape> MapFreeEdges,
                                        NCollection_Sequence<TopoDS_Shape> seqWires)
    {
        var VertEdge = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var seqFreeEdges = new NCollection_Sequence<TopoDS_Shape>();
        for (int k = 1; k <= MapFreeEdges.Extent(); k++)
        {
            TopoDS_Shape edge = MapFreeEdges.FindKey(k);
            seqFreeEdges.Append(edge);
            for (var Iv = new TopoDS_Iterator(edge, false); Iv.More(); Iv.Next())
            {
                TopoDS_Vertex V1 = TopoDS.Vertex(Iv.Value());
                if (VertEdge.IsBound(V1))
                {
                    VertEdge.ChangeFind(V1).Append(edge);
                }
                else
                {
                    var ls = new NCollection_List<TopoDS_Shape>();
                    ls.Append(edge);
                    VertEdge.Bind(V1, ls);
                }
            }
        }
        var B = new BRep_Builder();
        int i, j;
        for (i = 1; i <= seqFreeEdges.Length(); i++)
        {
            var seqEdges = new NCollection_Sequence<TopoDS_Shape>();
            TopoDS_Shape edge = seqFreeEdges.Value(i);
            if (!MapFreeEdges.Contains(edge))
            {
                continue;
            }
            seqEdges.Append(edge);
            GetSeqEdges(edge, seqEdges, VertEdge);
            var wire = new TopoDS_Wire();
            B.MakeWire(ref wire);
            for (j = 1; j <= seqEdges.Length(); j++)
            {
                B.Add(wire, seqEdges.Value(j));
                MapFreeEdges.RemoveKey(seqEdges.Value(j));
            }
            seqWires.Append(wire);
            if (MapFreeEdges.IsEmpty())
            {
                break;
            }
        }
    }

    //=================================================================================================

    private static bool IsDegeneratedWire(in TopoDS_Shape wire)
    {
        if (wire.ShapeType() != TopAbs_WIRE)
        {
            return false;
        }
        // Get maximal vertices tolerance
        TopoDS_Vertex V1 = new(), V2 = new();
        double wireLength = 0.0;
        int nume = 0;
        int isSmall = 0;
        for (var aIt = new TopoDS_Iterator(wire, false); aIt.More(); aIt.Next())
        {
            nume++;
            TopoDS_Shape edge = aIt.Value();
            TopExp.Vertices(TopoDS.Edge(edge), out TopoDS_Vertex Ve1, out TopoDS_Vertex Ve2);
            if (nume == 1)
            {
                V1 = Ve1;
                V2 = Ve2;
            }
            else
            {
                if (Ve1.IsSame(V1))
                {
                    V1 = Ve2;
                }
                else if (Ve1.IsSame(V2))
                {
                    V2 = Ve2;
                }
                if (Ve2.IsSame(V1))
                {
                    V1 = Ve1;
                }
                else if (Ve2.IsSame(V2))
                {
                    V2 = Ve1;
                }
            }
            Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(aIt.Value()), out TopLoc_Location loc, out double first, out double last);
            if (c3d is not null)
            {
                c3d = (Geom_Curve)c3d.Copy();
                if (!loc.IsIdentity())
                {
                    c3d.Transform(loc.Transformation());
                }
                gp_Pnt pfirst = c3d.Value(first);
                gp_Pnt plast = c3d.Value(last);
                gp_Pnt pmid = c3d.Value((first + last) * 0.5);
                double length = 0;
                if (pfirst.Distance(plast) > pfirst.Distance(pmid))
                {
                    length = pfirst.Distance(plast);
                }
                else
                {
                    var cAdapt = new GeomAdaptor_Curve(c3d);
                    length = GCPnts_AbscissaPoint.Length(cAdapt, first, last);
                }
                double tole = BRep_Tool.Tolerance(Ve1) + BRep_Tool.Tolerance(Ve2);
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
        double tol = BRep_Tool.Tolerance(V1) + BRep_Tool.Tolerance(V2); // Max(BRep_Tool::Tolerance(V1),BRep_Tool::Tolerance(V2));
        return wireLength <= tol;
    }

    //=======================================================================
    // function :  DegeneratedSection
    // purpose  :  internal use
    //            create a new degenerated edge if the section is degenerated
    //=======================================================================

    private static TopoDS_Edge DegeneratedSection(in TopoDS_Shape section, in TopoDS_Shape face)
    {
        // Return if section is already degenerated
        if (BRep_Tool.Degenerated(TopoDS.Edge(section)))
        {
            return TopoDS.Edge(section);
        }

        // Retrieve edge curve
        Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(section), out TopLoc_Location loc, out double first, out double last);
        if (c3d is null)
        { // gka
            var aB = new BRep_Builder();
            TopoDS_Edge edge1 = TopoDS.Edge(section);
            aB.Degenerated(edge1, true);
            return edge1;
        }
        if (!loc.IsIdentity())
        {
            c3d = (Geom_Curve)c3d.Copy();
            c3d.Transform(loc.Transformation());
        }

        // Test if the new edge is degenerated
        TopExp.Vertices(TopoDS.Edge(section), out TopoDS_Vertex v1, out TopoDS_Vertex v2);

        gp_Pnt p1, p2, p3;
        p1 = BRep_Tool.Pnt(v1);
        p3 = BRep_Tool.Pnt(v2);
        c3d.D0(0.5 * (first + last), out p2);

        TopoDS_Edge edge;

        // processing
        var aBuilder = new BRep_Builder();
        edge = TopoDS.Edge(section);
        edge.EmptyCopy();
        if (v1.IsSame(v2))
        {
            TopoDS_Shape anEdge1 = edge.Oriented(TopAbs_FORWARD);
            aBuilder.Add(anEdge1, v1.Oriented(TopAbs_FORWARD));
            aBuilder.Add(anEdge1, v2.Oriented(TopAbs_REVERSED));
        }
        else
        {
            var newVertex = new TopoDS_Vertex();
            if (p1.Distance(p3) < BRep_Tool.Tolerance(v1))
            {
                newVertex = v1;
            }
            else if (p1.Distance(p3) < BRep_Tool.Tolerance(v2))
            {
                newVertex = v2;
            }
            else
            {
                double d1 = BRep_Tool.Tolerance(v1) + p2.Distance(p1);
                double d2 = BRep_Tool.Tolerance(v2) + p2.Distance(p3);
                double newTolerance = Max(d1, d2);
                aBuilder.MakeVertex(ref newVertex, p2, newTolerance);
            }
            TopoDS_Shape anEdge1 = edge.Oriented(TopAbs_FORWARD);
            aBuilder.Add(anEdge1, newVertex.Oriented(TopAbs_FORWARD));
            aBuilder.Add(anEdge1, newVertex.Oriented(TopAbs_REVERSED));
        }

        BRep_Tool.Range(TopoDS.Edge(section), out first, out last);
        TopoDS_Shape anEdge = edge.Oriented(TopAbs_FORWARD);
        aBuilder.Range(TopoDS.Edge(anEdge), first, last);
        aBuilder.Degenerated(edge, true);
        Geom_Curve? aC3dNew = null;
        if (!face.IsNull())
        {
            Geom2d_Curve? aC2dt =
                BRep_Tool.CurveOnSurface(TopoDS.Edge(section), TopoDS.Face(face), out double af, out double al);
            aBuilder.UpdateEdge(edge, aC3dNew, 0);
            Geom2d_Curve? aC2dn = BRep_Tool.CurveOnSurface(edge, TopoDS.Face(face), out af, out al);
            if (aC2dn is null)
            {
                aBuilder.UpdateEdge(edge, aC2dt, TopoDS.Face(face), 0);
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
    protected void EdgeProcessing(Message_ProgressRange theProgress = default)
    {
        // constructs sectionEdge
        var MapFreeEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var EdgeFace = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        using var aPS = new Message_ProgressScope(theProgress, "Edge processing", myBoundFaces.Extent());
        for (int anIterB = 1; anIterB <= myBoundFaces.Extent() && aPS.More(); anIterB++, aPS.Next())
        {
            TopoDS_Shape bound = myBoundFaces.FindKey(anIterB);
            NCollection_List<TopoDS_Shape> listFaces = myBoundFaces[anIterB];
            if (listFaces.Extent() == 1)
            {
                if (myBoundSections.IsBound(bound))
                {
                    var liter = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[bound]);
                    for (; liter.More(); liter.Next())
                    {
                        if (!myMergedEdges.Contains(liter.Value()))
                        { // myReShape->IsRecorded(liter.Value())) {
                            TopoDS_Shape edge = myReShape.InternalApply(liter.Value());
                            if (!MapFreeEdges.Contains(edge))
                            {
                                TopoDS_Shape face = listFaces.First();
                                EdgeFace.Bind(edge, face);
                                MapFreeEdges.Add(edge);
                            }
                        }
                    }
                }
                else
                {
                    if (!myMergedEdges.Contains(bound))
                    {
                        TopoDS_Shape edge = myReShape.InternalApply(bound);
                        if (!MapFreeEdges.Contains(edge))
                        {
                            TopoDS_Shape face = listFaces.First();
                            EdgeFace.Bind(edge, face);
                            MapFreeEdges.Add(edge);
                        }
                    }
                }
            }
        }

        if (!MapFreeEdges.IsEmpty())
        {
            var seqWires = new NCollection_Sequence<TopoDS_Shape>();
            GetFreeWires(MapFreeEdges, seqWires);
            for (int j = 1; j <= seqWires.Length(); j++)
            {
                TopoDS_Wire wire = TopoDS.Wire(seqWires.Value(j));
                if (!IsDegeneratedWire(wire))
                {
                    continue;
                }
                for (var Ie = new TopoDS_Iterator(wire, false); Ie.More(); Ie.Next())
                {
                    TopoDS_Shape aTmpShape = myReShape.InternalApply(Ie.Value()); // for porting
                    TopoDS_Edge edge = TopoDS.Edge(aTmpShape);
                    var face = new TopoDS_Shape();
                    if (EdgeFace.IsBound(edge))
                    {
                        face = EdgeFace.Find(edge);
                    }
                    TopoDS_Shape degedge = DegeneratedSection(edge, face);
                    if (degedge.IsNull())
                    {
                        continue;
                    }
                    if (!degedge.IsSame(edge))
                    {
                        ReplaceEdge(edge, degedge, myReShape);
                    }
                    if (BRep_Tool.Degenerated(TopoDS.Edge(degedge)))
                    {
                        myDegenerated.Add(degedge);
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
    protected void EdgeRegularity(Message_ProgressRange theProgress = default)
    {
        var aMapEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(mySewedShape, TopAbs_EDGE, TopAbs_FACE, aMapEF);

        using var aPS = new Message_ProgressScope(theProgress, "Encode edge regularity", myMergedEdges.Extent());
        // OCCT iterates its map in address order; the port in insertion order (decision 18). Each edge is encoded on
        // its own, so the order doesn't change the result.
        foreach (TopoDS_Shape aMergedEdge in myMergedEdges)
        {
            if (!aPS.More())
            {
                break;
            }
            TopoDS_Edge anEdge = TopoDS.Edge(myReShape.InternalApply(aMergedEdge));
            // encode regularity if and only if edges is shared by two faces
            if (aMapEF.FindFromKey(anEdge, out NCollection_List<TopoDS_Shape> aFaces) && aFaces.Extent() == 2)
            {
                BRepLib.EncodeRegularity(anEdge,
                                         TopoDS.Face(aFaces.First()),
                                         TopoDS.Face(aFaces.Last()));
            }
            aPS.Next();
        }

        myMergedEdges.Clear(); // OCCT: CreateOutputInformations reads it afterwards, so no section is bound after sewing
    }

    //=================================================================================================

    /// <summary>Builds the sewn shape from the reshaped inputs. Called from Perform only.</summary>
    protected virtual void CreateSewedShape()
    {
        // ---------------------
        // create the new shapes
        // ---------------------
        var aQuilt = new BRepTools_Quilt();
        bool isLocal = !myShape.IsNull();
        if (isLocal)
        {
            // Local sewing
            TopoDS_Shape ns = myReShape.InternalApply(myShape);
            aQuilt.Add(ns);
        }
        for (int anIter = 1; anIter <= myOldShapes.Extent(); anIter++)
        {
            TopoDS_Shape sh = myOldShapes[anIter];
            if (!sh.IsNull())
            {
                sh = myReShape.InternalApply(sh);
                myOldShapes[anIter] = sh;
                if (!isLocal)
                {
                    aQuilt.Add(sh);
                }
            }
        }
        TopoDS_Shape aNewShape = aQuilt.InternalShells();
        int numsh = 0;

        var OldShells = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        var aB = new BRep_Builder();
        var aComp = new TopoDS_Compound();
        aB.MakeCompound(ref aComp);
        for (var aExpSh = new TopoDS_Iterator(aNewShape, false); aExpSh.More(); aExpSh.Next())
        {
            TopoDS_Shape sh = aExpSh.Value();
            bool hasEdges = false;
            if (sh.ShapeType() == TopAbs_SHELL)
            {
                if (myNonmanifold)
                {
                    hasEdges = !OldShells.Contains(sh);
                }
                else
                {
                    var face = new TopoDS_Shape();
                    int numf = 0;
                    for (var aExpF = new TopExp_Explorer(sh, TopAbs_FACE); aExpF.More() && (numf < 2); aExpF.Next())
                    {
                        face = aExpF.Current();
                        numf++;
                    }
                    if (numf == 1)
                    {
                        aB.Add(aComp, face);
                    }
                    else if (numf > 1)
                    {
                        aB.Add(aComp, sh);
                    }
                    if (numf != 0)
                    {
                        numsh++;
                    }
                }
            }
            else if (sh.ShapeType() == TopAbs_FACE)
            {
                if (myNonmanifold)
                {
                    var ss = new TopoDS_Shell();
                    aB.MakeShell(ref ss);
                    aB.Add(ss, sh);
                    sh = ss;
                    hasEdges = true;
                }
                else
                {
                    aB.Add(aComp, sh);
                    numsh++;
                }
            }
            else
            {
                aB.Add(aComp, sh);
                numsh++;
            }
            if (hasEdges)
            {
                OldShells.Add(sh);
            }
        }
        // Process collected shells
        if (myNonmanifold)
        {
            int nbOldShells = OldShells.Extent();
            if (nbOldShells == 1)
            {
                // Single shell - check for single face
                TopoDS_Shape sh = OldShells.FindKey(1);
                var face = new TopoDS_Shape();
                int numf = 0;
                for (var aExpF = new TopExp_Explorer(sh, TopAbs_FACE); aExpF.More() && (numf < 2); aExpF.Next())
                {
                    face = aExpF.Current();
                    numf++;
                }
                if (numf == 1)
                {
                    aB.Add(aComp, face);
                }
                else if (numf > 1)
                {
                    aB.Add(aComp, sh);
                }
                if (numf != 0)
                {
                    numsh++;
                }
            }
            else if (nbOldShells != 0)
            {
                // Several shells should be merged
                var IndexMerged = new NCollection_Map<int>();
                while (IndexMerged.Extent() < nbOldShells)
                {
                    var NewShell = new TopoDS_Shell();
                    var NewEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    for (int i = 1; i <= nbOldShells; i++)
                    {
                        if (IndexMerged.Contains(i))
                        {
                            continue;
                        }
                        TopoDS_Shell shell = TopoDS.Shell(OldShells.FindKey(i));
                        if (NewShell.IsNull())
                        {
                            aB.MakeShell(ref NewShell);
                            var aItSS = new TopoDS_Iterator(shell);
                            for (; aItSS.More(); aItSS.Next())
                            {
                                aB.Add(NewShell, aItSS.Value());
                            }
                            // Fill map of edges
                            for (var eexp = new TopExp_Explorer(shell, TopAbs_EDGE); eexp.More(); eexp.Next())
                            {
                                TopoDS_Shape edge = eexp.Current();
                                NewEdges.Add(edge);
                            }
                            IndexMerged.Add(i);
                        }
                        else
                        {
                            bool hasSharedEdge = false;
                            var eexp = new TopExp_Explorer(shell, TopAbs_EDGE);
                            for (; eexp.More() && !hasSharedEdge; eexp.Next())
                            {
                                hasSharedEdge = NewEdges.Contains(eexp.Current());
                            }
                            if (hasSharedEdge)
                            {
                                // Add edges to the map
                                for (var eexp1 = new TopExp_Explorer(shell, TopAbs_EDGE); eexp1.More(); eexp1.Next())
                                {
                                    TopoDS_Shape edge = eexp1.Current();
                                    NewEdges.Add(edge);
                                }
                                // Add faces to the shell
                                for (var fexp = new TopExp_Explorer(shell, TopAbs_FACE); fexp.More(); fexp.Next())
                                {
                                    TopoDS_Shape face = fexp.Current();
                                    aB.Add(NewShell, face);
                                }
                                IndexMerged.Add(i);
                            }
                        }
                    }
                    // Process new shell
                    var face1 = new TopoDS_Shape();
                    int numf = 0;
                    var aExpF = new TopExp_Explorer(NewShell, TopAbs_FACE);
                    for (; aExpF.More() && (numf < 2); aExpF.Next())
                    {
                        face1 = aExpF.Current();
                        numf++;
                    }
                    if (numf == 1)
                    {
                        aB.Add(aComp, face1);
                    }
                    else if (numf > 1)
                    {
                        aB.Add(aComp, NewShell);
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
            var aIt = new TopoDS_Iterator(aComp, false);
            mySewedShape = aIt.Value();
        }
        else
        {
            mySewedShape = aComp;
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
        var edgeSections = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= myBoundFaces.Extent(); i++)
        {
            TopoDS_Shape bound = myBoundFaces.FindKey(i);
            var lsect = new NCollection_List<TopoDS_Shape>();
            if (myBoundSections.IsBound(bound))
            {
                lsect.Assign(myBoundSections[bound]);
            }
            var aExp = new TopExp_Explorer(myReShape.InternalApply(bound), TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape sec = bound;
                TopoDS_Shape edge = aExp.Current();
                var aI = new NCollection_List<TopoDS_Shape>.Iterator(lsect);
                for (; aI.More(); aI.Next())
                {
                    TopoDS_Shape section = aI.Value();
                    if (edge.IsSame(myReShape.InternalApply(section)))
                    {
                        sec = section;
                        break;
                    }
                }
                if (edgeSections.Contains(edge))
                {
                    edgeSections.ChangeFromKey(edge).Append(sec);
                }
                else
                {
                    var listSec = new NCollection_List<TopoDS_Shape>();
                    listSec.Append(sec);
                    edgeSections.Add(edge, listSec);
                }
            }
        }

        // Fill maps of Free, Contiguous and Multiple edges
        for (int anIter = 1; anIter <= edgeSections.Extent(); anIter++)
        {
            TopoDS_Shape edge = edgeSections.FindKey(anIter);
            NCollection_List<TopoDS_Shape> listSection = edgeSections[anIter];
            if (listSection.Extent() == 1)
            {
                if (BRep_Tool.Degenerated(TopoDS.Edge(edge)))
                {
                    myDegenerated.Add(edge);
                }
                else
                {
                    myFreeEdges.Add(edge);
                }
            }
            else if (listSection.Extent() == 2)
            {
                myContigousEdges.Add(edge, listSection);
            }
            else
            {
                myMultipleEdges.Add(edge);
            }
        }

        // constructs myContigSectBound
        for (i = 1; i <= myBoundFaces.Extent(); i++)
        {
            TopoDS_Shape bound = myBoundFaces.FindKey(i);
            if (myBoundSections.IsBound(bound))
            {
                var iter = new NCollection_List<TopoDS_Shape>.Iterator(myBoundSections[bound]);
                for (; iter.More(); iter.Next())
                {
                    TopoDS_Shape section = iter.Value();
                    if (!myMergedEdges.Contains(section)) // OCCT: empty after EdgeRegularity when sewing
                    {
                        continue;
                    }
                    // if (!myReShape->IsRecorded(section)) continue; // section is free
                    TopoDS_Shape nedge = myReShape.InternalApply(section);
                    if (nedge.IsNull())
                    {
                        continue; // szv debug
                    }
                    if (!bound.IsSame(section))
                    {
                        if (myContigousEdges.Contains(nedge))
                        {
                            myContigSecBound.Bind(section, bound);
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
    protected void ProjectPointsOnCurve(NCollection_Array1<gp_Pnt> arrPnt,
                                        Geom_Curve? c3d,
                                        double first,
                                        double last,
                                        NCollection_Array1<double> arrDist,
                                        NCollection_Array1<double> arrPara,
                                        NCollection_Array1<gp_Pnt> arrProj,
                                        bool isConsiderEnds)
    {
        arrDist.Init(-1.0);

        var GAC = new GeomAdaptor_Curve(c3d!);
        var locProj = new Extrema_ExtPC();
        locProj.Initialize(GAC, first, last);
        gp_Pnt pfirst = GAC.Value(first), plast = GAC.Value(last);
        int find = 1;              //(isConsiderEnds ? 1 : 2);
        int lind = arrPnt.Length(); //(isConsiderEnds ? arrPnt.Length() : arrPnt.Length() -1);

        for (int i1 = find; i1 <= lind; i1++)
        {
            gp_Pnt pt = arrPnt[i1];
            double worktol = myTolerance;
            double distF2 = pfirst.SquareDistance(pt);
            double distL2 = plast.SquareDistance(pt);
            bool isProjected = false;
            try
            {
                // Project current point on curve
                locProj.Perform(pt);
                if (locProj.IsDone() && locProj.NbExt() > 0)
                {
                    double dist2Min = (isConsiderEnds || i1 == find || i1 == lind ? Min(distF2, distL2)
                                                                                 : Precision.Infinite());
                    int ind, indMin = 0;
                    for (ind = 1; ind <= locProj.NbExt(); ind++)
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
                        Extrema_POnCurv pOnC = locProj.Point(indMin);
                        double paramProj = pOnC.Parameter();
                        gp_Pnt ptProj = GAC.Value(paramProj);
                        double distProj2 = ptProj.SquareDistance(pt);
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
            catch (Standard_Failure)
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
    protected virtual void CreateCuttingNodes(NCollection_IndexedMap<TopoDS_Shape> MapVert,
                                              in TopoDS_Shape bound,
                                              in TopoDS_Shape vfirst,
                                              in TopoDS_Shape vlast,
                                              NCollection_Array1<double> arrDist,
                                              NCollection_Array1<double> arrPara,
                                              NCollection_Array1<gp_Pnt> arrPnt,
                                              NCollection_Sequence<TopoDS_Shape> seqVert,
                                              NCollection_Sequence<double> seqPara)
    {
        int i, j, nbProj = MapVert.Extent();

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

        var aBuilder = new BRep_Builder();

        // Insert two initial vertices (to be removed later)
        var seqDist = new NCollection_Sequence<double>();
        var seqPnt = new NCollection_Sequence<gp_Pnt>();
        { // szv: Use brackets to destroy local variables
            // Retrieve bound curve
            Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(bound), out TopLoc_Location loc, out double first, out double last);
            if (!loc.IsIdentity())
            {
                c3d = (Geom_Curve)c3d!.Copy();
                c3d.Transform(loc.Transformation());
            }
            var GAC = new GeomAdaptor_Curve(c3d!);
            seqVert.Prepend(vfirst);
            seqVert.Append(vlast);
            seqPara.Prepend(first);
            seqPara.Append(last);
            seqDist.Prepend(-1.0);
            seqDist.Append(-1.0);
            seqPnt.Prepend(GAC.Value(first));
            seqPnt.Append(GAC.Value(last));
        }

        var NodeCuttingVertex = new NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= nbProj; i++)
        {
            int index = seqOrderedIndex[i];
            double disProj = arrDist[index];
            gp_Pnt pntProj = arrPnt[index];

            // Skip node if already bound to cutting vertex
            TopoDS_Shape node = myVertexNode.FindFromKey(MapVert[index]);
            if (NodeCuttingVertex.Contains(node))
            {
                continue;
            }

            // Find the closest vertex
            int indexMin = 1;
            double dist, distMin = pntProj.Distance(seqPnt[1]);
            for (j = 2; j <= seqPnt.Length(); j++)
            {
                dist = pntProj.Distance(seqPnt[j]);
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
                    TopoDS_Shape cvertex = seqVert.Value(indexMin);
                    NodeCuttingVertex.Add(node, cvertex);
                }
                else
                {
                    // Bind secondary cutting nodes
                    NodeCuttingVertex.Add(node, new TopoDS_Vertex());
                }
            }
            else
            {
                // Build new cutting vertex
                var cvertex = new TopoDS_Vertex();
                aBuilder.MakeVertex(ref cvertex, pntProj, Precision.Confusion());
                // Bind new cutting vertex
                NodeCuttingVertex.Add(node, cvertex);
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
        for (int aMIt = 1; aMIt <= NodeCuttingVertex.Extent(); aMIt++)
        {
            TopoDS_Shape cnode = NodeCuttingVertex[aMIt];
            // Skip secondary nodes
            if (cnode.IsNull())
            {
                continue;
            }
            // Obtain vertex node
            TopoDS_Shape node = NodeCuttingVertex.FindKey(aMIt);
            if (myVertexNode.Contains(cnode))
            {
                // This is an end vertex
                cnode = myVertexNode.FindFromKey(cnode);
            }
            else
            {
                // Create link: cutting vertex -> node
                var ls1 = new NCollection_List<TopoDS_Shape>();
                ls1.Append(node);
                myCuttingNode.Bind(cnode, ls1);
            }
            // Create link: node -> cutting vertex
            if (myCuttingNode.IsBound(node))
            {
                myCuttingNode.ChangeFind(node).Append(cnode);
            }
            else
            {
                var ls = new NCollection_List<TopoDS_Shape>();
                ls.Append(cnode);
                myCuttingNode.Bind(node, ls);
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
    protected virtual void CreateSections(in TopoDS_Shape section,
                                          NCollection_Sequence<TopoDS_Shape> seqNode,
                                          NCollection_Sequence<double> seqPara,
                                          NCollection_List<TopoDS_Shape> listEdge)
    {
        TopoDS_Edge sec = TopoDS.Edge(section);
        //  TopAbs_Orientation aInitOr = sec.Orientation();

        // To keep NM vertices on edge
        var aSeqNMVert = new NCollection_Sequence<TopoDS_Shape>();
        var aSeqNMPars = new NCollection_Sequence<double>();
        findNMVertices(sec, aSeqNMVert, aSeqNMPars);

        var aBuilder = new BRep_Builder();

        BRep_Tool.Range(sec, out double first, out double last);

        // Create cutting sections
        double par1, par2;
        TopoDS_Shape V1, V2;
        int i, len = seqPara.Length() + 1;
        for (i = 1; i <= len; i++)
        {
            TopoDS_Edge edge = sec;
            edge.EmptyCopy();

            if (i == 1)
            {
                par1 = first;
                par2 = seqPara[i];
                V1 = TopExp.FirstVertex(sec);
                V2 = seqNode[i];
            }
            else if (i == len)
            {
                par1 = seqPara[i - 1];
                par2 = last;
                V1 = seqNode[i - 1];
                V2 = TopExp.LastVertex(sec);
            }
            else
            {
                par1 = seqPara[i - 1];
                par2 = seqPara[i];
                V1 = seqNode[i - 1];
                V2 = seqNode[i];
            }

            TopoDS_Shape aTmpShape = edge.Oriented(TopAbs_FORWARD);
            TopoDS_Edge aTmpEdge = TopoDS.Edge(aTmpShape); // for porting
            aTmpShape = V1.Oriented(TopAbs_FORWARD);
            aBuilder.Add(aTmpEdge, aTmpShape);
            aTmpShape = V2.Oriented(TopAbs_REVERSED);
            aBuilder.Add(aTmpEdge, aTmpShape);
            aBuilder.Range(aTmpEdge, par1, par2);

            int k = 1;
            for (; k <= aSeqNMPars.Length(); k++)
            {
                double apar = aSeqNMPars.Value(k);
                if (apar >= par1 && apar <= par2)
                {
                    aBuilder.Add(aTmpEdge, aSeqNMVert.Value(k));
                    aSeqNMVert.Remove(k);
                    aSeqNMPars.Remove(k);
                    k--;
                }
            }
            listEdge.Append(edge);
        }

        NCollection_List<TopoDS_Shape> listFaces = myBoundFaces.FindFromKey(sec);
        if (listFaces.Extent() == 0)
        {
            return;
        }

        double tolEdge = BRep_Tool.Tolerance(sec);

        // Add cutting pcurves
        var itf = new NCollection_List<TopoDS_Shape>.Iterator(listFaces);
        for (; itf.More(); itf.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itf.Value());

            // Retrieve curve on surface
            double first2d1 = 0, last2d1 = 0.0;
            Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(sec, fac, out double first2d, out double last2d);
            if (c2d is null)
            {
                continue;
            }
            Geom2d_Curve? c2d1 = null;
            bool isSeam = BRep_Tool.IsClosed(sec, fac);

            // gka  - Convert to BSpline was commented because
            // it is not necessary to create BSpline instead of Lines or cIrcles.
            // Besides after conversion circles to BSpline
            // it is necessary to recompute parameters of cutting because parameterization of created
            // BSpline curve differs from parametrization of circle.

            // gka fix for bug OCC12203 21.04.06 addition second curve for seam edges

            if (isSeam)
            {
                TopoDS_Edge secRev = TopoDS.Edge(sec.Reversed());

                c2d1 = BRep_Tool.CurveOnSurface(secRev, fac, out first2d1, out last2d1);
                if (c2d1 is null)
                {
                    continue;
                }
            }

            // Update cutting sections
            Geom2d_Curve? c2dNew = null, c2d1New = null;
            var ite = new NCollection_List<TopoDS_Shape>.Iterator(listEdge);
            for (; ite.More(); ite.Next())
            {
                // Retrieve cutting section
                TopoDS_Edge edge = TopoDS.Edge(ite.Value());
                BRep_Tool.Range(edge, out par1, out par2);

                // Cut BSpline pcurve
                c2dNew = (Geom2d_Curve)c2d.Copy();
                if (c2d1 is not null)
                {
                    c2d1New = (Geom2d_Curve)c2d1.Copy();
                }

                if (!isSeam && c2d1New is null)
                {
                    aBuilder.UpdateEdge(edge, c2dNew, fac, tolEdge);
                }
                else
                {
                    TopAbs_Orientation Ori = edge.Orientation();
                    if (fac.Orientation() == TopAbs_REVERSED)
                    {
                        Ori = TopAbs.Reverse(Ori);
                    }

                    if (Ori == TopAbs_FORWARD)
                    {
                        aBuilder.UpdateEdge(edge, c2dNew, c2d1New, fac, tolEdge);
                    }
                    else
                    {
                        aBuilder.UpdateEdge(edge, c2d1New, c2dNew, fac, tolEdge);
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
        if (!mySameParameterMode)
        {
            return;
        }
        var exp = new TopExp_Explorer(mySewedShape, TopAbs_EDGE);
        // Le flag sameparameter est a false pour chaque edge cousue
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge sec = TopoDS.Edge(exp.Current());
            try
            {
                BRepLib.SameParameter(sec, BRep_Tool.Tolerance(sec));
            }
            catch (Standard_Failure)
            {
                continue;
            }
        }
    }

    //=================================================================================================

    /// <summary>The context, which records the replacements made by sewing.</summary>
    /// <remarks>When the inputs were copied (they held frozen shapes), it records the working copies' sub-shapes.</remarks>
    public BRepTools_ReShape GetContext() => myReShape;

    //=================================================================================================

    /// <summary>Sets the context.</summary>
    public void SetContext(BRepTools_ReShape theContext) => myReShape = theContext;
}
