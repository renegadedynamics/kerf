// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FaceConnect.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FaceConnect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Rebuilds the connectivity between faces of a shell: the free edges of the pairs of faces recorded with
/// <see cref="Add"/> are sewn, the edges replaced by the sewn ones, the wires fixed, and the vertices merged.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Add"/> records the connections; <see cref="Build"/> runs OCCT's
/// algorithm, which writes into the shell's sub-shapes in place (sewing same-parameters the free edges, the merged
/// vertices are moved and their tolerances raised). A shell holding frozen shapes (any shell a caller holds) is copied
/// first (<see cref="BRepTools_CopyAtEntry"/>, decision 583), the recorded faces mapped onto their copies: the
/// caller's shell never changes, the result holds the caller's own sub-shapes wherever they are unchanged (the shell
/// itself when nothing changed), and it is frozen. <see cref="InternalBuild"/> doesn't copy or freeze, for ported code
/// connecting a shell it owns.
/// </para>
/// </remarks>
public sealed class ShapeFix_FaceConnect
{
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myConnected;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOriFreeEdges;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myResFreeEdges;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myResSharEdges;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_FaceConnect()
    {
        myConnected    = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        myOriFreeEdges = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        myResFreeEdges = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        myResSharEdges = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
    }

    /// <summary>Records that the two faces are to be connected; false if one is null.</summary>
    public bool Add(in TopoDS_Face aFirst, in TopoDS_Face aSecond)
    {
        if (!aFirst.IsNull() && !aSecond.IsNull())
        {
            // Process first face
            if (myConnected.IsBound(aFirst))
            {
                // Find list for the first face
                NCollection_List<TopoDS_Shape> theFirstList = myConnected[aFirst];
                // Append second face to the first list
                var theIter = new NCollection_List<TopoDS_Shape>.Iterator();
                for (theIter.Initialize(theFirstList); theIter.More(); theIter.Next())
                {
                    if (theIter.Value().IsSame(aSecond))
                    {
                        return true;
                    }
                }
                theFirstList.Append(aSecond);
            }
            else
            {
                // Append second face to the first list
                var theNewFirstList = new NCollection_List<TopoDS_Shape>();
                theNewFirstList.Append(aSecond);
                myConnected.Bind(aFirst, theNewFirstList);
            }

            // Process second face if not same
            if (!aFirst.IsSame(aSecond))
            {
                if (myConnected.IsBound(aSecond))
                {
                    // No need to iterate on faces - append first
                    myConnected[aSecond].Append(aFirst);
                }
                else
                {
                    // Append first face to the second list
                    var theNewSecondList = new NCollection_List<TopoDS_Shape>();
                    theNewSecondList.Append(aFirst);
                    myConnected.Bind(aSecond, theNewSecondList);
                }
            }
            return true;
        }

        return false;
    }

    /// <summary>
    /// Connects the recorded faces of <paramref name="shell"/>: sews their free edges with <paramref name="sewtoler"/>,
    /// fixes the wires with <paramref name="fixtoler"/> and merges the vertices. Returns the new shell, frozen (the
    /// shell itself if nothing was connected); the caller's shell is unchanged.
    /// </summary>
    public TopoDS_Shell Build(in TopoDS_Shell shell, double sewtoler, double fixtoler)
    {
        // OCCT.NET: OCCT's algorithm writes into the shell's sub-shapes; a frozen shell is copied (decision 583), and the
        // recorded faces are mapped onto their copies
        var anEntry = new BRepTools_CopyAtEntry(shell);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aConnected = myConnected;
        if (anEntry.IsCopied())
        {
            aConnected = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aPair in myConnected)
            {
                var aList = new NCollection_List<TopoDS_Shape>();
                for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(aPair.Value); anIt.More(); anIt.Next())
                {
                    aList.Append(anEntry.Copied(anIt.Value()));
                }
                aConnected.Bind(anEntry.Copied(aPair.Key), aList);
            }
        }

        TopoDS_Shell aResult = build(aConnected, TopoDS.Shell(anEntry.Shape()), sewtoler, fixtoler);
        TopoDS_Shape aFinal = anEntry.Finish(aResult);
        aFinal.TShape()?.FreezeGraph();
        return TopoDS.Shell(aFinal);
    }

    /// <summary>
    /// OCCT's Build: connects the recorded faces of <paramref name="shell"/> in place (its sub-shapes are written).
    /// For ported code that owns the shell.
    /// </summary>
    internal TopoDS_Shell InternalBuild(in TopoDS_Shell shell, double sewtoler, double fixtoler) =>
        build(myConnected, shell, sewtoler, fixtoler);

    private TopoDS_Shell build(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theConnected,
                               in TopoDS_Shell shell,
                               double sewtoler,
                               double fixtoler)
    {
        TopoDS_Shell result = shell;

        /***************************************************************
        / INITIAL PREPARATIONS
        / Fill map of original free edges,
        / fill maps of resulting free and shared edges
        ***************************************************************/

        // Clear maps of free and shared edges
        myOriFreeEdges.Clear();
        myResFreeEdges.Clear();
        myResSharEdges.Clear();

        // OCCT iterates this map in hash order (TShape addresses); the port's iterates in a deterministic order, so the
        // order of the free edges (and of the sewing's inputs) may differ from native (occt-issues.md).
        var theFreeEdges = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopoDS_Shape theEdge, theFace;

        // Fill map of free edges / faces
        for (var itf = new TopoDS_Iterator(result); itf.More(); itf.Next())
        {
            theFace = itf.Value();
            for (var expe = new TopExp_Explorer(theFace, TopAbs_EDGE); expe.More(); expe.Next())
            {
                theEdge = expe.Current();
                if (theFreeEdges.IsBound(theEdge))
                {
                    theFreeEdges.UnBind(theEdge);
                }
                else
                {
                    theFreeEdges.Bind(theEdge, theFace);
                }
            }
        }

        // Fill maps of original and resulting edges
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> theFEIter in theFreeEdges)
        {
            // Get pair (face / free edge)
            theEdge = theFEIter.Key;
            theFace = theFEIter.Value;
            // Process faces with bad connectivities only
            if (theConnected.IsBound(theFace) && !BRep_Tool.Degenerated(TopoDS.Edge(theEdge)))
            {
                // Add to the map of original free edges
                if (myOriFreeEdges.IsBound(theFace))
                {
                    // Append free edge to the existing list
                    myOriFreeEdges[theFace].Append(theEdge);
                }
                else
                {
                    // Append free edge to the new list
                    var theNewList = new NCollection_List<TopoDS_Shape>();
                    theNewList.Append(theEdge);
                    myOriFreeEdges.Bind(theFace, theNewList);
                }
                // Add to the maps of intermediate free and resulting edges
                if (!myResFreeEdges.IsBound(theEdge))
                {
                    var theFree = new NCollection_List<TopoDS_Shape>();
                    var theShared = new NCollection_List<TopoDS_Shape>();
                    theFree.Append(theEdge);
                    myResFreeEdges.Bind(theEdge, theFree);
                    myResSharEdges.Bind(theEdge, theShared);
                }
            }
        }

        // Clear the temporary map of free edges
        theFreeEdges.Clear();

        /***************************************************************
        / APPLY SEWING ON CONNECTED FACES
        / Change maps of original free edges and resulting shared edges
        ***************************************************************/

        if (!myOriFreeEdges.IsEmpty())
        {
            // Allocate array of faces to be sewed
            TopoDS_Shape theFirstFace, theSecondFace;
            var theFacesToSew = new NCollection_Array1<TopoDS_Shape>(1, 2);
            int theNumOfFacesToSew = 0;
            bool skip_pair = false;
            var theOriginalIter = new NCollection_List<TopoDS_Shape>.Iterator();
            var theResultsIter = new NCollection_List<TopoDS_Shape>.Iterator();
            TopoDS_Shape theAuxE, theOrigE, theAuxF;
            var theBuilder = new BRep_Builder();
            var theProcessed = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theConnectedIter in theConnected)
            {
                // Process first face only if it is in the map of faces / free edges
                theFirstFace = theConnectedIter.Key;
                if (myOriFreeEdges.IsBound(theFirstFace))
                {
                    // Place first face into the array
                    theFacesToSew.SetValue(1, theFirstFace);
                    theNumOfFacesToSew = 1;

                    // Create the list of processed faces
                    var theProcessedList = new NCollection_List<TopoDS_Shape>();

                    // Explore the list of connected faces
                    NCollection_List<TopoDS_Shape> theConnectedList = theConnectedIter.Value;
                    var theConnectedListIter = new NCollection_List<TopoDS_Shape>.Iterator();
                    for (theConnectedListIter.Initialize(theConnectedList); theConnectedListIter.More();
                         theConnectedListIter.Next())
                    {
                        // Process second face only if it is in the map of faces / free edges
                        theSecondFace = theConnectedListIter.Value();
                        if (myOriFreeEdges.IsBound(theSecondFace))
                        {
                            // Place second face into the array
                            theFacesToSew.SetValue(2, theSecondFace);
                            // Add second face to the list of processed faces
                            theProcessedList.Append(theSecondFace);

                            // Skip the pair if already processed
                            skip_pair = false;
                            if (theProcessed.IsBound(theSecondFace))
                            {
                                NCollection_List<TopoDS_Shape> theProcCnxList = theProcessed[theSecondFace];
                                var theProcCnxListIter = new NCollection_List<TopoDS_Shape>.Iterator();
                                for (theProcCnxListIter.Initialize(theProcCnxList);
                                     theProcCnxListIter.More() && !skip_pair;
                                     theProcCnxListIter.Next())
                                {
                                    if (theFirstFace.IsSame(theProcCnxListIter.Value()))
                                    {
                                        skip_pair = true;
                                    }
                                }
                            }

                            if (!skip_pair)
                            {
                                // Process second face for the pair of different faces only
                                if (theFirstFace.IsSame(theSecondFace))
                                {
                                    // Self-connected face (OCCT_DEBUG warning)
                                }
                                else
                                {
                                    theNumOfFacesToSew = 2;
                                }

                                var theSewerWires = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                                var theSewer = new BRepBuilderAPI_Sewing(sewtoler);

                                // Prepare set of faces containing free edges
                                int i = 1;
                                for (i = 1; i <= theNumOfFacesToSew; i++)
                                {
                                    // Prepare empty face to fill with free edges
                                    TopoDS_Shape theFaceToSew = theFacesToSew[i];
                                    theAuxF = theFaceToSew.EmptyCopied();
                                    // Fill empty face with free edges
                                    for (theOriginalIter.Initialize(myOriFreeEdges[theFaceToSew]);
                                         theOriginalIter.More();
                                         theOriginalIter.Next())
                                    {
                                        for (theResultsIter.Initialize(myResFreeEdges[theOriginalIter.Value()]);
                                             theResultsIter.More();
                                             theResultsIter.Next())
                                        {
                                            // Bind free edge to wire to find results later
                                            theAuxE = theResultsIter.Value();
                                            var theAuxW = new TopoDS_Wire();
                                            theBuilder.MakeWire(ref theAuxW);
                                            theBuilder.Add(theAuxW, theAuxE);
                                            theBuilder.Add(theAuxF, theAuxW);
                                            theSewerWires.Bind(theAuxE, theAuxW);
                                            theSewer.Add(theAuxW);
                                        }
                                    }
                                    // Add constructed face to sewer
                                    theSewer.Add(theAuxF);
                                }

                                // Perform sewing on the list of free edges
                                bool sewing_ok = true;
                                {
                                    try
                                    {
                                        theSewer.Perform();
                                    }
                                    catch (Standard_Failure)
                                    {
                                        sewing_ok = false;
                                    }
                                }
                                if (sewing_ok)
                                {
                                    // OCCT.NET: the internal forms (decision 606): the shapes sewn are the ones being built
                                    if (theSewer.InternalSewedShape().IsNull())
                                    {
                                        sewing_ok = false;
                                    }
                                }

                                if (sewing_ok)
                                {
                                    var theResultEdges = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

                                    // Find modified edges for the faces
                                    for (i = 1; i <= theNumOfFacesToSew; i++)
                                    {
                                        for (theOriginalIter.Initialize(myOriFreeEdges[theFacesToSew[i]]);
                                             theOriginalIter.More();
                                             theOriginalIter.Next())
                                        {
                                            // Get original free edge
                                            theOrigE = theOriginalIter.Value();
                                            NCollection_List<TopoDS_Shape> theOldFreeList = myResFreeEdges[theOrigE];
                                            theResultsIter.Initialize(theOldFreeList);
                                            while (theResultsIter.More())
                                            {
                                                theAuxE = theSewerWires[theResultsIter.Value()];
                                                // Process modified edges
                                                if (theSewer.IsModified(theAuxE))
                                                {
                                                    // Fill map of result edges
                                                    for (var expe = new TopExp_Explorer(theSewer.InternalModified(theAuxE), TopAbs_EDGE);
                                                         expe.More();
                                                         expe.Next())
                                                    {
                                                        theAuxE = expe.Current();
                                                        // Check edge for being shared
                                                        if (theResultEdges.IsBound(theAuxE))
                                                        {
                                                            // Edge was shared - move in results list
                                                            myResSharEdges[theResultEdges[theAuxE]].Append(theAuxE);
                                                            myResSharEdges[theOrigE].Append(theAuxE);
                                                            theResultEdges.UnBind(theAuxE);
                                                        }
                                                        else
                                                        {
                                                            theResultEdges.Bind(theAuxE, theOrigE);
                                                        }
                                                    }
                                                    // Remove modified free edge from the list
                                                    theOldFreeList.Remove(theResultsIter);
                                                }
                                                else
                                                {
                                                    theResultsIter.Next();
                                                }
                                            }
                                        }
                                    }

                                    // Put free edges back to the lists of results
                                    foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> theResIter in theResultEdges)
                                    {
                                        theAuxE = theResIter.Key;
                                        myResFreeEdges[theResIter.Value].Append(theAuxE);
                                    }
                                }
                            }
                        }
                    }

                    // Bind the list of processed faces to the processed face
                    theProcessed.Bind(theFirstFace, theProcessedList);
                }
            }

            // Clear the temporary map of processed faces
            theProcessed.Clear();

            /***************************************************************
            / PERFORM EDGES REPLACEMENT
            ***************************************************************/

            var theRepEdges = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var theRepVertices = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            var theOldVertices = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var theNewVertices = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

            // Replace old edges by resulting ones
            var theNewW = new TopoDS_Wire();
            TopoDS_Vertex theOldV1 = new(), theOldV2 = new(), theNewV1 = new(), theNewV2 = new();
            gp_Pnt theOldP1, theOldP2;
            double dist1, dist2, curdist1, curdist2;
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theOEIter in myOriFreeEdges)
            {
                // Iterate on original free edges
                for (theOriginalIter.Initialize(theOEIter.Value); theOriginalIter.More(); theOriginalIter.Next())
                {
                    TopoDS_Edge theOldE = TopoDS.Edge(theOriginalIter.Value());

                    // Prepare empty wire to add new edges for reshape
                    theBuilder.MakeWire(ref theNewW);

                    // Explore new edges and vertices
                    bool emptywire = true;
                    for (int i = 1; i <= 2; i++)
                    {
                        // Select list of free or shared edges
                        if (i == 1)
                        {
                            theResultsIter.Initialize(myResFreeEdges[theOldE]);
                        }
                        else
                        {
                            theResultsIter.Initialize(myResSharEdges[theOldE]);
                        }

                        // Iterate on new edges
                        for (; theResultsIter.More(); theResultsIter.Next())
                        {
                            theAuxE = theResultsIter.Value();
                            if (!theAuxE.IsSame(theOldE))
                            {
                                // Add new edge to the wire
                                theBuilder.Add(theNewW, theAuxE);
                                emptywire = false;
                            }
                        }
                    }

                    if (!emptywire)
                    {
                        // Get vertices on old and new edges
                        TopExp.Vertices(theOldE, out theOldV1, out theOldV2);
                        theOldP1 = BRep_Tool.Pnt(theOldV1);
                        theOldP2 = BRep_Tool.Pnt(theOldV2);

                        // Process vertices for replacing
                        dist1 = -1.0;
                        dist2 = -1.0;
                        for (var expv = new TopExp_Explorer(theNewW, TopAbs_VERTEX); expv.More(); expv.Next())
                        {
                            TopoDS_Vertex theNewVtx = TopoDS.Vertex(expv.Current());
                            gp_Pnt theNewPt = BRep_Tool.Pnt(theNewVtx);
                            curdist1 = theOldP1.Distance(theNewPt);
                            curdist2 = theOldP2.Distance(theNewPt);
                            if (dist1 < 0 || curdist1 < dist1)
                            {
                                dist1 = curdist1;
                                theNewV1 = theNewVtx;
                            }
                            if (dist2 < 0 || curdist2 < dist2)
                            {
                                dist2 = curdist2;
                                theNewV2 = theNewVtx;
                            }
                        }

                        // Place results in map for replacing
                        if (!theOldV1.IsSame(theNewV1))
                        {
                            if (theRepVertices.IsBound(theOldV1))
                            {
                                NCollection_List<TopoDS_Shape> theList1 = theRepVertices[theOldV1];
                                var theIter1 = new NCollection_List<TopoDS_Shape>.Iterator();
                                bool found = false;
                                for (theIter1.Initialize(theList1); theIter1.More(); theIter1.Next())
                                {
                                    if (theIter1.Value().IsSame(theNewV1))
                                    {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    theList1.Append(theNewV1);
                                }
                            }
                            else
                            {
                                var theNewList1 = new NCollection_List<TopoDS_Shape>();
                                theNewList1.Append(theNewV1);
                                theRepVertices.Bind(theOldV1, theNewList1);
                            }
                        }
                        if (!theOldV2.IsSame(theNewV2))
                        {
                            if (theRepVertices.IsBound(theOldV2))
                            {
                                NCollection_List<TopoDS_Shape> theList2 = theRepVertices[theOldV2];
                                var theIter2 = new NCollection_List<TopoDS_Shape>.Iterator();
                                bool found = false;
                                for (theIter2.Initialize(theList2); theIter2.More(); theIter2.Next())
                                {
                                    if (theIter2.Value().IsSame(theNewV2))
                                    {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    theList2.Append(theNewV2);
                                }
                            }
                            else
                            {
                                var theNewList2 = new NCollection_List<TopoDS_Shape>();
                                theNewList2.Append(theNewV2);
                                theRepVertices.Bind(theOldV2, theNewList2);
                            }
                        }

                        // Bind edge to replace
                        theRepEdges.Bind(theOldE, theNewW);
                    }
                }
            }

            if (!theRepEdges.IsEmpty())
            {
                var theReShape = new ShapeBuild_ReShape();

                // Replace edges
                foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> theREIter in theRepEdges)
                {
                    theReShape.Replace(theREIter.Key /*.Oriented(TopAbs_FORWARD)*/,
                                       theREIter.Value /*.Oriented(TopAbs_FORWARD)*/);
                }

                // smh#8
                TopoDS_Shape tmpReShape = theReShape.InternalApply(result);
                result = TopoDS.Shell(tmpReShape);
                if (theReShape.Status(ShapeExtend_OK))
                {
                    // Edges not replaced by ReShape (OCCT_DEBUG warning)
                }
                else if (theReShape.Status(ShapeExtend_FAIL1))
                {
                    // ReShape failed on edges (OCCT_DEBUG error)
                }
                else
                {
                    var SFW = new ShapeFix_Wire();
                    var SFF = new ShapeFix_Face();
                    var SAE = new ShapeAnalysis_Edge();
                    ShapeExtend_WireData sewd;

                    // Perform necessary fixes on subshapes
                    // smh#8
                    TopoDS_Shape emptyCopiedShell = result.EmptyCopied();
                    TopoDS_Shell theShell = TopoDS.Shell(emptyCopiedShell);
                    for (var itf1 = new TopoDS_Iterator(result); itf1.More(); itf1.Next())
                    {
                        TopoDS_Face newface = TopoDS.Face(itf1.Value());
                        // smh#8
                        TopoDS_Shape emptyCopiedFace = newface.EmptyCopied();
                        TopoDS_Face EmpFace = TopoDS.Face(emptyCopiedFace);
                        for (var itw = new TopoDS_Iterator(newface); itw.More(); itw.Next())
                        {
                            if (itw.Value().ShapeType() != TopAbs_WIRE)
                            {
                                continue;
                            }
                            TopoDS_Wire theWire = TopoDS.Wire(itw.Value());
                            sewd = new ShapeExtend_WireData(theWire);
                            var SAWO = new ShapeAnalysis_WireOrder(false, 0);
                            for (int i = 1; i <= sewd.NbEdges(); i++)
                            {
                                // smh#8
                                TopoDS_Shape tmpFace = EmpFace.Oriented(TopAbs_FORWARD);
                                if (!SAE.PCurve(sewd.Edge(i), TopoDS.Face(tmpFace), out Geom2d_Curve? c2d, out double f, out double l))
                                {
                                    continue;
                                }
                                SAWO.Add(c2d!.Value(f).XY(), c2d.Value(l).XY());
                            }
                            SAWO.Perform();
                            SFW.Load(sewd);
                            SFW.FixReorder(SAWO);
                            SFW.FixReorder();
                            SFW.SetFace(EmpFace);
                            SFW.SetPrecision(fixtoler);
                            SFW.SetMaxTolerance(sewtoler);
                            SFW.FixEdgeCurves();
                            SFW.FixSelfIntersection();
                            theWire = SFW.InternalWire(); // OCCT.NET: the wire of the shell being built
                            theBuilder.Add(EmpFace, theWire);
                        }

                        // #ifdef AIX  CKY : applies to all platforms
                        SFF.Init(EmpFace);
                        //	  SFF->Init(TopoDS::Face(EmpFace));

                        var MapWires = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
                        MapWires.Clear();
                        if (SFF.FixOrientation(MapWires))
                        {
                            EmpFace = SFF.InternalFace(); // OCCT.NET: the face of the shell being built
                        }
                        theBuilder.Add(theShell, EmpFace);
                    }
                    theShell.Closed(BRep_Tool.IsClosed(theShell));
                    result = theShell;

                    if (!theRepVertices.IsEmpty())
                    {
                        // Prepare vertices to replace
                        TopoDS_Shape theOld, theNew, theRep, theAux;
                        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theRV1Iter in theRepVertices)
                        {
                            // Get the old vertex, create empty list of replaced vertices
                            theOld = theRV1Iter.Key;
                            var theNewList = new NCollection_List<TopoDS_Shape>();
                            // Explore the list of new vertices
                            var theN1Iter = new NCollection_List<TopoDS_Shape>.Iterator();
                            for (theN1Iter.Initialize(theRV1Iter.Value); theN1Iter.More(); theN1Iter.Next())
                            {
                                theNew = theN1Iter.Value();
                                if (theOldVertices.IsBound(theNew))
                                {
                                    // Vertex has a replacing vertex in the map
                                    theRep = theOldVertices[theNew];
                                    if (!theRep.IsSame(theOld))
                                    {
                                        // Vertex is not in current list
                                        theOldVertices.Bind(theRep, theOld);
                                        theNewList.Append(theRep);
                                        var theN3Iter = new NCollection_List<TopoDS_Shape>.Iterator();
                                        for (theN3Iter.Initialize(theNewVertices[theRep]); theN3Iter.More(); theN3Iter.Next())
                                        {
                                            theAux = theN3Iter.Value();
                                            theOldVertices[theAux] = theOld;
                                            theNewList.Append(theAux);
                                        }
                                        theNewVertices.UnBind(theRep);
                                    }
                                }
                                else
                                {
                                    theOldVertices.Bind(theNew, theOld);
                                    theNewList.Append(theNew);
                                }
                            }
                            theNewVertices.Bind(theOld, theNewList);
                        }

                        // Update vertices positions and tolerances
                        TopoDS_Vertex theNewVert, theOldVert;
                        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theRV2Iter in theNewVertices)
                        {
                            theNewVert = TopoDS.Vertex(theRV2Iter.Key);
                            // Calculate the vertex position
                            gp_Pnt theLBound, theRBound, thePosition;
                            theLBound = theRBound = BRep_Tool.Pnt(theNewVert);
                            var theN2Iter = new NCollection_List<TopoDS_Shape>.Iterator();
                            for (theN2Iter.Initialize(theRV2Iter.Value); theN2Iter.More(); theN2Iter.Next())
                            {
                                thePosition = BRep_Tool.Pnt(TopoDS.Vertex(theN2Iter.Value()));
                                double val = thePosition.X();
                                if (val < theLBound.X())
                                {
                                    theLBound.SetX(val);
                                }
                                else if (val > theRBound.X())
                                {
                                    theRBound.SetX(val);
                                }
                                val = thePosition.Y();
                                if (val < theLBound.Y())
                                {
                                    theLBound.SetY(val);
                                }
                                else if (val > theRBound.Y())
                                {
                                    theRBound.SetY(val);
                                }
                                val = thePosition.Z();
                                if (val < theLBound.Z())
                                {
                                    theLBound.SetZ(val);
                                }
                                else if (val > theRBound.Z())
                                {
                                    theRBound.SetZ(val);
                                }
                            }
                            thePosition = new gp_Pnt((theLBound.XYZ() + theRBound.XYZ()) / 2.0);
                            double theTolerance = 0.0, curtoler;
                            // Calculate the vertex tolerance
                            for (theN2Iter.Initialize(theRV2Iter.Value); theN2Iter.More(); theN2Iter.Next())
                            {
                                theOldVert = TopoDS.Vertex(theN2Iter.Value());
                                curtoler = thePosition.Distance(BRep_Tool.Pnt(theOldVert)) + BRep_Tool.Tolerance(theOldVert);
                                if (curtoler > theTolerance)
                                {
                                    theTolerance = curtoler;
                                }
                            }
                            curtoler = thePosition.Distance(BRep_Tool.Pnt(theNewVert)) + BRep_Tool.Tolerance(theNewVert);
                            if (curtoler > theTolerance)
                            {
                                theTolerance = curtoler;
                            }
                            theBuilder.UpdateVertex(theNewVert, thePosition, theTolerance);
                        }

                        // Replace vertices
                        theReShape.Clear();
                        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> theNVIter in theOldVertices)
                        {
                            theReShape.Replace(theNVIter.Key.Oriented(TopAbs_FORWARD),
                                               theNVIter.Value.Oriented(TopAbs_FORWARD));
                        }
                        // smh#8
                        TopoDS_Shape tmpshape = theReShape.InternalApply(result);
                        result = TopoDS.Shell(tmpshape);
                        if (theReShape.Status(ShapeExtend_FAIL1))
                        {
                            // ReShape failed on vertices (OCCT_DEBUG error)
                        }
                    }
                }
            }
        }

        return result;
    }

    /// <summary>Clears the recorded connections and the internal data.</summary>
    public void Clear()
    {
        myConnected.Clear();
        myOriFreeEdges.Clear();
        myResFreeEdges.Clear();
        myResSharEdges.Clear();
    }
}
