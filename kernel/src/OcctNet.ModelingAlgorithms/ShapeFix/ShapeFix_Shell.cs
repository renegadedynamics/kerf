// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// pdn 17.12.98 ie_exhaust-A.stp

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Fixes a shell: its faces (by ShapeFix_Face), the orientation of its faces (a shell whose faces can't be oriented
/// consistently is split into several shells), and its closed flag.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> only records the shell. <see cref="Perform"/> and
/// <see cref="FixFaceOrientation"/> edit the shell they work on in place, as OCCT's: a shell holding frozen shapes (any
/// shell a caller holds) is copied first (<see cref="BRepTools_CopyAtEntry"/>, decision 583) and the caller's shell
/// never changes. At the end of the fix the copies left unchanged are the caller's own again, and the context answers
/// for the caller's sub-shapes. <see cref="Shell"/>, <see cref="Shape"/> and <see cref="ErrorFaces"/> freeze what they
/// return; ported code that goes on building reads the internal forms.
/// </para>
/// </remarks>
public class ShapeFix_Shell : ShapeFix_Root
{
    /// <summary>The shell fixed (the first of the shells made).</summary>
    protected TopoDS_Shell    myShell;
    /// <summary>The faces that couldn't be oriented (Moebius-like).</summary>
    protected TopoDS_Compound myErrFaces;
    /// <summary>The status.</summary>
    protected int             myStatus;
    /// <summary>The face fixing tool.</summary>
    protected ShapeFix_Face   myFixFace;
    /// <summary>The mode of the face fixes.</summary>
    protected int             myFixFaceMode;
    /// <summary>The mode of the orientation fix.</summary>
    protected int             myFixOrientationMode;
    /// <summary>The number of shells made.</summary>
    protected int             myNbShells;
    /// <summary>Whether a non-manifold shell may be made.</summary>
    protected bool            myNonManifold;

    // Type aliases for DataMaps with shape hasher (OCCT's FaceEdgesMap is keyed by std::hash, compared with IsEqual)
    private const int DEFAULT_EDGE_FACES_INCREMENT = 5;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_Shell()
    {
        myShell              = new TopoDS_Shell();
        myErrFaces           = new TopoDS_Compound();
        myStatus             = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixFaceMode        = -1;
        myFixOrientationMode = -1;
        myFixFace            = new ShapeFix_Face();
        myNbShells           = 0;
        myNonManifold        = false;
    }

    /// <summary>Initializes by a shell.</summary>
    public ShapeFix_Shell(in TopoDS_Shell shape)
    {
        myShell              = new TopoDS_Shell();
        myErrFaces           = new TopoDS_Compound();
        myStatus             = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixFaceMode        = -1;
        myFixOrientationMode = -1;
        myFixFace            = new ShapeFix_Face();
        Init(shape);
        myNonManifold = false;
    }

    /// <summary>Initializes by a shell.</summary>
    public void Init(in TopoDS_Shell shell)
    {
        myShape    = shell;
        myShell    = shell;
        myNbShells = 0;
    }

    /// <summary>
    /// Fixes the faces of the shell (if the face mode allows) and the orientation of the faces (if the orientation
    /// mode allows), and corrects the closed flag of the shells. Returns true if something was done.
    /// </summary>
    public bool Perform(ProgressRange theProgress = default)
    {
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        var anEntry = new BRepTools_CopyAtEntry(myShell, Context());
        myShell = TopoDS.Shell(anEntry.Shape());
        myShape = myShell;
        bool status = perform(theProgress);
        finish(anEntry);
        return status;
    }

    // ShapeFix_Shell::Perform on the working shell
    private bool perform(ProgressRange theProgress)
    {
        bool status = false;
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        ShapeBuild_ReShape aContext = Context()!;
        myFixFace.SetContext(aContext);

        if (NeedFix(myFixFaceMode))
        {
            TopoDS_Shape S = aContext.InternalApply(myShell);

            // Get the number of faces for progress indication
            int aNbFaces = S.NbChildren();

            // Start progress scope (no need to check if progress exists -- it is safe)
            using var aPS = new ProgressScope(theProgress, "Fixing face", aNbFaces);

            for (var iter = new TopoDS_Iterator(S); iter.More() && aPS.More(); iter.Next(), aPS.Next())
            {
                TopoDS_Shape sh      = iter.Value();
                TopoDS_Face  tmpFace = TopoDS.Face(sh);
                myFixFace.Init(tmpFace);
                if (myFixFace.Perform())
                {
                    status = true;
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
            }

            // Halt algorithm in case of user's abort
            if (!aPS.More())
            {
                return false;
            }
        }

        TopoDS_Shape newsh = aContext.InternalApply(myShell);
        if (NeedFix(myFixOrientationMode))
        {
            fixFaceOrientation(TopoDS.Shell(newsh), true, myNonManifold);
        }

        TopoDS_Shape aNewsh = aContext.InternalApply(newsh);
        var          aSas   = new ShapeAnalysis_Shell();
        for (var aShellExp = new TopExp_Explorer(aNewsh, TopAbs_SHELL); aShellExp.More(); aShellExp.Next())
        {
            TopoDS_Shell aCurShell = TopoDS.Shell(aShellExp.Current());
            if (aCurShell.Closed())
            {
                aSas.LoadShells(aCurShell);
                aSas.CheckOrientedShells(aCurShell, true);
                if (aSas.HasFreeEdges())
                {
                    aCurShell.Closed(false);
                    SendWarning(new Msg("FixAdvShell.FixClosedFlag.MSG0")); // Shell has incorrect flag isClosed
                }
                aSas.Clear();
            }
        }

        if (status)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        if (Status(ShapeExtend_DONE2))
        {
            status = true;
        }
        return status;
    }

    //=================================================================================================

    private static bool GetFreeEdges(in TopoDS_Shape aShape, NCollection_Map<TopoDS_Shape> MapEdges)
    {
        for (var aExpF = new TopExp_Explorer(aShape, TopAbs_FACE); aExpF.More(); aExpF.Next())
        {
            for (var aExpE = new TopExp_Explorer(aExpF.Current(), TopAbs_EDGE); aExpE.More(); aExpE.Next())
            {
                TopoDS_Edge edge = TopoDS.Edge(aExpE.Current());
                if (!MapEdges.Contains(edge))
                {
                    MapEdges.Add(edge);
                }
                else
                {
                    MapEdges.Remove(edge);
                }
            }
        }
        return !MapEdges.IsEmpty();
    }

    // OCCT's FaceEdgesMap: NCollection_IndexedDataMap<TopoDS_Face, NCollection_Array1<TopoDS_Edge>> with the default
    // hasher (IsEqual); EdgeFacesMap: a DataMap of the edges' faces with the shape hasher.
    private static NCollection_IndexedDataMap<TopoDS_Shape, NCollection_Array1<TopoDS_Shape>> NewFaceEdgesMap() =>
        new(EqualityComparer<TopoDS_Shape>.Default);

    /// Groups connected faces into separate sequences using existing connectivity data.
    /// Uses depth-first search to find connected components through shared edges.
    /// Each face appears in exactly one group, ensuring no duplicates across groups.
    /// Groups are sorted by size with the largest group first.
    private static NCollection_List<NCollection_Sequence<TopoDS_Shape>> GetConnectedFaceGroups(
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_Array1<TopoDS_Shape>> theFaceEdges,
        NCollection_DataMap<TopoDS_Shape, NCollection_DynamicArray<TopoDS_Shape>> theEdgeFaces)
    {
        var aConnectedGroups = new NCollection_List<NCollection_Sequence<TopoDS_Shape>>();

        if (theFaceEdges.IsEmpty())
        {
            return aConnectedGroups;
        }

        var aVisitedFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        for (int aFaceIndex = 1; aFaceIndex <= theFaceEdges.Extent(); aFaceIndex++)
        {
            TopoDS_Shape aStartFace = theFaceEdges.FindKey(aFaceIndex);

            if (aVisitedFaces.Contains(aStartFace))
            {
                continue;
            }

            // Start new connected group
            var aConnectedGroup = new NCollection_Sequence<TopoDS_Shape>();

            // DFS traversal
            var aStack = new Stack<TopoDS_Shape>();
            aStack.Push(aStartFace);
            aVisitedFaces.Add(aStartFace);

            while (aStack.Count != 0)
            {
                TopoDS_Shape aCurrentFace = aStack.Pop();
                aConnectedGroup.Append(aCurrentFace);

                // Find connected faces through shared edges
                if (theFaceEdges.FindFromKey(aCurrentFace, out NCollection_Array1<TopoDS_Shape> aFaceEdgesArray))
                {
                    for (int anEdgeIdx = aFaceEdgesArray.Lower(); anEdgeIdx <= aFaceEdgesArray.Upper(); ++anEdgeIdx)
                    {
                        TopoDS_Shape anEdge = aFaceEdgesArray.Value(anEdgeIdx);

                        if (theEdgeFaces.Find(anEdge, out NCollection_DynamicArray<TopoDS_Shape> aConnectedFaces))
                        {
                            for (int aFaceIdx = 0; aFaceIdx < aConnectedFaces.Length(); ++aFaceIdx)
                            {
                                TopoDS_Shape aNeighborFace = aConnectedFaces.Value(aFaceIdx);

                                if (!aVisitedFaces.Contains(aNeighborFace))
                                {
                                    aVisitedFaces.Add(aNeighborFace);
                                    aStack.Push(aNeighborFace);
                                }
                            }
                        }
                    }
                }
            }

            // Insert in sorted order (largest groups first)
            bool anIsInserted = false;

            for (var anIter = new NCollection_List<NCollection_Sequence<TopoDS_Shape>>.Iterator(aConnectedGroups);
                 anIter.More();
                 anIter.Next())
            {
                if (aConnectedGroup.Length() > anIter.Value().Length())
                {
                    aConnectedGroups.InsertBefore(aConnectedGroup, anIter);
                    anIsInserted = true;
                    break;
                }
            }

            if (!anIsInserted)
            {
                aConnectedGroups.Append(aConnectedGroup);
            }
        }

        return aConnectedGroups;
    }

    /// Creates shells from connected face groups using connectivity analysis.
    /// Processes only the largest connected group for shell construction, improving efficiency
    /// by focusing on faces that are actually topologically connected.
    private static bool GetShells(NCollection_Sequence<TopoDS_Shape> theLfaces,
                                  NCollection_Map<TopoDS_Shape> theMapMultiConnectEdges,
                                  NCollection_Sequence<TopoDS_Shape> theSeqShells,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMapFaceShells,
                                  NCollection_Sequence<TopoDS_Shape> theErrFaces)
    {
        bool aDone = false;
        if (theLfaces.Length() == 0)
        {
            return false;
        }
        var nshell = new TopoDS_Shell();
        var B      = new BRep_Builder();
        B.MakeShell(ref nshell);
        bool anIsMultiConnex    = !theMapMultiConnectEdges.IsEmpty();
        int  aFaceIdx           = 1, aFacesInShellCount = 1;
        var  aSeqUnconnectFaces = new NCollection_Sequence<TopoDS_Shape>();

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_Array1<TopoDS_Shape>> aFaceEdges = NewFaceEdgesMap();
        int aNumberOfEdges = 0;
        var aTempEdges     = new NCollection_DynamicArray<TopoDS_Shape>();
        for (int aLfaceIdx = 1; aLfaceIdx <= theLfaces.Length(); aLfaceIdx++)
        {
            aTempEdges.Clear();
            TopoDS_Face aFace = TopoDS.Face(theLfaces.Value(aLfaceIdx));
            for (var anEdgeExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
            {
                aTempEdges.Append(TopoDS.Edge(anEdgeExp.Current()));
                aNumberOfEdges++;
            }
            var aFaceEdgesArray = new NCollection_Array1<TopoDS_Shape>(1, aTempEdges.Length());
            for (int idx = 0; idx < aTempEdges.Length(); ++idx)
            {
                aFaceEdgesArray.SetValue(idx + 1, aTempEdges.Value(idx));
            }
            aFaceEdges.Add(aFace, aFaceEdgesArray);
        }

        var aEdgeFaces = new NCollection_DataMap<TopoDS_Shape, NCollection_DynamicArray<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

        for (int aFaceInd = 1; aFaceInd <= aFaceEdges.Extent(); ++aFaceInd)
        {
            TopoDS_Shape                       aFace           = aFaceEdges.FindKey(aFaceInd);
            NCollection_Array1<TopoDS_Shape> aFaceEdgesArray = aFaceEdges.FindFromIndex(aFaceInd);

            for (int anEdgeInd = aFaceEdgesArray.Lower(); anEdgeInd <= aFaceEdgesArray.Upper(); ++anEdgeInd)
            {
                TopoDS_Shape anEdge = aFaceEdgesArray.Value(anEdgeInd);

                if (!aEdgeFaces.IsBound(anEdge))
                {
                    aEdgeFaces.Bind(anEdge, new NCollection_DynamicArray<TopoDS_Shape>());
                }
                NCollection_DynamicArray<TopoDS_Shape> aFacesArray = aEdgeFaces.Find(anEdge);

                // Check if face already exists in the array
                bool aFaceExists = false;
                for (int aFaceCheckIdx = 0; aFaceCheckIdx < aFacesArray.Length(); ++aFaceCheckIdx)
                {
                    if (aFacesArray.Value(aFaceCheckIdx).IsSame(aFace))
                    {
                        aFaceExists = true;
                        break;
                    }
                }

                // (OCCT sets the array's increment to DEFAULT_EDGE_FACES_INCREMENT here: an allocation detail)
                _ = DEFAULT_EDGE_FACES_INCREMENT;

                if (!aFaceExists)
                {
                    aFacesArray.Append(aFace);
                }
            }
        }

        // Get connected groups of faces using existing connectivity data
        NCollection_List<NCollection_Sequence<TopoDS_Shape>> aConnectedGroups = GetConnectedFaceGroups(aFaceEdges, aEdgeFaces);

        // Process only the largest connected group for shell construction
        if (aConnectedGroups.IsEmpty())
        {
            return false;
        }

        // Some assumption that each edge can be in two orientations
        var aProcessedEdges = new NCollection_DataMap<TopoDS_Shape, (bool first, bool second)>(TopTools_ShapeMapHasher.Instance);

        NCollection_Sequence<TopoDS_Shape> aProcessingFaces = aConnectedGroups.First();

        var aTempProcessedEdges = new NCollection_DataMap<TopoDS_Shape, (bool first, bool second)>(TopTools_ShapeMapHasher.Instance);
        for (; aFaceIdx <= aProcessingFaces.Length(); aFaceIdx++)
        {
            aTempProcessedEdges.Clear();

            int         aBadOrientationCount = 0, aGoodOrientationCount = 0;
            TopoDS_Face F1                   = TopoDS.Face(aProcessingFaces.Value(aFaceIdx));
            // Get edges of the face
            NCollection_Array1<TopoDS_Shape> aFaceEdgesArray = aFaceEdges.FindFromKey(F1);

            for (int anEdgeInd = aFaceEdgesArray.Lower(); anEdgeInd <= aFaceEdgesArray.Upper(); ++anEdgeInd)
            {
                TopoDS_Shape edge = aFaceEdgesArray.Value(anEdgeInd);

                // if multiconnexity mode is equal to true faces contains
                // the same multiconnexity edges are not added to one shell.
                if (anIsMultiConnex && theMapMultiConnectEdges.Contains(edge))
                {
                    continue;
                }

                if (!aProcessedEdges.IsBound(edge))
                {
                    if (!aTempProcessedEdges.IsBound(edge))
                    {
                        (bool, bool) anEdgeOrientationPair = (edge.Orientation() == TopAbs_FORWARD,
                                                              edge.Orientation() == TopAbs_REVERSED);

                        aTempProcessedEdges.Bind(edge, anEdgeOrientationPair);
                    }
                    else
                    {
                        ref (bool first, bool second) aTempProcessedEdgeIt = ref aTempProcessedEdges.ChangeFind(edge);
                        aTempProcessedEdgeIt.first  = aTempProcessedEdgeIt.first || (edge.Orientation() == TopAbs_FORWARD);
                        aTempProcessedEdgeIt.second = aTempProcessedEdgeIt.second || (edge.Orientation() == TopAbs_REVERSED);
                    }
                    continue;
                }

                ref (bool first, bool second) aPair = ref aProcessedEdges.ChangeFind(edge);

                bool isDirect   = aPair.first;
                bool isReversed = aPair.second;

                if ((edge.Orientation() == TopAbs_FORWARD && isDirect)
                    || (edge.Orientation() == TopAbs_REVERSED && isReversed))
                {
                    aBadOrientationCount++;
                }
                else if ((edge.Orientation() == TopAbs_FORWARD && isReversed)
                         || (edge.Orientation() == TopAbs_REVERSED && isDirect))
                {
                    aGoodOrientationCount++;
                }

                if (isDirect)
                {
                    aPair.first = false;
                }
                else if (isReversed)
                {
                    aPair.second = false;
                }

                if (!aPair.first && !aPair.second)
                {
                    // if edge is processed in this face it is removed from map of processed edges
                    aProcessedEdges.UnBind(edge);
                }
            }

            if (aBadOrientationCount == 0 && aGoodOrientationCount == 0 && aTempProcessedEdges.IsEmpty())
            {
                continue;
            }

            // if face can not be added to shell it added to sequence of error faces.

            if (aGoodOrientationCount != 0 && aBadOrientationCount != 0)
            {
                theErrFaces.Append(F1);
                aProcessingFaces.Remove(aFaceIdx);
                aFacesInShellCount++;
                continue;
            }

            // Addition of face to shell. In the dependance of orientation faces in the shell
            //  added face can be reversed.

            if ((aGoodOrientationCount != 0 || aBadOrientationCount != 0) || aFacesInShellCount == 1)
            {
                if (aBadOrientationCount != 0)
                {
                    F1.Reverse();

                    foreach (KeyValuePair<TopoDS_Shape, (bool first, bool second)> aTempEdgeIter in aTempProcessedEdges)
                    {
                        TopoDS_Shape                  edge                  = aTempEdgeIter.Key;
                        (bool first, bool second)     anEdgeOrientationPair = aTempEdgeIter.Value;

                        (bool, bool) aRevertedPair = (!anEdgeOrientationPair.first, !anEdgeOrientationPair.second);

                        if (!aProcessedEdges.IsBound(edge))
                        {
                            aProcessedEdges.Bind(edge, aRevertedPair);
                        }
                        else
                        {
                            aProcessedEdges.ChangeFind(edge) = aRevertedPair;
                        }
                    }
                    aDone = true;
                }
                else
                {
                    foreach (KeyValuePair<TopoDS_Shape, (bool first, bool second)> aTempEdgeIter in aTempProcessedEdges)
                    {
                        TopoDS_Shape              edge                  = aTempEdgeIter.Key;
                        (bool first, bool second) anEdgeOrientationPair = aTempEdgeIter.Value;

                        if (!aProcessedEdges.IsBound(edge))
                        {
                            aProcessedEdges.Bind(edge, anEdgeOrientationPair);
                        }
                        else
                        {
                            aProcessedEdges.ChangeFind(edge) = anEdgeOrientationPair;
                        }
                    }
                }
                aFacesInShellCount++;
                B.Add(nshell, F1);
                theMapFaceShells.Bind(F1, nshell);
                aProcessingFaces.Remove(aFaceIdx);

                // check if closed shell is obtained in multi connex mode and add to sequence of
                // shells and new shell begin to construct.
                // (check is n*2)
                if (anIsMultiConnex && BRep_Tool.IsClosed(nshell))
                {
                    nshell.Closed(true);
                    theSeqShells.Append(nshell);
                    var nshellnext = new TopoDS_Shell();
                    B.MakeShell(ref nshellnext);
                    nshell             = nshellnext;
                    aFacesInShellCount = 1;
                }

                aFaceIdx = 0;
            }
            // if shell contains of one face. This face is added to sequence of faces.
            //  This shell is removed.
            if (aProcessingFaces.Length() != 0 && aFaceIdx == aProcessingFaces.Length() && aFacesInShellCount <= 2)
            {
                var aItf = new TopoDS_Iterator(nshell, false);
                if (aItf.More())
                {
                    aSeqUnconnectFaces.Append(aItf.Value());
                    theMapFaceShells.UnBind(aItf.Value());
                }
                var nshellnext = new TopoDS_Shell();
                B.MakeShell(ref nshellnext);
                nshell             = nshellnext;
                aFaceIdx           = 0;
                aFacesInShellCount = 1;
            }
        }
        bool isContains = false;
        for (int k = 1; k <= theSeqShells.Length() && !isContains; k++)
        {
            isContains = nshell.IsSame(theSeqShells.Value(k));
        }
        if (!isContains)
        {
            int          numFace = 0;
            TopoDS_Shape aFace   = new();
            for (var aItf = new TopoDS_Iterator(nshell, false); aItf.More(); aItf.Next())
            {
                aFace = aItf.Value();
                numFace++;
            }
            if (numFace > 1)
            {
                // close all closed shells in no multi connex mode
                if (!anIsMultiConnex)
                {
                    nshell.Closed(BRep_Tool.IsClosed(nshell));
                }
                theSeqShells.Append(nshell);
            }
            else if (numFace == 1)
            {
                if (theMapFaceShells.IsBound(aFace))
                {
                    theMapFaceShells.UnBind(aFace);
                }
                aProcessingFaces.Append(aFace);
            }
        }

        // Add all unprocessed connected groups (second group and after) to unconnected faces
        int aGroupIndex = 1;
        foreach (NCollection_Sequence<TopoDS_Shape> aUnprocessedGroup in aConnectedGroups)
        {
            if (aGroupIndex++ == 1)
            {
                continue; // Skip first group (already processed)
            }

            for (int anUnprocFaceIdx = 1; anUnprocFaceIdx <= aUnprocessedGroup.Length(); ++anUnprocFaceIdx)
            {
                aSeqUnconnectFaces.Append(aUnprocessedGroup.Value(anUnprocFaceIdx));
            }
        }

        theLfaces.Assign(aProcessingFaces);

        // Add unconnected faces from the largest group that couldn't be added to shells
        for (int j1 = 1; j1 <= aSeqUnconnectFaces.Length(); j1++)
        {
            theLfaces.Append(aSeqUnconnectFaces.Value(j1));
        }

        return aDone;
    }

    //=======================================================================
    // function : AddMultiConexityFaces
    // purpose  : In this function faces have only of multiconnexity boundary
    //            are added to shells having free boundary contains the same
    //            multiconnexity edges as faces.
    //=======================================================================
    private static bool AddMultiConexityFaces(NCollection_Sequence<TopoDS_Shape> Lface,
                                              NCollection_Map<TopoDS_Shape> aMapMultiConnectEdges,
                                              NCollection_Sequence<TopoDS_Shape> SeqShells,
                                              NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aMapFaceShells,
                                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapEdgeFaces,
                                              NCollection_Sequence<TopoDS_Shape> ErrFaces,
                                              bool NonManifold)
    {
        bool done = false;
        //  BRep_Builder aB;
        var llPosibleShells = new NCollection_Sequence<TopoDS_Shape>();
        var AddShapes       = new NCollection_Sequence<TopoDS_Shape>();
        for (int i1 = 1; i1 <= Lface.Length(); i1++)
        {
            TopoDS_Shape aShape = Lface.Value(i1);

            int aNbMultEdges = 0;

            // Finds faces having only multiconnexity boundary.
            for (var aItWires = new TopoDS_Iterator(aShape, false); aItWires.More(); aItWires.Next())
            {
                int aNbEdges = 0;
                for (var aItEdges = new TopoDS_Iterator(aItWires.Value(), false); aItEdges.More(); aItEdges.Next(), aNbEdges++)
                {
                    TopoDS_Shape edge = aItEdges.Value();
                    if (!aMapMultiConnectEdges.Contains(edge))
                    {
                        continue;
                    }
                    aNbMultEdges++;
                }
                if (aNbMultEdges == 0)
                {
                    continue;
                }

                if (aNbMultEdges == aNbEdges)
                {
                    AddShapes.Append(aShape);
                }
                else
                {
                    llPosibleShells.Append(aShape);
                }
            }
        }

        // Attempt to create shell from unconnected which have not only multiconnexity boundary.
        var aTmpShells = new NCollection_Sequence<TopoDS_Shape>();
        if (!llPosibleShells.IsEmpty())
        {
            var aMap          = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var aTmp          = new NCollection_Sequence<TopoDS_Shape>();
            var aTmpFaceShell = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            if (GetShells(llPosibleShells, aMap, aTmpShells, aTmpFaceShell, aTmp))
            {
                for (int kk = 1; kk <= aTmpShells.Length(); kk++)
                {
                    TopoDS_Shape aSh      = aTmpShells.Value(kk);
                    var          mapEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    if (GetFreeEdges(aSh, mapEdges))
                    {
                        int nbedge = 0;
                        foreach (TopoDS_Shape aKey in mapEdges)
                        {
                            if (aMapMultiConnectEdges.Contains(aKey))
                            {
                                nbedge++;
                            }
                        }
                        if (nbedge != 0 && nbedge == mapEdges.Extent())
                        {
                            AddShapes.Append(aSh);
                        }
                    }
                }
            }
        }

        // Add chosen faces to shells.
        for (int k1 = 1; k1 <= AddShapes.Length(); k1++)
        {
            var          MapOtherShells = new NCollection_DataMap<TopoDS_Shape, int>(TopTools_ShapeMapHasher.Instance);
            var          dire           = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var          reve           = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopoDS_Shape aSh            = AddShapes.Value(k1);
            var          mapEdges       = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            if (!GetFreeEdges(aSh, mapEdges))
            {
                continue;
            }
            var lfaces = new NCollection_List<TopoDS_Shape>();

            // Fill MapOtherShells which will contain shells with orientation in which selected shape aSh
            // will be add.

            foreach (TopoDS_Shape aKey in mapEdges)
            {
                if (!aMapMultiConnectEdges.Contains(aKey))
                {
                    continue;
                }
                TopoDS_Edge edge = TopoDS.Edge(aKey);
                if (edge.Orientation() == TopAbs_FORWARD)
                {
                    dire.Add(edge);
                }
                else
                {
                    reve.Add(edge);
                }
                NCollection_List<TopoDS_Shape> lf = aMapEdgeFaces.FindFromKey(edge);
                lfaces.Append(new NCollection_List<TopoDS_Shape>(lf));
            }
            foreach (TopoDS_Shape aF in lfaces)
            {
                if (!aMapFaceShells.IsBound(aF))
                {
                    continue;
                }

                TopoDS_Shape aOthershell = aMapFaceShells.Find(aF);
                if (MapOtherShells.IsBound(aOthershell))
                {
                    continue;
                }
                if (!NonManifold && BRep_Tool.IsClosed(aOthershell))
                {
                    continue;
                }

                var mapShellEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                GetFreeEdges(aOthershell, mapShellEdges);
                bool isAdd = true;
                foreach (TopoDS_Shape aKey1 in mapEdges)
                {
                    if (!isAdd)
                    {
                        break;
                    }
                    isAdd = mapShellEdges.Contains(aKey1);
                }

                if (!isAdd)
                {
                    continue;
                }
                int nbdir = 0, nbrev = 0;

                // add only free face whome all edges contains in the shell as open boundary.
                foreach (TopoDS_Shape aKeyS in mapShellEdges)
                {
                    TopoDS_Edge edgeS = TopoDS.Edge(aKeyS);
                    if (!aMapMultiConnectEdges.Contains(edgeS))
                    {
                        continue;
                    }
                    if ((edgeS.Orientation() == TopAbs_FORWARD && dire.Contains(edgeS))
                        || (edgeS.Orientation() == TopAbs_REVERSED && reve.Contains(edgeS)))
                    {
                        nbrev++;
                    }
                    else if ((edgeS.Orientation() == TopAbs_FORWARD && reve.Contains(edgeS))
                             || (edgeS.Orientation() == TopAbs_REVERSED && dire.Contains(edgeS)))
                    {
                        nbdir++;
                    }
                }
                if (nbdir != 0 && nbrev != 0)
                {
                    ErrFaces.Append(aSh);
                    continue;
                }
                if (nbdir != 0 || nbrev != 0)
                {
                    int isReverse = (nbrev != 0 ? 1 : 0);
                    MapOtherShells.Bind(aOthershell, isReverse);
                }
            }
            if (MapOtherShells.IsEmpty())
            {
                //      i1++;
                continue;
            }

            // Adds face to open shells containing the same multishared edges.
            // For nonmanifold mode creation ine shell from face and shells containing the same multishared
            // edges.
            //  If one face can be added to a few shells (case of compsolid) face will be added to each
            //  shell.
            done = true;
            int FirstRev = 0, FirstInd = 0;
            int ind      = 0;
            for (int l = 1; l <= SeqShells.Length(); l++)
            {
                if (!MapOtherShells.IsBound(SeqShells.Value(l)))
                {
                    continue;
                }
                ind++;
                int          isRev     = MapOtherShells.Find(SeqShells.Value(l));
                TopoDS_Shape anewShape = (isRev != 0 ? aSh.Reversed() : aSh);

                var          aB1    = new BRep_Builder();
                TopoDS_Shape aShell = SeqShells.Value(l);
                if (ind == 1 || !NonManifold)
                {
                    if (ind == 1)
                    {
                        FirstRev = isRev;
                        FirstInd = l;
                    }
                    for (var aE = new TopExp_Explorer(anewShape, TopAbs_FACE); aE.More(); aE.Next())
                    {
                        aB1.Add(aShell, aE.Current());
                    }
                    SeqShells.ChangeValue(l) = aShell;
                }
                else if (NonManifold)
                {
                    bool isReversed = ((isRev != 0) || FirstRev != 0) && (!(isRev != 0) || FirstRev == 0);
                    aShell          = SeqShells.Value(FirstInd);
                    for (var aItF = new TopoDS_Iterator(SeqShells.Value(l), false); aItF.More(); aItF.Next())
                    {
                        TopoDS_Shape nF = (isReversed ? aItF.Value().Reversed() : aItF.Value());
                        aB1.Add(aShell, nF);
                    }
                    SeqShells.ChangeValue(FirstInd) = aShell;
                    SeqShells.Remove(l--);
                }
            }

            dire.Clear();
            reve.Clear();
            for (var aEt = new TopExp_Explorer(aSh, TopAbs_FACE); aEt.More(); aEt.Next())
            {
                for (int kk = 1; kk <= Lface.Length(); kk++)
                {
                    if (aEt.Current().IsSame(Lface.Value(kk)))
                    {
                        Lface.Remove(kk--);
                    }
                }
            }
        }
        return done;
    }

    //=======================================================================
    // function : BoxIn
    // purpose  : Check if one face contains inside other.
    //=======================================================================
    private static int BoxIn(in Box theBox1, in Box theBox2)
    {
        int aNumIn = 0;
        theBox1.Get(out double aXmin1, out double aYmin1, out double aZmin1, out double aXmax1, out double aYmax1, out double aZmax1);
        theBox2.Get(out double aXmin2, out double aYmin2, out double aZmin2, out double aXmax2, out double aYmax2, out double aZmax2);
        if (aXmin1 == aXmin2 && aXmax1 == aXmax2 && aYmin1 == aYmin2 && aYmax1 == aYmax2
            && aZmin1 == aZmin2 && aZmax1 == aZmax2)
        {
            aNumIn = 0;
        }
        else if (aXmin1 >= aXmin2 && aXmax1 <= aXmax2 && aYmin1 >= aYmin2 && aYmax1 <= aYmax2
                 && aZmin1 >= aZmin2 && aZmax1 <= aZmax2)
        {
            aNumIn = 1;
        }
        else if (aXmin1 <= aXmin2 && aXmax1 >= aXmax2 && aYmin1 <= aYmin2 && aYmax1 >= aYmax2
                 && aZmin1 <= aZmin2 && aZmax1 >= aZmax2)
        {
            aNumIn = 2;
        }
        return aNumIn;
    }

    //=======================================================================
    // function : GetClosedShells
    // purpose  : Check if one shell is a part from other shell.
    //            For case of compsolid when a few shells are created from
    //            the same set of faces.
    //=======================================================================
    private static void GetClosedShells(NCollection_Sequence<TopoDS_Shape> Shells, NCollection_Sequence<TopoDS_Shape> aRemainShells)
    {
        var aBoxes = new NCollection_Array1<Box>(1, Shells.Length());
        for (int i = 1; i <= Shells.Length(); i++)
        {
            var Box = new Box();
            BRepBndLib.AddClose(Shells.Value(i), ref Box);
            aBoxes.SetValue(i, Box);
        }
        var aMapNum = new NCollection_Map<int>();
        for (int j = 1; j <= aBoxes.Length(); j++)
        {
            for (int k = j + 1; k <= aBoxes.Length(); k++)
            {
                int NumIn = BoxIn(aBoxes.Value(j), aBoxes.Value(k));
                switch (NumIn)
                {
                    case 1:
                        aMapNum.Add(k);
                        break;
                    case 2:
                        aMapNum.Add(j);
                        break;
                    default:
                        break;
                }
            }
        }
        for (int i1 = 1; i1 <= Shells.Length(); i1++)
        {
            if (!aMapNum.Contains(i1))
            {
                aRemainShells.Append(Shells.Value(i1));
            }
        }
    }

    //=======================================================================
    // function : GlueClosedCandidate
    // purpose  : First, attempt to create closed shells from sequence of open shells.
    //=======================================================================
    private static void GlueClosedCandidate(NCollection_Sequence<TopoDS_Shape> OpenShells,
                                            NCollection_Map<TopoDS_Shape> aMapMultiConnectEdges,
                                            NCollection_Sequence<TopoDS_Shape> aSeqNewShells)
    {
        // Creating new shells if some open shells contain the same free boundary.
        for (int i = 1; i < OpenShells.Length(); i++)
        {
            TopoDS_Shape aShell    = OpenShells.Value(i);
            var          mapEdges1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var          dire      = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var          reve      = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            if (!GetFreeEdges(aShell, mapEdges1))
            {
                continue;
            }

            foreach (TopoDS_Shape aKey in mapEdges1)
            {
                TopoDS_Edge edge1 = TopoDS.Edge(aKey);
                if (!aMapMultiConnectEdges.Contains(edge1))
                {
                    break;
                }
                if (edge1.Orientation() == TopAbs_FORWARD)
                {
                    dire.Add(edge1);
                }
                else if (edge1.Orientation() == TopAbs_REVERSED)
                {
                    reve.Add(edge1);
                }
            }
            if (mapEdges1.Extent() > (dire.Extent() + reve.Extent()))
            {
                continue;
            }

            // Filling map MapOtherShells which contains candidate to creation of closed shell
            //  with aShell.
            var MapOtherShells = new NCollection_DataMap<TopoDS_Shape, bool>(TopTools_ShapeMapHasher.Instance);
            for (int j = i + 1; j <= OpenShells.Length(); j++)
            {
                bool         isAddShell = true;
                bool         isReversed = false;
                var          mapEdges2  = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                TopoDS_Shape aShell2    = OpenShells.Value(j);
                if (!GetFreeEdges(aShell2, mapEdges2))
                {
                    continue;
                }
                foreach (TopoDS_Shape aKey2 in mapEdges2)
                {
                    if (!isAddShell)
                    {
                        break;
                    }
                    TopoDS_Edge edge2 = TopoDS.Edge(aKey2);
                    if (!aMapMultiConnectEdges.Contains(edge2))
                    {
                        isAddShell = false;
                        break;
                        // continue;
                    }
                    isAddShell = (dire.Contains(edge2) || reve.Contains(edge2));
                    if ((edge2.Orientation() == TopAbs_FORWARD && dire.Contains(edge2))
                        || (edge2.Orientation() == TopAbs_REVERSED && reve.Contains(edge2)))
                    {
                        isReversed = true;
                    }
                }

                if (!isAddShell)
                {
                    continue;
                }
                MapOtherShells.Bind(OpenShells.Value(j), isReversed);
            }
            if (MapOtherShells.IsEmpty())
            {
                continue;
            }

            if (!MapOtherShells.IsEmpty())
            {
                // Case of compsolid when more than two shells have the same free boundary.
                var aSeqCandidate = new NCollection_Sequence<TopoDS_Shape>();
                aSeqCandidate.Append(OpenShells.Value(i));

                foreach (KeyValuePair<TopoDS_Shape, bool> aIt in MapOtherShells)
                {
                    aSeqCandidate.Append(aIt.Key);
                }

                // Creation of all possible shells from chosen candidate.
                //  And the addition of them to temporary sequence.

                var aTmpSeq = new NCollection_Sequence<TopoDS_Shape>();
                for (int k = 1; k <= aSeqCandidate.Length(); k++)
                {
                    for (int l = k + 1; l <= aSeqCandidate.Length(); l++)
                    {
                        var aNewSh = new TopoDS_Shell();
                        var aB     = new BRep_Builder();
                        aB.MakeShell(ref aNewSh);
                        for (var aIt1 = new TopoDS_Iterator(aSeqCandidate.Value(k), false); aIt1.More(); aIt1.Next())
                        {
                            aB.Add(aNewSh, aIt1.Value());
                        }
                        bool isRev = MapOtherShells.Find(aSeqCandidate.Value(l));
                        if (k != 1)
                        {
                            isRev = (isRev == MapOtherShells.Find(aSeqCandidate.Value(k)));
                        }
                        for (var aExp = new TopExp_Explorer(aSeqCandidate.Value(l), TopAbs_FACE); aExp.More(); aExp.Next())
                        {
                            TopoDS_Shape aFace = (isRev ? aExp.Current().Reversed() : aExp.Current());
                            aB.Add(aNewSh, aFace);
                        }
                        aTmpSeq.Append(aNewSh);
                    }
                }

                // Choice from temporary sequence shells contains different set of faces (case of compsolid)
                var aRemainShells = new NCollection_Sequence<TopoDS_Shape>();
                GetClosedShells(aTmpSeq, aRemainShells);
                aSeqNewShells.Append(aRemainShells);

                for (int j1 = i + 1; j1 <= OpenShells.Length(); j1++)
                {
                    if (!MapOtherShells.IsBound(OpenShells.Value(j1)))
                    {
                        continue;
                    }
                    OpenShells.Remove(j1--);
                }
            }
            else
            {
                var          aB         = new BRep_Builder();
                TopoDS_Shape aNewShell  = aShell;
                TopoDS_Shape addShell   = new();
                bool         isReversed = false;
                for (int j1 = i + 1; j1 <= OpenShells.Length(); j1++)
                {
                    if (!MapOtherShells.Find(OpenShells.Value(j1), out isReversed))
                    {
                        continue;
                    }
                    addShell = OpenShells.Value(j1);
                    OpenShells.Remove(j1);
                    break;
                }

                for (var aExpF = new TopExp_Explorer(addShell, TopAbs_FACE); aExpF.More(); aExpF.Next())
                {
                    TopoDS_Shape aFace = aExpF.Current();
                    if (isReversed)
                    {
                        aFace.Reverse();
                    }
                    aB.Add(aNewShell, aFace);
                }
                aSeqNewShells.Append(aNewShell);
            }

            // OpenShells.ChangeValue(i) = aShell;
            OpenShells.Remove(i--);
        }
    }

    //=======================================================================
    // function : CreateNonManifoldShells
    // purpose  : Attempt to create max possible shells from open shells.
    //=======================================================================
    private static void CreateNonManifoldShells(NCollection_Sequence<TopoDS_Shape> SeqShells,
                                                NCollection_Map<TopoDS_Shape> aMapMultiConnectEdges)
    {
        var aMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= SeqShells.Length(); i++)
        {
            TopoDS_Shape aShell = SeqShells.Value(i);
            var          medeg  = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aShell, TopAbs_EDGE, medeg);
            foreach (TopoDS_Shape ae in aMapMultiConnectEdges)
            {
                // for(TopExp_Explorer aExp(aShell,TopAbs_EDGE); aExp.More(); aExp.Next(),nbe++) {
                // TopoDS_Shape ae = aExp.Current();
                // if( aMapMultiConnectEdges.Contains(aExp.Current())) {
                if (medeg.Contains(ae))
                {
                    if (aMap.Contains(ae))
                    {
                        aMap.ChangeFromKey(ae).Append(aShell);
                    }
                    else
                    {
                        var al = new NCollection_List<TopoDS_Shape>();
                        al.Append(aShell);
                        aMap.Add(ae, al);
                    }
                }
            }
        }
        var aMapShells = new NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (int j = 1; j <= aMap.Extent(); j++)
        {
            NCollection_List<TopoDS_Shape> LShells   = aMap.FindFromIndex(j);
            var                            aNewShell = new TopoDS_Shell();
            var                            aB        = new BRep_Builder();
            aB.MakeShell(ref aNewShell);
            var  mapmerge = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            bool ismerged = false;
            int  num      = 1;
            for (var alit = new NCollection_List<TopoDS_Shape>.Iterator(LShells); alit.More(); alit.Next(), num++)
            {
                if (!aMapShells.Contains(alit.Value()))
                {
                    for (var aEf = new TopExp_Explorer(alit.Value(), TopAbs_FACE); aEf.More(); aEf.Next())
                    {
                        aB.Add(aNewShell, aEf.Current());
                    }
                    ismerged = true;
                    mapmerge.Add(alit.Value());
                }
                else if (ismerged)
                {
                    TopoDS_Shape arshell = aMapShells.FindFromKey(alit.Value());
                    while (aMapShells.Contains(arshell))
                    {
                        TopoDS_Shape ss = aMapShells.FindFromKey(arshell);
                        if (ss.IsSame(arshell))
                        {
                            break;
                        }
                        arshell = ss;
                    }

                    if (!mapmerge.Contains(arshell))
                    {
                        for (var aEf = new TopExp_Explorer(arshell, TopAbs_FACE); aEf.More(); aEf.Next())
                        {
                            aB.Add(aNewShell, aEf.Current());
                        }
                        mapmerge.Add(arshell);
                    }
                }
                else
                {
                    TopoDS_Shape arshell = aMapShells.FindFromKey(alit.Value());
                    while (aMapShells.Contains(arshell))
                    {
                        TopoDS_Shape ss = aMapShells.FindFromKey(arshell);
                        if (ss.IsSame(arshell))
                        {
                            break;
                        }
                        arshell = ss;
                    }
                    if (num == 1)
                    {
                        for (var aEf = new TopExp_Explorer(arshell, TopAbs_FACE); aEf.More(); aEf.Next())
                        {
                            aB.Add(aNewShell, aEf.Current());
                        }

                        mapmerge.Add(arshell);
                    }
                    else if (!mapmerge.Contains(arshell))
                    {
                        for (var aEf = new TopExp_Explorer(arshell, TopAbs_FACE); aEf.More(); aEf.Next())
                        {
                            aB.Add(aNewShell, aEf.Current());
                        }
                        mapmerge.Add(arshell);
                    }
                }
            }
            if (mapmerge.Extent() > 1 || ismerged)
            {
                // OCCT iterates a hashed map here (NCollection_Map); the port's indexed map keeps the insertion order,
                // and IndexedDataMap::Add of a key already bound keeps its first value either way
                for (int m = 1; m <= mapmerge.Extent(); m++)
                {
                    TopoDS_Shape oldShell = mapmerge.FindKey(m);
                    // while(aMapShells.Contains(oldShell)) {
                    //   TopoDS_Shape ss = aMapShells.FindFromKey(oldShell);
                    //   if(ss.IsSame(oldShell)) break;
                    //   oldShell = ss;
                    // }
                    aMapShells.Add(oldShell, aNewShell);
                }
            }
        }
        var MapNewShells = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (int nn = 1; nn <= SeqShells.Length(); nn++)
        {
            if (aMapShells.Contains(SeqShells.Value(nn)))
            {
                TopoDS_Shape aNewShell = aMapShells.FindFromKey(SeqShells.Value(nn));
                while (aMapShells.Contains(aNewShell))
                {
                    TopoDS_Shape ss = aMapShells.FindFromKey(aNewShell);
                    if (ss.IsSame(aNewShell))
                    {
                        break;
                    }
                    aNewShell = ss;
                }
                MapNewShells.Add(aNewShell);

                SeqShells.Remove(nn--);
            }
        }
        for (int ii = 1; ii <= MapNewShells.Extent(); ii++)
        {
            SeqShells.Append(MapNewShells.FindKey(ii));
        }
    }

    //=======================================================================
    // function : CreateClosedShell
    // purpose  : Attempt to create max possible shells from open shells.
    //=======================================================================
    private static void CreateClosedShell(NCollection_Sequence<TopoDS_Shape> OpenShells,
                                          NCollection_Map<TopoDS_Shape> aMapMultiConnectEdges)
    {
        var aNewShells = new NCollection_Sequence<TopoDS_Shape>();
        // First, attempt to create closed shells.
        GlueClosedCandidate(OpenShells, aMapMultiConnectEdges, aNewShells);

        // Creating new shells if some open shells contain the multishared same edges.
        for (int i = 1; i < OpenShells.Length(); i++)
        {
            bool         isAddShell = false;
            TopoDS_Shape aShell     = OpenShells.Value(i);
            bool         isReversed = false;
            for (int j = i + 1; j <= OpenShells.Length(); j++)
            {
                var mapEdges1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                var dire      = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                var reve      = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                if (!GetFreeEdges(aShell, mapEdges1))
                {
                    break;
                }
                foreach (TopoDS_Shape aKey in mapEdges1)
                {
                    TopoDS_Edge edge1 = TopoDS.Edge(aKey);
                    if (!aMapMultiConnectEdges.Contains(edge1))
                    {
                        continue;
                    }
                    if (edge1.Orientation() == TopAbs_FORWARD)
                    {
                        dire.Add(edge1);
                    }
                    else if (edge1.Orientation() == TopAbs_REVERSED)
                    {
                        reve.Add(edge1);
                    }
                }
                if (dire.IsEmpty() && reve.IsEmpty())
                {
                    break;
                }
                var          mapEdges2 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                TopoDS_Shape aShell2   = OpenShells.Value(j);
                if (!GetFreeEdges(aShell2, mapEdges2))
                {
                    continue;
                }
                foreach (TopoDS_Shape aKey2 in mapEdges2)
                {
                    TopoDS_Edge edge2 = TopoDS.Edge(aKey2);
                    if (!aMapMultiConnectEdges.Contains(edge2))
                    {
                        continue;
                    }
                    if (!dire.Contains(edge2) && !reve.Contains(edge2))
                    {
                        continue;
                    }
                    isAddShell = true;
                    if ((edge2.Orientation() == TopAbs_FORWARD && dire.Contains(edge2))
                        || (edge2.Orientation() == TopAbs_REVERSED && reve.Contains(edge2)))
                    {
                        isReversed = true;
                    }
                }

                if (!isAddShell)
                {
                    continue;
                }
                var aB = new BRep_Builder();

                for (var aExpF21 = new TopExp_Explorer(OpenShells.Value(j), TopAbs_FACE); aExpF21.More(); aExpF21.Next())
                {
                    TopoDS_Shape aFace = aExpF21.Current();
                    if (isReversed)
                    {
                        aFace.Reverse();
                    }
                    aB.Add(aShell, aFace);
                }

                OpenShells.ChangeValue(i) = aShell;
                OpenShells.Remove(j--);
            }
        }

        OpenShells.Append(aNewShells);
    }

    //=================================================================================================

    /// <summary>
    /// Fixes the orientation of the faces of <paramref name="shell"/>: faces are reversed so that every edge between two
    /// faces is met in both orientations; faces that can't be oriented with the others make further shells. With
    /// <paramref name="isAccountMultiConex"/>, faces sharing an edge with more than one other face are kept apart; with
    /// <paramref name="NonManifold"/>, one non-manifold shell may be made. Returns true if something was done; the
    /// replacement of the shell is recorded in the context.
    /// </summary>
    /// <remarks>A shell holding frozen shapes is copied first (see the class's remarks).</remarks>
    public bool FixFaceOrientation(in TopoDS_Shell shell, bool isAccountMultiConex = true, bool NonManifold = false)
    {
        var anEntry = new BRepTools_CopyAtEntry(shell, Context());
        bool done = fixFaceOrientation(TopoDS.Shell(anEntry.Shape()), isAccountMultiConex, NonManifold);
        finish(anEntry);
        return done;
    }

    private bool fixFaceOrientation(in TopoDS_Shell shell, bool isAccountMultiConex, bool NonManifold)
    {
        // myStatus = ShapeExtend::EncodeStatus (ShapeExtend_OK);
        bool done           = false;
        var  aSeqShells     = new NCollection_Sequence<TopoDS_Shape>();
        var  aErrFaces      = new NCollection_Sequence<TopoDS_Shape>(); // Compound of faces like to Mebiuce leaf.
        var  Lface          = new NCollection_Sequence<TopoDS_Shape>();
        var  aMapFaceShells = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        myShell = shell;
        myShape = shell;
        int aNumMultShell = 0;
        int nbF           = 0;
        var aMapAdded     = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (var iter = new TopoDS_Iterator(shell); iter.More(); iter.Next(), nbF++)
        {
            if (aMapAdded.Add(iter.Value()))
            {
                Lface.Append(iter.Value());
            }
        }
        if (Lface.Length() < nbF)
        {
            done = true;
        }

        var aMapEdgeFaces = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(myShell, TopAbs_EDGE, TopAbs_FACE, aMapEdgeFaces);
        var  aMapMultiConnectEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        bool isFreeBoundaries      = false;
        for (int k = 1; k <= aMapEdgeFaces.Extent(); k++)
        {
            int aFaceCount = aMapEdgeFaces.FindFromIndex(k).Extent();
            if (!isFreeBoundaries && aFaceCount == 1)
            {
                TopoDS_Edge E = TopoDS.Edge(aMapEdgeFaces.FindKey(k));
                if (!BRep_Tool.Degenerated(E))
                {
                    isFreeBoundaries = true;
                }
            }
            // Finds multishared edges
            else if (isAccountMultiConex && aFaceCount > 2)
            {
                aMapMultiConnectEdges.Add(aMapEdgeFaces.FindKey(k));
            }
        }
        if (BRep_Tool.IsClosed(myShell) ? isFreeBoundaries : !isFreeBoundaries)
        {
            myShell.Closed(!isFreeBoundaries);
            SendWarning(new Msg("FixAdvShell.FixClosedFlag.MSG0")); // Shell has incorrect flag isClosed
        }
        bool isGetShells = true;
        // Gets possible shells with taking in account of multiconnexity.
        while (isGetShells && Lface.Length() != 0)
        {
            var aTmpSeqShells = new NCollection_Sequence<TopoDS_Shape>();
            if (GetShells(Lface, aMapMultiConnectEdges, aTmpSeqShells, aMapFaceShells, aErrFaces))
            {
                done = true;
            }
            isGetShells = !aTmpSeqShells.IsEmpty();
            if (isGetShells)
            {
                aSeqShells.Append(aTmpSeqShells);
            }
        }
        if (!done)
        {
            done = (aSeqShells.Length() > 1);
        }
        bool aIsDone = false;
        if (Lface.Length() > 0 && aSeqShells.Length() != 0)
        {
            for (int jj = 1; jj <= Lface.Length(); jj++)
            {
                if (aMapFaceShells.IsBound(Lface.Value(jj)))
                {
                    aMapFaceShells.UnBind(Lface.Value(jj));
                }
            }

            // Addition of faces having only multiconnexity boundary to shells having holes
            //  containing only the multiconnexity edges
            aIsDone = AddMultiConexityFaces(Lface, aMapMultiConnectEdges, aSeqShells, aMapFaceShells, aMapEdgeFaces,
                                            aErrFaces, NonManifold);
        }
        aNumMultShell = aSeqShells.Length();
        if (!aErrFaces.IsEmpty())
        {
            // if Shell contains of Mebius faces one shell will be created from each those face.
            var B = new BRep_Builder();
            B.MakeCompound(ref myErrFaces);
            var aCompShells = new TopoDS_Compound();
            B.MakeCompound(ref aCompShells);
            for (int n = 1; n <= aErrFaces.Length(); n++)
            {
                B.Add(myErrFaces, aErrFaces.Value(n));
            }
            if (aNumMultShell != 0)
            {
                if (aNumMultShell == 1)
                {
                    B.Add(aCompShells, aSeqShells.Value(1));
                    for (int n1 = 1; n1 <= aErrFaces.Length(); n1++)
                    {
                        var aSh = new TopoDS_Shell();
                        B.MakeShell(ref aSh);
                        B.Add(aSh, aErrFaces.Value(n1));
                        B.Add(aCompShells, aSh);
                    }
                    myShape = aCompShells;
                }
                else
                {
                    for (int i = 1; i <= aSeqShells.Length(); i++)
                    {
                        B.Add(aCompShells, aSeqShells.Value(i));
                    }
                    for (int n1 = 1; n1 <= aErrFaces.Length(); n1++)
                    {
                        var aSh = new TopoDS_Shell();
                        B.MakeShell(ref aSh);
                        B.Add(aSh, aErrFaces.Value(n1));
                        B.Add(aCompShells, aSh);
                    }
                    myShape = aCompShells;
                }
            }

            done     = true;
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL);
            SendWarning(new Msg("FixAdvShell.FixOrientation.MSG20")); // Impossible to orient faces in shell, several shells created
            return true;
        }
        if (aNumMultShell > 1)
        {
            var OpenShells = new NCollection_Sequence<TopoDS_Shape>();
            for (int i1 = 1; i1 <= aSeqShells.Length(); i1++)
            {
                TopoDS_Shape aShell = aSeqShells.Value(i1);
                if (!BRep_Tool.IsClosed(aShell))
                {
                    OpenShells.Append(aShell);
                    aSeqShells.Remove(i1--);
                }
            }
            if (OpenShells.Length() > 1)
            {
                // Attempt of creation closed shell from open shells with taking into account multiconnexity.
                CreateClosedShell(OpenShells, aMapMultiConnectEdges);
            }
            aSeqShells.Append(OpenShells);
        }

        // In the case if NonManifold is equal to true one non-manifold shell will be created.
        // Else compound from shells will be created if length of sequence of shape >1.
        if (Lface.Length() != 0)
        {
            for (int i = 1; i <= Lface.Length(); i++)
            {
                var aB       = new BRep_Builder();
                var OneShell = new TopoDS_Shell();
                aB.MakeShell(ref OneShell);
                aB.Add(OneShell, Lface.Value(i));
                aSeqShells.Append(OneShell);
            }
        }
        if (NonManifold && aSeqShells.Length() > 1)
        {
            CreateNonManifoldShells(aSeqShells, aMapMultiConnectEdges);
        }
        if (!done)
        {
            done = (aSeqShells.Length() > 1 || aIsDone);
        }
        if (aSeqShells.Length() == 1)
        {
            myShell    = TopoDS.Shell(aSeqShells.Value(1));
            myShape    = myShell;
            myNbShells = 1;
        }
        else
        {
            var B           = new BRep_Builder();
            var aCompShells = new TopoDS_Compound();
            B.MakeCompound(ref aCompShells);
            for (int i = 1; i <= aSeqShells.Length(); i++)
            {
                B.Add(aCompShells, aSeqShells.Value(i));
            }
            myShape    = aCompShells;
            myNbShells = aSeqShells.Length();
        }
        if (done)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            Context()?.Replace(shell, myShape);
            if (myNbShells == 1)
            {
                SendWarning(new Msg("FixAdvShell.FixOrientation.MSG0")); // Faces were incorrectly oriented in the shell, corrected
            }
            else
            {
                SendWarning(new Msg("FixAdvShell.FixOrientation.MSG30")); // Improperly connected shell split into parts
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    //=================================================================================================

    /// <summary>Returns the status (the bits of the last fix of the orientation and of the faces).</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Returns the fixed shell (or the first of the subshells made), frozen.</summary>
    public TopoDS_Shell Shell() => TopoDS.Shell(Published(myShell));

    /// <summary>Returns the fixed shell, unfrozen, for ported callers working on shapes they own.</summary>
    internal TopoDS_Shell InternalShell() => myShell;

    /// <summary>Returns the result: a shell, or a compound of the shells made, frozen.</summary>
    public TopoDS_Shape Shape() => Published(myShape);

    /// <summary>Returns the result, unfrozen, for ported callers working on shapes they own.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>Returns the number of shells made.</summary>
    public int NbShells() => myNbShells;

    /// <summary>Returns the faces that couldn't be oriented (Moebius-like), as a compound, frozen.</summary>
    public TopoDS_Compound ErrorFaces() => TopoDS.Compound(Published(myErrFaces));

    /// <summary>The faces that couldn't be oriented, unfrozen.</summary>
    internal TopoDS_Compound InternalErrorFaces() => myErrFaces;

    /// <summary>Returns the face fixing tool.</summary>
    public ShapeFix_Face FixFaceTool() => myFixFace;

    /// <summary>Sets the message registrator, for this tool and the face tool.</summary>
    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        myFixFace.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and the face tool.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        myFixFace.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and the face tool.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        myFixFace.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and the face tool.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        myFixFace.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the face fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixFaceMode() => ref myFixFaceMode;

    /// <summary>The mode of the orientation fix (-1 default, 0 off, 1 on).</summary>
    public ref int FixOrientationMode() => ref myFixOrientationMode;

    /// <summary>Sets whether a non-manifold shell may be made.</summary>
    public virtual void SetNonManifoldFlag(bool isNonManifold)
    {
        myNonManifold = isNonManifold;
    }

    // The end of a fix of a copied shell: the copies left unchanged are the caller's again, in the results and in the
    // context.
    private void finish(BRepTools_CopyAtEntry theEntry)
    {
        if (!theEntry.IsCopied())
        {
            theEntry.Finish(myShape);
            return;
        }
        myShape    = theEntry.Finish(myShape, [myShell, myErrFaces]);
        myShell    = TopoDS.Shell(theEntry.Final(myShell));
        myErrFaces = TopoDS.Compound(theEntry.Final(myErrFaces));
    }

    private static TopoDS_Shape Published(in TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }
}
