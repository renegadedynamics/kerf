// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// pdn 17.12.98 ie_exhaust-A.stp

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes a shell: its faces (by ShapeFix_Face), the orientation of its faces (a shell whose faces can't be oriented
/// consistently is split into several shells), and its closed flag.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> only records the shell. <see cref="Perform"/> and
/// <see cref="FixFaceOrientation"/> edit the shell they work on in place, as OCCT's, in a working overlay (Kerf unit
/// 10H4) that takes the writes into frozen shapes (any shell a caller holds), so the caller's shell never changes. At
/// the end of the fix the results hold new TShapes for what changed and the caller's own for the rest, and the context
/// answers for the caller's sub-shapes. Inside another operation's overlay (ShapeFix_Solid's) they run in place in
/// that one. <see cref="Shell"/>, <see cref="Shape"/> and <see cref="ErrorFaces"/> freeze what they
/// return; ported code that goes on building reads the internal forms.
/// </para>
/// </remarks>
public class ShellFixer : RootFixer
{
    /// <summary>The shell fixed (the first of the shells made).</summary>
    protected Shell    _shell;
    /// <summary>The faces that couldn't be oriented (Moebius-like).</summary>
    protected Compound _errFaces;
    /// <summary>The status.</summary>
    protected int             _status;
    /// <summary>The face fixing tool.</summary>
    protected FaceFixer   _fixFace;
    /// <summary>The mode of the face fixes.</summary>
    protected int             _fixFaceMode;
    /// <summary>The mode of the orientation fix.</summary>
    protected int             _fixOrientationMode;
    /// <summary>The number of shells made.</summary>
    protected int             _nbShells;
    /// <summary>Whether a non-manifold shell may be made.</summary>
    protected bool            _nonManifold;

    // Type aliases for DataMaps with shape hasher (OCCT's FaceEdgesMap is keyed by std::hash, compared with IsEqual)
    private const int DEFAULT_EDGE_FACES_INCREMENT = 5;

    /// <summary>Empty constructor.</summary>
    public ShellFixer()
    {
        _shell              = new Shell();
        _errFaces           = new Compound();
        _status             = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixFaceMode        = -1;
        _fixOrientationMode = -1;
        _fixFace            = new FaceFixer();
        _nbShells           = 0;
        _nonManifold        = false;
    }

    /// <summary>Initializes by a shell.</summary>
    public ShellFixer(in Shell shape)
    {
        _shell              = new Shell();
        _errFaces           = new Compound();
        _status             = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixFaceMode        = -1;
        _fixOrientationMode = -1;
        _fixFace            = new FaceFixer();
        Init(shape);
        _nonManifold = false;
    }

    /// <summary>Initializes by a shell.</summary>
    public void Init(in Shell shell)
    {
        _shape    = shell;
        _shell    = shell;
        _nbShells = 0;
    }

    /// <summary>
    /// Fixes the faces of the shell (if the face mode allows) and the orientation of the faces (if the orientation
    /// mode allows), and corrects the closed flag of the shells. Returns true if something was done.
    /// </summary>
    public bool Perform(ProgressRange progress = default)
    {
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_shell, Context());
        _shape = _shell;
        bool status = PerformCore(progress);
        Finish(overlay);
        return status;
    }

    // ShapeFix_Shell::Perform on the working shell
    private bool PerformCore(ProgressRange progress)
    {
        bool status = false;
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        ReShapeBuilder context = Context()!;
        _fixFace.SetContext(context);

        if (NeedFix(_fixFaceMode))
        {
            Shape s = context.InternalApply(_shell);

            // Get the number of faces for progress indication
            int nbFaces = s.ChildCount;

            // Start progress scope (no need to check if progress exists -- it is safe)
            using var ps = new ProgressScope(progress, "Fixing face", nbFaces);

            for (var iter = new Iterator(s); iter.More() && ps.More(); iter.Next(), ps.Next())
            {
                Shape sh      = iter.Value();
                Face  tmpFace = ShapeCasts.Face(sh);
                _fixFace.Init(tmpFace);
                if (_fixFace.Perform())
                {
                    status = true;
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
            }

            // Halt algorithm in case of user's abort
            if (!ps.More())
            {
                return false;
            }
        }

        Shape newsh = context.InternalApply(_shell);
        if (NeedFix(_fixOrientationMode))
        {
            FixFaceOrientationCore(ShapeCasts.Shell(newsh), true, _nonManifold);
        }

        Shape aNewsh = context.InternalApply(newsh);
        var          sas   = new ShellAnalysis();
        for (var shellExp = new Explorer(aNewsh, global::Kerf.Topology.ShapeType.Shell); shellExp.More(); shellExp.Next())
        {
            Shell curShell = ShapeCasts.Shell(shellExp.Current());
            if (curShell.IsClosed)
            {
                sas.LoadShells(curShell);
                sas.CheckOrientedShells(curShell, true);
                if (sas.HasFreeEdges)
                {
                    curShell.Closed(false);
                    SendWarning(new Msg("FixAdvShell.FixClosedFlag.MSG0")); // Shell has incorrect flag isClosed
                }
                sas.Clear();
            }
        }

        if (status)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (Status(ExtendStatus.DONE2))
        {
            status = true;
        }
        return status;
    }

    //=================================================================================================

    private static bool GetFreeEdges(in Shape aShape, HashSet<Shape> mapEdges)
    {
        for (var expF = new Explorer(aShape, ShapeType.Face); expF.More(); expF.Next())
        {
            for (var expE = new Explorer(expF.Current(), ShapeType.Edge); expE.More(); expE.Next())
            {
                Edge edge = ShapeCasts.Edge(expE.Current());
                if (!mapEdges.Contains(edge))
                {
                    mapEdges.Add(edge);
                }
                else
                {
                    mapEdges.Remove(edge);
                }
            }
        }
        return mapEdges.Count != 0;
    }

    // OCCT's FaceEdgesMap: NCollection_IndexedDataMap<TopoDS_Face, NCollection_Array1<TopoDS_Edge>> with the default
    // hasher (IsEqual); EdgeFacesMap: a DataMap of the edges' faces with the shape hasher.
    private static IndexedMap<Shape, NCollection_Array1<Shape>> NewFaceEdgesMap() =>
        new(EqualityComparer<Shape>.Default);

    /// Groups connected faces into separate sequences using existing connectivity data.
    /// Uses depth-first search to find connected components through shared edges.
    /// Each face appears in exactly one group, ensuring no duplicates across groups.
    /// Groups are sorted by size with the largest group first.
    private static NCollection_List<NCollection_Sequence<Shape>> GetConnectedFaceGroups(
        IndexedMap<Shape, NCollection_Array1<Shape>> faceEdges,
        Dictionary<Shape, List<Shape>> edgeFaces)
    {
        var connectedGroups = new NCollection_List<NCollection_Sequence<Shape>>();

        if (faceEdges.Count == 0)
        {
            return connectedGroups;
        }

        var visitedFaces = new HashSet<Shape>(ShapeHasher.Instance);

        for (int faceIndex = 0; faceIndex < faceEdges.Count; faceIndex++)
        {
            Shape startFace = faceEdges.KeyAt(faceIndex);

            if (visitedFaces.Contains(startFace))
            {
                continue;
            }

            // Start new connected group
            var connectedGroup = new NCollection_Sequence<Shape>();

            // DFS traversal
            var stack = new Stack<Shape>();
            stack.Push(startFace);
            visitedFaces.Add(startFace);

            while (stack.Count != 0)
            {
                Shape currentFace = stack.Pop();
                connectedGroup.Append(currentFace);

                // Find connected faces through shared edges
                if (faceEdges.TryGetValue(currentFace, out NCollection_Array1<Shape> faceEdgesArray))
                {
                    for (int edgeIdx = faceEdgesArray.Lower(); edgeIdx <= faceEdgesArray.Upper(); ++edgeIdx)
                    {
                        Shape edge = faceEdgesArray.Value(edgeIdx);

                        if (edgeFaces.TryGetValue(edge, out List<Shape>? connectedFaces))
                        {
                            for (int faceIdx = 0; faceIdx < connectedFaces.Count; ++faceIdx)
                            {
                                Shape neighborFace = connectedFaces[faceIdx];

                                if (!visitedFaces.Contains(neighborFace))
                                {
                                    visitedFaces.Add(neighborFace);
                                    stack.Push(neighborFace);
                                }
                            }
                        }
                    }
                }
            }

            // Insert in sorted order (largest groups first)
            bool isInserted = false;

            for (var iter = new NCollection_List<NCollection_Sequence<Shape>>.Iterator(connectedGroups);
                 iter.More();
                 iter.Next())
            {
                if (connectedGroup.Length() > iter.Value().Length())
                {
                    connectedGroups.InsertBefore(connectedGroup, iter);
                    isInserted = true;
                    break;
                }
            }

            if (!isInserted)
            {
                connectedGroups.Append(connectedGroup);
            }
        }

        return connectedGroups;
    }

    /// Creates shells from connected face groups using connectivity analysis.
    /// Processes only the largest connected group for shell construction, improving efficiency
    /// by focusing on faces that are actually topologically connected.
    private static bool GetShells(NCollection_Sequence<Shape> lfaces,
                                  HashSet<Shape> mapMultiConnectEdges,
                                  NCollection_Sequence<Shape> seqShells,
                                  Dictionary<Shape, Shape> mapFaceShells,
                                  NCollection_Sequence<Shape> errFaces)
    {
        bool done = false;
        if (lfaces.Length() == 0)
        {
            return false;
        }
        var nshell = new Shell();
        var b      = new BrepBuilder();
        b.MakeShell(ref nshell);
        bool isMultiConnex    = mapMultiConnectEdges.Count != 0;
        int  faceIdx           = 1, facesInShellCount = 1;
        var  seqUnconnectFaces = new NCollection_Sequence<Shape>();

        IndexedMap<Shape, NCollection_Array1<Shape>> faceEdges = NewFaceEdgesMap();
        int numberOfEdges = 0;
        var tempEdges     = new List<Shape>();
        for (int lfaceIdx = 1; lfaceIdx <= lfaces.Length(); lfaceIdx++)
        {
            tempEdges.Clear();
            Face face = ShapeCasts.Face(lfaces.Value(lfaceIdx));
            for (var edgeExp = new Explorer(face, ShapeType.Edge); edgeExp.More(); edgeExp.Next())
            {
                tempEdges.Add(ShapeCasts.Edge(edgeExp.Current()));
                numberOfEdges++;
            }
            var faceEdgesArray = new NCollection_Array1<Shape>(1, tempEdges.Count);
            for (int idx = 0; idx < tempEdges.Count; ++idx)
            {
                faceEdgesArray.SetValue(idx + 1, tempEdges[idx]);
            }
            faceEdges.Add(face, faceEdgesArray);
        }

        var edgeFaces = new Dictionary<Shape, List<Shape>>(ShapeHasher.Instance);

        for (int faceInd = 0; faceInd < faceEdges.Count; ++faceInd)
        {
            Shape                       face           = faceEdges.KeyAt(faceInd);
            NCollection_Array1<Shape> faceEdgesArray = faceEdges.ValueAt(faceInd);

            for (int edgeInd = faceEdgesArray.Lower(); edgeInd <= faceEdgesArray.Upper(); ++edgeInd)
            {
                Shape anEdge = faceEdgesArray.Value(edgeInd);

                if (!edgeFaces.ContainsKey(anEdge))
                {
                    edgeFaces[anEdge] = new List<Shape>();
                }
                List<Shape> facesArray = edgeFaces.GetValue(anEdge);

                // Check if face already exists in the array
                bool faceExists = false;
                for (int faceCheckIdx = 0; faceCheckIdx < facesArray.Count; ++faceCheckIdx)
                {
                    if (facesArray[faceCheckIdx].IsSame(face))
                    {
                        faceExists = true;
                        break;
                    }
                }

                // (OCCT sets the array's increment to DEFAULT_EDGE_FACES_INCREMENT here: an allocation detail)
                _ = DEFAULT_EDGE_FACES_INCREMENT;

                if (!faceExists)
                {
                    facesArray.Add(face);
                }
            }
        }

        // Get connected groups of faces using existing connectivity data
        NCollection_List<NCollection_Sequence<Shape>> connectedGroups = GetConnectedFaceGroups(faceEdges, edgeFaces);

        // Process only the largest connected group for shell construction
        if (connectedGroups.IsEmpty())
        {
            return false;
        }

        // Some assumption that each edge can be in two orientations
        var processedEdges = new Dictionary<Shape, (bool first, bool second)>(ShapeHasher.Instance);

        NCollection_Sequence<Shape> processingFaces = connectedGroups.First();

        var tempProcessedEdges = new Dictionary<Shape, (bool first, bool second)>(ShapeHasher.Instance);
        for (; faceIdx <= processingFaces.Length(); faceIdx++)
        {
            tempProcessedEdges.Clear();

            int         badOrientationCount = 0, goodOrientationCount = 0;
            Face f1                   = ShapeCasts.Face(processingFaces.Value(faceIdx));
            // Get edges of the face
            NCollection_Array1<Shape> faceEdgesArray = faceEdges.GetValue(f1);

            for (int edgeInd = faceEdgesArray.Lower(); edgeInd <= faceEdgesArray.Upper(); ++edgeInd)
            {
                Shape edge = faceEdgesArray.Value(edgeInd);

                // if multiconnexity mode is equal to true faces contains
                // the same multiconnexity edges are not added to one shell.
                if (isMultiConnex && mapMultiConnectEdges.Contains(edge))
                {
                    continue;
                }

                if (!processedEdges.ContainsKey(edge))
                {
                    if (!tempProcessedEdges.ContainsKey(edge))
                    {
                        (bool, bool) edgeOrientationPair = (edge.Orientation == Forward,
                                                              edge.Orientation == Reversed);

                        tempProcessedEdges[edge] = edgeOrientationPair;
                    }
                    else
                    {
                        ref (bool first, bool second) tempProcessedEdgeIt = ref tempProcessedEdges.GetValueRef(edge);
                        tempProcessedEdgeIt.first  = tempProcessedEdgeIt.first || (edge.Orientation == Forward);
                        tempProcessedEdgeIt.second = tempProcessedEdgeIt.second || (edge.Orientation == Reversed);
                    }
                    continue;
                }

                ref (bool first, bool second) pair = ref processedEdges.GetValueRef(edge);

                bool isDirect   = pair.first;
                bool isReversed = pair.second;

                if ((edge.Orientation == Forward && isDirect)
                    || (edge.Orientation == Reversed && isReversed))
                {
                    badOrientationCount++;
                }
                else if ((edge.Orientation == Forward && isReversed)
                         || (edge.Orientation == Reversed && isDirect))
                {
                    goodOrientationCount++;
                }

                if (isDirect)
                {
                    pair.first = false;
                }
                else if (isReversed)
                {
                    pair.second = false;
                }

                if (!pair.first && !pair.second)
                {
                    // if edge is processed in this face it is removed from map of processed edges
                    processedEdges.Remove(edge);
                }
            }

            if (badOrientationCount == 0 && goodOrientationCount == 0 && tempProcessedEdges.Count == 0)
            {
                continue;
            }

            // if face can not be added to shell it added to sequence of error faces.

            if (goodOrientationCount != 0 && badOrientationCount != 0)
            {
                errFaces.Append(f1);
                processingFaces.Remove(faceIdx);
                facesInShellCount++;
                continue;
            }

            // Addition of face to shell. In the dependance of orientation faces in the shell
            //  added face can be reversed.

            if ((goodOrientationCount != 0 || badOrientationCount != 0) || facesInShellCount == 1)
            {
                if (badOrientationCount != 0)
                {
                    f1.Reverse();

                    foreach (KeyValuePair<Shape, (bool first, bool second)> tempEdgeIter in tempProcessedEdges)
                    {
                        Shape                  edge                  = tempEdgeIter.Key;
                        (bool first, bool second)     edgeOrientationPair = tempEdgeIter.Value;

                        (bool, bool) revertedPair = (!edgeOrientationPair.first, !edgeOrientationPair.second);

                        if (!processedEdges.ContainsKey(edge))
                        {
                            processedEdges[edge] = revertedPair;
                        }
                        else
                        {
                            processedEdges.GetValueRef(edge) = revertedPair;
                        }
                    }
                    done = true;
                }
                else
                {
                    foreach (KeyValuePair<Shape, (bool first, bool second)> tempEdgeIter in tempProcessedEdges)
                    {
                        Shape              edge                  = tempEdgeIter.Key;
                        (bool first, bool second) edgeOrientationPair = tempEdgeIter.Value;

                        if (!processedEdges.ContainsKey(edge))
                        {
                            processedEdges[edge] = edgeOrientationPair;
                        }
                        else
                        {
                            processedEdges.GetValueRef(edge) = edgeOrientationPair;
                        }
                    }
                }
                facesInShellCount++;
                b.Add(nshell, f1);
                mapFaceShells[f1] = nshell;
                processingFaces.Remove(faceIdx);

                // check if closed shell is obtained in multi connex mode and add to sequence of
                // shells and new shell begin to construct.
                // (check is n*2)
                if (isMultiConnex && Tool.IsClosed(nshell))
                {
                    nshell.Closed(true);
                    seqShells.Append(nshell);
                    var nshellnext = new Shell();
                    b.MakeShell(ref nshellnext);
                    nshell             = nshellnext;
                    facesInShellCount = 1;
                }

                faceIdx = 0;
            }
            // if shell contains of one face. This face is added to sequence of faces.
            //  This shell is removed.
            if (processingFaces.Length() != 0 && faceIdx == processingFaces.Length() && facesInShellCount <= 2)
            {
                var itf = new Iterator(nshell, false);
                if (itf.More())
                {
                    seqUnconnectFaces.Append(itf.Value());
                    mapFaceShells.Remove(itf.Value());
                }
                var nshellnext = new Shell();
                b.MakeShell(ref nshellnext);
                nshell             = nshellnext;
                faceIdx           = 0;
                facesInShellCount = 1;
            }
        }
        bool isContains = false;
        for (int k = 1; k <= seqShells.Length() && !isContains; k++)
        {
            isContains = nshell.IsSame(seqShells.Value(k));
        }
        if (!isContains)
        {
            int          numFace = 0;
            Shape face   = new();
            for (var itf = new Iterator(nshell, false); itf.More(); itf.Next())
            {
                face = itf.Value();
                numFace++;
            }
            if (numFace > 1)
            {
                // close all closed shells in no multi connex mode
                if (!isMultiConnex)
                {
                    nshell.Closed(Tool.IsClosed(nshell));
                }
                seqShells.Append(nshell);
            }
            else if (numFace == 1)
            {
                if (mapFaceShells.ContainsKey(face))
                {
                    mapFaceShells.Remove(face);
                }
                processingFaces.Append(face);
            }
        }

        // Add all unprocessed connected groups (second group and after) to unconnected faces
        int groupIndex = 1;
        foreach (NCollection_Sequence<Shape> unprocessedGroup in connectedGroups)
        {
            if (groupIndex++ == 1)
            {
                continue; // Skip first group (already processed)
            }

            for (int unprocFaceIdx = 1; unprocFaceIdx <= unprocessedGroup.Length(); ++unprocFaceIdx)
            {
                seqUnconnectFaces.Append(unprocessedGroup.Value(unprocFaceIdx));
            }
        }

        lfaces.Assign(processingFaces);

        // Add unconnected faces from the largest group that couldn't be added to shells
        for (int j1 = 1; j1 <= seqUnconnectFaces.Length(); j1++)
        {
            lfaces.Append(seqUnconnectFaces.Value(j1));
        }

        return done;
    }

    //=======================================================================
    // function : AddMultiConexityFaces
    // purpose  : In this function faces have only of multiconnexity boundary
    //            are added to shells having free boundary contains the same
    //            multiconnexity edges as faces.
    //=======================================================================
    private static bool AddMultiConexityFaces(NCollection_Sequence<Shape> lface,
                                              HashSet<Shape> aMapMultiConnectEdges,
                                              NCollection_Sequence<Shape> seqShells,
                                              Dictionary<Shape, Shape> aMapFaceShells,
                                              IndexedMap<Shape, NCollection_List<Shape>> aMapEdgeFaces,
                                              NCollection_Sequence<Shape> errFaces,
                                              bool nonManifold)
    {
        bool done = false;
        //  BRep_Builder aB;
        var llPosibleShells = new NCollection_Sequence<Shape>();
        var addShapes       = new NCollection_Sequence<Shape>();
        for (int i1 = 1; i1 <= lface.Length(); i1++)
        {
            Shape shape = lface.Value(i1);

            int nbMultEdges = 0;

            // Finds faces having only multiconnexity boundary.
            for (var itWires = new Iterator(shape, false); itWires.More(); itWires.Next())
            {
                int nbEdges = 0;
                for (var itEdges = new Iterator(itWires.Value(), false); itEdges.More(); itEdges.Next(), nbEdges++)
                {
                    Shape edge = itEdges.Value();
                    if (!aMapMultiConnectEdges.Contains(edge))
                    {
                        continue;
                    }
                    nbMultEdges++;
                }
                if (nbMultEdges == 0)
                {
                    continue;
                }

                if (nbMultEdges == nbEdges)
                {
                    addShapes.Append(shape);
                }
                else
                {
                    llPosibleShells.Append(shape);
                }
            }
        }

        // Attempt to create shell from unconnected which have not only multiconnexity boundary.
        var tmpShells = new NCollection_Sequence<Shape>();
        if (!llPosibleShells.IsEmpty())
        {
            var map          = new HashSet<Shape>(ShapeHasher.Instance);
            var tmp          = new NCollection_Sequence<Shape>();
            var tmpFaceShell = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
            if (GetShells(llPosibleShells, map, tmpShells, tmpFaceShell, tmp))
            {
                for (int kk = 1; kk <= tmpShells.Length(); kk++)
                {
                    Shape sh      = tmpShells.Value(kk);
                    var          mapEdges = new HashSet<Shape>(ShapeHasher.Instance);
                    if (GetFreeEdges(sh, mapEdges))
                    {
                        int nbedge = 0;
                        foreach (Shape key in mapEdges)
                        {
                            if (aMapMultiConnectEdges.Contains(key))
                            {
                                nbedge++;
                            }
                        }
                        if (nbedge != 0 && nbedge == mapEdges.Count)
                        {
                            addShapes.Append(sh);
                        }
                    }
                }
            }
        }

        // Add chosen faces to shells.
        for (int k1 = 1; k1 <= addShapes.Length(); k1++)
        {
            var          mapOtherShells = new Dictionary<Shape, int>(ShapeHasher.Instance);
            var          dire           = new HashSet<Shape>(ShapeHasher.Instance);
            var          reve           = new HashSet<Shape>(ShapeHasher.Instance);
            Shape sh            = addShapes.Value(k1);
            var          mapEdges       = new HashSet<Shape>(ShapeHasher.Instance);
            if (!GetFreeEdges(sh, mapEdges))
            {
                continue;
            }
            var lfaces = new NCollection_List<Shape>();

            // Fill MapOtherShells which will contain shells with orientation in which selected shape aSh
            // will be add.

            foreach (Shape key in mapEdges)
            {
                if (!aMapMultiConnectEdges.Contains(key))
                {
                    continue;
                }
                Edge edge = ShapeCasts.Edge(key);
                if (edge.Orientation == Forward)
                {
                    dire.Add(edge);
                }
                else
                {
                    reve.Add(edge);
                }
                NCollection_List<Shape> lf = aMapEdgeFaces.GetValue(edge);
                lfaces.Append(new NCollection_List<Shape>(lf));
            }
            foreach (Shape f in lfaces)
            {
                if (!aMapFaceShells.ContainsKey(f))
                {
                    continue;
                }

                Shape othershell = aMapFaceShells.GetValue(f);
                if (mapOtherShells.ContainsKey(othershell))
                {
                    continue;
                }
                if (!nonManifold && Tool.IsClosed(othershell))
                {
                    continue;
                }

                var mapShellEdges = new HashSet<Shape>(ShapeHasher.Instance);
                GetFreeEdges(othershell, mapShellEdges);
                bool isAdd = true;
                foreach (Shape key1 in mapEdges)
                {
                    if (!isAdd)
                    {
                        break;
                    }
                    isAdd = mapShellEdges.Contains(key1);
                }

                if (!isAdd)
                {
                    continue;
                }
                int nbdir = 0, nbrev = 0;

                // add only free face whome all edges contains in the shell as open boundary.
                foreach (Shape keyS in mapShellEdges)
                {
                    Edge edgeS = ShapeCasts.Edge(keyS);
                    if (!aMapMultiConnectEdges.Contains(edgeS))
                    {
                        continue;
                    }
                    if ((edgeS.Orientation == Forward && dire.Contains(edgeS))
                        || (edgeS.Orientation == Reversed && reve.Contains(edgeS)))
                    {
                        nbrev++;
                    }
                    else if ((edgeS.Orientation == Forward && reve.Contains(edgeS))
                             || (edgeS.Orientation == Reversed && dire.Contains(edgeS)))
                    {
                        nbdir++;
                    }
                }
                if (nbdir != 0 && nbrev != 0)
                {
                    errFaces.Append(sh);
                    continue;
                }
                if (nbdir != 0 || nbrev != 0)
                {
                    int isReverse = (nbrev != 0 ? 1 : 0);
                    mapOtherShells[othershell] = isReverse;
                }
            }
            if (mapOtherShells.Count == 0)
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
            int firstRev = 0, firstInd = 0;
            int ind      = 0;
            for (int l = 1; l <= seqShells.Length(); l++)
            {
                if (!mapOtherShells.ContainsKey(seqShells.Value(l)))
                {
                    continue;
                }
                ind++;
                int          isRev     = mapOtherShells.GetValue(seqShells.Value(l));
                Shape anewShape = (isRev != 0 ? sh.Reversed() : sh);

                var          b1    = new BrepBuilder();
                Shape shell = seqShells.Value(l);
                if (ind == 1 || !nonManifold)
                {
                    if (ind == 1)
                    {
                        firstRev = isRev;
                        firstInd = l;
                    }
                    for (var e = new Explorer(anewShape, ShapeType.Face); e.More(); e.Next())
                    {
                        b1.Add(shell, e.Current());
                    }
                    seqShells.ChangeValue(l) = shell;
                }
                else if (nonManifold)
                {
                    bool isReversed = ((isRev != 0) || firstRev != 0) && (!(isRev != 0) || firstRev == 0);
                    shell          = seqShells.Value(firstInd);
                    for (var itF = new Iterator(seqShells.Value(l), false); itF.More(); itF.Next())
                    {
                        Shape nF = (isReversed ? itF.Value().Reversed() : itF.Value());
                        b1.Add(shell, nF);
                    }
                    seqShells.ChangeValue(firstInd) = shell;
                    seqShells.Remove(l--);
                }
            }

            dire.Clear();
            reve.Clear();
            for (var et = new Explorer(sh, ShapeType.Face); et.More(); et.Next())
            {
                for (int kk = 1; kk <= lface.Length(); kk++)
                {
                    if (et.Current().IsSame(lface.Value(kk)))
                    {
                        lface.Remove(kk--);
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
    private static int BoxIn(in Box box1, in Box box2)
    {
        int numIn = 0;
        box1.Get(out double xmin1, out double ymin1, out double zmin1, out double xmax1, out double ymax1, out double zmax1);
        box2.Get(out double xmin2, out double ymin2, out double zmin2, out double xmax2, out double ymax2, out double zmax2);
        if (xmin1 == xmin2 && xmax1 == xmax2 && ymin1 == ymin2 && ymax1 == ymax2
            && zmin1 == zmin2 && zmax1 == zmax2)
        {
            numIn = 0;
        }
        else if (xmin1 >= xmin2 && xmax1 <= xmax2 && ymin1 >= ymin2 && ymax1 <= ymax2
                 && zmin1 >= zmin2 && zmax1 <= zmax2)
        {
            numIn = 1;
        }
        else if (xmin1 <= xmin2 && xmax1 >= xmax2 && ymin1 <= ymin2 && ymax1 >= ymax2
                 && zmin1 <= zmin2 && zmax1 >= zmax2)
        {
            numIn = 2;
        }
        return numIn;
    }

    //=======================================================================
    // function : GetClosedShells
    // purpose  : Check if one shell is a part from other shell.
    //            For case of compsolid when a few shells are created from
    //            the same set of faces.
    //=======================================================================
    private static void GetClosedShells(NCollection_Sequence<Shape> shells, NCollection_Sequence<Shape> aRemainShells)
    {
        var boxes = new NCollection_Array1<Box>(1, shells.Length());
        for (int i = 1; i <= shells.Length(); i++)
        {
            var box = new Box();
            Bounds.AddClose(shells.Value(i), ref box);
            boxes.SetValue(i, box);
        }
        var mapNum = new HashSet<int>();
        for (int j = 1; j <= boxes.Length(); j++)
        {
            for (int k = j + 1; k <= boxes.Length(); k++)
            {
                int numIn = BoxIn(boxes.Value(j), boxes.Value(k));
                switch (numIn)
                {
                    case 1:
                        mapNum.Add(k);
                        break;
                    case 2:
                        mapNum.Add(j);
                        break;
                    default:
                        break;
                }
            }
        }
        for (int i1 = 1; i1 <= shells.Length(); i1++)
        {
            if (!mapNum.Contains(i1))
            {
                aRemainShells.Append(shells.Value(i1));
            }
        }
    }

    //=======================================================================
    // function : GlueClosedCandidate
    // purpose  : First, attempt to create closed shells from sequence of open shells.
    //=======================================================================
    private static void GlueClosedCandidate(NCollection_Sequence<Shape> openShells,
                                            HashSet<Shape> aMapMultiConnectEdges,
                                            NCollection_Sequence<Shape> aSeqNewShells)
    {
        // Creating new shells if some open shells contain the same free boundary.
        for (int i = 1; i < openShells.Length(); i++)
        {
            Shape shell    = openShells.Value(i);
            var          mapEdges1 = new HashSet<Shape>(ShapeHasher.Instance);
            var          dire      = new HashSet<Shape>(ShapeHasher.Instance);
            var          reve      = new HashSet<Shape>(ShapeHasher.Instance);
            if (!GetFreeEdges(shell, mapEdges1))
            {
                continue;
            }

            foreach (Shape key in mapEdges1)
            {
                Edge edge1 = ShapeCasts.Edge(key);
                if (!aMapMultiConnectEdges.Contains(edge1))
                {
                    break;
                }
                if (edge1.Orientation == Forward)
                {
                    dire.Add(edge1);
                }
                else if (edge1.Orientation == Reversed)
                {
                    reve.Add(edge1);
                }
            }
            if (mapEdges1.Count > (dire.Count + reve.Count))
            {
                continue;
            }

            // Filling map MapOtherShells which contains candidate to creation of closed shell
            //  with aShell.
            var mapOtherShells = new Dictionary<Shape, bool>(ShapeHasher.Instance);
            for (int j = i + 1; j <= openShells.Length(); j++)
            {
                bool         isAddShell = true;
                bool         isReversed = false;
                var          mapEdges2  = new HashSet<Shape>(ShapeHasher.Instance);
                Shape shell2    = openShells.Value(j);
                if (!GetFreeEdges(shell2, mapEdges2))
                {
                    continue;
                }
                foreach (Shape key2 in mapEdges2)
                {
                    if (!isAddShell)
                    {
                        break;
                    }
                    Edge edge2 = ShapeCasts.Edge(key2);
                    if (!aMapMultiConnectEdges.Contains(edge2))
                    {
                        isAddShell = false;
                        break;
                        // continue;
                    }
                    isAddShell = (dire.Contains(edge2) || reve.Contains(edge2));
                    if ((edge2.Orientation == Forward && dire.Contains(edge2))
                        || (edge2.Orientation == Reversed && reve.Contains(edge2)))
                    {
                        isReversed = true;
                    }
                }

                if (!isAddShell)
                {
                    continue;
                }
                mapOtherShells[openShells.Value(j)] = isReversed;
            }
            if (mapOtherShells.Count == 0)
            {
                continue;
            }

            if (mapOtherShells.Count != 0)
            {
                // Case of compsolid when more than two shells have the same free boundary.
                var seqCandidate = new NCollection_Sequence<Shape>();
                seqCandidate.Append(openShells.Value(i));

                foreach (KeyValuePair<Shape, bool> it in mapOtherShells)
                {
                    seqCandidate.Append(it.Key);
                }

                // Creation of all possible shells from chosen candidate.
                //  And the addition of them to temporary sequence.

                var tmpSeq = new NCollection_Sequence<Shape>();
                for (int k = 1; k <= seqCandidate.Length(); k++)
                {
                    for (int l = k + 1; l <= seqCandidate.Length(); l++)
                    {
                        var newSh = new Shell();
                        var b     = new BrepBuilder();
                        b.MakeShell(ref newSh);
                        for (var it1 = new Iterator(seqCandidate.Value(k), false); it1.More(); it1.Next())
                        {
                            b.Add(newSh, it1.Value());
                        }
                        bool isRev = mapOtherShells.GetValue(seqCandidate.Value(l));
                        if (k != 1)
                        {
                            isRev = (isRev == mapOtherShells.GetValue(seqCandidate.Value(k)));
                        }
                        for (var exp = new Explorer(seqCandidate.Value(l), ShapeType.Face); exp.More(); exp.Next())
                        {
                            Shape face = (isRev ? exp.Current().Reversed() : exp.Current());
                            b.Add(newSh, face);
                        }
                        tmpSeq.Append(newSh);
                    }
                }

                // Choice from temporary sequence shells contains different set of faces (case of compsolid)
                var remainShells = new NCollection_Sequence<Shape>();
                GetClosedShells(tmpSeq, remainShells);
                aSeqNewShells.Append(remainShells);

                for (int j1 = i + 1; j1 <= openShells.Length(); j1++)
                {
                    if (!mapOtherShells.ContainsKey(openShells.Value(j1)))
                    {
                        continue;
                    }
                    openShells.Remove(j1--);
                }
            }
            else
            {
                var          b         = new BrepBuilder();
                Shape newShell  = shell;
                Shape addShell   = new();
                bool         isReversed = false;
                for (int j1 = i + 1; j1 <= openShells.Length(); j1++)
                {
                    if (!mapOtherShells.TryGetValue(openShells.Value(j1), out isReversed))
                    {
                        continue;
                    }
                    addShell = openShells.Value(j1);
                    openShells.Remove(j1);
                    break;
                }

                for (var expF = new Explorer(addShell, ShapeType.Face); expF.More(); expF.Next())
                {
                    Shape face = expF.Current();
                    if (isReversed)
                    {
                        face.Reverse();
                    }
                    b.Add(newShell, face);
                }
                aSeqNewShells.Append(newShell);
            }

            // OpenShells.ChangeValue(i) = aShell;
            openShells.Remove(i--);
        }
    }

    //=======================================================================
    // function : CreateNonManifoldShells
    // purpose  : Attempt to create max possible shells from open shells.
    //=======================================================================
    private static void CreateNonManifoldShells(NCollection_Sequence<Shape> seqShells,
                                                HashSet<Shape> aMapMultiConnectEdges)
    {
        var map = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        for (int i = 1; i <= seqShells.Length(); i++)
        {
            Shape shell = seqShells.Value(i);
            var          medeg  = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(shell, ShapeType.Edge, medeg);
            foreach (Shape ae in aMapMultiConnectEdges)
            {
                // for(TopExp_Explorer aExp(aShell,TopAbs_EDGE); aExp.More(); aExp.Next(),nbe++) {
                // TopoDS_Shape ae = aExp.Current();
                // if( aMapMultiConnectEdges.Contains(aExp.Current())) {
                if (medeg.Contains(ae))
                {
                    if (map.ContainsKey(ae))
                    {
                        map.GetValue(ae).Append(shell);
                    }
                    else
                    {
                        var al = new NCollection_List<Shape>();
                        al.Append(shell);
                        map.Add(ae, al);
                    }
                }
            }
        }
        var mapShells = new IndexedMap<Shape, Shape>(ShapeHasher.Instance);
        for (int j = 0; j < map.Count; j++)
        {
            NCollection_List<Shape> lShells   = map.ValueAt(j);
            var                            newShell = new Shell();
            var                            b        = new BrepBuilder();
            b.MakeShell(ref newShell);
            var  mapmerge = new IndexedSet<Shape>(ShapeHasher.Instance);
            bool ismerged = false;
            int  num      = 1;
            for (var alit = new NCollection_List<Shape>.Iterator(lShells); alit.More(); alit.Next(), num++)
            {
                if (!mapShells.ContainsKey(alit.Value()))
                {
                    for (var ef = new Explorer(alit.Value(), ShapeType.Face); ef.More(); ef.Next())
                    {
                        b.Add(newShell, ef.Current());
                    }
                    ismerged = true;
                    mapmerge.Add(alit.Value());
                }
                else if (ismerged)
                {
                    Shape arshell = mapShells.GetValue(alit.Value());
                    while (mapShells.ContainsKey(arshell))
                    {
                        Shape ss = mapShells.GetValue(arshell);
                        if (ss.IsSame(arshell))
                        {
                            break;
                        }
                        arshell = ss;
                    }

                    if (!mapmerge.Contains(arshell))
                    {
                        for (var ef = new Explorer(arshell, ShapeType.Face); ef.More(); ef.Next())
                        {
                            b.Add(newShell, ef.Current());
                        }
                        mapmerge.Add(arshell);
                    }
                }
                else
                {
                    Shape arshell = mapShells.GetValue(alit.Value());
                    while (mapShells.ContainsKey(arshell))
                    {
                        Shape ss = mapShells.GetValue(arshell);
                        if (ss.IsSame(arshell))
                        {
                            break;
                        }
                        arshell = ss;
                    }
                    if (num == 1)
                    {
                        for (var ef = new Explorer(arshell, ShapeType.Face); ef.More(); ef.Next())
                        {
                            b.Add(newShell, ef.Current());
                        }

                        mapmerge.Add(arshell);
                    }
                    else if (!mapmerge.Contains(arshell))
                    {
                        for (var ef = new Explorer(arshell, ShapeType.Face); ef.More(); ef.Next())
                        {
                            b.Add(newShell, ef.Current());
                        }
                        mapmerge.Add(arshell);
                    }
                }
            }
            if (mapmerge.Count > 1 || ismerged)
            {
                // OCCT iterates a hashed map here (NCollection_Map); the port's indexed map keeps the insertion order,
                // and IndexedDataMap::Add of a key already bound keeps its first value either way
                for (int m = 0; m < mapmerge.Count; m++)
                {
                    Shape oldShell = mapmerge[m];
                    // while(aMapShells.Contains(oldShell)) {
                    //   TopoDS_Shape ss = aMapShells.FindFromKey(oldShell);
                    //   if(ss.IsSame(oldShell)) break;
                    //   oldShell = ss;
                    // }
                    mapShells.Add(oldShell, newShell);
                }
            }
        }
        var mapNewShells = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (int nn = 1; nn <= seqShells.Length(); nn++)
        {
            if (mapShells.ContainsKey(seqShells.Value(nn)))
            {
                Shape newShell = mapShells.GetValue(seqShells.Value(nn));
                while (mapShells.ContainsKey(newShell))
                {
                    Shape ss = mapShells.GetValue(newShell);
                    if (ss.IsSame(newShell))
                    {
                        break;
                    }
                    newShell = ss;
                }
                mapNewShells.Add(newShell);

                seqShells.Remove(nn--);
            }
        }
        for (int ii = 0; ii < mapNewShells.Count; ii++)
        {
            seqShells.Append(mapNewShells[ii]);
        }
    }

    //=======================================================================
    // function : CreateClosedShell
    // purpose  : Attempt to create max possible shells from open shells.
    //=======================================================================
    private static void CreateClosedShell(NCollection_Sequence<Shape> openShells,
                                          HashSet<Shape> aMapMultiConnectEdges)
    {
        var newShells = new NCollection_Sequence<Shape>();
        // First, attempt to create closed shells.
        GlueClosedCandidate(openShells, aMapMultiConnectEdges, newShells);

        // Creating new shells if some open shells contain the multishared same edges.
        for (int i = 1; i < openShells.Length(); i++)
        {
            bool         isAddShell = false;
            Shape shell     = openShells.Value(i);
            bool         isReversed = false;
            for (int j = i + 1; j <= openShells.Length(); j++)
            {
                var mapEdges1 = new HashSet<Shape>(ShapeHasher.Instance);
                var dire      = new HashSet<Shape>(ShapeHasher.Instance);
                var reve      = new HashSet<Shape>(ShapeHasher.Instance);
                if (!GetFreeEdges(shell, mapEdges1))
                {
                    break;
                }
                foreach (Shape key in mapEdges1)
                {
                    Edge edge1 = ShapeCasts.Edge(key);
                    if (!aMapMultiConnectEdges.Contains(edge1))
                    {
                        continue;
                    }
                    if (edge1.Orientation == Forward)
                    {
                        dire.Add(edge1);
                    }
                    else if (edge1.Orientation == Reversed)
                    {
                        reve.Add(edge1);
                    }
                }
                if (dire.Count == 0 && reve.Count == 0)
                {
                    break;
                }
                var          mapEdges2 = new HashSet<Shape>(ShapeHasher.Instance);
                Shape shell2   = openShells.Value(j);
                if (!GetFreeEdges(shell2, mapEdges2))
                {
                    continue;
                }
                foreach (Shape key2 in mapEdges2)
                {
                    Edge edge2 = ShapeCasts.Edge(key2);
                    if (!aMapMultiConnectEdges.Contains(edge2))
                    {
                        continue;
                    }
                    if (!dire.Contains(edge2) && !reve.Contains(edge2))
                    {
                        continue;
                    }
                    isAddShell = true;
                    if ((edge2.Orientation == Forward && dire.Contains(edge2))
                        || (edge2.Orientation == Reversed && reve.Contains(edge2)))
                    {
                        isReversed = true;
                    }
                }

                if (!isAddShell)
                {
                    continue;
                }
                var b = new BrepBuilder();

                for (var expF21 = new Explorer(openShells.Value(j), ShapeType.Face); expF21.More(); expF21.Next())
                {
                    Shape face = expF21.Current();
                    if (isReversed)
                    {
                        face.Reverse();
                    }
                    b.Add(shell, face);
                }

                openShells.ChangeValue(i) = shell;
                openShells.Remove(j--);
            }
        }

        openShells.Append(newShells);
    }

    //=================================================================================================

    /// <summary>
    /// Fixes the orientation of the faces of <paramref name="shell"/>: faces are reversed so that every edge between two
    /// faces is met in both orientations; faces that can't be oriented with the others make further shells. With
    /// <paramref name="isAccountMultiConex"/>, faces sharing an edge with more than one other face are kept apart; with
    /// <paramref name="nonManifold"/>, one non-manifold shell may be made. Returns true if something was done; the
    /// replacement of the shell is recorded in the context.
    /// </summary>
    /// <remarks>It runs in a working overlay (see the class's remarks).</remarks>
    public bool FixFaceOrientation(in Shell shell, bool isAccountMultiConex = true, bool nonManifold = false)
    {
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(shell, Context());
        bool done = FixFaceOrientationCore(shell, isAccountMultiConex, nonManifold);
        Finish(overlay);
        return done;
    }

    private bool FixFaceOrientationCore(in Shell shell, bool isAccountMultiConex, bool nonManifold)
    {
        // myStatus = ShapeExtend::EncodeStatus (ShapeExtend_OK);
        bool done           = false;
        var  seqShells     = new NCollection_Sequence<Shape>();
        var  errFaces      = new NCollection_Sequence<Shape>(); // Compound of faces like to Mebiuce leaf.
        var  lface          = new NCollection_Sequence<Shape>();
        var  mapFaceShells = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        _shell = shell;
        _shape = shell;
        int numMultShell = 0;
        int nbF           = 0;
        var mapAdded     = new HashSet<Shape>(ShapeHasher.Instance);
        for (var iter = new Iterator(shell); iter.More(); iter.Next(), nbF++)
        {
            if (mapAdded.Add(iter.Value()))
            {
                lface.Append(iter.Value());
            }
        }
        if (lface.Length() < nbF)
        {
            done = true;
        }

        var mapEdgeFaces = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shell, ShapeType.Edge, ShapeType.Face, mapEdgeFaces);
        var  mapMultiConnectEdges = new HashSet<Shape>(ShapeHasher.Instance);
        bool isFreeBoundaries      = false;
        for (int k = 0; k < mapEdgeFaces.Count; k++)
        {
            int faceCount = mapEdgeFaces.ValueAt(k).Extent();
            if (!isFreeBoundaries && faceCount == 1)
            {
                Edge e = ShapeCasts.Edge(mapEdgeFaces.KeyAt(k));
                if (!Tool.Degenerated(e))
                {
                    isFreeBoundaries = true;
                }
            }
            // Finds multishared edges
            else if (isAccountMultiConex && faceCount > 2)
            {
                mapMultiConnectEdges.Add(mapEdgeFaces.KeyAt(k));
            }
        }
        if (Tool.IsClosed(_shell) ? isFreeBoundaries : !isFreeBoundaries)
        {
            _shell.Closed(!isFreeBoundaries);
            SendWarning(new Msg("FixAdvShell.FixClosedFlag.MSG0")); // Shell has incorrect flag isClosed
        }
        bool isGetShells = true;
        // Gets possible shells with taking in account of multiconnexity.
        while (isGetShells && lface.Length() != 0)
        {
            var tmpSeqShells = new NCollection_Sequence<Shape>();
            if (GetShells(lface, mapMultiConnectEdges, tmpSeqShells, mapFaceShells, errFaces))
            {
                done = true;
            }
            isGetShells = !tmpSeqShells.IsEmpty();
            if (isGetShells)
            {
                seqShells.Append(tmpSeqShells);
            }
        }
        if (!done)
        {
            done = (seqShells.Length() > 1);
        }
        bool isDone = false;
        if (lface.Length() > 0 && seqShells.Length() != 0)
        {
            for (int jj = 1; jj <= lface.Length(); jj++)
            {
                if (mapFaceShells.ContainsKey(lface.Value(jj)))
                {
                    mapFaceShells.Remove(lface.Value(jj));
                }
            }

            // Addition of faces having only multiconnexity boundary to shells having holes
            //  containing only the multiconnexity edges
            isDone = AddMultiConexityFaces(lface, mapMultiConnectEdges, seqShells, mapFaceShells, mapEdgeFaces,
                                            errFaces, nonManifold);
        }
        numMultShell = seqShells.Length();
        if (!errFaces.IsEmpty())
        {
            // if Shell contains of Mebius faces one shell will be created from each those face.
            var B = new BrepBuilder();
            B.MakeCompound(ref _errFaces);
            var compShells = new Compound();
            B.MakeCompound(ref compShells);
            for (int n = 1; n <= errFaces.Length(); n++)
            {
                B.Add(_errFaces, errFaces.Value(n));
            }
            if (numMultShell != 0)
            {
                if (numMultShell == 1)
                {
                    B.Add(compShells, seqShells.Value(1));
                    for (int n1 = 1; n1 <= errFaces.Length(); n1++)
                    {
                        var sh = new Shell();
                        B.MakeShell(ref sh);
                        B.Add(sh, errFaces.Value(n1));
                        B.Add(compShells, sh);
                    }
                    _shape = compShells;
                }
                else
                {
                    for (int i = 1; i <= seqShells.Length(); i++)
                    {
                        B.Add(compShells, seqShells.Value(i));
                    }
                    for (int n1 = 1; n1 <= errFaces.Length(); n1++)
                    {
                        var sh = new Shell();
                        B.MakeShell(ref sh);
                        B.Add(sh, errFaces.Value(n1));
                        B.Add(compShells, sh);
                    }
                    _shape = compShells;
                }
            }

            done     = true;
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.Fail);
            SendWarning(new Msg("FixAdvShell.FixOrientation.MSG20")); // Impossible to orient faces in shell, several shells created
            return true;
        }
        if (numMultShell > 1)
        {
            var openShells = new NCollection_Sequence<Shape>();
            for (int i1 = 1; i1 <= seqShells.Length(); i1++)
            {
                Shape aShell = seqShells.Value(i1);
                if (!Tool.IsClosed(aShell))
                {
                    openShells.Append(aShell);
                    seqShells.Remove(i1--);
                }
            }
            if (openShells.Length() > 1)
            {
                // Attempt of creation closed shell from open shells with taking into account multiconnexity.
                CreateClosedShell(openShells, mapMultiConnectEdges);
            }
            seqShells.Append(openShells);
        }

        // In the case if NonManifold is equal to true one non-manifold shell will be created.
        // Else compound from shells will be created if length of sequence of shape >1.
        if (lface.Length() != 0)
        {
            for (int i = 1; i <= lface.Length(); i++)
            {
                var aB       = new BrepBuilder();
                var oneShell = new Shell();
                aB.MakeShell(ref oneShell);
                aB.Add(oneShell, lface.Value(i));
                seqShells.Append(oneShell);
            }
        }
        if (nonManifold && seqShells.Length() > 1)
        {
            CreateNonManifoldShells(seqShells, mapMultiConnectEdges);
        }
        if (!done)
        {
            done = (seqShells.Length() > 1 || isDone);
        }
        if (seqShells.Length() == 1)
        {
            _shell    = ShapeCasts.Shell(seqShells.Value(1));
            _shape    = _shell;
            _nbShells = 1;
        }
        else
        {
            var b           = new BrepBuilder();
            var compShells = new Compound();
            b.MakeCompound(ref compShells);
            for (int i = 1; i <= seqShells.Length(); i++)
            {
                b.Add(compShells, seqShells.Value(i));
            }
            _shape    = compShells;
            _nbShells = seqShells.Length();
        }
        if (done)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            Context()?.Replace(shell, _shape);
            if (_nbShells == 1)
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
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the fixed shell (or the first of the subshells made), frozen.</summary>
    public Shell Shell() => ShapeCasts.Shell(Published(_shell));

    /// <summary>Returns the fixed shell, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shell InternalShell() => _shell;

    /// <summary>Returns the result: a shell, or a compound of the shells made, frozen.</summary>
    public Shape Shape() => Published(_shape);

    /// <summary>Returns the result, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>Returns the number of shells made.</summary>
    public int ShellCount => _nbShells;

    /// <summary>Returns the faces that couldn't be oriented (Moebius-like), as a compound, frozen.</summary>
    public Compound ErrorFaces() => ShapeCasts.Compound(Published(_errFaces));

    /// <summary>The faces that couldn't be oriented, unfrozen.</summary>
    internal Compound InternalErrorFaces() => _errFaces;

    /// <summary>Returns the face fixing tool.</summary>
    public FaceFixer FixFaceTool() => _fixFace;

    /// <summary>Sets the message registrator, for this tool and the face tool.</summary>
    public override void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        _fixFace.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and the face tool.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        _fixFace.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and the face tool.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        _fixFace.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and the face tool.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        _fixFace.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the face fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixFaceMode() => ref _fixFaceMode;

    /// <summary>The mode of the orientation fix (-1 default, 0 off, 1 on).</summary>
    public ref int FixOrientationMode() => ref _fixOrientationMode;

    /// <summary>Sets whether a non-manifold shell may be made.</summary>
    public virtual void SetNonManifoldFlag(bool isNonManifold)
    {
        _nonManifold = isNonManifold;
    }

    // The end of a fix in its own working overlay (none inside another operation's): the results with new TShapes for
    // what changed and the caller's own for the rest, and the context answering for the caller's shapes.
    private void Finish(WorkingOverlay? overlay)
    {
        if (overlay is null)
        {
            return;
        }
        _shape    = overlay.Finish(_shape, [_shell, _errFaces]);
        _shell    = ShapeCasts.Shell(overlay.Final(_shell));
        _errFaces = ShapeCasts.Compound(overlay.Final(_errFaces));
    }

    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
