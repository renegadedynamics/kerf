// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// Draw's commands that read the meshes shapes already carry: DBRep's <c>tclean</c>; MeshTest.cxx's <c>trinfo</c>,
/// <c>triepoints</c>, <c>trmergenodes</c> and <c>correctnormals</c>; MeshTest_PluginCommands.cxx's <c>triarea</c>,
/// <c>tricheck</c> (over <see cref="MeshTestCheckTopology"/>) and <c>mpparallel</c>.
/// </summary>
/// <remarks>
/// <para>
/// tclean, trmergenodes (without -oneFace) and correctnormals change their shape in place in OCCT; the port runs
/// OCCT's in-place algorithm on a copy of the graph (DrawShapeCopy) and binds the copy to the name (decision 480).
/// Other variables that hold sub-shapes of it keep the old TShapes.
/// </para>
/// <para>
/// Not here: <c>incmesh</c>, <c>tessellate</c> and the mesher's factory commands (<c>mpnames</c>,
/// <c>mpsetdefaultname</c>, <c>mpgetdefaultname</c>, <c>mpincmesh</c>) wait for BRepMesh (unit 4L2);
/// <c>trlateload</c> (deferred triangulations), <c>tri2d</c>, <c>veriftriangles</c> and <c>wavefront</c> aren't in
/// this unit.
/// </para>
/// </remarks>
internal static class MeshTestCommands
{
    /// <summary>
    /// mpparallel's flag: BRepMesh_IncrementalMesh's parallel default, which isn't ported yet (unit 4L2 makes the
    /// command set BRepMesh_IncrementalMesh's static). Kept per interpreter, so parallel corpus runs don't share it.
    /// </summary>
    private static readonly ConditionalWeakTable<DrawInterpreter, StrongBox<bool>> s_parallelDefault = new();

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "tclean", TClean);
        DrawCxx.Add(theInterpreter, "trinfo", TrianglesInfo);
        DrawCxx.Add(theInterpreter, "triepoints", TriEdgePoints);
        DrawCxx.Add(theInterpreter, "trmergenodes", TrMergeNodes);
        DrawCxx.Add(theInterpreter, "correctnormals", CorrectNormals);
        DrawCxx.Add(theInterpreter, "mpparallel", MpParallel);
        DrawCxx.Add(theInterpreter, "triarea", TriArea);
        DrawCxx.Add(theInterpreter, "tricheck", TriCheck);
    }

    /// <summary>Whether mpparallel has turned the mesher's parallel default on in this interpreter.</summary>
    internal static bool IsParallelDefault(DrawInterpreter theInterpreter)
        => s_parallelDefault.TryGetValue(theInterpreter, out StrongBox<bool>? aBox) && aBox.Value;

    // -----------------------------------------------------------------------------------------------------------------
    // DBRep.cxx

    /// <summary>
    /// tclean [-force] [-geom] name...: BRepTools::Clean (or CleanGeometry) on a compound of the shapes, which changes
    /// them in place in OCCT. The port cleans a copy of the compound and binds each name to its copy.
    /// </summary>
    private static int TClean(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        bool toRemoveGeometry = false;
        bool isForceClean = false;
        int n = a.Count;
        if (n <= 1)
        {
            // di.PrintHelp(a[0]): Draw-lite has no help texts
            return 1;
        }

        var aNames = new List<string>();
        var aShapes = new List<TopoDS_Shape>();
        for (int anArgIter = 1; anArgIter < n; ++anArgIter)
        {
            if (a[anArgIter] == "-geom")
            {
                toRemoveGeometry = true;
            }
            else if (a[anArgIter] == "-force")
            {
                isForceClean = true;
            }
            else
            {
                TopoDS_Shape aShape = theDI.GetShape(a[anArgIter]);
                if (aShape.IsNull())
                {
                    di.Append("Syntax error : NULL shape '").Append(a[anArgIter]).Append("'");
                    return 1;
                }

                aNames.Add(a[anArgIter]);
                aShapes.Add(aShape);
            }
        }

        if (aShapes.Count == 0)
        {
            // OCCT appends this, then PrintHelp's Tcl_Eval of "help tclean" replaces the result with help's (empty)
            return 1;
        }

        if (isForceClean && toRemoveGeometry)
        {
            di.Append("Syntax error: wrong usage of arguments: do not use 'force' and 'geom' flags together");
            return 1;
        }

        // in place in OCCT: on a copy of the graph here, one copy for all the shapes, so shared sub-shapes stay shared
        TopoDS_Shape aCompound = DrawShapeCopy.Copy(TopoDS_Builder.MakeCompound(aShapes));
        if (toRemoveGeometry)
        {
            BRepTools.CleanGeometry(aCompound);
        }
        else
        {
            BRepTools.Clean(aCompound, isForceClean);
        }

        int anIndex = 0;
        for (var anIter = new TopoDS_Iterator(aCompound); anIter.More(); anIter.Next())
        {
            theDI.SetShape(aNames[anIndex++], anIter.Value());
        }

        return 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // MeshTest.cxx

    private sealed class TriangulationStat
    {
        public readonly List<(string Type, int Count)> TypeMap = [];
        public int NbFaces;
        public int NbEmptyFaces;
        public int NbTriangles;
        public int NbDeferredFaces;
        public int NbUnloadedFaces;
        public int NbUnloadedTriangles;
    }

    /// <summary>trinfo shape [-lods]: the faces, triangles, nodes and polygons on triangulation, and the deflections.</summary>
    private static int TrianglesInfo(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        if (theArgVec.Count < 2)
        {
            theDI.Output.WriteLine("Syntax error: not enough arguments");
            return 1;
        }

        TopoDS_Shape aShape = theDI.GetShape(theArgVec[1]);
        if (aShape.IsNull())
        {
            theResult.Append(theArgVec[1]).Append(" is not a shape\n");
            return 1;
        }

        bool toPrintLODs = theArgVec.Count > 2 && theArgVec[2].Equals("-lods", StringComparison.OrdinalIgnoreCase);

        double aMaxDeflection = 0.0, aMeshingDefl = -1.0, aMeshingAngDefl = -1.0, aMeshingMinSize = -1.0;
        int aNbFaces = 0, aNbEmptyFaces = 0, aNbTriangles = 0, aNbNodes = 0, aNbRepresentations = 0;
        var aLODsStat = new List<TriangulationStat>(); // NCollection_IndexedDataMap<int, ...> keyed 0, 1, 2 ...
        var aNbLODs = new List<int>();
        for (var anExp = new TopExp_Explorer(aShape, TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            aNbFaces++;
            Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aFace, out _);
            if (aTriangulation is not null)
            {
                aNbTriangles += aTriangulation.NbTriangles();
                aNbNodes += aTriangulation.NbNodes();
                aMaxDeflection = Math.Max(aMaxDeflection, aTriangulation.Deflection());
                if (aTriangulation.Parameters() is { } aParams)
                {
                    aMeshingDefl = Math.Max(aMeshingDefl, aParams.Deflection());
                    aMeshingAngDefl = Math.Max(aMeshingAngDefl, aParams.Angle());
                    aMeshingMinSize = Math.Max(aMeshingMinSize, aParams.MinSize());
                }
            }
            else
            {
                aNbEmptyFaces++;
            }

            if (toPrintLODs)
            {
                // Collect LODs information
                NCollection_List<Poly_Triangulation> aLODs = BRep_Tool.Triangulations(aFace, out _);
                if (aLODs.Extent() != 0)
                {
                    aNbLODs.Add(aLODs.Extent());
                }

                int aTriangIndex = 0;
                for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(aLODs); anIter.More(); anIter.Next(), ++aTriangIndex)
                {
                    if (aTriangIndex == aLODsStat.Count)
                    {
                        aLODsStat.Add(new TriangulationStat());
                    }

                    TriangulationStat aStats = aLODsStat[aTriangIndex];
                    aStats.NbFaces++;
                    Poly_Triangulation? aLOD = anIter.Value();
                    if (aLOD is null)
                    {
                        aStats.NbEmptyFaces++;
                        continue;
                    }

                    string aType = aLOD.GetType().Name;
                    int aTypeIndex = aStats.TypeMap.FindIndex(t => t.Type == aType);
                    if (aTypeIndex < 0)
                    {
                        aStats.TypeMap.Add((aType, 1));
                    }
                    else
                    {
                        aStats.TypeMap[aTypeIndex] = (aType, aStats.TypeMap[aTypeIndex].Count + 1);
                    }

                    if (aLOD.HasDeferredData())
                    {
                        aStats.NbDeferredFaces++;
                        if (aLOD.HasGeometry())
                        {
                            aStats.NbTriangles += aLOD.NbTriangles();
                        }
                        else
                        {
                            aStats.NbUnloadedFaces++;
                            aStats.NbTriangles += aLOD.NbDeferredTriangles();
                            aStats.NbUnloadedTriangles += aLOD.NbDeferredTriangles();
                        }
                    }
                    else if (aLOD.HasGeometry())
                    {
                        aStats.NbTriangles += aLOD.NbTriangles();
                    }
                    else
                    {
                        aStats.NbEmptyFaces++;
                    }
                }
            }
        }

        var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aShape, TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
        for (int i = 1; i <= anEdges.Extent(); ++i)
        {
            var aTE = (BRep_TEdge)anEdges.FindKey(i).TShape()!;
            for (var anIterCR = new NCollection_List<BRep_CurveRepresentation>.Iterator(aTE.Curves()); anIterCR.More(); anIterCR.Next())
            {
                if (anIterCR.Value().IsPolygonOnTriangulation())
                {
                    aNbRepresentations++;
                }
            }
        }

        theResult.Append("\n");
        theResult.Append("This shape contains ").Append(aNbFaces).Append(" faces.\n");
        if (aNbEmptyFaces > 0)
        {
            theResult.Append("                    ").Append(aNbEmptyFaces).Append(" empty faces.\n");
        }

        theResult.Append("                    ").Append(aNbTriangles).Append(" triangles.\n");
        theResult.Append("                    ").Append(aNbNodes).Append(" nodes.\n");
        theResult.Append("                    ").Append(aNbRepresentations).Append(" polygons on triangulation.\n");
        theResult.Append("Maximal deflection ").Append(aMaxDeflection).Append("\n");
        if (aMeshingDefl > 0.0)
        {
            theResult.Append("Meshing deflection ").Append(aMeshingDefl).Append("\n");
        }

        if (aMeshingAngDefl > 0.0)
        {
            theResult.Append("Meshing angular deflection ").Append(aMeshingAngDefl * 180.0 / Math.PI).Append("\n");
        }

        if (aMeshingMinSize > 0.0)
        {
            theResult.Append("Meshing min size ").Append(aMeshingMinSize).Append("\n");
        }

        if (aNbLODs.Count > 0)
        {
            // Find all different numbers of triangulation LODs and their average value per face
            aNbLODs.Sort();
            List<int> aLODsRange = aNbLODs.Distinct().ToList();
            theResult.Append("Number of triangulation LODs [" + string.Join(" ", aLODsRange) + "]\n");
            if (aLODsRange.Count > 1)
            {
                // Find average number of triangulation LODs per face
                int aMedian = aNbLODs[aNbLODs.Count / 2];
                if (aNbLODs.Count % 2 == 0)
                {
                    aMedian += aNbLODs[aNbLODs.Count / 2 - 1];
                    aMedian /= 2;
                }

                theResult.Append("                             [average per face: " + aMedian + "]\n");
            }
        }

        if (aLODsStat.Count > 0)
        {
            var aLODsStatStr = new System.Text.StringBuilder();
            for (int aKey = 0; aKey < aLODsStat.Count; aKey++)
            {
                TriangulationStat aLodStat = aLODsStat[aKey];
                aLODsStatStr.Append("LOD #").Append(aKey).Append(". ");
                if (aLodStat.NbEmptyFaces > 0 || aLodStat.NbFaces < aNbFaces)
                {
                    int aNbEmpty = aLodStat.NbEmptyFaces + (aNbFaces - aLodStat.NbFaces);
                    aLODsStatStr.Append("NbEmpty: ").Append(aNbEmpty).Append(", ");
                }

                aLODsStatStr.Append("NbTris: ").Append(aLodStat.NbTriangles);
                if (aLodStat.NbDeferredFaces > 0)
                {
                    aLODsStatStr.Append(", NbDeferred: ").Append(aLodStat.NbDeferredFaces);
                    if (aLodStat.NbUnloadedFaces > 0)
                    {
                        aLODsStatStr.Append(", NbUnloaded: ").Append(aLodStat.NbUnloadedFaces)
                                    .Append(", NbUnloadedTris: ").Append(aLodStat.NbUnloadedTriangles);
                    }
                }

                aLODsStatStr.Append(".\n");

                // Add types
                aLODsStatStr.Append("        Types: ");
                aLODsStatStr.Append(string.Join(", ", aLodStat.TypeMap.Select(t => $"{t.Type} ({t.Count})")));
                aLODsStatStr.Append(".\n");
            }

            theResult.Append(aLODsStatStr.ToString());
        }

        theResult.Append("\n");
        return 0;
    }

    /// <summary>
    /// triepoints shape...: binds a vertex at each node of the edges' polygons on triangulation, named
    /// <c>shape_edge_node</c> (<c>shape_node</c> for one edge), and returns the names.
    /// </summary>
    /// <remarks>
    /// OCCT collects the edges in an NCollection_Map, whose iteration follows the TShapes' addresses; the port takes
    /// them in the order TopExp_Explorer meets them (decision 18), so the edge numbers in the names can differ.
    /// </remarks>
    private static int TriEdgePoints(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int nbarg = argv.Count;
        if (nbarg < 2)
        {
            return 1;
        }

        for (int i = 1; i < nbarg; i++)
        {
            TopoDS_Shape aShape = theDI.GetShape(argv[i]);
            if (aShape.IsNull())
            {
                continue;
            }

            var anEdgeSet = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var anEdgeMap = new List<TopoDS_Shape>();
            if (aShape.ShapeType() == TopAbs_ShapeEnum.TopAbs_EDGE)
            {
                anEdgeSet.Add(aShape);
                anEdgeMap.Add(aShape);
            }
            else
            {
                for (var ex = new TopExp_Explorer(aShape, TopAbs_ShapeEnum.TopAbs_EDGE); ex.More(); ex.Next())
                {
                    if (anEdgeSet.Add(ex.Current()))
                    {
                        anEdgeMap.Add(ex.Current());
                    }
                }
            }

            if (anEdgeMap.Count == 0)
            {
                continue;
            }

            string aPrefix = argv[i] + "_";
            int nbEdge = 1;
            foreach (TopoDS_Shape anEdge in anEdgeMap)
            {
                BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(anEdge), out Poly_PolygonOnTriangulation? aPoly, out Poly_Triangulation? aT, out TopLoc_Location aLoc);
                if (aT is null || aPoly is null)
                {
                    continue;
                }

                NCollection_Array1<int> Indices = aPoly.Nodes();
                int nbnodes = Indices.Length();
                for (int j = 1; j <= nbnodes; j++)
                {
                    gp_Pnt P3d = aT.Node(Indices[j]);
                    if (!aLoc.IsIdentity())
                    {
                        P3d = P3d.Transformed(aLoc.Transformation());
                    }

                    string newname = anEdgeMap.Count > 1 ? $"{aPrefix}{nbEdge}_{j}" : $"{aPrefix}{j}";
                    theDI.SetShape(newname, new BRepBuilderAPI_MakeVertex(P3d).Shape());
                    di.AppendElement(newname);
                }

                nbEdge++;
            }
        }

        return 0;
    }

    /// <summary>
    /// trmergenodes shape [-angle A] [-tolerance T] [-force] [-oneFace result]: Poly_MergeNodesTool on each face's
    /// triangulation (in place in OCCT: on a copy here), or on all of them into one new face.
    /// </summary>
    private static int TrMergeNodes(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theNbArgs = theArgVec.Count;
        if (theNbArgs < 2)
        {
            theResult.Append("Syntax error: not enough arguments");
            return 1;
        }

        TopoDS_Shape aShape = theDI.GetShape(theArgVec[1]);
        if (aShape.IsNull())
        {
            theResult.Append("Syntax error: '").Append(theArgVec[1]).Append("' is not a shape");
            return 1;
        }

        double aMergeAngle = Math.PI / 4.0, aMergeToler = 0.0;
        bool toForce = false;
        string aResFace = "";
        for (int anArgIter = 2; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = theArgVec[anArgIter].ToLowerInvariant();
            if (anArgIter + 1 < theNbArgs
                && anArgCase is "-angle" or "-smoothangle" or "-mergeangle"
                && theDI.ParseReal(theArgVec[anArgIter + 1], out double anAngle))
            {
                aMergeAngle = anAngle;
                if (aMergeAngle < 0.0 || aMergeAngle > 90.0)
                {
                    theResult.Append("Syntax error: angle should be within [0,90] range");
                    return 1;
                }

                ++anArgIter;
                aMergeAngle = aMergeAngle * Math.PI / 180.0;
            }
            else if (anArgIter + 1 < theNbArgs && anArgCase == "-tolerance"
                     && theDI.ParseReal(theArgVec[anArgIter + 1], out double aToler))
            {
                aMergeToler = aToler;
                if (aMergeToler < 0.0)
                {
                    theResult.Append("Syntax error: tolerance should be within >=0");
                    return 1;
                }

                ++anArgIter;
            }
            else if (anArgCase == "-force")
            {
                toForce = DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (anArgIter + 1 < theNbArgs && anArgCase == "-oneface")
            {
                aResFace = theArgVec[++anArgIter];
            }
            else
            {
                theResult.Append("Syntax error at '").Append(theArgVec[anArgIter]).Append("'");
                return 1;
            }
        }

        int aNbNodesOld = 0, aNbTrisOld = 0;
        int aNbNodesNew = 0, aNbTrisNew = 0;
        if (aResFace.Length > 0)
        {
            var aMergeTool = new Poly_MergeNodesTool(aMergeAngle, aMergeToler);
            for (var aFaceIter = new TopExp_Explorer(aShape, TopAbs_ShapeEnum.TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(aFaceIter.Value());
                Poly_Triangulation? aTris = BRep_Tool.Triangulation(aFace, out TopLoc_Location aFaceLoc);
                if (aTris is null || aTris.NbNodes() < 3 || aTris.NbTriangles() < 1)
                {
                    continue;
                }

                aNbNodesOld += aTris.NbNodes();
                aNbTrisOld += aTris.NbTriangles();
                aMergeTool.AddTriangulation(aTris, aFaceLoc.Transformation(), aFace.Orientation() == TopAbs_Orientation.TopAbs_REVERSED);
            }

            Poly_Triangulation? aNewTris = aMergeTool.Result();
            if (aNewTris is null)
            {
                theResult.Append("Error: empty result");
                return 0;
            }

            aNbNodesNew += aNewTris.NbNodes();
            aNbTrisNew += aNewTris.NbTriangles();
            var aFace1 = new TopoDS_Face();
            new BRep_Builder().MakeFace(ref aFace1, aNewTris);
            theDI.SetShape(aResFace, aFace1);
        }
        else
        {
            // in place in OCCT: on a copy of the graph here, bound when a face changed
            TopoDS_Shape aCopy = DrawShapeCopy.Copy(aShape);
            bool isChanged = false;
            var aProcessedFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            for (var aFaceIter = new TopExp_Explorer(aCopy, TopAbs_ShapeEnum.TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(aFaceIter.Value());
                if (!aProcessedFaces.Add(aFace.Located(new TopLoc_Location())))
                {
                    continue;
                }

                Poly_Triangulation? aTris = BRep_Tool.Triangulation(aFace, out _);
                if (aTris is null || aTris.NbNodes() < 3 || aTris.NbTriangles() < 1)
                {
                    continue;
                }

                aNbNodesOld += aTris.NbNodes();
                aNbTrisOld += aTris.NbTriangles();
                var aMergeTool = new Poly_MergeNodesTool(aMergeAngle, aMergeToler, aTris.NbTriangles());
                aMergeTool.AddTriangulation(aTris);
                if (toForce || aMergeTool.NbNodes() != aTris.NbNodes() || aMergeTool.NbElements() != aTris.NbTriangles())
                {
                    new BRep_Builder().UpdateFace(aFace, aMergeTool.Result(), false);
                    isChanged = true;
                }

                aTris = BRep_Tool.Triangulation(aFace, out _)!;
                aNbNodesNew += aTris.NbNodes();
                aNbTrisNew += aTris.NbTriangles();
            }

            if (isChanged)
            {
                theDI.SetShape(theArgVec[1], aCopy);
            }
        }

        theResult.Append("Old, Triangles: ").Append(aNbTrisOld).Append(", Nodes: ").Append(aNbNodesOld).Append("\n");
        theResult.Append("New, Triangles: ").Append(aNbTrisNew).Append(", Nodes: ").Append(aNbNodesNew).Append("\n");
        return 0;
    }

    /// <summary>
    /// correctnormals shape: BRepLib::EnsureNormalConsistency, in place in OCCT; on a copy here, bound when it made
    /// corrections.
    /// </summary>
    private static int CorrectNormals(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        // OCCT reads argv[1] without checking the count (Tcl's null argv[argc]); an empty word here
        string aName = theArgVal.Count > 1 ? theArgVal[1] : "";
        TopoDS_Shape S = theDI.GetShape(aName);
        TopoDS_Shape aCopy = DrawShapeCopy.Copy(S);

        // OCCT writes the normals into the attached triangulations, which leaves the faces' flags as they are; the
        // port attaches corrected copies with BRep_Builder::UpdateFace, which marks the faces modified and unchecked.
        // The faces get their flags back, so the shape writes OCCT's text.
        var aFlags = new List<(TopoDS_TShape Face, bool Modified, bool Checked)>();
        for (var anExp = new TopExp_Explorer(aCopy, TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_TShape aTFace = anExp.Current().TShape()!;
            aFlags.Add((aTFace, aTFace.Modified(), aTFace.Checked()));
        }

        // OCCT bug: EnsureNormalConsistency reads UV nodes a triangulation may not have (after trmergenodes); the port
        // throws Standard_OutOfRange there (occt-issues.md)
        bool isCorrected = BRepLib.EnsureNormalConsistency(aCopy);
        foreach ((TopoDS_TShape aTFace, bool aModified, bool aChecked) in aFlags)
        {
            aTFace.Modified(aModified);
            aTFace.Checked(aChecked);
        }

        if (!isCorrected)
        {
            theResult.Append("Normals have not been changed!\n");
        }
        else
        {
            theDI.SetShape(aName, aCopy);
            theResult.Append("Some corrections in source shape have been made!\n");
        }

        return 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // MeshTest_PluginCommands.cxx

    /// <summary>mpparallel [toTurnOn]: shows or sets the incremental mesher's parallel default.</summary>
    private static int MpParallel(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        StrongBox<bool> aDefault = s_parallelDefault.GetOrCreateValue(theDI);
        if (argv.Count == 2)
        {
            aDefault.Value = theDI.Atoi(argv[1]) == 1;
        }

        theDI.Output.Write("Incremental Mesh, multi-threading " + (aDefault.Value ? "ON\n" : "OFF\n"));
        return 0;
    }

    /// <summary>triarea shape [eps]: the area of the triangles and the area of the surfaces (BRepGProp).</summary>
    private static int TriArea(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        TopoDS_Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull())
        {
            return 1;
        }

        double anEps = -1.0;
        if (n > 2)
        {
            anEps = theDI.Atof(a[2]);
        }

        var aMapF = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(shape, TopAbs_ShapeEnum.TopAbs_FACE, aMapF);

        // detect if a shape has triangulation
        bool hasPoly = false;
        for (int i = 1; i <= aMapF.Extent(); i++)
        {
            if (BRep_Tool.Triangulation(TopoDS.Face(aMapF.FindKey(i)), out _) is not null)
            {
                hasPoly = true;
                break;
            }
        }

        // compute area by triangles
        double aTriArea = 0;
        if (hasPoly)
        {
            for (int i = 1; i <= aMapF.Extent(); i++)
            {
                Poly_Triangulation? aPoly = BRep_Tool.Triangulation(TopoDS.Face(aMapF.FindKey(i)), out _);
                if (aPoly is null)
                {
                    theDI.Output.Write("face " + i + " has no triangulation\n");
                    continue;
                }

                for (int j = 1; j <= aPoly.NbTriangles(); j++)
                {
                    aPoly.Triangle(j).Get(out int n1, out int n2, out int n3);
                    gp_Pnt p1 = aPoly.Node(n1);
                    gp_Pnt p2 = aPoly.Node(n2);
                    gp_Pnt p3 = aPoly.Node(n3);
                    var v1 = new gp_Vec(p1, p2);
                    var v2 = new gp_Vec(p1, p3);
                    aTriArea += v1.CrossMagnitude(v2);
                }
            }

            aTriArea /= 2;
        }

        // compute area by geometry
        var props = new GProp_GProps();
        if (anEps <= 0.0)
        {
            BRepGProp.SurfaceProperties(shape, props);
        }
        else
        {
            BRepGProp.SurfaceProperties(shape, props, anEps);
        }

        double aGeomArea = props.Mass();
        di.Append(aTriArea).Append(" ").Append(aGeomArea).Append("\n");
        return 0;
    }

    /// <summary>
    /// tricheck shape [-small]: MeshTest_CheckTopology's free links, cross-face errors, asynchronous edges, free
    /// nodes and (with -small) null-area triangles, then the faces whose mesh isn't connected inside.
    /// </summary>
    /// <remarks>
    /// OCCT also draws the faulty links, nodes and triangles (DrawTrSurf::Set with the name ".", which only displays
    /// them); Draw-lite has no viewer, so they aren't made. The links of "Not connected mesh inside face" are listed
    /// in the order they were first met, where OCCT iterates an NCollection_Map in bucket order (decision 18).
    /// </remarks>
    private static int TriCheck(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        TopoDS_Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull())
        {
            return 1;
        }

        bool isToFindSmallTriangles = n >= 3 && a[2] == "-small";

        // execute check
        var aCheck = new MeshTestCheckTopology(shape);
        aCheck.Perform(di);

        // dump info on free links inside the triangulation
        int nbFree = 0;
        int nbFac = aCheck.NbFacesWithFL();
        for (int k = 1; k <= nbFac; k++)
        {
            int nbEdge = aCheck.NbFreeLinks(k);
            int iF = aCheck.GetFaceNumWithFL(k);
            nbFree += nbEdge;
            di.Append("free links of face ").Append(iF).Append("\n");
            for (int i = 1; i <= nbEdge; i++)
            {
                aCheck.GetFreeLink(k, i, out int n1, out int n2);
                di.Append("{").Append(n1).Append(" ").Append(n2).Append("} ");
            }

            di.Append("\n");
        }

        // dump info on cross face errors
        int nbErr = aCheck.NbCrossFaceErrors();
        if (nbErr > 0)
        {
            di.Append("cross face errors: {face1, node1, face2, node2, distance}\n");
            for (int i = 1; i <= nbErr; i++)
            {
                aCheck.GetCrossFaceError(i, out int iF1, out int n1, out int iF2, out int n2, out double aVal);
                di.Append("{").Append(iF1).Append(" ").Append(n1).Append(" ").Append(iF2).Append(" ").Append(n2).Append(" ")
                  .Append(aVal).Append("} ");
            }

            di.Append("\n");
        }

        // dump info on edges
        int nbAsync = aCheck.NbAsyncEdges();
        if (nbAsync > 0)
        {
            di.Append("async edges:\n");
            for (int i = 1; i <= nbAsync; i++)
            {
                di.Append(aCheck.GetAsyncEdgeNum(i)).Append(" ");
            }

            di.Append("\n");
        }

        // dump info on free nodes
        int nbFreeNodes = aCheck.NbFreeNodes();
        if (nbFreeNodes > 0)
        {
            di.Append("free nodes (in pairs: face / node): \n");
            for (int i = 1; i <= nbFreeNodes; i++)
            {
                aCheck.GetFreeNodeNum(i, out int iface, out int inode);
                di.Append("{").Append(iface).Append(" ").Append(inode).Append("} ");
            }

            di.Append("\n");
        }

        int aNbSmallTriangles = isToFindSmallTriangles ? aCheck.NbSmallTriangles() : 0;
        if (aNbSmallTriangles > 0)
        {
            di.Append("triangles with null area (in pairs: face / triangle): \n");
            for (int i = 1; i <= aNbSmallTriangles; i++)
            {
                aCheck.GetSmallTriangle(i, out int aFaceId, out int aTriID);
                di.Append("{").Append(aFaceId).Append(" ").Append(aTriID).Append("} ");
            }

            di.Append("\n");
        }

        // output errors summary to DRAW
        if (nbFree > 0 || nbErr > 0 || nbAsync > 0 || nbFreeNodes > 0 || aNbSmallTriangles > 0)
        {
            di.Append("Free_links ").Append(nbFree).Append(" Cross_face_errors ").Append(nbErr).Append(" Async_edges ").Append(nbAsync)
              .Append(" Free_nodes ").Append(nbFreeNodes).Append(" Small triangles ").Append(aNbSmallTriangles).Append("\n");
        }

        int aFaceIndex = 1;
        for (var aFaceExp = new TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE); aFaceExp.More(); aFaceExp.Next(), ++aFaceIndex)
        {
            TopoDS_Shape aShape = aFaceExp.Current();
            TopoDS_Face aFace = TopoDS.Face(aShape);
            Poly_Triangulation? aT = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLoc);

            // Iterate boundary edges (BRepMesh_Edge: equal in either direction)
            var aBoundaryEdgeMap = new HashSet<(int, int)>();
            for (var anExp = new TopExp_Explorer(aShape, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                Poly_PolygonOnTriangulation? aPoly = BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(anExp.Current()), aT, aLoc);
                if (aPoly is null)
                {
                    continue;
                }

                NCollection_Array1<int> anIndices = aPoly.Nodes();
                int aPrevNode = -1;
                for (int j = anIndices.Lower(); j <= anIndices.Upper(); ++j)
                {
                    int aNodeIdx = anIndices.Value(j);
                    if (j != anIndices.Lower())
                    {
                        aBoundaryEdgeMap.Add(Link(aPrevNode, aNodeIdx));
                    }

                    aPrevNode = aNodeIdx;
                }
            }

            // OCCT bug: it stops at the first face without boundary polygons; the faces after it aren't checked
            // (occt-issues.md)
            if (aBoundaryEdgeMap.Count == 0)
            {
                break;
            }

            // the free links, each kept with the direction it was added in, in the order they were added
            var aFreeEdgeMap = new Dictionary<(int, int), long>();
            var aFreeEdges = new SortedDictionary<long, (int First, int Last)>();
            long anOrder = 0;
            int aTriNum = aT!.NbTriangles();
            for (int aTriIndx = 1; aTriIndx <= aTriNum; aTriIndx++)
            {
                Poly_Triangle aTri = aT.Triangle(aTriIndx);
                int[] aTriNodes = [aTri.Value(1), aTri.Value(2), aTri.Value(3)];
                for (int j = 1; j <= 3; ++j)
                {
                    int aLastId = aTriNodes[j % 3];
                    int aFirstId = aTriNodes[j - 1];
                    (int, int) aLink = Link(aFirstId, aLastId);
                    if (!aBoundaryEdgeMap.Contains(aLink))
                    {
                        if (aFreeEdgeMap.Remove(aLink, out long anAdded))
                        {
                            aFreeEdges.Remove(anAdded);
                        }
                        else
                        {
                            aFreeEdgeMap.Add(aLink, anOrder);
                            aFreeEdges.Add(anOrder++, (aFirstId, aLastId));
                        }
                    }
                }
            }

            if (aFreeEdges.Count != 0)
            {
                di.Append("Not connected mesh inside face ").Append(aFaceIndex).Append("\n");
                foreach ((int aFirst, int aLast) in aFreeEdges.Values)
                {
                    di.Append("{").Append(aFirst).Append(" ").Append(aLast).Append("} ");
                }

                di.Append("\n");
            }
        }

        return 0;
    }

    private static (int, int) Link(int theNode1, int theNode2) => theNode1 < theNode2 ? (theNode1, theNode2) : (theNode2, theNode1);
}
