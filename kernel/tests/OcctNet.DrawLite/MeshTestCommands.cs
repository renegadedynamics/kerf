// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// Draw's mesh commands: DBRep's <c>tclean</c>; MeshTest.cxx's <c>incmesh</c>, <c>tessellate</c>, <c>trinfo</c>,
/// <c>triepoints</c>, <c>trmergenodes</c> and <c>correctnormals</c>; MeshTest_PluginCommands.cxx's <c>mpnames</c>,
/// <c>mpsetdefaultname</c>, <c>mpgetdefaultname</c>, <c>mpincmesh</c>, <c>mpparallel</c>, <c>triarea</c> and
/// <c>tricheck</c> (over <see cref="MeshTestCheckTopology"/>).
/// </summary>
/// <remarks>
/// <para>
/// incmesh, mpincmesh, tclean, trmergenodes (without -oneFace) and correctnormals change their shape in place in
/// OCCT; the port meshes a frozen shape on a copy (BRepMesh_IncrementalMesh, decision 489) and runs the other
/// in-place algorithms on a copy of the graph (DrawShapeCopy, decision 480). <see cref="DrawShapeRebind"/> then gives
/// the copy's TShapes to every variable that shares the changed ones, as the in-place change does in DRAWEXE
/// (decision 495).
/// </para>
/// <para>
/// mpparallel and mpsetdefaultname set process-wide state, as in OCCT (decision 492): BRepMesh_IncrementalMesh's
/// parallel default and BRepMesh_DiscretFactory's default name.
/// </para>
/// <para>
/// Not here: <c>trlateload</c> (deferred triangulations), <c>tri2d</c>, <c>veriftriangles</c>, <c>wavefront</c> and
/// <c>MemLeakTest</c>.
/// </para>
/// </remarks>
internal static class MeshTestCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "tclean", TClean);
        DrawCxx.Add(theInterpreter, "incmesh", IncrementalMesh);
        DrawCxx.Add(theInterpreter, "tessellate", Tessellate);
        DrawCxx.Add(theInterpreter, "mpnames", MpNames);
        DrawCxx.Add(theInterpreter, "mpsetdefaultname", MpSetDefaultName);
        DrawCxx.Add(theInterpreter, "mpgetdefaultname", MpGetDefaultName);
        DrawCxx.Add(theInterpreter, "mpincmesh", MpIncMesh);
        DrawCxx.Add(theInterpreter, "trinfo", TrianglesInfo);
        DrawCxx.Add(theInterpreter, "triepoints", TriEdgePoints);
        DrawCxx.Add(theInterpreter, "trmergenodes", TrMergeNodes);
        DrawCxx.Add(theInterpreter, "correctnormals", CorrectNormals);
        DrawCxx.Add(theInterpreter, "mpparallel", MpParallel);
        DrawCxx.Add(theInterpreter, "triarea", TriArea);
        DrawCxx.Add(theInterpreter, "tricheck", TriCheck);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // DBRep.cxx

    /// <summary>
    /// tclean [-force] [-geom] name...: BRepTools::Clean (or CleanGeometry) on a compound of the shapes, which changes
    /// them in place in OCCT. The port cleans a copy of the compound and rebinds the variables that share its
    /// sub-shapes to the copy's (decision 495).
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

        var aShapes = new List<Shape>();
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
                Shape aShape = theDI.GetShape(a[anArgIter]);
                if (aShape.IsNull)
                {
                    di.Append("Syntax error : NULL shape '").Append(a[anArgIter]).Append("'");
                    return 1;
                }

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
        Shape anOriginal = global::Kerf.Engine.Topology.Builder.MakeCompound(aShapes);
        Shape aCompound = DrawShapeCopy.Copy(anOriginal);
        if (toRemoveGeometry)
        {
            BrepTools.CleanGeometry(aCompound);
        }
        else
        {
            BrepTools.Clean(aCompound, isForceClean);
        }

        DrawShapeRebind.Rebind(theDI, anOriginal, aCompound);
        return 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // MeshTest.cxx

    /// <summary>
    /// incmesh shape... deflection [-angular A] [-prs] [-relative {0|1}] [-parallel {0|1}] [-min size]
    /// [-algo {watson|delabella}] [-di V] [-ai A] [-int_vert_off] [-surf_def_off] [-surf_def_all] [-adjust_min]
    /// [-force_face_def] [-decrease]: BRepMesh_IncrementalMesh with these parameters, on a compound of the shapes when
    /// there are several. OCCT meshes the shapes in place; the port meshes a copy (decision 489) and rebinds every
    /// variable that shares the meshed sub-shapes (decision 495).
    /// </summary>
    private static int IncrementalMesh(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        int theNbArgs = theArgVec.Count;
        if (theNbArgs < 3)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        var aListOfShapes = new List<Shape>();
        var aMeshParams = new IMeshTools_Parameters();
        bool hasDefl = false, hasAngDefl = false, isPrsDefl = false;

        IMeshTools_Context aContext = new BRepMesh_Context();
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string aNameCase = theArgVec[anArgIter].ToLowerInvariant();
            if (aNameCase is "-relative" or "-norelative")
            {
                aMeshParams.Relative = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase is "-parallel" or "-noparallel")
            {
                aMeshParams.InParallel = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-int_vert_off")
            {
                aMeshParams.InternalVerticesMode = !DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-surf_def_off")
            {
                aMeshParams.ControlSurfaceDeflection = !DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-surf_def_all")
            {
                aMeshParams.EnableControlSurfaceDeflectionAllSurfaces = DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-adjust_min")
            {
                aMeshParams.AdjustMinSize = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-force_face_def")
            {
                aMeshParams.ForceFaceDeflection = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-decrease")
            {
                aMeshParams.AllowQualityDecrease = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter);
            }
            else if (aNameCase == "-algo" && anArgIter + 1 < theNbArgs)
            {
                string anAlgoStr = theArgVec[++anArgIter].ToLowerInvariant();
                if (anAlgoStr is "watson" or "0")
                {
                    aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType.IMeshTools_MeshAlgoType_Watson;
                    aContext.SetFaceDiscret(new BRepMesh_FaceDiscret(new BRepMesh_MeshAlgoFactory()));
                }
                else if (anAlgoStr is "delabella" or "1")
                {
                    aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType.IMeshTools_MeshAlgoType_Delabella;
                    aContext.SetFaceDiscret(new BRepMesh_FaceDiscret(new BRepMesh_DelabellaMeshAlgoFactory()));
                }
                else if (anAlgoStr is "-1" or "default")
                {
                    // already handled by BRepMesh_Context constructor
                }
                else
                {
                    di.Append("Syntax error at '").Append(anAlgoStr).Append("'");
                    return 1;
                }
            }
            else if (aNameCase is "-prs" or "-presentation" or "-vis" or "-visualization" && !isPrsDefl)
            {
                isPrsDefl = true;
            }
            else if (aNameCase is "-angular" or "-angdefl" or "-angulardeflection" or "-a" && anArgIter + 1 < theNbArgs)
            {
                double aVal = theDI.Atof(theArgVec[++anArgIter]) * Math.PI / 180.0;
                if (aVal <= global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    di.Append("Syntax error: invalid input parameter '").Append(theArgVec[anArgIter]).Append("'");
                    return 1;
                }

                aMeshParams.Angle = aVal;
                hasAngDefl = true;
            }
            else if (aNameCase == "-ai" && anArgIter + 1 < theNbArgs)
            {
                double aVal = theDI.Atof(theArgVec[++anArgIter]) * Math.PI / 180.0;
                if (aVal <= global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    di.Append("Syntax error: invalid input parameter '").Append(theArgVec[anArgIter]).Append("'");
                    return 1;
                }

                aMeshParams.AngleInterior = aVal;
            }
            else if (aNameCase == "-min" && anArgIter + 1 < theNbArgs)
            {
                double aVal = theDI.Atof(theArgVec[++anArgIter]);
                if (aVal <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    di.Append("Syntax error: invalid input parameter '").Append(theArgVec[anArgIter]).Append("'");
                    return 1;
                }

                aMeshParams.MinSize = aVal;
            }
            else if (aNameCase == "-di" && anArgIter + 1 < theNbArgs)
            {
                double aVal = theDI.Atof(theArgVec[++anArgIter]);
                if (aVal <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    di.Append("Syntax error: invalid input parameter '").Append(theArgVec[anArgIter]).Append("'");
                    return 1;
                }

                aMeshParams.DeflectionInterior = aVal;
            }
            else if (IsFullRealValue(aNameCase) && !hasDefl)
            {
                aMeshParams.Deflection = Math.Max(theDI.Atof(theArgVec[anArgIter]), global::Kerf.Engine.Foundation.Precision.Confusion());
                if (aMeshParams.DeflectionInterior < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    aMeshParams.DeflectionInterior = aMeshParams.Deflection;
                }

                hasDefl = true;
            }
            else
            {
                Shape anArgShape = theDI.GetShape(theArgVec[anArgIter]);
                if (anArgShape.IsNull)
                {
                    di.Append("Syntax error: null shapes are not allowed here '").Append(theArgVec[anArgIter]).Append("'\n");
                    return 1;
                }

                aListOfShapes.Add(anArgShape);
            }
        }

        if (aListOfShapes.Count == 0)
        {
            theDI.Output.WriteLine("Syntax error: wrong number of arguments.");
            return 1;
        }

        di.Append("Incremental Mesh, multi-threading ").Append(aMeshParams.InParallel ? "ON" : "OFF").Append("\n");

        Shape aShape = aListOfShapes.Count == 1 ? aListOfShapes[0] : global::Kerf.Engine.Topology.Builder.MakeCompound(aListOfShapes);
        if (isPrsDefl)
        {
            var aDrawer = new Prs3d_Drawer();
            if (hasDefl)
            {
                aDrawer.SetDeviationCoefficient(aMeshParams.Deflection);
            }

            aMeshParams.Deflection = StdPrs_ToolTriangulatedShape.GetDeflection(aShape, aDrawer);
            if (!hasAngDefl)
            {
                aMeshParams.Angle = aDrawer.DeviationAngle();
            }
        }

        var aMesher = new BRepMesh_IncrementalMesh();
        aMesher.SetShape(aShape);
        aMesher.ChangeParameters() = aMeshParams;
        aMesher.Perform(aContext);

        // OCCT meshes the shapes in place: the meshed copy, for every variable that shares their sub-shapes
        DrawShapeRebind.Rebind(theDI, aShape, aMesher.Shape());

        di.Append("Meshing statuses: ");
        int aStatus = aMesher.GetStatusFlags();
        if (aStatus == 0)
        {
            di.Append("NoError");
            return 0;
        }

        for (int i = 0; i < 9; i++)
        {
            int aFlag = aStatus & (1 << i);
            if (aFlag != 0)
            {
                di.Append((IMeshData_Status)aFlag switch
                {
                    IMeshData_Status.IMeshData_OpenWire => "OpenWire ",
                    IMeshData_Status.IMeshData_SelfIntersectingWire => "SelfIntersectingWire ",
                    IMeshData_Status.IMeshData_Failure => "Failure ",
                    IMeshData_Status.IMeshData_ReMesh => "ReMesh ",
                    IMeshData_Status.IMeshData_UnorientedWire => "UnorientedWire ",
                    IMeshData_Status.IMeshData_TooFewPoints => "TooFewPoints ",
                    IMeshData_Status.IMeshData_Outdated => "Outdated ",
                    IMeshData_Status.IMeshData_Reused => "Reused ",
                    IMeshData_Status.IMeshData_UserBreak => "UserBreak ",
                    _ => "",
                });
            }
        }

        return 0;
    }

    /// <summary>TCollection_AsciiString::IsRealValue(true): strtod reads the whole text.</summary>
    private static bool IsFullRealValue(string theText)
    {
        global::Kerf.Engine.Foundation.StreamUtilities.Strtod(theText, out int anEnd);
        return anEnd == theText.Length;
    }

    /// <summary>
    /// tessellate result {surface|face} nbu nbv: a face with natural bounds on the surface (or the face's surface
    /// within its UV bounds) carrying a regular triangulation of 2*nbu*nbv triangles, with polygons on its edges.
    /// </summary>
    private static int Tessellate(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 5)
        {
            theDI.Output.WriteLine("Builds regular triangulation with specified number of triangles\n"
                                   + "    Usage: tessellate result {surface|face} nbu nbv\n"
                                   + "    Triangulation is put into the face with natural bounds (result);\n"
                                   + "    it will have 2*nbu*nbv triangles and (nbu+1)*(nbv+1) nodes");
            return 1;
        }

        string aResName = argv[1];
        string aSrcName = argv[2];
        int aNbU = theDI.Atoi(argv[3]);
        int aNbV = theDI.Atoi(argv[4]);

        if (aNbU <= 0 || aNbV <= 0)
        {
            theDI.Output.WriteLine("Error: Arguments nbu and nbv must be both greater than 0");
            return 1;
        }

        Surface? aSurf = DrawTrSurf.GetSurface(theDI, aSrcName);
        double aUMin, aUMax, aVMin, aVMax;
        if (aSurf is not null)
        {
            aSurf.Bounds(out aUMin, out aUMax, out aVMin, out aVMax);
        }
        else
        {
            Shape aShape = theDI.GetShape(aSrcName);
            if (aShape.IsNull || aShape.ShapeType() != ShapeType.Face)
            {
                theDI.Output.WriteLine("Error: " + aSrcName + " is not a face");
                return 1;
            }

            Face aSrcFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aShape);
            aSurf = Tool.Surface(aSrcFace);
            if (aSurf is null)
            {
                theDI.Output.WriteLine("Error: Face " + aSrcName + " has no surface");
                return 1;
            }

            BrepTools.UVBounds(aSrcFace, out aUMin, out aUMax, out aVMin, out aVMax);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aUMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aUMax) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aVMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aVMax))
        {
            theDI.Output.WriteLine("Error: surface has infinite parametric range, aborting");
            return 1;
        }

        var aFaceMaker = new BRepBuilderAPI_MakeFace(aSurf, aUMin, aUMax, aVMin, aVMax, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (!aFaceMaker.IsDone())
        {
            theDI.Output.WriteLine("Error: cannot build face with natural bounds, aborting");
            return 1;
        }

        // OCCT writes into the new face: here into a copy of its graph, as the maker's face is frozen (decision 480)
        Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(DrawShapeCopy.Copy(aFaceMaker.Face()));

        // create triangulation
        int aNbNodes = (aNbU + 1) * (aNbV + 1);
        int aNbTriangles = 2 * aNbU * aNbV;
        var aTriangulation = new Triangulation(aNbNodes, aNbTriangles, false);

        // fill nodes
        var anAdSurf = new SurfaceAdaptor(aSurf);
        double aDU = (aUMax - aUMin) / aNbU;
        double aDV = (aVMax - aVMin) / aNbV;
        for (int iU = 0, iShift = 1; iU <= aNbU; iU++, iShift += aNbV + 1)
        {
            double aU = aUMin + iU * aDU;
            for (int iV = 0; iV <= aNbV; iV++)
            {
                double aV = aVMin + iV * aDV;
                Point3D aP = anAdSurf.Value(aU, aV);
                aTriangulation.SetNode(iShift + iV, aP);
            }
        }

        // fill triangles
        for (int iU = 0, iShift = 1, iTri = 0; iU < aNbU; iU++, iShift += aNbV + 1)
        {
            for (int iV = 0; iV < aNbV; iV++)
            {
                int iBase = iShift + iV;
                var aTri1 = new Triangle1(iBase, iBase + aNbV + 2, iBase + 1);
                var aTri2 = new Triangle1(iBase, iBase + aNbV + 1, iBase + aNbV + 2);
                aTriangulation.SetTriangle(++iTri, aTri1);
                aTriangulation.SetTriangle(++iTri, aTri2);
            }
        }

        // put triangulation to face
        var B = new BrepBuilder();
        B.UpdateFace(aFace, aTriangulation);

        // fill edge polygons
        var aUMinIso = new NCollection_Array1<int>(1, aNbV + 1);
        var aUMaxIso = new NCollection_Array1<int>(1, aNbV + 1);
        for (int iV = 0; iV <= aNbV; iV++)
        {
            aUMinIso.SetValue(1 + iV, 1 + iV);
            aUMaxIso.SetValue(1 + iV, 1 + iV + aNbU * (1 + aNbV));
        }

        var aVMinIso = new NCollection_Array1<int>(1, aNbU + 1);
        var aVMaxIso = new NCollection_Array1<int>(1, aNbU + 1);
        for (int iU = 0; iU <= aNbU; iU++)
        {
            aVMinIso.SetValue(1 + iU, 1 + iU * (1 + aNbV));
            aVMaxIso.SetValue(1 + iU, (1 + iU) * (1 + aNbV));
        }

        var aUMinPoly = new PolygonOnTriangulation(aUMinIso);
        var aUMaxPoly = new PolygonOnTriangulation(aUMaxIso);
        var aVMinPoly = new PolygonOnTriangulation(aVMinIso);
        var aVMaxPoly = new PolygonOnTriangulation(aVMaxIso);
        for (var exp = new global::Kerf.Engine.Topology.Explorer(aFace, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge anEdge = global::Kerf.Engine.Topology.ShapeCasts.Edge(exp.Current());
            Curve2D aC = Tool.CurveOnSurface(anEdge, aFace, out double aFirst, out double aLast)!;
            Point2D aPFirst = aC.Value(aFirst);
            Point2D aPLast = aC.Value(aLast);
            if (Math.Abs(aPFirst.X() - aPLast.X()) < 0.1 * (aUMax - aUMin)) // U=const
            {
                if (Tool.IsClosed(anEdge, aFace))
                {
                    B.UpdateEdge(anEdge, aUMinPoly, aUMaxPoly, aTriangulation);
                }
                else
                {
                    B.UpdateEdge(anEdge, aPFirst.X() < 0.5 * (aUMin + aUMax) ? aUMinPoly : aUMaxPoly, aTriangulation);
                }
            }
            else // V=const
            {
                if (Tool.IsClosed(anEdge, aFace))
                {
                    B.UpdateEdge(anEdge, aVMinPoly, aVMaxPoly, aTriangulation);
                }
                else
                {
                    B.UpdateEdge(anEdge, aPFirst.Y() < 0.5 * (aVMin + aVMax) ? aVMinPoly : aVMaxPoly, aTriangulation);
                }
            }
        }

        theDI.SetShape(aResName, aFace);
        return 0;
    }

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

        Shape aShape = theDI.GetShape(theArgVec[1]);
        if (aShape.IsNull)
        {
            theResult.Append(theArgVec[1]).Append(" is not a shape\n");
            return 1;
        }

        bool toPrintLODs = theArgVec.Count > 2 && theArgVec[2].Equals("-lods", StringComparison.OrdinalIgnoreCase);

        double aMaxDeflection = 0.0, aMeshingDefl = -1.0, aMeshingAngDefl = -1.0, aMeshingMinSize = -1.0;
        int aNbFaces = 0, aNbEmptyFaces = 0, aNbTriangles = 0, aNbNodes = 0, aNbRepresentations = 0;
        var aLODsStat = new List<TriangulationStat>(); // NCollection_IndexedDataMap<int, ...> keyed 0, 1, 2 ...
        var aNbLODs = new List<int>();
        for (var anExp = new global::Kerf.Engine.Topology.Explorer(aShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(anExp.Current());
            aNbFaces++;
            Triangulation? aTriangulation = Tool.Triangulation(aFace, out _);
            if (aTriangulation is not null)
            {
                aNbTriangles += aTriangulation.TriangleCount;
                aNbNodes += aTriangulation.NodeCount;
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
                NCollection_List<Triangulation> aLODs = Tool.Triangulations(aFace, out _);
                if (aLODs.Extent() != 0)
                {
                    aNbLODs.Add(aLODs.Extent());
                }

                int aTriangIndex = 0;
                for (var anIter = new NCollection_List<Triangulation>.Iterator(aLODs); anIter.More(); anIter.Next(), ++aTriangIndex)
                {
                    if (aTriangIndex == aLODsStat.Count)
                    {
                        aLODsStat.Add(new TriangulationStat());
                    }

                    TriangulationStat aStats = aLODsStat[aTriangIndex];
                    aStats.NbFaces++;
                    Triangulation? aLOD = anIter.Value();
                    if (aLOD is null)
                    {
                        aStats.NbEmptyFaces++;
                        continue;
                    }

                    string aType = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aLOD.GetType());
                    int aTypeIndex = aStats.TypeMap.FindIndex(t => t.Type == aType);
                    if (aTypeIndex < 0)
                    {
                        aStats.TypeMap.Add((aType, 1));
                    }
                    else
                    {
                        aStats.TypeMap[aTypeIndex] = (aType, aStats.TypeMap[aTypeIndex].Count + 1);
                    }

                    if (aLOD.HasDeferredData)
                    {
                        aStats.NbDeferredFaces++;
                        if (aLOD.HasGeometry)
                        {
                            aStats.NbTriangles += aLOD.TriangleCount;
                        }
                        else
                        {
                            aStats.NbUnloadedFaces++;
                            aStats.NbTriangles += aLOD.DeferredTriangleCount;
                            aStats.NbUnloadedTriangles += aLOD.DeferredTriangleCount;
                        }
                    }
                    else if (aLOD.HasGeometry)
                    {
                        aStats.NbTriangles += aLOD.TriangleCount;
                    }
                    else
                    {
                        aStats.NbEmptyFaces++;
                    }
                }
            }
        }

        var anEdges = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(aShape, ShapeType.Edge, anEdges);
        for (int i = 1; i <= anEdges.Extent(); ++i)
        {
            var aTE = (BrepTEdge)anEdges.FindKey(i).TShape()!;
            for (var anIterCR = new NCollection_List<CurveRepresentation>.Iterator(aTE.Curves()); anIterCR.More(); anIterCR.Next())
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
            Shape aShape = theDI.GetShape(argv[i]);
            if (aShape.IsNull)
            {
                continue;
            }

            var anEdgeSet = new NCollection_Map<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            var anEdgeMap = new List<Shape>();
            if (aShape.ShapeType() == ShapeType.Edge)
            {
                anEdgeSet.Add(aShape);
                anEdgeMap.Add(aShape);
            }
            else
            {
                for (var ex = new global::Kerf.Engine.Topology.Explorer(aShape, ShapeType.Edge); ex.More(); ex.Next())
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
            foreach (Shape anEdge in anEdgeMap)
            {
                Tool.PolygonOnTriangulation(global::Kerf.Engine.Topology.ShapeCasts.Edge(anEdge), out PolygonOnTriangulation? aPoly, out Triangulation? aT, out Placement aLoc);
                if (aT is null || aPoly is null)
                {
                    continue;
                }

                NCollection_Array1<int> Indices = aPoly.Nodes();
                int nbnodes = Indices.Length();
                for (int j = 1; j <= nbnodes; j++)
                {
                    Point3D P3d = aT.Node(Indices[j]);
                    if (!aLoc.IsIdentity)
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

        Shape aShape = theDI.GetShape(theArgVec[1]);
        if (aShape.IsNull)
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
            var aMergeTool = new MergeNodesTool(aMergeAngle, aMergeToler);
            for (var aFaceIter = new global::Kerf.Engine.Topology.Explorer(aShape, ShapeType.Face); aFaceIter.More(); aFaceIter.Next())
            {
                Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aFaceIter.Value());
                Triangulation? aTris = Tool.Triangulation(aFace, out Placement aFaceLoc);
                if (aTris is null || aTris.NodeCount < 3 || aTris.TriangleCount < 1)
                {
                    continue;
                }

                aNbNodesOld += aTris.NodeCount;
                aNbTrisOld += aTris.TriangleCount;
                aMergeTool.AddTriangulation(aTris, aFaceLoc.Transformation(), aFace.Orientation() == Orientation.Reversed);
            }

            Triangulation? aNewTris = aMergeTool.Result();
            if (aNewTris is null)
            {
                theResult.Append("Error: empty result");
                return 0;
            }

            aNbNodesNew += aNewTris.NodeCount;
            aNbTrisNew += aNewTris.TriangleCount;
            var aFace1 = new Face();
            new BrepBuilder().MakeFace(ref aFace1, aNewTris);
            theDI.SetShape(aResFace, aFace1);
        }
        else
        {
            // in place in OCCT: on a copy of the graph here, bound when a face changed
            Shape aCopy = DrawShapeCopy.Copy(aShape);
            bool isChanged = false;
            var aProcessedFaces = new NCollection_Map<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            for (var aFaceIter = new global::Kerf.Engine.Topology.Explorer(aCopy, ShapeType.Face); aFaceIter.More(); aFaceIter.Next())
            {
                Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aFaceIter.Value());
                if (!aProcessedFaces.Add(aFace.Located(new Placement())))
                {
                    continue;
                }

                Triangulation? aTris = Tool.Triangulation(aFace, out _);
                if (aTris is null || aTris.NodeCount < 3 || aTris.TriangleCount < 1)
                {
                    continue;
                }

                aNbNodesOld += aTris.NodeCount;
                aNbTrisOld += aTris.TriangleCount;
                var aMergeTool = new MergeNodesTool(aMergeAngle, aMergeToler, aTris.TriangleCount);
                aMergeTool.AddTriangulation(aTris);
                if (toForce || aMergeTool.NodeCount != aTris.NodeCount || aMergeTool.ElementCount != aTris.TriangleCount)
                {
                    new BrepBuilder().UpdateFace(aFace, aMergeTool.Result(), false);
                    isChanged = true;
                }

                aTris = Tool.Triangulation(aFace, out _)!;
                aNbNodesNew += aTris.NodeCount;
                aNbTrisNew += aTris.TriangleCount;
            }

            if (isChanged)
            {
                DrawShapeRebind.Rebind(theDI, aShape, aCopy);
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
        Shape S = theDI.GetShape(aName);
        Shape aCopy = DrawShapeCopy.Copy(S);

        // OCCT writes the normals into the attached triangulations, which leaves the faces' flags as they are; the
        // port attaches corrected copies with BRep_Builder::UpdateFace, which marks the faces modified and unchecked.
        // The faces get their flags back, so the shape writes OCCT's text.
        var aFlags = new List<(global::Kerf.Engine.Topology.TShape Face, bool Modified, bool Checked)>();
        for (var anExp = new global::Kerf.Engine.Topology.Explorer(aCopy, ShapeType.Face); anExp.More(); anExp.Next())
        {
            global::Kerf.Engine.Topology.TShape aTFace = anExp.Current().TShape()!;
            aFlags.Add((aTFace, aTFace.Modified(), aTFace.Checked()));
        }

        // OCCT bug: EnsureNormalConsistency reads UV nodes a triangulation may not have (after trmergenodes); the port
        // throws Standard_OutOfRange there (occt-issues.md)
        bool isCorrected = BRepLib.EnsureNormalConsistency(aCopy);
        foreach ((global::Kerf.Engine.Topology.TShape aTFace, bool aModified, bool aChecked) in aFlags)
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
            DrawShapeRebind.Rebind(theDI, S, aCopy);
            theResult.Append("Some corrections in source shape have been made!\n");
        }

        return 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // MeshTest_PluginCommands.cxx

    /// <summary>mpparallel [toTurnOn]: shows or sets the incremental mesher's parallel default.</summary>
    private static int MpParallel(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count == 2)
        {
            BRepMesh_IncrementalMesh.SetParallelDefault(theDI.Atoi(argv[1]) == 1);
        }

        theDI.Output.Write("Incremental Mesh, multi-threading " + (BRepMesh_IncrementalMesh.IsParallelDefault() ? "ON\n" : "OFF\n"));
        return 0;
    }

    /// <summary>mpnames: the names of the registered meshing algorithms (printf).</summary>
    private static int MpNames(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            theDI.Output.Write(" use mpnames\n");
            return 0;
        }

        NCollection_List<BRepMesh_DiscretAlgoFactory> aFactories = BRepMesh_DiscretAlgoFactory.Factories();
        if (aFactories.IsEmpty())
        {
            theDI.Output.Write(" *no algorithms registered\n");
            return 0;
        }

        theDI.Output.Write(" *available algorithms:\n");
        foreach (BRepMesh_DiscretAlgoFactory aFactory in aFactories)
        {
            theDI.Output.Write("  " + aFactory.Name() + "\n");
        }

        return 0;
    }

    /// <summary>mpsetdefaultname name: sets the default meshing algorithm (process-wide, as in OCCT).</summary>
    private static int MpSetDefaultName(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            theDI.Output.Write(" use mpsetdefaultname name\n");
            return 0;
        }

        string aName = a[1];
        if (BRepMesh_DiscretFactory.Get().SetDefaultName(aName))
        {
            theDI.Output.Write(" *ready\n");
        }
        else
        {
            theDI.Output.Write(" *algorithm '" + aName + "' not found\n");
        }

        return 0;
    }

    /// <summary>mpgetdefaultname: the default meshing algorithm's name.</summary>
    private static int MpGetDefaultName(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            theDI.Output.Write(" use mpgetdefaultname\n");
            return 0;
        }

        theDI.Output.Write(" *default name: " + BRepMesh_DiscretFactory.Get().DefaultName() + "\n");
        return 0;
    }

    /// <summary>
    /// mpincmesh shape deflection [angle]: meshes the shape with the default algorithm of the mesh factory. A frozen
    /// shape is meshed on a copy, which the variables sharing its sub-shapes are rebound to (decision 495).
    /// </summary>
    private static int MpIncMesh(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            theDI.Output.Write(" use mpincmesh shape deflection [angle]\n");
            return 0;
        }

        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            theDI.Output.Write(" null shape is not allowed here\n");
            return 0;
        }

        double aDeflection = theDI.Atof(a[2]);
        double aAngle = 0.5;
        if (a.Count > 3)
        {
            aAngle = theDI.Atof(a[3]);
        }

        BRepMesh_DiscretRoot? aMeshAlgo = BRepMesh_DiscretFactory.Get().Discret(aS, aDeflection, aAngle);
        if (aMeshAlgo is null)
        {
            theDI.Output.Write(" *Cannot create the meshing algorithm\n");
            return 0;
        }

        aMeshAlgo.Perform();
        if (!aMeshAlgo.IsDone())
        {
            theDI.Output.Write(" *Meshing not done\n");
        }

        // OCCT.NET: the meshed copy (decision 489), seen by every variable that shares its sub-shapes
        DrawShapeRebind.Rebind(theDI, aS, aMeshAlgo.Shape());
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

        Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull)
        {
            return 1;
        }

        double anEps = -1.0;
        if (n > 2)
        {
            anEps = theDI.Atof(a[2]);
        }

        var aMapF = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(shape, ShapeType.Face, aMapF);

        // detect if a shape has triangulation
        bool hasPoly = false;
        for (int i = 1; i <= aMapF.Extent(); i++)
        {
            if (Tool.Triangulation(global::Kerf.Engine.Topology.ShapeCasts.Face(aMapF.FindKey(i)), out _) is not null)
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
                Triangulation? aPoly = Tool.Triangulation(global::Kerf.Engine.Topology.ShapeCasts.Face(aMapF.FindKey(i)), out _);
                if (aPoly is null)
                {
                    theDI.Output.Write("face " + i + " has no triangulation\n");
                    continue;
                }

                for (int j = 1; j <= aPoly.TriangleCount; j++)
                {
                    aPoly.Triangle(j).Get(out int n1, out int n2, out int n3);
                    Point3D p1 = aPoly.Node(n1);
                    Point3D p2 = aPoly.Node(n2);
                    Point3D p3 = aPoly.Node(n3);
                    var v1 = new Vector3D(p1, p2);
                    var v2 = new Vector3D(p1, p3);
                    aTriArea += v1.CrossMagnitude(v2);
                }
            }

            aTriArea /= 2;
        }

        // compute area by geometry
        var props = new Properties();
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

        Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull)
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
        for (var aFaceExp = new global::Kerf.Engine.Topology.Explorer(shape, ShapeType.Face); aFaceExp.More(); aFaceExp.Next(), ++aFaceIndex)
        {
            Shape aShape = aFaceExp.Current();
            Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aShape);
            Triangulation? aT = Tool.Triangulation(aFace, out Placement aLoc);

            // Iterate boundary edges (BRepMesh_Edge: equal in either direction)
            var aBoundaryEdgeMap = new HashSet<(int, int)>();
            for (var anExp = new global::Kerf.Engine.Topology.Explorer(aShape, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                PolygonOnTriangulation? aPoly = Tool.PolygonOnTriangulation(global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current()), aT, aLoc);
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
            int aTriNum = aT!.TriangleCount;
            for (int aTriIndx = 1; aTriIndx <= aTriNum; aTriIndx++)
            {
                Triangle1 aTri = aT.Triangle(aTriIndx);
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
