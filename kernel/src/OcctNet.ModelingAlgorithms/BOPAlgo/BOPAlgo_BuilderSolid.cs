// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderSolid.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Builds the solids bounded by a set of faces.</summary>
/// <remarks>
/// <para>
/// The faces are split into closed shells (BOPAlgo_ShellSplitter); a shell is a growth (an outer shell of a solid)
/// unless an infinite point classifies IN it, when it is a hole; each hole is put into the smallest growth containing
/// it, or makes a solid of its own. Faces that make no closed shell are added as internal shells to the solids
/// containing them (unless <see cref="BOPAlgo_BuilderArea.SetAvoidInternalShapes"/>), and the faces inside no solid
/// are reported by <see cref="BOPAlgo_AlertSolidBuilderUnusedFaces"/>. Infinite faces make shells by themselves.
/// </para>
/// <para><see cref="GetBoxesMap"/> gives the boxes of the solids built (an infinite one for the hole solids).</para>
/// </remarks>
public class BOPAlgo_BuilderSolid : BOPAlgo_BuilderArea
{
    private readonly NCollection_DataMap<TopoDS_Shape, Box> myBoxes = new(TopTools_ShapeMapHasher.Instance); // Boxes of the produced solids

    public BOPAlgo_BuilderSolid()
    {
    }

    /// <summary>Builds the solids.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        using var aPS = new ProgressScope(theRange, null, 100);

        GetReport().Clear();
        //
        if (myShapes.IsEmpty())
        {
            return;
        }

        myContext ??= new IntTools_Context();

        myBoxes.Clear();

        TopoDS_Compound aC = new();
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        aBB.MakeCompound(ref aC);
        aIt.Initialize(myShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aF = aIt.Value();
            aBB.Add(aC, aF);
        }
        //
        PerformShapesToAvoid(aPS.Next(1));
        if (HasErrors())
        {
            return;
        }
        //
        PerformLoops(aPS.Next(10));
        if (HasErrors())
        {
            return;
        }
        //
        PerformAreas(aPS.Next(80));
        if (HasErrors())
        {
            return;
        }
        //
        PerformInternalShapes(aPS.Next(9));
    }

    /// <summary>The boxes of the created solids.</summary>
    public NCollection_DataMap<TopoDS_Shape, Box> GetBoxesMap() => myBoxes;

    /// <summary>Collects the faces that can't be in a closed shell (with free edges) into the shapes to avoid.</summary>
    protected override void PerformShapesToAvoid(ProgressRange theRange)
    {
        bool bFound;
        int i, aNbE, aNbF;
        TopAbs_Orientation aOrE;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        myShapesToAvoid.Clear();
        //
        using var aPS = new ProgressScope(theRange, null, 1);
        //
        for (;;)
        {
            if (UserBreak(aPS))
            {
                return;
            }
            bFound = false;
            //
            // 1. MEF
            aMEF.Clear();
            aIt.Initialize(myShapes);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aF = aIt.Value();
                if (!myShapesToAvoid.Contains(aF))
                {
                    TopExp.MapShapesAndAncestors(aF, TopAbs_EDGE, TopAbs_FACE, aMEF);
                }
            }
            aNbE = aMEF.Extent();
            //
            // 2. myFacesToAvoid
            for (i = 1; i <= aNbE; ++i)
            {
                TopoDS_Edge aE = TopoDS.Edge(aMEF.FindKey(i));
                if (BRep_Tool.Degenerated(aE))
                {
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape> aLF = aMEF.ChangeFromKey(aE);
                aNbF = aLF.Extent();
                if (aNbF == 0)
                {
                    continue;
                }
                //
                aOrE = aE.Orientation();
                //
                TopoDS_Face aF1 = TopoDS.Face(aLF.First());
                if (aNbF == 1)
                {
                    if (aOrE == TopAbs_INTERNAL)
                    {
                        continue;
                    }
                    bFound = true;
                    myShapesToAvoid.Add(aF1);
                }
                else if (aNbF == 2)
                {
                    TopoDS_Face aF2 = TopoDS.Face(aLF.Last());
                    if (aF2.IsSame(aF1))
                    {
                        if (BRep_Tool.IsClosed(aE, aF1))
                        {
                            continue;
                        }
                        //
                        if (aOrE == TopAbs_INTERNAL)
                        {
                            continue;
                        }
                        //
                        bFound = true;
                        myShapesToAvoid.Add(aF1);
                        myShapesToAvoid.Add(aF2);
                    }
                }
            } // for (i=1; i<=aNbE; ++i) {
            //
            if (!bFound)
            {
                break;
            }
            //
        } // for(;;) {
    }

    /// <summary>Builds the closed shells (the loops) and the internal shells of the faces to avoid.</summary>
    protected override void PerformLoops(ProgressRange theRange)
    {
        int i, aNbSh;
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        TopoDS_Iterator aItS = new();
        //
        myLoops.Clear();
        //
        BOPAlgo_ShellSplitter aSSp = new();
        //
        using var aMainScope = new ProgressScope(theRange, "Building shells", 10);

        // 1. Shells Usual
        aIt.Initialize(myShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aIt.Value());
            if (myContext!.IsInfiniteFace(aF))
            {
                TopoDS_Shell aSh = new();
                BRep_Builder aBB0 = new();
                //
                aBB0.MakeShell(ref aSh);
                aBB0.Add(aSh, aF);
                myLoops.Append(aSh);
                continue;
            }
            //
            if (!myShapesToAvoid.Contains(aF))
            {
                aSSp.AddStartElement(aF);
            }
        }
        //
        aSSp.SetRunParallel(myRunParallel);
        aSSp.Perform(aMainScope.Next(9));
        if (aSSp.HasErrors())
        {
            // add warning status
            if (aMainScope.More())
            {
                TopoDS_Compound aFacesSp = new();
                new BRep_Builder().MakeCompound(ref aFacesSp);
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aSSp.StartElements());
                for (; aItLF.More(); aItLF.Next())
                {
                    new BRep_Builder().Add(aFacesSp, aItLF.Value());
                }
                AddWarning(new BOPAlgo_AlertShellSplitterFailed(aFacesSp));
            }
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLSh = aSSp.Shells();
        aIt.Initialize(aLSh);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSh = aIt.Value();
            myLoops.Append(aSh);
        }
        //=================================================
        //
        // 2. Post Treatment
        BRep_Builder aBB = new();
        NCollection_Map<TopoDS_Shape> AddedFacesMap = new(EqualityComparer<TopoDS_Shape>.Default);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aEFMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMP = new(EqualityComparer<TopoDS_Shape>.Default);
        //
        // a. collect all edges that are in loops
        aIt.Initialize(myLoops);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            aItS.Initialize(aS);
            for (; aItS.More(); aItS.Next())
            {
                TopoDS_Shape aF = aItS.Value();
                aMP.Add(aF);
            }
        }
        if (UserBreak(aMainScope))
        {
            return;
        }
        //
        // b. collect all edges that are to avoid
        aNbSh = myShapesToAvoid.Extent();
        for (i = 1; i <= aNbSh; ++i)
        {
            TopoDS_Shape aF = myShapesToAvoid[i];
            aMP.Add(aF);
        }
        //
        // c. add all edges that are not processed to myShapesToAvoid
        aIt.Initialize(myShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aIt.Value());
            if (!myContext!.IsInfiniteFace(aF))
            {
                if (!aMP.Contains(aF))
                {
                    myShapesToAvoid.Add(aF);
                }
            }
        }
        //=================================================
        if (UserBreak(aMainScope))
        {
            return;
        }
        //
        // 3.Internal Shells
        myLoopsInternal.Clear();
        //
        aEFMap.Clear();
        AddedFacesMap.Clear();
        //
        aNbSh = myShapesToAvoid.Extent();
        for (i = 1; i <= aNbSh; ++i)
        {
            TopoDS_Shape aFF = myShapesToAvoid[i];
            TopExp.MapShapesAndAncestors(aFF, TopAbs_EDGE, TopAbs_FACE, aEFMap);
        }
        //
        for (i = 1; i <= aNbSh; ++i)
        {
            if (UserBreak(aMainScope))
            {
                return;
            }
            TopoDS_Shape aFF = myShapesToAvoid[i];
            if (!AddedFacesMap.Add(aFF))
            {
                continue;
            }
            //
            // make a new shell
            TopExp_Explorer aExp = new();
            TopoDS_Shell aShell = new();
            aBB.MakeShell(ref aShell);
            aBB.Add(aShell, aFF);
            //
            aItS.Initialize(aShell);
            for (; aItS.More(); aItS.Next())
            {
                TopoDS_Face aF = TopoDS.Face(aItS.Value());
                //
                aExp.Init(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                    NCollection_List<TopoDS_Shape> aLF = aEFMap.FindFromKey(aE);
                    aIt.Initialize(aLF);
                    for (; aIt.More(); aIt.Next())
                    {
                        TopoDS_Face aFL = TopoDS.Face(aIt.Value());
                        if (AddedFacesMap.Add(aFL))
                        {
                            aBB.Add(aShell, aFL);
                        }
                    }
                }
            }
            aShell.Closed(BRep_Tool.IsClosed(aShell));
            myLoopsInternal.Append(aShell);
        }
    }

    /// <summary>Makes the solids of the growth shells and puts the holes into them.</summary>
    protected override void PerformAreas(ProgressRange theRange)
    {
        myAreas.Clear();
        BRep_Builder aBB = new();
        // The new solids
        NCollection_List<TopoDS_Shape> aNewSolids = new();
        // The hole shells which has to be classified relatively new solids
        NCollection_IndexedMap<TopoDS_Shape> aHoleShells = new(TopTools_ShapeMapHasher.Instance);
        // Map of the faces of the hole shells for quick check of the growths.
        // If the analyzed shell contains any of the hole faces, it is considered as growth.
        NCollection_IndexedMap<TopoDS_Shape> aMHF = new(TopTools_ShapeMapHasher.Instance);

        using var aMainScope = new ProgressScope(theRange, "Building solids", 10);

        // Analyze the shells
        using (var aPSClass = new ProgressScope(aMainScope.Next(5), "Classify solids", myLoops.Extent()))
        {
            NCollection_List<TopoDS_Shape>.Iterator aItLL = new(myLoops);
            for (; aItLL.More(); aItLL.Next(), aPSClass.Next())
            {
                if (UserBreak(aPSClass))
                {
                    return;
                }
                TopoDS_Shape aShell = aItLL.Value();

                bool bIsGrowth = IsGrowthShell(aShell, aMHF);
                if (!bIsGrowth)
                {
                    // Fast check did not give the result, run classification
                    bIsGrowth = !IsHole(aShell, myContext!);
                }

                // Save the solid
                if (bIsGrowth)
                {
                    TopoDS_Solid aSolid = new();
                    aBB.MakeSolid(ref aSolid);
                    aBB.Add(aSolid, aShell);
                    aNewSolids.Append(aSolid);
                }
                else
                {
                    aHoleShells.Add(aShell);
                    TopExp.MapShapes(aShell, TopAbs_FACE, aMHF);
                }
            }
        }

        if (aHoleShells.IsEmpty())
        {
            // No holes, stop the analysis
            NCollection_List<TopoDS_Shape>.Iterator aItLS0 = new(aNewSolids);
            for (; aItLS0.More(); aItLS0.Next())
            {
                TopoDS_Shape aSol = aItLS0.Value();
                myAreas.Append(aSol);
                // Build box
                Box aBox = new();
                BRepBndLib.Add(aSol, ref aBox);
                myBoxes.Bind(aSol, aBox);
            }
            return;
        }

        // Classify holes relatively solids

        // Prepare tree with the boxes of the hole shells
        BOPTools_BoxTree aBBTree = new();
        int i, aNbH = aHoleShells.Extent();
        aBBTree.SetSize(aNbH);
        for (i = 1; i <= aNbH; ++i)
        {
            TopoDS_Shape aHShell = aHoleShells[i];
            //
            Box aBox = new();
            BRepBndLib.Add(aHShell, ref aBox);
            aBBTree.Add(i, Tools.Bnd2BVH(aBox));

            myBoxes.Bind(aHShell, aBox);
        }

        // Build BVH
        aBBTree.Build();

        // Find outer growth shell that is most close to each hole shell
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> aHoleSolidMap = new(TopTools_ShapeMapHasher.Instance);

        using (var aPSH = new ProgressScope(aMainScope.Next(4), "Adding holes", aNewSolids.Extent()))
        {
            NCollection_List<TopoDS_Shape>.Iterator aItLS1 = new(aNewSolids);
            for (; aItLS1.More(); aItLS1.Next(), aPSH.Next())
            {
                if (UserBreak(aPSH))
                {
                    return;
                }
                TopoDS_Shape aSolid = aItLS1.Value();

                // Build box
                Box aBox = new();
                BRepBndLib.Add(aSolid, ref aBox);

                myBoxes.Bind(aSolid, aBox);

                BOPTools_BoxTreeSelector aSelector = new();
                aSelector.SetBox(Tools.Bnd2BVH(aBox));
                aSelector.SetBVHSet(aBBTree);
                aSelector.Select();

                NCollection_List<int> aLI = aSelector.Indices();
                NCollection_List<int>.Iterator aItLI = new(aLI);
                for (; aItLI.More(); aItLI.Next())
                {
                    int k = aItLI.Value();
                    TopoDS_Shape aHole = aHoleShells[k];
                    // Check if it is inside
                    if (!IsInside(aHole, aSolid, myContext!))
                    {
                        continue;
                    }

                    // Save the relation
                    int anIndexWas = aHoleSolidMap.FindIndex(aHole);
                    if (anIndexWas > 0)
                    {
                        ref TopoDS_Shape pSolidWas = ref aHoleSolidMap.ChangeFromIndex(anIndexWas);
                        if (IsInside(aSolid, pSolidWas, myContext!))
                        {
                            pSolidWas = aSolid;
                        }
                    }
                    else
                    {
                        aHoleSolidMap.Add(aHole, aSolid);
                    }
                }
            }
        }

        // Make the back map from solids to holes
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aSolidHolesMap = new(TopTools_ShapeMapHasher.Instance);

        aNbH = aHoleSolidMap.Extent();
        for (i = 1; i <= aNbH; ++i)
        {
            TopoDS_Shape aHole = aHoleSolidMap.FindKey(i);
            TopoDS_Shape aSolid = aHoleSolidMap[i];
            //
            int anIndex = aSolidHolesMap.FindIndex(aSolid);
            NCollection_List<TopoDS_Shape> pLHoles = anIndex > 0
                                                     ? aSolidHolesMap[anIndex]
                                                     : aSolidHolesMap[aSolidHolesMap.Add(aSolid, new NCollection_List<TopoDS_Shape>())];
            pLHoles.Append(aHole);
        }

        // Add Holes to Solids and add them to myAreas
        using (var aPSU = new ProgressScope(aMainScope.Next(), null, aNewSolids.Extent()))
        {
            NCollection_List<TopoDS_Shape>.Iterator aItLS2 = new(aNewSolids);
            for (; aItLS2.More(); aItLS2.Next(), aPSU.Next())
            {
                if (UserBreak(aPSU))
                {
                    return;
                }
                TopoDS_Solid aSolid = TopoDS.Solid(aItLS2.Value());
                if (aSolidHolesMap.FindFromKey(aSolid, out NCollection_List<TopoDS_Shape> pLHoles))
                {
                    // update solid
                    NCollection_List<TopoDS_Shape>.Iterator aItLH = new(pLHoles);
                    for (; aItLH.More(); aItLH.Next())
                    {
                        TopoDS_Shape aHole = aItLH.Value();
                        aBB.Add(aSolid, aHole);
                    }

                    // update classifier
                    myContext!.SolidClassifier(aSolid).Load(aSolid);
                }

                myAreas.Append(aSolid);
            }
        }

        // Add holes that outside the solids to myAreas
        aNbH = aHoleShells.Extent();
        for (i = 1; i <= aNbH; ++i)
        {
            TopoDS_Shape aHole = aHoleShells[i];
            if (!aHoleSolidMap.Contains(aHole))
            {
                TopoDS_Solid aSolid = new();
                aBB.MakeSolid(ref aSolid);
                aBB.Add(aSolid, aHole);
                //
                myAreas.Append(aSolid);
                // Make an infinite box for the hole
                Box aBox = new();
                aBox.SetWhole();
                myBoxes.Bind(aSolid, aBox);
            }

            myBoxes.UnBind(aHole);
        }
    }

    /// <summary>Puts the internal shells into the solids containing them.</summary>
    protected override void PerformInternalShapes(ProgressRange theRange)
    {
        if (myAvoidInternalShapes)
        {
            // user-defined option to avoid internal parts is in force
            return;
        }

        if (myLoopsInternal.IsEmpty())
        {
            // no internal parts
            return;
        }

        using var aMainScope = new ProgressScope(theRange, "Adding internal shapes", 2);

        // Get all faces to classify
        NCollection_IndexedMap<TopoDS_Shape> aMFs = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLS = new(myLoopsInternal);
        for (; aItLS.More(); aItLS.Next())
        {
            TopoDS_Shape aShell = aItLS.Value();
            TopoDS_Iterator aIt = new(aShell);
            for (; aIt.More(); aIt.Next())
            {
                aMFs.Add(aIt.Value());
            }
        }

        BRep_Builder aBB = new();
        // Check existence of the growths solids
        if (myAreas.IsEmpty())
        {
            // No areas.
            // Just make solid of the faces
            TopoDS_Solid aSolid = new();
            aBB.MakeSolid(ref aSolid);
            //
            NCollection_List<TopoDS_Shape> aLSI0 = new();
            MakeInternalShells(aMFs, aLSI0);
            //
            aItLS.Initialize(aLSI0);
            for (; aItLS.More(); aItLS.Next())
            {
                aBB.Add(aSolid, aItLS.Value());
            }

            myAreas.Append(aSolid);
            return;
        }

        if (UserBreak(aMainScope))
        {
            return;
        }

        // Classify faces relatively solids

        // Prepare list of faces to classify
        NCollection_List<TopoDS_Shape> aLFaces = new();
        int i, aNbF = aMFs.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            aLFaces.Append(aMFs[i]);
        }

        // Map of solids with IN faces
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMSLF = new(TopTools_ShapeMapHasher.Instance);

        // Perform classification
        BOPAlgo_Tools.ClassifyFaces(aLFaces,
                                    myAreas,
                                    myRunParallel,
                                    myContext!,
                                    aMSLF,
                                    myBoxes,
                                    new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance),
                                    aMainScope.Next());

        // Update Solids by internal Faces

        NCollection_Map<TopoDS_Shape> aMFDone = new(TopTools_ShapeMapHasher.Instance);

        int aNbS = aMSLF.Extent();
        using (var aPSLoop = new ProgressScope(aMainScope.Next(), null, aNbS))
        {
            for (i = 1; i <= aNbS; ++i, aPSLoop.Next())
            {
                if (UserBreak(aPSLoop))
                {
                    return;
                }
                TopoDS_Shape pSolid = aMSLF.FindKey(i);

                NCollection_List<TopoDS_Shape> aLF = aMSLF[i];
                if (aLF.IsEmpty())
                {
                    continue;
                }

                NCollection_IndexedMap<TopoDS_Shape> aMF = new(TopTools_ShapeMapHasher.Instance);
                aItLS.Initialize(aLF);
                for (; aItLS.More(); aItLS.Next())
                {
                    TopoDS_Shape aF = aItLS.Value();
                    aMF.Add(aF);
                    aMFDone.Add(aF);
                }
                //
                NCollection_List<TopoDS_Shape> aLSI1 = new();
                MakeInternalShells(aMF, aLSI1);
                //
                aItLS.Initialize(aLSI1);
                for (; aItLS.More(); aItLS.Next())
                {
                    TopoDS_Shape aSI = aItLS.Value();
                    aBB.Add(pSolid, aSI);
                }
            }
        }

        // Find all unclassified faces and warn the user about them.
        // Do not put such faces into result as they will form not closed solid.
        NCollection_IndexedMap<TopoDS_Shape> aMFUnUsed = new(TopTools_ShapeMapHasher.Instance);

        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aF = aMFs[i];
            if (!aMFDone.Contains(aF))
            {
                aMFUnUsed.Add(aF);
            }
        }

        if (aMFUnUsed.Extent() != 0)
        {
            NCollection_List<TopoDS_Shape> aLSI = new();
            MakeInternalShells(aMFUnUsed, aLSI);

            TopoDS_Shape aWShape;
            if (aLSI.Extent() == 1)
            {
                aWShape = aLSI.First();
            }
            else
            {
                TopoDS_Compound aWC = new();
                aBB.MakeCompound(ref aWC);
                aWShape = aWC;
                aItLS.Initialize(aLSI);
                for (; aItLS.More(); aItLS.Next())
                {
                    aBB.Add(aWShape, aItLS.Value());
                }
            }

            AddWarning(new BOPAlgo_AlertSolidBuilderUnusedFaces(aWShape));
        }
    }

    // Makes the internal shells of the faces (OCCT's file-static MakeInternalShells).
    private static void MakeInternalShells(NCollection_IndexedMap<TopoDS_Shape> theMF, NCollection_List<TopoDS_Shape> theShells)
    {
        int i, aNbF;
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aItF = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aAddedFacesMap = new(TopTools_ShapeMapHasher.Instance);
        //
        aNbF = theMF.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aF = theMF[i];
            TopExp.MapShapesAndAncestors(aF, TopAbs_EDGE, TopAbs_FACE, aMEF);
        }
        //
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aFF = theMF[i];
            if (!aAddedFacesMap.Add(aFF))
            {
                continue;
            }
            //
            // make a new shell
            TopoDS_Shell aShell = new();
            aBB.MakeShell(ref aShell);
            aFF.Orientation(TopAbs_INTERNAL);
            aBB.Add(aShell, aFF);
            //
            TopoDS_Iterator aItAddedF = new(aShell);
            for (; aItAddedF.More(); aItAddedF.Next())
            {
                TopoDS_Shape aF = aItAddedF.Value();
                //
                TopExp_Explorer aEdgeExp = new(aF, TopAbs_EDGE);
                for (; aEdgeExp.More(); aEdgeExp.Next())
                {
                    TopoDS_Shape aE = aEdgeExp.Current();
                    NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromKey(aE);
                    aItF.Initialize(aLF);
                    for (; aItF.More(); aItF.Next())
                    {
                        TopoDS_Shape aFL = aItF.Value();
                        if (aAddedFacesMap.Add(aFL))
                        {
                            aFL.Orientation(TopAbs_INTERNAL);
                            aBB.Add(aShell, aFL);
                        }
                    }
                }
            }
            aShell.Closed(BRep_Tool.IsClosed(aShell));
            theShells.Append(aShell);
        }
    }

    // True if an infinite point is IN the shell (OCCT's file-static IsHole). OCCT classifies the shell through a
    // TopoDS_Solid reference to it (a reinterpret cast): the port makes the same unchecked solid of the shell's TShape.
    private static bool IsHole(in TopoDS_Shape theS2, IntTools_Context theContext)
    {
        TopoDS_Solid pS2 = new(theS2.TShape(), theS2.Location(), theS2.Orientation());
        BRepClass3d_SolidClassifier aClsf = theContext.SolidClassifier(pS2);
        //
        aClsf.PerformInfinitePoint(RealSmall());
        //
        return (aClsf.State() == TopAbs_IN);
    }

    // True if theS1 is inside theS2 (OCCT's file-static IsInside); theS2 is a solid, or (from IsHole's callers) a
    // shell cast to one.
    private static bool IsInside(in TopoDS_Shape theS1, in TopoDS_Shape theS2, IntTools_Context theContext)
    {
        TopExp_Explorer aExp = new();
        TopAbs_State aState;
        //
        TopoDS_Solid pS2 = new(theS2.TShape(), theS2.Location(), theS2.Orientation());
        //
        aExp.Init(theS1, TopAbs_FACE);
        if (!aExp.More())
        {
            BRepClass3d_SolidClassifier aClsf = theContext.SolidClassifier(pS2);
            aClsf.PerformInfinitePoint(RealSmall());
            aState = aClsf.State();
        }
        else
        {
            NCollection_IndexedMap<TopoDS_Shape> aBounds = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(pS2, TopAbs_EDGE, aBounds);
            TopoDS_Face aF = TopoDS.Face(aExp.Current());
            aState = BOPTools_AlgoTools.ComputeState(aF, pS2, global::Kerf.Engine.Foundation.Precision.Confusion(), aBounds, theContext);
        }
        return (aState == TopAbs_IN);
    }

    // True if the shell holds one of the hole faces (OCCT's file-static IsGrowthShell).
    private static bool IsGrowthShell(in TopoDS_Shape theShell, NCollection_IndexedMap<TopoDS_Shape> theMHF)
    {
        if (theMHF.Extent() != 0)
        {
            TopoDS_Iterator aIt = new(theShell);
            for (; aIt.More(); aIt.Next())
            {
                if (theMHF.Contains(aIt.Value()))
                {
                    return true;
                }
            }
        }
        return false;
    }
}
