// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderSolid.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans;

/// <summary>Builds the solids bounded by a set of faces.</summary>
/// <remarks>
/// <para>
/// The faces are split into closed shells (BOPAlgo_ShellSplitter); a shell is a growth (an outer shell of a solid)
/// unless an infinite point classifies IN it, when it is a hole; each hole is put into the smallest growth containing
/// it, or makes a solid of its own. Faces that make no closed shell are added as internal shells to the solids
/// containing them (unless <see cref="BuilderArea.SetAvoidInternalShapes"/>), and the faces inside no solid
/// are reported by <see cref="AlertSolidBuilderUnusedFaces"/>. Infinite faces make shells by themselves.
/// </para>
/// <para><see cref="GetBoxesMap"/> gives the boxes of the solids built (an infinite one for the hole solids).</para>
/// </remarks>
public class BuilderSolid : BuilderArea
{
    private readonly Dictionary<Shape, Box> _boxes = new(ShapeHasher.Instance); // Boxes of the produced solids

    public BuilderSolid()
    {
    }

    /// <summary>Builds the solids.</summary>
    public override void Perform(ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, null, 100);

        GetReport().Clear();
        //
        if (_shapes.IsEmpty())
        {
            return;
        }

        _context ??= new IntersectionContext();

        _boxes.Clear();

        Compound c = new();
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator it = new();
        //
        bb.MakeCompound(ref c);
        it.Initialize(_shapes);
        for (; it.More(); it.Next())
        {
            Shape f = it.Value();
            bb.Add(c, f);
        }
        //
        PerformShapesToAvoid(ps.Next(1));
        if (HasErrors)
        {
            return;
        }
        //
        PerformLoops(ps.Next(10));
        if (HasErrors)
        {
            return;
        }
        //
        PerformAreas(ps.Next(80));
        if (HasErrors)
        {
            return;
        }
        //
        PerformInternalShapes(ps.Next(9));
    }

    /// <summary>The boxes of the created solids.</summary>
    public Dictionary<Shape, Box> GetBoxesMap() => _boxes;

    /// <summary>Collects the faces that can't be in a closed shell (with free edges) into the shapes to avoid.</summary>
    protected override void PerformShapesToAvoid(ProgressRange range)
    {
        bool bFound;
        int i, nbE, nbF;
        Orientation orE;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new();
        //
        _shapesToAvoid.Clear();
        //
        using var ps = new ProgressScope(range, null, 1);
        //
        for (;;)
        {
            if (UserBreak(ps))
            {
                return;
            }
            bFound = false;
            //
            // 1. MEF
            mef.Clear();
            it.Initialize(_shapes);
            for (; it.More(); it.Next())
            {
                Shape f = it.Value();
                if (!_shapesToAvoid.Contains(f))
                {
                    Exploration.MapShapesAndAncestors(f, ShapeType.Edge, ShapeType.Face, mef);
                }
            }
            nbE = mef.Extent();
            //
            // 2. myFacesToAvoid
            for (i = 1; i <= nbE; ++i)
            {
                Edge e = ShapeCasts.Edge(mef.FindKey(i));
                if (Tool.Degenerated(e))
                {
                    continue;
                }
                //
                NCollection_List<Shape> lf = mef.ChangeFromKey(e);
                nbF = lf.Extent();
                if (nbF == 0)
                {
                    continue;
                }
                //
                orE = e.Orientation();
                //
                Face f1 = ShapeCasts.Face(lf.First());
                if (nbF == 1)
                {
                    if (orE == Internal)
                    {
                        continue;
                    }
                    bFound = true;
                    _shapesToAvoid.Add(f1);
                }
                else if (nbF == 2)
                {
                    Face f2 = ShapeCasts.Face(lf.Last());
                    if (f2.IsSame(f1))
                    {
                        if (Tool.IsClosed(e, f1))
                        {
                            continue;
                        }
                        //
                        if (orE == Internal)
                        {
                            continue;
                        }
                        //
                        bFound = true;
                        _shapesToAvoid.Add(f1);
                        _shapesToAvoid.Add(f2);
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
    protected override void PerformLoops(ProgressRange range)
    {
        int i, nbSh;
        NCollection_List<Shape>.Iterator it = new();
        Iterator itS = new();
        //
        _loops.Clear();
        //
        ShellSplitter sSp = new();
        //
        using var mainScope = new ProgressScope(range, "Building shells", 10);

        // 1. Shells Usual
        it.Initialize(_shapes);
        for (; it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
            if (_context!.IsInfiniteFace(f))
            {
                Shell sh = new();
                BrepBuilder bb0 = new();
                //
                bb0.MakeShell(ref sh);
                bb0.Add(sh, f);
                _loops.Append(sh);
                continue;
            }
            //
            if (!_shapesToAvoid.Contains(f))
            {
                sSp.AddStartElement(f);
            }
        }
        //
        sSp.SetRunParallel(_runParallel);
        sSp.Perform(mainScope.Next(9));
        if (sSp.HasErrors)
        {
            // add warning status
            if (mainScope.More())
            {
                Compound facesSp = new();
                new BrepBuilder().MakeCompound(ref facesSp);
                NCollection_List<Shape>.Iterator itLF = new(sSp.StartElements());
                for (; itLF.More(); itLF.Next())
                {
                    new BrepBuilder().Add(facesSp, itLF.Value());
                }
                AddWarning(new AlertShellSplitterFailed(facesSp));
            }
            return;
        }
        //
        NCollection_List<Shape> lSh = sSp.Shells();
        it.Initialize(lSh);
        for (; it.More(); it.Next())
        {
            Shape sh = it.Value();
            _loops.Append(sh);
        }
        //=================================================
        //
        // 2. Post Treatment
        BrepBuilder bb = new();
        HashSet<Shape> addedFacesMap = new(EqualityComparer<Shape>.Default);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        HashSet<Shape> mp = new(EqualityComparer<Shape>.Default);
        //
        // a. collect all edges that are in loops
        it.Initialize(_loops);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            itS.Initialize(s);
            for (; itS.More(); itS.Next())
            {
                Shape f = itS.Value();
                mp.Add(f);
            }
        }
        if (UserBreak(mainScope))
        {
            return;
        }
        //
        // b. collect all edges that are to avoid
        nbSh = _shapesToAvoid.Extent();
        for (i = 1; i <= nbSh; ++i)
        {
            Shape f = _shapesToAvoid[i];
            mp.Add(f);
        }
        //
        // c. add all edges that are not processed to myShapesToAvoid
        it.Initialize(_shapes);
        for (; it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
            if (!_context!.IsInfiniteFace(f))
            {
                if (!mp.Contains(f))
                {
                    _shapesToAvoid.Add(f);
                }
            }
        }
        //=================================================
        if (UserBreak(mainScope))
        {
            return;
        }
        //
        // 3.Internal Shells
        _loopsInternal.Clear();
        //
        efMap.Clear();
        addedFacesMap.Clear();
        //
        nbSh = _shapesToAvoid.Extent();
        for (i = 1; i <= nbSh; ++i)
        {
            Shape ff = _shapesToAvoid[i];
            Exploration.MapShapesAndAncestors(ff, ShapeType.Edge, ShapeType.Face, efMap);
        }
        //
        for (i = 1; i <= nbSh; ++i)
        {
            if (UserBreak(mainScope))
            {
                return;
            }
            Shape ff = _shapesToAvoid[i];
            if (!addedFacesMap.Add(ff))
            {
                continue;
            }
            //
            // make a new shell
            Explorer exp = new();
            Shell shell = new();
            bb.MakeShell(ref shell);
            bb.Add(shell, ff);
            //
            itS.Initialize(shell);
            for (; itS.More(); itS.Next())
            {
                Face f = ShapeCasts.Face(itS.Value());
                //
                exp.Init(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Edge e = ShapeCasts.Edge(exp.Current());
                    NCollection_List<Shape> lf = efMap.FindFromKey(e);
                    it.Initialize(lf);
                    for (; it.More(); it.Next())
                    {
                        Face fl = ShapeCasts.Face(it.Value());
                        if (addedFacesMap.Add(fl))
                        {
                            bb.Add(shell, fl);
                        }
                    }
                }
            }
            shell.Closed(Tool.IsClosed(shell));
            _loopsInternal.Append(shell);
        }
    }

    /// <summary>Makes the solids of the growth shells and puts the holes into them.</summary>
    protected override void PerformAreas(ProgressRange range)
    {
        _areas.Clear();
        BrepBuilder bb = new();
        // The new solids
        NCollection_List<Shape> newSolids = new();
        // The hole shells which has to be classified relatively new solids
        NCollection_IndexedMap<Shape> holeShells = new(ShapeHasher.Instance);
        // Map of the faces of the hole shells for quick check of the growths.
        // If the analyzed shell contains any of the hole faces, it is considered as growth.
        NCollection_IndexedMap<Shape> mhf = new(ShapeHasher.Instance);

        using var mainScope = new ProgressScope(range, "Building solids", 10);

        // Analyze the shells
        using (var psClass = new ProgressScope(mainScope.Next(5), "Classify solids", _loops.Extent()))
        {
            NCollection_List<Shape>.Iterator itLL = new(_loops);
            for (; itLL.More(); itLL.Next(), psClass.Next())
            {
                if (UserBreak(psClass))
                {
                    return;
                }
                Shape shell = itLL.Value();

                bool bIsGrowth = IsGrowthShell(shell, mhf);
                if (!bIsGrowth)
                {
                    // Fast check did not give the result, run classification
                    bIsGrowth = !IsHole(shell, _context!);
                }

                // Save the solid
                if (bIsGrowth)
                {
                    Solid solid = new();
                    bb.MakeSolid(ref solid);
                    bb.Add(solid, shell);
                    newSolids.Append(solid);
                }
                else
                {
                    holeShells.Add(shell);
                    Exploration.MapShapes(shell, ShapeType.Face, mhf);
                }
            }
        }

        if (holeShells.IsEmpty())
        {
            // No holes, stop the analysis
            NCollection_List<Shape>.Iterator itLS0 = new(newSolids);
            for (; itLS0.More(); itLS0.Next())
            {
                Shape sol = itLS0.Value();
                _areas.Append(sol);
                // Build box
                Box box = new();
                Bounds.Add(sol, ref box);
                _boxes[sol] = box;
            }
            return;
        }

        // Classify holes relatively solids

        // Prepare tree with the boxes of the hole shells
        BoxTree bbTree = new();
        int i, nbH = holeShells.Extent();
        bbTree.SetSize(nbH);
        for (i = 1; i <= nbH; ++i)
        {
            Shape hShell = holeShells[i];
            //
            Box box = new();
            Bounds.Add(hShell, ref box);
            bbTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));

            _boxes[hShell] = box;
        }

        // Build BVH
        bbTree.Build();

        // Find outer growth shell that is most close to each hole shell
        NCollection_IndexedDataMap<Shape, Shape> holeSolidMap = new(ShapeHasher.Instance);

        using (var psh = new ProgressScope(mainScope.Next(4), "Adding holes", newSolids.Extent()))
        {
            NCollection_List<Shape>.Iterator itLS1 = new(newSolids);
            for (; itLS1.More(); itLS1.Next(), psh.Next())
            {
                if (UserBreak(psh))
                {
                    return;
                }
                Shape solid = itLS1.Value();

                // Build box
                Box box = new();
                Bounds.Add(solid, ref box);

                _boxes[solid] = box;

                BoxTreeSelector selector = new();
                selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
                selector.SetBVHSet(bbTree);
                selector.Select();

                NCollection_List<int> li = selector.Indices();
                NCollection_List<int>.Iterator itLI = new(li);
                for (; itLI.More(); itLI.Next())
                {
                    int k = itLI.Value();
                    Shape hole = holeShells[k];
                    // Check if it is inside
                    if (!IsInside(hole, solid, _context!))
                    {
                        continue;
                    }

                    // Save the relation
                    int indexWas = holeSolidMap.FindIndex(hole);
                    if (indexWas > 0)
                    {
                        ref Shape pSolidWas = ref holeSolidMap.ChangeFromIndex(indexWas);
                        if (IsInside(solid, pSolidWas, _context!))
                        {
                            pSolidWas = solid;
                        }
                    }
                    else
                    {
                        holeSolidMap.Add(hole, solid);
                    }
                }
            }
        }

        // Make the back map from solids to holes
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> solidHolesMap = new(ShapeHasher.Instance);

        nbH = holeSolidMap.Extent();
        for (i = 1; i <= nbH; ++i)
        {
            Shape hole = holeSolidMap.FindKey(i);
            Shape solid = holeSolidMap[i];
            //
            int index = solidHolesMap.FindIndex(solid);
            NCollection_List<Shape> pLHoles = index > 0
                                                     ? solidHolesMap[index]
                                                     : solidHolesMap[solidHolesMap.Add(solid, new NCollection_List<Shape>())];
            pLHoles.Append(hole);
        }

        // Add Holes to Solids and add them to myAreas
        using (var psu = new ProgressScope(mainScope.Next(), null, newSolids.Extent()))
        {
            NCollection_List<Shape>.Iterator itLS2 = new(newSolids);
            for (; itLS2.More(); itLS2.Next(), psu.Next())
            {
                if (UserBreak(psu))
                {
                    return;
                }
                Solid solid = ShapeCasts.Solid(itLS2.Value());
                if (solidHolesMap.FindFromKey(solid, out NCollection_List<Shape> pLHoles))
                {
                    // update solid
                    NCollection_List<Shape>.Iterator itLH = new(pLHoles);
                    for (; itLH.More(); itLH.Next())
                    {
                        Shape hole = itLH.Value();
                        bb.Add(solid, hole);
                    }

                    // update classifier
                    _context!.SolidClassifier(solid).Load(solid);
                }

                _areas.Append(solid);
            }
        }

        // Add holes that outside the solids to myAreas
        nbH = holeShells.Extent();
        for (i = 1; i <= nbH; ++i)
        {
            Shape hole = holeShells[i];
            if (!holeSolidMap.Contains(hole))
            {
                Solid solid = new();
                bb.MakeSolid(ref solid);
                bb.Add(solid, hole);
                //
                _areas.Append(solid);
                // Make an infinite box for the hole
                Box box = new();
                box.SetWhole();
                _boxes[solid] = box;
            }

            _boxes.Remove(hole);
        }
    }

    /// <summary>Puts the internal shells into the solids containing them.</summary>
    protected override void PerformInternalShapes(ProgressRange range)
    {
        if (_avoidInternalShapes)
        {
            // user-defined option to avoid internal parts is in force
            return;
        }

        if (_loopsInternal.IsEmpty())
        {
            // no internal parts
            return;
        }

        using var mainScope = new ProgressScope(range, "Adding internal shapes", 2);

        // Get all faces to classify
        NCollection_IndexedMap<Shape> mFs = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(_loopsInternal);
        for (; itLS.More(); itLS.Next())
        {
            Shape shell = itLS.Value();
            Iterator it = new(shell);
            for (; it.More(); it.Next())
            {
                mFs.Add(it.Value());
            }
        }

        BrepBuilder bb = new();
        // Check existence of the growths solids
        if (_areas.IsEmpty())
        {
            // No areas.
            // Just make solid of the faces
            Solid solid = new();
            bb.MakeSolid(ref solid);
            //
            NCollection_List<Shape> lsi0 = new();
            MakeInternalShells(mFs, lsi0);
            //
            itLS.Initialize(lsi0);
            for (; itLS.More(); itLS.Next())
            {
                bb.Add(solid, itLS.Value());
            }

            _areas.Append(solid);
            return;
        }

        if (UserBreak(mainScope))
        {
            return;
        }

        // Classify faces relatively solids

        // Prepare list of faces to classify
        NCollection_List<Shape> lFaces = new();
        int i, nbF = mFs.Extent();
        for (i = 1; i <= nbF; ++i)
        {
            lFaces.Append(mFs[i]);
        }

        // Map of solids with IN faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mslf = new(ShapeHasher.Instance);

        // Perform classification
        BuilderTools.ClassifyFaces(lFaces,
                                    _areas,
                                    _runParallel,
                                    _context!,
                                    mslf,
                                    _boxes,
                                    new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance),
                                    mainScope.Next());

        // Update Solids by internal Faces

        HashSet<Shape> mfDone = new(ShapeHasher.Instance);

        int nbS = mslf.Extent();
        using (var psLoop = new ProgressScope(mainScope.Next(), null, nbS))
        {
            for (i = 1; i <= nbS; ++i, psLoop.Next())
            {
                if (UserBreak(psLoop))
                {
                    return;
                }
                Shape pSolid = mslf.FindKey(i);

                NCollection_List<Shape> lf = mslf[i];
                if (lf.IsEmpty())
                {
                    continue;
                }

                NCollection_IndexedMap<Shape> mf = new(ShapeHasher.Instance);
                itLS.Initialize(lf);
                for (; itLS.More(); itLS.Next())
                {
                    Shape f = itLS.Value();
                    mf.Add(f);
                    mfDone.Add(f);
                }
                //
                NCollection_List<Shape> lsi1 = new();
                MakeInternalShells(mf, lsi1);
                //
                itLS.Initialize(lsi1);
                for (; itLS.More(); itLS.Next())
                {
                    Shape si = itLS.Value();
                    bb.Add(pSolid, si);
                }
            }
        }

        // Find all unclassified faces and warn the user about them.
        // Do not put such faces into result as they will form not closed solid.
        NCollection_IndexedMap<Shape> mfUnUsed = new(ShapeHasher.Instance);

        for (i = 1; i <= nbF; ++i)
        {
            Shape f = mFs[i];
            if (!mfDone.Contains(f))
            {
                mfUnUsed.Add(f);
            }
        }

        if (mfUnUsed.Extent() != 0)
        {
            NCollection_List<Shape> lsi = new();
            MakeInternalShells(mfUnUsed, lsi);

            Shape wShape;
            if (lsi.Extent() == 1)
            {
                wShape = lsi.First();
            }
            else
            {
                Compound wc = new();
                bb.MakeCompound(ref wc);
                wShape = wc;
                itLS.Initialize(lsi);
                for (; itLS.More(); itLS.Next())
                {
                    bb.Add(wShape, itLS.Value());
                }
            }

            AddWarning(new AlertSolidBuilderUnusedFaces(wShape));
        }
    }

    // Makes the internal shells of the faces (OCCT's file-static MakeInternalShells).
    private static void MakeInternalShells(NCollection_IndexedMap<Shape> mf, NCollection_List<Shape> shells)
    {
        int i, nbF;
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator itF = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
        HashSet<Shape> addedFacesMap = new(ShapeHasher.Instance);
        //
        nbF = mf.Extent();
        for (i = 1; i <= nbF; ++i)
        {
            Shape f = mf[i];
            Exploration.MapShapesAndAncestors(f, ShapeType.Edge, ShapeType.Face, mef);
        }
        //
        for (i = 1; i <= nbF; ++i)
        {
            Shape ff = mf[i];
            if (!addedFacesMap.Add(ff))
            {
                continue;
            }
            //
            // make a new shell
            Shell shell = new();
            bb.MakeShell(ref shell);
            ff.Orientation(Internal);
            bb.Add(shell, ff);
            //
            Iterator itAddedF = new(shell);
            for (; itAddedF.More(); itAddedF.Next())
            {
                Shape f = itAddedF.Value();
                //
                Explorer edgeExp = new(f, ShapeType.Edge);
                for (; edgeExp.More(); edgeExp.Next())
                {
                    Shape e = edgeExp.Current();
                    NCollection_List<Shape> lf = mef.FindFromKey(e);
                    itF.Initialize(lf);
                    for (; itF.More(); itF.Next())
                    {
                        Shape fl = itF.Value();
                        if (addedFacesMap.Add(fl))
                        {
                            fl.Orientation(Internal);
                            bb.Add(shell, fl);
                        }
                    }
                }
            }
            shell.Closed(Tool.IsClosed(shell));
            shells.Append(shell);
        }
    }

    // True if an infinite point is IN the shell (OCCT's file-static IsHole). OCCT classifies the shell through a
    // TopoDS_Solid reference to it (a reinterpret cast): the port makes the same unchecked solid of the shell's TShape.
    private static bool IsHole(in Shape s2, IntersectionContext context)
    {
        Solid pS2 = new(s2.TShape(), s2.Location(), s2.Orientation());
        SolidClassifier clsf = context.SolidClassifier(pS2);
        //
        clsf.PerformInfinitePoint(RealSmall());
        //
        return (clsf.State() == In);
    }

    // True if theS1 is inside theS2 (OCCT's file-static IsInside); theS2 is a solid, or (from IsHole's callers) a
    // shell cast to one.
    private static bool IsInside(in Shape s1, in Shape s2, IntersectionContext context)
    {
        Explorer exp = new();
        Containment state;
        //
        Solid pS2 = new(s2.TShape(), s2.Location(), s2.Orientation());
        //
        exp.Init(s1, ShapeType.Face);
        if (!exp.More())
        {
            SolidClassifier clsf = context.SolidClassifier(pS2);
            clsf.PerformInfinitePoint(RealSmall());
            state = clsf.State();
        }
        else
        {
            NCollection_IndexedMap<Shape> bounds = new(ShapeHasher.Instance);
            Exploration.MapShapes(pS2, ShapeType.Edge, bounds);
            Face f = ShapeCasts.Face(exp.Current());
            state = AlgoTools.ComputeState(f, pS2, global::Kerf.Engine.Foundation.Precision.Confusion(), bounds, context);
        }
        return (state == In);
    }

    // True if the shell holds one of the hole faces (OCCT's file-static IsGrowthShell).
    private static bool IsGrowthShell(in Shape shell, NCollection_IndexedMap<Shape> mhf)
    {
        if (mhf.Extent() != 0)
        {
            Iterator it = new(shell);
            for (; it.More(); it.Next())
            {
                if (mhf.Contains(it.Value()))
                {
                    return true;
                }
            }
        }
        return false;
    }
}
