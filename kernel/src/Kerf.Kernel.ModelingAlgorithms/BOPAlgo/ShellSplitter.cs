// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>Builds the loops (shells) from a set of faces.</summary>
/// <remarks>
/// The irregular connexity blocks are split in parallel (BOPAlgo_CBK solvers, each changing its own block's loops)
/// when <see cref="Options.SetRunParallel"/> is set. The allocator constructor goes.
/// </remarks>
public class ShellSplitter : BooleanAlgorithm
{
    protected NCollection_List<Shape> _startShapes = new();
    protected NCollection_List<Shape> _shells = new();
    protected NCollection_List<ConnexityBlock> _lcb = new();

    public ShellSplitter()
    {
    }

    /// <summary>Adds a face to process.</summary>
    public void AddStartElement(in Shape aE) => _startShapes.Append(aE);

    /// <summary>The faces to process.</summary>
    public NCollection_List<Shape> StartElements() => _startShapes;

    /// <summary>The shells built.</summary>
    public NCollection_List<Shape> Shells() => _shells;

    /// <summary>Builds the shells.</summary>
    public override void Perform(ProgressRange range = default)
    {
        GetReport().Clear();
        using var ps = new ProgressScope(range, "Building shells", 1);
        //
        AlgoTools.MakeConnexityBlocks(_startShapes, ShapeType.Edge, ShapeType.Face, _lcb);
        if (UserBreak(ps))
        {
            return;
        }

        MakeShells(ps.Next());
    }

    /// <summary>Splits the (irregular) connexity block into shells, put into its loops.</summary>
    public static void SplitBlock(ConnexityBlock aCB)
    {
        int nbLF, nbOff, nbFP;
        int i;
        Orientation or;
        Edge el = new();
        BrepBuilder bb = new();
        Iterator itS = new();
        Explorer exp = new();
        NCollection_List<Shape>.Iterator itF = new();
        CoupleOfShape csOff = new();
        HashSet<Shape> addedFacesMap = new(EqualityComparer<Shape>.Default);
        IndexedMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance),
                                                                                  mefp = new(ShapeHasher.Instance);
        IntersectionContext context;
        //
        context = new IntersectionContext();
        //
        NCollection_List<Shape> myShapes = aCB.Shapes();
        //
        NCollection_List<Shape> myLoops = aCB.ChangeLoops();
        myLoops.Clear();
        //
        // Copy faces into the map, for recursive search of free bounds
        HashSet<Shape> mFaces = new(EqualityComparer<Shape>.Default);
        itF.Initialize(myShapes);
        for (; itF.More(); itF.Next())
        {
            mFaces.Add(itF.Value());
        }
        //
        // remove the faces with free edges from processing
        for (;;)
        {
            // map the shapes
            efMap.Clear();
            itF.Initialize(myShapes);
            for (; itF.More(); itF.Next())
            {
                Shape f = itF.Value();
                if (mFaces.Contains(f))
                {
                    Exploration.MapShapesAndAncestors(f, ShapeType.Edge, ShapeType.Face, efMap);
                }
            }
            //
            int nbBegin = mFaces.Count;
            // check the free edges
            int nbE = efMap.Count;
            for (i = 1; i <= nbE; ++i)
            {
                Edge e = ShapeCasts.Edge(efMap.KeyAt(i - 1));
                if (!Tool.Degenerated(e) && e.Orientation() != Internal)
                {
                    NCollection_List<Shape> lf = efMap.ValueAt(i - 1);
                    if (lf.Extent() == 1)
                    {
                        // remove the face
                        mFaces.Remove(lf.First());
                    }
                }
            }
            //
            // check if any faces have been removed
            int nbEnd = mFaces.Count;
            if ((nbEnd == nbBegin) || (nbEnd == 0))
            {
                break;
            }
        }
        //
        if (mFaces.Count == 0)
        {
            return;
        }
        //
        // use only connected faces
        NCollection_List<Shape> lfConnected = new();
        // Boundary faces
        HashSet<Shape> boundaryFaces = new(ShapeHasher.Instance);
        itF.Initialize(myShapes);
        for (; itF.More(); itF.Next())
        {
            Shape f = itF.Value();
            if (mFaces.Contains(f))
            {
                lfConnected.Append(f);
                if (!boundaryFaces.Add(f))
                {
                    boundaryFaces.Remove(f);
                }
            }
        }
        //
        int nbShapes = lfConnected.Extent();
        bool bAllFacesTaken = false;
        //
        // Build the shells
        itF.Initialize(lfConnected);
        for (i = 1; itF.More() && !bAllFacesTaken; itF.Next(), ++i)
        {
            Shape ff = itF.Value();
            if (!addedFacesMap.Add(ff))
            {
                continue;
            }
            //
            // make a new shell
            Shell shell = new();
            bb.MakeShell(ref shell);
            bb.Add(shell, ff);

            mefp.Clear();
            Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, mefp);
            //
            // loop on faces added to Shell;
            // add their neighbor faces to Shell and so on
            itS.Initialize(shell);
            for (; itS.More(); itS.Next())
            {
                Face f = ShapeCasts.Face(itS.Value());
                bool isBoundary = boundaryFaces.Contains(f);
                //
                // loop on edges of aF; find a good neighbor face of aF by aE
                exp.Init(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Edge e = ShapeCasts.Edge(exp.Current());
                    //
                    // proceed only free edges in this shell
                    if (mefp.ContainsKey(e))
                    {
                        NCollection_List<Shape> lfp = mefp.GetValue(e);
                        nbFP = lfp.Extent();
                        if (nbFP > 1)
                        {
                            continue;
                        }
                    }
                    // avoid processing of internal edges
                    or = e.Orientation();
                    if (or == Internal)
                    {
                        continue;
                    }
                    // avoid processing of degenerated edges
                    if (Tool.Degenerated(e))
                    {
                        continue;
                    }
                    //
                    // candidate faces list
                    NCollection_List<Shape> lf = efMap.GetValue(e);
                    nbLF = lf.Extent();
                    if (nbLF == 0)
                    {
                        continue;
                    }
                    //
                    // prepare for selecting the next face
                    // take only not-processed faces as a candidates
                    NCollection_List<CoupleOfShape> lcsOff = new();
                    //
                    int nbWaysInside = 0;
                    Face selF = new();
                    NCollection_List<Shape>.Iterator itLF = new(lf);
                    for (; itLF.More(); itLF.Next())
                    {
                        Face fl = ShapeCasts.Face(itLF.Value());
                        if (f.IsSame(fl) || addedFacesMap.Contains(fl))
                        {
                            continue;
                        }
                        //
                        // find current edge in the face
                        if (!AlgoTools.GetEdgeOff(e, fl, ref el))
                        {
                            continue;
                        }
                        //
                        if (isBoundary && !boundaryFaces.Contains(fl))
                        {
                            ++nbWaysInside;
                            selF = fl;
                        }
                        csOff.SetShape1(el);
                        csOff.SetShape2(fl);
                        lcsOff.Append(csOff);
                    } // for (; aItLF.More(); aItLF.Next()) {
                    //
                    nbOff = lcsOff.Extent();
                    if (nbOff == 0)
                    {
                        continue;
                    }
                    //
                    // among all the adjacent faces chose one with the minimal
                    // angle to the current one
                    if (!isBoundary || nbWaysInside != 1)
                    {
                        if (nbOff == 1)
                        {
                            selF = ShapeCasts.Face(lcsOff.First().Shape2());
                        }
                        else if (nbOff > 1)
                        {
                            AlgoTools.GetFaceOff(e, f, lcsOff, ref selF, context);
                        }
                    }
                    //
                    if (!selF.IsNull && addedFacesMap.Add(selF))
                    {
                        bb.Add(shell, selF);
                        Exploration.MapShapesAndAncestors(selF, ShapeType.Edge, ShapeType.Face, mefp);
                    }
                } // for (; aExp.More(); aExp.Next()) {
            } // for (; aItS.More(); aItS.Next()) {
            //
            // split the shell on multi-connected edges
            NCollection_List<Shape> lShSp = new();
            RefineShell(shell, mefp, lShSp);
            //
            // collect the not closed shells for further processing
            NCollection_List<Shape> lShNC = new();
            //
            NCollection_List<Shape>.Iterator itLShSp = new(lShSp);
            for (; itLShSp.More(); itLShSp.Next())
            {
                Shell shSp = ShapeCasts.Shell(itLShSp.Value());
                //
                if (Tool.IsClosed(shSp))
                {
                    shSp.Closed(true);
                    myLoops.Append(shSp);
                }
                else
                {
                    lShNC.Append(shSp);
                }
            }
            //
            bAllFacesTaken = (addedFacesMap.Count == nbShapes);
            if (bAllFacesTaken)
            {
                break;
            }
            //
            if (lShSp.Extent() == 1)
            {
                // not further processing of not closed shells is needed,
                // as it will not bring any new results
                continue;
            }
            //

            // remove th faces of not closed shells from the map of processed faces
            // and try to rebuild the shells using all not processed faces,
            // because faces of one shell might be needed for building the other
            NCollection_List<Shape>.Iterator itLShNC = new(lShNC);
            for (; itLShNC.More(); itLShNC.Next())
            {
                Iterator itNC = new(itLShNC.Value());
                for (; itNC.More(); itNC.Next())
                {
                    addedFacesMap.Remove(itNC.Value());
                }
            }
        } // for (; aItF.More(); aItF.Next()) {
    }

    // The sub-shape of theShape that is the same as theShapeToFind (OCCT's file-level FindShape).
    private static Shape FindShape(in Shape shapeToFind, in Shape shape)
    {
        Shape res = new();
        Explorer exp = new(shape, shapeToFind.ShapeType());
        for (; exp.More(); exp.Next())
        {
            Shape aShape = exp.Current();
            if (aShape.IsSame(shapeToFind))
            {
                res = aShape;
                break;
            }
        }
        return res;
    }

    // Splits the shell on its branch edges (OCCT's file-static RefineShell).
    private static void RefineShell(in Shell shell,
                                    IndexedMap<Shape, NCollection_List<Shape>> mef,
                                    NCollection_List<Shape> lShSp)
    {
        Iterator it = new(shell);
        if (!it.More())
        {
            return;
        }
        //
        // Find edges with more than 2 adjacent faces - branch edges -
        // edges on which the input shell should be split
        HashSet<Shape> meStop = new(ShapeHasher.Instance);
        //
        int i, nbMEF = mef.Count;
        for (i = 1; i <= nbMEF; ++i)
        {
            Edge e = ShapeCasts.Edge(mef.KeyAt(i - 1));
            NCollection_List<Shape> lf = mef.ValueAt(i - 1);
            if (lf.Extent() > 2)
            {
                meStop.Add(e);
                continue;
            }

            if (lf.Extent() == 2)
            {
                Face f1 = ShapeCasts.Face(lf.First());
                Face f2 = ShapeCasts.Face(lf.Last());

                Shape e1 = FindShape(e, f1);
                Shape e2 = FindShape(e, f2);

                if (e1.Orientation() == e2.Orientation())
                {
                    meStop.Add(e);
                    continue;
                }
            }
            //
            // check for internal edges - count faces, in which the edge
            // is internal, twice
            int nbF = 0;
            NCollection_List<Shape>.Iterator itLF0 = new(lf);
            for (; itLF0.More() && nbF <= 2; itLF0.Next())
            {
                Face f = ShapeCasts.Face(itLF0.Value());
                ++nbF;
                Explorer exp0 = new(f, ShapeType.Edge);
                for (; exp0.More(); exp0.Next())
                {
                    Shape ef = exp0.Current();
                    if (ef.IsSame(e))
                    {
                        if (ef.Orientation() == Internal)
                        {
                            ++nbF;
                        }
                        break;
                    }
                }
            }
            //
            if (nbF > 2)
            {
                meStop.Add(e);
            }
        }
        //
        if (meStop.Count == 0)
        {
            lShSp.Append(shell);
            return;
        }
        //
        Builder bb = new();
        Explorer exp = new();
        IndexedSet<Shape> mfb = new(ShapeHasher.Instance);
        HashSet<Shape> mfProcessed = new(EqualityComparer<Shape>.Default);
        NCollection_List<Shape> lfp = new(), lfp1 = new();
        NCollection_List<Shape>.Iterator itLF = new(), itLFP = new();
        //
        // The first Face
        for (; it.More(); it.Next())
        {
            Shape f1 = it.Value();
            if (!mfProcessed.Add(f1))
            {
                continue;
            }
            //
            mfb.Clear();
            lfp.Clear();
            //
            mfb.Add(f1);
            lfp.Append(f1);
            //
            // Trying to reach the branch point
            for (;;)
            {
                itLFP.Initialize(lfp);
                for (; itLFP.More(); itLFP.Next())
                {
                    Shape fp = itLFP.Value();
                    //
                    exp.Init(fp, ShapeType.Edge);
                    for (; exp.More(); exp.Next())
                    {
                        Edge e = ShapeCasts.Edge(exp.Current());
                        if (meStop.Contains(e))
                        {
                            continue;
                        }
                        //
                        if (e.Orientation() == Internal)
                        {
                            continue;
                        }
                        //
                        if (Tool.Degenerated(e))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape> lf = mef.GetValue(e);
                        //
                        itLF.Initialize(lf);
                        for (; itLF.More(); itLF.Next())
                        {
                            Shape fp1 = itLF.Value();
                            if (fp1.IsSame(fp))
                            {
                                continue;
                            }
                            if (mfb.Contains(fp1))
                            {
                                continue;
                            }
                            //
                            if (mfProcessed.Add(fp1))
                            {
                                mfb.Add(fp1);
                                lfp1.Append(fp1);
                            }
                        } // for (; aItLF.More(); aItLF.Next()) {
                    } // for (; aExp.More(); aExp.Next()) {
                } // for (; aItLFP.More(); aItLFP.Next()) {
                //
                //
                if (lfp1.IsEmpty())
                {
                    break;
                }
                //
                lfp.Clear();
                lfp.Append(lfp1);
            } // for (;;) {
            //
            int nbMFB = mfb.Count;
            if (nbMFB != 0)
            {
                Shell shSp = new();
                bb.MakeShell(ref shSp);
                //
                for (i = 1; i <= nbMFB; ++i)
                {
                    Shape fb = mfb[i - 1];
                    bb.Add(shSp, fb);
                }
                lShSp.Append(shSp);
            }
        } // for (; aIt.More(); aIt.Next()) {
    }

    /// <summary>Makes a shell of each regular block, and splits the others (in parallel).</summary>
    protected void MakeShells(ProgressRange range)
    {
        bool bIsRegular;
        int nbVCBK, k;
        NCollection_List<ConnexityBlock>.Iterator itCB = new();
        NCollection_List<Shape>.Iterator it = new();
        List<CBK> vcbk = new();
        //
        using var psOuter = new ProgressScope(range, null, 1);
        _shells.Clear();
        //
        itCB.Initialize(_lcb);
        for (; itCB.More(); itCB.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            ConnexityBlock cb = itCB.Value();
            bIsRegular = cb.IsRegular;
            if (bIsRegular)
            {
                Shell shell = new();
                //
                NCollection_List<Shape> lf = cb.Shapes();
                MakeShell(lf, ref shell);
                shell.Closed(true);
                _shells.Append(shell);
            }
            else
            {
                CBK cbk =  new CBK();
                vcbk.Add(cbk);
                cbk.SetConnexityBlock(cb);
            }
        }
        //
        nbVCBK = vcbk.Count;
        using var psParallel = new ProgressScope(psOuter.Next(), null, nbVCBK);
        for (int iS = 0; iS < nbVCBK; ++iS)
        {
            vcbk[iS].SetProgressRange(psParallel.Next());
        }
        //===================================================
        BooleanParallel.Perform(_runParallel, vcbk);
        //===================================================
        for (k = 0; k < nbVCBK; ++k)
        {
            CBK cbk = vcbk[k];
            ConnexityBlock cb = cbk.ConnexityBlock();
            NCollection_List<Shape> ls = cb.Loops();
            it.Initialize(ls);
            for (; it.More(); it.Next())
            {
                Shape shell = it.Value();
                shell.Closed(true);
                _shells.Append(shell);
            }
        }
    }

    // Makes a shell of the faces and orients them (OCCT's file-static MakeShell).
    private static void MakeShell(NCollection_List<Shape> aLS, ref Shell aShell)
    {
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator it = new();
        //
        bb.MakeShell(ref aShell);
        //
        it.Initialize(aLS);
        for (; it.More(); it.Next())
        {
            Shape f = it.Value();
            bb.Add(aShell, f);
        }
        //
        Shape sh = aShell;
        AlgoTools.OrientFacesOnShell(ref sh);
        aShell = ShapeCasts.Shell(sh);
    }
}

/// <summary>The solver splitting one irregular connexity block (OCCT's file-local BOPAlgo_CBK).</summary>
/// <remarks>It refers to the splitter's own block, whose loops <see cref="ShellSplitter.SplitBlock"/> fills.</remarks>
internal sealed class CBK : BooleanParallel.ISolver
{
    private ConnexityBlock? _pcb;
    private ProgressRange _progressRange;

    public CBK()
    {
        _pcb = null;
    }

    //
    public void SetConnexityBlock(ConnexityBlock aCB) => _pcb = aCB;

    //
    public ConnexityBlock ConnexityBlock() => _pcb!;

    //
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    //
    public void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (!ps.More())
        {
            return;
        }
        ShellSplitter.SplitBlock(_pcb!);
    }
}
