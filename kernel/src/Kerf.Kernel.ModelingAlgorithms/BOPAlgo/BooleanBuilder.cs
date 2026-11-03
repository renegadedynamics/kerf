// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The General Fuse algorithm: builds the split parts of the arguments from the intersections computed by a pave
/// filler, and the history of the operation.
/// </summary>
/// <remarks>
/// <para>
/// The result is a compound of the split arguments, of the same dimension as the arguments, sharing their common
/// parts (same-domain vertices, edges, faces and solids are represented by one shape). The images map
/// (<see cref="Images"/>) gives the splits of every argument sub-shape, <see cref="Origins"/> the reverse, and
/// <see cref="ShapesSD"/> the same-domain shapes.
/// </para>
/// <para>
/// The class is partial over OCCT's files: this one, <c>_1</c> (vertices, edges, containers, compounds), <c>_2</c>
/// (faces), <c>_3</c> (solids) and <c>_4</c> (history). OCCT's builder deletes the pave filler it made itself
/// (<c>myEntryPoint == 1</c>); the port lets it go.
/// </para>
/// </remarks>
public partial class BooleanBuilder : BuilderShape
{
    protected NCollection_List<Shape> _arguments = new();                                                     //!< Arguments of the operation
    protected HashSet<Shape> _mapFence = new(ShapeHasher.Instance);                         //!< Fence map providing the uniqueness of the shapes in the list of arguments
    protected PaveFiller? _paveFiller;                                                                         //!< Pave Filler - algorithm for sub-shapes intersection
    protected DS? _ds;                                                                                           //!< Data Structure - holder of intersection information
    protected IntersectionContext? _context;                                                                              //!< Context - tool for cashing heavy algorithms such as Projectors and Classifiers
    protected int _entryPoint;                                                                                         //!< EntryPoint - controls the deletion of the PaveFiller, which could live longer than the Builder
    protected Dictionary<Shape, NCollection_List<Shape>> _images = new(ShapeHasher.Instance);  //!< Images - map of Images of the sub-shapes of arguments
    protected Dictionary<Shape, Shape> _shapesSD = new(ShapeHasher.Instance);                  //!< ShapesSD - map of SD Shapes
    protected Dictionary<Shape, NCollection_List<Shape>> _origins = new(ShapeHasher.Instance); //!< Origins - map of Origins, back map of Images
    protected Dictionary<Shape, NCollection_List<Shape>> _inParts = new(ShapeHasher.Instance); //!< InParts - map of own and acquired IN faces of the arguments solids
    protected bool _nonDestructive;                                                                                    //!< Safe processing option allows avoiding modification of the input shapes
    protected global::Kerf.Booleans.GlueMode _glue;                                                                                  //!< Gluing option allows speeding up the intersection of the input shapes
    protected bool _checkInverted;                                                                                     //!< Check inverted option allows disabling the check of input solids on inverted status

    public BooleanBuilder()
    {
        _paveFiller = null;
        _ds = null;
        _entryPoint = 0;
        _nonDestructive = false;
        _glue = global::Kerf.Booleans.GlueMode.Off;
        _checkInverted = true;
    }

    /// <summary>Clears the content of the algorithm.</summary>
    public override void Clear()
    {
        base.Clear();
        _arguments.Clear();
        _mapFence.Clear();
        _images.Clear();
        _shapesSD.Clear();
        _origins.Clear();
        _inParts.Clear();
    }

    /// <summary>The pave filler used.</summary>
    internal PaveFiller? PPaveFiller() => _paveFiller;

    /// <summary>The data structure used.</summary>
    internal DS? PDS() => _ds;

    /// <summary>The context used.</summary>
    internal IntersectionContext? Context() => _context;

    //! @name Arguments

    /// <summary>Adds an argument (once).</summary>
    /// <remarks>
    /// As in OCCT, the fence of added arguments is cleared only by <see cref="Clear"/>, not by
    /// <see cref="SetArguments"/>: setting again arguments added before leaves them out (occt-issues.md, unit 6A).
    /// </remarks>
    public virtual void AddArgument(in Shape shape)
    {
        if (_mapFence.Add(shape))
        {
            _arguments.Append(shape);
        }
    }

    /// <summary>Sets the arguments.</summary>
    public virtual void SetArguments(NCollection_List<Shape> shapes)
    {
        NCollection_List<Shape>.Iterator it = new();
        //
        _arguments.Clear();
        // OCCT: myMapFence isn't cleared (occt-issues.md, unit 6A)
        //
        it.Initialize(shapes);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            AddArgument(s);
        }
    }

    /// <summary>The arguments.</summary>
    public NCollection_List<Shape> Arguments() => _arguments;

    //! @name Options

    /// <summary>Sets the flag that defines the mode of treatment: in non-destructive mode the arguments are not modified.</summary>
    public void SetNonDestructive(bool flag) => _nonDestructive = flag;

    /// <summary>The flag defining the mode of treatment.</summary>
    public bool NonDestructive() => _nonDestructive;

    /// <summary>Sets the glue option.</summary>
    public void SetGlue(global::Kerf.Booleans.GlueMode glue) => _glue = glue;

    /// <summary>The glue option.</summary>
    public global::Kerf.Booleans.GlueMode Glue() => _glue;

    /// <summary>Enables or disables the check of the input solids for being inverted.</summary>
    public void SetCheckInverted(bool check) => _checkInverted = check;

    /// <summary>The flag of the check of the input solids for being inverted.</summary>
    public bool CheckInverted() => _checkInverted;

    //! @name Performing the operation

    /// <summary>Intersects the arguments with a new pave filler and builds the result.</summary>
    public override void Perform(ProgressRange range = default)
    {
        GetReport().Clear();
        //
        if (_entryPoint == 1)
        {
            _paveFiller = null;
        }
        //
        PaveFiller pPF = new();
        //
        pPF.SetArguments(_arguments);
        pPF.SetRunParallel(_runParallel);
        using var ps = new ProgressScope(range, "Performing General Fuse operation", 10);
        pPF.SetFuzzyValue(_fuzzyValue);
        pPF.SetNonDestructive(_nonDestructive);
        pPF.SetGlue(_glue);
        pPF.SetUseOBB(_useOBB);
        //
        pPF.Perform(ps.Next(9));
        //
        _entryPoint = 1;
        PerformInternal(pPF, ps.Next(1));
    }

    /// <summary>Builds the result from the intersections of <paramref name="filler"/>.</summary>
    public virtual void PerformWithFiller(PaveFiller filler, ProgressRange range = default)
    {
        GetReport().Clear();
        _entryPoint = 0;
        _nonDestructive = filler.NonDestructive();
        _fuzzyValue = filler.FuzzyValue();
        _glue = filler.Glue();
        _useOBB = filler.UseOBB();
        PerformInternal(filler, range);
    }

    //! @name BOPs on open solids

    /// <summary>
    /// Builds the result of a Boolean operation from the splits of the given solids (or compounds of solids), each
    /// group taken in the given state relatively the other (IN or OUT), as the General Fuse built them.
    /// </summary>
    /// <remarks>
    /// Works for open solids too, and keeps the internal parts of the solids where the operation keeps IN faces.
    /// Errors go to <paramref name="report"/> (the algorithm's own report when null).
    /// </remarks>
    public virtual void BuildBOP(NCollection_List<Shape> objects, Containment objState,
                                 NCollection_List<Shape> tools, Containment toolsState,
                                 ProgressRange range, Report? report = null)
    {
        if (HasErrors)
        {
            return;
        }

        // Report for the method
        Report aReport = report is null ? _report : report;

        if (_arguments.IsEmpty() || _shape.IsNull)
        {
            aReport.AddAlert(Fail, new AlertBuilderFailed());
            return;
        }
        // Check the input data
        if ((objState != In && objState != Out)
            || (toolsState != In && toolsState != Out))
        {
            aReport.AddAlert(Fail, new AlertBopNotSet());
            return;
        }

        // Check input shapes
        bool hasObjects = !objects.IsEmpty();
        bool hasTools = !tools.IsEmpty();
        if (!hasObjects && !hasTools)
        {
            aReport.AddAlert(Fail, new AlertTooFewArguments());
            return;
        }

        // Check that all input solids are from the arguments
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> list = i == 0 ? objects : tools;
            NCollection_List<Shape>.Iterator itLS = new(list);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                // Check if the shape belongs to the arguments of operation
                if (_ds!.Index(s) < 0)
                {
                    aReport.AddAlert(Fail, new AlertUnknownShape(s));
                    return;
                }

                // Check if the shape is a solid or collection of them
                if (s.Type != ShapeType.Solid)
                {
                    NCollection_List<Shape> ls = new();
                    HashSet<Shape> mFence0 = new(ShapeHasher.Instance);
                    AlgoTools.TreatCompound(s, ls, mFence0);

                    NCollection_List<Shape>.Iterator it = new(ls);
                    for (; it.More(); it.Next())
                    {
                        Shape sx = it.Value();
                        if (sx.Type != ShapeType.Solid && sx.Type != ShapeType.CompSolid)
                        {
                            aReport.AddAlert(Fail, new AlertUnsupportedType(s));
                            return;
                        }
                    }
                }
            }
        }

        // Classification of the faces relatively solids has been made
        // on the stage of Solids splitting. All results are saved into
        // myInParts map, which connects the solids with its IN faces from
        // other arguments. All faces not contained in the list of IN faces
        // will be considered as OUT.

        // Prepare the maps of splits of solids faces with orientations (OCCT's default hasher: orientation counts)
        IndexedSet<Shape> mObjFacesOri = new(EqualityComparer<Shape>.Default),
                                             mToolFacesOri = new(EqualityComparer<Shape>.Default);
        // Prepare the maps of splits of solids faces
        IndexedSet<Shape> mObjFaces = new(ShapeHasher.Instance),
                                             mToolFaces = new(ShapeHasher.Instance);
        // Copy the list of IN faces of the solids into map
        HashSet<Shape> inObjects = new(ShapeHasher.Instance),
                                      inTools = new(ShapeHasher.Instance);

        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> list = i == 0 ? objects : tools;
            IndexedSet<Shape> mapOri = i == 0 ? mObjFacesOri : mToolFacesOri;
            IndexedSet<Shape> map = i == 0 ? mObjFaces : mToolFaces;
            NCollection_List<Shape>.Iterator itLS = new(list);
            for (; itLS.More(); itLS.Next())
            {
                Shape shape = itLS.Value();
                Explorer expS = new(shape, ShapeType.Solid);
                for (; expS.More(); expS.Next())
                {
                    Shape s = expS.Current();
                    Explorer expF = new(s, ShapeType.Face);
                    for (; expF.More(); expF.Next())
                    {
                        Shape f = expF.Current();
                        if (f.Orientation != Forward && f.Orientation != Reversed)
                        {
                            continue;
                        }
                        if (_images.TryGetValue(f, out NCollection_List<Shape>? pLFIm))
                        {
                            NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                            for (; itLFIm.More(); itLFIm.Next())
                            {
                                Face fImRef = ShapeCasts.Face(itLFIm.Value());
                                if (AlgoTools.IsSplitToReverse(fImRef, f, _context))
                                {
                                    Face fIm = fImRef;
                                    fIm.Reverse();
                                    mapOri.Add(fIm);
                                    map.Add(fIm);
                                }
                                else
                                {
                                    mapOri.Add(fImRef);
                                    map.Add(fImRef);
                                }
                            }
                        }
                        else
                        {
                            mapOri.Add(f);
                            map.Add(f);
                        }
                    }

                    // Copy the list of IN faces into a map
                    if (_inParts.TryGetValue(s, out NCollection_List<Shape>? pLFIN))
                    {
                        HashSet<Shape> inMap = i == 0 ? inObjects : inTools;
                        NCollection_List<Shape>.Iterator itLFIn = new(pLFIN);
                        for (; itLFIn.More(); itLFIn.Next())
                        {
                            inMap.Add(itLFIn.Value());
                        }
                    }
                }
            }
        }

        // Now we need to select all faces which will participate in
        // building of the resulting solids. The final set of faces
        // depends on the given states for the groups.
        bool isObjectsIN = objState == In, isToolsIN = toolsState == In;

        // Shortcuts
        bool bAvoidIN = !isObjectsIN && !isToolsIN,  // avoid all in faces
             bAvoidINforBoth = isObjectsIN != isToolsIN; // avoid faces IN for both groups

        // Choose which SD faces are needed to be taken - equally or differently oriented faces
        bool isSameOriNeeded = objState == toolsState;
        // Resulting faces
        IndexedSet<Shape> mResFacesOri = new(EqualityComparer<Shape>.Default);
        HashSet<Shape> mResFacesFence = new(ShapeHasher.Instance);
        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance),
                                      mfToAvoid = new(ShapeHasher.Instance);
        // Oriented fence map
        HashSet<Shape> mFenceOri = new(EqualityComparer<Shape>.Default);

        for (int i = 0; i < 2; ++i)
        {
            IndexedSet<Shape> map = i == 0 ? mObjFacesOri : mToolFacesOri;
            IndexedSet<Shape> oppositeMap = i == 0 ? mToolFaces : mObjFaces;
            HashSet<Shape> inMap = i == 0 ? inObjects : inTools;
            HashSet<Shape> oppositeINMap = i == 0 ? inTools : inObjects;
            bool bTakeIN = i == 0 ? isObjectsIN : isToolsIN;

            int nbF = map.Count;
            for (int j = 0; j < nbF; ++j)
            {
                Shape fIm = map[j];

                bool isIN = inMap.Contains(fIm);
                bool isINOpposite = oppositeINMap.Contains(fIm);

                // Filtering for FUSE - avoid any IN faces
                if (bAvoidIN && (isIN || isINOpposite))
                {
                    continue;
                }

                // Filtering for CUT - avoid faces IN for both groups
                if (bAvoidINforBoth && isIN && isINOpposite)
                {
                    continue;
                }

                // Treatment of SD faces
                if (!mFence.Add(fIm))
                {
                    if (!oppositeMap.Contains(fIm))
                    {
                        // The face belongs to only one group
                        if (bTakeIN != isSameOriNeeded)
                        {
                            mfToAvoid.Add(fIm);
                        }
                    }
                    else
                    {
                        // The face belongs to both groups.
                        // Using its orientation decide if it is needed in the result or not.
                        bool isSameOri = !mFenceOri.Add(fIm);
                        if (isSameOriNeeded == isSameOri)
                        {
                            // Take the shape without classification
                            if (mResFacesFence.Add(fIm))
                            {
                                mResFacesOri.Add(fIm);
                            }
                        }
                        else
                        {
                            // Remove the face
                            mfToAvoid.Add(fIm);
                        }

                        continue;
                    }
                }
                if (!mFenceOri.Add(fIm))
                {
                    continue;
                }

                if (bTakeIN == isINOpposite)
                {
                    if (isIN)
                    {
                        mResFacesOri.Add(fIm);
                        mResFacesOri.Add(fIm.Reversed());
                    }
                    else if (bTakeIN && !isSameOriNeeded)
                    {
                        mResFacesOri.Add(fIm.Reversed());
                    }
                    else
                    {
                        mResFacesOri.Add(fIm);
                    }
                    mResFacesFence.Add(fIm);
                }
            }
        }

        // Remove the faces which has to be avoided
        NCollection_List<Shape> resFaces = new();
        int nbRF = mResFacesOri.Count;
        for (int i = 0; i < nbRF; ++i)
        {
            Shape rf = mResFacesOri[i];
            if (!mfToAvoid.Contains(rf))
            {
                resFaces.Append(rf);
            }
        }
        using var ps = new ProgressScope(range, null, 2);
        BrepBuilder bb = new();

        // Try to build closed solids from the faces
        BuilderSolid bs = new();
        bs.SetShapes(resFaces);
        bs.SetRunParallel(_runParallel);
        bs.SetContext(_context);
        bs.SetFuzzyValue(_fuzzyValue);
        bs.Perform(ps.Next());

        // Resulting solids
        NCollection_List<Shape> resSolids = new();

        mFence.Clear();
        if (!bs.HasErrors)
        {
            // If any, add solids into resulting compound
            NCollection_List<Shape>.Iterator itA = new(bs.Areas());
            for (; itA.More(); itA.Next())
            {
                Shape solid = itA.Value();
                // The solid must contain at least one face
                // from either of objects or tools
                Explorer expF = new(solid, ShapeType.Face);
                for (; expF.More(); expF.Next())
                {
                    Shape f = expF.Current();
                    if (mObjFacesOri.Contains(f) || mToolFacesOri.Contains(f))
                    {
                        break;
                    }
                }
                if (expF.More())
                {
                    resSolids.Append(solid);
                    Exploration.MapShapes(solid, mFence);
                }
            }
        }
        else
        {
            return;
        }

        // Collect unused faces
        Compound unUsedFaces = new();
        bb.MakeCompound(ref unUsedFaces);

        NCollection_List<Shape>.Iterator itLF = new(resFaces);
        for (; itLF.More(); itLF.Next())
        {
            if (mFence.Add(itLF.Value()))
            {
                bb.Add(unUsedFaces, itLF.Value());
            }
        }

        // Build blocks from the unused faces
        NCollection_List<Shape> lcb = new();
        AlgoTools.MakeConnexityBlocks(unUsedFaces, ShapeType.Edge, ShapeType.Face, lcb);

        // Build solid from each block
        NCollection_List<Shape>.Iterator itCB = new(lcb);
        for (; itCB.More(); itCB.Next())
        {
            Shape cb = itCB.Value();
            Shell shell0 = new();
            bb.MakeShell(ref shell0);
            Shape shell = shell0;
            // Add faces of the block to the shell
            Explorer anExpF = new(cb, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                bb.Add(shell, anExpF.Current());
            }

            AlgoTools.OrientFacesOnShell(ref shell);
            // Make solid out of the shell
            Solid solid = new();
            bb.MakeSolid(ref solid);
            bb.Add(solid, shell);
            // Add new solid to result
            resSolids.Append(solid);
        }

        if (!bAvoidIN)
        {
            // Fill solids with internal parts coming with the solids
            NCollection_List<Shape> inParts = new();
            for (int i = 0; i < 2; ++i)
            {
                NCollection_List<Shape> list = i == 0 ? objects : tools;
                NCollection_List<Shape>.Iterator itLS = new(list);
                for (; itLS.More(); itLS.Next())
                {
                    Explorer expS = new(itLS.Value(), ShapeType.Solid);
                    for (; expS.More(); expS.Next())
                    {
                        Shape s = expS.Current(); // Solid
                        for (Iterator it = new(s); it.More(); it.Next())
                        {
                            Shape sInt = it.Value();
                            if (sInt.Orientation == Internal)
                            {
                                inParts.Append(sInt); // vertex or edge
                            }
                            else
                            {
                                // shell treatment
                                Iterator itInt = new(sInt);
                                if (itInt.More() && itInt.Value().Orientation == Internal)
                                {
                                    inParts.Append(sInt);
                                }
                            }
                        }
                    }
                }
            }

            BuilderTools.FillInternals(resSolids, inParts, _images, _context!);
        }

        // Combine solids into compound
        Compound result0 = new();
        bb.MakeCompound(ref result0);
        Shape result = result0;

        NCollection_List<Shape>.Iterator itLS2 = new(resSolids);
        for (; itLS2.More(); itLS2.Next())
        {
            bb.Add(result, itLS2.Value());
        }

        _shape = result;
        PrepareHistory(ps.Next());
    }

    /// <summary>
    /// Builds the result of a Boolean operation of the given type from the splits of the given solids (see the
    /// overload with states).
    /// </summary>
    public void BuildBOP(NCollection_List<Shape> objects, NCollection_List<Shape> tools,
                         Operation operation, ProgressRange range, Report? report = null)
    {
        Containment objState, toolsState;
        switch (operation)
        {
            case Operation.Common:
            {
                objState = In;
                toolsState = In;
                break;
            }
            case Operation.Fuse:
            {
                objState = Out;
                toolsState = Out;
                break;
            }
            case Operation.Cut:
            {
                objState = Out;
                toolsState = In;
                break;
            }
            case Operation.CUT21:
            {
                objState = In;
                toolsState = Out;
                break;
            }
            default:
            {
                objState = global::Kerf.Topology.Containment.Unknown;
                toolsState = global::Kerf.Topology.Containment.Unknown;
                break;
            }
        }
        BuildBOP(objects, objState, tools, toolsState, range, report);
    }

    //! @name Images/Origins

    /// <summary>The map of images: the splits of the argument sub-shapes.</summary>
    public Dictionary<Shape, NCollection_List<Shape>> Images() => _images;

    /// <summary>The map of origins: the argument sub-shapes each split comes from.</summary>
    public Dictionary<Shape, NCollection_List<Shape>> Origins() => _origins;

    /// <summary>The map of same-domain shapes: each shape to its representative.</summary>
    public Dictionary<Shape, Shape> ShapesSD() => _shapesSD;

    //! @name Analyze progress of the operation

    protected enum BOPAlgo_PIOperation
    {
        PIOperation_TreatVertices = 0,
        PIOperation_TreatEdges,
        PIOperation_TreatWires,
        PIOperation_TreatFaces,
        PIOperation_TreatShells,
        PIOperation_TreatSolids,
        PIOperation_TreatCompsolids,
        PIOperation_TreatCompounds,
        PIOperation_FillHistory,
        PIOperation_PostTreat,
        PIOperation_Last,
    }

    /// <summary>The numbers of the shapes of each type the operation treats, for the progress steps.</summary>
    protected sealed class NbShapes
    {
        private readonly int[] _nbShapesArr = new int[8];

        public ref int NbVertices() => ref _nbShapesArr[0];

        public ref int NbEdges() => ref _nbShapesArr[1];

        public ref int NbWires() => ref _nbShapesArr[2];

        public ref int NbFaces() => ref _nbShapesArr[3];

        public ref int NbShells() => ref _nbShapesArr[4];

        public ref int NbSolids() => ref _nbShapesArr[5];

        public ref int NbCompsolids() => ref _nbShapesArr[6];

        public ref int NbCompounds() => ref _nbShapesArr[7];
    }

    /// <summary>Counts the shapes of each type the operation treats.</summary>
    protected NbShapes GetNbShapes()
    {
        NbShapes counter = new();
        counter.NbVertices() = _ds!.ShapesSD().Extent();
        for (int i = 0; i < _ds.SourceShapeCount; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            switch (si.ShapeType())
            {
                case ShapeType.Edge:
                {
                    if (_ds.HasPaveBlocks(i))
                    {
                        counter.NbEdges()++;
                    }
                    break;
                }
                case ShapeType.Wire:
                    counter.NbWires()++;
                    break;
                case ShapeType.Face:
                {
                    if (_ds.HasFaceInfo(i))
                    {
                        counter.NbFaces()++;
                    }
                    break;
                }
                case ShapeType.Shell:
                    counter.NbShells()++;
                    break;
                case ShapeType.Solid:
                    counter.NbSolids()++;
                    break;
                case ShapeType.CompSolid:
                    counter.NbCompsolids()++;
                    break;
                case ShapeType.Compound:
                    counter.NbCompounds()++;
                    break;
                default:
                    break;
            }
        }
        return counter;
    }

    /// <summary>The steps of the constant operations.</summary>
    protected override void FillPIConstants(double whole, PISteps steps)
    {
        // Fill in the constants:
        if (_fillHistory)
        {
            // for FillHistroty, which takes about 5% of the whole operation
            steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory, 0.05 * whole);
        }

        // and for PostTreat, which takes about 3% of the whole operation
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat, 0.03 * whole);
    }

    /// <summary>The steps of the operations depending on the numbers of shapes.</summary>
    protected override void FillPISteps(PISteps steps)
    {
        // Compute the rest of the operations - all depend on the number of sub-shapes of certain type
        NbShapes nbShapes = GetNbShapes();

        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices, nbShapes.NbVertices());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges, nbShapes.NbEdges());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatWires, nbShapes.NbWires());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatFaces, 20 * nbShapes.NbFaces());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatShells, nbShapes.NbShells());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatSolids, 50 * nbShapes.NbSolids());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompsolids, nbShapes.NbCompsolids());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompounds, nbShapes.NbCompounds());
    }

    //! @name Methods for building the result

    /// <summary>Builds the result, catching the failures as <see cref="AlertBuilderFailed"/>.</summary>
    protected virtual void PerformInternal(PaveFiller filler, ProgressRange range)
    {
        GetReport().Clear();
        //
        try
        {
            PerformInternal1(filler, range);
        }
        //
        catch (FailureException)
        {
            AddError(new AlertBuilderFailed());
        }
    }

    /// <summary>Builds the result: the images of the vertices, edges, wires, faces, shells, solids and compounds.</summary>
    protected virtual void PerformInternal1(PaveFiller filler, ProgressRange range)
    {
        _paveFiller = filler;
        _ds = _paveFiller.PDS();
        _context = _paveFiller.Context();
        _fuzzyValue = _paveFiller.FuzzyValue();
        _nonDestructive = _paveFiller.NonDestructive();
        //
        using var ps = new ProgressScope(range, "Building the result of General Fuse operation", 100);
        // 1. CheckData
        CheckData();
        if (HasErrors)
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors)
        {
            return;
        }
        //
        PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        AnalyzeProgress(100.0, steps);
        // 3. Fill Images
        // 3.1 Vertice
        FillImagesVertices(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Vertex);
        if (HasErrors)
        {
            return;
        }
        // 3.2 Edges
        FillImagesEdges(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Edge);
        if (HasErrors)
        {
            return;
        }
        //
        // 3.3 Wires
        FillImagesContainers(ShapeType.Wire, ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatWires)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Wire);
        if (HasErrors)
        {
            return;
        }

        // 3.4 Faces
        FillImagesFaces(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatFaces)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Face);
        if (HasErrors)
        {
            return;
        }
        // 3.5 Shells
        FillImagesContainers(ShapeType.Shell, ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatShells)));
        if (HasErrors)
        {
            return;
        }

        BuildResult(ShapeType.Shell);
        if (HasErrors)
        {
            return;
        }
        // 3.6 Solids
        FillImagesSolids(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatSolids)));
        if (HasErrors)
        {
            return;
        }

        BuildResult(ShapeType.Solid);
        if (HasErrors)
        {
            return;
        }
        // 3.7 CompSolids
        FillImagesContainers(ShapeType.CompSolid, ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompsolids)));
        if (HasErrors)
        {
            return;
        }

        BuildResult(ShapeType.CompSolid);
        if (HasErrors)
        {
            return;
        }

        // 3.8 Compounds
        FillImagesCompounds(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompounds)));
        if (HasErrors)
        {
            return;
        }

        BuildResult(ShapeType.Compound);
        if (HasErrors)
        {
            return;
        }
        //
        // 4 History
        PrepareHistory(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors)
        {
            return;
        }
        //
        // 5 Post-treatment
        PostTreat(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    //! @name Checking input arguments

    /// <summary>Checks the input data: at least two arguments and a pave filler.</summary>
    protected override void CheckData()
    {
        int nb = _arguments.Extent();
        if (nb < 2)
        {
            AddError(new AlertTooFewArguments()); // too few arguments to process
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>Checks that there is a pave filler and takes its warnings and errors.</summary>
    protected void CheckFiller()
    {
        if (_paveFiller is null)
        {
            AddError(new AlertNoFiller());
            return;
        }
        GetReport().Merge(_paveFiller.GetReport());
    }

    /// <summary>Makes the result an empty compound.</summary>
    protected virtual void Prepare()
    {
        BrepBuilder bb = new();
        Compound c = new();
        //
        // 1. myShape is empty compound
        bb.MakeCompound(ref c);
        _shape = c;
    }

    //! @name Post treatment

    /// <summary>Corrects the tolerances of the result (not of the argument sub-shapes in the non-destructive mode).</summary>
    protected virtual void PostTreat(ProgressRange range)
    {
        int i, nbS;
        ShapeType type;
        IndexedSet<Shape> ma = new(ShapeHasher.Instance);
        if (_paveFiller!.NonDestructive())
        {
            // MapToAvoid
            nbS = _ds!.SourceShapeCount;
            for (i = 0; i < nbS; ++i)
            {
                ShapeInfo si = _ds.ShapeInfo(i);
                type = si.ShapeType();
                if (type == ShapeType.Vertex || type == ShapeType.Edge || type == ShapeType.Face)
                {
                    Shape s = si.Shape();
                    ma.Add(s);
                }
            }
        }
        //
        using var ps = new ProgressScope(range, "Post treatment of result shape", 2);
        AlgoTools.CorrectTolerances(_shape, ma, 0.05, _runParallel);
        ps.Next();
        AlgoTools.CorrectShapeTolerances(_shape, ma, _runParallel);
    }
}
