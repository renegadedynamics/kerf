// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The Boolean operation (FUSE, COMMON, CUT, CUT21) of objects and tools.</summary>
/// <remarks>
/// <para>
/// The arguments are in two groups, the objects (<see cref="BooleanBuilder.AddArgument"/>) and the tools
/// (<see cref="ToolsProvider.AddTool"/>). The rules: for FUSE all arguments have the same dimension; for CUT
/// the objects' largest dimension is at most the tools' smallest; for CUT21 the reverse; COMMON takes any dimensions.
/// </para>
/// <para>
/// Errors besides the builder's: <see cref="AlertBopNotSet"/> (no operation),
/// <see cref="AlertBopNotAllowed"/> (dimensions), <see cref="AlertSolidBuilderFailed"/>; warning
/// <see cref="AlertEmptyShape"/>.
/// </para>
/// </remarks>
public class Bop : ToolsProvider
{
    protected Operation _operation;
    protected int[] _dims = [-1, -1];
    protected Shape _rc = new();

    public Bop()
    {
        Clear();
    }

    /// <summary>Clears the contents of the algorithm.</summary>
    public override void Clear()
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Unknown;
        _dims[0] = -1;
        _dims[1] = -1;

        base.Clear();
    }

    /// <summary>Sets the type of the Boolean operation.</summary>
    public void SetOperation(Operation operation) => _operation = operation;

    /// <summary>The type of the Boolean operation.</summary>
    public Operation Operation() => _operation;

    protected override void CheckData()
    {
        int i, j, nbArgs, nbTools;
        bool bFuse;
        NCollection_List<Shape>.Iterator itLS = new();
        //
        if (_operation != global::Kerf.Engine.Booleans.Operation.Common && _operation != global::Kerf.Engine.Booleans.Operation.Fuse && _operation != global::Kerf.Engine.Booleans.Operation.Cut
            && _operation != global::Kerf.Engine.Booleans.Operation.CUT21)
        {
            // non-licit operation
            AddError(new AlertBopNotSet());
            return;
        }
        //
        nbArgs = _arguments.Extent();
        if (nbArgs == 0)
        {
            // invalid number of Arguments
            AddError(new AlertTooFewArguments());
            return;
        }
        //
        nbTools = _tools.Extent();
        if (nbTools == 0)
        {
            // invalid number of Tools
            AddError(new AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
        if (HasErrors)
        {
            return;
        }
        //
        bFuse = _operation == global::Kerf.Engine.Booleans.Operation.Fuse;
        //
        // The rules for different types of operations are the following:
        // 1. FUSE:   All arguments and tools should have the same dimension;
        // 2. CUT:    The MAXIMAL dimension of the ARGUMENTS should be less
        //            or equal to the MINIMAL dimension of the TOOLS;
        // 3. CUT21:  The MINIMAL dimension of ARGUMENTS should be grater
        //            or equal to the MAXIMAL dimension of the TOOLS;
        // 4. COMMON: The arguments and tools could have any dimensions.
        //
        Span<int> iDimMin = [3, 3], iDimMax = [0, 0];
        Span<bool> bHasValid = [false, false];
        //
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> ls = i == 0 ? _arguments : _tools;
            itLS.Initialize(ls);
            for (j = 0; itLS.More(); itLS.Next(), ++j)
            {
                Shape s = itLS.Value();
                bool bIsEmpty = AlgoTools3D.IsEmptyShape(s);
                if (bIsEmpty)
                {
                    AddWarning(new AlertEmptyShape(s));
                    continue;
                }

                AlgoTools.Dimensions(s, out int iDMin, out int iDMax);

                if (iDMin < iDimMin[i])
                {
                    iDimMin[i] = iDMin;
                }
                if (iDMax > iDimMax[i])
                {
                    iDimMax[i] = iDMax;
                }

                if (bFuse && (iDimMin[i] != iDimMax[i]))
                {
                    // non-homogeneous argument
                    AddError(new AlertBopNotAllowed());
                    return;
                }
                bHasValid[i] = true;
            }
        }
        //
        if (bHasValid[0] && bHasValid[1])
        {
            if (((_operation == global::Kerf.Engine.Booleans.Operation.Fuse) && (iDimMax[0] != iDimMax[1]))
                || ((_operation == global::Kerf.Engine.Booleans.Operation.Cut) && (iDimMax[0] > iDimMin[1]))
                || ((_operation == global::Kerf.Engine.Booleans.Operation.CUT21) && (iDimMin[0] < iDimMax[1])))
            {
                // non-licit operation for the arguments
                AddError(new AlertBopNotAllowed());
                return;
            }
        }

        if (bHasValid[0] || bHasValid[1])
        {
            // In case of all empty shapes in one of the groups
            // this group acquires the dimension of other group
            _dims[0] = bHasValid[0] ? iDimMin[0] : iDimMin[1];
            _dims[1] = bHasValid[1] ? iDimMin[1] : iDimMin[0];
        }
    }

    /// <summary>Treats the case of empty arguments: true if the result could be built without splitting.</summary>
    protected bool TreatEmptyShape()
    {
        if (!GetReport().HasAlert(typeof(AlertEmptyShape)))
        {
            return false;
        }
        //
        // Find non-empty objects
        NCollection_List<Shape> lValidObjs = new();
        NCollection_List<Shape>.Iterator itLS = new(_arguments);
        for (; itLS.More(); itLS.Next())
        {
            if (!AlgoTools3D.IsEmptyShape(itLS.Value()))
            {
                lValidObjs.Append(itLS.Value());
            }
        }
        //
        // Find non-empty tools
        NCollection_List<Shape> lValidTools = new();
        itLS.Initialize(_tools);
        for (; itLS.More(); itLS.Next())
        {
            if (!AlgoTools3D.IsEmptyShape(itLS.Value()))
            {
                lValidTools.Append(itLS.Value());
            }
        }
        //
        bool bHasValidObj = lValidObjs.Extent() > 0;
        bool bHasValidTool = lValidTools.Extent() > 0;
        //
        if (bHasValidObj && bHasValidTool)
        {
            // We need to continue the operation to obtain the result
            return false;
        }
        //
        if (!bHasValidObj && !bHasValidTool)
        {
            // All shapes are empty shapes, the result will always be empty shape
            return true;
        }
        //
        // One of the groups of arguments consists of empty shapes only,
        // so we can build the result of operation right away just by
        // choosing the list of shapes to add to result, depending on
        // the type of the operation.
        // Although, if the group with valid shapes consists from more
        // than just one shape, depending on the operation type we may need
        // to split the shapes in this group before adding them into result.

        NCollection_List<Shape>? pLResult = null;
        //
        switch (_operation)
        {
            case global::Kerf.Engine.Booleans.Operation.Fuse:
            {
                if (lValidObjs.Extent() + lValidTools.Extent() > 1)
                {
                    // The arguments must be split before adding into result
                    return false;
                }

                // Add not empty shapes into result
                pLResult = bHasValidObj ? lValidObjs : lValidTools;
                break;
            }
            case global::Kerf.Engine.Booleans.Operation.Cut:
            {
                if (lValidObjs.Extent() > 1)
                {
                    // The objects must be split before adding into result
                    return false;
                }

                // Add objects into result
                pLResult = lValidObjs;
                break;
            }
            case global::Kerf.Engine.Booleans.Operation.CUT21:
            {
                if (lValidTools.Extent() > 1)
                {
                    // The tools must be split before adding into result
                    return false;
                }

                // Add tools into result
                pLResult = lValidTools;
                break;
            }
            case global::Kerf.Engine.Booleans.Operation.Common:
                // Common will always be empty
                break;
            default:
                break;
        }
        //
        if (pLResult is not null)
        {
            itLS.Initialize(pLResult);
            for (; itLS.More(); itLS.Next())
            {
                new BrepBuilder().Add(_shape, itLS.Value());
            }
        }
        return true;
    }

    protected override void BuildResult(ShapeType type)
    {
        ShapeType aType;
        BrepBuilder bb = new();
        HashSet<Shape> m = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(), itIm = new();
        //
        NCollection_List<Shape> la = _ds!.Arguments();
        it.Initialize(la);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            aType = s.ShapeType();
            if (aType == type)
            {
                if (_images.ContainsKey(s))
                {
                    NCollection_List<Shape> lsIm = _images.GetValue(s);
                    itIm.Initialize(lsIm);
                    for (; itIm.More(); itIm.Next())
                    {
                        Shape sIm = itIm.Value();
                        if (m.Add(sIm))
                        {
                            bb.Add(_shape, sIm);
                        }
                    }
                }
                else
                {
                    if (m.Add(s))
                    {
                        bb.Add(_shape, s);
                    }
                }
            }
        }
    }

    /// <summary>Intersects the objects and tools with a new pave filler and builds the result.</summary>
    public override void Perform(ProgressRange range = default)
    {
        PaveFiller pPF;
        NCollection_List<Shape>.Iterator itLS = new();
        //
        GetReport().Clear();
        //
        if (_entryPoint == 1)
        {
            _paveFiller = null;
        }
        //
        NCollection_List<Shape> ls = new();
        //
        itLS.Initialize(_arguments);
        for (; itLS.More(); itLS.Next())
        {
            Shape s = itLS.Value();
            ls.Append(s);
        }
        //
        itLS.Initialize(_tools);
        for (; itLS.More(); itLS.Next())
        {
            Shape s = itLS.Value();
            ls.Append(s);
        }
        //
        pPF = new PaveFiller();
        pPF.SetArguments(ls);
        pPF.SetRunParallel(_runParallel);
        using var ps = new ProgressScope(range, "Performing Boolean operation", 10);

        pPF.SetFuzzyValue(_fuzzyValue);
        pPF.SetNonDestructive(_nonDestructive);
        pPF.SetGlue(_glue);
        pPF.SetUseOBB(_useOBB);
        //
        pPF.Perform(ps.Next(9));
        //
        _entryPoint = 1;
        PerformInternal(pPF, ps.Next());
    }

    /// <summary>The steps of the operation (OCCT's nested enum, after BOPAlgo_ToolsProvider's).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_BuildShape = (int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_Last,
        PIOperation_Last,
    }

    protected override void FillPIConstants(double whole, PISteps steps)
    {
        base.FillPIConstants(whole, steps);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildShape,
                         (_operation == global::Kerf.Engine.Booleans.Operation.Fuse ? 10.0 : 5.0) * whole / 100.0);
    }

    protected override void PerformInternal1(PaveFiller filler, ProgressRange range)
    {
        _paveFiller = filler;
        _ds = _paveFiller.PDS();
        _context = _paveFiller.Context();
        _fuzzyValue = _paveFiller.FuzzyValue();
        _nonDestructive = _paveFiller.NonDestructive();
        //
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
        if (GetReport().HasAlert(typeof(AlertEmptyShape)))
        {
            bool bDone = TreatEmptyShape();
            if (bDone)
            {
                PrepareHistory(range);
                return;
            }
        }
        using var ps = new ProgressScope(range, "Building the result of Boolean operation", 100);
        //
        PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        AnalyzeProgress(100, steps);

        // 3. Fill Images
        // 3.1 Vertices
        FillImagesVertices(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatVertices)));
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
        FillImagesEdges(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatEdges)));
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
        FillImagesContainers(ShapeType.Wire, ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatWires)));
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
        //
        // 3.4 Faces
        FillImagesFaces(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatFaces)));
        if (HasErrors)
        {
            return;
        }

        BuildResult(ShapeType.Face);
        if (HasErrors)
        {
            return;
        }
        //
        // 3.5 Shells
        FillImagesContainers(ShapeType.Shell, ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatShells)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Shell);
        if (HasErrors)
        {
            return;
        }
        //
        // 3.6 Solids
        FillImagesSolids(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatSolids)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Solid);
        if (HasErrors)
        {
            return;
        }
        //
        // 3.7 CompSolids
        FillImagesContainers(ShapeType.CompSolid, ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatCompsolids)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.CompSolid);
        if (HasErrors)
        {
            return;
        }
        //
        // 3.8 Compounds
        FillImagesCompounds(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatCompounds)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Compound);
        if (HasErrors)
        {
            return;
        }
        //
        // 4.BuildShape;
        BuildShape(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildShape)));
        if (HasErrors)
        {
            return;
        }
        //
        // 5.History
        PrepareHistory(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors)
        {
            return;
        }
        //
        // 6 Post-treatment
        PostTreat(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>Builds the compound of the split building elements kept by the operation (myRC).</summary>
    protected void BuildRC(ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, 1);

        ShapeType type;
        Compound c = new();
        BrepBuilder bb = new();
        //
        bb.MakeCompound(ref c);
        //
        // A. Fuse
        if (_operation == global::Kerf.Engine.Booleans.Operation.Fuse)
        {
            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            type = TypeToExplore(_dims[0]);
            Explorer exp = new(_shape, type);
            for (; exp.More(); exp.Next())
            {
                Shape s = exp.Current();
                if (mFence.Add(s))
                {
                    bb.Add(c, s);
                }
            }
            _rc = c;
            return;
        }
        //
        if (UserBreak(ps))
        {
            return;
        }
        // B. Common, Cut, Cut21
        //
        int i, j, nb, iDim;
        bool bCheckEdges, bContains, bCut21, bCommon;
        NCollection_List<Shape>.Iterator itLS = new();
        //
        // prepare the building elements of arguments to get its splits
        NCollection_IndexedMap<Shape> mArgs = new(ShapeHasher.Instance), mTools = new(ShapeHasher.Instance);
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> ls = i == 0 ? _arguments : _tools;
            NCollection_IndexedMap<Shape> ms = i == 0 ? mArgs : mTools;
            itLS.Initialize(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                NCollection_List<Shape> list = new();
                AlgoTools.TreatCompound(s, list);
                for (NCollection_List<Shape>.Iterator itList = new(list); itList.More(); itList.Next())
                {
                    Shape ss = itList.Value();
                    iDim = AlgoTools.Dimension(ss);
                    if (iDim < 0)
                    {
                        continue;
                    }
                    type = TypeToExplore(iDim);
                    Exploration.MapShapes(ss, type, ms);
                }
            }
        }
        //
        if (UserBreak(ps))
        {
            return;
        }
        //
        bCheckEdges = false;
        //
        // get splits of building elements
        NCollection_IndexedMap<Shape> mArgsIm = new(ShapeHasher.Instance), mToolsIm = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<ToolSet, Shape> mSetArgs = new(), mSetTools = new();

        for (i = 0; i < 2; ++i)
        {
            NCollection_IndexedMap<Shape> ms = i == 0 ? mArgs : mTools;
            NCollection_IndexedMap<Shape> msIm = i == 0 ? mArgsIm : mToolsIm;
            NCollection_IndexedDataMap<ToolSet, Shape> mSet = i == 0 ? mSetArgs : mSetTools;
            //
            nb = ms.Extent();
            for (j = 1; j <= nb; ++j)
            {
                Shape s = ms[j];
                type = s.ShapeType();
                if (type == ShapeType.Edge)
                {
                    Edge e = ShapeCasts.Edge(s);
                    bCheckEdges = true;
                    if (Tool.Degenerated(e))
                    {
                        continue;
                    }
                }
                //
                if (_images.ContainsKey(s))
                {
                    NCollection_List<Shape> lsIm = _images.GetValue(s);
                    itLS.Initialize(lsIm);
                    for (; itLS.More(); itLS.Next())
                    {
                        Shape sIm = itLS.Value();
                        msIm.Add(sIm);
                    }
                }
                else
                {
                    msIm.Add(s);
                    if (s.ShapeType() == ShapeType.Solid)
                    {
                        ToolSet st = new();
                        st.Add(s, ShapeType.Face);
                        if (!mSet.Contains(st))
                        {
                            mSet.Add(st, s);
                        }
                    }
                }
            }
        }
        //
        // compare the maps and make the result
        //
        int iDimMin, iDimMax;
        //
        iDimMin = Min(_dims[0], _dims[1]);
        bCommon = _operation == global::Kerf.Engine.Booleans.Operation.Common;
        bCut21 = _operation == global::Kerf.Engine.Booleans.Operation.CUT21;
        //
        NCollection_IndexedMap<Shape> mIt = bCut21 ? mToolsIm : mArgsIm;
        NCollection_IndexedMap<Shape> mCheck = bCut21 ? mArgsIm : mToolsIm;
        NCollection_IndexedDataMap<ToolSet, Shape> mSetCheck = bCut21 ? mSetArgs : mSetTools;
        //
        NCollection_IndexedMap<Shape> mCheckExp = new(ShapeHasher.Instance), mItExp;
        //
        if (bCommon)
        {
            mItExp = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            nb = mIt.Extent();
            for (i = 1; i <= nb; ++i)
            {
                Shape s = mIt[i];
                iDimMax = AlgoTools.Dimension(s);
                for (iDim = iDimMin; iDim < iDimMax; ++iDim)
                {
                    type = TypeToExplore(iDim);
                    Exploration.MapShapes(s, type, mItExp);
                }
                mItExp.Add(s);
            }
        }
        else
        {
            // a copy, as OCCT's assignment
            mItExp = new NCollection_IndexedMap<Shape>(mIt);
        }
        //
        nb = mCheck.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape s = mCheck[i];
            iDimMax = AlgoTools.Dimension(s);
            for (iDim = iDimMin; iDim < iDimMax; ++iDim)
            {
                type = TypeToExplore(iDim);
                Exploration.MapShapes(s, type, mCheckExp);
            }
            mCheckExp.Add(s);
        }
        //
        nb = mItExp.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape s = mItExp[i];
            //
            bContains = mCheckExp.Contains(s);
            if (!bContains && s.ShapeType() == ShapeType.Solid)
            {
                ToolSet st = new();
                st.Add(s, ShapeType.Face);
                bContains = mSetCheck.Contains(st);
            }
            //
            if (bCommon)
            {
                if (bContains)
                {
                    bb.Add(c, s);
                }
            }
            else
            {
                if (!bContains)
                {
                    bb.Add(c, s);
                }
            }
        }
        //
        // filter result for COMMON operation
        if (bCommon)
        {
            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            Explorer exp = new();
            Compound cx = new();
            bb.MakeCompound(ref cx);
            //
            for (iDim = 3; iDim >= iDimMin; --iDim)
            {
                type = TypeToExplore(iDim);
                exp.Init(c, type);
                for (; exp.More(); exp.Next())
                {
                    Shape s = exp.Current();
                    if (mFence.Add(s))
                    {
                        bb.Add(cx, s);
                        Exploration.MapShapes(s, mFence);
                    }
                }
            }
            c = cx;
        }
        //
        if (!bCheckEdges)
        {
            _rc = c;
            return;
        }
        //
        if (UserBreak(ps))
        {
            return;
        }
        // The squats around degenerated edges
        int nVD;
        NCollection_IndexedMap<Shape> mvc = new(ShapeHasher.Instance);
        //
        // 1. Vertices of aC
        Exploration.MapShapes(c, ShapeType.Vertex, mvc);
        //
        // 2. DE candidates
        nb = _ds!.SourceShapeCount;
        for (i = 0; i < nb; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            type = si.ShapeType();
            if (type != ShapeType.Edge)
            {
                continue;
            }
            //
            Edge e = ShapeCasts.Edge(si.Shape());
            if (!Tool.Degenerated(e))
            {
                continue;
            }
            //
            nVD = si.SubShapes().First();
            Shape vd = _ds.Shape(nVD);
            //
            if (!mvc.Contains(vd))
            {
                continue;
            }
            //
            if (_ds.IsNewShape(nVD))
            {
                continue;
            }
            //
            if (_ds.HasInterf(nVD))
            {
                continue;
            }
            //
            bb.Add(c, e);
        }
        //
        _rc = c;
    }

    /// <summary>Builds the result shape from the kept building elements and the arguments' containers.</summary>
    protected void BuildShape(ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, 10.0);

        if (_dims[0] == 3 && _dims[1] == 3)
        {
            // For the Boolean operation on solids we need to check first
            // if we are dealing with closed solids, because for open solids
            // we cannot expect the BuilderSolid algorithm to produce good
            // splits for them and have to try the alternative approach for
            // building the result shape.
            // This approach is not used by default as it will loose the
            // modification history for solids, because the result solid
            // will be built from scratch using the splits of faces.
            bool hasNotClosedSolids = CheckArgsForOpenSolid();
            if (hasNotClosedSolids)
            {
                Report report = new();
                BuildBOP(_arguments, _tools, _operation, new ProgressRange(), report);
                if (report.GetAlerts(Fail).IsEmpty())
                {
                    // Success. Merge the report into the main report.
                    _report.Merge(report);
                    return;
                }
            }
        }

        // Build the result using splits of arguments.
        BuildRC(ps.Next(2.0));
        //
        if ((_operation == global::Kerf.Engine.Booleans.Operation.Fuse) && (_dims[0] == 3))
        {
            BuildSolid(ps.Next(8.0));
            return;
        }

        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        //
        int i;
        ShapeType type, t1, t2;
        NCollection_List<Shape> lsc = new(), lcb = new();
        NCollection_List<Shape>.Iterator itLS = new(), itLSIm = new(), itLCB = new();
        Iterator it = new();
        BrepBuilder bb = new();
        Shape rc = new(), rcb = new();
        //
        HashSet<Shape> msrc = new(ShapeHasher.Instance);
        Exploration.MapShapes(_rc, msrc);
        //
        // collect images of containers
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> ls = i == 0 ? _arguments : _tools;
            //
            itLS.Initialize(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                //
                CollectContainers(s, lsc);
            }
        }
        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        // make containers
        NCollection_List<Shape> lcRes = new();
        HashSet<Shape> mInpFence = new(ShapeHasher.Instance);
        itLS.Initialize(lsc);
        for (; itLS.More(); itLS.Next())
        {
            Shape sc = itLS.Value();
            mInpFence.Add(sc);
            //
            AlgoTools.MakeContainer(ShapeType.Compound, ref rc);
            //
            it.Initialize(sc);
            for (; it.More(); it.Next())
            {
                Shape s = it.Value();
                if (_images.ContainsKey(s))
                {
                    NCollection_List<Shape> lsIm = _images.GetValue(s);
                    //
                    itLSIm.Initialize(lsIm);
                    for (; itLSIm.More(); itLSIm.Next())
                    {
                        Shape sIm = itLSIm.Value();
                        if (msrc.Contains(sIm))
                        {
                            bb.Add(rc, sIm);
                        }
                    }
                }
                else if (msrc.Contains(s))
                {
                    bb.Add(rc, s);
                }
            }
            //
            type = sc.ShapeType();
            switch (type)
            {
                case ShapeType.Wire:
                {
                    t1 = ShapeType.Vertex;
                    t2 = ShapeType.Edge;
                    break;
                }
                case ShapeType.Shell:
                {
                    t1 = ShapeType.Edge;
                    t2 = ShapeType.Face;
                    break;
                }
                default:
                {
                    t1 = ShapeType.Face;
                    t2 = ShapeType.Solid;
                    break;
                }
            }
            //
            lcb.Clear();
            AlgoTools.MakeConnexityBlocks(rc, t1, t2, lcb);
            if (lcb.IsEmpty())
            {
                continue;
            }
            //
            itLCB.Initialize(lcb);
            for (; itLCB.More(); itLCB.Next())
            {
                AlgoTools.MakeContainer(type, ref rcb);
                //
                Shape cb = itLCB.Value();
                it.Initialize(cb);
                for (; it.More(); it.Next())
                {
                    Shape cbs = it.Value();
                    bb.Add(rcb, cbs);
                }
                //
                if (type == ShapeType.Wire)
                {
                    // reorient wire
                    AlgoTools.OrientEdgesOnWire(ref rcb);
                }
                else if (type == ShapeType.Shell)
                {
                    AlgoTools.OrientFacesOnShell(ref rcb);
                }
                //
                rcb.Orientation(sc.Orientation());
                //
                lcRes.Append(rcb);
            }
        }
        //
        RemoveDuplicates(lcRes);

        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        //
        // add containers to result
        Compound result = new();
        bb.MakeCompound(ref result);
        //
        itLS.Initialize(lcRes);
        for (; itLS.More(); itLS.Next())
        {
            bb.Add(result, itLS.Value());
        }

        // create map of containers
        HashSet<Shape> msResult = new(ShapeHasher.Instance);
        Exploration.MapShapes(result, msResult);

        // get input non-container shapes
        NCollection_List<Shape> lsNonCont = new();
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> ls = i == 0 ? _arguments : _tools;
            itLS.Initialize(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                AlgoTools.TreatCompound(s, lsNonCont, mInpFence);
            }
        }

        // put non-container shapes in the result
        itLS.Initialize(lsNonCont);
        for (; itLS.More(); itLS.Next())
        {
            Shape s = itLS.Value();
            if (_images.ContainsKey(s))
            {
                NCollection_List<Shape> lsIm = _images.GetValue(s);
                itLSIm.Initialize(lsIm);
                for (; itLSIm.More(); itLSIm.Next())
                {
                    Shape sIm = itLSIm.Value();
                    if (msrc.Contains(sIm) && msResult.Add(sIm))
                    {
                        bb.Add(result, sIm);
                    }
                }
            }
            else if (msrc.Contains(s) && msResult.Add(s))
            {
                bb.Add(result, s);
            }
        }

        _shape = result;
    }

    /// <summary>Builds the result of the FUSE of solids: new solids from the outer faces of the kept splits.</summary>
    protected void BuildSolid(ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, 10.0);
        // Containers
        NCollection_List<Shape> lsc = new();
        //
        NCollection_List<Shape>.Iterator itLS = new();
        Explorer exp = new();
        BrepBuilder bb = new();
        //
        // Get solids from input arguments
        HashSet<Shape> msa = new(ShapeHasher.Instance);
        // Map the arguments to find shared faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mfs = new(ShapeHasher.Instance);
        for (int i0 = 0; i0 < 2; ++i0)
        {
            NCollection_List<Shape> lsa = i0 != 0 ? _arguments : _tools;
            itLS.Initialize(lsa);
            for (; itLS.More(); itLS.Next())
            {
                Shape sa = itLS.Value();
                exp.Init(sa, ShapeType.Solid);
                for (; exp.More(); exp.Next())
                {
                    Shape sol = exp.Current();
                    msa.Add(sol);
                    Exploration.MapShapesAndAncestors(sol, ShapeType.Face, ShapeType.Solid, mfs);
                }
                //
                // get Compsolids from input arguments
                CollectContainers(sa, lsc);
            }
        }
        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        //
        // Find solids in input arguments sharing faces with other solids
        HashSet<Shape> mtSols = new(ShapeHasher.Instance);
        int i, nb = mfs.Extent();
        for (i = 1; i < nb; ++i) // OCCT: the last face of the map is not checked
        {
            NCollection_List<Shape> lSols = mfs[i];
            if (lSols.Extent() > 1)
            {
                itLS.Initialize(lSols);
                for (; itLS.More(); itLS.Next())
                {
                    mtSols.Add(itLS.Value());
                }
            }
        }
        //
        // Possibly untouched solids - to be added to results as is
        NCollection_IndexedMap<Shape> muSols = new(ShapeHasher.Instance);
        // Use map to chose the most outer faces to build result solids
        mfs.Clear();
        //
        Iterator it = new(_rc);
        for (; it.More(); it.Next())
        {
            Shape sx = it.Value();
            if (msa.Contains(sx))
            {
                if (!mtSols.Contains(sx))
                {
                    muSols.Add(sx);
                    continue;
                }
            }
            //
            MapFacesToBuildSolids(sx, mfs);
        }
        //
        // Process possibly untouched solids.
        // Really untouched solids will be added into result as is.
        // Others will be processed by BuilderSolid.
        NCollection_IndexedDataMap<ToolSet, Shape> dmsts = new();
        //
        nb = muSols.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape sx = muSols[i];
            //
            exp.Init(sx, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                if (mfs.Contains(exp.Current()))
                {
                    break;
                }
            }
            //
            if (exp.More())
            {
                MapFacesToBuildSolids(sx, mfs);
            }
            else
            {
                ToolSet st = new();
                st.Add(sx, ShapeType.Face);
                if (!dmsts.Contains(st))
                {
                    dmsts.Add(st, sx);
                }
            }
        }
        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
        // Fill the list of faces to build the result solids
        NCollection_List<Shape> sfs = new();
        nb = mfs.Extent();
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lSx = mfs[i];
            if (lSx.Extent() == 1)
            {
                Shape fx = mfs.FindKey(i);
                Exploration.MapShapesAndAncestors(fx, ShapeType.Edge, ShapeType.Face, mef);
                sfs.Append(fx);
            }
        }
        //
        Shape rc = new();
        AlgoTools.MakeContainer(ShapeType.Compound, ref rc);
        if (sfs.Extent() != 0)
        {
            // Build solids from set of faces
            BuilderSolid bs = new();
            bs.SetContext(_context);
            bs.SetShapes(sfs);
            bs.SetAvoidInternalShapes(true);
            bs.Perform(ps.Next(8.0));
            if (bs.HasErrors)
            {
                AddError(new AlertSolidBuilderFailed()); // SolidBuilder failed
                return;
            }

            _report.Merge(bs.GetReport());

            // new solids
            NCollection_List<Shape> lsr = bs.Areas();
            //
            // add new solids to result
            itLS.Initialize(lsr);
            for (; itLS.More(); itLS.Next())
            {
                Shape sr = itLS.Value();
                bb.Add(rc, sr);
            }
        }
        //
        // add untouched solids to result
        nb = dmsts.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape sx = dmsts[i];
            bb.Add(rc, sx);
        }
        //
        if (lsc.IsEmpty())
        {
            // no Compsolids in arguments
            _shape = rc;
            return;
        }
        //
        // build new Compsolids from new solids containing splits
        // of faces from arguments of type Compsolid
        //
        Shape result = new();
        AlgoTools.MakeContainer(ShapeType.Compound, ref result);
        //

        it.Initialize(rc);
        if (!it.More())
        {
            // no solids in the result
            _shape = rc;
            return;
        }
        //
        Shape sol1 = it.Value();
        it.Next();
        //
        // optimization for one solid in the result
        if (!it.More())
        {
            Shape aCS = new();
            AlgoTools.MakeContainer(ShapeType.CompSolid, ref aCS);
            bb.Add(aCS, sol1);
            //
            bb.Add(result, aCS);
            _shape = result;
            return;
        }
        //
        // get splits of faces of the Compsolid arguments
        HashSet<Shape> mfCs = new(ShapeHasher.Instance);
        itLS.Initialize(lsc);
        for (; itLS.More(); itLS.Next())
        {
            Shape aCs = itLS.Value();
            exp.Init(aCs, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Shape f = exp.Current();
                if (!_images.TryGetValue(f, out NCollection_List<Shape>? pLFIm))
                {
                    mfCs.Add(f);
                }
                else
                {
                    NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                    for (; itLFIm.More(); itLFIm.Next())
                    {
                        mfCs.Add(itLFIm.Value());
                    }
                }
            }
        }
        //
        // Check for user break
        if (UserBreak(ps))
        {
            return;
        }
        // build connexity blocks from new solids
        NCollection_List<Shape> lcbs = new();
        AlgoTools.MakeConnexityBlocks(rc, ShapeType.Face, ShapeType.Solid, lcbs);
        //
        itLS.Initialize(lcbs);
        for (; itLS.More(); itLS.Next())
        {
            Shape cb = itLS.Value();
            //
            // check if the Compsolid should be created
            exp.Init(cb, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                if (mfCs.Contains(exp.Current()))
                {
                    break;
                }
            }
            //
            if (!exp.More())
            {
                // add solids directly into result as their origins are not Compsolids
                for (it.Initialize(cb); it.More(); it.Next())
                {
                    bb.Add(result, it.Value());
                }
                continue;
            }
            //
            // make Compsolid
            Shape cs = new();
            AlgoTools.MakeContainer(ShapeType.CompSolid, ref cs);
            //
            it.Initialize(cb);
            for (; it.More(); it.Next())
            {
                bb.Add(cs, it.Value());
            }
            //
            bb.Add(result, cs);
        }
        //
        _shape = result;
    }

    /// <summary>
    /// True if an argument solid is open and the operation should be built from the split faces (BuildBOP), as the
    /// solid builder can't be expected to split open solids well.
    /// </summary>
    protected virtual bool CheckArgsForOpenSolid()
    {
        // Analyze the report to find if BuilderSolid has generated warnings
        // for any of the solids and collect these solids to check if they are open.
        HashSet<Shape> failedSolids = new(ShapeHasher.Instance);
        {
            NCollection_List<Alert> list = _report.GetAlerts(Warning);
            for (NCollection_List<Alert>.Iterator it = new(list); it.More(); it.Next())
            {
                Type type = it.Value().GetType();
                if (type != typeof(AlertSolidBuilderUnusedFaces))
                {
                    continue;
                }

                if (it.Value() is AlertWithShape shapeAlert)
                {
                    Shape warnShape = shapeAlert.InternalGetShape();
                    if (!warnShape.IsNull)
                    {
                        Explorer expS = new(warnShape, ShapeType.Solid);
                        for (; expS.More(); expS.Next())
                        {
                            failedSolids.Add(expS.Current());
                        }
                    }
                }
            }
        }

        // Iterate on all solids from the arguments and check if any
        // of them are not closed.
        // At the same time, collect all internal faces of the input solids
        // to check if the splits of open solids did not acquire any new
        // internal faces.

        int nbS = _ds!.SourceShapeCount;
        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Solid)
            {
                continue;
            }

            Shape solid = si.Shape();

            // Check that not INTERNAL faces create closed loops
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
            // Collect all splits of internal faces
            HashSet<Shape> mfInternal = new(ShapeHasher.Instance);

            for (Iterator itSh = new(solid); itSh.More(); itSh.Next())
            {
                Shape sh = itSh.Value();
                if (sh.ShapeType() != ShapeType.Shell)
                {
                    continue;
                }

                for (Iterator itF = new(sh); itF.More(); itF.Next())
                {
                    Shape f = itF.Value();
                    if (f.Orientation() == Internal)
                    {
                        if (_images.TryGetValue(f, out NCollection_List<Shape>? pLFIm))
                        {
                            NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                            for (; itLFIm.More(); itLFIm.Next())
                            {
                                mfInternal.Add(itLFIm.Value());
                            }
                        }
                        else
                        {
                            mfInternal.Add(f);
                        }
                    }
                    else
                    {
                        Exploration.MapShapesAndAncestors(f, ShapeType.Edge, ShapeType.Face, mef);
                    }
                }
            }

            // Analyze the Edge-Face connection map on free edges
            bool isClosed = true;
            int nbE = mef.Extent();
            for (int j = 1; j <= nbE && isClosed; ++j)
            {
                Edge e = ShapeCasts.Edge(mef.FindKey(j));
                if (Tool.Degenerated(e))
                {
                    // Skip degenerated edges
                    continue;
                }

                isClosed = mef[j].Extent() > 1;
                if (!isClosed)
                {
                    Face f = ShapeCasts.Face(mef[j].First());
                    isClosed = Tool.IsClosed(e, f); // Check for seam edges
                    if (!isClosed)
                    {
                        // Check if the edge is not internal in the face
                        Explorer expE = new(f, ShapeType.Edge);
                        for (; expE.More(); expE.Next())
                        {
                            if (expE.Current().IsSame(e))
                            {
                                isClosed = expE.Current().Orientation() == Internal;
                                break;
                            }
                        }
                    }
                }
            }

            if (isClosed)
            {
                continue;
            }

            // Not closed solid is found

            if (failedSolids.Contains(solid))
            {
                // Warning has been generated for this solid, return positive result right away.
                return true;
            }

            // Check the splits not to acquire new INTERNAL faces
            if (!_images.TryGetValue(solid, out NCollection_List<Shape>? pLSIm))
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itLSIm = new(pLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape sIm = itLSIm.Value();
                for (Iterator itSh = new(sIm); itSh.More(); itSh.Next())
                {
                    Shape sh = itSh.Value();
                    if (sh.ShapeType() != ShapeType.Shell)
                    {
                        continue;
                    }

                    for (Iterator itF = new(sh); itF.More(); itF.Next())
                    {
                        Shape f = itF.Value();
                        if (f.Orientation() == Internal)
                        {
                            if (!mfInternal.Contains(f))
                            {
                                // New internal face is found
                                return true;
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    private static ShapeType TypeToExplore(int dim)
    {
        ShapeType ret;
        //
        switch (dim)
        {
            case 0:
                ret = ShapeType.Vertex;
                break;
            case 1:
                ret = ShapeType.Edge;
                break;
            case 2:
                ret = ShapeType.Face;
                break;
            case 3:
                ret = ShapeType.Solid;
                break;
            default:
                ret = global::Kerf.Topology.ShapeType.Shape;
                break;
        }
        return ret;
    }

    private static void CollectContainers(in Shape s, NCollection_List<Shape> lsc)
    {
        ShapeType type = s.ShapeType();
        if (type == ShapeType.Wire || type == ShapeType.Shell || type == ShapeType.CompSolid)
        {
            lsc.Append(s);
            return;
        }
        //
        if (type != ShapeType.Compound)
        {
            return;
        }
        //
        Iterator it = new(s);
        for (; it.More(); it.Next())
        {
            Shape aS = it.Value();
            CollectContainers(aS, lsc);
        }
    }

    // Filters the containers with identical contents
    private static void RemoveDuplicates(NCollection_List<Shape> containers)
    {
        RemoveDuplicates(containers, ShapeType.Wire);
        RemoveDuplicates(containers, ShapeType.Shell);
        RemoveDuplicates(containers, ShapeType.CompSolid);
    }

    // Filters the containers of given type with identical contents
    private static void RemoveDuplicates(NCollection_List<Shape> containers, ShapeType type)
    {
        // get containers of given type
        NCollection_List<Shape> lc = new();
        NCollection_List<Shape>.Iterator itLC = new(containers);
        for (; itLC.More(); itLC.Next())
        {
            Shape c = itLC.Value();
            if (c.ShapeType() == type)
            {
                lc.Append(c);
            }
        }
        //
        if (lc.IsEmpty())
        {
            return;
        }
        //
        // map containers to compare its contents (OCCT's default hasher: orientation included, decision 721)
        NCollection_IndexedDataMap<Shape, HashSet<Shape>> contents = new(EqualityComparer<Shape>.Default);
        //
        itLC.Initialize(lc);
        for (; itLC.More(); itLC.Next())
        {
            Shape c = itLC.Value();
            //
            HashSet<Shape> mc = contents[contents.Add(c, new HashSet<Shape>(ShapeHasher.Instance))];
            //
            Iterator it = new(c);
            for (; it.More(); it.Next())
            {
                mc.Add(it.Value());
            }
        }
        //
        // compare the contents of the containers and find duplicates
        HashSet<Shape> duplicates = new(ShapeHasher.Instance);
        //
        int i, j, nb = contents.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape ci = contents.FindKey(i);
            if (duplicates.Contains(ci))
            {
                continue;
            }
            HashSet<Shape> mi = contents[i];
            int nbi = mi.Count;
            //
            for (j = i + 1; j <= nb; ++j)
            {
                Shape cj = contents.FindKey(j);
                if (duplicates.Contains(cj))
                {
                    continue;
                }
                HashSet<Shape> mj = contents[j];
                int nbj = mj.Count;
                //
                int nbCommon = NbCommonItemsInMap(mi, mj);
                //
                if (nbj == nbCommon)
                {
                    duplicates.Add(cj);
                    continue;
                }
                //
                if (nbi == nbCommon)
                {
                    duplicates.Add(ci);
                    break;
                }
            }
        }
        //
        if (duplicates.Count == 0)
        {
            return;
        }
        //
        // remove duplicating containers
        itLC.Initialize(containers);
        for (; itLC.More();)
        {
            Shape c = itLC.Value();
            if (duplicates.Contains(c))
            {
                containers.Remove(itLC);
                continue;
            }
            itLC.Next();
        }
    }

    // Counts the items contained in both maps
    private static int NbCommonItemsInMap(HashSet<Shape> m1, HashSet<Shape> m2)
    {
        HashSet<Shape> map1 = m1;
        HashSet<Shape> map2 = m2;
        //
        if (m2.Count < m1.Count)
        {
            map1 = m2;
            map2 = m1;
        }
        //
        int iCommon = 0;
        foreach (Shape s in map1)
        {
            if (map2.Contains(s))
            {
                ++iCommon;
            }
        }
        return iCommon;
    }

    // Stores the faces of the given solid into outgoing maps:
    // <theMFS> - not internal faces with reference to solid.
    private static void MapFacesToBuildSolids(in Shape sol,
                                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mfs)
    {
        Explorer exp = new(sol, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            //
            if (f.Orientation() == Internal)
            {
                continue;
            }
            //
            int index = mfs.FindIndex(f);
            if (index == 0)
            {
                NCollection_List<Shape> pLSol = mfs[mfs.Add(f, new NCollection_List<Shape>())];
                pLSol.Append(sol);
            }
            else
            {
                NCollection_List<Shape> pLSol = mfs[index];
                Shape f1 = mfs.FindKey(index);
                if (f1.Orientation() != f.Orientation())
                {
                    pLSol.Append(sol);
                }
            }
        }
    }
}
