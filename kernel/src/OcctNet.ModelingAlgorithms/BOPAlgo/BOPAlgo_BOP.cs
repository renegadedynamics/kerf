// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Occt.BOPAlgo_Operation;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The Boolean operation (FUSE, COMMON, CUT, CUT21) of objects and tools.</summary>
/// <remarks>
/// <para>
/// The arguments are in two groups, the objects (<see cref="BOPAlgo_Builder.AddArgument"/>) and the tools
/// (<see cref="BOPAlgo_ToolsProvider.AddTool"/>). The rules: for FUSE all arguments have the same dimension; for CUT
/// the objects' largest dimension is at most the tools' smallest; for CUT21 the reverse; COMMON takes any dimensions.
/// </para>
/// <para>
/// Errors besides the builder's: <see cref="BOPAlgo_AlertBOPNotSet"/> (no operation),
/// <see cref="BOPAlgo_AlertBOPNotAllowed"/> (dimensions), <see cref="BOPAlgo_AlertSolidBuilderFailed"/>; warning
/// <see cref="BOPAlgo_AlertEmptyShape"/>.
/// </para>
/// </remarks>
public class BOPAlgo_BOP : BOPAlgo_ToolsProvider
{
    protected BOPAlgo_Operation myOperation;
    protected int[] myDims = [-1, -1];
    protected Shape myRC = new();

    public BOPAlgo_BOP()
    {
        Clear();
    }

    /// <summary>Clears the contents of the algorithm.</summary>
    public override void Clear()
    {
        myOperation = BOPAlgo_UNKNOWN;
        myDims[0] = -1;
        myDims[1] = -1;

        base.Clear();
    }

    /// <summary>Sets the type of the Boolean operation.</summary>
    public void SetOperation(BOPAlgo_Operation theOperation) => myOperation = theOperation;

    /// <summary>The type of the Boolean operation.</summary>
    public BOPAlgo_Operation Operation() => myOperation;

    protected override void CheckData()
    {
        int i, j, aNbArgs, aNbTools;
        bool bFuse;
        NCollection_List<Shape>.Iterator aItLS = new();
        //
        if (myOperation != BOPAlgo_COMMON && myOperation != BOPAlgo_FUSE && myOperation != BOPAlgo_CUT
            && myOperation != BOPAlgo_CUT21)
        {
            // non-licit operation
            AddError(new BOPAlgo_AlertBOPNotSet());
            return;
        }
        //
        aNbArgs = myArguments.Extent();
        if (aNbArgs == 0)
        {
            // invalid number of Arguments
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        aNbTools = myTools.Extent();
        if (aNbTools == 0)
        {
            // invalid number of Tools
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
        if (HasErrors())
        {
            return;
        }
        //
        bFuse = myOperation == BOPAlgo_FUSE;
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
            NCollection_List<Shape> aLS = i == 0 ? myArguments : myTools;
            aItLS.Initialize(aLS);
            for (j = 0; aItLS.More(); aItLS.Next(), ++j)
            {
                Shape aS = aItLS.Value();
                bool bIsEmpty = BOPTools_AlgoTools3D.IsEmptyShape(aS);
                if (bIsEmpty)
                {
                    AddWarning(new BOPAlgo_AlertEmptyShape(aS));
                    continue;
                }

                BOPTools_AlgoTools.Dimensions(aS, out int iDMin, out int iDMax);

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
                    AddError(new BOPAlgo_AlertBOPNotAllowed());
                    return;
                }
                bHasValid[i] = true;
            }
        }
        //
        if (bHasValid[0] && bHasValid[1])
        {
            if (((myOperation == BOPAlgo_FUSE) && (iDimMax[0] != iDimMax[1]))
                || ((myOperation == BOPAlgo_CUT) && (iDimMax[0] > iDimMin[1]))
                || ((myOperation == BOPAlgo_CUT21) && (iDimMin[0] < iDimMax[1])))
            {
                // non-licit operation for the arguments
                AddError(new BOPAlgo_AlertBOPNotAllowed());
                return;
            }
        }

        if (bHasValid[0] || bHasValid[1])
        {
            // In case of all empty shapes in one of the groups
            // this group acquires the dimension of other group
            myDims[0] = bHasValid[0] ? iDimMin[0] : iDimMin[1];
            myDims[1] = bHasValid[1] ? iDimMin[1] : iDimMin[0];
        }
    }

    /// <summary>Treats the case of empty arguments: true if the result could be built without splitting.</summary>
    protected bool TreatEmptyShape()
    {
        if (!GetReport().HasAlert(typeof(BOPAlgo_AlertEmptyShape)))
        {
            return false;
        }
        //
        // Find non-empty objects
        NCollection_List<Shape> aLValidObjs = new();
        NCollection_List<Shape>.Iterator aItLS = new(myArguments);
        for (; aItLS.More(); aItLS.Next())
        {
            if (!BOPTools_AlgoTools3D.IsEmptyShape(aItLS.Value()))
            {
                aLValidObjs.Append(aItLS.Value());
            }
        }
        //
        // Find non-empty tools
        NCollection_List<Shape> aLValidTools = new();
        aItLS.Initialize(myTools);
        for (; aItLS.More(); aItLS.Next())
        {
            if (!BOPTools_AlgoTools3D.IsEmptyShape(aItLS.Value()))
            {
                aLValidTools.Append(aItLS.Value());
            }
        }
        //
        bool bHasValidObj = aLValidObjs.Extent() > 0;
        bool bHasValidTool = aLValidTools.Extent() > 0;
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
        switch (myOperation)
        {
            case BOPAlgo_FUSE:
            {
                if (aLValidObjs.Extent() + aLValidTools.Extent() > 1)
                {
                    // The arguments must be split before adding into result
                    return false;
                }

                // Add not empty shapes into result
                pLResult = bHasValidObj ? aLValidObjs : aLValidTools;
                break;
            }
            case BOPAlgo_CUT:
            {
                if (aLValidObjs.Extent() > 1)
                {
                    // The objects must be split before adding into result
                    return false;
                }

                // Add objects into result
                pLResult = aLValidObjs;
                break;
            }
            case BOPAlgo_CUT21:
            {
                if (aLValidTools.Extent() > 1)
                {
                    // The tools must be split before adding into result
                    return false;
                }

                // Add tools into result
                pLResult = aLValidTools;
                break;
            }
            case BOPAlgo_COMMON:
                // Common will always be empty
                break;
            default:
                break;
        }
        //
        if (pLResult is not null)
        {
            aItLS.Initialize(pLResult);
            for (; aItLS.More(); aItLS.Next())
            {
                new BrepBuilder().Add(myShape, aItLS.Value());
            }
        }
        return true;
    }

    protected override void BuildResult(ShapeType theType)
    {
        ShapeType aType;
        BrepBuilder aBB = new();
        NCollection_Map<Shape> aM = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aIt = new(), aItIm = new();
        //
        NCollection_List<Shape> aLA = myDS!.Arguments();
        aIt.Initialize(aLA);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            aType = aS.ShapeType();
            if (aType == theType)
            {
                if (myImages.IsBound(aS))
                {
                    NCollection_List<Shape> aLSIm = myImages.Find(aS);
                    aItIm.Initialize(aLSIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        Shape aSIm = aItIm.Value();
                        if (aM.Add(aSIm))
                        {
                            aBB.Add(myShape, aSIm);
                        }
                    }
                }
                else
                {
                    if (aM.Add(aS))
                    {
                        aBB.Add(myShape, aS);
                    }
                }
            }
        }
    }

    /// <summary>Intersects the objects and tools with a new pave filler and builds the result.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        BOPAlgo_PaveFiller pPF;
        NCollection_List<Shape>.Iterator aItLS = new();
        //
        GetReport().Clear();
        //
        if (myEntryPoint == 1)
        {
            myPaveFiller = null;
        }
        //
        NCollection_List<Shape> aLS = new();
        //
        aItLS.Initialize(myArguments);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aS = aItLS.Value();
            aLS.Append(aS);
        }
        //
        aItLS.Initialize(myTools);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aS = aItLS.Value();
            aLS.Append(aS);
        }
        //
        pPF = new BOPAlgo_PaveFiller();
        pPF.SetArguments(aLS);
        pPF.SetRunParallel(myRunParallel);
        using var aPS = new ProgressScope(theRange, "Performing Boolean operation", 10);

        pPF.SetFuzzyValue(myFuzzyValue);
        pPF.SetNonDestructive(myNonDestructive);
        pPF.SetGlue(myGlue);
        pPF.SetUseOBB(myUseOBB);
        //
        pPF.Perform(aPS.Next(9));
        //
        myEntryPoint = 1;
        PerformInternal(pPF, aPS.Next());
    }

    /// <summary>The steps of the operation (OCCT's nested enum, after BOPAlgo_ToolsProvider's).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_BuildShape = (int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_Last,
        PIOperation_Last,
    }

    protected override void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
        base.fillPIConstants(theWhole, theSteps);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildShape,
                         (myOperation == BOPAlgo_FUSE ? 10.0 : 5.0) * theWhole / 100.0);
    }

    protected override void PerformInternal1(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        myPaveFiller = theFiller;
        myDS = myPaveFiller.PDS();
        myContext = myPaveFiller.Context();
        myFuzzyValue = myPaveFiller.FuzzyValue();
        myNonDestructive = myPaveFiller.NonDestructive();
        //
        // 1. CheckData
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors())
        {
            return;
        }
        //
        if (GetReport().HasAlert(typeof(BOPAlgo_AlertEmptyShape)))
        {
            bool bDone = TreatEmptyShape();
            if (bDone)
            {
                PrepareHistory(theRange);
                return;
            }
        }
        using var aPS = new ProgressScope(theRange, "Building the result of Boolean operation", 100);
        //
        BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        analyzeProgress(100, aSteps);

        // 3. Fill Images
        // 3.1 Vertices
        FillImagesVertices(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatVertices)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Vertex);
        if (HasErrors())
        {
            return;
        }
        // 3.2 Edges
        FillImagesEdges(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatEdges)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Edge);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.3 Wires
        FillImagesContainers(ShapeType.Wire, aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatWires)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Wire);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.4 Faces
        FillImagesFaces(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatFaces)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(ShapeType.Face);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.5 Shells
        FillImagesContainers(ShapeType.Shell, aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatShells)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Shell);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.6 Solids
        FillImagesSolids(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatSolids)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Solid);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.7 CompSolids
        FillImagesContainers(ShapeType.CompSolid, aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatCompsolids)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.CompSolid);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.8 Compounds
        FillImagesCompounds(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatCompounds)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Compound);
        if (HasErrors())
        {
            return;
        }
        //
        // 4.BuildShape;
        BuildShape(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildShape)));
        if (HasErrors())
        {
            return;
        }
        //
        // 5.History
        PrepareHistory(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors())
        {
            return;
        }
        //
        // 6 Post-treatment
        PostTreat(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>Builds the compound of the split building elements kept by the operation (myRC).</summary>
    protected void BuildRC(ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, null, 1);

        ShapeType aType;
        Compound aC = new();
        BrepBuilder aBB = new();
        //
        aBB.MakeCompound(ref aC);
        //
        // A. Fuse
        if (myOperation == BOPAlgo_FUSE)
        {
            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            aType = TypeToExplore(myDims[0]);
            Explorer aExp = new(myShape, aType);
            for (; aExp.More(); aExp.Next())
            {
                Shape aS = aExp.Current();
                if (aMFence.Add(aS))
                {
                    aBB.Add(aC, aS);
                }
            }
            myRC = aC;
            return;
        }
        //
        if (UserBreak(aPS))
        {
            return;
        }
        // B. Common, Cut, Cut21
        //
        int i, j, aNb, iDim;
        bool bCheckEdges, bContains, bCut21, bCommon;
        NCollection_List<Shape>.Iterator aItLS = new();
        //
        // prepare the building elements of arguments to get its splits
        NCollection_IndexedMap<Shape> aMArgs = new(ShapeHasher.Instance), aMTools = new(ShapeHasher.Instance);
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aLS = i == 0 ? myArguments : myTools;
            NCollection_IndexedMap<Shape> aMS = i == 0 ? aMArgs : aMTools;
            aItLS.Initialize(aLS);
            for (; aItLS.More(); aItLS.Next())
            {
                Shape aS = aItLS.Value();
                NCollection_List<Shape> aList = new();
                BOPTools_AlgoTools.TreatCompound(aS, aList);
                for (NCollection_List<Shape>.Iterator itList = new(aList); itList.More(); itList.Next())
                {
                    Shape aSS = itList.Value();
                    iDim = BOPTools_AlgoTools.Dimension(aSS);
                    if (iDim < 0)
                    {
                        continue;
                    }
                    aType = TypeToExplore(iDim);
                    Exploration.MapShapes(aSS, aType, aMS);
                }
            }
        }
        //
        if (UserBreak(aPS))
        {
            return;
        }
        //
        bCheckEdges = false;
        //
        // get splits of building elements
        NCollection_IndexedMap<Shape> aMArgsIm = new(ShapeHasher.Instance), aMToolsIm = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<BOPTools_Set, Shape> aMSetArgs = new(), aMSetTools = new();

        for (i = 0; i < 2; ++i)
        {
            NCollection_IndexedMap<Shape> aMS = i == 0 ? aMArgs : aMTools;
            NCollection_IndexedMap<Shape> aMSIm = i == 0 ? aMArgsIm : aMToolsIm;
            NCollection_IndexedDataMap<BOPTools_Set, Shape> aMSet = i == 0 ? aMSetArgs : aMSetTools;
            //
            aNb = aMS.Extent();
            for (j = 1; j <= aNb; ++j)
            {
                Shape aS = aMS[j];
                aType = aS.ShapeType();
                if (aType == ShapeType.Edge)
                {
                    Edge aE = ShapeCasts.Edge(aS);
                    bCheckEdges = true;
                    if (Tool.Degenerated(aE))
                    {
                        continue;
                    }
                }
                //
                if (myImages.IsBound(aS))
                {
                    NCollection_List<Shape> aLSIm = myImages.Find(aS);
                    aItLS.Initialize(aLSIm);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        Shape aSIm = aItLS.Value();
                        aMSIm.Add(aSIm);
                    }
                }
                else
                {
                    aMSIm.Add(aS);
                    if (aS.ShapeType() == ShapeType.Solid)
                    {
                        BOPTools_Set aST = new();
                        aST.Add(aS, ShapeType.Face);
                        if (!aMSet.Contains(aST))
                        {
                            aMSet.Add(aST, aS);
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
        iDimMin = Min(myDims[0], myDims[1]);
        bCommon = myOperation == BOPAlgo_COMMON;
        bCut21 = myOperation == BOPAlgo_CUT21;
        //
        NCollection_IndexedMap<Shape> aMIt = bCut21 ? aMToolsIm : aMArgsIm;
        NCollection_IndexedMap<Shape> aMCheck = bCut21 ? aMArgsIm : aMToolsIm;
        NCollection_IndexedDataMap<BOPTools_Set, Shape> aMSetCheck = bCut21 ? aMSetArgs : aMSetTools;
        //
        NCollection_IndexedMap<Shape> aMCheckExp = new(ShapeHasher.Instance), aMItExp;
        //
        if (bCommon)
        {
            aMItExp = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            aNb = aMIt.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                Shape aS = aMIt[i];
                iDimMax = BOPTools_AlgoTools.Dimension(aS);
                for (iDim = iDimMin; iDim < iDimMax; ++iDim)
                {
                    aType = TypeToExplore(iDim);
                    Exploration.MapShapes(aS, aType, aMItExp);
                }
                aMItExp.Add(aS);
            }
        }
        else
        {
            // a copy, as OCCT's assignment
            aMItExp = new NCollection_IndexedMap<Shape>(aMIt);
        }
        //
        aNb = aMCheck.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aS = aMCheck[i];
            iDimMax = BOPTools_AlgoTools.Dimension(aS);
            for (iDim = iDimMin; iDim < iDimMax; ++iDim)
            {
                aType = TypeToExplore(iDim);
                Exploration.MapShapes(aS, aType, aMCheckExp);
            }
            aMCheckExp.Add(aS);
        }
        //
        aNb = aMItExp.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aS = aMItExp[i];
            //
            bContains = aMCheckExp.Contains(aS);
            if (!bContains && aS.ShapeType() == ShapeType.Solid)
            {
                BOPTools_Set aST = new();
                aST.Add(aS, ShapeType.Face);
                bContains = aMSetCheck.Contains(aST);
            }
            //
            if (bCommon)
            {
                if (bContains)
                {
                    aBB.Add(aC, aS);
                }
            }
            else
            {
                if (!bContains)
                {
                    aBB.Add(aC, aS);
                }
            }
        }
        //
        // filter result for COMMON operation
        if (bCommon)
        {
            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            Explorer aExp = new();
            Compound aCx = new();
            aBB.MakeCompound(ref aCx);
            //
            for (iDim = 3; iDim >= iDimMin; --iDim)
            {
                aType = TypeToExplore(iDim);
                aExp.Init(aC, aType);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aS = aExp.Current();
                    if (aMFence.Add(aS))
                    {
                        aBB.Add(aCx, aS);
                        Exploration.MapShapes(aS, aMFence);
                    }
                }
            }
            aC = aCx;
        }
        //
        if (!bCheckEdges)
        {
            myRC = aC;
            return;
        }
        //
        if (UserBreak(aPS))
        {
            return;
        }
        // The squats around degenerated edges
        int nVD;
        NCollection_IndexedMap<Shape> aMVC = new(ShapeHasher.Instance);
        //
        // 1. Vertices of aC
        Exploration.MapShapes(aC, ShapeType.Vertex, aMVC);
        //
        // 2. DE candidates
        aNb = myDS!.NbSourceShapes();
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            aType = aSI.ShapeType();
            if (aType != ShapeType.Edge)
            {
                continue;
            }
            //
            Edge aE = ShapeCasts.Edge(aSI.Shape());
            if (!Tool.Degenerated(aE))
            {
                continue;
            }
            //
            nVD = aSI.SubShapes().First();
            Shape aVD = myDS.Shape(nVD);
            //
            if (!aMVC.Contains(aVD))
            {
                continue;
            }
            //
            if (myDS.IsNewShape(nVD))
            {
                continue;
            }
            //
            if (myDS.HasInterf(nVD))
            {
                continue;
            }
            //
            aBB.Add(aC, aE);
        }
        //
        myRC = aC;
    }

    /// <summary>Builds the result shape from the kept building elements and the arguments' containers.</summary>
    protected void BuildShape(ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, null, 10.0);

        if (myDims[0] == 3 && myDims[1] == 3)
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
                Report aReport = new();
                BuildBOP(myArguments, myTools, myOperation, new ProgressRange(), aReport);
                if (aReport.GetAlerts(Fail).IsEmpty())
                {
                    // Success. Merge the report into the main report.
                    myReport.Merge(aReport);
                    return;
                }
            }
        }

        // Build the result using splits of arguments.
        BuildRC(aPS.Next(2.0));
        //
        if ((myOperation == BOPAlgo_FUSE) && (myDims[0] == 3))
        {
            BuildSolid(aPS.Next(8.0));
            return;
        }

        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        //
        int i;
        ShapeType aType, aT1, aT2;
        NCollection_List<Shape> aLSC = new(), aLCB = new();
        NCollection_List<Shape>.Iterator aItLS = new(), aItLSIm = new(), aItLCB = new();
        Iterator aIt = new();
        BrepBuilder aBB = new();
        Shape aRC = new(), aRCB = new();
        //
        NCollection_Map<Shape> aMSRC = new(ShapeHasher.Instance);
        Exploration.MapShapes(myRC, aMSRC);
        //
        // collect images of containers
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aLS = i == 0 ? myArguments : myTools;
            //
            aItLS.Initialize(aLS);
            for (; aItLS.More(); aItLS.Next())
            {
                Shape aS = aItLS.Value();
                //
                CollectContainers(aS, aLSC);
            }
        }
        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        // make containers
        NCollection_List<Shape> aLCRes = new();
        NCollection_Map<Shape> aMInpFence = new(ShapeHasher.Instance);
        aItLS.Initialize(aLSC);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aSC = aItLS.Value();
            aMInpFence.Add(aSC);
            //
            BOPTools_AlgoTools.MakeContainer(ShapeType.Compound, ref aRC);
            //
            aIt.Initialize(aSC);
            for (; aIt.More(); aIt.Next())
            {
                Shape aS = aIt.Value();
                if (myImages.IsBound(aS))
                {
                    NCollection_List<Shape> aLSIm = myImages.Find(aS);
                    //
                    aItLSIm.Initialize(aLSIm);
                    for (; aItLSIm.More(); aItLSIm.Next())
                    {
                        Shape aSIm = aItLSIm.Value();
                        if (aMSRC.Contains(aSIm))
                        {
                            aBB.Add(aRC, aSIm);
                        }
                    }
                }
                else if (aMSRC.Contains(aS))
                {
                    aBB.Add(aRC, aS);
                }
            }
            //
            aType = aSC.ShapeType();
            switch (aType)
            {
                case ShapeType.Wire:
                {
                    aT1 = ShapeType.Vertex;
                    aT2 = ShapeType.Edge;
                    break;
                }
                case ShapeType.Shell:
                {
                    aT1 = ShapeType.Edge;
                    aT2 = ShapeType.Face;
                    break;
                }
                default:
                {
                    aT1 = ShapeType.Face;
                    aT2 = ShapeType.Solid;
                    break;
                }
            }
            //
            aLCB.Clear();
            BOPTools_AlgoTools.MakeConnexityBlocks(aRC, aT1, aT2, aLCB);
            if (aLCB.IsEmpty())
            {
                continue;
            }
            //
            aItLCB.Initialize(aLCB);
            for (; aItLCB.More(); aItLCB.Next())
            {
                BOPTools_AlgoTools.MakeContainer(aType, ref aRCB);
                //
                Shape aCB = aItLCB.Value();
                aIt.Initialize(aCB);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aCBS = aIt.Value();
                    aBB.Add(aRCB, aCBS);
                }
                //
                if (aType == ShapeType.Wire)
                {
                    // reorient wire
                    BOPTools_AlgoTools.OrientEdgesOnWire(ref aRCB);
                }
                else if (aType == ShapeType.Shell)
                {
                    BOPTools_AlgoTools.OrientFacesOnShell(ref aRCB);
                }
                //
                aRCB.Orientation(aSC.Orientation());
                //
                aLCRes.Append(aRCB);
            }
        }
        //
        RemoveDuplicates(aLCRes);

        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        //
        // add containers to result
        Compound aResult = new();
        aBB.MakeCompound(ref aResult);
        //
        aItLS.Initialize(aLCRes);
        for (; aItLS.More(); aItLS.Next())
        {
            aBB.Add(aResult, aItLS.Value());
        }

        // create map of containers
        NCollection_Map<Shape> aMSResult = new(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, aMSResult);

        // get input non-container shapes
        NCollection_List<Shape> aLSNonCont = new();
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aLS = i == 0 ? myArguments : myTools;
            aItLS.Initialize(aLS);
            for (; aItLS.More(); aItLS.Next())
            {
                Shape aS = aItLS.Value();
                BOPTools_AlgoTools.TreatCompound(aS, aLSNonCont, aMInpFence);
            }
        }

        // put non-container shapes in the result
        aItLS.Initialize(aLSNonCont);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aS = aItLS.Value();
            if (myImages.IsBound(aS))
            {
                NCollection_List<Shape> aLSIm = myImages.Find(aS);
                aItLSIm.Initialize(aLSIm);
                for (; aItLSIm.More(); aItLSIm.Next())
                {
                    Shape aSIm = aItLSIm.Value();
                    if (aMSRC.Contains(aSIm) && aMSResult.Add(aSIm))
                    {
                        aBB.Add(aResult, aSIm);
                    }
                }
            }
            else if (aMSRC.Contains(aS) && aMSResult.Add(aS))
            {
                aBB.Add(aResult, aS);
            }
        }

        myShape = aResult;
    }

    /// <summary>Builds the result of the FUSE of solids: new solids from the outer faces of the kept splits.</summary>
    protected void BuildSolid(ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, null, 10.0);
        // Containers
        NCollection_List<Shape> aLSC = new();
        //
        NCollection_List<Shape>.Iterator aItLS = new();
        Explorer aExp = new();
        BrepBuilder aBB = new();
        //
        // Get solids from input arguments
        NCollection_Map<Shape> aMSA = new(ShapeHasher.Instance);
        // Map the arguments to find shared faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMFS = new(ShapeHasher.Instance);
        for (int i0 = 0; i0 < 2; ++i0)
        {
            NCollection_List<Shape> aLSA = i0 != 0 ? myArguments : myTools;
            aItLS.Initialize(aLSA);
            for (; aItLS.More(); aItLS.Next())
            {
                Shape aSA = aItLS.Value();
                aExp.Init(aSA, ShapeType.Solid);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aSol = aExp.Current();
                    aMSA.Add(aSol);
                    Exploration.MapShapesAndAncestors(aSol, ShapeType.Face, ShapeType.Solid, aMFS);
                }
                //
                // get Compsolids from input arguments
                CollectContainers(aSA, aLSC);
            }
        }
        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        //
        // Find solids in input arguments sharing faces with other solids
        NCollection_Map<Shape> aMTSols = new(ShapeHasher.Instance);
        int i, aNb = aMFS.Extent();
        for (i = 1; i < aNb; ++i) // OCCT: the last face of the map is not checked
        {
            NCollection_List<Shape> aLSols = aMFS[i];
            if (aLSols.Extent() > 1)
            {
                aItLS.Initialize(aLSols);
                for (; aItLS.More(); aItLS.Next())
                {
                    aMTSols.Add(aItLS.Value());
                }
            }
        }
        //
        // Possibly untouched solids - to be added to results as is
        NCollection_IndexedMap<Shape> aMUSols = new(ShapeHasher.Instance);
        // Use map to chose the most outer faces to build result solids
        aMFS.Clear();
        //
        Iterator aIt = new(myRC);
        for (; aIt.More(); aIt.Next())
        {
            Shape aSx = aIt.Value();
            if (aMSA.Contains(aSx))
            {
                if (!aMTSols.Contains(aSx))
                {
                    aMUSols.Add(aSx);
                    continue;
                }
            }
            //
            MapFacesToBuildSolids(aSx, aMFS);
        }
        //
        // Process possibly untouched solids.
        // Really untouched solids will be added into result as is.
        // Others will be processed by BuilderSolid.
        NCollection_IndexedDataMap<BOPTools_Set, Shape> aDMSTS = new();
        //
        aNb = aMUSols.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aSx = aMUSols[i];
            //
            aExp.Init(aSx, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                if (aMFS.Contains(aExp.Current()))
                {
                    break;
                }
            }
            //
            if (aExp.More())
            {
                MapFacesToBuildSolids(aSx, aMFS);
            }
            else
            {
                BOPTools_Set aST = new();
                aST.Add(aSx, ShapeType.Face);
                if (!aDMSTS.Contains(aST))
                {
                    aDMSTS.Add(aST, aSx);
                }
            }
        }
        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        // Fill the list of faces to build the result solids
        NCollection_List<Shape> aSFS = new();
        aNb = aMFS.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLSx = aMFS[i];
            if (aLSx.Extent() == 1)
            {
                Shape aFx = aMFS.FindKey(i);
                Exploration.MapShapesAndAncestors(aFx, ShapeType.Edge, ShapeType.Face, aMEF);
                aSFS.Append(aFx);
            }
        }
        //
        Shape aRC = new();
        BOPTools_AlgoTools.MakeContainer(ShapeType.Compound, ref aRC);
        if (aSFS.Extent() != 0)
        {
            // Build solids from set of faces
            BOPAlgo_BuilderSolid aBS = new();
            aBS.SetContext(myContext);
            aBS.SetShapes(aSFS);
            aBS.SetAvoidInternalShapes(true);
            aBS.Perform(aPS.Next(8.0));
            if (aBS.HasErrors())
            {
                AddError(new BOPAlgo_AlertSolidBuilderFailed()); // SolidBuilder failed
                return;
            }

            myReport.Merge(aBS.GetReport());

            // new solids
            NCollection_List<Shape> aLSR = aBS.Areas();
            //
            // add new solids to result
            aItLS.Initialize(aLSR);
            for (; aItLS.More(); aItLS.Next())
            {
                Shape aSR = aItLS.Value();
                aBB.Add(aRC, aSR);
            }
        }
        //
        // add untouched solids to result
        aNb = aDMSTS.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aSx = aDMSTS[i];
            aBB.Add(aRC, aSx);
        }
        //
        if (aLSC.IsEmpty())
        {
            // no Compsolids in arguments
            myShape = aRC;
            return;
        }
        //
        // build new Compsolids from new solids containing splits
        // of faces from arguments of type Compsolid
        //
        Shape aResult = new();
        BOPTools_AlgoTools.MakeContainer(ShapeType.Compound, ref aResult);
        //

        aIt.Initialize(aRC);
        if (!aIt.More())
        {
            // no solids in the result
            myShape = aRC;
            return;
        }
        //
        Shape aSol1 = aIt.Value();
        aIt.Next();
        //
        // optimization for one solid in the result
        if (!aIt.More())
        {
            Shape aCS = new();
            BOPTools_AlgoTools.MakeContainer(ShapeType.CompSolid, ref aCS);
            aBB.Add(aCS, aSol1);
            //
            aBB.Add(aResult, aCS);
            myShape = aResult;
            return;
        }
        //
        // get splits of faces of the Compsolid arguments
        NCollection_Map<Shape> aMFCs = new(ShapeHasher.Instance);
        aItLS.Initialize(aLSC);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aCs = aItLS.Value();
            aExp.Init(aCs, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                Shape aF = aExp.Current();
                if (!myImages.Find(aF, out NCollection_List<Shape> pLFIm))
                {
                    aMFCs.Add(aF);
                }
                else
                {
                    NCollection_List<Shape>.Iterator aItLFIm = new(pLFIm);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        aMFCs.Add(aItLFIm.Value());
                    }
                }
            }
        }
        //
        // Check for user break
        if (UserBreak(aPS))
        {
            return;
        }
        // build connexity blocks from new solids
        NCollection_List<Shape> aLCBS = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aRC, ShapeType.Face, ShapeType.Solid, aLCBS);
        //
        aItLS.Initialize(aLCBS);
        for (; aItLS.More(); aItLS.Next())
        {
            Shape aCB = aItLS.Value();
            //
            // check if the Compsolid should be created
            aExp.Init(aCB, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                if (aMFCs.Contains(aExp.Current()))
                {
                    break;
                }
            }
            //
            if (!aExp.More())
            {
                // add solids directly into result as their origins are not Compsolids
                for (aIt.Initialize(aCB); aIt.More(); aIt.Next())
                {
                    aBB.Add(aResult, aIt.Value());
                }
                continue;
            }
            //
            // make Compsolid
            Shape aCS = new();
            BOPTools_AlgoTools.MakeContainer(ShapeType.CompSolid, ref aCS);
            //
            aIt.Initialize(aCB);
            for (; aIt.More(); aIt.Next())
            {
                aBB.Add(aCS, aIt.Value());
            }
            //
            aBB.Add(aResult, aCS);
        }
        //
        myShape = aResult;
    }

    /// <summary>
    /// True if an argument solid is open and the operation should be built from the split faces (BuildBOP), as the
    /// solid builder can't be expected to split open solids well.
    /// </summary>
    protected virtual bool CheckArgsForOpenSolid()
    {
        // Analyze the report to find if BuilderSolid has generated warnings
        // for any of the solids and collect these solids to check if they are open.
        NCollection_Map<Shape> aFailedSolids = new(ShapeHasher.Instance);
        {
            NCollection_List<Alert> aList = myReport.GetAlerts(Warning);
            for (NCollection_List<Alert>.Iterator aIt = new(aList); aIt.More(); aIt.Next())
            {
                Type aType = aIt.Value().GetType();
                if (aType != typeof(BOPAlgo_AlertSolidBuilderUnusedFaces))
                {
                    continue;
                }

                if (aIt.Value() is AlertWithShape aShapeAlert)
                {
                    Shape aWarnShape = aShapeAlert.InternalGetShape();
                    if (!aWarnShape.IsNull)
                    {
                        Explorer expS = new(aWarnShape, ShapeType.Solid);
                        for (; expS.More(); expS.Next())
                        {
                            aFailedSolids.Add(expS.Current());
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

        int aNbS = myDS!.NbSourceShapes();
        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }

            Shape aSolid = aSI.Shape();

            // Check that not INTERNAL faces create closed loops
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
            // Collect all splits of internal faces
            NCollection_Map<Shape> aMFInternal = new(ShapeHasher.Instance);

            for (Iterator itSh = new(aSolid); itSh.More(); itSh.Next())
            {
                Shape aSh = itSh.Value();
                if (aSh.ShapeType() != ShapeType.Shell)
                {
                    continue;
                }

                for (Iterator itF = new(aSh); itF.More(); itF.Next())
                {
                    Shape aF = itF.Value();
                    if (aF.Orientation() == Internal)
                    {
                        if (myImages.Find(aF, out NCollection_List<Shape> pLFIm))
                        {
                            NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                            for (; itLFIm.More(); itLFIm.Next())
                            {
                                aMFInternal.Add(itLFIm.Value());
                            }
                        }
                        else
                        {
                            aMFInternal.Add(aF);
                        }
                    }
                    else
                    {
                        Exploration.MapShapesAndAncestors(aF, ShapeType.Edge, ShapeType.Face, aMEF);
                    }
                }
            }

            // Analyze the Edge-Face connection map on free edges
            bool isClosed = true;
            int aNbE = aMEF.Extent();
            for (int j = 1; j <= aNbE && isClosed; ++j)
            {
                Edge aE = ShapeCasts.Edge(aMEF.FindKey(j));
                if (Tool.Degenerated(aE))
                {
                    // Skip degenerated edges
                    continue;
                }

                isClosed = aMEF[j].Extent() > 1;
                if (!isClosed)
                {
                    Face aF = ShapeCasts.Face(aMEF[j].First());
                    isClosed = Tool.IsClosed(aE, aF); // Check for seam edges
                    if (!isClosed)
                    {
                        // Check if the edge is not internal in the face
                        Explorer expE = new(aF, ShapeType.Edge);
                        for (; expE.More(); expE.Next())
                        {
                            if (expE.Current().IsSame(aE))
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

            if (aFailedSolids.Contains(aSolid))
            {
                // Warning has been generated for this solid, return positive result right away.
                return true;
            }

            // Check the splits not to acquire new INTERNAL faces
            if (!myImages.Find(aSolid, out NCollection_List<Shape> pLSIm))
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itLSIm = new(pLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape aSIm = itLSIm.Value();
                for (Iterator itSh = new(aSIm); itSh.More(); itSh.Next())
                {
                    Shape aSh = itSh.Value();
                    if (aSh.ShapeType() != ShapeType.Shell)
                    {
                        continue;
                    }

                    for (Iterator itF = new(aSh); itF.More(); itF.Next())
                    {
                        Shape aF = itF.Value();
                        if (aF.Orientation() == Internal)
                        {
                            if (!aMFInternal.Contains(aF))
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

    private static ShapeType TypeToExplore(int theDim)
    {
        ShapeType aRet;
        //
        switch (theDim)
        {
            case 0:
                aRet = ShapeType.Vertex;
                break;
            case 1:
                aRet = ShapeType.Edge;
                break;
            case 2:
                aRet = ShapeType.Face;
                break;
            case 3:
                aRet = ShapeType.Solid;
                break;
            default:
                aRet = global::Kerf.Topology.ShapeType.Shape;
                break;
        }
        return aRet;
    }

    private static void CollectContainers(in Shape theS, NCollection_List<Shape> theLSC)
    {
        ShapeType aType = theS.ShapeType();
        if (aType == ShapeType.Wire || aType == ShapeType.Shell || aType == ShapeType.CompSolid)
        {
            theLSC.Append(theS);
            return;
        }
        //
        if (aType != ShapeType.Compound)
        {
            return;
        }
        //
        Iterator aIt = new(theS);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            CollectContainers(aS, theLSC);
        }
    }

    // Filters the containers with identical contents
    private static void RemoveDuplicates(NCollection_List<Shape> theContainers)
    {
        RemoveDuplicates(theContainers, ShapeType.Wire);
        RemoveDuplicates(theContainers, ShapeType.Shell);
        RemoveDuplicates(theContainers, ShapeType.CompSolid);
    }

    // Filters the containers of given type with identical contents
    private static void RemoveDuplicates(NCollection_List<Shape> theContainers, ShapeType theType)
    {
        // get containers of given type
        NCollection_List<Shape> aLC = new();
        NCollection_List<Shape>.Iterator aItLC = new(theContainers);
        for (; aItLC.More(); aItLC.Next())
        {
            Shape aC = aItLC.Value();
            if (aC.ShapeType() == theType)
            {
                aLC.Append(aC);
            }
        }
        //
        if (aLC.IsEmpty())
        {
            return;
        }
        //
        // map containers to compare its contents (OCCT's default hasher: orientation included, decision 721)
        NCollection_IndexedDataMap<Shape, NCollection_Map<Shape>> aContents = new(EqualityComparer<Shape>.Default);
        //
        aItLC.Initialize(aLC);
        for (; aItLC.More(); aItLC.Next())
        {
            Shape aC = aItLC.Value();
            //
            NCollection_Map<Shape> aMC = aContents[aContents.Add(aC, new NCollection_Map<Shape>(ShapeHasher.Instance))];
            //
            Iterator aIt = new(aC);
            for (; aIt.More(); aIt.Next())
            {
                aMC.Add(aIt.Value());
            }
        }
        //
        // compare the contents of the containers and find duplicates
        NCollection_Map<Shape> aDuplicates = new(ShapeHasher.Instance);
        //
        int i, j, aNb = aContents.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aCi = aContents.FindKey(i);
            if (aDuplicates.Contains(aCi))
            {
                continue;
            }
            NCollection_Map<Shape> aMi = aContents[i];
            int aNbi = aMi.Extent();
            //
            for (j = i + 1; j <= aNb; ++j)
            {
                Shape aCj = aContents.FindKey(j);
                if (aDuplicates.Contains(aCj))
                {
                    continue;
                }
                NCollection_Map<Shape> aMj = aContents[j];
                int aNbj = aMj.Extent();
                //
                int aNbCommon = NbCommonItemsInMap(aMi, aMj);
                //
                if (aNbj == aNbCommon)
                {
                    aDuplicates.Add(aCj);
                    continue;
                }
                //
                if (aNbi == aNbCommon)
                {
                    aDuplicates.Add(aCi);
                    break;
                }
            }
        }
        //
        if (aDuplicates.IsEmpty())
        {
            return;
        }
        //
        // remove duplicating containers
        aItLC.Initialize(theContainers);
        for (; aItLC.More();)
        {
            Shape aC = aItLC.Value();
            if (aDuplicates.Contains(aC))
            {
                theContainers.Remove(aItLC);
                continue;
            }
            aItLC.Next();
        }
    }

    // Counts the items contained in both maps
    private static int NbCommonItemsInMap(NCollection_Map<Shape> theM1, NCollection_Map<Shape> theM2)
    {
        NCollection_Map<Shape> aMap1 = theM1;
        NCollection_Map<Shape> aMap2 = theM2;
        //
        if (theM2.Extent() < theM1.Extent())
        {
            aMap1 = theM2;
            aMap2 = theM1;
        }
        //
        int iCommon = 0;
        foreach (Shape aS in aMap1)
        {
            if (aMap2.Contains(aS))
            {
                ++iCommon;
            }
        }
        return iCommon;
    }

    // Stores the faces of the given solid into outgoing maps:
    // <theMFS> - not internal faces with reference to solid.
    private static void MapFacesToBuildSolids(in Shape theSol,
                                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMFS)
    {
        Explorer aExp = new(theSol, ShapeType.Face);
        for (; aExp.More(); aExp.Next())
        {
            Shape aF = aExp.Current();
            //
            if (aF.Orientation() == Internal)
            {
                continue;
            }
            //
            int anIndex = theMFS.FindIndex(aF);
            if (anIndex == 0)
            {
                NCollection_List<Shape> pLSol = theMFS[theMFS.Add(aF, new NCollection_List<Shape>())];
                pLSol.Append(theSol);
            }
            else
            {
                NCollection_List<Shape> pLSol = theMFS[anIndex];
                Shape aF1 = theMFS.FindKey(anIndex);
                if (aF1.Orientation() != aF.Orientation())
                {
                    pLSol.Append(theSol);
                }
            }
        }
    }
}
