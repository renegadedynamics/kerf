// Unit 6A stub of OCCT's BOPAlgo_BOP: the API the BRepAlgoAPI shell calls, replaced by unit 6F's port.
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_Operation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The Boolean operation (FUSE, COMMON, CUT, CUT21) of objects and tools.</summary>
/// <remarks>
/// <b>Unit 6A stub</b> (decision 638). <see cref="CheckData"/> is OCCT's (operation, counts, the dimension rules);
/// the stub's result is OCCT's for objects and tools that don't interfere (the stub pave filler fails otherwise):
/// FUSE keeps every argument, COMMON nothing, CUT the objects, CUT21 the tools; the arguments left out are deleted in
/// the history. The empty-shape treatment (TreatEmptyShape), the open-solid check and the solid building are unit 6F's.
/// </remarks>
public class BOPAlgo_BOP : BOPAlgo_ToolsProvider
{
    protected BOPAlgo_Operation myOperation;
    protected int[] myDims = [-1, -1];

    public BOPAlgo_BOP()
    {
        myOperation = BOPAlgo_UNKNOWN;
    }

    public override void Clear()
    {
        myOperation = BOPAlgo_UNKNOWN;
        myDims[0] = -1;
        myDims[1] = -1;

        base.Clear();
    }

    public void SetOperation(BOPAlgo_Operation theOperation) => myOperation = theOperation;

    public BOPAlgo_Operation Operation() => myOperation;

    protected override void CheckData()
    {
        int i, aNbArgs, aNbTools;
        bool bFuse;
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
        // 3. CUT21:  The MINIMAL dimension of ARGUMENTS should be greater
        //            or equal to the MAXIMAL dimension of the TOOLS;
        // 4. COMMON: The arguments and tools could have any dimensions.
        //
        int[] iDimMin = [3, 3], iDimMax = [0, 0];
        bool[] bHasValid = [false, false];
        //
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aLS = i == 0 ? myArguments : myTools;
            for (NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLS); aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aS = aItLS.Value();
                bool bIsEmpty = StubIsEmptyShape(aS);
                if (bIsEmpty)
                {
                    AddWarning(new BOPAlgo_AlertEmptyShape(aS));
                    continue;
                }

                StubDimensions(aS, out int iDMin, out int iDMax);

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

    /// <summary>The stub's result on non-interfering arguments: each operation's kept group.</summary>
    protected override void BuildStubResult()
    {
        BRep_Builder aBB = new();
        bool bObjects = myOperation == BOPAlgo_FUSE || myOperation == BOPAlgo_CUT;
        bool bTools = myOperation == BOPAlgo_FUSE || myOperation == BOPAlgo_CUT21;
        if (bObjects)
        {
            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(myArguments); aIt.More(); aIt.Next())
            {
                aBB.Add(myShape, aIt.Value());
            }
        }
        if (bTools)
        {
            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(myTools); aIt.More(); aIt.Next())
            {
                aBB.Add(myShape, aIt.Value());
            }
        }
    }

    // Stand-in for BOPTools_AlgoTools3D::IsEmptyShape (unit 6B): a shape with no vertex has no geometry.
    internal static bool StubIsEmptyShape(in TopoDS_Shape theS)
    {
        if (theS.IsNull())
        {
            return true;
        }
        return !new TopExp_Explorer(theS, TopAbs_VERTEX).More();
    }

    // Stand-in for BOPTools_AlgoTools::Dimensions (unit 6B): the smallest and largest dimension of the shape's
    // non-empty parts (a compound's components recursively).
    internal static void StubDimensions(in TopoDS_Shape theS, out int theDMin, out int theDMax)
    {
        theDMin = 3;
        theDMax = 0;
        if (theS.ShapeType() != TopAbs_COMPOUND)
        {
            theDMin = theDMax = StubDimension(theS.ShapeType());
            return;
        }
        bool bFound = false;
        for (TopoDS_Iterator aIt = new(theS); aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSx = aIt.Value();
            if (StubIsEmptyShape(aSx))
            {
                continue;
            }
            StubDimensions(aSx, out int aMin, out int aMax);
            theDMin = Math.Min(theDMin, aMin);
            theDMax = Math.Max(theDMax, aMax);
            bFound = true;
        }
        if (!bFound)
        {
            theDMin = theDMax = -1;
        }
    }

    private static int StubDimension(TopAbs_ShapeEnum theType) => theType switch
    {
        TopAbs_VERTEX => 0,
        TopAbs_EDGE or TopAbs_WIRE => 1,
        TopAbs_FACE or TopAbs_SHELL => 2,
        _ => 3,
    };
}
