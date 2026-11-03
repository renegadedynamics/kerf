// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Splitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Splitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Splits the objects by the tools; the result holds the split objects only.</summary>
/// <remarks>
/// The objects and tools are intersected together (General Fuse), and the result is built from the splits of the
/// objects. With one object split into one shape (or not split), the result is that shape, not a compound.
/// </remarks>
public class BOPAlgo_Splitter : BOPAlgo_ToolsProvider
{
    public BOPAlgo_Splitter()
    {
    }

    /// <summary>Intersects the objects and tools with a new pave filler and builds the result.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        GetReport().Clear();
        //
        if (myEntryPoint == 1)
        {
            myPaveFiller = null;
        }
        //
        // prepare shapes for intersection
        NCollection_List<TopoDS_Shape> aLS = new();
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLS = new(myArguments);
        for (; aItLS.More(); aItLS.Next())
        {
            aLS.Append(aItLS.Value());
        }
        //
        aItLS.Initialize(myTools);
        for (; aItLS.More(); aItLS.Next())
        {
            aLS.Append(aItLS.Value());
        }
        //
        BOPAlgo_PaveFiller pPF = new();
        pPF.SetArguments(aLS);
        pPF.SetRunParallel(myRunParallel);

        pPF.SetFuzzyValue(myFuzzyValue);
        pPF.SetNonDestructive(myNonDestructive);
        pPF.SetGlue(myGlue);
        pPF.SetUseOBB(myUseOBB);
        //
        using var aPS = new ProgressScope(theRange, "Performing Split operation", 10);
        pPF.Perform(aPS.Next(9));
        //
        myEntryPoint = 1;
        PerformInternal(pPF, aPS.Next(1));
    }

    protected override void CheckData()
    {
        if (myArguments.IsEmpty() || (myArguments.Extent() + myTools.Extent()) < 2)
        {
            // too few arguments to process
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
    }

    protected override void BuildResult(TopAbs_ShapeEnum theType)
    {
        base.BuildResult(theType);

        if (theType == TopAbs_COMPOUND)
        {
            // The method is called for the last time for this operation.
            // If there is only one argument shape and it has been modified into
            // a single shape, or has not been modified at all, the result shape
            // has to be overwritten to avoid the unnecessary enclosure into compound.
            if (myArguments.Extent() == 1)
            {
                TopoDS_Iterator it = new(myShape);
                if (it.More())
                {
                    TopoDS_Shape aSFirst = it.Value();
                    it.Next();
                    if (!it.More())
                    {
                        myShape = aSFirst;
                    }
                }
            }
        }
    }
}
