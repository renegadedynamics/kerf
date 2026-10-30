// Unit 6A stub of OCCT's BOPAlgo_Splitter: the API the BRepAlgoAPI shell calls, replaced by unit 6F's port.
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Splits the objects by the tools; the result holds the split objects only.</summary>
/// <remarks>
/// <b>Unit 6A stub</b> (decision 638): <see cref="CheckData"/> is OCCT's; on objects and tools that don't interfere
/// (the stub pave filler fails otherwise) nothing is split, so the result holds the objects themselves and the tools'
/// sub-shapes are deleted in the history.
/// </remarks>
public class BOPAlgo_Splitter : BOPAlgo_ToolsProvider
{
    public BOPAlgo_Splitter()
    {
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
}
