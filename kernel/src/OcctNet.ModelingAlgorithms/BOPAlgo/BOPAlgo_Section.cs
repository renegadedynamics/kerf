// Unit 6A stub of OCCT's BOPAlgo_Section: the API the BRepAlgoAPI shell calls, replaced by unit 6F's port.
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The section of the arguments: the edges and vertices where they meet.</summary>
/// <remarks>
/// <b>Unit 6A stub</b> (decision 6181): <see cref="CheckData"/> is OCCT's; on arguments that don't interfere (the
/// stub pave filler fails otherwise) the section is an empty compound, and every argument sub-shape is deleted in the
/// history. BuildSection is unit 6F's.
/// </remarks>
public class BOPAlgo_Section : BOPAlgo_Builder
{
    public BOPAlgo_Section()
    {
    }

    protected override void CheckData()
    {
        int aNbArgs;
        //
        aNbArgs = myArguments.Extent();
        if (aNbArgs == 0)
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>The stub's section of non-interfering arguments: nothing.</summary>
    protected override void BuildStubResult()
    {
    }
}
