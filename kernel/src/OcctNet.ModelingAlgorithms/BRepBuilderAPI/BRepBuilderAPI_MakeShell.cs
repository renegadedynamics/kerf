// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_ShellError;
using static Occt.BRepLib_ShellError;

namespace Occt;

/// <summary>Builds a shell on a surface (the skin of a surface), split into C2 faces along its non-regular curves if <c>Segment</c>.</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeShell"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakeShell : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeShell myMakeShell;

    // OCCT's "if (myMakeShell.IsDone()) { Done(); myShape = myMakeShell.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeShell.IsDone())
        {
            Done();
            myShape = myMakeShell.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeShell()
    {
        myMakeShell = new BRepLib_MakeShell();
    }

    public BRepBuilderAPI_MakeShell(Surface S, bool Segment = false)
    {
        myMakeShell = new BRepLib_MakeShell(S, Segment);
        TakeResult();
    }

    public BRepBuilderAPI_MakeShell(Surface S, double UMin, double UMax, double VMin, double VMax, bool Segment = false)
    {
        myMakeShell = new BRepLib_MakeShell(S, UMin, UMax, VMin, VMax, Segment);
        TakeResult();
    }

    public void Init(Surface S, double UMin, double UMax, double VMin, double VMax, bool Segment = false)
    {
        myMakeShell.Init(S, UMin, UMax, VMin, VMax, Segment);
        TakeResult();
    }


    public override bool IsDone()
    {
        return myMakeShell.IsDone();
    }

    /// <summary>The construction status.</summary>
    public BRepBuilderAPI_ShellError Error()
    {
        switch (myMakeShell.Error())
        {
            case BRepLib_ShellDone:
                return BRepBuilderAPI_ShellDone;
            case BRepLib_EmptyShell:
                return BRepBuilderAPI_EmptyShell;
            case BRepLib_DisconnectedShell:
                return BRepBuilderAPI_DisconnectedShell;
            case BRepLib_ShellParametersOutOfRange:
                return BRepBuilderAPI_ShellParametersOutOfRange;
        }

        // portage WNT
        return BRepBuilderAPI_ShellDone;
    }

    /// <summary>The shell (frozen). As in OCCT, it doesn't check <see cref="IsDone"/>.</summary>
    public Shell Shell()
    {
        return myMakeShell.Shell();
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        return myMakeShell.InternalShell();
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(BRepBuilderAPI_MakeShell theMaker) => theMaker.Shell();
}
