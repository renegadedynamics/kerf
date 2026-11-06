// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a shell on a surface (the skin of a surface), split into C2 faces along its non-regular curves if <c>Segment</c>.</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeShell"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
internal class MakeShell : MakeShape
{
    private readonly LibraryMakeShell _makeShell;

    // OCCT's "if (myMakeShell.IsDone()) { Done(); myShape = myMakeShell.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeShell.IsDone)
        {
            Done();
            _shape = _makeShell.InternalShape();
        }
    }

    public MakeShell()
    {
        _makeShell = new LibraryMakeShell();
    }

    public MakeShell(Surface s, bool segment = false)
    {
        _makeShell = new LibraryMakeShell(s, segment);
        TakeResult();
    }

    public MakeShell(Surface s, double uMin, double uMax, double vMin, double vMax, bool segment = false)
    {
        _makeShell = new LibraryMakeShell(s, uMin, uMax, vMin, vMax, segment);
        TakeResult();
    }

    public void Init(Surface s, double uMin, double uMax, double vMin, double vMax, bool segment = false)
    {
        _makeShell.Init(s, uMin, uMax, vMin, vMax, segment);
        TakeResult();
    }


    public override bool IsDone
    {
        get
        {
            return _makeShell.IsDone;
        }
    }

    /// <summary>The construction status.</summary>
    public ShellError Error()
    {
        switch (_makeShell.Error())
        {
            case LibraryShellError.ShellDone:
                return ShellError.ShellDone;
            case LibraryShellError.EmptyShell:
                return ShellError.EmptyShell;
            case LibraryShellError.DisconnectedShell:
                return ShellError.DisconnectedShell;
            case LibraryShellError.ShellParametersOutOfRange:
                return ShellError.ShellParametersOutOfRange;
        }

        // portage WNT
        return ShellError.ShellDone;
    }

    /// <summary>The shell (frozen). As in OCCT, it doesn't check <see cref="IsDone"/>.</summary>
    public Shell Shell()
    {
        return _makeShell.Shell();
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        return _makeShell.InternalShell();
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(MakeShell maker) => maker.Shell();
}
