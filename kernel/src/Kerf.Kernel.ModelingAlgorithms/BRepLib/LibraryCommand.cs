// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_Command.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_Command.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Construction.Library;

/// <summary>The root of the BRepLib makers: a command that may or may not be done.</summary>
internal abstract class LibraryCommand
{
    private bool _done;

    protected LibraryCommand()
    {
        _done = false;
    }

    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    /// <summary>Throws <see cref="NotDoneException"/> if the command is not done.</summary>
    public void Check()
    {
        if (!_done)
        {
            throw new NotDoneException("BRep_API: command not done");
        }
    }

    /// <summary>Sets the done flag.</summary>
    protected void Done()
    {
        _done = true;
    }

    /// <summary>Clears the done flag.</summary>
    protected void NotDone()
    {
        _done = false;
    }
}
