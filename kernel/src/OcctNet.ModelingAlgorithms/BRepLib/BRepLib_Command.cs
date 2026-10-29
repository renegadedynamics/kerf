// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_Command.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_Command.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The root of the BRepLib makers: a command that may or may not be done.</summary>
public abstract class BRepLib_Command
{
    private bool myDone;

    protected BRepLib_Command()
    {
        myDone = false;
    }

    public bool IsDone()
    {
        return myDone;
    }

    /// <summary>Throws <see cref="StdFail_NotDone"/> if the command is not done.</summary>
    public void Check()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone("BRep_API: command not done");
        }
    }

    /// <summary>Sets the done flag.</summary>
    protected void Done()
    {
        myDone = true;
    }

    /// <summary>Clears the done flag.</summary>
    protected void NotDone()
    {
        myDone = false;
    }
}
