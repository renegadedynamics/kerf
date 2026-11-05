// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>A list of loops with an iteration on them.</summary>
/// <remarks>A working object.</remarks>
internal class LoopSet
{
    private readonly NCollection_List<BuildLoop> _listOfLoop = new();
    private readonly NCollection_List<BuildLoop>.Iterator _loopIterator = new();
#pragma warning disable CS0414 // OCCT counts the loops without reading the counts
    private int _loopIndex;
    private int _nbLoop;
#pragma warning restore CS0414

    /// <summary>An empty set.</summary>
    public LoopSet()
    {
        _loopIndex = 1;
        _nbLoop = 0;
    }

    /// <summary>The list of loops, to fill.</summary>
    public NCollection_List<BuildLoop> ChangeListOfLoop() => _listOfLoop;

    /// <summary>Starts the iteration on the loops.</summary>
    public virtual void InitLoop()
    {
        _loopIterator.Initialize(_listOfLoop);
        _loopIndex = 1;
        _nbLoop = _listOfLoop.Extent();
    }

    /// <summary>Whether there is a current loop.</summary>
    public virtual bool MoreLoop()
    {
        bool b = _loopIterator.More();
        return b;
    }

    /// <summary>Moves to the next loop.</summary>
    public virtual void NextLoop()
    {
        _loopIndex++;
        _loopIterator.Next();
    }

    /// <summary>The current loop.</summary>
    public virtual BuildLoop Loop()
    {
        BuildLoop l = _loopIterator.Value();
        return l;
    }
}
