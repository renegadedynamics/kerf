// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A list of loops with an iteration on them.</summary>
/// <remarks>A working object.</remarks>
public class TopOpeBRepBuild_LoopSet
{
    private readonly NCollection_List<TopOpeBRepBuild_Loop> myListOfLoop = new();
    private readonly NCollection_List<TopOpeBRepBuild_Loop>.Iterator myLoopIterator = new();
#pragma warning disable CS0414 // OCCT counts the loops without reading the counts
    private int myLoopIndex;
    private int myNbLoop;
#pragma warning restore CS0414

    /// <summary>An empty set.</summary>
    public TopOpeBRepBuild_LoopSet()
    {
        myLoopIndex = 1;
        myNbLoop = 0;
    }

    /// <summary>The list of loops, to fill.</summary>
    public NCollection_List<TopOpeBRepBuild_Loop> ChangeListOfLoop() => myListOfLoop;

    /// <summary>Starts the iteration on the loops.</summary>
    public virtual void InitLoop()
    {
        myLoopIterator.Initialize(myListOfLoop);
        myLoopIndex = 1;
        myNbLoop = myListOfLoop.Extent();
    }

    /// <summary>Whether there is a current loop.</summary>
    public virtual bool MoreLoop()
    {
        bool b = myLoopIterator.More();
        return b;
    }

    /// <summary>Moves to the next loop.</summary>
    public virtual void NextLoop()
    {
        myLoopIndex++;
        myLoopIterator.Next();
    }

    /// <summary>The current loop.</summary>
    public virtual TopOpeBRepBuild_Loop Loop()
    {
        TopOpeBRepBuild_Loop L = myLoopIterator.Value();
        return L;
    }
}
