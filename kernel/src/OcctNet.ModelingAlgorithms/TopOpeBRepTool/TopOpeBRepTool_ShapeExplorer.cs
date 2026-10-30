// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeExplorer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>A <see cref="TopExp_Explorer"/> that also counts the sub-shapes it has met (<see cref="Index"/>).</summary>
/// <remarks>
/// OCCT derives it from TopExp_Explorer; the port's explorer is sealed, so this one holds an explorer and forwards to
/// it (decision 5961). <c>DumpCurrent</c> is left out (decision 493: no console output).
/// </remarks>
public sealed class TopOpeBRepTool_ShapeExplorer
{
    private readonly TopExp_Explorer myExplorer = new();
    private int myIndex;

    /// <summary>An empty explorer, useful after <see cref="Init"/>.</summary>
    public TopOpeBRepTool_ShapeExplorer()
    {
        myIndex = 0;
    }

    /// <summary>
    /// An explorer of the sub-shapes of <paramref name="S"/> of type <paramref name="ToFind"/>, skipping those of type
    /// <paramref name="ToAvoid"/>.
    /// </summary>
    public TopOpeBRepTool_ShapeExplorer(in TopoDS_Shape S, TopAbs_ShapeEnum ToFind, TopAbs_ShapeEnum ToAvoid = TopAbs_SHAPE)
    {
        myExplorer.Init(S, ToFind, ToAvoid);
        myIndex = More() ? 1 : 0;
    }

    /// <summary>Starts the exploration of <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S, TopAbs_ShapeEnum ToFind, TopAbs_ShapeEnum ToAvoid = TopAbs_SHAPE)
    {
        myExplorer.Init(S, ToFind, ToAvoid);
        myIndex = More() ? 1 : 0;
    }

    /// <summary>Whether there is a current sub-shape.</summary>
    public bool More() => myExplorer.More();

    /// <summary>Moves to the next sub-shape.</summary>
    public void Next()
    {
        if (More())
        {
            myIndex++;
        }
        myExplorer.Next();
    }

    /// <summary>The current sub-shape.</summary>
    public TopoDS_Shape Current() => myExplorer.Current();

    /// <summary>The index of the current sub-shape.</summary>
    public int Index() => myIndex;
}
