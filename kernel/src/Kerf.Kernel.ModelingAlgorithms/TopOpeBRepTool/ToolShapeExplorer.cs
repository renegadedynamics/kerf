// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeExplorer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>A <see cref="Explorer"/> that also counts the sub-shapes it has met (<see cref="Index"/>).</summary>
/// <remarks>
/// OCCT derives it from TopExp_Explorer; the port's explorer is sealed, so this one holds an explorer and forwards to
/// it (decision 791). <c>DumpCurrent</c> is left out (decision 493: no console output).
/// </remarks>
internal sealed class ToolShapeExplorer
{
    private Explorer _explorer = new();
    private int _index;

    /// <summary>An empty explorer, useful after <see cref="Init"/>.</summary>
    public ToolShapeExplorer()
    {
        _index = 0;
    }

    /// <summary>
    /// An explorer of the sub-shapes of <paramref name="s"/> of type <paramref name="toFind"/>, skipping those of type
    /// <paramref name="toAvoid"/>.
    /// </summary>
    public ToolShapeExplorer(in Shape s, ShapeType toFind, ShapeType toAvoid = ShapeType.Shape)
    {
        _explorer.Init(s, toFind, toAvoid);
        _index = More() ? 1 : 0;
    }

    /// <summary>Starts the exploration of <paramref name="s"/>.</summary>
    public void Init(in Shape s, ShapeType toFind, ShapeType toAvoid = ShapeType.Shape)
    {
        _explorer.Init(s, toFind, toAvoid);
        _index = More() ? 1 : 0;
    }

    /// <summary>Whether there is a current sub-shape.</summary>
    public bool More() => _explorer.More();

    /// <summary>Moves to the next sub-shape.</summary>
    public void Next()
    {
        if (More())
        {
            _index++;
        }
        _explorer.Next();
    }

    /// <summary>The current sub-shape.</summary>
    public Shape Current() => _explorer.Current();

    /// <summary>The index of the current sub-shape.</summary>
    public int Index() => _index;
}
