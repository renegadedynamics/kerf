// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Indexes the sub-shapes of the generating shape of a sweep (TopExp::MapShapes).</summary>
internal sealed class SweepingTool
{
    private readonly IndexedSet<Shape> _map = new(ShapeHasher.Instance);

    public SweepingTool(in Shape aShape)
    {
        Exploration.MapShapes(aShape, _map);
    }

    public int ShapeCount => _map.Count;

    /// <summary>The index of the shape (0 if it isn't a sub-shape).</summary>
    public int Index(in Shape aShape)
    {
        if (!_map.Contains(aShape))
        {
            return 0;
        }
        return _map.IndexOf(aShape) + 1;
    }

    public Shape Shape(int anIndex) => _map[anIndex - 1];

    public ShapeType Type(in Shape aShape) => aShape.Type;

    public Orientation Orientation(in Shape aShape) => aShape.Orientation;

    public void SetOrientation(ref Shape aShape, Orientation or) => aShape.Orient(or);
}
