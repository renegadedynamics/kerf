// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Indexes the sub-shapes of the generating shape of a sweep (TopExp::MapShapes).</summary>
internal sealed class SweepingTool
{
    private readonly NCollection_IndexedMap<Shape> _map = new(ShapeHasher.Instance);

    public SweepingTool(in Shape aShape)
    {
        Exploration.MapShapes(aShape, _map);
    }

    public int ShapeCount => _map.Extent();

    /// <summary>The index of the shape (0 if it isn't a sub-shape).</summary>
    public int Index(in Shape aShape)
    {
        if (!_map.Contains(aShape))
        {
            return 0;
        }
        return _map.FindIndex(aShape);
    }

    public Shape Shape(int anIndex) => _map.FindKey(anIndex);

    public ShapeType Type(in Shape aShape) => aShape.ShapeType();

    public Orientation Orientation(in Shape aShape) => aShape.Orientation();

    public void SetOrientation(ref Shape aShape, Orientation or) => aShape.Orientation(or);
}
