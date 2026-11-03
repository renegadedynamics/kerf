// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Iterator.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Iterator.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Iterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;

namespace Occt;

/// <summary>Iterates on the direct sub-shapes of a generating shape (a TopoDS_Iterator).</summary>
internal sealed class BRepSweep_Iterator
{
    private readonly Iterator myIterator = new();

    public BRepSweep_Iterator()
    {
    }

    /// <summary>Resets the iterator on the sub-shapes of <paramref name="aShape"/>.</summary>
    public void Init(in Shape aShape) => myIterator.Initialize(aShape);

    public bool More() => myIterator.More();

    public void Next() => myIterator.Next();

    public Shape Value() => myIterator.Value();

    public Orientation Orientation() => Value().Orientation();
}
