// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetAncestors.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetAncestors.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Filling;

/// <summary>The ancestors (spine edges and vertices) of the edges of a parallel built by BRepFill_OffsetWire.</summary>
/// <remarks>A working object (decision 21); it reads the offset's internal (unfrozen) forms.</remarks>
internal sealed class OffsetAncestors
{
    private bool _isPerform;
    private readonly Dictionary<Shape, Shape> _map = new(ShapeHasher.Instance);

    /// <summary>An empty tool.</summary>
    public OffsetAncestors()
    {
        _isPerform = false;
    }

    /// <summary>The ancestors of the parallel of <paramref name="paral"/>.</summary>
    public OffsetAncestors(OffsetWire paral)
    {
        Perform(paral);
    }

    /// <summary>Computes the ancestors of the parallel of <paramref name="paral"/>.</summary>
    public void Perform(OffsetWire paral)
    {
        Face spine = paral.InternalSpine();

        Explorer exp = new();
        NCollection_List<Shape>.Iterator it = new();

        // on itere sur les edges.
        for (exp.Init(spine, ShapeType.Edge); exp.More(); exp.Next())
        {
            for (it.Initialize(paral.InternalGeneratedShapes(exp.Current())); it.More(); it.Next())
            {
                _map[it.Value()] = exp.Current();
            }
        }

        // on itere sur les vertex.
        for (exp.Init(spine, ShapeType.Vertex); exp.More(); exp.Next())
        {
            for (it.Initialize(paral.InternalGeneratedShapes(exp.Current())); it.More(); it.Next())
            {
                _map[it.Value()] = exp.Current();
            }
        }

        _isPerform = true;
    }

    /// <summary>True once <see cref="Perform"/> has run.</summary>
    public bool IsDone => _isPerform;

    /// <summary>True if the edge <paramref name="s1"/> of the parallel has an ancestor.</summary>
    public bool HasAncestor(in Edge s1) => _map.ContainsKey(s1);

    /// <summary>The ancestor of the edge <paramref name="s1"/> of the parallel.</summary>
    /// <exception cref="NotDoneException"><see cref="Perform"/> hasn't run.</exception>
    public Shape Ancestor(in Edge s1)
    {
        StdFail_NotDone_Raise_if(!_isPerform,
            "BRepFill_OffsetAncestors::Ancestor() - Perform() should be called before accessing results");
        return _map.GetValue(s1);
    }
}
