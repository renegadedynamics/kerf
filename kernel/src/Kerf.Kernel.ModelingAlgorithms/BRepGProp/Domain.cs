// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Arc iterator over the boundary of a face: the edges of the face, forward, skipping INTERNAL and EXTERNAL edges after
/// the first, in an undigested order.
/// </summary>
/// <remarks>A working object, as OCCT's (an explorer): its iteration methods change it.</remarks>
internal sealed class Domain
{
    private readonly Explorer _explorer;

    /// <summary>An empty domain.</summary>
    public Domain() => _explorer = new Explorer();

    /// <summary>The domain of the face.</summary>
    public Domain(in Face f) => _explorer = new Explorer(f.Oriented(Forward), ShapeType.Edge);

    /// <summary>Initializes the domain with the face.</summary>
    public void Init(in Face f) => _explorer.Init(f.Oriented(Forward), ShapeType.Edge);

    /// <summary>True if there is another arc of curve in the list.</summary>
    public bool More() => _explorer.More();

    /// <summary>Restarts the exploration of the face already set.</summary>
    public void Init() => _explorer.ReInit();

    /// <summary>The current edge.</summary>
    public Edge Value() => ShapeCasts.Edge(_explorer.Current());

    /// <summary>Moves to the next FORWARD or REVERSED edge.</summary>
    public void Next()
    {
        // skip INTERNAL and EXTERNAL edges
        _explorer.Next();
        while (_explorer.More())
        {
            Orientation or = _explorer.Current().Orientation;
            if ((or == Forward) || (or == Reversed))
            {
                return;
            }

            _explorer.Next();
        }
    }
}
