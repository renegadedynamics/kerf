// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Arc iterator over the boundary of a face: the edges of the face, forward, skipping INTERNAL and EXTERNAL edges after
/// the first, in an undigested order.
/// </summary>
/// <remarks>A working object, as OCCT's (an explorer): its iteration methods change it.</remarks>
public sealed class BRepGProp_Domain
{
    private readonly Explorer myExplorer;

    /// <summary>An empty domain.</summary>
    public BRepGProp_Domain() => myExplorer = new Explorer();

    /// <summary>The domain of the face.</summary>
    public BRepGProp_Domain(in Face F) => myExplorer = new Explorer(F.Oriented(Forward), ShapeType.Edge);

    /// <summary>Initializes the domain with the face.</summary>
    public void Init(in Face F) => myExplorer.Init(F.Oriented(Forward), ShapeType.Edge);

    /// <summary>True if there is another arc of curve in the list.</summary>
    public bool More() => myExplorer.More();

    /// <summary>Restarts the exploration of the face already set.</summary>
    public void Init() => myExplorer.ReInit();

    /// <summary>The current edge.</summary>
    public Edge Value() => ShapeCasts.Edge(myExplorer.Current());

    /// <summary>Moves to the next FORWARD or REVERSED edge.</summary>
    public void Next()
    {
        // skip INTERNAL and EXTERNAL edges
        myExplorer.Next();
        while (myExplorer.More())
        {
            Orientation Or = myExplorer.Current().Orientation();
            if ((Or == Forward) || (Or == Reversed))
            {
                return;
            }

            myExplorer.Next();
        }
    }
}
