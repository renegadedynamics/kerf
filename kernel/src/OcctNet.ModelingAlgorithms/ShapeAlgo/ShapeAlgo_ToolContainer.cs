// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo_ToolContainer.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo_ToolContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>Returns the tools used by <see cref="ShapeAlgo_AlgoContainer"/> (new ones at each call).</summary>
/// <remarks>A working object (decision 21), to be derived from to substitute other tools.</remarks>
public class ShapeAlgo_ToolContainer : Transient
{
    /// <summary>Empty constructor.</summary>
    public ShapeAlgo_ToolContainer()
    {
    }

    /// <summary>Returns a new ShapeFix_Shape.</summary>
    public virtual ShapeFix_Shape FixShape() => new ShapeFix_Shape();

    /// <summary>Returns a new ShapeFix_EdgeProjAux.</summary>
    public virtual ShapeFix_EdgeProjAux EdgeProjAux() => new ShapeFix_EdgeProjAux();
}
