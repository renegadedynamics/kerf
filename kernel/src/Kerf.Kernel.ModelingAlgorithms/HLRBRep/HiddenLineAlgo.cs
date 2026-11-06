// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Algo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Algo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Topology;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The exact hidden-line removal on shapes: add shapes (each in an outliner), set the projector, <c>Update</c>,
/// <c>Hide</c>, then read the visible and hidden lines with HLRBRep_HLRToShape.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. The caller's shapes are not written into: the outliners write in
/// working overlays (unit 10H6).
/// </remarks>
internal class HiddenLineAlgo : InternalAlgo
{
    public HiddenLineAlgo()
    {
    }

    /// <summary>OCCT's copy (see <see cref="InternalAlgo(InternalAlgo)"/>).</summary>
    public HiddenLineAlgo(HiddenLineAlgo a)
        : base(a)
    {
    }

    /// <summary>Adds the shape S, with the shape data SData and nbIso isolines per face direction.</summary>
    public void Add(in Shape s, Transient? sData, int nbIso = 0)
    {
        Load(new OutLiner(s), sData, nbIso);
    }

    /// <summary>Adds the shape S, with nbIso isolines per face direction.</summary>
    public void Add(in Shape s, int nbIso = 0)
    {
        Load(new OutLiner(s), nbIso);
    }

    /// <summary>The index of the shape S (its original or outlined shape), 0 if it isn't loaded.</summary>
    public int Index(in Shape s)
    {
        int n = ShapeCount;

        for (int i = 1; i <= n; i++)
        {
            if (ShapeBounds(i).Shape()!.OriginalShape().IsEqual(s))
            {
                return i;
            }
            if (ShapeBounds(i).Shape()!.InternalOutLinedShape().IsEqual(s))
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Nullifies the outlined shapes, and clears the outliners' data, so the next update outlines again.</summary>
    public void OutLinedShapeNullify()
    {
        int n = ShapeCount;

        for (int i = 1; i <= n; i++)
        {
            ShapeBounds(i).Shape()!.OutLinedShape(new Shape());
            ShapeBounds(i).Shape()!.DataStructure().Clear();
        }
    }
}
