// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Algo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Algo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// The exact hidden-line removal on shapes: add shapes (each in an outliner), set the projector, <c>Update</c>,
/// <c>Hide</c>, then read the visible and hidden lines with HLRBRep_HLRToShape.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. The caller's shapes are not written into: the outliners copy frozen
/// shapes at the entry (decision 1095).
/// </remarks>
public class HLRBRep_Algo : HLRBRep_InternalAlgo
{
    public HLRBRep_Algo()
    {
    }

    /// <summary>OCCT's copy (see <see cref="HLRBRep_InternalAlgo(HLRBRep_InternalAlgo)"/>).</summary>
    public HLRBRep_Algo(HLRBRep_Algo A)
        : base(A)
    {
    }

    /// <summary>Adds the shape S, with the shape data SData and nbIso isolines per face direction.</summary>
    public void Add(in Shape S, Transient? SData, int nbIso = 0)
    {
        Load(new HLRTopoBRep_OutLiner(S), SData, nbIso);
    }

    /// <summary>Adds the shape S, with nbIso isolines per face direction.</summary>
    public void Add(in Shape S, int nbIso = 0)
    {
        Load(new HLRTopoBRep_OutLiner(S), nbIso);
    }

    /// <summary>The index of the shape S (its original or outlined shape), 0 if it isn't loaded.</summary>
    public int Index(in Shape S)
    {
        int n = NbShapes();

        for (int i = 1; i <= n; i++)
        {
            if (ShapeBounds(i).Shape()!.OriginalShape().IsEqual(S))
            {
                return i;
            }
            if (ShapeBounds(i).Shape()!.InternalOutLinedShape().IsEqual(S))
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Nullifies the outlined shapes, and clears the outliners' data, so the next update outlines again.</summary>
    public void OutLinedShapeNullify()
    {
        int n = NbShapes();

        for (int i = 1; i <= n; i++)
        {
            ShapeBounds(i).Shape()!.OutLinedShape(new Shape());
            ShapeBounds(i).Shape()!.DataStructure().Clear();
        }
    }
}
