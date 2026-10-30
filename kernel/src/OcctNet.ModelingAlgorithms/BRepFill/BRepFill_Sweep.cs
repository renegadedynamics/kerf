// OCCT.NET placeholder, not a port: unit 5S2 replaces this file with the port of
// src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Sweep.hxx/.cxx (it needs BRepFill_TrimShellCorner, hence
// BRepAlgoAPI_Section of unit 6P). It declares, with OCCT's signatures, only the members BRepFill_Pipe calls, so that
// the pipe is ported now (decision 5881). The constructor throws: nothing here can be called silently.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Placeholder for OCCT's topological sweep (unit 5S2): every construction throws.</summary>
internal sealed class BRepFill_Sweep
{
    /// <summary>Throws <see cref="Standard_NotImplemented"/> until unit 5S2 ports the sweep.</summary>
    public BRepFill_Sweep(BRepFill_SectionLaw Section, BRepFill_LocationLaw Location, bool WithKPart)
    {
        throw new Standard_NotImplemented("BRepFill_Sweep is ported by unit 5S2");
    }

    public void SetBounds(in TopoDS_Wire FirstShape, in TopoDS_Wire LastShape) => throw NotPorted();

    public void SetForceApproxC1(bool ForceApproxC1) => throw NotPorted();

    public void Build(NCollection_Map<TopoDS_Shape> ReversedEdges,
                      NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Tapes,
                      NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Rails,
                      BRepFill_TransitionStyle Transition,
                      GeomAbs_Shape Continuity,
                      GeomFill_ApproxStyle Approx,
                      int Degmax,
                      int Segmax) => throw NotPorted();

    public bool IsDone() => throw NotPorted();

    public TopoDS_Shape Shape() => throw NotPorted();

    public double ErrorOnSurface() => throw NotPorted();

    public NCollection_HArray2<TopoDS_Shape>? SubShape() => throw NotPorted();

    public NCollection_HArray2<TopoDS_Shape>? InterFaces() => throw NotPorted();

    public NCollection_HArray2<TopoDS_Shape>? Sections() => throw NotPorted();

    public TopoDS_Shape Tape(int Index) => throw NotPorted();

    private static Standard_NotImplemented NotPorted() => new("BRepFill_Sweep is ported by unit 5S2");
}
