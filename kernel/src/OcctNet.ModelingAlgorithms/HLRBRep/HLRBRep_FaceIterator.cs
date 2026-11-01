// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Iterates on the edges of the wires of a face's hidden-line data.</summary>
/// <remarks>A working object, as OCCT's.</remarks>
public sealed class HLRBRep_FaceIterator
{
    private int iWire;
    private int nbWires;
    private int iEdge;
    private int nbEdges;
    private HLRAlgo_WiresBlock? myWires;
    private HLRAlgo_EdgesBlock? myEdges;

    public HLRBRep_FaceIterator()
    {
    }

    /// <summary>Starts on the first edge of the first wire of fd.</summary>
    public void InitEdge(HLRBRep_FaceData fd)
    {
        iWire = 0;
        myWires = fd.Wires();
        nbWires = myWires!.NbWires();

        iEdge = 0;
        nbEdges = 0;
        NextEdge();
    }

    public bool MoreEdge() => iWire <= nbWires;

    public void NextEdge()
    {
        iEdge++;
        if (iEdge > nbEdges)
        {
            iWire++;
            if (iWire <= nbWires)
            {
                iEdge = 1;
                myEdges = myWires!.Wire(iWire);
                nbEdges = myEdges!.NbEdges();
            }
        }
    }

    public bool BeginningOfWire() => iEdge == 1;

    public bool EndOfWire() => iEdge == nbEdges;

    public void SkipWire()
    {
        iEdge = nbEdges;
        NextEdge();
    }

    public HLRAlgo_EdgesBlock Wire() => myWires!.Wire(iWire)!;

    public int Edge() => myEdges!.Edge(iEdge);

    public TopAbs_Orientation Orientation() => myEdges!.Orientation(iEdge);

    public bool OutLine() => myEdges!.OutLine(iEdge);

    public bool Internal() => myEdges!.Internal(iEdge);

    public bool Double() => myEdges!.Double(iEdge);

    public bool IsoLine() => myEdges!.IsoLine(iEdge);
}
