// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The hidden-line data of a face: its orientation and flags (selected, back, side, closed, hiding, simple, cut, with
/// outlines, and its kind of quadric), its size, its wires of edge indices, and its projected surface.
/// </summary>
/// <remarks>
/// OCCT's value class, ported as a class (decision 1092), as <see cref="HLRBRep_EdgeData"/>. OCCT's copy assignment
/// is <see cref="Assign"/>: the wires block is a handle the copies share (so <c>HLRBRep_Data::Write</c>'s renumbering
/// of a copy's edges renumbers the source's too, as in OCCT) and the surface is copied. A working object.
/// </remarks>
public sealed class HLRBRep_FaceData
{
    private const int EMaskOrient = 15;
    private const int FMaskSelected = 16;
    private const int FMaskBack = 32;
    private const int FMaskSide = 64;
    private const int FMaskClosed = 128;
    private const int FMaskHiding = 256;
    private const int FMaskSimple = 512;
    private const int FMaskCut = 1024;
    private const int FMaskWithOutL = 2048;
    private const int FMaskPlane = 4096;
    private const int FMaskCylinder = 8192;
    private const int FMaskCone = 16384;
    private const int FMaskSphere = 32768;
    private const int FMaskTorus = 65536;

    private int myFlags;
    private HLRAlgo_WiresBlock? myWires;
    private HLRBRep_Surface myGeometry = new();
    private double mySize;
    private float myTolerance;

    public HLRBRep_FaceData()
    {
        myFlags = 0;
        mySize = 0;
        Selected(true);
    }

    /// <summary>OCCT's copy assignment: the wires shared, the surface copied.</summary>
    public HLRBRep_FaceData Assign(HLRBRep_FaceData theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }
        myFlags = theOther.myFlags;
        myWires = theOther.myWires;
        myGeometry = theOther.myGeometry.Copy();
        mySize = theOther.mySize;
        myTolerance = theOther.myTolerance;
        return this;
    }

    /// <summary>Sets the face, its orientation, whether it is closed and its number of wires.</summary>
    public void Set(in TopoDS_Face FG, TopAbs_Orientation Or, bool Cl, int NW)
    {
        Closed(Cl);
        Geometry().Surface(FG);
        myTolerance = (float)(BRep_Tool.Tolerance(FG));
        Orientation(Or);
        Wires() = new HLRAlgo_WiresBlock(NW);
    }

    /// <summary>Sets the number of edges of the wire WI.</summary>
    public void SetWire(int WI, int NE)
    {
        Wires()!.Set(WI, new HLRAlgo_EdgesBlock(NE));
    }

    /// <summary>Sets the edge EWI of the wire WI: its index, orientation and flags.</summary>
    public void SetWEdge(int WI, int EWI, int EI, TopAbs_Orientation Or, bool OutL, bool Inte, bool Dble, bool IsoL)
    {
        Wires()!.Wire(WI)!.Edge(EWI, EI);
        Wires()!.Wire(WI)!.Orientation(EWI, Or);
        Wires()!.Wire(WI)!.OutLine(EWI, OutL);
        Wires()!.Wire(WI)!.Internal(EWI, Inte);
        Wires()!.Wire(WI)!.Double(EWI, Dble);
        Wires()!.Wire(WI)!.IsoLine(EWI, IsoL);
    }

    private bool Flag(int theMask) => (myFlags & theMask) != 0;

    private void Flag(int theMask, bool B)
    {
        if (B)
        {
            myFlags |= theMask;
        }
        else
        {
            myFlags &= ~theMask;
        }
    }

    public bool Selected() => Flag(FMaskSelected);

    public void Selected(bool B) => Flag(FMaskSelected, B);

    public bool Back() => Flag(FMaskBack);

    public void Back(bool B) => Flag(FMaskBack, B);

    public bool Side() => Flag(FMaskSide);

    public void Side(bool B) => Flag(FMaskSide, B);

    public bool Closed() => Flag(FMaskClosed);

    public void Closed(bool B) => Flag(FMaskClosed, B);

    public bool Hiding() => Flag(FMaskHiding);

    public void Hiding(bool B) => Flag(FMaskHiding, B);

    public bool Simple() => Flag(FMaskSimple);

    public void Simple(bool B) => Flag(FMaskSimple, B);

    public bool Cut() => Flag(FMaskCut);

    public void Cut(bool B) => Flag(FMaskCut, B);

    public bool WithOutL() => Flag(FMaskWithOutL);

    public void WithOutL(bool B) => Flag(FMaskWithOutL, B);

    public bool Plane() => Flag(FMaskPlane);

    public void Plane(bool B) => Flag(FMaskPlane, B);

    public bool Cylinder() => Flag(FMaskCylinder);

    public void Cylinder(bool B) => Flag(FMaskCylinder, B);

    public bool Cone() => Flag(FMaskCone);

    public void Cone(bool B) => Flag(FMaskCone, B);

    public bool Sphere() => Flag(FMaskSphere);

    public void Sphere(bool B) => Flag(FMaskSphere, B);

    public bool Torus() => Flag(FMaskTorus);

    public void Torus(bool B) => Flag(FMaskTorus, B);

    public double Size() => mySize;

    public void Size(double S) => mySize = S;

    public TopAbs_Orientation Orientation() => ((TopAbs_Orientation)(myFlags & EMaskOrient));

    public void Orientation(TopAbs_Orientation O)
    {
        myFlags &= ~EMaskOrient;
        myFlags |= ((int)O & EMaskOrient);
    }

    /// <summary>The wires (a reference to the handle, as OCCT's).</summary>
    public ref HLRAlgo_WiresBlock? Wires() => ref myWires;

    public HLRBRep_Surface Geometry() => myGeometry;

    public float Tolerance() => myTolerance;
}
