// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology.Brep;

using Kerf.Topology;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The hidden-line data of a face: its orientation and flags (selected, back, side, closed, hiding, simple, cut, with
/// outlines, and its kind of quadric), its size, its wires of edge indices, and its projected surface.
/// </summary>
/// <remarks>
/// OCCT's value class, ported as a class (decision 1092), as <see cref="HiddenLineEdgeData"/>. OCCT's copy assignment
/// is <see cref="Assign"/>: the wires block is a handle the copies share (so <c>HLRBRep_Data::Write</c>'s renumbering
/// of a copy's edges renumbers the source's too, as in OCCT) and the surface is copied. A working object.
/// </remarks>
internal sealed class HiddenLineFaceData
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

    private int _flags;
    private WiresBlock? _wires;
    private HlrSurface _geometry = new();
    private double _size;
    private float _tolerance;

    public HiddenLineFaceData()
    {
        _flags = 0;
        _size = 0;
        Selected(true);
    }

    /// <summary>OCCT's copy assignment: the wires shared, the surface copied.</summary>
    public HiddenLineFaceData Assign(HiddenLineFaceData other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }
        _flags = other._flags;
        _wires = other._wires;
        _geometry = other._geometry.Copy();
        _size = other._size;
        _tolerance = other._tolerance;
        return this;
    }

    /// <summary>Sets the face, its orientation, whether it is closed and its number of wires.</summary>
    public void Set(in Face fg, Orientation or, bool cl, int nw)
    {
        Closed(cl);
        Geometry().Surface(fg);
        _tolerance = (float)(Tool.Tolerance(fg));
        Orientation(or);
        Wires() = new WiresBlock(nw);
    }

    /// <summary>Sets the number of edges of the wire WI.</summary>
    public void SetWire(int wi, int ne)
    {
        Wires()!.Set(wi, new EdgesBlock(ne));
    }

    /// <summary>Sets the edge EWI of the wire WI: its index, orientation and flags.</summary>
    public void SetWEdge(int wi, int ewi, int ei, Orientation or, bool outL, bool inte, bool dble, bool isoL)
    {
        Wires()!.Wire(wi)!.Edge(ewi, ei);
        Wires()!.Wire(wi)!.Orientation(ewi, or);
        Wires()!.Wire(wi)!.OutLine(ewi, outL);
        Wires()!.Wire(wi)!.Internal(ewi, inte);
        Wires()!.Wire(wi)!.Double(ewi, dble);
        Wires()!.Wire(wi)!.IsoLine(ewi, isoL);
    }

    private bool Flag(int mask) => (_flags & mask) != 0;

    private void Flag(int mask, bool b)
    {
        if (b)
        {
            _flags |= mask;
        }
        else
        {
            _flags &= ~mask;
        }
    }

    public bool Selected() => Flag(FMaskSelected);

    public void Selected(bool b) => Flag(FMaskSelected, b);

    public bool Back() => Flag(FMaskBack);

    public void Back(bool b) => Flag(FMaskBack, b);

    public bool Side() => Flag(FMaskSide);

    public void Side(bool b) => Flag(FMaskSide, b);

    public bool Closed() => Flag(FMaskClosed);

    public void Closed(bool b) => Flag(FMaskClosed, b);

    public bool Hiding() => Flag(FMaskHiding);

    public void Hiding(bool b) => Flag(FMaskHiding, b);

    public bool Simple() => Flag(FMaskSimple);

    public void Simple(bool b) => Flag(FMaskSimple, b);

    public bool Cut() => Flag(FMaskCut);

    public void Cut(bool b) => Flag(FMaskCut, b);

    public bool WithOutL() => Flag(FMaskWithOutL);

    public void WithOutL(bool b) => Flag(FMaskWithOutL, b);

    public bool Plane() => Flag(FMaskPlane);

    public void Plane(bool b) => Flag(FMaskPlane, b);

    public bool Cylinder() => Flag(FMaskCylinder);

    public void Cylinder(bool b) => Flag(FMaskCylinder, b);

    public bool Cone() => Flag(FMaskCone);

    public void Cone(bool b) => Flag(FMaskCone, b);

    public bool Sphere() => Flag(FMaskSphere);

    public void Sphere(bool b) => Flag(FMaskSphere, b);

    public bool Torus() => Flag(FMaskTorus);

    public void Torus(bool b) => Flag(FMaskTorus, b);

    public double Size() => _size;

    public void Size(double s) => _size = s;

    public Orientation Orientation() => ((Orientation)(_flags & EMaskOrient));

    public void Orientation(Orientation o)
    {
        _flags &= ~EMaskOrient;
        _flags |= ((int)o & EMaskOrient);
    }

    /// <summary>The wires (a reference to the handle, as OCCT's).</summary>
    public ref WiresBlock? Wires() => ref _wires;

    public HlrSurface Geometry() => _geometry;

    public float Tolerance() => _tolerance;
}
