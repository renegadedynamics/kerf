// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>An edge of the contour seen by MAT2d_Mat2d's algorithm: its number, its two bisectors, the distance
/// and index of the intersection point of its bisectors.</summary>
/// <remarks>A working object of MAT2d_Mat2d (decision 21): the setters are internal. OCCT's Dump does nothing and is
/// left out.</remarks>
internal sealed class MedialEdge : Transient
{
    private int _theedgenumber;
    private MedialBisector? _thefirstbisector;
    private MedialBisector? _thesecondbisector;
    private double _thedistance;
    private int _theintersectionpoint;

    public MedialEdge()
    {
        _theedgenumber = 0;
        _thedistance = 0.0;
        _theintersectionpoint = 0;
    }

    internal void EdgeNumber(int anumber) => _theedgenumber = anumber;

    internal void FirstBisector(MedialBisector? abisector) => _thefirstbisector = abisector;

    internal void SecondBisector(MedialBisector? abisector) => _thesecondbisector = abisector;

    internal void Distance(double adistance) => _thedistance = adistance;

    internal void IntersectionPoint(int apoint) => _theintersectionpoint = apoint;

    public int EdgeNumber() => _theedgenumber;

    public MedialBisector FirstBisector() => _thefirstbisector!;

    public MedialBisector SecondBisector() => _thesecondbisector!;

    public double Distance() => _thedistance;

    public int IntersectionPoint() => _theintersectionpoint;
}
