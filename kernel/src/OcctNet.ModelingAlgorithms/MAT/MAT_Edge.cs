// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>An edge of the contour seen by MAT2d_Mat2d's algorithm: its number, its two bisectors, the distance
/// and index of the intersection point of its bisectors.</summary>
/// <remarks>A working object of MAT2d_Mat2d (decision 21): the setters are internal. OCCT's Dump does nothing and is
/// left out.</remarks>
public sealed class MAT_Edge : Transient
{
    private int theedgenumber;
    private MAT_Bisector? thefirstbisector;
    private MAT_Bisector? thesecondbisector;
    private double thedistance;
    private int theintersectionpoint;

    public MAT_Edge()
    {
        theedgenumber = 0;
        thedistance = 0.0;
        theintersectionpoint = 0;
    }

    internal void EdgeNumber(int anumber) => theedgenumber = anumber;

    internal void FirstBisector(MAT_Bisector? abisector) => thefirstbisector = abisector;

    internal void SecondBisector(MAT_Bisector? abisector) => thesecondbisector = abisector;

    internal void Distance(double adistance) => thedistance = adistance;

    internal void IntersectionPoint(int apoint) => theintersectionpoint = apoint;

    public int EdgeNumber() => theedgenumber;

    public MAT_Bisector FirstBisector() => thefirstbisector!;

    public MAT_Bisector SecondBisector() => thesecondbisector!;

    public double Distance() => thedistance;

    public int IntersectionPoint() => theintersectionpoint;
}
