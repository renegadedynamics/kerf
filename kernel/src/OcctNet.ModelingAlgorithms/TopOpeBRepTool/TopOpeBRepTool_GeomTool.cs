// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_GeomTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_GeomTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepTool_OutCurveType;

namespace Occt;

/// <summary>
/// The settings of the curves the legacy Booleans compute from an intersection line: whether the 3D curve and the two
/// pcurves are computed, the kind of 3D curve (<see cref="TopOpeBRepTool_OutCurveType"/>), the approximation
/// tolerances and the maximum number of points.
/// </summary>
/// <remarks>A value, as in OCCT. <c>new()</c> is OCCT's default (BSPLINE1, every curve computed); the setters are
/// internal.</remarks>
public partial struct TopOpeBRepTool_GeomTool : IEquatable<TopOpeBRepTool_GeomTool>
{
    private TopOpeBRepTool_OutCurveType myTypeC3D;
    private bool myCompC3D;
    private bool myCompPC1;
    private bool myCompPC2;
    private double myTol3d;
    private double myTol2d;
    private int myNbPntMax;

    /// <summary>OCCT's default settings.</summary>
    public TopOpeBRepTool_GeomTool() : this(TopOpeBRepTool_BSPLINE1)
    {
    }

    /// <summary>
    /// Settings that compute (or not) the 3D curve and the pcurves on the first and second surface, the 3D curve of a
    /// walking line being of kind <paramref name="TypeC3D"/>.
    /// </summary>
    public TopOpeBRepTool_GeomTool(TopOpeBRepTool_OutCurveType TypeC3D,
                                   bool CompC3D = true,
                                   bool CompPC1 = true,
                                   bool CompPC2 = true)
    {
        myTypeC3D = TypeC3D;
        myCompC3D = CompC3D;
        myCompPC1 = CompPC1;
        myCompPC2 = CompPC2;
        myTol3d = Precision.Approximation();
        myTol2d = Precision.PApproximation();
        myNbPntMax = 30;
    }

    internal void Define(TopOpeBRepTool_OutCurveType TypeC3D, bool CompC3D, bool CompPC1, bool CompPC2)
    {
        myTypeC3D = TypeC3D;
        myCompC3D = CompC3D;
        myCompPC1 = CompPC1;
        myCompPC2 = CompPC2;
    }

    internal void Define(TopOpeBRepTool_OutCurveType TypeC3D)
    {
        myTypeC3D = TypeC3D;
    }

    internal void DefineCurves(bool CompC3D)
    {
        myCompC3D = CompC3D;
    }

    internal void DefinePCurves1(bool CompPC1)
    {
        myCompPC1 = CompPC1;
    }

    internal void DefinePCurves2(bool CompPC2)
    {
        myCompPC2 = CompPC2;
    }

    internal void Define(in TopOpeBRepTool_GeomTool GT)
    {
        this = GT;
    }

    /// <summary>The 3D and 2D approximation tolerances.</summary>
    public readonly void GetTolerances(out double tol3d, out double tol2d)
    {
        tol3d = myTol3d;
        tol2d = myTol2d;
    }

    internal void SetTolerances(double tol3d, double tol2d)
    {
        myTol3d = tol3d;
        myTol2d = tol2d;
    }

    /// <summary>The maximum number of points of an approximation.</summary>
    public readonly int NbPntMax() => myNbPntMax;

    internal void SetNbPntMax(int NbPntMax)
    {
        myNbPntMax = NbPntMax;
    }

    /// <summary>The kind of 3D curve of a walking line.</summary>
    public readonly TopOpeBRepTool_OutCurveType TypeC3D() => myTypeC3D;

    /// <summary>Whether the 3D curve is computed.</summary>
    public readonly bool CompC3D() => myCompC3D;

    /// <summary>Whether the pcurve on the first surface is computed.</summary>
    public readonly bool CompPC1() => myCompPC1;

    /// <summary>Whether the pcurve on the second surface is computed.</summary>
    public readonly bool CompPC2() => myCompPC2;

    // OCCT.NET: value equality

    public readonly bool Equals(TopOpeBRepTool_GeomTool other)
        => myTypeC3D == other.myTypeC3D && myCompC3D == other.myCompC3D && myCompPC1 == other.myCompPC1
           && myCompPC2 == other.myCompPC2 && myTol3d.Equals(other.myTol3d) && myTol2d.Equals(other.myTol2d)
           && myNbPntMax == other.myNbPntMax;

    public override readonly bool Equals(object? obj) => obj is TopOpeBRepTool_GeomTool aGT && Equals(aGT);

    public override readonly int GetHashCode()
        => HashCode.Combine(myTypeC3D, myCompC3D, myCompPC1, myCompPC2, myTol3d, myTol2d, myNbPntMax);

    public static bool operator ==(in TopOpeBRepTool_GeomTool theLeft, in TopOpeBRepTool_GeomTool theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepTool_GeomTool theLeft, in TopOpeBRepTool_GeomTool theRight)
        => !theLeft.Equals(theRight);
}
