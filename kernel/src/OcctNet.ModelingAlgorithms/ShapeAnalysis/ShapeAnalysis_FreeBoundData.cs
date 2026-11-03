// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A free bound and its properties, as <see cref="ShapeAnalysis_FreeBoundsProperties"/> computes them: the area and
/// perimeter of the contour, the ratio of its average length to its average width, its average width, and its notches
/// (narrow V-like sub-contours) with their maximum widths. Unset properties are -1.
/// </summary>
/// <remarks>A working object with OCCT's setters (decision 21). The notches are keyed by <see cref="TopTools_ShapeMapHasher"/>.</remarks>
public class ShapeAnalysis_FreeBoundData : Transient
{
    private TopoDS_Wire myBound = new();
    private double myArea;
    private double myPerimeter;
    private double myRatio;
    private double myWidth;
    private readonly NCollection_HSequence<TopoDS_Shape> myNotches;
    private readonly NCollection_DataMap<TopoDS_Shape, double> myNotchesParams = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty data set.</summary>
    public ShapeAnalysis_FreeBoundData()
    {
        myNotches = new NCollection_HSequence<TopoDS_Shape>();
        Clear();
    }

    /// <summary>The data of the contour.</summary>
    public ShapeAnalysis_FreeBoundData(in TopoDS_Wire freebound)
    {
        myNotches = new NCollection_HSequence<TopoDS_Shape>();
        Clear();
        SetFreeBound(freebound);
    }

    /// <summary>Clears the properties of the contour (not the contour).</summary>
    public void Clear()
    {
        myArea = -1;
        myPerimeter = -1;
        myRatio = -1;
        myWidth = -1;
        myNotches.Clear();
        myNotchesParams.Clear();
    }

    /// <summary>Sets the contour.</summary>
    public void SetFreeBound(in TopoDS_Wire freebound) => myBound = freebound;

    /// <summary>Sets the area of the contour.</summary>
    public void SetArea(double area) => myArea = area;

    /// <summary>Sets the perimeter of the contour.</summary>
    public void SetPerimeter(double perimeter) => myPerimeter = perimeter;

    /// <summary>Sets the ratio of the average length to the average width of the contour.</summary>
    public void SetRatio(double ratio) => myRatio = ratio;

    /// <summary>Sets the average width of the contour.</summary>
    public void SetWidth(double width) => myWidth = width;

    /// <summary>Adds a notch of the contour with its maximum width (ignored if already added).</summary>
    public void AddNotch(in TopoDS_Wire notch, double width)
    {
        if (myNotchesParams.IsBound(notch))
        {
            return;
        }
        myNotches.Append(notch);
        myNotchesParams.Bind(notch, width);
    }

    /// <summary>The contour.</summary>
    public TopoDS_Wire FreeBound() => myBound;

    /// <summary>The area of the contour.</summary>
    public double Area() => myArea;

    /// <summary>The perimeter of the contour.</summary>
    public double Perimeter() => myPerimeter;

    /// <summary>The ratio of the average length to the average width of the contour.</summary>
    public double Ratio() => myRatio;

    /// <summary>The average width of the contour.</summary>
    public double Width() => myWidth;

    /// <summary>The number of notches.</summary>
    public int NbNotches() => myNotches.Length();

    /// <summary>The notches (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<TopoDS_Shape> Notches() => myNotches;

    /// <summary>The notch of the given rank.</summary>
    public TopoDS_Wire Notch(int index) => TopoDS.Wire(myNotches.Value(index));

    /// <summary>The maximum width of the notch of the given rank.</summary>
    public double NotchWidth(int index)
    {
        TopoDS_Wire wire = TopoDS.Wire(myNotches.Value(index));
        return myNotchesParams.Find(wire);
    }

    /// <summary>The maximum width of the notch.</summary>
    public double NotchWidth(in TopoDS_Wire notch) => myNotchesParams.Find(notch);
}
