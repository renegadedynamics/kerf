// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// A free bound and its properties, as <see cref="FreeBoundsPropertiesAnalysis"/> computes them: the area and
/// perimeter of the contour, the ratio of its average length to its average width, its average width, and its notches
/// (narrow V-like sub-contours) with their maximum widths. Unset properties are -1.
/// </summary>
/// <remarks>A working object with OCCT's setters (decision 21). The notches are keyed by <see cref="ShapeHasher"/>.</remarks>
public class FreeBoundDataAnalysis : Transient
{
    private Wire _bound = new();
    private double _area;
    private double _perimeter;
    private double _ratio;
    private double _width;
    private readonly NCollection_HSequence<Shape> _notches;
    private readonly NCollection_DataMap<Shape, double> _notchesParams = new(ShapeHasher.Instance);

    /// <summary>An empty data set.</summary>
    public FreeBoundDataAnalysis()
    {
        _notches = new NCollection_HSequence<Shape>();
        Clear();
    }

    /// <summary>The data of the contour.</summary>
    public FreeBoundDataAnalysis(in Wire freebound)
    {
        _notches = new NCollection_HSequence<Shape>();
        Clear();
        SetFreeBound(freebound);
    }

    /// <summary>Clears the properties of the contour (not the contour).</summary>
    public void Clear()
    {
        _area = -1;
        _perimeter = -1;
        _ratio = -1;
        _width = -1;
        _notches.Clear();
        _notchesParams.Clear();
    }

    /// <summary>Sets the contour.</summary>
    public void SetFreeBound(in Wire freebound) => _bound = freebound;

    /// <summary>Sets the area of the contour.</summary>
    public void SetArea(double area) => _area = area;

    /// <summary>Sets the perimeter of the contour.</summary>
    public void SetPerimeter(double perimeter) => _perimeter = perimeter;

    /// <summary>Sets the ratio of the average length to the average width of the contour.</summary>
    public void SetRatio(double ratio) => _ratio = ratio;

    /// <summary>Sets the average width of the contour.</summary>
    public void SetWidth(double width) => _width = width;

    /// <summary>Adds a notch of the contour with its maximum width (ignored if already added).</summary>
    public void AddNotch(in Wire notch, double width)
    {
        if (_notchesParams.IsBound(notch))
        {
            return;
        }
        _notches.Append(notch);
        _notchesParams.Bind(notch, width);
    }

    /// <summary>The contour.</summary>
    public Wire FreeBound() => _bound;

    /// <summary>The area of the contour.</summary>
    public double Area() => _area;

    /// <summary>The perimeter of the contour.</summary>
    public double Perimeter() => _perimeter;

    /// <summary>The ratio of the average length to the average width of the contour.</summary>
    public double Ratio() => _ratio;

    /// <summary>The average width of the contour.</summary>
    public double Width() => _width;

    /// <summary>The number of notches.</summary>
    public int NotchCount => _notches.Length();

    /// <summary>The notches (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<Shape> Notches() => _notches;

    /// <summary>The notch of the given rank.</summary>
    public Wire Notch(int index) => ShapeCasts.Wire(_notches.Value(index));

    /// <summary>The maximum width of the notch of the given rank.</summary>
    public double NotchWidth(int index)
    {
        Wire wire = ShapeCasts.Wire(_notches.Value(index));
        return _notchesParams.Find(wire);
    }

    /// <summary>The maximum width of the notch.</summary>
    public double NotchWidth(in Wire notch) => _notchesParams.Find(notch);
}
