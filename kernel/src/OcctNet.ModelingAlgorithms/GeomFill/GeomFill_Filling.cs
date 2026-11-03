// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Filling.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Filling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The root of the fillings of a grid of poles from four rows of boundary poles (Coons, Curved, Stretch).</summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the poles. <see cref="Poles"/> and <see cref="Weights"/>
/// copy them into the caller's arrays. Before any <c>Init</c> the accessors throw (OCCT reads a null handle).
/// </remarks>
public class GeomFill_Filling
{
    protected bool IsRational;
    protected NCollection_HArray2<Point3D>? myPoles;
    protected NCollection_HArray2<double>? myWeights;

    public GeomFill_Filling()
    {
        IsRational = false;
    }

    public int NbUPoles() => myPoles!.ColLength();

    public int NbVPoles() => myPoles!.RowLength();

    /// <summary>Copies the poles into <paramref name="Poles"/> (OCCT's array assignment: resized to the grid).</summary>
    public void Poles(NCollection_Array2<Point3D> Poles)
    {
        Poles.Assign(myPoles!.ChangeArray2());
    }

    public bool isRational() => IsRational;

    /// <summary>Copies the weights into <paramref name="Weights"/>; only for a rational filling.</summary>
    public void Weights(NCollection_Array2<double> Weights)
    {
        Weights.Assign(myWeights!.ChangeArray2());
    }
}
