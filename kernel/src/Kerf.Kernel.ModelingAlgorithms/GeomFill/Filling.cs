// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Filling.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Filling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>The root of the fillings of a grid of poles from four rows of boundary poles (Coons, Curved, Stretch).</summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the poles. <see cref="Poles"/> and <see cref="Weights"/>
/// copy them into the caller's arrays. Before any <c>Init</c> the accessors throw (OCCT reads a null handle).
/// </remarks>
internal class Filling
{
    protected bool _isRational;
    protected NCollection_HArray2<Point3D>? _poles;
    protected NCollection_HArray2<double>? _weights;

    public Filling()
    {
        _isRational = false;
    }

    public int UPoleCount => _poles!.ColLength();

    public int VPoleCount => _poles!.RowLength();

    /// <summary>Copies the poles into <paramref name="poles"/> (OCCT's array assignment: resized to the grid).</summary>
    public void Poles(NCollection_Array2<Point3D> poles)
    {
        poles.Assign(_poles!.ChangeArray2());
    }

    public bool IsRational() => _isRational;

    /// <summary>Copies the weights into <paramref name="weights"/>; only for a rational filling.</summary>
    public void Weights(NCollection_Array2<double> weights)
    {
        weights.Assign(_weights!.ChangeArray2());
    }
}
