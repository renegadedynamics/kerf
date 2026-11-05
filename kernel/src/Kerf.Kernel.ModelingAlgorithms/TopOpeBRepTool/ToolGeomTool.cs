// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_GeomTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_GeomTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Booleans.Legacy.Tools.OutCurveType;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// The settings of the curves the legacy Booleans compute from an intersection line: whether the 3D curve and the two
/// pcurves are computed, the kind of 3D curve (<see cref="OutCurveType"/>), the approximation
/// tolerances and the maximum number of points.
/// </summary>
/// <remarks>A value, as in OCCT. <c>new()</c> is OCCT's default (BSPLINE1, every curve computed); the setters are
/// internal.</remarks>
internal partial struct ToolGeomTool : IEquatable<ToolGeomTool>
{
    private OutCurveType _typeC3D;
    private bool _compC3D;
    private bool _compPC1;
    private bool _compPC2;
    private double _tol3d;
    private double _tol2d;
    private int _nbPntMax;

    /// <summary>OCCT's default settings.</summary>
    public ToolGeomTool() : this(OutCurveType.BSPLINE1)
    {
    }

    /// <summary>
    /// Settings that compute (or not) the 3D curve and the pcurves on the first and second surface, the 3D curve of a
    /// walking line being of kind <paramref name="typeC3D"/>.
    /// </summary>
    public ToolGeomTool(OutCurveType typeC3D,
                                   bool compC3D = true,
                                   bool compPC1 = true,
                                   bool compPC2 = true)
    {
        _typeC3D = typeC3D;
        _compC3D = compC3D;
        _compPC1 = compPC1;
        _compPC2 = compPC2;
        _tol3d = global::Kerf.Engine.Foundation.Precision.Approximation();
        _tol2d = global::Kerf.Engine.Foundation.Precision.PApproximation();
        _nbPntMax = 30;
    }

    internal void Define(OutCurveType typeC3D, bool compC3D, bool compPC1, bool compPC2)
    {
        _typeC3D = typeC3D;
        _compC3D = compC3D;
        _compPC1 = compPC1;
        _compPC2 = compPC2;
    }

    internal void Define(OutCurveType typeC3D)
    {
        _typeC3D = typeC3D;
    }

    internal void DefineCurves(bool compC3D)
    {
        _compC3D = compC3D;
    }

    internal void DefinePCurves1(bool compPC1)
    {
        _compPC1 = compPC1;
    }

    internal void DefinePCurves2(bool compPC2)
    {
        _compPC2 = compPC2;
    }

    internal void Define(in ToolGeomTool gt)
    {
        this = gt;
    }

    /// <summary>The 3D and 2D approximation tolerances.</summary>
    public readonly void GetTolerances(out double tol3d, out double tol2d)
    {
        tol3d = _tol3d;
        tol2d = _tol2d;
    }

    internal void SetTolerances(double tol3d, double tol2d)
    {
        _tol3d = tol3d;
        _tol2d = tol2d;
    }

    /// <summary>The maximum number of points of an approximation.</summary>
    public readonly int PntMaxCount => _nbPntMax;

    internal void SetNbPntMax(int nbPntMax)
    {
        _nbPntMax = nbPntMax;
    }

    /// <summary>The kind of 3D curve of a walking line.</summary>
    public readonly OutCurveType TypeC3D() => _typeC3D;

    /// <summary>Whether the 3D curve is computed.</summary>
    public readonly bool CompC3D() => _compC3D;

    /// <summary>Whether the pcurve on the first surface is computed.</summary>
    public readonly bool CompPC1() => _compPC1;

    /// <summary>Whether the pcurve on the second surface is computed.</summary>
    public readonly bool CompPC2() => _compPC2;

    // Kerf: value equality

    public readonly bool Equals(ToolGeomTool other)
        => _typeC3D == other._typeC3D && _compC3D == other._compC3D && _compPC1 == other._compPC1
           && _compPC2 == other._compPC2 && _tol3d.Equals(other._tol3d) && _tol2d.Equals(other._tol2d)
           && _nbPntMax == other._nbPntMax;

    public override readonly bool Equals(object? obj) => obj is ToolGeomTool gt && Equals(gt);

    public override readonly int GetHashCode()
        => HashCode.Combine(_typeC3D, _compC3D, _compPC1, _compPC2, _tol3d, _tol2d, _nbPntMax);

    public static bool operator ==(in ToolGeomTool left, in ToolGeomTool right)
        => left.Equals(right);

    public static bool operator !=(in ToolGeomTool left, in ToolGeomTool right)
        => !left.Equals(right);
}
