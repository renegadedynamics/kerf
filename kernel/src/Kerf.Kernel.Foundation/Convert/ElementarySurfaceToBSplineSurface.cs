// Port of: src/FoundationClasses/TKMath/Convert/Convert_ElementarySurfaceToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ElementarySurfaceToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Root class for the algorithms converting an elementary surface (gp_Cone, gp_Cylinder, gp_Sphere, gp_Torus) into
/// a B-spline surface: the degrees, periodicity, poles with weights, and knots with multiplicities in each
/// parametric direction, computed by the constructor of the derived class.
/// </summary>
/// <remarks>
/// The batch accessors (<see cref="Poles"/>, <see cref="Weights"/>, <see cref="UKnots"/>, ...) return copies, where
/// OCCT returns references to its fields: the converter's result can't be changed through them. The per-index
/// accessors (<see cref="Pole"/>, ...) are deprecated in OCCT in favour of the batch accessors.
/// </remarks>
internal class ElementarySurfaceToBSplineSurface
{
    protected NCollection_Array2<Point3D> _poles;
    protected NCollection_Array2<double> _weights;
    protected NCollection_Array1<double> _uKnots;
    protected NCollection_Array1<double> _vKnots;
    protected NCollection_Array1<int> _uMults;
    protected NCollection_Array1<int> _vMults;
    protected int _uDegree = 0;
    protected int _vDegree = 0;
    protected bool _isUPeriodic = false;
    protected bool _isVPeriodic = false;
    protected int _nbUPoles;
    protected int _nbVPoles;
    protected int _nbUKnots;
    protected int _nbVKnots;

    /// <summary>
    /// Trims the arrays, sized for the largest case, down to the actual numbers of poles and knots. OCCT's
    /// <c>Finalize</c>, renamed because <c>Finalize()</c> is .NET's finaliser.
    /// </summary>
    protected void FinalizeArrays()
    {
        // Trim oversized arrays down to actual sizes, preserving 2D element positions.
        if (_poles.NbRows() != _nbUPoles || _poles.NbColumns() != _nbVPoles)
        {
            _poles.ResizeWithTrim(1, _nbUPoles, 1, _nbVPoles, true);
        }
        if (_weights.NbRows() != _nbUPoles || _weights.NbColumns() != _nbVPoles)
        {
            _weights.ResizeWithTrim(1, _nbUPoles, 1, _nbVPoles, true);
        }
        if (_uKnots.Length() != _nbUKnots)
        {
            _uKnots.Resize(1, _nbUKnots, true);
        }
        if (_uMults.Length() != _nbUKnots)
        {
            _uMults.Resize(1, _nbUKnots, true);
        }
        if (_vKnots.Length() != _nbVKnots)
        {
            _vKnots.Resize(1, _nbVKnots, true);
        }
        if (_vMults.Length() != _nbVKnots)
        {
            _vMults.Resize(1, _nbVKnots, true);
        }
    }

    protected ElementarySurfaceToBSplineSurface(int nbUPoles,
                                                        int nbVPoles,
                                                        int nbUKnots,
                                                        int nbVKnots,
                                                        int uDegree,
                                                        int vDegree)
    {
        _poles = new NCollection_Array2<Point3D>(1, nbUPoles, 1, nbVPoles);
        _weights = new NCollection_Array2<double>(1, nbUPoles, 1, nbVPoles);
        _uKnots = new NCollection_Array1<double>(1, nbUKnots);
        _vKnots = new NCollection_Array1<double>(1, nbVKnots);
        _uMults = new NCollection_Array1<int>(1, nbUKnots);
        _vMults = new NCollection_Array1<int>(1, nbVKnots);
        _uDegree = uDegree;
        _vDegree = vDegree;
        _nbUPoles = nbUPoles;
        _nbVPoles = nbVPoles;
        _nbUKnots = nbUKnots;
        _nbVKnots = nbVKnots;
    }

    /// <summary>The degree in the U direction.</summary>
    public int UDegree()
    {
        return _uDegree;
    }

    /// <summary>The degree in the V direction.</summary>
    public int VDegree()
    {
        return _vDegree;
    }

    /// <summary>The number of poles in the U direction.</summary>
    public int UPoleCount
    {
        get
        {
            return _nbUPoles;
        }
    }

    /// <summary>The number of poles in the V direction.</summary>
    public int VPoleCount
    {
        get
        {
            return _nbVPoles;
        }
    }

    /// <summary>The number of knots in the U direction.</summary>
    public int UKnotCount
    {
        get
        {
            return _nbUKnots;
        }
    }

    /// <summary>The number of knots in the V direction.</summary>
    public int VKnotCount
    {
        get
        {
            return _nbVKnots;
        }
    }

    /// <summary>True if the surface is periodic in U.</summary>
    public bool IsUPeriodic
    {
        get
        {
            return _isUPeriodic;
        }
    }

    /// <summary>True if the surface is periodic in V.</summary>
    public bool IsVPeriodic
    {
        get
        {
            return _isVPeriodic;
        }
    }

    /// <summary>The pole (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 1. Deprecated in OCCT: use <see cref="Poles"/>.</summary>
    public Point3D Pole(int uIndex, int vIndex)
    {
        Standard_OutOfRange_Raise_if(
          uIndex < 1 || uIndex > _nbUPoles || vIndex < 1 || vIndex > _nbVPoles,
          "Convert_ElementarySurfaceToBSplineSurface::Pole: Index out of range");
        return _poles[uIndex, vIndex];
    }

    /// <summary>The weight of the pole (<paramref name="uIndex"/>, <paramref name="vIndex"/>). Deprecated in OCCT: use <see cref="Weights"/>.</summary>
    public double Weight(int uIndex, int vIndex)
    {
        Standard_OutOfRange_Raise_if(
          uIndex < 1 || uIndex > _nbUPoles || vIndex < 1 || vIndex > _nbVPoles,
          "Convert_ElementarySurfaceToBSplineSurface::Weight: Index out of range");
        return _weights[uIndex, vIndex];
    }

    /// <summary>The U knot of index <paramref name="uIndex"/>. Deprecated in OCCT: use <see cref="UKnots"/>.</summary>
    public double UKnot(int uIndex)
    {
        Standard_OutOfRange_Raise_if(
          uIndex < 1 || uIndex > _nbUKnots,
          "Convert_ElementarySurfaceToBSplineSurface::UKnot: Index out of range");
        return _uKnots[uIndex];
    }

    /// <summary>The V knot of index <paramref name="vIndex"/>. Deprecated in OCCT: use <see cref="VKnots"/>.</summary>
    public double VKnot(int vIndex)
    {
        Standard_OutOfRange_Raise_if(
          vIndex < 1 || vIndex > _nbVKnots,
          "Convert_ElementarySurfaceToBSplineSurface::VKnot: Index out of range");
        return _vKnots[vIndex];
    }

    /// <summary>The multiplicity of the U knot of index <paramref name="uIndex"/>. Deprecated in OCCT: use <see cref="UMultiplicities"/>.</summary>
    public int UMultiplicity(int uIndex)
    {
        Standard_OutOfRange_Raise_if(
          uIndex < 1 || uIndex > _nbUKnots,
          "Convert_ElementarySurfaceToBSplineSurface::UMultiplicity: Index out of range");
        return _uMults[uIndex];
    }

    /// <summary>The multiplicity of the V knot of index <paramref name="vIndex"/>. Deprecated in OCCT: use <see cref="VMultiplicities"/>.</summary>
    public int VMultiplicity(int vIndex)
    {
        Standard_OutOfRange_Raise_if(
          vIndex < 1 || vIndex > _nbVKnots,
          "Convert_ElementarySurfaceToBSplineSurface::VMultiplicity: Index out of range");
        return _vMults[vIndex];
    }

    /// <summary>The poles: a copy.</summary>
    public NCollection_Array2<Point3D> Poles()
    {
        return new NCollection_Array2<Point3D>(_poles);
    }

    /// <summary>The weights: a copy.</summary>
    public NCollection_Array2<double> Weights()
    {
        return new NCollection_Array2<double>(_weights);
    }

    /// <summary>The U knots: a copy.</summary>
    public NCollection_Array1<double> UKnots()
    {
        return new NCollection_Array1<double>(_uKnots);
    }

    /// <summary>The V knots: a copy.</summary>
    public NCollection_Array1<double> VKnots()
    {
        return new NCollection_Array1<double>(_vKnots);
    }

    /// <summary>The multiplicities of the U knots: a copy.</summary>
    public NCollection_Array1<int> UMultiplicities()
    {
        return new NCollection_Array1<int>(_uMults);
    }

    /// <summary>The multiplicities of the V knots: a copy.</summary>
    public NCollection_Array1<int> VMultiplicities()
    {
        return new NCollection_Array1<int>(_vMults);
    }
}
