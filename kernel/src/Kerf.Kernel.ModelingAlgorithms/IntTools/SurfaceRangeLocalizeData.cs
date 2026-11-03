// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The localisation data of a surface for the edge/face intersector: the numbers of samples and minimal ranges in U
/// and V, the sample ranges found out of the edge, their boxes, and a grid of points with its frame.
/// </summary>
/// <remarks>
/// <para>
/// A class, a working object (IntTools_Context caches one per face; plan section 6), because it holds maps and
/// arrays. OCCT's copy constructor and <see cref="Assign"/> copy the maps but share the grid arrays (handles), as here.
/// As in OCCT, <see cref="Assign"/> keeps this object's grid when the other has none.
/// </para>
/// <para>
/// <see cref="ListRangeOut"/> lists the out ranges in insertion order, where OCCT lists them in its hash order
/// (decision 629).
/// </para>
/// </remarks>
public sealed class SurfaceRangeLocalizeData
{
    private int _nbSampleU;
    private int _nbSampleV;
    private double _minRangeU;
    private double _minRangeV;
    private HashSet<SurfaceRangeSample> _mapRangeOut = new();
    private Dictionary<SurfaceRangeSample, Box> _mapBox = new();
    private NCollection_HArray1<double>? _uParams;
    private NCollection_HArray1<double>? _vParams;
    private NCollection_HArray2<Point3D>? _gridPoints;
    private int _uIndMin;
    private int _uIndMax;
    private int _vIndMin;
    private int _vIndMax;
    private double _deflection;

    public SurfaceRangeLocalizeData()
    {
        _nbSampleU = 1;
        _nbSampleV = 1;
        _minRangeU = 0.0;
        _minRangeV = 0.0;
        _deflection = 0.0;
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;
    }

    public SurfaceRangeLocalizeData(int nbSampleU, int nbSampleV, double minRangeU, double minRangeV)
    {
        _nbSampleU = nbSampleU;
        _nbSampleV = nbSampleV;
        _minRangeU = minRangeU;
        _minRangeV = minRangeV;
        _deflection = 0.0;
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;
    }

    /// <summary>A copy (OCCT's copy constructor): the maps copied, the grid arrays shared.</summary>
    public SurfaceRangeLocalizeData(SurfaceRangeLocalizeData other)
    {
        Assign(other);
    }

    /// <summary>OCCT's assignment: the maps copied, the grid arrays shared (kept if the other has none).</summary>
    internal SurfaceRangeLocalizeData Assign(SurfaceRangeLocalizeData other)
    {
        _nbSampleU = other._nbSampleU;
        _nbSampleV = other._nbSampleV;
        _minRangeU = other._minRangeU;
        _minRangeV = other._minRangeV;
        _mapRangeOut.Clear();
        _mapRangeOut = new HashSet<SurfaceRangeSample>(other._mapRangeOut,other._mapRangeOut.Comparer);
        _mapBox.Clear();
        _mapBox = new Dictionary<SurfaceRangeSample, Box>(other._mapBox,other._mapBox.Comparer);
        _deflection = other._deflection;
        _uIndMin = other._uIndMin;
        _uIndMax = other._uIndMax;
        _vIndMin = other._vIndMin;
        _vIndMax = other._vIndMax;

        if (other._uParams is not null)
        {
            _uParams = other._uParams;
        }

        if (other._vParams is not null)
        {
            _vParams = other._vParams;
        }

        if (other._gridPoints is not null)
        {
            _gridPoints = other._gridPoints;
        }
        return this;
    }

    public int GetNbSampleU() => _nbSampleU;

    public int GetNbSampleV() => _nbSampleV;

    public double GetMinRangeU() => _minRangeU;

    public double GetMinRangeV() => _minRangeV;

    /// <summary>Records the range as out (its box is kept, as in OCCT).</summary>
    public void AddOutRange(in SurfaceRangeSample range)
    {
        _mapRangeOut.Add(range);
        //   myMapBox.UnBind(theRange);
    }

    public void AddBox(in SurfaceRangeSample range, in Box box) => _mapBox[range] = box;

    /// <summary>The box of the range, and true, if recorded; otherwise false, the box left as it was.</summary>
    public bool FindBox(in SurfaceRangeSample range, ref Box box)
    {
        if (_mapBox.TryGetValue(range, out Box aBox))
        {
            box = aBox;
            return true;
        }
        return false;
    }

    public bool IsRangeOut(in SurfaceRangeSample range) => _mapRangeOut.Contains(range);

    /// <summary>Appends the out ranges to the list.</summary>
    public void ListRangeOut(NCollection_List<SurfaceRangeSample> list)
    {
        foreach (SurfaceRangeSample range in _mapRangeOut)
        {
            list.Append(range);
        }
    }

    public void RemoveRangeOutAll() => _mapRangeOut.Clear();

    /// <summary>Sets the grid deflection.</summary>
    public void SetGridDeflection(double deflection) => _deflection = deflection;

    /// <summary>The grid deflection.</summary>
    public double GetGridDeflection() => _deflection;

    /// <summary>Sets the number of U parameters of the grid (resets the frame; reallocates if it changes).</summary>
    public void SetRangeUGrid(int nbUGrid)
    {
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;

        if (_uParams is null || nbUGrid != _uParams.Length())
        {
            _uParams = new NCollection_HArray1<double>(1, nbUGrid);

            if (_vParams is not null)
            {
                _gridPoints = NewGrid(nbUGrid, _vParams.Length());
            }
        }
    }

    /// <summary>The number of U parameters of the grid.</summary>
    public int GetRangeUGrid() => _uParams is null ? 0 : _uParams.Length();

    public void SetUParam(int index, double uParam) => _uParams!.SetValue(index, uParam);

    public double GetUParam(int index) => _uParams!.Value(index);

    /// <summary>Sets the number of V parameters of the grid (resets the frame; reallocates if it changes).</summary>
    public void SetRangeVGrid(int nbVGrid)
    {
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;

        if (_vParams is null || nbVGrid != _vParams.Length())
        {
            _vParams = new NCollection_HArray1<double>(1, nbVGrid);

            if (_uParams is not null)
            {
                _gridPoints = NewGrid(_uParams.Length(), nbVGrid);
            }
        }
    }

    /// <summary>The number of V parameters of the grid.</summary>
    public int GetRangeVGrid() => _vParams is null ? 0 : _vParams.Length();

    public void SetVParam(int index, double vParam) => _vParams!.SetValue(index, vParam);

    public double GetVParam(int index) => _vParams!.Value(index);

    public void SetGridPoint(int uIndex, int vIndex, in Point3D point) =>
        _gridPoints!.SetValue(uIndex, vIndex, point);

    public Point3D GetGridPoint(int uIndex, int vIndex) => _gridPoints!.Value(uIndex, vIndex);

    /// <summary>
    /// Sets the frame: the grid indices strictly inside [<paramref name="uMin"/>, <paramref name="uMax"/>] ×
    /// [<paramref name="vMin"/>, <paramref name="vMax"/>].
    /// </summary>
    public void SetFrame(double uMin, double uMax, double vMin, double vMax)
    {
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;

        if (_uParams is null || _vParams is null)
        {
            return;
        }

        int i;
        int lmI;
        int len = _uParams.Length();

        // Compute frame along U.
        for (i = 1; i <= len; i++)
        {
            if (_uIndMin == 0 && uMin < _uParams.Value(i))
            {
                _uIndMin = i;
            }

            lmI = len - i + 1;

            if (_uIndMax == 0 && uMax > _uParams.Value(lmI))
            {
                _uIndMax = lmI;
            }
        }

        if (_uIndMin == 0)
        {
            _uIndMin = len + 1;
        }

        // Compute frame along V.
        len = _vParams.Length();

        for (i = 1; i <= len; i++)
        {
            if (_vIndMin == 0 && vMin < _vParams.Value(i))
            {
                _vIndMin = i;
            }

            lmI = len - i + 1;

            if (_vIndMax == 0 && vMax > _vParams.Value(lmI))
            {
                _vIndMax = lmI;
            }
        }

        if (_vIndMin == 0)
        {
            _vIndMin = len + 1;
        }
    }

    /// <summary>The number of grid points along U in the frame.</summary>
    public int GetNBUPointsInFrame() => _uIndMax - _uIndMin + 1;

    /// <summary>The number of grid points along V in the frame.</summary>
    public int GetNBVPointsInFrame() => _vIndMax - _vIndMin + 1;

    /// <summary>The grid point at the indices in the frame; the origin if outside.</summary>
    public Point3D GetPointInFrame(int uIndex, int vIndex)
    {
        int frmUInd = uIndex + _uIndMin - 1;
        int frmVInd = vIndex + _vIndMin - 1;

        if (_gridPoints is null || frmUInd > _uIndMax || frmVInd > _vIndMax)
        {
            return Constants.Origin();
        }

        return _gridPoints.Value(frmUInd, frmVInd);
    }

    /// <summary>The U parameter at the index in the frame; <see cref="Precision.Infinite"/> if outside.</summary>
    public double GetUParamInFrame(int index)
    {
        int frmInd = index + _uIndMin - 1;

        if (_uParams is null || frmInd > _uIndMax)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        return _uParams.Value(frmInd);
    }

    /// <summary>The V parameter at the index in the frame; <see cref="Precision.Infinite"/> if outside.</summary>
    public double GetVParamInFrame(int index)
    {
        int frmInd = index + _vIndMin - 1;

        if (_vParams is null || frmInd > _vIndMax)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        return _vParams.Value(frmInd);
    }

    /// <summary>Clears the grid of points.</summary>
    public void ClearGrid()
    {
        _deflection = 0.0;
        _uIndMin = 0;
        _uIndMax = 0;
        _vIndMin = 0;
        _vIndMax = 0;

        _uParams = null;
        _vParams = null;
        _gridPoints = null;
    }

    // OCCT's NCollection_HArray2<gp_Pnt>(1, NbU, 1, NbV): default-constructed points.
    private static NCollection_HArray2<Point3D> NewGrid(int nbU, int nbV) => new(1, nbU, 1, nbV);
}
