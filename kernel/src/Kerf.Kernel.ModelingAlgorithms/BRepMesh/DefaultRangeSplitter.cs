// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DefaultRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DefaultRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The default tool defining the range of a discrete face and the grid points distributed in it. The base of the
/// range splitters of each surface type.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. OCCT's <c>std::pair&lt;double, double&gt;</c> are value tuples with the fields
/// <c>first</c> and <c>second</c>. The face algorithms of unit 4D are generics over the splitter
/// (<c>where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()</c>), so the methods the splitters redefine are
/// virtual, as in OCCT; <see cref="Scale"/> and <see cref="Point"/> aren't, in OCCT either.
/// </remarks>
internal class DefaultRangeSplitter
{
    protected FaceModel? _dFace;
    protected (double first, double second) _rangeU;
    protected (double first, double second) _rangeV;
    protected (double first, double second) _delta;
    protected (double first, double second) _tolerance;
    protected bool _isValid;

    /// <summary>A splitter; <see cref="Reset"/> must be called before its first use.</summary>
    public DefaultRangeSplitter()
    {
        _isValid = true;
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public virtual void Reset(FaceModel dFace, in ToolParameters parameters)
    {
        _dFace = dFace;
        _rangeU.first = _rangeV.first = 1.0e100;
        _rangeU.second = _rangeV.second = -1.0e100;
        _delta.first = _delta.second = 1.0;
        _tolerance.first = _tolerance.second = global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Registers a border point.</summary>
    public virtual void AddPoint(in Point2D point)
    {
        _rangeU.first = Min(point.X, _rangeU.first);
        _rangeU.second = Max(point.X, _rangeU.second);
        _rangeV.first = Min(point.Y, _rangeV.first);
        _rangeV.second = Max(point.Y, _rangeV.second);
    }

    /// <summary>Updates the discrete range of the surface from its geometric range.</summary>
    public virtual void AdjustRange()
    {
        FaceSurfaceAdaptor surface = GetSurface();
        UpdateRange(surface.FirstUParameter(),
                    surface.LastUParameter(),
                    surface.IsUPeriodic,
                    ref _rangeU.first,
                    ref _rangeU.second);

        if (_rangeU.second < _rangeU.first)
        {
            _isValid = false;
            return;
        }

        UpdateRange(surface.FirstVParameter(),
                    surface.LastVParameter(),
                    surface.IsVPeriodic,
                    ref _rangeV.first,
                    ref _rangeV.second);

        if (_rangeV.second < _rangeV.first)
        {
            _isValid = false;
            return;
        }

        double lengthU = ComputeLengthU();
        double lengthV = ComputeLengthV();
        _isValid = lengthU > global::Kerf.Engine.Foundation.Precision.PConfusion() && lengthV > global::Kerf.Engine.Foundation.Precision.PConfusion();

        if (_isValid)
        {
            ComputeTolerance(lengthU, lengthV);
            ComputeDelta(lengthU, lengthV);
        }
    }

    /// <summary>True if the computed range is valid.</summary>
    public virtual bool IsValid => _isValid;

    /// <summary>
    /// The point scaled from the real parametric space to the face's basis if <paramref name="isToFaceBasis"/>, or
    /// back.
    /// </summary>
    public Point2D Scale(in Point2D point, bool isToFaceBasis)
    {
        return isToFaceBasis ? new Point2D((point.X - _rangeU.first) / _delta.first,
                                            (point.Y - _rangeV.first) / _delta.second)
                             : new Point2D(point.X * _delta.first + _rangeU.first,
                                            point.Y * _delta.second + _rangeV.first);
    }

    /// <summary>The nodes generated from the surface and the parameters; none (null) by default.</summary>
    public virtual NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        return null;
    }

    /// <summary>The 3D point of the point of the surface's parametric space.</summary>
    public Point3D Point(in Point2D point2d) => GetSurface().Value(point2d.X, point2d.Y);

    /// <summary>Computes the parametric tolerance from the lengths along U and V.</summary>
    protected virtual void ComputeTolerance(double lenU, double lenV)
    {
        double diffU = _rangeU.second - _rangeU.first;
        double diffV = _rangeV.second - _rangeV.first;

        // Slightly increase exact resolution so to cover links with approximate
        // length equal to resolution itself on sub-resolution differences.
        double tolerance = Tool.Tolerance(_dFace!.GetFace());
        SurfaceAdaptorBase surface = GetSurface();
        double resU = surface.UResolution(tolerance) * 1.1;
        double resV = surface.VResolution(tolerance) * 1.1;

        double deflectionUV = 1.0e-05;
        _tolerance.first = Max(Min(deflectionUV, resU), 1e-7 * diffU);
        _tolerance.second = Max(Min(deflectionUV, resV), 1e-7 * diffV);
    }

    /// <summary>Computes the parametric delta from the lengths along U and V and the tolerance.</summary>
    protected virtual void ComputeDelta(double lengthU, double lengthV)
    {
        double diffU = _rangeU.second - _rangeU.first;
        double diffV = _rangeV.second - _rangeV.first;

        _delta.first = diffU / (lengthU < _tolerance.first ? 1.0 : lengthU);
        _delta.second = diffV / (lengthV < _tolerance.second ? 1.0 : lengthV);
    }

    /// <summary>The face model.</summary>
    public FaceModel GetDFace() => _dFace!;

    /// <summary>The surface.</summary>
    public FaceSurfaceAdaptor GetSurface() => _dFace!.GetSurface();

    /// <summary>The U range.</summary>
    public (double first, double second) GetRangeU() => _rangeU;

    /// <summary>The V range.</summary>
    public (double first, double second) GetRangeV() => _rangeV;

    /// <summary>The delta.</summary>
    public (double first, double second) GetDelta() => _delta;

    /// <summary>The parametric tolerances along U and V.</summary>
    public (double first, double second) GetToleranceUV() => _tolerance;

    // Computes the length along U by evaluating a grid.
    private double ComputeLengthU()
    {
        double longu = 0.0;
        Point3D p11, p12, p21, p22, p31, p32;

        double du = 0.05 * (_rangeU.second - _rangeU.first);
        double dfvave = 0.5 * (_rangeV.second + _rangeV.first);
        double dfucur;
        int i1;

        FaceSurfaceAdaptor gFace = GetSurface();
        gFace.D0(_rangeU.first, _rangeV.first, out p11);
        gFace.D0(_rangeU.first, dfvave, out p21);
        gFace.D0(_rangeU.first, _rangeV.second, out p31);
        for (i1 = 1, dfucur = _rangeU.first + du; i1 <= 20; i1++, dfucur += du)
        {
            gFace.D0(dfucur, _rangeV.first, out p12);
            gFace.D0(dfucur, dfvave, out p22);
            gFace.D0(dfucur, _rangeV.second, out p32);
            longu += (p11.DistanceTo(p12) + p21.DistanceTo(p22) + p31.DistanceTo(p32));
            p11 = p12;
            p21 = p22;
            p31 = p32;
        }

        return longu / 3.0;
    }

    // Computes the length along V by evaluating a grid.
    private double ComputeLengthV()
    {
        double longv = 0.0;
        Point3D p11, p12, p21, p22, p31, p32;

        double dv = 0.05 * (_rangeV.second - _rangeV.first);
        double dfuave = 0.5 * (_rangeU.second + _rangeU.first);
        double dfvcur;
        int i1;

        FaceSurfaceAdaptor gFace = GetSurface();
        gFace.D0(_rangeU.first, _rangeV.first, out p11);
        gFace.D0(dfuave, _rangeV.first, out p21);
        gFace.D0(_rangeU.second, _rangeV.first, out p31);
        for (i1 = 1, dfvcur = _rangeV.first + dv; i1 <= 20; i1++, dfvcur += dv)
        {
            gFace.D0(_rangeU.first, dfvcur, out p12);
            gFace.D0(dfuave, dfvcur, out p22);
            gFace.D0(_rangeU.second, dfvcur, out p32);
            longv += (p11.DistanceTo(p12) + p21.DistanceTo(p22) + p31.DistanceTo(p32));
            p11 = p12;
            p21 = p22;
            p31 = p32;
        }

        return longv / 3.0;
    }

    // Updates the discrete range of the surface from its geometric range.
    private static void UpdateRange(double geomFirst,
                                    double geomLast,
                                    bool isPeriodic,
                                    ref double discreteFirst,
                                    ref double discreteLast)
    {
        if (discreteFirst < geomFirst || discreteLast > geomLast)
        {
            if (isPeriodic)
            {
                if ((discreteLast - discreteFirst) > (geomLast - geomFirst))
                {
                    discreteLast = discreteFirst + (geomLast - geomFirst);
                }
            }
            else
            {
                if ((discreteFirst < geomLast) && (discreteLast > geomFirst))
                {
                    // Protection against the faces whose pcurve is out of the surface's domain
                    //(see issue #23675 and test cases "bugs iges buc60820*")
                    if (geomFirst > discreteFirst)
                    {
                        discreteFirst = geomFirst;
                    }

                    if (geomLast < discreteLast)
                    {
                        discreteLast = geomLast;
                    }
                }
            }
        }
    }
}
