// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ApproxLine.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ApproxLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The line BRepApprox_Approx approximates: the points of an <see cref="LineOn2S"/>, or the poles of a 3D
/// B-spline curve and of two 2D B-spline curves (null curves give the origin).
/// </summary>
/// <remarks>
/// A working object (decision 609), sharing the curves and the line it is given; it is the line of BRepApprox's
/// instantiation of ApproxInt's generics (<see cref="IWLine{TSelf}"/>).
/// </remarks>
internal class ApproxLine : Transient, IWLine<ApproxLine>
{
    private readonly BSplineCurve3D? _curveXYZ;
    private readonly BSplineCurve2D? _curveUV1;
    private readonly BSplineCurve2D? _curveUV2;
    private readonly LineOn2S? _lineOn2S;

    public ApproxLine(BSplineCurve3D? curveXYZ, BSplineCurve2D? curveUV1,
                                 BSplineCurve2D? curveUV2)
    {
        _curveXYZ = curveXYZ;
        _curveUV1 = curveUV1;
        _curveUV2 = curveUV2;
    }

    /// <summary>The line on the points of <paramref name="lin"/>.</summary>
    /// <remarks><paramref name="tang"/> is only for compatibility with IntPatch_WLine: it is not used.</remarks>
    public ApproxLine(LineOn2S lin, bool tang = false)
    {
        _lineOn2S = lin;
    }

    static ApproxLine IWLine<ApproxLine>.Create(LineOn2S lin, bool tang)
        => new(lin, tang);

    public int NbPnts()
    {
        if (_curveXYZ is not null)
        {
            return (_curveXYZ.PoleCount);
        }
        if (_curveUV1 is not null)
        {
            return (_curveUV1.PoleCount);
        }
        if (_curveUV2 is not null)
        {
            return (_curveUV2.PoleCount);
        }
        return (_lineOn2S!.PointCount);
    }

    public PntOn2S Point(int index)
    {
        if (_lineOn2S is not null)
        {
            if (_lineOn2S.PointCount != 0)
            {
                return (_lineOn2S.Value(index));
            }
        }
        Point2D p1 = new(), p2 = new();
        Point3D p = new();
        if (_curveXYZ is not null)
        {
            p = _curveXYZ.Pole(index);
        }
        if (_curveUV1 is not null)
        {
            p1 = _curveUV1.Pole(index);
        }
        if (_curveUV2 is not null)
        {
            p2 = _curveUV2.Pole(index);
        }

        PntOn2S pntOn2S = new();
        pntOn2S.SetValue(p, p1.X, p1.Y, p2.X, p2.Y);

        return pntOn2S;
    }
}
