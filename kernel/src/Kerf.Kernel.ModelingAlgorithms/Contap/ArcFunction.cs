// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.HiddenLines.Contours.ContourTFunction;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The contour function along an arc of restriction of a surface: the normal's component along the view direction
/// (or towards the eye), scaled by the mean normal length, F(t) = N(S(A(t))) . D / mean, with its derivative; the
/// function of the boundary search (<see cref="TheSearch"/>). Each state saved by <see cref="GetStateNumber"/>
/// keeps the last evaluated point.
/// </summary>
/// <remarks>
/// A working object. It implements <see cref="IArcFunction"/>, what IntStart_SearchOnBoundaries reads; its
/// quadric is OCCT's default one, never set. With <see cref="DraftPrs"/> OCCT computes nothing and leaves the
/// value and derivative unset; the port returns 0 (decision 1076).
/// </remarks>
internal class ArcFunction : FunctionWithDerivative, IArcFunction
{
    private Curve2DAdaptorBase? _arc;
    private SurfaceAdaptorBase? _surf;
    private double _mean;
    private ContourTFunction _type;
    private Direction3D _dir;
    private double _cosAng;
    private Point3D _eye;
    private Point3D _solpt;
    private readonly NCollection_Sequence<Point3D> _seqpt = new();
    private SurfaceQuadric _quad;

    public ArcFunction()
    {
        _mean = 1.0;
        _type = ContourTFunction.ContourStd;
        _dir = new Direction3D(Direction3D.D.Z);
        _cosAng = 0.0;
        _eye = new Point3D();
        _solpt = new Point3D();
        _quad = new SurfaceQuadric();
    }

    /// <summary>Sets the surface, and the mean length of its normal at its sample points.</summary>
    public void Set(SurfaceAdaptorBase s)
    {
        _surf = s;
        int i;
        int nbs = HContTool.NbSamplePoints(s);
        double u, v;
        //  gp_Vec d1u,d1v;
        Vector3D norm;
        if (nbs > 0)
        {
            _mean = 0.0;
            for (i = 1; i <= nbs; i++)
            {
                HContTool.SamplePoint(s, i, out u, out v);
                //      Adaptor3d_HSurfaceTool::D1(S,U,V,solpt,d1u,d1v);
                //      myMean = myMean + d1u.Crossed(d1v).Magnitude();
                SurfProps.Normale(s, u, v, out _solpt, out norm);
                _mean = _mean + norm.Magnitude;
            }
            _mean = _mean / ((double)nbs);
        }
    }

    /// <summary>A silhouette for the view direction <paramref name="direction"/>.</summary>
    public void Set(in Direction3D direction)
    {
        _type = ContourTFunction.ContourStd;
        _dir = direction;
    }

    /// <summary>A draft contour of angle <paramref name="angle"/> for the direction <paramref name="direction"/>.</summary>
    public void Set(in Direction3D direction, double angle)
    {
        _type = ContourTFunction.DraftStd;
        _dir = direction;
        _cosAng = Math.Cos(M_PI / 2.0 + angle);
    }

    /// <summary>A silhouette for the eye <paramref name="eye"/>.</summary>
    public void Set(in Point3D eye)
    {
        _type = ContourTFunction.ContourPrs;
        _eye = eye;
    }

    /// <summary>A draft contour of angle <paramref name="angle"/> for the eye <paramref name="eye"/>.</summary>
    public void Set(in Point3D eye, double angle)
    {
        _type = ContourTFunction.DraftPrs;
        _eye = eye;
        _cosAng = Math.Cos(M_PI / 2.0 + angle);
    }

    /// <summary>Sets the arc (and forgets the saved points).</summary>
    public void Set(Curve2DAdaptorBase a)
    {
        _arc = a;
        _seqpt.Clear();
    }

    public override bool Value(double u, out double f)
    {
        f = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 1076)
        // gp_Vec d1u,d1v;
        Point2D pt2d = ContourHCurve2dTool.Value(_arc!, u);
        //  Adaptor3d_HSurfaceTool::D1(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v);
        //  gp_Vec norm(d1u.Crossed(d1v));
        SurfProps.Normale(_surf!, pt2d.X, pt2d.Y, out _solpt, out Vector3D norm);

        switch (_type)
        {
            case ContourTFunction.ContourStd:
                f = (norm.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
                f = (norm.Dot(new Vector3D(_eye, _solpt))) / _mean;
                break;
            case ContourTFunction.DraftStd:
                f = (norm.Dot(new Vector3D(_dir)) - _cosAng * norm.Magnitude) / _mean;
                break;
            case ContourTFunction.DraftPrs:
            default:
                break;
        }
        return true;
    }

    public override bool Derivative(double u, out double d)
    {
        double dfu = 0.0, dfv = 0.0;
        //  gp_Vec d1u,d1v,d2u,d2v,d2uv;
        ContourHCurve2dTool.D1(_arc!, u, out Point2D pt2d, out Vector2D d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        SurfProps.NormAndDn(_surf!, pt2d.X, pt2d.Y, out _solpt, out Vector3D norm, out Vector3D dnu,
                                   out Vector3D dnv);

        switch (_type)
        {
            case ContourTFunction.ContourStd:
                //      dfu = ((d2u.Crossed(d1v) + d1u.Crossed(d2uv)).Dot(myDir))/myMean;
                //      dfv = ((d2uv.Crossed(d1v) + d1u.Crossed(d2v)).Dot(myDir))/myMean;
                dfu = (dnu.Dot(new Vector3D(_dir))) / _mean;
                dfv = (dnv.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
            {
                Vector3D ep = new(_eye, _solpt);
                dfu = (dnu.Dot(ep)) / _mean;
                dfv = (dnv.Dot(ep)) / _mean;
            }
            break;
            case ContourTFunction.DraftStd:
                norm = norm.Normalized();
                dfu = (dnu.Dot(new Vector3D(_dir)) - _cosAng * dnu.Dot(norm)) / _mean;
                dfv = (dnv.Dot(new Vector3D(_dir)) - _cosAng * dnv.Dot(norm)) / _mean;
                break;
            case ContourTFunction.DraftPrs:
            default:
                break;
        }
        d = d2d.X * dfu + d2d.Y * dfv;
        return true;
    }

    public override bool Values(double u, out double f, out double d)
    {
        f = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 1076)
        double dfu = 0.0, dfv = 0.0;
        // gp_Vec d1u,d1v,d2u,d2v,d2uv;
        ContourHCurve2dTool.D1(_arc!, u, out Point2D pt2d, out Vector2D d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        //  gp_Vec norm(d1u.Crossed(d1v));
        SurfProps.NormAndDn(_surf!, pt2d.X, pt2d.Y, out _solpt, out Vector3D norm, out Vector3D dnu,
                                   out Vector3D dnv);

        switch (_type)
        {
            case ContourTFunction.ContourStd:
                f = (norm.Dot(new Vector3D(_dir))) / _mean;
                dfu = (dnu.Dot(new Vector3D(_dir))) / _mean;
                dfv = (dnv.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
            {
                Vector3D ep = new(_eye, _solpt);
                f = (norm.Dot(ep)) / _mean;
                dfu = (dnu.Dot(ep)) / _mean;
                dfv = (dnv.Dot(ep)) / _mean;
            }
            break;
            case ContourTFunction.DraftStd:
                f = (norm.Dot(new Vector3D(_dir)) - _cosAng * norm.Magnitude) / _mean;
                norm = norm.Normalized();
                dfu = (dnu.Dot(new Vector3D(_dir)) - _cosAng * dnu.Dot(norm)) / _mean;
                dfv = (dnv.Dot(new Vector3D(_dir)) - _cosAng * dnv.Dot(norm)) / _mean;
                break;
            case ContourTFunction.DraftPrs:
            default:
                break;
        }

        d = d2d.X * dfu + d2d.Y * dfv;
        return true;
    }

    /// <summary>Saves the last evaluated point; returns its rank for <see cref="Valpoint"/>.</summary>
    public override int GetStateNumber()
    {
        _seqpt.Append(_solpt);
        return _seqpt.Length();
    }

    /// <summary>The number of samples on the arc: the largest of the surface's and the arc's.</summary>
    public int NbSamples()
    {
        return Max(Max(HContTool.NbSamplesU(_surf!, 0.0, 0.0), HContTool.NbSamplesV(_surf!, 0.0, 0.0)),
                   HContTool.NbSamplesOnArc(_arc!));
    }

    /// <summary>The point saved at state <paramref name="index"/>.</summary>
    public Point3D Valpoint(int index) => _seqpt.Value(index);

    /// <summary>OCCT's default quadric (the contour function has none).</summary>
    public SurfaceQuadric Quadric() => (_quad);

    /// <summary>The surface.</summary>
    public SurfaceAdaptorBase Surface() => (_surf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public Point3D LastComputedPoint() => _solpt;
}
