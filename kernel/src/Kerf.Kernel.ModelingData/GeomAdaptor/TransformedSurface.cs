// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// An adaptor for surfaces with an applied transformation: it wraps a <see cref="SurfaceAdaptor"/> and
/// applies a gp_Trsf to every point and derivative it evaluates. The base class of BRepAdaptor_Surface.
/// </summary>
/// <remarks>
/// The queries that return geometry (<see cref="Plane"/>, <see cref="BSpline"/>, <see cref="BasisSurface"/>, ...)
/// answer from an adaptor of the transformed surface, built on first use and kept until the surface or the
/// transformation changes.
/// </remarks>
public class TransformedSurface : SurfaceAdaptorBase
{
    protected SurfaceAdaptor _surf;
    protected Transform3D _trsf;
    private protected SurfaceAdaptor? _transformedAdaptor;

    /// <summary>An undefined surface with the identity transformation.</summary>
    public TransformedSurface()
    {
        _surf = new SurfaceAdaptor();
        _trsf = new Transform3D();
    }

    /// <summary>theSurface transformed by theTrsf.</summary>
    public TransformedSurface(Surface surface, in Transform3D trsf)
    {
        _surf = new SurfaceAdaptor(surface);
        _trsf = trsf;
        InvalidateTransformedCache();
    }

    /// <summary>theSurface over the given bounds (and snapping tolerances), transformed by theTrsf.</summary>
    public TransformedSurface(Surface surface,
                                          double uFirst,
                                          double uLast,
                                          double vFirst,
                                          double vLast,
                                          in Transform3D trsf,
                                          double tolU = 0.0,
                                          double tolV = 0.0)
    {
        _surf = new SurfaceAdaptor(surface, uFirst, uLast, vFirst, vLast, tolU, tolV);
        _trsf = trsf;
        InvalidateTransformedCache();
    }

    /// <summary>A shallow copy (shallow copies of the inner adaptors).</summary>
    public override SurfaceAdaptorBase ShallowCopy()
    {
        TransformedSurface copy = new TransformedSurface();
        copy._surf = (SurfaceAdaptor)_surf.ShallowCopy();
        copy._trsf = _trsf;
        // OCCT copies the optional adaptor by value (sharing its cache); the port gives the copy its own.
        copy._transformedAdaptor = (SurfaceAdaptor?)_transformedAdaptor?.ShallowCopy();
        return copy;
    }

    /// <summary>Loads theSurface over its bounds, with theTrsf.</summary>
    public void Load(Surface surface, in Transform3D trsf)
    {
        _surf.Load(surface);
        _trsf = trsf;
        InvalidateTransformedCache();
    }

    /// <summary>Loads theSurface over the given bounds, with theTrsf.</summary>
    public void Load(Surface surface,
                     double uFirst,
                     double uLast,
                     double vFirst,
                     double vLast,
                     in Transform3D trsf,
                     double tolU = 0.0,
                     double tolV = 0.0)
    {
        _surf.Load(surface, uFirst, uLast, vFirst, vLast, tolU, tolV);
        _trsf = trsf;
        InvalidateTransformedCache();
    }

    /// <summary>Sets the transformation.</summary>
    public void SetTrsf(in Transform3D trsf)
    {
        _trsf = trsf;
        InvalidateTransformedCache();
    }

    /// <summary>True if the transformation is not the identity.</summary>
    public bool HasTrsf => _trsf.Form() != Identity;

    /// <summary>The transformation.</summary>
    public Transform3D Trsf() => _trsf;

    /// <summary>The adaptor of the original surface (the adaptor itself: a working object).</summary>
    [Obsolete("Use AdaptorSurfaceOriginal() instead to get the original surface without transformation")]
    public SurfaceAdaptor Surface() => _surf;

    /// <summary>The adaptor of the original surface, without transformation.</summary>
    public SurfaceAdaptor AdaptorSurfaceOriginal() => _surf;

    /// <summary>The adaptor of the transformed surface (the original one for the identity).</summary>
    public SurfaceAdaptor AdaptorSurfaceTransformed() => TransformedAdaptor();

    /// <summary>The original Geom surface (shared).</summary>
    public Surface? GeomSurfaceOriginal() => _surf.Surface();

    /// <summary>The transformed Geom surface (the original one for the identity).</summary>
    public Surface? GeomSurfaceTransformed()
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.Surface();
        }
        EnsureTransformedCache();
        return _transformedAdaptor!.Surface();
    }

    /// <summary>The original Geom surface.</summary>
    [Obsolete("Use GeomSurfaceOriginal() or GeomSurfaceTransformed() instead")]
    public Surface? GeomSurface() => GeomSurfaceOriginal();

    public override double FirstUParameter() => _surf.FirstUParameter();

    public override double LastUParameter() => _surf.LastUParameter();

    public override double FirstVParameter() => _surf.FirstVParameter();

    public override double LastVParameter() => _surf.LastVParameter();

    public override Continuity UContinuity() => _surf.UContinuity();

    public override Continuity VContinuity() => _surf.VContinuity();

    public override int NbUIntervals(Continuity s) => _surf.NbUIntervals(s);

    public override int NbVIntervals(Continuity s) => _surf.NbVIntervals(s);

    public override void UIntervals(NCollection_Array1<double> t, Continuity s) => _surf.UIntervals(t, s);

    public override void VIntervals(NCollection_Array1<double> t, Continuity s) => _surf.VIntervals(t, s);

    /// <summary>The transformed surface's adaptor, trimmed in U.</summary>
    public override SurfaceAdaptorBase UTrim(double first, double last, double tol) =>
        TransformedAdaptor().UTrim(first, last, tol);

    /// <summary>The transformed surface's adaptor, trimmed in V.</summary>
    public override SurfaceAdaptorBase VTrim(double first, double last, double tol) =>
        TransformedAdaptor().VTrim(first, last, tol);

    public override bool IsUClosed => _surf.IsUClosed;

    public override bool IsVClosed => _surf.IsVClosed;

    public override bool IsUPeriodic => _surf.IsUPeriodic;

    public override double UPeriod() => _surf.UPeriod();

    public override bool IsVPeriodic => _surf.IsVPeriodic;

    public override double VPeriod() => _surf.VPeriod();

    /// <summary>The tolerance within which a U parameter is snapped to a bound.</summary>
    public double ToleranceU() => _surf.ToleranceU();

    /// <summary>The tolerance within which a V parameter is snapped to a bound.</summary>
    public double ToleranceV() => _surf.ToleranceV();

    /// <summary>Point evaluation. Applies transformation after evaluation.</summary>
    public sealed override Point3D EvalD0(double u, double v)
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.EvalD0(u, v);
        }
        return _surf.EvalD0(u, v).Transformed(_trsf);
    }

    /// <summary>D1 evaluation. Applies transformation after evaluation.</summary>
    public sealed override global::Kerf.Geometry.Surface.ResD1 EvalD1(double u, double v)
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.EvalD1(u, v);
        }
        global::Kerf.Geometry.Surface.ResD1 res = _surf.EvalD1(u, v);
        res.Point = res.Point.Transformed(_trsf);
        res.D1U = res.D1U.Transformed(_trsf);
        res.D1V = res.D1V.Transformed(_trsf);
        return res;
    }

    /// <summary>D2 evaluation. Applies transformation after evaluation.</summary>
    public sealed override global::Kerf.Geometry.Surface.ResD2 EvalD2(double u, double v)
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.EvalD2(u, v);
        }
        global::Kerf.Geometry.Surface.ResD2 res = _surf.EvalD2(u, v);
        res.Point = res.Point.Transformed(_trsf);
        res.D1U = res.D1U.Transformed(_trsf);
        res.D1V = res.D1V.Transformed(_trsf);
        res.D2U = res.D2U.Transformed(_trsf);
        res.D2V = res.D2V.Transformed(_trsf);
        res.D2UV = res.D2UV.Transformed(_trsf);
        return res;
    }

    /// <summary>D3 evaluation. Applies transformation after evaluation.</summary>
    public sealed override global::Kerf.Geometry.Surface.ResD3 EvalD3(double u, double v)
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.EvalD3(u, v);
        }
        global::Kerf.Geometry.Surface.ResD3 res = _surf.EvalD3(u, v);
        res.Point = res.Point.Transformed(_trsf);
        res.D1U = res.D1U.Transformed(_trsf);
        res.D1V = res.D1V.Transformed(_trsf);
        res.D2U = res.D2U.Transformed(_trsf);
        res.D2V = res.D2V.Transformed(_trsf);
        res.D2UV = res.D2UV.Transformed(_trsf);
        res.D3U = res.D3U.Transformed(_trsf);
        res.D3V = res.D3V.Transformed(_trsf);
        res.D3UUV = res.D3UUV.Transformed(_trsf);
        res.D3UVV = res.D3UVV.Transformed(_trsf);
        return res;
    }

    /// <summary>DN evaluation. Applies transformation after evaluation.</summary>
    public sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (_trsf.Form() == Identity)
        {
            return _surf.EvalDN(u, v, nu, nv);
        }
        return _surf.EvalDN(u, v, nu, nv).Transformed(_trsf);
    }

    public override double UResolution(double r3d) => _surf.UResolution(r3d);

    public override double VResolution(double r3d) => _surf.VResolution(r3d);

    public override SurfaceKind GetType() => _surf.GetType();

    public override Plane3D Plane() => TransformedAdaptor().Plane();

    public override Cylinder Cylinder() => TransformedAdaptor().Cylinder();

    public override Cone Cone() => TransformedAdaptor().Cone();

    public override Sphere Sphere() => TransformedAdaptor().Sphere();

    public override Torus Torus() => TransformedAdaptor().Torus();

    public override int UDegree() => _surf.UDegree();

    public override int UPoleCount => _surf.UPoleCount;

    public override int VDegree() => _surf.VDegree();

    public override int VPoleCount => _surf.VPoleCount;

    public override int UKnotCount => _surf.UKnotCount;

    public override int VKnotCount => _surf.VKnotCount;

    public override bool IsURational => _surf.IsURational;

    public override bool IsVRational => _surf.IsVRational;

    public override global::Kerf.Geometry.BezierSurface Bezier() => TransformedAdaptor().Bezier();

    public override global::Kerf.Geometry.BSplineSurface BSpline() => TransformedAdaptor().BSpline();

    public override Axis3D AxeOfRevolution() => TransformedAdaptor().AxeOfRevolution();

    public override Direction3D Direction() => TransformedAdaptor().Direction();

    public override CurveAdaptorBase BasisCurve() => TransformedAdaptor().BasisCurve();

    public override SurfaceAdaptorBase BasisSurface() => TransformedAdaptor().BasisSurface();

    public override double OffsetValue() => TransformedAdaptor().OffsetValue();

    protected void InvalidateTransformedCache()
    {
        _transformedAdaptor = null;
    }

    protected void EnsureTransformedCache()
    {
        if (_trsf.Form() == Identity || _transformedAdaptor is not null)
        {
            return;
        }
        InitTransformedCache();
    }

    protected void InitTransformedCache()
    {
        if (_surf.Surface() is null)
        {
            _transformedAdaptor = new SurfaceAdaptor();
            return;
        }
        bool isIdentity = _trsf.Form() == Identity;
        Transform3D trsf = _trsf;
        Surface surface;
        switch (_surf.GetType())
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                surface = isIdentity ? _surf.Surface()! : new PlaneSurface(_surf.Plane().Transformed(trsf));
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                surface = isIdentity
                    ? _surf.Surface()!
                    : new CylindricalSurface(_surf.Cylinder().Transformed(trsf));
                break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                surface = isIdentity ? _surf.Surface()! : new ConicalSurface(_surf.Cone().Transformed(trsf));
                break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                surface = isIdentity
                    ? _surf.Surface()!
                    : new SphericalSurface(_surf.Sphere().Transformed(trsf));
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                surface = isIdentity ? _surf.Surface()! : new ToroidalSurface(_surf.Torus().Transformed(trsf));
                break;
            default:
                surface = isIdentity ? _surf.Surface()! : (Surface)_surf.Surface()!.Transformed(trsf);
                break;
        }
        _transformedAdaptor = new SurfaceAdaptor(surface,
                                                       _surf.FirstUParameter(),
                                                       _surf.LastUParameter(),
                                                       _surf.FirstVParameter(),
                                                       _surf.LastVParameter(),
                                                       _surf.ToleranceU(),
                                                       _surf.ToleranceV());
    }

    protected SurfaceAdaptor TransformedAdaptor()
    {
        if (_trsf.Form() == Identity)
        {
            return _surf;
        }
        EnsureTransformedCache();
        return _transformedAdaptor!;
    }
}
