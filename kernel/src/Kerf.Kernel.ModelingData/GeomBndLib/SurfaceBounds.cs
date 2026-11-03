// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Surface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Bounding box of a surface: detects the surface type once and dispatches to the evaluator of that type
/// (GeomBndLib_Plane, _Cylinder, ..., _OtherSurface).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom_Surface, it owns a
/// GeomAdaptor_Surface for a rectangular trimmed surface or an unknown type. OCCT's <c>std::variant</c> of
/// evaluators is an interface field, null for <c>std::monostate</c> (decision 236's pattern).
/// </remarks>
public sealed class SurfaceBounds
{
    /// <summary>The members of the per-type evaluators that the dispatcher calls through OCCT's <c>std::visit</c>.</summary>
    internal interface ISurfaceEvaluator
    {
        Box Box(double tol);

        Box Box(double uMin, double uMax, double vMin, double vMax, double tol);

        Box BoxOptimal(double tol);

        Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol);
    }

    private readonly SurfaceAdaptorBase? _adaptorRef;
    private SurfaceAdaptorBase? _adaptorOwned;
    private ISurfaceEvaluator? _evaluator;
    private SurfaceKind _surfType;

    private static Surface? UnwrapSurface(Surface? surf)
    {
        TrimmedSurface? trim = surf as TrimmedSurface;
        return trim is null ? surf : UnwrapSurface(trim.BasisSurface);
    }

    /// <summary>Construct from an adaptor surface.</summary>
    public SurfaceBounds(SurfaceAdaptorBase surf)
    {
        _surfType = surf.GetType();
        _adaptorRef = surf;

        // Detect the adaptor type ONCE and extract the underlying Geom_Surface handle
        // (for GeomAdaptor_TransformedSurface, the transformed geometry).
        Surface? geomSurf = null;
        if (surf is SurfaceAdaptor ga)
        {
            geomSurf = ga.Surface();
        }
        else if (surf is TransformedSurface gts)
        {
            geomSurf = gts.GeomSurfaceTransformed();
        }

        Surface? baseGeom = UnwrapSurface(geomSurf);
        if (baseGeom is not null && InitFromSurfaceType(_surfType, baseGeom))
        {
            return;
        }

        switch (_surfType)
        {
            case Plane:
                _evaluator = new PlaneBounds(new PlaneSurface(_adaptorRef.Plane()));
                break;
            case SurfaceKind.Cylinder:
                _evaluator = new CylinderBounds(new CylindricalSurface(_adaptorRef.Cylinder()));
                break;
            case SurfaceKind.Cone:
                _evaluator = new ConeBounds(new ConicalSurface(_adaptorRef.Cone()));
                break;
            case SurfaceKind.Sphere:
                _evaluator = new SphereBounds(new SphericalSurface(_adaptorRef.Sphere()));
                break;
            case SurfaceKind.Torus:
                _evaluator = new TorusBounds(new ToroidalSurface(_adaptorRef.Torus()));
                break;
            case SurfaceKind.BezierSurface:
                _evaluator = new BezierSurfaceBounds(_adaptorRef.Bezier());
                break;
            case SurfaceKind.BSplineSurface:
                _evaluator = new BSplineSurfaceBounds(_adaptorRef.BSpline());
                break;
            default:
                InitOtherSurface(_adaptorRef);
                break;
        }
    }

    /// <summary>Construct from a Geom_Surface.</summary>
    public SurfaceBounds(Surface surf)
    {
        _surfType = OtherSurface;

        if (surf is TrimmedSurface trim)
        {
            _adaptorOwned = new SurfaceAdaptor(surf);
            if (!InitFromSurface(trim.BasisSurface))
            {
                _evaluator = new OtherSurfaceBounds(_adaptorOwned);
            }
            return;
        }

        if (!InitFromSurface(surf))
        {
            InitOtherSurface(surf);
        }
    }

    private bool InitFromSurface(Surface? surf)
    {
        if (surf is null)
        {
            return false;
        }

        // OCCT compares the dynamic type exactly (STANDARD_TYPE), not IsKind.
        Type type = ((object)surf).GetType();
        SurfaceKind surfType;
        if (type == typeof(PlaneSurface))
        {
            surfType = Plane;
        }
        else if (type == typeof(CylindricalSurface))
        {
            surfType = SurfaceKind.Cylinder;
        }
        else if (type == typeof(ConicalSurface))
        {
            surfType = SurfaceKind.Cone;
        }
        else if (type == typeof(SphericalSurface))
        {
            surfType = SurfaceKind.Sphere;
        }
        else if (type == typeof(ToroidalSurface))
        {
            surfType = SurfaceKind.Torus;
        }
        else if (type == typeof(global::Kerf.Geometry.BezierSurface))
        {
            surfType = SurfaceKind.BezierSurface;
        }
        else if (type == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            surfType = SurfaceKind.BSplineSurface;
        }
        else if (type == typeof(RevolutionSurface))
        {
            surfType = SurfaceOfRevolution;
        }
        else if (type == typeof(ExtrusionSurface))
        {
            surfType = SurfaceOfExtrusion;
        }
        else if (type == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            surfType = SurfaceKind.OffsetSurface;
        }
        else
        {
            return false;
        }

        _surfType = surfType;
        return InitFromSurfaceType(surfType, surf);
    }

    // OCCT's down_cast yields a null handle when the adaptor's type does not match its surface; the port's
    // evaluators then hold null, as OCCT's do, and fail when used.
    private bool InitFromSurfaceType(SurfaceKind type, Surface surf)
    {
        switch (type)
        {
            case Plane:
                _evaluator = new PlaneBounds((surf as PlaneSurface)!);
                return true;
            case SurfaceKind.Cylinder:
                _evaluator = new CylinderBounds((surf as CylindricalSurface)!);
                return true;
            case SurfaceKind.Cone:
                _evaluator = new ConeBounds((surf as ConicalSurface)!);
                return true;
            case SurfaceKind.Sphere:
                _evaluator = new SphereBounds((surf as SphericalSurface)!);
                return true;
            case SurfaceKind.Torus:
                _evaluator = new TorusBounds((surf as ToroidalSurface)!);
                return true;
            case SurfaceKind.BezierSurface:
                _evaluator = new BezierSurfaceBounds((surf as global::Kerf.Geometry.BezierSurface)!);
                return true;
            case SurfaceKind.BSplineSurface:
                _evaluator = new BSplineSurfaceBounds((surf as global::Kerf.Geometry.BSplineSurface)!);
                return true;
            case SurfaceOfRevolution:
                _evaluator = new SurfaceOfRevolutionBounds((surf as RevolutionSurface)!);
                return true;
            case SurfaceOfExtrusion:
                _evaluator = new SurfaceOfExtrusionBounds((surf as ExtrusionSurface)!);
                return true;
            case SurfaceKind.OffsetSurface:
                _evaluator = new OffsetSurfaceBounds((surf as global::Kerf.Geometry.OffsetSurface)!);
                return true;
            default:
                return false;
        }
    }

    private void InitOtherSurface(SurfaceAdaptorBase surf)
    {
        _surfType = OtherSurface;
        _evaluator = new OtherSurfaceBounds(surf);
    }

    private void InitOtherSurface(Surface surf)
    {
        _adaptorOwned = new SurfaceAdaptor(surf);
        _surfType = OtherSurface;
        _evaluator = new OtherSurfaceBounds(_adaptorOwned);
    }

    /// <summary>Return detected surface type.</summary>
    public new SurfaceKind GetType() => _surfType;

    private SurfaceAdaptorBase? Adaptor() => _adaptorRef ?? _adaptorOwned;

    private double EffectiveU1()
    {
        SurfaceAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.FirstUParameter() : 0.0;
    }

    private double EffectiveU2()
    {
        SurfaceAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.LastUParameter() : 0.0;
    }

    private double EffectiveV1()
    {
        SurfaceAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.FirstVParameter() : 0.0;
    }

    private double EffectiveV2()
    {
        SurfaceAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.LastVParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
    {
        if (Adaptor() is not null)
        {
            return Box(EffectiveU1(), EffectiveU2(), EffectiveV1(), EffectiveV2(), tol);
        }

        return _evaluator is null ? new Box() : _evaluator.Box(tol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
        => _evaluator is null ? new Box() : _evaluator.Box(uMin, uMax, vMin, vMax, tol);

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double tol)
    {
        if (Adaptor() is not null)
        {
            return BoxOptimal(EffectiveU1(), EffectiveU2(), EffectiveV1(), EffectiveV2(), tol);
        }

        return _evaluator is null ? new Box() : _evaluator.BoxOptimal(tol);
    }

    /// <summary>Compute precise bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
        => _evaluator is null ? new Box() : _evaluator.BoxOptimal(uMin, uMax, vMin, vMax, tol);

    /// <summary>Add bounding box for full surface.</summary>
    public void Add(double tol, ref Box box) => box.Add(Box(tol));

    /// <summary>Add bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public void Add(double uMin, double uMax, double vMin, double vMax, double tol, ref Box box)
        => box.Add(Box(uMin, uMax, vMin, vMax, tol));

    /// <summary>Add precise bounding box for full surface.</summary>
    public void AddOptimal(double tol, ref Box box) => box.Add(BoxOptimal(tol));

    /// <summary>Add precise bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public void AddOptimal(double uMin, double uMax, double vMin, double vMax, double tol, ref Box box)
        => box.Add(BoxOptimal(uMin, uMax, vMin, vMax, tol));
}
