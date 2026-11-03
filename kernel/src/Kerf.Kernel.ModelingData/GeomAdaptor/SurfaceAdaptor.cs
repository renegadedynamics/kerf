// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// An interface between the services provided by any surface from the package Geom and those required of the
/// surface by algorithms which use it. Creation of the loaded surface: the surface is C1 by piece.
/// </summary>
/// <remarks>
/// <para>Polynomial coefficients of B-spline and Bezier surfaces are cached in the adaptor (a
/// <see cref="SurfaceCache"/>, rebuilt when the evaluated span changes), so evaluating one adaptor from several
/// threads at once is not safe; give each thread its own (<see cref="ShallowCopy"/> doesn't copy the cache).</para>
/// <para>A rectangular trimmed surface is replaced by its basis surface (with the given bounds); the surface and
/// the surfaces returned by <see cref="BSpline"/>, <see cref="Bezier"/> and <see cref="Surface"/> are the shared
/// geometry: the adaptor never changes it.</para>
/// </remarks>
public class SurfaceAdaptor : SurfaceAdaptorBase
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.5;

    /// <summary>Evaluation data of a surface of linear extrusion.</summary>
    internal sealed class ExtrusionData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public CurveAdaptorBase BasisCurve = null!;

        /// <summary>Extrusion direction XYZ (normalized).</summary>
        public Xyz Direction;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a surface of revolution.</summary>
    internal sealed class RevolutionData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public CurveAdaptorBase BasisCurve = null!;

        /// <summary>Revolution axis.</summary>
        public Axis3D Axis;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of an offset surface.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for basis surface.</summary>
        public SurfaceAdaptor BasisAdaptor = null!;

        /// <summary>Adaptor for equivalent surface (if exists).</summary>
        public SurfaceAdaptor? EquivalentAdaptor;

        /// <summary>Original offset surface for osculating queries.</summary>
        public global::Kerf.Geometry.OffsetSurface OffsetSurface = null!;

        /// <summary>Offset distance.</summary>
        public double Offset;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier surface.</summary>
    internal sealed class BezierData
    {
        /// <summary>Bezier surface to prevent downcasts.</summary>
        public global::Kerf.Geometry.BezierSurface Surface = null!;

        /// <summary>Cached data for evaluation.</summary>
        public SurfaceCache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline surface.</summary>
    internal sealed class BSplineData
    {
        /// <summary>BSpline surface to prevent downcasts.</summary>
        public global::Kerf.Geometry.BSplineSurface Surface = null!;

        /// <summary>Cached data for evaluation.</summary>
        public SurfaceCache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public RepSurfaceDesc.Base? EvalRep;
    }

    protected Surface? _surface;
    protected double _uFirst;
    protected double _uLast;
    protected double _vFirst;
    protected double _vLast;
    protected double _tolU;
    protected double _tolV;
    protected SurfaceKind _surfaceType;

    // OCCT's std::variant<monostate, gp_Pln, gp_Cylinder, gp_Cone, gp_Sphere, gp_Torus, ExtrusionData,
    // RevolutionData, OffsetData, BezierData, BSplineData>: null, a boxed gp_* value or one of the data classes.
    private protected object? _surfaceData;

    /// <summary>An adaptor without a surface.</summary>
    public SurfaceAdaptor()
    {
        _uFirst = 0.0;
        _uLast = 0.0;
        _vFirst = 0.0;
        _vLast = 0.0;
        _tolU = 0.0;
        _tolV = 0.0;
        _surfaceType = OtherSurface;
    }

    /// <summary>An adaptor of theSurf over its bounds.</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    public SurfaceAdaptor(Surface surf)
    {
        _tolU = 0.0;
        _tolV = 0.0;
        Load(surf);
    }

    /// <summary>
    /// An adaptor of theSurf over [theUFirst, theULast] x [theVFirst, theVLast]. Evaluation snaps parameters
    /// within theTolU (theTolV) of a bound to the bound.
    /// </summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast or theVFirst &gt; theVLast.</exception>
    public SurfaceAdaptor(Surface surf,
                               double uFirst,
                               double uLast,
                               double vFirst,
                               double vLast,
                               double tolU = 0.0,
                               double tolV = 0.0)
    {
        Load(surf, uFirst, uLast, vFirst, vLast, tolU, tolV);
    }

    private static bool HasEvalRep(object? data)
    {
        if (data is BezierData bezierData)
        {
            return bezierData.EvalRep is not null;
        }
        if (data is BSplineData bSplineData)
        {
            return bSplineData.EvalRep is not null;
        }
        if (data is ExtrusionData extrusionData)
        {
            return extrusionData.EvalRep is not null;
        }
        if (data is RevolutionData revolutionData)
        {
            return revolutionData.EvalRep is not null;
        }
        if (data is OffsetData offsetData)
        {
            return offsetData.EvalRep is not null;
        }
        return false;
    }

    private static Continuity LocalContinuity(int degree,
                                                 int nb,
                                                 NCollection_Array1<double> tk,
                                                 NCollection_Array1<int> tm,
                                                 double pFirst,
                                                 double pLast,
                                                 bool isPeriodic)
    {
        Standard_DomainError_Raise_if((tk.Length() != nb || tm.Length() != nb), " ");
        int index1;
        int index2;
        double newFirst, newLast;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, tk, tm, pFirst, isPeriodic, 1, nb, out index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, tk, tm, pLast, isPeriodic, 1, nb, out index2, out newLast);
        double epsKnot = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (Math.Abs(newFirst - tk[index1 + 1]) < epsKnot)
        {
            index1++;
        }
        if (Math.Abs(newLast - tk[index2]) < epsKnot)
        {
            index2--;
        }
        // Handle periodic curves.
        if ((isPeriodic) && (index1 == nb))
        {
            index1 = 1;
        }

        if (index2 != index1)
        {
            int i, multmax = tm[index1 + 1];
            for (i = index1 + 1; i <= index2; i++)
            {
                if (tm[i] > multmax)
                {
                    multmax = tm[i];
                }
            }
            multmax = degree - multmax;
            if (multmax <= 0)
            {
                return C0;
            }
            switch (multmax)
            {
                case 1:
                    return C1;
                case 2:
                    return C2;
                case 3:
                    return C3;
            }
        }
        return CN;
    }

    // Offset surface evaluation with retry mechanism for singular points; uses the equivalent surface adaptor
    // for faster evaluation when available. Return true if evaluation succeeded.
    private static bool OffsetD0(double u, double v, OffsetData data, out Point3D value)
    {
        if (data.EquivalentAdaptor is not null)
        {
            value = data.EquivalentAdaptor.EvalD0(u, v);
            return true;
        }
        return OffsetSurfaceUtils.EvaluateD0(u,
                                                  v,
                                                  new OffsetSurfaceUtils.AdaptorBasis(data.BasisAdaptor),
                                                  data.Offset,
                                                  data.OffsetSurface.OsculatingSurface(),
                                                  out value);
    }

    private static bool OffsetD1(double u, double v, OffsetData data, out global::Kerf.Geometry.Surface.ResD1 result)
    {
        if (data.EquivalentAdaptor is not null)
        {
            result = data.EquivalentAdaptor.EvalD1(u, v);
            return true;
        }
        return OffsetSurfaceUtils.EvaluateD1(u,
                                                  v,
                                                  new OffsetSurfaceUtils.AdaptorBasis(data.BasisAdaptor),
                                                  data.Offset,
                                                  data.OffsetSurface.OsculatingSurface(),
                                                  out result.Point,
                                                  out result.D1U,
                                                  out result.D1V);
    }

    private static bool OffsetD2(double u, double v, OffsetData data, out global::Kerf.Geometry.Surface.ResD2 result)
    {
        if (data.EquivalentAdaptor is not null)
        {
            result = data.EquivalentAdaptor.EvalD2(u, v);
            return true;
        }
        return OffsetSurfaceUtils.EvaluateD2(u,
                                                  v,
                                                  new OffsetSurfaceUtils.AdaptorBasis(data.BasisAdaptor),
                                                  data.Offset,
                                                  data.OffsetSurface.OsculatingSurface(),
                                                  out result.Point,
                                                  out result.D1U,
                                                  out result.D1V,
                                                  out result.D2U,
                                                  out result.D2V,
                                                  out result.D2UV);
    }

    private static bool OffsetD3(double u, double v, OffsetData data, out global::Kerf.Geometry.Surface.ResD3 result)
    {
        if (data.EquivalentAdaptor is not null)
        {
            result = data.EquivalentAdaptor.EvalD3(u, v);
            return true;
        }
        return OffsetSurfaceUtils.EvaluateD3(u,
                                                  v,
                                                  new OffsetSurfaceUtils.AdaptorBasis(data.BasisAdaptor),
                                                  data.Offset,
                                                  data.OffsetSurface.OsculatingSurface(),
                                                  out result.Point,
                                                  out result.D1U,
                                                  out result.D1V,
                                                  out result.D2U,
                                                  out result.D2V,
                                                  out result.D2UV,
                                                  out result.D3U,
                                                  out result.D3V,
                                                  out result.D3UUV,
                                                  out result.D3UVV);
    }

    private static bool OffsetDN(double u, double v, OffsetData data, int nu, int nv,
                                 out Vector3D result)
    {
        if (data.EquivalentAdaptor is not null)
        {
            result = data.EquivalentAdaptor.EvalDN(u, v, nu, nv);
            return true;
        }
        return OffsetSurfaceUtils.EvaluateDN(u,
                                                  v,
                                                  nu,
                                                  nv,
                                                  new OffsetSurfaceUtils.AdaptorBasis(data.BasisAdaptor),
                                                  data.Offset,
                                                  data.OffsetSurface.OsculatingSurface(),
                                                  out result);
    }

    /// <summary>A shallow copy: the same surface, new evaluation caches (and copies of the inner adaptors).</summary>
    public override SurfaceAdaptorBase ShallowCopy()
    {
        SurfaceAdaptor copy = new SurfaceAdaptor();

        copy._surface = _surface;
        copy._uFirst = _uFirst;
        copy._uLast = _uLast;
        copy._vFirst = _vFirst;
        copy._vLast = _vLast;
        copy._tolU = _tolU;
        copy._tolV = _tolV;
        copy._surfaceType = _surfaceType;

        // Copy surface-specific evaluation data
        if (_surfaceData is ExtrusionData extrusionData)
        {
            ExtrusionData newData = new ExtrusionData();
            newData.BasisCurve = extrusionData.BasisCurve.ShallowCopy();
            newData.Direction = extrusionData.Direction;
            newData.EvalRep = extrusionData.EvalRep;
            copy._surfaceData = newData;
        }
        else if (_surfaceData is RevolutionData revolutionData)
        {
            RevolutionData newData = new RevolutionData();
            newData.BasisCurve = revolutionData.BasisCurve.ShallowCopy();
            newData.Axis = revolutionData.Axis;
            newData.EvalRep = revolutionData.EvalRep;
            copy._surfaceData = newData;
        }
        else if (_surfaceData is OffsetData offsetData)
        {
            OffsetData newData = new OffsetData();
            newData.BasisAdaptor = (SurfaceAdaptor)offsetData.BasisAdaptor.ShallowCopy();
            if (offsetData.EquivalentAdaptor is not null)
            {
                newData.EquivalentAdaptor = (SurfaceAdaptor)offsetData.EquivalentAdaptor.ShallowCopy();
            }
            newData.OffsetSurface = offsetData.OffsetSurface; // Shared handle to original surface
            newData.Offset = offsetData.Offset;
            newData.EvalRep = offsetData.EvalRep;
            copy._surfaceData = newData;
        }
        else if (_surfaceData is BSplineData bSplineData)
        {
            BSplineData newData = new BSplineData();
            newData.Surface = bSplineData.Surface;
            newData.EvalRep = bSplineData.EvalRep;
            // Cache is not copied - will be rebuilt on demand
            copy._surfaceData = newData;
        }
        else if (_surfaceData is BezierData bezierData)
        {
            BezierData newData = new BezierData();
            newData.Surface = bezierData.Surface;
            newData.EvalRep = bezierData.EvalRep;
            copy._surfaceData = newData;
        }
        else
        {
            // Elementary surface types (gp_Pln, gp_Cylinder, etc.) are value types - direct copy (boxed values are
            // never changed in place)
            copy._surfaceData = _surfaceData;
        }

        return copy;
    }

    /// <summary>Loads theSurf over its bounds.</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    public void Load(Surface surf)
    {
        if (surf is null)
        {
            throw new NullObjectException("GeomAdaptor_Surface::Load");
        }
        surf.Bounds(out double u1, out double u2, out double v1, out double v2);
        LoadCore(surf, u1, u2, v1, v2);
    }

    /// <summary>Loads theSurf over [theUFirst, theULast] x [theVFirst, theVLast].</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast or theVFirst &gt; theVLast.</exception>
    public void Load(Surface surf,
                     double uFirst,
                     double uLast,
                     double vFirst,
                     double vLast,
                     double tolU = 0.0,
                     double tolV = 0.0)
    {
        if (surf is null)
        {
            throw new NullObjectException("GeomAdaptor_Surface::Load");
        }
        if (uFirst > uLast || vFirst > vLast)
        {
            throw new ConstructionErrorException("GeomAdaptor_Surface::Load");
        }
        LoadCore(surf, uFirst, uLast, vFirst, vLast, tolU, tolV);
    }

    /// <summary>The adapted surface (the basis surface for a trimmed surface), shared: null before a Load.</summary>
    public Surface? Surface() => _surface;

    public override double FirstUParameter() => _uFirst;

    public override double LastUParameter() => _uLast;

    public override double FirstVParameter() => _vFirst;

    public override double LastVParameter() => _vLast;

    /// <summary>The parameter bounds (virtual First/Last parameters).</summary>
    public void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = FirstUParameter();
        u2 = LastUParameter();
        v1 = FirstVParameter();
        v2 = LastVParameter();
    }

    /// <summary>The tolerance within which a U parameter is snapped to a bound.</summary>
    public double ToleranceU() => _tolU;

    /// <summary>The tolerance within which a V parameter is snapped to a bound.</summary>
    public double ToleranceV() => _tolV;

    private void LoadCore(Surface s,
                      double uFirst,
                      double uLast,
                      double vFirst,
                      double vLast,
                      double tolU = 0.0,
                      double tolV = 0.0)
    {
        _tolU = tolU;
        _tolV = tolV;
        _uFirst = uFirst;
        _uLast = uLast;
        _vFirst = vFirst;
        _vLast = vLast;

        if (!ReferenceEquals(_surface, s))
        {
            _surface = s;
            _surfaceData = null;

            Type theType = s.DynamicType();
            if (theType == typeof(TrimmedSurface))
            {
                Load(((TrimmedSurface)s).BasisSurface(), uFirst, uLast, vFirst, vLast);
            }
            else if (theType == typeof(PlaneSurface))
            {
                _surfaceType = global::Kerf.Geometry.SurfaceKind.Plane;
                _surfaceData = ((PlaneSurface)s).Pln();
            }
            else if (theType == typeof(CylindricalSurface))
            {
                _surfaceType = global::Kerf.Geometry.SurfaceKind.Cylinder;
                _surfaceData = ((CylindricalSurface)s).Cylinder();
            }
            else if (theType == typeof(ConicalSurface))
            {
                _surfaceType = global::Kerf.Geometry.SurfaceKind.Cone;
                _surfaceData = ((ConicalSurface)s).Cone();
            }
            else if (theType == typeof(SphericalSurface))
            {
                _surfaceType = global::Kerf.Geometry.SurfaceKind.Sphere;
                _surfaceData = ((SphericalSurface)s).Sphere();
            }
            else if (theType == typeof(ToroidalSurface))
            {
                _surfaceType = global::Kerf.Geometry.SurfaceKind.Torus;
                _surfaceData = ((ToroidalSurface)s).Torus();
            }
            else if (theType == typeof(RevolutionSurface))
            {
                _surfaceType = SurfaceOfRevolution;
                RevolutionSurface revSurf = (RevolutionSurface)_surface;
                // Populate revolution surface data
                RevolutionData revData = new RevolutionData();
                revData.BasisCurve = new CurveAdaptor(revSurf.BasisCurve(), _vFirst, _vLast);
                revData.Axis = revSurf.Axis();
                revData.EvalRep = revSurf.EvalRepresentation();
                _surfaceData = revData;
            }
            else if (theType == typeof(ExtrusionSurface))
            {
                _surfaceType = SurfaceOfExtrusion;
                ExtrusionSurface extSurf = (ExtrusionSurface)_surface;
                // Populate extrusion surface data with XYZ for fast evaluation
                ExtrusionData extData = new ExtrusionData();
                extData.BasisCurve = new CurveAdaptor(extSurf.BasisCurve(), _uFirst, _uLast);
                extData.Direction = extSurf.Direction().XYZ();
                extData.EvalRep = extSurf.EvalRepresentation();
                _surfaceData = extData;
            }
            else if (theType == typeof(global::Kerf.Geometry.BezierSurface))
            {
                _surfaceType = SurfaceKind.BezierSurface;
                BezierData bezierData = new BezierData();
                bezierData.Surface = (global::Kerf.Geometry.BezierSurface)_surface;
                bezierData.EvalRep = bezierData.Surface.EvalRepresentation();
                _surfaceData = bezierData;
            }
            else if (theType == typeof(global::Kerf.Geometry.BSplineSurface))
            {
                _surfaceType = SurfaceKind.BSplineSurface;
                BSplineData bSplineData = new BSplineData();
                bSplineData.Surface = (global::Kerf.Geometry.BSplineSurface)_surface;
                bSplineData.EvalRep = bSplineData.Surface.EvalRepresentation();
                _surfaceData = bSplineData;
            }
            else if (theType == typeof(global::Kerf.Geometry.OffsetSurface))
            {
                _surfaceType = SurfaceKind.OffsetSurface;
                global::Kerf.Geometry.OffsetSurface offSurf = (global::Kerf.Geometry.OffsetSurface)_surface;
                // Populate offset surface data - reuse the original surface for osculating queries
                OffsetData offsetData = new OffsetData();
                offsetData.BasisAdaptor = new SurfaceAdaptor(offSurf.BasisSurface(),
                                                                    _uFirst,
                                                                    _uLast,
                                                                    _vFirst,
                                                                    _vLast,
                                                                    _tolU,
                                                                    _tolV);
                offsetData.OffsetSurface = offSurf;
                offsetData.Offset = offSurf.Offset();
                offsetData.EvalRep = offSurf.EvalRepresentation();
                // Check if equivalent canonical surface exists for faster evaluation
                Surface? equivSurf = offSurf.Surface();
                if (equivSurf is not null)
                {
                    offsetData.EquivalentAdaptor = new SurfaceAdaptor(equivSurf,
                                                                             _uFirst,
                                                                             _uLast,
                                                                             _vFirst,
                                                                             _vLast,
                                                                             _tolU,
                                                                             _tolV);
                }
                _surfaceData = offsetData;
            }
            else
            {
                _surfaceType = OtherSurface;
            }
        }
    }

    //    --
    //    --     Global methods - Apply to the whole Surface.
    //    --

    /// <exception cref="NoSuchObjectException">The surface is not a Geom surface this adaptor knows.</exception>
    public override Continuity UContinuity()
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                NCollection_Array1<double> tk = new(bSpl.InternalUKnots());
                NCollection_Array1<int> tm = new(bSpl.InternalUMultiplicities());
                return LocalContinuity(bSpl.UDegree(), bSpl.UKnotCount, tk, tm, _uFirst, _uLast, IsUPeriodic);
            }
            case SurfaceKind.OffsetSurface:
            {
                switch (BasisSurface().UContinuity())
                {
                    case CN:
                    case C3:
                        return CN;
                    case G2:
                    case C2:
                        return C1;
                    case G1:
                    case C1:
                    case C0:
                        return C0;
                }
                throw new NoSuchObjectException("GeomAdaptor_Surface::UContinuity");
            }
            case SurfaceOfExtrusion:
            {
                return ((ExtrusionData)_surfaceData!).BasisCurve.Continuity();
            }
            case OtherSurface:
                throw new NoSuchObjectException("GeomAdaptor_Surface::UContinuity");
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case SurfaceOfRevolution:
                break;
        }
        return CN;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a Geom surface this adaptor knows.</exception>
    public override Continuity VContinuity()
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                NCollection_Array1<double> tk = new(bSpl.InternalVKnots());
                NCollection_Array1<int> tm = new(bSpl.InternalVMultiplicities());
                return LocalContinuity(bSpl.VDegree(), bSpl.VKnotCount, tk, tm, _vFirst, _vLast, IsVPeriodic);
            }
            case SurfaceKind.OffsetSurface:
            {
                switch (BasisSurface().VContinuity())
                {
                    case CN:
                    case C3:
                        return CN;
                    case G2:
                    case C2:
                        return C1;
                    case G1:
                    case C1:
                    case C0:
                        return C0;
                }
                throw new NoSuchObjectException("GeomAdaptor_Surface::VContinuity");
            }
            case SurfaceOfRevolution:
            {
                return ((RevolutionData)_surfaceData!).BasisCurve.Continuity();
            }
            case OtherSurface:
                throw new NoSuchObjectException("GeomAdaptor_Surface::VContinuity");
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case SurfaceOfExtrusion:
                break;
        }
        return CN;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override int NbUIntervals(Continuity s)
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                CurveAdaptor myBasisCurve = new(bSpl.VIso(bSpl.VKnot(bSpl.FirstVKnotIndex())), _uFirst, _uLast);
                return myBasisCurve.NbIntervals(s);
            }
            case SurfaceOfExtrusion:
            {
                ExtrusionSurface myExtSurf = (ExtrusionSurface)_surface!;
                CurveAdaptor myBasisCurve = new(myExtSurf.BasisCurve(), _uFirst, _uLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    return myBasisCurve.NbIntervals(s);
                }
                break;
            }
            case SurfaceKind.OffsetSurface:
            {
                Continuity baseS = CN;
                switch (s)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::NbUIntervals");
                    case C0:
                        baseS = C1;
                        break;
                    case C1:
                        baseS = C2;
                        break;
                    case C2:
                        baseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                return ((OffsetData)_surfaceData!).BasisAdaptor.NbUIntervals(baseS);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case OtherSurface:
            case SurfaceOfRevolution:
                break;
        }
        return 1;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override int NbVIntervals(Continuity s)
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                CurveAdaptor myBasisCurve = new(bSpl.UIso(bSpl.UKnot(bSpl.FirstUKnotIndex())), _vFirst, _vLast);
                return myBasisCurve.NbIntervals(s);
            }
            case SurfaceOfRevolution:
            {
                RevolutionSurface myRevSurf = (RevolutionSurface)_surface!;
                CurveAdaptor myBasisCurve = new(myRevSurf.BasisCurve(), _vFirst, _vLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    return myBasisCurve.NbIntervals(s);
                }
                break;
            }
            case SurfaceKind.OffsetSurface:
            {
                Continuity baseS = CN;
                switch (s)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::NbVIntervals");
                    case C0:
                        baseS = C1;
                        break;
                    case C1:
                        baseS = C2;
                        break;
                    case C2:
                        baseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                return ((OffsetData)_surfaceData!).BasisAdaptor.NbVIntervals(baseS);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case OtherSurface:
            case SurfaceOfExtrusion:
                break;
        }
        return 1;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override void UIntervals(NCollection_Array1<double> t, Continuity s)
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                CurveAdaptor myBasisCurve = new(bSpl.VIso(bSpl.VKnot(bSpl.FirstVKnotIndex())), _uFirst, _uLast);
                myBasisCurve.Intervals(t, s);
                return;
            }
            case SurfaceOfExtrusion:
            {
                CurveAdaptor myBasisCurve = new(((ExtrusionSurface)_surface!).BasisCurve(),
                                                     _uFirst,
                                                     _uLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    myBasisCurve.Intervals(t, s);
                    return;
                }
                break;
            }
            case SurfaceKind.OffsetSurface:
            {
                Continuity baseS = CN;
                switch (s)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::UIntervals");
                    case C0:
                        baseS = C1;
                        break;
                    case C1:
                        baseS = C2;
                        break;
                    case C2:
                        baseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                ((OffsetData)_surfaceData!).BasisAdaptor.UIntervals(t, baseS);
                return;
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case OtherSurface:
            case SurfaceOfRevolution:
                break;
        }

        t[t.Lower()] = _uFirst;
        t[t.Lower() + 1] = _uLast;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override void VIntervals(NCollection_Array1<double> t, Continuity s)
    {
        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                CurveAdaptor myBasisCurve = new(bSpl.UIso(bSpl.UKnot(bSpl.FirstUKnotIndex())), _vFirst, _vLast);
                myBasisCurve.Intervals(t, s);
                return;
            }
            case SurfaceOfRevolution:
            {
                RevolutionSurface myRevSurf = (RevolutionSurface)_surface!;
                CurveAdaptor myBasisCurve = new(myRevSurf.BasisCurve(), _vFirst, _vLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    myBasisCurve.Intervals(t, s);
                    return;
                }
                break;
            }
            case SurfaceKind.OffsetSurface:
            {
                Continuity baseS = CN;
                switch (s)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::VIntervals");
                    case C0:
                        baseS = C1;
                        break;
                    case C1:
                        baseS = C2;
                        break;
                    case C2:
                        baseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                ((OffsetData)_surfaceData!).BasisAdaptor.VIntervals(t, baseS);
                return;
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceKind.BezierSurface:
            case OtherSurface:
            case SurfaceOfExtrusion:
                break;
        }

        t[t.Lower()] = _vFirst;
        t[t.Lower() + 1] = _vLast;
    }

    /// <summary>A new adaptor of the same surface between First and Last in U.</summary>
    public override SurfaceAdaptorBase UTrim(double first, double last, double tol)
    {
        return new SurfaceAdaptor(_surface!, first, last, _vFirst, _vLast, tol, _tolV);
    }

    /// <summary>A new adaptor of the same surface between First and Last in V.</summary>
    public override SurfaceAdaptorBase VTrim(double first, double last, double tol)
    {
        return new SurfaceAdaptor(_surface!, _uFirst, _uLast, first, last, _tolU, tol);
    }

    /// <summary>True if the surface is U-closed and the adaptor spans its whole U range (a period if periodic).</summary>
    public override bool IsUClosed
    {
        get
        {
            if (!_surface!.IsUClosed)
            {
                return false;
            }

            _surface.Bounds(out double u1, out double u2, out _, out _);
            if (_surface.IsUPeriodic)
            {
                return (Math.Abs(Math.Abs(u1 - u2) - Math.Abs(_uFirst - _uLast)) < global::Kerf.Engine.Foundation.Precision.PConfusion());
            }

            return (Math.Abs(u1 - _uFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(u2 - _uLast) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
    }

    /// <summary>True if the surface is V-closed and the adaptor spans its whole V range (a period if periodic).</summary>
    public override bool IsVClosed
    {
        get
        {
            if (!_surface!.IsVClosed)
            {
                return false;
            }

            _surface.Bounds(out _, out _, out double v1, out double v2);
            if (_surface.IsVPeriodic)
            {
                return (Math.Abs(Math.Abs(v1 - v2) - Math.Abs(_vFirst - _vLast)) < global::Kerf.Engine.Foundation.Precision.PConfusion());
            }

            return (Math.Abs(v1 - _vFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(v2 - _vLast) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
    }

    public override bool IsUPeriodic => (_surface!.IsUPeriodic);

    /// <exception cref="NoSuchObjectException">The surface is not U-periodic.</exception>
    public override double UPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsUPeriodic, " ");
        return _surface!.UPeriod();
    }

    public override bool IsVPeriodic => (_surface!.IsVPeriodic);

    /// <exception cref="NoSuchObjectException">The surface is not V-periodic.</exception>
    public override double VPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsVPeriodic, " ");
        return _surface!.VPeriod();
    }

    private void RebuildCache(double u, double v)
    {
        if (_surfaceType == SurfaceKind.BezierSurface)
        {
            // Create cache for Bezier
            BezierData bezData = (BezierData)_surfaceData!;
            global::Kerf.Geometry.BezierSurface bezier = (global::Kerf.Geometry.BezierSurface)_surface!;
            int degU = bezier.UDegree();
            int degV = bezier.VDegree();
            NCollection_Array1<double> flatKnotsU = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(degU));
            NCollection_Array1<double> flatKnotsV = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(degV));
            if (bezData.Cache is null)
            {
                bezData.Cache = new SurfaceCache(degU,
                                                    bezier.IsUPeriodic,
                                                    flatKnotsU,
                                                    degV,
                                                    bezier.IsVPeriodic,
                                                    flatKnotsV,
                                                    bezier.InternalWeights());
            }
            bezData.Cache.BuildCache(u,
                                      v,
                                      flatKnotsU,
                                      flatKnotsV,
                                      bezier.InternalPoles(),
                                      bezier.InternalWeights());
        }
        else if (_surfaceType == SurfaceKind.BSplineSurface)
        {
            // Create cache for B-spline
            BSplineData bSplData = (BSplineData)_surfaceData!;
            global::Kerf.Geometry.BSplineSurface bSpl = bSplData.Surface;
            if (bSplData.Cache is null)
            {
                bSplData.Cache = new SurfaceCache(bSpl.UDegree(),
                                                     bSpl.IsUPeriodic,
                                                     bSpl.InternalUKnotSequence(),
                                                     bSpl.VDegree(),
                                                     bSpl.IsVPeriodic,
                                                     bSpl.InternalVKnotSequence(),
                                                     bSpl.InternalWeights());
            }
            bSplData.Cache.BuildCache(u,
                                       v,
                                       bSpl.InternalUKnotSequence(),
                                       bSpl.InternalVKnotSequence(),
                                       bSpl.InternalPoles(),
                                       bSpl.InternalWeights());
        }
    }

    /// <exception cref="UndefinedValue3DException">The point is undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override Point3D EvalD0(double u, double v)
    {
        double U = u;
        double V = v;
        Point3D p;
        switch (_surfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Plane3D)_surfaceData!, out p);
                return p;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Cylinder)_surfaceData!, out p);
                return p;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Cone)_surfaceData!, out p);
                return p;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Sphere)_surfaceData!, out p);
                return p;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Torus)_surfaceData!, out p);
                return p;

            case SurfaceKind.BezierSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD0(U, V);
                }
                BezierData bezData = (BezierData)_surfaceData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                bezData.Cache!.D0(U, V, out p);
                return p;
            }
            case SurfaceKind.BSplineSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD0(U, V);
                }
                BSplineData bSplData = (BSplineData)_surfaceData!;
                if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U, V))
                {
                    RebuildCache(U, V);
                }
                bSplData.Cache!.D0(U, V, out p);
                return p;
            }

            case SurfaceOfExtrusion:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD0(U, V);
                }
                ExtrusionData extData = (ExtrusionData)_surfaceData!;
                if (!ExtrusionUtils.D0(U, V, extData.BasisCurve, extData.Direction, out p))
                {
                    throw new UndefinedValue3DException("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return p;
            }

            case SurfaceOfRevolution:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD0(U, V);
                }
                RevolutionData revData = (RevolutionData)_surfaceData!;
                if (!RevolutionUtils.D0(U, V, revData.BasisCurve, revData.Axis, out p))
                {
                    throw new UndefinedValue3DException("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return p;
            }

            case SurfaceKind.OffsetSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD0(U, V);
                }
                OffsetData offData = (OffsetData)_surfaceData!;
                if (!OffsetD0(U, V, offData, out p))
                {
                    throw new UndefinedValue3DException("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return p;
            }

            default:
                return _surface!.EvalD0(U, V);
        }
    }

    // Snaps the parameters within the tolerances of a bound to the bound, and tells which side (1 first, -1 last).
    private void BoundSides(double U, double V, out double u, out double v, out int uSide, out int vSide)
    {
        uSide = 0;
        vSide = 0;
        u = U;
        v = V;
        if (Math.Abs(U - _uFirst) <= _tolU)
        {
            uSide = 1;
            u = _uFirst;
        }
        else if (Math.Abs(U - _uLast) <= _tolU)
        {
            uSide = -1;
            u = _uLast;
        }
        if (Math.Abs(V - _vFirst) <= _tolV)
        {
            vSide = 1;
            v = _vFirst;
        }
        else if (Math.Abs(V - _vLast) <= _tolV)
        {
            vSide = -1;
            v = _vLast;
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override global::Kerf.Geometry.Surface.ResD1 EvalD1(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        global::Kerf.Geometry.Surface.ResD1 result;
        BoundSides(U, V, out double u, out double v, out int uSide, out int vSide);

        switch (_surfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Cylinder)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Cone)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Sphere)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Torus)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                return result;

            case SurfaceKind.BezierSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD1(U, V);
                }
                BezierData bezData = (BezierData)_surfaceData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                bezData.Cache!.D1(U, V, out result.Point, out result.D1U, out result.D1V);
                return result;
            }
            case SurfaceKind.BSplineSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD1(U, V);
                }
                BSplineData bSplData = (BSplineData)_surfaceData!;
                global::Kerf.Geometry.BSplineSurface bSpl = bSplData.Surface;
                int ideb = 0, ifin = 0, iVdeb = 0, iVfin = 0;
                if ((uSide != 0 || vSide != 0) && IfUVBound(u, v, out ideb, out ifin, out iVdeb, out iVfin, uSide, vSide))
                {
                    bSpl.LocalD1(u, v, ideb, ifin, iVdeb, iVfin, out result.Point, out result.D1U, out result.D1V);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U, V))
                    {
                        RebuildCache(U, V);
                    }
                    bSplData.Cache!.D1(U, V, out result.Point, out result.D1U, out result.D1V);
                }
                return result;
            }

            case SurfaceOfExtrusion:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD1(U, V);
                }
                ExtrusionData extData = (ExtrusionData)_surfaceData!;
                if (!ExtrusionUtils.D1(u,
                                            v,
                                            extData.BasisCurve,
                                            extData.Direction,
                                            out result.Point,
                                            out result.D1U,
                                            out result.D1V))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return result;
            }

            case SurfaceOfRevolution:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD1(U, V);
                }
                RevolutionData revData = (RevolutionData)_surfaceData!;
                if (!RevolutionUtils.D1(u,
                                             v,
                                             revData.BasisCurve,
                                             revData.Axis,
                                             out result.Point,
                                             out result.D1U,
                                             out result.D1V))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return result;
            }

            case SurfaceKind.OffsetSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD1(U, V);
                }
                OffsetData offData = (OffsetData)_surfaceData!;
                if (!OffsetD1(u, v, offData, out result))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return result;
            }

            default:
                return _surface!.EvalD1(u, v);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override global::Kerf.Geometry.Surface.ResD2 EvalD2(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        global::Kerf.Geometry.Surface.ResD2 result;
        BoundSides(U, V, out double u, out double v, out int uSide, out int vSide);

        switch (_surfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)_surfaceData!, out result.Point, out result.D1U, out result.D1V);
                result.D2U = new Vector3D(0.0, 0.0, 0.0);
                result.D2V = new Vector3D(0.0, 0.0, 0.0);
                result.D2UV = new Vector3D(0.0, 0.0, 0.0);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Cylinder)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Cone)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Sphere)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Torus)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV);
                return result;

            case SurfaceKind.BezierSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD2(U, V);
                }
                BezierData bezData = (BezierData)_surfaceData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                bezData.Cache!.D2(U,
                                   V,
                                   out result.Point,
                                   out result.D1U,
                                   out result.D1V,
                                   out result.D2U,
                                   out result.D2V,
                                   out result.D2UV);
                return result;
            }
            case SurfaceKind.BSplineSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD2(U, V);
                }
                BSplineData bSplData = (BSplineData)_surfaceData!;
                global::Kerf.Geometry.BSplineSurface bSpl = bSplData.Surface;
                int ideb = 0, ifin = 0, iVdeb = 0, iVfin = 0;
                if ((uSide != 0 || vSide != 0) && IfUVBound(u, v, out ideb, out ifin, out iVdeb, out iVfin, uSide, vSide))
                {
                    bSpl.LocalD2(u,
                                  v,
                                  ideb,
                                  ifin,
                                  iVdeb,
                                  iVfin,
                                  out result.Point,
                                  out result.D1U,
                                  out result.D1V,
                                  out result.D2U,
                                  out result.D2V,
                                  out result.D2UV);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U, V))
                    {
                        RebuildCache(U, V);
                    }
                    bSplData.Cache!.D2(U,
                                        V,
                                        out result.Point,
                                        out result.D1U,
                                        out result.D1V,
                                        out result.D2U,
                                        out result.D2V,
                                        out result.D2UV);
                }
                return result;
            }

            case SurfaceOfExtrusion:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD2(U, V);
                }
                ExtrusionData extData = (ExtrusionData)_surfaceData!;
                if (!ExtrusionUtils.D2(u,
                                            v,
                                            extData.BasisCurve,
                                            extData.Direction,
                                            out result.Point,
                                            out result.D1U,
                                            out result.D1V,
                                            out result.D2U,
                                            out result.D2V,
                                            out result.D2UV))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return result;
            }

            case SurfaceOfRevolution:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD2(U, V);
                }
                RevolutionData revData = (RevolutionData)_surfaceData!;
                if (!RevolutionUtils.D2(u,
                                             v,
                                             revData.BasisCurve,
                                             revData.Axis,
                                             out result.Point,
                                             out result.D1U,
                                             out result.D1V,
                                             out result.D2U,
                                             out result.D2V,
                                             out result.D2UV))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return result;
            }

            case SurfaceKind.OffsetSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD2(U, V);
                }
                OffsetData offData = (OffsetData)_surfaceData!;
                if (!OffsetD2(u, v, offData, out result))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return result;
            }

            default:
                return _surface!.EvalD2(u, v);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override global::Kerf.Geometry.Surface.ResD3 EvalD3(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        global::Kerf.Geometry.Surface.ResD3 result;
        BoundSides(U, V, out double u, out double v, out int uSide, out int vSide);

        switch (_surfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)_surfaceData!, out Point3D plnP, out Vector3D plnD1U, out Vector3D plnD1V);
                result = new global::Kerf.Geometry.Surface.ResD3();
                result.Point = plnP;
                result.D1U = plnD1U;
                result.D1V = plnD1V;
                result.D2U = new Vector3D(0.0, 0.0, 0.0);
                result.D2V = new Vector3D(0.0, 0.0, 0.0);
                result.D2UV = new Vector3D(0.0, 0.0, 0.0);
                result.D3U = new Vector3D(0.0, 0.0, 0.0);
                result.D3V = new Vector3D(0.0, 0.0, 0.0);
                result.D3UUV = new Vector3D(0.0, 0.0, 0.0);
                result.D3UVV = new Vector3D(0.0, 0.0, 0.0);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Cylinder)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV,
                          out result.D3U,
                          out result.D3V,
                          out result.D3UUV,
                          out result.D3UVV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Cone)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV,
                          out result.D3U,
                          out result.D3V,
                          out result.D3UUV,
                          out result.D3UVV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Sphere)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV,
                          out result.D3U,
                          out result.D3V,
                          out result.D3UUV,
                          out result.D3UVV);
                return result;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Torus)_surfaceData!,
                          out result.Point,
                          out result.D1U,
                          out result.D1V,
                          out result.D2U,
                          out result.D2V,
                          out result.D2UV,
                          out result.D3U,
                          out result.D3V,
                          out result.D3UUV,
                          out result.D3UVV);
                return result;

            case SurfaceKind.BSplineSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD3(U, V);
                }
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                if ((uSide == 0) && (vSide == 0))
                {
                    return bSpl.EvalD3(u, v);
                }
                else
                {
                    if (IfUVBound(u, v, out int ideb, out int ifin, out int iVdeb, out int iVfin, uSide, vSide))
                    {
                        bSpl.LocalD3(u,
                                      v,
                                      ideb,
                                      ifin,
                                      iVdeb,
                                      iVfin,
                                      out result.Point,
                                      out result.D1U,
                                      out result.D1V,
                                      out result.D2U,
                                      out result.D2V,
                                      out result.D2UV,
                                      out result.D3U,
                                      out result.D3V,
                                      out result.D3UUV,
                                      out result.D3UVV);
                    }
                    else
                    {
                        return bSpl.EvalD3(u, v);
                    }
                }
                return result;
            }

            case SurfaceOfExtrusion:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD3(U, V);
                }
                ExtrusionData extData = (ExtrusionData)_surfaceData!;
                if (!ExtrusionUtils.D3(u,
                                            v,
                                            extData.BasisCurve,
                                            extData.Direction,
                                            out result.Point,
                                            out result.D1U,
                                            out result.D1V,
                                            out result.D2U,
                                            out result.D2V,
                                            out result.D2UV,
                                            out result.D3U,
                                            out result.D3V,
                                            out result.D3UUV,
                                            out result.D3UVV))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return result;
            }

            case SurfaceOfRevolution:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD3(U, V);
                }
                RevolutionData revData = (RevolutionData)_surfaceData!;
                if (!RevolutionUtils.D3(u,
                                             v,
                                             revData.BasisCurve,
                                             revData.Axis,
                                             out result.Point,
                                             out result.D1U,
                                             out result.D1V,
                                             out result.D2U,
                                             out result.D2V,
                                             out result.D2UV,
                                             out result.D3U,
                                             out result.D3V,
                                             out result.D3UUV,
                                             out result.D3UVV))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return result;
            }

            case SurfaceKind.OffsetSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalD3(U, V);
                }
                OffsetData offData = (OffsetData)_surfaceData!;
                if (!OffsetD3(u, v, offData, out result))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return result;
            }

            default:
                return _surface!.EvalD3(u, v);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">
    /// The derivative is undefined at (theU, theV), or the orders are invalid.
    /// </exception>
    public sealed override Vector3D EvalDN(double theU, double theV, int nu, int nv)
    {
        double U = theU;
        double V = theV;
        int Nu = nu;
        int Nv = nv;
        BoundSides(U, V, out double u, out double v, out int uSide, out int vSide);

        switch (_surfaceType)
        {
            case SurfaceKind.BSplineSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalDN(U, V, Nu, Nv);
                }
                global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
                if ((uSide == 0) && (vSide == 0))
                {
                    return bSpl.EvalDN(u, v, Nu, Nv);
                }
                else
                {
                    if (IfUVBound(u, v, out int ideb, out int ifin, out int iVdeb, out int iVfin, uSide, vSide))
                    {
                        return bSpl.LocalDN(u, v, ideb, ifin, iVdeb, iVfin, Nu, Nv);
                    }
                    else
                    {
                        return bSpl.EvalDN(u, v, Nu, Nv);
                    }
                }
            }

            case SurfaceOfExtrusion:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalDN(U, V, Nu, Nv);
                }
                ExtrusionData extData = (ExtrusionData)_surfaceData!;
                if (!ExtrusionUtils.DN(u, extData.BasisCurve, extData.Direction, Nu, Nv, out Vector3D dn))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return dn;
            }

            case SurfaceOfRevolution:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalDN(U, V, Nu, Nv);
                }
                RevolutionData revData = (RevolutionData)_surfaceData!;
                if (!RevolutionUtils.DN(u, v, revData.BasisCurve, revData.Axis, Nu, Nv, out Vector3D dn))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return dn;
            }

            case SurfaceKind.OffsetSurface:
            {
                if (HasEvalRep(_surfaceData))
                {
                    return _surface!.EvalDN(U, V, Nu, Nv);
                }
                OffsetData offData = (OffsetData)_surfaceData!;
                if (!OffsetDN(u, v, offData, Nu, Nv, out Vector3D dn))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return dn;
            }

            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Plane3D)_surfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Cylinder)_surfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Cone)_surfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Sphere)_surfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Torus)_surfaceData!, Nu, Nv);

            case SurfaceKind.BezierSurface:
            case OtherSurface:
            default:
                break;
        }

        return _surface!.EvalDN(u, v, Nu, Nv);
    }

    /// <summary>The U resolution corresponding to the 3D resolution R3d.</summary>
    public override double UResolution(double r3d)
    {
        double res = 0.0;

        switch (_surfaceType)
        {
            case SurfaceOfExtrusion:
            {
                return ((ExtrusionData)_surfaceData!).BasisCurve.Resolution(r3d);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                ToroidalSurface s = (ToroidalSurface)_surface!;
                double r = s.MajorRadius() + s.MinorRadius();
                if (r > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    res = r3d / (2.0 * r);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                SphericalSurface s = (SphericalSurface)_surface!;
                double r = s.Radius();
                if (r > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    res = r3d / (2.0 * r);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                CylindricalSurface s = (CylindricalSurface)_surface!;
                double r = s.Radius();
                if (r > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    res = r3d / (2.0 * r);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                if (_vLast - _vFirst > 1.0e10)
                {
                    // Not truly bounded => unknown resolution
                    return global::Kerf.Engine.Foundation.Precision.Parametric(r3d);
                }
                ConicalSurface s = (ConicalSurface)_surface!;
                Curve3D c = s.VIso(_vLast);
                double rayon1 = ((CircleCurve3D)c).Radius();
                c = s.VIso(_vFirst);
                double rayon2 = ((CircleCurve3D)c).Radius();
                double r = (rayon1 > rayon2) ? rayon1 : rayon2;
                return (r > global::Kerf.Engine.Foundation.Precision.Confusion() ? (r3d / r) : 0.0);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                return r3d;
            }
            case SurfaceKind.BezierSurface:
            {
                ((global::Kerf.Geometry.BezierSurface)_surface!).Resolution(r3d, out double ures, out _);
                return ures;
            }
            case SurfaceKind.BSplineSurface:
            {
                ((BSplineData)_surfaceData!).Surface.Resolution(r3d, out double ures, out _);
                return ures;
            }
            case SurfaceKind.OffsetSurface:
            {
                return ((OffsetData)_surfaceData!).BasisAdaptor.UResolution(r3d);
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(r3d);
        }

        if (res <= 1.0)
        {
            return 2.0 * Math.Asin(res);
        }

        return 2.0 * Math.PI;
    }

    /// <summary>The V resolution corresponding to the 3D resolution R3d.</summary>
    public override double VResolution(double r3d)
    {
        double res = 0.0;

        switch (_surfaceType)
        {
            case SurfaceOfRevolution:
            {
                return ((RevolutionData)_surfaceData!).BasisCurve.Resolution(r3d);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                ToroidalSurface s = (ToroidalSurface)_surface!;
                double r = s.MinorRadius();
                if (r > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    res = r3d / (2.0 * r);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                SphericalSurface s = (SphericalSurface)_surface!;
                double r = s.Radius();
                if (r > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    res = r3d / (2.0 * r);
                }
                break;
            }
            case SurfaceOfExtrusion:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                return r3d;
            }
            case SurfaceKind.BezierSurface:
            {
                ((global::Kerf.Geometry.BezierSurface)_surface!).Resolution(r3d, out _, out double vres);
                return vres;
            }
            case SurfaceKind.BSplineSurface:
            {
                ((BSplineData)_surfaceData!).Surface.Resolution(r3d, out _, out double vres);
                return vres;
            }
            case SurfaceKind.OffsetSurface:
            {
                return ((OffsetData)_surfaceData!).BasisAdaptor.VResolution(r3d);
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(r3d);
        }

        if (res <= 1.0)
        {
            return 2.0 * Math.Asin(res);
        }

        return 2.0 * Math.PI;
    }

    /// <summary>The type of the surface (GeomAbs_OtherSurface before a Load).</summary>
    public override SurfaceKind GetType() => _surfaceType;

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        if (_surfaceType != global::Kerf.Geometry.SurfaceKind.Plane)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Plane");
        }
        return (Plane3D)_surfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder Cylinder()
    {
        if (_surfaceType != global::Kerf.Geometry.SurfaceKind.Cylinder)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Cylinder");
        }
        return (Cylinder)_surfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cone.</exception>
    public override Cone Cone()
    {
        if (_surfaceType != global::Kerf.Geometry.SurfaceKind.Cone)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Cone");
        }
        return (Cone)_surfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a sphere.</exception>
    public override Sphere Sphere()
    {
        if (_surfaceType != global::Kerf.Geometry.SurfaceKind.Sphere)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Sphere");
        }
        return (Sphere)_surfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a torus.</exception>
    public override Torus Torus()
    {
        if (_surfaceType != global::Kerf.Geometry.SurfaceKind.Torus)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Torus");
        }
        return (Torus)_surfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or extrusion surface.</exception>
    public override int UDegree()
    {
        if (_surfaceType == SurfaceKind.BSplineSurface)
        {
            return ((BSplineData)_surfaceData!).Surface.UDegree();
        }
        if (_surfaceType == SurfaceKind.BezierSurface)
        {
            return ((global::Kerf.Geometry.BezierSurface)_surface!).UDegree();
        }
        if (_surfaceType == SurfaceOfExtrusion)
        {
            return ((ExtrusionData)_surfaceData!).BasisCurve.Degree();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::UDegree");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or extrusion surface.</exception>
    public override int UPoleCount
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.UPoleCount;
            }
            if (_surfaceType == SurfaceKind.BezierSurface)
            {
                return ((global::Kerf.Geometry.BezierSurface)_surface!).UPoleCount;
            }
            if (_surfaceType == SurfaceOfExtrusion)
            {
                return ((ExtrusionData)_surfaceData!).BasisCurve.PoleCount;
            }
            throw new NoSuchObjectException("GeomAdaptor_Surface::NbUPoles");
        }
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or revolution surface.</exception>
    public override int VDegree()
    {
        if (_surfaceType == SurfaceKind.BSplineSurface)
        {
            return ((BSplineData)_surfaceData!).Surface.VDegree();
        }
        if (_surfaceType == SurfaceKind.BezierSurface)
        {
            return ((global::Kerf.Geometry.BezierSurface)_surface!).VDegree();
        }
        if (_surfaceType == SurfaceOfRevolution)
        {
            return ((RevolutionData)_surfaceData!).BasisCurve.Degree();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::VDegree");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or revolution surface.</exception>
    public override int VPoleCount
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.VPoleCount;
            }
            if (_surfaceType == SurfaceKind.BezierSurface)
            {
                return ((global::Kerf.Geometry.BezierSurface)_surface!).VPoleCount;
            }
            if (_surfaceType == SurfaceOfRevolution)
            {
                return ((RevolutionData)_surfaceData!).BasisCurve.PoleCount;
            }
            throw new NoSuchObjectException("GeomAdaptor_Surface::NbVPoles");
        }
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline or extrusion surface.</exception>
    public override int UKnotCount
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.UKnotCount;
            }
            if (_surfaceType == SurfaceOfExtrusion)
            {
                return ((ExtrusionData)_surfaceData!).BasisCurve.KnotCount;
            }
            throw new NoSuchObjectException("GeomAdaptor_Surface::NbUKnots");
        }
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline surface.</exception>
    public override int VKnotCount
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.VKnotCount;
            }
            throw new NoSuchObjectException("GeomAdaptor_Surface::NbVKnots");
        }
    }

    /// <summary>True for a B-spline or Bezier surface rational in U; false for any other surface.</summary>
    public override bool IsURational
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.IsURational;
            }
            if (_surfaceType == SurfaceKind.BezierSurface)
            {
                return ((global::Kerf.Geometry.BezierSurface)_surface!).IsURational;
            }
            return false;
        }
    }

    /// <summary>True for a B-spline or Bezier surface rational in V; false for any other surface.</summary>
    public override bool IsVRational
    {
        get
        {
            if (_surfaceType == SurfaceKind.BSplineSurface)
            {
                return ((BSplineData)_surfaceData!).Surface.IsVRational;
            }
            if (_surfaceType == SurfaceKind.BezierSurface)
            {
                return ((global::Kerf.Geometry.BezierSurface)_surface!).IsVRational;
            }
            return false;
        }
    }

    /// <summary>The Bezier surface (shared).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a Bezier surface.</exception>
    public override global::Kerf.Geometry.BezierSurface Bezier()
    {
        if (_surfaceType != SurfaceKind.BezierSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Bezier");
        }
        return (global::Kerf.Geometry.BezierSurface)_surface!;
    }

    /// <summary>The B-spline surface (shared).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a B-spline surface.</exception>
    public override global::Kerf.Geometry.BSplineSurface BSpline()
    {
        if (_surfaceType != SurfaceKind.BSplineSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BSpline");
        }
        return ((BSplineData)_surfaceData!).Surface;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a surface of revolution.</exception>
    public override Axis3D AxeOfRevolution()
    {
        if (_surfaceType != SurfaceOfRevolution)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::AxeOfRevolution");
        }
        return ((RevolutionSurface)_surface!).Axis();
    }

    /// <exception cref="NoSuchObjectException">The surface is not a surface of linear extrusion.</exception>
    public override Direction3D Direction()
    {
        if (_surfaceType != SurfaceOfExtrusion)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Direction");
        }
        return ((ExtrusionSurface)_surface!).Direction();
    }

    /// <summary>A new adaptor of the basis curve of a swept surface (over the whole curve).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a swept surface.</exception>
    public override CurveAdaptorBase BasisCurve()
    {
        Curve3D c;
        if (_surfaceType == SurfaceOfExtrusion)
        {
            c = ((ExtrusionSurface)_surface!).BasisCurve();
        }
        else if (_surfaceType == SurfaceOfRevolution)
        {
            c = ((RevolutionSurface)_surface!).BasisCurve();
        }
        else
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisCurve");
        }
        return new CurveAdaptor(c);
    }

    /// <summary>A new adaptor of the basis surface of an offset surface, over this adaptor's bounds.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not an offset surface.</exception>
    public override SurfaceAdaptorBase BasisSurface()
    {
        if (_surfaceType != SurfaceKind.OffsetSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisSurface");
        }
        return new SurfaceAdaptor(((global::Kerf.Geometry.OffsetSurface)_surface!).BasisSurface(),
                                       _uFirst,
                                       _uLast,
                                       _vFirst,
                                       _vLast);
    }

    /// <exception cref="NoSuchObjectException">The surface is not an offset surface.</exception>
    public override double OffsetValue()
    {
        if (_surfaceType != SurfaceKind.OffsetSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisSurface");
        }
        return ((global::Kerf.Geometry.OffsetSurface)_surface!).Offset();
    }

    // Locates U, V if they are a First or Last bound, processes the found span and returns the parameters for
    // LocalDi.
    private bool IfUVBound(double u,
                           double v,
                           out int iOutDeb,
                           out int iOutFin,
                           out int iOutVDeb,
                           out int iOutVFin,
                           int uSide,
                           int vSide)
    {
        global::Kerf.Geometry.BSplineSurface bSpl = ((BSplineData)_surfaceData!).Surface;
        int ideb, ifin;
        int ufkIndx = bSpl.FirstUKnotIndex(), ulkIndx = bSpl.LastUKnotIndex(),
            vfkIndx = bSpl.FirstVKnotIndex(), vlkIndx = bSpl.LastVKnotIndex();
        bSpl.LocateU(u, PosTol, out ideb, out ifin, false);
        bool local = (ideb == ifin);
        Span(uSide, ideb, ifin, out ideb, out ifin, ufkIndx, ulkIndx);
        int iVdeb, iVfin;
        bSpl.LocateV(v, PosTol, out iVdeb, out iVfin, false);
        if (iVdeb == iVfin)
        {
            local = true;
        }
        Span(vSide, iVdeb, iVfin, out iVdeb, out iVfin, vfkIndx, vlkIndx);

        iOutDeb = ideb;
        iOutFin = ifin;
        iOutVDeb = iVdeb;
        iOutVFin = iVfin;

        return local;
    }

    // Locates U, V if U = UFirst or U = ULast, processes the found span and returns the parameters for LocalDi.
    private static void Span(int side, int ideb, int ifin, out int outIdeb, out int outIfin, int fkIndx,
                             int lkIndx)
    {
        if (ideb != ifin) // not a knot
        {
            if (ideb < fkIndx)
            {
                outIdeb = fkIndx;
                outIfin = fkIndx + 1;
            }
            else if (ifin > lkIndx)
            {
                outIdeb = lkIndx - 1;
                outIfin = lkIndx;
            }
            else if (ideb >= (lkIndx - 1))
            {
                outIdeb = lkIndx - 1;
                outIfin = lkIndx;
            }
            else if (ifin <= fkIndx + 1)
            {
                outIdeb = fkIndx;
                outIfin = fkIndx + 1;
            }
            else if (ideb > ifin)
            {
                outIdeb = ifin - 1;
                outIfin = ifin;
            }
            else
            {
                outIdeb = ideb;
                outIfin = ifin;
            }
        }
        else
        {
            if (ideb <= fkIndx)
            {
                outIdeb = fkIndx;
                outIfin = fkIndx + 1;
            } // first knot
            else if (ifin >= lkIndx)
            {
                outIdeb = lkIndx - 1;
                outIfin = lkIndx;
            } // last knot
            else
            {
                if (side == -1)
                {
                    outIdeb = ideb - 1;
                    outIfin = ifin;
                }
                else
                {
                    outIdeb = ideb;
                    outIfin = ifin + 1;
                }
            }
        }
    }
}
