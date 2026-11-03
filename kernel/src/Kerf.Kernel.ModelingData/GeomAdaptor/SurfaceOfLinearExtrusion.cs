// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The surface of linear extrusion of an adapted curve along a direction: U is the curve's parameter, V the
/// distance along the direction (unbounded). <see cref="GetType"/> recognises planes and cylinders.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_ExtrusionUtils over the curve adaptor).</remarks>
public class SurfaceOfLinearExtrusion : SurfaceAdaptor
{
    private CurveAdaptorBase? _basisCurve; // extruded curve
    private Direction3D _direction; // direction of extrusion
    private bool _haveDir; // whether the direction of extrusion is initialized

    /// <summary>An adaptor without a curve or direction.</summary>
    public SurfaceOfLinearExtrusion()
    {
        _direction = new Direction3D();
        _haveDir = false;
    }

    /// <summary>The extrusion of C, the direction to be loaded.</summary>
    public SurfaceOfLinearExtrusion(CurveAdaptorBase c)
    {
        _direction = new Direction3D();
        _haveDir = false;
        Load(c);
    }

    /// <summary>The extrusion of C along V.</summary>
    public SurfaceOfLinearExtrusion(CurveAdaptorBase c, in Direction3D v)
    {
        _direction = new Direction3D();
        _haveDir = false;
        Load(c);
        Load(v);
    }

    /// <summary>A shallow copy: shallow copies of the curve adaptors.</summary>
    public override SurfaceAdaptorBase ShallowCopy()
    {
        SurfaceOfLinearExtrusion copy = new SurfaceOfLinearExtrusion();

        if (_basisCurve is not null)
        {
            copy._basisCurve = _basisCurve.ShallowCopy();
        }
        copy._direction = _direction;
        copy._haveDir = _haveDir;

        copy._surface = _surface;
        copy._uFirst = _uFirst;
        copy._uLast = _uLast;
        copy._vFirst = _vFirst;
        copy._vLast = _vLast;
        copy._tolU = _tolU;
        copy._tolV = _tolV;
        copy._surfaceType = _surfaceType;

        // Copy surface data variant
        if (_surfaceData is ExtrusionData extData)
        {
            ExtrusionData newData = new ExtrusionData();
            newData.BasisCurve = extData.BasisCurve.ShallowCopy();
            newData.Direction = extData.Direction;
            copy._surfaceData = newData;
        }

        return copy;
    }

    /// <summary>Changes the extruded curve.</summary>
    public void Load(CurveAdaptorBase c)
    {
        _basisCurve = c;
        if (_haveDir)
        {
            Load(_direction);
        }
    }

    /// <summary>Changes the direction of extrusion (the curve must be loaded).</summary>
    public void Load(in Direction3D v)
    {
        _haveDir = true;
        _direction = v;

        _surfaceType = SurfaceOfExtrusion;

        // Populate extrusion surface data for fast evaluation
        ExtrusionData extData = new ExtrusionData();
        extData.BasisCurve = _basisCurve!;
        extData.Direction = _direction.XYZ();
        _surfaceData = extData;
    }

    public override double FirstUParameter() => _basisCurve!.FirstParameter();

    public override double LastUParameter() => _basisCurve!.LastParameter();

    public override double FirstVParameter() => RealFirst();

    public override double LastVParameter() => RealLast();

    public override Continuity UContinuity() => _basisCurve!.Continuity();

    public override Continuity VContinuity() => CN;

    public override int NbUIntervals(Continuity s) => _basisCurve!.NbIntervals(s);

    public override int NbVIntervals(Continuity s) => 1;

    public override void UIntervals(NCollection_Array1<double> t, Continuity s) => _basisCurve!.Intervals(t, s);

    public override void VIntervals(NCollection_Array1<double> t, Continuity s)
    {
        t[t.Lower()] = FirstVParameter();
        t[t.Lower() + 1] = LastVParameter();
    }

    /// <summary>
    /// The extrusion of the curve trimmed between First and Last: OCCT trims the curve, which is the U direction
    /// (occt-issues.md).
    /// </summary>
    public override SurfaceAdaptorBase VTrim(double first, double last, double tol)
    {
        // OCCT: VTrim trims the basis curve, which carries U, and UTrim (below) trims nothing.
        CurveAdaptorBase hc = BasisCurve().Trim(first, last, tol);
        SurfaceOfLinearExtrusion hr = new SurfaceOfLinearExtrusion(hc, _direction);
        return hr;
    }

    /// <summary>The same extrusion, untrimmed (OCCT ignores the parameters: occt-issues.md).</summary>
    public override SurfaceAdaptorBase UTrim(double first, double last, double tol)
    {
        SurfaceOfLinearExtrusion hr = new SurfaceOfLinearExtrusion(_basisCurve!, _direction);
        return hr;
    }

    public override bool IsUClosed => _basisCurve!.IsClosed;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => _basisCurve!.IsPeriodic;

    public override double UPeriod() => _basisCurve!.Period();

    public override bool IsVPeriodic => false;

    /// <exception cref="DomainErrorException">Always (not periodic in V).</exception>
    public override double VPeriod() => throw new DomainErrorException("GeomAdaptor_SurfaceOfLinearExtrusion::VPeriod");

    public override double UResolution(double r3d) => _basisCurve!.Resolution(r3d);

    public override double VResolution(double r3d) => r3d;

    /// <summary>
    /// GeomAbs_Plane for an extruded line not parallel to the direction, or a conic whose plane contains it;
    /// GeomAbs_Cylinder for a circle extruded along its axis; GeomAbs_SurfaceOfExtrusion otherwise.
    /// </summary>
    public override SurfaceKind GetType()
    {
        switch (_basisCurve!.GetType())
        {
            case Line:
            {
                Direction3D d = _basisCurve.Line().Direction;
                if (!_direction.IsParallel(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Circle:
            {
                Direction3D d = (_basisCurve.Circle()).Axis.Direction;
                if (_direction.IsParallel(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Cylinder;
                }
                else if (_direction.IsNormal(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Ellipse:
            {
                Direction3D d = (_basisCurve.Ellipse()).Axis.Direction;
                if (_direction.IsNormal(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Parabola:
            {
                Direction3D d = (_basisCurve.Parabola()).Axis.Direction;
                if (_direction.IsNormal(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Hyperbola:
            {
                Direction3D d = (_basisCurve.Hyperbola()).Axis.Direction;
                if (_direction.IsNormal(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            default:
                break;
        }

        return SurfaceOfExtrusion;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "GeomAdaptor_SurfaceOfLinearExtrusion::Plane");

        Point3D p = new Point3D();
        Vector3D d1u = new Vector3D(), newZ = new Vector3D();
        double uFirst = _basisCurve!.FirstParameter();
        double uLast = _basisCurve.LastParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uFirst) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uLast))
        {
            uFirst = -100.0;
            uLast = 100.0;
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uFirst))
        {
            uFirst = uLast - 200.0;
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uLast))
        {
            uLast = uFirst + 200.0;
        }
        double deltau = (uLast - uFirst) / 20.0;
        for (int i = 1; i <= 21; i++)
        {
            double prm = uFirst + (i - 1) * deltau;
            _basisCurve.D1(prm, out p, out d1u);
            newZ = d1u.Normalized().Crossed(new Vector3D(_direction));
            if (newZ.Magnitude > 1.0e-12)
            {
                break;
            }
        }
        CoordinateSystem ax3 = new CoordinateSystem(p, new Direction3D(newZ), new Direction3D(d1u));
        if (_direction.Dot(ax3.YDirection) < 0.0)
        {
            ax3 = ax3.YReversed();
        }
        return new Plane3D(ax3);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cylinder, "GeomAdaptor_SurfaceOfLinearExtrusion::Cylinder");

        Circle3D c = _basisCurve!.Circle();
        CoordinateSystem ax3 = new CoordinateSystem(c.Position);
        if (_direction.Dot((c.Axis).Direction) < 0.0)
        {
            ax3 = ax3.ZReversed();
        }
        return new Cylinder(ax3, c.Radius);
    }

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Cone Cone() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Cone");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Sphere Sphere() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Sphere");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Torus Torus() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Torus");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Axis3D AxeOfRevolution() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Axes");

    public override int UDegree() => _basisCurve!.Degree();

    public override int UPoleCount => _basisCurve!.PoleCount;

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsURational =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::IsURational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsVRational =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::IsVRational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override global::Kerf.Geometry.BezierSurface Bezier() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Bezier");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override global::Kerf.Geometry.BSplineSurface BSpline() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::BSpline");

    public override Direction3D Direction() => _direction;

    /// <summary>The extruded curve (the adaptor given to Load).</summary>
    public override CurveAdaptorBase BasisCurve() => _basisCurve!;
}
