// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The surface of revolution of an adapted curve about an axis: U in [0, 2*PI] is the angle, V the curve's
/// parameter. <see cref="GetType"/> recognises planes, cylinders, cones, spheres and tori.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_RevolutionUtils over the curve adaptor).</remarks>
internal class RevolutionSurfaceAdaptor : SurfaceAdaptor
{
    private CurveAdaptorBase? _basisCurve; // revolved curve
    private Axis3D _axis; // axis of revolution
    private bool _haveAxis; // whether axis of revolution is initialized
    private CoordinateSystem _axeRev; // auxiliary trihedron according to the curve position

    /// <summary>An adaptor without a curve or axis.</summary>
    public RevolutionSurfaceAdaptor()
    {
        _haveAxis = false;
        _axeRev = new CoordinateSystem();
    }

    /// <summary>The revolution of C, the axis to be loaded.</summary>
    public RevolutionSurfaceAdaptor(CurveAdaptorBase c)
    {
        _haveAxis = false;
        _axeRev = new CoordinateSystem();
        Load(c);
    }

    /// <summary>The revolution of C about V.</summary>
    /// <exception cref="ConstructionErrorException">The axis and the meridian are confused.</exception>
    public RevolutionSurfaceAdaptor(CurveAdaptorBase c, in Axis3D v)
    {
        _haveAxis = false;
        _axeRev = new CoordinateSystem();
        Load(c);
        Load(v);
    }

    /// <summary>A shallow copy: shallow copies of the curve adaptors.</summary>
    public override SurfaceAdaptorBase ShallowCopy()
    {
        RevolutionSurfaceAdaptor copy = new RevolutionSurfaceAdaptor();

        if (_basisCurve is not null)
        {
            copy._basisCurve = _basisCurve.ShallowCopy();
        }
        copy._axis = _axis;
        copy._haveAxis = _haveAxis;
        copy._axeRev = _axeRev;

        copy._surface = _surface;
        copy._uFirst = _uFirst;
        copy._uLast = _uLast;
        copy._vFirst = _vFirst;
        copy._vLast = _vLast;
        copy._tolU = _tolU;
        copy._tolV = _tolV;
        copy._surfaceType = _surfaceType;

        // Copy surface data variant
        if (_surfaceData is RevolutionData revData)
        {
            RevolutionData newData = new RevolutionData();
            newData.BasisCurve = revData.BasisCurve.ShallowCopy();
            newData.Axis = revData.Axis;
            copy._surfaceData = newData;
        }

        return copy;
    }

    /// <summary>Changes the revolved curve (and recomputes the trihedron if the axis is loaded).</summary>
    public void Load(CurveAdaptorBase c)
    {
        _basisCurve = c;
        if (_haveAxis)
        {
            Load(_axis); // to evaluate the new myAxeRev.
        }
    }

    /// <summary>Changes the axis of revolution (the curve must be loaded).</summary>
    /// <exception cref="ConstructionErrorException">The axis and the meridian are confused.</exception>
    public void Load(in Axis3D v)
    {
        _haveAxis = true;
        _axis = v;

        _surfaceType = SurfaceOfRevolution;

        // Populate revolution surface data for fast evaluation
        RevolutionData revData = new RevolutionData();
        revData.BasisCurve = _basisCurve!;
        revData.Axis = _axis;
        _surfaceData = revData;

        // Eval myAxeRev : axe of revolution ( Determination de Ox).
        Point3D p, q;
        Point3D o = _axis.Location;
        Direction3D ox;
        Direction3D oz = _axis.Direction;
        bool yrev = false;
        if (_basisCurve!.GetType() == Line)
        {
            if ((_basisCurve.Line().Direction).Dot(oz) < 0.0)
            {
                yrev = true;
                oz = oz.Reversed();
            }
        }

        if (_basisCurve.GetType() == Circle)
        {
            q = p = (_basisCurve.Circle()).Location;
        }
        else
        {
            double first = _basisCurve.FirstParameter();
            p = Value(0.0, 0.0); // which does not mean much
            if (GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                if (new Line3D(_axis).DistanceTo(p) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    q = global::Kerf.Engine.Geometry.ElCLib.Value(1.0, _basisCurve.Line());
                }
                else
                {
                    q = p;
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(first))
            {
                q = p;
            }
            else
            {
                q = Value(0.0, first);
            }
        }

        Direction3D dz = _axis.Direction;
        o = o.WithXYZ(o.XYZ() + (new Vector3D(o, p) * new Vector3D(dz)) * dz.XYZ());
        if (new Line3D(_axis).DistanceTo(q) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ox = new Direction3D(q.XYZ() - o.XYZ());
        }
        else
        {
            double first = _basisCurve.FirstParameter();
            double last = _basisCurve.LastParameter();
            int ratio = 1;
            double dist;
            Point3D pp;
            do
            {
                pp = _basisCurve.Value(first + (last - first) / ratio);
                dist = new Line3D(_axis).DistanceTo(pp);
                ratio++;
            } while (dist < global::Kerf.Engine.Foundation.Precision.Confusion() && ratio < 100);

            if (ratio >= 100)
            {
                throw new ConstructionErrorException("Adaptor3d_SurfaceOfRevolution : Axe and meridian are confused");
            }
            // C++: (Oz ^ gp_Vec(...)) ^ Oz, with the vector converted to a gp_Dir (normalised), and gp_Dir's
            // operator^ normalising each product.
            ox = (oz ^ new Direction3D(new Vector3D(pp.XYZ() - o.XYZ()))) ^ oz;
        }

        _axeRev = new CoordinateSystem(o, oz, ox);

        if (yrev)
        {
            _axeRev = _axeRev.YReversed();
        }
        else if (_basisCurve.GetType() == Circle)
        {
            Direction3D dc = (_basisCurve.Circle()).Axis.Direction;
            if ((ox.Crossed(oz)).Dot(dc) < 0.0)
            {
                _axeRev = _axeRev.ZReversed();
            }
        }
    }

    public override Axis3D AxeOfRevolution() => _axis;

    public override double FirstUParameter() => 0.0;

    public override double LastUParameter() => 2 * Math.PI;

    public override double FirstVParameter() => _basisCurve!.FirstParameter();

    public override double LastVParameter() => _basisCurve!.LastParameter();

    public override Continuity UContinuity() => CN;

    public override Continuity VContinuity() => _basisCurve!.Continuity();

    public override int NbUIntervals(Continuity s) => 1;

    public override int NbVIntervals(Continuity s) => _basisCurve!.NbIntervals(s);

    public override void UIntervals(NCollection_Array1<double> t, Continuity s)
    {
        t[t.Lower()] = 0.0;
        t[t.Lower() + 1] = 2 * Math.PI;
    }

    public override void VIntervals(NCollection_Array1<double> t, Continuity s) => _basisCurve!.Intervals(t, s);

    /// <summary>The whole surface again: only [0, 2*PI] is accepted.</summary>
    /// <exception cref="OutOfRangeException">First or Last differ from 0 or 2*PI by more than Precision::PConfusion().</exception>
    public override SurfaceAdaptorBase UTrim(double first, double last, double tol)
    {
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Standard_OutOfRange_Raise_if(Math.Abs(first) > eps || Math.Abs(last - 2.0 * Math.PI) > eps,
                                     "GeomAdaptor_SurfaceOfRevolution : UTrim : Parameters out of range");

        RevolutionSurfaceAdaptor hr = new RevolutionSurfaceAdaptor(_basisCurve!, _axis);
        return hr;
    }

    /// <summary>The revolution of the curve trimmed between First and Last.</summary>
    public override SurfaceAdaptorBase VTrim(double first, double last, double tol)
    {
        CurveAdaptorBase hc = BasisCurve().Trim(first, last, tol);
        RevolutionSurfaceAdaptor hr = new RevolutionSurfaceAdaptor(hc, _axis);
        return hr;
    }

    public override bool IsUClosed => true;

    public override bool IsVClosed => _basisCurve!.IsClosed;

    public override bool IsUPeriodic => true;

    public override double UPeriod() => 2 * Math.PI;

    public override bool IsVPeriodic => _basisCurve!.IsPeriodic;

    public override double VPeriod() => _basisCurve!.Period();

    public override double UResolution(double r3d) => global::Kerf.Engine.Foundation.Precision.Parametric(r3d);

    public override double VResolution(double r3d) => _basisCurve!.Resolution(r3d);

    /// <summary>
    /// GeomAbs_Plane, Cylinder or Cone for a line (normal to, parallel to or crossing the axis), GeomAbs_Sphere or
    /// Torus for a circle coplanar with the axis; GeomAbs_SurfaceOfRevolution otherwise.
    /// </summary>
    public override SurfaceKind GetType()
    {
        double tolConf = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolAng = global::Kerf.Engine.Foundation.Precision.Angular();
        double tolConeSemiAng = global::Kerf.Engine.Foundation.Precision.Confusion();

        switch (_basisCurve!.GetType())
        {
            case Line:
            {
                Axis3D axe = _basisCurve.Line().Position;

                if (_axis.IsParallel(axe, tolAng))
                {
                    Point3D p = Value(0.0, 0.0);
                    double R = new Vector3D(_axeRev.Location, p) * new Vector3D(_axeRev.XDirection);
                    if (R > tolConf)
                    {
                        return global::Kerf.Geometry.SurfaceKind.Cylinder;
                    }
                }
                else if (_axis.IsNormal(axe, tolAng))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                else
                {
                    double uf = _basisCurve.FirstParameter();
                    double ul = _basisCurve.LastParameter();
                    bool istrim = (!global::Kerf.Engine.Foundation.Precision.IsInfinite(uf) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(ul));
                    if (istrim)
                    {
                        Point3D pf = _basisCurve.Value(uf);
                        Point3D pl = _basisCurve.Value(ul);
                        double len = pf.DistanceTo(pl);
                        // Compute the distance projected onto the axis.
                        Vector3D vlin = new Vector3D(pf, pl);
                        Vector3D vaxe = new Vector3D(_axis.Direction);
                        double projlen = Math.Abs(vaxe.Dot(vlin));
                        if ((len - projlen) <= tolConf)
                        {
                            Point3D p = Value(0.0, 0.0);
                            double R = new Vector3D(_axeRev.Location, p) * new Vector3D(_axeRev.XDirection);
                            if (R > tolConf)
                            {
                                return global::Kerf.Geometry.SurfaceKind.Cylinder;
                            }
                        }
                        else if (projlen <= tolConf)
                        {
                            return global::Kerf.Geometry.SurfaceKind.Plane;
                        }
                    }
                    Vector3D v = new Vector3D(_axis.Location, _basisCurve.Line().Location);
                    Vector3D w = new Vector3D(axe.Direction);
                    Vector3D axisDir = new Vector3D(_axis.Direction);
                    double proj = Math.Abs(w.Dot(axisDir));
                    if (Math.Abs(v.DotCross(axisDir, w)) <= tolConf
                        && (proj >= tolConeSemiAng && proj <= 1.0 - tolConeSemiAng))
                    {
                        return global::Kerf.Geometry.SurfaceKind.Cone;
                    }
                }
                break;
            } // case GeomAbs_Line:
            //
            case Circle:
            {
                double majorRadius, r;
                Line3D lin = new Line3D(_axis);
                //
                Circle3D c = _basisCurve.Circle();
                Point3D lc = c.Location;
                r = c.Radius;
                //

                if (!c.Position.IsCoplanar(_axis, tolConf, tolAng))
                {
                    return SurfaceOfRevolution;
                }
                else if (lin.DistanceTo(lc) <= tolConf)
                {
                    return global::Kerf.Geometry.SurfaceKind.Sphere;
                }
                else
                {
                    majorRadius = lin.DistanceTo(lc);
                    // OCCT's bare comparison makes a torus of a circle that touches the axis to within the tolerance, whose
                    // end vertex is then taken as on the axis too: an invalid solid (OCCT #1566, decision 20206)
                    if (majorRadius > r + tolConf)
                    {
                        return global::Kerf.Geometry.SurfaceKind.Torus;
                    }
                }
                break;
            }
            //
            default:
                break;
        }

        return SurfaceOfRevolution;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "GeomAdaptor_SurfaceOfRevolution:Plane");

        CoordinateSystem axe = _axeRev;
        Point3D ponCurve = Value(0.0, 0.0);
        double dot = (ponCurve.XYZ() - _axis.Location.XYZ()).Dot(_axis.Direction.XYZ());

        Point3D p = new Point3D(_axis.Location.XYZ() + dot * _axis.Direction.XYZ());
        axe = axe.WithLocation(p);
        if (axe.XDirection.Dot(_basisCurve!.Line().Direction) >= -global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            axe = axe.XReversed();
        }

        return new Plane3D(axe);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder3D Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cylinder, "GeomAdaptor_SurfaceOfRevolution::Cylinder");

        Point3D p = Value(0.0, 0.0);
        double r = new Vector3D(_axeRev.Location, p) * new Vector3D(_axeRev.XDirection);
        return new Cylinder3D(_axeRev, r);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cone.</exception>
    public override Cone3D Cone()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cone, "GeomAdaptor_SurfaceOfRevolution:Cone");

        CoordinateSystem axe = _axeRev;
        Direction3D ldir = (_basisCurve!.Line()).Direction;
        double angle = (axe.Direction).AngleTo(ldir);
        Point3D p0 = Value(0.0, 0.0);
        double r = (axe.Location).DistanceTo(p0);
        if (r >= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Point3D o = axe.Location;
            Vector3D op0 = new Vector3D(o, p0);
            double t = op0.Dot(new Vector3D(axe.XDirection));
            t /= ldir.Dot(axe.XDirection);
            op0 = op0.Added(-t * new Vector3D(ldir));
            if (op0.Dot(new Vector3D(axe.Direction)) > 0.0)
            {
                angle = -angle;
            }
        }
        return new Cone3D(axe, angle, r);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a sphere.</exception>
    public override Sphere3D Sphere()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Sphere, "GeomAdaptor_SurfaceOfRevolution:Sphere");

        Circle3D c = _basisCurve!.Circle();
        CoordinateSystem axe = _axeRev;
        axe = axe.WithLocation(c.Location);
        return new Sphere3D(axe, c.Radius);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a torus.</exception>
    public override Torus3D Torus()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Torus, "GeomAdaptor_SurfaceOfRevolution:Torus");

        Circle3D c = _basisCurve!.Circle();
        double majorRadius = new Line3D(_axis).DistanceTo(c.Location);
        return new Torus3D(_axeRev, majorRadius, c.Radius);
    }

    public override int VDegree() => _basisCurve!.Degree();

    public override int VPoleCount => _basisCurve!.PoleCount;

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override int VKnotCount => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::NbVKnots");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsURational =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::IsURational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsVRational =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::IsVRational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override global::Kerf.Geometry.BezierSurface Bezier() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::Bezier");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override global::Kerf.Geometry.BSplineSurface BSpline() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::BSpline");

    /// <summary>The trihedron of the revolution: the axis, with X towards the curve.</summary>
    public CoordinateSystem Axis() => _axeRev;

    /// <summary>The revolved curve (the adaptor given to Load).</summary>
    public override CurveAdaptorBase BasisCurve() => _basisCurve!;
}
