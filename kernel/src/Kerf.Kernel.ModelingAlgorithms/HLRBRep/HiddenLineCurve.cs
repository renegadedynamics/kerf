// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// An edge's 3D curve seen through a projector: the 2D curve of its projection (points, derivatives, conic types) and
/// the 3D curve it comes from, with the conversions between the parameters on both.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (decision 1092): HLRBRep_EdgeData holds one by value and HLRBRep_Data updates it.
/// OCCT keeps a pointer to HLRBRep_Data's projector; the port keeps the box that holds it
/// (<see cref="StrongBox{T}"/>), so the projector set later by <c>HLRBRep_Data.Update</c> is the one every curve sees
/// (decision 1093). <see cref="Copy"/> is OCCT's implicit copy (the adaptor copied, the projector shared).
/// OCCT's <see cref="D3"/> writes nothing and <see cref="DN"/> returns the null vector; both are kept.
/// </remarks>
internal sealed class HiddenLineCurve
{
    private readonly EdgeCurveAdaptor _curve;
    private CurveKind _type;
    private StrongBox<AlgorithmProjector>? _proj;
    private double _ox;
    private double _oz;
    private double _vx;
    private double _vz;
    private double _of;

    /// <summary>Creates an undefined curve.</summary>
    public HiddenLineCurve()
    {
        _curve = new EdgeCurveAdaptor();
    }

    private HiddenLineCurve(HiddenLineCurve other)
    {
        _curve = (EdgeCurveAdaptor)other._curve.ShallowCopy();
        _type = other._type;
        _proj = other._proj;
        _ox = other._ox;
        _oz = other._oz;
        _vx = other._vx;
        _vz = other._vz;
        _of = other._of;
    }

    /// <summary>OCCT's implicit copy: the adaptor copied, the projector shared.</summary>
    public HiddenLineCurve Copy() => new(this);

    /// <summary>Sets the projector (OCCT's pointer to the projector HLRBRep_Data holds).</summary>
    public void Projector(StrongBox<AlgorithmProjector> proj) => _proj = proj;

    private ref readonly AlgorithmProjector Proj() => ref _proj!.Value;

    /// <summary>Returns the 3D curve.</summary>
    public EdgeCurveAdaptor Curve() => _curve;

    /// <summary>Sets the 3D curve to be projected.</summary>
    public void Curve(in Edge e)
    {
        _curve.Initialize(e);
    }

    /// <summary>Returns the 3D curve.</summary>
    public EdgeCurveAdaptor GetCurve() => _curve;

    /// <summary>Returns the parameter on the 2d curve from the parameter on the 3d curve.</summary>
    public double Parameter2d(double p3d)
    {
        // Mathematical formula for lines

        //        myOF P3d (myOF myVX - myOZ myVX + myOX myVZ)
        // Res -> --------------------------------------------
        //        (-myOF + myOZ) (-myOF + myOZ + P3d myVZ)

        switch (_type)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                if (Proj().Perspective())
                {
                    double fmOZ = _of - _oz;
                    return _of * p3d * (_vx * fmOZ + _ox * _vz) / (fmOZ * (fmOZ - p3d * _vz));
                }
                return p3d * _vx;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return p3d + _ox;

            default: // implemented to avoid gcc compiler warnings
                break;
        }
        return p3d;
    }

    /// <summary>Returns the parameter on the 3d curve from the parameter on the 2d curve.</summary>
    public double Parameter3d(double p2d)
    {
        // Mathematical formula for lines

        //                                 2
        //                   (-myOF + myOZ)  P2d
        // P3d -> -----------------------------------------------------
        //        (myOF - myOZ) (myOF myVX + P2d myVZ) + myOF myOX myVZ

        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            if (Proj().Perspective())
            {
                double fmOZ = _of - _oz;
                return p2d * fmOZ * fmOZ / (fmOZ * (_of * _vx + p2d * _vz) + _of * _ox * _vz);
            }
            return ((_vx <= Constants.Resolution()) ? p2d : (p2d / _vx));
        }

        else if (_type == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            return p2d - _ox;
        }

        return p2d;
    }

    /// <summary>Updates the min-max boxes and the internal data; returns the tolerance to enlarge them by.</summary>
    public double Update(Span<double> totMin, Span<double> totMax)
    {
        CurveKind typ = BCurveTool.GetType(_curve);
        _type = OtherCurve;

        switch (typ)
        {

            case global::Kerf.Geometry.CurveKind.Line:
                _type = typ;
                break;

            case global::Kerf.Geometry.CurveKind.Circle:
                if (!Proj().Perspective())
                {
                    Direction3D d1 = BCurveTool.Circle(_curve).Axis.Direction;
                    d1 = d1.Transformed(Proj().Transformation());
                    if (d1.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                    }
                    else if (Abs(d1.Dot(Constants.DZ())) < global::Kerf.Engine.Foundation.Precision.Angular() * 10)
                    { //*10: The minor radius of ellipse should not be too small.
                        _type = OtherCurve;
                    }
                    else
                    {
                        _type = global::Kerf.Geometry.CurveKind.Ellipse;
                        // compute the angle offset
                        Direction3D d3 = d1.Crossed(Constants.DZ());
                        Direction3D d2 = BCurveTool.Circle(_curve).XAxis.Direction;
                        d2 = d2.Transformed(Proj().Transformation());
                        _ox = d3.AngleWithRef(d2, d1);
                    }
                }
                break;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                if (!Proj().Perspective())
                {
                    Direction3D d1 = BCurveTool.Ellipse(_curve).Axis.Direction;
                    d1 = d1.Transformed(Proj().Transformation());
                    if (d1.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        _ox = 0.0; // no offset on the angle
                        _type = global::Kerf.Geometry.CurveKind.Ellipse;
                    }
                }
                break;

            case BezierCurve:
                if (BCurveTool.Degree(_curve) == 1)
                {
                    _type = global::Kerf.Geometry.CurveKind.Line;
                }
                else if (!Proj().Perspective())
                {
                    _type = typ;
                }
                break;

            case BSplineCurve:
                if (!Proj().Perspective())
                {
                    _type = typ;
                }
                break;

            default:
                break;
        }

        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            // compute the values for a line
            Line3D L;
            double l3d = 1.0; // length of the 3d bezier curve
            if (BCurveTool.GetType(_curve) == global::Kerf.Geometry.CurveKind.Line)
            {
                L = BCurveTool.Line(_curve);
            }
            else
            { // bezier degree 1
                BCurveTool.D1(_curve, 0, out Point3D pl, out Vector3D vl);
                L = new Line3D(pl, new Direction3D(vl));
                l3d = pl.DistanceTo(BCurveTool.Value(_curve, 1.0));
            }
            Point3D p = L.Location;
            Vector3D v = new(L.Direction);
            p = p.Transformed(Proj().Transformation());
            v = v.Transformed(Proj().Transformation());
            if (Proj().Perspective())
            {
                D1(0.0, out Point2D f, out Vector2D vfx);
                vfx = vfx.Normalized();
                _vx = (vfx.X * v.X + vfx.Y * v.Y) * l3d;
                double l = -(vfx.X * f.X + vfx.Y * f.Y);
                f = f.WithCoord(f.X + vfx.X * l, f.Y + vfx.Y * l);
                _ox = vfx.X * (p.X - f.X) + vfx.Y * (p.Y - f.Y);
                Vector3D vfz = new(-f.X, -f.Y, Proj().Focus());
                _of = vfz.Magnitude;
                vfz /= _of;
                _vz = vfz * v;
                _vz *= l3d;
                _oz = vfz * new Vector3D(p.X - f.X, p.Y - f.Y, p.Z);
            }
            else
            {
                _vx = Sqrt(v.X * v.X + v.Y * v.Y) * l3d;
            }
        }
        return (UpdateMinMax(totMin, totMax));
    }

    /// <summary>Updates the min-max boxes; returns the tolerance to enlarge them by.</summary>
    public double UpdateMinMax(Span<double> totMin, Span<double> totMax)
    {
        double a = BCurveTool.FirstParameter(_curve);
        double b = BCurveTool.LastParameter(_curve);
        double x, y, z, tolMinMax = 0;
        Proj().Project(Value3D(a), out x, out y, out z);
        HlrAlgo.UpdateMinMax(x, y, z, totMin, totMax);

        if (_type != global::Kerf.Geometry.CurveKind.Line)
        {
            int nbPnt = 30;
            int i;
            double step = (b - a) / (nbPnt + 1);
            double xa, ya, za, xb = 0.0, yb = 0.0, zb = 0.0;
            double dx1, dy1, dz1, dd1;
            double dx2, dy2, dz2, dd2;

            for (i = 1; i <= nbPnt; i++)
            {
                a += step;
                xa = xb;
                ya = yb;
                za = zb;
                xb = x;
                yb = y;
                zb = z;
                Proj().Project(Value3D(a), out x, out y, out z);
                HlrAlgo.UpdateMinMax(x, y, z, totMin, totMax);
                if (i >= 2)
                {
                    dx1 = x - xa;
                    dy1 = y - ya;
                    dz1 = z - za;
                    dd1 = Sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
                    if (dd1 > 0)
                    {
                        dx2 = xb - xa;
                        dy2 = yb - ya;
                        dz2 = zb - za;
                        dd2 = Sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
                        if (dd2 > 0)
                        {
                            double p = (dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (dd1 * dd2);
                            dx1 = xa + p * dx1 - xb;
                            dy1 = ya + p * dy1 - yb;
                            dz1 = za + p * dz1 - zb;
                            dd1 = Sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
                            if (dd1 > tolMinMax)
                            {
                                tolMinMax = dd1;
                            }
                        }
                    }
                }
            }
        }
        Proj().Project(Value3D(b), out x, out y, out z);
        HlrAlgo.UpdateMinMax(x, y, z, totMin, totMax);
        return tolMinMax;
    }

    /// <summary>The Z coordinate, in the viewing coordinate system, of the point of parameter U.</summary>
    public double Z(double u)
    {
        BCurveTool.D0(_curve, u, out Point3D p3d);
        p3d = p3d.Transformed(Proj().Transformation());
        return p3d.Z;
    }

    /// <summary>The 3D point of parameter U.</summary>
    public Point3D Value3D(double u)
    {
        D0(u, out Point3D p);
        return p;
    }

    /// <summary>The 3D point of parameter U.</summary>
    public void D0(double u, out Point3D p)
    {
        BCurveTool.D0(_curve, u, out p);
    }

    /// <summary>The 3D point of parameter U and the first derivative.</summary>
    public void D1(double u, out Point3D p, out Vector3D v)
    {
        BCurveTool.D1(_curve, u, out p, out v);
    }

    /// <summary>
    /// The 2D point and tangent at the start (or the end) of the curve. If the first derivative is null, looks a little
    /// after the start (or before the end) with the second derivative.
    /// </summary>
    /// <exception cref="UndefinedDerivativeException">The tangent is undefined.</exception>
    public void Tangent(bool atStart, out Point2D p, out Direction2D d)
    {
        double u = atStart ? BCurveTool.FirstParameter(_curve) : BCurveTool.LastParameter(_curve);

        D0(u, out p);
        HiddenLineCLProps clp = new(2, Epsilon(1.0));
        HiddenLineCurve curve = this;
        clp.SetCurve(curve);
        clp.SetParameter(u);
        StdFail_UndefinedDerivative_Raise_if(!clp.IsTangentDefined(), "HLRBRep_Curve::Tangent");
        clp.Tangent(out d);
    }

    public double FirstParameter() => Parameter2d(BCurveTool.FirstParameter(_curve));

    public double LastParameter() => Parameter2d(BCurveTool.LastParameter(_curve));

    public Continuity Continuity() => BCurveTool.Continuity(_curve);

    /// <summary>The number of intervals of continuity S.</summary>
    public int NbIntervals(Continuity s) => BCurveTool.NbIntervals(_curve, s);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S.</summary>
    public void Intervals(NCollection_Array1<double> tab, Continuity sh)
    {
        BCurveTool.Intervals(_curve, tab, sh);
    }

    public bool IsClosed => BCurveTool.IsClosed(_curve);

    public bool IsPeriodic => BCurveTool.IsPeriodic(_curve);

    public double Period() => BCurveTool.Period(_curve);

    /// <summary>The 2D point of parameter U (a parameter on the 3D curve).</summary>
    public Point2D Value(double u)
    {
        D0(u, out Point2D p);
        return p;
    }

    /// <summary>The 2D point of parameter U.</summary>
    public void D0(double u, out Point2D p)
    {
        /* gp_Pnt P3d;
        HLRBRep_BCurveTool::D0(myCurve,U,P3d);
        P3d.Transform(((HLRAlgo_Projector*) myProj)->Transformation());
        if (((HLRAlgo_Projector*) myProj)->Perspective()) {
          double R = 1.-P3d.Z()/((HLRAlgo_Projector*) myProj)->Focus();
          P.SetCoord(P3d.X()/R,P3d.Y()/R);
        }
        else P.SetCoord(P3d.X(),P3d.Y()); */
        BCurveTool.D0(_curve, u, out Point3D p3d);
        p = new Point2D(); // C++'s default point, which the first axonometric view leaves (decision 1078)
        Proj().Project(p3d, ref p);
    }

    /// <summary>The 2D point of parameter U and the first derivative.</summary>
    public void D1(double u, out Point2D p, out Vector2D v)
    {
        // Mathematical formula for lines

        //        X'[t]      X[t] Z'[t]
        // D1 =  -------- + -------------
        //           Z[t]          Z[t] 2
        //       1 - ----   f (1 - ----)
        //            f             f

        BCurveTool.D1(_curve, u, out Point3D p3D, out Vector3D v13D);
        if (Proj().Perspective())
        {
            p3D = p3D.Transformed(Proj().Transformation());
            v13D = v13D.Transformed(Proj().Transformation());

            double f = Proj().Focus();
            double r = 1.0 - p3D.Z / f;
            double e = v13D.Z / (f * r * r);
            p = new Point2D(p3D.X / r, p3D.Y / r);
            v = new Vector2D(v13D.X / r + p3D.X * e, v13D.Y / r + p3D.Y * e);
        }
        else
        {
            // OCC155
            Proj().Project(p3D, v13D, out p, out v);
        }
    }

    /// <summary>The 2D point of parameter U and the first two derivatives.</summary>
    public void D2(double u, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        // Mathematical formula for lines

        //                                   2
        //       2 X'[t] Z'[t]   2 X[t] Z'[t]      X''[t]     X[t] Z''[t]
        // D2 =  ------------- + -------------- + -------- + -------------
        //              Z[t] 2    2      Z[t] 3       Z[t]          Z[t] 2
        //       f (1 - ----)    f  (1 - ----)    1 - ----   f (1 - ----)
        //               f                f            f             f

        BCurveTool.D2(_curve, u, out Point3D p3D, out Vector3D v13D, out Vector3D v23D);
        p3D = p3D.Transformed(Proj().Transformation());
        v13D = v13D.Transformed(Proj().Transformation());
        v23D = v23D.Transformed(Proj().Transformation());
        if (Proj().Perspective())
        {
            double f = Proj().Focus();
            double r = 1.0 - p3D.Z / f;
            double q = f * r * r;
            double e = v13D.Z / q;
            double c = e * v13D.Z / (f * r);
            p = new Point2D(p3D.X / r, p3D.Y / r);
            v1 = new Vector2D(v13D.X / r + p3D.X * e, v13D.Y / r + p3D.Y * e);
            v2 = new Vector2D(v23D.X / r + 2 * v13D.X * e + p3D.X * v23D.Z / q + 2 * p3D.X * c,
                              v23D.Y / r + 2 * v13D.Y * e + p3D.Y * v23D.Z / q + 2 * p3D.Y * c);
        }
        else
        {
            p = new Point2D(p3D.X, p3D.Y);
            v1 = new Vector2D(v13D.X, v13D.Y);
            v2 = new Vector2D(v23D.X, v23D.Y);
        }
    }

    /// <summary>OCCT's third derivative: writes nothing (the arguments keep their values).</summary>
    public void D3(double u, ref Point2D p, ref Vector2D v1, ref Vector2D v2, ref Vector2D v3) { } // OCCT

    /// <summary>OCCT's derivative of order N: the null vector.</summary>
    public Vector2D DN(double u, int n) => new(); // OCCT

    /// <summary>The parametric resolution for the 3D resolution R3d.</summary>
    public double Resolution(double r3d) => BCurveTool.Resolution(_curve, r3d);

    /// <summary>The type of the projected curve.</summary>
    public new CurveKind GetType() => _type;

    public Line2D Line()
    {
        D1(0.0, out Point2D p, out Vector2D v);
        return new Line2D(p, new Direction2D(v));
    }

    public Circle2D Circle()
    {
        Circle3D c = BCurveTool.Circle(_curve);
        c = c.Transformed(Proj().Transformation());
        return global::Kerf.Engine.Geometry.Projection.ProjLib.Project(new Plane3D(new CoordinateSystem(Constants.XOY())), c);
    }

    public Ellipse2D Ellipse()
    {
        if (BCurveTool.GetType(_curve) == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            Ellipse3D e = BCurveTool.Ellipse(_curve);
            e = e.Transformed(Proj().Transformation());
            return global::Kerf.Engine.Geometry.Projection.ProjLib.Project(new Plane3D(new CoordinateSystem(Constants.XOY())), e);
        }
        // this is a circle
        Circle3D c = BCurveTool.Circle(_curve);
        c = c.Transformed(Proj().Transformation());
        Direction3D d1 = c.Axis.Direction;
        Direction3D d3 = d1.Crossed(Constants.DZ());
        Direction3D d2 = d1.Crossed(d3);
        double rap = Sqrt(d2.X * d2.X + d2.Y * d2.Y);
        Direction2D d = new(d1.Y, -d1.X);
        Point2D p = new(c.Location.X, c.Location.Y);
        Ellipse2D el = new(new Axis2D(p, d), c.Radius, c.Radius * rap);
        if (d1.Z < 0)
        {
            el = el.Reversed();
        }
        return el;
    }

    public Hyperbola2D Hyperbola() => new();

    public Parabola2D Parabola() => new();

    public bool IsRational => BCurveTool.IsRational(_curve);

    public int Degree() => BCurveTool.Degree(_curve);

    public int PoleCount => BCurveTool.NbPoles(_curve);

    /// <summary>The projected poles of the Bezier or B-spline curve.</summary>
    public void Poles(NCollection_Array1<Point2D> tp)
    {
        int i1 = tp.Lower();
        int i2 = tp.Upper();
        NCollection_Array1<Point3D> tp3 = new(i1, i2);
        if (BCurveTool.GetType(_curve) == BSplineCurve)
        {
            BSplineCurve3D bSpl = BCurveTool.BSpline(_curve);
            NCollection_Array1<Point3D> srcPoles = bSpl.InternalPoles();
            for (int i = i1; i <= i2; i++)
            {
                tp3[i] = srcPoles[i];
            }
        }
        else
        {
            BezierCurve3D bez = BCurveTool.Bezier(_curve);
            NCollection_Array1<Point3D> srcPoles = bez.InternalPoles();
            for (int i = i1; i <= i2; i++)
            {
                tp3[i] = srcPoles[i];
            }
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref tp3[i]);
            tp[i] = new Point2D(tp3[i].X, tp3[i].Y);
        }
    }

    /// <summary>The projected poles of the given B-spline curve.</summary>
    public void Poles(BSplineCurve3D aCurve, NCollection_Array1<Point2D> tp)
    {
        int i1 = tp.Lower();
        int i2 = tp.Upper();
        NCollection_Array1<Point3D> tp3 = new(i1, i2);
        NCollection_Array1<Point3D> srcPoles = aCurve.InternalPoles();
        for (int i = i1; i <= i2; i++)
        {
            tp3[i] = srcPoles[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref tp3[i]);
            tp[i] = new Point2D(tp3[i].X, tp3[i].Y);
        }
    }

    /// <summary>The projected poles and the weights of the Bezier or B-spline curve.</summary>
    public void PolesAndWeights(NCollection_Array1<Point2D> tp, NCollection_Array1<double> tw)
    {
        int i1 = tp.Lower();
        int i2 = tp.Upper();
        NCollection_Array1<Point3D> tp3 = new(i1, i2);
        if (BCurveTool.GetType(_curve) == BSplineCurve)
        {
            BSplineCurve3D hb = (BCurveTool.BSpline(_curve));
            NCollection_Array1<Point3D> srcPoles = hb.InternalPoles();
            NCollection_Array1<double> srcWeights = hb.InternalWeightsArray();
            for (int i = i1; i <= i2; i++)
            {
                tp3[i] = srcPoles[i];
            }
            for (int i = i1; i <= i2; i++)
            {
                tw[i] = srcWeights[i];
            }
        }
        else
        {
            BezierCurve3D hb = (BCurveTool.Bezier(_curve));
            NCollection_Array1<Point3D> srcPoles = hb.InternalPoles();
            NCollection_Array1<double> srcWeights = hb.InternalWeightsArray();
            for (int i = i1; i <= i2; i++)
            {
                tp3[i] = srcPoles[i];
            }
            for (int i = i1; i <= i2; i++)
            {
                tw[i] = srcWeights[i];
            }
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref tp3[i]);
            tp[i] = new Point2D(tp3[i].X, tp3[i].Y);
        }
    }

    /// <summary>The projected poles and the weights of the given B-spline curve.</summary>
    public void PolesAndWeights(BSplineCurve3D aCurve, NCollection_Array1<Point2D> tp,
                                NCollection_Array1<double> tw)
    {
        int i1 = tp.Lower();
        int i2 = tp.Upper();
        NCollection_Array1<Point3D> tp3 = new(i1, i2);
        NCollection_Array1<Point3D> srcPoles = aCurve.InternalPoles();
        NCollection_Array1<double> srcWeights = aCurve.InternalWeightsArray();
        for (int i = i1; i <= i2; i++)
        {
            tp3[i] = srcPoles[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            tw[i] = srcWeights[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref tp3[i]);
            tp[i] = new Point2D(tp3[i].X, tp3[i].Y);
        }
    }

    public int KnotCount => BCurveTool.NbKnots(_curve);

    /// <summary>The knots of the B-spline curve (nothing for other curves).</summary>
    public void Knots(NCollection_Array1<double> kn)
    {
        if (BCurveTool.GetType(_curve) == BSplineCurve)
        {
            BSplineCurve3D bSpl = BCurveTool.BSpline(_curve);
            NCollection_Array1<double> srcKnots = bSpl.InternalKnots();
            for (int i = kn.Lower(); i <= kn.Upper(); i++)
            {
                kn[i] = srcKnots[i];
            }
        }
    }

    /// <summary>The multiplicities of the B-spline curve (nothing for other curves).</summary>
    public void Multiplicities(NCollection_Array1<int> mu)
    {
        if (BCurveTool.GetType(_curve) == BSplineCurve)
        {
            BSplineCurve3D bSpl = BCurveTool.BSpline(_curve);
            NCollection_Array1<int> srcMults = bSpl.InternalMultiplicities();
            for (int i = mu.Lower(); i <= mu.Upper(); i++)
            {
                mu[i] = srcMults[i];
            }
        }
    }
}
