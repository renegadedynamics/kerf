// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnPlane.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnPlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// 09-Aug-95 : xab : changed the ProjLib_ProjectOnPlane in the case
//                   of the line and the parameteriation is kept

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Projects a curve on a plane along a direction (the plane's normal by default). The projected curve is a line,
/// conic, Bezier or B-spline where it can be computed exactly or by apex search; otherwise it is approximated by a
/// B-spline. Lines, conics and polynomial curves keep their parametrization when requested.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Load"/> is public. Keeps the caller's curve adaptor by reference, as
/// OCCT keeps the handle. The curves it builds are frozen when handed out through <see cref="Bezier"/>,
/// <see cref="BSpline"/> or <see cref="GetResult"/> (decision 243): the same objects at every call.
/// </remarks>
internal class ProjectOnPlane : CurveAdaptorBase
{
    private const double AParabolaLimit = 20000.0;
    private const double AHyperbolaLimit = 10.0;

    private CurveAdaptorBase? _curve;
    private CoordinateSystem _plane = new();
    private Direction3D _direction = new();
    private bool _keepParam;
    private double _firstPar;
    private double _lastPar;
    private double _tolerance;
    private CurveKind _type;
    private CurveAdaptor? _result;
    private bool _isApprox;

    private static Point3D OnPlane_Value(double u, CurveAdaptorBase aCurvePtr, in CoordinateSystem pl, in Direction3D d)
    {
        //                   PO . Z                /  Z = Pl.Direction()
        // Proj(u) = P(u) + -------  * D     with  \  O = Pl.Location()
        //                   D  . Z

        Point3D point = aCurvePtr.Value(u);

        Vector3D po = new(point, pl.Location);

        double alpha = po * new Vector3D(pl.Direction);
        alpha /= d * pl.Direction;
        point = point.WithXYZ(point.XYZ() + alpha * d.XYZ());

        return point;
    }

    private static Vector3D OnPlane_DN(double u, int derivativeRequest, CurveAdaptorBase aCurvePtr, in CoordinateSystem pl, in Direction3D d)
    {
        //                   PO . Z                /  Z = Pl.Direction()
        // Proj(u) = P(u) + -------  * D     with  \  O = Pl.Location()
        //                   D  . Z

        Vector3D vector = aCurvePtr.DN(u, derivativeRequest);

        Direction3D z = pl.Direction;

        double alpha = vector * new Vector3D(z);
        alpha /= d * z;

        vector = vector.WithXYZ(vector.XYZ() - alpha * d.XYZ());
        return vector;
    }

    private static bool OnPlane_D1(double u, out Point3D p, out Vector3D v, CurveAdaptorBase aCurvePtr, in CoordinateSystem pl, in Direction3D d)
    {
        double alpha;
        Point3D point;
        Vector3D vector;

        Direction3D z = pl.Direction;

        aCurvePtr.D1(u, out point, out vector);

        // evaluate the point as in `OnPlane_Value`
        Vector3D po = new(point, pl.Location);
        alpha = po * new Vector3D(z);
        alpha /= d * z;
        p = new Point3D();
        p = p.WithXYZ(point.XYZ() + alpha * d.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        alpha = vector * new Vector3D(z);
        alpha /= d * z;

        v = new Vector3D();
        v = v.WithXYZ(vector.XYZ() - alpha * d.XYZ());

        return true;
    }

    private static bool OnPlane_D2(double u,
                                   out Point3D p,
                                   out Vector3D v1,
                                   out Vector3D v2,
                                   CurveAdaptorBase aCurvePtr,
                                   in CoordinateSystem pl,
                                   in Direction3D d)
    {
        double alpha;
        Point3D point;
        Vector3D vector1, vector2;

        Direction3D z = pl.Direction;

        aCurvePtr.D2(u, out point, out vector1, out vector2);

        // evaluate the point as in `OnPlane_Value`
        Vector3D po = new(point, pl.Location);
        alpha = po * new Vector3D(z);
        alpha /= d * z;
        p = new Point3D();
        p = p.WithXYZ(point.XYZ() + alpha * d.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        alpha = vector1 * new Vector3D(z);
        alpha /= d * z;

        v1 = new Vector3D();
        v1 = v1.WithXYZ(vector1.XYZ() - alpha * d.XYZ());

        alpha = vector2 * new Vector3D(z);
        alpha /= d * z;

        v2 = new Vector3D();
        v2 = v2.WithXYZ(vector2.XYZ() - alpha * d.XYZ());
        return true;
    }

    private static bool OnPlane_D3(double u,
                                   out Point3D p,
                                   out Vector3D v1,
                                   out Vector3D v2,
                                   out Vector3D v3,
                                   CurveAdaptorBase aCurvePtr,
                                   in CoordinateSystem pl,
                                   in Direction3D d)
    {
        double alpha;
        Point3D point;
        Vector3D vector1, vector2, vector3;

        Direction3D z = pl.Direction;

        aCurvePtr.D3(u, out point, out vector1, out vector2, out vector3);

        // evaluate the point as in `OnPlane_Value`
        Vector3D po = new(point, pl.Location);
        alpha = po * new Vector3D(z);
        alpha /= d * z;
        p = new Point3D();
        p = p.WithXYZ(point.XYZ() + alpha * d.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        alpha = vector1 * new Vector3D(z);
        alpha /= d * z;

        v1 = new Vector3D();
        v1 = v1.WithXYZ(vector1.XYZ() - alpha * d.XYZ());

        alpha = vector2 * new Vector3D(z);
        alpha /= d * z;

        v2 = new Vector3D();
        v2 = v2.WithXYZ(vector2.XYZ() - alpha * d.XYZ());
        alpha = vector3 * new Vector3D(z);
        alpha /= d * z;

        v3 = new Vector3D();
        v3 = v3.WithXYZ(vector3.XYZ() - alpha * d.XYZ());
        return true;
    }

    /// <summary>OCCT's file-local class ProjLib_OnPlane: the projected point, approximated by PerformApprox.</summary>
    private sealed class ProjLib_OnPlane : global::Kerf.Engine.Approximation.Function
    {
        private readonly CurveAdaptorBase _curve;
        private readonly CoordinateSystem _plane;
        private readonly Direction3D _direction;

        public ProjLib_OnPlane(CurveAdaptorBase c, in CoordinateSystem pl, in Direction3D d)
        {
            _curve = c;
            _plane = pl;
            _direction = d;
            _nbPnt = 1;
            _nbPnt2d = 0;
        }

        public override double FirstParameter() => _curve.FirstParameter();

        public override double LastParameter() => _curve.LastParameter();

        public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
        {
            pnt[1] = OnPlane_Value(t, _curve, _plane, _direction);
            return true;
        }

        public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec)
        {
            return OnPlane_D1(t, out _, out vec[1], _curve, _plane, _direction);
        }
    }

    /// <summary>OCCT's file-local class ProjLib_MaxCurvature: minus the curvature, to be minimised.</summary>
    private sealed class ProjLib_MaxCurvature : Function
    {
        private readonly CLProps3d _props;

        public ProjLib_MaxCurvature(CLProps3d props)
        {
            _props = props;
        }

        public override bool Value(double x, out double f)
        {
            _props.SetParameter(x);
            f = -_props.Curvature();
            return true;
        }
    }

    //=====================================================================//
    //                                                                     //
    //  D E S C R I P T I O N   O F   T H E   C L A S S  :                 //
    //                                                                     //
    //         P r o j L i b _ A p p r o x P r o j e c t O n P l a n e     //
    //                                                                     //
    //=====================================================================//

    private static void PerformApprox(CurveAdaptorBase c, in CoordinateSystem pl, in Direction3D d, ref BSplineCurve3D? bSplineCurvePtr)
    {
        ProjLib_OnPlane f = new(c, pl, d);

        int deg1, deg2;
        deg1 = 8;
        deg2 = 8;
        if (c.GetType() == global::Kerf.Geometry.CurveKind.Parabola)
        {
            deg1 = 2;
            deg2 = 2;
        }
        int nbSegm = 100;
        if (c.GetType() == global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            deg1 = 14;
            deg2 = 14;
            nbSegm = 1000;
        }
        global::Kerf.Engine.Approximation.FitAndDivide fit = new(deg1, deg2, global::Kerf.Engine.Foundation.Precision.Approximation(), global::Kerf.Engine.Foundation.Precision.PApproximation(), true);
        fit.SetMaxSegments(nbSegm);
        fit.Perform(f);
        if (!fit.IsAllApproximated)
        {
            return;
        }
        int i;
        int nbCurves = fit.MultiCurveCount;
        int maxDeg = 0;

        // To convert the MultiCurve to BSpline, all constituent Bezier curves
        // must have the same degree -> Calculate MaxDeg
        int nbPoles;
        for (i = 1; i <= nbCurves; i++)
        {
            int deg = fit.Value(i).Degree();
            maxDeg = Max(maxDeg, deg);
        }
        nbPoles = maxDeg * nbCurves + 1; // Poles on the BSpline

        NCollection_Array1<Point3D> poles = new(1, nbPoles);

        NCollection_Array1<Point3D> tempPoles = new(1, maxDeg + 1); // for degree elevation

        NCollection_Array1<double> knots = new(1, nbCurves + 1); // Knots of the BSpline

        int compt = 1;
        double errMax = 0.0, err3d, err2d;
        for (i = 1; i <= fit.MultiCurveCount; i++)
        {
            fit.Parameters(i, out knots[i], out knots[i + 1]);
            fit.Error(i, out err3d, out err2d);
            errMax = Max(errMax, err3d);
            global::Kerf.Engine.Approximation.MultiCurve mc = fit.Value(i);                    // Load the i-th Curve
            NCollection_Array1<Point3D> localPoles = new(1, mc.Degree() + 1); // Get the poles
            mc.Curve(1, localPoles);

            // Possible degree elevation
            if (maxDeg > mc.Degree())
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(maxDeg, localPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), tempPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                // update the poles of the PCurve
                for (int j = 1; j <= maxDeg + 1; j++)
                {
                    poles.SetValue(compt, tempPoles[j]);
                    compt++;
                }
            }
            else
            {
                // update the poles of the PCurve
                for (int j = 1; j <= maxDeg + 1; j++)
                {
                    poles.SetValue(compt, localPoles[j]);
                    compt++;
                }
            }

            compt--;
        }

        // update the fields of ProjLib_Approx

        int nbKnots = nbCurves + 1;

        NCollection_Array1<int> mults = new(1, nbKnots);
        mults.SetValue(1, maxDeg + 1);
        for (i = 2; i <= nbCurves; i++)
        {
            mults.SetValue(i, maxDeg);
        }
        mults.SetValue(nbKnots, maxDeg + 1);
        bSplineCurvePtr = new BSplineCurve3D(poles, knots, mults, maxDeg, false);

        // Try to smooth
        int m1 = maxDeg - 1;
        for (i = 2; i < nbKnots; ++i)
        {
            if (bSplineCurvePtr.Multiplicity(i) == maxDeg)
            {
                bSplineCurvePtr.RemoveKnot(i, m1, errMax);
            }
        }
    }

    /// <summary>An empty projector: the plane is the global XOY plane.</summary>
    public ProjectOnPlane()
    {
        _keepParam = false;
        _firstPar = 0.0;
        _lastPar = 0.0;
        _tolerance = 0.0;
        _type = OtherCurve;
        _isApprox = false;
    }

    /// <summary>A projector on <paramref name="pl"/> along its normal.</summary>
    public ProjectOnPlane(in CoordinateSystem pl)
    {
        _plane = pl;
        _direction = pl.Direction;
        _keepParam = false;
        _firstPar = 0.0;
        _lastPar = 0.0;
        _tolerance = 0.0;
        _type = OtherCurve;
        _isApprox = false;
    }

    /// <summary>A projector on <paramref name="pl"/> along <paramref name="d"/> (not checked to be oblique to the
    /// plane).</summary>
    public ProjectOnPlane(in CoordinateSystem pl, in Direction3D d)
    {
        _plane = pl;
        _direction = d;
        _keepParam = false;
        _firstPar = 0.0;
        _lastPar = 0.0;
        _tolerance = 0.0;
        _type = OtherCurve;
        _isApprox = false;
        //  if ( std::abs(D * Pl.Direction()) < Precision::Confusion()) {
        //    throw Standard_ConstructionError
        //      ("ProjLib_ProjectOnPlane:  The Direction and the Plane are parallel");
        //  }
    }

    /// <summary>A copy sharing the plane and direction, with shallow copies of the curve and the result adaptors.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        ProjectOnPlane copy = new();

        if (_curve is not null)
        {
            copy._curve = _curve.ShallowCopy();
        }
        copy._plane = _plane;
        copy._direction = _direction;
        copy._keepParam = _keepParam;
        copy._firstPar = _firstPar;
        copy._lastPar = _lastPar;
        copy._tolerance = _tolerance;
        copy._type = _type;
        if (_result is not null)
        {
            copy._result = _result.ShallowCopy() as CurveAdaptor;
        }
        copy._isApprox = _isApprox;

        return copy;
    }

    private static Point3D ProjectPnt(in CoordinateSystem thePlane, in Direction3D theDir, in Point3D point)
    {
        Vector3D po = new(point, thePlane.Location);

        double alpha = po * new Vector3D(thePlane.Direction);
        alpha /= theDir * thePlane.Direction;

        Point3D p = new Point3D().WithXYZ(point.XYZ() + alpha * theDir.XYZ());

        return p;
    }

    private static Vector3D ProjectVec(in CoordinateSystem thePlane, in Direction3D theDir, in Vector3D vec)
    {
        Vector3D d = vec;
        Vector3D z = new(thePlane.Direction);

        d -= ((vec * z) / (theDir * new Direction3D(z))) * new Vector3D(theDir);

        return d;
    }

    /// <summary>
    /// Projects <paramref name="c"/>. With <paramref name="keepParametrization"/>, the projection keeps the curve's
    /// parameters (lines become degree-1 B-splines, conics that don't stay orthogonal are approximated); Bezier and
    /// B-spline curves always keep them.
    /// </summary>
    public void Load(CurveAdaptorBase c, double tolerance, bool keepParametrization = true)
    {
        _curve = c;
        _type = OtherCurve;
        _isApprox = false;
        _tolerance = tolerance;

        BSplineCurve3D? approxCurve = null;

        LineCurve3D geomLinePtr;
        CircleCurve3D? geomCirclePtr = null;
        EllipseCurve3D? geomEllipsePtr = null;
        HyperbolaCurve3D? geomHyperbolaPtr = null;
        ParabolaCurve3D? geomParabolaPtr = null;

        Line3D line;
        Ellipse3D elips;
        //  gp_Hypr  Hypr ;

        CurveKind type = c.GetType();

        Frame3D axis = new();
        double R1 = 0.0, R2 = 0.0;

        _keepParam = keepParametrization;

        switch (type)
        {
            case global::Kerf.Geometry.CurveKind.Line:
            {
                //     P(u) = O + u * Xc
                // ==> Q(u) = f(P(u))
                //          = f(O) + u * f(Xc)

                Line3D l = _curve.Line();
                Vector3D xc = ProjectVec(_plane, _direction, new Vector3D(l.Direction));

                if (xc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
                { // line orthogonal to the plane
                    _type = BSplineCurve;
                    Point3D p = ProjectPnt(_plane, _direction, l.Location);
                    NCollection_Array1<int> mults = new(1, 2);
                    mults.Init(2);
                    NCollection_Array1<Point3D> poles = new(1, 2);
                    poles.Init(p);
                    NCollection_Array1<double> knots = new(1, 2);
                    knots[1] = _curve.FirstParameter();
                    knots[2] = _curve.LastParameter();
                    BSplineCurve3D bsp = new(poles, knots, mults, 1);

                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    _result = new CurveAdaptor(bsp);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else if (Math.Abs(xc.Magnitude - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    _type = global::Kerf.Geometry.CurveKind.Line;
                    Point3D p = ProjectPnt(_plane, _direction, l.Location);
                    _firstPar = _curve.FirstParameter();
                    _lastPar = _curve.LastParameter();
                    line = new Line3D(p, new Direction3D(xc));
                    geomLinePtr = new LineCurve3D(line);

                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    _result = new CurveAdaptor(geomLinePtr, _curve.FirstParameter(), _curve.LastParameter());
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else
                {
                    _type = global::Kerf.Geometry.CurveKind.Line;
                    Point3D p = ProjectPnt(_plane, _direction, l.Location);
                    line = new Line3D(p, new Direction3D(xc));
                    double udeb, ufin;

                    // eval the first and last parameters of the projected curve
                    udeb = _curve.FirstParameter();
                    ufin = _curve.LastParameter();
                    Point3D p1 = ProjectPnt(_plane, _direction, _curve.Value(udeb));
                    Point3D p2 = ProjectPnt(_plane, _direction, _curve.Value(ufin));
                    _firstPar = new Vector3D(line.Direction).Dot(new Vector3D(p, p1));
                    _lastPar = new Vector3D(line.Direction).Dot(new Vector3D(p, p2));
                    geomLinePtr = new LineCurve3D(line);
                    if (!_keepParam)
                    {
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                        _result = new CurveAdaptor(geomLinePtr, _firstPar, _lastPar);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                    }
                    else
                    {
                        _type = BSplineCurve;
                        //
                        // make a linear BSpline of degree 1 between the end points of
                        // the projected line
                        //
                        TrimmedCurve3D newTrimCurvePtr = new(geomLinePtr, _firstPar, _lastPar);

                        BSplineCurve3D newCurvePtr = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(newTrimCurvePtr);
                        NCollection_Array1<double> bsplineKnots = new(newCurvePtr.InternalKnots());

                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(_curve.FirstParameter(), _curve.LastParameter(), bsplineKnots);

                        newCurvePtr.SetKnots(bsplineKnots);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                        _result = new CurveAdaptor(newCurvePtr);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                    }
                }
                break;
            }
            case global::Kerf.Geometry.CurveKind.Circle:
            case global::Kerf.Geometry.CurveKind.Ellipse:
            {
                if (type == global::Kerf.Geometry.CurveKind.Circle)
                {
                    // For circle and ellipse we have the following relations:
                    // (Note: for circle R1 = R2 = R)
                    //     P(u) = O + R1 * std::cos(u) * Xc + R2 * std::sin(u) * Yc
                    // ==> Q(u) = f(P(u))
                    //          = f(O) + R1 * std::cos(u) * f(Xc) + R2 * std::sin(u) * f(Yc)

                    Circle3D circ = _curve.Circle();
                    axis = circ.Position;
                    R1 = R2 = circ.Radius;
                }
                // [[fallthrough]] from the circle case
                if (type == global::Kerf.Geometry.CurveKind.Ellipse)
                {
                    Ellipse3D e = _curve.Ellipse();
                    axis = e.Position;
                    R1 = e.MajorRadius;
                    R2 = e.MinorRadius;
                }

                // Common Code  for CIRCLE & ELLIPSE begin here
                Direction3D x = axis.XDirection;
                Direction3D y = axis.YDirection;
                Vector3D vDx = ProjectVec(_plane, _direction, new Vector3D(x));
                Vector3D vDy = ProjectVec(_plane, _direction, new Vector3D(y));
                Direction3D dx = new(), dy = new();

                double tol2 = _tolerance * _tolerance;
                if (vDx.SquareMagnitude < tol2 || vDy.SquareMagnitude < tol2 || vDx.CrossSquareMagnitude(vDy) < tol2)
                {
                    _isApprox = true;
                }

                if (!_isApprox)
                {
                    dx = new Direction3D(vDx);
                    dy = new Direction3D(vDy);
                    Point3D o = axis.Location;
                    Point3D p = ProjectPnt(_plane, _direction, o);
                    Point3D px = ProjectPnt(_plane, _direction, o.Translated(R1 * new Vector3D(x)));
                    Point3D py = ProjectPnt(_plane, _direction, o.Translated(R2 * new Vector3D(y)));
                    double major = p.DistanceTo(px);
                    double minor = p.DistanceTo(py);

                    if (_keepParam)
                    {
                        _isApprox = !new Direction3D(vDx).IsNormal(new Direction3D(vDy), global::Kerf.Engine.Foundation.Precision.Angular());
                    }
                    else
                    {
                        // Since it is not necessary to keep the same parameter for the point on the original and
                        // on the projected curves, we will use the following approach to find axes of the
                        // projected ellipse and provide the canonical curve:
                        // https://www.geometrictools.com/Documentation/ParallelProjectionEllipse.pdf
                        Matrix matrA = new(1, 2, 1, 2);
                        // A = Jp^T * Pr(Je), where
                        //   Pr(Je) - projection of axes of original ellipse to the target plane
                        //   Jp - X and Y axes of the target plane
                        matrA[1, 1] = _plane.XDirection.XYZ().Dot(vDx.XYZ());
                        matrA[1, 2] = _plane.XDirection.XYZ().Dot(vDy.XYZ());
                        matrA[2, 1] = _plane.YDirection.XYZ().Dot(vDx.XYZ());
                        matrA[2, 2] = _plane.YDirection.XYZ().Dot(vDy.XYZ());

                        Matrix matrDelta2 = new(1, 2, 1, 2, 0.0);
                        //           | 1/MajorRad^2       0       |
                        // Delta^2 = |                            |
                        //           |      0        1/MajorRad^2 |
                        matrDelta2[1, 1] = 1.0 / (R1 * R1);
                        matrDelta2[2, 2] = 1.0 / (R2 * R2);

                        Matrix matrAInv = matrA.Inverse();
                        Matrix matrM = matrAInv.Transposed() * matrDelta2 * matrAInv;

                        // perform eigenvalues calculation
                        Jacobi eigenCalc = new(matrM);
                        if (eigenCalc.IsDone)
                        {
                            // radii of the projected ellipse
                            minor = 1.0 / Math.Sqrt(eigenCalc.Value(1));
                            major = 1.0 / Math.Sqrt(eigenCalc.Value(2));

                            // calculate the rotation angle for the plane axes to meet the correct axes of the
                            // projected ellipse (swap eigenvectors in respect to major and minor axes)
                            Matrix eigenVec = eigenCalc.Vectors();
                            Transform2D trsfInPlane = new Transform2D().WithValues(eigenVec[1, 2], eigenVec[1, 1], 0.0, eigenVec[2, 2], eigenVec[2, 1], 0.0);
                            Transform3D rot = new Transform3D().WithRotation(new Axis3D(p, _plane.Direction), trsfInPlane.RotationAngle);

                            dx = _plane.XDirection.Transformed(rot);
                            dy = _plane.YDirection.Transformed(rot);
                        }
                        else
                        {
                            _isApprox = true;
                        }
                    }

                    if (!_isApprox)
                    {
                        Frame3D axe = new(p, dx ^ dy, dx);

                        if (Math.Abs(major - minor) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            _type = global::Kerf.Geometry.CurveKind.Circle;
                            Circle3D circ = new(axe, major);
                            geomCirclePtr = new CircleCurve3D(circ);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                            _result = new CurveAdaptor(geomCirclePtr);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                        }
                        else if (major > minor)
                        {
                            _type = global::Kerf.Geometry.CurveKind.Ellipse;
                            elips = new Ellipse3D(axe, major, minor);

                            geomEllipsePtr = new EllipseCurve3D(elips);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                            _result = new CurveAdaptor(geomEllipsePtr);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                        }
                        else
                        {
                            _isApprox = true;
                        }
                    }
                }

                // No way to build the canonical curve, approximate as B-spline
                if (_isApprox)
                {
                    _type = BSplineCurve;
                    PerformApprox(_curve, _plane, _direction, ref approxCurve);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    // OCCT: when the approximation fails ApproxCurve stays null and GeomAdaptor_Curve throws
                    // Standard_NullObject.
                    _result = new CurveAdaptor(approxCurve!);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else if (geomCirclePtr is not null || geomEllipsePtr is not null)
                {
                    Curve3D? resultCurve = geomCirclePtr;
                    if (resultCurve is null)
                    {
                        resultCurve = geomEllipsePtr;
                    }
                    // start and end parameters of the projected curve
                    double parFirst = _curve.FirstParameter();
                    double parLast = _curve.LastParameter();
                    Point3D pntFirst = ProjectPnt(_plane, _direction, _curve.Value(parFirst));
                    Point3D pntLast = ProjectPnt(_plane, _direction, _curve.Value(parLast));
                    GeometryLibraryTool.Parameter(resultCurve, pntFirst, global::Kerf.Engine.Foundation.Precision.Confusion(), out _firstPar);
                    GeometryLibraryTool.Parameter(resultCurve, pntLast, global::Kerf.Engine.Foundation.Precision.Confusion(), out _lastPar);
                    while (_lastPar <= _firstPar)
                    {
                        _lastPar += _result!.Period();
                    }
                }
            }
            break;
            case global::Kerf.Geometry.CurveKind.Parabola:
            {
                //     P(u) = O + (u*u)/(4*f) * Xc + u * Yc
                // ==> Q(u) = f(P(u))
                //          = f(O) + (u*u)/(4*f) * f(Xc) + u * f(Yc)

                Parabola3D parab = _curve.Parabola();
                Frame3D axeRef = parab.Position;
                Vector3D xc = ProjectVec(_plane, _direction, new Vector3D(axeRef.XDirection));
                Vector3D yc = ProjectVec(_plane, _direction, new Vector3D(axeRef.YDirection));
                Point3D p = ProjectPnt(_plane, _direction, axeRef.Location);

                _isApprox = false;

                if ((Math.Abs(yc.Magnitude - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion()) && (xc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    _type = global::Kerf.Geometry.CurveKind.Line;
                    line = new Line3D(p, new Direction3D(yc));
                    geomLinePtr = new LineCurve3D(line);
                }
                else if (xc.IsNormal(yc, global::Kerf.Engine.Foundation.Precision.Angular())
                         && (!_keepParam || Math.Abs(yc.Magnitude - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    _type = global::Kerf.Geometry.CurveKind.Parabola;
                    // Q(s) = f(O) + s^2 |f(Xc)| / (4 Focal |f(Yc)|^2) * Xc' + s * Yc' with s = u |f(Yc)|: the focal is
                    // Focal * |f(Yc)|^2 / |f(Xc)|, and the parameter is kept only where f(Yc) stays unit; otherwise a
                    // kept parameter needs the approximation below (decision 1465; OCCT's focal ignored |f(Yc)|).
                    double f = parab.Focal * yc.SquareMagnitude / xc.Magnitude;
                    Parabola3D projParab = new(new Frame3D(p, new Direction3D(xc ^ yc), new Direction3D(xc)), f);
                    geomParabolaPtr = new ParabolaCurve3D(projParab);
                }
                else if (yc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion() || yc.IsParallel(xc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    _isApprox = true;
                }
                else if (!_keepParam)
                {
                    // Try building parabola with help of apex position
                    Curve3D? parabola = null;
                    _isApprox = !BuildParabolaByApex(ref parabola);
                    geomParabolaPtr = parabola as ParabolaCurve3D;
                }
                else
                {
                    _isApprox = true;
                }

                if (!_isApprox)
                {
                    // OCCT: in the line case GeomParabolaPtr is null (the line built above is never used), and
                    // GetTrimmedResult dereferences it (a crash in OCCT; NullReferenceException in the port).
                    GetTrimmedResult(geomParabolaPtr);
                }
                else
                {
                    BuildByApprox(AParabolaLimit);
                }
            }
            break;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
            {
                //     P(u) = O + R1 * std::cosh(u) * Xc + R2 * std::sinh(u) * Yc
                // ==> Q(u) = f(P(u))
                //          = f(O) + R1 * std::cosh(u) * f(Xc) + R2 * std::sinh(u) * f(Yc)

                Hyperbola3D hypr = _curve.Hyperbola();
                Frame3D axeRef = hypr.Position;
                Vector3D xc = ProjectVec(_plane, _direction, new Vector3D(axeRef.XDirection));
                Vector3D yc = ProjectVec(_plane, _direction, new Vector3D(axeRef.YDirection));
                Point3D p = ProjectPnt(_plane, _direction, axeRef.Location);
                double r1 = hypr.MajorRadius;
                double r2 = hypr.MinorRadius;
                Direction3D z = _plane.Direction;
                _isApprox = false;

                if (xc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    _type = global::Kerf.Geometry.CurveKind.Hyperbola;
                    Direction3D x = new Direction3D(yc) ^ z;
                    hypr = new Hyperbola3D(new Frame3D(p, z, x), 0.0, r2 * yc.Magnitude);
                    geomHyperbolaPtr = new HyperbolaCurve3D(hypr);
                }
                else if (yc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    _type = global::Kerf.Geometry.CurveKind.Hyperbola;
                    hypr = new Hyperbola3D(new Frame3D(p, z, new Direction3D(xc)), r1 * xc.Magnitude, 0.0);
                    geomHyperbolaPtr = new HyperbolaCurve3D(hypr);
                }
                else if (xc.IsNormal(yc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    _type = global::Kerf.Geometry.CurveKind.Hyperbola;
                    hypr = new Hyperbola3D(new Frame3D(p, new Direction3D(xc ^ yc), new Direction3D(xc)),
                                       r1 * xc.Magnitude,
                                       r2 * yc.Magnitude);
                    geomHyperbolaPtr = new HyperbolaCurve3D(hypr);
                }
                else if (yc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion() || yc.IsParallel(xc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    _isApprox = true;
                }
                else if (!_keepParam)
                {
                    Curve3D? hyperbola = null;
                    _isApprox = !BuildHyperbolaByApex(ref hyperbola);
                    geomHyperbolaPtr = hyperbola as HyperbolaCurve3D;
                }
                else
                {
                    _isApprox = true;
                }
                if (!_isApprox)
                {
                    GetTrimmedResult(geomHyperbolaPtr);
                }
                else
                {
                    BuildByApprox(AHyperbolaLimit);
                }
            }
            break;
            case BezierCurve:
            {
                BezierCurve3D bezierCurvePtr = _curve.Bezier();
                int nbPoles = bezierCurvePtr.PoleCount;

                // The poles are set on a copy: the caller's curve is unchanged, as in OCCT.
                BezierCurve3D projCu = (BezierCurve3D)bezierCurvePtr.Copy();

                _keepParam = true;
                _isApprox = false;
                _type = type;
                for (int i = 1; i <= nbPoles; i++)
                {
                    projCu.SetPole(i, ProjectPnt(_plane, _direction, bezierCurvePtr.Pole(i)));
                }

                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                _result = new CurveAdaptor(projCu);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
            case BSplineCurve:
            {
                BSplineCurve3D bSplineCurvePtr = _curve.BSpline();
                //
                //    make a copy of the curve and projects its poles
                //
                BSplineCurve3D projectedBSplinePtr = (BSplineCurve3D)bSplineCurvePtr.Copy();

                _keepParam = true;
                _isApprox = false;
                _type = type;
                for (int i = 1; i <= bSplineCurvePtr.PoleCount; i++)
                {
                    projectedBSplinePtr.SetPole(i, ProjectPnt(_plane, _direction, bSplineCurvePtr.Pole(i)));
                }

                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                _result = new CurveAdaptor(projectedBSplinePtr);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
            default:
            {
                _keepParam = true;
                _isApprox = true;
                _type = BSplineCurve;
                PerformApprox(_curve, _plane, _direction, ref approxCurve);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                // OCCT: when the approximation fails ApproxCurve stays null and GeomAdaptor_Curve throws
                // Standard_NullObject.
                _result = new CurveAdaptor(approxCurve!);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
        }
    }

    public CoordinateSystem GetPlane() => _plane;

    public Direction3D GetDirection() => _direction;

    /// <summary>The loaded curve (the caller's adaptor), null before <see cref="Load"/>.</summary>
    public CurveAdaptorBase? GetCurve() => _curve;

    /// <summary>
    /// The adaptor of the projected curve, null before <see cref="Load"/> (OCCT's handle: the object itself, not a
    /// copy). Its curve is frozen.
    /// </summary>
    public CurveAdaptor? GetResult()
    {
        _result?.Curve()?.Freeze();
        return _result;
    }

    public override double FirstParameter()
    {
        if (_keepParam || _isApprox)
        {
            return _curve!.FirstParameter();
        }
        else
        {
            return _firstPar;
        }
    }

    public override double LastParameter()
    {
        if (_keepParam || _isApprox)
        {
            return _curve!.LastParameter();
        }
        else
        {
            return _lastPar;
        }
    }

    public override Continuity Continuity() => _curve!.Continuity();

    public override int NbIntervals(Continuity s) => _curve!.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _curve!.Intervals(t, s);
    }

    /// <exception cref="MethodNotImplementedException">The projection has no result (type OtherCurve).</exception>
    public override CurveAdaptorBase Trim(double first, double last, double tolerance)
    {
        if (_type != OtherCurve)
        {
            return _result!.Trim(first, last, tolerance);
        }
        else
        {
            throw new MethodNotImplementedException("ProjLib_ProjectOnPlane::Trim() - curve of unsupported type");
        }
    }

    public override bool IsClosed => _curve!.IsClosed;

    public override bool IsPeriodic
    {
        get
        {
            if (_isApprox)
            {
                return false;
            }
            else
            {
                return _curve!.IsPeriodic;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is not periodic.</exception>
    public override double Period()
    {
        if (!IsPeriodic)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane::Period");
        }

        if (_isApprox)
        {
            return 0.0; // OCCT: returns false (unreachable: an approximation is not periodic)
        }
        else
        {
            return _curve!.Period();
        }
    }

    public sealed override Point3D EvalD0(double u)
    {
        if (_type != OtherCurve)
        {
            return _result!.EvalD0(u);
        }
        else
        {
            return OnPlane_Value(u, _curve!, _plane, _direction);
        }
    }

    internal sealed override Curve3D.ResD1 EvalD1(double u)
    {
        if (_type != OtherCurve)
        {
            return _result!.EvalD1(u);
        }
        else
        {
            Curve3D.ResD1 res = new();
            OnPlane_D1(u, out res.Point, out res.D1, _curve!, _plane, _direction);
            return res;
        }
    }

    internal sealed override Curve3D.ResD2 EvalD2(double u)
    {
        if (_type != OtherCurve)
        {
            return _result!.EvalD2(u);
        }
        else
        {
            Curve3D.ResD2 res = new();
            OnPlane_D2(u, out res.Point, out res.D1, out res.D2, _curve!, _plane, _direction);
            return res;
        }
    }

    internal sealed override Curve3D.ResD3 EvalD3(double u)
    {
        if (_type != OtherCurve)
        {
            return _result!.EvalD3(u);
        }
        else
        {
            Curve3D.ResD3 res = new();
            OnPlane_D3(u, out res.Point, out res.D1, out res.D2, out res.D3, _curve!, _plane, _direction);
            return res;
        }
    }

    public sealed override Vector3D EvalDN(double u, int n)
    {
        if (_type != OtherCurve)
        {
            return _result!.EvalDN(u, n);
        }
        else
        {
            return OnPlane_DN(u, n, _curve!, _plane, _direction);
        }
    }

    public override double Resolution(double tolerance)
    {
        if (_type != OtherCurve)
        {
            return _result!.Resolution(tolerance);
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The type of the projected curve. Hides <see cref="object.GetType"/> (decision 98).</summary>
    public override CurveKind GetType() => _type;

    /// <exception cref="NoSuchObjectException">The projection is not a line.</exception>
    public override Line3D Line()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Line)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Line");
        }

        return _result!.Line();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a circle.</exception>
    public override Circle3D Circle()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Circle");
        }

        return _result!.Circle();
    }

    /// <exception cref="NoSuchObjectException">The projection is not an ellipse.</exception>
    public override Ellipse3D Ellipse()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Ellipse)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Ellipse");
        }

        return _result!.Ellipse();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a hyperbola.</exception>
    public override Hyperbola3D Hyperbola()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Hyperbola");
        }

        return _result!.Hyperbola();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a parabola.</exception>
    public override Parabola3D Parabola()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Parabola)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Parabola");
        }

        return _result!.Parabola();
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override int Degree()
    {
        if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Degree");
        }

        if (_isApprox)
        {
            return _result!.Degree();
        }
        else
        {
            return _curve!.Degree();
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override bool IsRational
    {
        get
        {
            if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
            {
                throw new NoSuchObjectException("ProjLib_ProjectOnPlane:IsRational");
            }

            if (_isApprox)
            {
                return _result!.IsRational;
            }
            else
            {
                return _curve!.IsRational;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override int PoleCount
    {
        get
        {
            if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
            {
                throw new NoSuchObjectException("ProjLib_ProjectOnPlane:NbPoles");
            }

            if (_isApprox)
            {
                return _result!.PoleCount;
            }
            else
            {
                return _curve!.PoleCount;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is not a B-spline curve.</exception>
    public override int KnotCount
    {
        get
        {
            if (GetType() != BSplineCurve)
            {
                throw new NoSuchObjectException("ProjLib_ProjectOnPlane:NbKnots");
            }

            if (_isApprox)
            {
                return _result!.KnotCount;
            }
            else
            {
                return _curve!.KnotCount;
            }
        }
    }

    /// <summary>The projected Bezier curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not a Bezier curve.</exception>
    public override BezierCurve3D Bezier()
    {
        BezierCurve3D bezier = InternalBezier();
        bezier.Freeze();
        return bezier;
    }

    /// <summary>The projected B-spline curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not a B-spline curve.</exception>
    public override BSplineCurve3D BSpline()
    {
        BSplineCurve3D bSpline = InternalBSpline();
        bSpline.Freeze();
        return bSpline;
    }

    /// <summary>OCCT's <c>Bezier()</c> without freezing, for ported code that discards the projector.</summary>
    internal BezierCurve3D InternalBezier()
    {
        if (_type != BezierCurve)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Bezier");
        }

        return _result!.Bezier();
    }

    /// <summary>OCCT's <c>BSpline()</c> without freezing, for ported code that discards the projector.</summary>
    internal BSplineCurve3D InternalBSpline()
    {
        if (_type != BSplineCurve)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:BSpline");
        }

        return _result!.BSpline();
    }

    protected void GetTrimmedResult(Curve3D? projCurve)
    {
        Line3D lin = new();
        Parabola3D parab = new();
        Hyperbola3D hypr = new();
        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            lin = ((LineCurve3D)projCurve!).Line;
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Parabola)
        {
            parab = ((ParabolaCurve3D)projCurve!).Parabola;
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            hypr = ((HyperbolaCurve3D)projCurve!).Hyperbola;
        }

        _firstPar = projCurve!.FirstParameter;
        _lastPar = projCurve.LastParameter;
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(_curve!.FirstParameter()))
        {
            Point3D p = _curve.Value(_curve.FirstParameter());
            p = ProjectPnt(_plane, _direction, p);
            if (_type == global::Kerf.Geometry.CurveKind.Line)
            {
                _firstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, p);
            }
            else if (_type == global::Kerf.Geometry.CurveKind.Parabola)
            {
                _firstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, p);
            }
            else if (_type == global::Kerf.Geometry.CurveKind.Hyperbola)
            {
                _firstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p);
            }
            else
            {
                GeometryLibraryTool.Parameter(projCurve, p, global::Kerf.Engine.Foundation.Precision.Confusion(), out _firstPar);
            }
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(_curve.LastParameter()))
        {
            Point3D p = _curve.Value(_curve.LastParameter());
            p = ProjectPnt(_plane, _direction, p);
            if (_type == global::Kerf.Geometry.CurveKind.Line)
            {
                _lastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, p);
            }
            else if (_type == global::Kerf.Geometry.CurveKind.Parabola)
            {
                _lastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, p);
            }
            else if (_type == global::Kerf.Geometry.CurveKind.Hyperbola)
            {
                _lastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p);
            }
            else
            {
                GeometryLibraryTool.Parameter(projCurve, p, global::Kerf.Engine.Foundation.Precision.Confusion(), out _lastPar);
            }
        }
        _result = new CurveAdaptor(projCurve, _firstPar, _lastPar);
    }

    /// <summary>Builds the projected parabola from its apex, found as the point of maximal curvature.</summary>
    protected bool BuildParabolaByApex(ref Curve3D? geomParabolaPtr)
    {
        //
        // Searching parabola apex as point with maximal curvature
        double f = _curve!.Parabola().Focal;
        CurveKind curType = _type;
        _type = OtherCurve; // To provide correct calculation of derivativesb by projection for
                                     // copy of instance;
        CurveAdaptorBase projCrv = ShallowCopy();
        _type = curType;
        CLProps3d props = new(projCrv, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        ProjLib_MaxCurvature maxCur = new(props);
        BrentMinimum solver = new(global::Kerf.Engine.Foundation.Precision.PConfusion());
        solver.Perform(maxCur, -10.0 * f, 0.0, 10.0 * f);

        if (!solver.IsDone)
        {
            return false;
        }

        double t;
        t = solver.Location();
        props.SetParameter(t);
        Point3D p0 = props.Value();
        Vector3D dy = props.D1();
        Direction3D yDir = new(dy);
        Direction3D xDir;
        double curv = props.Curvature();
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(curv) || curv < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }
        props.Normal(out xDir);
        //
        Line3D xLine = new(p0, xDir);
        Point3D p1 = Value(t + 10.0 * f);
        //
        double x = global::Kerf.Engine.Geometry.ElCLib.LineParameter(xLine.Position, p1);
        double y = xLine.DistanceTo(p1);
        double newF = y * y / 4.0 / x;
        Direction3D n = xDir ^ yDir;
        Frame3D a2 = new(p0, n, xDir);
        MakeParab mkParab = new(a2, newF);
        if (!mkParab.IsDone)
        {
            return false;
        }

        Parabola3D projParab = mkParab.Value();

        _type = global::Kerf.Geometry.CurveKind.Parabola;
        geomParabolaPtr = new ParabolaCurve3D(projParab);
        // GetTrimmedResult(theGeomParabolaPtr);

        return true;
    }

    /// <summary>Builds the projected hyperbola from its apex, found as the point of maximal curvature.</summary>
    protected bool BuildHyperbolaByApex(ref Curve3D? geomHyperbolaPtr)
    {
        // Try to build hyperbola with help of apex position
        CurveKind curType = _type;
        _type = OtherCurve; // To provide correct calculation of derivativesb by projection for
                                     // copy of instance;
        CurveAdaptorBase projCrv = ShallowCopy();
        _type = curType;
        // Searching hyperbola apex as point with maximal curvature
        CLProps3d props = new(projCrv, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        ProjLib_MaxCurvature maxCur = new(props);
        BrentMinimum solver = new(global::Kerf.Engine.Foundation.Precision.PConfusion());
        solver.Perform(maxCur, -5.0, 0.0, 5.0);

        if (solver.IsDone)
        {
            double t;
            t = solver.Location();
            props.SetParameter(t);
            double curv = props.Curvature();
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(curv) || curv < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
            else
            {
                Hyperbola3D Hypr = _curve!.Hyperbola();
                Frame3D axeRef = Hypr.Position;
                Point3D p = ProjectPnt(_plane, _direction, axeRef.Location);
                Direction3D z = _plane.Direction;
                Point3D p0 = props.Value();
                Direction3D xDir = new MakeDir(p, p0).Value();
                Direction3D yDir = new MakeDir(props.D1()).Value();
                //
                double majRad = p.DistanceTo(p0);
                Point3D p1 = Value(t + 1.0);
                Vector3D v = new(p, p1);
                double x = v * new Vector3D(xDir);
                double y = v * new Vector3D(yDir);
                double minRad = y / Math.Sqrt(x * x / majRad / majRad - 1.0);
                Frame3D a2 = new(p, z, xDir);
                Hyperbola3D hypr = new(a2, majRad, minRad);
                geomHyperbolaPtr = new HyperbolaCurve3D(hypr);
                _type = global::Kerf.Geometry.CurveKind.Hyperbola;
            }
        }
        else
        {
            return false;
        }
        return true;
    }

    /// <summary>Approximates the projection by a B-spline, bounding an infinite curve by
    /// <paramref name="limitParameter"/>.</summary>
    protected void BuildByApprox(double limitParameter)
    {
        _type = BSplineCurve;
        BSplineCurve3D? approxCurve = null;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_curve!.FirstParameter()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(_curve.LastParameter()))
        {
            // To avoid exception in approximation
            double f = Max(-limitParameter, _curve.FirstParameter());
            double l = Min(limitParameter, _curve.LastParameter());
            CurveAdaptorBase trimCurve = _curve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
            PerformApprox(trimCurve, _plane, _direction, ref approxCurve);
        }
        else
        {
            PerformApprox(_curve, _plane, _direction, ref approxCurve);
        }
        // OCCT: a failed approximation leaves anApproxCurve null and dereferences it (NullReferenceException in
        // the port).
        _firstPar = approxCurve!.FirstParameter;
        _lastPar = approxCurve.LastParameter;
        _result = new CurveAdaptor(approxCurve);
    }
}
