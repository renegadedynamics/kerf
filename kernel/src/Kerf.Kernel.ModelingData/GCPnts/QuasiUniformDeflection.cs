// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Measurement.DeflectionType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes points on a curve such that the deflection between the curve and the polygon through the points stays
/// below a given value, by recursive subdivision. Quicker than GCPnts_UniformDeflection and works on curves that
/// are not C2, but gives more points. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
public sealed class QuasiUniformDeflection
{
    private const int MyMaxQuasiFleshe = 2000;

    private bool _done;
    private double _deflection;
    private readonly NCollection_Sequence<double> _params = new();
    private readonly NCollection_Sequence<Point3D> _points = new();
    private Continuity _cont;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public QuasiUniformDeflection()
    {
        _done = false;
        _deflection = 0.0;
        _cont = C1;
    }

    /// <summary>
    /// Points on the part theU1..theU2 of the curve with the deflection theDeflection. With theContinuity above G1
    /// the deflection is estimated from the derivatives too.
    /// </summary>
    public QuasiUniformDeflection(CurveAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity = C1)
    {
        _done = false;
        _deflection = deflection;
        _cont = C1;
        Initialize(c, deflection, u1, u2, continuity);
    }

    /// <summary>Points on the part theU1..theU2 of the curve with the deflection theDeflection.</summary>
    public QuasiUniformDeflection(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity = C1)
    {
        _done = false;
        _deflection = deflection;
        _cont = C1;
        Initialize(c, deflection, u1, u2, continuity);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public QuasiUniformDeflection(CurveAdaptorBase c, double deflection,
        Continuity continuity = C1)
    {
        _done = false;
        _deflection = deflection;
        _cont = C1;
        Initialize(c, deflection, continuity);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public QuasiUniformDeflection(Curve2DAdaptorBase c, double deflection,
        Continuity continuity = C1)
    {
        _done = false;
        _deflection = deflection;
        _cont = C1;
        Initialize(c, deflection, continuity);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(CurveAdaptorBase c, double deflection, Continuity continuity = C1)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), continuity);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, Continuity continuity = C1)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), continuity);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(CurveAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity = C1)
    {
        InitializeCore(c, deflection, u1, u2, continuity);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity = C1)
    {
        InitializeCore(c, deflection, u1, u2, continuity);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int PointCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformDeflection::NbPoints()");
            return _params.Length();
        }
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int index)
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformDeflection::Parameter()");
        return _params[index];
    }

    /// <summary>The point of index theIndex (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point3D Value(int index)
    {
        // OCCT names the wrong class and method in the message (occt-issues.md).
        StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformAbscissa::Parameter()");
        return _points.Value(index);
    }

    /// <summary>The deflection given to the algorithm.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Deflection()
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformDeflection::Deflection()");
        return _deflection;
    }

    // mask the return of a Adaptor2d_Curve2d as a gp_Pnt
    private static Point3D Value(CurveAdaptorBase c, double parameter)
    {
        return c.Value(parameter);
    }

    private static Point3D Value(Curve2DAdaptorBase c, double parameter)
    {
        Point3D point = new();
        Point2D a2dPoint = c.Value(parameter);
        point = point.WithCoord(a2dPoint.X, a2dPoint.Y, 0.0);
        return point;
    }

    private static void D1(CurveAdaptorBase c, double parameter, out Point3D p, out Vector3D v)
    {
        c.D1(parameter, out p, out v);
    }

    private static void D1(Curve2DAdaptorBase c, double parameter, out Point3D p, out Vector3D v)
    {
        c.D1(parameter, out Point2D a2dPoint, out Vector2D a2dVec);
        p = new Point3D();
        v = new Vector3D();
        p = p.WithCoord(a2dPoint.X, a2dPoint.Y, 0.0);
        v = v.WithCoord(a2dVec.X, a2dVec.Y, 0.0);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    private static void QuasiFleche(CurveAdaptorBase c,
                                    double deflection2,
                                    double udeb,
                                    in Point3D pdeb,
                                    in Vector3D vdeb,
                                    double ufin,
                                    in Point3D pfin,
                                    in Vector3D vfin,
                                    int nbmin,
                                    double eps,
                                    NCollection_Sequence<double> parameters,
                                    NCollection_Sequence<Point3D> points,
                                    ref int nbCalls)
    {
        ++nbCalls;
        if (nbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int ptslength = points.Length();
        if (nbCalls > 100 && ptslength < 2)
        {
            return;
        }

        double udelta = ufin - udeb;
        Point3D pdelta;
        Vector3D vdelta;
        if (nbmin > 2)
        {
            udelta /= (nbmin - 1);
            D1(c, udeb + udelta, out pdelta, out vdelta);
        }
        else
        {
            pdelta = pfin;
            vdelta = vfin;
        }

        // square length of chord
        double norme = new Vector3D(pdeb, pdelta).SquareMagnitude;
        double fleche = 0.0;
        bool isFlecheOk = false;
        if (norme > eps && norme > 16.0 * deflection2)
        {
            // Evaluation of the deflection by interpolation. See IntWalk_IWalking::TestDeflection
            double n1 = vdeb.SquareMagnitude;
            double n2 = vdelta.SquareMagnitude;
            if (n1 > eps && n2 > eps)
            {
                // square distance between ends of two normalized vectors [0; 4]
                double normediff = (vdeb.Normalized().XYZ() - vdelta.Normalized().XYZ()).SquareModulus();
                if (normediff > eps)
                {
                    fleche = normediff * norme / 64.0;
                    // So, fleche <= (aNorme / 16), independently of Vdeb and Vdelta.
                    // And if (aNorme / 16) < theDeflection2, this approach gives
                    // fleche < theDeflection2 independently of real curve.
                    // That is why we exclude case aNorme < (16. * theDeflection2)
                    isFlecheOk = true;
                }
            }
        }

        Point3D pmid = new((pdeb.XYZ() + pdelta.XYZ()) * 0.5);
        Point3D pverif = Value(c, udeb + udelta * 0.5);
        double flecheMidMid = pmid.SquareDistanceTo(pverif);

        if (isFlecheOk)
        {
            // Algorithm, evaluating "fleche" by interpolation,
            // can give false-positive result.
            // So we check also distance between Pmid and Pverif (aFlecheMidMid).
            // But aFlecheMidMid gives worse result in case of non-uniform parameterisation.
            // Maximum aFlecheMidMid, that seems reasonable, is (chord/2)^2 + theDeflection2
            //   .---------------.Pverif  .
            //   |               |        | Deflection
            //   ._______. ______.        .
            // Pdeb    Pmid    Pdelta
            if (flecheMidMid > norme / 4.0 + deflection2)
            // if (aFlecheMidMid > aNorme/4.)
            {
                fleche = flecheMidMid;
            }
        }
        else
        {
            fleche = flecheMidMid;
        }

        if (fleche < deflection2)
        {
            parameters.Append(udeb + udelta);
            points.Append(pdelta);
        }
        else
        {
            QuasiFleche(c,
                        deflection2,
                        udeb,
                        pdeb,
                        vdeb,
                        udeb + udelta,
                        pdelta,
                        vdelta,
                        3,
                        eps,
                        parameters,
                        points,
                        ref nbCalls);
        }

        if (nbmin > 2)
        {
            QuasiFleche(c,
                        deflection2,
                        udeb + udelta,
                        pdelta,
                        vdelta,
                        ufin,
                        pfin,
                        vfin,
                        nbmin - (points.Length() - ptslength),
                        eps,
                        parameters,
                        points,
                        ref nbCalls);
        }
        --nbCalls;
    }

    private static void QuasiFleche(CurveAdaptorBase c,
                                    double deflection2,
                                    double udeb,
                                    in Point3D pdeb,
                                    double ufin,
                                    in Point3D pfin,
                                    int nbmin,
                                    NCollection_Sequence<double> parameters,
                                    NCollection_Sequence<Point3D> points,
                                    ref int nbCalls)
    {
        ++nbCalls;
        if (nbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int ptslength = points.Length();
        if (nbCalls > 100 && ptslength < 2)
        {
            return;
        }

        double udelta = ufin - udeb;
        Point3D pdelta;
        if (nbmin > 2)
        {
            udelta /= (nbmin - 1);
            pdelta = Value(c, udeb + udelta);
        }
        else
        {
            pdelta = pfin;
        }

        Point3D pmid = new((pdeb.XYZ() + pdelta.XYZ()) * 0.5);
        Point3D pverif = Value(c, udeb + udelta * 0.5);
        double fleche = pmid.SquareDistanceTo(pverif);
        if (fleche < deflection2)
        {
            parameters.Append(udeb + udelta);
            points.Append(pdelta);
        }
        else
        {
            QuasiFleche(c,
                        deflection2,
                        udeb,
                        pdeb,
                        udeb + udelta * 0.5,
                        pverif,
                        2,
                        parameters,
                        points,
                        ref nbCalls);

            QuasiFleche(c,
                        deflection2,
                        udeb + udelta * 0.5,
                        pverif,
                        udeb + udelta,
                        pdelta,
                        2,
                        parameters,
                        points,
                        ref nbCalls);
        }

        if (nbmin > 2)
        {
            QuasiFleche(c,
                        deflection2,
                        udeb + udelta,
                        pdelta,
                        ufin,
                        pfin,
                        nbmin - (points.Length() - ptslength),
                        parameters,
                        points,
                        ref nbCalls);
        }
        --nbCalls;
    }

    private static bool PerformLinear(CurveAdaptorBase c,
                                      NCollection_Sequence<double> parameters,
                                      NCollection_Sequence<Point3D> points,
                                      double u1,
                                      double u2)
    {
        parameters.Append(u1);
        Point3D point = Value(c, u1);
        points.Append(point);

        parameters.Append(u2);
        point = Value(c, u2);
        points.Append(point);
        return true;
    }

    private static bool PerformCircular(CurveAdaptorBase c,
                                        NCollection_Sequence<double> parameters,
                                        NCollection_Sequence<Point3D> points,
                                        double deflection,
                                        double u1,
                                        double u2)
    {
        double angle = Max(1.0 - (deflection / c.Circle().Radius), 0.0);
        angle = 2.0 * Math.Acos(angle);
        int nbPoints = (int)((u2 - u1) / angle);
        nbPoints += 2;
        angle = (u2 - u1) / (double)(nbPoints - 1);
        double u = u1;
        for (int i = 1; i <= nbPoints; ++i)
        {
            parameters.Append(u);
            Point3D point = Value(c, u);
            points.Append(point);
            u += angle;
        }
        return true;
    }

    private static DeflectionType GetDefType(CurveAdaptorBase c)
    {
        if (c.NbIntervals(C1) > 1)
        {
            return DefComposite;
        }
        // to force subdivisions at discontinuities.
        // G1 should work, but gives exceptions...

        switch (c.GetType())
        {
            case Line:
                return Linear;
            case Circle:
                return Circular;
            case BSplineCurve:
            {
                BSplineCurve3D bs = c.BSpline();
                return (bs.PoleCount == 2) ? Linear : Curved;
            }
            case BezierCurve:
            {
                BezierCurve3D bz = c.Bezier();
                return (bz.PoleCount == 2) ? Linear : Curved;
            }
            default:
                return Curved;
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> parameters,
                                     NCollection_Sequence<Point3D> points,
                                     CurveAdaptorBase c,
                                     double deflection,
                                     double u1,
                                     double u2,
                                     double epsilon,
                                     Continuity continuity)
    {
        int nbmin = 2;
        int nbCallQF = 0;

        Point3D pdeb;
        if (continuity <= G1)
        {
            pdeb = Value(c, u1);
            parameters.Append(u1);
            points.Append(pdeb);

            Point3D pfin = Value(c, u2);
            QuasiFleche(c,
                        deflection * deflection,
                        u1,
                        pdeb,
                        u2,
                        pfin,
                        nbmin,
                        parameters,
                        points,
                        ref nbCallQF);
        }
        else
        {
            Point3D pfin;
            Vector3D ddeb, dfin;
            D1(c, u1, out pdeb, out ddeb);
            parameters.Append(u1);
            points.Append(pdeb);

            double decreasedU2 = u2 - Epsilon(u2) * 10.0;
            D1(c, decreasedU2, out pfin, out dfin);
            QuasiFleche(c,
                        deflection * deflection,
                        u1,
                        pdeb,
                        ddeb,
                        u2,
                        pfin,
                        dfin,
                        nbmin,
                        epsilon * epsilon,
                        parameters,
                        points,
                        ref nbCallQF);
        }
        //  cout << "Nb of pts: " << Points.Length()<< endl;
        return true;
    }

    private static bool PerformComposite(NCollection_Sequence<double> parameters,
                                         NCollection_Sequence<Point3D> points,
                                         CurveAdaptorBase c,
                                         double deflection,
                                         double u1,
                                         double u2,
                                         double epsilon,
                                         Continuity continuity)
    {
        // Consistency with Intervals
        int nbIntervals = c.NbIntervals(C2);
        int pIndex;
        NCollection_Array1<double> ti = new(1, nbIntervals + 1);
        c.Intervals(ti, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u1, out pIndex);

        // iterate by continuous segments
        double ua = u1;
        for (int index = pIndex; ;)
        {
            double ub = index + 1 <= ti.Upper() ? Min(u2, ti[index + 1]) : u2;
            if (!PerformCurve(parameters,
                              points,
                              c,
                              deflection,
                              ua,
                              ub,
                              epsilon,
                              continuity))
            {
                return false;
            }

            ++index;
            if (index > nbIntervals || u2 < ti[index])
            {
                return true;
            }

            // remove last point to avoid duplication
            parameters.Remove(parameters.Length());
            points.Remove(points.Length());

            ua = ub;
        }
    }

    private void InitializeCore(CurveAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity)
    {
        _cont = (continuity > G1) ? C1 : C0;
        _deflection = deflection;
        _done = false;
        _params.Clear();
        _points.Clear();

        double epsilon = Min(c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), 1.0e50);
        DeflectionType type = GetDefType(c);
        double aU1 = Min(u1, u2);
        double aU2 = Max(u1, u2);

        if (type == Curved || type == DefComposite)
        {
            if (c.GetType() == BSplineCurve || c.GetType() == BezierCurve)
            {
                double maxPar = Max(Math.Abs(c.FirstParameter()), Math.Abs(c.LastParameter()));
                if (epsilon < Epsilon(maxPar))
                {
                    return;
                }
            }
        }

        switch (type)
        {
            case Linear:
            {
                _done = PerformLinear(c, _params, _points, aU1, aU2);
                break;
            }
            case Circular:
            {
                _done = PerformCircular(c, _params, _points, deflection, aU1, aU2);
                break;
            }
            case Curved:
            {
                _done = PerformCurve(_params, _points, c, deflection, aU1, aU2, epsilon, _cont);
                break;
            }
            case DefComposite:
            {
                _done = PerformComposite(_params, _points, c, deflection, aU1, aU2, epsilon, _cont);
                break;
            }
        }
    }

    // ---- Templates over the curve type (2D instantiation) ----

    private static void QuasiFleche(Curve2DAdaptorBase c,
                                    double deflection2,
                                    double udeb,
                                    in Point3D pdeb,
                                    in Vector3D vdeb,
                                    double ufin,
                                    in Point3D pfin,
                                    in Vector3D vfin,
                                    int nbmin,
                                    double eps,
                                    NCollection_Sequence<double> parameters,
                                    NCollection_Sequence<Point3D> points,
                                    ref int nbCalls)
    {
        ++nbCalls;
        if (nbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int ptslength = points.Length();
        if (nbCalls > 100 && ptslength < 2)
        {
            return;
        }

        double udelta = ufin - udeb;
        Point3D pdelta;
        Vector3D vdelta;
        if (nbmin > 2)
        {
            udelta /= (nbmin - 1);
            D1(c, udeb + udelta, out pdelta, out vdelta);
        }
        else
        {
            pdelta = pfin;
            vdelta = vfin;
        }

        // square length of chord
        double norme = new Vector3D(pdeb, pdelta).SquareMagnitude;
        double fleche = 0.0;
        bool isFlecheOk = false;
        if (norme > eps && norme > 16.0 * deflection2)
        {
            // Evaluation of the deflection by interpolation. See IntWalk_IWalking::TestDeflection
            double n1 = vdeb.SquareMagnitude;
            double n2 = vdelta.SquareMagnitude;
            if (n1 > eps && n2 > eps)
            {
                // square distance between ends of two normalized vectors [0; 4]
                double normediff = (vdeb.Normalized().XYZ() - vdelta.Normalized().XYZ()).SquareModulus();
                if (normediff > eps)
                {
                    fleche = normediff * norme / 64.0;
                    // So, fleche <= (aNorme / 16), independently of Vdeb and Vdelta.
                    // And if (aNorme / 16) < theDeflection2, this approach gives
                    // fleche < theDeflection2 independently of real curve.
                    // That is why we exclude case aNorme < (16. * theDeflection2)
                    isFlecheOk = true;
                }
            }
        }

        Point3D pmid = new((pdeb.XYZ() + pdelta.XYZ()) * 0.5);
        Point3D pverif = Value(c, udeb + udelta * 0.5);
        double flecheMidMid = pmid.SquareDistanceTo(pverif);

        if (isFlecheOk)
        {
            // Algorithm, evaluating "fleche" by interpolation,
            // can give false-positive result.
            // So we check also distance between Pmid and Pverif (aFlecheMidMid).
            // But aFlecheMidMid gives worse result in case of non-uniform parameterisation.
            // Maximum aFlecheMidMid, that seems reasonable, is (chord/2)^2 + theDeflection2
            //   .---------------.Pverif  .
            //   |               |        | Deflection
            //   ._______. ______.        .
            // Pdeb    Pmid    Pdelta
            if (flecheMidMid > norme / 4.0 + deflection2)
            // if (aFlecheMidMid > aNorme/4.)
            {
                fleche = flecheMidMid;
            }
        }
        else
        {
            fleche = flecheMidMid;
        }

        if (fleche < deflection2)
        {
            parameters.Append(udeb + udelta);
            points.Append(pdelta);
        }
        else
        {
            QuasiFleche(c,
                        deflection2,
                        udeb,
                        pdeb,
                        vdeb,
                        udeb + udelta,
                        pdelta,
                        vdelta,
                        3,
                        eps,
                        parameters,
                        points,
                        ref nbCalls);
        }

        if (nbmin > 2)
        {
            QuasiFleche(c,
                        deflection2,
                        udeb + udelta,
                        pdelta,
                        vdelta,
                        ufin,
                        pfin,
                        vfin,
                        nbmin - (points.Length() - ptslength),
                        eps,
                        parameters,
                        points,
                        ref nbCalls);
        }
        --nbCalls;
    }

    private static void QuasiFleche(Curve2DAdaptorBase c,
                                    double deflection2,
                                    double udeb,
                                    in Point3D pdeb,
                                    double ufin,
                                    in Point3D pfin,
                                    int nbmin,
                                    NCollection_Sequence<double> parameters,
                                    NCollection_Sequence<Point3D> points,
                                    ref int nbCalls)
    {
        ++nbCalls;
        if (nbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int ptslength = points.Length();
        if (nbCalls > 100 && ptslength < 2)
        {
            return;
        }

        double udelta = ufin - udeb;
        Point3D pdelta;
        if (nbmin > 2)
        {
            udelta /= (nbmin - 1);
            pdelta = Value(c, udeb + udelta);
        }
        else
        {
            pdelta = pfin;
        }

        Point3D pmid = new((pdeb.XYZ() + pdelta.XYZ()) * 0.5);
        Point3D pverif = Value(c, udeb + udelta * 0.5);
        double fleche = pmid.SquareDistanceTo(pverif);
        if (fleche < deflection2)
        {
            parameters.Append(udeb + udelta);
            points.Append(pdelta);
        }
        else
        {
            QuasiFleche(c,
                        deflection2,
                        udeb,
                        pdeb,
                        udeb + udelta * 0.5,
                        pverif,
                        2,
                        parameters,
                        points,
                        ref nbCalls);

            QuasiFleche(c,
                        deflection2,
                        udeb + udelta * 0.5,
                        pverif,
                        udeb + udelta,
                        pdelta,
                        2,
                        parameters,
                        points,
                        ref nbCalls);
        }

        if (nbmin > 2)
        {
            QuasiFleche(c,
                        deflection2,
                        udeb + udelta,
                        pdelta,
                        ufin,
                        pfin,
                        nbmin - (points.Length() - ptslength),
                        parameters,
                        points,
                        ref nbCalls);
        }
        --nbCalls;
    }

    private static bool PerformLinear(Curve2DAdaptorBase c,
                                      NCollection_Sequence<double> parameters,
                                      NCollection_Sequence<Point3D> points,
                                      double u1,
                                      double u2)
    {
        parameters.Append(u1);
        Point3D point = Value(c, u1);
        points.Append(point);

        parameters.Append(u2);
        point = Value(c, u2);
        points.Append(point);
        return true;
    }

    private static bool PerformCircular(Curve2DAdaptorBase c,
                                        NCollection_Sequence<double> parameters,
                                        NCollection_Sequence<Point3D> points,
                                        double deflection,
                                        double u1,
                                        double u2)
    {
        double angle = Max(1.0 - (deflection / c.Circle().Radius), 0.0);
        angle = 2.0 * Math.Acos(angle);
        int nbPoints = (int)((u2 - u1) / angle);
        nbPoints += 2;
        angle = (u2 - u1) / (double)(nbPoints - 1);
        double u = u1;
        for (int i = 1; i <= nbPoints; ++i)
        {
            parameters.Append(u);
            Point3D point = Value(c, u);
            points.Append(point);
            u += angle;
        }
        return true;
    }

    private static DeflectionType GetDefType(Curve2DAdaptorBase c)
    {
        if (c.NbIntervals(C1) > 1)
        {
            return DefComposite;
        }
        // to force subdivisions at discontinuities.
        // G1 should work, but gives exceptions...

        switch (c.GetType())
        {
            case Line:
                return Linear;
            case Circle:
                return Circular;
            case BSplineCurve:
            {
                BSplineCurve2D bs = c.BSpline();
                return (bs.PoleCount == 2) ? Linear : Curved;
            }
            case BezierCurve:
            {
                BezierCurve2D bz = c.Bezier();
                return (bz.PoleCount == 2) ? Linear : Curved;
            }
            default:
                return Curved;
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> parameters,
                                     NCollection_Sequence<Point3D> points,
                                     Curve2DAdaptorBase c,
                                     double deflection,
                                     double u1,
                                     double u2,
                                     double epsilon,
                                     Continuity continuity)
    {
        int nbmin = 2;
        int nbCallQF = 0;

        Point3D pdeb;
        if (continuity <= G1)
        {
            pdeb = Value(c, u1);
            parameters.Append(u1);
            points.Append(pdeb);

            Point3D pfin = Value(c, u2);
            QuasiFleche(c,
                        deflection * deflection,
                        u1,
                        pdeb,
                        u2,
                        pfin,
                        nbmin,
                        parameters,
                        points,
                        ref nbCallQF);
        }
        else
        {
            Point3D pfin;
            Vector3D ddeb, dfin;
            D1(c, u1, out pdeb, out ddeb);
            parameters.Append(u1);
            points.Append(pdeb);

            double decreasedU2 = u2 - Epsilon(u2) * 10.0;
            D1(c, decreasedU2, out pfin, out dfin);
            QuasiFleche(c,
                        deflection * deflection,
                        u1,
                        pdeb,
                        ddeb,
                        u2,
                        pfin,
                        dfin,
                        nbmin,
                        epsilon * epsilon,
                        parameters,
                        points,
                        ref nbCallQF);
        }
        //  cout << "Nb of pts: " << Points.Length()<< endl;
        return true;
    }

    private static bool PerformComposite(NCollection_Sequence<double> parameters,
                                         NCollection_Sequence<Point3D> points,
                                         Curve2DAdaptorBase c,
                                         double deflection,
                                         double u1,
                                         double u2,
                                         double epsilon,
                                         Continuity continuity)
    {
        // Consistency with Intervals
        int nbIntervals = c.NbIntervals(C2);
        int pIndex;
        NCollection_Array1<double> ti = new(1, nbIntervals + 1);
        c.Intervals(ti, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u1, out pIndex);

        // iterate by continuous segments
        double ua = u1;
        for (int index = pIndex; ;)
        {
            double ub = index + 1 <= ti.Upper() ? Min(u2, ti[index + 1]) : u2;
            if (!PerformCurve(parameters,
                              points,
                              c,
                              deflection,
                              ua,
                              ub,
                              epsilon,
                              continuity))
            {
                return false;
            }

            ++index;
            if (index > nbIntervals || u2 < ti[index])
            {
                return true;
            }

            // remove last point to avoid duplication
            parameters.Remove(parameters.Length());
            points.Remove(points.Length());

            ua = ub;
        }
    }

    private void InitializeCore(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        Continuity continuity)
    {
        _cont = (continuity > G1) ? C1 : C0;
        _deflection = deflection;
        _done = false;
        _params.Clear();
        _points.Clear();

        double epsilon = Min(c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), 1.0e50);
        DeflectionType type = GetDefType(c);
        double aU1 = Min(u1, u2);
        double aU2 = Max(u1, u2);

        if (type == Curved || type == DefComposite)
        {
            if (c.GetType() == BSplineCurve || c.GetType() == BezierCurve)
            {
                double maxPar = Max(Math.Abs(c.FirstParameter()), Math.Abs(c.LastParameter()));
                if (epsilon < Epsilon(maxPar))
                {
                    return;
                }
            }
        }

        switch (type)
        {
            case Linear:
            {
                _done = PerformLinear(c, _params, _points, aU1, aU2);
                break;
            }
            case Circular:
            {
                _done = PerformCircular(c, _params, _points, deflection, aU1, aU2);
                break;
            }
            case Curved:
            {
                _done = PerformCurve(_params, _points, c, deflection, aU1, aU2, epsilon, _cont);
                break;
            }
            case DefComposite:
            {
                _done = PerformComposite(_params, _points, c, deflection, aU1, aU2, epsilon, _cont);
                break;
            }
        }
    }
}
