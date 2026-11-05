// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.lxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a surface S(u, v) (and possibly associated 2D curves) defined by a law of sections: F(u, v) where
/// Ft(u) = F(u, t) is a B-spline curve, given by an <see cref="SweepFunction"/>. Used by blending and sweeping.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Perform"/> runs the approximation. The accessors <see cref="SurfPoles"/>,
/// <see cref="SurfWeights"/>, the knot and multiplicity accessors and <see cref="Curve2dPoles"/> return the object's
/// own arrays (OCCT's const references; shared: don't modify); <see cref="Surface"/> and <see cref="Curve2d"/> copy
/// into the caller's arrays.
/// </remarks>
internal sealed class SweepApproximation
{
    private sealed class Approx_SweepApproximation_Eval : EvaluatorFunction
    {
        private readonly SweepApproximation _tool;

        public Approx_SweepApproximation_Eval(SweepApproximation tool)
        {
            _tool = tool;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int errorCode)
        {
            errorCode = _tool.Eval(parameter, derivativeRequest, startEnd[0], startEnd[1], result);
        }
    }

    private readonly SweepFunction _func;
    private bool _done;
    private int _num1DSS;
    private int _num2DSS;
    private int _num3DSS;
    private int _udeg;
    private int _vdeg;
    private int _deg2d;
    private NCollection_HArray2<Point3D>? _tabPoles;
    private NCollection_HArray2<double>? _tabWeights;
    private NCollection_HArray1<double>? _tabUKnots;
    private NCollection_HArray1<double>? _tabVKnots;
    private NCollection_HArray1<double>? _tab2dKnots;
    private NCollection_HArray1<int>? _tabUMults;
    private NCollection_HArray1<int>? _tabVMults;
    private NCollection_HArray1<int>? _tab2dMults;
    private readonly NCollection_Sequence<NCollection_HArray1<Point2D>> _seqPoles2d = new();
    private NCollection_HArray1<double>? _mError1d;
    private NCollection_HArray1<double>? _tab2dError;
    private NCollection_HArray1<double>? _mError3d;
    private NCollection_HArray1<double>? _aError1d;
    private NCollection_HArray1<double>? _ave2dError;
    private NCollection_HArray1<double>? _aError3d;
    private NCollection_HArray1<GeneralTransform2D>? _aAffin;
    private NCollection_HArray1<double>? _cOnSurfErr;
    private Vector3D _translation = new();
    private NCollection_HArray1<Point3D>? _poles;
    private NCollection_HArray1<Point2D>? _poles2d;
    private NCollection_HArray1<double>? _weigths;
    private NCollection_HArray1<Vector3D>? _dPoles;
    private NCollection_HArray1<Vector3D>? _d2Poles;
    private NCollection_HArray1<Vector2D>? _dPoles2d;
    private NCollection_HArray1<Vector2D>? _d2Poles2d;
    private NCollection_HArray1<double>? _dWeigths;
    private NCollection_HArray1<double>? _d2Weigths;
    private int _order;
    private double _param;
    private double _first;
    private double _last;

    /// <summary>An approximation of the sweep of Func (kept, and changed by the approximation as in OCCT).</summary>
    public SweepApproximation(SweepFunction func)
    {
        _func = func;
        //  Init of variables of control
        _param = 0;
        _order = -1;
        _first = 1.0e100;
        _last = -1.0e100;
        _done = false;
    }

    /// <summary>
    /// Performs the approximation on [First, Last]: Tol3d is the tolerance of the surface, BoundTol that at its
    /// boundaries, Tol2d the largest tolerance of the 2D curves (normally given by the resolutions of the support
    /// surfaces), TolAngular the angle (in radians) between the tangents of the section law and of the iso-v of the
    /// surface; Continuity, Degmax and Segmax the continuity, maximum degree and maximum number of spans in v.
    /// </summary>
    public void Perform(double first,
                        double last,
                        double tol3d,
                        double boundTol,
                        double tol2d,
                        double tolAngular,
                        Continuity Continuity = C0,
                        int degmax = 11,
                        int segmax = 50)
    {
        int nbPolSect, nbKnotSect, ii;
        double tol, tol3dMin = tol3d, the3D2DTol = 0;
        Continuity continuity = Continuity;

        // (1) Characteristics of a section
        _func.SectionShape(out nbPolSect, out nbKnotSect, out _udeg);
        _num2DSS = _func.Nb2dCurves();
        _tabUKnots = new NCollection_HArray1<double>(1, nbKnotSect);
        _tabUMults = new NCollection_HArray1<int>(1, nbKnotSect);
        _func.Knots(_tabUKnots.ChangeArray1());
        _func.Mults(_tabUMults.ChangeArray1());

        // (2) Decompositition into sub-spaces
        NCollection_HArray1<double>? oneDTol, twoDTol, threeDTol;
        _num3DSS = nbPolSect;

        // (2.1) Tolerance 3d and 1d
        oneDTol = new NCollection_HArray1<double>(1, _num3DSS);
        threeDTol = new NCollection_HArray1<double>(1, _num3DSS);

        _func.GetTolerance(boundTol, tol3d, tolAngular, threeDTol.ChangeArray1());

        for (ii = 1; ii <= _num3DSS; ii++)
        {
            if (threeDTol.Value(ii) < tol3dMin)
            {
                tol3dMin = threeDTol.Value(ii);
            }
        }

        if (_func.IsRational)
        {
            double size;
            _num1DSS = nbPolSect;
            NCollection_Array1<double> wmin = new(1, _num1DSS);
            _func.GetMinimalWeight(wmin);
            size = _func.MaximalSection();
            _translation = _translation.WithXYZ(_func.BarycentreOfSurf().XYZ());
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                tol = threeDTol.Value(ii) / 2; // To take account of the error on the final result.
                oneDTol.SetValue(ii, tol * wmin[ii] / size);
                tol *= wmin[ii]; // Factor of projection
                threeDTol.SetValue(ii, Max(tol, 1.0e-20));
            }
        }
        else
        {
            _num1DSS = 0;
        }

        // (2.2) Tolerance and Transformation 2d.
        if (_num2DSS == 0)
        {
            twoDTol = null;
        }
        else
        {
            // for 2d define affinity using resolutions, to
            // avoid homogeneous tolerance of approximation (u/v and 2d/3d)
            double res, tolu, tolv;
            twoDTol = new NCollection_HArray1<double>(1, _num2DSS);
            _aAffin = new NCollection_HArray1<GeneralTransform2D>(1, _num2DSS);
            the3D2DTol = 0.9 * boundTol; // 10% of security
            for (ii = 1; ii <= _num2DSS; ii++)
            {
                _func.Resolution(ii, the3D2DTol, out tolu, out tolv);
                if (tolu > tolv)
                {
                    res = tolv;
                    _aAffin[ii] = _aAffin[ii].WithValue(1, 1, tolv / tolu);
                }
                else
                {
                    res = tolu;
                    _aAffin[ii] = _aAffin[ii].WithValue(2, 2, tolu / tolv);
                }
                twoDTol.SetValue(ii, Min(tol2d, res));
            }
        }

        // (3) Approximation

        // Init
        _poles = new NCollection_HArray1<Point3D>(1, _num3DSS);
        _dPoles = new NCollection_HArray1<Vector3D>(1, _num3DSS);
        _d2Poles = new NCollection_HArray1<Vector3D>(1, _num3DSS);

        _weigths = new NCollection_HArray1<double>(1, _num3DSS);
        _dWeigths = new NCollection_HArray1<double>(1, _num3DSS);
        _d2Weigths = new NCollection_HArray1<double>(1, _num3DSS);

        if (_num2DSS > 0)
        {
            _poles2d = new NCollection_HArray1<Point2D>(1, _num2DSS);
            _dPoles2d = new NCollection_HArray1<Vector2D>(1, _num2DSS);
            _d2Poles2d = new NCollection_HArray1<Vector2D>(1, _num2DSS);
            _cOnSurfErr = new NCollection_HArray1<double>(1, _num2DSS);
        }
        else
        {
            _poles2d = new NCollection_HArray1<Point2D>();
            _dPoles2d = new NCollection_HArray1<Vector2D>();
            _d2Poles2d = new NCollection_HArray1<Vector2D>();
            _cOnSurfErr = new NCollection_HArray1<double>();
        }

        // Checks if myFunc->D2 is implemented
        if (continuity >= C2)
        {
            bool b;
            b = _func.D2(first,
                          first,
                          last,
                          _poles.ChangeArray1(),
                          _dPoles.ChangeArray1(),
                          _d2Poles.ChangeArray1(),
                          _poles2d.ChangeArray1(),
                          _dPoles2d.ChangeArray1(),
                          _d2Poles2d.ChangeArray1(),
                          _weigths.ChangeArray1(),
                          _dWeigths.ChangeArray1(),
                          _d2Weigths.ChangeArray1());
            if (!b)
            {
                continuity = C1;
            }
        }
        // Checks if myFunc->D1 is implemented
        if (continuity == C1)
        {
            bool b;
            b = _func.D1(first,
                          first,
                          last,
                          _poles.ChangeArray1(),
                          _dPoles.ChangeArray1(),
                          _poles2d.ChangeArray1(),
                          _dPoles2d.ChangeArray1(),
                          _weigths.ChangeArray1(),
                          _dWeigths.ChangeArray1());
            if (!b)
            {
                continuity = C0;
            }
        }

        // So that F was at least 20 times more exact than its approx
        _func.SetTolerance(tol3dMin / 20, tol2d / 20);

        int nbIntervalC2 = _func.NbIntervals(C2);
        int nbIntervalC3 = _func.NbIntervals(C3);

        if (nbIntervalC3 > 1)
        {
            // (3.1) Approximation with preferential cut
            NCollection_Array1<double> Param_de_decoupeC2 = new(1, nbIntervalC2 + 1);
            _func.Intervals(Param_de_decoupeC2, C2);
            NCollection_Array1<double> Param_de_decoupeC3 = new(1, nbIntervalC3 + 1);
            _func.Intervals(Param_de_decoupeC3, C3);

            PrefAndRec preferentiel = new(Param_de_decoupeC2, Param_de_decoupeC3);

            Approx_SweepApproximation_Eval ev = new(this);
            Approximation(oneDTol,
                          twoDTol,
                          threeDTol,
                          the3D2DTol,
                          first,
                          last,
                          continuity,
                          degmax,
                          segmax,
                          ev,
                          preferentiel);
        }
        else
        {
            // (3.2) Approximation without preferential cut
            DichoCutting dichotomie = new();
            Approx_SweepApproximation_Eval ev = new(this);
            Approximation(oneDTol,
                          twoDTol,
                          threeDTol,
                          the3D2DTol,
                          first,
                          last,
                          continuity,
                          degmax,
                          segmax,
                          ev,
                          dichotomie);
        }
    }

    // Calls F(t) and stores the results.
    private void Approximation(NCollection_HArray1<double>? oneDTol,
                               NCollection_HArray1<double>? twoDTol,
                               NCollection_HArray1<double>? threeDTol,
                               double boundTol,
                               double first,
                               double last,
                               Continuity continuity,
                               int degmax,
                               int segmax,
                               EvaluatorFunction theApproxFunction,
                               Cutting theCuttingTool)
    {
        ApproxAFunction approx = new(_num1DSS,
                                               _num2DSS,
                                               _num3DSS,
                                               oneDTol,
                                               twoDTol,
                                               threeDTol,
                                               first,
                                               last,
                                               continuity,
                                               degmax,
                                               segmax,
                                               theApproxFunction,
                                               theCuttingTool);
        _done = approx.HasResult;

        if (_done)
        {
            // --> Fill Champs of the surface ----
            int ii, jj;

            _vdeg = approx.Degree();
            int nbPoles = approx.NbPoles();
            // Unfortunately Adv_Approx stores the transposition of the required
            // so, writing tabPoles = Approx.Poles() will give an erroneous result
            // It is only possible to allocate and recopy term by term...
            _tabPoles = new NCollection_HArray2<Point3D>(1, _num3DSS, 1, nbPoles);
            _tabWeights = new NCollection_HArray2<double>(1, _num3DSS, 1, nbPoles);

            // OCCT reads Approx.Poles() (a handle) in the loops; the port's accessor copies, so it is read once.
            NCollection_HArray2<Point3D> poles = approx.Poles()!;
            if (_num1DSS == _num3DSS)
            {
                NCollection_HArray2<double> poles1d = approx.Poles1d()!;
                double wpoid;
                Point3D p;
                for (ii = 1; ii <= _num3DSS; ii++)
                {
                    for (jj = 1; jj <= nbPoles; jj++)
                    {
                        p = poles.Value(jj, ii);
                        wpoid = poles1d.Value(jj, ii);
                        p = new Point3D(p.XYZ().Divided(wpoid)); // It is necessary to divide poles by weight
                        p = p.Translated(_translation);
                        _tabPoles.SetValue(ii, jj, p);
                        _tabWeights.SetValue(ii, jj, wpoid);
                    }
                }
            }
            else
            {
                _tabWeights.Init(1);
                for (ii = 1; ii <= _num3DSS; ii++)
                {
                    for (jj = 1; jj <= nbPoles; jj++)
                    {
                        _tabPoles.SetValue(ii, jj, poles.Value(jj, ii));
                    }
                }
            }

            // this is better
            _tabVKnots = approx.Knots();
            _tabVMults = approx.Multiplicities();

            // --> Filling of curves 2D  ----------
            if (_num2DSS > 0)
            {
                GeneralTransform2D trsfInv;
                _deg2d = _vdeg;
                _tab2dKnots = approx.Knots();
                _tab2dMults = approx.Multiplicities();

                for (ii = 1; ii <= _num2DSS; ii++)
                {
                    trsfInv = _aAffin!.Value(ii).Inverted();
                    NCollection_HArray1<Point2D> p2d = new(1, nbPoles);
                    approx.Poles2d(ii, p2d.ChangeArray1());
                    // do not forget to apply inverted homothety.
                    for (jj = 1; jj <= nbPoles; jj++)
                    {
                        Xy coord = p2d[jj].XY();
                        trsfInv.Transforms(ref coord);
                        p2d[jj] = new Point2D(coord);
                    }
                    _seqPoles2d.Append(p2d);
                }
            }
            // ---> Filling of errors
            _mError3d = new NCollection_HArray1<double>(1, _num3DSS);
            _aError3d = new NCollection_HArray1<double>(1, _num3DSS);
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                _mError3d.SetValue(ii, approx.MaxError(3, ii));
                _aError3d.SetValue(ii, approx.AverageError(3, ii));
            }

            if (_func.IsRational)
            {
                _mError1d = new NCollection_HArray1<double>(1, _num3DSS);
                _aError1d = new NCollection_HArray1<double>(1, _num3DSS);
                for (ii = 1; ii <= _num1DSS; ii++)
                {
                    _mError1d.SetValue(ii, approx.MaxError(1, ii));
                    _aError1d.SetValue(ii, approx.AverageError(1, ii));
                }
            }

            if (_num2DSS > 0)
            {
                _tab2dError = new NCollection_HArray1<double>(1, _num2DSS);
                _ave2dError = new NCollection_HArray1<double>(1, _num2DSS);
                for (ii = 1; ii <= _num2DSS; ii++)
                {
                    _tab2dError.SetValue(ii, approx.MaxError(2, ii));
                    _ave2dError.SetValue(ii, approx.AverageError(2, ii));
                    _cOnSurfErr!.SetValue(ii, (_tab2dError.Value(ii) / twoDTol!.Value(ii)) * boundTol);
                }
            }
        }
    }

    /// <summary>
    /// The evaluator of AdvApprox: writes the derivative DerivativeRequest of the 1D, 2D and 3D poles at Parameter
    /// into Result, and returns the error code (0 when done).
    /// </summary>
    public int Eval(double parameter, int derivativeRequest, double first, double last, Span<double> result)
    {
        int ier = 0;
        switch (derivativeRequest)
        {
            case 0:
                ier = (!D0(parameter, first, last, result)) ? 1 : 0;
                break;
            case 1:
                ier = (!D1(parameter, first, last, result)) ? 1 : 0;
                break;
            case 2:
                ier = (!D2(parameter, first, last, result)) ? 1 : 0;
                break;
            default:
                ier = 2;
                break;
        }
        return ier;
    }

    private bool D0(double param, double first, double last, Span<double> localResult)
    {
        int index, ii;
        bool ok = true;

        // Management of limits
        if ((_first != first) || (last != _last))
        {
            _func.SetInterval(first, last);
        }

        if ((param != _param) || (_order < 0) || (_first != first) || (last != _last))
        {
            // Positioning in case when the last operation is not repeated.
            ok = _func.D0(param,
                           first,
                           last,
                           _poles!.ChangeArray1(),
                           _poles2d!.ChangeArray1(),
                           _weigths!.ChangeArray1());

            //  poles3d are multiplied by weight after translation.
            for (ii = 1; ii <= _num1DSS; ii++)
            {
                _poles[ii] = new Point3D(_poles[ii].XYZ().Subtracted(_translation.XYZ()));
                _poles[ii] = new Point3D(_poles[ii].XYZ().Multiplied(_weigths.Value(ii)));
            }

            //  The transformation is applied to poles 2d.
            for (ii = 1; ii <= _num2DSS; ii++)
            {
                Xy coord = _poles2d[ii].XY();
                _aAffin!.Value(ii).Transforms(ref coord);
                _poles2d[ii] = new Point2D(coord);
            }

            // Update variables of control and return
            _first = first;
            _last = last;
            _order = 0;
            _param = param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= _num1DSS; ii++)
        {
            localResult[index] = _weigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= _num2DSS; ii++)
        {
            localResult[index] = _poles2d!.Value(ii).X;
            localResult[index + 1] = _poles2d.Value(ii).Y;
            index += 2;
        }
        for (ii = 1; ii <= _num3DSS; ii++, index += 3)
        {
            localResult[index] = _poles!.Value(ii).X;
            localResult[index + 1] = _poles.Value(ii).Y;
            localResult[index + 2] = _poles.Value(ii).Z;
        }

        return ok;
    }

    private bool D1(double param, double first, double last, Span<double> localResult)
    {
        Xy vcoord;
        Vector3D vaux = new();
        int index, ii;
        bool ok = true;

        if ((_first != first) || (last != _last))
        {
            _func.SetInterval(first, last);
        }

        if ((param != _param) || (_order < 1) || (_first != first) || (last != _last))
        {
            // Positioning
            ok = _func.D1(param,
                           first,
                           last,
                           _poles!.ChangeArray1(),
                           _dPoles!.ChangeArray1(),
                           _poles2d!.ChangeArray1(),
                           _dPoles2d!.ChangeArray1(),
                           _weigths!.ChangeArray1(),
                           _dWeigths!.ChangeArray1());

            //  Take into account the multiplication of poles3d by weights.
            //  and the translation.
            for (ii = 1; ii <= _num1DSS; ii++)
            {
                // Translation on the section
                _poles[ii] = new Point3D(_poles[ii].XYZ().Subtracted(_translation.XYZ()));
                // Homothety on all.
                double weight = _weigths.Value(ii);
                _dPoles[ii] = _dPoles[ii].Multiplied(weight);
                vaux = vaux.WithXYZ(_poles.Value(ii).Coord());
                _dPoles[ii] = _dPoles[ii].Added(_dWeigths.Value(ii) * vaux);
                _poles[ii] = new Point3D(_poles[ii].XYZ().Multiplied(weight)); // for the cash
            }

            //  Apply transformation 2d to suitable vectors
            for (ii = 1; ii <= _num2DSS; ii++)
            {
                vcoord = _dPoles2d.Value(ii).XY();
                _aAffin!.Value(ii).Transforms(ref vcoord);
                _dPoles2d[ii] = _dPoles2d[ii].WithXY(vcoord);
                Xy coord = _poles2d[ii].XY();
                _aAffin.Value(ii).Transforms(ref coord);
                _poles2d[ii] = new Point2D(coord);
            }

            // Update control variables and return
            _first = first;
            _last = last;
            _order = 1;
            _param = param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= _num1DSS; ii++)
        {
            localResult[index] = _dWeigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= _num2DSS; ii++)
        {
            localResult[index] = _dPoles2d!.Value(ii).X;
            localResult[index + 1] = _dPoles2d.Value(ii).Y;
            index += 2;
        }
        for (ii = 1; ii <= _num3DSS; ii++, index += 3)
        {
            localResult[index] = _dPoles!.Value(ii).X;
            localResult[index + 1] = _dPoles.Value(ii).Y;
            localResult[index + 2] = _dPoles.Value(ii).Z;
        }
        return ok;
    }

    private bool D2(double param, double first, double last, Span<double> localResult)
    {
        Xy vcoord;
        Vector3D vaux = new();
        int index, ii;
        bool ok = true;

        // management of limits
        if ((_first != first) || (last != _last))
        {
            _func.SetInterval(first, last);
        }

        if ((param != _param) || (_order < 2) || (_first != first) || (last != _last))
        {
            // Positioning in case when the last operation is not repeated
            ok = _func.D2(param,
                           first,
                           last,
                           _poles!.ChangeArray1(),
                           _dPoles!.ChangeArray1(),
                           _d2Poles!.ChangeArray1(),
                           _poles2d!.ChangeArray1(),
                           _dPoles2d!.ChangeArray1(),
                           _d2Poles2d!.ChangeArray1(),
                           _weigths!.ChangeArray1(),
                           _dWeigths!.ChangeArray1(),
                           _d2Weigths!.ChangeArray1());

            //  Multiply poles3d by the weight after translations.
            for (ii = 1; ii <= _num1DSS; ii++)
            {
                // First translate
                _poles[ii] = new Point3D(_poles[ii].XYZ().Subtracted(_translation.XYZ()));

                // Calculate the second derivative
                _d2Poles[ii] = _d2Poles[ii].Multiplied(_weigths.Value(ii));
                vaux = vaux.WithXYZ(_dPoles.Value(ii).XYZ());
                _d2Poles[ii] = _d2Poles[ii].Added((2 * _dWeigths.Value(ii)) * vaux);
                vaux = vaux.WithXYZ(_poles.Value(ii).Coord());
                _d2Poles[ii] = _d2Poles[ii].Added(_d2Weigths.Value(ii) * vaux);

                // Then the remainder for the cash
                _dPoles[ii] = _dPoles[ii].Multiplied(_weigths.Value(ii));
                vaux = vaux.WithXYZ(_poles.Value(ii).Coord());
                _dPoles[ii] = _dPoles[ii].Added(_dWeigths.Value(ii) * vaux);
                _poles[ii] = new Point3D(_poles[ii].XYZ().Multiplied(_weigths.Value(ii)));
            }

            //  Apply transformation to poles 2d.
            for (ii = 1; ii <= _num2DSS; ii++)
            {
                vcoord = _d2Poles2d.Value(ii).XY();
                _aAffin!.Value(ii).Transforms(ref vcoord);
                _d2Poles2d[ii] = _d2Poles2d[ii].WithXY(vcoord);
                vcoord = _dPoles2d.Value(ii).XY();
                _aAffin.Value(ii).Transforms(ref vcoord);
                _dPoles2d[ii] = _dPoles2d[ii].WithXY(vcoord);
                Xy coord = _poles2d[ii].XY();
                _aAffin.Value(ii).Transforms(ref coord);
                _poles2d[ii] = new Point2D(coord);
            }

            // Update variables of control and return
            _first = first;
            _last = last;
            _order = 2;
            _param = param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= _num1DSS; ii++)
        {
            localResult[index] = _d2Weigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= _num2DSS; ii++)
        {
            localResult[index] = _d2Poles2d!.Value(ii).X;
            localResult[index + 1] = _d2Poles2d.Value(ii).Y;
            index += 2;
        }
        for (ii = 1; ii <= _num3DSS; ii++, index += 3)
        {
            localResult[index] = _d2Poles!.Value(ii).X;
            localResult[index + 1] = _d2Poles.Value(ii).Y;
            localResult[index + 2] = _d2Poles.Value(ii).Z;
        }

        return ok;
    }

    /// <summary>Whether the approximation has a result.</summary>
    public bool IsDone => _done;

    /// <summary>The shape of the approximated surface: degrees and numbers of poles and knots in U and V.</summary>
    public void SurfShape(out int uDegree, out int vDegree, out int nbUPoles, out int nbVPoles, out int nbUKnots,
                          out int nbVKnots)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        uDegree = _udeg;
        vDegree = _vdeg;
        nbUPoles = _tabPoles!.ColLength();
        nbVPoles = _tabPoles.RowLength();
        nbUKnots = _tabUKnots!.Length();
        nbVKnots = _tabVKnots!.Length();
    }

    /// <summary>Copies the approximated surface into the caller's arrays (sized by <see cref="SurfShape"/>).</summary>
    public void Surface(NCollection_Array2<Point3D> tPoles,
                        NCollection_Array2<double> tWeights,
                        NCollection_Array1<double> tuKnots,
                        NCollection_Array1<double> tvKnots,
                        NCollection_Array1<int> tuMults,
                        NCollection_Array1<int> tvMults)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        tPoles.Assign(_tabPoles!.Array2());
        tWeights.Assign(_tabWeights!.Array2());
        tuKnots.Assign(_tabUKnots!.Array1());
        tuMults.Assign(_tabUMults!.Array1());
        tvKnots.Assign(_tabVKnots!.Array1());
        tvMults.Assign(_tabVMults!.Array1());
    }

    public int UDegree()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _udeg;
    }

    public int VDegree()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _vdeg;
    }

    /// <summary>The poles of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array2<Point3D> SurfPoles()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabPoles!.Array2();
    }

    /// <summary>The weights of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array2<double> SurfWeights()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabWeights!.Array2();
    }

    /// <summary>The U knots of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> SurfUKnots()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabUKnots!.Array1();
    }

    /// <summary>The V knots of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> SurfVKnots()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabVKnots!.Array1();
    }

    /// <summary>The U multiplicities of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> SurfUMults()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabUMults!.Array1();
    }

    /// <summary>The V multiplicities of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> SurfVMults()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return _tabVMults!.Array1();
    }

    /// <summary>The maximum error of the surface approximation.</summary>
    public double MaxErrorOnSurf()
    {
        int ii;
        double maxError = 0, err;
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }

        if (_func.IsRational)
        {
            NCollection_Array1<double> wmin = new(1, _num1DSS);
            _func.GetMinimalWeight(wmin);
            double size = _func.MaximalSection();
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                err = (size * _mError1d!.Value(ii) + _mError3d!.Value(ii)) / wmin[ii];
                if (err > maxError)
                {
                    maxError = err;
                }
            }
        }
        else
        {
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                err = _mError3d!.Value(ii);
                if (err > maxError)
                {
                    maxError = err;
                }
            }
        }
        return maxError;
    }

    /// <summary>The average error of the surface approximation.</summary>
    public double AverageErrorOnSurf()
    {
        int ii;
        double moyError = 0, err;
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }

        if (_func.IsRational)
        {
            NCollection_Array1<double> wmin = new(1, _num1DSS);
            _func.GetMinimalWeight(wmin);
            double size = _func.MaximalSection();
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                err = (size * _aError1d!.Value(ii) + _aError3d!.Value(ii)) / wmin[ii];
                moyError += err;
            }
        }
        else
        {
            for (ii = 1; ii <= _num3DSS; ii++)
            {
                err = _aError3d!.Value(ii);
                moyError += err;
            }
        }
        return moyError / _num3DSS;
    }

    public int Curves2dCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException(" Approx_SweepApproximation");
            }
            return _num2DSS;
        }
    }

    /// <summary>The shape of the 2D curves: degree and numbers of poles and knots.</summary>
    public void Curves2dShape(out int degree, out int nbPoles, out int nbKnots)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException("Approx_SweepApproximation");
        }
        degree = _deg2d;
        nbPoles = _seqPoles2d.Value(1).Length();
        nbKnots = _tab2dKnots!.Length();
    }

    /// <summary>Copies the 2D curve Index into the caller's arrays (sized by <see cref="Curves2dShape"/>).</summary>
    public void Curve2d(int index,
                        NCollection_Array1<Point2D> tPoles,
                        NCollection_Array1<double> tKnots,
                        NCollection_Array1<int> tMults)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException("Approx_SweepApproximation");
        }
        tPoles.Assign(_seqPoles2d.Value(index).Array1());
        tKnots.Assign(_tab2dKnots!.Array1());
        tMults.Assign(_tab2dMults!.Array1());
    }

    public int Curves2dDegree()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _deg2d;
    }

    /// <summary>The poles of the 2D curve Index (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<Point2D> Curve2dPoles(int index)
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _seqPoles2d.Value(index).Array1();
    }

    /// <summary>The knots of the 2D curves (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> Curves2dKnots()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _tab2dKnots!.Array1();
    }

    /// <summary>The multiplicities of the 2D curves (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> Curves2dMults()
    {
        if (!_done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _tab2dMults!.Array1();
    }

    /// <summary>The maximum error of the approximation of the 2D curve Index.</summary>
    public double Max2dError(int index)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return _tab2dError!.Value(index);
    }

    /// <summary>The average error of the approximation of the 2D curve Index.</summary>
    public double Average2dError(int index)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return _ave2dError!.Value(index);
    }

    /// <summary>The maximum 3D error, on the surface, of the approximation of the 2D curve Index.</summary>
    public double TolCurveOnSurf(int index)
    {
        if (!_done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return _cOnSurfErr!.Value(index);
    }

    /// <summary>Writes information on the approximation.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("Dump of SweepApproximation" + '\n');
        if (_done)
        {
            o.Write("Error 3d = " + MaxErrorOnSurf().ToString(CultureInfo.InvariantCulture) + '\n');

            if (_num2DSS > 0)
            {
                o.Write("Error 2d = ");
                for (int ii = 1; ii <= _num2DSS; ii++)
                {
                    o.Write(Max2dError(ii).ToString(CultureInfo.InvariantCulture));
                    if (ii < _num2DSS)
                    {
                        o.Write(" , " + '\n');
                    }
                }
                // OCCT writes this line end and " Not Done " to std::cout instead of o; the port writes them to o.
                o.Write('\n');
            }
            o.Write((_tabVKnots!.Length() - 1).ToString(CultureInfo.InvariantCulture) + " Segment(s) of degree "
                    + _vdeg.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        else
        {
            o.Write(" Not Done " + '\n');
        }
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
