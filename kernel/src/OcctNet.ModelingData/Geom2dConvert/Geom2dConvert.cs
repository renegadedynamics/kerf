// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Conversions of 2D curves to B-spline curves: of any bounded curve (conics exactly, offset curves by
/// approximation), splitting of B-splines between knots or parameters, concatenation of B-splines into G1 or C1
/// curves, and splitting of C0 B-splines into C1 ones.
/// </summary>
/// <remarks>
/// Every curve these functions return belongs to the caller and is not frozen; it may be one of the caller's own
/// curves where OCCT returns it as is (a group of one curve in ConcatG1/ConcatC1, a C1 curve in
/// C0BSplineToArrayOfC1BSplineCurve). The caller's curves are never modified: where OCCT changes a curve of the
/// input array (the weight normalisation of ConcatG1 and ConcatC1), the port puts a modified copy in the array.
/// </remarks>
public static class Geom2dConvert
{
    private static Geom2d_BSplineCurve BSplineCurveBuilder(Geom2d_Conic TheConic, Convert_ConicToBSplineCurve Convert)
    {
        Geom2d_BSplineCurve TheCurve;
        TheCurve = new Geom2d_BSplineCurve(Convert.Poles(),
                                           Convert.Weights(),
                                           Convert.Knots(),
                                           Convert.Multiplicities(),
                                           Convert.Degree(),
                                           Convert.IsPeriodic());

        gp_Ax22d Axis = TheConic.Position();
        if ((Axis.XDirection() ^ Axis.YDirection()) < 0.0)
        {
            // Then the axis is left-handed, apply a symmetry to the curve.
            gp_Trsf2d Sym = new();
            Sym.SetMirror(gp.OX2d());
            TheCurve.Transform(Sym);
        }

        gp_Trsf2d T = new();
        T.SetTransformation(TheConic.XAxis(), gp.OX2d());
        Geom2d_BSplineCurve Cres = (Geom2d_BSplineCurve)TheCurve.Transformed(T);
        return Cres;
    }

    /// <summary>
    /// The part of C between its knots FromK1 and ToK2. For a non-periodic curve, it is reversed if FromK1 &gt; ToK2;
    /// for a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="Standard_DomainError">FromK1 = ToK2.</exception>
    /// <exception cref="Standard_OutOfRange">A knot index is outside [FirstUKnotIndex, LastUKnotIndex].</exception>
    public static Geom2d_BSplineCurve SplitBSplineCurve(Geom2d_BSplineCurve C, int FromK1, int ToK2,
                                                        bool SameOrientation = true)
    {
        int TheFirst = C.FirstUKnotIndex();
        int TheLast = C.LastUKnotIndex();
        if (FromK1 == ToK2)
        {
            throw new Standard_DomainError();
        }

        int FirstK = Min(FromK1, ToK2);
        int LastK = Max(FromK1, ToK2);
        if (FirstK < TheFirst || LastK > TheLast)
        {
            throw new Standard_OutOfRange();
        }

        Geom2d_BSplineCurve NewCurve = (Geom2d_BSplineCurve)C.Copy();

        NewCurve.Segment(C.Knot(FirstK), C.Knot(LastK));

        if (C.IsPeriodic())
        {
            if (!SameOrientation)
            {
                NewCurve.Reverse();
            }
        }
        else
        {
            if (FromK1 > ToK2)
            {
                NewCurve.Reverse();
            }
        }

        return NewCurve;
    }

    /// <summary>
    /// The part of C between the parameters FromU1 and ToU2. For a non-periodic curve, it is reversed if
    /// FromU1 &gt; ToU2; for a periodic one, if SameOrientation is false. ParametricTolerance is not used.
    /// </summary>
    public static Geom2d_BSplineCurve SplitBSplineCurve(Geom2d_BSplineCurve C, double FromU1, double ToU2,
                                                        double ParametricTolerance, bool SameOrientation = true)
    {
        double FirstU = Min(FromU1, ToU2);
        double LastU = Max(FromU1, ToU2);

        Geom2d_BSplineCurve C1 = (Geom2d_BSplineCurve)C.Copy();

        C1.Segment(FirstU, LastU);

        if (C.IsPeriodic())
        {
            if (!SameOrientation)
            {
                C1.Reverse();
            }
        }
        else
        {
            if (FromU1 > ToU2)
            {
                C1.Reverse();
            }
        }

        return C1;
    }

    /// <summary>
    /// C as a B-spline curve: lines (trimmed), conics, Bezier and B-spline curves exactly (Parameterisation chooses
    /// how circles and ellipses are parametrised), offset curves by approximation (tolerance 1e-4, C2, at most 16
    /// segments of degree 14). A whole circle or ellipse gives a periodic B-spline.
    /// </summary>
    /// <exception cref="Standard_DomainError">C is of another kind (an untrimmed line, hyperbola or parabola).</exception>
    /// <exception cref="Standard_ConstructionError">The approximation of an offset curve fails.</exception>
    public static Geom2d_BSplineCurve CurveToBSplineCurve(Geom2d_Curve C,
                                                          Convert_ParameterisationType Parameterisation = Convert_TgtThetaOver2)
    {
        Geom2d_BSplineCurve TheCurve;

        if (C is Geom2d_TrimmedCurve Ctrim)
        {
            Geom2d_Curve Curv;
            Curv = Ctrim.BasisCurve();
            double U1 = Ctrim.FirstParameter();
            double U2 = Ctrim.LastParameter();

            // If the curve is not truly restricted, there is no risk of
            // a Raise in BS->Segment.
            if (!Curv.IsPeriodic())
            {
                if (U1 < Curv.FirstParameter())
                {
                    U1 = Curv.FirstParameter();
                }

                if (U2 > Curv.LastParameter())
                {
                    U2 = Curv.LastParameter();
                }
            }

            if (Curv is Geom2d_Line)
            {
                gp_Pnt2d Pdeb = Ctrim.StartPoint();
                gp_Pnt2d Pfin = Ctrim.EndPoint();
                NCollection_Array1<gp_Pnt2d> Poles = new(1, 2);
                Poles[1] = Pdeb;
                Poles[2] = Pfin;
                NCollection_Array1<double> Knots = new(1, 2);
                Knots[1] = Ctrim.FirstParameter();
                Knots[2] = Ctrim.LastParameter();
                NCollection_Array1<int> Mults = new(1, 2);
                Mults[1] = 2;
                Mults[2] = 2;
                int Degree = 1;
                TheCurve = new Geom2d_BSplineCurve(Poles, Knots, Mults, Degree);
            }

            else if (Curv is Geom2d_Circle TheCircle)
            {
                gp_Circ2d C2d = new(gp.OX2d(), TheCircle.Radius());
                if (Parameterisation != Convert_RationalC1)
                {
                    Convert_CircleToBSplineCurve Convert = new(C2d, U1, U2, Parameterisation);
                    TheCurve = BSplineCurveBuilder(TheCircle, Convert);
                }
                else
                {
                    if (U2 - U1 < 6.0)
                    {
                        Convert_CircleToBSplineCurve Convert = new(C2d, U1, U2, Parameterisation);
                        TheCurve = BSplineCurveBuilder(TheCircle, Convert);
                    }
                    else
                    { // split circle to avoide numerical
                      // overflow when U2 - U1 =~ 2*PI

                        double Umed = (U1 + U2) * .5;
                        Convert_CircleToBSplineCurve Convert1 = new(C2d, U1, Umed, Parameterisation);

                        Geom2d_BSplineCurve TheCurve1 = BSplineCurveBuilder(TheCircle, Convert1);

                        Convert_CircleToBSplineCurve Convert2 = new(C2d, Umed, U2, Parameterisation);

                        Geom2d_BSplineCurve TheCurve2 = BSplineCurveBuilder(TheCircle, Convert2);

                        Geom2dConvert_CompCurveToBSplineCurve CCTBSpl = new(TheCurve1, Parameterisation);

                        CCTBSpl.Add(TheCurve2, Precision.PConfusion(), true);

                        TheCurve = CCTBSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (Curv is Geom2d_Ellipse TheEllipse)
            {
                gp_Elips2d E2d = new(gp.OX2d(), TheEllipse.MajorRadius(), TheEllipse.MinorRadius());
                if (Parameterisation != Convert_RationalC1)
                {
                    Convert_EllipseToBSplineCurve Convert = new(E2d, U1, U2, Parameterisation);
                    TheCurve = BSplineCurveBuilder(TheEllipse, Convert);
                }
                else
                {
                    if (U2 - U1 < 6.0)
                    {
                        Convert_EllipseToBSplineCurve Convert = new(E2d, U1, U2, Parameterisation);
                        TheCurve = BSplineCurveBuilder(TheEllipse, Convert);
                    }
                    else
                    { // split ellipse to avoide numerical
                      // overflow when U2 - U1 =~ 2*PI

                        double Umed = (U1 + U2) * .5;
                        Convert_EllipseToBSplineCurve Convert1 = new(E2d, U1, Umed, Parameterisation);

                        Geom2d_BSplineCurve TheCurve1 = BSplineCurveBuilder(TheEllipse, Convert1);

                        Convert_EllipseToBSplineCurve Convert2 = new(E2d, Umed, U2, Parameterisation);

                        Geom2d_BSplineCurve TheCurve2 = BSplineCurveBuilder(TheEllipse, Convert2);

                        Geom2dConvert_CompCurveToBSplineCurve CCTBSpl = new(TheCurve1, Parameterisation);

                        CCTBSpl.Add(TheCurve2, Precision.PConfusion(), true);

                        TheCurve = CCTBSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (Curv is Geom2d_Hyperbola TheHyperbola)
            {
                gp_Hypr2d H2d = new(gp.OX2d(), TheHyperbola.MajorRadius(), TheHyperbola.MinorRadius());
                Convert_HyperbolaToBSplineCurve Convert = new(H2d, U1, U2);
                TheCurve = BSplineCurveBuilder(TheHyperbola, Convert);
            }

            else if (Curv is Geom2d_Parabola TheParabola)
            {
                gp_Parab2d Prb2d = new(gp.OX2d(), TheParabola.Focal());
                Convert_ParabolaToBSplineCurve Convert = new(Prb2d, U1, U2);
                TheCurve = BSplineCurveBuilder(TheParabola, Convert);
            }

            else if (Curv is Geom2d_BezierCurve)
            {
                Geom2d_BezierCurve CBez = (Geom2d_BezierCurve)Curv.Copy();

                CBez.Segment(U1, U2);
                int Degree = CBez.Degree();
                NCollection_Array1<double> Knots = new(1, 2);
                NCollection_Array1<int> Mults = new(1, 2);
                Knots[1] = 0.0;
                Knots[2] = 1.0;
                Mults[1] = Degree + 1;
                Mults[2] = Degree + 1;
                NCollection_Array1<gp_Pnt2d> Poles = CBez.InternalPoles();
                if (CBez.IsRational())
                {
                    TheCurve = new Geom2d_BSplineCurve(Poles, CBez.InternalWeightsArray(), Knots, Mults, Degree);
                }
                else
                {
                    TheCurve = new Geom2d_BSplineCurve(Poles, Knots, Mults, Degree);
                }
            }

            else if (Curv is Geom2d_BSplineCurve)
            {
                TheCurve = (Geom2d_BSplineCurve)Curv.Copy();
                TheCurve.Segment(U1, U2);
            }

            else if (Curv is Geom2d_OffsetCurve)
            {
                double Tol2d = 1.0e-4;
                GeomAbs_Shape Order = GeomAbs_C2;
                int MaxSegments = 16, MaxDegree = 14;
                Geom2dConvert_ApproxCurve ApprCOffs = new(C, Tol2d, Order, MaxSegments, MaxDegree);
                if (ApprCOffs.HasResult())
                {
                    TheCurve = ApprCOffs.InternalCurve()!;
                }
                else
                {
                    throw new Standard_ConstructionError();
                }
            }

            else
            {
                throw new Standard_DomainError("No such curve");
            }
        }

        else
        {
            if (C is Geom2d_Ellipse TheEllipse)
            {
                gp_Elips2d E2d = new(gp.OX2d(), TheEllipse.MajorRadius(), TheEllipse.MinorRadius());
                Convert_EllipseToBSplineCurve Convert = new(E2d, Parameterisation);
                TheCurve = BSplineCurveBuilder(TheEllipse, Convert);
                TheCurve.SetPeriodic();
            }

            else if (C is Geom2d_Circle TheCircle)
            {
                gp_Circ2d C2d = new(gp.OX2d(), TheCircle.Radius());
                Convert_CircleToBSplineCurve Convert = new(C2d, Parameterisation);
                TheCurve = BSplineCurveBuilder(TheCircle, Convert);
                TheCurve.SetPeriodic();
            }

            else if (C is Geom2d_BezierCurve CBez)
            {
                int Degree = CBez.Degree();
                NCollection_Array1<double> Knots = new(1, 2);
                NCollection_Array1<int> Mults = new(1, 2);
                Knots[1] = 0.0;
                Knots[2] = 1.0;
                Mults[1] = Degree + 1;
                Mults[2] = Degree + 1;
                NCollection_Array1<gp_Pnt2d> Poles = CBez.InternalPoles();
                if (CBez.IsRational())
                {
                    TheCurve = new Geom2d_BSplineCurve(Poles, CBez.InternalWeightsArray(), Knots, Mults, Degree);
                }
                else
                {
                    TheCurve = new Geom2d_BSplineCurve(Poles, Knots, Mults, Degree);
                }
            }
            else if (C is Geom2d_BSplineCurve)
            {
                TheCurve = (Geom2d_BSplineCurve)C.Copy();
            }

            else if (C is Geom2d_OffsetCurve)
            {
                double Tol2d = 1.0e-4;
                GeomAbs_Shape Order = GeomAbs_C2;
                int MaxSegments = 16, MaxDegree = 14;
                Geom2dConvert_ApproxCurve ApprCOffs = new(C, Tol2d, Order, MaxSegments, MaxDegree);
                if (ApprCOffs.HasResult())
                {
                    TheCurve = ApprCOffs.InternalCurve()!;
                }
                else
                {
                    throw new Standard_ConstructionError();
                }
            }

            else
            {
                throw new Standard_DomainError();
            }
        }

        return TheCurve;
    }

    private sealed class Geom2dConvert_law_evaluator : BSplCLib_EvaluatorFunction
    {
        private readonly Geom2d_BSplineCurve? myAncore;

        public Geom2dConvert_law_evaluator(Geom2d_BSplineCurve? theAncore) => myAncore = theAncore;

        public override void Evaluate(int theDerivativeRequest, ReadOnlySpan<double> theStartEnd, double theParameter,
                                      out double theResult, out int theErrorCode)
        {
            theErrorCode = 0;
            if (myAncore is not null && theParameter >= theStartEnd[0] && theParameter <= theStartEnd[1]
                && theDerivativeRequest == 0)
            {
                myAncore.D0(theParameter, out gp_Pnt2d aPoint);
                theResult = aPoint.Coord(2);
            }
            else
            {
                // OCCT leaves the result unset; the port sets 0 (decision 32)
                theResult = 0.0;
                theErrorCode = 1;
            }
        }
    }

    private static Geom2d_BSplineCurve MultNumandDenom(Geom2d_BSplineCurve a, Geom2d_BSplineCurve BS)
    {
        Geom2d_BSplineCurve res;
        double start_value, end_value;
        int resNbPoles, degree, ii, jj, aStatus;

        NCollection_Array1<double> BSKnots = BS.InternalKnots();
        NCollection_Array1<int> BSMults = BS.InternalMultiplicities();
        NCollection_Array1<gp_Pnt2d> BSPoles = new(BS.InternalPoles());
        NCollection_Array1<double> BSWeights = BS.InternalWeightsArray();
        NCollection_Array1<double> BSFlatKnots = BS.InternalKnotSequence();
        start_value = BSKnots[1];
        end_value = BSKnots[BS.NbKnots()];
        double tolerance = 10.0 * Epsilon(Math.Abs(end_value));

        NCollection_Array1<double> aKnots = new(a.InternalKnots());
        NCollection_Array1<gp_Pnt2d> aPoles = a.InternalPoles();
        NCollection_Array1<int> aMults = a.InternalMultiplicities();
        BSplCLib.Reparametrize(BS.FirstParameter(), BS.LastParameter(), aKnots);
        Geom2d_BSplineCurve anAncore = new(aPoles, aKnots, aMults, a.Degree());

        BSplCLib.MergeBSplineKnots(tolerance,
                                   start_value,
                                   end_value,
                                   a.Degree(),
                                   aKnots,
                                   aMults,
                                   BS.Degree(),
                                   BSKnots,
                                   BSMults,
                                   out resNbPoles,
                                   out NCollection_HArray1<double> resKnots,
                                   out NCollection_HArray1<int> resMults);
        degree = BS.Degree() + a.Degree();
        NCollection_Array1<gp_Pnt2d> resNumPoles = new(1, resNbPoles);
        NCollection_Array1<double> resDenPoles = new(1, resNbPoles);
        NCollection_Array1<gp_Pnt2d> resPoles = new(1, resNbPoles);
        NCollection_Array1<double> resFlatKnots = new(1, resNbPoles + degree + 1);
        BSplCLib.KnotSequence(resKnots, resMults, resFlatKnots);
        for (ii = 1; ii <= BS.NbPoles(); ii++)
        {
            for (jj = 1; jj <= 2; jj++)
            {
                BSPoles[ii].SetCoord(jj, BSPoles[ii].Coord(jj) * BSWeights[ii]);
            }
        }

        // POP for NT
        Geom2dConvert_law_evaluator ev = new(anAncore);
        BSplCLib.FunctionMultiply(ev, BS.Degree(), BSFlatKnots, BSPoles, resFlatKnots, degree, resNumPoles, out aStatus);
        BSplCLib.FunctionMultiply(ev, BS.Degree(), BSFlatKnots, BSWeights, resFlatKnots, degree, resDenPoles, out aStatus);
        for (ii = 1; ii <= resNbPoles; ii++)
        {
            for (jj = 1; jj <= 2; jj++)
            {
                resPoles[ii].SetCoord(jj, resNumPoles[ii].Coord(jj) / resDenPoles[ii]);
            }
        }

        res = new Geom2d_BSplineCurve(resPoles, resDenPoles, resKnots, resMults, degree);
        return res;
    }

    private static void Pretreatment(NCollection_Array1<Geom2d_BSplineCurve> tab)
    {
        int i, j;
        double a;

        for (i = 0; i <= (tab.Length() - 1); i++)
        {
            if (tab[i].IsRational())
            {
                a = tab[i].Weight(1);
                if ((tab[i].Weight(2) == a) && (tab[i].Weight(tab[i].NbPoles() - 1) == a)
                    && (tab[i].Weight(tab[i].NbPoles()) == a))
                {
                    // OCCT changes the caller's curve in place; the port puts a changed copy in the array
                    Geom2d_BSplineCurve aCurve = (Geom2d_BSplineCurve)tab[i].Copy();
                    for (j = 1; j <= aCurve.NbPoles(); j++)
                    {
                        aCurve.SetWeight(j, aCurve.Weight(j) / a);
                    }

                    tab[i] = aCurve;
                }
            }
        }
    }

    private static bool NeedToBeTreated(Geom2d_BSplineCurve BS)
    {
        if (BS.IsRational())
        {
            NCollection_Array1<double> tabWeights = BS.InternalWeightsArray();
            return (BSplCLib.IsRational(tabWeights, 1, BS.NbPoles()))
                   && ((BS.Weight(1) < (1 - Precision.Confusion()))
                       || (BS.Weight(1) > (1 + Precision.Confusion()))
                       || (BS.Weight(2) < (1 - Precision.Confusion()))
                       || (BS.Weight(2) > (1 + Precision.Confusion()))
                       || (BS.Weight(BS.NbPoles() - 1) < (1 - Precision.Confusion()))
                       || (BS.Weight(BS.NbPoles() - 1) > (1 + Precision.Confusion()))
                       || (BS.Weight(BS.NbPoles()) < (1 - Precision.Confusion()))
                       || (BS.Weight(BS.NbPoles()) > (1 + Precision.Confusion())));
        }
        else
        {
            return false;
        }
    }

    private static bool Need2DegRepara(NCollection_Array1<Geom2d_BSplineCurve> tab)
    {
        int i;
        gp_Vec2d Vec1, Vec2;
        double Rapport = 1.0e0;

        for (i = 0; i <= tab.Length() - 2; i++)
        {
            tab[i + 1].D1(tab[i + 1].FirstParameter(), out _, out Vec1);
            tab[i].D1(tab[i].LastParameter(), out _, out Vec2);
            Rapport = Rapport * Vec2.Magnitude() / Vec1.Magnitude();
        }

        return (Rapport > (1.0e0 + Precision.Confusion())) || (Rapport < (1.0e0 - Precision.Confusion()));
    }

    private static int Indexmin(NCollection_Array1<Geom2d_BSplineCurve> tab)
    {
        int i, index = 0, degree;

        degree = tab[0].Degree();
        for (i = 0; i <= tab.Length() - 1; i++)
        {
            if (tab[i].Degree() <= degree)
            {
                degree = tab[i].Degree();
                index = i;
            }
        }

        return index;
    }

    private static void ReorderArrayOfG1(NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves,
                                         NCollection_Array1<double> ArrayOfToler,
                                         NCollection_Array1<bool> tabG1,
                                         int StartIndex,
                                         double ClosedTolerance)
    {
        int i;
        NCollection_Array1<Geom2d_BSplineCurve> ArraybisOfCurves = new(0, ArrayOfCurves.Length() - 1);
        NCollection_Array1<double> ArraybisOfToler = new(0, ArrayOfToler.Length() - 1);
        NCollection_Array1<bool> tabbisG1 = new(0, tabG1.Length() - 1);

        for (i = 0; i <= ArrayOfCurves.Length() - 1; i++)
        {
            if (i != ArrayOfCurves.Length() - 1)
            {
                ArraybisOfCurves[i] = ArrayOfCurves[i];
                ArraybisOfToler[i] = ArrayOfToler[i];
                tabbisG1[i] = tabG1[i];
            }
            else
            {
                ArraybisOfCurves[i] = ArrayOfCurves[i];
            }
        }

        for (i = 0; i <= (ArrayOfCurves.Length() - (StartIndex + 2)); i++)
        {
            ArrayOfCurves[i] = ArraybisOfCurves[i + StartIndex + 1];
            if (i != (ArrayOfCurves.Length() - (StartIndex + 2)))
            {
                ArrayOfToler[i] = ArraybisOfToler[i + StartIndex + 1];
                tabG1[i] = tabbisG1[i + StartIndex + 1];
            }
        }

        ArrayOfToler[ArrayOfCurves.Length() - (StartIndex + 2)] = ClosedTolerance;
        tabG1[ArrayOfCurves.Length() - (StartIndex + 2)] = true;

        for (i = (ArrayOfCurves.Length() - (StartIndex + 1)); i <= (ArrayOfCurves.Length() - 1); i++)
        {
            if (i != ArrayOfCurves.Length() - 1)
            {
                ArrayOfCurves[i] = ArraybisOfCurves[i - (ArrayOfCurves.Length() - (StartIndex + 1))];
                ArrayOfToler[i] = ArraybisOfToler[i - (ArrayOfCurves.Length() - (StartIndex + 1))];
                tabG1[i] = tabbisG1[i - (ArrayOfCurves.Length() - (StartIndex + 1))];
            }
            else
            {
                ArrayOfCurves[i] = ArraybisOfCurves[i - (ArrayOfCurves.Length() - (StartIndex + 1))];
            }
        }
    }

    private static int GeomAbsToInteger(GeomAbs_Shape gcont)
    {
        int cont = 0;
        switch (gcont)
        {
            case GeomAbs_C0:
                cont = 0;
                break;
            case GeomAbs_G1:
                cont = 1;
                break;
            case GeomAbs_C1:
                cont = 2;
                break;
            case GeomAbs_G2:
                cont = 3;
                break;
            case GeomAbs_C2:
                cont = 4;
                break;
            case GeomAbs_C3:
                cont = 5;
                break;
            case GeomAbs_CN:
                cont = 6;
                break;
        }

        return cont;
    }

    private static GeomAbs_Shape Continuity(Geom2d_Curve C1,
                                            Geom2d_Curve C2,
                                            double u1,
                                            double u2,
                                            bool r1,
                                            bool r2,
                                            double tl,
                                            double ta)
    {
        GeomAbs_Shape cont = GeomAbs_C0;
        int index1, index2;
        double tolerance, value;
        gp_Vec2d d1, d2;
        gp_Pnt2d point1, point2;
        int cont1, cont2;
        GeomAbs_Shape gcont1 = C1.Continuity(), gcont2 = C2.Continuity();
        cont1 = GeomAbsToInteger(gcont1);
        cont2 = GeomAbsToInteger(gcont2);

        Geom2d_Curve aCurve1 = C1;
        Geom2d_Curve aCurve2 = C2;
        if (C1 is Geom2d_TrimmedCurve aTrimmed1)
        {
            aCurve1 = aTrimmed1.BasisCurve();
        }

        if (C2 is Geom2d_TrimmedCurve aTrimmed2)
        {
            aCurve2 = aTrimmed2.BasisCurve();
        }

        if (aCurve1 is Geom2d_BSplineCurve BSplineCurve1)
        {
            BSplineCurve1.Resolution(tl, out tolerance);
            BSplineCurve1.LocateU(u1, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < BSplineCurve1.NbKnots() && index1 == index2)
            {
                cont1 = BSplineCurve1.Degree() - BSplineCurve1.Multiplicity(index1);
            }
            else
            {
                cont1 = 5;
            }
        }

        if (aCurve2 is Geom2d_BSplineCurve BSplineCurve2)
        {
            BSplineCurve2.Resolution(tl, out tolerance);
            BSplineCurve2.LocateU(u2, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < BSplineCurve2.NbKnots() && index1 == index2)
            {
                cont2 = BSplineCurve2.Degree() - BSplineCurve2.Multiplicity(index1);
            }
            else
            {
                cont2 = 5;
            }
        }

        aCurve1.D1(u1, out point1, out d1);
        aCurve2.D1(u2, out point2, out d2);
        if (point1.SquareDistance(point2) <= tl * tl)
        {
            if (cont1 != 0 && cont2 != 0)
            {
                if (d1.SquareMagnitude() >= tl * tl && d2.SquareMagnitude() >= tl * tl)
                {
                    if (r1)
                    {
                        d1.SetCoord(-d1.X(), -d1.Y());
                    }

                    if (r2)
                    {
                        d2.SetCoord(-d2.X(), -d2.Y());
                    }

                    value = d1.Dot(d2);
                    if ((d1.Magnitude() <= (d2.Magnitude() + tl)) && (d1.Magnitude() >= (d2.Magnitude() - tl))
                        && (value / (d1.Magnitude() * d2.Magnitude()) >= 1.0e0 - ta * ta))
                    {
                        cont = GeomAbs_C1;
                    }
                    else
                    {
                        d1.Normalize();
                        d2.Normalize();
                        value = Math.Abs(d1.Dot(d2));
                        if (value >= 1.0e0 - ta * ta)
                        {
                            cont = GeomAbs_G1;
                        }
                    }
                }
            }
        }
        else
        {
            throw new Standard_Failure("Courbes non jointives");
        }

        return cont;
    }

    private static GeomAbs_Shape Continuity(Geom2d_Curve C1, Geom2d_Curve C2, double u1, double u2, bool r1, bool r2)
    {
        return Continuity(C1, C2, u1, u2, r1, r2, Precision.Confusion(), Precision.Angular());
    }

    private sealed class Geom2dConvert_reparameterise_evaluator : BSplCLib_EvaluatorFunction
    {
        private readonly double[] myPolynomialCoefficient = new double[3];

        public Geom2dConvert_reparameterise_evaluator(ReadOnlySpan<double> thePolynomialCoefficient) =>
            thePolynomialCoefficient[..3].CopyTo(myPolynomialCoefficient);

        public override void Evaluate(int theDerivativeRequest, ReadOnlySpan<double> theStartEnd, double theParameter,
                                      out double theResult, out int theErrorCode)
        {
            theErrorCode = 0;
            // OCCT passes a single double as the result, which PLib::EvalPolynomial fills with the value and
            // theDerivativeRequest derivatives: it writes past it when a derivative is requested (BSplCLib's
            // FunctionReparameterise only asks for the value). The port evaluates into a buffer of that size and
            // returns its first value, as OCCT's double holds.
            Span<double> aResults = stackalloc double[theDerivativeRequest + 1];
            PLib.EvalPolynomial(theParameter, theDerivativeRequest, 2, 1, myPolynomialCoefficient, aResults);
            theResult = aResults[0];
        }
    }

    /// <summary>
    /// Concatenates the consecutive curves of ArrayOfCurves (indexed from 0) that join with G1 continuity into
    /// B-splines, in ArrayOfConcatenated (from 0). ArrayOfToler (from 0, one less than the curves) holds the
    /// tolerances of the junctions. If ClosedFlag is true and the curves form one closed G1 group, the result is one
    /// periodic curve, closed within ClosedTolerance; ClosedFlag is set to false if the closing reparametrisation
    /// would raise the degree beyond the maximum. ArrayOfCurves may be reordered, and its rational curves with equal
    /// end weights replaced by normalised copies.
    /// </summary>
    /// <exception cref="Standard_Failure">Two consecutive curves don't join.</exception>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void ConcatG1(NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<Geom2d_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedFlag,
                                double ClosedTolerance)
    {
        int nb_curve = ArrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            First, PreLast = 0;
        gp_Vec2d Vec1, Vec2; // consecutive tangent vectors
        Geom2d_BSplineCurve Curve1, Curve2;
        NCollection_Array1<bool> tabG1 = new(0, nb_curve - 2); // G1 continuity table at junctions
        NCollection_Array1<double> local_tolerance = new(0, ArrayOfToler.Length() - 1);

        for (i = 0; i < ArrayOfToler.Length(); i++)
        {
            local_tolerance[i] = ArrayOfToler[i];
        }

        for (i = 0; i < nb_curve; i++)
        {
            if (i >= 1)
            {
                First = ArrayOfCurves[i].FirstParameter();
                if (Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true) < GeomAbs_C0)
                {
                    throw new Standard_ConstructionError("Geom2dConvert curves not C0"); // renvoi d'une erreur
                }
                else
                {
                    if (Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true) >= GeomAbs_G1)
                    {
                        tabG1[i - 1] = true; // True=Continuite G1
                    }
                    else
                    {
                        tabG1[i - 1] = false;
                    }
                }
            }

            PreLast = ArrayOfCurves[i].LastParameter();
        }

        while (index <= nb_curve - 1)
        { // Determination of Wire characteristics
            nb_vertexG1 = 0;
            while (((index + nb_vertexG1) <= nb_curve - 2) && (tabG1[index + nb_vertexG1]))
            {
                nb_vertexG1++;
            }

            nb_group++;
            if (index == 0)
            {
                nb_vertex_group0 = nb_vertexG1;
            }

            index = index + 1 + nb_vertexG1;
        }

        if ((ClosedFlag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1(ArrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, ClosedTolerance);
        }

        ArrayOfConcatenated = new NCollection_HArray1<Geom2d_BSplineCurve>(0, nb_group - 1);

        bool fusion;
        index = 0;
        Pretreatment(ArrayOfCurves);

        Span<double> aPolynomialCoefficient = stackalloc double[3];

        bool NeedDoubleDegRepara = Need2DegRepara(ArrayOfCurves);
        if (nb_group == 1 && ClosedFlag && NeedDoubleDegRepara)
        {
            Curve1 = ArrayOfCurves[nb_curve - 1];
            if (Curve1.Degree() > Geom2d_BSplineCurve.MaxDegree() / 2)
            {
                ClosedFlag = false;
            }
        }

        if ((nb_group == 1) && (ClosedFlag))
        { // treatment of a particular case
            indexmin = Indexmin(ArrayOfCurves);
            if (indexmin != (ArrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1(ArrayOfCurves, local_tolerance, tabG1, indexmin, ClosedTolerance);
            }

            Curve2 = ArrayOfCurves[0];
            for (j = 1; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                Curve1 = ArrayOfCurves[j];
                if ((j == (nb_curve - 1)) && (NeedDoubleDegRepara))
                {
                    int aNewCurveDegree = 2 * Curve1.Degree();
                    Curve2.D1(Curve2.LastParameter(), out _, out Vec1);
                    Curve1.D1(Curve1.FirstParameter(), out _, out Vec2);
                    lambda = Vec2.Magnitude() / Vec1.Magnitude();
                    NCollection_Array1<double> KnotC1 = new(Curve1.InternalKnots());
                    Curve1.D1(Curve1.LastParameter(), out _, out Vec2);
                    ArrayOfCurves[0].D1(ArrayOfCurves[0].FirstParameter(), out _, out Vec1);
                    double lambda2 = Vec1.Magnitude() / Vec2.Magnitude();
                    double tmax, a, b, c, umin = Curve1.FirstParameter(), umax = Curve1.LastParameter();
                    tmax = 2 * lambda * (umax - umin) / (1 + lambda * lambda2);
                    a = (lambda * lambda2 - 1) / (2 * lambda * tmax);
                    aPolynomialCoefficient[2] = a;
                    b = (1 / lambda);
                    aPolynomialCoefficient[1] = b;
                    c = umin;
                    aPolynomialCoefficient[0] = c;
                    NCollection_Array1<double> Curve1FlatKnots = new(1, Curve1.NbPoles() + Curve1.Degree() + 1);
                    NCollection_Array1<int> KnotC1Mults = new(Curve1.InternalMultiplicities());
                    BSplCLib.KnotSequence(KnotC1, KnotC1Mults, Curve1FlatKnots);
                    KnotC1[1] = 0.0;
                    for (ii = 2; ii <= KnotC1.Length(); ii++)
                    {
                        KnotC1[ii] = (-b + Math.Sqrt(b * b - 4 * a * (c - KnotC1[ii]))) / (2 * a); // ifv 17.05.00 buc60667
                    }

                    NCollection_Array1<gp_Pnt2d> Curve1Poles = new(Curve1.InternalPoles());

                    for (ii = 1; ii <= Curve1.NbKnots(); ii++)
                    {
                        KnotC1Mults[ii] = (Curve1.Degree() + KnotC1Mults[ii]);
                    }

                    NCollection_Array1<double> FlatKnots = new(1, Curve1FlatKnots.Length() + (Curve1.Degree() * Curve1.NbKnots()));

                    BSplCLib.KnotSequence(KnotC1, KnotC1Mults, FlatKnots);
                    NCollection_Array1<gp_Pnt2d> NewPoles = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                    int aStatus;
                    NCollection_Array1<double> Curve1Weights = Curve1.InternalWeightsArray();
                    for (ii = 1; ii <= Curve1.NbPoles(); ii++)
                    {
                        for (jj = 1; jj <= 2; jj++)
                        {
                            Curve1Poles[ii].SetCoord(jj, Curve1Poles[ii].Coord(jj) * Curve1Weights[ii]);
                        }
                    }

                    // POP for NT
                    Geom2dConvert_reparameterise_evaluator ev = new(aPolynomialCoefficient);
                    BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Poles, FlatKnots,
                                                    aNewCurveDegree, NewPoles, out aStatus);
                    NCollection_Array1<double> NewWeights = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                    BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Weights, FlatKnots,
                                                    aNewCurveDegree, NewWeights, out aStatus);
                    for (ii = 1; ii <= NewPoles.Length(); ii++)
                    {
                        for (jj = 1; jj <= 2; jj++)
                        {
                            NewPoles[ii].SetCoord(jj, NewPoles[ii].Coord(jj) / NewWeights[ii]);
                        }
                    }

                    Curve1 = new Geom2d_BSplineCurve(NewPoles, NewWeights, KnotC1, KnotC1Mults, aNewCurveDegree);
                }

                Geom2dConvert_CompCurveToBSplineCurve C = new(Curve2);
                fusion = C.Add(Curve1, local_tolerance[j - 1]); // merge two adjacent curves
                if (!fusion)
                {
                    throw new Standard_ConstructionError("Geom2dConvert Concatenation Error");
                }

                Curve2 = C.InternalBSplineCurve()!;
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                Curve2 = (Geom2d_BSplineCurve)Curve2.Copy();
            }

            Curve2.SetPeriodic(); // single C1 curve
            Curve2.RemoveKnot(Curve2.LastUKnotIndex(), Curve2.Multiplicity(Curve2.LastUKnotIndex()) - 1,
                              Precision.Confusion());
            ArrayOfConcatenated.SetValue(0, Curve2);
        }

        else
        {
            for (i = 0; i <= nb_group - 1; i++)
            {                    // main loop on each group of
                nb_vertexG1 = 0; // internal G1 continuity

                while (((index + nb_vertexG1) <= nb_curve - 2) && (tabG1[index + nb_vertexG1]))
                {
                    nb_vertexG1++;
                }

                for (j = index; j <= index + nb_vertexG1; j++)
                { // boucle secondaire a l'interieur de chaque groupe
                    Curve1 = ArrayOfCurves[j];

                    if (index == j)
                    { // initialisation en debut de groupe
                        ArrayOfConcatenated.SetValue(i, Curve1);
                    }
                    else
                    {
                        Geom2dConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(i));
                        // OCCT reads the caller's tolerances, not the ones reordered for a closed wire (occt-issues.md)
                        fusion = C.Add(Curve1, ArrayOfToler[j - 1]); // fusion de deux courbes adjacentes

                        if (!fusion)
                        {
                            throw new Standard_ConstructionError("Geom2dConvert Concatenation Error");
                        }

                        ArrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// <see cref="ConcatC1(NCollection_Array1{Geom2d_BSplineCurve}, NCollection_Array1{double}, out NCollection_HArray1{int}, out NCollection_HArray1{Geom2d_BSplineCurve}, ref bool, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void ConcatC1(NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<int> ArrayOfIndices,
                                out NCollection_HArray1<Geom2d_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedFlag,
                                double ClosedTolerance)
    {
        ConcatC1(ArrayOfCurves, ArrayOfToler, out ArrayOfIndices, out ArrayOfConcatenated, ref ClosedFlag,
                 ClosedTolerance, Precision.Angular());
    }

    /// <summary>
    /// As <see cref="ConcatG1"/>, but the curves are made C1 where they join with G1 continuity (rational curves
    /// are reparametrised first), and AngularTolerance is the tolerance of the G1 test. ArrayOfIndices (from 0) holds
    /// the indices in ArrayOfCurves of the first curve of each concatenated group, and a last index.
    /// </summary>
    /// <exception cref="Standard_Failure">Two consecutive curves don't join.</exception>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void ConcatC1(NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<int> ArrayOfIndices,
                                out NCollection_HArray1<Geom2d_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedFlag,
                                double ClosedTolerance,
                                double AngularTolerance)
    {
        int nb_curve = ArrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            First, PreLast = 0;
        gp_Vec2d Vec1, Vec2; // consecutive tangent vectors
        Geom2d_BSplineCurve Curve1, Curve2 = null!;
        NCollection_Array1<bool> tabG1 = new(0, nb_curve - 2); // G1 continuity table at junctions
        NCollection_Array1<double> local_tolerance = new(0, ArrayOfToler.Length() - 1);

        for (i = 0; i < ArrayOfToler.Length(); i++)
        {
            local_tolerance[i] = ArrayOfToler[i];
        }

        for (i = 0; i < nb_curve; i++)
        {
            if (i >= 1)
            {
                First = ArrayOfCurves[i].FirstParameter();
                if (Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true, ArrayOfToler[i - 1],
                               AngularTolerance)
                    < GeomAbs_C0)
                {
                    throw new Standard_ConstructionError("Geom2dConvert curves not C0"); // renvoi d'une erreur
                }
                else
                {
                    if (Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true,
                                   ArrayOfToler[i - 1], AngularTolerance)
                        >= GeomAbs_G1)
                    {
                        tabG1[i - 1] = true; // True=Continuite G1
                    }
                    else
                    {
                        tabG1[i - 1] = false;
                    }
                }
            }

            PreLast = ArrayOfCurves[i].LastParameter();
        }

        while (index <= nb_curve - 1)
        { // Determination of Wire characteristics
            nb_vertexG1 = 0;
            while (((index + nb_vertexG1) <= nb_curve - 2) && (tabG1[index + nb_vertexG1]))
            {
                nb_vertexG1++;
            }

            nb_group++;
            if (index == 0)
            {
                nb_vertex_group0 = nb_vertexG1;
            }

            index = index + 1 + nb_vertexG1;
        }

        if ((ClosedFlag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1(ArrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, ClosedTolerance);
        }

        ArrayOfIndices = new NCollection_HArray1<int>(0, nb_group);
        ArrayOfConcatenated = new NCollection_HArray1<Geom2d_BSplineCurve>(0, nb_group - 1);

        bool fusion;
        int k = 0;
        index = 0;
        Pretreatment(ArrayOfCurves);
        Span<double> aPolynomialCoefficient = stackalloc double[3];

        bool NeedDoubleDegRepara = Need2DegRepara(ArrayOfCurves);
        if (nb_group == 1 && ClosedFlag && NeedDoubleDegRepara)
        {
            Curve1 = ArrayOfCurves[nb_curve - 1];
            if (Curve1.Degree() > Geom2d_BSplineCurve.MaxDegree() / 2)
            {
                ClosedFlag = false;
            }
        }

        if ((nb_group == 1) && (ClosedFlag))
        { // treatment of a particular case
            ArrayOfIndices.SetValue(0, 0);
            ArrayOfIndices.SetValue(1, 0);
            indexmin = Indexmin(ArrayOfCurves);
            if (indexmin != (ArrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1(ArrayOfCurves, local_tolerance, tabG1, indexmin, ClosedTolerance);
            }

            for (j = 0; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                if (NeedToBeTreated(ArrayOfCurves[j]))
                {
                    Curve1 = MultNumandDenom(Hermit.Solution(ArrayOfCurves[j]), ArrayOfCurves[j]);
                }
                else
                {
                    Curve1 = ArrayOfCurves[j];
                }

                int aNewCurveDegree = 2 * Curve1.Degree();

                if (j == 0)
                { // initialisation en debut de groupe
                    Curve2 = Curve1;
                }
                else
                {
                    if ((j == (nb_curve - 1)) && (NeedDoubleDegRepara))
                    {
                        Curve2.D1(Curve2.LastParameter(), out _, out Vec1);
                        Curve1.D1(Curve1.FirstParameter(), out _, out Vec2);
                        lambda = Vec2.Magnitude() / Vec1.Magnitude();
                        NCollection_Array1<double> KnotC1 = new(Curve1.InternalKnots());
                        Curve1.D1(Curve1.LastParameter(), out _, out Vec2);
                        ArrayOfCurves[0].D1(ArrayOfCurves[0].FirstParameter(), out _, out Vec1);
                        double lambda2 = Vec1.Magnitude() / Vec2.Magnitude();
                        double tmax, a, b, c, umin = Curve1.FirstParameter(), umax = Curve1.LastParameter();
                        tmax = 2 * lambda * (umax - umin) / (1 + lambda * lambda2);
                        a = (lambda * lambda2 - 1) / (2 * lambda * tmax);
                        aPolynomialCoefficient[2] = a;
                        b = (1 / lambda);
                        aPolynomialCoefficient[1] = b;
                        c = umin;
                        aPolynomialCoefficient[0] = c;
                        NCollection_Array1<double> Curve1FlatKnots = new(1, Curve1.NbPoles() + Curve1.Degree() + 1);
                        NCollection_Array1<int> KnotC1Mults = new(Curve1.InternalMultiplicities());
                        BSplCLib.KnotSequence(KnotC1, KnotC1Mults, Curve1FlatKnots);
                        KnotC1[1] = 0.0;
                        for (ii = 2; ii <= KnotC1.Length(); ii++)
                        {
                            KnotC1[ii] = (-b + Math.Sqrt(b * b - 4 * a * (c - KnotC1[ii]))) / (2 * a); // ifv 17.05.00 buc60667
                        }

                        NCollection_Array1<gp_Pnt2d> Curve1Poles = new(Curve1.InternalPoles());

                        for (ii = 1; ii <= Curve1.NbKnots(); ii++)
                        {
                            KnotC1Mults[ii] = (Curve1.Degree() + KnotC1Mults[ii]);
                        }

                        NCollection_Array1<double> FlatKnots = new(1, Curve1FlatKnots.Length() + (Curve1.Degree() * Curve1.NbKnots()));

                        BSplCLib.KnotSequence(KnotC1, KnotC1Mults, FlatKnots);
                        NCollection_Array1<gp_Pnt2d> NewPoles = new(1, FlatKnots.Length() - (aNewCurveDegree + 1));
                        int aStatus;
                        NCollection_Array1<double> Curve1Weights = Curve1.InternalWeightsArray();
                        for (ii = 1; ii <= Curve1.NbPoles(); ii++)
                        {
                            for (jj = 1; jj <= 2; jj++)
                            {
                                Curve1Poles[ii].SetCoord(jj, Curve1Poles[ii].Coord(jj) * Curve1Weights[ii]);
                            }
                        }

                        // POP for NT
                        Geom2dConvert_reparameterise_evaluator ev = new(aPolynomialCoefficient);
                        BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Poles, FlatKnots,
                                                        aNewCurveDegree, NewPoles, out aStatus);
                        NCollection_Array1<double> NewWeights = new(1, FlatKnots.Length() - (aNewCurveDegree + 1));
                        BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Weights, FlatKnots,
                                                        aNewCurveDegree, NewWeights, out aStatus);
                        for (ii = 1; ii <= NewPoles.Length(); ii++)
                        {
                            for (jj = 1; jj <= 2; jj++)
                            {
                                NewPoles[ii].SetCoord(jj, NewPoles[ii].Coord(jj) / NewWeights[ii]);
                            }
                        }

                        Curve1 = new Geom2d_BSplineCurve(NewPoles, NewWeights, KnotC1, KnotC1Mults, aNewCurveDegree);
                    }

                    Geom2dConvert_CompCurveToBSplineCurve C = new(Curve2);
                    fusion = C.Add(Curve1, local_tolerance[j - 1]); // merge two adjacent curves
                    if (!fusion)
                    {
                        throw new Standard_ConstructionError("Geom2dConvert Concatenation Error");
                    }

                    Curve2 = C.InternalBSplineCurve()!;
                }
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                Curve2 = (Geom2d_BSplineCurve)Curve2.Copy();
            }

            Curve2.SetPeriodic(); // single C1 curve
            Curve2.RemoveKnot(Curve2.LastUKnotIndex(), Curve2.Multiplicity(Curve2.LastUKnotIndex()) - 1,
                              Precision.Confusion());
            ArrayOfConcatenated.SetValue(0, Curve2);
        }

        else
        {
            for (i = 0; i <= nb_group - 1; i++)
            {                    // boucle principale sur chaque groupe de
                nb_vertexG1 = 0; // continuite interne G1

                while (((index + nb_vertexG1) <= nb_curve - 2) && (tabG1[index + nb_vertexG1]))
                {
                    nb_vertexG1++;
                }

                if ((!ClosedFlag) || (nb_group == 1))
                { // Filling the array of preserved indices
                    k++;
                    ArrayOfIndices.SetValue(k - 1, index);
                    if (k == nb_group)
                    {
                        ArrayOfIndices.SetValue(k, 0);
                    }
                }
                else
                {
                    k++;
                    ArrayOfIndices.SetValue(k - 1, index + nb_vertex_group0 + 1);
                    if (k == nb_group)
                    {
                        ArrayOfIndices.SetValue(k, nb_vertex_group0 + 1);
                    }
                }

                for (j = index; j <= index + nb_vertexG1; j++)
                { // boucle secondaire a l'interieur de chaque groupe
                    if (NeedToBeTreated(ArrayOfCurves[j]))
                    {
                        Curve1 = MultNumandDenom(Hermit.Solution(ArrayOfCurves[j]), ArrayOfCurves[j]);
                    }
                    else
                    {
                        Curve1 = ArrayOfCurves[j];
                    }

                    if (index == j)
                    { // initialisation en debut de groupe
                        ArrayOfConcatenated.SetValue(i, Curve1);
                    }
                    else
                    {
                        Geom2dConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(i));
                        // OCCT reads the caller's tolerances, not the ones reordered for a closed wire (occt-issues.md)
                        fusion = C.Add(Curve1, ArrayOfToler[j - 1]); // fusion de deux courbes adjacentes
                        if (!fusion)
                        {
                            throw new Standard_ConstructionError("Geom2dConvert Concatenation Error");
                        }

                        ArrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// Makes BS C1 where it is only C0 but G1 within tolerance: BS is replaced by a new curve when it has C0 knots
    /// (the curve given is not changed); otherwise it is left as it is.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void C0BSplineToC1BSplineCurve(ref Geom2d_BSplineCurve BS, double tolerance)
    {
        NCollection_Array1<int> BSMults = BS.InternalMultiplicities();
        NCollection_Array1<double> BSKnots = BS.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double U1, U2;
        bool closed_flag = false;
        gp_Pnt2d point1, point2;
        gp_Vec2d V1, V2;
        bool fusion;
        for (i = BS.FirstUKnotIndex() + 1; i <= (BS.LastUKnotIndex() - 1); i++)
        {
            if (BSMults[i] == BS.Degree())
            {
                nbcurveC1++;
            }
        }

        nbcurveC1 = Min(nbcurveC1, BS.NbKnots() - 1);

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> ArrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                ArrayOfToler[i] = tolerance;
            }

            U2 = BS.FirstParameter();
            j = BS.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                U1 = U2;

                while (j < BS.LastUKnotIndex() && BSMults[j] < BS.Degree())
                {
                    j++;
                }

                U2 = BSKnots[j];
                j++;
                Geom2d_BSplineCurve BSbis = (Geom2d_BSplineCurve)BS.Copy();
                BSbis.Segment(U1, U2);
                ArrayOfCurves[i] = BSbis;
            }

            const double anAngularToler = 1.0e-7;

            BS.D1(BS.FirstParameter(), out point1, out V1); // a verifier
            BS.D1(BS.LastParameter(), out point2, out V2);

            if ((point1.SquareDistance(point2) < tolerance * tolerance) && (V1.IsParallel(V2, anAngularToler)))
            {
                closed_flag = true;
            }

            Geom2dConvert.ConcatC1(ArrayOfCurves, ArrayOfToler, out _, out NCollection_HArray1<Geom2d_BSplineCurve> ArrayOfConcatenated,
                                   ref closed_flag, tolerance);

            Geom2dConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(0));
            if (ArrayOfConcatenated.Length() >= 2)
            {
                for (i = 1; i < ArrayOfConcatenated.Length(); i++)
                {
                    fusion = C.Add(ArrayOfConcatenated.Value(i), tolerance, true);
                    if (!fusion)
                    {
                        throw new Standard_ConstructionError("Geom2dConvert Concatenation Error");
                    }
                }
            }

            BS = C.InternalBSplineCurve()!;
        }
    }

    /// <summary>
    /// <see cref="C0BSplineToArrayOfC1BSplineCurve(Geom2d_BSplineCurve, out NCollection_HArray1{Geom2d_BSplineCurve}, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void C0BSplineToArrayOfC1BSplineCurve(Geom2d_BSplineCurve BS,
                                                        out NCollection_HArray1<Geom2d_BSplineCurve> tabBS,
                                                        double Tolerance)
    {
        C0BSplineToArrayOfC1BSplineCurve(BS, out tabBS, Tolerance, Precision.Angular());
    }

    /// <summary>
    /// Cuts BS at its C0 knots and concatenates the pieces that join with G1 continuity (within the tolerances) into
    /// C1 curves, returned in tabBS (from 0). A curve without C0 knots is returned as the single element.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void C0BSplineToArrayOfC1BSplineCurve(Geom2d_BSplineCurve BS,
                                                        out NCollection_HArray1<Geom2d_BSplineCurve> tabBS,
                                                        double AngularTolerance,
                                                        double Tolerance)
    {
        NCollection_Array1<int> BSMults = BS.InternalMultiplicities();
        NCollection_Array1<double> BSKnots = BS.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double U1, U2;
        bool closed_flag = false;
        gp_Pnt2d point1, point2;
        gp_Vec2d V1, V2;
        for (i = BS.FirstUKnotIndex(); i <= (BS.LastUKnotIndex() - 1); i++)
        {
            if (BSMults[i] == BS.Degree())
            {
                nbcurveC1++;
            }
        }

        nbcurveC1 = Min(nbcurveC1, BS.NbKnots() - 1);

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<Geom2d_BSplineCurve> ArrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> ArrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                ArrayOfToler[i] = Tolerance;
            }

            U2 = BS.FirstParameter();
            j = BS.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                U1 = U2;
                while (j < BS.LastUKnotIndex() && BSMults[j] < BS.Degree())
                {
                    j++;
                }

                U2 = BSKnots[j];
                j++;
                Geom2d_BSplineCurve BSbis = (Geom2d_BSplineCurve)BS.Copy();
                BSbis.Segment(U1, U2);
                ArrayOfCurves[i] = BSbis;
            }

            BS.D1(BS.FirstParameter(), out point1, out V1);
            BS.D1(BS.LastParameter(), out point2, out V2);

            // OCCT compares the squared distance with Tolerance, not its square (C0BSplineToC1BSplineCurve squares it)
            if (((point1.SquareDistance(point2) < Tolerance)) && (V1.IsParallel(V2, AngularTolerance)))
            {
                closed_flag = true;
            }

            Geom2dConvert.ConcatC1(ArrayOfCurves, ArrayOfToler, out _, out tabBS, ref closed_flag, Tolerance,
                                   AngularTolerance);
        }
        else
        {
            tabBS = new NCollection_HArray1<Geom2d_BSplineCurve>(0, 0);
            tabBS.SetValue(0, BS);
        }
    }
}
