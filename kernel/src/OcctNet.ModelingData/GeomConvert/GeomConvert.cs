// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert.cxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Conversions of 3D curves and surfaces to B-splines: of any bounded curve or surface (conics and elementary
/// surfaces exactly, offset and other curves and surfaces by approximation), splitting of B-spline curves and
/// surfaces between knots or parameters, concatenation of B-spline curves into G1 or C1 curves, and splitting of C0
/// B-spline curves into C1 ones.
/// </summary>
/// <remarks>
/// Every curve and surface these functions return belongs to the caller and is not frozen; it may be one of the
/// caller's own curves where OCCT returns it as is (a group of one curve in ConcatG1/ConcatC1, a C1 curve in
/// C0BSplineToArrayOfC1BSplineCurve). The caller's curves are never modified: where OCCT changes a curve of the
/// input array (the weight normalisation of ConcatG1 and ConcatC1), the port puts a modified copy in the array.
/// </remarks>
public static class GeomConvert
{
    private static Geom_BSplineCurve BSplineCurveBuilder(Geom_Conic TheConic, Convert_ConicToBSplineCurve Convert)
    {
        Geom_BSplineCurve TheCurve;
        NCollection_Array1<gp_Pnt2d> aPoles2d = Convert.Poles();
        NCollection_Array1<double> aWeights = Convert.Weights();
        NCollection_Array1<double> aKnots = Convert.Knots();
        NCollection_Array1<int> aMults = Convert.Multiplicities();
        NCollection_Array1<gp_Pnt> Poles = new(1, aPoles2d.Length());
        for (int i = aPoles2d.Lower(); i <= aPoles2d.Upper(); i++)
        {
            gp_Pnt2d aP2d = aPoles2d[i];
            Poles[i].SetCoord(aP2d.X(), aP2d.Y(), 0.0);
        }

        TheCurve = new Geom_BSplineCurve(Poles, aWeights, aKnots, aMults, Convert.Degree(), Convert.IsPeriodic());
        gp_Trsf T = new();
        T.SetTransformation(new gp_Ax3(TheConic.Position()), new gp_Ax3(gp.XOY()));
        Geom_BSplineCurve Cres;
        Cres = (Geom_BSplineCurve)TheCurve.Transformed(T);
        return Cres;
    }

    /// <summary>
    /// The part of C between its knots FromK1 and ToK2. For a non-periodic curve, it is reversed if FromK1 &gt; ToK2;
    /// for a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="Standard_DomainError">FromK1 = ToK2.</exception>
    /// <exception cref="Standard_DomainError">A knot index is outside [FirstUKnotIndex, LastUKnotIndex].</exception>
    public static Geom_BSplineCurve SplitBSplineCurve(Geom_BSplineCurve C, int FromK1, int ToK2,
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
            throw new Standard_DomainError();
        }

        Geom_BSplineCurve C1 = (Geom_BSplineCurve)C.Copy();

        C1.Segment(C.Knot(FirstK), C.Knot(LastK));

        if (C.IsPeriodic())
        {
            if (!SameOrientation)
            {
                C1.Reverse();
            }
        }
        else
        {
            if (FromK1 > ToK2)
            {
                C1.Reverse();
            }
        }

        return C1;
    }

    /// <summary>
    /// The part of C between the parameters FromU1 and ToU2. For a non-periodic curve, it is reversed if
    /// FromU1 &gt; ToU2; for a periodic one, if SameOrientation is false. ParametricTolerance is not used.
    /// </summary>
    public static Geom_BSplineCurve SplitBSplineCurve(Geom_BSplineCurve C, double FromU1, double ToU2,
                                                        double ParametricTolerance, bool SameOrientation = true)
    {
        double FirstU = Min(FromU1, ToU2);
        double LastU = Max(FromU1, ToU2);

        Geom_BSplineCurve C1 = (Geom_BSplineCurve)C.Copy();

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
    public static Geom_BSplineCurve CurveToBSplineCurve(Geom_Curve C,
                                                          Convert_ParameterisationType Parameterisation = Convert_TgtThetaOver2)
    {
        Geom_BSplineCurve TheCurve;

        if (C is Geom_TrimmedCurve Ctrim)
        {
            Geom_Curve Curv;
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

            if (Curv is Geom_Line)
            {
                gp_Pnt Pdeb = Ctrim.StartPoint();
                gp_Pnt Pfin = Ctrim.EndPoint();
                NCollection_Array1<gp_Pnt> Poles = new(1, 2);
                Poles[1] = Pdeb;
                Poles[2] = Pfin;
                NCollection_Array1<double> Knots = new(1, 2);
                Knots[1] = Ctrim.FirstParameter();
                Knots[2] = Ctrim.LastParameter();
                NCollection_Array1<int> Mults = new(1, 2);
                Mults[1] = 2;
                Mults[2] = 2;
                int Degree = 1;
                TheCurve = new Geom_BSplineCurve(Poles, Knots, Mults, Degree);
            }

            else if (Curv is Geom_Circle TheCircle)
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

                        Geom_BSplineCurve TheCurve1 = BSplineCurveBuilder(TheCircle, Convert1);

                        Convert_CircleToBSplineCurve Convert2 = new(C2d, Umed, U2, Parameterisation);

                        Geom_BSplineCurve TheCurve2 = BSplineCurveBuilder(TheCircle, Convert2);

                        GeomConvert_CompCurveToBSplineCurve CCTBSpl = new(TheCurve1, Parameterisation);

                        CCTBSpl.Add(TheCurve2, Precision.PConfusion(), true);

                        TheCurve = CCTBSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (Curv is Geom_Ellipse TheEllipse)
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

                        Geom_BSplineCurve TheCurve1 = BSplineCurveBuilder(TheEllipse, Convert1);

                        Convert_EllipseToBSplineCurve Convert2 = new(E2d, Umed, U2, Parameterisation);

                        Geom_BSplineCurve TheCurve2 = BSplineCurveBuilder(TheEllipse, Convert2);

                        GeomConvert_CompCurveToBSplineCurve CCTBSpl = new(TheCurve1, Parameterisation);

                        CCTBSpl.Add(TheCurve2, Precision.PConfusion(), true);

                        TheCurve = CCTBSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (Curv is Geom_Hyperbola TheHyperbola)
            {
                gp_Hypr2d H2d = new(gp.OX2d(), TheHyperbola.MajorRadius(), TheHyperbola.MinorRadius());
                Convert_HyperbolaToBSplineCurve Convert = new(H2d, U1, U2);
                TheCurve = BSplineCurveBuilder(TheHyperbola, Convert);
            }

            else if (Curv is Geom_Parabola TheParabola)
            {
                gp_Parab2d Prb2d = new(gp.OX2d(), TheParabola.Focal());
                Convert_ParabolaToBSplineCurve Convert = new(Prb2d, U1, U2);
                TheCurve = BSplineCurveBuilder(TheParabola, Convert);
            }

            else if (Curv is Geom_BezierCurve)
            {
                Geom_BezierCurve CBez = (Geom_BezierCurve)Curv.Copy();

                CBez.Segment(U1, U2);
                int Degree = CBez.Degree();
                NCollection_Array1<double> Knots = new(1, 2);
                NCollection_Array1<int> Mults = new(1, 2);
                Knots[1] = 0.0;
                Knots[2] = 1.0;
                Mults[1] = Degree + 1;
                Mults[2] = Degree + 1;
                NCollection_Array1<gp_Pnt> Poles = CBez.InternalPoles();
                if (CBez.IsRational())
                {
                    TheCurve = new Geom_BSplineCurve(Poles, CBez.InternalWeightsArray(), Knots, Mults, Degree);
                }
                else
                {
                    TheCurve = new Geom_BSplineCurve(Poles, Knots, Mults, Degree);
                }
            }

            else if (Curv is Geom_BSplineCurve)
            {
                TheCurve = (Geom_BSplineCurve)Curv.Copy();
                //// modified by jgv, 14.01.05 for OCC7355 ////
                if (TheCurve.IsPeriodic())
                {
                    double Uf = TheCurve.FirstParameter();
                    double Ul = TheCurve.LastParameter();
                    ElCLib.AdjustPeriodic(Uf, Ul, Precision.Confusion(), ref U1, ref U2);
                    if (Math.Abs(U1 - Uf) <= Precision.Confusion() && Math.Abs(U2 - Ul) <= Precision.Confusion())
                    {
                        TheCurve.SetNotPeriodic();
                    }
                }
                ///////////////////////////////////////////////
                TheCurve.Segment(U1, U2);
            }

            else if (Curv is Geom_OffsetCurve)
            {
                double Tol3d = 1.0e-4;
                GeomAbs_Shape Order = GeomAbs_C2;
                int MaxSegments = 16, MaxDegree = 14;
                GeomConvert_ApproxCurve ApprCOffs = new(C, Tol3d, Order, MaxSegments, MaxDegree);
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
            if (C is Geom_Ellipse TheEllipse)
            {
                gp_Elips2d E2d = new(gp.OX2d(), TheEllipse.MajorRadius(), TheEllipse.MinorRadius());
                Convert_EllipseToBSplineCurve Convert = new(E2d, Parameterisation);
                TheCurve = BSplineCurveBuilder(TheEllipse, Convert);
                TheCurve.SetPeriodic();
            }

            else if (C is Geom_Circle TheCircle)
            {
                gp_Circ2d C2d = new(gp.OX2d(), TheCircle.Radius());
                Convert_CircleToBSplineCurve Convert = new(C2d, Parameterisation);
                TheCurve = BSplineCurveBuilder(TheCircle, Convert);
                TheCurve.SetPeriodic();
            }

            else if (C is Geom_BezierCurve CBez)
            {
                int Degree = CBez.Degree();
                NCollection_Array1<double> Knots = new(1, 2);
                NCollection_Array1<int> Mults = new(1, 2);
                Knots[1] = 0.0;
                Knots[2] = 1.0;
                Mults[1] = Degree + 1;
                Mults[2] = Degree + 1;
                NCollection_Array1<gp_Pnt> Poles = CBez.InternalPoles();
                if (CBez.IsRational())
                {
                    TheCurve = new Geom_BSplineCurve(Poles, CBez.InternalWeightsArray(), Knots, Mults, Degree);
                }
                else
                {
                    TheCurve = new Geom_BSplineCurve(Poles, Knots, Mults, Degree);
                }
            }
            else if (C is Geom_BSplineCurve)
            {
                TheCurve = (Geom_BSplineCurve)C.Copy();
            }

            else if (C is Geom_OffsetCurve)
            {
                double Tol3d = 1.0e-4;
                GeomAbs_Shape Order = GeomAbs_C2;
                int MaxSegments = 16, MaxDegree = 14;
                GeomConvert_ApproxCurve ApprCOffs = new(C, Tol3d, Order, MaxSegments, MaxDegree);
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

        return TheCurve;
    }

    private sealed class GeomConvert_law_evaluator : BSplCLib_EvaluatorFunction
    {
        private readonly Geom2d_BSplineCurve? myAncore;

        public GeomConvert_law_evaluator(Geom2d_BSplineCurve? theAncore) => myAncore = theAncore;

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

    private static Geom_BSplineCurve MultNumandDenom(Geom2d_BSplineCurve a, Geom_BSplineCurve BS)
    {
        NCollection_Array1<double> BSKnots = BS.InternalKnots();
        NCollection_Array1<int> BSMults = BS.InternalMultiplicities();
        NCollection_Array1<gp_Pnt> BSPoles = new(BS.InternalPoles());
        NCollection_Array1<double> BSWeights = BS.InternalWeightsArray();
        NCollection_Array1<double> BSFlatKnots = BS.InternalKnotSequence();
        Geom_BSplineCurve res;
        double start_value, end_value;
        double tolerance = Precision.PConfusion();
        int resNbPoles, degree, ii, jj, aStatus;

        start_value = BSKnots[1];
        end_value = BSKnots[BS.NbKnots()];
        if ((end_value - start_value) / 5 < tolerance)
        {
            tolerance = (end_value - start_value) / 5;
        }

        NCollection_Array1<double> aKnots = new(a.InternalKnots());
        NCollection_Array1<gp_Pnt2d> aPoles = a.InternalPoles();
        NCollection_Array1<int> aMults = a.InternalMultiplicities();
        BSplCLib.Reparametrize(BS.FirstParameter(), BS.LastParameter(), aKnots);
        Geom2d_BSplineCurve anAncore = new(aPoles, aKnots, aMults, a.Degree());

        BSplCLib.MergeBSplineKnots(tolerance,
                                   start_value,
                                   end_value, // merge of the knots
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
        NCollection_Array1<gp_Pnt> resNumPoles = new(1, resNbPoles);
        NCollection_Array1<double> resDenPoles = new(1, resNbPoles);
        NCollection_Array1<gp_Pnt> resPoles = new(1, resNbPoles);
        NCollection_Array1<double> resFlatKnots = new(1, resNbPoles + degree + 1);
        BSplCLib.KnotSequence(resKnots, resMults, resFlatKnots);
        for (ii = 1; ii <= BS.NbPoles(); ii++)
        {
            for (jj = 1; jj <= 3; jj++)
            {
                BSPoles[ii].SetCoord(jj, BSPoles[ii].Coord(jj) * BSWeights[ii]);
            }
        }

        // POP for WNT
        GeomConvert_law_evaluator ev = new(anAncore);

        BSplCLib.FunctionMultiply(ev, BS.Degree(), BSFlatKnots, BSPoles, resFlatKnots, degree, resNumPoles, out aStatus);

        BSplCLib.FunctionMultiply(ev, BS.Degree(), BSFlatKnots, BSWeights, resFlatKnots, degree, resDenPoles, out aStatus);
        for (ii = 1; ii <= resNbPoles; ii++)
        {
            for (jj = 1; jj <= 3; jj++)
            {
                resPoles[ii].SetCoord(jj, resNumPoles[ii].Coord(jj) / resDenPoles[ii]);
            }
        }

        res = new Geom_BSplineCurve(resPoles, resDenPoles, resKnots, resMults, degree);
        return res;
    }

    private static void Pretreatment(NCollection_Array1<Geom_BSplineCurve> tab)
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
                    Geom_BSplineCurve aCurve = (Geom_BSplineCurve)tab[i].Copy();
                    for (j = 1; j <= aCurve.NbPoles(); j++)
                    {
                        aCurve.SetWeight(j, aCurve.Weight(j) / a);
                    }

                    tab[i] = aCurve;
                }
            }
        }
    }

    private static bool NeedToBeTreated(Geom_BSplineCurve BS)
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

    private static bool Need2DegRepara(NCollection_Array1<Geom_BSplineCurve> tab)
    {
        int i;
        gp_Vec Vec1, Vec2;
        double Rapport = 1.0e0;

        for (i = 0; i <= tab.Length() - 2; i++)
        {
            tab[i + 1].D1(tab[i + 1].FirstParameter(), out _, out Vec1);
            tab[i].D1(tab[i].LastParameter(), out _, out Vec2);
            Rapport = Rapport * Vec2.Magnitude() / Vec1.Magnitude();
        }

        return (Rapport > (1.0e0 + Precision.Confusion())) || (Rapport < (1.0e0 - Precision.Confusion()));
    }

    private static int Indexmin(NCollection_Array1<Geom_BSplineCurve> tab)
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

    private static void ReorderArrayOfG1Curves(NCollection_Array1<Geom_BSplineCurve> ArrayOfCurves,
                                         NCollection_Array1<double> ArrayOfToler,
                                         NCollection_Array1<bool> tabG1,
                                         int StartIndex,
                                         double ClosedTolerance)
    {
        int i;
        NCollection_Array1<Geom_BSplineCurve> ArraybisOfCurves = new(0, ArrayOfCurves.Length() - 1);
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

    private sealed class GeomConvert_reparameterise_evaluator : BSplCLib_EvaluatorFunction
    {
        private readonly double[] myPolynomialCoefficient = new double[3];

        public GeomConvert_reparameterise_evaluator(ReadOnlySpan<double> thePolynomialCoefficient) =>
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
    /// tolerances of the junctions. If ClosedG1Flag is true and the curves form one closed G1 group, the result is one
    /// periodic curve, closed within ClosedTolerance; ClosedG1Flag is set to false if the closing reparametrisation
    /// would raise the degree beyond the maximum. ArrayOfCurves may be reordered, and its rational curves with equal
    /// end weights replaced by normalised copies.
    /// </summary>
    /// <exception cref="Standard_Failure">Two consecutive curves don't join.</exception>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void ConcatG1(NCollection_Array1<Geom_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<Geom_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedG1Flag,
                                double ClosedTolerance)
    {
        int nb_curve = ArrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            First, PreLast = 0;
        GeomAbs_Shape Cont;
        gp_Vec Vec1, Vec2; // consecutive tangential vectors
        Geom_BSplineCurve Curve1, Curve2;
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
                Cont = GeomLProp.Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true);
                if (Cont < GeomAbs_C0)
                {
                    throw new Standard_ConstructionError("GeomConvert curves not C0");
                }
                else
                {
                    if (Cont >= GeomAbs_G1)
                    {
                        tabG1[i - 1] = true; // True=G1 continuity
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

        if ((ClosedG1Flag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1Curves(ArrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, ClosedTolerance);
        }

        ArrayOfConcatenated = new NCollection_HArray1<Geom_BSplineCurve>(0, nb_group - 1);

        bool fusion;
        index = 0;
        Pretreatment(ArrayOfCurves);

        Span<double> aPolynomialCoefficient = stackalloc double[3];

        bool NeedDoubleDegRepara = Need2DegRepara(ArrayOfCurves);
        if (nb_group == 1 && ClosedG1Flag && NeedDoubleDegRepara)
        {
            Curve1 = ArrayOfCurves[nb_curve - 1];
            // OCCT reads Geom2d_BSplineCurve's maximum degree, which is Geom_BSplineCurve's (25)
            if (Curve1.Degree() > Geom2d_BSplineCurve.MaxDegree() / 2)
            {
                ClosedG1Flag = false;
            }
        }

        if ((nb_group == 1) && (ClosedG1Flag))
        { // treatment of a particular case
            indexmin = Indexmin(ArrayOfCurves);
            if (indexmin != (ArrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1Curves(ArrayOfCurves, local_tolerance, tabG1, indexmin, ClosedTolerance);
            }

            Curve2 = ArrayOfCurves[0];
            for (j = 1; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                Curve1 = ArrayOfCurves[j];
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

                    NCollection_Array1<gp_Pnt> Curve1Poles = new(Curve1.InternalPoles());

                    for (ii = 1; ii <= Curve1.NbKnots(); ii++)
                    {
                        KnotC1Mults[ii] = (Curve1.Degree() + KnotC1Mults[ii]);
                    }

                    NCollection_Array1<double> FlatKnots = new(1, Curve1FlatKnots.Length() + (Curve1.Degree() * Curve1.NbKnots()));

                    BSplCLib.KnotSequence(KnotC1, KnotC1Mults, FlatKnots);
                    NCollection_Array1<gp_Pnt> NewPoles = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                    int aStatus;
                    NCollection_Array1<double> Curve1Weights = Curve1.InternalWeightsArray();
                    for (ii = 1; ii <= Curve1.NbPoles(); ii++)
                    {
                        for (jj = 1; jj <= 3; jj++)
                        {
                            Curve1Poles[ii].SetCoord(jj, Curve1Poles[ii].Coord(jj) * Curve1Weights[ii]);
                        }
                    }

                    // POP for WNT
                    GeomConvert_reparameterise_evaluator ev = new(aPolynomialCoefficient);
                    BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Poles, FlatKnots,
                                                    2 * Curve1.Degree(), NewPoles, out aStatus);
                    NCollection_Array1<double> NewWeights = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                    BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Weights, FlatKnots,
                                                    2 * Curve1.Degree(), NewWeights, out aStatus);
                    for (ii = 1; ii <= NewPoles.Length(); ii++)
                    {
                        for (jj = 1; jj <= 3; jj++)
                        {
                            NewPoles[ii].SetCoord(jj, NewPoles[ii].Coord(jj) / NewWeights[ii]);
                        }
                    }

                    Curve1 = new Geom_BSplineCurve(NewPoles, NewWeights, KnotC1, KnotC1Mults, 2 * Curve1.Degree());
                }

                GeomConvert_CompCurveToBSplineCurve C = new(Curve2);
                fusion = C.Add(Curve1, local_tolerance[j - 1]); // merge two adjacent curves
                if (!fusion)
                {
                    throw new Standard_ConstructionError("GeomConvert Concatenation Error");
                }

                Curve2 = C.InternalBSplineCurve()!;
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                Curve2 = (Geom_BSplineCurve)Curve2.Copy();
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
                        GeomConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(i));
                        // OCCT reads the caller's tolerances, not the ones reordered for a closed wire (occt-issues.md)
                        fusion = C.Add(Curve1, ArrayOfToler[j - 1]); // fusion de deux courbes adjacentes

                        if (!fusion)
                        {
                            throw new Standard_ConstructionError("GeomConvert Concatenation Error");
                        }

                        ArrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// <see cref="ConcatC1(NCollection_Array1{Geom_BSplineCurve}, NCollection_Array1{double}, out NCollection_HArray1{int}, out NCollection_HArray1{Geom_BSplineCurve}, ref bool, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void ConcatC1(NCollection_Array1<Geom_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<int> ArrayOfIndices,
                                out NCollection_HArray1<Geom_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedG1Flag,
                                double ClosedTolerance)
    {
        ConcatC1(ArrayOfCurves, ArrayOfToler, out ArrayOfIndices, out ArrayOfConcatenated, ref ClosedG1Flag,
                 ClosedTolerance, Precision.Angular());
    }

    /// <summary>
    /// As <see cref="ConcatG1"/>, but the curves are made C1 where they join with G1 continuity (rational curves
    /// are reparametrised first), and AngularTolerance is the tolerance of the G1 test. ArrayOfIndices (from 0) holds
    /// the indices in ArrayOfCurves of the first curve of each concatenated group, and a last index.
    /// </summary>
    /// <exception cref="Standard_Failure">Two consecutive curves don't join.</exception>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void ConcatC1(NCollection_Array1<Geom_BSplineCurve> ArrayOfCurves,
                                NCollection_Array1<double> ArrayOfToler,
                                out NCollection_HArray1<int> ArrayOfIndices,
                                out NCollection_HArray1<Geom_BSplineCurve> ArrayOfConcatenated,
                                ref bool ClosedG1Flag,
                                double ClosedTolerance,
                                double AngularTolerance)
    {
        int nb_curve = ArrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            First, PreLast = 0;
        GeomAbs_Shape Cont;
        gp_Vec Vec1, Vec2; // consecutive tangential vectors
        Geom_BSplineCurve Curve1, Curve2 = null!;
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
                Cont = GeomLProp.Continuity(ArrayOfCurves[i - 1], ArrayOfCurves[i], PreLast, First, true, true,
                                            local_tolerance[i - 1], AngularTolerance);
                if (Cont < GeomAbs_C0)
                {
                    throw new Standard_ConstructionError("GeomConvert curves not C0");
                }
                else
                {
                    if (Cont >= GeomAbs_G1)
                    {
                        tabG1[i - 1] = true; // True=G1 continuity
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

        if ((ClosedG1Flag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1Curves(ArrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, ClosedTolerance);
        }

        ArrayOfIndices = new NCollection_HArray1<int>(0, nb_group);
        ArrayOfConcatenated = new NCollection_HArray1<Geom_BSplineCurve>(0, nb_group - 1);

        bool fusion;
        int k = 0;
        index = 0;
        Pretreatment(ArrayOfCurves);
        Span<double> aPolynomialCoefficient = stackalloc double[3];

        bool NeedDoubleDegRepara = Need2DegRepara(ArrayOfCurves);
        if (nb_group == 1 && ClosedG1Flag && NeedDoubleDegRepara)
        {
            Curve1 = ArrayOfCurves[nb_curve - 1];
            // OCCT reads Geom2d_BSplineCurve's maximum degree, which is Geom_BSplineCurve's (25)
            if (Curve1.Degree() > Geom2d_BSplineCurve.MaxDegree() / 2)
            {
                ClosedG1Flag = false;
            }
        }

        if ((nb_group == 1) && (ClosedG1Flag))
        { // treatment of a particular case
            ArrayOfIndices.SetValue(0, 0);
            ArrayOfIndices.SetValue(1, 0);
            indexmin = Indexmin(ArrayOfCurves);
            if (indexmin != (ArrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1Curves(ArrayOfCurves, local_tolerance, tabG1, indexmin, ClosedTolerance);
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

                        NCollection_Array1<gp_Pnt> Curve1Poles = new(Curve1.InternalPoles());

                        for (ii = 1; ii <= Curve1.NbKnots(); ii++)
                        {
                            KnotC1Mults[ii] = (Curve1.Degree() + KnotC1Mults[ii]);
                        }

                        NCollection_Array1<double> FlatKnots = new(1, Curve1FlatKnots.Length() + (Curve1.Degree() * Curve1.NbKnots()));

                        BSplCLib.KnotSequence(KnotC1, KnotC1Mults, FlatKnots);
                        NCollection_Array1<gp_Pnt> NewPoles = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                        int aStatus;
                        NCollection_Array1<double> Curve1Weights = Curve1.InternalWeightsArray();
                        for (ii = 1; ii <= Curve1.NbPoles(); ii++)
                        {
                            for (jj = 1; jj <= 3; jj++)
                            {
                                Curve1Poles[ii].SetCoord(jj, Curve1Poles[ii].Coord(jj) * Curve1Weights[ii]);
                            }
                        }

                        // POP for WNT
                        GeomConvert_reparameterise_evaluator ev = new(aPolynomialCoefficient);
                        BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Poles, FlatKnots,
                                                        2 * Curve1.Degree(), NewPoles, out aStatus);
                        NCollection_Array1<double> NewWeights = new(1, FlatKnots.Length() - (2 * Curve1.Degree() + 1));
                        BSplCLib.FunctionReparameterise(ev, Curve1.Degree(), Curve1FlatKnots, Curve1Weights, FlatKnots,
                                                        2 * Curve1.Degree(), NewWeights, out aStatus);
                        for (ii = 1; ii <= NewPoles.Length(); ii++)
                        {
                            for (jj = 1; jj <= 3; jj++)
                            {
                                NewPoles[ii].SetCoord(jj, NewPoles[ii].Coord(jj) / NewWeights[ii]);
                            }
                        }

                        Curve1 = new Geom_BSplineCurve(NewPoles, NewWeights, KnotC1, KnotC1Mults, 2 * Curve1.Degree());
                    }

                    GeomConvert_CompCurveToBSplineCurve C = new(Curve2);
                    fusion = C.Add(Curve1, local_tolerance[j - 1]); // merge two adjacent curves
                    if (!fusion)
                    {
                        throw new Standard_ConstructionError("GeomConvert Concatenation Error");
                    }

                    Curve2 = C.InternalBSplineCurve()!;
                }
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                Curve2 = (Geom_BSplineCurve)Curve2.Copy();
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

                if ((!ClosedG1Flag) || (nb_group == 1))
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
                        // Merge of two consecutive curves.
                        GeomConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(i));
                        fusion = C.Add(Curve1, local_tolerance[j - 1], true);
                        if (!fusion)
                        {
                            throw new Standard_ConstructionError("GeomConvert Concatenation Error");
                        }

                        ArrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// Makes BS C1 where it is only C0 but G1 within the tolerances: BS is cut at its C0 knots, the pieces are
    /// concatenated by <see cref="C0BSplineToArrayOfC1BSplineCurve(Geom_BSplineCurve, out NCollection_HArray1{Geom_BSplineCurve}, double, double)"/>
    /// and joined again. BS is replaced by a new curve; the curve given is not changed.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void C0BSplineToC1BSplineCurve(ref Geom_BSplineCurve BS, double tolerance, double AngularTol = 1.0e-7)
    {
        bool fusion;
        // the array with the resulting curves

        C0BSplineToArrayOfC1BSplineCurve(BS, out NCollection_HArray1<Geom_BSplineCurve> ArrayOfConcatenated, AngularTol,
                                         tolerance);

        GeomConvert_CompCurveToBSplineCurve C = new(ArrayOfConcatenated.Value(0));
        if (ArrayOfConcatenated.Length() >= 2)
        {
            int i;
            for (i = 1; i < ArrayOfConcatenated.Length(); i++)
            {
                fusion = C.Add(ArrayOfConcatenated.Value(i), tolerance);
                if (!fusion)
                {
                    throw new Standard_ConstructionError("GeomConvert Concatenation Error");
                }
            }
        }

        BS = C.InternalBSplineCurve()!;
    }

    /// <summary>
    /// <see cref="C0BSplineToArrayOfC1BSplineCurve(Geom_BSplineCurve, out NCollection_HArray1{Geom_BSplineCurve}, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void C0BSplineToArrayOfC1BSplineCurve(Geom_BSplineCurve BS,
                                                        out NCollection_HArray1<Geom_BSplineCurve> tabBS,
                                                        double tolerance)
    {
        C0BSplineToArrayOfC1BSplineCurve(BS, out tabBS, Precision.Angular(), tolerance);
    }

    /// <summary>
    /// Cuts BS at its C0 knots and concatenates the pieces that join with G1 continuity (within the tolerances) into
    /// C1 curves, returned in tabBS (from 0). A curve without C0 knots is returned as the single element.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A concatenation fails.</exception>
    public static void C0BSplineToArrayOfC1BSplineCurve(Geom_BSplineCurve BS,
                                                        out NCollection_HArray1<Geom_BSplineCurve> tabBS,
                                                        double AngularTolerance,
                                                        double tolerance)
    {
        NCollection_Array1<int> BSMults = BS.InternalMultiplicities();
        NCollection_Array1<double> BSKnots = BS.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double U1, U2;
        bool closed_flag = false;
        gp_Vec V1, V2;
        for (i = BS.FirstUKnotIndex(); i <= (BS.LastUKnotIndex() - 1); i++)
        { // give the number of C1 curves
            if (BSMults[i] == BS.Degree())
            {
                nbcurveC1++;
            }
        }

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<Geom_BSplineCurve> ArrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> ArrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                // filling of the array of tolerances
                ArrayOfToler[i] = tolerance;
            }

            // with the variable tolerance
            U2 = BS.FirstParameter();
            j = BS.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                // filling of the array of curves
                U1 = U2;
                // with the curves C1 segmented
                while (BSMults[j] < BS.Degree() && j < BS.LastUKnotIndex())
                {
                    j++;
                }

                U2 = BSKnots[j];
                j++;
                Geom_BSplineCurve BSbis = (Geom_BSplineCurve)BS.Copy();
                BSbis.Segment(U1, U2);
                ArrayOfCurves[i] = BSbis;
            }

            BS.D1(BS.FirstParameter(), out _, out V1);
            BS.D1(BS.LastParameter(), out _, out V2);

            if ((BS.IsClosed()) && (V1.IsParallel(V2, AngularTolerance)))
            {
                // check if the BSpline is closed G1
                closed_flag = true;
            }

            ConcatC1(ArrayOfCurves, ArrayOfToler, out _, out tabBS, ref closed_flag, tolerance, AngularTolerance);
        }
        else
        {
            tabBS = new NCollection_HArray1<Geom_BSplineCurve>(0, 0);
            tabBS.SetValue(0, BS);
        }
    }

    // GeomConvert_1.cxx: the surfaces

    private static Geom_BSplineSurface BSplineSurfaceBuilder(Convert_ElementarySurfaceToBSplineSurface Convert)
    {
        Geom_BSplineSurface TheSurface;
        TheSurface = new Geom_BSplineSurface(Convert.Poles(),
                                             Convert.Weights(),
                                             Convert.UKnots(),
                                             Convert.VKnots(),
                                             Convert.UMultiplicities(),
                                             Convert.VMultiplicities(),
                                             Convert.UDegree(),
                                             Convert.VDegree(),
                                             Convert.IsUPeriodic(),
                                             Convert.IsVPeriodic());
        return TheSurface;
    }

    /// <summary>
    /// The part of S between its U knots FromUK1 and ToUK2 and its V knots FromVK1 and ToVK2. In a non-periodic
    /// direction, it is reversed if the first knot index is the greater; in a periodic one, if SameUOrientation
    /// (SameVOrientation) is false.
    /// </summary>
    /// <exception cref="Standard_DomainError">
    /// FromUK1 = ToUK2 or FromVK1 = ToVK2, or a knot index is outside the first and last knot indices.
    /// </exception>
    public static Geom_BSplineSurface SplitBSplineSurface(Geom_BSplineSurface S,
                                                          int FromUK1,
                                                          int ToUK2,
                                                          int FromVK1,
                                                          int ToVK2,
                                                          bool SameUOrientation = true,
                                                          bool SameVOrientation = true)
    {
        int FirstU = S.FirstUKnotIndex();
        int FirstV = S.FirstVKnotIndex();
        int LastU = S.LastUKnotIndex();
        int LastV = S.LastVKnotIndex();
        if (FromUK1 == ToUK2 || FromVK1 == ToVK2)
        {
            throw new Standard_DomainError();
        }

        int FirstUK = Min(FromUK1, ToUK2);
        int LastUK = Max(FromUK1, ToUK2);
        int FirstVK = Min(FromVK1, ToVK2);
        int LastVK = Max(FromVK1, ToVK2);
        if (FirstUK < FirstU || LastUK > LastU || FirstVK < FirstV || LastVK > LastV)
        {
            throw new Standard_DomainError();
        }

        Geom_BSplineSurface S1 = (Geom_BSplineSurface)S.Copy();

        S1.Segment(S1.UKnot(FirstUK), S1.UKnot(LastUK), S1.VKnot(FirstVK), S1.VKnot(LastVK));

        if (S.IsUPeriodic())
        {
            if (!SameUOrientation)
            {
                S1.UReverse();
            }
        }
        else
        {
            if (FromUK1 > ToUK2)
            {
                S1.UReverse();
            }
        }

        if (S.IsVPeriodic())
        {
            if (!SameVOrientation)
            {
                S1.VReverse();
            }
        }
        else
        {
            if (FromVK1 > ToVK2)
            {
                S1.VReverse();
            }
        }

        return S1;
    }

    /// <summary>
    /// The part of S between its knots FromK1 and ToK2 in U (USplit true) or V, whole in the other direction. In a
    /// non-periodic direction, it is reversed if FromK1 &gt; ToK2; in a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="Standard_DomainError">FromK1 = ToK2, or a knot index is outside the first and last.</exception>
    public static Geom_BSplineSurface SplitBSplineSurface(Geom_BSplineSurface S,
                                                          int FromK1,
                                                          int ToK2,
                                                          bool USplit,
                                                          bool SameOrientation = true)
    {
        if (FromK1 == ToK2)
        {
            throw new Standard_DomainError();
        }

        Geom_BSplineSurface S1 = (Geom_BSplineSurface)S.Copy();

        if (USplit)
        {
            int FirstU = S.FirstUKnotIndex();
            int LastU = S.LastUKnotIndex();
            int FirstUK = Min(FromK1, ToK2);
            int LastUK = Max(FromK1, ToK2);
            if (FirstUK < FirstU || LastUK > LastU)
            {
                throw new Standard_DomainError();
            }

            S1.Segment(S1.UKnot(FirstUK),
                       S1.UKnot(LastUK),
                       S1.VKnot(S1.FirstVKnotIndex()),
                       S1.VKnot(S1.LastVKnotIndex()));

            if (S.IsUPeriodic())
            {
                if (!SameOrientation)
                {
                    S1.UReverse();
                }
            }
            else
            {
                if (FromK1 > ToK2)
                {
                    S1.UReverse();
                }
            }
        }
        else
        {
            int FirstV = S.FirstVKnotIndex();
            int LastV = S.LastVKnotIndex();
            int FirstVK = Min(FromK1, ToK2);
            int LastVK = Max(FromK1, ToK2);
            if (FirstVK < FirstV || LastVK > LastV)
            {
                throw new Standard_DomainError();
            }

            S1.Segment(S1.UKnot(S1.FirstUKnotIndex()),
                       S1.UKnot(S1.LastUKnotIndex()),
                       S1.VKnot(FirstVK),
                       S1.VKnot(LastVK));

            if (S.IsVPeriodic())
            {
                if (!SameOrientation)
                {
                    S1.VReverse();
                }
            }
            else
            {
                if (FromK1 > ToK2)
                {
                    S1.VReverse();
                }
            }
        }

        return S1;
    }

    /// <summary>
    /// The part of S between the parameters FromU1 and ToU2 in U and FromV1 and ToV2 in V. In a non-periodic
    /// direction, it is reversed if the first parameter is the greater; in a periodic one, if SameUOrientation
    /// (SameVOrientation) is false. ParametricTolerance is not used.
    /// </summary>
    public static Geom_BSplineSurface SplitBSplineSurface(Geom_BSplineSurface S,
                                                          double FromU1,
                                                          double ToU2,
                                                          double FromV1,
                                                          double ToV2,
                                                          double ParametricTolerance,
                                                          bool SameUOrientation = true,
                                                          bool SameVOrientation = true)
    {
        double FirstU = Min(FromU1, ToU2);
        double LastU = Max(FromU1, ToU2);
        double FirstV = Min(FromV1, ToV2);
        double LastV = Max(FromV1, ToV2);

        Geom_BSplineSurface NewSurface = (Geom_BSplineSurface)S.Copy();

        NewSurface.Segment(FirstU, LastU, FirstV, LastV);

        if (S.IsUPeriodic())
        {
            if (!SameUOrientation)
            {
                NewSurface.UReverse();
            }
        }
        else
        {
            if (FromU1 > ToU2)
            {
                NewSurface.UReverse();
            }
        }

        if (S.IsVPeriodic())
        {
            if (!SameVOrientation)
            {
                NewSurface.VReverse();
            }
        }
        else
        {
            if (FromV1 > ToV2)
            {
                NewSurface.VReverse();
            }
        }

        return NewSurface;
    }

    /// <summary>
    /// The part of S between the parameters FromParam1 and ToParam2 in U (USplit true) or V, whole in the other
    /// direction. In U, it is reversed if FromParam1 &gt; ToParam2 (non-periodic) or SameOrientation is false
    /// (periodic). In V, OCCT tests and reverses U instead (occt-issues.md).
    /// </summary>
    /// <exception cref="Standard_DomainError">The parameters are within ParametricTolerance.</exception>
    public static Geom_BSplineSurface SplitBSplineSurface(Geom_BSplineSurface S,
                                                          double FromParam1,
                                                          double ToParam2,
                                                          bool USplit,
                                                          double ParametricTolerance,
                                                          bool SameOrientation = true)
    {
        if (Math.Abs(FromParam1 - ToParam2) <= Math.Abs(ParametricTolerance))
        {
            throw new Standard_DomainError();
        }

        Geom_BSplineSurface NewSurface = (Geom_BSplineSurface)S.Copy();

        if (USplit)
        {
            double FirstU = Min(FromParam1, ToParam2);
            double LastU = Max(FromParam1, ToParam2);
            double FirstV = S.VKnot(S.FirstVKnotIndex());
            double LastV = S.VKnot(S.LastVKnotIndex());

            NewSurface.Segment(FirstU, LastU, FirstV, LastV);

            if (S.IsUPeriodic())
            {
                if (!SameOrientation)
                {
                    NewSurface.UReverse();
                }
            }
            else
            {
                if (FromParam1 > ToParam2)
                {
                    NewSurface.UReverse();
                }
            }
        }
        else
        {
            double FirstU = S.UKnot(S.FirstUKnotIndex());
            double LastU = S.UKnot(S.LastUKnotIndex());
            double FirstV = Min(FromParam1, ToParam2);
            double LastV = Max(FromParam1, ToParam2);

            NewSurface.Segment(FirstU, LastU, FirstV, LastV);

            // OCCT: a split in V tests the U periodicity and reverses U, where V is meant
            if (S.IsUPeriodic())
            {
                if (!SameOrientation)
                {
                    NewSurface.UReverse();
                }
            }
            else
            {
                if (FromParam1 > ToParam2)
                {
                    NewSurface.UReverse();
                }
            }
        }

        return NewSurface;
    }

    /// <summary>
    /// Sr as a B-spline surface: planes, quadrics (trimmed), surfaces of revolution and extrusion, Bezier and
    /// B-spline surfaces exactly; offset surfaces through their equivalent surface when they have one; other
    /// surfaces by approximation (GeomConvert_ApproxSurface, tolerance 1e-4, degree 14).
    /// </summary>
    /// <exception cref="Standard_DomainError">Sr is infinite.</exception>
    public static Geom_BSplineSurface SurfaceToBSplineSurface(Geom_Surface Sr)
    {
        Sr.Bounds(out double U1, out double U2, out double V1, out double V2);
        double UFirst = Min(U1, U2);
        double ULast = Max(U1, U2);
        double VFirst = Min(V1, V2);
        double VLast = Max(V1, V2);

        // If the surface Sr is infinite stop the computation
        if (Precision.IsNegativeInfinite(UFirst) || Precision.IsPositiveInfinite(ULast)
            || Precision.IsNegativeInfinite(VFirst) || Precision.IsPositiveInfinite(VLast))
        {
            throw new Standard_DomainError("GeomConvert::SurfaceToBSplineSurface() - infinite surface");
        }

        Geom_BSplineSurface TheSurface;
        Geom_Surface? S;
        if (Sr is Geom_OffsetSurface OffsetSur)
        {
            S = OffsetSur.Surface();
            if (S is not null)
            { // Convert the equivalent surface.
                return SurfaceToBSplineSurface(S);
            }
        }

        S = Sr;

        if (S is Geom_RectangularTrimmedSurface Strim)
        {
            Geom_Surface Surf = Strim.BasisSurface();
            UFirst = U1;
            ULast = U2;
            VFirst = V1;
            VLast = V2;
            if (Surf is Geom_OffsetSurface OffsetSurBasis)
            {
                S = OffsetSurBasis.Surface();
                if (S is not null)
                {
                    Surf = S;
                }
                else
                {
                    S = Surf;
                }
            }

            if (Surf is Geom_RectangularTrimmedSurface)
            {
                Geom_RectangularTrimmedSurface aStrim = new(Surf, UFirst, ULast, VFirst, VLast);
                return SurfaceToBSplineSurface(aStrim);
            }

            //
            // For cylinders, cones, spheres, toruses
            bool isUClosed = Math.Abs((ULast - UFirst) - 2.0 * M_PI) <= Precision.PConfusion();
            double eps = 100.0 * Epsilon(2.0 * M_PI);
            //
            if (Surf is Geom_Plane)
            {
                NCollection_Array2<gp_Pnt> Poles = new(1, 2, 1, 2);
                Poles[1, 1] = Strim.Value(U1, V1);
                Poles[1, 2] = Strim.Value(U1, V2);
                Poles[2, 1] = Strim.Value(U2, V1);
                Poles[2, 2] = Strim.Value(U2, V2);
                NCollection_Array1<double> UKnots = new(1, 2);
                NCollection_Array1<double> VKnots = new(1, 2);
                NCollection_Array1<int> UMults = new(1, 2);
                NCollection_Array1<int> VMults = new(1, 2);
                UKnots[1] = U1;
                UKnots[2] = U2;
                VKnots[1] = V1;
                VKnots[2] = V2;
                UMults[1] = 2;
                UMults[2] = 2;
                VMults[1] = 2;
                VMults[2] = 2;
                int UDegree = 1;
                int VDegree = 1;
                TheSurface = new Geom_BSplineSurface(Poles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);
            }
            else if (Surf is Geom_CylindricalSurface TheCylSurf)
            {
                gp_Cylinder Cyl = TheCylSurf.Cylinder();
                if (isUClosed)
                {
                    Convert_CylinderToBSplineSurface Convert = new(Cyl, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                    int aNbK = TheSurface.NbUKnots();
                    if (Math.Abs(TheSurface.UKnot(1) - UFirst) > eps || Math.Abs(TheSurface.UKnot(aNbK) - ULast) > eps)
                    {
                        TheSurface.CheckAndSegment(UFirst, ULast, VFirst, VLast);
                    }
                }
                else
                {
                    Convert_CylinderToBSplineSurface Conv = new(Cyl, UFirst, ULast, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Conv);
                }
            }

            else if (Surf is Geom_ConicalSurface TheConeSurf)
            {
                gp_Cone Co = TheConeSurf.Cone();
                if (isUClosed)
                {
                    Convert_ConeToBSplineSurface Convert = new(Co, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                    int aNbK = TheSurface.NbUKnots();
                    if (Math.Abs(TheSurface.UKnot(1) - UFirst) > eps || Math.Abs(TheSurface.UKnot(aNbK) - ULast) > eps)
                    {
                        TheSurface.CheckAndSegment(UFirst, ULast, VFirst, VLast);
                    }
                }
                else
                {
                    Convert_ConeToBSplineSurface Convert = new(Co, UFirst, ULast, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                }
            }

            else if (Surf is Geom_SphericalSurface TheSphSurf)
            {
                gp_Sphere Sph = TheSphSurf.Sphere();
                // OCC217
                if (isUClosed)
                {
                    Convert_SphereToBSplineSurface Convert = new(Sph, VFirst, VLast, false);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                    int aNbK = TheSurface.NbUKnots();
                    if (Math.Abs(TheSurface.UKnot(1) - UFirst) > eps || Math.Abs(TheSurface.UKnot(aNbK) - ULast) > eps)
                    {
                        TheSurface.CheckAndSegment(UFirst, ULast, VFirst, VLast);
                    }
                }
                else
                {
                    Convert_SphereToBSplineSurface Convert = new(Sph, UFirst, ULast, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                }
            }

            else if (Surf is Geom_ToroidalSurface TheTorSurf)
            {
                gp_Torus Tr = TheTorSurf.Torus();
                //
                // if isUClosed = true and U trim does not coincide with first period of torus,
                // method CheckAndSegment shifts position of U seam boundary of surface.
                // probably bug? So, for this case we must build not periodic surface.
                bool isUFirstPeriod = UFirst >= 0.0 && ULast <= 2.0 * M_PI;
                bool isVFirstPeriod = VFirst >= 0.0 && VLast <= 2.0 * M_PI;
                if (isUClosed && isUFirstPeriod)
                {
                    Convert_TorusToBSplineSurface Convert = new(Tr, VFirst, VLast, false);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                    int aNbK = TheSurface.NbUKnots();
                    if (Math.Abs(TheSurface.UKnot(1) - UFirst) > eps || Math.Abs(TheSurface.UKnot(aNbK) - ULast) > eps)
                    {
                        TheSurface.CheckAndSegment(UFirst, ULast, VFirst, VLast);
                    }
                }
                else if (Strim.IsVClosed() && isVFirstPeriod)
                {
                    Convert_TorusToBSplineSurface Convert = new(Tr, UFirst, ULast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                    int aNbK = TheSurface.NbVKnots();
                    if (Math.Abs(TheSurface.VKnot(1) - VFirst) > eps || Math.Abs(TheSurface.VKnot(aNbK) - VLast) > eps)
                    {
                        TheSurface.CheckAndSegment(UFirst, ULast, VFirst, VLast);
                    }
                }
                else
                {
                    Convert_TorusToBSplineSurface Convert = new(Tr, UFirst, ULast, VFirst, VLast);
                    TheSurface = BSplineSurfaceBuilder(Convert);
                }
            }

            else if (Surf is Geom_SurfaceOfRevolution Revol)
            {
                Geom_Curve Meridian = Revol.BasisCurve();
                Geom_BSplineCurve C;
                if (Strim.IsVClosed())
                {
                    C = CurveToBSplineCurve(Meridian);
                }
                else
                {
                    Geom_TrimmedCurve CT = new(Meridian, VFirst, VLast);
                    C = CurveToBSplineCurve(CT);
                }

                int NbUPoles, NbUKnots;
                int NbVPoles;
                bool periodic = false;

                // Poles of meridian = Vpoles
                NbVPoles = C.NbPoles();
                NCollection_Array1<gp_Pnt> Poles = new(C.InternalPoles());
                NCollection_Array1<double> Weights = C.InternalWeightsArray();

                double AlfaU;
                if (Strim.IsUPeriodic())
                {
                    NbUKnots = 4;
                    AlfaU = M_PI / 3.0;
                    NbUPoles = 6;
                    periodic = true;
                }
                else
                {
                    // Number of spans: maximum opening = 150 degrees ( = PI / 1.2 rad)
                    int nbUSpans = (int)Math.Truncate(1.2 * (ULast - UFirst) / M_PI) + 1;
                    AlfaU = (ULast - UFirst) / (nbUSpans * 2);
                    NbUPoles = 2 * nbUSpans + 1;
                    NbUKnots = nbUSpans + 1;
                }

                // Compute Knots and Mults
                NCollection_Array1<double> UKnots = new(1, NbUKnots);
                NCollection_Array1<int> UMults = new(1, NbUKnots);
                int i, j;
                for (i = 1; i <= NbUKnots; i++)
                {
                    UKnots[i] = UFirst + (i - 1) * 2 * AlfaU;
                    UMults[i] = 2;
                }

                if (!periodic)
                {
                    UMults[1]++;
                    UMults[NbUKnots]++;
                }

                NCollection_Array1<double> VKnots = C.InternalKnots();
                NCollection_Array1<int> VMults = C.InternalMultiplicities();

                // Compute the poles.
                NCollection_Array2<gp_Pnt> NewPoles = new(1, NbUPoles, 1, NbVPoles);
                NCollection_Array2<double> NewWeights = new(1, NbUPoles, 1, NbVPoles);
                gp_Trsf Trsf = new();

                for (i = 1; i <= NbUPoles; i += 2)
                {
                    Trsf.SetRotation(Revol.Axis(), UFirst + (i - 1) * AlfaU);
                    for (j = 1; j <= NbVPoles; j++)
                    {
                        NewPoles[i, j] = Poles[j].Transformed(Trsf);
                        NewWeights[i, j] = Weights[j];
                    }
                }

                gp_GTrsf Aff = new();
                Aff.SetAffinity(Revol.Axis(), 1 / Math.Cos(AlfaU));
                gp_XYZ coord;
                for (j = 1; j <= NbVPoles; j++)
                {
                    coord = Poles[j].XYZ();
                    Aff.Transforms(ref coord);
                    Poles[j].SetXYZ(coord);
                }

                for (i = 2; i <= NbUPoles; i += 2)
                {
                    Trsf.SetRotation(Revol.Axis(), UFirst + (i - 1) * AlfaU);
                    for (j = 1; j <= NbVPoles; j++)
                    {
                        NewPoles[i, j] = Poles[j].Transformed(Trsf);
                        NewWeights[i, j] = Weights[j] * Math.Cos(AlfaU);
                    }
                }

                TheSurface = new Geom_BSplineSurface(NewPoles,
                                                     NewWeights,
                                                     UKnots,
                                                     VKnots,
                                                     UMults,
                                                     VMults,
                                                     2,
                                                     C.Degree(),
                                                     periodic,
                                                     C.IsPeriodic());
            }

            else if (Surf is Geom_SurfaceOfLinearExtrusion Extru)
            {
                Geom_Curve Meridian = Extru.BasisCurve();
                Geom_BSplineCurve C;
                if (Strim.IsUClosed())
                {
                    C = CurveToBSplineCurve(Meridian);
                }
                else
                {
                    Geom_TrimmedCurve CT = new(Meridian, UFirst, ULast);
                    C = CurveToBSplineCurve(CT);
                }

                NCollection_Array2<gp_Pnt> Poles = new(1, C.NbPoles(), 1, 2);
                NCollection_Array2<double> Weights = new(1, C.NbPoles(), 1, 2);
                NCollection_Array1<double> UKnots = C.InternalKnots();
                NCollection_Array1<int> UMults = C.InternalMultiplicities();
                NCollection_Array1<double> VKnots = new(1, 2);
                VKnots[1] = VFirst;
                VKnots[2] = VLast;
                NCollection_Array1<int> VMults = new(1, 2);
                VMults.Init(2);

                gp_Vec D = new(Extru.Direction());
                gp_Vec DV1 = VFirst * D;
                gp_Vec DV2 = VLast * D;
                for (int i = 1; i <= C.NbPoles(); i++)
                {
                    Poles[i, 1] = C.Pole(i).Translated(DV1);
                    Poles[i, 2] = C.Pole(i).Translated(DV2);
                    Weights[i, 1] = Weights[i, 2] = C.Weight(i);
                }

                TheSurface = new Geom_BSplineSurface(Poles,
                                                     Weights,
                                                     UKnots,
                                                     VKnots,
                                                     UMults,
                                                     VMults,
                                                     C.Degree(),
                                                     1,
                                                     C.IsPeriodic(),
                                                     false);
            }

            else if (Surf is Geom_BezierSurface)
            {
                Geom_BezierSurface SBez = (Geom_BezierSurface)Surf.Copy();

                SBez.Segment(U1, U2, V1, V2);
                int UDegree = SBez.UDegree();
                int VDegree = SBez.VDegree();
                NCollection_Array1<double> UKnots = new(1, 2);
                NCollection_Array1<int> UMults = new(1, 2);
                NCollection_Array1<double> VKnots = new(1, 2);
                NCollection_Array1<int> VMults = new(1, 2);
                UKnots[1] = 0.0;
                UKnots[2] = 1.0;
                UMults[1] = UDegree + 1;
                UMults[2] = UDegree + 1;
                VKnots[1] = 0.0;
                VKnots[2] = 1.0;
                VMults[1] = VDegree + 1;
                VMults[2] = VDegree + 1;
                NCollection_Array2<gp_Pnt> aPoles = SBez.InternalPoles();
                if (SBez.IsURational() || SBez.IsVRational())
                {
                    TheSurface = new Geom_BSplineSurface(aPoles,
                                                         SBez.InternalWeightsArray(),
                                                         UKnots,
                                                         VKnots,
                                                         UMults,
                                                         VMults,
                                                         UDegree,
                                                         VDegree);
                }
                else
                {
                    TheSurface = new Geom_BSplineSurface(aPoles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);
                }
            }

            else if (Surf is Geom_BSplineSurface)
            {
                Geom_BSplineSurface BS = (Geom_BSplineSurface)Surf.Copy();
                BS.Bounds(out double umin, out double umax, out double vmin, out double vmax);
                if (!BS.IsUPeriodic())
                {
                    if (U1 < umin)
                    {
                        U1 = umin;
                    }

                    if (U2 > umax)
                    {
                        U2 = umax;
                    }
                }

                if (!BS.IsVPeriodic())
                {
                    if (V1 < vmin)
                    {
                        V1 = vmin;
                    }

                    if (V2 > vmax)
                    {
                        V2 = vmax;
                    }
                }

                if (BS.IsUPeriodic() || BS.IsVPeriodic())
                {
                    BS.CheckAndSegment(U1, U2, V1, V2);
                }
                else
                {
                    BS.Segment(U1, U2, V1, V2);
                }

                TheSurface = BS;
            }

            else
            {
                double Tol3d = 1.0e-4;
                int MaxDegree = 14, MaxSeg;
                GeomAbs_Shape cont;
                GeomAdaptor_Surface AS = new(Sr);
                if (AS.NbUIntervals(GeomAbs_C2) > 1 || AS.NbVIntervals(GeomAbs_C2) > 1)
                {
                    cont = GeomAbs_C1;
                }
                else
                {
                    cont = GeomAbs_C2;
                }

                MaxSeg = 4 * (AS.NbUIntervals(GeomAbs_CN) + 1) * (AS.NbVIntervals(GeomAbs_CN) + 1);
                GeomConvert_ApproxSurface BSpS = new(Sr, Tol3d, cont, cont, MaxDegree, MaxDegree, MaxSeg, 1);
                // The approximation's surface is frozen (decision 197): the caller gets a copy it owns
                TheSurface = (Geom_BSplineSurface)BSpS.Surface()!.Copy();
            }
        } // End of Rectangular::TrimmedSurface case

        else
        {
            if (S is Geom_SphericalSurface TheSphSurf)
            {
                gp_Sphere Sph = TheSphSurf.Sphere();
                Convert_SphereToBSplineSurface Convert = new(Sph);
                TheSurface = BSplineSurfaceBuilder(Convert);
            }

            else if (S is Geom_ToroidalSurface TheTorSurf)
            {
                gp_Torus Tr = TheTorSurf.Torus();
                Convert_TorusToBSplineSurface Convert = new(Tr);
                TheSurface = BSplineSurfaceBuilder(Convert);
            }

            else if (S is Geom_SurfaceOfRevolution Revol)
            {
                Geom_Curve Meridian = Revol.BasisCurve();
                Geom_BSplineCurve C = CurveToBSplineCurve(Meridian);

                int NbUPoles, NbUKnots;
                int NbVPoles;
                bool periodic = true;

                // Poles of meridian = Vpoles
                NbVPoles = C.NbPoles();
                NCollection_Array1<gp_Pnt> Poles = new(C.InternalPoles());
                NCollection_Array1<double> Weights = C.InternalWeightsArray();

                double AlfaU;
                NbUKnots = 4;
                AlfaU = M_PI / 3.0;
                NbUPoles = 6;

                // Compute Knots and Mults
                NCollection_Array1<double> UKnots = new(1, NbUKnots);
                NCollection_Array1<int> UMults = new(1, NbUKnots);
                int i, j;
                for (i = 1; i <= NbUKnots; i++)
                {
                    UKnots[i] = UFirst + (i - 1) * 2 * AlfaU;
                    UMults[i] = 2;
                }

                NCollection_Array1<double> VKnots = C.InternalKnots();
                NCollection_Array1<int> VMults = C.InternalMultiplicities();

                // Compute the poles.
                NCollection_Array2<gp_Pnt> NewPoles = new(1, NbUPoles, 1, NbVPoles);
                NCollection_Array2<double> NewWeights = new(1, NbUPoles, 1, NbVPoles);
                gp_Trsf Trsf = new();

                for (i = 1; i <= NbUPoles; i += 2)
                {
                    Trsf.SetRotation(Revol.Axis(), UFirst + (i - 1) * AlfaU);
                    for (j = 1; j <= NbVPoles; j++)
                    {
                        NewPoles[i, j] = Poles[j].Transformed(Trsf);
                        NewWeights[i, j] = Weights[j];
                    }
                }

                gp_GTrsf Aff = new();
                Aff.SetAffinity(Revol.Axis(), 1 / Math.Cos(AlfaU));
                gp_XYZ coord;
                for (j = 1; j <= NbVPoles; j++)
                {
                    coord = Poles[j].XYZ();
                    Aff.Transforms(ref coord);
                    Poles[j].SetXYZ(coord);
                }

                for (i = 2; i <= NbUPoles; i += 2)
                {
                    Trsf.SetRotation(Revol.Axis(), UFirst + (i - 1) * AlfaU);
                    for (j = 1; j <= NbVPoles; j++)
                    {
                        NewPoles[i, j] = Poles[j].Transformed(Trsf);
                        NewWeights[i, j] = Weights[j] * Math.Cos(AlfaU);
                    }
                }

                TheSurface = new Geom_BSplineSurface(NewPoles,
                                                     NewWeights,
                                                     UKnots,
                                                     VKnots,
                                                     UMults,
                                                     VMults,
                                                     2,
                                                     C.Degree(),
                                                     periodic,
                                                     C.IsPeriodic());
            }

            else if (S is Geom_BezierSurface SBez)
            {
                int UDegree = SBez.UDegree();
                int VDegree = SBez.VDegree();
                NCollection_Array1<double> UKnots = new(1, 2);
                NCollection_Array1<int> UMults = new(1, 2);
                NCollection_Array1<double> VKnots = new(1, 2);
                NCollection_Array1<int> VMults = new(1, 2);
                UKnots[1] = 0.0;
                UKnots[2] = 1.0;
                UMults[1] = UDegree + 1;
                UMults[2] = UDegree + 1;
                VKnots[1] = 0.0;
                VKnots[2] = 1.0;
                VMults[1] = VDegree + 1;
                VMults[2] = VDegree + 1;
                NCollection_Array2<gp_Pnt> aPoles = SBez.InternalPoles();
                if (SBez.IsURational() || SBez.IsVRational())
                {
                    TheSurface = new Geom_BSplineSurface(aPoles,
                                                         SBez.InternalWeightsArray(),
                                                         UKnots,
                                                         VKnots,
                                                         UMults,
                                                         VMults,
                                                         UDegree,
                                                         VDegree);
                }
                else
                {
                    TheSurface = new Geom_BSplineSurface(aPoles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);
                }
            }

            else if (S is Geom_BSplineSurface)
            {
                TheSurface = (Geom_BSplineSurface)S.Copy(); // Just a copy
            }

            else
            { // In other cases => Approx
                double Tol3d = 1.0e-4;
                int MaxDegree = 14, MaxSeg;
                GeomAbs_Shape ucont = GeomAbs_C0, vcont = GeomAbs_C0;
                GeomAdaptor_Surface AS = new(Sr);
                //
                if (Sr.IsCNu(2))
                {
                    ucont = GeomAbs_C2;
                }
                else if (Sr.IsCNu(1))
                {
                    ucont = GeomAbs_C1;
                }

                //
                if (Sr.IsCNv(2))
                {
                    vcont = GeomAbs_C2;
                }
                else if (Sr.IsCNv(1))
                {
                    vcont = GeomAbs_C1;
                }

                //
                MaxSeg = 4 * (AS.NbUIntervals(GeomAbs_CN) + 1) * (AS.NbVIntervals(GeomAbs_CN) + 1);
                GeomConvert_ApproxSurface BSpS = new(Sr, Tol3d, ucont, vcont, MaxDegree, MaxDegree, MaxSeg, 1);
                // The approximation's surface is frozen (decision 197): the caller gets a copy it owns
                TheSurface = (Geom_BSplineSurface)BSpS.Surface()!.Copy();
            }
        } // End of direct case

        return TheSurface;
    }
}
