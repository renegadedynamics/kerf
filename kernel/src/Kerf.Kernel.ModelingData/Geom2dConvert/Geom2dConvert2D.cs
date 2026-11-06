// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Conversion;

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
internal static class Geom2dConvert2D
{
    private static BSplineCurve2D BSplineCurveBuilder(ConicCurve2D theConic, ConicToBSplineCurve convert)
    {
        BSplineCurve2D theCurve;
        theCurve = new BSplineCurve2D(convert.Poles(),
                                           convert.Weights(),
                                           convert.Knots(),
                                           convert.Multiplicities(),
                                           convert.Degree(),
                                           convert.IsPeriodic);

        Frame2D axis = theConic.Position;
        if ((axis.XDirection ^ axis.YDirection) < 0.0)
        {
            // Then the axis is left-handed, apply a symmetry to the curve.
            Transform2D sym = new Transform2D().WithMirror(Constants.OX2d());
            theCurve.Transform(sym);
        }

        Transform2D t = new Transform2D().WithTransformation(theConic.XAxis, Constants.OX2d());
        BSplineCurve2D cres = (BSplineCurve2D)theCurve.Transformed(t);
        return cres;
    }

    /// <summary>
    /// The part of C between its knots FromK1 and ToK2. For a non-periodic curve, it is reversed if FromK1 &gt; ToK2;
    /// for a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="DomainErrorException">FromK1 = ToK2.</exception>
    /// <exception cref="OutOfRangeException">A knot index is outside [FirstUKnotIndex, LastUKnotIndex].</exception>
    public static BSplineCurve2D SplitBSplineCurve(BSplineCurve2D c, int fromK1, int toK2,
                                                        bool sameOrientation = true)
    {
        int theFirst = c.FirstUKnotIndex();
        int theLast = c.LastUKnotIndex();
        if (fromK1 == toK2)
        {
            throw new DomainErrorException();
        }

        int firstK = Min(fromK1, toK2);
        int lastK = Max(fromK1, toK2);
        if (firstK < theFirst || lastK > theLast)
        {
            throw new OutOfRangeException();
        }

        BSplineCurve2D newCurve = (BSplineCurve2D)c.Copy();

        newCurve.Segment(c.Knot(firstK), c.Knot(lastK));

        if (c.IsPeriodic)
        {
            if (!sameOrientation)
            {
                newCurve.Reverse();
            }
        }
        else
        {
            if (fromK1 > toK2)
            {
                newCurve.Reverse();
            }
        }

        return newCurve;
    }

    /// <summary>
    /// The part of C between the parameters FromU1 and ToU2. For a non-periodic curve, it is reversed if
    /// FromU1 &gt; ToU2; for a periodic one, if SameOrientation is false. ParametricTolerance is not used.
    /// </summary>
    public static BSplineCurve2D SplitBSplineCurve(BSplineCurve2D c, double fromU1, double toU2,
                                                        double parametricTolerance, bool sameOrientation = true)
    {
        double firstU = Min(fromU1, toU2);
        double lastU = Max(fromU1, toU2);

        BSplineCurve2D c1 = (BSplineCurve2D)c.Copy();

        c1.Segment(firstU, lastU);

        if (c.IsPeriodic)
        {
            if (!sameOrientation)
            {
                c1.Reverse();
            }
        }
        else
        {
            if (fromU1 > toU2)
            {
                c1.Reverse();
            }
        }

        return c1;
    }

    /// <summary>
    /// C as a B-spline curve: lines (trimmed), conics, Bezier and B-spline curves exactly (Parameterisation chooses
    /// how circles and ellipses are parametrised), offset curves by approximation (tolerance 1e-4, C2, at most 16
    /// segments of degree 14). A whole circle or ellipse gives a periodic B-spline.
    /// </summary>
    /// <exception cref="DomainErrorException">C is of another kind (an untrimmed line, hyperbola or parabola).</exception>
    /// <exception cref="ConstructionErrorException">The approximation of an offset curve fails.</exception>
    public static BSplineCurve2D CurveToBSplineCurve(Curve2D c,
                                                          ParameterisationType parameterisation = TgtThetaOver2)
    {
        BSplineCurve2D theCurve;

        if (c is TrimmedCurve2D ctrim)
        {
            Curve2D curv;
            curv = ctrim.BasisCurve;
            double u1 = ctrim.FirstParameter;
            double u2 = ctrim.LastParameter;

            // If the curve is not truly restricted, there is no risk of
            // a Raise in BS->Segment.
            if (!curv.IsPeriodic)
            {
                if (u1 < curv.FirstParameter)
                {
                    u1 = curv.FirstParameter;
                }

                if (u2 > curv.LastParameter)
                {
                    u2 = curv.LastParameter;
                }
            }

            if (curv is LineCurve2D)
            {
                Point2D pdeb = ctrim.StartPoint;
                Point2D pfin = ctrim.EndPoint;
                NCollection_Array1<Point2D> poles = new(1, 2);
                poles[1] = pdeb;
                poles[2] = pfin;
                NCollection_Array1<double> knots = new(1, 2);
                knots[1] = ctrim.FirstParameter;
                knots[2] = ctrim.LastParameter;
                NCollection_Array1<int> mults = new(1, 2);
                mults[1] = 2;
                mults[2] = 2;
                int degree = 1;
                theCurve = new BSplineCurve2D(poles, knots, mults, degree);
            }

            else if (curv is CircleCurve2D theCircle)
            {
                Circle2D c2d = new(Constants.OX2d(), theCircle.Radius);
                if (parameterisation != RationalC1)
                {
                    CircleToBSplineCurve convert = new(c2d, u1, u2, parameterisation);
                    theCurve = BSplineCurveBuilder(theCircle, convert);
                }
                else
                {
                    if (u2 - u1 < 6.0)
                    {
                        CircleToBSplineCurve convert = new(c2d, u1, u2, parameterisation);
                        theCurve = BSplineCurveBuilder(theCircle, convert);
                    }
                    else
                    { // split circle to avoide numerical
                      // overflow when U2 - U1 =~ 2*PI

                        double umed = (u1 + u2) * .5;
                        CircleToBSplineCurve convert1 = new(c2d, u1, umed, parameterisation);

                        BSplineCurve2D theCurve1 = BSplineCurveBuilder(theCircle, convert1);

                        CircleToBSplineCurve convert2 = new(c2d, umed, u2, parameterisation);

                        BSplineCurve2D theCurve2 = BSplineCurveBuilder(theCircle, convert2);

                        CompCurveToBSplineCurve2D cctbSpl = new(theCurve1, parameterisation);

                        cctbSpl.Add(theCurve2, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

                        theCurve = cctbSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (curv is EllipseCurve2D theEllipse)
            {
                Ellipse2D e2d = new(Constants.OX2d(), theEllipse.MajorRadius, theEllipse.MinorRadius);
                if (parameterisation != RationalC1)
                {
                    EllipseToBSplineCurve convert = new(e2d, u1, u2, parameterisation);
                    theCurve = BSplineCurveBuilder(theEllipse, convert);
                }
                else
                {
                    if (u2 - u1 < 6.0)
                    {
                        EllipseToBSplineCurve convert = new(e2d, u1, u2, parameterisation);
                        theCurve = BSplineCurveBuilder(theEllipse, convert);
                    }
                    else
                    { // split ellipse to avoide numerical
                      // overflow when U2 - U1 =~ 2*PI

                        double umed = (u1 + u2) * .5;
                        EllipseToBSplineCurve convert1 = new(e2d, u1, umed, parameterisation);

                        BSplineCurve2D theCurve1 = BSplineCurveBuilder(theEllipse, convert1);

                        EllipseToBSplineCurve convert2 = new(e2d, umed, u2, parameterisation);

                        BSplineCurve2D theCurve2 = BSplineCurveBuilder(theEllipse, convert2);

                        CompCurveToBSplineCurve2D cctbSpl = new(theCurve1, parameterisation);

                        cctbSpl.Add(theCurve2, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

                        theCurve = cctbSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (curv is HyperbolaCurve2D theHyperbola)
            {
                Hyperbola2D h2d = new(Constants.OX2d(), theHyperbola.MajorRadius, theHyperbola.MinorRadius);
                HyperbolaToBSplineCurve convert = new(h2d, u1, u2);
                theCurve = BSplineCurveBuilder(theHyperbola, convert);
            }

            else if (curv is ParabolaCurve2D theParabola)
            {
                Parabola2D prb2d = new(Constants.OX2d(), theParabola.Focal);
                ParabolaToBSplineCurve convert = new(prb2d, u1, u2);
                theCurve = BSplineCurveBuilder(theParabola, convert);
            }

            else if (curv is BezierCurve2D)
            {
                BezierCurve2D cBez = (BezierCurve2D)curv.Copy();

                cBez.Segment(u1, u2);
                int degree = cBez.Degree;
                NCollection_Array1<double> knots = new(1, 2);
                NCollection_Array1<int> mults = new(1, 2);
                knots[1] = 0.0;
                knots[2] = 1.0;
                mults[1] = degree + 1;
                mults[2] = degree + 1;
                NCollection_Array1<Point2D> poles = cBez.InternalPoles();
                if (cBez.IsRational)
                {
                    theCurve = new BSplineCurve2D(poles, cBez.InternalWeightsArray(), knots, mults, degree);
                }
                else
                {
                    theCurve = new BSplineCurve2D(poles, knots, mults, degree);
                }
            }

            else if (curv is BSplineCurve2D)
            {
                theCurve = (BSplineCurve2D)curv.Copy();
                theCurve.Segment(u1, u2);
            }

            else if (curv is OffsetCurve2D)
            {
                double tol2d = 1.0e-4;
                Continuity order = C2;
                int maxSegments = 16, maxDegree = 14;
                ApproxCurve2D apprCOffs = new(c, tol2d, order, maxSegments, maxDegree);
                if (apprCOffs.HasResult)
                {
                    theCurve = apprCOffs.InternalCurve()!;
                }
                else
                {
                    throw new ConstructionErrorException();
                }
            }

            else
            {
                throw new DomainErrorException("No such curve");
            }
        }

        else
        {
            if (c is EllipseCurve2D theEllipse)
            {
                Ellipse2D e2d = new(Constants.OX2d(), theEllipse.MajorRadius, theEllipse.MinorRadius);
                EllipseToBSplineCurve convert = new(e2d, parameterisation);
                theCurve = BSplineCurveBuilder(theEllipse, convert);
                theCurve.SetPeriodic();
            }

            else if (c is CircleCurve2D theCircle)
            {
                Circle2D c2d = new(Constants.OX2d(), theCircle.Radius);
                CircleToBSplineCurve convert = new(c2d, parameterisation);
                theCurve = BSplineCurveBuilder(theCircle, convert);
                theCurve.SetPeriodic();
            }

            else if (c is BezierCurve2D cBez)
            {
                int degree = cBez.Degree;
                NCollection_Array1<double> knots = new(1, 2);
                NCollection_Array1<int> mults = new(1, 2);
                knots[1] = 0.0;
                knots[2] = 1.0;
                mults[1] = degree + 1;
                mults[2] = degree + 1;
                NCollection_Array1<Point2D> poles = cBez.InternalPoles();
                if (cBez.IsRational)
                {
                    theCurve = new BSplineCurve2D(poles, cBez.InternalWeightsArray(), knots, mults, degree);
                }
                else
                {
                    theCurve = new BSplineCurve2D(poles, knots, mults, degree);
                }
            }
            else if (c is BSplineCurve2D)
            {
                theCurve = (BSplineCurve2D)c.Copy();
            }

            else if (c is OffsetCurve2D)
            {
                double tol2d = 1.0e-4;
                Continuity order = C2;
                int maxSegments = 16, maxDegree = 14;
                ApproxCurve2D apprCOffs = new(c, tol2d, order, maxSegments, maxDegree);
                if (apprCOffs.HasResult)
                {
                    theCurve = apprCOffs.InternalCurve()!;
                }
                else
                {
                    throw new ConstructionErrorException();
                }
            }

            else
            {
                throw new DomainErrorException();
            }
        }

        return theCurve;
    }

    private sealed class Geom2dConvert_law_evaluator : EvaluatorFunction
    {
        private readonly BSplineCurve2D? _ancore;

        public Geom2dConvert_law_evaluator(BSplineCurve2D? ancore) => _ancore = ancore;

        public override void Evaluate(int derivativeRequest, ReadOnlySpan<double> startEnd, double parameter,
                                      out double result, out int errorCode)
        {
            errorCode = 0;
            if (_ancore is not null && parameter >= startEnd[0] && parameter <= startEnd[1]
                && derivativeRequest == 0)
            {
                _ancore.D0(parameter, out Point2D point);
                result = point.Coord(2);
            }
            else
            {
                // OCCT leaves the result unset; the port sets 0 (decision 32)
                result = 0.0;
                errorCode = 1;
            }
        }
    }

    private static BSplineCurve2D MultNumandDenom(BSplineCurve2D a, BSplineCurve2D bs)
    {
        BSplineCurve2D res;
        double start_value, end_value;
        int resNbPoles, degree, ii, jj, status;

        NCollection_Array1<double> bsKnots = bs.InternalKnots();
        NCollection_Array1<int> bsMults = bs.InternalMultiplicities();
        NCollection_Array1<Point2D> bsPoles = new(bs.InternalPoles());
        NCollection_Array1<double> bsWeights = bs.InternalWeightsArray();
        NCollection_Array1<double> bsFlatKnots = bs.InternalKnotSequence();
        start_value = bsKnots[1];
        end_value = bsKnots[bs.KnotCount];
        double tolerance = 10.0 * Epsilon(Math.Abs(end_value));

        NCollection_Array1<double> knots = new(a.InternalKnots());
        NCollection_Array1<Point2D> poles = a.InternalPoles();
        NCollection_Array1<int> mults = a.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(bs.FirstParameter, bs.LastParameter, knots);
        BSplineCurve2D ancore = new(poles, knots, mults, a.Degree);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.MergeBSplineKnots(tolerance,
                                   start_value,
                                   end_value,
                                   a.Degree,
                                   knots,
                                   mults,
                                   bs.Degree,
                                   bsKnots,
                                   bsMults,
                                   out resNbPoles,
                                   out NCollection_HArray1<double> resKnots,
                                   out NCollection_HArray1<int> resMults);
        degree = bs.Degree + a.Degree;
        NCollection_Array1<Point2D> resNumPoles = new(1, resNbPoles);
        NCollection_Array1<double> resDenPoles = new(1, resNbPoles);
        NCollection_Array1<Point2D> resPoles = new(1, resNbPoles);
        NCollection_Array1<double> resFlatKnots = new(1, resNbPoles + degree + 1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(resKnots, resMults, resFlatKnots);
        for (ii = 1; ii <= bs.PoleCount; ii++)
        {
            for (jj = 1; jj <= 2; jj++)
            {
                bsPoles[ii] = bsPoles[ii].WithCoord(jj, bsPoles[ii].Coord(jj) * bsWeights[ii]);
            }
        }

        // POP for NT
        Geom2dConvert_law_evaluator ev = new(ancore);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(ev, bs.Degree, bsFlatKnots, bsPoles, resFlatKnots, degree, resNumPoles, out status);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(ev, bs.Degree, bsFlatKnots, bsWeights, resFlatKnots, degree, resDenPoles, out status);
        for (ii = 1; ii <= resNbPoles; ii++)
        {
            for (jj = 1; jj <= 2; jj++)
            {
                resPoles[ii] = resPoles[ii].WithCoord(jj, resNumPoles[ii].Coord(jj) / resDenPoles[ii]);
            }
        }

        res = new BSplineCurve2D(resPoles, resDenPoles, resKnots, resMults, degree);
        return res;
    }

    private static void Pretreatment(NCollection_Array1<BSplineCurve2D> tab)
    {
        int i, j;
        double a;

        for (i = 0; i <= (tab.Length() - 1); i++)
        {
            if (tab[i].IsRational)
            {
                a = tab[i].Weight(1);
                if ((tab[i].Weight(2) == a) && (tab[i].Weight(tab[i].PoleCount - 1) == a)
                    && (tab[i].Weight(tab[i].PoleCount) == a))
                {
                    // OCCT changes the caller's curve in place; the port puts a changed copy in the array
                    BSplineCurve2D curve = (BSplineCurve2D)tab[i].Copy();
                    for (j = 1; j <= curve.PoleCount; j++)
                    {
                        curve.SetWeight(j, curve.Weight(j) / a);
                    }

                    tab[i] = curve;
                }
            }
        }
    }

    private static bool NeedToBeTreated(BSplineCurve2D bs)
    {
        if (bs.IsRational)
        {
            NCollection_Array1<double> tabWeights = bs.InternalWeightsArray();
            return (global::Kerf.Engine.Geometry.Splines.BSplCLib.IsRational(tabWeights, 1, bs.PoleCount))
                   && ((bs.Weight(1) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(1) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(2) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(2) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(bs.PoleCount - 1) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(bs.PoleCount - 1) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(bs.PoleCount) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                       || (bs.Weight(bs.PoleCount) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion())));
        }
        else
        {
            return false;
        }
    }

    private static bool Need2DegRepara(NCollection_Array1<BSplineCurve2D> tab)
    {
        int i;
        Vector2D vec1, vec2;
        double rapport = 1.0e0;

        for (i = 0; i <= tab.Length() - 2; i++)
        {
            tab[i + 1].D1(tab[i + 1].FirstParameter, out _, out vec1);
            tab[i].D1(tab[i].LastParameter, out _, out vec2);
            rapport = rapport * vec2.Magnitude / vec1.Magnitude;
        }

        return (rapport > (1.0e0 + global::Kerf.Engine.Foundation.Precision.Confusion())) || (rapport < (1.0e0 - global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    private static int Indexmin(NCollection_Array1<BSplineCurve2D> tab)
    {
        int i, index = 0, degree;

        degree = tab[0].Degree;
        for (i = 0; i <= tab.Length() - 1; i++)
        {
            if (tab[i].Degree <= degree)
            {
                degree = tab[i].Degree;
                index = i;
            }
        }

        return index;
    }

    private static void ReorderArrayOfG1(NCollection_Array1<BSplineCurve2D> arrayOfCurves,
                                         NCollection_Array1<double> arrayOfToler,
                                         NCollection_Array1<bool> tabG1,
                                         int startIndex,
                                         double closedTolerance)
    {
        int i;
        NCollection_Array1<BSplineCurve2D> arraybisOfCurves = new(0, arrayOfCurves.Length() - 1);
        NCollection_Array1<double> arraybisOfToler = new(0, arrayOfToler.Length() - 1);
        NCollection_Array1<bool> tabbisG1 = new(0, tabG1.Length() - 1);

        for (i = 0; i <= arrayOfCurves.Length() - 1; i++)
        {
            if (i != arrayOfCurves.Length() - 1)
            {
                arraybisOfCurves[i] = arrayOfCurves[i];
                arraybisOfToler[i] = arrayOfToler[i];
                tabbisG1[i] = tabG1[i];
            }
            else
            {
                arraybisOfCurves[i] = arrayOfCurves[i];
            }
        }

        for (i = 0; i <= (arrayOfCurves.Length() - (startIndex + 2)); i++)
        {
            arrayOfCurves[i] = arraybisOfCurves[i + startIndex + 1];
            if (i != (arrayOfCurves.Length() - (startIndex + 2)))
            {
                arrayOfToler[i] = arraybisOfToler[i + startIndex + 1];
                tabG1[i] = tabbisG1[i + startIndex + 1];
            }
        }

        arrayOfToler[arrayOfCurves.Length() - (startIndex + 2)] = closedTolerance;
        tabG1[arrayOfCurves.Length() - (startIndex + 2)] = true;

        for (i = (arrayOfCurves.Length() - (startIndex + 1)); i <= (arrayOfCurves.Length() - 1); i++)
        {
            if (i != arrayOfCurves.Length() - 1)
            {
                arrayOfCurves[i] = arraybisOfCurves[i - (arrayOfCurves.Length() - (startIndex + 1))];
                arrayOfToler[i] = arraybisOfToler[i - (arrayOfCurves.Length() - (startIndex + 1))];
                tabG1[i] = tabbisG1[i - (arrayOfCurves.Length() - (startIndex + 1))];
            }
            else
            {
                arrayOfCurves[i] = arraybisOfCurves[i - (arrayOfCurves.Length() - (startIndex + 1))];
            }
        }
    }

    private static int GeomAbsToInteger(Continuity gcont)
    {
        int cont = 0;
        switch (gcont)
        {
            case C0:
                cont = 0;
                break;
            case G1:
                cont = 1;
                break;
            case C1:
                cont = 2;
                break;
            case G2:
                cont = 3;
                break;
            case C2:
                cont = 4;
                break;
            case C3:
                cont = 5;
                break;
            case CN:
                cont = 6;
                break;
        }

        return cont;
    }

    private static Continuity Continuity(Curve2D c1,
                                            Curve2D c2,
                                            double u1,
                                            double u2,
                                            bool r1,
                                            bool r2,
                                            double tl,
                                            double ta)
    {
        Continuity cont = C0;
        int index1, index2;
        double tolerance, value;
        Vector2D d1, d2;
        Point2D point1, point2;
        int cont1, cont2;
        Continuity gcont1 = c1.Continuity, gcont2 = c2.Continuity;
        cont1 = GeomAbsToInteger(gcont1);
        cont2 = GeomAbsToInteger(gcont2);

        Curve2D curve1 = c1;
        Curve2D curve2 = c2;
        if (c1 is TrimmedCurve2D trimmed1)
        {
            curve1 = trimmed1.BasisCurve;
        }

        if (c2 is TrimmedCurve2D trimmed2)
        {
            curve2 = trimmed2.BasisCurve;
        }

        if (curve1 is BSplineCurve2D bSplineCurve1)
        {
            bSplineCurve1.Resolution(tl, out tolerance);
            bSplineCurve1.LocateU(u1, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < bSplineCurve1.KnotCount && index1 == index2)
            {
                cont1 = bSplineCurve1.Degree - bSplineCurve1.Multiplicity(index1);
            }
            else
            {
                cont1 = 5;
            }
        }

        if (curve2 is BSplineCurve2D bSplineCurve2)
        {
            bSplineCurve2.Resolution(tl, out tolerance);
            bSplineCurve2.LocateU(u2, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < bSplineCurve2.KnotCount && index1 == index2)
            {
                cont2 = bSplineCurve2.Degree - bSplineCurve2.Multiplicity(index1);
            }
            else
            {
                cont2 = 5;
            }
        }

        curve1.D1(u1, out point1, out d1);
        curve2.D1(u2, out point2, out d2);
        if (point1.SquareDistanceTo(point2) <= tl * tl)
        {
            if (cont1 != 0 && cont2 != 0)
            {
                if (d1.SquareMagnitude >= tl * tl && d2.SquareMagnitude >= tl * tl)
                {
                    if (r1)
                    {
                        d1 = d1.WithCoord(-d1.X, -d1.Y);
                    }

                    if (r2)
                    {
                        d2 = d2.WithCoord(-d2.X, -d2.Y);
                    }

                    value = d1.Dot(d2);
                    if ((d1.Magnitude <= (d2.Magnitude + tl)) && (d1.Magnitude >= (d2.Magnitude - tl))
                        && (value / (d1.Magnitude * d2.Magnitude) >= 1.0e0 - ta * ta))
                    {
                        cont = global::Kerf.Geometry.Continuity.C1;
                    }
                    else
                    {
                        d1 = d1.Normalized();
                        d2 = d2.Normalized();
                        value = Math.Abs(d1.Dot(d2));
                        if (value >= 1.0e0 - ta * ta)
                        {
                            cont = G1;
                        }
                    }
                }
            }
        }
        else
        {
            throw new FailureException("Courbes non jointives");
        }

        return cont;
    }

    private static Continuity Continuity(Curve2D c1, Curve2D c2, double u1, double u2, bool r1, bool r2)
    {
        return Continuity(c1, c2, u1, u2, r1, r2, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    private sealed class Geom2dConvert_reparameterise_evaluator : EvaluatorFunction
    {
        private readonly double[] _polynomialCoefficient = new double[3];

        public Geom2dConvert_reparameterise_evaluator(ReadOnlySpan<double> polynomialCoefficient) =>
            polynomialCoefficient[..3].CopyTo(_polynomialCoefficient);

        public override void Evaluate(int derivativeRequest, ReadOnlySpan<double> startEnd, double parameter,
                                      out double result, out int errorCode)
        {
            errorCode = 0;
            // OCCT passes a single double as the result, which PLib::EvalPolynomial fills with the value and
            // theDerivativeRequest derivatives: it writes past it when a derivative is requested (BSplCLib's
            // FunctionReparameterise only asks for the value). The port evaluates into a buffer of that size and
            // returns its first value, as OCCT's double holds.
            Span<double> results = stackalloc double[derivativeRequest + 1];
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(parameter, derivativeRequest, 2, 1, _polynomialCoefficient, results);
            result = results[0];
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
    /// <exception cref="FailureException">Two consecutive curves don't join.</exception>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void ConcatG1(NCollection_Array1<BSplineCurve2D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<BSplineCurve2D> arrayOfConcatenated,
                                ref bool closedFlag,
                                double closedTolerance)
    {
        int nb_curve = arrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            first, preLast = 0;
        Vector2D vec1, vec2; // consecutive tangent vectors
        BSplineCurve2D curve1, curve2;
        NCollection_Array1<bool> tabG1 = new(0, nb_curve - 2); // G1 continuity table at junctions
        NCollection_Array1<double> local_tolerance = new(0, arrayOfToler.Length() - 1);

        for (i = 0; i < arrayOfToler.Length(); i++)
        {
            local_tolerance[i] = arrayOfToler[i];
        }

        for (i = 0; i < nb_curve; i++)
        {
            if (i >= 1)
            {
                first = arrayOfCurves[i].FirstParameter;
                if (Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true) < C0)
                {
                    throw new ConstructionErrorException("Geom2dConvert curves not C0"); // renvoi d'une erreur
                }
                else
                {
                    if (Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true) >= G1)
                    {
                        tabG1[i - 1] = true; // True=Continuite G1
                    }
                    else
                    {
                        tabG1[i - 1] = false;
                    }
                }
            }

            preLast = arrayOfCurves[i].LastParameter;
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

        if ((closedFlag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1(arrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, closedTolerance);
        }

        arrayOfConcatenated = new NCollection_HArray1<BSplineCurve2D>(0, nb_group - 1);

        bool fusion;
        index = 0;
        Pretreatment(arrayOfCurves);

        Span<double> polynomialCoefficient = stackalloc double[3];

        bool needDoubleDegRepara = Need2DegRepara(arrayOfCurves);
        if (nb_group == 1 && closedFlag && needDoubleDegRepara)
        {
            curve1 = arrayOfCurves[nb_curve - 1];
            if (curve1.Degree > BSplineCurve2D.MaxDegree / 2)
            {
                closedFlag = false;
            }
        }

        if ((nb_group == 1) && (closedFlag))
        { // treatment of a particular case
            indexmin = Indexmin(arrayOfCurves);
            if (indexmin != (arrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1(arrayOfCurves, local_tolerance, tabG1, indexmin, closedTolerance);
            }

            curve2 = arrayOfCurves[0];
            for (j = 1; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                curve1 = arrayOfCurves[j];
                if ((j == (nb_curve - 1)) && (needDoubleDegRepara))
                {
                    int newCurveDegree = 2 * curve1.Degree;
                    curve2.D1(curve2.LastParameter, out _, out vec1);
                    curve1.D1(curve1.FirstParameter, out _, out vec2);
                    lambda = vec2.Magnitude / vec1.Magnitude;
                    NCollection_Array1<double> knotC1 = new(curve1.InternalKnots());
                    curve1.D1(curve1.LastParameter, out _, out vec2);
                    arrayOfCurves[0].D1(arrayOfCurves[0].FirstParameter, out _, out vec1);
                    double lambda2 = vec1.Magnitude / vec2.Magnitude;
                    double tmax, a, b, c, umin = curve1.FirstParameter, umax = curve1.LastParameter;
                    tmax = 2 * lambda * (umax - umin) / (1 + lambda * lambda2);
                    a = (lambda * lambda2 - 1) / (2 * lambda * tmax);
                    polynomialCoefficient[2] = a;
                    b = (1 / lambda);
                    polynomialCoefficient[1] = b;
                    c = umin;
                    polynomialCoefficient[0] = c;
                    NCollection_Array1<double> curve1FlatKnots = new(1, curve1.PoleCount + curve1.Degree + 1);
                    NCollection_Array1<int> knotC1Mults = new(curve1.InternalMultiplicities());
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, curve1FlatKnots);
                    knotC1[1] = 0.0;
                    for (ii = 2; ii <= knotC1.Length(); ii++)
                    {
                        knotC1[ii] = (-b + Math.Sqrt(b * b - 4 * a * (c - knotC1[ii]))) / (2 * a); // ifv 17.05.00 buc60667
                    }

                    NCollection_Array1<Point2D> curve1Poles = new(curve1.InternalPoles());

                    for (ii = 1; ii <= curve1.KnotCount; ii++)
                    {
                        knotC1Mults[ii] = (curve1.Degree + knotC1Mults[ii]);
                    }

                    NCollection_Array1<double> flatKnots = new(1, curve1FlatKnots.Length() + (curve1.Degree * curve1.KnotCount));

                    global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, flatKnots);
                    NCollection_Array1<Point2D> newPoles = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                    int status;
                    NCollection_Array1<double> curve1Weights = curve1.InternalWeightsArray();
                    for (ii = 1; ii <= curve1.PoleCount; ii++)
                    {
                        for (jj = 1; jj <= 2; jj++)
                        {
                            curve1Poles[ii] = curve1Poles[ii].WithCoord(jj, curve1Poles[ii].Coord(jj) * curve1Weights[ii]);
                        }
                    }

                    // POP for NT
                    Geom2dConvert_reparameterise_evaluator ev = new(polynomialCoefficient);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Poles, flatKnots,
                                                    newCurveDegree, newPoles, out status);
                    NCollection_Array1<double> newWeights = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Weights, flatKnots,
                                                    newCurveDegree, newWeights, out status);
                    for (ii = 1; ii <= newPoles.Length(); ii++)
                    {
                        for (jj = 1; jj <= 2; jj++)
                        {
                            newPoles[ii] = newPoles[ii].WithCoord(jj, newPoles[ii].Coord(jj) / newWeights[ii]);
                        }
                    }

                    curve1 = new BSplineCurve2D(newPoles, newWeights, knotC1, knotC1Mults, newCurveDegree);
                }

                CompCurveToBSplineCurve2D C = new(curve2);
                fusion = C.Add(curve1, local_tolerance[j - 1]); // merge two adjacent curves
                if (!fusion)
                {
                    throw new ConstructionErrorException("Geom2dConvert Concatenation Error");
                }

                curve2 = C.InternalBSplineCurve()!;
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                curve2 = (BSplineCurve2D)curve2.Copy();
            }

            curve2.SetPeriodic(); // single C1 curve
            curve2.RemoveKnot(curve2.LastUKnotIndex(), curve2.Multiplicity(curve2.LastUKnotIndex()) - 1,
                              global::Kerf.Engine.Foundation.Precision.Confusion());
            arrayOfConcatenated.SetValue(0, curve2);
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
                    curve1 = arrayOfCurves[j];

                    if (index == j)
                    { // initialisation en debut de groupe
                        arrayOfConcatenated.SetValue(i, curve1);
                    }
                    else
                    {
                        CompCurveToBSplineCurve2D C = new(arrayOfConcatenated.Value(i));
                        fusion = C.Add(curve1, local_tolerance[j - 1], true); // fusion de deux courbes adjacentes

                        if (!fusion)
                        {
                            throw new ConstructionErrorException("Geom2dConvert Concatenation Error");
                        }

                        arrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// <see cref="ConcatC1(NCollection_Array1{BSplineCurve2D}, NCollection_Array1{double}, out NCollection_HArray1{int}, out NCollection_HArray1{BSplineCurve2D}, ref bool, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void ConcatC1(NCollection_Array1<BSplineCurve2D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<int> arrayOfIndices,
                                out NCollection_HArray1<BSplineCurve2D> arrayOfConcatenated,
                                ref bool closedFlag,
                                double closedTolerance)
    {
        ConcatC1(arrayOfCurves, arrayOfToler, out arrayOfIndices, out arrayOfConcatenated, ref closedFlag,
                 closedTolerance, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    /// <summary>
    /// As <see cref="ConcatG1"/>, but the curves are made C1 where they join with G1 continuity (rational curves
    /// are reparametrised first), and AngularTolerance is the tolerance of the G1 test. ArrayOfIndices (from 0) holds
    /// the indices in ArrayOfCurves of the first curve of each concatenated group, and a last index.
    /// </summary>
    /// <exception cref="FailureException">Two consecutive curves don't join.</exception>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void ConcatC1(NCollection_Array1<BSplineCurve2D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<int> arrayOfIndices,
                                out NCollection_HArray1<BSplineCurve2D> arrayOfConcatenated,
                                ref bool closedFlag,
                                double closedTolerance,
                                double angularTolerance)
    {
        int nb_curve = arrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            first, preLast = 0;
        Vector2D vec1, vec2; // consecutive tangent vectors
        BSplineCurve2D curve1, curve2 = null!;
        NCollection_Array1<bool> tabG1 = new(0, nb_curve - 2); // G1 continuity table at junctions
        NCollection_Array1<double> local_tolerance = new(0, arrayOfToler.Length() - 1);

        for (i = 0; i < arrayOfToler.Length(); i++)
        {
            local_tolerance[i] = arrayOfToler[i];
        }

        for (i = 0; i < nb_curve; i++)
        {
            if (i >= 1)
            {
                first = arrayOfCurves[i].FirstParameter;
                if (Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true, arrayOfToler[i - 1],
                               angularTolerance)
                    < C0)
                {
                    throw new ConstructionErrorException("Geom2dConvert curves not C0"); // renvoi d'une erreur
                }
                else
                {
                    if (Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true,
                                   arrayOfToler[i - 1], angularTolerance)
                        >= G1)
                    {
                        tabG1[i - 1] = true; // True=Continuite G1
                    }
                    else
                    {
                        tabG1[i - 1] = false;
                    }
                }
            }

            preLast = arrayOfCurves[i].LastParameter;
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

        if ((closedFlag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1(arrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, closedTolerance);
        }

        arrayOfIndices = new NCollection_HArray1<int>(0, nb_group);
        arrayOfConcatenated = new NCollection_HArray1<BSplineCurve2D>(0, nb_group - 1);

        bool fusion;
        int k = 0;
        index = 0;
        Pretreatment(arrayOfCurves);
        Span<double> polynomialCoefficient = stackalloc double[3];

        bool needDoubleDegRepara = Need2DegRepara(arrayOfCurves);
        if (nb_group == 1 && closedFlag && needDoubleDegRepara)
        {
            curve1 = arrayOfCurves[nb_curve - 1];
            if (curve1.Degree > BSplineCurve2D.MaxDegree / 2)
            {
                closedFlag = false;
            }
        }

        if ((nb_group == 1) && (closedFlag))
        { // treatment of a particular case
            arrayOfIndices.SetValue(0, 0);
            arrayOfIndices.SetValue(1, 0);
            indexmin = Indexmin(arrayOfCurves);
            if (indexmin != (arrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1(arrayOfCurves, local_tolerance, tabG1, indexmin, closedTolerance);
            }

            for (j = 0; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                if (NeedToBeTreated(arrayOfCurves[j]))
                {
                    curve1 = MultNumandDenom(global::Kerf.Engine.Approximation.Hermit.Solution(arrayOfCurves[j]), arrayOfCurves[j]);
                }
                else
                {
                    curve1 = arrayOfCurves[j];
                }

                int newCurveDegree = 2 * curve1.Degree;

                if (j == 0)
                { // initialisation en debut de groupe
                    curve2 = curve1;
                }
                else
                {
                    if ((j == (nb_curve - 1)) && (needDoubleDegRepara))
                    {
                        curve2.D1(curve2.LastParameter, out _, out vec1);
                        curve1.D1(curve1.FirstParameter, out _, out vec2);
                        lambda = vec2.Magnitude / vec1.Magnitude;
                        NCollection_Array1<double> knotC1 = new(curve1.InternalKnots());
                        curve1.D1(curve1.LastParameter, out _, out vec2);
                        arrayOfCurves[0].D1(arrayOfCurves[0].FirstParameter, out _, out vec1);
                        double lambda2 = vec1.Magnitude / vec2.Magnitude;
                        double tmax, a, b, c, umin = curve1.FirstParameter, umax = curve1.LastParameter;
                        tmax = 2 * lambda * (umax - umin) / (1 + lambda * lambda2);
                        a = (lambda * lambda2 - 1) / (2 * lambda * tmax);
                        polynomialCoefficient[2] = a;
                        b = (1 / lambda);
                        polynomialCoefficient[1] = b;
                        c = umin;
                        polynomialCoefficient[0] = c;
                        NCollection_Array1<double> curve1FlatKnots = new(1, curve1.PoleCount + curve1.Degree + 1);
                        NCollection_Array1<int> knotC1Mults = new(curve1.InternalMultiplicities());
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, curve1FlatKnots);
                        knotC1[1] = 0.0;
                        for (ii = 2; ii <= knotC1.Length(); ii++)
                        {
                            knotC1[ii] = (-b + Math.Sqrt(b * b - 4 * a * (c - knotC1[ii]))) / (2 * a); // ifv 17.05.00 buc60667
                        }

                        NCollection_Array1<Point2D> curve1Poles = new(curve1.InternalPoles());

                        for (ii = 1; ii <= curve1.KnotCount; ii++)
                        {
                            knotC1Mults[ii] = (curve1.Degree + knotC1Mults[ii]);
                        }

                        NCollection_Array1<double> flatKnots = new(1, curve1FlatKnots.Length() + (curve1.Degree * curve1.KnotCount));

                        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, flatKnots);
                        NCollection_Array1<Point2D> newPoles = new(1, flatKnots.Length() - (newCurveDegree + 1));
                        int status;
                        NCollection_Array1<double> curve1Weights = curve1.InternalWeightsArray();
                        for (ii = 1; ii <= curve1.PoleCount; ii++)
                        {
                            for (jj = 1; jj <= 2; jj++)
                            {
                                curve1Poles[ii] = curve1Poles[ii].WithCoord(jj, curve1Poles[ii].Coord(jj) * curve1Weights[ii]);
                            }
                        }

                        // POP for NT
                        Geom2dConvert_reparameterise_evaluator ev = new(polynomialCoefficient);
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Poles, flatKnots,
                                                        newCurveDegree, newPoles, out status);
                        NCollection_Array1<double> newWeights = new(1, flatKnots.Length() - (newCurveDegree + 1));
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Weights, flatKnots,
                                                        newCurveDegree, newWeights, out status);
                        for (ii = 1; ii <= newPoles.Length(); ii++)
                        {
                            for (jj = 1; jj <= 2; jj++)
                            {
                                newPoles[ii] = newPoles[ii].WithCoord(jj, newPoles[ii].Coord(jj) / newWeights[ii]);
                            }
                        }

                        curve1 = new BSplineCurve2D(newPoles, newWeights, knotC1, knotC1Mults, newCurveDegree);
                    }

                    CompCurveToBSplineCurve2D C = new(curve2);
                    fusion = C.Add(curve1, local_tolerance[j - 1]); // merge two adjacent curves
                    if (!fusion)
                    {
                        throw new ConstructionErrorException("Geom2dConvert Concatenation Error");
                    }

                    curve2 = C.InternalBSplineCurve()!;
                }
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                curve2 = (BSplineCurve2D)curve2.Copy();
            }

            curve2.SetPeriodic(); // single C1 curve
            curve2.RemoveKnot(curve2.LastUKnotIndex(), curve2.Multiplicity(curve2.LastUKnotIndex()) - 1,
                              global::Kerf.Engine.Foundation.Precision.Confusion());
            arrayOfConcatenated.SetValue(0, curve2);
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

                if ((!closedFlag) || (nb_group == 1))
                { // Filling the array of preserved indices
                    k++;
                    arrayOfIndices.SetValue(k - 1, index);
                    if (k == nb_group)
                    {
                        arrayOfIndices.SetValue(k, 0);
                    }
                }
                else
                {
                    k++;
                    arrayOfIndices.SetValue(k - 1, index + nb_vertex_group0 + 1);
                    if (k == nb_group)
                    {
                        arrayOfIndices.SetValue(k, nb_vertex_group0 + 1);
                    }
                }

                for (j = index; j <= index + nb_vertexG1; j++)
                { // boucle secondaire a l'interieur de chaque groupe
                    if (NeedToBeTreated(arrayOfCurves[j]))
                    {
                        curve1 = MultNumandDenom(global::Kerf.Engine.Approximation.Hermit.Solution(arrayOfCurves[j]), arrayOfCurves[j]);
                    }
                    else
                    {
                        curve1 = arrayOfCurves[j];
                    }

                    if (index == j)
                    { // initialisation en debut de groupe
                        arrayOfConcatenated.SetValue(i, curve1);
                    }
                    else
                    {
                        CompCurveToBSplineCurve2D C = new(arrayOfConcatenated.Value(i));
                        fusion = C.Add(curve1, local_tolerance[j - 1], true); // fusion de deux courbes adjacentes
                        if (!fusion)
                        {
                            throw new ConstructionErrorException("Geom2dConvert Concatenation Error");
                        }

                        arrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
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
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void C0BSplineToC1BSplineCurve(ref BSplineCurve2D bs, double tolerance)
    {
        NCollection_Array1<int> bsMults = bs.InternalMultiplicities();
        NCollection_Array1<double> bsKnots = bs.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double u1, u2;
        bool closed_flag = false;
        Point2D point1, point2;
        Vector2D v1, v2;
        bool fusion;
        for (i = bs.FirstUKnotIndex() + 1; i <= (bs.LastUKnotIndex() - 1); i++)
        {
            if (bsMults[i] == bs.Degree)
            {
                nbcurveC1++;
            }
        }

        nbcurveC1 = Min(nbcurveC1, bs.KnotCount - 1);

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<BSplineCurve2D> arrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> arrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                arrayOfToler[i] = tolerance;
            }

            u2 = bs.FirstParameter;
            j = bs.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                u1 = u2;

                while (j < bs.LastUKnotIndex() && bsMults[j] < bs.Degree)
                {
                    j++;
                }

                u2 = bsKnots[j];
                j++;
                BSplineCurve2D bSbis = (BSplineCurve2D)bs.Copy();
                bSbis.Segment(u1, u2);
                arrayOfCurves[i] = bSbis;
            }

            const double angularToler = 1.0e-7;

            bs.D1(bs.FirstParameter, out point1, out v1); // a verifier
            bs.D1(bs.LastParameter, out point2, out v2);

            if ((point1.SquareDistanceTo(point2) < tolerance * tolerance) && (v1.IsParallel(v2, angularToler)))
            {
                closed_flag = true;
            }

            Geom2dConvert2D.ConcatC1(arrayOfCurves, arrayOfToler, out _, out NCollection_HArray1<BSplineCurve2D> arrayOfConcatenated,
                                   ref closed_flag, tolerance);

            CompCurveToBSplineCurve2D c = new(arrayOfConcatenated.Value(0));
            if (arrayOfConcatenated.Length() >= 2)
            {
                for (i = 1; i < arrayOfConcatenated.Length(); i++)
                {
                    fusion = c.Add(arrayOfConcatenated.Value(i), tolerance, true);
                    if (!fusion)
                    {
                        throw new ConstructionErrorException("Geom2dConvert Concatenation Error");
                    }
                }
            }

            bs = c.InternalBSplineCurve()!;
        }
    }

    /// <summary>
    /// <see cref="C0BSplineToArrayOfC1BSplineCurve(BSplineCurve2D, out NCollection_HArray1{BSplineCurve2D}, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void C0BSplineToArrayOfC1BSplineCurve(BSplineCurve2D bs,
                                                        out NCollection_HArray1<BSplineCurve2D> tabBS,
                                                        double tolerance)
    {
        C0BSplineToArrayOfC1BSplineCurve(bs, out tabBS, tolerance, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    /// <summary>
    /// Cuts BS at its C0 knots and concatenates the pieces that join with G1 continuity (within the tolerances) into
    /// C1 curves, returned in tabBS (from 0). A curve without C0 knots is returned as the single element.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void C0BSplineToArrayOfC1BSplineCurve(BSplineCurve2D bs,
                                                        out NCollection_HArray1<BSplineCurve2D> tabBS,
                                                        double angularTolerance,
                                                        double tolerance)
    {
        NCollection_Array1<int> bsMults = bs.InternalMultiplicities();
        NCollection_Array1<double> bsKnots = bs.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double u1, u2;
        bool closed_flag = false;
        Point2D point1, point2;
        Vector2D v1, v2;
        for (i = bs.FirstUKnotIndex(); i <= (bs.LastUKnotIndex() - 1); i++)
        {
            if (bsMults[i] == bs.Degree)
            {
                nbcurveC1++;
            }
        }

        nbcurveC1 = Min(nbcurveC1, bs.KnotCount - 1);

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<BSplineCurve2D> arrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> arrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                arrayOfToler[i] = tolerance;
            }

            u2 = bs.FirstParameter;
            j = bs.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                u1 = u2;
                while (j < bs.LastUKnotIndex() && bsMults[j] < bs.Degree)
                {
                    j++;
                }

                u2 = bsKnots[j];
                j++;
                BSplineCurve2D bSbis = (BSplineCurve2D)bs.Copy();
                bSbis.Segment(u1, u2);
                arrayOfCurves[i] = bSbis;
            }

            bs.D1(bs.FirstParameter, out point1, out v1);
            bs.D1(bs.LastParameter, out point2, out v2);

            // OCCT compares the squared distance with Tolerance, not its square (C0BSplineToC1BSplineCurve squares it)
            if (((point1.SquareDistanceTo(point2) < tolerance)) && (v1.IsParallel(v2, angularTolerance)))
            {
                closed_flag = true;
            }

            Geom2dConvert2D.ConcatC1(arrayOfCurves, arrayOfToler, out _, out tabBS, ref closed_flag, tolerance,
                                   angularTolerance);
        }
        else
        {
            tabBS = new NCollection_HArray1<BSplineCurve2D>(0, 0);
            tabBS.SetValue(0, bs);
        }
    }
}
