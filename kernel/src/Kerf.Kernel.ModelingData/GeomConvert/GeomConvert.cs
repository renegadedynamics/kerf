// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert.cxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Conversion;

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
internal static class GeomConvert
{
    private static BSplineCurve3D BSplineCurveBuilder(ConicCurve3D theConic, ConicToBSplineCurve convert)
    {
        BSplineCurve3D theCurve;
        NCollection_Array1<Point2D> poles2d = convert.Poles();
        NCollection_Array1<double> weights = convert.Weights();
        NCollection_Array1<double> knots = convert.Knots();
        NCollection_Array1<int> mults = convert.Multiplicities();
        NCollection_Array1<Point3D> poles = new(1, poles2d.Length());
        for (int i = poles2d.Lower(); i <= poles2d.Upper(); i++)
        {
            Point2D p2d = poles2d[i];
            poles[i] = poles[i].WithCoord(p2d.X, p2d.Y, 0.0);
        }

        theCurve = new BSplineCurve3D(poles, weights, knots, mults, convert.Degree(), convert.IsPeriodic);
        Transform3D t = new Transform3D().WithTransformation(new CoordinateSystem(theConic.Position), new CoordinateSystem(Constants.XOY()));
        BSplineCurve3D cres;
        cres = (BSplineCurve3D)theCurve.Transformed(t);
        return cres;
    }

    /// <summary>
    /// The part of C between its knots FromK1 and ToK2. For a non-periodic curve, it is reversed if FromK1 &gt; ToK2;
    /// for a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="DomainErrorException">FromK1 = ToK2.</exception>
    /// <exception cref="DomainErrorException">A knot index is outside [FirstUKnotIndex, LastUKnotIndex].</exception>
    public static BSplineCurve3D SplitBSplineCurve(BSplineCurve3D c, int fromK1, int toK2,
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
            throw new DomainErrorException();
        }

        BSplineCurve3D c1 = (BSplineCurve3D)c.Copy();

        c1.Segment(c.Knot(firstK), c.Knot(lastK));

        if (c.IsPeriodic)
        {
            if (!sameOrientation)
            {
                c1.Reverse();
            }
        }
        else
        {
            if (fromK1 > toK2)
            {
                c1.Reverse();
            }
        }

        return c1;
    }

    /// <summary>
    /// The part of C between the parameters FromU1 and ToU2. For a non-periodic curve, it is reversed if
    /// FromU1 &gt; ToU2; for a periodic one, if SameOrientation is false. ParametricTolerance is not used.
    /// </summary>
    public static BSplineCurve3D SplitBSplineCurve(BSplineCurve3D c, double fromU1, double toU2,
                                                        double parametricTolerance, bool sameOrientation = true)
    {
        double firstU = Min(fromU1, toU2);
        double lastU = Max(fromU1, toU2);

        BSplineCurve3D c1 = (BSplineCurve3D)c.Copy();

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
    public static BSplineCurve3D CurveToBSplineCurve(Curve3D c,
                                                          ParameterisationType parameterisation = TgtThetaOver2)
    {
        BSplineCurve3D theCurve;

        if (c is TrimmedCurve3D ctrim)
        {
            Curve3D curv;
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

            if (curv is LineCurve3D)
            {
                Point3D pdeb = ctrim.StartPoint;
                Point3D pfin = ctrim.EndPoint;
                NCollection_Array1<Point3D> poles = new(1, 2);
                poles[1] = pdeb;
                poles[2] = pfin;
                NCollection_Array1<double> knots = new(1, 2);
                knots[1] = ctrim.FirstParameter;
                knots[2] = ctrim.LastParameter;
                NCollection_Array1<int> mults = new(1, 2);
                mults[1] = 2;
                mults[2] = 2;
                int degree = 1;
                theCurve = new BSplineCurve3D(poles, knots, mults, degree);
            }

            else if (curv is CircleCurve3D theCircle)
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

                        BSplineCurve3D theCurve1 = BSplineCurveBuilder(theCircle, convert1);

                        CircleToBSplineCurve convert2 = new(c2d, umed, u2, parameterisation);

                        BSplineCurve3D theCurve2 = BSplineCurveBuilder(theCircle, convert2);

                        CompCurveToBSplineCurve cctbSpl = new(theCurve1, parameterisation);

                        cctbSpl.Add(theCurve2, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

                        theCurve = cctbSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (curv is EllipseCurve3D theEllipse)
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

                        BSplineCurve3D theCurve1 = BSplineCurveBuilder(theEllipse, convert1);

                        EllipseToBSplineCurve convert2 = new(e2d, umed, u2, parameterisation);

                        BSplineCurve3D theCurve2 = BSplineCurveBuilder(theEllipse, convert2);

                        CompCurveToBSplineCurve cctbSpl = new(theCurve1, parameterisation);

                        cctbSpl.Add(theCurve2, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

                        theCurve = cctbSpl.InternalBSplineCurve()!;
                    }
                }
            }

            else if (curv is HyperbolaCurve3D theHyperbola)
            {
                Hyperbola2D h2d = new(Constants.OX2d(), theHyperbola.MajorRadius, theHyperbola.MinorRadius);
                HyperbolaToBSplineCurve convert = new(h2d, u1, u2);
                theCurve = BSplineCurveBuilder(theHyperbola, convert);
            }

            else if (curv is ParabolaCurve3D theParabola)
            {
                Parabola2D prb2d = new(Constants.OX2d(), theParabola.Focal);
                ParabolaToBSplineCurve convert = new(prb2d, u1, u2);
                theCurve = BSplineCurveBuilder(theParabola, convert);
            }

            else if (curv is BezierCurve3D)
            {
                BezierCurve3D cBez = (BezierCurve3D)curv.Copy();

                cBez.Segment(u1, u2);
                int degree = cBez.Degree;
                NCollection_Array1<double> knots = new(1, 2);
                NCollection_Array1<int> mults = new(1, 2);
                knots[1] = 0.0;
                knots[2] = 1.0;
                mults[1] = degree + 1;
                mults[2] = degree + 1;
                NCollection_Array1<Point3D> poles = cBez.InternalPoles();
                if (cBez.IsRational)
                {
                    theCurve = new BSplineCurve3D(poles, cBez.InternalWeightsArray(), knots, mults, degree);
                }
                else
                {
                    theCurve = new BSplineCurve3D(poles, knots, mults, degree);
                }
            }

            else if (curv is BSplineCurve3D)
            {
                theCurve = (BSplineCurve3D)curv.Copy();
                //// modified by jgv, 14.01.05 for OCC7355 ////
                if (theCurve.IsPeriodic)
                {
                    double uf = theCurve.FirstParameter;
                    double ul = theCurve.LastParameter;
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uf, ul, global::Kerf.Engine.Foundation.Precision.Confusion(), ref u1, ref u2);
                    if (Math.Abs(u1 - uf) <= global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(u2 - ul) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        theCurve.SetNotPeriodic();
                    }
                }
                ///////////////////////////////////////////////
                theCurve.Segment(u1, u2);
            }

            else if (curv is OffsetCurve3D)
            {
                double tol3d = 1.0e-4;
                Continuity order = C2;
                int maxSegments = 16, maxDegree = 14;
                ApproxCurve apprCOffs = new(c, tol3d, order, maxSegments, maxDegree);
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
            if (c is EllipseCurve3D theEllipse)
            {
                Ellipse2D e2d = new(Constants.OX2d(), theEllipse.MajorRadius, theEllipse.MinorRadius);
                EllipseToBSplineCurve convert = new(e2d, parameterisation);
                theCurve = BSplineCurveBuilder(theEllipse, convert);
                theCurve.SetPeriodic();
            }

            else if (c is CircleCurve3D theCircle)
            {
                Circle2D c2d = new(Constants.OX2d(), theCircle.Radius);
                CircleToBSplineCurve convert = new(c2d, parameterisation);
                theCurve = BSplineCurveBuilder(theCircle, convert);
                theCurve.SetPeriodic();
            }

            else if (c is BezierCurve3D cBez)
            {
                int degree = cBez.Degree;
                NCollection_Array1<double> knots = new(1, 2);
                NCollection_Array1<int> mults = new(1, 2);
                knots[1] = 0.0;
                knots[2] = 1.0;
                mults[1] = degree + 1;
                mults[2] = degree + 1;
                NCollection_Array1<Point3D> poles = cBez.InternalPoles();
                if (cBez.IsRational)
                {
                    theCurve = new BSplineCurve3D(poles, cBez.InternalWeightsArray(), knots, mults, degree);
                }
                else
                {
                    theCurve = new BSplineCurve3D(poles, knots, mults, degree);
                }
            }
            else if (c is BSplineCurve3D)
            {
                theCurve = (BSplineCurve3D)c.Copy();
            }

            else if (c is OffsetCurve3D)
            {
                double tol3d = 1.0e-4;
                Continuity order = C2;
                int maxSegments = 16, maxDegree = 14;
                ApproxCurve apprCOffs = new(c, tol3d, order, maxSegments, maxDegree);
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

        return theCurve;
    }

    private sealed class GeomConvert_law_evaluator : EvaluatorFunction
    {
        private readonly BSplineCurve2D? _ancore;

        public GeomConvert_law_evaluator(BSplineCurve2D? ancore) => _ancore = ancore;

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

    private static BSplineCurve3D MultNumandDenom(BSplineCurve2D a, BSplineCurve3D bs)
    {
        NCollection_Array1<double> bsKnots = bs.InternalKnots();
        NCollection_Array1<int> bsMults = bs.InternalMultiplicities();
        NCollection_Array1<Point3D> bsPoles = new(bs.InternalPoles());
        NCollection_Array1<double> bsWeights = bs.InternalWeightsArray();
        NCollection_Array1<double> bsFlatKnots = bs.InternalKnotSequence();
        BSplineCurve3D res;
        double start_value, end_value;
        double tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int resNbPoles, degree, ii, jj, status;

        start_value = bsKnots[1];
        end_value = bsKnots[bs.KnotCount];
        if ((end_value - start_value) / 5 < tolerance)
        {
            tolerance = (end_value - start_value) / 5;
        }

        NCollection_Array1<double> knots = new(a.InternalKnots());
        NCollection_Array1<Point2D> poles = a.InternalPoles();
        NCollection_Array1<int> mults = a.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(bs.FirstParameter, bs.LastParameter, knots);
        BSplineCurve2D ancore = new(poles, knots, mults, a.Degree);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.MergeBSplineKnots(tolerance,
                                   start_value,
                                   end_value, // merge of the knots
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
        NCollection_Array1<Point3D> resNumPoles = new(1, resNbPoles);
        NCollection_Array1<double> resDenPoles = new(1, resNbPoles);
        NCollection_Array1<Point3D> resPoles = new(1, resNbPoles);
        NCollection_Array1<double> resFlatKnots = new(1, resNbPoles + degree + 1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(resKnots, resMults, resFlatKnots);
        for (ii = 1; ii <= bs.PoleCount; ii++)
        {
            for (jj = 1; jj <= 3; jj++)
            {
                bsPoles[ii] = bsPoles[ii].WithCoord(jj, bsPoles[ii].Coord(jj) * bsWeights[ii]);
            }
        }

        // POP for WNT
        GeomConvert_law_evaluator ev = new(ancore);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(ev, bs.Degree, bsFlatKnots, bsPoles, resFlatKnots, degree, resNumPoles, out status);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(ev, bs.Degree, bsFlatKnots, bsWeights, resFlatKnots, degree, resDenPoles, out status);
        for (ii = 1; ii <= resNbPoles; ii++)
        {
            for (jj = 1; jj <= 3; jj++)
            {
                resPoles[ii] = resPoles[ii].WithCoord(jj, resNumPoles[ii].Coord(jj) / resDenPoles[ii]);
            }
        }

        res = new BSplineCurve3D(resPoles, resDenPoles, resKnots, resMults, degree);
        return res;
    }

    private static void Pretreatment(NCollection_Array1<BSplineCurve3D> tab)
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
                    BSplineCurve3D curve = (BSplineCurve3D)tab[i].Copy();
                    for (j = 1; j <= curve.PoleCount; j++)
                    {
                        curve.SetWeight(j, curve.Weight(j) / a);
                    }

                    tab[i] = curve;
                }
            }
        }
    }

    private static bool NeedToBeTreated(BSplineCurve3D bs)
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

    private static bool Need2DegRepara(NCollection_Array1<BSplineCurve3D> tab)
    {
        int i;
        Vector3D vec1, vec2;
        double rapport = 1.0e0;

        for (i = 0; i <= tab.Length() - 2; i++)
        {
            tab[i + 1].D1(tab[i + 1].FirstParameter, out _, out vec1);
            tab[i].D1(tab[i].LastParameter, out _, out vec2);
            rapport = rapport * vec2.Magnitude / vec1.Magnitude;
        }

        return (rapport > (1.0e0 + global::Kerf.Engine.Foundation.Precision.Confusion())) || (rapport < (1.0e0 - global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    private static int Indexmin(NCollection_Array1<BSplineCurve3D> tab)
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

    private static void ReorderArrayOfG1Curves(NCollection_Array1<BSplineCurve3D> arrayOfCurves,
                                         NCollection_Array1<double> arrayOfToler,
                                         NCollection_Array1<bool> tabG1,
                                         int startIndex,
                                         double closedTolerance)
    {
        int i;
        NCollection_Array1<BSplineCurve3D> arraybisOfCurves = new(0, arrayOfCurves.Length() - 1);
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

    private sealed class GeomConvert_reparameterise_evaluator : EvaluatorFunction
    {
        private readonly double[] _polynomialCoefficient = new double[3];

        public GeomConvert_reparameterise_evaluator(ReadOnlySpan<double> polynomialCoefficient) =>
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
    /// tolerances of the junctions. If ClosedG1Flag is true and the curves form one closed G1 group, the result is one
    /// periodic curve, closed within ClosedTolerance; ClosedG1Flag is set to false if the closing reparametrisation
    /// would raise the degree beyond the maximum. ArrayOfCurves may be reordered, and its rational curves with equal
    /// end weights replaced by normalised copies.
    /// </summary>
    /// <exception cref="FailureException">Two consecutive curves don't join.</exception>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void ConcatG1(NCollection_Array1<BSplineCurve3D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<BSplineCurve3D> arrayOfConcatenated,
                                ref bool closedG1Flag,
                                double closedTolerance)
    {
        int nb_curve = arrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            first, preLast = 0;
        Continuity cont;
        Vector3D vec1, vec2; // consecutive tangential vectors
        BSplineCurve3D curve1, curve2;
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
                cont = global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true);
                if (cont < C0)
                {
                    throw new ConstructionErrorException("GeomConvert curves not C0");
                }
                else
                {
                    if (cont >= G1)
                    {
                        tabG1[i - 1] = true; // True=G1 continuity
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

        if ((closedG1Flag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1Curves(arrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, closedTolerance);
        }

        arrayOfConcatenated = new NCollection_HArray1<BSplineCurve3D>(0, nb_group - 1);

        bool fusion;
        index = 0;
        Pretreatment(arrayOfCurves);

        Span<double> polynomialCoefficient = stackalloc double[3];

        bool needDoubleDegRepara = Need2DegRepara(arrayOfCurves);
        if (nb_group == 1 && closedG1Flag && needDoubleDegRepara)
        {
            curve1 = arrayOfCurves[nb_curve - 1];
            // OCCT reads Geom2d_BSplineCurve's maximum degree, which is Geom_BSplineCurve's (25)
            if (curve1.Degree > BSplineCurve2D.MaxDegree / 2)
            {
                closedG1Flag = false;
            }
        }

        if ((nb_group == 1) && (closedG1Flag))
        { // treatment of a particular case
            indexmin = Indexmin(arrayOfCurves);
            if (indexmin != (arrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1Curves(arrayOfCurves, local_tolerance, tabG1, indexmin, closedTolerance);
            }

            curve2 = arrayOfCurves[0];
            for (j = 1; j <= nb_curve - 1; j++)
            { // secondary loop inside each group
                curve1 = arrayOfCurves[j];
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

                    NCollection_Array1<Point3D> curve1Poles = new(curve1.InternalPoles());

                    for (ii = 1; ii <= curve1.KnotCount; ii++)
                    {
                        knotC1Mults[ii] = (curve1.Degree + knotC1Mults[ii]);
                    }

                    NCollection_Array1<double> flatKnots = new(1, curve1FlatKnots.Length() + (curve1.Degree * curve1.KnotCount));

                    global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, flatKnots);
                    NCollection_Array1<Point3D> newPoles = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                    int status;
                    NCollection_Array1<double> curve1Weights = curve1.InternalWeightsArray();
                    for (ii = 1; ii <= curve1.PoleCount; ii++)
                    {
                        for (jj = 1; jj <= 3; jj++)
                        {
                            curve1Poles[ii] = curve1Poles[ii].WithCoord(jj, curve1Poles[ii].Coord(jj) * curve1Weights[ii]);
                        }
                    }

                    // POP for WNT
                    GeomConvert_reparameterise_evaluator ev = new(polynomialCoefficient);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Poles, flatKnots,
                                                    2 * curve1.Degree, newPoles, out status);
                    NCollection_Array1<double> newWeights = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Weights, flatKnots,
                                                    2 * curve1.Degree, newWeights, out status);
                    for (ii = 1; ii <= newPoles.Length(); ii++)
                    {
                        for (jj = 1; jj <= 3; jj++)
                        {
                            newPoles[ii] = newPoles[ii].WithCoord(jj, newPoles[ii].Coord(jj) / newWeights[ii]);
                        }
                    }

                    curve1 = new BSplineCurve3D(newPoles, newWeights, knotC1, knotC1Mults, 2 * curve1.Degree);
                }

                CompCurveToBSplineCurve C = new(curve2);
                fusion = C.Add(curve1, local_tolerance[j - 1]); // merge two adjacent curves
                if (!fusion)
                {
                    throw new ConstructionErrorException("GeomConvert Concatenation Error");
                }

                curve2 = C.InternalBSplineCurve()!;
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                curve2 = (BSplineCurve3D)curve2.Copy();
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
                        CompCurveToBSplineCurve C = new(arrayOfConcatenated.Value(i));
                        // OCCT reads the caller's tolerances, not the ones reordered for a closed wire (occt-issues.md)
                        fusion = C.Add(curve1, arrayOfToler[j - 1]); // fusion de deux courbes adjacentes

                        if (!fusion)
                        {
                            throw new ConstructionErrorException("GeomConvert Concatenation Error");
                        }

                        arrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// <see cref="ConcatC1(NCollection_Array1{BSplineCurve3D}, NCollection_Array1{double}, out NCollection_HArray1{int}, out NCollection_HArray1{BSplineCurve3D}, ref bool, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void ConcatC1(NCollection_Array1<BSplineCurve3D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<int> arrayOfIndices,
                                out NCollection_HArray1<BSplineCurve3D> arrayOfConcatenated,
                                ref bool closedG1Flag,
                                double closedTolerance)
    {
        ConcatC1(arrayOfCurves, arrayOfToler, out arrayOfIndices, out arrayOfConcatenated, ref closedG1Flag,
                 closedTolerance, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    /// <summary>
    /// As <see cref="ConcatG1"/>, but the curves are made C1 where they join with G1 continuity (rational curves
    /// are reparametrised first), and AngularTolerance is the tolerance of the G1 test. ArrayOfIndices (from 0) holds
    /// the indices in ArrayOfCurves of the first curve of each concatenated group, and a last index.
    /// </summary>
    /// <exception cref="FailureException">Two consecutive curves don't join.</exception>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void ConcatC1(NCollection_Array1<BSplineCurve3D> arrayOfCurves,
                                NCollection_Array1<double> arrayOfToler,
                                out NCollection_HArray1<int> arrayOfIndices,
                                out NCollection_HArray1<BSplineCurve3D> arrayOfConcatenated,
                                ref bool closedG1Flag,
                                double closedTolerance,
                                double angularTolerance)
    {
        int nb_curve = arrayOfCurves.Length(), nb_vertexG1, nb_group = 0, index = 0, i, ii, j, jj, indexmin,
            nb_vertex_group0 = 0;
        double lambda, // G1 junction coefficient
            first, preLast = 0;
        Continuity cont;
        Vector3D vec1, vec2; // consecutive tangential vectors
        BSplineCurve3D curve1, curve2 = null!;
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
                cont = global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(arrayOfCurves[i - 1], arrayOfCurves[i], preLast, first, true, true,
                                            local_tolerance[i - 1], angularTolerance);
                if (cont < C0)
                {
                    throw new ConstructionErrorException("GeomConvert curves not C0");
                }
                else
                {
                    if (cont >= G1)
                    {
                        tabG1[i - 1] = true; // True=G1 continuity
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

        if ((closedG1Flag) && (nb_group != 1))
        { // rearrangement du tableau
            nb_group--;
            ReorderArrayOfG1Curves(arrayOfCurves, local_tolerance, tabG1, nb_vertex_group0, closedTolerance);
        }

        arrayOfIndices = new NCollection_HArray1<int>(0, nb_group);
        arrayOfConcatenated = new NCollection_HArray1<BSplineCurve3D>(0, nb_group - 1);

        bool fusion;
        int k = 0;
        index = 0;
        Pretreatment(arrayOfCurves);
        Span<double> polynomialCoefficient = stackalloc double[3];

        bool needDoubleDegRepara = Need2DegRepara(arrayOfCurves);
        if (nb_group == 1 && closedG1Flag && needDoubleDegRepara)
        {
            curve1 = arrayOfCurves[nb_curve - 1];
            // OCCT reads Geom2d_BSplineCurve's maximum degree, which is Geom_BSplineCurve's (25)
            if (curve1.Degree > BSplineCurve2D.MaxDegree / 2)
            {
                closedG1Flag = false;
            }
        }

        if ((nb_group == 1) && (closedG1Flag))
        { // treatment of a particular case
            arrayOfIndices.SetValue(0, 0);
            arrayOfIndices.SetValue(1, 0);
            indexmin = Indexmin(arrayOfCurves);
            if (indexmin != (arrayOfCurves.Length() - 1))
            {
                ReorderArrayOfG1Curves(arrayOfCurves, local_tolerance, tabG1, indexmin, closedTolerance);
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

                        NCollection_Array1<Point3D> curve1Poles = new(curve1.InternalPoles());

                        for (ii = 1; ii <= curve1.KnotCount; ii++)
                        {
                            knotC1Mults[ii] = (curve1.Degree + knotC1Mults[ii]);
                        }

                        NCollection_Array1<double> flatKnots = new(1, curve1FlatKnots.Length() + (curve1.Degree * curve1.KnotCount));

                        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knotC1, knotC1Mults, flatKnots);
                        NCollection_Array1<Point3D> newPoles = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                        int status;
                        NCollection_Array1<double> curve1Weights = curve1.InternalWeightsArray();
                        for (ii = 1; ii <= curve1.PoleCount; ii++)
                        {
                            for (jj = 1; jj <= 3; jj++)
                            {
                                curve1Poles[ii] = curve1Poles[ii].WithCoord(jj, curve1Poles[ii].Coord(jj) * curve1Weights[ii]);
                            }
                        }

                        // POP for WNT
                        GeomConvert_reparameterise_evaluator ev = new(polynomialCoefficient);
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Poles, flatKnots,
                                                        2 * curve1.Degree, newPoles, out status);
                        NCollection_Array1<double> newWeights = new(1, flatKnots.Length() - (2 * curve1.Degree + 1));
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(ev, curve1.Degree, curve1FlatKnots, curve1Weights, flatKnots,
                                                        2 * curve1.Degree, newWeights, out status);
                        for (ii = 1; ii <= newPoles.Length(); ii++)
                        {
                            for (jj = 1; jj <= 3; jj++)
                            {
                                newPoles[ii] = newPoles[ii].WithCoord(jj, newPoles[ii].Coord(jj) / newWeights[ii]);
                            }
                        }

                        curve1 = new BSplineCurve3D(newPoles, newWeights, knotC1, knotC1Mults, 2 * curve1.Degree);
                    }

                    CompCurveToBSplineCurve C = new(curve2);
                    fusion = C.Add(curve1, local_tolerance[j - 1]); // merge two adjacent curves
                    if (!fusion)
                    {
                        throw new ConstructionErrorException("GeomConvert Concatenation Error");
                    }

                    curve2 = C.InternalBSplineCurve()!;
                }
            }

            // OCCT changes the caller's curve when there is only one; the port changes a copy
            if (nb_curve == 1)
            {
                curve2 = (BSplineCurve3D)curve2.Copy();
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

                if ((!closedG1Flag) || (nb_group == 1))
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
                        // Merge of two consecutive curves.
                        CompCurveToBSplineCurve C = new(arrayOfConcatenated.Value(i));
                        fusion = C.Add(curve1, local_tolerance[j - 1], true);
                        if (!fusion)
                        {
                            throw new ConstructionErrorException("GeomConvert Concatenation Error");
                        }

                        arrayOfConcatenated.SetValue(i, C.InternalBSplineCurve()!);
                    }
                }

                index = index + 1 + nb_vertexG1;
            }
        }
    }

    /// <summary>
    /// Makes BS C1 where it is only C0 but G1 within the tolerances: BS is cut at its C0 knots, the pieces are
    /// concatenated by <see cref="C0BSplineToArrayOfC1BSplineCurve(BSplineCurve3D, out NCollection_HArray1{BSplineCurve3D}, double, double)"/>
    /// and joined again. BS is replaced by a new curve; the curve given is not changed.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void C0BSplineToC1BSplineCurve(ref BSplineCurve3D bs, double tolerance, double angularTol = 1.0e-7)
    {
        bool fusion;
        // the array with the resulting curves

        C0BSplineToArrayOfC1BSplineCurve(bs, out NCollection_HArray1<BSplineCurve3D> arrayOfConcatenated, angularTol,
                                         tolerance);

        CompCurveToBSplineCurve c = new(arrayOfConcatenated.Value(0));
        if (arrayOfConcatenated.Length() >= 2)
        {
            int i;
            for (i = 1; i < arrayOfConcatenated.Length(); i++)
            {
                fusion = c.Add(arrayOfConcatenated.Value(i), tolerance);
                if (!fusion)
                {
                    throw new ConstructionErrorException("GeomConvert Concatenation Error");
                }
            }
        }

        bs = c.InternalBSplineCurve()!;
    }

    /// <summary>
    /// <see cref="C0BSplineToArrayOfC1BSplineCurve(BSplineCurve3D, out NCollection_HArray1{BSplineCurve3D}, double, double)"/>
    /// with the angular tolerance Precision::Angular().
    /// </summary>
    public static void C0BSplineToArrayOfC1BSplineCurve(BSplineCurve3D bs,
                                                        out NCollection_HArray1<BSplineCurve3D> tabBS,
                                                        double tolerance)
    {
        C0BSplineToArrayOfC1BSplineCurve(bs, out tabBS, global::Kerf.Engine.Foundation.Precision.Angular(), tolerance);
    }

    /// <summary>
    /// Cuts BS at its C0 knots and concatenates the pieces that join with G1 continuity (within the tolerances) into
    /// C1 curves, returned in tabBS (from 0). A curve without C0 knots is returned as the single element.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A concatenation fails.</exception>
    public static void C0BSplineToArrayOfC1BSplineCurve(BSplineCurve3D bs,
                                                        out NCollection_HArray1<BSplineCurve3D> tabBS,
                                                        double angularTolerance,
                                                        double tolerance)
    {
        NCollection_Array1<int> bsMults = bs.InternalMultiplicities();
        NCollection_Array1<double> bsKnots = bs.InternalKnots();
        int i, j, nbcurveC1 = 1;
        double u1, u2;
        bool closed_flag = false;
        Vector3D v1, v2;
        for (i = bs.FirstUKnotIndex(); i <= (bs.LastUKnotIndex() - 1); i++)
        { // give the number of C1 curves
            if (bsMults[i] == bs.Degree)
            {
                nbcurveC1++;
            }
        }

        if (nbcurveC1 > 1)
        {
            NCollection_Array1<BSplineCurve3D> arrayOfCurves = new(0, nbcurveC1 - 1);
            NCollection_Array1<double> arrayOfToler = new(0, nbcurveC1 - 2);

            for (i = 0; i <= nbcurveC1 - 2; i++)
            {
                // filling of the array of tolerances
                arrayOfToler[i] = tolerance;
            }

            // with the variable tolerance
            u2 = bs.FirstParameter;
            j = bs.FirstUKnotIndex() + 1;
            for (i = 0; i < nbcurveC1; i++)
            {
                // filling of the array of curves
                u1 = u2;
                // with the curves C1 segmented
                while (bsMults[j] < bs.Degree && j < bs.LastUKnotIndex())
                {
                    j++;
                }

                u2 = bsKnots[j];
                j++;
                BSplineCurve3D bSbis = (BSplineCurve3D)bs.Copy();
                bSbis.Segment(u1, u2);
                arrayOfCurves[i] = bSbis;
            }

            bs.D1(bs.FirstParameter, out _, out v1);
            bs.D1(bs.LastParameter, out _, out v2);

            if ((bs.IsClosed) && (v1.IsParallel(v2, angularTolerance)))
            {
                // check if the BSpline is closed G1
                closed_flag = true;
            }

            ConcatC1(arrayOfCurves, arrayOfToler, out _, out tabBS, ref closed_flag, tolerance, angularTolerance);
        }
        else
        {
            tabBS = new NCollection_HArray1<BSplineCurve3D>(0, 0);
            tabBS.SetValue(0, bs);
        }
    }

    // GeomConvert_1.cxx: the surfaces

    private static BSplineSurface BSplineSurfaceBuilder(ElementarySurfaceToBSplineSurface convert)
    {
        BSplineSurface theSurface;
        theSurface = new BSplineSurface(convert.Poles(),
                                             convert.Weights(),
                                             convert.UKnots(),
                                             convert.VKnots(),
                                             convert.UMultiplicities(),
                                             convert.VMultiplicities(),
                                             convert.UDegree(),
                                             convert.VDegree(),
                                             convert.IsUPeriodic,
                                             convert.IsVPeriodic);
        return theSurface;
    }

    /// <summary>
    /// The part of S between its U knots FromUK1 and ToUK2 and its V knots FromVK1 and ToVK2. In a non-periodic
    /// direction, it is reversed if the first knot index is the greater; in a periodic one, if SameUOrientation
    /// (SameVOrientation) is false.
    /// </summary>
    /// <exception cref="DomainErrorException">
    /// FromUK1 = ToUK2 or FromVK1 = ToVK2, or a knot index is outside the first and last knot indices.
    /// </exception>
    public static BSplineSurface SplitBSplineSurface(BSplineSurface s,
                                                          int fromUK1,
                                                          int toUK2,
                                                          int fromVK1,
                                                          int toVK2,
                                                          bool sameUOrientation = true,
                                                          bool sameVOrientation = true)
    {
        int firstU = s.FirstUKnotIndex();
        int firstV = s.FirstVKnotIndex();
        int lastU = s.LastUKnotIndex();
        int lastV = s.LastVKnotIndex();
        if (fromUK1 == toUK2 || fromVK1 == toVK2)
        {
            throw new DomainErrorException();
        }

        int firstUK = Min(fromUK1, toUK2);
        int lastUK = Max(fromUK1, toUK2);
        int firstVK = Min(fromVK1, toVK2);
        int lastVK = Max(fromVK1, toVK2);
        if (firstUK < firstU || lastUK > lastU || firstVK < firstV || lastVK > lastV)
        {
            throw new DomainErrorException();
        }

        BSplineSurface s1 = (BSplineSurface)s.Copy();

        s1.Segment(s1.UKnot(firstUK), s1.UKnot(lastUK), s1.VKnot(firstVK), s1.VKnot(lastVK));

        if (s.IsUPeriodic)
        {
            if (!sameUOrientation)
            {
                s1.UReverse();
            }
        }
        else
        {
            if (fromUK1 > toUK2)
            {
                s1.UReverse();
            }
        }

        if (s.IsVPeriodic)
        {
            if (!sameVOrientation)
            {
                s1.VReverse();
            }
        }
        else
        {
            if (fromVK1 > toVK2)
            {
                s1.VReverse();
            }
        }

        return s1;
    }

    /// <summary>
    /// The part of S between its knots FromK1 and ToK2 in U (USplit true) or V, whole in the other direction. In a
    /// non-periodic direction, it is reversed if FromK1 &gt; ToK2; in a periodic one, if SameOrientation is false.
    /// </summary>
    /// <exception cref="DomainErrorException">FromK1 = ToK2, or a knot index is outside the first and last.</exception>
    public static BSplineSurface SplitBSplineSurface(BSplineSurface s,
                                                          int fromK1,
                                                          int toK2,
                                                          bool uSplit,
                                                          bool sameOrientation = true)
    {
        if (fromK1 == toK2)
        {
            throw new DomainErrorException();
        }

        BSplineSurface s1 = (BSplineSurface)s.Copy();

        if (uSplit)
        {
            int firstU = s.FirstUKnotIndex();
            int lastU = s.LastUKnotIndex();
            int firstUK = Min(fromK1, toK2);
            int lastUK = Max(fromK1, toK2);
            if (firstUK < firstU || lastUK > lastU)
            {
                throw new DomainErrorException();
            }

            s1.Segment(s1.UKnot(firstUK),
                       s1.UKnot(lastUK),
                       s1.VKnot(s1.FirstVKnotIndex()),
                       s1.VKnot(s1.LastVKnotIndex()));

            if (s.IsUPeriodic)
            {
                if (!sameOrientation)
                {
                    s1.UReverse();
                }
            }
            else
            {
                if (fromK1 > toK2)
                {
                    s1.UReverse();
                }
            }
        }
        else
        {
            int firstV = s.FirstVKnotIndex();
            int lastV = s.LastVKnotIndex();
            int firstVK = Min(fromK1, toK2);
            int lastVK = Max(fromK1, toK2);
            if (firstVK < firstV || lastVK > lastV)
            {
                throw new DomainErrorException();
            }

            s1.Segment(s1.UKnot(s1.FirstUKnotIndex()),
                       s1.UKnot(s1.LastUKnotIndex()),
                       s1.VKnot(firstVK),
                       s1.VKnot(lastVK));

            if (s.IsVPeriodic)
            {
                if (!sameOrientation)
                {
                    s1.VReverse();
                }
            }
            else
            {
                if (fromK1 > toK2)
                {
                    s1.VReverse();
                }
            }
        }

        return s1;
    }

    /// <summary>
    /// The part of S between the parameters FromU1 and ToU2 in U and FromV1 and ToV2 in V. In a non-periodic
    /// direction, it is reversed if the first parameter is the greater; in a periodic one, if SameUOrientation
    /// (SameVOrientation) is false. ParametricTolerance is not used.
    /// </summary>
    public static BSplineSurface SplitBSplineSurface(BSplineSurface s,
                                                          double fromU1,
                                                          double toU2,
                                                          double fromV1,
                                                          double toV2,
                                                          double parametricTolerance,
                                                          bool sameUOrientation = true,
                                                          bool sameVOrientation = true)
    {
        double firstU = Min(fromU1, toU2);
        double lastU = Max(fromU1, toU2);
        double firstV = Min(fromV1, toV2);
        double lastV = Max(fromV1, toV2);

        BSplineSurface newSurface = (BSplineSurface)s.Copy();

        newSurface.Segment(firstU, lastU, firstV, lastV);

        if (s.IsUPeriodic)
        {
            if (!sameUOrientation)
            {
                newSurface.UReverse();
            }
        }
        else
        {
            if (fromU1 > toU2)
            {
                newSurface.UReverse();
            }
        }

        if (s.IsVPeriodic)
        {
            if (!sameVOrientation)
            {
                newSurface.VReverse();
            }
        }
        else
        {
            if (fromV1 > toV2)
            {
                newSurface.VReverse();
            }
        }

        return newSurface;
    }

    /// <summary>
    /// The part of S between the parameters FromParam1 and ToParam2 in U (USplit true) or V, whole in the other
    /// direction. In U, it is reversed if FromParam1 &gt; ToParam2 (non-periodic) or SameOrientation is false
    /// (periodic). In V, OCCT tests and reverses U instead (occt-issues.md).
    /// </summary>
    /// <exception cref="DomainErrorException">The parameters are within ParametricTolerance.</exception>
    public static BSplineSurface SplitBSplineSurface(BSplineSurface s,
                                                          double fromParam1,
                                                          double toParam2,
                                                          bool uSplit,
                                                          double parametricTolerance,
                                                          bool sameOrientation = true)
    {
        if (Math.Abs(fromParam1 - toParam2) <= Math.Abs(parametricTolerance))
        {
            throw new DomainErrorException();
        }

        BSplineSurface newSurface = (BSplineSurface)s.Copy();

        if (uSplit)
        {
            double firstU = Min(fromParam1, toParam2);
            double lastU = Max(fromParam1, toParam2);
            double firstV = s.VKnot(s.FirstVKnotIndex());
            double lastV = s.VKnot(s.LastVKnotIndex());

            newSurface.Segment(firstU, lastU, firstV, lastV);

            if (s.IsUPeriodic)
            {
                if (!sameOrientation)
                {
                    newSurface.UReverse();
                }
            }
            else
            {
                if (fromParam1 > toParam2)
                {
                    newSurface.UReverse();
                }
            }
        }
        else
        {
            double firstU = s.UKnot(s.FirstUKnotIndex());
            double lastU = s.UKnot(s.LastUKnotIndex());
            double firstV = Min(fromParam1, toParam2);
            double lastV = Max(fromParam1, toParam2);

            newSurface.Segment(firstU, lastU, firstV, lastV);

            // V's periodicity and V reversed (OCCT tested U's and reversed U; Kerf decision 1490)
            if (s.IsVPeriodic)
            {
                if (!sameOrientation)
                {
                    newSurface.VReverse();
                }
            }
            else
            {
                if (fromParam1 > toParam2)
                {
                    newSurface.VReverse();
                }
            }
        }

        return newSurface;
    }

    /// <summary>
    /// Sr as a B-spline surface: planes, quadrics (trimmed), surfaces of revolution and extrusion, Bezier and
    /// B-spline surfaces exactly; offset surfaces through their equivalent surface when they have one; other
    /// surfaces by approximation (GeomConvert_ApproxSurface, tolerance 1e-4, degree 14).
    /// </summary>
    /// <exception cref="DomainErrorException">Sr is infinite.</exception>
    public static BSplineSurface SurfaceToBSplineSurface(Surface sr)
    {
        sr.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        double uFirst = Min(u1, u2);
        double uLast = Max(u1, u2);
        double vFirst = Min(v1, v2);
        double vLast = Max(v1, v2);

        // If the surface Sr is infinite stop the computation
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uFirst) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uLast)
            || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vFirst) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vLast))
        {
            throw new DomainErrorException("GeomConvert::SurfaceToBSplineSurface() - infinite surface");
        }

        BSplineSurface theSurface;
        Surface? s;
        if (sr is OffsetSurface offsetSur)
        {
            s = offsetSur.Surface();
            if (s is not null)
            { // Convert the equivalent surface.
                return SurfaceToBSplineSurface(s);
            }
        }

        s = sr;

        if (s is TrimmedSurface Strim)
        {
            Surface surf = Strim.BasisSurface;
            uFirst = u1;
            uLast = u2;
            vFirst = v1;
            vLast = v2;
            if (surf is OffsetSurface offsetSurBasis)
            {
                s = offsetSurBasis.Surface();
                if (s is not null)
                {
                    surf = s;
                }
                else
                {
                    s = surf;
                }
            }

            if (surf is TrimmedSurface)
            {
                TrimmedSurface strim = new(surf, uFirst, uLast, vFirst, vLast);
                return SurfaceToBSplineSurface(strim);
            }

            //
            // For cylinders, cones, spheres, toruses
            bool isUClosed = Math.Abs((uLast - uFirst) - 2.0 * M_PI) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
            double eps = 100.0 * Epsilon(2.0 * M_PI);
            //
            if (surf is PlaneSurface)
            {
                NCollection_Array2<Point3D> Poles = new(1, 2, 1, 2);
                Poles[1, 1] = Strim.Value(u1, v1);
                Poles[1, 2] = Strim.Value(u1, v2);
                Poles[2, 1] = Strim.Value(u2, v1);
                Poles[2, 2] = Strim.Value(u2, v2);
                NCollection_Array1<double> uKnots = new(1, 2);
                NCollection_Array1<double> vKnots = new(1, 2);
                NCollection_Array1<int> uMults = new(1, 2);
                NCollection_Array1<int> vMults = new(1, 2);
                uKnots[1] = u1;
                uKnots[2] = u2;
                vKnots[1] = v1;
                vKnots[2] = v2;
                uMults[1] = 2;
                uMults[2] = 2;
                vMults[1] = 2;
                vMults[2] = 2;
                int uDegree = 1;
                int vDegree = 1;
                theSurface = new BSplineSurface(Poles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);
            }
            else if (surf is CylindricalSurface theCylSurf)
            {
                Cylinder cyl = theCylSurf.Cylinder;
                if (isUClosed)
                {
                    CylinderToBSplineSurface convert = new(cyl, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                    int nbK = theSurface.UKnotCount;
                    if (Math.Abs(theSurface.UKnot(1) - uFirst) > eps || Math.Abs(theSurface.UKnot(nbK) - uLast) > eps)
                    {
                        theSurface.CheckAndSegment(uFirst, uLast, vFirst, vLast);
                    }
                }
                else
                {
                    CylinderToBSplineSurface conv = new(cyl, uFirst, uLast, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(conv);
                }
            }

            else if (surf is ConicalSurface theConeSurf)
            {
                Cone co = theConeSurf.Cone;
                if (isUClosed)
                {
                    ConeToBSplineSurface convert = new(co, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                    int nbK = theSurface.UKnotCount;
                    if (Math.Abs(theSurface.UKnot(1) - uFirst) > eps || Math.Abs(theSurface.UKnot(nbK) - uLast) > eps)
                    {
                        theSurface.CheckAndSegment(uFirst, uLast, vFirst, vLast);
                    }
                }
                else
                {
                    ConeToBSplineSurface convert = new(co, uFirst, uLast, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                }
            }

            else if (surf is SphericalSurface theSphSurf)
            {
                Sphere sph = theSphSurf.Sphere;
                // OCC217
                if (isUClosed)
                {
                    SphereToBSplineSurface convert = new(sph, vFirst, vLast, false);
                    theSurface = BSplineSurfaceBuilder(convert);
                    int nbK = theSurface.UKnotCount;
                    if (Math.Abs(theSurface.UKnot(1) - uFirst) > eps || Math.Abs(theSurface.UKnot(nbK) - uLast) > eps)
                    {
                        theSurface.CheckAndSegment(uFirst, uLast, vFirst, vLast);
                    }
                }
                else
                {
                    SphereToBSplineSurface convert = new(sph, uFirst, uLast, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                }
            }

            else if (surf is ToroidalSurface theTorSurf)
            {
                Torus tr = theTorSurf.Torus;
                //
                // if isUClosed = true and U trim does not coincide with first period of torus,
                // method CheckAndSegment shifts position of U seam boundary of surface.
                // probably bug? So, for this case we must build not periodic surface.
                bool isUFirstPeriod = uFirst >= 0.0 && uLast <= 2.0 * M_PI;
                bool isVFirstPeriod = vFirst >= 0.0 && vLast <= 2.0 * M_PI;
                if (isUClosed && isUFirstPeriod)
                {
                    TorusToBSplineSurface convert = new(tr, vFirst, vLast, false);
                    theSurface = BSplineSurfaceBuilder(convert);
                    int nbK = theSurface.UKnotCount;
                    if (Math.Abs(theSurface.UKnot(1) - uFirst) > eps || Math.Abs(theSurface.UKnot(nbK) - uLast) > eps)
                    {
                        theSurface.CheckAndSegment(uFirst, uLast, vFirst, vLast);
                    }
                }
                else if (Strim.IsVClosed && isVFirstPeriod)
                {
                    TorusToBSplineSurface convert = new(tr, uFirst, uLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                    int nbK = theSurface.VKnotCount;
                    if (Math.Abs(theSurface.VKnot(1) - vFirst) > eps || Math.Abs(theSurface.VKnot(nbK) - vLast) > eps)
                    {
                        theSurface.CheckAndSegment(uFirst, uLast, vFirst, vLast);
                    }
                }
                else
                {
                    TorusToBSplineSurface convert = new(tr, uFirst, uLast, vFirst, vLast);
                    theSurface = BSplineSurfaceBuilder(convert);
                }
            }

            else if (surf is RevolutionSurface revol)
            {
                Curve3D meridian = revol.BasisCurve;
                BSplineCurve3D c;
                if (Strim.IsVClosed)
                {
                    c = CurveToBSplineCurve(meridian);
                }
                else
                {
                    TrimmedCurve3D ct = new(meridian, vFirst, vLast);
                    c = CurveToBSplineCurve(ct);
                }

                int nbUPoles, nbUKnots;
                int nbVPoles;
                bool periodic = false;

                // Poles of meridian = Vpoles
                nbVPoles = c.PoleCount;
                NCollection_Array1<Point3D> Poles = new(c.InternalPoles());
                NCollection_Array1<double> weights = c.InternalWeightsArray();

                double alfaU;
                if (Strim.IsUPeriodic)
                {
                    nbUKnots = 4;
                    alfaU = M_PI / 3.0;
                    nbUPoles = 6;
                    periodic = true;
                }
                else
                {
                    // Number of spans: maximum opening = 150 degrees ( = PI / 1.2 rad)
                    int nbUSpans = (int)Math.Truncate(1.2 * (uLast - uFirst) / M_PI) + 1;
                    alfaU = (uLast - uFirst) / (nbUSpans * 2);
                    nbUPoles = 2 * nbUSpans + 1;
                    nbUKnots = nbUSpans + 1;
                }

                // Compute Knots and Mults
                NCollection_Array1<double> uKnots = new(1, nbUKnots);
                NCollection_Array1<int> uMults = new(1, nbUKnots);
                int i, j;
                for (i = 1; i <= nbUKnots; i++)
                {
                    uKnots[i] = uFirst + (i - 1) * 2 * alfaU;
                    uMults[i] = 2;
                }

                if (!periodic)
                {
                    uMults[1]++;
                    uMults[nbUKnots]++;
                }

                NCollection_Array1<double> vKnots = c.InternalKnots();
                NCollection_Array1<int> vMults = c.InternalMultiplicities();

                // Compute the poles.
                NCollection_Array2<Point3D> newPoles = new(1, nbUPoles, 1, nbVPoles);
                NCollection_Array2<double> newWeights = new(1, nbUPoles, 1, nbVPoles);
                Transform3D trsf = new();

                for (i = 1; i <= nbUPoles; i += 2)
                {
                    trsf = trsf.WithRotation(revol.Axis, uFirst + (i - 1) * alfaU);
                    for (j = 1; j <= nbVPoles; j++)
                    {
                        newPoles[i, j] = Poles[j].Transformed(trsf);
                        newWeights[i, j] = weights[j];
                    }
                }

                GeneralTransform3D aff = new GeneralTransform3D().WithAffinity(revol.Axis, 1 / Math.Cos(alfaU));
                Xyz coord;
                for (j = 1; j <= nbVPoles; j++)
                {
                    coord = Poles[j].XYZ();
                    aff.Transforms(ref coord);
                    Poles[j] = Poles[j].WithXYZ(coord);
                }

                for (i = 2; i <= nbUPoles; i += 2)
                {
                    trsf = trsf.WithRotation(revol.Axis, uFirst + (i - 1) * alfaU);
                    for (j = 1; j <= nbVPoles; j++)
                    {
                        newPoles[i, j] = Poles[j].Transformed(trsf);
                        newWeights[i, j] = weights[j] * Math.Cos(alfaU);
                    }
                }

                theSurface = new BSplineSurface(newPoles,
                                                     newWeights,
                                                     uKnots,
                                                     vKnots,
                                                     uMults,
                                                     vMults,
                                                     2,
                                                     c.Degree,
                                                     periodic,
                                                     c.IsPeriodic);
            }

            else if (surf is ExtrusionSurface extru)
            {
                Curve3D meridian = extru.BasisCurve;
                BSplineCurve3D c;
                if (Strim.IsUClosed)
                {
                    c = CurveToBSplineCurve(meridian);
                }
                else
                {
                    TrimmedCurve3D ct = new(meridian, uFirst, uLast);
                    c = CurveToBSplineCurve(ct);
                }

                NCollection_Array2<Point3D> Poles = new(1, c.PoleCount, 1, 2);
                NCollection_Array2<double> weights = new(1, c.PoleCount, 1, 2);
                NCollection_Array1<double> uKnots = c.InternalKnots();
                NCollection_Array1<int> uMults = c.InternalMultiplicities();
                NCollection_Array1<double> vKnots = new(1, 2);
                vKnots[1] = vFirst;
                vKnots[2] = vLast;
                NCollection_Array1<int> vMults = new(1, 2);
                vMults.Init(2);

                Vector3D d = new(extru.Direction);
                Vector3D dv1 = vFirst * d;
                Vector3D dv2 = vLast * d;
                for (int i = 1; i <= c.PoleCount; i++)
                {
                    Poles[i, 1] = c.Pole(i).Translated(dv1);
                    Poles[i, 2] = c.Pole(i).Translated(dv2);
                    weights[i, 1] = weights[i, 2] = c.Weight(i);
                }

                theSurface = new BSplineSurface(Poles,
                                                     weights,
                                                     uKnots,
                                                     vKnots,
                                                     uMults,
                                                     vMults,
                                                     c.Degree,
                                                     1,
                                                     c.IsPeriodic,
                                                     false);
            }

            else if (surf is BezierSurface)
            {
                BezierSurface sBez = (BezierSurface)surf.Copy();

                sBez.Segment(u1, u2, v1, v2);
                int uDegree = sBez.UDegree;
                int vDegree = sBez.VDegree;
                NCollection_Array1<double> uKnots = new(1, 2);
                NCollection_Array1<int> uMults = new(1, 2);
                NCollection_Array1<double> vKnots = new(1, 2);
                NCollection_Array1<int> vMults = new(1, 2);
                uKnots[1] = 0.0;
                uKnots[2] = 1.0;
                uMults[1] = uDegree + 1;
                uMults[2] = uDegree + 1;
                vKnots[1] = 0.0;
                vKnots[2] = 1.0;
                vMults[1] = vDegree + 1;
                vMults[2] = vDegree + 1;
                NCollection_Array2<Point3D> aPoles = sBez.InternalPoles();
                if (sBez.IsURational || sBez.IsVRational)
                {
                    theSurface = new BSplineSurface(aPoles,
                                                         sBez.InternalWeightsArray(),
                                                         uKnots,
                                                         vKnots,
                                                         uMults,
                                                         vMults,
                                                         uDegree,
                                                         vDegree);
                }
                else
                {
                    theSurface = new BSplineSurface(aPoles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);
                }
            }

            else if (surf is BSplineSurface)
            {
                BSplineSurface bs = (BSplineSurface)surf.Copy();
                bs.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);
                if (!bs.IsUPeriodic)
                {
                    if (u1 < umin)
                    {
                        u1 = umin;
                    }

                    if (u2 > umax)
                    {
                        u2 = umax;
                    }
                }

                if (!bs.IsVPeriodic)
                {
                    if (v1 < vmin)
                    {
                        v1 = vmin;
                    }

                    if (v2 > vmax)
                    {
                        v2 = vmax;
                    }
                }

                if (bs.IsUPeriodic || bs.IsVPeriodic)
                {
                    bs.CheckAndSegment(u1, u2, v1, v2);
                }
                else
                {
                    bs.Segment(u1, u2, v1, v2);
                }

                theSurface = bs;
            }

            else
            {
                double tol3d = 1.0e-4;
                int maxDegree = 14, maxSeg;
                Continuity cont;
                SurfaceAdaptor AS = new(sr);
                if (AS.NbUIntervals(C2) > 1 || AS.NbVIntervals(C2) > 1)
                {
                    cont = C1;
                }
                else
                {
                    cont = C2;
                }

                maxSeg = 4 * (AS.NbUIntervals(CN) + 1) * (AS.NbVIntervals(CN) + 1);
                ApproxSurface bSpS = new(sr, tol3d, cont, cont, maxDegree, maxDegree, maxSeg, 1);
                // The approximation's surface is frozen (decision 197): the caller gets a copy it owns
                theSurface = (BSplineSurface)bSpS.Surface()!.Copy();
            }
        } // End of Rectangular::TrimmedSurface case

        else
        {
            if (s is SphericalSurface theSphSurf)
            {
                Sphere sph = theSphSurf.Sphere;
                SphereToBSplineSurface convert = new(sph);
                theSurface = BSplineSurfaceBuilder(convert);
            }

            else if (s is ToroidalSurface theTorSurf)
            {
                Torus tr = theTorSurf.Torus;
                TorusToBSplineSurface convert = new(tr);
                theSurface = BSplineSurfaceBuilder(convert);
            }

            else if (s is RevolutionSurface revol)
            {
                Curve3D meridian = revol.BasisCurve;
                BSplineCurve3D c = CurveToBSplineCurve(meridian);

                int nbUPoles, nbUKnots;
                int nbVPoles;
                bool periodic = true;

                // Poles of meridian = Vpoles
                nbVPoles = c.PoleCount;
                NCollection_Array1<Point3D> Poles = new(c.InternalPoles());
                NCollection_Array1<double> weights = c.InternalWeightsArray();

                double alfaU;
                nbUKnots = 4;
                alfaU = M_PI / 3.0;
                nbUPoles = 6;

                // Compute Knots and Mults
                NCollection_Array1<double> uKnots = new(1, nbUKnots);
                NCollection_Array1<int> uMults = new(1, nbUKnots);
                int i, j;
                for (i = 1; i <= nbUKnots; i++)
                {
                    uKnots[i] = uFirst + (i - 1) * 2 * alfaU;
                    uMults[i] = 2;
                }

                NCollection_Array1<double> vKnots = c.InternalKnots();
                NCollection_Array1<int> vMults = c.InternalMultiplicities();

                // Compute the poles.
                NCollection_Array2<Point3D> newPoles = new(1, nbUPoles, 1, nbVPoles);
                NCollection_Array2<double> newWeights = new(1, nbUPoles, 1, nbVPoles);
                Transform3D trsf = new();

                for (i = 1; i <= nbUPoles; i += 2)
                {
                    trsf = trsf.WithRotation(revol.Axis, uFirst + (i - 1) * alfaU);
                    for (j = 1; j <= nbVPoles; j++)
                    {
                        newPoles[i, j] = Poles[j].Transformed(trsf);
                        newWeights[i, j] = weights[j];
                    }
                }

                GeneralTransform3D aff = new GeneralTransform3D().WithAffinity(revol.Axis, 1 / Math.Cos(alfaU));
                Xyz coord;
                for (j = 1; j <= nbVPoles; j++)
                {
                    coord = Poles[j].XYZ();
                    aff.Transforms(ref coord);
                    Poles[j] = Poles[j].WithXYZ(coord);
                }

                for (i = 2; i <= nbUPoles; i += 2)
                {
                    trsf = trsf.WithRotation(revol.Axis, uFirst + (i - 1) * alfaU);
                    for (j = 1; j <= nbVPoles; j++)
                    {
                        newPoles[i, j] = Poles[j].Transformed(trsf);
                        newWeights[i, j] = weights[j] * Math.Cos(alfaU);
                    }
                }

                theSurface = new BSplineSurface(newPoles,
                                                     newWeights,
                                                     uKnots,
                                                     vKnots,
                                                     uMults,
                                                     vMults,
                                                     2,
                                                     c.Degree,
                                                     periodic,
                                                     c.IsPeriodic);
            }

            else if (s is BezierSurface sBez)
            {
                int uDegree = sBez.UDegree;
                int vDegree = sBez.VDegree;
                NCollection_Array1<double> uKnots = new(1, 2);
                NCollection_Array1<int> uMults = new(1, 2);
                NCollection_Array1<double> vKnots = new(1, 2);
                NCollection_Array1<int> vMults = new(1, 2);
                uKnots[1] = 0.0;
                uKnots[2] = 1.0;
                uMults[1] = uDegree + 1;
                uMults[2] = uDegree + 1;
                vKnots[1] = 0.0;
                vKnots[2] = 1.0;
                vMults[1] = vDegree + 1;
                vMults[2] = vDegree + 1;
                NCollection_Array2<Point3D> poles = sBez.InternalPoles();
                if (sBez.IsURational || sBez.IsVRational)
                {
                    theSurface = new BSplineSurface(poles,
                                                         sBez.InternalWeightsArray(),
                                                         uKnots,
                                                         vKnots,
                                                         uMults,
                                                         vMults,
                                                         uDegree,
                                                         vDegree);
                }
                else
                {
                    theSurface = new BSplineSurface(poles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);
                }
            }

            else if (s is BSplineSurface)
            {
                theSurface = (BSplineSurface)s.Copy(); // Just a copy
            }

            else
            { // In other cases => Approx
                double tol3d = 1.0e-4;
                int maxDegree = 14, maxSeg;
                Continuity ucont = C0, vcont = C0;
                SurfaceAdaptor AS = new(sr);
                //
                if (sr.IsCNu(2))
                {
                    ucont = C2;
                }
                else if (sr.IsCNu(1))
                {
                    ucont = C1;
                }

                //
                if (sr.IsCNv(2))
                {
                    vcont = C2;
                }
                else if (sr.IsCNv(1))
                {
                    vcont = C1;
                }

                //
                maxSeg = 4 * (AS.NbUIntervals(CN) + 1) * (AS.NbVIntervals(CN) + 1);
                ApproxSurface bSpS = new(sr, tol3d, ucont, vcont, maxDegree, maxDegree, maxSeg, 1);
                // The approximation's surface is frozen (decision 197): the caller gets a copy it owns
                theSurface = (BSplineSurface)bSpS.Surface()!.Copy();
            }
        } // End of direct case

        return theSurface;
    }
}
