// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSplineSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Approximates or interpolates a grid of points by a B-spline surface: the points' rows (constant J) are
/// approximated first as curves in U, then the curves' poles as curves in V. The methods are least squares
/// (AppDef_BSplineCompute) with a parametrisation type, interpolation, the variational method
/// (AppDef_Variational) with criterion weights, and approximation of a grid of Z values over a regular XY grid.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's implicit conversion to the surface is an explicit conversion (decision 28).
/// <see cref="Surface"/> returns the computed surface itself, as OCCT returns its handle, frozen (decisions 243, 341
/// and 5805). As in OCCT, the methods on points index the multi-points by the arrays' own row and column indices, so
/// arrays must start at 1 (occt-issues.md).
/// </remarks>
internal class PointsToBSplineSurface
{
    private bool _isDone;
    private BSplineSurface? _surface;

    private static bool HasMeaningfulSpan(double span)
    {
        double resolution = Constants.Resolution();
        return span * span > resolution * resolution;
    }

    private static void BuildParameters(MultiLine line,
                                        global::Kerf.Geometry.ParametrizationType parT,
                                        NCollection_Array1<double> pars)
    {
        int i, j, nbP3d = line.PointCount;
        double dist;
        int firstP = 1, lastP = line.MultiPointCount;
        int nbp = lastP - firstP + 1;

        if (nbp == 2)
        {
            pars[firstP] = 0.0;
            pars[lastP] = 1.0;
        }
        else if (parT == ChordLength || parT == Centripetal)
        {
            pars[firstP] = 0.0;
            dist = 0.0;

            for (i = firstP + 1; i <= lastP; i++)
            {
                MultiPointConstraint mpc = line.Value(i - 1);
                MultiPointConstraint mpc1 = line.Value(i);

                dist = 0.0;
                for (j = 1; j <= nbP3d; j++)
                {
                    Point3D p1 = mpc.Point(j), p2 = mpc1.Point(j);
                    dist += p2.SquareDistanceTo(p1);
                }

                dist = Math.Sqrt(dist);
                if (parT == ChordLength)
                {
                    pars[i] = pars[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    pars[i] = pars[i - 1] + Math.Sqrt(dist);
                }
            }
            if (HasMeaningfulSpan(pars[lastP]))
            {
                for (i = firstP; i <= lastP; i++)
                {
                    pars[i] /= pars[lastP];
                }
            }
            else
            {
                for (i = firstP; i <= lastP; i++)
                {
                    pars[i] = ((double)i - firstP) / ((double)(lastP - firstP));
                }
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                pars[i] = ((double)i - firstP) / ((double)(lastP - (double)firstP));
            }
        }
    }

    private static void BuildPeriodicTangent(MultiLine line,
                                             NCollection_Array1<double> pars,
                                             math_Vector tang)
    {
        int firstpt = 1, lastpt = line.MultiPointCount;
        int nbpoints = lastpt - firstpt + 1;
        //
        if (nbpoints <= 2)
        {
            return;
        }
        //
        int i, nnpol, nnp = Math.Min(nbpoints, 9);
        nnpol = nnp;
        int lastp = Math.Min(lastpt, firstpt + nnp - 1);
        double u;
        Constraint cons = TangencyPoint;
        if (nnp <= 4)
        {
            cons = PassPoint;
        }
        int nbP = 3 * line.PointCount;
        math_Vector v1 = new(1, nbP), v2 = new(1, nbP);
        math_Vector p1 = new(firstpt, lastp);
        //
        for (i = firstpt; i <= lastp; i++)
        {
            p1[i] = pars[i];
        }

        BSpParLeastSquareOfMyBSplGradientOfBSplineCompute sq1 = new(line,
                                                                          firstpt,
                                                                          lastp,
                                                                          cons,
                                                                          cons,
                                                                          nnpol);
        sq1.Perform(p1);
        MultiCurve c1 = sq1.BezierValue();
        u = 0.0;
        int j, nbP3d = line.PointCount;

        Point3D p;
        Vector3D v;
        j = 1;
        for (i = 1; i <= nbP3d; i++)
        {
            c1.D1(i, u, out p, out v);
            v1[j] = v.X;
            v1[j + 1] = v.Y;
            v1[j + 2] = v.Z;
            j += 3;
        }

        int firstp = Math.Max(firstpt, lastpt - nnp + 1);

        if (firstp == firstpt && lastp == lastpt)
        {
            u = 1.0;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                c1.D1(i, u, out p, out v);
                v2[j] = v.X;
                v2[j + 1] = v.Y;
                v2[j + 2] = v.Z;
                j += 3;
            }
        }
        else
        {
            BSpParLeastSquareOfMyBSplGradientOfBSplineCompute sq2 = new(line,
                                                                              firstp,
                                                                              lastpt,
                                                                              cons,
                                                                              cons,
                                                                              nnpol);

            math_Vector p2 = new(firstp, lastpt);
            for (i = firstp; i <= lastpt; i++)
            {
                p2[i] = pars[i];
            }
            sq2.Perform(p2);

            MultiCurve c2 = sq2.BezierValue();
            u = 1.0;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                c2.D1(i, u, out p, out v);
                v2[j] = v.X;
                v2[j + 1] = v.Y;
                v2[j + 2] = v.Z;
                j += 3;
            }
        }

        tang.Initialized(0.5 * (v1 + v2));
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c> or <c>Interpolate</c>.</summary>
    public PointsToBSplineSurface()
    {
        _isDone = false;
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public PointsToBSplineSurface(NCollection_Array2<Point3D> points, int degMin = 3, int degMax = 8,
                                          Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, degMin, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>.</summary>
    public PointsToBSplineSurface(NCollection_Array2<Point3D> points, global::Kerf.Geometry.ParametrizationType parType,
                                          int degMin = 3, int degMax = 8, Continuity continuity = C2,
                                          double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, parType, degMin, degMax, continuity, tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising Weight1 x length + Weight2 x
    /// curvature + Weight3 x torsion.
    /// </summary>
    public PointsToBSplineSurface(NCollection_Array2<Point3D> points, double weight1, double weight2,
                                          double weight3, int degMax = 8, Continuity continuity = C2,
                                          double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, weight1, weight2, weight3, degMax, continuity, tol3D);
    }

    /// <summary>
    /// Approximates the heights <paramref name="zPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY.
    /// </summary>
    public PointsToBSplineSurface(NCollection_Array2<double> zPoints, double x0, double dX, double y0,
                                          double dY, int degMin = 3, int degMax = 8,
                                          Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(zPoints, x0, dX, y0, dY, degMin, degMax, continuity, tol3D);
    }

    /// <summary>Interpolates <paramref name="points"/> with chord-length parameters, periodic in U if asked.</summary>
    public void Interpolate(NCollection_Array2<Point3D> points, bool periodic = false)
    {
        Interpolate(points, ChordLength, periodic);
    }

    /// <summary>Interpolates <paramref name="points"/> with the parametrisation <paramref name="parType"/>, periodic in U if asked.</summary>
    public void Interpolate(NCollection_Array2<Point3D> points, global::Kerf.Geometry.ParametrizationType parType,
                            bool periodic = false)
    {
        int degMin, degMax;
        degMin = degMax = 3;
        Continuity cc = C2;
        double tol3d = -1.0;
        Init(points, parType, degMin, degMax, cc, tol3d, periodic);
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array2<Point3D> points, int degMin = 3, int degMax = 8,
                     Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        Init(points, ChordLength, degMin, degMax, continuity, tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>, or interpolates
    /// them if <paramref name="tol3D"/> ≤ 0; periodic in U if asked.
    /// </summary>
    public void Init(NCollection_Array2<Point3D> points, global::Kerf.Geometry.ParametrizationType parType, int degMin = 3,
                     int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3,
                     bool periodic = false)
    {
        _isDone = false;
        int imin = points.LowerRow();
        int imax = points.UpperRow();
        int jmin = points.LowerCol();
        int jmax = points.UpperCol();

        double tol2D = tol3D;

        // first approximate the U isos:
        int add = 1;
        if (periodic)
        {
            add = 2;
        }
        MultiLine line = new(jmax - jmin + 1);
        int i, j;

        for (j = jmin; j <= jmax; j++)
        {
            MultiPointConstraint mp = new(imax - imin + add, 0);
            for (i = imin; i <= imax; i++)
            {
                mp.SetPoint(i, points[i, j]); // OCCT: the grid's own indices (occt-issues.md)
            }
            if (periodic)
            {
                mp.SetPoint(imax + 1, points[1, j]);
            }
            line.SetValue(j, mp);
        }

        int nbit = 2;
        bool useSquares = false;
        if (tol3D <= 1.0e-3)
        {
            useSquares = true;
        }

        BSplineCompute theComputer = new(degMin, degMax, tol3D, tol2D, nbit, true, parType, useSquares);

        SetContinuity(theComputer, continuity);

        if (tol3D <= 0.0)
        {
            theComputer.Interpol(line);
        }
        else
        {
            theComputer.Perform(line);
        }

        MultiBSpCurve theCurve = theComputer.Value();

        int vDegree = theCurve.Degree();
        NCollection_Array1<Point3D> poles = new(1, theCurve.NbPoles());
        NCollection_Array1<double> vKnots = theCurve.Knots();
        NCollection_Array1<int> vMults = theCurve.Multiplicities();

        int nbisosu = imax - imin + add;
        MultiLine line2 = new(nbisosu);

        for (i = 1; i <= nbisosu; i++)
        {
            theCurve.Curve(i, poles);
            MultiPointConstraint mp = new(poles.Upper(), 0);
            for (j = 1; j <= poles.Upper(); j++)
            {
                mp.SetPoint(j, poles[j]);
            }
            line2.SetValue(i, mp);
        }

        // approximate the resulting poles:

        BSplineCompute theComputer2 = new(degMin, degMax, tol3D, tol2D, nbit, true, parType, useSquares);
        if (tol3D <= 0.0)
        {
            if (periodic)
            {
                theComputer2.SetPeriodic(periodic);
            }
            theComputer2.Interpol(line2);
        }
        else
        {
            if (periodic && line2.MultiPointCount > 2)
            {
                theComputer2.SetPeriodic(periodic);
                //
                NCollection_Array1<double> pars = new(1, line2.MultiPointCount);
                BuildParameters(line2, parType, pars);
                math_Vector tang = new(1, 3 * poles.Upper());
                BuildPeriodicTangent(line2, pars, tang);
                int ind = 1;
                theCurve.Curve(ind, poles);
                MultiPointConstraint mp1 = new(poles.Upper(), 0);
                for (j = 1; j <= poles.Upper(); j++)
                {
                    mp1.SetPoint(j, poles[j]);
                    int k = 3 * (j - 1);
                    Vector3D t = new(tang[k + 1], tang[k + 2], tang[k + 3]);
                    mp1.SetTang(j, t);
                }
                line2.SetValue(ind, mp1);
                //
                ind = line2.MultiPointCount;
                theCurve.Curve(ind, poles);
                MultiPointConstraint mp2 = new(poles.Upper(), 0);
                for (j = 1; j <= poles.Upper(); j++)
                {
                    mp2.SetPoint(j, poles[j]);
                    int k = 3 * (j - 1);
                    Vector3D t = new(tang[k + 1], tang[k + 2], tang[k + 3]);
                    mp2.SetTang(j, t);
                }
                line2.SetValue(ind, mp2);
            }
            theComputer2.Perform(line2);
        }

        MultiBSpCurve theCurve2 = theComputer2.Value();

        int uDegree = theCurve2.Degree();
        NCollection_Array1<Point3D> poles2 = new(1, theCurve2.NbPoles());
        NCollection_Array1<double> uKnots = theCurve2.Knots();
        NCollection_Array1<int> uMults = theCurve2.Multiplicities();

        // computing the surface
        NCollection_Array2<Point3D> thePoles = new(1, poles2.Upper(), 1, poles.Upper());
        for (j = 1; j <= poles.Upper(); j++)
        {
            theCurve2.Curve(j, poles2);
            for (i = 1; i <= poles2.Upper(); i++)
            {
                thePoles[i, j] = poles2[i];
            }
        }

        _surface = new BSplineSurface(thePoles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);
        if (periodic && line2.MultiPointCount > 2)
        {
            _surface.SetUPeriodic();
        }

        _isDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising Weight1 x length + Weight2 x
    /// curvature + Weight3 x torsion.
    /// </summary>
    public void Init(NCollection_Array2<Point3D> points, double weight1, double weight2, double weight3,
                     int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        int imin = points.LowerRow();
        int imax = points.UpperRow();
        int jmin = points.LowerCol();
        int jmax = points.UpperCol();

        int nbit = 2;
        if (tol3D <= 1.0e-3)
        {
            nbit = 0;
        }

        // first approximate the U isos:
        int nbPointJ = jmax - jmin + 1;
        int nbPointI = imax - imin + 1;
        int i, j;

        MultiLine line = new(nbPointJ);

        for (j = jmin; j <= jmax; j++)
        {
            MultiPointConstraint mp = new(nbPointI, 0);
            for (i = imin; i <= imax; i++)
            {
                mp.SetPoint(i, points[i, j]); // OCCT: the grid's own indices (occt-issues.md)
            }
            line.SetValue(j, mp);
        }

        NCollection_HArray1<ConstraintCouple> taBofCC = new(1, nbPointJ);
        Constraint constraint = NoConstraint;

        for (i = 1; i <= nbPointJ; ++i)
        {
            ConstraintCouple acc = new(i, constraint);
            taBofCC.SetValue(i, acc);
        }

        Variational variation = new(line, 1, nbPointJ, taBofCC);

        //===================================
        int theMaxSegments = 1000;
        bool theWithMinMax = false;
        //===================================

        variation.SetMaxDegree(degMax);
        variation.SetContinuity(continuity);
        variation.SetMaxSegment(theMaxSegments);

        variation.SetTolerance(tol3D);
        variation.SetWithMinMax(theWithMinMax);
        variation.SetNbIterations(nbit);

        variation.SetCriteriumWeight(weight1, weight2, weight3);

        if (!variation.IsCreated)
        {
            return;
        }

        if (variation.IsOverConstrained)
        {
            return;
        }

        try
        {
            variation.Approximate();
        }
        catch (FailureException)
        {
            return;
        }

        if (!variation.IsDone)
        {
            return;
        }

        MultiBSpCurve theCurve = variation.Value();

        int vDegree = theCurve.Degree();
        NCollection_Array1<Point3D> poles = new(1, theCurve.NbPoles());
        NCollection_Array1<double> vKnots = theCurve.Knots();
        NCollection_Array1<int> vMults = theCurve.Multiplicities();

        MultiLine line2 = new(nbPointI);

        for (i = 1; i <= nbPointI; i++)
        {
            theCurve.Curve(i, poles);
            MultiPointConstraint mp = new(poles.Upper(), 0);
            for (j = 1; j <= poles.Upper(); j++)
            {
                mp.SetPoint(j, poles[j]);
            }
            line2.SetValue(i, mp);
        }

        NCollection_HArray1<ConstraintCouple> taBofCC2 = new(1, nbPointI);

        for (i = 1; i <= nbPointI; ++i)
        {
            ConstraintCouple acc = new(i, constraint);
            taBofCC2.SetValue(i, acc);
        }

        Variational variation2 = new(line2, 1, nbPointI, taBofCC2);

        variation2.SetMaxDegree(degMax);
        variation2.SetContinuity(continuity);
        variation2.SetMaxSegment(theMaxSegments);

        variation2.SetTolerance(tol3D);
        variation2.SetWithMinMax(theWithMinMax);
        variation2.SetNbIterations(nbit);

        variation2.SetCriteriumWeight(weight1, weight2, weight3);

        if (!variation2.IsCreated)
        {
            return;
        }

        if (variation2.IsOverConstrained)
        {
            return;
        }

        try
        {
            variation2.Approximate();
        }
        catch (FailureException)
        {
            return;
        }

        if (!variation2.IsDone)
        {
            return;
        }

        MultiBSpCurve theCurve2 = variation2.Value();

        int uDegree = theCurve2.Degree();
        NCollection_Array1<Point3D> poles2 = new(1, theCurve2.NbPoles());
        NCollection_Array1<double> uKnots = theCurve2.Knots();
        NCollection_Array1<int> uMults = theCurve2.Multiplicities();

        // computing the surface
        NCollection_Array2<Point3D> thePoles = new(1, poles2.Upper(), 1, poles.Upper());
        for (j = 1; j <= poles.Upper(); j++)
        {
            theCurve2.Curve(j, poles2);
            for (i = 1; i <= poles2.Upper(); i++)
            {
                thePoles[i, j] = poles2[i];
            }
        }

        _surface = new BSplineSurface(thePoles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);

        _isDone = true;
    }

    /// <summary>
    /// Interpolates the heights <paramref name="zPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY.
    /// </summary>
    public void Interpolate(NCollection_Array2<double> zPoints, double x0, double dX, double y0, double dY)
    {
        int degMin, degMax;
        degMin = degMax = 3;
        double tol3D = -1.0;
        Continuity cc = C2;
        Init(zPoints, x0, dX, y0, dY, degMin, degMax, cc, tol3D);
    }

    /// <summary>
    /// Approximates the heights <paramref name="zPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY, or
    /// interpolates them if <paramref name="tol3D"/> ≤ 0.
    /// </summary>
    public void Init(NCollection_Array2<double> zPoints, double x0, double dX, double y0, double dY,
                     int degMin = 3, int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        int imin = zPoints.LowerRow();
        int imax = zPoints.UpperRow();
        int jmin = zPoints.LowerCol();
        int jmax = zPoints.UpperCol();
        double length;

        double tol2D = tol3D;

        // first approximate the U isos:
        MultiLine line = new(jmax - jmin + 1);
        math_Vector param = new(jmin, jmax);
        int i, j;
        int vSpan = jmax - jmin;
        //  double X, Y;
        length = dY * (jmax - jmin);

        for (j = jmin; j <= jmax; j++)
        {
            MultiPointConstraint mp = new(0, imax - imin + 1);
            for (i = imin; i <= imax; i++)
            {
                mp.SetPoint2d(i, new Point2D(0.0, zPoints[i, j])); // OCCT: the grid's own indices (occt-issues.md)
            }
            param[j] = (vSpan > 0) ? (double)(j - jmin) / (double)(vSpan) : 0.0;
            line.SetValue(j, mp);
        }

        BSplineCompute theComputer = new(param, degMin, degMax, tol3D, tol2D, 0, true, true);

        SetContinuity(theComputer, continuity);

        if (tol3D <= 0.0)
        {
            theComputer.Interpol(line);
        }
        else
        {
            theComputer.Perform(line);
        }

        MultiBSpCurve theCurve = theComputer.Value();

        int vDegree = theCurve.Degree();
        NCollection_Array1<Point2D> poles = new(1, theCurve.NbPoles());
        int nk = theCurve.Knots().Length();
        NCollection_Array1<double> vKnots = new(1, nk);
        NCollection_Array1<int> vMults = new(1, nk);

        // compute Y values for the poles
        NCollection_Array1<double> yPoles = new(1, poles.Upper());

        // start with a line
        NCollection_Array1<double> tempPoles = new(1, 2);
        NCollection_Array1<double> tempKnots = new(1, 2);
        NCollection_Array1<int> tempMults = new(1, 2);
        tempMults.Init(2);
        tempPoles[1] = y0;
        tempPoles[2] = y0 + length;
        tempKnots[1] = 0.0;
        tempKnots[2] = 1.0;

        // increase the Degree
        NCollection_Array1<double> newTempPoles = new(1, vDegree + 1);
        NCollection_Array1<double> newTempKnots = new(1, 2);
        NCollection_Array1<int> newTempMults = new(1, 2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(1,
                                vDegree,
                                false,
                                1,
                                tempPoles,
                                tempKnots,
                                tempMults,
                                newTempPoles,
                                newTempKnots,
                                newTempMults);

        // insert the Knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(vDegree,
                             false,
                             1,
                             newTempPoles,
                             newTempKnots,
                             newTempMults,
                             theCurve.Knots(),
                             theCurve.Multiplicities(),
                             yPoles,
                             vKnots,
                             vMults,
                             Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk; i++)
        {
            vKnots[i] = y0 + length * vKnots[i];
        }

        int nbisosu = imax - imin + 1;
        MultiLine line2 = new(nbisosu);
        math_Vector param2 = new(1, nbisosu);
        int uSpan = nbisosu - 1;
        length = dX * (imax - imin);

        for (i = 1; i <= nbisosu; i++)
        {
            theCurve.Curve(i, poles);
            MultiPointConstraint mp = new(0, poles.Upper());
            for (j = 1; j <= poles.Upper(); j++)
            {
                mp.SetPoint2d(j, new Point2D(0.0, poles[j].Y));
            }
            param2[i] = (uSpan > 0) ? (double)(i - 1) / (double)(uSpan) : 0.0;
            line2.SetValue(i, mp);
        }

        // approximate the resulting poles:

        BSplineCompute theComputer2 = new(param2, degMin, degMax, tol3D, tol2D, 0, true, true);
        if (tol3D <= 0.0)
        {
            theComputer2.Interpol(line2);
        }
        else
        {
            theComputer2.Perform(line2);
        }

        MultiBSpCurve theCurve2 = theComputer2.Value();

        int uDegree = theCurve2.Degree();
        NCollection_Array1<Point2D> poles2 = new(1, theCurve2.NbPoles());
        int nk2 = theCurve2.Knots().Length();
        NCollection_Array1<double> uKnots = new(1, nk2);
        NCollection_Array1<int> uMults = new(1, nk2);

        // compute X values for the poles
        NCollection_Array1<double> xPoles = new(1, poles2.Upper());

        // start with a line
        tempPoles[1] = x0;
        tempPoles[2] = x0 + length;
        tempKnots[1] = 0.0;
        tempKnots[2] = 1.0;
        tempMults[1] = tempMults[2] = 2;

        // increase the Degree
        NCollection_Array1<double> newTempPoles2 = new(1, uDegree + 1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(1,
                                uDegree,
                                false,
                                1,
                                tempPoles,
                                tempKnots,
                                tempMults,
                                newTempPoles2,
                                newTempKnots,
                                newTempMults);

        // insert the Knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(uDegree,
                             false,
                             1,
                             newTempPoles2,
                             newTempKnots,
                             newTempMults,
                             theCurve2.Knots(),
                             theCurve2.Multiplicities(),
                             xPoles,
                             uKnots,
                             uMults,
                             Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk2; i++)
        {
            uKnots[i] = x0 + length * uKnots[i];
        }

        // creating the surface
        NCollection_Array2<Point3D> thePoles = new(1, poles2.Upper(), 1, poles.Upper());

        for (j = 1; j <= poles.Upper(); j++)
        {
            theCurve2.Curve(j, poles2);
            for (i = 1; i <= poles2.Upper(); i++)
            {
                thePoles[i, j] = thePoles[i, j].WithCoord(xPoles[i], yPoles[j], poles2[i].Y);
            }
        }

        _surface = new BSplineSurface(thePoles, uKnots, vKnots, uMults, vMults, uDegree, vDegree);

        _isDone = true;
    }

    // The switch on the continuity that OCCT writes out in both least-squares Init.
    private static void SetContinuity(BSplineCompute theComputer, Continuity continuity)
    {
        switch (continuity)
        {
            case C0:
                theComputer.SetContinuity(0);
                break;

            case G1:
            case C1:
                theComputer.SetContinuity(1);
                break;

            case G2:
            case C2:
                theComputer.SetContinuity(2);
                break;

            default:
                theComputer.SetContinuity(3);
                break;
        }
    }

    /// <summary>The approximating surface (frozen).</summary>
    /// <exception cref="NotDoneException">The approximation failed.</exception>
    public BSplineSurface Surface()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_PointsToBSplineSurface: Surface not done");
        _surface!.Freeze();
        return _surface;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineSurface&gt;</c>: <see cref="Surface"/>.</summary>
    public static explicit operator BSplineSurface(PointsToBSplineSurface approx) =>
        approx.Surface();

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone => _isDone;
}
