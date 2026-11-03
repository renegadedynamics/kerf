// Port of: src/ModelingData/TKGeomBase/Approx/Approx_ComputeLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Interfaces;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Engine.Approximation.Status;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a MultiLine by a sequence of MultiCurves (Bezier curves): from degree degreemin to degreemax,
/// cutting the line (when cutting is on) until the tolerances are reached. With Squares, the computation is a
/// plain least-squares fit, without iterations on the parameters.
/// </summary>
/// <remarks>
/// <para>
/// The generic form of OCCT's <c>Approx_ComputeLine.gxx</c> (decisions 142 and 202), instantiated by OCCT as
/// AppDef_Compute (over AppDef_MultiLine) and as GeomInt's and BRepApprox's Bezier approximators. The gxx's helper
/// classes (Approx_MyGradient, Approx_ParLeastSquareOfMyGradient, ...) are, in every instantiation, the
/// AppParCurves generics over the same MultiLine and tool, so this class names them directly.
/// </para>
/// <para>
/// A working object (decision 21): the setters are public; <see cref="Value"/>, <see cref="ChangeValue"/>,
/// <see cref="SplineValue"/> and <see cref="Parameters(int)"/> return its own results (shared: don't modify,
/// except through ChangeValue, as in OCCT).
/// </para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="LineTool">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class ComputeLine<MultiLine, LineTool>
    where LineTool : struct, IApproxLineTool<MultiLine>
{
    private readonly NCollection_Sequence<MultiCurve> _multiCurves = new();
    private MultiCurve _theMultiCurve = new();
    private MultiBSpCurve _myspline = new();
    private bool _alldone;
    private bool _tolreached; // OCCT never initialises it before the first approximation; false here
    private ParametrizationType _parametrization;
    private NCollection_HArray1<double>? _parameters;
    private NCollection_HArray1<double>? _myfirstParam;
    private readonly NCollection_Sequence<NCollection_HArray1<double>> _par = new();
    private readonly NCollection_Sequence<double> _tolers3d = new();
    private readonly NCollection_Sequence<double> _tolers2d = new();
    private readonly NCollection_HArray1<ConstraintCouple> _constraints;
    private int _mydegremin;
    private int _mydegremax;
    private double _mytol3d;
    private double _mytol2d;
    private double _currenttol3d;
    private double _currenttol2d;
    private bool _mycut;
    private bool _mysquares;
    private int _myitermax;
    private Constraint _myfirstC;
    private Constraint _mylastC;
    private int _multiLineNb;
    private bool _isClear;

    // OCCT's file-static CheckMultiCurve: false (with the index of a bad point in theIndbad) if the Bezier
    // curve makes a loop that the points don't.
    private static bool CheckMultiCurve(MultiCurve multiCurve, MultiLine line, int indfirst,
                                        int indlast, out int indbad)
    {
        int nbp3d = LineTool.NbP3d(line);
        int nbp2d = LineTool.NbP2d(line);

        const double coeff = 4.0; // 2*2

        indbad = 0;
        if (nbp3d > 1) // only simple cases are analysed
        {
            return true;
        }

        const double minScalProd = -0.9;
        double sqTol3d = global::Kerf.Engine.Foundation.Precision.SquareConfusion();

        // OCCT writes indbads[indcur] for every curve, past its 4 elements when a line has more than 3 curves
        // (2 2d curves beside a 3d one); the port throws there (occt-issues.md).
        Span<int> indbads = stackalloc int[4];
        indbads[1] = indbads[2] = indbads[3] = 0;

        int nbCur = multiCurve.NbCurves();
        bool loopFound = false;

        int nbP3d = Max(nbp3d, 1);
        int nbP2d = Max(nbp2d, 1);

        NCollection_Array1<Point3D> tabP = new(1, nbP3d);
        NCollection_Array1<Point2D> tabP2d = new(1, nbP2d);

        if (multiCurve.Dimension(1) == 3 /*myNbP3d == 1*/)
        {
            NCollection_Array1<Point3D> poles = new(1, multiCurve.NbPoles());
            multiCurve.Curve(1, poles);
            Vector3D firstVec = new(), secondVec;
            int indp = 2;
            while (indp <= poles.Upper())
            {
                firstVec = new Vector3D(poles[1], poles[indp++]);
                double length = firstVec.Magnitude;
                if (length > Constants.Resolution())
                {
                    firstVec /= length;
                    break;
                }
            }
            Point3D midPnt = poles[indp - 1];
            // for (int k = 3; k <= aPoles.Upper(); k++)
            while (indp <= poles.Upper())
            {
                secondVec = new Vector3D(midPnt, poles[indp]);
                double length = secondVec.Magnitude;
                if (length <= Constants.Resolution())
                {
                    indp++;
                    continue;
                }
                secondVec /= length;
                double scalProd = firstVec * secondVec;
                if (scalProd < minScalProd)
                {
                    loopFound = true;
                    break;
                }
                firstVec = secondVec;
                midPnt = poles[indp];
                indp++;
            }
            // Check: may be it is a real loop
            if (loopFound)
            {
                for (int firstInd = indfirst; firstInd <= indlast - 2; firstInd++)
                {
                    LineTool.Value(line, firstInd, tabP);
                    Point3D firstPnt = tabP[1];
                    for (int k = firstInd + 1; k < indlast; k++)
                    {
                        LineTool.Value(line, k, tabP);
                        Point3D pnt1 = tabP[1];
                        LineTool.Value(line, k + 1, tabP);
                        Point3D pnt2 = tabP[1];
                        if (firstPnt.SquareDistanceTo(pnt1) <= sqTol3d || firstPnt.SquareDistanceTo(pnt2) <= sqTol3d)
                        {
                            loopFound = false;
                            break;
                        }
                        Vector3D vec1 = new(firstPnt, pnt1);
                        vec1 = vec1.Normalized();
                        Vector3D vec2 = new(firstPnt, pnt2);
                        vec2 = vec2.Normalized();
                        double scalProd = vec1 * vec2;
                        if (scalProd < minScalProd)
                        {
                            loopFound = false;
                            break;
                        }
                    }
                    if (loopFound == false)
                    {
                        break;
                    }
                }
            }
            if (loopFound)
            {
                // search <indbad>
                double maxSqDist = 0.0;
                double minSqDist = RealLast();
                for (int k = indfirst + 1; k <= indlast; k++)
                {
                    LineTool.Value(line, k - 1, tabP);
                    Point3D prevPnt = tabP[1];
                    LineTool.Value(line, k, tabP);
                    Point3D curPnt = tabP[1];
                    double sqDist = prevPnt.SquareDistanceTo(curPnt);
                    if (sqDist > maxSqDist)
                    {
                        maxSqDist = sqDist;
                        indbads[1] = k;
                    }
                    if (sqDist > Constants.Resolution() && sqDist < minSqDist)
                    {
                        minSqDist = sqDist;
                    }
                }
                double relation = maxSqDist / minSqDist;
                if (relation < coeff)
                {
                    loopFound = false;
                }
                else
                {
                    for (int indcur = 2; indcur <= nbCur; indcur++)
                    {
                        maxSqDist = 0.0;
                        for (int k = indfirst + 1; k <= indlast; k++)
                        {
                            LineTool.Value(line, k - 1, tabP2d);
                            Point2D prevPnt = tabP2d[indcur - 1];
                            LineTool.Value(line, k, tabP2d);
                            Point2D curPnt = tabP2d[indcur - 1];
                            double sqDist = prevPnt.SquareDistanceTo(curPnt);
                            if (sqDist > maxSqDist)
                            {
                                maxSqDist = sqDist;
                                indbads[indcur] = k;
                            }
                        }
                    }
                }
            }
        } // if (myNbP3d == 1)
        else // 2d case
        {
            NCollection_Array1<Point2D> poles2d = new(1, multiCurve.NbPoles());
            multiCurve.Curve(1, poles2d);
            double sqNormToler = Epsilon(1.0) * Epsilon(1.0);
            Vector2D firstVec = new(poles2d[1], poles2d[2]), secondVec = new();
            double vecSqNorm = firstVec.SquareMagnitude;
            if (vecSqNorm < sqNormToler)
            {
                indbad = indfirst + 1;
                return false;
            }

            firstVec /= Math.Sqrt(sqNormToler); // OCCT: divides by the tolerance, not by the norm (occt-issues.md)
            Point2D midPnt = poles2d[2];
            for (int k = 3; k <= poles2d.Upper(); k++)
            {
                secondVec = secondVec.WithXY(poles2d[k].XY() - midPnt.XY());
                vecSqNorm = secondVec.SquareMagnitude;
                if (vecSqNorm < sqNormToler)
                {
                    indbad = indfirst + k - 1;
                    return false;
                }

                secondVec /= Math.Sqrt(vecSqNorm);
                double scalProd = firstVec * secondVec;
                if (scalProd < minScalProd)
                {
                    loopFound = true;
                    break;
                }
                firstVec = secondVec;
                midPnt = poles2d[k];
            }
            // Check: may be it is a real loop
            if (loopFound)
            {
                for (int firstInd = indfirst; firstInd <= indlast - 2; firstInd++)
                {
                    LineTool.Value(line, firstInd, tabP2d);
                    Point2D firstPnt = tabP2d[1];
                    for (int k = firstInd + 1; k < indlast; k++)
                    {
                        LineTool.Value(line, k, tabP2d);
                        Point2D pnt1 = tabP2d[1];
                        LineTool.Value(line, k + 1, tabP2d);
                        Point2D pnt2 = tabP2d[1];
                        if (firstPnt.SquareDistanceTo(pnt1) <= sqTol3d || firstPnt.SquareDistanceTo(pnt2) <= sqTol3d)
                        {
                            loopFound = false;
                            break;
                        }
                        Vector2D vec1 = new(firstPnt, pnt1);
                        vec1 = vec1.Normalized();
                        Vector2D vec2 = new(firstPnt, pnt2);
                        vec2 = vec2.Normalized();
                        double scalProd = vec1 * vec2;
                        if (scalProd < minScalProd)
                        {
                            loopFound = false;
                            break;
                        }
                    }
                    if (loopFound == false)
                    {
                        break;
                    }
                }
            }
            if (loopFound)
            {
                // search <indbad>
                for (int indcur = 1; indcur <= nbCur; indcur++)
                {
                    double maxSqDist = 0.0;
                    double minSqDist = RealLast();
                    for (int k = indfirst + 1; k <= indlast; k++)
                    {
                        LineTool.Value(line, k - 1, tabP2d);
                        Point2D prevPnt = tabP2d[indcur];
                        LineTool.Value(line, k, tabP2d);
                        Point2D curPnt = tabP2d[indcur];
                        double sqDist = prevPnt.SquareDistanceTo(curPnt);
                        if (sqDist > maxSqDist)
                        {
                            maxSqDist = sqDist;
                            indbads[indcur] = k;
                        }
                        if (sqDist > Constants.Resolution() && sqDist < minSqDist)
                        {
                            minSqDist = sqDist;
                        }
                    }
                    double relation = maxSqDist / minSqDist;
                    if (relation < coeff)
                    {
                        loopFound = false;
                    }
                }
            }
        }

        // Define <indbad>
        for (int i = 1; i <= 3; i++)
        {
            if (indbads[i] != 0)
            {
                indbad = indbads[i];
                break;
            }
        }

        if (!loopFound)
        {
            indbad = 0;
        }

        return (!loopFound);
    }

    private void FirstTangencyVector(MultiLine line, int index, math_Vector v)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool ok = false;
        NCollection_Array1<Vector3D> tabV = new(1, mynbP3d);
        NCollection_Array1<Vector2D> tabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV, tabV2d);
        }
        else if (nbP2d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV2d);
        }
        else if (nbP3d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV);
        }

        if (ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = tabV.Lower(); i <= tabV.Upper(); i++)
                {
                    v[j] = tabV[i].X;
                    v[j + 1] = tabV[i].Y;
                    v[j + 2] = tabV[i].Z;
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = tabV2d.Lower(); i <= tabV2d.Upper(); i++)
                {
                    v[j] = tabV2d[i].X;
                    v[j + 1] = tabV2d[i].Y;
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            Constraint firstC, lastC;
            firstC = lastC = PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index, index + 2);
            Parameters(line, index, index + 2, mypar);
            LeastSquare<MultiLine, LineTool> lsq = new(line, index, index + 2, firstC, lastC, mypar, nbpoles);
            MultiCurve c = lsq.BezierValue();

            Point3D myP;
            Vector3D myV;
            Point2D myP2d;
            Vector2D myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                c.D1(i, 0.0, out myP, out myV);
                v[j] = myV.X;
                v[j + 1] = myV.Y;
                v[j + 2] = myV.Z;
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                c.D1(i, 0.0, out myP2d, out myV2d);
                v[j] = myV2d.X;
                v[j + 1] = myV2d.Y;
                j += 2;
            }
        }
    }

    private void LastTangencyVector(MultiLine line, int index, math_Vector v)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool ok = false;
        NCollection_Array1<Vector3D> tabV = new(1, mynbP3d);
        NCollection_Array1<Vector2D> tabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV, tabV2d);
        }
        else if (nbP2d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV2d);
        }
        else if (nbP3d != 0)
        {
            ok = LineTool.Tangency(line, index, tabV);
        }

        if (ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = tabV.Lower(); i <= tabV.Upper(); i++)
                {
                    v[j] = tabV[i].X;
                    v[j + 1] = tabV[i].Y;
                    v[j + 2] = tabV[i].Z;
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = tabV2d.Lower(); i <= tabV2d.Upper(); i++)
                {
                    v[j] = tabV2d[i].X;
                    v[j + 1] = tabV2d[i].Y;
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            Constraint firstC, lastC;
            firstC = lastC = PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index - 2, index);
            Parameters(line, index - 2, index, mypar);
            LeastSquare<MultiLine, LineTool> lsq = new(line, index - 2, index, firstC, lastC, mypar, nbpoles);
            MultiCurve c = lsq.BezierValue();

            Point3D myP;
            Vector3D myV;
            Point2D myP2d;
            Vector2D myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                c.D1(i, 1.0, out myP, out myV);
                v[j] = myV.X;
                v[j + 1] = myV.Y;
                v[j + 2] = myV.Z;
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                c.D1(i, 1.0, out myP2d, out myV2d);
                v[j] = myV2d.X;
                v[j + 1] = myV2d.Y;
                j += 2;
            }
        }
    }

    private static double SearchFirstLambda(MultiLine line, math_Vector theParam, math_Vector v, int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        Point3D p1, p2;
        Point2D p12d, p22d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> tabP1 = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<Point2D> tabP12d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(line, index, tabP1, tabP12d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(line, index, tabP12d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(line, index, tabP1);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(line, index + 1, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(line, index + 1, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(line, index + 1, tabP2);
        }

        double u1 = theParam[index], u2 = theParam[index + 1];
        double lambda, s;
        int low = v.Lower();

        if (nbP3d != 0)
        {
            p1 = tabP1[1];
            p2 = tabP2[1];
            Vector3D p1P2 = new(p1, p2), myV = new();
            myV = myV.WithCoord(v[low], v[low + 1], v[low + 2]);
            lambda = (p1P2.Magnitude) / (myV.Magnitude * (u2 - u1));
            s = (p1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            p12d = tabP12d[1];
            p22d = tabP22d[1];
            Vector2D p1P2 = new(p12d, p22d), myV = new();
            myV = myV.WithCoord(v[low], v[low + 1]);
            lambda = (p1P2.Magnitude) / (myV.Magnitude * (u2 - u1));
            s = (p1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        return (s * lambda);
    }

    private static double SearchLastLambda(MultiLine line, math_Vector theParam, math_Vector v, int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        Point3D p1, p2;
        Point2D p12d, p22d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> tabP = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(line, index - 1, tabP, tabP2d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(line, index - 1, tabP2d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(line, index - 1, tabP);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(line, index, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(line, index, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(line, index, tabP2);
        }

        double u1 = theParam[index - 1], u2 = theParam[index];
        double lambda, s;
        int low = v.Lower();

        if (nbP3d != 0)
        {
            p1 = tabP[1];
            p2 = tabP2[1];
            Vector3D p1P2 = new(p1, p2), myV = new();
            myV = myV.WithCoord(v[low], v[low + 1], v[low + 2]);
            lambda = (p1P2.Magnitude) / (myV.Magnitude * (u2 - u1));
            s = (p1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            p12d = tabP2d[1];
            p22d = tabP22d[1];
            Vector2D p1P2 = new(p12d, p22d), myV = new();
            myV = myV.WithCoord(v[low], v[low + 1]);
            lambda = (p1P2.Magnitude) / (myV.Magnitude * (u2 - u1));
            s = (p1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }

        return (s * lambda);
    }

    /// <summary>
    /// Approximates <paramref name="line"/> from the parameters <paramref name="parameters"/> until the tolerances
    /// are reached, from degree <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if
    /// <paramref name="cutting"/>. With <paramref name="squares"/>, without iterations.
    /// </summary>
    public ComputeLine(MultiLine line, math_Vector parameters, int degreemin = 4, int degreemax = 8,
                              double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06, int nbIterations = 5,
                              bool cutting = true, bool squares = false)
    {
        _multiLineNb = 0;
        _isClear = false;
        _myfirstParam = new NCollection_HArray1<double>(parameters.Lower(), parameters.Upper());
        for (int i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            _myfirstParam.SetValue(i, parameters[i]);
        }
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _parametrization = IsoParametric;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _alldone = false;
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        Perform(line);
    }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="parameters"/>.</summary>
    public ComputeLine(math_Vector parameters, int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03,
                              double tolerance2d = 1.0e-06, int nbIterations = 5, bool cutting = true,
                              bool squares = false)
    {
        _multiLineNb = 0;
        _isClear = false;
        _myfirstParam = new NCollection_HArray1<double>(parameters.Lower(), parameters.Upper());
        for (int i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            _myfirstParam.SetValue(i, parameters[i]);
        }
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _parametrization = IsoParametric;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _alldone = false;
    }

    /// <summary>Initialises the fields of the algorithm; <see cref="Perform"/> runs it.</summary>
    public ComputeLine(int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03,
                              double tolerance2d = 1.0e-06, int nbIterations = 5, bool cutting = true,
                              ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _multiLineNb = 0;
        _isClear = false;
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _parametrization = parametrization;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        _alldone = false;
    }

    /// <summary>
    /// Approximates <paramref name="line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if
    /// <paramref name="cutting"/>. With <paramref name="squares"/>, without iterations.
    /// </summary>
    public ComputeLine(MultiLine line, int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-3,
                              double tolerance2d = 1.0e-6, int nbIterations = 5, bool cutting = true,
                              ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _multiLineNb = 0;
        _isClear = false;
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _alldone = false;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _parametrization = parametrization;
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;

        Perform(line);
    }

    /// <summary>Runs the algorithm on <paramref name="line"/> after the fields have been initialised.</summary>
    public void Perform(MultiLine line)
    {
        if (!_isClear)
        {
            _multiCurves.Clear();
            _par.Clear();
            _tolers3d.Clear();
            _tolers2d.Clear();
            _multiLineNb = 0;
        }
        else
        {
            _isClear = false;
        }

        int i, nbp, thefirstpt, thelastpt, oldlastpt;
        bool finish = false, begin = true, ok = false, goUp = false, interpol;
        double thetol3d, thetol2d;
        Status myStatus;
        thefirstpt = LineTool.FirstPoint(line);
        thelastpt = LineTool.LastPoint(line);
        int myfirstpt = thefirstpt;
        int mylastpt = thelastpt;

        ConstraintCouple myCouple1 = new(myfirstpt, _myfirstC);
        ConstraintCouple myCouple2 = new(mylastpt, _mylastC);
        _constraints.SetValue(1, myCouple1);
        _constraints.SetValue(2, myCouple2);

        math_Vector theParam = new(thefirstpt, thelastpt);

        if (!_mycut)
        {
            if (_myfirstParam is null)
            {
                Parameters(line, thefirstpt, thelastpt, theParam);
            }
            else
            {
                for (i = _myfirstParam.Lower(); i <= _myfirstParam.Upper(); i++)
                {
                    theParam[i + thefirstpt - 1] = _myfirstParam.Value(i);
                }
            }
            _theMultiCurve = new MultiCurve();
            MultiLine otherLine0 = default!;
            bool isOtherLine0Made = false;
            int indbad = 0;
            _alldone = Compute(line, myfirstpt, mylastpt, theParam, out thetol3d, out thetol2d, out indbad);
            if (indbad != 0)
            {
                isOtherLine0Made = LineTool.MakeMLOneMorePoint(line, myfirstpt, mylastpt, indbad, out otherLine0);
            }
            if (isOtherLine0Made)
            {
                _isClear = true;
                //++myMultiLineNb;
                Perform(otherLine0);
                _alldone = true;
            }
            if (!_alldone && _theMultiCurve.NbCurves() > 0)
            {
                _multiCurves.Append(new MultiCurve(_theMultiCurve));
                _tolers3d.Append(_currenttol3d);
                _tolers2d.Append(_currenttol2d);
                int mylen = mylastpt - myfirstpt + 1;
                int myParLen = _parameters!.Length();
                int len = (myParLen > mylen) ? myParLen : mylen;
                NCollection_HArray1<double> thePar = new(myfirstpt, myfirstpt + len - 1);
                for (i = 0; i < len; i++)
                {
                    thePar.SetValue(myfirstpt + i, _parameters.Value(_parameters.Lower() + i));
                }
                _par.Append(thePar);
            }
        }
        else
        {
            while (!finish)
            {
                oldlastpt = mylastpt;
                // Management of the multiline splitting for approximation:
                if (!begin)
                {
                    if (!goUp)
                    {
                        if (ok)
                        {
                            // Calcul de la partie a approximer.
                            myfirstpt = mylastpt;
                            mylastpt = thelastpt;
                            if (myfirstpt == thelastpt)
                            {
                                finish = true;
                                _alldone = true;
                                return;
                            }
                        }
                        else
                        {
                            nbp = mylastpt - myfirstpt + 1;
                            myStatus = LineTool.WhatStatus(line, myfirstpt, mylastpt);
                            if (myStatus == NoPointsAdded && nbp <= _mydegremax + 1)
                            {
                                interpol = ComputeCurve(line, myfirstpt, mylastpt);
                                if (interpol)
                                {
                                    if (mylastpt == thelastpt)
                                    {
                                        finish = true;
                                        _alldone = true;
                                        return;
                                    }
                                }
                            }
                            mylastpt = (int)((myfirstpt + mylastpt) / 2);
                        }
                    }
                    goUp = false;
                }

                // Verification du nombre de points restants par rapport au degre
                // demande.
                // ==============================================================
                nbp = mylastpt - myfirstpt + 1;
                myStatus = LineTool.WhatStatus(line, myfirstpt, mylastpt);
                if (nbp <= _mydegremax + 5)
                {
                    // Rajout necessaire de points si possible.
                    // ========================================
                    goUp = false;
                    ok = true;
                    if (myStatus == PointsAdded)
                    {
                        // Appel recursif du decoupage:
                        goUp = true;

                        MultiLine otherLine1 = LineTool.MakeMLBetween(line, myfirstpt, mylastpt, nbp - 1);

                        int nbpdsotherligne = LineTool.FirstPoint(otherLine1) - LineTool.LastPoint(otherLine1);

                        //-- If MakeML failed, return an empty line
                        if ((nbpdsotherligne == 0) || _multiLineNb >= 3)
                        {
                            //-- cout<<" ** ApproxComputeLine MakeML Echec ** LBR lbr "<<endl;
                            if (myfirstpt == mylastpt)
                            {
                                break; // Pour etre sur de ne pas
                            }
                            // planter la station !!
                            myCouple1.SetIndex(myfirstpt);
                            myCouple2.SetIndex(mylastpt);
                            _constraints.SetValue(1, myCouple1);
                            _constraints.SetValue(2, myCouple2);

                            math_Vector param = new(myfirstpt, mylastpt);
                            ParametrizationType savePar = _parametrization;
                            _parametrization = IsoParametric;
                            Parameters(line, myfirstpt, mylastpt, param);
                            _theMultiCurve = new MultiCurve();
                            MultiLine otherLine2 = default!;
                            bool isOtherLine2Made = false;
                            int indbad = 0;
                            ok = Compute(line, myfirstpt, mylastpt, param, out thetol3d, out thetol2d, out indbad);
                            if (indbad != 0)
                            {
                                isOtherLine2Made = LineTool.MakeMLOneMorePoint(line, myfirstpt, mylastpt, indbad, out otherLine2);
                            }
                            if (isOtherLine2Made)
                            {
                                _isClear = true;
                                //++myMultiLineNb;
                                _parametrization = savePar;
                                Perform(otherLine2);
                                ok = true;
                            }

                            if (!ok)
                            {
                                double tt3d = _currenttol3d, tt2d = _currenttol2d;
                                NCollection_HArray1<double>? saveParameters = _parameters;
                                MultiCurve saveMultiCurve = _theMultiCurve;

                                if (savePar != IsoParametric)
                                {
                                    _parametrization = savePar;
                                }
                                else
                                {
                                    _parametrization = ChordLength;
                                }

                                Parameters(line, myfirstpt, mylastpt, param);
                                isOtherLine2Made = false;
                                indbad = 0;
                                ok = Compute(line, myfirstpt, mylastpt, param, out thetol3d, out thetol2d, out indbad);
                                if (indbad != 0)
                                {
                                    isOtherLine2Made = LineTool.MakeMLOneMorePoint(line, myfirstpt, mylastpt, indbad,
                                                                                   out otherLine2);
                                }
                                if (isOtherLine2Made)
                                {
                                    _isClear = true;
                                    //++myMultiLineNb;
                                    Perform(otherLine2);
                                    ok = true;
                                }

                                if (!ok && tt3d <= _currenttol3d && tt2d <= _currenttol2d)
                                {
                                    _currenttol3d = tt3d;
                                    _currenttol2d = tt2d;
                                    _parameters = saveParameters;
                                    _theMultiCurve = saveMultiCurve;
                                }
                            }
                            _parametrization = savePar;
                            if (myfirstpt == thelastpt)
                            {
                                finish = true;
                                _alldone = true;
                                return;
                            }

                            oldlastpt = mylastpt;
                            if (!ok)
                            {
                                _tolreached = false;
                                if (_theMultiCurve.NbCurves() == 0)
                                {
                                    _multiCurves.Clear();
                                    return;
                                }
                                MultiLine otherLine3 = default!;
                                bool isOtherLine3Made = false;
                                int indbad2 = 0;
                                if (!CheckMultiCurve(_theMultiCurve, line, myfirstpt, mylastpt, out indbad2))
                                {
                                    isOtherLine3Made = LineTool.MakeMLOneMorePoint(line, myfirstpt, mylastpt, indbad2,
                                                                                   out otherLine3);
                                }
                                if (isOtherLine3Made)
                                {
                                    _isClear = true;
                                    //++myMultiLineNb;
                                    Perform(otherLine3);
                                    myfirstpt = mylastpt;
                                    mylastpt = thelastpt;
                                }
                                else
                                {
                                    _multiCurves.Append(new MultiCurve(_theMultiCurve));
                                    _tolers3d.Append(_currenttol3d);
                                    _tolers2d.Append(_currenttol2d);
                                    int mylen = oldlastpt - myfirstpt + 1;
                                    int myParLen = _parameters!.Length();
                                    int len = (myParLen > mylen) ? myParLen : mylen;
                                    NCollection_HArray1<double> thePar = new(myfirstpt, myfirstpt + len - 1);
                                    for (i = 0; i < len; i++)
                                    {
                                        thePar.SetValue(myfirstpt + i, _parameters.Value(_parameters.Lower() + i));
                                    }
                                    _par.Append(thePar);
                                }
                            }
                            myfirstpt = oldlastpt;
                            mylastpt = thelastpt;
                        }
                        else
                        {
                            _isClear = true;
                            ++_multiLineNb;
                            Perform(otherLine1);
                            myfirstpt = mylastpt;
                            mylastpt = thelastpt;
                        }
                    }

                    if (myStatus == NoPointsAdded && !begin)
                    {
                        // On rend la meilleure approximation obtenue precedemment.
                        // ========================================================
                        goUp = true;
                        _tolreached = false;
                        if (_theMultiCurve.NbCurves() == 0)
                        {
                            _multiCurves.Clear();
                            return;
                        }
                        _multiCurves.Append(new MultiCurve(_theMultiCurve));
                        _tolers3d.Append(_currenttol3d);
                        _tolers2d.Append(_currenttol2d);
                        int mylen = oldlastpt - myfirstpt + 1;
                        int myParLen = _parameters!.Length();
                        int len = (myParLen > mylen) ? myParLen : mylen;
                        NCollection_HArray1<double> thePar = new(myfirstpt, myfirstpt + len - 1);
                        for (i = 0; i < len; i++)
                        {
                            thePar.SetValue(myfirstpt + i, _parameters.Value(_parameters.Lower() + i));
                        }
                        _par.Append(thePar);

                        myfirstpt = oldlastpt;
                        mylastpt = thelastpt;
                    }
                    else if (myStatus == NoApproximation)
                    {
                        // No approximation is done between myfirstpt and mylastpt.
                        // ===========================================================
                        // Store to be able to inform the user.
                        goUp = true;
                        myfirstpt = mylastpt;
                        mylastpt = thelastpt;
                    }
                }

                if (myfirstpt == thelastpt)
                {
                    finish = true;
                    _alldone = true;
                    return;
                }
                if (!goUp)
                {
                    if (myfirstpt == mylastpt)
                    {
                        break; // Pour etre sur de ne pas
                    }
                    // planter la station !!
                    myCouple1.SetIndex(myfirstpt);
                    myCouple2.SetIndex(mylastpt);
                    _constraints.SetValue(1, myCouple1);
                    _constraints.SetValue(2, myCouple2);

                    // Calculation of parameters on this new interval.
                    // Retrieve the initial parameters during splitting.

                    math_Vector param = new(myfirstpt, mylastpt);
                    if (begin)
                    {
                        if (_myfirstParam is null)
                        {
                            Parameters(line, myfirstpt, mylastpt, param);
                        }
                        else
                        {
                            for (i = _myfirstParam.Lower(); i <= _myfirstParam.Upper(); i++)
                            {
                                param[i] = _myfirstParam.Value(i);
                            }
                            _myfirstParam = null;
                        }
                        theParam.Initialized(param);
                        begin = false;
                    }
                    else
                    {
                        double pfirst = theParam.Value(myfirstpt);
                        double plast = theParam.Value(mylastpt);
                        for (i = myfirstpt; i <= mylastpt; i++)
                        {
                            param[i] = (theParam.Value(i) - pfirst) / (plast - pfirst);
                        }
                    }

                    _theMultiCurve = new MultiCurve();
                    int indbad = 0;
                    ok = Compute(line, myfirstpt, mylastpt, param, out thetol3d, out thetol2d, out indbad);
                    if (myfirstpt == thelastpt)
                    {
                        finish = true;
                        _alldone = true;
                        return;
                    }
                }
            }
        }
    }

    /// <summary>
    /// The parameters of the points of the MultiCurve <paramref name="index"/>: the object's own array (shared:
    /// don't modify).
    /// </summary>
    public NCollection_Array1<double> Parameters(int index = 1) => (_par.Value(index)).Array1();

    /// <summary>The number of MultiCurves of the approximation.</summary>
    public int MultiCurveCount => _multiCurves.Length();

    /// <summary>
    /// The MultiCurve <paramref name="index"/> of the approximation: the stored one, which a caller may modify in
    /// place, as through OCCT's non-const reference.
    /// </summary>
    public MultiCurve ChangeValue(int index = 1) => _multiCurves.Value(index);

    /// <summary>The MultiCurve <paramref name="index"/> of the approximation: the stored one (shared: don't modify).</summary>
    public MultiCurve Value(int index = 1) => _multiCurves.Value(index);

    /// <summary>
    /// The MultiCurves joined into one MultiBSpCurve, computed on each call; the object's own result (shared:
    /// don't modify).
    /// </summary>
    public MultiBSpCurve SplineValue()
    {
        MCurvesToBSpCurve trans = new();
        trans.Perform(_multiCurves);
        _myspline = new MultiBSpCurve(trans.Value());
        return _myspline;
    }

    // Computes new parameters between firstP and lastP.
    private void Parameters(MultiLine line, int firstP, int lastP, math_Vector theParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;

        if (_parametrization == ChordLength || _parametrization == Centripetal)
        {
            nbP3d = LineTool.NbP3d(line);
            nbP2d = LineTool.NbP2d(line);
            int mynbP3d = nbP3d, mynbP2d = nbP2d;
            if (nbP3d == 0)
            {
                mynbP3d = 1;
            }
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }

            theParameters[firstP] = 0.0;
            NCollection_Array1<Point3D> tabP = new(1, mynbP3d);
            NCollection_Array1<Point3D> tabPP = new(1, mynbP3d);
            NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);
            NCollection_Array1<Point2D> tabPP2d = new(1, mynbP2d);

            for (i = firstP + 1; i <= lastP; i++)
            {
                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(line, i - 1, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(line, i - 1, tabP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(line, i - 1, tabP);
                }

                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(line, i, tabPP, tabPP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(line, i, tabPP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(line, i, tabPP);
                }
                dist = 0;
                for (j = 1; j <= nbP3d; j++)
                {
                    Point3D p1 = tabP[j], p2 = tabPP[j];
                    dist += p2.SquareDistanceTo(p1);
                }
                for (j = 1; j <= nbP2d; j++)
                {
                    Point2D p12d = tabP2d[j], p22d = tabPP2d[j];

                    dist += p22d.SquareDistanceTo(p12d);
                }

                dist = Math.Sqrt(dist);
                if (_parametrization == ChordLength)
                {
                    theParameters[i] = theParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    theParameters[i] = theParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP; i <= lastP; i++)
            {
                theParameters[i] /= theParameters[lastP];
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                theParameters[i] = ((double)i - firstP) / ((double)lastP - (double)firstP);
            }
        }
    }

    private bool Compute(MultiLine line, int fpt, int lpt, math_Vector para, out double theTol3d, out double theTol2d,
                         out int indbad)
    {
        indbad = 0;
        // OCCT leaves the tolerances unchanged when no degree is tried; the port's out parameters start at 0.
        theTol3d = 0.0;
        theTol2d = 0.0;
        int deg, i;
        bool mydone;
        double fv;
        int nbp = lpt - fpt + 1;

        math_Vector parSav = new(para.Lower(), para.Upper());
        for (i = para.Lower(); i <= para.Upper(); i++)
        {
            parSav[i] = para[i];
        }
        int mdegmax = _mydegremax;
        if (nbp < mdegmax + 5 && _mycut)
        {
            mdegmax = nbp - 5;
        }
        if (mdegmax < _mydegremin)
        {
            mdegmax = _mydegremin;
        }

        _currenttol3d = _currenttol2d = RealLast();
        for (deg = Min(nbp - 1, _mydegremin); deg <= mdegmax; deg++)
        {
            MultiCurve mySCU;
            if (_mysquares)
            {
                LeastSquare<MultiLine, LineTool> sq = new(line, fpt, lpt, _myfirstC, _mylastC, para, deg + 1);
                mydone = sq.IsDone;
                mySCU = sq.BezierValue();
                sq.Error(out fv, out theTol3d, out theTol2d);
            }
            else
            {
                Gradient<MultiLine, LineTool> grad =
                    new(line, fpt, lpt, _constraints, para, deg, _mytol3d, _mytol2d, _myitermax);
                mydone = grad.IsDone;
                mySCU = grad.Value();
                if (mySCU.NbCurves() == 0)
                {
                    continue;
                }
                theTol3d = grad.MaxError3d();
                theTol2d = grad.MaxError2d();
            }
            double uu1 = para[para.Lower()], uu2;
            bool restau = false;
            for (i = para.Lower() + 1; i <= para.Upper(); i++)
            {
                uu2 = para[i];
                if (uu2 <= uu1)
                {
                    restau = true;
                    break;
                }
                uu1 = uu2;
            }
            if (restau)
            {
                for (i = para.Lower(); i <= para.Upper(); i++)
                {
                    para[i] = parSav[i];
                }
            }
            if (mydone)
            {
                if (theTol3d <= _mytol3d && theTol2d <= _mytol2d)
                {
                    // Stockage de la multicurve approximee.
                    _tolreached = true;
                    if (!CheckMultiCurve(mySCU, line, fpt, lpt, out indbad))
                    {
                        return false;
                    }
                    else
                    {
                        _multiCurves.Append(mySCU);
                        // Storage of parameters of the approximated MultiLine part:
                        // A ameliorer !! (bq trop de recopies)
                        NCollection_HArray1<double> thePar = new(para.Lower(), para.Upper());
                        for (i = para.Lower(); i <= para.Upper(); i++)
                        {
                            thePar.SetValue(i, para[i]);
                        }
                        _par.Append(thePar);
                        _tolers3d.Append(theTol3d);
                        _tolers2d.Append(theTol2d);
                        return true;
                    }
                }
            }

            if (theTol3d <= _currenttol3d && theTol2d <= _currenttol2d)
            {
                _theMultiCurve = mySCU;
                _currenttol3d = theTol3d;
                _currenttol2d = theTol2d;
                _parameters = new NCollection_HArray1<double>(para.Lower(), para.Upper());
                for (i = para.Lower(); i <= para.Upper(); i++)
                {
                    _parameters.SetValue(i, para[i]);
                }
            }
        }

        return false;
    }

    private bool ComputeCurve(MultiLine line, int firstpt, int lastpt)
    {
        int i, j, nbP3d, nbP2d, deg;
        Vector3D v13d, v23d;
        Vector2D v12d, v22d;
        Point3D p1, p2;
        Point2D p12d, p22d;
        bool tangent1, tangent2, mydone = false;
        int myfirstpt = firstpt, mylastpt = lastpt;
        int nbp = lastpt - firstpt + 1;
        math_Vector para = new(firstpt, lastpt);

        Parameters(line, firstpt, lastpt, para);

        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }

        NCollection_Array1<Vector3D> tabV1 = new(1, mynbP3d), tabV2 = new(1, mynbP3d);
        NCollection_Array1<Point3D> tabP1 = new(1, mynbP3d), tabP2 = new(1, mynbP3d), tabP = new(1, mynbP3d);
        NCollection_Array1<Vector2D> tabV12d = new(1, mynbP2d), tabV22d = new(1, mynbP2d);
        NCollection_Array1<Point2D> tabP12d = new(1, mynbP2d), tabP22d = new(1, mynbP2d), tabP2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(line, myfirstpt, tabP1, tabP12d);
            LineTool.Value(line, mylastpt, tabP2, tabP22d);
            tangent1 = LineTool.Tangency(line, myfirstpt, tabV1, tabV12d);
            tangent2 = LineTool.Tangency(line, mylastpt, tabV2, tabV22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(line, myfirstpt, tabP12d);
            LineTool.Value(line, mylastpt, tabP22d);
            tangent1 = LineTool.Tangency(line, myfirstpt, tabV12d);
            tangent2 = LineTool.Tangency(line, mylastpt, tabV22d);
        }
        else
        {
            LineTool.Value(line, myfirstpt, tabP1);
            LineTool.Value(line, mylastpt, tabP2);
            tangent1 = LineTool.Tangency(line, myfirstpt, tabV1);
            tangent2 = LineTool.Tangency(line, mylastpt, tabV2);
        }
        if (nbp == 2)
        {
            // If there are only 2 points, we still verify that the tangents are
            // aligned (OCCT only reports it in debug builds).
            if (tangent1)
            {
                for (i = 1; i <= nbP3d; i++)
                {
                    Vector3D pVec = new(tabP1[i], tabP2[i]);
                    v13d = tabV1[i];
                    if (!pVec.IsParallel(v13d, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        break;
                    }
                }
                for (i = 1; i <= nbP2d; i++)
                {
                    Vector2D pVec2d = new(tabP12d[i], tabP22d[i]);
                    v12d = tabV12d[i];
                    if (!pVec2d.IsParallel(v12d, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        break;
                    }
                }
            }

            if (tangent2)
            {
                for (i = 1; i <= nbP3d; i++)
                {
                    Vector3D pVec = new(tabP1[i], tabP2[i]);
                    v23d = tabV2[i];
                    if (!pVec.IsParallel(v23d, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        break;
                    }
                }
                for (i = 1; i <= nbP2d; i++)
                {
                    Vector2D pVec2d = new(tabP12d[i], tabP22d[i]);
                    v22d = tabV22d[i];
                    if (!pVec2d.IsParallel(v22d, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        break;
                    }
                }
            }

            MultiCurve mySCU2 = new(_mydegremin + 1);
            if (nbP3d != 0 && nbP2d != 0)
            {
                MultiPoint mPole1 = new(tabP1, tabP12d);
                MultiPoint mPole2 = new(tabP2, tabP22d);
                mySCU2.SetValue(1, mPole1);
                mySCU2.SetValue(_mydegremin + 1, mPole2);
                for (i = 2; i <= _mydegremin; i++)
                {
                    for (j = 1; j <= nbP3d; j++)
                    {
                        p1 = tabP1[j];
                        p2 = tabP2[j];
                        tabP[j] = tabP[j].WithXYZ(p1.XYZ() + (i - 1) * (p2.XYZ() - p1.XYZ()) / _mydegremin);
                    }
                    for (j = 1; j <= nbP2d; j++)
                    {
                        p12d = tabP12d[j];
                        p22d = tabP22d[j];
                        tabP2d[j] = tabP2d[j].WithXY(p12d.XY() + (i - 1) * (p22d.XY() - p12d.XY()) / _mydegremin);
                    }
                    MultiPoint mPole = new(tabP, tabP2d);
                    mySCU2.SetValue(i, mPole);
                }
            }
            else if (nbP3d != 0)
            {
                MultiPoint mPole1 = new(tabP1);
                MultiPoint mPole2 = new(tabP2);
                mySCU2.SetValue(1, mPole1);
                mySCU2.SetValue(_mydegremin + 1, mPole2);
                for (i = 2; i <= _mydegremin; i++)
                {
                    for (j = 1; j <= nbP3d; j++)
                    {
                        p1 = tabP1[j];
                        p2 = tabP2[j];
                        tabP[j] = tabP[j].WithXYZ(p1.XYZ() + (i - 1) * (p2.XYZ() - p1.XYZ()) / _mydegremin);
                    }
                    MultiPoint mPole = new(tabP);
                    mySCU2.SetValue(i, mPole);
                }
            }
            else if (nbP2d != 0)
            {
                MultiPoint mPole1 = new(tabP12d);
                MultiPoint mPole2 = new(tabP22d);
                mySCU2.SetValue(1, mPole1);
                mySCU2.SetValue(_mydegremin + 1, mPole2);
                for (i = 2; i <= _mydegremin; i++)
                {
                    for (j = 1; j <= nbP2d; j++)
                    {
                        p12d = tabP12d[j];
                        p22d = tabP22d[j];
                        tabP2d[j] = tabP2d[j].WithXY(p12d.XY() + (i - 1) * (p22d.XY() - p12d.XY()) / _mydegremin);
                    }
                    MultiPoint mPole = new(tabP2d);
                    mySCU2.SetValue(i, mPole);
                }
            }
            mydone = true;
            // Stockage de la multicurve approximee.
            _tolreached = true;
            _multiCurves.Append(mySCU2);
            NCollection_HArray1<double> thePar2 = new(para.Lower(), para.Upper());
            for (i = para.Lower(); i <= para.Upper(); i++)
            {
                thePar2.SetValue(i, para[i]);
            }
            _par.Append(thePar2);
            _tolers3d.Append(global::Kerf.Engine.Foundation.Precision.Confusion());
            _tolers2d.Append(global::Kerf.Engine.Foundation.Precision.PConfusion());
            return mydone;
        }

        // with the tangents.
        deg = nbp + 1;
        MultiCurve mySCU;
        Constraint cons = TangencyPoint;
        double lambda1, lambda2;
        math_Vector v1 = new(1, nbP3d * 3 + nbP2d * 2);
        math_Vector v2 = new(1, nbP3d * 3 + nbP2d * 2);
        FirstTangencyVector(line, myfirstpt, v1);
        lambda1 = SearchFirstLambda(line, para, v1, myfirstpt);

        LastTangencyVector(line, mylastpt, v2);
        lambda2 = SearchLastLambda(line, para, v2, mylastpt);

        LeastSquare<MultiLine, LineTool> lsq = new(line, myfirstpt, mylastpt, cons, cons, para, deg + 1);

        lambda1 = lambda1 / deg;
        lambda2 = lambda2 / deg;
        lsq.Perform(para, v1, v2, lambda1, lambda2);
        mydone = lsq.IsDone;
        mySCU = lsq.BezierValue();

        if (mydone)
        {
            double fv, theTol3d, theTol2d;
            lsq.Error(out fv, out theTol3d, out theTol2d);

            // Stockage de la multicurve approximee.
            _tolreached = true;
            _multiCurves.Append(mySCU);
            NCollection_HArray1<double> thePar = new(para.Lower(), para.Upper());
            for (i = para.Lower(); i <= para.Upper(); i++)
            {
                thePar.SetValue(i, para[i]);
            }
            _par.Append(thePar);
            _tolers3d.Append(theTol3d);
            _tolers2d.Append(theTol2d);
            return true;
        }
        return mydone;
    }

    /// <summary>Initialises the fields of the algorithm.</summary>
    public void Init(int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06,
                     int nbIterations = 5, bool cutting = true,
                     ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _parametrization = parametrization;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
    }

    /// <summary>Changes the degrees of the approximation.</summary>
    public void SetDegrees(int degreemin, int degreemax)
    {
        _mydegremin = degreemin;
        _mydegremax = degreemax;
    }

    /// <summary>Changes the tolerances of the approximation.</summary>
    public void SetTolerances(double tolerance3d, double tolerance2d)
    {
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
    }

    /// <summary>Changes the constraints at the first and the last points.</summary>
    public void SetConstraints(Constraint firstC, Constraint lastC)
    {
        _myfirstC = firstC;
        _mylastC = lastC;
    }

    /// <summary>False if, during the approximation, the line tool returned NoApproximation when more points were needed.</summary>
    public bool IsAllApproximated => _alldone;

    /// <summary>False if the line tool returned NoPointsAdded.</summary>
    public bool IsToleranceReached => _tolreached;

    /// <summary>The 3d and 2d tolerances of the MultiCurve <paramref name="index"/>.</summary>
    public void Error(int index, out double tol3d, out double tol2d)
    {
        tol3d = _tolers3d.Value(index);
        tol2d = _tolers2d.Value(index);
    }

    /// <summary>The type of parametrisation.</summary>
    public ParametrizationType Parametrization() => _parametrization;
}
