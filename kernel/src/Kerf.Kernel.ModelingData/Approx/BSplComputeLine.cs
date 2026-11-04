// Port of: src/ModelingData/TKGeomBase/Approx/Approx_BSplComputeLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Interfaces;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.ParametrizationType;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a MultiLine by one MultiBSpCurve: from degree degreemin to degreemax, adding knots (when cutting
/// is on) until the tolerances are reached. With Squares, the computation is a plain least-squares fit, without
/// iterations on the parameters.
/// </summary>
/// <remarks>
/// <para>
/// The generic form of OCCT's <c>Approx_BSplComputeLine.gxx</c> (decisions 142 and 202), instantiated by OCCT as
/// AppDef_BSplineCompute (over AppDef_MultiLine), and as GeomInt's and BRepApprox's BSpline approximators. The
/// gxx's helper classes (Approx_MyBSplGradient, Approx_BSpParLeastSquareOfMyBSplGradient, Approx_MyGradientbis,
/// ...) are, in every instantiation, the AppParCurves generics over the same MultiLine and tool, so this class
/// names them directly.
/// </para>
/// <para>A working object (decision 21): the setters are public, and <see cref="Value"/> and
/// <see cref="Parameters()"/> return its own results (shared: don't modify).</para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="LineTool">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class BSplComputeLine<MultiLine, LineTool>
    where LineTool : struct, IApproxLineTool<MultiLine>
{
    private MultiBSpCurve _theMultiBSpCurve = new();
    private bool _alldone;
    private bool _tolreached;
    private global::Kerf.Geometry.ParametrizationType _par;
    private NCollection_HArray1<double>? _parameters;
    private NCollection_HArray1<double>? _myfirstParam;
    private NCollection_HArray1<double>? _myknots;
    private NCollection_HArray1<int>? _mymults;
    private bool _myhasknots;
    private bool _myhasmults;
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
    private Constraint _realfirstC;
    private Constraint _reallastC;
    private int _mycont;
    private double _mylambda1;
    private double _mylambda2;
    private bool _periodic;

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

    private double SearchFirstLambda(MultiLine line, math_Vector aPar, NCollection_Array1<double> theknots, math_Vector v,
                                     int index)
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

        double u1 = aPar[index], u2 = aPar[index + 1];
        double lambda, s;
        int low = v.Lower();
        int nbknots = theknots.Length();

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
        return ((s * lambda) * (theknots[2] - theknots[1]) / (theknots[nbknots] - theknots[1]));
    }

    private double SearchLastLambda(MultiLine line, math_Vector aPar, NCollection_Array1<double> theknots, math_Vector v,
                                    int index)
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

        double u1 = aPar[index - 1], u2 = aPar[index];
        double lambda, s;
        int low = v.Lower();
        int nbknots = theknots.Length();
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

        return ((s * lambda) * (theknots[nbknots] - theknots[nbknots - 1])
                / (theknots[nbknots] - theknots[1]));
    }

    /// <summary>
    /// Approximates <paramref name="line"/> from the parameters <paramref name="parameters"/> until the tolerances
    /// are reached, from degree <paramref name="degreemin"/> to <paramref name="degreemax"/>, adding knots if
    /// <paramref name="cutting"/>. With <paramref name="squares"/>, without iterations.
    /// </summary>
    public BSplComputeLine(MultiLine line, math_Vector parameters, int degreemin = 4, int degreemax = 8,
                                  double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06, int nbIterations = 5,
                                  bool cutting = true, bool squares = false)
    {
        _myfirstParam = new NCollection_HArray1<double>(parameters.Lower(), parameters.Upper());
        for (int i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            _myfirstParam.SetValue(i, parameters[i]);
        }
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _par = IsoParametric;
        _periodic = false;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _alldone = false;
        _mycont = -1;
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        _myhasknots = false;
        _myhasmults = false;
        _currenttol3d = _currenttol2d = RealLast();
        _tolreached = false;
        Perform(line);
    }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="parameters"/>.</summary>
    public BSplComputeLine(math_Vector parameters, int degreemin = 4, int degreemax = 8,
                                  double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06, int nbIterations = 5,
                                  bool cutting = true, bool squares = false)
    {
        _myfirstParam = new NCollection_HArray1<double>(parameters.Lower(), parameters.Upper());
        for (int i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            _myfirstParam.SetValue(i, parameters[i]);
        }
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _par = IsoParametric;
        _periodic = false;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _alldone = false;
        _myhasknots = false;
        _myhasmults = false;
        _mycont = -1;
        _currenttol3d = _currenttol2d = RealLast();
        _tolreached = false;
    }

    /// <summary>Initialises the fields of the algorithm; <see cref="Perform"/> runs it.</summary>
    public BSplComputeLine(int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03,
                                  double tolerance2d = 1.0e-06, int nbIterations = 5, bool cutting = true,
                                  global::Kerf.Geometry.ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _par = parametrization;
        _periodic = false;
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
        _myhasknots = false;
        _myhasmults = false;
        _mycont = -1;
        _currenttol3d = _currenttol2d = RealLast();
        _tolreached = false;
    }

    /// <summary>
    /// Approximates <paramref name="line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, adding knots if <paramref name="cutting"/>.
    /// With <paramref name="squares"/>, without iterations. The multiplicities of the interior knots are set by
    /// default.
    /// </summary>
    public BSplComputeLine(MultiLine line, int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-3,
                                  double tolerance2d = 1.0e-6, int nbIterations = 5, bool cutting = true,
                                  global::Kerf.Geometry.ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _constraints = new NCollection_HArray1<ConstraintCouple>(1, 2);
        _alldone = false;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mysquares = squares;
        _mycut = cutting;
        _myitermax = nbIterations;
        _par = parametrization;
        _periodic = false;
        _myfirstC = TangencyPoint;
        _mylastC = TangencyPoint;
        _myhasknots = false;
        _myhasmults = false;
        _mycont = -1;
        _currenttol3d = _currenttol2d = RealLast();
        _tolreached = false;
        Perform(line);
    }

    /// <summary>Runs the algorithm on <paramref name="line"/> after the fields have been initialised.</summary>
    public void Perform(MultiLine line)
    {
        int i, thefirstpt, thelastpt;
        bool finish = false, begin = true;

        // Search for the actual constraints given by the Line:
        FindRealConstraints(line);

        thefirstpt = LineTool.FirstPoint(line);
        thelastpt = LineTool.LastPoint(line);
        int myfirstpt = thefirstpt;
        int mylastpt = thelastpt;

        ConstraintCouple myCouple1 = new(myfirstpt, _realfirstC);
        ConstraintCouple myCouple2 = new(mylastpt, _reallastC);
        _constraints.SetValue(1, myCouple1);
        _constraints.SetValue(2, myCouple2);

        math_Vector theParam = new(thefirstpt, thelastpt, 0.0);
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

        _parameters = new NCollection_HArray1<double>(theParam.Lower(), theParam.Upper());
        for (i = theParam.Lower(); i <= theParam.Upper(); i++)
        {
            _parameters.SetValue(i, theParam[i]);
        }
        int nbknots = 2;
        double l;
        _alldone = false;

        if (!_mycut)
        {
            // Case where no additional knots are desired.
            // ==================================================

            if (!_myhasknots)
            {
                NCollection_Array1<double> theknots = new(1, 2);
                NCollection_Array1<int> themults = new(1, 2);
                theknots[1] = 0.0;
                theknots[2] = 1.0;
                _alldone = Compute(line, myfirstpt, mylastpt, theParam, theknots, themults);
            }
            else
            {
                if (!_myhasmults)
                {
                    NCollection_Array1<int> themults = new(1, _myknots!.Length());
                    _alldone = Compute(line, myfirstpt, mylastpt, theParam, _myknots.Array1(), themults);
                }
                else
                {
                    _alldone = Compute(line, myfirstpt, mylastpt, theParam, _myknots!.Array1(), _mymults!.ChangeArray1());
                }
            }
        }
        else
        {
            // cas ou on va iterer a partir de noeuds donnes par l'utilisateur
            // ou a partir d'une bezier.
            // ================================================================

            while (!finish)
            {
                _currenttol3d = _currenttol2d = RealLast();

                if (_myhasknots && begin)
                {
                    if (!_myhasmults)
                    {
                        // 1st case: the user provides starting knots but
                        // it is up to us to set the multiplicities according to the
                        // desired continuity.
                        // ========================================================

                        NCollection_Array1<int> theMults = new(1, _myknots!.Length());
                        _alldone = Compute(line, myfirstpt, mylastpt, theParam, _myknots.Array1(), theMults);
                    }
                    else
                    {
                        // 2nd case: the user provides starting knots
                        // with their multiplicities.
                        // ===================================================

                        _alldone = Compute(line, myfirstpt, mylastpt, theParam, _myknots!.Array1(), _mymults!.ChangeArray1());
                    }
                    begin = false;
                }
                else
                {
                    // 3eme cas: l'utilisateur ne donne aucun noeuds de depart
                    // ========================================================

                    NCollection_Array1<double> Theknots = new(1, nbknots);
                    NCollection_Array1<int> theMults = new(1, nbknots);
                    Theknots[1] = 0.0;
                    Theknots[nbknots] = 1.0;
                    for (i = 2; i <= nbknots - 1; i++)
                    {
                        l = (mylastpt - myfirstpt) * (double)(i - 1) / (double)(nbknots - 1);
                        int ll = (int)(l);
                        double a = l - ll;
                        double p1 = theParam[ll + myfirstpt];
                        double p2 = theParam[ll + 1 + myfirstpt];
                        Theknots[i] = (1.0 - a) * p1 + a * p2;
                    }

                    _alldone = Compute(line, myfirstpt, mylastpt, theParam, Theknots, theMults);
                }

                if (!_alldone)
                {
                    nbknots++;
                }
                else
                {
                    finish = true;
                }
            }
        }
    }

    /// <summary>
    /// The new parameters of the points, corresponding to the MultiBSpCurve: the object's own array (shared: don't
    /// modify).
    /// </summary>
    public NCollection_Array1<double> Parameters() => _parameters!.Array1();

    /// <summary>The result of the approximation: the object's own MultiBSpCurve (shared).</summary>
    public MultiBSpCurve Value() => _theMultiBSpCurve;

    /// <summary>The result of the approximation, as <see cref="Value"/> (OCCT's modifiable reference).</summary>
    public MultiBSpCurve ChangeValue() => _theMultiBSpCurve;

    // Computes new parameters between firstP and lastP.
    private void Parameters(MultiLine line, int firstP, int lastP, math_Vector theParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;
        int nbp = lastP - firstP + 1;

        // The first parameter should always be zero according to all the logic below,
        // so division by any value will give zero anyway, so it should never be scaled
        // to avoid case when there is only one parameter in the array thus division by zero happens.
        theParameters[firstP] = 0.0;
        if (nbp == 2)
        {
            theParameters[lastP] = 1.0;
        }
        else if (_par == ChordLength || _par == Centripetal)
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
                dist = 0.0;
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
                if (_par == ChordLength)
                {
                    theParameters[i] = theParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    theParameters[i] = theParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP + 1; i <= lastP; i++)
            {
                theParameters[i] /= theParameters[lastP];
            }
        }
        else
        {
            for (i = firstP + 1; i <= lastP; i++)
            {
                theParameters[i] = ((double)i - firstP) / ((double)(lastP - (double)firstP));
            }
        }
    }

    private bool Compute(MultiLine line, int fpt, int lpt, math_Vector para, NCollection_Array1<double> knots,
                         NCollection_Array1<int> mults)
    {
        int i, deg, nbpoles, multinter;
        bool mydone;
        double fv, theTol3d, theTol2d, l1, l2;
        int nbp = lpt - fpt + 1;
        _mylambda1 = 0.0;
        _mylambda2 = 0.0;

        math_Vector aParams = new(para.Lower(), para.Upper());

        for (deg = _mydegremin; deg <= _mydegremax; deg++)
        {
            aParams.Initialized(para);

            if (!_myhasmults)
            {
                mults[mults.Lower()] = deg + 1;
                mults[mults.Upper()] = deg + 1;
                nbpoles = deg + 1;
                if (_mycont == -1)
                {
                    multinter = 1;
                }
                else
                {
                    multinter = Max(1, deg - _mycont);
                }
                for (i = mults.Lower() + 1; i <= mults.Upper() - 1; i++)
                {
                    mults[i] = multinter;
                    nbpoles += multinter;
                }
            }
            else
            {
                nbpoles = -deg - 1;
                for (i = mults.Lower(); i <= mults.Upper(); i++)
                {
                    nbpoles += mults.Value(i);
                }
            }

            int nbpolestocompare = nbpoles;
            if (_realfirstC == TangencyPoint)
            {
                nbpolestocompare++;
            }
            if (_reallastC == TangencyPoint)
            {
                nbpolestocompare++;
            }
            if (_realfirstC == CurvaturePoint)
            {
                nbpolestocompare++;
            }
            if (_reallastC == CurvaturePoint)
            {
                nbpolestocompare++;
            }
            if (nbpolestocompare > nbp)
            {
                Interpol(line);
                _tolreached = true;
                return true;
            }

            MultiBSpCurve mySCU;

            if (_mysquares)
            {
                LeastSquare<MultiLine, LineTool> sq =
                    new(line, knots, mults, fpt, lpt, _realfirstC, _reallastC, aParams, nbpoles);
                mydone = sq.IsDone;
                if (mydone)
                {
                    mySCU = new MultiBSpCurve(sq.BSplineValue());
                    sq.Error(out fv, out theTol3d, out theTol2d);
                }
                else
                {
                    continue;
                }
            }
            else
            {
                if (nbpoles != deg + 1)
                {
                    if (deg == _mydegremin
                        && (_realfirstC >= TangencyPoint
                            || _reallastC >= TangencyPoint))
                    {
                        LeastSquare<MultiLine, LineTool> thefitt =
                            new(line, knots, mults, fpt, lpt, _realfirstC, _reallastC, aParams, nbpoles);
                        _mylambda1 = thefitt.FirstLambda() * deg;
                        _mylambda2 = thefitt.LastLambda() * deg;
                    }
                    l1 = _mylambda1 / deg;
                    l2 = _mylambda2 / deg;

                    BSpGradient<MultiLine, LineTool> grad = new(line, fpt, lpt, _constraints, aParams, knots,
                                                                            mults, deg, _mytol3d, _mytol2d, _myitermax,
                                                                            l1, l2);

                    mydone = grad.IsDone;
                    if (mydone)
                    {
                        mySCU = grad.Value();
                        theTol3d = grad.MaxError3d();
                        theTol2d = grad.MaxError2d();
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    Gradient<MultiLine, LineTool> grad2 =
                        new(line, fpt, lpt, _constraints, aParams, deg, _mytol3d, _mytol2d, _myitermax);
                    mydone = grad2.IsDone;
                    if (mydone)
                    {
                        if (grad2.Value().NbCurves() == 0)
                        {
                            continue;
                        }
                        mySCU = new MultiBSpCurve(grad2.Value(), knots, mults);
                        theTol3d = grad2.MaxError3d();
                        theTol2d = grad2.MaxError2d();
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            bool save = true;

            for (i = aParams.Lower(); i <= aParams.Upper(); i++)
            {
                if (aParams[i] <= -0.000001 || aParams[i] >= 1.000001)
                {
                    save = false;
                    break;
                }
            }

            if (mydone)
            {
                if (theTol3d <= _mytol3d && theTol2d <= _mytol2d)
                {
                    // Stockage de la multicurve approximee.
                    _tolreached = true;
                    _theMultiBSpCurve = mySCU;
                    _currenttol3d = theTol3d;
                    _currenttol2d = theTol2d;
                    if (save)
                    {
                        for (i = aParams.Lower(); i <= aParams.Upper(); i++)
                        {
                            _parameters!.SetValue(i, aParams[i]);
                        }
                    }
                    return true;
                }
            }

            if (theTol3d <= _currenttol3d && theTol2d <= _currenttol2d)
            {
                _theMultiBSpCurve = mySCU;
                _currenttol3d = theTol3d;
                _currenttol2d = theTol2d;
                if (save)
                {
                    for (i = aParams.Lower(); i <= aParams.Upper(); i++)
                    {
                        _parameters!.SetValue(i, aParams[i]);
                    }
                }
            }
        }

        return false;
    }

    /// <summary>The approximation will start from the parameters <paramref name="thePar"/> (copied).</summary>
    public void SetParameters(math_Vector thePar)
    {
        _myfirstParam = new NCollection_HArray1<double>(thePar.Lower(), thePar.Upper());
        for (int i = thePar.Lower(); i <= thePar.Upper(); i++)
        {
            _myfirstParam.SetValue(i, thePar[i]);
        }
    }

    /// <summary>
    /// The approximation will use the knots <paramref name="knots"/> (copied); the multiplicities follow from the
    /// degree and the continuity.
    /// </summary>
    public void SetKnots(NCollection_Array1<double> knots)
    {
        _myhasknots = true;
        _myknots = new NCollection_HArray1<double>(knots.Lower(), knots.Upper());
        for (int i = knots.Lower(); i <= knots.Upper(); i++)
        {
            _myknots.SetValue(i, knots[i]);
        }
    }

    /// <summary>The approximation will use the knots <paramref name="knots"/> and multiplicities <paramref name="mults"/> (copied).</summary>
    public void SetKnotsAndMultiplicities(NCollection_Array1<double> knots, NCollection_Array1<int> mults)
    {
        _myhasknots = true;
        _myhasmults = true;
        int i;
        _myknots = new NCollection_HArray1<double>(knots.Lower(), knots.Upper());
        for (i = knots.Lower(); i <= knots.Upper(); i++)
        {
            _myknots.SetValue(i, knots[i]);
        }
        _mymults = new NCollection_HArray1<int>(mults.Lower(), mults.Upper());
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            _mymults.SetValue(i, mults[i]);
        }
    }

    /// <summary>Initialises the fields of the algorithm.</summary>
    public void Init(int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06,
                     int nbIterations = 5, bool cutting = true,
                     global::Kerf.Geometry.ParametrizationType parametrization = ChordLength, bool squares = false)
    {
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _par = parametrization;
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

    /// <summary>
    /// If true, tries to build a periodic MultiCurve, with C1 conditions at the first and the last points. The
    /// MultiLine must be closed.
    /// </summary>
    public void SetPeriodic(bool periodic)
    {
        _periodic = periodic;
    }

    /// <summary>False if, during the approximation, the line tool returned NoApproximation when more points were needed.</summary>
    public bool IsAllApproximated => _alldone;

    /// <summary>False if the line tool returned NoPointsAdded.</summary>
    public bool IsToleranceReached => _tolreached;

    /// <summary>The 3d and 2d tolerances of the MultiBSpCurve.</summary>
    public void Error(out double tol3d, out double tol2d)
    {
        tol3d = _currenttol3d;
        tol2d = _currenttol2d;
    }

    /// <summary>Sets the continuity of the spline: with C = 2, it is C2.</summary>
    public void SetContinuity(int c)
    {
        _mycont = c;
    }

    private void FindRealConstraints(MultiLine line)
    {
        _realfirstC = _myfirstC;
        _reallastC = _mylastC;
        int nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);
        bool ok = false;
        NCollection_Array1<Vector3D> tabV = new(1, Max(1, nbP3d));
        NCollection_Array1<Vector2D> tabV2d = new(1, Max(1, nbP2d));
        int thefirstpt = LineTool.FirstPoint(line);
        int thelastpt = LineTool.LastPoint(line);

        if (_myfirstC >= TangencyPoint)
        {
            if (nbP3d != 0 && nbP2d != 0)
            {
                ok = LineTool.Tangency(line, thefirstpt, tabV, tabV2d);
            }
            else if (nbP2d != 0)
            {
                ok = LineTool.Tangency(line, thefirstpt, tabV2d);
            }
            else if (nbP3d != 0)
            {
                ok = LineTool.Tangency(line, thefirstpt, tabV);
            }

            _realfirstC = PassPoint;
            if (ok)
            {
                _realfirstC = TangencyPoint;
                if (_myfirstC == CurvaturePoint)
                {
                    // OCCT tests the tangents again, not the curvatures (occt-issues.md).
                    if (nbP3d != 0 && nbP2d != 0)
                    {
                        ok = LineTool.Tangency(line, thefirstpt, tabV, tabV2d);
                    }
                    else if (nbP2d != 0)
                    {
                        ok = LineTool.Tangency(line, thefirstpt, tabV2d);
                    }
                    else if (nbP3d != 0)
                    {
                        ok = LineTool.Tangency(line, thefirstpt, tabV);
                    }
                    if (ok)
                    {
                        _realfirstC = CurvaturePoint;
                    }
                }
            }
        }

        if (_mylastC >= TangencyPoint)
        {
            if (nbP3d != 0 && nbP2d != 0)
            {
                ok = LineTool.Tangency(line, thelastpt, tabV, tabV2d);
            }
            else if (nbP2d != 0)
            {
                ok = LineTool.Tangency(line, thelastpt, tabV2d);
            }
            else if (nbP3d != 0)
            {
                ok = LineTool.Tangency(line, thelastpt, tabV);
            }

            _reallastC = PassPoint;
            if (ok)
            {
                _reallastC = TangencyPoint;
                if (_mylastC == CurvaturePoint)
                {
                    // OCCT tests the tangents again, not the curvatures (occt-issues.md).
                    if (nbP3d != 0 && nbP2d != 0)
                    {
                        ok = LineTool.Tangency(line, thelastpt, tabV, tabV2d);
                    }
                    else if (nbP2d != 0)
                    {
                        ok = LineTool.Tangency(line, thelastpt, tabV2d);
                    }
                    else if (nbP3d != 0)
                    {
                        ok = LineTool.Tangency(line, thelastpt, tabV);
                    }
                    if (ok)
                    {
                        _reallastC = CurvaturePoint;
                    }
                }
            }
        }
    }

    /// <summary>Interpolates the points of <paramref name="line"/> by a C2 cubic B-spline.</summary>
    public void Interpol(MultiLine line)
    {
        int i, thefirstpt, thelastpt, deg = 3;
        _mycont = 2;
        thefirstpt = LineTool.FirstPoint(line);
        thelastpt = LineTool.LastPoint(line);
        math_Vector theParam = new(thefirstpt, thelastpt, 0.0);
        // Par = Approx_ChordLength;
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
        Constraint cons = TangencyPoint;
        double lambda1, lambda2;
        double fv;

        // Recherche du nombre de noeuds.
        int nbknots, nbpoles, nbpoints;
        nbpoints = thelastpt - thefirstpt + 1;

        if (nbpoints == 2)
        {
            cons = NoConstraint;
            int mydeg = 1;
            LeastSquare<MultiLine, LineTool> lsq = new(line, thefirstpt, thelastpt, cons, cons, theParam, mydeg + 1);
            _alldone = lsq.IsDone;
            NCollection_Array1<double> theKnots = new(1, 2);
            NCollection_Array1<int> theMults = new(1, 2);
            theKnots[1] = theParam[thefirstpt];
            theKnots[2] = theParam[thelastpt];
            theMults[1] = theMults[2] = mydeg + 1;
            _theMultiBSpCurve = new MultiBSpCurve(lsq.BezierValue(), theKnots, theMults);
            lsq.Error(out fv, out _currenttol3d, out _currenttol2d);
        }
        else
        {
            nbpoles = nbpoints + 2;
            nbknots = nbpoints;

            // Resolution:
            NCollection_Array1<double> theknots = new(1, nbknots);
            theknots[1] = theParam[thefirstpt];
            theknots[nbknots] = theParam[thelastpt];
            NCollection_Array1<int> theMults = new(1, nbknots);
            theMults[1] = deg + 1;
            theMults[nbknots] = deg + 1;

            int low = theParam.Lower();
            for (i = 2; i <= nbknots - 1; i++)
            {
                theknots[i] = theParam[i + low - 1];
                theMults[i] = 1;
            }

            int nbP = 3 * LineTool.NbP3d(line) + 2 * LineTool.NbP2d(line);
            math_Vector v1 = new(1, nbP), v2 = new(1, nbP);

            if (nbpoints == 3 || nbpoints == 4)
            {
                FirstTangencyVector(line, thefirstpt, v1);
                lambda1 = SearchFirstLambda(line, theParam, theknots, v1, thefirstpt);

                LastTangencyVector(line, thelastpt, v2);
                lambda2 = SearchLastLambda(line, theParam, theknots, v2, thelastpt);

                lambda1 = lambda1 / deg;
                lambda2 = lambda2 / deg;
            }
            else
            {
                int nnpol, nnp = Min(nbpoints, 9);
                nnpol = nnp;
                int lastp = Min(thelastpt, thefirstpt + nnp - 1);
                double u;
                LeastSquare<MultiLine, LineTool> sq1 = new(line, thefirstpt, lastp, cons, cons, nnpol);

                math_Vector p1 = new(thefirstpt, lastp);
                for (i = thefirstpt; i <= lastp; i++)
                {
                    p1[i] = theParam[i];
                }
                sq1.Perform(p1);
                MultiCurve c1 = sq1.BezierValue();
                u = 0.0;
                TangencyVector(line, c1, u, v1);

                int firstp = Max(thefirstpt, thelastpt - nnp + 1);

                if (firstp == thefirstpt && lastp == thelastpt)
                {
                    u = 1.0;
                    TangencyVector(line, c1, u, v2);
                }
                else
                {
                    LeastSquare<MultiLine, LineTool> sq2 = new(line, firstp, thelastpt, cons, cons, nnpol);

                    math_Vector p2 = new(firstp, thelastpt);
                    for (i = firstp; i <= thelastpt; i++)
                    {
                        p2[i] = theParam[i];
                    }
                    sq2.Perform(p2);
                    MultiCurve c2 = sq2.BezierValue();
                    u = 1.0;
                    TangencyVector(line, c2, u, v2);
                }

                lambda1 = 1.0 / deg;
                lambda1 = lambda1 * (theknots[2] - theknots[1]) / (theknots[nbknots] - theknots[1]);
                lambda2 = 1.0 / deg;
                lambda2 = lambda2 * (theknots[nbknots] - theknots[nbknots - 1]) / (theknots[nbknots] - theknots[1]);
            }

            if (_periodic)
            {
                v1.Initialized(0.5 * (v1 + v2));
                v2.Initialized(v1);
            }

            LeastSquare<MultiLine, LineTool> sq = new(line, theknots, theMults, thefirstpt, thelastpt, cons, cons,
                                                                  nbpoles);

            sq.Perform(theParam, v1, v2, lambda1, lambda2);
            _alldone = sq.IsDone;
            _theMultiBSpCurve = new MultiBSpCurve(sq.BSplineValue());
            sq.Error(out fv, out _currenttol3d, out _currenttol2d);
            _tolreached = true;
        }
        _parameters = new NCollection_HArray1<double>(theParam.Lower(), theParam.Upper());
        for (i = theParam.Lower(); i <= theParam.Upper(); i++)
        {
            _parameters.SetValue(i, theParam[i]);
        }
    }

    private static void TangencyVector(MultiLine line, MultiCurve c, double u, math_Vector v)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(line);
        nbP2d = LineTool.NbP2d(line);

        Point3D myP;
        Vector3D myV;
        Point2D myP2d;
        Vector2D myV2d;
        j = 1;
        for (i = 1; i <= nbP3d; i++)
        {
            c.D1(i, u, out myP, out myV);
            v[j] = myV.X;
            v[j + 1] = myV.Y;
            v[j + 2] = myV.Z;
            j += 3;
        }
        j = nbP3d * 3 + 1;
        for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
        {
            c.D1(i, u, out myP2d, out myV2d);
            v[j] = myV2d.X;
            v[j + 1] = myV2d.Y;
            j += 2;
        }
    }
}
