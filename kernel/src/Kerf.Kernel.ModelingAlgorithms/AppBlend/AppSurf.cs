// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_AppSurf.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_AppSurf.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Fillets.Blend.AppBlendDebug;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The approximation of a surface through sections (AppBlend_AppSurf.gxx): each section, a B-spline curve with
/// optional 2d curves, is a multi-point of its poles, its 2d poles and its weights, and AppDef approximates the
/// multi-line in V (Bezier pieces joined, a B-spline least squares, or the variational smoothing).
/// </summary>
/// <remarks>
/// <para>
/// One generic class for OCCT's `.gxx` (decisions 142, 202 and 607; decision 845): the section generator and the
/// line are instance interfaces (<see cref="ISectionGenerator{ThePoint}"/>,
/// <see cref="ILine{ThePoint}"/>) and <typeparamref name="ThePoint"/> is what the line gives for a point.
/// OCCT's instantiations are sealed subclasses: <see cref="SurfaceAppSurf"/> here, GeomFill_AppSweep and
/// BRepBlend_AppSurf later.
/// </para>
/// <para>
/// A working object (decision 21). The arrays the accessors return are its own (decision 143): shared, don't modify.
/// </para>
/// </remarks>
internal class AppSurf<TheSectionGenerator, TheLine, ThePoint> : Approx
    where TheSectionGenerator : ISectionGenerator<ThePoint>
    where TheLine : class, ILine<ThePoint>
{
    // OCCT's file-static `scal`, never changed.
    private const bool Scal = true;

    private bool _done;
    private int _dmin;
    private int _dmax;
    private double _tol3d;
    private double _tol2d;
    private int _nbit;
    private int _udeg;
    private int _vdeg;
    private bool _knownp;
    private NCollection_HArray2<Point3D>? _tabPoles;
    private NCollection_HArray2<double>? _tabWeights;
    private NCollection_HArray1<double>? _tabUKnots;
    private NCollection_HArray1<double>? _tabVKnots;
    private NCollection_HArray1<int>? _tabUMults;
    private NCollection_HArray1<int>? _tabVMults;
    private readonly NCollection_Sequence<NCollection_HArray1<Point2D>> _seqPoles2d = new();
    private double _tol3dreached;
    private double _tol2dreached;
    private global::Kerf.Geometry.ParametrizationType _paramtype;
    private Continuity _continuity;
    private readonly double[] _critweights = new double[3];

    public AppSurf()
    {
        _done = false;
        _dmin = 0;
        _dmax = 0;
        _tol3d = 0.0;
        _tol2d = 0.0;
        _nbit = 0;
        _udeg = 0;
        _vdeg = 0;
        _knownp = false;
        _tol3dreached = 0.0;
        _tol2dreached = 0.0;
        _paramtype = ChordLength;
        _continuity = C2;
        _critweights[0] = 0.4;
        _critweights[1] = 0.2;
        _critweights[2] = 0.4;
    }

    public AppSurf(int degmin, int degmax, double tol3d, double tol2d, int nbIt, bool knownParameters = false)
    {
        _done = false;
        _dmin = degmin;
        _dmax = degmax;
        _tol3d = tol3d;
        _tol2d = tol2d;
        _nbit = nbIt;
        _udeg = 0;
        _vdeg = 0;
        _knownp = knownParameters;
        _tol3dreached = 0.0;
        _tol2dreached = 0.0;
        _paramtype = ChordLength;
        _continuity = C2;
        _critweights[0] = 0.4;
        _critweights[1] = 0.2;
        _critweights[2] = 0.4;
    }

    public void Init(int degmin, int degmax, double tol3d, double tol2d, int nbIt, bool knownParameters = false)
    {
        _done = false;
        _dmin = degmin;
        _dmax = degmax;
        _tol3d = tol3d;
        _tol2d = tol2d;
        _nbit = nbIt;
        _knownp = knownParameters;
        _continuity = C2;
        _paramtype = ChordLength;
        _critweights[0] = 0.4;
        _critweights[1] = 0.2;
        _critweights[2] = 0.4;
    }

    /// <summary>The weights of the criterion used in the optimisation (the variational smoothing).</summary>
    public void CriteriumWeight(out double w1, out double w2, out double w3)
    {
        w1 = _critweights[0];
        w2 = _critweights[1];
        w3 = _critweights[2];
    }

    /// <summary>Sets the weights of the criterion; throws Standard_DomainError if one is negative.</summary>
    public void SetCriteriumWeight(double w1, double w2, double w3)
    {
        if (w1 < 0 || w2 < 0 || w3 < 0)
        {
            throw new DomainErrorException();
        }
        _critweights[0] = w1;
        _critweights[1] = w2;
        _critweights[2] = w3;
    }

    /// <summary>The continuity of the spline and smoothing approximations.</summary>
    public void SetContinuity(Continuity theCont)
    {
        _continuity = theCont;
    }

    public Continuity Continuity() => _continuity;

    /// <summary>The type of parametrisation of the approximation.</summary>
    public void SetParType(global::Kerf.Geometry.ParametrizationType parType)
    {
        _paramtype = parType;
    }

    public global::Kerf.Geometry.ParametrizationType ParType() => _paramtype;

    /// <summary>
    /// Approximates the sections of <paramref name="lin"/>: Bezier pieces joined (AppDef_Compute), or with
    /// <paramref name="spApprox"/> a B-spline least squares (AppDef_BSplineCompute) on every section.
    /// </summary>
    public void Perform(TheLine? lin, TheSectionGenerator f, bool spApprox = false)
    {
        InternalPerform(lin, f, spApprox, false);
    }

    /// <summary>Approximates the sections of <paramref name="lin"/> by the variational smoothing (AppDef_Variational).</summary>
    public void PerformSmoothing(TheLine? lin, TheSectionGenerator f)
    {
        InternalPerform(lin, f, true, true);
    }

    private void InternalPerform(TheLine? lin, TheSectionGenerator f, bool spApprox, bool useSmoothing)
    {
        _done = false;
        if (lin is null)
        {
            return;
        }
        int i, j, k, nbPoint;
        int nbUPoles, nbUKnots, nbPoles2d, nbVPoles;
        bool withderiv;
        Constraint cfirst, clast;

        double mytol3d, mytol2d;
        Xyz newDv = new();

        _seqPoles2d.Clear();

        nbPoint = lin.NbPoints();
        MultiPointConstraint multP;
        MultiLine multL = new(nbPoint);

        f.GetShape(out nbUPoles, out nbUKnots, out _udeg, out nbPoles2d);

        _tabUKnots = new NCollection_HArray1<double>(1, nbUKnots);
        _tabUMults = new NCollection_HArray1<int>(1, nbUKnots);

        f.Knots(_tabUKnots.ChangeArray1());
        f.Mults(_tabUMults.ChangeArray1());

        NCollection_Array1<Point3D> tabAppP = new(1, nbUPoles);
        NCollection_Array1<Vector3D> tabAppV = new(1, nbUPoles);

        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, nbPoles2d));
        NCollection_Array1<Vector2D> tabV2d = new(1, Max(1, nbPoles2d));

        NCollection_Array1<double> tabW = new(1, nbUPoles), tabDW = new(1, nbUPoles);

        NCollection_Array1<Point2D> tabAppP2d = new(1, nbPoles2d + nbUPoles); // points2d + poids
        NCollection_Array1<Vector2D> tabAppV2d = new(1, nbPoles2d + nbUPoles);

        MultiBSpCurve multC;

        withderiv = f.Section(lin.Point(1), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);

        if (AppBlend_GetContextApproxWithNoTgt())
        {
            withderiv = false;
        }

        for (j = 1; j <= nbPoles2d; j++)
        {
            tabAppP2d[j] = tabP2d[j];
            if (withderiv)
            {
                tabAppV2d[j] = tabV2d[j];
            }
        }
        for (j = 1; j <= nbUPoles; j++)
        {
            // pour les courbes rationnelles il faut multiplier les poles par
            // leurs poids respectifs
            if (withderiv)
            {
                tabAppV2d[nbPoles2d + j] = tabAppV2d[nbPoles2d + j].WithCoord(tabDW[j], 0.0);
                newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                tabAppV[j] = tabAppV[j].WithXYZ(newDv);
            }
            tabAppP[j] = tabAppP[j].WithXYZ(tabAppP[j].XYZ() * tabW[j]);
            tabAppP2d[nbPoles2d + j] = tabAppP2d[nbPoles2d + j].WithCoord(tabW[j], 0.0);
        }

        if (withderiv)
        {
            multP = new MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
            cfirst = TangencyPoint;
        }
        else
        {
            multP = new MultiPointConstraint(tabAppP, tabAppP2d);
            cfirst = PassPoint;
        }
        multL.SetValue(1, multP);

        for (i = 2; i <= nbPoint - 1; i++)
        {
            if (spApprox)
            {
                f.Section(lin.Point(i), tabAppP, tabP2d, tabW);
                for (j = 1; j <= nbPoles2d; j++)
                {
                    tabAppP2d[j] = tabP2d[j];
                }
                for (j = 1; j <= nbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    tabAppP[j] = tabAppP[j].WithXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[nbPoles2d + j] = tabAppP2d[nbPoles2d + j].WithCoord(tabW[j], 0.0);
                }
                multP = new MultiPointConstraint(tabAppP, tabAppP2d);
                multL.SetValue(i, multP);
            }
            // ***********************
            else
            {
                withderiv = f.Section(lin.Point(i), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
                if (AppBlend_GetContextApproxWithNoTgt())
                {
                    withderiv = false;
                }

                for (j = 1; j <= nbPoles2d; j++)
                {
                    tabAppP2d[j] = tabP2d[j];
                    if (withderiv)
                    {
                        tabAppV2d[j] = tabV2d[j];
                    }
                }
                for (j = 1; j <= nbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    if (withderiv)
                    {
                        tabAppV2d[nbPoles2d + j] = tabAppV2d[nbPoles2d + j].WithCoord(tabDW[j], 0.0);
                        newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                        tabAppV[j] = tabAppV[j].WithXYZ(newDv);
                    }
                    tabAppP[j] = tabAppP[j].WithXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[nbPoles2d + j] = tabAppP2d[nbPoles2d + j].WithCoord(tabW[j], 0.0);
                }
                if (withderiv)
                {
                    multP = new MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
                }
                else
                {
                    multP = new MultiPointConstraint(tabAppP, tabAppP2d);
                }
                multL.SetValue(i, multP);
            }
            // ******************************
        }

        withderiv = f.Section(lin.Point(nbPoint), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
        if (AppBlend_GetContextApproxWithNoTgt())
        {
            withderiv = false;
        }

        for (j = 1; j <= nbPoles2d; j++)
        {
            tabAppP2d[j] = tabP2d[j];
            if (withderiv)
            {
                tabAppV2d[j] = tabV2d[j];
            }
        }
        for (j = 1; j <= nbUPoles; j++)
        {
            // pour les courbes rationnelles il faut multiplier les poles par
            // leurs poids respectifs
            if (withderiv)
            {
                tabAppV2d[nbPoles2d + j] = tabAppV2d[nbPoles2d + j].WithCoord(tabDW[j], 0.0);
                newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                tabAppV[j] = tabAppV[j].WithXYZ(newDv);
            }
            tabAppP[j] = tabAppP[j].WithXYZ(tabAppP[j].XYZ() * tabW[j]);
            tabAppP2d[nbPoles2d + j] = tabAppP2d[nbPoles2d + j].WithCoord(tabW[j], 0.0);
        }

        if (withderiv)
        {
            multP = new MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
            clast = TangencyPoint;
        }
        else
        {
            multP = new MultiPointConstraint(tabAppP, tabAppP2d);
            clast = PassPoint;
        }
        multL.SetValue(nbPoint, multP);

        // IFV 04.06.07 occ13904
        if (nbPoint == 2)
        {
            _dmin = 1;
            if (cfirst == PassPoint && clast == PassPoint)
            {
                _dmax = 1;
            }
        }

        if (!spApprox)
        {
            Compute theapprox = new(_dmin, _dmax, _tol3d, _tol2d, _nbit, true, _paramtype);
            if (_knownp)
            {
                math_Vector theParams = new(1, nbPoint);

                // On recale les parametres entre 0 et 1.
                theParams[1] = 0.0;
                theParams[nbPoint] = 1.0;
                double uf = f.Parameter(lin.Point(1));
                double ul = f.Parameter(lin.Point(nbPoint)) - uf;
                for (i = 2; i < nbPoint; i++)
                {
                    theParams[i] = (f.Parameter(lin.Point(i)) - uf) / ul;
                }
                Compute theAppDef = new(theParams, _dmin, _dmax, _tol3d, _tol2d, _nbit, true, true);
                theapprox = theAppDef;
            }
            theapprox.SetConstraints(cfirst, clast);
            theapprox.Perform(multL);

            double theTol3d, theTol2d;
            mytol3d = mytol2d = 0.0;
            for (int index = 1; index <= theapprox.MultiCurveCount; index++)
            {
                theapprox.Error(index, out theTol3d, out theTol2d);
                mytol3d = Max(theTol3d, mytol3d);
                mytol2d = Max(theTol2d, mytol2d);
            }
            // OCCT: the tolerances reached are not stored on this path (TolReached keeps the previous values).
            multC = theapprox.SplineValue();
        }
        else
        {
            if (!useSmoothing)
            {
                bool useSquares = false;
                if (_nbit == 0)
                {
                    useSquares = true;
                }
                BSplineCompute theapprox = new(_dmin, _dmax, _tol3d, _tol2d, _nbit, true, _paramtype, useSquares);
                if (_continuity == C0)
                {
                    theapprox.SetContinuity(0);
                }
                if (_continuity == C1)
                {
                    theapprox.SetContinuity(1);
                }
                else if (_continuity == C2)
                {
                    theapprox.SetContinuity(2);
                }
                else
                {
                    // OCCT: C0 falls through to here too, so it ends as continuity 3 (occt-issues.md).
                    theapprox.SetContinuity(3);
                }

                theapprox.SetConstraints(cfirst, clast);

                if (_knownp)
                {
                    math_Vector theParams = new(1, nbPoint);
                    // On recale les parametres entre 0 et 1.
                    theParams[1] = 0.0;
                    theParams[nbPoint] = 1.0;
                    double uf = f.Parameter(lin.Point(1));
                    double ul = f.Parameter(lin.Point(nbPoint)) - uf;
                    for (i = 2; i < nbPoint; i++)
                    {
                        theParams[i] = (f.Parameter(lin.Point(i)) - uf) / ul;
                    }

                    theapprox.Init(_dmin, _dmax, _tol3d, _tol2d, _nbit, true, IsoParametric, true);
                    theapprox.SetParameters(theParams);
                }
                theapprox.Perform(multL);
                theapprox.Error(out mytol3d, out mytol2d);
                _tol3dreached = mytol3d;
                _tol2dreached = mytol2d;
                multC = theapprox.Value();
            }
            else
            {
                // Variational algo
                NCollection_HArray1<ConstraintCouple> taBofCC = new(1, nbPoint);
                Constraint constraint = NoConstraint;

                for (i = 1; i <= nbPoint; ++i)
                {
                    ConstraintCouple acc = new(i, constraint);
                    taBofCC.SetValue(i, acc);
                }

                taBofCC.ChangeValue(1).SetConstraint(cfirst);
                taBofCC.ChangeValue(nbPoint).SetConstraint(clast);

                Variational variation = new(multL, 1, nbPoint, taBofCC);

                //===================================
                int theMaxSegments = 1000;
                bool theWithMinMax = false;
                bool theWithCutting = true;
                //===================================

                variation.SetMaxDegree(_dmax);
                variation.SetContinuity(_continuity);
                variation.SetMaxSegment(theMaxSegments);

                variation.SetTolerance(_tol3d);
                variation.SetWithMinMax(theWithMinMax);
                variation.SetWithCutting(theWithCutting);
                variation.SetNbIterations(_nbit);

                variation.SetCriteriumWeight(_critweights[0], _critweights[1], _critweights[2]);

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

                mytol3d = variation.MaxError();
                mytol2d = 0.0;
                _tol3dreached = mytol3d;
                _tol2dreached = mytol2d;
                multC = variation.Value();
            }
        }

        _vdeg = multC.Degree();
        nbVPoles = multC.NbPoles();

        _tabPoles = new NCollection_HArray2<Point3D>(1, nbUPoles, 1, nbVPoles);
        _tabWeights = new NCollection_HArray2<double>(1, nbUPoles, 1, nbVPoles);
        NCollection_Array1<double> vKnots = multC.Knots();
        _tabVKnots = new NCollection_HArray1<double>(vKnots.Lower(), vKnots.Upper());
        _tabVKnots.ChangeArray1().Assign(vKnots);

        if (_knownp && !useSmoothing)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(f.Parameter(lin.Point(1)), f.Parameter(lin.Point(nbPoint)), _tabVKnots.ChangeArray1());
        }

        NCollection_Array1<int> vMults = multC.Multiplicities();
        _tabVMults = new NCollection_HArray1<int>(vMults.Lower(), vMults.Upper());
        _tabVMults.ChangeArray1().Assign(vMults);

        NCollection_Array1<Point3D> newtabP = new(1, nbVPoles);
        NCollection_HArray1<Point2D> newtabP2d = new(1, nbVPoles);
        for (j = 1; j <= nbUPoles; j++)
        {
            multC.Curve(j, newtabP);
            multC.Curve(j + nbUPoles + nbPoles2d, newtabP2d.ChangeArray1());
            for (k = 1; k <= nbVPoles; k++)
            {
                // pour les courbes rationnelles il faut maintenant diviser
                // les poles par leurs poids respectifs
                _tabPoles[j, k] = _tabPoles[j, k].WithXYZ(newtabP[k].XYZ() / newtabP2d.Value(k).X);
                double weight = newtabP2d.Value(k).X;
                if (weight < Constants.Resolution())
                {
                    _done = false;
                    return;
                }
                _tabWeights.SetValue(j, k, weight);
            }
        }

        for (j = 1; j <= nbPoles2d; j++)
        {
            newtabP2d = new NCollection_HArray1<Point2D>(1, nbVPoles);
            multC.Curve(nbUPoles + j, newtabP2d.ChangeArray1());
            _seqPoles2d.Append(newtabP2d);
        }

        _done = true;
    }

    /// <summary>
    /// Approximates the sections of <paramref name="lin"/> by pieces of at most about <paramref name="nbMaxP"/>
    /// sections each (each scaled to the unit box and approximated by AppDef_Compute), joined into one B-spline.
    /// </summary>
    public void Perform(TheLine? lin, TheSectionGenerator f, int nbMaxP)
    {
        _done = false;
        if (lin is null)
        {
            return;
        }
        int i, j, k;
        int nbUPoles, nbUKnots, nbPoles2d, nbVPoles;
        bool withderiv;
        Constraint cfirst = NoConstraint, clast = NoConstraint;

        double mytol3d = 0.0, mytol2d = 0.0;
        Xyz newDv = new();

        _seqPoles2d.Clear();

        int nbPointTot = lin.NbPoints();

        f.GetShape(out nbUPoles, out nbUKnots, out _udeg, out nbPoles2d);

        _tabUKnots = new NCollection_HArray1<double>(1, nbUKnots);
        _tabUMults = new NCollection_HArray1<int>(1, nbUKnots);

        f.Knots(_tabUKnots.ChangeArray1());
        f.Mults(_tabUMults.ChangeArray1());

        NCollection_Array1<Point3D> tabAppP = new(1, nbUPoles);
        NCollection_Array1<Vector3D> tabAppV = new(1, nbUPoles);
        double X, Y, Z, dx, dy, dz;
        X = Y = Z = RealLast();
        dx = dy = dz = RealFirst();

        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, nbPoles2d));
        NCollection_Array1<Vector2D> tabV2d = new(1, Max(1, nbPoles2d));
        NCollection_Array1<double> x2d = new(1, Max(1, nbPoles2d));
        x2d.Init(RealLast());
        NCollection_Array1<double> y2d = new(1, Max(1, nbPoles2d));
        y2d.Init(RealLast());
        NCollection_Array1<double> dx2d = new(1, Max(1, nbPoles2d));
        dx2d.Init(RealFirst());
        NCollection_Array1<double> dy2d = new(1, Max(1, nbPoles2d));
        dy2d.Init(RealFirst());

        NCollection_Array1<double> tabW = new(1, nbUPoles), tabDW = new(1, nbUPoles);

        NCollection_Array1<Point2D> tabAppP2d = new(1, nbPoles2d + nbUPoles); // points2d + poids
        NCollection_Array1<Vector2D> tabAppV2d = new(1, nbPoles2d + nbUPoles);

        // On calcule les boites de chaque ligne (box for all lines)
        for (i = 1; i <= nbPointTot; i++)
        {
            f.Section(lin.Point(i), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
            double x, y, z;
            for (j = 1; j <= nbUPoles; j++)
            {
                tabAppP[j].Coord(out x, out y, out z);
                if (x < X)
                {
                    X = x;
                }
                if (x > dx)
                {
                    dx = x;
                }
                if (y < Y)
                {
                    Y = y;
                }
                if (y > dy)
                {
                    dy = y;
                }
                if (z < Z)
                {
                    Z = z;
                }
                if (z > dz)
                {
                    dz = z;
                }
            }
            for (j = 1; j <= nbPoles2d; j++)
            {
                tabP2d[j].Coord(out x, out y);
                if (x < x2d[j])
                {
                    x2d[j] = x;
                }
                if (x > dx2d[j])
                {
                    dx2d[j] = x;
                }
                if (y < y2d[j])
                {
                    y2d[j] = y;
                }
                if (y > dy2d[j])
                {
                    dy2d[j] = y;
                }
            }
        }
        // On calcule pour chaque ligne la transformation vers 0 1.
        double seuil = 1000.0 * _tol3d;
        double seuil2d = 1000.0 * _tol2d;
        if ((dx - X) < seuil)
        {
            dx = 1.0;
            X = 0.0;
        }
        else
        {
            dx = 1.0 / (dx - X);
            X *= -dx;
        }
        if ((dy - Y) < seuil)
        {
            dy = 1.0;
            Y = 0.0;
        }
        else
        {
            dy = 1.0 / (dy - Y);
            Y *= -dy;
        }
        if ((dz - Z) < seuil)
        {
            dz = 1.0;
            Z = 0.0;
        }
        else
        {
            dz = 1.0 / (dz - Z);
            Z *= -dz;
        }
        for (j = 1; j <= nbPoles2d; j++)
        {
            if ((dx2d[j] - x2d[j]) < seuil2d)
            {
                dx2d[j] = 1.0;
                x2d[j] = 0.0;
            }
            else
            {
                dx2d[j] = 1.0 / (dx2d[j] - x2d[j]);
                x2d[j] *= -dx2d[j];
            }
            if ((dy2d[j] - y2d[j]) < seuil2d)
            {
                dy2d[j] = 1.0;
                y2d[j] = 0.0;
            }
            else
            {
                dy2d[j] = 1.0 / (dy2d[j] - y2d[j]);
                y2d[j] *= -dy2d[j];
            }
        }
#pragma warning disable CS0162 // OCCT's `scal` is always on: the unscaled branch is dead, kept as OCCT has it
        if (!Scal)
        {
            dx = 1.0;
            X = 0.0;
            dy = 1.0;
            Y = 0.0;
            dz = 1.0;
            Z = 0.0;
            for (j = 1; j <= nbPoles2d; j++)
            {
                dx2d[j] = 1.0;
                x2d[j] = 0.0;
                dy2d[j] = 1.0;
                y2d[j] = 0.0;
            }
        }
#pragma warning restore CS0162
        //  modified by eap Thu Jan  3 14:45:22 2002 ___BEGIN___
        // Keep "inter-troncons" parameters, not only first and last
        NCollection_Sequence<double> paramSeq = new();
        if (_knownp)
        {
            paramSeq.Append(f.Parameter(lin.Point(1)));
        }
        //  modified by EAP Thu Jan  3 14:45:41 2002 ___END___

        MCurvesToBSpCurve concat = new();

        // On calcule le nombre de troncons.
        int nbtronc = nbPointTot / nbMaxP;
        int reste = nbPointTot - (nbtronc * nbMaxP);
        // On regarde si il faut prendre un troncon de plus.
        int nmax = nbMaxP;
        if (nbtronc > 0 && reste > 0)
        {
            nmax = nbPointTot / (nbtronc + 1);
            if (nmax > (2 * nbMaxP) / 3)
            {
                nbtronc++;
                reste = nbPointTot - (nbtronc * nmax);
            }
            else
            {
                nmax = nbMaxP;
            }
        }
        else if (nbtronc == 0)
        {
            nbtronc = 1;
            nmax = reste;
            reste = 0;
        }

        // Approximate each "troncon" with nb of Bezier's using AppDef_Compute
        // and concat them into BSpline with Approx_MCurvesToBSpCurve

        NCollection_Array1<int> troncsize = new(1, nbtronc);
        NCollection_Array1<int> troncstart = new(1, nbtronc);

        int rab = reste / nbtronc + 1;
        int start = 1;
        int itronc;
        for (itronc = 1; itronc <= nbtronc; itronc++)
        {
            troncstart[itronc] = start;
            int rabrab = Min(rab, reste);
            if (reste > 0)
            {
                reste -= rabrab;
            }
            troncsize[itronc] = nmax + rabrab + 1;
            start += (nmax + rabrab);
        }
        troncsize[nbtronc] = troncsize[nbtronc] - 1;
        for (itronc = 1; itronc <= nbtronc; itronc++)
        {
            int nbPoint = troncsize[itronc];
            int stPoint = troncstart[itronc];
            MultiPointConstraint multP;
            MultiLine multL = new(nbPoint);

            for (i = 1; i <= nbPoint; i++)
            {
                int iLin = stPoint + i - 1;
                double x, y, z;
                withderiv = f.Section(lin.Point(iLin), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
                if (AppBlend_GetContextApproxWithNoTgt())
                {
                    withderiv = false;
                }

                for (j = 1; j <= nbPoles2d; j++)
                {
                    tabP2d[j].Coord(out x, out y);
                    tabAppP2d[j] = tabAppP2d[j].WithCoord(dx2d[j] * x + x2d[j], dy2d[j] * y + y2d[j]);
                    if (withderiv)
                    {
                        tabV2d[j].Coord(out x, out y);
                        tabAppV2d[j] = tabAppV2d[j].WithCoord(dx2d[j] * x, dy2d[j] * y);
                    }
                }
                for (j = 1; j <= nbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    if (withderiv)
                    {
                        tabAppV2d[nbPoles2d + j] = tabAppV2d[nbPoles2d + j].WithCoord(tabDW[j], 0.0);
                        newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                        tabAppV[j] = tabAppV[j].WithCoord(dx * newDv.X(), dy * newDv.Y(), dz * newDv.Z());
                    }
                    tabAppP[j] = tabAppP[j].WithXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[nbPoles2d + j] = tabAppP2d[nbPoles2d + j].WithCoord(tabW[j], 0.0);
                    tabAppP[j].Coord(out x, out y, out z);
                    tabAppP[j] = tabAppP[j].WithCoord(dx * x + X, dy * y + Y, dz * z + Z);
                }
                if (withderiv)
                {
                    multP = new MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
                    if (i == 1)
                    {
                        cfirst = TangencyPoint;
                    }
                    else if (i == nbPoint)
                    {
                        clast = TangencyPoint;
                    }
                }
                else
                {
                    multP = new MultiPointConstraint(tabAppP, tabAppP2d);
                    if (i == 1)
                    {
                        cfirst = PassPoint;
                    }
                    else if (i == nbPoint)
                    {
                        clast = PassPoint;
                    }
                }
                multL.SetValue(i, multP);
            }

            // IFV 04.06.07 occ13904
            if (nbPoint == 2)
            {
                _dmin = 1;
                if (cfirst == PassPoint && clast == PassPoint)
                {
                    _dmax = 1;
                }
            }

            //  modified by EAP Thu Jan  3 15:44:13 2002 ___BEGIN___
            double ufloc = 0.0, ulloc = 0.0;
            Compute theapprox = new(_dmin, _dmax, _tol3d, _tol2d, _nbit);
            if (_knownp)
            {
                math_Vector theParams = new(1, nbPoint);
                // On recale les parametres entre 0 et 1.
                ufloc = f.Parameter(lin.Point(stPoint));
                ulloc = f.Parameter(lin.Point(stPoint + nbPoint - 1));
                //  modified by EAP Thu Jan  3 15:45:17 2002 ___END___
                for (i = 1; i <= nbPoint; i++)
                {
                    int iLin = stPoint + i - 1;
                    theParams[i] = (f.Parameter(lin.Point(iLin)) - ufloc) / (ulloc - ufloc);
                }
                Compute theAppDef1 = new(theParams, _dmin, _dmax, _tol3d, _tol2d, _nbit, true, true);
                theapprox = theAppDef1;
            }
            theapprox.SetConstraints(cfirst, clast);
            theapprox.Perform(multL);

            //  modified by EAP Thu Jan  3 16:00:43 2002 ___BEGIN___
            // To know internal parameters if multicurve is approximated by several Bezier's
            NCollection_Sequence<double> poleDistSeq = new();
            double wholeDist = 0;
            //  modified by EAP Thu Jan  3 16:45:48 2002 ___END___
            double theTol3d, theTol2d;
            for (int index = 1; index <= theapprox.MultiCurveCount; index++)
            {
                MultiCurve mucu = theapprox.ChangeValue(index);
                theapprox.Error(index, out theTol3d, out theTol2d);
                mytol3d = Max(theTol3d / dx, mytol3d);
                mytol3d = Max(theTol3d / dy, mytol3d);
                mytol3d = Max(theTol3d / dz, mytol3d);
                for (j = 1; j <= nbUPoles; j++)
                {
                    mucu.Transform(j, -X / dx, 1.0 / dx, -Y / dy, 1.0 / dy, -Z / dz, 1.0 / dz);
                }
                for (j = 1; j <= nbPoles2d; j++)
                {
                    mucu.Transform2d(j + nbUPoles, -x2d[j] / dx2d[j], 1.0 / dx2d[j], -y2d[j] / dy2d[j], 1.0 / dy2d[j]);
                    mytol2d = Max(theTol2d / dx2d[j], mytol2d);
                    mytol2d = Max(theTol2d / dy2d[j], mytol2d);
                }
                concat.Append(mucu);

                //  modified by EAP Thu Jan  3 15:45:23 2002 ___BEGIN___
                if (_knownp && theapprox.MultiCurveCount > 1)
                {
                    // OCCT: the poles of curve Index (the piece's number), not of the first curve (occt-issues.md)
                    Point3D firstPole = mucu.Pole(index, 1);
                    Point3D lastPole = mucu.Pole(index, mucu.NbPoles());
                    poleDistSeq.Append(firstPole.DistanceTo(lastPole));
                    wholeDist += poleDistSeq.Last();
                }
            }
            if (_knownp)
            {
                int iDist;
                double iU = ufloc;
                for (iDist = 1; iDist < poleDistSeq.Length(); iDist++)
                {
                    iU += poleDistSeq[iDist] / wholeDist * (ulloc - ufloc);
                    paramSeq.Append(iU);
                }
                paramSeq.Append(ulloc);
            }
            //  modified by EAP Thu Jan  3 15:45:27 2002 ___END___
        }
        _tol3dreached = mytol3d;
        _tol2dreached = mytol2d;
        concat.Perform();
        MultiBSpCurve multC = concat.Value();
        _vdeg = multC.Degree();
        nbVPoles = multC.NbPoles();

        _tabPoles = new NCollection_HArray2<Point3D>(1, nbUPoles, 1, nbVPoles);
        _tabWeights = new NCollection_HArray2<double>(1, nbUPoles, 1, nbVPoles);
        NCollection_Array1<double> vKnots = multC.Knots();
        _tabVKnots = new NCollection_HArray1<double>(vKnots.Lower(), vKnots.Upper());
        _tabVKnots.ChangeArray1().Assign(vKnots);

        if (_knownp)
        {
            //  modified by EAP Fri Jan  4 12:07:30 2002 ___BEGIN___
            if (paramSeq.Length() != _tabVKnots.Length())
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(f.Parameter(lin.Point(1)), f.Parameter(lin.Point(lin.NbPoints())),
                                       _tabVKnots.ChangeArray1());
            }
            else
            {
                int iKnot, iTabKnot = _tabVKnots.Lower();
                for (iKnot = 1; iKnot <= paramSeq.Length(); iKnot++, iTabKnot++)
                {
                    _tabVKnots.SetValue(iTabKnot, paramSeq[iKnot]);
                }
            }
            //  modified by EAP Fri Jan  4 12:07:35 2002 ___END___
        }

        NCollection_Array1<int> vMults = multC.Multiplicities();
        _tabVMults = new NCollection_HArray1<int>(vMults.Lower(), vMults.Upper());
        _tabVMults.ChangeArray1().Assign(vMults);

        NCollection_Array1<Point3D> newtabP = new(1, nbVPoles);
        NCollection_HArray1<Point2D> newtabP2d = new(1, nbVPoles);
        for (j = 1; j <= nbUPoles; j++)
        {
            multC.Curve(j, newtabP);
            multC.Curve(j + nbUPoles + nbPoles2d, newtabP2d.ChangeArray1());
            for (k = 1; k <= nbVPoles; k++)
            {
                // pour les courbes rationnelles il faut maintenant diviser
                // les poles par leurs poids respectifs
                _tabPoles[j, k] = _tabPoles[j, k].WithXYZ(newtabP[k].XYZ() / newtabP2d.Value(k).X);
                double weight = newtabP2d.Value(k).X;
                if (weight < Constants.Resolution())
                {
                    _done = false;
                    return;
                }
                _tabWeights.SetValue(j, k, weight);
            }
        }

        for (j = 1; j <= nbPoles2d; j++)
        {
            newtabP2d = new NCollection_HArray1<Point2D>(1, nbVPoles);
            multC.Curve(nbUPoles + j, newtabP2d.ChangeArray1());
            _seqPoles2d.Append(newtabP2d);
        }

        _done = true;
    }

    public override bool IsDone => _done;

    public override void SurfShape(out int uDegree, out int vDegree, out int nbUPoles, out int nbVPoles,
                                   out int nbUKnots, out int nbVKnots)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        uDegree = _udeg;
        vDegree = _vdeg;
        nbUPoles = _tabPoles!.ColLength();
        nbVPoles = _tabPoles.RowLength();
        nbUKnots = _tabUKnots!.Length();
        nbVKnots = _tabVKnots!.Length();
    }

    public override void Surface(NCollection_Array2<Point3D> tPoles, NCollection_Array2<double> tWeights,
                                 NCollection_Array1<double> tuKnots, NCollection_Array1<double> tvKnots,
                                 NCollection_Array1<int> tuMults, NCollection_Array1<int> tvMults)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        tPoles.Assign(_tabPoles!.Array2());
        tWeights.Assign(_tabWeights!.Array2());
        tuKnots.Assign(_tabUKnots!.Array1());
        tuMults.Assign(_tabUMults!.Array1());
        tvKnots.Assign(_tabVKnots!.Array1());
        tvMults.Assign(_tabVMults!.Array1());
    }

    public override int UDegree()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _udeg;
    }

    public override int VDegree()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _vdeg;
    }

    public override NCollection_Array2<Point3D> SurfPoles()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabPoles!.Array2();
    }

    public override NCollection_Array2<double> SurfWeights()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabWeights!.Array2();
    }

    public override NCollection_Array1<double> SurfUKnots()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabUKnots!.Array1();
    }

    public override NCollection_Array1<double> SurfVKnots()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabVKnots!.Array1();
    }

    public override NCollection_Array1<int> SurfUMults()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabUMults!.Array1();
    }

    public override NCollection_Array1<int> SurfVMults()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tabVMults!.Array1();
    }

    public override int Curves2dCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _seqPoles2d.Length();
        }
    }

    public override void Curves2dShape(out int degree, out int nbPoles, out int nbKnots)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        degree = _vdeg;
        // OCCT: the number of U poles, where the 2d curves have the V poles (occt-issues.md)
        nbPoles = _tabPoles!.ColLength();
        nbKnots = _tabVKnots!.Length();
    }

    public override void Curve2d(int index, NCollection_Array1<Point2D> tPoles, NCollection_Array1<double> tKnots,
                                 NCollection_Array1<int> tMults)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        tPoles.Assign(_seqPoles2d[index].Array1());
        tKnots.Assign(_tabVKnots!.Array1());
        tMults.Assign(_tabVMults!.Array1());
    }

    public override int Curves2dDegree()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _vdeg;
    }

    public override NCollection_Array1<Point2D> Curve2dPoles(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _seqPoles2d[index].Array1();
    }

    public override NCollection_Array1<double> Curves2dKnots()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _tabVKnots!.Array1();
    }

    public override NCollection_Array1<int> Curves2dMults()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return _tabVMults!.Array1();
    }

    public override void TolReached(out double tol3d, out double tol2d)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        tol3d = _tol3dreached;
        tol2d = _tol2dreached;
    }

    public override double TolCurveOnSurf(int index)
    {
        return _tol3dreached; // On ne s'embete pas !!
    }
}
