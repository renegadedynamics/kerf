// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_AppSurf.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_AppSurf.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppBlend_Debug;
using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The approximation of a surface through sections (AppBlend_AppSurf.gxx): each section, a B-spline curve with
/// optional 2d curves, is a multi-point of its poles, its 2d poles and its weights, and AppDef approximates the
/// multi-line in V (Bezier pieces joined, a B-spline least squares, or the variational smoothing).
/// </summary>
/// <remarks>
/// <para>
/// One generic class for OCCT's `.gxx` (decisions 142, 202 and 607; decision 5922): the section generator and the
/// line are instance interfaces (<see cref="IAppBlend_SectionGenerator{ThePoint}"/>,
/// <see cref="IAppBlend_Line{ThePoint}"/>) and <typeparamref name="ThePoint"/> is what the line gives for a point.
/// OCCT's instantiations are sealed subclasses: <see cref="GeomFill_AppSurf"/> here, GeomFill_AppSweep and
/// BRepBlend_AppSurf later.
/// </para>
/// <para>
/// A working object (decision 21). The arrays the accessors return are its own (decision 143): shared, don't modify.
/// </para>
/// </remarks>
public class AppBlend_AppSurf<TheSectionGenerator, TheLine, ThePoint> : AppBlend_Approx
    where TheSectionGenerator : IAppBlend_SectionGenerator<ThePoint>
    where TheLine : class, IAppBlend_Line<ThePoint>
{
    // OCCT's file-static `scal`, never changed.
    private const bool scal = true;

    private bool done;
    private int dmin;
    private int dmax;
    private double tol3d;
    private double tol2d;
    private int nbit;
    private int udeg;
    private int vdeg;
    private bool knownp;
    private NCollection_HArray2<gp_Pnt>? tabPoles;
    private NCollection_HArray2<double>? tabWeights;
    private NCollection_HArray1<double>? tabUKnots;
    private NCollection_HArray1<double>? tabVKnots;
    private NCollection_HArray1<int>? tabUMults;
    private NCollection_HArray1<int>? tabVMults;
    private readonly NCollection_Sequence<NCollection_HArray1<gp_Pnt2d>> seqPoles2d = new();
    private double tol3dreached;
    private double tol2dreached;
    private Approx_ParametrizationType paramtype;
    private GeomAbs_Shape continuity;
    private readonly double[] critweights = new double[3];

    public AppBlend_AppSurf()
    {
        done = false;
        dmin = 0;
        dmax = 0;
        tol3d = 0.0;
        tol2d = 0.0;
        nbit = 0;
        udeg = 0;
        vdeg = 0;
        knownp = false;
        tol3dreached = 0.0;
        tol2dreached = 0.0;
        paramtype = Approx_ChordLength;
        continuity = GeomAbs_C2;
        critweights[0] = 0.4;
        critweights[1] = 0.2;
        critweights[2] = 0.4;
    }

    public AppBlend_AppSurf(int Degmin, int Degmax, double Tol3d, double Tol2d, int NbIt, bool KnownParameters = false)
    {
        done = false;
        dmin = Degmin;
        dmax = Degmax;
        tol3d = Tol3d;
        tol2d = Tol2d;
        nbit = NbIt;
        udeg = 0;
        vdeg = 0;
        knownp = KnownParameters;
        tol3dreached = 0.0;
        tol2dreached = 0.0;
        paramtype = Approx_ChordLength;
        continuity = GeomAbs_C2;
        critweights[0] = 0.4;
        critweights[1] = 0.2;
        critweights[2] = 0.4;
    }

    public void Init(int Degmin, int Degmax, double Tol3d, double Tol2d, int NbIt, bool KnownParameters = false)
    {
        done = false;
        dmin = Degmin;
        dmax = Degmax;
        tol3d = Tol3d;
        tol2d = Tol2d;
        nbit = NbIt;
        knownp = KnownParameters;
        continuity = GeomAbs_C2;
        paramtype = Approx_ChordLength;
        critweights[0] = 0.4;
        critweights[1] = 0.2;
        critweights[2] = 0.4;
    }

    /// <summary>The weights of the criterion used in the optimisation (the variational smoothing).</summary>
    public void CriteriumWeight(out double W1, out double W2, out double W3)
    {
        W1 = critweights[0];
        W2 = critweights[1];
        W3 = critweights[2];
    }

    /// <summary>Sets the weights of the criterion; throws Standard_DomainError if one is negative.</summary>
    public void SetCriteriumWeight(double W1, double W2, double W3)
    {
        if (W1 < 0 || W2 < 0 || W3 < 0)
        {
            throw new Standard_DomainError();
        }
        critweights[0] = W1;
        critweights[1] = W2;
        critweights[2] = W3;
    }

    /// <summary>The continuity of the spline and smoothing approximations.</summary>
    public void SetContinuity(GeomAbs_Shape TheCont)
    {
        continuity = TheCont;
    }

    public GeomAbs_Shape Continuity() => continuity;

    /// <summary>The type of parametrisation of the approximation.</summary>
    public void SetParType(Approx_ParametrizationType ParType)
    {
        paramtype = ParType;
    }

    public Approx_ParametrizationType ParType() => paramtype;

    /// <summary>
    /// Approximates the sections of <paramref name="Lin"/>: Bezier pieces joined (AppDef_Compute), or with
    /// <paramref name="SpApprox"/> a B-spline least squares (AppDef_BSplineCompute) on every section.
    /// </summary>
    public void Perform(TheLine? Lin, TheSectionGenerator F, bool SpApprox = false)
    {
        InternalPerform(Lin, F, SpApprox, false);
    }

    /// <summary>Approximates the sections of <paramref name="Lin"/> by the variational smoothing (AppDef_Variational).</summary>
    public void PerformSmoothing(TheLine? Lin, TheSectionGenerator F)
    {
        InternalPerform(Lin, F, true, true);
    }

    private void InternalPerform(TheLine? Lin, TheSectionGenerator F, bool SpApprox, bool UseSmoothing)
    {
        done = false;
        if (Lin is null)
        {
            return;
        }
        int i, j, k, NbPoint;
        int NbUPoles, NbUKnots, NbPoles2d, NbVPoles;
        bool withderiv;
        AppParCurves_Constraint Cfirst, Clast;

        double mytol3d, mytol2d;
        gp_XYZ newDv = new();

        seqPoles2d.Clear();

        NbPoint = Lin.NbPoints();
        AppDef_MultiPointConstraint multP;
        AppDef_MultiLine multL = new(NbPoint);

        F.GetShape(out NbUPoles, out NbUKnots, out udeg, out NbPoles2d);

        tabUKnots = new NCollection_HArray1<double>(1, NbUKnots);
        tabUMults = new NCollection_HArray1<int>(1, NbUKnots);

        F.Knots(tabUKnots.ChangeArray1());
        F.Mults(tabUMults.ChangeArray1());

        NCollection_Array1<gp_Pnt> tabAppP = new(1, NbUPoles);
        NCollection_Array1<gp_Vec> tabAppV = new(1, NbUPoles);

        NCollection_Array1<gp_Pnt2d> tabP2d = new(1, Max(1, NbPoles2d));
        NCollection_Array1<gp_Vec2d> tabV2d = new(1, Max(1, NbPoles2d));

        NCollection_Array1<double> tabW = new(1, NbUPoles), tabDW = new(1, NbUPoles);

        NCollection_Array1<gp_Pnt2d> tabAppP2d = new(1, NbPoles2d + NbUPoles); // points2d + poids
        NCollection_Array1<gp_Vec2d> tabAppV2d = new(1, NbPoles2d + NbUPoles);

        AppParCurves_MultiBSpCurve multC;

        withderiv = F.Section(Lin.Point(1), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);

        if (AppBlend_GetContextApproxWithNoTgt())
        {
            withderiv = false;
        }

        for (j = 1; j <= NbPoles2d; j++)
        {
            tabAppP2d[j] = tabP2d[j];
            if (withderiv)
            {
                tabAppV2d[j] = tabV2d[j];
            }
        }
        for (j = 1; j <= NbUPoles; j++)
        {
            // pour les courbes rationnelles il faut multiplier les poles par
            // leurs poids respectifs
            if (withderiv)
            {
                tabAppV2d[NbPoles2d + j].SetCoord(tabDW[j], 0.0);
                newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                tabAppV[j].SetXYZ(newDv);
            }
            tabAppP[j].SetXYZ(tabAppP[j].XYZ() * tabW[j]);
            tabAppP2d[NbPoles2d + j].SetCoord(tabW[j], 0.0);
        }

        if (withderiv)
        {
            multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
            Cfirst = AppParCurves_TangencyPoint;
        }
        else
        {
            multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d);
            Cfirst = AppParCurves_PassPoint;
        }
        multL.SetValue(1, multP);

        for (i = 2; i <= NbPoint - 1; i++)
        {
            if (SpApprox)
            {
                F.Section(Lin.Point(i), tabAppP, tabP2d, tabW);
                for (j = 1; j <= NbPoles2d; j++)
                {
                    tabAppP2d[j] = tabP2d[j];
                }
                for (j = 1; j <= NbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    tabAppP[j].SetXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[NbPoles2d + j].SetCoord(tabW[j], 0.0);
                }
                multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d);
                multL.SetValue(i, multP);
            }
            // ***********************
            else
            {
                withderiv = F.Section(Lin.Point(i), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
                if (AppBlend_GetContextApproxWithNoTgt())
                {
                    withderiv = false;
                }

                for (j = 1; j <= NbPoles2d; j++)
                {
                    tabAppP2d[j] = tabP2d[j];
                    if (withderiv)
                    {
                        tabAppV2d[j] = tabV2d[j];
                    }
                }
                for (j = 1; j <= NbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    if (withderiv)
                    {
                        tabAppV2d[NbPoles2d + j].SetCoord(tabDW[j], 0.0);
                        newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                        tabAppV[j].SetXYZ(newDv);
                    }
                    tabAppP[j].SetXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[NbPoles2d + j].SetCoord(tabW[j], 0.0);
                }
                if (withderiv)
                {
                    multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
                }
                else
                {
                    multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d);
                }
                multL.SetValue(i, multP);
            }
            // ******************************
        }

        withderiv = F.Section(Lin.Point(NbPoint), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
        if (AppBlend_GetContextApproxWithNoTgt())
        {
            withderiv = false;
        }

        for (j = 1; j <= NbPoles2d; j++)
        {
            tabAppP2d[j] = tabP2d[j];
            if (withderiv)
            {
                tabAppV2d[j] = tabV2d[j];
            }
        }
        for (j = 1; j <= NbUPoles; j++)
        {
            // pour les courbes rationnelles il faut multiplier les poles par
            // leurs poids respectifs
            if (withderiv)
            {
                tabAppV2d[NbPoles2d + j].SetCoord(tabDW[j], 0.0);
                newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                tabAppV[j].SetXYZ(newDv);
            }
            tabAppP[j].SetXYZ(tabAppP[j].XYZ() * tabW[j]);
            tabAppP2d[NbPoles2d + j].SetCoord(tabW[j], 0.0);
        }

        if (withderiv)
        {
            multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
            Clast = AppParCurves_TangencyPoint;
        }
        else
        {
            multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d);
            Clast = AppParCurves_PassPoint;
        }
        multL.SetValue(NbPoint, multP);

        // IFV 04.06.07 occ13904
        if (NbPoint == 2)
        {
            dmin = 1;
            if (Cfirst == AppParCurves_PassPoint && Clast == AppParCurves_PassPoint)
            {
                dmax = 1;
            }
        }

        if (!SpApprox)
        {
            AppDef_Compute theapprox = new(dmin, dmax, tol3d, tol2d, nbit, true, paramtype);
            if (knownp)
            {
                math_Vector theParams = new(1, NbPoint);

                // On recale les parametres entre 0 et 1.
                theParams[1] = 0.0;
                theParams[NbPoint] = 1.0;
                double Uf = F.Parameter(Lin.Point(1));
                double Ul = F.Parameter(Lin.Point(NbPoint)) - Uf;
                for (i = 2; i < NbPoint; i++)
                {
                    theParams[i] = (F.Parameter(Lin.Point(i)) - Uf) / Ul;
                }
                AppDef_Compute theAppDef = new(theParams, dmin, dmax, tol3d, tol2d, nbit, true, true);
                theapprox = theAppDef;
            }
            theapprox.SetConstraints(Cfirst, Clast);
            theapprox.Perform(multL);

            double TheTol3d, TheTol2d;
            mytol3d = mytol2d = 0.0;
            for (int Index = 1; Index <= theapprox.NbMultiCurves(); Index++)
            {
                theapprox.Error(Index, out TheTol3d, out TheTol2d);
                mytol3d = Max(TheTol3d, mytol3d);
                mytol2d = Max(TheTol2d, mytol2d);
            }
            // OCCT: the tolerances reached are not stored on this path (TolReached keeps the previous values).
            multC = theapprox.SplineValue();
        }
        else
        {
            if (!UseSmoothing)
            {
                bool UseSquares = false;
                if (nbit == 0)
                {
                    UseSquares = true;
                }
                AppDef_BSplineCompute theapprox = new(dmin, dmax, tol3d, tol2d, nbit, true, paramtype, UseSquares);
                if (continuity == GeomAbs_C0)
                {
                    theapprox.SetContinuity(0);
                }
                if (continuity == GeomAbs_C1)
                {
                    theapprox.SetContinuity(1);
                }
                else if (continuity == GeomAbs_C2)
                {
                    theapprox.SetContinuity(2);
                }
                else
                {
                    // OCCT: C0 falls through to here too, so it ends as continuity 3 (occt-issues.md).
                    theapprox.SetContinuity(3);
                }

                theapprox.SetConstraints(Cfirst, Clast);

                if (knownp)
                {
                    math_Vector theParams = new(1, NbPoint);
                    // On recale les parametres entre 0 et 1.
                    theParams[1] = 0.0;
                    theParams[NbPoint] = 1.0;
                    double Uf = F.Parameter(Lin.Point(1));
                    double Ul = F.Parameter(Lin.Point(NbPoint)) - Uf;
                    for (i = 2; i < NbPoint; i++)
                    {
                        theParams[i] = (F.Parameter(Lin.Point(i)) - Uf) / Ul;
                    }

                    theapprox.Init(dmin, dmax, tol3d, tol2d, nbit, true, Approx_IsoParametric, true);
                    theapprox.SetParameters(theParams);
                }
                theapprox.Perform(multL);
                theapprox.Error(out mytol3d, out mytol2d);
                tol3dreached = mytol3d;
                tol2dreached = mytol2d;
                multC = theapprox.Value();
            }
            else
            {
                // Variational algo
                NCollection_HArray1<AppParCurves_ConstraintCouple> TABofCC = new(1, NbPoint);
                AppParCurves_Constraint Constraint = AppParCurves_NoConstraint;

                for (i = 1; i <= NbPoint; ++i)
                {
                    AppParCurves_ConstraintCouple ACC = new(i, Constraint);
                    TABofCC.SetValue(i, ACC);
                }

                TABofCC.ChangeValue(1).SetConstraint(Cfirst);
                TABofCC.ChangeValue(NbPoint).SetConstraint(Clast);

                AppDef_Variational Variation = new(multL, 1, NbPoint, TABofCC);

                //===================================
                int theMaxSegments = 1000;
                bool theWithMinMax = false;
                bool theWithCutting = true;
                //===================================

                Variation.SetMaxDegree(dmax);
                Variation.SetContinuity(continuity);
                Variation.SetMaxSegment(theMaxSegments);

                Variation.SetTolerance(tol3d);
                Variation.SetWithMinMax(theWithMinMax);
                Variation.SetWithCutting(theWithCutting);
                Variation.SetNbIterations(nbit);

                Variation.SetCriteriumWeight(critweights[0], critweights[1], critweights[2]);

                if (!Variation.IsCreated())
                {
                    return;
                }

                if (Variation.IsOverConstrained())
                {
                    return;
                }

                try
                {
                    Variation.Approximate();
                }
                catch (Standard_Failure)
                {
                    return;
                }

                if (!Variation.IsDone())
                {
                    return;
                }

                mytol3d = Variation.MaxError();
                mytol2d = 0.0;
                tol3dreached = mytol3d;
                tol2dreached = mytol2d;
                multC = Variation.Value();
            }
        }

        vdeg = multC.Degree();
        NbVPoles = multC.NbPoles();

        tabPoles = new NCollection_HArray2<gp_Pnt>(1, NbUPoles, 1, NbVPoles);
        tabWeights = new NCollection_HArray2<double>(1, NbUPoles, 1, NbVPoles);
        NCollection_Array1<double> aVKnots = multC.Knots();
        tabVKnots = new NCollection_HArray1<double>(aVKnots.Lower(), aVKnots.Upper());
        tabVKnots.ChangeArray1().Assign(aVKnots);

        if (knownp && !UseSmoothing)
        {
            BSplCLib.Reparametrize(F.Parameter(Lin.Point(1)), F.Parameter(Lin.Point(NbPoint)), tabVKnots.ChangeArray1());
        }

        NCollection_Array1<int> aVMults = multC.Multiplicities();
        tabVMults = new NCollection_HArray1<int>(aVMults.Lower(), aVMults.Upper());
        tabVMults.ChangeArray1().Assign(aVMults);

        NCollection_Array1<gp_Pnt> newtabP = new(1, NbVPoles);
        NCollection_HArray1<gp_Pnt2d> newtabP2d = new(1, NbVPoles);
        for (j = 1; j <= NbUPoles; j++)
        {
            multC.Curve(j, newtabP);
            multC.Curve(j + NbUPoles + NbPoles2d, newtabP2d.ChangeArray1());
            for (k = 1; k <= NbVPoles; k++)
            {
                // pour les courbes rationnelles il faut maintenant diviser
                // les poles par leurs poids respectifs
                tabPoles[j, k].SetXYZ(newtabP[k].XYZ() / newtabP2d.Value(k).X());
                double aWeight = newtabP2d.Value(k).X();
                if (aWeight < gp.Resolution())
                {
                    done = false;
                    return;
                }
                tabWeights.SetValue(j, k, aWeight);
            }
        }

        for (j = 1; j <= NbPoles2d; j++)
        {
            newtabP2d = new NCollection_HArray1<gp_Pnt2d>(1, NbVPoles);
            multC.Curve(NbUPoles + j, newtabP2d.ChangeArray1());
            seqPoles2d.Append(newtabP2d);
        }

        done = true;
    }

    /// <summary>
    /// Approximates the sections of <paramref name="Lin"/> by pieces of at most about <paramref name="NbMaxP"/>
    /// sections each (each scaled to the unit box and approximated by AppDef_Compute), joined into one B-spline.
    /// </summary>
    public void Perform(TheLine? Lin, TheSectionGenerator F, int NbMaxP)
    {
        done = false;
        if (Lin is null)
        {
            return;
        }
        int i, j, k;
        int NbUPoles, NbUKnots, NbPoles2d, NbVPoles;
        bool withderiv;
        AppParCurves_Constraint Cfirst = AppParCurves_NoConstraint, Clast = AppParCurves_NoConstraint;

        double mytol3d = 0.0, mytol2d = 0.0;
        gp_XYZ newDv = new();

        seqPoles2d.Clear();

        int NbPointTot = Lin.NbPoints();

        F.GetShape(out NbUPoles, out NbUKnots, out udeg, out NbPoles2d);

        tabUKnots = new NCollection_HArray1<double>(1, NbUKnots);
        tabUMults = new NCollection_HArray1<int>(1, NbUKnots);

        F.Knots(tabUKnots.ChangeArray1());
        F.Mults(tabUMults.ChangeArray1());

        NCollection_Array1<gp_Pnt> tabAppP = new(1, NbUPoles);
        NCollection_Array1<gp_Vec> tabAppV = new(1, NbUPoles);
        double X, Y, Z, DX, DY, DZ;
        X = Y = Z = RealLast();
        DX = DY = DZ = RealFirst();

        NCollection_Array1<gp_Pnt2d> tabP2d = new(1, Max(1, NbPoles2d));
        NCollection_Array1<gp_Vec2d> tabV2d = new(1, Max(1, NbPoles2d));
        NCollection_Array1<double> X2d = new(1, Max(1, NbPoles2d));
        X2d.Init(RealLast());
        NCollection_Array1<double> Y2d = new(1, Max(1, NbPoles2d));
        Y2d.Init(RealLast());
        NCollection_Array1<double> DX2d = new(1, Max(1, NbPoles2d));
        DX2d.Init(RealFirst());
        NCollection_Array1<double> DY2d = new(1, Max(1, NbPoles2d));
        DY2d.Init(RealFirst());

        NCollection_Array1<double> tabW = new(1, NbUPoles), tabDW = new(1, NbUPoles);

        NCollection_Array1<gp_Pnt2d> tabAppP2d = new(1, NbPoles2d + NbUPoles); // points2d + poids
        NCollection_Array1<gp_Vec2d> tabAppV2d = new(1, NbPoles2d + NbUPoles);

        // On calcule les boites de chaque ligne (box for all lines)
        for (i = 1; i <= NbPointTot; i++)
        {
            F.Section(Lin.Point(i), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
            double x, y, z;
            for (j = 1; j <= NbUPoles; j++)
            {
                tabAppP[j].Coord(out x, out y, out z);
                if (x < X)
                {
                    X = x;
                }
                if (x > DX)
                {
                    DX = x;
                }
                if (y < Y)
                {
                    Y = y;
                }
                if (y > DY)
                {
                    DY = y;
                }
                if (z < Z)
                {
                    Z = z;
                }
                if (z > DZ)
                {
                    DZ = z;
                }
            }
            for (j = 1; j <= NbPoles2d; j++)
            {
                tabP2d[j].Coord(out x, out y);
                if (x < X2d[j])
                {
                    X2d[j] = x;
                }
                if (x > DX2d[j])
                {
                    DX2d[j] = x;
                }
                if (y < Y2d[j])
                {
                    Y2d[j] = y;
                }
                if (y > DY2d[j])
                {
                    DY2d[j] = y;
                }
            }
        }
        // On calcule pour chaque ligne la transformation vers 0 1.
        double seuil = 1000.0 * tol3d;
        double seuil2d = 1000.0 * tol2d;
        if ((DX - X) < seuil)
        {
            DX = 1.0;
            X = 0.0;
        }
        else
        {
            DX = 1.0 / (DX - X);
            X *= -DX;
        }
        if ((DY - Y) < seuil)
        {
            DY = 1.0;
            Y = 0.0;
        }
        else
        {
            DY = 1.0 / (DY - Y);
            Y *= -DY;
        }
        if ((DZ - Z) < seuil)
        {
            DZ = 1.0;
            Z = 0.0;
        }
        else
        {
            DZ = 1.0 / (DZ - Z);
            Z *= -DZ;
        }
        for (j = 1; j <= NbPoles2d; j++)
        {
            if ((DX2d[j] - X2d[j]) < seuil2d)
            {
                DX2d[j] = 1.0;
                X2d[j] = 0.0;
            }
            else
            {
                DX2d[j] = 1.0 / (DX2d[j] - X2d[j]);
                X2d[j] *= -DX2d[j];
            }
            if ((DY2d[j] - Y2d[j]) < seuil2d)
            {
                DY2d[j] = 1.0;
                Y2d[j] = 0.0;
            }
            else
            {
                DY2d[j] = 1.0 / (DY2d[j] - Y2d[j]);
                Y2d[j] *= -DY2d[j];
            }
        }
#pragma warning disable CS0162 // OCCT's `scal` is always on: the unscaled branch is dead, kept as OCCT has it
        if (!scal)
        {
            DX = 1.0;
            X = 0.0;
            DY = 1.0;
            Y = 0.0;
            DZ = 1.0;
            Z = 0.0;
            for (j = 1; j <= NbPoles2d; j++)
            {
                DX2d[j] = 1.0;
                X2d[j] = 0.0;
                DY2d[j] = 1.0;
                Y2d[j] = 0.0;
            }
        }
#pragma warning restore CS0162
        //  modified by eap Thu Jan  3 14:45:22 2002 ___BEGIN___
        // Keep "inter-troncons" parameters, not only first and last
        NCollection_Sequence<double> aParamSeq = new();
        if (knownp)
        {
            aParamSeq.Append(F.Parameter(Lin.Point(1)));
        }
        //  modified by EAP Thu Jan  3 14:45:41 2002 ___END___

        Approx_MCurvesToBSpCurve concat = new();

        // On calcule le nombre de troncons.
        int nbtronc = NbPointTot / NbMaxP;
        int reste = NbPointTot - (nbtronc * NbMaxP);
        // On regarde si il faut prendre un troncon de plus.
        int nmax = NbMaxP;
        if (nbtronc > 0 && reste > 0)
        {
            nmax = NbPointTot / (nbtronc + 1);
            if (nmax > (2 * NbMaxP) / 3)
            {
                nbtronc++;
                reste = NbPointTot - (nbtronc * nmax);
            }
            else
            {
                nmax = NbMaxP;
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
            int NbPoint = troncsize[itronc];
            int StPoint = troncstart[itronc];
            AppDef_MultiPointConstraint multP;
            AppDef_MultiLine multL = new(NbPoint);

            for (i = 1; i <= NbPoint; i++)
            {
                int iLin = StPoint + i - 1;
                double x, y, z;
                withderiv = F.Section(Lin.Point(iLin), tabAppP, tabAppV, tabP2d, tabV2d, tabW, tabDW);
                if (AppBlend_GetContextApproxWithNoTgt())
                {
                    withderiv = false;
                }

                for (j = 1; j <= NbPoles2d; j++)
                {
                    tabP2d[j].Coord(out x, out y);
                    tabAppP2d[j].SetCoord(DX2d[j] * x + X2d[j], DY2d[j] * y + Y2d[j]);
                    if (withderiv)
                    {
                        tabV2d[j].Coord(out x, out y);
                        tabAppV2d[j].SetCoord(DX2d[j] * x, DY2d[j] * y);
                    }
                }
                for (j = 1; j <= NbUPoles; j++)
                {
                    // pour les courbes rationnelles il faut multiplier les poles par
                    // leurs poids respectifs
                    if (withderiv)
                    {
                        tabAppV2d[NbPoles2d + j].SetCoord(tabDW[j], 0.0);
                        newDv.SetLinearForm(tabDW[j], tabAppP[j].XYZ(), tabW[j], tabAppV[j].XYZ());
                        tabAppV[j].SetCoord(DX * newDv.X(), DY * newDv.Y(), DZ * newDv.Z());
                    }
                    tabAppP[j].SetXYZ(tabAppP[j].XYZ() * tabW[j]);
                    tabAppP2d[NbPoles2d + j].SetCoord(tabW[j], 0.0);
                    tabAppP[j].Coord(out x, out y, out z);
                    tabAppP[j].SetCoord(DX * x + X, DY * y + Y, DZ * z + Z);
                }
                if (withderiv)
                {
                    multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d, tabAppV, tabAppV2d);
                    if (i == 1)
                    {
                        Cfirst = AppParCurves_TangencyPoint;
                    }
                    else if (i == NbPoint)
                    {
                        Clast = AppParCurves_TangencyPoint;
                    }
                }
                else
                {
                    multP = new AppDef_MultiPointConstraint(tabAppP, tabAppP2d);
                    if (i == 1)
                    {
                        Cfirst = AppParCurves_PassPoint;
                    }
                    else if (i == NbPoint)
                    {
                        Clast = AppParCurves_PassPoint;
                    }
                }
                multL.SetValue(i, multP);
            }

            // IFV 04.06.07 occ13904
            if (NbPoint == 2)
            {
                dmin = 1;
                if (Cfirst == AppParCurves_PassPoint && Clast == AppParCurves_PassPoint)
                {
                    dmax = 1;
                }
            }

            //  modified by EAP Thu Jan  3 15:44:13 2002 ___BEGIN___
            double Ufloc = 0.0, Ulloc = 0.0;
            AppDef_Compute theapprox = new(dmin, dmax, tol3d, tol2d, nbit);
            if (knownp)
            {
                math_Vector theParams = new(1, NbPoint);
                // On recale les parametres entre 0 et 1.
                Ufloc = F.Parameter(Lin.Point(StPoint));
                Ulloc = F.Parameter(Lin.Point(StPoint + NbPoint - 1));
                //  modified by EAP Thu Jan  3 15:45:17 2002 ___END___
                for (i = 1; i <= NbPoint; i++)
                {
                    int iLin = StPoint + i - 1;
                    theParams[i] = (F.Parameter(Lin.Point(iLin)) - Ufloc) / (Ulloc - Ufloc);
                }
                AppDef_Compute theAppDef1 = new(theParams, dmin, dmax, tol3d, tol2d, nbit, true, true);
                theapprox = theAppDef1;
            }
            theapprox.SetConstraints(Cfirst, Clast);
            theapprox.Perform(multL);

            //  modified by EAP Thu Jan  3 16:00:43 2002 ___BEGIN___
            // To know internal parameters if multicurve is approximated by several Bezier's
            NCollection_Sequence<double> aPoleDistSeq = new();
            double aWholeDist = 0;
            //  modified by EAP Thu Jan  3 16:45:48 2002 ___END___
            double TheTol3d, TheTol2d;
            for (int Index = 1; Index <= theapprox.NbMultiCurves(); Index++)
            {
                AppParCurves_MultiCurve mucu = theapprox.ChangeValue(Index);
                theapprox.Error(Index, out TheTol3d, out TheTol2d);
                mytol3d = Max(TheTol3d / DX, mytol3d);
                mytol3d = Max(TheTol3d / DY, mytol3d);
                mytol3d = Max(TheTol3d / DZ, mytol3d);
                for (j = 1; j <= NbUPoles; j++)
                {
                    mucu.Transform(j, -X / DX, 1.0 / DX, -Y / DY, 1.0 / DY, -Z / DZ, 1.0 / DZ);
                }
                for (j = 1; j <= NbPoles2d; j++)
                {
                    mucu.Transform2d(j + NbUPoles, -X2d[j] / DX2d[j], 1.0 / DX2d[j], -Y2d[j] / DY2d[j], 1.0 / DY2d[j]);
                    mytol2d = Max(TheTol2d / DX2d[j], mytol2d);
                    mytol2d = Max(TheTol2d / DY2d[j], mytol2d);
                }
                concat.Append(mucu);

                //  modified by EAP Thu Jan  3 15:45:23 2002 ___BEGIN___
                if (knownp && theapprox.NbMultiCurves() > 1)
                {
                    // OCCT: the poles of curve Index (the piece's number), not of the first curve (occt-issues.md)
                    gp_Pnt aFirstPole = mucu.Pole(Index, 1);
                    gp_Pnt aLastPole = mucu.Pole(Index, mucu.NbPoles());
                    aPoleDistSeq.Append(aFirstPole.Distance(aLastPole));
                    aWholeDist += aPoleDistSeq.Last();
                }
            }
            if (knownp)
            {
                int iDist;
                double iU = Ufloc;
                for (iDist = 1; iDist < aPoleDistSeq.Length(); iDist++)
                {
                    iU += aPoleDistSeq[iDist] / aWholeDist * (Ulloc - Ufloc);
                    aParamSeq.Append(iU);
                }
                aParamSeq.Append(Ulloc);
            }
            //  modified by EAP Thu Jan  3 15:45:27 2002 ___END___
        }
        tol3dreached = mytol3d;
        tol2dreached = mytol2d;
        concat.Perform();
        AppParCurves_MultiBSpCurve multC = concat.Value();
        vdeg = multC.Degree();
        NbVPoles = multC.NbPoles();

        tabPoles = new NCollection_HArray2<gp_Pnt>(1, NbUPoles, 1, NbVPoles);
        tabWeights = new NCollection_HArray2<double>(1, NbUPoles, 1, NbVPoles);
        NCollection_Array1<double> aVKnots = multC.Knots();
        tabVKnots = new NCollection_HArray1<double>(aVKnots.Lower(), aVKnots.Upper());
        tabVKnots.ChangeArray1().Assign(aVKnots);

        if (knownp)
        {
            //  modified by EAP Fri Jan  4 12:07:30 2002 ___BEGIN___
            if (aParamSeq.Length() != tabVKnots.Length())
            {
                BSplCLib.Reparametrize(F.Parameter(Lin.Point(1)), F.Parameter(Lin.Point(Lin.NbPoints())),
                                       tabVKnots.ChangeArray1());
            }
            else
            {
                int iKnot, iTabKnot = tabVKnots.Lower();
                for (iKnot = 1; iKnot <= aParamSeq.Length(); iKnot++, iTabKnot++)
                {
                    tabVKnots.SetValue(iTabKnot, aParamSeq[iKnot]);
                }
            }
            //  modified by EAP Fri Jan  4 12:07:35 2002 ___END___
        }

        NCollection_Array1<int> aVMults = multC.Multiplicities();
        tabVMults = new NCollection_HArray1<int>(aVMults.Lower(), aVMults.Upper());
        tabVMults.ChangeArray1().Assign(aVMults);

        NCollection_Array1<gp_Pnt> newtabP = new(1, NbVPoles);
        NCollection_HArray1<gp_Pnt2d> newtabP2d = new(1, NbVPoles);
        for (j = 1; j <= NbUPoles; j++)
        {
            multC.Curve(j, newtabP);
            multC.Curve(j + NbUPoles + NbPoles2d, newtabP2d.ChangeArray1());
            for (k = 1; k <= NbVPoles; k++)
            {
                // pour les courbes rationnelles il faut maintenant diviser
                // les poles par leurs poids respectifs
                tabPoles[j, k].SetXYZ(newtabP[k].XYZ() / newtabP2d.Value(k).X());
                double aWeight = newtabP2d.Value(k).X();
                if (aWeight < gp.Resolution())
                {
                    done = false;
                    return;
                }
                tabWeights.SetValue(j, k, aWeight);
            }
        }

        for (j = 1; j <= NbPoles2d; j++)
        {
            newtabP2d = new NCollection_HArray1<gp_Pnt2d>(1, NbVPoles);
            multC.Curve(NbUPoles + j, newtabP2d.ChangeArray1());
            seqPoles2d.Append(newtabP2d);
        }

        done = true;
    }

    public override bool IsDone() => done;

    public override void SurfShape(out int UDegree, out int VDegree, out int NbUPoles, out int NbVPoles,
                                   out int NbUKnots, out int NbVKnots)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        UDegree = udeg;
        VDegree = vdeg;
        NbUPoles = tabPoles!.ColLength();
        NbVPoles = tabPoles.RowLength();
        NbUKnots = tabUKnots!.Length();
        NbVKnots = tabVKnots!.Length();
    }

    public override void Surface(NCollection_Array2<gp_Pnt> TPoles, NCollection_Array2<double> TWeights,
                                 NCollection_Array1<double> TUKnots, NCollection_Array1<double> TVKnots,
                                 NCollection_Array1<int> TUMults, NCollection_Array1<int> TVMults)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        TPoles.Assign(tabPoles!.Array2());
        TWeights.Assign(tabWeights!.Array2());
        TUKnots.Assign(tabUKnots!.Array1());
        TUMults.Assign(tabUMults!.Array1());
        TVKnots.Assign(tabVKnots!.Array1());
        TVMults.Assign(tabVMults!.Array1());
    }

    public override int UDegree()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return udeg;
    }

    public override int VDegree()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return vdeg;
    }

    public override NCollection_Array2<gp_Pnt> SurfPoles()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabPoles!.Array2();
    }

    public override NCollection_Array2<double> SurfWeights()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabWeights!.Array2();
    }

    public override NCollection_Array1<double> SurfUKnots()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabUKnots!.Array1();
    }

    public override NCollection_Array1<double> SurfVKnots()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabVKnots!.Array1();
    }

    public override NCollection_Array1<int> SurfUMults()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabUMults!.Array1();
    }

    public override NCollection_Array1<int> SurfVMults()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tabVMults!.Array1();
    }

    public override int NbCurves2d()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return seqPoles2d.Length();
    }

    public override void Curves2dShape(out int Degree, out int NbPoles, out int NbKnots)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        Degree = vdeg;
        // OCCT: the number of U poles, where the 2d curves have the V poles (occt-issues.md)
        NbPoles = tabPoles!.ColLength();
        NbKnots = tabVKnots!.Length();
    }

    public override void Curve2d(int Index, NCollection_Array1<gp_Pnt2d> TPoles, NCollection_Array1<double> TKnots,
                                 NCollection_Array1<int> TMults)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        TPoles.Assign(seqPoles2d[Index].Array1());
        TKnots.Assign(tabVKnots!.Array1());
        TMults.Assign(tabVMults!.Array1());
    }

    public override int Curves2dDegree()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        return vdeg;
    }

    public override NCollection_Array1<gp_Pnt2d> Curve2dPoles(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        return seqPoles2d[Index].Array1();
    }

    public override NCollection_Array1<double> Curves2dKnots()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        return tabVKnots!.Array1();
    }

    public override NCollection_Array1<int> Curves2dMults()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new Standard_DomainError();
        }
        return tabVMults!.Array1();
    }

    public override void TolReached(out double Tol3d, out double Tol2d)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        Tol3d = tol3dreached;
        Tol2d = tol2dreached;
    }

    public override double TolCurveOnSurf(int Index)
    {
        return tol3dreached; // On ne s'embete pas !!
    }
}
