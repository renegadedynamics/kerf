// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSplineSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;
using static Occt.GeomAbs_Shape;

namespace Occt;

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
public class GeomAPI_PointsToBSplineSurface
{
    private bool myIsDone;
    private Geom_BSplineSurface? mySurface;

    private static bool hasMeaningfulSpan(double theSpan)
    {
        double aResolution = gp.Resolution();
        return theSpan * theSpan > aResolution * aResolution;
    }

    private static void BuildParameters(AppDef_MultiLine theLine,
                                        Approx_ParametrizationType theParT,
                                        NCollection_Array1<double> thePars)
    {
        int i, j, nbP3d = theLine.NbPoints();
        double dist;
        int firstP = 1, lastP = theLine.NbMultiPoints();
        int aNbp = lastP - firstP + 1;

        if (aNbp == 2)
        {
            thePars[firstP] = 0.0;
            thePars[lastP] = 1.0;
        }
        else if (theParT == Approx_ChordLength || theParT == Approx_Centripetal)
        {
            thePars[firstP] = 0.0;
            dist = 0.0;

            for (i = firstP + 1; i <= lastP; i++)
            {
                AppDef_MultiPointConstraint aMPC = theLine.Value(i - 1);
                AppDef_MultiPointConstraint aMPC1 = theLine.Value(i);

                dist = 0.0;
                for (j = 1; j <= nbP3d; j++)
                {
                    gp_Pnt aP1 = aMPC.Point(j), aP2 = aMPC1.Point(j);
                    dist += aP2.SquareDistance(aP1);
                }

                dist = Math.Sqrt(dist);
                if (theParT == Approx_ChordLength)
                {
                    thePars[i] = thePars[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    thePars[i] = thePars[i - 1] + Math.Sqrt(dist);
                }
            }
            if (hasMeaningfulSpan(thePars[lastP]))
            {
                for (i = firstP; i <= lastP; i++)
                {
                    thePars[i] /= thePars[lastP];
                }
            }
            else
            {
                for (i = firstP; i <= lastP; i++)
                {
                    thePars[i] = ((double)i - firstP) / ((double)(lastP - firstP));
                }
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                thePars[i] = ((double)i - firstP) / ((double)(lastP - (double)firstP));
            }
        }
    }

    private static void BuildPeriodicTangent(AppDef_MultiLine theLine,
                                             NCollection_Array1<double> thePars,
                                             math_Vector theTang)
    {
        int firstpt = 1, lastpt = theLine.NbMultiPoints();
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
        double U;
        AppParCurves_Constraint Cons = AppParCurves_TangencyPoint;
        if (nnp <= 4)
        {
            Cons = AppParCurves_PassPoint;
        }
        int nbP = 3 * theLine.NbPoints();
        math_Vector V1 = new(1, nbP), V2 = new(1, nbP);
        math_Vector P1 = new(firstpt, lastp);
        //
        for (i = firstpt; i <= lastp; i++)
        {
            P1[i] = thePars[i];
        }

        AppDef_BSpParLeastSquareOfMyBSplGradientOfBSplineCompute SQ1 = new(theLine,
                                                                          firstpt,
                                                                          lastp,
                                                                          Cons,
                                                                          Cons,
                                                                          nnpol);
        SQ1.Perform(P1);
        AppParCurves_MultiCurve C1 = SQ1.BezierValue();
        U = 0.0;
        int j, nbP3d = theLine.NbPoints();

        gp_Pnt aP;
        gp_Vec aV;
        j = 1;
        for (i = 1; i <= nbP3d; i++)
        {
            C1.D1(i, U, out aP, out aV);
            V1[j] = aV.X();
            V1[j + 1] = aV.Y();
            V1[j + 2] = aV.Z();
            j += 3;
        }

        int firstp = Math.Max(firstpt, lastpt - nnp + 1);

        if (firstp == firstpt && lastp == lastpt)
        {
            U = 1.0;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C1.D1(i, U, out aP, out aV);
                V2[j] = aV.X();
                V2[j + 1] = aV.Y();
                V2[j + 2] = aV.Z();
                j += 3;
            }
        }
        else
        {
            AppDef_BSpParLeastSquareOfMyBSplGradientOfBSplineCompute SQ2 = new(theLine,
                                                                              firstp,
                                                                              lastpt,
                                                                              Cons,
                                                                              Cons,
                                                                              nnpol);

            math_Vector P2 = new(firstp, lastpt);
            for (i = firstp; i <= lastpt; i++)
            {
                P2[i] = thePars[i];
            }
            SQ2.Perform(P2);

            AppParCurves_MultiCurve C2 = SQ2.BezierValue();
            U = 1.0;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C2.D1(i, U, out aP, out aV);
                V2[j] = aV.X();
                V2[j + 1] = aV.Y();
                V2[j + 2] = aV.Z();
                j += 3;
            }
        }

        theTang.Initialized(0.5 * (V1 + V2));
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c> or <c>Interpolate</c>.</summary>
    public GeomAPI_PointsToBSplineSurface()
    {
        myIsDone = false;
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public GeomAPI_PointsToBSplineSurface(NCollection_Array2<gp_Pnt> Points, int DegMin = 3, int DegMax = 8,
                                          GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>.</summary>
    public GeomAPI_PointsToBSplineSurface(NCollection_Array2<gp_Pnt> Points, Approx_ParametrizationType ParType,
                                          int DegMin = 3, int DegMax = 8, GeomAbs_Shape Continuity = GeomAbs_C2,
                                          double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, ParType, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising Weight1 x length + Weight2 x
    /// curvature + Weight3 x torsion.
    /// </summary>
    public GeomAPI_PointsToBSplineSurface(NCollection_Array2<gp_Pnt> Points, double Weight1, double Weight2,
                                          double Weight3, int DegMax = 8, GeomAbs_Shape Continuity = GeomAbs_C2,
                                          double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, Weight1, Weight2, Weight3, DegMax, Continuity, Tol3D);
    }

    /// <summary>
    /// Approximates the heights <paramref name="ZPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY.
    /// </summary>
    public GeomAPI_PointsToBSplineSurface(NCollection_Array2<double> ZPoints, double X0, double dX, double Y0,
                                          double dY, int DegMin = 3, int DegMax = 8,
                                          GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(ZPoints, X0, dX, Y0, dY, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>Interpolates <paramref name="Points"/> with chord-length parameters, periodic in U if asked.</summary>
    public void Interpolate(NCollection_Array2<gp_Pnt> Points, bool thePeriodic = false)
    {
        Interpolate(Points, Approx_ChordLength, thePeriodic);
    }

    /// <summary>Interpolates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>, periodic in U if asked.</summary>
    public void Interpolate(NCollection_Array2<gp_Pnt> Points, Approx_ParametrizationType ParType,
                            bool thePeriodic = false)
    {
        int DegMin, DegMax;
        DegMin = DegMax = 3;
        GeomAbs_Shape CC = GeomAbs_C2;
        double Tol3d = -1.0;
        Init(Points, ParType, DegMin, DegMax, CC, Tol3d, thePeriodic);
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array2<gp_Pnt> Points, int DegMin = 3, int DegMax = 8,
                     GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3)
    {
        Init(Points, Approx_ChordLength, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>, or interpolates
    /// them if <paramref name="Tol3D"/> ≤ 0; periodic in U if asked.
    /// </summary>
    public void Init(NCollection_Array2<gp_Pnt> Points, Approx_ParametrizationType ParType, int DegMin = 3,
                     int DegMax = 8, GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3,
                     bool thePeriodic = false)
    {
        myIsDone = false;
        int Imin = Points.LowerRow();
        int Imax = Points.UpperRow();
        int Jmin = Points.LowerCol();
        int Jmax = Points.UpperCol();

        double Tol2D = Tol3D;

        // first approximate the U isos:
        int add = 1;
        if (thePeriodic)
        {
            add = 2;
        }
        AppDef_MultiLine Line = new(Jmax - Jmin + 1);
        int i, j;

        for (j = Jmin; j <= Jmax; j++)
        {
            AppDef_MultiPointConstraint MP = new(Imax - Imin + add, 0);
            for (i = Imin; i <= Imax; i++)
            {
                MP.SetPoint(i, Points[i, j]); // OCCT: the grid's own indices (occt-issues.md)
            }
            if (thePeriodic)
            {
                MP.SetPoint(Imax + 1, Points[1, j]);
            }
            Line.SetValue(j, MP);
        }

        int nbit = 2;
        bool UseSquares = false;
        if (Tol3D <= 1.0e-3)
        {
            UseSquares = true;
        }

        AppDef_BSplineCompute TheComputer = new(DegMin, DegMax, Tol3D, Tol2D, nbit, true, ParType, UseSquares);

        SetContinuity(TheComputer, Continuity);

        if (Tol3D <= 0.0)
        {
            TheComputer.Interpol(Line);
        }
        else
        {
            TheComputer.Perform(Line);
        }

        AppParCurves_MultiBSpCurve TheCurve = TheComputer.Value();

        int VDegree = TheCurve.Degree();
        NCollection_Array1<gp_Pnt> Poles = new(1, TheCurve.NbPoles());
        NCollection_Array1<double> VKnots = TheCurve.Knots();
        NCollection_Array1<int> VMults = TheCurve.Multiplicities();

        int nbisosu = Imax - Imin + add;
        AppDef_MultiLine Line2 = new(nbisosu);

        for (i = 1; i <= nbisosu; i++)
        {
            TheCurve.Curve(i, Poles);
            AppDef_MultiPointConstraint MP = new(Poles.Upper(), 0);
            for (j = 1; j <= Poles.Upper(); j++)
            {
                MP.SetPoint(j, Poles[j]);
            }
            Line2.SetValue(i, MP);
        }

        // approximate the resulting poles:

        AppDef_BSplineCompute TheComputer2 = new(DegMin, DegMax, Tol3D, Tol2D, nbit, true, ParType, UseSquares);
        if (Tol3D <= 0.0)
        {
            if (thePeriodic)
            {
                TheComputer2.SetPeriodic(thePeriodic);
            }
            TheComputer2.Interpol(Line2);
        }
        else
        {
            if (thePeriodic && Line2.NbMultiPoints() > 2)
            {
                TheComputer2.SetPeriodic(thePeriodic);
                //
                NCollection_Array1<double> aPars = new(1, Line2.NbMultiPoints());
                BuildParameters(Line2, ParType, aPars);
                math_Vector aTang = new(1, 3 * Poles.Upper());
                BuildPeriodicTangent(Line2, aPars, aTang);
                int ind = 1;
                TheCurve.Curve(ind, Poles);
                AppDef_MultiPointConstraint MP1 = new(Poles.Upper(), 0);
                for (j = 1; j <= Poles.Upper(); j++)
                {
                    MP1.SetPoint(j, Poles[j]);
                    int k = 3 * (j - 1);
                    gp_Vec aT = new(aTang[k + 1], aTang[k + 2], aTang[k + 3]);
                    MP1.SetTang(j, aT);
                }
                Line2.SetValue(ind, MP1);
                //
                ind = Line2.NbMultiPoints();
                TheCurve.Curve(ind, Poles);
                AppDef_MultiPointConstraint MP2 = new(Poles.Upper(), 0);
                for (j = 1; j <= Poles.Upper(); j++)
                {
                    MP2.SetPoint(j, Poles[j]);
                    int k = 3 * (j - 1);
                    gp_Vec aT = new(aTang[k + 1], aTang[k + 2], aTang[k + 3]);
                    MP2.SetTang(j, aT);
                }
                Line2.SetValue(ind, MP2);
            }
            TheComputer2.Perform(Line2);
        }

        AppParCurves_MultiBSpCurve TheCurve2 = TheComputer2.Value();

        int UDegree = TheCurve2.Degree();
        NCollection_Array1<gp_Pnt> Poles2 = new(1, TheCurve2.NbPoles());
        NCollection_Array1<double> UKnots = TheCurve2.Knots();
        NCollection_Array1<int> UMults = TheCurve2.Multiplicities();

        // computing the surface
        NCollection_Array2<gp_Pnt> ThePoles = new(1, Poles2.Upper(), 1, Poles.Upper());
        for (j = 1; j <= Poles.Upper(); j++)
        {
            TheCurve2.Curve(j, Poles2);
            for (i = 1; i <= Poles2.Upper(); i++)
            {
                ThePoles[i, j] = Poles2[i];
            }
        }

        mySurface = new Geom_BSplineSurface(ThePoles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);
        if (thePeriodic && Line2.NbMultiPoints() > 2)
        {
            mySurface.SetUPeriodic();
        }

        myIsDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising Weight1 x length + Weight2 x
    /// curvature + Weight3 x torsion.
    /// </summary>
    public void Init(NCollection_Array2<gp_Pnt> Points, double Weight1, double Weight2, double Weight3,
                     int DegMax = 8, GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        int Imin = Points.LowerRow();
        int Imax = Points.UpperRow();
        int Jmin = Points.LowerCol();
        int Jmax = Points.UpperCol();

        int nbit = 2;
        if (Tol3D <= 1.0e-3)
        {
            nbit = 0;
        }

        // first approximate the U isos:
        int NbPointJ = Jmax - Jmin + 1;
        int NbPointI = Imax - Imin + 1;
        int i, j;

        AppDef_MultiLine Line = new(NbPointJ);

        for (j = Jmin; j <= Jmax; j++)
        {
            AppDef_MultiPointConstraint MP = new(NbPointI, 0);
            for (i = Imin; i <= Imax; i++)
            {
                MP.SetPoint(i, Points[i, j]); // OCCT: the grid's own indices (occt-issues.md)
            }
            Line.SetValue(j, MP);
        }

        NCollection_HArray1<AppParCurves_ConstraintCouple> TABofCC = new(1, NbPointJ);
        AppParCurves_Constraint Constraint = AppParCurves_NoConstraint;

        for (i = 1; i <= NbPointJ; ++i)
        {
            AppParCurves_ConstraintCouple ACC = new(i, Constraint);
            TABofCC.SetValue(i, ACC);
        }

        AppDef_Variational Variation = new(Line, 1, NbPointJ, TABofCC);

        //===================================
        int theMaxSegments = 1000;
        bool theWithMinMax = false;
        //===================================

        Variation.SetMaxDegree(DegMax);
        Variation.SetContinuity(Continuity);
        Variation.SetMaxSegment(theMaxSegments);

        Variation.SetTolerance(Tol3D);
        Variation.SetWithMinMax(theWithMinMax);
        Variation.SetNbIterations(nbit);

        Variation.SetCriteriumWeight(Weight1, Weight2, Weight3);

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

        AppParCurves_MultiBSpCurve TheCurve = Variation.Value();

        int VDegree = TheCurve.Degree();
        NCollection_Array1<gp_Pnt> Poles = new(1, TheCurve.NbPoles());
        NCollection_Array1<double> VKnots = TheCurve.Knots();
        NCollection_Array1<int> VMults = TheCurve.Multiplicities();

        AppDef_MultiLine Line2 = new(NbPointI);

        for (i = 1; i <= NbPointI; i++)
        {
            TheCurve.Curve(i, Poles);
            AppDef_MultiPointConstraint MP = new(Poles.Upper(), 0);
            for (j = 1; j <= Poles.Upper(); j++)
            {
                MP.SetPoint(j, Poles[j]);
            }
            Line2.SetValue(i, MP);
        }

        NCollection_HArray1<AppParCurves_ConstraintCouple> TABofCC2 = new(1, NbPointI);

        for (i = 1; i <= NbPointI; ++i)
        {
            AppParCurves_ConstraintCouple ACC = new(i, Constraint);
            TABofCC2.SetValue(i, ACC);
        }

        AppDef_Variational Variation2 = new(Line2, 1, NbPointI, TABofCC2);

        Variation2.SetMaxDegree(DegMax);
        Variation2.SetContinuity(Continuity);
        Variation2.SetMaxSegment(theMaxSegments);

        Variation2.SetTolerance(Tol3D);
        Variation2.SetWithMinMax(theWithMinMax);
        Variation2.SetNbIterations(nbit);

        Variation2.SetCriteriumWeight(Weight1, Weight2, Weight3);

        if (!Variation2.IsCreated())
        {
            return;
        }

        if (Variation2.IsOverConstrained())
        {
            return;
        }

        try
        {
            Variation2.Approximate();
        }
        catch (Standard_Failure)
        {
            return;
        }

        if (!Variation2.IsDone())
        {
            return;
        }

        AppParCurves_MultiBSpCurve TheCurve2 = Variation2.Value();

        int UDegree = TheCurve2.Degree();
        NCollection_Array1<gp_Pnt> Poles2 = new(1, TheCurve2.NbPoles());
        NCollection_Array1<double> UKnots = TheCurve2.Knots();
        NCollection_Array1<int> UMults = TheCurve2.Multiplicities();

        // computing the surface
        NCollection_Array2<gp_Pnt> ThePoles = new(1, Poles2.Upper(), 1, Poles.Upper());
        for (j = 1; j <= Poles.Upper(); j++)
        {
            TheCurve2.Curve(j, Poles2);
            for (i = 1; i <= Poles2.Upper(); i++)
            {
                ThePoles[i, j] = Poles2[i];
            }
        }

        mySurface = new Geom_BSplineSurface(ThePoles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);

        myIsDone = true;
    }

    /// <summary>
    /// Interpolates the heights <paramref name="ZPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY.
    /// </summary>
    public void Interpolate(NCollection_Array2<double> ZPoints, double X0, double dX, double Y0, double dY)
    {
        int DegMin, DegMax;
        DegMin = DegMax = 3;
        double Tol3D = -1.0;
        GeomAbs_Shape CC = GeomAbs_C2;
        Init(ZPoints, X0, dX, Y0, dY, DegMin, DegMax, CC, Tol3D);
    }

    /// <summary>
    /// Approximates the heights <paramref name="ZPoints"/> over the grid X0 + (i − 1) dX, Y0 + (j − 1) dY, or
    /// interpolates them if <paramref name="Tol3D"/> ≤ 0.
    /// </summary>
    public void Init(NCollection_Array2<double> ZPoints, double X0, double dX, double Y0, double dY,
                     int DegMin = 3, int DegMax = 8, GeomAbs_Shape Continuity = GeomAbs_C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        int Imin = ZPoints.LowerRow();
        int Imax = ZPoints.UpperRow();
        int Jmin = ZPoints.LowerCol();
        int Jmax = ZPoints.UpperCol();
        double length;

        double Tol2D = Tol3D;

        // first approximate the U isos:
        AppDef_MultiLine Line = new(Jmax - Jmin + 1);
        math_Vector Param = new(Jmin, Jmax);
        int i, j;
        int aVSpan = Jmax - Jmin;
        //  double X, Y;
        length = dY * (Jmax - Jmin);

        for (j = Jmin; j <= Jmax; j++)
        {
            AppDef_MultiPointConstraint MP = new(0, Imax - Imin + 1);
            for (i = Imin; i <= Imax; i++)
            {
                MP.SetPoint2d(i, new gp_Pnt2d(0.0, ZPoints[i, j])); // OCCT: the grid's own indices (occt-issues.md)
            }
            Param[j] = (aVSpan > 0) ? (double)(j - Jmin) / (double)(aVSpan) : 0.0;
            Line.SetValue(j, MP);
        }

        AppDef_BSplineCompute TheComputer = new(Param, DegMin, DegMax, Tol3D, Tol2D, 0, true, true);

        SetContinuity(TheComputer, Continuity);

        if (Tol3D <= 0.0)
        {
            TheComputer.Interpol(Line);
        }
        else
        {
            TheComputer.Perform(Line);
        }

        AppParCurves_MultiBSpCurve TheCurve = TheComputer.Value();

        int VDegree = TheCurve.Degree();
        NCollection_Array1<gp_Pnt2d> Poles = new(1, TheCurve.NbPoles());
        int nk = TheCurve.Knots().Length();
        NCollection_Array1<double> VKnots = new(1, nk);
        NCollection_Array1<int> VMults = new(1, nk);

        // compute Y values for the poles
        NCollection_Array1<double> YPoles = new(1, Poles.Upper());

        // start with a line
        NCollection_Array1<double> TempPoles = new(1, 2);
        NCollection_Array1<double> TempKnots = new(1, 2);
        NCollection_Array1<int> TempMults = new(1, 2);
        TempMults.Init(2);
        TempPoles[1] = Y0;
        TempPoles[2] = Y0 + length;
        TempKnots[1] = 0.0;
        TempKnots[2] = 1.0;

        // increase the Degree
        NCollection_Array1<double> NewTempPoles = new(1, VDegree + 1);
        NCollection_Array1<double> NewTempKnots = new(1, 2);
        NCollection_Array1<int> NewTempMults = new(1, 2);
        BSplCLib.IncreaseDegree(1,
                                VDegree,
                                false,
                                1,
                                TempPoles,
                                TempKnots,
                                TempMults,
                                NewTempPoles,
                                NewTempKnots,
                                NewTempMults);

        // insert the Knots
        BSplCLib.InsertKnots(VDegree,
                             false,
                             1,
                             NewTempPoles,
                             NewTempKnots,
                             NewTempMults,
                             TheCurve.Knots(),
                             TheCurve.Multiplicities(),
                             YPoles,
                             VKnots,
                             VMults,
                             Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk; i++)
        {
            VKnots[i] = Y0 + length * VKnots[i];
        }

        int nbisosu = Imax - Imin + 1;
        AppDef_MultiLine Line2 = new(nbisosu);
        math_Vector Param2 = new(1, nbisosu);
        int aUSpan = nbisosu - 1;
        length = dX * (Imax - Imin);

        for (i = 1; i <= nbisosu; i++)
        {
            TheCurve.Curve(i, Poles);
            AppDef_MultiPointConstraint MP = new(0, Poles.Upper());
            for (j = 1; j <= Poles.Upper(); j++)
            {
                MP.SetPoint2d(j, new gp_Pnt2d(0.0, Poles[j].Y()));
            }
            Param2[i] = (aUSpan > 0) ? (double)(i - 1) / (double)(aUSpan) : 0.0;
            Line2.SetValue(i, MP);
        }

        // approximate the resulting poles:

        AppDef_BSplineCompute TheComputer2 = new(Param2, DegMin, DegMax, Tol3D, Tol2D, 0, true, true);
        if (Tol3D <= 0.0)
        {
            TheComputer2.Interpol(Line2);
        }
        else
        {
            TheComputer2.Perform(Line2);
        }

        AppParCurves_MultiBSpCurve TheCurve2 = TheComputer2.Value();

        int UDegree = TheCurve2.Degree();
        NCollection_Array1<gp_Pnt2d> Poles2 = new(1, TheCurve2.NbPoles());
        int nk2 = TheCurve2.Knots().Length();
        NCollection_Array1<double> UKnots = new(1, nk2);
        NCollection_Array1<int> UMults = new(1, nk2);

        // compute X values for the poles
        NCollection_Array1<double> XPoles = new(1, Poles2.Upper());

        // start with a line
        TempPoles[1] = X0;
        TempPoles[2] = X0 + length;
        TempKnots[1] = 0.0;
        TempKnots[2] = 1.0;
        TempMults[1] = TempMults[2] = 2;

        // increase the Degree
        NCollection_Array1<double> NewTempPoles2 = new(1, UDegree + 1);
        BSplCLib.IncreaseDegree(1,
                                UDegree,
                                false,
                                1,
                                TempPoles,
                                TempKnots,
                                TempMults,
                                NewTempPoles2,
                                NewTempKnots,
                                NewTempMults);

        // insert the Knots
        BSplCLib.InsertKnots(UDegree,
                             false,
                             1,
                             NewTempPoles2,
                             NewTempKnots,
                             NewTempMults,
                             TheCurve2.Knots(),
                             TheCurve2.Multiplicities(),
                             XPoles,
                             UKnots,
                             UMults,
                             Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk2; i++)
        {
            UKnots[i] = X0 + length * UKnots[i];
        }

        // creating the surface
        NCollection_Array2<gp_Pnt> ThePoles = new(1, Poles2.Upper(), 1, Poles.Upper());

        for (j = 1; j <= Poles.Upper(); j++)
        {
            TheCurve2.Curve(j, Poles2);
            for (i = 1; i <= Poles2.Upper(); i++)
            {
                ThePoles[i, j].SetCoord(XPoles[i], YPoles[j], Poles2[i].Y());
            }
        }

        mySurface = new Geom_BSplineSurface(ThePoles, UKnots, VKnots, UMults, VMults, UDegree, VDegree);

        myIsDone = true;
    }

    // The switch on the continuity that OCCT writes out in both least-squares Init.
    private static void SetContinuity(AppDef_BSplineCompute TheComputer, GeomAbs_Shape Continuity)
    {
        switch (Continuity)
        {
            case GeomAbs_C0:
                TheComputer.SetContinuity(0);
                break;

            case GeomAbs_G1:
            case GeomAbs_C1:
                TheComputer.SetContinuity(1);
                break;

            case GeomAbs_G2:
            case GeomAbs_C2:
                TheComputer.SetContinuity(2);
                break;

            default:
                TheComputer.SetContinuity(3);
                break;
        }
    }

    /// <summary>The approximating surface (frozen).</summary>
    /// <exception cref="StdFail_NotDone">The approximation failed.</exception>
    public Geom_BSplineSurface Surface()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_PointsToBSplineSurface: Surface not done");
        mySurface!.Freeze();
        return mySurface;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineSurface&gt;</c>: <see cref="Surface"/>.</summary>
    public static explicit operator Geom_BSplineSurface(GeomAPI_PointsToBSplineSurface theApprox) =>
        theApprox.Surface();

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone() => myIsDone;
}
