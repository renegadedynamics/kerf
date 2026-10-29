// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_LeastSquare.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheLeastSquares.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 27/07/91
// pmn, 30/10/98 : Protection for overconstrained cases -> "isready"

// Approximation of a MultiLine of points described by the tool MLineTool.
// This program uses the least squares method for the following case:
// passing points and tangencies at extremities.

using static Occt.AppParCurves_Constraint;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// Least-squares approximation of a MultiLine by Bezier curves (Householder-QR or a banded Cholesky
/// resolution) or, given knots and multiplicities, by B-spline curves. The first and last points may be
/// passing, tangency or curvature points; the tangency and curvature values are computed in the resolution.
/// </summary>
/// <remarks>
/// <para>
/// The generic form of OCCT's <c>AppParCurves_LeastSquare.gxx</c> (decision 142), which OCCT instantiates as
/// AppDef_TheLeastSquares, AppDef_ParLeastSquareOf..., AppDef_BSpParLeastSquareOf..., and the GeomInt and
/// BRepApprox equivalents. The public members are those of the instantiations' headers.
/// </para>
/// <para>
/// A working object (decision 21). The system solved is A X = B, where A is the Bernstein (or B-spline) matrix
/// at the parameters, B the point coordinates and X the poles; A is the same for every coordinate of every curve.
/// The matrices returned (<see cref="FunctionMatrix"/>, <see cref="Poles"/>, <see cref="Distance"/>, ...) and the
/// MultiBSpCurve returned by <see cref="BSplineValue"/> are the object's own (decision 33; shared: don't modify).
/// </para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_LeastSquare<MultiLine, ToolLine>
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    private AppParCurves_Constraint FirstConstraint;
    private AppParCurves_Constraint LastConstraint;
    private readonly AppParCurves_MultiBSpCurve SCU;
    private NCollection_HArray1<double>? myknots;
    private NCollection_HArray1<int>? mymults;
    private readonly math_Matrix mypoles;
    private readonly math_Matrix A;
    private readonly math_Matrix DA;
    private readonly math_Matrix B2;
    private readonly math_Matrix mypoints;
    private readonly math_Vector Vflatknots;
    private readonly math_Vector Vec1t;
    private readonly math_Vector Vec1c;
    private readonly math_Vector Vec2t;
    private readonly math_Vector Vec2c;
    private readonly math_Matrix theError;
    private readonly math_IntegerVector myindex;
    private double lambda1;
    private double lambda2;
    private int FirstP;
    private int LastP;
    private int Nlignes;
    private int Ninc;
    private int NA;
    private int myfirstp;
    private int mylastp;
    private int resinit;
    private int resfin;
    private int nbP2d;
    private int nbP;
    private readonly int nbpoles;
    private int deg;
    private bool done;
    private bool iscalculated;
    private bool isready;

    private static int FlatLength(NCollection_Array1<int> Mults)
    {
        int sum = 0;
        for (int i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            sum += Mults.Value(i);
        }
        return sum;
    }

    //=======================================================================
    // function : CheckTangents
    // purpose  : Checks if theArrTg3d and theArrTg2d have direction
    //            corresponded to the direction between theArrPt1 and theArrPt2.
    //            If it is not then reverses tangent vectors.
    //              theArrPt1 (as same as theArrPt2) is sub-set of all 3D-points in
    //            one multy-point (multy-point is union of sets of 2D- and 3D-points).
    //
    // ATTENTION!!!
    //              The property of correlation between Tg3d and Tg2d is used here.
    //            Therefore, only 3D-coinciding is checked.
    //=======================================================================
    private static void CheckTangents(NCollection_Array1<gp_Pnt> theArrPt1, NCollection_Array1<gp_Pnt> theArrPt2,
                                      NCollection_Array1<gp_Vec> theArrTg3d, NCollection_Array1<gp_Vec2d> theArrTg2d)
    {
        if (theArrPt1.Lower() != theArrPt2.Lower())
        {
            return;
        }

        if (theArrPt1.Upper() != theArrPt2.Upper())
        {
            return;
        }

        if (theArrTg3d.Length() != theArrPt1.Length())
        {
            return;
        }

        bool isToChangeDir = false;

        for (int i = theArrPt1.Lower(); i <= theArrPt1.Upper(); i++)
        {
            gp_Vec aV1 = new(theArrPt1[i], theArrPt2[i]);
            gp_Vec aV2 = theArrTg3d[i];

            if (aV1.Dot(aV2) < 0.0)
            {
                isToChangeDir = true;
                break;
            }
        }

        if (!isToChangeDir)
        {
            return;
        }

        // Change directions for every 2D- and 3D-tangents

        for (int i = theArrTg3d.Lower(); i <= theArrTg3d.Upper(); i++)
        {
            theArrTg3d[i].Reverse();
        }

        for (int i = theArrTg2d.Lower(); i <= theArrTg2d.Upper(); i++)
        {
            theArrTg2d[i].Reverse();
        }
    }

    //=======================================================================
    // function : CheckTangents
    // purpose  : Checks if theArrTg2d have direction
    //            corresponded to the direction between theArrPt1 and theArrPt2.
    //            If it is not then reverses tangent vector.
    //              theArrPt1 (as same as theArrPt2) is sub-set of all 2D-points in
    //            one multy-point (multy-point is union of sets of 2D- and 3D-points).
    //=======================================================================
    private static void CheckTangents(NCollection_Array1<gp_Pnt2d> theArrPt1, NCollection_Array1<gp_Pnt2d> theArrPt2,
                                      NCollection_Array1<gp_Vec2d> theArrTg2d)
    {
        if (theArrPt1.Lower() != theArrPt2.Lower())
        {
            return;
        }

        if (theArrPt1.Upper() != theArrPt2.Upper())
        {
            return;
        }

        for (int i = theArrPt1.Lower(); i <= theArrPt1.Upper(); i++)
        {
            gp_Vec2d aV1 = new(theArrPt1[i], theArrPt2[i]);
            gp_Vec2d aV2 = theArrTg2d[i];

            if (aV1.Dot(aV2) < 0.0)
            {
                theArrTg2d[i].Reverse();
            }
        }
    }

    // The member initialisers shared by OCCT's four constructors
    private AppParCurves_LeastSquare(MultiLine SSP, int FirstPoint, int LastPoint, AppParCurves_Constraint FirstCons,
                                     AppParCurves_Constraint LastCons, int NbPol, int theFlatLength)
    {
        SCU = new AppParCurves_MultiBSpCurve(NbPol);
        mypoles = new math_Matrix(1, NbPol, 1, NbBColumns(SSP));
        A = new math_Matrix(FirstPoint, LastPoint, 1, NbPol);
        DA = new math_Matrix(FirstPoint, LastPoint, 1, NbPol);
        B2 = new math_Matrix(TheFirstPoint(FirstCons, FirstPoint),
                             Math.Max(TheFirstPoint(FirstCons, FirstPoint), TheLastPoint(LastCons, LastPoint)),
                             1,
                             NbBColumns(SSP));
        mypoints = new math_Matrix(FirstPoint, LastPoint, 1, NbBColumns(SSP));
        Vflatknots = new math_Vector(1, theFlatLength);
        Vec1t = new math_Vector(1, NbBColumns(SSP));
        Vec1c = new math_Vector(1, NbBColumns(SSP));
        Vec2t = new math_Vector(1, NbBColumns(SSP));
        Vec2c = new math_Vector(1, NbBColumns(SSP));
        theError = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        myindex = new math_IntegerVector(FirstPoint, LastPoint, 0);
        nbpoles = NbPol;
    }

    /// <summary>
    /// Approximates the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/> by Bezier curves of <paramref name="NbPol"/> poles at the parameters
    /// <paramref name="Parameters"/>, with the given end constraints.
    /// </summary>
    public AppParCurves_LeastSquare(MultiLine SSP, int FirstPoint, int LastPoint, AppParCurves_Constraint FirstCons,
                                    AppParCurves_Constraint LastCons, math_Vector Parameters, int NbPol)
        : this(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol, 1)
    {
        FirstConstraint = FirstCons;
        LastConstraint = LastCons;
        Init(SSP, FirstPoint, LastPoint);
        Perform(Parameters);
    }

    /// <summary>Initialises the fields for Bezier curves of <paramref name="NbPol"/> poles; call a Perform method next.</summary>
    public AppParCurves_LeastSquare(MultiLine SSP, int FirstPoint, int LastPoint, AppParCurves_Constraint FirstCons,
                                    AppParCurves_Constraint LastCons, int NbPol)
        : this(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol, 1)
    {
        FirstConstraint = FirstCons;
        LastConstraint = LastCons;
        Init(SSP, FirstPoint, LastPoint);
    }

    /// <summary>
    /// Approximates the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/> by B-spline curves with the knots <paramref name="Knots"/> and multiplicities
    /// <paramref name="Mults"/> at the parameters <paramref name="Parameters"/>, with the given end constraints.
    /// </summary>
    public AppParCurves_LeastSquare(MultiLine SSP, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                    int FirstPoint, int LastPoint, AppParCurves_Constraint FirstCons,
                                    AppParCurves_Constraint LastCons, math_Vector Parameters, int NbPol)
        : this(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol, FlatLength(Mults))
    {
        FirstConstraint = FirstCons;
        LastConstraint = LastCons;
        myknots = new NCollection_HArray1<double>(Knots);
        mymults = new NCollection_HArray1<int>(Mults);
        SCU.SetKnots(Knots);
        SCU.SetMultiplicities(Mults);
        Init(SSP, FirstPoint, LastPoint);
        Perform(Parameters);
    }

    /// <summary>Initialises the fields for B-spline curves; call a Perform method next.</summary>
    public AppParCurves_LeastSquare(MultiLine SSP, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                    int FirstPoint, int LastPoint, AppParCurves_Constraint FirstCons,
                                    AppParCurves_Constraint LastCons, int NbPol)
        : this(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol, FlatLength(Mults))
    {
        myknots = new NCollection_HArray1<double>(Knots);
        mymults = new NCollection_HArray1<int>(Mults);
        SCU.SetKnots(Knots);
        SCU.SetMultiplicities(Mults);
        FirstConstraint = FirstCons;
        LastConstraint = LastCons;
        Init(SSP, FirstPoint, LastPoint);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Init(MultiLine SSP, int FirstPoint, int LastPoint)
    {
        // Variable de controle
        iscalculated = false;
        isready = true;

        myfirstp = FirstPoint;
        mylastp = LastPoint;
        FirstP = TheFirstPoint(FirstConstraint, myfirstp);
        LastP = TheLastPoint(LastConstraint, mylastp);

        // Identification of constraints at extremities:
        // ========================================
        int i, j, k, i2;

        nbP2d = ToolLine.NbP2d(SSP);
        nbP = ToolLine.NbP3d(SSP);
        gp_Pnt Poi = new();
        gp_Pnt2d Poi2d = new();
        int mynbP2d = nbP2d, mynbP = nbP;
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        if (nbP == 0)
        {
            mynbP = 1;
        }
        NCollection_Array1<gp_Pnt> TabP = new(1, mynbP);
        NCollection_Array1<gp_Pnt2d> TabP2d = new(1, mynbP2d);

        deg = nbpoles - 1;

        if (mymults is not null)
        {
            int sum = 0;
            for (i = mymults.Lower(); i <= mymults.Upper(); i++)
            {
                sum += mymults.Value(i);
            }
            deg = sum - nbpoles - 1;
            k = 1;
            double val;
            for (i = myknots!.Lower(); i <= myknots.Upper(); i++)
            {
                for (j = 1; j <= mymults.Value(i); j++)
                {
                    val = myknots.Value(i);
                    Vflatknots[k] = val;
                    k++;
                }
            }
        }

        Affect(SSP, FirstPoint, ref FirstConstraint, Vec1t, Vec1c);

        Affect(SSP, LastPoint, ref LastConstraint, Vec2t, Vec2c);

        for (j = myfirstp; j <= mylastp; j++)
        {
            i2 = 1;
            if (nbP != 0 && nbP2d != 0)
            {
                ToolLine.Value(SSP, j, TabP, TabP2d);
            }
            else if (nbP2d != 0)
            {
                ToolLine.Value(SSP, j, TabP2d);
            }
            else
            {
                ToolLine.Value(SSP, j, TabP);
            }
            for (i = 1; i <= nbP; i++)
            {
                TabP[i].Coord(out mypoints[j, i2], out mypoints[j, i2 + 1], out mypoints[j, i2 + 2]);
                i2 += 3;
            }
            for (i = 1; i <= nbP2d; i++)
            {
                TabP2d[i].Coord(out mypoints[j, i2], out mypoints[j, i2 + 1]);
                i2 += 2;
            }
        }

        AppParCurves_MultiPoint Pole1 = new(nbP, nbP2d), PoleN = new(nbP, nbP2d);

        if (FirstConstraint == AppParCurves_PassPoint || FirstConstraint == AppParCurves_TangencyPoint
            || FirstConstraint == AppParCurves_CurvaturePoint)
        {
            i2 = 1;
            for (i = 1; i <= nbP; i++)
            {
                Poi.SetCoord(mypoints[myfirstp, i2], mypoints[myfirstp, i2 + 1], mypoints[myfirstp, i2 + 2]);
                Pole1.SetPoint(i, Poi);
                i2 += 3;
            }
            for (i = 1; i <= nbP2d; i++)
            {
                Poi2d.SetCoord(mypoints[myfirstp, i2], mypoints[myfirstp, i2 + 1]);
                Pole1.SetPoint2d(i + nbP, Poi2d);
                i2 += 2;
            }
            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[1, i] = mypoints[myfirstp, i];
            }
        }

        // OCCT: the last term tests FirstConstraint where LastConstraint is meant, so a curvature constraint at the
        // first point also copies the last point into the last pole, even when the last point is unconstrained
        if (LastConstraint == AppParCurves_PassPoint || LastConstraint == AppParCurves_TangencyPoint
            || FirstConstraint == AppParCurves_CurvaturePoint)
        {
            i2 = 1;
            for (i = 1; i <= nbP; i++)
            {
                Poi.SetCoord(mypoints[mylastp, i2], mypoints[mylastp, i2 + 1], mypoints[mylastp, i2 + 2]);
                PoleN.SetPoint(i, Poi);
                i2 += 3;
            }
            for (i = 1; i <= nbP2d; i++)
            {
                Poi2d.SetCoord(mypoints[mylastp, i2], mypoints[mylastp, i2 + 1]);
                PoleN.SetPoint2d(i + nbP, Poi2d);
                i2 += 2;
            }

            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[nbpoles, i] = mypoints[mylastp, i];
            }
        }

        if (FirstConstraint == AppParCurves_NoConstraint)
        {
            resinit = 1;
            SCU.SetValue(1, Pole1);
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                resfin = nbpoles;
            }
            else if (LastConstraint == AppParCurves_PassPoint)
            {
                resfin = nbpoles - 1;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_TangencyPoint)
            {
                resfin = nbpoles - 2;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                resfin = nbpoles - 3;
                SCU.SetValue(nbpoles, PoleN);
            }
        }
        else if (FirstConstraint == AppParCurves_PassPoint)
        {
            resinit = 2;
            SCU.SetValue(1, Pole1);
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                resfin = nbpoles;
            }
            else if (LastConstraint == AppParCurves_PassPoint)
            {
                resfin = nbpoles - 1;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_TangencyPoint)
            {
                resfin = nbpoles - 2;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                resfin = nbpoles - 3;
                SCU.SetValue(nbpoles, PoleN);
            }
        }
        else if (FirstConstraint == AppParCurves_TangencyPoint)
        {
            resinit = 3;
            SCU.SetValue(1, Pole1);
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                resfin = nbpoles;
            }
            if (LastConstraint == AppParCurves_PassPoint)
            {
                resfin = nbpoles - 1;
                SCU.SetValue(nbpoles, PoleN);
            }
            if (LastConstraint == AppParCurves_TangencyPoint)
            {
                resfin = nbpoles - 2;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                resfin = nbpoles - 3;
                SCU.SetValue(nbpoles, PoleN);
            }
        }
        else if (FirstConstraint == AppParCurves_CurvaturePoint)
        {
            resinit = 4;
            SCU.SetValue(1, Pole1);
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                resfin = nbpoles;
            }
            if (LastConstraint == AppParCurves_PassPoint)
            {
                resfin = nbpoles - 1;
                SCU.SetValue(nbpoles, PoleN);
            }
            if (LastConstraint == AppParCurves_TangencyPoint)
            {
                resfin = nbpoles - 2;
                SCU.SetValue(nbpoles, PoleN);
            }
            else if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                resfin = nbpoles - 3;
                SCU.SetValue(nbpoles, PoleN);
            }
        }

        int Nincx = resfin - resinit + 1;
        if (Nincx < 1)
        { // Impossible d'aller plus loin
            isready = false;
            return;
        }
        int Neq = LastP - FirstP + 1;

        NA = 3 * nbP + 2 * nbP2d;
        Nlignes = NA * Neq;
        Ninc = NA * Nincx;
        if (FirstConstraint >= AppParCurves_TangencyPoint)
        {
            Ninc++;
        }
        if (LastConstraint >= AppParCurves_TangencyPoint)
        {
            Ninc++;
        }
    }

    /// <summary>
    /// Computes the approximation at <paramref name="Parameters"/>, after the fields have been initialised.
    /// The curvature case is not treated here.
    /// </summary>
    public void Perform(math_Vector Parameters)
    {
        done = false;
        if (!isready)
        {
            return;
        }
        int i, j, k, Ci, i2, k1, k2;
        int nbpol1 = nbpoles - 1, Ninc1 = Ninc - 1;
        double AD1, A0;
        iscalculated = false;

        // Calculation of matrix A and DA of approximation functions:
        ComputeFunction(Parameters);

        if (FirstConstraint != AppParCurves_TangencyPoint && LastConstraint != AppParCurves_TangencyPoint)
        {
            if (FirstConstraint == AppParCurves_NoConstraint)
            {
                if (LastConstraint == AppParCurves_NoConstraint)
                {
                    math_Householder HouResol = new(A, mypoints);
                    if (!(HouResol.IsDone()))
                    {
                        done = false;
                        return;
                    }
                    done = true;
                    mypoles.Initialized(HouResol.AllValues());
                    return;
                }
                else
                {
                    for (j = FirstP; j <= LastP; j++)
                    {
                        AD1 = A[j, nbpoles];
                        for (i = 1; i <= B2.ColNumber(); i++)
                        {
                            B2[j, i] = mypoints[j, i] - AD1 * mypoles[nbpoles, i];
                        }
                    }
                }
            }
            else if (FirstConstraint == AppParCurves_PassPoint)
            {
                if (LastConstraint == AppParCurves_NoConstraint)
                {
                    for (j = FirstP; j <= LastP; j++)
                    {
                        A0 = A[j, 1];
                        for (i = 1; i <= B2.ColNumber(); i++)
                        {
                            B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i];
                        }
                    }
                }
                else if (LastConstraint == AppParCurves_PassPoint)
                {
                    for (j = FirstP; j <= LastP; j++)
                    {
                        A0 = A[j, 1];
                        AD1 = A[j, nbpoles];
                        for (i = 1; i <= B2.ColNumber(); i++)
                        {
                            B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - AD1 * mypoles[nbpoles, i];
                        }
                    }
                }
            }

            // resolution:

            // (OCCT declares this block's Nincx and Index again below, in the enclosing scope)
            int NincxB = resfin - resinit + 1;
            if (NincxB < 1)
            {
                done = true;
                return;
            }
            math_IntegerVector IndexB = new(1, NincxB);
            SearchIndex(IndexB);
            math_Matrix mytab = new(resinit, resfin, 1, B2.ColNumber(), 0.0);
            math_Vector TheAA = new(1, IndexB[NincxB], 0.0);
            math_Vector myTABB = new(1, NincxB, 0.0);

            MakeTAA(TheAA, mytab);
            DACTCL_Decompose(TheAA, IndexB);

            int kk2;
            for (j = 1; j <= B2.ColNumber(); j++)
            {
                kk2 = 1;
                for (i = resinit; i <= resfin; i++)
                {
                    myTABB[kk2] = mytab[i, j];
                    kk2++;
                }
                DACTCL_Solve(TheAA, myTABB, IndexB);

                i2 = 1;
                for (k = resinit; k <= resfin; k++)
                {
                    mypoles[k, j] = myTABB.Value(i2);
                    i2++;
                }
            }
            done = true;
            // No return here in OCCT: the passing-point cases go on to solve the system below as well, which has
            // the same solution and overwrites the poles just computed with it
        }

        // ===========================================================
        // cas de tangence:
        // ===========================================================

        int Nincx = resfin - resinit + 1;
        int deport = 0, Nincx2 = 2 * Nincx;

        math_IntegerVector InternalIndex = new(1, Nincx);
        SearchIndex(InternalIndex);
        math_IntegerVector Index = new(1, Ninc);

        int l = 1;
        if (resinit <= resfin)
        {
            for (j = 0; j <= NA - 1; j++)
            {
                deport = j * InternalIndex[Nincx];
                for (i = 1; i <= Nincx; i++)
                {
                    Index[l] = InternalIndex[i] + deport;
                    l++;
                }
            }
        }

        if (resinit > resfin)
        {
            Index[1] = 1;
        }
        if (Ninc1 > 1)
        {
            if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
            {
                Index[Ninc1] = Index[Ninc1 - 1] + Ninc1;
            }
        }
        if (FirstConstraint >= AppParCurves_TangencyPoint || LastConstraint >= AppParCurves_TangencyPoint)
        {
            Index[Ninc] = Index[Ninc - 1] + Ninc;
        }

        math_Vector TheA = new(1, Index[Ninc], 0.0);
        math_Vector myTAB = new(1, Ninc, 0.0);

        MakeTAA(TheA, myTAB);

        int Error = DACTCL_Decompose(TheA, Index);
        Error = DACTCL_Solve(TheA, myTAB, Index);

        if (Error == 0)
        {
            done = true;
        }

        if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
        {
            lambda1 = myTAB.Value(Ninc1);
            lambda2 = myTAB.Value(Ninc);
        }
        else if (FirstConstraint >= AppParCurves_TangencyPoint)
        {
            lambda1 = myTAB.Value(Ninc);
        }
        else if (LastConstraint >= AppParCurves_TangencyPoint)
        {
            lambda2 = myTAB.Value(Ninc);
        }

        // The results are stored in mypoles.
        //=========================================
        k = 1;
        i2 = 1;
        for (Ci = 1; Ci <= nbP; Ci++)
        {
            k1 = k + 1;
            k2 = k + 2;
            for (j = resinit; j <= resfin; j++)
            {
                mypoles[j, k] = myTAB.Value(i2);
                mypoles[j, k1] = myTAB.Value(i2 + Nincx);
                mypoles[j, k2] = myTAB.Value(i2 + Nincx2);
                i2++;
            }

            if (FirstConstraint >= AppParCurves_TangencyPoint)
            {
                mypoles[2, k] = mypoints[myfirstp, k] + lambda1 * Vec1t[k];
                mypoles[2, k1] = mypoints[myfirstp, k1] + lambda1 * Vec1t[k1];
                mypoles[2, k2] = mypoints[myfirstp, k2] + lambda1 * Vec1t[k2];
            }
            if (LastConstraint >= AppParCurves_TangencyPoint)
            {
                mypoles[nbpol1, k] = mypoints[mylastp, k] - lambda2 * Vec2t[k];
                mypoles[nbpol1, k1] = mypoints[mylastp, k1] - lambda2 * Vec2t[k1];
                mypoles[nbpol1, k2] = mypoints[mylastp, k2] - lambda2 * Vec2t[k2];
            }
            k += 3;
            i2 += Nincx2;
        }

        for (Ci = 1; Ci <= nbP2d; Ci++)
        {
            k1 = k + 1;
            k2 = k + 2;
            for (j = resinit; j <= resfin; j++)
            {
                mypoles[j, k] = myTAB.Value(i2);
                mypoles[j, k1] = myTAB.Value(i2 + Nincx);
                i2++;
            }
            if (FirstConstraint >= AppParCurves_TangencyPoint)
            {
                mypoles[2, k] = mypoints[myfirstp, k] + lambda1 * Vec1t[k];
                mypoles[2, k1] = mypoints[myfirstp, k1] + lambda1 * Vec1t[k1];
            }
            if (LastConstraint >= AppParCurves_TangencyPoint)
            {
                mypoles[nbpol1, k] = mypoints[mylastp, k] - lambda2 * Vec2t[k];
                mypoles[nbpol1, k1] = mypoints[mylastp, k1] - lambda2 * Vec2t[k1];
            }
            k += 2;
            i2 += Nincx;
        }
    }

    /// <summary>
    /// Computes the approximation with the tangent vectors <paramref name="V1t"/> and <paramref name="V2t"/> at the
    /// first and last points, after the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector Parameters, math_Vector V1t, math_Vector V2t, double l1, double l2)
    {
        done = false;
        if (!isready)
        {
            return;
        }
        int i, lower1 = V1t.Lower(), lower2 = V2t.Lower();
        resinit = 3;
        resfin = nbpoles - 2;
        int Nincx = resfin - resinit + 1;
        Ninc = NA * Nincx + 2;
        FirstConstraint = AppParCurves_TangencyPoint;
        LastConstraint = AppParCurves_TangencyPoint;
        for (i = 1; i <= Vec1t.Upper(); i++)
        {
            Vec1t[i] = V1t[i + lower1 - 1];
            Vec2t[i] = V2t[i + lower2 - 1];
        }
        Perform(Parameters, l1, l2);
    }

    /// <summary>
    /// Computes the approximation with the tangent vectors <paramref name="V1t"/>, <paramref name="V2t"/> and
    /// the curvature vectors <paramref name="V1c"/>, <paramref name="V2c"/> at the first and last points, after
    /// the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector Parameters, math_Vector V1t, math_Vector V2t, math_Vector V1c, math_Vector V2c,
                        double l1, double l2)
    {
        done = false;
        if (!isready)
        {
            return;
        }
        int i, lower1 = V1t.Lower(), lower2 = V2t.Lower();
        int lowc1 = V1c.Lower(), lowc2 = V2c.Lower();
        resinit = 4;
        resfin = nbpoles - 3;
        int Nincx = resfin - resinit + 1;
        Ninc = NA * Nincx + 2;
        FirstConstraint = AppParCurves_CurvaturePoint;
        LastConstraint = AppParCurves_CurvaturePoint;

        for (i = 1; i <= Vec1t.Upper(); i++)
        {
            Vec1t[i] = V1t[i + lower1 - 1];
            Vec2t[i] = V2t[i + lower2 - 1];
            Vec1c[i] = V1c[i + lowc1 - 1];
            Vec2c[i] = V2c[i + lowc2 - 1];
        }
        Perform(Parameters, l1, l2);
    }

    /// <summary>
    /// Computes the approximation with the tangency values <paramref name="l1"/> and <paramref name="l2"/> given,
    /// after the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector Parameters, double l1, double l2)
    {
        done = false;
        if (!isready)
        {
            return;
        }
        if (FirstConstraint < AppParCurves_TangencyPoint && LastConstraint < AppParCurves_TangencyPoint)
        {
            Perform(Parameters);
            return;
        }
        iscalculated = false;

        lambda1 = l1;
        lambda2 = l2;
        int i, j, k, i2;
        double AD0, AD1, AD2, A0, A1, A2;
        double l11 = deg * l1, l22 = deg * l2;

        ComputeFunction(Parameters);

        if (FirstConstraint >= AppParCurves_TangencyPoint)
        {
            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[2, i] = mypoints[myfirstp, i] + l1 * Vec1t[i];
            }
        }

        if (FirstConstraint == AppParCurves_CurvaturePoint)
        {
            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[3, i] = 2 * mypoles[2, i] - mypoles[1, i] + l11 * l11 * Vec1c[i] / (deg * (deg - 1));
            }
        }

        if (LastConstraint >= AppParCurves_TangencyPoint)
        {
            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[nbpoles - 1, i] = mypoints[mylastp, i] - l2 * Vec2t[i];
            }
        }

        if (LastConstraint == AppParCurves_CurvaturePoint)
        {
            for (i = 1; i <= mypoles.ColNumber(); i++)
            {
                mypoles[nbpoles - 2, i] = 2 * mypoles[nbpoles - 1, i] - mypoles[nbpoles, i]
                                          + l22 * l22 * Vec2c[i] / (deg * (deg - 1));
            }
        }

        if (resinit > resfin)
        {
            done = true;
            return;
        }

        if (FirstConstraint == AppParCurves_NoConstraint)
        {
            if (LastConstraint == AppParCurves_TangencyPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - AD0 * mypoles[nbpoles, i] - AD1 * mypoles[nbpoles - 1, i];
                    }
                }
            }
            if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    AD2 = A[j, nbpoles - 2];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - AD0 * mypoles[nbpoles, i] - AD1 * mypoles[nbpoles - 1, i]
                                   - AD2 * mypoles[nbpoles - 2, i];
                    }
                }
            }
        }
        else if (FirstConstraint == AppParCurves_PassPoint)
        {
            if (LastConstraint == AppParCurves_TangencyPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - AD0 * mypoles[nbpoles, i]
                                   - AD1 * mypoles[nbpoles - 1, i];
                    }
                }
            }
            if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    AD2 = A[j, nbpoles - 2];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - AD0 * mypoles[nbpoles, i]
                                   - AD1 * mypoles[nbpoles - 1, i] - AD2 * mypoles[nbpoles - 2, i];
                    }
                }
            }
        }
        else if (FirstConstraint == AppParCurves_TangencyPoint)
        {
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    A1 = A[j, 2];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - A1 * mypoles[2, i];
                    }
                }
            }
            else if (LastConstraint == AppParCurves_PassPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    AD0 = A[j, nbpoles];
                    A1 = A[j, 2];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - AD0 * mypoles[nbpoles, i] - A1 * mypoles[2, i];
                    }
                }
            }
            else if (LastConstraint == AppParCurves_TangencyPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    AD0 = A[j, nbpoles];
                    A1 = A[j, 2];
                    AD1 = A[j, nbpoles - 1];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - AD0 * mypoles[nbpoles, i]
                                   - A1 * mypoles[2, i] - AD1 * mypoles[nbpoles - 1, i];
                    }
                }
            }
        }
        else if (FirstConstraint == AppParCurves_CurvaturePoint)
        {
            if (LastConstraint == AppParCurves_NoConstraint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    A1 = A[j, 2];
                    A2 = A[j, 3];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - A1 * mypoles[2, i] - A2 * mypoles[3, i];
                    }
                }
            }
            else if (LastConstraint == AppParCurves_PassPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    A1 = A[j, 2];
                    A2 = A[j, 3];
                    AD0 = A[j, nbpoles];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - A1 * mypoles[2, i] - A2 * mypoles[3, i]
                                   - AD0 * mypoles[nbpoles, i];
                    }
                }
            }
            else if (LastConstraint == AppParCurves_TangencyPoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    A1 = A[j, 2];
                    A2 = A[j, 3];
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - A1 * mypoles[2, i] - A2 * mypoles[3, i]
                                   - AD0 * mypoles[nbpoles, i] - AD1 * mypoles[nbpoles - 1, i];
                    }
                }
            }
            else if (LastConstraint == AppParCurves_CurvaturePoint)
            {
                for (j = FirstP; j <= LastP; j++)
                {
                    A0 = A[j, 1];
                    A1 = A[j, 2];
                    A2 = A[j, 3];
                    AD0 = A[j, nbpoles];
                    AD1 = A[j, nbpoles - 1];
                    AD2 = A[j, nbpoles - 2];
                    for (i = 1; i <= B2.ColNumber(); i++)
                    {
                        B2[j, i] = mypoints[j, i] - A0 * mypoles[1, i] - A1 * mypoles[2, i] - A2 * mypoles[3, i]
                                   - AD0 * mypoles[nbpoles, i] - AD1 * mypoles[nbpoles - 1, i]
                                   - AD2 * mypoles[nbpoles - 2, i];
                    }
                }
            }
        }

        int Nincx = resfin - resinit + 1;

        math_Matrix mytab = new(resinit, resfin, 1, B2.ColNumber(), 0.0);
        math_IntegerVector Index = new(1, Nincx);
        SearchIndex(Index);
        math_Vector AA = new(1, Index[Nincx], 0.0);
        MakeTAA(AA, mytab);

        math_Vector myTABB = new(1, Nincx, 0.0);

        DACTCL_Decompose(AA, Index);

        int kk2;
        for (j = 1; j <= B2.ColNumber(); j++)
        {
            kk2 = 1;
            for (i = resinit; i <= resfin; i++)
            {
                myTABB[kk2] = mytab[i, j];
                kk2++;
            }

            DACTCL_Solve(AA, myTABB, Index);

            i2 = 1;
            for (k = resinit; k <= resfin; k++)
            {
                mypoles[k, j] = myTABB.Value(i2);
                i2++;
            }
        }

        done = true;
    }

    //=======================================================================
    // function : Affect
    // purpose  :    Index is an ID of the point in MultiLine. Every point is set of
    //            several 3D- and 2D-points. E.g. every points of Walking-line,
    //            obtained in intersection algorithm, is set of one 3D points
    //            (nbP == 1) and two 2D-points (nbP2d == 2).
    //=======================================================================
    /// <summary>Sets the tangent and curvature vectors of a constrained end point, relaxing the constraint if the line has none.</summary>
    protected void Affect(MultiLine SSP, int Index, ref AppParCurves_Constraint Cons, math_Vector Vt, math_Vector Vc)
    {
        // Vt: vector of tangent, Vc: vector of curvature.

        if (Cons >= AppParCurves_TangencyPoint)
        {
            int i, i2 = 1;
            bool Ok;
            int mynbP2d = nbP2d, mynbP = nbP;
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }
            if (nbP == 0)
            {
                mynbP = 1;
            }
            NCollection_Array1<gp_Vec> TabV = new(1, mynbP);
            NCollection_Array1<gp_Vec2d> TabV2d = new(1, mynbP2d);

            if (Cons == AppParCurves_CurvaturePoint)
            {
                if (nbP != 0 && nbP2d != 0)
                {
                    Ok = ToolLine.Curvature(SSP, Index, TabV, TabV2d);
                    if (!Ok)
                    {
                        Cons = AppParCurves_TangencyPoint;
                    }
                }
                else if (nbP2d != 0)
                {
                    Ok = ToolLine.Curvature(SSP, Index, TabV2d);
                    if (!Ok)
                    {
                        Cons = AppParCurves_TangencyPoint;
                    }
                }
                else
                {
                    Ok = ToolLine.Curvature(SSP, Index, TabV);
                    if (!Ok)
                    {
                        Cons = AppParCurves_TangencyPoint;
                    }
                }
                if (Ok)
                {
                    for (i = 1; i <= nbP; i++)
                    {
                        TabV[i].Coord(out Vc[i2], out Vc[i2 + 1], out Vc[i2 + 2]);
                        i2 += 3;
                    }

                    for (i = 1; i <= nbP2d; i++)
                    {
                        TabV2d[i].Coord(out Vc[i2], out Vc[i2 + 1]);
                        i2 += 2;
                    }
                }
            }

            i2 = 1;
            if (Cons >= AppParCurves_TangencyPoint)
            {
                if (nbP != 0 && nbP2d != 0)
                {
                    Ok = ToolLine.Tangency(SSP, Index, TabV, TabV2d);
                    if (!Ok)
                    {
                        Cons = AppParCurves_PassPoint;
                    }
                }
                else if (nbP2d != 0)
                {
                    Ok = ToolLine.Tangency(SSP, Index, TabV2d);
                    if (!Ok)
                    {
                        Cons = AppParCurves_PassPoint;
                    }
                }
                else
                {
                    Ok = ToolLine.Tangency(SSP, Index, TabV);
                    if (!Ok)
                    {
                        Cons = AppParCurves_PassPoint;
                    }
                }

                if (Ok)
                {
                    NCollection_Array1<gp_Pnt> anArrPts3d1 = new(1, mynbP), anArrPts3d2 = new(1, mynbP);

                    if (nbP != 0)
                    {
                        if (Index < ToolLine.LastPoint(SSP))
                        {
                            ToolLine.Value(SSP, Index, anArrPts3d1);
                            ToolLine.Value(SSP, Index + 1, anArrPts3d2);
                        }
                        else
                        { // (Index == ToolLine::LastPoint(theML))
                            ToolLine.Value(SSP, Index - 1, anArrPts3d1);
                            ToolLine.Value(SSP, Index, anArrPts3d2);
                        }

                        CheckTangents(anArrPts3d1, anArrPts3d2, TabV, TabV2d);
                    }
                    else if (nbP2d != 0)
                    {
                        NCollection_Array1<gp_Pnt2d> anArrPts2d1 = new(1, mynbP2d), anArrPts2d2 = new(1, mynbP2d);

                        if (Index < ToolLine.LastPoint(SSP))
                        {
                            ToolLine.Value(SSP, Index, anArrPts3d1, anArrPts2d1);
                            ToolLine.Value(SSP, Index + 1, anArrPts3d2, anArrPts2d2);
                        }
                        else
                        { // (Index == ToolLine::LastPoint(theML))
                            ToolLine.Value(SSP, Index - 1, anArrPts3d1, anArrPts2d1);
                            ToolLine.Value(SSP, Index, anArrPts3d2, anArrPts2d2);
                        }

                        CheckTangents(anArrPts2d1, anArrPts2d2, TabV2d);
                    }

                    for (i = 1; i <= nbP; i++)
                    {
                        TabV[i].Coord(out Vt[i2], out Vt[i2 + 1], out Vt[i2 + 2]);
                        i2 += 3;
                    }

                    for (i = 1; i <= nbP2d; i++)
                    {
                        TabV2d[i].Coord(out Vt[i2], out Vt[i2 + 1]);
                        i2 += 2;
                    }
                }
            }
        }
    }

    /// <summary>The number of columns of the second member: 3 per 3d point, 2 per 2d point.</summary>
    protected int NbBColumns(MultiLine SSP)
    {
        int BCol;
        BCol = (ToolLine.NbP3d(SSP)) * 3 + (ToolLine.NbP2d(SSP)) * 2;
        return BCol;
    }

    /// <summary>
    /// The errors between the MultiLine and the curves: <paramref name="F"/> is the sum of the squared distances,
    /// <paramref name="MaxE3d"/> and <paramref name="MaxE2d"/> the largest 3d and 2d distances.
    /// </summary>
    public void Error(out double F, out double MaxE3d, out double MaxE2d)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        int i, j, k, i2, indexdeb, indexfin;
        int i21, i22;
        double AA, BB, CC, Fi, FX, FY, FZ, AIJ;
        MaxE3d = MaxE2d = 0.0;
        F = 0.0;
        i2 = 1;
        math_Vector Px = new(1, nbpoles), Py = new(1, nbpoles), Pz = new(1, nbpoles);

        for (k = 1; k <= nbP + nbP2d; k++)
        {
            i21 = i2 + 1;
            i22 = i2 + 2;
            for (i = 1; i <= nbpoles; i++)
            {
                Px[i] = mypoles[i, i2];
                Py[i] = mypoles[i, i21];
                if (k <= nbP)
                {
                    Pz[i] = mypoles[i, i22];
                }
            }
            for (i = FirstP; i <= LastP; i++)
            {
                AA = 0.0;
                BB = 0.0;
                CC = 0.0;
                indexdeb = myindex[i] + 1;
                indexfin = indexdeb + deg;
                for (j = indexdeb; j <= indexfin; j++)
                {
                    AIJ = A[i, j];
                    AA += AIJ * Px[j];
                    BB += AIJ * Py[j];
                    if (k <= nbP)
                    {
                        CC += AIJ * Pz[j];
                    }
                }
                FX = AA - mypoints[i, i2];
                FY = BB - mypoints[i, i21];
                Fi = FX * FX + FY * FY;
                if (k <= nbP)
                {
                    FZ = CC - mypoints[i, i22];
                    Fi += FZ * FZ;
                    if (Fi > MaxE3d)
                    {
                        MaxE3d = Fi;
                    }
                }
                else
                {
                    if (Fi > MaxE2d)
                    {
                        MaxE2d = Fi;
                    }
                }
                theError[i, k] = Fi;
                F += Fi;
            }
            if (k <= nbP)
            {
                i2 += 3;
            }
            else
            {
                i2 += 2;
            }
        }
        MaxE3d = Math.Sqrt(MaxE3d);
        MaxE2d = Math.Sqrt(MaxE2d);
    }

    /// <summary>
    /// As <see cref="Error"/>, and fills <paramref name="Grad"/> with the derivative of <paramref name="F"/> with
    /// respect to each parameter.
    /// </summary>
    public void ErrorGradient(math_Vector Grad, out double F, out double MaxE3d, out double MaxE2d)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        int i, j, k, i2, indexdeb, indexfin;
        double AA, BB, CC, Fi, FX, FY, FZ, AIJ;
        double DAIJ, DAA, DBB, DCC, Gr;
        MaxE3d = MaxE2d = 0.0;
        int i21, i22;
        F = 0.0;
        i2 = 1;
        math_Vector Px = new(1, nbpoles), Py = new(1, nbpoles), Pz = new(1, nbpoles);

        for (k = Grad.Lower(); k <= Grad.Upper(); k++)
        {
            Grad[k] = 0.0;
        }

        for (k = 1; k <= nbP + nbP2d; k++)
        {
            i21 = i2 + 1;
            i22 = i2 + 2;
            for (i = 1; i <= nbpoles; i++)
            {
                Px[i] = mypoles[i, i2];
                Py[i] = mypoles[i, i21];
                if (k <= nbP)
                {
                    Pz[i] = mypoles[i, i22];
                }
            }
            for (i = FirstP; i <= LastP; i++)
            {
                AA = 0.0;
                BB = 0.0;
                CC = 0.0;
                DAA = 0.0;
                DBB = 0.0;
                DCC = 0.0;
                indexdeb = myindex[i] + 1;
                indexfin = indexdeb + deg;
                for (j = indexdeb; j <= indexfin; j++)
                {
                    AIJ = A[i, j];
                    DAIJ = DA[i, j];
                    AA += AIJ * Px[j];
                    DAA += DAIJ * Px[j];
                    BB += AIJ * Py[j];
                    DBB += DAIJ * Py[j];
                    if (k <= nbP)
                    {
                        CC += AIJ * Pz[j];
                        DCC += DAIJ * Pz[j];
                    }
                }
                FX = AA - mypoints[i, i2];
                FY = BB - mypoints[i, i2 + 1];
                Fi = FX * FX + FY * FY;
                Gr = 2.0 * (DAA * FX + DBB * FY);

                if (k <= nbP)
                {
                    FZ = CC - mypoints[i, i2 + 2];
                    Fi += FZ * FZ;
                    Gr += 2.0 * DCC * FZ;
                    if (Fi > MaxE3d)
                    {
                        MaxE3d = Fi;
                    }
                }
                else
                {
                    if (Fi > MaxE2d)
                    {
                        MaxE2d = Fi;
                    }
                }
                theError[i, k] = Fi;
                Grad[i] += Gr;
                F += Fi;
            }
            if (k <= nbP)
            {
                i2 += 3;
            }
            else
            {
                i2 += 2;
            }
        }
        MaxE3d = Math.Sqrt(MaxE3d);
        MaxE2d = Math.Sqrt(MaxE2d);
    }

    /// <summary>
    /// The distances between the points of the MultiLine and the curves, after <see cref="Error"/> or
    /// <see cref="ErrorGradient"/> (shared: don't modify).
    /// </summary>
    public math_Matrix Distance()
    {
        if (!iscalculated)
        {
            for (int i = myfirstp; i <= mylastp; i++)
            {
                for (int j = 1; j <= nbP + nbP2d; j++)
                {
                    theError[i, j] = Math.Sqrt(theError[i, j]);
                }
            }
            iscalculated = true;
        }
        return theError;
    }

    /// <summary>(P2 - P1) / V1 if the first point is a tangency point.</summary>
    public double FirstLambda() => lambda1;

    /// <summary>(PN - PN-1) / VN if the last point is a tangency point.</summary>
    public double LastLambda() => lambda2;

    /// <summary>True if the approximation was computed.</summary>
    public bool IsDone() => done;

    /// <summary>The Bezier curves: a MultiCurve sharing the poles of <see cref="BSplineValue"/> (OCCT's slicing copy).</summary>
    public AppParCurves_MultiCurve BezierValue()
    {
        if (myknots is not null)
        {
            throw new Standard_NoSuchObject();
        }
        return new AppParCurves_MultiCurve(BSplineValue());
    }

    /// <summary>The curves; the object's own MultiBSpCurve (shared: don't modify).</summary>
    public AppParCurves_MultiBSpCurve BSplineValue()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        int i, j, j2, npoints = nbP + nbP2d;
        gp_Pnt Pt = new();
        gp_Pnt2d Pt2d = new();
        int ideb = resinit, ifin = resfin;
        if (ideb >= 2)
        {
            ideb = 2;
        }
        if (ifin <= nbpoles - 1)
        {
            ifin = nbpoles - 1;
        }

        // Put the result in the corresponding curves
        for (i = ideb; i <= ifin; i++)
        {
            j2 = 1;
            AppParCurves_MultiPoint MPole = new(nbP, nbP2d);
            for (j = 1; j <= nbP; j++)
            {
                Pt.SetCoord(mypoles[i, j2], mypoles[i, j2 + 1], mypoles[i, j2 + 2]);
                MPole.SetPoint(j, Pt);
                j2 += 3;
            }
            for (j = nbP + 1; j <= npoints; j++)
            {
                Pt2d.SetCoord(mypoles[i, j2], mypoles[i, j2 + 1]);
                MPole.SetPoint2d(j, Pt2d);
                j2 += 2;
            }
            SCU.SetValue(i, MPole);
        }
        return SCU;
    }

    /// <summary>The function matrix A (shared: don't modify).</summary>
    public math_Matrix FunctionMatrix()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return A;
    }

    /// <summary>The derivative function matrix DA (shared: don't modify).</summary>
    public math_Matrix DerivativeFunctionMatrix()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return DA;
    }

    /// <summary>The first point fitted.</summary>
    protected int TheFirstPoint(AppParCurves_Constraint FirstCons, int FirstPoint)
    {
        if (FirstCons == AppParCurves_NoConstraint)
        {
            return FirstPoint;
        }
        else
        {
            return FirstPoint + 1;
        }
    }

    /// <summary>The last point fitted.</summary>
    protected int TheLastPoint(AppParCurves_Constraint LastCons, int LastPoint)
    {
        if (LastCons == AppParCurves_NoConstraint)
        {
            return LastPoint;
        }
        else
        {
            return LastPoint - 1;
        }
    }

    /// <summary>Computes A and DA at the parameters.</summary>
    protected void ComputeFunction(math_Vector Parameters)
    {
        if (myknots is null)
        {
            AppParCurves.Bernstein(nbpoles, Parameters, A, DA);
        }
        else
        {
            AppParCurves.SplineFunction(nbpoles, deg, Parameters, Vflatknots, A, DA, myindex);
        }
    }

    /// <summary>The matrix of the point coordinates (shared: don't modify).</summary>
    public math_Matrix Points() => mypoints;

    /// <summary>The matrix of the pole coordinates (shared: don't modify).</summary>
    public math_Matrix Poles() => mypoles;

    /// <summary>
    /// The index before the first non-zero value of A and DA for each point: they are non-zero from
    /// KIndex(i) + 1 to KIndex(i) + degree + 1 (shared: don't modify).
    /// </summary>
    public math_IntegerVector KIndex() => myindex;

    /// <summary>Builds the tangency system (packed for DACTCL) and its second member.</summary>
    protected void MakeTAA(math_Vector TheA, math_Vector myTAB)
    {
        int i, j, k, Ci, i2, i21, i22;
        double xx = 0.0, yy = 0.0;

        int Nincx = resfin - resinit + 1;
        int Nrow, Neq = LastP - FirstP + 1;

        int Ninc1;

        if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
        {
            Ninc1 = Ninc - 1;
        }
        else
        {
            Ninc1 = Ninc;
        }

        int myfirst = A.LowerRow();
        int ix, iy, iz;
        int mylast = myfirst + Nlignes - 1;
        int k1;
        double taf1 = 0.0, taf2 = 0.0, taf3 = 0.0, tab1 = 0.0, tab2 = 0.0;
        int nb, inc, jinit, jfin, u;
        int indexdeb, indexfin;
        int NA1 = NA - 1;
        double v1 = 0, v2 = 0, b;
        double Ai2, Aid, Akj;
        math_Vector myB = new(myfirst, mylast, 0.0), myV1 = new(myfirst, mylast, 0.0), myV2 = new(myfirst, mylast, 0.0);
        math_Vector TheV1 = new(1, Ninc, 0.0), TheV2 = new(1, Ninc, 0.0);

        for (i = FirstP; i <= LastP; i++)
        {
            Ai2 = A[i, 2];
            Aid = A[i, nbpoles - 1];
            if (FirstConstraint >= AppParCurves_PassPoint)
            {
                xx = A[i, 1];
            }
            if (FirstConstraint >= AppParCurves_TangencyPoint)
            {
                xx += Ai2;
            }
            if (LastConstraint >= AppParCurves_PassPoint)
            {
                yy = A[i, nbpoles];
            }
            if (LastConstraint >= AppParCurves_TangencyPoint)
            {
                yy += Aid;
            }
            i2 = 1;
            Nrow = myfirst - FirstP;
            for (Ci = 1; Ci <= nbP; Ci++)
            {
                i21 = i2 + 1;
                i22 = i2 + 2;
                ix = i + Nrow;
                iy = ix + Neq;
                iz = iy + Neq;
                if (FirstConstraint >= AppParCurves_TangencyPoint)
                {
                    myV1[ix] = Ai2 * Vec1t[i2];
                    myV1[iy] = Ai2 * Vec1t[i21];
                    myV1[iz] = Ai2 * Vec1t[i22];
                }
                if (LastConstraint >= AppParCurves_TangencyPoint)
                {
                    myV2[ix] = -Aid * Vec2t[i2];
                    myV2[iy] = -Aid * Vec2t[i21];
                    myV2[iz] = -Aid * Vec2t[i22];
                }
                myB[ix] = mypoints[i, i2] - xx * mypoints[myfirstp, i2] - yy * mypoints[mylastp, i2];
                myB[iy] = mypoints[i, i21] - xx * mypoints[myfirstp, i21] - yy * mypoints[mylastp, i21];
                myB[iz] = mypoints[i, i22] - xx * mypoints[myfirstp, i22] - yy * mypoints[mylastp, i22];
                i2 += 3;
                Nrow = Nrow + 3 * Neq;
            }

            for (Ci = 1; Ci <= nbP2d; Ci++)
            {
                i21 = i2 + 1;
                i22 = i2 + 2;
                ix = i + Nrow;
                iy = ix + Neq;
                if (FirstConstraint >= AppParCurves_TangencyPoint)
                {
                    myV1[ix] = Ai2 * Vec1t[i2];
                    myV1[iy] = Ai2 * Vec1t[i21];
                }
                if (LastConstraint >= AppParCurves_TangencyPoint)
                {
                    myV2[ix] = -Aid * Vec2t[i2];
                    myV2[iy] = -Aid * Vec2t[i21];
                }
                myB[ix] = mypoints[i, i2] - xx * mypoints[myfirstp, i2] - yy * mypoints[mylastp, i2];
                myB[iy] = mypoints[i, i21] - xx * mypoints[myfirstp, i21] - yy * mypoints[mylastp, i21];
                Nrow = Nrow + 2 * Neq;
                i2 += 2;
            }
        }

        // Construction de TA*A et TA*B:

        for (k = FirstP; k <= LastP; k++)
        {
            indexdeb = myindex[k] + 1;
            indexfin = indexdeb + deg;
            jinit = Math.Max(resinit, indexdeb);
            jfin = Math.Min(resfin, indexfin);
            k1 = k + myfirst - FirstP;
            for (i = 0; i <= NA1; i++)
            {
                nb = i * Neq + k1;
                if (FirstConstraint >= AppParCurves_TangencyPoint)
                {
                    v1 = myV1[nb];
                }
                if (LastConstraint >= AppParCurves_TangencyPoint)
                {
                    v2 = myV2[nb];
                }
                b = myB[nb];
                inc = i * Nincx - resinit + 1;
                for (j = jinit; j <= jfin; j++)
                {
                    Akj = A[k, j];
                    u = j + inc;
                    if (FirstConstraint >= AppParCurves_TangencyPoint)
                    {
                        TheV1[u] += Akj * v1;
                    }
                    if (LastConstraint >= AppParCurves_TangencyPoint)
                    {
                        TheV2[u] += Akj * v2;
                    }
                    myTAB[u] += Akj * b;
                }
                if (FirstConstraint >= AppParCurves_TangencyPoint)
                {
                    taf1 += v1 * v1;
                    tab1 += v1 * b;
                }
                if (LastConstraint >= AppParCurves_TangencyPoint)
                {
                    taf2 += v2 * v2;
                    tab2 += v2 * b;
                }
                if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
                {
                    taf3 += v1 * v2;
                }
            }
        }

        if (FirstConstraint >= AppParCurves_TangencyPoint)
        {
            TheV1[Ninc1] = taf1;
            myTAB[Ninc1] = tab1;
        }
        if (LastConstraint >= AppParCurves_TangencyPoint)
        {
            TheV2[Ninc] = taf2;
            myTAB[Ninc] = tab2;
        }
        if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
        {
            TheV2[Ninc1] = taf3;
        }

        if (resinit <= resfin)
        {
            math_IntegerVector Index = new(1, Nincx);
            SearchIndex(Index);
            math_Vector AA = new(1, Index[Nincx]);
            MakeTAA(AA);

            int kk = 1;
            for (k = 1; k <= NA; k++)
            {
                for (i = 1; i <= AA.Length(); i++)
                {
                    TheA[kk] = AA[i];
                    kk++;
                }
            }
        }

        int length = TheA.Length();

        if (FirstConstraint >= AppParCurves_TangencyPoint && LastConstraint >= AppParCurves_TangencyPoint)
        {
            for (j = 1; j <= Ninc1; j++)
            {
                TheA[length - 2 * Ninc + j + 1] = TheV1[j];
            }
            for (j = 1; j <= Ninc; j++)
            {
                TheA[length - Ninc + j] = TheV2[j];
            }
        }

        else if (FirstConstraint >= AppParCurves_TangencyPoint)
        {
            for (j = 1; j <= Ninc; j++)
            {
                TheA[length - Ninc + j] = TheV1[j];
            }
        }

        else if (LastConstraint >= AppParCurves_TangencyPoint)
        {
            for (j = 1; j <= Ninc; j++)
            {
                TheA[length - Ninc + j] = TheV2[j];
            }
        }
    }

    /// <summary>Builds the packed normal matrix tA A and the second members tA B2.</summary>
    protected void MakeTAA(math_Vector AA, math_Matrix myTAB)
    {
        int i, j, k;
        int indexdeb, indexfin, jinit, jfin;
        int iinit, ifin;
        double Akj;
        math_Matrix TheA = new(resinit, resfin, resinit, resfin);
        TheA.Init(0.0);

        for (k = FirstP; k <= LastP; k++)
        {
            indexdeb = myindex[k] + 1;
            indexfin = indexdeb + deg;
            jinit = Math.Max(resinit, indexdeb);
            jfin = Math.Min(resfin, indexfin);
            for (i = jinit; i <= jfin; i++)
            {
                Akj = A[k, i];
                for (j = jinit; j <= i; j++)
                {
                    TheA[i, j] += A[k, j] * Akj;
                }
                for (j = 1; j <= B2.ColNumber(); j++)
                {
                    myTAB[i, j] += Akj * B2[k, j];
                }
            }
        }

        int len;
        if (myknots is not null)
        {
            len = myknots.Length();
        }
        else
        {
            len = 2;
        }
        int d, i2 = 1;
        iinit = resinit;
        jinit = resinit;
        ifin = Math.Min(resfin, deg + 1);
        for (k = 2; k <= len; k++)
        {
            for (i = iinit; i <= ifin; i++)
            {
                for (j = jinit; j <= i; j++)
                {
                    AA[i2] = TheA[i, j];
                    i2++;
                }
            }
            if (mymults is not null)
            {
                iinit = ifin + 1;
                d = ifin + mymults.Value(k);
                ifin = Math.Min(d, resfin);
                jinit = Math.Max(resinit, d - deg);
            }
        }
    }

    /// <summary>Builds the packed normal matrix tA A.</summary>
    protected void MakeTAA(math_Vector AA)
    {
        int i, j, k;
        int indexdeb, indexfin, jinit, jfin;
        int iinit, ifin;
        double Akj;
        math_Matrix TheA = new(resinit, resfin, resinit, resfin, 0.0);

        for (k = FirstP; k <= LastP; k++)
        {
            indexdeb = myindex[k] + 1;
            indexfin = indexdeb + deg;
            jinit = Math.Max(resinit, indexdeb);
            jfin = Math.Min(resfin, indexfin);
            for (i = jinit; i <= jfin; i++)
            {
                Akj = A[k, i];
                for (j = jinit; j <= i; j++)
                {
                    TheA[i, j] += A[k, j] * Akj;
                }
            }
        }

        int i2 = 1;
        iinit = resinit;
        jinit = resinit;
        ifin = Math.Min(resfin, deg + 1);
        int len, d;
        if (myknots is not null)
        {
            len = myknots.Length();
        }
        else
        {
            len = 2;
        }
        for (k = 2; k <= len; k++)
        {
            for (i = iinit; i <= ifin; i++)
            {
                for (j = jinit; j <= i; j++)
                {
                    AA[i2] = TheA[i, j];
                    i2++;
                }
            }
            if (mymults is not null)
            {
                iinit = ifin + 1;
                d = ifin + mymults.Value(k);
                ifin = Math.Min(d, resfin);
                jinit = Math.Max(resinit, d - deg);
            }
        }
    }

    /// <summary>The DACTCL profile of the packed normal matrix: the index of each diagonal term.</summary>
    protected void SearchIndex(math_IntegerVector Index)
    {
        int iinit, jinit, ifin;
        int i, j, k, d, i2 = 1;
        int len;
        int Nincx = resfin - resinit + 1;
        Index[1] = 1;

        if (myknots is null)
        {
            if (resinit <= resfin)
            {
                int l = 1;
                for (i = 2; i <= Nincx; i++)
                {
                    l++;
                    Index[l] = Index[l - 1] + i;
                }
            }
        }
        else
        {
            iinit = resinit;
            jinit = resinit;
            ifin = Math.Min(resfin, deg + 1);
            len = myknots.Length();

            for (k = 2; k <= len; k++)
            {
                for (i = iinit; i <= ifin; i++)
                {
                    for (j = jinit; j <= i; j++)
                    {
                        if (i2 != 1)
                        {
                            Index[i2] = Index[i2 - 1] + i - jinit + 1;
                        }
                    }
                    i2++;
                }
                iinit = ifin + 1;
                d = ifin + mymults!.Value(k);
                ifin = Math.Min(d, resfin);
                jinit = Math.Max(resinit, d - deg);
            }
        }
    }
}
