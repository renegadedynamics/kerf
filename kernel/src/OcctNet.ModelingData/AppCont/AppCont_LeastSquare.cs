// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_LeastSquare.hxx
// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_LeastSquare.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// Approximates a continuous function (<see cref="AppCont_Function"/>) on [U0, U1] by Bezier curves of one degree,
/// minimising the squared distance integrated with Gauss-Legendre quadrature, with optional pass-point or tangency
/// constraints at the ends. The computation runs in the constructor.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Value"/> returns the object's own MultiCurve (shared: don't modify).
/// As in OCCT, <see cref="IsDone"/> is true only when a constrained system was solved: it stays false when both ends
/// are free (the poles are still computed) or when the constraints fix every pole (occt-issues.md).
/// </remarks>
public class AppCont_LeastSquare
{
    // OCCT's struct PeriodicityInfo, declared in AppCont_LeastSquare.hxx and used only here.
    private struct PeriodicityInfo
    {
        public bool isPeriodic;
        public double myPeriod;
    }

    private readonly AppParCurves_MultiCurve mySCU;
    private readonly math_Matrix myPoints;
    private readonly math_Matrix myPoles;
    private readonly math_Vector myParam;
    private readonly math_Matrix myVB;
    private readonly NCollection_Array1<PeriodicityInfo> myPerInfo;
    private bool myDone;
    private int myDegre;
    private int myNbdiscret, myNbP, myNbP2d;

    // Fix border point evaluation.
    private void FixSingleBorderPoint(AppCont_Function theSSP, double theU, double theU0, double theU1,
                                      NCollection_Array1<gp_Pnt2d> theFix2d, NCollection_Array1<gp_Pnt> theFix)
    {
        int aMaxIter = 15;
        NCollection_Array1<gp_Pnt> aTabP = new(1, Max(myNbP, 1)), aPrevP = new(1, Max(myNbP, 1));
        NCollection_Array1<gp_Pnt2d> aTabP2d = new(1, Max(myNbP2d, 1)), aPrevP2d = new(1, Max(myNbP2d, 1));
        double aMult = ((theU - theU0) > (theU1 - theU)) ? 1.0 : -1.0;
        double aStartParam = theU, aCurrParam, aPrevDist = 1.0, aCurrDist = 1.0;

        double du = -(theU1 - theU0) / 2.0 * aMult;
        double eps = Epsilon(1.0);
        double dd = du, dec = .1;
        for (int anIter = 1; anIter < aMaxIter; anIter++)
        {
            dd *= dec;
            aCurrParam = aStartParam + dd;
            theSSP.Value(aCurrParam, aTabP2d, aTabP);

            // from second iteration
            if (anIter > 1)
            {
                aCurrDist = 0.0;

                for (int j = 1; j <= myNbP; j++)
                {
                    aCurrDist += aTabP[j].Distance(aPrevP[j]);
                }
                for (int j = 1; j <= myNbP2d; j++)
                {
                    aCurrDist += aTabP2d[j].Distance(aPrevP2d[j]);
                }

                // from the third iteration
                if (anIter > 2 && aCurrDist / aPrevDist > 10.0)
                {
                    break;
                }
            }
            aPrevP.Assign(aTabP);
            aPrevP2d.Assign(aTabP2d);
            aPrevDist = aCurrDist;
            if (aPrevDist <= eps)
            {
                break;
            }
        }
        theFix2d.Assign(aPrevP2d);
        theFix.Assign(aPrevP);
    }

    /// <summary>
    /// Approximates <paramref name="SSP"/> on [<paramref name="U0"/>, <paramref name="U1"/>] by Bezier curves of
    /// degree <paramref name="Deg"/> (at most 23), with the constraints <paramref name="FirstCons"/> and
    /// <paramref name="LastCons"/> at the ends (NoConstraint, PassPoint or TangencyPoint; a tangency the function
    /// can't give becomes a pass point), from <paramref name="myNbPoints"/> Gauss points (at most 24).
    /// </summary>
    public AppCont_LeastSquare(AppCont_Function SSP, double U0, double U1, AppParCurves_Constraint FirstCons,
                               AppParCurves_Constraint LastCons, int Deg, int myNbPoints)
    {
        mySCU = new AppParCurves_MultiCurve(Deg + 1);
        myPoints = new math_Matrix(1, myNbPoints, 1, 3 * SSP.GetNbOf3dPoints() + 2 * SSP.GetNbOf2dPoints());
        myPoles = new math_Matrix(1, Deg + 1, 1, 3 * SSP.GetNbOf3dPoints() + 2 * SSP.GetNbOf2dPoints(), 0.0);
        myParam = new math_Vector(1, myNbPoints);
        myVB = new math_Matrix(1, Deg + 1, 1, myNbPoints);
        myPerInfo = new NCollection_Array1<PeriodicityInfo>(1, 3 * SSP.GetNbOf3dPoints() + 2 * SSP.GetNbOf2dPoints());

        myDone = false;
        myDegre = Deg;
        int i, j, k, c, i2;
        int classe = Deg + 1, cl1 = Deg;
        double U, dU, Coeff, Coeff2;
        double IBij, IBPij;

        int FirstP = 1, LastP = myNbPoints;
        int nbcol = 3 * SSP.GetNbOf3dPoints() + 2 * SSP.GetNbOf2dPoints();
        math_Matrix B = new(1, classe, 1, nbcol, 0.0);
        int bdeb = 1, bfin = classe;
        AppParCurves_Constraint myFirstC = FirstCons, myLastC = LastCons;
        SSP.GetNumberOfPoints(out myNbP, out myNbP2d);

        int i2plus1, i2plus2;
        myNbdiscret = myNbPoints;
        NCollection_Array1<gp_Pnt> aTabP = new(1, Max(myNbP, 1));
        NCollection_Array1<gp_Pnt2d> aTabP2d = new(1, Max(myNbP2d, 1));
        NCollection_Array1<gp_Vec> aTabV = new(1, Max(myNbP, 1));
        NCollection_Array1<gp_Vec2d> aTabV2d = new(1, Max(myNbP2d, 1));

        for (int aDimIdx = 1; aDimIdx <= myNbP * 3 + myNbP2d * 2; aDimIdx++)
        {
            SSP.PeriodInformation(aDimIdx, out myPerInfo[aDimIdx].isPeriodic, out myPerInfo[aDimIdx].myPeriod);
        }

        bool Ok;
        if (myFirstC == AppParCurves_TangencyPoint)
        {
            Ok = SSP.D1(U0, aTabV2d, aTabV);
            if (!Ok)
            {
                myFirstC = AppParCurves_PassPoint;
            }
        }

        if (myLastC == AppParCurves_TangencyPoint)
        {
            Ok = SSP.D1(U1, aTabV2d, aTabV);
            if (!Ok)
            {
                myLastC = AppParCurves_PassPoint;
            }
        }

        // Compute control points params on which approximation will be built.
        math_Vector GaussP = new(1, myNbPoints), GaussW = new(1, myNbPoints);
        math.GaussPoints(myNbPoints, GaussP);
        math.GaussWeights(myNbPoints, GaussW);
        math_Vector TheWeights = new(1, myNbPoints), VBParam = new(1, myNbPoints);
        dU = 0.5 * (U1 - U0);
        for (i = FirstP; i <= LastP; i++)
        {
            U = 0.5 * (U1 + U0) + dU * GaussP[i];
            if (i <= (myNbPoints + 1) / 2)
            {
                myParam[LastP - i + 1] = U;
                VBParam[LastP - i + 1] = 0.5 * (1 + GaussP[i]);
                TheWeights[LastP - i + 1] = 0.5 * GaussW[i];
            }
            else
            {
                VBParam[i - (myNbPoints + 1) / 2] = 0.5 * (1 + GaussP[i]);
                myParam[i - (myNbPoints + 1) / 2] = U;
                TheWeights[i - (myNbPoints + 1) / 2] = 0.5 * GaussW[i];
            }
        }

        // Compute control points.
        for (i = FirstP; i <= LastP; i++)
        {
            U = myParam[i];
            SSP.Value(U, aTabP2d, aTabP);

            i2 = 1;
            for (j = 1; j <= myNbP; j++)
            {
                aTabP[j].Coord(out myPoints[i, i2], out myPoints[i, i2 + 1], out myPoints[i, i2 + 2]);
                i2 += 3;
            }
            for (j = 1; j <= myNbP2d; j++)
            {
                aTabP2d[j].Coord(out myPoints[i, i2], out myPoints[i, i2 + 1]);
                i2 += 2;
            }
        }

        // Fix possible "period jump".
        int aMaxDim = 3 * myNbP + 2 * myNbP2d;
        for (int aDimIdx = 1; aDimIdx <= aMaxDim; aDimIdx++)
        {
            if (myPerInfo[aDimIdx].isPeriodic
                && Math.Abs(myPoints[1, aDimIdx] - myPoints[2, aDimIdx]) > myPerInfo[aDimIdx].myPeriod / 2.01
                && Math.Abs(myPoints[2, aDimIdx] - myPoints[3, aDimIdx]) < myPerInfo[aDimIdx].myPeriod / 2.01)
            {
                double aPeriodMult = (myPoints[1, aDimIdx] < myPoints[2, aDimIdx]) ? 1.0 : -1.0;
                double aNewParam = myPoints[1, aDimIdx] + aPeriodMult * myPerInfo[aDimIdx].myPeriod;
                myPoints[1, aDimIdx] = aNewParam;
            }
        }
        for (int aPntIdx = 1; aPntIdx < myNbPoints; aPntIdx++)
        {
            for (int aDimIdx = 1; aDimIdx <= aMaxDim; aDimIdx++)
            {
                if (myPerInfo[aDimIdx].isPeriodic
                    && Math.Abs(myPoints[aPntIdx, aDimIdx] - myPoints[aPntIdx + 1, aDimIdx]) > myPerInfo[aDimIdx].myPeriod / 2.01)
                {
                    double aPeriodMult = (myPoints[aPntIdx, aDimIdx] > myPoints[aPntIdx + 1, aDimIdx]) ? 1.0 : -1.0;
                    double aNewParam = myPoints[aPntIdx + 1, aDimIdx] + aPeriodMult * myPerInfo[aDimIdx].myPeriod;
                    myPoints[aPntIdx + 1, aDimIdx] = aNewParam;
                }
            }
        }

        AppCont_ContMatrices.VBernstein(classe, myNbPoints, myVB);

        // Processing of the right-hand side:
        NCollection_Array1<double> tmppoints = new(1, nbcol);

        for (c = 1; c <= classe; c++)
        {
            tmppoints.Init(0.0);
            for (i = 1; i <= myNbPoints; i++)
            {
                Coeff = TheWeights[i] * myVB[c, i];
                for (j = 1; j <= nbcol; j++)
                {
                    tmppoints[j] += myPoints[i, j] * Coeff;
                }
            }
            for (k = 1; k <= nbcol; k++)
            {
                B[c, k] += tmppoints[k];
            }
        }

        if (myFirstC == AppParCurves_NoConstraint && myLastC == AppParCurves_NoConstraint)
        {
            math_Matrix InvM = new(1, classe, 1, classe);
            AppCont_ContMatrices.InvMMatrix(classe, InvM);
            // Direct computation of poles:

            for (i = 1; i <= classe; i++)
            {
                for (j = 1; j <= classe; j++)
                {
                    IBij = InvM[i, j];
                    for (k = 1; k <= nbcol; k++)
                    {
                        myPoles[i, k] += IBij * B[j, k];
                    }
                }
            }
            // OCCT: myDone stays false here although the poles are computed (occt-issues.md).
        }
        else
        {
            math_Matrix M = new(1, classe, 1, classe);
            AppCont_ContMatrices.MMatrix(classe, M);
            NCollection_Array1<gp_Pnt2d> aFixP2d = new(1, Max(myNbP2d, 1));
            NCollection_Array1<gp_Pnt> aFixP = new(1, Max(myNbP, 1));

            if (myFirstC == AppParCurves_PassPoint || myFirstC == AppParCurves_TangencyPoint)
            {
                SSP.Value(U0, aTabP2d, aTabP);
                FixSingleBorderPoint(SSP, U0, U0, U1, aFixP2d, aFixP);

                i2 = 1;
                for (k = 1; k <= myNbP; k++)
                {
                    if (aFixP[k].Distance(aTabP[k]) > 0.1)
                    {
                        aFixP[k].Coord(out myPoles[1, i2], out myPoles[1, i2 + 1], out myPoles[1, i2 + 2]);
                    }
                    else
                    {
                        aTabP[k].Coord(out myPoles[1, i2], out myPoles[1, i2 + 1], out myPoles[1, i2 + 2]);
                    }
                    i2 += 3;
                }
                for (k = 1; k <= myNbP2d; k++)
                {
                    if (aFixP2d[k].Distance(aTabP2d[k]) > 0.1)
                    {
                        aFixP2d[k].Coord(out myPoles[1, i2], out myPoles[1, i2 + 1]);
                    }
                    else
                    {
                        aTabP2d[k].Coord(out myPoles[1, i2], out myPoles[1, i2 + 1]);
                    }
                    i2 += 2;
                }

                for (int aDimIdx = 1; aDimIdx <= aMaxDim; aDimIdx++)
                {
                    if (myPerInfo[aDimIdx].isPeriodic
                        && Math.Abs(myPoles[1, aDimIdx] - myPoints[1, aDimIdx]) > myPerInfo[aDimIdx].myPeriod / 2.01)
                    {
                        double aMult = myPoles[1, aDimIdx] < myPoints[1, aDimIdx] ? 1.0 : -1.0;
                        myPoles[1, aDimIdx] += aMult * myPerInfo[aDimIdx].myPeriod;
                    }
                }
            }

            if (myLastC == AppParCurves_PassPoint || myLastC == AppParCurves_TangencyPoint)
            {
                SSP.Value(U1, aTabP2d, aTabP);
                FixSingleBorderPoint(SSP, U1, U0, U1, aFixP2d, aFixP);

                i2 = 1;
                for (k = 1; k <= myNbP; k++)
                {
                    if (aFixP[k].Distance(aTabP[k]) > 0.1)
                    {
                        aFixP[k].Coord(out myPoles[classe, i2], out myPoles[classe, i2 + 1], out myPoles[classe, i2 + 2]);
                    }
                    else
                    {
                        aTabP[k].Coord(out myPoles[classe, i2], out myPoles[classe, i2 + 1], out myPoles[classe, i2 + 2]);
                    }
                    i2 += 3;
                }
                for (k = 1; k <= myNbP2d; k++)
                {
                    if (aFixP2d[k].Distance(aTabP2d[k]) > 0.1)
                    {
                        aFixP2d[k].Coord(out myPoles[classe, i2], out myPoles[classe, i2 + 1]);
                    }
                    else
                    {
                        aTabP2d[k].Coord(out myPoles[classe, i2], out myPoles[classe, i2 + 1]);
                    }
                    i2 += 2;
                }

                // OCCT: only the first two coordinates are checked for a period jump here, where the first point
                // checks all aMaxDim (occt-issues.md).
                for (int aDimIdx = 1; aDimIdx <= 2; aDimIdx++)
                {
                    if (myPerInfo[aDimIdx].isPeriodic
                        && Math.Abs(myPoles[classe, aDimIdx] - myPoints[myNbPoints, aDimIdx]) > myPerInfo[aDimIdx].myPeriod / 2.01)
                    {
                        double aMult = myPoles[classe, aDimIdx] < myPoints[myNbPoints, aDimIdx] ? 1.0 : -1.0;
                        myPoles[classe, aDimIdx] += aMult * myPerInfo[aDimIdx].myPeriod;
                    }
                }
            }

            if (myFirstC == AppParCurves_PassPoint)
            {
                bdeb = 2;
                // Update the right-hand side:
                for (i = 1; i <= classe; i++)
                {
                    Coeff = M[i, 1];
                    for (k = 1; k <= nbcol; k++)
                    {
                        B[i, k] -= myPoles[1, k] * Coeff;
                    }
                }
            }

            if (myLastC == AppParCurves_PassPoint)
            {
                bfin = cl1;
                for (i = 1; i <= classe; i++)
                {
                    Coeff = M[i, classe];
                    for (k = 1; k <= nbcol; k++)
                    {
                        B[i, k] -= myPoles[classe, k] * Coeff;
                    }
                }
            }

            if (myFirstC == AppParCurves_TangencyPoint)
            {
                // Fix the second pole:
                bdeb = 3;
                SSP.D1(U0, aTabV2d, aTabV);

                i2 = 1;
                Coeff = (U1 - U0) / myDegre;
                for (k = 1; k <= myNbP; k++)
                {
                    i2plus1 = i2 + 1;
                    i2plus2 = i2 + 2;
                    myPoles[2, i2] = myPoles[1, i2] + aTabV[k].X() * Coeff;
                    myPoles[2, i2plus1] = myPoles[1, i2plus1] + aTabV[k].Y() * Coeff;
                    myPoles[2, i2plus2] = myPoles[1, i2plus2] + aTabV[k].Z() * Coeff;
                    i2 += 3;
                }
                for (k = 1; k <= myNbP2d; k++)
                {
                    i2plus1 = i2 + 1;
                    myPoles[2, i2] = myPoles[1, i2] + aTabV2d[k].X() * Coeff;
                    myPoles[2, i2plus1] = myPoles[1, i2plus1] + aTabV2d[k].Y() * Coeff;
                    i2 += 2;
                }

                for (i = 1; i <= classe; i++)
                {
                    Coeff = M[i, 1];
                    Coeff2 = M[i, 2];
                    for (k = 1; k <= nbcol; k++)
                    {
                        B[i, k] -= myPoles[1, k] * Coeff + myPoles[2, k] * Coeff2;
                    }
                }
            }

            if (myLastC == AppParCurves_TangencyPoint)
            {
                bfin = classe - 2;
                SSP.D1(U1, aTabV2d, aTabV);
                i2 = 1;
                Coeff = (U1 - U0) / myDegre;
                for (k = 1; k <= myNbP; k++)
                {
                    i2plus1 = i2 + 1;
                    i2plus2 = i2 + 2;
                    myPoles[cl1, i2] = myPoles[classe, i2] - aTabV[k].X() * Coeff;
                    myPoles[cl1, i2plus1] = myPoles[classe, i2plus1] - aTabV[k].Y() * Coeff;
                    myPoles[cl1, i2plus2] = myPoles[classe, i2plus2] - aTabV[k].Z() * Coeff;
                    i2 += 3;
                }
                for (k = 1; k <= myNbP2d; k++)
                {
                    i2plus1 = i2 + 1;
                    myPoles[cl1, i2] = myPoles[classe, i2] - aTabV2d[k].X() * Coeff;
                    myPoles[cl1, i2plus1] = myPoles[classe, i2plus1] - aTabV2d[k].Y() * Coeff;
                    i2 += 2;
                }

                for (i = 1; i <= classe; i++)
                {
                    Coeff = M[i, classe];
                    Coeff2 = M[i, cl1];
                    for (k = 1; k <= nbcol; k++)
                    {
                        B[i, k] -= myPoles[classe, k] * Coeff + myPoles[cl1, k] * Coeff2;
                    }
                }
            }

            if (bdeb <= bfin)
            {
                math_Matrix B2 = new(bdeb, bfin, 1, B.UpperCol(), 0.0);

                for (i = bdeb; i <= bfin; i++)
                {
                    for (j = 1; j <= classe; j++)
                    {
                        Coeff = M[i, j];
                        for (k = 1; k <= nbcol; k++)
                        {
                            B2[i, k] += B[j, k] * Coeff;
                        }
                    }
                }

                // Resolution:
                // ===========
                math_Matrix IBP = new(bdeb, bfin, bdeb, bfin);

                // In IBPMatrix and IBTMatrix only results for a degree
                // less than or equal to 26 are stored (for now at least).

                if (bdeb == 2 && bfin == classe - 1 && classe <= 26)
                {
                    AppCont_ContMatrices.IBPMatrix(classe, IBP);
                }
                else if (bdeb == 3 && bfin == classe - 2 && classe <= 26)
                {
                    AppCont_ContMatrices.IBTMatrix(classe, IBP);
                }
                else
                {
                    math_Matrix MP = new(1, classe, bdeb, bfin);
                    for (i = 1; i <= classe; i++)
                    {
                        for (j = bdeb; j <= bfin; j++)
                        {
                            MP[i, j] = M[i, j];
                        }
                    }
                    math_Matrix IBP1 = new(bdeb, bfin, bdeb, bfin);
                    IBP1.Initialized(MP.Transposed() * MP);
                    IBP.Initialized(IBP1.Inverse());
                }

                myDone = true;
                for (i = bdeb; i <= bfin; i++)
                {
                    for (j = bdeb; j <= bfin; j++)
                    {
                        IBPij = IBP[i, j];
                        for (k = 1; k <= nbcol; k++)
                        {
                            myPoles[i, k] += IBPij * B2[j, k];
                        }
                    }
                }
            }
        }
    }

    /// <summary>The approximation: Degree + 1 MultiPoints, rebuilt from the poles at each call (shared: don't modify).</summary>
    public AppParCurves_MultiCurve Value()
    {
        int i, j, j2;
        gp_Pnt Pt = new();
        gp_Pnt2d Pt2d = new();
        int ideb = 1, ifin = myDegre + 1;

        // Store the result in the corresponding curves
        for (i = ideb; i <= ifin; i++)
        {
            j2 = 1;
            AppParCurves_MultiPoint MPole = new(myNbP, myNbP2d);
            for (j = 1; j <= myNbP; j++)
            {
                Pt.SetCoord(myPoles[i, j2], myPoles[i, j2 + 1], myPoles[i, j2 + 2]);
                MPole.SetPoint(j, Pt);
                j2 += 3;
            }
            for (j = myNbP + 1; j <= myNbP + myNbP2d; j++)
            {
                Pt2d.SetCoord(myPoles[i, j2], myPoles[i, j2 + 1]);
                MPole.SetPoint2d(j, Pt2d);
                j2 += 2;
            }
            mySCU.SetValue(i, MPole);
        }
        return mySCU;
    }

    /// <summary>
    /// The errors at the Gauss points: <paramref name="F"/> the sum of the squared distances,
    /// <paramref name="MaxE3d"/> and <paramref name="MaxE2d"/> the largest 3d and 2d distances.
    /// </summary>
    public void Error(out double F, out double MaxE3d, out double MaxE2d)
    {
        int i, j, k, c, i2, classe = myDegre + 1;
        double Coeff, err3d = 0.0, err2d = 0.0;
        int ncol = myPoints.UpperCol() - myPoints.LowerCol() + 1;

        math_Matrix MyPoints = new(1, myNbdiscret, 1, ncol);
        MyPoints.Initialized(myPoints);

        MaxE3d = MaxE2d = F = 0.0;

        NCollection_Array1<double> tmppoles = new(1, ncol);

        for (c = 1; c <= classe; c++)
        {
            for (k = 1; k <= ncol; k++)
            {
                tmppoles[k] = myPoles[c, k];
            }
            for (i = 1; i <= myNbdiscret; i++)
            {
                Coeff = myVB[c, i];
                for (j = 1; j <= ncol; j++)
                {
                    MyPoints[i, j] -= tmppoles[j] * Coeff;
                }
            }
        }

        double e1, e2, e3;
        for (i = 1; i <= myNbdiscret; i++)
        {
            i2 = 1;
            for (j = 1; j <= myNbP; j++)
            {
                e1 = MyPoints[i, i2];
                e2 = MyPoints[i, i2 + 1];
                e3 = MyPoints[i, i2 + 2];
                err3d = e1 * e1 + e2 * e2 + e3 * e3;
                MaxE3d = Max(MaxE3d, err3d);
                F += err3d;
                i2 += 3;
            }
            for (j = 1; j <= myNbP2d; j++)
            {
                e1 = MyPoints[i, i2];
                e2 = MyPoints[i, i2 + 1];
                err2d = e1 * e1 + e2 * e2;
                MaxE2d = Max(MaxE2d, err2d);
                F += err2d;
                i2 += 2;
            }
        }

        MaxE3d = Math.Sqrt(MaxE3d);
        MaxE2d = Math.Sqrt(MaxE2d);
    }

    /// <summary>True if a constrained system was solved (see the remarks).</summary>
    public bool IsDone() => myDone;
}
