// Port of: src/ModelingData/TKGeomBase/Approx/Approx_ComputeLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;
using static Occt.Approx_Status;

namespace Occt;

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
public class Approx_ComputeLine<MultiLine, LineTool>
    where LineTool : struct, IApprox_LineTool<MultiLine>
{
    private readonly NCollection_Sequence<AppParCurves_MultiCurve> myMultiCurves = new();
    private AppParCurves_MultiCurve TheMultiCurve = new();
    private AppParCurves_MultiBSpCurve myspline = new();
    private bool alldone;
    private bool tolreached; // OCCT never initialises it before the first approximation; false here
    private Approx_ParametrizationType Par;
    private NCollection_HArray1<double>? myParameters;
    private NCollection_HArray1<double>? myfirstParam;
    private readonly NCollection_Sequence<NCollection_HArray1<double>> myPar = new();
    private readonly NCollection_Sequence<double> Tolers3d = new();
    private readonly NCollection_Sequence<double> Tolers2d = new();
    private readonly NCollection_HArray1<AppParCurves_ConstraintCouple> myConstraints;
    private int mydegremin;
    private int mydegremax;
    private double mytol3d;
    private double mytol2d;
    private double currenttol3d;
    private double currenttol2d;
    private bool mycut;
    private bool mysquares;
    private int myitermax;
    private AppParCurves_Constraint myfirstC;
    private AppParCurves_Constraint mylastC;
    private int myMultiLineNb;
    private bool myIsClear;

    // OCCT's file-static CheckMultiCurve: false (with the index of a bad point in theIndbad) if the Bezier
    // curve makes a loop that the points don't.
    private static bool CheckMultiCurve(AppParCurves_MultiCurve theMultiCurve, MultiLine theLine, int theIndfirst,
                                        int theIndlast, out int theIndbad)
    {
        int nbp3d = LineTool.NbP3d(theLine);
        int nbp2d = LineTool.NbP2d(theLine);

        const double coeff = 4.0; // 2*2

        theIndbad = 0;
        if (nbp3d > 1) // only simple cases are analysed
        {
            return true;
        }

        const double MinScalProd = -0.9;
        double SqTol3d = Precision.SquareConfusion();

        // OCCT writes indbads[indcur] for every curve, past its 4 elements when a line has more than 3 curves
        // (2 2d curves beside a 3d one); the port throws there (occt-issues.md).
        Span<int> indbads = stackalloc int[4];
        indbads[1] = indbads[2] = indbads[3] = 0;

        int NbCur = theMultiCurve.NbCurves();
        bool LoopFound = false;

        int aNbP3d = Max(nbp3d, 1);
        int aNbP2d = Max(nbp2d, 1);

        NCollection_Array1<gp_Pnt> tabP = new(1, aNbP3d);
        NCollection_Array1<gp_Pnt2d> tabP2d = new(1, aNbP2d);

        if (theMultiCurve.Dimension(1) == 3 /*myNbP3d == 1*/)
        {
            NCollection_Array1<gp_Pnt> aPoles = new(1, theMultiCurve.NbPoles());
            theMultiCurve.Curve(1, aPoles);
            gp_Vec FirstVec = new(), SecondVec;
            int indp = 2;
            while (indp <= aPoles.Upper())
            {
                FirstVec = new gp_Vec(aPoles[1], aPoles[indp++]);
                double aLength = FirstVec.Magnitude();
                if (aLength > gp.Resolution())
                {
                    FirstVec /= aLength;
                    break;
                }
            }
            gp_Pnt MidPnt = aPoles[indp - 1];
            // for (int k = 3; k <= aPoles.Upper(); k++)
            while (indp <= aPoles.Upper())
            {
                SecondVec = new gp_Vec(MidPnt, aPoles[indp]);
                double aLength = SecondVec.Magnitude();
                if (aLength <= gp.Resolution())
                {
                    indp++;
                    continue;
                }
                SecondVec /= aLength;
                double ScalProd = FirstVec * SecondVec;
                if (ScalProd < MinScalProd)
                {
                    LoopFound = true;
                    break;
                }
                FirstVec = SecondVec;
                MidPnt = aPoles[indp];
                indp++;
            }
            // Check: may be it is a real loop
            if (LoopFound)
            {
                for (int FirstInd = theIndfirst; FirstInd <= theIndlast - 2; FirstInd++)
                {
                    LineTool.Value(theLine, FirstInd, tabP);
                    gp_Pnt FirstPnt = tabP[1];
                    for (int k = FirstInd + 1; k < theIndlast; k++)
                    {
                        LineTool.Value(theLine, k, tabP);
                        gp_Pnt Pnt1 = tabP[1];
                        LineTool.Value(theLine, k + 1, tabP);
                        gp_Pnt Pnt2 = tabP[1];
                        if (FirstPnt.SquareDistance(Pnt1) <= SqTol3d || FirstPnt.SquareDistance(Pnt2) <= SqTol3d)
                        {
                            LoopFound = false;
                            break;
                        }
                        gp_Vec Vec1 = new(FirstPnt, Pnt1);
                        Vec1.Normalize();
                        gp_Vec Vec2 = new(FirstPnt, Pnt2);
                        Vec2.Normalize();
                        double ScalProd = Vec1 * Vec2;
                        if (ScalProd < MinScalProd)
                        {
                            LoopFound = false;
                            break;
                        }
                    }
                    if (LoopFound == false)
                    {
                        break;
                    }
                }
            }
            if (LoopFound)
            {
                // search <indbad>
                double MaxSqDist = 0.0;
                double MinSqDist = RealLast();
                for (int k = theIndfirst + 1; k <= theIndlast; k++)
                {
                    LineTool.Value(theLine, k - 1, tabP);
                    gp_Pnt PrevPnt = tabP[1];
                    LineTool.Value(theLine, k, tabP);
                    gp_Pnt CurPnt = tabP[1];
                    double aSqDist = PrevPnt.SquareDistance(CurPnt);
                    if (aSqDist > MaxSqDist)
                    {
                        MaxSqDist = aSqDist;
                        indbads[1] = k;
                    }
                    if (aSqDist > gp.Resolution() && aSqDist < MinSqDist)
                    {
                        MinSqDist = aSqDist;
                    }
                }
                double Relation = MaxSqDist / MinSqDist;
                if (Relation < coeff)
                {
                    LoopFound = false;
                }
                else
                {
                    for (int indcur = 2; indcur <= NbCur; indcur++)
                    {
                        MaxSqDist = 0.0;
                        for (int k = theIndfirst + 1; k <= theIndlast; k++)
                        {
                            LineTool.Value(theLine, k - 1, tabP2d);
                            gp_Pnt2d PrevPnt = tabP2d[indcur - 1];
                            LineTool.Value(theLine, k, tabP2d);
                            gp_Pnt2d CurPnt = tabP2d[indcur - 1];
                            double aSqDist = PrevPnt.SquareDistance(CurPnt);
                            if (aSqDist > MaxSqDist)
                            {
                                MaxSqDist = aSqDist;
                                indbads[indcur] = k;
                            }
                        }
                    }
                }
            }
        } // if (myNbP3d == 1)
        else // 2d case
        {
            NCollection_Array1<gp_Pnt2d> aPoles2d = new(1, theMultiCurve.NbPoles());
            theMultiCurve.Curve(1, aPoles2d);
            double aSqNormToler = Epsilon(1.0) * Epsilon(1.0);
            gp_Vec2d FirstVec = new(aPoles2d[1], aPoles2d[2]), SecondVec = new();
            double aVecSqNorm = FirstVec.SquareMagnitude();
            if (aVecSqNorm < aSqNormToler)
            {
                theIndbad = theIndfirst + 1;
                return false;
            }

            FirstVec /= Math.Sqrt(aSqNormToler); // OCCT: divides by the tolerance, not by the norm (occt-issues.md)
            gp_Pnt2d MidPnt = aPoles2d[2];
            for (int k = 3; k <= aPoles2d.Upper(); k++)
            {
                SecondVec.SetXY(aPoles2d[k].XY() - MidPnt.XY());
                aVecSqNorm = SecondVec.SquareMagnitude();
                if (aVecSqNorm < aSqNormToler)
                {
                    theIndbad = theIndfirst + k - 1;
                    return false;
                }

                SecondVec /= Math.Sqrt(aVecSqNorm);
                double ScalProd = FirstVec * SecondVec;
                if (ScalProd < MinScalProd)
                {
                    LoopFound = true;
                    break;
                }
                FirstVec = SecondVec;
                MidPnt = aPoles2d[k];
            }
            // Check: may be it is a real loop
            if (LoopFound)
            {
                for (int FirstInd = theIndfirst; FirstInd <= theIndlast - 2; FirstInd++)
                {
                    LineTool.Value(theLine, FirstInd, tabP2d);
                    gp_Pnt2d FirstPnt = tabP2d[1];
                    for (int k = FirstInd + 1; k < theIndlast; k++)
                    {
                        LineTool.Value(theLine, k, tabP2d);
                        gp_Pnt2d Pnt1 = tabP2d[1];
                        LineTool.Value(theLine, k + 1, tabP2d);
                        gp_Pnt2d Pnt2 = tabP2d[1];
                        if (FirstPnt.SquareDistance(Pnt1) <= SqTol3d || FirstPnt.SquareDistance(Pnt2) <= SqTol3d)
                        {
                            LoopFound = false;
                            break;
                        }
                        gp_Vec2d Vec1 = new(FirstPnt, Pnt1);
                        Vec1.Normalize();
                        gp_Vec2d Vec2 = new(FirstPnt, Pnt2);
                        Vec2.Normalize();
                        double ScalProd = Vec1 * Vec2;
                        if (ScalProd < MinScalProd)
                        {
                            LoopFound = false;
                            break;
                        }
                    }
                    if (LoopFound == false)
                    {
                        break;
                    }
                }
            }
            if (LoopFound)
            {
                // search <indbad>
                for (int indcur = 1; indcur <= NbCur; indcur++)
                {
                    double MaxSqDist = 0.0;
                    double MinSqDist = RealLast();
                    for (int k = theIndfirst + 1; k <= theIndlast; k++)
                    {
                        LineTool.Value(theLine, k - 1, tabP2d);
                        gp_Pnt2d PrevPnt = tabP2d[indcur];
                        LineTool.Value(theLine, k, tabP2d);
                        gp_Pnt2d CurPnt = tabP2d[indcur];
                        double aSqDist = PrevPnt.SquareDistance(CurPnt);
                        if (aSqDist > MaxSqDist)
                        {
                            MaxSqDist = aSqDist;
                            indbads[indcur] = k;
                        }
                        if (aSqDist > gp.Resolution() && aSqDist < MinSqDist)
                        {
                            MinSqDist = aSqDist;
                        }
                    }
                    double Relation = MaxSqDist / MinSqDist;
                    if (Relation < coeff)
                    {
                        LoopFound = false;
                    }
                }
            }
        }

        // Define <indbad>
        for (int i = 1; i <= 3; i++)
        {
            if (indbads[i] != 0)
            {
                theIndbad = indbads[i];
                break;
            }
        }

        if (!LoopFound)
        {
            theIndbad = 0;
        }

        return (!LoopFound);
    }

    private void FirstTangencyVector(MultiLine Line, int index, math_Vector V)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool Ok = false;
        NCollection_Array1<gp_Vec> TabV = new(1, mynbP3d);
        NCollection_Array1<gp_Vec2d> TabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV, TabV2d);
        }
        else if (nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV2d);
        }
        else if (nbP3d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV);
        }

        if (Ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = TabV.Lower(); i <= TabV.Upper(); i++)
                {
                    V[j] = TabV[i].X();
                    V[j + 1] = TabV[i].Y();
                    V[j + 2] = TabV[i].Z();
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = TabV2d.Lower(); i <= TabV2d.Upper(); i++)
                {
                    V[j] = TabV2d[i].X();
                    V[j + 1] = TabV2d[i].Y();
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            AppParCurves_Constraint firstC, lastC;
            firstC = lastC = AppParCurves_PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index, index + 2);
            Parameters(Line, index, index + 2, mypar);
            AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, index, index + 2, firstC, lastC, mypar, nbpoles);
            AppParCurves_MultiCurve C = LSQ.BezierValue();

            gp_Pnt myP;
            gp_Vec myV;
            gp_Pnt2d myP2d;
            gp_Vec2d myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C.D1(i, 0.0, out myP, out myV);
                V[j] = myV.X();
                V[j + 1] = myV.Y();
                V[j + 2] = myV.Z();
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                C.D1(i, 0.0, out myP2d, out myV2d);
                V[j] = myV2d.X();
                V[j + 1] = myV2d.Y();
                j += 2;
            }
        }
    }

    private void LastTangencyVector(MultiLine Line, int index, math_Vector V)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool Ok = false;
        NCollection_Array1<gp_Vec> TabV = new(1, mynbP3d);
        NCollection_Array1<gp_Vec2d> TabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV, TabV2d);
        }
        else if (nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV2d);
        }
        else if (nbP3d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV);
        }

        if (Ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = TabV.Lower(); i <= TabV.Upper(); i++)
                {
                    V[j] = TabV[i].X();
                    V[j + 1] = TabV[i].Y();
                    V[j + 2] = TabV[i].Z();
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = TabV2d.Lower(); i <= TabV2d.Upper(); i++)
                {
                    V[j] = TabV2d[i].X();
                    V[j + 1] = TabV2d[i].Y();
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            AppParCurves_Constraint firstC, lastC;
            firstC = lastC = AppParCurves_PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index - 2, index);
            Parameters(Line, index - 2, index, mypar);
            AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, index - 2, index, firstC, lastC, mypar, nbpoles);
            AppParCurves_MultiCurve C = LSQ.BezierValue();

            gp_Pnt myP;
            gp_Vec myV;
            gp_Pnt2d myP2d;
            gp_Vec2d myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C.D1(i, 1.0, out myP, out myV);
                V[j] = myV.X();
                V[j + 1] = myV.Y();
                V[j + 2] = myV.Z();
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                C.D1(i, 1.0, out myP2d, out myV2d);
                V[j] = myV2d.X();
                V[j + 1] = myV2d.Y();
                j += 2;
            }
        }
    }

    private static double SearchFirstLambda(MultiLine Line, math_Vector TheParam, math_Vector V, int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        gp_Pnt P1, P2;
        gp_Pnt2d P12d, P22d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<gp_Pnt> tabP1 = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<gp_Pnt2d> tabP12d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP1, tabP12d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP12d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index, tabP1);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index + 1, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index + 1, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index + 1, tabP2);
        }

        double U1 = TheParam[index], U2 = TheParam[index + 1];
        double lambda, S;
        int low = V.Lower();

        if (nbP3d != 0)
        {
            P1 = tabP1[1];
            P2 = tabP2[1];
            gp_Vec P1P2 = new(P1, P2), myV = new();
            myV.SetCoord(V[low], V[low + 1], V[low + 2]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            P12d = tabP12d[1];
            P22d = tabP22d[1];
            gp_Vec2d P1P2 = new(P12d, P22d), myV = new();
            myV.SetCoord(V[low], V[low + 1]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        return (S * lambda);
    }

    private static double SearchLastLambda(MultiLine Line, math_Vector TheParam, math_Vector V, int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        gp_Pnt P1, P2;
        gp_Pnt2d P12d, P22d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<gp_Pnt> tabP = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<gp_Pnt2d> tabP2d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index - 1, tabP, tabP2d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index - 1, tabP2d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index - 1, tabP);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index, tabP2);
        }

        double U1 = TheParam[index - 1], U2 = TheParam[index];
        double lambda, S;
        int low = V.Lower();

        if (nbP3d != 0)
        {
            P1 = tabP[1];
            P2 = tabP2[1];
            gp_Vec P1P2 = new(P1, P2), myV = new();
            myV.SetCoord(V[low], V[low + 1], V[low + 2]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            P12d = tabP2d[1];
            P22d = tabP22d[1];
            gp_Vec2d P1P2 = new(P12d, P22d), myV = new();
            myV.SetCoord(V[low], V[low + 1]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }

        return (S * lambda);
    }

    /// <summary>
    /// Approximates <paramref name="Line"/> from the parameters <paramref name="Parameters"/> until the tolerances
    /// are reached, from degree <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if
    /// <paramref name="cutting"/>. With <paramref name="Squares"/>, without iterations.
    /// </summary>
    public Approx_ComputeLine(MultiLine Line, math_Vector Parameters, int degreemin = 4, int degreemax = 8,
                              double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06, int NbIterations = 5,
                              bool cutting = true, bool Squares = false)
    {
        myMultiLineNb = 0;
        myIsClear = false;
        myfirstParam = new NCollection_HArray1<double>(Parameters.Lower(), Parameters.Upper());
        for (int i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myfirstParam.SetValue(i, Parameters[i]);
        }
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = Approx_IsoParametric;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        alldone = false;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        Perform(Line);
    }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="Parameters"/>.</summary>
    public Approx_ComputeLine(math_Vector Parameters, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                              double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true,
                              bool Squares = false)
    {
        myMultiLineNb = 0;
        myIsClear = false;
        myfirstParam = new NCollection_HArray1<double>(Parameters.Lower(), Parameters.Upper());
        for (int i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myfirstParam.SetValue(i, Parameters[i]);
        }
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = Approx_IsoParametric;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        alldone = false;
    }

    /// <summary>Initialises the fields of the algorithm; <see cref="Perform"/> runs it.</summary>
    public Approx_ComputeLine(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                              double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true,
                              Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        myMultiLineNb = 0;
        myIsClear = false;
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = parametrization;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        alldone = false;
    }

    /// <summary>
    /// Approximates <paramref name="Line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if
    /// <paramref name="cutting"/>. With <paramref name="Squares"/>, without iterations.
    /// </summary>
    public Approx_ComputeLine(MultiLine Line, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-3,
                              double Tolerance2d = 1.0e-6, int NbIterations = 5, bool cutting = true,
                              Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        myMultiLineNb = 0;
        myIsClear = false;
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        alldone = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        Par = parametrization;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;

        Perform(Line);
    }

    /// <summary>Runs the algorithm on <paramref name="Line"/> after the fields have been initialised.</summary>
    public void Perform(MultiLine Line)
    {
        if (!myIsClear)
        {
            myMultiCurves.Clear();
            myPar.Clear();
            Tolers3d.Clear();
            Tolers2d.Clear();
            myMultiLineNb = 0;
        }
        else
        {
            myIsClear = false;
        }

        int i, nbp, Thefirstpt, Thelastpt, oldlastpt;
        bool Finish = false, begin = true, Ok = false, GoUp = false, Interpol;
        double thetol3d, thetol2d;
        Approx_Status MyStatus;
        Thefirstpt = LineTool.FirstPoint(Line);
        Thelastpt = LineTool.LastPoint(Line);
        int myfirstpt = Thefirstpt;
        int mylastpt = Thelastpt;

        AppParCurves_ConstraintCouple myCouple1 = new(myfirstpt, myfirstC);
        AppParCurves_ConstraintCouple myCouple2 = new(mylastpt, mylastC);
        myConstraints.SetValue(1, myCouple1);
        myConstraints.SetValue(2, myCouple2);

        math_Vector TheParam = new(Thefirstpt, Thelastpt);

        if (!mycut)
        {
            if (myfirstParam is null)
            {
                Parameters(Line, Thefirstpt, Thelastpt, TheParam);
            }
            else
            {
                for (i = myfirstParam.Lower(); i <= myfirstParam.Upper(); i++)
                {
                    TheParam[i + Thefirstpt - 1] = myfirstParam.Value(i);
                }
            }
            TheMultiCurve = new AppParCurves_MultiCurve();
            MultiLine anOtherLine0 = default!;
            bool isOtherLine0Made = false;
            int indbad = 0;
            alldone = Compute(Line, myfirstpt, mylastpt, TheParam, out thetol3d, out thetol2d, out indbad);
            if (indbad != 0)
            {
                isOtherLine0Made = LineTool.MakeMLOneMorePoint(Line, myfirstpt, mylastpt, indbad, out anOtherLine0);
            }
            if (isOtherLine0Made)
            {
                myIsClear = true;
                //++myMultiLineNb;
                Perform(anOtherLine0);
                alldone = true;
            }
            if (!alldone && TheMultiCurve.NbCurves() > 0)
            {
                myMultiCurves.Append(new AppParCurves_MultiCurve(TheMultiCurve));
                Tolers3d.Append(currenttol3d);
                Tolers2d.Append(currenttol2d);
                int mylen = mylastpt - myfirstpt + 1;
                int myParLen = myParameters!.Length();
                int aLen = (myParLen > mylen) ? myParLen : mylen;
                NCollection_HArray1<double> ThePar = new(myfirstpt, myfirstpt + aLen - 1);
                for (i = 0; i < aLen; i++)
                {
                    ThePar.SetValue(myfirstpt + i, myParameters.Value(myParameters.Lower() + i));
                }
                myPar.Append(ThePar);
            }
        }
        else
        {
            while (!Finish)
            {
                oldlastpt = mylastpt;
                // Management of the multiline splitting for approximation:
                if (!begin)
                {
                    if (!GoUp)
                    {
                        if (Ok)
                        {
                            // Calcul de la partie a approximer.
                            myfirstpt = mylastpt;
                            mylastpt = Thelastpt;
                            if (myfirstpt == Thelastpt)
                            {
                                Finish = true;
                                alldone = true;
                                return;
                            }
                        }
                        else
                        {
                            nbp = mylastpt - myfirstpt + 1;
                            MyStatus = LineTool.WhatStatus(Line, myfirstpt, mylastpt);
                            if (MyStatus == Approx_NoPointsAdded && nbp <= mydegremax + 1)
                            {
                                Interpol = ComputeCurve(Line, myfirstpt, mylastpt);
                                if (Interpol)
                                {
                                    if (mylastpt == Thelastpt)
                                    {
                                        Finish = true;
                                        alldone = true;
                                        return;
                                    }
                                }
                            }
                            mylastpt = (int)((myfirstpt + mylastpt) / 2);
                        }
                    }
                    GoUp = false;
                }

                // Verification du nombre de points restants par rapport au degre
                // demande.
                // ==============================================================
                nbp = mylastpt - myfirstpt + 1;
                MyStatus = LineTool.WhatStatus(Line, myfirstpt, mylastpt);
                if (nbp <= mydegremax + 5)
                {
                    // Rajout necessaire de points si possible.
                    // ========================================
                    GoUp = false;
                    Ok = true;
                    if (MyStatus == Approx_PointsAdded)
                    {
                        // Appel recursif du decoupage:
                        GoUp = true;

                        MultiLine anOtherLine1 = LineTool.MakeMLBetween(Line, myfirstpt, mylastpt, nbp - 1);

                        int nbpdsotherligne = LineTool.FirstPoint(anOtherLine1) - LineTool.LastPoint(anOtherLine1);

                        //-- If MakeML failed, return an empty line
                        if ((nbpdsotherligne == 0) || myMultiLineNb >= 3)
                        {
                            //-- cout<<" ** ApproxComputeLine MakeML Echec ** LBR lbr "<<endl;
                            if (myfirstpt == mylastpt)
                            {
                                break; // Pour etre sur de ne pas
                            }
                            // planter la station !!
                            myCouple1.SetIndex(myfirstpt);
                            myCouple2.SetIndex(mylastpt);
                            myConstraints.SetValue(1, myCouple1);
                            myConstraints.SetValue(2, myCouple2);

                            math_Vector Param = new(myfirstpt, mylastpt);
                            Approx_ParametrizationType SavePar = Par;
                            Par = Approx_IsoParametric;
                            Parameters(Line, myfirstpt, mylastpt, Param);
                            TheMultiCurve = new AppParCurves_MultiCurve();
                            MultiLine anOtherLine2 = default!;
                            bool isOtherLine2Made = false;
                            int indbad = 0;
                            Ok = Compute(Line, myfirstpt, mylastpt, Param, out thetol3d, out thetol2d, out indbad);
                            if (indbad != 0)
                            {
                                isOtherLine2Made = LineTool.MakeMLOneMorePoint(Line, myfirstpt, mylastpt, indbad, out anOtherLine2);
                            }
                            if (isOtherLine2Made)
                            {
                                myIsClear = true;
                                //++myMultiLineNb;
                                Par = SavePar;
                                Perform(anOtherLine2);
                                Ok = true;
                            }

                            if (!Ok)
                            {
                                double tt3d = currenttol3d, tt2d = currenttol2d;
                                NCollection_HArray1<double>? saveParameters = myParameters;
                                AppParCurves_MultiCurve saveMultiCurve = TheMultiCurve;

                                if (SavePar != Approx_IsoParametric)
                                {
                                    Par = SavePar;
                                }
                                else
                                {
                                    Par = Approx_ChordLength;
                                }

                                Parameters(Line, myfirstpt, mylastpt, Param);
                                isOtherLine2Made = false;
                                indbad = 0;
                                Ok = Compute(Line, myfirstpt, mylastpt, Param, out thetol3d, out thetol2d, out indbad);
                                if (indbad != 0)
                                {
                                    isOtherLine2Made = LineTool.MakeMLOneMorePoint(Line, myfirstpt, mylastpt, indbad,
                                                                                   out anOtherLine2);
                                }
                                if (isOtherLine2Made)
                                {
                                    myIsClear = true;
                                    //++myMultiLineNb;
                                    Perform(anOtherLine2);
                                    Ok = true;
                                }

                                if (!Ok && tt3d <= currenttol3d && tt2d <= currenttol2d)
                                {
                                    currenttol3d = tt3d;
                                    currenttol2d = tt2d;
                                    myParameters = saveParameters;
                                    TheMultiCurve = saveMultiCurve;
                                }
                            }
                            Par = SavePar;
                            if (myfirstpt == Thelastpt)
                            {
                                Finish = true;
                                alldone = true;
                                return;
                            }

                            oldlastpt = mylastpt;
                            if (!Ok)
                            {
                                tolreached = false;
                                if (TheMultiCurve.NbCurves() == 0)
                                {
                                    myMultiCurves.Clear();
                                    return;
                                }
                                MultiLine anOtherLine3 = default!;
                                bool isOtherLine3Made = false;
                                int indbad2 = 0;
                                if (!CheckMultiCurve(TheMultiCurve, Line, myfirstpt, mylastpt, out indbad2))
                                {
                                    isOtherLine3Made = LineTool.MakeMLOneMorePoint(Line, myfirstpt, mylastpt, indbad2,
                                                                                   out anOtherLine3);
                                }
                                if (isOtherLine3Made)
                                {
                                    myIsClear = true;
                                    //++myMultiLineNb;
                                    Perform(anOtherLine3);
                                    myfirstpt = mylastpt;
                                    mylastpt = Thelastpt;
                                }
                                else
                                {
                                    myMultiCurves.Append(new AppParCurves_MultiCurve(TheMultiCurve));
                                    Tolers3d.Append(currenttol3d);
                                    Tolers2d.Append(currenttol2d);
                                    int mylen = oldlastpt - myfirstpt + 1;
                                    int myParLen = myParameters!.Length();
                                    int aLen = (myParLen > mylen) ? myParLen : mylen;
                                    NCollection_HArray1<double> ThePar = new(myfirstpt, myfirstpt + aLen - 1);
                                    for (i = 0; i < aLen; i++)
                                    {
                                        ThePar.SetValue(myfirstpt + i, myParameters.Value(myParameters.Lower() + i));
                                    }
                                    myPar.Append(ThePar);
                                }
                            }
                            myfirstpt = oldlastpt;
                            mylastpt = Thelastpt;
                        }
                        else
                        {
                            myIsClear = true;
                            ++myMultiLineNb;
                            Perform(anOtherLine1);
                            myfirstpt = mylastpt;
                            mylastpt = Thelastpt;
                        }
                    }

                    if (MyStatus == Approx_NoPointsAdded && !begin)
                    {
                        // On rend la meilleure approximation obtenue precedemment.
                        // ========================================================
                        GoUp = true;
                        tolreached = false;
                        if (TheMultiCurve.NbCurves() == 0)
                        {
                            myMultiCurves.Clear();
                            return;
                        }
                        myMultiCurves.Append(new AppParCurves_MultiCurve(TheMultiCurve));
                        Tolers3d.Append(currenttol3d);
                        Tolers2d.Append(currenttol2d);
                        int mylen = oldlastpt - myfirstpt + 1;
                        int myParLen = myParameters!.Length();
                        int aLen = (myParLen > mylen) ? myParLen : mylen;
                        NCollection_HArray1<double> ThePar = new(myfirstpt, myfirstpt + aLen - 1);
                        for (i = 0; i < aLen; i++)
                        {
                            ThePar.SetValue(myfirstpt + i, myParameters.Value(myParameters.Lower() + i));
                        }
                        myPar.Append(ThePar);

                        myfirstpt = oldlastpt;
                        mylastpt = Thelastpt;
                    }
                    else if (MyStatus == Approx_NoApproximation)
                    {
                        // No approximation is done between myfirstpt and mylastpt.
                        // ===========================================================
                        // Store to be able to inform the user.
                        GoUp = true;
                        myfirstpt = mylastpt;
                        mylastpt = Thelastpt;
                    }
                }

                if (myfirstpt == Thelastpt)
                {
                    Finish = true;
                    alldone = true;
                    return;
                }
                if (!GoUp)
                {
                    if (myfirstpt == mylastpt)
                    {
                        break; // Pour etre sur de ne pas
                    }
                    // planter la station !!
                    myCouple1.SetIndex(myfirstpt);
                    myCouple2.SetIndex(mylastpt);
                    myConstraints.SetValue(1, myCouple1);
                    myConstraints.SetValue(2, myCouple2);

                    // Calculation of parameters on this new interval.
                    // Retrieve the initial parameters during splitting.

                    math_Vector Param = new(myfirstpt, mylastpt);
                    if (begin)
                    {
                        if (myfirstParam is null)
                        {
                            Parameters(Line, myfirstpt, mylastpt, Param);
                        }
                        else
                        {
                            for (i = myfirstParam.Lower(); i <= myfirstParam.Upper(); i++)
                            {
                                Param[i] = myfirstParam.Value(i);
                            }
                            myfirstParam = null;
                        }
                        TheParam.Initialized(Param);
                        begin = false;
                    }
                    else
                    {
                        double pfirst = TheParam.Value(myfirstpt);
                        double plast = TheParam.Value(mylastpt);
                        for (i = myfirstpt; i <= mylastpt; i++)
                        {
                            Param[i] = (TheParam.Value(i) - pfirst) / (plast - pfirst);
                        }
                    }

                    TheMultiCurve = new AppParCurves_MultiCurve();
                    int indbad = 0;
                    Ok = Compute(Line, myfirstpt, mylastpt, Param, out thetol3d, out thetol2d, out indbad);
                    if (myfirstpt == Thelastpt)
                    {
                        Finish = true;
                        alldone = true;
                        return;
                    }
                }
            }
        }
    }

    /// <summary>
    /// The parameters of the points of the MultiCurve <paramref name="Index"/>: the object's own array (shared:
    /// don't modify).
    /// </summary>
    public NCollection_Array1<double> Parameters(int Index = 1) => (myPar.Value(Index)).Array1();

    /// <summary>The number of MultiCurves of the approximation.</summary>
    public int NbMultiCurves() => myMultiCurves.Length();

    /// <summary>
    /// The MultiCurve <paramref name="Index"/> of the approximation: the stored one, which a caller may modify in
    /// place, as through OCCT's non-const reference.
    /// </summary>
    public AppParCurves_MultiCurve ChangeValue(int Index = 1) => myMultiCurves.Value(Index);

    /// <summary>The MultiCurve <paramref name="Index"/> of the approximation: the stored one (shared: don't modify).</summary>
    public AppParCurves_MultiCurve Value(int Index = 1) => myMultiCurves.Value(Index);

    /// <summary>
    /// The MultiCurves joined into one MultiBSpCurve, computed on each call; the object's own result (shared:
    /// don't modify).
    /// </summary>
    public AppParCurves_MultiBSpCurve SplineValue()
    {
        Approx_MCurvesToBSpCurve Trans = new();
        Trans.Perform(myMultiCurves);
        myspline = new AppParCurves_MultiBSpCurve(Trans.Value());
        return myspline;
    }

    // Computes new parameters between firstP and lastP.
    private void Parameters(MultiLine Line, int firstP, int lastP, math_Vector TheParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;

        if (Par == Approx_ChordLength || Par == Approx_Centripetal)
        {
            nbP3d = LineTool.NbP3d(Line);
            nbP2d = LineTool.NbP2d(Line);
            int mynbP3d = nbP3d, mynbP2d = nbP2d;
            if (nbP3d == 0)
            {
                mynbP3d = 1;
            }
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }

            TheParameters[firstP] = 0.0;
            NCollection_Array1<gp_Pnt> tabP = new(1, mynbP3d);
            NCollection_Array1<gp_Pnt> tabPP = new(1, mynbP3d);
            NCollection_Array1<gp_Pnt2d> tabP2d = new(1, mynbP2d);
            NCollection_Array1<gp_Pnt2d> tabPP2d = new(1, mynbP2d);

            for (i = firstP + 1; i <= lastP; i++)
            {
                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP);
                }

                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(Line, i, tabPP, tabPP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(Line, i, tabPP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(Line, i, tabPP);
                }
                dist = 0;
                for (j = 1; j <= nbP3d; j++)
                {
                    gp_Pnt aP1 = tabP[j], aP2 = tabPP[j];
                    dist += aP2.SquareDistance(aP1);
                }
                for (j = 1; j <= nbP2d; j++)
                {
                    gp_Pnt2d aP12d = tabP2d[j], aP22d = tabPP2d[j];

                    dist += aP22d.SquareDistance(aP12d);
                }

                dist = Math.Sqrt(dist);
                if (Par == Approx_ChordLength)
                {
                    TheParameters[i] = TheParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    TheParameters[i] = TheParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP; i <= lastP; i++)
            {
                TheParameters[i] /= TheParameters[lastP];
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                TheParameters[i] = ((double)i - firstP) / ((double)lastP - (double)firstP);
            }
        }
    }

    private bool Compute(MultiLine Line, int fpt, int lpt, math_Vector Para, out double TheTol3d, out double TheTol2d,
                         out int indbad)
    {
        indbad = 0;
        // OCCT leaves the tolerances unchanged when no degree is tried; the port's out parameters start at 0.
        TheTol3d = 0.0;
        TheTol2d = 0.0;
        int deg, i;
        bool mydone;
        double Fv;
        int nbp = lpt - fpt + 1;

        math_Vector ParSav = new(Para.Lower(), Para.Upper());
        for (i = Para.Lower(); i <= Para.Upper(); i++)
        {
            ParSav[i] = Para[i];
        }
        int Mdegmax = mydegremax;
        if (nbp < Mdegmax + 5 && mycut)
        {
            Mdegmax = nbp - 5;
        }
        if (Mdegmax < mydegremin)
        {
            Mdegmax = mydegremin;
        }

        currenttol3d = currenttol2d = RealLast();
        for (deg = Min(nbp - 1, mydegremin); deg <= Mdegmax; deg++)
        {
            AppParCurves_MultiCurve mySCU;
            if (mysquares)
            {
                AppParCurves_LeastSquare<MultiLine, LineTool> SQ = new(Line, fpt, lpt, myfirstC, mylastC, Para, deg + 1);
                mydone = SQ.IsDone();
                mySCU = SQ.BezierValue();
                SQ.Error(out Fv, out TheTol3d, out TheTol2d);
            }
            else
            {
                AppParCurves_Gradient<MultiLine, LineTool> GRAD =
                    new(Line, fpt, lpt, myConstraints, Para, deg, mytol3d, mytol2d, myitermax);
                mydone = GRAD.IsDone();
                mySCU = GRAD.Value();
                if (mySCU.NbCurves() == 0)
                {
                    continue;
                }
                TheTol3d = GRAD.MaxError3d();
                TheTol2d = GRAD.MaxError2d();
            }
            double uu1 = Para[Para.Lower()], uu2;
            bool restau = false;
            for (i = Para.Lower() + 1; i <= Para.Upper(); i++)
            {
                uu2 = Para[i];
                if (uu2 <= uu1)
                {
                    restau = true;
                    break;
                }
                uu1 = uu2;
            }
            if (restau)
            {
                for (i = Para.Lower(); i <= Para.Upper(); i++)
                {
                    Para[i] = ParSav[i];
                }
            }
            if (mydone)
            {
                if (TheTol3d <= mytol3d && TheTol2d <= mytol2d)
                {
                    // Stockage de la multicurve approximee.
                    tolreached = true;
                    if (!CheckMultiCurve(mySCU, Line, fpt, lpt, out indbad))
                    {
                        return false;
                    }
                    else
                    {
                        myMultiCurves.Append(mySCU);
                        // Storage of parameters of the approximated MultiLine part:
                        // A ameliorer !! (bq trop de recopies)
                        NCollection_HArray1<double> ThePar = new(Para.Lower(), Para.Upper());
                        for (i = Para.Lower(); i <= Para.Upper(); i++)
                        {
                            ThePar.SetValue(i, Para[i]);
                        }
                        myPar.Append(ThePar);
                        Tolers3d.Append(TheTol3d);
                        Tolers2d.Append(TheTol2d);
                        return true;
                    }
                }
            }

            if (TheTol3d <= currenttol3d && TheTol2d <= currenttol2d)
            {
                TheMultiCurve = mySCU;
                currenttol3d = TheTol3d;
                currenttol2d = TheTol2d;
                myParameters = new NCollection_HArray1<double>(Para.Lower(), Para.Upper());
                for (i = Para.Lower(); i <= Para.Upper(); i++)
                {
                    myParameters.SetValue(i, Para[i]);
                }
            }
        }

        return false;
    }

    private bool ComputeCurve(MultiLine Line, int firstpt, int lastpt)
    {
        int i, j, nbP3d, nbP2d, deg;
        gp_Vec V13d, V23d;
        gp_Vec2d V12d, V22d;
        gp_Pnt P1, P2;
        gp_Pnt2d P12d, P22d;
        bool Tangent1, Tangent2, mydone = false;
        int myfirstpt = firstpt, mylastpt = lastpt;
        int nbp = lastpt - firstpt + 1;
        math_Vector Para = new(firstpt, lastpt);

        Parameters(Line, firstpt, lastpt, Para);

        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }

        NCollection_Array1<gp_Vec> tabV1 = new(1, mynbP3d), tabV2 = new(1, mynbP3d);
        NCollection_Array1<gp_Pnt> tabP1 = new(1, mynbP3d), tabP2 = new(1, mynbP3d), tabP = new(1, mynbP3d);
        NCollection_Array1<gp_Vec2d> tabV12d = new(1, mynbP2d), tabV22d = new(1, mynbP2d);
        NCollection_Array1<gp_Pnt2d> tabP12d = new(1, mynbP2d), tabP22d = new(1, mynbP2d), tabP2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, myfirstpt, tabP1, tabP12d);
            LineTool.Value(Line, mylastpt, tabP2, tabP22d);
            Tangent1 = LineTool.Tangency(Line, myfirstpt, tabV1, tabV12d);
            Tangent2 = LineTool.Tangency(Line, mylastpt, tabV2, tabV22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, myfirstpt, tabP12d);
            LineTool.Value(Line, mylastpt, tabP22d);
            Tangent1 = LineTool.Tangency(Line, myfirstpt, tabV12d);
            Tangent2 = LineTool.Tangency(Line, mylastpt, tabV22d);
        }
        else
        {
            LineTool.Value(Line, myfirstpt, tabP1);
            LineTool.Value(Line, mylastpt, tabP2);
            Tangent1 = LineTool.Tangency(Line, myfirstpt, tabV1);
            Tangent2 = LineTool.Tangency(Line, mylastpt, tabV2);
        }
        if (nbp == 2)
        {
            // If there are only 2 points, we still verify that the tangents are
            // aligned (OCCT only reports it in debug builds).
            if (Tangent1)
            {
                for (i = 1; i <= nbP3d; i++)
                {
                    gp_Vec PVec = new(tabP1[i], tabP2[i]);
                    V13d = tabV1[i];
                    if (!PVec.IsParallel(V13d, Precision.Angular()))
                    {
                        break;
                    }
                }
                for (i = 1; i <= nbP2d; i++)
                {
                    gp_Vec2d PVec2d = new(tabP12d[i], tabP22d[i]);
                    V12d = tabV12d[i];
                    if (!PVec2d.IsParallel(V12d, Precision.Angular()))
                    {
                        break;
                    }
                }
            }

            if (Tangent2)
            {
                for (i = 1; i <= nbP3d; i++)
                {
                    gp_Vec PVec = new(tabP1[i], tabP2[i]);
                    V23d = tabV2[i];
                    if (!PVec.IsParallel(V23d, Precision.Angular()))
                    {
                        break;
                    }
                }
                for (i = 1; i <= nbP2d; i++)
                {
                    gp_Vec2d PVec2d = new(tabP12d[i], tabP22d[i]);
                    V22d = tabV22d[i];
                    if (!PVec2d.IsParallel(V22d, Precision.Angular()))
                    {
                        break;
                    }
                }
            }

            AppParCurves_MultiCurve mySCU2 = new(mydegremin + 1);
            if (nbP3d != 0 && nbP2d != 0)
            {
                AppParCurves_MultiPoint MPole1 = new(tabP1, tabP12d);
                AppParCurves_MultiPoint MPole2 = new(tabP2, tabP22d);
                mySCU2.SetValue(1, MPole1);
                mySCU2.SetValue(mydegremin + 1, MPole2);
                for (i = 2; i <= mydegremin; i++)
                {
                    for (j = 1; j <= nbP3d; j++)
                    {
                        P1 = tabP1[j];
                        P2 = tabP2[j];
                        tabP[j].SetXYZ(P1.XYZ() + (i - 1) * (P2.XYZ() - P1.XYZ()) / mydegremin);
                    }
                    for (j = 1; j <= nbP2d; j++)
                    {
                        P12d = tabP12d[j];
                        P22d = tabP22d[j];
                        tabP2d[j].SetXY(P12d.XY() + (i - 1) * (P22d.XY() - P12d.XY()) / mydegremin);
                    }
                    AppParCurves_MultiPoint MPole = new(tabP, tabP2d);
                    mySCU2.SetValue(i, MPole);
                }
            }
            else if (nbP3d != 0)
            {
                AppParCurves_MultiPoint MPole1 = new(tabP1);
                AppParCurves_MultiPoint MPole2 = new(tabP2);
                mySCU2.SetValue(1, MPole1);
                mySCU2.SetValue(mydegremin + 1, MPole2);
                for (i = 2; i <= mydegremin; i++)
                {
                    for (j = 1; j <= nbP3d; j++)
                    {
                        P1 = tabP1[j];
                        P2 = tabP2[j];
                        tabP[j].SetXYZ(P1.XYZ() + (i - 1) * (P2.XYZ() - P1.XYZ()) / mydegremin);
                    }
                    AppParCurves_MultiPoint MPole = new(tabP);
                    mySCU2.SetValue(i, MPole);
                }
            }
            else if (nbP2d != 0)
            {
                AppParCurves_MultiPoint MPole1 = new(tabP12d);
                AppParCurves_MultiPoint MPole2 = new(tabP22d);
                mySCU2.SetValue(1, MPole1);
                mySCU2.SetValue(mydegremin + 1, MPole2);
                for (i = 2; i <= mydegremin; i++)
                {
                    for (j = 1; j <= nbP2d; j++)
                    {
                        P12d = tabP12d[j];
                        P22d = tabP22d[j];
                        tabP2d[j].SetXY(P12d.XY() + (i - 1) * (P22d.XY() - P12d.XY()) / mydegremin);
                    }
                    AppParCurves_MultiPoint MPole = new(tabP2d);
                    mySCU2.SetValue(i, MPole);
                }
            }
            mydone = true;
            // Stockage de la multicurve approximee.
            tolreached = true;
            myMultiCurves.Append(mySCU2);
            NCollection_HArray1<double> ThePar2 = new(Para.Lower(), Para.Upper());
            for (i = Para.Lower(); i <= Para.Upper(); i++)
            {
                ThePar2.SetValue(i, Para[i]);
            }
            myPar.Append(ThePar2);
            Tolers3d.Append(Precision.Confusion());
            Tolers2d.Append(Precision.PConfusion());
            return mydone;
        }

        // with the tangents.
        deg = nbp + 1;
        AppParCurves_MultiCurve mySCU;
        AppParCurves_Constraint Cons = AppParCurves_TangencyPoint;
        double lambda1, lambda2;
        math_Vector V1 = new(1, nbP3d * 3 + nbP2d * 2);
        math_Vector V2 = new(1, nbP3d * 3 + nbP2d * 2);
        FirstTangencyVector(Line, myfirstpt, V1);
        lambda1 = SearchFirstLambda(Line, Para, V1, myfirstpt);

        LastTangencyVector(Line, mylastpt, V2);
        lambda2 = SearchLastLambda(Line, Para, V2, mylastpt);

        AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, myfirstpt, mylastpt, Cons, Cons, Para, deg + 1);

        lambda1 = lambda1 / deg;
        lambda2 = lambda2 / deg;
        LSQ.Perform(Para, V1, V2, lambda1, lambda2);
        mydone = LSQ.IsDone();
        mySCU = LSQ.BezierValue();

        if (mydone)
        {
            double Fv, TheTol3d, TheTol2d;
            LSQ.Error(out Fv, out TheTol3d, out TheTol2d);

            // Stockage de la multicurve approximee.
            tolreached = true;
            myMultiCurves.Append(mySCU);
            NCollection_HArray1<double> ThePar = new(Para.Lower(), Para.Upper());
            for (i = Para.Lower(); i <= Para.Upper(); i++)
            {
                ThePar.SetValue(i, Para[i]);
            }
            myPar.Append(ThePar);
            Tolers3d.Append(TheTol3d);
            Tolers2d.Append(TheTol2d);
            return true;
        }
        return mydone;
    }

    /// <summary>Initialises the fields of the algorithm.</summary>
    public void Init(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06,
                     int NbIterations = 5, bool cutting = true,
                     Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        Par = parametrization;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
    }

    /// <summary>Changes the degrees of the approximation.</summary>
    public void SetDegrees(int degreemin, int degreemax)
    {
        mydegremin = degreemin;
        mydegremax = degreemax;
    }

    /// <summary>Changes the tolerances of the approximation.</summary>
    public void SetTolerances(double Tolerance3d, double Tolerance2d)
    {
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
    }

    /// <summary>Changes the constraints at the first and the last points.</summary>
    public void SetConstraints(AppParCurves_Constraint firstC, AppParCurves_Constraint lastC)
    {
        myfirstC = firstC;
        mylastC = lastC;
    }

    /// <summary>False if, during the approximation, the line tool returned NoApproximation when more points were needed.</summary>
    public bool IsAllApproximated() => alldone;

    /// <summary>False if the line tool returned NoPointsAdded.</summary>
    public bool IsToleranceReached() => tolreached;

    /// <summary>The 3d and 2d tolerances of the MultiCurve <paramref name="Index"/>.</summary>
    public void Error(int Index, out double tol3d, out double tol2d)
    {
        tol3d = Tolers3d.Value(Index);
        tol2d = Tolers2d.Value(Index);
    }

    /// <summary>The type of parametrisation.</summary>
    public Approx_ParametrizationType Parametrization() => Par;
}
