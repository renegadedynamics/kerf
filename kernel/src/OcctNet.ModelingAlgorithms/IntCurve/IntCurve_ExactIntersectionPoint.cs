// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ExactIntersectionPoint.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The exact intersection point of two 2D curves near the crossing of two segments of their polygons: a root of
/// <see cref="IntCurve_DistBetweenPCurvesGen{TheCurve, TheCurveTool}"/> found by math_FunctionSetRoot, with the
/// search bounds widened over the neighbouring segments when none is found.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_ExactIntersectionPoint.gxx, ported once as a generic class (decision 324). An
/// algorithm object (decision 117) keeping the curves, as OCCT keeps pointers to them.
/// </remarks>
public class IntCurve_ExactIntersectionPoint<TheCurve, TheCurveTool>
    where TheCurveTool : IIntCurve_CurveTool<TheCurve>
{
    // OCCT sets done and never reads it.
#pragma warning disable CS0414
    private bool done;
#pragma warning restore CS0414
    private int nbroots;
    private readonly double myTol;
    private readonly IntCurve_DistBetweenPCurvesGen<TheCurve, TheCurveTool> FctDist;
    private readonly math_Vector ToleranceVector;
    private readonly math_Vector BInfVector;
    private readonly math_Vector BSupVector;
    private readonly math_Vector StartingPoint;
    private readonly math_Vector Root;
    private bool anErrorOccurred;

    public IntCurve_ExactIntersectionPoint(in TheCurve C1, in TheCurve C2, double Tol)
    {
        done = false;
        nbroots = 0;
        myTol = Tol * Tol;
        FctDist = new IntCurve_DistBetweenPCurvesGen<TheCurve, TheCurveTool>(C1, C2);
        ToleranceVector = new math_Vector(1, 2);
        BInfVector = new math_Vector(1, 2);
        BSupVector = new math_Vector(1, 2);
        StartingPoint = new math_Vector(1, 2);
        Root = new math_Vector(1, 2);
        anErrorOccurred = false;
        ToleranceVector.Value(1) = TheCurveTool.EpsX(C1);
        ToleranceVector.Value(2) = TheCurveTool.EpsX(C2);
    }

    /// <summary>
    /// Searches the root near the point <paramref name="ParamOnSeg1"/> of the segment <paramref name="NumSegOn1"/> of
    /// <paramref name="Poly1"/> and the point <paramref name="ParamOnSeg2"/> of the segment
    /// <paramref name="NumSegOn2"/> of <paramref name="Poly2"/>; the segments and parameters are moved inside the
    /// polygons when they are at an end.
    /// </summary>
    public void Perform(IntCurve_Polygon2dGen<TheCurve, TheCurveTool> Poly1,
                        IntCurve_Polygon2dGen<TheCurve, TheCurveTool> Poly2,
                        ref int NumSegOn1,
                        ref int NumSegOn2,
                        ref double ParamOnSeg1,
                        ref double ParamOnSeg2)
    {
        //----------------------------------------------------------------------
        //-- On prend comme bornes de recherches  :
        //--
        //--   Segment      :      i-1        i           i+1        i+2
        //--
        //--                  |---------|-----X-------|---------|----------|
        //--                Inf                                Sup
        //--
        if (NumSegOn1 >= Poly1.NbSegments() && ParamOnSeg1 == 0.0)
        {
            NumSegOn1--;
            ParamOnSeg1 = 1.0;
        }
        if (NumSegOn2 >= Poly2.NbSegments() && ParamOnSeg2 == 0.0)
        {
            NumSegOn2--;
            ParamOnSeg2 = 1.0;
        }
        if (NumSegOn1 <= 0)
        {
            NumSegOn1 = 1;
            ParamOnSeg1 = 0.0;
        }
        if (NumSegOn2 <= 0)
        {
            NumSegOn2 = 1;
            ParamOnSeg2 = 0.0;
        }

        StartingPoint.Value(1) = Poly1.ApproxParamOnCurve(NumSegOn1, ParamOnSeg1);
        if (NumSegOn1 <= 2)
        {
            BInfVector.Value(1) = Poly1.InfParameter();
        }
        else
        {
            BInfVector.Value(1) = Poly1.ApproxParamOnCurve(NumSegOn1 - 1, (double)0.0);
        }
        if (NumSegOn1 >= (Poly1.NbSegments() - 2))
        {
            BSupVector.Value(1) = Poly1.SupParameter();
        }
        else
        {
            BSupVector.Value(1) = Poly1.ApproxParamOnCurve(NumSegOn1 + 2, (double)0.0);
        }

        StartingPoint.Value(2) = Poly2.ApproxParamOnCurve(NumSegOn2, ParamOnSeg2);
        if (NumSegOn2 <= 2)
        {
            BInfVector.Value(2) = Poly2.InfParameter();
        }
        else
        {
            BInfVector.Value(2) = Poly2.ApproxParamOnCurve(NumSegOn2 - 1, (double)0.0);
        }
        if (NumSegOn2 >= (Poly2.NbSegments() - 2))
        {
            BSupVector.Value(2) = Poly2.SupParameter();
        }
        else
        {
            BSupVector.Value(2) = Poly2.ApproxParamOnCurve(NumSegOn2 + 2, (double)0.0);
        }

        MathPerform();
        if (nbroots == 0)
        {
            // OCCT reads both deflections and discards them.
            Poly1.DeflectionOverEstimation();
            Poly2.DeflectionOverEstimation();
            {
                //-- On risque de donner des bornes sur la courbe 1 trop etroites.
                int diff = 1;
                double AnBinfVector = BInfVector.Value(1);
                double AnBsupVector = BSupVector.Value(1);
                //---------------- On elargit les bornes par la gauche --------------------
                do
                {
                    diff++;
                    if ((NumSegOn1 - diff) <= 1)
                    {
                        BInfVector.Value(1) = Poly1.InfParameter();
                        diff = 0;
                    }
                    else
                    {
                        BInfVector.Value(1) = Poly1.ApproxParamOnCurve(NumSegOn1 - diff, (double)0.0);
                    }
                    MathPerform();
                    //-- le 18 nov 97
                    if (diff > 3)
                    {
                        diff += NumSegOn1 / 2;
                    }
                } while (nbroots == 0 && diff != 0);
                //---------------- On elargit les bornes par la droite --------------------
                if (nbroots == 0)
                {
                    BInfVector.Value(1) = AnBinfVector;
                    diff = 1;
                    do
                    {
                        diff++;
                        if ((NumSegOn1 + diff) >= (Poly1.NbSegments() - 1))
                        {
                            BSupVector.Value(1) = Poly1.SupParameter();
                            diff = 0;
                        }
                        else
                        {
                            BSupVector.Value(1) = Poly1.ApproxParamOnCurve(NumSegOn1 + 1 + diff, (double)0.0);
                        }
                        MathPerform();
                        //-- le 18 nov 97
                        if (diff > 3)
                        {
                            diff += 1 + (Poly1.NbSegments() - NumSegOn1) / 2;
                        }
                    } while (nbroots == 0 && diff != 0);
                }
                BSupVector.Value(1) = AnBsupVector;
            }

            if (nbroots == 0)
            {
                //-- On risque de donner des bornes sur la courbe 1 trop etroites.
                int diff = 1;
                double AnBinfVector = BInfVector.Value(2);
                double AnBsupVector = BSupVector.Value(2);
                //---------------- On elargit les bornes par la gauche --------------------
                do
                {
                    diff++;
                    if ((NumSegOn2 - diff) <= 1)
                    {
                        BInfVector.Value(2) = Poly2.InfParameter();
                        diff = 0;
                    }
                    else
                    {
                        BInfVector.Value(2) = Poly2.ApproxParamOnCurve(NumSegOn2 - diff, (double)0.0);
                    }
                    MathPerform();
                    //-- le 18 nov 97
                    if (diff > 3)
                    {
                        diff += NumSegOn2 / 2;
                    }
                } while (nbroots == 0 && diff != 0);
                //---------------- On elargit les bornes par la droite --------------------
                if (nbroots == 0)
                {
                    BInfVector.Value(2) = AnBinfVector;
                    diff = 1;
                    do
                    {
                        diff++;
                        if ((NumSegOn2 + diff) >= (Poly2.NbSegments() - 1))
                        {
                            BSupVector.Value(2) = Poly2.SupParameter();
                            diff = 0;
                        }
                        else
                        {
                            BSupVector.Value(2) = Poly2.ApproxParamOnCurve(NumSegOn2 + 1 + diff, (double)0.0);
                        }
                        MathPerform();
                        //-- le 18 nov 97
                        if (diff > 3)
                        {
                            diff += 1 + (Poly2.NbSegments() - NumSegOn2) / 2;
                        }
                    } while (nbroots == 0 && diff != 0);
                }
                BSupVector.Value(2) = AnBsupVector;
            }
        }
    }

    /// <summary>Searches the root from (Uo, Vo) inside the given bounds.</summary>
    public void Perform(double Uo, double Vo, double UInf, double VInf, double USup, double VSup)
    {
        done = true;

        BInfVector.Value(1) = UInf;
        BInfVector.Value(2) = VInf;
        BSupVector.Value(1) = USup;
        BSupVector.Value(2) = VSup;
        StartingPoint.Value(1) = Uo;
        StartingPoint.Value(2) = Vo;

        MathPerform();
    }

    /// <summary>The number of roots found (0 or 1).</summary>
    public int NbRoots() => (nbroots);

    /// <summary>The parameters of the root on both curves.</summary>
    public void Roots(out double U, out double V)
    {
        U = Root.Value(1);
        V = Root.Value(2);
    }

    private void MathPerform()
    {
        FunctionSetRoot Fct = new(FctDist, ToleranceVector, 60);
        Fct.Perform(FctDist, StartingPoint, BInfVector, BSupVector);

        if (Fct.IsDone)
        {
            Fct.Root(Root);
            nbroots = 1;
            math_Vector XY = new(1, 2);
            FctDist.Value(Root, XY);
            double dist2 = ((XY[1] * XY[1] + XY[2] * XY[2]));

            if (dist2 > myTol)
            {
                nbroots = 0;
            }
        }
        else
        {
            anErrorOccurred = true;
            nbroots = 0;
        }
    }

    /// <summary>True if the root search failed (not just found no root) at least once.</summary>
    public bool AnErrorOccurred() => anErrorOccurred;
}
