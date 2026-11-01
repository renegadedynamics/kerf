// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2dContinuity.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2dContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Corrects or splits a 2D curve with a continuity criterion: at each interior knot of a B-spline below the criterion
/// the knot is removed within the tolerance if possible (the tool's copy of the curve is changed), else it becomes a
/// splitting value. Trimmed and offset curves are processed through their basis.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_SplitCurve2dContinuity : ShapeUpgrade_SplitCurve2d
{
    private GeomAbs_Shape myCriterion;
    private int myCont;
    private double myTolerance;

    /// <summary>Empty constructor (criterion C1, tolerance PConfusion).</summary>
    public ShapeUpgrade_SplitCurve2dContinuity()
    {
        myCriterion = GeomAbs_C1;
        myTolerance = Occt.Precision.PConfusion();
        myCont = 1;
    }

    /// <summary>Sets the criterion of continuity (C0, C1, C2, C3 or CN; G1 and G2 count as C1).</summary>
    public void SetCriterion(GeomAbs_Shape Criterion)
    {
        myCriterion = Criterion;
        switch (myCriterion)
        {
            case GeomAbs_C0:
                myCont = 0;
                break;
            case GeomAbs_C1:
                myCont = 1;
                break;
            case GeomAbs_C2:
                myCont = 2;
                break;
            case GeomAbs_C3:
                myCont = 3;
                break;
            case GeomAbs_CN:
                myCont = 4;
                break;
            default:
                myCont = 1;
                break;
        }
    }

    /// <summary>Sets the tolerance for removing knots.</summary>
    public void SetTolerance(double Tol)
    {
        myTolerance = Tol;
    }

    /// <summary>Computes the splitting values and corrects the curve where it can.</summary>
    public override void Compute()
    {
        if (myCurve!.Continuity() < myCriterion)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        if (mySplitValues!.Length() > 2)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        double precision = Occt.Precision.PConfusion();
        double First = mySplitValues.Value(1);
        double Last = mySplitValues.Value(mySplitValues.Length());
        if (myCurve is Geom2d_TrimmedCurve tmp)
        {
            Geom2d_Curve BasCurve = tmp.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve2dContinuity();
            //    spc.Init(BasCurve,Max(First,tmp->FirstParameter()),Min(Last,tmp->LastParameter()));
            spc.Init(BasCurve, First, Last);
            spc.SetSplitValues(mySplitValues);
            spc.SetTolerance(myTolerance);
            spc.SetCriterion(myCriterion);
            spc.Compute();
            mySplitValues.Clear();
            mySplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            // mySplitValues = spc.SplitValues();
            myStatus |= spc.myStatus;
            return;
        }
        else if (myCurve is Geom2d_OffsetCurve tmpOffset)
        {
            GeomAbs_Shape BasCriterion;
            switch (myCriterion)
            {
                default:
                case GeomAbs_C1:
                    BasCriterion = GeomAbs_C2;
                    break;
                case GeomAbs_C2:
                    BasCriterion = GeomAbs_C3;
                    break;
                case GeomAbs_C3: // OCCT_DEBUG warning left out: criterion C3 for Offset curve
                case GeomAbs_CN:
                    BasCriterion = GeomAbs_CN;
                    break;
            }
            Geom2d_Curve BasCurve = tmpOffset.BasisCurve();
            // double Offset = tmp->Offset(); // Offset not used (skl)
            var spc = new ShapeUpgrade_SplitCurve2dContinuity();
            //    spc.Init(BasCurve,Max(tmp->FirstParameter(),First),Min(tmp->LastParameter(),Last));
            spc.Init(BasCurve, First, Last);
            spc.SetSplitValues(mySplitValues);
            spc.SetTolerance(myTolerance);
            spc.SetCriterion(BasCriterion);
            spc.Compute();
            mySplitValues.Clear();
            mySplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            myStatus |= spc.myStatus;
            return;
        }

        Geom2d_BSplineCurve? MyBSpline = myCurve as Geom2d_BSplineCurve;
        if (MyBSpline is null)
        {
            return;
        }

        myNbCurves = 1;
        int Deg = MyBSpline.Degree();
        int NbKnots = MyBSpline.NbKnots();
        if (NbKnots <= 2)
        {
            return;
        }

        int FirstInd = MyBSpline.FirstUKnotIndex() + 1, LastInd = MyBSpline.LastUKnotIndex() - 1;
        int iknot = FirstInd;
        for (int j = 2; j <= mySplitValues.Length(); j++)
        {
            Last = mySplitValues.Value(j);
            for (; iknot <= LastInd; iknot++)
            {
                double valknot = MyBSpline.Knot(iknot);
                if (valknot <= First + precision)
                {
                    continue;
                }
                if (valknot >= Last - precision)
                {
                    break;
                }
                int Continuity = Deg - MyBSpline.Multiplicity(iknot);
                // double tt = MyBSpline->Knot(iknot); // tt not used (skl)
                if (Continuity < myCont)
                {
                    // At this knot, the curve is C0; try to remove Knot.
                    bool corrected = false;
                    int newMultiplicity = Deg - myCont;
                    if (newMultiplicity < 0)
                    {
                        newMultiplicity = 0;
                    }
                    {
                        try
                        {
                            corrected = MyBSpline.RemoveKnot(iknot, newMultiplicity, myTolerance);
                        }
                        catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                        {
                            corrected = false;
                        }
                    }
                    if (corrected && newMultiplicity > 0)
                    {
                        Continuity = Deg - MyBSpline.Multiplicity(iknot);
                        corrected = (Continuity >= myCont);
                    }
                    if (corrected)
                    {
                        // at this knot, the continuity is now C1. Nothing else to do.
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                    }
                    else
                    {
                        // impossible to force C1 within the tolerance:
                        // this knot will be a splitting value.
                        mySplitValues.InsertBefore(j++, MyBSpline.Knot(iknot));
                        myNbCurves++;
                    }
                }
            }

            First = Last;
        }

        if (mySplitValues.Length() > 2)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
    }
}
