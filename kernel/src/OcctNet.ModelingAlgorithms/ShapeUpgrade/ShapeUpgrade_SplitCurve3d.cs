// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve3d.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Splits a 3D curve with a criterion (the base class splits at the values given). The curve given is copied at
/// <see cref="Init(Geom_Curve, double, double)"/>, so it is never modified.
/// </summary>
/// <remarks>A working object (decision 21). The resulting curves are the tool's own, unfrozen.</remarks>
public class ShapeUpgrade_SplitCurve3d : ShapeUpgrade_SplitCurve
{
    /// <summary>The tool's copy of the curve.</summary>
    protected Geom_Curve? myCurve;
    /// <summary>The resulting curves.</summary>
    protected NCollection_HArray1<Geom_Curve>? myResultingCurves;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_SplitCurve3d()
    {
    }

    /// <summary>Initializes with the curve over its whole range.</summary>
    public void Init(Geom_Curve C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>Initializes with a copy of the curve, bounded by the parameters (clipped to a non-periodic curve's domain).</summary>
    public void Init(Geom_Curve C, double First, double Last)
    {
        Geom_Curve CopyOfC = (Geom_Curve)C.Copy();
        myCurve = CopyOfC;

        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double firstPar = First;
        double lastPar = Last;
        Geom_Curve aCurve = myCurve;
        if (aCurve is Geom_TrimmedCurve aTrimmed)
        {
            aCurve = aTrimmed.BasisCurve();
        }
        // 15.11.2002 PTV OCC966
        if (!ShapeAnalysis_Curve.IsPeriodic(C))
        {
            double fP = aCurve.FirstParameter();
            double lP = aCurve.LastParameter();
            if (Math.Abs(firstPar - fP) < precision)
            {
                firstPar = fP;
            }
            if (Math.Abs(lastPar - lP) < precision)
            {
                lastPar = lP;
            }
            if (firstPar < fP)
            {
                // OCCT_DEBUG output left out: the range of the edge exceeds the curve domain
                firstPar = fP;
            }
            if (lastPar > lP)
            {
                // OCCT_DEBUG output left out
                lastPar = lP;
            }
            if ((lastPar - firstPar) < precision)
            {
                lastPar = firstPar + 2 * precision;
            }
        }

        base.Init(firstPar, lastPar);

        // first, we make a copy of C to prevent modification:
        myNbCurves = 1;
    }

    /// <summary>
    /// Builds the resulting curves: segments of a B-spline or Bezier curve when <paramref name="Segment"/> (trimmed
    /// curves if segmenting fails), trimmed curves otherwise; trimmed and offset curves are split through their basis.
    /// </summary>
    public override void Build(bool Segment)
    {
        double First = mySplitValues!.Value(1);
        double Last = mySplitValues.Value(mySplitValues.Length());
        if (mySplitValues.Length() > 2)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        if (myCurve is Geom_TrimmedCurve tmp)
        {
            Geom_Curve BasCurve = tmp.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve3d();
            spc.Init(BasCurve, First, Last);
            spc.SetSplitValues(mySplitValues);
            spc.Build(Segment);
            myNbCurves = spc.GetCurves()!.Length();

            myResultingCurves = new NCollection_HArray1<Geom_Curve>(1, myNbCurves);
            if (myNbCurves == 1)
            {
                var NewTrimCurve = new Geom_TrimmedCurve(spc.GetCurves()!.Value(1), First, Last);
                myResultingCurves.SetValue(1, NewTrimCurve);
            }
            else
            {
                myResultingCurves = spc.GetCurves();
            }
            myStatus |= spc.myStatus;
            return;
        }
        else if (myCurve is Geom_OffsetCurve tmpOffset)
        {
            Geom_Curve BasCurve = tmpOffset.BasisCurve();
            double Offset = tmpOffset.Offset();
            Direction3D Direct = tmpOffset.Direction();
            var spc = new ShapeUpgrade_SplitCurve3d();
            spc.Init(BasCurve, First, Last);
            spc.SetSplitValues(mySplitValues);
            spc.Build(Segment);
            myNbCurves = spc.GetCurves()!.Length();
            myResultingCurves = new NCollection_HArray1<Geom_Curve>(1, myNbCurves);
            for (int i = 1; i <= myNbCurves; i++)
            {
                var NewOffsetCurve = new Geom_OffsetCurve(spc.GetCurves()!.Value(i), Offset, Direct);
                myResultingCurves.SetValue(i, NewOffsetCurve);
            }
            myStatus |= spc.myStatus;
            return;
        }

        // pdn fix on BuildCurve 3d
        //  15.11.2002 PTV OCC966
        if (!ShapeAnalysis_Curve.IsPeriodic(myCurve!))
        {
            // pdn exceptons only on non periodic curves
            double precision = Kerf.Engine.Foundation.Precision.PConfusion();
            double firstPar = myCurve!.FirstParameter();
            double lastPar = myCurve.LastParameter();
            if (Math.Abs(First - firstPar) < precision)
            {
                First = firstPar;
            }
            if (Math.Abs(Last - lastPar) < precision)
            {
                Last = lastPar;
            }
            if (First < firstPar)
            {
                // OCCT_DEBUG output left out
                First = firstPar;
                mySplitValues.SetValue(1, First);
            }
            if (Last > lastPar)
            {
                // OCCT_DEBUG output left out
                Last = lastPar;
                mySplitValues.SetValue(mySplitValues.Length(), Last);
            }
        }

        myNbCurves = mySplitValues.Length() - 1;
        myResultingCurves = new NCollection_HArray1<Geom_Curve>(1, myNbCurves);
        if (myNbCurves == 1)
        {
            bool filled = true;
            if (Math.Abs(myCurve!.FirstParameter() - First) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(myCurve.LastParameter() - Last) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                myResultingCurves.SetValue(1, myCurve);
            }
            else if (!Segment || (myCurve is not Geom_BSplineCurve && myCurve is not Geom_BezierCurve)
                     || !Status(ShapeExtend_DONE2))
            {
                // OCCT's commented-out segmentation left out
                var theNewCurve = new Geom_TrimmedCurve((Geom_Curve)myCurve.Copy(), First, Last);
                myResultingCurves.SetValue(1, theNewCurve);
            }
            else
            {
                filled = false;
            }
            if (filled)
            {
                return;
            }
        }

        if (myCurve is Geom_BSplineCurve)
        {
            var BsCurve = (Geom_BSplineCurve)myCurve.Copy();
            int FirstInd = BsCurve.FirstUKnotIndex(), LastInd = BsCurve.LastUKnotIndex();
            int j = FirstInd;
            for (int ii = 1; ii <= mySplitValues.Length(); ii++)
            {
                double spval = mySplitValues.Value(ii);
                for (; j <= LastInd; j++)
                {
                    if (spval > BsCurve.Knot(j) + Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < BsCurve.Knot(j) - Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        break;
                    }
                    mySplitValues.SetValue(ii, BsCurve.Knot(j));
                }
                if (j == LastInd)
                {
                    break;
                }
            }
        }

        for (int i = 1; i <= myNbCurves; i++)
        {
            // skl : in the next block I change "First","Last" to "Firstt","Lastt"
            double Firstt = mySplitValues.Value(i), Lastt = mySplitValues.Value(i + 1);
            Geom_Curve? theNewCurve = null;
            if (Segment)
            {
                if (myCurve is Geom_BSplineCurve || myCurve is Geom_BezierCurve)
                {
                    theNewCurve = (Geom_Curve)myCurve!.Copy();
                    try
                    {
                        if (myCurve is Geom_BSplineCurve)
                        {
                            ((Geom_BSplineCurve)theNewCurve).Segment(Firstt, Lastt);
                        }
                        else if (myCurve is Geom_BezierCurve)
                        {
                            ((Geom_BezierCurve)theNewCurve).Segment(Firstt, Lastt);
                        }
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                    }
                    catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                    {
                        // OCCT_DEBUG output left out
                        theNewCurve = new Geom_TrimmedCurve((Geom_Curve)myCurve.Copy(), Firstt, Lastt);
                    }
                }
                else
                {
                    theNewCurve = new Geom_TrimmedCurve((Geom_Curve)myCurve!.Copy(), Firstt, Lastt);
                }
            }
            myResultingCurves.SetValue(i, theNewCurve!);
        }
    }

    /// <summary>Returns the resulting curves (the tool's own).</summary>
    public NCollection_HArray1<Geom_Curve>? GetCurves() => myResultingCurves;
}
