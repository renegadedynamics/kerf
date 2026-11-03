// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceContinuity.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Corrects or splits a surface with a continuity criterion: at each interior knot of a B-spline below the criterion
/// the knot is removed within the tolerance if possible (a copy of the surface), else it becomes a splitting value.
/// Swept surfaces are processed through their basis curve, trimmed and offset surfaces through their basis surface.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT replaces the basis curve of a surface of linear extrusion in place when it
/// removed knots from it; the port does that on a copy of a frozen surface, which then becomes the tool's surface
/// (decision 998).
/// </remarks>
public class ShapeUpgrade_SplitSurfaceContinuity : ShapeUpgrade_SplitSurface
{
    private Continuity myCriterion;
    private double myTolerance;
    private int myCont;

    /// <summary>Empty constructor (criterion C1, tolerance Confusion).</summary>
    public ShapeUpgrade_SplitSurfaceContinuity()
    {
        myCont = 0;
        myCriterion = C1;
        myTolerance = Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Sets the criterion for splitting (C1, C2, C3 or CN; any other counts as C1).</summary>
    public void SetCriterion(Continuity Criterion)
    {
        myCriterion = Criterion;
        switch (myCriterion)
        {
            default:
            case C1:
                myCont = 1;
                break;
            case C2:
                myCont = 2;
                break;
            case C3:
                myCont = 3;
                break;
            case CN:
                myCont = 4;
                break;
        }
    }

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double Tol)
    {
        myTolerance = Tol;
    }

    /// <summary>Computes the splitting values and corrects the surface where it can.</summary>
    public override void Compute(bool Segment)
    {
        if (!Segment)
        {
            mySurface!.Bounds(out double UF, out double UL, out double VF, out double VL);
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(UF))
            {
                myUSplitValues!.SetValue(1, UF);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(UL))
            {
                myUSplitValues!.SetValue(myUSplitValues.Length(), UL);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(VF))
            {
                myVSplitValues!.SetValue(1, VF);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(VL))
            {
                myVSplitValues!.SetValue(myVSplitValues.Length(), VL);
            }
        }

        double UFirst = myUSplitValues!.Value(1);
        double ULast = myUSplitValues.Value(myUSplitValues.Length());
        double VFirst = myVSplitValues!.Value(1);
        double VLast = myVSplitValues.Value(myVSplitValues.Length());
        double precision = Kerf.Engine.Foundation.Precision.Confusion();

        if (mySurface!.Continuity() < myCriterion)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        if (myUSplitValues.Length() > 2 || myVSplitValues.Length() > 2)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (mySurface is Geom_SurfaceOfRevolution SurfaceRev)
        {
            if (SurfaceRev.Continuity() >= myCriterion && myUSplitValues.Length() == 2
                && myVSplitValues.Length() == 2)
            {
                return;
            }
            Geom_Curve BasCurve = SurfaceRev.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve3dContinuity();
            spc.Init(BasCurve, VFirst, VLast);
            spc.SetCriterion(myCriterion);
            spc.SetTolerance(myTolerance);
            spc.SetSplitValues(myVSplitValues);
            spc.Compute();
            myVSplitValues.Clear();
            myVSplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            if (spc.Status(ShapeExtend_DONE1))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
            if (spc.Status(ShapeExtend_DONE2))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            if (spc.Status(ShapeExtend_DONE3))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
            return;
        }
        if (mySurface is Geom_SurfaceOfLinearExtrusion SurfaceExt)
        {
            if (SurfaceExt.Continuity() >= myCriterion && myUSplitValues.Length() == 2
                && myVSplitValues.Length() == 2)
            {
                return;
            }
            Geom_Curve BasCurve = SurfaceExt.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve3dContinuity();
            spc.Init(BasCurve, UFirst, ULast);
            spc.SetCriterion(myCriterion);
            spc.SetTolerance(myTolerance);
            spc.SetSplitValues(myUSplitValues);
            spc.Compute();
            myUSplitValues.Clear();
            myUSplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            if (spc.Status(ShapeExtend_DONE1))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
            if (spc.Status(ShapeExtend_DONE2))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            if (spc.Status(ShapeExtend_DONE3))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                Geom_Curve aNewBascurve = spc.GetCurve()!;
                // OCCT.NET: OCCT sets the basis curve of the surface it was given (the face's own); a frozen surface
                // is copied first and the copy becomes the tool's surface (decision 998)
                if (SurfaceExt.IsFrozen)
                {
                    SurfaceExt = (Geom_SurfaceOfLinearExtrusion)SurfaceExt.Copy();
                    mySurface = SurfaceExt;
                }
                SurfaceExt.SetBasisCurve(aNewBascurve);
            }
            return;
        }

        if (mySurface is Geom_RectangularTrimmedSurface tmpRect)
        {
            if (tmpRect.Continuity() >= myCriterion && myUSplitValues.Length() == 2
                && myVSplitValues.Length() == 2)
            {
                return;
            }

            tmpRect.Bounds(out double U1, out double U2, out double V1, out double V2);
            Geom_Surface theSurf = tmpRect.BasisSurface();
            var sps = new ShapeUpgrade_SplitSurfaceContinuity();
            sps.Init(theSurf,
                     Math.Max(U1, UFirst),
                     Math.Min(U2, ULast),
                     Math.Max(V1, VFirst),
                     Math.Min(V2, VLast));
            sps.SetUSplitValues(myUSplitValues);
            sps.SetVSplitValues(myVSplitValues);
            sps.SetTolerance(myTolerance);
            sps.SetCriterion(myCriterion);
            sps.Compute(true);
            myUSplitValues.Clear();
            myUSplitValues.ChangeSequence().Assign(sps.USplitValues()!.Sequence());
            myVSplitValues.Clear();
            myVSplitValues.ChangeSequence().Assign(sps.VSplitValues()!.Sequence());
            myStatus |= sps.myStatus;
            return;
        }
        else if (mySurface is Geom_OffsetSurface tmpOffset)
        {
            Continuity BasCriterion;
            switch (myCriterion)
            {
                default:
                case C1:
                    BasCriterion = C2;
                    break;
                case C2:
                    BasCriterion = C3;
                    break;
                case C3: // OCCT_DEBUG warning left out: criterion C3 for Offset surface
                case CN:
                    BasCriterion = CN;
                    break;
            }
            Geom_Surface theSurf = tmpOffset.BasisSurface();
            if (theSurf.Continuity() >= BasCriterion && myUSplitValues.Length() == 2
                && myVSplitValues.Length() == 2)
            {
                return;
            }

            var sps = new ShapeUpgrade_SplitSurfaceContinuity();
            sps.Init(theSurf, UFirst, ULast, VFirst, VLast);
            sps.SetUSplitValues(myUSplitValues);
            sps.SetVSplitValues(myVSplitValues);
            sps.SetTolerance(myTolerance);
            sps.SetCriterion(BasCriterion);
            sps.Compute(true);
            myUSplitValues.Clear();
            myUSplitValues.ChangeSequence().Assign(sps.USplitValues()!.Sequence());
            myVSplitValues.Clear();
            myVSplitValues.ChangeSequence().Assign(sps.VSplitValues()!.Sequence());
            myStatus |= sps.myStatus;
            return;
        }

        Geom_BSplineSurface? MyBSpline = null;
        if (mySurface is Geom_BSplineSurface)
        {
            MyBSpline = (Geom_BSplineSurface)mySurface.Copy();
        }
        if (MyBSpline is null)
        {
            return;
        }
        if (mySurface.Continuity() >= myCriterion)
        {
            return;
        }

        // it is a BSplineSurface
        int UDeg = MyBSpline.UDegree();
        int VDeg = MyBSpline.VDegree();
        int NbUKnots = MyBSpline.NbUKnots();
        int UFirstInd = MyBSpline.FirstUKnotIndex() + 1, ULastInd = MyBSpline.LastUKnotIndex() - 1,
            VFirstInd = MyBSpline.FirstVKnotIndex() + 1, VLastInd = MyBSpline.LastVKnotIndex() - 1;
        int NbVKnots = MyBSpline.NbVKnots();

        if (NbUKnots > 2)
        {
            // Only the internal knots are checked.
            int iknot = UFirstInd;
            for (int j = 2; j <= myUSplitValues.Length(); j++)
            {
                ULast = myUSplitValues.Value(j);

                for (; iknot <= ULastInd; iknot++)
                {
                    double valknot = MyBSpline.UKnot(iknot);
                    if (valknot <= UFirst + precision)
                    {
                        continue;
                    }
                    if (valknot >= ULast - precision)
                    {
                        break;
                    }
                    int Continuity = UDeg - MyBSpline.UMultiplicity(iknot);
                    if (Continuity < myCont)
                    {
                        // At this knot, the Surface is C0; try to remove Knot.
                        int newMultiplicity = UDeg - myCont;
                        bool corrected = false;
                        if (newMultiplicity >= 0)
                        {
                            corrected = MyBSpline.RemoveUKnot(iknot, newMultiplicity, myTolerance);
                        }
                        if (corrected && newMultiplicity > 0)
                        {
                            Continuity = UDeg - MyBSpline.UMultiplicity(iknot);
                            corrected = (Continuity >= myCont);
                        }
                        if (corrected)
                        {
                            // at this knot, the continuity is now C1. Nothing else to do.
                            // PTV 15.05.2002 decrease iknot and ULastIndex values if knot removed
                            if (newMultiplicity == 0)
                            {
                                iknot--;
                                ULastInd--;
                            }
                            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                        }
                        else
                        {
                            // impossible to force C1 within the tolerance:
                            // this knot will be a splitting value.
                            double u = MyBSpline.UKnot(iknot);
                            myUSplitValues.InsertBefore(j++, u);
                            myNbResultingRow++;
                        }
                    }
                }
                UFirst = ULast;
            }
        }

        if (NbVKnots > 2)
        {
            // Only the internal knots are checked.
            int iknot = VFirstInd;
            for (int j1 = 2; j1 <= myVSplitValues.Length(); j1++)
            {
                VLast = myVSplitValues.Value(j1);
                for (; iknot <= VLastInd; iknot++)
                {
                    double valknot = MyBSpline.VKnot(iknot);
                    if (valknot <= VFirst + precision)
                    {
                        continue;
                    }
                    if (valknot >= VLast - precision)
                    {
                        break;
                    }
                    int Continuity = VDeg - MyBSpline.VMultiplicity(iknot);
                    if (Continuity < myCont)
                    {
                        // At this knot, the Surface is C0; try to remove Knot.
                        int newMultiplicity = VDeg - myCont;
                        bool corrected = false;
                        if (newMultiplicity >= 0)
                        {
                            corrected = MyBSpline.RemoveVKnot(iknot, newMultiplicity, myTolerance);
                        }
                        if (corrected && newMultiplicity > 0)
                        {
                            Continuity = VDeg - MyBSpline.VMultiplicity(iknot);
                            corrected = (Continuity >= myCont);
                        }
                        if (corrected)
                        {
                            // at this knot, the continuity is now Criterion. Nothing else to do.
                            // PTV 15.05.2002 decrease iknot and ULastIndex values if knot removed
                            if (newMultiplicity == 0)
                            {
                                iknot--;
                                VLastInd--;
                            }
                            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                        }
                        else
                        {
                            // this knot will be a splitting value.
                            double v = MyBSpline.VKnot(iknot);
                            myVSplitValues.InsertBefore(j1++, v);
                            myNbResultingCol++;
                        }
                    }
                }
                VFirst = VLast;
            }
        }
        if (Status(ShapeExtend_DONE3))
        {
            mySurface = MyBSpline;
        }

        if (myUSplitValues.Length() > 2 || myVSplitValues.Length() > 2)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
    }
}
