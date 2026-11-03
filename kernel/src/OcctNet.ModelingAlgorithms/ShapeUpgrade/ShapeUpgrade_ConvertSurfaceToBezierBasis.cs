// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Converts a surface into a grid of Bezier-based surfaces: planes and B-splines into Bezier patches, surfaces of
/// revolution and of linear extrusion into surfaces on Bezier curves (each conversion with its mode), trimmed and offset
/// surfaces through their basis; the grid is then cut at the splitting values.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_ConvertSurfaceToBezierBasis : ShapeUpgrade_SplitSurface
{
    private ShapeExtend_CompositeSurface? mySegments;
    private bool myPlaneMode;
    private bool myRevolutionMode;
    private bool myExtrusionMode;
    private bool myBSplineMode;

    /// <summary>Empty constructor (all modes on).</summary>
    public ShapeUpgrade_ConvertSurfaceToBezierBasis()
    {
        myPlaneMode = true;
        myRevolutionMode = true;
        myExtrusionMode = true;
        myBSplineMode = true;
    }

    /// <summary>Converts the surface into a grid of Bezier-based surfaces and stores it.</summary>
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
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();

        if (mySurface is Geom_RectangularTrimmedSurface Surface)
        {
            Geom_Surface BasSurf = Surface.BasisSurface();
            var converter = new ShapeUpgrade_ConvertSurfaceToBezierBasis();
            converter.Init(BasSurf, UFirst, ULast, VFirst, VLast);
            converter.SetUSplitValues(myUSplitValues);
            converter.SetVSplitValues(myVSplitValues);
            converter.Compute(true);
            myUSplitValues.ChangeSequence().Assign(converter.USplitValues()!.Sequence());
            myVSplitValues.ChangeSequence().Assign(converter.VSplitValues()!.Sequence());
            myStatus |= converter.myStatus;
            mySegments = converter.Segments();
            return;
        }
        else if (mySurface is Geom_OffsetSurface Offset)
        {
            Geom_Surface BasSurf = Offset.BasisSurface();
            var converter = new ShapeUpgrade_ConvertSurfaceToBezierBasis();
            converter.Init(BasSurf, UFirst, ULast, VFirst, VLast);
            converter.SetUSplitValues(myUSplitValues);
            converter.SetVSplitValues(myVSplitValues);
            converter.Compute(true);
            myUSplitValues.ChangeSequence().Assign(converter.USplitValues()!.Sequence());
            myVSplitValues.ChangeSequence().Assign(converter.VSplitValues()!.Sequence());
            myStatus |= converter.myStatus;
            mySegments = converter.Segments();
            return;
        }
        else if (mySurface is Geom_Plane pln && myPlaneMode)
        {
            var poles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
            Point3D dp;
            poles[1, 1] = dp = pln.Value(UFirst, VFirst);
            poles[1, 2] = dp = pln.Value(UFirst, VLast);
            poles[2, 1] = dp = pln.Value(ULast, VFirst);
            poles[2, 2] = dp = pln.Value(ULast, VLast);
            var bezierPlane = new Geom_BezierSurface(poles);
            var UJoints = new NCollection_Array1<double>(1, 2);
            UJoints[1] = UFirst;
            UJoints[2] = ULast;
            var VJoints = new NCollection_Array1<double>(1, 2);
            VJoints[1] = VFirst;
            VJoints[2] = VLast;
            var surf = new NCollection_HArray2<Geom_Surface>(1, 1, 1, 1);
            surf.SetValue(1, 1, bezierPlane);
            mySegments = new ShapeExtend_CompositeSurface(surf, UJoints, VJoints);
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return;
        }
        else if (mySurface is Geom_BezierSurface bezierSurf)
        {
            var surf = new NCollection_HArray2<Geom_Surface>(1, 1, 1, 1);
            var UJoints = new NCollection_Array1<double>(1, 2);
            UJoints[1] = UFirst;
            UJoints[2] = ULast;
            var VJoints = new NCollection_Array1<double>(1, 2);
            VJoints[1] = VFirst;
            VJoints[2] = VLast;
            if (UFirst < precision && ULast > 1 - precision && VFirst < precision && VLast > 1 - precision)
            {
                surf.SetValue(1, 1, bezierSurf);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
            }
            else
            {
                var besNew = (Geom_BezierSurface)bezierSurf.Copy();
                // pdn K4L+ (work around)
                // rln C30 (direct use)
                double u1 = UFirst;
                double u2 = ULast;
                double v1 = VFirst;
                double v2 = VLast;
                besNew.Segment(u1, u2, v1, v2);
                surf.SetValue(1, 1, besNew);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            mySegments = new ShapeExtend_CompositeSurface(surf, UJoints, VJoints);
            return;
        }
        else if (mySurface is Geom_BSplineSurface bspline && myBSplineMode)
        {
            // pdn
            bspline.Bounds(out _, out _, out _, out _);
            var converter = new GeomConvert_BSplineSurfaceToBezierSurface(bspline); //,UFirst,ULast,VFirst,VLast,precision;
            int nbUPatches = converter.NbUPatches();
            int nbVPatches = converter.NbVPatches();
            var UJoints = new NCollection_Array1<double>(1, nbUPatches + 1);
            var UReject = new NCollection_Array1<bool>(1, nbUPatches + 1);
            UReject.Init(false);
            var VJoints = new NCollection_Array1<double>(1, nbVPatches + 1);
            var VReject = new NCollection_Array1<bool>(1, nbVPatches + 1);
            VReject.Init(false);
            int NbUFiltered = 0;
            int NbVFiltered = 0;

            converter.UKnots(UJoints);
            var UFilteredJoints = new NCollection_Sequence<double>();
            UFilteredJoints.Append(UJoints[1]);
            int i;
            for (i = 2; i <= nbUPatches + 1; i++)
            {
                if (UJoints[i] - UJoints[i - 1] < precision)
                {
                    NbUFiltered++;
                    UReject[i - 1] = true;
                }
                else
                {
                    UFilteredJoints.Append(UJoints[i]);
                }
            }

            converter.VKnots(VJoints);
            var VFilteredJoints = new NCollection_Sequence<double>();
            VFilteredJoints.Append(VJoints[1]);
            for (i = 2; i <= nbVPatches + 1; i++)
            {
                if (VJoints[i] - VJoints[i - 1] < precision)
                {
                    NbVFiltered++;
                    VReject[i - 1] = true;
                }
                else
                {
                    VFilteredJoints.Append(VJoints[i]);
                }
            }

            // OCCT_DEBUG output left out: thin patches dropped

            var Surfaces = new NCollection_Array2<Geom_BezierSurface>(1, nbUPatches, 1, nbVPatches);
            converter.Patches(Surfaces);
            var srf = new NCollection_HArray2<Geom_Surface>(1, nbUPatches - NbUFiltered, 1, nbVPatches - NbVFiltered);
            int indApp1 = 0;
            for (int ind1 = 1; ind1 <= nbUPatches; ind1++)
            {
                if (UReject[ind1])
                {
                    continue;
                }
                indApp1++;
                int indApp2 = 0;
                for (int ind2 = 1; ind2 <= nbVPatches; ind2++)
                {
                    if (VReject[ind2])
                    {
                        continue;
                    }
                    indApp2++;
                    srf.SetValue(indApp1, indApp2, Surfaces[ind1, ind2]);
                }
            }

            var uj = new NCollection_Array1<double>(1, UFilteredJoints.Length());
            for (i = 1; i <= UFilteredJoints.Length(); i++)
            {
                uj[i] = UFilteredJoints.Value(i);
            }

            var vj = new NCollection_Array1<double>(1, VFilteredJoints.Length());
            for (i = 1; i <= VFilteredJoints.Length(); i++)
            {
                vj[i] = VFilteredJoints.Value(i);
            }

            mySegments = new ShapeExtend_CompositeSurface(srf, uj, vj);

            int j;
            for (j = 2; j <= myUSplitValues.Length(); j++)
            {
                ULast = myUSplitValues.Value(j);
                for (int ii = 2; ii <= nbUPatches + 1; ii++)
                {
                    double valknot = UJoints[ii];
                    if (valknot - UFirst <= precision)
                    {
                        continue;
                    }
                    if (ULast - valknot <= precision)
                    {
                        break;
                    }
                    myUSplitValues.InsertBefore(j++, valknot);
                }
                UFirst = ULast;
            }

            for (j = 2; j <= myVSplitValues.Length(); j++)
            {
                VLast = myVSplitValues.Value(j);
                for (int ii = 2; ii <= nbVPatches + 1; ii++)
                {
                    double valknot = VJoints[ii];
                    if (valknot - VFirst <= precision)
                    {
                        continue;
                    }
                    if (VLast - valknot <= precision)
                    {
                        break;
                    }
                    myVSplitValues.InsertBefore(j++, valknot);
                }
                VFirst = VLast;
            }
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return;
        }
        else if (mySurface is Geom_SurfaceOfRevolution revol && myRevolutionMode)
        {
            Geom_Curve basis = revol.BasisCurve();
            if (basis is Geom_TrimmedCurve tc)
            {
                basis = tc.BasisCurve();
            }
            NCollection_HArray1<Geom_Curve> curves;
            int nbCurves;
            var vPar = new NCollection_HSequence<double>();
            var vSVal = new NCollection_HSequence<double>();
            if (basis is Geom_OffsetCurve offset)
            {
                double value = offset.Offset();
                Direction3D direction = offset.Direction();
                Geom_Curve bas = offset.BasisCurve();
                var converter = new ShapeUpgrade_ConvertCurve3dToBezier();
                converter.Init(bas, VFirst, VLast);
                converter.Perform(true);
                if (converter.Status(ShapeExtend_DONE))
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                else
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
                }

                vPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
                vSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
                curves = converter.GetCurves()!;
                nbCurves = curves.Length();
                for (int i = 1; i <= nbCurves; i++)
                {
                    var offCur = new Geom_OffsetCurve(curves.Value(i), value, direction);
                    curves.SetValue(i, offCur);
                }
            }
            else
            {
                var converter = new ShapeUpgrade_ConvertCurve3dToBezier();
                converter.Init(basis, VFirst, VLast);
                converter.Perform(true);
                if (converter.Status(ShapeExtend_DONE))
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                else
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
                }

                vPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
                vSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
                curves = converter.GetCurves()!;
                nbCurves = curves.Length();
            }

            Axis3D axis = revol.Axis();
            var surf = new NCollection_HArray2<Geom_Surface>(1, 1, 1, nbCurves);
            mySurface.Bounds(out double Umin, out double Umax, out _, out _);
            int k;
            for (k = 1; k <= nbCurves; k++)
            {
                var rev = new Geom_SurfaceOfRevolution(curves.Value(k), axis);
                if (UFirst - Umin < Kerf.Engine.Foundation.Precision.PConfusion() && Umax - ULast < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    surf.SetValue(1, k, rev);
                }
                else
                {
                    var rect = new Geom_RectangularTrimmedSurface(rev, UFirst, ULast, true);
                    surf.SetValue(1, k, rect);
                }
            }
            var UJoints = new NCollection_Array1<double>(1, 2);
            var VJoints = new NCollection_Array1<double>(1, nbCurves + 1);
            UJoints[1] = UFirst;
            UJoints[2] = ULast;
            for (k = 1; k <= nbCurves + 1; k++)
            {
                VJoints[k] = vPar.Value(k);
            }

            mySegments = new ShapeExtend_CompositeSurface(surf, UJoints, VJoints);

            for (int j = 2; j <= myVSplitValues.Length(); j++)
            {
                VLast = myVSplitValues.Value(j);
                for (int ii = 2; ii <= nbCurves + 1; ii++)
                {
                    double valknot = vSVal.Value(ii);
                    if (valknot - VFirst <= precision)
                    {
                        continue;
                    }
                    if (VLast - valknot <= precision)
                    {
                        break;
                    }
                    myVSplitValues.InsertBefore(j++, valknot);
                }
                VFirst = VLast;
            }
            return;
        }
        else if (mySurface is Geom_SurfaceOfLinearExtrusion extr && myExtrusionMode)
        {
            Geom_Curve basis = extr.BasisCurve();
            // gp_Dir direction = extr->Direction(); // direction not used (skl)
            NCollection_HArray1<Geom_Curve> curves;
            int nbCurves;
            var uPar = new NCollection_HSequence<double>();
            var uSVal = new NCollection_HSequence<double>();
            var converter = new ShapeUpgrade_ConvertCurve3dToBezier();
            converter.Init(basis, UFirst, ULast);
            converter.Perform(true);
            uPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
            uSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            curves = converter.GetCurves()!;
            nbCurves = curves.Length();

            Transform3D shiftF = new(), shiftL = new();
            shiftF.SetTranslation(extr.Value(UFirst, 0), extr.Value(UFirst, VFirst));
            shiftL.SetTranslation(extr.Value(UFirst, 0), extr.Value(UFirst, VLast));
            var surf = new NCollection_HArray2<Geom_Surface>(1, nbCurves, 1, 1);

            int k;
            for (k = 1; k <= nbCurves; k++)
            {
                // OCCT down-casts the curve to a Bezier and dereferences it
                var bez = (Geom_BezierCurve)curves.Value(k);
                int nbPoles = bez.NbPoles();
                NCollection_Array1<Point3D> poles = bez.InternalPoles();
                var resPoles = new NCollection_Array2<Point3D>(1, nbPoles, 1, 2);
                for (int j = 1; j <= nbPoles; j++)
                {
                    resPoles[j, 1] = poles[j].Transformed(shiftF);
                    resPoles[j, 2] = poles[j].Transformed(shiftL);
                }
                var bezier = new Geom_BezierSurface(resPoles);
                surf.SetValue(k, 1, bezier);
            }

            var UJoints = new NCollection_Array1<double>(1, nbCurves + 1);
            var VJoints = new NCollection_Array1<double>(1, 2);
            VJoints[1] = VFirst;
            VJoints[2] = VLast;
            for (k = 1; k <= nbCurves + 1; k++)
            {
                UJoints[k] = uPar.Value(k);
            }

            mySegments = new ShapeExtend_CompositeSurface(surf, UJoints, VJoints);

            for (int j = 2; j <= myUSplitValues.Length(); j++)
            {
                ULast = myUSplitValues.Value(j);
                for (int ii = 2; ii <= nbCurves + 1; ii++)
                {
                    double valknot = uSVal.Value(ii);
                    if (valknot + UFirst <= precision)
                    {
                        continue;
                    }
                    if (ULast - valknot <= precision)
                    {
                        break;
                    }
                    myUSplitValues.InsertBefore(j++, valknot);
                }
                UFirst = ULast;
            }
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return;
        }
        else
        {
            var UJoints = new NCollection_Array1<double>(1, 2);
            UJoints[1] = UFirst;
            UJoints[2] = ULast;
            var VJoints = new NCollection_Array1<double>(1, 2);
            VJoints[1] = VFirst;
            VJoints[2] = VLast;
            var surf = new NCollection_HArray2<Geom_Surface>(1, 1, 1, 1);
            mySurface!.Bounds(out double U1, out double U2, out _, out double V2);
            Geom_Surface S;
            if (U1 - UFirst < precision && ULast - U2 < precision && V2 - VFirst < precision
                && VLast - V2 < precision)
            {
                S = mySurface;
            }
            else
            {
                var rts = new Geom_RectangularTrimmedSurface(mySurface, UFirst, ULast, VFirst, VLast);
                S = rts;
            }
            surf.SetValue(1, 1, S);
            mySegments = new ShapeExtend_CompositeSurface(surf, UJoints, VJoints);
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
            return;
        }
    }

    // OCCT's file-static GetSegment: the part of a patch between the parameters.
    private static Geom_Surface GetSegment(Geom_Surface surf, double U1, double U2, double V1, double V2)
    {
        if (surf is Geom_BezierSurface)
        {
            var bezier = (Geom_BezierSurface)surf.Copy();
            double prec = Kerf.Engine.Foundation.Precision.PConfusion();
            if (U1 < prec && U2 > 1 - prec && V1 < prec && V2 > 1 - prec)
            {
                return bezier;
            }
            // pdn K4L+ (work around)
            // rln C30 (direct use)
            double u1 = U1;
            double u2 = U2;
            double v1 = V1;
            double v2 = V2;
            bezier.Segment(u1, u2, v1, v2);
            return bezier;
        }

        Geom_Surface S;
        if (surf is Geom_RectangularTrimmedSurface rect)
        {
            S = rect.BasisSurface();
        }
        else
        {
            S = surf;
        }

        if (S is Geom_SurfaceOfRevolution)
        {
            var revol = (Geom_SurfaceOfRevolution)S.Copy();
            revol.Bounds(out double Umin, out double Umax, out _, out _);
            Geom_Curve basis = revol.BasisCurve();
            Geom_OffsetCurve? offset = null;
            if (basis is Geom_OffsetCurve anOffset)
            {
                offset = anOffset;
                basis = anOffset.BasisCurve();
            }
            if (basis is Geom_BezierCurve bezierBasis)
            {
                // OCCT.NET: OCCT segments the copy's basis curve in place; the port's basis curves are shared and
                // frozen once handed out (decision 159), so the segment is made on a copy, rebuilt into the offset
                // curve if there is one and set as the copy's basis (decision 998)
                var aSegment = (Geom_BezierCurve)bezierBasis.Copy();
                aSegment.Segment(V1, V2);
                Geom_Curve aNewBasis = offset is null ? aSegment : new Geom_OffsetCurve(aSegment, offset.Offset(), offset.Direction());
                revol.SetBasisCurve(aNewBasis);
            }
            else
            {
                // OCCT_DEBUG output left out: resulting path is not surface of revolution basis on bezier curve
            }
            if (Math.Abs(U1 - Umin) < Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(U2 - Umax) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return revol;
            }

            var res = new Geom_RectangularTrimmedSurface(revol, U1, U2, true);
            return res;
        }
        else
        {
            surf.Bounds(out double Umin, out double Umax, out double Vmin, out double Vmax);
            if (U1 - Umin < Kerf.Engine.Foundation.Precision.PConfusion() && Umax - U2 < Kerf.Engine.Foundation.Precision.PConfusion()
                && V1 - Vmin < Kerf.Engine.Foundation.Precision.PConfusion() && Vmax - V2 < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return surf;
            }

            var res = new Geom_RectangularTrimmedSurface(surf, U1, U2, V1, V2);
            return res;
        }
    }

    /// <summary>Cuts the grid of Bezier-based surfaces made by <see cref="Compute"/> at the splitting values.</summary>
    public override void Build(bool Segment)
    {
        bool isOffset = false;
        double offsetValue = 0;
        Geom_Surface S;
        if (mySurface is Geom_RectangularTrimmedSurface Surface)
        {
            S = Surface.BasisSurface();
        }
        else
        {
            S = mySurface!;
        }
        if (S is Geom_OffsetSurface offSur)
        {
            isOffset = true;
            offsetValue = offSur.Offset();
        }

        double prec = Kerf.Engine.Foundation.Precision.PConfusion();
        NCollection_HArray1<double> myUSplitParams = mySegments!.InternalUJointValues()!;
        NCollection_HArray1<double> myVSplitParams = mySegments.InternalVJointValues()!;
        int nbU = myUSplitValues!.Length();
        int nbV = myVSplitValues!.Length();

        var resSurfaces = new NCollection_HArray2<Geom_Surface>(1, nbU - 1, 1, nbV - 1);
        int j1 = 2;
        for (int i1 = 2; i1 <= nbU; i1++)
        {
            double parU = myUSplitValues.Value(i1);
            for (; j1 <= myUSplitParams.Length(); j1++)
            {
                double param = myUSplitParams.Value(j1);
                if (parU - param < prec)
                {
                    break;
                }
            }

            int j2 = 2;
            for (int i2 = 2; i2 <= nbV; i2++)
            {
                double parV = myVSplitValues.Value(i2);
                for (; j2 <= myVSplitParams.Length(); j2++)
                {
                    if (parV - myVSplitParams.Value(j2) < prec)
                    {
                        break;
                    }
                }

                Geom_Surface patch = mySegments.Patch(j1 - 1, j2 - 1);
                patch.Bounds(out double U1, out double U2, out double V1, out double V2);
                // linear recomputation of part:
                double uFirst = myUSplitParams.Value(j1 - 1);
                double uLast = myUSplitParams.Value(j1);
                double vFirst = myVSplitParams.Value(j2 - 1);
                double vLast = myVSplitParams.Value(j2);
                double uFact = (U2 - U1) / (uLast - uFirst);
                double vFact = (V2 - V1) / (vLast - vFirst);
                double ppU = myUSplitValues.Value(i1 - 1);
                double ppV = myVSplitValues.Value(i2 - 1);
                // defining a part
                double uL1 = U1 + (ppU - uFirst) * uFact;
                double uL2 = U1 + (parU - uFirst) * uFact;
                double vL1 = V1 + (ppV - vFirst) * vFact;
                double vL2 = V1 + (parV - vFirst) * vFact;
                Geom_Surface res = GetSegment(patch, uL1, uL2, vL1, vL2);
                if (isOffset)
                {
                    var resOff = new Geom_OffsetSurface(res, offsetValue);
                    res = resOff;
                }
                resSurfaces.SetValue(i1 - 1, i2 - 1, res);
            }
        }

        var UJoints = new NCollection_Array1<double>(1, nbU);
        int i;
        for (i = 1; i <= nbU; i++)
        {
            UJoints[i] = myUSplitValues.Value(i);
        }

        var VJoints = new NCollection_Array1<double>(1, nbV);
        for (i = 1; i <= nbV; i++)
        {
            VJoints[i] = myVSplitValues.Value(i);
        }

        myResSurfaces = new ShapeExtend_CompositeSurface(resSurfaces, UJoints, VJoints);
    }

    /// <summary>Returns the grid of Bezier-based surfaces made from the surface.</summary>
    public ShapeExtend_CompositeSurface? Segments() => mySegments;

    /// <summary>Sets the mode for converting Geom_Plane to a Bezier.</summary>
    public void SetPlaneMode(bool mode)
    {
        myPlaneMode = mode;
    }

    /// <summary>Returns the Geom_Plane conversion mode.</summary>
    public bool GetPlaneMode() => myPlaneMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfRevolution to Beziers.</summary>
    public void SetRevolutionMode(bool mode)
    {
        myRevolutionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfRevolution conversion mode.</summary>
    public bool GetRevolutionMode() => myRevolutionMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfLinearExtrusion to Beziers.</summary>
    public void SetExtrusionMode(bool mode)
    {
        myExtrusionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfLinearExtrusion conversion mode.</summary>
    public bool GetExtrusionMode() => myExtrusionMode;

    /// <summary>Sets the mode for converting Geom_BSplineSurface to Beziers.</summary>
    public void SetBSplineMode(bool mode)
    {
        myBSplineMode = mode;
    }

    /// <summary>Returns the Geom_BSplineSurface conversion mode.</summary>
    public bool GetBSplineMode() => myBSplineMode;
}
