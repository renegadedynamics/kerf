// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Splits a surface with a criterion (the base class splits at the U and V values given) into a grid of patches,
/// returned as a <see cref="ShapeExtend_CompositeSurface"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). The surface given is not copied (OCCT's isn't); the patches are segments or
/// trimmed copies of it, or the surface itself when nothing is cut.
/// </remarks>
public class ShapeUpgrade_SplitSurface : Standard_Transient
{
    /// <summary>The U splitting values, the first and last included.</summary>
    protected NCollection_HSequence<double>? myUSplitValues;
    /// <summary>The V splitting values, the first and last included.</summary>
    protected NCollection_HSequence<double>? myVSplitValues;
    /// <summary>The number of rows of the resulting grid.</summary>
    protected int myNbResultingRow;
    /// <summary>The number of columns of the resulting grid.</summary>
    protected int myNbResultingCol;
    /// <summary>The surface.</summary>
    protected Geom_Surface? mySurface;
    /// <summary>The status.</summary>
    protected int myStatus;
    /// <summary>The resulting grid.</summary>
    protected ShapeExtend_CompositeSurface? myResSurfaces;
    /// <summary>The area of the face (0 when not given).</summary>
    protected double myArea;
    /// <summary>The length of the middle V iso (with an area).</summary>
    protected double myUsize;
    /// <summary>The length of the middle U iso (with an area).</summary>
    protected double myVsize;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_SplitSurface()
    {
        myNbResultingRow = 0;
        myNbResultingCol = 0;
        myStatus = 0;
    }

    /// <summary>Initializes with a single supporting surface (its bounds).</summary>
    public void Init(Geom_Surface S)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        myUSplitValues = new NCollection_HSequence<double>();
        myVSplitValues = new NCollection_HSequence<double>();
        mySurface = S;
        myResSurfaces = new ShapeExtend_CompositeSurface();
        myNbResultingRow = 1;
        myNbResultingCol = 1;
        mySurface.Bounds(out double U1, out double U2, out double V1, out double V2);

        myUSplitValues.Append(U1);
        myUSplitValues.Append(U2);

        myVSplitValues.Append(V1);
        myVSplitValues.Append(V2);
    }

    /// <summary>
    /// Initializes with a single supporting surface and bounding parameters (and, when <paramref name="theArea"/> isn't
    /// 0, the face's area, measuring the middle isos for splitting by area).
    /// </summary>
    public void Init(Geom_Surface S, double UFirst, double ULast, double VFirst, double VLast, double theArea = 0.0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        mySurface = S;
        myResSurfaces = new ShapeExtend_CompositeSurface();
        myUSplitValues = new NCollection_HSequence<double>();
        myVSplitValues = new NCollection_HSequence<double>();

        myNbResultingRow = 1;
        myNbResultingCol = 1;

        myArea = theArea;

        mySurface.Bounds(out double U1, out double U2, out double V1, out double V2);
        double precision = Occt.Precision.PConfusion();
        if (mySurface.IsUPeriodic() && ULast - UFirst <= U2 - U1 + precision)
        {
            U1 = UFirst;
            U2 = U1 + mySurface.UPeriod();
        }
        if (mySurface.IsVPeriodic() && VLast - VFirst <= V2 - V1 + precision)
        {
            V1 = VFirst;
            V2 = V1 + mySurface.VPeriod();
        }
        double UF, UL, VF, VL;
        if (UFirst > U2 - precision || ULast < U1 - precision)
        {
            UF = U1;
            UL = U2;
        }
        else
        {
            UF = Math.Max(U1, UFirst);
            UL = Math.Min(U2, ULast);
        }
        if (VFirst > V2 - precision || VLast < V1 - precision)
        {
            VF = V1;
            VL = V2;
        }
        else
        {
            VF = Math.Max(V1, VFirst);
            VL = Math.Min(V2, VLast);
        }

        if (myArea != 0.0)
        {
            //<myArea> is set and will be used with <myUsize> and <myVsize>
            // in further computations
            double Umid = (UF + UL) / 2, Vmid = (VF + VL) / 2;
            var aTrSurf = new Geom_RectangularTrimmedSurface(mySurface, UF, UL, VF, VL);
            Geom_Curve anUiso = aTrSurf.UIso(Umid);
            Geom_Curve aViso = aTrSurf.VIso(Vmid);
            TopoDS_Edge anEdgeUiso = new BRepLib_MakeEdge(anUiso).InternalEdge();
            TopoDS_Edge anEdgeViso = new BRepLib_MakeEdge(aViso).InternalEdge();
            GProp_GProps aGprop1 = new(), aGprop2 = new();
            BRepGProp.LinearProperties(anEdgeViso, aGprop1);
            myUsize = aGprop1.Mass();
            BRepGProp.LinearProperties(anEdgeUiso, aGprop2);
            myVsize = aGprop2.Mass();
        }

        if (UL - UF < precision)
        {
            double p2 = precision / 2.0;
            UF -= p2;
            UL += p2;
        }
        if (VL - VF < precision)
        {
            double p2 = precision / 2.0;
            VF -= p2;
            VL += p2;
        }

        myUSplitValues.Append(UF);
        myUSplitValues.Append(UL);
        myVSplitValues.Append(VF);
        myVSplitValues.Append(VL);
    }

    /// <summary>Sets the U parameters where splitting has to be done (those inside the current intervals).</summary>
    public void SetUSplitValues(NCollection_HSequence<double>? UValues)
    {
        if (UValues is null)
        {
            return;
        }
        double precision = Occt.Precision.PConfusion();
        double UFirst = myUSplitValues!.Value(1), ULast = myUSplitValues.Value(myUSplitValues.Length());
        int i = 1;
        int len = UValues.Length();

        for (int ku = 2; ku <= myUSplitValues.Length(); ku++)
        {
            ULast = myUSplitValues.Value(ku);
            for (; i <= len; i++)
            {
                if ((UFirst + precision) >= UValues.Value(i))
                {
                    continue;
                }
                if ((ULast - precision) <= UValues.Value(i))
                {
                    break;
                }
                myUSplitValues.InsertBefore(ku++, UValues.Value(i));
            }
            UFirst = ULast;
        }
    }

    /// <summary>Sets the V parameters where splitting has to be done (those inside the current intervals).</summary>
    public void SetVSplitValues(NCollection_HSequence<double>? VValues)
    {
        if (VValues is null)
        {
            return;
        }
        double precision = Occt.Precision.PConfusion();
        double VFirst = myVSplitValues!.Value(1), VLast = myVSplitValues.Value(myVSplitValues.Length());
        int i = 1;
        int len = VValues.Length();
        for (int kv = 2; kv <= myVSplitValues.Length(); kv++)
        {
            VLast = myVSplitValues.Value(kv);
            for (; i <= len; i++)
            {
                if ((VFirst + precision) >= VValues.Value(i))
                {
                    continue;
                }
                if ((VLast - precision) <= VValues.Value(i))
                {
                    break;
                }
                myVSplitValues.InsertBefore(kv++, VValues.Value(i));
            }
            VFirst = VLast;
        }
    }

    /// <summary>
    /// Performs the splitting of the surface: with <paramref name="Segment"/> a B-spline or Bezier result is made of
    /// segments of the surface bounded by the splitting values, otherwise of rectangular trimmed surfaces on the
    /// same surface.
    /// </summary>
    public virtual void Build(bool Segment)
    {
        double UFirst = myUSplitValues!.Value(1);
        double ULast = myUSplitValues.Value(myUSplitValues.Length());
        double VFirst = myVSplitValues!.Value(1);
        double VLast = myVSplitValues.Value(myVSplitValues.Length());

        if (myUSplitValues.Length() > 2 || myVSplitValues.Length() > 2)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        mySurface!.Bounds(out double U1, out double U2, out double V1, out double V2);
        if (mySurface is Geom_SurfaceOfRevolution SurfaceRev)
        {
            Geom_Curve BasCurve = SurfaceRev.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve3d();
            spc.Init(BasCurve, VFirst, VLast);
            spc.SetSplitValues(myVSplitValues);
            spc.Build(Segment);
            NCollection_HArray2<Geom_Surface> Surfaces;
            myNbResultingCol = spc.GetCurves()!.Length();
            if (myUSplitValues.Length() > 2)
            {
                myNbResultingRow = myUSplitValues.Length() - 1;
                Surfaces = new NCollection_HArray2<Geom_Surface>(1, myNbResultingRow, 1, myNbResultingCol);
                for (int nc = 1; nc <= myNbResultingCol; nc++)
                {
                    var NewSurfaceRev = new Geom_SurfaceOfRevolution(spc.GetCurves()!.Value(nc), SurfaceRev.Axis());
                    NewSurfaceRev.Bounds(out double U1p, out double U2p, out double V1p, out double V2p);
                    for (int nc1 = 1; nc1 <= myNbResultingRow; nc1++)
                    {
                        var NewSurf = new Geom_RectangularTrimmedSurface(NewSurfaceRev,
                                                                         myUSplitValues.Value(nc1),
                                                                         myUSplitValues.Value(nc1 + 1),
                                                                         V1p,
                                                                         V2p);
                        Surfaces.SetValue(nc1, nc, NewSurf);
                    }
                }
            }
            else
            {
                Surfaces = new NCollection_HArray2<Geom_Surface>(1, 1, 1, myNbResultingCol);

                for (int nc = 1; nc <= spc.GetCurves()!.Length(); nc++)
                {
                    var NewSurfaceRev = new Geom_SurfaceOfRevolution(spc.GetCurves()!.Value(nc), SurfaceRev.Axis());
                    NewSurfaceRev.Bounds(out U1, out U2, out V1, out V2);
                    if (UFirst == U1 && ULast == U2)
                    {
                        Surfaces.SetValue(1, nc, NewSurfaceRev);
                    }
                    else
                    {
                        var NewSurf = new Geom_RectangularTrimmedSurface(NewSurfaceRev, UFirst, ULast, V1, V2); // pdn correction for main seq
                        Surfaces.SetValue(1, nc, NewSurf);
                    }
                }
            }
            myResSurfaces!.Init(Surfaces);
            myResSurfaces.SetUFirstValue(myUSplitValues.Sequence().First());
            myResSurfaces.SetVFirstValue(myVSplitValues.Sequence().First());
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
            Geom_Curve BasCurve = SurfaceExt.BasisCurve();
            var spc = new ShapeUpgrade_SplitCurve3d();
            spc.Init(BasCurve, UFirst, ULast);
            spc.SetSplitValues(myUSplitValues);
            spc.Build(Segment);
            myNbResultingRow = spc.GetCurves()!.Length();
            NCollection_HArray2<Geom_Surface> Surfaces;
            if (myVSplitValues.Length() > 2)
            {
                myNbResultingCol = myVSplitValues.Length() - 1;
                Surfaces = new NCollection_HArray2<Geom_Surface>(1, myNbResultingRow, 1, myNbResultingCol);
                for (int nc1 = 1; nc1 <= myNbResultingRow; nc1++)
                {
                    var NewSurfaceEx = new Geom_SurfaceOfLinearExtrusion(spc.GetCurves()!.Value(nc1), SurfaceExt.Direction());
                    NewSurfaceEx.Bounds(out double U1p, out double U2p, out double V1p, out double V2p);
                    for (int nc2 = 1; nc2 <= myNbResultingCol; nc2++)
                    {
                        var NewSurf = new Geom_RectangularTrimmedSurface(NewSurfaceEx,
                                                                         U1p,
                                                                         U2p,
                                                                         myVSplitValues.Value(nc2),
                                                                         myVSplitValues.Value(nc2 + 1));
                        Surfaces.SetValue(nc1, nc2, NewSurf);
                    }
                }
            }
            else
            {
                Surfaces = new NCollection_HArray2<Geom_Surface>(1, myNbResultingRow, 1, 1);

                for (int nc1 = 1; nc1 <= myNbResultingRow; nc1++)
                {
                    var NewSurfaceEx = new Geom_SurfaceOfLinearExtrusion(spc.GetCurves()!.Value(nc1), SurfaceExt.Direction());
                    NewSurfaceEx.Bounds(out U1, out U2, out V1, out V2);
                    if (VFirst == V1 && VLast == V2)
                    {
                        Surfaces.SetValue(nc1, 1, NewSurfaceEx);
                    }
                    else
                    {
                        var NewSurf = new Geom_RectangularTrimmedSurface(NewSurfaceEx,
                                                                         Math.Max(U1, UFirst),
                                                                         Math.Min(ULast, U2),
                                                                         Math.Max(VFirst, V1),
                                                                         Math.Min(VLast, V2));
                        Surfaces.SetValue(nc1, 1, NewSurf);
                    }
                }
            }
            myResSurfaces!.Init(Surfaces);
            myResSurfaces.SetUFirstValue(myUSplitValues.Sequence().First());
            myResSurfaces.SetVFirstValue(myVSplitValues.Sequence().First());
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

        if (mySurface is Geom_RectangularTrimmedSurface tmpRect)
        {
            Geom_Surface theSurf = tmpRect.BasisSurface();
            var sps = new ShapeUpgrade_SplitSurface();
            sps.Init(theSurf, UFirst, ULast, VFirst, VLast);
            sps.SetUSplitValues(myUSplitValues);
            sps.SetVSplitValues(myVSplitValues);
            sps.myStatus = myStatus;
            sps.Build(Segment);
            myStatus |= sps.myStatus;
            myResSurfaces = sps.myResSurfaces;
            return;
        }
        else if (mySurface is Geom_OffsetSurface tmpOffset)
        {
            Geom_Surface theSurf = tmpOffset.BasisSurface();
            var sps = new ShapeUpgrade_SplitSurface();
            sps.Init(theSurf, UFirst, ULast, VFirst, VLast);
            sps.SetUSplitValues(myUSplitValues);
            sps.SetVSplitValues(myVSplitValues);
            sps.myStatus = myStatus;
            sps.Build(Segment);
            var Patches = new NCollection_HArray2<Geom_Surface>(1,
                                                                 sps.ResSurfaces()!.NbUPatches(),
                                                                 1,
                                                                 sps.ResSurfaces()!.NbVPatches());
            for (int i1 = 1; i1 <= sps.ResSurfaces()!.NbUPatches(); i1++)
            {
                for (int j1 = 1; j1 <= sps.ResSurfaces()!.NbVPatches(); j1++)
                {
                    var NewOffSur = new Geom_OffsetSurface(sps.ResSurfaces()!.Patch(i1, j1), tmpOffset.Offset());
                    Patches.SetValue(i1, j1, NewOffSur);
                }
            }
            myResSurfaces!.Init(Patches);
            myResSurfaces.SetUFirstValue(myUSplitValues.Sequence().First());
            myResSurfaces.SetVFirstValue(myVSplitValues.Sequence().First());
            myStatus |= sps.myStatus;
            return;
        }

        // splitting the surfaces:
        myNbResultingRow = myUSplitValues.Length() - 1;
        myNbResultingCol = myVSplitValues.Length() - 1;
        var SurfacesGrid = new NCollection_HArray2<Geom_Surface>(1, myNbResultingRow, 1, myNbResultingCol);
        bool isBSpline = mySurface is Geom_BSplineSurface;
        bool isBezier = mySurface is Geom_BezierSurface;

        if (myNbResultingRow == 1 && myNbResultingCol == 1)
        {
            mySurface.Bounds(out U1, out U2, out V1, out V2);
            bool filled = true;
            if (Math.Abs(U1 - UFirst) < Occt.Precision.PConfusion()
                && Math.Abs(U2 - ULast) < Occt.Precision.PConfusion()
                && Math.Abs(V1 - VFirst) < Occt.Precision.PConfusion()
                && Math.Abs(V2 - VLast) < Occt.Precision.PConfusion())
            {
                SurfacesGrid.SetValue(1, 1, mySurface);
            }
            else if (!Segment || mySurface is not Geom_BSplineSurface || !Status(ShapeExtend_DONE2))
            {
                // pdn copying of surface
                var tmp = (Geom_Surface)mySurface.Copy();
                var Surf = new Geom_RectangularTrimmedSurface(tmp, UFirst, ULast, VFirst, VLast);
                SurfacesGrid.SetValue(1, 1, Surf);
            }
            else
            {
                filled = false;
            }
            if (filled)
            {
                myResSurfaces!.Init(SurfacesGrid);
                myResSurfaces.SetUFirstValue(myUSplitValues.Sequence().First());
                myResSurfaces.SetVFirstValue(myVSplitValues.Sequence().First());
                return;
            }
        }

        if (mySurface is Geom_BSplineSurface)
        {
            var BsSurface = (Geom_BSplineSurface)mySurface.Copy();
            int FirstInd = BsSurface.FirstUKnotIndex(), LastInd = BsSurface.LastUKnotIndex();
            int j = FirstInd;
            for (int ii = 1; ii <= myUSplitValues.Length(); ii++)
            {
                double spval = myUSplitValues.Value(ii);
                for (; j <= LastInd; j++)
                {
                    if (spval > BsSurface.UKnot(j) + Occt.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < BsSurface.UKnot(j) - Occt.Precision.PConfusion())
                    {
                        break;
                    }
                    myUSplitValues.ChangeValue(ii) = BsSurface.UKnot(j);
                }
                if (j == LastInd)
                {
                    break;
                }
            }
            FirstInd = BsSurface.FirstVKnotIndex();
            LastInd = BsSurface.LastVKnotIndex();
            j = FirstInd;
            for (int ii1 = 1; ii1 <= myVSplitValues.Length(); ii1++)
            {
                double spval = myVSplitValues.Value(ii1);
                for (; j <= LastInd; j++)
                {
                    if (spval > BsSurface.VKnot(j) + Occt.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < BsSurface.VKnot(j) - Occt.Precision.PConfusion())
                    {
                        break;
                    }
                    myVSplitValues.ChangeValue(ii1) = BsSurface.VKnot(j);
                }
                if (j == LastInd)
                {
                    break;
                }
            }
        }

        U1 = myUSplitValues.Value(1);
        V1 = myVSplitValues.Value(1);
        for (int irow = 2; irow <= myUSplitValues.Length(); irow++)
        {
            U2 = myUSplitValues.Value(irow);
            for (int icol = 2; icol <= myVSplitValues.Length(); icol++)
            {
                V2 = myVSplitValues.Value(icol);
                // creates a copy of theSurf before to segment:
                var theNew = (Geom_Surface)mySurface.Copy();
                if (isBSpline || isBezier)
                {
                    try
                    {
                        if (isBSpline)
                        {
                            ((Geom_BSplineSurface)theNew).Segment(U1, U2, V1, V2);
                        }
                        else if (isBezier)
                        {
                            // pdn K4L+ (work around)
                            // rln C30 (direct use)
                            double u1 = U1;
                            double u2 = U2;
                            double v1 = V1;
                            double v2 = V2;
                            ((Geom_BezierSurface)theNew).Segment(u1, u2, v1, v2);
                        }
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                    }
                    catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                    {
                        // OCCT_DEBUG output left out
                        Geom_Surface theNewSurf = theNew;
                        theNew = new Geom_RectangularTrimmedSurface(theNewSurf, U1, U2, V1, V2);
                    }
                    SurfacesGrid.SetValue((irow - 1), (icol - 1), theNew);
                }
                else
                {
                    // not a BSpline: trimming instead of segmentation
                    var SplittedSurf = new Geom_RectangularTrimmedSurface(theNew, U1, U2, V1, V2);
                    SurfacesGrid.SetValue((irow - 1), (icol - 1), SplittedSurf);
                }

                V1 = V2;
            }
            U1 = U2;
            V1 = myVSplitValues.Value(1);
        }
        int nbU = myUSplitValues.Length();
        var UJoints = new NCollection_Array1<double>(1, nbU);
        int i;
        for (i = 1; i <= nbU; i++)
        {
            UJoints[i] = myUSplitValues.Value(i);
        }

        int nbV = myVSplitValues.Length();
        var VJoints = new NCollection_Array1<double>(1, nbV);
        for (i = 1; i <= nbV; i++)
        {
            VJoints[i] = myVSplitValues.Value(i);
        }
        myResSurfaces!.Init(SurfacesGrid, UJoints, VJoints);
    }

    /// <summary>Calculates the splitting values; the base class only resets the status.</summary>
    public virtual void Compute(bool Segment = true)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Computes the splitting values (<see cref="Compute"/>), then splits the surface (<see cref="Build"/>).</summary>
    public void Perform(bool Segment = true)
    {
        Compute(Segment);
        //  SetUSplitValues(myUSplitValues);
        //  SetVSplitValues(myVSplitValues);
        Build(Segment);
    }

    /// <summary>Returns all the U splitting values, the first and last parameters of the surface included.</summary>
    public NCollection_HSequence<double>? USplitValues() => myUSplitValues;

    /// <summary>Returns all the V splitting values, the first and last parameters of the surface included.</summary>
    public NCollection_HSequence<double>? VSplitValues() => myVSplitValues;

    /// <summary>
    /// Returns the status: OK (no splitting needed), DONE1 (splitting gives more than one patch), DONE2 (splitting
    /// required, a single patch), DONE3 (the surface's form or parametrisation is modified).
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Returns the surfaces obtained by splitting, as a composite surface.</summary>
    public ShapeExtend_CompositeSurface? ResSurfaces() => myResSurfaces;
}
