// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NSections.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A section law through N sections: a B-spline surface interpolating the sections (approximated with
/// GeomFill_AppSurf, or cut from a given reference surface), whose v-isos are the sections.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps its own sequence of the caller's curves, as OCCT copies the sequence of
/// handles. OCCT reparametrises a B-spline section in place (<c>SetKnots</c> on the caller's curve); the port
/// reparametrises a copy and keeps the copy in its own sequence, so the law behaves as OCCT's and the caller's curve
/// is untouched (decisions.md). <see cref="BSplineSurface"/> hands out the law's surface frozen.
/// </remarks>
public class GeomFill_NSections : GeomFill_SectionLaw
{
    private double UFirst;
    private double ULast;
    private double VFirst;
    private double VLast;
    private readonly NCollection_Sequence<Curve3D> mySections = new();
    private readonly NCollection_Sequence<Transform3D> myTrsfs = new();
    private readonly NCollection_Sequence<double> myParams = new();
    private BSplineSurface? mySurface;
    private BSplineSurface? myRefSurf;

    // fonction d'evaluation des poles et des poids de mySurface pour D1 et D2
    private static void ResultEval(BSplineSurface surf, double V, int deriv, NCollection_Array1<double> Result)
    {
        bool rational = surf.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }
        int Cdeg = surf.VDegree(), Cdim = surf.UPoleCount * gap, NbP = surf.VPoleCount;

        //  les noeuds plats
        NCollection_Array1<double> FKnots = surf.InternalVKnotSequence();

        //  les poles
        int Psize = Cdim * NbP;
        NCollection_Array1<double> SPoles = new(1, Psize);
        int ii, jj, ipole = 1;
        for (jj = 1; jj <= NbP; jj++)
        {
            for (ii = 1; ii <= surf.UPoleCount; ii++)
            {
                SPoles[ipole] = surf.Pole(ii, jj).X();
                SPoles[ipole + 1] = surf.Pole(ii, jj).Y();
                SPoles[ipole + 2] = surf.Pole(ii, jj).Z();
                if (rational)
                {
                    SPoles[ipole + 3] = surf.Weight(ii, jj);
                    SPoles[ipole] *= SPoles[ipole + 3];
                    SPoles[ipole + 1] *= SPoles[ipole + 3];
                    SPoles[ipole + 2] *= SPoles[ipole + 3];
                }
                ipole += gap;
            }
        }

        bool periodic_flag = false;
        Span<int> extrap_mode = stackalloc int[2];
        extrap_mode[0] = extrap_mode[1] = Cdeg;
        NCollection_Array1<double> EvalBS = new(1, Cdim * (deriv + 1));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(V, periodic_flag, deriv, extrap_mode, Cdeg, FKnots, Cdim, SPoles.AsSpan(1), EvalBS.AsSpan(1));

        for (ii = 1; ii <= Cdim; ii++)
        {
            Result[ii] = EvalBS[ii + deriv * Cdim];
        }
    }

    /// <summary>The law through the sections NC, with u in [0, 1] and v in [0, 1].</summary>
    public GeomFill_NSections(NCollection_Sequence<Curve3D> NC)
    {
        mySections.Assign(NC);
        UFirst = 0.0;
        ULast = 1.0;
        VFirst = 0.0;
        VLast = 1.0;
        myRefSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP.</summary>
    public GeomFill_NSections(NCollection_Sequence<Curve3D> NC, NCollection_Sequence<double> NP)
    {
        mySections.Assign(NC);
        myParams.Assign(NP);
        UFirst = 0.0;
        ULast = 1.0;
        VFirst = 0.0;
        VLast = 1.0;
        myRefSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP, with u in [theUF, theUL].</summary>
    public GeomFill_NSections(NCollection_Sequence<Curve3D> theNC, NCollection_Sequence<double> theNP,
                              double theUF, double theUL)
    {
        mySections.Assign(theNC);
        myParams.Assign(theNP);
        UFirst = theUF;
        ULast = theUL;
        VFirst = 0.0;
        VLast = 1.0;
        myRefSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP, with u in [UF, UL] and v in [VF, VL].</summary>
    public GeomFill_NSections(NCollection_Sequence<Curve3D> NC, NCollection_Sequence<double> NP, double UF,
                              double UL, double VF, double VL)
    {
        mySections.Assign(NC);
        myParams.Assign(NP);
        UFirst = UF;
        ULast = UL;
        VFirst = VF;
        VLast = VL;
        myRefSurf = null;
        ComputeSurface();
    }

    /// <summary>
    /// The law through the sections NC placed by Trsfs at the parameters NP, cut from the reference surface Surf
    /// (u in [UF, UL], v in [VF, VL]).
    /// </summary>
    public GeomFill_NSections(NCollection_Sequence<Curve3D> NC, NCollection_Sequence<Transform3D> Trsfs,
                              NCollection_Sequence<double> NP, double UF, double UL, double VF, double VL,
                              BSplineSurface? Surf)
    {
        mySections.Assign(NC);
        myTrsfs.Assign(Trsfs);
        myParams.Assign(NP);
        UFirst = UF;
        ULast = UL;
        VFirst = VF;
        VLast = VL;
        myRefSurf = Surf;
        ComputeSurface();
    }

    public override bool D0(double V, NCollection_Array1<Point3D> Poles, NCollection_Array1<double> Weights)
    {
        if (mySurface is null)
        {
            return false;
        }
        else
        {
            BSplineCurve3D Curve = (BSplineCurve3D)mySurface.VIso(V, false);
            NCollection_Array1<Point3D> poles = Curve.InternalPoles();
            NCollection_Array1<double> weights = Curve.InternalWeightsArray();
            int ii, L = Poles.Length();
            for (ii = 1; ii <= L; ii++)
            {
                Poles[ii].SetXYZ(poles[ii].XYZ());
                Weights[ii] = weights[ii];
            }
        }
        return true;
    }

    public override bool D1(double V, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        if (mySurface is null)
        {
            return false;
        }

        bool ok = D0(V, Poles, Weights);
        if (!ok)
        {
            return false;
        }

        int L = Poles.Length(), derivative_request = 1;
        bool rational = mySurface.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int dimResult = mySurface.UPoleCount * gap;
        BSplineSurface? surf_deper = null;
        if (mySurface.IsVPeriodic)
        {
            surf_deper = (BSplineSurface)mySurface.Copy();
            surf_deper.SetVNotPeriodic();
            dimResult = surf_deper.UPoleCount * gap;
        }
        NCollection_Array1<double> Result = new(1, dimResult);
        if (mySurface.IsVPeriodic)
        {
            ResultEval(surf_deper!, V, derivative_request, Result);
        }
        else
        {
            ResultEval(mySurface, V, derivative_request, Result);
        }

        double ww;
        double EpsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool NullWeight = false;
        if (!rational)
        {
            DWeights.Init(0.0);
        }
        int indice = 1, ii;

        //  recopie des poles du resultat sous forme de points 3D et de poids
        for (ii = 1; ii <= L && (!NullWeight); ii++)
        {
            DPoles[ii].SetX(Result[indice]);
            DPoles[ii].SetY(Result[indice + 1]);
            DPoles[ii].SetZ(Result[indice + 2]);
            if (rational)
            {
                ww = Weights[ii];
                if (ww < EpsW)
                {
                    NullWeight = true;
                }
                else
                {
                    DWeights[ii] = Result[indice + 3];
                    DPoles[ii].SetXYZ((DPoles[ii].XYZ() - DWeights[ii] * Poles[ii].Coord()) / ww);
                }
            }
            indice += gap;
        }
        return !NullWeight;
    }

    public override bool D2(double V, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<double> Weights,
                            NCollection_Array1<double> DWeights, NCollection_Array1<double> D2Weights)
    {
        if (mySurface is null)
        {
            return false;
        }

        // pb dans BSplCLib::Eval() pour les surfaces rationnelles de degre 1
        // si l'ordre de derivation est egal a 2.
        if (mySurface.VDegree() < 2)
        {
            return false;
        }

        bool ok = D1(V, Poles, DPoles, Weights, DWeights);
        if (!ok)
        {
            return false;
        }

        int L = Poles.Length(), derivative_request = 2;
        bool rational = mySurface.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int dimResult = mySurface.UPoleCount * gap;
        BSplineSurface? surf_deper = null;
        if (mySurface.IsVPeriodic)
        {
            surf_deper = (BSplineSurface)mySurface.Copy();
            surf_deper.SetVNotPeriodic();
            dimResult = surf_deper.UPoleCount * gap;
        }
        NCollection_Array1<double> Result = new(1, dimResult);
        if (mySurface.IsVPeriodic)
        {
            ResultEval(surf_deper!, V, derivative_request, Result);
        }
        else
        {
            ResultEval(mySurface, V, derivative_request, Result);
        }

        double ww;
        double EpsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool NullWeight = false;
        if (!rational)
        {
            D2Weights.Init(0.0);
        }
        int indice = 1, ii;

        //  recopie des poles du resultat sous forme de points 3D et de poids
        for (ii = 1; ii <= L && (!NullWeight); ii++)
        {
            D2Poles[ii].SetX(Result[indice]);
            D2Poles[ii].SetY(Result[indice + 1]);
            D2Poles[ii].SetZ(Result[indice + 2]);
            if (rational)
            {
                ww = Weights[ii];
                if (ww < EpsW)
                {
                    NullWeight = true;
                }
                else
                {
                    D2Weights[ii] = Result[indice + 3];
                    D2Poles[ii].SetXYZ((D2Poles[ii].XYZ() - D2Weights[ii] * Poles[ii].Coord()
                                        - 2 * DWeights[ii] * DPoles[ii].XYZ())
                                       / ww);
                }
            }
            indice += gap;
        }
        return !NullWeight;
    }

    /// <summary>The law's surface (frozen), null if it couldn't be built (fewer than two sections).</summary>
    public override BSplineSurface? BSplineSurface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The law's surface, not frozen, for ported callers.</summary>
    internal BSplineSurface? InternalBSplineSurface() => mySurface;

    /// <summary>Sets the reference surface used by the next <see cref="ComputeSurface"/>.</summary>
    public void SetSurface(BSplineSurface? RefSurf)
    {
        myRefSurf = RefSurf;
    }

    /// <summary>Builds the law's surface: approximated through the sections, or cut from the reference surface.</summary>
    public void ComputeSurface()
    {
        BSplineSurface BS;
        if (myRefSurf is null)
        {
            double myPres3d = 1.0e-06;
            int i, j, jdeb = 1, jfin = mySections.Length();

            if (jfin <= jdeb)
            {
                // We will not be able to create surface from single curve.
                return;
            }

            GeomFill_SectionGenerator section = new GeomFill_SectionGenerator();

            for (j = jdeb; j <= jfin; j++)
            {
                // read the j-th curve
                Curve3D curv = mySections[j];

                // transformation to BSpline reparametrized to [UFirst,ULast]
                BSplineCurve3D? curvBS = curv as BSplineCurve3D;
                if (curvBS is null)
                {
                    curvBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(curv, QuasiAngular);
                }
                else
                {
                    // OCCT reparametrises the caller's curve in place; the port reparametrises a copy and keeps it in
                    // its own sequence, as OCCT's sequence then holds the reparametrised curve (decisions.md).
                    curvBS = (BSplineCurve3D)curvBS.Copy();
                    mySections[j] = curvBS;
                }

                NCollection_Array1<double> BSK = new(curvBS.InternalKnots());
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(UFirst, ULast, BSK);
                curvBS.SetKnots(BSK);

                section.AddCurve(curvBS);
            }

            int Nbcurves = mySections.Length();
            int Nbpar = myParams.Length();
            if (Nbpar > 0)
            {
                NCollection_HArray1<double> HPar = new NCollection_HArray1<double>(1, Nbpar);
                for (i = 1; i <= Nbpar; i++)
                {
                    HPar.SetValue(i, myParams[i]);
                }
                section.SetParam(HPar);
            }
            section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

            GeomFill_Line line = new GeomFill_Line(Nbcurves);
            int nbIt = 0, degmin = 2, degmax = 6;
            bool knownP = Nbpar > 0;
            GeomFill_AppSurf anApprox = new GeomFill_AppSurf(degmin, degmax, myPres3d, myPres3d, nbIt, knownP);
            anApprox.SetContinuity(C1);
            bool SpApprox = true;
            anApprox.Perform(line, section, SpApprox);

            BS = new BSplineSurface(anApprox.SurfPoles(),
                                         anApprox.SurfWeights(),
                                         anApprox.SurfUKnots(),
                                         anApprox.SurfVKnots(),
                                         anApprox.SurfUMults(),
                                         anApprox.SurfVMults(),
                                         anApprox.UDegree(),
                                         anApprox.VDegree(),
                                         section.IsPeriodic());
        }

        else
        {
            // segmentation de myRefSurf
            double Ui1, Ui2, V0, V1;
            BS = (BSplineSurface)myRefSurf.Copy();
            Ui1 = UFirst;
            Ui2 = ULast;
            int i1, i2;
            myRefSurf.LocateU(Ui1, global::Kerf.Engine.Foundation.Precision.PConfusion(), out i1, out i2);
            if (Math.Abs(Ui1 - myRefSurf.UKnot(i1)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Ui1 = myRefSurf.UKnot(i1);
            }
            if (Math.Abs(Ui1 - myRefSurf.UKnot(i2)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Ui1 = myRefSurf.UKnot(i2);
            }
            myRefSurf.LocateU(Ui2, global::Kerf.Engine.Foundation.Precision.PConfusion(), out i1, out i2);
            if (Math.Abs(Ui2 - myRefSurf.UKnot(i1)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Ui2 = myRefSurf.UKnot(i1);
            }
            if (Math.Abs(Ui2 - myRefSurf.UKnot(i2)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Ui2 = myRefSurf.UKnot(i2);
            }
            V0 = myRefSurf.VKnot(myRefSurf.FirstVKnotIndex());
            V1 = myRefSurf.VKnot(myRefSurf.LastVKnotIndex());
            BS.CheckAndSegment(Ui1, Ui2, V0, V1);
        }
        mySurface = BS;
        // On augmente le degre pour que le positionnement D2 soit correct
        if (mySurface.VDegree() < 2)
        {
            mySurface.IncreaseDegree(mySurface.UDegree(), 2);
        }
    }

    public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
    {
        // OCCT leaves the outputs unwritten when there is no surface; they are 0 here.
        NbPoles = NbKnots = Degree = 0;
        if (mySurface is null)
        {
            return;
        }

        NbPoles = mySurface.UPoleCount;
        NbKnots = mySurface.UKnotCount;
        Degree = mySurface.UDegree();
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        if (mySurface is not null)
        {
            TKnots.Assign(mySurface.InternalUKnots());
        }
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        if (mySurface is not null)
        {
            TMults.Assign(mySurface.InternalUMultiplicities());
        }
    }

    public override bool IsRational()
    {
        if (mySurface is not null)
        {
            return mySurface.IsURational;
        }

        return false;
    }

    public override bool IsUPeriodic()
    {
        if (mySurface is not null)
        {
            return mySurface.IsUPeriodic;
        }

        return false;
    }

    public override bool IsVPeriodic()
    {
        if (mySurface is not null)
        {
            return mySurface.IsVPeriodic;
        }

        return false;
    }

    public override int NbIntervals(Continuity S)
    {
        if (mySurface is null)
        {
            return 0;
        }

        SurfaceAdaptor AdS = new SurfaceAdaptor(mySurface);
        return AdS.NbVIntervals(S);
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        if (mySurface is null)
        {
            return;
        }

        SurfaceAdaptor AdS = new SurfaceAdaptor(mySurface);
        AdS.VIntervals(T, S);
    }

    public override void SetInterval(double First, double Last)
    {
        // rien a faire : mySurface est supposee Cn en V
    }

    public override void GetInterval(out double F, out double L)
    {
        F = VFirst;
        L = VLast;
    }

    public override void GetDomain(out double F, out double L)
    {
        F = VFirst;
        L = VLast;
    }

    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, NCollection_Array1<double> Tol3d)
    {
        Tol3d.Init(SurfTol);
        if (BoundTol < SurfTol)
        {
            Tol3d[Tol3d.Lower()] = BoundTol;
            Tol3d[Tol3d.Upper()] = BoundTol;
        }
    }

    public override Point3D BarycentreOfSurf()
    {
        Point3D P;
        Point3D Bary = new Point3D();
        Bary.SetCoord(0.0, 0.0, 0.0);

        if (mySurface is null)
        {
            return Bary;
        }

        int ii, jj;
        double U0, U1, V0, V1;
        mySurface.Bounds(out U0, out U1, out V0, out V1);
        double V = V0, DeltaV = (V1 - V0) / 20;
        // OCCT: U is not reset for each V, so only the first row samples the surface's u range.
        double U = U0, DeltaU = (U1 - U0) / 20;
        for (jj = 0; jj <= 20; jj++, V += DeltaV)
        {
            for (ii = 0; ii <= 20; ii++, U += DeltaU)
            {
                P = mySurface.Value(U, V);
                Bary.ChangeCoord() += P.XYZ();
            }
        }

        Bary.ChangeCoord() /= (21 * 21);
        return Bary;
    }

    public override double MaximalSection()
    {
        double L, Lmax = 0.0;
        int ii;
        for (ii = 1; ii <= mySections.Length(); ii++)
        {
            CurveAdaptor AC = new CurveAdaptor(mySections[ii]);
            L = AbscissaPoint.Length(AC);
            if (L > Lmax)
            {
                Lmax = L;
            }
        }
        return Lmax;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weights)
    {
        if (mySurface is null)
        {
            return;
        }

        if (mySurface.IsURational)
        {
            int NbU = mySurface.UPoleCount, NbV = mySurface.VPoleCount;
            NCollection_Array2<double>? aWSurfPtr = mySurface.InternalWeights();
            if (aWSurfPtr is null)
            {
                Weights.Init(1);
                return;
            }
            NCollection_Array2<double> WSurf = aWSurfPtr;
            int i, j;
            for (i = 1; i <= NbU; i++)
            {
                double min = WSurf[i, 1];
                for (j = 2; j <= NbV; j++)
                {
                    if (min > WSurf[i, j])
                    {
                        min = WSurf[i, j];
                    }
                }
                Weights.SetValue(i, min);
            }
        }
        else
        {
            Weights.Init(1);
        }
    }

    public override bool IsConstant(ref double Error)
    {
        // on se limite a 2 sections
        bool isconst = (mySections.Length() == 2);
        double Err = 0.0;

        if (isconst)
        {
            CurveAdaptor AC1 = new CurveAdaptor(mySections[1]);
            CurveKind CType = AC1.GetType();
            CurveAdaptor AC2 = new CurveAdaptor(mySections[2]);
            // les sections doivent avoir le meme type
            isconst = (AC2.GetType() == CType);

            if (isconst)
            {
                if (CType == Circle)
                {
                    Circle3D C1 = AC1.Circle();
                    Circle3D C2 = AC2.Circle();
                    double Tol = 1.0e-7;
                    bool samedir, samerad, samepos;
                    samedir = (C1.Axis().IsParallel(C2.Axis(), 1.0e-4));
                    samerad = (Math.Abs(C1.Radius() - C2.Radius()) < Tol);
                    samepos = (C1.Location().Distance(C2.Location()) < Tol);
                    if (!samepos)
                    {
                        Axis3D D = new Axis3D(C1.Location(), new Direction3D(new Vector3D(C1.Location(), C2.Location())));
                        samepos = (C1.Axis().IsParallel(D, 1.0e-4));
                    }
                    isconst = samedir && samerad && samepos;
                }
                else if (CType == Line)
                {
                    Line3D L1 = AC1.Line();
                    Line3D L2 = AC2.Line();
                    double Tol = 1.0e-7;
                    bool samedir, samelength, samepos;
                    samedir = (L1.Direction().IsParallel(L2.Direction(), 1.0e-4));
                    Point3D P11 = AC1.Value(AC1.FirstParameter()), P12 = AC1.Value(AC1.LastParameter()),
                           P21 = AC2.Value(AC2.FirstParameter()), P22 = AC2.Value(AC2.LastParameter());
                    samelength = (Math.Abs(P11.Distance(P12) - P21.Distance(P22)) < Tol);
                    // l'ecart entre les 2 sections ne compte pas
                    samepos = ((P11.Distance(P21) < Tol && P12.Distance(P22) < Tol)
                               || (P12.Distance(P21) < Tol && P11.Distance(P22) < Tol));
                    // samepos = true;
                    isconst = samedir && samelength && samepos;
                }
                else
                {
                    isconst = false;
                }
            }
        }

        Error = Err;
        return isconst;
    }

    /// <summary>A new copy of the first section.</summary>
    public override Curve3D ConstantSection()
    {
        //  double Err;
        //  if (!IsConstant(Err)) throw StdFail_NotDone("The Law is not Constant!");
        Curve3D C;
        C = (Curve3D)mySections[1].Copy();
        return C;
    }

    public override bool IsConicalLaw(ref double Error)
    {
        bool isconic = (mySections.Length() == 2);
        double Err = 0.0;
        if (isconic)
        {
            CurveAdaptor AC1 = new CurveAdaptor(mySections[1]);
            CurveAdaptor AC2 = new CurveAdaptor(mySections[2]);
            isconic = (AC1.GetType() == Circle) && (AC2.GetType() == Circle);
            if (isconic)
            {
                Circle3D C1 = AC1.Circle();
                if (!myTrsfs.IsEmpty())
                {
                    C1.Transform(myTrsfs[1].Inverted());
                }
                Circle3D C2 = AC2.Circle();
                if (!myTrsfs.IsEmpty())
                {
                    C2.Transform(myTrsfs[2].Inverted());
                }
                double Tol = 1.0e-7;
                // bool samedir, linearrad, sameaxis;
                isconic = (C1.Axis().IsParallel(C2.Axis(), 1.0e-4));
                //  pour 2 sections, la variation du rayon est forcement lineaire
                if (isconic)
                {
                    Line3D Line1 = new Line3D(C1.Axis());
                    isconic = (Line1.Distance(C2.Location()) < Tol);
                    if (isconic)
                    {
                        //// Modified by jgv, 18.02.2009 for OCC20866 ////
                        double first1 = AC1.FirstParameter(), last1 = AC1.LastParameter();
                        double first2 = AC2.FirstParameter(), last2 = AC2.LastParameter();
                        isconic = (Math.Abs(first1 - first2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                   && Math.Abs(last1 - last2) <= global::Kerf.Engine.Foundation.Precision.PConfusion());
                        //////////////////////////////////////////////////
                    }
                }
            }
        }

        Error = Err;
        return isconic;
    }

    /// <summary>The circle section at V (new geometry); throws StdFail_NotDone if the law is not conical.</summary>
    public override Curve3D CirclSection(double V)
    {
        double Err = 0;
        if (!IsConicalLaw(ref Err))
        {
            throw new NotDoneException("The Law is not Conical!");
        }

        CurveAdaptor AC1 = new CurveAdaptor(mySections[1]);
        CurveAdaptor AC2 = new CurveAdaptor(mySections[mySections.Length()]);
        Circle3D C1 = AC1.Circle();
        Circle3D C2 = AC2.Circle();

        double p1 = myParams[1], p2 = myParams[myParams.Length()];
        double radius = (C2.Radius() - C1.Radius()) * (V - p1) / (p2 - p1) + C1.Radius();

        C1.SetRadius(radius);
        Curve3D C = new CircleCurve3D(C1);

        double aParF = AC1.FirstParameter();
        double aParL = AC1.LastParameter();
        double aPeriod = AC1.IsPeriodic ? AC1.Period() : 0.0;

        if ((aPeriod == 0.0) || (Math.Abs(aParL - aParF - aPeriod) > global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            Curve3D Cbis = new TrimmedCurve3D(C, aParF, aParL);
            C = Cbis;
        }
        return C;
    }
}
