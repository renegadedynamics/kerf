// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Patch.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Patch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AdvApp2Var_CriterionType;

namespace Occt;

/// <summary>
/// A patch [Ui, Ui+1] x [Vj, Vj+1] of <see cref="AdvApp2Var_ApproxAFunc2Var"/>: the discretisation of F on it, and
/// its polynomial approximation with the errors.
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 194). The arrays it returns are
/// its own (shared: don't modify); they are null until the patch is approximated.
/// </remarks>
public class AdvApp2Var_Patch : Standard_Transient
{
    private double myU0;
    private double myU1;
    private double myV0;
    private double myV1;
    private readonly int myOrdInU;
    private readonly int myOrdInV;
    private int myNbCoeffInU;
    private int myNbCoeffInV;
    private bool myApprIsDone;
    private bool myHasResult;
    private NCollection_HArray1<double>? myEquation;
    private NCollection_HArray1<double>? myMaxErrors;
    private NCollection_HArray1<double>? myMoyErrors;
    private NCollection_HArray2<double>? myIsoErrors;
    private int myCutSense;
    private bool myDiscIsDone;
    private NCollection_HArray1<double>? mySosoTab;
    private NCollection_HArray1<double>? myDisoTab;
    private NCollection_HArray1<double>? mySodiTab;
    private NCollection_HArray1<double>? myDidiTab;
    private double myCritValue;

    public AdvApp2Var_Patch()
    {
        myU0 = 0.0;
        myU1 = 1.0;
        myV0 = 0.0;
        myV1 = 1.0;
        myOrdInU = 0;
        myOrdInV = 0;
        myNbCoeffInU = 0;
        myNbCoeffInV = 0;
        myApprIsDone = false;
        myHasResult = false;
        myCutSense = 0;
        myDiscIsDone = false;
        myCritValue = 0.0;
    }

    public AdvApp2Var_Patch(double U0, double U1, double V0, double V1, int iu, int iv)
    {
        myU0 = U0;
        myU1 = U1;
        myV0 = V0;
        myV1 = V1;
        myOrdInU = iu;
        myOrdInV = iv;
        myNbCoeffInU = 0;
        myNbCoeffInV = 0;
        myApprIsDone = false;
        myHasResult = false;
        myCutSense = 0;
        myDiscIsDone = false;
        myCritValue = 0.0;
    }

    public bool IsDiscretised() => myDiscIsDone;

    /// <summary>Discretises F on the patch, less the interpolation of its constraints.</summary>
    public void Discretise(AdvApp2Var_Context Conditions,
                           AdvApp2Var_Framework Constraints,
                           AdvApp2Var_EvaluatorFunc2Var Func)
    {
        // data stored in the Context
        int NDIMEN, ISOFAV;
        NDIMEN = Conditions.TotalDimension();
        // Attention : works only for 3D
        ISOFAV = Conditions.FavorIso();

        // data related to the patch to be discretized
        int NBPNTU, NBPNTV;
        int IORDRU = myOrdInU, IORDRV = myOrdInV;
        NCollection_HArray1<double> HUROOT = Conditions.URoots()!;
        NCollection_HArray1<double> HVROOT = Conditions.VRoots()!;
        Span<double> UROOT = HUROOT.AsSpan();
        NBPNTU = Conditions.URoots()!.Length();
        if (myOrdInU > -1)
        {
            NBPNTU -= 2;
        }

        Span<double> VROOT = HVROOT.AsSpan();
        NBPNTV = Conditions.VRoots()!.Length();
        if (myOrdInV > -1)
        {
            NBPNTV -= 2;
        }

        // data stored in the Framework Constraints cad Nodes and Isos
        // C1, C2, C3 and C4 are dimensionnes in FORTRAN with (NDIMEN,IORDRU+2,IORDRV+2)
        int SIZE = NDIMEN * (IORDRU + 2) * (IORDRV + 2);
        NCollection_HArray1<double> HCOINS = new(1, SIZE * 4);
        HCOINS.Init(0.0);

        int iu, iv;
        double du = (myU1 - myU0) / 2, dv = (myV1 - myV0) / 2, rho, valnorm;

        for (iu = 0; iu <= myOrdInU; iu++)
        {
            for (iv = 0; iv <= myOrdInV; iv++)
            {
                // factor of normalization
                rho = Math.Pow(du, iu) * Math.Pow(dv, iv);

                // F(U0,V0) and its derivatives normalized on (-1,1)
                valnorm = rho * Constraints.Node(myU0, myV0).Point(iu, iv).X();
                HCOINS.SetValue(1 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU0, myV0).Point(iu, iv).Y();
                HCOINS.SetValue(2 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU0, myV0).Point(iu, iv).Z();
                HCOINS.SetValue(3 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);

                // F(U1,V0) and its derivatives normalized on (-1,1)
                valnorm = rho * Constraints.Node(myU1, myV0).Point(iu, iv).X();
                HCOINS.SetValue(SIZE + 1 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU1, myV0).Point(iu, iv).Y();
                HCOINS.SetValue(SIZE + 2 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU1, myV0).Point(iu, iv).Z();
                HCOINS.SetValue(SIZE + 3 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);

                // F(U0,V1) and its derivatives normalized on (-1,1)
                valnorm = rho * Constraints.Node(myU0, myV1).Point(iu, iv).X();
                HCOINS.SetValue(2 * SIZE + 1 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU0, myV1).Point(iu, iv).Y();
                HCOINS.SetValue(2 * SIZE + 2 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU0, myV1).Point(iu, iv).Z();
                HCOINS.SetValue(2 * SIZE + 3 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);

                // F(U1,V1) and its derivatives normalized on (-1,1)
                valnorm = rho * Constraints.Node(myU1, myV1).Point(iu, iv).X();
                HCOINS.SetValue(3 * SIZE + 1 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU1, myV1).Point(iu, iv).Y();
                HCOINS.SetValue(3 * SIZE + 2 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
                valnorm = rho * Constraints.Node(myU1, myV1).Point(iu, iv).Z();
                HCOINS.SetValue(3 * SIZE + 3 + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv, valnorm);
            }
        }
        F2cPtr<double> C1 = HCOINS.AsSpan();
        F2cPtr<double> C2 = C1 + SIZE;
        F2cPtr<double> C3 = C2 + SIZE;
        F2cPtr<double> C4 = C3 + SIZE;

        // tables SomTab and Diftab of discretization of isos U=U0 and U=U1
        // SU0, SU1, DU0 and DU1 are dimensioned in FORTRAN to
        // (1+NBPNTV/2)*NDIMEN*(IORDRU+1)

        SIZE = (1 + NBPNTV / 2) * NDIMEN;

        NCollection_HArray1<double> HSU0 = new(1, SIZE * (IORDRU + 1));
        HSU0.ChangeArray1().Assign(Constraints.IsoU(myU0, myV0, myV1).SomTab()!.Array1());

        NCollection_HArray1<double> HDU0 = new(1, SIZE * (IORDRU + 1));
        HDU0.ChangeArray1().Assign(Constraints.IsoU(myU0, myV0, myV1).DifTab()!.Array1());

        NCollection_HArray1<double> HSU1 = new(1, SIZE * (IORDRU + 1));
        HSU1.ChangeArray1().Assign(Constraints.IsoU(myU1, myV0, myV1).SomTab()!.Array1());

        NCollection_HArray1<double> HDU1 = new(1, SIZE * (IORDRU + 1));
        HDU1.ChangeArray1().Assign(Constraints.IsoU(myU1, myV0, myV1).DifTab()!.Array1());

        // normalization
        int ideb1, ideb2, ideb3, ideb4, jj;
        for (iu = 1; iu <= IORDRU; iu++)
        {
            rho = Math.Pow(du, iu);
            ideb1 = HSU0.Lower() + iu * SIZE - 1;
            ideb2 = HDU0.Lower() + iu * SIZE - 1;
            ideb3 = HSU1.Lower() + iu * SIZE - 1;
            ideb4 = HDU1.Lower() + iu * SIZE - 1;
            for (jj = 1; jj <= SIZE; jj++)
            {
                HSU0.SetValue(ideb1 + jj, rho * HSU0.Value(ideb1 + jj));
                HDU0.SetValue(ideb2 + jj, rho * HDU0.Value(ideb2 + jj));
                HSU1.SetValue(ideb3 + jj, rho * HSU1.Value(ideb3 + jj));
                HDU1.SetValue(ideb4 + jj, rho * HDU1.Value(ideb4 + jj));
            }
        }

        Span<double> SU0 = HSU0.AsSpan();
        Span<double> DU0 = HDU0.AsSpan();
        Span<double> SU1 = HSU1.AsSpan();
        Span<double> DU1 = HDU1.AsSpan();

        // tables SomTab and Diftab of discretization of isos V=V0 and V=V1
        // SU0, SU1, DU0 and DU1 are dimensioned in FORTRAN at
        // (1+NBPNTU/2)*NDIMEN*(IORDRV+1)

        SIZE = (1 + NBPNTU / 2) * NDIMEN;

        NCollection_HArray1<double> HSV0 = new(1, SIZE * (IORDRV + 1));
        HSV0.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV0).SomTab()!.Array1());

        NCollection_HArray1<double> HDV0 = new(1, SIZE * (IORDRV + 1));
        HDV0.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV0).DifTab()!.Array1());

        NCollection_HArray1<double> HSV1 = new(1, SIZE * (IORDRV + 1));
        HSV1.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV1).SomTab()!.Array1());

        NCollection_HArray1<double> HDV1 = new(1, SIZE * (IORDRV + 1));
        HDV1.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV1).DifTab()!.Array1());

        // normalisation
        for (iv = 1; iv <= IORDRV; iv++)
        {
            rho = Math.Pow(dv, iv);
            ideb1 = HSV0.Lower() + iv * SIZE - 1;
            ideb2 = HDV0.Lower() + iv * SIZE - 1;
            ideb3 = HSV1.Lower() + iv * SIZE - 1;
            ideb4 = HDV1.Lower() + iv * SIZE - 1;
            for (jj = 1; jj <= SIZE; jj++)
            {
                HSV0.SetValue(ideb1 + jj, rho * HSV0.Value(ideb1 + jj));
                HDV0.SetValue(ideb2 + jj, rho * HDV0.Value(ideb2 + jj));
                HSV1.SetValue(ideb3 + jj, rho * HSV1.Value(ideb3 + jj));
                HDV1.SetValue(ideb4 + jj, rho * HDV1.Value(ideb4 + jj));
            }
        }

        Span<double> SV0 = HSV0.AsSpan();
        Span<double> DV0 = HDV0.AsSpan();
        Span<double> SV1 = HSV1.AsSpan();
        Span<double> DV1 = HDV1.AsSpan();

        // SOSOTB and DIDITB are dimensioned in FORTRAN at
        // (0:NBPNTU/2,0:NBPNTV/2,NDIMEN)

        SIZE = (1 + NBPNTU / 2) * (1 + NBPNTV / 2) * NDIMEN;

        NCollection_HArray1<double> HSOSO = new(1, SIZE);
        Span<double> SOSOTB = HSOSO.AsSpan();
        HSOSO.Init(0.0);
        NCollection_HArray1<double> HDIDI = new(1, SIZE);
        Span<double> DIDITB = HDIDI.AsSpan();
        HDIDI.Init(0.0);

        // SODITB and DISOTB are dimensioned in FORTRAN at
        // (1:NBPNTU/2,1:NBPNTV/2,NDIMEN)

        SIZE = (NBPNTU / 2) * (NBPNTV / 2) * NDIMEN;

        NCollection_HArray1<double> HSODI = new(1, SIZE);
        Span<double> SODITB = HSODI.AsSpan();
        HSODI.Init(0.0);
        NCollection_HArray1<double> HDISO = new(1, SIZE);
        Span<double> DISOTB = HDISO.AsSpan();
        HDISO.Init(0.0);

        int IERCOD = 0;

        //  discretization of polynoms of interpolation
        AdvApp2Var_ApproxF2var.mma2cdi_(NDIMEN,
                                        NBPNTU,
                                        UROOT,
                                        NBPNTV,
                                        VROOT,
                                        IORDRU,
                                        IORDRV,
                                        C1,
                                        C2,
                                        C3,
                                        C4,
                                        SU0,
                                        SU1,
                                        DU0,
                                        DU1,
                                        SV0,
                                        SV1,
                                        DV0,
                                        DV1,
                                        SOSOTB,
                                        SODITB,
                                        DISOTB,
                                        DIDITB,
                                        ref IERCOD);

        //  discretization of the square
        Span<double> UDBFN = stackalloc double[2];
        Span<double> VDBFN = stackalloc double[2];
        UDBFN[0] = myU0;
        UDBFN[1] = myU1;
        VDBFN[0] = myV0;
        VDBFN[1] = myV1;

        SIZE = Max(NBPNTU, NBPNTV);
        NCollection_HArray1<double> HTABLE = new(1, SIZE);
        Span<double> TAB = HTABLE.AsSpan();

        NCollection_HArray1<double> HPOINTS = new(1, SIZE * NDIMEN);
        Span<double> PTS = HPOINTS.AsSpan();

        AdvApp2Var_ApproxF2var.mma2ds1_(NDIMEN,
                                        UDBFN,
                                        VDBFN,
                                        Func,
                                        NBPNTU,
                                        NBPNTV,
                                        UROOT,
                                        VROOT,
                                        ISOFAV,
                                        SOSOTB,
                                        DISOTB,
                                        SODITB,
                                        DIDITB,
                                        PTS,
                                        TAB,
                                        ref IERCOD);

        // the results are stored
        if (IERCOD == 0)
        {
            myDiscIsDone = true;
            mySosoTab = HSOSO;
            myDisoTab = HDISO;
            mySodiTab = HSODI;
            myDidiTab = HDIDI;
        }
        else
        {
            myDiscIsDone = false;
        }
    }

    public bool HasResult() => myHasResult;

    public bool IsApproximated() => myApprIsDone;

    /// <summary>Adds the interpolation of the constraints (isos and nodes) to the patch's polynomial.</summary>
    /// <exception cref="Standard_ConstructionError">An error in the Fortran.</exception>
    public void AddConstraints(AdvApp2Var_Context Conditions, AdvApp2Var_Framework Constraints)
    {
        // data stored in the  Context
        int NDIMEN;
        int IERCOD = 0, NCFLMU, NCFLMV, NDegU, NDegV;
        NDIMEN = Conditions.TotalDimension();
        // Attention : works only for 3D
        NCFLMU = Conditions.ULimit();
        NCFLMV = Conditions.VLimit();
        NDegU = NCFLMU - 1;
        NDegV = NCFLMV - 1;

        // data relative to the patch
        int IORDRU = myOrdInU, IORDRV = myOrdInV;
        Span<double> PATCAN = myEquation!.AsSpan();

        // curves of approximation of Isos U
        int SIZE = NCFLMV * NDIMEN;
        NCollection_HArray1<double> HIsoU0 = new(1, SIZE * (IORDRU + 1));
        HIsoU0.ChangeArray1().Assign(Constraints.IsoU(myU0, myV0, myV1).Polynom()!.Array1());
        F2cPtr<double> IsoU0 = HIsoU0.AsSpan();
        NCollection_HArray1<int> HCFU0 = new(1, IORDRU + 1);
        Span<int> NCFU0 = HCFU0.AsSpan();
        HCFU0.Init(Constraints.IsoU(myU0, myV0, myV1).NbCoeff());

        NCollection_HArray1<double> HIsoU1 = new(1, SIZE * (IORDRU + 1));
        HIsoU1.ChangeArray1().Assign(Constraints.IsoU(myU1, myV0, myV1).Polynom()!.Array1());
        F2cPtr<double> IsoU1 = HIsoU1.AsSpan();
        NCollection_HArray1<int> HCFU1 = new(1, IORDRU + 1);
        Span<int> NCFU1 = HCFU1.AsSpan();
        HCFU1.Init(Constraints.IsoU(myU1, myV0, myV1).NbCoeff());

        // normalization of Isos U
        int iu, iv;
        double du = (myU1 - myU0) / 2, dv = (myV1 - myV0) / 2, rho, valnorm;
        int ideb0, ideb1, jj;

        for (iu = 1; iu <= IORDRU; iu++)
        {
            rho = Math.Pow(du, iu);
            ideb0 = HIsoU0.Lower() + iu * SIZE - 1;
            ideb1 = HIsoU1.Lower() + iu * SIZE - 1;
            for (jj = 1; jj <= SIZE; jj++)
            {
                HIsoU0.SetValue(ideb0 + jj, rho * HIsoU0.Value(ideb0 + jj));
                HIsoU1.SetValue(ideb1 + jj, rho * HIsoU1.Value(ideb1 + jj));
            }
        }

        // curves of approximation of Isos V
        SIZE = NCFLMU * NDIMEN;
        NCollection_HArray1<double> HIsoV0 = new(1, SIZE * (IORDRV + 1));
        HIsoV0.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV0).Polynom()!.Array1());
        F2cPtr<double> IsoV0 = HIsoV0.AsSpan();
        NCollection_HArray1<int> HCFV0 = new(1, IORDRV + 1);
        Span<int> NCFV0 = HCFV0.AsSpan();
        HCFV0.Init(Constraints.IsoV(myU0, myU1, myV0).NbCoeff());

        NCollection_HArray1<double> HIsoV1 = new(1, SIZE * (IORDRV + 1));
        HIsoV1.ChangeArray1().Assign(Constraints.IsoV(myU0, myU1, myV1).Polynom()!.Array1());
        F2cPtr<double> IsoV1 = HIsoV1.AsSpan();
        NCollection_HArray1<int> HCFV1 = new(1, IORDRV + 1);
        Span<int> NCFV1 = HCFV1.AsSpan();
        HCFV1.Init(Constraints.IsoV(myU0, myU1, myV1).NbCoeff());

        //  normalization of Isos V
        for (iv = 1; iv <= IORDRV; iv++)
        {
            rho = Math.Pow(dv, iv);
            ideb0 = HIsoV0.Lower() + iv * SIZE - 1;
            ideb1 = HIsoV1.Lower() + iv * SIZE - 1;
            for (jj = 1; jj <= SIZE; jj++)
            {
                HIsoV0.SetValue(ideb0 + jj, rho * HIsoV0.Value(ideb0 + jj));
                HIsoV1.SetValue(ideb1 + jj, rho * HIsoV1.Value(ideb1 + jj));
            }
        }

        // add constraints to constant V
        NCollection_HArray1<double> HHERMV = new(1, (2 * IORDRV + 2) * (2 * IORDRV + 2));
        Span<double> HermV = HHERMV.AsSpan();
        if (IORDRV >= 0)
        {
            AdvApp2Var_ApproxF2var.mma1her_(IORDRV, HermV, ref IERCOD);
            if (IERCOD != 0)
            {
                throw new Standard_ConstructionError("AdvApp2Var_Patch::AddConstraints : Error in FORTRAN");
            }
            AdvApp2Var_ApproxF2var.mma2ac2_(NDIMEN, NDegU, NDegV, IORDRV, NCFLMU, NCFV0, IsoV0, NCFV1, IsoV1, HermV,
                                            PATCAN);
        }

        // add constraints to constant U
        NCollection_HArray1<double> HHERMU = new(1, (2 * IORDRU + 2) * (2 * IORDRU + 2));
        Span<double> HermU = HHERMU.AsSpan();
        if (IORDRU >= 0)
        {
            AdvApp2Var_ApproxF2var.mma1her_(IORDRU, HermU, ref IERCOD);
            if (IERCOD != 0)
            {
                throw new Standard_ConstructionError("AdvApp2Var_Patch::AddConstraints : Error in FORTRAN");
            }
            AdvApp2Var_ApproxF2var.mma2ac3_(NDIMEN, NDegU, NDegV, IORDRU, NCFLMV, NCFU0, IsoU0, NCFU1, IsoU1, HermU,
                                            PATCAN);
        }

        // add constraints at the corners
        int ideb;
        SIZE = NDIMEN * (IORDRU + 2) * (IORDRV + 2);
        NCollection_HArray1<double> HCOINS = new(1, SIZE * 4);

        for (iu = 0; iu <= myOrdInU; iu++)
        {
            for (iv = 0; iv <= myOrdInV; iv++)
            {
                rho = Math.Pow(du, iu) * Math.Pow(dv, iv);

                // -F(U0,V0) and its derivatives normalized on (-1,1)
                ideb = HCOINS.Lower() + NDIMEN * iu + NDIMEN * (IORDRU + 2) * iv - 1;
                valnorm = -rho * Constraints.Node(myU0, myV0).Point(iu, iv).X();
                HCOINS.SetValue(1 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU0, myV0).Point(iu, iv).Y();
                HCOINS.SetValue(2 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU0, myV0).Point(iu, iv).Z();
                HCOINS.SetValue(3 + ideb, valnorm);

                // -F(U1,V0) and its derivatives normalized on (-1,1)
                ideb += SIZE;
                valnorm = -rho * Constraints.Node(myU1, myV0).Point(iu, iv).X();
                HCOINS.SetValue(1 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU1, myV0).Point(iu, iv).Y();
                HCOINS.SetValue(2 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU1, myV0).Point(iu, iv).Z();
                HCOINS.SetValue(3 + ideb, valnorm);

                // -F(U0,V1) and its derivatives normalized on (-1,1)
                ideb += SIZE;
                valnorm = -rho * Constraints.Node(myU0, myV1).Point(iu, iv).X();
                HCOINS.SetValue(1 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU0, myV1).Point(iu, iv).Y();
                HCOINS.SetValue(2 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU0, myV1).Point(iu, iv).Z();
                HCOINS.SetValue(3 + ideb, valnorm);

                // -F(U1,V1) and its derivatives normalized on (-1,1)
                ideb += SIZE;
                valnorm = -rho * Constraints.Node(myU1, myV1).Point(iu, iv).X();
                HCOINS.SetValue(1 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU1, myV1).Point(iu, iv).Y();
                HCOINS.SetValue(2 + ideb, valnorm);
                valnorm = -rho * Constraints.Node(myU1, myV1).Point(iu, iv).Z();
                HCOINS.SetValue(3 + ideb, valnorm);
            }
        }

        //  tables required for FORTRAN
        int IORDMX = Max(IORDRU, IORDRV);
        NCollection_HArray1<double> HEXTR = new(1, 2 * IORDMX + 2);
        Span<double> EXTR = HEXTR.AsSpan();
        NCollection_HArray1<double> HFACT = new(1, IORDMX + 1);
        Span<double> FACT = HFACT.AsSpan();

        int idim, ncf0, ncf1, iun = 1;
        F2cPtr<double> Is;

        // add extremities of isos U
        for (iu = 1; iu <= IORDRU + 1; iu++)
        {
            ncf0 = HCFU0.Value(HCFU0.Lower() + iu - 1);
            ncf1 = HCFU1.Value(HCFU1.Lower() + iu - 1);
            for (idim = 1; idim <= NDIMEN; idim++)
            {
                Is = IsoU0 + (NCFLMV * (idim - 1) + NCFLMV * NDIMEN * (iu - 1));
                AdvApp2Var_MathBase.mmdrc11_(IORDRV, iun, ncf0, Is, EXTR, FACT);
                for (iv = 1; iv <= IORDRV + 1; iv++)
                {
                    ideb = HCOINS.Lower() + NDIMEN * (iu - 1) + NDIMEN * (IORDRU + 2) * (iv - 1) - 1;
                    HCOINS.ChangeValue(idim + ideb) += HEXTR.Value(1 + 2 * (iv - 1));
                    HCOINS.ChangeValue(2 * SIZE + idim + ideb) += HEXTR.Value(2 + 2 * (iv - 1));
                }
                Is = IsoU1 + (NCFLMV * (idim - 1) + NCFLMV * NDIMEN * (iu - 1));
                AdvApp2Var_MathBase.mmdrc11_(IORDRV, iun, ncf1, Is, EXTR, FACT);
                for (iv = 1; iv <= IORDRV + 1; iv++)
                {
                    ideb = HCOINS.Lower() + NDIMEN * (iu - 1) + NDIMEN * (IORDRU + 2) * (iv - 1) - 1;
                    HCOINS.ChangeValue(SIZE + idim + ideb) += HEXTR.Value(1 + 2 * (iv - 1));
                    HCOINS.ChangeValue(3 * SIZE + idim + ideb) += HEXTR.Value(2 + 2 * (iv - 1));
                }
            }
        }

        // add extremities of isos V
        for (iv = 1; iv <= IORDRV + 1; iv++)
        {
            ncf0 = HCFV0.Value(HCFV0.Lower() + iv - 1);
            ncf1 = HCFV1.Value(HCFV1.Lower() + iv - 1);
            for (idim = 1; idim <= NDIMEN; idim++)
            {
                Is = IsoV0 + (NCFLMU * (idim - 1) + NCFLMU * NDIMEN * (iv - 1));
                AdvApp2Var_MathBase.mmdrc11_(IORDRU, iun, ncf0, Is, EXTR, FACT);
                for (iu = 1; iu <= IORDRU + 1; iu++)
                {
                    ideb = HCOINS.Lower() + NDIMEN * (iu - 1) + NDIMEN * (IORDRU + 2) * (iv - 1) - 1;
                    HCOINS.ChangeValue(idim + ideb) += HEXTR.Value(1 + 2 * (iu - 1));
                    HCOINS.ChangeValue(SIZE + idim + ideb) += HEXTR.Value(2 + 2 * (iu - 1));
                }
                Is = IsoV1 + (NCFLMU * (idim - 1) + NCFLMU * NDIMEN * (iv - 1));
                AdvApp2Var_MathBase.mmdrc11_(IORDRU, iun, ncf1, Is, EXTR, FACT);
                for (iu = 1; iu <= IORDRU + 1; iu++)
                {
                    ideb = HCOINS.Lower() + NDIMEN * (iu - 1) + NDIMEN * (IORDRU + 2) * (iv - 1) - 1;
                    HCOINS.ChangeValue(2 * SIZE + idim + ideb) += HEXTR.Value(1 + 2 * (iu - 1));
                    HCOINS.ChangeValue(3 * SIZE + idim + ideb) += HEXTR.Value(2 + 2 * (iu - 1));
                }
            }
        }

        // add all to PATCAN
        F2cPtr<double> C1 = HCOINS.AsSpan();
        F2cPtr<double> C2 = C1 + SIZE;
        F2cPtr<double> C3 = C2 + SIZE;
        F2cPtr<double> C4 = C3 + SIZE;
        if (IORDRU >= 0 && IORDRV >= 0)
        {
            AdvApp2Var_ApproxF2var.mma2ac1_(NDIMEN, NDegU, NDegV, IORDRU, IORDRV, C1, C2, C3, C4, HermU, HermV,
                                            PATCAN);
        }
    }

    /// <summary>Adds the errors of the constraints (isos and nodes) to the patch's errors.</summary>
    public void AddErrors(AdvApp2Var_Framework Constraints)
    {
        int NBSESP = 1, iesp;
        int iu, iv;

        double errU, errV, error;
        ReadOnlySpan<double> hmax = [0, 1, 1.5, 1.75];

        for (iesp = 1; iesp <= NBSESP; iesp++)
        {
            //  error max in sub-space iesp
            errU = 0.0;
            for (iv = 1; iv <= myOrdInV + 1; iv++)
            {
                error = Constraints.IsoV(myU0, myU1, myV0).MaxErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
                error = Constraints.IsoV(myU0, myU1, myV1).MaxErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
            }
            errV = 0.0;
            for (iu = 1; iu <= myOrdInU + 1; iu++)
            {
                error = Constraints.IsoU(myU0, myV0, myV1).MaxErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
                error = Constraints.IsoU(myU1, myV0, myV1).MaxErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
            }
            myMaxErrors!.ChangeValue(iesp) += errU * hmax[myOrdInV + 1] + errV * hmax[myOrdInU + 1];

            // average error in sub-space iesp
            errU = 0.0;
            for (iv = 1; iv <= myOrdInV + 1; iv++)
            {
                error = Constraints.IsoV(myU0, myU1, myV0).MoyErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
                error = Constraints.IsoV(myU0, myU1, myV1).MoyErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
            }
            errV = 0.0;
            for (iu = 1; iu <= myOrdInU + 1; iu++)
            {
                error = Constraints.IsoU(myU0, myV0, myV1).MoyErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
                error = Constraints.IsoU(myU1, myV0, myV1).MoyErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
            }
            error = myMoyErrors!.Value(iesp);
            error *= error;
            error += errU * hmax[myOrdInV + 1] * errU * hmax[myOrdInV + 1]
                     + errV * hmax[myOrdInU + 1] * errV * hmax[myOrdInU + 1];
            myMoyErrors.SetValue(iesp, Math.Sqrt(error));

            // max errors at iso-borders
            NCollection_HArray2<double> HERISO = new(1, NBSESP, 1, 4);
            HERISO.SetValue(iesp, 1, Constraints.IsoV(myU0, myU1, myV0).MaxErrors()!.Value(iesp, 1));
            HERISO.SetValue(iesp, 2, Constraints.IsoV(myU0, myU1, myV1).MaxErrors()!.Value(iesp, 1));
            HERISO.SetValue(iesp, 3, Constraints.IsoU(myU0, myV0, myV1).MaxErrors()!.Value(iesp, 1));
            HERISO.SetValue(iesp, 4, Constraints.IsoU(myU1, myV0, myV1).MaxErrors()!.Value(iesp, 1));

            // calculate max errors at the corners
            double emax1 = 0.0, emax2 = 0.0, emax3 = 0.0, emax4 = 0.0, err1, err2, err3, err4;
            for (iu = 0; iu <= myOrdInU; iu++)
            {
                for (iv = 0; iv <= myOrdInV; iv++)
                {
                    error = Constraints.Node(myU0, myV0).Error(iu, iv);
                    emax1 = Max(emax1, error);
                    error = Constraints.Node(myU1, myV0).Error(iu, iv);
                    emax2 = Max(emax2, error);
                    error = Constraints.Node(myU0, myV1).Error(iu, iv);
                    emax3 = Max(emax3, error);
                    error = Constraints.Node(myU1, myV1).Error(iu, iv);
                    emax4 = Max(emax4, error);
                }
            }

            // calculate max errors on borders
            err1 = Max(emax1, emax2);
            err2 = Max(emax3, emax4);
            err3 = Max(emax1, emax3);
            err4 = Max(emax2, emax4);

            //   calculate final errors on internal isos
            if (Constraints.IsoV(myU0, myU1, myV0).Position() == 0)
            {
                HERISO.ChangeValue(iesp, 1) += err1 * hmax[myOrdInU + 1];
            }
            if (Constraints.IsoV(myU0, myU1, myV1).Position() == 0)
            {
                HERISO.ChangeValue(iesp, 2) += err2 * hmax[myOrdInU + 1];
            }
            if (Constraints.IsoU(myU0, myV0, myV1).Position() == 0)
            {
                HERISO.ChangeValue(iesp, 3) += err3 * hmax[myOrdInV + 1];
            }
            if (Constraints.IsoU(myU1, myV0, myV1).Position() == 0)
            {
                HERISO.ChangeValue(iesp, 4) += err4 * hmax[myOrdInV + 1];
            }
            myIsoErrors = HERISO;
        }
    }

    /// <summary>
    /// Approximates the discretised patch. <paramref name="NumDec"/> says which cuts are allowed (0 none, 1 or 3 in
    /// U, 2 or 4 in V, 5 both); <see cref="CutSense()"/> then says which one the approximation asks for.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">An error in the Fortran.</exception>
    public void MakeApprox(AdvApp2Var_Context Conditions, AdvApp2Var_Framework Constraints, int NumDec)
    {
        // data stored in the Context
        int NDIMEN, NBSESP, NDIMSE;
        int NBPNTU, NBPNTV, NCFLMU, NCFLMV, NDJACU, NDJACV;
        int NDegU, NDegV, NJacU, NJacV;
        NDIMEN = Conditions.TotalDimension();
        NBSESP = Conditions.TotalNumberSSP();
        NDIMSE = 3;
        NBPNTU = Conditions.URoots()!.Length();
        if (myOrdInU > -1)
        {
            NBPNTU -= 2;
        }
        NBPNTV = Conditions.VRoots()!.Length();
        if (myOrdInV > -1)
        {
            NBPNTV -= 2;
        }
        NCFLMU = Conditions.ULimit();
        NCFLMV = Conditions.VLimit();
        NDegU = NCFLMU - 1;
        NDegV = NCFLMV - 1;
        NDJACU = Conditions.UJacDeg();
        NDJACV = Conditions.VJacDeg();
        NJacU = NDJACU + 1;
        NJacV = NDJACV + 1;

        // data relative to the processed patch
        int IORDRU = myOrdInU, IORDRV = myOrdInV, NDMINU = 1, NDMINV = 1, NCOEFU, NCOEFV;
        // NDMINU and NDMINV depend on the nb of coeff of neighboring isos
        // and of the required order of continuity
        NDMINU = Max(1, 2 * IORDRU + 1);
        NCOEFU = Constraints.IsoV(myU0, myU1, myV0).NbCoeff() - 1;
        NDMINU = Max(NDMINU, NCOEFU);
        NCOEFU = Constraints.IsoV(myU0, myU1, myV1).NbCoeff() - 1;
        NDMINU = Max(NDMINU, NCOEFU);

        NDMINV = Max(1, 2 * IORDRV + 1);
        NCOEFV = Constraints.IsoU(myU0, myV0, myV1).NbCoeff() - 1;
        NDMINV = Max(NDMINV, NCOEFV);
        NCOEFV = Constraints.IsoU(myU1, myV0, myV1).NbCoeff() - 1;
        NDMINV = Max(NDMINV, NCOEFV);

        // tables of approximations
        NCollection_HArray1<double> HEPSAPR = new(1, NBSESP);
        NCollection_HArray1<double> HEPSFRO = new(1, NBSESP * 8);
        int iesp;
        for (iesp = 1; iesp <= NBSESP; iesp++)
        {
            HEPSAPR.SetValue(iesp, Conditions.IToler()!.Value(iesp));
            HEPSFRO.SetValue(iesp, Conditions.FToler()!.Value(iesp, 1));
            HEPSFRO.SetValue(iesp + NBSESP, Conditions.FToler()!.Value(iesp, 2));
            HEPSFRO.SetValue(iesp + 2 * NBSESP, Conditions.FToler()!.Value(iesp, 3));
            HEPSFRO.SetValue(iesp + 3 * NBSESP, Conditions.FToler()!.Value(iesp, 4));
            HEPSFRO.SetValue(iesp + 4 * NBSESP, Conditions.CToler()!.Value(iesp, 1));
            HEPSFRO.SetValue(iesp + 5 * NBSESP, Conditions.CToler()!.Value(iesp, 2));
            HEPSFRO.SetValue(iesp + 6 * NBSESP, Conditions.CToler()!.Value(iesp, 3));
            HEPSFRO.SetValue(iesp + 7 * NBSESP, Conditions.CToler()!.Value(iesp, 4));
        }
        Span<double> EPSAPR = HEPSAPR.AsSpan();
        Span<double> EPSFRO = HEPSFRO.AsSpan();

        int SIZE = (1 + NDJACU) * (1 + NDJACV) * NDIMEN;
        NCollection_HArray1<double> HPJAC = new(1, SIZE);
        Span<double> PATJAC = HPJAC.AsSpan();
        SIZE = 2 * SIZE;
        NCollection_HArray1<double> HPAUX = new(1, SIZE);
        Span<double> PATAUX = HPAUX.AsSpan();
        SIZE = NCFLMU * NCFLMV * NDIMEN;
        NCollection_HArray1<double> HPCAN = new(1, SIZE);
        Span<double> PATCAN = HPCAN.AsSpan();
        NCollection_HArray1<double> HERRMAX = new(1, NBSESP);
        Span<double> ERRMAX = HERRMAX.AsSpan();
        NCollection_HArray1<double> HERRMOY = new(1, NBSESP);
        Span<double> ERRMOY = HERRMOY.AsSpan();

        // tables of discretization of the square
        Span<double> SOSOTB = mySosoTab!.AsSpan();
        Span<double> DISOTB = myDisoTab!.AsSpan();
        Span<double> SODITB = mySodiTab!.AsSpan();
        Span<double> DIDITB = myDidiTab!.AsSpan();

        //  approximation
        int ITYDEC = 0, IERCOD = 0;
        int iun = 1, itrois = 3;
        NCOEFU = 0;
        NCOEFV = 0;
        AdvApp2Var_ApproxF2var.mma2ce1_(NumDec,
                                        NDIMEN,
                                        NBSESP,
                                        F2c.In(in NDIMSE),
                                        NDMINU,
                                        NDMINV,
                                        NDegU,
                                        NDegV,
                                        NDJACU,
                                        NDJACV,
                                        IORDRU,
                                        IORDRV,
                                        NBPNTU,
                                        NBPNTV,
                                        EPSAPR,
                                        SOSOTB,
                                        DISOTB,
                                        SODITB,
                                        DIDITB,
                                        PATJAC,
                                        ERRMAX,
                                        ERRMOY,
                                        ref NCOEFU,
                                        ref NCOEFV,
                                        ref ITYDEC,
                                        ref IERCOD);

        // results
        myCutSense = ITYDEC;
        if (ITYDEC == 0 && IERCOD <= 0)
        {
            myHasResult = true;
            myApprIsDone = IERCOD == 0;
            myNbCoeffInU = NCOEFU + 1;
            myNbCoeffInV = NCOEFV + 1;
            myMaxErrors = HERRMAX;
            myMoyErrors = HERRMOY;

            // Passage to canonic on [-1,1]
            AdvApp2Var_MathBase.mmfmca9_(NJacU, NJacV, NDIMEN, myNbCoeffInU, myNbCoeffInV, NDIMEN, PATJAC, PATJAC);
            AdvApp2Var_ApproxF2var.mma2can_(NCFLMU,
                                            NCFLMV,
                                            NDIMEN,
                                            myOrdInU,
                                            myOrdInV,
                                            myNbCoeffInU,
                                            myNbCoeffInV,
                                            PATJAC,
                                            PATAUX,
                                            PATCAN,
                                            ref IERCOD);
            if (IERCOD != 0)
            {
                throw new Standard_ConstructionError("AdvApp2Var_Patch::MakeApprox : Error in FORTRAN");
            }
            myEquation = HPCAN;

            // Add constraints and errors
            AddConstraints(Conditions, Constraints);
            AddErrors(Constraints);

            // Reduction of degrees if possible
            PATCAN = myEquation.AsSpan();

            AdvApp2Var_ApproxF2var.mma2fx6_(NCFLMU,
                                            NCFLMV,
                                            NDIMEN,
                                            NBSESP,
                                            F2c.In(in itrois),
                                            iun,
                                            iun,
                                            IORDRU,
                                            IORDRV,
                                            EPSAPR,
                                            EPSFRO,
                                            PATCAN,
                                            ERRMAX,
                                            F2c.Ref(ref myNbCoeffInU),
                                            F2c.Ref(ref myNbCoeffInV));

            // transposition (NCFLMU,NCFLMV,NDIMEN)Fortran-C++
            int aIU, aIN, dim, ii, jj;
            for (dim = 1; dim <= NDIMEN; dim++)
            {
                aIN = (dim - 1) * NCFLMU * NCFLMV;
                for (ii = 1; ii <= NCFLMU; ii++)
                {
                    aIU = (ii - 1) * NDIMEN * NCFLMV;
                    for (jj = 1; jj <= NCFLMV; jj++)
                    {
                        HPAUX.SetValue(dim + NDIMEN * (jj - 1) + aIU, myEquation.Value(ii + NCFLMU * (jj - 1) + aIN));
                    }
                }
            }
            myEquation = HPAUX;
        }
        else
        {
            myApprIsDone = false;
            myHasResult = false;
        }
    }

    public void ChangeDomain(double a, double b, double c, double d)
    {
        myU0 = a;
        myU1 = b;
        myV0 = c;
        myV1 = d;
    }

    public void ResetApprox()
    {
        myApprIsDone = false;
        myHasResult = false;
    }

    public void OverwriteApprox()
    {
        if (myHasResult)
        {
            myApprIsDone = true;
        }
    }

    public double U0() => myU0;

    public double U1() => myU1;

    public double V0() => myV0;

    public double V1() => myV1;

    public int UOrder() => myOrdInU;

    public int VOrder() => myOrdInV;

    /// <summary>The cut the last approximation asked for: 0 none, 1 in U, 2 in V, 3 both.</summary>
    public int CutSense() => myCutSense;

    /// <summary>The cut that <paramref name="Crit"/> asks for: 0 if it is satisfied, else <paramref name="NumDec"/>.</summary>
    public int CutSense(AdvApp2Var_Criterion Crit, int NumDec)
    {
        bool CritRel = Crit.Type() == AdvApp2Var_Relative;
        if (CritRel && !IsApproximated())
        {
            return myCutSense;
        }
        else
        {
            if (Crit.IsSatisfied(this))
            {
                return 0;
            }
            else
            {
                return NumDec;
            }
        }
    }

    public int NbCoeffInU() => myNbCoeffInU;

    public int NbCoeffInV() => myNbCoeffInV;

    public void ChangeNbCoeff(int NbCoeffU, int NbCoeffV)
    {
        if (myNbCoeffInU < NbCoeffU)
        {
            myNbCoeffInU = NbCoeffU;
        }
        if (myNbCoeffInV < NbCoeffV)
        {
            myNbCoeffInV = NbCoeffV;
        }
    }

    /// <summary>The poles of the patch's polynomial on [-1, 1] x [-1, 1], as a Bezier patch.</summary>
    /// <exception cref="Standard_ConstructionError">Unless the context has one subspace and SSPIndex is 1.</exception>
    public NCollection_HArray2<gp_Pnt> Poles(int SSPIndex, AdvApp2Var_Context Cond)
    {
        NCollection_HArray1<double>? SousEquation;
        if (Cond.TotalNumberSSP() == 1 && SSPIndex == 1)
        {
            SousEquation = myEquation;
        }
        else
        {
            throw new Standard_ConstructionError("AdvApp2Var_Patch::Poles :  SSPIndex out of range");
        }
        NCollection_Array1<double> Intervalle = new(1, 2);
        Intervalle.SetValue(1, -1);
        Intervalle.SetValue(2, 1);
        NCollection_Array1<int> NbCoeff = new(1, 2);
        NbCoeff.SetValue(1, myNbCoeffInU);
        NbCoeff.SetValue(2, myNbCoeffInV);
        Convert_GridPolynomialToPoles Conv = new(Cond.ULimit() - 1,
                                                 Cond.VLimit() - 1,
                                                 NbCoeff,
                                                 SousEquation!.Array1(),
                                                 Intervalle,
                                                 Intervalle);
        return new NCollection_HArray2<gp_Pnt>(Conv.Poles());
    }

    /// <summary>The coefficients of the patch's polynomial on [-1, 1] x [-1, 1] (shared: don't modify).</summary>
    /// <exception cref="Standard_ConstructionError">Unless the context has one subspace and SSPIndex is 1.</exception>
    public NCollection_HArray1<double>? Coefficients(int SSPIndex, AdvApp2Var_Context Cond)
    {
        NCollection_HArray1<double>? SousEquation;
        if (Cond.TotalNumberSSP() == 1 && SSPIndex == 1)
        {
            SousEquation = myEquation;
        }
        else
        {
            throw new Standard_ConstructionError("AdvApp2Var_Patch::Poles :  SSPIndex out of range");
        }
        return SousEquation;
    }

    public NCollection_HArray1<double>? MaxErrors() => myMaxErrors;

    public NCollection_HArray1<double>? AverageErrors() => myMoyErrors;

    public NCollection_HArray2<double>? IsoErrors() => myIsoErrors;

    public double CritValue() => myCritValue;

    public void SetCritValue(double dist) => myCritValue = dist;
}
