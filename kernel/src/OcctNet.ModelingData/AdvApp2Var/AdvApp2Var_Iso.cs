// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Iso.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Iso.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_IsoType;

namespace Occt;

/// <summary>
/// The constraints of <see cref="AdvApp2Var_ApproxAFunc2Var"/> on a line U = Ui or V = Vj: the approximation of F
/// and of its cross derivatives along the iso-curve.
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 1903). The arrays it returns are
/// its own (shared: don't modify).
/// </remarks>
public class AdvApp2Var_Iso : Standard_Transient
{
    private readonly GeomAbs_IsoType myType;
    private double myConstPar;
    private double myU0;
    private double myU1;
    private double myV0;
    private double myV1;
    private int myPosition;
    private readonly int myExtremOrder;
    private readonly int myDerivOrder;
    private int myNbCoeff;
    private bool myApprIsDone;
    private bool myHasResult;
    private NCollection_HArray1<double>? myEquation;
    private NCollection_HArray2<double>? myMaxErrors;
    private NCollection_HArray2<double>? myMoyErrors;
    private NCollection_HArray1<double>? mySomTab;
    private NCollection_HArray1<double>? myDifTab;

    public AdvApp2Var_Iso()
    {
        myType = GeomAbs_IsoU;
        myConstPar = 0.5;
        myU0 = 0.0;
        myU1 = 1.0;
        myV0 = 0.0;
        myV1 = 1.0;
        myPosition = 0;
        myExtremOrder = 2;
        myDerivOrder = 2;
        myNbCoeff = 0;
        myApprIsDone = false;
        myHasResult = false;
    }

    public AdvApp2Var_Iso(GeomAbs_IsoType type, int iu, int iv)
        : this(type, 0.5, 0.0, 1.0, 0.0, 1.0, 0, iu, iv)
    {
    }

    public AdvApp2Var_Iso(GeomAbs_IsoType type,
                          double cte,
                          double Ufirst,
                          double Ulast,
                          double Vfirst,
                          double Vlast,
                          int pos,
                          int iu,
                          int iv)
    {
        myType = type;
        myConstPar = cte;
        myU0 = Ufirst;
        myU1 = Ulast;
        myV0 = Vfirst;
        myV1 = Vlast;
        myPosition = pos;
        myNbCoeff = 0;
        myApprIsDone = false;
        myHasResult = false;
        if (myType == GeomAbs_IsoU)
        {
            myExtremOrder = iv;
            myDerivOrder = iu;
        }
        else
        {
            myExtremOrder = iu;
            myDerivOrder = iv;
        }
    }

    public bool IsApproximated() => myApprIsDone;

    public bool HasResult() => myHasResult;

    /// <summary>
    /// Approximates F and its cross derivatives on the iso-curve, within the domain [U0, U1] x [V0, V1], and stores
    /// the values and derivatives at its ends in <paramref name="NodeBegin"/> and <paramref name="NodeEnd"/>.
    /// </summary>
    public void MakeApprox(AdvApp2Var_Context Conditions,
                           double U0,
                           double U1,
                           double V0,
                           double V1,
                           AdvApp2Var_EvaluatorFunc2Var Func,
                           AdvApp2Var_Node NodeBegin,
                           AdvApp2Var_Node NodeEnd)
    {
        // fixed values
        int NBCRMX = 1, NBCRBE = 0;
        // data stored in the Context
        int NDIMEN, NBSESP, NDIMSE;
        NDIMEN = Conditions.TotalDimension();
        NBSESP = Conditions.TotalNumberSSP();
        // Attention : works only in 3D
        NDIMSE = 3;
        // the domain of the grid
        Span<double> UVFONC = stackalloc double[4];
        UVFONC[0] = U0;
        UVFONC[1] = U1;
        UVFONC[2] = V0;
        UVFONC[3] = V1;

        // data related to the processed iso
        int IORDRE = myExtremOrder, IDERIV = myDerivOrder;
        double TCONST = myConstPar;

        // data related to the type of the iso
        int ISOFAV = 0, NBROOT = 0, NDGJAC = 0, NCFLIM = 1;
        Span<double> TABDEC = stackalloc double[2];
        NCollection_HArray1<double> HUROOT = Conditions.URoots()!;
        NCollection_HArray1<double> HVROOT = Conditions.VRoots()!;
        Span<double> ROOTLG = default;
        switch (myType)
        {
            case GeomAbs_IsoV:
                ISOFAV = 2;
                TABDEC[0] = myU0;
                TABDEC[1] = myU1;
                UVFONC[0] = myU0;
                UVFONC[1] = myU1;
                NBROOT = Conditions.URoots()!.Length();
                if (myExtremOrder > -1)
                {
                    NBROOT -= 2;
                }
                ROOTLG = HUROOT.AsSpan();
                NDGJAC = Conditions.UJacDeg();
                NCFLIM = Conditions.ULimit();
                break;
            case GeomAbs_IsoU:
                ISOFAV = 1;
                TABDEC[0] = myV0;
                TABDEC[1] = myV1;
                UVFONC[2] = myV0;
                UVFONC[3] = myV1;
                NBROOT = Conditions.VRoots()!.Length();
                if (myExtremOrder > -1)
                {
                    NBROOT -= 2;
                }
                ROOTLG = HVROOT.AsSpan();
                NDGJAC = Conditions.VJacDeg();
                NCFLIM = Conditions.VLimit();
                break;
            case GeomAbs_NoneIso:
            default:
                break;
        }

        // data relative to the position of iso (front or cut line)
        NCollection_HArray1<double> HEPSAPR = new(1, NBSESP);
        int iesp;
        switch (myPosition)
        {
            case 0:
                for (iesp = 1; iesp <= NBSESP; iesp++)
                {
                    HEPSAPR.SetValue(iesp, Conditions.CToler()!.Value(iesp, 1));
                }
                break;
            case 1:
                for (iesp = 1; iesp <= NBSESP; iesp++)
                {
                    HEPSAPR.SetValue(iesp, Conditions.FToler()!.Value(iesp, 1));
                }
                break;
            case 2:
                for (iesp = 1; iesp <= NBSESP; iesp++)
                {
                    HEPSAPR.SetValue(iesp, Conditions.FToler()!.Value(iesp, 2));
                }
                break;
            case 3:
                for (iesp = 1; iesp <= NBSESP; iesp++)
                {
                    HEPSAPR.SetValue(iesp, Conditions.FToler()!.Value(iesp, 3));
                }
                break;
            case 4:
                for (iesp = 1; iesp <= NBSESP; iesp++)
                {
                    HEPSAPR.SetValue(iesp, Conditions.FToler()!.Value(iesp, 4));
                }
                break;
        }
        Span<double> EPSAPR = HEPSAPR.AsSpan();

        // the tables of approximations
        int SZCRB = NDIMEN * NCFLIM;
        NCollection_HArray1<double> HCOURBE = new(1, SZCRB * (IDERIV + 1));
        Span<double> COURBE = HCOURBE.AsSpan();
        int CRBAPP = 0; // C++: double* CRBAPP = COURBE, moved along; the port keeps the offset
        int SZTAB = (1 + NBROOT / 2) * NDIMEN;
        NCollection_HArray1<double> HSOMTAB = new(1, SZTAB * (IDERIV + 1));
        Span<double> SOMTAB = HSOMTAB.AsSpan();
        int SOMAPP = 0;
        NCollection_HArray1<double> HDIFTAB = new(1, SZTAB * (IDERIV + 1));
        Span<double> DIFTAB = HDIFTAB.AsSpan();
        int DIFAPP = 0;
        NCollection_HArray1<double> HCONTR1 = new(1, (IORDRE + 2) * NDIMEN);
        Span<double> CONTR1 = HCONTR1.AsSpan();
        NCollection_HArray1<double> HCONTR2 = new(1, (IORDRE + 2) * NDIMEN);
        Span<double> CONTR2 = HCONTR2.AsSpan();
        NCollection_HArray2<double> HERRMAX = new(1, NBSESP, 1, IDERIV + 1);
        double[] EMXAPP = new double[NBSESP];
        NCollection_HArray2<double> HERRMOY = new(1, NBSESP, 1, IDERIV + 1);
        double[] EMYAPP = new double[NBSESP];
        //
        // the approximations
        //
        int IERCOD = 0, NCOEFF = 0;
        int iapp, ncfapp, ierapp;
        for (iapp = 0; iapp <= IDERIV; iapp++)
        {
            //   approximation of the derivative of order iapp
            ncfapp = 0;
            ierapp = 0;
            AdvApp2Var_ApproxF2var.mma2fnc_(NDIMEN,
                                            NBSESP,
                                            F2c.In(in NDIMSE),
                                            UVFONC,
                                            Func,
                                            TCONST,
                                            ISOFAV,
                                            NBROOT,
                                            ROOTLG,
                                            IORDRE,
                                            iapp,
                                            NDGJAC,
                                            NBCRMX,
                                            NCFLIM,
                                            EPSAPR,
                                            F2c.Ref(ref ncfapp),
                                            COURBE.Slice(CRBAPP),
                                            ref NBCRBE,
                                            SOMTAB.Slice(SOMAPP),
                                            DIFTAB.Slice(DIFAPP),
                                            CONTR1,
                                            CONTR2,
                                            TABDEC,
                                            EMXAPP,
                                            EMYAPP,
                                            ref ierapp);
            //   error and coefficient management.
            if (ierapp > 0)
            {
                myApprIsDone = false;
                myHasResult = false;
                return; // C++: goto FINISH, which frees EMXAPP and EMYAPP
            }
            if (NCOEFF <= ncfapp)
            {
                NCOEFF = ncfapp;
            }
            if (ierapp == -1)
            {
                IERCOD = -1;
            }
            //   return constraints of order 0 to IORDRE of extremities
            int ider, jpos = HCONTR1.Lower();
            for (ider = 0; ider <= IORDRE; ider++)
            {
                // OCCT reads three coordinates whatever the dimension (occt-issues.md)
                gp_Pnt pt = new(HCONTR1.Value(jpos), HCONTR1.Value(jpos + 1), HCONTR1.Value(jpos + 2));
                if (ISOFAV == 2)
                {
                    NodeBegin.SetPoint(ider, iapp, pt);
                }
                else
                {
                    NodeBegin.SetPoint(iapp, ider, pt);
                }
                jpos += 3;
            }
            jpos = HCONTR2.Lower();
            for (ider = 0; ider <= IORDRE; ider++)
            {
                gp_Pnt pt = new(HCONTR2.Value(jpos), HCONTR2.Value(jpos + 1), HCONTR2.Value(jpos + 2));
                if (ISOFAV == 2)
                {
                    NodeEnd.SetPoint(ider, iapp, pt);
                }
                else
                {
                    NodeEnd.SetPoint(iapp, ider, pt);
                }
                jpos += 3;
            }
            //   return errors
            for (iesp = 1; iesp <= NBSESP; iesp++)
            {
                HERRMAX.SetValue(iesp, iapp + 1, EMXAPP[iesp - 1]);
                HERRMOY.SetValue(iesp, iapp + 1, EMYAPP[iesp - 1]);
            }
            // passage to the approximation of higher order
            CRBAPP += SZCRB;
            SOMAPP += SZTAB;
            DIFAPP += SZTAB;
        }

        // management of results
        if (IERCOD == 0)
        {
            //   all approximations are correct
            myApprIsDone = true;
            myHasResult = true;
        }
        else if (IERCOD == -1)
        {
            //   at least one approximation is not correct
            myApprIsDone = false;
            myHasResult = true;
        }
        else
        {
            myApprIsDone = false;
            myHasResult = false;
        }
        if (myHasResult)
        {
            myEquation = HCOURBE;
            myNbCoeff = NCOEFF;
            myMaxErrors = HERRMAX;
            myMoyErrors = HERRMOY;
            mySomTab = HSOMTAB;
            myDifTab = HDIFTAB;
        }
    }

    /// <summary>Changes the bounds of the iso-curve's own parameter.</summary>
    public void ChangeDomain(double a, double b)
    {
        if (myType == GeomAbs_IsoU)
        {
            myV0 = a;
            myV1 = b;
        }
        else
        {
            myU0 = a;
            myU1 = b;
        }
    }

    public void ChangeDomain(double a, double b, double c, double d)
    {
        myU0 = a;
        myU1 = b;
        myV0 = c;
        myV1 = d;
    }

    public void SetConstante(double newcte) => myConstPar = newcte;

    public void SetPosition(int newpos) => myPosition = newpos;

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

    public GeomAbs_IsoType Type() => myType;

    public double Constante() => myConstPar;

    public double T0() => myType == GeomAbs_IsoU ? myV0 : myU0;

    public double T1() => myType == GeomAbs_IsoU ? myV1 : myU1;

    public double U0() => myU0;

    public double U1() => myU1;

    public double V0() => myV0;

    public double V1() => myV1;

    public int UOrder() => Type() == GeomAbs_IsoU ? myDerivOrder : myExtremOrder;

    public int VOrder() => Type() == GeomAbs_IsoV ? myDerivOrder : myExtremOrder;

    public int Position() => myPosition;

    public int NbCoeff() => myNbCoeff;

    /// <summary>The coefficients of the approximation (shared: don't modify); null before one.</summary>
    public NCollection_HArray1<double>? Polynom() => myEquation;

    public NCollection_HArray1<double>? SomTab() => mySomTab;

    public NCollection_HArray1<double>? DifTab() => myDifTab;

    public NCollection_HArray2<double>? MaxErrors() => myMaxErrors;

    public NCollection_HArray2<double>? MoyErrors() => myMoyErrors;
}
