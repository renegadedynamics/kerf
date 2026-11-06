// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Iso.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Iso.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The constraints of <see cref="ApproxAFunc2Var"/> on a line U = Ui or V = Vj: the approximation of F
/// and of its cross derivatives along the iso-curve.
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 194). The arrays it returns are
/// its own (shared: don't modify).
/// </remarks>
internal class Iso : Transient
{
    private readonly IsoType _type;
    private double _constPar;
    private double _u0;
    private double _u1;
    private double _v0;
    private double _v1;
    private int _position;
    private readonly int _extremOrder;
    private readonly int _derivOrder;
    private int _nbCoeff;
    private bool _apprIsDone;
    private bool _hasResult;
    private NCollection_HArray1<double>? _equation;
    private NCollection_HArray2<double>? _maxErrors;
    private NCollection_HArray2<double>? _moyErrors;
    private NCollection_HArray1<double>? _somTab;
    private NCollection_HArray1<double>? _difTab;

    public Iso()
    {
        _type = IsoU;
        _constPar = 0.5;
        _u0 = 0.0;
        _u1 = 1.0;
        _v0 = 0.0;
        _v1 = 1.0;
        _position = 0;
        _extremOrder = 2;
        _derivOrder = 2;
        _nbCoeff = 0;
        _apprIsDone = false;
        _hasResult = false;
    }

    public Iso(IsoType type, int iu, int iv)
        : this(type, 0.5, 0.0, 1.0, 0.0, 1.0, 0, iu, iv)
    {
    }

    public Iso(IsoType type,
                          double cte,
                          double ufirst,
                          double ulast,
                          double vfirst,
                          double vlast,
                          int pos,
                          int iu,
                          int iv)
    {
        _type = type;
        _constPar = cte;
        _u0 = ufirst;
        _u1 = ulast;
        _v0 = vfirst;
        _v1 = vlast;
        _position = pos;
        _nbCoeff = 0;
        _apprIsDone = false;
        _hasResult = false;
        if (_type == IsoU)
        {
            _extremOrder = iv;
            _derivOrder = iu;
        }
        else
        {
            _extremOrder = iu;
            _derivOrder = iv;
        }
    }

    public bool IsApproximated => _apprIsDone;

    public bool HasResult => _hasResult;

    /// <summary>
    /// Approximates F and its cross derivatives on the iso-curve, within the domain [U0, U1] x [V0, V1], and stores
    /// the values and derivatives at its ends in <paramref name="nodeBegin"/> and <paramref name="nodeEnd"/>.
    /// </summary>
    public void MakeApprox(Context conditions,
                           double u0,
                           double u1,
                           double v0,
                           double v1,
                           EvaluatorFunc2Var func,
                           Node nodeBegin,
                           Node nodeEnd)
    {
        // fixed values
        int nbcrmx = 1, nbcrbe = 0;
        // data stored in the Context
        int ndimen, nbsesp, ndimse;
        ndimen = conditions.TotalDimension();
        nbsesp = conditions.TotalNumberSSP();
        // Attention : works only in 3D
        ndimse = 3;
        // the domain of the grid
        Span<double> uvfonc = stackalloc double[4];
        uvfonc[0] = u0;
        uvfonc[1] = u1;
        uvfonc[2] = v0;
        uvfonc[3] = v1;

        // data related to the processed iso
        int iordre = _extremOrder, ideriv = _derivOrder;
        double tconst = _constPar;

        // data related to the type of the iso
        int isofav = 0, nbroot = 0, ndgjac = 0, ncflim = 1;
        Span<double> tabdec = stackalloc double[2];
        NCollection_HArray1<double> huroot = conditions.URoots()!;
        NCollection_HArray1<double> hvroot = conditions.VRoots()!;
        Span<double> rootlg = default;
        switch (_type)
        {
            case IsoV:
                isofav = 2;
                tabdec[0] = _u0;
                tabdec[1] = _u1;
                uvfonc[0] = _u0;
                uvfonc[1] = _u1;
                nbroot = conditions.URoots()!.Length();
                if (_extremOrder > -1)
                {
                    nbroot -= 2;
                }
                rootlg = huroot.AsSpan();
                ndgjac = conditions.UJacDeg();
                ncflim = conditions.ULimit();
                break;
            case IsoU:
                isofav = 1;
                tabdec[0] = _v0;
                tabdec[1] = _v1;
                uvfonc[2] = _v0;
                uvfonc[3] = _v1;
                nbroot = conditions.VRoots()!.Length();
                if (_extremOrder > -1)
                {
                    nbroot -= 2;
                }
                rootlg = hvroot.AsSpan();
                ndgjac = conditions.VJacDeg();
                ncflim = conditions.VLimit();
                break;
            case NoneIso:
            default:
                break;
        }

        // data relative to the position of iso (front or cut line)
        NCollection_HArray1<double> hepsapr = new(1, nbsesp);
        int iesp;
        switch (_position)
        {
            case 0:
                for (iesp = 1; iesp <= nbsesp; iesp++)
                {
                    hepsapr.SetValue(iesp, conditions.CToler()!.Value(iesp, 1));
                }
                break;
            case 1:
                for (iesp = 1; iesp <= nbsesp; iesp++)
                {
                    hepsapr.SetValue(iesp, conditions.FToler()!.Value(iesp, 1));
                }
                break;
            case 2:
                for (iesp = 1; iesp <= nbsesp; iesp++)
                {
                    hepsapr.SetValue(iesp, conditions.FToler()!.Value(iesp, 2));
                }
                break;
            case 3:
                for (iesp = 1; iesp <= nbsesp; iesp++)
                {
                    hepsapr.SetValue(iesp, conditions.FToler()!.Value(iesp, 3));
                }
                break;
            case 4:
                for (iesp = 1; iesp <= nbsesp; iesp++)
                {
                    hepsapr.SetValue(iesp, conditions.FToler()!.Value(iesp, 4));
                }
                break;
        }
        Span<double> epsapr = hepsapr.AsSpan();

        // the tables of approximations
        int szcrb = ndimen * ncflim;
        NCollection_HArray1<double> hcourbe = new(1, szcrb * (ideriv + 1));
        Span<double> courbe = hcourbe.AsSpan();
        int crbapp = 0; // C++: double* CRBAPP = COURBE, moved along; the port keeps the offset
        int sztab = (1 + nbroot / 2) * ndimen;
        NCollection_HArray1<double> hsomtab = new(1, sztab * (ideriv + 1));
        Span<double> somtab = hsomtab.AsSpan();
        int somapp = 0;
        NCollection_HArray1<double> hdiftab = new(1, sztab * (ideriv + 1));
        Span<double> diftab = hdiftab.AsSpan();
        int difapp = 0;
        NCollection_HArray1<double> hcontr1 = new(1, (iordre + 2) * ndimen);
        Span<double> contr1 = hcontr1.AsSpan();
        NCollection_HArray1<double> hcontr2 = new(1, (iordre + 2) * ndimen);
        Span<double> contr2 = hcontr2.AsSpan();
        NCollection_HArray2<double> herrmax = new(1, nbsesp, 1, ideriv + 1);
        double[] emxapp = new double[nbsesp];
        NCollection_HArray2<double> herrmoy = new(1, nbsesp, 1, ideriv + 1);
        double[] emyapp = new double[nbsesp];
        //
        // the approximations
        //
        int iercod = 0, ncoeff = 0;
        int iapp, ncfapp, ierapp;
        for (iapp = 0; iapp <= ideriv; iapp++)
        {
            //   approximation of the derivative of order iapp
            ncfapp = 0;
            ierapp = 0;
            ApproxF2var.mma2fnc_(ndimen,
                                            nbsesp,
                                            global::Kerf.Engine.Approximation.Advanced.F2c.In(in ndimse),
                                            uvfonc,
                                            func,
                                            tconst,
                                            isofav,
                                            nbroot,
                                            rootlg,
                                            iordre,
                                            iapp,
                                            ndgjac,
                                            nbcrmx,
                                            ncflim,
                                            epsapr,
                                            global::Kerf.Engine.Approximation.Advanced.F2c.Ref(ref ncfapp),
                                            courbe.Slice(crbapp),
                                            ref nbcrbe,
                                            somtab.Slice(somapp),
                                            diftab.Slice(difapp),
                                            contr1,
                                            contr2,
                                            tabdec,
                                            emxapp,
                                            emyapp,
                                            ref ierapp);
            //   error and coefficient management.
            if (ierapp > 0)
            {
                _apprIsDone = false;
                _hasResult = false;
                return; // C++: goto FINISH, which frees EMXAPP and EMYAPP
            }
            if (ncoeff <= ncfapp)
            {
                ncoeff = ncfapp;
            }
            if (ierapp == -1)
            {
                iercod = -1;
            }
            //   return constraints of order 0 to IORDRE of extremities
            int ider, jpos = hcontr1.Lower();
            for (ider = 0; ider <= iordre; ider++)
            {
                // OCCT reads three coordinates whatever the dimension (occt-issues.md)
                Point3D pt = new(hcontr1.Value(jpos), hcontr1.Value(jpos + 1), hcontr1.Value(jpos + 2));
                if (isofav == 2)
                {
                    nodeBegin.SetPoint(ider, iapp, pt);
                }
                else
                {
                    nodeBegin.SetPoint(iapp, ider, pt);
                }
                jpos += 3;
            }
            jpos = hcontr2.Lower();
            for (ider = 0; ider <= iordre; ider++)
            {
                Point3D pt = new(hcontr2.Value(jpos), hcontr2.Value(jpos + 1), hcontr2.Value(jpos + 2));
                if (isofav == 2)
                {
                    nodeEnd.SetPoint(ider, iapp, pt);
                }
                else
                {
                    nodeEnd.SetPoint(iapp, ider, pt);
                }
                jpos += 3;
            }
            //   return errors
            for (iesp = 1; iesp <= nbsesp; iesp++)
            {
                herrmax.SetValue(iesp, iapp + 1, emxapp[iesp - 1]);
                herrmoy.SetValue(iesp, iapp + 1, emyapp[iesp - 1]);
            }
            // passage to the approximation of higher order
            crbapp += szcrb;
            somapp += sztab;
            difapp += sztab;
        }

        // management of results
        if (iercod == 0)
        {
            //   all approximations are correct
            _apprIsDone = true;
            _hasResult = true;
        }
        else if (iercod == -1)
        {
            //   at least one approximation is not correct
            _apprIsDone = false;
            _hasResult = true;
        }
        else
        {
            _apprIsDone = false;
            _hasResult = false;
        }
        if (_hasResult)
        {
            _equation = hcourbe;
            _nbCoeff = ncoeff;
            _maxErrors = herrmax;
            _moyErrors = herrmoy;
            _somTab = hsomtab;
            _difTab = hdiftab;
        }
    }

    /// <summary>Changes the bounds of the iso-curve's own parameter.</summary>
    public void ChangeDomain(double a, double b)
    {
        if (_type == IsoU)
        {
            _v0 = a;
            _v1 = b;
        }
        else
        {
            _u0 = a;
            _u1 = b;
        }
    }

    public void ChangeDomain(double a, double b, double c, double d)
    {
        _u0 = a;
        _u1 = b;
        _v0 = c;
        _v1 = d;
    }

    public void SetConstante(double newcte) => _constPar = newcte;

    public void SetPosition(int newpos) => _position = newpos;

    public void ResetApprox()
    {
        _apprIsDone = false;
        _hasResult = false;
    }

    public void OverwriteApprox()
    {
        if (_hasResult)
        {
            _apprIsDone = true;
        }
    }

    public IsoType Type() => _type;

    public double Constante() => _constPar;

    public double T0() => _type == IsoU ? _v0 : _u0;

    public double T1() => _type == IsoU ? _v1 : _u1;

    public double U0() => _u0;

    public double U1() => _u1;

    public double V0() => _v0;

    public double V1() => _v1;

    public int UOrder() => Type() == IsoU ? _derivOrder : _extremOrder;

    public int VOrder() => Type() == IsoV ? _derivOrder : _extremOrder;

    public int Position() => _position;

    public int CoeffCount => _nbCoeff;

    /// <summary>The coefficients of the approximation (shared: don't modify); null before one.</summary>
    public NCollection_HArray1<double>? Polynom() => _equation;

    public NCollection_HArray1<double>? SomTab() => _somTab;

    public NCollection_HArray1<double>? DifTab() => _difTab;

    public NCollection_HArray2<double>? MaxErrors() => _maxErrors;

    public NCollection_HArray2<double>? MoyErrors() => _moyErrors;
}
