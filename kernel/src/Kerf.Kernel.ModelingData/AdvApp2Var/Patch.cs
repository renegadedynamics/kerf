// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Patch.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Patch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Advanced.CriterionType;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// A patch [Ui, Ui+1] x [Vj, Vj+1] of <see cref="ApproxAFunc2Var"/>: the discretisation of F on it, and
/// its polynomial approximation with the errors.
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 194). The arrays it returns are
/// its own (shared: don't modify); they are null until the patch is approximated.
/// </remarks>
public class Patch : Transient
{
    private double _u0;
    private double _u1;
    private double _v0;
    private double _v1;
    private readonly int _ordInU;
    private readonly int _ordInV;
    private int _nbCoeffInU;
    private int _nbCoeffInV;
    private bool _apprIsDone;
    private bool _hasResult;
    private NCollection_HArray1<double>? _equation;
    private NCollection_HArray1<double>? _maxErrors;
    private NCollection_HArray1<double>? _moyErrors;
    private NCollection_HArray2<double>? _isoErrors;
    private int _cutSense;
    private bool _discIsDone;
    private NCollection_HArray1<double>? _sosoTab;
    private NCollection_HArray1<double>? _disoTab;
    private NCollection_HArray1<double>? _sodiTab;
    private NCollection_HArray1<double>? _didiTab;
    private double _critValue;

    public Patch()
    {
        _u0 = 0.0;
        _u1 = 1.0;
        _v0 = 0.0;
        _v1 = 1.0;
        _ordInU = 0;
        _ordInV = 0;
        _nbCoeffInU = 0;
        _nbCoeffInV = 0;
        _apprIsDone = false;
        _hasResult = false;
        _cutSense = 0;
        _discIsDone = false;
        _critValue = 0.0;
    }

    public Patch(double u0, double u1, double v0, double v1, int iu, int iv)
    {
        _u0 = u0;
        _u1 = u1;
        _v0 = v0;
        _v1 = v1;
        _ordInU = iu;
        _ordInV = iv;
        _nbCoeffInU = 0;
        _nbCoeffInV = 0;
        _apprIsDone = false;
        _hasResult = false;
        _cutSense = 0;
        _discIsDone = false;
        _critValue = 0.0;
    }

    public bool IsDiscretised => _discIsDone;

    /// <summary>Discretises F on the patch, less the interpolation of its constraints.</summary>
    public void Discretise(Context conditions,
                           Framework constraints,
                           EvaluatorFunc2Var func)
    {
        // data stored in the Context
        int ndimen, isofav;
        ndimen = conditions.TotalDimension();
        // Attention : works only for 3D
        isofav = conditions.FavorIso();

        // data related to the patch to be discretized
        int nbpntu, nbpntv;
        int iordru = _ordInU, iordrv = _ordInV;
        NCollection_HArray1<double> huroot = conditions.URoots()!;
        NCollection_HArray1<double> hvroot = conditions.VRoots()!;
        Span<double> uroot = huroot.AsSpan();
        nbpntu = conditions.URoots()!.Length();
        if (_ordInU > -1)
        {
            nbpntu -= 2;
        }

        Span<double> vroot = hvroot.AsSpan();
        nbpntv = conditions.VRoots()!.Length();
        if (_ordInV > -1)
        {
            nbpntv -= 2;
        }

        // data stored in the Framework Constraints cad Nodes and Isos
        // C1, C2, C3 and C4 are dimensionnes in FORTRAN with (NDIMEN,IORDRU+2,IORDRV+2)
        int size = ndimen * (iordru + 2) * (iordrv + 2);
        NCollection_HArray1<double> hcoins = new(1, size * 4);
        hcoins.Init(0.0);

        int iu, iv;
        double du = (_u1 - _u0) / 2, dv = (_v1 - _v0) / 2, rho, valnorm;

        for (iu = 0; iu <= _ordInU; iu++)
        {
            for (iv = 0; iv <= _ordInV; iv++)
            {
                // factor of normalization
                rho = Math.Pow(du, iu) * Math.Pow(dv, iv);

                // F(U0,V0) and its derivatives normalized on (-1,1)
                valnorm = rho * constraints.Node(_u0, _v0).Point(iu, iv).X;
                hcoins.SetValue(1 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u0, _v0).Point(iu, iv).Y;
                hcoins.SetValue(2 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u0, _v0).Point(iu, iv).Z;
                hcoins.SetValue(3 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);

                // F(U1,V0) and its derivatives normalized on (-1,1)
                valnorm = rho * constraints.Node(_u1, _v0).Point(iu, iv).X;
                hcoins.SetValue(size + 1 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u1, _v0).Point(iu, iv).Y;
                hcoins.SetValue(size + 2 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u1, _v0).Point(iu, iv).Z;
                hcoins.SetValue(size + 3 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);

                // F(U0,V1) and its derivatives normalized on (-1,1)
                valnorm = rho * constraints.Node(_u0, _v1).Point(iu, iv).X;
                hcoins.SetValue(2 * size + 1 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u0, _v1).Point(iu, iv).Y;
                hcoins.SetValue(2 * size + 2 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u0, _v1).Point(iu, iv).Z;
                hcoins.SetValue(2 * size + 3 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);

                // F(U1,V1) and its derivatives normalized on (-1,1)
                valnorm = rho * constraints.Node(_u1, _v1).Point(iu, iv).X;
                hcoins.SetValue(3 * size + 1 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u1, _v1).Point(iu, iv).Y;
                hcoins.SetValue(3 * size + 2 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
                valnorm = rho * constraints.Node(_u1, _v1).Point(iu, iv).Z;
                hcoins.SetValue(3 * size + 3 + ndimen * iu + ndimen * (iordru + 2) * iv, valnorm);
            }
        }
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c1 = hcoins.AsSpan();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c2 = c1 + size;
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c3 = c2 + size;
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c4 = c3 + size;

        // tables SomTab and Diftab of discretization of isos U=U0 and U=U1
        // SU0, SU1, DU0 and DU1 are dimensioned in FORTRAN to
        // (1+NBPNTV/2)*NDIMEN*(IORDRU+1)

        size = (1 + nbpntv / 2) * ndimen;

        NCollection_HArray1<double> hsu0 = new(1, size * (iordru + 1));
        hsu0.ChangeArray1().Assign(constraints.IsoU(_u0, _v0, _v1).SomTab()!.Array1());

        NCollection_HArray1<double> hdu0 = new(1, size * (iordru + 1));
        hdu0.ChangeArray1().Assign(constraints.IsoU(_u0, _v0, _v1).DifTab()!.Array1());

        NCollection_HArray1<double> hsu1 = new(1, size * (iordru + 1));
        hsu1.ChangeArray1().Assign(constraints.IsoU(_u1, _v0, _v1).SomTab()!.Array1());

        NCollection_HArray1<double> hdu1 = new(1, size * (iordru + 1));
        hdu1.ChangeArray1().Assign(constraints.IsoU(_u1, _v0, _v1).DifTab()!.Array1());

        // normalization
        int ideb1, ideb2, ideb3, ideb4, jj;
        for (iu = 1; iu <= iordru; iu++)
        {
            rho = Math.Pow(du, iu);
            ideb1 = hsu0.Lower() + iu * size - 1;
            ideb2 = hdu0.Lower() + iu * size - 1;
            ideb3 = hsu1.Lower() + iu * size - 1;
            ideb4 = hdu1.Lower() + iu * size - 1;
            for (jj = 1; jj <= size; jj++)
            {
                hsu0.SetValue(ideb1 + jj, rho * hsu0.Value(ideb1 + jj));
                hdu0.SetValue(ideb2 + jj, rho * hdu0.Value(ideb2 + jj));
                hsu1.SetValue(ideb3 + jj, rho * hsu1.Value(ideb3 + jj));
                hdu1.SetValue(ideb4 + jj, rho * hdu1.Value(ideb4 + jj));
            }
        }

        Span<double> su0 = hsu0.AsSpan();
        Span<double> du0 = hdu0.AsSpan();
        Span<double> su1 = hsu1.AsSpan();
        Span<double> du1 = hdu1.AsSpan();

        // tables SomTab and Diftab of discretization of isos V=V0 and V=V1
        // SU0, SU1, DU0 and DU1 are dimensioned in FORTRAN at
        // (1+NBPNTU/2)*NDIMEN*(IORDRV+1)

        size = (1 + nbpntu / 2) * ndimen;

        NCollection_HArray1<double> hsv0 = new(1, size * (iordrv + 1));
        hsv0.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v0).SomTab()!.Array1());

        NCollection_HArray1<double> hdv0 = new(1, size * (iordrv + 1));
        hdv0.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v0).DifTab()!.Array1());

        NCollection_HArray1<double> hsv1 = new(1, size * (iordrv + 1));
        hsv1.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v1).SomTab()!.Array1());

        NCollection_HArray1<double> hdv1 = new(1, size * (iordrv + 1));
        hdv1.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v1).DifTab()!.Array1());

        // normalisation
        for (iv = 1; iv <= iordrv; iv++)
        {
            rho = Math.Pow(dv, iv);
            ideb1 = hsv0.Lower() + iv * size - 1;
            ideb2 = hdv0.Lower() + iv * size - 1;
            ideb3 = hsv1.Lower() + iv * size - 1;
            ideb4 = hdv1.Lower() + iv * size - 1;
            for (jj = 1; jj <= size; jj++)
            {
                hsv0.SetValue(ideb1 + jj, rho * hsv0.Value(ideb1 + jj));
                hdv0.SetValue(ideb2 + jj, rho * hdv0.Value(ideb2 + jj));
                hsv1.SetValue(ideb3 + jj, rho * hsv1.Value(ideb3 + jj));
                hdv1.SetValue(ideb4 + jj, rho * hdv1.Value(ideb4 + jj));
            }
        }

        Span<double> sv0 = hsv0.AsSpan();
        Span<double> dv0 = hdv0.AsSpan();
        Span<double> sv1 = hsv1.AsSpan();
        Span<double> dv1 = hdv1.AsSpan();

        // SOSOTB and DIDITB are dimensioned in FORTRAN at
        // (0:NBPNTU/2,0:NBPNTV/2,NDIMEN)

        size = (1 + nbpntu / 2) * (1 + nbpntv / 2) * ndimen;

        NCollection_HArray1<double> hsoso = new(1, size);
        Span<double> sosotb = hsoso.AsSpan();
        hsoso.Init(0.0);
        NCollection_HArray1<double> hdidi = new(1, size);
        Span<double> diditb = hdidi.AsSpan();
        hdidi.Init(0.0);

        // SODITB and DISOTB are dimensioned in FORTRAN at
        // (1:NBPNTU/2,1:NBPNTV/2,NDIMEN)

        size = (nbpntu / 2) * (nbpntv / 2) * ndimen;

        NCollection_HArray1<double> hsodi = new(1, size);
        Span<double> soditb = hsodi.AsSpan();
        hsodi.Init(0.0);
        NCollection_HArray1<double> hdiso = new(1, size);
        Span<double> disotb = hdiso.AsSpan();
        hdiso.Init(0.0);

        int iercod = 0;

        //  discretization of polynoms of interpolation
        ApproxF2var.mma2cdi_(ndimen,
                                        nbpntu,
                                        uroot,
                                        nbpntv,
                                        vroot,
                                        iordru,
                                        iordrv,
                                        c1,
                                        c2,
                                        c3,
                                        c4,
                                        su0,
                                        su1,
                                        du0,
                                        du1,
                                        sv0,
                                        sv1,
                                        dv0,
                                        dv1,
                                        sosotb,
                                        soditb,
                                        disotb,
                                        diditb,
                                        ref iercod);

        //  discretization of the square
        Span<double> udbfn = stackalloc double[2];
        Span<double> vdbfn = stackalloc double[2];
        udbfn[0] = _u0;
        udbfn[1] = _u1;
        vdbfn[0] = _v0;
        vdbfn[1] = _v1;

        size = Max(nbpntu, nbpntv);
        NCollection_HArray1<double> htable = new(1, size);
        Span<double> tab = htable.AsSpan();

        NCollection_HArray1<double> hpoints = new(1, size * ndimen);
        Span<double> pts = hpoints.AsSpan();

        ApproxF2var.mma2ds1_(ndimen,
                                        udbfn,
                                        vdbfn,
                                        func,
                                        nbpntu,
                                        nbpntv,
                                        uroot,
                                        vroot,
                                        isofav,
                                        sosotb,
                                        disotb,
                                        soditb,
                                        diditb,
                                        pts,
                                        tab,
                                        ref iercod);

        // the results are stored
        if (iercod == 0)
        {
            _discIsDone = true;
            _sosoTab = hsoso;
            _disoTab = hdiso;
            _sodiTab = hsodi;
            _didiTab = hdidi;
        }
        else
        {
            _discIsDone = false;
        }
    }

    public bool HasResult => _hasResult;

    public bool IsApproximated => _apprIsDone;

    /// <summary>Adds the interpolation of the constraints (isos and nodes) to the patch's polynomial.</summary>
    /// <exception cref="ConstructionErrorException">An error in the Fortran.</exception>
    public void AddConstraints(Context conditions, Framework constraints)
    {
        // data stored in the  Context
        int ndimen;
        int iercod = 0, ncflmu, ncflmv, nDegU, nDegV;
        ndimen = conditions.TotalDimension();
        // Attention : works only for 3D
        ncflmu = conditions.ULimit();
        ncflmv = conditions.VLimit();
        nDegU = ncflmu - 1;
        nDegV = ncflmv - 1;

        // data relative to the patch
        int iordru = _ordInU, iordrv = _ordInV;
        Span<double> patcan = _equation!.AsSpan();

        // curves of approximation of Isos U
        int size = ncflmv * ndimen;
        NCollection_HArray1<double> hIsoU0 = new(1, size * (iordru + 1));
        hIsoU0.ChangeArray1().Assign(constraints.IsoU(_u0, _v0, _v1).Polynom()!.Array1());
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> isoU0 = hIsoU0.AsSpan();
        NCollection_HArray1<int> hcfu0 = new(1, iordru + 1);
        Span<int> ncfu0 = hcfu0.AsSpan();
        hcfu0.Init(constraints.IsoU(_u0, _v0, _v1).CoeffCount);

        NCollection_HArray1<double> hIsoU1 = new(1, size * (iordru + 1));
        hIsoU1.ChangeArray1().Assign(constraints.IsoU(_u1, _v0, _v1).Polynom()!.Array1());
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> isoU1 = hIsoU1.AsSpan();
        NCollection_HArray1<int> hcfu1 = new(1, iordru + 1);
        Span<int> ncfu1 = hcfu1.AsSpan();
        hcfu1.Init(constraints.IsoU(_u1, _v0, _v1).CoeffCount);

        // normalization of Isos U
        int iu, iv;
        double du = (_u1 - _u0) / 2, dv = (_v1 - _v0) / 2, rho, valnorm;
        int ideb0, ideb1, jj;

        for (iu = 1; iu <= iordru; iu++)
        {
            rho = Math.Pow(du, iu);
            ideb0 = hIsoU0.Lower() + iu * size - 1;
            ideb1 = hIsoU1.Lower() + iu * size - 1;
            for (jj = 1; jj <= size; jj++)
            {
                hIsoU0.SetValue(ideb0 + jj, rho * hIsoU0.Value(ideb0 + jj));
                hIsoU1.SetValue(ideb1 + jj, rho * hIsoU1.Value(ideb1 + jj));
            }
        }

        // curves of approximation of Isos V
        size = ncflmu * ndimen;
        NCollection_HArray1<double> hIsoV0 = new(1, size * (iordrv + 1));
        hIsoV0.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v0).Polynom()!.Array1());
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> isoV0 = hIsoV0.AsSpan();
        NCollection_HArray1<int> hcfv0 = new(1, iordrv + 1);
        Span<int> ncfv0 = hcfv0.AsSpan();
        hcfv0.Init(constraints.IsoV(_u0, _u1, _v0).CoeffCount);

        NCollection_HArray1<double> hIsoV1 = new(1, size * (iordrv + 1));
        hIsoV1.ChangeArray1().Assign(constraints.IsoV(_u0, _u1, _v1).Polynom()!.Array1());
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> isoV1 = hIsoV1.AsSpan();
        NCollection_HArray1<int> hcfv1 = new(1, iordrv + 1);
        Span<int> ncfv1 = hcfv1.AsSpan();
        hcfv1.Init(constraints.IsoV(_u0, _u1, _v1).CoeffCount);

        //  normalization of Isos V
        for (iv = 1; iv <= iordrv; iv++)
        {
            rho = Math.Pow(dv, iv);
            ideb0 = hIsoV0.Lower() + iv * size - 1;
            ideb1 = hIsoV1.Lower() + iv * size - 1;
            for (jj = 1; jj <= size; jj++)
            {
                hIsoV0.SetValue(ideb0 + jj, rho * hIsoV0.Value(ideb0 + jj));
                hIsoV1.SetValue(ideb1 + jj, rho * hIsoV1.Value(ideb1 + jj));
            }
        }

        // add constraints to constant V
        NCollection_HArray1<double> hhermv = new(1, (2 * iordrv + 2) * (2 * iordrv + 2));
        Span<double> hermV = hhermv.AsSpan();
        if (iordrv >= 0)
        {
            ApproxF2var.mma1her_(iordrv, hermV, ref iercod);
            if (iercod != 0)
            {
                throw new ConstructionErrorException("AdvApp2Var_Patch::AddConstraints : Error in FORTRAN");
            }
            ApproxF2var.mma2ac2_(ndimen, nDegU, nDegV, iordrv, ncflmu, ncfv0, isoV0, ncfv1, isoV1, hermV,
                                            patcan);
        }

        // add constraints to constant U
        NCollection_HArray1<double> hhermu = new(1, (2 * iordru + 2) * (2 * iordru + 2));
        Span<double> hermU = hhermu.AsSpan();
        if (iordru >= 0)
        {
            ApproxF2var.mma1her_(iordru, hermU, ref iercod);
            if (iercod != 0)
            {
                throw new ConstructionErrorException("AdvApp2Var_Patch::AddConstraints : Error in FORTRAN");
            }
            ApproxF2var.mma2ac3_(ndimen, nDegU, nDegV, iordru, ncflmv, ncfu0, isoU0, ncfu1, isoU1, hermU,
                                            patcan);
        }

        // add constraints at the corners
        int ideb;
        size = ndimen * (iordru + 2) * (iordrv + 2);
        NCollection_HArray1<double> hcoins = new(1, size * 4);

        for (iu = 0; iu <= _ordInU; iu++)
        {
            for (iv = 0; iv <= _ordInV; iv++)
            {
                rho = Math.Pow(du, iu) * Math.Pow(dv, iv);

                // -F(U0,V0) and its derivatives normalized on (-1,1)
                ideb = hcoins.Lower() + ndimen * iu + ndimen * (iordru + 2) * iv - 1;
                valnorm = -rho * constraints.Node(_u0, _v0).Point(iu, iv).X;
                hcoins.SetValue(1 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u0, _v0).Point(iu, iv).Y;
                hcoins.SetValue(2 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u0, _v0).Point(iu, iv).Z;
                hcoins.SetValue(3 + ideb, valnorm);

                // -F(U1,V0) and its derivatives normalized on (-1,1)
                ideb += size;
                valnorm = -rho * constraints.Node(_u1, _v0).Point(iu, iv).X;
                hcoins.SetValue(1 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u1, _v0).Point(iu, iv).Y;
                hcoins.SetValue(2 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u1, _v0).Point(iu, iv).Z;
                hcoins.SetValue(3 + ideb, valnorm);

                // -F(U0,V1) and its derivatives normalized on (-1,1)
                ideb += size;
                valnorm = -rho * constraints.Node(_u0, _v1).Point(iu, iv).X;
                hcoins.SetValue(1 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u0, _v1).Point(iu, iv).Y;
                hcoins.SetValue(2 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u0, _v1).Point(iu, iv).Z;
                hcoins.SetValue(3 + ideb, valnorm);

                // -F(U1,V1) and its derivatives normalized on (-1,1)
                ideb += size;
                valnorm = -rho * constraints.Node(_u1, _v1).Point(iu, iv).X;
                hcoins.SetValue(1 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u1, _v1).Point(iu, iv).Y;
                hcoins.SetValue(2 + ideb, valnorm);
                valnorm = -rho * constraints.Node(_u1, _v1).Point(iu, iv).Z;
                hcoins.SetValue(3 + ideb, valnorm);
            }
        }

        //  tables required for FORTRAN
        int iordmx = Max(iordru, iordrv);
        NCollection_HArray1<double> hextr = new(1, 2 * iordmx + 2);
        Span<double> extr = hextr.AsSpan();
        NCollection_HArray1<double> hfact = new(1, iordmx + 1);
        Span<double> fact = hfact.AsSpan();

        int idim, ncf0, ncf1, iun = 1;
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> Is;

        // add extremities of isos U
        for (iu = 1; iu <= iordru + 1; iu++)
        {
            ncf0 = hcfu0.Value(hcfu0.Lower() + iu - 1);
            ncf1 = hcfu1.Value(hcfu1.Lower() + iu - 1);
            for (idim = 1; idim <= ndimen; idim++)
            {
                Is = isoU0 + (ncflmv * (idim - 1) + ncflmv * ndimen * (iu - 1));
                MathBase.mmdrc11_(iordrv, iun, ncf0, Is, extr, fact);
                for (iv = 1; iv <= iordrv + 1; iv++)
                {
                    ideb = hcoins.Lower() + ndimen * (iu - 1) + ndimen * (iordru + 2) * (iv - 1) - 1;
                    hcoins.ChangeValue(idim + ideb) += hextr.Value(1 + 2 * (iv - 1));
                    hcoins.ChangeValue(2 * size + idim + ideb) += hextr.Value(2 + 2 * (iv - 1));
                }
                Is = isoU1 + (ncflmv * (idim - 1) + ncflmv * ndimen * (iu - 1));
                MathBase.mmdrc11_(iordrv, iun, ncf1, Is, extr, fact);
                for (iv = 1; iv <= iordrv + 1; iv++)
                {
                    ideb = hcoins.Lower() + ndimen * (iu - 1) + ndimen * (iordru + 2) * (iv - 1) - 1;
                    hcoins.ChangeValue(size + idim + ideb) += hextr.Value(1 + 2 * (iv - 1));
                    hcoins.ChangeValue(3 * size + idim + ideb) += hextr.Value(2 + 2 * (iv - 1));
                }
            }
        }

        // add extremities of isos V
        for (iv = 1; iv <= iordrv + 1; iv++)
        {
            ncf0 = hcfv0.Value(hcfv0.Lower() + iv - 1);
            ncf1 = hcfv1.Value(hcfv1.Lower() + iv - 1);
            for (idim = 1; idim <= ndimen; idim++)
            {
                Is = isoV0 + (ncflmu * (idim - 1) + ncflmu * ndimen * (iv - 1));
                MathBase.mmdrc11_(iordru, iun, ncf0, Is, extr, fact);
                for (iu = 1; iu <= iordru + 1; iu++)
                {
                    ideb = hcoins.Lower() + ndimen * (iu - 1) + ndimen * (iordru + 2) * (iv - 1) - 1;
                    hcoins.ChangeValue(idim + ideb) += hextr.Value(1 + 2 * (iu - 1));
                    hcoins.ChangeValue(size + idim + ideb) += hextr.Value(2 + 2 * (iu - 1));
                }
                Is = isoV1 + (ncflmu * (idim - 1) + ncflmu * ndimen * (iv - 1));
                MathBase.mmdrc11_(iordru, iun, ncf1, Is, extr, fact);
                for (iu = 1; iu <= iordru + 1; iu++)
                {
                    ideb = hcoins.Lower() + ndimen * (iu - 1) + ndimen * (iordru + 2) * (iv - 1) - 1;
                    hcoins.ChangeValue(2 * size + idim + ideb) += hextr.Value(1 + 2 * (iu - 1));
                    hcoins.ChangeValue(3 * size + idim + ideb) += hextr.Value(2 + 2 * (iu - 1));
                }
            }
        }

        // add all to PATCAN
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c1 = hcoins.AsSpan();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c2 = c1 + size;
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c3 = c2 + size;
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> c4 = c3 + size;
        if (iordru >= 0 && iordrv >= 0)
        {
            ApproxF2var.mma2ac1_(ndimen, nDegU, nDegV, iordru, iordrv, c1, c2, c3, c4, hermU, hermV,
                                            patcan);
        }
    }

    /// <summary>Adds the errors of the constraints (isos and nodes) to the patch's errors.</summary>
    public void AddErrors(Framework constraints)
    {
        int nbsesp = 1, iesp;
        int iu, iv;

        double errU, errV, error;
        ReadOnlySpan<double> hmax = [0, 1, 1.5, 1.75];

        for (iesp = 1; iesp <= nbsesp; iesp++)
        {
            //  error max in sub-space iesp
            errU = 0.0;
            for (iv = 1; iv <= _ordInV + 1; iv++)
            {
                error = constraints.IsoV(_u0, _u1, _v0).MaxErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
                error = constraints.IsoV(_u0, _u1, _v1).MaxErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
            }
            errV = 0.0;
            for (iu = 1; iu <= _ordInU + 1; iu++)
            {
                error = constraints.IsoU(_u0, _v0, _v1).MaxErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
                error = constraints.IsoU(_u1, _v0, _v1).MaxErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
            }
            _maxErrors!.ChangeValue(iesp) += errU * hmax[_ordInV + 1] + errV * hmax[_ordInU + 1];

            // average error in sub-space iesp
            errU = 0.0;
            for (iv = 1; iv <= _ordInV + 1; iv++)
            {
                error = constraints.IsoV(_u0, _u1, _v0).MoyErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
                error = constraints.IsoV(_u0, _u1, _v1).MoyErrors()!.Value(iesp, iv);
                errU = Max(errU, error);
            }
            errV = 0.0;
            for (iu = 1; iu <= _ordInU + 1; iu++)
            {
                error = constraints.IsoU(_u0, _v0, _v1).MoyErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
                error = constraints.IsoU(_u1, _v0, _v1).MoyErrors()!.Value(iesp, iu);
                errV = Max(errV, error);
            }
            error = _moyErrors!.Value(iesp);
            error *= error;
            error += errU * hmax[_ordInV + 1] * errU * hmax[_ordInV + 1]
                     + errV * hmax[_ordInU + 1] * errV * hmax[_ordInU + 1];
            _moyErrors.SetValue(iesp, Math.Sqrt(error));

            // max errors at iso-borders
            NCollection_HArray2<double> heriso = new(1, nbsesp, 1, 4);
            heriso.SetValue(iesp, 1, constraints.IsoV(_u0, _u1, _v0).MaxErrors()!.Value(iesp, 1));
            heriso.SetValue(iesp, 2, constraints.IsoV(_u0, _u1, _v1).MaxErrors()!.Value(iesp, 1));
            heriso.SetValue(iesp, 3, constraints.IsoU(_u0, _v0, _v1).MaxErrors()!.Value(iesp, 1));
            heriso.SetValue(iesp, 4, constraints.IsoU(_u1, _v0, _v1).MaxErrors()!.Value(iesp, 1));

            // calculate max errors at the corners
            double emax1 = 0.0, emax2 = 0.0, emax3 = 0.0, emax4 = 0.0, err1, err2, err3, err4;
            for (iu = 0; iu <= _ordInU; iu++)
            {
                for (iv = 0; iv <= _ordInV; iv++)
                {
                    error = constraints.Node(_u0, _v0).Error(iu, iv);
                    emax1 = Max(emax1, error);
                    error = constraints.Node(_u1, _v0).Error(iu, iv);
                    emax2 = Max(emax2, error);
                    error = constraints.Node(_u0, _v1).Error(iu, iv);
                    emax3 = Max(emax3, error);
                    error = constraints.Node(_u1, _v1).Error(iu, iv);
                    emax4 = Max(emax4, error);
                }
            }

            // calculate max errors on borders
            err1 = Max(emax1, emax2);
            err2 = Max(emax3, emax4);
            err3 = Max(emax1, emax3);
            err4 = Max(emax2, emax4);

            //   calculate final errors on internal isos
            if (constraints.IsoV(_u0, _u1, _v0).Position() == 0)
            {
                heriso.ChangeValue(iesp, 1) += err1 * hmax[_ordInU + 1];
            }
            if (constraints.IsoV(_u0, _u1, _v1).Position() == 0)
            {
                heriso.ChangeValue(iesp, 2) += err2 * hmax[_ordInU + 1];
            }
            if (constraints.IsoU(_u0, _v0, _v1).Position() == 0)
            {
                heriso.ChangeValue(iesp, 3) += err3 * hmax[_ordInV + 1];
            }
            if (constraints.IsoU(_u1, _v0, _v1).Position() == 0)
            {
                heriso.ChangeValue(iesp, 4) += err4 * hmax[_ordInV + 1];
            }
            _isoErrors = heriso;
        }
    }

    /// <summary>
    /// Approximates the discretised patch. <paramref name="numDec"/> says which cuts are allowed (0 none, 1 or 3 in
    /// U, 2 or 4 in V, 5 both); <see cref="CutSense()"/> then says which one the approximation asks for.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An error in the Fortran.</exception>
    public void MakeApprox(Context conditions, Framework constraints, int numDec)
    {
        // data stored in the Context
        int ndimen, nbsesp, ndimse;
        int nbpntu, nbpntv, ncflmu, ncflmv, ndjacu, ndjacv;
        int nDegU, nDegV, nJacU, nJacV;
        ndimen = conditions.TotalDimension();
        nbsesp = conditions.TotalNumberSSP();
        ndimse = 3;
        nbpntu = conditions.URoots()!.Length();
        if (_ordInU > -1)
        {
            nbpntu -= 2;
        }
        nbpntv = conditions.VRoots()!.Length();
        if (_ordInV > -1)
        {
            nbpntv -= 2;
        }
        ncflmu = conditions.ULimit();
        ncflmv = conditions.VLimit();
        nDegU = ncflmu - 1;
        nDegV = ncflmv - 1;
        ndjacu = conditions.UJacDeg();
        ndjacv = conditions.VJacDeg();
        nJacU = ndjacu + 1;
        nJacV = ndjacv + 1;

        // data relative to the processed patch
        int iordru = _ordInU, iordrv = _ordInV, ndminu = 1, ndminv = 1, ncoefu, ncoefv;
        // NDMINU and NDMINV depend on the nb of coeff of neighboring isos
        // and of the required order of continuity
        ndminu = Max(1, 2 * iordru + 1);
        ncoefu = constraints.IsoV(_u0, _u1, _v0).CoeffCount - 1;
        ndminu = Max(ndminu, ncoefu);
        ncoefu = constraints.IsoV(_u0, _u1, _v1).CoeffCount - 1;
        ndminu = Max(ndminu, ncoefu);

        ndminv = Max(1, 2 * iordrv + 1);
        ncoefv = constraints.IsoU(_u0, _v0, _v1).CoeffCount - 1;
        ndminv = Max(ndminv, ncoefv);
        ncoefv = constraints.IsoU(_u1, _v0, _v1).CoeffCount - 1;
        ndminv = Max(ndminv, ncoefv);

        // tables of approximations
        NCollection_HArray1<double> hepsapr = new(1, nbsesp);
        NCollection_HArray1<double> hepsfro = new(1, nbsesp * 8);
        int iesp;
        for (iesp = 1; iesp <= nbsesp; iesp++)
        {
            hepsapr.SetValue(iesp, conditions.IToler()!.Value(iesp));
            hepsfro.SetValue(iesp, conditions.FToler()!.Value(iesp, 1));
            hepsfro.SetValue(iesp + nbsesp, conditions.FToler()!.Value(iesp, 2));
            hepsfro.SetValue(iesp + 2 * nbsesp, conditions.FToler()!.Value(iesp, 3));
            hepsfro.SetValue(iesp + 3 * nbsesp, conditions.FToler()!.Value(iesp, 4));
            hepsfro.SetValue(iesp + 4 * nbsesp, conditions.CToler()!.Value(iesp, 1));
            hepsfro.SetValue(iesp + 5 * nbsesp, conditions.CToler()!.Value(iesp, 2));
            hepsfro.SetValue(iesp + 6 * nbsesp, conditions.CToler()!.Value(iesp, 3));
            hepsfro.SetValue(iesp + 7 * nbsesp, conditions.CToler()!.Value(iesp, 4));
        }
        Span<double> epsapr = hepsapr.AsSpan();
        Span<double> epsfro = hepsfro.AsSpan();

        int size = (1 + ndjacu) * (1 + ndjacv) * ndimen;
        NCollection_HArray1<double> hpjac = new(1, size);
        Span<double> patjac = hpjac.AsSpan();
        size = 2 * size;
        NCollection_HArray1<double> hpaux = new(1, size);
        Span<double> pataux = hpaux.AsSpan();
        size = ncflmu * ncflmv * ndimen;
        NCollection_HArray1<double> hpcan = new(1, size);
        Span<double> patcan = hpcan.AsSpan();
        NCollection_HArray1<double> herrmax = new(1, nbsesp);
        Span<double> errmax = herrmax.AsSpan();
        NCollection_HArray1<double> herrmoy = new(1, nbsesp);
        Span<double> errmoy = herrmoy.AsSpan();

        // tables of discretization of the square
        Span<double> sosotb = _sosoTab!.AsSpan();
        Span<double> disotb = _disoTab!.AsSpan();
        Span<double> soditb = _sodiTab!.AsSpan();
        Span<double> diditb = _didiTab!.AsSpan();

        //  approximation
        int itydec = 0, iercod = 0;
        int iun = 1, itrois = 3;
        ncoefu = 0;
        ncoefv = 0;
        ApproxF2var.mma2ce1_(numDec,
                                        ndimen,
                                        nbsesp,
                                        global::Kerf.Engine.Approximation.Advanced.F2c.In(in ndimse),
                                        ndminu,
                                        ndminv,
                                        nDegU,
                                        nDegV,
                                        ndjacu,
                                        ndjacv,
                                        iordru,
                                        iordrv,
                                        nbpntu,
                                        nbpntv,
                                        epsapr,
                                        sosotb,
                                        disotb,
                                        soditb,
                                        diditb,
                                        patjac,
                                        errmax,
                                        errmoy,
                                        ref ncoefu,
                                        ref ncoefv,
                                        ref itydec,
                                        ref iercod);

        // results
        _cutSense = itydec;
        if (itydec == 0 && iercod <= 0)
        {
            _hasResult = true;
            _apprIsDone = iercod == 0;
            _nbCoeffInU = ncoefu + 1;
            _nbCoeffInV = ncoefv + 1;
            _maxErrors = herrmax;
            _moyErrors = herrmoy;

            // Passage to canonic on [-1,1]
            MathBase.mmfmca9_(nJacU, nJacV, ndimen, _nbCoeffInU, _nbCoeffInV, ndimen, patjac, patjac);
            ApproxF2var.mma2can_(ncflmu,
                                            ncflmv,
                                            ndimen,
                                            _ordInU,
                                            _ordInV,
                                            _nbCoeffInU,
                                            _nbCoeffInV,
                                            patjac,
                                            pataux,
                                            patcan,
                                            ref iercod);
            if (iercod != 0)
            {
                throw new ConstructionErrorException("AdvApp2Var_Patch::MakeApprox : Error in FORTRAN");
            }
            _equation = hpcan;

            // Add constraints and errors
            AddConstraints(conditions, constraints);
            AddErrors(constraints);

            // Reduction of degrees if possible
            patcan = _equation.AsSpan();

            ApproxF2var.mma2fx6_(ncflmu,
                                            ncflmv,
                                            ndimen,
                                            nbsesp,
                                            global::Kerf.Engine.Approximation.Advanced.F2c.In(in itrois),
                                            iun,
                                            iun,
                                            iordru,
                                            iordrv,
                                            epsapr,
                                            epsfro,
                                            patcan,
                                            errmax,
                                            global::Kerf.Engine.Approximation.Advanced.F2c.Ref(ref _nbCoeffInU),
                                            global::Kerf.Engine.Approximation.Advanced.F2c.Ref(ref _nbCoeffInV));

            // transposition (NCFLMU,NCFLMV,NDIMEN)Fortran-C++
            int iu, aIN, dim, ii, jj;
            for (dim = 1; dim <= ndimen; dim++)
            {
                aIN = (dim - 1) * ncflmu * ncflmv;
                for (ii = 1; ii <= ncflmu; ii++)
                {
                    iu = (ii - 1) * ndimen * ncflmv;
                    for (jj = 1; jj <= ncflmv; jj++)
                    {
                        hpaux.SetValue(dim + ndimen * (jj - 1) + iu, _equation.Value(ii + ncflmu * (jj - 1) + aIN));
                    }
                }
            }
            _equation = hpaux;
        }
        else
        {
            _apprIsDone = false;
            _hasResult = false;
        }
    }

    public void ChangeDomain(double a, double b, double c, double d)
    {
        _u0 = a;
        _u1 = b;
        _v0 = c;
        _v1 = d;
    }

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

    public double U0() => _u0;

    public double U1() => _u1;

    public double V0() => _v0;

    public double V1() => _v1;

    public int UOrder() => _ordInU;

    public int VOrder() => _ordInV;

    /// <summary>The cut the last approximation asked for: 0 none, 1 in U, 2 in V, 3 both.</summary>
    public int CutSense() => _cutSense;

    /// <summary>The cut that <paramref name="crit"/> asks for: 0 if it is satisfied, else <paramref name="numDec"/>.</summary>
    public int CutSense(Criterion crit, int numDec)
    {
        bool critRel = crit.Type() == Relative;
        if (critRel && !IsApproximated)
        {
            return _cutSense;
        }
        else
        {
            if (crit.IsSatisfied(this))
            {
                return 0;
            }
            else
            {
                return numDec;
            }
        }
    }

    public int CoeffInUCount => _nbCoeffInU;

    public int CoeffInVCount => _nbCoeffInV;

    public void ChangeNbCoeff(int nbCoeffU, int nbCoeffV)
    {
        if (_nbCoeffInU < nbCoeffU)
        {
            _nbCoeffInU = nbCoeffU;
        }
        if (_nbCoeffInV < nbCoeffV)
        {
            _nbCoeffInV = nbCoeffV;
        }
    }

    /// <summary>The poles of the patch's polynomial on [-1, 1] x [-1, 1], as a Bezier patch.</summary>
    /// <exception cref="ConstructionErrorException">Unless the context has one subspace and SSPIndex is 1.</exception>
    public NCollection_HArray2<Point3D> Poles(int sspIndex, Context cond)
    {
        NCollection_HArray1<double>? sousEquation;
        if (cond.TotalNumberSSP() == 1 && sspIndex == 1)
        {
            sousEquation = _equation;
        }
        else
        {
            throw new ConstructionErrorException("AdvApp2Var_Patch::Poles :  SSPIndex out of range");
        }
        NCollection_Array1<double> intervalle = new(1, 2);
        intervalle.SetValue(1, -1);
        intervalle.SetValue(2, 1);
        NCollection_Array1<int> nbCoeff = new(1, 2);
        nbCoeff.SetValue(1, _nbCoeffInU);
        nbCoeff.SetValue(2, _nbCoeffInV);
        GridPolynomialToPoles conv = new(cond.ULimit() - 1,
                                                 cond.VLimit() - 1,
                                                 nbCoeff,
                                                 sousEquation!.Array1(),
                                                 intervalle,
                                                 intervalle);
        return new NCollection_HArray2<Point3D>(conv.Poles());
    }

    /// <summary>The coefficients of the patch's polynomial on [-1, 1] x [-1, 1] (shared: don't modify).</summary>
    /// <exception cref="ConstructionErrorException">Unless the context has one subspace and SSPIndex is 1.</exception>
    public NCollection_HArray1<double>? Coefficients(int sspIndex, Context cond)
    {
        NCollection_HArray1<double>? sousEquation;
        if (cond.TotalNumberSSP() == 1 && sspIndex == 1)
        {
            sousEquation = _equation;
        }
        else
        {
            throw new ConstructionErrorException("AdvApp2Var_Patch::Poles :  SSPIndex out of range");
        }
        return sousEquation;
    }

    public NCollection_HArray1<double>? MaxErrors() => _maxErrors;

    public NCollection_HArray1<double>? AverageErrors() => _moyErrors;

    public NCollection_HArray2<double>? IsoErrors() => _isoErrors;

    public double CritValue() => _critValue;

    public void SetCritValue(double dist) => _critValue = dist;
}
