// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Context.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The parameters of <see cref="ApproxAFunc2Var"/>: continuity orders, degree limits, the Gauss points
/// and Jacobi tables of each direction, and the tolerances of each subspace.
/// </summary>
/// <remarks>
/// OCCT copies it by value; it has no mutators, so the port shares it. The arrays it returns are its own (shared:
/// don't modify); they are null for a default-constructed context, as OCCT's handles are.
/// </remarks>
public class Context
{
    private readonly int _fav;
    private readonly int _ordU;
    private readonly int _ordV;
    private readonly int _limU;
    private readonly int _limV;
    private readonly int _nb1DSS;
    private readonly int _nb2DSS;
    private readonly int _nb3DSS;
    private readonly int _nbURoot;
    private readonly int _nbVRoot;
    private readonly int _jDegU;
    private readonly int _jDegV;
    private readonly NCollection_HArray1<double>? _jMaxU;
    private readonly NCollection_HArray1<double>? _jMaxV;
    private readonly NCollection_HArray1<double>? _uRoots;
    private readonly NCollection_HArray1<double>? _vRoots;
    private readonly NCollection_HArray1<double>? _uGauss;
    private readonly NCollection_HArray1<double>? _vGauss;
    private readonly NCollection_HArray1<double>? _internalTol;
    private readonly NCollection_HArray2<double>? _frontierTol;
    private readonly NCollection_HArray2<double>? _cuttingTol;

    // Calculation of parameters
    private static bool Lesparam(int iordre, int ncflim, int icodeo, ref int nbpnts, ref int ndgjac)
    {
        // jacobi degree
        ndgjac = ncflim; // it always keeps a reserve coefficient
        if (icodeo < 0)
        {
            return false;
        }
        if (icodeo > 0)
        {
            ndgjac += 9 - (iordre + 1); // iordre rescales the frequences upwards
            ndgjac += (icodeo - 1) * 10;
        }
        // ---> Min Number of required points.
        if (ndgjac < 8)
        {
            nbpnts = 8;
        }
        else if (ndgjac < 10)
        {
            nbpnts = 10;
        }
        //  else if (ndgjac < 15) { nbpnt = 15; } Bug Uneven number
        else if (ndgjac < 20)
        {
            nbpnts = 20;
        }
        //  else if (ndgjac < 25) { nbpnt = 25; } Bug Uneven number
        else if (ndgjac < 30)
        {
            nbpnts = 30;
        }
        else if (ndgjac < 40)
        {
            nbpnts = 40;
        }
        else if (ndgjac < 50)
        {
            nbpnts = 50;
        }
        //  else if (*ndgjac < 61) { nbpnt = 61;} Bug Uneven number
        else
        {
            nbpnts = 50;
        }

        // If constraints are on borders, this adds 2 points
        if (iordre > -1)
        {
            nbpnts += 2;
        }

        return true;
    }

    private static void AppendInternalTolerance(NCollection_HArray1<double>? source,
                                                int count,
                                                int offset,
                                                NCollection_HArray1<double> target)
    {
        for (int sspIndex = 1; sspIndex <= count; ++sspIndex)
        {
            target.SetValue(offset + sspIndex, source!.Value(sspIndex));
        }
    }

    private static void AppendFrontierTolerance(NCollection_HArray2<double>? source,
                                                int count,
                                                int offset,
                                                NCollection_HArray2<double> frontierTol,
                                                NCollection_HArray2<double> cuttingTol)
    {
        for (int sspIndex = 1; sspIndex <= count; ++sspIndex)
        {
            int globalSSPIndex = offset + sspIndex;
            for (int tolIndex = 1; tolIndex <= 4; ++tolIndex)
            {
                frontierTol.SetValue(globalSSPIndex, tolIndex, source!.Value(sspIndex, tolIndex));
                cuttingTol.SetValue(globalSSPIndex, tolIndex, 0.0);
            }
        }
    }

    private static double HMaxFactor(int order)
    {
        switch (order)
        {
            case -1:
                return 0.0;
            case 0:
                return 1.0;
            case 1:
                return 1.5;
            default:
                return 1.75;
        }
    }

    public Context()
    {
    }

    /// <summary>
    /// Builds the context. <paramref name="ifav"/> is the favoured iso (1 for U, 2 for V), <paramref name="iu"/>
    /// and <paramref name="iv"/> the continuity orders (-1 to 2), <paramref name="nlimu"/> and
    /// <paramref name="nlimv"/> the maximum numbers of coefficients, <paramref name="iprecis"/> the precision code
    /// (0 to 3). The tolerance arrays of a dimension without subspaces may be null.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A negative precision code, or an error in the Fortran.</exception>
    public Context(int ifav,
                              int iu,
                              int iv,
                              int nlimu,
                              int nlimv,
                              int iprecis,
                              int nb1Dss,
                              int nb2Dss,
                              int nb3Dss,
                              NCollection_HArray1<double>? tol1D,
                              NCollection_HArray1<double>? tol2D,
                              NCollection_HArray1<double>? tol3D,
                              NCollection_HArray2<double>? tof1D,
                              NCollection_HArray2<double>? tof2D,
                              NCollection_HArray2<double>? tof3D)
    {
        _fav = ifav;
        _ordU = iu;
        _ordV = iv;
        _limU = nlimu;
        _limV = nlimv;
        _nb1DSS = nb1Dss;
        _nb2DSS = nb2Dss;
        _nb3DSS = nb3Dss;

        int errorCode = 0, nbPntU = 0, jDegU = 0, nbPntV = 0, jDegV = 0;
        int coeffLimit;
        int orderU = iu;
        int orderV = iv;

        // myNbURoot,myJDegU
        coeffLimit = nlimu;
        if (coeffLimit < 2 * iu + 2)
        {
            coeffLimit = 2 * iu + 2;
        }
        if (!Lesparam(iu, coeffLimit, iprecis, ref nbPntU, ref jDegU))
        {
            throw new ConstructionErrorException("AdvApp2Var_Context");
        }
        _nbURoot = nbPntU;
        _jDegU = jDegU;
        if (iu > -1)
        {
            nbPntU = _nbURoot - 2;
        }

        // myJMaxU
        int size = jDegU - 2 * iu - 1;
        NCollection_HArray1<double> jMaxU = new(1, size);
        ApproxF2var.mma2jmx_(jDegU, orderU, jMaxU.AsSpan());
        _jMaxU = jMaxU;

        // myNbVRoot,myJDegV
        coeffLimit = nlimv;
        if (coeffLimit < 2 * iv + 2)
        {
            coeffLimit = 2 * iv + 2;
        }
        if (!Lesparam(iv, coeffLimit, iprecis, ref nbPntV, ref jDegV))
        {
            throw new ConstructionErrorException("AdvApp2Var_Context");
        }
        _nbVRoot = nbPntV;
        _jDegV = jDegV;
        if (iv > -1)
        {
            nbPntV = _nbVRoot - 2;
        }

        // myJMaxV
        size = jDegV - 2 * iv - 1;
        NCollection_HArray1<double> jMaxV = new(1, size);
        ApproxF2var.mma2jmx_(jDegV, orderV, jMaxV.AsSpan());
        _jMaxV = jMaxV;

        // myURoots, myVRoots
        NCollection_HArray1<double> uRoots = new(1, _nbURoot);
        NCollection_HArray1<double> vRoots = new(1, _nbVRoot);
        ApproxF2var.mma2roo_(nbPntU, nbPntV, uRoots.AsSpan(), vRoots.AsSpan());
        _uRoots = uRoots;
        _vRoots = vRoots;

        // myUGauss
        size = (nbPntU / 2 + 1) * (_jDegU - 2 * iu - 1);
        NCollection_HArray1<double> uGauss = new(1, size);
        ApproxF2var.mmapptt_(jDegU, nbPntU, orderU, uGauss.AsSpan(), ref errorCode);
        if (errorCode != 0)
        {
            throw new ConstructionErrorException("AdvApp2Var_Context : Error in FORTRAN");
        }
        _uGauss = uGauss;

        // myVGauss
        size = (nbPntV / 2 + 1) * (_jDegV - 2 * iv - 1);
        NCollection_HArray1<double> vGauss = new(1, size);
        ApproxF2var.mmapptt_(jDegV, nbPntV, orderV, vGauss.AsSpan(), ref errorCode);
        if (errorCode != 0)
        {
            throw new ConstructionErrorException("AdvApp2Var_Context : Error in FORTRAN");
        }
        _vGauss = vGauss;

        // myInternalTol, myFrontierTol, myCuttingTol
        int nbSSP = nb1Dss + nb2Dss + nb3Dss;
        NCollection_HArray1<double> iTol = new(1, nbSSP);
        AppendInternalTolerance(tol1D, nb1Dss, 0, iTol);
        AppendInternalTolerance(tol2D, nb2Dss, nb1Dss, iTol);
        AppendInternalTolerance(tol3D, nb3Dss, nb1Dss + nb2Dss, iTol);
        if (iu > -1 || iv > -1)
        {
            for (int sspIndex = 1; sspIndex <= nbSSP; ++sspIndex)
            {
                iTol.SetValue(sspIndex, iTol.Value(sspIndex) / 2);
            }
        }
        NCollection_HArray2<double> fTol = new(1, nbSSP, 1, 4);
        NCollection_HArray2<double> cTol = new(1, nbSSP, 1, 4);
        AppendFrontierTolerance(tof1D, nb1Dss, 0, fTol, cTol);
        AppendFrontierTolerance(tof2D, nb2Dss, nb1Dss, fTol, cTol);
        AppendFrontierTolerance(tof3D, nb3Dss, nb1Dss + nb2Dss, fTol, cTol);
        if (iu > -1 || iv > -1)
        {
            double tolMin;
            double hMaxU = HMaxFactor(iu);
            double hMaxV = HMaxFactor(iv);
            double weight = hMaxU * hMaxV + hMaxU + hMaxV;
            for (int sspIndex = 1; sspIndex <= nbSSP; ++sspIndex)
            {
                for (int tolIndex = 1; tolIndex <= 4; ++tolIndex)
                {
                    tolMin = iTol.Value(sspIndex) / weight;
                    if (tolMin < fTol.Value(sspIndex, tolIndex))
                    {
                        fTol.SetValue(sspIndex, tolIndex, tolMin);
                    }
                    cTol.SetValue(sspIndex, tolIndex, tolMin);
                }
            }
        }
        _internalTol = iTol;
        _frontierTol = fTol;
        _cuttingTol = cTol;
    }

    public int TotalDimension() => _nb1DSS + 2 * _nb2DSS + 3 * _nb3DSS;

    public int TotalNumberSSP() => _nb1DSS + _nb2DSS + _nb3DSS;

    public int FavorIso() => _fav;

    public int UOrder() => _ordU;

    public int VOrder() => _ordV;

    public int ULimit() => _limU;

    public int VLimit() => _limV;

    public int UJacDeg() => _jDegU;

    public int VJacDeg() => _jDegV;

    public NCollection_HArray1<double>? UJacMax() => _jMaxU;

    public NCollection_HArray1<double>? VJacMax() => _jMaxV;

    public NCollection_HArray1<double>? URoots() => _uRoots;

    public NCollection_HArray1<double>? VRoots() => _vRoots;

    public NCollection_HArray1<double>? UGauss() => _uGauss;

    public NCollection_HArray1<double>? VGauss() => _vGauss;

    public NCollection_HArray1<double>? IToler() => _internalTol;

    public NCollection_HArray2<double>? FToler() => _frontierTol;

    public NCollection_HArray2<double>? CToler() => _cuttingTol;
}
