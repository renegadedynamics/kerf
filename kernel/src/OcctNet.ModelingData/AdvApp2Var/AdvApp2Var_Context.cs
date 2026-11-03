// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Context.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The parameters of <see cref="AdvApp2Var_ApproxAFunc2Var"/>: continuity orders, degree limits, the Gauss points
/// and Jacobi tables of each direction, and the tolerances of each subspace.
/// </summary>
/// <remarks>
/// OCCT copies it by value; it has no mutators, so the port shares it. The arrays it returns are its own (shared:
/// don't modify); they are null for a default-constructed context, as OCCT's handles are.
/// </remarks>
public class AdvApp2Var_Context
{
    private readonly int myFav;
    private readonly int myOrdU;
    private readonly int myOrdV;
    private readonly int myLimU;
    private readonly int myLimV;
    private readonly int myNb1DSS;
    private readonly int myNb2DSS;
    private readonly int myNb3DSS;
    private readonly int myNbURoot;
    private readonly int myNbVRoot;
    private readonly int myJDegU;
    private readonly int myJDegV;
    private readonly NCollection_HArray1<double>? myJMaxU;
    private readonly NCollection_HArray1<double>? myJMaxV;
    private readonly NCollection_HArray1<double>? myURoots;
    private readonly NCollection_HArray1<double>? myVRoots;
    private readonly NCollection_HArray1<double>? myUGauss;
    private readonly NCollection_HArray1<double>? myVGauss;
    private readonly NCollection_HArray1<double>? myInternalTol;
    private readonly NCollection_HArray2<double>? myFrontierTol;
    private readonly NCollection_HArray2<double>? myCuttingTol;

    // Calculation of parameters
    private static bool lesparam(int iordre, int ncflim, int icodeo, ref int nbpnts, ref int ndgjac)
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

    private static void appendInternalTolerance(NCollection_HArray1<double>? theSource,
                                                int theCount,
                                                int theOffset,
                                                NCollection_HArray1<double> theTarget)
    {
        for (int anSSPIndex = 1; anSSPIndex <= theCount; ++anSSPIndex)
        {
            theTarget.SetValue(theOffset + anSSPIndex, theSource!.Value(anSSPIndex));
        }
    }

    private static void appendFrontierTolerance(NCollection_HArray2<double>? theSource,
                                                int theCount,
                                                int theOffset,
                                                NCollection_HArray2<double> theFrontierTol,
                                                NCollection_HArray2<double> theCuttingTol)
    {
        for (int anSSPIndex = 1; anSSPIndex <= theCount; ++anSSPIndex)
        {
            int aGlobalSSPIndex = theOffset + anSSPIndex;
            for (int aTolIndex = 1; aTolIndex <= 4; ++aTolIndex)
            {
                theFrontierTol.SetValue(aGlobalSSPIndex, aTolIndex, theSource!.Value(anSSPIndex, aTolIndex));
                theCuttingTol.SetValue(aGlobalSSPIndex, aTolIndex, 0.0);
            }
        }
    }

    private static double hMaxFactor(int theOrder)
    {
        switch (theOrder)
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

    public AdvApp2Var_Context()
    {
    }

    /// <summary>
    /// Builds the context. <paramref name="ifav"/> is the favoured iso (1 for U, 2 for V), <paramref name="iu"/>
    /// and <paramref name="iv"/> the continuity orders (-1 to 2), <paramref name="nlimu"/> and
    /// <paramref name="nlimv"/> the maximum numbers of coefficients, <paramref name="iprecis"/> the precision code
    /// (0 to 3). The tolerance arrays of a dimension without subspaces may be null.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A negative precision code, or an error in the Fortran.</exception>
    public AdvApp2Var_Context(int ifav,
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
        myFav = ifav;
        myOrdU = iu;
        myOrdV = iv;
        myLimU = nlimu;
        myLimV = nlimv;
        myNb1DSS = nb1Dss;
        myNb2DSS = nb2Dss;
        myNb3DSS = nb3Dss;

        int anErrorCode = 0, NbPntU = 0, JDegU = 0, NbPntV = 0, JDegV = 0;
        int aCoeffLimit;
        int anOrderU = iu;
        int anOrderV = iv;

        // myNbURoot,myJDegU
        aCoeffLimit = nlimu;
        if (aCoeffLimit < 2 * iu + 2)
        {
            aCoeffLimit = 2 * iu + 2;
        }
        if (!lesparam(iu, aCoeffLimit, iprecis, ref NbPntU, ref JDegU))
        {
            throw new ConstructionErrorException("AdvApp2Var_Context");
        }
        myNbURoot = NbPntU;
        myJDegU = JDegU;
        if (iu > -1)
        {
            NbPntU = myNbURoot - 2;
        }

        // myJMaxU
        int aSize = JDegU - 2 * iu - 1;
        NCollection_HArray1<double> JMaxU = new(1, aSize);
        AdvApp2Var_ApproxF2var.mma2jmx_(JDegU, anOrderU, JMaxU.AsSpan());
        myJMaxU = JMaxU;

        // myNbVRoot,myJDegV
        aCoeffLimit = nlimv;
        if (aCoeffLimit < 2 * iv + 2)
        {
            aCoeffLimit = 2 * iv + 2;
        }
        if (!lesparam(iv, aCoeffLimit, iprecis, ref NbPntV, ref JDegV))
        {
            throw new ConstructionErrorException("AdvApp2Var_Context");
        }
        myNbVRoot = NbPntV;
        myJDegV = JDegV;
        if (iv > -1)
        {
            NbPntV = myNbVRoot - 2;
        }

        // myJMaxV
        aSize = JDegV - 2 * iv - 1;
        NCollection_HArray1<double> JMaxV = new(1, aSize);
        AdvApp2Var_ApproxF2var.mma2jmx_(JDegV, anOrderV, JMaxV.AsSpan());
        myJMaxV = JMaxV;

        // myURoots, myVRoots
        NCollection_HArray1<double> URoots = new(1, myNbURoot);
        NCollection_HArray1<double> VRoots = new(1, myNbVRoot);
        AdvApp2Var_ApproxF2var.mma2roo_(NbPntU, NbPntV, URoots.AsSpan(), VRoots.AsSpan());
        myURoots = URoots;
        myVRoots = VRoots;

        // myUGauss
        aSize = (NbPntU / 2 + 1) * (myJDegU - 2 * iu - 1);
        NCollection_HArray1<double> UGauss = new(1, aSize);
        AdvApp2Var_ApproxF2var.mmapptt_(JDegU, NbPntU, anOrderU, UGauss.AsSpan(), ref anErrorCode);
        if (anErrorCode != 0)
        {
            throw new ConstructionErrorException("AdvApp2Var_Context : Error in FORTRAN");
        }
        myUGauss = UGauss;

        // myVGauss
        aSize = (NbPntV / 2 + 1) * (myJDegV - 2 * iv - 1);
        NCollection_HArray1<double> VGauss = new(1, aSize);
        AdvApp2Var_ApproxF2var.mmapptt_(JDegV, NbPntV, anOrderV, VGauss.AsSpan(), ref anErrorCode);
        if (anErrorCode != 0)
        {
            throw new ConstructionErrorException("AdvApp2Var_Context : Error in FORTRAN");
        }
        myVGauss = VGauss;

        // myInternalTol, myFrontierTol, myCuttingTol
        int aNbSSP = nb1Dss + nb2Dss + nb3Dss;
        NCollection_HArray1<double> ITol = new(1, aNbSSP);
        appendInternalTolerance(tol1D, nb1Dss, 0, ITol);
        appendInternalTolerance(tol2D, nb2Dss, nb1Dss, ITol);
        appendInternalTolerance(tol3D, nb3Dss, nb1Dss + nb2Dss, ITol);
        if (iu > -1 || iv > -1)
        {
            for (int anSSPIndex = 1; anSSPIndex <= aNbSSP; ++anSSPIndex)
            {
                ITol.SetValue(anSSPIndex, ITol.Value(anSSPIndex) / 2);
            }
        }
        NCollection_HArray2<double> FTol = new(1, aNbSSP, 1, 4);
        NCollection_HArray2<double> CTol = new(1, aNbSSP, 1, 4);
        appendFrontierTolerance(tof1D, nb1Dss, 0, FTol, CTol);
        appendFrontierTolerance(tof2D, nb2Dss, nb1Dss, FTol, CTol);
        appendFrontierTolerance(tof3D, nb3Dss, nb1Dss + nb2Dss, FTol, CTol);
        if (iu > -1 || iv > -1)
        {
            double aTolMin;
            double aHMaxU = hMaxFactor(iu);
            double aHMaxV = hMaxFactor(iv);
            double aWeight = aHMaxU * aHMaxV + aHMaxU + aHMaxV;
            for (int anSSPIndex = 1; anSSPIndex <= aNbSSP; ++anSSPIndex)
            {
                for (int aTolIndex = 1; aTolIndex <= 4; ++aTolIndex)
                {
                    aTolMin = ITol.Value(anSSPIndex) / aWeight;
                    if (aTolMin < FTol.Value(anSSPIndex, aTolIndex))
                    {
                        FTol.SetValue(anSSPIndex, aTolIndex, aTolMin);
                    }
                    CTol.SetValue(anSSPIndex, aTolIndex, aTolMin);
                }
            }
        }
        myInternalTol = ITol;
        myFrontierTol = FTol;
        myCuttingTol = CTol;
    }

    public int TotalDimension() => myNb1DSS + 2 * myNb2DSS + 3 * myNb3DSS;

    public int TotalNumberSSP() => myNb1DSS + myNb2DSS + myNb3DSS;

    public int FavorIso() => myFav;

    public int UOrder() => myOrdU;

    public int VOrder() => myOrdV;

    public int ULimit() => myLimU;

    public int VLimit() => myLimV;

    public int UJacDeg() => myJDegU;

    public int VJacDeg() => myJDegV;

    public NCollection_HArray1<double>? UJacMax() => myJMaxU;

    public NCollection_HArray1<double>? VJacMax() => myJMaxV;

    public NCollection_HArray1<double>? URoots() => myURoots;

    public NCollection_HArray1<double>? VRoots() => myVRoots;

    public NCollection_HArray1<double>? UGauss() => myUGauss;

    public NCollection_HArray1<double>? VGauss() => myVGauss;

    public NCollection_HArray1<double>? IToler() => myInternalTol;

    public NCollection_HArray2<double>? FToler() => myFrontierTol;

    public NCollection_HArray2<double>? CToler() => myCuttingTol;
}
