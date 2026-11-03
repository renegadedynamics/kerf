// Port of: src/ModelingData/TKG3d/Geom/Geom_OsculatingSurface.pxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OsculatingSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;

namespace Occt;

/// <summary>
/// Internal helper of Geom_OffsetSurface: detects whether a B-spline or Bezier surface has a punctual U or V
/// iso-curve on its bounds (relative to the tolerance) and builds the corresponding osculating surfaces, from
/// which the offset normal is computed at those singular points.
/// </summary>
/// <remarks>
/// A private implementation class in OCCT (a .pxx), internal here. It is immutable once initialized: the
/// osculating surfaces it builds are frozen, so copies of an offset surface share them as OCCT's copy
/// constructor shares the handles.
/// </remarks>
internal sealed class Geom_OsculatingSurface
{
    private Geom_Surface? myBasisSurf;
    private double myTol;
    private readonly NCollection_Sequence<Geom_BSplineSurface> myOsculSurf1 = new();
    private readonly NCollection_Sequence<Geom_BSplineSurface> myOsculSurf2 = new();
    private readonly NCollection_Sequence<int> myKdeg = new();
    private readonly bool[] myAlong = new bool[4];

    /// <summary>An empty object (no osculating surface).</summary>
    public Geom_OsculatingSurface()
    {
        myTol = 0.0;
    }

    /// <summary>Detects the punctual iso-curves of theBS and builds the osculating surfaces.</summary>
    public Geom_OsculatingSurface(Geom_Surface theBS, double theTol)
    {
        Init(theBS, theTol);
    }

    /// <summary>A copy sharing the (frozen) basis and osculating surfaces, as OCCT's copy constructor.</summary>
    public Geom_OsculatingSurface(Geom_OsculatingSurface theOther)
    {
        myBasisSurf = theOther.myBasisSurf;
        myTol = theOther.myTol;
        myOsculSurf1.Assign(theOther.myOsculSurf1);
        myOsculSurf2.Assign(theOther.myOsculSurf2);
        myKdeg.Assign(theOther.myKdeg);
        Array.Copy(theOther.myAlong, myAlong, 4);
    }

    /// <summary>Initializes with a surface and tolerance.</summary>
    public void Init(Geom_Surface theBS, double theTol)
    {
        clearOsculFlags();
        myTol = theTol;
        double TolMin = 0.0; // consider all singularities below Tol, not just above 1.e-12
        bool OsculSurf = true;
        myBasisSurf = (Geom_Surface)theBS.Copy();
        myOsculSurf1.Clear();
        myOsculSurf2.Clear();
        myKdeg.Clear();

        if (theBS is Geom_BSplineSurface || theBS is Geom_BezierSurface)
        {
            double U1, U2, V1, V2;

            int i;
            theBS.Bounds(out U1, out U2, out V1, out V2);
            myAlong[0] = isQPunctual(theBS, V1, IsoV, TolMin, theTol);
            myAlong[1] = isQPunctual(theBS, V2, IsoV, TolMin, theTol);
            myAlong[2] = isQPunctual(theBS, U1, IsoU, TolMin, theTol);
            myAlong[3] = isQPunctual(theBS, U2, IsoU, TolMin, theTol);
            if (myAlong[0] || myAlong[1] || myAlong[2] || myAlong[3])
            {
                Geom_BSplineSurface InitSurf, S;
                Geom_BSplineSurface? L = null;
                if (theBS is Geom_BezierSurface BzS)
                {
                    NCollection_Array1<double> UKnots = new(1, 2);
                    NCollection_Array1<double> VKnots = new(1, 2);
                    NCollection_Array1<int> UMults = new(1, 2);
                    NCollection_Array1<int> VMults = new(1, 2);
                    for (i = 1; i <= 2; i++)
                    {
                        UKnots.SetValue(i, (i - 1));
                        VKnots.SetValue(i, (i - 1));
                        UMults.SetValue(i, BzS.UDegree() + 1);
                        VMults.SetValue(i, BzS.VDegree() + 1);
                    }
                    InitSurf = new Geom_BSplineSurface(BzS.InternalPoles(),
                                                       UKnots,
                                                       VKnots,
                                                       UMults,
                                                       VMults,
                                                       BzS.UDegree(),
                                                       BzS.VDegree(),
                                                       BzS.IsUPeriodic(),
                                                       BzS.IsVPeriodic());
                }
                else
                {
                    InitSurf = (Geom_BSplineSurface)myBasisSurf;
                }

                if (IsAlongU() && IsAlongV())
                {
                    clearOsculFlags();
                }

                if ((IsAlongU() && InitSurf.VDegree() > 1) || (IsAlongV() && InitSurf.UDegree() > 1))
                {
                    int k;
                    bool IsQPunc;
                    int UKnot, VKnot;
                    if (myAlong[0] || myAlong[1])
                    {
                        for (i = 1; i < InitSurf.NbUKnots(); i++)
                        {
                            if (myAlong[0])
                            {
                                S = InitSurf;
                                k = 0;
                                IsQPunc = true;
                                UKnot = i;
                                VKnot = 1;
                                while (IsQPunc)
                                {
                                    OsculSurf = buildOsculatingSurface(V1, UKnot, VKnot, S, ref L);
                                    if (!OsculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    IsQPunc = isQPunctual(L!, V1, IsoV, 0.0, theTol);
                                    UKnot = 1;
                                    VKnot = 1;
                                    S = L!;
                                }
                                if (OsculSurf)
                                {
                                    myOsculSurf1.Append(frozen(L!));
                                }
                                else
                                {
                                    clearOsculFlags();
                                }
                                if (myAlong[1] && OsculSurf)
                                {
                                    S = InitSurf;
                                    k = 0;
                                    IsQPunc = true;
                                    UKnot = i;
                                    VKnot = InitSurf.NbVKnots() - 1;

                                    while (IsQPunc)
                                    {
                                        OsculSurf = buildOsculatingSurface(V2, UKnot, VKnot, S, ref L);
                                        if (!OsculSurf)
                                        {
                                            break;
                                        }
                                        k++;
                                        IsQPunc = isQPunctual(L!, V2, IsoV, 0.0, theTol);
                                        UKnot = 1;
                                        VKnot = 1;
                                        S = L!;
                                    }
                                    if (OsculSurf)
                                    {
                                        myOsculSurf2.Append(frozen(L!));
                                        myKdeg.Append(k);
                                    }
                                }
                            }
                            else
                            {
                                S = InitSurf;
                                k = 0;
                                IsQPunc = true;
                                UKnot = i;
                                VKnot = InitSurf.NbVKnots() - 1;
                                while (IsQPunc)
                                {
                                    OsculSurf = buildOsculatingSurface(V2, UKnot, VKnot, S, ref L);
                                    if (!OsculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    IsQPunc = isQPunctual(L!, V2, IsoV, 0.0, theTol);
                                    UKnot = 1;
                                    VKnot = 1;
                                    S = L!;
                                }
                                if (OsculSurf)
                                {
                                    myOsculSurf2.Append(frozen(L!));
                                    myKdeg.Append(k);
                                }
                                else
                                {
                                    clearOsculFlags();
                                }
                            }
                        }
                    }
                    if (myAlong[2] || myAlong[3])
                    {
                        for (i = 1; i < InitSurf.NbVKnots(); i++)
                        {
                            if (myAlong[2])
                            {
                                S = InitSurf;
                                k = 0;
                                IsQPunc = true;
                                UKnot = 1;
                                VKnot = i;
                                while (IsQPunc)
                                {
                                    OsculSurf = buildOsculatingSurface(U1, UKnot, VKnot, S, ref L);
                                    if (!OsculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    IsQPunc = isQPunctual(L!, U1, IsoU, 0.0, theTol);
                                    UKnot = 1;
                                    VKnot = 1;
                                    S = L!;
                                }
                                if (OsculSurf)
                                {
                                    myOsculSurf1.Append(frozen(L!));
                                }
                                else
                                {
                                    clearOsculFlags();
                                }
                                if (myAlong[3] && OsculSurf)
                                {
                                    S = InitSurf;
                                    k = 0;
                                    IsQPunc = true;
                                    UKnot = InitSurf.NbUKnots() - 1;
                                    VKnot = i;
                                    while (IsQPunc)
                                    {
                                        OsculSurf = buildOsculatingSurface(U2, UKnot, VKnot, S, ref L);
                                        if (!OsculSurf)
                                        {
                                            break;
                                        }
                                        k++;
                                        IsQPunc = isQPunctual(L!, U2, IsoU, 0.0, theTol);
                                        UKnot = 1;
                                        VKnot = 1;
                                        S = L!;
                                    }
                                    if (OsculSurf)
                                    {
                                        myOsculSurf2.Append(frozen(L!));
                                        myKdeg.Append(k);
                                    }
                                }
                            }
                            else
                            {
                                S = InitSurf;
                                k = 0;
                                IsQPunc = true;
                                UKnot = InitSurf.NbUKnots() - 1;
                                VKnot = i;
                                while (IsQPunc)
                                {
                                    OsculSurf = buildOsculatingSurface(U2, UKnot, VKnot, S, ref L);
                                    if (!OsculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    IsQPunc = isQPunctual(L!, U2, IsoU, 0.0, theTol);
                                    UKnot = 1;
                                    VKnot = 1;
                                    S = L!;
                                }
                                if (OsculSurf)
                                {
                                    myOsculSurf2.Append(frozen(L!));
                                    myKdeg.Append(k);
                                }
                                else
                                {
                                    clearOsculFlags();
                                }
                            }
                        }
                    }
                }
                else
                {
                    clearOsculFlags();
                }
            }
        }
        else
        {
            clearOsculFlags();
        }
    }

    // OCCT.NET: the osculating surfaces are shared by copies and handed out by Geom_OffsetSurface's
    // U/VOsculatingSurface, so they are frozen (rules, section 8).
    private static Geom_BSplineSurface frozen(Geom_BSplineSurface theSurf)
    {
        theSurf.Freeze();
        return theSurf;
    }

    /// <summary>The basis surface (a copy of the surface given to Init, shared: don't modify).</summary>
    public Geom_Surface? BasisSurface() => myBasisSurf;

    /// <summary>The tolerance.</summary>
    public double Tolerance() => myTol;

    /// <summary>True if any osculating surface exists.</summary>
    public bool HasOscSurf() => myAlong[0] || myAlong[1] || myAlong[2] || myAlong[3];

    /// <summary>True if osculating surface exists along U direction.</summary>
    public bool IsAlongU() => myAlong[0] || myAlong[1];

    /// <summary>True if osculating surface exists along V direction.</summary>
    public bool IsAlongV() => myAlong[2] || myAlong[3];

    /// <summary>
    /// If true, theL is the local osculating surface along U at (theU, theV), and theT tells whether its derivative
    /// is opposite to the basis surface's. theT and theL are left unchanged when false.
    /// </summary>
    public bool UOsculatingSurface(double theU, double theV, ref bool theT, ref Geom_BSplineSurface? theL)
    {
        bool along = false;
        if (myAlong[0] || myAlong[1])
        {
            int NU = 1, NV = 1;
            theT = false;
            myBasisSurf!.Bounds(out _, out _, out _, out _);
            int NbUK, NbVK;
            bool isToSkipSecond = false;
            if (myBasisSurf is Geom_BSplineSurface BSur)
            {
                NbUK = BSur.NbUKnots();
                NbVK = BSur.NbVKnots();
                NCollection_Array1<double> UKnots = BSur.InternalUKnots();
                NCollection_Array1<double> VKnots = BSur.InternalVKnots();
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(UKnots, theU, out NU);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(VKnots, theV, out NV);
                if (NU < 1)
                {
                    NU = 1;
                }
                if (NU >= NbUK)
                {
                    NU = NbUK - 1;
                }
                if (NbVK == 2 && NV == 1)
                {
                    // Need to find the closest end
                    if (VKnots[NbVK] - theV > theV - VKnots[1])
                    {
                        isToSkipSecond = true;
                    }
                }
            }
            else
            {
                NU = 1;
                NV = 1;
                NbVK = 2;
            }

            if (myAlong[0] && NV == 1)
            {
                theL = myOsculSurf1.Value(NU);
                along = true;
            }
            if (myAlong[1] && (NV == NbVK - 1) && !isToSkipSecond)
            {
                // theT means that derivative vector of osculating surface is opposite
                // to the original. This happens when (v-t)^k is negative, i.e.
                // difference between degrees (k) is odd and t is the last parameter
                if (myKdeg.Value(NU) % 2 != 0)
                {
                    theT = true;
                }
                theL = myOsculSurf2.Value(NU);
                along = true;
            }
        }
        return along;
    }

    /// <summary>
    /// If true, theL is the local osculating surface along V at (theU, theV), and theT tells whether its derivative
    /// is opposite to the basis surface's. theT and theL are left unchanged when false.
    /// </summary>
    public bool VOsculatingSurface(double theU, double theV, ref bool theT, ref Geom_BSplineSurface? theL)
    {
        bool along = false;
        if (myAlong[2] || myAlong[3])
        {
            int NU = 1, NV = 1;
            theT = false;
            myBasisSurf!.Bounds(out _, out _, out _, out _);
            int NbUK, NbVK;
            bool isToSkipSecond = false;
            if (myBasisSurf is Geom_BSplineSurface BSur)
            {
                NbUK = BSur.NbUKnots();
                NbVK = BSur.NbVKnots();
                NCollection_Array1<double> UKnots = BSur.InternalUKnots();
                NCollection_Array1<double> VKnots = BSur.InternalVKnots();
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(UKnots, theU, out NU);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(VKnots, theV, out NV);
                if (NV < 1)
                {
                    NV = 1;
                }
                if (NV >= NbVK)
                {
                    NV = NbVK - 1;
                }
                if (NbUK == 2 && NU == 1)
                {
                    // Need to find the closest end
                    if (UKnots[NbUK] - theU > theU - UKnots[1])
                    {
                        isToSkipSecond = true;
                    }
                }
            }
            else
            {
                NU = 1;
                NV = 1;
                NbUK = 2;
            }

            if (myAlong[2] && NU == 1)
            {
                theL = myOsculSurf1.Value(NV);
                along = true;
            }
            if (myAlong[3] && (NU == NbUK - 1) && !isToSkipSecond)
            {
                if (myKdeg.Value(NV) % 2 != 0)
                {
                    theT = true;
                }
                theL = myOsculSurf2.Value(NV);
                along = true;
            }
        }
        return along;
    }

    // Returns false if the osculating surface can't be built.
    private bool buildOsculatingSurface(double theParam,
                                        int theSUKnot,
                                        int theSVKnot,
                                        Geom_BSplineSurface theBS,
                                        ref Geom_BSplineSurface? theBSpl)
    {
        bool OsculSurf = true;

        // for cache
        int MinDegree, MaxDegree;
        double udeg, vdeg;
        udeg = theBS.UDegree();
        vdeg = theBS.VDegree();
        if ((IsAlongU() && vdeg <= 1) || (IsAlongV() && udeg <= 1))
        {
            OsculSurf = false;
        }
        else
        {
            MinDegree = (int)Math.Min(udeg, vdeg);
            MaxDegree = (int)Math.Max(udeg, vdeg);

            NCollection_Array2<Point3D> cachepoles = new(1, MaxDegree + 1, 1, MinDegree + 1);
            // end for cache

            // for polynomial grid
            int MaxUDegree, MaxVDegree;
            int UContinuity, VContinuity;

            NCollection_HArray2<int> NumCoeffPerSurface = new(1, 1, 1, 2);
            NCollection_HArray1<double> PolynomialUIntervals = new(1, 2);
            NCollection_HArray1<double> PolynomialVIntervals = new(1, 2);
            NCollection_HArray1<double> TrueUIntervals = new(1, 2);
            NCollection_HArray1<double> TrueVIntervals = new(1, 2);
            MaxUDegree = (int)udeg;
            MaxVDegree = (int)vdeg;

            for (int i = 1; i <= 2; i++)
            {
                PolynomialUIntervals[i] = i - 1;
                PolynomialVIntervals[i] = i - 1;
                TrueUIntervals[i] = theBS.UKnot(theSUKnot + i - 1);
                TrueVIntervals[i] = theBS.VKnot(theSVKnot + i - 1);
            }

            int OscUNumCoeff = 0, OscVNumCoeff = 0;
            if (IsAlongU())
            {
                OscUNumCoeff = (int)udeg + 1;
                OscVNumCoeff = (int)vdeg;
            }
            if (IsAlongV())
            {
                OscUNumCoeff = (int)udeg;
                OscVNumCoeff = (int)vdeg + 1;
            }
            NumCoeffPerSurface[1, 1] = OscUNumCoeff;
            NumCoeffPerSurface[1, 2] = OscVNumCoeff;
            int nbc = NumCoeffPerSurface.Value(1, 1) * NumCoeffPerSurface.Value(1, 2) * 3;
            //
            if (nbc == 0)
            {
                return false;
            }
            //
            NCollection_HArray1<double> Coefficients = new(1, nbc);
            //    end for polynomial grid

            //    building the cache
            int ULocalIndex, VLocalIndex;
            double ucacheparameter, vcacheparameter, uspanlength, vspanlength;

            NCollection_Array2<Point3D> aPoles = theBS.InternalPoles();
            NCollection_Array1<double> aUFlatKnts = theBS.InternalUKnotSequence();
            NCollection_Array1<double> aVFlatKnts = theBS.InternalVKnotSequence();

            VLocalIndex = 0;
            ULocalIndex = 0;
            ucacheparameter = theBS.UKnot(theSUKnot);
            vcacheparameter = theBS.VKnot(theSVKnot);
            vspanlength = theBS.VKnot(theSVKnot + 1) - theBS.VKnot(theSVKnot);
            uspanlength = theBS.UKnot(theSUKnot + 1) - theBS.UKnot(theSUKnot);

            // Always reduce to a parametrization such that locally it is the iso
            // u=0 or v=0 that is degenerate

            bool IsVNegative = theParam > vcacheparameter + vspanlength / 2;
            bool IsUNegative = theParam > ucacheparameter + uspanlength / 2;

            if (IsAlongU() && (theParam > vcacheparameter + vspanlength / 2))
            {
                vcacheparameter = vcacheparameter + vspanlength;
            }
            if (IsAlongV() && (theParam > ucacheparameter + uspanlength / 2))
            {
                ucacheparameter = ucacheparameter + uspanlength;
            }

            SurfaceBSplSLib.BuildCache(ucacheparameter,
                                vcacheparameter,
                                uspanlength,
                                vspanlength,
                                theBS.IsUPeriodic(),
                                theBS.IsVPeriodic(),
                                theBS.UDegree(),
                                theBS.VDegree(),
                                ULocalIndex,
                                VLocalIndex,
                                aUFlatKnts,
                                aVFlatKnts,
                                aPoles,
                                SurfaceBSplSLib.NoWeights(),
                                cachepoles,
                                SurfaceBSplSLib.NoWeights());
            int m, n, index;
            NCollection_Array2<Point3D> OscCoeff = new(1, OscUNumCoeff, 1, OscVNumCoeff);

            if (IsAlongU())
            {
                if (udeg > vdeg)
                {
                    for (n = 1; n <= (int)udeg + 1; n++)
                    {
                        for (m = 1; m <= (int)vdeg; m++)
                        {
                            OscCoeff[n, m] = cachepoles[n, m + 1];
                        }
                    }
                }
                else
                {
                    for (n = 1; n <= (int)udeg + 1; n++)
                    {
                        for (m = 1; m <= (int)vdeg; m++)
                        {
                            OscCoeff[n, m] = cachepoles[m + 1, n];
                        }
                    }
                }
                if (IsVNegative)
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.VTrimming(-1, 0, OscCoeff, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
                }

                index = 1;
                for (n = 1; n <= (int)udeg + 1; n++)
                {
                    for (m = 1; m <= (int)vdeg; m++)
                    {
                        Coefficients[index++] = OscCoeff[n, m].X();
                        Coefficients[index++] = OscCoeff[n, m].Y();
                        Coefficients[index++] = OscCoeff[n, m].Z();
                    }
                }
            }

            if (IsAlongV())
            {
                if (udeg > vdeg)
                {
                    for (n = 1; n <= (int)udeg; n++)
                    {
                        for (m = 1; m <= (int)vdeg + 1; m++)
                        {
                            OscCoeff[n, m] = cachepoles[n + 1, m];
                        }
                    }
                }
                else
                {
                    for (n = 1; n <= (int)udeg; n++)
                    {
                        for (m = 1; m <= (int)vdeg + 1; m++)
                        {
                            OscCoeff[n, m] = cachepoles[m, n + 1];
                        }
                    }
                }
                if (IsUNegative)
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.UTrimming(-1, 0, OscCoeff, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
                }
                index = 1;
                for (n = 1; n <= (int)udeg; n++)
                {
                    for (m = 1; m <= (int)vdeg + 1; m++)
                    {
                        Coefficients[index++] = OscCoeff[n, m].X();
                        Coefficients[index++] = OscCoeff[n, m].Y();
                        Coefficients[index++] = OscCoeff[n, m].Z();
                    }
                }
            }

            if (IsAlongU())
            {
                MaxVDegree--;
            }
            if (IsAlongV())
            {
                MaxUDegree--;
            }
            UContinuity = -1;
            VContinuity = -1;

            GridPolynomialToPoles Data = new(1,
                                                     1,
                                                     UContinuity,
                                                     VContinuity,
                                                     MaxUDegree,
                                                     MaxVDegree,
                                                     NumCoeffPerSurface,
                                                     Coefficients,
                                                     PolynomialUIntervals,
                                                     PolynomialVIntervals,
                                                     TrueUIntervals,
                                                     TrueVIntervals);

            theBSpl = new Geom_BSplineSurface(Data.Poles(),
                                              Data.UKnots(),
                                              Data.VKnots(),
                                              Data.UMultiplicities(),
                                              Data.VMultiplicities(),
                                              Data.UDegree(),
                                              Data.VDegree(),
                                              false,
                                              false);
        }
        return OsculSurf;
    }

    // Returns true if the isoparametric is quasi-punctual.
    private static bool isQPunctual(Geom_Surface theS, double theParam, IsoType theIT, double theTolMin,
                                    double theTolMax)
    {
        double U1, U2, V1, V2, T;
        bool Along = true;
        theS.Bounds(out U1, out U2, out V1, out V2);
        Vector3D D1U, D1V;
        Point3D P;
        double Step, D1NormMax;
        if (theIT == IsoV)
        {
            Step = (U2 - U1) / 10;
            D1NormMax = 0.0;
            for (T = U1; T <= U2; T = T + Step)
            {
                theS.D1(T, theParam, out P, out D1U, out D1V);
                D1NormMax = Math.Max(D1NormMax, D1U.Magnitude());
            }

            if (D1NormMax > theTolMax || D1NormMax < theTolMin)
            {
                Along = false;
            }
        }
        else
        {
            Step = (V2 - V1) / 10;
            D1NormMax = 0.0;
            for (T = V1; T <= V2; T = T + Step)
            {
                theS.D1(theParam, T, out P, out D1U, out D1V);
                D1NormMax = Math.Max(D1NormMax, D1V.Magnitude());
            }
            if (D1NormMax > theTolMax || D1NormMax < theTolMin)
            {
                Along = false;
            }
        }
        return Along;
    }

    // Clear osculating flags.
    private void clearOsculFlags()
    {
        Array.Fill(myAlong, false);
    }
}
