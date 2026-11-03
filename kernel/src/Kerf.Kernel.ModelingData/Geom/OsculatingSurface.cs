// Port of: src/ModelingData/TKG3d/Geom/Geom_OsculatingSurface.pxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OsculatingSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.IsoType;

namespace Kerf.Geometry;

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
internal sealed class OsculatingSurface
{
    private Surface? _basisSurf;
    private double _tol;
    private readonly NCollection_Sequence<BSplineSurface> _osculSurf1 = new();
    private readonly NCollection_Sequence<BSplineSurface> _osculSurf2 = new();
    private readonly NCollection_Sequence<int> _kdeg = new();
    private readonly bool[] _along = new bool[4];

    /// <summary>An empty object (no osculating surface).</summary>
    public OsculatingSurface()
    {
        _tol = 0.0;
    }

    /// <summary>Detects the punctual iso-curves of theBS and builds the osculating surfaces.</summary>
    public OsculatingSurface(Surface bs, double tol)
    {
        Init(bs, tol);
    }

    /// <summary>A copy sharing the (frozen) basis and osculating surfaces, as OCCT's copy constructor.</summary>
    public OsculatingSurface(OsculatingSurface other)
    {
        _basisSurf = other._basisSurf;
        _tol = other._tol;
        _osculSurf1.Assign(other._osculSurf1);
        _osculSurf2.Assign(other._osculSurf2);
        _kdeg.Assign(other._kdeg);
        Array.Copy(other._along, _along, 4);
    }

    /// <summary>Initializes with a surface and tolerance.</summary>
    public void Init(Surface bs, double tol)
    {
        ClearOsculFlags();
        _tol = tol;
        double tolMin = 0.0; // consider all singularities below Tol, not just above 1.e-12
        bool osculSurf = true;
        _basisSurf = (Surface)bs.Copy();
        _osculSurf1.Clear();
        _osculSurf2.Clear();
        _kdeg.Clear();

        if (bs is BSplineSurface || bs is BezierSurface)
        {
            double u1, u2, v1, v2;

            int i;
            bs.Bounds(out u1, out u2, out v1, out v2);
            _along[0] = IsQPunctual(bs, v1, IsoV, tolMin, tol);
            _along[1] = IsQPunctual(bs, v2, IsoV, tolMin, tol);
            _along[2] = IsQPunctual(bs, u1, IsoU, tolMin, tol);
            _along[3] = IsQPunctual(bs, u2, IsoU, tolMin, tol);
            if (_along[0] || _along[1] || _along[2] || _along[3])
            {
                BSplineSurface initSurf, s;
                BSplineSurface? l = null;
                if (bs is BezierSurface bzS)
                {
                    NCollection_Array1<double> uKnots = new(1, 2);
                    NCollection_Array1<double> vKnots = new(1, 2);
                    NCollection_Array1<int> uMults = new(1, 2);
                    NCollection_Array1<int> vMults = new(1, 2);
                    for (i = 1; i <= 2; i++)
                    {
                        uKnots.SetValue(i, (i - 1));
                        vKnots.SetValue(i, (i - 1));
                        uMults.SetValue(i, bzS.UDegree() + 1);
                        vMults.SetValue(i, bzS.VDegree() + 1);
                    }
                    initSurf = new BSplineSurface(bzS.InternalPoles(),
                                                       uKnots,
                                                       vKnots,
                                                       uMults,
                                                       vMults,
                                                       bzS.UDegree(),
                                                       bzS.VDegree(),
                                                       bzS.IsUPeriodic,
                                                       bzS.IsVPeriodic);
                }
                else
                {
                    initSurf = (BSplineSurface)_basisSurf;
                }

                if (IsAlongU && IsAlongV)
                {
                    ClearOsculFlags();
                }

                if ((IsAlongU && initSurf.VDegree() > 1) || (IsAlongV && initSurf.UDegree() > 1))
                {
                    int k;
                    bool isQPunc;
                    int uKnot, vKnot;
                    if (_along[0] || _along[1])
                    {
                        for (i = 1; i < initSurf.UKnotCount; i++)
                        {
                            if (_along[0])
                            {
                                s = initSurf;
                                k = 0;
                                isQPunc = true;
                                uKnot = i;
                                vKnot = 1;
                                while (isQPunc)
                                {
                                    osculSurf = BuildOsculatingSurface(v1, uKnot, vKnot, s, ref l);
                                    if (!osculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    isQPunc = IsQPunctual(l!, v1, IsoV, 0.0, tol);
                                    uKnot = 1;
                                    vKnot = 1;
                                    s = l!;
                                }
                                if (osculSurf)
                                {
                                    _osculSurf1.Append(Frozen(l!));
                                }
                                else
                                {
                                    ClearOsculFlags();
                                }
                                if (_along[1] && osculSurf)
                                {
                                    s = initSurf;
                                    k = 0;
                                    isQPunc = true;
                                    uKnot = i;
                                    vKnot = initSurf.VKnotCount - 1;

                                    while (isQPunc)
                                    {
                                        osculSurf = BuildOsculatingSurface(v2, uKnot, vKnot, s, ref l);
                                        if (!osculSurf)
                                        {
                                            break;
                                        }
                                        k++;
                                        isQPunc = IsQPunctual(l!, v2, IsoV, 0.0, tol);
                                        uKnot = 1;
                                        vKnot = 1;
                                        s = l!;
                                    }
                                    if (osculSurf)
                                    {
                                        _osculSurf2.Append(Frozen(l!));
                                        _kdeg.Append(k);
                                    }
                                }
                            }
                            else
                            {
                                s = initSurf;
                                k = 0;
                                isQPunc = true;
                                uKnot = i;
                                vKnot = initSurf.VKnotCount - 1;
                                while (isQPunc)
                                {
                                    osculSurf = BuildOsculatingSurface(v2, uKnot, vKnot, s, ref l);
                                    if (!osculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    isQPunc = IsQPunctual(l!, v2, IsoV, 0.0, tol);
                                    uKnot = 1;
                                    vKnot = 1;
                                    s = l!;
                                }
                                if (osculSurf)
                                {
                                    _osculSurf2.Append(Frozen(l!));
                                    _kdeg.Append(k);
                                }
                                else
                                {
                                    ClearOsculFlags();
                                }
                            }
                        }
                    }
                    if (_along[2] || _along[3])
                    {
                        for (i = 1; i < initSurf.VKnotCount; i++)
                        {
                            if (_along[2])
                            {
                                s = initSurf;
                                k = 0;
                                isQPunc = true;
                                uKnot = 1;
                                vKnot = i;
                                while (isQPunc)
                                {
                                    osculSurf = BuildOsculatingSurface(u1, uKnot, vKnot, s, ref l);
                                    if (!osculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    isQPunc = IsQPunctual(l!, u1, IsoU, 0.0, tol);
                                    uKnot = 1;
                                    vKnot = 1;
                                    s = l!;
                                }
                                if (osculSurf)
                                {
                                    _osculSurf1.Append(Frozen(l!));
                                }
                                else
                                {
                                    ClearOsculFlags();
                                }
                                if (_along[3] && osculSurf)
                                {
                                    s = initSurf;
                                    k = 0;
                                    isQPunc = true;
                                    uKnot = initSurf.UKnotCount - 1;
                                    vKnot = i;
                                    while (isQPunc)
                                    {
                                        osculSurf = BuildOsculatingSurface(u2, uKnot, vKnot, s, ref l);
                                        if (!osculSurf)
                                        {
                                            break;
                                        }
                                        k++;
                                        isQPunc = IsQPunctual(l!, u2, IsoU, 0.0, tol);
                                        uKnot = 1;
                                        vKnot = 1;
                                        s = l!;
                                    }
                                    if (osculSurf)
                                    {
                                        _osculSurf2.Append(Frozen(l!));
                                        _kdeg.Append(k);
                                    }
                                }
                            }
                            else
                            {
                                s = initSurf;
                                k = 0;
                                isQPunc = true;
                                uKnot = initSurf.UKnotCount - 1;
                                vKnot = i;
                                while (isQPunc)
                                {
                                    osculSurf = BuildOsculatingSurface(u2, uKnot, vKnot, s, ref l);
                                    if (!osculSurf)
                                    {
                                        break;
                                    }
                                    k++;
                                    isQPunc = IsQPunctual(l!, u2, IsoU, 0.0, tol);
                                    uKnot = 1;
                                    vKnot = 1;
                                    s = l!;
                                }
                                if (osculSurf)
                                {
                                    _osculSurf2.Append(Frozen(l!));
                                    _kdeg.Append(k);
                                }
                                else
                                {
                                    ClearOsculFlags();
                                }
                            }
                        }
                    }
                }
                else
                {
                    ClearOsculFlags();
                }
            }
        }
        else
        {
            ClearOsculFlags();
        }
    }

    // OCCT.NET: the osculating surfaces are shared by copies and handed out by Geom_OffsetSurface's
    // U/VOsculatingSurface, so they are frozen (rules, section 8).
    private static BSplineSurface Frozen(BSplineSurface surf)
    {
        surf.Freeze();
        return surf;
    }

    /// <summary>The basis surface (a copy of the surface given to Init, shared: don't modify).</summary>
    public Surface? BasisSurface() => _basisSurf;

    /// <summary>The tolerance.</summary>
    public double Tolerance() => _tol;

    /// <summary>True if any osculating surface exists.</summary>
    public bool HasOscSurf => _along[0] || _along[1] || _along[2] || _along[3];

    /// <summary>True if osculating surface exists along U direction.</summary>
    public bool IsAlongU => _along[0] || _along[1];

    /// <summary>True if osculating surface exists along V direction.</summary>
    public bool IsAlongV => _along[2] || _along[3];

    /// <summary>
    /// If true, theL is the local osculating surface along U at (theU, theV), and theT tells whether its derivative
    /// is opposite to the basis surface's. theT and theL are left unchanged when false.
    /// </summary>
    public bool UOsculatingSurface(double u, double v, ref bool t, ref BSplineSurface? l)
    {
        bool along = false;
        if (_along[0] || _along[1])
        {
            int nu = 1, nv = 1;
            t = false;
            _basisSurf!.Bounds(out _, out _, out _, out _);
            int nbUK, nbVK;
            bool isToSkipSecond = false;
            if (_basisSurf is BSplineSurface bSur)
            {
                nbUK = bSur.UKnotCount;
                nbVK = bSur.VKnotCount;
                NCollection_Array1<double> uKnots = bSur.InternalUKnots();
                NCollection_Array1<double> vKnots = bSur.InternalVKnots();
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(uKnots, u, out nu);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(vKnots, v, out nv);
                if (nu < 1)
                {
                    nu = 1;
                }
                if (nu >= nbUK)
                {
                    nu = nbUK - 1;
                }
                if (nbVK == 2 && nv == 1)
                {
                    // Need to find the closest end
                    if (vKnots[nbVK] - v > v - vKnots[1])
                    {
                        isToSkipSecond = true;
                    }
                }
            }
            else
            {
                nu = 1;
                nv = 1;
                nbVK = 2;
            }

            if (_along[0] && nv == 1)
            {
                l = _osculSurf1.Value(nu);
                along = true;
            }
            if (_along[1] && (nv == nbVK - 1) && !isToSkipSecond)
            {
                // theT means that derivative vector of osculating surface is opposite
                // to the original. This happens when (v-t)^k is negative, i.e.
                // difference between degrees (k) is odd and t is the last parameter
                if (_kdeg.Value(nu) % 2 != 0)
                {
                    t = true;
                }
                l = _osculSurf2.Value(nu);
                along = true;
            }
        }
        return along;
    }

    /// <summary>
    /// If true, theL is the local osculating surface along V at (theU, theV), and theT tells whether its derivative
    /// is opposite to the basis surface's. theT and theL are left unchanged when false.
    /// </summary>
    public bool VOsculatingSurface(double u, double v, ref bool t, ref BSplineSurface? l)
    {
        bool along = false;
        if (_along[2] || _along[3])
        {
            int nu = 1, nv = 1;
            t = false;
            _basisSurf!.Bounds(out _, out _, out _, out _);
            int nbUK, nbVK;
            bool isToSkipSecond = false;
            if (_basisSurf is BSplineSurface bSur)
            {
                nbUK = bSur.UKnotCount;
                nbVK = bSur.VKnotCount;
                NCollection_Array1<double> uKnots = bSur.InternalUKnots();
                NCollection_Array1<double> vKnots = bSur.InternalVKnots();
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(uKnots, u, out nu);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(vKnots, v, out nv);
                if (nv < 1)
                {
                    nv = 1;
                }
                if (nv >= nbVK)
                {
                    nv = nbVK - 1;
                }
                if (nbUK == 2 && nu == 1)
                {
                    // Need to find the closest end
                    if (uKnots[nbUK] - u > u - uKnots[1])
                    {
                        isToSkipSecond = true;
                    }
                }
            }
            else
            {
                nu = 1;
                nv = 1;
                nbUK = 2;
            }

            if (_along[2] && nu == 1)
            {
                l = _osculSurf1.Value(nv);
                along = true;
            }
            if (_along[3] && (nu == nbUK - 1) && !isToSkipSecond)
            {
                if (_kdeg.Value(nv) % 2 != 0)
                {
                    t = true;
                }
                l = _osculSurf2.Value(nv);
                along = true;
            }
        }
        return along;
    }

    // Returns false if the osculating surface can't be built.
    private bool BuildOsculatingSurface(double param,
                                        int suKnot,
                                        int svKnot,
                                        BSplineSurface bs,
                                        ref BSplineSurface? bSpl)
    {
        bool osculSurf = true;

        // for cache
        int minDegree, maxDegree;
        double udeg, vdeg;
        udeg = bs.UDegree();
        vdeg = bs.VDegree();
        if ((IsAlongU && vdeg <= 1) || (IsAlongV && udeg <= 1))
        {
            osculSurf = false;
        }
        else
        {
            minDegree = (int)Math.Min(udeg, vdeg);
            maxDegree = (int)Math.Max(udeg, vdeg);

            NCollection_Array2<Point3D> cachepoles = new(1, maxDegree + 1, 1, minDegree + 1);
            // end for cache

            // for polynomial grid
            int maxUDegree, maxVDegree;
            int uContinuity, vContinuity;

            NCollection_HArray2<int> numCoeffPerSurface = new(1, 1, 1, 2);
            NCollection_HArray1<double> polynomialUIntervals = new(1, 2);
            NCollection_HArray1<double> polynomialVIntervals = new(1, 2);
            NCollection_HArray1<double> trueUIntervals = new(1, 2);
            NCollection_HArray1<double> trueVIntervals = new(1, 2);
            maxUDegree = (int)udeg;
            maxVDegree = (int)vdeg;

            for (int i = 1; i <= 2; i++)
            {
                polynomialUIntervals[i] = i - 1;
                polynomialVIntervals[i] = i - 1;
                trueUIntervals[i] = bs.UKnot(suKnot + i - 1);
                trueVIntervals[i] = bs.VKnot(svKnot + i - 1);
            }

            int oscUNumCoeff = 0, oscVNumCoeff = 0;
            if (IsAlongU)
            {
                oscUNumCoeff = (int)udeg + 1;
                oscVNumCoeff = (int)vdeg;
            }
            if (IsAlongV)
            {
                oscUNumCoeff = (int)udeg;
                oscVNumCoeff = (int)vdeg + 1;
            }
            numCoeffPerSurface[1, 1] = oscUNumCoeff;
            numCoeffPerSurface[1, 2] = oscVNumCoeff;
            int nbc = numCoeffPerSurface.Value(1, 1) * numCoeffPerSurface.Value(1, 2) * 3;
            //
            if (nbc == 0)
            {
                return false;
            }
            //
            NCollection_HArray1<double> coefficients = new(1, nbc);
            //    end for polynomial grid

            //    building the cache
            int uLocalIndex, vLocalIndex;
            double ucacheparameter, vcacheparameter, uspanlength, vspanlength;

            NCollection_Array2<Point3D> poles = bs.InternalPoles();
            NCollection_Array1<double> uFlatKnts = bs.InternalUKnotSequence();
            NCollection_Array1<double> vFlatKnts = bs.InternalVKnotSequence();

            vLocalIndex = 0;
            uLocalIndex = 0;
            ucacheparameter = bs.UKnot(suKnot);
            vcacheparameter = bs.VKnot(svKnot);
            vspanlength = bs.VKnot(svKnot + 1) - bs.VKnot(svKnot);
            uspanlength = bs.UKnot(suKnot + 1) - bs.UKnot(suKnot);

            // Always reduce to a parametrization such that locally it is the iso
            // u=0 or v=0 that is degenerate

            bool isVNegative = param > vcacheparameter + vspanlength / 2;
            bool isUNegative = param > ucacheparameter + uspanlength / 2;

            if (IsAlongU && (param > vcacheparameter + vspanlength / 2))
            {
                vcacheparameter = vcacheparameter + vspanlength;
            }
            if (IsAlongV && (param > ucacheparameter + uspanlength / 2))
            {
                ucacheparameter = ucacheparameter + uspanlength;
            }

            SurfaceBSplSLib.BuildCache(ucacheparameter,
                                vcacheparameter,
                                uspanlength,
                                vspanlength,
                                bs.IsUPeriodic,
                                bs.IsVPeriodic,
                                bs.UDegree(),
                                bs.VDegree(),
                                uLocalIndex,
                                vLocalIndex,
                                uFlatKnts,
                                vFlatKnts,
                                poles,
                                SurfaceBSplSLib.NoWeights(),
                                cachepoles,
                                SurfaceBSplSLib.NoWeights());
            int m, n, index;
            NCollection_Array2<Point3D> oscCoeff = new(1, oscUNumCoeff, 1, oscVNumCoeff);

            if (IsAlongU)
            {
                if (udeg > vdeg)
                {
                    for (n = 1; n <= (int)udeg + 1; n++)
                    {
                        for (m = 1; m <= (int)vdeg; m++)
                        {
                            oscCoeff[n, m] = cachepoles[n, m + 1];
                        }
                    }
                }
                else
                {
                    for (n = 1; n <= (int)udeg + 1; n++)
                    {
                        for (m = 1; m <= (int)vdeg; m++)
                        {
                            oscCoeff[n, m] = cachepoles[m + 1, n];
                        }
                    }
                }
                if (isVNegative)
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.VTrimming(-1, 0, oscCoeff, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
                }

                index = 1;
                for (n = 1; n <= (int)udeg + 1; n++)
                {
                    for (m = 1; m <= (int)vdeg; m++)
                    {
                        coefficients[index++] = oscCoeff[n, m].X();
                        coefficients[index++] = oscCoeff[n, m].Y();
                        coefficients[index++] = oscCoeff[n, m].Z();
                    }
                }
            }

            if (IsAlongV)
            {
                if (udeg > vdeg)
                {
                    for (n = 1; n <= (int)udeg; n++)
                    {
                        for (m = 1; m <= (int)vdeg + 1; m++)
                        {
                            oscCoeff[n, m] = cachepoles[n + 1, m];
                        }
                    }
                }
                else
                {
                    for (n = 1; n <= (int)udeg; n++)
                    {
                        for (m = 1; m <= (int)vdeg + 1; m++)
                        {
                            oscCoeff[n, m] = cachepoles[m, n + 1];
                        }
                    }
                }
                if (isUNegative)
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.UTrimming(-1, 0, oscCoeff, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
                }
                index = 1;
                for (n = 1; n <= (int)udeg; n++)
                {
                    for (m = 1; m <= (int)vdeg + 1; m++)
                    {
                        coefficients[index++] = oscCoeff[n, m].X();
                        coefficients[index++] = oscCoeff[n, m].Y();
                        coefficients[index++] = oscCoeff[n, m].Z();
                    }
                }
            }

            if (IsAlongU)
            {
                maxVDegree--;
            }
            if (IsAlongV)
            {
                maxUDegree--;
            }
            uContinuity = -1;
            vContinuity = -1;

            GridPolynomialToPoles data = new(1,
                                                     1,
                                                     uContinuity,
                                                     vContinuity,
                                                     maxUDegree,
                                                     maxVDegree,
                                                     numCoeffPerSurface,
                                                     coefficients,
                                                     polynomialUIntervals,
                                                     polynomialVIntervals,
                                                     trueUIntervals,
                                                     trueVIntervals);

            bSpl = new BSplineSurface(data.Poles(),
                                              data.UKnots(),
                                              data.VKnots(),
                                              data.UMultiplicities(),
                                              data.VMultiplicities(),
                                              data.UDegree(),
                                              data.VDegree(),
                                              false,
                                              false);
        }
        return osculSurf;
    }

    // Returns true if the isoparametric is quasi-punctual.
    private static bool IsQPunctual(Surface s, double param, IsoType it, double tolMin,
                                    double tolMax)
    {
        double u1, u2, v1, v2, t;
        bool along = true;
        s.Bounds(out u1, out u2, out v1, out v2);
        Vector3D d1U, d1V;
        Point3D p;
        double step, d1NormMax;
        if (it == IsoV)
        {
            step = (u2 - u1) / 10;
            d1NormMax = 0.0;
            for (t = u1; t <= u2; t = t + step)
            {
                s.D1(t, param, out p, out d1U, out d1V);
                d1NormMax = Math.Max(d1NormMax, d1U.Magnitude());
            }

            if (d1NormMax > tolMax || d1NormMax < tolMin)
            {
                along = false;
            }
        }
        else
        {
            step = (v2 - v1) / 10;
            d1NormMax = 0.0;
            for (t = v1; t <= v2; t = t + step)
            {
                s.D1(param, t, out p, out d1U, out d1V);
                d1NormMax = Math.Max(d1NormMax, d1V.Magnitude());
            }
            if (d1NormMax > tolMax || d1NormMax < tolMin)
            {
                along = false;
            }
        }
        return along;
    }

    // Clear osculating flags.
    private void ClearOsculFlags()
    {
        Array.Fill(_along, false);
    }
}
