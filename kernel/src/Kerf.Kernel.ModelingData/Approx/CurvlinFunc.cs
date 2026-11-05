// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvlinFunc.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvlinFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// An abstract curve with curvilinear parametrization (S in [0, 1]): a 3D curve (case 1), a 2D curve on a surface
/// (case 2), or two 2D curves on two surfaces (case 3), whose evaluations Approx_CurvilinearParameter approximates.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Trim"/> and <see cref="SetTol"/> change it, and the evaluations
/// remember the last parameter found (OCCT casts const away to write it). The adaptors are kept as OCCT keeps the
/// handles; <see cref="Trim"/> replaces them by trimmed adaptors and never changes the caller's.
/// </remarks>
internal sealed class CurvlinFunc : Transient
{
    private CurveAdaptorBase? _c3D;
    private Curve2DAdaptorBase? _c2D1;
    private Curve2DAdaptorBase? _c2D2;
    private SurfaceAdaptorBase? _surf1;
    private SurfaceAdaptorBase? _surf2;
    private readonly int _case;
    private double _firstS;
    private double _lastS;
    private double _firstU1;
    private double _lastU1;
    private double _firstU2;
    private double _lastU2;
    private double _length;
    private double _length1;
    private double _length2;
    private double _tolLen;
    private double _prevS;
    private double _prevU;
    private NCollection_HArray1<double>? _ui1;
    private NCollection_HArray1<double>? _si1;
    private NCollection_HArray1<double>? _ui2;
    private NCollection_HArray1<double>? _si2;

    private static double Cubic(double x, ReadOnlySpan<double> xi, ReadOnlySpan<double> yi)
    {
        double i1, i2, i3, i21, i22, i31, result;

        i1 = (yi[0] - yi[1]) / (xi[0] - xi[1]);
        i2 = (yi[1] - yi[2]) / (xi[1] - xi[2]);
        i3 = (yi[2] - yi[3]) / (xi[2] - xi[3]);

        i21 = (i1 - i2) / (xi[0] - xi[2]);
        i22 = (i2 - i3) / (xi[1] - xi[3]);

        i31 = (i21 - i22) / (xi[0] - xi[3]);

        result = yi[0] + (x - xi[0]) * (i1 + (x - xi[1]) * (i21 + (x - xi[2]) * i31));

        return result;
    }

    private static void Findfourpoints(double /*S*/ _,
                                       int nInterval,
                                       NCollection_HArray1<double> si,
                                       NCollection_HArray1<double> ui,
                                       double prevS,
                                       double prevU,
                                       Span<double> xi,
                                       Span<double> yi)
    {
        int i, j;
        int nbInt = si.Length() - 1;
        if (nbInt < 3)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::GetUParameter");
        }

        if (nInterval < 1)
        {
            nInterval = 1;
        }
        else if (nInterval > nbInt - 2)
        {
            nInterval = nbInt - 2;
        }

        for (i = 0; i < 4; i++)
        {
            xi[i] = si.Value(nInterval - 1 + i);
            yi[i] = ui.Value(nInterval - 1 + i);
        }
        // try to insert (S, U)
        for (i = 0; i < 3; i++)
        {
            if (xi[i] < prevS && prevS < xi[i + 1])
            {
                for (j = 0; j < i; j++)
                {
                    xi[j] = xi[j + 1];
                    yi[j] = yi[j + 1];
                }
                xi[i] = prevS;
                yi[i] = prevU;
                break;
            }
        }
    }

    /// <summary>A 3D curve, with the tolerance of the length computations.</summary>
    public CurvlinFunc(CurveAdaptorBase c, double tol)
    {
        _c3D = c;
        _case = 1;
        _firstS = 0;
        _lastS = 1;
        _tolLen = tol;
        _prevS = 0.0;
        _prevU = 0.0;
        Init();
    }

    /// <summary>A 2D curve on a surface, with the tolerance of the length computations.</summary>
    public CurvlinFunc(Curve2DAdaptorBase c2D, SurfaceAdaptorBase s, double tol)
    {
        _c2D1 = c2D;
        _surf1 = s;
        _case = 2;
        _firstS = 0;
        _lastS = 1;
        _tolLen = tol;
        _prevS = 0.0;
        _prevU = 0.0;
        Init();
    }

    /// <summary>Two 2D curves on two surfaces, with the tolerance of the length computations.</summary>
    public CurvlinFunc(Curve2DAdaptorBase c2D1,
                              Curve2DAdaptorBase c2D2,
                              SurfaceAdaptorBase s1,
                              SurfaceAdaptorBase s2,
                              double tol)
    {
        _c2D1 = c2D1;
        _c2D2 = c2D2;
        _surf1 = s1;
        _surf2 = s2;
        _case = 3;
        _firstS = 0;
        _lastS = 1;
        _tolLen = tol;
        _prevS = 0.0;
        _prevU = 0.0;
        Init();
    }

    private void Init()
    {
        CurveOnSurface curOnSur = new();

        switch (_case)
        {
            case 1:
                Init(_c3D!, out _si1, out _ui1);
                _firstU1 = _c3D!.FirstParameter();
                _lastU1 = _c3D.LastParameter();
                _firstU2 = _lastU2 = 0;
                break;
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                Init(curOnSur, out _si1, out _ui1);
                _firstU1 = curOnSur.FirstParameter();
                _lastU1 = curOnSur.LastParameter();
                _firstU2 = _lastU2 = 0;
                break;
            case 3:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                Init(curOnSur, out _si1, out _ui1);
                _firstU1 = curOnSur.FirstParameter();
                _lastU1 = curOnSur.LastParameter();
                // OCCT: the reloaded adaptor keeps the first curve's C3 intervals (see NbIntervals).
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                Init(curOnSur, out _si2, out _ui2);
                _firstU2 = curOnSur.FirstParameter();
                _lastU2 = curOnSur.LastParameter();
                break;
        }

        Length();
    }

    // Init the values. 23/10/1998 PMN: cut at the curve's discontinuities.
    private void Init(CurveAdaptorBase c, out NCollection_HArray1<double> si, out NCollection_HArray1<double> ui)
    {
        double step, firstU, lastU;
        int i, j, k, nbInt, nbIntC3;
        firstU = c.FirstParameter();
        lastU = c.LastParameter();

        nbInt = 10;
        nbIntC3 = c.NbIntervals(C3);
        NCollection_Array1<double> disc = new(1, nbIntC3 + 1);

        if (nbIntC3 > 1)
        {
            c.Intervals(disc, C3);
        }
        else
        {
            disc[1] = firstU;
            disc[2] = lastU;
        }

        ui = new NCollection_HArray1<double>(0, nbIntC3 * nbInt);
        si = new NCollection_HArray1<double>(0, nbIntC3 * nbInt);

        ui.SetValue(0, firstU);
        si.SetValue(0, 0);

        for (j = 1, i = 1; j <= nbIntC3; j++)
        {
            step = (disc[j + 1] - disc[j]) / nbInt;
            for (k = 1; k <= nbInt; k++, i++)
            {
                ui[i] = ui.Value(i - 1) + step;
                si[i] = si.Value(i - 1) + Length(c, ui.Value(i - 1), ui.Value(i));
            }
        }

        double len = si.Value(si.Upper());
        for (i = si.Lower(); i <= si.Upper(); i++)
        {
            si[i] /= len;
        }

        _prevS = _firstS;
        _prevU = firstU;
    }

    /// <summary>Updates the tolerance of the length computations.</summary>
    public void SetTol(double tol)
    {
        _tolLen = tol;
    }

    public double FirstParameter() => _firstS;

    public double LastParameter() => _lastS;

    /// <summary>The number of intervals of continuity S.</summary>
    public int NbIntervals(Continuity s)
    {
        CurveOnSurface curOnSur = new();

        switch (_case)
        {
            case 1:
                return _c3D!.NbIntervals(s);
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                return curOnSur.NbIntervals(s);
            case 3:
            {
                int nbInt;
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                nbInt = curOnSur.NbIntervals(s);
                NCollection_Array1<double> t1 = new(1, nbInt + 1);
                curOnSur.Intervals(t1, s);
                // OCCT: Adaptor3d_CurveOnSurface::Load keeps the intervals cached for the first curve, so these are
                // the first curve's intervals again (occt-issues.md).
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                nbInt = curOnSur.NbIntervals(s);
                NCollection_Array1<double> t2 = new(1, nbInt + 1);
                curOnSur.Intervals(t2, s);

                NCollection_Sequence<double> fusion = new();
                global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(t1, t2, fusion);
                return fusion.Length() - 1;
            }
        }

        // POP pour WNT
        return 1;
    }

    /// <summary>Stores in T the curvilinear parameters bounding the intervals of continuity S.</summary>
    public void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        CurveOnSurface curOnSur = new();
        int i;

        switch (_case)
        {
            case 1:
                _c3D!.Intervals(t, s);
                break;
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                curOnSur.Intervals(t, s);
                break;
            case 3:
            {
                int nbInt;
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                nbInt = curOnSur.NbIntervals(s);
                NCollection_Array1<double> t1 = new(1, nbInt + 1);
                curOnSur.Intervals(t1, s);
                // OCCT: Adaptor3d_CurveOnSurface::Load keeps the intervals cached for the first curve, so these are
                // the first curve's intervals again (occt-issues.md).
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                nbInt = curOnSur.NbIntervals(s);
                NCollection_Array1<double> t2 = new(1, nbInt + 1);
                curOnSur.Intervals(t2, s);

                NCollection_Sequence<double> fusion = new();
                global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(t1, t2, fusion);

                for (i = 1; i <= fusion.Length(); i++)
                {
                    t[i] = fusion.Value(i);
                }
                break;
            }
        }

        for (i = 1; i <= t.Length(); i++)
        {
            t[i] = GetSParameter(t.Value(i));
        }
    }

    /// <summary>Restricts the curve to the curvilinear range [First, Last] (within [0, 1]).</summary>
    public void Trim(double first, double last, double tol)
    {
        if (first < 0 || last > 1)
        {
            throw new OutOfRangeException("Approx_CurvlinFunc::Trim");
        }
        if ((last - first) < tol)
        {
            return;
        }

        double firstU, lastU;
        CurveOnSurface curOnSur = new();
        CurveOnSurface? hCurOnSur;

        switch (_case)
        {
            case 1:
                _c3D = _c3D!.Trim(_firstU1, _lastU1, tol);
                firstU = GetUParameter(_c3D, first, 1);
                lastU = GetUParameter(_c3D, last, 1);
                _c3D = _c3D.Trim(firstU, lastU, tol);
                break;
            case 3:
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                hCurOnSur = curOnSur.Trim(_firstU2, _lastU2, tol) as CurveOnSurface;
                _c2D2 = hCurOnSur!.GetCurve();
                _surf2 = hCurOnSur.GetSurface();
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);

                // OCCT reads the parameter tables of the first curve (NumberOfCurve 1) for the second curve.
                firstU = GetUParameter(curOnSur, first, 1);
                lastU = GetUParameter(curOnSur, last, 1);
                hCurOnSur = curOnSur.Trim(firstU, lastU, tol) as CurveOnSurface;
                _c2D2 = hCurOnSur!.GetCurve();
                _surf2 = hCurOnSur.GetSurface();

                goto case 2;
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                hCurOnSur = curOnSur.Trim(_firstU1, _lastU1, tol) as CurveOnSurface;
                _c2D1 = hCurOnSur!.GetCurve();
                _surf1 = hCurOnSur.GetSurface();
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);

                firstU = GetUParameter(curOnSur, first, 1);
                lastU = GetUParameter(curOnSur, last, 1);
                hCurOnSur = curOnSur.Trim(firstU, lastU, tol) as CurveOnSurface;
                _c2D1 = hCurOnSur!.GetCurve();
                _surf1 = hCurOnSur.GetSurface();
                break;
        }
        _firstS = first;
        _lastS = last;
    }

    /// <summary>Computes the length of the curve.</summary>
    public void Length()
    {
        CurveOnSurface curOnSur = new();
        double firstU, lastU;

        switch (_case)
        {
            case 1:
                firstU = _c3D!.FirstParameter();
                lastU = _c3D.LastParameter();
                _length = Length(_c3D, firstU, lastU);
                _length1 = _length2 = 0;
                break;
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                firstU = curOnSur.FirstParameter();
                lastU = curOnSur.LastParameter();
                _length = Length(curOnSur, firstU, lastU);
                _length1 = _length2 = 0;
                break;
            case 3:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                firstU = curOnSur.FirstParameter();
                lastU = curOnSur.LastParameter();
                _length1 = Length(curOnSur, firstU, lastU);
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                firstU = curOnSur.FirstParameter();
                lastU = curOnSur.LastParameter();
                _length2 = Length(curOnSur, firstU, lastU);
                _length = (_length1 + _length2) / 2;
                break;
        }
    }

    /// <summary>Computes the length of the curve segment [FirstU, LastU].</summary>
    public double Length(CurveAdaptorBase c, double firstU, double lastU)
    {
        double length;

        length = AbscissaPoint.Length(c, firstU, lastU, _tolLen);
        return length;
    }

    public double GetLength() => _length;

    /// <summary>The curvilinear parameter of the original parameter U.</summary>
    public double GetSParameter(double u)
    {
        double s = 0, s1, s2;
        CurveOnSurface curOnSur = new();

        switch (_case)
        {
            case 1:
                s = GetSParameter(_c3D!, u, _length);
                break;
            case 2:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                s = GetSParameter(curOnSur, u, _length);
                break;
            case 3:
                curOnSur.Load(_c2D1!);
                curOnSur.Load(_surf1!);
                s1 = GetSParameter(curOnSur, u, _length1);
                curOnSur.Load(_c2D2!);
                curOnSur.Load(_surf2!);
                s2 = GetSParameter(curOnSur, u, _length2);
                s = (s1 + s2) / 2;
                break;
        }
        return s;
    }

    /// <summary>
    /// The original parameter of the curvilinear parameter S on C, read with the tables of the first curve
    /// (NumberOfCurve 1) or of the second.
    /// </summary>
    public double GetUParameter(CurveAdaptorBase c, double s, int numberOfCurve)
    {
        double deltaS, @base, u, length;
        int nbInt, nInterval, i;
        NCollection_HArray1<double> initUArray, initSArray;
        if (s < 0 || s > 1)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::GetUParameter");
        }

        if (numberOfCurve == 1)
        {
            initUArray = _ui1!;
            initSArray = _si1!;
            if (_case == 3)
            {
                length = _length1;
            }
            else
            {
                length = _length;
            }
        }
        else
        {
            initUArray = _ui2!;
            initSArray = _si2!;
            length = _length2;
        }

        nbInt = initUArray.Length() - 1;

        if (s == 1)
        {
            nInterval = nbInt - 1;
        }
        else
        {
            for (i = 0; i < nbInt; i++)
            {
                if ((initSArray.Value(i) <= s && s < initSArray.Value(i + 1)))
                {
                    break;
                }
            }
            nInterval = i;
        }
        if (s == initSArray.Value(nInterval))
        {
            return initUArray.Value(nInterval);
        }
        if (s == initSArray.Value(nInterval + 1))
        {
            return initUArray.Value(nInterval + 1);
        }

        @base = initUArray.Value(nInterval);
        deltaS = (s - initSArray.Value(nInterval)) * length;

        // to find an initial point
        Span<double> xi = stackalloc double[4], yi = stackalloc double[4];
        double uGuess;
        Findfourpoints(s, nInterval, initSArray, initUArray, _prevS, _prevU, xi, yi);
        uGuess = Cubic(s, xi, yi);

        u = new AbscissaPoint(c, deltaS, @base, uGuess, _tolLen).Parameter();

        _prevS = s;
        _prevU = u;

        return u;
    }

    // The curvilinear parameter of U on C.
    private double GetSParameter(CurveAdaptorBase c, double u, double len)
    {
        double s, origin;

        origin = c.FirstParameter();
        s = _firstS + Length(c, origin, u) / len;
        return s;
    }

    /// <summary>Evaluates the 3D curve (case 1) at S: the point or derivative Order in Result(0..2).</summary>
    public bool EvalCase1(double s, int order, NCollection_Array1<double> result)
    {
        if (_case != 1)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase1");
        }

        Point3D c;
        Vector3D dC_dU, dC_dS, d2C_dU2, d2C_dS2;
        double u, mag, dU_dS, d2U_dS2;

        u = GetUParameter(_c3D!, s, 1);

        switch (order)
        {
            case 0:
                _c3D!.D0(u, out c);

                result[0] = c.X;
                result[1] = c.Y;
                result[2] = c.Z;
                break;

            case 1:
                _c3D!.D1(u, out c, out dC_dU);
                mag = dC_dU.Magnitude;
                dU_dS = _length / mag;
                dC_dS = dC_dU * dU_dS;

                result[0] = dC_dS.X;
                result[1] = dC_dS.Y;
                result[2] = dC_dS.Z;
                break;

            case 2:
                _c3D!.D2(u, out c, out dC_dU, out d2C_dU2);
                mag = dC_dU.Magnitude;
                dU_dS = _length / mag;
                d2U_dS2 = -_length * dC_dU.Dot(d2C_dU2) * dU_dS / (mag * mag * mag);
                d2C_dS2 = d2C_dU2 * dU_dS * dU_dS + dC_dU * d2U_dS2;

                result[0] = d2C_dS2.X;
                result[1] = d2C_dS2.Y;
                result[2] = d2C_dS2.Z;
                break;

            default:
                result[0] = result[1] = result[2] = 0;
                return false;
        }
        return true;
    }

    /// <summary>Evaluates the curve on surface (case 2) at S: (u, v, x, y, z) or their derivatives in Result(0..4).</summary>
    public bool EvalCase2(double s, int order, NCollection_Array1<double> result)
    {
        if (_case != 2)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase2");
        }

        bool done;

        done = EvalCurOnSur(s, order, result, 1);

        return done;
    }

    /// <summary>
    /// Evaluates the two curves on surfaces (case 3) at S: (u1, v1, u2, v2) and the mean 3D point, or their
    /// derivatives, in Result(0..6).
    /// </summary>
    public bool EvalCase3(double s, int order, NCollection_Array1<double> result)
    {
        if (_case != 3)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase3");
        }

        NCollection_Array1<double> tmpRes1 = new(0, 4), tmpRes2 = new(0, 4);
        bool done;

        done = EvalCurOnSur(s, order, tmpRes1, 1);

        done = EvalCurOnSur(s, order, tmpRes2, 2) && done;

        result[0] = tmpRes1[0];
        result[1] = tmpRes1[1];
        result[2] = tmpRes2[0];
        result[3] = tmpRes2[1];
        result[4] = 0.5 * (tmpRes1[2] + tmpRes2[2]);
        result[5] = 0.5 * (tmpRes1[3] + tmpRes2[3]);
        result[6] = 0.5 * (tmpRes1[4] + tmpRes2[4]);

        return done;
    }

    private bool EvalCurOnSur(double s, int order, NCollection_Array1<double> result, int numberOfCurve)
    {
        Curve2DAdaptorBase cur2D;
        SurfaceAdaptorBase surf;
        double u = 0, length = 0;

        if (numberOfCurve == 1)
        {
            cur2D = _c2D1!;
            surf = _surf1!;
            CurveOnSurface curOnSur = new(_c2D1!, _surf1!);
            u = GetUParameter(curOnSur, s, 1);
            if (_case == 3)
            {
                length = _length1;
            }
            else
            {
                length = _length;
            }
        }
        else if (numberOfCurve == 2)
        {
            cur2D = _c2D2!;
            surf = _surf2!;
            CurveOnSurface curOnSur = new(_c2D2!, _surf2!);
            u = GetUParameter(curOnSur, s, 2);
            length = _length2;
        }
        else
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCurOnSur");
        }

        double mag, dU_dS, d2U_dS2, dV_dU, dW_dU, dV_dS, dW_dS, d2V_dS2, d2W_dS2, d2V_dU2, d2W_dU2;
        Point2D c2D;
        Point3D c;
        Vector2D dC2D_dU, d2C2D_dU2;
        Vector3D dC_dU, d2C_dU2, dC_dS, d2C_dS2, dS_dV, dS_dW, d2S_dV2, d2S_dW2, d2S_dVdW;

        switch (order)
        {
            case 0:
                cur2D.D0(u, out c2D);
                surf.D0(c2D.X, c2D.Y, out c);

                result[0] = c2D.X;
                result[1] = c2D.Y;
                result[2] = c.X;
                result[3] = c.Y;
                result[4] = c.Z;
                break;

            case 1:
                cur2D.D1(u, out c2D, out dC2D_dU);
                dV_dU = dC2D_dU.X;
                dW_dU = dC2D_dU.Y;
                surf.D1(c2D.X, c2D.Y, out c, out dS_dV, out dS_dW);
                dC_dU = dS_dV * dV_dU + dS_dW * dW_dU;
                mag = dC_dU.Magnitude;
                dU_dS = length / mag;

                dV_dS = dV_dU * dU_dS;
                dW_dS = dW_dU * dU_dS;
                dC_dS = dC_dU * dU_dS;

                result[0] = dV_dS;
                result[1] = dW_dS;
                result[2] = dC_dS.X;
                result[3] = dC_dS.Y;
                result[4] = dC_dS.Z;
                break;

            case 2:
                cur2D.D2(u, out c2D, out dC2D_dU, out d2C2D_dU2);
                dV_dU = dC2D_dU.X;
                dW_dU = dC2D_dU.Y;
                d2V_dU2 = d2C2D_dU2.X;
                d2W_dU2 = d2C2D_dU2.Y;
                surf.D2(c2D.X, c2D.Y, out c, out dS_dV, out dS_dW, out d2S_dV2, out d2S_dW2, out d2S_dVdW);
                dC_dU = dS_dV * dV_dU + dS_dW * dW_dU;
                d2C_dU2 = (d2S_dV2 * dV_dU + d2S_dVdW * dW_dU) * dV_dU + dS_dV * d2V_dU2
                          + (d2S_dVdW * dV_dU + d2S_dW2 * dW_dU) * dW_dU + dS_dW * d2W_dU2;
                mag = dC_dU.Magnitude;
                dU_dS = length / mag;
                d2U_dS2 = -length * dC_dU.Dot(d2C_dU2) * dU_dS / (mag * mag * mag);

                dV_dS = dV_dU * dU_dS;
                dW_dS = dW_dU * dU_dS;
                d2V_dS2 = d2V_dU2 * dU_dS * dU_dS + dV_dU * d2U_dS2;
                d2W_dS2 = d2W_dU2 * dU_dS * dU_dS + dW_dU * d2U_dS2;

                // OCCT recomputes d2U_dS2 here (another formula) and never reads it again.
                d2C_dS2 = (d2S_dV2 * dV_dS + d2S_dVdW * dW_dS) * dV_dS + dS_dV * d2V_dS2
                          + (d2S_dW2 * dW_dS + d2S_dVdW * dV_dS) * dW_dS + dS_dW * d2W_dS2;

                result[0] = d2V_dS2;
                result[1] = d2W_dS2;
                result[2] = d2C_dS2.X;
                result[3] = d2C_dS2.Y;
                result[4] = d2C_dS2.Z;
                break;

            default:
                result[0] = result[1] = result[2] = result[3] = result[4] = 0;
                return false;
        }
        return true;
    }
}
