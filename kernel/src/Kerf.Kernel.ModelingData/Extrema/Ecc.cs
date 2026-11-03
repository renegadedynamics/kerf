// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ECC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;
using static Kerf.Engine.Extrema.GGenExtCC;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The minimal distances between two 3D curves, found by global optimisation (math_GlobOptMin on
/// <see cref="GlobOptFuncCcc2"/>) over the pairs of continuity intervals of the curves. When the minimum is
/// reached along a whole range (parallel curves), one solution is kept and <see cref="IsParallel"/> is true.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GGenExtCC (Extrema_ECC). An algorithm object (decision 117): set
/// the curves, then <c>Perform</c>. Keeps references to the curves, as OCCT does. As in OCCT, <c>Perform</c> does not
/// clear the solutions of a previous <c>Perform</c> (occt-issues.md).
/// </remarks>
public class Ecc
{
    private bool _isFindSingleSolution;
    private bool _parallel;
    private double _curveMinTol;
    private readonly math_Vector _lowBorder;
    private readonly math_Vector _uppBorder;
    private readonly NCollection_Sequence<double> _points1 = new();
    private readonly NCollection_Sequence<double> _points2 = new();
    private readonly CurveAdaptorBase?[] _c = new CurveAdaptorBase?[2];
    private bool _done;

    public Ecc()
    {
        _isFindSingleSolution = false;
        _parallel = false;
        _curveMinTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _lowBorder = new math_Vector(1, 2);
        _uppBorder = new math_Vector(1, 2);
        _done = false;
        _c[0] = _c[1] = null;
    }

    /// <summary>The algorithm for the curves theC1 and theC2 on their whole ranges.</summary>
    public Ecc(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _isFindSingleSolution = false;
        _parallel = false;
        _curveMinTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _lowBorder = new math_Vector(1, 2);
        _uppBorder = new math_Vector(1, 2);
        _done = false;
        _c[0] = c1;
        _c[1] = c2;
        _lowBorder[1] = c1.FirstParameter();
        _lowBorder[2] = c2.FirstParameter();
        _uppBorder[1] = c1.LastParameter();
        _uppBorder[2] = c2.LastParameter();
    }

    /// <summary>The algorithm for theC1 on [theUinf, theUsup] and theC2 on [theVinf, theVsup].</summary>
    public Ecc(CurveAdaptorBase c1, CurveAdaptorBase c2, double uinf, double usup, double vinf,
        double vsup)
    {
        _isFindSingleSolution = false;
        _parallel = false;
        _curveMinTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _lowBorder = new math_Vector(1, 2);
        _uppBorder = new math_Vector(1, 2);
        _done = false;
        _c[0] = c1;
        _c[1] = c2;
        _lowBorder[1] = uinf;
        _lowBorder[2] = vinf;
        _uppBorder[1] = usup;
        _uppBorder[2] = vsup;
    }

    /// <summary>Sets the curves and their ranges.</summary>
    public void SetParams(CurveAdaptorBase c1, CurveAdaptorBase c2, double uinf, double usup,
        double vinf, double vsup)
    {
        _c[0] = c1;
        _c[1] = c2;
        _lowBorder[1] = uinf;
        _lowBorder[2] = vinf;
        _uppBorder[1] = usup;
        _uppBorder[2] = vsup;
    }

    /// <summary>Sets the parametric tolerance under which two solutions are the same.</summary>
    public void SetTolerance(double tol) => _curveMinTol = tol;

    /// <summary>Asks for one solution only (faster).</summary>
    public void SetSingleSolutionFlag(bool flag) => _isFindSingleSolution = flag;

    public bool GetSingleSolutionFlag() => _isFindSingleSolution;

    /// <summary>Computes the minimal distances.</summary>
    public void Perform()
    {
        // a second Perform replaces the first one's solutions (OCCT appended to them: decision 16703)
        _points1.Clear();
        _points2.Clear();
        _done = false;
        _parallel = false;

        CurveAdaptorBase c1 = _c[0]!;
        CurveAdaptorBase c2 = _c[1]!;

        Span<int> nbInter = stackalloc int[2];
        Continuity continuity = global::Kerf.Geometry.Continuity.C2;
        nbInter[0] = c1.NbIntervals(continuity);
        nbInter[1] = c2.NbIntervals(continuity);

        if (nbInter[0] * nbInter[1] > 100)
        {
            continuity = global::Kerf.Geometry.Continuity.C1;
            nbInter[0] = c1.NbIntervals(continuity);
            nbInter[1] = c2.NbIntervals(continuity);
        }

        Span<double> l = stackalloc double[2];
        int indmax = -1, indmin = -1;
        const double mult = 20.0;
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(c1.FirstParameter()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(c1.LastParameter())
              || global::Kerf.Engine.Foundation.Precision.IsInfinite(c2.FirstParameter()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(c2.LastParameter())))
        {
            l[0] = AbscissaPoint.Length(c1);
            l[1] = AbscissaPoint.Length(c2);
            if (l[0] / nbInter[0] > mult * l[1] / nbInter[1])
            {
                indmax = 0;
                indmin = 1;
            }
            else if (l[1] / nbInter[1] > mult * l[0] / nbInter[0])
            {
                indmax = 1;
                indmin = 0;
            }
        }
        int nbIntOpt = 0;
        if (indmax >= 0)
        {
            nbIntOpt = RealToInt(l[indmax] * nbInter[indmin] / l[indmin] / (mult / 4.0)) + 1;
            if (nbIntOpt > 100 || nbIntOpt < nbInter[indmax])
            {
                indmax = -1;
            }
            else
            {
                if (nbIntOpt * nbInter[indmin] > 100)
                {
                    nbIntOpt = 100 / nbInter[indmin];
                    if (nbIntOpt < nbInter[indmax])
                    {
                        indmax = -1;
                    }
                }
            }
        }

        NCollection_HArray1<double> intervals1 = new(1, nbInter[0] + 1);
        NCollection_HArray1<double> intervals2 = new(1, nbInter[1] + 1);
        c1.Intervals(intervals1.ChangeArray1(), continuity);
        c2.Intervals(intervals2.ChangeArray1(), continuity);
        if (indmax >= 0)
        {
            if (indmax == 0)
            {
                Extrema_GGenExtCC_ChangeIntervals(ref intervals1, nbIntOpt);
                nbInter[0] = intervals1.Length() - 1;
            }
            else
            {
                Extrema_GGenExtCC_ChangeIntervals(ref intervals2, nbIntOpt);
                nbInter[1] = intervals2.Length() - 1;
            }
        }
        if (c1.IsClosed && nbInter[0] == 1)
        {
            Extrema_GGenExtCC_ChangeIntervals(ref intervals1, 3);
            nbInter[0] = intervals1.Length() - 1;
        }
        if (c2.IsClosed && nbInter[1] == 1)
        {
            Extrema_GGenExtCC_ChangeIntervals(ref intervals2, 3);
            nbInter[1] = intervals2.Length() - 1;
        }

        const double maxLC = 10000.0;
        double lc = 100.0;
        double maxDer1 = 1.0 / c1.Resolution(1.0);
        double maxDer2 = 1.0 / c2.Resolution(1.0);
        double maxDer = Max(maxDer1, maxDer2) * M_SQRT2;
        if (lc > maxDer)
        {
            lc = maxDer;
        }

        bool isConstLockedFlag = false;
        const double cr = 0.001;
        if (maxDer1 / maxDer < cr || maxDer2 / maxDer < cr)
        {
            isConstLockedFlag = true;
        }
        if (maxDer > maxLC)
        {
            lc = maxLC;
            isConstLockedFlag = true;
        }
        if (c1.GetType() == Line)
        {
            maxDer = 1.0 / c2.Resolution(1.0);
            if (lc > maxDer)
            {
                isConstLockedFlag = true;
                lc = maxDer;
            }
        }
        if (c2.GetType() == Line)
        {
            maxDer = 1.0 / c1.Resolution(1.0);
            if (lc > maxDer)
            {
                isConstLockedFlag = true;
                lc = maxDer;
            }
        }

        GlobOptFuncCcc2 func = new(c1, c2);
        if (lc < maxLC || maxDer > maxLC)
        {
            math_Vector t = new(1, 2), g = new(1, 2);
            double f0, maxG = 0.0;
            double t1, t2, dt1, dt2;
            int n1 = 21, n2 = 21, i1, i2;
            dt1 = (c1.LastParameter() - c1.FirstParameter()) / (n1 - 1);
            dt2 = (c2.LastParameter() - c2.FirstParameter()) / (n2 - 1);
            for (i1 = 1, t1 = c1.FirstParameter(); i1 <= n1; ++i1, t1 += dt1)
            {
                t[1] = t1;
                for (i2 = 1, t2 = c2.FirstParameter(); i2 <= n2; ++i2, t2 += dt2)
                {
                    t[2] = t2;
                    func.Values(t, out f0, g);
                    double mod = g[1] * g[1] + g[2] * g[2];
                    maxG = Max(maxG, mod);
                }
            }
            maxG = Math.Sqrt(maxG);
            if (maxG > maxDer)
            {
                lc = Min(maxG, maxLC);
                isConstLockedFlag = true;
            }
            if (maxG > 100.0 * maxLC)
            {
                lc = 100.0 * maxLC;
                isConstLockedFlag = true;
            }
            else if (maxG < 0.1 * maxDer)
            {
                isConstLockedFlag = true;
            }
        }
        GlobOptMin finder = new(func, _lowBorder, _uppBorder, lc);
        finder.SetLipConstState(isConstLockedFlag);
        finder.SetContinuity(continuity == global::Kerf.Geometry.Continuity.C2 ? 2 : 1);
        double discTol = 1.0e-2;
        double valueTol = 1.0e-2;
        double sameTol = _curveMinTol / (discTol);
        finder.SetTol(discTol, sameTol);
        finder.SetFunctionalMinimalValue(0.0);

        double cellSize = Max(Max(intervals1.Last() - intervals1.First(),
                                   intervals2.Last() - intervals2.First())
                                 * global::Kerf.Engine.Foundation.Precision.PConfusion() / (2.0 * M_SQRT2),
                               global::Kerf.Engine.Foundation.Precision.PConfusion());
        Extrema_GGenExtCC_PointsInspector inspector = new(cellSize);
        CellFilter<Extrema_GGenExtCC_PointsInspector, Xy, Xy> filter = new(cellSize);
        List<Xy> pnts = new();

        int i, j, k;
        math_Vector firstBorderInterval = new(1, 2);
        math_Vector secondBorderInterval = new(1, 2);
        double f = RealLast();
        double currF = RealLast();
        for (i = 1; i <= nbInter[0]; i++)
        {
            for (j = 1; j <= nbInter[1]; j++)
            {
                firstBorderInterval[1] = intervals1.Value(i);
                firstBorderInterval[2] = intervals2.Value(j);
                secondBorderInterval[1] = intervals1.Value(i + 1);
                secondBorderInterval[2] = intervals2.Value(j + 1);

                finder.SetLocalParams(firstBorderInterval, secondBorderInterval);
                finder.Perform(GetSingleSolutionFlag());

                currF = finder.GetF();
                if (currF >= f + sameTol * valueTol)
                {
                    continue;
                }

                if (currF > f - sameTol * valueTol)
                {
                    if (currF < f)
                    {
                        f = currF;
                    }
                }
                else
                {
                    f = currF;
                    filter.Reset(cellSize);
                    pnts.Clear();
                }

                math_Vector sol = new(1, 2);
                for (k = 1; k <= finder.ExtremaCount; k++)
                {
                    finder.Points(k, sol);
                    Xy pnt2d = new(sol[1], sol[2]);

                    Xy xYmin = Extrema_GGenExtCC_PointsInspector.Shift(pnt2d, -cellSize);
                    Xy xYmax = Extrema_GGenExtCC_PointsInspector.Shift(pnt2d, cellSize);

                    inspector.ClearFind();
                    inspector.SetCurrent(pnt2d);
                    filter.Inspect(xYmin, xYmax, ref inspector);
                    if (!inspector.IsFind())
                    {
                        filter.Add(pnt2d, pnt2d);
                        pnts.Add(new Xy(sol[1], sol[2]));
                    }
                }
            }
        }

        int nbSol = pnts.Count;
        if (nbSol == 0)
        {
            _done = false;
            return;
        }

        _done = true;

        if (nbSol == 1)
        {
            Xy aSol = pnts[0];
            _points1.Append(aSol.X());
            _points2.Append(aSol.Y());
            return;
        }

        CollectionsMarshal.AsSpan(pnts).Sort(Extrema_GGenExtCC_Compare);

        NCollection_List<int> solutions = new();

        bool bSaveSolution = true;
        bool bDirsCoinside = true;
        bool bDifferentSolutions = false;

        bool isParallel = true;
        double val = 0.0;
        math_Vector vec = new(1, 2, 0.0);

        for (int idx = 0; idx < nbSol - 1; idx++)
        {
            Xy current = pnts[idx];
            Xy next = pnts[idx + 1];

            vec[1] = (current.X() + next.X()) * 0.5;
            vec[2] = (current.Y() + next.Y()) * 0.5;

            func.Value(vec, out val);
            if (Math.Abs(val - f) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (bSaveSolution)
                {
                    solutions.Append(idx);
                    bSaveSolution = false;
                }
            }
            else
            {
                isParallel = false;
                solutions.Append(idx);
                bSaveSolution = true;
            }

            if (!bDifferentSolutions)
            {
                if (next.X() > current.X())
                {
                    if (next.Y() > current.Y())
                    {
                        bDifferentSolutions = true;
                        bDirsCoinside = true;
                    }
                    else if (next.Y() < current.Y())
                    {
                        bDifferentSolutions = true;
                        bDirsCoinside = false;
                    }
                }
            }
        }
        solutions.Append(nbSol - 1);

        if (!bDifferentSolutions)
        {
            isParallel = false;
        }

        if (isParallel)
        {
            ReadOnlySpan<double> aT1 = [_lowBorder[1], _uppBorder[1]];
            ReadOnlySpan<double> aT2 = [bDirsCoinside ? _lowBorder[2] : _uppBorder[2],
                                        bDirsCoinside ? _uppBorder[2] : _lowBorder[2]];

            CurvePointExtrema extPC1 = new(), extPC2 = new();
            extPC1.Initialize(c1, _lowBorder[1], _uppBorder[1]);
            extPC2.Initialize(c2, _lowBorder[2], _uppBorder[2]);

            for (int iT = 0; isParallel && (iT < 2); ++iT)
            {
                double dist1 = Extrema_GGenExtCC_ProjPOnC(c1.Value(aT1[iT]), extPC2);
                double dist2 = Extrema_GGenExtCC_ProjPOnC(c2.Value(aT2[iT]), extPC1);
                isParallel = (Math.Abs(Min(dist1, dist2) - f * f) < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        if (isParallel)
        {
            Xy aSol = pnts[0];
            _points1.Append(aSol.X());
            _points2.Append(aSol.Y());
            _parallel = true;
        }
        else
        {
            NCollection_List<int>.Iterator itSol = new(solutions);
            for (; itSol.More(); itSol.Next())
            {
                Xy aSol = pnts[itSol.Value()];
                _points1.Append(aSol.X());
                _points2.Append(aSol.Y());
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>True if the curves are parallel (the minimum is reached along a range; one solution is kept).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _parallel;
        }
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _points1.Length();
        }
    }

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int n = 1)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return CurveTool.Value(_c[0]!, _points1[n])
            .SquareDistance(CurveTool.Value(_c[1]!, _points2[n]));
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public void Points(int n, out POnCurv p1, out POnCurv p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        p1 = new POnCurv(_points1[n], CurveTool.Value(_c[0]!, _points1[n]));
        p2 = new POnCurv(_points2[n], CurveTool.Value(_c[1]!, _points2[n]));
    }
}
