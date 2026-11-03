// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between a curve and a surface in the general case: a global minimum search by
/// particle swarm optimisation (math_PSO) over sampled points, over (t, u, v), over (u, v) for a conic or over t for
/// a quadric, refined by math_FunctionSetRoot on <see cref="FuncExtCS"/>.
/// </summary>
/// <remarks>
/// The surface (set by <see cref="Initialize(SurfaceAdaptorBase, int, int, double)"/>) and the curve (given to
/// <see cref="Perform(CurveAdaptorBase, int, double)"/>) are kept by reference, as OCCT keeps them: they must outlive
/// the algorithm. Infinite ranges are bounded at 1e10 (23 on hyperbolas and surfaces built on them).
/// </remarks>
public class GenExtCS
{
    private const double MaxParamVal = 1.0e+10;
    private const double ABorderDivisor = 1.0e+4;
    private const double HyperbolaLimit = 23.0; // ln(MaxParamVal)

    private bool _done;
    private double _mytmin;
    private double _mytsup;
    private double _myumin;
    private double _myusup;
    private double _myvmin;
    private double _myvsup;
    private int _mytsample;
    private int _myusample;
    private int _myvsample;
    private double _mytol1;
    private double _mytol2;
    private readonly FuncExtCS _f = new();
    private SurfaceAdaptorBase? _s;
    private NCollection_HArray2<Point3D>? _surfPnts;

    private static bool IsQuadric(SurfaceKind sType)
    {
        if (sType == Plane)
        {
            return true;
        }
        if (sType == SurfaceKind.Cylinder)
        {
            return true;
        }
        if (sType == SurfaceKind.Cone)
        {
            return true;
        }
        if (sType == SurfaceKind.Sphere)
        {
            return true;
        }
        if (sType == SurfaceKind.Torus)
        {
            return true;
        }
        return false;
    }

    private static bool IsConic(CurveKind cType)
    {
        if (cType == Line)
        {
            return true;
        }
        if (cType == Circle)
        {
            return true;
        }
        if (cType == Ellipse)
        {
            return true;
        }
        if (cType == Hyperbola)
        {
            return true;
        }
        if (cType == Parabola)
        {
            return true;
        }
        return false;
    }

    // restrict maximal parameter on hyperbola to avoid FPE
    private static double GetCurvMaxParamVal(CurveAdaptorBase c)
    {
        if (c.GetType() == Hyperbola)
        {
            return HyperbolaLimit;
        }
        if (c.GetType() == OffsetCurve)
        {
            Curve3D bc = c.OffsetCurve().BasisCurve();
            TrimmedCurve3D? tc = bc as TrimmedCurve3D;
            if (tc is not null)
            {
                bc = tc.BasisCurve();
            }
            if (bc is HyperbolaCurve3D)
            {
                return HyperbolaLimit;
            }
        }
        return MaxParamVal;
    }

    // restrict maximal parameter on surfaces based on hyperbola to avoid FPE
    private static void GetSurfMaxParamVals(SurfaceAdaptorBase s, out double umax, out double vmax)
    {
        umax = vmax = MaxParamVal;

        if (s.GetType() == SurfaceOfExtrusion)
        {
            umax = GetCurvMaxParamVal(s.BasisCurve());
        }
        else if (s.GetType() == SurfaceOfRevolution)
        {
            vmax = GetCurvMaxParamVal(s.BasisCurve());
        }
        else if (s.GetType() == SurfaceKind.OffsetSurface)
        {
            GetSurfMaxParamVals(s.BasisSurface(), out umax, out vmax);
        }
    }

    /// <summary>An empty algorithm.</summary>
    public GenExtCS()
    {
        _done = false;
        _mytmin = 0.0;
        _mytsup = 0.0;
        _myumin = 0.0;
        _myusup = 0.0;
        _myvmin = 0.0;
        _myvsup = 0.0;
        _mytsample = 0;
        _myusample = 0;
        _myvsample = 0;
        _mytol1 = 0.0;
        _mytol2 = 0.0;
        _s = null;
    }

    /// <summary>
    /// Computes all the extrema between C and S over their whole ranges. NbU and NbV sample the surface, NbT the
    /// curve, to locate the starting points.
    /// </summary>
    public GenExtCS(CurveAdaptorBase c, SurfaceAdaptorBase s, int nbT, int nbU, int nbV, double tol1, double tol2)
    {
        Initialize(s, nbU, nbV, tol2);
        Perform(c, nbT, tol1);
    }

    /// <summary>Computes all the extrema between C on [tmin, tsup] and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public GenExtCS(CurveAdaptorBase c, SurfaceAdaptorBase s, int nbT, int nbU, int nbV, double tmin,
        double tsup, double umin, double usup, double vmin, double vsup, double tol1, double tol2)
    {
        Initialize(s, nbU, nbV, umin, usup, vmin, vsup, tol2);
        Perform(c, nbT, tmin, tsup, tol1);
    }

    /// <summary>Sets the surface over its whole range and samples it.</summary>
    public void Initialize(SurfaceAdaptorBase s, int nbU, int nbV, double tol2)
    {
        _myumin = s.FirstUParameter();
        _myusup = s.LastUParameter();
        _myvmin = s.FirstVParameter();
        _myvsup = s.LastVParameter();
        Initialize(s, nbU, nbV, _myumin, _myusup, _myvmin, _myvsup, tol2);
    }

    /// <summary>Sets the surface over [Umin, Usup] x [Vmin, Vsup] and samples it.</summary>
    public void Initialize(SurfaceAdaptorBase s, int nbU, int nbV, double umin, double usup, double vmin, double vsup,
        double tol2)
    {
        _s = s;
        _myusample = nbU;
        _myvsample = nbV;
        _myumin = umin;
        _myusup = usup;
        _myvmin = vmin;
        _myvsup = vsup;
        _mytol2 = tol2;

        GetSurfMaxParamVals(_s, out double umaxpar, out double vmaxpar);

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myusup))
        {
            _myusup = umaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myumin))
        {
            _myumin = -umaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myvsup))
        {
            _myvsup = vmaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myvmin))
        {
            _myvmin = -vmaxpar;
        }

        double du = (_myusup - _myumin) / ABorderDivisor;
        double dv = (_myvsup - _myvmin) / ABorderDivisor;
        double minU = _myumin + du;
        double minV = _myvmin + dv;
        double maxU = _myusup - du;
        double maxV = _myvsup - dv;

        double stepSU = (maxU - minU) / _myusample;
        double stepSV = (maxV - minV) / _myvsample;

        _surfPnts = new NCollection_HArray2<Point3D>(0, _myusample, 0, _myvsample);

        // Build UV parameter arrays for batch evaluation
        NCollection_Array1<double> uParams = new(0, _myusample);
        NCollection_Array1<double> vParams = new(0, _myvsample);

        double su = minU;
        for (int sui = 0; sui <= _myusample; sui++, su += stepSU)
        {
            uParams.SetValue(sui, su);
        }
        double sv = minV;
        for (int svi = 0; svi <= _myvsample; svi++, sv += stepSV)
        {
            vParams.SetValue(svi, sv);
        }

        // Use batch grid evaluation for optimized surface point computation
        SurfaceGrid evaluator = new(_s);

        NCollection_Array2<Point3D> grid = evaluator.EvaluateGrid(uParams, vParams);

        // aGrid is 1-based, mySurfPnts is 0-based - adjust indexing
        for (int sui = 0; sui <= _myusample; sui++)
        {
            for (int svi = 0; svi <= _myvsample; svi++)
            {
                _surfPnts.ChangeValue(sui, svi) = grid.Value(sui + 1, svi + 1);
            }
        }
    }

    /// <summary>Computes the extrema with C over its whole range (the surface must have been initialised).</summary>
    public void Perform(CurveAdaptorBase c, int nbT, double tol1)
    {
        _mytmin = c.FirstParameter();
        _mytsup = c.LastParameter();
        Perform(c, nbT, _mytmin, _mytsup, tol1);
    }

    /// <summary>Computes the extrema with C over [tmin, tsup] (the surface must have been initialised).</summary>
    public void Perform(CurveAdaptorBase c, int nbT, double tmin, double tsup, double tol1)
    {
        _done = false;
        if (_s is null)
        {
            throw new NotDoneException("Extrema_GenExtCS: the surface is not initialized");
        }
        _f.Initialize(c, _s);
        _mytmin = tmin;
        _mytsup = tsup;
        _mytol1 = tol1;
        _mytsample = nbT;
        // Modification by lvt to trim the surface not to infinity but to +/- 10000

        double trimusup = _myusup, trimumin = _myumin, trimvsup = _myvsup, trimvmin = _myvmin;
        double cMaxVal = GetCurvMaxParamVal(c);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_mytsup))
        {
            _mytsup = cMaxVal;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_mytmin))
        {
            _mytmin = -cMaxVal;
        }
        //
        int nbVar = 3;
        SurfaceKind sType = _s.GetType();
        if (IsQuadric(sType))
        {
            nbVar = 1;
        }
        else
        {
            CurveKind cType = c.GetType();
            if (IsConic(cType))
            {
                nbVar = 2;
            }
        }

        math_Vector Tol = new(1, 3), tuv = new(1, 3), tuVinf = new(1, 3), tuVsup = new(1, 3);
        //
        Tol[1] = _mytol1;
        Tol[2] = _mytol2;
        Tol[3] = _mytol2;
        //
        // Number of particles used in PSO algorithm (particle swarm optimization).
        const int nbParticles = 48;

        int nbIntC = 1;
        if (c.IsClosed || c.IsPeriodic)
        {
            double period = c.Period();
            if (c.LastParameter() - c.FirstParameter() > 2.0 * period / 3.0)
            {
                nbIntC = 2;
            }
        }

        int anInt;
        double dT = (_mytsup - _mytmin) / nbIntC;
        for (anInt = 1; anInt <= nbIntC; anInt++)
        {
            tuVinf[1] = _mytmin + (anInt - 1) * dT;
            tuVinf[2] = trimumin;
            tuVinf[3] = trimvmin;
            //
            tuVsup[1] = tuVinf[1] + dT; // mytsup;
            tuVsup[2] = trimusup;
            tuVsup[3] = trimvsup;
            //
            if (nbVar == 3)
            {
                GlobMinGenCS(c, nbParticles, tuVinf, tuVsup, tuv);
            }
            else if (nbVar == 2)
            {
                GlobMinConicS(c, nbParticles, tuVinf, tuVsup, tuv);
            }
            else
            {
                GlobMinCQuadric(c, nbParticles, tuVinf, tuVsup, tuv);
            }

            // Find min approximation
            FunctionSetRoot a = new(_f, Tol);
            a.Perform(_f, tuv, tuVinf, tuVsup);
        }
        if (nbIntC > 1 && _f.ExtCount > 1)
        {
            // Try to remove "false" extrema caused by dividing curve interval
            NCollection_Sequence<double> sqDists = _f.SquareDistances();
            NCollection_Sequence<POnCurv> pntsOnCrv = _f.PointsOnCurve();
            NCollection_Sequence<POnSurf> pntsOnSurf = _f.PointsOnSurf();
            NCollection_Sequence<double> sqDists1 = new(sqDists);
            NCollection_Sequence<POnCurv> pntsOnCrv1 = new(pntsOnCrv);
            NCollection_Sequence<POnSurf> pntsOnSurf1 = new(pntsOnSurf);

            double minDist = sqDists[1];
            int i;
            for (i = 2; i <= sqDists.Length(); ++i)
            {
                double dist = sqDists[i];
                if (dist < minDist)
                {
                    minDist = dist;
                }
            }
            sqDists.Clear();
            pntsOnCrv.Clear();
            pntsOnSurf.Clear();
            double tol = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
            for (i = 1; i <= sqDists1.Length(); ++i)
            {
                double dist = sqDists1[i];
                if (Math.Abs(dist - minDist) <= tol)
                {
                    sqDists.Append(dist);
                    pntsOnCrv.Append(pntsOnCrv1[i]);
                    pntsOnSurf.Append(pntsOnSurf1[i]);
                }
            }
        }
        _done = true;
    }

    private void GlobMinGenCS(CurveAdaptorBase c, int nbParticles, math_Vector tuVinf, math_Vector tuVsup,
        math_Vector tuv)
    {
        PsoParticlesPool particles = new(nbParticles, 3);

        math_Vector minTUV = new(1, 3);
        minTUV.Initialized(tuVinf + (tuVsup - tuVinf) / ABorderDivisor);

        math_Vector maxTUV = new(1, 3);
        maxTUV.Initialized(tuVsup - (tuVsup - tuVinf) / ABorderDivisor);

        double stepCU = (maxTUV[1] - minTUV[1]) / _mytsample;
        double stepSU = (maxTUV[2] - minTUV[2]) / _myusample;
        double stepSV = (maxTUV[3] - minTUV[3]) / _myvsample;

        // Correct number of curve samples in case of low resolution
        int newCsample = _mytsample;
        double scaleFactor = 5.0;
        double resolutionCU = stepCU / c.Resolution(1.0);

        double minResolution = scaleFactor
                                * Min(resolutionCU,
                                    Min(stepSU / _s!.UResolution(1.0), stepSV / _s.VResolution(1.0)));

        if (minResolution > Epsilon(1.0))
        {
            if (resolutionCU > minResolution)
            {
                const int maxNbNodes = 50;

                newCsample = Min(maxNbNodes, RealToInt(_mytsample * resolutionCU / minResolution));

                stepCU = (maxTUV[1] - minTUV[1]) / newCsample;
            }
        }

        // Pre-compute curve sample points.
        NCollection_Array1<Point3D> curvPnts = new(0, newCsample);

        double cu1 = minTUV[1];
        for (int cui = 0; cui <= newCsample; cui++, cu1 += stepCU)
        {
            curvPnts.SetValue(cui, c.Value(cu1));
        }

        Particle particle = particles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double su = minTUV[2];
        for (int sui = 0; sui <= _myusample; sui++, su += stepSU)
        {
            double sv = minTUV[3];
            for (int svi = 0; svi <= _myvsample; svi++, sv += stepSV)
            {
                double cu2 = minTUV[1];
                for (int cui = 0; cui <= newCsample; cui++, cu2 += stepCU)
                {
                    double sqDist = _surfPnts!.Value(sui, svi).SquareDistance(curvPnts.Value(cui));

                    if (sqDist < particle.Distance)
                    {
                        particle.Position[0] = cu2;
                        particle.Position[1] = su;
                        particle.Position[2] = sv;

                        particle.BestPosition[0] = cu2;
                        particle.BestPosition[1] = su;
                        particle.BestPosition[2] = sv;

                        particle.Distance = sqDist;
                        particle.BestDistance = sqDist;

                        particle = particles.GetWorstParticle();
                    }
                }
            }
        }

        math_Vector step = new(1, 3);
        step[1] = stepCU;
        step[2] = stepSU;
        step[3] = stepSV;

        // Find min approximation
        GlobOptFuncCS func = new(c, _s);
        Pso pso = new(func, tuVinf, tuVsup, step);
        pso.Perform(particles, nbParticles, out _, tuv);
    }

    private void GlobMinConicS(CurveAdaptorBase c, int nbParticles, math_Vector tuVinf, math_Vector tuVsup,
        math_Vector tuv)
    {
        int nbVar = 2;
        math_Vector uVinf = new(1, nbVar), uVsup = new(1, nbVar), uv = new(1, nbVar);
        int i;
        for (i = 1; i <= nbVar; ++i)
        {
            uVinf[i] = tuVinf[i + 1];
            uVsup[i] = tuVsup[i + 1];
        }
        //
        //
        PsoParticlesPool particles = new(nbParticles, nbVar);

        math_Vector minUV = new(1, nbVar);
        minUV.Initialized(uVinf + (uVsup - uVinf) / ABorderDivisor);

        math_Vector maxUV = new(1, nbVar);
        maxUV.Initialized(uVsup - (uVsup - uVinf) / ABorderDivisor);

        // Increase numbers of UV samples to improve searching global minimum
        int addsample = Max(_mytsample / 2, 3);
        int usample = _myusample + addsample;
        int vsample = _myvsample + addsample;
        //
        double stepSU = (maxUV[1] - minUV[1]) / usample;
        double stepSV = (maxUV[2] - minUV[2]) / vsample;
        //
        GlobOptFuncConicS func = new(_s!, uVinf[1], uVsup[1], uVinf[2], uVsup[2]);
        func.LoadConic(c, tuVinf[1], tuVsup[1]);

        Particle particle = particles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double su = minUV[1];

        for (int sui = 0; sui <= usample; sui++, su += stepSU)
        {
            uv[1] = su;
            double sv = minUV[2];
            for (int svi = 0; svi <= vsample; svi++, sv += stepSV)
            {
                uv[2] = sv;
                if (!func.Value(uv, out double sqDist))
                {
                    sqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                }

                if (sqDist < particle.Distance)
                {
                    particle.Position[0] = su;
                    particle.Position[1] = sv;

                    particle.BestPosition[0] = su;
                    particle.BestPosition[1] = sv;

                    particle.Distance = sqDist;
                    particle.BestDistance = sqDist;

                    particle = particles.GetWorstParticle();
                }
            }
        }

        math_Vector step = new(1, nbVar);
        step[1] = stepSU;
        step[2] = stepSV;

        // Find min approximation
        Pso pso = new(func, uVinf, uVsup, step);
        pso.Perform(particles, nbParticles, out _, uv);
        //
        double ct = func.ConicParameter(uv);
        if (c.IsPeriodic)
        {
            if (ct < tuVinf[1] - global::Kerf.Engine.Foundation.Precision.PConfusion() || ct > tuVsup[1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ct = global::Kerf.Engine.Geometry.ElCLib.InPeriod(ct, tuVinf[1], tuVinf[1] + 2.0 * M_PI);
            }
        }

        tuv[1] = ct;
        tuv[2] = uv[1];
        tuv[3] = uv[2];

        bool isBadSol = false;
        _s!.D1(uv[1], uv[2], out Point3D pOnS, out Vector3D du, out Vector3D dv);
        c.D1(ct, out Point3D pOnC, out Vector3D dt);
        double sqDist2 = pOnC.SquareDistance(pOnS);
        if (sqDist2 <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return;
        }

        Vector3D n = du.Crossed(dv);
        if (n.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return;
        }

        Vector3D pcPs = new(pOnC, pOnS);

        double angMin = M_PI_2 - M_PI_2 / 10.0;
        double angMax = M_PI_2 + M_PI_2 / 10.0;

        double angN = pcPs.Angle(n);
        if (angN >= angMin && angN <= angMax)
        {
            // PcPs is perpendicular to surface normal, it means that
            // aPOnC can be on surface, but far from aPOnS
            isBadSol = true;
            int iu, iv;
            for (iu = -1; iu <= 1; ++iu)
            {
                double u = uv[1] + iu * stepSU;
                u = Max(uVinf[1], u);
                u = Min(uVsup[1], u);
                for (iv = -1; iv <= 1; ++iv)
                {
                    double v = uv[2] + iv * stepSV;
                    v = Max(uVinf[2], v);
                    v = Min(uVsup[2], v);
                    _s.D1(u, v, out pOnS, out du, out dv);
                    if (pOnC.SquareDistance(pOnS) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        isBadSol = false;
                        break;
                    }
                    n = du.Crossed(dv);
                    if (n.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        isBadSol = false;
                        break;
                    }
                    pcPs = pcPs.WithXYZ(pOnS.XYZ() - pOnC.XYZ());
                    angN = pcPs.Angle(n);
                    if (angN < angMin || angN > angMax)
                    {
                        isBadSol = false;
                        break;
                    }
                }
                if (!isBadSol)
                {
                    break;
                }
            }
        }

        if (isBadSol)
        {
            // Try to precise solution with help of Extrema PS

            math_Vector f = new(1, 3);
            f[1] = pcPs.Dot(dt);
            f[2] = pcPs.Dot(du);
            f[3] = pcPs.Dot(dv);
            double ff = f.Norm2();

            GenLocateExtPS extPS = new(_s, _mytol2, _mytol2);
            extPS.Perform(pOnC, uv[1], uv[2], false);
            if (extPS.IsDone)
            {
                POnSurf pmin = extPS.Point();
                pmin.Parameter(out uv[1], out uv[2]);
                math_Vector aTUV = new(tuv);
                aTUV[2] = uv[1];
                aTUV[3] = uv[2];
                _f.Value(aTUV, f);
                double ff1 = f.Norm2();

                if (extPS.SquareDistance() < sqDist2 && ff1 <= 1.1 * ff)
                {
                    tuv[2] = aTUV[2];
                    tuv[3] = aTUV[3];
                }
            }
        }
    }

    private void GlobMinCQuadric(CurveAdaptorBase c, int nbParticles, math_Vector tuVinf,
        math_Vector tuVsup, math_Vector tuv)
    {
        int nbVar = 1;
        math_Vector tinf = new(1, nbVar), tsup = new(1, nbVar), t = new(1, nbVar);
        tinf[1] = tuVinf[1];
        tsup[1] = tuVsup[1];
        //
        PsoParticlesPool particles = new(nbParticles, nbVar);

        math_Vector minT = new(1, nbVar);
        minT.Initialized(tinf + (tsup - tinf) / ABorderDivisor);

        math_Vector maxT = new(1, nbVar);
        maxT.Initialized(tsup - (tsup - tinf) / ABorderDivisor);
        //

        // Increase numbers of curve samples to improve searching global minimum
        // because dimension of optimisation task is reduced
        const int maxNbNodes = 50;
        int newCsample = _mytsample;
        int addsample = Max(_myusample / 2, 3);
        newCsample += addsample;
        newCsample = Min(newCsample, maxNbNodes);
        //
        // Correct number of curve samples in case of low resolution
        double stepCT = (maxT[1] - minT[1]) / newCsample;
        double stepSU = (tuVsup[2] - tuVinf[2]) / _myusample;
        double stepSV = (tuVsup[3] - tuVinf[3]) / _myvsample;
        double scaleFactor = 5.0;
        double resolutionCU = stepCT / c.Resolution(1.0);

        double minResolution = scaleFactor
                                * Min(resolutionCU,
                                    Min(stepSU / _s!.UResolution(1.0), stepSV / _s.VResolution(1.0)));

        if (minResolution > Epsilon(1.0))
        {
            if (resolutionCU > minResolution)
            {
                newCsample = Min(maxNbNodes, RealToInt(newCsample * resolutionCU / minResolution));

                stepCT = (maxT[1] - minT[1]) / newCsample;
            }
        }

        //
        GlobOptFuncCQuadric func = new(c, tinf[1], tsup[1]);
        func.LoadQuad(_s, tuVinf[2], tuVsup[2], tuVinf[3], tuVsup[3]);

        Particle particle = particles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double ct = minT[1];
        for (int cui = 0; cui <= newCsample; cui++, ct += stepCT)
        {
            t[1] = ct;
            if (!func.Value(t, out double sqDist))
            {
                sqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            }

            if (sqDist < particle.Distance)
            {
                particle.Position[0] = ct;

                particle.BestPosition[0] = ct;

                particle.Distance = sqDist;
                particle.BestDistance = sqDist;

                particle = particles.GetWorstParticle();
            }
        }
        //
        math_Vector step = new(1, nbVar);
        step[1] = stepCT;

        // Find min approximation
        Pso pso = new(func, tinf, tsup, step);
        pso.Perform(particles, nbParticles, out _, t);
        //
        math_Vector uv = new(1, 2);
        func.QuadricParameters(t, uv);
        if (_s.IsUPeriodic)
        {
            if (uv[1] < tuVinf[2] - global::Kerf.Engine.Foundation.Precision.PConfusion() || uv[1] > tuVsup[2] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                uv[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uv[1], tuVinf[2], tuVinf[2] + 2.0 * M_PI);
            }
        }
        //
        if (_s.IsVPeriodic)
        {
            if (uv[2] < tuVinf[3] - global::Kerf.Engine.Foundation.Precision.PConfusion() || uv[2] > tuVsup[3] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                uv[2] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uv[2], tuVinf[3], tuVinf[3] + 2.0 * M_PI);
            }
        }
        //
        tuv[1] = t[1];
        tuv[2] = uv[1];
        tuv[3] = uv[2];
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extrema.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _f.ExtCount;
        }
    }

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _f.SquareDistance(n);
    }

    /// <summary>The point on the curve of the Nth extremum.</summary>
    public POnCurv PointOnCurve(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _f.PointOnCurve(n);
    }

    /// <summary>The point on the surface of the Nth extremum.</summary>
    public POnSurf PointOnSurface(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _f.PointOnSurface(n);
    }
}
