// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between two surfaces in the general case: the closest and the farthest pairs of
/// sample points of NbU x NbV grids on both surfaces are refined, the minimum by math_BFGS on the square distance
/// (math_FunctionSetRoot if BFGS fails), the maximum by math_FunctionSetRoot on <see cref="FuncExtSS"/>.
/// </summary>
/// <remarks>
/// The surfaces are kept by reference, as OCCT keeps them: they must outlive the algorithm. NbU and NbV must be at
/// least 2.
/// </remarks>
public class GenExtSS
{
    // This class represents distance objective function for surface / surface.
    private sealed class Extrema_FuncDistSS : MultipleVarFunctionWithGradient
    {
        private readonly SurfaceAdaptorBase _s1;
        private readonly SurfaceAdaptorBase _s2;

        public Extrema_FuncDistSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2)
        {
            _s1 = s1;
            _s2 = s2;
        }

        public override int VariableCount => 4;

        public override bool Value(math_Vector x, out double f)
        {
            f = _s1.Value(x[1], x[2]).SquareDistanceTo(_s2.Value(x[3], x[4]));
            return true;
        }

        public override bool Gradient(math_Vector x, math_Vector g)
        {
            _s1.D1(x[1], x[2], out Point3D p1, out Vector3D du1s1, out Vector3D dv1s1);
            _s2.D1(x[3], x[4], out Point3D p2, out Vector3D du2s2, out Vector3D dv2s2);

            Vector3D p1P2 = new(p2, p1);

            g[1] = p1P2.Dot(du1s1);
            g[2] = p1P2.Dot(dv1s1);
            g[3] = -p1P2.Dot(du2s2);
            g[4] = -p1P2.Dot(dv2s2);

            return true;
        }

        public override bool Values(math_Vector x, out double f, math_Vector g)
        {
            f = _s1.Value(x[1], x[2]).SquareDistanceTo(_s2.Value(x[3], x[4]));

            _s1.D1(x[1], x[2], out Point3D p1, out Vector3D du1s1, out Vector3D dv1s1);
            _s2.D1(x[3], x[4], out Point3D p2, out Vector3D du2s2, out Vector3D dv2s2);

            Vector3D p1P2 = new(p2, p1);

            g[1] = p1P2.Dot(du1s1);
            g[2] = p1P2.Dot(dv1s1);
            g[3] = -p1P2.Dot(du2s2);
            g[4] = -p1P2.Dot(dv2s2);

            return true;
        }
    }

    private bool _done;
#pragma warning disable CS0414 // set and never read in OCCT; kept (decision 221)
    private bool _init;
#pragma warning restore CS0414
    private double _myu1min;
    private double _myu1sup;
    private double _myv1min;
    private double _myv1sup;
    private double _myu2min;
    private double _myu2sup;
    private double _myv2min;
    private double _myv2sup;
    private int _myusample;
    private int _myvsample;
    private NCollection_HArray2<Point3D>? _mypoints1;
    private NCollection_HArray2<Point3D>? _mypoints2;
    private double _mytol1;
    private double _mytol2;
    private readonly FuncExtSS _f;
    private SurfaceAdaptorBase? _s2;

    /// <summary>An empty algorithm.</summary>
    public GenExtSS()
    {
        _f = new FuncExtSS();
        _myu1min = 0.0;
        _myu1sup = 0.0;
        _myv1min = 0.0;
        _myv1sup = 0.0;
        _myu2min = 0.0;
        _myu2sup = 0.0;
        _myv2min = 0.0;
        _myv2sup = 0.0;
        _myusample = 0;
        _myvsample = 0;
        _mytol1 = 0.0;
        _mytol2 = 0.0;
        _s2 = null;
        _done = false;
        _init = false;
    }

    /// <summary>
    /// Computes the extrema between S1 and S2 over their whole ranges. NbU and NbV sample both surfaces to find the
    /// starting points.
    /// </summary>
    public GenExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, int nbU, int nbV, double tol1, double tol2)
    {
        _f = new FuncExtSS(s1, s2);
        Initialize(s2, nbU, nbV, tol2);
        Perform(s1, tol1);
    }

    /// <summary>Computes the extrema between S1 and S2 over the given ranges.</summary>
    public GenExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, int nbU, int nbV, double u1min, double u1sup,
        double v1min, double v1sup, double u2min, double u2sup, double v2min, double v2sup, double tol1, double tol2)
    {
        _f = new FuncExtSS(s1, s2);
        Initialize(s2, nbU, nbV, u2min, u2sup, v2min, v2sup, tol2);
        Perform(s1, u1min, u1sup, v1min, v1sup, tol1);
    }

    /// <summary>Sets S2 over its whole range and samples it.</summary>
    public void Initialize(SurfaceAdaptorBase s2, int nbU, int nbV, double tol2)
    {
        _myu2min = s2.FirstUParameter();
        _myu2sup = s2.LastUParameter();
        _myv2min = s2.FirstVParameter();
        _myv2sup = s2.LastVParameter();
        Initialize(s2, nbU, nbV, _myu2min, _myu2sup, _myv2min, _myv2sup, tol2);
    }

    /// <summary>Sets S2 over [U2min, U2sup] x [V2min, V2sup] and samples it.</summary>
    public void Initialize(SurfaceAdaptorBase s2, int nbU, int nbV, double u2min, double u2sup, double v2min,
        double v2sup, double tol2)
    {
        _s2 = s2;
        _mypoints1 = new NCollection_HArray2<Point3D>(0, nbU + 1, 0, nbV + 1);
        _mypoints2 = new NCollection_HArray2<Point3D>(0, nbU + 1, 0, nbV + 1);
        _myusample = nbU;
        _myvsample = nbV;
        _myu2min = u2min;
        _myu2sup = u2sup;
        _myv2min = v2min;
        _myv2sup = v2sup;
        _mytol2 = tol2;

        // Parametrage de l echantillon sur S2

        double pasU = _myu2sup - _myu2min;
        double pasV = _myv2sup - _myv2min;
        double u0 = pasU / _myusample / 100.0;
        double v0 = pasV / _myvsample / 100.0;
        pasU = (pasU - u0) / (_myusample - 1);
        pasV = (pasV - v0) / (_myvsample - 1);
        u0 = _myu2min + u0 / 2.0;
        v0 = _myv2min + v0 / 2.0;

        // Build UV parameter arrays for batch evaluation
        NCollection_Array1<double> uParams = new(1, _myusample);
        NCollection_Array1<double> vParams = new(1, _myvsample);

        double u = u0;
        for (int noU = 1; noU <= _myusample; noU++, u += pasU)
        {
            uParams.SetValue(noU, u);
        }
        double v = v0;
        for (int noV = 1; noV <= _myvsample; noV++, v += pasV)
        {
            vParams.SetValue(noV, v);
        }

        // Use batch grid evaluation for optimized surface point computation
        SurfaceGrid evaluator = new(_s2);

        NCollection_Array2<Point3D> grid = evaluator.EvaluateGrid(uParams, vParams);

        for (int noU = 1; noU <= _myusample; noU++)
        {
            for (int noV = 1; noV <= _myvsample; noV++)
            {
                _mypoints2.SetValue(noU, noV, grid.Value(noU, noV));
            }
        }
    }

    /// <summary>Computes the extrema with S1 over its whole range (S2 must have been initialised).</summary>
    public void Perform(SurfaceAdaptorBase s1, double tol1)
    {
        _myu1min = s1.FirstUParameter();
        _myu1sup = s1.LastUParameter();
        _myv1min = s1.FirstVParameter();
        _myv1sup = s1.LastVParameter();
        Perform(s1, _myu1min, _myu1sup, _myv1min, _myv1sup, tol1);
    }

    /// <summary>Computes the extrema with S1 over [U1min, U1sup] x [V1min, V1sup] (S2 must have been initialised).</summary>
    public void Perform(SurfaceAdaptorBase s1, double u1min, double u1sup, double v1min, double v1sup, double tol1)
    {
        if (_s2 is null)
        {
            throw new NotDoneException("Extrema_GenExtSS: the second surface is not initialized");
        }
        _f.Initialize(s1, _s2);
        _myu1min = u1min;
        _myu1sup = u1sup;
        _myv1min = v1min;
        _myv1sup = v1sup;
        _mytol1 = tol1;

        double u1, v1, u2, v2;
        int noU1, noV1, noU2, noV2;
        Point3D p1, p2;

        // Parametrage de l echantillon sur S1

        double pasU1 = _myu1sup - _myu1min;
        double pasV1 = _myv1sup - _myv1min;
        double u10 = pasU1 / _myusample / 100.0;
        double v10 = pasV1 / _myvsample / 100.0;
        pasU1 = (pasU1 - u10) / (_myusample - 1);
        pasV1 = (pasV1 - v10) / (_myvsample - 1);
        u10 = _myu1min + u10 / 2.0;
        v10 = _myv1min + v10 / 2.0;

        double pasU2 = _myu2sup - _myu2min;
        double pasV2 = _myv2sup - _myv2min;
        double u20 = pasU2 / _myusample / 100.0;
        double v20 = pasV2 / _myvsample / 100.0;
        pasU2 = (pasU2 - u20) / (_myusample - 1);
        pasV2 = (pasV2 - v20) / (_myvsample - 1);
        u20 = _myu2min + u20 / 2.0;
        v20 = _myv2min + v20 / 2.0;

        // Build UV parameter arrays for batch evaluation of S1
        NCollection_Array1<double> u1Params = new(1, _myusample);
        NCollection_Array1<double> v1Params = new(1, _myvsample);

        u1 = u10;
        for (noU1 = 1; noU1 <= _myusample; noU1++, u1 += pasU1)
        {
            u1Params.SetValue(noU1, u1);
        }
        v1 = v10;
        for (noV1 = 1; noV1 <= _myvsample; noV1++, v1 += pasV1)
        {
            v1Params.SetValue(noV1, v1);
        }

        // Use batch grid evaluation for optimized surface point computation
        SurfaceGrid evaluator = new(s1);

        NCollection_Array2<Point3D> grid = evaluator.EvaluateGrid(u1Params, v1Params);

        for (noU1 = 1; noU1 <= _myusample; noU1++)
        {
            for (noV1 = 1; noV1 <= _myvsample; noV1++)
            {
                _mypoints1!.SetValue(noU1, noV1, grid.Value(noU1, noV1));
            }
        }

        /*
        b- Calcul des minima:
           -----------------
           b.a) Initialisations:
        */

        math_Vector tol = new(1, 4);
        tol[1] = _mytol1;
        tol[2] = _mytol1;
        tol[3] = _mytol2;
        tol[4] = _mytol2;
        math_Vector uv = new(1, 4), uVinf = new(1, 4), uVsup = new(1, 4);
        uVinf[1] = _myu1min;
        uVinf[2] = _myv1min;
        uVinf[3] = _myu2min;
        uVinf[4] = _myv2min;
        uVsup[1] = _myu1sup;
        uVsup[2] = _myv1sup;
        uVsup[3] = _myu2sup;
        uVsup[4] = _myv2sup;

        double distmin = RealLast(), distmax = 0.0, theDist;

        int n1Umin = 0, n1Vmin = 0, n2Umin = 0, n2Vmin = 0;
        int n1Umax = 0, n1Vmax = 0, n2Umax = 0, n2Vmax = 0;
        // OCCT also keeps the points of both pairs (PP1min, PP2min, PP1max, PP2max), never read; left out

        for (noU1 = 1, u1 = u10; noU1 <= _myusample; noU1++, u1 += pasU1)
        {
            for (noV1 = 1, v1 = v10; noV1 <= _myvsample; noV1++, v1 += pasV1)
            {
                p1 = _mypoints1!.Value(noU1, noV1);
                for (noU2 = 1, u2 = u20; noU2 <= _myusample; noU2++, u2 += pasU2)
                {
                    for (noV2 = 1, v2 = v20; noV2 <= _myvsample; noV2++, v2 += pasV2)
                    {
                        p2 = _mypoints2!.Value(noU2, noV2);
                        theDist = p1.SquareDistanceTo(p2);
                        if (theDist < distmin)
                        {
                            distmin = theDist;
                            n1Umin = noU1;
                            n1Vmin = noV1;
                            n2Umin = noU2;
                            n2Vmin = noV2;
                        }
                        if (theDist > distmax)
                        {
                            distmax = theDist;
                            n1Umax = noU1;
                            n1Vmax = noV1;
                            n2Umax = noU2;
                            n2Vmax = noV2;
                        }
                    }
                }
            }
        }

        uv[1] = u10 + (n1Umin - 1) * pasU1;
        uv[2] = v10 + (n1Vmin - 1) * pasV1;
        uv[3] = u20 + (n2Umin - 1) * pasU2;
        uv[4] = v20 + (n2Vmin - 1) * pasV2;

        Extrema_FuncDistSS gfss = new(s1, _s2);
        Bfgs bfgsSolver = new(4);
        bfgsSolver.Perform(gfss, uv);
        if (bfgsSolver.IsDone)
        {
            bfgsSolver.Location(uv);

            //  Store result in myF (OCCT passes UV as both the variables and the values).
            _f.Value(uv, uv);
            _f.GetStateNumber();
        }
        else
        {
            // If optimum is not computed successfully then compute by old approach.

            // Restore initial point.
            uv[1] = u10 + (n1Umin - 1) * pasU1;
            uv[2] = v10 + (n1Vmin - 1) * pasV1;
            uv[3] = u20 + (n2Umin - 1) * pasU2;
            uv[4] = v20 + (n2Vmin - 1) * pasV2;

            FunctionSetRoot sr1 = new(_f, tol);
            sr1.Perform(_f, uv, uVinf, uVsup);
        }

        uv[1] = u10 + (n1Umax - 1) * pasU1;
        uv[2] = v10 + (n1Vmax - 1) * pasV1;
        uv[3] = u20 + (n2Umax - 1) * pasU2;
        uv[4] = v20 + (n2Vmax - 1) * pasV2;

        // It is impossible to compute max distance in the same manner,
        // since for the distance functional for max have bad definition.
        // So, for max computation old approach is used.
        FunctionSetRoot sr2 = new(_f, tol);
        sr2.Perform(_f, uv, uVinf, uVsup);

        _done = true;
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

    /// <summary>The point on S1 of the Nth extremum.</summary>
    public POnSurf PointOnS1(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _f.PointOnS1(n);
    }

    /// <summary>The point on S2 of the Nth extremum.</summary>
    public POnSurf PointOnS2(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _f.PointOnS2(n);
    }
}
