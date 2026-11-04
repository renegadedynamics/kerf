// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of a general chamfer between two surfaces along a guide curve: linear sections.</summary>
internal abstract class GenChamfer : BlendFunction
{
    protected readonly SurfaceAdaptorBase _surf1;
    protected readonly SurfaceAdaptorBase _surf2;
    protected readonly CurveAdaptorBase _curv;
    protected int _choix;
    protected double _tol;
    protected double _distmin;

    protected GenChamfer(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase cg)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = cg;
        _choix = 0;
        _tol = 0.0;
        _distmin = RealLast();
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    /// <summary>Value then Derivatives.</summary>
    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        bool val = Value(x, f);
        return (val && Derivatives(x, d));
    }

    public abstract override void Set(double param);

    /// <summary>Does nothing.</summary>
    public override void Set(double first, double last)
    {
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _surf1.UResolution(tol);
        tolerance[2] = _surf1.VResolution(tol);
        tolerance[3] = _surf2.UResolution(tol);
        tolerance[4] = _surf2.VResolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _surf1.FirstUParameter();
        infBound[2] = _surf1.FirstVParameter();
        infBound[3] = _surf2.FirstUParameter();
        infBound[4] = _surf2.FirstVParameter();
        supBound[1] = _surf1.LastUParameter();
        supBound[2] = _surf1.LastVParameter();
        supBound[3] = _surf2.LastUParameter();
        supBound[4] = _surf2.LastVParameter();

        for (int i = 1; i <= 4; i++)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[i]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[i]))
            {
                double range = (supBound[i] - infBound[i]);
                infBound[i] -= range;
                supBound[i] += range;
            }
        }
    }

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    /// <summary>Sets the distances and the "quadrant".</summary>
    public abstract void Set(double dist1, double dist2, int choix);

    /// <summary>Returns false.</summary>
    public override bool IsRational
    {
        get
        {
            return false;
        }
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        weights.Init(1);
    }

    public override int NbIntervals(Continuity s)
    {
        return _curv.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _curv.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
    }

    public override void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        nbPoles = 2;
        nbPoles2d = 2;
        nbKnots = 2;
        degree = 1;
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerance a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1D)
    {
        tol3d.Init(boundTol);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        tKnots[1] = 0.0;
        tKnots[2] = 1.0;
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        tMults[1] = 2;
        tMults[2] = 2;
    }

    /// <summary>Obsolete: the section at (U1, V1), (U2, V2) as a line with its parameter range.</summary>
    public void Section(double param, double u1, double v1, double u2, double v2, out double pdeb, out double pfin,
                        ref Line3D c)
    {
        Point3D pts1 = _surf1.Value(u1, v1);
        Point3D pts2 = _surf2.Value(u2, v2);
        Direction3D dir = new Direction3D(new Vector3D(pts1, pts2));

        c = c.WithLocation(pts1);
        c = c.WithDirection(dir);

        pdeb = 0.0;
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, pts2);
    }

    /// <summary>Not computed: returns false.</summary>
    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights,
                                 NCollection_Array1<double> d2Weights)
    {
        return false;
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        math_Vector sol = new(1, 4), valsol = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        p.ParametersOnS1(out sol[1], out sol[2]);
        p.ParametersOnS2(out sol[3], out sol[4]);

        Set(prm);

        Values(sol, valsol, gradsol);
        IsSolution(sol, _tol);

        istgt = IsTangencyPoint;

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(sol[3], sol[4]);
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(Tangent2dOnS1().X, Tangent2dOnS1().Y);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(Tangent2dOnS2().X, Tangent2dOnS2().Y);
        }

        poles[low] = PointOnS1();
        poles[upp] = PointOnS2();
        weights[low] = 1.0;
        weights[upp] = 1.0;
        if (!istgt)
        {
            dPoles[low] = TangentOnS1();
            dPoles[upp] = TangentOnS2();
            dWeights[low] = 0.0;
            dWeights[upp] = 0.0;
        }

        return (!istgt);
    }

    public override void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weights)
    {
        double u1, v1, u2, v2, prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        math_Vector x = new(1, 4), f = new(1, 4);

        p.ParametersOnS1(out u1, out v1);
        p.ParametersOnS2(out u2, out v2);
        x[1] = u1;
        x[2] = v1;
        x[3] = u2;
        x[4] = v2;
        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(u1, v1);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(u2, v2);

        Set(prm);
        Value(x, f);
        poles[low] = PointOnS1();
        poles[upp] = PointOnS2();
        weights[low] = 1.0;
        weights[upp] = 1.0;
    }

    public override void Resolution(int ic2d, double tol, out double tolU, out double tolV)
    {
        if (ic2d == 1)
        {
            tolU = _surf1.UResolution(tol);
            tolV = _surf1.VResolution(tol);
        }
        else
        {
            tolU = _surf2.UResolution(tol);
            tolV = _surf2.VResolution(tol);
        }
    }
}
