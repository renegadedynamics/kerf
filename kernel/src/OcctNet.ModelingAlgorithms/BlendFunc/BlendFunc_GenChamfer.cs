// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>The function of a general chamfer between two surfaces along a guide curve: linear sections.</summary>
public abstract class BlendFunc_GenChamfer : Blend_Function
{
    protected readonly Adaptor3d_Surface surf1;
    protected readonly Adaptor3d_Surface surf2;
    protected readonly Adaptor3d_Curve curv;
    protected int choix;
    protected double tol;
    protected double distmin;

    protected BlendFunc_GenChamfer(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve CG)
    {
        surf1 = S1;
        surf2 = S2;
        curv = CG;
        choix = 0;
        tol = 0.0;
        distmin = RealLast();
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    /// <summary>Value then Derivatives.</summary>
    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        bool val = Value(X, F);
        return (val && Derivatives(X, D));
    }

    public abstract override void Set(double Param);

    /// <summary>Does nothing.</summary>
    public override void Set(double First, double Last)
    {
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = surf1.UResolution(Tol);
        Tolerance[2] = surf1.VResolution(Tol);
        Tolerance[3] = surf2.UResolution(Tol);
        Tolerance[4] = surf2.VResolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = surf1.FirstUParameter();
        InfBound[2] = surf1.FirstVParameter();
        InfBound[3] = surf2.FirstUParameter();
        InfBound[4] = surf2.FirstVParameter();
        SupBound[1] = surf1.LastUParameter();
        SupBound[2] = surf1.LastVParameter();
        SupBound[3] = surf2.LastUParameter();
        SupBound[4] = surf2.LastVParameter();

        for (int i = 1; i <= 4; i++)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[i]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[i]))
            {
                double range = (SupBound[i] - InfBound[i]);
                InfBound[i] -= range;
                SupBound[i] += range;
            }
        }
    }

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public override double GetMinimalDistance()
    {
        return distmin;
    }

    /// <summary>Sets the distances and the "quadrant".</summary>
    public abstract void Set(double Dist1, double Dist2, int Choix);

    /// <summary>Returns false.</summary>
    public override bool IsRational()
    {
        return false;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weights)
    {
        Weights.Init(1);
    }

    public override int NbIntervals(Continuity S)
    {
        return curv.NbIntervals(BlendFunc.NextShape(S));
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        curv.Intervals(T, BlendFunc.NextShape(S));
    }

    public override void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d)
    {
        NbPoles = 2;
        NbPoles2d = 2;
        NbKnots = 2;
        Degree = 1;
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerance a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1D)
    {
        Tol3d.Init(BoundTol);
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        TKnots[1] = 0.0;
        TKnots[2] = 1.0;
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        TMults[1] = 2;
        TMults[2] = 2;
    }

    /// <summary>Obsolete: the section at (U1, V1), (U2, V2) as a line with its parameter range.</summary>
    public void Section(double Param, double U1, double V1, double U2, double V2, out double Pdeb, out double Pfin,
                        ref Line3D C)
    {
        Point3D pts1 = surf1.Value(U1, V1);
        Point3D pts2 = surf2.Value(U2, V2);
        Direction3D dir = new Direction3D(new Vector3D(pts1, pts2));

        C.SetLocation(pts1);
        C.SetDirection(dir);

        Pdeb = 0.0;
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, pts2);
    }

    /// <summary>Not computed: returns false.</summary>
    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights,
                                 NCollection_Array1<double> D2Weights)
    {
        return false;
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        math_Vector sol = new(1, 4), valsol = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt;

        P.ParametersOnS1(out sol[1], out sol[2]);
        P.ParametersOnS2(out sol[3], out sol[4]);

        Set(prm);

        Values(sol, valsol, gradsol);
        IsSolution(sol, tol);

        istgt = IsTangencyPoint();

        Poles2d[Poles2d.Lower()].SetCoord(sol[1], sol[2]);
        Poles2d[Poles2d.Upper()].SetCoord(sol[3], sol[4]);
        if (!istgt)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(Tangent2dOnS1().X(), Tangent2dOnS1().Y());
            DPoles2d[Poles2d.Upper()].SetCoord(Tangent2dOnS2().X(), Tangent2dOnS2().Y());
        }

        Poles[low] = PointOnS1();
        Poles[upp] = PointOnS2();
        Weights[low] = 1.0;
        Weights[upp] = 1.0;
        if (!istgt)
        {
            DPoles[low] = TangentOnS1();
            DPoles[upp] = TangentOnS2();
            DWeights[low] = 0.0;
            DWeights[upp] = 0.0;
        }

        return (!istgt);
    }

    public override void Section(in Blend_Point P, NCollection_Array1<Point3D> Poles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weights)
    {
        double u1, v1, u2, v2, prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        math_Vector X = new(1, 4), F = new(1, 4);

        P.ParametersOnS1(out u1, out v1);
        P.ParametersOnS2(out u2, out v2);
        X[1] = u1;
        X[2] = v1;
        X[3] = u2;
        X[4] = v2;
        Poles2d[Poles2d.Lower()].SetCoord(u1, v1);
        Poles2d[Poles2d.Upper()].SetCoord(u2, v2);

        Set(prm);
        Value(X, F);
        Poles[low] = PointOnS1();
        Poles[upp] = PointOnS2();
        Weights[low] = 1.0;
        Weights[upp] = 1.0;
    }

    public override void Resolution(int IC2d, double Tol, out double TolU, out double TolV)
    {
        if (IC2d == 1)
        {
            TolU = surf1.UResolution(Tol);
            TolV = surf1.VResolution(Tol);
        }
        else
        {
            TolU = surf2.UResolution(Tol);
            TolV = surf2.VResolution(Tol);
        }
    }
}
