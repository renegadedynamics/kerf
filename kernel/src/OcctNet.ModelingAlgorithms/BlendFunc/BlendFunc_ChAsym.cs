// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsym.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsym.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function of an asymmetric chamfer between two surfaces along a guide curve: a distance on the first surface
/// and an angle.
/// </summary>
public class BlendFunc_ChAsym : Blend_Function
{
    private readonly SurfaceAdaptorBase surf1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly CurveAdaptorBase curv;
    private CurveAdaptorBase tcurv;
    private double param;
    private double dist1;
    private double angle;
    private double tgang;
    private Vector3D nplan;
    private Point3D pt1;
    private Vector3D tsurf1;
    private Point3D pt2;
    private readonly math_Vector FX;
    private readonly Matrix DX;
    private bool istangent;
    private Vector3D tg1;
    private Vector2D tg12d;
    private Vector3D tg2;
    private Vector2D tg22d;
    private int choix;
    private double distmin;

    public BlendFunc_ChAsym(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, CurveAdaptorBase C)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        tcurv = C;
        param = 0;
        dist1 = RealLast();
        angle = RealLast();
        tgang = RealLast();
        FX = new math_Vector(1, 4);
        DX = new Matrix(1, 4, 1, 4);
        istangent = true;
        choix = 0;
        distmin = RealLast();
        _ = angle;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(double Param)
    {
        param = Param;
    }

    //=======================================================================
    // function : Set
    // purpose  : Segmente la courbe a sa partie utile.
    //           La precision est prise arbitrairement petite !?
    //=======================================================================
    public override void Set(double First, double Last)
    {
        tcurv = curv.Trim(First, Last, 1.0e-12);
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

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        Vector3D np, dnp, Nsurf1, dwtsurf1;
        double Normg;

        tcurv.D2(param, out Point3D ptgui, out Vector3D d1gui, out Vector3D d2gui);
        Normg = d1gui.Magnitude();
        np = d1gui.Normalized();
        dnp = (d2gui - np.Dot(d2gui) * np) / Normg;

        if (choix % 2 != 0)
        {
            np.Reverse();
            dnp.Reverse();
            Normg = -Normg;
        }

        surf1.D1(Sol[1], Sol[2], out pt1, out Vector3D d1u1, out Vector3D d1v1);
        Nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = Nsurf1.Crossed(np);
        dwtsurf1 = Nsurf1.Crossed(dnp);

        surf2.D1(Sol[3], Sol[4], out pt2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D pguis1 = new Vector3D(ptgui, pt1), pguis2 = new Vector3D(ptgui, pt2);
        Vector3D CrossVec, s1s2 = new Vector3D(pt1, pt2);
        double PScaInv = 1.0 / tsurf1.Dot(s1s2), F4, temp;
        double maxpiv = 1.0e-9;
        double Nordu1 = d1u1.Magnitude(), Nordv1 = d1v1.Magnitude();

        temp = 2.0 * (Nordu1 + Nordv1) * s1s2.Magnitude() + 2.0 * Nordu1 * Nordv1;

        Values(Sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) < Tol && Math.Abs(valsol[2]) < Tol
            && Math.Abs(valsol[3]) < 2.0 * dist1 * Tol
            && Math.Abs(valsol[4]) < Tol * (1.0 + tgang) * Math.Abs(PScaInv) * temp)
        {
            secmember[1] = Normg - dnp.Dot(pguis1);
            secmember[2] = Normg - dnp.Dot(pguis2);
            secmember[3] = -2.0 * d1gui.Dot(pguis1);

            CrossVec = tsurf1.Crossed(s1s2);
            F4 = np.Dot(CrossVec) * PScaInv;
            temp = dnp.Dot(CrossVec) + np.Dot(dwtsurf1.Crossed(s1s2));

            temp -= F4 * dwtsurf1.Dot(s1s2);
            secmember[4] = PScaInv * temp;

            Gauss Resol = new(gradsol, maxpiv);

            if (Resol.IsDone)
            {
                Resol.Solve(secmember);
                istangent = false;
            }
            else
            {
                Svd SingRS = new(gradsol);
                if (SingRS.IsDone)
                {
                    math_Vector DEDT = new(1, 4);
                    DEDT.Initialized(secmember);
                    SingRS.Solve(DEDT, secmember, 1.0e-6);
                    istangent = false;
                }
                else
                {
                    istangent = true;
                }
            }

            if (!istangent)
            {
                tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);
                tg12d.SetCoord(secmember[1], secmember[2]);
                tg22d.SetCoord(secmember[3], secmember[4]);
            }

            distmin = Min(distmin, pt1.Distance(pt2));

            return true;
        }

        istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return distmin;
    }

    /// <summary>The values (from <paramref name="DegF"/> 0) and the derivatives (to <paramref name="DegL"/> 1).</summary>
    public bool ComputeValues(math_Vector X, int DegF, int DegL)
    {
        if (DegF > DegL)
        {
            return false;
        }

        Vector3D np, d1u1, d1v1, d2u1 = new Vector3D(), d2v1 = new Vector3D(), d2uv1 = new Vector3D(), d1u2 = new Vector3D(),
               d1v2 = new Vector3D(), Nsurf1;
        double PScaInv, F4;

        tcurv.D1(param, out Point3D ptgui, out Vector3D d1gui);
        nplan = d1gui.Normalized();
        np = nplan;

        if (choix % 2 != 0)
        {
            np.Reverse();
        }

        if ((DegF == 0) && (DegL == 0))
        {
            surf1.D1(X[1], X[2], out pt1, out d1u1, out d1v1);
            pt2 = surf2.Value(X[3], X[4]);
        }
        else
        {
            surf1.D2(X[1], X[2], out pt1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D1(X[3], X[4], out pt2, out d1u2, out d1v2);
        }

        Nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = Nsurf1.Crossed(np);

        Vector3D nps1 = new Vector3D(ptgui, pt1), s1s2 = new Vector3D(pt1, pt2);
        PScaInv = 1.0 / tsurf1.Dot(s1s2);
        F4 = np.Dot(tsurf1.Crossed(s1s2)) * PScaInv;

        if (DegF == 0)
        {
            double Dist;
            Dist = ptgui.XYZ().Dot(np.XYZ());

            FX[1] = pt1.XYZ().Dot(np.XYZ()) - Dist;
            FX[2] = pt2.XYZ().Dot(np.XYZ()) - Dist;
            FX[3] = dist1 * dist1 - nps1.SquareMagnitude();
            FX[4] = tgang - F4;
        }

        if (DegL == 1)
        {
            double temp;
            Vector3D tempVec;
            Vector3D d1utsurf1, d1vtsurf1;
            d1utsurf1 = (d2u1.Crossed(d1v1) + d1u1.Crossed(d2uv1)).Crossed(np);
            d1vtsurf1 = (d2uv1.Crossed(d1v1) + d1u1.Crossed(d2v1)).Crossed(np);

            DX[1, 1] = np.Dot(d1u1);
            DX[1, 2] = np.Dot(d1v1);
            DX[1, 3] = 0.0;
            DX[1, 4] = 0.0;

            DX[2, 1] = 0.0;
            DX[2, 2] = 0.0;
            DX[2, 3] = np.Dot(d1u2);
            DX[2, 4] = np.Dot(d1v2);

            tempVec = -2.0 * nps1;
            DX[3, 1] = d1u1.Dot(tempVec);
            DX[3, 2] = d1v1.Dot(tempVec);
            DX[3, 3] = 0.0;
            DX[3, 4] = 0.0;

            temp = F4 * (d1utsurf1.Dot(s1s2) - tsurf1.Dot(d1u1));
            temp += np.Dot(tsurf1.Crossed(d1u1) - d1utsurf1.Crossed(s1s2));
            DX[4, 1] = temp * PScaInv;

            temp = F4 * (d1vtsurf1.Dot(s1s2) - tsurf1.Dot(d1v1));
            temp += np.Dot(tsurf1.Crossed(d1v1) - d1vtsurf1.Crossed(s1s2));
            DX[4, 2] = temp * PScaInv;

            temp = F4 * tsurf1.Dot(d1u2) - np.Dot(tsurf1.Crossed(d1u2));
            DX[4, 3] = temp * PScaInv;

            temp = F4 * tsurf1.Dot(d1v2) - np.Dot(tsurf1.Crossed(d1v2));
            DX[4, 4] = temp * PScaInv;
        }

        return true;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        bool Error = ComputeValues(X, 0, 0);
        F.Initialized(FX);
        return Error;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        bool Error = ComputeValues(X, 1, 1);
        D.Initialized(DX);
        return Error;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        bool Error = ComputeValues(X, 0, 1);
        F.Initialized(FX);
        D.Initialized(DX);
        return Error;
    }

    public override Point3D PointOnS1()
    {
        return pt1;
    }

    public override Point3D PointOnS2()
    {
        return pt2;
    }

    public override bool IsTangencyPoint()
    {
        return istangent;
    }

    public override Vector3D TangentOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TangentOnS1");
        }

        return tg1;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::Tangent2dOnS1");
        }

        return tg12d;
    }

    public override Vector3D TangentOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TangentOnS2");
        }

        return tg2;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::Tangent2dOnS2");
        }

        return tg22d;
    }

    public override bool TwistOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TwistOnS1");
        }

        return tg1.Dot(nplan) < 0.0;
    }

    public override bool TwistOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TwistOnS2");
        }

        return tg2.Dot(nplan) < 0.0;
    }

    //=======================================================================
    // function : Tangent
    // purpose  : TgF,NmF et TgL,NmL les tangentes et normales respectives
    //           aux surfaces S1 et S2
    //=======================================================================
    public override void Tangent(double U1, double V1, double U2, double V2, out Vector3D TgF, out Vector3D TgL,
                                 out Vector3D NmF, out Vector3D NmL)
    {
        Vector3D np;
        bool revF = false;
        bool revL = false;

        tcurv.D1(param, out _, out Vector3D d1gui);
        np = d1gui.Normalized();

        surf1.D1(U1, V1, out _, out Vector3D d1u1, out Vector3D d1v1);
        NmF = d1u1.Crossed(d1v1);

        surf2.D1(U2, V2, out _, out Vector3D d1u2, out Vector3D d1v2);
        NmL = d1u2.Crossed(d1v2);

        TgF = (np.Crossed(NmF)).Normalized();
        TgL = (np.Crossed(NmL)).Normalized();

        if ((choix == 2) || (choix == 5))
        {
            revF = true;
            revL = true;
        }

        if ((choix == 4) || (choix == 7))
        {
            revL = true;
        }

        if ((choix == 3) || (choix == 8))
        {
            revF = true;
        }

        if (revF)
        {
            TgF.Reverse();
        }

        if (revL)
        {
            TgL.Reverse();
        }
    }

    /// <summary>The section at (U1, V1), (U2, V2) as a line with its parameter range.</summary>
    public void Section(double Param, double U1, double V1, double U2, double V2, out double Pdeb, out double Pfin,
                        ref Line3D C)
    {
        Point3D Pt1 = surf1.Value(U1, V1);
        Point3D Pt2 = surf2.Value(U2, V2);
        Direction3D dir = new Direction3D(new Vector3D(Pt1, Pt2));

        C.SetLocation(Pt1);
        C.SetDirection(dir);

        Pdeb = 0.0;
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, Pt2);
    }

    /// <summary>Returns false.</summary>
    public override bool IsRational()
    {
        return false;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_ChAsym::GetSectionSize()");
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
    // purpose  : Determine les Tolerances a utiliser dans les approximations.
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

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        math_Vector Sol = new(1, 4), valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);
        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();

        P.ParametersOnS1(out Sol[1], out Sol[2]);
        P.ParametersOnS2(out Sol[3], out Sol[4]);
        Set(prm);

        Poles2d[Poles2d.Lower()].SetCoord(Sol[1], Sol[2]);
        Poles2d[Poles2d.Upper()].SetCoord(Sol[3], Sol[4]);

        // OCCT: the poles are the points of the previous computation, set before the D1s below (decision 1040).
        Poles[low] = PointOnS1();
        Poles[upp] = PointOnS2();
        Weights[low] = 1.0;
        Weights[upp] = 1.0;

        Vector3D np, dnp, Nsurf1, dwtsurf1;
        double Normg;

        tcurv.D2(param, out Point3D ptgui, out Vector3D d1gui, out Vector3D d2gui);
        Normg = d1gui.Magnitude();
        np = d1gui.Normalized();
        dnp = (d2gui - np.Dot(d2gui) * np) / Normg;

        if (choix % 2 != 0)
        {
            np.Reverse();
            dnp.Reverse();
            Normg = -Normg;
        }

        surf1.D1(Sol[1], Sol[2], out pt1, out Vector3D d1u1, out Vector3D d1v1);
        Nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = Nsurf1.Crossed(np);
        dwtsurf1 = Nsurf1.Crossed(dnp);

        surf2.D1(Sol[3], Sol[4], out pt2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D pguis1 = new Vector3D(ptgui, pt1), pguis2 = new Vector3D(ptgui, pt2);
        Vector3D CrossVec, s1s2 = new Vector3D(pt1, pt2);
        double PScaInv = 1.0 / tsurf1.Dot(s1s2), F4, temp;
        double maxpiv = 1.0e-9;
        double Nordu1 = d1u1.Magnitude(), Nordv1 = d1v1.Magnitude();

        temp = 2.0 * (Nordu1 + Nordv1) * s1s2.Magnitude() + 2.0 * Nordu1 * Nordv1;

        Values(Sol, valsol, gradsol);

        secmember[1] = Normg - dnp.Dot(pguis1);
        secmember[2] = Normg - dnp.Dot(pguis2);
        secmember[3] = -2.0 * d1gui.Dot(pguis1);

        CrossVec = tsurf1.Crossed(s1s2);
        F4 = np.Dot(CrossVec) * PScaInv;
        temp = dnp.Dot(CrossVec) + np.Dot(dwtsurf1.Crossed(s1s2));

        temp -= F4 * dwtsurf1.Dot(s1s2);
        secmember[4] = PScaInv * temp;

        Gauss Resol = new(gradsol, maxpiv);

        if (Resol.IsDone)
        {
            Resol.Solve(secmember);
            istangent = false;
        }
        else
        {
            Svd SingRS = new(gradsol);
            if (SingRS.IsDone)
            {
                math_Vector DEDT = new(1, 4);
                DEDT.Initialized(secmember);
                SingRS.Solve(DEDT, secmember, 1.0e-6);
                istangent = false;
            }
            else
            {
                istangent = true;
            }
        }

        if (!istangent)
        {
            tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);
            tg12d.SetCoord(secmember[1], secmember[2]);
            tg22d.SetCoord(secmember[3], secmember[4]);
        }

        distmin = Min(distmin, pt1.Distance(pt2));

        if (!istangent)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(Tangent2dOnS1().X(), Tangent2dOnS1().Y());
            DPoles2d[Poles2d.Upper()].SetCoord(Tangent2dOnS2().X(), Tangent2dOnS2().Y());
            DPoles[low] = TangentOnS1();
            DPoles[upp] = TangentOnS2();
            DWeights[low] = 0.0;
            DWeights[upp] = 0.0;
        }

        return (!istangent);
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

    /// <summary>Sets the distance on the first surface, the angle and the "quadrant".</summary>
    public void Set(double Dist1, double Angle, int Choix)
    {
        dist1 = Math.Abs(Dist1);
        angle = Angle;
        tgang = Math.Tan(Angle);
        choix = Choix;
    }
}
