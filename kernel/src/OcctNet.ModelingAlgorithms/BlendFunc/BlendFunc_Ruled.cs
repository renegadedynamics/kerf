// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Ruled.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Ruled.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function of a ruled blend between two surfaces along a guide curve: the segment between two points in the
/// plane normal to the guide, normal to both surfaces' sections.
/// </summary>
public class BlendFunc_Ruled : Blend_Function
{
    private readonly Adaptor3d_Surface surf1;
    private readonly Adaptor3d_Surface surf2;
    private readonly Adaptor3d_Curve curv;
    private Point3D pts1;
    private Point3D pts2;
    private bool istangent;
    private Vector3D tg1;
    private Vector2D tg12d;
    private Vector3D tg2;
    private Vector2D tg22d;
    private Point3D ptgui;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D nplan;
    private double normtg;
    private double theD;
    private readonly double distmin;

    public BlendFunc_Ruled(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve C)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        istangent = true;
        normtg = 0.0;
        theD = 0.0;
        distmin = RealLast();
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(double Param)
    {
        curv.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
        istangent = true;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override void Set(double First, double Last)
    {
        throw new MethodNotImplementedException("BlendFunc_Ruled::Set");
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

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ncrossns;
        double norm, ndotns, grosterme;

        Values(Sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= Tol
            && Math.Abs(valsol[4]) <= Tol)
        {
            // Calcul des tangentes

            surf1.D1(Sol[1], Sol[2], out pts1, out Vector3D d1u1, out Vector3D d1v1);
            surf2.D1(Sol[3], Sol[4], out pts2, out Vector3D d1u2, out Vector3D d1v2);

            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            secmember[1] = normtg - dnplan.Dot(new Vector3D(ptgui, pts1));
            secmember[2] = normtg - dnplan.Dot(new Vector3D(ptgui, pts2));

            ns = d1u1.Crossed(d1v1);
            ncrossns = nplan.Crossed(ns);
            ndotns = nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            // Derivee de nor1 par rapport au parametre sur la ligne guide
            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                               grosterme / norm, ns);

            secmember[3] = -(temp.Dot(new Vector3D(pts1, pts2)));

            ns = d1u2.Crossed(d1v2);
            ncrossns = nplan.Crossed(ns);
            ndotns = nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            // Derivee de nor2 par rapport au parametre sur la ligne guide
            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                               grosterme / norm, ns);

            secmember[4] = -(temp.Dot(new Vector3D(pts1, pts2)));

            Gauss Resol = new(gradsol);
            if (Resol.IsDone)
            {
                Resol.Solve(secmember);

                tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);
                tg12d.SetCoord(secmember[1], secmember[2]);
                tg22d.SetCoord(secmember[3], secmember[4]);
                istangent = false;
            }
            else
            {
                istangent = true;
            }

            return true;
        }

        istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return distmin;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        surf1.D1(X[1], X[2], out pts1, out Vector3D d1u1, out Vector3D d1v1);
        surf2.D1(X[3], X[4], out pts2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D temp = new Vector3D(pts1, pts2);

        Vector3D ns1 = d1u1.Crossed(d1v1);
        Vector3D ns2 = d1u2.Crossed(d1v2);

        double norm1 = nplan.Crossed(ns1).Magnitude();
        double norm2 = nplan.Crossed(ns2).Magnitude();
        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        F[1] = (nplan.XYZ().Dot(pts1.XYZ())) + theD;
        F[2] = (nplan.XYZ().Dot(pts2.XYZ())) + theD;

        F[3] = temp.Dot(ns1);
        F[4] = temp.Dot(ns2);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D nor1 = new Vector3D(), nor2 = new Vector3D(), p1p2;
        Vector3D ns1, ns2, ncrossns1, ncrossns2, resul = new Vector3D(), temp;

        double norm1, norm2, ndotns1, ndotns2, grosterme;

        surf1.D2(X[1], X[2], out pts1, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                 out Vector3D d2uv1);
        surf2.D2(X[3], X[4], out pts2, out Vector3D d1u2, out Vector3D d1v2, out Vector3D d2u2, out Vector3D d2v2,
                 out Vector3D d2uv2);

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);
        D[1, 3] = 0.0;
        D[1, 4] = 0.0;
        D[2, 1] = 0.0;
        D[2, 2] = 0.0;
        D[2, 3] = nplan.Dot(d1u2);
        D[2, 4] = nplan.Dot(d1v2);

        ns1 = d1u1.Crossed(d1v1);
        ns2 = d1u2.Crossed(d1v2);
        ncrossns1 = nplan.Crossed(ns1);
        ncrossns2 = nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude();
        norm2 = ncrossns2.Magnitude();

        ndotns1 = nplan.Dot(ns1);
        ndotns2 = nplan.Dot(ns2);

        nor1.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        nor2.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);

        p1p2 = new Vector3D(pts1, pts2);

        // Derivee de nor1 par rapport a u1

        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        D[3, 1] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul);

        // Derivee par rapport a v1

        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        D[3, 2] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul);

        D[3, 3] = d1u2.Dot(nor1);
        D[3, 4] = d1v2.Dot(nor1);

        D[4, 1] = -(d1u1.Dot(nor2));
        D[4, 2] = -(d1v1.Dot(nor2));

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        D[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        D[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D nor1 = new Vector3D(), nor2 = new Vector3D(), p1p2;
        Vector3D ns1, ns2, ncrossns1, ncrossns2, resul = new Vector3D(), temp;

        double norm1, norm2, ndotns1, ndotns2, grosterme;

        surf1.D2(X[1], X[2], out pts1, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                 out Vector3D d2uv1);
        surf2.D2(X[3], X[4], out pts2, out Vector3D d1u2, out Vector3D d1v2, out Vector3D d2u2, out Vector3D d2v2,
                 out Vector3D d2uv2);

        p1p2 = new Vector3D(pts1, pts2);

        ns1 = d1u1.Crossed(d1v1);
        ns2 = d1u2.Crossed(d1v2);
        ncrossns1 = nplan.Crossed(ns1);
        ncrossns2 = nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude();
        norm2 = ncrossns2.Magnitude();

        ndotns1 = nplan.Dot(ns1);
        ndotns2 = nplan.Dot(ns2);

        nor1.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        nor2.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);

        F[1] = (nplan.XYZ().Dot(pts1.XYZ())) + theD;
        F[2] = (nplan.XYZ().Dot(pts2.XYZ())) + theD;

        F[3] = p1p2.Dot(nor1);
        F[4] = p1p2.Dot(nor2);

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);
        D[1, 3] = 0.0;
        D[1, 4] = 0.0;
        D[2, 1] = 0.0;
        D[2, 2] = 0.0;
        D[2, 3] = nplan.Dot(d1u2);
        D[2, 4] = nplan.Dot(d1v2);

        // Derivee de nor1 par rapport a u1

        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        D[3, 1] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul);

        // Derivee par rapport a v1

        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        D[3, 2] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul);

        D[3, 3] = d1u2.Dot(nor1);
        D[3, 4] = d1v2.Dot(nor1);

        D[4, 1] = -(d1u1.Dot(nor2));
        D[4, 2] = -(d1v1.Dot(nor2));

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        D[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        D[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul);

        return true;
    }

    public override void Tangent(double U1, double V1, double U2, double V2, out Vector3D TgF, out Vector3D TgL,
                                 out Vector3D NmF, out Vector3D NmL)
    {
        surf2.D1(U2, V2, out _, out Vector3D d1u, out Vector3D d1v);
        NmL = d1u.Crossed(d1v);

        surf1.D1(U1, V1, out _, out d1u, out d1v);
        NmF = d1u.Crossed(d1v);

        TgF = TgL = new Vector3D(pts1, pts2);
    }

    public override Point3D PointOnS1()
    {
        return pts1;
    }

    public override Point3D PointOnS2()
    {
        return pts2;
    }

    public override bool IsTangencyPoint()
    {
        return istangent;
    }

    public override Vector3D TangentOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::TangentOnS1");
        }

        return tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::TangentOnS2");
        }

        return tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::Tangent2dOnS1");
        }

        return tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::Tangent2dOnS2");
        }

        return tg22d;
    }

    /// <summary>
    /// The points and tangents of the section at <paramref name="Param"/> through (U1, V1) and (U2, V2), sampled at
    /// tabP.Length() points; false when the tangents cannot be computed.
    /// </summary>
    public bool GetSection(double Param, double U1, double V1, double U2, double V2, NCollection_Array1<Point3D> tabP,
                           NCollection_Array1<Vector3D> tabV)
    {
        int NbPoint = tabP.Length();
        if (NbPoint != tabV.Length() || NbPoint < 2)
        {
            throw new RangeErrorException();
        }

        int i, lowp = tabP.Lower(), lowv = tabV.Lower();

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ncrossns;
        double norm, ndotns, grosterme, lambda;

        math_Vector sol = new(1, 4), valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        curv.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));

        sol[1] = U1;
        sol[2] = V1;
        sol[3] = U2;
        sol[4] = V2;

        Values(sol, valsol, gradsol);

        surf1.D1(sol[1], sol[2], out pts1, out Vector3D d1u1, out Vector3D d1v1);
        surf2.D1(sol[3], sol[4], out pts2, out Vector3D d1u2, out Vector3D d1v2);

        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        secmember[1] = normtg - dnplan.Dot(new Vector3D(ptgui, pts1));
        secmember[2] = normtg - dnplan.Dot(new Vector3D(ptgui, pts2));

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();

        // Derivee de nor1 par rapport au parametre sur la ligne guide
        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        temp.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                           grosterme / norm, ns);

        secmember[3] = -(temp.Dot(new Vector3D(pts1, pts2)));

        ns = d1u2.Crossed(d1v2);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();

        // Derivee de nor2 par rapport au parametre sur la ligne guide
        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        temp.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                           grosterme / norm, ns);

        secmember[4] = -(temp.Dot(new Vector3D(pts1, pts2)));

        Gauss Resol = new(gradsol);
        if (Resol.IsDone)
        {
            Resol.Solve(secmember);

            tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);

            tabP[lowp] = pts1;
            tabP[lowp + NbPoint - 1] = pts2;

            tabV[lowv] = tg1;
            tabV[lowv + NbPoint - 1] = tg2;

            for (i = 2; i <= NbPoint - 1; i++)
            {
                lambda = (double)(i - 1) / (double)(NbPoint - 1);
                tabP[lowp + i - 1].SetXYZ((1.0 - lambda) * pts1.XYZ() + lambda * pts2.XYZ());
                tabV[lowv + i - 1].SetLinearForm(1.0 - lambda, tg1, lambda, tg2);
            }

            return true;
        }

        return false;
    }

    /// <summary>Returns false.</summary>
    public override bool IsRational()
    {
        return false;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_Ruled::GetSectionSize()");
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        Weigths.Init(1);
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
        NbKnots = 2;
        Degree = 1;
        NbPoles2d = 2;
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
        TKnots[TKnots.Lower()] = 0.0;
        TKnots[TKnots.Upper()] = 1.0;
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        TMults[TMults.Lower()] = TMults[TMults.Upper()] = 2;
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
        int lowp = Poles.Lower(), lowp2d = Poles2d.Lower();
        double u, v;
        Poles[lowp] = P.PointOnS1();
        Poles[lowp + 1] = P.PointOnS2();
        P.ParametersOnS1(out u, out v);
        Poles2d[lowp2d].SetCoord(u, v);
        P.ParametersOnS2(out u, out v);
        Poles2d[lowp2d + 1].SetCoord(u, v);
        Weights[lowp] = 1.0;
        Weights[lowp + 1] = 1.0;
        if (!P.IsTangencyPoint())
        {
            DPoles[lowp] = P.TangentOnS1();
            DPoles[lowp + 1] = P.TangentOnS2();
            DPoles2d[lowp2d] = P.Tangent2dOnS1();
            DPoles2d[lowp2d + 1] = P.Tangent2dOnS2();
            DWeights[lowp] = 0.0;
            DWeights[lowp + 1] = 0.0;
            return true;
        }

        return false;
    }

    public override void Section(in Blend_Point P, NCollection_Array1<Point3D> Poles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weights)
    {
        int lowp = Poles.Lower(), lowp2d = Poles2d.Lower();
        double u, v;
        Poles[lowp] = P.PointOnS1();
        Poles[lowp + 1] = P.PointOnS2();
        P.ParametersOnS1(out u, out v);
        Poles2d[lowp2d].SetCoord(u, v);
        P.ParametersOnS2(out u, out v);
        Poles2d[lowp2d + 1].SetCoord(u, v);
        Weights[lowp] = 1.0;
        Weights[lowp + 1] = 1.0;
    }

    /// <summary>The axis of rotation of the section plane at the guide parameter <paramref name="Prm"/>.</summary>
    public Axis3D AxeRot(double Prm)
    {
        Axis3D axrot = new Axis3D();
        Vector3D dirax, dnplan = new Vector3D();
        Point3D oriax = new Point3D();

        curv.D2(Prm, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        dirax = nplan.Crossed(dnplan);
        axrot.SetDirection(new Direction3D(dirax));
        oriax.SetXYZ(ptgui.XYZ() + (normtg / dnplan.Magnitude()) * dnplan.Normalized().XYZ());
        axrot.SetLocation(oriax);
        return axrot;
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
