// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSCircular.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSCircular.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified 10/09/1996 PMN Ajout de (Nb)Intervalles, IsRationnal
//                        + Utilisation de GeomFill::GetCircle dans Section.
// Modified 23/06/1997 PMN : Pb de division par 0.

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;

namespace Occt;

/// <summary>
/// The function of a circular blend between a curve C and a surface S. The planes' directions come from CGuide; their
/// position is on C, at the parameter the law L gives: Pguide = C(L(t)), Nguide = CGuide'(t)/||CGuide'(t)||. Its
/// variables are U, V on the surface.
/// </summary>
/// <remarks>OCCT leaves the radius, the "quadrant" and the conversion uninitialised; here they are zero.</remarks>
public class BlendFunc_CSCircular : Blend_CSFunction
{
    private const double Eps = 1.0e-15;

    private readonly Adaptor3d_Surface surf;
    private readonly Adaptor3d_Curve curv;
    private readonly Adaptor3d_Curve guide;
    private readonly Law_Function law;
    private Point3D pts;
    private Point3D ptc;
    private Point2D pt2d;
    private double prmc;
    private double dprmc;
    private bool istangent;
    private Vector3D tgs;
    private Vector2D tg2d;
    private Vector3D tgc;
    private double ray;
    private int choix;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D nplan;
    private double normtg;
    private double maxang;
    private double minang;
    private BlendFunc_SectionShape mySShape;
    private ParameterisationType myTConv;

    public BlendFunc_CSCircular(Adaptor3d_Surface S, Adaptor3d_Curve C, Adaptor3d_Curve CGuide, Law_Function L)
    {
        surf = S;
        curv = C;
        guide = CGuide;
        law = L;
        maxang = RealFirst();
        minang = RealLast();
        mySShape = BlendFunc_Rational;
        // prmc, dprmc, istangent, ray, choix, normtg, myTConv
        law = L;
    }

    /// <summary>Returns 2.</summary>
    public override int NbVariables()
    {
        return 2;
    }

    /// <summary>Returns 2.</summary>
    public override int NbEquations()
    {
        return 2;
    }

    /// <summary>Sets the radius and the "quadrant".</summary>
    public void Set(double Radius, int Choix)
    {
        choix = Choix;
        switch (Choix)
        {
            case 3:
            case 4:
                ray = Math.Abs(Radius);
                break;
            default:
                ray = -Math.Abs(Radius);
                break;
        }
    }

    /// <summary>Sets the type of the sections the approximation makes.</summary>
    public void Set(BlendFunc_SectionShape TypeSection)
    {
        mySShape = TypeSection;
    }

    public override void Set(double Param)
    {
        guide.D2(Param, out _, out d1gui, out d2gui);
        law.D1(Param, out prmc, out dprmc);

        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override void Set(double First, double Last)
    {
        throw new MethodNotImplementedException("BlendFunc_CSCircular::Set");
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = surf.UResolution(Tol);
        Tolerance[2] = surf.VResolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = surf.FirstUParameter();
        InfBound[2] = surf.FirstVParameter();
        SupBound[1] = surf.LastUParameter();
        SupBound[2] = surf.LastVParameter();

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[1]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[1]))
        {
            double range = (SupBound[1] - InfBound[1]);
            InfBound[1] -= range;
            SupBound[1] += range;
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[2]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[2]))
        {
            double range = (SupBound[2] - InfBound[2]);
            InfBound[2] -= range;
            SupBound[2] += range;
        }
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ncrossns, resul = new Vector3D(), nc;
        double norm, ndotns, grosterme;
        double Cosa, Sina, Angle;

        Values(Sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol * Tol)
        {
            // Calcul des tangentes

            pt2d = new Point2D(Sol[1], Sol[2]);

            surf.D1(Sol[1], Sol[2], out pts, out Vector3D d1u1, out Vector3D d1v1);
            curv.D2(prmc, out ptc, out Vector3D d1c, out _);

            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            ns = d1u1.Crossed(d1v1);
            ncrossns = nplan.Crossed(ns);
            ndotns = nplan.Dot(ns);
            norm = ncrossns.Magnitude();
            if (norm < Eps)
            {
                norm = 1.0;
            }

            temp.SetXYZ(pts.XYZ() - ptc.XYZ());
            secmember[1] = dprmc * (nplan.Dot(d1c)) - dnplan.Dot(temp);

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp.SetLinearForm(ray / norm * (dnplan.Dot(ns) - grosterme * ndotns), nplan, ray * ndotns / norm, dnplan,
                               ray * grosterme / norm, ns);
            temp -= dprmc * d1c;

            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
            resul.SetLinearForm(ray, ns, new Vector3D(ptc, pts));

            secmember[2] = -2.0 * (resul.Dot(temp));

            Gauss Resol = new(gradsol);
            if (Resol.IsDone)
            {
                Resol.Solve(secmember);

                tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tgc = dprmc * d1c;
                tg2d.SetCoord(secmember[1], secmember[2]);
                istangent = false;
            }
            else
            {
                istangent = true;
            }

            // mise a jour de maxang
            if (ray > 0.0)
            {
                ns.Reverse();
            }

            nc = -resul.Normalized();

            Cosa = ns.Dot(nc);
            Sina = nplan.Dot(ns.Crossed(nc));
            if (choix % 2 != 0)
            {
                Sina = -Sina; // nplan est change en -nplan
            }

            Angle = Math.Acos(Cosa);
            if (Sina < 0.0)
            {
                Angle = 2.0 * M_PI - Angle;
            }

            if (Angle > maxang)
            {
                maxang = Angle;
            }

            if (Angle < minang)
            {
                minang = Angle;
            }

            return true;
        }

        istangent = true;
        return false;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        surf.D1(X[1], X[2], out pts, out Vector3D d1u1, out Vector3D d1v1);
        curv.D1(prmc, out ptc, out _);

        F[1] = nplan.XYZ().Dot(pts.XYZ() - ptc.XYZ());

        Vector3D ns = d1u1.Crossed(d1v1);
        double norm = nplan.Crossed(ns).Magnitude();
        if (norm < Eps)
        {
            norm = 1.0;
        }

        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        Vector3D vref = new Vector3D();
        vref.SetLinearForm(ray, ns, new Vector3D(ptc, pts));

        F[2] = vref.SquareMagnitude() - ray * ray;

        pt2d = new Point2D(X[1], X[2]);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, nsov = new Vector3D(), vref = new Vector3D();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        curv.D1(prmc, out ptc, out _);

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        if (norm < Eps)
        {
            norm = 1.0;
        }

        ndotns = nplan.Dot(ns);

        nsov.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        vref.SetLinearForm(ray, nsov, new Vector3D(ptc, pts));

        // Derivee par rapport a u de Ps + ray*ns
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[2, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[2, 2] = 2.0 * (resul.Dot(vref));

        pt2d = new Point2D(X[1], X[2]);
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, vref = new Vector3D(), nsov = new Vector3D();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        curv.D1(prmc, out ptc, out _);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        if (norm < Eps)
        {
            norm = 1.0;
        }

        ndotns = nplan.Dot(ns);
        nsov.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
        vref.SetLinearForm(ray, nsov, new Vector3D(ptc, pts));

        F[1] = nplan.XYZ().Dot(pts.XYZ() - ptc.XYZ());
        F[2] = vref.SquareMagnitude() - ray * ray;

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);

        // Derivee par rapport a u
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[2, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[2, 2] = 2.0 * (resul.Dot(vref));

        pt2d = new Point2D(X[1], X[2]);
        return true;
    }

    public override Point3D PointOnS()
    {
        return pts;
    }

    public override Point3D PointOnC()
    {
        return ptc;
    }

    public override Point2D Pnt2d()
    {
        return pt2d;
    }

    public override double ParameterOnC()
    {
        return prmc;
    }

    public override bool IsTangencyPoint()
    {
        return istangent;
    }

    public override Vector3D TangentOnS()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_CSCircular::TangentOnS");
        }

        return tgs;
    }

    public override Vector3D TangentOnC()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_CSCircular::TangentOnC");
        }

        return tgc;
    }

    public override Vector2D Tangent2d()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_CSCircular::Tangent2d");
        }

        return tg2d;
    }

    public override void Tangent(double U, double V, out Vector3D TgS, out Vector3D NmS)
    {
        Vector3D ns;
        surf.D1(U, V, out _, out Vector3D d1u, out Vector3D d1v);
        NmS = ns = d1u.Crossed(d1v);

        double norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        if (ray > 0.0)
        {
            ns.Reverse();
        }

        TgS = nplan.Crossed(ns);
        if (choix % 2 == 1)
        {
            TgS.Reverse();
        }
    }

    /// <summary>The circular section at <paramref name="Param"/> through (U, V) and W, with its parameter range.</summary>
    public void Section(double Param, double U, double V, double W, out double Pdeb, out double Pfin, ref Circle3D C)
    {
        Vector3D ns;
        double norm;
        Point3D Center = new Point3D();

        guide.D1(Param, out _, out d1gui);
        nplan = d1gui.Normalized();

        surf.D1(U, V, out pts, out Vector3D d1u1, out Vector3D d1v1);
        ptc = curv.Value(W);

        ns = d1u1.Crossed(d1v1);
        norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());
        C.SetRadius(Math.Abs(ray));
        if (ray > 0.0)
        {
            ns.Reverse();
        }

        if (choix % 2 == 0)
        {
            C.SetPosition(new Frame3D(Center, new Direction3D(nplan), new Direction3D(ns)));
        }
        else
        {
            C.SetPosition(new Frame3D(Center, new Direction3D(nplan.Reversed()), new Direction3D(ns)));
        }

        Pdeb = 0.0;
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, ptc);
    }

    /// <summary>Blend_CSFunction's: returns false.</summary>
    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths,
                                 NCollection_Array1<double> D2Weigths)
    {
        return base.Section(in P, Poles, DPoles, D2Poles, Poles2d, DPoles2d, D2Poles2d, Weigths, DWeigths, D2Weigths);
    }

    /// <summary>
    /// The points and tangents of the section at <paramref name="Param"/> through (U, V), sampled at tabP.Length()
    /// points; false when the tangents cannot be computed. W is not used.
    /// </summary>
    public bool GetSection(double Param, double U, double V, double W, NCollection_Array1<Point3D> tabP,
                           NCollection_Array1<Vector3D> tabV)
    {
        int NbPoint = tabP.Length();
        if (NbPoint != tabV.Length() || NbPoint < 2)
        {
            throw new RangeErrorException();
        }

        int i, lowp = tabP.Lower(), lowv = tabV.Lower();

        Vector3D ns, dnplan = new Vector3D(), dnw = new Vector3D(), dn2w = new Vector3D(), ncrn = new Vector3D(),
               dncrn = new Vector3D(), ns2 = new Vector3D();
        Vector3D ncrossns, resul = new Vector3D();
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D();

        double norm, ndotns, grosterme;
        double lambda, Cosa, Sina;
        double Angle = 0.0, Dangle = 0.0;
        math_Vector sol = new(1, 2), valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        Set(Param);
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        curv.D2(prmc, out ptc, out Vector3D d1c, out _);
        surf.D2(U, V, out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1, out Vector3D d2uv1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();

        temp.SetXYZ(pts.XYZ() - ptc.XYZ());
        secmember[1] = dprmc * (nplan.Dot(d1c)) - dnplan.Dot(temp);

        ns2.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);

        // Derivee de n1 par rapport a w (param sur ligne guide)

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ray, dnw, -dprmc, d1c);
        resul.SetLinearForm(ray, ns2, new Vector3D(ptc, pts));

        secmember[2] = -2.0 * (resul.Dot(temp));

        sol[1] = U;
        sol[2] = V;

        Values(sol, valsol, gradsol);
        Gauss Resol = new(gradsol);

        if (Resol.IsDone)
        {
            Resol.Solve(secmember);

            tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tgc = dprmc * d1c;

            // Derivee de n1 par rapport a u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulu.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivee de n1 par rapport a v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulv.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw.SetLinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns = ns2;
            dn2w.SetLinearForm(ray, dnw, -1.0, tgc, tgs);
            norm = resul.Magnitude();
            dn2w.Divide(norm);
            ns2 = -resul.Normalized();
            dn2w.SetLinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

            if (ray > 0.0)
            {
                ns.Reverse();
                dnw.Reverse();
            }

            if (choix % 2 != 0)
            {
                nplan.Reverse();
                dnplan.Reverse();
            }

            tabP[lowp] = pts;
            tabP[lowp + NbPoint - 1] = ptc;

            tabV[lowv] = tgs;
            tabV[lowv + NbPoint - 1] = tgc;

            if (NbPoint > 2)
            {
                Cosa = ns.Dot(ns2);
                Sina = nplan.Dot(ns.Crossed(ns2));
                Angle = Math.Acos(Cosa);
                if (Sina < 0.0)
                {
                    Angle = 2.0 * M_PI - Angle;
                }

                Dangle = -(dnw.Dot(ns2) + ns.Dot(dn2w)) / Sina;
                ncrn = nplan.Crossed(ns);
                dncrn = dnplan.Crossed(ns).Added(nplan.Crossed(dnw));
            }

            for (i = 2; i <= NbPoint - 1; i++)
            {
                lambda = (double)(i - 1) / (double)(NbPoint - 1);
                Cosa = Math.Cos(lambda * Angle);
                Sina = Math.Sin(lambda * Angle);
                tabP[lowp + i - 1]
                    .SetXYZ(pts.XYZ() + Math.Abs(ray) * ((Cosa - 1) * ns.XYZ() + Sina * ncrn.XYZ()));

                temp.SetLinearForm(-Sina, ns, Cosa, ncrn);
                temp.Multiply(lambda * Dangle);
                temp.Add(((Cosa - 1) * dnw).Added(Sina * dncrn));
                temp.Multiply(Math.Abs(ray));
                temp.Add(tgs);
                tabV[lowv + i - 1] = temp;
            }

            return true;
        }

        return false;
    }

    public override bool IsRational()
    {
        return (mySShape == BlendFunc_Rational || mySShape == BlendFunc_QuasiAngular);
    }

    public override double GetSectionSize()
    {
        return maxang * Math.Abs(ray);
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        BlendFunc.GetMinimalWeights(mySShape, myTConv, minang, maxang, Weigths);
        // On suppose que cela ne depend pas du Rayon!
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
        NbPoles2d = 1;
        BlendFunc.GetShape(mySShape, maxang, out NbPoles, out NbKnots, out Degree, ref myTConv);
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerances a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1d)
    {
        int low = Tol3d.Lower();
        int up = Tol3d.Upper();
        // OCCT passes the signed radius here, where BlendFunc_CSConstRad passes its absolute value.
        double Tol = GeomFill.GetTolerance(myTConv, minang, ray, AngleTol, SurfTol);
        Tol1d.Init(SurfTol);
        Tol3d.Init(SurfTol);
        Tol3d[low + 1] = Tol3d[up - 1] = Min(Tol, SurfTol);
        Tol3d[low] = Tol3d[up] = Min(Tol, BoundTol);
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        GeomFill.Knots(myTConv, TKnots);
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        GeomFill.Mults(myTConv, TMults);
    }

    public override void Section(in Blend_Point P, NCollection_Array1<Point3D> Poles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weights)
    {
        Vector3D ns, ns2;
        Point3D Center = new Point3D();
        double norm, u1, v1;

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();

        Set(prm);

        P.ParametersOnS(out u1, out v1);
        surf.D1(u1, v1, out pts, out Vector3D d1u1, out Vector3D d1v1);
        ptc = curv.Value(prmc);

        Poles2d[Poles2d.Lower()].SetCoord(u1, v1);

        // Cas Linear
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts;
            Poles[upp] = ptc;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            return;
        }

        ns = d1u1.Crossed(d1v1);
        norm = nplan.Crossed(ns).Magnitude();

        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);

        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());

        ns2 = new Vector3D(Center, ptc).Normalized();
        if (ray > 0.0)
        {
            ns.Reverse();
        }

        if (choix % 2 != 0)
        {
            nplan.Reverse();
        }

        GeomFill.GetCircle(myTConv, ns, ns2, nplan, pts, ptc, Math.Abs(ray), Center, Poles, Weights);
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        Vector3D ns, ns2 = new Vector3D(), dnplan = new Vector3D(), dnw = new Vector3D(), dn2w = new Vector3D();
        Vector3D ncrossns;
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D(), tgct = new Vector3D(),
               resul = new Vector3D();

        Point3D Center = new Point3D();
        double norm, ndotns, grosterme;

        math_Vector sol = new(1, 2), valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt;

        Set(prm);
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        curv.D2(prmc, out ptc, out Vector3D d1, out _);
        P.ParametersOnS(out sol[1], out sol[2]);
        surf.D2(sol[1], sol[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();
        ns2.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);

        temp.SetXYZ(pts.XYZ() - ptc.XYZ());
        secmember[1] = dprmc * (nplan.Dot(d1)) - dnplan.Dot(temp);

        // Derivee de n1 par rapport a w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ray, dnw, -dprmc, d1);
        resul.SetLinearForm(ray, ns2, new Vector3D(ptc, pts));

        secmember[2] = -2.0 * (resul.Dot(temp));

        Values(sol, valsol, gradsol);
        Gauss Resol = new(gradsol);

        if (Resol.IsDone)
        {
            Resol.Solve(secmember);

            tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tgc = dprmc * d1;

            // Derivee de n1 par rapport a u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulu.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivee de n1 par rapport a v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulv.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw.SetLinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns = ns2;
            dn2w.SetLinearForm(ray, dnw, -1.0, tgc, tgs);
            norm = resul.Magnitude();
            dn2w.Divide(norm);
            ns2 = -resul.Normalized();
            dn2w.SetLinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

            istgt = false;
        }
        else
        {
            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
            ns2 = -resul.Normalized();
            istgt = true;
        }

        // Les poles 2d

        Poles2d[Poles2d.Lower()].SetCoord(sol[1], sol[2]);
        if (!istgt)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(secmember[1], secmember[2]);
        }

        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts;
            Poles[upp] = ptc;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            if (!istgt)
            {
                DPoles[low] = tgs;
                DPoles[upp] = tgc;
                DWeights[low] = 0.0;
                DWeights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Cas du cercle
        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());
        if (!istgt)
        {
            tgct = tgs.Added(ray * dnw);
        }

        if (ray > 0.0)
        {
            ns.Reverse();
            if (!istgt)
            {
                dnw.Reverse();
            }
        }

        if (choix % 2 != 0)
        {
            nplan.Reverse();
            dnplan.Reverse();
        }

        if (!istgt)
        {
            return GeomFill.GetCircle(myTConv, ns, ns2, dnw, dn2w, nplan, dnplan, pts, ptc, tgs, tgc, Math.Abs(ray), 0,
                                      Center, tgct, Poles, DPoles, Weights, DWeights);
        }
        else
        {
            GeomFill.GetCircle(myTConv, ns, ns2, nplan, pts, ptc, Math.Abs(ray), Center, Poles, Weights);
            return false;
        }
    }

    public override void Resolution(int IC2d, double Tol, out double TolU, out double TolV)
    {
        TolU = surf.UResolution(Tol);
        TolV = surf.VResolution(Tol);
    }
}
