// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified 10/09/1996 PMN Ajout de (Nb)Intervalles, IsRationnal
//                       + Utilisation de GeomFill::GetCircle dans Section.

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Occt;

/// <summary>
/// The function of a constant-radius fillet between a surface and a curve along a guide curve: its variables are
/// U, V on the surface and W on the curve.
/// </summary>
public class BlendFunc_CSConstRad : Blend_CSFunction
{
    private readonly SurfaceAdaptorBase surf;
    private readonly CurveAdaptorBase curv;
    private readonly CurveAdaptorBase guide;
    private Point3D pts;
    private Point3D ptc;
    private Point2D pt2d;
    private double prmc;
    private bool istangent;
    private Vector3D tgs;
    private Vector2D tg2d;
    private Vector3D tgc;
    private double ray;
    private int choix;
    private Point3D ptgui;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D nplan;
    private double normtg;
    private double theD;
    private double maxang;
    private double minang;
    private BlendFunc_SectionShape mySShape;
    private ParameterisationType myTConv;

    public BlendFunc_CSConstRad(SurfaceAdaptorBase S, CurveAdaptorBase C, CurveAdaptorBase CGuide)
    {
        surf = S;
        curv = C;
        guide = CGuide;
        prmc = 0.0;
        istangent = true;
        ray = 0.0;
        choix = 0;
        normtg = 0.0;
        theD = 0.0;
        maxang = RealFirst();
        minang = RealLast();
        mySShape = BlendFunc_Rational;
        myTConv = TgtThetaOver2;
    }

    /// <summary>Returns 3.</summary>
    public override int NbEquations()
    {
        return 3;
    }

    /// <summary>Sets the radius and the "quadrant".</summary>
    public void Set(double Radius, int Choix)
    {
        choix = Choix;
        ray = -Math.Abs(Radius);
    }

    /// <summary>Sets the type of the sections the approximation makes.</summary>
    public void Set(BlendFunc_SectionShape TypeSection)
    {
        mySShape = TypeSection;
    }

    public override void Set(double Param)
    {
        guide.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override void Set(double First, double Last)
    {
        throw new MethodNotImplementedException("BlendFunc_CSConstRad::Set");
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = surf.UResolution(Tol);
        Tolerance[2] = surf.VResolution(Tol);
        Tolerance[3] = curv.Resolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = surf.FirstUParameter();
        InfBound[2] = surf.FirstVParameter();
        InfBound[3] = curv.FirstParameter();
        SupBound[1] = surf.LastUParameter();
        SupBound[2] = surf.LastVParameter();
        SupBound[3] = curv.LastParameter();

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
        math_Vector valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ns2, ncrossns, resul = new Vector3D();
        double norm, ndotns, grosterme;
        double Cosa, Sina, Angle;

        Values(Sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= Tol * Tol)
        {
            // Calcul des tangentes

            pt2d = new Point2D(Sol[1], Sol[2]);
            prmc = Sol[3];
            surf.D1(Sol[1], Sol[2], out pts, out Vector3D d1u1, out Vector3D d1v1);
            curv.D1(Sol[3], out ptc, out Vector3D d1);
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp.SetXYZ(pts.XYZ() - ptgui.XYZ());
            secmember[1] = normtg - dnplan.Dot(temp);

            temp.SetXYZ(ptc.XYZ() - ptgui.XYZ());
            secmember[2] = normtg - dnplan.Dot(temp);

            ns = d1u1.Crossed(d1v1);
            ncrossns = nplan.Crossed(ns);
            ndotns = nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp.SetLinearForm(ray / norm * (dnplan.Dot(ns) - grosterme * ndotns), nplan, ray * ndotns / norm, dnplan,
                               ray * grosterme / norm, ns);

            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
            resul.SetLinearForm(ray, ns, new Vector3D(ptc, pts));

            secmember[3] = -2.0 * (temp.Dot(resul));

            Gauss Resol = new(gradsol);
            if (Resol.IsDone)
            {
                Resol.Solve(secmember);

                tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tgc = secmember[3] * d1;
                tg2d.SetCoord(secmember[1], secmember[2]);
                istangent = false;
            }
            else
            {
                istangent = true;
            }

            // mise a jour de maxang

            ns2 = -resul.Normalized();

            Cosa = ns.Dot(ns2);
            Sina = nplan.Dot(ns.Crossed(ns2));
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
        ptc = curv.Value(X[3]);

        F[1] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(ptc.XYZ()) + theD;

        Vector3D vref = new Vector3D(), ns = d1u1.Crossed(d1v1);
        double norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        vref.SetLinearForm(ray, ns, new Vector3D(ptc, pts));

        F[3] = vref.SquareMagnitude() - ray * ray;

        pt2d = new Point2D(X[1], X[2]);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, vref = new Vector3D();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        curv.D1(X[3], out ptc, out Vector3D d1);

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);
        D[1, 3] = 0.0;

        D[2, 1] = 0.0;
        D[2, 2] = 0.0;
        D[2, 3] = nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        ndotns = nplan.Dot(ns);

        vref.SetLinearForm(ndotns, nplan, -1.0, ns);
        vref.Divide(norm);
        vref.SetLinearForm(ray, vref, new Vector3D(ptc, pts));

        // Derivee par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[3, 2] = 2.0 * (resul.Dot(vref));

        D[3, 3] = -2.0 * (d1.Dot(vref));

        pt2d = new Point2D(X[1], X[2]);
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, vref = new Vector3D();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        curv.D1(X[3], out ptc, out Vector3D d1);

        F[1] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(ptc.XYZ()) + theD;

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);
        D[1, 3] = 0.0;

        D[2, 1] = 0.0;
        D[2, 2] = 0.0;
        D[2, 3] = nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        ndotns = nplan.Dot(ns);

        vref.SetLinearForm(ndotns, nplan, -1.0, ns);
        vref.Divide(norm);
        vref.SetLinearForm(ray, vref, new Vector3D(ptc, pts));

        F[3] = vref.SquareMagnitude() - ray * ray;

        // Derivee par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[3, 2] = 2.0 * (resul.Dot(vref));

        D[3, 3] = -2.0 * (d1.Dot(vref));

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
            throw new DomainErrorException("BlendFunc_CSConstRad::TangentOnS");
        }

        return tgs;
    }

    public override Vector3D TangentOnC()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_CSConstRad::TangentOnC");
        }

        return tgc;
    }

    public override Vector2D Tangent2d()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_CSConstRad::Tangent2d");
        }

        return tg2d;
    }

    public override void Tangent(double U, double V, out Vector3D TgS, out Vector3D NmS)
    {
        surf.D1(U, V, out Point3D bid, out Vector3D d1u, out Vector3D d1v);
        Vector3D ns;
        NmS = ns = d1u.Crossed(d1v);

        double norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        Point3D Center = new Point3D(bid.XYZ() + ray * ns.XYZ());
        TgS = nplan.Crossed(new Vector3D(Center, bid));
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

        guide.D1(Param, out ptgui, out d1gui);
        nplan = d1gui.Normalized();

        surf.D1(U, V, out pts, out Vector3D d1u1, out Vector3D d1v1);
        ptc = curv.Value(W);

        ns = d1u1.Crossed(d1v1);
        norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());
        C.SetRadius(Math.Abs(ray));
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

    /// <summary>Not implemented: throws <see cref="DomainErrorException"/>.</summary>
    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights,
                                 NCollection_Array1<double> D2Weights)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    /// <summary>
    /// The points and tangents of the section at <paramref name="Param"/> through (U, V) and W, sampled at
    /// tabP.Length() points; false when the tangents cannot be computed.
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
               dncrn = new Vector3D(), ns2;
        Vector3D ncrossns, resul = new Vector3D();
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D();

        double norm, ndotns, grosterme;
        double lambda, Cosa, Sina;
        double Angle = 0.0, Dangle = 0.0;
        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        guide.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        sol[1] = U;
        sol[2] = V;
        sol[3] = W;

        Values(sol, valsol, gradsol);

        surf.D2(U, V, out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1, out Vector3D d2uv1);
        curv.D1(W, out ptc, out Vector3D d1);

        temp.SetXYZ(pts.XYZ() - ptgui.XYZ());
        secmember[1] = normtg - dnplan.Dot(temp);

        temp.SetXYZ(ptc.XYZ() - ptgui.XYZ());
        secmember[2] = normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();

        // Derivee de n1 par rapport a w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
        resul.SetLinearForm(ray, temp, new Vector3D(ptc, pts));
        secmember[3] = -2.0 * ray * (dnw.Dot(resul)); // jag 950105 il manquait ray

        Gauss Resol = new(gradsol);
        if (Resol.IsDone)
        {
            Resol.Solve(secmember);

            tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tgc = secmember[3] * d1;

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
            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);

            dn2w.SetLinearForm(ray, dnw, -1.0, tgc, tgs);
            norm = resul.Magnitude();
            dn2w.Divide(norm);
            ns2 = -resul.Normalized();
            dn2w.SetLinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

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

    public override void GetMinimalWeight(NCollection_Array1<double> Weights)
    {
        BlendFunc.GetMinimalWeights(mySShape, myTConv, minang, maxang, Weights);
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
    // purpose  : Determine les Tolerance a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1d)
    {
        int low = Tol3d.Lower();
        int up = Tol3d.Upper();
        double Tol = GeomFill.GetTolerance(myTConv, minang, Math.Abs(ray), AngleTol, SurfTol);
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
        double norm, u1, v1, w;

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();

        guide.D1(prm, out ptgui, out d1gui);
        nplan = d1gui.Normalized();

        P.ParametersOnS(out u1, out v1);
        w = P.ParameterOnC();

        surf.D1(u1, v1, out pts, out Vector3D d1u1, out Vector3D d1v1);
        ptc = curv.Value(w);

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
        Vector3D ns, ns2, dnplan = new Vector3D(), dnw = new Vector3D(), dn2w = new Vector3D();
        Vector3D ncrossns;
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D(), tgct = new Vector3D(),
               resul = new Vector3D();

        Point3D Center = new Point3D();
        double norm, ndotns, grosterme;

        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt;

        guide.D2(prm, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        P.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = P.ParameterOnC();

        Values(sol, valsol, gradsol);

        surf.D2(sol[1], sol[2], out pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        curv.D1(sol[3], out ptc, out Vector3D d1);

        temp.SetXYZ(pts.XYZ() - ptgui.XYZ());
        secmember[1] = normtg - dnplan.Dot(temp);

        temp.SetXYZ(ptc.XYZ() - ptgui.XYZ());
        secmember[2] = normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();

        // Derivee de n1 par rapport a w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
        resul.SetLinearForm(ray, temp, new Vector3D(ptc, pts));
        secmember[3] = -2.0 * ray * (dnw.Dot(resul)); // jag 950105 il manquait ray

        Gauss Resol = new(gradsol);

        if (Resol.IsDone)
        {
            Resol.Solve(secmember);

            tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tgc = secmember[3] * d1;

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
            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);

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

        // Cas Linear
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
