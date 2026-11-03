// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;

namespace Occt;

/// <summary>
/// The function of a constant-radius fillet between a surface and a restriction (an arc of another surface) along a
/// guide: the variables are (U, V) on the surface and the parameter on the restriction.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
public class BRepBlend_SurfRstConstRad : Blend_SurfRstFunction
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase surf;
    private readonly SurfaceAdaptorBase surfrst;
    private readonly Curve2DAdaptorBase rst;
    private readonly CurveOnSurface cons;
    private readonly CurveAdaptorBase guide;
    private CurveAdaptorBase tguide;
    private Point3D pts;
    private Point3D ptrst;
    private Point2D pt2ds;
    private Point2D pt2drst;
    private double prmrst;
    private bool istangent;
    private Vector3D tgs;
    private Vector2D tg2ds;
    private Vector3D tgrst;
    private Vector2D tg2drst;
    private double ray;
    private int choix;
    private Point3D ptgui;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D nplan;
    private double normtg;
    private double theD;
    private SurfaceAdaptorBase? surfref;
    private Curve2DAdaptorBase? rstref;
    private double maxang;
    private double minang;
    private double distmin;
    private BlendFunc_SectionShape mySShape;
    // OCCT leaves myTConv uninitialised until GetShape sets it; here it starts at its first value.
    private ParameterisationType myTConv;

    private static void t3dto2d(out double a, out double b, in Vector3D A, in Vector3D B, in Vector3D C)
    {
        double AB = A.Dot(B);
        double AC = A.Dot(C);
        double BC = B.Dot(C);
        double BB = B.Dot(B);
        double CC = C.Dot(C);
        double deno = (BB * CC - BC * BC);
        a = (AB * CC - AC * BC) / deno;
        b = (AC * BB - AB * BC) / deno;
    }

    public BRepBlend_SurfRstConstRad(SurfaceAdaptorBase Surf, SurfaceAdaptorBase SurfRst, Curve2DAdaptorBase Rst,
                                     CurveAdaptorBase CGuide)
    {
        surf = Surf;
        surfrst = SurfRst;
        rst = Rst;
        cons = new CurveOnSurface(Rst, SurfRst);
        guide = CGuide;
        tguide = CGuide;
        prmrst = 0.0;
        istangent = true;
        ray = 0.0;
        choix = 0;
        normtg = 0.0;
        theD = 0.0;
        maxang = RealFirst();
        minang = RealLast();
        distmin = RealLast();
        mySShape = BlendFunc_Rational;
    }

    public override int NbVariables()
    {
        return 3;
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        Vector3D d1u1, d1v1, ns, vref = new();
        double norm;

        surf.D1(X[1], X[2], out pts, out d1u1, out d1v1);
        ptrst = cons.Value(X[3]);

        F[1] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(ptrst.XYZ()) + theD;

        ns = d1u1.Crossed(d1v1);
        norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        vref.SetLinearForm(ray, ns, new Vector3D(ptrst, pts));
        vref /= ray;
        F[3] = (vref.SquareMagnitude() - 1) * ray * ray;
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D d1u1, d1v1, d2u1, d2v1, d2uv1, d1;
        Vector3D ns, ncrossns, resul = new(), temp, vref = new();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        cons.D1(X[3], out ptrst, out d1);

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
        vref.SetLinearForm(ray, vref, new Vector3D(ptrst, pts));

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[3, 1] = resul.Dot(vref);
        D[3, 1] = D[3, 1] * 2.0;

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[3, 2] = resul.Dot(vref);
        D[3, 2] = D[3, 2] * 2.0;

        D[3, 3] = d1.Dot(vref);
        D[3, 3] = D[3, 3] * (-2.0);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D d1u1, d1v1, d1;
        Vector3D d2u1, d2v1, d2uv1;
        Vector3D ns, ncrossns, resul = new(), temp, vref = new();

        double norm, ndotns, grosterme;

        surf.D2(X[1], X[2], out pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        cons.D1(X[3], out ptrst, out d1);

        F[1] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(ptrst.XYZ()) + theD;

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
        vref.SetLinearForm(ray, vref, new Vector3D(ptrst, pts));

        temp = vref / ray;
        F[3] = (temp.SquareMagnitude() - 1) * ray * ray; // more stable numerically

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1u1);

        D[3, 1] = resul.Dot(vref);
        D[3, 1] = D[3, 1] * 2.0;

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-ray / norm * (grosterme * ndotns - nplan.Dot(temp)), nplan, ray * grosterme / norm, ns,
                            -ray / norm, temp, d1v1);

        D[3, 2] = resul.Dot(vref);
        D[3, 2] = D[3, 2] * 2.0;

        D[3, 3] = d1.Dot(vref);
        D[3, 3] = D[3, 3] * (-2.0);

        return true;
    }

    /// <summary>The reference surface and restriction <see cref="Decroch"/> tests against.</summary>
    public void Set(SurfaceAdaptorBase SurfRef, Curve2DAdaptorBase RstRef)
    {
        surfref = SurfRef;
        rstref = RstRef;
    }

    public override void Set(double Param)
    {
        d1gui = new Vector3D(0.0, 0.0, 0.0);
        nplan = new Vector3D(0.0, 0.0, 0.0);
        tguide.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan.SetXYZ(d1gui.Normalized().XYZ());
        Xyz nplanXYZ = nplan.XYZ();
        Xyz ptguiXYZ = ptgui.XYZ();
        theD = nplanXYZ.Dot(ptguiXYZ);
        theD = theD * (-1.0);
    }

    public override void Set(double First, double Last)
    {
        tguide = guide.Trim(First, Last, 1.0e-12);
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = surf.UResolution(Tol);
        Tolerance[2] = surf.VResolution(Tol);
        Tolerance[3] = cons.Resolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = surf.FirstUParameter();
        InfBound[2] = surf.FirstVParameter();
        InfBound[3] = cons.FirstParameter();
        SupBound[1] = surf.LastUParameter();
        SupBound[2] = surf.LastVParameter();
        SupBound[3] = cons.LastParameter();

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

    /// <summary>
    /// True if <paramref name="Sol"/> solves the function within <paramref name="Tol"/>; then computes the tangents
    /// and updates the extreme angles and the minimal distance of the sections (called once per accepted point).
    /// </summary>
    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        Vector3D dnplan = new(), d1u1, d1v1, d1urst, d1vrst, d1, temp = new(), ns, ns2, ncrossns, resul = new();
        Point3D bid;
        double norm, ndotns, grosterme;
        double Cosa, Sina, Angle;

        Values(Sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= 2 * Tol * Math.Abs(ray))
        {
            // Calculation of tangents

            pt2ds = new Point2D(Sol[1], Sol[2]);
            prmrst = Sol[3];
            pt2drst = rst.Value(prmrst);
            surf.D1(Sol[1], Sol[2], out pts, out d1u1, out d1v1);
            cons.D1(Sol[3], out ptrst, out d1);
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp.SetXYZ(pts.XYZ() - ptgui.XYZ());
            secmember[1] = normtg - dnplan.Dot(temp);

            temp.SetXYZ(ptrst.XYZ() - ptgui.XYZ());
            secmember[2] = normtg - dnplan.Dot(temp);

            ns = d1u1.Crossed(d1v1);
            ncrossns = nplan.Crossed(ns);
            ndotns = nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp.SetLinearForm(ray / norm * (dnplan.Dot(ns) - grosterme * ndotns), nplan, ray * ndotns / norm, dnplan,
                               ray * grosterme / norm, ns);

            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
            resul.SetLinearForm(ray, ns, new Vector3D(ptrst, pts));
            secmember[3] = -2.0 * (temp.Dot(resul));

            Gauss Resol = new(gradsol);
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
                    math_Vector DEDT = new(1, 3);
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
                tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tgrst = secmember[3] * d1;
                tg2ds.SetCoord(secmember[1], secmember[2]);
                surfrst.D1(pt2drst.X(), pt2drst.Y(), out bid, out d1urst, out d1vrst);
                double a, b;
                t3dto2d(out a, out b, tgrst, d1urst, d1vrst);
                tg2drst.SetCoord(a, b);
            }

            // update of maxang
            if (ray > 0.0)
            {
                ns.Reverse();
            }
            ns2 = -resul.Normalized();

            Cosa = ns.Dot(ns2);
            Sina = nplan.Dot(ns.Crossed(ns2));
            if (choix % 2 != 0)
            {
                Sina = -Sina; // nplan is changed to -nplan
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
            distmin = Min(distmin, pts.Distance(ptrst));

            return true;
        }
        istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return distmin;
    }

    public override Point3D PointOnS() => pts;

    public override Point3D PointOnRst() => ptrst;

    public override Point2D Pnt2dOnS() => pt2ds;

    public override Point2D Pnt2dOnRst() => pt2drst;

    public override double ParameterOnRst() => prmrst;

    public override bool IsTangencyPoint() => istangent;

    public override Vector3D TangentOnS()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tgs;
    }

    public override Vector2D Tangent2dOnS()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tg2ds;
    }

    public override Vector3D TangentOnRst()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tgrst;
    }

    public override Vector2D Tangent2dOnRst()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tg2drst;
    }

    /// <summary>True if the section at <paramref name="Sol"/> leaves the reference surface (see <see cref="Set(SurfaceAdaptorBase, Curve2DAdaptorBase)"/>).</summary>
    public override bool Decroch(math_Vector Sol, out Vector3D NS, out Vector3D TgS)
    {
        Vector3D TgRst, NRst, NRstInPlane, NSInPlane;
        Point3D bid, Center = new();
        Vector3D d1u, d1v;
        double norm, unsurnorm;

        surf.D1(Sol[1], Sol[2], out bid, out d1u, out d1v);
        NS = NSInPlane = d1u.Crossed(d1v);

        norm = nplan.Crossed(NS).Magnitude();
        unsurnorm = 1.0 / norm;
        NSInPlane.SetLinearForm(nplan.Dot(NS) * unsurnorm, nplan, -unsurnorm, NS);

        Center.SetXYZ(bid.XYZ() + ray * NSInPlane.XYZ());
        if (choix > 2)
        {
            NSInPlane.Reverse();
        }
        TgS = nplan.Crossed(new Vector3D(Center, bid));
        if (choix % 2 == 1)
        {
            TgS.Reverse();
        }
        double u, v;
        rstref!.Value(Sol[3]).Coord(out u, out v);
        surfref!.D1(u, v, out bid, out d1u, out d1v);
        NRst = d1u.Crossed(d1v);
        norm = nplan.Crossed(NRst).Magnitude();
        unsurnorm = 1.0 / norm;
        NRstInPlane = new Vector3D();
        NRstInPlane.SetLinearForm(nplan.Dot(NRst) * unsurnorm, nplan, -unsurnorm, NRst);
        Vector3D centptrst = new(Center, bid);
        if (centptrst.Dot(NRstInPlane) < 0.0)
        {
            NRstInPlane.Reverse();
        }
        TgRst = nplan.Crossed(centptrst);
        if (choix % 2 == 1)
        {
            TgRst.Reverse();
        }

        double dot, NT = NRstInPlane.Magnitude();
        NT *= TgRst.Magnitude();
        if (Math.Abs(NT) < 1.0e-7)
        {
            return false; // Singularity or Incoherence.
        }
        dot = NRstInPlane.Dot(TgRst);
        dot /= NT;

        return (dot < 1.0e-10);
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="Choix"/> (1 to 4) in which the fillet lies.</summary>
    public void Set(double Radius, int Choix)
    {
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
                ray = -Math.Abs(Radius);
                break;
            case 3:
            case 4:
                ray = Math.Abs(Radius);
                break;
            default:
                ray = -Math.Abs(Radius);
                break;
        }
    }

    /// <summary>Sets the type of section generation for the approximations.</summary>
    public void Set(BlendFunc_SectionShape TypeSection)
    {
        mySShape = TypeSection;
    }

    /// <summary>The circular section at <paramref name="Param"/> through the points (U, V) and W, and its bounds.</summary>
    public void Section(double Param, double U, double V, double W, out double Pdeb, out double Pfin, ref Circle3D C)
    {
        Vector3D d1u1, d1v1;
        Vector3D ns, np;
        double norm;
        Point3D Center = new();

        tguide.D1(Param, out ptgui, out d1gui);
        np = d1gui.Normalized();

        surf.D1(U, V, out pts, out d1u1, out d1v1);
        ptrst = cons.Value(W);

        ns = d1u1.Crossed(d1v1);

        norm = nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);
        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());
        C.SetRadius(Math.Abs(ray));

        if (ray > 0)
        {
            ns.Reverse();
        }

        if (choix % 2 != 0)
        {
            np.Reverse();
        }

        C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns)));
        Pdeb = 0; // ElCLib::Parameter(C,pts);
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, ptrst);

        // Test negative and almost null angles : Special case
        if (Pfin > 1.5 * M_PI)
        {
            np.Reverse();
            C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns)));
            Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, ptrst);
        }
        if (Pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
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
        // It is supposed that it does not depend on the Radius!
    }

    public override int NbIntervals(Continuity S)
    {
        return guide.NbIntervals(BlendFunc.NextShape(S));
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        guide.Intervals(T, BlendFunc.NextShape(S));
    }

    public override void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d)
    {
        NbPoles2d = 2;
        BlendFunc.GetShape(mySShape, maxang, out NbPoles, out NbKnots, out Degree, ref myTConv);
    }

    /// <summary>The tolerances to use in the approximations.</summary>
    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1d)
    {
        int low = Tol3d.Lower(), up = Tol3d.Upper();
        double Tol;
        Tol = GeomFill.GetTolerance(myTConv, minang, Math.Abs(ray), AngleTol, SurfTol);
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
        Vector3D d1u1, d1v1;
        Vector3D ns, ns2;
        Point3D Center = new();

        double norm, u1, v1, w;

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();

        tguide.D1(prm, out ptgui, out d1gui);
        nplan = d1gui.Normalized();

        P.ParametersOnS(out u1, out v1);
        w = P.ParameterOnC(); // jlr : point on curve not on surface
        Point2D pt2d = rst.Value(w);

        surf.D1(u1, v1, out pts, out d1u1, out d1v1);
        ptrst = cons.Value(w);
        distmin = Min(distmin, pts.Distance(ptrst));

        Poles2d[Poles2d.Lower()].SetCoord(u1, v1);
        Poles2d[Poles2d.Upper()].SetCoord(pt2d.X(), pt2d.Y());

        // Linear Case
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts;
            Poles[upp] = ptrst;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            return;
        }

        ns = d1u1.Crossed(d1v1);
        norm = nplan.Crossed(ns).Magnitude();

        ns.SetLinearForm(nplan.Dot(ns) / norm, nplan, -1.0 / norm, ns);

        Center.SetXYZ(pts.XYZ() + ray * ns.XYZ());

        ns2 = new Vector3D(Center, ptrst).Normalized();
        if (ray > 0)
        {
            ns.Reverse();
        }
        if (choix % 2 != 0)
        {
            nplan.Reverse();
        }

        GeomFill.GetCircle(myTConv, ns, ns2, nplan, pts, ptrst, Math.Abs(ray), Center, Poles, Weights);
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        Vector3D d1u1, d1v1, d2u1, d2v1, d2uv1, d1;
        Vector3D ns, ns2, dnplan = new(), dnw = new(), dn2w = new();
        Vector3D ncrossns;
        Vector3D resulu = new(), resulv = new(), temp = new(), tgct = new(), resul = new();
        Vector3D d1urst, d1vrst;
        Point3D Center = new(), bid;

        double norm, ndotns, grosterme;

        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt;

        tguide.D2(prm, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        P.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = prmrst = P.ParameterOnC();
        pt2drst = rst.Value(prmrst);

        Values(sol, valsol, gradsol);

        surf.D2(sol[1], sol[2], out pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        cons.D1(sol[3], out ptrst, out d1);

        temp.SetXYZ(pts.XYZ() - ptgui.XYZ());
        secmember[1] = normtg - dnplan.Dot(temp);

        temp.SetXYZ(ptrst.XYZ() - ptgui.XYZ());
        secmember[2] = normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = nplan.Crossed(ns);
        ndotns = nplan.Dot(ns);
        norm = ncrossns.Magnitude();
        if (norm < Eps)
        {
            norm = 1; // Not enough, but it is not necessary to stop
            // OCCT_DEBUG trace left out.
        }

        // Derivative of n1 corresponding to w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
        resul.SetLinearForm(ray, temp, new Vector3D(ptrst, pts));
        secmember[3] = dnw.Dot(resul);
        secmember[3] = -2.0 * ray * secmember[3];

        Gauss Resol = new(gradsol, 1.0e-9);

        if (Resol.IsDone)
        {
            istgt = false;
            Resol.Solve(secmember);
        }
        else
        {
            Svd SingRS = new(gradsol);
            if (SingRS.IsDone)
            {
                math_Vector DEDT = new(1, 3);
                DEDT.Initialized(secmember);
                SingRS.Solve(DEDT, secmember, 1.0e-6);
                istgt = false;
            }
            else
            {
                istgt = true;
            }
        }

        if (!istgt)
        {
            tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tgrst = secmember[3] * d1;

            // Derivative of n1 corresponding to u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulu.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivative of n1 corresponding to v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(nplan.Crossed(temp)) / norm / norm;
            resulv.SetLinearForm(-(grosterme * ndotns - nplan.Dot(temp)) / norm, nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw.SetLinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);

            dn2w.SetLinearForm(ray, dnw, -1.0, tgrst, tgs);
            norm = resul.Magnitude();
            dn2w.Divide(norm);
            ns2 = -resul.Normalized();
            dn2w.SetLinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);
        }
        else
        {
            ns.SetLinearForm(ndotns / norm, nplan, -1.0 / norm, ns);
            ns2 = -resul.Normalized();
        }

        // Tops 2D

        Poles2d[Poles2d.Lower()].SetCoord(sol[1], sol[2]);
        Poles2d[Poles2d.Upper()].SetCoord(pt2drst.X(), pt2drst.Y());
        if (!istgt)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(secmember[1], secmember[2]);
            surfrst.D1(pt2drst.X(), pt2drst.Y(), out bid, out d1urst, out d1vrst);
            double a, b;
            t3dto2d(out a, out b, tgrst, d1urst, d1vrst);
            DPoles2d[Poles2d.Upper()].SetCoord(a, b);
        }

        // Linear Case
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts;
            Poles[upp] = ptrst;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            if (!istgt)
            {
                DPoles[low] = tgs;
                DPoles[upp] = tgrst;
                DWeights[low] = 0.0;
                DWeights[upp] = 0.0;
            }
            return (!istgt);
        }

        // Case of the circle
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
            return GeomFill.GetCircle(myTConv, ns, ns2, dnw, dn2w, nplan, dnplan, pts, ptrst, tgs, tgrst,
                                      Math.Abs(ray), 0, Center, tgct, Poles, DPoles, Weights, DWeights);
        }
        else
        {
            GeomFill.GetCircle(myTConv, ns, ns2, nplan, pts, ptrst, Math.Abs(ray), Center, Poles, Weights);
            return false;
        }
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths,
                                 NCollection_Array1<double> D2Weigths)
    {
        return false;
    }

    public override void Resolution(int IC2d, double Tol, out double TolU, out double TolV)
    {
        if (IC2d == 1)
        {
            TolU = surf.UResolution(Tol);
            TolV = surf.VResolution(Tol);
        }
        else
        {
            TolU = surfrst.UResolution(Tol);
            TolV = surfrst.VResolution(Tol);
        }
    }
}
