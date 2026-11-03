// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Blend_DecrochStatus;
using static Occt.BlendFunc_SectionShape;

namespace Occt;

/// <summary>
/// The function of a constant-radius fillet between two restrictions (arcs of two surfaces) along a guide: the
/// variables are the parameters on the two restrictions.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
public class BRepBlend_RstRstConstRad : Blend_RstRstFunction
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase surf1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly Curve2DAdaptorBase rst1;
    private readonly Curve2DAdaptorBase rst2;
    private readonly CurveOnSurface cons1;
    private readonly CurveOnSurface cons2;
    private readonly CurveAdaptorBase guide;
    private CurveAdaptorBase tguide;
    private Point3D ptrst1;
    private Point3D ptrst2;
    private Point2D pt2drst1;
    private Point2D pt2drst2;
    private double prmrst1;
    private double prmrst2;
    private bool istangent;
    private Vector3D tgrst1;
    private Vector2D tg2drst1;
    private Vector3D tgrst2;
    private Vector2D tg2drst2;
    private double ray;
    private int choix;
    private Point3D ptgui;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D nplan;
    private double normtg;
    private double theD;
    private SurfaceAdaptorBase? surfref1;
    private Curve2DAdaptorBase? rstref1;
    private SurfaceAdaptorBase? surfref2;
    private Curve2DAdaptorBase? rstref2;
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

    public BRepBlend_RstRstConstRad(SurfaceAdaptorBase Surf1, Curve2DAdaptorBase Rst1, SurfaceAdaptorBase Surf2,
                                    Curve2DAdaptorBase Rst2, CurveAdaptorBase CGuide)
    {
        surf1 = Surf1;
        surf2 = Surf2;
        rst1 = Rst1;
        rst2 = Rst2;
        cons1 = new CurveOnSurface(Rst1, Surf1);
        cons2 = new CurveOnSurface(Rst2, Surf2);
        guide = CGuide;
        tguide = CGuide;
        prmrst1 = 0.0;
        prmrst2 = 0.0;
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
        return 2;
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        ptrst1 = cons1.Value(X[1]);
        ptrst2 = cons2.Value(X[2]);

        F[1] = nplan.XYZ().Dot(ptrst1.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(ptrst2.XYZ()) + theD;

        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D d11, d21;

        cons1.D1(X[1], out ptrst1, out d11);
        cons2.D1(X[2], out ptrst2, out d21);

        D[1, 1] = nplan.Dot(d11);
        D[1, 2] = 0.0;

        D[2, 1] = 0.0;
        D[2, 2] = nplan.Dot(d21);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Value(X, F);
        Derivatives(X, D);

        return true;
    }

    /// <summary>The reference surfaces and restrictions <see cref="Decroch"/> tests against.</summary>
    public void Set(SurfaceAdaptorBase SurfRef1, Curve2DAdaptorBase RstRef1, SurfaceAdaptorBase SurfRef2,
                    Curve2DAdaptorBase RstRef2)
    {
        surfref1 = SurfRef1;
        surfref2 = SurfRef2;
        rstref1 = RstRef1;
        rstref2 = RstRef2;
    }

    public override void Set(double Param)
    {
        tguide.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
    }

    public override void Set(double First, double Last)
    {
        tguide = guide.Trim(First, Last, 1.0e-12);
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = cons1.Resolution(Tol);
        Tolerance[2] = cons2.Resolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = cons1.FirstParameter();
        InfBound[2] = cons2.FirstParameter();
        SupBound[1] = cons1.LastParameter();
        SupBound[2] = cons2.LastParameter();
    }

    /// <summary>
    /// True if <paramref name="Sol"/> solves the function within <paramref name="Tol"/>; then computes the tangents
    /// and updates the extreme angles and the minimal distance of the sections (called once per accepted point).
    /// </summary>
    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        Vector3D dnplan = new(), d1urst1, d1vrst1, d1urst2, d1vrst2, d11, d21, temp = new();
        Point3D bid;

        double Cosa, Sina, Angle;

        Values(Sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol)
        {
            // Calculation of tangents
            prmrst1 = Sol[1];
            pt2drst1 = rst1.Value(prmrst1);
            prmrst2 = Sol[2];
            pt2drst2 = rst2.Value(prmrst2);

            cons1.D1(Sol[1], out ptrst1, out d11);
            cons2.D1(Sol[2], out ptrst2, out d21);

            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp.SetXYZ(ptrst1.XYZ() - ptgui.XYZ());
            secmember[1] = normtg - dnplan.Dot(temp);

            temp.SetXYZ(ptrst2.XYZ() - ptgui.XYZ());
            secmember[2] = normtg - dnplan.Dot(temp);

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
                    // OCCT: DEDT has 3 entries for this 2x2 system; native's release build (its dimension checks
                    // compiled out) copies secmember's 2 and solves on them, as here.
                    math_Vector DEDT = new(1, 2);
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
                tgrst1 = secmember[1] * d11;
                tgrst2 = secmember[2] * d21;

                double a, b;
                surf1.D1(pt2drst1.X(), pt2drst1.Y(), out bid, out d1urst1, out d1vrst1);
                t3dto2d(out a, out b, tgrst1, d1urst1, d1vrst1);
                tg2drst1.SetCoord(a, b);
                surf2.D1(pt2drst2.X(), pt2drst2.Y(), out bid, out d1urst2, out d1vrst2);
                t3dto2d(out a, out b, tgrst1, d1urst2, d1vrst2); // OCCT: tgrst1 where tgrst2 is meant
                tg2drst2.SetCoord(a, b);
            }

            Point3D Center = new();
            Vector3D NotUsed;
            bool IsCenter;

            IsCenter = CenterCircleRst1Rst2(ptrst1, ptrst2, nplan, ref Center, out NotUsed);

            if (!IsCenter)
            {
                return false;
            }

            Vector3D n1 = new(Center, ptrst1), n2 = new(Center, ptrst2);

            n1.Normalize();
            n2.Normalize();

            Cosa = n1.Dot(n2);
            Sina = nplan.Dot(n1.Crossed(n2));

            if (choix % 2 != 0)
            {
                Sina = -Sina; // nplan is changed into -nplan
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
            distmin = Min(distmin, ptrst1.Distance(ptrst2));

            return true;
        }
        istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return distmin;
    }

    public override Point3D PointOnRst1() => ptrst1;

    public override Point3D PointOnRst2() => ptrst2;

    public override Point2D Pnt2dOnRst1() => pt2drst1;

    public override Point2D Pnt2dOnRst2() => pt2drst2;

    public override double ParameterOnRst1() => prmrst1;

    public override double ParameterOnRst2() => prmrst2;

    public override bool IsTangencyPoint() => istangent;

    public override Vector3D TangentOnRst1()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tgrst1;
    }

    public override Vector2D Tangent2dOnRst1()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tg2drst1;
    }

    public override Vector3D TangentOnRst2()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tgrst2;
    }

    public override Vector2D Tangent2dOnRst2()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }
        return tg2drst2;
    }

    /// <summary>Which of the restrictions the section at <paramref name="Sol"/> leaves its reference surface on.</summary>
    public override Blend_DecrochStatus Decroch(math_Vector Sol, out Vector3D NRst1, out Vector3D TgRst1,
                                                out Vector3D NRst2, out Vector3D TgRst2)
    {
        Vector3D NRst1InPlane = new(), NRst2InPlane = new();
        Point3D PtTmp1, PtTmp2, Center = new();
        Vector3D d1u, d1v, centptrst = new(), NotUsed;
        double norm, unsurnorm;
        double u, v;

        rstref1!.Value(Sol[1]).Coord(out u, out v);
        surfref1!.D1(u, v, out PtTmp1, out d1u, out d1v);
        // Normal to the reference surface 1
        NRst1 = d1u.Crossed(d1v);
        rstref2!.Value(Sol[2]).Coord(out u, out v);
        surfref2!.D1(u, v, out PtTmp2, out d1u, out d1v);
        // Normal to the reference surface 2
        NRst2 = d1u.Crossed(d1v);

        CenterCircleRst1Rst2(PtTmp1, PtTmp2, nplan, ref Center, out NotUsed);

        norm = nplan.Crossed(NRst1).Magnitude();
        unsurnorm = 1.0 / norm;

        NRst1InPlane.SetLinearForm(nplan.Dot(NRst1) * unsurnorm, nplan, -unsurnorm, NRst1);

        centptrst.SetXYZ(PtTmp1.XYZ() - Center.XYZ());

        if (centptrst.Dot(NRst1InPlane) < 0.0)
        {
            NRst1InPlane.Reverse();
        }

        TgRst1 = nplan.Crossed(centptrst);

        norm = nplan.Crossed(NRst2).Magnitude();
        unsurnorm = 1.0 / norm;
        NRst2InPlane.SetLinearForm(nplan.Dot(NRst2) * unsurnorm, nplan, -unsurnorm, NRst2);
        centptrst.SetXYZ(PtTmp2.XYZ() - Center.XYZ());

        if (centptrst.Dot(NRst2InPlane) < 0.0)
        {
            NRst2InPlane.Reverse();
        }

        TgRst2 = nplan.Crossed(centptrst);

        if (choix % 2 != 0)
        {
            TgRst1.Reverse();
            TgRst2.Reverse();
        }

        // The vectors are returned
        if (NRst1InPlane.Dot(TgRst1) > -1.0e-10)
        {
            if (NRst2InPlane.Dot(TgRst2) < 1.0e-10)
            {
                return Blend_DecrochBoth;
            }
            else
            {
                return Blend_DecrochRst1;
            }
        }
        else
        {
            if (NRst2InPlane.Dot(TgRst2) < 1.0e-10)
            {
                return Blend_DecrochRst2;
            }
            else
            {
                return Blend_NoDecroch;
            }
        }
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="Choix"/> (1 to 4) in which the fillet lies.</summary>
    public void Set(double Radius, int Choix)
    {
        choix = Choix;
        ray = Math.Abs(Radius);
    }

    /// <summary>Sets the type of section generation for the approximations.</summary>
    public void Set(BlendFunc_SectionShape TypeSection)
    {
        mySShape = TypeSection;
    }

    /// <summary>
    /// The center of the circle of radius the fillet's through the two points, in the plane of normal
    /// <paramref name="np"/>: false if they are too far apart. <paramref name="Center"/> is written only on success;
    /// <paramref name="VdMed"/> is the direction of the perpendicular bisector.
    /// </summary>
    public bool CenterCircleRst1Rst2(in Point3D PtRst1, in Point3D PtRst2, in Vector3D np, ref Point3D Center,
                                     out Vector3D VdMed)
    {
        Vector3D rst1rst2 = new(PtRst1, PtRst2);
        Vector3D vdmedNor; // directing vector of the perpendicular bisector
        double norm2;
        double Dist; // distance between the middle of PtRst1,PtRst2 and Center

        // Calculate the center of the circle
        VdMed = rst1rst2.Crossed(np);
        norm2 = rst1rst2.SquareMagnitude();
        Dist = ray * ray - 0.25 * norm2;

        if (choix > 2)
        {
            VdMed.Reverse();
        }

        if (Dist < -1.0E-07)
        {
            return false;
        }

        if (Dist > 1.0E-07)
        {
            Dist = Math.Sqrt(Dist);
            vdmedNor = VdMed.Normalized();
            Center.SetXYZ(0.5 * rst1rst2.XYZ() + PtRst1.XYZ() + Dist * vdmedNor.XYZ());
        }
        else
        {
            Center.SetXYZ(0.5 * rst1rst2.XYZ() + PtRst1.XYZ());
        }

        return true;
    }

    /// <summary>The circular section at <paramref name="Param"/> through the points U and V, and its bounds.</summary>
    public void Section(double Param, double U, double V, out double Pdeb, out double Pfin, ref Circle3D C)
    {
        Vector3D ns, np, NotUsed;
        Point3D Center = new();

        tguide.D1(Param, out ptgui, out d1gui);
        np = d1gui.Normalized();
        ptrst1 = cons1.Value(U);
        ptrst2 = cons2.Value(V);

        CenterCircleRst1Rst2(ptrst1, ptrst2, np, ref Center, out NotUsed);

        C.SetRadius(Math.Abs(ray));
        ns = new Vector3D(Center, ptrst1).Normalized();

        if (choix % 2 != 0)
        {
            np.Reverse();
        }

        C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns)));
        Pdeb = 0; // ElCLib::Parameter(C, pts);
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, ptrst2);

        // Test of angles negative and almost null : Special Case
        if (Pfin > 1.5 * M_PI)
        {
            np.Reverse();
            C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns)));
            Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, ptrst2);
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
        Vector3D ns, ns2, NotUsed;
        Point3D Center = new();
        double u, v;

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();

        tguide.D1(prm, out ptgui, out d1gui);
        nplan = d1gui.Normalized();

        u = P.ParameterOnC1();
        v = P.ParameterOnC2();

        Point2D pt2d1 = rst1.Value(u);
        Point2D pt2d2 = rst2.Value(v);

        ptrst1 = cons1.Value(u);
        ptrst2 = cons2.Value(v);
        distmin = Min(distmin, ptrst1.Distance(ptrst2));

        Poles2d[Poles2d.Lower()].SetCoord(pt2d1.X(), pt2d1.Y());
        Poles2d[Poles2d.Upper()].SetCoord(pt2d2.X(), pt2d2.Y());

        // Linear case
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = ptrst1;
            Poles[upp] = ptrst2;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            return;
        }

        // Calculate the center of the circle
        CenterCircleRst1Rst2(ptrst1, ptrst2, nplan, ref Center, out NotUsed);

        // normals to the section with points
        ns = new Vector3D(Center, ptrst1).Normalized();
        ns2 = new Vector3D(Center, ptrst2).Normalized();

        if (choix % 2 != 0)
        {
            nplan.Reverse();
        }

        GeomFill.GetCircle(myTConv, ns, ns2, nplan, ptrst1, ptrst2, Math.Abs(ray), Center, Poles, Weights);
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        Vector3D d11, d21;
        Vector3D dnplan = new(), d1n1 = new(), d1n2 = new();
        Vector3D temp = new(), tgct = new();
        Vector3D d1urst, d1vrst;
        Point3D Center = new(), NotUsed;

        double norm2, normmed, Dist;

        math_Vector sol = new(1, 2), valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt;

        tguide.D2(prm, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        sol[1] = prmrst1 = P.ParameterOnC1();
        sol[2] = prmrst2 = P.ParameterOnC2();
        pt2drst1 = rst1.Value(prmrst1);
        pt2drst2 = rst2.Value(prmrst2);

        Values(sol, valsol, gradsol);

        cons1.D1(sol[1], out ptrst1, out d11);
        cons2.D1(sol[2], out ptrst2, out d21);

        temp.SetXYZ(ptrst1.XYZ() - ptgui.XYZ());
        secmember[1] = normtg - dnplan.Dot(temp);

        temp.SetXYZ(ptrst2.XYZ() - ptgui.XYZ());
        secmember[2] = normtg - dnplan.Dot(temp);

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
                math_Vector DEDT = new(1, 2);
                DEDT.Initialized(secmember);
                SingRS.Solve(DEDT, secmember, 1.0e-6);
                istgt = false;
            }
            else
            {
                istgt = true;
            }
        }

        Vector3D med;
        Vector3D rst1rst2 = new(ptrst1, ptrst2);
        bool IsCenter;

        IsCenter = CenterCircleRst1Rst2(ptrst1, ptrst2, nplan, ref Center, out med);
        if (!IsCenter)
        {
            return false;
        }

        normmed = med.Magnitude();
        med.Normalize();
        Vector3D n1 = new(Center, ptrst1), n2 = new(Center, ptrst2);
        n1.Normalize();
        n2.Normalize();

        if (!istgt)
        {
            // secmember contains derivatives of parameters on curves compared to t
            tgrst1 = secmember[1] * d11;
            tgrst2 = secmember[2] * d21;

            Vector3D d1rst1rst2;

            norm2 = rst1rst2.SquareMagnitude();
            d1rst1rst2 = tgrst2 - tgrst1;
            Dist = ray * ray - 0.25 * norm2;

            if (Dist > 1.0E-07)
            {
                Vector3D d1P1P2CrosNp, dmed;
                d1P1P2CrosNp = d1rst1rst2.Crossed(nplan) + rst1rst2.Crossed(dnplan);
                // derivative of the perpendicular bisector
                dmed = d1P1P2CrosNp - med.Dot(d1P1P2CrosNp) * med;
                dmed /= normmed;
                Dist = Math.Sqrt(Dist);
                double d1Dist = -(0.25 / Dist) * rst1rst2.Dot(d1rst1rst2);

                if (choix > 2)
                {
                    dmed.Reverse();
                }

                // the derivative of coefficient Dist is located in dmed
                dmed.SetLinearForm(Dist, dmed, d1Dist, med);
                d1rst1rst2 *= 0.5;
                // derivative of the Normal to the curve in P1
                d1n1 = -(dmed + d1rst1rst2) / ray;

                // derivative of the Normal to the curve in P2
                d1n2 = (d1rst1rst2 - dmed) / ray;
            }
            else
            {
                d1rst1rst2 *= 0.5;
                // Normal to the curve in P1
                d1n1 = -d1rst1rst2 / ray;

                // Normal to the curve in P2
                d1n2 = d1rst1rst2 / ray;
            }
        }

        // Tops 2d

        Poles2d[Poles2d.Lower()].SetCoord(pt2drst1.X(), pt2drst1.Y());
        Poles2d[Poles2d.Upper()].SetCoord(pt2drst2.X(), pt2drst2.Y());
        if (!istgt)
        {
            double a, b;
            surf1.D1(pt2drst1.X(), pt2drst1.Y(), out NotUsed, out d1urst, out d1vrst);
            t3dto2d(out a, out b, tgrst1, d1urst, d1vrst);
            DPoles2d[Poles2d.Lower()].SetCoord(a, b);

            surf2.D1(pt2drst2.X(), pt2drst2.Y(), out NotUsed, out d1urst, out d1vrst);
            t3dto2d(out a, out b, tgrst2, d1urst, d1vrst);
            DPoles2d[Poles2d.Upper()].SetCoord(a, b);
        }

        // Linear case
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = ptrst1;
            Poles[upp] = ptrst2;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            if (!istgt)
            {
                DPoles[low] = tgrst1;
                DPoles[upp] = tgrst2;
                DWeights[low] = 0.0;
                DWeights[upp] = 0.0;
            }
            return (!istgt);
        }

        // Case of the circle
        // tangent to the center of the circle
        if (!istgt)
        {
            tgct.SetLinearForm(-ray, d1n1, tgrst1);
        }

        if (choix % 2 != 0)
        {
            nplan.Reverse();
            dnplan.Reverse();
        }

        if (!istgt)
        {
            return GeomFill.GetCircle(myTConv, n1, n2, d1n1, d1n2, nplan, dnplan, ptrst1, ptrst2, tgrst1, tgrst2,
                                      Math.Abs(ray), 0, Center, tgct, Poles, DPoles, Weights, DWeights);
        }
        else
        {
            GeomFill.GetCircle(myTConv, n1, n2, nplan, ptrst1, ptrst2, Math.Abs(ray), Center, Poles, Weights);
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
