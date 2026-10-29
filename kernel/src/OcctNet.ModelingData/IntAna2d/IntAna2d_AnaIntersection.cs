// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_1.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_2.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_3.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_4.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_5.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_6.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_7.cxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_AnaIntersection_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna2d_Outils;

namespace Occt;

/// <summary>
/// Analytic intersection of two 2D curves: two lines, two circles, a line and a circle, or a line, circle,
/// ellipse, parabola or hyperbola with a conic given by its implicit equation.
/// </summary>
/// <remarks>An algorithm object: construct it with the curves, or construct it empty and call <c>Perform</c>.</remarks>
public class IntAna2d_AnaIntersection
{
    private bool done;
    private bool para;
    private bool iden;
#pragma warning disable CS0414 // OCCT sets empt everywhere but never reads it (IsEmpty is computed from nbp and iden)
    private bool empt;
#pragma warning restore CS0414
    private int nbp;
    private readonly IntAna2d_IntPoint[] lpnt = [new(), new(), new(), new()];

    /// <summary>Empty constructor. IsDone returns false.</summary>
    public IntAna2d_AnaIntersection()
    {
        done = false;
        para = false;
        iden = false;
        empt = true;
        nbp = 0;
    }

    /// <summary>Intersection between two lines.</summary>
    public IntAna2d_AnaIntersection(in gp_Lin2d L1, in gp_Lin2d L2) => Perform(L1, L2);

    /// <summary>Intersection between two circles.</summary>
    public IntAna2d_AnaIntersection(in gp_Circ2d C1, in gp_Circ2d C2) => Perform(C1, C2);

    /// <summary>Intersection between a line and a circle.</summary>
    public IntAna2d_AnaIntersection(in gp_Lin2d L, in gp_Circ2d C) => Perform(L, C);

    /// <summary>Intersection between a line and a conic.</summary>
    public IntAna2d_AnaIntersection(in gp_Lin2d L, in IntAna2d_Conic Conic) => Perform(L, Conic);

    /// <summary>Intersection between a parabola and another conic.</summary>
    public IntAna2d_AnaIntersection(in gp_Parab2d P, in IntAna2d_Conic Conic) => Perform(P, Conic);

    /// <summary>Intersection between a circle and another conic.</summary>
    public IntAna2d_AnaIntersection(in gp_Circ2d C, in IntAna2d_Conic Conic) => Perform(C, Conic);

    /// <summary>Intersection between an ellipse and another conic.</summary>
    public IntAna2d_AnaIntersection(in gp_Elips2d E, in IntAna2d_Conic Conic) => Perform(E, Conic);

    /// <summary>Intersection between a hyperbola and another conic.</summary>
    public IntAna2d_AnaIntersection(in gp_Hypr2d E, in IntAna2d_Conic Conic) => Perform(E, Conic);

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>
    /// True when there is no intersection: no intersection point and the elements are not identical. The elements
    /// may be parallel in this case.
    /// </summary>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return ((nbp == 0) && (!iden));
    }

    /// <summary>
    /// For the intersection between an element of gp and a conic known by an implicit equation, true if the element
    /// of gp verifies the implicit equation. For two Lin2d or two Circ2d, true if the elements are identical.
    /// False in all the other cases.
    /// </summary>
    public bool IdenticalElements()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return iden;
    }

    /// <summary>For the intersection between two Lin2d or two Circ2d, true if the elements are parallel.</summary>
    public bool ParallelElements()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return para;
    }

    /// <summary>The number of intersection points between the two curves.</summary>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return nbp;
    }

    /// <summary>The intersection point of range N.</summary>
    /// <exception cref="Standard_OutOfRange">N is not in [1, NbPoints].</exception>
    public IntAna2d_IntPoint Point(int N)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            if ((N <= 0) || (N > nbp))
            {
                throw new Standard_OutOfRange();
            }
            else
            {
                return lpnt[N - 1];
            }
        }
    }

    // IntAna2d_AnaIntersection_1.cxx

    /// <summary>Intersection between two lines.</summary>
    public void Perform(in gp_Lin2d L1, in gp_Lin2d L2)
    {
        done = false;

        double A1, B1, C1;
        double A2, B2, C2;
        L1.Coefficients(out A1, out B1, out C1);
        L2.Coefficients(out A2, out B2, out C2);

        double al1, be1, ga1;
        double al2, be2, ga2;

        double Det = Max(Math.Abs(A1), Max(Math.Abs(A2), Max(Math.Abs(B1), Math.Abs(B2))));

        if (Math.Abs(A1) == Det)
        {
            al1 = A1;
            be1 = B1;
            ga1 = C1;
            al2 = A2;
            be2 = B2;
            ga2 = C2;
        }
        else if (Math.Abs(B1) == Det)
        {
            al1 = B1;
            be1 = A1;
            ga1 = C1;
            al2 = B2;
            be2 = A2;
            ga2 = C2;
        }
        else if (Math.Abs(A2) == Det)
        {
            al1 = A2;
            be1 = B2;
            ga1 = C2;
            al2 = A1;
            be2 = B1;
            ga2 = C1;
        }
        else
        {
            al1 = B2;
            be1 = A2;
            ga1 = C2;
            al2 = B1;
            be2 = A1;
            ga2 = C1;
        }

        double rap = al2 / al1;
        double denom = be2 - rap * be1;

        if (Math.Abs(denom) <= RealEpsilon())
        { // Coincident directions
            para = true;
            nbp = 0;
            if (Math.Abs(ga2 - rap * ga1) <= RealEpsilon())
            { // Coincident lines
                iden = true;
                empt = false;
            }
            else
            { // Parallel lines
                iden = false;
                empt = true;
            }
        }
        else
        {
            para = false;
            iden = false;
            empt = false;
            nbp = 1;
            double XS = (be1 * ga2 / al1 - be2 * ga1 / al1) / denom;
            double YS = (rap * ga1 - ga2) / denom;

            if (((Math.Abs(A1) != Det) && (Math.Abs(B1) == Det))
                || ((Math.Abs(A1) != Det) && (Math.Abs(B1) != Det) && (Math.Abs(A2) != Det)))
            {
                double temp = XS;
                XS = YS;
                YS = temp;
            }

            double La, Mu;
            if (Math.Abs(A1) >= Math.Abs(B1))
            {
                La = (YS - L1.Location().Y()) / A1;
            }
            else
            {
                La = (L1.Location().X() - XS) / B1;
            }
            if (Math.Abs(A2) >= Math.Abs(B2))
            {
                Mu = (YS - L2.Location().Y()) / A2;
            }
            else
            {
                Mu = (L2.Location().X() - XS) / B2;
            }
            lpnt[0].SetValue(XS, YS, La, Mu);
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_2.cxx

    /// <summary>Intersection between two circles.</summary>
    public void Perform(in gp_Circ2d C1, in gp_Circ2d C2)
    {
        // OCCT: the parameters are angles from each X axis counterclockwise, wrong on an indirect circle
        done = false;
        double d = C1.Location().Distance(C2.Location());
        double R1 = C1.Radius();
        double R2 = C2.Radius();
        double sum = R1 + R2;
        double dif = Math.Abs(R1 - R2);

        if (d <= RealEpsilon())
        { // Concentric circles
            para = true;
            nbp = 0;
            if (dif <= RealEpsilon())
            { // Coincident circles
                empt = false;
                iden = true;
            }
            else
            { // Parallel circles
                empt = true;
                iden = false;
            }
        }
        else if ((d - sum) > Epsilon(sum))
        { // Circles exterior to each other
            // et No solution
            empt = true;
            para = false;
            iden = false;
            nbp = 0;
        }
        else if (Math.Abs(d - sum) <= Epsilon(sum))
        { // Externally tangent circles
            empt = false;
            para = false;
            iden = false;
            nbp = 1;
            gp_Vec2d ax = new(C1.Location(), C2.Location());
            gp_Vec2d Ox1 = new(C1.XAxis().Direction());
            gp_Vec2d Ox2 = new(C2.XAxis().Direction());

            double XS = (C1.Location().X() * R2 + C2.Location().X() * R1) / sum;
            double YS = (C1.Location().Y() * R2 + C2.Location().Y() * R1) / sum;
            double ang1 = Ox1.Angle(ax); // Result between -PI and +PI
            double ang2 = Ox2.Angle(ax) + M_PI;
            if (ang1 < 0)
            {
                ang1 = 2 * M_PI + ang1;
            } // Normalize to range [0, 2PI]
            lpnt[0].SetValue(XS, YS, ang1, ang2);
        }
        else if (((sum - d) > Epsilon(sum)) && ((d - dif) > Epsilon(d + dif)))
        {
            empt = false;
            para = false;
            iden = false;
            nbp = 2;
            gp_Vec2d ax = new(C1.Location(), C2.Location());
            gp_Vec2d Ox1 = new(C1.XAxis().Direction());
            gp_Vec2d Ox2 = new(C2.XAxis().Direction());
            double ref1 = Ox1.Angle(ax); // Result between -PI and +PI
            double ref2 = Ox2.Angle(ax); // Result between -PI and +PI

            double l1 = (d * d + R1 * R1 - R2 * R2) / (2.0 * d);
            if (R1 * R1 - l1 * l1 < 0.0)
            {
                l1 = (l1 > 0 ? R1 : -R1);
            }
            double h = Math.Sqrt(R1 * R1 - l1 * l1);

            double XS1 = C1.Location().X() + l1 * ax.X() / d - h * ax.Y() / d;
            double YS1 = C1.Location().Y() + l1 * ax.Y() / d + h * ax.X() / d;

            double XS2 = C1.Location().X() + l1 * ax.X() / d + h * ax.Y() / d;
            double YS2 = C1.Location().Y() + l1 * ax.Y() / d - h * ax.X() / d;

            double sint1 = h / R1;
            double cost1 = l1 / R1;

            double sint2 = h / R2;
            double cost2 = (l1 - d) / R2;

            double ang1, ang2;

            // ang1 and ang2 correspond to the solutions with positive sine
            // when the reference axis is the axis of centers C1C2.
            // We use arccos between pi/2 and 3pi/2, arcsin otherwise.

            if (Math.Abs(cost1) <= 0.707)
            {
                ang1 = Math.Acos(cost1);
            }
            else
            {
                ang1 = Math.Asin(sint1);
                if (cost1 < 0.0)
                {
                    ang1 = M_PI - ang1;
                }
            }
            if (Math.Abs(cost2) <= 0.707)
            {
                ang2 = Math.Acos(cost2);
            }
            else
            {
                ang2 = Math.Asin(sint2);
                if (cost2 < 0.0)
                {
                    ang2 = M_PI - ang2;
                }
            }
            double ang11 = ref1 + ang1;
            double ang21 = ref2 + ang2;
            double ang12 = ref1 - ang1;
            double ang22 = ref2 - ang2;
            if (ang11 < 0.0)
            {
                ang11 = 2 * M_PI + ang11;
            }
            else if (ang11 >= 2 * M_PI)
            {
                ang11 = ang11 - 2 * M_PI;
            }
            if (ang21 < 0.0)
            {
                ang21 = 2 * M_PI + ang21;
            }
            else if (ang21 >= 2 * M_PI)
            {
                ang21 = ang21 - 2 * M_PI;
            }
            if (ang12 < 0.0)
            {
                ang12 = 2 * M_PI + ang12;
            }
            else if (ang12 >= 2 * M_PI)
            {
                ang12 = ang12 - 2 * M_PI;
            }
            if (ang22 < 0.0)
            {
                ang22 = 2 * M_PI + ang22;
            }
            else if (ang22 >= 2 * M_PI)
            {
                ang22 = ang22 - 2 * M_PI;
            }
            lpnt[0].SetValue(XS1, YS1, ang11, ang21);
            lpnt[1].SetValue(XS2, YS2, ang12, ang22);
        }
        else if (Math.Abs(d - dif) <= Epsilon(sum))
        { // Internally tangent circles
            empt = false;
            para = false;
            iden = false;
            nbp = 1;
            gp_Vec2d ax = new(C1.Location(), C2.Location());
            if (C1.Radius() < C2.Radius())
            {
                ax.Reverse();
            }

            gp_Vec2d Ox1 = new(C1.XAxis().Direction());
            gp_Vec2d Ox2 = new(C2.XAxis().Direction());
            double ang1 = Ox1.Angle(ax); // Result between -PI and +PI
            double ang2 = Ox2.Angle(ax);
            if (ang1 < 0)
            {
                ang1 = 2 * M_PI + ang1;
            } // Normalize to range [0, 2PI]
            if (ang2 < 0)
            {
                ang2 = 2 * M_PI + ang2;
            } // Normalize to range [0, 2PI]
            double XS = (C1.Location().X() * R2 - C2.Location().X() * R1) / (R2 - R1);
            double YS = (C1.Location().Y() * R2 - C2.Location().Y() * R1) / (R2 - R1);
            lpnt[0].SetValue(XS, YS, ang1, ang2);
        }
        else
        { // We must have d<dif-Resol and d<>0, therefore
            // one circle inside the other and no solution
            empt = true;
            para = false;
            iden = false;
            nbp = 0;
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_3.cxx

    /// <summary>Intersection between a line and a circle.</summary>
    public void Perform(in gp_Lin2d L, in gp_Circ2d C)
    {
        done = false;

        iden = false;
        para = false;
        //
        double A, B, C0, d;
        gp_Pnt2d aP2D, aP2D1, aP2D2;
        //
        L.Coefficients(out A, out B, out C0);
        d = A * C.Location().X() + B * C.Location().Y() + C0;

        if (Math.Abs(d) - C.Radius() > Epsilon(C.Radius()))
        {
            empt = true;
            nbp = 0;
        }
        else
        { // At least 1 solution
            empt = false;
            if (Math.Abs(Math.Abs(d) - C.Radius()) <= Epsilon(C.Radius()))
            { // Tangency case
                double u, XS, YS, ang;
                //
                nbp = 1;
                XS = C.Location().X() - d * A;
                YS = C.Location().Y() - d * B;
                //
                aP2D = new gp_Pnt2d(XS, YS);
                u = ElCLib.Parameter(L, aP2D);
                ang = ElCLib.Parameter(C, aP2D);
                lpnt[0].SetValue(XS, YS, u, ang);
            }
            else
            { // 2 intersection points
                double h, XS1, YS1, XS2, YS2, ang1, ang2, u1, u2;
                nbp = 2;
                h = Math.Sqrt(C.Radius() * C.Radius() - d * d);
                XS1 = C.Location().X() - d * A - h * B;
                YS1 = C.Location().Y() - d * B + h * A;
                XS2 = C.Location().X() - d * A + h * B;
                YS2 = C.Location().Y() - d * B - h * A;
                //
                aP2D1 = new gp_Pnt2d(XS1, YS1);
                aP2D2 = new gp_Pnt2d(XS2, YS2);
                u1 = ElCLib.Parameter(L, aP2D1);
                u2 = ElCLib.Parameter(L, aP2D2);
                ang1 = ElCLib.Parameter(C, aP2D1);
                ang2 = ElCLib.Parameter(C, aP2D2);

                lpnt[0].SetValue(XS1, YS1, u1, ang1);
                lpnt[1].SetValue(XS2, YS2, u2, ang2);
            }
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_4.cxx

    /// <summary>Intersection between a line and a conic.</summary>
    public void Perform(in gp_Lin2d L, in IntAna2d_Conic Conic)
    {
        double A, B, C, D, E, F;
        double px0, px1, px2;
        double DR_A, DR_B, DR_C, X0, Y0;
        int i;
        double tx, ty, S;

        done = false;
        nbp = 0;
        para = false;
        iden = false;

        Conic.Coefficients(out A, out B, out C, out D, out E, out F);
        L.Coefficients(out DR_A, out DR_B, out DR_C);
        X0 = L.Location().X();
        Y0 = L.Location().Y();

        // Parameter: L
        // X = Xo - L*DR_B  and  Y = Yo + L*DR_A

        px0 = F + X0 * (D + D + A * X0 + 2.0 * C * Y0) + Y0 * (E + E + B * Y0);
        px1 = 2.0 * (E * DR_A - D * DR_B + X0 * (C * DR_A - A * DR_B) + Y0 * (B * DR_A - C * DR_B));
        px2 = DR_A * (B * DR_A - 2.0 * C * DR_B) + A * (DR_B * DR_B);

        MyDirectPolynomialRoots Sol = new(px2, px1, px0);

        if (!Sol.IsDone())
        {
            done = false;
            return;
        }
        else
        {
            if (Sol.InfiniteRoots())
            {
                iden = true;
                done = true;
                return;
            }
            nbp = Sol.NbSolutions();
            for (i = 1; i <= nbp; i++)
            {
                S = Sol.Value(i);
                tx = X0 - S * DR_B;
                ty = Y0 + S * DR_A;
                lpnt[i - 1].SetValue(tx, ty, S);
            }
            Traitement_Points_Confondus(ref nbp, lpnt);
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_5.cxx

    /// <summary>Intersection between a circle and another conic.</summary>
    public void Perform(in gp_Circ2d Circle, in IntAna2d_Conic Conic)
    {
        bool CIsDirect = Circle.IsDirect();
        double A, B, C, D, E, F;
        double pcc, pss, p2sc, pc, ps, pcte;
        double radius = Circle.Radius();
        double radius_P2 = radius * radius;
        int i;
        double tx, ty, S;

        done = false;
        nbp = 0;
        para = false;
        empt = false;
        iden = false;

        gp_Ax2d Axe_rep = Circle.XAxis();

        Conic.Coefficients(out A, out B, out C, out D, out E, out F);
        Conic.NewCoefficients(ref A, ref B, ref C, ref D, ref E, ref F, Axe_rep);

        // Parameter a with x=Radius*cos(a) and y=Radius*sin(a)

        pss = B * radius_P2;
        pcc = A * radius_P2 - pss; // COS ^2
        p2sc = C * radius_P2; // 2 SIN COS
        pc = 2.0 * D * radius; // COS
        ps = 2.0 * E * radius; // SIN
        pcte = F + pss; // 1

        math_TrigonometricFunctionRoots Sol = new(pcc, p2sc, pc, ps, pcte, 0.0, 2.0 * M_PI);

        if (!Sol.IsDone())
        {
            // std::cout of "math_TrigonometricFunctionRoots -> NotDone" left out
            done = false;
            return;
        }
        else
        {
            if (Sol.InfiniteRoots())
            {
                iden = true;
                done = true;
                return;
            }
            nbp = Sol.NbSolutions();
            for (i = 1; i <= nbp; i++)
            {
                S = Sol.Value(i);
                tx = radius * Math.Cos(S);
                ty = radius * Math.Sin(S);
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!CIsDirect)
                {
                    S = M_PI + M_PI - S;
                }
                lpnt[i - 1].SetValue(tx, ty, S);
            }
            Traitement_Points_Confondus(ref nbp, lpnt);
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_6.cxx

    /// <summary>Intersection between an ellipse and another conic.</summary>
    public void Perform(in gp_Elips2d Elips, in IntAna2d_Conic Conic)
    {
        bool EIsDirect = Elips.IsDirect();
        double A, B, C, D, E, F;
        double pcte, ps, pc, p2sc, pcc, pss;
        double minor_radius = Elips.MinorRadius();
        double major_radius = Elips.MajorRadius();
        int i;
        double tx, ty, S;

        done = false;
        nbp = 0;
        para = false;
        iden = false;
        empt = false;

        gp_Ax2d Axe_rep = Elips.XAxis();

        Conic.Coefficients(out A, out B, out C, out D, out E, out F);
        Conic.NewCoefficients(ref A, ref B, ref C, ref D, ref E, ref F, Axe_rep);

        // Parameter: a with x=MajorRadius*cos(a) and y=MinorRadius*sin(a)

        pss = B * minor_radius * minor_radius; // SIN ^2
        pcc = A * major_radius * major_radius - pss; // COS ^2
        p2sc = C * major_radius * minor_radius; // 2 SIN COS
        pc = 2.0 * D * major_radius; // COS
        ps = 2.0 * E * minor_radius; // SIN
        pcte = F + pss; // 1

        math_TrigonometricFunctionRoots Sol = new(pcc, p2sc, pc, ps, pcte, 0.0, 2.0 * M_PI);

        if (!Sol.IsDone())
        {
            done = false;
            return;
        }
        else
        {
            if (Sol.InfiniteRoots())
            {
                iden = true;
                done = true;
                return;
            }
            nbp = Sol.NbSolutions();
            for (i = 1; i <= nbp; i++)
            {
                S = Sol.Value(i);
                tx = major_radius * Math.Cos(S);
                ty = minor_radius * Math.Sin(S);
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!EIsDirect)
                {
                    S = M_PI + M_PI - S;
                }
                lpnt[i - 1].SetValue(tx, ty, S);
            }
            Traitement_Points_Confondus(ref nbp, lpnt);
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_7.cxx

    /// <summary>Intersection between a parabola and another conic.</summary>
    public void Perform(in gp_Parab2d P, in IntAna2d_Conic Conic)
    {
        bool PIsDirect = P.IsDirect();
        double A, B, C, D, E, F;
        double px4, px3, px2, px1, px0;
        int i;
        double tx, ty, S;
        double un_sur_2p = 0.5 / (P.Parameter());
        gp_Ax2d Axe_rep = P.MirrorAxis();

        done = false;
        nbp = 0;
        para = false;
        empt = false;
        iden = false;

        Conic.Coefficients(out A, out B, out C, out D, out E, out F);
        Conic.NewCoefficients(ref A, ref B, ref C, ref D, ref E, ref F, Axe_rep);

        //-------- Parameter y with y=y, x=y^2/(2p)

        px0 = F;
        px1 = E + E;
        px2 = B + un_sur_2p * (D + D);
        px3 = (C + C) * un_sur_2p;
        px4 = A * (un_sur_2p * un_sur_2p);

        MyDirectPolynomialRoots Sol = new(px4, px3, px2, px1, px0);

        if (!Sol.IsDone())
        {
            done = false; // OCCT: no return, so done ends true (the hyperbola case returns not done)
        }
        else
        {
            if (Sol.InfiniteRoots())
            {
                iden = true;
                done = true;
            }
            nbp = Sol.NbSolutions();
            for (i = 1; i <= nbp; i++)
            {
                S = Sol.Value(i);
                tx = un_sur_2p * S * S;
                ty = S;
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!PIsDirect)
                {
                    S = -S;
                }
                lpnt[i - 1].SetValue(tx, ty, S);
            }
            Traitement_Points_Confondus(ref nbp, lpnt);
        }
        done = true;
    }

    // IntAna2d_AnaIntersection_8.cxx

    // -----------------------------------------------------------------
    // ------ Verification of the validity of obtained points ----------
    // --- Method to implement in other routines if numerical
    // --- instability problems are observed in:
    // ---      * polynomial construction in t (t: parameter)
    // ---      * polynomial solving
    // ---      * reverse mapping: parameter t -> intersection point
    // --- Problem: From which tolerance is a point no longer
    // ---          a point on the curve? (f(x,y)=1e-10 ??)
    // ---          gives no info on the distance from pt to curve
    // -----------------------------------------------------------------
    // ------ Method not implemented for other Intersections
    // --- If a problem is observed: Duplicate the code between
    // --- the VERIF-VALID comments
    // -----------------------------------------------------------------

    /// <summary>Intersection between a hyperbola and another conic.</summary>
    public void Perform(in gp_Hypr2d H, in IntAna2d_Conic Conic)
    {
        bool HIsDirect = H.IsDirect();
        double A, B, C, D, E, F;
        double px0, px1, px2, px3, px4;
        double minor_radius = H.MinorRadius();
        double major_radius = H.MajorRadius();
        int i;
        double tx, ty, S;

        done = false;
        nbp = 0;
        para = false;
        iden = false;
        empt = false;

        gp_Ax2d Axe_rep = H.XAxis();
        Conic.Coefficients(out A, out B, out C, out D, out E, out F);
        Conic.NewCoefficients(ref A, ref B, ref C, ref D, ref E, ref F, Axe_rep);

        double A_major_radiusP2 = A * major_radius * major_radius;
        double B_minor_radiusP2 = B * minor_radius * minor_radius;
        double C_2_major_minor_radius = C * 2.0 * major_radius * minor_radius;

        // Parameter: t with x=MajorRadius*Ch(t)  y=minorRadius*Sh(t)
        // The polynomial is rewritten in Exp(t)
        // The following are the coefficients of polynomial P multiplied by 4*Exp(t)^2

        px0 = A_major_radiusP2 - C_2_major_minor_radius + B_minor_radiusP2;
        px1 = 4.0 * (D * major_radius - E * minor_radius);
        px2 = 2.0 * (A_major_radiusP2 + 2.0 * F - B_minor_radiusP2);
        px3 = 4.0 * (D * major_radius + E * minor_radius);
        px4 = A_major_radiusP2 + C_2_major_minor_radius + B_minor_radiusP2;

        MyDirectPolynomialRoots Sol = new(px4, px3, px2, px1, px0);

        // OCCT: Sol is not done when the quartic has no real root, so a hyperbola missing the conic is not done
        if (!Sol.IsDone())
        {
            done = false;
            return;
        }
        else
        {
            if (Sol.InfiniteRoots())
            {
                iden = true;
                done = true;
                return;
            }
            // We have X=(CosH(t)*major_radius)/2, Y=(SinH(t)*minor_radius)/2
            //         the resolution is in S=Exp(t)
            nbp = Sol.NbSolutions();
            int nb_sol_valides = 0;
            for (i = 1; i <= nbp; i++)
            {
                S = Sol.Value(i);
                if (S > RealEpsilon())
                {
                    tx = 0.5 * major_radius * (S + 1 / S);
                    ty = 0.5 * minor_radius * (S - 1 / S);

                    //--- Are we on the correct branch of the Hyperbola?
                    //--------------- VERIF-VALIDITY-INTERSECTION ----------
                    //--- We assume that the deviation on curve1 is zero
                    //--- (the point was obtained by parameterization)
                    //--- ??? the tolerance was set to 1e-10 ?????????????

                    nb_sol_valides++;
                    Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                    S = Math.Log(S);
                    if (!HIsDirect)
                    {
                        S = -S;
                    }
                    lpnt[nb_sol_valides - 1].SetValue(tx, ty, S);
                }
            }
            nbp = nb_sol_valides;
            Traitement_Points_Confondus(ref nbp, lpnt);
        }
        done = true;
    }
}
