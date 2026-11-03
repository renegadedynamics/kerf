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

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.TwoD.Outils;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// Analytic intersection of two 2D curves: two lines, two circles, a line and a circle, or a line, circle,
/// ellipse, parabola or hyperbola with a conic given by its implicit equation.
/// </summary>
/// <remarks>An algorithm object: construct it with the curves, or construct it empty and call <c>Perform</c>.</remarks>
public class AnalyticIntersection
{
    private bool _done;
    private bool _para;
    private bool _iden;
#pragma warning disable CS0414 // OCCT sets empt everywhere but never reads it (IsEmpty is computed from nbp and iden)
    private bool _empt;
#pragma warning restore CS0414
    private int _nbp;
    private readonly IntPoint[] _lpnt = [new(), new(), new(), new()];

    /// <summary>Empty constructor. IsDone returns false.</summary>
    public AnalyticIntersection()
    {
        _done = false;
        _para = false;
        _iden = false;
        _empt = true;
        _nbp = 0;
    }

    /// <summary>Intersection between two lines.</summary>
    public AnalyticIntersection(in Line2D l1, in Line2D l2) => Perform(l1, l2);

    /// <summary>Intersection between two circles.</summary>
    public AnalyticIntersection(in Circle2D c1, in Circle2D c2) => Perform(c1, c2);

    /// <summary>Intersection between a line and a circle.</summary>
    public AnalyticIntersection(in Line2D l, in Circle2D c) => Perform(l, c);

    /// <summary>Intersection between a line and a conic.</summary>
    public AnalyticIntersection(in Line2D l, in Conic conic) => Perform(l, conic);

    /// <summary>Intersection between a parabola and another conic.</summary>
    public AnalyticIntersection(in Parabola2D p, in Conic conic) => Perform(p, conic);

    /// <summary>Intersection between a circle and another conic.</summary>
    public AnalyticIntersection(in Circle2D c, in Conic conic) => Perform(c, conic);

    /// <summary>Intersection between an ellipse and another conic.</summary>
    public AnalyticIntersection(in Ellipse2D e, in Conic conic) => Perform(e, conic);

    /// <summary>Intersection between a hyperbola and another conic.</summary>
    public AnalyticIntersection(in Hyperbola2D e, in Conic conic) => Perform(e, conic);

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>
    /// True when there is no intersection: no intersection point and the elements are not identical. The elements
    /// may be parallel in this case.
    /// </summary>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return ((_nbp == 0) && (!_iden));
        }
    }

    /// <summary>
    /// For the intersection between an element of gp and a conic known by an implicit equation, true if the element
    /// of gp verifies the implicit equation. For two Lin2d or two Circ2d, true if the elements are identical.
    /// False in all the other cases.
    /// </summary>
    public bool IdenticalElements()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _iden;
    }

    /// <summary>For the intersection between two Lin2d or two Circ2d, true if the elements are parallel.</summary>
    public bool ParallelElements()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _para;
    }

    /// <summary>The number of intersection points between the two curves.</summary>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _nbp;
        }
    }

    /// <summary>The intersection point of range N.</summary>
    /// <exception cref="OutOfRangeException">N is not in [1, NbPoints].</exception>
    public IntPoint Point(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        else
        {
            if ((n <= 0) || (n > _nbp))
            {
                throw new OutOfRangeException();
            }
            else
            {
                return _lpnt[n - 1];
            }
        }
    }

    // IntAna2d_AnaIntersection_1.cxx

    /// <summary>Intersection between two lines.</summary>
    public void Perform(in Line2D l1, in Line2D l2)
    {
        _done = false;

        double a1, b1, c1;
        double a2, b2, c2;
        l1.Coefficients(out a1, out b1, out c1);
        l2.Coefficients(out a2, out b2, out c2);

        double al1, be1, ga1;
        double al2, be2, ga2;

        double det = Max(Math.Abs(a1), Max(Math.Abs(a2), Max(Math.Abs(b1), Math.Abs(b2))));

        if (Math.Abs(a1) == det)
        {
            al1 = a1;
            be1 = b1;
            ga1 = c1;
            al2 = a2;
            be2 = b2;
            ga2 = c2;
        }
        else if (Math.Abs(b1) == det)
        {
            al1 = b1;
            be1 = a1;
            ga1 = c1;
            al2 = b2;
            be2 = a2;
            ga2 = c2;
        }
        else if (Math.Abs(a2) == det)
        {
            al1 = a2;
            be1 = b2;
            ga1 = c2;
            al2 = a1;
            be2 = b1;
            ga2 = c1;
        }
        else
        {
            al1 = b2;
            be1 = a2;
            ga1 = c2;
            al2 = b1;
            be2 = a1;
            ga2 = c1;
        }

        double rap = al2 / al1;
        double denom = be2 - rap * be1;

        if (Math.Abs(denom) <= RealEpsilon())
        { // Coincident directions
            _para = true;
            _nbp = 0;
            if (Math.Abs(ga2 - rap * ga1) <= RealEpsilon())
            { // Coincident lines
                _iden = true;
                _empt = false;
            }
            else
            { // Parallel lines
                _iden = false;
                _empt = true;
            }
        }
        else
        {
            _para = false;
            _iden = false;
            _empt = false;
            _nbp = 1;
            double xs = (be1 * ga2 / al1 - be2 * ga1 / al1) / denom;
            double ys = (rap * ga1 - ga2) / denom;

            if (((Math.Abs(a1) != det) && (Math.Abs(b1) == det))
                || ((Math.Abs(a1) != det) && (Math.Abs(b1) != det) && (Math.Abs(a2) != det)))
            {
                double temp = xs;
                xs = ys;
                ys = temp;
            }

            double la, mu;
            if (Math.Abs(a1) >= Math.Abs(b1))
            {
                la = (ys - l1.Location().Y()) / a1;
            }
            else
            {
                la = (l1.Location().X() - xs) / b1;
            }
            if (Math.Abs(a2) >= Math.Abs(b2))
            {
                mu = (ys - l2.Location().Y()) / a2;
            }
            else
            {
                mu = (l2.Location().X() - xs) / b2;
            }
            _lpnt[0].SetValue(xs, ys, la, mu);
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_2.cxx

    // The parameter on a circle of the angle a in [0, 2PI[ measured counterclockwise from its X axis: -a on an indirect
    // circle, in [0, 2PI[ (decision 16706; OCCT kept a, wrong on an indirect circle).
    private static double CircleParameter(double a, in Circle2D c)
        => c.IsDirect || a == 0.0 ? a : 2 * M_PI - a;

    /// <summary>Intersection between two circles.</summary>
    public void Perform(in Circle2D c1, in Circle2D c2)
    {
        _done = false;
        double d = c1.Location().Distance(c2.Location());
        double r1 = c1.Radius();
        double r2 = c2.Radius();
        double sum = r1 + r2;
        double dif = Math.Abs(r1 - r2);

        if (d <= RealEpsilon())
        { // Concentric circles
            _para = true;
            _nbp = 0;
            if (dif <= RealEpsilon())
            { // Coincident circles
                _empt = false;
                _iden = true;
            }
            else
            { // Parallel circles
                _empt = true;
                _iden = false;
            }
        }
        else if ((d - sum) > Epsilon(sum))
        { // Circles exterior to each other
            // et No solution
            _empt = true;
            _para = false;
            _iden = false;
            _nbp = 0;
        }
        else if (Math.Abs(d - sum) <= Epsilon(sum))
        { // Externally tangent circles
            _empt = false;
            _para = false;
            _iden = false;
            _nbp = 1;
            Vector2D ax = new(c1.Location(), c2.Location());
            Vector2D ox1 = new(c1.XAxis().Direction());
            Vector2D ox2 = new(c2.XAxis().Direction());

            double xs = (c1.Location().X() * r2 + c2.Location().X() * r1) / sum;
            double ys = (c1.Location().Y() * r2 + c2.Location().Y() * r1) / sum;
            double ang1 = ox1.Angle(ax); // Result between -PI and +PI
            double ang2 = ox2.Angle(ax) + M_PI;
            if (ang1 < 0)
            {
                ang1 = 2 * M_PI + ang1;
            } // Normalize to range [0, 2PI]
            if (ang2 >= 2 * M_PI)
            {
                ang2 -= 2 * M_PI;
            }
            _lpnt[0].SetValue(xs, ys, CircleParameter(ang1, c1), CircleParameter(ang2, c2));
        }
        else if (((sum - d) > Epsilon(sum)) && ((d - dif) > Epsilon(d + dif)))
        {
            _empt = false;
            _para = false;
            _iden = false;
            _nbp = 2;
            Vector2D ax = new(c1.Location(), c2.Location());
            Vector2D ox1 = new(c1.XAxis().Direction());
            Vector2D ox2 = new(c2.XAxis().Direction());
            double ref1 = ox1.Angle(ax); // Result between -PI and +PI
            double ref2 = ox2.Angle(ax); // Result between -PI and +PI

            double l1 = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            if (r1 * r1 - l1 * l1 < 0.0)
            {
                l1 = (l1 > 0 ? r1 : -r1);
            }
            double h = Math.Sqrt(r1 * r1 - l1 * l1);

            double xs1 = c1.Location().X() + l1 * ax.X() / d - h * ax.Y() / d;
            double ys1 = c1.Location().Y() + l1 * ax.Y() / d + h * ax.X() / d;

            double xs2 = c1.Location().X() + l1 * ax.X() / d + h * ax.Y() / d;
            double ys2 = c1.Location().Y() + l1 * ax.Y() / d - h * ax.X() / d;

            double sint1 = h / r1;
            double cost1 = l1 / r1;

            double sint2 = h / r2;
            double cost2 = (l1 - d) / r2;

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
            _lpnt[0].SetValue(xs1, ys1, CircleParameter(ang11, c1), CircleParameter(ang21, c2));
            _lpnt[1].SetValue(xs2, ys2, CircleParameter(ang12, c1), CircleParameter(ang22, c2));
        }
        else if (Math.Abs(d - dif) <= Epsilon(sum))
        { // Internally tangent circles
            _empt = false;
            _para = false;
            _iden = false;
            _nbp = 1;
            Vector2D ax = new(c1.Location(), c2.Location());
            if (c1.Radius() < c2.Radius())
            {
                ax = ax.Reversed();
            }

            Vector2D ox1 = new(c1.XAxis().Direction());
            Vector2D ox2 = new(c2.XAxis().Direction());
            double ang1 = ox1.Angle(ax); // Result between -PI and +PI
            double ang2 = ox2.Angle(ax);
            if (ang1 < 0)
            {
                ang1 = 2 * M_PI + ang1;
            } // Normalize to range [0, 2PI]
            if (ang2 < 0)
            {
                ang2 = 2 * M_PI + ang2;
            } // Normalize to range [0, 2PI]
            double xs = (c1.Location().X() * r2 - c2.Location().X() * r1) / (r2 - r1);
            double ys = (c1.Location().Y() * r2 - c2.Location().Y() * r1) / (r2 - r1);
            _lpnt[0].SetValue(xs, ys, CircleParameter(ang1, c1), CircleParameter(ang2, c2));
        }
        else
        { // We must have d<dif-Resol and d<>0, therefore
            // one circle inside the other and no solution
            _empt = true;
            _para = false;
            _iden = false;
            _nbp = 0;
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_3.cxx

    /// <summary>Intersection between a line and a circle.</summary>
    public void Perform(in Line2D l, in Circle2D c)
    {
        _done = false;

        _iden = false;
        _para = false;
        //
        double a, b, c0, d;
        Point2D p2D, p2D1, p2D2;
        //
        l.Coefficients(out a, out b, out c0);
        d = a * c.Location().X() + b * c.Location().Y() + c0;

        if (Math.Abs(d) - c.Radius() > Epsilon(c.Radius()))
        {
            _empt = true;
            _nbp = 0;
        }
        else
        { // At least 1 solution
            _empt = false;
            if (Math.Abs(Math.Abs(d) - c.Radius()) <= Epsilon(c.Radius()))
            { // Tangency case
                double u, xs, ys, ang;
                //
                _nbp = 1;
                xs = c.Location().X() - d * a;
                ys = c.Location().Y() - d * b;
                //
                p2D = new Point2D(xs, ys);
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p2D);
                ang = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p2D);
                _lpnt[0].SetValue(xs, ys, u, ang);
            }
            else
            { // 2 intersection points
                double h, xs1, ys1, xs2, ys2, ang1, ang2, u1, u2;
                _nbp = 2;
                h = Math.Sqrt(c.Radius() * c.Radius() - d * d);
                xs1 = c.Location().X() - d * a - h * b;
                ys1 = c.Location().Y() - d * b + h * a;
                xs2 = c.Location().X() - d * a + h * b;
                ys2 = c.Location().Y() - d * b - h * a;
                //
                p2D1 = new Point2D(xs1, ys1);
                p2D2 = new Point2D(xs2, ys2);
                u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p2D1);
                u2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p2D2);
                ang1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p2D1);
                ang2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p2D2);

                _lpnt[0].SetValue(xs1, ys1, u1, ang1);
                _lpnt[1].SetValue(xs2, ys2, u2, ang2);
            }
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_4.cxx

    /// <summary>Intersection between a line and a conic.</summary>
    public void Perform(in Line2D l, in Conic conic)
    {
        double a, b, c, d, e, f;
        double px0, px1, px2;
        double DR_A, DR_B, DR_C, x0, y0;
        int i;
        double tx, ty, s;

        _done = false;
        _nbp = 0;
        _para = false;
        _iden = false;

        conic.Coefficients(out a, out b, out c, out d, out e, out f);
        l.Coefficients(out DR_A, out DR_B, out DR_C);
        x0 = l.Location().X();
        y0 = l.Location().Y();

        // Parameter: L
        // X = Xo - L*DR_B  and  Y = Yo + L*DR_A

        px0 = f + x0 * (d + d + a * x0 + 2.0 * c * y0) + y0 * (e + e + b * y0);
        px1 = 2.0 * (e * DR_A - d * DR_B + x0 * (c * DR_A - a * DR_B) + y0 * (b * DR_A - c * DR_B));
        px2 = DR_A * (b * DR_A - 2.0 * c * DR_B) + a * (DR_B * DR_B);

        MyDirectPolynomialRoots sol = new(px2, px1, px0);

        if (!sol.IsDone)
        {
            _done = false;
            return;
        }
        else
        {
            if (sol.InfiniteRoots())
            {
                _iden = true;
                _done = true;
                return;
            }
            _nbp = sol.SolutionCount;
            for (i = 1; i <= _nbp; i++)
            {
                s = sol.Value(i);
                tx = x0 - s * DR_B;
                ty = y0 + s * DR_A;
                _lpnt[i - 1].SetValue(tx, ty, s);
            }
            Traitement_Points_Confondus(ref _nbp, _lpnt);
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_5.cxx

    /// <summary>Intersection between a circle and another conic.</summary>
    public void Perform(in Circle2D circle, in Conic conic)
    {
        bool cIsDirect = circle.IsDirect;
        double a, b, c, d, e, f;
        double pcc, pss, p2sc, pc, ps, pcte;
        double radius = circle.Radius();
        double radius_P2 = radius * radius;
        int i;
        double tx, ty, s;

        _done = false;
        _nbp = 0;
        _para = false;
        _empt = false;
        _iden = false;

        Axis2D Axe_rep = circle.XAxis();

        conic.Coefficients(out a, out b, out c, out d, out e, out f);
        conic.NewCoefficients(ref a, ref b, ref c, ref d, ref e, ref f, Axe_rep);

        // Parameter a with x=Radius*cos(a) and y=Radius*sin(a)

        pss = b * radius_P2;
        pcc = a * radius_P2 - pss; // COS ^2
        p2sc = c * radius_P2; // 2 SIN COS
        pc = 2.0 * d * radius; // COS
        ps = 2.0 * e * radius; // SIN
        pcte = f + pss; // 1

        TrigonometricFunctionRoots sol = new(pcc, p2sc, pc, ps, pcte, 0.0, 2.0 * M_PI);

        if (!sol.IsDone)
        {
            // std::cout of "math_TrigonometricFunctionRoots -> NotDone" left out
            _done = false;
            return;
        }
        else
        {
            if (sol.InfiniteRoots())
            {
                _iden = true;
                _done = true;
                return;
            }
            _nbp = sol.SolutionCount;
            for (i = 1; i <= _nbp; i++)
            {
                s = sol.Value(i);
                tx = radius * Math.Cos(s);
                ty = radius * Math.Sin(s);
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!cIsDirect)
                {
                    s = M_PI + M_PI - s;
                }
                _lpnt[i - 1].SetValue(tx, ty, s);
            }
            Traitement_Points_Confondus(ref _nbp, _lpnt);
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_6.cxx

    /// <summary>Intersection between an ellipse and another conic.</summary>
    public void Perform(in Ellipse2D elips, in Conic conic)
    {
        bool eIsDirect = elips.IsDirect;
        double a, b, c, d, e, f;
        double pcte, ps, pc, p2sc, pcc, pss;
        double minor_radius = elips.MinorRadius();
        double major_radius = elips.MajorRadius();
        int i;
        double tx, ty, s;

        _done = false;
        _nbp = 0;
        _para = false;
        _iden = false;
        _empt = false;

        Axis2D Axe_rep = elips.XAxis();

        conic.Coefficients(out a, out b, out c, out d, out e, out f);
        conic.NewCoefficients(ref a, ref b, ref c, ref d, ref e, ref f, Axe_rep);

        // Parameter: a with x=MajorRadius*cos(a) and y=MinorRadius*sin(a)

        pss = b * minor_radius * minor_radius; // SIN ^2
        pcc = a * major_radius * major_radius - pss; // COS ^2
        p2sc = c * major_radius * minor_radius; // 2 SIN COS
        pc = 2.0 * d * major_radius; // COS
        ps = 2.0 * e * minor_radius; // SIN
        pcte = f + pss; // 1

        TrigonometricFunctionRoots sol = new(pcc, p2sc, pc, ps, pcte, 0.0, 2.0 * M_PI);

        if (!sol.IsDone)
        {
            _done = false;
            return;
        }
        else
        {
            if (sol.InfiniteRoots())
            {
                _iden = true;
                _done = true;
                return;
            }
            _nbp = sol.SolutionCount;
            for (i = 1; i <= _nbp; i++)
            {
                s = sol.Value(i);
                tx = major_radius * Math.Cos(s);
                ty = minor_radius * Math.Sin(s);
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!eIsDirect)
                {
                    s = M_PI + M_PI - s;
                }
                _lpnt[i - 1].SetValue(tx, ty, s);
            }
            Traitement_Points_Confondus(ref _nbp, _lpnt);
        }
        _done = true;
    }

    // IntAna2d_AnaIntersection_7.cxx

    /// <summary>Intersection between a parabola and another conic.</summary>
    public void Perform(in Parabola2D p, in Conic conic)
    {
        bool pIsDirect = p.IsDirect;
        double a, b, c, d, e, f;
        double px4, px3, px2, px1, px0;
        int i;
        double tx, ty, s;
        double un_sur_2p = 0.5 / (p.Parameter());
        Axis2D Axe_rep = p.MirrorAxis();

        _done = false;
        _nbp = 0;
        _para = false;
        _empt = false;
        _iden = false;

        conic.Coefficients(out a, out b, out c, out d, out e, out f);
        conic.NewCoefficients(ref a, ref b, ref c, ref d, ref e, ref f, Axe_rep);

        //-------- Parameter y with y=y, x=y^2/(2p)

        px0 = f;
        px1 = e + e;
        px2 = b + un_sur_2p * (d + d);
        px3 = (c + c) * un_sur_2p;
        px4 = a * (un_sur_2p * un_sur_2p);

        MyDirectPolynomialRoots sol = new(px4, px3, px2, px1, px0);

        if (!sol.IsDone)
        {
            _done = false; // OCCT: no return, so done ends true (the hyperbola case returns not done)
        }
        else
        {
            if (sol.InfiniteRoots())
            {
                _iden = true;
                _done = true;
            }
            _nbp = sol.SolutionCount;
            for (i = 1; i <= _nbp; i++)
            {
                s = sol.Value(i);
                tx = un_sur_2p * s * s;
                ty = s;
                Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                if (!pIsDirect)
                {
                    s = -s;
                }
                _lpnt[i - 1].SetValue(tx, ty, s);
            }
            Traitement_Points_Confondus(ref _nbp, _lpnt);
        }
        _done = true;
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
    public void Perform(in Hyperbola2D h, in Conic conic)
    {
        bool hIsDirect = h.IsDirect;
        double a, b, c, d, e, f;
        double px0, px1, px2, px3, px4;
        double minor_radius = h.MinorRadius();
        double major_radius = h.MajorRadius();
        int i;
        double tx, ty, s;

        _done = false;
        _nbp = 0;
        _para = false;
        _iden = false;
        _empt = false;

        Axis2D Axe_rep = h.XAxis();
        conic.Coefficients(out a, out b, out c, out d, out e, out f);
        conic.NewCoefficients(ref a, ref b, ref c, ref d, ref e, ref f, Axe_rep);

        double A_major_radiusP2 = a * major_radius * major_radius;
        double B_minor_radiusP2 = b * minor_radius * minor_radius;
        double C_2_major_minor_radius = c * 2.0 * major_radius * minor_radius;

        // Parameter: t with x=MajorRadius*Ch(t)  y=minorRadius*Sh(t)
        // The polynomial is rewritten in Exp(t)
        // The following are the coefficients of polynomial P multiplied by 4*Exp(t)^2

        px0 = A_major_radiusP2 - C_2_major_minor_radius + B_minor_radiusP2;
        px1 = 4.0 * (d * major_radius - e * minor_radius);
        px2 = 2.0 * (A_major_radiusP2 + 2.0 * f - B_minor_radiusP2);
        px3 = 4.0 * (d * major_radius + e * minor_radius);
        px4 = A_major_radiusP2 + C_2_major_minor_radius + B_minor_radiusP2;

        MyDirectPolynomialRoots sol = new(px4, px3, px2, px1, px0);

        // OCCT: Sol is not done when the quartic has no real root, so a hyperbola missing the conic is not done
        if (!sol.IsDone)
        {
            _done = false;
            return;
        }
        else
        {
            if (sol.InfiniteRoots())
            {
                _iden = true;
                _done = true;
                return;
            }
            // We have X=(CosH(t)*major_radius)/2, Y=(SinH(t)*minor_radius)/2
            //         the resolution is in S=Exp(t)
            _nbp = sol.SolutionCount;
            int nb_sol_valides = 0;
            for (i = 1; i <= _nbp; i++)
            {
                s = sol.Value(i);
                if (s > RealEpsilon())
                {
                    tx = 0.5 * major_radius * (s + 1 / s);
                    ty = 0.5 * minor_radius * (s - 1 / s);

                    //--- Are we on the correct branch of the Hyperbola?
                    //--------------- VERIF-VALIDITY-INTERSECTION ----------
                    //--- We assume that the deviation on curve1 is zero
                    //--- (the point was obtained by parameterization)
                    //--- ??? the tolerance was set to 1e-10 ?????????????

                    nb_sol_valides++;
                    Coord_Ancien_Repere(ref tx, ref ty, Axe_rep);
                    s = Math.Log(s);
                    if (!hIsDirect)
                    {
                        s = -s;
                    }
                    _lpnt[nb_sol_valides - 1].SetValue(tx, ty, s);
                }
            }
            _nbp = nb_sol_valides;
            Traitement_Points_Confondus(ref _nbp, _lpnt);
        }
        _done = true;
    }
}
