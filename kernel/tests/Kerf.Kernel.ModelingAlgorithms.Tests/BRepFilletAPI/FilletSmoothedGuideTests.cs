// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Engine.Collections;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// A fillet of constant radius along a guide ChFi3d smooths (unit 15AO): where an edge's curve isn't C2, ChFi3d walks along a
/// B-spline through 101 of its points, kept to 1e-3 whatever the shape's size (decision 2319), and a chamfer kept its distance
/// from that guide, off the edge. A fillet doesn't: BlendFunc_ConstRad puts the ball's centre at the radius from each contact
/// along that surface's own normal (decision 2156), and the guide only places the plane each section is solved in. So the
/// guide moves where the sections fall, not how far the ball is from the faces. Checked here by the ball put back on the
/// surfaces ChFi3d computed, at 39 sections along each and 7 points across, on the cases that move the guide most.
/// </summary>
public class FilletSmoothedGuideTests
{
    private static Shape Read(string name)
    {
        using Stream stream = typeof(FilletSmoothedGuideTests).Assembly.GetManifestResourceStream($"BRepFilletAPI.Issues.{name}.brep")
                              ?? throw new InvalidOperationException(name);
        BrepTools.Read(out Shape shape, new StreamReader(stream));
        return shape;
    }

    // A prism 5 high of a profile whose top is the curve, from x 0 to 20, closed by three lines down to y -10
    private static Shape Prism(BSplineCurve3D curve)
    {
        Edge top = ShapeFactory.FromCurve(curve).Value;
        Point3D a = curve.Value(curve.FirstParameter), b = curve.Value(curve.LastParameter);
        Point3D c = new(b.X, -10, 0), d = new(a.X, -10, 0);
        Wire wire = ShapeFactory.FromEdges([top, ShapeFactory.Line(b, c).Value, ShapeFactory.Line(c, d).Value, ShapeFactory.Line(d, a).Value]).Value;
        return ShapeFactory.FromWire(wire, onlyPlane: true).Value.Extrude(new Vector3D(0, 0, 5)).Value.Shape;
    }

    private static (Shape Shape, Edge Edge) Case(string name)
    {
        switch (name)
        {
            case "quadratic":
            {
                // Degree 2 with simple knots: C1, its curvature jumping at each knot
                Point3D[] poles = [new(0, 0, 0), new(4, 4, 0), new(8, 0, 0), new(12, 3, 0), new(16, 1, 0), new(20, 0, 0)];
                Shape prism = Prism(new BSplineCurve3D(poles, [0, 1, 2, 3, 4], [3, 1, 1, 1, 3], 2));
                return (prism, TopCurve(prism));
            }
            case "cubic, a double knot":
            {
                Point3D[] poles = [new(0, 0, 0), new(4, 3, 0), new(8, 3, 0), new(12, 0, 0), new(16, -2, 0), new(20, 0, 0)];
                Shape prism = Prism(new BSplineCurve3D(poles, [0, 1, 2], [4, 2, 4], 3));
                return (prism, TopCurve(prism));
            }
            default:
            {
                // 15U4's C1 (CCH_001_qhff), its long curved top edge, the one a chamfer was off along (decision 2324)
                Shape shape = Read("chamfer-C1");
                Point3D from = new(-0.118375, -0.14323, 0.127352), to = new(0.168673, 0, 0.127371);
                return (shape, shape.AllEdges().Single(e => e.AllVertices().Length == 2 && e.AllVertices().Any(v => v.Point.DistanceTo(from) < 1e-5)
                                                             && e.AllVertices().Any(v => v.Point.DistanceTo(to) < 1e-5)));
            }
        }
    }

    private static Edge TopCurve(Shape prism) =>
        prism.AllEdges().Single(e => Tool.Curve(e, out _, out _) is BSplineCurve3D && e.AllVertices().All(v => Math.Abs(v.Point.Z - 5) < 1e-9));

    // The worst found over 15AO's cases (decision 33801): the quadratic at 2, 1.3e-5 from the ball against the approximation's
    // 1e-4, its guide 1.2e-3 off the edge. The others: the cubic with a double knot at 1, 1.9e-6, its guide 8e-4 off; C1's top
    // edge at 0.01, 4.5e-6, its guide 6.0e-3 off on a solid 0.3 across, more than half the radius
    [Theory]
    [InlineData("quadratic", 2.0)]
    [InlineData("cubic, a double knot", 1.0)]
    [InlineData("C1", 0.01)]
    public void A_fillet_along_a_guide_ChFi3d_smooths_keeps_its_radius(string name, double radius)
    {
        (Shape shape, Edge edge) = Case(name);
        var fillet = new MakeFillet(shape);
        fillet.Add(radius, edge);
        fillet.Build();
        Assert.True(fillet.IsDone);
        Assert.True(fillet.Shape().Validate().IsValid);
        FilBuilder builder = fillet.InternalBuilder();
        Assert.Equal(1, builder._guidesSmoothed);
        Assert.True(GuideOff(builder) > 5e-4); // the guide ChFi3d walked isn't the edge

        (double miss, int sections) = BallMiss(builder, radius);
        Assert.True(sections > 0);
        Assert.True(miss < builder._tolapp3d, $"{name} at {radius}: the ball misses by {miss}");

        // and the public fillet builds it, with its checks
        Result<Built<Shape>> made = shape.Fillet([edge], radius);
        Assert.True(made.IsSuccess, made.Error?.ToString());
    }

    // How far ChFi3d's guides are from their edges: the most at 201 points of each
    private static double GuideOff(FilBuilder builder)
    {
        double worst = 0;
        foreach (Stripe stripe in builder._listStripe)
        {
            Spine spine = stripe.Spine()!;
            for (int i = 1; i <= spine.EdgeCount; i++)
            {
                ElSpine guide = spine.ElSpine(i)!;
                double f = Math.Max(spine.FirstParameter(i), guide.FirstParameter()), l = Math.Min(spine.LastParameter(i), guide.LastParameter());
                Curve3D curve = Tool.Curve(spine.Edges(i), out double a, out double b)!;
                for (int k = 0; k <= 200; k++)
                {
                    var projection = new ProjectPointOnCurve(guide.EvalD0(f + ((l - f) * k / 200)), curve, a, b);
                    worst = Math.Max(worst, projection.NbPoints() > 0 ? projection.LowerDistance() : double.MaxValue);
                }
            }
        }
        return worst;
    }

    // The ball on ChFi3d's surfaces between two faces: at each point, centred at the radius along the surface's normal, on the
    // nearer side, and its distance from each face's surface at the foot of its normal, found from that section's contact (on
    // the surface carried on, where the fillet ends across a face slanted to the edge). The most it is off the radius
    private static (double Miss, int Sections) BallMiss(FilBuilder builder, double radius)
    {
        double worst = 0;
        int sections = 0;
        foreach (Stripe stripe in builder._listStripe)
        {
            NCollection_HSequence<SurfData> set = stripe.SetOfSurfData()!;
            for (int k = 1; k <= set.Length(); k++)
            {
                SurfData data = set.Value(k);
                Surface surface = builder._ds.Surface(data.Surf()).Surface()!;
                Surface one = Tool.Surface(ShapeCasts.Face(builder._ds.Shape(data.IndexOfS1())))!;
                Surface two = Tool.Surface(ShapeCasts.Face(builder._ds.Shape(data.IndexOfS2())))!;
                FaceInterference on1 = data.InterferenceOnS1(), on2 = data.InterferenceOnS2();
                Curve2D c1 = on1.PCurveOnSurf()!, c2 = on2.PCurveOnSurf()!, f1 = on1.PCurveOnFace()!, f2 = on2.PCurveOnFace()!;
                double a1 = on1.FirstParameter(), b1 = on1.LastParameter(), a2 = on2.FirstParameter(), b2 = on2.LastParameter();
                if (c1.EvalD0(a1).DistanceTo(c2.EvalD0(a2)) > c1.EvalD0(a1).DistanceTo(c2.EvalD0(b2)))
                {
                    (a2, b2) = (b2, a2);
                }
                for (int i = 1; i < 40; i++)
                {
                    double s = i / 40.0, t1 = a1 + ((b1 - a1) * s), t2 = a2 + ((b2 - a2) * s);
                    Point2D from = c1.EvalD0(t1), to = c2.EvalD0(t2);
                    sections++;
                    for (int j = 1; j < 8; j++)
                    {
                        double w = j / 8.0;
                        surface.D1(from.X + ((to.X - from.X) * w), from.Y + ((to.Y - from.Y) * w), out Point3D p, out Vector3D du, out Vector3D dv);
                        Vector3D n = du.Crossed(dv);
                        n = n.Multiplied(radius / n.Magnitude);
                        double best = double.MaxValue;
                        foreach (Point3D centre in new[] { new Point3D(p.X + n.X, p.Y + n.Y, p.Z + n.Z), new Point3D(p.X - n.X, p.Y - n.Y, p.Z - n.Z) })
                        {
                            best = Math.Min(best, Math.Max(Math.Abs(Foot(one, f1.EvalD0(t1), centre) - radius), Math.Abs(Foot(two, f2.EvalD0(t2), centre) - radius)));
                        }
                        worst = Math.Max(worst, best);
                    }
                }
            }
        }
        return (worst, sections);
    }

    // The distance from a point to the foot of its normal on a surface, by Newton's method from a start near it
    private static double Foot(Surface surface, Point2D start, Point3D point)
    {
        double u = start.X, v = start.Y;
        for (int i = 0; i < 20; i++)
        {
            surface.D2(u, v, out Point3D p, out Vector3D su, out Vector3D sv, out Vector3D suu, out Vector3D svv, out Vector3D suv);
            var d = new Vector3D(point, p);
            double fu = d.Dot(su), fv = d.Dot(sv);
            double a = su.Dot(su) + d.Dot(suu), b = su.Dot(sv) + d.Dot(suv), c = sv.Dot(sv) + d.Dot(svv);
            double det = (a * c) - (b * b);
            if (Math.Abs(det) < 1e-300)
            {
                break;
            }
            double du = ((c * fu) - (b * fv)) / det, dv = ((a * fv) - (b * fu)) / det;
            u -= du;
            v -= dv;
            if (Math.Abs(du) + Math.Abs(dv) < 1e-14)
            {
                break;
            }
        }
        return surface.EvalD0(u, v).DistanceTo(point);
    }
}
