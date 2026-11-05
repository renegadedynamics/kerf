// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The laws every geometric value keeps (question 49, every public member tested): equality with its operators, hash
/// codes and text; and each transformation's effect on the value's location and size, against arithmetic.
/// </summary>
public class GeometryValueLawsTests
{
    private static readonly Point3D P = new(1, 2, 3);
    private static readonly Point2D P2 = new(1, 2);
    private static readonly Direction3D Up = new(0, 0, 1);
    private static readonly Direction3D Right = new(1, 0, 0);
    private static readonly Frame3D Frame = new(P, Up, Right);
    private static readonly Frame3D Other = new(new Point3D(4, 5, 6), Up, Right);
    private static readonly CoordinateSystem System = new(P, Up, Right);
    private static readonly CoordinateSystem OtherSystem = new(new Point3D(4, 5, 6), Up, Right);
    private static readonly Frame2D Frame2 = new(P2, new Direction2D(1, 0));
    private static readonly Frame2D Other2 = new(new Point2D(4, 5), new Direction2D(1, 0));

    private static object?[] Row(object one, object same, object different) => [one.GetType().Name, one, same, different];

    private static Transform3D Shift(double x) => Transform3D.Translation(new Vector3D(x, 0, 0));

    private static Transform2D Shift2(double x) => Transform2D.Translation(new Vector2D(x, 0));

    public static IEnumerable<object?[]> Values =>
    [
        Row(P, new Point3D(1, 2, 3), new Point3D(1, 2, 4)),
        Row(P2, new Point2D(1, 2), new Point2D(1, 3)),
        Row(new Vector3D(1, 2, 3), new Vector3D(1, 2, 3), new Vector3D(3, 2, 1)),
        Row(new Vector2D(1, 2), new Vector2D(1, 2), new Vector2D(2, 1)),
        Row(Up, new Direction3D(0, 0, 2), Right),
        Row(new Direction2D(1, 0), new Direction2D(2, 0), new Direction2D(0, 1)),
        Row(new Axis3D(P, Up), new Axis3D(P, Up), new Axis3D(P, Right)),
        Row(new Axis2D(P2, new Direction2D(1, 0)), new Axis2D(P2, new Direction2D(1, 0)), new Axis2D(P2, new Direction2D(0, 1))),
        Row(Frame, new Frame3D(P, Up, Right), Other),
        Row(Frame2, new Frame2D(P2, new Direction2D(1, 0)), Other2),
        Row(System, new CoordinateSystem(P, Up, Right), OtherSystem),
        Row(new Line3D(P, Up), new Line3D(P, Up), new Line3D(P, Right)),
        Row(new Line2D(P2, new Direction2D(1, 0)), new Line2D(P2, new Direction2D(1, 0)), new Line2D(P2, new Direction2D(0, 1))),
        Row(new Circle3D(Frame, 2), new Circle3D(Frame, 2), new Circle3D(Frame, 3)),
        Row(new Circle2D(Frame2, 2), new Circle2D(Frame2, 2), new Circle2D(Frame2, 3)),
        Row(new Ellipse3D(Frame, 3, 2), new Ellipse3D(Frame, 3, 2), new Ellipse3D(Frame, 4, 2)),
        Row(new Ellipse2D(Frame2, 3, 2), new Ellipse2D(Frame2, 3, 2), new Ellipse2D(Frame2, 4, 2)),
        Row(new Hyperbola3D(Frame, 3, 4), new Hyperbola3D(Frame, 3, 4), new Hyperbola3D(Frame, 4, 4)),
        Row(new Hyperbola2D(Frame2, 3, 4), new Hyperbola2D(Frame2, 3, 4), new Hyperbola2D(Frame2, 4, 4)),
        Row(new Parabola3D(Frame, 2), new Parabola3D(Frame, 2), new Parabola3D(Frame, 3)),
        Row(new Parabola2D(Frame2, 2), new Parabola2D(Frame2, 2), new Parabola2D(Frame2, 3)),
        Row(new Plane3D(System), new Plane3D(P, Up), new Plane3D(OtherSystem)),
        Row(new Cylinder3D(System, 2), new Cylinder3D(System, 2), new Cylinder3D(System, 3)),
        Row(new Cone3D(System, 0.5, 2), new Cone3D(System, 0.5, 2), new Cone3D(System, 0.25, 2)),
        Row(new Sphere3D(System, 2), new Sphere3D(System, 2), new Sphere3D(System, 3)),
        Row(new Torus3D(System, 5, 2), new Torus3D(System, 5, 2), new Torus3D(System, 5, 3)),
        Row(Shift(1), Shift(1), Shift(2)),
        Row(Shift2(1), Shift2(1), Shift2(2)),
        Row(new GeneralTransform3D(Shift(1)), new GeneralTransform3D(Shift(1)), new GeneralTransform3D(Shift(2))),
        Row(new GeneralTransform2D(Shift2(1)), new GeneralTransform2D(Shift2(1)), new GeneralTransform2D(Shift2(2))),
        Row(Matrix3x3.Identity, new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), new Matrix3x3(2, 0, 0, 0, 1, 0, 0, 0, 1)),
        Row(new BoundingBox(Point3D.Origin, P), new BoundingBox(Point3D.Origin, P), new BoundingBox(P, new Point3D(2, 3, 4))),
        Row(OrientedBox.FromBoundingBox(new BoundingBox(Point3D.Origin, P)), OrientedBox.FromBoundingBox(new BoundingBox(Point3D.Origin, P)),
            OrientedBox.FromBoundingBox(new BoundingBox(P, new Point3D(2, 3, 4)))),
        Row(new Placement(), new Placement(), new Placement(Shift(1))),
        Row(new Color(0.1f, 0.2f, 0.3f), new Color(0.1f, 0.2f, 0.3f), new Color(0.3f, 0.2f, 0.1f)),
    ];

    [Theory]
    [MemberData(nameof(Values))]
    public void Equal_values_agree_and_different_ones_do_not(string type, object oneValue, object sameValue, object differentValue)
    {
        Assert.True((bool)Call(oneValue, "Equals", sameValue), type);
        Assert.True(oneValue.Equals(sameValue), type);
        Assert.True((bool)Call(null, oneValue.GetType(), "op_Equality", oneValue, sameValue), type);
        Assert.False((bool)Call(null, oneValue.GetType(), "op_Inequality", oneValue, sameValue), type);
        Assert.Equal(oneValue.GetHashCode(), sameValue.GetHashCode());
        Assert.Equal(oneValue.ToString(), sameValue.ToString());
        Assert.False(string.IsNullOrEmpty(oneValue.ToString()), type);

        Assert.False((bool)Call(oneValue, "Equals", differentValue), type);
        Assert.False(oneValue.Equals(differentValue), type);
        Assert.False((bool)Call(null, oneValue.GetType(), "op_Equality", oneValue, differentValue), type);
        Assert.True((bool)Call(null, oneValue.GetType(), "op_Inequality", oneValue, differentValue), type);
        Assert.NotEqual(oneValue.ToString(), differentValue.ToString());

        Assert.False(oneValue.Equals("something else"), type);
    }

    public static IEnumerable<object?[]> Placed3D =>
    [
        [new Axis3D(P, Up), null],
        [Frame, null],
        [System, null],
        [new Line3D(P, Up), null],
        [new Plane3D(System), null],
        [new Circle3D(Frame, 2), "Radius"],
        [new Ellipse3D(Frame, 3, 2), "MajorRadius"],
        [new Hyperbola3D(Frame, 3, 4), "MajorRadius"],
        [new Parabola3D(Frame, 2), "Focal"],
        [new Cylinder3D(System, 2), "Radius"],
        [new Cone3D(System, 0.5, 2), "RefRadius"],
        [new Sphere3D(System, 2), "Radius"],
        [new Torus3D(System, 5, 2), "MajorRadius"],
    ];

    private static void Same(Point3D expected, Point3D actual)
    {
        Assert.Equal(expected.X, actual.X, 9);
        Assert.Equal(expected.Y, actual.Y, 9);
        Assert.Equal(expected.Z, actual.Z, 9);
    }

    private static void Same(Point2D expected, Point2D actual)
    {
        Assert.Equal(expected.X, actual.X, 9);
        Assert.Equal(expected.Y, actual.Y, 9);
    }

    private static double Size(object value, string? property) =>
        property is null ? 1 : Convert.ToDouble(value.GetType().GetProperty(property)!.GetValue(value), CultureInfo.InvariantCulture);

    [Theory]
    [MemberData(nameof(Placed3D))]
    public void A_placed_value_moves_as_its_location_does(object value, string? size)
    {
        // located at (1, 2, 3)
        Same(new Point3D(2, 2, 3), Location(Call(value, "Translated", new Vector3D(1, 0, 0))));
        Same(new Point3D(1, 3, 3), Location(Call(value, "Translated", Point3D.Origin, new Point3D(0, 1, 0))));
        Same(new Point3D(-1, -2, -3), Location(Call(value, "Mirrored", Point3D.Origin)));
        Same(new Point3D(-1, -2, 3), Location(Call(value, "Mirrored", Axis3D.ZAxis)));
        Same(new Point3D(1, 2, -3), Location(Call(value, "Mirrored", Frame3D.World)));
        Same(new Point3D(-2, 1, 3), Location(Call(value, "Rotated", Axis3D.ZAxis, Math.PI / 2)));
        Same(new Point3D(2, 4, 6), Location(Call(value, "Scaled", Point3D.Origin, 2.0)));

        object scaled = Call(value, "Scaled", Point3D.Origin, 2.0);
        Assert.Equal(2 * Size(value, size) - (size is null ? 1 : 0), Size(scaled, size), 9);
        object moved = Call(value, "Translated", new Vector3D(5, 0, 0));
        Assert.Equal(Size(value, size), Size(moved, size), 9);
    }

    public static IEnumerable<object?[]> Placed2D =>
    [
        [new Axis2D(P2, new Direction2D(1, 0)), null],
        [Frame2, null],
        [new Line2D(P2, new Direction2D(1, 0)), null],
        [new Circle2D(Frame2, 2), "Radius"],
        [new Ellipse2D(Frame2, 3, 2), "MajorRadius"],
        [new Hyperbola2D(Frame2, 3, 4), "MajorRadius"],
        [new Parabola2D(Frame2, 2), "Focal"],
    ];

    [Theory]
    [MemberData(nameof(Placed2D))]
    public void A_placed_2D_value_moves_as_its_location_does(object value, string? size)
    {
        // located at (1, 2)
        Same(new Point2D(2, 2), Location2D(Call(value, "Translated", new Vector2D(1, 0))));
        Same(new Point2D(1, 3), Location2D(Call(value, "Translated", Point2D.Origin, new Point2D(0, 1))));
        Same(new Point2D(-1, -2), Location2D(Call(value, "Mirrored", Point2D.Origin)));
        Same(new Point2D(1, -2), Location2D(Call(value, "Mirrored", Axis2D.XAxis)));
        Same(new Point2D(-2, 1), Location2D(Call(value, "Rotated", Point2D.Origin, Math.PI / 2)));
        Same(new Point2D(2, 4), Location2D(Call(value, "Scaled", Point2D.Origin, 2.0)));

        object scaled = Call(value, "Scaled", Point2D.Origin, 2.0);
        Assert.Equal(2 * Size(value, size) - (size is null ? 1 : 0), Size(scaled, size), 9);
    }

    // A method by its name and argument types (the operators and "in" parameters rule out dynamic calls)
    private static object Call(object target, string name, params object[] arguments) => Call(target, target.GetType(), name, arguments);

    private static object Call(object? target, Type type, string name, params object[] arguments)
    {
        var method = type.GetMethods().Single(m => m.Name == name && m.GetParameters()
            .Select(p => p.ParameterType.IsByRef ? p.ParameterType.GetElementType()! : p.ParameterType)
            .SequenceEqual(arguments.Select(a => a.GetType())));
        return method.Invoke(target, arguments)!;
    }

    private static Point3D Location(object value) => (Point3D)value.GetType().GetProperty("Location")!.GetValue(value)!;

    private static Point2D Location2D(object value) => (Point2D)value.GetType().GetProperty("Location")!.GetValue(value)!;

    [Fact]
    public void An_ellipse_knows_its_foci_and_directrices()
    {
        // a = 3, b = 2 about the origin: c = sqrt(5), e = c / a
        double c = Math.Sqrt(5), e = c / 3;
        foreach (var (focal, eccentricity, parameter, focus1, focus2, directrix1, directrix2) in new[]
        {
            Facts(new Ellipse3D(Frame3D.World, 3, 2)),
            Facts(new EllipseCurve3D(Frame3D.World, 3, 2)),
        })
        {
            Assert.Equal(2 * c, focal, 9);
            Assert.Equal(e, eccentricity, 9);
            Assert.Equal(4.0 / 3, parameter, 9);
            Same(new Point3D(c, 0, 0), focus1);
            Same(new Point3D(-c, 0, 0), focus2);
            Assert.Equal(3 / e, directrix1, 9);
            Assert.Equal(-3 / e, directrix2, 9);
        }
        Assert.Equal(Math.PI * 6, new Ellipse3D(Frame3D.World, 3, 2).Area, 9);

        var flat = new Ellipse2D(Frame2D.World, 3, 2);
        Assert.Equal(Math.PI * 6, flat.Area, 9);
        Assert.Equal(2 * c, flat.Focal, 9);
        Assert.Equal(e, flat.Eccentricity, 9);
        Assert.Equal(4.0 / 3, flat.Parameter, 9);
        Same(new Point2D(c, 0), flat.Focus1);
        Same(new Point2D(-c, 0), flat.Focus2);
        Assert.Equal(3 / e, flat.Directrix1.Location.X, 9);
        Assert.Equal(-3 / e, flat.Directrix2.Location.X, 9);
        var curve = new EllipseCurve2D(Frame2D.World, 3, 2);
        Assert.Equal(3 / e, curve.Directrix1.Location.X, 9);
        Assert.Equal(-3 / e, curve.Directrix2.Location.X, 9);
    }

    private static (double, double, double, Point3D, Point3D, double, double) Facts(Ellipse3D e) =>
        (e.Focal, e.Eccentricity, e.Parameter, e.Focus1, e.Focus2, e.Directrix1.Location.X, e.Directrix2.Location.X);

    private static (double, double, double, Point3D, Point3D, double, double) Facts(EllipseCurve3D e) =>
        (e.Focal, e.Eccentricity, e.Parameter, e.Focus1, e.Focus2, e.Directrix1.Location.X, e.Directrix2.Location.X);

    [Fact]
    public void A_hyperbola_knows_its_foci_branches_and_asymptotes()
    {
        // a = 3, b = 4 about the origin: c = 5, e = 5 / 3
        var h = new Hyperbola3D(Frame3D.World, 3, 4);
        Assert.Equal(10, h.Focal, 9);
        Assert.Equal(5.0 / 3, h.Eccentricity, 9);
        Assert.Equal(16.0 / 3, h.Parameter, 9);
        Same(new Point3D(5, 0, 0), h.Focus1);
        Same(new Point3D(-5, 0, 0), h.Focus2);
        Assert.Equal(9.0 / 5, h.Directrix1.Location.X, 9);
        Assert.Equal(-9.0 / 5, h.Directrix2.Location.X, 9);
        Assert.Equal(3.0 / 5, h.Asymptote1.Direction.X, 9);
        Assert.Equal(4.0 / 5, h.Asymptote1.Direction.Y, 9);
        Assert.Equal(3.0 / 5, h.Asymptote2.Direction.X, 9);
        Assert.Equal(-4.0 / 5, h.Asymptote2.Direction.Y, 9);

        Assert.Equal(-1, h.OtherBranch().Position.XDirection.X, 9);
        Assert.Equal(4, h.ConjugateBranch1().MajorRadius, 9);
        Assert.Equal(3, h.ConjugateBranch1().MinorRadius, 9);
        Assert.Equal(1, h.ConjugateBranch1().Position.XDirection.Y, 9);
        Assert.Equal(-1, h.ConjugateBranch2().Position.XDirection.Y, 9);

        var c = new HyperbolaCurve3D(Frame3D.World, 3, 4);
        Assert.Equal(10, c.Focal, 9);
        Assert.Equal(5.0 / 3, c.Eccentricity, 9);
        Assert.Equal(16.0 / 3, c.Parameter, 9);
        Same(new Point3D(5, 0, 0), c.Focus1);
        Same(new Point3D(-5, 0, 0), c.Focus2);
        Assert.Equal(9.0 / 5, c.Directrix1.Location.X, 9);
        Assert.Equal(-9.0 / 5, c.Directrix2.Location.X, 9);
        Assert.Equal(4.0 / 5, c.Asymptote1.Direction.Y, 9);
        Assert.Equal(-4.0 / 5, c.Asymptote2.Direction.Y, 9);
        Assert.Equal(-1, c.OtherBranch().Position.XDirection.X, 9);
        Assert.Equal(4, c.ConjugateBranch1().MajorRadius, 9);
        Assert.Equal(-1, c.ConjugateBranch2().Position.XDirection.Y, 9);
        Assert.Equal(6, c.WithMajorRadius(6).MajorRadius, 9);
        Assert.Equal(7, c.WithMinorRadius(7).MinorRadius, 9);
        Assert.Equal(new Hyperbola3D(Other, 3, 4), c.WithHypr(new Hyperbola3D(Other, 3, 4)).Hyperbola);

        var flat = new Hyperbola2D(Frame2D.World, 3, 4);
        Assert.Equal(10, flat.Focal, 9);
        Assert.Equal(16.0 / 3, flat.Parameter, 9);
        Same(new Point2D(5, 0), flat.Focus1);
        Assert.Equal(9.0 / 5, flat.Directrix1.Location.X, 9);
        Assert.Equal(4, flat.ConjugateBranch1().MajorRadius, 9);
        Assert.Equal(1, flat.ConjugateBranch1().XAxis.Direction.Y, 9);
        Assert.Equal(-1, flat.ConjugateBranch2().XAxis.Direction.Y, 9);
        Assert.Equal(-1, flat.Reversed().YAxis.Direction.Y, 9);
        Assert.Equal(6, flat.WithMajorRadius(6).MajorRadius, 9);
        Assert.Equal(7, flat.WithMinorRadius(7).MinorRadius, 9);
        Same(new Point2D(9, 0), flat.Transformed(Shift2(9)).Location);

        var flatCurve = new HyperbolaCurve2D(Frame2D.World, 3, 4);
        Assert.Equal(9.0 / 5, flatCurve.Directrix1.Location.X, 9);
        Assert.Equal(-9.0 / 5, flatCurve.Directrix2.Location.X, 9);
        Assert.Equal(-1, flatCurve.OtherBranch().XAxis.Direction.X, 9);
        Assert.Equal(4, flatCurve.ConjugateBranch1().MajorRadius, 9);
        Assert.Equal(-1, flatCurve.ConjugateBranch2().XAxis.Direction.Y, 9);
        Assert.Equal(6, flatCurve.WithMajorRadius(6).MajorRadius, 9);
        Assert.Equal(7, flatCurve.WithMinorRadius(7).MinorRadius, 9);
        Assert.Equal(new Hyperbola2D(Other2, 3, 4), flatCurve.WithHypr2d(new Hyperbola2D(Other2, 3, 4)).Hyperbola);
    }

    [Fact]
    public void A_parabola_knows_its_focus_and_directrix()
    {
        var p = new Parabola3D(Frame3D.World, 2);
        Same(new Point3D(2, 0, 0), p.Focus);
        Assert.Equal(-2, p.Directrix.Location.X, 9);
        Assert.Equal(5, p.WithFocal(5).Focal, 9);

        // from a directrix and a focus: the vertex half way
        var byFocus = new Parabola3D(new Axis3D(Point3D.Origin, new Direction3D(0, 1, 0)), new Point3D(4, 0, 0));
        Assert.Equal(2, byFocus.Focal, 9);
        Same(new Point3D(2, 0, 0), byFocus.Location);

        var curve = new ParabolaCurve3D(new Axis3D(Point3D.Origin, new Direction3D(0, 1, 0)), new Point3D(4, 0, 0));
        Assert.Equal(2, curve.Focal, 9);
        Same(new Point3D(4, 0, 0), curve.Focus);
        Assert.Equal(0, curve.Directrix.Location.X, 9);
        Assert.Equal(1, curve.Eccentricity, 9);
        Assert.Equal(4, curve.Parameter, 9);
        Assert.Equal(7, curve.WithFocal(7).Focal, 9);
        Assert.Equal(new Parabola3D(Other, 2), curve.WithParab(new Parabola3D(Other, 2)).Parabola);

        var flat = new Parabola2D(Frame2D.World, 2);
        Assert.Equal(5, flat.WithFocal(5).Focal, 9);
        Assert.Equal(-1, flat.Reversed().Axis.YDirection.Y, 9);
        Same(new Point2D(9, 0), flat.Transformed(Shift2(9)).Location);
        Same(new Point2D(2, 3), flat.WithLocation(new Point2D(2, 3)).Location);
        Assert.Equal(1, flat.WithMirrorAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).MirrorAxis.Direction.Y, 9);
        Assert.Equal(new Point2D(4, 5), flat.WithAxis(Other2).Location);

        var flatCurve = new ParabolaCurve2D(new Axis2D(Point2D.Origin, new Direction2D(0, 1)), new Point2D(4, 0));
        Assert.Equal(2, flatCurve.Focal, 9);
        Assert.Equal(new Parabola2D(Other2, 2), flatCurve.WithParab2d(new Parabola2D(Other2, 2)).Parabola);
    }

    [Fact]
    public void Circles_measure_and_contain()
    {
        var flat = new Circle2D(Frame2D.World, 2);
        Assert.Equal(Math.PI * 4, flat.Area, 9);
        Assert.Equal(Math.PI * 4, flat.Length, 9);
        Assert.True(flat.Contains(new Point2D(2, 0), 1e-9));
        Assert.False(flat.Contains(new Point2D(3, 0), 1e-9));
        Assert.Equal(1, flat.SquareDistanceTo(new Point2D(3, 0)), 9);
        Same(new Point2D(9, 0), flat.Transformed(Shift2(9)).Location);
        Assert.Equal(new Point2D(4, 5), flat.WithAxis(Other2).Location);
        Assert.Equal(1, flat.WithXAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).XAxis.Direction.Y, 9);
        Assert.Equal(-1, flat.WithYAxis(new Axis2D(Point2D.Origin, new Direction2D(1, 0))).XAxis.Direction.Y, 9);

        var circle = new Circle3D(Frame3D.World, 2);
        Assert.True(circle.Contains(new Point3D(2, 0, 0), 1e-9));
        Assert.False(circle.Contains(new Point3D(2, 0, 1), 1e-9));
        Same(P, circle.WithLocation(P).Location);
        Assert.Equal(1, circle.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);

        var curve = new CircleCurve3D(Frame3D.World, 2);
        Assert.Equal(0, curve.Eccentricity, 9);
        Assert.Equal(5, curve.WithRadius(5).Radius, 9);
        Assert.Equal(new Circle3D(Other, 2), curve.WithCirc(new Circle3D(Other, 2)).Circle);
        Assert.Equal(new Circle2D(Other2, 2), new CircleCurve2D(Frame2D.World, 2).WithCirc2d(new Circle2D(Other2, 2)).Circle);
    }

    [Fact]
    public void Ellipses_and_hyperbolas_replace_their_parts()
    {
        var e = new Ellipse3D(Frame3D.World, 3, 2);
        Same(P, e.WithLocation(P).Location);
        Assert.Equal(1, e.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);

        var flat = new Ellipse2D(Frame2D.World, 3, 2);
        Same(new Point2D(2, 3), flat.WithLocation(new Point2D(2, 3)).Location);
        Assert.Equal(new Point2D(4, 5), flat.WithAxis(Other2).Location);
        Assert.Equal(1, flat.WithXAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).XAxis.Direction.Y, 9);
        Assert.Equal(-1, flat.WithYAxis(new Axis2D(Point2D.Origin, new Direction2D(1, 0))).XAxis.Direction.Y, 9);

        var curve = new EllipseCurve3D(Frame3D.World, 3, 2);
        Assert.Equal(4, curve.WithMajorRadius(4).MajorRadius, 9);
        Assert.Equal(1, curve.WithMinorRadius(1).MinorRadius, 9);
        Assert.Equal(new Ellipse3D(Other, 3, 2), curve.WithElips(new Ellipse3D(Other, 3, 2)).Ellipse);
        Assert.Equal(new Ellipse2D(Other2, 3, 2), new EllipseCurve2D(Frame2D.World, 3, 2).WithElips2d(new Ellipse2D(Other2, 3, 2)).Ellipse);

        var h = new Hyperbola3D(Frame3D.World, 3, 4);
        Same(P, h.WithLocation(P).Location);
        Assert.Equal(1, h.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);
        Assert.Equal(6, h.WithMajorRadius(6).MajorRadius, 9);
        Assert.Equal(7, h.WithMinorRadius(7).MinorRadius, 9);
        Same(new Point3D(4, 5, 6), h.WithPosition(Other).Location);

        var flatH = new Hyperbola2D(Frame2D.World, 3, 4);
        Same(new Point2D(2, 3), flatH.WithLocation(new Point2D(2, 3)).Location);
        Assert.Equal(new Point2D(4, 5), flatH.WithAxis(Other2).Location);
        Assert.Equal(1, flatH.WithXAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).XAxis.Direction.Y, 9);
        Assert.Equal(-1, flatH.WithYAxis(new Axis2D(Point2D.Origin, new Direction2D(1, 0))).XAxis.Direction.Y, 9);

        var p = new Parabola3D(Frame3D.World, 2);
        Same(P, p.WithLocation(P).Location);
        Assert.Equal(1, p.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);
        Same(new Point3D(4, 5, 6), p.WithPosition(Other).Location);
    }

    [Fact]
    public void Elementary_surfaces_measure_and_reverse()
    {
        var sphere = new Sphere3D(CoordinateSystem.World, 2);
        Assert.Equal(16 * Math.PI, sphere.Area, 9);
        Assert.Equal(32.0 / 3 * Math.PI, sphere.Volume, 9);
        Same(P, sphere.WithLocation(P).Location);
        Assert.False(sphere.UReversed().IsDirect);
        Assert.False(sphere.VReversed().IsDirect);

        var torus = new Torus3D(CoordinateSystem.World, 5, 2);
        Assert.Equal(4 * Math.PI * Math.PI * 5 * 2, torus.Area, 9);
        Assert.Equal(2 * Math.PI * Math.PI * 5 * 4, torus.Volume, 9);
        Assert.True(torus.IsDirect);
        Assert.False(torus.UReversed().IsDirect);
        Same(P, torus.WithLocation(P).Location);
        Assert.Equal(1, torus.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);
        Assert.Equal(6, torus.WithMajorRadius(6).MajorRadius, 9);
        Assert.Equal(1, torus.WithMinorRadius(1).MinorRadius, 9);
        Same(P, torus.WithPosition(System).Location);

        var cylinder = new Cylinder3D(CoordinateSystem.World, 2);
        Same(P, cylinder.WithLocation(P).Location);
        Assert.False(cylinder.UReversed().IsDirect);
        Assert.False(cylinder.VReversed().IsDirect);

        var cone = new Cone3D(CoordinateSystem.World, 0.5, 2);
        Same(P, cone.WithLocation(P).Location);
        Assert.Equal(1, cone.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);
        Assert.False(cone.UReversed().IsDirect);
        Assert.False(cone.VReversed().IsDirect);

        var plane = new Plane3D(CoordinateSystem.World);
        Assert.Equal(1, plane.WithAxis(Axis3D.XAxis).Axis.Direction.X, 9);
        Assert.False(plane.UReversed().IsDirect);
        Assert.False(plane.VReversed().IsDirect);
    }

    [Fact]
    public void Points_vectors_and_directions_take_apart_and_mirror()
    {
        (double x, double y, double z) = new Vector3D(1, 2, 3);
        Assert.Equal((1.0, 2.0, 3.0), (x, y, z));
        (double u, double v) = new Vector2D(4, 5);
        Assert.Equal((4.0, 5.0), (u, v));
        (x, y, z) = Up;
        Assert.Equal((0.0, 0.0, 1.0), (x, y, z));
        (u, v) = new Direction2D(1, 0);
        Assert.Equal((1.0, 0.0), (u, v));

        Assert.Equal(new Vector3D(2, 4, 6), new Vector3D(1, 2, 3).Scaled(2));
        Assert.Equal(new Vector2D(2, 4), new Vector2D(1, 2).Scaled(2));
        Assert.Equal(-3, new Vector3D(1, 2, 3).Mirrored(new Frame3D(Point3D.Origin, Up, Right)).Z, 9);
        Assert.Equal(new Vector3D(-1, -2, 3), new Vector3D(1, 2, 3).Mirrored(new Vector3D(0, 0, 1)));
        Assert.Equal(new Vector2D(1, -2), new Vector2D(1, 2).Mirrored(Axis2D.XAxis));
        Assert.Equal(new Vector2D(1, -2), new Vector2D(1, 2).Mirrored(new Vector2D(1, 0)));
        Assert.Equal(new Vector2D(2, -1), new Vector2D(1, 2).GetNormal());
        Assert.Equal(new Vector2D(3, 3), Vector2D.LinearForm(new Vector2D(1, 2), new Vector2D(2, 1)));
        Assert.Equal(new Vector2D(4, 5), Vector2D.LinearForm(2, new Vector2D(1, 2), new Vector2D(2, 1)));
        Assert.Equal(new Vector2D(5, 6), Vector2D.LinearForm(2, new Vector2D(1, 2), 1, new Vector2D(2, 1), new Vector2D(1, 1)));

        Assert.Equal(new Direction3D(1, 0, 1), Up.WithX(1));
        Assert.Equal(new Direction3D(1, 0, 1), Right.WithZ(1));
        Assert.Equal(new Direction3D(0, 1, 1), Up.WithY(1));
        Assert.Equal(new Direction2D(1, 1), new Direction2D(1, 0).WithY(1));
        Assert.Equal(new Direction2D(1, 1), new Direction2D(0, 1).WithX(1));
        Assert.Equal(new Direction2D(-1, 0), -new Direction2D(1, 0));
        Assert.Equal(new Direction2D(1, 0), new Direction2D(1, 0).Mirrored(new Direction2D(1, 0)));
        Same(new Point2D(2, 3), new Point2D(1, 2).Translated(new Point2D(0, 0), new Point2D(1, 1)));
    }

    [Fact]
    public void Axes_and_frames_compare()
    {
        var x = new Axis2D(Point2D.Origin, new Direction2D(1, 0));
        Assert.True(x.IsCoaxial(new Axis2D(new Point2D(5, 0), new Direction2D(1, 0)), 1e-9, 1e-9));
        Assert.False(x.IsCoaxial(new Axis2D(new Point2D(5, 1), new Direction2D(1, 0)), 1e-9, 1e-9));
        Assert.True(x.IsNormal(Axis2D.YAxis, 1e-9));
        Assert.True(x.IsOpposite(new Axis2D(Point2D.Origin, new Direction2D(-1, 0)), 1e-9));

        Assert.Equal(Math.PI / 2, new Frame3D(Point3D.Origin, Up, Right).AngleTo(new Frame3D(Point3D.Origin, Right, Up)), 9);
        Assert.True(Frame3D.World.IsCoplanar(new Frame3D(new Point3D(5, 5, 0), Up, Right), 1e-9, 1e-9));
        Assert.False(Frame3D.World.IsCoplanar(Frame, 1e-9, 1e-9));
        Assert.Equal(Math.PI / 2, CoordinateSystem.World.AngleTo(new CoordinateSystem(Point3D.Origin, Right, Up)), 9);
        Assert.True(CoordinateSystem.World.IsCoplanar(new Axis3D(new Point3D(5, 5, 0), new Direction3D(1, 1, 0)), 1e-9, 1e-9));
        Assert.False(CoordinateSystem.World.IsCoplanar(new Axis3D(new Point3D(5, 5, 1), new Direction3D(1, 1, 0)), 1e-9, 1e-9));

        // by a named axis
        Assert.Equal(1, new CoordinateSystem(Direction3D.D.Y).Direction.Y, 12);
        Assert.Equal(P, new CoordinateSystem(P, Direction3D.D.Y).Location);
        Assert.Equal(P, new Frame3D(P, Direction3D.D.Z, Direction3D.D.X).Location);

        var frame2 = Frame2D.World;
        Assert.Equal(1, frame2.WithXAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).XDirection.Y, 9);
        Assert.Equal(1, frame2.WithYAxis(new Axis2D(Point2D.Origin, new Direction2D(1, 0))).YDirection.X, 9);
        Assert.Equal(1, frame2.WithXDirection(new Direction2D(0, 1)).XDirection.Y, 9);
        Assert.Equal(1, frame2.WithYDirection(new Direction2D(1, 0)).YDirection.X, 9);

        var line = new Line2D(Point2D.Origin, new Direction2D(1, 0));
        Assert.Equal(1, line.Normal(new Point2D(5, 3)).Direction.Y * line.Normal(new Point2D(5, 3)).Direction.Y, 9);
        Assert.Equal(9, line.SquareDistanceTo(new Line2D(new Point2D(0, 3), new Direction2D(1, 0))), 9);
        Assert.Equal(1, line.WithPosition(Axis2D.YAxis).Direction.Y, 9);
    }

    [Fact]
    public void Transforms_compose_and_read_back()
    {
        var t = Transform2D.Translation(new Point2D(0, 0), new Point2D(1, 2));
        Same(new Point2D(1, 2), t * Point2D.Origin);
        Same(new Point2D(2, 3), t.PreMultiplied(Transform2D.Translation(new Vector2D(1, 1))) * Point2D.Origin);
        Same(new Point2D(-1, 2), Transform2D.Mirror(Axis2D.YAxis) * new Point2D(1, 2));
        Same(new Point2D(1, 2), Transform2D.Values(1, 0, 1, 0, 1, 2) * Point2D.Origin);
        Same(new Point2D(5, 2), t.WithTranslationPart(new Vector2D(5, 2)) * Point2D.Origin);
        Same(new Point2D(3, 3), Transform2D.Scale(Point2D.Origin, 1).WithScaleFactor(3) * new Point2D(1, 1));
        Same(new Point2D(0, 2), Transform2D.Transformation(new Axis2D(new Point2D(1, 0), new Direction2D(1, 0))) * new Point2D(1, 2));
        Same(new Point2D(1, 3), Transform2D.Transformation(Axis2D.XAxis, new Axis2D(new Point2D(0, -1), new Direction2D(1, 0))) * new Point2D(1, 2));

        // between 2D and 3D
        Same(new Point3D(1, 2, 0), new Transform3D(t) * Point3D.Origin);
        Same(new Point2D(1, 2), new Transform2D(new Transform3D(t)) * Point2D.Origin);

        Same(new Point3D(1, 2, 3), Transform3D.Values(1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3) * Point3D.Origin);
        Same(new Point3D(4, 5, 6), Transform3D.Displacement(CoordinateSystem.World, OtherSystem) * Point3D.Origin);

        var general = new GeneralTransform3D(Transform3D.Translation(new Vector3D(1, 0, 0)));
        Assert.Equal(1, general.TranslationPart.X, 9);
        Assert.False(general.IsSingular);
        Assert.Equal(2, (general * general).TranslationPart.X, 9);
        Assert.Equal(2, general.PreMultiplied(general).TranslationPart.X, 9);
        Assert.Equal(5, general.WithTrsf(Transform3D.Translation(new Vector3D(5, 0, 0))).TranslationPart.X, 9);
        Same(new Point3D(1, 2, 6), GeneralTransform3D.Affinity(Frame3D.World, 2) * new Point3D(1, 2, 3));
        Span<double> matrix = stackalloc double[16];
        general.GetMat4(matrix);
        Assert.Equal(1, matrix[3], 9);
        Assert.Equal(1, matrix[15], 9);

        var flat = new GeneralTransform2D(Transform2D.Translation(new Vector2D(1, 0)));
        Assert.Equal(1, flat.TranslationPart.X, 9);
        Assert.False(flat.IsSingular);
        Assert.False(flat.IsNegative);
        Assert.Equal(1, flat[1, 1], 9);
        Assert.Equal(1, flat[1, 3], 9);
        Assert.Equal(2, (flat * flat).TranslationPart.X, 9);
        Assert.Equal(2, flat.Multiplied(flat).TranslationPart.X, 9);
        Assert.Equal(2, flat.PreMultiplied(flat).TranslationPart.X, 9);

        Assert.Equal(new Matrix3x3(2, 0, 0, 0, 2, 0, 0, 0, 2), 2 * Matrix3x3.Identity);
    }
}
