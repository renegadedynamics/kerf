// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Tests.Shared;

/// <summary>
/// The inputs of two of OCCT's issues whose Booleans come out valid and wrong, rebuilt from their scripts, for the tests of the
/// check of a Boolean's volume (decision 21301).
/// </summary>
internal static class BooleanIssueShapes
{
    /// <summary>
    /// OCCT #1629's thread: a block with a bore, and a helical groove fused with a core of the bore's radius. At some depths the
    /// cut gives two solids, one of them inside out.
    /// </summary>
    public static (Shape Part, Shape Tool) Thread(double depth, double pitch, double inner, double outer, double side, double deep)
    {
        Shape bore = new Transform(new ApiMakeCylinder(inner, deep + 1).Shape(),
                                   new Transform3D().WithTranslation(new Vector3D(side / 2, side / 2, side - deep)), true).Shape();
        Shape part = new Cut(new MakeBox(side, side, side).Shape(), bore).Shape();

        double c = Math.Tan(Math.PI / 6), r = inner - pitch / 20, width = pitch / 8 + 2 * (outer - r) * c;
        var profile = new MakePolygon();
        profile.Add(new Point3D(r, 0, -width / 2));
        profile.Add(new Point3D(outer, 0, -pitch / 16));
        profile.Add(new Point3D(outer, 0, pitch / 16));
        profile.Add(new Point3D(r, 0, width / 2));
        profile.Close();

        double endWidth = pitch / 8 + 2 * (outer - inner) * c, turns = (depth + pitch - endWidth / 2) / pitch;
        var cylinder = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), (r + outer) / 2);
        var line = new LineCurve2D(new Point2D(0, 0), new Direction2D(2 * Math.PI, pitch));
        Wire spine = new MakeWire(new MakeEdge(line, cylinder, 0, turns * Math.Sqrt(4 * Math.PI * Math.PI + pitch * pitch)).InternalEdge()).InternalWire();
        BrepLibrary.BuildCurves3d(spine, 1e-5, Continuity.C1, 14, 0);
        var sweep = new MakePipeShell(spine);
        sweep.SetMode(true);
        sweep.Add(profile.Wire());
        sweep.Build();
        sweep.MakeSolid();

        var fuse = new Fuse(sweep.Shape(), new ApiMakeCylinder(inner, depth + pitch).Shape());
        fuse.SimplifyResult();
        Shape tool = new Transform(fuse.Shape(), new Transform3D().WithTranslation(new Vector3D(0, 0, -pitch)), true).Shape();
        tool = new Transform(tool, new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), Math.PI), true).Shape();
        tool = new Transform(tool, new Transform3D().WithTranslation(new Vector3D(side / 2, side / 2, side)), true).Shape();
        return (part, tool);
    }

    /// <summary>OCCT #1643's lofts, which share an end section: a body through three sections, a lip through two.</summary>
    public static (Shape Body, Shape Lip) Lofts() => (Loft(-1629.0, -1660.0, -1700.0), Loft(-1629.0, -1638.0));

    private static Shape Loft(params double[] stations)
    {
        var maker = new ThruSections(true, false);
        foreach (double x in stations)
        {
            maker.AddWire(Section(x));
        }
        maker.Build();
        return maker.Shape();
    }

    private static List<(double Y, double Z)> Arc(double halfWidth, double halfHeight, double exponent, double from, double to, int count, double middle)
    {
        var points = new List<(double, double)>();
        for (int i = 0; i < count; i++)
        {
            double a = (from + (to - from) * i / (count - 1)) * Math.PI / 180, c = Math.Cos(a), s = Math.Sin(a);
            points.Add((halfWidth * Math.CopySign(Math.Pow(Math.Abs(c), 2 / exponent), c),
                        middle + halfHeight * Math.CopySign(Math.Pow(Math.Abs(s), 2 / exponent), s)));
        }
        return points;
    }

    private static Wire Section(double x)
    {
        (double width, double top, double bottom, double narrowing, double upper, double lower) = x switch
        {
            -1629.0 => (122.0, 916.0, 784.0, 0.52, 2.45, 2.55),
            -1638.0 => (121.8235, 915.1174, 783.0291, 0.5174, 2.45, 2.5522),
            -1660.0 => (120.3782, 907.8912, 775.0803, 0.4957, 2.45, 2.5703),
            _ => (118.0, 896.0, 762.0, 0.46, 2.45, 2.6),
        };
        double middle = (top + bottom) / 2, height = (top - bottom) / 2;
        var half = Arc(width, height, lower, -90, 0, 8, middle).SkipLast(1).Concat(Arc(width, height, upper, 0, 90, 14, middle))
            .Select(p => (Y: p.Y * (1 - narrowing * Math.Pow(Math.Max((p.Z - middle) / height, 0), 1.25)), p.Z)).ToList();
        var loop = half.Concat(Enumerable.Reverse(half.Skip(1).Take(half.Count - 2)).Select(p => (Y: -p.Y, p.Z))).ToList();
        var points = new NCollection_HArray1<Point3D>(1, loop.Count);
        for (int i = 0; i < loop.Count; i++)
        {
            points.SetValue(i + 1, new Point3D(x, loop[i].Y, loop[i].Z));
        }
        var interpolation = new ApiInterpolate(points, true, 1e-6);
        interpolation.Perform();
        return new MakeWire(new MakeEdge(interpolation.Curve()).Edge()).Wire();
    }
}
