// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_ALineToWLine on the analytic lines of IntPatch_ImpImpIntersection (cylinder/sphere, cylinder/cone,
/// cone/sphere, a sphere through a cone's apex): every point of every walking line on both quadrics at its
/// parameters, the vertices kept at the line's points, the creation way, purging off; and its tolerances.
/// </summary>
public class IntPatch_ALineToWLineTests
{
    private const double Tol = 1.0e-7;

    private static CoordinateSystem Ax(double x, double y, double z, double dx, double dy, double dz) =>
        new(new Point3D(x, y, z), new Direction3D(dx, dy, dz));

    private static (NCollection_Sequence<PatchLine> Lines, int NbALines) Convert(
        SurfaceAdaptor theS1, SurfaceAdaptor theS2, int theNbPoints = 200)
    {
        var anII = new ImpImpIntersection(theS1, new TopolTool(theS1), theS2,
                                                   new TopolTool(theS2), Tol, Tol);
        Assert.True(anII.IsDone);
        var aConv = new ALineToWLine(theS1, theS2, theNbPoints);
        var aLines = new NCollection_Sequence<PatchLine>();
        int aNbA = 0;
        for (int i = 1; i <= anII.LineCount; i++)
        {
            if (anII.Line(i) is ALine anA)
            {
                aNbA++;
                aConv.MakeWLine(anA, aLines);
            }
        }
        return (aLines, aNbA);
    }

    private static void Check(NCollection_Sequence<PatchLine> theLines, SurfaceAdaptor theS1,
                              SurfaceAdaptor theS2)
    {
        Assert.True(theLines.Length() > 0);
        for (int i = 1; i <= theLines.Length(); i++)
        {
            var aWL = Assert.IsType<WLine>(theLines.Value(i));
            Assert.Equal(PatchLineType.Walking, aWL.ArcType());
            Assert.Equal(WLine.IntPatch_WLType.WLImpImp, aWL.GetCreatingWay());
            Assert.False(aWL.IsPurgingAllowed);
            Assert.True(aWL.NbPnts() > 1);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                PntOn2S aP = aWL.Point(k);
                aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < 1.0e-6, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < 1.0e-6, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.VertexCount; k++)
            {
                PatchPoint aV = aWL.Vertex(k);
                double aPar = aV.ParameterOnLine();
                Assert.InRange(aPar, 1.0, aWL.NbPnts());
                Assert.True(aWL.Point((int)aPar).Value().Distance(aV.Value()) < 1.0e-6, $"line {i} vertex {k}");
            }
        }
    }

    [Fact]
    public void Cylinder_and_sphere()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(Ax(0, 0, 0, 0, 0, 1), 1), 0, 2 * M_PI, -3, 3);
        var aSph = new SurfaceAdaptor(new SphericalSurface(Ax(1, 0, 0, 0, 0, 1), 2));
        var (aLines, aNbA) = Convert(aCyl, aSph);
        Assert.Equal(2, aNbA);
        Check(aLines, aCyl, aSph);
    }

    [Fact]
    public void Cylinder_and_cone()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(Ax(0, 0, 0, 0, 0, 1), 1), 0, 2 * M_PI, -5, 5);
        var aCone = new SurfaceAdaptor(new ConicalSurface(Ax(0.3, 0, 0, 1, 0, 1), M_PI / 8, 0.5), 0,
                                            2 * M_PI, -5, 5);
        var (aLines, aNbA) = Convert(aCyl, aCone);
        Assert.True(aNbA > 0);
        Check(aLines, aCyl, aCone);
    }

    [Fact]
    public void Cone_and_sphere()
    {
        var aCone = new SurfaceAdaptor(new ConicalSurface(Ax(0, 0, 0, 0, 0, 1), M_PI / 6, 1), 0, 2 * M_PI,
                                            -3, 3);
        var aSph = new SurfaceAdaptor(new SphericalSurface(Ax(0.2, 0, 1, 0, 0, 1), 2));
        var (aLines, aNbA) = Convert(aCone, aSph);
        Assert.Equal(2, aNbA);
        Check(aLines, aCone, aSph);
    }

    [Fact]
    public void Fewer_points_still_on_both_surfaces()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(Ax(0, 0, 0, 0, 0, 1), 1), 0, 2 * M_PI, -3, 3);
        var aSph = new SurfaceAdaptor(new SphericalSurface(Ax(1, 0, 0, 0, 0, 1), 2));
        var (aLines, _) = Convert(aCyl, aSph, 20);
        Check(aLines, aCyl, aSph);
    }

    [Fact]
    public void Same_lines_every_run_and_in_parallel()
    {
        static string Bits()
        {
            var aCyl = new SurfaceAdaptor(new CylindricalSurface(Ax(0, 0, 0, 0, 0, 1), 1), 0, 2 * M_PI, -5,
                                               5);
            var aCone = new SurfaceAdaptor(new ConicalSurface(Ax(0.3, 0, 0, 1, 0, 1), M_PI / 8, 0.5), 0,
                                                2 * M_PI, -5, 5);
            var (aLines, _) = Convert(aCyl, aCone);
            var aSb = new System.Text.StringBuilder();
            for (int i = 1; i <= aLines.Length(); i++)
            {
                var aWL = (WLine)aLines.Value(i);
                for (int k = 1; k <= aWL.NbPnts(); k++)
                {
                    aWL.Point(k).Parameters(out double u1, out double v1, out double u2, out double v2);
                    aSb.Append(BitConverter.DoubleToInt64Bits(u1)).Append(BitConverter.DoubleToInt64Bits(v1))
                       .Append(BitConverter.DoubleToInt64Bits(u2)).Append(BitConverter.DoubleToInt64Bits(v2));
                }
                aSb.Append('|');
            }
            return aSb.ToString();
        }

        string aRef = Bits();
        Assert.Equal(aRef, Bits());
        string[] aPar = new string[4];
        Parallel.For(0, aPar.Length, i => aPar[i] = Bits());
        Assert.All(aPar, s => Assert.Equal(aRef, s));
    }

    [Fact]
    public void Tolerances()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(Ax(0, 0, 0, 0, 0, 1), 1));
        var aConv = new ALineToWLine(aCyl, aCyl);
        Assert.Equal(1.0e-9, aConv.TolOpenDomain());
        Assert.Equal(1.0e-8, aConv.TolTransition());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aConv.Tol3D());
        aConv.SetTolOpenDomain(1.0e-6);
        aConv.SetTolTransition(1.0e-5);
        aConv.SetTol3D(1.0e-4);
        Assert.Equal(1.0e-6, aConv.TolOpenDomain());
        Assert.Equal(1.0e-5, aConv.TolTransition());
        Assert.Equal(1.0e-4, aConv.Tol3D());
    }
}
