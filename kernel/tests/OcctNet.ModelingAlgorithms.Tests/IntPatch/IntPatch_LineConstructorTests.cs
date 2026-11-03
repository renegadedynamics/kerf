// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Occt.IntPatch_IType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_LineConstructor (unit 5G): a line split at its vertices into the parts inside both domains, for a
/// circle (GLine) of a plane and a cylinder on finite faces, a walking line half outside a face, and the analytic lines
/// of a sphere and a cylinder. The comparison with native OCCT is through GeomAPI_IntSS (the oracle).
/// </summary>
public class IntPatch_LineConstructorTests
{
    private const double Tol = 1.0e-7;

    private static bool Inside(SurfaceAdaptorBase theS, double u, double v) =>
        u >= theS.FirstUParameter() - 1e-9 && u <= theS.LastUParameter() + 1e-9 && v >= theS.FirstVParameter() - 1e-9
        && v <= theS.LastVParameter() + 1e-9;

    // The GLine branch reads the arcs' 3D tolerances (Tol3d), which a face's topology tool has
    // (BRepTopAdaptor_TopolTool) and Adaptor3d_TopolTool hasn't: it throws Standard_DomainError, as in OCCT.
    private sealed class TopolTool3d(SurfaceAdaptorBase theS) : TopolTool(theS)
    {
        public override bool Has3d() => true;

        public override double Tol3d(Curve2DAdaptorBase C) => Tol;
    }

    [Fact]
    public void Circle_of_plane_and_cylinder_split_by_the_square_face()
    {
        // The plane's face is the square [-1, 1]²; the circle of radius 1.2 leaves it around the axes: four arcs.
        var aS1 = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()), -1, 1, -1, 1);
        var aS2 = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(), 1.2), 0, 2 * Math.PI, -1, 1);
        TopolTool aD1 = new TopolTool3d(aS1);
        TopolTool aD2 = new TopolTool3d(aS2);
        var anInter = new IntPatch_ImpImpIntersection(aS1, aD1, aS2, aD2, Tol, Tol);
        Assert.True(anInter.IsDone());
        Assert.Equal(1, anInter.NbLines());
        IntPatch_Line aL = anInter.Line(1);
        Assert.Equal(IntPatch_Circle, aL.ArcType());

        var aLC = new IntPatch_LineConstructor(2);
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        aSLin.Append(aL);
        Assert.Throws<DomainErrorException>(() => aLC.Perform(aSLin, aL, aS1, new TopolTool(aS1), aS2,
                                                               new TopolTool(aS2), Tol));
        aLC.Perform(aSLin, aL, aS1, aD1, aS2, aD2, Tol);
        Assert.Equal(4, aLC.NbLines());
        for (int i = 1; i <= aLC.NbLines(); i++)
        {
            var aG = Assert.IsType<IntPatch_GLine>(aLC.Line(i));
            Assert.Equal(IntPatch_Circle, aG.ArcType());
            Assert.True(aG.HasFirstPoint() && aG.HasLastPoint());
            double f = aG.FirstPoint().ParameterOnLine(), l = aG.LastPoint().ParameterOnLine();
            Assert.True(l > f);
            Point3D aMid = global::Kerf.Engine.Geometry.ElCLib.Value(0.5 * (f + l), aG.Circle());
            Assert.True(Math.Abs(aMid.X()) <= 1 && Math.Abs(aMid.Y()) <= 1, $"arc {i} is outside the square");
            foreach (double t in new[] { f, l })
            {
                Point3D aP = global::Kerf.Engine.Geometry.ElCLib.Value(t, aG.Circle());
                Assert.True(Math.Abs(Math.Max(Math.Abs(aP.X()), Math.Abs(aP.Y())) - 1.0) < 1e-9,
                            $"arc {i} doesn't end on the square");
            }
        }
    }

    private static IntSurf_PntOn2S CirclePnt(double t)
    {
        var aP = new IntSurf_PntOn2S();
        aP.SetValue(new Point3D(Math.Cos(t), Math.Sin(t), 0), Math.Cos(t), Math.Sin(t), t, 0);
        return aP;
    }

    [Fact]
    public void Walking_line_half_outside_the_face_keeps_the_inside_part()
    {
        // The unit circle from t = 0 to π (21 points, vertices at points 1, 11 and 21) on the plane's face x >= 0.
        var aS1 = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()), 0, 2, -2, 2);
        var aS2 = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(), 1.0), 0, 2 * Math.PI, -1, 1);
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        var aL = new IntSurf_LineOn2S();
        for (int i = 0; i <= 20; i++)
        {
            aL.Add(CirclePnt(Math.PI * i / 20));
        }
        var aW = new IntPatch_WLine(aL, false);
        foreach (int k in new[] { 1, 11, 21 })
        {
            var aV = new IntPatch_Point();
            aV.SetValue(aL.Value(k));
            aV.SetParameter(k);
            aV.SetTolerance(Tol);
            aW.AddVertex(aV);
        }
        aW.SetPeriod(0, 0, 2 * Math.PI, 0);

        var aLC = new IntPatch_LineConstructor(2);
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        aSLin.Append(aW);
        aLC.Perform(aSLin, aW, aS1, aD1, aS2, aD2, Tol);
        Assert.Equal(1, aLC.NbLines());
        var aPart = Assert.IsType<IntPatch_WLine>(aLC.Line(1));
        Assert.Equal(11, aPart.NbPnts());
        Assert.Equal(2, aPart.NbVertex());
        Assert.Equal(1.0, aPart.Vertex(1).ParameterOnLine());
        Assert.Equal(11.0, aPart.Vertex(2).ParameterOnLine());
        for (int k = 1; k <= 11; k++)
        {
            Assert.Equal(aL.Value(k).Value(), aPart.Point(k).Value());
        }
        Assert.Equal(2 * Math.PI, aPart.U2Period());
        // The given line is unchanged.
        Assert.Equal(21, aW.NbPnts());
        Assert.Equal(3, aW.NbVertex());
    }

    [Fact]
    public void Analytic_lines_of_sphere_and_cylinder_split_at_their_vertices()
    {
        var aS1 = new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(), 1.0));
        var aS2 = new SurfaceAdaptor(new CylindricalSurface(
                                              new CoordinateSystem(new Point3D(0.6, 0.1, 0), new Direction3D(0, 0, 1)), 0.5),
                                          0, 2 * Math.PI, -2, 2);
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        var anInter = new IntPatch_ImpImpIntersection(aS1, aD1, aS2, aD2, Tol, Tol);
        Assert.True(anInter.IsDone());
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        for (int i = 1; i <= anInter.NbLines(); i++)
        {
            aSLin.Append(anInter.Line(i));
        }
        int aNbALines = 0, aNbParts = 0;
        for (int i = 1; i <= aSLin.Length(); i++)
        {
            if (aSLin.Value(i) is not IntPatch_ALine anA)
            {
                continue;
            }
            aNbALines++;
            var aLC = new IntPatch_LineConstructor(2);
            aLC.Perform(aSLin, anA, aS1, aD1, aS2, aD2, Tol);
            for (int j = 1; j <= aLC.NbLines(); j++)
            {
                var aPart = Assert.IsType<IntPatch_ALine>(aLC.Line(j));
                aNbParts++;
                double f = aPart.FirstPoint().ParameterOnLine(), l = aPart.LastPoint().ParameterOnLine();
                Point3D aMid = aPart.Value(0.5 * (f + l));
                Assert.True(Math.Abs(aMid.Distance(new Point3D()) - 1.0) < 1e-7, "off the sphere");
                double dx = aMid.X() - 0.6, dy = aMid.Y() - 0.1;
                Assert.True(Math.Abs(Math.Sqrt(dx * dx + dy * dy) - 0.5) < 1e-7, "off the cylinder");
            }
        }
        Assert.True(aNbALines > 0);
        Assert.True(aNbParts >= aNbALines);
    }
}
