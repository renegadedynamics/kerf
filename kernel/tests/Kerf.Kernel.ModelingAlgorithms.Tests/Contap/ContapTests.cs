// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.HiddenLines.Contours;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.HiddenLines.Contours.ContourLineType;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of Contap: the contours are where the surface's normal is orthogonal to the view (a direction) or to
/// the line of sight (an eye), on quadrics and on a B-spline surface; the sampling statics are per thread. The native
/// comparisons are <see cref="HlrNativeTests"/>.
/// </summary>
public class ContapTests
{
    private static Contour Contour(Face theFace, Func<Contour> theMake)
    {
        Face aF = ShapeCasts.Face(theFace.Oriented(Forward));
        AdaptorTool aTool = new(aF, global::Kerf.Engine.Foundation.Precision.PConfusion());
        Contour C = theMake();
        C.Perform(aTool.GetSurface()!, aTool.GetTopolTool());
        Assert.True(C.IsDone);
        return C;
    }

    // The points of a line: the walking points, or samples of a line or a circle between its vertices.
    private static IEnumerable<Point3D> Points(ContourLine L)
    {
        switch (L.TypeContour())
        {
            case ContourLineType.Walking:
                for (int i = 1; i <= L.PntCount; i++)
                {
                    yield return L.Point(i).Value();
                }
                break;
            case ContourLineType.Lin:
            case ContourLineType.Circle:
                if (L.VertexCount >= 2)
                {
                    double a = L.Vertex(1).ParameterOnLine(), b = L.Vertex(L.VertexCount).ParameterOnLine();
                    for (int i = 0; i <= 10; i++)
                    {
                        double t = a + (b - a) * i / 10.0;
                        yield return L.TypeContour() == ContourLineType.Lin ? global::Kerf.Engine.Geometry.ElCLib.Value(t, L.Line()) : global::Kerf.Engine.Geometry.ElCLib.Value(t, L.Circle());
                    }
                }
                break;
        }
    }

    private static Vector3D Normal(Face theFace, Point3D P)
    {
        Surface aS = Tool.Surface(theFace)!;
        ProjectPointOnSurface aProj = new(P, aS);
        aProj.LowerDistanceParameters(out double u, out double v);
        aS.D1(u, v, out _, out Vector3D du, out Vector3D dv);
        return du.Crossed(dv).Normalized();
    }

    public static TheoryData<string> FaceNames() => ["sphere", "cylinder", "cone", "torus", "bspline"];

    [Theory]
    [MemberData(nameof(FaceNames))]
    public void Contour_ForADirection_IsWhereTheNormalIsOrthogonal(string theName)
    {
        Face aFace = HlrRecorder.Faces().First(f => f.Name == theName).Face;
        Vector3D aDir = theName == "bspline" ? new Vector3D(1, 0, 0) : new Vector3D(0.3, -0.4, 0.8);
        Contour C = Contour(aFace, () => new Contour(aDir));
        Assert.True(C.LineCount > 0);
        int aChecked = 0;
        for (int l = 1; l <= C.LineCount; l++)
        {
            foreach (Point3D P in Points(C.Line(l)))
            {
                double aDot = Normal(aFace, P).Dot(aDir.Normalized());
                Assert.True(Math.Abs(aDot) < 1e-3, $"{theName}: line {l}, {P}: n.d = {aDot}");
                aChecked++;
            }
        }
        Assert.True(aChecked > 0);
    }

    [Theory]
    [MemberData(nameof(FaceNames))]
    public void Contour_ForAnEye_IsWhereTheNormalIsOrthogonalToTheLineOfSight(string theName)
    {
        Face aFace = HlrRecorder.Faces().First(f => f.Name == theName).Face;
        Point3D aEye = new(40, -25, 30);
        Contour C = Contour(aFace, () => new Contour(aEye));
        int aChecked = 0;
        for (int l = 1; l <= C.LineCount; l++)
        {
            foreach (Point3D P in Points(C.Line(l)))
            {
                double aDot = Normal(aFace, P).Dot(new Vector3D(aEye, P).Normalized());
                Assert.True(Math.Abs(aDot) < 1e-3, $"{theName}: line {l}, {P}: n.d = {aDot}");
                aChecked++;
            }
        }
        Assert.True(theName == "bspline" || aChecked > 0);
    }

    [Fact]
    public void Contour_OfASphere_IsAGreatCircle()
    {
        Face aFace = HlrRecorder.Faces().First(f => f.Name == "sphere").Face;
        Contour C = Contour(aFace, () => new Contour(new Vector3D(0, 0, 1)));
        for (int l = 1; l <= C.LineCount; l++)
        {
            ContourLine L = C.Line(l);
            Assert.Equal(ContourLineType.Circle, L.TypeContour());
            Assert.Equal(10.0, L.Circle().Radius, 1e-12);
            Assert.True(L.Circle().Location.IsEqual(new Point3D(1, 2, 3), 1e-12));
            Assert.True(L.Circle().Axis.Direction.IsParallel(new Direction3D(0, 0, 1), 1e-12));
        }
    }

    [Fact]
    public void Contour_BeforePerform_Throws()
    {
        Contour C = new();
        Assert.False(C.IsDone);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => C.LineCount);
        Face aFace = HlrRecorder.Faces().First(f => f.Name == "sphere").Face;
        AdaptorTool aTool = new(aFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => C.Perform(aTool.GetSurface()!, aTool.GetTopolTool()));
    }

    [Fact]
    public void Contour_InParallel_EqualsSerial()
    {
        // Contap_HContTool's sampling bounds are [ThreadStatic] (decision 1075): contours computed at once on several
        // threads equal the serial ones.
        string[] aNames = ["torus", "bspline", "torus part", "cylinder part"];
        string Run(string theName)
        {
            HlrRecorder aRec = new();
            aRec.RecContours(theName);
            return aRec.Text;
        }
        Dictionary<string, string> aSerial = aNames.ToDictionary(n => n, Run);
        string[] aParallel = new string[aNames.Length * 3];
        Parallel.For(0, aParallel.Length, i => aParallel[i] = Run(aNames[i % aNames.Length]));
        for (int i = 0; i < aParallel.Length; i++)
        {
            Assert.Equal(aSerial[aNames[i % aNames.Length]], aParallel[i]);
        }
    }

    [Fact]
    public void ContAna_Cylinder_TwoGeneratrices()
    {
        ContAna C = new();
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3.0);
        C.Perform(aCyl, new Direction3D(1, 0, 0));
        Assert.Equal(2, C.ContourCount);
        Assert.Equal(CurveKind.Line, C.TypeContour());
        Assert.True(C.Line(1).Location.IsEqual(new Point3D(0, 3, 0), 1e-12) || C.Line(1).Location.IsEqual(new Point3D(0, -3, 0), 1e-12));
        C.Perform(aCyl, new Direction3D(0, 0, 1));
        Assert.Equal(0, C.ContourCount);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => C.TypeContour());
    }
}
