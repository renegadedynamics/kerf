// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Contap_IType;
using static Occt.TopAbs_Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of Contap: the contours are where the surface's normal is orthogonal to the view (a direction) or to
/// the line of sight (an eye), on quadrics and on a B-spline surface; the sampling statics are per thread. The native
/// comparisons are <see cref="HlrNativeTests"/>.
/// </summary>
public class ContapTests
{
    private static Contap_Contour Contour(TopoDS_Face theFace, Func<Contap_Contour> theMake)
    {
        TopoDS_Face aF = TopoDS.Face(theFace.Oriented(TopAbs_FORWARD));
        BRepTopAdaptor_Tool aTool = new(aF, Precision.PConfusion());
        Contap_Contour C = theMake();
        C.Perform(aTool.GetSurface()!, aTool.GetTopolTool());
        Assert.True(C.IsDone());
        return C;
    }

    // The points of a line: the walking points, or samples of a line or a circle between its vertices.
    private static IEnumerable<gp_Pnt> Points(Contap_Line L)
    {
        switch (L.TypeContour())
        {
            case Contap_Walking:
                for (int i = 1; i <= L.NbPnts(); i++)
                {
                    yield return L.Point(i).Value();
                }
                break;
            case Contap_Lin:
            case Contap_Circle:
                if (L.NbVertex() >= 2)
                {
                    double a = L.Vertex(1).ParameterOnLine(), b = L.Vertex(L.NbVertex()).ParameterOnLine();
                    for (int i = 0; i <= 10; i++)
                    {
                        double t = a + (b - a) * i / 10.0;
                        yield return L.TypeContour() == Contap_Lin ? ElCLib.Value(t, L.Line()) : ElCLib.Value(t, L.Circle());
                    }
                }
                break;
        }
    }

    private static gp_Vec Normal(TopoDS_Face theFace, gp_Pnt P)
    {
        Geom_Surface aS = BRep_Tool.Surface(theFace)!;
        GeomAPI_ProjectPointOnSurf aProj = new(P, aS);
        aProj.LowerDistanceParameters(out double u, out double v);
        aS.D1(u, v, out _, out gp_Vec du, out gp_Vec dv);
        return du.Crossed(dv).Normalized();
    }

    public static TheoryData<string> FaceNames() => ["sphere", "cylinder", "cone", "torus", "bspline"];

    [Theory]
    [MemberData(nameof(FaceNames))]
    public void Contour_ForADirection_IsWhereTheNormalIsOrthogonal(string theName)
    {
        TopoDS_Face aFace = HlrRecorder.Faces().First(f => f.Name == theName).Face;
        gp_Vec aDir = theName == "bspline" ? new gp_Vec(1, 0, 0) : new gp_Vec(0.3, -0.4, 0.8);
        Contap_Contour C = Contour(aFace, () => new Contap_Contour(aDir));
        Assert.True(C.NbLines() > 0);
        int aChecked = 0;
        for (int l = 1; l <= C.NbLines(); l++)
        {
            foreach (gp_Pnt P in Points(C.Line(l)))
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
        TopoDS_Face aFace = HlrRecorder.Faces().First(f => f.Name == theName).Face;
        gp_Pnt aEye = new(40, -25, 30);
        Contap_Contour C = Contour(aFace, () => new Contap_Contour(aEye));
        int aChecked = 0;
        for (int l = 1; l <= C.NbLines(); l++)
        {
            foreach (gp_Pnt P in Points(C.Line(l)))
            {
                double aDot = Normal(aFace, P).Dot(new gp_Vec(aEye, P).Normalized());
                Assert.True(Math.Abs(aDot) < 1e-3, $"{theName}: line {l}, {P}: n.d = {aDot}");
                aChecked++;
            }
        }
        Assert.True(theName == "bspline" || aChecked > 0);
    }

    [Fact]
    public void Contour_OfASphere_IsAGreatCircle()
    {
        TopoDS_Face aFace = HlrRecorder.Faces().First(f => f.Name == "sphere").Face;
        Contap_Contour C = Contour(aFace, () => new Contap_Contour(new gp_Vec(0, 0, 1)));
        for (int l = 1; l <= C.NbLines(); l++)
        {
            Contap_Line L = C.Line(l);
            Assert.Equal(Contap_Circle, L.TypeContour());
            Assert.Equal(10.0, L.Circle().Radius(), 1e-12);
            Assert.True(L.Circle().Location().IsEqual(new gp_Pnt(1, 2, 3), 1e-12));
            Assert.True(L.Circle().Axis().Direction().IsParallel(new gp_Dir(0, 0, 1), 1e-12));
        }
    }

    [Fact]
    public void Contour_BeforePerform_Throws()
    {
        Contap_Contour C = new();
        Assert.False(C.IsDone());
        Assert.ThrowsAny<StdFail_NotDone>(() => C.NbLines());
        TopoDS_Face aFace = HlrRecorder.Faces().First(f => f.Name == "sphere").Face;
        BRepTopAdaptor_Tool aTool = new(aFace, Precision.PConfusion());
        Assert.ThrowsAny<Standard_ConstructionError>(() => C.Perform(aTool.GetSurface()!, aTool.GetTopolTool()));
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
        Contap_ContAna C = new();
        gp_Cylinder aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3.0);
        C.Perform(aCyl, new gp_Dir(1, 0, 0));
        Assert.Equal(2, C.NbContours());
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, C.TypeContour());
        Assert.True(C.Line(1).Location().IsEqual(new gp_Pnt(0, 3, 0), 1e-12) || C.Line(1).Location().IsEqual(new gp_Pnt(0, -3, 0), 1e-12));
        C.Perform(aCyl, new gp_Dir(0, 0, 1));
        Assert.Equal(0, C.NbContours());
        Assert.ThrowsAny<Standard_DomainError>(() => C.TypeContour());
    }
}
