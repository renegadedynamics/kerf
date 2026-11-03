// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>BRepAdaptor_Curve and BRepAdaptor_Curve2d on edges with 3D curves, at locations, and on pcurves only.</summary>
public class BRepAdaptor_CurveTests
{
    private static Placement SomeLocation()
    {
        Transform3D aT = new Transform3D();
        aT.SetRotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0)), 0.7);
        Transform3D aT2 = Transform3D.Translation(new Vector3D(5, -1, 2));
        return new Placement(aT2.Multiplied(aT));
    }

    /// <summary>A planar face on XOY whose edge from (0,0) to (3,4) has a pcurve and no 3D curve, frozen.</summary>
    private static (TopoDS_Edge Edge, TopoDS_Face Face) PCurveOnlyEdge()
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new Point3D(0, 0, 1), new Direction3D(0, 0, 1));
        TopoDS_Face aF = new TopoDS_Face();
        aB.MakeFace(ref aF, aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        TopoDS_Edge anE = new TopoDS_Edge();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(0, 0), new Direction2D(3, 4)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Range(anE, 0.0, 5.0);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0, 1)).Oriented(TopAbs_FORWARD));
        aB.Add(anE, TestShapes.Vertex(new Point3D(3, 4, 1)).Oriented(TopAbs_REVERSED));
        TopoDS_Wire aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        return (TopoDS.Edge(new TopExp_Explorer(aF, TopAbs_EDGE).Current()), aF);
    }

    [Fact]
    public void Edge_With3DCurve_IsTheCurve()
    {
        var aCircle = new Geom_Circle(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.0);
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aCircle, 0.5, 2.5));
        var aC = new BRepAdaptor_Curve(anE);
        Assert.True(aC.Is3DCurve());
        Assert.False(aC.IsCurveOnSurface());
        Assert.Equal(0.5, aC.FirstParameter());
        Assert.Equal(2.5, aC.LastParameter());
        Assert.Equal(Circle, aC.GetType());
        Assert.Same(aCircle, aC.GeomCurve());
        Assert.True(aC.Edge().IsEqual(anE));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aC.Tolerance());
        for (double u = 0.5; u <= 2.5; u += 0.25)
        {
            Assert.Equal(aCircle.Value(u), aC.Value(u));
            aC.D3(u, out Point3D aP, out Vector3D aV1, out Vector3D aV2, out Vector3D aV3);
            aCircle.D3(u, out Point3D aRP, out Vector3D aRV1, out Vector3D aRV2, out Vector3D aRV3);
            Assert.Equal(aRP, aP);
            Assert.Equal(aRV1, aV1);
            Assert.Equal(aRV2, aV2);
            Assert.Equal(aRV3, aV3);
        }
    }

    [Fact]
    public void LocatedAndReversedEdges()
    {
        var aLine = new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 2, 2));
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aLine, -1.0, 4.0));
        Placement aL = SomeLocation();
        TopoDS_Edge aMoved = TopoDS.Edge(anE.Moved(aL));
        var aC = new BRepAdaptor_Curve(aMoved);
        Assert.Equal(aL.Transformation().TranslationPart(), aC.Trsf().TranslationPart());
        for (double u = -1.0; u <= 4.0; u += 0.5)
        {
            Assert.True(aC.Value(u).Distance(aLine.Value(u).Transformed(aL.Transformation())) < 1e-13);
        }

        Line3D aLin = aC.Line();
        Assert.True(aLin.Direction().IsEqual(new Direction3D(1, 2, 2).Transformed(aL.Transformation()), 1e-14));

        // the orientation doesn't change the parametrisation
        var aR = new BRepAdaptor_Curve(TopoDS.Edge(aMoved.Reversed()));
        Assert.Equal(aC.Value(1.3), aR.Value(1.3));
        Assert.Equal(aC.DN(1.3, 1), aR.DN(1.3, 1));
    }

    [Fact]
    public void PCurveOnlyEdge_IsCurveOnSurface()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = PCurveOnlyEdge();
        var aC = new BRepAdaptor_Curve(anE);
        Assert.True(aC.IsCurveOnSurface());
        Assert.False(aC.Is3DCurve());
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(5.0, aC.LastParameter());
        Assert.True(aC.Value(5.0).Distance(new Point3D(3, 4, 1)) < 1e-14);
        Assert.Equal(Line, aC.GetType());

        // the same through the face, and at a location
        var aCF = new BRepAdaptor_Curve(anE, aF);
        Assert.Equal(aC.Value(2.0), aCF.Value(2.0));
        Placement aL = SomeLocation();
        var aCL = new BRepAdaptor_Curve(TopoDS.Edge(anE.Moved(aL)));
        Assert.True(aCL.Value(2.0).Distance(aC.Value(2.0).Transformed(aL.Transformation())) < 1e-13);

        var aT = (BRepAdaptor_Curve)aC.Trim(1.0, 2.0, 1e-9);
        Assert.True(aT.IsCurveOnSurface());
        Assert.Equal(1.0, aT.FirstParameter());
        Assert.Equal(2.0, aT.LastParameter());
        Assert.True(aT.Edge().IsSame(anE));
        Assert.Equal(aC.Value(1.5), aT.Value(1.5));

        var aCopy = (BRepAdaptor_Curve)aC.ShallowCopy();
        Assert.True(aCopy.IsCurveOnSurface());
        Assert.Equal(aC.Value(3.5), aCopy.Value(3.5));
    }

    [Fact]
    public void Trim_KeepsEdgeAndLocation()
    {
        var aLine = new Geom_Line(new Point3D(1, 0, 0), new Direction3D(0, 1, 0));
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aLine, 0.0, 10.0));
        Placement aL = SomeLocation();
        var aC = new BRepAdaptor_Curve(TopoDS.Edge(anE.Moved(aL)));
        var aT = (BRepAdaptor_Curve)aC.Trim(2.0, 3.0, 1e-9);
        Assert.Equal(2.0, aT.FirstParameter());
        Assert.Equal(3.0, aT.LastParameter());
        Assert.Equal(aC.Value(2.5), aT.Value(2.5));
        Assert.True(aT.Edge().IsEqual(aC.Edge()));
        // the original is unchanged
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(10.0, aC.LastParameter());
    }

    [Fact]
    public void Reset_And_NoGeometry()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = new TopoDS_Edge();
        aB.MakeEdge(ref anE);
        Assert.Throws<global::Kerf.Engine.Foundation.NullObjectException>(() => new BRepAdaptor_Curve(anE));

        TopoDS_Edge aLine = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        var aC = new BRepAdaptor_Curve(aLine);
        aC.Reset();
        Assert.True(aC.Edge().IsNull());
        Assert.Null(aC.GeomCurve());
    }

    [Fact]
    public void Curve2d_IsThePCurve()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        int aCount = 0;
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next(), aCount++)
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            var aC2 = new BRepAdaptor_Curve2d(anE, aFace);
            Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anE, aFace, out double f, out double l);
            Assert.Equal(f, aC2.FirstParameter());
            Assert.Equal(l, aC2.LastParameter());
            Assert.Equal(aPC!.Value(f), aC2.Value(f));
            Assert.True(aC2.Edge().IsEqual(anE));
            Assert.True(aC2.Face().IsEqual(aFace));
            var aCopy = (BRepAdaptor_Curve2d)aC2.ShallowCopy();
            Assert.Equal(aC2.Value(l), aCopy.Value(l));
            Assert.True(aCopy.Edge().IsEqual(anE));

            // the 3D adaptor through the face agrees with the 3D curve on the surface
            var aC3 = new BRepAdaptor_Curve(anE, aFace);
            var aC3d = new BRepAdaptor_Curve(anE);
            double u = 0.3 * f + 0.7 * l;
            Assert.True(aC3.Value(u).Distance(aC3d.Value(u)) < 1e-12);
        }
        Assert.Equal(4, aCount);

        // the seam's two pcurves: FORWARD at U = 2 Pi, REVERSED at U = 0
        TopoDS_Edge aSeam = TopoDS.Edge(TopExp_Explorer_At(aFace, 1));
        Assert.Equal(Line, new BRepAdaptor_Curve2d(aSeam, aFace).GetType());
        double aU = new BRepAdaptor_Curve2d(TopoDS.Edge(aSeam.Oriented(TopAbs_FORWARD)), aFace).Value(1.0).X();
        double aUr = new BRepAdaptor_Curve2d(TopoDS.Edge(aSeam.Oriented(TopAbs_REVERSED)), aFace).Value(1.0).X();
        Assert.Equal(2 * Math.PI, aU, 1e-15);
        Assert.Equal(0.0, aUr, 1e-15);

        // no pcurve on the face: nothing is loaded
        TopoDS_Edge aLine = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        var anEmpty = new BRepAdaptor_Curve2d(aLine, aFace);
        Assert.Null(anEmpty.Curve());
    }

    private static TopoDS_Shape TopExp_Explorer_At(in TopoDS_Shape theS, int theIndex)
    {
        var anExp = new TopExp_Explorer(theS, TopAbs_EDGE);
        for (int i = 0; i < theIndex; i++)
        {
            anExp.Next();
        }
        return anExp.Current();
    }

    [Fact]
    public void Intervals_And_Resolution()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new Point3D(i, i * i % 3, 0);
        }
        var aKnots = new NCollection_Array1<double>(1, 4);
        var aMults = new NCollection_Array1<int>(1, 4);
        aKnots[1] = 0; aKnots[2] = 1; aKnots[3] = 2; aKnots[4] = 3;
        aMults[1] = 3; aMults[2] = 1; aMults[3] = 1; aMults[4] = 3;
        var aBS = new Geom_BSplineCurve(aPoles, aKnots, aMults, 2);
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aBS, 0.0, 3.0));
        var aC = new BRepAdaptor_Curve(TopoDS.Edge(anE.Moved(SomeLocation())));
        Assert.Equal(BSplineCurve, aC.GetType());
        Assert.Equal(3, aC.NbIntervals(C2));
        Assert.Equal(1, aC.NbIntervals(C1));
        Assert.Equal(new GeomAdaptor_Curve(aBS).Resolution(0.1), aC.Resolution(0.1));
        // the B-spline is handed out transformed
        Geom_BSplineCurve aT = aC.BSpline();
        Assert.True(aT.Value(1.5).Distance(aC.Value(1.5)) < 1e-13);
    }
}
