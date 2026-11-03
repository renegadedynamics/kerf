// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>Shapes for the BOPTools tests: primitives and data/occ shapes.</summary>
internal static class BOPToolsTestShapes
{
    public static Shape Box() => new MakeBox(new Point3D(-1, -2, -3), 2, 4, 6).Shape();
    public static Shape Cylinder() => new ApiMakeCylinder(2, 5).Shape();
    public static Shape Sphere() => new MakeSphere(3).Shape();
    public static Shape Cone() => new ApiMakeCone(3, 1, 4).Shape();
    public static Shape Torus() => new MakeTorus(5, 1.5).Shape();

    public static IEnumerable<(string Name, Shape Shape)> Primitives()
    {
        yield return ("box", Box());
        yield return ("cylinder", Cylinder());
        yield return ("sphere", Sphere());
        yield return ("cone", Cone());
        yield return ("torus", Torus());
    }

    public static readonly string[] DataOccFiles = ["bottle.brep", "Pump_Nut.brep", "face1.brep", "face2.brep", "wing.brep", "mal_ecrou.brep"];

    public static IEnumerable<(string Name, Shape Shape)> DataOcc()
    {
        foreach (string aFile in DataOccFiles)
        {
            yield return (aFile, BRepMeshModelKit.ReadDataOcc(aFile));
        }
    }

    public static List<Face> Faces(in Shape theShape)
    {
        List<Face> aFaces = [];
        for (Explorer anExp = new(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(ShapeCasts.Face(anExp.Current()));
        }
        return aFaces;
    }

    public static List<Edge> Edges(in Shape theShape)
    {
        List<Edge> anEdges = [];
        for (Explorer anExp = new(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            anEdges.Add(ShapeCasts.Edge(anExp.Current()));
        }
        return anEdges;
    }

    /// <summary>The state of the 2D point in the face (taken FORWARD) by BRepClass_FaceClassifier.</summary>
    public static Containment Classify(in Face theFace, in Point2D theP)
    {
        Face aFF = theFace;
        aFF.Orientation(Forward);
        FaceClassifier aClassifier = new();
        aClassifier.Perform(aFF, theP, global::Kerf.Engine.Foundation.Precision.PConfusion());
        return aClassifier.State();
    }

    public static void SameBits(in Point3D theA, in Point3D theB)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.X()), BitConverter.DoubleToInt64Bits(theB.X()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Y()), BitConverter.DoubleToInt64Bits(theB.Y()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Z()), BitConverter.DoubleToInt64Bits(theB.Z()));
    }

    public static void SameBits(in Point2D theA, in Point2D theB)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.X()), BitConverter.DoubleToInt64Bits(theB.X()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Y()), BitConverter.DoubleToInt64Bits(theB.Y()));
    }
}

/// <summary>BOPTools_AlgoTools3D: normals along edges, points in faces, seams.</summary>
public class BOPTools_AlgoTools3DTests
{
    private static void CheckPointInFace(in Face theFace, int theErr, in Point3D theP, in Point2D theP2D, string theWhat)
    {
        Assert.True(theErr == 0, $"{theWhat}: error {theErr}");
        Assert.Equal(In, BOPToolsTestShapes.Classify(theFace, theP2D));
        Point3D aPS = Tool.Surface(theFace)!.Value(theP2D.X(), theP2D.Y());
        BOPToolsTestShapes.SameBits(aPS, theP);
    }

    [Fact]
    public void PointInFace_gives_a_point_inside_every_face_of_the_primitives()
    {
        IntersectionContext aCtx = new();
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                Point3D aP = new();
                Point2D aP2D = new();
                int iErr = AlgoTools3D.PointInFace(aF, ref aP, ref aP2D, aCtx);
                CheckPointInFace(aF, iErr, aP, aP2D, aName);
            }
        }
    }

    [Fact]
    public void PointInFace_from_an_edge_steps_into_the_face()
    {
        IntersectionContext aCtx = new();
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    Tool.Range(aE, out double aT1, out double aT2);
                    double aT = AlgoTools2D.IntermediatePoint(aT1, aT2);
                    Point3D aP = new();
                    Point2D aP2D = new();
                    int iErr = AlgoTools3D.PointInFace(aF, aE, aT, 1.0e-4, ref aP, ref aP2D, aCtx);
                    CheckPointInFace(aF, iErr, aP, aP2D, aName);
                    // the point is 1e-4 from the edge's pcurve point (the domain is longer than that)
                    Tool.CurveOnSurface(aE, aF, out _, out _)!.D0(aT, out Point2D aPE);
                    Assert.Equal(1.0e-4, aPE.Distance(aP2D), 1.0e-12);
                }
            }
        }
    }

    [Fact]
    public void PointInFace_along_a_line_without_the_face_fails_and_keeps_the_outputs()
    {
        IntersectionContext aCtx = new();
        Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        // a line far outside the face's UV domain
        LineCurve2D aL = new(new Point2D(1000, 1000), new Direction2D(0, 1));
        Point3D aP = new(7, 8, 9);
        Point2D aP2D = new(7, 8);
        int iErr = AlgoTools3D.PointInFace(aF, aL, ref aP, ref aP2D, aCtx);
        Assert.NotEqual(0, iErr);
        Assert.Equal(new Point3D(7, 8, 9), aP);
        Assert.Equal(new Point2D(7, 8), aP2D);
        // no edge pcurve: 5
        // (a plane gives every edge a pcurve, so a torus face)
        Face aTorusFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        Edge aForeign = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        Assert.Equal(5, AlgoTools3D.PointInFace(aTorusFace, aForeign, 0.0, 1.0e-4, ref aP, ref aP2D, aCtx));
    }

    [Fact]
    public void PointInFace_on_data_occ_faces_gives_points_inside()
    {
        IntersectionContext aCtx = new();
        int aNbFaces = 0, aNbDone = 0;
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.DataOcc())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                ++aNbFaces;
                Point3D aP = new();
                Point2D aP2D = new();
                int iErr = AlgoTools3D.PointInFace(aF, ref aP, ref aP2D, aCtx);
                if (iErr != 0)
                {
                    continue;
                }
                ++aNbDone;
                Assert.NotEqual(Out, BOPToolsTestShapes.Classify(aF, aP2D));
                Point3D aPS = Tool.Surface(aF)!.Value(aP2D.X(), aP2D.Y());
                BOPToolsTestShapes.SameBits(aPS, aP);
                _ = aName;
            }
        }
        Assert.True(aNbFaces > 100, $"{aNbFaces} faces");
        Assert.True(aNbDone >= 0.95 * aNbFaces, $"{aNbDone} of {aNbFaces} faces");
    }

    /// <summary>
    /// The hatcher audit (plan 2.4, decision 670): a context reused over many faces, in shuffled order and twice per
    /// face, answers bit for bit as a fresh context per call.
    /// </summary>
    [Fact]
    public void PointInFace_through_a_reused_context_matches_fresh_contexts()
    {
        List<Face> aFaces = [];
        foreach ((_, Shape aShape) in BOPToolsTestShapes.Primitives().Concat(BOPToolsTestShapes.DataOcc()))
        {
            aFaces.AddRange(BOPToolsTestShapes.Faces(aShape));
        }
        // fresh answers
        int aN = aFaces.Count;
        var aFresh = new (int Err, Point3D P, Point2D P2D)[aN];
        var aFreshEdge = new (int Err, Point3D P, Point2D P2D)[aN];
        for (int i = 0; i < aN; ++i)
        {
            Point3D aP = new();
            Point2D aP2D = new();
            int iErr = AlgoTools3D.PointInFace(aFaces[i], ref aP, ref aP2D, new IntersectionContext());
            aFresh[i] = (iErr, aP, aP2D);
            aFreshEdge[i] = EdgeVariant(aFaces[i], new IntersectionContext());
        }
        // one context, shuffled order, every face twice
        IntersectionContext aCtx = new();
        Random aRandom = new(6420);
        int[] anOrder = [.. Enumerable.Range(0, aN).Concat(Enumerable.Range(0, aN)).OrderBy(_ => aRandom.Next())];
        foreach (int i in anOrder)
        {
            Point3D aP = new();
            Point2D aP2D = new();
            int iErr = AlgoTools3D.PointInFace(aFaces[i], ref aP, ref aP2D, aCtx);
            Assert.Equal(aFresh[i].Err, iErr);
            BOPToolsTestShapes.SameBits(aFresh[i].P, aP);
            BOPToolsTestShapes.SameBits(aFresh[i].P2D, aP2D);
            (int Err, Point3D P, Point2D P2D) anEdge = EdgeVariant(aFaces[i], aCtx);
            Assert.Equal(aFreshEdge[i].Err, anEdge.Err);
            BOPToolsTestShapes.SameBits(aFreshEdge[i].P, anEdge.P);
            BOPToolsTestShapes.SameBits(aFreshEdge[i].P2D, anEdge.P2D);
        }
    }

    // PointInFace from the face's first non-degenerated edge with a pcurve.
    private static (int Err, Point3D P, Point2D P2D) EdgeVariant(in Face theFace, IntersectionContext theCtx)
    {
        Point3D aP = new();
        Point2D aP2D = new();
        foreach (Edge aE in BOPToolsTestShapes.Edges(theFace))
        {
            if (Tool.Degenerated(aE) || Tool.CurveOnSurface(aE, theFace, out double aT1, out double aT2) is null)
            {
                continue;
            }
            int iErr = AlgoTools3D.PointInFace(theFace, aE, AlgoTools2D.IntermediatePoint(aT1, aT2), 1.0e-4,
                                                       ref aP, ref aP2D, theCtx);
            return (iErr, aP, aP2D);
        }
        return (-1, aP, aP2D);
    }

    [Fact]
    public void PointNearEdge_gives_a_point_of_the_face_next_to_the_edge()
    {
        IntersectionContext aCtx = new();
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    Point3D aP = new();
                    Point2D aP2D = new();
                    int iErr = AlgoTools3D.PointNearEdge(aE, aF, ref aP2D, ref aP, aCtx);
                    Assert.True(iErr == 0, $"{aName}: {iErr}");
                    Assert.NotEqual(Out, BOPToolsTestShapes.Classify(aF, aP2D));
                    BOPToolsTestShapes.SameBits(Tool.Surface(aF)!.Value(aP2D.X(), aP2D.Y()), aP);
                    // close to the edge
                    Tool.Range(aE, out double aT1, out double aT2);
                    Point3D aPE = Tool.Curve(aE, out _, out _)!.Value(AlgoTools2D.IntermediatePoint(aT1, aT2));
                    Assert.True(aPE.Distance(aP) < 0.05, $"{aName}: {aPE.Distance(aP)}");
                }
            }
        }
    }

    [Fact]
    public void PointNearEdge_without_context_steps_perpendicular_to_the_pcurve()
    {
        // a box face: planar pcurves (lines), tolerances 1e-7, so the step is exactly aDt2D
        foreach (Face aF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (Edge anE in BOPToolsTestShapes.Edges(aF))
            {
                AlgoTools3D.OrientEdgeOnFace(anE, aF, out Edge aE);
                Point3D aP = new();
                Point2D aP2D = new();
                Assert.Equal(0, AlgoTools3D.PointNearEdge(aE, aF, 1.0, 0.01, ref aP2D, ref aP));
                Tool.CurveOnSurface(aE, aF, out _, out _)!.D0(1.0, out Point2D aPE);
                Assert.Equal(0.01, aPE.Distance(aP2D), 1.0e-12);
                Assert.Equal(In, BOPToolsTestShapes.Classify(aF, aP2D));
            }
        }
        // an edge without pcurve on the face: 1, outputs kept
        Face aBoxFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        Edge aForeign = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        Point3D aKeep = new(1, 2, 3);
        Point2D aKeep2D = new(4, 5);
        Assert.Equal(1, AlgoTools3D.PointNearEdge(aForeign, aBoxFace, 0.5, 0.01, ref aKeep2D, ref aKeep));
        Assert.Equal(new Point3D(1, 2, 3), aKeep);
        Assert.Equal(new Point2D(4, 5), aKeep2D);
    }

    // The outward normal of the primitive solids at a point of their surface.
    private static Direction3D OutwardNormal(string theName, in Point3D theP, in Face theF, double theU, double theV)
    {
        switch (theName)
        {
            case "sphere":
                return new Direction3D(theP.XYZ());
            case "box":
                {
                    // the face's plane normal, outward
                    Surface aS = Tool.Surface(theF)!;
                    aS.D1(theU, theV, out _, out Vector3D aDU, out Vector3D aDV);
                    Direction3D aN = new(aDU.Crossed(aDV));
                    return theF.Orientation() == Reversed ? aN.Reversed() : aN;
                }
            default:
                throw new InvalidOperationException(theName);
        }
    }

    [Fact]
    public void GetNormalToFaceOnEdge_gives_the_outward_normal_of_box_and_sphere_faces()
    {
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives().Where(s => s.Name is "box" or "sphere"))
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, out Direction3D aN);
                    Curve2D aPC = Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2)!;
                    aPC.D0(AlgoTools2D.IntermediatePoint(aT1, aT2), out Point2D aUV);
                    Point3D aP = Tool.Surface(aF)!.Value(aUV.X(), aUV.Y());
                    Direction3D anExpected = OutwardNormal(aName, aP, aF, aUV.X(), aUV.Y());
                    Assert.True(aN.IsEqual(anExpected, 1.0e-9), $"{aName}: {aN} vs {anExpected}");
                    if (aName == "box")
                    {
                        // the box's outward normals point away from its centre (0, 0, 0)
                        Assert.True(aN.XYZ().Dot(aP.XYZ()) > 0);
                    }

                    // the parameter overload doesn't reverse for the face's orientation
                    AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, AlgoTools2D.IntermediatePoint(aT1, aT2), out Direction3D aN2,
                                                               new IntersectionContext());
                    Direction3D aN2Oriented = aF.Orientation() == Reversed ? aN2.Reversed() : aN2;
                    Assert.Equal(aN, aN2Oriented);
                }
            }
        }
    }

    [Fact]
    public void GetApproxNormalToFaceOnEdge_gives_a_point_inside_and_its_normal()
    {
        IntersectionContext aCtx = new();
        foreach (Face aF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (Edge anE in BOPToolsTestShapes.Edges(aF))
            {
                AlgoTools3D.OrientEdgeOnFace(anE, aF, out Edge aE);
                Tool.Range(aE, out double aT1, out double aT2);
                double aT = AlgoTools2D.IntermediatePoint(aT1, aT2);
                AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, out Direction3D anExact);

                Point3D aP = new();
                Direction3D aN = new();
                Assert.True(AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, ref aP, ref aN, aCtx));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                Assert.True(AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, 1.0e-3, ref aP, ref aN, aCtx));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                Assert.True(AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, ref aP, ref aN, 1.0e-3));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                Point3D aPE = Tool.Curve(aE, out _, out _)!.Value(aT);
                Assert.Equal(1.0e-3, aPE.Distance(aP), 1.0e-12);
            }
        }
    }

    [Fact]
    public void SenseFlag_and_GetNormalToSurface_known_answers()
    {
        Direction3D aZ = new(0, 0, 1);
        Assert.Equal(1, AlgoTools3D.SenseFlag(aZ, aZ));
        Assert.Equal(-1, AlgoTools3D.SenseFlag(aZ, aZ.Reversed()));
        Assert.Equal(0, AlgoTools3D.SenseFlag(aZ, new Direction3D(1, 0, 0)));
        Assert.Equal(1, AlgoTools3D.SenseFlag(aZ, new Direction3D(1.0e-5, 0, 1)));

        SphericalSurface aS = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        ConicalSurface aCone = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5, 0.0);
        Direction3D aD = new(0, 1, 0);
        Assert.True(AlgoTools3D.GetNormalToSurface(aS, 0.3, 0.2, ref aD));
        Point3D aP = aS.Value(0.3, 0.2);
        Assert.True(aD.IsEqual(new Direction3D(aP.XYZ()), 1.0e-12));
        // at the apex of a cone, DU vanishes: false and the output unchanged
        Direction3D aKeep = new(0, 1, 0);
        Assert.False(AlgoTools3D.GetNormalToSurface(aCone, 0.3, 0.0, ref aKeep));
        Assert.Equal(new Direction3D(0, 1, 0), aKeep);
    }

    [Fact]
    public void OrientEdgeOnFace_gives_the_orientation_in_the_face()
    {
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    Edge aFwd = ShapeCasts.Edge(aE.Oriented(Forward));
                    AlgoTools3D.OrientEdgeOnFace(aFwd, aF, out Edge aR);
                    Assert.True(aR.IsSame(aE), aName);
                    if (Tool.IsClosed(aE, aF))
                    {
                        // a seam found twice keeps the given orientation
                        Assert.Equal(Forward, aR.Orientation());
                    }
                    else
                    {
                        Assert.Equal(aE.Orientation(), aR.Orientation());
                    }
                }
            }
        }
        // an edge not in the face keeps its orientation
        Face aBoxFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        Edge aForeign = ShapeCasts.Edge(BOPToolsTestShapes.Edges(BOPToolsTestShapes.Cone())[0].Oriented(Reversed));
        AlgoTools3D.OrientEdgeOnFace(aForeign, aBoxFace, out Edge aRF);
        Assert.Equal(Reversed, aRF.Orientation());
    }

    [Fact]
    public void MinStepIn2d_and_IsEmptyShape()
    {
        Assert.Equal(1.0e-5, AlgoTools3D.MinStepIn2d());
        Assert.False(AlgoTools3D.IsEmptyShape(BOPToolsTestShapes.Box()));
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        Assert.True(AlgoTools3D.IsEmptyShape(aC));
        Compound aC2 = new();
        aBB.MakeCompound(ref aC2);
        aBB.Add(aC2, aC);
        Assert.True(AlgoTools3D.IsEmptyShape(aC2));
        Compound aC3 = new();
        aBB.MakeCompound(ref aC3);
        aBB.Add(aC3, aC2);
        aBB.Add(aC3, BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0]);
        Assert.False(AlgoTools3D.IsEmptyShape(aC3));
        Assert.True(AlgoTools3D.IsEmptyShape(new Shape()));
    }

    // The cylinder's lateral face and its seam edge, unfrozen.
    private static (Face Face, Edge Seam) CylinderSeam()
    {
        Shape aCyl = new ApiMakeCylinder(2, 5).InternalShape();
        foreach (Face aF in BOPToolsTestShapes.Faces(aCyl))
        {
            if (Tool.Surface(aF) is not CylindricalSurface)
            {
                continue;
            }
            foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
            {
                if (Tool.IsClosed(aE, aF))
                {
                    return (aF, aE);
                }
            }
        }
        throw new InvalidOperationException();
    }

    // A new edge on part of the seam's line with only the seam's FORWARD pcurve (or its translation by the period).
    private static Edge SplitOfSeam(in Face theFace, in Edge theSeam, double theT1, double theT2,
                                           Orientation theWhich = Forward)
    {
        BrepBuilder aBB = new();
        Curve3D aC = Tool.Curve(theSeam, out _, out _)!;
        Curve2D aPC = (Curve2D)Tool.CurveOnSurface(ShapeCasts.Edge(theSeam.Oriented(theWhich)), theFace, out _, out _)!.Copy();
        Edge aE = new();
        aBB.MakeEdge(ref aE, aC, 1.0e-7);
        aBB.Range(aE, theT1, theT2);
        aBB.UpdateEdge(aE, aPC, theFace, 1.0e-7);
        return aE;
    }

    [Fact]
    public void DoSplitSEAMOnFace_makes_the_split_a_seam()
    {
        (Face aF, Edge aSeam) = CylinderSeam();
        foreach (Orientation aWhich in new[] { Forward, Reversed })
        {
            Edge aSplit = SplitOfSeam(aF, aSeam, 1.0, 3.0, aWhich);
            Assert.False(Tool.IsClosed(aSplit, aF));
            Assert.True(AlgoTools3D.DoSplitSEAMOnFace(aSplit, aF));
            Assert.True(Tool.IsClosed(aSplit, aF));
            // the two pcurves are 2 pi apart and ordered as the seam's
            Curve2D aP1 = Tool.CurveOnSurface(ShapeCasts.Edge(aSplit.Oriented(Forward)), aF, out _, out _)!;
            Curve2D aP2 = Tool.CurveOnSurface(ShapeCasts.Edge(aSplit.Oriented(Reversed)), aF, out _, out _)!;
            Curve2D aS1 = Tool.CurveOnSurface(ShapeCasts.Edge(aSeam.Oriented(Forward)), aF, out _, out _)!;
            Curve2D aS2 = Tool.CurveOnSurface(ShapeCasts.Edge(aSeam.Oriented(Reversed)), aF, out _, out _)!;
            Assert.Equal(aS1.Value(2.0).X(), aP1.Value(2.0).X(), 1.0e-12);
            Assert.Equal(aS2.Value(2.0).X(), aP2.Value(2.0).X(), 1.0e-12);
        }

        // the two-edge form
        Edge aSplit2 = SplitOfSeam(aF, aSeam, 0.5, 4.0);
        Assert.True(AlgoTools3D.DoSplitSEAMOnFace(aSeam, aSplit2, aF));
        Assert.True(Tool.IsClosed(aSplit2, aF));
        Curve2D aQ1 = Tool.CurveOnSurface(ShapeCasts.Edge(aSplit2.Oriented(Forward)), aF, out _, out _)!;
        Curve2D aT1 = Tool.CurveOnSurface(ShapeCasts.Edge(aSeam.Oriented(Forward)), aF, out _, out _)!;
        Assert.Equal(aT1.Value(2.0).X(), aQ1.Value(2.0).X(), 1.0e-12);
        // already a seam: true without change; origin not a seam: false
        Assert.True(AlgoTools3D.DoSplitSEAMOnFace(aSeam, aSplit2, aF));
        Edge aNotSeam = BOPToolsTestShapes.Edges(aF).First(e => !Tool.IsClosed(e, aF) && !Tool.Degenerated(e));
        Assert.False(AlgoTools3D.DoSplitSEAMOnFace(aNotSeam, SplitOfSeam(aF, aSeam, 1.0, 2.0), aF));
    }

    [Fact]
    public void DoSplitSEAMOnFace_does_nothing_on_planes_and_throws_on_frozen_edges()
    {
        Shape aBox = new MakeBox(1, 1, 1).InternalShape();
        Face aF = BOPToolsTestShapes.Faces(aBox)[0];
        Assert.False(AlgoTools3D.DoSplitSEAMOnFace(BOPToolsTestShapes.Edges(aF)[0], aF));

        // a frozen split is written into: it throws (plan 2.3, ported callers give it their own edges)
        Shape aCyl = BOPToolsTestShapes.Cylinder();
        Face aCF = BOPToolsTestShapes.Faces(aCyl).First(f => Tool.Surface(f) is CylindricalSurface);
        Edge aSeam = BOPToolsTestShapes.Edges(aCF).First(e => Tool.IsClosed(e, aCF));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => AlgoTools3D.DoSplitSEAMOnFace(aSeam, aCF));
    }
}
