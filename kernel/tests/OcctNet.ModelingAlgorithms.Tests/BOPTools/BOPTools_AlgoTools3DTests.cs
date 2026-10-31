// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>Shapes for the BOPTools tests: primitives and data/occ shapes.</summary>
internal static class BOPToolsTestShapes
{
    public static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(new gp_Pnt(-1, -2, -3), 2, 4, 6).Shape();
    public static TopoDS_Shape Cylinder() => new BRepPrimAPI_MakeCylinder(2, 5).Shape();
    public static TopoDS_Shape Sphere() => new BRepPrimAPI_MakeSphere(3).Shape();
    public static TopoDS_Shape Cone() => new BRepPrimAPI_MakeCone(3, 1, 4).Shape();
    public static TopoDS_Shape Torus() => new BRepPrimAPI_MakeTorus(5, 1.5).Shape();

    public static IEnumerable<(string Name, TopoDS_Shape Shape)> Primitives()
    {
        yield return ("box", Box());
        yield return ("cylinder", Cylinder());
        yield return ("sphere", Sphere());
        yield return ("cone", Cone());
        yield return ("torus", Torus());
    }

    public static readonly string[] DataOccFiles = ["bottle.brep", "Pump_Nut.brep", "face1.brep", "face2.brep", "wing.brep", "mal_ecrou.brep"];

    public static IEnumerable<(string Name, TopoDS_Shape Shape)> DataOcc()
    {
        foreach (string aFile in DataOccFiles)
        {
            yield return (aFile, BRepMeshModelKit.ReadDataOcc(aFile));
        }
    }

    public static List<TopoDS_Face> Faces(in TopoDS_Shape theShape)
    {
        List<TopoDS_Face> aFaces = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaces.Add(TopoDS.Face(anExp.Current()));
        }
        return aFaces;
    }

    public static List<TopoDS_Edge> Edges(in TopoDS_Shape theShape)
    {
        List<TopoDS_Edge> anEdges = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            anEdges.Add(TopoDS.Edge(anExp.Current()));
        }
        return anEdges;
    }

    /// <summary>The state of the 2D point in the face (taken FORWARD) by BRepClass_FaceClassifier.</summary>
    public static TopAbs_State Classify(in TopoDS_Face theFace, in gp_Pnt2d theP)
    {
        TopoDS_Face aFF = theFace;
        aFF.Orientation(TopAbs_FORWARD);
        BRepClass_FaceClassifier aClassifier = new();
        aClassifier.Perform(aFF, theP, Precision.PConfusion());
        return aClassifier.State();
    }

    public static void SameBits(in gp_Pnt theA, in gp_Pnt theB)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.X()), BitConverter.DoubleToInt64Bits(theB.X()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Y()), BitConverter.DoubleToInt64Bits(theB.Y()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Z()), BitConverter.DoubleToInt64Bits(theB.Z()));
    }

    public static void SameBits(in gp_Pnt2d theA, in gp_Pnt2d theB)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.X()), BitConverter.DoubleToInt64Bits(theB.X()));
        Assert.Equal(BitConverter.DoubleToInt64Bits(theA.Y()), BitConverter.DoubleToInt64Bits(theB.Y()));
    }
}

/// <summary>BOPTools_AlgoTools3D: normals along edges, points in faces, seams.</summary>
public class BOPTools_AlgoTools3DTests
{
    private static void CheckPointInFace(in TopoDS_Face theFace, int theErr, in gp_Pnt theP, in gp_Pnt2d theP2D, string theWhat)
    {
        Assert.True(theErr == 0, $"{theWhat}: error {theErr}");
        Assert.Equal(TopAbs_IN, BOPToolsTestShapes.Classify(theFace, theP2D));
        gp_Pnt aPS = BRep_Tool.Surface(theFace)!.Value(theP2D.X(), theP2D.Y());
        BOPToolsTestShapes.SameBits(aPS, theP);
    }

    [Fact]
    public void PointInFace_gives_a_point_inside_every_face_of_the_primitives()
    {
        IntTools_Context aCtx = new();
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                gp_Pnt aP = new();
                gp_Pnt2d aP2D = new();
                int iErr = BOPTools_AlgoTools3D.PointInFace(aF, ref aP, ref aP2D, aCtx);
                CheckPointInFace(aF, iErr, aP, aP2D, aName);
            }
        }
    }

    [Fact]
    public void PointInFace_from_an_edge_steps_into_the_face()
    {
        IntTools_Context aCtx = new();
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (BRep_Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    BRep_Tool.Range(aE, out double aT1, out double aT2);
                    double aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
                    gp_Pnt aP = new();
                    gp_Pnt2d aP2D = new();
                    int iErr = BOPTools_AlgoTools3D.PointInFace(aF, aE, aT, 1.0e-4, ref aP, ref aP2D, aCtx);
                    CheckPointInFace(aF, iErr, aP, aP2D, aName);
                    // the point is 1e-4 from the edge's pcurve point (the domain is longer than that)
                    BRep_Tool.CurveOnSurface(aE, aF, out _, out _)!.D0(aT, out gp_Pnt2d aPE);
                    Assert.Equal(1.0e-4, aPE.Distance(aP2D), 1.0e-12);
                }
            }
        }
    }

    [Fact]
    public void PointInFace_along_a_line_without_the_face_fails_and_keeps_the_outputs()
    {
        IntTools_Context aCtx = new();
        TopoDS_Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        // a line far outside the face's UV domain
        Geom2d_Line aL = new(new gp_Pnt2d(1000, 1000), new gp_Dir2d(0, 1));
        gp_Pnt aP = new(7, 8, 9);
        gp_Pnt2d aP2D = new(7, 8);
        int iErr = BOPTools_AlgoTools3D.PointInFace(aF, aL, ref aP, ref aP2D, aCtx);
        Assert.NotEqual(0, iErr);
        Assert.Equal(new gp_Pnt(7, 8, 9), aP);
        Assert.Equal(new gp_Pnt2d(7, 8), aP2D);
        // no edge pcurve: 5
        // (a plane gives every edge a pcurve, so a torus face)
        TopoDS_Face aTorusFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        TopoDS_Edge aForeign = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        Assert.Equal(5, BOPTools_AlgoTools3D.PointInFace(aTorusFace, aForeign, 0.0, 1.0e-4, ref aP, ref aP2D, aCtx));
    }

    [Fact]
    public void PointInFace_on_data_occ_faces_gives_points_inside()
    {
        IntTools_Context aCtx = new();
        int aNbFaces = 0, aNbDone = 0;
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.DataOcc())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                ++aNbFaces;
                gp_Pnt aP = new();
                gp_Pnt2d aP2D = new();
                int iErr = BOPTools_AlgoTools3D.PointInFace(aF, ref aP, ref aP2D, aCtx);
                if (iErr != 0)
                {
                    continue;
                }
                ++aNbDone;
                Assert.NotEqual(TopAbs_OUT, BOPToolsTestShapes.Classify(aF, aP2D));
                gp_Pnt aPS = BRep_Tool.Surface(aF)!.Value(aP2D.X(), aP2D.Y());
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
        List<TopoDS_Face> aFaces = [];
        foreach ((_, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives().Concat(BOPToolsTestShapes.DataOcc()))
        {
            aFaces.AddRange(BOPToolsTestShapes.Faces(aShape));
        }
        // fresh answers
        int aN = aFaces.Count;
        var aFresh = new (int Err, gp_Pnt P, gp_Pnt2d P2D)[aN];
        var aFreshEdge = new (int Err, gp_Pnt P, gp_Pnt2d P2D)[aN];
        for (int i = 0; i < aN; ++i)
        {
            gp_Pnt aP = new();
            gp_Pnt2d aP2D = new();
            int iErr = BOPTools_AlgoTools3D.PointInFace(aFaces[i], ref aP, ref aP2D, new IntTools_Context());
            aFresh[i] = (iErr, aP, aP2D);
            aFreshEdge[i] = EdgeVariant(aFaces[i], new IntTools_Context());
        }
        // one context, shuffled order, every face twice
        IntTools_Context aCtx = new();
        Random aRandom = new(6420);
        int[] anOrder = [.. Enumerable.Range(0, aN).Concat(Enumerable.Range(0, aN)).OrderBy(_ => aRandom.Next())];
        foreach (int i in anOrder)
        {
            gp_Pnt aP = new();
            gp_Pnt2d aP2D = new();
            int iErr = BOPTools_AlgoTools3D.PointInFace(aFaces[i], ref aP, ref aP2D, aCtx);
            Assert.Equal(aFresh[i].Err, iErr);
            BOPToolsTestShapes.SameBits(aFresh[i].P, aP);
            BOPToolsTestShapes.SameBits(aFresh[i].P2D, aP2D);
            (int Err, gp_Pnt P, gp_Pnt2d P2D) anEdge = EdgeVariant(aFaces[i], aCtx);
            Assert.Equal(aFreshEdge[i].Err, anEdge.Err);
            BOPToolsTestShapes.SameBits(aFreshEdge[i].P, anEdge.P);
            BOPToolsTestShapes.SameBits(aFreshEdge[i].P2D, anEdge.P2D);
        }
    }

    // PointInFace from the face's first non-degenerated edge with a pcurve.
    private static (int Err, gp_Pnt P, gp_Pnt2d P2D) EdgeVariant(in TopoDS_Face theFace, IntTools_Context theCtx)
    {
        gp_Pnt aP = new();
        gp_Pnt2d aP2D = new();
        foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(theFace))
        {
            if (BRep_Tool.Degenerated(aE) || BRep_Tool.CurveOnSurface(aE, theFace, out double aT1, out double aT2) is null)
            {
                continue;
            }
            int iErr = BOPTools_AlgoTools3D.PointInFace(theFace, aE, BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2), 1.0e-4,
                                                       ref aP, ref aP2D, theCtx);
            return (iErr, aP, aP2D);
        }
        return (-1, aP, aP2D);
    }

    [Fact]
    public void PointNearEdge_gives_a_point_of_the_face_next_to_the_edge()
    {
        IntTools_Context aCtx = new();
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (BRep_Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    gp_Pnt aP = new();
                    gp_Pnt2d aP2D = new();
                    int iErr = BOPTools_AlgoTools3D.PointNearEdge(aE, aF, ref aP2D, ref aP, aCtx);
                    Assert.True(iErr == 0, $"{aName}: {iErr}");
                    Assert.NotEqual(TopAbs_OUT, BOPToolsTestShapes.Classify(aF, aP2D));
                    BOPToolsTestShapes.SameBits(BRep_Tool.Surface(aF)!.Value(aP2D.X(), aP2D.Y()), aP);
                    // close to the edge
                    BRep_Tool.Range(aE, out double aT1, out double aT2);
                    gp_Pnt aPE = BRep_Tool.Curve(aE, out _, out _)!.Value(BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2));
                    Assert.True(aPE.Distance(aP) < 0.05, $"{aName}: {aPE.Distance(aP)}");
                }
            }
        }
    }

    [Fact]
    public void PointNearEdge_without_context_steps_perpendicular_to_the_pcurve()
    {
        // a box face: planar pcurves (lines), tolerances 1e-7, so the step is exactly aDt2D
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (TopoDS_Edge anE in BOPToolsTestShapes.Edges(aF))
            {
                BOPTools_AlgoTools3D.OrientEdgeOnFace(anE, aF, out TopoDS_Edge aE);
                gp_Pnt aP = new();
                gp_Pnt2d aP2D = new();
                Assert.Equal(0, BOPTools_AlgoTools3D.PointNearEdge(aE, aF, 1.0, 0.01, ref aP2D, ref aP));
                BRep_Tool.CurveOnSurface(aE, aF, out _, out _)!.D0(1.0, out gp_Pnt2d aPE);
                Assert.Equal(0.01, aPE.Distance(aP2D), 1.0e-12);
                Assert.Equal(TopAbs_IN, BOPToolsTestShapes.Classify(aF, aP2D));
            }
        }
        // an edge without pcurve on the face: 1, outputs kept
        TopoDS_Face aBoxFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        TopoDS_Edge aForeign = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        gp_Pnt aKeep = new(1, 2, 3);
        gp_Pnt2d aKeep2D = new(4, 5);
        Assert.Equal(1, BOPTools_AlgoTools3D.PointNearEdge(aForeign, aBoxFace, 0.5, 0.01, ref aKeep2D, ref aKeep));
        Assert.Equal(new gp_Pnt(1, 2, 3), aKeep);
        Assert.Equal(new gp_Pnt2d(4, 5), aKeep2D);
    }

    // The outward normal of the primitive solids at a point of their surface.
    private static gp_Dir OutwardNormal(string theName, in gp_Pnt theP, in TopoDS_Face theF, double theU, double theV)
    {
        switch (theName)
        {
            case "sphere":
                return new gp_Dir(theP.XYZ());
            case "box":
                {
                    // the face's plane normal, outward
                    Geom_Surface aS = BRep_Tool.Surface(theF)!;
                    aS.D1(theU, theV, out _, out gp_Vec aDU, out gp_Vec aDV);
                    gp_Dir aN = new(aDU.Crossed(aDV));
                    return theF.Orientation() == TopAbs_REVERSED ? aN.Reversed() : aN;
                }
            default:
                throw new InvalidOperationException(theName);
        }
    }

    [Fact]
    public void GetNormalToFaceOnEdge_gives_the_outward_normal_of_box_and_sphere_faces()
    {
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives().Where(s => s.Name is "box" or "sphere"))
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    if (BRep_Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, out gp_Dir aN);
                    Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2)!;
                    aPC.D0(BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2), out gp_Pnt2d aUV);
                    gp_Pnt aP = BRep_Tool.Surface(aF)!.Value(aUV.X(), aUV.Y());
                    gp_Dir anExpected = OutwardNormal(aName, aP, aF, aUV.X(), aUV.Y());
                    Assert.True(aN.IsEqual(anExpected, 1.0e-9), $"{aName}: {aN} vs {anExpected}");
                    if (aName == "box")
                    {
                        // the box's outward normals point away from its centre (0, 0, 0)
                        Assert.True(aN.XYZ().Dot(aP.XYZ()) > 0);
                    }

                    // the parameter overload doesn't reverse for the face's orientation
                    BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2), out gp_Dir aN2,
                                                               new IntTools_Context());
                    gp_Dir aN2Oriented = aF.Orientation() == TopAbs_REVERSED ? aN2.Reversed() : aN2;
                    Assert.Equal(aN, aN2Oriented);
                }
            }
        }
    }

    [Fact]
    public void GetApproxNormalToFaceOnEdge_gives_a_point_inside_and_its_normal()
    {
        IntTools_Context aCtx = new();
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (TopoDS_Edge anE in BOPToolsTestShapes.Edges(aF))
            {
                BOPTools_AlgoTools3D.OrientEdgeOnFace(anE, aF, out TopoDS_Edge aE);
                BRep_Tool.Range(aE, out double aT1, out double aT2);
                double aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
                BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, out gp_Dir anExact);

                gp_Pnt aP = new();
                gp_Dir aN = new();
                Assert.True(BOPTools_AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, ref aP, ref aN, aCtx));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                Assert.True(BOPTools_AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, 1.0e-3, ref aP, ref aN, aCtx));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                Assert.True(BOPTools_AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, ref aP, ref aN, 1.0e-3));
                Assert.True(aN.IsEqual(anExact, 1.0e-12));
                gp_Pnt aPE = BRep_Tool.Curve(aE, out _, out _)!.Value(aT);
                Assert.Equal(1.0e-3, aPE.Distance(aP), 1.0e-12);
            }
        }
    }

    [Fact]
    public void SenseFlag_and_GetNormalToSurface_known_answers()
    {
        gp_Dir aZ = new(0, 0, 1);
        Assert.Equal(1, BOPTools_AlgoTools3D.SenseFlag(aZ, aZ));
        Assert.Equal(-1, BOPTools_AlgoTools3D.SenseFlag(aZ, aZ.Reversed()));
        Assert.Equal(0, BOPTools_AlgoTools3D.SenseFlag(aZ, new gp_Dir(1, 0, 0)));
        Assert.Equal(1, BOPTools_AlgoTools3D.SenseFlag(aZ, new gp_Dir(1.0e-5, 0, 1)));

        Geom_SphericalSurface aS = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        Geom_ConicalSurface aCone = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0.5, 0.0);
        gp_Dir aD = new(0, 1, 0);
        Assert.True(BOPTools_AlgoTools3D.GetNormalToSurface(aS, 0.3, 0.2, ref aD));
        gp_Pnt aP = aS.Value(0.3, 0.2);
        Assert.True(aD.IsEqual(new gp_Dir(aP.XYZ()), 1.0e-12));
        // at the apex of a cone, DU vanishes: false and the output unchanged
        gp_Dir aKeep = new(0, 1, 0);
        Assert.False(BOPTools_AlgoTools3D.GetNormalToSurface(aCone, 0.3, 0.0, ref aKeep));
        Assert.Equal(new gp_Dir(0, 1, 0), aKeep);
    }

    [Fact]
    public void OrientEdgeOnFace_gives_the_orientation_in_the_face()
    {
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    TopoDS_Edge aFwd = TopoDS.Edge(aE.Oriented(TopAbs_FORWARD));
                    BOPTools_AlgoTools3D.OrientEdgeOnFace(aFwd, aF, out TopoDS_Edge aR);
                    Assert.True(aR.IsSame(aE), aName);
                    if (BRep_Tool.IsClosed(aE, aF))
                    {
                        // a seam found twice keeps the given orientation
                        Assert.Equal(TopAbs_FORWARD, aR.Orientation());
                    }
                    else
                    {
                        Assert.Equal(aE.Orientation(), aR.Orientation());
                    }
                }
            }
        }
        // an edge not in the face keeps its orientation
        TopoDS_Face aBoxFace = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        TopoDS_Edge aForeign = TopoDS.Edge(BOPToolsTestShapes.Edges(BOPToolsTestShapes.Cone())[0].Oriented(TopAbs_REVERSED));
        BOPTools_AlgoTools3D.OrientEdgeOnFace(aForeign, aBoxFace, out TopoDS_Edge aRF);
        Assert.Equal(TopAbs_REVERSED, aRF.Orientation());
    }

    [Fact]
    public void MinStepIn2d_and_IsEmptyShape()
    {
        Assert.Equal(1.0e-5, BOPTools_AlgoTools3D.MinStepIn2d());
        Assert.False(BOPTools_AlgoTools3D.IsEmptyShape(BOPToolsTestShapes.Box()));
        BRep_Builder aBB = new();
        TopoDS_Compound aC = new();
        aBB.MakeCompound(ref aC);
        Assert.True(BOPTools_AlgoTools3D.IsEmptyShape(aC));
        TopoDS_Compound aC2 = new();
        aBB.MakeCompound(ref aC2);
        aBB.Add(aC2, aC);
        Assert.True(BOPTools_AlgoTools3D.IsEmptyShape(aC2));
        TopoDS_Compound aC3 = new();
        aBB.MakeCompound(ref aC3);
        aBB.Add(aC3, aC2);
        aBB.Add(aC3, BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0]);
        Assert.False(BOPTools_AlgoTools3D.IsEmptyShape(aC3));
        Assert.True(BOPTools_AlgoTools3D.IsEmptyShape(new TopoDS_Shape()));
    }

    // The cylinder's lateral face and its seam edge, unfrozen.
    private static (TopoDS_Face Face, TopoDS_Edge Seam) CylinderSeam()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(2, 5).InternalShape();
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aCyl))
        {
            if (BRep_Tool.Surface(aF) is not Geom_CylindricalSurface)
            {
                continue;
            }
            foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
            {
                if (BRep_Tool.IsClosed(aE, aF))
                {
                    return (aF, aE);
                }
            }
        }
        throw new InvalidOperationException();
    }

    // A new edge on part of the seam's line with only the seam's FORWARD pcurve (or its translation by the period).
    private static TopoDS_Edge SplitOfSeam(in TopoDS_Face theFace, in TopoDS_Edge theSeam, double theT1, double theT2,
                                           TopAbs_Orientation theWhich = TopAbs_FORWARD)
    {
        BRep_Builder aBB = new();
        Geom_Curve aC = BRep_Tool.Curve(theSeam, out _, out _)!;
        Geom2d_Curve aPC = (Geom2d_Curve)BRep_Tool.CurveOnSurface(TopoDS.Edge(theSeam.Oriented(theWhich)), theFace, out _, out _)!.Copy();
        TopoDS_Edge aE = new();
        aBB.MakeEdge(ref aE, aC, 1.0e-7);
        aBB.Range(aE, theT1, theT2);
        aBB.UpdateEdge(aE, aPC, theFace, 1.0e-7);
        return aE;
    }

    [Fact]
    public void DoSplitSEAMOnFace_makes_the_split_a_seam()
    {
        (TopoDS_Face aF, TopoDS_Edge aSeam) = CylinderSeam();
        foreach (TopAbs_Orientation aWhich in new[] { TopAbs_FORWARD, TopAbs_REVERSED })
        {
            TopoDS_Edge aSplit = SplitOfSeam(aF, aSeam, 1.0, 3.0, aWhich);
            Assert.False(BRep_Tool.IsClosed(aSplit, aF));
            Assert.True(BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSplit, aF));
            Assert.True(BRep_Tool.IsClosed(aSplit, aF));
            // the two pcurves are 2 pi apart and ordered as the seam's
            Geom2d_Curve aP1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSplit.Oriented(TopAbs_FORWARD)), aF, out _, out _)!;
            Geom2d_Curve aP2 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSplit.Oriented(TopAbs_REVERSED)), aF, out _, out _)!;
            Geom2d_Curve aS1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeam.Oriented(TopAbs_FORWARD)), aF, out _, out _)!;
            Geom2d_Curve aS2 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeam.Oriented(TopAbs_REVERSED)), aF, out _, out _)!;
            Assert.Equal(aS1.Value(2.0).X(), aP1.Value(2.0).X(), 1.0e-12);
            Assert.Equal(aS2.Value(2.0).X(), aP2.Value(2.0).X(), 1.0e-12);
        }

        // the two-edge form
        TopoDS_Edge aSplit2 = SplitOfSeam(aF, aSeam, 0.5, 4.0);
        Assert.True(BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSeam, aSplit2, aF));
        Assert.True(BRep_Tool.IsClosed(aSplit2, aF));
        Geom2d_Curve aQ1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSplit2.Oriented(TopAbs_FORWARD)), aF, out _, out _)!;
        Geom2d_Curve aT1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeam.Oriented(TopAbs_FORWARD)), aF, out _, out _)!;
        Assert.Equal(aT1.Value(2.0).X(), aQ1.Value(2.0).X(), 1.0e-12);
        // already a seam: true without change; origin not a seam: false
        Assert.True(BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSeam, aSplit2, aF));
        TopoDS_Edge aNotSeam = BOPToolsTestShapes.Edges(aF).First(e => !BRep_Tool.IsClosed(e, aF) && !BRep_Tool.Degenerated(e));
        Assert.False(BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aNotSeam, SplitOfSeam(aF, aSeam, 1.0, 2.0), aF));
    }

    [Fact]
    public void DoSplitSEAMOnFace_does_nothing_on_planes_and_throws_on_frozen_edges()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).InternalShape();
        TopoDS_Face aF = BOPToolsTestShapes.Faces(aBox)[0];
        Assert.False(BOPTools_AlgoTools3D.DoSplitSEAMOnFace(BOPToolsTestShapes.Edges(aF)[0], aF));

        // a frozen split is written into: it throws (plan 2.3, ported callers give it their own edges)
        TopoDS_Shape aCyl = BOPToolsTestShapes.Cylinder();
        TopoDS_Face aCF = BOPToolsTestShapes.Faces(aCyl).First(f => BRep_Tool.Surface(f) is Geom_CylindricalSurface);
        TopoDS_Edge aSeam = BOPToolsTestShapes.Edges(aCF).First(e => BRep_Tool.IsClosed(e, aCF));
        Assert.ThrowsAny<Standard_DomainError>(() => BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSeam, aCF));
    }
}
