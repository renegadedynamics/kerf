// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Random scenes for the unit 3G oracle tests (BRepClass3d, BRepIntCurveSurface, BRepBndLib), built identically on
/// both sides with BRep_Builder from the same raw numbers: boxes, closed cylinders whose lateral face is a cylinder,
/// a surface of linear extrusion or a surface of revolution (the last two take IntCurvesFace's polyhedron path),
/// faces with holes, natural spheres, tori and sphere patches, natural B-spline faces with a B-spline edge, compounds
/// of placed solids, and (for the boxes only) boxes carrying meshes and tetrahedra that are only a mesh. Placed at
/// random rigid (sometimes scaled) locations and reversed half the time. The port's shape is frozen.
/// </summary>
internal static class ClassificationScenes
{
    public const int Kinds = 10;

    /// <summary>The kinds of scene that have a surface on every face (all but the mesh-only tetrahedron).</summary>
    public static bool HasSurfaces(int theScene) => theScene % Kinds != 9;

    /// <summary>A description of the scene's kind.</summary>
    public static string Name(int theScene) => (theScene % Kinds) switch
    {
        0 => "box",
        1 => "cylinder",
        2 => "extruded cylinder",
        3 => "revolved cylinder",
        4 => "holed face",
        5 => "natural surface",
        6 => "B-spline face and edge",
        7 => "compound of solids",
        8 => "meshed box",
        _ => "mesh tetrahedron",
    };

    public static Twin<N.TopoDS_Shape, TopoDS_Shape> Scene(Oracle theOracle, int theScene)
    {
        var aTwins = new GPropTwins();
        Twin<N.TopoDS_Shape, TopoDS_Shape> aShape;
        switch (theScene % Kinds)
        {
            case 0:
                aShape = Placed(theOracle, TwinShapes.Up(aTwins.Twins.Box(theOracle.Real(0.5, 20), theOracle.Real(0.5, 20), theOracle.Real(0.5, 20))), true);
                break;
            case 1:
                aShape = Placed(theOracle, TwinShapes.Up(aTwins.CylinderSolid(theOracle.Real(0.5, 5), theOracle.Real(0.5, 5))), false);
                break;
            case 2:
            case 3:
                aShape = Placed(theOracle, TwinShapes.Up(SweptCylinder(aTwins, theOracle.Real(0.5, 5), theOracle.Real(0.5, 5), theScene % Kinds == 3)), false);
                break;
            case 4:
                aShape = Placed(theOracle, TwinShapes.Up(FaceWithHole(theOracle, aTwins.Twins)), false);
                break;
            case 5:
                aShape = Placed(theOracle, aTwins.NaturalSurfaceShape(theOracle), false);
                break;
            case 6:
                aShape = Placed(theOracle, aTwins.BSplineScene(theOracle), false);
                break;
            case 7:
            {
                Twin<N.TopoDS_Shape, TopoDS_Shape> aBox = TwinShapes.Up(aTwins.Twins.Box(theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5)));
                Twin<N.TopoDS_Shape, TopoDS_Shape> aCyl = TwinShapes.Up(aTwins.CylinderSolid(theOracle.Real(0.5, 2), theOracle.Real(0.5, 3)));
                (N.TopLoc_Location aNL1, Placement aL1) = RandomLocation(theOracle, false);
                (N.TopLoc_Location aNL2, Placement aL2) = RandomLocation(theOracle, false);
                Twin<N.TopoDS_Compound, TopoDS_Compound> aCompound = aTwins.Twins.Compound(
                    aBox,
                    new(aBox.N.Moved(aNL1), aBox.P.Moved(aL1)),
                    new(aCyl.N.Moved(aNL2), aCyl.P.Moved(aL2)));
                aShape = Placed(theOracle, TwinShapes.Up(aCompound), false);
                break;
            }

            case 8:
            {
                Twin<N.TopoDS_Solid, TopoDS_Solid> aBox = aTwins.Twins.Box(theOracle.Real(0.5, 10), theOracle.Real(0.5, 10), theOracle.Real(0.5, 10));
                aTwins.AttachQuadMeshes(TwinShapes.Up(aBox));
                aShape = Placed(theOracle, TwinShapes.Up(aBox), true);
                break;
            }

            default:
                aShape = Placed(theOracle, aTwins.MeshTetra(theOracle), true);
                break;
        }

        aShape.P.TShape()!.FreezeGraph();
        return aShape;
    }

    /// <summary>The shape at a random location (or none; scaled if allowed, sometimes), and reversed half the time.</summary>
    public static Twin<N.TopoDS_Shape, TopoDS_Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, bool theAllowScale)
    {
        Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle, theAllowScale && theOracle.Bool());
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    /// <summary>A location built from the same raw numbers on both sides: a rotation, a translation or both, and a scale if asked.</summary>
    public static (N.TopLoc_Location, Placement) RandomLocation(Oracle theOracle, bool theScale)
    {
        var aNT = new N.gp_Trsf();
        var aT = new Transform3D();
        if (theOracle.Bool())
        {
            (N.gp_Ax1 aNAx, Axis3D anAx) = GeomOracleInputs.Ax1s(theOracle);
            double anAngle = theOracle.Angle();
            using (aNAx)
            {
                aNT.SetRotation(aNAx, anAngle);
            }

            aT.SetRotation(anAx, anAngle);
        }

        if (theOracle.Bool())
        {
            Vector3D aV = new(theOracle.Real(-20, 20), theOracle.Real(-20, 20), theOracle.Real(-20, 20));
            using var aNV = Oracle.ToNative(aV);
            var aNT2 = new N.gp_Trsf();
            aNT2.SetTranslation(aNV);
            aNT = aNT2.Multiplied(aNT);
            aT = Transform3D.Translation(aV).Multiplied(aT);
        }

        if (theScale)
        {
            Point3D aP = new(theOracle.Real(-10, 10), theOracle.Real(-10, 10), theOracle.Real(-10, 10));
            double aS = theOracle.Real(0.3, 3);
            using var aNP = Oracle.ToNative(aP);
            var aNT2 = new N.gp_Trsf();
            aNT2.SetScale(aNP, aS);
            aNT = aNT2.Multiplied(aNT);
            aT = Transform3D.Scale(aP, aS).Multiplied(aT);
        }

        return (new N.TopLoc_Location(aNT), new Placement(aT));
    }

    /// <summary>A square face with a rectangular hole, on a random plane (the hole's wire first).</summary>
    public static Twin<N.TopoDS_Face, TopoDS_Face> FaceWithHole(Oracle theOracle, TwinShapes theTwins)
    {
        Point3D anOrigin = new(theOracle.Real(-10, 10), theOracle.Real(-10, 10), theOracle.Real(-10, 10));
        global::Kerf.Engine.Geometry.Xyz aNormal = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz aRef = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz anX = aNormal.Crossed(aRef);
        while (anX.Modulus() < 0.1)
        {
            aRef = GeomOracleInputs.RawDir(theOracle);
            anX = aNormal.Crossed(aRef);
        }

        var aPln = new Plane3D(new CoordinateSystem(anOrigin, new Direction3D(aNormal), new Direction3D(anX)));
        double aSize = theOracle.Real(4, 10);
        double hu1 = theOracle.Real(0.5, 1.5), hu2 = theOracle.Real(2, 3.5), hv1 = theOracle.Real(0.5, 1.5), hv2 = theOracle.Real(2, 3.5);
        (double U, double V)[] aUV = [(0, 0), (aSize, 0), (aSize, aSize), (0, aSize), (hu1, hv1), (hu1, hv2), (hu2, hv2), (hu2, hv1)];
        var aPoints = new Point3D[8];
        var aV = new Twin<N.TopoDS_Vertex, TopoDS_Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = global::Kerf.Engine.Geometry.ElSLib.Value(aUV[i].U, aUV[i].V, aPln);
            aV[i] = theTwins.Vertex(aPoints[i], 1e-6);
        }

        return theTwins.PolygonFace(aPoints, aV, new Dictionary<(int, int), Twin<N.TopoDS_Edge, TopoDS_Edge>>(),
                                    anOrigin, aNormal, anX, [4, 5, 6, 7], [0, 1, 2, 3]);
    }

    /// <summary>
    /// A closed cylinder around Z whose lateral face is a surface of linear extrusion of the bottom circle along Z, or
    /// a surface of revolution of the seam line around Z: the same parametrization as the cylindrical surface, so the
    /// pcurves are the cylinder's. Two discs share its circles.
    /// </summary>
    public static Twin<N.TopoDS_Solid, TopoDS_Solid> SweptCylinder(GPropTwins theTwins, double r, double h, bool theRevolution)
    {
        var aNB = new N.BRep_Builder();
        var aPB = new BRep_Builder();
        const double Tol = 1e-7;
        double aTwoPi = 2.0 * Math.PI;
        TwinShapes aTw = theTwins.Twins;
        Twin<N.TopoDS_Vertex, TopoDS_Vertex> aV0 = aTw.Vertex(new Point3D(r, 0, 0)), aV1 = aTw.Vertex(new Point3D(r, 0, h));
        using var aNO = new N.gp_Pnt(0, 0, 0);
        using var aNOh = new N.gp_Pnt(0, 0, h);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx2 = new N.gp_Ax2(aNO, aNZ, aNX);
        using var aNAx2h = new N.gp_Ax2(aNOh, aNZ, aNX);
        var aNoLoc = new Placement();
        var aNBottomCircle = new N.Geom_Circle(aNAx2, r);
        var aBottomCircle = new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);
        Twin<N.TopoDS_Edge, TopoDS_Edge> aBottom = aTw.EdgeOn(aNBottomCircle, aBottomCircle, aNoLoc, null, 0, aTwoPi, aV0, aV0);
        Twin<N.TopoDS_Edge, TopoDS_Edge> aTop = aTw.EdgeOn(new N.Geom_Circle(aNAx2h, r),
            new Geom_Circle(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r), aNoLoc, null, 0, aTwoPi, aV1, aV1);
        using var aNSeamP = new N.gp_Pnt(r, 0, 0);
        var aNSeamLine = new N.Geom_Line(aNSeamP, aNZ);
        var aSeamLine = new Geom_Line(new Point3D(r, 0, 0), new Direction3D(0, 0, 1));
        Twin<N.TopoDS_Edge, TopoDS_Edge> aSeam = aTw.EdgeOn(aNSeamLine, aSeamLine, aNoLoc, null, 0, h, aV0, aV1);

        N.Geom_Surface aNS;
        Geom_Surface aS;
        if (theRevolution)
        {
            using var aNAx1 = new N.gp_Ax1(aNO, aNZ);
            aNS = new N.Geom_SurfaceOfRevolution(new N.Geom_Line(aNSeamP, aNZ), aNAx1);
            aS = new Geom_SurfaceOfRevolution(new Geom_Line(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        }
        else
        {
            aNS = new N.Geom_SurfaceOfLinearExtrusion(new N.Geom_Circle(aNAx2, r), aNZ);
            aS = new Geom_SurfaceOfLinearExtrusion(new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r), new Direction3D(0, 0, 1));
        }

        var aNF = new N.TopoDS_Face();
        aNB.MakeFace(aNF, aNS, Tol);
        TopoDS_Face aF = new();
        aPB.MakeFace(ref aF, aS, Tol);

        // the pcurves (u along the circle, v along Z) as on the cylinder
        void Update(Twin<N.TopoDS_Edge, TopoDS_Edge> theE, (double X, double Y, double DX, double DY) theL1,
                    (double X, double Y, double DX, double DY)? theL2)
        {
            N.Geom2d_Line NativeLine((double X, double Y, double DX, double DY) l)
            {
                using var aP = new N.gp_Pnt2d(l.X, l.Y);
                using var aD = new N.gp_Dir2d(l.DX, l.DY);
                return new N.Geom2d_Line(aP, aD);
            }

            static Geom2d_Line PortLine((double X, double Y, double DX, double DY) l) => new(new Point2D(l.X, l.Y), new Direction2D(l.DX, l.DY));

            if (theL2 is null)
            {
                aNB.UpdateEdge(theE.N, NativeLine(theL1), aNF, Tol);
                aPB.UpdateEdge(theE.P, PortLine(theL1), aF, Tol);
            }
            else
            {
                aNB.UpdateEdge(theE.N, NativeLine(theL1), NativeLine(theL2.Value), aNF, Tol);
                aPB.UpdateEdge(theE.P, PortLine(theL1), PortLine(theL2.Value), aF, Tol);
            }
        }

        Update(aBottom, (0, 0, 1, 0), null);
        Update(aTop, (0, h, 1, 0), null);
        Update(aSeam, (aTwoPi, 0, 0, 1), (0, 0, 0, 1));
        Twin<N.TopoDS_Wire, TopoDS_Wire> aW = aTw.Wire(true, aBottom,
                                                       aSeam,
                                                       new(N.TopoDS.Edge(aTop.N.Reversed()), TopoDS.Edge(aTop.P.Reversed())),
                                                       new(N.TopoDS.Edge(aSeam.N.Reversed()), TopoDS.Edge(aSeam.P.Reversed())));
        aNB.Add(aNF, aW.N);
        aPB.Add(aF, aW.P);

        Twin<N.TopoDS_Face, TopoDS_Face> aBottomFace = Disc(aNB, aPB, aBottom, 0, r), aTopFace = Disc(aNB, aPB, aTop, h, r);
        return theTwins.Solid(TwinShapes.Up(new Twin<N.TopoDS_Face, TopoDS_Face>(aNF, aF)),
                              new(aBottomFace.N.Reversed(), aBottomFace.P.Reversed()),
                              TwinShapes.Up(aTopFace));
    }

    private static Twin<N.TopoDS_Face, TopoDS_Face> Disc(N.BRep_Builder theNB, BRep_Builder thePB, Twin<N.TopoDS_Edge, TopoDS_Edge> theCircle, double z, double r)
    {
        const double Tol = 1e-7;
        using var aNO = new N.gp_Pnt(0, 0, z);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNX);
        var aNF = new N.TopoDS_Face();
        theNB.MakeFace(aNF, new N.Geom_Plane(aNAx3), Tol);
        TopoDS_Face aF = new();
        thePB.MakeFace(ref aF, new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, z), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), Tol);
        using (var aNC = new N.gp_Pnt2d(0, 0))
        using (var aND = new N.gp_Dir2d(1, 0))
        using (var aNAx = new N.gp_Ax2d(aNC, aND))
        {
            theNB.UpdateEdge(theCircle.N, new N.Geom2d_Circle(aNAx, r), aNF, Tol);
        }

        thePB.UpdateEdge(theCircle.P, new Geom2d_Circle(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), r), aF, Tol);
        var aNW = new N.TopoDS_Wire();
        theNB.MakeWire(aNW);
        TopoDS_Wire aW = new();
        thePB.MakeWire(ref aW);
        theNB.Add(aNW, theCircle.N);
        thePB.Add(aW, theCircle.P);
        aNW.Closed(true);
        aW.Closed(true);
        theNB.Add(aNF, aNW);
        thePB.Add(aF, aW);
        return new(aNF, aF);
    }

    /// <summary>The sub-shapes of the type, in exploration order, on both sides.</summary>
    public static List<Twin<N.TopoDS_Shape, TopoDS_Shape>> Explore(Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, TopAbs_ShapeEnum theType)
    {
        var aList = new List<Twin<N.TopoDS_Shape, TopoDS_Shape>>();
        var aNE = new N.TopExp_Explorer(theShape.N, TwinShapes.ToNative(theType));
        for (var aPE = new TopExp_Explorer(theShape.P, theType); aPE.More(); aPE.Next(), aNE.Next())
        {
            Assert.True(aNE.More());
            aList.Add(new(aNE.Current(), aPE.Current()));
        }

        return aList;
    }

    /// <summary>The index (from 1) of the face IsSame as <paramref name="theFace"/> among the native faces, 0 if none or null.</summary>
    public static int NativeIndex(List<Twin<N.TopoDS_Shape, TopoDS_Shape>> theFaces, N.TopoDS_Shape theFace)
    {
        if (theFace.IsNull())
        {
            return 0;
        }

        for (int i = 0; i < theFaces.Count; i++)
        {
            if (theFaces[i].N.IsSame(theFace))
            {
                return i + 1;
            }
        }

        return -1;
    }

    /// <summary>The index (from 1) of the face IsSame as <paramref name="theFace"/> among the port's faces, 0 if none or null.</summary>
    public static int PortIndex(List<Twin<N.TopoDS_Shape, TopoDS_Shape>> theFaces, in TopoDS_Shape theFace)
    {
        if (theFace.IsNull())
        {
            return 0;
        }

        for (int i = 0; i < theFaces.Count; i++)
        {
            if (theFaces[i].P.IsSame(theFace))
            {
                return i + 1;
            }
        }

        return -1;
    }

    /// <summary>
    /// Points to classify or aim at: random points in the box of the shape (enlarged by a fifth), points on its faces
    /// (from random UV in the faces' bounds) with small and large offsets along the normal, points on its edges and
    /// its vertices.
    /// </summary>
    public static List<Point3D> SamplePoints(Oracle theOracle, in TopoDS_Shape theShape, int theNbRandom, int thePerFace)
    {
        var aPoints = new List<Point3D>();
        Box aBox = BRepBndLib.Box(theShape);
        aBox.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
        double dx = (x2 - x1) * 0.2, dy = (y2 - y1) * 0.2, dz = (z2 - z1) * 0.2;
        for (int i = 0; i < theNbRandom; i++)
        {
            aPoints.Add(new Point3D(theOracle.Real(x1 - dx, x2 + dx), theOracle.Real(y1 - dy, y2 + dy), theOracle.Real(z1 - dz, z2 + dz)));
        }

        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExp.Current());
            if (BRep_Tool.Surface(aF) is null)
            {
                continue;
            }

            BRepTools.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
            var aS = new BRepAdaptor_Surface(aF, false);
            for (int i = 0; i < thePerFace; i++)
            {
                double u = theOracle.Real(u1, u2), v = theOracle.Real(v1, v2);
                aS.D1(u, v, out Point3D aP, out Vector3D aDU, out Vector3D aDV);
                Vector3D aN = aDU.Crossed(aDV);
                double anOffset = theOracle.Int(0, 4) switch
                {
                    0 => 0.0,
                    1 => 1e-9,
                    2 => -1e-6,
                    3 => 1e-2,
                    _ => -0.3,
                };
                if (anOffset != 0.0 && aN.Magnitude() > 1e-9)
                {
                    aP = aP.Translated(aN.Normalized().Multiplied(anOffset));
                }

                aPoints.Add(aP);
            }
        }

        for (var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            if (!BRep_Tool.IsGeometric(anE) || theOracle.Int(0, 2) != 0)
            {
                continue;
            }

            var aC = new BRepAdaptor_Curve(anE);
            aPoints.Add(aC.Value(theOracle.Real(aC.FirstParameter(), aC.LastParameter())));
        }

        for (var anExp = new TopExp_Explorer(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            if (theOracle.Int(0, 3) == 0)
            {
                aPoints.Add(BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
            }
        }

        return aPoints;
    }
}
