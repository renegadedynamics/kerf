// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepGProp against native OCCT on random shapes built identically on both sides with BRep_Builder: boxes, closed
/// cylinders, faces with holes, compounds sharing solids at rigid and scaled locations, natural spheres, tori and
/// sphere patches, B-spline faces and edges, boxes carrying triangulations and faces that are only a mesh, placed and
/// reversed at random. Every form of LinearProperties, SurfaceProperties, VolumeProperties and VolumePropertiesGK
/// (with the flags varied) is compared: mass, centre of mass, matrix of inertia and the error returned. BRepGProp_Face
/// is compared method by method on the faces and their edges. The port's shapes are frozen first.
/// </summary>
public class BRepGPropOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 48;

    private sealed class Counts
    {
        public int Calls;
        public int Faces;
        public int Edges;
    }

    [Fact]
    public void Global_properties_match_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261029);
        var aCounts = new Counts();
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            (Twin<N.TopoDS_Shape, Shape> aShape, bool aMeshed) = RandomScene(anOracle, aScene);
            aShape.P.TShape()!.FreezeGraph();
            CompareProperties(anOracle, aShape, aMeshed, aScene % 8 is not (2 or 3), $"scene {aScene}", aCounts);
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Calls} BRepGProp calls");
        anOracle.Report("BRepGProp (Linear, Surface, Volume, VolumeGK; Ex and mesh forms)");
    }

    [Fact]
    public void Face_tool_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261030);
        var aCounts = new Counts();
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            (Twin<N.TopoDS_Shape, Shape> aShape, bool aMeshed) = RandomScene(anOracle, aScene);
            if (aMeshed && aScene % 8 == 7)
            {
                continue; // faces without a surface
            }

            aShape.P.TShape()!.FreezeGraph();
            foreach (Twin<N.TopoDS_Shape, Shape> aF in Explore(aShape, ShapeType.Face))
            {
                CompareFace(anOracle, new(N.TopoDS.Face(aF.N), ShapeCasts.Face(aF.P)), $"scene {aScene}", aCounts);
            }
        }

        output.WriteLine($"{aCounts.Faces} faces, {aCounts.Edges} edges");
        anOracle.Report("BRepGProp_Face");
    }

    // Scenes

    internal static (Twin<N.TopoDS_Shape, Shape>, bool) RandomScene(Oracle theOracle, int theScene)
    {
        var aTwins = new GPropTwins();
        switch (theScene % 8)
        {
            case 0:
                return (Placed(theOracle, TwinShapes.Up(aTwins.Twins.Box(theOracle.Real(0.5, 20), theOracle.Real(0.5, 20), theOracle.Real(0.5, 20))), true), false);
            case 1:
                return (Placed(theOracle, TwinShapes.Up(aTwins.CylinderSolid(theOracle.Real(0.5, 5), theOracle.Real(0.5, 5))), false), false);
            case 2:
                return (Placed(theOracle, TwinShapes.Up(FaceWithHole(theOracle, aTwins.Twins)), false), false);
            case 3:
            {
                // a compound sharing a solid at several locations (rigid, reversed, sometimes scaled), with free faces
                Twin<N.TopoDS_Shape, Shape> aBox = TwinShapes.Up(aTwins.Twins.Box(theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5)));
                Twin<N.TopoDS_Shape, Shape> aCyl = TwinShapes.Up(aTwins.CylinderSolid(theOracle.Real(0.5, 2), theOracle.Real(0.5, 3)));
                (N.TopLoc_Location aNL1, Placement aL1) = RandomLocation(theOracle, false);
                (N.TopLoc_Location aNL2, Placement aL2) = RandomLocation(theOracle, theOracle.Bool());
                (N.TopLoc_Location aNL3, Placement aL3) = RandomLocation(theOracle, false);
                Twin<N.TopoDS_Compound, Compound> aCompound = aTwins.Twins.Compound(
                    aBox,
                    new(aBox.N.Moved(aNL1), aBox.P.Moved(aL1)),
                    new(aBox.N.Moved(aNL2).Reversed(), aBox.P.Moved(aL2).Reversed()),
                    aBox,
                    aCyl,
                    new(aCyl.N.Moved(aNL3), aCyl.P.Moved(aL3)),
                    TwinShapes.Up(FaceWithHole(theOracle, aTwins.Twins)));
                return (Placed(theOracle, TwinShapes.Up(aCompound), false), false);
            }

            case 4:
                return (Placed(theOracle, aTwins.NaturalSurfaceShape(theOracle), false), false);
            case 5:
                return (Placed(theOracle, aTwins.BSplineScene(theOracle), false), false);
            case 6:
            {
                Twin<N.TopoDS_Solid, Solid> aBox = aTwins.Twins.Box(theOracle.Real(0.5, 10), theOracle.Real(0.5, 10), theOracle.Real(0.5, 10));
                aTwins.AttachQuadMeshes(TwinShapes.Up(aBox));
                return (Placed(theOracle, TwinShapes.Up(aBox), true), true);
            }

            default:
                return (Placed(theOracle, aTwins.MeshTetra(theOracle), true), true);
        }
    }

    /// <summary>The shape at a random location (or none; scaled if allowed, sometimes), and reversed half the time.</summary>
    private static Twin<N.TopoDS_Shape, Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, bool theAllowScale)
    {
        Twin<N.TopoDS_Shape, Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle, theAllowScale && theOracle.Bool());
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    /// <summary>A location built from the same raw numbers on both sides: a rotation, a translation or both, and a scale if asked.</summary>
    private static (N.TopLoc_Location, Placement) RandomLocation(Oracle theOracle, bool theScale)
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
            Vector3D aV = theOracle.Vec();
            using var aNV = Oracle.ToNative(aV);
            var aNT2 = new N.gp_Trsf();
            aNT2.SetTranslation(aNV);
            aNT = aNT2.Multiplied(aNT);
            aT = Transform3D.Translation(aV).Multiplied(aT);
        }

        if (theScale)
        {
            Point3D aP = theOracle.Pnt();
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
    private static Twin<N.TopoDS_Face, Face> FaceWithHole(Oracle theOracle, TwinShapes theTwins)
    {
        Point3D anOrigin = theOracle.Pnt();
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
        var aV = new Twin<N.TopoDS_Vertex, Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = global::Kerf.Engine.Geometry.ElSLib.Value(aUV[i].U, aUV[i].V, aPln);
            aV[i] = theTwins.Vertex(aPoints[i], 1e-6);
        }

        return theTwins.PolygonFace(aPoints, aV, new Dictionary<(int, int), Twin<N.TopoDS_Edge, Edge>>(),
                                    anOrigin, aNormal, anX, [4, 5, 6, 7], [0, 1, 2, 3]);
    }

    // Comparisons

    private static void SameProps(Oracle theOracle, N.GProp_GProps theN, Properties theP, string theWhat)
    {
        theOracle.Same(theN.Mass(), theP.Mass(), theWhat + " mass");
        using (N.gp_Pnt aNC = theN.CentreOfMass())
        {
            theOracle.Same(aNC, theP.CentreOfMass(), theWhat + " centre");
        }

        Matrix3x3 aN;
        using (N.gp_Mat aNM = theN.MatrixOfInertia())
        {
            aN = Oracle.FromNative(aNM);
        }

        Matrix3x3 aP = theP.MatrixOfInertia();
        // each entry relative to the largest entry of the native matrix (at least 1)
        double aScale = 1.0;
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                aScale = Math.Max(aScale, Math.Abs(aN.Value(r, c)));
            }
        }

        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                double anEntry = Math.Max(1.0, Math.Abs(aN.Value(r, c)));
                theOracle.Same(aN.Value(r, c), aP.Value(r, c), $"{theWhat} inertia({r},{c})", Oracle.Tolerance * aScale / anEntry);
            }
        }
    }

    private static void CompareProperties(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, bool theMeshed,
                                          bool theGK, string theScene, Counts theCounts)
    {
        N.TopoDS_Shape aNS = theShape.N;
        Shape aPS = theShape.P;
        bool[] aBools = [false, true];

        foreach (bool aSkip in aBools)
        {
            foreach (bool aTri in aBools)
            {
                string aWhat = $"{theScene} skip {aSkip} tri {aTri}";
                {
                    using var aN = new N.GProp_GProps();
                    var aP = new Properties();
                    N.BRepGProp.LinearProperties(aNS, aN, aSkip, aTri);
                    MassPropertiesBuilder.LinearProperties(aPS, aP, aSkip, aTri);
                    SameProps(theOracle, aN, aP, aWhat + " linear");
                }
                {
                    using var aN = new N.GProp_GProps();
                    var aP = new Properties();
                    N.BRepGProp.SurfaceProperties(aNS, aN, aSkip, aTri);
                    MassPropertiesBuilder.SurfaceProperties(aPS, aP, aSkip, aTri);
                    SameProps(theOracle, aN, aP, aWhat + " surface");
                }
                foreach (bool aClosed in aBools)
                {
                    using var aN = new N.GProp_GProps();
                    var aP = new Properties();
                    N.BRepGProp.VolumeProperties(aNS, aN, aClosed, aSkip, aTri);
                    MassPropertiesBuilder.VolumeProperties(aPS, aP, aClosed, aSkip, aTri);
                    SameProps(theOracle, aN, aP, aWhat + $" volume closed {aClosed}");
                }

                theCounts.Calls += 4;
            }

            if (theMeshed)
            {
                continue; // faces with no surface, or meshes only on the mesh forms
            }

            foreach (double anEps in new[] { 1e-5, 0.01 })
            {
                string aWhat = $"{theScene} skip {aSkip} eps {anEps}";
                {
                    using var aN = new N.GProp_GProps();
                    var aP = new Properties();
                    double aNE = N.BRepGProp.SurfaceProperties(aNS, aN, anEps, aSkip);
                    double aPE = MassPropertiesBuilder.SurfaceProperties(aPS, aP, anEps, aSkip);
                    theOracle.Same(aNE, aPE, aWhat + " surface error");
                    SameProps(theOracle, aN, aP, aWhat + " surface");
                }
                foreach (bool aClosed in aBools)
                {
                    using var aN = new N.GProp_GProps();
                    var aP = new Properties();
                    double aNE = N.BRepGProp.VolumeProperties(aNS, aN, anEps, aClosed, aSkip);
                    double aPE = MassPropertiesBuilder.VolumeProperties(aPS, aP, anEps, aClosed, aSkip);
                    theOracle.Same(aNE, aPE, aWhat + $" volume error closed {aClosed}");
                    SameProps(theOracle, aN, aP, aWhat + $" volume closed {aClosed}");
                }

                theCounts.Calls += 3;
            }

            // Gauss-Kronrod, by point and by a plane, with random flags; not on faces with holes, where native OCCT
            // takes minutes even at 1e-3
            for (int k = 0; k < (theGK ? 2 : 0); k++)
            {
                double anEps = theOracle.Bool() ? 1e-3 : 1e-5;
                bool aClosed = theOracle.Bool(), aSpan = theOracle.Bool(), aCG = theOracle.Bool(), anI = theOracle.Bool();
                string aWhat = $"{theScene} GK eps {anEps} closed {aClosed} span {aSpan} CG {aCG} I {anI} skip {aSkip}";
                using var aN = new N.GProp_GProps();
                var aP = new Properties();
                double aNE, aPE;
                if (k == 0)
                {
                    aNE = N.BRepGProp.VolumePropertiesGK(aNS, aN, anEps, aClosed, aSpan, aCG, anI, aSkip);
                    aPE = MassPropertiesBuilder.VolumePropertiesGK(aPS, aP, anEps, aClosed, aSpan, aCG, anI, aSkip);
                }
                else
                {
                    Point3D aPlnP = theOracle.Pnt();
                    global::Kerf.Engine.Geometry.Xyz aPlnD = GeomOracleInputs.RawDir(theOracle);
                    using var aNPlnP = Oracle.ToNative(aPlnP);
                    using var aNPlnD = new N.gp_Dir(aPlnD.X(), aPlnD.Y(), aPlnD.Z());
                    using var aNPln = new N.gp_Pln(aNPlnP, aNPlnD);
                    var aPln = new Plane3D(aPlnP, new Direction3D(aPlnD.X(), aPlnD.Y(), aPlnD.Z()));
                    aNE = N.BRepGProp.VolumePropertiesGK(aNS, aN, aNPln, anEps, aClosed, aSpan, aCG, anI, aSkip);
                    aPE = MassPropertiesBuilder.VolumePropertiesGK(aPS, aP, aPln, anEps, aClosed, aSpan, aCG, anI, aSkip);
                    aWhat += " plane";
                }

                theOracle.Same(aNE, aPE, aWhat + " error");
                SameProps(theOracle, aN, aP, aWhat);
                theCounts.Calls++;
            }
        }
    }

    private static void CompareFace(Oracle theOracle, Twin<N.TopoDS_Face, Face> theFace, string theScene, Counts theCounts)
    {
        bool aSpan = theOracle.Bool();
        using var aNF = new N.BRepGProp_Face(theFace.N, aSpan);
        var aPF = new FaceIntegrand(theFace.P, aSpan);
        theCounts.Faces++;
        string aWhat = $"{theScene} face {theCounts.Faces}";

        theOracle.Same(aNF.NaturalRestriction(), aPF.NaturalRestriction(), aWhat + " NaturalRestriction");
        double nu1 = 0, nu2 = 0, nv1 = 0, nv2 = 0;
        aNF.Bounds(ref nu1, ref nu2, ref nv1, ref nv2);
        aPF.Bounds(out double u1, out double u2, out double v1, out double v2);
        theOracle.Same(nu1, u1, aWhat + " U1");
        theOracle.Same(nu2, u2, aWhat + " U2");
        theOracle.Same(nv1, v1, aWhat + " V1");
        theOracle.Same(nv2, v2, aWhat + " V2");
        theOracle.Same(aNF.UIntegrationOrder(), aPF.UIntegrationOrder(), aWhat + " UIntegrationOrder");
        theOracle.Same(aNF.VIntegrationOrder(), aPF.VIntegrationOrder(), aWhat + " VIntegrationOrder");
        foreach (double anEps in new[] { 1.0, 0.05, 1e-4, 1e-7 })
        {
            theOracle.Same(aNF.SIntOrder(anEps), aPF.SIntOrder(anEps), aWhat + $" SIntOrder({anEps})");
        }

        int aNbU = aPF.SUIntSubs(), aNbV = aPF.SVIntSubs();
        theOracle.Same(aNF.SUIntSubs(), aNbU, aWhat + " SUIntSubs");
        theOracle.Same(aNF.SVIntSubs(), aNbV, aWhat + " SVIntSubs");
        CompareKnots(theOracle, aNbU + 1, a => aNF.UKnots(a), a => aPF.UKnots(a), aWhat + " UKnots");
        CompareKnots(theOracle, aNbV + 1, a => aNF.VKnots(a), a => aPF.VKnots(a), aWhat + " VKnots");

        for (int i = 0; i < 4; i++)
        {
            double u = theOracle.Real(u1, u2), v = theOracle.Real(v1, v2);
            using var aNP = new N.gp_Pnt();
            using var aNV = new N.gp_Vec();
            aNF.Normal(u, v, aNP, aNV);
            aPF.Normal(u, v, out Point3D aP, out Vector3D aV);
            theOracle.Same(aNP, aP, aWhat + " Normal P");
            theOracle.Same(aNV, aV, aWhat + " Normal V");
        }

        double aMid = 0.5 * (u1 + u2);
        CompareHArray(theOracle, aNF.GetUKnots(u1, aMid), aPF.GetUKnots(u1, aMid), aWhat + " GetUKnots");

        // the edges, then the sides of the UV rectangle
        var aNE = new N.TopExp_Explorer(theFace.N.Oriented(N.TopAbs_Orientation.TopAbs_FORWARD), N.TopAbs_ShapeEnum.TopAbs_EDGE);
        var aPE = new Explorer(theFace.P.Oriented(Forward), ShapeType.Edge);
        for (; aPE.More(); aPE.Next(), aNE.Next())
        {
            theCounts.Edges++;
            bool aLoaded = aPF.Load(ShapeCasts.Edge(aPE.Current()));
            theOracle.Same(aNF.Load(N.TopoDS.Edge(aNE.Current())), aLoaded, aWhat + " Load(E)");
            if (aLoaded)
            {
                CompareArc(theOracle, aNF, aPF, aWhat + $" edge {theCounts.Edges}");
            }
        }

        foreach (bool aFirst in new[] { true, false })
        {
            foreach (global::Kerf.Engine.Geometry.IsoType anIso in new[] { IsoU, IsoV })
            {
                aNF.Load(aFirst, (N.GeomAbs_IsoType)(int)anIso);
                aPF.Load(aFirst, anIso);
                CompareArc(theOracle, aNF, aPF, aWhat + $" iso {anIso} {aFirst}");
            }
        }
    }

    private static void CompareArc(Oracle theOracle, N.BRepGProp_Face theNF, FaceIntegrand thePF, string theWhat)
    {
        double f = thePF.FirstParameter(), l = thePF.LastParameter();
        theOracle.Same(theNF.FirstParameter(), f, theWhat + " First");
        theOracle.Same(theNF.LastParameter(), l, theWhat + " Last");
        theOracle.Same(theNF.IntegrationOrder(), thePF.IntegrationOrder(), theWhat + " IntegrationOrder");
        foreach (double anEps in new[] { 1.0, 1e-3, 1e-6 })
        {
            theOracle.Same(theNF.LIntOrder(anEps), thePF.LIntOrder(anEps), theWhat + $" LIntOrder({anEps})");
        }

        int aNbL = thePF.LIntSubs();
        theOracle.Same(theNF.LIntSubs(), aNbL, theWhat + " LIntSubs");
        CompareKnots(theOracle, aNbL + 1, a => theNF.LKnots(a), a => thePF.LKnots(a), theWhat + " LKnots");
        double t = f + 0.37 * (l - f);
        using (var aNP = new N.gp_Pnt2d())
        using (var aNV = new N.gp_Vec2d())
        {
            theNF.D12d(t, aNP, aNV);
            thePF.D12d(t, out Point2D aP, out Vector2D aV);
            theOracle.Same(aNP, aP, theWhat + " D12d P");
            theOracle.Same(aNV, aV, theWhat + " D12d V");
        }

        using (N.gp_Pnt2d aNP = theNF.Value2d(t))
        {
            theOracle.Same(aNP, thePF.Value2d(t), theWhat + " Value2d");
        }

        CompareHArray(theOracle, theNF.GetTKnots(f, l), thePF.GetTKnots(f, l), theWhat + " GetTKnots");
    }

    private static void CompareKnots(Oracle theOracle, int theNb, Action<N.TColStd_Array1OfReal> theN,
                                     Action<NCollection_Array1<double>> theP, string theWhat)
    {
        using var aN = new N.TColStd_Array1OfReal(1, theNb);
        var aP = new NCollection_Array1<double>(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aN.SetValue(i, 0.0);
        }

        theN(aN);
        theP(aP);
        for (int i = 1; i <= theNb; i++)
        {
            theOracle.Same(aN.Value(i), aP[i], $"{theWhat}({i})");
        }
    }

    private static void CompareHArray(Oracle theOracle, N.TColStd_HArray1OfReal? theN, NCollection_HArray1<double> theP, string theWhat)
    {
        Assert.NotNull(theN);
        theOracle.Same(theN!.Lower(), theP.Lower(), theWhat + " Lower");
        theOracle.Same(theN.Upper(), theP.Upper(), theWhat + " Upper");
        for (int i = theP.Lower(); i <= theP.Upper(); i++)
        {
            theOracle.Same(theN.Value(i), theP[i], $"{theWhat}({i})");
        }
    }

    private static List<Twin<N.TopoDS_Shape, Shape>> Explore(Twin<N.TopoDS_Shape, Shape> theShape, ShapeType theType)
    {
        var aList = new List<Twin<N.TopoDS_Shape, Shape>>();
        var aNE = new N.TopExp_Explorer(theShape.N, TwinShapes.ToNative(theType));
        for (var aPE = new Explorer(theShape.P, theType); aPE.More(); aPE.Next(), aNE.Next())
        {
            Assert.True(aNE.More());
            aList.Add(new(aNE.Current(), aPE.Current()));
        }

        return aList;
    }
}

/// <summary>
/// More twin shapes for the BRepGProp oracle: closed cylinders, natural faces of spheres, tori and trimmed spheres,
/// B-spline faces and edges, quad meshes on box faces and faces that are only a mesh. Built from the same raw numbers on
/// both sides, as <see cref="TwinShapes"/>.
/// </summary>
internal sealed class GPropTwins
{
    private readonly N.BRep_Builder myN = new();
    private readonly BrepBuilder myP = new();
    private const double Tol = 1e-7;

    public TwinShapes Twins { get; } = new();

    private static Point3D Pnt(Oracle theOracle, double theMin, double theMax)
        => new(theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax));

    /// <summary>A closed cylinder around Z: the lateral face and two discs sharing its circles, in a solid.</summary>
    public Twin<N.TopoDS_Solid, Solid> CylinderSolid(double r, double h)
    {
        double aTwoPi = 2.0 * Math.PI;
        Twin<N.TopoDS_Vertex, Vertex> aV0 = Twins.Vertex(new Point3D(r, 0, 0)), aV1 = Twins.Vertex(new Point3D(r, 0, h));
        using var aNO = new N.gp_Pnt(0, 0, 0);
        using var aNOh = new N.gp_Pnt(0, 0, h);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx2 = new N.gp_Ax2(aNO, aNZ, aNX);
        using var aNAx2h = new N.gp_Ax2(aNOh, aNZ, aNX);
        var aNoLoc = new Placement();
        Twin<N.TopoDS_Edge, Edge> aBottom = Twins.EdgeOn(new N.Geom_Circle(aNAx2, r),
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r), aNoLoc, null, 0, aTwoPi, aV0, aV0);
        Twin<N.TopoDS_Edge, Edge> aTop = Twins.EdgeOn(new N.Geom_Circle(aNAx2h, r),
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r), aNoLoc, null, 0, aTwoPi, aV1, aV1);
        using var aNSeamP = new N.gp_Pnt(r, 0, 0);
        Twin<N.TopoDS_Edge, Edge> aSeam = Twins.EdgeOn(new N.Geom_Line(aNSeamP, aNZ),
            new LineCurve3D(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), aNoLoc, null, 0, h, aV0, aV1);

        using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNX);
        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, new N.Geom_CylindricalSurface(aNAx3, r), Tol);
        Face aF = new();
        myP.MakeFace(ref aF, new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r), Tol);
        UpdateEdge(aBottom, (0, 0, 1, 0), null, aNF, aF);
        UpdateEdge(aTop, (0, h, 1, 0), null, aNF, aF);
        UpdateEdge(aSeam, (aTwoPi, 0, 0, 1), (0, 0, 0, 1), aNF, aF);
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        Wire aW = new();
        myP.MakeWire(ref aW);
        AddToWire(aNW, aW, aBottom.N, aBottom.P);
        AddToWire(aNW, aW, aSeam.N, aSeam.P);
        AddToWire(aNW, aW, aTop.N.Reversed(), aTop.P.Reversed());
        AddToWire(aNW, aW, aSeam.N.Reversed(), aSeam.P.Reversed());
        myN.Add(aNF, aNW);
        myP.Add(aF, aW);

        Twin<N.TopoDS_Face, Face> aBottomFace = Disc(aBottom, 0, r), aTopFace = Disc(aTop, h, r);
        return Solid(new(aNF, aF), new(aBottomFace.N.Reversed(), aBottomFace.P.Reversed()), TwinShapes.Up(aTopFace));
    }

    private Twin<N.TopoDS_Face, Face> Disc(Twin<N.TopoDS_Edge, Edge> theCircle, double z, double r)
    {
        using var aNO = new N.gp_Pnt(0, 0, z);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNX);
        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, new N.Geom_Plane(aNAx3), Tol);
        Face aF = new();
        myP.MakeFace(ref aF, new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, z), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), Tol);
        using (var aNC = new N.gp_Pnt2d(0, 0))
        using (var aND = new N.gp_Dir2d(1, 0))
        using (var aNAx = new N.gp_Ax2d(aNC, aND))
        {
            myN.UpdateEdge(theCircle.N, new N.Geom2d_Circle(aNAx, r), aNF, Tol);
        }

        myP.UpdateEdge(theCircle.P, new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), r), aF, Tol);
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        Wire aW = new();
        myP.MakeWire(ref aW);
        AddToWire(aNW, aW, theCircle.N, theCircle.P);
        myN.Add(aNF, aNW);
        myP.Add(aF, aW);
        return new(aNF, aF);
    }

    private void UpdateEdge(Twin<N.TopoDS_Edge, Edge> theE, (double X, double Y, double DX, double DY) theL1,
                            (double X, double Y, double DX, double DY)? theL2, N.TopoDS_Face theNF, Face theF)
    {
        N.Geom2d_Line NativeLine((double X, double Y, double DX, double DY) l)
        {
            using var aP = new N.gp_Pnt2d(l.X, l.Y);
            using var aD = new N.gp_Dir2d(l.DX, l.DY);
            return new N.Geom2d_Line(aP, aD);
        }

        static LineCurve2D PortLine((double X, double Y, double DX, double DY) l) => new(new Point2D(l.X, l.Y), new Direction2D(l.DX, l.DY));

        if (theL2 is null)
        {
            myN.UpdateEdge(theE.N, NativeLine(theL1), theNF, Tol);
            myP.UpdateEdge(theE.P, PortLine(theL1), theF, Tol);
        }
        else
        {
            myN.UpdateEdge(theE.N, NativeLine(theL1), NativeLine(theL2.Value), theNF, Tol);
            myP.UpdateEdge(theE.P, PortLine(theL1), PortLine(theL2.Value), theF, Tol);
        }
    }

    private void AddToWire(N.TopoDS_Wire theNW, Wire theW, N.TopoDS_Shape theNE, Shape theE)
    {
        myN.Add(theNW, theNE);
        myP.Add(theW, theE);
        theNW.Closed(true);
        theW.Closed(true);
    }

    /// <summary>A closed shell of the faces in a solid.</summary>
    public Twin<N.TopoDS_Solid, Solid> Solid(params Twin<N.TopoDS_Shape, Shape>[] theFaces)
    {
        var aNSh = new N.TopoDS_Shell();
        myN.MakeShell(aNSh);
        Shell aSh = new();
        myP.MakeShell(ref aSh);
        foreach (Twin<N.TopoDS_Shape, Shape> aF in theFaces)
        {
            myN.Add(aNSh, aF.N);
            myP.Add(aSh, aF.P);
        }

        aNSh.Closed(true);
        aSh.Closed(true);
        var aNSo = new N.TopoDS_Solid();
        myN.MakeSolid(aNSo);
        myN.Add(aNSo, aNSh);
        Solid aSo = new();
        myP.MakeSolid(ref aSo);
        myP.Add(aSo, aSh);
        return new(aNSo, aSo);
    }

    /// <summary>A face on the surface with no wire, its natural restriction flag set.</summary>
    public Twin<N.TopoDS_Face, Face> NaturalFace(N.Geom_Surface theNS, Surface theS)
    {
        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, theNS, Tol);
        myN.NaturalRestriction(aNF, true);
        Face aF = new();
        myP.MakeFace(ref aF, theS, Tol);
        myP.NaturalRestriction(aF, true);
        return new(aNF, aF);
    }

    /// <summary>A coordinate system from raw coordinates on both sides.</summary>
    private static (N.gp_Ax3, CoordinateSystem) Ax3(Oracle theOracle)
    {
        Point3D aP = Pnt(theOracle, -5, 5);
        global::Kerf.Engine.Geometry.Xyz aN = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz aRef = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz anX = aN.Crossed(aRef);
        while (anX.Modulus() < 0.1)
        {
            aRef = GeomOracleInputs.RawDir(theOracle);
            anX = aN.Crossed(aRef);
        }

        using var aNP = Oracle.ToNative(aP);
        using var aNN = new N.gp_Dir(aN.X(), aN.Y(), aN.Z());
        using var aNX = new N.gp_Dir(anX.X(), anX.Y(), anX.Z());
        return (new N.gp_Ax3(aNP, aNN, aNX), new CoordinateSystem(aP, new Direction3D(aN.X(), aN.Y(), aN.Z()), new Direction3D(anX.X(), anX.Y(), anX.Z())));
    }

    /// <summary>A natural sphere or torus in a closed shell, or a natural patch of a trimmed sphere.</summary>
    public Twin<N.TopoDS_Shape, Shape> NaturalSurfaceShape(Oracle theOracle)
    {
        (N.gp_Ax3 aNAx, CoordinateSystem anAx) = Ax3(theOracle);
        using (aNAx)
        {
            switch (theOracle.Int(0, 2))
            {
                case 0:
                {
                    double r = theOracle.Real(0.5, 5);
                    return TwinShapes.Up(Solid(TwinShapes.Up(NaturalFace(new N.Geom_SphericalSurface(aNAx, r), new SphericalSurface(anAx, r)))));
                }

                case 1:
                {
                    double r1 = theOracle.Real(2, 5), r2 = theOracle.Real(0.3, 1.5);
                    return TwinShapes.Up(Solid(TwinShapes.Up(NaturalFace(new N.Geom_ToroidalSurface(aNAx, r1, r2), new ToroidalSurface(anAx, r1, r2)))));
                }

                default:
                {
                    double r = theOracle.Real(0.5, 5);
                    double u1 = theOracle.Real(0, 3), u2 = u1 + theOracle.Real(0.5, 3);
                    double v1 = theOracle.Real(-1.5, 0), v2 = v1 + theOracle.Real(0.3, 1.5);
                    var aNS = new N.Geom_RectangularTrimmedSurface(new N.Geom_SphericalSurface(aNAx, r), u1, u2, v1, v2);
                    var aS = new TrimmedSurface(new SphericalSurface(anAx, r), u1, u2, v1, v2);
                    return TwinShapes.Up(NaturalFace(aNS, aS));
                }
            }
        }
    }

    /// <summary>A compound of a natural B-spline face and a B-spline edge.</summary>
    public Twin<N.TopoDS_Shape, Shape> BSplineScene(Oracle theOracle)
    {
        int aUDeg = theOracle.Int(1, 3), aVDeg = theOracle.Int(1, 3);
        theOracle.BSplineKnots(aUDeg, false, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        theOracle.BSplineKnots(aVDeg, false, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        int aNbU = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aUDeg, false, aUMults), aNbV = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aVDeg, false, aVMults);
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[i, j] = new Point3D(i + theOracle.Real(-0.3, 0.3), j + theOracle.Real(-0.3, 0.3), theOracle.Real(-1, 1));
            }
        }

        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(aUKnots);
        using N.TColStd_Array1OfReal aNVKnots = Oracle.ToNative(aVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(aUMults);
        using N.TColStd_Array1OfInteger aNVMults = Oracle.ToNative(aVMults);
        Twin<N.TopoDS_Face, Face> aFace = NaturalFace(
            new N.Geom_BSplineSurface(aNPoles, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, false, false),
            new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, false, false));

        int aDeg = theOracle.Int(1, 4);
        theOracle.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
        int aNb = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, false, aMults);
        var aCPoles = new NCollection_Array1<Point3D>(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            aCPoles[i] = Pnt(theOracle, -5, 5);
        }

        var aCurve = new BSplineCurve3D(aCPoles, aKnots, aMults, aDeg);
        N.Geom_BSplineCurve aNCurve = Oracle.ToNativeCurve(aCPoles, null, aKnots, aMults, aDeg, false);
        double f = aKnots[aKnots.Lower()], l = aKnots[aKnots.Upper()];
        Twin<N.TopoDS_Vertex, Vertex> aV1 = Twins.Vertex(aCurve.Value(f)), aV2 = Twins.Vertex(aCurve.Value(l));
        Twin<N.TopoDS_Edge, Edge> anEdge = Twins.EdgeOn(aNCurve, aCurve, new Placement(), null, f, l, aV1, aV2);
        return TwinShapes.Up(Twins.Compound(TwinShapes.Up(aFace), TwinShapes.Up(anEdge)));
    }

    /// <summary>Attaches to each face of a box a two-triangle mesh of its quadrangle, wound as the face's surface.</summary>
    public void AttachQuadMeshes(Twin<N.TopoDS_Shape, Shape> theBox)
    {
        var aNE = new N.TopExp_Explorer(theBox.N, N.TopAbs_ShapeEnum.TopAbs_FACE);
        for (var aPE = new Explorer(theBox.P, ShapeType.Face); aPE.More(); aPE.Next(), aNE.Next())
        {
            Face aF = ShapeCasts.Face(aPE.Current());
            var aCorners = new List<Point3D>();
            for (var aW = new Explorer(aF.Oriented(Forward), ShapeType.Edge); aW.More(); aW.Next())
            {
                Exploration.Vertices(ShapeCasts.Edge(aW.Current()), out Vertex aFirst, out _, true);
                aCorners.Add(Tool.Pnt(aFirst));
            }

            var aNT = new N.Poly_Triangulation(4, 2, false);
            var aT = new Triangulation(4, 2, false);
            for (int i = 0; i < 4; i++)
            {
                using var aNP = Oracle.ToNative(aCorners[i]);
                aNT.SetNode(i + 1, aNP);
                aT.SetNode(i + 1, aCorners[i]);
            }

            using (var aT1 = new N.Poly_Triangle(1, 2, 3))
            using (var aT2 = new N.Poly_Triangle(1, 3, 4))
            {
                aNT.SetTriangle(1, aT1);
                aNT.SetTriangle(2, aT2);
            }

            aT.SetTriangle(1, new Triangle1(1, 2, 3));
            aT.SetTriangle(2, new Triangle1(1, 3, 4));
            myN.UpdateFace(N.TopoDS.Face(aNE.Current()), aNT);
            myP.UpdateFace(aF, aT);
        }
    }

    /// <summary>A tetrahedron of faces that are only a triangle each (no surface), in a closed shell, in a solid.</summary>
    public Twin<N.TopoDS_Shape, Shape> MeshTetra(Oracle theOracle)
    {
        Point3D[] aNodes = [Pnt(theOracle, -3, 3), Pnt(theOracle, -3, 3), Pnt(theOracle, -3, 3), Pnt(theOracle, -3, 3)];
        // orient the triangles outwards
        Vector3D aN = new Vector3D(aNodes[0], aNodes[1]).Crossed(new Vector3D(aNodes[0], aNodes[2]));
        if (aN.Dot(new Vector3D(aNodes[0], aNodes[3])) > 0)
        {
            (aNodes[1], aNodes[2]) = (aNodes[2], aNodes[1]);
        }

        var aFaces = new List<Twin<N.TopoDS_Shape, Shape>>();
        foreach ((int, int, int) aTri in new[] { (1, 2, 3), (1, 4, 2), (1, 3, 4), (2, 4, 3) })
        {
            var aNT = new N.Poly_Triangulation(4, 1, false);
            var aT = new Triangulation(4, 1, false);
            for (int i = 0; i < 4; i++)
            {
                using var aNP = Oracle.ToNative(aNodes[i]);
                aNT.SetNode(i + 1, aNP);
                aT.SetNode(i + 1, aNodes[i]);
            }

            using (var aNTri = new N.Poly_Triangle(aTri.Item1, aTri.Item2, aTri.Item3))
            {
                aNT.SetTriangle(1, aNTri);
            }

            aT.SetTriangle(1, new Triangle1(aTri.Item1, aTri.Item2, aTri.Item3));
            var aNF = new N.TopoDS_Face();
            myN.MakeFace(aNF, aNT);
            Face aF = new();
            myP.MakeFace(ref aF, aT);
            aFaces.Add(new(aNF, aF));
        }

        return TwinShapes.Up(Solid([.. aFaces]));
    }
}
