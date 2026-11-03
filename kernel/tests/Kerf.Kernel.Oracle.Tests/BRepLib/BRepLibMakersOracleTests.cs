// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The port's BRepLib makers against native OCCT's BRepBuilderAPI makers (which wrap native BRepLib's) on the same
/// random inputs, compared through BRep text: both results are written with <c>BRepTools::Write</c> and the files
/// compared. Inputs on the port's side are frozen, as callers hold them; a frozen sub-shape added to a new shape keeps
/// its Free flag where native OCCT clears it (design 4.4), so files that differ are compared again with the Free column
/// masked, and the test requires every file to match that way. The share that is byte-identical as written is reported.
/// </summary>
public sealed class BRepLibMakersOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-breplib-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private sealed class Tally(string theWhat)
    {
        public int Files;
        public int Identical;
        public int MaskedEqual;
        public int Failed;
        public int ErrorsCompared;
        public readonly List<string> Differences = [];

        public string Report() =>
            $"{theWhat}: {Files} files, {Identical} byte-identical, {MaskedEqual} equal with the Free column masked, "
            + $"{Failed} different; {ErrorsCompared} failed constructions with equal errors";
    }

    private void Compare(Tally theTally, string theWhat, N.TopoDS_Shape theNative, in Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        theTally.Files++;
        if (aN == aP)
        {
            theTally.Identical++;
        }
        else if (MaskFree(aN) == MaskFree(aP))
        {
            theTally.MaskedEqual++;
        }
        else
        {
            theTally.Failed++;
            if (theTally.Differences.Count < 5)
            {
                theTally.Differences.Add($"{theWhat}:\n--- native\n{aN}\n--- port\n{aP}");
            }
        }
    }

    private void Finish(Tally theTally)
    {
        output.WriteLine(theTally.Report());
        foreach (string aD in theTally.Differences)
        {
            output.WriteLine(aD);
        }
        Assert.Equal(0, theTally.Failed);
    }

    // ------------------------------------------------------------ settled inputs

    // Placements that native OCCT and the port hold with the same numbers: the native one is rebuilt from its own
    // directions until stable, and the port's, built from those numbers, must come out the same; otherwise another
    // random placement is drawn. (gp_Ax2's constructor recomputes the X direction, and the two compilers don't always
    // round it alike.)
    private static bool Same(N.gp_Dir theN, in Direction3D theP) => theN.X() == theP.X() && theN.Y() == theP.Y() && theN.Z() == theP.Z();

    private static Direction3D FromNative(N.gp_Dir theD) => new(theD.X(), theD.Y(), theD.Z());

    // A direction that native OCCT and the port normalise to the same numbers.
    private static Direction3D Dir(Oracle theO)
    {
        while (true)
        {
            Direction3D aD = Oracle.Settled(theO.Dir());
            using var aND = Oracle.ToNative(aD);
            if (Same(aND, aD))
            {
                return aD;
            }
        }
    }

    private static Frame3D Ax2(Oracle theO)
    {
        while (true)
        {
            Point3D aP = theO.Pnt();
            using var aNP = Oracle.ToNative(aP);
            using var aN0 = Oracle.ToNative(theO.Dir());
            using var aX0 = Oracle.ToNative(theO.Dir());
            var aNAx = new N.gp_Ax2(aNP, aN0, aX0);
            for (int i = 0; i < 4; i++)
            {
                using var aD = aNAx.Direction();
                using var aX = aNAx.XDirection();
                aNAx = new N.gp_Ax2(aNP, aD, aX);
            }
            using var aND = aNAx.Direction();
            using var aNX = aNAx.XDirection();
            using var aNY = aNAx.YDirection();
            var anAx = new Frame3D(aP, FromNative(aND), FromNative(aNX));
            if (Same(aND, anAx.Direction()) && Same(aNX, anAx.XDirection()) && Same(aNY, anAx.YDirection()))
            {
                return anAx;
            }
        }
    }

    private static CoordinateSystem Ax3(Oracle theO, bool theDirect = true)
    {
        while (true)
        {
            Frame3D anAx2 = Ax2(theO);
            var anAx = new CoordinateSystem(anAx2);
            if (!theDirect)
            {
                anAx = anAx.YReversed();
            }
            using var aNAx = Oracle.ToNative(anAx);
            using var aND = aNAx.Direction();
            using var aNX = aNAx.XDirection();
            using var aNY = aNAx.YDirection();
            if (Same(aND, anAx.Direction()) && Same(aNX, anAx.XDirection()) && Same(aNY, anAx.YDirection()))
            {
                return anAx;
            }
        }
    }

    // ------------------------------------------------------------ vertices and edges

    [Fact]
    public void Vertices_and_edges_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3401);
        var aT = new Tally("edges");
        foreach (int aCase in Tape.Budget(600))
        {
            int aKind = aCase % 12;
            N.BRepBuilderAPI_MakeEdge aNM;
            LibraryMakeEdge aPM;
            string aWhat;
            switch (aKind)
            {
                case 0:
                {
                    // two points (identical a tenth of the time)
                    Point3D aP1 = anO.Pnt();
                    Point3D aP2 = anO.Int(0, 9) == 0 ? aP1 : anO.Pnt();
                    using var aN1 = Oracle.ToNative(aP1);
                    using var aN2 = Oracle.ToNative(aP2);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aN1, aN2);
                    aPM = new LibraryMakeEdge(aP1, aP2);
                    aWhat = "points";
                    break;
                }
                case 1:
                {
                    // two vertices made by the vertex makers
                    Point3D aP1 = anO.Pnt(), aP2 = anO.Pnt();
                    using var aN1 = Oracle.ToNative(aP1);
                    using var aN2 = Oracle.ToNative(aP2);
                    N.TopoDS_Vertex aNV1 = new N.BRepBuilderAPI_MakeVertex(aN1).Vertex(), aNV2 = new N.BRepBuilderAPI_MakeVertex(aN2).Vertex();
                    Vertex aV1 = new LibraryMakeVertex(aP1).Vertex(), aV2 = new LibraryMakeVertex(aP2).Vertex();
                    Compare(aT, "vertex", aNV1, aV1);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNV1, aNV2);
                    aPM = new LibraryMakeEdge(aV1, aV2);
                    aWhat = "vertices";
                    break;
                }
                case 2:
                {
                    // a line between parameters, in either order
                    Point3D aP = anO.Pnt();
                    Direction3D aD = Dir(anO);
                    double p1 = anO.Real(-50, 50), p2 = anO.Real(-50, 50);
                    using var aNP = Oracle.ToNative(aP);
                    using var aND = Oracle.ToNative(aD);
                    using var aNL = new N.gp_Lin(aNP, aND);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNL, p1, p2);
                    aPM = new LibraryMakeEdge(new Line3D(aP, aD), p1, p2);
                    aWhat = "line";
                    break;
                }
                case 3:
                {
                    // a whole circle, or an arc between parameters
                    Frame3D anAx = Ax2(anO);
                    double r = anO.Real(0.1, 20);
                    using var aNAx = Oracle.ToNative(anAx);
                    using var aNC = new N.gp_Circ(aNAx, r);
                    if (anO.Bool())
                    {
                        aNM = new N.BRepBuilderAPI_MakeEdge(aNC);
                        aPM = new LibraryMakeEdge(new Circle3D(anAx, r));
                    }
                    else
                    {
                        double p1 = anO.Real(-10, 10), p2 = anO.Real(-10, 10);
                        aNM = new N.BRepBuilderAPI_MakeEdge(aNC, p1, p2);
                        aPM = new LibraryMakeEdge(new Circle3D(anAx, r), p1, p2);
                    }
                    aWhat = "circle";
                    break;
                }
                case 4:
                {
                    // an arc between points on the circle (or off it)
                    Frame3D anAx = Ax2(anO);
                    double r = anO.Real(0.1, 20);
                    var aC = new Circle3D(anAx, r);
                    Point3D aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(anO.Real(0, 6), aC);
                    Point3D aP2 = anO.Int(0, 4) == 0 ? anO.Pnt() : global::Kerf.Engine.Geometry.ElCLib.Value(anO.Real(0, 6), aC);
                    using var aNAx = Oracle.ToNative(anAx);
                    using var aNC = new N.gp_Circ(aNAx, r);
                    using var aN1 = Oracle.ToNative(aP1);
                    using var aN2 = Oracle.ToNative(aP2);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNC, aN1, aN2);
                    aPM = new LibraryMakeEdge(aC, aP1, aP2);
                    aWhat = "circle points";
                    break;
                }
                case 5:
                {
                    Frame3D anAx = Ax2(anO);
                    double a = anO.Real(1, 20), b = anO.Real(0.1, 1) * a;
                    double p1 = anO.Real(-5, 5), p2 = anO.Real(-5, 5);
                    using var aNAx = Oracle.ToNative(anAx);
                    using var aNE = new N.gp_Elips(aNAx, a, b);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNE, p1, p2);
                    aPM = new LibraryMakeEdge(new Ellipse3D(anAx, a, b), p1, p2);
                    aWhat = "ellipse";
                    break;
                }
                case 6:
                {
                    Frame3D anAx = Ax2(anO);
                    double a = anO.Real(0.5, 10), b = anO.Real(0.5, 10);
                    double p1 = anO.Real(-2, 2), p2 = anO.Real(-2, 2);
                    using var aNAx = Oracle.ToNative(anAx);
                    using var aNH = new N.gp_Hypr(aNAx, a, b);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNH, p1, p2);
                    aPM = new LibraryMakeEdge(new Hyperbola3D(anAx, a, b), p1, p2);
                    aWhat = "hyperbola";
                    break;
                }
                case 7:
                {
                    Frame3D anAx = Ax2(anO);
                    double f = anO.Real(0.1, 5);
                    double p1 = anO.Real(-10, 10), p2 = anO.Real(-10, 10);
                    using var aNAx = Oracle.ToNative(anAx);
                    using var aNP = new N.gp_Parab(aNAx, f);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNP, p1, p2);
                    aPM = new LibraryMakeEdge(new Parabola3D(anAx, f), p1, p2);
                    aWhat = "parabola";
                    break;
                }
                case 8:
                case 9:
                {
                    // a B-spline, whole or between parameters (sometimes out of range), trimmed half the time
                    int aDeg = anO.Int(1, 4);
                    anO.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
                    int aNbPoles = 0;
                    for (int i = aMults.Lower(); i <= aMults.Upper(); i++)
                    {
                        aNbPoles += aMults[i];
                    }
                    aNbPoles -= aDeg + 1;
                    var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
                    for (int i = 1; i <= aNbPoles; i++)
                    {
                        aPoles[i] = anO.Pnt();
                    }
                    var aC = new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
                    N.Geom_BSplineCurve aNC = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false);
                    double f = aKnots[aKnots.Lower()], l = aKnots[aKnots.Upper()];
                    double p1 = f + anO.Real(-0.1, 0.5) * (l - f), p2 = f + anO.Real(0.5, 1.1) * (l - f);
                    if (aKind == 8)
                    {
                        aNM = new N.BRepBuilderAPI_MakeEdge(aNC, p1, p2);
                        aPM = new LibraryMakeEdge(aC, p1, p2);
                    }
                    else
                    {
                        double t1 = f + 0.1 * (l - f), t2 = f + 0.9 * (l - f);
                        var aNT = new N.Geom_TrimmedCurve(aNC, t1, t2);
                        aNM = new N.BRepBuilderAPI_MakeEdge(aNT);
                        aPM = new LibraryMakeEdge(new TrimmedCurve3D(aC, t1, t2));
                    }
                    aWhat = "bspline";
                    break;
                }
                case 10:
                {
                    // a vertex given at a point near or away from the curve (DifferentsPointAndParameter)
                    Point3D aP = anO.Pnt();
                    Direction3D aD = Dir(anO);
                    var aL = new LineCurve3D(aP, aD);
                    using var aNP0 = Oracle.ToNative(aP);
                    using var aND = Oracle.ToNative(aD);
                    var aNL = new N.Geom_Line(aNP0, aND);
                    double p1 = anO.Real(-5, 0), p2 = anO.Real(1, 5);
                    Point3D aQ1 = aL.Value(p1), aQ2 = aL.Value(p2).Translated(new Vector3D(anO.Real(-1, 1), anO.Real(-1, 1), anO.Real(-1, 1)) * (anO.Bool() ? 1e-9 : 1e-3));
                    using var aNQ1 = Oracle.ToNative(aQ1);
                    using var aNQ2 = Oracle.ToNative(aQ2);
                    aNM = new N.BRepBuilderAPI_MakeEdge(aNL, aNQ1, aNQ2, p1, p2);
                    aPM = new LibraryMakeEdge(aL, aQ1, aQ2, p1, p2);
                    aWhat = "points and parameters";
                    break;
                }
                default:
                {
                    // a curve on a surface: a 2D line or circle on a plane or a cylinder
                    CoordinateSystem anAx = Ax3(anO, anO.Bool());
                    using var aNAx = Oracle.ToNative(anAx);
                    Surface aS;
                    N.Geom_Surface aNS;
                    if (anO.Bool())
                    {
                        aS = new PlaneSurface(anAx);
                        aNS = new N.Geom_Plane(aNAx);
                    }
                    else
                    {
                        double r = anO.Real(0.5, 10);
                        aS = new CylindricalSurface(anAx, r);
                        aNS = new N.Geom_CylindricalSurface(aNAx, r);
                    }
                    Point2D aP = new(anO.Real(-3, 3), anO.Real(-3, 3));
                    Direction2D aD = anO.Dir2d();
                    using var aNP = Oracle.ToNative(aP);
                    using var aND = Oracle.ToNative(aD);
                    double p1 = anO.Real(-3, 3), p2 = anO.Real(-3, 3);
                    if (anO.Bool())
                    {
                        aNM = new N.BRepBuilderAPI_MakeEdge(new N.Geom2d_Line(aNP, aND), aNS, p1, p2);
                        aPM = new LibraryMakeEdge(new LineCurve2D(aP, aD), aS, p1, p2);
                    }
                    else
                    {
                        double r = anO.Real(0.2, 2);
                        using var aNA2 = new N.gp_Ax2d(aNP, aND);
                        aNM = new N.BRepBuilderAPI_MakeEdge(new N.Geom2d_Circle(aNA2, r), aNS);
                        aPM = new LibraryMakeEdge(new CircleCurve2D(new Axis2D(aP, aD), r), aS);
                    }
                    aWhat = "curve on surface";
                    break;
                }
            }

            Assert.Equal((int)aNM.Error(), (int)aPM.Error());
            Assert.Equal(aNM.IsDone(), aPM.IsDone);
            if (aNM.IsDone())
            {
                Compare(aT, aWhat, aNM.Edge(), aPM.Edge());
            }
            else
            {
                aT.ErrorsCompared++;
            }
        }
        Finish(aT);
    }

    // ------------------------------------------------------------ wires and polygons

    [Fact]
    public void Wires_and_polygons_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3402);
        var aT = new Tally("wires");
        foreach (int aCase in Tape.Budget(400))
        {
            int aKind = aCase % 5;
            int n = anO.Int(3, 7);
            var aPts = new Point3D[n];
            for (int i = 0; i < n; i++)
            {
                aPts[i] = anO.Pnt();
            }

            if (aKind == 0)
            {
                // polygons, closed half the time, with a repeated point now and then
                if (anO.Int(0, 3) == 0)
                {
                    aPts[n / 2] = aPts[n / 2 - 1];
                }
                bool aClose = anO.Bool();
                var aNP = new N.BRepBuilderAPI_MakePolygon();
                var aPP = new LibraryMakePolygon();
                foreach (Point3D aP in aPts)
                {
                    using var aN = Oracle.ToNative(aP);
                    aNP.Add(aN);
                    aPP.Add(aP);
                    Assert.Equal(aNP.Added(), aPP.Added());
                }
                if (aClose)
                {
                    aNP.Close();
                    aPP.Close();
                }
                Assert.Equal(aNP.IsDone(), aPP.IsDone);
                Compare(aT, "polygon", aNP.Wire(), aPP.Wire());
                continue;
            }

            // edges between shared vertices (kind 1), or each with its own vertices, which coincide (kinds 2 to 4);
            // kind 3 moves each next start within the vertices' tolerance; kind 4 adds them as a shuffled list
            var aNV = new N.TopoDS_Vertex[n];
            var aPV = new Vertex[n];
            for (int i = 0; i < n; i++)
            {
                using var aN = Oracle.ToNative(aPts[i]);
                aNV[i] = new N.BRepBuilderAPI_MakeVertex(aN).Vertex();
                aPV[i] = new LibraryMakeVertex(aPts[i]).Vertex();
            }
            bool aClosed = anO.Bool();
            int aNbEdges = aClosed ? n : n - 1;
            var aNE = new N.TopoDS_Edge[aNbEdges];
            var aPE = new Edge[aNbEdges];
            for (int i = 0; i < aNbEdges; i++)
            {
                int j = (i + 1) % n;
                if (aKind == 1)
                {
                    aNE[i] = new N.BRepBuilderAPI_MakeEdge(aNV[i], aNV[j]).Edge();
                    aPE[i] = new LibraryMakeEdge(aPV[i], aPV[j]).Edge();
                }
                else
                {
                    Point3D aStart = aPts[i];
                    if (aKind == 3 && i > 0)
                    {
                        aStart = aStart.Translated(new Vector3D(anO.Real(-1, 1), anO.Real(-1, 1), anO.Real(-1, 1)) * 4e-8);
                    }
                    using var aN1 = Oracle.ToNative(aStart);
                    using var aN2 = Oracle.ToNative(aPts[j]);
                    aNE[i] = new N.BRepBuilderAPI_MakeEdge(aN1, aN2).Edge();
                    aPE[i] = new LibraryMakeEdge(aStart, aPts[j]).Edge();
                }
                if (anO.Int(0, 3) == 0)
                {
                    aNE[i] = N.TopoDS.Edge(aNE[i].Reversed());
                    aPE[i] = ShapeCasts.Edge(aPE[i].Reversed());
                }
            }

            var aNW = new N.BRepBuilderAPI_MakeWire();
            var aPW = new LibraryMakeWire();
            if (aKind == 4)
            {
                int[] anOrder = Enumerable.Range(0, aNbEdges).OrderBy(_ => anO.Real()).ToArray();
                using var aNL = new N.TopTools_ListOfShape();
                var aPL = new NCollection_List<Shape>();
                foreach (int i in anOrder)
                {
                    aNL.Append(aNE[i]);
                    aPL.Append(aPE[i]);
                }
                aNW.Add(aNL);
                aPW.Add(aPL);
            }
            else
            {
                for (int i = 0; i < aNbEdges; i++)
                {
                    aNW.Add(aNE[i]);
                    aPW.Add(aPE[i]);
                    Assert.Equal((int)aNW.Error(), (int)aPW.Error());
                }
            }
            Assert.Equal((int)aNW.Error(), (int)aPW.Error());
            Assert.Equal(aNW.IsDone(), aPW.IsDone);
            if (aNW.IsDone())
            {
                Compare(aT, $"wire {aKind}", aNW.Wire(), aPW.Wire());
            }
            else
            {
                aT.ErrorsCompared++;
            }
        }
        Finish(aT);
    }

    // ------------------------------------------------------------ faces

    [Fact]
    public void Faces_on_surfaces_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3403);
        var aT = new Tally("faces on surfaces");
        foreach (int aCase in Tape.Budget(500))
        {
            int aKind = aCase % 6;
            CoordinateSystem anAx = Ax3(anO, anO.Int(0, 3) != 0);
            using var aNAx = Oracle.ToNative(anAx);
            bool aWhole = anO.Int(0, 3) == 0;
            double u1 = anO.Real(-1, 4), u2 = anO.Real(-1, 7), v1 = anO.Real(-3, 3), v2 = anO.Real(-3, 3);
            N.BRepBuilderAPI_MakeFace aNM;
            LibraryMakeFace aPM;
            string aWhat;
            switch (aKind)
            {
                case 0:
                {
                    using var aNS = new N.gp_Pln(aNAx);
                    var aS = new Plane3D(anAx);
                    (aNM, aPM) = aWhole ? (new N.BRepBuilderAPI_MakeFace(aNS), new LibraryMakeFace(aS))
                                        : (new N.BRepBuilderAPI_MakeFace(aNS, u1, u2, v1, v2), new LibraryMakeFace(aS, u1, u2, v1, v2));
                    aWhat = "plane";
                    break;
                }
                case 1:
                {
                    double r = anO.Real(0.5, 10);
                    using var aNS = new N.gp_Cylinder(aNAx, r);
                    var aS = new Cylinder(anAx, r);
                    if (anO.Bool())
                    {
                        u1 = 0;
                        u2 = 2 * Math.PI;
                    }
                    (aNM, aPM) = aWhole ? (new N.BRepBuilderAPI_MakeFace(aNS), new LibraryMakeFace(aS))
                                        : (new N.BRepBuilderAPI_MakeFace(aNS, u1, u2, v1, v2), new LibraryMakeFace(aS, u1, u2, v1, v2));
                    aWhat = "cylinder";
                    break;
                }
                case 2:
                {
                    double ang = anO.Real(0.1, 1.2), r = anO.Real(0.5, 10);
                    using var aNS = new N.gp_Cone(aNAx, ang, r);
                    var aS = new Cone(anAx, ang, r);
                    if (anO.Bool())
                    {
                        v1 = -r / Math.Sin(ang); // the apex
                    }
                    (aNM, aPM) = aWhole ? (new N.BRepBuilderAPI_MakeFace(aNS), new LibraryMakeFace(aS))
                                        : (new N.BRepBuilderAPI_MakeFace(aNS, u1, u2, v1, v2), new LibraryMakeFace(aS, u1, u2, v1, v2));
                    aWhat = "cone";
                    break;
                }
                case 3:
                {
                    double r = anO.Real(0.5, 10);
                    using var aNS = new N.gp_Sphere(aNAx, r);
                    var aS = new Sphere(anAx, r);
                    v1 = anO.Real(-1.6, 0);
                    v2 = anO.Real(0, 1.6);
                    if (anO.Bool())
                    {
                        v1 = -Math.PI / 2;
                    }
                    (aNM, aPM) = aWhole ? (new N.BRepBuilderAPI_MakeFace(aNS), new LibraryMakeFace(aS))
                                        : (new N.BRepBuilderAPI_MakeFace(aNS, u1, u2, v1, v2), new LibraryMakeFace(aS, u1, u2, v1, v2));
                    aWhat = "sphere";
                    break;
                }
                case 4:
                {
                    double r1 = anO.Real(2, 10), r2 = anO.Real(0.2, 1.9);
                    using var aNS = new N.gp_Torus(aNAx, r1, r2);
                    var aS = new Torus(anAx, r1, r2);
                    (aNM, aPM) = aWhole ? (new N.BRepBuilderAPI_MakeFace(aNS), new LibraryMakeFace(aS))
                                        : (new N.BRepBuilderAPI_MakeFace(aNS, u1, u2, v1, v2), new LibraryMakeFace(aS, u1, u2, v1, v2));
                    aWhat = "torus";
                    break;
                }
                default:
                {
                    // a Bezier or B-spline surface between parameters (sometimes out of range or reversed)
                    int nu = anO.Int(2, 4), nv = anO.Int(2, 4);
                    var aPoles = new NCollection_Array2<Point3D>(1, nu, 1, nv);
                    for (int i = 1; i <= nu; i++)
                    {
                        for (int j = 1; j <= nv; j++)
                        {
                            aPoles[i, j] = new Point3D(i * 3 + anO.Real(-1, 1), j * 3 + anO.Real(-1, 1), anO.Real(-2, 2));
                        }
                    }
                    var aS = new BezierSurface(aPoles);
                    using var aNPoles = Oracle.ToNative(aPoles);
                    var aNS = new N.Geom_BezierSurface(aNPoles);
                    double tol = anO.Bool() ? global::Kerf.Engine.Foundation.Precision.Confusion() : 1e-3;
                    if (aWhole)
                    {
                        aNM = new N.BRepBuilderAPI_MakeFace(aNS, tol);
                        aPM = new LibraryMakeFace(aS, tol);
                    }
                    else
                    {
                        double a = anO.Real(-0.2, 1), b = anO.Real(-0.2, 1.2), c = anO.Real(0, 1), d = anO.Real(0, 1);
                        aNM = new N.BRepBuilderAPI_MakeFace(aNS, a, b, c, d, tol);
                        aPM = new LibraryMakeFace(aS, a, b, c, d, tol);
                    }
                    aWhat = "bezier";
                    break;
                }
            }

            Assert.Equal((int)aNM.Error(), (int)aPM.Error());
            Assert.Equal(aNM.IsDone(), aPM.IsDone);
            if (aNM.IsDone())
            {
                Compare(aT, aWhat, aNM.Face(), aPM.Face());
            }
            else
            {
                aT.ErrorsCompared++;
            }
        }
        Finish(aT);
    }

    // A random simple polygon in the XY plane of the placement: points at increasing angles around its origin.
    private static Point3D[] StarPolygon(Oracle theO, in CoordinateSystem theAx, int theN, double theScale)
    {
        var aPts = new Point3D[theN];
        double anAngle = theO.Real(0, 1);
        for (int i = 0; i < theN; i++)
        {
            anAngle += theO.Real(0.3, 2 * Math.PI / theN);
            double r = theScale * theO.Real(0.5, 1.0);
            aPts[i] = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(r * Math.Cos(anAngle), r * Math.Sin(anAngle), theAx);
        }
        return aPts;
    }

    [Fact]
    public void Faces_on_planes_from_wires_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3404);
        var aT = new Tally("faces from wires");
        foreach (int aCase in Tape.Budget(300))
        {
            int aKind = aCase % 3;
            CoordinateSystem anAx = Ax3(anO);
            Point3D[] aPts = StarPolygon(anO, anAx, anO.Int(3, 7), anO.Real(1, 20));
            if (anO.Bool())
            {
                Array.Reverse(aPts);
            }
            var aNP = new N.BRepBuilderAPI_MakePolygon();
            var aPP = new LibraryMakePolygon();
            foreach (Point3D aP in aPts)
            {
                using var aN = Oracle.ToNative(aP);
                aNP.Add(aN);
                aPP.Add(aP);
            }
            aNP.Close();
            aPP.Close();
            N.TopoDS_Wire aNW = aNP.Wire();
            Wire aPW = aPP.Wire();

            N.BRepBuilderAPI_MakeFace aNM;
            LibraryMakeFace aPM;
            if (aKind == 0)
            {
                // a plane and the wire
                using var aNAx = Oracle.ToNative(anAx);
                using var aNPln = new N.gp_Pln(aNAx);
                bool anInside = anO.Int(0, 4) != 0;
                aNM = new N.BRepBuilderAPI_MakeFace(aNPln, aNW, anInside);
                aPM = new LibraryMakeFace(new Plane3D(anAx), aPW, anInside);
            }
            else
            {
                // the plane found through the wire (the wire's edges updated on a copy for write)
                bool anOnlyPlane = aKind == 2;
                aNM = new N.BRepBuilderAPI_MakeFace(aNW, anOnlyPlane);
                aPM = new LibraryMakeFace(aPW, anOnlyPlane);
            }
            Assert.Equal((int)aNM.Error(), (int)aPM.Error());
            Assert.Equal(aNM.IsDone(), aPM.IsDone);
            if (!aNM.IsDone())
            {
                aT.ErrorsCompared++;
                continue;
            }

            // a hole: a smaller polygon around the same centre, added to a face loaded from the result
            N.TopoDS_Face aNF = aNM.Face();
            Face aPF = aPM.Face();
            if (anO.Bool())
            {
                Point3D[] aHole = StarPolygon(anO, anAx, anO.Int(3, 6), 0.2);
                var aNH = new N.BRepBuilderAPI_MakePolygon();
                var aPH = new LibraryMakePolygon();
                foreach (Point3D aP in aHole)
                {
                    using var aN = Oracle.ToNative(aP);
                    aNH.Add(aN);
                    aPH.Add(aP);
                }
                aNH.Close();
                aPH.Close();
                aNF = new N.BRepBuilderAPI_MakeFace(aNF, N.TopoDS.Wire(aNH.Wire().Reversed())).Face();
                aPF = new LibraryMakeFace(aPF, ShapeCasts.Wire(aPH.Wire().Reversed())).Face();
            }
            Compare(aT, $"face from wire {aKind}", aNF, aPF);
        }
        Finish(aT);
    }

    // ------------------------------------------------------------ shells and solids

    [Fact]
    public void Shells_on_BSpline_surfaces_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3405);
        var aT = new Tally("shells");
        foreach (int aCase in Tape.Budget(120))
        {
            int aDegU = anO.Int(1, 3), aDegV = anO.Int(1, 3);
            anO.BSplineKnots(aDegU, false, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
            anO.BSplineKnots(aDegV, false, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
            int nu = -aDegU - 1, nv = -aDegV - 1;
            for (int i = aUMults.Lower(); i <= aUMults.Upper(); i++)
            {
                nu += aUMults[i];
            }
            for (int i = aVMults.Lower(); i <= aVMults.Upper(); i++)
            {
                nv += aVMults[i];
            }
            var aPoles = new NCollection_Array2<Point3D>(1, nu, 1, nv);
            for (int i = 1; i <= nu; i++)
            {
                for (int j = 1; j <= nv; j++)
                {
                    aPoles[i, j] = new Point3D(i * 2 + anO.Real(-0.5, 0.5), j * 2 + anO.Real(-0.5, 0.5), anO.Real(-2, 2));
                }
            }
            var aS = new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, aDegU, aDegV);
            using var aNPoles = Oracle.ToNative(aPoles);
            using var aNUK = Oracle.ToNative(aUKnots);
            using var aNVK = Oracle.ToNative(aVKnots);
            using var aNUM = Oracle.ToNative(aUMults);
            using var aNVM = Oracle.ToNative(aVMults);
            var aNS = new N.Geom_BSplineSurface(aNPoles, aNUK, aNVK, aNUM, aNVM, aDegU, aDegV);
            bool aSegment = anO.Bool();
            N.BRepBuilderAPI_MakeShell aNM;
            LibraryMakeShell aPM;
            if (anO.Bool())
            {
                aNM = new N.BRepBuilderAPI_MakeShell(aNS, aSegment);
                aPM = new LibraryMakeShell(aS, aSegment);
            }
            else
            {
                double u1 = aUKnots[1] + anO.Real(0, 0.3), u2 = aUKnots[aUKnots.Upper()] - anO.Real(0, 0.3);
                double v1 = aVKnots[1] + anO.Real(0, 0.3), v2 = aVKnots[aVKnots.Upper()] - anO.Real(0, 0.3);
                aNM = new N.BRepBuilderAPI_MakeShell(aNS, u1, u2, v1, v2, aSegment);
                aPM = new LibraryMakeShell(aS, u1, u2, v1, v2, aSegment);
            }
            Assert.Equal((int)aNM.Error(), (int)aPM.Error());
            Assert.Equal(aNM.IsDone(), aPM.IsDone);
            if (aNM.IsDone())
            {
                Compare(aT, "shell", aNM.Shell(), aPM.Shell());
            }
            else
            {
                aT.ErrorsCompared++;
            }
        }
        Finish(aT);
    }

    [Fact]
    public void Solids_of_boxes_made_with_the_makers_are_built_as_BRepBuilderAPI_builds_them()
    {
        var anO = new Oracle(output, 3406);
        var aT = new Tally("solids");
        foreach (int aCase in Tape.Budget(60))
        {
            CoordinateSystem anAx = Ax3(anO);
            double dx = anO.Real(0.5, 10), dy = anO.Real(0.5, 10), dz = anO.Real(0.5, 10);
            (N.TopoDS_Shell N, Shell P) aShell = BoxShell(anAx, dx, dy, dz);
            N.BRepBuilderAPI_MakeSolid aNM;
            LibraryMakeSolid aPM;
            if (aCase % 2 == 0)
            {
                aNM = new N.BRepBuilderAPI_MakeSolid(aShell.N);
                aPM = new LibraryMakeSolid(aShell.P);
            }
            else
            {
                (N.TopoDS_Shell N, Shell P) aInner = BoxShell(anAx, dx / 2, dy / 2, dz / 2);
                aNM = new N.BRepBuilderAPI_MakeSolid(aShell.N, N.TopoDS.Shell(aInner.N.Reversed()));
                aPM = new LibraryMakeSolid(aShell.P, ShapeCasts.Shell(aInner.P.Reversed()));
            }
            Assert.Equal(aNM.IsDone(), aPM.IsDone);
            Compare(aT, "solid", aNM.Solid(), aPM.Solid());
        }
        Finish(aT);
    }

    // A box [0, dx] x [0, dy] x [0, dz] in the placement: 8 vertices, 12 edges, 6 wires and 6 faces from the makers
    // (MakeVertex, MakeEdge, MakeWire, MakeFace(plane, wire)), sewn into a shell with BRep_Builder on both sides.
    private static (N.TopoDS_Shell, Shell) BoxShell(in CoordinateSystem theAx, double dx, double dy, double dz)
    {
        var aP = new Point3D[8];
        for (int i = 0; i < 8; i++)
        {
            aP[i] = global::Kerf.Engine.Geometry.ElSLib.PlaneValue((i & 1) * dx, ((i >> 1) & 1) * dy, theAx).Translated(
                new Vector3D(theAx.Direction()) * (((i >> 2) & 1) * dz));
        }
        var aNV = new N.TopoDS_Vertex[8];
        var aPV = new Vertex[8];
        for (int i = 0; i < 8; i++)
        {
            using var aN = Oracle.ToNative(aP[i]);
            aNV[i] = new N.BRepBuilderAPI_MakeVertex(aN).Vertex();
            aPV[i] = new LibraryMakeVertex(aP[i]).Vertex();
        }
        int[][] anEdges = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
        var aNE = new N.TopoDS_Edge[12];
        var aPE = new Edge[12];
        for (int i = 0; i < 12; i++)
        {
            aNE[i] = new N.BRepBuilderAPI_MakeEdge(aNV[anEdges[i][0]], aNV[anEdges[i][1]]).Edge();
            aPE[i] = new LibraryMakeEdge(aPV[anEdges[i][0]], aPV[anEdges[i][1]]).Edge();
        }
        // faces by their edges, in order around them
        int[][] aFaces = [[0, 5, 1, 4], [2, 7, 3, 6], [0, 9, 2, 8], [1, 11, 3, 10], [4, 10, 6, 8], [5, 11, 7, 9]];
        var aNB = new N.BRep_Builder();
        var aPB = new BrepBuilder();
        var aNShell = new N.TopoDS_Shell();
        aNB.MakeShell(aNShell);
        Shell aPShell = new();
        aPB.MakeShell(ref aPShell);
        foreach (int[] aF in aFaces)
        {
            var aNW = new N.BRepBuilderAPI_MakeWire();
            var aPW = new LibraryMakeWire();
            foreach (int e in aF)
            {
                aNW.Add(aNE[e]);
                aPW.Add(aPE[e]);
            }
            N.TopoDS_Face aNF = new N.BRepBuilderAPI_MakeFace(aNW.Wire(), true).Face();
            Face aPF = new LibraryMakeFace(aPW.Wire(), true).Face();
            aNB.Add(aNShell, aNF);
            aPB.Add(aPShell, aPF);
        }
        return (aNShell, aPShell);
    }

    // ------------------------------------------------------------ FindSurface

    [Fact]
    public void FindSurface_answers_as_native_OCCT()
    {
        var anO = new Oracle(output, 3407);
        var aT = new Tally("surfaces found");
        int aFound = 0, anExisted = 0, aValues = 0;
        foreach (int aCase in Tape.Budget(300))
        {
            CoordinateSystem anAx = Ax3(anO);
            Point3D[] aPts = StarPolygon(anO, anAx, anO.Int(3, 8), anO.Real(1, 20));
            // lift some points off the plane by up to 1e-6, 1e-4 or 1e-2
            double aLift = aCase % 4 == 0 ? 0 : Math.Pow(10, -2 * (aCase % 4));
            for (int i = 0; i < aPts.Length; i++)
            {
                aPts[i] = aPts[i].Translated(new Vector3D(anAx.Direction()) * (anO.Real(-1, 1) * aLift));
            }
            var aNP = new N.BRepBuilderAPI_MakePolygon();
            var aPP = new LibraryMakePolygon();
            foreach (Point3D aP in aPts)
            {
                using var aN = Oracle.ToNative(aP);
                aNP.Add(aN);
                aPP.Add(aP);
            }
            if (anO.Int(0, 4) != 0)
            {
                aNP.Close();
                aPP.Close();
            }
            N.TopoDS_Shape aNS = aNP.Wire();
            Shape aPS = aPP.Wire();
            if (aCase % 5 == 0)
            {
                // a face of a cylinder or sphere between parameters: its edges have pcurves on the surface
                using var aNAx = Oracle.ToNative(anAx);
                double r = anO.Real(1, 10), u1 = anO.Real(0, 2), u2 = u1 + anO.Real(0.5, 4), v1 = anO.Real(-1.2, 0), v2 = anO.Real(0.1, 1.2);
                if (anO.Bool())
                {
                    using var aNC = new N.gp_Cylinder(aNAx, r);
                    aNS = new N.BRepBuilderAPI_MakeFace(aNC, u1, u2, v1, v2).Face();
                    aPS = new LibraryMakeFace(new Cylinder(anAx, r), u1, u2, v1, v2).Face();
                }
                else
                {
                    using var aNC = new N.gp_Sphere(aNAx, r);
                    aNS = new N.BRepBuilderAPI_MakeFace(aNC, u1, u2, v1, v2).Face();
                    aPS = new LibraryMakeFace(new Sphere(anAx, r), u1, u2, v1, v2).Face();
                }
            }
            double aTol = aCase % 3 == 0 ? -1 : Math.Pow(10, -anO.Int(2, 8));
            bool anOnlyPlane = anO.Bool(), anOnlyClosed = anO.Bool();
            using var aNF = new N.BRepLib_FindSurface(aNS, aTol, anOnlyPlane, anOnlyClosed);
            var aPF = new FindSurface(aPS, aTol, anOnlyPlane, anOnlyClosed);
            Assert.Equal(aNF.Found(), aPF.Found());
            Assert.Equal(aNF.Existed(), aPF.Existed());
            Assert.Equal(aNF.Tolerance(), aPF.Tolerance());
            Assert.Equal(aNF.ToleranceReached(), aPF.ToleranceReached());
            aValues += 4;
            if (aPF.Found())
            {
                aFound++;
                if (aPF.Existed())
                {
                    anExisted++;
                }
                // the surface and its location, through a face on them
                var aNB = new N.BRep_Builder();
                var aNFace = new N.TopoDS_Face();
                aNB.MakeFace(aNFace, aNF.Surface(), aNF.Location(), 1e-7);
                Face aPFace = new();
                new BrepBuilder().MakeFace(ref aPFace, aPF.Surface(), aPF.Location(), 1e-7);
                Compare(aT, "found surface", aNFace, aPFace);
            }
        }
        output.WriteLine($"FindSurface: 300 shapes (a fifth of them faces with pcurves), {aFound} found ({anExisted} existing), {aValues} values equal");
        Finish(aT);
    }
}
