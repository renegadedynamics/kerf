#if HAS_ORACLE || HAS_NATIVE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text;
using Build123d;
using Occt;
using B = Build123d;
using N = Build123d.Occt;
using Ops = Build123d.Operations;

namespace OcctNet.Oracle.Tests;

/// <summary>The phase 7 calls build123d.NET makes: the native operation and how build123d.NET finishes its result.</summary>
internal enum P7Op { ThickSolid, MakeOffset, Draft, DPrism, Split, ToSplines }

/// <summary>
/// What build123d.NET does with the call's result before handing it back (compared separately, on native's result in
/// both): <c>Hollow</c> without openings makes a solid of the two shells and fixes it; <c>Offset3D</c> makes the solid
/// FORWARD and reverses an inside-out one (and the Offset operation fixes it); the Thicken operation cleans; ExtrudeTaper
/// checks the drafted solid's validity; SplitByPerimeter sews each side of more than one face.
/// </summary>
internal enum P7Post { None, HollowSolid, Offset3D, OffsetOperation, Clean, Validity, SewSides }

/// <summary>The arguments of one call (the faces of <c>Faces</c> are indices in the first operand's face map).</summary>
internal sealed record P7Args(double Value = 0, double Tol = 0, int Join = 0, bool Inter = false, bool Thickening = false,
                              bool RemoveIntEdges = false, bool ThreeArgs = false, int[]? Faces = null, double[]? Angles = null,
                              double[]? Dir = null, double[]? PlnLoc = null, int Mode = 0, double Taper = 0, int Fuse = 0,
                              bool HasBasis = false, int NbProfiles = 1, int Degree = 3, bool Nurbs = false);

/// <summary>
/// One native call of build123d.NET's phase 7 path, from one of its tests: its operands (as native shapes), arguments and
/// finishing step, and build123d.NET's own result of the operation (or its exception), which checks the transcription.
/// </summary>
internal sealed record P7Call(string Name, string Test, P7Op Op, N.TopoDS_Shape[] Operands, P7Args Args, P7Post Post,
                              B.Shape? Result, string? Error);

/// <summary>A native run: the exception, the status, the result, the history, and SplitShape's Left and Right.</summary>
internal sealed record P7Native(string? Error, string Status, N.TopoDS_Shape? Result, string History,
                                N.TopoDS_Shape[]? Left = null, N.TopoDS_Shape[]? Right = null);

/// <summary>A port's run, as <see cref="P7Native"/>.</summary>
internal sealed record P7Port(string? Error, string Status, TopoDS_Shape? Result, string History,
                              TopoDS_Shape[]? Left = null, TopoDS_Shape[]? Right = null);

/// <summary>
/// The phase 7 calls build123d.NET's own tests make (unit 7X, plan 1.5 and milestone 2), rebuilt with its public API as
/// unit 6M rebuilt its Booleans (decision 947): <c>ShapeCustom::BSplineRestriction</c> (Shape.ToSplines),
/// <c>BRepOffsetAPI_MakeThickSolid::MakeThickSolidByJoin</c> (Hollow, Offset3D and the Offset operation),
/// <c>BRepOffset_MakeOffset</c> (Solid.Thicken and the Thicken operation, Shape2D.Offset, OneD.OffsetTopodsFace),
/// <c>BRepOffsetAPI_DraftAngle</c> (Solid.Draft, the Draft operation, ExtrudeTaper's draft) and the features
/// <c>BRepFeat_MakeDPrism</c> (DPrism) and <c>BRepFeat_SplitShape</c> (SplitByPerimeter). Each call is transcribed from
/// build123d.NET's source with its arguments and order; its inputs are those of the tests (Shape3DTests, SolidTests,
/// OperationsGenericTests, OperationsPartTests, FaceTests, ShellTests, TopologyReferenceTests, MalformedInputTests,
/// InputsUnchangedTests). The runners (shared with the benchmarks) run one call on either side from operands read from
/// the same BRep text.
/// </summary>
internal static class Build123dPhase7Capture
{
    // ---------------------------------------------------------------------------------------------------------------
    // The cases

    /// <summary>Every call, in the tests' order; <paramref name="theFailure"/> hears of scenarios that failed to build.</summary>
    internal static List<P7Call> Capture(Action<string>? theFailure = null)
    {
        var aCalls = new List<P7Call>();
        foreach ((string aTest, Action<List<P7Call>, string> aScenario) in Scenarios())
        {
            try
            {
                aScenario(aCalls, aTest);
            }
            catch (Exception e)
            {
                theFailure?.Invoke($"{aTest}: {e.GetType().Name}: {e.Message}");
            }
        }
        return aCalls;
    }

    private const double DegToRad = Math.PI / 180.0; // build123d.NET's Constants.DegToRad

    private static string? ErrorOfB(Exception e) => e.GetType().Name;

    // build123d.NET's own result of the operation (or its exception's type)
    private static (B.Shape?, string?) Own(Func<B.Shape> theRun)
    {
        try
        {
            return (theRun(), null);
        }
        catch (Exception e)
        {
            return (null, ErrorOfB(e));
        }
    }

    private static int[] FaceIndices(B.Shape theShape, IEnumerable<B.Face> theFaces)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape.Wrapped, N.TopAbs_ShapeEnum.TopAbs_FACE, aMap);
        return [.. theFaces.Select(f => aMap.FindIndex(f.Wrapped))];
    }

    private static int JoinOf(B.Kind theKind) => theKind == B.Kind.Arc ? 0 : 2; // GeomAbs_Arc, GeomAbs_Intersection

    /// <summary>Shape3D.Hollow: MakeThickSolidByJoin (Skin, intersection on, no self-intersection, the join), then Build.</summary>
    private static void Hollow(List<P7Call> theCalls, string theTest, string theName, B.Shape3D theShape, IReadOnlyList<B.Face> theFaces,
                               double theThickness, double theTolerance = 0.0001, B.Kind theKind = B.Kind.Arc)
    {
        (B.Shape? aResult, string? anError) = Own(() => theShape.Hollow(theFaces, theThickness, theTolerance, theKind));
        theCalls.Add(new P7Call(theName, theTest, P7Op.ThickSolid, [theShape.Wrapped],
                                new P7Args(theThickness, theTolerance, JoinOf(theKind), Inter: true, Faces: FaceIndices(theShape, theFaces)),
                                theFaces.Count == 0 ? P7Post.HollowSolid : P7Post.None, aResult, anError));
    }

    /// <summary>Shape3D.Offset3D: as Hollow, removing the internal edges; the Offset operation fixes its result.</summary>
    private static void Offset3D(List<P7Call> theCalls, string theTest, string theName, B.Shape3D theShape, IReadOnlyList<B.Face> theOpenings,
                                 double theThickness, B.Kind theKind = B.Kind.Arc, bool theOperation = false)
    {
        (B.Shape? aResult, string? anError) = Own(() => theOperation
                                                            ? theShape.Offset3D(theOpenings, theThickness, kind: theKind).Fix()
                                                            : theShape.Offset3D(theOpenings, theThickness, kind: theKind));
        theCalls.Add(new P7Call(theName, theTest, P7Op.ThickSolid, [theShape.Wrapped],
                                new P7Args(theThickness, 0.0001, JoinOf(theKind), Inter: true, RemoveIntEdges: true,
                                           Faces: FaceIndices(theShape, theOpenings)),
                                theOperation ? P7Post.OffsetOperation : P7Post.Offset3D, aResult, anError));
    }

    /// <summary>The Offset operation on a solid or Part: Offset3D of each solid with its own openings, then Fix.</summary>
    private static void OffsetOperation(List<P7Call> theCalls, string theTest, string theName, B.Shape theObject, double theAmount,
                                        IEnumerable<B.Face>? theOpenings = null, B.Kind theKind = B.Kind.Arc)
    {
        List<B.Face> anOpenings = [.. theOpenings ?? []];
        var aSolids = theObject is B.Solid s ? [s] : theObject.Solids().ToList();
        int k = 0;
        foreach (B.Solid aSolid in aSolids)
        {
            var aFaces = aSolid.Faces();
            Offset3D(theCalls, theTest, aSolids.Count > 1 ? $"{theName} solid {k++}" : theName, aSolid,
                     [.. anOpenings.Where(aFaces.Contains)], theAmount, theKind, true);
        }
    }

    /// <summary>Solid.Thicken: BRepOffset_MakeOffset's nine-argument Initialize (Skin, intersection, thickening, removing the internal edges), MakeOffsetShape.</summary>
    private static void Thicken(List<P7Call> theCalls, string theTest, string theName, B.Shape2D theSurface, double theDepth,
                                B.Vector? theNormalOverride = null, bool theClean = false)
    {
        double anAdjusted = theDepth;
        if (theSurface is B.Face aFace && theNormalOverride is { } anUp)
        {
            var aNormal = aFace.NormalAt(aFace.Center()).Normalized();
            if (aNormal.Dot(anUp.Normalized()) < 0)
            {
                anAdjusted = -theDepth;
            }
        }
        (B.Shape? aResult, string? anError) = Own(() => theClean
                                                            ? B.Solid.Thicken(theSurface, theDepth, theNormalOverride).Clean()
                                                            : B.Solid.Thicken(theSurface, theDepth, theNormalOverride));
        theCalls.Add(new P7Call(theName, theTest, P7Op.MakeOffset, [theSurface.Wrapped],
                                new P7Args(anAdjusted, 1.0e-5, 2, Inter: true, Thickening: true, RemoveIntEdges: true),
                                theClean ? P7Post.Clean : P7Post.None, aResult, anError));
    }

    /// <summary>The Thicken operation: Solid.Thicken of each face along its own normal (both ways if asked), then the clean.</summary>
    private static void ThickenOperation(List<P7Call> theCalls, string theTest, string theName, IEnumerable<B.Face> theFaces, double theAmount,
                                         bool theBoth = false)
    {
        int k = 0;
        foreach (B.Face aFace in theFaces)
        {
            var aNormal = aFace.NormalAt();
            foreach (int aSign in theBoth ? new[] { 1, -1 } : [1])
            {
                // with both ways the two solids are fused before the clean (a Boolean, unit 6M's)
                Thicken(theCalls, theTest, $"{theName} face {k}{(theBoth ? $" sign {aSign}" : "")}", aFace, theAmount, aNormal * aSign, !theBoth);
            }
            k++;
        }
    }

    /// <summary>Shape2D.Offset (OffsetCore): Initialize(tolerance 1e-6, Skin, intersection, Intersection join, no thickening, removing the internal edges).</summary>
    private static void OffsetCore(List<P7Call> theCalls, string theTest, string theName, B.Shape2D theShape, double theAmount)
    {
        (B.Shape? aResult, string? anError) = Own(() => theShape is B.Face f ? f.Offset(theAmount) : ((B.Shell)theShape).Offset(theAmount));
        theCalls.Add(new P7Call(theName, theTest, P7Op.MakeOffset, [theShape.Wrapped],
                                new P7Args(theAmount, B.Constants.Tolerance, 2, Inter: true, Thickening: false, RemoveIntEdges: true),
                                P7Post.None, aResult, anError));
    }

    /// <summary>OneD.OffsetTopodsFace: the three-argument Initialize (no test calls it; kept for its binding).</summary>
    private static void OffsetTopodsFace(List<P7Call> theCalls, string theName, B.Face theFace, double theAmount)
    {
        (B.Shape? aResult, string? anError) = Own(() => B.Shape.Cast(B.OneD.OffsetTopodsFace(N.TopoDS.Face(theFace.Wrapped), theAmount)));
        theCalls.Add(new P7Call(theName, "OneD.OffsetTopodsFace", P7Op.MakeOffset, [theFace.Wrapped],
                                new P7Args(theAmount, B.Constants.Tolerance, ThreeArgs: true), P7Post.None, aResult, anError));
    }

    /// <summary>Solid.Draft: DraftAngle's Add of each face (the plane's normal, the angle, the plane, Flag on), stopping at a refusal, then Build.</summary>
    private static void Draft(List<P7Call> theCalls, string theTest, string theName, B.Solid theSolid, IReadOnlyList<B.Face> theFaces,
                              B.Plane theNeutral, double theAngle, bool theOperation = false)
    {
        (B.Shape? aResult, string? anError) = Own(() => theOperation ? Ops.Draft(theFaces, theNeutral, theAngle) : theSolid.Draft(theFaces, theNeutral, theAngle));
        theCalls.Add(new P7Call(theName, theTest, P7Op.Draft, [theSolid.Wrapped],
                                new P7Args(Faces: FaceIndices(theSolid, theFaces), Angles: [.. theFaces.Select(_ => theAngle * DegToRad)],
                                           Dir: [theNeutral.ZDir.X, theNeutral.ZDir.Y, theNeutral.ZDir.Z],
                                           PlnLoc: [theNeutral.Origin.X, theNeutral.Origin.Y, theNeutral.Origin.Z]),
                                P7Post.None, aResult, anError));
    }

    /// <summary>
    /// Solid.ExtrudeTaper's draft: a copy of the profile extruded (BRepPrimAPI_MakePrism), its sides (generated from the
    /// edges of each wire, the outer first) drafted about the profile's plane through its centre, the holes the other way
    /// unless flipped; made only along the profile's normal with planar, cylindrical or conical sides. The loft
    /// fallback (MakeLoft, Offset2D, a cut) is phase 6's.
    /// </summary>
    private static void ExtrudeTaper(List<P7Call> theCalls, string theTest, string theName, B.Face theProfile, B.Vector theDirection, double theTaper,
                                     bool theFlipInner = true)
    {
        (B.Shape? aResult, string? anError) = Own(() => B.Solid.ExtrudeTaper(theProfile, theDirection, theTaper, theFlipInner));
        var aPull = theDirection.Normalized();
        using var aCopier = new N.BRepBuilderAPI_Copy(theProfile.Wrapped, false, false);
        var aProfile = new B.Face(N.TopoDS.Face(aCopier.Shape()));
        List<B.Wire> aWires = [aProfile.OuterWire(), .. aProfile.InnerWires()];
        using var aVec = new N.gp_Vec(theDirection.X, theDirection.Y, theDirection.Z);
        using var aPrismBuilder = new N.BRepPrimAPI_MakePrism(aProfile.Wrapped, aVec);
        N.TopoDS_Shape aPrism = aPrismBuilder.Shape();
        using var aFaceMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aPrism, N.TopAbs_ShapeEnum.TopAbs_FACE, aFaceMap);
        var aFaces = new List<int>();
        var anAngles = new List<double>();
        var aTypes = new List<B.GeomType>();
        for (int i = 0; i < aWires.Count; i++)
        {
            double anAngle = i > 0 && !theFlipInner ? -theTaper : theTaper;
            foreach (B.Edge anEdge in aWires[i].Edges())
            {
                using N.TopTools_ListOfShape aGenerated = aPrismBuilder.Generated(anEdge.Wrapped);
                foreach (N.TopoDS_Shape aSide in aGenerated)
                {
                    aFaces.Add(aFaceMap.FindIndex(aSide));
                    anAngles.Add(anAngle * DegToRad);
                    aTypes.Add(new B.Face(N.TopoDS.Face(aSide)).GeomType);
                }
            }
        }
        bool isAlongNormal = Math.Abs(aPull.Dot(aProfile.NormalAt())) > 1 - B.Constants.Tolerance;
        if (!isAlongNormal || !aTypes.All(t => t is B.GeomType.Plane or B.GeomType.Cylinder or B.GeomType.Cone))
        {
            return; // lofted: no phase 7 call
        }
        B.Vector aCenter = aProfile.Center();
        theCalls.Add(new P7Call(theName, theTest, P7Op.Draft, [aPrism],
                                new P7Args(Faces: [.. aFaces], Angles: [.. anAngles], Dir: [aPull.X, aPull.Y, aPull.Z],
                                           PlnLoc: [aCenter.X, aCenter.Y, aCenter.Z]),
                                P7Post.Validity, aResult, anError));
    }

    /// <summary>
    /// Shape3D.DPrism: for each profile face in turn, BRepFeat_MakeDPrism(shape, face, basis or a null face, taper, fuse or
    /// cut, no modify), then Perform up to the face, through all, or by the depth; the result feeds the next face.
    /// </summary>
    private static void DPrism(List<P7Call> theCalls, string theTest, string theName, B.Shape3D theShape, B.Face? theBasis,
                               IReadOnlyList<B.Face> theFaces, double? theDepth = null, double theTaper = 0, B.Face? theUpToFace = null,
                               bool theThruAll = true, bool theAdditive = true)
    {
        (B.Shape? aResult, string? anError) = Own(() => theShape.DPrism(theBasis, theFaces.Cast<B.Shape2D>().ToList(), theDepth, theTaper, theUpToFace,
                                                                        theThruAll, theAdditive));
        int aMode = theUpToFace is not null ? 2 : (theThruAll || theDepth is null) ? 0 : 1;
        N.TopoDS_Shape[] anOps = [theShape.Wrapped, .. theFaces.Select(f => f.Wrapped),
                                  .. theBasis is null ? Array.Empty<N.TopoDS_Shape>() : [theBasis.Wrapped],
                                  .. theUpToFace is null ? Array.Empty<N.TopoDS_Shape>() : [theUpToFace.Wrapped]];
        theCalls.Add(new P7Call(theName, theTest, P7Op.DPrism, anOps,
                                new P7Args(theDepth ?? 0, Mode: aMode, Taper: theTaper * DegToRad, Fuse: theAdditive ? 1 : 0,
                                           HasBasis: theBasis is not null, NbProfiles: theFaces.Count),
                                P7Post.None, aResult, anError));
    }

    private static readonly MethodInfo s_splitEdgeAtVertices =
        typeof(B.Shape2D).GetMethod("SplitEdgeAtVertices", BindingFlags.NonPublic | BindingFlags.Static)
        ?? throw new InvalidOperationException("Shape2D.SplitEdgeAtVertices not found");

    /// <summary>
    /// Shape2D.SplitByPerimeter: the perimeter's edges split where they cross the faces' seams (build123d.NET's own
    /// SplitEdgeAtVertices, called by reflection), then BRepFeat_SplitShape(shape), Add(edges), Build, Left and Right.
    /// </summary>
    private static void SplitByPerimeter(List<P7Call> theCalls, string theTest, string theName, B.Shape2D theShape, B.Shape1D thePerimeter)
    {
        (B.Shape? aResult, string? anError) = Own(() =>
        {
            var (anInside, anOutside) = theShape.SplitByPerimeter(thePerimeter);
            return new B.Compound([.. anInside, .. anOutside]);
        });
        var aSeams = theShape.Faces().SelectMany(f => f.Seams).ToList();
        var anEdges = new List<N.TopoDS_Shape>();
        foreach (B.Edge anEdge in thePerimeter.Edges())
        {
            if (anEdge.IsEmpty)
            {
                continue;
            }
            var aSeamVertices = new List<B.Vertex>();
            foreach (B.Edge aSeam in aSeams)
            {
                var aCrossing = anEdge.Intersect(aSeam);
                if (aCrossing is null || aCrossing.Count == 0)
                {
                    continue;
                }
                foreach (B.Vertex aVertex in aCrossing.Vertices())
                {
                    if (anEdge.Vertices().All(ev => aVertex.DistanceTo(ev) > B.Constants.Tolerance)
                        && aSeamVertices.All(existing => aVertex.DistanceTo(existing) > B.Constants.Tolerance))
                    {
                        aSeamVertices.Add(aVertex);
                    }
                }
            }
            foreach (B.Edge aPiece in (List<B.Edge>)s_splitEdgeAtVertices.Invoke(null, [anEdge, aSeamVertices])!)
            {
                anEdges.Add(aPiece.Wrapped);
            }
        }
        theCalls.Add(new P7Call(theName, theTest, P7Op.Split, [theShape.Wrapped, .. anEdges], new P7Args(), P7Post.SewSides, aResult, anError));
    }

    /// <summary>Shape.ToSplines: BSplineRestriction(tolerance, tolerance, degree, 1 segment, C0, C0, degree priority, rational unless NURBS, the default parameters).</summary>
    private static void ToSplines(List<P7Call> theCalls, string theTest, string theName, B.Shape theShape, int theDegree = 3, double theTolerance = 1e-3,
                                  bool theNurbs = false)
    {
        (B.Shape? aResult, string? anError) = Own(() => theShape.ToSplines(theDegree, theTolerance, theNurbs));
        theCalls.Add(new P7Call(theName, theTest, P7Op.ToSplines, [theShape.Wrapped],
                                new P7Args(Tol: theTolerance, Degree: theDegree, Nurbs: theNurbs), P7Post.None, aResult, anError));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The tests' scenarios

    private static B.Solid Block() => B.Solid.MakeBox(1, 1, 1, new B.Plane((-0.5, -0.5, 0)));

    private static B.Face QuarterCylinder(double theRadius = 5.0) =>
        B.Face.Extrude(B.Edge.MakeCircle(theRadius, B.Plane.XY, 0, 90), (0, 0, 10));

    private static B.Vector[][] WavePoints() =>
        [.. Enumerable.Range(0, 11).Select(y => Enumerable.Range(0, 11)
            .Select(x => new B.Vector(x, y, Math.Cos(Math.PI * x / 10) + Math.Sin(Math.PI * y / 10))).ToArray())];

    private static B.Solid ObjBox(double l, double w, double h) => B.Solid.MakeBox(l, w, h, new B.Plane((-l / 2, -w / 2, -h / 2)));

    private static B.Face ObjCircle(double r) => new(B.Wire.MakeCircle(r));

    private static IEnumerable<(string, Action<List<P7Call>, string>)> Scenarios()
    {
        // Shape3DTests.Hollow
        yield return ("Shape3DTests.Hollow", (c, t) =>
        {
            Hollow(c, t, "box inward", B.Solid.MakeBox(1, 1, 1), [], -0.1);
            var aBox = B.Solid.MakeBox(1, 1, 1);
            Hollow(c, t, "box open top and bottom", aBox, aBox.Faces().FilterBy(B.Axis.Z), 0.1, theKind: B.Kind.Intersection);
            Hollow(c, t, "box outward", B.Solid.MakeBox(1, 1, 1), [], 0.1, theKind: B.Kind.Intersection);
        });
        // Shape3DTests.DPrism
        yield return ("Shape3DTests.DPrism", (c, t) =>
        {
            var f = B.Face.MakeRect(0.5, 0.5);
            DPrism(c, t, "face through all", Block(), null, [f], theAdditive: false);
            DPrism(c, t, "face by depth", Block(), null, [f], 0.5, theThruAll: false, theAdditive: false);
            var aLimit = B.Face.MakeRect(1, 1, new B.Plane((0, 0, 0.5)));
            DPrism(c, t, "face up to a face", Block(), null, [f], theUpToFace: aLimit, theThruAll: false, theAdditive: false);
            var w = B.Face.MakeRect(0.5, 0.5).OuterWire();
            DPrism(c, t, "wire through all", Block(), null, [new B.Face(w)], theAdditive: false);
        });
        // SolidTests: Offset3D, the offset orientation tests, ExtrudeTaper, SolidDraftTests
        yield return ("SolidTests.OffsetOrientationTests", (c, t) =>
        {
            var aCone = B.Solid.MakeCone(22.5, 33.75, 40);
            Offset3D(c, t, "cup", aCone, aCone.Faces().SortBy(B.Axis.Z)[^1..], 3, B.Kind.Intersection);
            foreach ((double anAmount, B.Kind aKind) in new[] { (2.0, B.Kind.Arc), (2.0, B.Kind.Intersection), (-2.0, B.Kind.Arc), (-2.0, B.Kind.Intersection) })
            {
                var aBox = B.Solid.MakeBox(30, 20, 10);
                Offset3D(c, t, $"box {anAmount} {aKind}", aBox, aBox.Faces().SortBy(B.Axis.Z)[^1..], anAmount, aKind);
            }
        });
        yield return ("SolidTests.ExtrudeTaper", (c, t) =>
        {
            foreach (double z in new[] { 2.0, -2.0 })
            {
                foreach (double aTaper in new[] { 10.0, -10.0 })
                {
                    foreach (bool isFlipped in new[] { false, true })
                    {
                        var aRect = B.Face.MakeRect(1, 1);
                        ExtrudeTaper(c, t, $"rect z {z} taper {aTaper} flipped {isFlipped}", isFlipped ? (B.Face)(-aRect) : aRect, new B.Vector(0, 0, z), aTaper);
                    }
                }
            }
            {
                var aCross = (new B.Rectangle(5.3, 5.3 / 6) + new B.Rectangle(5.3 / 6, 5.3)).SingleFace();
                var anInner = new B.ShapeList<B.Vertex>(aCross.Vertices()).SortByDistance((0, 0, 0))[..4];
                ExtrudeTaper(c, t, "recess (falls back to a loft)", aCross.Fillet2D(5.3 / 3, anInner), (0, 0, -3.27), 20);
            }
            var aRectHole = B.Face.MakeRect(1, 1).MakeHoles([B.Wire.MakeCircle(0.25)]);
            ExtrudeTaper(c, t, "with hole", aRectHole, (0, 0, 0.5), 10);
            ExtrudeTaper(c, t, "with hole flipped", aRectHole, (0, 0, 1), 10, true);
            ExtrudeTaper(c, t, "with hole not flipped", aRectHole, (0, 0, 1), 10, false);
            ExtrudeTaper(c, t, "hole through wall (falls back)", B.Face.MakeRect(20, 12).MakeHoles([B.Wire.MakeCircle(3)]), (0, 0, 5), 20);
            {
                var aRectHole2 = B.Face.MakeRect(2, 1).MakeHoles([B.Wire.MakeCircle(0.25)]);
                var anOblique = aRectHole2.Moved(new B.Location((0, 0, 0), (1, 0.1, 0), 77));
                ExtrudeTaper(c, t, "oblique, upright", aRectHole2, (0, 0, 1), 5);
                ExtrudeTaper(c, t, "oblique", anOblique, anOblique.NormalAt(), 5);
            }
            {
                B.Vector[] aPoints = [(0, 0), (0, -20), (-10, -21), (-11, 0)];
                int k = 0;
                foreach (B.Vector[] anOutline in new[] { aPoints, aPoints.Reverse().ToArray() })
                {
                    var aProfile = new B.Face(B.Wire.MakePolygon(anOutline));
                    foreach (B.Vector aDirection in new B.Vector[] { (0, 0, 5), (0, 0, -5) })
                    {
                        ExtrudeTaper(c, t, $"follows direction {k++}", aProfile, aDirection, 1);
                    }
                }
            }
            ExtrudeTaper(c, t, "keeps planar sides", B.Face.MakeRect(10, 10), (0, 0, 5), -10);
            {
                var aBar = B.Face.MakeRect(100, 1);
                var aStar = (aBar + aBar.Rotate(B.Axis.Z, 60) + aBar.Rotate(B.Axis.Z, 120)).SingleFace();
                ExtrudeTaper(c, t, "star", aStar, (0, 0, -2), -15);
            }
            {
                var aCross = (B.Face.MakeRect(10, 1) + B.Face.MakeRect(1, 10)).SingleFace();
                ExtrudeTaper(c, t, "collapsing cross 10", aCross, (0, 0, 2), 10);
                ExtrudeTaper(c, t, "collapsing cross 15", aCross, (0, 0, 2), 15);
            }
        });
        yield return ("SolidTests.SolidDraftTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(10, 10, 10, new B.Plane((-5, -5, -5)));
            var aSides = aBox.Faces().FilterBy(B.Axis.Z, reverse: true);
            var aBottom = aBox.Faces().SortBy(B.Axis.Z)[0];
            Draft(c, t, "successful draft", aBox, aSides, new B.Plane(aBottom), 5);
        });
        // OperationsGenericTests' offsets of solids
        yield return ("OperationsGenericTests.Offset", (c, t) =>
        {
            OffsetOperation(c, t, "BoxOffset inner", new B.Box(10, 10, 10), -1, theKind: B.Kind.Intersection);
            OffsetOperation(c, t, "BoxOffset 4 inward", new B.Box(4, 4, 4), -1, theKind: B.Kind.Intersection);
            OffsetOperation(c, t, "BoxOffset 4 outward", new B.Box(4, 4, 4), 1, theKind: B.Kind.Intersection);
            var aBox = new B.Box(10, 10, 10);
            OffsetOperation(c, t, "BoxOffsetWithOpening >>", aBox, -1, aBox.Faces() >> B.Axis.Z, B.Kind.Intersection);
            OffsetOperation(c, t, "BoxOffsetWithOpening top", aBox, -1, [aBox.Faces().SortBy(B.Axis.Z)[^1]], B.Kind.Intersection);
            OffsetOperation(c, t, "OffsetABareSolid", B.Solid.MakeBox(10, 10, 10), -1, []);
            var aCup = Ops.Loft([new B.Circle(35), new B.Pos(0, 0, 50) * new B.Circle(37.62)]);
            OffsetOperation(c, t, "OffsetFailure", aCup, -2, [aCup.Faces().SortBy(B.Axis.Z)[^1]]);
        });
        // OperationsPartTests: draft and thicken
        yield return ("OperationsPartTests.Draft", (c, t) =>
        {
            var aBox = new B.Box(10, 10, 10).SingleSolid();
            var aSides = aBox.Faces().FilterBy(B.Axis.Z, reverse: true);
            var aNeutral = new B.Plane(aBox.Faces().SortBy(B.Axis.Z)[0]);
            Draft(c, t, "SuccessfulDraft", aBox, aSides, aNeutral, 5, true);
            var aBox2 = new B.Box(10, 10, 10);
            Draft(c, t, "SuccessfulDraft about XY offset", aBox2.SingleSolid(), aBox2.Faces().FilterBy(B.Axis.Z, reverse: true), B.Plane.XY.Offset(-5), 5, true);
            Draft(c, t, "BadDraftFaces", aBox, [aBox.Faces().SortBy(B.Axis.Z)[0]], aNeutral, 10, true);
        });
        yield return ("OperationsPartTests.Thicken", (c, t) =>
        {
            ThickenOperation(c, t, "ThickenFaces rounded rectangle", new B.RectangleRounded(10, 10, 1).Faces(), 1);
            ThickenOperation(c, t, "ThickenFaces sphere", [new B.Sphere(1).Faces()[0]], 0.1);
            var aWire = (B.Shape1D)(new B.JernArc((0, -2), (-1, 0), 1, -180) + new B.JernArc((0, 0), (1, 0), 2, -90));
            var aSurface = Ops.Sweep((aWire ^ 0) * new B.RadiusArc((0, 0), (0, -1), 1), aWire);
            ThickenOperation(c, t, "ThickenSweptSurface", aSurface.Faces(), 0.4);
            ThickenOperation(c, t, "ThickenSweptSurface both", aSurface.Faces(), 0.4, true);
            ThickenOperation(c, t, "ThickenBoth", [B.Face.MakeRect(2, 2)], 1, true);
        });
        // FaceTests and ShellTests: thicken, offsets, split by perimeter
        yield return ("FaceTests.Thicken", (c, t) =>
        {
            Thicken(c, t, "wave surface", B.Face.MakeSurfaceFromArrayOfPoints(WavePoints()), 1);
            Thicken(c, t, "square, normal down", B.Face.MakeRect(10, 10), 1, (0, 0, -1));
        });
        yield return ("ShellTests.Thicken", (c, t) =>
            Thicken(c, t, "extruded rectangle", B.Shell.Extrude(B.Wire.MakeRect(10, 5), new B.Vector(0, 0, 3)), 1));
        yield return ("FaceTests.SurfaceOffsetTests", (c, t) =>
        {
            OffsetCore(c, t, "plane +2", B.Face.MakeRect(10, 10), 2);
            OffsetCore(c, t, "plane -2", B.Face.MakeRect(10, 10), -2);
            OffsetCore(c, t, "quarter cylinder -1", QuarterCylinder(), -1);
            OffsetCore(c, t, "quarter cylinder +1", QuarterCylinder(), 1);
            OffsetCore(c, t, "sphere +1", B.Solid.MakeSphere(5).Faces()[0], 1);
            OffsetCore(c, t, "sphere -1", B.Solid.MakeSphere(5).Faces()[0], -1);
            OffsetCore(c, t, "box shell", B.Solid.MakeBox(10, 10, 10).Shells()[0], 0.5);
            OffsetCore(c, t, "collapse", QuarterCylinder(), 5.0);
            OffsetCore(c, t, "labelled plate", B.Face.MakeRect(10, 10).WithLabel("plate"), 1);
        });
        yield return ("FaceTests.SplitByPerimeter", (c, t) =>
        {
            var aSphere = B.Solid.MakeSphere(10).Rotate(B.Axis.Z, 90);
            var aTarget0 = aSphere.Faces()[0];
            var aCircle = B.Plane.YZ.Offset(15) * ObjCircle(5);
            var aCircleProjected = aCircle.ProjectToShape(aSphere, (-1, 0, 0))[0];
            SplitByPerimeter(c, t, "spherical cap", aTarget0, aCircleProjected.SingleEdge());
            var aRing = new B.Pos(z: 15) * (ObjCircle(5) - ObjCircle(3)).SingleFace();
            var aRingProjected = (B.Face)aRing.ProjectToShape(aSphere, (0, 0, -1))[0];
            SplitByPerimeter(c, t, "ring of a sphere", aTarget0, aRingProjected.OuterWire());
            var aTarget2 = ObjBox(1, 10, 10).SingleShell();
            var aSquare = B.Face.MakeRect(3, 3, new B.Plane((12, 0, 0), zDir: (1, 0, 0)));
            var aSquareProjected = (B.Face)aSquare.ProjectToShape(aTarget2, (-1, 0, 0))[0];
            SplitByPerimeter(c, t, "faces of a shell", aTarget2, aSquareProjected.OuterWire());
            var aSphere3 = B.Solid.MakeSphere(10);
            var aTarget3 = aSphere3.SingleFace();
            var aCircle3 = B.Plane.YZ.Offset(15) * ObjCircle(5);
            var aProjectedWire = new B.Wire(aCircle3.ProjectToShape(aSphere3, (-1, 0, 0))[0].Edges().GroupBy(B.Axis.X)[0]);
            var aPerimeter = B.Edge.MakeCircle(aProjectedWire.Edges()[0].Radius,
                                               new B.Plane(aProjectedWire.Edges()[0].ArcCenter, zDir: (1, 0, 0)));
            SplitByPerimeter(c, t, "sphere across the seam", aTarget3, aPerimeter);
        });
        yield return ("EmptyShapeTests.SplitByPerimeterOutsideTheFace", (c, t) =>
            SplitByPerimeter(c, t, "perimeter elsewhere", B.Face.MakeRect(4, 4), new B.Pos(10, 0, 0) * B.Wire.MakeRect(1, 1)));
        // TopologyReferenceTests' features
        yield return ("TopologyReferenceTests", (c, t) =>
        {
            var aBlock = B.Solid.MakeBox(10, 10, 10, new B.Plane(new B.Vector(0, 0, 0)));
            Hollow(c, t, "hollow", aBlock, [aBlock.Faces().SortBy(B.Axis.Z).Last()], -1);
            Offset3D(c, t, "offset_3d", aBlock, [], 1);
            ExtrudeTaper(c, t, "extrude_taper", B.Face.MakeRect(4, 4), (0, 0, 2), 10);
            Thicken(c, t, "thicken_face", B.Face.MakeRect(3, 3), 0.5);
        });
        // MalformedInputTests and InputsUnchangedTests
        yield return ("MalformedInputTests", (c, t) =>
        {
            OffsetOperation(c, t, "offset through itself", new B.Box(1, 1, 1), -2);
            Thicken(c, t, "thicken by nothing", B.Face.MakeRect(1, 1), 0);
        });
        yield return ("InputsUnchangedTests", (c, t) =>
        {
            var aBox = new B.Box(10, 10, 10);
            var aSolid = aBox.Solids()[0];
            Hollow(c, t, "hollow", aSolid, aBox.Faces().SortBy(B.Axis.Z)[^1..], 1);
            Draft(c, t, "draft", aBox.SingleSolid(), aBox.Faces().FilterBy(B.Axis.Z, reverse: true), B.Plane.XY.Offset(-5), 5, true);
            OffsetOperation(c, t, "offset", aBox, 1);
            OffsetOperation(c, t, "shell", aBox, -1, [aBox.Faces().SortBy(B.Axis.Z).Last()]);
            ThickenOperation(c, t, "thicken", B.Face.MakeRect(10, 6).Faces(), 1);
        });
        // OneD.OffsetTopodsFace and Shape.ToSplines aren't called by the tests on a shape (ShapeTests' ToSplines is on an
        // empty one, which throws before OCCT): their bindings are exercised on the reference primitives
        yield return ("OneD.OffsetTopodsFace", (c, _) =>
        {
            OffsetTopodsFace(c, "plane", B.Face.MakeRect(10, 10), 2);
            OffsetTopodsFace(c, "quarter cylinder", QuarterCylinder(), 1);
        });
        yield return ("Shape.ToSplines (reference primitives)", (c, t) =>
        {
            ToSplines(c, t, "box", B.Solid.MakeBox(1, 2, 3));
            ToSplines(c, t, "cylinder", B.Solid.MakeCylinder(2, 5));
            ToSplines(c, t, "cone", B.Solid.MakeCone(3, 1, 4));
            ToSplines(c, t, "sphere", B.Solid.MakeSphere(2.5));
            ToSplines(c, t, "torus", B.Solid.MakeTorus(5, 1));
            ToSplines(c, t, "torus NURBS degree 5", B.Solid.MakeTorus(5, 1), 5, 1e-4, true);
            ToSplines(c, t, "ellipse face", new B.Face(new B.Wire(B.Edge.MakeEllipse(8, 4))));
        });
    }

    /// <summary>build123d.NET's operation behind a call.</summary>
    internal static string Label(P7Call theCall) => theCall.Op switch
    {
        P7Op.ThickSolid => theCall.Post is P7Post.Offset3D or P7Post.OffsetOperation ? "Offset3D" : "Hollow",
        P7Op.MakeOffset => theCall.Args.ThreeArgs ? "OffsetTopodsFace" : theCall.Args.Thickening ? "Thicken" : "OffsetCore",
        P7Op.Draft => theCall.Post == P7Post.Validity ? "ExtrudeTaper" : "Draft",
        P7Op.DPrism => "DPrism",
        P7Op.Split => "SplitByPerimeter",
        _ => "ToSplines",
    };

    // ---------------------------------------------------------------------------------------------------------------
    // The operands as BRep text

    /// <summary>The operands as native OCCT writes them: one compound, whose children are read back in order.</summary>
    internal static string OperandsText(IEnumerable<N.TopoDS_Shape> theShapes)
    {
        using var aBuilder = new N.BRep_Builder();
        var aCompound = new N.TopoDS_Compound();
        aBuilder.MakeCompound(aCompound);
        foreach (N.TopoDS_Shape aShape in theShapes)
        {
            aBuilder.Add(aCompound, aShape);
        }
        return NativeText(aCompound);
    }

    internal static string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"p7-{Guid.NewGuid():N}.brep");
        try
        {
            if (!N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3))
            {
                throw new InvalidOperationException("native OCCT can't write the shape");
            }
            return File.ReadAllText(aFile);
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    internal static List<N.TopoDS_Shape> NativeOperands(string theText)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"p7-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, theText);
        try
        {
            var aC = new N.TopoDS_Shape();
            using var aBuilder = new N.BRep_Builder();
            if (!N.BRepTools.Read(aC, aFile, aBuilder))
            {
                throw new InvalidOperationException("native OCCT can't read the operands");
            }
            var aList = new List<N.TopoDS_Shape>();
            for (var anIt = new N.TopoDS_Iterator(aC); anIt.More(); anIt.Next())
            {
                aList.Add(anIt.Value());
            }
            return aList;
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    /// <summary>The operands read by the port (frozen, as a caller's shapes).</summary>
    internal static List<TopoDS_Shape> PortOperands(string theText)
    {
        BRepTools.Read(out TopoDS_Shape aC, new StringReader(theText));
        var aList = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(aC); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        return aList;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The runners

    /// <summary>The type of a native exception (OCCT's), as the oracles name it.</summary>
    internal static string ErrorOf(Exception e) => e switch
    {
        N.OcctException o => o.Message.IndexOf(':') is int c and > 0 ? o.Message[..c] : o.Message,
        Standard_Failure f => f.GetType().Name,
        NullReferenceException => "OSD_Exception_ACCESS_VIOLATION",
        _ => e.GetType().Name,
    };

    private static string Index(N.TopTools_IndexedMapOfShape theMap, N.TopoDS_Shape theS)
    {
        int anI = theMap.FindIndex(theS);
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation() == N.TopAbs_Orientation.TopAbs_REVERSED ? "r" : "");
    }

    private static string Index(NCollection_IndexedMap<TopoDS_Shape> theMap, in TopoDS_Shape theS)
    {
        int anI = theMap.FindIndex(theS);
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation() == TopAbs_Orientation.TopAbs_REVERSED ? "r" : "");
    }

    private static string Sorted(IEnumerable<string> theItems) => string.Join(' ', theItems.Order(StringComparer.Ordinal));

    // The history: for each vertex, edge and face of the operands, the sorted indices (in the result's map) of Generated
    // and Modified, and IsDeleted
    private static string NativeHistory(List<N.TopoDS_Shape> theOps, N.TopoDS_Shape theResult, Func<N.TopoDS_Shape, N.TopTools_ListOfShape> theGen,
                                        Func<N.TopoDS_Shape, N.TopTools_ListOfShape> theMod, Func<N.TopoDS_Shape, bool> theDel)
    {
        using var anIn = new N.TopTools_IndexedMapOfShape();
        foreach (N.TopoDS_Shape anOp in theOps)
        {
            N.TopExp.MapShapes(anOp, anIn);
        }
        using var aRes = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theResult, aRes);
        var aSb = new StringBuilder();
        for (int i = 1; i <= anIn.Extent(); i++)
        {
            using N.TopoDS_Shape aS = anIn.FindKey(i);
            if (aS.ShapeType() is not (N.TopAbs_ShapeEnum.TopAbs_VERTEX or N.TopAbs_ShapeEnum.TopAbs_EDGE or N.TopAbs_ShapeEnum.TopAbs_FACE))
            {
                continue;
            }
            using N.TopTools_ListOfShape aG = theGen(aS);
            using N.TopTools_ListOfShape aM = theMod(aS);
            aSb.Append(i).Append(":G[").Append(Sorted(aG.Select(x => Index(aRes, x)))).Append("]M[")
               .Append(Sorted(aM.Select(x => Index(aRes, x)))).Append(']').Append(theDel(aS) ? "D" : "").Append('\n');
        }
        return aSb.ToString();
    }

    private static string PortHistory(List<TopoDS_Shape> theOps, in TopoDS_Shape theResult, Func<TopoDS_Shape, IEnumerable<TopoDS_Shape>> theGen,
                                      Func<TopoDS_Shape, IEnumerable<TopoDS_Shape>> theMod, Func<TopoDS_Shape, bool> theDel)
    {
        var anIn = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        foreach (TopoDS_Shape anOp in theOps)
        {
            TopExp.MapShapes(anOp, anIn);
        }
        var aRes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theResult, aRes);
        var aSb = new StringBuilder();
        for (int i = 1; i <= anIn.Extent(); i++)
        {
            TopoDS_Shape aS = anIn[i];
            if (aS.ShapeType() is not (TopAbs_ShapeEnum.TopAbs_VERTEX or TopAbs_ShapeEnum.TopAbs_EDGE or TopAbs_ShapeEnum.TopAbs_FACE))
            {
                continue;
            }
            aSb.Append(i).Append(":G[").Append(Sorted(theGen(aS).Select(x => Index(aRes, x)))).Append("]M[")
               .Append(Sorted(theMod(aS).Select(x => Index(aRes, x)))).Append(']').Append(theDel(aS) ? "D" : "").Append('\n');
        }
        return aSb.ToString();
    }

    /// <summary>One call natively, on operands read from the text; with <paramref name="theHistory"/>, its history too.</summary>
    internal static P7Native RunNative(P7Call theCall, List<N.TopoDS_Shape> theOps, bool theHistory = true)
    {
        P7Args a = theCall.Args;
        using var aFaces = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theOps[0], N.TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
        try
        {
            switch (theCall.Op)
            {
                case P7Op.ThickSolid:
                {
                    using var aMT = new N.BRepOffsetAPI_MakeThickSolid();
                    using var aClosing = new N.TopTools_ListOfShape();
                    foreach (int i in a.Faces ?? [])
                    {
                        aClosing.Append(aFaces.FindKey(i));
                    }
                    aMT.MakeThickSolidByJoin(theOps[0], aClosing, a.Value, a.Tol, N.BRepOffset_Mode.BRepOffset_Skin, a.Inter, false,
                                             (N.GeomAbs_JoinType)a.Join, a.RemoveIntEdges);
                    aMT.Build();
                    string aStatus = $"done={aMT.IsDone()} error={(int)aMT.MakeOffset().Error()}";
                    N.TopoDS_Shape aRes = aMT.Shape();
                    return new P7Native(null, aStatus, aRes,
                                        theHistory && !aRes.IsNull() ? NativeHistory(theOps, aRes, aMT.Generated, aMT.Modified, aMT.IsDeleted) : "");
                }
                case P7Op.MakeOffset:
                {
                    using var aMO = new N.BRepOffset_MakeOffset();
                    if (a.ThreeArgs)
                    {
                        aMO.Initialize(theOps[0], a.Value, a.Tol);
                    }
                    else
                    {
                        aMO.Initialize(theOps[0], a.Value, a.Tol, N.BRepOffset_Mode.BRepOffset_Skin, a.Inter, false, (N.GeomAbs_JoinType)a.Join,
                                       a.Thickening, a.RemoveIntEdges);
                    }
                    aMO.MakeOffsetShape();
                    string aStatus = $"done={aMO.IsDone()} error={(int)aMO.Error()}";
                    N.TopoDS_Shape aRes = aMO.Shape();
                    return new P7Native(null, aStatus, aRes,
                                        theHistory && !aRes.IsNull() ? NativeHistory(theOps, aRes, aMO.Generated, aMO.Modified, aMO.IsDeleted) : "");
                }
                case P7Op.Draft:
                {
                    using var anIn = new N.TopTools_IndexedMapOfShape();
                    N.TopExp.MapShapes(theOps[0], anIn);
                    using var aDraft = new N.BRepOffsetAPI_DraftAngle(theOps[0]);
                    using var aDir = new N.gp_Dir(a.Dir![0], a.Dir[1], a.Dir[2]);
                    using var aLoc = new N.gp_Pnt(a.PlnLoc![0], a.PlnLoc[1], a.PlnLoc[2]);
                    using var aPln = new N.gp_Pln(aLoc, aDir);
                    for (int k = 0; k < a.Faces!.Length; k++)
                    {
                        aDraft.Add(N.TopoDS.Face(aFaces.FindKey(a.Faces[k])), aDir, a.Angles![k], aPln, true);
                        if (!aDraft.AddDone())
                        {
                            using N.TopoDS_Shape aBad = aDraft.ProblematicShape();
                            return new P7Native(null, $"add=False status={(int)aDraft.Status()} bad={Index(anIn, aBad)}", null, "");
                        }
                    }
                    aDraft.Build();
                    string aStatus = $"add=True done={aDraft.IsDone()} status={(int)aDraft.Status()}";
                    if (!aDraft.IsDone())
                    {
                        using N.TopoDS_Shape aBad = aDraft.ProblematicShape();
                        return new P7Native(null, aStatus + $" bad={Index(anIn, aBad)}", null, "");
                    }
                    N.TopoDS_Shape aRes = aDraft.Shape();
                    return new P7Native(null, aStatus, aRes,
                                        theHistory ? NativeHistory(theOps, aRes, aDraft.Generated, aDraft.Modified, aDraft.IsDeleted) : "");
                }
                case P7Op.DPrism:
                {
                    using var aNoFace = new N.TopoDS_Face();
                    N.TopoDS_Shape aShape = theOps[0];
                    N.TopoDS_Face aBasis = a.HasBasis ? N.TopoDS.Face(theOps[1 + a.NbProfiles]) : aNoFace;
                    string aStatus = "";
                    string aHistory = "";
                    for (int k = 1; k <= a.NbProfiles; k++)
                    {
                        using var aFeature = new N.BRepFeat_MakeDPrism(aShape, N.TopoDS.Face(theOps[k]), aBasis, a.Taper, a.Fuse, false);
                        switch (a.Mode)
                        {
                            case 2:
                                aFeature.Perform(theOps[^1]);
                                break;
                            case 0:
                                aFeature.PerformThruAll();
                                break;
                            default:
                                aFeature.Perform(a.Value);
                                break;
                        }
                        aStatus += $"done={aFeature.IsDone()} status={(int)aFeature.CurrentStatusError()} ";
                        N.TopoDS_Shape aRes = aFeature.Shape();
                        if (theHistory && a.NbProfiles == 1)
                        {
                            aHistory = NativeHistory(theOps, aRes, aFeature.Generated, aFeature.Modified, aFeature.IsDeleted);
                        }
                        aShape = aRes;
                    }
                    return new P7Native(null, aStatus, aShape, aHistory);
                }
                case P7Op.Split:
                {
                    using var aSplit = new N.BRepFeat_SplitShape(theOps[0]);
                    using var anEdges = new N.TopTools_SequenceOfShape();
                    foreach (N.TopoDS_Shape anEdge in theOps.Skip(1))
                    {
                        anEdges.Append(anEdge);
                    }
                    aSplit.Add(anEdges);
                    aSplit.Build();
                    N.TopoDS_Shape aRes = aSplit.Shape();
                    using N.TopTools_ListOfShape aLefts = aSplit.Left();
                    using N.TopTools_ListOfShape aRights = aSplit.Right();
                    N.TopoDS_Shape[] aLeft = [.. aLefts.Where(s => !s.IsNull())];
                    N.TopoDS_Shape[] aRight = [.. aRights.Where(s => !s.IsNull())];
                    string aHistory = "";
                    if (theHistory)
                    {
                        using var aResMap = new N.TopTools_IndexedMapOfShape();
                        N.TopExp.MapShapes(aRes, aResMap);
                        aHistory = $"left [{Sorted(aLeft.Select(s => Index(aResMap, s)))}] right [{Sorted(aRight.Select(s => Index(aResMap, s)))}]\n"
                                   + NativeHistory(theOps, aRes, aSplit.Generated, aSplit.Modified, aSplit.IsDeleted);
                    }
                    return new P7Native(null, $"done={aSplit.IsDone()}", aRes, aHistory, aLeft, aRight);
                }
                default:
                {
                    using var aParameters = new N.ShapeCustom_RestrictionParameters();
                    N.TopoDS_Shape aRes = N.ShapeCustom.BSplineRestriction(theOps[0], a.Tol, a.Tol, a.Degree, 1, N.GeomAbs_Shape.GeomAbs_C0,
                                                                          N.GeomAbs_Shape.GeomAbs_C0, true, !a.Nurbs, aParameters);
                    return new P7Native(null, "", aRes, "");
                }
            }
        }
        catch (Exception e) when (e is N.OcctException)
        {
            return new P7Native(ErrorOf(e), "", null, "");
        }
    }

    /// <summary>One call in the port, on its (frozen) operands, as <see cref="RunNative"/>.</summary>
    internal static P7Port RunPort(P7Call theCall, List<TopoDS_Shape> theOps, bool theHistory = true)
    {
        P7Args a = theCall.Args;
        var aFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theOps[0], TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
        try
        {
            switch (theCall.Op)
            {
                case P7Op.ThickSolid:
                {
                    var aMT = new BRepOffsetAPI_MakeThickSolid();
                    var aClosing = new NCollection_List<TopoDS_Shape>();
                    foreach (int i in a.Faces ?? [])
                    {
                        aClosing.Append(aFaces[i]);
                    }
                    aMT.MakeThickSolidByJoin(theOps[0], aClosing, a.Value, a.Tol, BRepOffset_Mode.BRepOffset_Skin, a.Inter, false,
                                             (GeomAbs_JoinType)a.Join, a.RemoveIntEdges);
                    aMT.Build();
                    string aStatus = $"done={aMT.IsDone()} error={(int)aMT.MakeOffset().Error()}";
                    TopoDS_Shape aRes = aMT.Shape();
                    return new P7Port(null, aStatus, aRes,
                                      theHistory && !aRes.IsNull() ? PortHistory(theOps, aRes, s => aMT.Generated(s), s => aMT.Modified(s), s => aMT.IsDeleted(s)) : "");
                }
                case P7Op.MakeOffset:
                {
                    var aMO = new BRepOffset_MakeOffset();
                    if (a.ThreeArgs)
                    {
                        aMO.Initialize(theOps[0], a.Value, a.Tol);
                    }
                    else
                    {
                        aMO.Initialize(theOps[0], a.Value, a.Tol, BRepOffset_Mode.BRepOffset_Skin, a.Inter, false, (GeomAbs_JoinType)a.Join,
                                       a.Thickening, a.RemoveIntEdges);
                    }
                    aMO.MakeOffsetShape();
                    string aStatus = $"done={aMO.IsDone()} error={(int)aMO.Error()}";
                    TopoDS_Shape aRes = aMO.Shape();
                    return new P7Port(null, aStatus, aRes,
                                      theHistory && !aRes.IsNull() ? PortHistory(theOps, aRes, s => aMO.Generated(s), s => aMO.Modified(s), s => aMO.IsDeleted(s)) : "");
                }
                case P7Op.Draft:
                {
                    var anIn = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapes(theOps[0], anIn);
                    var aDraft = new BRepOffsetAPI_DraftAngle(theOps[0]);
                    var aDir = new gp_Dir(a.Dir![0], a.Dir[1], a.Dir[2]);
                    var aPln = new gp_Pln(new gp_Pnt(a.PlnLoc![0], a.PlnLoc[1], a.PlnLoc[2]), aDir);
                    for (int k = 0; k < a.Faces!.Length; k++)
                    {
                        aDraft.Add(TopoDS.Face(aFaces[a.Faces[k]]), aDir, a.Angles![k], aPln, true);
                        if (!aDraft.AddDone())
                        {
                            return new P7Port(null, $"add=False status={(int)aDraft.Status()} bad={Index(anIn, aDraft.ProblematicShape())}", null, "");
                        }
                    }
                    aDraft.Build();
                    string aStatus = $"add=True done={aDraft.IsDone()} status={(int)aDraft.Status()}";
                    if (!aDraft.IsDone())
                    {
                        return new P7Port(null, aStatus + $" bad={Index(anIn, aDraft.ProblematicShape())}", null, "");
                    }
                    TopoDS_Shape aRes = aDraft.Shape();
                    return new P7Port(null, aStatus, aRes,
                                      theHistory ? PortHistory(theOps, aRes, s => aDraft.Generated(s), s => aDraft.Modified(s), s => aDraft.IsDeleted(s)) : "");
                }
                case P7Op.DPrism:
                {
                    TopoDS_Shape aShape = theOps[0];
                    TopoDS_Face aBasis = a.HasBasis ? TopoDS.Face(theOps[1 + a.NbProfiles]) : new TopoDS_Face();
                    string aStatus = "";
                    string aHistory = "";
                    for (int k = 1; k <= a.NbProfiles; k++)
                    {
                        var aFeature = new BRepFeat_MakeDPrism(aShape, TopoDS.Face(theOps[k]), aBasis, a.Taper, a.Fuse, false);
                        switch (a.Mode)
                        {
                            case 2:
                                aFeature.Perform(theOps[^1]);
                                break;
                            case 0:
                                aFeature.PerformThruAll();
                                break;
                            default:
                                aFeature.Perform(a.Value);
                                break;
                        }
                        aStatus += $"done={aFeature.IsDone()} status={(int)aFeature.CurrentStatusError()} ";
                        TopoDS_Shape aRes = aFeature.Shape();
                        if (theHistory && a.NbProfiles == 1)
                        {
                            aHistory = PortHistory(theOps, aRes, s => aFeature.Generated(s), s => aFeature.Modified(s), s => aFeature.IsDeleted(s));
                        }
                        aShape = aRes;
                    }
                    return new P7Port(null, aStatus, aShape, aHistory);
                }
                case P7Op.Split:
                {
                    var aSplit = new BRepFeat_SplitShape(theOps[0]);
                    var anEdges = new NCollection_Sequence<TopoDS_Shape>();
                    foreach (TopoDS_Shape anEdge in theOps.Skip(1))
                    {
                        anEdges.Append(anEdge);
                    }
                    aSplit.Add(anEdges);
                    aSplit.Build();
                    TopoDS_Shape aRes = aSplit.Shape();
                    TopoDS_Shape[] aLeft = [.. aSplit.Left().Where(s => !s.IsNull())];
                    TopoDS_Shape[] aRight = [.. aSplit.Right().Where(s => !s.IsNull())];
                    string aHistory = "";
                    if (theHistory)
                    {
                        var aResMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                        TopExp.MapShapes(aRes, aResMap);
                        aHistory = $"left [{Sorted(aLeft.Select(s => Index(aResMap, s)))}] right [{Sorted(aRight.Select(s => Index(aResMap, s)))}]\n"
                                   + PortHistory(theOps, aRes, s => aSplit.Generated(s), s => aSplit.Modified(s), s => aSplit.IsDeleted(s));
                    }
                    return new P7Port(null, $"done={aSplit.IsDone()}", aRes, aHistory, aLeft, aRight);
                }
                default:
                {
                    TopoDS_Shape aRes = ShapeCustom.BSplineRestriction(theOps[0], a.Tol, a.Tol, a.Degree, 1, GeomAbs_Shape.GeomAbs_C0,
                                                                       GeomAbs_Shape.GeomAbs_C0, true, !a.Nurbs, new ShapeCustom_RestrictionParameters());
                    return new P7Port(null, "", aRes, "");
                }
            }
        }
        catch (Exception e) when (e is Standard_Failure or NullReferenceException)
        {
            return new P7Port(ErrorOf(e), "", null, "");
        }
    }
}
#endif
