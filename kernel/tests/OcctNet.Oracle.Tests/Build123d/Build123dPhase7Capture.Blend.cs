// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
#if HAS_ORACLE || HAS_NATIVE
using Build123d;
using Occt;
using B = Build123d;
using W = Build123d.Occt;
using Ops = Build123d.Operations;
#endif

#if HAS_NATIVE
using N = Build123d.Occt;
#endif

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The fillets and chamfers build123d.NET's tests make (unit 7X, its second part): <c>BRepFilletAPI_MakeFillet</c>
/// (Shape3D.Fillet and the fillet operation: <c>Add(r, edge)</c> per selected edge, <c>Shape()</c>, and on a failure
/// the faulty contours' <c>StripeStatus</c> and edges for the message; MaxFillet's binary search, a new maker per
/// probe) and <c>BRepFilletAPI_MakeChamfer</c> (Shape3D.Chamfer and the chamfer operation: <c>Add(d1, d2, edge,
/// face)</c>, the face the given one or the edge's first ancestor). The sketch and wire fillets and chamfers are 2D
/// (MakeFillet2d, phase 5) and don't reach these makers; the argument checks that throw before OCCT make no call.
/// </summary>
internal static partial class Build123dPhase7Capture
{
#if HAS_ORACLE || HAS_NATIVE
    // ---------------------------------------------------------------------------------------------------------------
    // The calls

    // build123d.NET's own result (or its exception's type and message)
    private static (B.Shape?, string?, string?) OwnWithMessage(Func<B.Shape> theRun)
    {
        try
        {
            return (theRun(), null, null);
        }
        catch (Exception e)
        {
            return (null, ErrorOfB(e), e.Message);
        }
    }

    private static int[] EdgeIndices(B.Shape theShape, IEnumerable<B.Edge> theEdges)
    {
        using var aMap = new W.TopTools_IndexedMapOfShape();
        W.TopExp.MapShapes(theShape.Wrapped, W.TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
        return [.. theEdges.Select(e => aMap.FindIndex(e.Wrapped) * (e.Wrapped.Orientation() == W.TopAbs_Orientation.TopAbs_REVERSED ? -1 : 1))];
    }

    // The shape build123d.NET's fillet and chamfer operations rebuild: the selection's first parent
    private static B.Shape TargetOf(IReadOnlyList<B.Edge> theEdges) => theEdges[0].TopoParent ?? throw new InvalidOperationException("no parent");

    /// <summary>Shape3D.Fillet (or the fillet operation): <c>Add(radius, edge)</c> for each selected edge, then <c>Shape()</c>.</summary>
    private static void Fillet(List<W7Call> theCalls, string theTest, string theName, B.Shape3D? theShape, IEnumerable<B.Edge> theEdges,
                               double theRadius)
    {
        List<B.Edge> anEdges = [.. theEdges];
        (B.Shape? aResult, string? anError, string? aMessage) =
            OwnWithMessage(() => theShape is null ? Ops.Fillet(anEdges, theRadius) : theShape.Fillet(theRadius, anEdges));
        B.Shape aTarget = theShape ?? TargetOf(anEdges);
        theCalls.Add(new W7Call(theName, theTest, P7Op.Fillet, [aTarget.Wrapped],
                                new P7Args(theRadius, Edges: EdgeIndices(aTarget, anEdges), Operation: theShape is null),
                                P7Post.None, aResult, anError, aMessage));
    }

    /// <summary>The fillet operation (the shape is the selection's parent).</summary>
    private static void FilletOperation(List<W7Call> theCalls, string theTest, string theName, IEnumerable<B.Edge> theEdges, double theRadius) =>
        Fillet(theCalls, theTest, theName, null, theEdges, theRadius);

    /// <summary>
    /// Shape3D.Chamfer (or the chamfer operation, which turns an angle into the second length first):
    /// <c>Add(length, length2 or length, edge, face)</c>, the face given or the edge's first ancestor, then <c>Shape()</c>.
    /// </summary>
    private static void Chamfer(List<W7Call> theCalls, string theTest, string theName, B.Shape3D? theShape, IEnumerable<B.Edge> theEdges,
                                double theLength, double? theLength2 = null, double? theAngle = null, B.Face? theFace = null)
    {
        List<B.Edge> anEdges = [.. theEdges];
        (B.Shape? aResult, string? anError, string? aMessage) =
            OwnWithMessage(() => theShape is null
                                     ? Ops.Chamfer(anEdges, theLength, theLength2, theAngle, theFace)
                                     : theShape.Chamfer(theLength, theLength2, anEdges, theFace));
        double? aLength2 = theLength2;
        if (theAngle is { } anAngle && anAngle != 0)
        {
            aLength2 = theLength * Math.Tan(anAngle * DegToRad);
        }
        double aDistance2 = aLength2 is { } l2 && l2 != 0 ? l2 : theLength;
        B.Shape aTarget = theShape ?? TargetOf(anEdges);
        int aFace = 0;
        if (theFace is not null)
        {
            using var aMap = new W.TopTools_IndexedMapOfShape();
            W.TopExp.MapShapes(aTarget.Wrapped, W.TopAbs_ShapeEnum.TopAbs_FACE, aMap);
            aFace = aMap.FindIndex(theFace.Wrapped);
        }
        theCalls.Add(new W7Call(theName, theTest, P7Op.Chamfer, [aTarget.Wrapped],
                                new P7Args(theLength, Edges: EdgeIndices(aTarget, anEdges), Value2: aDistance2, Operation: theShape is null,
                                           Faces: [.. anEdges.Select(_ => aFace)]),
                                P7Post.None, aResult, anError, aMessage));
    }

    /// <summary>Shape3D.MaxFillet: the binary search of fillets over (0, twice the bounding box's diagonal).</summary>
    private static void MaxFillet(List<W7Call> theCalls, string theTest, string theName, B.Shape3D theShape, IEnumerable<B.Edge> theEdges,
                                  double theTolerance = 0.1, int theMaxIterations = 10)
    {
        List<B.Edge> anEdges = [.. theEdges];
        string? anError = null, aMessage;
        try
        {
            aMessage = theShape.MaxFillet(anEdges, theTolerance, theMaxIterations).ToString("G17", CultureInfo.InvariantCulture);
        }
        catch (Exception e)
        {
            anError = ErrorOfB(e);
            aMessage = e.Message;
        }
        theCalls.Add(new W7Call(theName, theTest, P7Op.MaxFillet, [theShape.Wrapped],
                                new P7Args(Tol: theTolerance, Edges: EdgeIndices(theShape, anEdges), Value2: theShape.BoundingBox().Diagonal,
                                           MaxIterations: theMaxIterations),
                                P7Post.None, null, anError, aMessage));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The tests' scenarios

    private static B.Part PBox(double l, double w, double h) => new([B.Solid.MakeBox(l, w, h, new B.Plane((-l / 2, -w / 2, -h / 2)))]);

    private static B.Part PCylinder(double r, double h) => new([B.Solid.MakeCylinder(r, h, new B.Plane((0, 0, -h / 2)))]);

    private static IEnumerable<(string, Action<List<W7Call>, string>)> BlendScenarios()
    {
        // Shape3DTests.cs: FilletFailuresTests
        yield return ("FilletFailuresTests.Plate", (c, t) =>
        {
            var aBox = PBox(20, 20, 2);
            Fillet(c, t, "plate as a Part", aBox, aBox.Edges(), 3);
            var aSolid = aBox.SingleSolid();
            Fillet(c, t, "plate as a Solid", aSolid, aSolid.Edges(), 3);
            var aPart = new B.Part(aBox.Wrapped);
            Fillet(c, t, "plate as a Part of the compound", aPart, aPart.Edges(), 3);
        });
        yield return ("FilletFailuresTests.Operation", (c, t) => FilletOperation(c, t, "plate", PBox(20, 20, 2).Edges(), 3));
        yield return ("FilletFailuresTests.TangentChain", (c, t) =>
        {
            var aRounded = Ops.Extrude(new B.RectangleRounded(20, 10, 3), 2);
            var aTop = aRounded.Edges().GroupBy(B.Axis.Z)[^1];
            Fillet(c, t, "a line", aRounded, aTop.FilterBy(B.GeomType.Line)[..1], 3);
            Fillet(c, t, "an arc", aRounded, aTop.FilterBy(B.GeomType.Circle)[..1], 3);
            Fillet(c, t, "seven edges", aRounded, aTop[..7], 3);
        });
        yield return ("FilletFailuresTests.PlainMessage", (c, t) =>
        {
            var aCube = PBox(10, 10, 10);
            Fillet(c, t, "cube", aCube, aCube.Edges(), 6);
            var aNotched = (B.Shape3D)(PBox(20, 20, 12) - new B.Pos(7, 7, 4) * PBox(12, 12, 12));
            Fillet(c, t, "notched", aNotched, aNotched.Edges().Where((_, i) => i % 2 == 0), 0.5);
        });
        yield return ("FilletFailuresTests.GeneratorDuplicatesAndReversedEdges", (c, t) =>
        {
            var aPlate = PBox(20, 20, 2);
            var anEdges = aPlate.Edges();
            var aReversed = new B.Edge(W.TopoDS.Edge(anEdges[1].Wrapped.Reversed()));
            Fillet(c, t, "repeated and reversed", aPlate, [anEdges[3], anEdges[0], aReversed, anEdges[1]], 3);
        });
        // Shape3DTests
        yield return ("Shape3DTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(1, 1, 1);
            Chamfer(c, t, "Chamfer", aBox, aBox.Edges().SortBy(B.Axis.Z)[^1..], 0.1);
            Chamfer(c, t, "ChamferAsymLength", aBox, aBox.Edges().SortBy(B.Axis.Z)[^1..], 0.1, 0.2);
            var aBottom = aBox.Faces().SortBy(B.Axis.Z)[0];
            Chamfer(c, t, "ChamferAsymLength on a face", aBox, [aBottom.Edges().SortBy(B.Axis.Y)[0]], 0.1, 0.2, theFace: aBottom);
            var aPBox = PBox(1, 1, 1);
            Chamfer(c, t, "ChamferObjectInstanceReturnsTopology", aPBox, aPBox.Edges().SortBy(B.Axis.Z)[^1..], 0.1);
            Fillet(c, t, "FilletObjectInstanceReturnsTopology box", aPBox, aPBox.Edges().SortBy(B.Axis.Z)[^1..], 0.1);
            var aCylinder = PCylinder(1, 1);
            Fillet(c, t, "FilletObjectInstanceReturnsTopology cylinder", aCylinder, aCylinder.Edges().SortBy(B.Axis.Z)[0..1], 0.1);
            MaxFillet(c, t, "MaxFilletObjectInstance", aPBox, aPBox.Edges().SortBy(B.Axis.Z)[^1..]);
            Chamfer(c, t, "ChamferTooHighLength", aBox, aBox.Edges().SortBy(B.Axis.Z)[^1..], 2);
        });
        // ShapeTests
        yield return ("ShapeTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(10, 8, 2);
            MaxFillet(c, t, "MaxFillet box", aBox, aBox.Edges());
            var aCone = B.Solid.MakeCone(5, 3, 8);
            MaxFillet(c, t, "MaxFillet cone", aCone, aCone.Edges());
            MaxFillet(c, t, "MaxFillet in one iteration", aBox, aBox.Edges(), 1e-6, 1);
            var aCube = B.Solid.MakeBox(1, 1, 1);
            Fillet(c, t, "MaxFillet's fillet too large", aCube, aCube.Edges(), 0.75);
            var aBox4 = B.Solid.MakeBox(4, 4, 4);
            Fillet(c, t, "IsManifold", aBox4, aBox4.Edges(), 0.5);
        });
        // OperationsGenericTests
        yield return ("OperationsGenericTests", (c, t) =>
        {
            FilletOperation(c, t, "Fillet3D", new B.Box(1, 2, 3).Edges(), 0.2);
            Chamfer(c, t, "Chamfer3D", null, new B.Box(1, 2, 3).Edges(), 0.2);
            var aCylinder = Ops.Extrude(new B.Circle(1), amount: 1);
            Chamfer(c, t, "ExtrudeThenChamfer", null, aCylinder.Edges().FilterBy(B.GeomType.Circle), 0.3);
            Chamfer(c, t, "PartChamfer", null, new B.Box(10, 10, 10).Edges(), 1);
            Chamfer(c, t, "PartChamferAsymLength", null, new B.Box(10, 10, 10).Edges().FilterBy(B.Axis.Z), 1, Math.Sqrt(3));
            Chamfer(c, t, "PartChamferAsymAngle", null, new B.Box(10, 10, 10).Edges().FilterBy(B.Axis.Z), 1, theAngle: 60);
            var aBox = new B.Box(10, 10, 10);
            var aTop = aBox.Faces().SortBy(B.Axis.Z)[^1];
            Chamfer(c, t, "PartChamferAsymLengthReference", null, [aTop.Edges()[0]], 1, Math.Sqrt(3), theFace: aTop);
            Chamfer(c, t, "PartChamferAsymAngleReference", null, [aTop.Edges()[0]], 1, theAngle: 60, theFace: aTop);
            Chamfer(c, t, "PartChamferRecordsHistory", null, new B.Box(10, 10, 10).Edges(), 1);
            FilletOperation(c, t, "PartFillet", new B.Box(10, 10, 10).Edges(), 1);
        });
        // Errors
        yield return ("ErrorContextTests", (c, t) => FilletOperation(c, t, "fillet too large", new B.Box(1, 1, 1).Edges(), 5));
        yield return ("MalformedInputTests", (c, t) =>
        {
            FilletOperation(c, t, "ValidBRepStillReads", new B.Box(4, 4, 4).Edges(), 1);
            FilletOperation(c, t, "fillet too large", new B.Box(1, 1, 1).Edges(), 5);
            Chamfer(c, t, "chamfer too large", null, new B.Box(1, 1, 1).Edges(), 5);
        });
        yield return ("MemoryPressureTests", (c, t) => FilletOperation(c, t, "box", new B.Box(10, 10, 10).Edges(), 1));
        yield return ("GeometryWithShapesTests", (c, t) => FilletOperation(c, t, "ShiftOriginVector", new B.Box(4, 4, 2).Edges().FilterBy(B.Axis.Z), 0.5));
        yield return ("InputsUnchangedTests", (c, t) =>
        {
            var aBox = new B.Box(10, 10, 10);
            FilletOperation(c, t, "fillet", aBox.Edges(), 1);
            var aFused = (B.Part)(aBox + new B.Cylinder(3, 14));
            FilletOperation(c, t, "fillet top", aFused.Edges().GroupBy(B.Axis.Z).Last, 1);
            Chamfer(c, t, "chamfer", null, aBox.Edges().GroupBy(B.Axis.Z).Last, 1);
        });
        yield return ("OperationsPartTests", (c, t) => FilletOperation(c, t, "SplitWrappedObject", new B.Box(1, 1, 1).Edges().GroupBy(B.Axis.Z).Last, 0.1));
        yield return ("EdgeTests", (c, t) => FilletOperation(c, t, "ConvexitySmooth", new B.Box(10, 10, 10).Edges(), 2));
        yield return ("FaceTests", (c, t) =>
        {
            var aBox = new B.Box(20, 20, 5);
            var anOpenBox = Ops.Offset(aBox, amount: -2, openings: [aBox.Faces().SortBy(B.Axis.Z)[^1]]);
            FilletOperation(c, t, "IsConvexConcave", anOpenBox.Edges(), 0.5);
        });
        yield return ("HistoryTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(10, 10, 10);
            Fillet(c, t, "AlgorithmHistoryFromAFillet", aBox, aBox.Edges().FilterBy(B.Axis.Z), 1);
            FilletOperation(c, t, "OperationsOtherThanBooleans", new B.Box(10, 10, 10).Edges().FilterBy(B.Axis.Z), 1);
        });
        yield return ("SelectorTests", (c, t) =>
        {
            FilletOperation(c, t, "plate with a hole", (new B.Box(20, 10, 4) - new B.Cylinder(2, 10)).Edges().GroupBy(B.Axis.Z).Last.FilterBy(B.GeomType.Line), 1);
            FilletOperation(c, t, "bigger plate", (new B.Box(30, 15, 6) - new B.Cylinder(3, 10)).Edges().GroupBy(B.Axis.Z).Last.FilterBy(B.GeomType.Line), 1);
        });
        yield return ("Shape1DTests.EdgesTopoParent", (c, t) =>
        {
            var aPlan = B.Face.MakeRect(80, 150) - B.Face.MakeRect(25, 25, new B.Plane((-20, 55)));
            var aCase = Ops.Extrude(aPlan, 2);
            FilletOperation(c, t, "window", aCase.Faces().SortBy(B.Axis.Z)[^1].InnerWires()[0].Edges(), 1);
            var aCaseF = Ops.Fillet(aCase.Faces().SortBy(B.Axis.Z)[^1].InnerWires()[0].Edges(), 1);
            FilletOperation(c, t, "perimeter", aCaseF.Faces().SortBy(B.Axis.Z)[^1].OuterWire().Edges(), 1);
        });
        yield return ("ShapeListTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(20, 20, 10, new B.Plane((-10, -10, -5)));
            Fillet(c, t, "FilterByValueRadiusOverMixedFaces", aBox, aBox.Edges().FilterBy(B.Axis.Z), 2);
        });
        yield return ("ShapeMeshTests", (c, t) =>
        {
            FilletOperation(c, t, "TheOrderIsTheShapes", new B.Box(10, 10, 10).Edges().GroupBy(B.Axis.Z).Last, 2);
            FilletOperation(c, t, "TheSameShapeAndTolerancesGiveTheSameMesh", new B.Box(10, 10, 10).Edges(), 1);
        });
        yield return ("SolidSmokeTests", (c, t) =>
        {
            var aBox = B.Solid.MakeBox(10, 10, 10);
            Fillet(c, t, "FilletTheTopEdges", aBox, aBox.Edges().GroupBy(B.Axis.Z).Last, 1);
        });
        yield return ("SolidTests.OffsetOrientationTests", (c, t) =>
        {
            var aCone = B.Solid.MakeCone(22.5, 33.75, 40);
            var aCup = (B.Shape3D)aCone.Offset3D(aCone.Faces().SortBy(B.Axis.Z)[^1..], 3, kind: B.Kind.Intersection);
            Fillet(c, t, "OffsetCupCanBeFilleted", aCup, [aCup.Edges().FilterBy(B.GeomType.Circle).SortBy(B.Axis.Z)[0]], 1);
            foreach ((double anAmount, B.Kind aKind) in new[] { (2.0, B.Kind.Arc), (2.0, B.Kind.Intersection), (-2.0, B.Kind.Arc), (-2.0, B.Kind.Intersection) })
            {
                var aBox = B.Solid.MakeBox(30, 20, 10);
                var aShelled = (B.Shape3D)aBox.Offset3D(aBox.Faces().SortBy(B.Axis.Z)[^1..], anAmount, kind: aKind);
                Fillet(c, t, $"OffsetBoxBothWaysAndKinds {anAmount} {aKind}", aShelled,
                       aShelled.Edges().FilterBy(B.GeomType.Line).GroupBy(B.Axis.Z)[^1], 0.5);
            }
        });
        yield return ("TopologyReferenceTests", (c, t) =>
        {
            var aBlock = B.Solid.MakeBox(10, 10, 10, new B.Plane(new B.Vector(0, 0, 0)));
            Fillet(c, t, "fillet_top", aBlock, aBlock.Edges().GroupBy(B.Axis.Z).Last, 1.5);
            Chamfer(c, t, "chamfer_vertical", aBlock, aBlock.Edges().FilterBy(B.Axis.Z), 1);
            Chamfer(c, t, "chamfer_asymmetric", aBlock, aBlock.Edges().GroupBy(B.Axis.Z).Last, 2, 1);
        });
    }

#endif

    // ---------------------------------------------------------------------------------------------------------------
    // The runners

    private static bool IsTracked(N.TopAbs_ShapeEnum theType) =>
        theType is N.TopAbs_ShapeEnum.TopAbs_VERTEX or N.TopAbs_ShapeEnum.TopAbs_EDGE or N.TopAbs_ShapeEnum.TopAbs_FACE or N.TopAbs_ShapeEnum.TopAbs_SOLID;

    private static bool IsTracked(TopAbs_ShapeEnum theType) =>
        theType is TopAbs_ShapeEnum.TopAbs_VERTEX or TopAbs_ShapeEnum.TopAbs_EDGE or TopAbs_ShapeEnum.TopAbs_FACE or TopAbs_ShapeEnum.TopAbs_SOLID;

    // Make3DResult's unwrapping of a compound of one shape
    private static N.TopoDS_Shape NUnwrap(N.TopoDS_Shape theShape)
    {
        while (theShape.ShapeType() == N.TopAbs_ShapeEnum.TopAbs_COMPOUND && theShape.NbChildren() == 1)
        {
            using var anIt = new N.TopoDS_Iterator(theShape);
            N.TopoDS_Shape aChild = anIt.Value();
            GC.KeepAlive(theShape);
            theShape = aChild;
        }
        return theShape;
    }

    private static TopoDS_Shape PUnwrap(TopoDS_Shape theShape)
    {
        while (theShape.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND && theShape.NbChildren() == 1)
        {
            theShape = new TopoDS_Iterator(theShape).Value();
        }
        return theShape;
    }

    // Shape.IsValid on Make3DResult's shape
    private static bool NValid(N.TopoDS_Shape theShape)
    {
        using var aCheck = new N.BRepCheck_Analyzer(NUnwrap(theShape));
        aCheck.SetParallel(true);
        return aCheck.IsValid();
    }

    private static bool PValid(TopoDS_Shape theShape)
    {
        var aCheck = new BRepCheck_Analyzer(PUnwrap(theShape));
        aCheck.SetParallel(true);
        return aCheck.IsValid();
    }

    /// <summary>
    /// The faulty contours as build123d.NET's FilletFailureDetails reads them, by index: for each reason in order of
    /// first appearance, the selection's positions on a faulty contour and the number of the contours' other edges.
    /// </summary>
    private static string Details(int[] theSelection, List<(string Reason, HashSet<int> Edges)> theFaulty)
    {
        var anOrder = new List<string>();
        var aSets = new Dictionary<string, HashSet<int>>();
        foreach ((string aReason, HashSet<int> anEdges) in theFaulty)
        {
            if (!aSets.TryGetValue(aReason, out HashSet<int>? aSet))
            {
                aSets[aReason] = aSet = [];
                anOrder.Add(aReason);
            }
            aSet.UnionWith(anEdges);
        }
        if (anOrder.Count == 0)
        {
            return "";
        }
        int[] aSelected = [.. theSelection.Select(Math.Abs)];
        var aRejected = new HashSet<int>();
        var aGroups = new List<string>();
        foreach (string aReason in anOrder)
        {
            HashSet<int> aSet = aSets[aReason];
            var anEntries = new List<string>();
            for (int i = 0; i < aSelected.Length; i++)
            {
                if (aSet.Contains(aSelected[i]))
                {
                    aRejected.Add(i);
                    anEntries.Add($"[{i}]");
                }
            }
            int aTangent = aSet.Count(e => !aSelected.Contains(e));
            if (aTangent > 0)
            {
                anEntries.Add($"{aTangent} tangent");
            }
            aGroups.Add($"{aReason}: {string.Join(", ", anEntries)}");
        }
        return $" on {aRejected.Count} of {aSelected.Length} edges ({string.Join("; ", aGroups)})";
    }

    private static string Reason(int theStatus) => theStatus switch
    {
        2 => "walking failure", // ChFiDS_WalkingFailure
        3 => "no starting solution", // ChFiDS_StartsolFailure
        4 => "twisted surface", // ChFiDS_TwistedSurface
        _ => "error",
    };

    // The history build123d.NET records (ShapeHistory.FromAlgorithm) and IsDeleted: for each vertex, edge, face and
    // solid of the operand, Generated (of the tracked types) and Modified as indices in the result's map, D if
    // IsDeleted, P if the result has it
    private static string BlendHistory(N.TopoDS_Shape theOp, N.TopoDS_Shape theResult, N.BRepBuilderAPI_MakeShape theMaker)
    {
        using var anIn = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theOp, anIn);
        using var aRes = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theResult, aRes);
        var aSb = new StringBuilder();
        for (int i = 1; i <= anIn.Extent(); i++)
        {
            using N.TopoDS_Shape aS = anIn.FindKey(i);
            if (!IsTracked(aS.ShapeType()))
            {
                continue;
            }
            using N.TopTools_ListOfShape aG = theMaker.Generated(aS);
            using N.TopTools_ListOfShape aM = theMaker.Modified(aS);
            aSb.Append(i).Append(":G[").Append(Sorted(aG.Where(x => IsTracked(x.ShapeType())).Select(x => Index(aRes, x)))).Append("]M[")
               .Append(Sorted(aM.Select(x => Index(aRes, x)))).Append(']').Append(theMaker.IsDeleted(aS) ? "D" : "")
               .Append(aRes.Contains(aS) ? "P" : "").Append('\n');
        }
        return aSb.ToString();
    }

    private static string BlendHistory(in TopoDS_Shape theOp, in TopoDS_Shape theResult, BRepBuilderAPI_MakeShape theMaker)
    {
        var anIn = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theOp, anIn);
        var aRes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theResult, aRes);
        var aSb = new StringBuilder();
        for (int i = 1; i <= anIn.Extent(); i++)
        {
            TopoDS_Shape aS = anIn[i];
            if (!IsTracked(aS.ShapeType()))
            {
                continue;
            }
            aSb.Append(i).Append(":G[").Append(Sorted(theMaker.Generated(aS).Where(x => IsTracked(x.ShapeType())).Select(x => Index(aRes, x))))
               .Append("]M[").Append(Sorted(theMaker.Modified(aS).Select(x => Index(aRes, x)))).Append(']')
               .Append(theMaker.IsDeleted(aS) ? "D" : "").Append(aRes.Contains(aS) ? "P" : "").Append('\n');
        }
        return aSb.ToString();
    }

    private static N.TopoDS_Edge NEdge(N.TopTools_IndexedMapOfShape theEdges, int theK) =>
        N.TopoDS.Edge(theEdges.FindKey(Math.Abs(theK)).Oriented(theK < 0 ? N.TopAbs_Orientation.TopAbs_REVERSED : N.TopAbs_Orientation.TopAbs_FORWARD));

    private static TopoDS_Edge PEdge(NCollection_IndexedMap<TopoDS_Shape> theEdges, int theK) =>
        TopoDS.Edge(theEdges[Math.Abs(theK)].Oriented(theK < 0 ? TopAbs_Orientation.TopAbs_REVERSED : TopAbs_Orientation.TopAbs_FORWARD));

    // One fillet natively: the result (or the exception), the status (done; the faulty contours' details by index)
    private static (string? Error, string Status, N.TopoDS_Shape? Result, string History, string? Details) NativeFillet(
        N.TopoDS_Shape theShape, int[] theEdges, double theRadius, bool theHistory, bool theMessage)
    {
        using var anEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
        using var aB = new N.BRepFilletAPI_MakeFillet(theShape);
        foreach (int k in theEdges)
        {
            aB.Add(theRadius, NEdge(anEdges, k));
        }
        string? anError = null;
        N.TopoDS_Shape? aRes = null;
        try
        {
            aRes = aB.Shape();
        }
        catch (N.OcctException e)
        {
            anError = ErrorOf(e);
        }
        // FilletFailureDetails: the faulty contours within NbContours, their stripe status and edges
        var aFaulty = new List<(string, HashSet<int>)>();
        var aCentres = new List<(string, List<N.TopoDS_Edge>)>();
        var aStatus = new StringBuilder($"done={aB.IsDone()} contours={aB.NbContours()} faulty={aB.NbFaultyContours()}:");
        for (int i = 1; i <= aB.NbFaultyContours(); i++)
        {
            int aContour = aB.FaultyContour(i);
            aStatus.Append(' ').Append(aContour);
            if (aContour < 1 || aContour > aB.NbContours())
            {
                continue;
            }
            int aStripe = (int)aB.StripeStatus(aContour);
            aStatus.Append('/').Append(aStripe);
            var aSet = new HashSet<int>();
            var aList = new List<N.TopoDS_Edge>();
            for (int j = 1; j <= aB.NbEdges(aContour); j++)
            {
                N.TopoDS_Edge anE = aB.Edge(aContour, j);
                aSet.Add(anEdges.FindIndex(anE));
                aList.Add(anE);
            }
            aFaulty.Add((Reason(aStripe), aSet));
            aCentres.Add((Reason(aStripe), aList));
        }
        aStatus.Append(" details=").Append(Details(theEdges, aFaulty));
        // build123d.NET's own formatting, so recorded (unit 10A)
#if HAS_NATIVE
        string? aDetails = theMessage ? MessageDetails(theShape, theEdges, aCentres) : null;
#else
        string? aDetails = theMessage ? Tape.Value("message details", () =>
        {
#if HAS_ORACLE
            return MessageDetails(theShape, theEdges, aCentres);
#else
            return Tape.NeedsNative<string>();
#endif
        }) : null;
#endif
        string aHistory = theHistory && aRes is not null ? BlendHistory(theShape, aRes, aB) : "";
        return (anError, aStatus.ToString(), aRes, aHistory, aDetails);
    }

#if HAS_ORACLE || HAS_NATIVE
    // build123d.NET's message details, from native's edges (their centres as build123d.NET prints them)
    private static string MessageDetails(N.TopoDS_Shape theShape, int[] theSelection, List<(string Reason, List<N.TopoDS_Edge> Edges)> theFaulty)
    {
        using var anEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
        var aFaulty = new Dictionary<string, HashSet<B.Edge>>();
        var anOrder = new List<string>();
        foreach ((string aReason, List<N.TopoDS_Edge> aList) in theFaulty)
        {
            if (!aFaulty.TryGetValue(aReason, out HashSet<B.Edge>? aSet))
            {
                aFaulty[aReason] = aSet = [];
                anOrder.Add(aReason);
            }
            foreach (N.TopoDS_Edge anE in aList)
            {
                aSet.Add(new B.Edge(RecordedNative.Real<W.TopoDS_Edge>(anE)));
            }
        }
        if (aFaulty.Count == 0)
        {
            return "";
        }
        List<B.Edge> aSelected = [.. theSelection.Select(k => new B.Edge(RecordedNative.Real<W.TopoDS_Edge>(NEdge(anEdges, k))))];
        var aRejected = new HashSet<int>();
        var aGroups = new List<string>();
        foreach (string aReason in anOrder)
        {
            HashSet<B.Edge> aSet = aFaulty[aReason];
            var anEntries = new List<string>();
            for (int i = 0; i < aSelected.Count; i++)
            {
                if (aSet.Contains(aSelected[i]))
                {
                    aRejected.Add(i);
                    anEntries.Add($"[{i}] at {aSelected[i].Center().ToString(".6g")}");
                }
            }
            int aTangent = aSet.Count(e => !aSelected.Contains(e));
            if (aTangent > 0)
            {
                anEntries.Add($"{aTangent} additional tangent {(aTangent == 1 ? "edge" : "edges")}");
            }
            aGroups.Add($"{aReason}: {string.Join(", ", anEntries)}");
        }
        return $" on {aRejected.Count} of {aSelected.Count} edges ({string.Join("; ", aGroups)})";
    }
#endif

    private static (string? Error, string Status, TopoDS_Shape? Result, string History) PortFillet(TopoDS_Shape theShape, int[] theEdges,
                                                                                                    double theRadius, bool theHistory)
    {
        var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
        var aB = new BRepFilletAPI_MakeFillet(theShape);
        foreach (int k in theEdges)
        {
            aB.Add(theRadius, PEdge(anEdges, k));
        }
        string? anError = null;
        TopoDS_Shape? aRes = null;
        try
        {
            aRes = aB.Shape();
        }
        catch (Exception e) when (e is Standard_Failure or NullReferenceException)
        {
            anError = ErrorOf(e);
        }
        var aFaulty = new List<(string, HashSet<int>)>();
        var aStatus = new StringBuilder($"done={aB.IsDone()} contours={aB.NbContours()} faulty={aB.NbFaultyContours()}:");
        for (int i = 1; i <= aB.NbFaultyContours(); i++)
        {
            int aContour = aB.FaultyContour(i);
            aStatus.Append(' ').Append(aContour);
            if (aContour < 1 || aContour > aB.NbContours())
            {
                continue;
            }
            int aStripe = (int)aB.StripeStatus(aContour);
            aStatus.Append('/').Append(aStripe);
            var aSet = new HashSet<int>();
            for (int j = 1; j <= aB.NbEdges(aContour); j++)
            {
                aSet.Add(anEdges.FindIndex(aB.Edge(aContour, j)));
            }
            aFaulty.Add((Reason(aStripe), aSet));
        }
        aStatus.Append(" details=").Append(Details(theEdges, aFaulty));
        string aHistory = theHistory && aRes is TopoDS_Shape r ? BlendHistory(theShape, r, aB) : "";
        return (anError, aStatus.ToString(), aRes, aHistory);
    }

    private static P7Native RunBlendNative(P7Call theCall, List<N.TopoDS_Shape> theOps, bool theHistory)
    {
        P7Args a = theCall.Args;
        N.TopoDS_Shape aShape = theOps[0];
        try
        {
            switch (theCall.Op)
            {
                case P7Op.Fillet:
                {
                    (string? anError, string aStatus, N.TopoDS_Shape? aRes, string aHistory, string? aDetails) =
                        NativeFillet(aShape, a.Edges!, a.Value, theHistory, theHistory);
                    return new P7Native(anError, aStatus, aRes, aHistory, Details: aDetails);
                }
                case P7Op.Chamfer:
                {
                    using var anEdges = new N.TopTools_IndexedMapOfShape();
                    N.TopExp.MapShapes(aShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
                    using var aFaces = new N.TopTools_IndexedMapOfShape();
                    N.TopExp.MapShapes(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
                    using var anEF = new N.TopTools_IndexedDataMapOfShapeListOfShape();
                    N.TopExp.MapShapesAndAncestors(aShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, N.TopAbs_ShapeEnum.TopAbs_FACE, anEF);
                    using var aB = new N.BRepFilletAPI_MakeChamfer(aShape);
                    var aChosen = new List<int>();
                    for (int i = 0; i < a.Edges!.Length; i++)
                    {
                        N.TopoDS_Edge anE = NEdge(anEdges, a.Edges[i]);
                        N.TopoDS_Face aF;
                        if (a.Faces![i] > 0)
                        {
                            aF = N.TopoDS.Face(aFaces.FindKey(a.Faces[i]));
                        }
                        else
                        {
                            using N.TopTools_ListOfShape anAncestors = anEF.FindFromKey(anE);
                            aF = N.TopoDS.Face(anAncestors.First());
                        }
                        aChosen.Add(aFaces.FindIndex(aF));
                        aB.Add(a.Value, a.Value2, anE, aF);
                    }
                    string? anError = null;
                    N.TopoDS_Shape? aRes = null;
                    try
                    {
                        aRes = aB.Shape();
                    }
                    catch (N.OcctException e)
                    {
                        anError = ErrorOf(e);
                    }
                    string aStatus = $"done={aB.IsDone()} contours={aB.NbContours()} faces={string.Join(',', aChosen)}";
                    return new P7Native(anError, aStatus, aRes, theHistory && aRes is not null ? BlendHistory(aShape, aRes, aB) : "");
                }
                default:
                {
                    // MaxFillet: Search(0, 2 * diagonal, 0), a probe being valid if the fillet's result is
                    if (!NValid(aShape))
                    {
                        return new P7Native("ArgumentException", "invalid shape", null, "");
                    }
                    var aTrace = new StringBuilder();
                    double Search(double theMin, double theMax, int theIteration)
                    {
                        double aMid = (theMin + theMax) / 2;
                        if (theIteration == a.MaxIterations)
                        {
                            throw new InvalidOperationException();
                        }
                        (string? anError, _, N.TopoDS_Shape? aRes, _, _) = NativeFillet(aShape, a.Edges!, aMid, false, false);
                        bool isOk = anError is null && aRes is not null && NValid(aRes);
                        aRes?.Dispose();
                        aTrace.Append(isOk ? " +" : " -").Append(aMid.ToString("R", CultureInfo.InvariantCulture));
                        if (!isOk)
                        {
                            return Search(theMin, aMid, theIteration + 1);
                        }
                        return aMid - theMin <= a.Tol ? aMid : Search(aMid, theMax, theIteration + 1);
                    }
                    try
                    {
                        double aR = Search(0.0, 2 * a.Value2, 0);
                        return new P7Native(null, "max=" + aR.ToString("G17", CultureInfo.InvariantCulture) + " probes" + aTrace, null, "");
                    }
                    catch (InvalidOperationException)
                    {
                        return new P7Native("InvalidOperationException", "probes" + aTrace, null, "");
                    }
                }
            }
        }
        catch (N.OcctException e)
        {
            return new P7Native(ErrorOf(e), "", null, "");
        }
    }

    private static P7Port RunBlendPort(P7Call theCall, List<TopoDS_Shape> theOps, bool theHistory)
    {
        P7Args a = theCall.Args;
        TopoDS_Shape aShape = theOps[0];
        try
        {
            switch (theCall.Op)
            {
                case P7Op.Fillet:
                {
                    (string? anError, string aStatus, TopoDS_Shape? aRes, string aHistory) = PortFillet(aShape, a.Edges!, a.Value, theHistory);
                    return new P7Port(anError, aStatus, aRes, aHistory);
                }
                case P7Op.Chamfer:
                {
                    var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapes(aShape, TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
                    var aFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapes(aShape, TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
                    var anEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapesAndAncestors(aShape, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_FACE, anEF);
                    var aB = new BRepFilletAPI_MakeChamfer(aShape);
                    var aChosen = new List<int>();
                    for (int i = 0; i < a.Edges!.Length; i++)
                    {
                        TopoDS_Edge anE = PEdge(anEdges, a.Edges[i]);
                        TopoDS_Face aF = a.Faces![i] > 0 ? TopoDS.Face(aFaces[a.Faces[i]]) : TopoDS.Face(anEF.FindFromKey(anE).First());
                        aChosen.Add(aFaces.FindIndex(aF));
                        aB.Add(a.Value, a.Value2, anE, aF);
                    }
                    string? anError = null;
                    TopoDS_Shape? aRes = null;
                    try
                    {
                        aRes = aB.Shape();
                    }
                    catch (Exception e) when (e is Standard_Failure or NullReferenceException)
                    {
                        anError = ErrorOf(e);
                    }
                    string aStatus = $"done={aB.IsDone()} contours={aB.NbContours()} faces={string.Join(',', aChosen)}";
                    return new P7Port(anError, aStatus, aRes, theHistory && aRes is TopoDS_Shape r ? BlendHistory(aShape, r, aB) : "");
                }
                default:
                {
                    if (!PValid(aShape))
                    {
                        return new P7Port("ArgumentException", "invalid shape", null, "");
                    }
                    var aTrace = new StringBuilder();
                    double Search(double theMin, double theMax, int theIteration)
                    {
                        double aMid = (theMin + theMax) / 2;
                        if (theIteration == a.MaxIterations)
                        {
                            throw new InvalidOperationException();
                        }
                        (string? anError, _, TopoDS_Shape? aRes, _) = PortFillet(aShape, a.Edges!, aMid, false);
                        bool isOk = anError is null && aRes is TopoDS_Shape r && PValid(r);
                        aTrace.Append(isOk ? " +" : " -").Append(aMid.ToString("R", CultureInfo.InvariantCulture));
                        if (!isOk)
                        {
                            return Search(theMin, aMid, theIteration + 1);
                        }
                        return aMid - theMin <= a.Tol ? aMid : Search(aMid, theMax, theIteration + 1);
                    }
                    try
                    {
                        double aR = Search(0.0, 2 * a.Value2, 0);
                        return new P7Port(null, "max=" + aR.ToString("G17", CultureInfo.InvariantCulture) + " probes" + aTrace, null, "");
                    }
                    catch (InvalidOperationException)
                    {
                        return new P7Port("InvalidOperationException", "probes" + aTrace, null, "");
                    }
                }
            }
        }
        catch (Exception e) when (e is Standard_Failure or NullReferenceException)
        {
            return new P7Port(ErrorOf(e), "", null, "");
        }
    }
}
