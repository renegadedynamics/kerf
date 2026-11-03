// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

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
        string? aDetails = theMessage ? Tape.Value<string>("message details") : null;
        string aHistory = theHistory && aRes is not null ? BlendHistory(theShape, aRes, aB) : "";
        return (anError, aStatus.ToString(), aRes, aHistory, aDetails);
    }

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
