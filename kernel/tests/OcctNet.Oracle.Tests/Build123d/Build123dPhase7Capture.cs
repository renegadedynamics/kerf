// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text;
using System.Text.Json;

namespace OcctNet.Oracle.Tests;

/// <summary>The phase 7 calls build123d.NET makes: the native operation and how build123d.NET finishes its result.</summary>
internal enum P7Op { ThickSolid, MakeOffset, Draft, DPrism, Split, ToSplines, Fillet, Chamfer, MaxFillet }

/// <summary>
/// What build123d.NET does with the call's result before handing it back (compared separately, on native's result in
/// both): <c>Hollow</c> without openings makes a solid of the two shells and fixes it; <c>Offset3D</c> makes the solid
/// FORWARD and reverses an inside-out one (and the Offset operation fixes it); the Thicken operation cleans; ExtrudeTaper
/// checks the drafted solid's validity; SplitByPerimeter sews each side of more than one face.
/// </summary>
internal enum P7Post { None, HollowSolid, Offset3D, OffsetOperation, Clean, Validity, SewSides }

/// <summary>
/// The arguments of one call (the faces of <c>Faces</c> are indices in the first operand's face map). A fillet's or
/// chamfer's <c>Edges</c> are indices in the first operand's edge map, negative for an edge given REVERSED; a chamfer's
/// <c>Faces</c> give its face per edge (0: the edge's first ancestor face, as build123d.NET picks it) and its distances
/// are <c>Value</c> and <c>Value2</c>; MaxFillet's window is twice <c>Value2</c> (the shape's diagonal), its tolerance
/// <c>Tol</c>.
/// </summary>
internal sealed record P7Args(double Value = 0, double Tol = 0, int Join = 0, bool Inter = false, bool Thickening = false,
                              bool RemoveIntEdges = false, bool ThreeArgs = false, int[]? Faces = null, double[]? Angles = null,
                              double[]? Dir = null, double[]? PlnLoc = null, int Mode = 0, double Taper = 0, int Fuse = 0,
                              bool HasBasis = false, int NbProfiles = 1, int Degree = 3, bool Nurbs = false,
                              int[]? Edges = null, double Value2 = 0, bool Operation = false, int MaxIterations = 0);

/// <summary>
/// One native call of build123d.NET's phase 7 path, from one of its tests: its operands (as native shapes), arguments and
/// finishing step, and build123d.NET's own result of the operation (or its exception), which checks the transcription.
/// </summary>
internal sealed record P7Call(string Name, string Test, P7Op Op, N.TopoDS_Shape[] Operands, P7Args Args, P7Post Post,
                              N.TopoDS_Shape? Result, string? Error, string? Message = null);

/// <summary>A native run: the exception, the status, the result, the history, and SplitShape's Left and Right.</summary>
internal sealed record P7Native(string? Error, string Status, N.TopoDS_Shape? Result, string History,
                                N.TopoDS_Shape[]? Left = null, N.TopoDS_Shape[]? Right = null, string? Details = null);

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
internal static partial class Build123dPhase7Capture
{
    /// <summary>
    /// Every call, as recorded (unit 10A) by running build123d.NET's scenarios on native OCCT, which went in unit 10B: the
    /// calls' shapes are native's through the recorded facade, the scenarios' failures are kept with them.
    /// </summary>
    internal static List<P7Call> Recorded(List<string> theFailures)
    {
        (List<P7Call> aCalls, List<string> aFailures) = Tape.Value("build123d.NET's phase 7 operations", static r =>
        {
            List<string> aFailed = [.. ((object?[])r.Value()!).Cast<string>()];
            var aCalls = new List<P7Call>();
            for (int i = r.Int(); i > 0; i--)
            {
                aCalls.Add(new P7Call(r.String()!, r.String()!, (P7Op)r.Int(), Tape.Array<N.TopoDS_Shape>(r.Value()),
                                      JsonSerializer.Deserialize<P7Args>(r.String()!)!, (P7Post)r.Int(), r.Object<N.TopoDS_Shape>(),
                                      r.String(), r.String()));
            }
            return (aCalls, aFailed);
        });
        theFailures.AddRange(aFailures);
        return aCalls;
    }

    /// <summary>build123d.NET's operation behind a call.</summary>
    internal static string Label(P7Call theCall) => theCall.Op switch
    {
        P7Op.ThickSolid => theCall.Post is P7Post.Offset3D or P7Post.OffsetOperation ? "Offset3D" : "Hollow",
        P7Op.MakeOffset => theCall.Args.ThreeArgs ? "OffsetTopodsFace" : theCall.Args.Thickening ? "Thicken" : "OffsetCore",
        P7Op.Draft => theCall.Post == P7Post.Validity ? "ExtrudeTaper" : "Draft",
        P7Op.DPrism => "DPrism",
        P7Op.Split => "SplitByPerimeter",
        P7Op.Fillet => theCall.Args.Operation ? "Fillet operation" : "Fillet",
        P7Op.Chamfer => theCall.Args.Operation ? "Chamfer operation" : "Chamfer",
        P7Op.MaxFillet => "MaxFillet",
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
            GC.KeepAlive(aC); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
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
        if (theCall.Op is P7Op.Fillet or P7Op.Chamfer or P7Op.MaxFillet)
        {
            return RunBlendNative(theCall, theOps, theHistory);
        }
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
        if (theCall.Op is P7Op.Fillet or P7Op.Chamfer or P7Op.MaxFillet)
        {
            return RunBlendPort(theCall, theOps, theHistory);
        }
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
