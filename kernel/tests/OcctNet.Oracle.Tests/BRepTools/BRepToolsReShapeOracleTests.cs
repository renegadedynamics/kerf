// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepTools_ReShape against native OCCT on shapes built identically on both sides (<see cref="TwinShapes"/>; the port's
/// frozen, as a caller's are): random requests (replacing vertices, edges, faces and wires by other sub-shapes of the
/// same type, by new vertices or by compounds of edges; removals; recorded under random orientations), then
/// IsRecorded, Value, ValueLeaf, Status, Apply with random stopping types, and the History's Modified and IsRemoved.
/// Results are compared structurally: types, orientations, locations and children in order, and which shape each
/// result sub-shape is (an input's twin, or the same new shape on both sides). ModeConsiderLocation isn't bound
/// natively; the unit tests cover it.
/// </summary>
public class BRepToolsReShapeOracleTests(ITestOutputHelper output)
{
    private static readonly int Rounds = Tape.Cases(300, 4);

    private static readonly ShapeType[] theRequestTypes = [ShapeType.Vertex, ShapeType.Edge, ShapeType.Wire, ShapeType.Face];

    private static readonly ShapeType[] theUntils =
        [ShapeType.Shape, ShapeType.Shape, ShapeType.Shape, ShapeType.Vertex, ShapeType.Edge, ShapeType.Wire, ShapeType.Face, ShapeType.Shell, ShapeType.Solid, ShapeType.Compound];

    /// <summary>Which native shape each port TShape is: the inputs' twins, then the new shapes met so far.</summary>
    private sealed class Identities
    {
        private readonly Dictionary<TShape, N.TopoDS_Shape> myInputs = new(ReferenceEqualityComparer.Instance);
        private readonly List<N.TopoDS_Shape> myInputList = [];
        private readonly Dictionary<TShape, N.TopoDS_Shape> myNew = new(ReferenceEqualityComparer.Instance);
        public int NewShapes => myNew.Count;

        public void AddInput(Twin<N.TopoDS_Shape, Shape> theShape)
        {
            if (myInputs.TryAdd(theShape.P.TShape()!, theShape.N))
            {
                myInputList.Add(theShape.N);
            }

            var aNIter = new N.TopoDS_Iterator(theShape.N);
            for (var anIt = new Iterator(theShape.P); anIt.More(); anIt.Next(), aNIter.Next())
            {
                AddInput(new(aNIter.Value(), anIt.Value()));
            }
            GC.KeepAlive(theShape.N); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        }

        /// <summary>Compares the two shapes' identity, type, orientation and location; pairs new shapes.</summary>
        public void Match(Oracle theOracle, N.TopoDS_Shape theN, in Shape theP, string theWhat)
        {
            theOracle.Same(theN.IsNull(), theP.IsNull, theWhat + " null");
            if (theN.IsNull() || theP.IsNull)
            {
                return;
            }

            theOracle.Same((int)theN.ShapeType(), (int)theP.ShapeType(), theWhat + " type");
            theOracle.Same((int)theN.Orientation(), (int)theP.Orientation(), theWhat + " orientation");
            using (N.TopLoc_Location aNL = theN.Location())
            {
                theOracle.Same(aNL.IsIdentity(), theP.Location().IsIdentity, theWhat + " identity location");
                using N.gp_Trsf aNT = aNL.Transformation();
                theOracle.Same(aNT, theP.Location().Transformation(), theWhat + " location");
            }

            TShape aT = theP.TShape()!;
            if (myInputs.TryGetValue(aT, out N.TopoDS_Shape? anInput) || myNew.TryGetValue(aT, out anInput))
            {
                theOracle.Same(true, theN.IsPartner(anInput), theWhat + " is the twin");
            }
            else
            {
                bool isKnown = myInputList.Exists(theN.IsPartner) || myNew.Values.Any(theN.IsPartner);
                theOracle.Same(false, isKnown, theWhat + " is new on both sides");
                myNew.Add(aT, theN);
            }
        }

        /// <summary>Matches the shapes and, recursively, their children in order.</summary>
        public void MatchTree(Oracle theOracle, N.TopoDS_Shape theN, in Shape theP, string theWhat, HashSet<TShape> theDone)
        {
            Match(theOracle, theN, theP, theWhat);
            if (theN.IsNull() || theP.IsNull || !theDone.Add(theP.TShape()!))
            {
                return;
            }

            theOracle.Same(theN.NbChildren(), theP.ChildCount, theWhat + " children");
            if (theN.NbChildren() != theP.ChildCount)
            {
                return;
            }

            var aNIt = new N.TopoDS_Iterator(theN);
            for (var anIt = new Iterator(theP); anIt.More(); anIt.Next(), aNIt.Next())
            {
                MatchTree(theOracle, aNIt.Value(), anIt.Value(), theWhat + "/child", theDone);
            }
            GC.KeepAlive(theN); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        }

        /// <summary>True if the native shape is an input's twin or a new shape met so far.</summary>
        public bool IsKnown(N.TopoDS_Shape theN) => myInputList.Exists(theN.IsPartner) || myNew.Values.Any(theN.IsPartner);

        /// <summary>The port shape's native twin, if known.</summary>
        public N.TopoDS_Shape? Twin(in Shape theP)
        {
            TShape aT = theP.TShape()!;
            return myInputs.TryGetValue(aT, out N.TopoDS_Shape? aN) || myNew.TryGetValue(aT, out aN) ? aN : null;
        }
    }

    [Fact]
    public void ReShape_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        int aRequests = 0, anApplies = 0, aHistoryShapes = 0, aNewShapes = 0;
        for (int aRound = 0; aRound < Rounds; aRound++)
        {
            var aTwins = new TwinShapes();
            Twin<N.TopoDS_Shape, Shape> aRoot = Scene(anOracle, aTwins, aRound);
            var anIds = new Identities();
            anIds.AddInput(aRoot);

            // the candidates for requests and replacements: every sub-shape as explored, and some new ones
            var aSubs = new Dictionary<ShapeType, List<Twin<N.TopoDS_Shape, Shape>>>();
            foreach (ShapeType aType in theRequestTypes)
            {
                aSubs[aType] = Explore(aRoot, aType);
            }

            var anExtras = new List<Twin<N.TopoDS_Shape, Shape>>();
            for (int i = 0; i < 3; i++)
            {
                anExtras.Add(TwinShapes.Up(aTwins.Vertex(anOracle.Pnt())));
            }

            List<Twin<N.TopoDS_Shape, Shape>> anEdges = aSubs[ShapeType.Edge];
            if (anEdges.Count >= 2)
            {
                anExtras.Add(TwinShapes.Up(aTwins.Compound(Pick(anOracle, anEdges), Pick(anOracle, anEdges))));
            }

            aRoot.P.TShape()!.FreezeGraph();
            foreach (Twin<N.TopoDS_Shape, Shape> anExtra in anExtras)
            {
                anExtra.P.TShape()!.FreezeGraph();
                anIds.AddInput(anExtra);
            }

            var aNReShape = new N.BRepTools_ReShape();
            var aReShape = new ReShape();
            int aNbRequests = anOracle.Int(1, 6);
            for (int i = 0; i < aNbRequests; i++)
            {
                ShapeType aType = theRequestTypes[anOracle.Int(0, theRequestTypes.Length - 1)];
                if (aSubs[aType].Count == 0)
                {
                    continue;
                }

                Twin<N.TopoDS_Shape, Shape> aShape = Oriented(anOracle, Pick(anOracle, aSubs[aType]));
                int aKind = anOracle.Int(0, 9);
                if (aKind == 0)
                {
                    aNReShape.Remove(aShape.N);
                    aReShape.Remove(aShape.P);
                }
                else
                {
                    Twin<N.TopoDS_Shape, Shape> aNew = aKind switch
                    {
                        1 when aType == ShapeType.Vertex => anExtras[anOracle.Int(0, 2)],
                        2 when aType == ShapeType.Edge && anExtras.Count > 3 => anExtras[3],
                        _ => Pick(anOracle, aSubs[aType]),
                    };
                    aNew = Oriented(anOracle, aNew);
                    aNReShape.Replace(aShape.N, aNew.N);
                    aReShape.Replace(aShape.P, aNew.P);
                }

                aRequests++;
            }

            CompareQueries(anOracle, aNReShape, aReShape, aSubs, anIds, "before Apply");

            ShapeType anUntil = theUntils[anOracle.Int(0, theUntils.Length - 1)];
            Twin<N.TopoDS_Shape, Shape> aStart = Oriented(anOracle, aRoot);
            N.TopoDS_Shape aNResult = aNReShape.Apply(aStart.N, TwinShapes.ToNative(anUntil));
            Shape aResult = aReShape.Apply(aStart.P, anUntil);
            anIds.MatchTree(anOracle, aNResult, aResult, $"Apply until {anUntil}", new HashSet<TShape>(ReferenceEqualityComparer.Instance));
            anOracle.Same(true, aResult.IsNull || aResult.TShape()!.IsFrozen, "Apply's result is frozen");
            anApplies++;

            CompareQueries(anOracle, aNReShape, aReShape, aSubs, anIds, "after Apply");

            // a second Apply on a sub-shape reuses the recorded rebuilt shapes
            if (aSubs[ShapeType.Face].Count > 0)
            {
                Twin<N.TopoDS_Shape, Shape> aFace = Pick(anOracle, aSubs[ShapeType.Face]);
                anIds.MatchTree(anOracle, aNReShape.Apply(aFace.N), aReShape.Apply(aFace.P), "Apply of a face",
                                new HashSet<TShape>(ReferenceEqualityComparer.Instance));
                anApplies++;
            }

            // the history
            N.BRepTools_History aNHistory = aNReShape.History()!;
            History aHistory = aReShape.History();
            foreach (ShapeType aType in new[] { ShapeType.Vertex, ShapeType.Edge, ShapeType.Face, ShapeType.Solid })
            {
                foreach (Twin<N.TopoDS_Shape, Shape> aS in Explore(aRoot, aType))
                {
                    aHistoryShapes++;
                    anOracle.Same(aNHistory.IsRemoved(aS.N), aHistory.IsRemoved(aS.P), "History IsRemoved");
                    var aNModified = new List<N.TopoDS_Shape>(aNHistory.Modified(aS.N));
                    var aModified = aHistory.Modified(aS.P);
                    anOracle.Same(aNModified.Count, aModified.Count, "History Modified count");
                    if (aNModified.Count != aModified.Count)
                    {
                        continue;
                    }

                    // as sets: OCCT's order follows a hash map keyed by addresses
                    foreach (Shape aM in aModified)
                    {
                        N.TopoDS_Shape? aTwin = anIds.Twin(aM);
                        if (aTwin is null)
                        {
                            // a shape rebuilt inside Status(last) and never handed out whole: pair it with the native
                            // element that isn't known either
                            N.TopoDS_Shape? aNewN = aNModified.Find(n => !anIds.IsKnown(n)
                                                                         && (int)n.Orientation() == (int)aM.Orientation());
                            if (aNewN is not null)
                            {
                                anIds.MatchTree(anOracle, aNewN, aM, "History Modified new element",
                                                new HashSet<TShape>(ReferenceEqualityComparer.Instance));
                                aTwin = anIds.Twin(aM);
                            }
                        }

                        bool isFound = aTwin is not null && aNModified.Exists(n => n.IsPartner(aTwin)
                                                                                   && (int)n.Orientation() == (int)aM.Orientation()
                                                                                   && SameLocation(n, aM));
                        anOracle.Same(true, isFound, "History Modified element");
                        anOracle.Same(true, aM.TShape()!.IsFrozen, "History Modified element is frozen");
                    }
                }
            }

            aNewShapes += anIds.NewShapes;
        }

        output.WriteLine($"{Rounds} rounds: {aRequests} requests, {anApplies} applications, {aHistoryShapes} history queries, "
                         + $"{aNewShapes} new shapes paired");
        anOracle.Report("BRepTools_ReShape");
    }

    private static void CompareQueries(Oracle theOracle, N.BRepTools_ReShape theNReShape, ReShape theReShape,
                                       Dictionary<ShapeType, List<Twin<N.TopoDS_Shape, Shape>>> theSubs,
                                       Identities theIds, string theWhen)
    {
        foreach (List<Twin<N.TopoDS_Shape, Shape>> aList in theSubs.Values)
        {
            foreach (Twin<N.TopoDS_Shape, Shape> aS0 in aList)
            {
                foreach (Twin<N.TopoDS_Shape, Shape> aS in new[] { aS0, new(aS0.N.Reversed(), aS0.P.Reversed()) })
                {
                    theOracle.Same(theNReShape.IsRecorded(aS.N), theReShape.IsRecorded(aS.P), $"IsRecorded {theWhen}");
                    theIds.Match(theOracle, theNReShape.Value(aS.N), theReShape.Value(aS.P), $"Value {theWhen}");
                    theIds.Match(theOracle, theNReShape.ValueLeaf(aS.N), theReShape.ValueLeaf(aS.P), $"ValueLeaf {theWhen}");
                    foreach (bool isLast in new[] { false, true })
                    {
                        var aNNew = new N.TopoDS_Shape();
                        int aNStatus = theNReShape.Status(aS.N, aNNew, isLast);
                        int aStatus = theReShape.Status(aS.P, out Shape aNew, isLast);
                        theOracle.Same(aNStatus, aStatus, $"Status {theWhen}");
                        theIds.Match(theOracle, aNNew, aNew, $"Status's shape {theWhen}");
                    }
                }
            }
        }
    }

    private static bool SameLocation(N.TopoDS_Shape theN, in Shape theP)
    {
        using N.TopLoc_Location aNL = theN.Location();
        using N.gp_Trsf aNT = aNL.Transformation();
        Transform3D aT = theP.Location().Transformation();
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 4; c++)
            {
                if (aNT.Value(r, c) != aT.Value(r, c))
                {
                    return false;
                }
            }
        }

        return true;
    }

    private static Twin<N.TopoDS_Shape, Shape> Pick(Oracle theOracle, List<Twin<N.TopoDS_Shape, Shape>> theList)
        => theList[theOracle.Int(0, theList.Count - 1)];

    /// <summary>The shape as it is, reversed, or INTERNAL or EXTERNAL.</summary>
    private static Twin<N.TopoDS_Shape, Shape> Oriented(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape)
    {
        int aKind = theOracle.Int(0, 7);
        return aKind switch
        {
            0 or 1 => new(theShape.N.Reversed(), theShape.P.Reversed()),
            2 => new(theShape.N.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), theShape.P.Oriented(Orientation.Internal)),
            3 => new(theShape.N.Oriented(N.TopAbs_Orientation.TopAbs_EXTERNAL), theShape.P.Oriented(Orientation.External)),
            _ => theShape,
        };
    }

    private static Twin<N.TopoDS_Shape, Shape> Scene(Oracle theOracle, TwinShapes theTwins, int theRound)
    {
        switch (theRound % 3)
        {
            case 0:
                return Placed(theOracle, TwinShapes.Up(theTwins.Box(theOracle.Real(0.5, 20), theOracle.Real(0.5, 20), theOracle.Real(0.5, 20))));
            case 1:
            {
                (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
                return Placed(theOracle, TwinShapes.Up(theTwins.CylinderFace(theOracle.Real(0.5, 5), theOracle.Real(0.5, 5), aL, aNL)));
            }
            default:
            {
                // a compound sharing a box at several locations and orientations, and a cylinder face
                Twin<N.TopoDS_Shape, Shape> aBox = TwinShapes.Up(theTwins.Box(theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5)));
                (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
                Twin<N.TopoDS_Shape, Shape> aFace = TwinShapes.Up(theTwins.CylinderFace(1.0, 2.0, aL, aNL));
                (N.TopLoc_Location aNL1, Placement aL1) = RandomLocation(theOracle);
                return Placed(theOracle, TwinShapes.Up(theTwins.Compound(
                    aBox,
                    new(aBox.N.Moved(aNL1).Reversed(), aBox.P.Moved(aL1).Reversed()),
                    aFace)));
            }
        }
    }

    /// <summary>The shape at a random location (or none), and reversed half the time.</summary>
    private static Twin<N.TopoDS_Shape, Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape)
    {
        Twin<N.TopoDS_Shape, Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    /// <summary>A rigid location (a rotation, a translation or both) built from the same raw numbers on both sides.</summary>
    private static (N.TopLoc_Location, Placement) RandomLocation(Oracle theOracle)
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

        return (new N.TopLoc_Location(aNT), new Placement(aT));
    }

    private static List<Twin<N.TopoDS_Shape, Shape>> Explore(Twin<N.TopoDS_Shape, Shape> theShape, ShapeType theType)
    {
        var aList = new List<Twin<N.TopoDS_Shape, Shape>>();
        var aNExp = new N.TopExp_Explorer(theShape.N, TwinShapes.ToNative(theType), TwinShapes.ToNative(ShapeType.Shape));
        var anExp = new Explorer(theShape.P, theType);
        for (; anExp.More(); anExp.Next(), aNExp.Next())
        {
            Assert.True(aNExp.More(), $"native exploration of {theType} ended first");
            aList.Add(new(aNExp.Current(), anExp.Current()));
        }

        Assert.False(aNExp.More(), $"port exploration of {theType} ended first");
        return aList;
    }
}
