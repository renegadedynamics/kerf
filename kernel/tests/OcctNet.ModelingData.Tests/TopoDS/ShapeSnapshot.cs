// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// A record of a shape graph, to check that an algorithm left its input unchanged: each TShape reached (by identity),
/// its flags and frozen state, its children (TShape, location, orientation), and the BRep data (points, tolerances,
/// flags, every curve and point representation with its geometry by identity, ranges and locations, surfaces and
/// triangulations). Two snapshots are equal iff the text and the referenced objects are the same.
/// </summary>
internal sealed class ShapeSnapshot
{
    private readonly string myText;
    private readonly List<object?> myObjects;

    private ShapeSnapshot(string theText, List<object?> theObjects)
    {
        myText = theText;
        myObjects = theObjects;
    }

    public static ShapeSnapshot Of(in Shape theShape)
    {
        var aText = new StringBuilder();
        var anObjects = new List<object?>();
        var anIndex = new Dictionary<TShape, int>(ReferenceEqualityComparer.Instance);
        aText.Append(CultureInfo.InvariantCulture, $"root {Ref(theShape, anIndex, anObjects)}\n");
        var aQueue = new Queue<TShape>();
        if (!theShape.IsNull)
        {
            aQueue.Enqueue(theShape.TShape()!);
        }

        var aDone = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        while (aQueue.Count > 0)
        {
            TShape aT = aQueue.Dequeue();
            if (!aDone.Add(aT))
            {
                continue;
            }

            aText.Append(CultureInfo.InvariantCulture,
                         $"T{anIndex[aT]} {Kerf.Engine.Foundation.OcctTypeNames.Of(aT.ShapeType())} free={aT.Free()} locked={aT.LockedFlag()} mod={aT.Modified()} chk={aT.Checked()} "
                         + $"ori={aT.Orientable()} closed={aT.Closed()} inf={aT.Infinite()} cvx={aT.Convex()} frozen={aT.IsFrozen}\n");
            for (var anIt = new Iterator(new Shape(aT, new Placement(), Orientation.Forward), false, false);
                 anIt.More(); anIt.Next())
            {
                Shape aChild = anIt.Value();
                aText.Append(CultureInfo.InvariantCulture, $"  child {Ref(aChild, anIndex, anObjects)}\n");
                aQueue.Enqueue(aChild.TShape()!);
            }

            switch (aT)
            {
                case BrepTVertex aV:
                    aText.Append(CultureInfo.InvariantCulture, $"  pnt {aV.Pnt()} tol {aV.Tolerance():R}\n");
                    foreach (PointRepresentation aPR in aV.PointRepresentations())
                    {
                        anObjects.Add(aPR);
                        aText.Append(CultureInfo.InvariantCulture, $"  prep {Kerf.Engine.Foundation.OcctTypeNames.Of(aPR.GetType())} {aPR.Parameter():R} {Loc(aPR.Location())}\n");
                    }

                    break;
                case BrepTEdge anE:
                    aText.Append(CultureInfo.InvariantCulture,
                                 $"  tol {anE.Tolerance():R} sp={anE.SameParameter()} sr={anE.SameRange()} deg={anE.Degenerated()}\n");
                    foreach (CurveRepresentation aCR in anE.CurveRepresentations())
                    {
                        anObjects.Add(aCR);
                        aText.Append(CultureInfo.InvariantCulture, $"  crep {Kerf.Engine.Foundation.OcctTypeNames.Of(aCR.GetType())} {Loc(aCR.Location())} frozen={aCR.IsFrozen}");
                        if (aCR is GCurve aGC)
                        {
                            aText.Append(CultureInfo.InvariantCulture, $" [{aGC.First():R}, {aGC.Last():R}]");
                            anObjects.Add(aGC.IsCurve3D ? aGC.Curve3D() : null);
                            anObjects.Add(aGC.IsCurveOnSurface() ? aGC.PCurve() : null);
                            anObjects.Add(aGC.IsCurveOnSurface() ? aGC.Surface() : null);
                        }

                        aText.Append('\n');
                    }

                    break;
                case BrepTFace aF:
                    aText.Append(CultureInfo.InvariantCulture,
                                 $"  tol {aF.Tolerance():R} nat={aF.NaturalRestriction()} {Loc(aF.Location())} ntri={aF.TriangulationCount}\n");
                    anObjects.Add(aF.Surface());
                    anObjects.Add(aF.ActiveTriangulation());
                    foreach (Triangulation aTri in aF.TriangulationList())
                    {
                        anObjects.Add(aTri);
                    }

                    break;
            }
        }

        return new ShapeSnapshot(aText.ToString(), anObjects);
    }

    private static string Ref(in Shape theShape, Dictionary<TShape, int> theIndex, List<object?> theObjects)
    {
        if (theShape.IsNull)
        {
            return "null";
        }

        TShape aT = theShape.TShape()!;
        if (!theIndex.TryGetValue(aT, out int anIndex))
        {
            anIndex = theIndex.Count;
            theIndex.Add(aT, anIndex);
        }

        theObjects.Add(aT);
        return $"T{anIndex} {theShape.Orientation()} {Loc(theShape.Location())}";
    }

    private static string Loc(in Placement theLoc) => theLoc.IsIdentity ? "I" : theLoc.Transformation().ToString();

    /// <summary>
    /// Asserts that another shape graph has this one's values: the same structure, flags (apart from the frozen state),
    /// tolerances, ranges and locations, and the same curves, surfaces and meshes by identity, though its TShapes and
    /// representations are other objects.
    /// </summary>
    public void AssertSameValues(ShapeSnapshot theOther)
    {
        static string WithoutFrozen(string theText) => System.Text.RegularExpressions.Regex.Replace(theText, @" frozen=\w+", "");
        static bool IsGeometry(object? theObject) => theObject is null or Geometry3D or Geometry2D or Triangulation
                                                         or Polygon3D or Polygon2D or PolygonOnTriangulation;
        Assert.Equal(WithoutFrozen(myText), WithoutFrozen(theOther.myText));
        List<object?> aMine = myObjects.Where(IsGeometry).ToList();
        List<object?> aTheirs = theOther.myObjects.Where(IsGeometry).ToList();
        Assert.Equal(aMine.Count, aTheirs.Count);
        for (int i = 0; i < aMine.Count; i++)
        {
            Assert.Same(aMine[i], aTheirs[i]);
        }
    }

    /// <summary>Asserts that the shape graph is as recorded.</summary>
    public void AssertUnchanged(in Shape theShape)
    {
        ShapeSnapshot aNow = Of(theShape);
        Assert.Equal(myText, aNow.myText);
        Assert.Equal(myObjects.Count, aNow.myObjects.Count);
        for (int i = 0; i < myObjects.Count; i++)
        {
            Assert.Same(myObjects[i], aNow.myObjects[i]);
        }
    }
}
