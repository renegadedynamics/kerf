// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

    public static ShapeSnapshot Of(in TopoDS_Shape theShape)
    {
        var aText = new StringBuilder();
        var anObjects = new List<object?>();
        var anIndex = new Dictionary<TopoDS_TShape, int>(ReferenceEqualityComparer.Instance);
        aText.Append(CultureInfo.InvariantCulture, $"root {Ref(theShape, anIndex, anObjects)}\n");
        var aQueue = new Queue<TopoDS_TShape>();
        if (!theShape.IsNull())
        {
            aQueue.Enqueue(theShape.TShape()!);
        }

        var aDone = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        while (aQueue.Count > 0)
        {
            TopoDS_TShape aT = aQueue.Dequeue();
            if (!aDone.Add(aT))
            {
                continue;
            }

            aText.Append(CultureInfo.InvariantCulture,
                         $"T{anIndex[aT]} {aT.ShapeType()} free={aT.Free()} locked={aT.LockedFlag()} mod={aT.Modified()} chk={aT.Checked()} "
                         + $"ori={aT.Orientable()} closed={aT.Closed()} inf={aT.Infinite()} cvx={aT.Convex()} frozen={aT.IsFrozen()}\n");
            for (var anIt = new TopoDS_Iterator(new TopoDS_Shape(aT, new TopLoc_Location(), TopAbs_Orientation.TopAbs_FORWARD), false, false);
                 anIt.More(); anIt.Next())
            {
                TopoDS_Shape aChild = anIt.Value();
                aText.Append(CultureInfo.InvariantCulture, $"  child {Ref(aChild, anIndex, anObjects)}\n");
                aQueue.Enqueue(aChild.TShape()!);
            }

            switch (aT)
            {
                case BRep_TVertex aV:
                    aText.Append(CultureInfo.InvariantCulture, $"  pnt {aV.Pnt()} tol {aV.Tolerance():R}\n");
                    foreach (BRep_PointRepresentation aPR in aV.PointRepresentations())
                    {
                        anObjects.Add(aPR);
                        aText.Append(CultureInfo.InvariantCulture, $"  prep {aPR.GetType().Name} {aPR.Parameter():R} {Loc(aPR.Location())}\n");
                    }

                    break;
                case BRep_TEdge anE:
                    aText.Append(CultureInfo.InvariantCulture,
                                 $"  tol {anE.Tolerance():R} sp={anE.SameParameter()} sr={anE.SameRange()} deg={anE.Degenerated()}\n");
                    foreach (BRep_CurveRepresentation aCR in anE.CurveRepresentations())
                    {
                        anObjects.Add(aCR);
                        aText.Append(CultureInfo.InvariantCulture, $"  crep {aCR.GetType().Name} {Loc(aCR.Location())} frozen={aCR.IsFrozen()}");
                        if (aCR is BRep_GCurve aGC)
                        {
                            aText.Append(CultureInfo.InvariantCulture, $" [{aGC.First():R}, {aGC.Last():R}]");
                            anObjects.Add(aGC.IsCurve3D() ? aGC.Curve3D() : null);
                            anObjects.Add(aGC.IsCurveOnSurface() ? aGC.PCurve() : null);
                            anObjects.Add(aGC.IsCurveOnSurface() ? aGC.Surface() : null);
                        }

                        aText.Append('\n');
                    }

                    break;
                case BRep_TFace aF:
                    aText.Append(CultureInfo.InvariantCulture,
                                 $"  tol {aF.Tolerance():R} nat={aF.NaturalRestriction()} {Loc(aF.Location())} ntri={aF.NbTriangulations()}\n");
                    anObjects.Add(aF.Surface());
                    anObjects.Add(aF.ActiveTriangulation());
                    foreach (Poly_Triangulation aTri in aF.TriangulationList())
                    {
                        anObjects.Add(aTri);
                    }

                    break;
            }
        }

        return new ShapeSnapshot(aText.ToString(), anObjects);
    }

    private static string Ref(in TopoDS_Shape theShape, Dictionary<TopoDS_TShape, int> theIndex, List<object?> theObjects)
    {
        if (theShape.IsNull())
        {
            return "null";
        }

        TopoDS_TShape aT = theShape.TShape()!;
        if (!theIndex.TryGetValue(aT, out int anIndex))
        {
            anIndex = theIndex.Count;
            theIndex.Add(aT, anIndex);
        }

        theObjects.Add(aT);
        return $"T{anIndex} {theShape.Orientation()} {Loc(theShape.Location())}";
    }

    private static string Loc(in TopLoc_Location theLoc) => theLoc.IsIdentity() ? "I" : theLoc.Transformation().ToString();

    /// <summary>
    /// Asserts that another shape graph has this one's values: the same structure, flags (apart from the frozen state),
    /// tolerances, ranges and locations, and the same curves, surfaces and meshes by identity, though its TShapes and
    /// representations are other objects.
    /// </summary>
    public void AssertSameValues(ShapeSnapshot theOther)
    {
        static string WithoutFrozen(string theText) => System.Text.RegularExpressions.Regex.Replace(theText, @" frozen=\w+", "");
        static bool IsGeometry(object? theObject) => theObject is null or Geom_Geometry or Geom2d_Geometry or Poly_Triangulation
                                                         or Poly_Polygon3D or Poly_Polygon2D or Poly_PolygonOnTriangulation;
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
    public void AssertUnchanged(in TopoDS_Shape theShape)
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
