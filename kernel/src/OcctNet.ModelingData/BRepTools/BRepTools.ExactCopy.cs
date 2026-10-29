// OCCT.NET: an exact copy for write, and the second half of a copy for write (docs/phase3-design.md, section 4.6):
// after an in-place operation on a copy, the parts it didn't change go back to being the original's own TShapes.
// Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

public static partial class BRepTools
{
    /// <summary>
    /// An exact structural copy of the shape for ported code that must write into it: each TShape copied once, with
    /// its flags (but Locked; Free cleared on children, as adding them clears it), its children at the same locations and orientations, and its BRep data (points,
    /// tolerances, edge flags, every curve and point representation, a face's surface, natural restriction and list of
    /// triangulations with the active one) in new representations sharing the frozen curves, surfaces, polygons and
    /// triangulations. Unlike <see cref="CopyForWrite"/> (BRepTools_Modifier with BRepTools_CopyModification, as
    /// BRepBuilderAPI_Copy), nothing is recomputed: pcurves keep their own ranges, planar faces get no stored pcurves,
    /// vertices keep their representations, edges keep their polygons on surfaces. So an in-place operation on the copy
    /// sees what it would see on the original, and <see cref="KeepUnchanged"/> can give back the original's own
    /// sub-shapes wherever it changed nothing. <paramref name="theHistory"/> records each vertex, edge, face and solid as
    /// modified into its copy. The copy is unfrozen.
    /// </summary>
    internal static TopoDS_Shape ExactCopyForWrite(in TopoDS_Shape theShape, out BRepTools_History theHistory)
    {
        theHistory = new BRepTools_History();
        if (theShape.IsNull())
        {
            return new TopoDS_Shape();
        }

        var aCopies = new Dictionary<TopoDS_TShape, TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        TopoDS_Shape aCopy = theShape;
        aCopy.TShape(exactCopy(theShape.TShape()!, aCopies));
        recordImages(theShape, aCopies, theHistory);
        return aCopy;
    }

    // Records each sub-shape of the shape (as placed in it) whose TShape has another image as modified into the image,
    // at the same place, as CopyForWrite's history does.
    private static void recordImages(in TopoDS_Shape theShape, Dictionary<TopoDS_TShape, TopoDS_TShape> theImages,
                                     BRepTools_History theHistory)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Shape aS = aMap[i];
            if (BRepTools_History.IsSupportedType(aS) && theImages.TryGetValue(aS.TShape()!, out TopoDS_TShape? anImage)
                && !ReferenceEquals(anImage, aS.TShape()))
            {
                TopoDS_Shape aNew = aS;
                aNew.TShape(anImage);
                theHistory.AddModified(aS, aNew);
            }
        }
    }

    private static TopoDS_TShape exactCopy(TopoDS_TShape theTShape, Dictionary<TopoDS_TShape, TopoDS_TShape> theCopies)
    {
        if (theCopies.TryGetValue(theTShape, out TopoDS_TShape? aDone))
        {
            return aDone;
        }

        // children first (an iterative walk isn't needed: shapes nest a few levels deep, and a compound nesting is
        // bounded by the stack check)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        TopoDS_TShape aCopy;
        switch (theTShape)
        {
            case BRep_TVertex aV:
            {
                var aNewV = new BRep_TVertex();
                aNewV.Pnt(aV.Pnt());
                aNewV.Tolerance(aV.Tolerance());
                NCollection_List<BRep_PointRepresentation> aPoints = aNewV.ChangePoints();
                for (var anIt = new NCollection_List<BRep_PointRepresentation>.Iterator(aV.Points()); anIt.More(); anIt.Next())
                {
                    BRep_PointRepresentation aPR = anIt.Value();
                    if (aPR.IsPointOnCurve())
                    {
                        aPoints.Append(new BRep_PointOnCurve(aPR.Parameter(), aPR.Curve(), aPR.Location()));
                    }
                    else if (aPR.IsPointOnCurveOnSurface())
                    {
                        aPoints.Append(new BRep_PointOnCurveOnSurface(aPR.Parameter(), aPR.PCurve(), aPR.Surface(), aPR.Location()));
                    }
                    else if (aPR.IsPointOnSurface())
                    {
                        aPoints.Append(new BRep_PointOnSurface(aPR.Parameter(), aPR.Parameter2(), aPR.Surface(), aPR.Location()));
                    }
                }
                aCopy = aNewV;
                break;
            }
            case BRep_TEdge anE:
            {
                var aNewE = new BRep_TEdge();
                aNewE.Tolerance(anE.Tolerance());
                aNewE.SameParameter(anE.SameParameter());
                aNewE.SameRange(anE.SameRange());
                aNewE.Degenerated(anE.Degenerated());
                NCollection_List<BRep_CurveRepresentation> aCurves = aNewE.ChangeCurves();
                for (var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(anE.Curves()); anIt.More(); anIt.Next())
                {
                    aCurves.Append(anIt.Value().Copy());
                }
                aCopy = aNewE;
                break;
            }
            case BRep_TFace aF:
            {
                var aNewF = (BRep_TFace)aF.EmptyCopy();
                aNewF.NaturalRestriction(aF.NaturalRestriction());
                if (aF.NbTriangulations() > 0)
                {
                    aNewF.Triangulations(new NCollection_List<Poly_Triangulation>(aF.Triangulations()), aF.ActiveTriangulation());
                }
                aCopy = aNewF;
                break;
            }
            default:
                aCopy = theTShape.EmptyCopy();
                break;
        }

        for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(theTShape.myShapes); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aChild = anIt.Value();
            TopoDS_TShape aChildCopy = exactCopy(aChild.TShape()!, theCopies);
            aChildCopy.Free(false); // as TopoDS_Builder::Add clears it (a frozen original keeps its own, design 4.4)
            aChild.TShape(aChildCopy);
            aCopy.myShapes.Append(aChild);
        }

        aCopy.Free(theTShape.Free());
        aCopy.Modified(theTShape.Modified());
        aCopy.Checked(theTShape.Checked());
        aCopy.Orientable(theTShape.Orientable());
        aCopy.Closed(theTShape.Closed());
        aCopy.Infinite(theTShape.Infinite());
        aCopy.Convex(theTShape.Convex());

        theCopies[theTShape] = aCopy;
        return aCopy;
    }

    /// <summary>
    /// The copy <paramref name="theCopy"/> of <paramref name="theOriginal"/> (made by <see cref="ExactCopyForWrite"/> and
    /// then changed in place) with each sub-shape that the change left equal to its original replaced by that
    /// original: so an operation that changes nothing gives back the original itself, and one that changes a few
    /// edges shares everything else with it, as the in-place operation would. <paramref name="theHistory"/> records
    /// each vertex, edge, face and solid of the original whose result is another TShape.
    /// </summary>
    /// <remarks>
    /// Two sub-shapes are equal when they have the same flags (other than the cache bits Free, Modified, Checked and
    /// Locked), the same children (each kept or replaced in turn, with the same locations and orientations) and the
    /// same BRep data: points, tolerances and edge flags bit for bit, and representations of the same types with the
    /// same parameters, ranges, UV points, continuities and locations on the same curves, surfaces, polygons and
    /// triangulations (by identity; a copy for write shares them). A sub-shape whose children don't correspond one to
    /// one with its original's is taken from the copy. Changed parts are the copy's own TShapes (unfrozen), holding
    /// the kept children in place of their copies.
    /// <para>With <paramref name="theIgnoreToleranceGuard"/>, a vertex whose tolerance was only raised by OCCT's rounding
    /// guard (BRepLib's UpdateTolerances adds <c>2 * Epsilon(tol)</c> to every vertex tolerance it computes, so a
    /// vertex as tolerant as its edges goes up by two units in the last place) counts as unchanged.</para>
    /// </remarks>
    internal static TopoDS_Shape KeepUnchanged(in TopoDS_Shape theOriginal, in TopoDS_Shape theCopy, out BRepTools_History theHistory,
                                               bool theIgnoreToleranceGuard = false)
    {
        theHistory = new BRepTools_History();
        if (theOriginal.IsNull() || theCopy.IsNull())
        {
            return theCopy;
        }

        var aResults = new Dictionary<TopoDS_TShape, TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        TopoDS_TShape aResult = keepUnchanged(theOriginal.TShape()!, theCopy.TShape()!, aResults, theIgnoreToleranceGuard);

        recordImages(theOriginal, aResults, theHistory);

        TopoDS_Shape aShape = theCopy;
        aShape.TShape(aResult);
        return aShape;
    }

    private static TopoDS_TShape keepUnchanged(TopoDS_TShape theOriginal, TopoDS_TShape theCopy,
                                               Dictionary<TopoDS_TShape, TopoDS_TShape> theResults, bool theIgnoreToleranceGuard)
    {
        if (theResults.TryGetValue(theOriginal, out TopoDS_TShape? aDone))
        {
            return aDone;
        }

        // the children, side by side
        var anOriginals = new List<TopoDS_Shape>();
        var aCopies = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(new TopoDS_Shape(theOriginal, new TopLoc_Location(), TopAbs_Orientation.TopAbs_FORWARD), false, false);
             anIt.More(); anIt.Next())
        {
            anOriginals.Add(anIt.Value());
        }
        for (var anIt = new TopoDS_Iterator(new TopoDS_Shape(theCopy, new TopLoc_Location(), TopAbs_Orientation.TopAbs_FORWARD), false, false);
             anIt.More(); anIt.Next())
        {
            aCopies.Add(anIt.Value());
        }

        bool isCorresponding = anOriginals.Count == aCopies.Count;
        for (int i = 0; isCorresponding && i < anOriginals.Count; i++)
        {
            isCorresponding = anOriginals[i].Orientation() == aCopies[i].Orientation()
                              && anOriginals[i].Location().IsEqual(aCopies[i].Location())
                              && anOriginals[i].ShapeType() == aCopies[i].ShapeType();
        }

        TopoDS_TShape aResult;
        if (!isCorresponding)
        {
            aResult = theCopy;
        }
        else
        {
            var aChildren = new TopoDS_TShape[aCopies.Count];
            bool isAllOriginal = true, isAllCopies = true;
            for (int i = 0; i < aCopies.Count; i++)
            {
                aChildren[i] = keepUnchanged(anOriginals[i].TShape()!, aCopies[i].TShape()!, theResults, theIgnoreToleranceGuard);
                isAllOriginal &= ReferenceEquals(aChildren[i], anOriginals[i].TShape());
                isAllCopies &= ReferenceEquals(aChildren[i], aCopies[i].TShape());
            }

            if (isAllOriginal && isSameData(theOriginal, theCopy, theIgnoreToleranceGuard))
            {
                aResult = theOriginal;
            }
            else if (isAllCopies)
            {
                aResult = theCopy;
            }
            else
            {
                // the copy (unfrozen, made for this write) with the kept children in place of its own
                if (theCopy.IsFrozen())
                {
                    throw new Standard_ImmutableObject("BRepTools::KeepUnchanged");
                }
                theCopy.myShapes.Clear();
                for (int i = 0; i < aCopies.Count; i++)
                {
                    TopoDS_Shape aChild = aCopies[i];
                    aChild.TShape(aChildren[i]);
                    theCopy.myShapes.Append(aChild);
                }
                aResult = theCopy;
            }
        }

        theResults[theOriginal] = aResult;
        return aResult;
    }

    // The same type, flags (but the cache bits) and BRep data, the geometry by identity.
    private static bool isSameData(TopoDS_TShape theA, TopoDS_TShape theB, bool theIgnoreToleranceGuard)
    {
        if (theA.GetType() != theB.GetType()
            || theA.Orientable() != theB.Orientable() || theA.Closed() != theB.Closed()
            || theA.Infinite() != theB.Infinite() || theA.Convex() != theB.Convex())
        {
            return false;
        }

        switch (theA)
        {
            case BRep_TVertex aVA:
            {
                var aVB = (BRep_TVertex)theB;
                double aTolA = aVA.Tolerance(), aTolB = aVB.Tolerance();
                bool isSameTol = aTolA.Equals(aTolB)
                                 || (theIgnoreToleranceGuard && aTolB > aTolA && aTolB <= aTolA + 2.0 * Epsilon(aTolA));
                if (!aVA.Pnt().Equals(aVB.Pnt()) || !isSameTol)
                {
                    return false;
                }

                NCollection_List<BRep_PointRepresentation> aLA = aVA.Points(), aLB = aVB.Points();
                if (aLA.Extent() != aLB.Extent())
                {
                    return false;
                }
                for (NCollection_List<BRep_PointRepresentation>.Iterator anItA = new(aLA), anItB = new(aLB);
                     anItA.More(); anItA.Next(), anItB.Next())
                {
                    if (!isSamePoint(anItA.Value(), anItB.Value()))
                    {
                        return false;
                    }
                }
                return true;
            }
            case BRep_TEdge anEA:
            {
                var anEB = (BRep_TEdge)theB;
                if (!anEA.Tolerance().Equals(anEB.Tolerance()) || anEA.SameParameter() != anEB.SameParameter()
                    || anEA.SameRange() != anEB.SameRange() || anEA.Degenerated() != anEB.Degenerated())
                {
                    return false;
                }

                NCollection_List<BRep_CurveRepresentation> aLA = anEA.Curves(), aLB = anEB.Curves();
                if (aLA.Extent() != aLB.Extent())
                {
                    return false;
                }
                for (NCollection_List<BRep_CurveRepresentation>.Iterator anItA = new(aLA), anItB = new(aLB);
                     anItA.More(); anItA.Next(), anItB.Next())
                {
                    if (!isSameCurve(anItA.Value(), anItB.Value()))
                    {
                        return false;
                    }
                }
                return true;
            }
            case BRep_TFace aFA:
            {
                var aFB = (BRep_TFace)theB;
                if (!ReferenceEquals(aFA.Surface(), aFB.Surface()) || !aFA.Location().IsEqual(aFB.Location())
                    || !aFA.Tolerance().Equals(aFB.Tolerance()) || aFA.NaturalRestriction() != aFB.NaturalRestriction()
                    || !ReferenceEquals(aFA.ActiveTriangulation(), aFB.ActiveTriangulation())
                    || aFA.NbTriangulations() != aFB.NbTriangulations())
                {
                    return false;
                }
                for (NCollection_List<Poly_Triangulation>.Iterator anItA = new(aFA.Triangulations()), anItB = new(aFB.Triangulations());
                     anItA.More(); anItA.Next(), anItB.Next())
                {
                    if (!ReferenceEquals(anItA.Value(), anItB.Value()))
                    {
                        return false;
                    }
                }
                return true;
            }
            default:
                return true;
        }
    }

    private static bool isSamePoint(BRep_PointRepresentation theA, BRep_PointRepresentation theB)
    {
        if (theA.GetType() != theB.GetType() || !theA.Parameter().Equals(theB.Parameter()) || !theA.Location().IsEqual(theB.Location()))
        {
            return false;
        }
        if (theA.IsPointOnCurve())
        {
            return ReferenceEquals(theA.Curve(), theB.Curve());
        }
        if (theA.IsPointOnCurveOnSurface())
        {
            return ReferenceEquals(theA.PCurve(), theB.PCurve()) && ReferenceEquals(theA.Surface(), theB.Surface());
        }
        if (theA.IsPointOnSurface())
        {
            return theA.Parameter2().Equals(theB.Parameter2()) && ReferenceEquals(theA.Surface(), theB.Surface());
        }
        return false;
    }

    private static bool isSameCurve(BRep_CurveRepresentation theA, BRep_CurveRepresentation theB)
    {
        if (theA.GetType() != theB.GetType() || !theA.Location().IsEqual(theB.Location()))
        {
            return false;
        }

        if (theA is BRep_GCurve aGA)
        {
            var aGB = (BRep_GCurve)theB;
            if (!aGA.First().Equals(aGB.First()) || !aGA.Last().Equals(aGB.Last()))
            {
                return false;
            }
        }

        switch (theA)
        {
            case BRep_Curve3D:
                return ReferenceEquals(theA.Curve3D(), theB.Curve3D());
            case BRep_CurveOnSurface aCA:
            {
                var aCB = (BRep_CurveOnSurface)theB;
                aCA.UVPoints(out gp_Pnt2d aPA1, out gp_Pnt2d aPA2);
                aCB.UVPoints(out gp_Pnt2d aPB1, out gp_Pnt2d aPB2);
                if (!ReferenceEquals(aCA.PCurve(), aCB.PCurve()) || !ReferenceEquals(aCA.Surface(), aCB.Surface())
                    || !aPA1.Equals(aPB1) || !aPA2.Equals(aPB2))
                {
                    return false;
                }
                if (aCA is BRep_CurveOnClosedSurface aCCA)
                {
                    var aCCB = (BRep_CurveOnClosedSurface)theB;
                    aCCA.UVPoints2(out aPA1, out aPA2);
                    aCCB.UVPoints2(out aPB1, out aPB2);
                    return ReferenceEquals(aCCA.PCurve2(), aCCB.PCurve2()) && aCCA.Continuity() == aCCB.Continuity()
                           && aPA1.Equals(aPB1) && aPA2.Equals(aPB2);
                }
                return true;
            }
            case BRep_CurveOn2Surfaces:
                return ReferenceEquals(theA.Surface(), theB.Surface()) && ReferenceEquals(theA.Surface2(), theB.Surface2())
                       && theA.Location2().IsEqual(theB.Location2()) && theA.Continuity() == theB.Continuity();
            case BRep_Polygon3D:
                return ReferenceEquals(theA.Polygon3D(), theB.Polygon3D());
            case BRep_PolygonOnClosedSurface:
                return ReferenceEquals(theA.Polygon(), theB.Polygon()) && ReferenceEquals(theA.Polygon2(), theB.Polygon2())
                       && ReferenceEquals(theA.Surface(), theB.Surface());
            case BRep_PolygonOnSurface:
                return ReferenceEquals(theA.Polygon(), theB.Polygon()) && ReferenceEquals(theA.Surface(), theB.Surface());
            case BRep_PolygonOnClosedTriangulation:
                return ReferenceEquals(theA.PolygonOnTriangulation(), theB.PolygonOnTriangulation())
                       && ReferenceEquals(theA.PolygonOnTriangulation2(), theB.PolygonOnTriangulation2())
                       && ReferenceEquals(theA.Triangulation(), theB.Triangulation());
            case BRep_PolygonOnTriangulation:
                return ReferenceEquals(theA.PolygonOnTriangulation(), theB.PolygonOnTriangulation())
                       && ReferenceEquals(theA.Triangulation(), theB.Triangulation());
            default:
                return false;
        }
    }
}
