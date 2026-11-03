// OCCT.NET: an exact copy for write, and the second half of a copy for write (docs/phase3-design.md, section 4.6):
// after an in-place operation on a copy, the parts it didn't change go back to being the original's own TShapes.
// Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology.Tools;

public static partial class BrepTools
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
    /// sub-shapes wherever it changed nothing. <paramref name="history"/> records each vertex, edge, face and solid as
    /// modified into its copy. The copy is unfrozen.
    /// </summary>
    internal static Shape ExactCopyForWrite(in Shape shape, out History history) =>
        ExactCopyForWrite(shape, out history, out _);

    /// <summary>
    /// True if the shape's graph holds a frozen TShape (the shape's own or a sub-shape's): an in-place operation must
    /// then work on <see cref="ExactCopyForWrite(in Shape, out History)"/> of it. Each TShape is
    /// visited once; a frozen TShape's sub-graph is frozen, so the walk stops there.
    /// </summary>
    internal static bool HoldsFrozen(in Shape shape)
    {
        TShape? root = shape.TShape();
        if (root is null)
        {
            return false;
        }

        var visited = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var stack = new Stack<TShape>();
        stack.Push(root);
        while (stack.Count > 0)
        {
            TShape tShape = stack.Pop();
            if (tShape.IsFrozen)
            {
                return true;
            }

            if (!visited.Add(tShape))
            {
                continue;
            }

            for (var it = new Iterator(new Shape(tShape, new Placement(), Orientation.Forward));
                 it.More(); it.Next())
            {
                TShape? child = it.Value().TShape();
                if (child is not null)
                {
                    stack.Push(child);
                }
            }
        }

        return false;
    }

    /// <summary>
    /// <see cref="ExactCopyForWrite(in Shape, out History)"/>, with the copy of each TShape of the
    /// shape in <paramref name="copies"/> (by reference), for <see cref="KeepUnchangedIn"/>.
    /// </summary>
    internal static Shape ExactCopyForWrite(in Shape shape, out History history,
                                                   out Dictionary<TShape, TShape> copies)
    {
        history = new History();
        var aCopies = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        copies = aCopies;
        if (shape.IsNull)
        {
            return new Shape();
        }

        Shape copy = shape;
        copy.TShape(ExactCopy(shape.TShape()!, aCopies));
        RecordImages(shape, aCopies, history);
        return copy;
    }

    /// <summary>
    /// <see cref="ExactCopyForWrite(in Shape, out History)"/> that goes on filling
    /// <paramref name="copies"/>: a TShape already a key there is replaced by its value (an earlier copy, or itself
    /// if the caller maps it to itself to keep it), the others are copied and added. So several arguments of one
    /// operation copied one after the other share their common sub-shapes' copies (LocOpe's working shapes, unit 7H).
    /// </summary>
    internal static Shape ExactCopyForWrite(in Shape shape, Dictionary<TShape, TShape> copies)
    {
        if (shape.IsNull)
        {
            return new Shape();
        }

        Shape copy = shape;
        copy.TShape(ExactCopy(shape.TShape()!, copies));
        return copy;
    }

    // Records each sub-shape of the shape (as placed in it) whose TShape has another image as modified into the image,
    // at the same place, as CopyForWrite's history does.
    private static void RecordImages(in Shape shape, Dictionary<TShape, TShape> images,
                                     History history)
    {
        var map = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, map);
        for (int i = 1; i <= map.Extent(); i++)
        {
            Shape s = map[i];
            if (History.IsSupportedType(s) && images.TryGetValue(s.TShape()!, out TShape? image)
                && !ReferenceEquals(image, s.TShape()))
            {
                Shape aNew = s;
                aNew.TShape(image);
                history.AddModified(s, aNew);
            }
        }
    }

    private static TShape ExactCopy(TShape tShape, Dictionary<TShape, TShape> copies)
    {
        if (copies.TryGetValue(tShape, out TShape? done))
        {
            return done;
        }

        // children first (an iterative walk isn't needed: shapes nest a few levels deep, and a compound nesting is
        // bounded by the stack check)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        TShape copy = CopyData(tShape);
        for (var it = new NCollection_List<Shape>.Iterator(tShape._shapes); it.More(); it.Next())
        {
            Shape child = it.Value();
            TShape childCopy = ExactCopy(child.TShape()!, copies);
            childCopy.Free(false); // as TopoDS_Builder::Add clears it (a frozen original keeps its own, design 4.4)
            child.TShape(childCopy);
            copy._shapes.Append(child);
        }

        CopyFlags(tShape, copy);
        copies[tShape] = copy;
        return copy;
    }

    // A new TShape with the BRep data of theTShape (its geometry shared), without its children and flags.
    private static TShape CopyData(TShape tShape)
    {
        TShape copy;
        switch (tShape)
        {
            case BrepTVertex v:
            {
                var newV = new BrepTVertex();
                newV.Pnt(v.Pnt());
                newV.Tolerance(v.Tolerance());
                NCollection_List<PointRepresentation> points = newV.ChangePoints();
                for (var it = new NCollection_List<PointRepresentation>.Iterator(v.Points()); it.More(); it.Next())
                {
                    PointRepresentation pr = it.Value();
                    if (pr.IsPointOnCurve())
                    {
                        points.Append(new PointOnCurve(pr.Parameter(), pr.Curve(), pr.Location()));
                    }
                    else if (pr.IsPointOnCurveOnSurface())
                    {
                        points.Append(new PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), pr.Location()));
                    }
                    else if (pr.IsPointOnSurface())
                    {
                        points.Append(new PointOnSurface(pr.Parameter(), pr.Parameter2(), pr.Surface(), pr.Location()));
                    }
                }
                copy = newV;
                break;
            }
            case BrepTEdge e:
            {
                var newE = new BrepTEdge();
                newE.Tolerance(e.Tolerance());
                newE.SameParameter(e.SameParameter());
                newE.SameRange(e.SameRange());
                newE.Degenerated(e.Degenerated());
                NCollection_List<CurveRepresentation> curves = newE.ChangeCurves();
                for (var it = new NCollection_List<CurveRepresentation>.Iterator(e.Curves()); it.More(); it.Next())
                {
                    curves.Append(it.Value().Copy());
                }
                copy = newE;
                break;
            }
            case BrepTFace f:
            {
                var newF = (BrepTFace)f.EmptyCopy();
                newF.NaturalRestriction(f.NaturalRestriction());
                if (f.TriangulationCount > 0)
                {
                    newF.Triangulations(new NCollection_List<Triangulation>(f.Triangulations()), f.ActiveTriangulation());
                }
                copy = newF;
                break;
            }
            default:
                copy = tShape.EmptyCopy();
                break;
        }
        return copy;
    }

    private static void CopyFlags(TShape theFrom, TShape to)
    {
        to.Free(theFrom.Free());
        to.Modified(theFrom.Modified());
        to.Checked(theFrom.Checked());
        to.Orientable(theFrom.Orientable());
        to.Closed(theFrom.Closed());
        to.Infinite(theFrom.Infinite());
        to.Convex(theFrom.Convex());
    }

    /// <summary>
    /// The copy <paramref name="copy"/> of <paramref name="original"/> (made by <see cref="ExactCopyForWrite(in Shape, out History)"/> and
    /// then changed in place) with each sub-shape that the change left equal to its original replaced by that
    /// original: so an operation that changes nothing gives back the original itself, and one that changes a few
    /// edges shares everything else with it, as the in-place operation would. <paramref name="history"/> records
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
    /// <para>With <paramref name="ignoreToleranceGuard"/>, a vertex whose tolerance was only raised by OCCT's rounding
    /// guard (BRepLib's UpdateTolerances adds <c>2 * Epsilon(tol)</c> to every vertex tolerance it computes, so a
    /// vertex as tolerant as its edges goes up by two units in the last place) counts as unchanged.</para>
    /// </remarks>
    internal static Shape KeepUnchanged(in Shape original, in Shape copy, out History history,
                                               bool ignoreToleranceGuard = false)
    {
        history = new History();
        if (original.IsNull || copy.IsNull)
        {
            return copy;
        }

        var results = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        TShape result = KeepUnchangedCore(original.TShape()!, copy.TShape()!, results, ignoreToleranceGuard);

        RecordImages(original, results, history);

        Shape shape = copy;
        shape.TShape(result);
        return shape;
    }

    private static TShape KeepUnchangedCore(TShape original, TShape copy,
                                               Dictionary<TShape, TShape> results, bool ignoreToleranceGuard,
                                               bool toLeaveCopies = false)
    {
        if (results.TryGetValue(original, out TShape? done))
        {
            return done;
        }

        // the children, side by side
        var originals = new List<Shape>();
        var copies = new List<Shape>();
        for (var it = new Iterator(new Shape(original, new Placement(), Orientation.Forward), false, false);
             it.More(); it.Next())
        {
            originals.Add(it.Value());
        }
        for (var it = new Iterator(new Shape(copy, new Placement(), Orientation.Forward), false, false);
             it.More(); it.Next())
        {
            copies.Add(it.Value());
        }

        bool isCorresponding = originals.Count == copies.Count;
        for (int i = 0; isCorresponding && i < originals.Count; i++)
        {
            isCorresponding = originals[i].Orientation() == copies[i].Orientation()
                              && originals[i].Location().IsEqual(copies[i].Location())
                              && originals[i].ShapeType() == copies[i].ShapeType();
        }

        TShape result;
        if (!isCorresponding)
        {
            result = copy;
        }
        else
        {
            var children = new TShape[copies.Count];
            bool isAllOriginal = true, isAllCopies = true;
            for (int i = 0; i < copies.Count; i++)
            {
                children[i] = KeepUnchangedCore(originals[i].TShape()!, copies[i].TShape()!, results, ignoreToleranceGuard,
                                             toLeaveCopies);
                isAllOriginal &= ReferenceEquals(children[i], originals[i].TShape());
                isAllCopies &= ReferenceEquals(children[i], copies[i].TShape());
            }

            if (isAllOriginal && IsSameData(original, copy, ignoreToleranceGuard))
            {
                result = original;
            }
            else if (isAllCopies)
            {
                result = copy;
            }
            else if (toLeaveCopies)
            {
                // a new TShape with the copy's data and the kept children; the copy stays as it is
                result = CopyData(copy);
                for (int i = 0; i < copies.Count; i++)
                {
                    Shape child = copies[i];
                    child.TShape(children[i]);
                    result._shapes.Append(child);
                }
                CopyFlags(copy, result);
            }
            else
            {
                // the copy (unfrozen, made for this write) with the kept children in place of its own
                if (copy.IsFrozen)
                {
                    throw new ImmutableObjectException("BRepTools::KeepUnchanged");
                }
                copy._shapes.Clear();
                for (int i = 0; i < copies.Count; i++)
                {
                    Shape child = copies[i];
                    child.TShape(children[i]);
                    copy._shapes.Append(child);
                }
                result = copy;
            }
        }

        results[original] = result;
        return result;
    }

    // The same type, flags (but the cache bits) and BRep data, the geometry by identity.
    private static bool IsSameData(TShape a, TShape b, bool ignoreToleranceGuard)
    {
        if (a.GetType() != b.GetType()
            || a.Orientable() != b.Orientable() || a.Closed() != b.Closed()
            || a.Infinite() != b.Infinite() || a.Convex() != b.Convex())
        {
            return false;
        }

        switch (a)
        {
            case BrepTVertex va:
            {
                var vb = (BrepTVertex)b;
                double tolA = va.Tolerance(), tolB = vb.Tolerance();
                bool isSameTol = tolA.Equals(tolB)
                                 || (ignoreToleranceGuard && tolB > tolA && tolB <= tolA + 2.0 * Epsilon(tolA));
                if (!va.Pnt().Equals(vb.Pnt()) || !isSameTol)
                {
                    return false;
                }

                NCollection_List<PointRepresentation> la = va.Points(), lb = vb.Points();
                if (la.Extent() != lb.Extent())
                {
                    return false;
                }
                for (NCollection_List<PointRepresentation>.Iterator itA = new(la), itB = new(lb);
                     itA.More(); itA.Next(), itB.Next())
                {
                    if (!IsSamePoint(itA.Value(), itB.Value()))
                    {
                        return false;
                    }
                }
                return true;
            }
            case BrepTEdge ea:
            {
                var eb = (BrepTEdge)b;
                if (!ea.Tolerance().Equals(eb.Tolerance()) || ea.SameParameter() != eb.SameParameter()
                    || ea.SameRange() != eb.SameRange() || ea.Degenerated() != eb.Degenerated())
                {
                    return false;
                }

                NCollection_List<CurveRepresentation> la = ea.Curves(), lb = eb.Curves();
                if (la.Extent() != lb.Extent())
                {
                    return false;
                }
                for (NCollection_List<CurveRepresentation>.Iterator itA = new(la), itB = new(lb);
                     itA.More(); itA.Next(), itB.Next())
                {
                    if (!IsSameCurve(itA.Value(), itB.Value()))
                    {
                        return false;
                    }
                }
                return true;
            }
            case BrepTFace fa:
            {
                var fb = (BrepTFace)b;
                if (!ReferenceEquals(fa.Surface(), fb.Surface()) || !fa.Location().IsEqual(fb.Location())
                    || !fa.Tolerance().Equals(fb.Tolerance()) || fa.NaturalRestriction() != fb.NaturalRestriction()
                    || !ReferenceEquals(fa.ActiveTriangulation(), fb.ActiveTriangulation())
                    || fa.TriangulationCount != fb.TriangulationCount)
                {
                    return false;
                }
                for (NCollection_List<Triangulation>.Iterator itA = new(fa.Triangulations()), itB = new(fb.Triangulations());
                     itA.More(); itA.Next(), itB.Next())
                {
                    if (!ReferenceEquals(itA.Value(), itB.Value()))
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

    private static bool IsSamePoint(PointRepresentation a, PointRepresentation b)
    {
        if (a.GetType() != b.GetType() || !a.Parameter().Equals(b.Parameter()) || !a.Location().IsEqual(b.Location()))
        {
            return false;
        }
        if (a.IsPointOnCurve())
        {
            return ReferenceEquals(a.Curve(), b.Curve());
        }
        if (a.IsPointOnCurveOnSurface())
        {
            return ReferenceEquals(a.PCurve(), b.PCurve()) && ReferenceEquals(a.Surface(), b.Surface());
        }
        if (a.IsPointOnSurface())
        {
            return a.Parameter2().Equals(b.Parameter2()) && ReferenceEquals(a.Surface(), b.Surface());
        }
        return false;
    }

    private static bool IsSameCurve(CurveRepresentation a, CurveRepresentation b)
    {
        if (a.GetType() != b.GetType() || !a.Location().IsEqual(b.Location()))
        {
            return false;
        }

        if (a is GCurve ga)
        {
            var gb = (GCurve)b;
            if (!ga.First().Equals(gb.First()) || !ga.Last().Equals(gb.Last()))
            {
                return false;
            }
        }

        switch (a)
        {
            case Curve3DRepresentation:
                return ReferenceEquals(a.Curve3D(), b.Curve3D());
            case CurveOnSurfaceRepresentation ca:
            {
                var cb = (CurveOnSurfaceRepresentation)b;
                ca.UVPoints(out Point2D pa1, out Point2D pa2);
                cb.UVPoints(out Point2D pb1, out Point2D pb2);
                if (!ReferenceEquals(ca.PCurve(), cb.PCurve()) || !ReferenceEquals(ca.Surface(), cb.Surface())
                    || !pa1.Equals(pb1) || !pa2.Equals(pb2))
                {
                    return false;
                }
                if (ca is CurveOnClosedSurface cca)
                {
                    var ccb = (CurveOnClosedSurface)b;
                    cca.UVPoints2(out pa1, out pa2);
                    ccb.UVPoints2(out pb1, out pb2);
                    return ReferenceEquals(cca.PCurve2(), ccb.PCurve2()) && cca.Continuity() == ccb.Continuity()
                           && pa1.Equals(pb1) && pa2.Equals(pb2);
                }
                return true;
            }
            case CurveOn2Surfaces:
                return ReferenceEquals(a.Surface(), b.Surface()) && ReferenceEquals(a.Surface2(), b.Surface2())
                       && a.Location2().IsEqual(b.Location2()) && a.Continuity() == b.Continuity();
            case Polygon3DRepresentation:
                return ReferenceEquals(a.Polygon3D(), b.Polygon3D());
            case PolygonOnClosedSurface:
                return ReferenceEquals(a.Polygon(), b.Polygon()) && ReferenceEquals(a.Polygon2(), b.Polygon2())
                       && ReferenceEquals(a.Surface(), b.Surface());
            case PolygonOnSurface:
                return ReferenceEquals(a.Polygon(), b.Polygon()) && ReferenceEquals(a.Surface(), b.Surface());
            case PolygonOnClosedTriangulation:
                return ReferenceEquals(a.PolygonOnTriangulation(), b.PolygonOnTriangulation())
                       && ReferenceEquals(a.PolygonOnTriangulation2(), b.PolygonOnTriangulation2())
                       && ReferenceEquals(a.Triangulation(), b.Triangulation());
            case PolygonOnTriangulationRepresentation:
                return ReferenceEquals(a.PolygonOnTriangulation(), b.PolygonOnTriangulation())
                       && ReferenceEquals(a.Triangulation(), b.Triangulation());
            default:
                return false;
        }
    }

    /// <summary>
    /// <see cref="KeepUnchanged"/> for an operation that, working on the copy <paramref name="copy"/> of
    /// <paramref name="original"/> (made by <see cref="ExactCopyForWrite(in Shape, out History, out Dictionary{TShape, TShape})"/>
    /// with the copies <paramref name="copies"/>), built a new shape <paramref name="result"/> holding the copy's
    /// sub-shapes, as an in-place sweep holds its profile's: each copy the operation left equal to its original is
    /// replaced by that original in the result's unfrozen graph too (and <paramref name="result"/> itself if it is
    /// one). The returned map gives, for each copied TShape, the TShape that stands for it now (the original or the
    /// copy, or a new one); <paramref name="history"/> records the original's sub-shapes whose result is another
    /// TShape.
    /// <para>A changed copy holding kept children is given those children in place of its own, or, with
    /// <paramref name="toLeaveCopies"/>, replaced by a new TShape holding them (its data and flags copied), so the
    /// copy's own graph stays as the operation left it: for an operation that still reads its copy afterwards (a sweep
    /// answering its history from its generating shape).</para>
    /// </summary>
    internal static Dictionary<TShape, TShape> KeepUnchangedIn(in Shape original, in Shape copy,
                                                                          Dictionary<TShape, TShape> copies,
                                                                          ref Shape result,
                                                                          out History history,
                                                                          bool toLeaveCopies = false)
    {
        history = new History();
        var final = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        if (original.IsNull || copy.IsNull)
        {
            return final;
        }

        var results = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        KeepUnchangedCore(original.TShape()!, copy.TShape()!, results, false, toLeaveCopies);
        RecordImages(original, results, history);

        var replaced = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        foreach (KeyValuePair<TShape, TShape> oc in copies)
        {
            TShape now = results.TryGetValue(oc.Key, out TShape? kept) ? kept : oc.Value;
            final[oc.Value] = now;
            if (!ReferenceEquals(now, oc.Value))
            {
                replaced[oc.Value] = now;
            }
        }

        if (!result.IsNull && replaced.Count != 0)
        {
            var visited = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
            SubstituteTShapes(result.TShape()!, replaced, visited);
            if (replaced.TryGetValue(result.TShape()!, out TShape? top))
            {
                result.TShape(top);
            }
        }
        return final;
    }

    // Replaces, in the unfrozen TShapes reachable from theTShape, each child whose TShape is a key of the map by the
    // map's value, keeping its place, location and orientation. Frozen TShapes (inputs) are left as they are.
    private static void SubstituteTShapes(TShape tShape, Dictionary<TShape, TShape> map,
                                          HashSet<TShape> visited)
    {
        if (tShape.IsFrozen || !visited.Add(tShape))
        {
            return;
        }

        RuntimeHelpers.EnsureSufficientExecutionStack();
        var children = new List<Shape>();
        bool isChanged = false;
        for (NCollection_List<Shape>.Iterator it = new(tShape._shapes); it.More(); it.Next())
        {
            Shape child = it.Value();
            if (map.TryGetValue(child.TShape()!, out TShape? aNew))
            {
                child.TShape(aNew);
                isChanged = true;
            }
            else
            {
                SubstituteTShapes(child.TShape()!, map, visited);
            }
            children.Add(child);
        }

        if (isChanged)
        {
            tShape._shapes.Clear();
            foreach (Shape child in children)
            {
                tShape._shapes.Append(child);
            }
        }
    }
}
