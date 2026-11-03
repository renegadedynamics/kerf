// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The copy a Draw-lite command writes into where OCCT's command changes its shape in place (sameparameter,
/// updatetolerance, encoderegularity and mkedgecurve without a result name): every TShape copied once, with its
/// flags, its representations (polygons and triangulations included) and its children at the same locations and
/// orientations, sharing the frozen geometry. The copy has the shape's graph, so the shape OCCT's in-place algorithm
/// makes of it writes the same BRep text as OCCT's.
/// </summary>
/// <remarks>
/// BRepTools.CopyForWrite (BRepTools_Modifier with BRepTools_CopyModification) rebuilds the graph: shared sub-shapes
/// at several locations come out with other locations, so its text differs from the in-place result's.
/// </remarks>
internal static class DrawShapeCopy
{
    public static Shape Copy(in Shape theShape)
        => Copy(theShape, new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance));

    private static Shape Copy(in Shape theShape, Dictionary<TShape, TShape> theMap)
    {
        if (theShape.IsNull)
        {
            return theShape;
        }

        TShape aTShape = theShape.TShape()!;
        if (!theMap.TryGetValue(aTShape, out TShape? aCopy))
        {
            aCopy = CopyNode(aTShape);
            foreach (Shape aChild in aTShape._shapes)
            {
                aCopy._shapes.Append(Copy(aChild, theMap));
            }

            theMap.Add(aTShape, aCopy);
        }

        Shape aResult = theShape;
        aResult.TShape(aCopy);
        return aResult;
    }

    /// <summary>A copy of the TShape without its children: its flags and its representations.</summary>
    internal static TShape CopyNode(TShape theTShape)
    {
        TShape aCopy = CopyRepresentations(theTShape);
        aCopy.Free(theTShape.Free());
        aCopy.Modified(theTShape.Modified());
        aCopy.Checked(theTShape.Checked());
        aCopy.Orientable(theTShape.Orientable());
        aCopy.Closed(theTShape.Closed());
        aCopy.Infinite(theTShape.Infinite());
        aCopy.Convex(theTShape.Convex());
        return aCopy;
    }

    private static TShape CopyRepresentations(TShape theTShape)
    {
        switch (theTShape)
        {
            case BrepTEdge anEdge:
            {
                var aCopy = new BrepTEdge();
                aCopy.Tolerance(anEdge.Tolerance());
                foreach (CurveRepresentation aRep in anEdge.Curves())
                {
                    aCopy.ChangeCurves().Append(aRep.Copy());
                }

                aCopy.Degenerated(anEdge.Degenerated());
                aCopy.SameParameter(anEdge.SameParameter());
                aCopy.SameRange(anEdge.SameRange());
                return aCopy;
            }

            case BrepTFace aFace:
            {
                var aCopy = (BrepTFace)aFace.EmptyCopy();
                aCopy.NaturalRestriction(aFace.NaturalRestriction());
                if (aFace.TriangulationCount > 0)
                {
                    aCopy.Triangulations(new NCollection_List<Triangulation>(aFace.Triangulations()), aFace.ActiveTriangulation());
                }

                return aCopy;
            }

            case BrepTVertex aVertex:
            {
                var aCopy = (BrepTVertex)aVertex.EmptyCopy();
                foreach (PointRepresentation aRep in aVertex.Points())
                {
                    aCopy.ChangePoints().Append(aRep);
                }

                return aCopy;
            }

            default:
                return theTShape.EmptyCopy();
        }
    }
}
