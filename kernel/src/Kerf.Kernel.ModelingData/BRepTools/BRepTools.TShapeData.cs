// Kerf: the BRep data of a TShape, copied and compared one TShape at a time (docs/phase3-design.md, section 4.6):
// the working overlay's working copies are made with CopyData and CopyFlags, and a working copy left equal to its
// original (IsSameData) gives back the original (decisions 416, 1361). Not a port of an OCCT file. Kerf (unit 10H6):
// the exact copy for write and KeepUnchanged, which copy at entry used, are gone with it.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

namespace Kerf.Engine.Topology.Tools;

internal static partial class BrepTools
{
    // A new TShape with the BRep data of theTShape (its representations copied, its geometry shared), without its
    // children and flags: the working overlay's working copies.
    internal static TShape CopyData(TShape tShape)
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

    internal static void CopyFlags(TShape theFrom, TShape to)
    {
        to.Free(theFrom.Free());
        to.Modified(theFrom.Modified());
        to.Checked(theFrom.Checked());
        to.Orientable(theFrom.Orientable());
        to.Closed(theFrom.Closed());
        to.Infinite(theFrom.Infinite());
        to.Convex(theFrom.Convex());
        to._compoundNesting = theFrom._compoundNesting;
        if (theFrom is TCompound fromCompound && to is TCompound toCompound)
        {
            toCompound._walkLeaves = fromCompound._walkLeaves;
        }
    }

    // The same type, flags (but the cache bits) and BRep data, the geometry by identity.
    internal static bool IsSameData(TShape a, TShape b, bool ignoreToleranceGuard)
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
}
