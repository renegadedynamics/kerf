// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_MakeTriangulation.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_MakeTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Construct;

/// <summary>
/// Covers a closed polygonal contour (a wire, or points joined by a closing polygon) with planar faces: one face if
/// the contour is planar, else a shell of planar facets found by growing planar, non-self-intersecting pieces of the
/// contour.
/// </summary>
/// <remarks>
/// A maker (decision 454): <see cref="MakeShape.Shape"/> freezes the face or shell;
/// <c>InternalShape()</c> serves ported code. The faces hold the given wire's edges (or, for points, the polygon's),
/// which are only read: the new edges and wires are the maker's own.
/// </remarks>
public sealed class MakeTriangulation : MakeShape
{
    private readonly double _precision;
    private readonly Wire _wire = new();

    /// <summary>Triangulates the closed polygon through <paramref name="pnts"/>.</summary>
    public MakeTriangulation(NCollection_Array1<Point3D> pnts, double prec = 0.0)
    {
        _precision = (prec > 0.0) ? prec : global::Kerf.Engine.Foundation.Precision.Confusion();
        // Make polygonal wire from points
        MakePolygon mkPoly = new();
        for (int i = pnts.Lower(); i <= pnts.Upper(); i++)
        {
            mkPoly.Add(pnts[i]);
        }
        mkPoly.Close();
        mkPoly.Build();
        if (mkPoly.IsDone)
        {
            _wire = mkPoly.InternalWire();
            Build();
        }
    }

    /// <summary>Triangulates the closed polygonal wire.</summary>
    public MakeTriangulation(in Wire wire, double prec = 0.0)
    {
        _precision = (prec > 0.0) ? prec : global::Kerf.Engine.Foundation.Precision.Confusion();
        _wire = wire;
        Build();
    }

    /// <summary>Triangulates the wire (once).</summary>
    public override void Build(ProgressRange range = default)
    {
        if (_shape.IsNull)
        {
            // Triangulate polygonal wire
            if (!_wire.IsNull)
            {
                Triangulate(_wire);
            }
        }
    }

    /// <summary>True if a face or shell was built.</summary>
    public override bool IsDone => !_shape.IsNull;

    private void Triangulate(in Wire wire)
    {
        // Fill sequence of edges
        NCollection_HSequence<Shape>? theEdges = new();
        for (Iterator ite = new(wire); ite.More(); ite.Next())
        {
            theEdges.Append(ite.Value());
        }
        int nbEdges = theEdges.Length();

        if (nbEdges < 4)
        {
            AddFacet(wire);
        }
        else
        {
            // Fill array of end points
            EdgeAnalysis sae = new();
            NCollection_HArray1<Point3D>? theAPnt = new(1, nbEdges);
            for (int i = 1; i <= nbEdges; i++)
            {
                theAPnt.SetValue(i, Tool.Pnt(sae.FirstVertex(ShapeCasts.Edge(theEdges.Value(i)))));
            }

            Wire theNewWire = new();

            // Check planarity on array of points
            Xyz norm = new(0, 0, 0);
            if (CurveAnalysis.IsPlanar(theAPnt.Array1(), ref norm, _precision))
            {
                AddFacet(wire);
            }
            else
            {
                // Current contour is not planar - triangulate
                NCollection_Sequence<int> theSegments = new();
                int cindex = 1, seqFlag = 1;
                NCollection_Array1<int> llimits = new(1, 2), rlimits = new(1, 2);
                llimits.Init(nbEdges + 1);
                rlimits.Init(0);

                // Find all "good" planar segments
                while (seqFlag != 0)
                {
                    // Set up limits for current sequence
                    int llimit = llimits[seqFlag];
                    int rlimit = rlimits[seqFlag];
                    if (rlimit <= llimit)
                    {
                        rlimit += nbEdges;
                    }

                    // Check stop condition
                    if ((rlimit - llimit) > (nbEdges - 2))
                    {
                        break;
                    }

                    NCollection_Sequence<Point3D> theSPnt = new();

                    // Add 3 points of minimal facet
                    int lindex = (cindex == 1 ? nbEdges : cindex - 1);
                    int rindex = (cindex == nbEdges ? 1 : cindex + 1);
                    theSPnt.Append(theAPnt.Value(lindex));
                    theSPnt.Append(theAPnt.Value(cindex));
                    theSPnt.Append(theAPnt.Value(rindex));

                    // Try prepending points
                    bool isGood = true;
                    while (isGood)
                    {
                        cindex = (lindex == 1 ? nbEdges : lindex - 1);
                        if (cindex == rindex)
                        {
                            AddFacet(wire);
                            return;
                        }
                        int cid = cindex;
                        if (rlimit > nbEdges && cid <= llimit)
                        {
                            cid += nbEdges;
                        }
                        if (cid > llimit && cid < rlimit)
                        {
                            isGood = false;
                        }
                        if (isGood)
                        {
                            theSPnt.Prepend(theAPnt.Value(cindex));
                            isGood = IsRightContour(theSPnt, _precision);
                            if (isGood)
                            {
                                lindex = cindex;
                            }
                            else
                            {
                                theSPnt.Remove(1);
                            }
                        }
                    }

                    // Try appending points
                    isGood = true;
                    while (isGood)
                    {
                        cindex = (rindex == nbEdges ? 1 : rindex + 1);
                        if (cindex == lindex)
                        {
                            AddFacet(wire);
                            return;
                        }
                        int cid = cindex;
                        if (rlimit > nbEdges && cid <= llimit)
                        {
                            cid += nbEdges;
                        }
                        if (cid > llimit && cid < rlimit)
                        {
                            isGood = false;
                        }
                        if (isGood)
                        {
                            theSPnt.Append(theAPnt.Value(cindex));
                            isGood = IsRightContour(theSPnt, _precision);
                            if (isGood)
                            {
                                rindex = cindex;
                            }
                            else
                            {
                                theSPnt.Remove(theSPnt.Length());
                            }
                        }
                    }

                    // Record new planar segment
                    theSegments.Append(lindex);
                    theSegments.Append(rindex);

                    // Set up new limits
                    cindex = rindex;
                    rlimits[seqFlag] = rindex;
                    if (llimit > nbEdges)
                    {
                        llimits[seqFlag] = lindex;
                    }

                    // Set up sequence index
                    seqFlag = (seqFlag == 1) ? 2 : 1;
                }

                cindex = theSegments.Length();
                if (cindex % 4 == 0)
                {
                    Vector3D theBaseNorm = MeanNormal(theAPnt.Array1());
                    // Identify sequence of matching facets
                    int len = cindex / 2, lindex, rindex, seqlen, j;
                    double theC, theC1 = 0.0, theC2 = 0.0;
                    int ii;
                    for (ii = 0; ii < len; ii++)
                    {
                        // Compute normal collinearity for facet
                        lindex = theSegments[ii * 2 + 1];
                        rindex = theSegments[ii * 2 + 2];
                        seqlen = ((rindex > lindex) ? rindex - lindex + 1 : nbEdges - lindex + rindex + 1);
                        NCollection_Array1<Point3D> theArr = new(1, seqlen);
                        for (j = 1; j <= seqlen; j++)
                        {
                            theArr[j] = theAPnt.Value(lindex);
                            lindex++;
                            if (lindex > nbEdges)
                            {
                                lindex = 1;
                            }
                        }
                        theC = theBaseNorm * MeanNormal(theArr);
                        if (ii % 2 != 0)
                        {
                            theC2 += theC;
                        }
                        else
                        {
                            theC1 += theC;
                        }
                    }
                    int best1 = ((theC1 > theC2) ? 0 : 2);

                    // Add "best" planar facets
                    BrepBuilder b = new();
                    Wire theFacetWire = new();
                    Edge theLEdge = new(), theSLEdge = new();
                    len = cindex / 4;
                    // Check for special case - 1 shared line
                    bool special = (len == 2 && theSegments[best1 + 1] == theSegments[4 + best1 + 2]
                                    && theSegments[best1 + 2] == theSegments[4 + best1 + 1]);
                    int pindex = theSegments[(len - 1) * 4 + best1 + 2];
                    for (ii = 0; ii < len; ii++)
                    {
                        lindex = theSegments[ii * 4 + best1 + 1];
                        rindex = theSegments[ii * 4 + best1 + 2];
                        if (special && !theSLEdge.IsNull)
                        {
                            Shape localShape = theSLEdge.Reversed();
                            theLEdge = ShapeCasts.Edge(localShape);
                        }
                        else
                        {
                            Vertex v1 = sae.FirstVertex(ShapeCasts.Edge(theEdges.Value(lindex)));
                            Vertex v2 = sae.FirstVertex(ShapeCasts.Edge(theEdges.Value(rindex)));
                            v1.Orientation(Forward);
                            v2.Orientation(Reversed);
                            theLEdge = new MakeEdge(v1, v2).InternalEdge();
                            theSLEdge = theLEdge;
                        }
                        // Create new facet wire
                        b.MakeWire(ref theFacetWire);
                        b.Add(theFacetWire, theLEdge.Reversed());
                        int cur_edge = lindex;
                        while (cur_edge != rindex)
                        {
                            Edge theNEdge = ShapeCasts.Edge(theEdges.Value(cur_edge));
                            b.Add(theFacetWire, theNEdge);
                            if (cur_edge == nbEdges)
                            {
                                cur_edge = 1;
                            }
                            else
                            {
                                cur_edge++;
                            }
                        }
                        AddFacet(theFacetWire);
                        if (!special)
                        {
                            // Add edges to the new wire
                            if (theNewWire.IsNull)
                            {
                                b.MakeWire(ref theNewWire);
                            }
                            cur_edge = pindex;
                            while (cur_edge != lindex)
                            {
                                Edge theNEdge = ShapeCasts.Edge(theEdges.Value(cur_edge));
                                b.Add(theNewWire, theNEdge);
                                if (cur_edge == nbEdges)
                                {
                                    cur_edge = 1;
                                }
                                else
                                {
                                    cur_edge++;
                                }
                            }
                            b.Add(theNewWire, theLEdge);
                            pindex = rindex;
                        }
                    }
                }

                // Clear storage variables
                theEdges = null;
                theAPnt = null;

                // Triangulate the rest of edges
                if (!theNewWire.IsNull)
                {
                    Triangulate(theNewWire);
                }
            }
        }
    }

    private void AddFacet(in Wire wire)
    {
        if (wire.IsNull)
        {
            return;
        }

        // Fill sequence of points
        EdgeAnalysis sae = new();
        NCollection_Sequence<Point3D> pts = new();
        for (Iterator ite = new(wire); ite.More(); ite.Next())
        {
            pts.Append(Tool.Pnt(sae.FirstVertex(ShapeCasts.Edge(ite.Value()))));
        }
        int i, nbp = pts.Length();
        if (nbp < 3)
        {
            return;
        }

        // Create mean plane
        double cMod, maxMod = 0.0;
        Xyz maxVec = new(), normal = new(0, 0, 0);
        for (i = 1; i <= nbp; i++)
        {
            Xyz vb = pts[i].XYZ();
            Xyz v1 = pts[i == nbp ? 1 : i + 1].XYZ() - vb;
            cMod = v1.SquareModulus();
            if (cMod == 0.0)
            {
                continue;
            }
            else if (cMod > maxMod)
            {
                maxMod = cMod;
                maxVec = v1;
            }
            Xyz v2 = pts[i == 1 ? nbp : i - 1].XYZ() - vb;
            cMod = v2.SquareModulus();
            if (cMod == 0.0)
            {
                continue;
            }
            else if (cMod > maxMod)
            {
                maxMod = cMod;
                maxVec = v2;
            }
            normal += v1 ^ v2;
        }
        if (normal.SquareModulus() == 0.0)
        {
            if (maxMod == 0.0)
            {
                normal = new Xyz(0, 0, 1);
            }
            else if (maxVec.X() != 0.0)
            {
                normal = new Xyz(-maxVec.Y() / maxVec.X(), 1, 0);
            }
            else if (maxVec.Y() != 0.0)
            {
                normal = new Xyz(0, -maxVec.Z() / maxVec.Y(), 1);
            }
            else
            {
                normal = new Xyz(1, 0, 0);
            }
        }

        Plane3D pln = new(pts[1], new Direction3D(normal));
        PlaneSurface thePlane = new(pln);

        // Mean plane created - build face
        BrepBuilder b = new();
        Face theFace = new();
        b.MakeFace(ref theFace, thePlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        b.Add(theFace, wire);

        // Add new face to the shell
        if (_shape.IsNull)
        {
            _shape = theFace;
        }
        else
        {
            if (_shape.ShapeType() == ShapeType.Face)
            {
                Face firstFace = ShapeCasts.Face(_shape);
                Shell theShell = new();
                b.MakeShell(ref theShell);
                _shape = theShell;
                b.Add(_shape, firstFace);
            }
            b.Add(_shape, theFace);
        }
    }

    // True if the points (four or more) make a planar polygon that doesn't intersect itself.
    private static bool IsRightContour(NCollection_Sequence<Point3D> pts, double prec)
    {
        int len = pts.Length();
        if (len < 4)
        {
            return true;
        }

        NCollection_Array1<Point3D> thePts = new(1, len);
        int i;
        for (i = 1; i <= len; i++)
        {
            thePts[i] = pts[i];
        }

        Xyz norm = new(0, 0, 0);
        if (CurveAnalysis.IsPlanar(thePts, ref norm, prec))
        {
            // Make polygonal wire from points
            MakePolygon mkPoly = new();
            for (i = 1; i <= len; i++)
            {
                mkPoly.Add(thePts[i]);
            }
            mkPoly.Close();
            mkPoly.Build();
            if (mkPoly.IsDone)
            {
                // Create mean plane
                Xyz center = new(0, 0, 0);
                for (i = 1; i <= len; i++)
                {
                    center += thePts[i].XYZ();
                }
                center /= len;
                Point3D pc = new(center);
                Direction3D pd = new(norm);
                Plane3D pl = new(pc, pd);
                PlaneSurface thePlane = new(pl);

                BrepBuilder b = new();
                Face theFace = new();
                b.MakeFace(ref theFace, thePlane, global::Kerf.Engine.Foundation.Precision.Confusion());
                Wire theWire = mkPoly.InternalWire();
                b.Add(theFace, theWire);
                WireAnalysis saw = new(theWire, theFace, prec);
                return !saw.CheckSelfIntersection();
            }
        }

        return false;
    }

    // The mean normal of a closed polygon (unit, or null for fewer than three points).
    private static Vector3D MeanNormal(NCollection_Array1<Point3D> pts)
    {
        int len = pts.Length();
        if (len < 3)
        {
            return new Vector3D(0, 0, 0);
        }

        int i;
        Xyz theCenter = new(0, 0, 0);
        for (i = 1; i <= len; i++)
        {
            theCenter += pts[i].XYZ();
        }
        theCenter /= len;

        Xyz theNorm = new(0, 0, 0);
        for (i = 1; i <= len; i++)
        {
            Vector3D v1 = new(pts[i].XYZ() - theCenter);
            Vector3D v2 = new(pts[(i == len) ? 1 : i + 1].XYZ() - theCenter);
            theNorm += (v1 ^ v2).XYZ();
        }

        return new Vector3D(theNorm).Normalized();
    }
}
