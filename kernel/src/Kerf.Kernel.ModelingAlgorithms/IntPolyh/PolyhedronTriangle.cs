// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Triangle.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Triangle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// A triangle of three IntPolyh points and three IntPolyh edges (by index into the arrays of its surface), with its
/// deflection from the surface and its cached bounding box.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's default triangle has points and edges -1, orientations 0, intersection possible, a void box;
/// <c>default(IntPolyh_Triangle)</c> is all zeros, so ported code writes <c>new IntPolyh_Triangle()</c>.
/// </para>
/// <para>
/// The refining members (<c>MiddleRefinement</c>, <c>MultipleMiddleRefinement</c>) append triangles, points and edges to
/// the arrays and update the triangle itself: they are called on the array's item (<c>TTriangles[i].MiddleRefinement</c>),
/// whose storage never moves (see <see cref="PolyhedronArray{T}"/>).
/// </para>
/// </remarks>
internal partial struct PolyhedronTriangle : IEquatable<PolyhedronTriangle>
{
    private const double SquareMyConfusionPrecision = 10.0e-24;

    /// <summary>Three indices (C++'s <c>int[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Int3
    {
        private int _element0;
    }

    private Int3 _points;
    private Int3 _edges;
    private Int3 _edgesOrientations;
    private bool _hasIntersection;
    private bool _isIntersectionPossible;
    private bool _isDegenerated;
    private double _deflection;
    private Box _box;

    /// <summary>A triangle with no points and no edges.</summary>
    public PolyhedronTriangle()
    {
        _hasIntersection = false;
        _isIntersectionPossible = true;
        _isDegenerated = false;
        _deflection = 0.0;
        _box = new Box();
        _points[0] = -1;
        _points[1] = -1;
        _points[2] = -1;
        _edges[0] = -1;
        _edges[1] = -1;
        _edges[2] = -1;
        _edgesOrientations[0] = 0;
        _edgesOrientations[1] = 0;
        _edgesOrientations[2] = 0;
    }

    /// <summary>A triangle of the given points, with no edges.</summary>
    public PolyhedronTriangle(int point1, int point2, int point3)
    {
        _hasIntersection = false;
        _isIntersectionPossible = true;
        _isDegenerated = false;
        _deflection = 0.0;
        _box = new Box();
        _points[0] = point1;
        _points[1] = point2;
        _points[2] = point3;
        _edges[0] = -1;
        _edges[1] = -1;
        _edges[2] = -1;
        _edgesOrientations[0] = 0;
        _edgesOrientations[1] = 0;
        _edgesOrientations[2] = 0;
    }

    public readonly int FirstPoint() => _points[0];

    public readonly int SecondPoint() => _points[1];

    public readonly int ThirdPoint() => _points[2];

    public readonly int FirstEdge() => _edges[0];

    public readonly int FirstEdgeOrientation() => _edgesOrientations[0];

    public readonly int SecondEdge() => _edges[1];

    public readonly int SecondEdgeOrientation() => _edgesOrientations[1];

    public readonly int ThirdEdge() => _edges[2];

    public readonly int ThirdEdgeOrientation() => _edgesOrientations[2];

    /// <summary>The deflection of the triangle (-1 once it has been split).</summary>
    public readonly double Deflection() => _deflection;

    /// <summary>The possibility of intersection.</summary>
    public readonly bool IsIntersectionPossible => _isIntersectionPossible;

    /// <summary>True if the triangle has interfered with a triangle of the other surface.</summary>
    public readonly bool HasIntersection => _hasIntersection;

    public readonly bool IsDegenerated => _isDegenerated;

    /// <summary>The edge number by the index (1 to 3); 0 for another index.</summary>
    public readonly int GetEdgeNumber(int edgeIndex)
        => (edgeIndex >= 1 && edgeIndex <= 3) ? _edges[edgeIndex - 1] : 0;

    /// <summary>The edge's orientation by the index (1 to 3); 0 for another index.</summary>
    public readonly int GetEdgeOrientation(int edgeIndex)
        => (edgeIndex >= 1 && edgeIndex <= 3) ? _edgesOrientations[edgeIndex - 1] : 0;

    /// <summary>The cached bounding box, void until <see cref="BoundingBox"/> has computed it.</summary>
    public readonly Box CachedBoundingBox() => _box;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFirstPoint(int point) => _points[0] = point;

    internal void SetSecondPoint(int point) => _points[1] = point;

    internal void SetThirdPoint(int point) => _points[2] = point;

    internal void SetFirstEdge(int edge, int edgeOrientation)
    {
        _edges[0] = edge;
        _edgesOrientations[0] = edgeOrientation;
    }

    internal void SetSecondEdge(int edge, int edgeOrientation)
    {
        _edges[1] = edge;
        _edgesOrientations[1] = edgeOrientation;
    }

    internal void SetThirdEdge(int edge, int edgeOrientation)
    {
        _edges[2] = edge;
        _edgesOrientations[2] = edgeOrientation;
    }

    internal void SetDeflection(double deflection) => _deflection = deflection;

    internal void SetIntersectionPossible(bool ip) => _isIntersectionPossible = ip;

    internal void SetIntersection(bool theInt) => _hasIntersection = theInt;

    internal void SetDegenerated(bool degFlag) => _isDegenerated = degFlag;

    internal void SetEdge(int edgeIndex, int edgeNumber)
    {
        if (edgeIndex >= 1 && edgeIndex <= 3)
        {
            _edges[edgeIndex - 1] = edgeNumber;
        }
    }

    internal void SetEdgeOrientation(int edgeIndex, int edgeOrientation)
    {
        if (edgeIndex >= 1 && edgeIndex <= 3)
        {
            _edgesOrientations[edgeIndex - 1] = edgeOrientation;
        }
    }

    public readonly PolyhedronTriangle WithFirstPoint(int point)
    {
        PolyhedronTriangle copy = this;
        copy.SetFirstPoint(point);
        return copy;
    }

    public readonly PolyhedronTriangle WithSecondPoint(int point)
    {
        PolyhedronTriangle copy = this;
        copy.SetSecondPoint(point);
        return copy;
    }

    public readonly PolyhedronTriangle WithThirdPoint(int point)
    {
        PolyhedronTriangle copy = this;
        copy.SetThirdPoint(point);
        return copy;
    }

    public readonly PolyhedronTriangle WithFirstEdge(int edge, int edgeOrientation)
    {
        PolyhedronTriangle copy = this;
        copy.SetFirstEdge(edge, edgeOrientation);
        return copy;
    }

    public readonly PolyhedronTriangle WithSecondEdge(int edge, int edgeOrientation)
    {
        PolyhedronTriangle copy = this;
        copy.SetSecondEdge(edge, edgeOrientation);
        return copy;
    }

    public readonly PolyhedronTriangle WithThirdEdge(int edge, int edgeOrientation)
    {
        PolyhedronTriangle copy = this;
        copy.SetThirdEdge(edge, edgeOrientation);
        return copy;
    }

    public readonly PolyhedronTriangle WithDeflection(double deflection)
    {
        PolyhedronTriangle copy = this;
        copy.SetDeflection(deflection);
        return copy;
    }

    public readonly PolyhedronTriangle WithIntersectionPossible(bool ip)
    {
        PolyhedronTriangle copy = this;
        copy.SetIntersectionPossible(ip);
        return copy;
    }

    public readonly PolyhedronTriangle WithIntersection(bool theInt)
    {
        PolyhedronTriangle copy = this;
        copy.SetIntersection(theInt);
        return copy;
    }

    public readonly PolyhedronTriangle WithDegenerated(bool degFlag)
    {
        PolyhedronTriangle copy = this;
        copy.SetDegenerated(degFlag);
        return copy;
    }

    public readonly PolyhedronTriangle WithEdge(int edgeIndex, int edgeNumber)
    {
        PolyhedronTriangle copy = this;
        copy.SetEdge(edgeIndex, edgeNumber);
        return copy;
    }

    public readonly PolyhedronTriangle WithEdgeOrientation(int edgeIndex, int edgeOrientation)
    {
        PolyhedronTriangle copy = this;
        copy.SetEdgeOrientation(edgeIndex, edgeOrientation);
        return copy;
    }

    /// <summary>
    /// Computes (and stores) the deflection of the triangle: the distance from its plane to the surface's point at the
    /// barycenter of its (U, V). Marks the triangle degenerated (deflection 0) if two of its points are degenerated or
    /// its normal vanishes.
    /// </summary>
    internal double ComputeDeflection(SurfaceAdaptorBase surface, PolyhedronArray<PolyhedronPoint> tPoints)
    {
        _deflection = 0.0;
        //
        ref readonly PolyhedronPoint p1 = ref tPoints.Value(_points[0]);
        ref readonly PolyhedronPoint p2 = ref tPoints.Value(_points[1]);
        ref readonly PolyhedronPoint p3 = ref tPoints.Value(_points[2]);
        //
        {
            // check if the triangle is not degenerated - no more than one point
            // has a degenerated flag
            int iDeg = (p1.Degenerated() ? 1 : 0) + (p2.Degenerated() ? 1 : 0) + (p3.Degenerated() ? 1 : 0);
            if (iDeg > 1)
            {
                _isDegenerated = true;
                return _deflection;
            }
        }
        //
        // Plane of the triangle
        PolyhedronPoint normaleTri = new();
        normaleTri.Cross(p2 - p1, p3 - p1);
        double sqNorm = normaleTri.SquareModulus();
        if (sqNorm < SquareMyConfusionPrecision)
        {
            // The triangle is degenerated
            _isDegenerated = true;
            return _deflection;
        }
        //
        // Compute point on the surface
        double gu = (p1.U() + p2.U() + p3.U()) / 3.0;
        double gv = (p1.V() + p2.V() + p3.V()) / 3.0;
        Point3D ptXYZ = surface.Value(gu, gv);
        // Point on the surface
        PolyhedronPoint barycentreReel = new(ptXYZ.X, ptXYZ.Y, ptXYZ.Z, gu, gv);
        // compute distance to plane
        normaleTri = normaleTri / Math.Sqrt(sqNorm);
        _deflection = Math.Abs(normaleTri.Dot(barycentreReel - p1));
        return _deflection;
    }

    /// <summary>The triangle on the other side of the given edge (1 to 3); -1 if none or for another index.</summary>
    public readonly int GetNextTriangle(int triangle, int edgeNum, PolyhedronArray<PolyhedronEdge> tEdges)
    {
        int nextTriangle = -1;
        if (edgeNum < 1 || edgeNum > 3)
        {
            return nextTriangle;
        }
        //
        ref readonly PolyhedronEdge edge = ref tEdges.Value(_edges[edgeNum - 1]);
        nextTriangle = (edge.FirstTriangle() == triangle) ? edge.SecondTriangle() : edge.FirstTriangle();
        return nextTriangle;
    }

    /// <summary>Links the edges to the triangle, with their orientations; nothing if one is negative.</summary>
    internal void LinkEdges2Triangle(PolyhedronArray<PolyhedronEdge> tEdges, int edge1, int edge2, int edge3)
    {
        if (edge1 < 0 || edge2 < 0 || edge3 < 0)
        {
            return;
        }
        //
        _edges[0] = edge1;
        _edges[1] = edge2;
        _edges[2] = edge3;
        //
        _edgesOrientations[0] = (tEdges.Value(_edges[0]).FirstPoint() == _points[0]) ? 1 : -1;
        _edgesOrientations[1] = (tEdges.Value(_edges[1]).FirstPoint() == _points[1]) ? 1 : -1;
        _edgesOrientations[2] = (tEdges.Value(_edges[2]).FirstPoint() == _points[2]) ? 1 : -1;
    }

    // The third point of the adjacent triangle (TriAdj) and the edges going from it
    private static void GetInfoTA(int numP1, int numP2, int numTA, PolyhedronArray<PolyhedronTriangle> tTriangles,
                                  ref int numP3b, ref int p3bIndex, ref int edge2b, ref int edge3b)
    {
        ref readonly PolyhedronTriangle triAdj = ref tTriangles.Value(numTA);
        int p1b = triAdj.FirstPoint();
        int p2b = triAdj.SecondPoint();
        int p3b = triAdj.ThirdPoint();

        if ((p1b != numP1) && (p1b != numP2))
        {
            numP3b = p1b;
            p3bIndex = 1;
            if (p2b == numP1)
            {
                // P1bP2b==numP3bnumP1:Edge3b
                edge3b = triAdj.FirstEdge();
                // so P1bP3b==numP3bnumP2:Edge2b
                edge2b = triAdj.ThirdEdge();
            }
            else
            {
                edge2b = triAdj.FirstEdge();
                edge3b = triAdj.ThirdEdge();
            }
        }
        else if ((p2b != numP1) && (p2b != numP2))
        {
            numP3b = p2b;
            p3bIndex = 2;
            if (p1b == numP1)
            {
                // P2bP1b==numP3bnumP1:Edge3b
                edge3b = triAdj.FirstEdge();
                // so P2bP3b==numP3bnumP2:Edge2b
                edge2b = triAdj.SecondEdge();
            }
            else
            {
                edge2b = triAdj.FirstEdge();
                edge3b = triAdj.SecondEdge();
            }
        }
        else if ((p3b != numP1) && (p3b != numP2))
        {
            numP3b = p3b;
            p3bIndex = 3;
            if (p2b == numP1)
            {
                // P3bP2b==numP3bnumP1:Edge3b
                edge3b = triAdj.SecondEdge();
                // so P3bP1b==numP3bnumP2:Edge2b
                edge2b = triAdj.ThirdEdge();
            }
            else
            {
                edge2b = triAdj.SecondEdge();
                edge3b = triAdj.ThirdEdge();
            }
        }
    }

    private static void NewTriangle(int p1, int p2, int p3, PolyhedronArray<PolyhedronTriangle> tTriangles,
                                    SurfaceAdaptorBase mySurface, PolyhedronArray<PolyhedronPoint> tPoints)
    {
        int finTT = tTriangles.ItemCount;
        tTriangles[finTT].SetFirstPoint(p1);
        tTriangles[finTT].SetSecondPoint(p2);
        tTriangles[finTT].SetThirdPoint(p3);
        tTriangles[finTT].ComputeDeflection(mySurface, tPoints);
        tTriangles.IncrementNbItems();
    }

    private static void NewEdge(int p1, int p2, int t1, int t2, PolyhedronArray<PolyhedronEdge> tEdges)
    {
        int finTE = tEdges.ItemCount;

        tEdges[finTE].SetFirstPoint(p1);
        tEdges[finTE].SetSecondPoint(p2);
        tEdges[finTE].SetFirstTriangle(t1);
        tEdges[finTE].SetSecondTriangle(t2);
        tEdges.IncrementNbItems();
    }

    private static void OldEdge(int edgeN, int numTri, int newTriNum, PolyhedronArray<PolyhedronEdge> tEdges)
    {
        if (tEdges[edgeN].FirstTriangle() == numTri)
        {
            tEdges[edgeN].SetFirstTriangle(newTriNum);
        }
        else
        {
            tEdges[edgeN].SetSecondTriangle(newTriNum);
        }
    }

    /// <summary>
    /// Splits the triangle (number <paramref name="numTri"/>) in two by the middle of its longest side, and the triangle
    /// adjacent along that side too; the split triangles are made obsolete (deflection -1, no intersection possible).
    /// </summary>
    internal void MiddleRefinement(int numTri, SurfaceAdaptorBase mySurface, PolyhedronArray<PolyhedronPoint> tPoints,
                                   PolyhedronArray<PolyhedronTriangle> tTriangles, PolyhedronArray<PolyhedronEdge> tEdges)
    {
        int finTE = tEdges.ItemCount;
        int finTT = tTriangles.ItemCount;

        // Refinement of the mesh by the middle of the largest dimensions
        int numP1 = FirstPoint();
        int numP2 = SecondPoint();
        int numP3 = ThirdPoint();

        ref readonly PolyhedronPoint p1 = ref tPoints.Value(numP1);
        ref readonly PolyhedronPoint p2 = ref tPoints.Value(numP2);
        ref readonly PolyhedronPoint p3 = ref tPoints.Value(numP3);

        // compute the largest dimension
        double l12 = p1.SquareDistance(p2);
        double l23 = p2.SquareDistance(p3);
        double l31 = p3.SquareDistance(p1);

        if ((l12 > l23) && (l12 > l31))
        {
            int finTP = tPoints.ItemCount;
            tPoints[finTP].Middle(mySurface, p1, p2);

            // the new triangles
            int t1, t2, t3, t4;

            t1 = finTT;
            NewTriangle(numP2, numP3, finTP, tTriangles, mySurface, tPoints);
            finTT++;
            t2 = finTT;
            NewTriangle(numP3, numP1, finTP, tTriangles, mySurface, tPoints);
            finTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(numTri, 1, tEdges);

            if (numTA >= 0)
            {
                int numP3b = -1;
                int p3bIndex = -1;

                int edge2b = -1;
                int edge3b = -1;

                GetInfoTA(numP1, numP2, numTA, tTriangles, ref numP3b, ref p3bIndex, ref edge2b, ref edge3b);

                t3 = finTT;
                NewTriangle(numP2, numP3b, finTP, tTriangles, mySurface, tPoints);
                finTT++;
                t4 = finTT;
                NewTriangle(numP3b, numP1, finTP, tTriangles, mySurface, tPoints);

                // The new edges
                int e1, e2, e3, e4;

                e1 = finTE;
                NewEdge(numP1, finTP, t2, t4, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP2, t1, t3, tEdges);
                finTE++;
                e3 = finTE;
                NewEdge(finTP, numP3, t1, t2, tEdges);
                finTE++;
                e4 = finTE;
                NewEdge(finTP, numP3b, t3, t4, tEdges);

                // The old edges are updated
                OldEdge(_edges[1], numTri, t1, tEdges);
                OldEdge(_edges[2], numTri, t2, tEdges);
                OldEdge(edge2b, numTA, t3, tEdges);
                OldEdge(edge3b, numTA, t4, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[1], e3, e2);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[2], e1, e3);
                tTriangles[t3].LinkEdges2Triangle(tEdges, edge2b, e4, e2);
                tTriangles[t4].LinkEdges2Triangle(tEdges, edge3b, e1, e4);

                // The adjacent triangle is killed
                tTriangles[numTA].SetDeflection(-1.0);
                tTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles: the new edges with T1 and T2
                int e1, e2, e3;

                e1 = finTE;
                NewEdge(numP1, finTP, t2, -1, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP2, t1, -1, tEdges);
                finTE++;
                e3 = finTE;
                NewEdge(finTP, numP3, t1, t2, tEdges);

                // The old edges are updated
                OldEdge(_edges[1], numTri, t1, tEdges);
                OldEdge(_edges[2], numTri, t2, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[1], e3, e2);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[2], e1, e3);
            }
        }
        else if ((l23 > l31) && (l23 > l12))
        {
            int finTP = tPoints.ItemCount;
            tPoints[finTP].Middle(mySurface, p2, p3);

            // the new triangles
            int t1, t2, t3, t4;

            t1 = finTT;
            NewTriangle(numP1, numP2, finTP, tTriangles, mySurface, tPoints);
            finTT++;
            t2 = finTT;
            NewTriangle(numP3, numP1, finTP, tTriangles, mySurface, tPoints);
            finTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(numTri, 2, tEdges);

            if (numTA >= 0)
            {
                int numP1b = -1;
                int p1bIndex = -1;

                int edge1b = -1;
                int edge3b = -1;

                GetInfoTA(numP2, numP3, numTA, tTriangles, ref numP1b, ref p1bIndex, ref edge3b, ref edge1b);

                t3 = finTT;
                NewTriangle(numP2, numP1b, finTP, tTriangles, mySurface, tPoints);
                finTT++;
                t4 = finTT;
                NewTriangle(numP1b, numP3, finTP, tTriangles, mySurface, tPoints);

                // The new edges
                int e1, e2, e3, e4;

                e1 = finTE;
                NewEdge(numP2, finTP, t1, t3, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP3, t2, t4, tEdges);
                finTE++;
                e3 = finTE;
                NewEdge(finTP, numP1, t1, t2, tEdges);
                finTE++;
                e4 = finTE;
                NewEdge(finTP, numP1b, t3, t4, tEdges);

                // The old edges are updated
                OldEdge(_edges[0], numTri, t1, tEdges);
                OldEdge(_edges[2], numTri, t2, tEdges);
                OldEdge(edge1b, numTA, t3, tEdges);
                OldEdge(edge3b, numTA, t4, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[0], e1, e3);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[2], e3, e2);
                tTriangles[t3].LinkEdges2Triangle(tEdges, edge1b, e4, e1);
                tTriangles[t4].LinkEdges2Triangle(tEdges, edge3b, e2, e4);

                // The adjacent triangle is killed
                tTriangles[numTA].SetDeflection(-1.0);
                tTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles
                int e1, e2, e3;

                e1 = finTE;
                NewEdge(numP2, finTP, t1, -1, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP3, t2, -1, tEdges);
                finTE++;
                e3 = finTE;
                NewEdge(finTP, numP1, t1, t2, tEdges);

                // The old edges are updated
                OldEdge(_edges[0], numTri, t1, tEdges);
                OldEdge(_edges[2], numTri, t2, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[0], e1, e3);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[2], e3, e2);
            }
        }
        else
        {
            int finTP = tPoints.ItemCount;
            tPoints[finTP].Middle(mySurface, p3, p1);

            int t1, t2, t3, t4;

            t1 = finTT;
            NewTriangle(numP1, numP2, finTP, tTriangles, mySurface, tPoints);
            finTT++;
            t2 = finTT;
            NewTriangle(numP2, numP3, finTP, tTriangles, mySurface, tPoints);
            finTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(numTri, 3, tEdges);

            if (numTA >= 0)
            {
                int numP2b = -1;
                int p2bIndex = -1;

                int edge1b = -1;
                int edge2b = -1;

                GetInfoTA(numP3, numP1, numTA, tTriangles, ref numP2b, ref p2bIndex, ref edge1b, ref edge2b);

                t3 = finTT;
                NewTriangle(numP1, numP2b, finTP, tTriangles, mySurface, tPoints);
                finTT++;
                t4 = finTT;
                NewTriangle(numP2b, numP3, finTP, tTriangles, mySurface, tPoints);

                // The new edges
                int e1, e2, e3, e4;

                e1 = finTE;
                NewEdge(numP2, finTP, t1, t2, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP3, t2, t4, tEdges);
                finTE++;
                e3 = finTE;
                NewEdge(finTP, numP2b, t4, t3, tEdges);
                finTE++;
                e4 = finTE;
                NewEdge(finTP, numP1, t1, t3, tEdges);

                // The old edges are updated
                OldEdge(_edges[0], numTri, t1, tEdges);
                OldEdge(_edges[1], numTri, t2, tEdges);
                OldEdge(edge1b, numTA, t3, tEdges);
                OldEdge(edge2b, numTA, t4, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[0], e1, e4);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[1], e2, e1);
                tTriangles[t3].LinkEdges2Triangle(tEdges, edge1b, e3, e4);
                tTriangles[t4].LinkEdges2Triangle(tEdges, edge2b, e2, e3);

                // The adjacent triangle is killed
                tTriangles[numTA].SetDeflection(-1.0);
                tTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles
                int e1, e2, e4;

                e1 = finTE;
                NewEdge(numP2, finTP, t1, t2, tEdges);
                finTE++;
                e2 = finTE;
                NewEdge(finTP, numP3, t2, -1, tEdges);
                finTE++;
                e4 = finTE;
                NewEdge(finTP, numP1, t1, -1, tEdges);

                // The old edges are updated
                OldEdge(_edges[0], numTri, t1, tEdges);
                OldEdge(_edges[1], numTri, t2, tEdges);

                // The new triangles are filled with the edges
                tTriangles[t1].LinkEdges2Triangle(tEdges, _edges[0], e1, e4);
                tTriangles[t2].LinkEdges2Triangle(tEdges, _edges[1], e2, e1);
            }
        }

        tPoints.IncrementNbItems();

        // make the triangle obsolete
        _deflection = -1.0;
        _isIntersectionPossible = false;
    }

    /// <summary>
    /// Splits the triangle, then the new triangles, while their deflection is above the criterion, marking those out of
    /// the box; at most about twice the number of triangles there were before.
    /// </summary>
    internal void MultipleMiddleRefinement(double refineCriterion, in Box box, int triangleNumber,
                                           SurfaceAdaptorBase surface, PolyhedronArray<PolyhedronPoint> tPoints,
                                           PolyhedronArray<PolyhedronTriangle> tTriangles, PolyhedronArray<PolyhedronEdge> tEdges)
    {
        // Number of triangles before refinement of current triangle
        int finTTInit = tTriangles.ItemCount;
        // Criteria to stop splitting - double of the initial number of triangles,
        // i.e. allow each triangle to be split at least once. Add a constant
        // to allow the splits of triangles to be checked.
        int maxNbTT = 2 * finTTInit + 1000;
        // Split the current triangle
        MiddleRefinement(triangleNumber, surface, tPoints, tTriangles, tEdges);
        // Refine the new triangles
        for (int i = finTTInit; i < tTriangles.ItemCount && i < maxNbTT; ++i)
        {
            ref PolyhedronTriangle triangle = ref tTriangles[i];
            if (box.IsOut(triangle.BoundingBox(tPoints)))
            {
                triangle.SetIntersectionPossible(false);
            }
            else if (triangle.Deflection() > refineCriterion)
            {
                triangle.MiddleRefinement(i, surface, tPoints, tTriangles, tEdges);
            }
        }
    }

    /// <summary>Sets the edge and its orientation at the side of the triangle that has the edge's points.</summary>
    internal void SetEdgeAndOrientation(in PolyhedronEdge edge, int edgeIndex)
    {
        // Points on the edge - pe1, pe2
        int pe1 = edge.FirstPoint(), pe2 = edge.SecondPoint();
        //
        // We have points on the triangle - p1, p2 and p3;
        // And points on the edge - pe1, pe2;
        // By comparing these points we should define which
        // edge it is for the triangle and its orientation on it:
        // e1 = p1->p2;
        // e2 = p2->p3;
        // e3 = p3->p1;
        // In case the order of points on the edge is forward,
        // the orientation is positive, otherwise it is negative.

        for (int i = 0, i1 = 1; i < 3; ++i, ++i1)
        {
            if (i1 > 2)
            {
                i1 = 0;
            }
            //
            if (pe1 == _points[i] && pe2 == _points[i1])
            {
                _edges[i] = edgeIndex;
                _edgesOrientations[i] = 1;
                break;
            }

            if (pe1 == _points[i1] && pe2 == _points[i])
            {
                _edges[i] = edgeIndex;
                _edgesOrientations[i] = -1;
                break;
            }
        }
    }

    /// <summary>
    /// The bounding box of the triangle, enlarged by its deflection; computed on the first call and cached (the cache is
    /// never reset, as in OCCT).
    /// </summary>
    internal Box BoundingBox(PolyhedronArray<PolyhedronPoint> points)
    {
        if (_box.IsVoid)
        {
            ref readonly PolyhedronPoint p1 = ref points.Value(_points[0]);
            ref readonly PolyhedronPoint p2 = ref points.Value(_points[1]);
            ref readonly PolyhedronPoint p3 = ref points.Value(_points[2]);
            _box.Add(new Point3D(p1.X(), p1.Y(), p1.Z()));
            _box.Add(new Point3D(p2.X(), p2.Y(), p2.Z()));
            _box.Add(new Point3D(p3.X(), p3.Y(), p3.Z()));
            _box.SetGap(_deflection + global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        return _box;
    }

    /// <summary>Writes the triangle with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter stream)
    {
        stream.Write($"\nTriangle({Format.D(i, 3)}) : Points {Format.D(_points[0], 5)} "
                        + $"{Format.D(_points[1], 5)} {Format.D(_points[2], 5)} Edges "
                        + $"{Format.D(_edges[0], 5)} {Format.D(_edges[1], 5)} "
                        + $"{Format.D(_edges[2], 5)} deflection: {Format.F(_deflection, 8)} "
                        + $"intersection possible {Format.D(_isIntersectionPossible ? 1 : 0, 8)}  "
                        + $"intersection: {Format.D(_hasIntersection ? 1 : 0, 5)}\n");
    }

    public readonly bool Equals(PolyhedronTriangle other)
        => _points[0] == other._points[0] && _points[1] == other._points[1] && _points[2] == other._points[2]
           && _edges[0] == other._edges[0] && _edges[1] == other._edges[1] && _edges[2] == other._edges[2]
           && _edgesOrientations[0] == other._edgesOrientations[0]
           && _edgesOrientations[1] == other._edgesOrientations[1]
           && _edgesOrientations[2] == other._edgesOrientations[2] && _hasIntersection == other._hasIntersection
           && _isIntersectionPossible == other._isIntersectionPossible && _isDegenerated == other._isDegenerated
           && _deflection.Equals(other._deflection) && _box.Equals(other._box);

    public override readonly bool Equals(object? obj) => obj is PolyhedronTriangle other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(_points[0], _points[1], _points[2]),
                            HashCode.Combine(_edges[0], _edges[1], _edges[2]),
                            HashCode.Combine(_edgesOrientations[0], _edgesOrientations[1], _edgesOrientations[2]),
                            _hasIntersection, _isIntersectionPossible, _isDegenerated, _deflection, _box);

    public static bool operator ==(in PolyhedronTriangle left, in PolyhedronTriangle right) => left.Equals(right);

    public static bool operator !=(in PolyhedronTriangle left, in PolyhedronTriangle right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntPolyh_Triangle(P {_points[0]}, {_points[1]}, {_points[2]}; E {_edges[0]}, {_edges[1]}, {_edges[2]}; "
           + $"{Constants.Format("deflection", _deflection)}, possible {_isIntersectionPossible}, intersection {_hasIntersection})";
}
