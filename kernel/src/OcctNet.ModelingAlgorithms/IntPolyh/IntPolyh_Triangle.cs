// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Triangle.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Triangle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using System.Runtime.CompilerServices;

namespace Occt;

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
/// whose storage never moves (see <see cref="IntPolyh_Array{T}"/>).
/// </para>
/// </remarks>
public partial struct IntPolyh_Triangle : IEquatable<IntPolyh_Triangle>
{
    private const double SquareMyConfusionPrecision = 10.0e-24;

    /// <summary>Three indices (C++'s <c>int[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Int3
    {
        private int myElement0;
    }

    private Int3 myPoints;
    private Int3 myEdges;
    private Int3 myEdgesOrientations;
    private bool myHasIntersection;
    private bool myIsIntersectionPossible;
    private bool myIsDegenerated;
    private double myDeflection;
    private Box myBox;

    /// <summary>A triangle with no points and no edges.</summary>
    public IntPolyh_Triangle()
    {
        myHasIntersection = false;
        myIsIntersectionPossible = true;
        myIsDegenerated = false;
        myDeflection = 0.0;
        myBox = new Box();
        myPoints[0] = -1;
        myPoints[1] = -1;
        myPoints[2] = -1;
        myEdges[0] = -1;
        myEdges[1] = -1;
        myEdges[2] = -1;
        myEdgesOrientations[0] = 0;
        myEdgesOrientations[1] = 0;
        myEdgesOrientations[2] = 0;
    }

    /// <summary>A triangle of the given points, with no edges.</summary>
    public IntPolyh_Triangle(int thePoint1, int thePoint2, int thePoint3)
    {
        myHasIntersection = false;
        myIsIntersectionPossible = true;
        myIsDegenerated = false;
        myDeflection = 0.0;
        myBox = new Box();
        myPoints[0] = thePoint1;
        myPoints[1] = thePoint2;
        myPoints[2] = thePoint3;
        myEdges[0] = -1;
        myEdges[1] = -1;
        myEdges[2] = -1;
        myEdgesOrientations[0] = 0;
        myEdgesOrientations[1] = 0;
        myEdgesOrientations[2] = 0;
    }

    public readonly int FirstPoint() => myPoints[0];

    public readonly int SecondPoint() => myPoints[1];

    public readonly int ThirdPoint() => myPoints[2];

    public readonly int FirstEdge() => myEdges[0];

    public readonly int FirstEdgeOrientation() => myEdgesOrientations[0];

    public readonly int SecondEdge() => myEdges[1];

    public readonly int SecondEdgeOrientation() => myEdgesOrientations[1];

    public readonly int ThirdEdge() => myEdges[2];

    public readonly int ThirdEdgeOrientation() => myEdgesOrientations[2];

    /// <summary>The deflection of the triangle (-1 once it has been split).</summary>
    public readonly double Deflection() => myDeflection;

    /// <summary>The possibility of intersection.</summary>
    public readonly bool IsIntersectionPossible() => myIsIntersectionPossible;

    /// <summary>True if the triangle has interfered with a triangle of the other surface.</summary>
    public readonly bool HasIntersection() => myHasIntersection;

    public readonly bool IsDegenerated() => myIsDegenerated;

    /// <summary>The edge number by the index (1 to 3); 0 for another index.</summary>
    public readonly int GetEdgeNumber(int theEdgeIndex)
        => (theEdgeIndex >= 1 && theEdgeIndex <= 3) ? myEdges[theEdgeIndex - 1] : 0;

    /// <summary>The edge's orientation by the index (1 to 3); 0 for another index.</summary>
    public readonly int GetEdgeOrientation(int theEdgeIndex)
        => (theEdgeIndex >= 1 && theEdgeIndex <= 3) ? myEdgesOrientations[theEdgeIndex - 1] : 0;

    /// <summary>The cached bounding box, void until <see cref="BoundingBox"/> has computed it.</summary>
    public readonly Box CachedBoundingBox() => myBox;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFirstPoint(int thePoint) => myPoints[0] = thePoint;

    internal void SetSecondPoint(int thePoint) => myPoints[1] = thePoint;

    internal void SetThirdPoint(int thePoint) => myPoints[2] = thePoint;

    internal void SetFirstEdge(int theEdge, int theEdgeOrientation)
    {
        myEdges[0] = theEdge;
        myEdgesOrientations[0] = theEdgeOrientation;
    }

    internal void SetSecondEdge(int theEdge, int theEdgeOrientation)
    {
        myEdges[1] = theEdge;
        myEdgesOrientations[1] = theEdgeOrientation;
    }

    internal void SetThirdEdge(int theEdge, int theEdgeOrientation)
    {
        myEdges[2] = theEdge;
        myEdgesOrientations[2] = theEdgeOrientation;
    }

    internal void SetDeflection(double theDeflection) => myDeflection = theDeflection;

    internal void SetIntersectionPossible(bool theIP) => myIsIntersectionPossible = theIP;

    internal void SetIntersection(bool theInt) => myHasIntersection = theInt;

    internal void SetDegenerated(bool theDegFlag) => myIsDegenerated = theDegFlag;

    internal void SetEdge(int theEdgeIndex, int theEdgeNumber)
    {
        if (theEdgeIndex >= 1 && theEdgeIndex <= 3)
        {
            myEdges[theEdgeIndex - 1] = theEdgeNumber;
        }
    }

    internal void SetEdgeOrientation(int theEdgeIndex, int theEdgeOrientation)
    {
        if (theEdgeIndex >= 1 && theEdgeIndex <= 3)
        {
            myEdgesOrientations[theEdgeIndex - 1] = theEdgeOrientation;
        }
    }

    public readonly IntPolyh_Triangle WithFirstPoint(int thePoint)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetFirstPoint(thePoint);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithSecondPoint(int thePoint)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetSecondPoint(thePoint);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithThirdPoint(int thePoint)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetThirdPoint(thePoint);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithFirstEdge(int theEdge, int theEdgeOrientation)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetFirstEdge(theEdge, theEdgeOrientation);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithSecondEdge(int theEdge, int theEdgeOrientation)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetSecondEdge(theEdge, theEdgeOrientation);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithThirdEdge(int theEdge, int theEdgeOrientation)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetThirdEdge(theEdge, theEdgeOrientation);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithDeflection(double theDeflection)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetDeflection(theDeflection);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithIntersectionPossible(bool theIP)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetIntersectionPossible(theIP);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithIntersection(bool theInt)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetIntersection(theInt);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithDegenerated(bool theDegFlag)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetDegenerated(theDegFlag);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithEdge(int theEdgeIndex, int theEdgeNumber)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetEdge(theEdgeIndex, theEdgeNumber);
        return aCopy;
    }

    public readonly IntPolyh_Triangle WithEdgeOrientation(int theEdgeIndex, int theEdgeOrientation)
    {
        IntPolyh_Triangle aCopy = this;
        aCopy.SetEdgeOrientation(theEdgeIndex, theEdgeOrientation);
        return aCopy;
    }

    /// <summary>
    /// Computes (and stores) the deflection of the triangle: the distance from its plane to the surface's point at the
    /// barycenter of its (U, V). Marks the triangle degenerated (deflection 0) if two of its points are degenerated or
    /// its normal vanishes.
    /// </summary>
    internal double ComputeDeflection(SurfaceAdaptorBase theSurface, IntPolyh_Array<IntPolyh_Point> TPoints)
    {
        myDeflection = 0.0;
        //
        ref readonly IntPolyh_Point P1 = ref TPoints.Value(myPoints[0]);
        ref readonly IntPolyh_Point P2 = ref TPoints.Value(myPoints[1]);
        ref readonly IntPolyh_Point P3 = ref TPoints.Value(myPoints[2]);
        //
        {
            // check if the triangle is not degenerated - no more than one point
            // has a degenerated flag
            int iDeg = (P1.Degenerated() ? 1 : 0) + (P2.Degenerated() ? 1 : 0) + (P3.Degenerated() ? 1 : 0);
            if (iDeg > 1)
            {
                myIsDegenerated = true;
                return myDeflection;
            }
        }
        //
        // Plane of the triangle
        IntPolyh_Point NormaleTri = new();
        NormaleTri.Cross(P2 - P1, P3 - P1);
        double SqNorm = NormaleTri.SquareModulus();
        if (SqNorm < SquareMyConfusionPrecision)
        {
            // The triangle is degenerated
            myIsDegenerated = true;
            return myDeflection;
        }
        //
        // Compute point on the surface
        double Gu = (P1.U() + P2.U() + P3.U()) / 3.0;
        double Gv = (P1.V() + P2.V() + P3.V()) / 3.0;
        Point3D PtXYZ = theSurface.Value(Gu, Gv);
        // Point on the surface
        IntPolyh_Point BarycentreReel = new(PtXYZ.X(), PtXYZ.Y(), PtXYZ.Z(), Gu, Gv);
        // compute distance to plane
        NormaleTri = NormaleTri / Math.Sqrt(SqNorm);
        myDeflection = Math.Abs(NormaleTri.Dot(BarycentreReel - P1));
        return myDeflection;
    }

    /// <summary>The triangle on the other side of the given edge (1 to 3); -1 if none or for another index.</summary>
    public readonly int GetNextTriangle(int theTriangle, int theEdgeNum, IntPolyh_Array<IntPolyh_Edge> TEdges)
    {
        int aNextTriangle = -1;
        if (theEdgeNum < 1 || theEdgeNum > 3)
        {
            return aNextTriangle;
        }
        //
        ref readonly IntPolyh_Edge anEdge = ref TEdges.Value(myEdges[theEdgeNum - 1]);
        aNextTriangle = (anEdge.FirstTriangle() == theTriangle) ? anEdge.SecondTriangle() : anEdge.FirstTriangle();
        return aNextTriangle;
    }

    /// <summary>Links the edges to the triangle, with their orientations; nothing if one is negative.</summary>
    internal void LinkEdges2Triangle(IntPolyh_Array<IntPolyh_Edge> TEdges, int theEdge1, int theEdge2, int theEdge3)
    {
        if (theEdge1 < 0 || theEdge2 < 0 || theEdge3 < 0)
        {
            return;
        }
        //
        myEdges[0] = theEdge1;
        myEdges[1] = theEdge2;
        myEdges[2] = theEdge3;
        //
        myEdgesOrientations[0] = (TEdges.Value(myEdges[0]).FirstPoint() == myPoints[0]) ? 1 : -1;
        myEdgesOrientations[1] = (TEdges.Value(myEdges[1]).FirstPoint() == myPoints[1]) ? 1 : -1;
        myEdgesOrientations[2] = (TEdges.Value(myEdges[2]).FirstPoint() == myPoints[2]) ? 1 : -1;
    }

    // The third point of the adjacent triangle (TriAdj) and the edges going from it
    private static void GetInfoTA(int numP1, int numP2, int numTA, IntPolyh_Array<IntPolyh_Triangle> TTriangles,
                                  ref int numP3b, ref int P3bIndex, ref int Edge2b, ref int Edge3b)
    {
        ref readonly IntPolyh_Triangle TriAdj = ref TTriangles.Value(numTA);
        int P1b = TriAdj.FirstPoint();
        int P2b = TriAdj.SecondPoint();
        int P3b = TriAdj.ThirdPoint();

        if ((P1b != numP1) && (P1b != numP2))
        {
            numP3b = P1b;
            P3bIndex = 1;
            if (P2b == numP1)
            {
                // P1bP2b==numP3bnumP1:Edge3b
                Edge3b = TriAdj.FirstEdge();
                // so P1bP3b==numP3bnumP2:Edge2b
                Edge2b = TriAdj.ThirdEdge();
            }
            else
            {
                Edge2b = TriAdj.FirstEdge();
                Edge3b = TriAdj.ThirdEdge();
            }
        }
        else if ((P2b != numP1) && (P2b != numP2))
        {
            numP3b = P2b;
            P3bIndex = 2;
            if (P1b == numP1)
            {
                // P2bP1b==numP3bnumP1:Edge3b
                Edge3b = TriAdj.FirstEdge();
                // so P2bP3b==numP3bnumP2:Edge2b
                Edge2b = TriAdj.SecondEdge();
            }
            else
            {
                Edge2b = TriAdj.FirstEdge();
                Edge3b = TriAdj.SecondEdge();
            }
        }
        else if ((P3b != numP1) && (P3b != numP2))
        {
            numP3b = P3b;
            P3bIndex = 3;
            if (P2b == numP1)
            {
                // P3bP2b==numP3bnumP1:Edge3b
                Edge3b = TriAdj.SecondEdge();
                // so P3bP1b==numP3bnumP2:Edge2b
                Edge2b = TriAdj.ThirdEdge();
            }
            else
            {
                Edge2b = TriAdj.SecondEdge();
                Edge3b = TriAdj.ThirdEdge();
            }
        }
    }

    private static void NewTriangle(int P1, int P2, int P3, IntPolyh_Array<IntPolyh_Triangle> TTriangles,
                                    SurfaceAdaptorBase MySurface, IntPolyh_Array<IntPolyh_Point> TPoints)
    {
        int FinTT = TTriangles.NbItems();
        TTriangles[FinTT].SetFirstPoint(P1);
        TTriangles[FinTT].SetSecondPoint(P2);
        TTriangles[FinTT].SetThirdPoint(P3);
        TTriangles[FinTT].ComputeDeflection(MySurface, TPoints);
        TTriangles.IncrementNbItems();
    }

    private static void NewEdge(int P1, int P2, int T1, int T2, IntPolyh_Array<IntPolyh_Edge> TEdges)
    {
        int FinTE = TEdges.NbItems();

        TEdges[FinTE].SetFirstPoint(P1);
        TEdges[FinTE].SetSecondPoint(P2);
        TEdges[FinTE].SetFirstTriangle(T1);
        TEdges[FinTE].SetSecondTriangle(T2);
        TEdges.IncrementNbItems();
    }

    private static void OldEdge(int EdgeN, int NumTri, int NewTriNum, IntPolyh_Array<IntPolyh_Edge> TEdges)
    {
        if (TEdges[EdgeN].FirstTriangle() == NumTri)
        {
            TEdges[EdgeN].SetFirstTriangle(NewTriNum);
        }
        else
        {
            TEdges[EdgeN].SetSecondTriangle(NewTriNum);
        }
    }

    /// <summary>
    /// Splits the triangle (number <paramref name="NumTri"/>) in two by the middle of its longest side, and the triangle
    /// adjacent along that side too; the split triangles are made obsolete (deflection -1, no intersection possible).
    /// </summary>
    internal void MiddleRefinement(int NumTri, SurfaceAdaptorBase MySurface, IntPolyh_Array<IntPolyh_Point> TPoints,
                                   IntPolyh_Array<IntPolyh_Triangle> TTriangles, IntPolyh_Array<IntPolyh_Edge> TEdges)
    {
        int FinTE = TEdges.NbItems();
        int FinTT = TTriangles.NbItems();

        // Refinement of the mesh by the middle of the largest dimensions
        int numP1 = FirstPoint();
        int numP2 = SecondPoint();
        int numP3 = ThirdPoint();

        ref readonly IntPolyh_Point P1 = ref TPoints.Value(numP1);
        ref readonly IntPolyh_Point P2 = ref TPoints.Value(numP2);
        ref readonly IntPolyh_Point P3 = ref TPoints.Value(numP3);

        // compute the largest dimension
        double L12 = P1.SquareDistance(P2);
        double L23 = P2.SquareDistance(P3);
        double L31 = P3.SquareDistance(P1);

        if ((L12 > L23) && (L12 > L31))
        {
            int FinTP = TPoints.NbItems();
            TPoints[FinTP].Middle(MySurface, P1, P2);

            // the new triangles
            int T1, T2, T3, T4;

            T1 = FinTT;
            NewTriangle(numP2, numP3, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;
            T2 = FinTT;
            NewTriangle(numP3, numP1, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(NumTri, 1, TEdges);

            if (numTA >= 0)
            {
                int numP3b = -1;
                int P3bIndex = -1;

                int Edge2b = -1;
                int Edge3b = -1;

                GetInfoTA(numP1, numP2, numTA, TTriangles, ref numP3b, ref P3bIndex, ref Edge2b, ref Edge3b);

                T3 = FinTT;
                NewTriangle(numP2, numP3b, FinTP, TTriangles, MySurface, TPoints);
                FinTT++;
                T4 = FinTT;
                NewTriangle(numP3b, numP1, FinTP, TTriangles, MySurface, TPoints);

                // The new edges
                int E1, E2, E3, E4;

                E1 = FinTE;
                NewEdge(numP1, FinTP, T2, T4, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP2, T1, T3, TEdges);
                FinTE++;
                E3 = FinTE;
                NewEdge(FinTP, numP3, T1, T2, TEdges);
                FinTE++;
                E4 = FinTE;
                NewEdge(FinTP, numP3b, T3, T4, TEdges);

                // The old edges are updated
                OldEdge(myEdges[1], NumTri, T1, TEdges);
                OldEdge(myEdges[2], NumTri, T2, TEdges);
                OldEdge(Edge2b, numTA, T3, TEdges);
                OldEdge(Edge3b, numTA, T4, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[1], E3, E2);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[2], E1, E3);
                TTriangles[T3].LinkEdges2Triangle(TEdges, Edge2b, E4, E2);
                TTriangles[T4].LinkEdges2Triangle(TEdges, Edge3b, E1, E4);

                // The adjacent triangle is killed
                TTriangles[numTA].SetDeflection(-1.0);
                TTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles: the new edges with T1 and T2
                int E1, E2, E3;

                E1 = FinTE;
                NewEdge(numP1, FinTP, T2, -1, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP2, T1, -1, TEdges);
                FinTE++;
                E3 = FinTE;
                NewEdge(FinTP, numP3, T1, T2, TEdges);

                // The old edges are updated
                OldEdge(myEdges[1], NumTri, T1, TEdges);
                OldEdge(myEdges[2], NumTri, T2, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[1], E3, E2);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[2], E1, E3);
            }
        }
        else if ((L23 > L31) && (L23 > L12))
        {
            int FinTP = TPoints.NbItems();
            TPoints[FinTP].Middle(MySurface, P2, P3);

            // the new triangles
            int T1, T2, T3, T4;

            T1 = FinTT;
            NewTriangle(numP1, numP2, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;
            T2 = FinTT;
            NewTriangle(numP3, numP1, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(NumTri, 2, TEdges);

            if (numTA >= 0)
            {
                int numP1b = -1;
                int P1bIndex = -1;

                int Edge1b = -1;
                int Edge3b = -1;

                GetInfoTA(numP2, numP3, numTA, TTriangles, ref numP1b, ref P1bIndex, ref Edge3b, ref Edge1b);

                T3 = FinTT;
                NewTriangle(numP2, numP1b, FinTP, TTriangles, MySurface, TPoints);
                FinTT++;
                T4 = FinTT;
                NewTriangle(numP1b, numP3, FinTP, TTriangles, MySurface, TPoints);

                // The new edges
                int E1, E2, E3, E4;

                E1 = FinTE;
                NewEdge(numP2, FinTP, T1, T3, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP3, T2, T4, TEdges);
                FinTE++;
                E3 = FinTE;
                NewEdge(FinTP, numP1, T1, T2, TEdges);
                FinTE++;
                E4 = FinTE;
                NewEdge(FinTP, numP1b, T3, T4, TEdges);

                // The old edges are updated
                OldEdge(myEdges[0], NumTri, T1, TEdges);
                OldEdge(myEdges[2], NumTri, T2, TEdges);
                OldEdge(Edge1b, numTA, T3, TEdges);
                OldEdge(Edge3b, numTA, T4, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[0], E1, E3);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[2], E3, E2);
                TTriangles[T3].LinkEdges2Triangle(TEdges, Edge1b, E4, E1);
                TTriangles[T4].LinkEdges2Triangle(TEdges, Edge3b, E2, E4);

                // The adjacent triangle is killed
                TTriangles[numTA].SetDeflection(-1.0);
                TTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles
                int E1, E2, E3;

                E1 = FinTE;
                NewEdge(numP2, FinTP, T1, -1, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP3, T2, -1, TEdges);
                FinTE++;
                E3 = FinTE;
                NewEdge(FinTP, numP1, T1, T2, TEdges);

                // The old edges are updated
                OldEdge(myEdges[0], NumTri, T1, TEdges);
                OldEdge(myEdges[2], NumTri, T2, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[0], E1, E3);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[2], E3, E2);
            }
        }
        else
        {
            int FinTP = TPoints.NbItems();
            TPoints[FinTP].Middle(MySurface, P3, P1);

            int T1, T2, T3, T4;

            T1 = FinTT;
            NewTriangle(numP1, numP2, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;
            T2 = FinTT;
            NewTriangle(numP2, numP3, FinTP, TTriangles, MySurface, TPoints);
            FinTT++;

            // REFINING OF THE ADJACENT TRIANGLE

            int numTA = GetNextTriangle(NumTri, 3, TEdges);

            if (numTA >= 0)
            {
                int numP2b = -1;
                int P2bIndex = -1;

                int Edge1b = -1;
                int Edge2b = -1;

                GetInfoTA(numP3, numP1, numTA, TTriangles, ref numP2b, ref P2bIndex, ref Edge1b, ref Edge2b);

                T3 = FinTT;
                NewTriangle(numP1, numP2b, FinTP, TTriangles, MySurface, TPoints);
                FinTT++;
                T4 = FinTT;
                NewTriangle(numP2b, numP3, FinTP, TTriangles, MySurface, TPoints);

                // The new edges
                int E1, E2, E3, E4;

                E1 = FinTE;
                NewEdge(numP2, FinTP, T1, T2, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP3, T2, T4, TEdges);
                FinTE++;
                E3 = FinTE;
                NewEdge(FinTP, numP2b, T4, T3, TEdges);
                FinTE++;
                E4 = FinTE;
                NewEdge(FinTP, numP1, T1, T3, TEdges);

                // The old edges are updated
                OldEdge(myEdges[0], NumTri, T1, TEdges);
                OldEdge(myEdges[1], NumTri, T2, TEdges);
                OldEdge(Edge1b, numTA, T3, TEdges);
                OldEdge(Edge2b, numTA, T4, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[0], E1, E4);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[1], E2, E1);
                TTriangles[T3].LinkEdges2Triangle(TEdges, Edge1b, E3, E4);
                TTriangles[T4].LinkEdges2Triangle(TEdges, Edge2b, E2, E3);

                // The adjacent triangle is killed
                TTriangles[numTA].SetDeflection(-1.0);
                TTriangles[numTA].SetIntersectionPossible(false);
            }
            else
            {
                // only two new triangles
                int E1, E2, E4;

                E1 = FinTE;
                NewEdge(numP2, FinTP, T1, T2, TEdges);
                FinTE++;
                E2 = FinTE;
                NewEdge(FinTP, numP3, T2, -1, TEdges);
                FinTE++;
                E4 = FinTE;
                NewEdge(FinTP, numP1, T1, -1, TEdges);

                // The old edges are updated
                OldEdge(myEdges[0], NumTri, T1, TEdges);
                OldEdge(myEdges[1], NumTri, T2, TEdges);

                // The new triangles are filled with the edges
                TTriangles[T1].LinkEdges2Triangle(TEdges, myEdges[0], E1, E4);
                TTriangles[T2].LinkEdges2Triangle(TEdges, myEdges[1], E2, E1);
            }
        }

        TPoints.IncrementNbItems();

        // make the triangle obsolete
        myDeflection = -1.0;
        myIsIntersectionPossible = false;
    }

    /// <summary>
    /// Splits the triangle, then the new triangles, while their deflection is above the criterion, marking those out of
    /// the box; at most about twice the number of triangles there were before.
    /// </summary>
    internal void MultipleMiddleRefinement(double theRefineCriterion, in Box theBox, int theTriangleNumber,
                                           SurfaceAdaptorBase theSurface, IntPolyh_Array<IntPolyh_Point> TPoints,
                                           IntPolyh_Array<IntPolyh_Triangle> TTriangles, IntPolyh_Array<IntPolyh_Edge> TEdges)
    {
        // Number of triangles before refinement of current triangle
        int FinTTInit = TTriangles.NbItems();
        // Criteria to stop splitting - double of the initial number of triangles,
        // i.e. allow each triangle to be split at least once. Add a constant
        // to allow the splits of triangles to be checked.
        int MaxNbTT = 2 * FinTTInit + 1000;
        // Split the current triangle
        MiddleRefinement(theTriangleNumber, theSurface, TPoints, TTriangles, TEdges);
        // Refine the new triangles
        for (int i = FinTTInit; i < TTriangles.NbItems() && i < MaxNbTT; ++i)
        {
            ref IntPolyh_Triangle aTriangle = ref TTriangles[i];
            if (theBox.IsOut(aTriangle.BoundingBox(TPoints)))
            {
                aTriangle.SetIntersectionPossible(false);
            }
            else if (aTriangle.Deflection() > theRefineCriterion)
            {
                aTriangle.MiddleRefinement(i, theSurface, TPoints, TTriangles, TEdges);
            }
        }
    }

    /// <summary>Sets the edge and its orientation at the side of the triangle that has the edge's points.</summary>
    internal void SetEdgeAndOrientation(in IntPolyh_Edge theEdge, int theEdgeIndex)
    {
        // Points on the edge - pe1, pe2
        int pe1 = theEdge.FirstPoint(), pe2 = theEdge.SecondPoint();
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
            if (pe1 == myPoints[i] && pe2 == myPoints[i1])
            {
                myEdges[i] = theEdgeIndex;
                myEdgesOrientations[i] = 1;
                break;
            }

            if (pe1 == myPoints[i1] && pe2 == myPoints[i])
            {
                myEdges[i] = theEdgeIndex;
                myEdgesOrientations[i] = -1;
                break;
            }
        }
    }

    /// <summary>
    /// The bounding box of the triangle, enlarged by its deflection; computed on the first call and cached (the cache is
    /// never reset, as in OCCT).
    /// </summary>
    internal Box BoundingBox(IntPolyh_Array<IntPolyh_Point> thePoints)
    {
        if (myBox.IsVoid)
        {
            ref readonly IntPolyh_Point aP1 = ref thePoints.Value(myPoints[0]);
            ref readonly IntPolyh_Point aP2 = ref thePoints.Value(myPoints[1]);
            ref readonly IntPolyh_Point aP3 = ref thePoints.Value(myPoints[2]);
            myBox.Add(new Point3D(aP1.X(), aP1.Y(), aP1.Z()));
            myBox.Add(new Point3D(aP2.X(), aP2.Y(), aP2.Z()));
            myBox.Add(new Point3D(aP3.X(), aP3.Y(), aP3.Z()));
            myBox.SetGap(myDeflection + global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        return myBox;
    }

    /// <summary>Writes the triangle with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter theStream)
    {
        theStream.Write($"\nTriangle({IntPolyh_Format.D(i, 3)}) : Points {IntPolyh_Format.D(myPoints[0], 5)} "
                        + $"{IntPolyh_Format.D(myPoints[1], 5)} {IntPolyh_Format.D(myPoints[2], 5)} Edges "
                        + $"{IntPolyh_Format.D(myEdges[0], 5)} {IntPolyh_Format.D(myEdges[1], 5)} "
                        + $"{IntPolyh_Format.D(myEdges[2], 5)} deflection: {IntPolyh_Format.F(myDeflection, 8)} "
                        + $"intersection possible {IntPolyh_Format.D(myIsIntersectionPossible ? 1 : 0, 8)}  "
                        + $"intersection: {IntPolyh_Format.D(myHasIntersection ? 1 : 0, 5)}\n");
    }

    public readonly bool Equals(IntPolyh_Triangle other)
        => myPoints[0] == other.myPoints[0] && myPoints[1] == other.myPoints[1] && myPoints[2] == other.myPoints[2]
           && myEdges[0] == other.myEdges[0] && myEdges[1] == other.myEdges[1] && myEdges[2] == other.myEdges[2]
           && myEdgesOrientations[0] == other.myEdgesOrientations[0]
           && myEdgesOrientations[1] == other.myEdgesOrientations[1]
           && myEdgesOrientations[2] == other.myEdgesOrientations[2] && myHasIntersection == other.myHasIntersection
           && myIsIntersectionPossible == other.myIsIntersectionPossible && myIsDegenerated == other.myIsDegenerated
           && myDeflection.Equals(other.myDeflection) && myBox.Equals(other.myBox);

    public override readonly bool Equals(object? obj) => obj is IntPolyh_Triangle other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(myPoints[0], myPoints[1], myPoints[2]),
                            HashCode.Combine(myEdges[0], myEdges[1], myEdges[2]),
                            HashCode.Combine(myEdgesOrientations[0], myEdgesOrientations[1], myEdgesOrientations[2]),
                            myHasIntersection, myIsIntersectionPossible, myIsDegenerated, myDeflection, myBox);

    public static bool operator ==(in IntPolyh_Triangle theLeft, in IntPolyh_Triangle theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_Triangle theLeft, in IntPolyh_Triangle theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntPolyh_Triangle(P {myPoints[0]}, {myPoints[1]}, {myPoints[2]}; E {myEdges[0]}, {myEdges[1]}, {myEdges[2]}; "
           + $"{Constants.Format("deflection", myDeflection)}, possible {myIsIntersectionPossible}, intersection {myHasIntersection})";
}
