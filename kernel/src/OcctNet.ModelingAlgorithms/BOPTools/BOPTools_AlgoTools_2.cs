// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

// BOPTools_AlgoTools_2.cxx: vertices, edges, ranges and dimensions.
public static partial class BOPTools_AlgoTools
{
    /// <summary>
    /// Raises the tolerance of <paramref name="aVF"/> to cover <paramref name="aNewVertex"/> (distance plus its
    /// tolerance). Writes into <paramref name="aVF"/>.
    /// </summary>
    public static void UpdateVertex(in TopoDS_Vertex aVF, in TopoDS_Vertex aNewVertex)
    {
        double aTolVF, aTolNewVertex, aDist, aNewTol;
        //
        Point3D aPVF = BRep_Tool.Pnt(aVF);
        Point3D aPNewVertex = BRep_Tool.Pnt(aNewVertex);
        aTolVF = BRep_Tool.Tolerance(aVF);
        aTolNewVertex = BRep_Tool.Tolerance(aNewVertex);

        aDist = aPVF.Distance(aPNewVertex);
        aNewTol = aDist + aTolNewVertex;

        if (aNewTol > aTolVF)
        {
            BRep_Builder BB = new();
            BB.UpdateVertex(aVF, aNewTol + DTolerance());
        }
    }

    /// <summary>
    /// Raises the vertex's tolerance to cover the point of the edge's 3D curve at the parameter. Writes into
    /// <paramref name="aV"/>.
    /// </summary>
    public static void UpdateVertex(in TopoDS_Edge aE, double aT, in TopoDS_Vertex aV)
    {
        double aTolV, aDist;
        Point3D aPc;

        Point3D aPv = BRep_Tool.Pnt(aV);
        aTolV = BRep_Tool.Tolerance(aV);

        GeomAdaptor_Curve aCA = new(BRep_Tool.Curve(aE, out _, out _)!);
        aCA.D0(aT, out aPc);
        aDist = aPv.Distance(aPc);
        if (aDist > aTolV)
        {
            BRep_Builder BB = new();
            BB.UpdateVertex(aV, aDist + DTolerance());
        }
    }

    /// <summary>
    /// Raises the vertex's tolerance to cover the point of the intersection curve at the parameter. Writes into
    /// <paramref name="aV"/>.
    /// </summary>
    public static void UpdateVertex(in IntTools_Curve aC, double aT, in TopoDS_Vertex aV)
    {
        double aTolV, aDist;
        Point3D aPc;

        Point3D aPv = BRep_Tool.Pnt(aV);
        aTolV = BRep_Tool.Tolerance(aV);

        GeomAdaptor_Curve aCA = new(aC.Curve()!);
        aCA.D0(aT, out aPc);
        aDist = aPv.Distance(aPc);
        if (aDist > aTolV)
        {
            BRep_Builder BB = new();
            BB.UpdateVertex(aV, aDist + DTolerance());
        }
    }

    /// <summary>
    /// Makes an edge on the intersection curve between the vertices at the parameters, keeping the range as given.
    /// </summary>
    public static void MakeSectEdge(in IntTools_Curve aIC, in TopoDS_Vertex aV1, double aP1, in TopoDS_Vertex aV2, double aP2,
                                    out TopoDS_Edge aNewEdge)
    {
        Geom_Curve aC = aIC.Curve()!;

        BRepBuilderAPI_MakeEdge aMakeEdge = new(aC, aV1, aV2, aP1, aP2);

        // The maker's unfrozen result: the edge is new, and the range is written into it.
        TopoDS_Edge aE = TopoDS.Edge(aMakeEdge.InternalShape());
        //
        // Range must be as it was !
        BRep_Builder aBB = new();
        aBB.Range(aE, aP1, aP2);
        //
        aNewEdge = aE;
    }

    /// <summary>A new edge with the edge's geometry and sub-shapes (an empty copy refilled), and its orientation.</summary>
    public static TopoDS_Edge CopyEdge(in TopoDS_Edge theEdge)
    {
        TopoDS_Edge aNewEdge = TopoDS.Edge(theEdge.Oriented(TopAbs_FORWARD));
        aNewEdge.EmptyCopy();
        for (TopoDS_Iterator it = new(theEdge, false); it.More(); it.Next())
        {
            new BRep_Builder().Add(aNewEdge, it.Value());
        }
        aNewEdge.Orientation(theEdge.Orientation());
        return aNewEdge;
    }

    /// <summary>
    /// A split of the edge between the vertices at the parameters (in increasing order): an empty copy of the edge
    /// with the two vertices and the range, with the edge's orientation.
    /// </summary>
    public static void MakeSplitEdge(in TopoDS_Edge aE, in TopoDS_Vertex aV1, double aP1, in TopoDS_Vertex aV2, double aP2,
                                     out TopoDS_Edge aNewEdge)
    {
        TopoDS_Edge E = TopoDS.Edge(aE.Oriented(TopAbs_FORWARD));
        E.EmptyCopy();
        //
        BRep_Builder BB = new();
        if (!aV1.IsNull())
        {
            if (aP1 < aP2)
            {
                BB.Add(E, TopoDS.Vertex(aV1.Oriented(TopAbs_FORWARD)));
            }
            else
            {
                BB.Add(E, TopoDS.Vertex(aV1.Oriented(TopAbs_REVERSED)));
            }
        }
        if (!aV2.IsNull())
        {
            if (aP1 < aP2)
            {
                BB.Add(E, TopoDS.Vertex(aV2.Oriented(TopAbs_REVERSED)));
            }
            else
            {
                BB.Add(E, TopoDS.Vertex(aV2.Oriented(TopAbs_FORWARD)));
            }
        }

        if (aP1 < aP2)
        {
            BB.Range(E, aP1, aP2);
        }
        else
        {
            BB.Range(E, aP2, aP1);
        }

        aNewEdge = E;
        aNewEdge.Orientation(aE.Orientation());
    }

    /// <summary>
    /// A new vertex at the middle of the two vertices, with the larger tolerance plus half their distance.
    /// </summary>
    public static void MakeNewVertex(in TopoDS_Vertex aV1, in TopoDS_Vertex aV2, out TopoDS_Vertex aNewVertex)
    {
        Point3D aPnt1 = BRep_Tool.Pnt(aV1);
        double aTol1 = BRep_Tool.Tolerance(aV1);

        Point3D aPnt2 = BRep_Tool.Pnt(aV2);
        double aTol2 = BRep_Tool.Tolerance(aV2);

        double aMaxTol, aDist;

        aDist = aPnt1.Distance(aPnt2);
        aMaxTol = (aTol1 > aTol2) ? aTol1 : aTol2;
        aMaxTol = aMaxTol + 0.5 * aDist;

        Xyz aXYZ1 = aPnt1.XYZ();
        Xyz aXYZ2 = aPnt2.XYZ();
        Xyz aNewXYZ = 0.5 * (aXYZ1 + aXYZ2);

        Point3D aNewPnt = new(aNewXYZ);
        BRep_Builder aBB = new();
        aNewVertex = new TopoDS_Vertex();
        aBB.MakeVertex(ref aNewVertex, aNewPnt, aMaxTol);
    }

    /// <summary>A new vertex at the point with the tolerance.</summary>
    public static void MakeNewVertex(in Point3D aP, double aTol, out TopoDS_Vertex aNewVertex)
    {
        BRep_Builder aBB = new();
        aNewVertex = new TopoDS_Vertex();
        aBB.MakeVertex(ref aNewVertex, aP, aTol);
    }

    /// <summary>
    /// A new vertex at the middle of the two edges' points at the parameters, with the larger edge tolerance plus half
    /// their distance.
    /// </summary>
    public static void MakeNewVertex(in TopoDS_Edge aE1, double aParm1, in TopoDS_Edge aE2, double aParm2, out TopoDS_Vertex aNewVertex)
    {
        double aTol1, aTol2, aMaxTol, aDist;

        PointOnEdge(aE1, aParm1, out Point3D aPnt1);
        PointOnEdge(aE2, aParm2, out Point3D aPnt2);

        aTol1 = BRep_Tool.Tolerance(aE1);
        aTol2 = BRep_Tool.Tolerance(aE2);

        aDist = aPnt1.Distance(aPnt2);
        aMaxTol = (aTol1 > aTol2) ? aTol1 : aTol2;
        aMaxTol = aMaxTol + 0.5 * aDist;

        Xyz aXYZ1 = aPnt1.XYZ();
        Xyz aXYZ2 = aPnt2.XYZ();
        Xyz aNewXYZ = 0.5 * (aXYZ1 + aXYZ2);

        Point3D aNewPnt = new(aNewXYZ);
        BRep_Builder aBB = new();
        aNewVertex = new TopoDS_Vertex();
        aBB.MakeVertex(ref aNewVertex, aNewPnt, aMaxTol);
    }

    /// <summary>
    /// A new vertex at the edge's point at the parameter, with the sum of the edge's and the face's tolerances.
    /// </summary>
    public static void MakeNewVertex(in TopoDS_Edge aE1, double aParm1, in TopoDS_Face aF1, out TopoDS_Vertex aNewVertex)
    {
        double aTol1, aTol2, aMaxTol;

        PointOnEdge(aE1, aParm1, out Point3D aPnt);

        aTol1 = BRep_Tool.Tolerance(aE1);
        aTol2 = BRep_Tool.Tolerance(aF1);
        //
        aMaxTol = aTol1 + aTol2 + DTolerance();
        //
        BRep_Builder aBB = new();
        aNewVertex = new TopoDS_Vertex();
        aBB.MakeVertex(ref aNewVertex, aPnt, aMaxTol);
    }

    /// <summary>The point of the edge's 3D curve at the parameter.</summary>
    public static void PointOnEdge(in TopoDS_Edge aE, double aParm, out Point3D aPnt)
    {
        Geom_Curve C1 = BRep_Tool.Curve(aE, out _, out _)!;
        C1.D0(aParm, out aPnt);
    }

    /// <summary>
    /// The range shrunk at both ends by the resolution of twice the two edges' tolerances on the first edge (not for
    /// lines); the range itself if it would become shorter than PConfusion.
    /// </summary>
    public static void CorrectRange(in TopoDS_Edge aE1, in TopoDS_Edge aE2, in IntTools_Range aSR, out IntTools_Range aNewSR)
    {
        int i;
        double aRes, aTolE1, aTolE2, aTF, aTL, dT;
        BRepAdaptor_Curve aBC = new();
        CurveKind aCT;
        //
        aNewSR = aSR;
        //
        aBC.Initialize(aE1);
        aCT = aBC.GetType();
        if (aCT == Line)
        {
            return;
        }
        //
        dT = global::Kerf.Engine.Foundation.Precision.PConfusion();
        aTF = aSR.First();
        aTL = aSR.Last();
        //
        aTolE1 = BRep_Tool.Tolerance(aE1);
        aTolE2 = BRep_Tool.Tolerance(aE2);
        //
        for (i = 0; i < 2; ++i)
        {
            aRes = 2.0 * (aTolE1 + aTolE2);
            //
            if (aCT == BezierCurve || aCT == BSplineCurve || aCT == OffsetCurve
                || aCT == OtherCurve)
            {
                Point3D aP;
                Vector3D aDer;
                if (i == 0)
                {
                    aBC.D1(aTF, out aP, out aDer);
                }
                else
                {
                    aBC.D1(aTL, out aP, out aDer);
                }
                //
                double aMgn = aDer.Magnitude();

                if (aMgn > 1.0e-12)
                {
                    aRes = aRes / aMgn;
                }
                else
                {
                    aRes = aBC.Resolution(aRes);
                }
            } // if (aCT==GeomAbs_BezierCurve||...
            else
            {
                aRes = aBC.Resolution(aRes);
            }
            //
            if (i == 0)
            {
                aNewSR.SetFirst(aTF + aRes);
            }
            else
            {
                aNewSR.SetLast(aTL - aRes);
            }
            //
            if ((aNewSR.Last() - aNewSR.First()) < dT)
            {
                aNewSR = aSR;
            }
            // aNewSR=((aNewSR.Last()-aNewSR.First()) < dT) ? aSR : aNewSR;
        }
    }

    /// <summary>
    /// The range shrunk at both ends by the resolution of the face's tolerance on the edge; the range itself if it
    /// would become shorter than PConfusion.
    /// </summary>
    public static void CorrectRange(in TopoDS_Edge aE, in TopoDS_Face aF, in IntTools_Range aSR, out IntTools_Range aNewSR)
    {
        int i;
        double aRes, aTolF, aTF, aTL, dT;
        BRepAdaptor_Curve aBC = new();
        CurveKind aCT;
        //
        aNewSR = aSR;
        //
        dT = global::Kerf.Engine.Foundation.Precision.PConfusion();
        aTF = aSR.First();
        aTL = aSR.Last();
        //
        aBC.Initialize(aE);
        aCT = aBC.GetType();
        //
        aTolF = BRep_Tool.Tolerance(aF);
        //
        for (i = 0; i < 2; ++i)
        {
            aRes = aTolF;

            if (aCT == BezierCurve || aCT == BSplineCurve || aCT == OffsetCurve
                || aCT == OtherCurve)
            {
                Point3D aP;
                Vector3D aDer;
                if (i == 0)
                {
                    aBC.D1(aTF, out aP, out aDer);
                }
                else
                {
                    aBC.D1(aTL, out aP, out aDer);
                }
                //
                double aMgn = aDer.Magnitude();

                if (aMgn > 1.0e-12)
                {
                    aRes = aRes / aMgn;
                }
                else
                {
                    aRes = aBC.Resolution(aRes);
                }
            } // if (aCT==GeomAbs_BezierCurve||...
            else
            {
                aRes = aBC.Resolution(aRes);
            }
            //
            if (i == 0)
            {
                aNewSR.SetFirst(aTF + aRes);
            }
            else
            {
                aNewSR.SetLast(aTL - aRes);
            }
            //
            if ((aNewSR.Last() - aNewSR.First()) < dT)
            {
                aNewSR = aSR;
            }
        }
    }

    // The dimension of an elementary shape (-1 for compounds).
    private static int dimension(in TopoDS_Shape theS)
    {
        switch (theS.ShapeType())
        {
            case TopAbs_VERTEX:
                return 0;
            case TopAbs_EDGE:
            case TopAbs_WIRE:
                return 1;
            case TopAbs_FACE:
            case TopAbs_SHELL:
                return 2;
            case TopAbs_SOLID:
            case TopAbs_COMPSOLID:
                return 3;
            default:
                return -1;
        }
    }

    /// <summary>
    /// The minimal and maximal dimensions of the shape's elementary parts (compounds explored); -1 and -1 for an empty
    /// shape.
    /// </summary>
    public static void Dimensions(in TopoDS_Shape theS, out int theDMin, out int theDMax)
    {
        theDMin = theDMax = dimension(theS);
        if (theDMax >= 0)
        {
            return;
        }

        NCollection_List<TopoDS_Shape> aLS = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        TreatCompound(theS, aLS, aMFence);
        if (aLS.IsEmpty())
        {
            // empty shape
            theDMin = theDMax = -1;
            return;
        }

        theDMin = 3;
        theDMax = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLS); it.More(); it.Next())
        {
            int aDim = dimension(it.Value());
            if (aDim < theDMin)
            {
                theDMin = aDim;
            }
            if (aDim > theDMax)
            {
                theDMax = aDim;
            }
        }
    }

    /// <summary>The dimension of the shape, or -1 for a shape mixing dimensions (or empty).</summary>
    public static int Dimension(in TopoDS_Shape theS)
    {
        Dimensions(theS, out int aDMin, out int aDMax);
        return (aDMin == aDMax) ? aDMin : -1;
    }

    /// <summary>
    /// Appends the non-compound parts of the shape (exploring compounds) to the list, each once if a fence map is
    /// given.
    /// </summary>
    public static void TreatCompound(in TopoDS_Shape theS, NCollection_List<TopoDS_Shape> theLS, NCollection_Map<TopoDS_Shape>? theMFence = null)
    {
        TopAbs_ShapeEnum aType = theS.ShapeType();
        if (aType != TopAbs_COMPOUND)
        {
            if (theMFence is null || theMFence.Add(theS))
            {
                theLS.Append(theS);
            }
            return;
        }

        for (TopoDS_Iterator it = new(theS); it.More(); it.Next())
        {
            TreatCompound(it.Value(), theLS, theMFence);
        }
    }
}
