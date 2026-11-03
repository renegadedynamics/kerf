// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    /// <summary>Treats the degenerated edges: splits them by the pave blocks passing through their vertex.</summary>
    protected void ProcessDE(ProgressRange theRange)
    {
        using var aPSOuter = new ProgressScope(theRange, null, 1);
        //
        // 1. Find degenerated edges
        //-----------------------------------------------------scope f
        //

        for (int anEdgeIndex = 0; anEdgeIndex < myDS!.NbSourceShapes(); ++anEdgeIndex)
        {
            BOPDS_ShapeInfo anEdgeInfo = myDS.ShapeInfo(anEdgeIndex);
            if (anEdgeInfo.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }

            if (anEdgeInfo.HasFlag(out int nF))
            {
                BOPDS_ShapeInfo aSIF = myDS.ShapeInfo(nF);
                int nV = anEdgeInfo.SubShapes().First();
                int nVSD = 0;
                if (myDS.HasShapeSD(nV, ref nVSD))
                {
                    nV = nVSD;
                }

                if (aSIF.ShapeType() == TopAbs_FACE)
                {
                    // 1. Find PaveBlocks that go through nV for nF
                    NCollection_List<BOPDS_PaveBlock> aLPBOut = new();
                    FindPaveBlocks(nV, nF, aLPBOut);
                    if (!aLPBOut.IsEmpty())
                    {
                        //
                        // 2.
                        NCollection_List<BOPDS_PaveBlock> aLPBD = myDS.ChangePaveBlocks(anEdgeIndex);
                        // OCCT's Standard_ASSERT_VOID(!aLPBD.IsEmpty(), ...) is compiled out in release builds.
                        if (aLPBD.IsEmpty())
                        {
                            continue;
                        }
                        BOPDS_PaveBlock aPBD = aLPBD.First();
                        //
                        FillPaves(nV, anEdgeIndex, nF, aLPBOut, aPBD);
                        //
                        myDS.UpdatePaveBlock(aPBD);
                    }
                    //
                    MakeSplitEdge(anEdgeIndex, nF);
                }
                if (aSIF.ShapeType() == TopAbs_EDGE)
                {
                    TopoDS_Edge aDE = TopoDS.Edge(myDS.Shape(anEdgeIndex));
                    TopoDS_Vertex aVn = TopoDS.Vertex(myDS.Shape(nV));
                    //
                    TopoDS_Edge aE = aDE;
                    aE.EmptyCopy();
                    BRep_Builder BB = new();
                    BB.Add(aE, aVn);
                    BB.Degenerated(aE, true);
                    BB.UpdateEdge(aE, global::Kerf.Engine.Foundation.Precision.Confusion());
                    BOPDS_ShapeInfo aSI = new();
                    aSI.SetShapeType(TopAbs_EDGE);
                    aSI.SetShape(aE);
                    int nEn = myDS.Append(aSI);
                    NCollection_List<BOPDS_PaveBlock> aLPBD = myDS.ChangePaveBlocks(anEdgeIndex);
                    BOPDS_PaveBlock aPBD = aLPBD.First();
                    aPBD.SetEdge(nEn);
                }
            }

            if (UserBreak(aPSOuter))
            {
                return;
            }
        }
    }

    /// <summary>Finds the pave blocks of the face (In, On and Sc) that have the vertex as a bound.</summary>
    protected void FindPaveBlocks(int thePaveIndex, int theFaceInfoIndex, NCollection_List<BOPDS_PaveBlock> theFoundBlocks)
    {
        void processPaveBlocks(NCollection_IndexedMap<BOPDS_PaveBlock> thePaveBlocksMap)
        {
            for (int aBlockIndex = 1; aBlockIndex <= thePaveBlocksMap.Extent(); ++aBlockIndex)
            {
                BOPDS_PaveBlock aPaveBlock = thePaveBlocksMap[aBlockIndex];
                aPaveBlock.Indices(out int nV1, out int nV2);
                if (thePaveIndex == nV1 || thePaveIndex == nV2)
                {
                    theFoundBlocks.Append(aPaveBlock);
                }
            }
        }

        BOPDS_FaceInfo aFaceInfo = myDS!.ChangeFaceInfo(theFaceInfoIndex);
        processPaveBlocks(aFaceInfo.PaveBlocksIn());
        processPaveBlocks(aFaceInfo.PaveBlocksOn());
        processPaveBlocks(aFaceInfo.PaveBlocksSc());
    }

    /// <summary>Makes the split edges of the degenerated edge on the face.</summary>
    protected void MakeSplitEdge(int nDE, int nDF)
    {
        int nSp, nV1, nV2, aNbPB;
        double aT1, aT2;
        TopoDS_Edge aDE, aSp = new();
        TopoDS_Vertex aV1, aV2;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        BOPDS_ShapeInfo aSI = new();
        //
        aSI.SetShapeType(TopAbs_EDGE);
        //
        aDE = TopoDS.Edge(myDS!.Shape(nDE));
        aDE.Orientation(TopAbs_FORWARD);
        //
        TopoDS_Face aDF = TopoDS.Face(myDS.Shape(nDF));
        //
        NCollection_List<BOPDS_PaveBlock> aLPB = myDS.ChangePaveBlocks(nDE);
        aNbPB = aLPB.Extent();
        //
        aItLPB.Initialize(aLPB);
        for (; aItLPB.More(); aItLPB.Next())
        {
            BOPDS_PaveBlock aPB = aItLPB.Value();
            //
            BOPDS_Pave aPave1 = aPB.Pave1();
            aPave1.Contents(out nV1, out aT1);
            //
            BOPDS_Pave aPave2 = aPB.Pave2();
            aPave2.Contents(out nV2, out aT2);
            //
            if (myDS.IsNewShape(nV1) || aNbPB > 1)
            {
                aV1 = TopoDS.Vertex(myDS.Shape(nV1));
                aV1.Orientation(TopAbs_FORWARD);
                //
                aV2 = TopoDS.Vertex(myDS.Shape(nV2));
                aV2.Orientation(TopAbs_REVERSED);
                //
                MakeSplitEdge1(aDE, aDF, aV1, aT1, aV2, aT2, out aSp);
                //
                // OCCT reuses one ShapeInfo, which Append copies (decision 631).
                aSI.SetShape(aSp);
                nSp = myDS.Append(aSI);
                aPB.SetEdge(nSp);
            }
            else
            {
                myDS.ChangeShapeInfo(nDE).SetReference(-1);
                aLPB.Clear();
                break;
            }
        }
    }

    /// <summary>
    /// Finds all pave blocks passing through the vertex <paramref name="nVD"/> and intersecting the 2D curve of the
    /// degenerated edge somewhere in the middle, and saves the intersection points as extra paves of the degenerated
    /// edge's pave block for its later splitting.
    /// </summary>
    protected void FillPaves(int nVD, int nED, int nFD, NCollection_List<BOPDS_PaveBlock> aLPBOut, BOPDS_PaveBlock aPBD)
    {
        // Prepare pave to put to pave block as an Extra pave
        BOPDS_Pave aPave = new();
        aPave.SetIndex(nVD);
        //
        TopoDS_Vertex aDV = TopoDS.Vertex(myDS!.Shape(nVD));
        TopoDS_Edge aDE = TopoDS.Edge(myDS.Shape(nED));
        TopoDS_Face aDF = TopoDS.Face(myDS.Shape(nFD));
        //
        double aTolV = BRep_Tool.Tolerance(aDV);
        BRepAdaptor_Surface aBAS = myContext!.SurfaceAdaptor(aDF);
        //
        // 2D intersection tolerance should be computed as a resolution
        // from the tolerance of vertex to resolve the touching cases
        double aTolInt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        // UResolution from the tolerance of the vertex
        double aURes = aBAS.UResolution(aTolV);
        // VResolution from the tolerance of the vertex
        double aVRes = aBAS.VResolution(aTolV);
        //
        aTolInt = Max(aTolInt, Max(aURes, aVRes));
        //
        // Parametric tolerance to compare intersection point with boundaries
        // should be computed as a resolution from the tolerance of vertex
        // in the direction of the 2D curve of degenerated edge
        double aTolCmp = global::Kerf.Engine.Foundation.Precision.PConfusion();
        // Get 2D curve
        Geom2d_Curve? aC2DDE = BRep_Tool.CurveOnSurface(aDE, aDF, out double aTD1, out double aTD2);
        if (aC2DDE is null)
        {
            return;
        }

        // Get direction of the curve
        bool bUDir = Math.Abs(aC2DDE.Value(aTD1).Y() - aC2DDE.Value(aTD2).Y()) < global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        aTolCmp = Max(aTolCmp, (bUDir ? aURes : aVRes));
        //
        // Prepare adaptor for the degenerated edge for intersection
        Geom2dAdaptor_Curve aGAC1 = new();
        aGAC1.Load(aC2DDE, aTD1, aTD2);
        //
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPBOut);
        for (; aItLPB.More(); aItLPB.Next())
        {
            BOPDS_PaveBlock aPB = aItLPB.Value();
            int nE = aPB.Edge();
            if (nE < 0)
            {
                continue;
            }
            TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
            Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, aDF, out double aT1, out double aT2);
            if (aC2D is null)
            {
                continue;
            }
            //
            // Prepare adaptor for the passing edge for intersection
            Geom2dAdaptor_Curve aGAC2 = new();
            //
            Geom2d_Line? aL2D = aC2D as Geom2d_Line;
            if (aL2D is not null)
            {
                aGAC2.Load(aC2D);
            }
            else
            {
                aGAC2.Load(aC2D, aT1, aT2);
            }
            // Intersection
            Geom2dInt_GInter aGInter = new(aGAC1, aGAC2, aTolInt, aTolInt);
            if (aGInter.IsDone() && aGInter.NbPoints() != 0)
            {
                // Analyze intersection points
                int i, aNbPoints = aGInter.NbPoints();
                for (i = 1; i <= aNbPoints; ++i)
                {
                    double aX = aGInter.Point(i).ParamOnFirst();
                    aPave.SetParameter(aX);
                    AddSplitPoint(aPBD, aPave, aTolCmp);
                }
            }
            else
            {
                // If the intersection did not succeed, try the projection of the end point
                // of the curve corresponding to the vertex of degenerated edge
                double aT = (nVD == aPB.Pave1().Index() ? aPB.Pave1().Parameter() : aPB.Pave2().Parameter());
                Point2D aP2d = aC2D.Value(aT);
                Geom2dAPI_ProjectPointOnCurve aProj2d = new(aP2d, aC2DDE, aTD1, aTD2);
                if (aProj2d.NbPoints() != 0)
                {
                    double aX = aProj2d.LowerDistanceParameter();
                    aPave.SetParameter(aX);
                    AddSplitPoint(aPBD, aPave, aTolCmp);
                }
            }
        }
    }

    // A split of the degenerated edge: an empty copy with the two vertices and the range on the face.
    private static void MakeSplitEdge1(in TopoDS_Edge aE, in TopoDS_Face aF, in TopoDS_Vertex aV1, double aP1,
                                       in TopoDS_Vertex aV2, double aP2, out TopoDS_Edge aNewEdge)
    {
        double aTol = 1.0e-7;

        TopoDS_Edge E = aE;

        E.EmptyCopy();
        BRep_Builder BB = new();
        BB.Add(E, aV1);
        BB.Add(E, aV2);

        BB.Range(E, aF, aP1, aP2);

        BB.Degenerated(E, true);

        BB.UpdateEdge(E, aTol);
        aNewEdge = E;
    }

    // Validates the point represented by the pave for the pave block: if it passes the checks, it is added as an
    // extra pave of the pave block for its later splitting. Returns true if the point is added.
    private static bool AddSplitPoint(BOPDS_PaveBlock thePBD, in BOPDS_Pave thePave, double theTol)
    {
        thePBD.Range(out double aTD1, out double aTD2);

        double aT = thePave.Parameter();
        // Check that the parameter is inside the Pave Block
        if (aT - aTD1 < theTol || aTD2 - aT < theTol)
        {
            return false;
        }

        // Check that the pave block does not contain the same parameter
        int anInd = 0;
        if (thePBD.ContainsParameter(aT, theTol, ref anInd))
        {
            return false;
        }

        // Add the point as an Extra pave to the Pave Block for further
        // splitting of the latter
        thePBD.AppendExtPave1(thePave);
        return true;
    }
}
