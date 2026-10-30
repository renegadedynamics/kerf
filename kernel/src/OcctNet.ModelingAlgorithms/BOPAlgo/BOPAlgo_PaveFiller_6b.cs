// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The second half of BOPAlgo_PaveFiller_6.cxx (from getBoundPaves on), ported separately and fused with the first
// half on merge.

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    //=================================================================================================

    private static void getBoundPaves(BOPDS_DS theDS, BOPDS_Curve theNC, Span<int> theNV)
    {
        theNV[0] = theNV[1] = -1;

        // get extreme paves
        BOPDS_PaveBlock aPB = theNC.PaveBlocks().First();
        NCollection_List<BOPDS_Pave> aLP = aPB.ExtPaves();
        int aNbEP = aLP.Extent();
        if (aNbEP == 0)
        {
            return;
        }
        double aTmin = RealLast();
        double aTmax = -aTmin;
        for (NCollection_List<BOPDS_Pave>.Iterator aItLP = new(aLP); aItLP.More(); aItLP.Next())
        {
            BOPDS_Pave aPv = aItLP.Value();
            int nV;
            double aTV;
            aPv.Contents(out nV, out aTV);
            if (aTV < aTmin)
            {
                theNV[0] = aPv.Index();
                aTmin = aTV;
            }
            if (aTV > aTmax)
            {
                theNV[1] = aPv.Index();
                aTmax = aTV;
            }
        }

        // compare extreme vertices with ends of the curve
        IntTools_Curve aIC = theNC.Curve();
        double aT0 = 0.0, aT1 = 0.0;
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT0, ref aT1, ref aP[0], ref aP[1]);
        double aTol = Max(theNC.Tolerance(), theNC.TangentialTolerance());
        aTol += Precision.Confusion();
        for (int j = 0; j < 2; ++j)
        {
            BOPDS_ShapeInfo aSIV = theDS.ShapeInfo(theNV[j]);
            TopoDS_Vertex aV = TopoDS.Vertex(aSIV.Shape());
            int iFlag = BOPTools_AlgoTools.ComputeVV(aV, aP[j], aTol);
            if (iFlag != 0)
            {
                theNV[j] = -1;
            }
        }
    }

    //=================================================================================================

    protected void PutBoundPaveOnCurve(in TopoDS_Face aF1, in TopoDS_Face aF2, BOPDS_Curve aNC, NCollection_List<int> aLVB)
    {
        IntTools_Curve aIC = aNC.Curve();
        Span<double> aT = stackalloc double[2];
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT[0], ref aT[1], ref aP[0], ref aP[1]);
        double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        BOPDS_PaveBlock aPB = aNC.ChangePaveBlock1();
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> aBndNV = stackalloc int[2];
        getBoundPaves(myDS!, aNC, aBndNV);
        //
        double aTolVnew = Precision.Confusion();
        bool isClosed = aP[1].IsEqual(aP[0], aTolVnew);
        if (isClosed && (aBndNV[0] > 0 || aBndNV[1] > 0))
        {
            return;
        }

        for (int j = 0; j < 2; ++j)
        {
            if (aBndNV[j] < 0)
            {
                // no vertex on this end
                if (j != 0 && isClosed)
                {
                    // if curve is closed, process only one bound
                    continue;
                }
                bool bVF = myContext!.IsValidPointForFaces(aP[j], aF1, aF2, aTolR3D);
                if (!bVF)
                {
                    continue;
                }
                BOPTools_AlgoTools.MakeNewVertex(aP[j], aTolR3D, out TopoDS_Vertex aVn);
                BOPTools_AlgoTools.UpdateVertex(aIC, aT[j], aVn);
                aTolVnew = BRep_Tool.Tolerance(aVn);

                BOPDS_ShapeInfo aSIVn = new();
                aSIVn.SetShapeType(TopAbs_VERTEX);
                aSIVn.SetShape(aVn);

                ref Bnd_Box aBox = ref aSIVn.ChangeBox();
                BRepBndLib.Add(aVn, ref aBox);
                aBox.SetGap(aBox.GetGap() + Precision.Confusion());

                int nVn = myDS!.Append(aSIVn);

                BOPDS_Pave aPn = new();
                aPn.SetIndex(nVn);
                aPn.SetParameter(aT[j]);
                aPB.AppendExtPave(aPn);

                aLVB.Append(nVn);
            }
        }
    }

    //=================================================================================================

    protected void PutPavesOnCurve(NCollection_Map<int> theMVOnIn,
                                   NCollection_Map<int> theMVCommon,
                                   BOPDS_Curve theNC,
                                   NCollection_Map<int> theMI,
                                   NCollection_Map<int> theMVEF,
                                   NCollection_DataMap<int, double> theMVTol,
                                   NCollection_DataMap<int, NCollection_List<int>> theDMVLV)
    {
        //
        Bnd_Box aBoxC = theNC.Box();
        double aTolR3D = Max(theNC.Tolerance(), theNC.TangentialTolerance());
        //
        // Put EF vertices first
        // (decision 633: the order of the map decides the order of the extra paves and of the lists of theDMVLV)
        foreach (int nV in theMVEF)
        {
            PutPaveOnCurve(nV, aTolR3D, theNC, theMI, theMVTol, theDMVLV, 2);
        }

        // Put all other vertices
        foreach (int nV in theMVOnIn)
        {
            if (theMVEF.Contains(nV))
            {
                continue;
            }

            if (!theMVCommon.Contains(nV))
            {
                BOPDS_ShapeInfo aSIV = myDS!.ShapeInfo(nV);
                Bnd_Box aBoxV = aSIV.Box();
                //
                if (aBoxC.IsOut(aBoxV))
                {
                    continue;
                }
                if (!myDS.IsNewShape(nV))
                {
                    continue;
                }
            }
            //
            PutPaveOnCurve(nV, aTolR3D, theNC, theMI, theMVTol, theDMVLV, 1);
        }
    }

    //=================================================================================================

    private struct PaveBlockDist
    {
        public BOPDS_PaveBlock PB;
        public double SquareDist; // square distance from vertex to the paveblock
        public double SinAngle;   // sinus of angle between projection vector
        // and tangent at projection point
        public double Tolerance; // tolerance of the section curve
    }

    protected void FilterPavesOnCurves(NCollection_DynamicArray<BOPDS_Curve> theVNC, NCollection_DataMap<int, double> theMVTol)
    {
        // For each vertex found in ExtPaves of pave blocks of section curves
        // collect list of pave blocks with distance to the curve
        NCollection_IndexedDataMap<int, NCollection_List<PaveBlockDist>> aIDMVertPBs = new();
        int i;
        double anEps = gp.Resolution();
        for (i = 0; i < theVNC.Length(); ++i)
        {
            BOPDS_Curve aNC = theVNC[i];
            IntTools_Curve aIC = aNC.Curve();
            double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
            GeomAdaptor_Curve aGAC = new(aIC.Curve()!);
            BOPDS_PaveBlock aPB = aNC.PaveBlocks().First();
            NCollection_List<BOPDS_Pave> aPaves = aPB.ExtPaves();
            NCollection_List<BOPDS_Pave>.Iterator itPaves = new(aPaves);
            for (; itPaves.More(); itPaves.Next())
            {
                BOPDS_Pave aPave = itPaves.Value();
                int nV = aPave.Index();
                TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                // compute distance from vertex to the point on curve with vertex parameter
                gp_Pnt aPV = BRep_Tool.Pnt(aV);
                double aPar = aPave.Parameter();
                aGAC.D1(aPar, out gp_Pnt aPonC, out gp_Vec aD1);
                gp_Vec aProjVec = new(aPV, aPonC);
                double aSqDist = aProjVec.SquareMagnitude();
                double aSqD1Mod = aD1.SquareMagnitude();
                double aSin = aProjVec.CrossSquareMagnitude(aD1);
                if (aSqDist > anEps && aSqD1Mod > anEps)
                {
                    aSin = Math.Sqrt(aSin / aSqDist / aSqD1Mod);
                }
                int anIndex = aIDMVertPBs.FindIndex(nV);
                NCollection_List<PaveBlockDist> pList = anIndex > 0
                    ? aIDMVertPBs.ChangeFromIndex(anIndex)
                    : aIDMVertPBs.ChangeFromIndex(aIDMVertPBs.Add(nV, new NCollection_List<PaveBlockDist>()));
                PaveBlockDist aPBD = new() { PB = aPB, SquareDist = aSqDist, SinAngle = aSin, Tolerance = aTolR3D };
                pList.Append(aPBD);
            }
        }

        // Process each vertex
        const double aSinAngleMin = 0.5; // angle below which projection is suspicious
        for (i = 1; i <= aIDMVertPBs.Extent(); i++)
        {
            int nV = aIDMVertPBs.FindKey(i);
            NCollection_List<PaveBlockDist> aList = aIDMVertPBs[i];
            // Find a pave with minimal distance
            double aMinDist = RealLast();
            BOPDS_PaveBlock? aPBMinDist = null;
            NCollection_List<PaveBlockDist>.Iterator itL = new(aList);
            for (; itL.More(); itL.Next())
            {
                PaveBlockDist aPBD = itL.Value();
                if (aPBD.SquareDist < aMinDist)
                {
                    aMinDist = aPBD.SquareDist;
                    aPBMinDist = aPBD.PB;
                }
            }
            _ = aPBMinDist; // OCCT finds it and never reads it
            // Remove a vertex from a pave block if the distance is greater than the tolerance
            // and there are other pave blocks for which the distance is less than the current.
            // Do not remove a vertex if it is projected on the curve with quite large angle
            // (see test bugs modalg_6 bug27761).

            // Reduce tolerance for the vertex to the value of maximal distance to
            // to section curve on which it will be kept.
            double aMaxDistKept = -1;
            bool isRemoved = false;
            for (itL.Initialize(aList); itL.More(); itL.Next())
            {
                PaveBlockDist aPBD = itL.Value();
                double aCheckDist = 100.0 * Max(aPBD.Tolerance * aPBD.Tolerance, aMinDist);
                if (aPBD.SquareDist > aCheckDist && aPBD.SinAngle < aSinAngleMin)
                {
                    aPBD.PB.RemoveExtPave(nV);
                    isRemoved = true;
                }
                else if (aPBD.SquareDist > aMaxDistKept)
                {
                    aMaxDistKept = aPBD.SquareDist;
                }
            }

            if (isRemoved && aMaxDistKept > 0)
            {
                if (theMVTol.Find(nV, out double pTol))
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                    double aRealTol = Max(pTol, Math.Sqrt(aMaxDistKept) + Precision.Confusion());
                    // OCCT writes the tolerance into the vertex's TShape directly (a frozen vertex throws).
                    ((BRep_TVertex)aV.TShape()!).Tolerance(aRealTol);
                }
            }
        }
    }

    //=================================================================================================

    protected bool ExtendedTolerance(int nV, NCollection_Map<int> aMI, ref double aTolVExt, int aType = 0)
    {
        bool bFound = false;
        if (!(myDS!.IsNewShape(nV)))
        {
            return bFound;
        }
        //
        int i, k, aNbLines, aNbInt;
        double aT11, aT12, aD1, aD2, aD;
        TopoDS_Vertex aV;
        gp_Pnt aPV, aP11, aP12;
        //
        k = 0;
        aNbInt = 2;
        if (aType == 1)
        {
            aNbInt = 1;
        }
        else if (aType == 2)
        {
            k = 1;
        }
        //
        aV = TopoDS.Vertex(myDS.Shape(nV));
        aPV = BRep_Tool.Pnt(aV);
        //
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        //
        for (; k < aNbInt; ++k)
        {
            aNbLines = k == 0 ? aEEs.Length() : aEFs.Length();
            for (i = 0; i < aNbLines; ++i)
            {
                BOPDS_Interf aInt = k == 0 ? aEEs[i] : aEFs[i];
                if (aInt.IndexNew() == nV)
                {
                    if (aMI.Contains(aInt.Index1()) && aMI.Contains(aInt.Index2()))
                    {
                        IntTools_CommonPrt aComPrt = k == 0 ? aEEs[i].CommonPart() : aEFs[i].CommonPart();
                        //
                        TopoDS_Edge aE1 = aComPrt.Edge1();
                        aComPrt.Range1(out aT11, out aT12);
                        BOPTools_AlgoTools.PointOnEdge(aE1, aT11, out aP11);
                        BOPTools_AlgoTools.PointOnEdge(aE1, aT12, out aP12);
                        aD1 = aPV.Distance(aP11);
                        aD2 = aPV.Distance(aP12);
                        aD = (aD1 > aD2) ? aD1 : aD2;
                        if (aD > aTolVExt)
                        {
                            aTolVExt = aD;
                        }
                        return !bFound;
                    } // if (aMI.Contains(aEF.Index1()) && aMI.Contains(aEF.Index2())) {
                } // if (aInt->IndexNew() == nV) {
            } // for (i = 0; i < aNbLines; ++i) {
        } // for (k=0; k<2; ++k) {
        return bFound;
    }

    //=================================================================================================

    protected void GetEFPnts(int nF1, int nF2, NCollection_List<IntSurf_PntOn2S> aListOfPnts)
    {
        int nE, nF, nFOpposite, aNbEFs, i;
        double U1, U2, V1, V2, f, l;
        NCollection_Map<int> aMI = new();
        //
        // collect indexes of all shapes from nF1 and nF2.
        GetFullShapeMap(nF1, aMI);
        GetFullShapeMap(nF2, aMI);
        //
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS!.InterfEF();
        aNbEFs = aEFs.Length();
        //
        for (i = 0; i < aNbEFs; ++i)
        {
            BOPDS_InterfEF aEF = aEFs[i];
            if (aEF.HasIndexNew())
            {
                aEF.Indices(out nE, out nFOpposite);
                if (aMI.Contains(nE) && aMI.Contains(nFOpposite))
                {
                    IntTools_CommonPrt aCP = aEF.CommonPart();
                    double aPar = aCP.VertexParameter1();
                    TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                    TopoDS_Face aFOpposite = TopoDS.Face(myDS.Shape(nFOpposite));
                    //
                    Geom_Curve aCurve = BRep_Tool.Curve(aE, out f, out l)!;
                    //
                    nF = (nFOpposite == nF1) ? nF2 : nF1;
                    TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
                    Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(aE, aF, out f, out l);
                    //
                    GeomAPI_ProjectPointOnSurf aProj = myContext!.ProjPS(aFOpposite);
                    //
                    aCurve.D0(aPar, out gp_Pnt aPoint);
                    IntSurf_PntOn2S aPnt = new();
                    if (aPCurve is not null)
                    {
                        gp_Pnt2d aP2d = aPCurve.Value(aPar);
                        aProj.Perform(aPoint);
                        if (aProj.IsDone())
                        {
                            aProj.LowerDistanceParameters(out U1, out V1);
                            if (nF == nF1)
                            {
                                aPnt.SetValue(aP2d.X(), aP2d.Y(), U1, V1);
                            }
                            else
                            {
                                aPnt.SetValue(U1, V1, aP2d.X(), aP2d.Y());
                            }
                            aListOfPnts.Append(aPnt);
                        }
                    }
                    else
                    {
                        GeomAPI_ProjectPointOnSurf aProj1 = myContext.ProjPS(aF);
                        aProj1.Perform(aPoint);
                        aProj.Perform(aPoint);
                        if (aProj1.IsDone() && aProj.IsDone())
                        {
                            aProj1.LowerDistanceParameters(out U1, out V1);
                            aProj.LowerDistanceParameters(out U2, out V2);
                            if (nF == nF1)
                            {
                                aPnt.SetValue(U1, V1, U2, V2);
                            }
                            else
                            {
                                aPnt.SetValue(U2, V2, U1, V1);
                            }
                            aListOfPnts.Append(aPnt);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void PutEFPavesOnCurve(NCollection_DynamicArray<BOPDS_Curve> theVC,
                                     int theIndex,
                                     NCollection_Map<int> aMI,
                                     NCollection_Map<int> aMVEF,
                                     NCollection_DataMap<int, double> aMVTol,
                                     NCollection_DataMap<int, NCollection_List<int>> aDMVLV)
    {
        if (aMVEF.Extent() == 0)
        {
            return;
        }
        //
        BOPDS_Curve aNC = theVC.Value(theIndex);
        IntTools_Curve aIC = aNC.Curve();
        GeomAbs_CurveType aTypeC;
        aTypeC = aIC.Type();
        if (aTypeC != GeomAbs_BezierCurve && aTypeC != GeomAbs_BSplineCurve)
        {
            return;
        }
        //
        NCollection_Map<int> aMV = new(aMVEF);
        //
        RemoveUsedVertices(theVC, aMV);
        if (aMV.Extent() == 0)
        {
            return;
        }
        //
        double aDist;
        //
        Geom_Curve aC3D = aIC.Curve()!;
        GeomAPI_ProjectPointOnCurve aProjPT = myContext!.ProjPT(aC3D);
        //
        // (decision 633: the order of the map decides the order of the extra paves)
        foreach (int nV in aMV)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
            gp_Pnt aPV = BRep_Tool.Pnt(aV);
            aProjPT.Perform(aPV);
            int aNbPoints = aProjPT.NbPoints();
            if (aNbPoints != 0)
            {
                aDist = aProjPT.LowerDistance();
                PutPaveOnCurve(nV, aDist, aNC, aMI, aMVTol, aDMVLV);
            }
        }
    }

    //=================================================================================================

    protected void PutStickPavesOnCurve(in TopoDS_Face aF1,
                                        in TopoDS_Face aF2,
                                        NCollection_Map<int> aMI,
                                        NCollection_DynamicArray<BOPDS_Curve> theVC,
                                        int theIndex,
                                        NCollection_Map<int> aMVStick,
                                        NCollection_DataMap<int, double> aMVTol,
                                        NCollection_DataMap<int, NCollection_List<int>> aDMVLV)
    {
        BOPDS_Curve aNC = theVC.Value(theIndex);
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> aBndNV = stackalloc int[2];
        getBoundPaves(myDS!, aNC, aBndNV);
        if (aBndNV[0] >= 0 && aBndNV[1] >= 0)
        {
            // both curve ends already have assigned vertices
            return;
        }
        NCollection_Map<int> aMV = new(aMVStick);
        RemoveUsedVertices(theVC, aMV);
        //
        if (aMV.Extent() == 0)
        {
            return;
        }
        //
        Geom_Surface aS1 = BRep_Tool.Surface(aF1)!;
        Geom_Surface aS2 = BRep_Tool.Surface(aF2)!;
        //
        IntTools_Curve aIC = aNC.Curve();
        Geom2d_Curve?[] aC2D = new Geom2d_Curve?[2];
        //
        aC2D[0] = aIC.FirstCurve2d();
        aC2D[1] = aIC.SecondCurve2d();
        if (aC2D[0] is not null && aC2D[1] is not null)
        {
            int m, n;
            double aD, aD2, u, v, aDT2, aScPr, aDScPr;
            Span<double> aTC = stackalloc double[2];
            gp_Pnt[] aPC = [new gp_Pnt(), new gp_Pnt()];
            gp_Pnt aPV;
            gp_Dir[] aDN = [new gp_Dir(), new gp_Dir()];
            gp_Pnt2d aP2D;
            //
            aDT2 = 2e-7;  // the rich criteria
            aDScPr = 5.0e-9; // the creasing criteria
            aIC.Bounds(ref aTC[0], ref aTC[1], ref aPC[0], ref aPC[1]);
            //
            // (decision 633: the order of the map decides which vertex is put first)
            foreach (int nV in aMV)
            {
                TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                aPV = BRep_Tool.Pnt(aV);
                //
                for (m = 0; m < 2; ++m)
                {
                    if (aBndNV[m] >= 0)
                    {
                        continue;
                    }
                    aD2 = aPC[m].SquareDistance(aPV);
                    if (aD2 > aDT2)
                    { // no rich
                        continue;
                    }
                    //
                    for (n = 0; n < 2; ++n)
                    {
                        Geom_Surface aS = (n == 0) ? aS1 : aS2;
                        aC2D[n]!.D0(aTC[m], out aP2D);
                        aP2D.Coord(out u, out v);
                        BOPTools_AlgoTools3D.GetNormalToSurface(aS, u, v, ref aDN[n]);
                    }
                    //
                    aScPr = aDN[0] * aDN[1];
                    if (aScPr < 0.0)
                    {
                        aScPr = -aScPr;
                    }
                    aScPr = 1.0 - aScPr;
                    //
                    if (aScPr > aDScPr)
                    {
                        continue;
                    }
                    //
                    // The intersection curve aIC is vanishing curve (the crease)
                    aD = Math.Sqrt(aD2);
                    //
                    PutPaveOnCurve(nV, aD, aNC, aMI, aMVTol, aDMVLV);
                }
            } // for (jVU=1; jVU=aNbVU; ++jVU) {
        }
    }

    //=================================================================================================

    protected void GetStickVertices(int nF1, int nF2, NCollection_Map<int> aMVStick, NCollection_Map<int> aMVEF, NCollection_Map<int> aMI)
    {
        int nS1, nS2, nVNew, aTypeInt, i;
        //
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = myDS!.InterfVV();
        NCollection_DynamicArray<BOPDS_InterfVE> aVEs = myDS.InterfVE();
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS.InterfVF();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        //
        Span<int> aNbLines = [aVVs.Length(), aVEs.Length(), aEEs.Length(), aVFs.Length(), aEFs.Length()];
        // collect indices of all shapes from nF1 and nF2.
        aMI.Clear();
        GetFullShapeMap(nF1, aMI);
        GetFullShapeMap(nF2, aMI);
        //
        // collect VV, VE, EE, VF interferences
        for (aTypeInt = 0; aTypeInt < 4; ++aTypeInt)
        {
            for (i = 0; i < aNbLines[aTypeInt]; ++i)
            {
                BOPDS_Interf aInt = (aTypeInt == 0)
                                        ? aVVs[i]
                                        : ((aTypeInt == 1) ? aVEs[i]
                                                           : ((aTypeInt == 2) ? aEEs[i]
                                                                              : aVFs[i]));
                if (aInt.HasIndexNew())
                {
                    aInt.Indices(out nS1, out nS2);
                    if (aMI.Contains(nS1) && aMI.Contains(nS2))
                    {
                        nVNew = aInt.IndexNew();
                        myDS.HasShapeSD(nVNew, ref nVNew);
                        aMVStick.Add(nVNew);
                    }
                }
            }
        }
        // collect EF interferences
        for (i = 0; i < aNbLines[4]; ++i)
        {
            BOPDS_InterfEF aInt = aEFs[i];
            if (aInt.HasIndexNew())
            {
                aInt.Indices(out nS1, out nS2);
                if (aMI.Contains(nS1) && aMI.Contains(nS2))
                {
                    nVNew = aInt.IndexNew();
                    myDS.HasShapeSD(nVNew, ref nVNew);
                    aMVStick.Add(nVNew);
                    aMVEF.Add(nVNew);
                }
            }
        }
    }

    //=================================================================================================

    protected void GetFullShapeMap(int nF, NCollection_Map<int> aMI)
    {
        NCollection_List<int>.Iterator aIt = new();
        int nS;
        //
        BOPDS_ShapeInfo aSI = myDS!.ShapeInfo(nF);
        NCollection_List<int> aLI = aSI.SubShapes();
        //
        aMI.Add(nF);
        aIt.Initialize(aLI);
        for (; aIt.More(); aIt.Next())
        {
            nS = aIt.Value();
            aMI.Add(nS);
        }
    }

    //=================================================================================================

    protected void RemoveUsedVertices(NCollection_DynamicArray<BOPDS_Curve> aVC, NCollection_Map<int> aMV)
    {
        if (aMV.IsEmpty())
        {
            return;
        }

        for (int i = 0; i < aVC.Length(); ++i)
        {
            BOPDS_Curve aNC = aVC.Value(i);
            NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.PaveBlocks();
            NCollection_List<BOPDS_PaveBlock>.Iterator itPB = new(aLPBC);
            for (; itPB.More(); itPB.Next())
            {
                BOPDS_PaveBlock aPB = itPB.Value();
                NCollection_List<BOPDS_Pave> aLP = aPB.ExtPaves();
                NCollection_List<BOPDS_Pave>.Iterator itLP = new(aLP);
                for (; itLP.More(); itLP.Next())
                {
                    aMV.Remove(itLP.Value().Index());
                }

                aMV.Remove(aPB.Pave1().Index());
                aMV.Remove(aPB.Pave2().Index());
            }
        }
    }

    //=================================================================================================

    protected void PutPaveOnCurve(int nV,
                                  double aTolR3D,
                                  BOPDS_Curve aNC,
                                  NCollection_Map<int> aMI,
                                  NCollection_DataMap<int, double> aMVTol,
                                  NCollection_DataMap<int, NCollection_List<int>> aDMVLV,
                                  int iCheckExtend = 0)
    {
        bool bIsVertexOnLine;
        double aT = 0.0; // OCCT: uninitialised, written by IsVertexOnLine when it returns true
        //
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        BOPDS_PaveBlock aPB = aNC.PaveBlocks().First();
        IntTools_Curve aIC = aNC.Curve();
        //
        double aTolV = (aMVTol.IsBound(nV) ? aMVTol.Find(nV) : BRep_Tool.Tolerance(aV));

        bIsVertexOnLine = myContext!.IsVertexOnLine(aV, aTolV, aIC, aTolR3D + myFuzzyValue, ref aT);
        if (!bIsVertexOnLine && iCheckExtend != 0 && !myVertsToAvoidExtension.Contains(nV))
        {
            double anExtraTol = aTolV;
            if (ExtendedTolerance(nV, aMI, ref anExtraTol, iCheckExtend))
            {
                bIsVertexOnLine = myContext.IsVertexOnLine(aV, anExtraTol, aIC, aTolR3D + myFuzzyValue, ref aT);
                if (bIsVertexOnLine)
                {
                    gp_Pnt aPOnC = new();
                    aIC.D0(aT, ref aPOnC);
                    aTolV = aPOnC.Distance(BRep_Tool.Pnt(aV));
                }
            }
        }
        //
        if (bIsVertexOnLine)
        {
            // check if aPB contains the parameter aT
            bool bExist;
            int nVUsed = 0; // OCCT: uninitialised, written by ContainsParameter when it returns true
            double aPTol, aDTol;
            //
            aDTol = BOPTools_AlgoTools.DTolerance();
            //
            GeomAdaptor_Curve aGAC = new(aIC.Curve()!);
            aPTol = aGAC.Resolution(Max(aTolR3D, aTolV));
            //
            bExist = aPB.ContainsParameter(aT, aPTol, ref nVUsed);
            if (bExist)
            {
                // use existing pave
                if (!aDMVLV.Find(nVUsed, out NCollection_List<int> pList))
                {
                    pList = aDMVLV.Bound(nVUsed, new NCollection_List<int>());
                    pList.Append(nVUsed);
                    if (!aMVTol.IsBound(nVUsed))
                    {
                        TopoDS_Vertex aVUsed = TopoDS.Vertex(myDS.Shape(nVUsed));
                        aTolV = BRep_Tool.Tolerance(aVUsed);
                        aMVTol.Bind(nVUsed, aTolV);
                    }
                }
                // avoid repeated elements in the list
                NCollection_List<int>.Iterator aItLI = new(pList);
                for (; aItLI.More(); aItLI.Next())
                {
                    if (aItLI.Value() == nV)
                    {
                        break;
                    }
                }
                if (!aItLI.More())
                {
                    pList.Append(nV);
                }
                // save initial tolerance for the vertex
                if (!aMVTol.IsBound(nV))
                {
                    aTolV = BRep_Tool.Tolerance(aV);
                    aMVTol.Bind(nV, aTolV);
                }
            }
            else
            {
                // add new pave
                BOPDS_Pave aPave = new();
                aPave.SetIndex(nV);
                aPave.SetParameter(aT);
                aPB.AppendExtPave(aPave);
                //
                gp_Pnt aP1 = aGAC.Value(aT);
                aTolV = BRep_Tool.Tolerance(aV);
                gp_Pnt aP2 = BRep_Tool.Pnt(aV);
                double aDist = aP1.Distance(aP2);
                if (aTolV < aDist + aDTol)
                {
                    // OCCT writes the vertex in place, also an argument vertex shared by both faces (theMVCommon of
                    // PutPavesOnCurve) in non-destructive mode: a frozen vertex throws here.
                    new BRep_Builder().UpdateVertex(aV, aDist + aDTol);
                    //
                    if (!aMVTol.IsBound(nV))
                    {
                        aMVTol.Bind(nV, aTolV);
                    }
                    //
                    BOPDS_ShapeInfo aSIDS = myDS.ChangeShapeInfo(nV);
                    ref Bnd_Box aBoxDS = ref aSIDS.ChangeBox();
                    BRepBndLib.Add(aV, ref aBoxDS);
                    aBoxDS.SetGap(aBoxDS.GetGap() + Precision.Confusion());
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int theCur,
                                             int nF1,
                                             int nF2,
                                             in TopoDS_Edge theES,
                                             NCollection_IndexedMap<BOPDS_PaveBlock> theMPBOnIn,
                                             BOPTools_BoxTree thePBTree,
                                             NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB,
                                             NCollection_DataMap<TopoDS_Shape, int> theMVI,
                                             NCollection_List<BOPDS_PaveBlock> theLPBC,
                                             NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap,
                                             NCollection_Map<BOPDS_PaveBlock> theMPB)
    {
        Bnd_Box aBoxES = new();
        BRepBndLib.Add(theES, ref aBoxES, false);

        BOPTools_BoxTreeSelector aSelector = new();
        aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxES));
        aSelector.SetBVHSet(thePBTree);
        if (aSelector.Select() == 0)
        {
            return;
        }

        double aTolES = BRep_Tool.Tolerance(theES);

        BOPDS_FaceInfo aFI1 = myDS!.FaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.FaceInfo(nF2);

        for (NCollection_List<int>.Iterator itPB = new(aSelector.Indices()); itPB.More(); itPB.Next())
        {
            BOPDS_PaveBlock aPBF = theMPBOnIn[itPB.Value()];
            if (theMPB.Contains(aPBF))
            {
                continue;
            }

            bool bInF1 = (aFI1.PaveBlocksOn().Contains(aPBF) || aFI1.PaveBlocksIn().Contains(aPBF));
            bool bInF2 = (aFI2.PaveBlocksOn().Contains(aPBF) || aFI2.PaveBlocksIn().Contains(aPBF));
            if (bInF1 && bInF2)
            {
                // Add all common edges for post treatment
                theMPB.Add(aPBF);
                PreparePostTreatFF(theInt, theCur, aPBF, theMSCPB, theMVI, theLPBC);
                continue;
            }

            int nF = bInF1 ? nF2 : nF1;
            if (!myDistances.Find(new BOPDS_Pair(aPBF.OriginalEdge(), nF), out NCollection_List<EdgeRangeDistance> pList))
            {
                continue;
            }

            aPBF.Range(out double aT1, out double aT2);

            double aDist = RealLast();
            for (NCollection_List<EdgeRangeDistance>.Iterator itR = new(pList); itR.More(); itR.Next())
            {
                EdgeRangeDistance aRangeDist = itR.Value();
                if ((aT1 <= aRangeDist.First && aRangeDist.First <= aT2)
                    || (aT1 <= aRangeDist.Last && aRangeDist.Last <= aT2)
                    || (aRangeDist.First <= aT1 && aT1 <= aRangeDist.Last)
                    || (aRangeDist.First <= aT2 && aT2 <= aRangeDist.Last))
                {
                    aDist = aRangeDist.Distance;
                    break;
                }
            }
            if (aDist < RealLast())
            {
                TopoDS_Edge aEF = TopoDS.Edge(myDS.Shape(aPBF.Edge()));
                double aTolSum = aTolES + BRep_Tool.Tolerance(aEF);

                if (aDist <= aTolSum)
                {
                    theMPB.Add(aPBF);
                    PreparePostTreatFF(theInt, theCur, aPBF, theMSCPB, theMVI, theLPBC);

                    if (!thePBFacesMap.Find(aPBF, out NCollection_List<int> pFaces))
                    {
                        pFaces = thePBFacesMap.Bound(aPBF, new NCollection_List<int>());
                    }
                    if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                    {
                        pFaces.Append(nF);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int nF1,
                                             int nF2,
                                             NCollection_IndexedMap<BOPDS_PaveBlock> aMPBOnIn,
                                             BOPTools_BoxTree thePBTree,
                                             NCollection_DataMap<int, NCollection_List<int>> aDMBV,
                                             NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB,
                                             NCollection_DataMap<TopoDS_Shape, int> aMVI,
                                             NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap,
                                             NCollection_Map<BOPDS_PaveBlock> aMPB)
    {
        if (aDMBV.IsEmpty())
        {
            return;
        }
        //
        double aT = 0.0, dummy = 0.0;
        int nV, nE, iC, iFlag;
        NCollection_List<int>.Iterator aItLI = new();
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        BOPDS_InterfFF aFF = aFFs[theInt];
        NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
        //
        BOPDS_FaceInfo aFI1 = myDS.FaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.FaceInfo(nF2);
        //
        // (decision 633: OCCT iterates the DataMap<int, ...> in bucket order; the order of the curves decides the
        // order of the post-treated pave blocks)
        foreach (KeyValuePair<int, NCollection_List<int>> aItBV in aDMBV)
        {
            iC = aItBV.Key;
            NCollection_List<int> aLBV = aItBV.Value;
            //
            BOPDS_Curve aNC = aVC[iC];
            NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.ChangePaveBlocks();
            //
            aItLI.Initialize(aLBV);
            for (; aItLI.More(); aItLI.Next())
            {
                nV = aItLI.Value();
                BOPDS_ShapeInfo aSIV = myDS.ShapeInfo(nV);
                Bnd_Box aBoxV = aSIV.Box();
                TopoDS_Vertex aV = TopoDS.Vertex(aSIV.Shape());
                if (!aMVI.IsBound(aV))
                {
                    continue;
                }
                //
                BOPTools_BoxTreeSelector aSelector = new();
                aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxV));
                aSelector.SetBVHSet(thePBTree);
                if (aSelector.Select() == 0)
                {
                    continue;
                }

                for (NCollection_List<int>.Iterator it = new(aSelector.Indices()); it.More(); it.Next())
                {
                    BOPDS_PaveBlock aPB = aMPBOnIn[it.Value()];
                    if (aPB.Pave1().Index() == nV || aPB.Pave2().Index() == nV)
                    {
                        continue;
                    }
                    //
                    if (aMPB.Contains(aPB))
                    {
                        continue;
                    }
                    nE = aPB.Edge();
                    BOPDS_ShapeInfo aSIE = myDS.ShapeInfo(nE);
                    TopoDS_Edge aE = TopoDS.Edge(aSIE.Shape());
                    //
                    iFlag = myContext!.ComputeVE(aV, aE, ref aT, ref dummy, myFuzzyValue);
                    if (iFlag == 0)
                    {
                        aMPB.Add(aPB);
                        PreparePostTreatFF(theInt, iC, aPB, aMSCPB, aMVI, aLPBC);

                        // Add faces to PB
                        bool bInF1 = (aFI1.PaveBlocksOn().Contains(aPB) || aFI1.PaveBlocksIn().Contains(aPB));
                        bool bInF2 = (aFI2.PaveBlocksOn().Contains(aPB) || aFI2.PaveBlocksIn().Contains(aPB));
                        if (!bInF1 || !bInF2)
                        {
                            // Face without pave block
                            int nF = bInF1 ? nF2 : nF1;
                            if (!thePBFacesMap.Find(aPB, out NCollection_List<int> pFaces))
                            {
                                pFaces = thePBFacesMap.Bound(aPB, new NCollection_List<int>());
                            }
                            // List is expected to be short, so we allow the check here
                            if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                            {
                                pFaces.Append(nF);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void UpdateExistingPaveBlocks(BOPDS_PaveBlock aPBf,
                                            NCollection_List<BOPDS_PaveBlock> aLPB,
                                            NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap)
    {
        if (aLPB.Extent() == 0)
        {
            return;
        }
        //
        int nE;
        bool bCB;
        BOPDS_PaveBlock aPB, aPB1, aPB2, aPB2n;
        BOPDS_CommonBlock? aCB;
        NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new(), aIt1 = new(), aIt2 = new();
        //
        // 1. Remove old pave blocks
        BOPDS_CommonBlock? aCB1 = myDS!.CommonBlock(aPBf);
        bCB = aCB1 is not null;
        NCollection_List<BOPDS_PaveBlock> aLPB1 = new();
        //
        if (bCB)
        {
            aLPB1.Assign(aCB1!.PaveBlocks());
        }
        else
        {
            aLPB1.Append(aPBf);
        }
        aIt1.Initialize(aLPB1);
        for (; aIt1.More(); aIt1.Next())
        {
            aPB1 = aIt1.Value();
            nE = aPB1.OriginalEdge();
            //
            NCollection_List<BOPDS_PaveBlock> aLPB2 = myDS.ChangePaveBlocks(nE);
            aIt2.Initialize(aLPB2);
            for (; aIt2.More(); aIt2.Next())
            {
                aPB2 = aIt2.Value();
                if (aPB1 == aPB2)
                {
                    aLPB2.Remove(aIt2);
                    break;
                }
            }
        }
        //
        // 2. Update pave blocks
        if (bCB)
        {
            // Create new common blocks
            NCollection_List<BOPDS_PaveBlock> aLPBNew = new();
            NCollection_List<int> aFaces = aCB1!.Faces();
            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                BOPDS_PaveBlock aPBValue = aIt.Value();
                BOPDS_Pave aPBValuePave0 = aPBValue.Pave1(), aPBValuePave1 = aPBValue.Pave2();
                //
                aCB = new BOPDS_CommonBlock();
                aIt1.Initialize(aLPB1);
                for (; aIt1.More(); aIt1.Next())
                {
                    aPB2 = aIt1.Value();
                    nE = aPB2.OriginalEdge();
                    //
                    // Create new pave block
                    aPB2n = new BOPDS_PaveBlock();
                    if (aPBValue.OriginalEdge() == nE)
                    {
                        aPB2n.SetPave1(aPBValuePave0);
                        aPB2n.SetPave2(aPBValuePave1);
                    }
                    else
                    {
                        // For the different original edge compute the parameters of paves
                        BOPDS_Pave[] aPave = [new BOPDS_Pave(), new BOPDS_Pave()];

                        if (aPBValuePave0.Index() == aPBValuePave1.Index()
                            && aPB2.Pave1().Index() == aPB2.Pave2().Index())
                        {
                            // still closed
                            aPave[0].SetIndex(aPBValuePave0.Index());
                            aPave[0].SetParameter(aPB2.Pave1().Parameter());
                            aPave[1].SetIndex(aPBValuePave1.Index());
                            aPave[1].SetParameter(aPB2.Pave2().Parameter());
                        }
                        else
                        {
                            for (int i = 0; i < 2; ++i)
                            {
                                int nV = (i == 0 ? aPBValuePave0 : aPBValuePave1).Index();
                                aPave[i].SetIndex(nV);
                                if (nV == aPB2.Pave1().Index())
                                {
                                    aPave[i].SetParameter(aPB2.Pave1().Parameter());
                                }
                                else if (nV == aPB2.Pave2().Index())
                                {
                                    aPave[i].SetParameter(aPB2.Pave2().Parameter());
                                }
                                else
                                {
                                    // Compute the parameter by projecting the point
                                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
                                    TopoDS_Edge aEOr = TopoDS.Edge(myDS.Shape(nE));
                                    double aTOut = 0.0, aDist = 0.0;
                                    int iErr = myContext!.ComputeVE(aV, aEOr, ref aTOut, ref aDist, myFuzzyValue);
                                    if (iErr == 0)
                                    {
                                        aPave[i].SetParameter(aTOut);
                                    }
                                    else
                                    {
                                        // Unable to project - set the parameter of the closest boundary
                                        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(aPB2.Pave1().Index()));
                                        TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(aPB2.Pave2().Index()));
                                        //
                                        gp_Pnt aP = BRep_Tool.Pnt(aV);
                                        gp_Pnt aP1 = BRep_Tool.Pnt(aV1);
                                        gp_Pnt aP2 = BRep_Tool.Pnt(aV2);
                                        //
                                        double aDist1 = aP.SquareDistance(aP1);
                                        double aDist2 = aP.SquareDistance(aP2);
                                        //
                                        aPave[i].SetParameter(aDist1 < aDist2 ? aPB2.Pave1().Parameter()
                                                                              : aPB2.Pave2().Parameter());
                                    }
                                }
                            }
                        }
                        //
                        if (aPave[1].Parameter() < aPave[0].Parameter())
                        {
                            BOPDS_Pave aPaveTmp = aPave[0];
                            aPave[0] = aPave[1];
                            aPave[1] = aPaveTmp;
                        }
                        //
                        aPB2n.SetPave1(aPave[0]);
                        aPB2n.SetPave2(aPave[1]);
                    }
                    //
                    aPB2n.SetEdge(aPBValue.Edge());
                    aPB2n.SetOriginalEdge(nE);
                    aCB.AddPaveBlock(aPB2n);
                    myDS.SetCommonBlock(aPB2n, aCB);
                    myDS.ChangePaveBlocks(nE).Append(aPB2n);
                }
                aCB.SetFaces(aFaces);
                //
                BOPDS_PaveBlock aPBNew = aCB.PaveBlocks().First();
                aLPBNew.Append(aPBNew);
            }
            //
            aLPB.Assign(aLPBNew);
        }
        else
        {
            nE = aPBf.OriginalEdge();
            NCollection_List<BOPDS_PaveBlock> aLPBE = myDS.ChangePaveBlocks(nE);
            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                aPB = aIt.Value();
                aLPBE.Append(aPB);
            }
        }

        // Try to project the edge on the faces
        if (!thePBFacesMap.Find(aPBf, out NCollection_List<int> pLFaces))
        {
            return;
        }
        NCollection_List<int>.Iterator itLF = new(pLFaces);
        for (; itLF.More(); itLF.Next())
        {
            int nF = itLF.Value();
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));

            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                aPB = aIt.Value();
                if (aFI.PaveBlocksOn().Contains(aPB) || aFI.PaveBlocksIn().Contains(aPB))
                {
                    continue;
                }

                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(aPB.Edge()));
                //
                IntTools_EdgeFace anEF = new();
                anEF.SetEdge(aE);
                anEF.SetFace(aF);
                anEF.SetFuzzyValue(myFuzzyValue);
                anEF.SetRange(aPB.Pave1().Parameter(), aPB.Pave2().Parameter());
                anEF.SetContext(myContext);
                anEF.Perform();
                //
                NCollection_Sequence<IntTools_CommonPrt> aCPrts = anEF.CommonParts();
                bool bCoincide = (aCPrts.Length() == 1 && aCPrts.Value(1).Type() == TopAbs_EDGE);
                if (bCoincide)
                {
                    aCB = myDS.CommonBlock(aPB);
                    if (aCB is null)
                    {
                        aCB = new BOPDS_CommonBlock();
                        aCB.AddPaveBlock(aPB);
                        myDS.SetCommonBlock(aPB, aCB);
                    }
                    aCB.AddFace(nF);
                    aFI.ChangePaveBlocksIn().Add(aPB);
                }
            }
        }
    }

    //=================================================================================================

    protected void PutClosingPaveOnCurve(BOPDS_Curve aNC)
    {
        IntTools_Curve aIC = aNC.Curve();
        Geom_Curve? aC3D = aIC.Curve();
        // check 3d curve
        if (aC3D is null)
        {
            return;
        }

        // check bounds
        if (!aIC.HasBounds())
        {
            return;
        }

        // check closeness
        Span<double> aT = stackalloc double[2];
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT[0], ref aT[1], ref aP[0], ref aP[1]);

        // Find the pave which has been put at one of the ends
        BOPDS_Pave aPave = new();
        // Index of the vertex put at one of the ends
        int nV = -1;
        // Keep the opposite parameter
        double aTOp = 0.0;
        // Keep the opposite bounding point
        gp_Pnt aPOp = new();

        BOPDS_PaveBlock aPB = aNC.ChangePaveBlock1();
        NCollection_List<BOPDS_Pave> aLP = aPB.ChangeExtPaves();
        NCollection_List<BOPDS_Pave>.Iterator aItLP = new(aLP);
        for (; aItLP.More() && (nV < 0); aItLP.Next())
        {
            aPave = aItLP.Value();
            double aTC = aPave.Parameter();
            for (int j = 0; j < 2; ++j)
            {
                if (Math.Abs(aTC - aT[j]) < Precision.PConfusion())
                {
                    nV = aPave.Index();
                    aTOp = (j == 0) ? aT[1] : aT[0];
                    aPOp = (j == 0) ? aP[1] : aP[0];
                    break;
                }
            }
        }

        if (nV < 0)
        {
            // No paves on the bounds of the curve
            return;
        }

        // Check if the curve is closed using the tolerance
        // of found vertex
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        double aTolV = BRep_Tool.Tolerance(aV);
        gp_Pnt aPV = BRep_Tool.Pnt(aV);
        // Tolerance for the point on the curve
        double aTolP = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        aTolP += Precision.Confusion();

        double aDistVP = aPV.Distance(aPOp);
        if (aDistVP > aTolV + aTolP)
        {
            // Curve is not closed
            return;
        }

        // Check if there will be valid range on the curve
        double aFirst = 0.0, aLast = 0.0;
        double aNewTolV = Max(aTolV, aDistVP + BOPTools_AlgoTools.DTolerance());
        if (!BRepLib.FindValidRange(new GeomAdaptor_Curve(aC3D),
                                    aIC.Tolerance(),
                                    aT[0],
                                    aP[0],
                                    aNewTolV,
                                    aT[1],
                                    aP[1],
                                    aNewTolV,
                                    ref aFirst,
                                    ref aLast))
        {
            // No valid range
            return;
        }

        if (aNewTolV > aTolV)
        {
            int nVn = UpdateVertex(nV, aNewTolV);
            if (nVn != nV)
            {
                aPave.SetIndex(nVn);
                nV = nVn;
            }
            aTolV = BRep_Tool.Tolerance(TopoDS.Vertex(myDS.Shape(nV)));
        }

        // Add closing pave to the curve
        BOPDS_Pave aNewPave = new();
        aNewPave.SetIndex(nV);
        aNewPave.SetParameter(aTOp);
        aLP.Append(aNewPave);
    }

    //=================================================================================================

    protected void PreparePostTreatFF(int aInt,
                                      int aCur,
                                      BOPDS_PaveBlock aPB,
                                      NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB,
                                      NCollection_DataMap<TopoDS_Shape, int> aMVI,
                                      NCollection_List<BOPDS_PaveBlock> aLPBC)
    {
        int nV1, nV2;
        //
        aLPBC.Append(aPB);
        //
        aPB.Indices(out nV1, out nV2);
        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS!.Shape(nV1));
        TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));
        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(aPB.Edge()));
        // Keep info for post treatment
        BOPDS_CoupleOfPaveBlocks aCPB = new();
        aCPB.SetIndexInterf(aInt);
        aCPB.SetIndex(aCur);
        aCPB.SetPaveBlock1(aPB);
        //
        aMSCPB.Add(aE, aCPB);
        aMVI.Bind(aV1, nV1);
        aMVI.Bind(aV2, nV2);
    }

    //=================================================================================================

    protected bool CheckPlanes(int nF1, int nF2)
    {
        bool bToIntersect;
        int i, iCnt;
        //
        bToIntersect = false;
        //
        BOPDS_FaceInfo aFI1 = myDS!.ChangeFaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
        //
        NCollection_Map<int> aMVIn1 = aFI1.VerticesIn();
        NCollection_Map<int> aMVOn1 = aFI1.VerticesOn();
        //
        iCnt = 0;
        for (i = 0; (i < 2 && !bToIntersect); ++i)
        {
            NCollection_Map<int> aMV2 = (i == 0) ? aFI2.VerticesIn() : aFI2.VerticesOn();
            //
            foreach (int nV2 in aMV2)
            {
                if (aMVIn1.Contains(nV2) || aMVOn1.Contains(nV2))
                {
                    ++iCnt;
                    if (iCnt > 1)
                    {
                        bToIntersect = !bToIntersect;
                        break;
                    }
                }
            }
        }
        //
        return bToIntersect;
    }

    //=================================================================================================

    protected void UpdatePaveBlocks(NCollection_DataMap<int, int> aDMNewSD)
    {
        if (aDMNewSD.IsEmpty())
        {
            return;
        }
        //
        int nSp, aNbPBP, i, j;
        Span<int> nV = stackalloc int[2];
        Span<double> aT = stackalloc double[2];
        bool bCB, bRebuild;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        NCollection_Map<BOPDS_PaveBlock> aMPB = new();
        NCollection_Map<int> aMicroEdges = new();
        //
        NCollection_List<BOPDS_PaveBlock> anAllPBs = new();

        // Get pave blocks of section edges
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.Curves();
            int aNbC = aVNC.Length();
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVNC[j];
                NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.PaveBlocks();
                aItPB.Initialize(aLPBC);
                for (; aItPB.More(); aItPB.Next())
                {
                    anAllPBs.Append(aItPB.Value());
                }
            }
        }

        // Get pave blocks from the pool
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS.ChangePaveBlocksPool();
        aNbPBP = aPBP.Length();
        for (i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                anAllPBs.Append(aItPB.Value());
            }
        }

        // Process all pave blocks
        aItPB.Initialize(anAllPBs);
        for (; aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
            bCB = aCB is not null;
            if (bCB)
            {
                aPB = aCB!.PaveBlock1();
            }
            //
            if (aMPB.Add(aPB))
            {
                bRebuild = false;
                aPB.Indices(out nV[0], out nV[1]);
                aPB.Range(out aT[0], out aT[1]);
                // remember the fact if the edge had different vertices before substitution
                bool wasRegularEdge = (nV[0] != nV[1]);
                //
                for (j = 0; j < 2; ++j)
                {
                    if (aDMNewSD.IsBound(nV[j]))
                    {
                        BOPDS_Pave aPave = new();
                        //
                        nV[j] = aDMNewSD.Find(nV[j]);
                        aPave.SetIndex(nV[j]);
                        aPave.SetParameter(aT[j]);
                        //
                        bRebuild = true;
                        if (j == 0)
                        {
                            aPB.SetPave1(aPave);
                        }
                        else
                        {
                            aPB.SetPave2(aPave);
                        }
                    }
                }
                //
                if (bRebuild)
                {
                    int nE = aPB.Edge();
                    // Check if the Pave Block has the edge set
                    if (nE < 0)
                    {
                        // untouched edge
                        nE = aPB.OriginalEdge();
                    }
                    bool isDegEdge = myDS.ShapeInfo(nE).HasFlag();
                    if (wasRegularEdge && !isDegEdge && nV[0] == nV[1])
                    {
                        // now edge has the same vertex on both ends;
                        // check if it is not a regular closed curve.
                        FillShrunkData(aPB);
                        if (!aPB.HasShrunkData())
                        {
                            // micro edge, so mark it for removal
                            aMicroEdges.Add(nE);
                            continue;
                        }
                    }
                    nSp = SplitEdge(nE, nV[0], aT[0], nV[1], aT[1]);
                    if (bCB)
                    {
                        aCB!.SetEdge(nSp);
                    }
                    else
                    {
                        aPB.SetEdge(nSp);
                    }
                } // if (bRebuild) {
            } // if (aMPB.Add(aPB)) {
        } // for (; aItPB.More(); aItPB.Next()) {
        aMPB.Clear();

        if (aMicroEdges.Extent() != 0)
        {
            RemovePaveBlocks(aMicroEdges);
        }
    }

    //=================================================================================================

    protected void RemovePaveBlocks(NCollection_Map<int> theEdges)
    {
        // Remove all pave blocks referring to input edges:
        //
        // 1. from the Pave Blocks Pool
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length(), i;
        for (i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            //
            NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB);
            while (aItPB.More())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                if (theEdges.Contains(aPB.Edge()))
                {
                    aLPB.Remove(aItPB);
                }
                else
                {
                    aItPB.Next();
                }
            }
        }

        // 2. from section curves
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS.InterfFF();
        int aNbFF = aFFs.Length(), j;
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            // remove from Section pave blocks
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            int aNbC = aVNC.Length();
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVNC[j];
                NCollection_List<BOPDS_PaveBlock> aLPB = aNC.ChangePaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB);
                while (aItPB.More())
                {
                    BOPDS_PaveBlock aPB = aItPB.Value();
                    if (theEdges.Contains(aPB.Edge()))
                    {
                        aLPB.Remove(aItPB);
                    }
                    else
                    {
                        aItPB.Next();
                    }
                }
            }
        }

        // 3. From Face Info
        for (i = 0; i < myDS.NbSourceShapes(); ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            if (!aSI.HasReference())
            {
                continue;
            }

            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(i);
            NCollection_IndexedMap<BOPDS_PaveBlock>[] aIMPB = [aFI.ChangePaveBlocksIn(),
                                                               aFI.ChangePaveBlocksOn(),
                                                               aFI.ChangePaveBlocksSc()];
            for (int k = 0; k < 3; k++)
            {
                int aNbPB = aIMPB[k].Extent(), m;
                for (m = 1; m <= aNbPB; ++m)
                {
                    BOPDS_PaveBlock aPB = aIMPB[k].FindKey(m);
                    if (theEdges.Contains(aPB.Edge()))
                    {
                        break;
                    }
                }
                if (m <= aNbPB)
                {
                    NCollection_IndexedMap<BOPDS_PaveBlock> aMPBCopy = new(aIMPB[k]);
                    aIMPB[k].Clear();
                    for (m = 1; m <= aNbPB; ++m)
                    {
                        BOPDS_PaveBlock aPB = aMPBCopy[m];
                        if (!theEdges.Contains(aPB.Edge()))
                        {
                            aIMPB[k].Add(aPB);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : ToleranceFF
    // purpose  : Computes the TolFF according to the tolerance value and
    //           types of the faces.
    //=======================================================================
    private static double ToleranceFF(BRepAdaptor_Surface aBAS1, BRepAdaptor_Surface aBAS2)
    {
        double aTol1 = aBAS1.Tolerance();
        double aTol2 = aBAS2.Tolerance();
        double aTolFF = Max(aTol1, aTol2);
        //
        bool isAna1, isAna2;
        isAna1 = (aBAS1.GetType() == GeomAbs_Plane || aBAS1.GetType() == GeomAbs_Cylinder
                  || aBAS1.GetType() == GeomAbs_Cone || aBAS1.GetType() == GeomAbs_Sphere
                  || aBAS1.GetType() == GeomAbs_Torus);
        //
        isAna2 = (aBAS2.GetType() == GeomAbs_Plane || aBAS2.GetType() == GeomAbs_Cylinder
                  || aBAS2.GetType() == GeomAbs_Cone || aBAS2.GetType() == GeomAbs_Sphere
                  || aBAS2.GetType() == GeomAbs_Torus);
        //
        if (!isAna1 || !isAna2)
        {
            aTolFF = Max(aTolFF, 5.0e-6);
        }
        return aTolFF;
    }

    //=================================================================================================

    protected void UpdateBlocksWithSharedVertices()
    {
        if (!myNonDestructive)
        {
            return;
        }
        //
        int aNbFF;
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        aNbFF = aFFs.Length();
        if (aNbFF == 0)
        {
            return;
        }
        //
        bool bOnCurve, bHasShapeSD;
        int i, nF1, nF2, aNbC, j, nVSD = 0;
        double aTolV;
        NCollection_Map<int> aMF = new();
        //
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            //
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
            aNbC = aVC.Length();
            if (aNbC == 0)
            {
                continue;
            }
            //
            aFF.Indices(out nF1, out nF2);
            //
            if (aMF.Add(nF1))
            {
                myDS.UpdateFaceInfoOn(nF1);
            }
            if (aMF.Add(nF2))
            {
                myDS.UpdateFaceInfoOn(nF2);
            }
            //
            // Collect old vertices that are shared for nF1, nF2 ->aMI;
            NCollection_Map<int> aMI = new();
            //
            BOPDS_FaceInfo aFI1 = myDS.ChangeFaceInfo(nF1);
            BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
            //
            NCollection_Map<int> aMVOn1 = aFI1.VerticesOn();
            NCollection_Map<int> aMVIn1 = aFI1.VerticesIn();
            NCollection_Map<int> aMVOn2 = aFI2.VerticesOn();
            NCollection_Map<int> aMVIn2 = aFI2.VerticesIn();
            //
            for (j = 0; j < 2; ++j)
            {
                NCollection_Map<int> aMV1 = (j == 0) ? aMVOn1 : aMVIn1;
                foreach (int nV in aMV1)
                {
                    if (myDS.IsNewShape(nV))
                    {
                        continue;
                    }
                    if (aMVOn2.Contains(nV) || aMVIn2.Contains(nV))
                    {
                        aMI.Add(nV);
                    }
                }
            }
            //
            // Try to put vertices aMI on curves
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
                //
                // (decision 633: the order of aMI decides the order of the new vertices UpdateVertex appends)
                foreach (int nV in aMI)
                {
                    //
                    bHasShapeSD = myDS.HasShapeSD(nV, ref nVSD);
                    if (bHasShapeSD)
                    {
                        continue;
                    }
                    //
                    bOnCurve = EstimatePaveOnCurve(nV, aNC, aTolR3D);
                    if (!bOnCurve)
                    {
                        continue;
                    }
                    //
                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
                    aTolV = BRep_Tool.Tolerance(aV);
                    //
                    UpdateVertex(nV, aTolV);
                    myDS.InitPaveBlocksForVertex(nV);
                }
            } // for (j=0; j<aNbC; ++j) {
        } // for (i=0; i<aNbFF; ++i) {
        //
        UpdateCommonBlocksWithSDVertices();
    }

    //=================================================================================================

    protected bool EstimatePaveOnCurve(int nV, BOPDS_Curve aNC, double aTolR3D)
    {
        bool bIsVertexOnLine;
        double aT = 0.0;
        //
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        IntTools_Curve aIC = aNC.Curve();
        //
        bIsVertexOnLine = myContext!.IsVertexOnLine(aV, aIC, aTolR3D, ref aT);
        return bIsVertexOnLine;
    }

    //=================================================================================================

    protected void CorrectToleranceOfSE()
    {
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        NCollection_IndexedDataMap<int, NCollection_List<BOPDS_PaveBlock>> aMVIPBs = new();
        NCollection_Map<int> aMVIToReduce = new();
        // Fence map to avoid repeated checking of the same edge
        NCollection_Map<BOPDS_PaveBlock> aMPB = new();
        //
        // 1. iterate on all sections F-F
        int aNb = aFFs.Length(), i;
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            //
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            int aNbC = aVNC.Length(), k;
            for (k = 0; k < aNbC; ++k)
            {
                BOPDS_Curve aNC = aVNC[k];
                NCollection_List<BOPDS_PaveBlock> aLPB = aNC.ChangePaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
                for (; aItLPB.More();)
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    if (!aPB.HasEdge(out int nE))
                    {
                        aLPB.Remove(aItLPB);
                        continue;
                    }
                    //
                    if (!aMPB.Add(aPB))
                    {
                        aItLPB.Next();
                        continue;
                    }
                    //
                    bool bIsReduced = false;
                    if (aPB.OriginalEdge() < 0)
                    {
                        // It is possible that due to small angle between faces the
                        // common zone between faces can be large and the tangential
                        // tolerance of the curve will be large as well.
                        // Here we're trying to reduce the tolerance of the section
                        // edge using the valid tolerance of the edge.
                        // Note, that if the pave block has created common block with
                        // other edges its valid tolerance could have been changed to
                        // cover all edges in common block (see PostTreatFF() method).
                        double aTolC = aNC.Tolerance();
                        double aTolTang = aNC.TangentialTolerance();
                        if (aTolC < aTolTang)
                        {
                            TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                            double aTolE = BRep_Tool.Tolerance(aE);
                            if (aTolC < aTolE)
                            {
                                // reduce edge tolerance (a new section edge: never an argument's)
                                ((BRep_TEdge)aE.TShape()!).Tolerance(aTolC);
                                bIsReduced = true;
                            }
                        }
                    }
                    //
                    // fill in the map vertex index - pave blocks
                    for (int j = 0; j < 2; j++)
                    {
                        int nV = (j == 0 ? aPB.Pave1().Index() : aPB.Pave2().Index());
                        myDS.HasShapeSD(nV, ref nV);
                        int anIndex = aMVIPBs.FindIndex(nV);
                        NCollection_List<BOPDS_PaveBlock> pPBList = anIndex > 0
                            ? aMVIPBs.ChangeFromIndex(anIndex)
                            : aMVIPBs.ChangeFromIndex(aMVIPBs.Add(nV, new NCollection_List<BOPDS_PaveBlock>()));
                        pPBList.Append(aPB);
                        if (bIsReduced)
                        {
                            aMVIToReduce.Add(nV);
                        }
                    }
                    aItLPB.Next();
                }
            }
        }
        //
        if (aMVIToReduce.IsEmpty())
        {
            return;
        }
        //
        // 2. try to reduce tolerances of connected vertices
        // 2.1 find all other edges containing these connected vertices to avoid
        //     reducing the tolerance to the value less than the tolerances of edges,
        //     i.e. minimal tolerance for the vertex is the max tolerance of the
        //     edges containing this vertex
        NCollection_DataMap<int, double> aMVITol = new();
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS.ChangePaveBlocksPool();
        aNb = aPBP.Length();
        Span<int> nVs = stackalloc int[2];
        for (i = 0; i < aNb; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                if (!aPB.HasEdge(out int nE))
                {
                    continue;
                }
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                double aTolE = BRep_Tool.Tolerance(aE);
                //
                aPB.Indices(out nVs[0], out nVs[1]);
                //
                for (int j = 0; j < 2; j++)
                {
                    if (aMVIToReduce.Contains(nVs[j]))
                    {
                        if (!aMVITol.IsBound(nVs[j]))
                        {
                            aMVITol.Bind(nVs[j], aTolE);
                        }
                        else if (aTolE > aMVITol.Find(nVs[j]))
                        {
                            aMVITol.ChangeFind(nVs[j]) = aTolE;
                        }
                        NCollection_List<BOPDS_PaveBlock> aPBList = aMVIPBs.ChangeFromKey(nVs[j]);
                        aPBList.Append(aPB);
                    }
                }
            }
        }
        //
        // 2.2 reduce tolerances if possible
        aNb = aMVIPBs.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            int nV = aMVIPBs.FindKey(i);
            if (!aMVIToReduce.Contains(nV))
            {
                continue;
            }
            //
            TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
            double aTolV = BRep_Tool.Tolerance(aV);
            double aMaxTol = aMVITol.Find(nV, out double pMaxTol) ? pMaxTol : 0.0;
            // it makes no sense to compute the real tolerance if it is
            // impossible to reduce the tolerance at least 0.1% of the current value
            if (aTolV - aMaxTol < 0.001 * aTolV)
            {
                continue;
            }
            //
            // compute the maximal distance from the vertex to the adjacent edges
            gp_Pnt aP = BRep_Tool.Pnt(aV);
            //
            // Avoid repeated checks
            NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();
            //
            NCollection_List<BOPDS_PaveBlock> aLPB = aMVIPBs.FindFromIndex(i);
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                if (!aMPBFence.Add(aPB))
                {
                    continue;
                }
                int nE = aPB.Edge();
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                BRepAdaptor_Curve aC = new(aE);
                for (int iPave = 0; iPave < 2; ++iPave)
                {
                    BOPDS_Pave aPave = iPave == 0 ? aPB.Pave1() : aPB.Pave2();
                    int nVSD = aPave.Index();
                    myDS.HasShapeSD(nVSD, ref nVSD);
                    if (nVSD != nV)
                    {
                        continue;
                    }
                    //
                    gp_Pnt aPonE = aC.Value(aPave.Parameter());
                    double aDist = aP.Distance(aPonE);
                    aDist += BRep_Tool.Tolerance(aE);
                    if (aDist > aMaxTol)
                    {
                        aMaxTol = aDist;
                    }
                }
            }
            //
            if (aMaxTol < aTolV)
            {
                // OCCT reduces the vertex's tolerance in place, even for an argument vertex in non-destructive mode
                // (a vertex of a section edge that is an untouched argument vertex): a frozen vertex throws here.
                ((BRep_TVertex)aV.TShape()!).Tolerance(aMaxTol);
            }
        }
    }

    //=================================================================================================

    protected void PutSEInOtherFaces(Message_ProgressRange theRange)
    {
        // Try to intersect each section edge with the faces
        // not participated in its creation

        // Get all section edges
        NCollection_IndexedMap<BOPDS_PaveBlock> aMPBScAll = new();

        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        using var aPS = new Message_ProgressScope(theRange, null, 1);
        for (int i = 0; i < aNbFF; ++i)
        {
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFFs[i].Curves();
            int aNbC = aVNC.Length();
            for (int j = 0; j < aNbC; ++j)
            {
                NCollection_List<BOPDS_PaveBlock> aLPBC = aVNC[j].PaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPBC);
                for (; aItPB.More(); aItPB.Next())
                {
                    aMPBScAll.Add(aItPB.Value());
                }
            }
        }
        // Perform intersection of collected pave blocks
        ForceInterfEF(aMPBScAll, aPS.Next(), false);
    }

    //=================================================================================================

    protected void RemoveMicroSectionEdges(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB,
                                           NCollection_IndexedMap<BOPDS_PaveBlock> theMicroPB)
    {
        if (theMSCPB.IsEmpty())
        {
            // no section edges
            return;
        }

        // Get all F/F interferences
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();

        // Build the new map of section edges avoiding the micro edges
        NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aSEPBMap = new(TopTools_ShapeMapHasher.Instance);
        // Analyze all section edges
        int aNbCPB = theMSCPB.Extent();
        for (int i = 1; i <= aNbCPB; ++i)
        {
            TopoDS_Shape aSI = theMSCPB.FindKey(i);
            BOPDS_CoupleOfPaveBlocks aCPB = theMSCPB[i];

            if (aSI.ShapeType() != TopAbs_EDGE)
            {
                // Not an edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            // Get pave block for analysis
            BOPDS_PaveBlock aPB = aCPB.PaveBlock1()!;
            if (aPB.HasEdge())
            {
                // Not a real section edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            if (!BOPTools_AlgoTools.IsMicroEdge(TopoDS.Edge(aSI), myContext, false))
            {
                // Normal edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            // Micro edge is found, avoid it in the <theMSCPB> map
            // and remove from the F/F Intersection info structure

            // Get F/F interference which created this micro edge
            BOPDS_InterfFF aFF = aFFs[aCPB.IndexInterf()];
            // Get curve from which this edge has been created
            BOPDS_Curve aCurve = aFF.ChangeCurves()[aCPB.Index()];
            // Get all section pave blocks created from this curve
            NCollection_List<BOPDS_PaveBlock> aLPBC = aCurve.ChangePaveBlocks();
            // Remove pave block from the list
            for (NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPBC); it.More(); it.Next())
            {
                if (it.Value() == aPB)
                {
                    aLPBC.Remove(it);
                    break;
                }
            }

            // Add the pave block of "micro" edge into outgoing map for
            // unification of its vertices in the PostTreatFF method
            theMicroPB.Add(aPB);
        }

        // Overwrite the old map if necessary (OCCT's assignment: the caller's map takes the new contents)
        if (aSEPBMap.Extent() != theMSCPB.Extent())
        {
            theMSCPB.Clear();
            for (int i = 1; i <= aSEPBMap.Extent(); ++i)
            {
                theMSCPB.Add(aSEPBMap.FindKey(i), aSEPBMap.FindFromIndex(i));
            }
        }
    }

    //=================================================================================================

    protected void RemoveMicroEdges()
    {
        // Fence map
        NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();
        // Resulting map of micro edges
        NCollection_Map<int> aMicroEdges = new();
        // Check all pave blocks from the pool to find the micro edges
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length();
        for (int i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            if (aLPB.Extent() < 2)
            {
                // No splits
                continue;
            }

            if (myDS.ShapeInfo(aLPB.First().OriginalEdge()).HasFlag())
            {
                continue;
            }

            NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPB);
            for (; it.More(); it.Next())
            {
                BOPDS_PaveBlock aPB = it.Value();
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);

                if (aMPBFence.Add(aPBR))
                {
                    aPBR.Indices(out int nV1, out int nV2);
                    if (nV1 == nV2)
                    {
                        // Check if it has the valid range
                        FillShrunkData(aPBR);
                        if (!aPBR.HasShrunkData())
                        {
                            aMicroEdges.Add(aPBR.Edge());
                        }
                    }
                }
            }
        }
        RemovePaveBlocks(aMicroEdges);
    }
}
