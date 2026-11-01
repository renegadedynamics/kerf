// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.HLRAlgo_PolyMask;

namespace Occt;

/// <summary>
/// The triangles of a face for the polygonal hidden-line removal: the projected nodes, the triangles and the hiding
/// triangles' planes, and the hiding of a segment by them.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's; the nested structs are plain data with public fields.</remarks>
public class HLRAlgo_PolyData : Standard_Transient
{
    /// <summary>The face's index and its min-max indices (plain data).</summary>
    public struct FaceIndices
    {
        public int Index;
        public int Min;
        public int Max;

        public FaceIndices()
        {
            Index = 0;
            Min = 0;
            Max = 0;
        }
    }

    /// <summary>The triangle being tested and the segment's parameters and tolerances (plain data).</summary>
    public struct Triangle
    {
        public gp_XY V1;
        public gp_XY V2;
        public gp_XY V3;
        public double Param;
        public double TolParam;
        public double TolAng;
        public double Tolerance;
    }

    private FaceIndices myFaceIndices = new();
    private NCollection_HArray1<gp_XYZ>? myHNodes;
    private NCollection_HArray1<HLRAlgo_TriangleData>? myHTData;
    private NCollection_HArray1<HLRAlgo_PolyHidingData>? myHPHDat;

    public HLRAlgo_PolyData()
    {
    }

    public void HNodes(NCollection_HArray1<gp_XYZ>? HNodes)
    {
        myHNodes = HNodes;
    }

    public void HTData(NCollection_HArray1<HLRAlgo_TriangleData>? HTData)
    {
        myHTData = HTData;
    }

    public void HPHDat(NCollection_HArray1<HLRAlgo_PolyHidingData>? HPHDat)
    {
        myHPHDat = HPHDat;
    }

    public void FaceIndex(int I)
    {
        myFaceIndices.Index = I;
    }

    public int FaceIndex() => myFaceIndices.Index;

    public NCollection_Array1<gp_XYZ> Nodes() => myHNodes!.ChangeArray1();

    public NCollection_Array1<HLRAlgo_TriangleData> TData() => myHTData!.ChangeArray1();

    public NCollection_Array1<HLRAlgo_PolyHidingData> PHDat() => myHPHDat!.ChangeArray1();

    /// <summary>Adds the hiding triangles' nodes to <paramref name="theBox"/>.</summary>
    public void UpdateGlobalMinMax(ref Bnd_Box theBox)
    {
        NCollection_Array1<gp_XYZ> Nodes = myHNodes!.Array1();
        NCollection_Array1<HLRAlgo_TriangleData> TData = myHTData!.Array1();
        int nbT = TData.Upper();

        for (int i = 1; i <= nbT; i++)
        {
            HLRAlgo_TriangleData TD = TData.Value(i);
            if ((TD.Flags & HLRAlgo_PolyMask_FMskHiding) != 0)
            {
                gp_XYZ P1 = Nodes[TD.Node1];
                gp_XYZ P2 = Nodes[TD.Node2];
                gp_XYZ P3 = Nodes[TD.Node3];
                theBox.Update(P1.X(), P1.Y(), P1.Z());
                theBox.Update(P2.X(), P2.Y(), P2.Z());
                theBox.Update(P3.X(), P3.Y(), P3.Z());
            }
        }
    }

    /// <summary>True if the face has hiding triangles.</summary>
    public bool Hiding() => myHPHDat is not null;

    /// <summary>The face's index and min-max indices, by reference.</summary>
    public ref FaceIndices Indices() => ref myFaceIndices;

    /// <summary>Hides the parts of the segment <paramref name="thePoints"/> behind the face's triangles.</summary>
    public void HideByPolyData(in HLRAlgo_BiPoint.PointsT thePoints, ref Triangle theTriangle,
                               ref HLRAlgo_BiPoint.IndicesT theIndices, bool HidingShell, HLRAlgo_EdgeStatus status)
    {
        if (((myFaceIndices.Max - theIndices.MinSeg) & 0x80100200) == 0
                && ((theIndices.MaxSeg - myFaceIndices.Min) & 0x80100000) == 0)
        {
            NCollection_Array1<HLRAlgo_PolyHidingData> PHDat = myHPHDat!.ChangeArray1();
            NCollection_Array1<HLRAlgo_TriangleData> TData = myHTData!.Array1();
            double d1, d2;
            bool NotConnex = false;
            bool isCrossing = false;
            bool toHideBefore = false;
            int TFlag = 0;
            int h, h2 = PHDat.Upper();

            for (h = 1; h <= h2; h++)
            {
                ref HLRAlgo_PolyHidingData PH = ref PHDat[h];
                ref HLRAlgo_PolyHidingData.TriangleIndices aTriangleIndices = ref PH.Indices();
                if (((aTriangleIndices.Max - theIndices.MinSeg) & 0x80100200) == 0
                        && ((theIndices.MaxSeg - aTriangleIndices.Min) & 0x80100000) == 0)
                {
                    HLRAlgo_TriangleData aTriangle = TData[aTriangleIndices.Index];
                    NotConnex = true;
                    if (HidingShell)
                    {
                        if (myFaceIndices.Index == theIndices.FaceConex1)
                        {
                            if (theIndices.Face1Pt1 == aTriangle.Node1)
                            {
                                NotConnex =
                                    theIndices.Face1Pt2 != aTriangle.Node2 && theIndices.Face1Pt2 != aTriangle.Node3;
                            }
                            else if (theIndices.Face1Pt1 == aTriangle.Node2)
                            {
                                NotConnex =
                                    theIndices.Face1Pt2 != aTriangle.Node3 && theIndices.Face1Pt2 != aTriangle.Node1;
                            }
                            else if (theIndices.Face1Pt1 == aTriangle.Node3)
                            {
                                NotConnex =
                                    theIndices.Face1Pt2 != aTriangle.Node1 && theIndices.Face1Pt2 != aTriangle.Node2;
                            }
                        }
                        else if (myFaceIndices.Index == theIndices.FaceConex2)
                        {
                            if (theIndices.Face2Pt1 == aTriangle.Node1)
                            {
                                NotConnex =
                                    theIndices.Face2Pt2 != aTriangle.Node2 && theIndices.Face2Pt2 != aTriangle.Node3;
                            }
                            else if (theIndices.Face2Pt1 == aTriangle.Node2)
                            {
                                NotConnex =
                                    theIndices.Face2Pt2 != aTriangle.Node3 && theIndices.Face2Pt2 != aTriangle.Node1;
                            }
                            else if (theIndices.Face2Pt1 == aTriangle.Node3)
                            {
                                NotConnex =
                                    theIndices.Face2Pt2 != aTriangle.Node1 && theIndices.Face2Pt2 != aTriangle.Node2;
                            }
                        }
                    }
                    if (NotConnex)
                    {
                        ref HLRAlgo_PolyHidingData.PlaneT aPlane = ref PH.Plane();
                        d1 = aPlane.Normal * thePoints.PntP1 - aPlane.D;
                        d2 = aPlane.Normal * thePoints.PntP2 - aPlane.D;
                        if (d1 > theTriangle.Tolerance)
                        {
                            if (d2 < -theTriangle.Tolerance)
                            {
                                theTriangle.Param = d1 / (d1 - d2);
                                toHideBefore = false;
                                isCrossing = true;
                                TFlag = aTriangle.Flags;
                                NCollection_Array1<gp_XYZ> Nodes = myHNodes!.Array1();
                                gp_XYZ P1 = Nodes[aTriangle.Node1];
                                gp_XYZ P2 = Nodes[aTriangle.Node2];
                                gp_XYZ P3 = Nodes[aTriangle.Node3];
                                theTriangle.V1 = new gp_XY(P1.X(), P1.Y());
                                theTriangle.V2 = new gp_XY(P2.X(), P2.Y());
                                theTriangle.V3 = new gp_XY(P3.X(), P3.Y());
                                hideByOneTriangle(thePoints, ref theTriangle, isCrossing, toHideBefore, TFlag, status);
                            }
                        }
                        else if (d1 < -theTriangle.Tolerance)
                        {
                            if (d2 > theTriangle.Tolerance)
                            {
                                theTriangle.Param = d1 / (d1 - d2);
                                toHideBefore = true;
                                isCrossing = true;
                                TFlag = aTriangle.Flags;
                                NCollection_Array1<gp_XYZ> Nodes = myHNodes!.Array1();
                                gp_XYZ P1 = Nodes[aTriangle.Node1];
                                gp_XYZ P2 = Nodes[aTriangle.Node2];
                                gp_XYZ P3 = Nodes[aTriangle.Node3];
                                theTriangle.V1 = new gp_XY(P1.X(), P1.Y());
                                theTriangle.V2 = new gp_XY(P2.X(), P2.Y());
                                theTriangle.V3 = new gp_XY(P3.X(), P3.Y());
                                hideByOneTriangle(thePoints, ref theTriangle, isCrossing, toHideBefore, TFlag, status);
                            }
                            else
                            {
                                isCrossing = false;
                                TFlag = aTriangle.Flags;
                                NCollection_Array1<gp_XYZ> Nodes = myHNodes!.Array1();
                                gp_XYZ P1 = Nodes[aTriangle.Node1];
                                gp_XYZ P2 = Nodes[aTriangle.Node2];
                                gp_XYZ P3 = Nodes[aTriangle.Node3];
                                theTriangle.V1 = new gp_XY(P1.X(), P1.Y());
                                theTriangle.V2 = new gp_XY(P2.X(), P2.Y());
                                theTriangle.V3 = new gp_XY(P3.X(), P3.Y());
                                hideByOneTriangle(thePoints, ref theTriangle, isCrossing, toHideBefore, TFlag, status);
                            }
                        }
                        else if (d2 < -theTriangle.Tolerance)
                        {
                            isCrossing = false;
                            TFlag = aTriangle.Flags;
                            NCollection_Array1<gp_XYZ> Nodes = myHNodes!.Array1();
                            gp_XYZ P1 = Nodes[aTriangle.Node1];
                            gp_XYZ P2 = Nodes[aTriangle.Node2];
                            gp_XYZ P3 = Nodes[aTriangle.Node3];
                            theTriangle.V1 = new gp_XY(P1.X(), P1.Y());
                            theTriangle.V2 = new gp_XY(P2.X(), P2.Y());
                            theTriangle.V3 = new gp_XY(P3.X(), P3.Y());
                            hideByOneTriangle(thePoints, ref theTriangle, isCrossing, toHideBefore, TFlag, status);
                        }
                    }
                }
            }
        }
    }

    private void hideByOneTriangle(in HLRAlgo_BiPoint.PointsT thePoints, ref Triangle theTriangle, bool Crossing,
                                   bool HideBefore, int TrFlags, HLRAlgo_EdgeStatus status)
    {
        bool CrosSeg = false;
        int n1 = 0;
        double pd1 = 0.0, pd2 = 0.0;
        int nn1 = 0, nn2 = 0;
        double pend = 1.0, psta = 0.0, pp = 0.0, pdp = 0.0;
        int npi = -1;
        Span<bool> o = stackalloc bool[] { false, false };
        Span<bool> m = stackalloc bool[] { false, false };
        Span<double> p = stackalloc double[] { 0.0, 0.0 };
        int npiRej = 0;

        {
            gp_XY aD = theTriangle.V2 - theTriangle.V1;
            gp_XY aA = (1 / aD.Modulus()) * new gp_XY(-aD.Y(), aD.X());
            double aDot = aA * theTriangle.V1;
            double d1 = aA * thePoints.PntP12D() - aDot;
            double d2 = aA * thePoints.PntP22D() - aDot;
            if (d1 > theTriangle.Tolerance)
            {
                if (d2 < -theTriangle.Tolerance)
                {
                    n1 = 2;
                    CrosSeg = true;
                }
                else
                {
                    CrosSeg = false;
                }
            }
            else if (d1 < -theTriangle.Tolerance)
            {
                if (d2 > theTriangle.Tolerance)
                {
                    n1 = -1;
                    CrosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > theTriangle.Tolerance)
                {
                    CrosSeg = false;
                }
                else if (d2 < -theTriangle.Tolerance)
                {
                    return;
                }
                else
                {
                    CrosSeg = false;
                    if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin1) != 0)
                    {
                        pd1 = (thePoints.PntP1.X() - theTriangle.V1.X()) / aD.X();
                        pd2 = (thePoints.PntP2.X() - theTriangle.V1.X()) / aD.X();
                    }
                    else
                    {
                        pd1 = (thePoints.PntP1.Y() - theTriangle.V1.Y()) / aD.Y();
                        pd2 = (thePoints.PntP2.Y() - theTriangle.V1.Y()) / aD.Y();
                    }
                    if (pd1 < -theTriangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < theTriangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -theTriangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < theTriangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -theTriangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < theTriangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -theTriangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < theTriangle.TolParam)
                    {
                        nn2 = 4;
                    }
                    else
                    {
                        nn2 = 5;
                    }
                    if (nn1 == 3)
                    {
                        if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 1)
                    {
                        if (nn2 <= 2)
                        {
                            return;
                        }
                        else
                        {
                            psta = -pd1 / (pd2 - pd1);
                            if (nn2 == 5)
                            {
                                pend = (1.0 - pd1) / (pd2 - pd1);
                            }
                        }
                    }
                    else if (nn1 == 5)
                    {
                        if (nn2 >= 4)
                        {
                            return;
                        }
                        else
                        {
                            psta = (pd1 - 1.0) / (pd1 - pd2);
                            if (nn2 == 1)
                            {
                                pend = pd1 / (pd1 - pd2);
                            }
                        }
                    }
                    else if (nn1 == 2)
                    {
                        if (nn2 == 1)
                        {
                            return;
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 4)
                    {
                        if (nn2 == 5)
                        {
                            return;
                        }
                        else if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                    }
                }
            }
            if (CrosSeg)
            {
                double ad1 = d1;
                if (d1 < 0)
                {
                    ad1 = -d1;
                }
                double ad2 = d2;
                if (d2 < 0)
                {
                    ad2 = -d2;
                }
                pp = ad1 / (ad1 + ad2);
                if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin1) != 0)
                {
                    pdp = (thePoints.PntP1.X() + (thePoints.PntP2.X() - thePoints.PntP1.X()) * pp
                                  - theTriangle.V1.X())
                                / aD.X();
                }
                else
                {
                    pdp = (thePoints.PntP1.Y() + (thePoints.PntP2.Y() - thePoints.PntP1.Y()) * pp
                                  - theTriangle.V1.Y())
                                / aD.Y();
                }
                bool OutSideP = false;
                bool Multiple = false;
                if (pdp < -theTriangle.TolParam)
                {
                    OutSideP = true;
                }
                else if (pdp < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;

                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else if (pdp - 1.0 < -theTriangle.TolParam)
                {
                }
                else if (pdp - 1.0 < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;
                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else
                {
                    OutSideP = true;
                }
                if (OutSideP)
                {
                    npiRej++;
                }
                else
                {
                    npi++;
                    if (npi < 2)
                    {
                        p[npi] = pp;
                        o[npi] = n1 == -1;
                        m[npi] = Multiple;
                    }
                // OCCT_DEBUG trace (more than 2 points) left out
                }
            }
        }

        {
            gp_XY aD = theTriangle.V3 - theTriangle.V2;
            gp_XY aA = (1 / aD.Modulus()) * new gp_XY(-aD.Y(), aD.X());
            double aDot = aA * theTriangle.V2;
            double d1 = aA * thePoints.PntP12D() - aDot;
            double d2 = aA * thePoints.PntP22D() - aDot;
            if (d1 > theTriangle.Tolerance)
            {
                if (d2 < -theTriangle.Tolerance)
                {
                    n1 = 2;
                    CrosSeg = true;
                }
                else
                {
                    CrosSeg = false;
                }
            }
            else if (d1 < -theTriangle.Tolerance)
            {
                if (d2 > theTriangle.Tolerance)
                {
                    n1 = -1;
                    CrosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > theTriangle.Tolerance)
                {
                    CrosSeg = false;
                }
                else if (d2 < -theTriangle.Tolerance)
                {
                    return;
                }
                else
                {
                    CrosSeg = false;
                    if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin2) != 0)
                    {
                        pd1 = (thePoints.PntP1.X() - theTriangle.V2.X()) / aD.X();
                        pd2 = (thePoints.PntP2.X() - theTriangle.V2.X()) / aD.X();
                    }
                    else
                    {
                        pd1 = (thePoints.PntP1.Y() - theTriangle.V2.Y()) / aD.Y();
                        pd2 = (thePoints.PntP2.Y() - theTriangle.V2.Y()) / aD.Y();
                    }
                    if (pd1 < -theTriangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < theTriangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -theTriangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < theTriangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -theTriangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < theTriangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -theTriangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < theTriangle.TolParam)
                    {
                        nn2 = 4;
                    }
                    else
                    {
                        nn2 = 5;
                    }
                    if (nn1 == 3)
                    {
                        if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 1)
                    {
                        if (nn2 <= 2)
                        {
                            return;
                        }
                        else
                        {
                            psta = -pd1 / (pd2 - pd1);
                            if (nn2 == 5)
                            {
                                pend = (1.0 - pd1) / (pd2 - pd1);
                            }
                        }
                    }
                    else if (nn1 == 5)
                    {
                        if (nn2 >= 4)
                        {
                            return;
                        }
                        else
                        {
                            psta = (pd1 - 1.0) / (pd1 - pd2);
                            if (nn2 == 1)
                            {
                                pend = pd1 / (pd1 - pd2);
                            }
                        }
                    }
                    else if (nn1 == 2)
                    {
                        if (nn2 == 1)
                        {
                            return;
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 4)
                    {
                        if (nn2 == 5)
                        {
                            return;
                        }
                        else if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                    }
                }
            }
            if (CrosSeg)
            {
                double ad1 = d1;
                if (d1 < 0)
                {
                    ad1 = -d1;
                }
                double ad2 = d2;
                if (d2 < 0)
                {
                    ad2 = -d2;
                }
                pp = ad1 / (ad1 + ad2);
                if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin2) != 0)
                {
                    pdp = (thePoints.PntP1.X() + (thePoints.PntP2.X() - thePoints.PntP1.X()) * pp
                                  - theTriangle.V2.X())
                                / aD.X();
                }
                else
                {
                    pdp = (thePoints.PntP1.Y() + (thePoints.PntP2.Y() - thePoints.PntP1.Y()) * pp
                                  - theTriangle.V2.Y())
                                / aD.Y();
                }
                bool OutSideP = false;
                bool Multiple = false;
                if (pdp < -theTriangle.TolParam)
                {
                    OutSideP = true;
                }
                else if (pdp < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;
                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else if (pdp - 1.0 < -theTriangle.TolParam)
                {
                }
                else if (pdp - 1.0 < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;
                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else
                {
                    OutSideP = true;
                }
                if (OutSideP)
                {
                    npiRej++;
                }
                else
                {
                    npi++;
                    if (npi < 2)
                    {
                        p[npi] = pp;
                        o[npi] = n1 == -1;
                        m[npi] = Multiple;
                    }
                // OCCT_DEBUG trace (more than 2 points) left out
                }
            }
        }

        {
            gp_XY aD = theTriangle.V1 - theTriangle.V3;
            gp_XY aA = (1 / aD.Modulus()) * new gp_XY(-aD.Y(), aD.X());
            double aDot = aA * theTriangle.V3;
            double d1 = aA * thePoints.PntP12D() - aDot;
            double d2 = aA * thePoints.PntP22D() - aDot;
            if (d1 > theTriangle.Tolerance)
            {
                if (d2 < -theTriangle.Tolerance)
                {
                    n1 = 2;
                    CrosSeg = true;
                }
                else
                {
                    CrosSeg = false;
                }
            }
            else if (d1 < -theTriangle.Tolerance)
            {
                if (d2 > theTriangle.Tolerance)
                {
                    n1 = -1;
                    CrosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > theTriangle.Tolerance)
                {
                    CrosSeg = false;
                }
                else if (d2 < -theTriangle.Tolerance)
                {
                    return;
                }
                else
                {
                    CrosSeg = false;
                    if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin3) != 0)
                    {
                        pd1 = (thePoints.PntP1.X() - theTriangle.V3.X()) / aD.X();
                        pd2 = (thePoints.PntP2.X() - theTriangle.V3.X()) / aD.X();
                    }
                    else
                    {
                        pd1 = (thePoints.PntP1.Y() - theTriangle.V3.Y()) / aD.Y();
                        pd2 = (thePoints.PntP2.Y() - theTriangle.V3.Y()) / aD.Y();
                    }
                    if (pd1 < -theTriangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < theTriangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -theTriangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < theTriangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -theTriangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < theTriangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -theTriangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < theTriangle.TolParam)
                    {
                        nn2 = 4;
                    }
                    else
                    {
                        nn2 = 5;
                    }
                    if (nn1 == 3)
                    {
                        if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 1)
                    {
                        if (nn2 <= 2)
                        {
                            return;
                        }
                        else
                        {
                            psta = -pd1 / (pd2 - pd1);
                            if (nn2 == 5)
                            {
                                pend = (1.0 - pd1) / (pd2 - pd1);
                            }
                        }
                    }
                    else if (nn1 == 5)
                    {
                        if (nn2 >= 4)
                        {
                            return;
                        }
                        else
                        {
                            psta = (pd1 - 1.0) / (pd1 - pd2);
                            if (nn2 == 1)
                            {
                                pend = pd1 / (pd1 - pd2);
                            }
                        }
                    }
                    else if (nn1 == 2)
                    {
                        if (nn2 == 1)
                        {
                            return;
                        }
                        else if (nn2 == 5)
                        {
                            pend = (1.0 - pd1) / (pd2 - pd1);
                        }
                    }
                    else if (nn1 == 4)
                    {
                        if (nn2 == 5)
                        {
                            return;
                        }
                        else if (nn2 == 1)
                        {
                            pend = pd1 / (pd1 - pd2);
                        }
                    }
                }
            }
            if (CrosSeg)
            {
                double ad1 = d1;
                if (d1 < 0)
                {
                    ad1 = -d1;
                }
                double ad2 = d2;
                if (d2 < 0)
                {
                    ad2 = -d2;
                }
                pp = ad1 / (ad1 + ad2);
                if ((TrFlags & HLRAlgo_PolyMask_EMskGrALin3) != 0)
                {
                    pdp = (thePoints.PntP1.X() + (thePoints.PntP2.X() - thePoints.PntP1.X()) * pp
                                  - theTriangle.V3.X())
                                / aD.X();
                }
                else
                {
                    pdp = (thePoints.PntP1.Y() + (thePoints.PntP2.Y() - thePoints.PntP1.Y()) * pp
                                  - theTriangle.V3.Y())
                                / aD.Y();
                }
                bool OutSideP = false;
                bool Multiple = false;
                if (pdp < -theTriangle.TolParam)
                {
                    OutSideP = true;
                }
                else if (pdp < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;
                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else if (pdp - 1.0 < -theTriangle.TolParam)
                {
                }
                else if (pdp - 1.0 < theTriangle.TolParam)
                {
                    Multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            OutSideP = true;
                            if (o[l] != (n1 == -1))
                            {
                                if (l == 0 && npi == 1)
                                {
                                    p[0] = p[1];
                                    o[0] = o[1];
                                    m[0] = m[1];
                                }
                                npi--;
                                npiRej++;
                            }
                        }
                    }
                }
                else
                {
                    OutSideP = true;
                }
                if (OutSideP)
                {
                    npiRej++;
                }
                else
                {
                    npi++;
                    if (npi < 2)
                    {
                        p[npi] = pp;
                        o[npi] = n1 == -1;
                        m[npi] = Multiple;
                    }
                // OCCT_DEBUG trace (more than 2 points) left out
                }
            }
        }

        if (npi == -1)
        {
            if (npiRej >= 2)
            {
                return;
            }
        }
        else if (npi == 0)
        {
            if (o[0])
            {
                psta = p[0];
                pend = 1.0;
            }
            else
            {
                psta = 0.0;
                pend = p[0];
            }
        }
        else if (npi == 1)
        {
            if (p[0] > p[1])
            {
                psta = p[1];
                pend = p[0];
            }
            else
            {
                psta = p[0];
                pend = p[1];
            }
        }

        if (Crossing)
        {
            if (HideBefore)
            {
                if (theTriangle.Param - psta < theTriangle.TolParam)
                {
                    return;
                }
                else if (theTriangle.Param < pend)
                {
                    pend = theTriangle.Param;
                }
            }
            else
            {
                if (pend - theTriangle.Param < theTriangle.TolParam)
                {
                    return;
                }
                else if (psta < theTriangle.Param)
                {
                    psta = theTriangle.Param;
                }
            }
        }

        bool total;
        if (psta > 0)
        {
            total = psta < theTriangle.TolParam;
        }
        else
        {
            total = psta > -theTriangle.TolParam;
        }
        if (total)
        {
            double pfin = pend - 1.0;
            if (pfin > 0)
            {
                total = pfin < theTriangle.TolParam;
            }
            else
            {
                total = pfin > -theTriangle.TolParam;
            }
        }
        if (total)
        {
            status.HideAll();
        }
        else
        {
            status.Hide(psta, (float)theTriangle.TolParam, pend, (float)theTriangle.TolParam, false, false);
        }
    }
}
