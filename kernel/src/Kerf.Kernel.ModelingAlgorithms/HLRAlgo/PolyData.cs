// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Occt;
using static Kerf.Engine.HiddenLines.Algorithm.PolyMask;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The triangles of a face for the polygonal hidden-line removal: the projected nodes, the triangles and the hiding
/// triangles' planes, and the hiding of a segment by them.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's; the nested structs are plain data with public fields.</remarks>
internal class PolyData : Transient
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
        public Xy V1;
        public Xy V2;
        public Xy V3;
        public double Param;
        public double TolParam;
        public double TolAng;
        public double Tolerance;
    }

    private FaceIndices _faceIndices = new();
    private NCollection_HArray1<Xyz>? _hNodes;
    private NCollection_HArray1<TriangleData>? _htData;
    private NCollection_HArray1<PolyHidingData>? _hphDat;

    public PolyData()
    {
    }

    public void HNodes(NCollection_HArray1<Xyz>? hNodes)
    {
        _hNodes = hNodes;
    }

    public void HTData(NCollection_HArray1<TriangleData>? htData)
    {
        _htData = htData;
    }

    public void HPHDat(NCollection_HArray1<PolyHidingData>? hphDat)
    {
        _hphDat = hphDat;
    }

    public void FaceIndex(int i)
    {
        _faceIndices.Index = i;
    }

    public int FaceIndex() => _faceIndices.Index;

    public NCollection_Array1<Xyz> Nodes() => _hNodes!.ChangeArray1();

    public NCollection_Array1<TriangleData> TData() => _htData!.ChangeArray1();

    public NCollection_Array1<PolyHidingData> PHDat() => _hphDat!.ChangeArray1();

    /// <summary>Adds the hiding triangles' nodes to <paramref name="box"/>.</summary>
    public void UpdateGlobalMinMax(ref Box box)
    {
        NCollection_Array1<Xyz> nodes = _hNodes!.Array1();
        NCollection_Array1<TriangleData> tData = _htData!.Array1();
        int nbT = tData.Upper();

        for (int i = 1; i <= nbT; i++)
        {
            TriangleData td = tData.Value(i);
            if ((td.Flags & HLRAlgo_PolyMask_FMskHiding) != 0)
            {
                Xyz p1 = nodes[td.Node1];
                Xyz p2 = nodes[td.Node2];
                Xyz p3 = nodes[td.Node3];
                box.Update(p1.X(), p1.Y(), p1.Z());
                box.Update(p2.X(), p2.Y(), p2.Z());
                box.Update(p3.X(), p3.Y(), p3.Z());
            }
        }
    }

    /// <summary>True if the face has hiding triangles.</summary>
    public bool Hiding() => _hphDat is not null;

    /// <summary>The face's index and min-max indices, by reference.</summary>
    public ref FaceIndices Indices() => ref _faceIndices;

    /// <summary>Hides the parts of the segment <paramref name="points"/> behind the face's triangles.</summary>
    public void HideByPolyData(in AlgoBiPoint.PointsT points, ref Triangle triangle,
                               ref AlgoBiPoint.IndicesT indices, bool hidingShell, EdgeStatus status)
    {
        if (((_faceIndices.Max - indices.MinSeg) & 0x80100200) == 0
                && ((indices.MaxSeg - _faceIndices.Min) & 0x80100000) == 0)
        {
            NCollection_Array1<PolyHidingData> phDat = _hphDat!.ChangeArray1();
            NCollection_Array1<TriangleData> tData = _htData!.Array1();
            double d1, d2;
            bool notConnex = false;
            bool isCrossing = false;
            bool toHideBefore = false;
            int tFlag = 0;
            int h, h2 = phDat.Upper();

            for (h = 1; h <= h2; h++)
            {
                ref PolyHidingData ph = ref phDat[h];
                ref PolyHidingData.TriangleIndices triangleIndices = ref ph.Indices();
                if (((triangleIndices.Max - indices.MinSeg) & 0x80100200) == 0
                        && ((indices.MaxSeg - triangleIndices.Min) & 0x80100000) == 0)
                {
                    TriangleData aTriangle = tData[triangleIndices.Index];
                    notConnex = true;
                    if (hidingShell)
                    {
                        if (_faceIndices.Index == indices.FaceConex1)
                        {
                            if (indices.Face1Pt1 == aTriangle.Node1)
                            {
                                notConnex =
                                    indices.Face1Pt2 != aTriangle.Node2 && indices.Face1Pt2 != aTriangle.Node3;
                            }
                            else if (indices.Face1Pt1 == aTriangle.Node2)
                            {
                                notConnex =
                                    indices.Face1Pt2 != aTriangle.Node3 && indices.Face1Pt2 != aTriangle.Node1;
                            }
                            else if (indices.Face1Pt1 == aTriangle.Node3)
                            {
                                notConnex =
                                    indices.Face1Pt2 != aTriangle.Node1 && indices.Face1Pt2 != aTriangle.Node2;
                            }
                        }
                        else if (_faceIndices.Index == indices.FaceConex2)
                        {
                            if (indices.Face2Pt1 == aTriangle.Node1)
                            {
                                notConnex =
                                    indices.Face2Pt2 != aTriangle.Node2 && indices.Face2Pt2 != aTriangle.Node3;
                            }
                            else if (indices.Face2Pt1 == aTriangle.Node2)
                            {
                                notConnex =
                                    indices.Face2Pt2 != aTriangle.Node3 && indices.Face2Pt2 != aTriangle.Node1;
                            }
                            else if (indices.Face2Pt1 == aTriangle.Node3)
                            {
                                notConnex =
                                    indices.Face2Pt2 != aTriangle.Node1 && indices.Face2Pt2 != aTriangle.Node2;
                            }
                        }
                    }
                    if (notConnex)
                    {
                        ref PolyHidingData.PlaneT plane = ref ph.Plane();
                        d1 = plane.Normal * points.PntP1 - plane.D;
                        d2 = plane.Normal * points.PntP2 - plane.D;
                        if (d1 > triangle.Tolerance)
                        {
                            if (d2 < -triangle.Tolerance)
                            {
                                triangle.Param = d1 / (d1 - d2);
                                toHideBefore = false;
                                isCrossing = true;
                                tFlag = aTriangle.Flags;
                                NCollection_Array1<Xyz> nodes = _hNodes!.Array1();
                                Xyz p1 = nodes[aTriangle.Node1];
                                Xyz p2 = nodes[aTriangle.Node2];
                                Xyz p3 = nodes[aTriangle.Node3];
                                triangle.V1 = new Xy(p1.X(), p1.Y());
                                triangle.V2 = new Xy(p2.X(), p2.Y());
                                triangle.V3 = new Xy(p3.X(), p3.Y());
                                HideByOneTriangle(points, ref triangle, isCrossing, toHideBefore, tFlag, status);
                            }
                        }
                        else if (d1 < -triangle.Tolerance)
                        {
                            if (d2 > triangle.Tolerance)
                            {
                                triangle.Param = d1 / (d1 - d2);
                                toHideBefore = true;
                                isCrossing = true;
                                tFlag = aTriangle.Flags;
                                NCollection_Array1<Xyz> nodes = _hNodes!.Array1();
                                Xyz p1 = nodes[aTriangle.Node1];
                                Xyz p2 = nodes[aTriangle.Node2];
                                Xyz p3 = nodes[aTriangle.Node3];
                                triangle.V1 = new Xy(p1.X(), p1.Y());
                                triangle.V2 = new Xy(p2.X(), p2.Y());
                                triangle.V3 = new Xy(p3.X(), p3.Y());
                                HideByOneTriangle(points, ref triangle, isCrossing, toHideBefore, tFlag, status);
                            }
                            else
                            {
                                isCrossing = false;
                                tFlag = aTriangle.Flags;
                                NCollection_Array1<Xyz> nodes = _hNodes!.Array1();
                                Xyz p1 = nodes[aTriangle.Node1];
                                Xyz p2 = nodes[aTriangle.Node2];
                                Xyz p3 = nodes[aTriangle.Node3];
                                triangle.V1 = new Xy(p1.X(), p1.Y());
                                triangle.V2 = new Xy(p2.X(), p2.Y());
                                triangle.V3 = new Xy(p3.X(), p3.Y());
                                HideByOneTriangle(points, ref triangle, isCrossing, toHideBefore, tFlag, status);
                            }
                        }
                        else if (d2 < -triangle.Tolerance)
                        {
                            isCrossing = false;
                            tFlag = aTriangle.Flags;
                            NCollection_Array1<Xyz> nodes = _hNodes!.Array1();
                            Xyz p1 = nodes[aTriangle.Node1];
                            Xyz p2 = nodes[aTriangle.Node2];
                            Xyz p3 = nodes[aTriangle.Node3];
                            triangle.V1 = new Xy(p1.X(), p1.Y());
                            triangle.V2 = new Xy(p2.X(), p2.Y());
                            triangle.V3 = new Xy(p3.X(), p3.Y());
                            HideByOneTriangle(points, ref triangle, isCrossing, toHideBefore, tFlag, status);
                        }
                    }
                }
            }
        }
    }

    private void HideByOneTriangle(in AlgoBiPoint.PointsT points, ref Triangle triangle, bool crossing,
                                   bool hideBefore, int trFlags, EdgeStatus status)
    {
        bool crosSeg = false;
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
            Xy d = triangle.V2 - triangle.V1;
            Xy a = (1 / d.Modulus()) * new Xy(-d.Y(), d.X());
            double dot = a * triangle.V1;
            double d1 = a * points.PntP12D() - dot;
            double d2 = a * points.PntP22D() - dot;
            if (d1 > triangle.Tolerance)
            {
                if (d2 < -triangle.Tolerance)
                {
                    n1 = 2;
                    crosSeg = true;
                }
                else
                {
                    crosSeg = false;
                }
            }
            else if (d1 < -triangle.Tolerance)
            {
                if (d2 > triangle.Tolerance)
                {
                    n1 = -1;
                    crosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > triangle.Tolerance)
                {
                    crosSeg = false;
                }
                else if (d2 < -triangle.Tolerance)
                {
                    return;
                }
                else
                {
                    crosSeg = false;
                    if ((trFlags & HLRAlgo_PolyMask_EMskGrALin1) != 0)
                    {
                        pd1 = (points.PntP1.X() - triangle.V1.X()) / d.X();
                        pd2 = (points.PntP2.X() - triangle.V1.X()) / d.X();
                    }
                    else
                    {
                        pd1 = (points.PntP1.Y() - triangle.V1.Y()) / d.Y();
                        pd2 = (points.PntP2.Y() - triangle.V1.Y()) / d.Y();
                    }
                    if (pd1 < -triangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < triangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -triangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < triangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -triangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < triangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -triangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < triangle.TolParam)
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
            if (crosSeg)
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
                if ((trFlags & HLRAlgo_PolyMask_EMskGrALin1) != 0)
                {
                    pdp = (points.PntP1.X() + (points.PntP2.X() - points.PntP1.X()) * pp
                                  - triangle.V1.X())
                                / d.X();
                }
                else
                {
                    pdp = (points.PntP1.Y() + (points.PntP2.Y() - points.PntP1.Y()) * pp
                                  - triangle.V1.Y())
                                / d.Y();
                }
                bool outSideP = false;
                bool multiple = false;
                if (pdp < -triangle.TolParam)
                {
                    outSideP = true;
                }
                else if (pdp < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;

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
                else if (pdp - 1.0 < -triangle.TolParam)
                {
                }
                else if (pdp - 1.0 < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;
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
                    outSideP = true;
                }
                if (outSideP)
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
                        m[npi] = multiple;
                    }
                // OCCT_DEBUG trace (more than 2 points) left out
                }
            }
        }

        {
            Xy d = triangle.V3 - triangle.V2;
            Xy a = (1 / d.Modulus()) * new Xy(-d.Y(), d.X());
            double dot = a * triangle.V2;
            double d1 = a * points.PntP12D() - dot;
            double d2 = a * points.PntP22D() - dot;
            if (d1 > triangle.Tolerance)
            {
                if (d2 < -triangle.Tolerance)
                {
                    n1 = 2;
                    crosSeg = true;
                }
                else
                {
                    crosSeg = false;
                }
            }
            else if (d1 < -triangle.Tolerance)
            {
                if (d2 > triangle.Tolerance)
                {
                    n1 = -1;
                    crosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > triangle.Tolerance)
                {
                    crosSeg = false;
                }
                else if (d2 < -triangle.Tolerance)
                {
                    return;
                }
                else
                {
                    crosSeg = false;
                    if ((trFlags & HLRAlgo_PolyMask_EMskGrALin2) != 0)
                    {
                        pd1 = (points.PntP1.X() - triangle.V2.X()) / d.X();
                        pd2 = (points.PntP2.X() - triangle.V2.X()) / d.X();
                    }
                    else
                    {
                        pd1 = (points.PntP1.Y() - triangle.V2.Y()) / d.Y();
                        pd2 = (points.PntP2.Y() - triangle.V2.Y()) / d.Y();
                    }
                    if (pd1 < -triangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < triangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -triangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < triangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -triangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < triangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -triangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < triangle.TolParam)
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
            if (crosSeg)
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
                if ((trFlags & HLRAlgo_PolyMask_EMskGrALin2) != 0)
                {
                    pdp = (points.PntP1.X() + (points.PntP2.X() - points.PntP1.X()) * pp
                                  - triangle.V2.X())
                                / d.X();
                }
                else
                {
                    pdp = (points.PntP1.Y() + (points.PntP2.Y() - points.PntP1.Y()) * pp
                                  - triangle.V2.Y())
                                / d.Y();
                }
                bool outSideP = false;
                bool multiple = false;
                if (pdp < -triangle.TolParam)
                {
                    outSideP = true;
                }
                else if (pdp < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;
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
                else if (pdp - 1.0 < -triangle.TolParam)
                {
                }
                else if (pdp - 1.0 < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;
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
                    outSideP = true;
                }
                if (outSideP)
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
                        m[npi] = multiple;
                    }
                // OCCT_DEBUG trace (more than 2 points) left out
                }
            }
        }

        {
            Xy d = triangle.V1 - triangle.V3;
            Xy a = (1 / d.Modulus()) * new Xy(-d.Y(), d.X());
            double dot = a * triangle.V3;
            double d1 = a * points.PntP12D() - dot;
            double d2 = a * points.PntP22D() - dot;
            if (d1 > triangle.Tolerance)
            {
                if (d2 < -triangle.Tolerance)
                {
                    n1 = 2;
                    crosSeg = true;
                }
                else
                {
                    crosSeg = false;
                }
            }
            else if (d1 < -triangle.Tolerance)
            {
                if (d2 > triangle.Tolerance)
                {
                    n1 = -1;
                    crosSeg = true;
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (d2 > triangle.Tolerance)
                {
                    crosSeg = false;
                }
                else if (d2 < -triangle.Tolerance)
                {
                    return;
                }
                else
                {
                    crosSeg = false;
                    if ((trFlags & HLRAlgo_PolyMask_EMskGrALin3) != 0)
                    {
                        pd1 = (points.PntP1.X() - triangle.V3.X()) / d.X();
                        pd2 = (points.PntP2.X() - triangle.V3.X()) / d.X();
                    }
                    else
                    {
                        pd1 = (points.PntP1.Y() - triangle.V3.Y()) / d.Y();
                        pd2 = (points.PntP2.Y() - triangle.V3.Y()) / d.Y();
                    }
                    if (pd1 < -triangle.TolParam)
                    {
                        nn1 = 1;
                    }
                    else if (pd1 < triangle.TolParam)
                    {
                        nn1 = 2;
                    }
                    else if (pd1 - 1.0 < -triangle.TolParam)
                    {
                        nn1 = 3;
                    }
                    else if (pd1 - 1.0 < triangle.TolParam)
                    {
                        nn1 = 4;
                    }
                    else
                    {
                        nn1 = 5;
                    }
                    if (pd2 < -triangle.TolParam)
                    {
                        nn2 = 1;
                    }
                    else if (pd2 < triangle.TolParam)
                    {
                        nn2 = 2;
                    }
                    else if (pd2 - 1.0 < -triangle.TolParam)
                    {
                        nn2 = 3;
                    }
                    else if (pd2 - 1.0 < triangle.TolParam)
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
            if (crosSeg)
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
                if ((trFlags & HLRAlgo_PolyMask_EMskGrALin3) != 0)
                {
                    pdp = (points.PntP1.X() + (points.PntP2.X() - points.PntP1.X()) * pp
                                  - triangle.V3.X())
                                / d.X();
                }
                else
                {
                    pdp = (points.PntP1.Y() + (points.PntP2.Y() - points.PntP1.Y()) * pp
                                  - triangle.V3.Y())
                                / d.Y();
                }
                bool outSideP = false;
                bool multiple = false;
                if (pdp < -triangle.TolParam)
                {
                    outSideP = true;
                }
                else if (pdp < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;
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
                else if (pdp - 1.0 < -triangle.TolParam)
                {
                }
                else if (pdp - 1.0 < triangle.TolParam)
                {
                    multiple = true;

                    for (int l = 0; l <= npi; l++)
                    {
                        if (m[l])
                        {
                            outSideP = true;
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
                    outSideP = true;
                }
                if (outSideP)
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
                        m[npi] = multiple;
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

        if (crossing)
        {
            if (hideBefore)
            {
                if (triangle.Param - psta < triangle.TolParam)
                {
                    return;
                }
                else if (triangle.Param < pend)
                {
                    pend = triangle.Param;
                }
            }
            else
            {
                if (pend - triangle.Param < triangle.TolParam)
                {
                    return;
                }
                else if (psta < triangle.Param)
                {
                    psta = triangle.Param;
                }
            }
        }

        bool total;
        if (psta > 0)
        {
            total = psta < triangle.TolParam;
        }
        else
        {
            total = psta > -triangle.TolParam;
        }
        if (total)
        {
            double pfin = pend - 1.0;
            if (pfin > 0)
            {
                total = pfin < triangle.TolParam;
            }
            else
            {
                total = pfin > -triangle.TolParam;
            }
        }
        if (total)
        {
            status.HideAll();
        }
        else
        {
            status.Hide(psta, (float)triangle.TolParam, pend, (float)triangle.TolParam, false, false);
        }
    }
}
