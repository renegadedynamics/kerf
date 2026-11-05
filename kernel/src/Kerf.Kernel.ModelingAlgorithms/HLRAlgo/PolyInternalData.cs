// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The internal triangulation of a face for the polygonal hidden-line removal (to update the outlines): the
/// triangles, the segments linking them and the nodes, growing as nodes are inserted on edges.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. OCCT passes the arrays as pointers by reference
/// (<c>NCollection_Array1&lt;T&gt;*&amp;</c>) so that a reallocation (<c>Inc...</c>) moves the caller's pointers;
/// the port passes the arrays by <c>ref</c>, compared by reference as OCCT compares the pointers. OCCT's pointers to
/// elements are <c>ref</c> locals. <see cref="Dump"/> writes OCCT's trace to a writer (stdout by default).
/// </remarks>
internal class PolyInternalData : Transient
{
    private int _nbTData;
    private int _nbPISeg;
    private int _nbPINod;
    private int _mxTData;
    private int _mxPISeg;
    private int _mxPINod;
    private bool _intOutL;
    private bool _planar;
    private NCollection_HArray1<TriangleData> _tData;
    private NCollection_HArray1<PolyInternalSegment> _piSeg;
    private NCollection_HArray1<PolyInternalNode?> _piNod;

    public PolyInternalData(int nbNod, int nbTri)
    {
        _nbTData = nbTri;
        _nbPISeg = 0;
        _nbPINod = nbNod;
        _mxTData = nbTri;
        _mxPINod = nbNod;
        _intOutL = false;
        _planar = false;
        int i;
        _mxPISeg = 2 + (3 * nbTri + nbNod) / 2;
        _tData = new NCollection_HArray1<TriangleData>(0, _mxTData);
        _piSeg = new NCollection_HArray1<PolyInternalSegment>(0, _mxPISeg);
        _piNod = new NCollection_HArray1<PolyInternalNode?>(0, _mxPINod);

        NCollection_Array1<PolyInternalNode?> piNod = _piNod.ChangeArray1();
        for (i = 1; i <= _mxPINod; i++)
        {
            piNod[i] = new PolyInternalNode();
        }
    }

    /// <summary>Builds the segments linking the triangles.</summary>
    public void UpdateLinks(NCollection_Array1<TriangleData> tData,
                            NCollection_Array1<PolyInternalSegment> piSeg,
                            NCollection_Array1<PolyInternalNode?> piNod)
    {
        int n1, n2;
        int find, iiii, icsv = 0;
        ref PolyInternalSegment segIndices = ref Unsafe.NullRef<PolyInternalSegment>();

        bool newSeg = false;
        for (int i = 1; i <= _nbTData; i++)
        {
            ref TriangleData td = ref tData[i];

            ref PolyInternalNode.NodeIndices a1 = ref piNod[td.Node1]!.Indices();
            ref PolyInternalNode.NodeIndices a2 = ref piNod[td.Node2]!.Indices();
            ref PolyInternalNode.NodeIndices a3 = ref piNod[td.Node3]!.Indices();

            {
                n1 = td.Node1;
                n2 = td.Node2;
                newSeg = false;
                if (a1.NdSg == 0 && a2.NdSg == 0)
                {
                    newSeg = true;
                    _nbPISeg++;
                    a1.NdSg = _nbPISeg;
                    a2.NdSg = _nbPISeg;
                }
                else
                {
                    iiii = a1.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            segIndices = ref piSeg[iiii];
                            if (segIndices.LstSg1 == n1)
                            {
                                if (segIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (segIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            _nbPISeg++;
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            segIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        _nbPISeg++;
                        a1.NdSg = _nbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = a2.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                segIndices = ref piSeg[iiii];
                                if (segIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = segIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = segIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            a2.NdSg = _nbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    segIndices = ref piSeg[_nbPISeg];
                    segIndices.LstSg1 = n1;
                    segIndices.LstSg2 = n2;
                    segIndices.Conex1 = i;
                    segIndices.Conex2 = 0;
                    segIndices.NxtSg1 = 0;
                    segIndices.NxtSg2 = 0;
                }
            }

            {
                n1 = td.Node2;
                n2 = td.Node3;
                newSeg = false;
                if (a2.NdSg == 0 && a3.NdSg == 0)
                {
                    newSeg = true;
                    _nbPISeg++;
                    a2.NdSg = _nbPISeg;
                    a3.NdSg = _nbPISeg;
                }
                else
                {
                    iiii = a2.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            segIndices = ref piSeg[iiii];
                            if (segIndices.LstSg1 == n1)
                            {
                                if (segIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (segIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            _nbPISeg++;
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            segIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        _nbPISeg++;
                        a2.NdSg = _nbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = a3.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                segIndices = ref piSeg[iiii];
                                if (segIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = segIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = segIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            a3.NdSg = _nbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    segIndices = ref piSeg[_nbPISeg];
                    segIndices.LstSg1 = n1;
                    segIndices.LstSg2 = n2;
                    segIndices.Conex1 = i;
                    segIndices.Conex2 = 0;
                    segIndices.NxtSg1 = 0;
                    segIndices.NxtSg2 = 0;
                }
            }

            {
                n1 = td.Node3;
                n2 = td.Node1;
                newSeg = false;
                if (a3.NdSg == 0 && a1.NdSg == 0)
                {
                    newSeg = true;
                    _nbPISeg++;
                    a3.NdSg = _nbPISeg;
                    a1.NdSg = _nbPISeg;
                }
                else
                {
                    iiii = a3.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            segIndices = ref piSeg[iiii];
                            if (segIndices.LstSg1 == n1)
                            {
                                if (segIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (segIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = segIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            _nbPISeg++;
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            segIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        _nbPISeg++;
                        a3.NdSg = _nbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = a1.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                segIndices = ref piSeg[iiii];
                                if (segIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = segIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = segIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                segIndices.NxtSg1 = _nbPISeg;
                            }
                            else
                            {
                                segIndices.NxtSg2 = _nbPISeg;
                            }
                        }
                        else
                        {
                            a1.NdSg = _nbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    segIndices = ref piSeg[_nbPISeg];
                    segIndices.LstSg1 = n1;
                    segIndices.LstSg2 = n2;
                    segIndices.Conex1 = i;
                    segIndices.Conex2 = 0;
                    segIndices.NxtSg1 = 0;
                    segIndices.NxtSg2 = 0;
                }
            }
        }
    }

    /// <summary>
    /// Adds a node at (<paramref name="x3"/>, <paramref name="y3"/>, <paramref name="z3"/>), interpolated
    /// between two nodes' data at <paramref name="coef1"/>; returns its index.
    /// </summary>
    public int AddNode(ref PolyInternalNode.NodeData nod1RValues,
                       ref PolyInternalNode.NodeData nod2RValues,
                       ref NCollection_Array1<PolyInternalNode?> piNod1,
                       ref NCollection_Array1<PolyInternalNode?> piNod2, double coef1, double x3,
                       double y3, double z3)
    {
        double coef2 = 1 - coef1;
        IncPINod(ref piNod1, ref piNod2);
        int ip3 = _nbPINod;
        PolyInternalNode pip3 = new();
        piNod1[ip3] = pip3;
        ref PolyInternalNode.NodeData nod3RValues = ref pip3.Data();
        ref PolyInternalNode.NodeIndices nodeIndices = ref pip3.Indices();
        nodeIndices.NdSg = 0;
        nodeIndices.Flag = 0;
        nod3RValues.Point = new Xyz(x3, y3, z3);
        nod3RValues.UV = coef2 * nod1RValues.UV + coef1 * nod2RValues.UV;
        nod3RValues.Scal = nod1RValues.Scal * coef2 + nod2RValues.Scal * coef1;
        Xyz xyz = coef2 * nod1RValues.Normal + coef1 * nod2RValues.Normal;
        double norm = xyz.Modulus();
        if (norm > 0)
        {
            nod3RValues.Normal = (1 / norm) * xyz;
        }
        else
        {
            nod3RValues.Normal = new Xyz(1.0, 0.0, 0.0);
            // OCCT_DEBUG trace left out
        }
        return ip3;
    }

    /// <summary>Inserts the node <paramref name="ip3"/> on the segment (<paramref name="ip1"/>, <paramref name="ip2"/>), splitting its triangles.</summary>
    public void UpdateLinks(int ip1, int ip2, int ip3, ref NCollection_Array1<TriangleData> tData1,
                            ref NCollection_Array1<TriangleData> tData2,
                            ref NCollection_Array1<PolyInternalSegment> piSeg1,
                            ref NCollection_Array1<PolyInternalSegment> piSeg2,
                            ref NCollection_Array1<PolyInternalNode?> piNod1,
                            ref NCollection_Array1<PolyInternalNode?> piNod2)
    {
        int find, iiii, iisv, icsv, iip2 = 0, cnx1 = 0, cnx2 = 0;
        ref PolyInternalSegment segIndices = ref Unsafe.NullRef<PolyInternalSegment>();
        ref PolyInternalSegment segIndices2 = ref Unsafe.NullRef<PolyInternalSegment>();
        find = 0;
        iisv = 0;
        icsv = 0;
        IncPISeg(ref piSeg1, ref piSeg2);
        IncPISeg(ref piSeg1, ref piSeg2);
        IncPISeg(ref piSeg1, ref piSeg2);
        _nbPISeg--;
        _nbPISeg--;
        IncTData(ref tData1, ref tData2);
        IncTData(ref tData1, ref tData2);
        _nbTData--;
        _nbTData--;
        ref PolyInternalNode.NodeIndices nodIndices1 = ref piNod1[ip1]!.Indices();
        ref PolyInternalNode.NodeIndices nodIndices2 = ref piNod1[ip2]!.Indices();
        ref PolyInternalNode.NodeIndices nodIndices3 = ref piNod1[ip3]!.Indices();
        iiii = nodIndices2.NdSg;

        while (iiii != 0 && find == 0)
        {
            segIndices2 = ref piSeg1[iiii];
            if (segIndices2.LstSg1 == ip2)
            {
                if (segIndices2.LstSg2 == ip1)
                {
                    find = iiii;
                    cnx1 = segIndices2.Conex1;
                    cnx2 = segIndices2.Conex2;
                    segIndices2.LstSg1 = ip3;
                    iip2 = segIndices2.NxtSg1;
                    segIndices2.NxtSg1 = _nbPISeg;
                    if (iisv == 0)
                    {
                        nodIndices2.NdSg = _nbPISeg;
                    }
                    else if (icsv == 1)
                    {
                        segIndices.NxtSg1 = _nbPISeg;
                    }
                    else
                    {
                        segIndices.NxtSg2 = _nbPISeg;
                    }
                }
                else
                {
                    iisv = iiii;
                    icsv = 1;
                }
                iiii = segIndices2.NxtSg1;
            }
            else
            {
                if (segIndices2.LstSg1 == ip1)
                {
                    find = iiii;
                    cnx1 = segIndices2.Conex1;
                    cnx2 = segIndices2.Conex2;
                    segIndices2.LstSg2 = ip3;
                    iip2 = segIndices2.NxtSg2;
                    segIndices2.NxtSg2 = _nbPISeg;
                    if (iisv == 0)
                    {
                        nodIndices2.NdSg = _nbPISeg;
                    }
                    else if (icsv == 1)
                    {
                        segIndices.NxtSg1 = _nbPISeg;
                    }
                    else
                    {
                        segIndices.NxtSg2 = _nbPISeg;
                    }
                }
                else
                {
                    iisv = iiii;
                    icsv = 2;
                }
                iiii = segIndices2.NxtSg2;
            }
            segIndices = ref segIndices2;
        }
        if (find == 0)
        {
            _nbPISeg--;
    // OCCT_DEBUG trace left out
        }
        else
        {
            segIndices2 = ref piSeg1[_nbPISeg];
            segIndices2.NxtSg1 = 0;
            segIndices2.NxtSg2 = iip2;
            segIndices2.LstSg1 = ip3;
            segIndices2.LstSg2 = ip2;
            segIndices2.Conex1 = cnx1;
            segIndices2.Conex2 = cnx2;
            nodIndices3.NdSg = find;

            int iOld, iNew, iTr, skip, ip4;
            Span<int> itpk = stackalloc int[2];
            int n1, n2, n3;
            Span<int> nOld = stackalloc int[3], nNew = stackalloc int[3], New = stackalloc int[4];
            New[0] = cnx1;
            New[2] = _nbTData + 1;
            if (cnx2 == 0)
            {
                New[1] = 0;
                New[3] = 0;
            }
            else
            {
                New[1] = cnx2;
                New[3] = _nbTData + 2;
            }

            for (skip = 0; skip <= 1; skip++)
            {
                iOld = New[skip];
                iNew = New[skip + 2];
                if (iOld != 0)
                {
                    ref TriangleData triangle = ref tData1[iOld];
                    ref TriangleData triangle2 = ref tData1[iNew];
                    n1 = triangle.Node1;
                    n2 = triangle.Node2;
                    n3 = triangle.Node3;
                    nOld[0] = n1;
                    nOld[1] = n2;
                    nOld[2] = n3;
                    nNew[0] = n1;
                    nNew[1] = n2;
                    nNew[2] = n3;
                    bool found = false;
                    if (n1 == ip1 && n2 == ip2)
                    {
                        found = true;
                        nOld[1] = ip3;
                        nNew[0] = ip3;
                        itpk[skip] = n3;
                    }
                    else if (n1 == ip2 && n2 == ip1)
                    {
                        found = true;
                        nOld[0] = ip3;
                        nNew[1] = ip3;
                        itpk[skip] = n3;
                    }
                    else if (n2 == ip1 && n3 == ip2)
                    {
                        found = true;
                        nOld[2] = ip3;
                        nNew[1] = ip3;
                        itpk[skip] = n1;
                    }
                    else if (n2 == ip2 && n3 == ip1)
                    {
                        found = true;
                        nOld[1] = ip3;
                        nNew[2] = ip3;
                        itpk[skip] = n1;
                    }
                    else if (n3 == ip1 && n1 == ip2)
                    {
                        found = true;
                        nOld[0] = ip3;
                        nNew[2] = ip3;
                        itpk[skip] = n2;
                    }
                    else if (n3 == ip2 && n1 == ip1)
                    {
                        found = true;
                        nOld[2] = ip3;
                        nNew[0] = ip3;
                        itpk[skip] = n2;
                    }
                    if (found)
                    {
                        _nbTData++;
                        ip4 = itpk[skip];
                        ref PolyInternalNode.NodeIndices nodIndices4 = ref piNod1[ip4]!.Indices();
                        triangle.Node1 = nOld[0];
                        triangle.Node2 = nOld[1];
                        triangle.Node3 = nOld[2];
                        triangle2.Node1 = nNew[0];
                        triangle2.Node2 = nNew[1];
                        triangle2.Node3 = nNew[2];
                        triangle2.Flags = triangle.Flags;
                        _nbPISeg++;
                        segIndices2 = ref piSeg1[_nbPISeg];
                        segIndices2.LstSg1 = ip3;
                        segIndices2.LstSg2 = ip4;
                        segIndices2.NxtSg1 = nodIndices3.NdSg;
                        segIndices2.NxtSg2 = nodIndices4.NdSg;
                        segIndices2.Conex1 = iOld;
                        segIndices2.Conex2 = iNew;
                        nodIndices3.NdSg = _nbPISeg;
                        nodIndices4.NdSg = _nbPISeg;
                    }
    // OCCT_DEBUG trace left out
                }
            }

            for (iTr = 0; iTr <= 3; iTr++)
            {
                iNew = New[iTr];
                if (iTr < 2)
                {
                    skip = iTr;
                }
                else
                {
                    skip = iTr - 2;
                }
                iOld = New[skip];
                ip4 = itpk[skip];
                if (iNew != 0)
                {
                    ref TriangleData triangle2 = ref tData1[iNew];
                    n1 = triangle2.Node1;
                    n2 = triangle2.Node2;
                    n3 = triangle2.Node3;

                    if ((n1 != ip3 || n2 != ip4) && (n2 != ip3 || n1 != ip4))
                    {
                        bool found = false;
                        nodIndices1 = ref piNod1[n1]!.Indices();
                        iiii = nodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            segIndices2 = ref piSeg1[iiii];
                            if (segIndices2.LstSg1 == n1)
                            {
                                if (segIndices2.LstSg2 == n2)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (segIndices2.LstSg1 == n2)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg2;
                                }
                            }
                        }
                    }

                    if ((n2 != ip3 || n3 != ip4) && (n3 != ip3 || n2 != ip4))
                    {
                        bool found = false;
                        nodIndices1 = ref piNod1[n2]!.Indices();
                        iiii = nodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            segIndices2 = ref piSeg1[iiii];
                            if (segIndices2.LstSg1 == n2)
                            {
                                if (segIndices2.LstSg2 == n3)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (segIndices2.LstSg1 == n3)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg2;
                                }
                            }
                        }
                    }

                    if ((n3 != ip3 || n1 != ip4) && (n1 != ip3 || n3 != ip4))
                    {
                        bool found = false;
                        nodIndices1 = ref piNod1[n3]!.Indices();
                        iiii = nodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            segIndices2 = ref piSeg1[iiii];
                            if (segIndices2.LstSg1 == n3)
                            {
                                if (segIndices2.LstSg2 == n1)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (segIndices2.LstSg1 == n1)
                                {
                                    found = true;
                                    if (segIndices2.Conex1 == iOld)
                                    {
                                        segIndices2.Conex1 = iNew;
                                    }
                                    else if (segIndices2.Conex2 == iOld)
                                    {
                                        segIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = segIndices2.NxtSg2;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>Writes OCCT's trace of the nodes, segments and triangles (to stdout by default).</summary>
    public void Dump(TextWriter? stream = null)
    {
        TextWriter aStream = stream ?? Console.Out;
        int i;
        NCollection_Array1<TriangleData> tData = _tData.ChangeArray1();
        NCollection_Array1<PolyInternalSegment> piSeg = _piSeg.ChangeArray1();
        NCollection_Array1<PolyInternalNode?> piNod = _piNod.ChangeArray1();
        for (i = 1; i <= _nbPINod; i++)
        {
            ref PolyInternalNode.NodeIndices nodIndices1 = ref piNod[i]!.Indices();
            ref PolyInternalNode.NodeData nod1RValues = ref piNod[i]!.Data();
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Node {0,6} : {1,6}{2,20:G6}{3,20:G6}{4,20:G6}\n",
                                        i, nodIndices1.NdSg, nod1RValues.Point.X(), nod1RValues.Point.Y(),
                                        nod1RValues.Point.Z()));
        }
        for (i = 1; i <= _nbPISeg; i++)
        {
            ref PolyInternalSegment segIndices = ref piSeg[i];
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Segment {0,6} : {1,6}{2,6}{3,6}{4,6}{5,6}{6,6}\n",
                                        i, segIndices.LstSg1, segIndices.LstSg2, segIndices.NxtSg1,
                                        segIndices.NxtSg2, segIndices.Conex1, segIndices.Conex2));
        }
        for (i = 1; i <= _nbTData; i++)
        {
            ref TriangleData triangle = ref tData[i];
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Triangle {0,6} : {1,6}{2,6}{3,6}\n", i,
                                        triangle.Node1, triangle.Node2, triangle.Node3));
        }
    }

    /// <summary>
    /// Adds a triangle, growing the array when full: then <paramref name="tData1"/> (and <paramref name="tData2"/>
    /// if it is the same array) move to the new one.
    /// </summary>
    public void IncTData(ref NCollection_Array1<TriangleData> tData1,
                         ref NCollection_Array1<TriangleData> tData2)
    {
        if (_nbTData >= _mxTData)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = _mxTData;
            k = 2 * j;
            NCollection_HArray1<TriangleData> nwTData = new(0, k);
            NCollection_Array1<TriangleData> oTData = _tData.ChangeArray1();
            NCollection_Array1<TriangleData> nTData = nwTData.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nTData[i] = oTData.Value(i);
            }
            _mxTData = k;
            _tData = nwTData;
            if (ReferenceEquals(tData1, tData2))
            {
                tData1 = nTData;
                tData2 = tData1;
            }
            else
            {
                tData1 = nTData;
            }
        }
        _nbTData++;
    }

    /// <summary>Adds a segment, growing the array when full (as <see cref="IncTData"/>).</summary>
    public void IncPISeg(ref NCollection_Array1<PolyInternalSegment> piSeg1,
                         ref NCollection_Array1<PolyInternalSegment> piSeg2)
    {
        if (_nbPISeg >= _mxPISeg)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = _mxPISeg;
            k = 2 * j;
            NCollection_HArray1<PolyInternalSegment> nwPISeg = new(0, k);
            NCollection_Array1<PolyInternalSegment> oPISeg = _piSeg.ChangeArray1();
            NCollection_Array1<PolyInternalSegment> nPISeg = nwPISeg.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nPISeg[i] = oPISeg.Value(i);
            }
            _mxPISeg = k;
            _piSeg = nwPISeg;
            if (ReferenceEquals(piSeg1, piSeg2))
            {
                piSeg1 = nPISeg;
                piSeg2 = piSeg1;
            }
            else
            {
                piSeg1 = nPISeg;
            }
        }
        _nbPISeg++;
    }

    /// <summary>Adds a node, growing the array when full (as <see cref="IncTData"/>).</summary>
    public void IncPINod(ref NCollection_Array1<PolyInternalNode?> piNod1,
                         ref NCollection_Array1<PolyInternalNode?> piNod2)
    {
        if (_nbPINod >= _mxPINod)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = _mxPINod;
            k = 2 * j;
            NCollection_HArray1<PolyInternalNode?> nwPINod = new(0, k);
            NCollection_Array1<PolyInternalNode?> oPINod = _piNod.ChangeArray1();
            NCollection_Array1<PolyInternalNode?> nPINod = nwPINod.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nPINod[i] = oPINod[i];
            }
            _mxPINod = k;
            _piNod = nwPINod;
            if (ReferenceEquals(piNod1, piNod2))
            {
                piNod1 = nPINod;
                piNod2 = piNod1;
            }
            else
            {
                piNod1 = nPINod;
            }
        }
        _nbPINod++;
    }

    public void DecTData()
    {
        _nbTData--;
    }

    public void DecPISeg()
    {
        _nbPISeg--;
    }

    public void DecPINod()
    {
        _nbPINod--;
    }

    public int TDataCount => _nbTData;

    public int PISegCount => _nbPISeg;

    public int PINodCount => _nbPINod;

    public bool Planar() => _planar;

    public void Planar(bool b) => _planar = b;

    public bool IntOutL() => _intOutL;

    public void IntOutL(bool b) => _intOutL = b;

    public NCollection_Array1<TriangleData> TData() => _tData.ChangeArray1();

    public NCollection_Array1<PolyInternalSegment> PISeg() => _piSeg.ChangeArray1();

    public NCollection_Array1<PolyInternalNode?> PINod() => _piNod.ChangeArray1();
}
