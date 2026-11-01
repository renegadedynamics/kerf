// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;

namespace Occt;

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
public class HLRAlgo_PolyInternalData : Standard_Transient
{
    private int myNbTData;
    private int myNbPISeg;
    private int myNbPINod;
    private int myMxTData;
    private int myMxPISeg;
    private int myMxPINod;
    private bool myIntOutL;
    private bool myPlanar;
    private NCollection_HArray1<HLRAlgo_TriangleData> myTData;
    private NCollection_HArray1<HLRAlgo_PolyInternalSegment> myPISeg;
    private NCollection_HArray1<HLRAlgo_PolyInternalNode?> myPINod;

    public HLRAlgo_PolyInternalData(int nbNod, int nbTri)
    {
        myNbTData = nbTri;
        myNbPISeg = 0;
        myNbPINod = nbNod;
        myMxTData = nbTri;
        myMxPINod = nbNod;
        myIntOutL = false;
        myPlanar = false;
        int i;
        myMxPISeg = 2 + (3 * nbTri + nbNod) / 2;
        myTData = new NCollection_HArray1<HLRAlgo_TriangleData>(0, myMxTData);
        myPISeg = new NCollection_HArray1<HLRAlgo_PolyInternalSegment>(0, myMxPISeg);
        myPINod = new NCollection_HArray1<HLRAlgo_PolyInternalNode?>(0, myMxPINod);

        NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod = myPINod.ChangeArray1();
        for (i = 1; i <= myMxPINod; i++)
        {
            PINod[i] = new HLRAlgo_PolyInternalNode();
        }
    }

    /// <summary>Builds the segments linking the triangles.</summary>
    public void UpdateLinks(NCollection_Array1<HLRAlgo_TriangleData> theTData,
                            NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg,
                            NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod)
    {
        int n1, n2;
        int find, iiii, icsv = 0;
        ref HLRAlgo_PolyInternalSegment aSegIndices = ref Unsafe.NullRef<HLRAlgo_PolyInternalSegment>();

        bool newSeg = false;
        for (int i = 1; i <= myNbTData; i++)
        {
            ref HLRAlgo_TriangleData TD = ref theTData[i];

            ref HLRAlgo_PolyInternalNode.NodeIndices A1 = ref thePINod[TD.Node1]!.Indices();
            ref HLRAlgo_PolyInternalNode.NodeIndices A2 = ref thePINod[TD.Node2]!.Indices();
            ref HLRAlgo_PolyInternalNode.NodeIndices A3 = ref thePINod[TD.Node3]!.Indices();

            {
                n1 = TD.Node1;
                n2 = TD.Node2;
                newSeg = false;
                if (A1.NdSg == 0 && A2.NdSg == 0)
                {
                    newSeg = true;
                    myNbPISeg++;
                    A1.NdSg = myNbPISeg;
                    A2.NdSg = myNbPISeg;
                }
                else
                {
                    iiii = A1.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            aSegIndices = ref thePISeg[iiii];
                            if (aSegIndices.LstSg1 == n1)
                            {
                                if (aSegIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            myNbPISeg++;
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            aSegIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        myNbPISeg++;
                        A1.NdSg = myNbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = A2.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                aSegIndices = ref thePISeg[iiii];
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = aSegIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = aSegIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            A2.NdSg = myNbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    aSegIndices = ref thePISeg[myNbPISeg];
                    aSegIndices.LstSg1 = n1;
                    aSegIndices.LstSg2 = n2;
                    aSegIndices.Conex1 = i;
                    aSegIndices.Conex2 = 0;
                    aSegIndices.NxtSg1 = 0;
                    aSegIndices.NxtSg2 = 0;
                }
            }

            {
                n1 = TD.Node2;
                n2 = TD.Node3;
                newSeg = false;
                if (A2.NdSg == 0 && A3.NdSg == 0)
                {
                    newSeg = true;
                    myNbPISeg++;
                    A2.NdSg = myNbPISeg;
                    A3.NdSg = myNbPISeg;
                }
                else
                {
                    iiii = A2.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            aSegIndices = ref thePISeg[iiii];
                            if (aSegIndices.LstSg1 == n1)
                            {
                                if (aSegIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            myNbPISeg++;
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            aSegIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        myNbPISeg++;
                        A2.NdSg = myNbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = A3.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                aSegIndices = ref thePISeg[iiii];
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = aSegIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = aSegIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            A3.NdSg = myNbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    aSegIndices = ref thePISeg[myNbPISeg];
                    aSegIndices.LstSg1 = n1;
                    aSegIndices.LstSg2 = n2;
                    aSegIndices.Conex1 = i;
                    aSegIndices.Conex2 = 0;
                    aSegIndices.NxtSg1 = 0;
                    aSegIndices.NxtSg2 = 0;
                }
            }

            {
                n1 = TD.Node3;
                n2 = TD.Node1;
                newSeg = false;
                if (A3.NdSg == 0 && A1.NdSg == 0)
                {
                    newSeg = true;
                    myNbPISeg++;
                    A3.NdSg = myNbPISeg;
                    A1.NdSg = myNbPISeg;
                }
                else
                {
                    iiii = A3.NdSg;
                    if (iiii != 0)
                    {
                        find = 0;

                        while (iiii != 0 && find == 0)
                        {
                            aSegIndices = ref thePISeg[iiii];
                            if (aSegIndices.LstSg1 == n1)
                            {
                                if (aSegIndices.LstSg2 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg1;
                                }
                                icsv = 1;
                            }
                            else
                            {
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    find = iiii;
                                }
                                else
                                {
                                    iiii = aSegIndices.NxtSg2;
                                }
                                icsv = 2;
                            }
                        }
                        if (find == 0)
                        {
                            newSeg = true;
                            myNbPISeg++;
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            aSegIndices.Conex2 = i;
                        }
                    }
                    else
                    {
                        newSeg = true;
                        myNbPISeg++;
                        A3.NdSg = myNbPISeg;
                    }
                    if (newSeg)
                    {
                        iiii = A1.NdSg;
                        if (iiii != 0)
                        {

                            while (iiii != 0)
                            {
                                aSegIndices = ref thePISeg[iiii];
                                if (aSegIndices.LstSg1 == n2)
                                {
                                    icsv = 1;
                                    iiii = aSegIndices.NxtSg1;
                                }
                                else
                                {
                                    icsv = 2;
                                    iiii = aSegIndices.NxtSg2;
                                }
                            }
                            if (icsv == 1)
                            {
                                aSegIndices.NxtSg1 = myNbPISeg;
                            }
                            else
                            {
                                aSegIndices.NxtSg2 = myNbPISeg;
                            }
                        }
                        else
                        {
                            A1.NdSg = myNbPISeg;
                        }
                    }
                }
                if (newSeg)
                {
                    aSegIndices = ref thePISeg[myNbPISeg];
                    aSegIndices.LstSg1 = n1;
                    aSegIndices.LstSg2 = n2;
                    aSegIndices.Conex1 = i;
                    aSegIndices.Conex2 = 0;
                    aSegIndices.NxtSg1 = 0;
                    aSegIndices.NxtSg2 = 0;
                }
            }
        }
    }

    /// <summary>
    /// Adds a node at (<paramref name="theX3"/>, <paramref name="theY3"/>, <paramref name="theZ3"/>), interpolated
    /// between two nodes' data at <paramref name="theCoef1"/>; returns its index.
    /// </summary>
    public int AddNode(ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues,
                       ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues,
                       ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1,
                       ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod2, double theCoef1, double theX3,
                       double theY3, double theZ3)
    {
        double coef2 = 1 - theCoef1;
        IncPINod(ref thePINod1, ref thePINod2);
        int ip3 = myNbPINod;
        HLRAlgo_PolyInternalNode pip3 = new();
        thePINod1[ip3] = pip3;
        ref HLRAlgo_PolyInternalNode.NodeData Nod3RValues = ref pip3.Data();
        ref HLRAlgo_PolyInternalNode.NodeIndices aNodeIndices = ref pip3.Indices();
        aNodeIndices.NdSg = 0;
        aNodeIndices.Flag = 0;
        Nod3RValues.Point = new gp_XYZ(theX3, theY3, theZ3);
        Nod3RValues.UV = coef2 * theNod1RValues.UV + theCoef1 * theNod2RValues.UV;
        Nod3RValues.Scal = theNod1RValues.Scal * coef2 + theNod2RValues.Scal * theCoef1;
        gp_XYZ aXYZ = coef2 * theNod1RValues.Normal + theCoef1 * theNod2RValues.Normal;
        double aNorm = aXYZ.Modulus();
        if (aNorm > 0)
        {
            Nod3RValues.Normal = (1 / aNorm) * aXYZ;
        }
        else
        {
            Nod3RValues.Normal = new gp_XYZ(1.0, 0.0, 0.0);
            // OCCT_DEBUG trace left out
        }
        return ip3;
    }

    /// <summary>Inserts the node <paramref name="ip3"/> on the segment (<paramref name="ip1"/>, <paramref name="ip2"/>), splitting its triangles.</summary>
    public void UpdateLinks(int ip1, int ip2, int ip3, ref NCollection_Array1<HLRAlgo_TriangleData> TData1,
                            ref NCollection_Array1<HLRAlgo_TriangleData> TData2,
                            ref NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg1,
                            ref NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg2,
                            ref NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod1,
                            ref NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod2)
    {
        int find, iiii, iisv, icsv, iip2 = 0, cnx1 = 0, cnx2 = 0;
        ref HLRAlgo_PolyInternalSegment aSegIndices = ref Unsafe.NullRef<HLRAlgo_PolyInternalSegment>();
        ref HLRAlgo_PolyInternalSegment aSegIndices2 = ref Unsafe.NullRef<HLRAlgo_PolyInternalSegment>();
        find = 0;
        iisv = 0;
        icsv = 0;
        IncPISeg(ref PISeg1, ref PISeg2);
        IncPISeg(ref PISeg1, ref PISeg2);
        IncPISeg(ref PISeg1, ref PISeg2);
        myNbPISeg--;
        myNbPISeg--;
        IncTData(ref TData1, ref TData2);
        IncTData(ref TData1, ref TData2);
        myNbTData--;
        myNbTData--;
        ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices1 = ref PINod1[ip1]!.Indices();
        ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices2 = ref PINod1[ip2]!.Indices();
        ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices3 = ref PINod1[ip3]!.Indices();
        iiii = aNodIndices2.NdSg;

        while (iiii != 0 && find == 0)
        {
            aSegIndices2 = ref PISeg1[iiii];
            if (aSegIndices2.LstSg1 == ip2)
            {
                if (aSegIndices2.LstSg2 == ip1)
                {
                    find = iiii;
                    cnx1 = aSegIndices2.Conex1;
                    cnx2 = aSegIndices2.Conex2;
                    aSegIndices2.LstSg1 = ip3;
                    iip2 = aSegIndices2.NxtSg1;
                    aSegIndices2.NxtSg1 = myNbPISeg;
                    if (iisv == 0)
                    {
                        aNodIndices2.NdSg = myNbPISeg;
                    }
                    else if (icsv == 1)
                    {
                        aSegIndices.NxtSg1 = myNbPISeg;
                    }
                    else
                    {
                        aSegIndices.NxtSg2 = myNbPISeg;
                    }
                }
                else
                {
                    iisv = iiii;
                    icsv = 1;
                }
                iiii = aSegIndices2.NxtSg1;
            }
            else
            {
                if (aSegIndices2.LstSg1 == ip1)
                {
                    find = iiii;
                    cnx1 = aSegIndices2.Conex1;
                    cnx2 = aSegIndices2.Conex2;
                    aSegIndices2.LstSg2 = ip3;
                    iip2 = aSegIndices2.NxtSg2;
                    aSegIndices2.NxtSg2 = myNbPISeg;
                    if (iisv == 0)
                    {
                        aNodIndices2.NdSg = myNbPISeg;
                    }
                    else if (icsv == 1)
                    {
                        aSegIndices.NxtSg1 = myNbPISeg;
                    }
                    else
                    {
                        aSegIndices.NxtSg2 = myNbPISeg;
                    }
                }
                else
                {
                    iisv = iiii;
                    icsv = 2;
                }
                iiii = aSegIndices2.NxtSg2;
            }
            aSegIndices = ref aSegIndices2;
        }
        if (find == 0)
        {
            myNbPISeg--;
    // OCCT_DEBUG trace left out
        }
        else
        {
            aSegIndices2 = ref PISeg1[myNbPISeg];
            aSegIndices2.NxtSg1 = 0;
            aSegIndices2.NxtSg2 = iip2;
            aSegIndices2.LstSg1 = ip3;
            aSegIndices2.LstSg2 = ip2;
            aSegIndices2.Conex1 = cnx1;
            aSegIndices2.Conex2 = cnx2;
            aNodIndices3.NdSg = find;

            int iOld, iNew, iTr, skip, ip4;
            Span<int> itpk = stackalloc int[2];
            int n1, n2, n3;
            Span<int> nOld = stackalloc int[3], nNew = stackalloc int[3], New = stackalloc int[4];
            New[0] = cnx1;
            New[2] = myNbTData + 1;
            if (cnx2 == 0)
            {
                New[1] = 0;
                New[3] = 0;
            }
            else
            {
                New[1] = cnx2;
                New[3] = myNbTData + 2;
            }

            for (skip = 0; skip <= 1; skip++)
            {
                iOld = New[skip];
                iNew = New[skip + 2];
                if (iOld != 0)
                {
                    ref HLRAlgo_TriangleData aTriangle = ref TData1[iOld];
                    ref HLRAlgo_TriangleData aTriangle2 = ref TData1[iNew];
                    n1 = aTriangle.Node1;
                    n2 = aTriangle.Node2;
                    n3 = aTriangle.Node3;
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
                        myNbTData++;
                        ip4 = itpk[skip];
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices4 = ref PINod1[ip4]!.Indices();
                        aTriangle.Node1 = nOld[0];
                        aTriangle.Node2 = nOld[1];
                        aTriangle.Node3 = nOld[2];
                        aTriangle2.Node1 = nNew[0];
                        aTriangle2.Node2 = nNew[1];
                        aTriangle2.Node3 = nNew[2];
                        aTriangle2.Flags = aTriangle.Flags;
                        myNbPISeg++;
                        aSegIndices2 = ref PISeg1[myNbPISeg];
                        aSegIndices2.LstSg1 = ip3;
                        aSegIndices2.LstSg2 = ip4;
                        aSegIndices2.NxtSg1 = aNodIndices3.NdSg;
                        aSegIndices2.NxtSg2 = aNodIndices4.NdSg;
                        aSegIndices2.Conex1 = iOld;
                        aSegIndices2.Conex2 = iNew;
                        aNodIndices3.NdSg = myNbPISeg;
                        aNodIndices4.NdSg = myNbPISeg;
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
                    ref HLRAlgo_TriangleData aTriangle2 = ref TData1[iNew];
                    n1 = aTriangle2.Node1;
                    n2 = aTriangle2.Node2;
                    n3 = aTriangle2.Node3;

                    if ((n1 != ip3 || n2 != ip4) && (n2 != ip3 || n1 != ip4))
                    {
                        bool found = false;
                        aNodIndices1 = ref PINod1[n1]!.Indices();
                        iiii = aNodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            aSegIndices2 = ref PISeg1[iiii];
                            if (aSegIndices2.LstSg1 == n1)
                            {
                                if (aSegIndices2.LstSg2 == n2)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (aSegIndices2.LstSg1 == n2)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg2;
                                }
                            }
                        }
                    }

                    if ((n2 != ip3 || n3 != ip4) && (n3 != ip3 || n2 != ip4))
                    {
                        bool found = false;
                        aNodIndices1 = ref PINod1[n2]!.Indices();
                        iiii = aNodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            aSegIndices2 = ref PISeg1[iiii];
                            if (aSegIndices2.LstSg1 == n2)
                            {
                                if (aSegIndices2.LstSg2 == n3)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (aSegIndices2.LstSg1 == n3)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg2;
                                }
                            }
                        }
                    }

                    if ((n3 != ip3 || n1 != ip4) && (n1 != ip3 || n3 != ip4))
                    {
                        bool found = false;
                        aNodIndices1 = ref PINod1[n3]!.Indices();
                        iiii = aNodIndices1.NdSg;

                        while (iiii != 0 && !found)
                        {
                            aSegIndices2 = ref PISeg1[iiii];
                            if (aSegIndices2.LstSg1 == n3)
                            {
                                if (aSegIndices2.LstSg2 == n1)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg1;
                                }
                            }
                            else
                            {
                                if (aSegIndices2.LstSg1 == n1)
                                {
                                    found = true;
                                    if (aSegIndices2.Conex1 == iOld)
                                    {
                                        aSegIndices2.Conex1 = iNew;
                                    }
                                    else if (aSegIndices2.Conex2 == iOld)
                                    {
                                        aSegIndices2.Conex2 = iNew;
                                    }
                                }
                                else
                                {
                                    iiii = aSegIndices2.NxtSg2;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>Writes OCCT's trace of the nodes, segments and triangles (to stdout by default).</summary>
    public void Dump(TextWriter? theStream = null)
    {
        TextWriter aStream = theStream ?? Console.Out;
        int i;
        NCollection_Array1<HLRAlgo_TriangleData> TData = myTData.ChangeArray1();
        NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg = myPISeg.ChangeArray1();
        NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod = myPINod.ChangeArray1();
        for (i = 1; i <= myNbPINod; i++)
        {
            ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices1 = ref PINod[i]!.Indices();
            ref HLRAlgo_PolyInternalNode.NodeData Nod1RValues = ref PINod[i]!.Data();
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Node {0,6} : {1,6}{2,20:G6}{3,20:G6}{4,20:G6}\n",
                                        i, aNodIndices1.NdSg, Nod1RValues.Point.X(), Nod1RValues.Point.Y(),
                                        Nod1RValues.Point.Z()));
        }
        for (i = 1; i <= myNbPISeg; i++)
        {
            ref HLRAlgo_PolyInternalSegment aSegIndices = ref PISeg[i];
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Segment {0,6} : {1,6}{2,6}{3,6}{4,6}{5,6}{6,6}\n",
                                        i, aSegIndices.LstSg1, aSegIndices.LstSg2, aSegIndices.NxtSg1,
                                        aSegIndices.NxtSg2, aSegIndices.Conex1, aSegIndices.Conex2));
        }
        for (i = 1; i <= myNbTData; i++)
        {
            ref HLRAlgo_TriangleData aTriangle = ref TData[i];
            aStream.Write(string.Format(CultureInfo.InvariantCulture, "Triangle {0,6} : {1,6}{2,6}{3,6}\n", i,
                                        aTriangle.Node1, aTriangle.Node2, aTriangle.Node3));
        }
    }

    /// <summary>
    /// Adds a triangle, growing the array when full: then <paramref name="TData1"/> (and <paramref name="TData2"/>
    /// if it is the same array) move to the new one.
    /// </summary>
    public void IncTData(ref NCollection_Array1<HLRAlgo_TriangleData> TData1,
                         ref NCollection_Array1<HLRAlgo_TriangleData> TData2)
    {
        if (myNbTData >= myMxTData)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = myMxTData;
            k = 2 * j;
            NCollection_HArray1<HLRAlgo_TriangleData> NwTData = new(0, k);
            NCollection_Array1<HLRAlgo_TriangleData> oTData = myTData.ChangeArray1();
            NCollection_Array1<HLRAlgo_TriangleData> nTData = NwTData.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nTData[i] = oTData.Value(i);
            }
            myMxTData = k;
            myTData = NwTData;
            if (ReferenceEquals(TData1, TData2))
            {
                TData1 = nTData;
                TData2 = TData1;
            }
            else
            {
                TData1 = nTData;
            }
        }
        myNbTData++;
    }

    /// <summary>Adds a segment, growing the array when full (as <see cref="IncTData"/>).</summary>
    public void IncPISeg(ref NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg1,
                         ref NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg2)
    {
        if (myNbPISeg >= myMxPISeg)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = myMxPISeg;
            k = 2 * j;
            NCollection_HArray1<HLRAlgo_PolyInternalSegment> NwPISeg = new(0, k);
            NCollection_Array1<HLRAlgo_PolyInternalSegment> oPISeg = myPISeg.ChangeArray1();
            NCollection_Array1<HLRAlgo_PolyInternalSegment> nPISeg = NwPISeg.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nPISeg[i] = oPISeg.Value(i);
            }
            myMxPISeg = k;
            myPISeg = NwPISeg;
            if (ReferenceEquals(PISeg1, PISeg2))
            {
                PISeg1 = nPISeg;
                PISeg2 = PISeg1;
            }
            else
            {
                PISeg1 = nPISeg;
            }
        }
        myNbPISeg++;
    }

    /// <summary>Adds a node, growing the array when full (as <see cref="IncTData"/>).</summary>
    public void IncPINod(ref NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod1,
                         ref NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod2)
    {
        if (myNbPINod >= myMxPINod)
        {
            // OCCT_DEBUG trace left out
            int i, j, k;
            j = myMxPINod;
            k = 2 * j;
            NCollection_HArray1<HLRAlgo_PolyInternalNode?> NwPINod = new(0, k);
            NCollection_Array1<HLRAlgo_PolyInternalNode?> oPINod = myPINod.ChangeArray1();
            NCollection_Array1<HLRAlgo_PolyInternalNode?> nPINod = NwPINod.ChangeArray1();
            for (i = 1; i <= j; i++)
            {
                nPINod[i] = oPINod[i];
            }
            myMxPINod = k;
            myPINod = NwPINod;
            if (ReferenceEquals(PINod1, PINod2))
            {
                PINod1 = nPINod;
                PINod2 = PINod1;
            }
            else
            {
                PINod1 = nPINod;
            }
        }
        myNbPINod++;
    }

    public void DecTData()
    {
        myNbTData--;
    }

    public void DecPISeg()
    {
        myNbPISeg--;
    }

    public void DecPINod()
    {
        myNbPINod--;
    }

    public int NbTData() => myNbTData;

    public int NbPISeg() => myNbPISeg;

    public int NbPINod() => myNbPINod;

    public bool Planar() => myPlanar;

    public void Planar(bool B) => myPlanar = B;

    public bool IntOutL() => myIntOutL;

    public void IntOutL(bool B) => myIntOutL = B;

    public NCollection_Array1<HLRAlgo_TriangleData> TData() => myTData.ChangeArray1();

    public NCollection_Array1<HLRAlgo_PolyInternalSegment> PISeg() => myPISeg.ChangeArray1();

    public NCollection_Array1<HLRAlgo_PolyInternalNode?> PINod() => myPINod.ChangeArray1();
}
