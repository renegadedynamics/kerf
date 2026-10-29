// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangle.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A triangle of a <see cref="Poly_CoherentTriangulation"/>: three node indices, and on each side (indexed by the
/// opposite node, 0 to 2) the connected triangle, its third node and the link.
/// </summary>
/// <remarks>
/// A class, as triangles refer to each other and lists of them are kept per node (OCCT stores pointers; the port
/// compares references where OCCT compares addresses).
/// </remarks>
public class Poly_CoherentTriangle
{
    internal int myNConnections;
    internal NCollection_Buffer3<int> myNodes;
    internal NCollection_Buffer3<int> myNodesOnConnected;
    internal NCollection_Buffer3<Poly_CoherentTriangle?> mypConnected;
    internal NCollection_Buffer3<Poly_CoherentLink?> mypLink;

    /// <summary>An empty (invalid) triangle.</summary>
    public Poly_CoherentTriangle()
    {
        myNConnections = 0;
        myNodes[0] = -1;
        myNodes[1] = -1;
        myNodes[2] = -1;
        myNodesOnConnected[0] = -1;
        myNodesOnConnected[1] = -1;
        myNodesOnConnected[2] = -1;
        mypConnected[0] = null;
        mypConnected[1] = null;
        mypConnected[2] = null;
        mypLink[0] = null;
        mypLink[1] = null;
        mypLink[2] = null;
    }

    /// <summary>A triangle of three nodes, without connections.</summary>
    public Poly_CoherentTriangle(int iNode0, int iNode1, int iNode2)
    {
        myNConnections = 0;
        myNodes[0] = iNode0;
        myNodes[1] = iNode1;
        myNodes[2] = iNode2;
        myNodesOnConnected[0] = -1;
        myNodesOnConnected[1] = -1;
        myNodesOnConnected[2] = -1;
        mypConnected[0] = null;
        mypConnected[1] = null;
        mypConnected[2] = null;
        mypLink[0] = null;
        mypLink[1] = null;
        mypLink[2] = null;
    }

    /// <summary>The node index at position 0, 1 or 2.</summary>
    public int Node(int ind) => myNodes[ind];

    /// <summary>True if the triangle is invalid (removed): a node is -1.</summary>
    public bool IsEmpty() => myNodes[0] < 0 || myNodes[1] < 0 || myNodes[2] < 0;

    /// <summary>
    /// Connects triangle <paramref name="theTr"/> on side <paramref name="iConn"/> (the index of the opposite node) of
    /// this one, in both triangles. False if the connection is rejected because of the topology.
    /// </summary>
    public bool SetConnection(int iConn, Poly_CoherentTriangle theTr)
    {
        bool aResult = false;
        ReadOnlySpan<int> II = [2, 0, 1, 2, 0];

        if (theTr.Node(0) == myNodes[II[iConn + 2]])
        {
            if (theTr.Node(2) == myNodes[II[iConn]])
            {
                RemoveConnection(iConn);
                myNodesOnConnected[iConn] = theTr.Node(1);
                mypConnected[iConn] = theTr;
                myNConnections++;
                theTr.RemoveConnection(1);
                theTr.myNodesOnConnected[1] = myNodes[iConn];
                theTr.mypConnected[1] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }
        else if (theTr.Node(1) == myNodes[II[iConn + 2]])
        {
            if (theTr.Node(0) == myNodes[II[iConn]])
            {
                RemoveConnection(iConn);
                myNodesOnConnected[iConn] = theTr.Node(2);
                mypConnected[iConn] = theTr;
                myNConnections++;
                theTr.RemoveConnection(2);
                theTr.myNodesOnConnected[2] = myNodes[iConn];
                theTr.mypConnected[2] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }
        else if (theTr.Node(2) == myNodes[II[iConn + 2]])
        {
            if (theTr.Node(1) == myNodes[II[iConn]])
            {
                RemoveConnection(iConn);
                myNodesOnConnected[iConn] = theTr.Node(0);
                mypConnected[iConn] = theTr;
                myNConnections++;
                theTr.RemoveConnection(0);
                theTr.myNodesOnConnected[0] = myNodes[iConn];
                theTr.mypConnected[0] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }

        return aResult;
    }

    /// <summary>
    /// Connects triangle <paramref name="theTr"/> to this one on the side they share, in both triangles. Slower than
    /// <see cref="SetConnection(int, Poly_CoherentTriangle)"/>. False if they share no side or the connection is
    /// rejected because of the topology.
    /// </summary>
    public bool SetConnection(Poly_CoherentTriangle theTr)
    {
        bool aResult = false;
        if (myNodes[0] == theTr.Node(0))
        {
            if (myNodes[1] == theTr.Node(2) && mypConnected[2] != theTr)
            {
                RemoveConnection(2);
                myNodesOnConnected[2] = theTr.Node(1);
                mypConnected[2] = theTr;
                myNConnections++;
                theTr.RemoveConnection(1);
                theTr.myNodesOnConnected[1] = myNodes[2];
                theTr.mypConnected[1] = this;
                theTr.myNConnections++;
                aResult = true;
            }
            else if (myNodes[2] == theTr.Node(1) && mypConnected[1] != theTr)
            {
                RemoveConnection(1);
                myNodesOnConnected[1] = theTr.Node(2);
                mypConnected[1] = theTr;
                myNConnections++;
                theTr.RemoveConnection(2);
                theTr.myNodesOnConnected[2] = myNodes[1];
                theTr.mypConnected[2] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }
        else if (myNodes[0] == theTr.Node(1))
        {
            if (myNodes[1] == theTr.Node(0) && mypConnected[2] != theTr)
            {
                RemoveConnection(2);
                myNodesOnConnected[2] = theTr.Node(2);
                mypConnected[2] = theTr;
                myNConnections++;
                theTr.RemoveConnection(2);
                theTr.myNodesOnConnected[2] = myNodes[2];
                theTr.mypConnected[2] = this;
                theTr.myNConnections++;
                aResult = true;
            }
            else if (myNodes[2] == theTr.Node(2) && mypConnected[1] != theTr)
            {
                RemoveConnection(1);
                myNodesOnConnected[1] = theTr.Node(0);
                mypConnected[1] = theTr;
                myNConnections++;
                theTr.RemoveConnection(0);
                theTr.myNodesOnConnected[0] = myNodes[1];
                theTr.mypConnected[0] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }
        else if (myNodes[0] == theTr.Node(2))
        {
            if (myNodes[1] == theTr.Node(1) && mypConnected[2] != theTr)
            {
                RemoveConnection(2);
                myNodesOnConnected[2] = theTr.Node(0);
                mypConnected[2] = theTr;
                myNConnections++;
                theTr.RemoveConnection(0);
                theTr.myNodesOnConnected[0] = myNodes[2];
                theTr.mypConnected[0] = this;
                theTr.myNConnections++;
                aResult = true;
            }
            else if (myNodes[2] == theTr.Node(0) && mypConnected[1] != theTr)
            {
                RemoveConnection(1);
                myNodesOnConnected[1] = theTr.Node(1);
                mypConnected[1] = theTr;
                myNConnections++;
                theTr.RemoveConnection(1);
                theTr.myNodesOnConnected[1] = myNodes[1];
                theTr.mypConnected[1] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }
        else if (mypConnected[0] != theTr)
        {
            if (myNodes[1] == theTr.Node(0) && myNodes[2] == theTr.Node(2))
            {
                RemoveConnection(0);
                myNodesOnConnected[0] = theTr.Node(1);
                mypConnected[0] = theTr;
                myNConnections++;
                theTr.RemoveConnection(1);
                theTr.myNodesOnConnected[1] = myNodes[0];
                theTr.mypConnected[1] = this;
                theTr.myNConnections++;
                aResult = true;
            }
            else if (myNodes[1] == theTr.Node(2) && myNodes[2] == theTr.Node(1))
            {
                RemoveConnection(0);
                myNodesOnConnected[0] = theTr.Node(0);
                mypConnected[0] = theTr;
                myNConnections++;
                theTr.RemoveConnection(0);
                theTr.myNodesOnConnected[0] = myNodes[0];
                theTr.mypConnected[0] = this;
                theTr.myNConnections++;
                aResult = true;
            }
            else if (myNodes[1] == theTr.Node(1) && myNodes[2] == theTr.Node(0))
            {
                RemoveConnection(0);
                myNodesOnConnected[0] = theTr.Node(2);
                mypConnected[0] = theTr;
                myNConnections++;
                theTr.RemoveConnection(2);
                theTr.myNodesOnConnected[2] = myNodes[0];
                theTr.mypConnected[2] = this;
                theTr.myNConnections++;
                aResult = true;
            }
        }

        return aResult;
    }

    /// <summary>Removes the connection on side <paramref name="iConn"/>, in both triangles.</summary>
    public void RemoveConnection(int iConn)
    {
        Poly_CoherentTriangle? pConnectedTri = mypConnected[iConn];
        if (pConnectedTri is not null)
        {
            int iConn1 = 0;
            if (pConnectedTri.mypConnected[0] != this)
            {
                if (pConnectedTri.mypConnected[1] == this)
                {
                    iConn1 = 1;
                }
                else if (pConnectedTri.mypConnected[2] == this)
                {
                    iConn1 = 2;
                }
                else
                {
                    throw new Standard_ProgramError("Poly_CoherentTriangle::RemoveConnection: wrong connection between triangles");
                }
            }

            pConnectedTri.mypConnected[iConn1] = null;
            pConnectedTri.myNodesOnConnected[iConn1] = -1;
            pConnectedTri.myNConnections--;
            mypConnected[iConn] = null;
            myNodesOnConnected[iConn] = -1;
            myNConnections--;
        }
    }

    /// <summary>Removes the connection with a triangle, in both; false if they weren't connected.</summary>
    public bool RemoveConnection(Poly_CoherentTriangle theTri)
    {
        int iConn = FindConnection(theTri);
        if (iConn >= 0)
        {
            RemoveConnection(iConn);
        }

        return iConn >= 0;
    }

    /// <summary>The number of connected triangles.</summary>
    public int NConnections() => myNConnections;

    /// <summary>The third node of the triangle connected on a side, or -1.</summary>
    public int GetConnectedNode(int iConn) => myNodesOnConnected[iConn];

    /// <summary>The triangle connected on a side, or null.</summary>
    public Poly_CoherentTriangle? GetConnectedTri(int iConn) => mypConnected[iConn];

    /// <summary>The link on a side, or null (there are no links until the triangulation computes them).</summary>
    public Poly_CoherentLink? GetLink(int iLink) => mypLink[iLink];

    /// <summary>The side on which a triangle is connected, or -1.</summary>
    public int FindConnection(Poly_CoherentTriangle theTri)
    {
        int aResult;
        if (mypConnected[0] == theTri)
        {
            aResult = 0;
        }
        else if (mypConnected[1] == theTri)
        {
            aResult = 1;
        }
        else if (mypConnected[2] == theTri)
        {
            aResult = 2;
        }
        else
        {
            aResult = -1;
        }

        return aResult;
    }

    // C++ assignment (theTriangle = Poly_CoherentTriangle(...)), used by Poly_CoherentTriangulation
    internal void Assign(Poly_CoherentTriangle theOther)
    {
        myNConnections = theOther.myNConnections;
        myNodes = theOther.myNodes;
        myNodesOnConnected = theOther.myNodesOnConnected;
        mypConnected = theOther.mypConnected;
        mypLink = theOther.mypLink;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_CoherentTriangle({myNodes[0]}, {myNodes[1]}, {myNodes[2]})");
}
