// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangle.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A triangle of a <see cref="CoherentTriangulation"/>: three node indices, and on each side (indexed by the
/// opposite node, 0 to 2) the connected triangle, its third node and the link.
/// </summary>
/// <remarks>
/// A class, as triangles refer to each other and lists of them are kept per node (OCCT stores pointers; the port
/// compares references where OCCT compares addresses).
/// </remarks>
internal class CoherentTriangle
{
    internal int _nConnections;
    internal NCollection_Buffer3<int> _nodes;
    internal NCollection_Buffer3<int> _nodesOnConnected;
    internal NCollection_Buffer3<CoherentTriangle?> _mypConnected;
    internal NCollection_Buffer3<CoherentLink?> _mypLink;

    /// <summary>An empty (invalid) triangle.</summary>
    public CoherentTriangle()
    {
        _nConnections = 0;
        _nodes[0] = -1;
        _nodes[1] = -1;
        _nodes[2] = -1;
        _nodesOnConnected[0] = -1;
        _nodesOnConnected[1] = -1;
        _nodesOnConnected[2] = -1;
        _mypConnected[0] = null;
        _mypConnected[1] = null;
        _mypConnected[2] = null;
        _mypLink[0] = null;
        _mypLink[1] = null;
        _mypLink[2] = null;
    }

    /// <summary>A triangle of three nodes, without connections.</summary>
    public CoherentTriangle(int iNode0, int iNode1, int iNode2)
    {
        _nConnections = 0;
        _nodes[0] = iNode0;
        _nodes[1] = iNode1;
        _nodes[2] = iNode2;
        _nodesOnConnected[0] = -1;
        _nodesOnConnected[1] = -1;
        _nodesOnConnected[2] = -1;
        _mypConnected[0] = null;
        _mypConnected[1] = null;
        _mypConnected[2] = null;
        _mypLink[0] = null;
        _mypLink[1] = null;
        _mypLink[2] = null;
    }

    /// <summary>The node index at position 0, 1 or 2.</summary>
    public int Node(int ind) => _nodes[ind];

    /// <summary>True if the triangle is invalid (removed): a node is -1.</summary>
    public bool IsEmpty => _nodes[0] < 0 || _nodes[1] < 0 || _nodes[2] < 0;

    /// <summary>
    /// Connects triangle <paramref name="tr"/> on side <paramref name="iConn"/> (the index of the opposite node) of
    /// this one, in both triangles. False if the connection is rejected because of the topology.
    /// </summary>
    public bool SetConnection(int iConn, CoherentTriangle tr)
    {
        bool result = false;
        ReadOnlySpan<int> ii = [2, 0, 1, 2, 0];

        if (tr.Node(0) == _nodes[ii[iConn + 2]])
        {
            if (tr.Node(2) == _nodes[ii[iConn]])
            {
                RemoveConnection(iConn);
                _nodesOnConnected[iConn] = tr.Node(1);
                _mypConnected[iConn] = tr;
                _nConnections++;
                tr.RemoveConnection(1);
                tr._nodesOnConnected[1] = _nodes[iConn];
                tr._mypConnected[1] = this;
                tr._nConnections++;
                result = true;
            }
        }
        else if (tr.Node(1) == _nodes[ii[iConn + 2]])
        {
            if (tr.Node(0) == _nodes[ii[iConn]])
            {
                RemoveConnection(iConn);
                _nodesOnConnected[iConn] = tr.Node(2);
                _mypConnected[iConn] = tr;
                _nConnections++;
                tr.RemoveConnection(2);
                tr._nodesOnConnected[2] = _nodes[iConn];
                tr._mypConnected[2] = this;
                tr._nConnections++;
                result = true;
            }
        }
        else if (tr.Node(2) == _nodes[ii[iConn + 2]])
        {
            if (tr.Node(1) == _nodes[ii[iConn]])
            {
                RemoveConnection(iConn);
                _nodesOnConnected[iConn] = tr.Node(0);
                _mypConnected[iConn] = tr;
                _nConnections++;
                tr.RemoveConnection(0);
                tr._nodesOnConnected[0] = _nodes[iConn];
                tr._mypConnected[0] = this;
                tr._nConnections++;
                result = true;
            }
        }

        return result;
    }

    /// <summary>
    /// Connects triangle <paramref name="tr"/> to this one on the side they share, in both triangles. Slower than
    /// <see cref="SetConnection(int, CoherentTriangle)"/>. False if they share no side or the connection is
    /// rejected because of the topology.
    /// </summary>
    public bool SetConnection(CoherentTriangle tr)
    {
        bool result = false;
        if (_nodes[0] == tr.Node(0))
        {
            if (_nodes[1] == tr.Node(2) && _mypConnected[2] != tr)
            {
                RemoveConnection(2);
                _nodesOnConnected[2] = tr.Node(1);
                _mypConnected[2] = tr;
                _nConnections++;
                tr.RemoveConnection(1);
                tr._nodesOnConnected[1] = _nodes[2];
                tr._mypConnected[1] = this;
                tr._nConnections++;
                result = true;
            }
            else if (_nodes[2] == tr.Node(1) && _mypConnected[1] != tr)
            {
                RemoveConnection(1);
                _nodesOnConnected[1] = tr.Node(2);
                _mypConnected[1] = tr;
                _nConnections++;
                tr.RemoveConnection(2);
                tr._nodesOnConnected[2] = _nodes[1];
                tr._mypConnected[2] = this;
                tr._nConnections++;
                result = true;
            }
        }
        else if (_nodes[0] == tr.Node(1))
        {
            if (_nodes[1] == tr.Node(0) && _mypConnected[2] != tr)
            {
                RemoveConnection(2);
                _nodesOnConnected[2] = tr.Node(2);
                _mypConnected[2] = tr;
                _nConnections++;
                tr.RemoveConnection(2);
                tr._nodesOnConnected[2] = _nodes[2];
                tr._mypConnected[2] = this;
                tr._nConnections++;
                result = true;
            }
            else if (_nodes[2] == tr.Node(2) && _mypConnected[1] != tr)
            {
                RemoveConnection(1);
                _nodesOnConnected[1] = tr.Node(0);
                _mypConnected[1] = tr;
                _nConnections++;
                tr.RemoveConnection(0);
                tr._nodesOnConnected[0] = _nodes[1];
                tr._mypConnected[0] = this;
                tr._nConnections++;
                result = true;
            }
        }
        else if (_nodes[0] == tr.Node(2))
        {
            if (_nodes[1] == tr.Node(1) && _mypConnected[2] != tr)
            {
                RemoveConnection(2);
                _nodesOnConnected[2] = tr.Node(0);
                _mypConnected[2] = tr;
                _nConnections++;
                tr.RemoveConnection(0);
                tr._nodesOnConnected[0] = _nodes[2];
                tr._mypConnected[0] = this;
                tr._nConnections++;
                result = true;
            }
            else if (_nodes[2] == tr.Node(0) && _mypConnected[1] != tr)
            {
                RemoveConnection(1);
                _nodesOnConnected[1] = tr.Node(1);
                _mypConnected[1] = tr;
                _nConnections++;
                tr.RemoveConnection(1);
                tr._nodesOnConnected[1] = _nodes[1];
                tr._mypConnected[1] = this;
                tr._nConnections++;
                result = true;
            }
        }
        else if (_mypConnected[0] != tr)
        {
            if (_nodes[1] == tr.Node(0) && _nodes[2] == tr.Node(2))
            {
                RemoveConnection(0);
                _nodesOnConnected[0] = tr.Node(1);
                _mypConnected[0] = tr;
                _nConnections++;
                tr.RemoveConnection(1);
                tr._nodesOnConnected[1] = _nodes[0];
                tr._mypConnected[1] = this;
                tr._nConnections++;
                result = true;
            }
            else if (_nodes[1] == tr.Node(2) && _nodes[2] == tr.Node(1))
            {
                RemoveConnection(0);
                _nodesOnConnected[0] = tr.Node(0);
                _mypConnected[0] = tr;
                _nConnections++;
                tr.RemoveConnection(0);
                tr._nodesOnConnected[0] = _nodes[0];
                tr._mypConnected[0] = this;
                tr._nConnections++;
                result = true;
            }
            else if (_nodes[1] == tr.Node(1) && _nodes[2] == tr.Node(0))
            {
                RemoveConnection(0);
                _nodesOnConnected[0] = tr.Node(2);
                _mypConnected[0] = tr;
                _nConnections++;
                tr.RemoveConnection(2);
                tr._nodesOnConnected[2] = _nodes[0];
                tr._mypConnected[2] = this;
                tr._nConnections++;
                result = true;
            }
        }

        return result;
    }

    /// <summary>Removes the connection on side <paramref name="iConn"/>, in both triangles.</summary>
    public void RemoveConnection(int iConn)
    {
        CoherentTriangle? pConnectedTri = _mypConnected[iConn];
        if (pConnectedTri is not null)
        {
            int iConn1 = 0;
            if (pConnectedTri._mypConnected[0] != this)
            {
                if (pConnectedTri._mypConnected[1] == this)
                {
                    iConn1 = 1;
                }
                else if (pConnectedTri._mypConnected[2] == this)
                {
                    iConn1 = 2;
                }
                else
                {
                    throw new ProgramErrorException("Poly_CoherentTriangle::RemoveConnection: wrong connection between triangles");
                }
            }

            pConnectedTri._mypConnected[iConn1] = null;
            pConnectedTri._nodesOnConnected[iConn1] = -1;
            pConnectedTri._nConnections--;
            _mypConnected[iConn] = null;
            _nodesOnConnected[iConn] = -1;
            _nConnections--;
        }
    }

    /// <summary>Removes the connection with a triangle, in both; false if they weren't connected.</summary>
    public bool RemoveConnection(CoherentTriangle tri)
    {
        int iConn = FindConnection(tri);
        if (iConn >= 0)
        {
            RemoveConnection(iConn);
        }

        return iConn >= 0;
    }

    /// <summary>The number of connected triangles.</summary>
    public int NConnections() => _nConnections;

    /// <summary>The third node of the triangle connected on a side, or -1.</summary>
    public int GetConnectedNode(int iConn) => _nodesOnConnected[iConn];

    /// <summary>The triangle connected on a side, or null.</summary>
    public CoherentTriangle? GetConnectedTri(int iConn) => _mypConnected[iConn];

    /// <summary>The link on a side, or null (there are no links until the triangulation computes them).</summary>
    public CoherentLink? GetLink(int iLink) => _mypLink[iLink];

    /// <summary>The side on which a triangle is connected, or -1.</summary>
    public int FindConnection(CoherentTriangle tri)
    {
        int result;
        if (_mypConnected[0] == tri)
        {
            result = 0;
        }
        else if (_mypConnected[1] == tri)
        {
            result = 1;
        }
        else if (_mypConnected[2] == tri)
        {
            result = 2;
        }
        else
        {
            result = -1;
        }

        return result;
    }

    // C++ assignment (theTriangle = Poly_CoherentTriangle(...)), used by Poly_CoherentTriangulation
    internal void Assign(CoherentTriangle other)
    {
        _nConnections = other._nConnections;
        _nodes = other._nodes;
        _nodesOnConnected = other._nodesOnConnected;
        _mypConnected = other._mypConnected;
        _mypLink = other._mypLink;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_CoherentTriangle({_nodes[0]}, {_nodes[1]}, {_nodes[2]})");
}
