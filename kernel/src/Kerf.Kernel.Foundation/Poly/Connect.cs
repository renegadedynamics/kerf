// Port of: src/FoundationClasses/TKMath/Poly/Poly_Connect.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Connect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The adjacency of a triangulation: for each node, a triangle containing it; for each triangle, the three triangles
/// sharing its edges and their third nodes.
/// </summary>
/// <remarks>
/// <para>
/// A triangle T with nodes n1, n2, n3 has adjacent triangles t1, t2 and t3, whose third nodes are an1, an2 and an3.
/// OCCT documents t1 as sharing n2 and n3, t2 as sharing n3 and n1, and t3 as sharing n1 and n2; the code (here as in
/// OCCT) gives t1 across n1-n2, t2 across n2-n3 and t3 across n3-n1. The triangles around a node can be iterated with
/// <see cref="Initialize"/>, <see cref="More"/>, <see cref="Next"/> and <see cref="Value"/>.
/// </para>
/// <para>An algorithm object, a class (OCCT's is used by value).</para>
/// </remarks>
internal class Connect
{
    // this structure records one of the edges starting from a node
    private sealed class Polyedge
    {
        public Polyedge? next;           // the next edge in the list
        public readonly int[] nt = new int[2]; // the two adjacent triangles
        public readonly int[] nn = new int[2]; // the two adjacent nodes
        public int nd;                   // the second node of the edge
    }

    private Triangulation? _triangulation;
    private readonly NCollection_Array1<int> _triangles;
    private readonly NCollection_Array1<int> _adjacents;
    private int _mytr;
    private int _myfirst;
    private int _mynode;
    private int _myothernode;
    private bool _mysense;
    private bool _mymore;
    private readonly NCollection_PackedMapOfInteger _passedTr = new();

    /// <summary>An uninitialized algorithm.</summary>
    public Connect()
    {
        _triangles = new NCollection_Array1<int>();
        _adjacents = new NCollection_Array1<int>();
        _mytr = 0;
        _myfirst = 0;
        _mynode = 0;
        _myothernode = 0;
        _mysense = false;
        _mymore = false;
    }

    /// <summary>The adjacency of a triangulation.</summary>
    public Connect(Triangulation triangulation)
    {
        _triangulation = triangulation;
        _triangles = new NCollection_Array1<int>(1, triangulation.NodeCount);
        _adjacents = new NCollection_Array1<int>(1, 6 * triangulation.TriangleCount);
        _mytr = 0;
        _myfirst = 0;
        _mynode = 0;
        _myothernode = 0;
        _mysense = false;
        _mymore = false;
        Load(triangulation);
    }

    /// <summary>Computes the adjacency of a triangulation.</summary>
    public void Load(Triangulation triangulation)
    {
        _triangulation = triangulation;
        _mytr = 0;
        _myfirst = 0;
        _mynode = 0;
        _myothernode = 0;
        _mysense = false;
        _mymore = false;

        int nbNodes = _triangulation.NodeCount;
        int nbTris = _triangulation.TriangleCount;
        {
            int nbAdjs = 6 * nbTris;
            if (_triangles.Length() != nbNodes)
            {
                _triangles.Resize(1, nbNodes, false);
            }

            if (_adjacents.Length() != nbAdjs)
            {
                _adjacents.Resize(1, nbAdjs, false);
            }
        }

        _triangles.Init(0);
        _adjacents.Init(0);

        // We first build an array of the list of edges connected to the nodes
        // create an array to store the edges starting from the vertices
        var edges = new NCollection_Array1<Polyedge?>(1, nbNodes);
        edges.Init(null);

        // loop on the triangles
        Span<int> triNodes = [0, 0, 0];
        Span<int> edgeNodes = [0, 0];
        for (int triIter = 1; triIter <= nbTris; ++triIter)
        {
            // get the nodes
            _triangulation.Triangle(triIter).Get(out triNodes[0], out triNodes[1], out triNodes[2]);

            // Update the myTriangles array
            _triangles.SetValue(triNodes[0], triIter);
            _triangles.SetValue(triNodes[1], triIter);
            _triangles.SetValue(triNodes[2], triIter);

            // update the edge lists
            for (int nodeInTri = 0; nodeInTri < 3; ++nodeInTri)
            {
                int nodeNext = (nodeInTri + 1) % 3; // the following node of the edge
                if (triNodes[nodeInTri] < triNodes[nodeNext])
                {
                    edgeNodes[0] = triNodes[nodeInTri];
                    edgeNodes[1] = triNodes[nodeNext];
                }
                else
                {
                    edgeNodes[0] = triNodes[nodeNext];
                    edgeNodes[1] = triNodes[nodeInTri];
                }

                // edge from node 0 to node 1 with node 0 < node 1
                // insert in the list of node 0
                Polyedge? ced = edges[edgeNodes[0]];
                for (; ced is not null; ced = ced.next)
                {
                    // the edge already exists
                    if (ced.nd == edgeNodes[1])
                    {
                        // just mark the adjacency if found
                        ced.nt[1] = triIter;
                        ced.nn[1] = triNodes[3 - nodeInTri - nodeNext]; // the third node
                        break;
                    }
                }

                if (ced is null)
                {
                    // create the edge if not found
                    ced = new Polyedge();
                    ced.next = edges[edgeNodes[0]];
                    edges[edgeNodes[0]] = ced;
                    ced.nd = edgeNodes[1];
                    ced.nt[0] = triIter;
                    ced.nn[0] = triNodes[3 - nodeInTri - nodeNext]; // the third node
                    ced.nt[1] = 0;
                    ced.nn[1] = 0;
                }
            }
        }

        // now complete the myAdjacents array
        int adjIndex = 1;
        for (int triIter = 1; triIter <= nbTris; ++triIter)
        {
            // get the nodes
            _triangulation.Triangle(triIter).Get(out triNodes[0], out triNodes[1], out triNodes[2]);

            // for each edge in triangle
            for (int nodeInTri = 0; nodeInTri < 3; ++nodeInTri)
            {
                int nodeNext = (nodeInTri + 1) % 3; // the following node of the edge
                if (triNodes[nodeInTri] < triNodes[nodeNext])
                {
                    edgeNodes[0] = triNodes[nodeInTri];
                    edgeNodes[1] = triNodes[nodeNext];
                }
                else
                {
                    edgeNodes[0] = triNodes[nodeNext];
                    edgeNodes[1] = triNodes[nodeInTri];
                }

                // edge from node 0 to node 1 with node 0 < node 1
                // find in the list of node 0
                Polyedge ced = edges[edgeNodes[0]]!;
                while (ced.nd != edgeNodes[1])
                {
                    ced = ced.next!;
                }

                // Find the adjacent triangle
                int l = ced.nt[0] == triIter ? 1 : 0;

                _adjacents.SetValue(adjIndex, ced.nt[l]);
                _adjacents.SetValue(adjIndex + 3, ced.nn[l]);
                ++adjIndex;
            }

            adjIndex += 3;
        }
    }

    /// <summary>The triangulation analyzed.</summary>
    public Triangulation? Triangulation() => _triangulation;

    /// <summary>The index of a triangle containing node <paramref name="n"/>.</summary>
    public int Triangle(int n) => _triangles[n];

    /// <summary>
    /// The three triangles adjacent to triangle <paramref name="t"/>, across its edges n1-n2, n2-n3 and n3-n1 (OCCT's
    /// documentation says n2-n3, n3-n1 and n1-n2); 0 where there is none.
    /// </summary>
    public void Triangles(int t, out int t1, out int t2, out int t3)
    {
        // OCCT documents t1, t2, t3 as across n2-n3, n3-n1, n1-n2

        int index = 6 * (t - 1);
        t1 = _adjacents[index + 1];
        t2 = _adjacents[index + 2];
        t3 = _adjacents[index + 3];
    }

    /// <summary>The third nodes of the three triangles adjacent to triangle <paramref name="t"/>; 0 where there is none.</summary>
    public void Nodes(int t, out int n1, out int n2, out int n3)
    {
        int index = 6 * (t - 1);
        n1 = _adjacents[index + 4];
        n2 = _adjacents[index + 5];
        n3 = _adjacents[index + 6];
    }

    /// <summary>
    /// Starts iterating the triangles containing node <paramref name="n"/>:
    /// <c>for (C.Initialize(n); C.More(); C.Next()) { t = C.Value(); }</c>
    /// </summary>
    public void Initialize(int n)
    {
        _mynode = n;
        _myfirst = Triangle(n);
        _mytr = _myfirst;
        _mysense = true;
        _mymore = (_myfirst != 0);
        _passedTr.Clear();
        _passedTr.Add(_mytr);
        if (_mymore)
        {
            int i;
            _triangulation!.Triangle(_myfirst).Get(out int no0, out int no1, out int no2);
            ReadOnlySpan<int> no = [no0, no1, no2];
            for (i = 0; i < 3; i++)
            {
                if (no[i] == _mynode)
                {
                    break;
                }
            }

            _myothernode = no[(i + 2) % 3];
        }
    }

    /// <summary>True if there is a current triangle.</summary>
    public bool More() => _mymore;

    /// <summary>Moves to the next triangle around the node; nothing happens at the end.</summary>
    public void Next()
    {
        int i, j;
        Span<int> n = [0, 0, 0];
        Span<int> t = [0, 0, 0];
        Triangles(_mytr, out t[0], out t[1], out t[2]);
        if (_mysense)
        {
            for (i = 0; i < 3; i++)
            {
                if (t[i] != 0)
                {
                    _triangulation!.Triangle(t[i]).Get(out n[0], out n[1], out n[2]);
                    for (j = 0; j < 3; j++)
                    {
                        if ((n[j] == _mynode) && (n[(j + 1) % 3] == _myothernode))
                        {
                            _mytr = t[i];
                            _myothernode = n[(j + 2) % 3];
                            _mymore = !_passedTr.Contains(_mytr);
                            _passedTr.Add(_mytr);
                            return;
                        }
                    }
                }
            }

            // sinon, depart vers la gauche.
            _triangulation!.Triangle(_myfirst).Get(out n[0], out n[1], out n[2]);
            for (i = 0; i < 3; i++)
            {
                if (n[i] == _mynode)
                {
                    break;
                }
            }

            _myothernode = n[(i + 1) % 3];
            _mysense = false;
            _mytr = _myfirst;
            Triangles(_mytr, out t[0], out t[1], out t[2]);
        }

        if (!_mysense)
        {
            for (i = 0; i < 3; i++)
            {
                if (t[i] != 0)
                {
                    _triangulation!.Triangle(t[i]).Get(out n[0], out n[1], out n[2]);
                    for (j = 0; j < 3; j++)
                    {
                        if ((n[j] == _mynode) && (n[(j + 2) % 3] == _myothernode))
                        {
                            _mytr = t[i];
                            _myothernode = n[(j + 1) % 3];
                            _mymore = !_passedTr.Contains(_mytr);
                            _passedTr.Add(_mytr);
                            return;
                        }
                    }
                }
            }
        }

        _mymore = false;
    }

    /// <summary>The current triangle of the iteration.</summary>
    public int Value() => _mytr;
}
