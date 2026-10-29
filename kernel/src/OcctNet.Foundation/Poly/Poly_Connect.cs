// Port of: src/FoundationClasses/TKMath/Poly/Poly_Connect.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Connect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class Poly_Connect
{
    // this structure records one of the edges starting from a node
    private sealed class polyedge
    {
        public polyedge? next;           // the next edge in the list
        public readonly int[] nt = new int[2]; // the two adjacent triangles
        public readonly int[] nn = new int[2]; // the two adjacent nodes
        public int nd;                   // the second node of the edge
    }

    private Poly_Triangulation? myTriangulation;
    private readonly NCollection_Array1<int> myTriangles;
    private readonly NCollection_Array1<int> myAdjacents;
    private int mytr;
    private int myfirst;
    private int mynode;
    private int myothernode;
    private bool mysense;
    private bool mymore;
    private readonly NCollection_PackedMapOfInteger myPassedTr = new();

    /// <summary>An uninitialized algorithm.</summary>
    public Poly_Connect()
    {
        myTriangles = new NCollection_Array1<int>();
        myAdjacents = new NCollection_Array1<int>();
        mytr = 0;
        myfirst = 0;
        mynode = 0;
        myothernode = 0;
        mysense = false;
        mymore = false;
    }

    /// <summary>The adjacency of a triangulation.</summary>
    public Poly_Connect(Poly_Triangulation theTriangulation)
    {
        myTriangulation = theTriangulation;
        myTriangles = new NCollection_Array1<int>(1, theTriangulation.NbNodes());
        myAdjacents = new NCollection_Array1<int>(1, 6 * theTriangulation.NbTriangles());
        mytr = 0;
        myfirst = 0;
        mynode = 0;
        myothernode = 0;
        mysense = false;
        mymore = false;
        Load(theTriangulation);
    }

    /// <summary>Computes the adjacency of a triangulation.</summary>
    public void Load(Poly_Triangulation theTriangulation)
    {
        myTriangulation = theTriangulation;
        mytr = 0;
        myfirst = 0;
        mynode = 0;
        myothernode = 0;
        mysense = false;
        mymore = false;

        int aNbNodes = myTriangulation.NbNodes();
        int aNbTris = myTriangulation.NbTriangles();
        {
            int aNbAdjs = 6 * aNbTris;
            if (myTriangles.Length() != aNbNodes)
            {
                myTriangles.Resize(1, aNbNodes, false);
            }

            if (myAdjacents.Length() != aNbAdjs)
            {
                myAdjacents.Resize(1, aNbAdjs, false);
            }
        }

        myTriangles.Init(0);
        myAdjacents.Init(0);

        // We first build an array of the list of edges connected to the nodes
        // create an array to store the edges starting from the vertices
        var anEdges = new NCollection_Array1<polyedge?>(1, aNbNodes);
        anEdges.Init(null);

        // loop on the triangles
        Span<int> aTriNodes = [0, 0, 0];
        Span<int> anEdgeNodes = [0, 0];
        for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
        {
            // get the nodes
            myTriangulation.Triangle(aTriIter).Get(out aTriNodes[0], out aTriNodes[1], out aTriNodes[2]);

            // Update the myTriangles array
            myTriangles.SetValue(aTriNodes[0], aTriIter);
            myTriangles.SetValue(aTriNodes[1], aTriIter);
            myTriangles.SetValue(aTriNodes[2], aTriIter);

            // update the edge lists
            for (int aNodeInTri = 0; aNodeInTri < 3; ++aNodeInTri)
            {
                int aNodeNext = (aNodeInTri + 1) % 3; // the following node of the edge
                if (aTriNodes[aNodeInTri] < aTriNodes[aNodeNext])
                {
                    anEdgeNodes[0] = aTriNodes[aNodeInTri];
                    anEdgeNodes[1] = aTriNodes[aNodeNext];
                }
                else
                {
                    anEdgeNodes[0] = aTriNodes[aNodeNext];
                    anEdgeNodes[1] = aTriNodes[aNodeInTri];
                }

                // edge from node 0 to node 1 with node 0 < node 1
                // insert in the list of node 0
                polyedge? ced = anEdges[anEdgeNodes[0]];
                for (; ced is not null; ced = ced.next)
                {
                    // the edge already exists
                    if (ced.nd == anEdgeNodes[1])
                    {
                        // just mark the adjacency if found
                        ced.nt[1] = aTriIter;
                        ced.nn[1] = aTriNodes[3 - aNodeInTri - aNodeNext]; // the third node
                        break;
                    }
                }

                if (ced is null)
                {
                    // create the edge if not found
                    ced = new polyedge();
                    ced.next = anEdges[anEdgeNodes[0]];
                    anEdges[anEdgeNodes[0]] = ced;
                    ced.nd = anEdgeNodes[1];
                    ced.nt[0] = aTriIter;
                    ced.nn[0] = aTriNodes[3 - aNodeInTri - aNodeNext]; // the third node
                    ced.nt[1] = 0;
                    ced.nn[1] = 0;
                }
            }
        }

        // now complete the myAdjacents array
        int anAdjIndex = 1;
        for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
        {
            // get the nodes
            myTriangulation.Triangle(aTriIter).Get(out aTriNodes[0], out aTriNodes[1], out aTriNodes[2]);

            // for each edge in triangle
            for (int aNodeInTri = 0; aNodeInTri < 3; ++aNodeInTri)
            {
                int aNodeNext = (aNodeInTri + 1) % 3; // the following node of the edge
                if (aTriNodes[aNodeInTri] < aTriNodes[aNodeNext])
                {
                    anEdgeNodes[0] = aTriNodes[aNodeInTri];
                    anEdgeNodes[1] = aTriNodes[aNodeNext];
                }
                else
                {
                    anEdgeNodes[0] = aTriNodes[aNodeNext];
                    anEdgeNodes[1] = aTriNodes[aNodeInTri];
                }

                // edge from node 0 to node 1 with node 0 < node 1
                // find in the list of node 0
                polyedge ced = anEdges[anEdgeNodes[0]]!;
                while (ced.nd != anEdgeNodes[1])
                {
                    ced = ced.next!;
                }

                // Find the adjacent triangle
                int l = ced.nt[0] == aTriIter ? 1 : 0;

                myAdjacents.SetValue(anAdjIndex, ced.nt[l]);
                myAdjacents.SetValue(anAdjIndex + 3, ced.nn[l]);
                ++anAdjIndex;
            }

            anAdjIndex += 3;
        }
    }

    /// <summary>The triangulation analyzed.</summary>
    public Poly_Triangulation? Triangulation() => myTriangulation;

    /// <summary>The index of a triangle containing node <paramref name="N"/>.</summary>
    public int Triangle(int N) => myTriangles[N];

    /// <summary>
    /// The three triangles adjacent to triangle <paramref name="T"/>, across its edges n1-n2, n2-n3 and n3-n1 (OCCT's
    /// documentation says n2-n3, n3-n1 and n1-n2); 0 where there is none.
    /// </summary>
    public void Triangles(int T, out int t1, out int t2, out int t3)
    {
        int index = 6 * (T - 1);
        t1 = myAdjacents[index + 1];
        t2 = myAdjacents[index + 2];
        t3 = myAdjacents[index + 3];
    }

    /// <summary>The third nodes of the three triangles adjacent to triangle <paramref name="T"/>; 0 where there is none.</summary>
    public void Nodes(int T, out int n1, out int n2, out int n3)
    {
        int index = 6 * (T - 1);
        n1 = myAdjacents[index + 4];
        n2 = myAdjacents[index + 5];
        n3 = myAdjacents[index + 6];
    }

    /// <summary>
    /// Starts iterating the triangles containing node <paramref name="N"/>:
    /// <c>for (C.Initialize(n); C.More(); C.Next()) { t = C.Value(); }</c>
    /// </summary>
    public void Initialize(int N)
    {
        mynode = N;
        myfirst = Triangle(N);
        mytr = myfirst;
        mysense = true;
        mymore = (myfirst != 0);
        myPassedTr.Clear();
        myPassedTr.Add(mytr);
        if (mymore)
        {
            int i;
            myTriangulation!.Triangle(myfirst).Get(out int no0, out int no1, out int no2);
            ReadOnlySpan<int> no = [no0, no1, no2];
            for (i = 0; i < 3; i++)
            {
                if (no[i] == mynode)
                {
                    break;
                }
            }

            myothernode = no[(i + 2) % 3];
        }
    }

    /// <summary>True if there is a current triangle.</summary>
    public bool More() => mymore;

    /// <summary>Moves to the next triangle around the node; nothing happens at the end.</summary>
    public void Next()
    {
        int i, j;
        Span<int> n = [0, 0, 0];
        Span<int> t = [0, 0, 0];
        Triangles(mytr, out t[0], out t[1], out t[2]);
        if (mysense)
        {
            for (i = 0; i < 3; i++)
            {
                if (t[i] != 0)
                {
                    myTriangulation!.Triangle(t[i]).Get(out n[0], out n[1], out n[2]);
                    for (j = 0; j < 3; j++)
                    {
                        if ((n[j] == mynode) && (n[(j + 1) % 3] == myothernode))
                        {
                            mytr = t[i];
                            myothernode = n[(j + 2) % 3];
                            mymore = !myPassedTr.Contains(mytr);
                            myPassedTr.Add(mytr);
                            return;
                        }
                    }
                }
            }

            // sinon, depart vers la gauche.
            myTriangulation!.Triangle(myfirst).Get(out n[0], out n[1], out n[2]);
            for (i = 0; i < 3; i++)
            {
                if (n[i] == mynode)
                {
                    break;
                }
            }

            myothernode = n[(i + 1) % 3];
            mysense = false;
            mytr = myfirst;
            Triangles(mytr, out t[0], out t[1], out t[2]);
        }

        if (!mysense)
        {
            for (i = 0; i < 3; i++)
            {
                if (t[i] != 0)
                {
                    myTriangulation!.Triangle(t[i]).Get(out n[0], out n[1], out n[2]);
                    for (j = 0; j < 3; j++)
                    {
                        if ((n[j] == mynode) && (n[(j + 2) % 3] == myothernode))
                        {
                            mytr = t[i];
                            myothernode = n[(j + 1) % 3];
                            mymore = !myPassedTr.Contains(mytr);
                            myPassedTr.Add(mytr);
                            return;
                        }
                    }
                }
            }
        }

        mymore = false;
    }

    /// <summary>The current triangle of the iteration.</summary>
    public int Value() => mytr;
}
