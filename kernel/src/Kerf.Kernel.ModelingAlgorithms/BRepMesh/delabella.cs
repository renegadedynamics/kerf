// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/delabella.pxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/delabella.cpp
// Open CASCADE Technology V8_0_1 (b8f597c6). Delabella is third-party code inside OCCT, under the MIT licence below.
// Copyright (C) 2018 GUMIX - Marcin Sokalski.
// Copyright (c) 2026 The Kerf Contributors. This file, a port of MIT-licensed code, stays under the MIT licence
// below (decision 497; see THIRD-PARTY-NOTICES.md).
/*

MIT License

DELABELLA - Delaunay triangulation library
Copyright (C) 2018 GUMIX - Marcin Sokalski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Meshing.Delaunay;

// Left out: the float overload of Triangulate, the deprecated C function DelaBella(points, xy, abc, errlog) and the
// extern "C" wrappers (DelaBella_Create, ...): OCCT calls none of them.
//
// Arithmetic: delabella's typedefs (Signed14 ... Signed62) are double and long double. MSVC, which builds the native
// OCCT the port is compared with, makes long double a 64-bit double, so every one of them is double here, and every
// operation keeps the C++ order.

/// <summary>A vertex of Delabella's output: the input point's index and its coordinates.</summary>
/// <remarks>A working object of the triangulator, valid until its next <c>Triangulate</c> or <c>Destroy</c>.</remarks>
internal class DelaunayVertex
{
    /// <summary>Index of the original point.</summary>
    public int i;

    /// <summary>Coordinates (input copy).</summary>
    public double x, y;

    /// <summary>Next silhouette vertex.</summary>
    public DelaunayVertex? next;
}

/// <summary>A triangle of Delabella's output.</summary>
/// <remarks>A working object of the triangulator, valid until its next <c>Triangulate</c> or <c>Destroy</c>.</remarks>
internal class DelaunayTriangle
{
    /// <summary>The 3 vertices spanning this triangle.</summary>
    public readonly DelaunayVertex?[] v = new DelaunayVertex?[3];

    /// <summary>The 3 adjacent faces: f[i] is at the edge opposite to vertex v[i].</summary>
    public readonly DelaunayTriangle?[] f = new DelaunayTriangle?[3];

    /// <summary>Next triangle (of the Delaunay set or of the hull set).</summary>
    public DelaunayTriangle? next;
}

/// <summary>Delabella, the Delaunay triangulator OCCT's <c>BRepMesh_DelabellaBaseMeshAlgo</c> uses.</summary>
internal abstract class IDelaBella
{
    /// <summary>A new triangulator.</summary>
    public static global::Kerf.Engine.Meshing.Delaunay.IDelaBella Create()
    {
        DelaBella db = new();

        db._vertAlloc = null;
        db._faceAlloc = null;
        db._maxVerts = 0;
        db._maxFaces = 0;

        db._firstDelaFace = null;
        db._firstHullFace = null;
        db._firstHullVert = null;

        db._inpVerts = 0;
        db._outVerts = 0;

        db._errlogProc = null;
        db._errlogFile = null;

        return db;
    }

    /// <summary>Releases the triangulator's buffers. Its output must not be used afterwards.</summary>
    public abstract void Destroy();

    /// <summary>
    /// Sets the error log: <paramref name="proc"/> receives <paramref name="stream"/> and each message, formatted as
    /// C's printf would format it (with its trailing new line). OCCT's procedure is printf-style.
    /// </summary>
    public abstract void SetErrLog(Func<object?, string, int>? proc, object? stream);

    /// <summary>
    /// Triangulates the points: point i is (x[i * advance], y[i * advance]), with advance =
    /// <paramref name="advance_bytes"/> / sizeof(double), at least 2 doubles. An empty <paramref name="y"/> is C++'s
    /// null: the y coordinates follow the x ones (<c>y = x + 1</c>). An empty <paramref name="x"/> is C++'s null: no
    /// output.
    /// </summary>
    /// <returns>0: no output. Negative: all points are colinear, and the hull vertices form a colinear segment list
    /// (minus their count). Positive: 3 times the number of Delaunay triangles; the hull vertices form a
    /// counter-clockwise contour, and the Delaunay and hull triangles are available.</returns>
    /// <remarks>OCCT's interleaved call <c>Triangulate(n, &amp;p[0], &amp;p[1], 2 * sizeof(double))</c> is
    /// <c>Triangulate(n, p.AsSpan(0), p.AsSpan(1), 2 * sizeof(double))</c>. C++ addresses the points by bytes;
    /// <paramref name="advance_bytes"/> must be a multiple of sizeof(double).</remarks>
    public abstract int Triangulate(int points, ReadOnlySpan<double> x, ReadOnlySpan<double> y = default,
                                    int advance_bytes = 0);

    /// <summary>Number of points passed to the last call to Triangulate().</summary>
    public abstract int GetNumInputPoints();

    /// <summary>Number of vertices returned from the last call to Triangulate().</summary>
    public abstract int GetNumOutputVerts();

    /// <summary>The first Delaunay triangle; valid only if Triangulate() > 0.</summary>
    public abstract DelaunayTriangle? GetFirstDelaunayTriangle();

    /// <summary>The first hull triangle; valid only if Triangulate() > 0.</summary>
    public abstract DelaunayTriangle? GetFirstHullTriangle();

    /// <summary>The first hull vertex: if Triangulate() &lt; 0 a list, otherwise a closed contour.</summary>
    public abstract DelaunayVertex? GetFirstHullVertex();
}

internal sealed class DelaBella : global::Kerf.Engine.Meshing.Delaunay.IDelaBella
{
    // Unsigned28 s14sqr(const Signed14& s)
    private static double S14sqr(double s)
    {
        return s * s;
    }

    internal struct Norm
    {
        public double x; // Signed45
        public double y; // Signed45
        public double z; // Signed31
    }

    internal struct Vect
    {
        public double x, y; // Signed15
        public double z;    // Signed29

        public readonly Norm Cross(in Vect v) // cross prod
        {
            Norm n;
            n.x = y * v.z - z * v.y;
            n.y = z * v.x - x * v.z;
            n.z = x * v.y - y * v.x;
            return n;
        }
    }

    internal sealed class Vert : DelaunayVertex
    {
        public double z; // Unsigned28
        public Face? sew;

        public static Vect operator -(Vert a, Vert v) // diff
        {
            Vect d;
            d.x = a.x - v.x;
            d.y = a.y - v.y;
            d.z = a.z - v.z;
            return d;
        }

        public static bool Overlap(Vert v1, Vert v2) { return v1.x == v2.x && v1.y == v2.y; }

        public static int U28cmp(Vert va, Vert vb)
        {
            if (va.z < vb.z)
            {
                return -1;
            }
            if (va.z > vb.z)
            {
                return 1;
            }
            if (va.y < vb.y)
            {
                return -1;
            }
            if (va.y > vb.y)
            {
                return 1;
            }
            if (va.x < vb.x)
            {
                return -1;
            }
            if (va.x > vb.x)
            {
                return 1;
            }
            if (va.i < vb.i)
            {
                return -1;
            }
            if (va.i > vb.i)
            {
                return 1;
            }
            return 0;
        }

        // The C++ copies Vert structs by value (_vertAlloc[w++] = _vertAlloc[r++]).
        public void Assign(Vert other)
        {
            i = other.i;
            x = other.x;
            y = other.y;
            next = other.next;
            z = other.z;
            sew = other.sew;
        }
    }

    internal sealed class Face : DelaunayTriangle
    {
        public Norm n;

        // The position in _faceAlloc (pointer arithmetic over it in C++); -1 for the temporary face.
        public readonly int index;

        public Face(int theIndex) => index = theIndex;

        public static Face Alloc(ref Face? from)
        {
            Face f = from!;
            from = (Face?)f.next;
            f.next = null;
            return f;
        }

        public void Free(ref Face? to)
        {
            next = to;
            to = this;
        }

        public Face? Next(Vert p)
        {
            // return next face in same direction as face vertices are (cw/ccw)

            if (v[0] == p)
            {
                return (Face?)f[1];
            }
            if (v[1] == p)
            {
                return (Face?)f[2];
            }
            if (v[2] == p)
            {
                return (Face?)f[0];
            }
            return null;
        }

        public double Dot(Vert p) // dot
        {
            Vect d = p - (Vert)v[0]!;
            return n.x * d.x + n.y * d.y + n.z * d.z;
        }

        public Norm Cross() // cross of diffs
        {
            return ((Vert)v[1]! - (Vert)v[0]!).Cross((Vert)v[2]! - (Vert)v[0]!);
        }
    }

    [InlineArray(3)]
    private struct Vert3
    {
        private Vert? _element0;
    }

    [InlineArray(3)]
    private struct Face3
    {
        private Face? _element0;
    }

    // ab: given face adjacency [index][], it provides [][2] vertex indices on shared edge (CCW order)
    private static readonly int[,] Ab = { { 1, 2 }, { 2, 0 }, { 0, 1 } };

    private static readonly Comparer<Vert> VertLess = Comparer<Vert>.Create(Vert.U28cmp);

    internal Vert[]? _vertAlloc;
    internal Face[]? _faceAlloc;
    internal int _maxVerts;
    internal int _maxFaces;

    internal Face? _firstDelaFace;
    internal Face? _firstHullFace;
    internal Vert? _firstHullVert;

    internal int _inpVerts;
    internal int _outVerts;

    internal Func<object?, string, int>? _errlogProc;
    internal object? _errlogFile;

    private void Errlog(string message)
    {
        _errlogProc!(_errlogFile, message);
    }

    private int Triangulate()
    {
        int points = _inpVerts;
        // std::sort(_vertAlloc, _vertAlloc + points): u28cmp ends on the unique input index i, so the order is total
        // and any correct sort gives MSVC's std::sort result (decision 491's tie check). Not with NaN coordinates,
        // where the C++ order isn't a strict weak order either.
        if (points > 0) // std::sort of an empty range touches nothing, even a null _vertAlloc
        {
            Array.Sort(_vertAlloc!, 0, points, VertLess);
        }
        Vert[] verts = _vertAlloc!; // OCCT: _vertAlloc is null on a first call with 0 points (below)

        // remove dups
        {
            int w = 0, r = 1; // skip initial no-dups block
            while (r < points && !Vert.Overlap(verts[r], verts[w]))
            {
                w++;
                r++;
            }

            w++;

            while (r < points)
            {
                r++;

                // skip dups
                while (r < points && Vert.Overlap(verts[r], verts[r - 1]))
                {
                    r++;
                }

                // copy next no-dups block
                while (r < points && !Vert.Overlap(verts[r], verts[r - 1]))
                {
                    verts[w++].Assign(verts[r++]);
                }
            }

            // OCCT: with 0 points, w is 1 here: "detected -1 dups", then points = 1 and the single-point branch
            // writes _vertAlloc[0] of an empty (null) allocation. The port throws there (NullReferenceException).
            if (points - w != 0)
            {
                if (_errlogProc is not null)
                {
                    Errlog(string.Format(CultureInfo.InvariantCulture, "[WRN] detected {0} dups in xy array!\n", points - w));
                }
                points = w;
            }
        }

        if (points < 3)
        {
            if (points == 2)
            {
                if (_errlogProc is not null)
                {
                    Errlog("[WRN] all input points are colinear, returning single segment!\n");
                }
                _firstHullVert = verts[0];
                verts[0].next = verts[1];
                verts[1].next = null;
            }
            else
            {
                if (_errlogProc is not null)
                {
                    Errlog("[WRN] all input points are identical, returning single point!\n");
                }
                _firstHullVert = verts[0];
                verts[0].next = null;
            }

            return -points;
        }

        int hull_faces = 2 * points - 4;

        if (_maxFaces < hull_faces)
        {
            if (_maxFaces != 0)
            {
                _faceAlloc = null;
            }
            _maxFaces = 0;
            _faceAlloc = new Face[hull_faces];
            for (int k = 0; k < hull_faces; k++)
            {
                _faceAlloc[k] = new Face(k);
            }
            _maxFaces = hull_faces;
        }

        Face[] faces = _faceAlloc!;
        for (int k = 1; k < hull_faces; k++)
        {
            faces[k - 1].next = faces[k];
        }
        faces[hull_faces - 1].next = null;

        Face? cache = faces[0];
        Face? hull = null;

        Face f = new(-1); // tmp
        f.v[0] = verts[0];
        f.v[1] = verts[1];
        f.v[2] = verts[2];
        f.n = f.Cross();

        bool colinear = f.n.z == 0;
        int i = 3;

        /////////////////////////////////////////////////////////////////////////
        // UNTIL INPUT IS COPLANAR, GROW IT IN FORM OF A 2D CONTOUR
        /*
        . |                |         after adding     . |        ________* L
        . \ Last points to / Head     next point      . \ ______/        /
        .  *____          |             ----->        .H *____          |
        .  |\_  \_____    |                           .  |\_  \_____    |
        .   \ \_      \__* - Tail points to Last      .   \ \_      \__* T
        .    \  \_      /                             .    \  \_      /
        .     \__ \_ __/                              .     \__ \_ __/
        .        \__* - Head points to Tail           .        \__/
        */

        Vert head = (Vert)f.v[0]!;
        Vert tail = (Vert)f.v[1]!;
        Vert last = (Vert)f.v[2]!;

        head.next = tail;
        tail.next = last;
        last.next = head;

        while (i < points && f.Dot(verts[i]) == 0)
        {
            Vert v = verts[i];

            // it is enough to test just 1 non-zero coord
            // but we want also to test stability (assert)
            // so we calc all signs...

            // why not testing sign of dot prod of 2 normals?
            // that way we'd fall into precision problems

            Norm lvH = (v - last).Cross(head - last);
            bool lvh = (f.n.x > 0 && lvH.x > 0) || (f.n.x < 0 && lvH.x < 0) || (f.n.y > 0 && lvH.y > 0)
                       || (f.n.y < 0 && lvH.y < 0) || (f.n.z > 0 && lvH.z > 0)
                       || (f.n.z < 0 && lvH.z < 0);

            Norm tvL = (v - tail).Cross(last - tail);
            bool tvl = (f.n.x > 0 && tvL.x > 0) || (f.n.x < 0 && tvL.x < 0) || (f.n.y > 0 && tvL.y > 0)
                       || (f.n.y < 0 && tvL.y < 0) || (f.n.z > 0 && tvL.z > 0)
                       || (f.n.z < 0 && tvL.z < 0);

            if (lvh && !tvl) // insert new f on top of e(2,0) = (last,head)
            {
                // f.v[0] = head;
                f.v[1] = last;
                f.v[2] = v;

                last.next = v;
                v.next = head;
                tail = last;
            }
            else if (tvl && !lvh) // insert new f on top of e(1,2) = (tail,last)
            {
                f.v[0] = last;
                // f.v[1] = tail;
                f.v[2] = v;

                tail.next = v;
                v.next = last;
                head = last;
            }
            else
            {
                // wtf? dilithium crystals are fucked.
                // OCCT: assert(0), a no-op in release builds: the point is taken as `last` without being linked
                // into the contour.
            }

            last = v;
            i++;
        }

        if (i == points)
        {
            if (colinear)
            {
                if (_errlogProc is not null)
                {
                    Errlog("[WRN] all input points are colinear, returning segment list!\n");
                }
                _firstHullVert = head;
                last.next = null; // break contour, make it a list
                return -points;
            }
            else
            {
                if (points > 3)
                {
                    if (_errlogProc is not null)
                    {
                        Errlog("[NFO] all input points are cocircular.\n");
                    }
                }
                else
                {
                    if (_errlogProc is not null)
                    {
                        Errlog("[NFO] trivial case of 3 points, thank you.\n");
                    }

                    _firstDelaFace = Face.Alloc(ref cache);
                    _firstDelaFace.next = null;
                    _firstHullFace = Face.Alloc(ref cache);
                    _firstHullFace.next = null;

                    _firstDelaFace.f[0] = _firstDelaFace.f[1] = _firstDelaFace.f[2] = _firstHullFace;
                    _firstHullFace.f[0] = _firstHullFace.f[1] = _firstHullFace.f[2] = _firstDelaFace;

                    head.sew = tail.sew = last.sew = _firstHullFace;

                    if (f.n.z < 0)
                    {
                        _firstDelaFace.v[0] = head;
                        _firstDelaFace.v[1] = tail;
                        _firstDelaFace.v[2] = last;
                        _firstHullFace.v[0] = last;
                        _firstHullFace.v[1] = tail;
                        _firstHullFace.v[2] = head;

                        // reverse silhouette
                        head.next = last;
                        last.next = tail;
                        tail.next = head;

                        _firstHullVert = last;
                    }
                    else
                    {
                        _firstDelaFace.v[0] = last;
                        _firstDelaFace.v[1] = tail;
                        _firstDelaFace.v[2] = head;
                        _firstHullFace.v[0] = head;
                        _firstHullFace.v[1] = tail;
                        _firstHullFace.v[2] = last;

                        _firstHullVert = head;
                    }

                    _firstDelaFace.n = _firstDelaFace.Cross();
                    _firstHullFace.n = _firstHullFace.Cross();

                    return 3;
                }

                // retract last point it will be added as a cone's top later
                // OCCT: with all points cocircular, the cone's top is coplanar with its base and no point is left
                // for the main loop: the final loop then reads the adjacency of faces this run never allocated.
                // Native crashes (access violation); the port throws NullReferenceException there.
                last = head;
                head = (Vert)head.next!;
                i--;
            }
        }

        /////////////////////////////////////////////////////////////////////////
        // CREATE CONE HULL WITH TOP AT cloud[i] AND BASE MADE OF CONTOUR LIST
        // in 2 ways :( - depending on at which side of the contour a top vertex appears

        Vert q = verts[i];

        if (f.Dot(q) > 0)
        {
            Vert p = last;
            Vert n = (Vert)p.next!;

            Face first_side = Face.Alloc(ref cache);
            first_side.v[0] = p;
            first_side.v[1] = n;
            first_side.v[2] = q;
            first_side.n = first_side.Cross();
            hull = first_side;

            p = n;
            n = (Vert)n.next!;

            Face prev_side = first_side;
            Face? prev_base = null;
            Face? first_base = null;

            do
            {
                Face @base = Face.Alloc(ref cache);
                @base.v[0] = n;
                @base.v[1] = p;
                @base.v[2] = last;
                @base.n = @base.Cross();

                Face side = Face.Alloc(ref cache);
                side.v[0] = p;
                side.v[1] = n;
                side.v[2] = q;
                side.n = side.Cross();

                side.f[2] = @base;
                @base.f[2] = side;

                side.f[1] = prev_side;
                prev_side.f[0] = side;

                @base.f[0] = prev_base;
                if (prev_base is not null)
                {
                    prev_base.f[1] = @base;
                }
                else
                {
                    first_base = @base;
                }

                prev_base = @base;
                prev_side = side;

                p = n;
                n = (Vert)n.next!;
            } while (n != last);

            Face last_side = Face.Alloc(ref cache);
            last_side.v[0] = p;
            last_side.v[1] = n;
            last_side.v[2] = q;
            last_side.n = last_side.Cross();

            last_side.f[1] = prev_side;
            prev_side.f[0] = last_side;

            last_side.f[0] = first_side;
            first_side.f[1] = last_side;

            first_base!.f[0] = first_side;
            first_side.f[2] = first_base;

            last_side.f[2] = prev_base;
            prev_base!.f[1] = last_side;

            i++;
        }
        else
        {
            Vert p = last;
            Vert n = (Vert)p.next!;

            Face first_side = Face.Alloc(ref cache);
            first_side.v[0] = n;
            first_side.v[1] = p;
            first_side.v[2] = q;
            first_side.n = first_side.Cross();
            hull = first_side;

            p = n;
            n = (Vert)n.next!;

            Face prev_side = first_side;
            Face? prev_base = null;
            Face? first_base = null;

            do
            {
                Face @base = Face.Alloc(ref cache);
                @base.v[0] = p;
                @base.v[1] = n;
                @base.v[2] = last;
                @base.n = @base.Cross();

                Face side = Face.Alloc(ref cache);
                side.v[0] = n;
                side.v[1] = p;
                side.v[2] = q;
                side.n = side.Cross();

                side.f[2] = @base;
                @base.f[2] = side;

                side.f[0] = prev_side;
                prev_side.f[1] = side;

                @base.f[1] = prev_base;
                if (prev_base is not null)
                {
                    prev_base.f[0] = @base;
                }
                else
                {
                    first_base = @base;
                }

                prev_base = @base;
                prev_side = side;

                p = n;
                n = (Vert)n.next!;
            } while (n != last);

            Face last_side = Face.Alloc(ref cache);
            last_side.v[0] = n;
            last_side.v[1] = p;
            last_side.v[2] = q;
            last_side.n = last_side.Cross();

            last_side.f[0] = prev_side;
            prev_side.f[1] = last_side;

            last_side.f[1] = first_side;
            first_side.f[0] = last_side;

            first_base!.f[1] = first_side;
            first_side.f[2] = first_base;

            last_side.f[2] = prev_base;
            prev_base!.f[0] = last_side;

            i++;
        }

        /////////////////////////////////////////////////////////////////////////
        // ACTUAL ALGORITHM

        for (; i < points; i++)
        {
            // ValidateHull(alloc, 2 * i - 4);

            Vert _q = verts[i];
            Vert _p = verts[i - 1];
            Face _f = hull!;

            // 1. FIND FIRST VISIBLE FACE
            //    simply iterate around last vertex using last added triangle adjecency info
            while (_f.Dot(_q) <= 0)
            {
                _f = _f.Next(_p)!;
                if (_f == hull)
                {
                    // if no visible face can be located at last vertex,
                    // let's run through all faces (approximately last to first),
                    // yes this is emergency fallback and should not ever happen.
                    _f = faces[2 * i - 4 - 1];
                    while (_f.Dot(_q) <= 0)
                    {
                        // OCCT: assert(_f != _faceAlloc), a no-op in release builds, where _f then runs below
                        // _faceAlloc; the port throws IndexOutOfRangeException there.
                        _f = faces[_f.index - 1];
                    }
                }
            }

            // 2. DELETE VISIBLE FACES & ADD NEW ONES
            //    (we also build silhouette (vertex loop) between visible & invisible faces)

            // push first visible face onto stack (of visible faces)
            Face? stack = _f;
            _f.next = _f; // old trick to use list pointers as 'on-stack' markers
            while (stack is not null)
            {
                // pop, take care of last item ptr (it's not null!)
                _f = stack;
                stack = (Face?)_f.next;
                if (stack == _f)
                {
                    stack = null;
                }
                _f.next = null;

                // copy parts of old face that we still need after removal
                Vert3 fv = default;
                fv[0] = (Vert?)_f.v[0];
                fv[1] = (Vert?)_f.v[1];
                fv[2] = (Vert?)_f.v[2];
                Face3 ff = default;
                ff[0] = (Face?)_f.f[0];
                ff[1] = (Face?)_f.f[1];
                ff[2] = (Face?)_f.f[2];

                // delete visible face
                _f.Free(ref cache);

                // check all 3 neighbors
                for (int e = 0; e < 3; e++)
                {
                    Face? n = ff[e];
                    if (n is not null && n.next is null) // ensure neighbor is not processed yet & isn't on stack
                    {
                        if (n.Dot(_q) <= 0) // if neighbor is not visible we have slihouette edge
                        {
                            // build face

                            Vert a = fv[Ab[e, 0]]!;
                            Vert b = fv[Ab[e, 1]]!;

                            Face s = Face.Alloc(ref cache);
                            s.v[0] = a;
                            s.v[1] = b;
                            s.v[2] = _q;

                            s.n = s.Cross();
                            s.f[2] = n;

                            // change neighbour's adjacency from old visible face to cone side
                            if (n.f[0] == _f)
                            {
                                n.f[0] = s;
                            }
                            else if (n.f[1] == _f)
                            {
                                n.f[1] = s;
                            }
                            else if (n.f[2] == _f)
                            {
                                n.f[2] = s;
                            }
                            else
                            {
                                // assert(0);
                            }

                            // build silhouette needed for sewing sides in the second pass
                            a.sew = s;
                            a.next = b;
                        }
                        else
                        {
                            // disjoin visible faces
                            // so they won't be processed more than once

                            if (n.f[0] == _f)
                            {
                                n.f[0] = null;
                            }
                            else if (n.f[1] == _f)
                            {
                                n.f[1] = null;
                            }
                            else if (n.f[2] == _f)
                            {
                                n.f[2] = null;
                            }
                            else
                            {
                                // assert(0);
                            }

                            // push neighbor face, it's visible and requires processing
                            n.next = stack ?? n;
                            stack = n;
                        }
                    }
                }
            }

            // 3. SEW SIDES OF CONE BUILT ON SLIHOUTTE SEGMENTS

            hull = faces[2 * i - 4 + 1]; // last added face

            // last face must contain part of the silhouette
            // (edge between its v[0] and v[1])
            Vert entry = (Vert)hull.v[0]!;

            Vert pr = entry;
            do
            {
                // sew pr<->nx
                Vert nx = (Vert)pr.next!;
                pr.sew!.f[0] = nx.sew;
                nx.sew!.f[1] = pr.sew;
                pr = nx;
            } while (pr != entry);
        }

        // assert(2 * i - 4 == hull_faces);
        // ValidateHull(alloc, hull_faces);

        // needed?
        for (i = 0; i < points; i++)
        {
            verts[i].next = null;
            verts[i].sew = null;
        }

        // Face** prev_dela = &_firstDelaFace (and prev_hull): the last face linked, null for the list's head.
        i = 0;
        Face? prev_dela = null;
        Face? prev_hull = null;
        for (int j = 0; j < hull_faces; j++)
        {
            Face _f = faces[j];
            if (_f.n.z < 0)
            {
                if (prev_dela is null)
                {
                    _firstDelaFace = _f;
                }
                else
                {
                    prev_dela.next = _f;
                }
                prev_dela = _f;
                i++;
            }
            else
            {
                if (prev_hull is null)
                {
                    _firstHullFace = _f;
                }
                else
                {
                    prev_hull.next = _f;
                }
                prev_hull = _f;
                if (((Face)_f.f[0]!).n.z < 0)
                {
                    _f.v[1]!.next = _f.v[2];
                    ((Vert)_f.v[1]!).sew = _f;
                }
                if (((Face)_f.f[1]!).n.z < 0)
                {
                    _f.v[2]!.next = _f.v[0];
                    ((Vert)_f.v[2]!).sew = _f;
                }
                if (((Face)_f.f[2]!).n.z < 0)
                {
                    _f.v[0]!.next = _f.v[1];
                    ((Vert)_f.v[0]!).sew = _f;
                }
            }
        }

        // *prev_dela = nullptr; *prev_hull = nullptr;
        if (prev_dela is null)
        {
            _firstDelaFace = null;
        }
        else
        {
            prev_dela.next = null;
        }
        if (prev_hull is null)
        {
            _firstHullFace = null;
        }
        else
        {
            prev_hull.next = null;
        }

        _firstHullVert = (Vert)_firstHullFace!.v[0]!;

        // todo link slihouette verts into contour
        // and sew its edges with hull faces

        return 3 * i;
    }

    private bool ReallocVerts(int points)
    {
        _inpVerts = points;
        _outVerts = 0;

        _firstDelaFace = null;
        _firstHullFace = null;
        _firstHullVert = null;

        if (_maxVerts < points)
        {
            if (_maxVerts != 0)
            {
                _vertAlloc = null;
                _maxVerts = 0;
            }

            // malloc(sizeof(Vert) * points); the "[ERR] Not enough memory" branch can't be reached
            _vertAlloc = new Vert[points];
            for (int k = 0; k < points; k++)
            {
                _vertAlloc[k] = new Vert();
            }
            _maxVerts = points;
        }

        return true;
    }

    public override int Triangulate(int points, ReadOnlySpan<double> x, ReadOnlySpan<double> y = default,
                                    int advance_bytes = 0)
    {
        if (x.IsEmpty)
        {
            return 0;
        }

        if (y.IsEmpty)
        {
            y = x[1..];
        }

        if (advance_bytes < sizeof(double) * 2)
        {
            advance_bytes = sizeof(double) * 2;
        }

        if (advance_bytes % sizeof(double) != 0)
        {
            throw new ProgramErrorException("IDelaBella::Triangulate: advance_bytes must be a multiple of sizeof(double)");
        }

        if (!ReallocVerts(points))
        {
            return 0;
        }

        int advance = advance_bytes / sizeof(double);
        for (int i = 0; i < points; i++)
        {
            Vert v = _vertAlloc![i];
            v.i = i;
            v.x = x[i * advance];
            v.y = y[i * advance];
            v.z = S14sqr(v.x) + S14sqr(v.y);
        }

        _outVerts = Triangulate();
        return _outVerts;
    }

    public override void Destroy()
    {
        _faceAlloc = null;
        _vertAlloc = null;
        _maxFaces = 0;
        _maxVerts = 0;
        _firstDelaFace = null;
        _firstHullFace = null;
        _firstHullVert = null;
    }

    public override int GetNumInputPoints() { return _inpVerts; }

    public override int GetNumOutputVerts() { return _outVerts; }

    public override DelaunayTriangle? GetFirstDelaunayTriangle() { return _firstDelaFace; }

    public override DelaunayTriangle? GetFirstHullTriangle() { return _firstHullFace; }

    public override DelaunayVertex? GetFirstHullVertex() { return _firstHullVert; }

    public override void SetErrLog(Func<object?, string, int>? proc, object? stream)
    {
        _errlogProc = proc;
        _errlogFile = stream;
    }
}
