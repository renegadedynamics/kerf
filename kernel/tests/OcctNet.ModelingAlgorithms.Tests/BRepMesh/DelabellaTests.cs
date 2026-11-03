// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Delabella's triangulator (delabella.cs): structure, Delaunay property and degenerate inputs.</summary>
public class DelabellaTests
{
    private sealed record Result(int Verts, List<(int A, int B, int C)> Triangles, List<int> Hull, List<string> Log);

    // OCCT's call shape: an interleaved array, x at [0], y at [1], 2 doubles apart.
    private static Result Run(double[] theXY, global::Kerf.Engine.Meshing.Delaunay.IDelaBella? theTriangulator = null)
    {
        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aDb = theTriangulator ?? global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        List<string> aLog = [];
        aDb.SetErrLog((theStream, theMsg) => { aLog.Add(theMsg); return 0; }, null);
        int aVerts = aDb.Triangulate(theXY.Length / 2, theXY.AsSpan(0), theXY.AsSpan(1), 2 * sizeof(double));

        List<(int, int, int)> aTris = [];
        List<int> aHull = [];
        if (aVerts > 0)
        {
            for (global::Kerf.Engine.Meshing.Delaunay.DelaunayTriangle? t = aDb.GetFirstDelaunayTriangle(); t is not null; t = t.next)
            {
                aTris.Add((t.v[0]!.i, t.v[1]!.i, t.v[2]!.i));
            }

            global::Kerf.Engine.Meshing.Delaunay.DelaunayVertex aFirst = aDb.GetFirstHullVertex()!;
            global::Kerf.Engine.Meshing.Delaunay.DelaunayVertex? aV = aFirst;
            do
            {
                aHull.Add(aV!.i);
                aV = aV.next;
                Assert.True(aHull.Count <= theXY.Length / 2);
            } while (aV != aFirst);
        }
        else if (aVerts < 0)
        {
            for (global::Kerf.Engine.Meshing.Delaunay.DelaunayVertex? aV = aDb.GetFirstHullVertex(); aV is not null; aV = aV.next)
            {
                aHull.Add(aV.i);
            }
        }

        if (theTriangulator is null)
        {
            aDb.Destroy();
        }

        return new Result(aVerts, aTris, aHull, aLog);
    }

    private static double Orient(double[] p, int a, int b, int c)
        => (p[2 * b] - p[2 * a]) * (p[2 * c + 1] - p[2 * a + 1]) - (p[2 * b + 1] - p[2 * a + 1]) * (p[2 * c] - p[2 * a]);

    // > 0 when d is inside the circumcircle of the counter-clockwise triangle (a, b, c)
    private static double InCircle(double[] p, int a, int b, int c, int d)
    {
        double adx = p[2 * a] - p[2 * d], ady = p[2 * a + 1] - p[2 * d + 1];
        double bdx = p[2 * b] - p[2 * d], bdy = p[2 * b + 1] - p[2 * d + 1];
        double cdx = p[2 * c] - p[2 * d], cdy = p[2 * c + 1] - p[2 * d + 1];
        return (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
             - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
             + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
    }

    private static void CheckTriangulation(double[] p, Result r, int theUniquePoints)
    {
        Assert.True(r.Verts > 0);
        Assert.Equal(r.Verts, 3 * r.Triangles.Count);

        // Delaunay triangles come clockwise (n.z < 0 of the lifted face); OCCT's 4D reverses them (v[0], v[2], v[1])
        foreach ((int a, int b, int c) in r.Triangles)
        {
            Assert.True(Orient(p, a, b, c) < 0, $"triangle ({a},{b},{c}) is not clockwise");
        }

        // Euler: T = 2n - 2 - h
        Assert.Equal(2 * theUniquePoints - 2 - r.Hull.Count, r.Triangles.Count);

        // every edge is shared by at most two triangles, with opposite directions
        HashSet<(int, int)> aDirected = [];
        foreach ((int a, int b, int c) in r.Triangles)
        {
            Assert.True(aDirected.Add((a, b)));
            Assert.True(aDirected.Add((b, c)));
            Assert.True(aDirected.Add((c, a)));
        }

        // empty circumcircles (with a tolerance relative to the triangle's scale)
        HashSet<int> aUsed = [.. r.Triangles.SelectMany(t => new[] { t.A, t.B, t.C })];
        foreach ((int a, int b, int c) in r.Triangles)
        {
            double aScale = 0;
            foreach (int k in new[] { a, b, c })
            {
                aScale = Math.Max(aScale, Math.Abs(p[2 * k]) + Math.Abs(p[2 * k + 1]));
            }
            double aTol = 1e-9 * Math.Pow(aScale + 1, 4);
            foreach (int d in aUsed)
            {
                if (d == a || d == b || d == c)
                {
                    continue;
                }
                // (a, c, b) is counter-clockwise
                Assert.True(InCircle(p, a, c, b, d) <= aTol, $"point {d} inside the circumcircle of ({a},{b},{c})");
            }
        }
    }

    private static double[] RandomPoints(int theSeed, int theCount)
    {
        Random aRand = new(theSeed);
        double[] p = new double[2 * theCount];
        for (int i = 0; i < p.Length; i++)
        {
            p[i] = aRand.NextDouble() - 0.5;
        }
        return p;
    }

    [Theory]
    [InlineData(1, 4)]
    [InlineData(2, 10)]
    [InlineData(3, 100)]
    [InlineData(4, 1000)]
    public void Random_points_give_a_Delaunay_triangulation(int theSeed, int theCount)
    {
        double[] p = RandomPoints(theSeed, theCount);
        Result r = Run(p);
        CheckTriangulation(p, r, theCount);
        Assert.Equal(theCount, r.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct().Count());
    }

    [Fact]
    public void Repeated_runs_give_the_same_output_also_on_a_reused_triangulator()
    {
        double[] p = RandomPoints(7, 300);
        Result r1 = Run(p);
        Result r2 = Run(p);
        Assert.Equal(r1.Triangles, r2.Triangles);
        Assert.Equal(r1.Hull, r2.Hull);

        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aDb = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        Run(RandomPoints(8, 500), aDb); // a larger run first, so buffers are reused
        Result r3 = Run(p, aDb);
        Assert.Equal(r1.Triangles, r3.Triangles);
        Assert.Equal(r1.Hull, r3.Hull);
        Assert.Equal(300, aDb.GetNumInputPoints());
        Assert.Equal(r1.Verts, aDb.GetNumOutputVerts());
        aDb.Destroy();
    }

    [Fact]
    public void A_grid_is_triangulated_whole()
    {
        const int n = 8;
        double[] p = new double[2 * n * n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                p[2 * (i * n + j)] = i / (double)n - 0.5;
                p[2 * (i * n + j) + 1] = j / (double)n - 0.5;
            }
        }

        Result r = Run(p);
        Assert.True(r.Verts > 0);
        // every cell is split in two: the union is the square
        Assert.Equal(2 * (n - 1) * (n - 1), r.Triangles.Count);
        double anArea = r.Triangles.Sum(t => -Orient(p, t.A, t.B, t.C)) / 2;
        Assert.Equal(((n - 1) / (double)n) * ((n - 1) / (double)n), anArea, 12);
        foreach ((int a, int b, int c) in r.Triangles)
        {
            Assert.True(Orient(p, a, b, c) < 0);
        }
    }

    [Fact]
    public void Duplicates_are_reported_and_dropped()
    {
        double[] p = RandomPoints(11, 50);
        double[] q = new double[p.Length + 6];
        p.CopyTo(q, 0);
        // three duplicates: of points 3, 3 again and 17
        q[100] = p[6]; q[101] = p[7];
        q[102] = p[6]; q[103] = p[7];
        q[104] = p[34]; q[105] = p[35];

        Result r = Run(q);
        Assert.Contains("[WRN] detected 3 dups in xy array!\n", r.Log);
        CheckTriangulation(q, r, 50);
        // the duplicate kept is the one with the lowest index (u28cmp ends on the index)
        HashSet<int> aUsed = [.. r.Triangles.SelectMany(t => new[] { t.A, t.B, t.C })];
        Assert.Equal(50, aUsed.Count);
        Assert.DoesNotContain(50, aUsed);
        Assert.DoesNotContain(51, aUsed);
        Assert.DoesNotContain(52, aUsed);
    }

    [Fact]
    public void Colinear_points_give_a_segment_list()
    {
        double[] p = [0, 0, 3, 3, 1, 1, 2, 2, -1, -1];
        Result r = Run(p);
        Assert.Equal(-5, r.Verts);
        Assert.Contains("[WRN] all input points are colinear, returning segment list!\n", r.Log);
        Assert.Equal(5, r.Hull.Count);
        Assert.Equal(5, r.Hull.Distinct().Count());
        // the 2D contour grown point by point, broken after `last`: not in order along the line (native gives the same)
        Assert.Equal([3, 2, 0, 4, 1], r.Hull);
    }

    [Fact]
    public void Two_distinct_points_give_a_single_segment()
    {
        double[] p = [0, 0, 1, 0, 0, 0];
        Result r = Run(p);
        Assert.Equal(-2, r.Verts);
        Assert.Contains("[WRN] detected 1 dups in xy array!\n", r.Log);
        Assert.Contains("[WRN] all input points are colinear, returning single segment!\n", r.Log);
        Assert.Equal([0, 1], r.Hull);
    }

    [Fact]
    public void Identical_points_give_a_single_point()
    {
        double[] p = [0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5];
        Result r = Run(p);
        Assert.Equal(-1, r.Verts);
        Assert.Contains("[WRN] detected 3 dups in xy array!\n", r.Log);
        Assert.Contains("[WRN] all input points are identical, returning single point!\n", r.Log);
        Assert.Equal([0], r.Hull);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Three_points_give_one_clockwise_triangle(bool isReversed)
    {
        double[] p = isReversed ? [0, 0, 0, 1, 1, 0] : [0, 0, 1, 0, 0, 1];
        Result r = Run(p);
        Assert.Equal(3, r.Verts);
        Assert.Contains("[NFO] trivial case of 3 points, thank you.\n", r.Log);
        Assert.Single(r.Triangles);
        (int a, int b, int c) = r.Triangles[0];
        Assert.True(Orient(p, a, b, c) < 0);
        Assert.Equal(3, r.Hull.Count);
        // the hull contour is counter-clockwise
        Assert.True(Orient(p, r.Hull[0], r.Hull[1], r.Hull[2]) > 0);
    }

    [Fact]
    public void Nearly_cocircular_points_are_triangulated()
    {
        // eight points on a circle through cos and sin: not exactly cocircular in doubles
        double[] p = new double[16];
        for (int k = 0; k < 8; k++)
        {
            p[2 * k] = 0.5 * Math.Cos(k * Math.PI / 4);
            p[2 * k + 1] = 0.5 * Math.Sin(k * Math.PI / 4);
        }

        Result r = Run(p);
        Assert.True(r.Verts > 0);
        Assert.Equal(6, r.Triangles.Count);
        double anArea = r.Triangles.Sum(t => -Orient(p, t.A, t.B, t.C)) / 2;
        Assert.Equal(8 * 0.5 * 0.25 * Math.Sin(Math.PI / 4), anArea, 12);
    }

    [Fact]
    public void Exactly_cocircular_points_fail_as_in_OCCT_ReproducesOcctError()
    {
        // OCCT: when more than 3 points are all cocircular ("[NFO] all input points are cocircular."), the cone is
        // built on a coplanar top and the final loop reads the adjacency of faces never allocated in this run.
        // Native (MSVC, release) crashes with an access violation on the unit square's corners and on the twelve
        // integer points of the circle of radius 5; the port throws NullReferenceException.
        double[] aSquare = [1, 0, -1, 0, 0, 1, 0, -1];
        List<string> aLog = [];
        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aDb = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        aDb.SetErrLog((s, m) => { aLog.Add(m); return 0; }, null);
        Assert.ThrowsAny<NullReferenceException>(() => aDb.Triangulate(4, aSquare));
        Assert.Contains("[NFO] all input points are cocircular.\n", aLog);

        double[] aCircle5 = [5, 0, -5, 0, 0, 5, 0, -5, 3, 4, 3, -4, -3, 4, -3, -4, 4, 3, 4, -3, -4, 3, -4, -3];
        Assert.ThrowsAny<NullReferenceException>(() => global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create().Triangulate(12, aCircle5));
    }

    // Integer coordinates on a 1/1024 grid, exactly representable, from the LCG of C's rand() family.
    private static double[] LcgPoints(long theSeed, int theCount)
    {
        long s = theSeed;
        double[] p = new double[2 * theCount];
        for (int k = 0; k < p.Length; k++)
        {
            s = (s * 1103515245 + 12345) % 2147483648;
            p[k] = ((s >> 8) % 2001 - 1000) / 1024.0;
        }
        return p;
    }

    private static string Fingerprint(Result r)
    {
        string aText = string.Join(";", r.Triangles.Select(t => $"{t.A} {t.B} {t.C}"));
        return Convert.ToHexStringLower(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.ASCII.GetBytes(aText)))[..16];
    }

    // Recorded from OCCT's delabella.cpp built with MSVC 14.44 (cl /O2 /fp:precise, x64) and a driver printing the
    // return value, the Delaunay triangles' v[0..2]->i in list order and the hull vertices: the fingerprint is the
    // first 16 hex digits of the SHA-256 of "a b c;a b c;..." over the triangles.
    [Theory]
    [InlineData("random40", 207, "d5a89043f052b774", "2 30 9 37 6 22 21 31 10")]
    [InlineData("random200", 1143, "b42a28f9a5833851", "196 121 49 175 65 25 138 81 173 5 3 37 142 123 125 176 158")]
    [InlineData("random2000", 11931, "7435b226be3fd920", "1011 1448 1148 1466 324 151 1126 1712 1496 1460 190 1245 534 1317 1804 1257 1012 435 960")]
    [InlineData("grid5", 96, "796a54c98ebcec6b", "5 10 15 20 21 22 23 24 19 14 9 4 3 2 1 0")]
    [InlineData("dups", 18, "df268d1950e0e515", "0 1 3 2")]
    [InlineData("colinear", -5, "e3b0c44298fc1c14", "3 2 0 4 1")]
    public void Output_matches_native_MSVC_delabella(string theCase, int theVerts, string theFingerprint, string theHull)
    {
        double[] p = theCase switch
        {
            "random40" => LcgPoints(1, 40),
            "random200" => LcgPoints(2, 200),
            "random2000" => LcgPoints(3, 2000),
            "grid5" => [.. Enumerable.Range(0, 25).SelectMany(k => new[] { k / 5 / 4.0 - 0.5, k % 5 / 4.0 - 0.5 })],
            "dups" => [0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0.5, 0.5, 1, 1, 0.25, 0.75],
            _ => [0, 0, 3, 3, 1, 1, 2, 2, -1, -1],
        };

        Result r = Run(p);
        Assert.Equal(theVerts, r.Verts);
        Assert.Equal(theFingerprint, Fingerprint(r));
        Assert.Equal(theHull, string.Join(" ", r.Hull));
    }

    [Fact]
    public void A_null_y_reads_y_after_x_and_a_short_advance_is_two_doubles()
    {
        double[] p = RandomPoints(21, 40);
        Result r = Run(p);

        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aDb = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        // C++'s null y: y = x + 1; an advance below 2 * sizeof(double) is raised to it (so separate x and y arrays
        // with a stride of one double can't be passed, as in C++)
        Assert.Equal(r.Verts, aDb.Triangulate(40, p));
        Assert.Equal(r.Verts, aDb.Triangulate(40, p, default, sizeof(double)));
        List<(int, int, int)> aTris = [];
        for (global::Kerf.Engine.Meshing.Delaunay.DelaunayTriangle? t = aDb.GetFirstDelaunayTriangle(); t is not null; t = t.next)
        {
            aTris.Add((t.v[0]!.i, t.v[1]!.i, t.v[2]!.i));
        }
        Assert.Equal(r.Triangles, aTris);

        // C++'s null x: no output
        Assert.Equal(0, aDb.Triangulate(40, ReadOnlySpan<double>.Empty));
        aDb.Destroy();
    }

    [Fact]
    public void No_points_fail_as_in_OCCT_ReproducesOcctError()
    {
        // OCCT: with 0 points the duplicate removal reports "-1 dups" and the single-point branch writes into an
        // empty allocation (a null pointer on a first call). The port throws there.
        global::Kerf.Engine.Meshing.Delaunay.IDelaBella aDb = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        List<string> aLog = [];
        aDb.SetErrLog((s, m) => { aLog.Add(m); return 0; }, null);
        Assert.ThrowsAny<NullReferenceException>(() => aDb.Triangulate(0, new double[2]));
        Assert.Contains("[WRN] detected -1 dups in xy array!\n", aLog);
    }
}
