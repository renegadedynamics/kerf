// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

namespace OcctNet.Foundation.Tests;

/// <summary>
/// <see cref="Heap"/> against orders recorded from MSVC (decision 491): BRepMesh_Delaun sorts its vertices with
/// <c>std::make_heap</c> + <c>std::sort_heap</c> on keys with many ties, and the oracle's OCCT is built with MSVC.
/// </summary>
/// <remarks>
/// The orders were recorded with Visual Studio 2022 Build Tools (MSVC 14.44.35207, <c>cl /O2 /EHsc /std:c++17</c>,
/// x64) running this program, which sorts the indices 0..n-1 by their keys (ties everywhere) and prints the
/// permutation after <c>make_heap</c> and after <c>sort_heap</c>:
/// <code>
/// // Records MSVC's std::make_heap + std::sort_heap order on keys with ties (OCCT.NET unit 4B, decision 491).
/// #include &lt;algorithm&gt;
/// #include &lt;cstdio&gt;
/// #include &lt;vector&gt;
/// 
/// static unsigned int g_seed = 1;
/// static int lcg() { g_seed = g_seed * 214013u + 2531011u; return (g_seed &gt;&gt; 16) &amp; 0x7fff; }
/// 
/// static void run(const char* name, const std::vector&lt;double&gt;&amp; keys)
/// {
///   std::vector&lt;int&gt; idx(keys.size());
///   for (size_t i = 0; i &lt; keys.size(); ++i) idx[i] = (int)i;
///   auto cmp = [&amp;](int a, int b) { return keys[a] &lt; keys[b]; };
///   std::make_heap(idx.begin(), idx.end(), cmp);
///   std::printf("%s heap:", name);
///   for (int v : idx) std::printf(" %d", v);
///   std::printf("\n");
///   std::sort_heap(idx.begin(), idx.end(), cmp);
///   std::printf("%s sorted:", name);
///   for (int v : idx) std::printf(" %d", v);
///   std::printf("\n");
/// }
/// 
/// int main()
/// {
///   run("allequal7", std::vector&lt;double&gt;(7, 1.0));
///   run("allequal8", std::vector&lt;double&gt;(8, 1.0));
///   { std::vector&lt;double&gt; k; for (int i = 0; i &lt; 20; ++i) k.push_back((double)(i % 3)); run("mod3_20", k); }
///   { std::vector&lt;double&gt; k; for (int i = 0; i &lt; 33; ++i) k.push_back((double)((i * 7) % 5)); run("mod5_33", k); }
///   { // grid 6x6 keyed by x + y as Delaun's comparator, nodes row by row
///     std::vector&lt;double&gt; k; for (int j = 0; j &lt; 6; ++j) for (int i = 0; i &lt; 6; ++i) k.push_back(0.1 * i + 0.1 * j); run("grid6_xplusy", k); }
///   { g_seed = 42; std::vector&lt;double&gt; k; for (int i = 0; i &lt; 64; ++i) k.push_back((double)(lcg() % 10)); run("lcg64_mod10", k); }
///   { g_seed = 7; std::vector&lt;double&gt; k; for (int i = 0; i &lt; 257; ++i) k.push_back((double)(lcg() % 17)); run("lcg257_mod17", k); }
///   run("single", std::vector&lt;double&gt;(1, 3.0));
///   run("two", std::vector&lt;double&gt;{2.0, 2.0});
///   return 0;
/// }
/// </code>
/// </remarks>
public class std_heapTests
{
    private readonly struct KeyLess(double[] theKeys) : Heap.ILess<int>
    {
        public bool Less(int theLeft, int theRight) => theKeys[theLeft] < theKeys[theRight];
    }

    // C's rand() as MSVC's CRT implements it (srand(seed); rand()).
    private sealed class MsvcRand(uint theSeed)
    {
        private uint mySeed = theSeed;

        public int Next()
        {
            mySeed = unchecked(mySeed * 214013u + 2531011u);
            return (int)((mySeed >> 16) & 0x7fff);
        }
    }

    private static double[] Keys(string theName)
    {
        List<double> aKeys = [];
        switch (theName)
        {
            case "allequal7":
                aKeys.AddRange(Enumerable.Repeat(1.0, 7));
                break;
            case "allequal8":
                aKeys.AddRange(Enumerable.Repeat(1.0, 8));
                break;
            case "mod3_20":
                for (int i = 0; i < 20; ++i)
                {
                    aKeys.Add(i % 3);
                }

                break;
            case "mod5_33":
                for (int i = 0; i < 33; ++i)
                {
                    aKeys.Add((i * 7) % 5);
                }

                break;
            case "grid6_xplusy":
                for (int j = 0; j < 6; ++j)
                {
                    for (int i = 0; i < 6; ++i)
                    {
                        aKeys.Add(0.1 * i + 0.1 * j);
                    }
                }

                break;
            case "lcg64_mod10":
            {
                MsvcRand aRand = new(42);
                for (int i = 0; i < 64; ++i)
                {
                    aKeys.Add(aRand.Next() % 10);
                }

                break;
            }
            case "lcg257_mod17":
            {
                MsvcRand aRand = new(7);
                for (int i = 0; i < 257; ++i)
                {
                    aKeys.Add(aRand.Next() % 17);
                }

                break;
            }
            case "single":
                aKeys.Add(3.0);
                break;
            case "two":
                aKeys.AddRange([2.0, 2.0]);
                break;
            default:
                throw new ArgumentException(theName);
        }

        return [.. aKeys];
    }

    public static TheoryData<string, string, string> RecordedOrders => new()
    {
        { "allequal7", "6 4 2 3 1 5 0", "3 4 1 0 5 2 6" },
        { "allequal8", "6 4 2 7 1 5 0 3", "7 4 1 3 5 0 2 6" },
        { "mod3_20", "14 17 2 8 10 11 13 16 1 19 4 5 12 0 6 15 7 3 18 9", "9 15 3 18 12 0 6 16 7 1 19 4 10 13 8 17 5 11 2 14" },
        { "mod5_33", "27 22 2 17 4 12 29 32 18 19 21 24 26 6 14 7 16 8 3 9 20 1 10 23 11 25 5 13 28 0 30 31 15", "30 10 0 20 15 25 5 28 18 13 8 3 23 16 1 21 11 26 6 31 9 19 4 24 14 29 7 32 17 22 12 2 27" },
        { "grid6_xplusy", "35 34 29 33 22 23 28 32 17 20 21 11 26 27 30 31 16 8 18 19 9 4 10 5 24 25 12 2 13 14 6 0 15 7 1 3", "0 1 6 2 12 7 18 8 3 13 24 14 9 4 19 20 10 5 25 15 30 31 11 16 21 26 32 17 22 27 33 23 28 34 29 35" },
        { "lcg64_mod10", "59 45 13 36 10 52 2 16 8 40 43 47 5 58 62 32 34 35 38 9 20 44 22 23 49 51 54 56 28 60 6 31 15 33 7 3 17 37 18 39 19 41 42 21 1 4 46 11 48 24 50 12 25 53 26 55 27 57 0 29 14 61 30 63", "57 25 1 53 21 24 11 44 27 46 63 18 12 30 17 55 37 50 42 4 51 41 7 15 33 39 29 48 56 0 28 61 34 3 19 22 49 14 26 60 6 31 35 38 20 23 54 58 32 16 43 62 8 36 9 40 10 45 47 5 52 2 13 59" },
        { "lcg257_mod17", "237 44 118 145 170 197 228 68 77 84 189 49 106 226 252 131 138 150 151 163 168 180 22 192 205 213 109 230 236 240 250 31 132 7 141 35 36 152 156 162 40 1 172 43 182 186 187 193 48 201 102 207 211 53 54 223 0 233 235 29 243 249 62 255 129 15 66 67 16 139 142 143 17 148 149 3 76 38 158 160 161 19 82 9 20 85 86 87 178 179 181 184 185 188 94 191 96 97 98 200 100 203 50 208 210 105 25 215 217 26 221 224 55 113 114 231 116 6 28 239 242 244 246 247 61 30 254 63 128 64 130 65 32 133 134 135 136 137 33 69 140 70 34 71 144 72 146 147 73 8 74 75 37 153 154 155 18 157 78 159 79 39 80 81 164 165 166 167 83 169 41 171 42 173 174 175 176 177 88 21 89 10 90 183 91 45 92 93 4 46 190 47 95 11 194 195 196 23 198 199 99 24 202 101 204 5 206 103 51 209 104 12 212 52 214 107 216 108 218 219 220 110 222 111 2 225 112 227 56 229 27 115 232 57 234 13 117 58 238 59 119 241 120 121 60 245 122 123 248 14 124 251 125 253 126 127 256", "42 51 159 153 78 2 164 219 214 123 8 75 34 4 202 12 58 74 126 157 253 194 169 60 196 120 115 209 227 204 232 37 45 72 183 92 11 13 24 256 3 154 103 88 166 175 133 18 212 190 96 173 251 124 234 220 95 110 91 108 122 229 32 216 144 199 23 198 222 107 56 127 177 79 134 70 155 81 224 111 195 99 83 215 218 27 21 158 101 59 41 90 200 203 5 105 52 217 114 171 245 246 174 65 125 140 142 149 248 14 86 93 25 26 225 63 137 33 76 165 176 178 89 10 47 97 80 112 104 53 117 254 71 39 19 167 85 172 185 191 206 208 210 221 55 231 247 61 130 66 69 57 147 73 161 82 121 87 184 98 54 113 119 15 135 136 139 160 181 186 100 211 0 116 239 30 62 67 38 156 162 188 46 94 193 48 233 6 238 244 16 143 146 152 40 9 187 192 223 230 243 249 128 7 141 148 163 20 43 179 201 50 102 205 109 28 241 242 250 255 64 1 182 22 49 207 213 129 132 138 17 35 36 150 151 77 168 84 180 189 106 235 236 29 240 252 31 131 68 145 170 44 197 226 228 118 237" },
        { "single", "0", "0" },
        { "two", "1 0", "0 1" },
    };

    [Theory]
    [MemberData(nameof(RecordedOrders))]
    public void make_heap_and_sort_heap_give_MSVCs_order_on_ties(string theName, string theHeap, string theSorted)
    {
        double[] aKeys = Keys(theName);
        int[] anIndices = [.. Enumerable.Range(0, aKeys.Length)];
        KeyLess aLess = new(aKeys);

        Heap.make_heap(anIndices.AsSpan(), aLess);
        Assert.Equal(theHeap, string.Join(' ', anIndices));

        Heap.sort_heap(anIndices.AsSpan(), aLess);
        Assert.Equal(theSorted, string.Join(' ', anIndices));
    }

    [Fact]
    public void Ties_are_not_in_Array_Sorts_order()
    {
        // The reason for the helper: .NET's introsort orders the tied keys differently.
        double[] aKeys = Keys("grid6_xplusy");
        int[] aHeap = [.. Enumerable.Range(0, aKeys.Length)];
        KeyLess aLess = new(aKeys);
        Heap.make_heap(aHeap.AsSpan(), aLess);
        Heap.sort_heap(aHeap.AsSpan(), aLess);

        int[] aSorted = [.. Enumerable.Range(0, aKeys.Length)];
        Array.Sort(aSorted, (a, b) => aKeys[a].CompareTo(aKeys[b]));

        Assert.NotEqual(string.Join(' ', aSorted), string.Join(' ', aHeap));
        for (int i = 1; i < aHeap.Length; ++i)
        {
            Assert.True(aKeys[aHeap[i - 1]] <= aKeys[aHeap[i]]);
        }
    }

    [Fact]
    public void push_heap_and_pop_heap_keep_a_heap()
    {
        MsvcRand aRand = new(3);
        double[] aKeys = new double[50];
        for (int i = 0; i < aKeys.Length; ++i)
        {
            aKeys[i] = aRand.Next() % 7;
        }

        KeyLess aLess = new(aKeys);
        int[] anIndices = [.. Enumerable.Range(0, aKeys.Length)];
        for (int n = 2; n <= anIndices.Length; ++n)
        {
            Heap.push_heap(anIndices.AsSpan(0, n), aLess);
        }

        for (int n = anIndices.Length; n >= 2; --n)
        {
            Heap.pop_heap(anIndices.AsSpan(0, n), aLess);
            for (int i = 0; i < n - 1; ++i)
            {
                Assert.False(aKeys[anIndices[n - 1]] < aKeys[anIndices[i]]);
            }
        }
    }
}
