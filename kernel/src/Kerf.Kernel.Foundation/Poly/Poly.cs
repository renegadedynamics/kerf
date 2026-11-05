// Port of: src/FoundationClasses/TKMath/Poly/Poly.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Services for triangulations and polygons: joining triangulations, writing and reading them in OCCT's text format,
/// normals, point location on a triangle, ray intersection and polygon properties.
/// </summary>
/// <remarks>
/// OCCT's streams become <see cref="TextWriter"/> and <see cref="TextReader"/>. Numbers are written as a C++ stream
/// writes them by default (six significant digits, <c>%g</c>), so files match OCCT's byte for byte; reading follows
/// C++'s extraction, where a failed read gives zero and makes the following reads fail too.
/// </remarks>
internal static class Poly
{
    /// <summary>
    /// Joins several triangulations into a new one: a plain sum, without sharing nodes. UV nodes are dropped. Null
    /// entries are skipped; returns null if there are no nodes.
    /// </summary>
    public static Triangulation? Catenate(NCollection_List<Triangulation?> lstTri)
    {
        int nNodes = 0;
        int nTrian = 0;

        // Sum up the total number of nodes.
        var iter = new NCollection_List<Triangulation?>.Iterator(lstTri);
        for (; iter.More(); iter.Next())
        {
            Triangulation? tri = iter.Value();
            if (tri is not null)
            {
                nNodes += tri.NodeCount;
                nTrian += tri.TriangleCount;
            }
        }

        if (nNodes == 0)
        {
            return null;
        }

        var result = new Triangulation(nNodes, nTrian, false);
        Span<int> iNode = [0, 0, 0];
        nNodes = 0;
        nTrian = 0;
        for (iter.Initialize(lstTri); iter.More(); iter.Next())
        {
            Triangulation? tri = iter.Value();
            if (tri is null)
            {
                continue;
            }

            int nbNodes = tri.NodeCount;
            int nbTrian = tri.TriangleCount;
            for (int i = 1; i <= nbNodes; i++)
            {
                result.SetNode(i + nNodes, tri.Node(i));
            }

            for (int i = 1; i <= nbTrian; i++)
            {
                tri.Triangle(i).Get(out iNode[0], out iNode[1], out iNode[2]);
                result.SetTriangle(i + nTrian, new Triangle1(iNode[0] + nNodes, iNode[1] + nNodes, iNode[2] + nNodes));
            }

            nNodes += nbNodes;
            nTrian += nbTrian;
        }

        return result;
    }

    /// <summary>
    /// Writes a triangulation: compact ("save" format, read back by <see cref="ReadTriangulation"/>) or not (an
    /// informative dump).
    /// </summary>
    public static void Write(Triangulation t, TextWriter os, bool compact = true)
    {
        os.Write("Poly_Triangulation\n");
        if (compact)
        {
            os.Write(Int(t.NodeCount) + " " + Int(t.TriangleCount) + " ");
            os.Write((t.HasUVNodes ? "1" : "0") + "\n");
        }
        else
        {
            os.Write(Setw(8, Int(t.NodeCount)) + " Nodes\n");
            os.Write(Setw(8, Int(t.TriangleCount)) + " Triangles\n");
            os.Write((t.HasUVNodes ? "with" : "without") + " UV nodes\n");
        }

        // write the deflection

        if (!compact)
        {
            os.Write("Deflection : ");
        }

        os.Write(Real(t.Deflection()) + "\n");

        // write the 3d nodes

        if (!compact)
        {
            os.Write("\n3D Nodes :\n");
        }

        int i, nbNodes = t.NodeCount;
        for (i = 1; i <= nbNodes; i++)
        {
            Point3D node = t.Node(i);
            if (!compact)
            {
                os.Write(Setw(10, Int(i)) + " : ");
            }

            os.Write(Field(compact, 17, Real(node.X)) + " ");
            os.Write(Field(compact, 17, Real(node.Y)) + " ");
            os.Write(Field(compact, 17, Real(node.Z)) + "\n");
        }

        if (t.HasUVNodes)
        {
            if (!compact)
            {
                os.Write("\nUV Nodes :\n");
            }

            for (i = 1; i <= nbNodes; i++)
            {
                Point2D node2d = t.UVNode(i);
                if (!compact)
                {
                    os.Write(Setw(10, Int(i)) + " : ");
                }

                os.Write(Field(compact, 17, Real(node2d.X)) + " ");
                os.Write(Field(compact, 17, Real(node2d.Y)) + "\n");
            }
        }

        if (!compact)
        {
            os.Write("\nTriangles :\n");
        }

        int nbTriangles = t.TriangleCount;
        for (i = 1; i <= nbTriangles; i++)
        {
            if (!compact)
            {
                os.Write(Setw(10, Int(i)) + " : ");
            }

            t.Triangle(i).Get(out int n1, out int n2, out int n3);
            os.Write(Field(compact, 10, Int(n1)) + " ");
            os.Write(Field(compact, 10, Int(n2)) + " ");
            os.Write(Field(compact, 10, Int(n3)) + "\n");
        }
    }

    /// <summary>
    /// Writes a 3D polygon: compact ("save" format, read back by <see cref="ReadPolygon3D"/>) or not (an informative
    /// dump).
    /// </summary>
    public static void Write(Polygon3D p, TextWriter os, bool compact = true)
    {
        os.Write("Poly_Polygon3D\n");
        if (compact)
        {
            os.Write(Int(p.NodeCount) + " ");
            os.Write((p.HasParameters ? "1" : "0") + "\n");
        }
        else
        {
            os.Write(Setw(8, Int(p.NodeCount)) + " Nodes\n");
            os.Write((p.HasParameters ? "with" : "without") + " parameters\n");
        }

        // write the deflection

        if (!compact)
        {
            os.Write("Deflection : ");
        }

        os.Write(Real(p.Deflection()) + "\n");

        // write the nodes

        if (!compact)
        {
            os.Write("\nNodes :\n");
        }

        int i, nbNodes = p.NodeCount;
        NCollection_Array1<Point3D> nodes = p.Nodes();
        for (i = 1; i <= nbNodes; i++)
        {
            if (!compact)
            {
                os.Write(Setw(10, Int(i)) + " : ");
            }

            os.Write(Field(compact, 17, Real(nodes[i].X)) + " ");
            os.Write(Field(compact, 17, Real(nodes[i].Y)) + " ");
            os.Write(Field(compact, 17, Real(nodes[i].Z)) + "\n");
        }

        if (p.HasParameters)
        {
            if (!compact)
            {
                os.Write("\nParameters :\n");
            }

            NCollection_Array1<double> param = p.Parameters();
            for (i = 1; i <= nbNodes; i++)
            {
                os.Write(Real(param[i]) + " ");
            }

            os.Write("\n");
        }
    }

    /// <summary>
    /// Writes a 2D polygon: compact ("save" format, read back by <see cref="ReadPolygon2D"/>) or not (an informative
    /// dump).
    /// </summary>
    public static void Write(Polygon2D p, TextWriter os, bool compact = true)
    {
        os.Write("Poly_Polygon2D\n");
        if (compact)
        {
            os.Write(Int(p.NodeCount) + " ");
        }
        else
        {
            os.Write(Setw(8, Int(p.NodeCount)) + " Nodes\n");
        }

        // write the deflection

        if (!compact)
        {
            os.Write("Deflection : ");
        }

        os.Write(Real(p.Deflection()) + "\n");

        // write the nodes

        if (!compact)
        {
            os.Write("\nNodes :\n");
        }

        int i, nbNodes = p.NodeCount;
        NCollection_Array1<Point2D> nodes = p.Nodes();
        for (i = 1; i <= nbNodes; i++)
        {
            if (!compact)
            {
                os.Write(Setw(10, Int(i)) + " : ");
            }

            os.Write(Field(compact, 17, Real(nodes[i].X)) + " ");
            os.Write(Field(compact, 17, Real(nodes[i].Y)) + "\n");
        }
    }

    /// <summary>Dumps a triangulation (<see cref="Write(Triangulation, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Triangulation t, TextWriter os) => Write(t, os, false);

    /// <summary>Dumps a 3D polygon (<see cref="Write(Polygon3D, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Polygon3D p, TextWriter os) => Write(p, os, false);

    /// <summary>Dumps a 2D polygon (<see cref="Write(Polygon2D, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Polygon2D p, TextWriter os) => Write(p, os, false);

    /// <summary>Reads a triangulation written in the compact format, or returns null if the text isn't one.</summary>
    public static Triangulation? ReadTriangulation(TextReader IS)
    {
        // Read a triangulation

        var stream = new Extractor(IS);
        string line = stream.Word();
        if (line != "Poly_Triangulation")
        {
            return null;
        }

        int nbNodes = stream.Int();
        int nbTriangles = stream.Int();
        bool hasUV = stream.Bool();

        double d = stream.Real();

        // read the 3d nodes
        double x, y, z;
        int i;
        var nodes = new NCollection_Array1<Point3D>(1, nbNodes);
        var uvNodes = new NCollection_Array1<Point2D>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = stream.Real();
            y = stream.Real();
            z = stream.Real();
            nodes[i] = nodes[i].WithCoord(x, y, z);
        }

        // read the UV points if necessary

        if (hasUV)
        {
            for (i = 1; i <= nbNodes; i++)
            {
                x = stream.Real();
                y = stream.Real();
                uvNodes[i] = uvNodes[i].WithCoord(x, y);
            }
        }

        // read the triangles
        int n1, n2, n3;
        var triangles = new NCollection_Array1<Triangle1>(1, nbTriangles);
        for (i = 1; i <= nbTriangles; i++)
        {
            n1 = stream.Int();
            n2 = stream.Int();
            n3 = stream.Int();
            triangles[i].Set(n1, n2, n3);
        }

        Triangulation t;

        if (hasUV)
        {
            t = new Triangulation(nodes, uvNodes, triangles);
        }
        else
        {
            t = new Triangulation(nodes, triangles);
        }

        t.Deflection(d);

        return t;
    }

    /// <summary>Reads a 3D polygon written in the compact format, or returns null if the text isn't one.</summary>
    public static Polygon3D? ReadPolygon3D(TextReader IS)
    {
        // Read a 3d polygon

        var stream = new Extractor(IS);
        string line = stream.Word();
        if (line != "Poly_Polygon3D")
        {
            return null;
        }

        int nbNodes = stream.Int();

        bool hasparameters = stream.Bool();

        double d = stream.Real();

        // read the nodes
        double x, y, z;
        int i;
        var nodes = new NCollection_Array1<Point3D>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = stream.Real();
            y = stream.Real();
            z = stream.Real();
            nodes[i] = nodes[i].WithCoord(x, y, z);
        }

        var param = new NCollection_Array1<double>(1, nbNodes);
        if (hasparameters)
        {
            for (i = 1; i <= nbNodes; i++)
            {
                param[i] = stream.Real();
            }
        }

        Polygon3D p;
        if (!hasparameters)
        {
            p = new Polygon3D(nodes);
        }
        else
        {
            p = new Polygon3D(nodes, param);
        }

        p.Deflection(d);

        return p;
    }

    /// <summary>Reads a 2D polygon written in the compact format, or returns null if the text isn't one.</summary>
    public static Polygon2D? ReadPolygon2D(TextReader IS)
    {
        // Read a 2d polygon

        var stream = new Extractor(IS);
        string line = stream.Word();
        if (line != "Poly_Polygon2D")
        {
            return null;
        }

        int nbNodes = stream.Int();

        double d = stream.Real();

        // read the nodes
        double x, y;
        int i;
        var nodes = new NCollection_Array1<Point2D>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = stream.Real();
            y = stream.Real();
            nodes[i] = nodes[i].WithCoord(x, y);
        }

        var p = new Polygon2D(nodes);

        p.Deflection(d);

        return p;
    }

    /// <summary>Computes the normals at the nodes as the mean of the normals of the triangles around them.</summary>
    public static void ComputeNormals(Triangulation tri) => tri.ComputeNormals();

    /// <summary>
    /// The parameters (U, V) of point <paramref name="p"/> on the triangle (<paramref name="p1"/>,
    /// <paramref name="p2"/>, <paramref name="p3"/>), with P = P1 + U (P2 - P1) + V (P3 - P1), U ≥ 0, V ≥ 0,
    /// U + V ≤ 1. Returns 0 if the point is inside; otherwise, or for a degenerate triangle, the parameters of the
    /// closest point and the square of the distance to it.
    /// </summary>
    public static double PointOnTriangle(in Xy p1, in Xy p2, in Xy p3, in Xy p, out Xy uv)
    {
        Xy dp = p - p1;
        Xy du = p2 - p1;
        Xy dv = p3 - p1;
        double det = du ^ dv;

        // case of non-degenerated triangle
        if (Math.Abs(det) > Constants.Resolution())
        {
            double aU = (dp ^ dv) / det;
            double aV = -(dp ^ du) / det;

            // if point is inside triangle, just return parameters
            if (aU > -Constants.Resolution() && aV > -Constants.Resolution() && 1.0 - aU - aV > -Constants.Resolution())
            {
                uv = new Xy(aU, aV);
                return 0.0;
            }

            // else find closest point on triangle sides; note that in general case
            // triangle can be very distorted and it is necessary to check
            // projection on all sides regardless of values of computed parameters

            // project on side U=0
            aU = 0.0;
            aV = Min(1.0, Max(0.0, (dp * dv) / dv.SquareModulus()));
            double aD = (aV * dv - dp).SquareModulus();

            // project on side V=0
            double u = Min(1.0, Max(0.0, (dp * du) / du.SquareModulus()));
            double d = (u * du - dp).SquareModulus();
            if (d < aD)
            {
                aU = u;
                aV = 0.0;
                aD = d;
            }

            // project on side U+V=1
            Xy duv = dv - du;
            double v = Min(1.0, Max(0.0, ((dp - du) * duv) / duv.SquareModulus()));
            d = (p2 + v * duv - p).SquareModulus();
            if (d < aD)
            {
                aU = 1.0 - v;
                aV = v;
                aD = d;
            }

            uv = new Xy(aU, aV);
            return aD;
        }

        // degenerated triangle
        double l2U = du.SquareModulus();
        double l2V = dv.SquareModulus();
        if (l2U < Constants.Resolution()) // side 1-2 is degenerated
        {
            if (l2V < Constants.Resolution()) // whole triangle is degenerated to point
            {
                uv = new Xy(0.0, 0.0);
                return (p - p1).SquareModulus();
            }
            else
            {
                uv = new Xy(0.0, (dp * dv) / l2V);
                return (p - (p1 + uv.Y() * dv)).SquareModulus();
            }
        }
        else if (l2V < Constants.Resolution()) // side 1-3 is degenerated
        {
            uv = new Xy((dp * du) / l2U, 0.0);
            return (p - (p1 + uv.X() * du)).SquareModulus();
        }
        else // sides 1-2 and 1-3 are collinear
        {
            // select parameter on one of sides so as to have points closer to picked
            double aU = Min(1.0, Max(0.0, (dp * du) / l2U));
            double aV = Min(1.0, Max(0.0, (dp * dv) / l2V));
            double d1 = (dp - aU * du).SquareModulus();
            double d2 = (dp - aV * dv).SquareModulus();
            if (d1 < d2)
            {
                uv = new Xy((dp * du) / l2U, 0.0);
                return d1;
            }
            else
            {
                uv = new Xy(0.0, (dp * dv) / l2V);
                return d2;
            }
        }
    }

    /// <summary>
    /// The intersection of a ray with a triangulation: the closest (or the farthest) intersected triangle and the
    /// distance to it along the ray. False if the ray misses; <paramref name="triangle"/> and
    /// <paramref name="distance"/> are then unchanged.
    /// </summary>
    public static bool Intersect(Triangulation tri, in Axis3D axis, bool isClosest, ref Triangle1 triangle, ref double distance)
    {
        const double conf = 1E-15;
        Xyz loc = axis.Location.XYZ();
        Direction3D dir = axis.Direction;

        double result = isClosest ? RealLast() : 0.0;
        double param = 0.0;
        Span<int> triNodes = [0, 0, 0];
        for (int triIter = 1; triIter <= tri.TriangleCount; ++triIter)
        {
            Triangle1 aTri = tri.Triangle(triIter);
            aTri.Get(out triNodes[0], out triNodes[1], out triNodes[2]);
            if (IntersectTriLine(loc,
                                 dir,
                                 tri.Node(triNodes[0]).XYZ(),
                                 tri.Node(triNodes[1]).XYZ(),
                                 tri.Node(triNodes[2]).XYZ(),
                                 ref param) != 0)
            {
                if (param > conf)
                {
                    if (isClosest)
                    {
                        if (param < result)
                        {
                            result = param;
                            triangle = aTri;
                        }
                    }
                    else if (param > result)
                    {
                        result = param;
                        triangle = aTri;
                    }
                }
            }
        }

        if (conf < result && result < RealLast())
        {
            distance = result;
            return true;
        }

        return false;
    }

    // Calculate the minor of the given matrix, defined by the columns specified by values c1, c2, c3.
    private static double Determinant(ReadOnlySpan<double> a, int c1, int c2, int c3)
    {
        // a is the 3x4 matrix a[i][j] = a[4 * i + j]
        return a[0 * 4 + c1] * a[1 * 4 + c2] * a[2 * 4 + c3] + a[0 * 4 + c2] * a[1 * 4 + c3] * a[2 * 4 + c1]
               + a[0 * 4 + c3] * a[1 * 4 + c1] * a[2 * 4 + c2] - a[0 * 4 + c3] * a[1 * 4 + c2] * a[2 * 4 + c1]
               - a[0 * 4 + c2] * a[1 * 4 + c1] * a[2 * 4 + c3] - a[0 * 4 + c1] * a[1 * 4 + c3] * a[2 * 4 + c2];
    }

    /// <summary>
    /// The intersection of the line (<paramref name="start"/>, <paramref name="dir"/>) with the triangle
    /// (<paramref name="v0"/>, <paramref name="v1"/>, <paramref name="v2"/>): 1 and the parameter on the line
    /// if found; 0 otherwise, <paramref name="param"/> unchanged.
    /// </summary>
    public static int IntersectTriLine(in Xyz start, in Direction3D dir, in Xyz v0, in Xyz v1, in Xyz v2, ref double param)
    {
        int res = 0;
        const double conf = 1E-15;

        ReadOnlySpan<double> mat34 =
        [
            -dir.X, v1.X() - v0.X(), v2.X() - v0.X(), start.X() - v0.X(),
            -dir.Y, v1.Y() - v0.Y(), v2.Y() - v0.Y(), start.Y() - v0.Y(),
            -dir.Z, v1.Z() - v0.Z(), v2.Z() - v0.Z(), start.Z() - v0.Z(),
        ];

        double d = Determinant(mat34, 0, 1, 2);
        double dt = Determinant(mat34, 3, 1, 2);
        if (d > conf)
        {
            double da = Determinant(mat34, 0, 3, 2);
            if (da > -conf)
            {
                double db = Determinant(mat34, 0, 1, 3);
                res = ((db > -conf) && (da + db <= d + conf)) ? 1 : 0;
            }
        }
        else if (d < -conf)
        {
            double da = Determinant(mat34, 0, 3, 2);
            if (da < conf)
            {
                double db = Determinant(mat34, 0, 1, 3);
                res = ((db < conf) && (da + db >= d - conf)) ? 1 : 0;
            }
        }

        if (res != 0)
        {
            param = dt / d;
        }

        return res;
    }

    /// <summary>
    /// The area and perimeter of a 2D polygon given by its vertices (not repeating the first). The area is negative if
    /// the polygon runs clockwise; the perimeter is always positive.
    /// </summary>
    public static bool PolygonProperties(NCollection_Array1<Point2D> seqPnts, out double area, out double perimeter)
    {
        if (seqPnts.Length() < 2)
        {
            area = perimeter = 0.0;
            return true;
        }

        int startIndex = seqPnts.Lower();
        Xy refPnt = seqPnts.Value(startIndex++).XY();
        Xy prevPt = seqPnts.Value(startIndex++).XY() - refPnt, currPt;
        area = 0.0;
        perimeter = prevPt.Modulus();
        for (int i = startIndex; i <= seqPnts.Upper(); i++)
        {
            currPt = seqPnts.Value(i).XY() - refPnt;
            double delta = prevPt.Crossed(currPt);
            area += delta;
            perimeter += (prevPt - currPt).Modulus();
            prevPt = currPt;
        }

        perimeter += prevPt.Modulus();
        area *= 0.5;
        return true;
    }

    /// <summary>
    /// The area and perimeter of a 2D polygon given by its vertices (not repeating the first). The area is negative if
    /// the polygon runs clockwise; the perimeter is always positive.
    /// </summary>
    public static bool PolygonProperties(NCollection_Sequence<Point2D> seqPnts, out double area, out double perimeter)
    {
        if (seqPnts.Length() < 2)
        {
            area = perimeter = 0.0;
            return true;
        }

        int startIndex = NCollection_Sequence<Point2D>.Lower();
        Xy refPnt = seqPnts.Value(startIndex++).XY();
        Xy prevPt = seqPnts.Value(startIndex++).XY() - refPnt, currPt;
        area = 0.0;
        perimeter = prevPt.Modulus();
        for (int i = startIndex; i <= seqPnts.Upper(); i++)
        {
            currPt = seqPnts.Value(i).XY() - refPnt;
            double delta = prevPt.Crossed(currPt);
            area += delta;
            perimeter += (prevPt - currPt).Modulus();
            prevPt = currPt;
        }

        perimeter += prevPt.Modulus();
        area *= 0.5;
        return true;
    }

    // C++ stream formatting: OS << int, OS << double (six significant digits, as printf's %g), std::setw

    private static string Int(int value) => value.ToString(CultureInfo.InvariantCulture);

    // %g with six digits, as Microsoft's C runtime writes it (ties rounded away from zero; see Standard_Stream)
    internal static string Real(double value) => StreamUtilities.FormatReal(value, 6);

    private static string Setw(int width, string text) => text.PadLeft(width);

    private static string Field(bool compact, int width, string text) => compact ? text : text.PadLeft(width);

    // C++ stream extraction (operator>>) on whitespace-separated words. A failed extraction gives zero and puts the
    // stream in a failed state, in which every later extraction fails too.
    private sealed class Extractor(TextReader theReader)
    {
        private bool _failed;

        public string Word()
        {
            if (_failed)
            {
                return "";
            }

            int aChar = theReader.Peek();
            while (aChar >= 0 && char.IsWhiteSpace((char)aChar))
            {
                theReader.Read();
                aChar = theReader.Peek();
            }

            if (aChar < 0)
            {
                _failed = true;
                return "";
            }

            var builder = new System.Text.StringBuilder();
            while (aChar >= 0 && !char.IsWhiteSpace((char)aChar))
            {
                builder.Append((char)theReader.Read());
                aChar = theReader.Peek();
            }

            return builder.ToString();
        }

        public int Int()
        {
            string word = Word();
            if (!_failed && int.TryParse(word, NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out int value))
            {
                return value;
            }

            _failed = true;
            return 0;
        }

        public bool Bool()
        {
            // a bool is extracted as an integer, 0 or 1; any other integer gives true and fails the stream
            int value = Int();
            if (_failed || value is 0 or 1)
            {
                return value == 1;
            }

            _failed = true;
            return true;
        }

        public double Real()
        {
            string word = Word();
            if (!_failed && double.TryParse(word, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
            {
                return value;
            }

            _failed = true;
            return 0.0;
        }
    }
}
