// Port of: src/FoundationClasses/TKMath/Poly/Poly.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// Services for triangulations and polygons: joining triangulations, writing and reading them in OCCT's text format,
/// normals, point location on a triangle, ray intersection and polygon properties.
/// </summary>
/// <remarks>
/// OCCT's streams become <see cref="TextWriter"/> and <see cref="TextReader"/>. Numbers are written as a C++ stream
/// writes them by default (six significant digits, <c>%g</c>), so files match OCCT's byte for byte; reading follows
/// C++'s extraction, where a failed read gives zero and makes the following reads fail too.
/// </remarks>
public static class Poly
{
    /// <summary>
    /// Joins several triangulations into a new one: a plain sum, without sharing nodes. UV nodes are dropped. Null
    /// entries are skipped; returns null if there are no nodes.
    /// </summary>
    public static Poly_Triangulation? Catenate(NCollection_List<Poly_Triangulation?> lstTri)
    {
        int nNodes = 0;
        int nTrian = 0;

        // Sum up the total number of nodes.
        var anIter = new NCollection_List<Poly_Triangulation?>.Iterator(lstTri);
        for (; anIter.More(); anIter.Next())
        {
            Poly_Triangulation? aTri = anIter.Value();
            if (aTri is not null)
            {
                nNodes += aTri.NbNodes();
                nTrian += aTri.NbTriangles();
            }
        }

        if (nNodes == 0)
        {
            return null;
        }

        var aResult = new Poly_Triangulation(nNodes, nTrian, false);
        Span<int> iNode = [0, 0, 0];
        nNodes = 0;
        nTrian = 0;
        for (anIter.Initialize(lstTri); anIter.More(); anIter.Next())
        {
            Poly_Triangulation? aTri = anIter.Value();
            if (aTri is null)
            {
                continue;
            }

            int nbNodes = aTri.NbNodes();
            int nbTrian = aTri.NbTriangles();
            for (int i = 1; i <= nbNodes; i++)
            {
                aResult.SetNode(i + nNodes, aTri.Node(i));
            }

            for (int i = 1; i <= nbTrian; i++)
            {
                aTri.Triangle(i).Get(out iNode[0], out iNode[1], out iNode[2]);
                aResult.SetTriangle(i + nTrian, new Poly_Triangle(iNode[0] + nNodes, iNode[1] + nNodes, iNode[2] + nNodes));
            }

            nNodes += nbNodes;
            nTrian += nbTrian;
        }

        return aResult;
    }

    /// <summary>
    /// Writes a triangulation: compact ("save" format, read back by <see cref="ReadTriangulation"/>) or not (an
    /// informative dump).
    /// </summary>
    public static void Write(Poly_Triangulation T, TextWriter OS, bool Compact = true)
    {
        OS.Write("Poly_Triangulation\n");
        if (Compact)
        {
            OS.Write(Int(T.NbNodes()) + " " + Int(T.NbTriangles()) + " ");
            OS.Write((T.HasUVNodes() ? "1" : "0") + "\n");
        }
        else
        {
            OS.Write(Setw(8, Int(T.NbNodes())) + " Nodes\n");
            OS.Write(Setw(8, Int(T.NbTriangles())) + " Triangles\n");
            OS.Write((T.HasUVNodes() ? "with" : "without") + " UV nodes\n");
        }

        // write the deflection

        if (!Compact)
        {
            OS.Write("Deflection : ");
        }

        OS.Write(Real(T.Deflection()) + "\n");

        // write the 3d nodes

        if (!Compact)
        {
            OS.Write("\n3D Nodes :\n");
        }

        int i, nbNodes = T.NbNodes();
        for (i = 1; i <= nbNodes; i++)
        {
            gp_Pnt aNode = T.Node(i);
            if (!Compact)
            {
                OS.Write(Setw(10, Int(i)) + " : ");
            }

            OS.Write(Field(Compact, 17, Real(aNode.X())) + " ");
            OS.Write(Field(Compact, 17, Real(aNode.Y())) + " ");
            OS.Write(Field(Compact, 17, Real(aNode.Z())) + "\n");
        }

        if (T.HasUVNodes())
        {
            if (!Compact)
            {
                OS.Write("\nUV Nodes :\n");
            }

            for (i = 1; i <= nbNodes; i++)
            {
                gp_Pnt2d aNode2d = T.UVNode(i);
                if (!Compact)
                {
                    OS.Write(Setw(10, Int(i)) + " : ");
                }

                OS.Write(Field(Compact, 17, Real(aNode2d.X())) + " ");
                OS.Write(Field(Compact, 17, Real(aNode2d.Y())) + "\n");
            }
        }

        if (!Compact)
        {
            OS.Write("\nTriangles :\n");
        }

        int nbTriangles = T.NbTriangles();
        for (i = 1; i <= nbTriangles; i++)
        {
            if (!Compact)
            {
                OS.Write(Setw(10, Int(i)) + " : ");
            }

            T.Triangle(i).Get(out int n1, out int n2, out int n3);
            OS.Write(Field(Compact, 10, Int(n1)) + " ");
            OS.Write(Field(Compact, 10, Int(n2)) + " ");
            OS.Write(Field(Compact, 10, Int(n3)) + "\n");
        }
    }

    /// <summary>
    /// Writes a 3D polygon: compact ("save" format, read back by <see cref="ReadPolygon3D"/>) or not (an informative
    /// dump).
    /// </summary>
    public static void Write(Poly_Polygon3D P, TextWriter OS, bool Compact = true)
    {
        OS.Write("Poly_Polygon3D\n");
        if (Compact)
        {
            OS.Write(Int(P.NbNodes()) + " ");
            OS.Write((P.HasParameters() ? "1" : "0") + "\n");
        }
        else
        {
            OS.Write(Setw(8, Int(P.NbNodes())) + " Nodes\n");
            OS.Write((P.HasParameters() ? "with" : "without") + " parameters\n");
        }

        // write the deflection

        if (!Compact)
        {
            OS.Write("Deflection : ");
        }

        OS.Write(Real(P.Deflection()) + "\n");

        // write the nodes

        if (!Compact)
        {
            OS.Write("\nNodes :\n");
        }

        int i, nbNodes = P.NbNodes();
        NCollection_Array1<gp_Pnt> Nodes = P.Nodes();
        for (i = 1; i <= nbNodes; i++)
        {
            if (!Compact)
            {
                OS.Write(Setw(10, Int(i)) + " : ");
            }

            OS.Write(Field(Compact, 17, Real(Nodes[i].X())) + " ");
            OS.Write(Field(Compact, 17, Real(Nodes[i].Y())) + " ");
            OS.Write(Field(Compact, 17, Real(Nodes[i].Z())) + "\n");
        }

        if (P.HasParameters())
        {
            if (!Compact)
            {
                OS.Write("\nParameters :\n");
            }

            NCollection_Array1<double> Param = P.Parameters();
            for (i = 1; i <= nbNodes; i++)
            {
                OS.Write(Real(Param[i]) + " ");
            }

            OS.Write("\n");
        }
    }

    /// <summary>
    /// Writes a 2D polygon: compact ("save" format, read back by <see cref="ReadPolygon2D"/>) or not (an informative
    /// dump).
    /// </summary>
    public static void Write(Poly_Polygon2D P, TextWriter OS, bool Compact = true)
    {
        OS.Write("Poly_Polygon2D\n");
        if (Compact)
        {
            OS.Write(Int(P.NbNodes()) + " ");
        }
        else
        {
            OS.Write(Setw(8, Int(P.NbNodes())) + " Nodes\n");
        }

        // write the deflection

        if (!Compact)
        {
            OS.Write("Deflection : ");
        }

        OS.Write(Real(P.Deflection()) + "\n");

        // write the nodes

        if (!Compact)
        {
            OS.Write("\nNodes :\n");
        }

        int i, nbNodes = P.NbNodes();
        NCollection_Array1<gp_Pnt2d> Nodes = P.Nodes();
        for (i = 1; i <= nbNodes; i++)
        {
            if (!Compact)
            {
                OS.Write(Setw(10, Int(i)) + " : ");
            }

            OS.Write(Field(Compact, 17, Real(Nodes[i].X())) + " ");
            OS.Write(Field(Compact, 17, Real(Nodes[i].Y())) + "\n");
        }
    }

    /// <summary>Dumps a triangulation (<see cref="Write(Poly_Triangulation, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Poly_Triangulation T, TextWriter OS) => Write(T, OS, false);

    /// <summary>Dumps a 3D polygon (<see cref="Write(Poly_Polygon3D, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Poly_Polygon3D P, TextWriter OS) => Write(P, OS, false);

    /// <summary>Dumps a 2D polygon (<see cref="Write(Poly_Polygon2D, TextWriter, bool)"/>, not compact).</summary>
    public static void Dump(Poly_Polygon2D P, TextWriter OS) => Write(P, OS, false);

    /// <summary>Reads a triangulation written in the compact format, or returns null if the text isn't one.</summary>
    public static Poly_Triangulation? ReadTriangulation(TextReader IS)
    {
        // Read a triangulation

        var aStream = new Extractor(IS);
        string line = aStream.Word();
        if (line != "Poly_Triangulation")
        {
            return null;
        }

        int nbNodes = aStream.Int();
        int nbTriangles = aStream.Int();
        bool hasUV = aStream.Bool();

        double d = aStream.Real();

        // read the 3d nodes
        double x, y, z;
        int i;
        var Nodes = new NCollection_Array1<gp_Pnt>(1, nbNodes);
        var UVNodes = new NCollection_Array1<gp_Pnt2d>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = aStream.Real();
            y = aStream.Real();
            z = aStream.Real();
            Nodes[i].SetCoord(x, y, z);
        }

        // read the UV points if necessary

        if (hasUV)
        {
            for (i = 1; i <= nbNodes; i++)
            {
                x = aStream.Real();
                y = aStream.Real();
                UVNodes[i].SetCoord(x, y);
            }
        }

        // read the triangles
        int n1, n2, n3;
        var Triangles = new NCollection_Array1<Poly_Triangle>(1, nbTriangles);
        for (i = 1; i <= nbTriangles; i++)
        {
            n1 = aStream.Int();
            n2 = aStream.Int();
            n3 = aStream.Int();
            Triangles[i].Set(n1, n2, n3);
        }

        Poly_Triangulation T;

        if (hasUV)
        {
            T = new Poly_Triangulation(Nodes, UVNodes, Triangles);
        }
        else
        {
            T = new Poly_Triangulation(Nodes, Triangles);
        }

        T.Deflection(d);

        return T;
    }

    /// <summary>Reads a 3D polygon written in the compact format, or returns null if the text isn't one.</summary>
    public static Poly_Polygon3D? ReadPolygon3D(TextReader IS)
    {
        // Read a 3d polygon

        var aStream = new Extractor(IS);
        string line = aStream.Word();
        if (line != "Poly_Polygon3D")
        {
            return null;
        }

        int nbNodes = aStream.Int();

        bool hasparameters = aStream.Bool();

        double d = aStream.Real();

        // read the nodes
        double x, y, z;
        int i;
        var Nodes = new NCollection_Array1<gp_Pnt>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = aStream.Real();
            y = aStream.Real();
            z = aStream.Real();
            Nodes[i].SetCoord(x, y, z);
        }

        var Param = new NCollection_Array1<double>(1, nbNodes);
        if (hasparameters)
        {
            for (i = 1; i <= nbNodes; i++)
            {
                Param[i] = aStream.Real();
            }
        }

        Poly_Polygon3D P;
        if (!hasparameters)
        {
            P = new Poly_Polygon3D(Nodes);
        }
        else
        {
            P = new Poly_Polygon3D(Nodes, Param);
        }

        P.Deflection(d);

        return P;
    }

    /// <summary>Reads a 2D polygon written in the compact format, or returns null if the text isn't one.</summary>
    public static Poly_Polygon2D? ReadPolygon2D(TextReader IS)
    {
        // Read a 2d polygon

        var aStream = new Extractor(IS);
        string line = aStream.Word();
        if (line != "Poly_Polygon2D")
        {
            return null;
        }

        int nbNodes = aStream.Int();

        double d = aStream.Real();

        // read the nodes
        double x, y;
        int i;
        var Nodes = new NCollection_Array1<gp_Pnt2d>(1, nbNodes);

        for (i = 1; i <= nbNodes; i++)
        {
            x = aStream.Real();
            y = aStream.Real();
            Nodes[i].SetCoord(x, y);
        }

        var P = new Poly_Polygon2D(Nodes);

        P.Deflection(d);

        return P;
    }

    /// <summary>Computes the normals at the nodes as the mean of the normals of the triangles around them.</summary>
    public static void ComputeNormals(Poly_Triangulation theTri) => theTri.ComputeNormals();

    /// <summary>
    /// The parameters (U, V) of point <paramref name="theP"/> on the triangle (<paramref name="theP1"/>,
    /// <paramref name="theP2"/>, <paramref name="theP3"/>), with P = P1 + U (P2 - P1) + V (P3 - P1), U ≥ 0, V ≥ 0,
    /// U + V ≤ 1. Returns 0 if the point is inside; otherwise, or for a degenerate triangle, the parameters of the
    /// closest point and the square of the distance to it.
    /// </summary>
    public static double PointOnTriangle(in gp_XY theP1, in gp_XY theP2, in gp_XY theP3, in gp_XY theP, out gp_XY theUV)
    {
        gp_XY aDP = theP - theP1;
        gp_XY aDU = theP2 - theP1;
        gp_XY aDV = theP3 - theP1;
        double aDet = aDU ^ aDV;

        // case of non-degenerated triangle
        if (Math.Abs(aDet) > gp.Resolution())
        {
            double aU = (aDP ^ aDV) / aDet;
            double aV = -(aDP ^ aDU) / aDet;

            // if point is inside triangle, just return parameters
            if (aU > -gp.Resolution() && aV > -gp.Resolution() && 1.0 - aU - aV > -gp.Resolution())
            {
                theUV = new gp_XY(aU, aV);
                return 0.0;
            }

            // else find closest point on triangle sides; note that in general case
            // triangle can be very distorted and it is necessary to check
            // projection on all sides regardless of values of computed parameters

            // project on side U=0
            aU = 0.0;
            aV = Min(1.0, Max(0.0, (aDP * aDV) / aDV.SquareModulus()));
            double aD = (aV * aDV - aDP).SquareModulus();

            // project on side V=0
            double u = Min(1.0, Max(0.0, (aDP * aDU) / aDU.SquareModulus()));
            double d = (u * aDU - aDP).SquareModulus();
            if (d < aD)
            {
                aU = u;
                aV = 0.0;
                aD = d;
            }

            // project on side U+V=1
            gp_XY aDUV = aDV - aDU;
            double v = Min(1.0, Max(0.0, ((aDP - aDU) * aDUV) / aDUV.SquareModulus()));
            d = (theP2 + v * aDUV - theP).SquareModulus();
            if (d < aD)
            {
                aU = 1.0 - v;
                aV = v;
                aD = d;
            }

            theUV = new gp_XY(aU, aV);
            return aD;
        }

        // degenerated triangle
        double aL2U = aDU.SquareModulus();
        double aL2V = aDV.SquareModulus();
        if (aL2U < gp.Resolution()) // side 1-2 is degenerated
        {
            if (aL2V < gp.Resolution()) // whole triangle is degenerated to point
            {
                theUV = new gp_XY(0.0, 0.0);
                return (theP - theP1).SquareModulus();
            }
            else
            {
                theUV = new gp_XY(0.0, (aDP * aDV) / aL2V);
                return (theP - (theP1 + theUV.Y() * aDV)).SquareModulus();
            }
        }
        else if (aL2V < gp.Resolution()) // side 1-3 is degenerated
        {
            theUV = new gp_XY((aDP * aDU) / aL2U, 0.0);
            return (theP - (theP1 + theUV.X() * aDU)).SquareModulus();
        }
        else // sides 1-2 and 1-3 are collinear
        {
            // select parameter on one of sides so as to have points closer to picked
            double aU = Min(1.0, Max(0.0, (aDP * aDU) / aL2U));
            double aV = Min(1.0, Max(0.0, (aDP * aDV) / aL2V));
            double aD1 = (aDP - aU * aDU).SquareModulus();
            double aD2 = (aDP - aV * aDV).SquareModulus();
            if (aD1 < aD2)
            {
                theUV = new gp_XY((aDP * aDU) / aL2U, 0.0);
                return aD1;
            }
            else
            {
                theUV = new gp_XY(0.0, (aDP * aDV) / aL2V);
                return aD2;
            }
        }
    }

    /// <summary>
    /// The intersection of a ray with a triangulation: the closest (or the farthest) intersected triangle and the
    /// distance to it along the ray. False if the ray misses; <paramref name="theTriangle"/> and
    /// <paramref name="theDistance"/> are then unchanged.
    /// </summary>
    public static bool Intersect(Poly_Triangulation theTri, in gp_Ax1 theAxis, bool theIsClosest, ref Poly_Triangle theTriangle, ref double theDistance)
    {
        const double aConf = 1E-15;
        gp_XYZ aLoc = theAxis.Location().XYZ();
        gp_Dir aDir = theAxis.Direction();

        double aResult = theIsClosest ? RealLast() : 0.0;
        double aParam = 0.0;
        Span<int> aTriNodes = [0, 0, 0];
        for (int aTriIter = 1; aTriIter <= theTri.NbTriangles(); ++aTriIter)
        {
            Poly_Triangle aTri = theTri.Triangle(aTriIter);
            aTri.Get(out aTriNodes[0], out aTriNodes[1], out aTriNodes[2]);
            if (IntersectTriLine(aLoc,
                                 aDir,
                                 theTri.Node(aTriNodes[0]).XYZ(),
                                 theTri.Node(aTriNodes[1]).XYZ(),
                                 theTri.Node(aTriNodes[2]).XYZ(),
                                 ref aParam) != 0)
            {
                if (aParam > aConf)
                {
                    if (theIsClosest)
                    {
                        if (aParam < aResult)
                        {
                            aResult = aParam;
                            theTriangle = aTri;
                        }
                    }
                    else if (aParam > aResult)
                    {
                        aResult = aParam;
                        theTriangle = aTri;
                    }
                }
            }
        }

        if (aConf < aResult && aResult < RealLast())
        {
            theDistance = aResult;
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
    /// The intersection of the line (<paramref name="theStart"/>, <paramref name="theDir"/>) with the triangle
    /// (<paramref name="theV0"/>, <paramref name="theV1"/>, <paramref name="theV2"/>): 1 and the parameter on the line
    /// if found; 0 otherwise, <paramref name="theParam"/> unchanged.
    /// </summary>
    public static int IntersectTriLine(in gp_XYZ theStart, in gp_Dir theDir, in gp_XYZ theV0, in gp_XYZ theV1, in gp_XYZ theV2, ref double theParam)
    {
        int aRes = 0;
        const double aConf = 1E-15;

        ReadOnlySpan<double> aMat34 =
        [
            -theDir.X(), theV1.X() - theV0.X(), theV2.X() - theV0.X(), theStart.X() - theV0.X(),
            -theDir.Y(), theV1.Y() - theV0.Y(), theV2.Y() - theV0.Y(), theStart.Y() - theV0.Y(),
            -theDir.Z(), theV1.Z() - theV0.Z(), theV2.Z() - theV0.Z(), theStart.Z() - theV0.Z(),
        ];

        double aD = Determinant(aMat34, 0, 1, 2);
        double aDt = Determinant(aMat34, 3, 1, 2);
        if (aD > aConf)
        {
            double aDa = Determinant(aMat34, 0, 3, 2);
            if (aDa > -aConf)
            {
                double aDb = Determinant(aMat34, 0, 1, 3);
                aRes = ((aDb > -aConf) && (aDa + aDb <= aD + aConf)) ? 1 : 0;
            }
        }
        else if (aD < -aConf)
        {
            double aDa = Determinant(aMat34, 0, 3, 2);
            if (aDa < aConf)
            {
                double aDb = Determinant(aMat34, 0, 1, 3);
                aRes = ((aDb < aConf) && (aDa + aDb >= aD - aConf)) ? 1 : 0;
            }
        }

        if (aRes != 0)
        {
            theParam = aDt / aD;
        }

        return aRes;
    }

    /// <summary>
    /// The area and perimeter of a 2D polygon given by its vertices (not repeating the first). The area is negative if
    /// the polygon runs clockwise; the perimeter is always positive.
    /// </summary>
    public static bool PolygonProperties(NCollection_Array1<gp_Pnt2d> theSeqPnts, out double theArea, out double thePerimeter)
    {
        if (theSeqPnts.Length() < 2)
        {
            theArea = thePerimeter = 0.0;
            return true;
        }

        int aStartIndex = theSeqPnts.Lower();
        gp_XY aRefPnt = theSeqPnts.Value(aStartIndex++).XY();
        gp_XY aPrevPt = theSeqPnts.Value(aStartIndex++).XY() - aRefPnt, aCurrPt;
        theArea = 0.0;
        thePerimeter = aPrevPt.Modulus();
        for (int i = aStartIndex; i <= theSeqPnts.Upper(); i++)
        {
            aCurrPt = theSeqPnts.Value(i).XY() - aRefPnt;
            double aDelta = aPrevPt.Crossed(aCurrPt);
            theArea += aDelta;
            thePerimeter += (aPrevPt - aCurrPt).Modulus();
            aPrevPt = aCurrPt;
        }

        thePerimeter += aPrevPt.Modulus();
        theArea *= 0.5;
        return true;
    }

    /// <summary>
    /// The area and perimeter of a 2D polygon given by its vertices (not repeating the first). The area is negative if
    /// the polygon runs clockwise; the perimeter is always positive.
    /// </summary>
    public static bool PolygonProperties(NCollection_Sequence<gp_Pnt2d> theSeqPnts, out double theArea, out double thePerimeter)
    {
        if (theSeqPnts.Length() < 2)
        {
            theArea = thePerimeter = 0.0;
            return true;
        }

        int aStartIndex = NCollection_Sequence<gp_Pnt2d>.Lower();
        gp_XY aRefPnt = theSeqPnts.Value(aStartIndex++).XY();
        gp_XY aPrevPt = theSeqPnts.Value(aStartIndex++).XY() - aRefPnt, aCurrPt;
        theArea = 0.0;
        thePerimeter = aPrevPt.Modulus();
        for (int i = aStartIndex; i <= theSeqPnts.Upper(); i++)
        {
            aCurrPt = theSeqPnts.Value(i).XY() - aRefPnt;
            double aDelta = aPrevPt.Crossed(aCurrPt);
            theArea += aDelta;
            thePerimeter += (aPrevPt - aCurrPt).Modulus();
            aPrevPt = aCurrPt;
        }

        thePerimeter += aPrevPt.Modulus();
        theArea *= 0.5;
        return true;
    }

    // C++ stream formatting: OS << int, OS << double (six significant digits, as printf's %g), std::setw

    private static string Int(int theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    // %g with six digits, as Microsoft's C runtime writes it (ties rounded away from zero; see Standard_Stream)
    internal static string Real(double theValue) => Standard_Stream.FormatReal(theValue, 6);

    private static string Setw(int theWidth, string theText) => theText.PadLeft(theWidth);

    private static string Field(bool theCompact, int theWidth, string theText) => theCompact ? theText : theText.PadLeft(theWidth);

    // C++ stream extraction (operator>>) on whitespace-separated words. A failed extraction gives zero and puts the
    // stream in a failed state, in which every later extraction fails too.
    private sealed class Extractor(TextReader theReader)
    {
        private bool myFailed;

        public string Word()
        {
            if (myFailed)
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
                myFailed = true;
                return "";
            }

            var aBuilder = new System.Text.StringBuilder();
            while (aChar >= 0 && !char.IsWhiteSpace((char)aChar))
            {
                aBuilder.Append((char)theReader.Read());
                aChar = theReader.Peek();
            }

            return aBuilder.ToString();
        }

        public int Int()
        {
            string aWord = Word();
            if (!myFailed && int.TryParse(aWord, NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out int aValue))
            {
                return aValue;
            }

            myFailed = true;
            return 0;
        }

        public bool Bool()
        {
            // a bool is extracted as an integer, 0 or 1; any other integer gives true and fails the stream
            int aValue = Int();
            if (myFailed || aValue is 0 or 1)
            {
                return aValue == 1;
            }

            myFailed = true;
            return true;
        }

        public double Real()
        {
            string aWord = Word();
            if (!myFailed && double.TryParse(aWord, NumberStyles.Float, CultureInfo.InvariantCulture, out double aValue))
            {
                return aValue;
            }

            myFailed = true;
            return 0.0;
        }
    }
}
