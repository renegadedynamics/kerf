// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Poly_Triangulation and Poly_Triangle against native OCCT, on random triangulations. (A namespace named after the
/// package would hide the Poly class, so these tests are in the root namespace.)
/// </summary>
public class PolyTriangulationOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(300, 167);

    [Fact]
    public void Triangulation_ComputeNormals()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Poly_Triangulation aNative, Poly_Triangulation aPort) = RandomTriangulation(anOracle, false);
            using (aNative)
            {
                aNative.ComputeNormals();
                aPort.ComputeNormals();
                anOracle.Same(aNative.HasNormals(), aPort.HasNormals(), "HasNormals");
                for (int k = 1; k <= aPort.NbNodes(); k++)
                {
                    using N.gp_Dir aND = aNative.Normal(k);
                    gp_Dir aPD = aPort.Normal(k);
                    anOracle.Same(aND.X(), aPD.X(), $"Normal({k}).X");
                    anOracle.Same(aND.Y(), aPD.Y(), $"Normal({k}).Y");
                    anOracle.Same(aND.Z(), aPD.Z(), $"Normal({k}).Z");
                }
            }
        }

        anOracle.Report("Poly_Triangulation ComputeNormals");
    }

    [Fact]
    public void Triangulation_nodes_triangles_and_copy()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Poly_Triangulation aNative, Poly_Triangulation aPort) = RandomTriangulation(anOracle, true);
            using (aNative)
            {
                double aDefl = anOracle.Real(0, 1);
                aNative.Deflection(aDefl);
                aPort.Deflection(aDefl);
                using N.Poly_Triangulation aNCopy = aNative.Copy()!;
                Poly_Triangulation aPCopy = aPort.Copy();
                SameTriangulation(anOracle, aNCopy, aPCopy, "Copy");

                // resize, keeping the data, and a node array mapped from the triangulation; OCCT leaves added nodes
                // uninitialised (the port's are at the origin), so only the kept ones are compared
                int anOldNbNodes = aPort.NbNodes();
                int aNbNodes = aPort.NbNodes() + anOracle.Int(-2, 5);
                int aNbTris = Math.Max(1, aPort.NbTriangles() + anOracle.Int(-1, 5)); // OCCT can't resize an array to empty
                aNative.ResizeNodes(aNbNodes, true);
                aPort.ResizeNodes(aNbNodes, true);
                aNative.ResizeTriangles(aNbTris, true);
                aPort.ResizeTriangles(aNbTris, true);
                SameTriangulation(anOracle, aNative, aPort, "Resize", Math.Min(anOldNbNodes, aNbNodes));

                using N.TColgp_HArray1OfPnt aNMapped = aNative.MapNodeArray()!;
                NCollection_HArray1<gp_Pnt> aPMapped = aPort.MapNodeArray()!;
                anOracle.Same(aNMapped.Lower() == aPMapped.Lower() && aNMapped.Upper() == aPMapped.Upper(), true, "MapNodeArray bounds");
                for (int k = aPMapped.Lower(); k <= Math.Min(anOldNbNodes, aNbNodes); k++)
                {
                    using N.gp_Pnt aNP = aNMapped.Value(k);
                    anOracle.Same(Oracle.FromNative(aNP).XYZ(), aPMapped[k].XYZ(), "MapNodeArray");
                }
            }
        }

        anOracle.Report("Poly_Triangulation nodes, triangles, Copy, Resize and MapNodeArray");
    }

    [Fact]
    public void Triangulation_single_precision()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            int aNbNodes = anOracle.Int(1, 20);
            using var aNative = new N.Poly_Triangulation();
            var aPort = new Poly_Triangulation();
            aNative.SetDoublePrecision(false);
            aPort.SetDoublePrecision(false);
            aNative.ResizeNodes(aNbNodes, false);
            aPort.ResizeNodes(aNbNodes, false);
            aNative.AddUVNodes();
            aPort.AddUVNodes();
            anOracle.Same(aNative.IsDoublePrecision(), aPort.IsDoublePrecision(), "IsDoublePrecision");
            for (int k = 1; k <= aNbNodes; k++)
            {
                gp_Pnt aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                aNative.SetNode(k, aNP);
                aPort.SetNode(k, aP);
                var anUV = new gp_Pnt2d(anOracle.Real(), anOracle.Real());
                using var aNUV = new N.gp_Pnt2d(anUV.X(), anUV.Y());
                aNative.SetUVNode(k, aNUV);
                aPort.SetUVNode(k, anUV);
            }

            for (int k = 1; k <= aNbNodes; k++)
            {
                using N.gp_Pnt aNP = aNative.Node(k);
                anOracle.Same(Oracle.FromNative(aNP).XYZ(), aPort.Node(k).XYZ(), "single precision Node");
                using N.gp_Pnt2d aNUV = aNative.UVNode(k);
                anOracle.Same(aNUV.X(), aPort.UVNode(k).X(), "single precision UVNode.X");
                anOracle.Same(aNUV.Y(), aPort.UVNode(k).Y(), "single precision UVNode.Y");
            }
        }

        anOracle.Report("Poly_Triangulation in single precision");
    }

    [Fact]
    public void Triangulation_MinMax_and_cache()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Poly_Triangulation aNative, Poly_Triangulation aPort) = RandomTriangulation(anOracle, false);
            (N.gp_Trsf aNT, gp_Trsf aT) = RandomTrsf(anOracle);
            using (aNative)
            using (aNT)
            {
                bool anAccurate = anOracle.Int(0, 1) == 0;
                using (var aNBox = new N.Bnd_Box())
                {
                    var aBox = new Bnd_Box();
                    anOracle.Same(aNative.MinMax(aNBox, aNT, anAccurate), aPort.MinMax(ref aBox, aT, anAccurate), "MinMax");
                    SameBox(anOracle, aNBox, aBox, "MinMax");
                }

                aNative.UpdateCachedMinMax();
                aPort.UpdateCachedMinMax();
                anOracle.Same(aNative.HasCachedMinMax(), aPort.HasCachedMinMax(), "HasCachedMinMax");
                using (N.Bnd_Box aNCached = aNative.CachedMinMax())
                {
                    SameBox(anOracle, aNCached, aPort.CachedMinMax(), "CachedMinMax");
                }

                // with the cache: used unless accurate with a rotation
                using (var aNBox = new N.Bnd_Box())
                {
                    var aBox = new Bnd_Box();
                    anOracle.Same(aNative.MinMax(aNBox, aNT, anAccurate), aPort.MinMax(ref aBox, aT, anAccurate), "MinMax cached");
                    SameBox(anOracle, aNBox, aBox, "MinMax cached");
                }
            }
        }

        anOracle.Report("Poly_Triangulation MinMax and cached range");
    }

    [Fact]
    public void Triangle_access()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            int n1 = anOracle.Int(-5, 100), n2 = anOracle.Int(-5, 100), n3 = anOracle.Int(-5, 100);
            using var aNative = new N.Poly_Triangle(n1, n2, n3);
            var aPort = new Poly_Triangle(n1, n2, n3);
            int anIndex = anOracle.Int(1, 3), aNode = anOracle.Int(0, 50);
            aNative.Set(anIndex, aNode);
            aPort = aPort.WithNode(anIndex, aNode);
            int a1 = 0, a2 = 0, a3 = 0;
            aNative.Get(ref a1, ref a2, ref a3);
            aPort.Get(out int b1, out int b2, out int b3);
            anOracle.Same((a1, a2, a3) == (b1, b2, b3), true, "Get");
            for (int k = 1; k <= 3; k++)
            {
                anOracle.Same(aNative.Value(k) == aPort.Value(k), true, "Value");
            }
        }

        anOracle.Report("Poly_Triangle");
    }

    // A triangulation made the same way natively and in the port: random nodes, and triangles of random distinct
    // nodes, a few of them degenerate (collinear or coincident nodes) to reach the zero-normal case.
    private static (N.Poly_Triangulation Native, Poly_Triangulation Port) RandomTriangulation(Oracle theOracle, bool theWithUV)
    {
        int aNbNodes = theOracle.Int(3, 40);
        int aNbTris = theOracle.Int(1, 60);
        var aNative = new N.Poly_Triangulation(aNbNodes, aNbTris, theWithUV);
        var aPort = new Poly_Triangulation(aNbNodes, aNbTris, theWithUV);
        for (int k = 1; k <= aNbNodes; k++)
        {
            gp_Pnt aP = theOracle.Int(0, 15) == 0 && k > 1 ? aPort.Node(k - 1) : theOracle.Pnt();
            using var aNP = Oracle.ToNative(aP);
            aNative.SetNode(k, aNP);
            aPort.SetNode(k, aP);
            if (theWithUV)
            {
                var anUV = new gp_Pnt2d(theOracle.Real(), theOracle.Real());
                using var aNUV = new N.gp_Pnt2d(anUV.X(), anUV.Y());
                aNative.SetUVNode(k, aNUV);
                aPort.SetUVNode(k, anUV);
            }
        }

        for (int k = 1; k <= aNbTris; k++)
        {
            int n1 = theOracle.Int(1, aNbNodes), n2, n3;
            do
            {
                n2 = theOracle.Int(1, aNbNodes);
            }
            while (n2 == n1);

            do
            {
                n3 = theOracle.Int(1, aNbNodes);
            }
            while (n3 == n1 || n3 == n2);

            using var aNTri = new N.Poly_Triangle(n1, n2, n3);
            aNative.SetTriangle(k, aNTri);
            aPort.SetTriangle(k, new Poly_Triangle(n1, n2, n3));
        }

        return (aNative, aPort);
    }

    private static void SameTriangulation(Oracle theOracle, N.Poly_Triangulation theNative, Poly_Triangulation thePort, string theWhat,
                                          int theNbSetNodes = int.MaxValue)
    {
        theOracle.Same(theNative.NbNodes() == thePort.NbNodes(), true, $"{theWhat} NbNodes");
        theOracle.Same(theNative.NbTriangles() == thePort.NbTriangles(), true, $"{theWhat} NbTriangles");
        theOracle.Same(theNative.HasUVNodes(), thePort.HasUVNodes(), $"{theWhat} HasUVNodes");
        theOracle.Same(theNative.Deflection(), thePort.Deflection(), $"{theWhat} Deflection");
        for (int k = 1; k <= Math.Min(thePort.NbNodes(), theNbSetNodes); k++)
        {
            using N.gp_Pnt aNP = theNative.Node(k);
            theOracle.Same(Oracle.FromNative(aNP).XYZ(), thePort.Node(k).XYZ(), $"{theWhat} Node");
            if (thePort.HasUVNodes())
            {
                using N.gp_Pnt2d aNUV = theNative.UVNode(k);
                theOracle.Same(aNUV.X(), thePort.UVNode(k).X(), $"{theWhat} UVNode.X");
                theOracle.Same(aNUV.Y(), thePort.UVNode(k).Y(), $"{theWhat} UVNode.Y");
            }
        }

        for (int k = 1; k <= thePort.NbTriangles(); k++)
        {
            using N.Poly_Triangle aNT = theNative.Triangle(k);
            Poly_Triangle aPT = thePort.Triangle(k);
            theOracle.Same(aNT.Value(1) == aPT.Value(1) && aNT.Value(2) == aPT.Value(2) && aNT.Value(3) == aPT.Value(3), true,
                           $"{theWhat} Triangle");
        }
    }

    private static void SameBox(Oracle theOracle, N.Bnd_Box theNative, in Bnd_Box thePort, string theWhat)
    {
        theOracle.Same(theNative.IsVoid(), thePort.IsVoid(), $"{theWhat} IsVoid");
        if (thePort.IsVoid())
        {
            return;
        }

        double aXmin = 0, aYmin = 0, aZmin = 0, aXmax = 0, aYmax = 0, aZmax = 0;
        theNative.Get(ref aXmin, ref aYmin, ref aZmin, ref aXmax, ref aYmax, ref aZmax);
        thePort.Get(out double aPXmin, out double aPYmin, out double aPZmin, out double aPXmax, out double aPYmax, out double aPZmax);
        theOracle.Same(aXmin, aPXmin, $"{theWhat} Xmin");
        theOracle.Same(aYmin, aPYmin, $"{theWhat} Ymin");
        theOracle.Same(aZmin, aPZmin, $"{theWhat} Zmin");
        theOracle.Same(aXmax, aPXmax, $"{theWhat} Xmax");
        theOracle.Same(aYmax, aPYmax, $"{theWhat} Ymax");
        theOracle.Same(aZmax, aPZmax, $"{theWhat} Zmax");
    }

    // An identity, translation, scale, point mirror or rotation, made the same way natively and in the port.
    private static (N.gp_Trsf Native, gp_Trsf Port) RandomTrsf(Oracle theOracle)
    {
        var aNative = new N.gp_Trsf();
        switch (theOracle.Int(0, 4))
        {
            case 0:
                return (aNative, new gp_Trsf());
            case 1:
            {
                gp_Vec aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, gp_Trsf.Translation(aV));
            }
            case 2:
            {
                gp_Pnt aP = theOracle.Pnt();
                double aS = theOracle.Real(0.1, 3);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, gp_Trsf.Scale(aP, aS));
            }
            case 3:
            {
                gp_Pnt aP = theOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                aNative.SetMirror(aNP);
                return (aNative, gp_Trsf.Mirror(aP));
            }
            default:
            {
                gp_Ax1 anAxis = theOracle.Ax1();
                double anAngle = theOracle.Angle();
                using var aNAxis = Oracle.ToNative(anAxis);
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, gp_Trsf.Rotation(anAxis, anAngle));
            }
        }
    }
}
