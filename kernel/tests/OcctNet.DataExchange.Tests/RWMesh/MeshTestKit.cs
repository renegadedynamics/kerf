// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;

namespace OcctNet.DataExchange.Tests;

/// <summary>Shapes, trees and comparisons shared by the assembly tree's and the mesh formats' tests (unit 8T).</summary>
internal static class MeshTestKit
{
    public static readonly ColorWithAlpha Red = new(1.0f, 0.0f, 0.0f, 1.0f);
    public static readonly ColorWithAlpha Green = new(0.0f, 1.0f, 0.0f, 1.0f);
    public static readonly ColorWithAlpha Blue = new(0.0f, 0.0f, 1.0f, 0.5f);

    public static ToolParameters MeshParameters => new() { Deflection = 0.1, Angle = 0.5 };

    /// <summary>A box, unmeshed.</summary>
    public static Shape Box(double theDx, double theDy, double theDz)
        => new MakeBox(theDx, theDy, theDz).Shape();

    /// <summary>A box, meshed (frozen).</summary>
    public static Shape MeshedBox(double theDx, double theDy, double theDz)
        => IncrementalMesh.Meshed(Box(theDx, theDy, theDz), MeshParameters);

    public static Placement Translation(double theX, double theY, double theZ)
        => new(Transform3D.Translation(new Vector3D(theX, theY, theZ)));

    /// <summary>A quarter turn about Z, then a translation.</summary>
    public static Placement QuarterTurn(double theX, double theY, double theZ)
        => new(Transform3D.Translation(new Vector3D(theX, theY, theZ))
               * Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 2));

    /// <summary>
    /// A sample assembly placed at (1, 2, 3): a red box "Box" twice (the second turned, green, named "Box2" on the
    /// instance) and a plate whose first face is blue (translucent), the plate in layer "L1".
    /// </summary>
    public static XdeDoc_Tree SampleTree(bool isMeshed = true)
    {
        Shape aBox = isMeshed ? MeshedBox(10, 20, 30) : Box(10, 20, 30);
        Shape aPlate = isMeshed ? MeshedBox(5, 5, 1) : Box(5, 5, 1);
        XdeDoc_Product aBoxPart = XdeDoc_Product.Part(aBox, new XdeDoc_Attributes { Name = "Box", SurfaceColor = Red });
        Shape aFirstFace = new Explorer(aPlate, ShapeType.Face).Current();
        XdeDoc_Product aPlatePart = XdeDoc_Product.Part(aPlate, new XdeDoc_Attributes { Name = "Plate", Layers = ["L1"] })
                                                  .WithSubShape(aFirstFace, new XdeDoc_Attributes { SurfaceColor = Blue });
        XdeDoc_Product anAssembly = XdeDoc_Product.Assembly("Asm",
            aBoxPart.Placed(Translation(0, 0, 0)),
            aBoxPart.Placed(QuarterTurn(50, 0, 0), new XdeDoc_Attributes { Name = "Box2", SurfaceColor = Green }),
            aPlatePart.Placed(Translation(0, 40, 0)));
        return new XdeDoc_Tree([anAssembly.Placed(Translation(1, 2, 3))], null, XdeDoc_LengthUnit.Millimetre);
    }

    /// <summary>
    /// The tree's triangles in the world, as sorted canonical strings (each triangle's points rounded to 0.01, started at
    /// its smallest point, the winding kept).
    /// </summary>
    public static List<string> WorldTriangles(XdeDoc_Tree theTree)
    {
        var aResult = new List<string>();
        foreach (XdeDoc_FlatItem anItem in theTree.Flatten())
        {
            for (var anExp = new Explorer(anItem.Shape, ShapeType.Face); anExp.More(); anExp.Next())
            {
                Face aFace = ShapeCasts.Face(anExp.Current());
                Triangulation? aTri = Tool.Triangulation(aFace, out Placement aLoc);
                if (aTri is null)
                {
                    continue;
                }
                Transform3D aTrsf = aLoc.Transformation();
                bool toSwap = (aFace.Orientation() == Orientation.Reversed) ^ (aTrsf.VectorialPart().Determinant() < 0);
                for (int i = 1; i <= aTri.TriangleCount; i++)
                {
                    aTri.Triangle(i).Get(out int a, out int b, out int c);
                    if (toSwap)
                    {
                        (b, c) = (c, b);
                    }
                    aResult.Add(Canonical(aTri.Node(a).Transformed(aTrsf), aTri.Node(b).Transformed(aTrsf), aTri.Node(c).Transformed(aTrsf)));
                }
            }
        }
        aResult.Sort(StringComparer.Ordinal);
        return aResult;
    }

    public static string Canonical(Point3D theA, Point3D theB, Point3D theC)
    {
        string[] aPoints = [Key(theA), Key(theB), Key(theC)];
        int aFirst = 0;
        for (int i = 1; i < 3; i++)
        {
            if (string.CompareOrdinal(aPoints[i], aPoints[aFirst]) < 0)
            {
                aFirst = i;
            }
        }
        return aPoints[aFirst] + "|" + aPoints[(aFirst + 1) % 3] + "|" + aPoints[(aFirst + 2) % 3];
    }

    public static string Key(Point3D theP)
    {
        static string r(double v)
        {
            double aR = Math.Round(v, 2);
            return (aR == 0 ? 0.0 : aR).ToString("F2", CultureInfo.InvariantCulture);
        }
        return r(theP.X()) + "," + r(theP.Y()) + "," + r(theP.Z());
    }

    public static int CountShapes(in Shape theShape, ShapeType theType)
    {
        int aCount = 0;
        var aSeen = new HashSet<Shape>(ShapeHasher.Instance);
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            if (aSeen.Add(anExp.Current()))
            {
                aCount++;
            }
        }
        return aCount;
    }

    /// <summary>The triangles and nodes of the shape's faces, as Draw's checktrinfo counts them (each face found).</summary>
    public static (int Triangles, int Nodes) TriInfo(in Shape theShape)
    {
        int aTris = 0, aNodes = 0;
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Triangulation? aTri = Tool.Triangulation(ShapeCasts.Face(anExp.Current()), out _);
            if (aTri is not null)
            {
                aTris += aTri.TriangleCount;
                aNodes += aTri.NodeCount;
            }
        }
        return (aTris, aNodes);
    }

    public static string? DatasetFile(string theName)
    {
        string aPath = Path.Combine(RWStlNativeTests.OcctTestDataDir, "others", theName);
        return File.Exists(aPath) ? aPath : null;
    }
}
