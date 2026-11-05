// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest's box commands (BRepTest_BasicCommands.cxx): bounding (over BRepBndLib), isbbinterf, gbounding (over
/// BndLib) and getcoords.
/// </summary>
/// <remarks>
/// <c>bounding -shape</c> makes its box with BRepPrimAPI_MakeBox; <c>-dumpJson</c> needs Bnd_Box's and
/// Bnd_OBB's DumpJson, which aren't ported: it fails with a message saying so. Draw_Box (the drawn box) is left out.
/// </remarks>
internal static class BRepTestBoundingCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bounding", BoundBox);
        DrawCxx.Add(theInterpreter, "isbbinterf", IsBoxesInterfered);
        DrawCxx.Add(theInterpreter, "gbounding", GBounding);
        DrawCxx.Add(theInterpreter, "getcoords", GetCoords);
    }

    private static int GetCoords(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        for (int i = 1; i < n; i++)
        {
            Shape aShape = theDI.GetShape(a[i]);
            if (aShape.IsNull)
            {
                continue;
            }

            if (aShape.Type == ShapeType.Vertex)
            {
                Point3D aPnt = Tool.Pnt(ShapeCasts.Vertex(aShape));
                di.Append(a[i]).Append(" (x,y,z) : ").Append(aPnt.X).Append(" ").Append(aPnt.Y).Append(" ").Append(aPnt.Z).Append("\n");
            }
        }

        return 0;
    }

    /// <summary>TCollection_AsciiString::IsRealValue (strtod converts a prefix) and RealValue.</summary>
    private static bool IsRealValue(string theText, out double theValue)
    {
        theValue = global::Kerf.Engine.Foundation.StreamUtilities.Strtod(theText, out int anEnd);
        return anEnd != 0;
    }

    /// <summary>Parse 6 real values for defining AABB.</summary>
    private static bool ParseMinMax(IReadOnlyList<string> theArgVec, int theFrom, ref Box theBox)
    {
        double[] aValues = new double[6];
        for (int i = 0; i < 6; i++)
        {
            if (!IsRealValue(theArgVec[theFrom + i], out aValues[i]))
            {
                return false;
            }
        }

        var aPntMin = new Point3D(aValues[0], aValues[1], aValues[2]);
        var aPntMax = new Point3D(aValues[3], aValues[4], aValues[5]);
        theBox = Box.Void().Added(aPntMin).Added(aPntMax);
        return true;
    }

    private static int BoundBox(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        // 1. Parse arguments
        int theNArg = theArgVal.Count;
        Shape aShape = new();
        Box anAABB = new();

        bool doPrint = false;
        bool doDumpJson = false;
        bool useOldSyntax = false;
        bool isOBB = false;
        bool isTriangulationReq = true;
        bool isOptimal = false;
        bool isTolerUsed = false;
        bool isFinitePart = false;

        string[] anOutVars = ["", "", "", "", "", ""];
        string aResShapeName = "";
        for (int anArgIter = 1; anArgIter < theNArg; ++anArgIter)
        {
            string anArgCase = theArgVal[anArgIter].ToLowerInvariant();
            if (anArgCase == "-obb")
            {
                isOBB = true;
            }
            else if (anArgCase == "-aabb")
            {
                isOBB = false;
            }
            else if (anArgCase == "-shape" && anArgIter + 1 < theNArg && aResShapeName.Length == 0)
            {
                aResShapeName = theArgVal[++anArgIter];
            }
            else if (anArgCase is "-dump" or "-print")
            {
                doPrint = true;
            }
            else if (anArgCase == "-dumpjson")
            {
                doDumpJson = true;
            }
            else if (anArgCase == "-save" && anArgIter + 6 < theNArg && anOutVars[0].Length == 0)
            {
                for (int aCompIter = 0; aCompIter < 6; ++aCompIter)
                {
                    anOutVars[aCompIter] = theArgVal[anArgIter + aCompIter + 1];
                }

                anArgIter += 6;
            }
            else if (anArgCase == "-notriangulation")
            {
                isTriangulationReq = false;
            }
            else if (anArgCase == "-optimal")
            {
                isOptimal = true;
            }
            else if (anArgCase == "-exttoler")
            {
                isTolerUsed = true;
            }
            else if (anArgCase == "-nodraw")
            {
                // the drawn box: no viewer
            }
            else if (anArgCase is "-finite" or "-finitepart")
            {
                isFinitePart = true;
            }
            else if (aShape.IsNull && !theDI.GetShape(theArgVal[anArgIter]).IsNull)
            {
                aShape = theDI.GetShape(theArgVal[anArgIter]);
            }
            else if (anAABB.IsVoid && anArgIter + 5 < theNArg && ParseMinMax(theArgVal, anArgIter, ref anAABB))
            {
                anArgIter += 5;
            }
            else
            {
                theDI.Output.WriteLine($"Syntax error at argument '{theArgVal[anArgIter]}'");
                return 1;
            }
        }

        if (anAABB.IsVoid && aShape.IsNull)
        {
            theDI.Output.WriteLine("Syntax error: input is not specified (neither shape nor coordinates)");
            return 1;
        }
        else if (!anAABB.IsVoid && (isOBB || isOptimal || isTolerUsed))
        {
            theDI.Output.WriteLine("Syntax error: Options -obb, -optimal and -extToler cannot be used for explicitly defined AABB");
            return 1;
        }
        else if (isOBB && anOutVars[0].Length != 0)
        {
            theDI.Output.WriteLine("Error: Option -save works only with axes-aligned boxes");
            return 1;
        }

        // enable printing (old syntax) if neither saving to shape nor to DRAW variables is requested
        if (!doPrint && !doDumpJson && anOutVars[0].Length == 0 && aResShapeName.Length == 0)
        {
            doPrint = true;
            useOldSyntax = true;
        }

        // 2. Compute box and save results
        if (isOBB)
        {
            var anOBB = new Obb();
            Bounds.AddOBB(aShape, ref anOBB, isTriangulationReq, isOptimal, isTolerUsed);

            if (anOBB.IsVoid)
            {
                theResult.Append("Void box.\n");
            }
            else if (doPrint)
            {
                Xyz aBaryCenter = anOBB.Center();
                Xyz aXDir = anOBB.XDirection(), aYDir = anOBB.YDirection(), aZDir = anOBB.ZDirection();
                theResult.Append("Oriented bounding box\n");
                theResult.Append("Center: ").Append(aBaryCenter.X()).Append(" ").Append(aBaryCenter.Y()).Append(" ").Append(aBaryCenter.Z()).Append("\n");
                theResult.Append("X-axis: ").Append(aXDir.X()).Append(" ").Append(aXDir.Y()).Append(" ").Append(aXDir.Z()).Append("\n");
                theResult.Append("Y-axis: ").Append(aYDir.X()).Append(" ").Append(aYDir.Y()).Append(" ").Append(aYDir.Z()).Append("\n");
                theResult.Append("Z-axis: ").Append(aZDir.X()).Append(" ").Append(aZDir.Y()).Append(" ").Append(aZDir.Z()).Append("\n");
                theResult.Append("Half X: ").Append(anOBB.XHSize()).Append("\n")
                         .Append("Half Y: ").Append(anOBB.YHSize()).Append("\n")
                         .Append("Half Z: ").Append(anOBB.ZHSize()).Append("\n");
            }

            if (doDumpJson)
            {
                return NotPorted(theResult, "-dumpJson (Bnd_OBB::DumpJson)");
            }

            if (aResShapeName.Length != 0)
            {
                return ConvertBndToShape(theDI, theResult, anOBB, aResShapeName);
            }
        }
        else
        {
            if (!aShape.IsNull)
            {
                anAABB = Box.Void();
                if (isOptimal)
                {
                    Bounds.AddOptimal(aShape, ref anAABB, isTriangulationReq, isTolerUsed);
                }
                else
                {
                    Bounds.Add(aShape, ref anAABB, isTriangulationReq);
                }
            }

            if (anAABB.IsVoid)
            {
                theResult.Append("Void box.\n");
            }
            else
            {
                if (isFinitePart && anAABB.IsOpen)
                {
                    anAABB = anAABB.FinitePart();
                }

                Point3D aMin = anAABB.CornerMin();
                Point3D aMax = anAABB.CornerMax();

                // print to DRAW
                if (doPrint)
                {
                    if (useOldSyntax)
                    {
                        theResult.Append(aMin.X).Append(" ").Append(aMin.Y).Append(" ").Append(aMin.Z).Append(" ")
                                 .Append(aMax.X).Append(" ").Append(aMax.Y).Append(" ").Append(aMax.Z).Append("\n");
                    }
                    else
                    {
                        theResult.Append("Axes-aligned bounding box\n");
                        theResult.Append("X-range: ").Append(aMin.X).Append(" ").Append(aMax.X).Append("\n")
                                 .Append("Y-range: ").Append(aMin.Y).Append(" ").Append(aMax.Y).Append("\n")
                                 .Append("Z-range: ").Append(aMin.Z).Append(" ").Append(aMax.Z).Append("\n");
                        if (anAABB.IsOpen && anAABB.HasFinitePart)
                        {
                            Box aFinitAabb = anAABB.FinitePart();
                            Point3D aFinMin = aFinitAabb.CornerMin();
                            Point3D aFinMax = aFinitAabb.CornerMax();
                            theResult.Append("Finite part\n");
                            theResult.Append("X-range: ").Append(aFinMin.X).Append(" ").Append(aFinMax.X).Append("\n")
                                     .Append("Y-range: ").Append(aFinMin.Y).Append(" ").Append(aFinMax.Y).Append("\n")
                                     .Append("Z-range: ").Append(aFinMin.Z).Append(" ").Append(aFinMax.Z).Append("\n");
                        }
                    }
                }

                if (doDumpJson)
                {
                    return NotPorted(theResult, "-dumpJson (Bnd_Box::DumpJson)");
                }

                // save DRAW variables
                if (anOutVars[0].Length != 0)
                {
                    theDI.SetNumber(anOutVars[0], aMin.X);
                    theDI.SetNumber(anOutVars[1], aMin.Y);
                    theDI.SetNumber(anOutVars[2], aMin.Z);
                    theDI.SetNumber(anOutVars[3], aMax.X);
                    theDI.SetNumber(anOutVars[4], aMax.Y);
                    theDI.SetNumber(anOutVars[5], aMax.Z);
                }
            }

            // save as shape
            if (aResShapeName.Length != 0)
            {
                return ConvertBndToShape(theDI, theResult, new Obb(anAABB), aResShapeName);
            }
        }

        return 0;
    }

    /// <summary>ConvertBndToShape: the box as a solid (BRepPrimAPI_MakeBox on the box's axes), or a null shape for a void box.</summary>
    private static int ConvertBndToShape(DrawInterpreter theDI, DrawResult theResult, in Obb theBox, string theName)
    {
        if (theBox.IsVoid)
        {
            theDI.SetShape(theName, new Shape());
            return 0;
        }

        Point3D aBaryCenter = new(theBox.Center());
        Xyz aXDir = theBox.XDirection(), aYDir = theBox.YDirection(), aZDir = theBox.ZDirection();
        double aHalfX = theBox.XHSize(), aHalfY = theBox.YHSize(), aHalfZ = theBox.ZHSize();

        Frame3D anAxes = new(aBaryCenter, new Direction3D(aZDir), new Direction3D(aXDir));
        anAxes = anAxes.WithLocation(new Point3D(aBaryCenter.XYZ() - aHalfX * aXDir - aHalfY * aYDir - aHalfZ * aZDir));
        Solid aBox = new MakeBox(anAxes, 2.0 * aHalfX, 2.0 * aHalfY, 2.0 * aHalfZ).Solid();
        theDI.SetShape(theName, aBox);
        return 0;
    }

    private static int NotPorted(DrawResult theResult, string theWhat)
    {
        theResult.Append($"bounding {theWhat} is not ported yet\n");
        return 1;
    }

    private static int IsBoxesInterfered(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        int theNArg = theArgVal.Count;
        if (theNArg < 3)
        {
            // OCCT bug: checks for fewer than 2 arguments, then reads the third: a null shape name with 2 (occt-issues.md)
            theResult.Append("Use: isbbinterf shape1 shape2 [-o].\n");
            return 1;
        }

        Shape aShape1 = theDI.GetShape(theArgVal[1]);
        Shape aShape2 = theDI.GetShape(theArgVal[2]);

        bool isOBB = theNArg > 3 && theArgVal[3] == "-o";

        if (isOBB)
        {
            Obb anOBB1 = new(), anOBB2 = new();
            Bounds.AddOBB(aShape1, ref anOBB1);
            Bounds.AddOBB(aShape2, ref anOBB2);
            theResult.Append(anOBB1.IsOut(anOBB2) ? "The shapes are NOT interfered by OBB.\n" : "The shapes are interfered by OBB.\n");
        }
        else
        {
            Box anAABB1 = new(), anAABB2 = new();
            Bounds.Add(aShape1, ref anAABB1);
            Bounds.Add(aShape2, ref anAABB2);
            theResult.Append(anAABB1.IsOut(anAABB2) ? "The shapes are NOT interfered by AABB.\n" : "The shapes are interfered by AABB.\n");
        }

        return 0;
    }

    private static int GBounding(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 2 && n != 3)
        {
            di.Append("Usage: gbounding surf/curve/curve2d [-o] \n");
            di.Append("[-o] turn on Optimal mode ('off' by default) \n");
            return 1;
        }

        bool IsOptimal = n == 3 && a[2] == "-o";
        Box B = new();
        Box2d B2d = new();
        bool Is3d = true;
        object? aGeometry = (theDI.GetDrawable(a[1]) as DrawGeometry)?.Geometry;
        if (aGeometry is Surface S)
        {
            // add surf
            var aGAS = new SurfaceAdaptor(S);
            if (IsOptimal)
            {
                AddSurface.AddOptimal(aGAS, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B);
            }
            else
            {
                AddSurface.Add(aGAS, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B);
            }
        }
        else if (aGeometry is Curve3D C)
        {
            // add cur
            var aGAC = new CurveAdaptor(C);
            if (IsOptimal)
            {
                Add3dCurve.AddOptimal(aGAC, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B);
            }
            else
            {
                Add3dCurve.Add(aGAC, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B);
            }
        }
        else if (aGeometry is Curve2D C2d)
        {
            // add cur2d
            Is3d = false;
            if (IsOptimal)
            {
                Add2dCurve.AddOptimal(C2d, C2d.FirstParameter, C2d.LastParameter, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B2d);
            }
            else
            {
                Add2dCurve.Add(C2d, C2d.FirstParameter, C2d.LastParameter, global::Kerf.Engine.Foundation.Precision.Confusion(), ref B2d);
            }
        }
        else
        {
            di.Append("Wrong argument \n");
            return 1;
        }

        if (Is3d)
        {
            B.Get(out double axmin, out double aymin, out double azmin, out double axmax, out double aymax, out double azmax);
            di.Append(axmin).Append(" ").Append(aymin).Append(" ").Append(azmin).Append(" ").Append(axmax).Append(" ").Append(aymax).Append(" ").Append(azmax);
        }
        else
        {
            B2d.Get(out double axmin, out double aymin, out double axmax, out double aymax);
            di.Append(axmin).Append(" ").Append(aymin).Append(" ").Append(axmax).Append(" ").Append(aymax);
        }

        return 0;
    }
}
