// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The classification commands of BOPTest_LowCommands.cxx that need only BRepClass and BRepClass3d (bclassify,
/// b2dclassify), and GeomliteTest's <c>point</c>, which makes the points they classify.
/// </summary>
/// <remarks>
/// b2dclassifx classifies with IntTools_FClass2d (TKBO), which isn't ported; bhaspc builds pcurves with
/// BOPTools_AlgoTools2D. Neither is here.
/// </remarks>
internal static class BOPTestClassifyCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bclassify", BClassify);
        DrawCxx.Add(theInterpreter, "b2dclassify", B2dClassify);
        DrawCxx.Add(theInterpreter, "point", Point);
    }

    /// <summary>GeomliteTest's <c>point name x y [z]</c>: a DrawTrSurf_Point, 3D with z, 2D without.</summary>
    private static int Point(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        if (n >= 5)
        {
            theDI.SetDrawable(a[1], DrawGeometry.Of(new Point3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]))));
        }
        else
        {
            theDI.SetDrawable(a[1], DrawGeometry.Of(new Point2D(theDI.Atof(a[2]), theDI.Atof(a[3]))));
        }

        return 0;
    }

    /// <summary>DrawTrSurf::GetPoint: the 3D point of that name, else the point is left as it is.</summary>
    internal static bool GetPoint(DrawInterpreter theDI, string theName, ref Point3D P)
    {
        if (theDI.GetDrawable(theName) is DrawGeometry { Geometry: Point3D aPoint })
        {
            P = aPoint;
            return true;
        }

        return false;
    }

    /// <summary>DrawTrSurf::GetPoint2d: the 2D point of that name, else the point is left as it is.</summary>
    internal static bool GetPoint2d(DrawInterpreter theDI, string theName, ref Point2D P)
    {
        if (theDI.GetDrawable(theName) is DrawGeometry { Geometry: Point2D aPoint })
        {
            P = aPoint;
            return true;
        }

        return false;
    }

    private static int B2dClassify(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theArgNb = theArgVec.Count;
        if (theArgNb < 3)
        {
            theResult.Append(" use b2dclassify Face Point2d [Tol] [UseBox] [GapCheckTol]\n");
            return 1;
        }

        Shape aS = theDI.GetShape(theArgVec[1]);
        if (aS.IsNull)
        {
            theResult.Append(" Null Shape is not allowed here\n");
            return 1;
        }
        else if (aS.Type != ShapeType.Face)
        {
            theResult.Append(" Shape type must be FACE\n");
            return 1;
        }

        var aP = new Point2D(8.0, 9.0);
        GetPoint2d(theDI, theArgVec[2], ref aP);
        Face aF = ShapeCasts.Face(aS);
        double aTol = theArgNb >= 4 ? theDI.Atof(theArgVec[3]) : Tool.Tolerance(aF);
        bool anUseBox = theArgNb >= 5 && theDI.Atof(theArgVec[4]) == 1;
        double aGapCheckTol = theArgNb == 6 ? theDI.Atof(theArgVec[5]) : 0.1;
        var aClassifier = new FaceClassifier();
        aClassifier.Perform(aF, aP, aTol, anUseBox, aGapCheckTol);
        PrintState(theResult, aClassifier.State());
        return 0;
    }

    private static int BClassify(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theArgNb = theArgVec.Count;
        if (theArgNb < 3)
        {
            theResult.Append(" use bclassify Solid Point [Tolerance=1.e-7]\n");
            return 1;
        }

        Shape aS = theDI.GetShape(theArgVec[1]);
        if (aS.IsNull)
        {
            theResult.Append(" Null Shape is not allowed\n");
            return 1;
        }
        else if (aS.Type != ShapeType.Solid)
        {
            theResult.Append(" Shape type must be SOLID\n");
            return 1;
        }

        var aP = new Point3D(8.0, 9.0, 10.0);
        GetPoint(theDI, theArgVec[2], ref aP);
        double aTol = theArgNb == 4 ? theDI.Atof(theArgVec[3]) : 1.0e-7;

        var aSC = new SolidClassifier(aS);
        aSC.Perform(aP, aTol);

        PrintState(theResult, aSC.State());
        return 0;
    }

    private static void PrintState(DrawResult theDI, Containment theState)
    {
        switch (theState)
        {
            case Containment.In:
                theDI.Append("The point is IN shape\n");
                break;
            case Containment.Out:
                theDI.Append("The point is OUT of shape\n");
                break;
            case Containment.On:
                theDI.Append("The point is ON shape\n");
                break;
            default:
                theDI.Append("The point is UNKNOWN shape\n");
                break;
        }
    }
}
