// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest_ExtremaCommands.cxx: dist (BRepExtrema_Poly), distmini (BRepExtrema_DistShapeShape), proximity
/// (BRepExtrema_ShapeProximity) and selfintersect (BRepExtrema_SelfIntersection).
/// </summary>
/// <remarks>
/// distmini's solutions come in the order of OCCT's task split, which depends on the processor count (decision 447):
/// the names <c>name</c>, <c>name2</c>, ... bind the same solutions as DRAWEXE's on a machine with as many logical
/// processors. proximity and selfintersect print the overlapping sub-shapes in ascending index order, where OCCT
/// iterates its NCollection_DataMap in bucket order (decision 18); the names, made from the indices, are OCCT's.
/// </remarks>
internal static class BRepTestExtremaCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "dist", Distance);
        DrawCxx.Add(theInterpreter, "distmini", DistMini);
        DrawCxx.Add(theInterpreter, "proximity", ShapeProximity);
        DrawCxx.Add(theInterpreter, "selfintersect", ShapeSelfIntersection);
    }

    private static int Distance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        Shape S1 = theDI.GetShape(a[1]);
        Shape S2 = theDI.GetShape(a[2]);
        if (S1.IsNull || S2.IsNull)
        {
            return 1;
        }

        Point3D P1 = new(), P2 = new();
        if (!DistancePoly.Distance(S1, S2, ref P1, ref P2, out double D))
        {
            return 1;
        }

        di.Append(" distance : ").Append(D).Append("\n");
        theDI.SetShape("distance", new LibraryMakeEdge(P1, P2).Edge());
        return 0;
    }

    private static int DistMini(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4 || n > 6)
        {
            return 1;
        }

        string ns1 = a[2], ns2 = a[3], ns0 = a[1];
        Shape S1 = theDI.GetShape(ns1), S2 = theDI.GetShape(ns2);

        double aDeflection = global::Kerf.Engine.Foundation.Precision.Confusion();
        int anIndex = 4;
        if (n >= 5 && !a[4].StartsWith('-'))
        {
            aDeflection = theDI.Atof(a[4]);
            anIndex++;
        }

        bool isMultiThread = false;
        for (int anI = anIndex; anI < n; anI++)
        {
            string anArg = a[anI].ToLowerInvariant();
            if (anArg == "-parallel")
            {
                isMultiThread = true;
            }
            else
            {
                di.Append("Syntax error at '").Append(anArg).Append("'");
                return 1;
            }
        }

        var dst = new DistanceShapeShape();
        dst.LoadS1(S1);
        dst.LoadS2(S2);
        dst.SetDeflection(aDeflection);
        dst.SetMultiThread(isMultiThread);
        dst.Perform();

        if (dst.IsDone)
        {
            di.Append("\"distmini\" command returns:\n");

            string named = ns0 + "_val";
            theDI.SetNumber(named, dst.Value());
            di.Append(named).Append(" ");

            for (int i1 = 1; i1 <= dst.SolutionCount; i1++)
            {
                Point3D P1 = dst.PointOnShape1(i1);
                Point3D P2 = dst.PointOnShape2(i1);
                string aName = i1 == 1 ? ns0 : ns0 + i1.ToString(System.Globalization.CultureInfo.InvariantCulture);
                Shape aSolution = dst.Value() <= 1.0e-9
                                       ? new LibraryMakeVertex(P1).Vertex()
                                       : new LibraryMakeEdge(P1, P2).Edge();
                theDI.SetShape(aName, aSolution);
                di.Append(aName).Append(" ");
            }

            di.Append("\nOutput is complete.\n");
        }
        else
        {
            di.Append("problem: no distance is found\n");
        }

        return 0;
    }

    // OCCT's NCollection_DataMap iterates in bucket order; the port lists the keys in ascending order (decision 18)
    private static List<int> Keys(Dictionary<int, NCollection_PackedMapOfInteger> theMap)
        => theMap.Select(p => p.Key).Order().ToList();

    private static int ShapeProximity(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgs)
    {
        int theNbArgs = theArgs.Count;
        if (theNbArgs < 3 || theNbArgs > 6)
        {
            theDI.Output.WriteLine($"Usage: {theArgs[0]} Shape1 Shape2 [-tol <value> | -value] [-profile]");
            return 1;
        }

        Shape aShape1 = theDI.GetShape(theArgs[1]);
        Shape aShape2 = theDI.GetShape(theArgs[2]);
        if (aShape1.IsNull || aShape2.IsNull)
        {
            theDI.Output.WriteLine("Error: Failed to find specified shapes");
            return 1;
        }

        var aTool = new ShapeProximity(0.0);
        bool aProfile = false;
        bool isTolerance = false;
        bool isValue = false;
        for (int anArgIdx = 3; anArgIdx < theNbArgs; ++anArgIdx)
        {
            string aFlag = theArgs[anArgIdx].ToLowerInvariant();
            if (aFlag == "-tol")
            {
                isTolerance = true;
                if (++anArgIdx >= theNbArgs)
                {
                    theDI.Output.WriteLine($"Error: wrong syntax at argument '{aFlag}");
                    return 1;
                }

                double aTolerance = theDI.Atof(theArgs[anArgIdx]);
                if (aTolerance < 0.0)
                {
                    theDI.Output.WriteLine("Error: Tolerance value should be non-negative");
                    return 1;
                }

                aTool.SetTolerance(aTolerance);
            }
            else if (aFlag == "-value")
            {
                isValue = true;
                aTool.SetTolerance(global::Kerf.Engine.Foundation.Precision.Infinite());
            }
            else if (aFlag == "-profile")
            {
                aProfile = true;
            }
        }

        if (isTolerance && isValue)
        {
            theDI.Output.WriteLine("Error: Proximity value could not be computed if the tolerance is set");
            return 1;
        }

        var aTimer = Stopwatch.StartNew();
        aTool.LoadShape1(aShape1);
        aTool.LoadShape2(aShape2);
        double aInitTime = aTimer.Elapsed.TotalSeconds;
        aTimer.Restart();
        aTool.Perform();
        double aWorkTime = aTimer.Elapsed.TotalSeconds;

        if (!aTool.IsDone)
        {
            theDI.Output.WriteLine("Error: Failed to perform proximity test");
            return 1;
        }

        if (aProfile)
        {
            theResult.Append("Number of primitives in shape 1: ").Append(aTool.ElementSet1().Size()).Append("\n");
            theResult.Append("Number of primitives in shape 2: ").Append(aTool.ElementSet2().Size()).Append("\n");
            theResult.Append("Building data structures: ").Append(aInitTime).Append("\n");
            theResult.Append("Executing proximity test: ").Append(aWorkTime).Append("\n");
        }

        if (isValue)
        {
            theResult.Append("Proximity value: ").Append(aTool.Proximity()).Append("\n");
            theDI.SetShape("ProxPnt1", new LibraryMakeVertex(aTool.ProximityPoint1()).Vertex());
            theDI.SetShape("ProxPnt2", new LibraryMakeVertex(aTool.ProximityPoint2()).Vertex());
            theResult.Append(" Status of ProxPnt1 on ").Append(theArgs[1]).Append(" : ").Append(Status(aTool.ProxPntStatus1())).Append("\n");
            theResult.Append(" Status of ProxPnt2 on ").Append(theArgs[2]).Append(" : ").Append(Status(aTool.ProxPntStatus2())).Append("\n");
        }
        else
        {
            var aCompBuilder = new Builder();
            Compound aFaceCompound1 = new();
            aCompBuilder.MakeCompound(ref aFaceCompound1);
            foreach (int aKey in Keys(aTool.OverlapSubShapes1()))
            {
                string aStr = $"{theArgs[1]}_{aKey + 1}";
                Shape aShape = aTool.GetSubShape1(aKey);
                aCompBuilder.Add(aFaceCompound1, aShape);
                theDI.SetShape(aStr, aShape);
                theResult.Append(aStr).Append(" \n");
            }

            Compound aFaceCompound2 = new();
            aCompBuilder.MakeCompound(ref aFaceCompound2);
            foreach (int aKey in Keys(aTool.OverlapSubShapes2()))
            {
                string aStr = $"{theArgs[2]}_{aKey + 1}";
                Shape aShape = aTool.GetSubShape2(aKey);
                aCompBuilder.Add(aFaceCompound2, aShape);
                theDI.SetShape(aStr, aShape);
                theResult.Append(aStr).Append(" \n");
            }

            theDI.SetShape(theArgs[1] + "_overlapped", aFaceCompound1);
            theDI.SetShape(theArgs[2] + "_overlapped", aFaceCompound2);
        }

        return 0;
    }

    private static string Status(ProximityDistTool.ProxPnt_Status theStatus) => theStatus switch
    {
        ProximityDistTool.ProxPnt_Status.ProxPnt_Status_BORDER => "Border",
        ProximityDistTool.ProxPnt_Status.ProxPnt_Status_MIDDLE => "Middle",
        _ => "Unknown",
    };

    private static int ShapeSelfIntersection(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgs)
    {
        int theNbArgs = theArgs.Count;
        if (theNbArgs < 2 || theNbArgs > 5)
        {
            theDI.Output.WriteLine($"Usage: {theArgs[0]} Shape [-tol <value>] [-profile]");
            return 1;
        }

        Shape aShape = theDI.GetShape(theArgs[1]);
        if (aShape.IsNull)
        {
            theDI.Output.WriteLine("Error: Failed to find specified shape");
            return 1;
        }

        double aTolerance = 0.0;
        bool aToProfile = false;
        for (int anArgIdx = 2; anArgIdx < theNbArgs; ++anArgIdx)
        {
            string aFlag = theArgs[anArgIdx].ToLowerInvariant();
            if (aFlag == "-tol")
            {
                if (++anArgIdx >= theNbArgs)
                {
                    theDI.Output.WriteLine($"Error: wrong syntax at argument '{aFlag}");
                    return 1;
                }

                double aValue = theDI.Atof(theArgs[anArgIdx]);
                if (aValue < 0.0)
                {
                    theDI.Output.WriteLine("Error: Tolerance value should be non-negative");
                    return 1;
                }

                aTolerance = aValue;
            }

            if (aFlag == "-profile")
            {
                aToProfile = true;
            }
        }

        var aTimer = Stopwatch.StartNew();
        var aTool = new SelfIntersection(aShape, aTolerance);
        double aInitTime = aTimer.Elapsed.TotalSeconds;
        aTimer.Restart();
        aTool.Perform();
        if (!aTool.IsDone)
        {
            theDI.Output.WriteLine("Error: Failed to perform proximity test");
            return 1;
        }

        if (aToProfile)
        {
            double aWorkTime = aTimer.Elapsed.TotalSeconds;
            theResult.Append("Building data structure (BVH):    ").Append(aInitTime).Append("\n");
            theResult.Append("Executing self-intersection test: ").Append(aWorkTime).Append("\n");
        }

        var aCompBuilder = new Builder();
        Compound aFaceCompound = new();
        aCompBuilder.MakeCompound(ref aFaceCompound);
        foreach (int aKey in Keys(aTool.OverlapElements()))
        {
            string aStr = $"{theArgs[1]}_{aKey + 1}";
            Face aFace = aTool.GetSubShape(aKey);
            aCompBuilder.Add(aFaceCompound, aFace);
            theDI.SetShape(aStr, aFace);
            theResult.Append(aStr).Append(" \n");
        }

        theResult.Append("Compound of overlapped sub-faces: ").Append(theArgs[1]).Append("_overlapped\n");
        theDI.SetShape(theArgs[1] + "_overlapped", aFaceCompound);
        return 0;
    }
}
