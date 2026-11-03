// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BOPTest_UtilityCommands.cxx: <c>attachpcurve</c>, <c>edgestowire</c>, <c>edgestofaces</c> and
/// <c>buildpcurvesonplane</c> (unit 6L).
/// </summary>
/// <remarks>
/// <c>attachpcurve</c> and <c>buildpcurvesonplane</c> write pcurves into the edges they are given, which Draw's variables
/// hold frozen; they run as the healing commands do (<see cref="SWDRAWShapeFixCommands.InPlace"/>): on an unfrozen copy,
/// with every variable sharing a changed sub-shape rebound.
/// </remarks>
internal static class BOPTestUtilityCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "attachpcurve", AttachPCurve);
        DrawCxx.Add(theInterpreter, "edgestowire", EdgesToWire);
        DrawCxx.Add(theInterpreter, "edgestofaces", EdgesToFaces);
        DrawCxx.Add(theInterpreter, "buildpcurvesonplane", BuildPcurvesOnPlane);
    }

    /// <summary><c>attachpcurve eold enew face</c>: copies the old edge's pcurve on the face to the new edge.</summary>
    private static int AttachPCurve(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        if (theArgVal.Count != 4)
        {
            theResult.Append("Use: ").Append(theArgVal[0]).Append(" eold enew face\n");
            return 1;
        }

        Shape aShEOld = theDI.GetShape(theArgVal[1]);
        Shape aShENew = theDI.GetShape(theArgVal[2]);
        Shape aShFace = theDI.GetShape(theArgVal[3]);

        if (aShEOld.IsNull)
        {
            theResult.Append(theArgVal[1]).Append(" is null shape\n");
            return 1;
        }
        else if (aShEOld.ShapeType() != ShapeType.Edge)
        {
            theResult.Append(theArgVal[1]).Append(" is not an edge\n");
            return 1;
        }

        if (aShENew.IsNull)
        {
            theResult.Append(theArgVal[2]).Append(" is null shape\n");
            return 1;
        }
        else if (aShENew.ShapeType() != ShapeType.Edge)
        {
            theResult.Append(theArgVal[2]).Append(" is not an edge\n");
            return 1;
        }

        if (aShFace.IsNull)
        {
            theResult.Append(theArgVal[3]).Append(" is null shape\n");
            return 1;
        }
        else if (aShFace.ShapeType() != ShapeType.Face)
        {
            theResult.Append(theArgVal[3]).Append(" is not a face\n");
            return 1;
        }

        Edge aEOld = ShapeCasts.Edge(aShEOld);
        Face aFace = ShapeCasts.Face(aShFace);

        // Try to copy PCurve from old edge to the new one.
        IntersectionContext aCtx = new();
        int iRet = 0;
        // OCCT writes into the new edge itself: Draw-lite into an unfrozen copy, rebound afterwards (see the class)
        SWDRAWShapeFixCommands.InPlace(theDI, aShENew, aWork =>
        {
            iRet = AlgoTools2D.AttachExistingPCurve(aEOld, ShapeCasts.Edge(aWork), aFace, aCtx);
            return aWork;
        });

        if (iRet != 0)
        {
            theResult.Append("Error! Code: ").Append(iRet).Append("\n");
        }
        else
        {
            theResult.Append("PCurve is attached successfully\n");
        }

        return 0;
    }

    /// <summary><c>edgestowire wire edges</c>: orients the edges to make a wire.</summary>
    private static int EdgesToWire(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        if (theArgVal.Count != 3)
        {
            theResult.Append("Use: edgestowire wire edges\n");
            return 1;
        }
        //
        Shape anEdges = theDI.GetShape(theArgVal[2]);
        if (anEdges.IsNull)
        {
            theResult.Append("no edges\n");
            return 1;
        }
        //
        AlgoTools.OrientEdgesOnWire(ref anEdges);
        theDI.SetShape(theArgVal[1], anEdges);
        return 0;
    }

    /// <summary><c>edgestofaces faces edges [-a AngTol -s Shared(0/1)]</c>: planar faces from linear edges.</summary>
    private static int EdgesToFaces(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        int theNArg = theArgVal.Count;
        if (theNArg < 3)
        {
            theResult.Append("Use: edgestofaces faces edges [-a AngTol -s Shared(0/1)]\n");
            theResult.Append(" AngTol - angular tolerance for comparing the planes;\n");
            theResult.Append(" Shared - boolean flag which defines whether the input\n");
            theResult.Append("          edges are already shared or have to be intersected.\n");
            return 1;
        }
        //
        Shape anEdges = theDI.GetShape(theArgVal[2]);
        if (anEdges.IsNull)
        {
            theResult.Append("no edges\n");
            return 1;
        }
        //
        double anAngTol = 1.0e-8;
        bool bShared = false;
        //
        for (int i = 3; i < theNArg; ++i)
        {
            if (theArgVal[i] == "-a" && (i + 1 < theNArg))
            {
                anAngTol = theDI.Atof(theArgVal[i + 1]);
            }
            if (theArgVal[i] == "-s" && (i + 1 < theNArg))
            {
                bShared = theDI.Atoi(theArgVal[i + 1]) == 1;
            }
        }
        //
        Shape aWires = new();
        int iErr = BuilderTools.EdgesToWires(anEdges, ref aWires, bShared, anAngTol);
        if (iErr != 0)
        {
            theResult.Append("Unable to build wires from given edges\n");
            return 0;
        }
        //
        Shape aFaces = new();
        bool bDone = BuilderTools.WiresToFaces(aWires, ref aFaces, anAngTol);
        if (!bDone)
        {
            theResult.Append("Unable to build faces from wires\n");
            return 0;
        }
        //
        theDI.SetShape(theArgVal[1], aFaces);
        return 0;
    }

    /// <summary><c>buildpcurvesonplane shape</c>: builds and stores the pcurves of the edges of the planar faces.</summary>
    private static int BuildPcurvesOnPlane(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVal)
    {
        if (theArgVal.Count != 2)
        {
            theResult.Append("Use: ").Append(theArgVal[0]).Append(" shape\n");
            return 1;
        }

        Shape aShape = theDI.GetShape(theArgVal[1]);
        if (aShape.IsNull)
        {
            theResult.Append(theArgVal[1]).Append(" is null shape\n");
            return 1;
        }

        // OCCT writes into the shape itself: Draw-lite into an unfrozen copy, rebound afterwards (see the class)
        SWDRAWShapeFixCommands.InPlace(theDI, aShape, aWork =>
        {
            Explorer exp = new(aWork, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face aF = ShapeCasts.Face(exp.Current());
                FaceSurfaceAdaptor aS = new(aF, false);
                if (aS.GetType() == SurfaceKind.Plane)
                {
                    NCollection_List<Shape> aLE = new();
                    Explorer exp1 = new(aF, ShapeType.Edge);
                    for (; exp1.More(); exp1.Next())
                    {
                        aLE.Append(exp1.Current());
                    }
                    BrepLibrary.BuildPCurveForEdgesOnPlane(aLE, aF);
                }
            }
            return aWork;
        });
        return 0;
    }
}
