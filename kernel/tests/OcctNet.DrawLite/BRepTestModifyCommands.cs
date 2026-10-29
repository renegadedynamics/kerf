// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BRepTest_BasicCommands.cxx that modify or compare shapes with BRepBuilderAPI and BRepLib:
/// nurbsconvert, deform, scalexyz, sameparameter, fsameparameter, updatetolerance, solidorientation, mkedgecurve,
/// findplane, compare, issubshape and wexplo.
/// </summary>
/// <remarks>
/// sameparameter, updatetolerance and mkedgecurve change their shape in place when no result name is given; the
/// port runs OCCT's in-place algorithm (BRepLib's internal forms) on a structural copy and binds the copy to the
/// name, as decision 407 does for <c>add</c> and <c>setflags</c>. <c>precision</c> isn't here: it sets
/// BRepBuilderAPI's process-wide precision, which Draw-lite's parallel runs share.
/// </remarks>
internal static class BRepTestModifyCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "nurbsconvert", NurbsConvert);
        DrawCxx.Add(theInterpreter, "deform", Deform);
        DrawCxx.Add(theInterpreter, "scalexyz", ScaleXYZ);
        DrawCxx.Add(theInterpreter, "sameparameter", SameParameter);
        DrawCxx.Add(theInterpreter, "fsameparameter", SameParameter);
        DrawCxx.Add(theInterpreter, "updatetolerance", UpdateTol);
        DrawCxx.Add(theInterpreter, "solidorientation", OrientSolid);
        DrawCxx.Add(theInterpreter, "mkedgecurve", MkEdgeCurve);
        DrawCxx.Add(theInterpreter, "findplane", FindPlane);
        DrawCxx.Add(theInterpreter, "compare", CompareShapes);
        DrawCxx.Add(theInterpreter, "issubshape", IsSubShape);
        DrawCxx.Add(theInterpreter, "wexplo", WExplo);
    }

    private static int NurbsConvert(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        if ((n - 1) % 2 != 0)
        {
            return 1;
        }

        var nbscv = new BRepBuilderAPI_NurbsConvert();
        for (int i = 0; i < (n - 1) / 2; i++)
        {
            TopoDS_Shape S = theDI.GetShape(a[2 * i + 2]);
            if (S.IsNull())
            {
                di.Append(a[2 * i + 2]).Append(" is not a valid shape\n");
            }
            else
            {
                nbscv.Perform(S);
                if (nbscv.IsDone())
                {
                    theDI.SetShape(a[2 * i + 1], nbscv.Shape());
                }
                else
                {
                    return 1;
                }
            }
        }

        return 0;
    }

    private static int Deform(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 6)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        var GT = new gp_GTrsf(new gp_Trsf());
        var rot = new gp_Mat(theDI.Atof(a[3]), 0, 0, 0, theDI.Atof(a[4]), 0, 0, 0, theDI.Atof(a[5]));
        GT.SetVectorialPart(rot);
        var gtrf = new BRepBuilderAPI_GTransform(GT);
        TopoDS_Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull())
        {
            di.Append("Syntax error: '").Append(a[2]).Append("' is not a valid shape");
            return 1;
        }

        gtrf.Perform(aShape);
        if (!gtrf.IsDone())
        {
            di.Append("Error: transformation failed");
            return 1;
        }

        theDI.SetShape(a[1], gtrf.Shape());
        return 0;
    }

    private static int ScaleXYZ(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 6)
        {
            return 1;
        }

        TopoDS_Shape aShapeBase = theDI.GetShape(a[2]);
        if (aShapeBase.IsNull())
        {
            return 1;
        }

        double aFactorX = theDI.Atof(a[3]);
        double aFactorY = theDI.Atof(a[4]);
        double aFactorZ = theDI.Atof(a[5]);
        var aGTrsf = new gp_GTrsf();
        var rot = new gp_Mat(aFactorX, 0, 0, 0, aFactorY, 0, 0, 0, aFactorZ);
        aGTrsf.SetVectorialPart(rot);
        var aBRepGTrsf = new BRepBuilderAPI_GTransform(aShapeBase, aGTrsf, false);
        if (!aBRepGTrsf.IsDone())
        {
            throw new Standard_ConstructionError("Scaling not done");
        }

        theDI.SetShape(a[1], aBRepGTrsf.Shape());
        return 0;
    }

    private static int SameParameter(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("Use sameparameter [result] shape [toler]\n");
            di.Append("shape is an initial shape\n");
            di.Append("result is a result shape. if skipped = > initial shape will be modified\n");
            di.Append("toler is tolerance (default is 1.e-7)");
            return 1;
        }

        double aTol = 1.0e-7;
        bool force = a[0] == "fsameparameter";
        double aTol1 = theDI.Atof(a[n - 1]);
        bool IsUseTol = aTol1 > 0;
        if (IsUseTol)
        {
            aTol = aTol1;
        }

        TopoDS_Shape anInpS = theDI.GetShape(IsUseTol ? a[n - 2] : a[n - 1]);
        if (anInpS.IsNull())
        {
            return 1;
        }

        if ((n == 4 && IsUseTol) || (n == 3 && !IsUseTol))
        {
            var aResh = new BRepTools_ReShape();
            BRepLib.SameParameter(anInpS, aResh, aTol, force);
            theDI.SetShape(a[1], aResh.Apply(anInpS));
        }
        else
        {
            // in place in OCCT: on a copy here
            TopoDS_Shape aCopy = BRepTools.CopyForWrite(anInpS, out _);
            BRepLib.SameParameter(aCopy, aTol, force);
            theDI.SetShape(a[1], aCopy);
        }

        return 0;
    }

    private static int UpdateTol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("Use updatetololerance [result] shape [param]\n");
            di.Append("shape is an initial shape\n");
            di.Append("result is a result shape. if skipped = > initial shape will be modified\n");
            di.Append("if [param] is absent - not verify of face tolerance, else - perform it");
            return 1;
        }

        TopoDS_Shape aSh1 = theDI.GetShape(a[n - 1]);
        bool IsF = aSh1.IsNull();
        TopoDS_Shape anInpS = IsF ? theDI.GetShape(a[n - 2]) : aSh1;
        if (anInpS.IsNull())
        {
            return 1;
        }

        if ((n == 4 && IsF) || (n == 3 && !IsF))
        {
            var aResh = new BRepTools_ReShape();
            BRepLib.UpdateTolerances(anInpS, aResh, IsF);
            theDI.SetShape(a[1], aResh.Apply(anInpS));
        }
        else
        {
            // in place in OCCT: on a copy here
            TopoDS_Shape aCopy = BRepTools.CopyForWrite(anInpS, out _);
            BRepLib.UpdateTolerances(aCopy, IsF);
            theDI.SetShape(a[1], aCopy);
        }

        return 0;
    }

    /// <summary>solidorientation: OrientClosedSolid changes only the orientation of the value OCCT binds again.</summary>
    private static int OrientSolid(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[1]);
        if (S.IsNull())
        {
            return 1;
        }

        if (S.ShapeType() != TopAbs_ShapeEnum.TopAbs_SOLID)
        {
            return 1;
        }

        TopoDS_Solid aSolid = TopoDS.Solid(S);
        BRepLib.OrientClosedSolid(ref aSolid);
        theDI.SetShape(a[1], aSolid);
        return 0;
    }

    /// <summary>mkedgecurve: OCCT builds the 3D curves in place; the port builds them on a copy bound to the name.</summary>
    private static int MkEdgeCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        double Tolerance = theDI.Atof(a[2]);
        TopoDS_Shape S = theDI.GetShape(a[1]);
        if (S.IsNull())
        {
            return 1;
        }

        TopoDS_Shape aCopy = BRepTools.CopyForWrite(S, out _);
        BRepLib.BuildCurves3d(aCopy, Tolerance);
        theDI.SetShape(a[1], aCopy);
        return 0;
    }

    private static int FindPlane(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[1]);
        if (S.IsNull())
        {
            return 1;
        }

        double tolerance = 1.0e-5;
        var a_plane_finder = new BRepBuilderAPI_FindPlane(S, tolerance);
        if (a_plane_finder.Found())
        {
            di.Append(" a plane is found \n");
            DrawTrSurf.Set(theDI, a[2], a_plane_finder.Plane());
        }

        return 0;
    }

    private static int CompareShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Compare shapes. Usage: compare shape1 shape2\n");
            return 1;
        }

        TopoDS_Shape aS1 = theDI.GetShape(a[1]);
        TopoDS_Shape aS2 = theDI.GetShape(a[2]);
        if (aS1.IsSame(aS2))
        {
            di.Append("same shapes\n");
            if (aS1.IsEqual(aS2))
            {
                di.Append("equal shapes\n");
            }
        }
        else
        {
            di.Append("shapes are not same\n");
        }

        return 0;
    }

    private static int IsSubShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Check if the shape is sub-shape of other shape and get its index in the shape.\n");
            di.Append("Usage: issubshape subshape shape\n");
            return 1;
        }

        TopoDS_Shape aSubShape = theDI.GetShape(a[1]);
        TopoDS_Shape aShape = theDI.GetShape(a[2]);
        if (aSubShape.IsNull() || aShape.IsNull())
        {
            di.Append("null shapes\n");
            return 0;
        }

        var aMShapes = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var anExp = new TopExp_Explorer(aShape, aSubShape.ShapeType());
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Shape aSS = anExp.Current();
            if (aMShapes.Add(aSS) && aSS.IsSame(aSubShape))
            {
                break;
            }
        }

        if (anExp.More())
        {
            di.Append(a[1]).Append(" is sub-shape of ").Append(a[2]).Append(". Index in the shape: ").Append(aMShapes.Extent()).Append(".\n");
        }
        else
        {
            di.Append(a[1]).Append(" is NOT sub-shape of ").Append(a[2]).Append(".\n");
        }

        return 0;
    }

    private static int WExplo(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            return 1;
        }

        TopoDS_Shape C1 = theDI.GetShape(argv[1], TopAbs_ShapeEnum.TopAbs_WIRE);
        TopoDS_Shape C2 = new();
        if (argc > 2)
        {
            C2 = theDI.GetShape(argv[2], TopAbs_ShapeEnum.TopAbs_FACE);
        }

        if (C1.IsNull())
        {
            return 1;
        }

        var we = new BRepTools_WireExplorer();
        if (C2.IsNull())
        {
            we.Init(TopoDS.Wire(C1));
        }
        else
        {
            we.Init(TopoDS.Wire(C1), TopoDS.Face(C2));
        }

        for (int k = 1; we.More(); we.Next(), k++)
        {
            theDI.SetShape("WEDGE_" + k.ToString(CultureInfo.InvariantCulture), we.Current());
        }

        return 0;
    }
}
