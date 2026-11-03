// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BRepTest_BasicCommands.cxx that modify or compare shapes with BRepBuilderAPI and BRepLib:
/// nurbsconvert, deform, scalexyz, sameparameter, fsameparameter, updatetolerance, solidorientation, mkedgecurve,
/// findplane, compare, issubshape and wexplo; and nproject (BRepOffsetAPI_NormalProjection, unit 7G).
/// </summary>
/// <remarks>
/// sameparameter, updatetolerance and mkedgecurve change their shape in place when no result name is given; the
/// port runs OCCT's in-place algorithm (BRepLib's internal forms) on a copy of the graph (DrawShapeCopy) and binds it to the
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
        DrawCxx.Add(theInterpreter, "nproject", NProject);
    }

    // nproject pj e1 e2 e3 ... surf -g -d [dmax] [Tol [continuity [maxdeg [maxseg]]]
    private static int NProject(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }
        Shape InpShape;
        int arg = 2, i;
        NCollection_Sequence<Shape> Args = new();

        double Tol = 1.0e-4;
        double Tol2d;
        double MaxDistance = 1.0e-3;
        Continuity Continuity = global::Kerf.Geometry.Continuity.C1;
        int MaxDeg = 14;
        int MaxSeg = 16;

        while ((n > arg) && !(InpShape = theDI.GetShape(a[arg])).IsNull)
        {
            Args.Append(InpShape);
            arg++;
        }
        if (Args.Length() < 2)
        {
            return 1;
        }

        BRepOffsetAPI_NormalProjection OrtProj = new(Args.Last());

        for (i = 1; i < Args.Length(); i++)
        {
            OrtProj.Add(Args[i]);
        }

        if (n > arg)
        {
            if (a[arg] == "-g")
            {
                OrtProj.SetLimit(false);
                arg++;
            }
        }

        if (n > arg)
        {
            if (a[arg] == "-d")
            {
                arg++;
                if (n > arg)
                {
                    MaxDistance = theDI.Atof(a[arg++]);
                }
                OrtProj.SetMaxDistance(MaxDistance);
            }
        }
        if (n > arg)
        {
            Tol = Math.Max(theDI.Atof(a[arg++]), 1.0e-10);
        }

        if (n > arg)
        {
            if (theDI.Atoi(a[arg]) == 0)
            {
                Continuity = global::Kerf.Geometry.Continuity.C0;
            }
            else if (theDI.Atoi(a[arg]) == 2)
            {
                Continuity = global::Kerf.Geometry.Continuity.C2;
            }
            arg++;
        }

        if (n > arg)
        {
            MaxDeg = theDI.Atoi(a[arg++]);
            if (MaxDeg < 1 || MaxDeg > 14)
            {
                MaxDeg = 14;
            }
        }

        if (n > arg)
        {
            MaxSeg = theDI.Atoi(a[arg]);
        }

        Tol2d = Math.Pow(Tol, 2.0 / 3);

        OrtProj.SetParams(Tol, Tol2d, Continuity, MaxDeg, MaxSeg);
        OrtProj.Build();
        NCollection_List<Shape> Wire = new();
        bool IsWire = OrtProj.BuildWire(Wire);
        if (IsWire)
        {
            di.Append(" BuildWire OK \n");
        }
        theDI.SetShape(a[1], OrtProj.Shape());
        return 0;
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
            Shape S = theDI.GetShape(a[2 * i + 2]);
            if (S.IsNull)
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

        var GT = new GeneralTransform3D(new Transform3D());
        var rot = new Matrix3x3(theDI.Atof(a[3]), 0, 0, 0, theDI.Atof(a[4]), 0, 0, 0, theDI.Atof(a[5]));
        GT.SetVectorialPart(rot);
        var gtrf = new BRepBuilderAPI_GTransform(GT);
        Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull)
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

        Shape aShapeBase = theDI.GetShape(a[2]);
        if (aShapeBase.IsNull)
        {
            return 1;
        }

        double aFactorX = theDI.Atof(a[3]);
        double aFactorY = theDI.Atof(a[4]);
        double aFactorZ = theDI.Atof(a[5]);
        var aGTrsf = new GeneralTransform3D();
        var rot = new Matrix3x3(aFactorX, 0, 0, 0, aFactorY, 0, 0, 0, aFactorZ);
        aGTrsf.SetVectorialPart(rot);
        var aBRepGTrsf = new BRepBuilderAPI_GTransform(aShapeBase, aGTrsf, false);
        if (!aBRepGTrsf.IsDone())
        {
            throw new ConstructionErrorException("Scaling not done");
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

        Shape anInpS = theDI.GetShape(IsUseTol ? a[n - 2] : a[n - 1]);
        if (anInpS.IsNull)
        {
            return 1;
        }

        if ((n == 4 && IsUseTol) || (n == 3 && !IsUseTol))
        {
            var aResh = new ReShape();
            BRepLib.SameParameter(anInpS, aResh, aTol, force);
            theDI.SetShape(a[1], aResh.Apply(anInpS));
        }
        else
        {
            // in place in OCCT: on a copy of the graph here
            Shape aCopy = DrawShapeCopy.Copy(anInpS);
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

        Shape aSh1 = theDI.GetShape(a[n - 1]);
        bool IsF = aSh1.IsNull;
        Shape anInpS = IsF ? theDI.GetShape(a[n - 2]) : aSh1;
        if (anInpS.IsNull)
        {
            return 1;
        }

        if ((n == 4 && IsF) || (n == 3 && !IsF))
        {
            var aResh = new ReShape();
            BRepLib.UpdateTolerances(anInpS, aResh, IsF);
            theDI.SetShape(a[1], aResh.Apply(anInpS));
        }
        else
        {
            // in place in OCCT: on a copy of the graph here
            Shape aCopy = DrawShapeCopy.Copy(anInpS);
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

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 1;
        }

        if (S.ShapeType() != ShapeType.Solid)
        {
            return 1;
        }

        Solid aSolid = ShapeCasts.Solid(S);
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
        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 1;
        }

        Shape aCopy = DrawShapeCopy.Copy(S);
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

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
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

        Shape aS1 = theDI.GetShape(a[1]);
        Shape aS2 = theDI.GetShape(a[2]);
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

        Shape aSubShape = theDI.GetShape(a[1]);
        Shape aShape = theDI.GetShape(a[2]);
        if (aSubShape.IsNull || aShape.IsNull)
        {
            di.Append("null shapes\n");
            return 0;
        }

        var aMShapes = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var anExp = new Explorer(aShape, aSubShape.ShapeType());
        for (; anExp.More(); anExp.Next())
        {
            Shape aSS = anExp.Current();
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

        Shape C1 = theDI.GetShape(argv[1], ShapeType.Wire);
        Shape C2 = new();
        if (argc > 2)
        {
            C2 = theDI.GetShape(argv[2], ShapeType.Face);
        }

        if (C1.IsNull)
        {
            return 1;
        }

        var we = new WireExplorer();
        if (C2.IsNull)
        {
            we.Init(ShapeCasts.Wire(C1));
        }
        else
        {
            we.Init(ShapeCasts.Wire(C1), ShapeCasts.Face(C2));
        }

        for (int k = 1; we.More(); we.Next(), k++)
        {
            theDI.SetShape("WEDGE_" + k.ToString(CultureInfo.InvariantCulture), we.Current());
        }

        return 0;
    }
}
