// Port of: src/Draw/TKTopTest/BRepTest/BRepTest_ProjectionCommands.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest_ProjectionCommands.cxx (unit 5W): <c>prj</c> and <c>cprj</c>, the cylindrical and conical projections of a
/// wire or edge on a shape's faces (BRepProj_Projection).
/// </summary>
internal static class BRepTestProjectionCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "prj", prj);
        DrawCxx.Add(theInterpreter, "cprj", prj);
    }

    // prj: Draw command for Conical and Cylindrical projection
    private static int prj(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 7)
        {
            // di.PrintHelp(a[0]): Draw-lite has no help texts
            return 1;
        }
        //
        Shape anInputWire = theDI.GetShape(a[2]);
        Shape anInputShape = theDI.GetShape(a[3]);
        if (anInputWire.IsNull || anInputShape.IsNull)
        {
            di.Append("Null input shapes\n");
            return 1;
        }
        //
        double X = theDI.Atof(a[4]), Y = theDI.Atof(a[5]), Z = theDI.Atof(a[6]);
        //
        bool bCylProj = a[0] == "prj";
        //
        Projection aPrj = bCylProj
                                       ? new Projection(anInputWire, anInputShape, new Direction3D(X, Y, Z))
                                       : new Projection(anInputWire, anInputShape, new Point3D(X, Y, Z));
        //
        if (!aPrj.IsDone)
        {
            di.Append("Not done\n");
            return 0;
        }
        //
        for (int i = 1; aPrj.More(); aPrj.Next(), ++i)
        {
            string name = $"{a[1]}_{i}";
            theDI.SetShape(name, aPrj.Current());
            di.Append(name).Append(" ");
        }
        //
        di.Append("\n");
        return 0;
    }
}
