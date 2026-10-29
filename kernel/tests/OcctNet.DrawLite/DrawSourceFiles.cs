// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite;

/// <summary>
/// The Tcl array <c>Draw_Files</c>, which StandardCommands.tcl's <c>getsourcefile</c> reads: Draw_Interpretor::Add
/// sets <c>Draw_Files(command)</c> to the command's C++ source, as <c>__FILE__</c> with its last four directories
/// (<c>src/Draw/TKDraw/Draw/Draw_PloadCommands.cxx</c>).
/// </summary>
/// <remarks>
/// Draw-lite fills it for <c>pload</c> and for the commands of unit 3L2b, whose sources are known here; the commands
/// of earlier units aren't listed yet.
/// </remarks>
internal static class DrawSourceFiles
{
    private static readonly (string File, string[] Commands)[] theFiles =
    [
        ("src/Draw/TKDraw/Draw/Draw_PloadCommands.cxx", ["pload"]),
        ("src/Draw/TKTopTest/GeomliteTest/GeomliteTest_CurveCommands.cxx",
         ["line", "circle", "ellipse", "parabola", "hyperbola", "beziercurve", "bsplinecurve", "pbsplinecurve",
          "2dbeziercurve", "2dbsplinecurve", "2dpbsplinecurve", "reverse", "segment", "setorigin", "cvalue", "2dcvalue",
          "coord", "length", "point"]),
        ("src/Draw/TKTopTest/GeomliteTest/GeomliteTest_SurfaceCommands.cxx",
         ["plane", "cone", "cylinder", "sphere", "torus", "beziersurf", "bsplinesurf", "upbsplinesurf", "vpbsplinesurf",
          "uvpbsplinesurf", "extsurf", "revsurf", "offset", "trim", "trimu", "trimv", "ureverse", "vreverse", "uiso",
          "viso", "svalue", "sderivative", "parameters", "bounds"]),
        ("src/Draw/TKTopTest/GeometryTest/GeometryTest_ConstraintCommands.cxx", ["gcarc"]),
        ("src/Draw/TKTopTest/GeometryTest/GeometryTest_APICommands.cxx", ["extrema"]),
        ("src/Draw/TKDraw/DrawTrSurf/DrawTrSurf.cxx",
         ["translate", "rotate", "pmirror", "lmirror", "smirror", "pscale", "2dtranslate", "2drotate", "2dpmirror",
          "2dlmirror", "2dpscale"]),
        ("src/Draw/TKTopTest/BRepTest/BRepTest_ExtremaCommands.cxx", ["dist", "distmini", "proximity", "selfintersect"]),
        ("src/Draw/TKTopTest/BRepTest/BRepTest_CurveCommands.cxx",
         ["vertex", "edge", "mkedge", "wire", "polyline", "polyvertex", "mkcurve", "mkoricurve", "mkpoint", "mk2dcurve",
          "uisoedge", "visoedge", "profile"]),
        ("src/Draw/TKTopTest/BRepTest/BRepTest_SurfaceCommands.cxx",
         ["mkface", "mkshell", "mksurface", "mkplane", "pcurve", "encoderegularity", "getedgeregularity", "projponf"]),
        ("src/Draw/TKTopTest/BRepTest/BRepTest_BasicCommands.cxx",
         ["nurbsconvert", "deform", "scalexyz", "sameparameter", "fsameparameter", "updatetolerance", "solidorientation",
          "mkedgecurve", "findplane", "compare", "issubshape", "wexplo"]),
    ];

    public static void Register(DrawInterpreter theInterpreter)
    {
        foreach ((string aFile, string[] aCommands) in theFiles)
        {
            foreach (string aCommand in aCommands)
            {
                theInterpreter.SetVariable("Draw_Files", aCommand, aFile);
            }
        }
    }
}
