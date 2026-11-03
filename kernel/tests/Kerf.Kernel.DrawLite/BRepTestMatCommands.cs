// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Offsets.MedialAxis;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;
using static Kerf.Geometry.JoinKind;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest_MatCommands.cxx: the bisecting locus of a face. <c>topoload face</c> reads its contours, <c>side
/// left|right</c> chooses the side, <c>mat [i [o]]</c> computes the locus (Intersection joins, open result),
/// <c>result</c> and <c>zone shape</c> go through its bisectors, <c>drawcont</c> through the contours. OCCT keeps the
/// explorer, locus, links and side in file statics; Draw-lite keeps them per interpreter. Draw's commands only draw
/// curves (DrawCurve), which Draw-lite has no viewer for: the curves are computed and not drawn.
/// </summary>
internal static class BRepTestMatCommands
{
    private sealed class State
    {
        public BisectingLocus MapBiLo = new();
        public MedialAxisExplorer anExplo = new();
        public LinkTopoBilo TopoBilo = new();
        public MedialSide SideOfMat = MedialSide.Left;
        public bool LinkComputed;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, State> s_state = new();

    private static State Get(DrawInterpreter theDI) => s_state.GetOrCreateValue(theDI);

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "topoload", Topoload);
        DrawCxx.Add(theInterpreter, "drawcont", Drawcont);
        DrawCxx.Add(theInterpreter, "mat", Mat);
        DrawCxx.Add(theInterpreter, "side", Side);
        DrawCxx.Add(theInterpreter, "result", Result);
        DrawCxx.Add(theInterpreter, "zone", Zone);
    }

    // topoload face [-approx]. The -approx form converts the face with BRepOffsetAPI_MakeOffset::ConvertFace.
    private static int Topoload(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            return 1;
        }
        Shape C1 = theDI.GetShape(argv[1], ShapeType.Face);

        if (C1.IsNull)
        {
            return 1;
        }

        Face aFace = ShapeCasts.Face(C1);

        if (argc >= 3 && argv[2] == "-approx")
        {
            double aTol = 0.1;
            aFace = MakeWireOffset.ConvertFace(aFace, aTol);
        }

        Get(theDI).anExplo.Perform(aFace);
        return 0;
    }

    private static int Drawcont(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        MedialAxisExplorer anExplo = Get(theDI).anExplo;
        for (int j = 1; j <= anExplo.NumberOfContours(); j++)
        {
            for (anExplo.Init(j); anExplo.More(); anExplo.Next())
            {
                // DrawCurve(anExplo.Value(), 1): nothing to draw on.
            }
        }
        return 0;
    }

    // mat [i [o]]: the locus on the chosen side, with Intersection joins if i, an open result if o.
    private static int Mat(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        State aState = Get(theDI);
        JoinKind theJoinType = Arc;
        if (n >= 2 && a[1] == "i")
        {
            theJoinType = Intersection;
        }

        bool IsOpenResult = false;
        if (n == 3 && a[2] == "o")
        {
            IsOpenResult = true;
        }

        aState.MapBiLo.Compute(aState.anExplo, 1, aState.SideOfMat, theJoinType, IsOpenResult);
        aState.LinkComputed = false;

        return 0;
    }

    // zone edge|vertex: the frontiers of the zones of the basic elements of the shape.
    private static int Zone(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            return 1;
        }
        State aState = Get(theDI);

        Shape S = theDI.GetShape(argv[1], ShapeType.Edge);
        if (S.IsNull)
        {
            S = theDI.GetShape(argv[1], ShapeType.Vertex);
        }

        if (!aState.LinkComputed)
        {
            aState.TopoBilo.Perform(aState.anExplo, aState.MapBiLo);
            aState.LinkComputed = true;
        }

        MedialZone TheZone = new();

        for (aState.TopoBilo.Init(S); aState.TopoBilo.More(); aState.TopoBilo.Next())
        {
            MedialBasicElt BE = aState.TopoBilo.Value();
            TheZone.Perform(BE);
            for (int i = 1; i <= TheZone.NumberOfArcs(); i++)
            {
                // DrawCurve(MapBiLo.GeomBis(...).Value(), 2): computed, nothing to draw on.
                aState.MapBiLo.GeomBis(TheZone.ArcOnFrontier(i), out _);
            }
        }
        return 0;
    }

    // side left|right (anything but right is left).
    private static int Side(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        Get(theDI).SideOfMat = argv[1] == "right" ? MedialSide.Right : MedialSide.Left;
        return 0;
    }

    // result: every bisector of the locus.
    private static int Result(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        BisectingLocus MapBiLo = Get(theDI).MapBiLo;
        int NbArcs = MapBiLo.Graph().NumberOfArcs();

        for (int i = 1; i <= NbArcs; i++)
        {
            // DrawCurve(MapBiLo.GeomBis(...).Value(), 3): computed, nothing to draw on.
            MapBiLo.GeomBis(MapBiLo.Graph().Arc(i), out _);
        }
        return 0;
    }
}
