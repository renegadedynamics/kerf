// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The results of an exact hidden-line removal (HLRBRep_Algo, after <c>Hide</c>) as compounds of edges: visible or
/// hidden, of one kind (sharp edges, smooth edges, sewn edges, outlines, isolines), in the projection plane (2D
/// edges, the default) or on the 3D edges, for all the shapes or one of them (or one of its sub-shapes).
/// </summary>
/// <remarks>
/// <para>
/// Each compound is built anew and is null when it would be empty. The public methods publish it frozen; ported code
/// that edits it (HLRAppli_ReflectLines) reads <see cref="InternalCompoundOfEdges(Shape, TypeOfResultingEdge, bool, bool)"/>.
/// Building a compound marks the data structure's edges as used, and scales its projector while it runs, as OCCT does.
/// </para>
/// <para>
/// A sub-shape S of a loaded shape is found in the data structure as itself, as in OCCT: the outliner works on the
/// caller's shape in a working overlay (unit 10H6).
/// </para>
/// </remarks>
internal class HlrToShape
{
    private readonly HiddenLineAlgo _algo;

    public HlrToShape(HiddenLineAlgo a)
    {
        _algo = a;
    }

    /// <summary>The visible sharp edges.</summary>
    public Shape VCompound() => Published(InternalCompound(5, true, new Shape()));

    public Shape VCompound(in Shape s) => Published(InternalCompound(5, true, s));

    /// <summary>The visible smooth edges (G1 between two faces).</summary>
    public Shape Rg1LineVCompound() => Published(InternalCompound(3, true, new Shape()));

    public Shape Rg1LineVCompound(in Shape s) => Published(InternalCompound(3, true, s));

    /// <summary>The visible sewn edges (CN on one surface).</summary>
    public Shape RgNLineVCompound() => Published(InternalCompound(4, true, new Shape()));

    public Shape RgNLineVCompound(in Shape s) => Published(InternalCompound(4, true, s));

    /// <summary>The visible outlines (silhouettes).</summary>
    public Shape OutLineVCompound() => Published(InternalCompound(2, true, new Shape()));

    /// <summary>The visible outlines and edges, in 3D.</summary>
    public Shape OutLineVCompound3d() => Published(InternalCompound(2, true, new Shape(), true));

    public Shape OutLineVCompound(in Shape s) => Published(InternalCompound(2, true, s));

    /// <summary>The visible isolines.</summary>
    public Shape IsoLineVCompound() => Published(InternalCompound(1, true, new Shape()));

    public Shape IsoLineVCompound(in Shape s) => Published(InternalCompound(1, true, s));

    /// <summary>The hidden sharp edges.</summary>
    public Shape HCompound() => Published(InternalCompound(5, false, new Shape()));

    public Shape HCompound(in Shape s) => Published(InternalCompound(5, false, s));

    /// <summary>The hidden smooth edges.</summary>
    public Shape Rg1LineHCompound() => Published(InternalCompound(3, false, new Shape()));

    public Shape Rg1LineHCompound(in Shape s) => Published(InternalCompound(3, false, s));

    /// <summary>The hidden sewn edges.</summary>
    public Shape RgNLineHCompound() => Published(InternalCompound(4, false, new Shape()));

    public Shape RgNLineHCompound(in Shape s) => Published(InternalCompound(4, false, s));

    /// <summary>The hidden outlines.</summary>
    public Shape OutLineHCompound() => Published(InternalCompound(2, false, new Shape()));

    public Shape OutLineHCompound(in Shape s) => Published(InternalCompound(2, false, s));

    /// <summary>The hidden isolines.</summary>
    public Shape IsoLineHCompound() => Published(InternalCompound(1, false, new Shape()));

    public Shape IsoLineHCompound(in Shape s) => Published(InternalCompound(1, false, s));

    /// <summary>The visible or hidden edges of a type, in 2D or in 3D.</summary>
    public Shape CompoundOfEdges(TypeOfResultingEdge type, bool visible, bool in3d)
        => Published(InternalCompound((int)type, visible, new Shape(), in3d));

    /// <summary>The visible or hidden edges of a type of the shape S, in 2D or in 3D.</summary>
    public Shape CompoundOfEdges(in Shape s, TypeOfResultingEdge type, bool visible, bool in3d)
        => Published(InternalCompound((int)type, visible, s, in3d));

    /// <summary><see cref="CompoundOfEdges(in Shape, TypeOfResultingEdge, bool, bool)"/> without freezing, for ported code.</summary>
    internal Shape InternalCompoundOfEdges(Shape s, TypeOfResultingEdge type, bool visible,
                                                  bool in3d)
        => InternalCompound((int)type, visible, s, in3d);

    private static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    private Shape InternalCompound(int typ, bool visible, in Shape s, bool in3d = false)
    {
        bool added = false;
        Shape result = new();
        HiddenLineData? ds = _algo.DataStructure();

        if (ds is not null)
        {
            ds.Projector().Scaled(true);
            int e1 = 1;
            int e2 = ds.EdgeCount;
            int f1 = 1;
            int f2 = ds.FaceCount;
            bool explor = false;
            //    bool todraw;
            if (!s.IsNull)
            {
                int v1, v2;
                int index = _algo.Index(s);
                if (index == 0)
                {
                    explor = true;
                }
                else
                {
                    _algo.ShapeBounds(index).Bounds(out v1, out v2, out e1, out e2, out f1, out f2);
                }
            }
            BrepBuilder b = new();
            Compound compound = new();
            b.MakeCompound(ref compound);
            result = compound;
            NCollection_Array1<HiddenLineEdgeData> eData = ds.EDataArray();

            for (int ie = e1; ie <= e2; ie++)
            {
                HiddenLineEdgeData ed = eData[ie];
                if (ed.Selected() && !ed.Vertical())
                {
                    ed.Used(false);
                    ed.HideCount(0);
                }
                else
                {
                    ed.Used(true);
                }
            }
            if (explor)
            {
                IndexedSet<Shape> edges = ds.EdgeMap();
                IndexedSet<Shape> faces = ds.FaceMap();
                Explorer exp = new();

                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    int iface = faces.IndexOf(exp.Current()) + 1;
                    if (iface != 0)
                    {
                        DrawFace(visible, typ, iface, ds, ref result, ref added, in3d);
                    }
                }
                if (typ >= 3)
                {
                    for (exp.Init(s, ShapeType.Edge, ShapeType.Face); exp.More(); exp.Next())
                    {
                        int ie = edges.IndexOf(exp.Current()) + 1;
                        if (ie != 0)
                        {
                            HiddenLineEdgeData eDataIE = ds.EDataArray()[ie];
                            if (!eDataIE.Used())
                            {
                                DrawEdge(visible, false, typ, eDataIE, ref result, ref added, in3d);
                                eDataIE.Used(true);
                            }
                        }
                    }
                }
            }
            else
            {
                for (int iface = f1; iface <= f2; iface++)
                {
                    DrawFace(visible, typ, iface, ds, ref result, ref added, in3d);
                }

                if (typ >= 3)
                {
                    for (int ie = e1; ie <= e2; ie++)
                    {
                        HiddenLineEdgeData eDataE11 = eData[ie];
                        if (!eDataE11.Used())
                        {
                            DrawEdge(visible, false, typ, eDataE11, ref result, ref added, in3d);
                            eDataE11.Used(true);
                        }
                    }
                }
            }
            ds.Projector().Scaled(false);
        }
        if (!added)
        {
            result = new Shape();
        }
        return result;
    }

    private void DrawFace(bool visible, int typ, int iface, HiddenLineData ds, ref Shape result, ref bool added,
                          bool in3d = false)
    {
        FaceIterator itf = new();

        for (itf.InitEdge(ds.FDataArray()[iface]); itf.MoreEdge(); itf.NextEdge())
        {
            int ie = itf.Edge();
            HiddenLineEdgeData edf = ds.EDataArray()[ie];
            if (!edf.Used())
            {
                bool todraw;
                if (typ == 1)
                {
                    todraw = itf.IsoLine();
                }
                else if (typ == 2) // outlines
                {
                    if (in3d)
                    {
                        todraw = itf.Internal() || itf.OutLine();
                    }
                    else
                    {
                        todraw = itf.Internal();
                    }
                }
                else if (typ == 3)
                {
                    todraw = edf.Rg1Line() && !edf.RgNLine() && !itf.OutLine();
                }
                else if (typ == 4)
                {
                    todraw = edf.RgNLine() && !itf.OutLine();
                }
                else
                {
                    todraw = !itf.IsoLine() && !itf.Internal() && (!edf.Rg1Line() || itf.OutLine());
                }

                if (todraw)
                {
                    DrawEdge(visible, true, typ, edf, ref result, ref added, in3d);
                    edf.Used(true);
                }
                else
                {
                    if ((typ > 4 || typ == 2) && // sharp or outlines
                        (edf.Rg1Line() && !itf.OutLine()))
                    {
                        int hc = edf.HideCount();
                        if (hc > 0)
                        {
                            edf.Used(true);
                        }
                        else
                        {
                            ++hc;
                            edf.HideCount(hc); // to try with another face
                        }
                    }
                    else
                    {
                        edf.Used(true);
                    }
                }
            }
        }
    }

    private static void DrawEdge(bool visible, bool inFace, int typ, HiddenLineEdgeData ed, ref Shape result,
                                 ref bool added, bool in3d = false)
    {
        bool todraw = false;
        if (inFace)
        {
            todraw = true;
        }
        else if (typ == 3)
        {
            todraw = ed.Rg1Line() && !ed.RgNLine();
        }
        else if (typ == 4)
        {
            todraw = ed.RgNLine();
        }
        else
        {
            todraw = !ed.Rg1Line();
        }

        if (todraw)
        {
            double sta, end;
            float tolsta, tolend;
            BrepBuilder b = new();
            Edge e;
            EdgeIterator it = new();
            if (visible)
            {
                for (it.InitVisible(ed.Status()); it.MoreVisible(); it.NextVisible())
                {
                    it.Visible(out sta, out tolsta, out end, out tolend);
                    if (!in3d)
                    {
                        e = HlrbRep.MakeEdge(ed.Geometry(), sta, end);
                    }
                    else
                    {
                        e = HlrbRep.MakeEdge3d(ed.Geometry(), sta, end);
                    }
                    if (!e.IsNull)
                    {
                        b.Add(result, e);
                        added = true;
                    }
                }
            }
            else
            {
                for (it.InitHidden(ed.Status()); it.MoreHidden(); it.NextHidden())
                {
                    it.Hidden(out sta, out tolsta, out end, out tolend);
                    if (!in3d)
                    {
                        e = HlrbRep.MakeEdge(ed.Geometry(), sta, end);
                    }
                    else
                    {
                        e = HlrbRep.MakeEdge3d(ed.Geometry(), sta, end);
                    }
                    if (!e.IsNull)
                    {
                        b.Add(result, e);
                        added = true;
                    }
                }
            }
        }
    }
}
