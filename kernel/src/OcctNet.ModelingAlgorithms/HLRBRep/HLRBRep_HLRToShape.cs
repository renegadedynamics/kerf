// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_HLRToShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The results of an exact hidden-line removal (HLRBRep_Algo, after <c>Hide</c>) as compounds of edges: visible or
/// hidden, of one kind (sharp edges, smooth edges, sewn edges, outlines, isolines), in the projection plane (2D
/// edges, the default) or on the 3D edges, for all the shapes or one of them (or one of its sub-shapes).
/// </summary>
/// <remarks>
/// <para>
/// Each compound is built anew and is null when it would be empty. The public methods publish it frozen; ported code
/// that edits it (HLRAppli_ReflectLines) reads <see cref="InternalCompoundOfEdges(Shape, HLRBRep_TypeOfResultingEdge, bool, bool)"/>.
/// Building a compound marks the data structure's edges as used, and scales its projector while it runs, as OCCT does.
/// </para>
/// <para>
/// A sub-shape S of a loaded shape is found in the data structure through the outliner's working shape: when the
/// shape was copied at the entry (decision 1095), S's image in the copy (unit 7Q2).
/// </para>
/// </remarks>
public class HLRBRep_HLRToShape
{
    private readonly HLRBRep_Algo myAlgo;

    public HLRBRep_HLRToShape(HLRBRep_Algo A)
    {
        myAlgo = A;
    }

    /// <summary>The visible sharp edges.</summary>
    public Shape VCompound() => Published(InternalCompound(5, true, new Shape()));

    public Shape VCompound(in Shape S) => Published(InternalCompound(5, true, S));

    /// <summary>The visible smooth edges (G1 between two faces).</summary>
    public Shape Rg1LineVCompound() => Published(InternalCompound(3, true, new Shape()));

    public Shape Rg1LineVCompound(in Shape S) => Published(InternalCompound(3, true, S));

    /// <summary>The visible sewn edges (CN on one surface).</summary>
    public Shape RgNLineVCompound() => Published(InternalCompound(4, true, new Shape()));

    public Shape RgNLineVCompound(in Shape S) => Published(InternalCompound(4, true, S));

    /// <summary>The visible outlines (silhouettes).</summary>
    public Shape OutLineVCompound() => Published(InternalCompound(2, true, new Shape()));

    /// <summary>The visible outlines and edges, in 3D.</summary>
    public Shape OutLineVCompound3d() => Published(InternalCompound(2, true, new Shape(), true));

    public Shape OutLineVCompound(in Shape S) => Published(InternalCompound(2, true, S));

    /// <summary>The visible isolines.</summary>
    public Shape IsoLineVCompound() => Published(InternalCompound(1, true, new Shape()));

    public Shape IsoLineVCompound(in Shape S) => Published(InternalCompound(1, true, S));

    /// <summary>The hidden sharp edges.</summary>
    public Shape HCompound() => Published(InternalCompound(5, false, new Shape()));

    public Shape HCompound(in Shape S) => Published(InternalCompound(5, false, S));

    /// <summary>The hidden smooth edges.</summary>
    public Shape Rg1LineHCompound() => Published(InternalCompound(3, false, new Shape()));

    public Shape Rg1LineHCompound(in Shape S) => Published(InternalCompound(3, false, S));

    /// <summary>The hidden sewn edges.</summary>
    public Shape RgNLineHCompound() => Published(InternalCompound(4, false, new Shape()));

    public Shape RgNLineHCompound(in Shape S) => Published(InternalCompound(4, false, S));

    /// <summary>The hidden outlines.</summary>
    public Shape OutLineHCompound() => Published(InternalCompound(2, false, new Shape()));

    public Shape OutLineHCompound(in Shape S) => Published(InternalCompound(2, false, S));

    /// <summary>The hidden isolines.</summary>
    public Shape IsoLineHCompound() => Published(InternalCompound(1, false, new Shape()));

    public Shape IsoLineHCompound(in Shape S) => Published(InternalCompound(1, false, S));

    /// <summary>The visible or hidden edges of a type, in 2D or in 3D.</summary>
    public Shape CompoundOfEdges(HLRBRep_TypeOfResultingEdge type, bool visible, bool In3d)
        => Published(InternalCompound((int)type, visible, new Shape(), In3d));

    /// <summary>The visible or hidden edges of a type of the shape S, in 2D or in 3D.</summary>
    public Shape CompoundOfEdges(in Shape S, HLRBRep_TypeOfResultingEdge type, bool visible, bool In3d)
        => Published(InternalCompound((int)type, visible, S, In3d));

    /// <summary><see cref="CompoundOfEdges(in Shape, HLRBRep_TypeOfResultingEdge, bool, bool)"/> without freezing, for ported code.</summary>
    internal Shape InternalCompoundOfEdges(Shape S, HLRBRep_TypeOfResultingEdge type, bool visible,
                                                  bool In3d)
        => InternalCompound((int)type, visible, S, In3d);

    private static Shape Published(Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    private Shape InternalCompound(int typ, bool visible, in Shape S, bool In3d = false)
    {
        bool added = false;
        Shape Result = new();
        HLRBRep_Data? DS = myAlgo.DataStructure();

        if (DS is not null)
        {
            DS.Projector().Scaled(true);
            int e1 = 1;
            int e2 = DS.NbEdges();
            int f1 = 1;
            int f2 = DS.NbFaces();
            bool explor = false;
            //    bool todraw;
            if (!S.IsNull)
            {
                int v1, v2;
                int index = myAlgo.Index(S);
                if (index == 0)
                {
                    explor = true;
                }
                else
                {
                    myAlgo.ShapeBounds(index).Bounds(out v1, out v2, out e1, out e2, out f1, out f2);
                }
            }
            BrepBuilder B = new();
            Compound aCompound = new();
            B.MakeCompound(ref aCompound);
            Result = aCompound;
            NCollection_Array1<HLRBRep_EdgeData> anEData = DS.EDataArray();

            for (int ie = e1; ie <= e2; ie++)
            {
                HLRBRep_EdgeData ed = anEData[ie];
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
                NCollection_IndexedMap<Shape> Edges = DS.EdgeMap();
                NCollection_IndexedMap<Shape> Faces = DS.FaceMap();
                Explorer Exp = new();

                for (Exp.Init(S, ShapeType.Face); Exp.More(); Exp.Next())
                {
                    int iface = Faces.FindIndex(WorkingImage(Exp.Current()));
                    if (iface != 0)
                    {
                        DrawFace(visible, typ, iface, DS, ref Result, ref added, In3d);
                    }
                }
                if (typ >= 3)
                {
                    for (Exp.Init(S, ShapeType.Edge, ShapeType.Face); Exp.More(); Exp.Next())
                    {
                        int ie = Edges.FindIndex(WorkingImage(Exp.Current()));
                        if (ie != 0)
                        {
                            HLRBRep_EdgeData EDataIE = DS.EDataArray()[ie];
                            if (!EDataIE.Used())
                            {
                                DrawEdge(visible, false, typ, EDataIE, ref Result, ref added, In3d);
                                EDataIE.Used(true);
                            }
                        }
                    }
                }
            }
            else
            {
                for (int iface = f1; iface <= f2; iface++)
                {
                    DrawFace(visible, typ, iface, DS, ref Result, ref added, In3d);
                }

                if (typ >= 3)
                {
                    for (int ie = e1; ie <= e2; ie++)
                    {
                        HLRBRep_EdgeData EDataE11 = anEData[ie];
                        if (!EDataE11.Used())
                        {
                            DrawEdge(visible, false, typ, EDataE11, ref Result, ref added, In3d);
                            EDataE11.Used(true);
                        }
                    }
                }
            }
            DS.Projector().Scaled(false);
        }
        if (!added)
        {
            Result = new Shape();
        }
        return Result;
    }

    // A sub-shape of a loaded shape as the data structure knows it: its image in the outliner's working shape (the
    // copy at the entry of a frozen shape, decision 1095). OCCT finds the shape itself; the port looks in each
    // outliner, the first one that copied the sub-shape.
    private Shape WorkingImage(in Shape theSubShape)
    {
        for (int i = 1; i <= myAlgo.NbShapes(); i++)
        {
            HLRTopoBRep_OutLiner? anOutLiner = myAlgo.ShapeBounds(i).Shape();
            if (anOutLiner is null)
            {
                continue;
            }
            Shape anImage = anOutLiner.InternalWorkingImage(theSubShape);
            if (!ReferenceEquals(anImage.TShape(), theSubShape.TShape()))
            {
                return anImage;
            }
        }
        return theSubShape;
    }

    private void DrawFace(bool visible, int typ, int iface, HLRBRep_Data DS, ref Shape Result, ref bool added,
                          bool In3d = false)
    {
        HLRBRep_FaceIterator Itf = new();

        for (Itf.InitEdge(DS.FDataArray()[iface]); Itf.MoreEdge(); Itf.NextEdge())
        {
            int ie = Itf.Edge();
            HLRBRep_EdgeData edf = DS.EDataArray()[ie];
            if (!edf.Used())
            {
                bool todraw;
                if (typ == 1)
                {
                    todraw = Itf.IsoLine();
                }
                else if (typ == 2) // outlines
                {
                    if (In3d)
                    {
                        todraw = Itf.Internal() || Itf.OutLine();
                    }
                    else
                    {
                        todraw = Itf.Internal();
                    }
                }
                else if (typ == 3)
                {
                    todraw = edf.Rg1Line() && !edf.RgNLine() && !Itf.OutLine();
                }
                else if (typ == 4)
                {
                    todraw = edf.RgNLine() && !Itf.OutLine();
                }
                else
                {
                    todraw = !Itf.IsoLine() && !Itf.Internal() && (!edf.Rg1Line() || Itf.OutLine());
                }

                if (todraw)
                {
                    DrawEdge(visible, true, typ, edf, ref Result, ref added, In3d);
                    edf.Used(true);
                }
                else
                {
                    if ((typ > 4 || typ == 2) && // sharp or outlines
                        (edf.Rg1Line() && !Itf.OutLine()))
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

    private static void DrawEdge(bool visible, bool inFace, int typ, HLRBRep_EdgeData ed, ref Shape Result,
                                 ref bool added, bool In3d = false)
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
            BrepBuilder B = new();
            Edge E;
            HLRAlgo_EdgeIterator It = new();
            if (visible)
            {
                for (It.InitVisible(ed.Status()); It.MoreVisible(); It.NextVisible())
                {
                    It.Visible(out sta, out tolsta, out end, out tolend);
                    if (!In3d)
                    {
                        E = HLRBRep.MakeEdge(ed.Geometry(), sta, end);
                    }
                    else
                    {
                        E = HLRBRep.MakeEdge3d(ed.Geometry(), sta, end);
                    }
                    if (!E.IsNull)
                    {
                        B.Add(Result, E);
                        added = true;
                    }
                }
            }
            else
            {
                for (It.InitHidden(ed.Status()); It.MoreHidden(); It.NextHidden())
                {
                    It.Hidden(out sta, out tolsta, out end, out tolend);
                    if (!In3d)
                    {
                        E = HLRBRep.MakeEdge(ed.Geometry(), sta, end);
                    }
                    else
                    {
                        E = HLRBRep.MakeEdge3d(ed.Geometry(), sta, end);
                    }
                    if (!E.IsNull)
                    {
                        B.Add(Result, E);
                        added = true;
                    }
                }
            }
        }
    }
}
