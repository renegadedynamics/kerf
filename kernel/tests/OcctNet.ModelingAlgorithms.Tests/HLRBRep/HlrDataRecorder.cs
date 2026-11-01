// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// tools/hlrrec's data.cpp in C#, line for line: HLRTopoBRep's outlining and HLRBRep_Data on the same shapes in the
/// same projectors, the same calls in the same order, written in the same text (every double as its 64 bits in hex).
/// The shapes read from files are frozen, so the outliner works on its copy (decision 10605); the recorder explores that
/// working shape where native explores the original (the same topology).
/// </summary>
internal sealed class HlrDataRecorder
{
    private readonly StringBuilder gOut = new();
    private readonly string myDataDir;

    public HlrDataRecorder(string theDataDir) => myDataDir = theDataDir;

    public string Text => gOut.ToString();

    private void Hex(double theValue) =>
        gOut.Append(' ').Append(BitConverter.DoubleToUInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture));

    private void HexF(float theValue) =>
        gOut.Append(' ').Append(BitConverter.SingleToUInt32Bits(theValue).ToString("x8", CultureInfo.InvariantCulture));

    private void Tag(string theTag) => gOut.Append(theTag);

    private void Int(int theValue) => gOut.Append(' ').Append(theValue.ToString(CultureInfo.InvariantCulture));

    private void Int(bool theValue) => Int(theValue ? 1 : 0);

    private void End() => gOut.Append('\n');

    private void Line(string theText) => gOut.Append(theText).Append('\n');

    private void Pnt(gp_Pnt P)
    {
        Hex(P.X());
        Hex(P.Y());
        Hex(P.Z());
    }

    private void Pnt2d(gp_Pnt2d P)
    {
        Hex(P.X());
        Hex(P.Y());
    }

    private void Vec2d(gp_Vec2d V)
    {
        Hex(V.X());
        Hex(V.Y());
    }

    private void MinMaxRec(in HLRAlgo_EdgesBlock.MinMaxIndices M)
    {
        for (int k = 0; k < 8; k++)
        {
            Int(M.Min[k]);
        }
        for (int k = 0; k < 8; k++)
        {
            Int(M.Max[k]);
        }
    }

    private void VertexRec(TopoDS_Vertex V)
    {
        if (V.IsNull())
        {
            Tag(" null");
            return;
        }
        Int((int)V.Orientation());
        Pnt(BRep_Tool.Pnt(V));
        Hex(BRep_Tool.Tolerance(V));
    }

    private void EdgeRec(string theTag, TopoDS_Edge E, TopoDS_Face F)
    {
        Tag(theTag);
        Int((int)E.Orientation());
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        VertexRec(V1);
        VertexRec(V2);
        BRep_Tool.Range(E, out double f, out double l);
        Hex(f);
        Hex(l);
        if (!V1.IsNull())
        {
            Hex(BRep_Tool.Parameter(V1, E));
        }
        if (!V2.IsNull())
        {
            Hex(BRep_Tool.Parameter(V2, E));
        }
        Int(BRep_Tool.Curve(E, out f, out l) is null ? 0 : 1);
        Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(E, F, out double pf, out double pl);
        if (aPC is not null)
        {
            Hex(pf);
            Hex(pl);
            Pnt2d(aPC.Value(pf));
            Pnt2d(aPC.Value(0.37 * pf + 0.63 * pl));
            Pnt2d(aPC.Value(pl));
        }
        BRepAdaptor_Curve C = new(E);
        Pnt(C.Value(C.FirstParameter()));
        Pnt(C.Value(0.37 * C.FirstParameter() + 0.63 * C.LastParameter()));
        Pnt(C.Value(C.LastParameter()));
        End();
    }

    private void RecTopo(HLRTopoBRep_OutLiner S, TopoDS_Shape theWorking)
    {
        HLRTopoBRep_Data DS = S.DataStructure();
        NCollection_Map<TopoDS_Shape> aFaces = new(TopTools_ShapeMapHasher.Instance);
        int i = 0;
        for (TopExp_Explorer ex = new(theWorking, TopAbs_FACE); ex.More(); ex.Next())
        {
            if (!aFaces.Add(ex.Current()))
            {
                continue;
            }
            i++;
            TopoDS_Face F = TopoDS.Face(ex.Current().Oriented(TopAbs_Orientation.TopAbs_FORWARD));
            Tag("face");
            Int(i);
            Int(DS.FaceHasIntL(F) ? DS.FaceIntL(F).Extent() : 0);
            Int(DS.FaceHasOutL(F) ? DS.FaceOutL(F).Extent() : 0);
            Int(DS.FaceHasIsoL(F) ? DS.FaceIsoL(F).Extent() : 0);
            End();
            if (DS.FaceHasIntL(F))
            {
                for (NCollection_List<TopoDS_Shape>.Iterator it = new(DS.FaceIntL(F)); it.More(); it.Next())
                {
                    EdgeRec("intl", TopoDS.Edge(it.Value()), F);
                }
            }
            if (DS.FaceHasOutL(F))
            {
                for (NCollection_List<TopoDS_Shape>.Iterator it = new(DS.FaceOutL(F)); it.More(); it.Next())
                {
                    EdgeRec("outl", TopoDS.Edge(it.Value()), F);
                }
            }
            if (DS.FaceHasIsoL(F))
            {
                for (NCollection_List<TopoDS_Shape>.Iterator it = new(DS.FaceIsoL(F)); it.More(); it.Next())
                {
                    EdgeRec("isol", TopoDS.Edge(it.Value()), F);
                }
            }
            for (TopExp_Explorer exE = new(F, TopAbs_EDGE); exE.More(); exE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exE.Current());
                if (DS.EdgeHasSplE(E))
                {
                    Tag("split");
                    Int(DS.EdgeSplE(E).Extent());
                    End();
                    for (NCollection_List<TopoDS_Shape>.Iterator it = new(DS.EdgeSplE(E)); it.More(); it.Next())
                    {
                        EdgeRec("spl", TopoDS.Edge(it.Value()), F);
                    }
                }
            }
        }
    }

    private void RecEData(HLRBRep_Data DS)
    {
        for (int e = 1; e <= DS.NbEdges(); e++)
        {
            HLRBRep_EdgeData ed = DS.EDataArray()[e];
            Tag("edge");
            Int(e);
            Int(ed.Selected());
            Int(ed.Rg1Line());
            Int(ed.RgNLine());
            Int(ed.Vertical());
            Int(ed.Simple());
            Int(ed.OutLVSta());
            Int(ed.OutLVEnd());
            Int(ed.CutAtSta());
            Int(ed.CutAtEnd());
            Int(ed.VerAtSta());
            Int(ed.VerAtEnd());
            Int(ed.AutoIntersectionDone());
            Int(ed.Used());
            Int(ed.HideCount());
            Int(ed.VSta());
            Int(ed.VEnd());
            HexF(ed.Tolerance());
            End();
            Tag("mm");
            MinMaxRec(ed.MinMax());
            End();
            ed.Status().Bounds(out double p1, out float t1, out double p2, out float t2);
            Tag("st");
            Hex(p1);
            HexF(t1);
            Hex(p2);
            HexF(t2);
            End();
            HLRBRep_Curve C = ed.Geometry();
            Tag("crv");
            Int((int)C.GetType());
            double f = C.FirstParameter(), l = C.LastParameter();
            Hex(f);
            Hex(l);
            End();
            for (int k = 0; k <= 2; k++)
            {
                double p = C.Parameter3d(f + (l - f) * (k == 0 ? 0.0 : (k == 1 ? 0.37 : 1.0)));
                C.D1(p, out gp_Pnt2d P, out gp_Vec2d V);
                Tag("pt");
                Hex(p);
                Hex(C.Parameter2d(p));
                Pnt2d(P);
                Vec2d(V);
                Hex(C.Z(p));
                End();
            }
        }
    }

    private void RecFData(HLRBRep_Data DS)
    {
        for (int f = 1; f <= DS.NbFaces(); f++)
        {
            HLRBRep_FaceData fd = DS.FDataArray()[f];
            Tag("fdata");
            Int(f);
            Int(fd.Selected());
            Int(fd.Back());
            Int(fd.Side());
            Int(fd.Closed());
            Int(fd.Hiding());
            Int(fd.Simple());
            Int(fd.Cut());
            Int(fd.WithOutL());
            Int(fd.Plane());
            Int(fd.Cylinder());
            Int(fd.Cone());
            Int(fd.Sphere());
            Int(fd.Torus());
            Int((int)fd.Orientation());
            Int((int)fd.Geometry().GetType());
            Hex(fd.Size());
            HexF(fd.Tolerance());
            End();
            HLRAlgo_WiresBlock wb = fd.Wires()!;
            Tag("fmm");
            MinMaxRec(wb.MinMax());
            End();
            for (int w = 1; w <= wb.NbWires(); w++)
            {
                HLRAlgo_EdgesBlock eb = wb.Wire(w)!;
                Tag("wire");
                Int(eb.NbEdges());
                MinMaxRec(eb.MinMax());
                End();
                Tag("we");
                for (int ie = 1; ie <= eb.NbEdges(); ie++)
                {
                    Int(eb.Edge(ie));
                    Int((int)eb.Orientation(ie));
                    Int(eb.OutLine(ie) ? 1 : 0);
                    Int(eb.Internal(ie) ? 1 : 0);
                    Int(eb.Double(ie) ? 1 : 0);
                    Int(eb.IsoLine(ie) ? 1 : 0);
                }
                End();
            }
        }
    }

    private void InterferenceRec(string theTag, in HLRAlgo_Interference I)
    {
        Tag(theTag);
        HLRAlgo_Intersection X = I.Intersection();
        Int((int)X.Orientation());
        Int(X.Level());
        Int(X.SegIndex());
        Int(X.Index());
        Hex(X.Parameter());
        HexF(X.Tolerance());
        Int((int)X.State());
        I.Boundary().Value2D(out int FE, out double Param);
        Int(FE);
        Hex(Param);
        I.Boundary().State3D(out TopAbs_State b, out TopAbs_State a);
        Int((int)b);
        Int((int)a);
        Int((int)I.Orientation());
        Int((int)I.Transition());
        Int((int)I.BoundaryTransition());
        End();
    }

    private void RecHide(HLRBRep_Data DS, NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST, int theMaxFaces)
    {
        HLRAlgo_EdgesBlock.MinMaxIndices aTot = new();
        for (int k = 0; k < 8; k++)
        {
            aTot.Min[k] = 0;
            aTot.Max[k] = 0x7fff7fff;
        }
        DS.InitBoundSort(aTot, 1, DS.NbEdges());
        HLRBRep_EdgeInterferenceTool EIT = new(DS);
        int nbFaces = 0;
        for (int f = 1; f <= DS.NbFaces() && nbFaces < theMaxFaces; f++)
        {
            HLRBRep_FaceData fd = DS.FDataArray()[f];
            if (!fd.Hiding())
            {
                continue;
            }
            nbFaces++;
            Tag("hide");
            Int(f);
            End();
            try
            {
                DS.InitEdge(f, MST);
                Tag("bad");
                Int(DS.IsBadFace() ? 1 : 0);
                End();
                while (DS.MoreEdge())
                {
                    int e = DS.Edge();
                    HLRBRep_EdgeData ed = DS.EDataArray()[e];
                    Tag("le");
                    Int(e);
                    Int(DS.HidingTheFace() ? 1 : 0);
                    Int(DS.SimpleHidingFace() ? 1 : 0);
                    Int(DS.EdgeOfTheHidingFace(e, ed) ? 1 : 0);
                    End();
                    NCollection_List<HLRAlgo_Interference> IL = new();
                    DS.InitInterference();
                    while (DS.MoreInterference())
                    {
                        if (DS.RejectedInterference())
                        {
                            Tag("rej");
                            Int(DS.AboveInterference() ? 1 : 0);
                            End();
                        }
                        else
                        {
                            ref HLRAlgo_Interference I = ref DS.Interference();
                            InterferenceRec("itf", I);
                            HLRBRep_EdgeIList.AddInterference(IL, I, EIT);
                        }
                        DS.NextInterference();
                    }
                    if (!IL.IsEmpty())
                    {
                        EIT.LoadEdge();
                        HLRBRep_EdgeIList.ProcessComplex(IL, EIT);
                        for (NCollection_List<HLRAlgo_Interference>.Iterator it = new(IL); it.More(); it.Next())
                        {
                            InterferenceRec("cpx", it.Value());
                        }
                        NCollection_List<HLRAlgo_Interference>.Iterator itIL = new(IL);
                        HLRBRep_VertexList VL = new(EIT, itIL);
                        while (VL.More())
                        {
                            Tag("vl");
                            Hex(VL.Current().Parameter());
                            Int(VL.Current().Index());
                            Int(VL.IsBoundary() ? 1 : 0);
                            Int(VL.IsInterference() ? 1 : 0);
                            if (VL.IsBoundary())
                            {
                                Int((int)VL.Orientation());
                            }
                            if (VL.IsInterference())
                            {
                                Int((int)VL.Transition());
                                Int((int)VL.BoundaryTransition());
                            }
                            End();
                            VL.Next();
                        }
                        Tag("lvl");
                        Int(DS.HidingStartLevel(e, ed, IL));
                        End();
                    }
                    ed.Status().Bounds(out double p1, out float t1, out double p2, out float t2);
                    Tag("cls");
                    Int((int)DS.Compare(e, ed));
                    Int((int)DS.SimplClassify(e, ed, 3, p1, p2));
                    End();
                    DS.NextEdge();
                }
            }
            catch (Standard_Failure)
            {
                Line("exception");
            }
        }
    }

    public void RecData(string theName, TopoDS_Shape theShape, string theView, HLRAlgo_Projector P, int nbIso,
                        int theMaxFaces)
    {
        gOut.Append("case hlrdata ").Append(theName).Append(' ').Append(theView).Append('\n');
        try
        {
            HLRTopoBRep_OutLiner S = new(theShape);
            NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST = new(TopTools_ShapeMapHasher.Instance);
            HLRBRep_Data DS = HLRBRep_ShapeToHLR.Load(S, P, MST, nbIso);
            Tag("sizes");
            Int(DS.NbVertices());
            Int(DS.NbEdges());
            Int(DS.NbFaces());
            End();
            RecTopo(S, S.InternalWorkingShape());
            DS.Update(P);
            RecEData(DS);
            RecFData(DS);
            RecHide(DS, MST, theMaxFaces);
        }
        catch (Standard_Failure)
        {
            Line("exception");
        }
    }

    public static HLRAlgo_Projector View(int theView) => theView switch
    {
        0 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, -1, 1), new gp_Dir(-1, 1, 2))),
        1 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0))),
        2 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, -1), new gp_Dir(1, 0, 0))),
        3 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, -1, 0), new gp_Dir(1, 0, 0))),
        4 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0), new gp_Dir(-1, 0, 0))),
        5 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(-1, 0, 0), new gp_Dir(0, -1, 0))),
        6 => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0))),
        _ => new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, -1, 1), new gp_Dir(-1, 1, 2)), 400.0),
    };

    public static readonly string[] THE_VIEWS = ["axo", "top", "bottom", "front", "back", "left", "right", "persp"];

    public TopoDS_Shape ReadShape(string theFile)
    {
        string aPath = Path.Combine(myDataDir, "brep", theFile);
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aPath), aPath);
        return aShape;
    }

    /// <summary>The primitives in every view and in perspective (data.cpp's first loop).</summary>
    public void RecPrimitives()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
        TopoDS_Shape aTorus = new BRepPrimAPI_MakeTorus(30.0, 10.0).Shape();
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -10, -2), 10.0, 20.0, 30.0).Shape();
        TopoDS_Shape aSph = new BRepPrimAPI_MakeSphere(new gp_Pnt(3, 2, 1), 8.0).Shape();
        TopoDS_Shape aCone =
            new BRepPrimAPI_MakeCone(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 1)), 10.0, 4.0, 20.0).Shape();
        for (int v = 0; v < 8; v++)
        {
            RecData("cylinder", aCyl, THE_VIEWS[v], View(v), v == 0 ? 3 : 0, 1000);
            RecData("torus", aTorus, THE_VIEWS[v], View(v), v == 0 ? 3 : 0, 1000);
            RecData("box", aBox, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
            RecData("sphere", aSph, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
            RecData("cone", aCone, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
        }
    }

    /// <summary>The B-spline shape of bug25908 in its seven views.</summary>
    public void RecBSplineClone(int theView)
    {
        TopoDS_Shape aH1 = ReadShape("bug25908_hlr-bspline-clone2-tcl-h1.brep");
        RecData("bspline-clone2", aH1, THE_VIEWS[theView], View(theView), theView == 0 ? 2 : 0, 25);
    }

    /// <summary>The other shapes of the hlr group (data.cpp's last lines).</summary>
    public void RecOthers()
    {
        RecData("bus1", ReadShape("bug25813_hlr-bus1-draw-Fusion001.brep"), "axo", View(0), 0, 25);
        RecData("normandc1m1", ReadShape("bug25813_hlr-test_normandc1m1-draw-Cut001.brep"), "axo", View(0), 2, 25);
        RecData("fillet2", ReadShape("bug25813_hlr-fillet2-tcl-f.brep"), "axo", View(0), 0, 25);
        RecData("bug32747", ReadShape("bug32747.brep"), "front", View(3), 0, 25);
        RecData("bug28242", ReadShape("bug28242.brep"), "axo", View(0), 0, 25);
    }
}
