// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// tools/hlrrec's data.cpp in C#, line for line: HLRTopoBRep's outlining and HLRBRep_Data on the same shapes in the
/// same projectors, the same calls in the same order, written in the same text (every double as its 64 bits in hex).
/// The shapes read from files are frozen, so the outliner works on its copy (decision 1095); the recorder explores that
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

    private void Pnt(Point3D P)
    {
        Hex(P.X());
        Hex(P.Y());
        Hex(P.Z());
    }

    private void Pnt2d(Point2D P)
    {
        Hex(P.X());
        Hex(P.Y());
    }

    private void Vec2d(Vector2D V)
    {
        Hex(V.X());
        Hex(V.Y());
    }

    private void MinMaxRec(in EdgesBlock.MinMaxIndices M)
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

    private void VertexRec(Vertex V)
    {
        if (V.IsNull)
        {
            Tag(" null");
            return;
        }
        Int((int)V.Orientation());
        Pnt(Tool.Pnt(V));
        Hex(Tool.Tolerance(V));
    }

    private void EdgeRec(string theTag, Edge E, Face F)
    {
        Tag(theTag);
        Int((int)E.Orientation());
        global::Kerf.Engine.Topology.Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        VertexRec(V1);
        VertexRec(V2);
        Tool.Range(E, out double f, out double l);
        Hex(f);
        Hex(l);
        if (!V1.IsNull)
        {
            Hex(Tool.Parameter(V1, E));
        }
        if (!V2.IsNull)
        {
            Hex(Tool.Parameter(V2, E));
        }
        Int(Tool.Curve(E, out f, out l) is null ? 0 : 1);
        Curve2D? aPC = Tool.CurveOnSurface(E, F, out double pf, out double pl);
        if (aPC is not null)
        {
            Hex(pf);
            Hex(pl);
            Pnt2d(aPC.Value(pf));
            Pnt2d(aPC.Value(0.37 * pf + 0.63 * pl));
            Pnt2d(aPC.Value(pl));
        }
        EdgeCurveAdaptor C = new(E);
        Pnt(C.Value(C.FirstParameter()));
        Pnt(C.Value(0.37 * C.FirstParameter() + 0.63 * C.LastParameter()));
        Pnt(C.Value(C.LastParameter()));
        End();
    }

    private void RecTopo(OutLiner S, Shape theWorking)
    {
        TopologyData DS = S.DataStructure();
        HashSet<Shape> aFaces = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        int i = 0;
        for (global::Kerf.Engine.Topology.Explorer ex = new(theWorking, ShapeType.Face); ex.More(); ex.Next())
        {
            if (!aFaces.Add(ex.Current()))
            {
                continue;
            }
            i++;
            Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(ex.Current().Oriented(Orientation.Forward));
            Tag("face");
            Int(i);
            Int(DS.FaceHasIntL(F) ? DS.FaceIntL(F).Extent() : 0);
            Int(DS.FaceHasOutL(F) ? DS.FaceOutL(F).Extent() : 0);
            Int(DS.FaceHasIsoL(F) ? DS.FaceIsoL(F).Extent() : 0);
            End();
            if (DS.FaceHasIntL(F))
            {
                for (NCollection_List<Shape>.Iterator it = new(DS.FaceIntL(F)); it.More(); it.Next())
                {
                    EdgeRec("intl", global::Kerf.Engine.Topology.ShapeCasts.Edge(it.Value()), F);
                }
            }
            if (DS.FaceHasOutL(F))
            {
                for (NCollection_List<Shape>.Iterator it = new(DS.FaceOutL(F)); it.More(); it.Next())
                {
                    EdgeRec("outl", global::Kerf.Engine.Topology.ShapeCasts.Edge(it.Value()), F);
                }
            }
            if (DS.FaceHasIsoL(F))
            {
                for (NCollection_List<Shape>.Iterator it = new(DS.FaceIsoL(F)); it.More(); it.Next())
                {
                    EdgeRec("isol", global::Kerf.Engine.Topology.ShapeCasts.Edge(it.Value()), F);
                }
            }
            for (global::Kerf.Engine.Topology.Explorer exE = new(F, ShapeType.Edge); exE.More(); exE.Next())
            {
                Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(exE.Current());
                if (DS.EdgeHasSplE(E))
                {
                    Tag("split");
                    Int(DS.EdgeSplE(E).Extent());
                    End();
                    for (NCollection_List<Shape>.Iterator it = new(DS.EdgeSplE(E)); it.More(); it.Next())
                    {
                        EdgeRec("spl", global::Kerf.Engine.Topology.ShapeCasts.Edge(it.Value()), F);
                    }
                }
            }
        }
    }

    private void RecEData(HiddenLineData DS)
    {
        for (int e = 1; e <= DS.EdgeCount; e++)
        {
            HiddenLineEdgeData ed = DS.EDataArray()[e];
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
            HiddenLineCurve C = ed.Geometry();
            Tag("crv");
            Int((int)C.GetType());
            double f = C.FirstParameter(), l = C.LastParameter();
            Hex(f);
            Hex(l);
            End();
            for (int k = 0; k <= 2; k++)
            {
                double p = C.Parameter3d(f + (l - f) * (k == 0 ? 0.0 : (k == 1 ? 0.37 : 1.0)));
                C.D1(p, out Point2D P, out Vector2D V);
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

    private void RecFData(HiddenLineData DS)
    {
        for (int f = 1; f <= DS.FaceCount; f++)
        {
            HiddenLineFaceData fd = DS.FDataArray()[f];
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
            WiresBlock wb = fd.Wires()!;
            Tag("fmm");
            MinMaxRec(wb.MinMax());
            End();
            for (int w = 1; w <= wb.WireCount; w++)
            {
                EdgesBlock eb = wb.Wire(w)!;
                Tag("wire");
                Int(eb.EdgeCount);
                MinMaxRec(eb.MinMax());
                End();
                Tag("we");
                for (int ie = 1; ie <= eb.EdgeCount; ie++)
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

    private void InterferenceRec(string theTag, in Interference I)
    {
        Tag(theTag);
        AlgoIntersection X = I.Intersection();
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
        I.Boundary().State3D(out Containment b, out Containment a);
        Int((int)b);
        Int((int)a);
        Int((int)I.Orientation());
        Int((int)I.Transition());
        Int((int)I.BoundaryTransition());
        End();
    }

    private void RecHide(HiddenLineData DS, Dictionary<Shape, AdaptorTool> MST, int theMaxFaces)
    {
        EdgesBlock.MinMaxIndices aTot = new();
        for (int k = 0; k < 8; k++)
        {
            aTot.Min[k] = 0;
            aTot.Max[k] = 0x7fff7fff;
        }
        DS.InitBoundSort(aTot, 1, DS.EdgeCount);
        EdgeInterferenceTool EIT = new(DS);
        int nbFaces = 0;
        for (int f = 1; f <= DS.FaceCount && nbFaces < theMaxFaces; f++)
        {
            HiddenLineFaceData fd = DS.FDataArray()[f];
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
                    HiddenLineEdgeData ed = DS.EDataArray()[e];
                    Tag("le");
                    Int(e);
                    Int(DS.HidingTheFace() ? 1 : 0);
                    Int(DS.SimpleHidingFace() ? 1 : 0);
                    Int(DS.EdgeOfTheHidingFace(e, ed) ? 1 : 0);
                    End();
                    NCollection_List<Interference> IL = new();
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
                            ref Interference I = ref DS.Interference();
                            InterferenceRec("itf", I);
                            EdgeIList.AddInterference(IL, I, EIT);
                        }
                        DS.NextInterference();
                    }
                    if (!IL.IsEmpty())
                    {
                        EIT.LoadEdge();
                        EdgeIList.ProcessComplex(IL, EIT);
                        for (NCollection_List<Interference>.Iterator it = new(IL); it.More(); it.Next())
                        {
                            InterferenceRec("cpx", it.Value());
                        }
                        NCollection_List<Interference>.Iterator itIL = new(IL);
                        VertexList VL = new(EIT, itIL);
                        while (VL.More())
                        {
                            Tag("vl");
                            Hex(VL.Current().Parameter());
                            Int(VL.Current().Index());
                            Int(VL.IsBoundary ? 1 : 0);
                            Int(VL.IsInterference ? 1 : 0);
                            if (VL.IsBoundary)
                            {
                                Int((int)VL.Orientation());
                            }
                            if (VL.IsInterference)
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
            catch (FailureException)
            {
                Line("exception");
            }
        }
    }

    public void RecData(string theName, Shape theShape, string theView, AlgorithmProjector P, int nbIso,
                        int theMaxFaces)
    {
        gOut.Append("case hlrdata ").Append(theName).Append(' ').Append(theView).Append('\n');
        try
        {
            OutLiner S = new(theShape);
            Dictionary<Shape, AdaptorTool> MST = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            HiddenLineData DS = ShapeToHlr.Load(S, P, MST, nbIso);
            Tag("sizes");
            Int(DS.VertexCount);
            Int(DS.EdgeCount);
            Int(DS.FaceCount);
            End();
            RecTopo(S, S.InternalWorkingShape());
            DS.Update(P);
            RecEData(DS);
            RecFData(DS);
            RecHide(DS, MST, theMaxFaces);
        }
        catch (FailureException)
        {
            Line("exception");
        }
    }

    public static AlgorithmProjector View(int theView) => theView switch
    {
        0 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2))),
        1 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))),
        2 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, -1), new Direction3D(1, 0, 0))),
        3 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(1, 0, 0))),
        4 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(-1, 0, 0))),
        5 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(-1, 0, 0), new Direction3D(0, -1, 0))),
        6 => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0))),
        _ => new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2)), 400.0),
    };

    public static readonly string[] THE_VIEWS = ["axo", "top", "bottom", "front", "back", "left", "right", "persp"];

    public Shape ReadShape(string theFile)
    {
        string aPath = Path.Combine(myDataDir, "brep", theFile);
        Assert.True(BrepTools.Read(out Shape aShape, aPath), aPath);
        return aShape;
    }

    /// <summary>The primitives in every view and in perspective (data.cpp's first loop).</summary>
    public void RecPrimitives()
    {
        Shape aCyl = new ApiMakeCylinder(10.0, 30.0).Shape();
        Shape aTorus = new MakeTorus(30.0, 10.0).Shape();
        Shape aBox = new MakeBox(new Point3D(-5, -10, -2), 10.0, 20.0, 30.0).Shape();
        Shape aSph = new MakeSphere(new Point3D(3, 2, 1), 8.0).Shape();
        Shape aCone =
            new ApiMakeCone(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 1)), 10.0, 4.0, 20.0).Shape();
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
        Shape aH1 = ReadShape("bug25908_hlr-bspline-clone2-tcl-h1.brep");
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
