// hlrrec, the data model (unit 7Q1): records HLRTopoBRep's outlining and HLRBRep_Data on the hlr group's shapes in
// fixed projectors, every double as its 64 bits in hex. Included by main.cpp; run with a second argument (the output)
// and a third (the test data directory).
// tests/OcctNet.ModelingAlgorithms.Tests/HLRBRep/HlrDataRecorder.cs is the same program in C#, line for line.

#include <BRepAdaptor_Curve.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <HLRAlgo_Interference.hxx>
#include <HLRBRep_Data.hxx>
#include <HLRBRep_EdgeData.hxx>
#include <HLRBRep_EdgeIList.hxx>
#include <HLRBRep_EdgeInterferenceTool.hxx>
#include <HLRBRep_FaceData.hxx>
#include <HLRBRep_ShapeToHLR.hxx>
#include <HLRBRep_VertexList.hxx>
#include <HLRTopoBRep_Data.hxx>
#include <HLRTopoBRep_OutLiner.hxx>
#include <NCollection_List.hxx>
#include <NCollection_Map.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>

static void MinMaxRec(const HLRAlgo_EdgesBlock::MinMaxIndices& M)
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

static void VertexRec(const TopoDS_Vertex& V)
{
  if (V.IsNull())
  {
    Tag(" null");
    return;
  }
  Int((int)V.Orientation());
  Pnt(BRep_Tool::Pnt(V));
  Hex(BRep_Tool::Tolerance(V));
}

// An edge of the outlining: its orientation, vertices, range, points, and its parameters on its vertices.
static void EdgeRec(const char* theTag, const TopoDS_Edge& E, const TopoDS_Face& F)
{
  Tag(theTag);
  Int((int)E.Orientation());
  TopoDS_Vertex V1, V2;
  TopExp::Vertices(E, V1, V2);
  VertexRec(V1);
  VertexRec(V2);
  double f, l;
  BRep_Tool::Range(E, f, l);
  Hex(f);
  Hex(l);
  if (!V1.IsNull())
  {
    Hex(BRep_Tool::Parameter(V1, E));
  }
  if (!V2.IsNull())
  {
    Hex(BRep_Tool::Parameter(V2, E));
  }
  Int(BRep_Tool::Curve(E, f, l).IsNull() ? 0 : 1);
  double pf, pl;
  occ::handle<Geom2d_Curve> aPC = BRep_Tool::CurveOnSurface(E, F, pf, pl);
  if (!aPC.IsNull())
  {
    Hex(pf);
    Hex(pl);
    Pnt2d(aPC->Value(pf));
    Pnt2d(aPC->Value(0.37 * pf + 0.63 * pl));
    Pnt2d(aPC->Value(pl));
  }
  BRepAdaptor_Curve C(E);
  Pnt(C.Value(C.FirstParameter()));
  Pnt(C.Value(0.37 * C.FirstParameter() + 0.63 * C.LastParameter()));
  Pnt(C.Value(C.LastParameter()));
  End();
}

static void RecTopo(const occ::handle<HLRTopoBRep_OutLiner>& S, const TopoDS_Shape& theWorking)
{
  HLRTopoBRep_Data&                                      DS = S->DataStructure();
  NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  int                                                    i = 0;
  for (TopExp_Explorer ex(theWorking, TopAbs_FACE); ex.More(); ex.Next())
  {
    if (!aFaces.Add(ex.Current()))
    {
      continue;
    }
    i++;
    TopoDS_Face F = TopoDS::Face(ex.Current());
    F.Orientation(TopAbs_FORWARD);
    Tag("face");
    Int(i);
    Int(DS.FaceHasIntL(F) ? DS.FaceIntL(F).Extent() : 0);
    Int(DS.FaceHasOutL(F) ? DS.FaceOutL(F).Extent() : 0);
    Int(DS.FaceHasIsoL(F) ? DS.FaceIsoL(F).Extent() : 0);
    End();
    if (DS.FaceHasIntL(F))
    {
      for (NCollection_List<TopoDS_Shape>::Iterator it(DS.FaceIntL(F)); it.More(); it.Next())
      {
        EdgeRec("intl", TopoDS::Edge(it.Value()), F);
      }
    }
    if (DS.FaceHasOutL(F))
    {
      for (NCollection_List<TopoDS_Shape>::Iterator it(DS.FaceOutL(F)); it.More(); it.Next())
      {
        EdgeRec("outl", TopoDS::Edge(it.Value()), F);
      }
    }
    if (DS.FaceHasIsoL(F))
    {
      for (NCollection_List<TopoDS_Shape>::Iterator it(DS.FaceIsoL(F)); it.More(); it.Next())
      {
        EdgeRec("isol", TopoDS::Edge(it.Value()), F);
      }
    }
    // the splits of the face's edges
    for (TopExp_Explorer exE(F, TopAbs_EDGE); exE.More(); exE.Next())
    {
      const TopoDS_Edge& E = TopoDS::Edge(exE.Current());
      if (DS.EdgeHasSplE(E))
      {
        Tag("split");
        Int(DS.EdgeSplE(E).Extent());
        End();
        for (NCollection_List<TopoDS_Shape>::Iterator it(DS.EdgeSplE(E)); it.More(); it.Next())
        {
          EdgeRec("spl", TopoDS::Edge(it.Value()), F);
        }
      }
    }
  }
}

static void RecEData(const occ::handle<HLRBRep_Data>& DS)
{
  for (int e = 1; e <= DS->NbEdges(); e++)
  {
    HLRBRep_EdgeData& ed = DS->EDataArray().ChangeValue(e);
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
    double p1, p2;
    float  t1, t2;
    ed.Status().Bounds(p1, t1, p2, t2);
    Tag("st");
    Hex(p1);
    HexF(t1);
    Hex(p2);
    HexF(t2);
    End();
    const HLRBRep_Curve& C = ed.Geometry();
    Tag("crv");
    Int((int)C.GetType());
    double f = C.FirstParameter(), l = C.LastParameter();
    Hex(f);
    Hex(l);
    End();
    for (int k = 0; k <= 2; k++)
    {
      double p = C.Parameter3d(f + (l - f) * (k == 0 ? 0. : (k == 1 ? 0.37 : 1.)));
      gp_Pnt2d P;
      gp_Vec2d V;
      C.D1(p, P, V);
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

static void RecFData(const occ::handle<HLRBRep_Data>& DS)
{
  for (int f = 1; f <= DS->NbFaces(); f++)
  {
    HLRBRep_FaceData& fd = DS->FDataArray().ChangeValue(f);
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
    const occ::handle<HLRAlgo_WiresBlock>& wb = fd.Wires();
    Tag("fmm");
    MinMaxRec(wb->MinMax());
    End();
    for (int w = 1; w <= wb->NbWires(); w++)
    {
      const occ::handle<HLRAlgo_EdgesBlock>& eb = wb->Wire(w);
      Tag("wire");
      Int(eb->NbEdges());
      MinMaxRec(eb->MinMax());
      End();
      Tag("we");
      for (int ie = 1; ie <= eb->NbEdges(); ie++)
      {
        Int(eb->Edge(ie));
        Int((int)eb->Orientation(ie));
        Int(eb->OutLine(ie) ? 1 : 0);
        Int(eb->Internal(ie) ? 1 : 0);
        Int(eb->Double(ie) ? 1 : 0);
        Int(eb->IsoLine(ie) ? 1 : 0);
      }
      End();
    }
  }
}

static void InterferenceRec(const char* theTag, const HLRAlgo_Interference& I)
{
  Tag(theTag);
  const HLRAlgo_Intersection& X = I.Intersection();
  Int((int)X.Orientation());
  Int(X.Level());
  Int(X.SegIndex());
  Int(X.Index());
  Hex(X.Parameter());
  HexF(X.Tolerance());
  Int((int)X.State());
  int    FE;
  double Param;
  I.Boundary().Value2D(FE, Param);
  Int(FE);
  Hex(Param);
  TopAbs_State b, a;
  I.Boundary().State3D(b, a);
  Int((int)b);
  Int((int)a);
  Int((int)I.Orientation());
  Int((int)I.Transition());
  Int((int)I.BoundaryTransition());
  End();
}

// The hider's walk over the data, without hiding: for each hiding face, its edges, their interferences with the
// face's boundary (sorted and merged as the hider does), the vertex lists and the classifications.
static void RecHide(const occ::handle<HLRBRep_Data>&                                                  DS,
                    NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool, TopTools_ShapeMapHasher>& MST,
                    int                                                                              theMaxFaces)
{
  HLRAlgo_EdgesBlock::MinMaxIndices aTot;
  for (int k = 0; k < 8; k++)
  {
    aTot.Min[k] = 0;
    aTot.Max[k] = 0x7fff7fff;
  }
  DS->InitBoundSort(aTot, 1, DS->NbEdges());
  HLRBRep_EdgeInterferenceTool EIT(DS);
  int                          nbFaces = 0;
  for (int f = 1; f <= DS->NbFaces() && nbFaces < theMaxFaces; f++)
  {
    HLRBRep_FaceData& fd = DS->FDataArray().ChangeValue(f);
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
      DS->InitEdge(f, MST);
      Tag("bad");
      Int(DS->IsBadFace() ? 1 : 0);
      End();
      while (DS->MoreEdge())
      {
        int               e  = DS->Edge();
        HLRBRep_EdgeData& ed = DS->EDataArray().ChangeValue(e);
        Tag("le");
        Int(e);
        Int(DS->HidingTheFace() ? 1 : 0);
        Int(DS->SimpleHidingFace() ? 1 : 0);
        Int(DS->EdgeOfTheHidingFace(e, ed) ? 1 : 0);
        End();
        NCollection_List<HLRAlgo_Interference> IL;
        DS->InitInterference();
        while (DS->MoreInterference())
        {
          if (DS->RejectedInterference())
          {
            Tag("rej");
            Int(DS->AboveInterference() ? 1 : 0);
            End();
          }
          else
          {
            HLRAlgo_Interference& I = DS->Interference();
            InterferenceRec("itf", I);
            HLRBRep_EdgeIList::AddInterference(IL, I, EIT);
          }
          DS->NextInterference();
        }
        if (!IL.IsEmpty())
        {
          EIT.LoadEdge();
          HLRBRep_EdgeIList::ProcessComplex(IL, EIT);
          for (NCollection_List<HLRAlgo_Interference>::Iterator it(IL); it.More(); it.Next())
          {
            InterferenceRec("cpx", it.Value());
          }
          NCollection_List<HLRAlgo_Interference>::Iterator itIL(IL);
          HLRBRep_VertexList                               VL(EIT, itIL);
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
          Int(DS->HidingStartLevel(e, ed, IL));
          End();
        }
        double p1, p2;
        float  t1, t2;
        ed.Status().Bounds(p1, t1, p2, t2);
        Tag("cls");
        Int((int)DS->Compare(e, ed));
        Int((int)DS->SimplClassify(e, ed, 3, p1, p2));
        End();
        DS->NextEdge();
      }
    }
    catch (Standard_Failure&)
    {
      Line("exception");
    }
  }
}

static void RecData(const char* theName, const TopoDS_Shape& theShape, const char* theView, const HLRAlgo_Projector& P,
                    int nbIso, int theMaxFaces)
{
  std::fprintf(gOut, "case hlrdata %s %s\n", theName, theView);
  try
  {
    occ::handle<HLRTopoBRep_OutLiner> S = new HLRTopoBRep_OutLiner(theShape);
    NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool, TopTools_ShapeMapHasher> MST;
    occ::handle<HLRBRep_Data> DS = HLRBRep_ShapeToHLR::Load(S, P, MST, nbIso);
    Tag("sizes");
    Int(DS->NbVertices());
    Int(DS->NbEdges());
    Int(DS->NbFaces());
    End();
    RecTopo(S, S->OriginalShape());
    DS->Update(P);
    RecEData(DS);
    RecFData(DS);
    RecHide(DS, MST, theMaxFaces);
  }
  catch (Standard_Failure&)
  {
    Line("exception");
  }
}

static HLRAlgo_Projector View(int theView)
{
  switch (theView)
  {
    case 0:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, -1, 1), gp_Dir(-1, 1, 2)));
    case 1:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)));
    case 2:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, -1), gp_Dir(1, 0, 0)));
    case 3:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0)));
    case 4:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(-1, 0, 0)));
    case 5:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0), gp_Dir(0, -1, 0)));
    case 6:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 1, 0)));
    default:
      return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, -1, 1), gp_Dir(-1, 1, 2)), 400.);
  }
}

static const char* const THE_VIEWS[8] = {"axo", "top", "bottom", "front", "back", "left", "right", "persp"};

static TopoDS_Shape ReadShape(const char* theDir, const char* theFile)
{
  TopoDS_Shape aShape;
  BRep_Builder aB;
  std::string  aPath = std::string(theDir) + "/brep/" + theFile;
  if (!BRepTools::Read(aShape, aPath.c_str(), aB))
  {
    std::fprintf(stderr, "cannot read %s\n", aPath.c_str());
  }
  return aShape;
}

static void RecDataAll(const char* theDataDir)
{
  // primitives (the hlr group's bug25813_1 cylinders and torus), in every view and in perspective
  TopoDS_Shape aCyl   = BRepPrimAPI_MakeCylinder(10., 30.).Shape();
  TopoDS_Shape aTorus = BRepPrimAPI_MakeTorus(30., 10.).Shape();
  TopoDS_Shape aBox   = BRepPrimAPI_MakeBox(gp_Pnt(-5, -10, -2), 10., 20., 30.).Shape();
  TopoDS_Shape aSph   = BRepPrimAPI_MakeSphere(gp_Pnt(3, 2, 1), 8.).Shape();
  TopoDS_Shape aCone  = BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 1)), 10., 4., 20.).Shape();
  for (int v = 0; v < 8; v++)
  {
    RecData("cylinder", aCyl, THE_VIEWS[v], View(v), v == 0 ? 3 : 0, 1000);
    RecData("torus", aTorus, THE_VIEWS[v], View(v), v == 0 ? 3 : 0, 1000);
    RecData("box", aBox, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
    RecData("sphere", aSph, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
    RecData("cone", aCone, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 1000);
  }
  // the hlr group's shapes, in the views their tests use (the first in axonometry with isolines)
  TopoDS_Shape aH1 = ReadShape(theDataDir, "bug25908_hlr-bspline-clone2-tcl-h1.brep");
  for (int v = 0; v < 7; v++)
  {
    RecData("bspline-clone2", aH1, THE_VIEWS[v], View(v), v == 0 ? 2 : 0, 25);
  }
  RecData("bus1", ReadShape(theDataDir, "bug25813_hlr-bus1-draw-Fusion001.brep"), "axo", View(0), 0, 25);
  RecData("normandc1m1", ReadShape(theDataDir, "bug25813_hlr-test_normandc1m1-draw-Cut001.brep"), "axo", View(0), 2, 25);
  RecData("fillet2", ReadShape(theDataDir, "bug25813_hlr-fillet2-tcl-f.brep"), "axo", View(0), 0, 25);
  RecData("bug32747", ReadShape(theDataDir, "bug32747.brep"), "front", View(3), 0, 25);
  RecData("bug28242", ReadShape(theDataDir, "bug28242.brep"), "axo", View(0), 0, 25);
}
