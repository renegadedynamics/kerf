// dsrec --recon (unit 7R): included by main.cpp. Each case records, after its data structure, native's result
// (BRepTools::Write) and history: NbSurfaces, NewFaces of each surface, IsDeleted and Modified of each input face,
// Generated of each input edge and vertex, as indices in TopExp::MapShapes(result, FACE) (0: not in the result). The
// port's reconstruction (ChFi3d_BOPReconstruction) is compared with these geometrically
// (tests/OcctNet.ModelingAlgorithms.Tests/ChFi3d/ReconstructionNativeTests.cs).

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <cmath>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <functional>
#include <gp_Elips.hxx>
#include <gp_Parab.hxx>

static GeomAbs_CurveType CurveType(const TopoDS_Edge& E)
{
  return BRepAdaptor_Curve(E).GetType();
}

static gp_Pnt MidPoint(const TopoDS_Edge& E)
{
  BRepAdaptor_Curve C(E);
  return C.Value(0.5 * (C.FirstParameter() + C.LastParameter()));
}

// the edges of S (TopExp::MapShapes numbers) where the predicate holds, each with the radius (distance) r
static std::vector<Edge> EdgesWhere(const TopoDS_Shape& S, double r, std::function<bool(const TopoDS_Edge&)> thePred)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  std::vector<Edge> aRes;
  for (int k = 1; k <= M.Extent(); k++)
  {
    TopoDS_Edge E = TopoDS::Edge(M(k));
    if (!BRep_Tool::Degenerated(E) && thePred(E))
      aRes.push_back({k, r, -1});
  }
  return aRes;
}

template <class Maker>
static std::string ResultText(Maker& theMk, const TopoDS_Shape& theInput)
{
  std::ostringstream OS;
  if (!theMk.IsDone())
  {
    OS << "result 0\n";
    return OS.str();
  }
  const TopoDS_Shape& aRes = theMk.Shape();
  std::ostringstream  aBRep;
  BRepTools::Write(aRes, aBRep);
  std::string aB = aBRep.str();
  if (!aB.empty() && aB.back() != '\n')
  {
    aB += '\n';
  }
  int nB = 0;
  for (char c : aB)
    nB += c == '\n';
  OS << "result " << nB << "\n" << aB;
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aRF, aIF, aIE, aIV;
  TopExp::MapShapes(aRes, TopAbs_FACE, aRF);
  TopExp::MapShapes(theInput, TopAbs_FACE, aIF);
  TopExp::MapShapes(theInput, TopAbs_EDGE, aIE);
  TopExp::MapShapes(theInput, TopAbs_VERTEX, aIV);
  auto aList = [&](const NCollection_List<TopoDS_Shape>& L) {
    std::string s = std::to_string(L.Extent());
    for (NCollection_List<TopoDS_Shape>::Iterator it(L); it.More(); it.Next())
      s += " " + std::to_string(aRF.FindIndex(it.Value()));
    return s;
  };
  int nS = theMk.Builder()->DataStructure()->NbSurfaces(); // the HBuilder's
  int nC = theMk.Builder()->DataStructure()->NbCurves();
  OS << "history " << nS << " " << aIF.Extent() << " " << aIE.Extent() << " " << aIV.Extent() << " " << nC << "\n";
  for (int i = 1; i <= nS; i++)
    OS << "newfaces " << i << " " << aList(theMk.Builder()->NewFaces(i)) << "\n";
  // the edges of each curve, by their lengths (they need not be in the result: SetRegul reads them)
  for (int i = 1; i <= nC; i++)
  {
    const NCollection_List<TopoDS_Shape>& L = theMk.Builder()->NewEdges(i);
    OS << "newedges " << i << " " << L.Extent();
    for (NCollection_List<TopoDS_Shape>::Iterator it(L); it.More(); it.Next())
    {
      char b[64];
      std::snprintf(b, sizeof(b), " %.9g", GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(TopoDS::Edge(it.Value()))));
      OS << b;
    }
    OS << "\n";
  }
  for (int i = 1; i <= aIF.Extent(); i++)
  {
    bool aDel = theMk.IsDeleted(aIF(i));
    OS << "modified " << i << " " << (aDel ? 1 : 0) << " " << aList(theMk.Modified(aIF(i))) << "\n";
  }
  for (int i = 1; i <= aIE.Extent(); i++)
    OS << "generated E " << i << " " << aList(theMk.Generated(aIE(i))) << "\n";
  for (int i = 1; i <= aIV.Extent(); i++)
    OS << "generated V " << i << " " << aList(theMk.Generated(aIV(i))) << "\n";
  // unit 7R3: the history of the input's solids (IsDeleted, Modified as indices in MapShapes(result, SOLID))
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aIS, aRS;
  TopExp::MapShapes(theInput, TopAbs_SOLID, aIS);
  TopExp::MapShapes(aRes, TopAbs_SOLID, aRS);
  for (int i = 1; i <= aIS.Extent(); i++)
  {
    const NCollection_List<TopoDS_Shape>& L = theMk.Modified(aIS(i));
    OS << "msolid " << i << " " << (theMk.IsDeleted(aIS(i)) ? 1 : 0) << " " << L.Extent();
    for (NCollection_List<TopoDS_Shape>::Iterator it(L); it.More(); it.Next())
      OS << " " << aRS.FindIndex(it.Value());
    OS << "\n";
  }
  return OS.str();
}

static void RecordComplexCases(const char* theDataDir);
static void RecordCorpusCases(const char* theDataDir);

static void RecordReconCases(const char* theDataDir)
{
  const gp_Ax1 aZ(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  const double D = M_PI / 180.;
  // tests/blend/simple (Draw's "explode s E" numbers the edges as TopExp::MapShapes)
  TopoDS_Shape aBox  = BRepPrimAPI_MakeBox(100, 100, 100).Shape();
  TopoDS_Shape aFlat = BRepPrimAPI_MakeBox(100, 100, 10).Shape();
  Record("A2", aBox, false, {{5, 10, -1}, {6, 10, -1}, {10, 10, -1}});
  Record("A3", aFlat, false, {{5, 10, -1}});
  Record("A4", aFlat, false, {{5, 10, -1}, {6, 5, -1}, {10, 5, -1}});
  TopoDS_Shape aW1 = BRepPrimAPI_MakeWedge(40, 100, 100, 20, 20, 70, 70).Shape();
  Record("A5", aW1, false, {{5, 10, -1}});
  Record("A6", aW1, false, {{5, 10, -1}, {6, 10, -1}, {10, 10, -1}});
  TopoDS_Shape aW2 = BRepPrimAPI_MakeWedge(40, 100, 100, 0).Shape();
  Record("A7", aW2, false, {{5, 10, -1}});
  Record("A8", aW2, false, {{5, 10, -1}, {6, 10, -1}, {9, 10, -1}});
  TopoDS_Shape aPC90  = BRepPrimAPI_MakeCylinder(50, 100, 90 * D).Shape();
  TopoDS_Shape aPC270 = BRepPrimAPI_MakeCylinder(50, 100, 270 * D).Shape();
  Record("A9", Rotated(aPC90, aZ, 90), false, {{9, 10, -1}});
  Record("B2", Rotated(aPC90, aZ, 270), false, {{1, 10, -1}});
  Record("B3", Rotated(aPC90, aZ, 270), false, {{1, 10, -1}, {4, 10, -1}, {5, 10, -1}});
  Record("B4", aPC270, false, {{9, 10, -1}});
  Record("B5", aPC270, false, {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("B6", aPC270, false, {{2, 10, -1}});
  Record("B7", aPC270, false, {{2, 10, -1}, {1, 10, -1}, {6, 10, -1}});
  TopoDS_Shape aCo90  = BRepPrimAPI_MakeCone(90, 40, 150, 90 * D).Shape();
  TopoDS_Shape aCo270 = BRepPrimAPI_MakeCone(90, 40, 150, 270 * D).Shape();
  Record("B8", Rotated(aCo90, aZ, 90), false, {{9, 10, -1}});
  Record("B9", Rotated(aCo90, aZ, 90), false, {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("C2", Rotated(aCo90, aZ, 270), false, {{3, 10, -1}, {2, 10, -1}, {7, 10, -1}});
  Record("C3", aCo270, false, {{9, 10, -1}});
  Record("C4", aCo270, false, {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("C5", aCo270, false, {{1, 10, -1}});
  Record("C6", aCo270, false, {{1, 10, -1}, {2, 10, -1}, {6, 10, -1}});
  TopoDS_Shape aApex = BRepPrimAPI_MakeCone(50, 0, 120, 90 * D).Shape();
  Record("C7", Rotated(aApex, aZ, 90), false, {{7, 10, -1}});
  Record("C8", Rotated(aApex, aZ, 90), false, {{7, 10, -1}, {2, 10, -1}, {4, 10, -1}});
  Record("C9", Rotated(aApex, aZ, 270), false, {{3, 10, -1}});
  TopoDS_Shape aSph = Scaled(Rotated(BRepPrimAPI_MakeSphere(15, -60 * D, 60 * D, 90 * D).Shape(), aZ, 90), 10);
  Record("D2", aSph, false, {{9, 10, -1}});
  Record("D3", aSph, false, {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("D4", aSph, false, {{2, 10, -1}});

  // chamfers on the box's other edges and on a cylinder
  Record("chamfer_box_two_edges", aBox, true, {{5, 10, -1}, {6, 10, -1}});
  Record("chamfer_box_all_edges", aBox, true,
         {{1, 5, -1}, {2, 5, -1}, {3, 5, -1}, {4, 5, -1}, {5, 5, -1}, {6, 5, -1}, {7, 5, -1}, {8, 5, -1},
          {9, 5, -1}, {10, 5, -1}, {11, 5, -1}, {12, 5, -1}});
  Record("chamfer_A9", Rotated(aPC90, aZ, 90), true, {{9, 10, -1}});

  // tests/blend/simple D5 to F9
  Record("D5", aSph, false, {{2, 10, -1}, {1, 10, -1}, {6, 10, -1}});
  TopoDS_Shape aSph270 = Scaled(BRepPrimAPI_MakeSphere(15, -60 * D, 60 * D, 270 * D).Shape(), 10);
  Record("D6", aSph270, false, {{9, 10, -1}});
  Record("D7", aSph270, false, {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("D8", aSph270, false, {{1, 10, -1}});
  Record("D9", aSph270, false, {{1, 10, -1}, {6, 10, -1}, {2, 10, -1}});
  TopoDS_Shape aSphS = Scaled(Rotated(BRepPrimAPI_MakeSphere(15, -90 * D, 60 * D, 90 * D).Shape(), aZ, 90), 10);
  Record("E2", aSphS, false, {{7, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("E3", aSphS, false, {{2, 10, -1}});
  Record("E4", aSphS, false, {{2, 10, -1}, {7, 10, -1}, {4, 10, -1}});
  TopoDS_Shape aTor = Scaled(BRepPrimAPI_MakeTorus(10, 10, 0, 45 * D, 90 * D).Shape(), 10);
  Record("E5", aTor, false, {{4, 10, -1}});
  Record("E6", aTor, false, {{4, 10, -1}, {8, 10, -1}, {3, 10, -1}});
  Record("E7", aTor, false, {{1, 10, -1}});
  Record("E8", aTor, false, {{1, 10, -1}, {4, 10, -1}, {5, 10, -1}});
  TopoDS_Shape aTor2 = Scaled(BRepPrimAPI_MakeTorus(20, 5, 0, 160 * D, 90 * D).Shape(), 10);
  Record("E9", aTor2, false, {{4, 10, -1}});
  Record("F2", aTor2, false, {{8, 10, -1}});
  Record("F3", aTor2, false, {{8, 10, -1}, {4, 10, -1}, {3, 10, -1}});
  {
    gp_Elips    anEl(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 15, 10);
    TopoDS_Edge aE1 = BRepBuilderAPI_MakeEdge(anEl, 0, M_PI).Edge();
    BRepBuilderAPI_MakePolygon aPoly(gp_Pnt(-15, 0, 0), gp_Pnt(0, -5, 0), gp_Pnt(15, 0, 0));
    BRepBuilderAPI_MakeWire    aW(aE1);
    aW.Add(aPoly.Wire());
    TopoDS_Shape aPr = Scaled(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(), gp_Vec(2, 0, 10)).Shape(), 10);
    Record("F4", aPr, false, {{1, 10, -1}});
    Record("F5", aPr, false, {{1, 10, -1}, {4, 10, -1}, {9, 10, -1}});
    Record("F6", aPr, false, {{5, 10, -1}});
    Record("F7", aPr, false, {{5, 10, -1}, {7, 10, -1}, {9, 10, -1}});
  }
  {
    gp_Parab    aPa(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 0, 1)), 50);
    TopoDS_Edge aE1 = BRepBuilderAPI_MakeEdge(aPa, -100, 100).Edge();
    TopoDS_Edge aE2 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, -100, 50), gp_Pnt(0, 100, 50)).Edge();
    BRepBuilderAPI_MakeWire aW(aE1, aE2);
    TopoDS_Shape aPr = Scaled(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(), gp_Vec(20, 0, 100)).Shape(), 10);
    Record("F8", aPr, false, {{2, 10, -1}});
    Record("F9", aPr, false, {{2, 10, -1}, {6, 10, -1}});
  }

  // tests/blend/simple G1 to H9: parabolic, Bezier and B-spline prisms, revolutions
  {
    gp_Parab    aPa(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 0, 1)), 50);
    TopoDS_Edge aE1 = BRepBuilderAPI_MakeEdge(aPa, -100, 100).Edge();
    TopoDS_Edge aE2 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, -100, 50), gp_Pnt(0, 100, 50)).Edge();
    BRepBuilderAPI_MakeWire aW(aE1, aE2);
    TopoDS_Shape aPr = Scaled(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(), gp_Vec(20, 0, 100)).Shape(), 10);
    Record("G1", aPr, false, {{4, 1000, -1}});
    Record("G2", aPr, false, {{4, 10, -1}, {6, 10, -1}, {1, 10, -1}});
  }
  {
    NCollection_Array1<gp_Pnt> aP(1, 5);
    aP(1) = gp_Pnt(0, 0, 0);
    aP(2) = gp_Pnt(20, 0, 0);
    aP(3) = gp_Pnt(20, 5, 0);
    aP(4) = gp_Pnt(25, 10, 0);
    aP(5) = gp_Pnt(10, 20, 0);
    occ::handle<Geom_BezierCurve> aBz = new Geom_BezierCurve(aP);
    BRepBuilderAPI_MakeWire       aW(BRepBuilderAPI_MakeEdge(aBz).Edge(),
                               BRepBuilderAPI_MakeEdge(gp_Pnt(10, 20, 0), gp_Pnt(0, 0, 0)).Edge());
    TopoDS_Shape aPr = Scaled(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(), gp_Vec(0, 2, 10)).Shape(), 10);
    Record("G3", aPr, false, {{6, 10, -1}});
    Record("G4", aPr, false, {{6, 10, -1}, {1, 10, -1}});
    Record("G5", aPr, false, {{4, 10, -1}});
    Record("G6", aPr, false, {{4, 10, -1}, {1, 10, -1}, {6, 10, -1}});
  }
  {
    NCollection_Array1<gp_Pnt> aP(1, 4);
    aP(1) = gp_Pnt(0, 0, 0);
    aP(2) = gp_Pnt(2, 10, 0);
    aP(3) = gp_Pnt(4, 6, 0);
    aP(4) = gp_Pnt(10, 0, 0);
    NCollection_Array1<double> aK(1, 2);
    aK(1) = -1;
    aK(2) = 1;
    NCollection_Array1<int> aM(1, 2);
    aM(1) = 4;
    aM(2) = 4;
    occ::handle<Geom_BSplineCurve> aBs = new Geom_BSplineCurve(aP, aK, aM, 3);
    BRepBuilderAPI_MakeWire        aW(BRepBuilderAPI_MakeEdge(aBs).Edge(),
                               BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(0, 0, 0)).Edge());
    TopoDS_Shape aPr = Scaled(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(), gp_Vec(0, 2, 10)).Shape(), 10);
    Record("G7", aPr, false, {{1, 5, -1}});
    Record("G8", aPr, false, {{1, 5, -1}, {6, 5, -1}});
    Record("G9", aPr, false, {{4, 5, -1}});
    Record("H1", aPr, false, {{4, 5, -1}, {6, 5, -1}, {2, 5, -1}});
  }
  {
    gp_Elips    anEl(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 15, 10);
    TopoDS_Edge aE1 = BRepBuilderAPI_MakeEdge(anEl, 0, M_PI / 2).Edge();
    aE1             = TopoDS::Edge(Rotated(aE1, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), 90));
    BRepBuilderAPI_MakePolygon aPoly(gp_Pnt(0, 0, 10), gp_Pnt(0, 0, -10), gp_Pnt(25, 0, -10), gp_Pnt(15, 0, 0));
    BRepBuilderAPI_MakeWire    aW(aE1);
    aW.Add(aPoly.Wire());
    TopoDS_Shape aRv = Scaled(BRepPrimAPI_MakeRevol(BRepBuilderAPI_MakeFace(aW.Wire(), true).Face(),
                                                    gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 270 * D)
                                .Shape(),
                              10);
    Record("H2", aRv, false, {{1, 10, -1}});
    Record("H3", aRv, false, {{1, 10, -1}, {4, 10, -1}, {9, 10, -1}});
    Record("H4", aRv, false, {{10, 10, -1}});
    Record("H5", aRv, false, {{10, 10, -1}, {4, 10, -1}, {3, 10, -1}});
  }
  {
    BRepBuilderAPI_MakePolygon aPoly;
    for (const gp_Pnt& p : {gp_Pnt(0, 0, 25), gp_Pnt(20, 0, 5), gp_Pnt(5, 0, 5), gp_Pnt(5, 0, -5), gp_Pnt(20, 0, -5),
                            gp_Pnt(0, 0, -25)})
      aPoly.Add(p);
    aPoly.Close();
    TopoDS_Shape aRv = Scaled(BRepPrimAPI_MakeRevol(BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Face(),
                                                    gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 270 * D)
                                .Shape(),
                              10);
    Record("H6", aRv, false, {{8, 10, -1}});
    Record("H7", aRv, false, {{8, 10, -1}, {10, 10, -1}, {13, 10, -1}});
    Record("H8", aRv, false, {{4, 10, -1}});
    Record("H9", aRv, false, {{4, 10, -1}, {2, 10, -1}, {7, 10, -1}});
  }

  // build123d.NET's fillets and chamfers (tests/Build123d.Tests: Operations, Topology)
  {
    TopoDS_Shape      aB = BRepPrimAPI_MakeBox(1, 2, 3).Shape();
    std::vector<Edge> all;
    for (int k = 1; k <= 12; k++)
      all.push_back({k, 0.2, -1});
    Record("b3d_box_1x2x3_all_edges", aB, false, all); // OperationsGenericTests.Fillet3D
    std::vector<Edge> allc;
    for (int k = 1; k <= 12; k++)
      allc.push_back({k, 0.2, -1});
    Record("b3d_box_1x2x3_chamfer_all_edges", aB, true, allc); // Chamfer3D
  }
  {
    TopoDS_Shape aC = BRepPrimAPI_MakeCylinder(1, 1).Shape(); // ExtrudeThenChamfer: both circles
    Record("b3d_cylinder_chamfer_circles", aC, true,
           EdgesWhere(aC, 0.3, [](const TopoDS_Edge& E) { return CurveType(E) == GeomAbs_Circle; }));
    Record("b3d_cylinder_fillet_circles", aC, false,
           EdgesWhere(aC, 0.3, [](const TopoDS_Edge& E) { return CurveType(E) == GeomAbs_Circle; }));
  }
  {
    // SelectorTests: (Box(20, 10, 4) - Cylinder(2, 10)), the top face's lines, radius 1: the hole's wire untouched
    TopoDS_Shape aB = BRepPrimAPI_MakeBox(gp_Pnt(-10, -5, -2), 20, 10, 4).Shape();
    TopoDS_Shape aC = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -5), gp_Dir(0, 0, 1)), 2, 10).Shape();
    TopoDS_Shape aCut = BRepAlgoAPI_Cut(aB, aC).Shape();
    Record("b3d_box_minus_cylinder_top_lines", aCut, false, EdgesWhere(aCut, 1, [](const TopoDS_Edge& E) {
             return CurveType(E) == GeomAbs_Line && MidPoint(E).Z() > 1.9;
           }));
    // the hole's rim
    Record("b3d_box_minus_cylinder_hole_rim", aCut, false, EdgesWhere(aCut, 1, [](const TopoDS_Edge& E) {
             return CurveType(E) == GeomAbs_Circle && MidPoint(E).Z() > 1.9;
           }));
    // all the top edges
    Record("b3d_box_minus_cylinder_top", aCut, false,
           EdgesWhere(aCut, 1, [](const TopoDS_Edge& E) { return MidPoint(E).Z() > 1.9; }));
  }
  {
    // Shape1DTests.EdgesTopoParent: a plate with a window, the window's edges filleted (concave)
    TopoDS_Shape aP = BRepPrimAPI_MakeBox(gp_Pnt(-40, -75, 0), 80, 150, 2).Shape();
    TopoDS_Shape aW = BRepPrimAPI_MakeBox(gp_Pnt(-32.5, 42.5, -1), 25, 25, 4).Shape();
    TopoDS_Shape aCase = BRepAlgoAPI_Cut(aP, aW).Shape();
    Record("b3d_phone_case_window", aCase, false, EdgesWhere(aCase, 1, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 1.9 && P.X() > -33 && P.X() < -7 && P.Y() > 42 && P.Y() < 68;
           }));
    Record("b3d_phone_case_vertical_window_edges", aCase, false, EdgesWhere(aCase, 1, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 0.5 && P.Z() < 1.5 && P.X() > -33 && P.X() < -7 && P.Y() > 42 && P.Y() < 68;
           }));
  }
  {
    // FaceTests.IsConvexConcave: an open box (Offset with an opening), all its edges
    TopoDS_Shape                         aB = BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, -2.5), 20, 20, 5).Shape();
    NCollection_List<TopoDS_Shape>       aOpen;
    for (TopExp_Explorer ex(aB, TopAbs_FACE); ex.More(); ex.Next())
    {
      BRepAdaptor_Surface aS(TopoDS::Face(ex.Current()));
      if (aS.GetType() == GeomAbs_Plane && std::abs(aS.Plane().Location().Z() - 2.5) < 1e-9)
        aOpen.Append(ex.Current());
    }
    BRepOffsetAPI_MakeThickSolid aThick;
    aThick.MakeThickSolidByJoin(aB, aOpen, -2, 1.e-4, BRepOffset_Skin, false, false, GeomAbs_Arc);
    TopoDS_Shape anOpenBox = aThick.Shape();
    Record("b3d_open_box_all_edges", anOpenBox, false,
           EdgesWhere(anOpenBox, 0.5, [](const TopoDS_Edge&) { return true; }));
  }

  // a shell (a box without its top), and a compound of two solids
  {
    BRep_Builder    B;
    TopoDS_Shell    aSh;
    B.MakeShell(aSh);
    for (TopExp_Explorer ex(aBox, TopAbs_FACE); ex.More(); ex.Next())
    {
      BRepAdaptor_Surface aS(TopoDS::Face(ex.Current()));
      if (!(aS.GetType() == GeomAbs_Plane && std::abs(aS.Plane().Location().Z() - 100) < 1e-9))
        B.Add(aSh, ex.Current());
    }
    Record("shell_box_without_top", aSh, false, EdgesWhere(aSh, 10, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 1 && P.Z() < 99 && P.X() > 99 && P.Y() < 1;
           }));
    TopoDS_Compound aCo;
    B.MakeCompound(aCo);
    B.Add(aCo, aBox);
    gp_Trsf aT;
    aT.SetTranslation(gp_Vec(200, 0, 0));
    B.Add(aCo, BRepBuilderAPI_Transform(aBox, aT, true).Shape());
    // a solid with a void (two shells): an edge of the outer shell, then an edge of the void (concave)
    TopoDS_Shape aVoid = BRepAlgoAPI_Cut(aBox, BRepPrimAPI_MakeBox(gp_Pnt(30, 30, 30), 40, 40, 40).Shape()).Shape();
    Record("solid_with_void_outer_edge", aVoid, false, EdgesWhere(aVoid, 10, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 1 && P.Z() < 99 && P.Y() < 1 && P.X() > 99;
           }));
    Record("solid_with_void_inner_edge", aVoid, false, EdgesWhere(aVoid, 5, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 31 && P.Z() < 69 && std::abs(P.Y() - 30) < 1 && std::abs(P.X() - 70) < 1;
           }));
    Record("compound_two_boxes", aCo, false, EdgesWhere(aCo, 10, [](const TopoDS_Edge& E) {
             gp_Pnt P = MidPoint(E);
             return P.Z() > 1 && P.Z() < 99 && P.Y() < 1 && (std::abs(P.X() - 100) < 1 || std::abs(P.X() - 300) < 1);
           }));
  }

  // corners of four faces: a pyramid's apex (ChFi3d_Builder::PerformMoreThreeCorner, the plate filling)
  {
    TopoDS_Shape aPy = BRepPrimAPI_MakeWedge(100, 100, 100, 50, 50, 50, 50).Shape();
    auto         anApex = [](const TopoDS_Edge& E) {
      BRepAdaptor_Curve C(E);
      return C.Value(C.FirstParameter()).Y() > 99 || C.Value(C.LastParameter()).Y() > 99;
    };
    Record("pyramid_apex_edges", aPy, false, EdgesWhere(aPy, 10, anApex));
    Record("pyramid_all_edges", aPy, false, EdgesWhere(aPy, 5, [](const TopoDS_Edge&) { return true; }));
    Record("chamfer_pyramid_apex_edges", aPy, true, EdgesWhere(aPy, 5, anApex));
  }
  // a chamfer by a distance and an angle (build123d.NET's PartChamferAsymAngle)
  {
    TopoDS_Shape      aB    = BRepPrimAPI_MakeBox(10, 10, 10).Shape();
    std::vector<Edge> aVert = EdgesWhere(aB, 1, [](const TopoDS_Edge& E) {
      BRepAdaptor_Curve C(E);
      return C.GetType() == GeomAbs_Line && std::abs(C.Line().Direction().Z()) > 0.5;
    });
    for (Edge& e : aVert)
      e.r2 = -2; // a distance and an angle of 60 degrees (Record's chamfer branch)
    Record("chamfer_box_distance_angle", aB, true, aVert);
  }

  // failures: radii too large (Shape3DTests.Plate, PlainMessage)
  {
    TopoDS_Shape      aPl = BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, -1), 20, 20, 2).Shape();
    Record("fail_plate_radius_3", aPl, false, EdgesWhere(aPl, 3, [](const TopoDS_Edge&) { return true; }));
    TopoDS_Shape aCube = BRepPrimAPI_MakeBox(10, 10, 10).Shape();
    Record("fail_cube_radius_6", aCube, false, EdgesWhere(aCube, 6, [](const TopoDS_Edge&) { return true; }));
  }

  // two of unit 7N2's cases whose reconstructed results BRepCheck rejected (unit 7R2): built as dsrec builds them,
  // through BRep text, so that the data structure is dsrec.txt's
  {
    BRepBuilderAPI_MakePolygon aSq(gp_Pnt(0, 0, 0), gp_Pnt(100, 0, 0), gp_Pnt(100, 100, 0), gp_Pnt(0, 100, 0), true);
    BRepOffsetAPI_ThruSections aTS(true, true);
    aTS.AddWire(aSq.Wire());
    aTS.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(50, 50, 80)).Vertex());
    aTS.Build();
    TopoDS_Shape aPyr = ThroughText(aTS.Shape());
    Record("pyramid_base_edges", aPyr, false, All(EdgesAtZ(aPyr, 0), 5));
    TopoDS_Shape b = ThroughText(CBox(10, 10, 10));
    Record("b3d_fail_box_top_edges_r6", b, false, All(EdgesAtZ(b, 5), 6));
  }

  // unit 7R3: faces with a seam cut by a fillet (unit 7N3's oracle: box common sphere, cylinder T; radius 3% of the
  // box's diagonal as there)
  {
    auto aDiag = [](const TopoDS_Shape& S) {
      Bnd_Box aBB;
      BRepBndLib::Add(S, aBB);
      return std::round(std::sqrt(aBB.SquareExtent()) * 0.03 * 1000.0) / 1000.0;
    };
    TopoDS_Shape aBCS = ThroughText(
      BRepAlgoAPI_Common(BRepPrimAPI_MakeBox(10, 10, 10).Shape(), BRepPrimAPI_MakeSphere(gp_Pnt(5, 5, 5), 6.5).Shape())
        .Shape());
    double r = aDiag(aBCS);
    Record("seam_box_common_sphere_e4", aBCS, false, {{4, r, -1}});
    Record("seam_box_common_sphere_e7", aBCS, false, {{7, r, -1}});
    Record("seam_box_common_sphere_all", aBCS, false, EdgesWhere(aBCS, r, [](const TopoDS_Edge&) { return true; }));
    TopoDS_Shape aT = ThroughText(
      BRepAlgoAPI_Fuse(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 3, 20).Shape(),
                       BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-10, 0, 10), gp_Dir(1, 0, 0)), 2, 20).Shape())
        .Shape());
    r = aDiag(aT);
    Record("seam_cylinder_T_e2", aT, false, {{2, r, -1}});
    Record("seam_cylinder_T_e5", aT, false, {{5, r, -1}});
    Record("seam_cylinder_T_e7", aT, false, {{7, r, -1}});
    Record("seam_cylinder_T_e2_too_large", aT, false, {{2, 28.914, -1}}); // the oracle's "too large" radius
  }

  if (theDataDir != nullptr)
  {
    RecordComplexCases(theDataDir);
    RecordCorpusCases(theDataDir);
  }
}

// tests/blend/complex: the cases whose shapes the dataset has (Draw's "explode a e" numbers edges as MapShapes)
static void RecordComplex(const char* theDataDir, const char* theName, const char* theFile, double theScale,
                          const std::vector<Edge>& theEdges)
{
  TopoDS_Shape S;
  BRep_Builder B;
  std::string  aPath = std::string(theDataDir) + "/brep/" + theFile;
  if (!BRepTools::Read(S, aPath.c_str(), B))
  {
    std::fprintf(stderr, "dsrec: cannot read %s\n", aPath.c_str());
    return;
  }
  if (theScale != 1.0)
  {
    S = Scaled(S, theScale);
  }
  Record(theName, S, false, theEdges);
}

static void RecordComplexCases(const char* theDataDir)
{
  RecordComplex(theDataDir, "complex_A1", "CFI_indusfjm.rle", 1, {{4, 2, -1}, {5, 2, -1}, {15, 2, -1}, {16, 2, -1}});
  RecordComplex(theDataDir, "complex_A2", "CCH_indushem.rle", 1000, {{26, 2, -1}});
  RecordComplex(theDataDir, "complex_A3", "CCH_indushep.rle", 1000, {{8, 5, -1}});
  RecordComplex(theDataDir, "complex_A4", "CCH_indushep.rle", 1000, {{12, 5, -1}});
  RecordComplex(theDataDir, "complex_A5", "CCH_blendhdp.rle", 1000, {{3, 5, -1}});
  RecordComplex(theDataDir, "complex_B2", "CFI_pro10320.rle", 1, {{15, 15, -1}});
  RecordComplex(theDataDir, "complex_B3", "pro11555.rle", 1, {{1, 2, -1}, {2, 2, -1}});
  RecordComplex(theDataDir, "complex_D2", "CFI_pro10391.rle", 1,
                {{3, 10, -1}, {4, 5, -1}, {2, 10, -1}, {13, 5, -1}, {11, 5, -1}, {14, 10, -1}, {15, 5, -1},
                 {12, 10, -1}, {6, 5, -1}, {9, 5, -1}});
  RecordComplex(theDataDir, "complex_D6", "CFI_pro10321.rle", 1, {{13, 59, -1}});
  RecordComplex(theDataDir, "complex_E5", "CFI_pro7788.rle", 1, {{8, 20, -1}, {11, 20, -1}, {1, 20, -1}, {12, 20, -1}});
  RecordComplex(theDataDir, "complex_E7", "CFI_pro11850.rle", 1, {{22, 3, -1}});
  RecordComplex(theDataDir, "complex_E8", "CFI_pro12404.rle", 1, {{8, 10, -1}, {13, 10, -1}});
  RecordComplex(theDataDir, "complex_F1", "CFI_id150009.rle", 1, {{9, 10, -1}});
  RecordComplex(theDataDir, "complex_F2", "CFI_id150009.rle", 1, {{17, 10, -1}});
  RecordComplex(theDataDir, "complex_F4", "CFI_cts20686.rle", 1,
                {{4, 10, -1}, {15, 10, -1}, {13, 10, -1}, {10, 10, -1}, {7, 10, -1}});
  RecordComplex(theDataDir, "complex_F7", "CFI_pro13225.rle", 1, {{3, 10, -1}, {8, 10, -1}, {7, 10, -1}, {6, 10, -1}});
  RecordComplex(theDataDir, "complex_G1", "CFI_pro14035.rle", 1,
                {{9, 3, -1}, {7, 3, -1}, {5, 3, -1}, {3, 3, -1}, {1, 3, -1}, {2, 3, -1}, {4, 3, -1}, {6, 3, -1},
                 {8, 3, -1}});
  RecordComplex(theDataDir, "complex_G2", "CFI_pro14550.rle", 1, {{1, 5, -1}, {3, 5, -1}, {4, 5, -1}, {2, 5, -1}});
}

// unit 7R3: corpus cases that failed end to end with unit 7N3 (DrawFilletCorpusTests.KnownFailures), recorded as Draw's
// blend makes them (its tolerances; .draw files are BRep text after a two-line header) to see whether the
// reconstruction alone reproduces native's result from native's data structure
static TopoDS_Shape ReadData(const char* theDataDir, const char* theFile)
{
  std::ifstream anIn(std::string(theDataDir) + "/brep/" + theFile, std::ios::binary);
  std::stringstream aText;
  aText << anIn.rdbuf();
  std::string s = aText.str();
  if (s.rfind("DBRep_DrawableShape", 0) == 0)
  {
    s = s.substr(s.find("CASCADE Topology"));
  }
  std::istringstream aStream(s);
  TopoDS_Shape       S;
  BRep_Builder       B;
  BRepTools::Read(S, aStream, B);
  return S;
}

static void RecordCorpus(const char* theDataDir, const char* theName, const char* theFile, double theScale,
                         const std::vector<Edge>& theEdges)
{
  TopoDS_Shape S = ReadData(theDataDir, theFile);
  if (S.IsNull())
  {
    std::fprintf(stderr, "dsrec: cannot read %s\n", theFile);
    return;
  }
  if (theScale != 1.0)
  {
    S = Scaled(S, theScale);
  }
  gDrawParams = true;
  Record(theName, S, false, theEdges);
  gDrawParams = false;
}

static void RecordCorpusCases(const char* theDataDir)
{
  RecordCorpus(theDataDir, "corpus_simple_W4", "CCV_1_c12gsf.rle", 1, {{2, 0.2, -1}, {1, 0.2, -1}});
  RecordCorpus(theDataDir, "corpus_simple_Y1", "CCV_1_n12gsq.rle", 1, {{7, 10, -1}});
  RecordCorpus(theDataDir, "corpus_simple_V7", "8_J1.draw", 1000, {{3, 50, -1}});
  RecordCorpus(theDataDir, "corpus_tolblend_simple_C4", "CFI_11_a1few.rle", 5, {{17, 10, -1}});
  RecordCorpus(theDataDir, "corpus_tolblend_simple_C5", "CFI_11_a2fex.rle", 5, {{7, 50, -1}});
  RecordCorpus(theDataDir, "corpus_tolblend_simple_C7", "CFI_11_c1ffa.rle", 5, {{1, 40, -1}});
  RecordCorpus(theDataDir, "corpus_tolblend_simple_C8", "CFI_11_d1ffc.rle", 5, {{8, 100, -1}});
  RecordCorpus(theDataDir, "corpus_bug32929", "bug32929.brep", 1, {{8, 1, -1}});
  RecordCorpus(theDataDir, "corpus_bug354_1", "OCC354a.brep", 1, {{18, 2, -1}});
}
