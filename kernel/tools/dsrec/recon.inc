// dsrec --recon (unit 7R): included by main.cpp. Each case records, after its data structure, native's result
// (BRepTools::Write) and history: NbSurfaces, NewFaces of each surface, IsDeleted and Modified of each input face,
// Generated of each input edge and vertex, as indices in TopExp::MapShapes(result, FACE) (0: not in the result). The
// port's reconstruction (ChFi3d_BOPReconstruction) is compared with these geometrically
// (tests/OcctNet.ModelingAlgorithms.Tests/ChFi3d/ReconstructionNativeTests.cs).

#include <BRepPrimAPI_MakeWedge.hxx>

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
  OS << "history " << nS << " " << aIF.Extent() << " " << aIE.Extent() << " " << aIV.Extent() << "\n";
  for (int i = 1; i <= nS; i++)
    OS << "newfaces " << i << " " << aList(theMk.Builder()->NewFaces(i)) << "\n";
  for (int i = 1; i <= aIF.Extent(); i++)
  {
    bool aDel = theMk.IsDeleted(aIF(i));
    OS << "modified " << i << " " << (aDel ? 1 : 0) << " " << aList(theMk.Modified(aIF(i))) << "\n";
  }
  for (int i = 1; i <= aIE.Extent(); i++)
    OS << "generated E " << i << " " << aList(theMk.Generated(aIE(i))) << "\n";
  for (int i = 1; i <= aIV.Extent(); i++)
    OS << "generated V " << i << " " << aList(theMk.Generated(aIV(i))) << "\n";
  return OS.str();
}

static void RecordReconCases()
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
}
