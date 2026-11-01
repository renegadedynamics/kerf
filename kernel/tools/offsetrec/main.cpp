// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// offsetrec: records native OCCT's offset foundations, which build123d.NET doesn't bind (unit 7E):
// BRepOffset_Analyse's edge types, BRepOffset_Offset's offset faces and BRepOffset_MakeSimpleOffset's results, on
// primitives and Boolean and fillet results built here (their BRep text is recorded as the input) and on data/occ's
// shapes. Built against build123d.NET's native OCCT V8_0_1 (tools/offsetrec/build.ps1).
//
// Inputs: the BRep files listed one per line in the file given as the first argument; the recording goes to the file
// given as the second argument. Per input:
//   INPUT <name> <n>                    a generated input: n lines of its BRep text (format 3), or
//   FILE <name>                         a data/occ file (the tests find it there)
//   ANA <angle> <n>                     BRepOffset_Analyse(S, angle): n lines, one per edge of TopExp::MapShapes(EDGE):
//                                       "<index> <nb intervals> (<first> <last> <type>)*" (doubles in hex; or THROW)
//   EXPLODE <n> <faces>...              Explode(L, ChFiDS_Tangential): the number of faces of each compound
//   OFF <face> <offset>                 BRepOffset_Offset(F, offset) on face <face> of MapShapes(FACE), then
//   STATUS <s> <n>                      the status and n lines of the face's BRep text, or THROW <type>
//   SIMPLE <offset> <solid>             BRepOffset_MakeSimpleOffset with the build-solid flag (on a fresh copy), then
//   DONE <0|1> <error> <n>              n lines of the result's BRep text (0 if not done), or THROW <type>
//   THROW <type>                        in place of a line above if the call threw
//
//   offsetrec inputs.txt offsetrec.txt
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffset_Analyse.hxx>
#include <BRepOffset_MakeSimpleOffset.hxx>
#include <BRepOffset_Offset.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom_BSplineSurface.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_Ax2.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::ofstream gOut;

// Writes "<head> <n>" and the n lines of the text.
static void Block(const std::string& theHead, const std::string& theText)
{
  int aNb = 0;
  for (char c : theText)
  {
    aNb += c == '\n' ? 1 : 0;
  }
  if (!theText.empty() && theText.back() != '\n')
  {
    ++aNb;
  }
  gOut << theHead << " " << aNb << "\n" << theText;
  if (!theText.empty() && theText.back() != '\n')
  {
    gOut << "\n";
  }
}

static std::string Text(const TopoDS_Shape& theS)
{
  std::ostringstream aS;
  BRepTools::Write(theS, aS, true, false, TopTools_FormatVersion_VERSION_3);
  return aS.str();
}

static void Throw(const Standard_Failure& e)
{
  gOut << "THROW " << e.ExceptionType() << "\n";
}

static void Analyse(const TopoDS_Shape& theS, double theAngle)
{
  try
  {
    OCC_CATCH_SIGNALS
    BRepOffset_Analyse         anA(theS, theAngle);
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges;
    TopExp::MapShapes(theS, TopAbs_EDGE, anEdges);
    std::ostringstream aS;
    for (int i = 1; i <= anEdges.Extent(); i++)
    {
      const NCollection_List<BRepOffset_Interval>& aL = anA.Type(TopoDS::Edge(anEdges(i)));
      aS << i << " " << aL.Extent();
      for (NCollection_List<BRepOffset_Interval>::Iterator it(aL); it.More(); it.Next())
      {
        aS << " " << Hex(it.Value().First()) << " " << Hex(it.Value().Last()) << " " << (int)it.Value().Type();
      }
      aS << "\n";
    }
    Block("ANA " + Hex(theAngle), aS.str());
    NCollection_List<TopoDS_Shape> aCompounds;
    anA.Explode(aCompounds, ChFiDS_Tangential);
    gOut << "EXPLODE " << aCompounds.Extent();
    for (NCollection_List<TopoDS_Shape>::Iterator it(aCompounds); it.More(); it.Next())
    {
      int aNb = 0;
      for (TopExp_Explorer anExp(it.Value(), TopAbs_FACE); anExp.More(); anExp.Next())
      {
        ++aNb;
      }
      gOut << " " << aNb;
    }
    gOut << "\n";
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

static void Offsets(const TopoDS_Shape& theS, double theOffset)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(theS, TopAbs_FACE, aFaces);
  for (int i = 1; i <= aFaces.Extent() && i <= 30; i++)
  {
    for (double anOffset : {theOffset, -theOffset})
    {
      gOut << "OFF " << i << " " << Hex(anOffset) << "\n";
      try
      {
        OCC_CATCH_SIGNALS
        BRepOffset_Offset anOff(TopoDS::Face(aFaces(i)), anOffset);
        Block("STATUS " + std::to_string((int)anOff.Status()), Text(anOff.Face()));
      }
      catch (const Standard_Failure& e)
      {
        Throw(e);
      }
    }
  }
}

// MakeSimpleOffset writes into its input natively (the walls' pcurves on the free edges, vertex tolerances), so each
// run gets a fresh copy of the input, read again from its text: no run sees what an earlier one wrote.
static void Simple(const std::string& theText, double theOffset)
{
  for (double anOffset : {theOffset, -theOffset})
  {
    for (int aSolid = 0; aSolid <= 1; aSolid++)
    {
      gOut << "SIMPLE " << Hex(anOffset) << " " << aSolid << "\n";
      try
      {
        OCC_CATCH_SIGNALS
        std::istringstream aIn(theText);
        TopoDS_Shape       aS;
        BRep_Builder       aB;
        BRepTools::Read(aS, aIn, aB);
        BRepOffset_MakeSimpleOffset aMaker(aS, anOffset);
        aMaker.SetBuildSolidFlag(aSolid == 1);
        aMaker.Perform();
        Block("DONE " + std::string(aMaker.IsDone() ? "1" : "0") + " " + std::to_string((int)aMaker.GetError()),
              aMaker.IsDone() ? Text(aMaker.GetResultShape()) : std::string());
      }
      catch (const Standard_Failure& e)
      {
        Throw(e);
      }
    }
  }
}

// A twentieth of the diagonal of the shape's box, rounded to two significant digits
static double OffsetValue(const TopoDS_Shape& theS)
{
  Bnd_Box aBox;
  BRepBndLib::Add(theS, aBox);
  if (aBox.IsVoid())
  {
    return 1.0;
  }
  double aD = std::sqrt(aBox.SquareExtent()) / 20.0;
  double aScale = std::pow(10.0, std::floor(std::log10(aD)) - 1.0);
  return std::round(aD / aScale) * aScale;
}

static void Record(const TopoDS_Shape& theS, const std::string& theText, bool theIsSmall)
{
  double anOffset = OffsetValue(theS);
  Analyse(theS, 0.01);
  Analyse(theS, Precision::Angular());
  Offsets(theS, anOffset);
  if (theIsSmall)
  {
    Simple(theText, anOffset);
  }
}

static TopoDS_Shape BSplineFace()
{
  NCollection_Array2<gp_Pnt> aPoles(1, 4, 1, 4);
  for (int i = 1; i <= 4; i++)
  {
    for (int j = 1; j <= 4; j++)
    {
      aPoles(i, j) = gp_Pnt(i * 3.0, j * 3.0, std::sin(i * 1.3) * std::cos(j * 0.7) * 2.0);
    }
  }
  NCollection_Array1<double>    aKnots(1, 2);
  NCollection_Array1<int> aMults(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  aMults(1) = 4;
  aMults(2) = 4;
  occ::handle<Geom_BSplineSurface> aS = new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
  return BRepBuilderAPI_MakeFace(aS, Precision::Confusion()).Face();
}

static std::vector<std::pair<std::string, TopoDS_Shape>> Generated()
{
  std::vector<std::pair<std::string, TopoDS_Shape>> aRes;
  TopoDS_Shape aBox = BRepPrimAPI_MakeBox(10, 20, 30).Shape();
  TopoDS_Shape aCyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(5, 10, -5), gp::DZ()), 4, 40).Shape();
  aRes.push_back({"box", aBox});
  aRes.push_back({"cylinder", BRepPrimAPI_MakeCylinder(5, 10).Shape()});
  aRes.push_back({"cone", BRepPrimAPI_MakeCone(5, 2, 8).Shape()});
  aRes.push_back({"cone-apex", BRepPrimAPI_MakeCone(4, 0, 6).Shape()});
  aRes.push_back({"sphere", BRepPrimAPI_MakeSphere(6).Shape()});
  aRes.push_back({"torus", BRepPrimAPI_MakeTorus(10, 3).Shape()});
  aRes.push_back({"bspline-face", BSplineFace()});
  aRes.push_back({"nurbs-cylinder", BRepBuilderAPI_NurbsConvert(BRepPrimAPI_MakeCylinder(5, 10).Shape()).Shape()});
  aRes.push_back({"box-fuse-cylinder", BRepAlgoAPI_Fuse(aBox, aCyl).Shape()});
  aRes.push_back({"box-cut-cylinder", BRepAlgoAPI_Cut(aBox, aCyl).Shape()});
  aRes.push_back({"box-common-sphere",
                  BRepAlgoAPI_Common(aBox, BRepPrimAPI_MakeSphere(gp_Pnt(5, 10, 15), 14).Shape()).Shape()});
  {
    gp_Trsf aT;
    aT.SetRotation(gp_Ax1(gp_Pnt(5, 10, 0), gp::DZ()), M_PI / 4);
    TopoDS_Shape aBox2 = BRepBuilderAPI_Transform(BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 20), 10, 20, 20).Shape(), aT).Shape();
    aRes.push_back({"box-fuse-rotated-box", BRepAlgoAPI_Fuse(aBox, aBox2).Shape()});
  }
  {
    BRepFilletAPI_MakeFillet aF(aBox);
    for (TopExp_Explorer anExp(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      aF.Add(2.0, TopoDS::Edge(anExp.Current()));
    }
    aRes.push_back({"box-filleted", aF.Shape()});
  }
  {
    BRepFilletAPI_MakeFillet aF(aBox);
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges;
    TopExp::MapShapes(aBox, TopAbs_EDGE, anEdges);
    aF.Add(3.0, TopoDS::Edge(anEdges(1)));
    aF.Add(3.0, TopoDS::Edge(anEdges(2)));
    aRes.push_back({"box-two-fillets", aF.Shape()});
  }
  {
    TopoDS_Compound aC;
    BRep_Builder    aB;
    aB.MakeCompound(aC);
    int i = 0;
    for (TopExp_Explorer aF(aBox, TopAbs_FACE); aF.More() && i < 3; aF.Next(), i++)
    {
      aB.Add(aC, aF.Current());
    }
    aRes.push_back({"three-faces", aC});
  }
  return aRes;
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "usage: offsetrec inputs.txt offsetrec.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  gOut.open(argv[2], std::ios::binary);
  for (const auto& aPair : Generated())
  {
    // the input is the shape read back from its text, as the port reads it
    std::string        aText = Text(aPair.second);
    std::istringstream aIn(aText);
    TopoDS_Shape       aS;
    BRep_Builder       aB;
    BRepTools::Read(aS, aIn, aB);
    Block("INPUT " + aPair.first, aText);
    Record(aS, aText, true);
  }
  std::ifstream aList(argv[1]);
  std::string   aPath;
  while (std::getline(aList, aPath))
  {
    if (!aPath.empty() && aPath.back() == '\r')
    {
      aPath.pop_back();
    }
    if (aPath.empty())
    {
      continue;
    }
    TopoDS_Shape aS;
    BRep_Builder aB;
    if (!BRepTools::Read(aS, aPath.c_str(), aB) || aS.IsNull())
    {
      continue;
    }
    size_t aSlash = aPath.find_last_of("/\\");
    gOut << "FILE " << aPath.substr(aSlash == std::string::npos ? 0 : aSlash + 1) << "\n";
    std::ifstream aF(aPath, std::ios::binary | std::ios::ate);
    Record(aS, Text(aS), aF.tellg() <= 30000);
  }
  return 0;
}
