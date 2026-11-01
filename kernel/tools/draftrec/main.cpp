// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// draftrec: records native OCCT's draft surfaces, rolling-ball blends and evolved volumes, which build123d.NET doesn't
// bind (unit 7G): BRepOffsetAPI_MakeDraft (BRepFill_Draft), BiTgte_Blend and BRepOffsetAPI_MakeEvolved's volume mode
// (BRepFill_AdvancedEvolved), on inputs built here (their BRep text is recorded). Built against build123d.NET's native
// OCCT V8_0_1 (tools/draftrec/build.ps1).
//
// The recording (the file given as the first argument):
//   INPUT <name> <n>                         n lines of an input's BRep text (format 3)
//   DRAFT <input> <angle> <mode> <internal> <round> <stop>
//                                            MakeDraft(input, Z, angle) with SetDraft(internal), SetOptions(round);
//                                            mode L: Perform(length <stop>); S: Perform(plane z = <stop>, keep inside);
//                                            T: Perform(input <stop>, keep outside). Doubles in hex. Then
//   DONE <0|1> <n>                           n lines of the result's BRep text, and
//   SHELL <n>                                n lines of the draft skin's, or
//   THROW <type>                             if a call threw
//   ROLL <input> <radius> <build> <faces>    BiTgte_Blend(input, radius, 1e-4, false), SetFaces of the pairs of faces
//                                            (indices of TopExp::MapShapes(FACE), comma-separated), Perform(build); then
//   SURFACES <nb> <contact types>            NbSurfaces and the ContactType of each, and
//   DONE <0|1> <n>                           the result's BRep text, or THROW <type>
//   EVOL <spine> <profile> <solid>           BRepOffsetAPI_MakeEvolved(spine, profile, Arc, true, solid, false, 1e-7, true);
//   DONE <0|1> <n>                           IsDone and the result's BRep text, or THROW <type>
//
//   draftrec draftrec.txt
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <BiTgte_Blend.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_MakeEvolved.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <gp_Ax3.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::ofstream gOut;

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
  if (theS.IsNull())
  {
    return std::string();
  }
  std::ostringstream aS;
  BRepTools::Write(theS, aS, true, false, TopTools_FormatVersion_VERSION_3);
  return aS.str();
}

static void Throw(const Standard_Failure& e)
{
  gOut << "THROW " << e.ExceptionType() << "\n";
}

// Every run reads its input again from the text: the algorithms may write into it natively.
static std::map<std::string, std::string> gInputs;

static TopoDS_Shape Input(const std::string& theName)
{
  std::istringstream aIn(gInputs.at(theName));
  TopoDS_Shape       aS;
  BRep_Builder       aB;
  BRepTools::Read(aS, aIn, aB);
  return aS;
}

static void AddInput(const std::string& theName, const TopoDS_Shape& theS)
{
  std::string aText = Text(theS);
  gInputs[theName] = aText;
  Block("INPUT " + theName, aText);
}

static TopoDS_Wire Polygon(const std::vector<gp_Pnt>& thePoints, bool theClosed)
{
  BRepBuilderAPI_MakePolygon aP;
  for (const gp_Pnt& p : thePoints)
  {
    aP.Add(p);
  }
  if (theClosed)
  {
    aP.Close();
  }
  return aP.Wire();
}

static TopoDS_Wire Circle(double theR)
{
  return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp::DZ()), theR)).Edge()).Wire();
}

// A rectangle 10 x 6 with its corner at (10, 6) rounded with a radius 2
static TopoDS_Wire RoundedRectangle()
{
  BRepBuilderAPI_MakeWire aW;
  aW.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)).Edge());
  aW.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 4, 0)).Edge());
  occ::handle<Geom_TrimmedCurve> anArc = GC_MakeArcOfCircle(gp_Pnt(10, 4, 0), gp_Pnt(8 + std::sqrt(2.0), 4 + std::sqrt(2.0), 0), gp_Pnt(8, 6, 0)).Value();
  aW.Add(BRepBuilderAPI_MakeEdge(anArc).Edge());
  aW.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(8, 6, 0), gp_Pnt(0, 6, 0)).Edge());
  aW.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 6, 0), gp_Pnt(0, 0, 0)).Edge());
  return aW.Wire();
}

static void Draft(const std::string& theInput, double theAngle, char theMode, bool theInternal, bool theRound,
                  const std::string& theStop, double theValue)
{
  gOut << "DRAFT " << theInput << " " << Hex(theAngle) << " " << theMode << " " << (theInternal ? 1 : 0) << " "
       << (theRound ? 1 : 0) << " " << (theMode == 'T' ? theStop : Hex(theValue)) << "\n";
  try
  {
    OCC_CATCH_SIGNALS
    BRepOffsetAPI_MakeDraft aMk(Input(theInput), gp::DZ(), theAngle);
    if (theRound)
    {
      aMk.SetOptions(BRepBuilderAPI_RoundCorner);
    }
    if (theInternal)
    {
      aMk.SetDraft(true);
    }
    if (theMode == 'L')
    {
      aMk.Perform(theValue);
    }
    else if (theMode == 'S')
    {
      aMk.Perform(new Geom_Plane(gp_Pnt(0, 0, theValue), gp::DZ()), true);
    }
    else
    {
      aMk.Perform(Input(theStop), true);
    }
    Block(std::string("DONE ") + (aMk.IsDone() ? "1" : "0"), Text(aMk.Shape()));
    Block("SHELL", Text(aMk.Shell()));
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

static void Roll(const std::string& theInput, double theRadius, bool theBuild, const std::vector<int>& theEdges)
{
  std::string anEdges;
  for (int i : theEdges)
  {
    anEdges += (anEdges.empty() ? "" : ",") + std::to_string(i);
  }
  gOut << "ROLL " << theInput << " " << Hex(theRadius) << " " << (theBuild ? 1 : 0) << " " << anEdges << "\n";
  try
  {
    OCC_CATCH_SIGNALS
    TopoDS_Shape aS = Input(theInput);
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aMap;
    TopExp::MapShapes(aS, TopAbs_FACE, aMap);
    BiTgte_Blend aRoll;
    aRoll.Init(aS, theRadius, 1.0e-4, false);
    for (size_t i = 0; i + 1 < theEdges.size(); i += 2)
    {
      aRoll.SetFaces(TopoDS::Face(aMap(theEdges[i])), TopoDS::Face(aMap(theEdges[i + 1])));
    }
    aRoll.Perform(theBuild);
    gOut << "SURFACES " << aRoll.NbSurfaces();
    for (int i = 1; i <= aRoll.NbSurfaces(); i++)
    {
      gOut << " " << (int)aRoll.ContactType(i);
    }
    gOut << "\n";
    Block(std::string("DONE ") + (aRoll.IsDone() ? "1" : "0"), Text(aRoll.Shape()));
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

static void Evol(const std::string& theSpine, const std::string& theProfile, bool theSolid)
{
  gOut << "EVOL " << theSpine << " " << theProfile << " " << (theSolid ? 1 : 0) << "\n";
  try
  {
    OCC_CATCH_SIGNALS
    BRepOffsetAPI_MakeEvolved aMk(Input(theSpine), TopoDS::Wire(Input(theProfile)), GeomAbs_Arc, true, theSolid, false,
                                  1.0e-7, true);
    Block(std::string("DONE ") + (aMk.IsDone() ? "1" : "0"), aMk.IsDone() ? Text(aMk.Shape()) : std::string());
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: draftrec draftrec.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  gOut.open(argv[1], std::ios::binary);

  // The draft surfaces: wires, faces and an open shell, along Z
  AddInput("square-wire", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)}, true));
  AddInput("L-wire", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 4, 0), gp_Pnt(4, 4, 0), gp_Pnt(4, 10, 0), gp_Pnt(0, 10, 0)}, true));
  AddInput("circle-wire", Circle(5));
  AddInput("open-wire", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0), gp_Pnt(5, 12, 0)}, false));
  AddInput("rounded-wire", RoundedRectangle());
  AddInput("square-face", BRepBuilderAPI_MakeFace(TopoDS::Wire(Input("square-wire"))).Face());
  AddInput("disk-face", BRepBuilderAPI_MakeFace(TopoDS::Wire(Input("circle-wire"))).Face());
  {
    TopoDS_Shape aBox = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, -5), 10, 10, 5).Shape();
    TopoDS_Shell aShell;
    BRep_Builder aB;
    aB.MakeShell(aShell);
    for (TopExp_Explorer anExp(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
    {
      // every face but the top one (z = 0): the free boundary is the top square
      TopExp_Explorer aV(anExp.Current(), TopAbs_VERTEX);
      bool isTop = true;
      for (; aV.More(); aV.Next())
      {
        isTop = isTop && std::abs(BRep_Tool::Pnt(TopoDS::Vertex(aV.Current())).Z()) < 1.0e-9;
      }
      if (!isTop)
      {
        aB.Add(aShell, anExp.Current());
      }
    }
    AddInput("open-box-shell", aShell);
  }
  AddInput("stop-box", BRepPrimAPI_MakeBox(gp_Pnt(-20, -20, 6), 50, 50, 10).Shape());
  for (const char* anInput : {"square-wire", "L-wire", "circle-wire", "open-wire", "rounded-wire", "square-face", "disk-face", "open-box-shell"})
  {
    for (double anAngle : {5.0 * M_PI / 180.0, 15.0 * M_PI / 180.0})
    {
      Draft(anInput, anAngle, 'L', false, false, "", 8.0);
      Draft(anInput, anAngle, 'L', true, false, "", 8.0);
      Draft(anInput, anAngle, 'L', false, true, "", 8.0);
      Draft(anInput, anAngle, 'S', false, false, "", 6.0);
      Draft(anInput, anAngle, 'T', false, false, "stop-box", 0.0);
    }
  }

  // The rolling ball: single edges, all the edges of a box, and edges of a Boolean result
  AddInput("box", BRepPrimAPI_MakeBox(10, 10, 10).Shape());
  AddInput("box-fuse-cylinder", BRepAlgoAPI_Fuse(BRepPrimAPI_MakeBox(20, 20, 5).Shape(),
                                                 BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(10, 10, 5), gp::DZ()), 4, 8).Shape()).Shape());
  for (std::vector<int> aPair : std::vector<std::vector<int>>{{1, 3}, {1, 5}, {3, 5}, {2, 4}, {2, 6}, {4, 6}, {1, 4}, {2, 5}})
  {
    Roll("box", 2.0, true, aPair);
    Roll("box", 2.0, false, aPair);
  }
  Roll("box", 2.0, true, {1, 3, 3, 5, 1, 5});
  Roll("box", 1.0, true, {1, 3, 2, 4});
  {
    TopoDS_Shape aS = Input("box-fuse-cylinder");
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aMap;
    TopExp::MapShapes(aS, TopAbs_FACE, aMap);
    for (int i = 1; i <= aMap.Extent(); i++)
    {
      for (int j = i + 1; j <= aMap.Extent(); j++)
      {
        Roll("box-fuse-cylinder", 1.0, true, {i, j});
        Roll("box-fuse-cylinder", -1.0, true, {i, j});
      }
    }
  }
  {
    // bug29861: two cylindrical faces
    occ::handle<Geom_CylindricalSurface> aS1 = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 360, 0), gp::DZ(), gp::DY()), 160);
    occ::handle<Geom_CylindricalSurface> aS2 = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp::DY()), 80);
    TopoDS_Compound aC;
    BRep_Builder aB;
    aB.MakeCompound(aC);
    aB.Add(aC, BRepBuilderAPI_MakeFace(aS1, 0, 2 * M_PI, -113.2000012, 113.2000012, 1.0e-7).Face());
    aB.Add(aC, BRepBuilderAPI_MakeFace(aS2, 0, 2 * M_PI, 168.4, 600, 1.0e-7).Face());
    AddInput("two-cylinders", aC);
    Roll("two-cylinders", 30.0, true, {1, 2});
    Roll("two-cylinders", 30.0, false, {1, 2});
    Roll("two-cylinders", 20.0, true, {1, 2});
  }

  // The evolved volumes: planar spines, open profiles
  AddInput("square-spine", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)}, true));
  AddInput("L-spine", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 4, 0), gp_Pnt(4, 4, 0), gp_Pnt(4, 10, 0), gp_Pnt(0, 10, 0)}, true));
  AddInput("circle-spine", Circle(6));
  AddInput("rounded-spine", RoundedRectangle());
  AddInput("vertical-profile", Polygon({gp_Pnt(5, 0, 0), gp_Pnt(5, 0, 1)}, false));
  AddInput("slanted-profile", Polygon({gp_Pnt(-1, -1, 0), gp_Pnt(0, -1.5, 5)}, false));
  AddInput("vee-profile", Polygon({gp_Pnt(0, 0, 0), gp_Pnt(0, -1, 2), gp_Pnt(0, 0, 4)}, false));
  for (const char* aSpine : {"square-spine", "L-spine", "circle-spine", "rounded-spine"})
  {
    for (const char* aProfile : {"vertical-profile", "slanted-profile", "vee-profile"})
    {
      Evol(aSpine, aProfile, true);
      Evol(aSpine, aProfile, false);
    }
  }
  return 0;
}
