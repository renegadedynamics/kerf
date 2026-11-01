// hlrrec, the algorithms (unit 7Q2): records the exact hidden-line removal (HLRBRep_Algo with HLRBRep_HLRToShape),
// the polygonal one (HLRBRep_PolyAlgo with HLRBRep_PolyHLRToShape) and HLRAppli_ReflectLines on primitives and on
// the hlr group's shapes in fixed projectors: every input (meshed for the polygonal runs) as BRep text, and every
// result compound as BRep text. Included by main.cpp; run with a fourth argument (the output).
// Every run reads its input again from the text, so both sides start from the same shape (the outlining writes into
// its input natively). tests/OcctNet.ModelingAlgorithms.Tests/HLRBRep/HLRBRepAlgoNativeTests.cs runs the same calls
// on the port.

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <HLRAppli_ReflectLines.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <HLRBRep_PolyAlgo.hxx>
#include <HLRBRep_PolyHLRToShape.hxx>
#include <HLRBRep_TypeOfResultingEdge.hxx>
#include <map>
#include <sstream>

static std::map<std::string, std::string> gAlgoInputs;

static std::string AlgoText(const TopoDS_Shape& theS)
{
  if (theS.IsNull())
  {
    return std::string();
  }
  std::ostringstream aS;
  BRepTools::Write(theS, aS, true, false, TopTools_FormatVersion_VERSION_3);
  return aS.str();
}

static void Block(const std::string& theHead, const std::string& theText)
{
  int aNb = 0;
  for (char c : theText)
  {
    if (c == '\n')
    {
      ++aNb;
    }
  }
  if (!theText.empty() && theText.back() != '\n')
  {
    ++aNb;
  }
  std::fprintf(gOut, "%s %d\n%s", theHead.c_str(), aNb, theText.c_str());
  if (!theText.empty() && theText.back() != '\n')
  {
    std::fprintf(gOut, "\n");
  }
}

static void AddAlgoInput(const std::string& theName, const TopoDS_Shape& theS)
{
  std::string aText = AlgoText(theS);
  gAlgoInputs[theName] = aText;
  Block("INPUT " + theName, aText);
}

static TopoDS_Shape AlgoInput(const std::string& theName)
{
  std::istringstream aIn(gAlgoInputs.at(theName));
  TopoDS_Shape       aS;
  BRep_Builder       aB;
  BRepTools::Read(aS, aIn, aB);
  return aS;
}

// A meshed copy of an input, as a new input (the polygonal algorithm reads the triangulations).
static void AddMeshedInput(const std::string& theName, double theDefl, bool theRelative)
{
  TopoDS_Shape aS = AlgoInput(theName);
  BRepMesh_IncrementalMesh(aS, theDefl, theRelative, 0.5, false);
  AddAlgoInput(theName + "-mesh", aS);
}

static void Shape(const char* theKind, const TopoDS_Shape& theS)
{
  Block(std::string("SHAPE ") + theKind, AlgoText(theS));
}

static void Exc(const Standard_Failure& e)
{
  std::fprintf(gOut, "EXCEPTION %s\n", e.ExceptionType());
}

static TopoDS_Shape FirstFace(const TopoDS_Shape& theS)
{
  TopExp_Explorer anExp(theS, TopAbs_FACE);
  return anExp.More() ? anExp.Current() : TopoDS_Shape();
}

// The exact hidden-line removal of the inputs (several: each hides the others), every compound of HLRBRep_HLRToShape.
static void RecExact(const std::string& theCase, const std::vector<std::string>& theInputs, int theView, int nbIso)
{
  std::fprintf(gOut, "CASE exact %s %s %d\n", theCase.c_str(), THE_VIEWS[theView], nbIso);
  try
  {
    occ::handle<HLRBRep_Algo> anAlgo = new HLRBRep_Algo();
    std::vector<TopoDS_Shape> aShapes;
    for (const std::string& aName : theInputs)
    {
      aShapes.push_back(AlgoInput(aName));
      anAlgo->Add(aShapes.back(), nbIso);
    }
    anAlgo->Projector(View(theView));
    anAlgo->Update();
    anAlgo->Hide();
    HLRBRep_HLRToShape aToShape(anAlgo);
    Shape("V", aToShape.VCompound());
    Shape("V1", aToShape.Rg1LineVCompound());
    Shape("VN", aToShape.RgNLineVCompound());
    Shape("VO", aToShape.OutLineVCompound());
    Shape("VI", aToShape.IsoLineVCompound());
    Shape("H", aToShape.HCompound());
    Shape("H1", aToShape.Rg1LineHCompound());
    Shape("HN", aToShape.RgNLineHCompound());
    Shape("HO", aToShape.OutLineHCompound());
    Shape("HI", aToShape.IsoLineHCompound());
    Shape("VO3d", aToShape.OutLineVCompound3d());
    Shape("V3d", aToShape.CompoundOfEdges(HLRBRep_Sharp, true, true));
    Shape("H13d", aToShape.CompoundOfEdges(HLRBRep_Rg1Line, false, true));
    // one shape's results (by its bounds), and one face's (explored)
    Shape("V-shape", aToShape.VCompound(aShapes.front()));
    Shape("H-face", aToShape.HCompound(FirstFace(aShapes.front())));
    Shape("VO-face", aToShape.OutLineVCompound(FirstFace(aShapes.front())));
  }
  catch (Standard_Failure& e)
  {
    Exc(e);
  }
}

// The polygonal hidden-line removal of meshed inputs, every compound of HLRBRep_PolyHLRToShape, hidden and shown.
static void RecPoly(const std::string& theCase, const std::vector<std::string>& theInputs, int theView,
                    bool theFull = true)
{
  std::fprintf(gOut, "CASE poly %s %s\n", theCase.c_str(), THE_VIEWS[theView]);
  try
  {
    occ::handle<HLRBRep_PolyAlgo> anAlgo = new HLRBRep_PolyAlgo();
    std::vector<TopoDS_Shape>     aShapes;
    for (const std::string& aName : theInputs)
    {
      aShapes.push_back(AlgoInput(aName));
      anAlgo->Load(aShapes.back());
    }
    anAlgo->Projector(View(theView));
    anAlgo->Update();
    HLRBRep_PolyHLRToShape aToShape;
    aToShape.Update(anAlgo);
    Shape("V", aToShape.VCompound());
    Shape("V1", aToShape.Rg1LineVCompound());
    Shape("VN", aToShape.RgNLineVCompound());
    Shape("VO", aToShape.OutLineVCompound());
    Shape("H", aToShape.HCompound());
    Shape("H1", aToShape.Rg1LineHCompound());
    Shape("HN", aToShape.RgNLineHCompound());
    Shape("HO", aToShape.OutLineHCompound());
    if (!theFull)
    {
      return;
    }
    Shape("V-shape", aToShape.VCompound(aShapes.front()));
    Shape("H-shape", aToShape.HCompound(aShapes.front()));
    aToShape.Show();
    Shape("S", aToShape.VCompound());
    Shape("SO", aToShape.OutLineVCompound());
    aToShape.Hide();
    Shape("V-again", aToShape.VCompound());
    Shape("OutLined", anAlgo->OutLinedShape(aShapes.front()));
  }
  catch (Standard_Failure& e)
  {
    Exc(e);
  }
}

// HLRAppli_ReflectLines, as the reflectlines, hlrin3d and hlrin2d commands use it.
static void RecReflect(const std::string& theName, double Nx, double Ny, double Nz)
{
  std::fprintf(gOut, "CASE reflect %s %.17g %.17g %.17g\n", theName.c_str(), Nx, Ny, Nz);
  try
  {
    gp_Ax2                aAxes(gp_Pnt(0., 0., 0.), gp_Dir(Nx, Ny, Nz));
    gp_Dir                aDX = aAxes.XDirection();
    HLRAppli_ReflectLines aReflector(AlgoInput(theName));
    aReflector.SetAxes(Nx, Ny, Nz, 0., 0., 0., aDX.X(), aDX.Y(), aDX.Z());
    aReflector.Perform();
    Shape("Result", aReflector.GetResult());
    Shape("Sharp3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, true, true));
    Shape("Smooth3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Rg1Line, true, true));
    Shape("Sharp2d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, true, false));
    Shape("OutLine2d", aReflector.GetCompoundOf3dEdges(HLRBRep_OutLine, true, false));
    Shape("Hidden3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, false, true));
  }
  catch (Standard_Failure& e)
  {
    Exc(e);
  }
}

static void RecAlgoAll(const char* theDataDir)
{
  // primitives, a Boolean of two cylinders (hlr bug25813_1) and a box with a hole
  AddAlgoInput("cylinder", BRepPrimAPI_MakeCylinder(10., 30.).Shape());
  AddAlgoInput("torus", BRepPrimAPI_MakeTorus(30., 10.).Shape());
  AddAlgoInput("box", BRepPrimAPI_MakeBox(gp_Pnt(-5, -10, -2), 10., 20., 30.).Shape());
  AddAlgoInput("sphere", BRepPrimAPI_MakeSphere(gp_Pnt(3, 2, 1), 8.).Shape());
  AddAlgoInput("cone", BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 1)), 10., 4., 20.).Shape());
  {
    TopoDS_Shape aC1 = BRepPrimAPI_MakeCylinder(10., 30.).Shape();
    gp_Trsf      aT;
    aT.SetTranslation(gp_Vec(0., 0., 2.));
    TopoDS_Shape aC2 = BRepBuilderAPI_Transform(BRepPrimAPI_MakeCylinder(8., 50.).Shape(), aT).Shape();
    AddAlgoInput("fused", BRepAlgoAPI_Fuse(aC1, aC2).Shape());
  }
  AddAlgoInput("bspline-clone2", ReadShape(theDataDir, "bug25908_hlr-bspline-clone2-tcl-h1.brep"));
  AddAlgoInput("bus1", ReadShape(theDataDir, "bug25813_hlr-bus1-draw-Fusion001.brep"));
  AddAlgoInput("normandc1m1", ReadShape(theDataDir, "bug25813_hlr-test_normandc1m1-draw-Cut001.brep"));
  AddAlgoInput("fillet2", ReadShape(theDataDir, "bug25813_hlr-fillet2-tcl-f.brep"));
  AddAlgoInput("bug32747", ReadShape(theDataDir, "bug32747.brep"));
  AddAlgoInput("bug28242", ReadShape(theDataDir, "bug28242.brep"));

  const char* const aPrims[] = {"cylinder", "torus", "box", "sphere", "cone", "fused"};
  for (const char* aName : aPrims)
  {
    AddMeshedInput(aName, 0.5, false);
  }
  const char* const aData[] = {"bspline-clone2", "bus1", "normandc1m1", "fillet2", "bug32747", "bug28242"};
  for (const char* aName : aData)
  {
    AddMeshedInput(aName, 0.01, true);
  }

  for (const char* aName : aPrims)
  {
    for (int v = 0; v < 8; v++)
    {
      RecExact(aName, {aName}, v, v == 0 ? 2 : 0);
      RecPoly(aName, {std::string(aName) + "-mesh"}, v);
    }
  }
  RecExact("box+sphere", {"box", "sphere"}, 0, 0);
  RecExact("cone+torus", {"cone", "torus"}, 3, 1);
  RecPoly("box+sphere", {"box-mesh", "sphere-mesh"}, 0);
  RecPoly("cone+torus", {"cone-mesh", "torus-mesh"}, 3);

  for (int v = 0; v < 7; v++)
  {
    RecExact("bspline-clone2", {"bspline-clone2"}, v, 0);
    if (v == 0 || v == 3)
    {
      RecPoly("bspline-clone2", {"bspline-clone2-mesh"}, v, false);
    }
  }
  const char* const aOthers[] = {"bus1", "normandc1m1", "fillet2", "bug32747", "bug28242"};
  for (const char* aName : aOthers)
  {
    RecExact(aName, {aName}, 0, 0);
    RecExact(aName, {aName}, 3, 0);
    RecPoly(aName, {std::string(aName) + "-mesh"}, 0, false);
    if (std::string(aName) != "bus1")
    {
      RecPoly(aName, {std::string(aName) + "-mesh"}, 3, false);
    }
  }

  RecReflect("cylinder", 1., -1., 1.);
  RecReflect("torus", 0., 0., 1.);
  RecReflect("fused", 1., 1., 1.);
  RecReflect("bug32747", 0., -1., 0.);
  RecReflect("normandc1m1", 1., -1., 1.);
}
