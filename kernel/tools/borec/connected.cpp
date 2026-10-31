// borec's connected area (unit 6G): BOPAlgo_MakePeriodic and BOPAlgo_MakeConnected. The corpus's command sequences
// (makeperiodic, repeatshape, clearrepetitions; makeconnected, cmakeperiodic, crepeatshape, cclearrepetitions) are
// replayed as BOPTest's commands run them, on one periodicity maker and one connection tool per case, and so are
// random sequences on primitives. After each step the recording holds a dump the port's
// tests/OcctNet.ModelingAlgorithms.Tests/BOPAlgo/BOPAlgo_ConnectedNativeTests.cs writes the same way: the alerts, the
// result's BRep text, the history (common.hxx's HistoryText), and the twins (MakePeriodic) or the materials and
// origins (MakeConnected) of every result sub-shape as indices. The query commands (periodictwins, cmaterialson,
// cperiodictwins) aren't replayed: the dump lists all twins and materials.
//
// Recording: per case "case <name>", its calls as in the capture file (words, then the shapes' BRep text), each
// followed by "dump <length>" and the dump; "endcase".
#include "common.hxx"

#include <BOPAlgo_MakeConnected.hxx>
#include <BOPAlgo_MakePeriodic.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <cstdlib>

using namespace borec;

namespace
{
const char* THE_COMMANDS[] = {"makeperiodic", "repeatshape", "clearrepetitions", "makeconnected", "cmakeperiodic",
                              "crepeatshape", "cclearrepetitions"};

bool IsReplayed(const std::string& theCommand)
{
  for (const char* aC : THE_COMMANDS)
    if (theCommand == aC)
      return true;
  return false;
}

int DirID(const std::string& theKey)
{
  std::string aK = theKey;
  for (char& c : aK)
    c = (char)std::tolower((unsigned char)c);
  return aK == "-x" ? 0 : aK == "-y" ? 1 : aK == "-z" ? 2 : -1;
}

std::string Indices(const NCollection_List<TopoDS_Shape>&                            theList,
                    const NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>& theMap,
                    bool                                                                 theSort)
{
  std::vector<int> aV;
  for (NCollection_List<TopoDS_Shape>::Iterator it(theList); it.More(); it.Next())
    aV.push_back(theMap.FindIndex(it.Value()));
  if (theSort)
    std::sort(aV.begin(), aV.end());
  std::string aS;
  for (int i : aV)
    aS += " " + std::to_string(i);
  return aS;
}

// The replay state of a case: BOPTest's static tools
struct State
{
  BOPAlgo_MakePeriodic      Periodic;
  std::vector<TopoDS_Shape> PeriodicArgs;
  BOPAlgo_MakeConnected     Connected;
  std::vector<TopoDS_Shape> ConnectedArgs;
};

std::string Dump(const occ::handle<Message_Report>&    theReport,
                 const TopoDS_Shape&                   theResult,
                 const std::vector<TopoDS_Shape>&      theArgs,
                 const occ::handle<BRepTools_History>& theHistory,
                 const std::string&                    theExtra)
{
  std::ostringstream anOut;
  anOut << "errors" << Alerts(theReport, Message_Fail) << "\nwarnings" << Alerts(theReport, Message_Warning) << "\n";
  WriteShape(anOut, "result", theResult);
  WriteBlock(anOut, "history", HistoryText(theArgs, theResult, theHistory));
  WriteBlock(anOut, "extra", theExtra);
  return anOut.str();
}

// The twins of every sub-shape of the result, sorted (their order follows an address-ordered map)
std::string Twins(const BOPAlgo_MakePeriodic& theMaker, const TopoDS_Shape& theResult)
{
  std::string aS;
  if (theResult.IsNull())
    return aS;
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aMap;
  TopExp::MapShapes(theResult, aMap);
  for (int i = 1; i <= aMap.Extent(); ++i)
  {
    const NCollection_List<TopoDS_Shape>& aT = theMaker.GetTwins(aMap(i));
    if (!aT.IsEmpty())
      aS += "t " + std::to_string(i) + ":" + Indices(aT, aMap, true) + "\n";
  }
  return aS;
}

// The materials on both sides of every sub-shape of the result and its origins, as indices of the arguments' map
std::string Materials(BOPAlgo_MakeConnected& theTool, const TopoDS_Shape& theResult, const std::vector<TopoDS_Shape>& theArgs)
{
  std::string aS;
  if (theResult.IsNull())
    return aS;
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aMap, anArgMap;
  TopExp::MapShapes(theResult, aMap);
  for (const TopoDS_Shape& anArg : theArgs)
    TopExp::MapShapes(anArg, anArgMap);
  for (int i = 1; i <= aMap.Extent(); ++i)
  {
    const NCollection_List<TopoDS_Shape>& aP = theTool.MaterialsOnPositiveSide(aMap(i));
    const NCollection_List<TopoDS_Shape>& aN = theTool.MaterialsOnNegativeSide(aMap(i));
    const NCollection_List<TopoDS_Shape>& aO = theTool.GetOrigins(aMap(i));
    if (aP.IsEmpty() && aN.IsEmpty() && aO.IsEmpty())
      continue;
    aS += "m " + std::to_string(i) + " +" + Indices(aP, anArgMap, false) + " -" + Indices(aN, anArgMap, false) + " o"
          + Indices(aO, anArgMap, false) + "\n";
  }
  return aS;
}

// Runs one call as BOPTest does; returns its dump, or "" for a call that isn't replayed
std::string Run(State& theState, const CapturedCall& theCall)
{
  const std::vector<std::string>& a    = theCall.Words;
  const std::string&              aCmd = a[0];
  const int                       n    = (int)a.size();
  auto                            aShape = [&](const std::string& theName) {
    auto it = theCall.Shapes.find(theName);
    return it == theCall.Shapes.end() ? TopoDS_Shape() : it->second;
  };
  if (aCmd == "makeperiodic")
  {
    BOPAlgo_MakePeriodic& aM = theState.Periodic;
    TopoDS_Shape          aS = aShape(a[2]);
    aM.Clear();
    aM.SetShape(aS);
    theState.PeriodicArgs = {aS};
    for (int i = 3; i < n;)
    {
      int anID = DirID(a[i]);
      aM.MakePeriodic(anID, true, std::atof(a[++i].c_str()));
      ++i;
      if (n > i + 1 && a[i] == "-trim")
      {
        aM.SetTrimmed(anID, false, std::atof(a[++i].c_str()));
        ++i;
      }
    }
    aM.Perform();
    TopoDS_Shape aR = aM.HasErrors() ? TopoDS_Shape() : aM.Shape();
    return Dump(aM.GetReport(), aR, theState.PeriodicArgs, aM.History(), Twins(aM, aR));
  }
  if (aCmd == "repeatshape" || aCmd == "clearrepetitions")
  {
    BOPAlgo_MakePeriodic& aM = theState.Periodic;
    TopoDS_Shape          aR;
    if (aCmd == "repeatshape")
    {
      for (int i = 2; i < n; ++i)
      {
        int anID = DirID(a[i]);
        aM.RepeatShape(anID, std::atoi(a[++i].c_str()));
      }
      aR = aM.HasErrors() ? TopoDS_Shape() : aM.RepeatedShape();
    }
    else
    {
      aM.ClearRepetitions();
      aR = aM.Shape();
    }
    return Dump(aM.GetReport(), aR, theState.PeriodicArgs, aM.History(), Twins(aM, aR));
  }
  BOPAlgo_MakeConnected& aT = theState.Connected;
  if (aCmd == "makeconnected")
  {
    aT.Clear();
    theState.ConnectedArgs.clear();
    for (int i = 2; i < n; ++i)
    {
      TopoDS_Shape aS = aShape(a[i]);
      if (!aS.IsNull())
      {
        aT.AddArgument(aS);
        theState.ConnectedArgs.push_back(aS);
      }
    }
    aT.Perform();
    TopoDS_Shape aR = aT.HasErrors() ? TopoDS_Shape() : aT.Shape();
    return Dump(aT.GetReport(), aR, theState.ConnectedArgs, aT.History(), Materials(aT, aR, theState.ConnectedArgs));
  }
  if (aCmd == "cmakeperiodic")
  {
    BOPAlgo_MakePeriodic::PeriodicityParams aParams;
    for (int i = 2; i < n;)
    {
      int anID                   = DirID(a[i]);
      aParams.myPeriodic[anID]   = true;
      aParams.myPeriod[anID]     = std::atof(a[++i].c_str());
      ++i;
      if (n > i + 1 && a[i] == "-trim")
      {
        aParams.myIsTrimmed[anID]   = false;
        aParams.myPeriodFirst[anID] = std::atof(a[++i].c_str());
        ++i;
      }
    }
    aT.MakePeriodic(aParams);
  }
  else if (aCmd == "crepeatshape")
  {
    for (int i = 2; i < n; ++i)
    {
      int anID = DirID(a[i]);
      aT.RepeatShape(anID, std::atoi(a[++i].c_str()));
    }
  }
  else if (aCmd == "cclearrepetitions")
  {
    aT.ClearRepetitions();
  }
  else
  {
    return "";
  }
  TopoDS_Shape aR = aT.HasErrors() ? TopoDS_Shape() : aT.PeriodicShape();
  return Dump(aT.GetReport(), aR, theState.ConnectedArgs, aT.History(),
              Materials(aT, aR, theState.ConnectedArgs) + Twins(aT.PeriodicityTool(), aR));
}

void WriteCall(std::ostream& theOut, const CapturedCall& theCall)
{
  theOut << "call " << theCall.Words.size() << "\n";
  for (const std::string& aW : theCall.Words)
    theOut << "w " << aW << "\n";
  for (const auto& aS : theCall.ShapeTexts)
    WriteBlock(theOut, "shape " + aS.first, aS.second);
}

void Record(std::ostream& theOut, const std::string& theName, const std::vector<CapturedCall>& theCalls)
{
  theOut << "case " << theName << "\n";
  State aState;
  for (const CapturedCall& aCall : theCalls)
  {
    if (!IsReplayed(aCall.Words[0]))
      continue;
    WriteCall(theOut, aCall);
    std::string aDump;
    try
    {
      aDump = Run(aState, aCall);
    }
    catch (Standard_Failure&)
    {
      aDump = "throws\n";
    }
    WriteBlock(theOut, "dump", aDump);
  }
  theOut << "endcase\n";
}

std::string G(double theValue)
{
  char aBuf[40];
  std::snprintf(aBuf, sizeof(aBuf), "%.17g", theValue);
  return aBuf;
}

CapturedCall Call(const std::vector<std::string>& theWords, const std::vector<TopoDS_Shape>& theShapes = {})
{
  CapturedCall aCall;
  aCall.Words = theWords;
  for (size_t i = 0; i < theShapes.size(); ++i)
  {
    std::string aKey          = "s" + std::to_string(i + 1);
    aCall.ShapeTexts[aKey]    = BRepText(theShapes[i]);
    aCall.Shapes[aKey]        = ReadBRep(aCall.ShapeTexts[aKey]);
  }
  return aCall;
}

const char* DIRS[] = {"-x", "-y", "-z"};

// Random periodicity: a primitive, 1 to 3 periodic directions with periods about its size, trimmed or not, then
// repetitions, their clearing and a repetition again
void RandomPeriodic(std::ostream& theOut, Random& theR, int theIndex)
{
  int          aKind = theIndex % 5;
  TopoDS_Shape aS    = theR.Placed(theR.Primitive(aKind == 4 ? 0 : aKind), 0.5);
  Bnd_Box      aBox;
  BRepBndLib::Add(aS, aBox);
  double x0, y0, z0, x1, y1, z1;
  aBox.Get(x0, y0, z0, x1, y1, z1);
  double                   aLo[3] = {x0, y0, z0}, aHi[3] = {x1, y1, z1};
  std::vector<std::string> aW = {"makeperiodic", "r", "s1"};
  std::vector<int>         aDirs;
  for (int d = 0; d < 3; ++d)
  {
    if (theR.Next() < 0.55 || (d == 2 && aDirs.empty()))
    {
      aDirs.push_back(d);
      double aSize   = aHi[d] - aLo[d];
      bool   aTrim   = theR.Next() < 0.6;
      double aPeriod = aTrim ? aSize * theR.Range(0.3, 0.8) : aSize * theR.Range(0.9, 1.3);
      aW.push_back(DIRS[d]);
      aW.push_back(G(aPeriod));
      if (aTrim)
      {
        aW.push_back("-trim");
        aW.push_back(G(aLo[d] + aSize * theR.Range(0.05, 0.3)));
      }
    }
  }
  std::vector<CapturedCall> aCalls = {Call(aW, {aS})};
  std::vector<std::string>  aRep   = {"repeatshape", "r"};
  for (int d : aDirs)
  {
    aRep.push_back(DIRS[d]);
    aRep.push_back(std::to_string(theR.Next() < 0.5 ? 1 : (theR.Next() < 0.5 ? 2 : -1)));
  }
  aCalls.push_back(Call(aRep));
  aCalls.push_back(Call({"clearrepetitions", "r"}));
  aCalls.push_back(Call({"repeatshape", "r", DIRS[aDirs[0]], "-2"}));
  Record(theOut, "random/periodic-" + std::to_string(theIndex), aCalls);
}

// Random connection: two or three boxes stacked side by side (touching, sizes random), or a primitive overlapping a
// box; connected, made periodic over their extent, repeated, cleared
void RandomConnected(std::ostream& theOut, Random& theR, int theIndex)
{
  std::vector<TopoDS_Shape> aShapes;
  double                    aX = 0.0;
  const double              aDY = theR.Range(1.0, 3.0), aDZ = theR.Range(1.0, 3.0);
  if (theIndex % 3 != 2)
  {
    int aNb = 2 + theIndex % 2;
    for (int i = 0; i < aNb; ++i)
    {
      double aDX = theR.Range(0.5, 2.0);
      double aY0 = theR.Next() < 0.5 ? 0.0 : theR.Range(-0.4, 0.4);
      aShapes.push_back(BRepPrimAPI_MakeBox(gp_Pnt(aX, aY0, 0.0), aDX, aDY, aDZ).Shape());
      aX += aDX;
    }
  }
  else
  {
    aX = theR.Range(2.0, 3.0);
    aShapes.push_back(BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), aX, aDY, aDZ).Shape());
    aShapes.push_back(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(aX * 0.5, aDY * 0.5, -0.5), gp_Dir(0, 0, 1)),
                                               theR.Range(0.2, 0.45) * std::min(aX, aDY), aDZ + 1.0)
                        .Shape());
  }
  std::vector<std::string> aW = {"makeconnected", "c"};
  for (size_t i = 0; i < aShapes.size(); ++i)
    aW.push_back("s" + std::to_string(i + 1));
  std::vector<CapturedCall> aCalls = {Call(aW, aShapes)};
  std::vector<std::string>  aP     = {"cmakeperiodic", "cp", "-x", G(aX)};
  if (theR.Next() < 0.5)
  {
    aP.push_back("-y");
    aP.push_back(G(aDY));
  }
  else
  {
    aP.push_back("-z");
    aP.push_back(G(aDZ * theR.Range(0.6, 0.9)));
    aP.push_back("-trim");
    aP.push_back(G(aDZ * 0.05));
  }
  aCalls.push_back(Call(aP));
  aCalls.push_back(Call({"crepeatshape", "cr", "-x", theR.Next() < 0.5 ? "1" : "-2", aP[4], "1"}));
  aCalls.push_back(Call({"cclearrepetitions", "cc"}));
  Record(theOut, "random/connected-" + std::to_string(theIndex), aCalls);
}
} // namespace

int RecordConnected(const std::vector<CapturedCase>& theCases, std::ostream& theOut)
{
  for (const CapturedCase& aCase : theCases)
    Record(theOut, aCase.Name, aCase.Calls);
  Random aR(0x6a0c0ffee1234567ull);
  for (int i = 0; i < 60; ++i)
    RandomPeriodic(theOut, aR, i);
  for (int i = 0; i < 60; ++i)
    RandomConnected(theOut, aR, i);
  return 0;
}
