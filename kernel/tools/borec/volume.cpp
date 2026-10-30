// borec's volume area (unit 6G): BOPAlgo_MakerVolume and BOPAlgo_CellsBuilder, replayed as Draw's commands do them.
//
// The recording is a list of cases in the capture format (common.hxx), each call followed by native's output block
// ("out <length>"): the captured corpus calls (volumemaker's mkvolume, cells_test's session and bc* commands), then
// random cases written the same way (faces of overlapping primitives to mkvolume, with and without intersection and
// internal parts; cells of overlapping primitives with random add/remove/material sequences). The port's
// BOPAlgo_VolumeNativeTests replays the same calls with the same rules and writes the same output.
//
// A shape word of a call is resolved as both sides resolve it: a text already read gives the same shape again; else a
// sub-shape (with its location and orientation, as Draw's explode gives it) of a shape already read whose own BRep text
// is that text; else the text is read as a new shape. So the cells commands find the arguments, and sub-shapes of
// them, by identity as in Draw.
#include "common.hxx"

#include <BOPAlgo_CellsBuilder.hxx>
#include <BOPAlgo_MakerVolume.hxx>
#include <BOPAlgo_PaveFiller.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <TopExp_Explorer.hxx>
#include <typeinfo>
#include <typeinfo>

using namespace borec;

namespace
{
struct Session
{
  std::vector<std::pair<std::string, TopoDS_Shape>> Read; // texts read, in order
  NCollection_List<TopoDS_Shape>                    Objects, Tools;
  BOPAlgo_PaveFiller*                               PaveFiller = new BOPAlgo_PaveFiller();
  bool                                              HasDS      = false;
  BOPAlgo_CellsBuilder                              Cells;
  bool                                              RunParallel = false, NonDestructive = false;
  double                                            Fuzzy       = Precision::Confusion();
  BOPAlgo_GlueEnum                                  Glue        = BOPAlgo_GlueOff;

  TopoDS_Shape Resolve(const std::string& theText)
  {
    for (const auto& aP : Read)
      if (aP.first == theText)
        return aP.second;
    for (size_t i = 0; i < Read.size(); ++i)
    {
      for (int t = (int)TopAbs_COMPOUND; t <= (int)TopAbs_VERTEX; ++t)
        for (TopExp_Explorer anExp(Read[i].second, (TopAbs_ShapeEnum)t); anExp.More(); anExp.Next())
          if (BRepText(anExp.Current()) == theText)
          {
            Read.emplace_back(theText, anExp.Current());
            return anExp.Current();
          }
    }
    TopoDS_Shape aS = ReadBRep(theText);
    Read.emplace_back(theText, aS);
    return aS;
  }
};

std::string Outcome(const occ::handle<Message_Report>& theReport)
{
  return "errors" + Alerts(theReport, Message_Fail) + "\nwarnings" + Alerts(theReport, Message_Warning) + "\n";
}

std::string ShapeOut(const TopoDS_Shape& theS)
{
  if (theS.IsNull())
    return "null\n";
  std::string aText = BRepText(theS);
  return "shape " + std::to_string(aText.size()) + "\n" + aText;
}

std::vector<TopoDS_Shape> ToVector(const NCollection_List<TopoDS_Shape>& theL)
{
  std::vector<TopoDS_Shape> aV;
  for (NCollection_List<TopoDS_Shape>::Iterator it(theL); it.More(); it.Next())
    aV.push_back(it.Value());
  return aV;
}

int Atoi(const std::string& theS)
{
  return (int)std::atof(theS.c_str());
}

// Runs one call as Draw's command does, and returns the output the port compares.
std::string Run(Session& theS, const CapturedCall& theCall)
{
  const std::vector<std::string>& a = theCall.Words;
  const int                       n = (int)a.size();
  const std::string&              c = a[0];
  auto Shape = [&](int i) -> TopoDS_Shape {
    auto it = theCall.ShapeTexts.find(a[i]);
    return it == theCall.ShapeTexts.end() ? TopoDS_Shape() : theS.Resolve(it->second);
  };
  if (c == "baddobjects" || c == "baddtools")
  {
    for (int i = 1; i < n; ++i)
      (c == "baddobjects" ? theS.Objects : theS.Tools).Append(Shape(i));
    return "";
  }
  if (c == "baddcompound" || c == "baddctools")
  {
    for (TopoDS_Iterator it(Shape(1)); it.More(); it.Next())
      (c == "baddcompound" ? theS.Objects : theS.Tools).Append(it.Value());
    return "";
  }
  if (c == "bclearobjects")
  {
    theS.Objects.Clear();
    return "";
  }
  if (c == "bcleartools")
  {
    theS.Tools.Clear();
    return "";
  }
  if (c == "bclear")
  {
    theS.PaveFiller = new BOPAlgo_PaveFiller(); // OCCT deletes the old one; the recorder lets it leak
    theS.HasDS      = false;
    theS.Objects.Clear();
    theS.Tools.Clear();
    theS.RunParallel = theS.NonDestructive = false;
    theS.Fuzzy                             = Precision::Confusion();
    theS.Glue                              = BOPAlgo_GlueOff;
    return "";
  }
  if (c == "bfuzzyvalue")
  {
    theS.Fuzzy = std::atof(a[1].c_str());
    return "";
  }
  if (c == "brunparallel")
  {
    theS.RunParallel = Atoi(a[1]) != 0;
    return "";
  }
  if (c == "bnondestructive")
  {
    theS.NonDestructive = Atoi(a[1]) != 0;
    return "";
  }
  if (c == "bglue")
  {
    theS.Glue = (BOPAlgo_GlueEnum)Atoi(a[1]);
    return "";
  }
  if (c == "bfillds")
  {
    if (theS.Objects.IsEmpty())
      return "no objects\n";
    NCollection_List<TopoDS_Shape> aLC;
    for (NCollection_List<TopoDS_Shape>::Iterator it(theS.Objects); it.More(); it.Next())
      aLC.Append(it.Value());
    for (NCollection_List<TopoDS_Shape>::Iterator it(theS.Tools); it.More(); it.Next())
      aLC.Append(it.Value());
    BOPAlgo_PaveFiller& aPF = *theS.PaveFiller;
    aPF.SetArguments(aLC);
    aPF.SetRunParallel(theS.RunParallel);
    aPF.SetNonDestructive(true); // the port's arguments are frozen, so always non-destructive (decision 577)
    aPF.SetFuzzyValue(theS.Fuzzy);
    aPF.SetGlue(theS.Glue);
    aPF.Perform();
    theS.HasDS = aPF.PDS() != nullptr;
    return Outcome(aPF.GetReport());
  }
  BOPAlgo_CellsBuilder& aCB = theS.Cells;
  auto CellsOut = [&](const TopoDS_Shape& theR) {
    return Outcome(aCB.GetReport()) + ShapeOut(theR)
           + HistoryText(ToVector(aCB.Arguments()), aCB.Shape(), aCB.History()) + "\n";
  };
  if (c == "bcbuild")
  {
    if (!theS.HasDS)
      return "no filler\n";
    aCB.Clear();
    for (NCollection_List<TopoDS_Shape>::Iterator it(theS.Objects); it.More(); it.Next())
      aCB.AddArgument(it.Value());
    for (NCollection_List<TopoDS_Shape>::Iterator it(theS.Tools); it.More(); it.Next())
      aCB.AddArgument(it.Value());
    aCB.SetRunParallel(theS.RunParallel);
    aCB.SetFuzzyValue(theS.Fuzzy);
    aCB.SetNonDestructive(true); // the port's arguments are frozen, so always non-destructive (decision 577)
    aCB.SetGlue(theS.Glue);
    aCB.PerformWithFiller(*theS.PaveFiller);
    if (aCB.HasErrors())
      return Outcome(aCB.GetReport());
    return Outcome(aCB.GetReport()) + ShapeOut(aCB.GetAllParts())
           + HistoryText(ToVector(aCB.Arguments()), aCB.Shape(), aCB.History()) + "\n";
  }
  if (c == "bcaddall")
  {
    int  iMaterial = 0;
    bool bUpdate   = false;
    if (n > 3)
    {
      if (a[2] == "-m")
        iMaterial = Atoi(a[3]);
      if (n == 5)
        bUpdate = a[4] == "-u";
    }
    aCB.ClearWarnings();
    aCB.AddAllToResult(iMaterial, bUpdate);
    return CellsOut(aCB.Shape());
  }
  if (c == "bcremoveall")
  {
    aCB.RemoveAllFromResult();
    return HistoryText(ToVector(aCB.Arguments()), aCB.Shape(), aCB.History()) + "\n";
  }
  if (c == "bcadd" || c == "bcremove")
  {
    NCollection_List<TopoDS_Shape> aLSToTake, aLSToAvoid;
    int                            iMaterial = 0, n1 = n;
    bool                           bUpdate   = false;
    if (c == "bcadd")
    {
      if (n >= 3 && a[n - 3] == "-m")
      {
        iMaterial = Atoi(a[n - 2]);
        bUpdate   = a[n - 1] == "-u";
        n1        = n - 3;
      }
      else if (n >= 2 && a[n - 2] == "-m")
      {
        iMaterial = Atoi(a[n - 1]);
        n1        = n - 2;
      }
    }
    for (int i = 2; i < n1; i += 2)
    {
      TopoDS_Shape aS = Shape(i);
      if (aS.IsNull())
        continue;
      int iTake = i + 1 < n ? Atoi(a[i + 1]) : 0;
      (iTake ? aLSToTake : aLSToAvoid).Append(aS);
    }
    if (aLSToTake.IsEmpty())
      return "nothing to take\n";
    aCB.ClearWarnings();
    if (c == "bcadd")
      aCB.AddToResult(aLSToTake, aLSToAvoid, iMaterial, bUpdate);
    else
      aCB.RemoveFromResult(aLSToTake, aLSToAvoid);
    return CellsOut(aCB.Shape());
  }
  if (c == "bcremoveint")
  {
    aCB.ClearWarnings();
    aCB.RemoveInternalBoundaries();
    return CellsOut(aCB.Shape());
  }
  if (c == "bcmakecontainers")
  {
    aCB.MakeContainers();
    return ShapeOut(aCB.Shape());
  }
  if (c == "mkvolume")
  {
    bool                           bToIntersect = true, bCompounds = false, bAvoidInternal = false;
    NCollection_List<TopoDS_Shape> aLS;
    for (int i = 2; i < n; ++i)
    {
      TopoDS_Shape aS = Shape(i);
      if (!aS.IsNull())
        aLS.Append(aS);
      else if (a[i] == "-c")
        bCompounds = true;
      else if (a[i] == "-ni")
        bToIntersect = false;
      else if (a[i] == "-ai")
        bAvoidInternal = true;
    }
    if (aLS.IsEmpty())
      return "no shapes\n";
    if (bToIntersect && bCompounds)
    {
      NCollection_List<TopoDS_Shape> aLSx;
      for (NCollection_List<TopoDS_Shape>::Iterator it(aLS); it.More(); it.Next())
        for (TopoDS_Iterator itS(it.Value()); itS.More(); itS.Next())
          aLSx.Append(itS.Value());
      aLS = aLSx;
    }
    BOPAlgo_MakerVolume aMV;
    aMV.SetArguments(aLS);
    aMV.SetIntersect(bToIntersect);
    aMV.SetRunParallel(theS.RunParallel);
    aMV.SetFuzzyValue(theS.Fuzzy);
    aMV.SetNonDestructive(true); // the port's arguments are frozen, so always non-destructive (decision 577)
    aMV.SetAvoidInternalShapes(bAvoidInternal);
    aMV.SetGlue(theS.Glue);
    aMV.Perform();
    if (aMV.HasErrors())
      return Outcome(aMV.GetReport());
    return Outcome(aMV.GetReport()) + ShapeOut(aMV.Shape()) + HistoryText(ToVector(aLS), aMV.Shape(), aMV.History())
           + "\n";
  }
  return "unknown\n";
}

void WriteCall(std::ostream& theOut, const CapturedCall& theCall, const std::string& theOutput)
{
  theOut << "call " << theCall.Words.size() << "\n";
  for (const std::string& aW : theCall.Words)
    theOut << "w " << aW << "\n";
  for (const auto& aP : theCall.ShapeTexts)
    WriteBlock(theOut, "shape " + aP.first, aP.second);
  WriteBlock(theOut, "out", theOutput);
}

void RecordCase(std::ostream& theOut, const CapturedCase& theCase)
{
  Session aS;
  theOut << "case " << theCase.Name << "\n";
  for (const CapturedCall& aCall : theCase.Calls)
  {
    std::string anOut;
    try
    {
      OCC_CATCH_SIGNALS
      anOut = Run(aS, aCall);
    }
    catch (Standard_Failure& anE)
    {
      std::string aName = typeid(anE).name();
      if (aName.rfind("class ", 0) == 0)
        aName = aName.substr(6);
      anOut = "throws " + aName + "\n";
    }
    WriteCall(theOut, aCall, anOut);
  }
}

// ------------------------------------------------------------------------------------------------ random cases

struct CaseMaker
{
  CapturedCase Case;
  int          NbNames = 0;

  explicit CaseMaker(const std::string& theName) { Case.Name = theName; }

  //! A new shape name, its text recorded as read back.
  std::string Name(const TopoDS_Shape& theS, CapturedCall& theCall)
  {
    std::string aName = "s" + std::to_string(++NbNames);
    theCall.ShapeTexts[aName] = BRepText(theS);
    return aName;
  }

  CapturedCall& Call(std::initializer_list<std::string> theWords)
  {
    Case.Calls.emplace_back();
    Case.Calls.back().Words = theWords;
    return Case.Calls.back();
  }
};

std::vector<TopoDS_Shape> FacesOf(const TopoDS_Shape& theS)
{
  std::vector<TopoDS_Shape> aV;
  for (TopExp_Explorer anExp(theS, TopAbs_FACE); anExp.More(); anExp.Next())
    aV.push_back(anExp.Current());
  return aV;
}

void RandomCases(std::ostream& theOut)
{
  Random aR(0x6a09e667f3bcc909ull);
  // the volume maker on faces of one to three overlapping primitives
  for (int k = 0; k < 40; ++k)
  {
    CaseMaker     aM("random/mkvolume-" + std::to_string(k));
    int           aNb = 1 + k % 3;
    CapturedCall& aCall = aM.Call({"mkvolume", "r"});
    for (int i = 0; i < aNb; ++i)
    {
      TopoDS_Shape aP = RoundTrip(aR.Placed(aR.Primitive((int)(aR.Next() * 5) % 5), i == 0 ? 0.0 : 0.8));
      if (k % 5 == 4)
        aCall.Words.push_back(aM.Name(aP, aCall)); // whole solids
      else
        for (const TopoDS_Shape& aF : FacesOf(aP))
          aCall.Words.push_back(aM.Name(aF, aCall));
    }
    if (k % 7 == 3)
    {
      // an edge through the middle: an internal part
      TopoDS_Shape anE = RoundTrip(BRepBuilderAPI_MakeEdge(gp_Pnt(-0.3, -0.2, -0.1), gp_Pnt(0.3, 0.25, 0.2)).Shape());
      aCall.Words.push_back(aM.Name(anE, aCall));
    }
    if (aNb == 1 && k % 2 == 0)
      aCall.Words.push_back("-ni");
    if (k % 7 == 3 && k % 2 == 1)
      aCall.Words.push_back("-ai");
    RecordCase(theOut, aM.Case);
  }
  // the cells builder on two or three overlapping primitives, with random sequences
  for (int k = 0; k < 40; ++k)
  {
    CaseMaker                aM("random/cells-" + std::to_string(k));
    int                      aNb = 2 + k % 2;
    std::vector<std::string> aNames;
    aM.Call({"bclearobjects"});
    aM.Call({"bcleartools"});
    CapturedCall& anAdd = aM.Call({"baddobjects"});
    for (int i = 0; i < aNb; ++i)
    {
      TopoDS_Shape aP = RoundTrip(aR.Placed(aR.Primitive((int)(aR.Next() * 5) % 5), i == 0 ? 0.0 : 0.8));
      aNames.push_back(aM.Name(aP, anAdd));
      anAdd.Words.push_back(aNames.back());
    }
    // the later calls refer to the same shapes: their texts again
    std::map<std::string, std::string> aTexts = anAdd.ShapeTexts;
    aM.Call({"bfillds"});
    aM.Call({"bcbuild", "r"});
    int aNbOps = 3 + (int)(aR.Next() * 4);
    for (int j = 0; j < aNbOps; ++j)
    {
      int anOp = (int)(aR.Next() * 7);
      if (anOp <= 2)
      {
        CapturedCall& aCall = aM.Call({"bcadd", "res"});
        for (size_t i = 0; i < aNames.size(); ++i)
        {
          int aTake = (int)(aR.Next() * 3); // 0 avoid, 1 take, 2 leave out
          if (aTake == 2)
            continue;
          aCall.Words.push_back(aNames[i]);
          aCall.Words.push_back(aTake ? "1" : "0");
          aCall.ShapeTexts[aNames[i]] = aTexts[aNames[i]];
        }
        if (anOp >= 1)
        {
          aCall.Words.push_back("-m");
          aCall.Words.push_back(std::to_string(1 + (int)(aR.Next() * 2)));
          if (anOp == 2)
            aCall.Words.push_back("-u");
        }
      }
      else if (anOp == 3)
      {
        CapturedCall& aCall = aM.Call({"bcremove", "res"});
        size_t        i     = (size_t)(aR.Next() * aNames.size()) % aNames.size();
        aCall.Words.push_back(aNames[i]);
        aCall.Words.push_back("1");
        aCall.ShapeTexts[aNames[i]] = aTexts[aNames[i]];
      }
      else if (anOp == 4)
        aM.Call({"bcremoveint", "res"});
      else if (anOp == 5)
      {
        if (aR.Next() < 0.5)
          aM.Call({"bcaddall", "res", "-m", "1", "-u"});
        else
          aM.Call({"bcaddall", "res"});
      }
      else
        aM.Call({"bcremoveall"});
    }
    aM.Call({"bcmakecontainers", "res"});
    RecordCase(theOut, aM.Case);
  }
}
} // namespace

int RecordVolume(const std::vector<CapturedCase>& theCases, std::ostream& theOut)
{
  for (const CapturedCase& aCase : theCases)
    RecordCase(theOut, aCase);
  RandomCases(theOut);
  return 0;
}
