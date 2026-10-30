// borec's defeature area (unit 6G): native BRepAlgoAPI_Defeaturing (BOPAlgo_RemoveFeatures) on
// - the corpus's removefeatures calls, captured by Draw-lite with the indices of the faces to remove in the input's
//   TopExp::MapShapes(FACE) numbering (the word "#faces i j ..." the capture adds after the command's words);
// - random solids with holes, bosses and pockets made by native Booleans, removing random subsets of the features'
//   faces (and now and then one random face);
// - single random faces of data/occ shapes (not Ball.brep: one of its faces ran over ten minutes natively).
// Each case: "case <id> <name>", the input as a "shape" block, "faces <n> i...", "parallel 0|1", then native's
// "errors ...", "warnings ...", the "result" block (BRep text, empty on errors), the "history" block (common.hxx's
// HistoryText), "parallelsame 0|1" (native's run in the other parallel mode gave the same text and history) and, where it
// didn't, that run's outcome as an "other" block; "end".
#include "common.hxx"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Message_Gravity.hxx>
#include <TopExp_Explorer.hxx>

using namespace borec;

namespace
{
int gCase = 0;

std::string RunOnce(const TopoDS_Shape& theInput, const std::vector<int>& theFaces, bool theParallel)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(theInput, TopAbs_FACE, aFaces);
  BRepAlgoAPI_Defeaturing aDF;
  aDF.SetShape(theInput);
  for (int anIndex : theFaces)
    if (anIndex >= 1 && anIndex <= aFaces.Extent())
      aDF.AddFaceToRemove(aFaces(anIndex));
  aDF.SetToFillHistory(true);
  aDF.SetRunParallel(theParallel);
  std::ostringstream anOut;
  try
  {
    aDF.Build();
  }
  catch (Standard_Failure const&)
  {
    anOut << "throws\n";
    return anOut.str();
  }
  anOut << "errors" << Alerts(aDF.GetReport(), Message_Fail) << "\n";
  anOut << "warnings" << Alerts(aDF.GetReport(), Message_Warning) << "\n";
  TopoDS_Shape aResult;
  if (!aDF.HasErrors())
    aResult = aDF.Shape();
  WriteShape(anOut, "result", aResult);
  WriteBlock(anOut, "history", aDF.HasErrors() ? std::string("no history") : HistoryText({theInput}, aResult, aDF.History()));
  // where each input sub-shape of a supported type is in the result itself (its index in the result's MapShapes, or
  // 0): the port's history lists such a shape as modified where its copy at the entry changed (decision 643)
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anIn, aRes;
  TopExp::MapShapes(theInput, anIn);
  if (!aResult.IsNull())
    TopExp::MapShapes(aResult, aRes);
  anOut << "kept";
  for (int i = 1; i <= anIn.Extent(); ++i)
    if (BRepTools_History::IsSupportedType(anIn(i)))
      anOut << " " << aRes.FindIndex(anIn(i));
  anOut << "\n";
  return anOut.str();
}

void Record(std::ostream& theOut, const std::string& theName, const TopoDS_Shape& theShape, const std::vector<int>& theFaces,
            bool theParallel)
{
  // write and read back, so that native and the port start from the same bits
  std::string  aText  = BRepText(theShape);
  TopoDS_Shape anInput = ReadBRep(aText);
  theOut << "case " << ++gCase << " " << theName << "\n";
  WriteBlock(theOut, "shape", aText);
  theOut << "faces " << theFaces.size();
  for (int i : theFaces)
    theOut << " " << i;
  theOut << "\nparallel " << (theParallel ? 1 : 0) << "\n";
  std::string aRun = RunOnce(anInput, theFaces, theParallel);
  theOut << aRun;
  std::string aOther = RunOnce(ReadBRep(aText), theFaces, !theParallel);
  theOut << "parallelsame " << (aOther == aRun ? 1 : 0) << "\n";
  if (aOther != aRun)
    WriteBlock(theOut, "other", aOther);
  theOut << "end\n";
  theOut.flush();
}

int FaceIndex(const NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>& theFaces, const TopoDS_Shape& theF)
{
  return theFaces.FindIndex(theF);
}

// The faces of the Boolean result coming from the tool: the tool faces' images, or the tool faces kept as they are.
template <class TheBop>
std::vector<int> ToolFaces(TheBop& theBop, const TopoDS_Shape& theTool, const TopoDS_Shape& theResult)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(theResult, TopAbs_FACE, aFaces);
  std::vector<int> anIdx;
  for (TopExp_Explorer anExp(theTool, TopAbs_FACE); anExp.More(); anExp.Next())
  {
    const TopoDS_Shape& aF = anExp.Current();
    const NCollection_List<TopoDS_Shape>& aM = theBop.Modified(aF);
    if (aM.IsEmpty())
    {
      int i = FaceIndex(aFaces, aF);
      if (i > 0)
        anIdx.push_back(i);
    }
    for (NCollection_List<TopoDS_Shape>::Iterator it(aM); it.More(); it.Next())
    {
      int i = FaceIndex(aFaces, it.Value());
      if (i > 0)
        anIdx.push_back(i);
    }
  }
  return anIdx;
}

// The faces of theShape (numbering of theShape) that are images of faces numbered in theOld of theOldShape.
std::vector<int> Carry(const std::vector<int>& theOld, const TopoDS_Shape& theOldShape, BRepAlgoAPI_BooleanOperation& theBop,
                       const TopoDS_Shape& theShape)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anOld, aNew;
  TopExp::MapShapes(theOldShape, TopAbs_FACE, anOld);
  TopExp::MapShapes(theShape, TopAbs_FACE, aNew);
  std::vector<int> anIdx;
  for (int i : theOld)
  {
    const TopoDS_Shape&                   aF = anOld(i);
    const NCollection_List<TopoDS_Shape>& aM = theBop.Modified(aF);
    if (aM.IsEmpty() && !theBop.IsDeleted(aF))
    {
      int j = aNew.FindIndex(aF);
      if (j > 0)
        anIdx.push_back(j);
    }
    for (NCollection_List<TopoDS_Shape>::Iterator it(aM); it.More(); it.Next())
    {
      int j = aNew.FindIndex(it.Value());
      if (j > 0)
        anIdx.push_back(j);
    }
  }
  return anIdx;
}

void RandomCases(std::ostream& theOut)
{
  Random  aR(0x6a09e667f3bcc909ull);
  const char* aKinds[] = {"hole", "boss", "pocket", "slot"};
  for (int aCase = 0; aCase < 160; ++aCase)
  {
    // the base: a box [0, a] x [0, b] x [0, c], or a cylinder on z
    bool         aCyl = aR.Next() < 0.3;
    double       a = aR.Range(8, 14), b = aR.Range(8, 14), c = aR.Range(6, 12);
    TopoDS_Shape aShape =
      aCyl ? BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(a / 2, b / 2, 0), gp_Dir(0, 0, 1)), std::min(a, b) / 2, c).Shape()
           : BRepPrimAPI_MakeBox(a, b, c).Shape();
    std::string                    aName = std::string("random/") + (aCyl ? "cyl" : "box");
    std::vector<std::vector<int>> aFeatures;
    int                            aNbF = 1 + (int)(aR.Next() * 3);
    bool                           aFailed = false;
    for (int f = 0; f < aNbF && !aFailed; ++f)
    {
      int    aKind = (int)(aR.Next() * 4) % 4;
      double x = aR.Range(0.3 * a, 0.7 * a), y = aR.Range(0.3 * b, 0.7 * b), r = aR.Range(0.4, 1.4);
      bool   aFuse = false;
      TopoDS_Shape aTool;
      switch (aKind)
      {
        case 0: // a hole through, along z (or partly, a blind hole)
        {
          double aDepth = aR.Next() < 0.5 ? c + 2 : aR.Range(1.0, c - 1.0);
          aTool = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, c - aDepth + (aDepth > c ? 1 : 0)), gp_Dir(0, 0, 1)), r, aDepth + 1).Shape();
          break;
        }
        case 1: // a boss on top
          aTool = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, c), gp_Dir(0, 0, 1)), r, aR.Range(0.5, 3.0)).Shape();
          aFuse = true;
          break;
        case 2: // a pocket from the top
        {
          double w = aR.Range(0.8, 2.5), h = aR.Range(0.8, 2.5);
          aTool = BRepPrimAPI_MakeBox(gp_Pnt(x - w / 2, y - h / 2, c - aR.Range(0.5, c / 2)), 2 * w / 2, 2 * h / 2, c).Shape();
          break;
        }
        default: // a hole across, along x
          aTool = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-1, y, aR.Range(0.3 * c, 0.7 * c)), gp_Dir(1, 0, 0)), std::min(r, 0.2 * c), a + 2).Shape();
          break;
      }
      aName += std::string("-") + aKinds[aKind];
      TopoDS_Shape aNew;
      std::vector<int> aToolFaces;
      if (aFuse)
      {
        BRepAlgoAPI_Fuse aBop(aShape, aTool);
        if (!aBop.IsDone() || aBop.HasErrors())
        {
          aFailed = true;
          break;
        }
        aNew = aBop.Shape();
        for (auto& aFeat : aFeatures)
          aFeat = Carry(aFeat, aShape, aBop, aNew);
        aToolFaces = ToolFaces(aBop, aTool, aNew);
      }
      else
      {
        BRepAlgoAPI_Cut aBop(aShape, aTool);
        if (!aBop.IsDone() || aBop.HasErrors())
        {
          aFailed = true;
          break;
        }
        aNew = aBop.Shape();
        for (auto& aFeat : aFeatures)
          aFeat = Carry(aFeat, aShape, aBop, aNew);
        aToolFaces = ToolFaces(aBop, aTool, aNew);
      }
      aShape = aNew;
      aFeatures.push_back(aToolFaces);
    }
    if (aFailed)
      continue;
    // the faces to remove: a random non-empty subset of the features; now and then one random face of the shape
    std::vector<int> aFaces;
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anAll;
    TopExp::MapShapes(aShape, TopAbs_FACE, anAll);
    if (aCase % 9 == 8)
    {
      aFaces.push_back(1 + (int)(aR.Next() * anAll.Extent()) % anAll.Extent());
      aName += "-oneface";
    }
    else
    {
      for (size_t f = 0; f < aFeatures.size(); ++f)
        if (f == 0 || aR.Next() < 0.6)
          aFaces.insert(aFaces.end(), aFeatures[f].begin(), aFeatures[f].end());
    }
    Record(theOut, aName, aShape, aFaces, aCase % 4 == 3);
  }
}

void OccCases(std::ostream& theOut, const std::string& theDir)
{
  Random      aR(0xbb67ae8584caa73bull);
  const char* aFiles[] = {"bottle.brep", "CrankArm.brep", "hammer.brep", "wedge_ok.brep", "solid.brep",
                          "Pump_Nut.brep", "Pump_TopCover.brep", "mal_ecrou.brep", "mal_vis.brep"};
  for (const char* aFile : aFiles)
  {
    TopoDS_Shape aS;
    BRep_Builder aB;
    if (!BRepTools::Read(aS, (theDir + "/" + aFile).c_str(), aB))
    {
      std::fprintf(stderr, "cannot read %s\n", aFile);
      continue;
    }
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
    TopExp::MapShapes(aS, TopAbs_FACE, aFaces);
    if (aFaces.IsEmpty())
      continue;
    for (int k = 0; k < 4; ++k)
    {
      std::vector<int> anIdx = {1 + (int)(aR.Next() * aFaces.Extent()) % aFaces.Extent()};
      Record(theOut, std::string("occ/") + aFile + "-" + std::to_string(anIdx[0]), aS, anIdx, false);
    }
  }
}
} // namespace

int RecordDefeature(const std::vector<CapturedCase>& theCases, std::ostream& theOut)
{
  // corpus: removefeatures result shape f1 ... [-parallel] #faces i ...
  for (const CapturedCase& aCase : theCases)
  {
    int aCall = 0;
    for (const CapturedCall& aC : aCase.Calls)
    {
      ++aCall;
      if (aC.Words.size() < 4 || aC.Words[0] != "removefeatures")
        continue;
      auto anIt = aC.Shapes.find(aC.Words[2]);
      if (anIt == aC.Shapes.end())
        continue;
      bool             aParallel = false;
      std::vector<int> aFaces;
      for (const std::string& aW : aC.Words)
      {
        if (aW == "-parallel")
          aParallel = true;
        if (aW.rfind("#faces", 0) == 0)
        {
          std::istringstream aSS(aW.substr(6));
          int                i;
          while (aSS >> i)
            aFaces.push_back(i);
        }
      }
      Record(theOut, aCase.Name + "#" + std::to_string(aCall), anIt->second, aFaces, aParallel);
    }
  }
  RandomCases(theOut);
  const char* aData = std::getenv("BOREC_OCC_DATA");
  if (aData)
    OccCases(theOut, aData);
  return 0;
}
