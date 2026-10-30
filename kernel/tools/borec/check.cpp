// borec's check area (unit 6G): BOPAlgo_CheckerSI, BOPAlgo_ArgumentAnalyzer and BRepAlgoAPI_Check on the corpus's
// captured bopcheck/bopargcheck/bopapicheck calls and on random primitive configurations and data/occ shapes. Written
// as the port's tests/OcctNet.ModelingAlgorithms.Tests/BOPAlgo/BOPAlgo_CheckNativeTests.cs reads and writes it:
//   case <id> <name>
//   shapes <length>         the arguments' compound as BRep text (both sides read it back)
//   run si <level> <fuzzy>  | run aa <modes> <op> <fuzzy> <copy2> | run api <op> <se> <si> <fuzzy>
//   ... the outcome ...
//   end
// CheckerSI's outcome is its errors and warnings, its table of interferences in iteration order, the numbers of its
// solid interferences, and the pave filler's dump of tools/pfrec (written as PaveFillerDump.cs writes it). The
// analyzers' outcome is their list of results, each shape as its index in TopExp::MapShapes of the first shape then
// the second (with its orientation), every double as its 64 bits in hex.
#include "common.hxx"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <set>

#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BOPAlgo_CheckerSI.hxx>
#include <BOPDS_CommonBlock.hxx>
#include <BOPDS_Curve.hxx>
#include <BOPDS_DS.hxx>
#include <BOPDS_FaceInfo.hxx>
#include <BOPDS_Interf.hxx>
#include <BOPDS_PaveBlock.hxx>
#include <BOPDS_Point.hxx>
#include <BOPDS_ShapeInfo.hxx>
#include <BRepAlgoAPI_Check.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <IntTools_CommonPrt.hxx>
#include <IntTools_Curve.hxx>
#include <IntTools_Range.hxx>

using borec::H;

namespace
{
// ------------------------------------------------------------------------------------------------ pfrec's dump
std::string P(const gp_Pnt& theP)
{
  return H(theP.X()) + " " + H(theP.Y()) + " " + H(theP.Z());
}

std::string P2(const gp_Pnt2d& theP)
{
  return H(theP.X()) + " " + H(theP.Y());
}

std::string Join(const std::vector<int>& theValues, bool theSort)
{
  std::vector<int> aV = theValues;
  if (theSort)
    std::sort(aV.begin(), aV.end());
  std::string aS;
  for (size_t i = 0; i < aV.size(); i++)
    aS += (i ? "," : "") + std::to_string(aV[i]);
  return aS;
}

std::vector<int> ToVec(const NCollection_List<int>& theL)
{
  std::vector<int> aV;
  for (NCollection_List<int>::Iterator it(theL); it.More(); it.Next())
    aV.push_back(it.Value());
  return aV;
}

std::vector<int> ToVec(const NCollection_Map<int>& theM)
{
  std::vector<int> aV;
  for (NCollection_Map<int>::Iterator it(theM); it.More(); it.Next())
    aV.push_back(it.Value());
  return aV;
}

std::string PB(const occ::handle<BOPDS_PaveBlock>& thePB)
{
  int nV1, nV2;
  thePB->Indices(nV1, nV2);
  return "[" + std::to_string(thePB->OriginalEdge()) + " " + std::to_string(thePB->Edge()) + " " + std::to_string(nV1)
         + " " + H(thePB->Pave1().Parameter()) + " " + std::to_string(nV2) + " " + H(thePB->Pave2().Parameter()) + "]";
}

std::string CP(const IntTools_CommonPrt& theCP)
{
  std::string aS = std::to_string((int)theCP.Type());
  aS += " " + H(theCP.Range1().First()) + " " + H(theCP.Range1().Last());
  aS += " r2:";
  const NCollection_Sequence<IntTools_Range>& aR2 = theCP.Ranges2();
  for (int i = 1; i <= aR2.Length(); i++)
    aS += " " + H(aR2(i).First()) + " " + H(aR2(i).Last());
  aS += " vp " + H(theCP.VertexParameter1()) + " " + H(theCP.VertexParameter2());
  return aS;
}

// The dump, as PaveFillerDump.cs writes it (tools/pfrec's).
std::string Dump(const BOPDS_DS& theDS)
{
  std::string aB;
  const int   aNb = theDS.NbShapes(), aNbSrc = theDS.NbSourceShapes();
  aB += "ds " + std::to_string(aNb) + " " + std::to_string(aNbSrc) + "\n";
  std::vector<int> aFaces;
  for (int i = 0; i < aNb; i++)
  {
    const BOPDS_ShapeInfo& aSI   = theDS.ShapeInfo(i);
    const TopoDS_Shape&    aS    = aSI.Shape();
    TopAbs_ShapeEnum       aType = aSI.ShapeType();
    aB += "s " + std::to_string(i) + " " + std::to_string((int)aType) + " " + std::to_string((int)aS.Orientation());
    aB += " sub " + Join(ToVec(aSI.SubShapes()), i < aNbSrc);
    if (aType == TopAbs_VERTEX)
    {
      const TopoDS_Vertex& aV = TopoDS::Vertex(aS);
      aB += " tol " + H(BRep_Tool::Tolerance(aV)) + " p " + P(BRep_Tool::Pnt(aV));
    }
    else if (aType == TopAbs_EDGE)
    {
      const TopoDS_Edge& aE = TopoDS::Edge(aS);
      aB += " tol " + H(BRep_Tool::Tolerance(aE)) + " deg " + (BRep_Tool::Degenerated(aE) ? "1" : "0");
      if (i >= aNbSrc)
      {
        double                  aT1, aT2;
        occ::handle<Geom_Curve> aC = BRep_Tool::Curve(aE, aT1, aT2);
        aB += " r " + H(aT1) + " " + H(aT2);
        if (!aC.IsNull())
          aB += " c " + P(aC->Value(aT1)) + " " + P(aC->Value((aT1 + aT2) * 0.5)) + " " + P(aC->Value(aT2));
      }
    }
    else if (aType == TopAbs_FACE)
    {
      aB += " tol " + H(BRep_Tool::Tolerance(TopoDS::Face(aS)));
      aFaces.push_back(i);
    }
    aB += "\n";
  }
  for (int i = aNbSrc; i < aNb; i++)
  {
    if (theDS.ShapeInfo(i).ShapeType() != TopAbs_EDGE)
      continue;
    const TopoDS_Edge& aE = TopoDS::Edge(theDS.Shape(i));
    if (BRep_Tool::Degenerated(aE))
      continue;
    for (int nF : aFaces)
    {
      const TopoDS_Face&        aF = TopoDS::Face(theDS.Shape(nF));
      double                    aT1, aT2;
      bool                      anIsStored = false;
      occ::handle<Geom2d_Curve> aC2        = BRep_Tool::CurveOnSurface(aE, aF, aT1, aT2, &anIsStored);
      if (!aC2.IsNull() && anIsStored)
        aB += "pc " + std::to_string(i) + " " + std::to_string(nF) + " " + H(aT1) + " " + H(aT2) + " "
              + P2(aC2->Value((aT1 + aT2) * 0.5)) + "\n";
    }
  }
  for (int i = 0; i < aNb; i++)
  {
    int nSD = -1;
    if (theDS.HasShapeSD(i, nSD))
      aB += "sd " + std::to_string(i) + " " + std::to_string(nSD) + "\n";
  }
  std::map<const BOPDS_CommonBlock*, int>     aCBs;
  std::vector<occ::handle<BOPDS_CommonBlock>> aCBList;
  for (int i = 0; i < aNb; i++)
  {
    const BOPDS_ShapeInfo& aSI = theDS.ShapeInfo(i);
    if (aSI.ShapeType() != TopAbs_EDGE || !theDS.HasPaveBlocks(i))
      continue;
    const NCollection_List<occ::handle<BOPDS_PaveBlock>>& aLPB = theDS.PaveBlocks(i);
    for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aLPB); it.More(); it.Next())
    {
      const occ::handle<BOPDS_PaveBlock>& aPB = it.Value();
      aB += "pb " + std::to_string(i) + " " + PB(aPB);
      if (aPB->HasShrunkData())
      {
        double  aTS1, aTS2;
        Bnd_Box aBox;
        bool    aSplittable;
        aPB->ShrunkData(aTS1, aTS2, aBox, aSplittable);
        aB += " sh " + H(aTS1) + " " + H(aTS2) + " " + (aSplittable ? "1" : "0");
      }
      const occ::handle<BOPDS_CommonBlock>& aCB = theDS.CommonBlock(aPB);
      if (!aCB.IsNull())
      {
        auto anIt = aCBs.find(aCB.get());
        int  anId;
        if (anIt == aCBs.end())
        {
          anId = (int)aCBs.size();
          aCBs[aCB.get()] = anId;
          aCBList.push_back(aCB);
        }
        else
          anId = anIt->second;
        aB += " cb " + std::to_string(anId);
      }
      aB += "\n";
    }
  }
  for (size_t k = 0; k < aCBList.size(); k++)
  {
    const occ::handle<BOPDS_CommonBlock>& aCB = aCBList[k];
    aB += "cb " + std::to_string(k) + " " + H(aCB->Tolerance()) + " pbs";
    for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aCB->PaveBlocks()); it.More(); it.Next())
      aB += " " + PB(it.Value());
    aB += " faces " + Join(ToVec(aCB->Faces()), true) + "\n";
  }
  for (int nF : aFaces)
  {
    if (!theDS.HasFaceInfo(nF))
      continue;
    const BOPDS_FaceInfo& aFI = theDS.FaceInfo(nF);
    aB += "fi " + std::to_string(nF);
    const char* aTags[3] = {"on", "in", "sc"};
    const NCollection_IndexedMap<occ::handle<BOPDS_PaveBlock>>* aMaps[3] = {&aFI.PaveBlocksOn(),
                                                                             &aFI.PaveBlocksIn(),
                                                                             &aFI.PaveBlocksSc()};
    for (int t = 0; t < 3; t++)
    {
      aB += std::string(" ") + aTags[t];
      for (int j = 1; j <= aMaps[t]->Extent(); j++)
        aB += " " + PB(aMaps[t]->FindKey(j));
    }
    aB += " von " + Join(ToVec(aFI.VerticesOn()), true) + " vin " + Join(ToVec(aFI.VerticesIn()), true) + " vsc "
          + Join(ToVec(aFI.VerticesSc()), true) + "\n";
  }
  BOPDS_DS& aDS = const_cast<BOPDS_DS&>(theDS);
  for (int i = 0; i < aDS.InterfVV().Length(); i++)
  {
    const BOPDS_InterfVV& aI = aDS.InterfVV()(i);
    aB += "vv " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + "\n";
  }
  for (int i = 0; i < aDS.InterfVE().Length(); i++)
  {
    const BOPDS_InterfVE& aI = aDS.InterfVE()(i);
    aB += "ve " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + H(aI.Parameter()) + "\n";
  }
  for (int i = 0; i < aDS.InterfVF().Length(); i++)
  {
    const BOPDS_InterfVF& aI = aDS.InterfVF()(i);
    double                aU, aV;
    aI.UV(aU, aV);
    aB += "vf " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + H(aU) + " " + H(aV) + "\n";
  }
  for (int i = 0; i < aDS.InterfEE().Length(); i++)
  {
    const BOPDS_InterfEE& aI = aDS.InterfEE()(i);
    aB += "ee " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + CP(aI.CommonPart()) + "\n";
  }
  for (int i = 0; i < aDS.InterfEF().Length(); i++)
  {
    const BOPDS_InterfEF& aI = aDS.InterfEF()(i);
    aB += "ef " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + CP(aI.CommonPart()) + "\n";
  }
  for (int i = 0; i < aDS.InterfFF().Length(); i++)
  {
    const BOPDS_InterfFF&                        aI  = aDS.InterfFF()(i);
    const NCollection_DynamicArray<BOPDS_Curve>& aVC = aI.Curves();
    const NCollection_DynamicArray<BOPDS_Point>& aVP = aI.Points();
    aB += "ff " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " "
          + (aI.TangentFaces() ? "1" : "0") + " " + std::to_string(aVC.Length()) + " " + std::to_string(aVP.Length())
          + "\n";
    for (int j = 0; j < aVC.Length(); j++)
    {
      const BOPDS_Curve& aNC = aVC(j);
      aB += " c " + H(aNC.Tolerance()) + " " + H(aNC.TangentialTolerance()) + " tv "
            + Join(ToVec(aNC.TechnoVertices()), false);
      double aT1 = 0, aT2 = 0;
      gp_Pnt aP1, aP2;
      if (aNC.Curve().Bounds(aT1, aT2, aP1, aP2))
        aB += " b " + H(aT1) + " " + H(aT2) + " " + P(aP1) + " " + P(aP2);
      aB += " pbs";
      for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aNC.PaveBlocks()); it.More(); it.Next())
        aB += " " + PB(it.Value());
      aB += "\n";
    }
    for (int j = 0; j < aVP.Length(); j++)
      aB += " p " + std::to_string(aVP(j).Index()) + " " + P(aVP(j).Pnt()) + "\n";
  }
  return aB;
}

// ------------------------------------------------------------------------------------------------ outcomes

// The map of the shapes the results refer to: the first shape's sub-shapes, then the second's.
struct ShapeIndex
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> Map;

  ShapeIndex(const TopoDS_Shape& theS1, const TopoDS_Shape& theS2)
  {
    if (!theS1.IsNull())
      TopExp::MapShapes(theS1, Map);
    if (!theS2.IsNull())
      TopExp::MapShapes(theS2, Map);
  }

  std::string Of(const TopoDS_Shape& theS) const
  {
    if (theS.IsNull())
      return "-";
    return std::to_string(Map.FindIndex(theS)) + "o" + std::to_string((int)theS.Orientation());
  }
};

std::string Results(const NCollection_List<BOPAlgo_CheckResult>& theResults, const ShapeIndex& theIndex)
{
  std::string aB = "results " + std::to_string(theResults.Extent()) + "\n";
  for (NCollection_List<BOPAlgo_CheckResult>::Iterator it(theResults); it.More(); it.Next())
  {
    const BOPAlgo_CheckResult& aR = it.Value();
    aB += "r " + std::to_string((int)aR.GetCheckStatus()) + " " + theIndex.Of(aR.GetShape1()) + " "
          + theIndex.Of(aR.GetShape2()) + " f1";
    for (NCollection_List<TopoDS_Shape>::Iterator itS(aR.GetFaultyShapes1()); itS.More(); itS.Next())
      aB += " " + theIndex.Of(itS.Value());
    aB += " f2";
    for (NCollection_List<TopoDS_Shape>::Iterator itS(aR.GetFaultyShapes2()); itS.More(); itS.Next())
      aB += " " + theIndex.Of(itS.Value());
    aB += " " + H(aR.GetMaxDistance1()) + " " + H(aR.GetMaxParameter1()) + " " + H(aR.GetMaxDistance2()) + " "
          + H(aR.GetMaxParameter2()) + "\n";
  }
  return aB;
}

std::string RunSI(const TopoDS_Shape& theS, int theLevel, double theFuzzy)
{
  BOPAlgo_CheckerSI              aChecker;
  NCollection_List<TopoDS_Shape> aLS;
  aLS.Append(theS);
  aChecker.SetArguments(aLS);
  aChecker.SetLevelOfCheck(theLevel);
  aChecker.SetFuzzyValue(theFuzzy);
  try
  {
    aChecker.Perform();
  }
  catch (Standard_Failure const&)
  {
    return "throws\n";
  }
  std::string aB = "errors" + borec::Alerts(aChecker.GetReport(), Message_Fail) + "\nwarnings"
                   + borec::Alerts(aChecker.GetReport(), Message_Warning) + "\n";
  if (aChecker.PDS() == nullptr)
    return aB;
  BOPDS_DS& aDS = *aChecker.PDS();
  aB += "pairs";
  for (NCollection_Map<BOPDS_Pair>::Iterator it(aDS.Interferences()); it.More(); it.Next())
  {
    int n1, n2;
    it.Value().Indices(n1, n2);
    aB += " " + std::to_string(n1) + "," + std::to_string(n2);
  }
  aB += "\nz " + std::to_string(aDS.InterfVZ().Length()) + " " + std::to_string(aDS.InterfEZ().Length()) + " "
        + std::to_string(aDS.InterfFZ().Length()) + " " + std::to_string(aDS.InterfZZ().Length()) + "\n";
  return aB + Dump(aDS);
}

// The analyzer's modes, in this order: type, self-interference, small edges, rebuilt faces, tangency, merged
// vertices, merged edges, continuity, curves on surfaces, stop on the first fault.
std::string RunAA(const TopoDS_Shape& theS1,
                  const TopoDS_Shape& theS2,
                  const std::string&  theModes,
                  int                 theOp,
                  double              theFuzzy)
{
  BOPAlgo_ArgumentAnalyzer aChecker;
  aChecker.SetFuzzyValue(theFuzzy);
  aChecker.SetShape1(theS1);
  if (!theS2.IsNull())
    aChecker.SetShape2(theS2);
  aChecker.OperationType()      = (BOPAlgo_Operation)theOp;
  aChecker.ArgumentTypeMode()   = theModes[0] == '1';
  aChecker.SelfInterMode()      = theModes[1] == '1';
  aChecker.SmallEdgeMode()      = theModes[2] == '1';
  aChecker.RebuildFaceMode()    = theModes[3] == '1';
  aChecker.TangentMode()        = theModes[4] == '1';
  aChecker.MergeVertexMode()    = theModes[5] == '1';
  aChecker.MergeEdgeMode()      = theModes[6] == '1';
  aChecker.ContinuityMode()     = theModes[7] == '1';
  aChecker.CurveOnSurfaceMode() = theModes[8] == '1';
  aChecker.StopOnFirstFaulty()  = theModes[9] == '1';
  try
  {
    aChecker.Perform();
  }
  catch (Standard_Failure const&)
  {
    return "throws\n";
  }
  return std::string(aChecker.HasFaulty() ? "faulty 1\n" : "faulty 0\n")
         + Results(aChecker.GetCheckResult(), ShapeIndex(theS1, theS2));
}

std::string RunAPI(const TopoDS_Shape& theS1,
                   const TopoDS_Shape& theS2,
                   int                 theOp,
                   bool                theSE,
                   bool                theSI,
                   double              theFuzzy)
{
  BRepAlgoAPI_Check aChecker;
  aChecker.SetFuzzyValue(theFuzzy);
  if (theS2.IsNull())
    aChecker.SetData(theS1, theSE, theSI);
  else
    aChecker.SetData(theS1, theS2, (BOPAlgo_Operation)theOp, theSE, theSI);
  try
  {
    aChecker.Perform();
  }
  catch (Standard_Failure const&)
  {
    return "throws\n";
  }
  return std::string(aChecker.IsValid() ? "valid 1\n" : "valid 0\n")
         + Results(aChecker.Result(), ShapeIndex(theS1, theS2));
}

// ------------------------------------------------------------------------------------------------ cases

int gCase = 0;

// The arguments as a compound written and read back; the first and second arguments of the run are its children.
struct Args
{
  std::string  Text;
  TopoDS_Shape S1, S2;
};

Args MakeArgs(const TopoDS_Shape& theS1, const TopoDS_Shape& theS2)
{
  TopoDS_Compound aC;
  BRep_Builder    aB;
  aB.MakeCompound(aC);
  aB.Add(aC, theS1);
  if (!theS2.IsNull())
    aB.Add(aC, theS2);
  Args anArgs;
  anArgs.Text           = borec::BRepText(aC);
  TopoDS_Shape    aRead = borec::ReadBRep(anArgs.Text);
  TopoDS_Iterator anIt(aRead);
  anArgs.S1 = anIt.Value();
  anIt.Next();
  if (anIt.More())
    anArgs.S2 = anIt.Value();
  return anArgs;
}

void Begin(std::ostream& theOut, const std::string& theName, const Args& theArgs, const std::string& theRun)
{
  theOut << "case " << ++gCase << " " << theName << "\n";
  borec::WriteBlock(theOut, "shapes", theArgs.Text);
  theOut << "run " << theRun << "\n";
}

void SI(std::ostream& theOut, const std::string& theName, const TopoDS_Shape& theS, int theLevel, double theFuzzy)
{
  Args anArgs = MakeArgs(theS, TopoDS_Shape());
  Begin(theOut, theName, anArgs, "si " + std::to_string(theLevel) + " " + H(theFuzzy));
  theOut << RunSI(anArgs.S1, theLevel, theFuzzy) << "end\n";
}

// theCopy2: the second shape is checked as a BRepBuilderAPI_Copy of it, as bopargcheck does.
void AA(std::ostream&       theOut,
        const std::string&  theName,
        const TopoDS_Shape& theS1,
        const TopoDS_Shape& theS2,
        const std::string&  theModes,
        int                 theOp,
        double              theFuzzy,
        bool                theCopy2)
{
  Args anArgs = MakeArgs(theS1, theS2);
  Begin(theOut,
        theName,
        anArgs,
        "aa " + theModes + " " + std::to_string(theOp) + " " + H(theFuzzy) + " " + (theCopy2 ? "1" : "0"));
  TopoDS_Shape aS2 = anArgs.S2;
  if (theCopy2 && !aS2.IsNull())
    aS2 = BRepBuilderAPI_Copy(aS2).Shape();
  theOut << RunAA(anArgs.S1, aS2, theModes, theOp, theFuzzy) << "end\n";
}

void API(std::ostream&       theOut,
         const std::string&  theName,
         const TopoDS_Shape& theS1,
         const TopoDS_Shape& theS2,
         int                 theOp,
         bool                theSE,
         bool                theSI,
         double              theFuzzy)
{
  Args anArgs = MakeArgs(theS1, theS2);
  Begin(theOut,
        theName,
        anArgs,
        "api " + std::to_string(theOp) + " " + (theSE ? "1" : "0") + " " + (theSI ? "1" : "0") + " " + H(theFuzzy));
  theOut << RunAPI(anArgs.S1, anArgs.S2, theOp, theSE, theSI, theFuzzy) << "end\n";
}

int OpOfLetter(char theC)
{
  switch (std::toupper((unsigned char)theC))
  {
    case 'F': return BOPAlgo_FUSE;
    case 'O': return BOPAlgo_COMMON;
    case 'C': return BOPAlgo_CUT;
    case 'T': return BOPAlgo_CUT21;
    case 'S': return BOPAlgo_SECTION;
    case 'U': return BOPAlgo_UNKNOWN;
    default: return -1;
  }
}

int OpOfName(const std::string& theName, int theDefault)
{
  if (theName == "common")
    return BOPAlgo_COMMON;
  if (theName == "fuse")
    return BOPAlgo_FUSE;
  if (theName == "cut")
    return BOPAlgo_CUT;
  if (theName == "tuc")
    return BOPAlgo_CUT21;
  if (theName == "section")
    return BOPAlgo_SECTION;
  return theDefault;
}

// The corpus's calls, interpreted as Draw's commands do (BOPTest_CheckCommands.cxx); the fuzzy value is the case's
// last bfuzzyvalue (Precision::Confusion by default).
void Corpus(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut)
{
  for (const borec::CapturedCase& aCase : theCases)
  {
    double aFuzzy = Precision::Confusion();
    for (const borec::CapturedCall& aCall : aCase.Calls)
    {
      const std::vector<std::string>& a     = aCall.Words;
      const int                       n     = (int)a.size();
      auto                            Shape = [&](int i) {
        auto it = aCall.Shapes.find(a[i]);
        return it == aCall.Shapes.end() ? TopoDS_Shape() : it->second;
      };
      if (a[0] == "bfuzzyvalue" && n == 2)
      {
        aFuzzy = std::atof(a[1].c_str());
      }
      else if (a[0] == "bopcheck" && n >= 2 && !Shape(1).IsNull())
      {
        int aLevel = BOPDS_DS::NbInterfTypes() - 1;
        if (n > 2 && a[2][0] != '-')
          aLevel = std::atoi(a[2].c_str());
        if (aLevel < 0 || aLevel > BOPDS_DS::NbInterfTypes() - 1)
          continue;
        SI(theOut, aCase.Name, Shape(1), aLevel, aFuzzy);
      }
      else if (a[0] == "bopargcheck" && n >= 2 && !Shape(1).IsNull())
      {
        int  indxBO = 0, indxOP = 0, indxAD = 0, indxS2 = 0;
        bool isS2   = false;
        for (int i = 2; i < n; i++)
        {
          if (a[i][0] == '-')
            indxBO = i;
          else if (a[i][0] == '/')
            indxOP = i;
          else if (a[i][0] == '#')
            indxAD = i;
          else
          {
            isS2   = true;
            indxS2 = i;
          }
        }
        TopoDS_Shape aS2;
        if (isS2)
        {
          if (indxS2 != 2 || Shape(2).IsNull())
            continue;
          aS2 = Shape(2);
        }
        // type, SI, small edges, rebuilt faces, tangent, vertices, edges, C0, curves on surfaces, stop on first
        std::string aModes = "1111000110";
        int         anOp   = BOPAlgo_UNKNOWN;
        bool        anOK   = true;
        if (!aS2.IsNull())
        {
          anOp = BOPAlgo_SECTION;
          if (indxBO)
          {
            anOp = OpOfLetter(a[indxBO].size() > 1 ? a[indxBO][1] : '\0');
            anOK = anOp >= 0;
          }
          aModes[4] = aModes[5] = aModes[6] = '1';
        }
        if (indxOP)
          for (size_t k = 1; k < a[indxOP].size(); k++)
          {
            const char* aKeys   = "RFTVEIPCS";
            const int   aMode[] = {2, 3, 4, 5, 6, 1, 0, 7, 8};
            const char* aPos    = std::strchr(aKeys, std::toupper((unsigned char)a[indxOP][k]));
            if (aPos == nullptr)
              anOK = false;
            else
              aModes[aMode[aPos - aKeys]] = '0';
          }
        if (indxAD)
          for (size_t k = 1; k < a[indxAD].size(); k++)
          {
            char aC = (char)std::toupper((unsigned char)a[indxAD][k]);
            if (aC == 'B')
              aModes[9] = '1';
            else if (aC != 'F')
              anOK = false;
          }
        if (anOK)
          AA(theOut, aCase.Name, Shape(1), aS2, aModes, anOp, aFuzzy, true);
      }
      else if (a[0] == "bopapicheck" && n >= 2 && !Shape(1).IsNull())
      {
        TopoDS_Shape aS2  = n > 2 ? Shape(2) : TopoDS_Shape();
        int          anOp = BOPAlgo_UNKNOWN;
        bool         aSE = true, aSI = true;
        for (int i = aS2.IsNull() ? 2 : 3; i < n; ++i)
        {
          if (a[i] == "-op" && i + 1 < n)
            anOp = OpOfName(a[++i], anOp);
          else if (a[i] == "-se")
            aSE = false;
          else if (a[i] == "-si")
            aSI = false;
        }
        // bopapicheck's BRepAlgoAPI_Check keeps BOPAlgo_Options' default fuzzy value, not the session's
        API(theOut, aCase.Name, Shape(1), aS2, anOp, aSE, aSI, Precision::Confusion());
      }
    }
  }
}

TopoDS_Shape Compound(const std::vector<TopoDS_Shape>& theShapes)
{
  TopoDS_Compound aC;
  BRep_Builder    aB;
  aB.MakeCompound(aC);
  for (const TopoDS_Shape& aS : theShapes)
    aB.Add(aC, aS);
  return aC;
}

TopoDS_Shape ReadOcc(const std::string& theDir, const char* theFile)
{
  TopoDS_Shape aS;
  BRep_Builder aB;
  if (!BRepTools::Read(aS, (theDir + "/" + theFile).c_str(), aB))
    std::fprintf(stderr, "cannot read %s\n", theFile);
  return aS;
}

TopoDS_Shape Shifted(const TopoDS_Shape& theS, double theFx, double theFy, double theFz)
{
  Bnd_Box aBox;
  BRepBndLib::Add(theS, aBox);
  double x0, y0, z0, x1, y1, z1;
  aBox.Get(x0, y0, z0, x1, y1, z1);
  gp_Trsf aMove;
  aMove.SetTranslation(gp_Vec(theFx * (x1 - x0), theFy * (y1 - y0), theFz * (z1 - z0)));
  return theS.Moved(TopLoc_Location(aMove));
}

void RandomCases(std::ostream& theOut, const std::string& theOccDir)
{
  const char*   aNames[] = {"box", "cylinder", "cone", "sphere", "torus"};
  borec::Random aRnd(0x6a09e667f3bcc909ull);
  // CheckerSI on compounds of two or three primitives, every level of check, every seventh with a fuzzy value
  for (int k = 0; k < 120; k++)
  {
    int                       aNb = 2 + (k % 3 == 2 ? 1 : 0);
    std::vector<TopoDS_Shape> aShapes;
    std::string               aName = "random/si";
    for (int i = 0; i < aNb; i++)
    {
      int aKind = (int)(aRnd.Next() * 5) % 5;
      aShapes.push_back(aRnd.Placed(aRnd.Primitive(aKind), 0.3 + 0.6 * i));
      aName += std::string("-") + aNames[aKind];
    }
    double aFuzzy = (k % 7 == 3) ? 1.e-5 : Precision::Confusion();
    SI(theOut, aName + "-" + std::to_string(k), Compound(aShapes), k % 10, aFuzzy);
  }
  // single primitives (no self-interference), touching boxes, a box in a box
  for (int aKind = 0; aKind < 5; aKind++)
    SI(theOut, std::string("random/si-single-") + aNames[aKind], aRnd.Primitive(aKind), 9, Precision::Confusion());
  {
    TopoDS_Shape aB1 = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 1, 1, 1).Shape();
    TopoDS_Shape aB2 = BRepPrimAPI_MakeBox(gp_Pnt(1, 0, 0), 1, 1, 1).Shape();
    TopoDS_Shape aB3 = BRepPrimAPI_MakeBox(gp_Pnt(0.5, 0.5, 1), 1, 1, 1).Shape();
    TopoDS_Shape aB4 = BRepPrimAPI_MakeBox(gp_Pnt(0.25, 0.25, 0.25), 0.5, 0.5, 0.5).Shape();
    for (int aLevel = 0; aLevel < 10; aLevel++)
    {
      SI(theOut, "random/si-touching-boxes", Compound({aB1, aB2, aB3}), aLevel, Precision::Confusion());
      SI(theOut, "random/si-box-in-box", Compound({aB1, aB4}), aLevel, Precision::Confusion());
    }
  }
  // the analyzer and BRepAlgoAPI_Check on pairs of primitives, for every operation
  for (int k = 0; k < 90; k++)
  {
    int          aK1 = (int)(aRnd.Next() * 5) % 5, aK2 = (int)(aRnd.Next() * 5) % 5;
    TopoDS_Shape aS1 = aRnd.Placed(aRnd.Primitive(aK1), 0.3), aS2 = aRnd.Placed(aRnd.Primitive(aK2), 0.6);
    std::string  aName = std::string("random/pair-") + aNames[aK1] + "-" + aNames[aK2] + "-" + std::to_string(k);
    int          anOp  = k % 6;
    if (k % 3 == 0)
    {
      // a compound of the two as a single (self-interfering) shape
      AA(theOut,
         aName + "-single",
         Compound({aS1, aS2}),
         TopoDS_Shape(),
         "1111000110",
         BOPAlgo_UNKNOWN,
         Precision::Confusion(),
         false);
    }
    AA(theOut, aName, aS1, aS2, "1111111110", anOp, Precision::Confusion(), k % 2 == 0);
    if (k % 5 == 0)
      AA(theOut, aName + "-stop", aS1, Compound({aS2, aS1}), "1111111111", anOp, Precision::Confusion(), false);
    API(theOut, aName, aS1, aS2, anOp, k % 4 != 1, k % 4 != 2, Precision::Confusion());
    if (k % 3 == 1)
      API(theOut,
          aName + "-single",
          Compound({aS1, aS2}),
          TopoDS_Shape(),
          BOPAlgo_UNKNOWN,
          true,
          true,
          Precision::Confusion());
  }
  // data/occ shapes: alone and against a moved copy
  const char* anOcc[] = {"bottle.brep",
                         "CrankArm.brep",
                         "hammer.brep",
                         "wedge_ok.brep",
                         "solid.brep",
                         "Ball.brep",
                         "Pump_Nut.brep",
                         "mal_ecrou.brep"};
  for (const char* aFile : anOcc)
  {
    TopoDS_Shape aS = ReadOcc(theOccDir, aFile);
    if (aS.IsNull())
      continue;
    std::string  aName  = std::string("occ/") + aFile;
    TopoDS_Shape aMoved = Shifted(aS, 0.23, 0.11, 0.17);
    SI(theOut, aName, aS, 9, Precision::Confusion());
    SI(theOut, aName + "-shift", Compound({aS, aMoved}), 9, Precision::Confusion());
    AA(theOut, aName, aS, TopoDS_Shape(), "1111000110", BOPAlgo_UNKNOWN, Precision::Confusion(), false);
    AA(theOut, aName + "-shift", aS, aMoved, "1111111110", BOPAlgo_FUSE, Precision::Confusion(), true);
    API(theOut, aName, aS, TopoDS_Shape(), BOPAlgo_UNKNOWN, true, true, Precision::Confusion());
    API(theOut, aName + "-shift", aS, aMoved, BOPAlgo_CUT, true, true, Precision::Confusion());
  }
}
} // namespace

// The environment variable BOREC_OCC names OCCT's data/occ directory (no data/occ cases without it).
int RecordCheck(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut)
{
  Corpus(theCases, theOut);
  const char* anOcc = std::getenv("BOREC_OCC");
  RandomCases(theOut, anOcc != nullptr ? anOcc : "");
  return 0;
}
