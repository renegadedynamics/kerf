// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// xsalgorec: records native OCCT's XSAlgo (unit 8A2), which build123d.NET doesn't bind, on data/occ's shapes: the
// shape processor with STEP's default operator sequences (read from resources/XSTEPResource/STEP through
// ReadProcessingData, as XSControl's readers do, and DESTEP's write flags), the merge of its history into a transient
// process and a finder process, the AlgoContainer's ProcessShape (its STEP sequence and its default fixes), and
// CheckPCurve on every edge of every face.
// Built against build123d.NET's native OCCT V8_0_1 (tools/xsalgorec/build.ps1).
//
//   xsalgorec inputs.txt xsalgorec.txt <occt-src>
//
// Per input:
//   FILE <name>
//   PARAMS <scope> <flags> <hash>                 the parameters and operations ReadProcessingData gives (once)
//   <run> <same> <hash> <faces> <edges> <vertices> <map> <msgs> <msghash> <merge>   or   <run> THROW <type>
// runs: READ (FromSTEP's parameters and operations, the tolerances set as STEPControl_ActorRead does), WRITE (the
// default DE_ShapeFixParameters, DESTEP's write operations SplitCommonVertex and DirectFaces), TOSTEP (ToSTEP's
// parameters and operations), AC-READ (XSAlgo_AlgoContainer::ProcessShape with read.step.sequence), AC-FIX (with a
// sequence the resource file doesn't define, "read.none": the default ShapeFix), AC-WRITE ("write.none": DirectFaces);
// then
//   PCURVE <n> <true> <hash> <brephash>
// for CheckPCurve (precision 1e-3, read.stdsameparameter.mode 0) on each face of the input (TopExp::MapShapes order)
// and each edge of the face (TopExp_Explorer order), on a fresh copy read from the file: n checks, the number that
// returned true, the FNV-1a 64 of one line per check (the result and the edge's tolerance, %.17g), and the hash of the
// shape's BRep text afterwards.
// same: 1 if the result IsSame the input; hash: FNV-1a 64 of the result's BRep text (format 3, triangles, no normals),
// the Free and Checked columns masked; the counts are the result's distinct faces, edges and vertices; map: the
// context's Map's extent; msgs and msghash: the messages of the context (their sorted texts); merge: for READ and the
// AC runs, the FNV-1a 64 of one line per sub-shape of the input (root, solids, shells, faces, edges in TopExp::MapShapes
// order) bound as a ShapeBinder in a TransientProcess, after MergeTransferInfo: the type of its result, whether it is
// the same shape as before, and its warnings; for WRITE and TOSTEP, the FNV-1a 64 of the sorted lines of a
// FinderProcess (each face pre-bound to a transient) after MergeTransferInfo: per mapped item, the mapper's type name,
// the binder's result types along its chain, and its warnings count. With XSALGOREC_TRACE set, the finder process's
// lines go to the standard error too.
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Interface_Check.hxx>
#include <Interface_Static.hxx>
#include <Message_Msg.hxx>
#include <OSD.hxx>
#include <ShapeExtend_MsgRegistrator.hxx>
#include <ShapeProcess_ShapeContext.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TransferBRep.hxx>
#include <TransferBRep_ShapeBinder.hxx>
#include <TransferBRep_ShapeMapper.hxx>
#include <Transfer_FinderProcess.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSAlgo.hxx>
#include <XSAlgo_AlgoContainer.hxx>
#include <XSAlgo_ShapeProcessor.hxx>

static std::ofstream gOut;

static uint64_t Fnv(const std::string& theText)
{
  uint64_t aHash = 14695981039346656037ull;
  for (unsigned char c : theText)
  {
    aHash ^= c;
    aHash *= 1099511628211ull;
  }
  return aHash;
}

static std::string Masked(const std::string& theText)
{
  std::string        aRes;
  std::istringstream aS(theText);
  std::string        aLine;
  while (std::getline(aS, aLine))
  {
    bool isFlags = aLine.size() == 7;
    for (size_t i = 0; isFlags && i < 7; i++)
    {
      isFlags = aLine[i] == '0' || aLine[i] == '1';
    }
    if (isFlags)
    {
      aLine[0] = 'x';
      aLine[2] = 'x';
    }
    aRes += aLine;
    aRes += '\n';
  }
  return aRes;
}

static uint64_t BRepHash(const TopoDS_Shape& theShape)
{
  std::ostringstream aS;
  BRepTools::Write(theShape, aS, true, false, TopTools_FormatVersion_VERSION_3);
  return Fnv(Masked(aS.str()));
}

static char TypeChar(const TopoDS_Shape& theShape)
{
  if (theShape.IsNull())
  {
    return 'n';
  }
  switch (theShape.ShapeType())
  {
    case TopAbs_COMPOUND: return 'C';
    case TopAbs_COMPSOLID: return 'c';
    case TopAbs_SOLID: return 'S';
    case TopAbs_SHELL: return 's';
    case TopAbs_FACE: return 'F';
    case TopAbs_WIRE: return 'W';
    case TopAbs_EDGE: return 'E';
    case TopAbs_VERTEX: return 'V';
    default: return '?';
  }
}

static int Count(const TopoDS_Shape& theShape, TopAbs_ShapeEnum theType)
{
  if (theShape.IsNull())
  {
    return 0;
  }
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, theType, aMap);
  return aMap.Extent();
}

static TopoDS_Shape Fresh(const std::string& thePath)
{
  TopoDS_Shape aShape;
  BRep_Builder aB;
  BRepTools::Read(aShape, thePath.c_str(), aB);
  return aShape;
}

// The input's root, solids, shells, faces and edges (TopExp::MapShapes order; the root first)
static std::vector<TopoDS_Shape> Keys(const TopoDS_Shape& theShape)
{
  std::vector<TopoDS_Shape> aKeys{theShape};
  const TopAbs_ShapeEnum    aTypes[4] = {TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_EDGE};
  for (int t = 0; t < 4; t++)
  {
    TopTools_IndexedMapOfShape aMap;
    TopExp::MapShapes(theShape, aTypes[t], aMap);
    for (int i = 1; i <= aMap.Extent(); i++)
    {
      aKeys.push_back(aMap(i));
    }
  }
  return aKeys;
}

static std::string Messages(const occ::handle<ShapeExtend_MsgRegistrator>& theMessages, int& theNb)
{
  std::vector<std::string> aTexts;
  if (!theMessages.IsNull())
  {
    for (NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>, TopTools_ShapeMapHasher>::Iterator anIt(
           theMessages->MapShape());
         anIt.More();
         anIt.Next())
    {
      for (NCollection_List<Message_Msg>::Iterator aMsg(anIt.Value()); aMsg.More(); aMsg.Next())
      {
        aTexts.push_back(TCollection_AsciiString(aMsg.Value().Value(), '?').ToCString());
      }
    }
  }
  std::sort(aTexts.begin(), aTexts.end());
  std::string aJoined;
  for (const std::string& aText : aTexts)
  {
    aJoined += aText;
    aJoined += '\n';
  }
  theNb = (int)aTexts.size();
  return aJoined;
}

// The merge into a transient process: each key bound to a ShapeBinder of itself, then the context's history merged
static uint64_t MergeTP(const std::vector<TopoDS_Shape>&               theKeys,
                        const std::function<void(const occ::handle<Transfer_TransientProcess>&)>& theMerge)
{
  occ::handle<Transfer_TransientProcess> aTP = new Transfer_TransientProcess;
  for (size_t i = 0; i < theKeys.size(); i++)
  {
    TransferBRep::SetShapeResult(aTP, new TCollection_HAsciiString((int)i), theKeys[i]);
  }
  theMerge(aTP);
  std::string aLines;
  for (int i = 1; i <= aTP->NbMapped(); i++)
  {
    occ::handle<TransferBRep_ShapeBinder> aB = occ::down_cast<TransferBRep_ShapeBinder>(aTP->MapItem(i));
    char aBuf[64];
    std::snprintf(aBuf,
                  sizeof(aBuf),
                  "%c %d %d\n",
                  TypeChar(aB->Result()),
                  aB->Result().IsSame(theKeys[i - 1]) ? 1 : 0,
                  aB->Check()->NbWarnings());
    aLines += aBuf;
  }
  return Fnv(aLines);
}

// The merge into a finder process: each face of the input pre-bound to a transient, then the history merged
static uint64_t MergeFP(const TopoDS_Shape&                                                   theInput,
                        const std::function<void(const occ::handle<Transfer_FinderProcess>&)>& theMerge)
{
  occ::handle<Transfer_FinderProcess> aFP = new Transfer_FinderProcess;
  TopTools_IndexedMapOfShape          aFaces;
  TopExp::MapShapes(theInput, TopAbs_FACE, aFaces);
  for (int i = 1; i <= aFaces.Extent(); i++)
  {
    TransferBRep::SetTransientFromShape(aFP, aFaces(i), new TCollection_HAsciiString(i));
  }
  theMerge(aFP);
  std::vector<std::string> aLines;
  for (int i = 1; i <= aFP->NbMapped(); i++)
  {
    std::string                  aLine = aFP->Mapped(i)->ValueTypeName();
    occ::handle<Transfer_Binder> aB    = aFP->MapItem(i);
    int                          aNbW  = 0;
    for (occ::handle<Transfer_Binder> aN = aB; !aN.IsNull(); aN = aN->NextResult())
    {
      aLine += " ";
      aLine += aN->ResultTypeName();
      aNbW += aN->Check()->NbWarnings();
    }
    aLine += " " + std::to_string(aNbW);
    aLines.push_back(aLine);
  }
  std::sort(aLines.begin(), aLines.end());
  std::string aJoined;
  for (const std::string& aLine : aLines)
  {
    aJoined += aLine;
    aJoined += '\n';
  }
  if (std::getenv("XSALGOREC_TRACE") != nullptr)
  {
    std::fprintf(stderr, "FP:\n%s", aJoined.c_str());
  }
  return Fnv(aJoined);
}

static void Record(const char*                                   theRun,
                   const TopoDS_Shape&                           theInput,
                   const TopoDS_Shape&                           theResult,
                   const occ::handle<ShapeProcess_ShapeContext>& theContext,
                   uint64_t                                      theMerge)
{
  int         aNbMsg = 0;
  std::string aMsgs  = theContext.IsNull() ? std::string() : Messages(theContext->Messages(), aNbMsg);
  char        aBuf[320];
  std::snprintf(aBuf,
                sizeof(aBuf),
                "%s %d %016llx %d %d %d %d %d %016llx %016llx\n",
                theRun,
                theResult.IsSame(theInput) ? 1 : 0,
                (unsigned long long)BRepHash(theResult),
                Count(theResult, TopAbs_FACE),
                Count(theResult, TopAbs_EDGE),
                Count(theResult, TopAbs_VERTEX),
                theContext.IsNull() ? 0 : theContext->Map().Extent(),
                aNbMsg,
                (unsigned long long)Fnv(aMsgs),
                (unsigned long long)theMerge);
  gOut << aBuf;
}

static void Run(const char* theRun, const std::function<void()>& theBody)
{
  try
  {
    OCC_CATCH_SIGNALS
    theBody();
  }
  catch (const Standard_Failure& e)
  {
    gOut << theRun << " THROW " << e.ExceptionType() << "\n";
  }
}

static std::string FlagsText(const ShapeProcess::OperationsFlags& theFlags)
{
  std::string aText;
  for (int i = ShapeProcess::First; i <= ShapeProcess::Last; i++)
  {
    aText += theFlags.test(i) ? '1' : '0';
  }
  return aText;
}

static uint64_t ParamsHash(const XSAlgo_ShapeProcessor::ParameterMap& theParams)
{
  std::vector<std::string> aLines;
  for (XSAlgo_ShapeProcessor::ParameterMap::Iterator anIt(theParams); anIt.More(); anIt.Next())
  {
    aLines.push_back(std::string(anIt.Key().ToCString()) + "=" + anIt.Value().ToCString());
  }
  std::sort(aLines.begin(), aLines.end());
  std::string aJoined;
  for (const std::string& aLine : aLines)
  {
    aJoined += aLine;
    aJoined += '\n';
  }
  return Fnv(aJoined);
}

static void Processor(const std::string&                         thePath,
                      const char*                                theRun,
                      const XSAlgo_ShapeProcessor::ParameterMap& theParams,
                      const ShapeProcess::OperationsFlags&       theFlags,
                      bool                                       theToFinder)
{
  Run(theRun, [&]() {
    TopoDS_Shape          anInput = Fresh(thePath);
    XSAlgo_ShapeProcessor aProcessor(theParams);
    TopoDS_Shape          aResult = aProcessor.ProcessShape(anInput, theFlags, Message_ProgressRange());
    uint64_t              aMerge  = 0;
    if (theToFinder)
    {
      aMerge = MergeFP(anInput, [&](const occ::handle<Transfer_FinderProcess>& theFP) {
        aProcessor.MergeTransferInfo(theFP);
      });
    }
    else
    {
      aMerge = MergeTP(Keys(anInput), [&](const occ::handle<Transfer_TransientProcess>& theTP) {
        aProcessor.MergeTransferInfo(theTP, 1);
      });
    }
    Record(theRun, anInput, aResult, aProcessor.GetContext(), aMerge);
  });
}

static void Container(const std::string& thePath, const char* theRun, const char* theSeq)
{
  Run(theRun, [&]() {
    TopoDS_Shape                    anInput = Fresh(thePath);
    occ::handle<Standard_Transient> anInfo;
    TopoDS_Shape                    aResult = XSAlgo::AlgoContainer()->ProcessShape(anInput,
                                                                 1.e-3,
                                                                 1.,
                                                                 "read.step.resource.name",
                                                                 theSeq,
                                                                 anInfo,
                                                                 Message_ProgressRange());
    uint64_t aMerge = MergeTP(Keys(anInput), [&](const occ::handle<Transfer_TransientProcess>& theTP) {
      XSAlgo::AlgoContainer()->MergeTransferInfo(theTP, anInfo, 1);
    });
    Record(theRun, anInput, aResult, occ::down_cast<ShapeProcess_ShapeContext>(anInfo), aMerge);
  });
}

static void PCurves(const std::string& thePath)
{
  Run("PCURVE", [&]() {
    TopoDS_Shape               anInput = Fresh(thePath);
    TopTools_IndexedMapOfShape aFaces;
    TopExp::MapShapes(anInput, TopAbs_FACE, aFaces);
    int         aNb = 0, aNbTrue = 0;
    std::string aLines;
    for (int i = 1; i <= aFaces.Extent(); i++)
    {
      const TopoDS_Face& aFace = TopoDS::Face(aFaces(i));
      for (TopExp_Explorer anExp(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
      {
        const TopoDS_Edge& anEdge = TopoDS::Edge(anExp.Current());
        bool isSeam = BRep_Tool::IsClosed(anEdge, aFace);
        bool isOk   = XSAlgo_ShapeProcessor::CheckPCurve(anEdge, aFace, 1.e-3, isSeam);
        char aBuf[64];
        std::snprintf(aBuf, sizeof(aBuf), "%d %.17g\n", isOk ? 1 : 0, BRep_Tool::Tolerance(anEdge));
        aLines += aBuf;
        aNb++;
        aNbTrue += isOk ? 1 : 0;
      }
    }
    char aBuf[160];
    std::snprintf(aBuf,
                  sizeof(aBuf),
                  "PCURVE %d %d %016llx %016llx\n",
                  aNb,
                  aNbTrue,
                  (unsigned long long)Fnv(aLines),
                  (unsigned long long)BRepHash(anInput));
    gOut << aBuf;
  });
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::fprintf(stderr, "usage: xsalgorec inputs.txt out.txt <occt-src>\n");
    return 2;
  }
  OSD::SetSignal(false);
  std::string aStep = std::string("CSF_STEPDefaults=") + argv[3] + "/resources/XSTEPResource";
  _putenv(aStep.c_str());
  XSAlgo::Init();
  // STEPControl_Controller's statics naming STEP's resource file and sequences
  Interface_Static::Init("step", "read.step.resource.name", 't', "STEP");
  Interface_Static::Init("step", "read.step.sequence", 't', "FromSTEP");
  Interface_Static::Init("step", "write.step.resource.name", 't', "STEP");
  Interface_Static::Init("step", "write.step.sequence", 't', "ToSTEP");

  XSAlgo_ShapeProcessor::ProcessingData aRead =
    XSAlgo_ShapeProcessor::ReadProcessingData("read.step.resource.name", "read.step.sequence");
  XSAlgo_ShapeProcessor::SetParameter("FixShape.Tolerance3d", 1.e-3, true, aRead.first);
  XSAlgo_ShapeProcessor::SetParameter("FixShape.MaxTolerance3d", 1., true, aRead.first);
  XSAlgo_ShapeProcessor::ProcessingData aToStep =
    XSAlgo_ShapeProcessor::ReadProcessingData("write.step.resource.name", "write.step.sequence");
  XSAlgo_ShapeProcessor::ParameterMap aWriteParams;
  XSAlgo_ShapeProcessor::FillParameterMap(DE_ShapeFixParameters(), true, aWriteParams);
  ShapeProcess::OperationsFlags aWriteFlags;
  aWriteFlags.set(ShapeProcess::Operation::SplitCommonVertex);
  aWriteFlags.set(ShapeProcess::Operation::DirectFaces);

  gOut.open(argv[2], std::ios::binary);
  {
    char aBuf[200];
    std::snprintf(aBuf, sizeof(aBuf), "PARAMS FromSTEP %s %016llx\n", FlagsText(aRead.second).c_str(),
                  (unsigned long long)ParamsHash(aRead.first));
    gOut << aBuf;
    std::snprintf(aBuf, sizeof(aBuf), "PARAMS ToSTEP %s %016llx\n", FlagsText(aToStep.second).c_str(),
                  (unsigned long long)ParamsHash(aToStep.first));
    gOut << aBuf;
    std::snprintf(aBuf, sizeof(aBuf), "PARAMS DE %s %016llx\n", FlagsText(aWriteFlags).c_str(),
                  (unsigned long long)ParamsHash(aWriteParams));
    gOut << aBuf;
  }

  std::ifstream aList(argv[1]);
  std::string   aPath;
  while (std::getline(aList, aPath))
  {
    if (!aPath.empty() && aPath.back() == '\r')
    {
      aPath.pop_back();
    }
    if (aPath.empty() || Fresh(aPath).IsNull())
    {
      continue;
    }
    gOut << "FILE " << aPath.substr(aPath.find_last_of("/\\") + 1) << "\n";
    Processor(aPath, "READ", aRead.first, aRead.second, false);
    Processor(aPath, "WRITE", aWriteParams, aWriteFlags, true);
    Processor(aPath, "TOSTEP", aToStep.first, aToStep.second, true);
    Container(aPath, "AC-READ", "read.step.sequence");
    Container(aPath, "AC-FIX", "read.none");
    Container(aPath, "AC-WRITE", "write.none");
    PCurves(aPath);
    gOut.flush();
  }
  return 0;
}
