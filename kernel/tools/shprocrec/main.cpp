// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// shprocrec: records native OCCT's ShapeProcess (unit 7C), which build123d.NET doesn't bind, on data/occ's shapes and
// the heal group's public inputs: OCCT's default sequences and the heal group's, through ShapeProcess::Perform with a
// ShapeProcess_ShapeContext, and every operator alone through ShapeProcess::Perform with flags (default parameters).
// Built against build123d.NET's native OCCT V8_0_1 (tools/shprocrec/build.ps1).
//
//   shprocrec inputs.txt shprocrec.txt <occt-src> <brep-data-dir>
//
// The resource files are found as DRAWEXE finds them, through CSF_<name>Defaults: IGES and STEP in
// <occt-src>/resources/XSTEPResource (OCCT's defaults, which the port compiles in), data and MDTV in
// <occt-src>/tests/heal/data, res in <brep-data-dir>.
// Per input:
//   FILE <name>
//   <run> <done> <hash> <faces> <edges> <vertices> <map> <mapsig> <msgs> <msghash> <outhash>   or   <run> THROW <type>
// for each run, in this order: the sequences data1 (data), ToV4 (MDTV), remove1v and merge1v (res), ToIGES and FromIGES
// (IGES), ToSTEP and FromSTEP (STEP); then OP:<name> for each operation of ShapeProcess::Operation.
// done: Perform's result (0/1); hash: FNV-1a 64 of the result's BRep text (format 3, triangles, no normals), the Free
// and Checked columns of each line of flags replaced by 'x'; the counts are the result's distinct faces, edges and
// vertices; map: the number of entries of the context's Map; mapsig: FNV-1a 64 of one character per input's root,
// solids, shells and faces (TopExp::MapShapes order; the root first): '-' not in the map, 'n' mapped to null, else
// the type of what it became (C c S s F W E V); msgs: the number of messages of the context's Messages; msghash:
// FNV-1a 64 of their texts sorted; outhash: FNV-1a 64 of what the context's messenger printed (all gravities).
// Each run reads the file again (native writes into its input). With SHPROCREC_TRACE set, what the messenger printed
// goes to the standard error too; with SHPROCREC_DUMP
// set to a directory, the results' texts are written there (<input>.<run>.brep).
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Message_Messenger.hxx>
#include <Message_Msg.hxx>
#include <Message_Printer.hxx>
#include <OSD.hxx>
#include <ShapeExtend_MsgRegistrator.hxx>
#include <ShapeProcess.hxx>
#include <ShapeProcess_OperLibrary.hxx>
#include <ShapeProcess_ShapeContext.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

static std::ofstream gOut;
static std::string   gName;

class StringPrinter : public Message_Printer
{
public:
  std::string myText;
  StringPrinter() { myTraceLevel = Message_Trace; }

protected:
  void send(const TCollection_AsciiString& theString, const Message_Gravity) const override
  {
    const_cast<StringPrinter*>(this)->myText += theString.ToCString();
    const_cast<StringPrinter*>(this)->myText += "\n";
  }
};

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

static void Record(const char*                                     theRun,
                   const TopoDS_Shape&                             theInput,
                   const occ::handle<ShapeProcess_ShapeContext>&   theContext,
                   bool                                            theDone,
                   const StringPrinter&                            thePrinter)
{
  const TopoDS_Shape& aRes = theContext->Result();
  std::ostringstream  aS;
  BRepTools::Write(aRes, aS, true, false, TopTools_FormatVersion_VERSION_3);
  uint64_t aHash = Fnv(Masked(aS.str()));
  if (const char* aDump = std::getenv("SHPROCREC_DUMP"))
  {
    std::ofstream aF(std::string(aDump) + "/" + gName + "." + (theRun[2] == ':' ? theRun + 3 : theRun) + ".brep",
                     std::ios::binary);
    aF << aS.str();
  }
  int      aNb[3] = {0, 0, 0};
  if (!aRes.IsNull())
  {
    const TopAbs_ShapeEnum aTypes[3] = {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX};
    for (int i = 0; i < 3; i++)
    {
      TopTools_IndexedMapOfShape aMap;
      TopExp::MapShapes(aRes, aTypes[i], aMap);
      aNb[i] = aMap.Extent();
    }
  }

  // the map's signature over the input's root, solids, shells and faces
  const auto& aMap = theContext->Map();
  std::string aSig;
  auto        aSign = [&](const TopoDS_Shape& theKey) {
    TopoDS_Shape aKey = theKey.Located(TopLoc_Location());
    aSig += aMap.IsBound(aKey) ? TypeChar(aMap.Find(aKey)) : '-';
  };
  aSign(theInput);
  const TopAbs_ShapeEnum aKeyTypes[3] = {TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE};
  for (int t = 0; t < 3; t++)
  {
    TopTools_IndexedMapOfShape aKeys;
    TopExp::MapShapes(theInput, aKeyTypes[t], aKeys);
    for (int i = 1; i <= aKeys.Extent(); i++)
    {
      aSign(aKeys(i));
    }
  }

  // the messages' texts, sorted
  std::vector<std::string> aTexts;
  if (!theContext->Messages().IsNull())
  {
    for (NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>, TopTools_ShapeMapHasher>::Iterator anIt(
           theContext->Messages()->MapShape());
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

  char aBuf[320];
  std::snprintf(aBuf,
                sizeof(aBuf),
                "%s %d %016llx %d %d %d %d %016llx %d %016llx %016llx\n",
                theRun,
                theDone ? 1 : 0,
                (unsigned long long)aHash,
                aNb[0],
                aNb[1],
                aNb[2],
                aMap.Extent(),
                (unsigned long long)Fnv(aSig),
                (int)aTexts.size(),
                (unsigned long long)Fnv(aJoined),
                (unsigned long long)Fnv(thePrinter.myText));
  gOut << aBuf;
  if (std::getenv("SHPROCREC_TRACE") != nullptr && !thePrinter.myText.empty())
  {
    std::fprintf(stderr, "%s: %s", theRun, thePrinter.myText.c_str());
  }
}

static TopoDS_Shape Fresh(const std::string& thePath)
{
  TopoDS_Shape aShape;
  BRep_Builder aB;
  BRepTools::Read(aShape, thePath.c_str(), aB);
  return aShape;
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

static void Sequence(const std::string& thePath, const char* theRsc, const char* theSeq)
{
  Run(theSeq, [&]() {
    TopoDS_Shape                           anInput = Fresh(thePath);
    TopoDS_Shape                           aKeep   = Fresh(thePath); // the keys, as read
    occ::handle<StringPrinter>             aPrinter = new StringPrinter;
    occ::handle<ShapeProcess_ShapeContext> aContext = new ShapeProcess_ShapeContext(anInput, theRsc);
    aContext->SetMessenger(new Message_Messenger(aPrinter));
    bool isDone = ShapeProcess::Perform(aContext, theSeq);
    (void)aKeep;
    Record(theSeq, anInput, aContext, isDone, *aPrinter);
  });
}

static void Operation(const std::string& thePath, ShapeProcess::Operation theOperation, const char* theName)
{
  std::string aRun = std::string("OP:") + theName;
  Run(aRun.c_str(), [&]() {
    TopoDS_Shape                           anInput  = Fresh(thePath);
    occ::handle<StringPrinter>             aPrinter = new StringPrinter;
    occ::handle<ShapeProcess_ShapeContext> aContext = new ShapeProcess_ShapeContext(anInput, "");
    aContext->SetMessenger(new Message_Messenger(aPrinter));
    ShapeProcess::OperationsFlags aFlags;
    aFlags.set(theOperation);
    bool isDone = ShapeProcess::Perform(aContext, aFlags);
    Record(aRun.c_str(), anInput, aContext, isDone, *aPrinter);
  });
}

static const char* const THE_OPERATIONS[] = {
  "DirectFaces", "SameParameter", "SetTolerance", "SplitAngle", "BSplineRestriction", "ElementaryToRevolution",
  "SweptToElementary", "SurfaceToBSpline", "ToBezier", "SplitContinuity", "SplitClosedFaces", "FixWireGaps",
  "FixFaceSize", "DropSmallSolids", "DropSmallEdges", "FixShape", "SplitClosedEdges", "SplitCommonVertex"};

static void Runs(const std::string& thePath)
{
  Sequence(thePath, "data", "data1");
  Sequence(thePath, "MDTV", "ToV4");
  Sequence(thePath, "res", "remove1v");
  Sequence(thePath, "res", "merge1v");
  Sequence(thePath, "IGES", "ToIGES");
  Sequence(thePath, "IGES", "FromIGES");
  Sequence(thePath, "STEP", "ToSTEP");
  Sequence(thePath, "STEP", "FromSTEP");
  for (int i = ShapeProcess::First; i <= ShapeProcess::Last; i++)
  {
    Operation(thePath, (ShapeProcess::Operation)i, THE_OPERATIONS[i]);
  }
}

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    std::fprintf(stderr, "usage: shprocrec inputs.txt out.txt <occt-src> <brep-data-dir>\n");
    return 2;
  }
  OSD::SetSignal(false);
  std::string aSrc = argv[3];
  std::string aHeal = "CSF_dataDefaults=" + aSrc + "/tests/heal/data";
  std::string aMdtv = "CSF_MDTVDefaults=" + aSrc + "/tests/heal/data";
  std::string aRes  = std::string("CSF_resDefaults=") + argv[4];
  std::string aIges = "CSF_IGESDefaults=" + aSrc + "/resources/XSTEPResource";
  std::string aStep = "CSF_STEPDefaults=" + aSrc + "/resources/XSTEPResource";
  _putenv(aHeal.c_str());
  _putenv(aMdtv.c_str());
  _putenv(aRes.c_str());
  _putenv(aIges.c_str());
  _putenv(aStep.c_str());
  ShapeProcess_OperLibrary::Init();

  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  std::string aPath;
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
    TopoDS_Shape aShape;
    BRep_Builder aB;
    try
    {
      OCC_CATCH_SIGNALS
      if (!BRepTools::Read(aShape, aPath.c_str(), aB) || aShape.IsNull())
      {
        continue;
      }
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    gName = aPath.substr(aPath.find_last_of("/\\") + 1);
    gOut << "FILE " << gName << "\n";
    Runs(aPath);
    gOut.flush();
  }
  return 0;
}
