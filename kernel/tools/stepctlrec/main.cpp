// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// stepctlrec: records what native OCCT's STEPControl_Reader and STEPControl_Writer do, for unit 8F
// (tools/stepctlrec/build.ps1). build123d.NET binds the reader and the writer but neither the statics nor the check
// lists: this recorder covers the schemas and the transfers' messages, the in-process oracle the rest.
//
//   stepctlrec read <list> <out>
//       list: one "name<TAB>path" per line (the corpus's STEP files). Each file is read with STEPControl_Reader
//       (OCCT's default statics) and its roots transferred. Per file:
//         "F <name>"
//         "R <read status> <roots> <transferred> <shapes>"
//         "L <entities with a check> <fails> <warnings>"   the model's check list (loading)
//         "T <entities with a check> <fails> <warnings>"   the transient process's check list (transfer)
//         "M <count> <F|W> <message>"                      each distinct message of the transfer's checks, sorted
//   stepctlrec write <list> <out> [<dumpdir>]
//       list: "name<TAB>path" of BRep files. Each shape, and three assemblies of located instances made from
//       Ball.brep, Pump_Nut.brep and bottle.brep (two instances, one rotated and moved, the pair instanced twice), is
//       written with STEPControl_Writer in each schema (write.step.schema AP203, AP214IS, AP242DIS) and each model
//       type (AsIs, ManifoldSolidBrep, BrepWithVoids, FacetedBrep, ShellBasedSurfaceModel, GeometricCurveSet), to a
//       stream. Per case: "W <name> <schema> <mode> <status> <entities> <sha256 of the masked text> <its length>"
//       and "C <entities with a check> <fails> <warnings>" (the session's TransferWriteCheckList). The text is
//       masked as the port's replay masks it (Mask below): the records FILE_NAME, CALENDAR_DATE, LOCAL_TIME,
//       COORDINATED_UNIVERSAL_TIME_OFFSET, PERSON and ORGANIZATION (the time, the user, the host) have their
//       parameters replaced by "(*)", the NEXT_ASSEMBLY_USAGE_OCCURRENCEs their identifier by '#', and the product
//       names' index ("Open CASCADE STEP translator 8.0 <n>", counted per process) by '#'. With a dump directory, the
//       masked texts are written there too.
//
// The port's replay (tests/OcctNet.DataExchange.Tests/STEPControl/STEPControlNativeTests.cs) computes the same lines.
#include <windows.h>
#include <bcrypt.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Check.hxx>
#include <Interface_CheckIterator.hxx>
#include <Interface_Static.hxx>
#include <OSD.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StepData_StepModel.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

static std::string Sha256(const std::string& theData)
{
  BCRYPT_ALG_HANDLE  anAlg = nullptr;
  BCRYPT_HASH_HANDLE aHash = nullptr;
  unsigned char      aOut[32];
  BCryptOpenAlgorithmProvider(&anAlg, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
  BCryptCreateHash(anAlg, &aHash, nullptr, 0, nullptr, 0, 0);
  BCryptHashData(aHash, (PUCHAR)theData.data(), (ULONG)theData.size(), 0);
  BCryptFinishHash(aHash, aOut, 32, 0);
  BCryptDestroyHash(aHash);
  BCryptCloseAlgorithmProvider(anAlg, 0);
  std::string aRes;
  char        aBuf[4];
  for (int i = 0; i < 32; i++)
  {
    sprintf(aBuf, "%02x", aOut[i]);
    aRes += aBuf;
  }
  return aRes;
}

static std::vector<std::pair<std::string, std::string>> ReadList(const char* theList)
{
  std::vector<std::pair<std::string, std::string>> aRes;
  std::ifstream                                    aList(theList);
  std::string                                      aLine;
  while (std::getline(aList, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
      aLine.pop_back();
    size_t aTab = aLine.find('\t');
    if (aTab != std::string::npos)
      aRes.emplace_back(aLine.substr(0, aTab), aLine.substr(aTab + 1));
  }
  return aRes;
}

static std::string SafeName(std::string theName)
{
  for (char& c : theName)
    if (c == '/' || c == '\\' || c == ':' || c == ' ')
      c = '_';
  return theName;
}

// ---------------------------------------------------------------------------------------------------------------
// Reading

static void CountChecks(const Interface_CheckIterator&       theList,
                        const char*                          theTag,
                        std::ostream&                        theOut,
                        std::map<std::string, int>*          theMessages)
{
  int aNbItems = 0, aNbF = 0, aNbW = 0;
  for (Interface_CheckIterator anIt = theList; anIt.More(); anIt.Next())
  {
    const occ::handle<Interface_Check>& aCheck = anIt.Value();
    if (aCheck->NbFails() + aCheck->NbWarnings() == 0)
      continue;
    aNbItems++;
    aNbF += aCheck->NbFails();
    aNbW += aCheck->NbWarnings();
    if (theMessages != nullptr)
    {
      for (int i = 1; i <= aCheck->NbFails(); i++)
        (*theMessages)[std::string("F ") + aCheck->CFail(i)]++;
      for (int i = 1; i <= aCheck->NbWarnings(); i++)
        (*theMessages)[std::string("W ") + aCheck->CWarning(i)]++;
    }
  }
  theOut << theTag << " " << aNbItems << " " << aNbF << " " << aNbW << "\n";
}

static int Read(const char* theList, const char* theOut)
{
  std::ofstream anOut(theOut, std::ios::binary);
  for (const auto& aFile : ReadList(theList))
  {
    anOut << "F " << aFile.first << "\n";
    try
    {
      OCC_CATCH_SIGNALS
      STEPControl_Reader    aReader;
      IFSelect_ReturnStatus aStatus = aReader.ReadFile(aFile.second.c_str());
      if (aStatus != IFSelect_RetDone)
      {
        anOut << "R " << (int)aStatus << " 0 0 0\n";
        continue;
      }
      int aNbRoots = aReader.NbRootsForTransfer();
      int aNbTr    = aReader.TransferRoots();
      anOut << "R " << (int)aStatus << " " << aNbRoots << " " << aNbTr << " " << aReader.NbShapes() << "\n";
      CountChecks(aReader.WS()->ModelCheckList(), "L", anOut, nullptr);
      std::map<std::string, int> aMessages;
      CountChecks(aReader.WS()->TransferReader()->TransientProcess()->CheckList(false), "T", anOut, &aMessages);
      for (const auto& aMessage : aMessages)
        anOut << "M " << aMessage.second << " " << aMessage.first << "\n";
    }
    catch (Standard_Failure const& anExc)
    {
      anOut << "EXC " << anExc.what() << "\n";
    }
  }
  return 0;
}

// ---------------------------------------------------------------------------------------------------------------
// Writing

// The text's records with their varying parts masked (the port's replay masks the same way)
static std::string Mask(const std::string& theText)
{
  static const char* const aMasked[] = {"FILE_NAME", "CALENDAR_DATE", "LOCAL_TIME",
                                        "COORDINATED_UNIVERSAL_TIME_OFFSET", "PERSON", "ORGANIZATION"};
  std::string aRes;
  size_t      aPos = 0;
  while (aPos < theText.size())
  {
    // a record runs to ";\n"
    size_t      anEnd    = theText.find(";\n", aPos);
    size_t      aNext    = anEnd == std::string::npos ? theText.size() : anEnd + 2;
    std::string aRecord  = theText.substr(aPos, aNext - aPos);
    aPos                 = aNext;
    size_t      aTypeBeg = 0;
    if (!aRecord.empty() && aRecord[0] == '#')
    {
      size_t anEq = aRecord.find(" = ");
      aTypeBeg    = anEq == std::string::npos ? aRecord.size() : anEq + 3;
    }
    size_t      aParen = aRecord.find('(', aTypeBeg);
    std::string aType  = aParen == std::string::npos ? "" : aRecord.substr(aTypeBeg, aParen - aTypeBeg);
    bool        isMasked = false;
    for (const char* aName : aMasked)
      isMasked = isMasked || aType == aName;
    if (isMasked)
    {
      aRes += aRecord.substr(0, aParen) + "(*);\n";
      continue;
    }
    if (aType == "NEXT_ASSEMBLY_USAGE_OCCURRENCE" && aParen + 1 < aRecord.size() && aRecord[aParen + 1] == '\'')
    {
      size_t aQuote = aRecord.find('\'', aParen + 2);
      aRecord       = aRecord.substr(0, aParen + 2) + "#" + aRecord.substr(aQuote);
    }
    aRes += aRecord;
  }
  // the product names' index
  const std::string aPrefix = "Open CASCADE STEP translator 8.0 ";
  std::string       aMaskedText;
  size_t            aFrom = 0;
  for (size_t aHit = aRes.find(aPrefix); aHit != std::string::npos; aHit = aRes.find(aPrefix, aFrom))
  {
    size_t anIndex = aHit + aPrefix.size();
    size_t aStop   = anIndex;
    while (aStop < aRes.size() && ((aRes[aStop] >= '0' && aRes[aStop] <= '9') || aRes[aStop] == '.'))
      aStop++;
    aMaskedText += aRes.substr(aFrom, anIndex - aFrom) + "#";
    aFrom = aStop;
  }
  aMaskedText += aRes.substr(aFrom);
  return aMaskedText;
}

static TopoDS_Shape ReadBRep(const std::string& thePath)
{
  TopoDS_Shape aShape;
  BRep_Builder aBuilder;
  BRepTools::Read(aShape, thePath.c_str(), aBuilder);
  return aShape;
}

// An assembly of located instances, as the oracle tests make it
static TopoDS_Shape Assembly(const TopoDS_Shape& theShape)
{
  BRep_Builder aBuilder;
  gp_Trsf      aTrsf;
  aTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0.5);
  aTrsf.SetTranslationPart(gp_Vec(100.0, 50.0, 0.0));
  TopoDS_Compound anAssembly;
  aBuilder.MakeCompound(anAssembly);
  aBuilder.Add(anAssembly, theShape);
  aBuilder.Add(anAssembly, theShape.Moved(TopLoc_Location(aTrsf)));
  TopoDS_Compound aTop;
  aBuilder.MakeCompound(aTop);
  gp_Trsf aTrsf2;
  aTrsf2.SetTranslation(gp_Vec(0.0, 0.0, 300.0));
  aBuilder.Add(aTop, anAssembly);
  aBuilder.Add(aTop, anAssembly.Moved(TopLoc_Location(aTrsf2)));
  return aTop;
}

static int Write(const char* theList, const char* theOut, const char* theDump)
{
  std::vector<std::pair<std::string, TopoDS_Shape>> aShapes;
  std::map<std::string, std::string>                aPaths;
  for (const auto& aFile : ReadList(theList))
  {
    aShapes.emplace_back(aFile.first, ReadBRep(aFile.second));
    aPaths[aFile.first.substr(aFile.first.rfind('/') + 1)] = aFile.second;
  }
  for (const char* aName : {"Ball.brep", "Pump_Nut.brep", "bottle.brep"})
  {
    if (aPaths.count(aName) != 0)
      aShapes.emplace_back(std::string("assembly of ") + aName, Assembly(ReadBRep(aPaths[aName])));
  }

  static const char* const aSchemas[] = {"AP203", "AP214IS", "AP242DIS"};
  static const std::pair<STEPControl_StepModelType, const char*> aModes[] = {
    {STEPControl_AsIs, "AsIs"},
    {STEPControl_ManifoldSolidBrep, "ManifoldSolidBrep"},
    {STEPControl_BrepWithVoids, "BrepWithVoids"},
    {STEPControl_FacetedBrep, "FacetedBrep"},
    {STEPControl_ShellBasedSurfaceModel, "ShellBasedSurfaceModel"},
    {STEPControl_GeometricCurveSet, "GeometricCurveSet"}};

  std::ofstream anOut(theOut, std::ios::binary);
  for (const auto& aShape : aShapes)
  {
    for (const char* aSchema : aSchemas)
    {
      Interface_Static::SetCVal("write.step.schema", aSchema);
      for (const auto& aMode : aModes)
      {
        anOut << "W " << SafeName(aShape.first) << " " << aSchema << " " << aMode.second << " ";
        try
        {
          OCC_CATCH_SIGNALS
          STEPControl_Writer    aWriter;
          IFSelect_ReturnStatus aStatus = aWriter.Transfer(aShape.second, aMode.first);
          if (aStatus != IFSelect_RetDone)
          {
            anOut << (int)aStatus << "\n";
            continue;
          }
          std::ostringstream aStream;
          aWriter.WriteStream(aStream);
          std::string aText = Mask(aStream.str());
          anOut << (int)aStatus << " " << aWriter.Model()->NbEntities() << " " << Sha256(aText) << " "
                << aText.size() << "\n";
          CountChecks(aWriter.WS()->TransferWriteCheckList(), "C", anOut, nullptr);
          if (theDump != nullptr)
          {
            std::ofstream aDump(std::string(theDump) + "/" + SafeName(aShape.first) + "." + aSchema + "."
                                  + aMode.second + ".stp",
                                std::ios::binary);
            aDump << aText;
          }
        }
        catch (Standard_Failure const& anExc)
        {
          anOut << "EXC " << anExc.what() << "\n";
        }
      }
    }
  }
  Interface_Static::SetCVal("write.step.schema", "AP214IS");
  return 0;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    fprintf(stderr, "usage: stepctlrec read|write <list> <out> [<dumpdir>]\n");
    return 2;
  }
  // system signals (an access violation) as exceptions, which OCC_CATCH_SIGNALS catches ("EXC" lines)
  OSD::SetSignal(false);
  // STEPControl's controller (statics, protocol and libraries)
  STEPControl_Reader anInit;
  std::string        aMode = argv[1];
  if (aMode == "read")
    return Read(argv[2], argv[3]);
  if (aMode == "write")
    return Write(argv[2], argv[3], argc > 4 ? argv[4] : nullptr);
  return 2;
}
