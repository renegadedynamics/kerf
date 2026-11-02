// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// toporec: records what native OCCT's StepToTopoDS and TopoDSToStep do, for unit 8E (tools/toporec/build.ps1):
//
//   toporec s2t <list> <out> [<dumpdir>]
//       list: one "name<TAB>path" per line (the corpus's STEP files). Each file is read with STEPControl_Reader (the
//       AP214 protocol), its model's parameters reset to DESTEP_Parameters' defaults, and every shape representation
//       item of the model (manifold solid breps, breps with voids, faceted breps (with voids), shell based surface
//       models, edge based wireframe models, face based surface models, geometric sets, tessellated solids, shells,
//       faces and surface sets) is translated by StepToTopoDS_Builder with the default factors, on a transient
//       process of its own. Per item: "I <entity number> <type> <result>", the result being "NOTDONE", "EXC" or
//       "<sha256 of the BRep text> <its length>", then " W<warnings> F<fails>" (the transient process's checks).
//       With a dump directory, each BRep text is written there too ("<file>.<entity number>.brep").
//   toporec t2s <list> <out> [<dumpdir>]
//       list: "name<TAB>path" of BRep files. Each shape is converted by TopoDSToStep: each solid by
//       MakeManifoldSolidBrep, MakeBrepWithVoids (when it has several shells), MakeShellBasedSurfaceModel and
//       MakeFacetedBrep (when TopoDSToStep_FacetedTool accepts it); each shell by MakeManifoldSolidBrep,
//       MakeShellBasedSurfaceModel and MakeFacetedBrep (when closed and accepted); the first 20 faces by
//       MakeShellBasedSurfaceModel; the whole shape by MakeGeometricCurveSet; each with the default factors and with
//       inches and degrees, on a finder process of its own (a StepData_StepModel with the AP214 protocol and
//       DESTEP_Parameters' defaults). Each result is added with its references to a model of its own and written
//       with StepData_StepWriter. Per conversion: "C <op> <index> <factors> <result>", the result "NOTDONE", "EXC" or
//       "<entities> <sha256 of the lines> <first line>"; with a dump directory, the lines too.
//
// The port's replay (tests/OcctNet.DataExchange.Tests/TopoStep/TopoStepNativeTests.cs) computes the same lines.
#include <windows.h>
#include <bcrypt.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <DESTEP_Parameters.hxx>
#include <Interface_Check.hxx>
#include <Interface_CheckIterator.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StepAP214.hxx>
#include <StepAP214_Protocol.hxx>
#include <StepData_Factors.hxx>
#include <StepData_StepModel.hxx>
#include <StepData_StepWriter.hxx>
#include <StepData_WriterLib.hxx>
#include <StepShape_BrepWithVoids.hxx>
#include <StepShape_EdgeBasedWireframeModel.hxx>
#include <StepShape_FaceBasedSurfaceModel.hxx>
#include <StepShape_FacetedBrep.hxx>
#include <StepShape_FacetedBrepAndBrepWithVoids.hxx>
#include <StepShape_GeometricCurveSet.hxx>
#include <StepShape_GeometricSet.hxx>
#include <StepShape_ManifoldSolidBrep.hxx>
#include <StepShape_ShellBasedSurfaceModel.hxx>
#include <StepToTopoDS_Builder.hxx>
#include <StepToTopoDS_NMTool.hxx>
#include <StepVisual_TessellatedFace.hxx>
#include <StepVisual_TessellatedShell.hxx>
#include <StepVisual_TessellatedSolid.hxx>
#include <StepVisual_TessellatedSurfaceSet.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDSToStep_FacetedTool.hxx>
#include <TopoDSToStep_MakeBrepWithVoids.hxx>
#include <TopoDSToStep_MakeFacetedBrep.hxx>
#include <TopoDSToStep_MakeGeometricCurveSet.hxx>
#include <TopoDSToStep_MakeManifoldSolidBrep.hxx>
#include <TopoDSToStep_MakeShellBasedSurfaceModel.hxx>
#include <Transfer_ActorOfTransientProcess.hxx>
#include <Transfer_FinderProcess.hxx>
#include <Transfer_TransientProcess.hxx>

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
    if (c == '/' || c == '\\' || c == ':')
      c = '_';
  return theName;
}

static occ::handle<StepAP214_Protocol> theProtocol;

// ---------------------------------------------------------------------------------------------------------------
// StepToTopoDS

static std::string Checks(const occ::handle<Transfer_TransientProcess>& theTP)
{
  int                     aNbW = 0, aNbF = 0;
  Interface_CheckIterator aList = theTP->CheckList(false);
  for (aList.Start(); aList.More(); aList.Next())
  {
    aNbW += aList.Value()->NbWarnings();
    aNbF += aList.Value()->NbFails();
  }
  return " W" + std::to_string(aNbW) + " F" + std::to_string(aNbF);
}

// The item's translation: "NOTDONE", "EXC" or the BRep text's hash and length; the text in theText
static std::string Translate(const occ::handle<Standard_Transient>& theEnt,
                             const occ::handle<Transfer_TransientProcess>& theTP,
                             std::string&                                  theText)
{
  StepToTopoDS_Builder aBuilder;
  StepData_Factors     aFactors;
  bool                 isTried = true;
  bool                 aHasGeom = false;
  if (theEnt->IsKind(STANDARD_TYPE(StepShape_FacetedBrepAndBrepWithVoids)))
    aBuilder.Init(occ::down_cast<StepShape_FacetedBrepAndBrepWithVoids>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_BrepWithVoids)))
    aBuilder.Init(occ::down_cast<StepShape_BrepWithVoids>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_FacetedBrep)))
    aBuilder.Init(occ::down_cast<StepShape_FacetedBrep>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_ManifoldSolidBrep)))
    aBuilder.Init(occ::down_cast<StepShape_ManifoldSolidBrep>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_ShellBasedSurfaceModel)))
  {
    StepToTopoDS_NMTool aNMTool;
    aBuilder.Init(occ::down_cast<StepShape_ShellBasedSurfaceModel>(theEnt), theTP, aNMTool, aFactors);
  }
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_EdgeBasedWireframeModel)))
    aBuilder.Init(occ::down_cast<StepShape_EdgeBasedWireframeModel>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_FaceBasedSurfaceModel)))
    aBuilder.Init(occ::down_cast<StepShape_FaceBasedSurfaceModel>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepShape_GeometricSet)))
    aBuilder.Init(occ::down_cast<StepShape_GeometricSet>(theEnt), theTP, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepVisual_TessellatedSolid)))
    aBuilder.Init(occ::down_cast<StepVisual_TessellatedSolid>(theEnt), theTP, false, aHasGeom, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepVisual_TessellatedShell)))
    aBuilder.Init(occ::down_cast<StepVisual_TessellatedShell>(theEnt), theTP, false, aHasGeom, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepVisual_TessellatedFace)))
    aBuilder.Init(occ::down_cast<StepVisual_TessellatedFace>(theEnt), theTP, false, aHasGeom, aFactors);
  else if (theEnt->IsKind(STANDARD_TYPE(StepVisual_TessellatedSurfaceSet)))
    aBuilder.Init(occ::down_cast<StepVisual_TessellatedSurfaceSet>(theEnt), theTP, aHasGeom, aFactors);
  else
    isTried = false;
  if (!isTried)
    return "";
  if (!aBuilder.IsDone())
    return "NOTDONE";
  std::ostringstream aS;
  BRepTools::Write(aBuilder.Value(), aS);
  theText = aS.str();
  return Sha256(theText) + " " + std::to_string(theText.size());
}

static int S2T(const char* theList, const char* theOut, const char* theDump)
{
  FILE* anOut = fopen(theOut, "wb");
  for (const auto& anInput : ReadList(theList))
  {
    fprintf(anOut, "FILE %s\n", anInput.first.c_str());
    STEPControl_Reader aReader;
    if (aReader.ReadFile(anInput.second.c_str()) != IFSelect_RetDone)
    {
      fprintf(anOut, "READFAIL\n");
      continue;
    }
    occ::handle<StepData_StepModel> aModel = aReader.StepModel();
    aModel->InternalParameters                   = DESTEP_Parameters();
    for (int i = 1; i <= aModel->NbEntities(); i++)
    {
      occ::handle<Standard_Transient>        anEnt = aModel->Value(i);
      occ::handle<Transfer_TransientProcess> aTP   = new Transfer_TransientProcess(aModel->NbEntities());
      aTP->SetModel(aModel);
      std::string aRes, aText;
      try
      {
        OCC_CATCH_SIGNALS
        aRes = Translate(anEnt, aTP, aText);
      }
      catch (Standard_Failure const&)
      {
        aRes = "EXC";
      }
      catch (...)
      {
        aRes = "EXC";
      }
      if (aRes.empty())
        continue;
      fprintf(anOut, "I %d %s %s%s\n", i, anEnt->DynamicType()->Name(), aRes.c_str(), Checks(aTP).c_str());
      if (theDump != nullptr && !aText.empty())
      {
        std::string   aPath = std::string(theDump) + "\\" + SafeName(anInput.first) + "." + std::to_string(i) + ".brep";
        std::ofstream aFile(aPath, std::ios::binary);
        aFile << aText;
      }
    }
    fflush(anOut);
  }
  fclose(anOut);
  return 0;
}

// ---------------------------------------------------------------------------------------------------------------
// TopoDSToStep

static std::vector<std::string> Written(const occ::handle<Standard_Transient>& theEnt)
{
  occ::handle<StepData_StepModel> aModel = new StepData_StepModel;
  aModel->SetProtocol(theProtocol);
  aModel->AddWithRefs(theEnt, theProtocol);
  StepData_StepWriter aWriter(aModel);
  StepData_WriterLib  aLib(theProtocol);
  for (int i = 1; i <= aModel->NbEntities(); i++)
    aWriter.SendEntity(i, aLib);
  std::vector<std::string> aRes;
  for (int i = 1; i <= aWriter.NbLines(); i++)
    aRes.push_back(aWriter.Line(i)->ToCString());
  return aRes;
}

static occ::handle<Transfer_FinderProcess> NewFP()
{
  occ::handle<StepData_StepModel> aModel = new StepData_StepModel;
  aModel->SetProtocol(theProtocol);
  aModel->InternalParameters = DESTEP_Parameters();
  occ::handle<Transfer_FinderProcess> aFP = new Transfer_FinderProcess;
  aFP->SetModel(aModel);
  return aFP;
}

template <class TMaker, class TShape>
static void Convert(FILE*                   theOut,
                    std::ofstream*          theDump,
                    const char*             theOp,
                    int                     theIndex,
                    const char*             theTag,
                    const TShape&           theShape,
                    const StepData_Factors& theFactors)
{
  std::string              aRes;
  std::vector<std::string> aLines;
  try
  {
    OCC_CATCH_SIGNALS
    TMaker aMaker(theShape, NewFP(), theFactors);
    if (!aMaker.IsDone() || aMaker.Value().IsNull())
      aRes = "NOTDONE";
    else
    {
      aLines = Written(aMaker.Value());
      std::string anAll;
      for (const std::string& aLine : aLines)
        anAll += aLine + "\n";
      aRes = std::to_string(aLines.size()) + " " + Sha256(anAll) + " "
             + (aLines.empty() ? std::string() : aLines.front().substr(0, 120));
    }
  }
  catch (Standard_Failure const&)
  {
    aRes = "EXC";
  }
  catch (...)
  {
    aRes = "EXC";
  }
  fprintf(theOut, "C %s %d %s %s\n", theOp, theIndex, theTag, aRes.c_str());
  if (theDump != nullptr)
  {
    *theDump << "C " << theOp << " " << theIndex << " " << theTag << "\n";
    for (const std::string& aLine : aLines)
      *theDump << aLine << "\n";
  }
}

static int T2S(const char* theList, const char* theOut, const char* theDump)
{
  FILE*                  anOut      = fopen(theOut, "wb");
  const StepData_Factors aFactors[2] = {StepData_Factors(), [] {
                                          StepData_Factors F;
                                          F.InitializeFactors(25.4, M_PI / 180.0, 1.0);
                                          return F;
                                        }()};
  const char* const      aTags[2]    = {"mm", "in"};
  for (const auto& anInput : ReadList(theList))
  {
    fprintf(anOut, "FILE %s\n", anInput.first.c_str());
    TopoDS_Shape aShape;
    BRep_Builder aB;
    if (!BRepTools::Read(aShape, anInput.second.c_str(), aB) || aShape.IsNull())
    {
      fprintf(anOut, "READFAIL\n");
      continue;
    }
    std::ofstream  aDumpFile;
    std::ofstream* aDump = nullptr;
    if (theDump != nullptr)
    {
      aDumpFile.open(std::string(theDump) + "\\" + SafeName(anInput.first) + ".txt", std::ios::binary);
      aDump = &aDumpFile;
    }
    for (int f = 0; f < 2; f++)
    {
      const StepData_Factors& F = aFactors[f];
      int                     i = 0;
      for (TopExp_Explorer anExp(aShape, TopAbs_SOLID); anExp.More(); anExp.Next())
      {
        const TopoDS_Solid& aSolid = TopoDS::Solid(anExp.Current());
        ++i;
        int aNbShells = 0;
        for (TopoDS_Iterator anIt(aSolid); anIt.More(); anIt.Next())
          aNbShells += anIt.Value().ShapeType() == TopAbs_SHELL ? 1 : 0;
        Convert<TopoDSToStep_MakeManifoldSolidBrep>(anOut, aDump, "msb-solid", i, aTags[f], aSolid, F);
        if (aNbShells > 1)
          Convert<TopoDSToStep_MakeBrepWithVoids>(anOut, aDump, "bwv-solid", i, aTags[f], aSolid, F);
        Convert<TopoDSToStep_MakeShellBasedSurfaceModel>(anOut, aDump, "sbsm-solid", i, aTags[f], aSolid, F);
        if (TopoDSToStep_FacetedTool::CheckTopoDSShape(aSolid) == TopoDSToStep_FacetedDone)
          Convert<TopoDSToStep_MakeFacetedBrep>(anOut, aDump, "fb-solid", i, aTags[f], aSolid, F);
      }
      i = 0;
      for (TopExp_Explorer anExp(aShape, TopAbs_SHELL); anExp.More(); anExp.Next())
      {
        const TopoDS_Shell& aShell = TopoDS::Shell(anExp.Current());
        ++i;
        Convert<TopoDSToStep_MakeManifoldSolidBrep>(anOut, aDump, "msb-shell", i, aTags[f], aShell, F);
        Convert<TopoDSToStep_MakeShellBasedSurfaceModel>(anOut, aDump, "sbsm-shell", i, aTags[f], aShell, F);
        if (aShell.Closed() && TopoDSToStep_FacetedTool::CheckTopoDSShape(aShell) == TopoDSToStep_FacetedDone)
          Convert<TopoDSToStep_MakeFacetedBrep>(anOut, aDump, "fb-shell", i, aTags[f], aShell, F);
      }
      i = 0;
      for (TopExp_Explorer anExp(aShape, TopAbs_FACE); anExp.More() && i < 20; anExp.Next())
      {
        ++i;
        Convert<TopoDSToStep_MakeShellBasedSurfaceModel>(anOut, aDump, "sbsm-face", i, aTags[f], TopoDS::Face(anExp.Current()), F);
      }
      Convert<TopoDSToStep_MakeGeometricCurveSet>(anOut, aDump, "gcs", 1, aTags[f], aShape, F);
    }
    fflush(anOut);
  }
  fclose(anOut);
  return 0;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    fprintf(stderr, "usage: toporec s2t|t2s <list> <out> [<dumpdir>]\n");
    return 2;
  }
  // STEPControl's controller (its protocol and libraries, Shape Healing's ShapeAlgo container)
  STEPControl_Reader anInit;
  theProtocol           = StepAP214::Protocol();
  std::string aMode     = argv[1];
  const char* aDump     = argc > 4 ? argv[4] : nullptr;
  if (aMode == "s2t")
    return S2T(argv[2], argv[3], aDump);
  if (aMode == "t2s")
    return T2S(argv[2], argv[3], aDump);
  return 2;
}
