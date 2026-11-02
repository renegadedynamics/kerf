// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// stepdatarec: records what native OCCT's StepData does, for unit 8B (tools/stepdatarec/build.ps1):
//
//   stepdatarec rec <list> <out>      list: one "name<TAB>path" per line (the corpus's STEP files). Each file is read
//                                     as StepFile_Read does (OCCT's own scanner, parser and StepFile_ReadData, compiled
//                                     here from source as tools/steprec does, then StepData_StepReaderData,
//                                     StepData_StepReaderTool and a StepData_StepModel) with a protocol that knows only
//                                     the header (a StepData_FileProtocol of HeaderSection_Protocol): data entities are
//                                     StepData_UndefinedEntity. Dumped: the header entities, every entity (label,
//                                     type, parameters), the checks, every text parameter decoded by ReadString
//                                     (cleanText), then the model written back by StepData_StepWriter.
//   stepdatarec text <cases> <out>    cases: quoted STEP texts as hex bytes, one per line; each is decoded by
//                                     StepData_StepReaderData::ReadString under each of the 28 code pages.
//   stepdatarec tables <out>          Resource_Unicode's code tables (the parts OCCT's conversions reach), binary.
//
// The dumps' texts are written byte for byte but for '\' (as "\\") and the bytes below 0x20 and 0x7F (as \xHH).
#include <windows.h>
#include <bcrypt.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <HeaderSection_FileDescription.hxx>
#include <HeaderSection_FileName.hxx>
#include <HeaderSection_FileSchema.hxx>
#include <HeaderSection_Protocol.hxx>
#include <HeaderSection.hxx>
#include <Interface_Check.hxx>
#include <Interface_EntityIterator.hxx>
#include <Interface_UndefinedContent.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <NCollection_HArray1.hxx>
#include <RWHeaderSection.hxx>
#include <Resource_FormatType.hxx>
#include <Standard_Failure.hxx>
#include <StepData_FileProtocol.hxx>
#include <StepData_FileRecognizer.hxx>
#include <StepData_StepModel.hxx>
#include <StepData_StepReaderData.hxx>
#include <StepData_StepReaderTool.hxx>
#include <StepData_StepWriter.hxx>
#include <StepData_UndefinedEntity.hxx>
#include <StepFile_ReadData.hxx>
#include <TCollection_HAsciiString.hxx>

#include "step.tab.hxx"

typedef unsigned short char16;
#include <Resource_Big5.pxx>
#include <Resource_CodePages.pxx>
#include <Resource_GB2312.pxx>
#include <Resource_GBK.pxx>
#include <Resource_Shiftjis.pxx>

// Declared by step.tab.cxx, defined by StepFile_Read.cxx (not compiled here): the parser's trace of its errors
void StepFile_Interrupt(const char*, const bool) {}

static std::string Esc(const char* theText)
{
  if (theText == nullptr)
    return "(null)";
  std::string aRes;
  for (const unsigned char* p = (const unsigned char*)theText; *p != 0; ++p)
  {
    if (*p == '\\')
      aRes += "\\\\";
    else if (*p < 0x20 || *p == 0x7F)
    {
      char aBuf[8];
      sprintf(aBuf, "\\x%02X", *p);
      aRes += aBuf;
    }
    else
      aRes += (char)*p;
  }
  return aRes;
}

static std::string Str(const occ::handle<TCollection_HAsciiString>& theStr)
{
  return theStr.IsNull() ? std::string("(null)") : "'" + Esc(theStr->ToCString()) + "'";
}

static std::string Strs(const occ::handle<NCollection_HArray1<occ::handle<TCollection_HAsciiString>>>& theStrs)
{
  if (theStrs.IsNull())
    return "(null)";
  std::string aRes = "(";
  for (int i = theStrs->Lower(); i <= theStrs->Upper(); i++)
    aRes += (i > theStrs->Lower() ? "," : "") + Str(theStrs->Value(i));
  return aRes + ")";
}

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

// An undefined entity: TYPE(params) [& NEXT(...)], references as the model's numbers
static std::string Und(const occ::handle<StepData_UndefinedEntity>& theUnd,
                       const occ::handle<StepData_StepModel>&       theModel)
{
  std::string                             aRes  = Esc(theUnd->StepType()) + "(";
  occ::handle<Interface_UndefinedContent> aCont = theUnd->UndefinedContent();
  for (int i = 1; i <= aCont->NbParams(); i++)
  {
    if (i > 1)
      aRes += ",";
    Interface_ParamType aType = aCont->ParamType(i);
    aRes += std::to_string((int)aType) + ":";
    if (aType == Interface_ParamSub)
      aRes += Und(occ::down_cast<StepData_UndefinedEntity>(aCont->ParamEntity(i)), theModel);
    else if (aCont->IsParamEntity(i))
      aRes += "#" + std::to_string(theModel->Number(aCont->ParamEntity(i)));
    else
      aRes += Esc(aCont->ParamValue(i).IsNull() ? nullptr : aCont->ParamValue(i)->ToCString());
  }
  aRes += ")";
  if (!theUnd->Next().IsNull())
    aRes += " & " + Und(theUnd->Next(), theModel);
  return aRes;
}

static std::string Entity(const occ::handle<Standard_Transient>& theEnt, const occ::handle<StepData_StepModel>& theModel)
{
  if (theEnt.IsNull())
    return "(null)";
  if (occ::handle<StepData_UndefinedEntity> anUnd = occ::down_cast<StepData_UndefinedEntity>(theEnt))
    return "U " + Und(anUnd, theModel);
  if (occ::handle<HeaderSection_FileName> aFn = occ::down_cast<HeaderSection_FileName>(theEnt))
    return "FILE_NAME " + Str(aFn->Name()) + " " + Str(aFn->TimeStamp()) + " " + Strs(aFn->Author()) + " "
           + Strs(aFn->Organization()) + " " + Str(aFn->PreprocessorVersion()) + " "
           + Str(aFn->OriginatingSystem()) + " " + Str(aFn->Authorisation());
  if (occ::handle<HeaderSection_FileDescription> aFd = occ::down_cast<HeaderSection_FileDescription>(theEnt))
    return "FILE_DESCRIPTION " + Strs(aFd->Description()) + " " + Str(aFd->ImplementationLevel());
  if (occ::handle<HeaderSection_FileSchema> aFs = occ::down_cast<HeaderSection_FileSchema>(theEnt))
    return "FILE_SCHEMA " + Strs(aFs->SchemaIdentifiers());
  return std::string("? ") + theEnt->DynamicType()->Name();
}

static void Checks(std::vector<std::string>& theLines, const char* thePrefix, const occ::handle<Interface_Check>& theCheck)
{
  if (theCheck.IsNull())
    return;
  for (int i = 1; i <= theCheck->NbFails(); i++)
    theLines.push_back(std::string(thePrefix) + " F " + Esc(theCheck->CFail(i, true)));
  for (int i = 1; i <= theCheck->NbWarnings(); i++)
    theLines.push_back(std::string(thePrefix) + " W " + Esc(theCheck->CWarning(i, true)));
}

static occ::handle<StepData_Protocol> HeaderOnlyProtocol()
{
  static occ::handle<StepData_FileProtocol> aProto;
  if (aProto.IsNull())
  {
    RWHeaderSection::Init();
    aProto = new StepData_FileProtocol;
    aProto->Add(HeaderSection::Protocol());
  }
  return aProto;
}

// StepFile_Read's work, keeping the reader data to decode its texts; theRead and theWrite receive the dumps
static void ReadAndWrite(std::istream&             theStream,
                         std::vector<std::string>& theRead,
                         std::vector<std::string>& theWrite)
{
  occ::handle<StepData_Protocol>  aProtocol = HeaderOnlyProtocol();
  occ::handle<StepData_StepModel> aModel    = new StepData_StepModel;
  StepFile_ReadData               aFileDataModel;
  try
  {
    step::scanner aScanner(&aFileDataModel, &theStream);
    aScanner.yyrestart(&theStream);
    step::parser aParser(&aScanner);
    if (aParser.parse() != 0)
    {
      theRead.push_back("RESULT 1");
      return;
    }
  }
  catch (const Standard_Failure&)
  {
    theRead.push_back("RESULT FATAL");
    return;
  }
  theRead.push_back("RESULT 0");
  int nbhead, nbrec, nbpar;
  aFileDataModel.GetFileNbR(&nbhead, &nbrec, &nbpar);
  occ::handle<StepData_StepReaderData> undirec =
    new StepData_StepReaderData(nbhead, nbrec, nbpar, aModel->SourceCodePage());
  for (int nr = 1; nr <= nbrec; nr++)
  {
    int   nbarg;
    char* ident;
    char* typrec = nullptr;
    aFileDataModel.GetRecordDescription(&ident, &typrec, &nbarg);
    undirec->SetRecord(nr, ident, typrec, nbarg);
    if (nbarg > 0)
    {
      Interface_ParamType typa;
      char*               val;
      while (aFileDataModel.GetArgDescription(&typa, &val) == 1)
        undirec->AddStepParam(nr, val, typa);
    }
    undirec->InitParams(nr);
    aFileDataModel.NextRecord();
  }
  aFileDataModel.ErrorHandle(undirec->GlobalCheck());
  aFileDataModel.ClearRecorder(1);
  StepData_StepReaderTool readtool(undirec, aProtocol);
  readtool.SetErrorHandle(true);
  readtool.PrepareHeader(occ::handle<StepData_FileRecognizer>());
  readtool.Prepare(occ::handle<StepData_FileRecognizer>());
  readtool.LoadModel(aModel);
  if (aModel->Protocol().IsNull())
    aModel->SetProtocol(aProtocol);

  Interface_EntityIterator aHeader = aModel->Header();
  theRead.push_back("HEADER " + std::to_string(aHeader.NbEntities()));
  for (aHeader.Start(); aHeader.More(); aHeader.Next())
    theRead.push_back("H " + Entity(aHeader.Value(), aModel));
  int nb = aModel->NbEntities();
  theRead.push_back("NB " + std::to_string(nb));
  for (int i = 1; i <= nb; i++)
  {
    theRead.push_back("E " + std::to_string(i) + " #" + std::to_string(aModel->IdentLabel(aModel->Value(i))) + " "
                      + Entity(aModel->Value(i), aModel));
    if (aModel->IsReportEntity(i))
      Checks(theRead, ("C " + std::to_string(i)).c_str(), aModel->Check(i, true));
  }
  Checks(theRead, "G", aModel->GlobalCheck());

  // Every text parameter, decoded (cleanText through ReadString)
  for (int num = 1; num <= undirec->NbRecords(); num++)
  {
    for (int nump = 1; nump <= undirec->NbParams(num); nump++)
    {
      if (undirec->ParamType(num, nump) != Interface_ParamText)
        continue;
      occ::handle<Interface_Check>          ach = new Interface_Check;
      occ::handle<TCollection_HAsciiString> aVal;
      try
      {
        undirec->ReadString(num, nump, "t", ach, aVal);
        theRead.push_back("T " + std::to_string(num) + " " + std::to_string(nump) + " " + Str(aVal));
      }
      catch (const Standard_Failure& anEx)
      {
        theRead.push_back("T " + std::to_string(num) + " " + std::to_string(nump) + " THROW "
                          + Esc(anEx.GetMessageString()) + " " + Esc(undirec->ParamCValue(num, nump)));
      }
    }
  }
  Checks(theRead, "GT", undirec->GlobalCheck());
  aFileDataModel.ClearRecorder(2); // after the texts: the parameters point into the parse's text pages
  readtool.Clear();

  // The model written back
  StepData_StepWriter aWriter(aModel);
  aWriter.SendModel(aProtocol);
  for (int i = 1; i <= aWriter.NbLines(); i++)
    theWrite.push_back(aWriter.Line(i)->ToCString());
  Interface_CheckIterator aChecks = aWriter.CheckList();
  for (aChecks.Start(); aChecks.More(); aChecks.Next())
    Checks(theWrite, ("WC " + std::to_string(aChecks.Number())).c_str(), aChecks.Value());
}

static void Lines(FILE* theOut, const char* theTag, const std::vector<std::string>& theLines, size_t theKeep)
{
  std::string aAll;
  for (const std::string& aLine : theLines)
    aAll += aLine + "\n";
  fprintf(theOut, "%s %zu %s\n", theTag, theLines.size(), Sha256(aAll).c_str());
  // The first lines, and every check and exception line
  for (size_t i = 0; i < theLines.size(); i++)
  {
    const std::string& aLine = theLines[i];
    if (i < theKeep || aLine.compare(0, 2, "G ") == 0 || aLine.compare(0, 3, "GT ") == 0
        || aLine.compare(0, 2, "C ") == 0 || aLine.compare(0, 3, "WC ") == 0 || aLine.find(" THROW ") != std::string::npos
        || aLine.compare(0, 10, "EXCEPTION ") == 0)
      fprintf(theOut, "%s\n", aLine.c_str());
  }
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

static std::string FromHex(const std::string& theHex)
{
  std::string aRes;
  for (size_t i = 0; i + 1 < theHex.size(); i += 2)
    aRes += (char)std::stoi(theHex.substr(i, 2), nullptr, 16);
  return aRes;
}

// The exception's class name (Standard_Failure is not a transient in OCCT 8)
static std::string ExName(const Standard_Failure& theEx)
{
  return theEx.ExceptionType();
}

static void Put32(FILE* theOut, unsigned int theValue)
{
  fwrite(&theValue, 4, 1, theOut);
}

static void Table(FILE* theOut, const std::vector<std::pair<unsigned int, unsigned int>>& theEntries)
{
  Put32(theOut, (unsigned int)theEntries.size());
  for (const auto& anEntry : theEntries)
  {
    Put32(theOut, anEntry.first);
    Put32(theOut, anEntry.second);
  }
}

#define issjis1(c) (((c) >= 0x81 && (c) <= 0x9f) || ((c) >= 0xe0 && (c) <= 0xef))
#define issjis2(c) ((c) >= 0x40 && (c) <= 0xfc && (c) != 0x7f)

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    fprintf(stderr, "stepdatarec rec <list> <out> | text <cases> <out> | tables <out>\n");
    return 2;
  }
  // The messages go nowhere (they are OCCT's traces, not compared)
  Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
  std::string aMode = argv[1];
  if (aMode == "tables")
  {
    FILE* anOut = fopen(argv[2], "wb");
    std::vector<std::pair<unsigned int, unsigned int>> anEntries;
    for (unsigned int aPage = 0; aPage <= Resource_FormatType_CP850 - Resource_FormatType_CP1250; aPage++)
      for (unsigned int i = 0; i < 128; i++)
        anEntries.emplace_back(aPage * 128 + i, THE_CODEPAGES_ANSI[aPage][i]);
    Table(anOut, anEntries);
    anEntries.clear();
    for (unsigned int i = 0; i < 65536; i++)
      if (issjis1(i >> 8) && issjis2(i & 0xFF))
        anEntries.emplace_back(i, sjisuni[i]);
    Table(anOut, anEntries);
    anEntries.clear();
    for (unsigned int h = 0; h < 128; h++)
      for (unsigned int l = 0; l < 128; l++)
        anEntries.emplace_back((h << 8) | l, gbuni[(h << 8) | l]);
    Table(anOut, anEntries);
    anEntries.clear();
    for (unsigned int i = 0; i < 23940; i++)
      anEntries.emplace_back(i, gbkuni[i]);
    Table(anOut, anEntries);
    anEntries.clear();
    for (unsigned int i = 0; i < 19782; i++)
      anEntries.emplace_back(i, big5uni[i]);
    Table(anOut, anEntries);
    fclose(anOut);
    return 0;
  }
  if (argc < 4)
    return 2;
  FILE* anOut = fopen(argv[3], "wb");
  if (anOut == nullptr)
    return 3;
  if (aMode == "rec")
  {
    for (const auto& anInput : ReadList(argv[2]))
    {
      std::ifstream aFile(anInput.second, std::ios::in | std::ios::binary);
      aFile.seekg(0, std::ios::end);
      size_t aSize = (size_t)aFile.tellg();
      aFile.seekg(0, std::ios::beg);
      std::vector<std::string> aRead, aWrite;
      try
      {
        ReadAndWrite(aFile, aRead, aWrite);
      }
      catch (const Standard_Failure& anEx)
      {
        aRead.push_back(std::string("EXCEPTION ") + Esc(anEx.GetMessageString()));
      }
      fprintf(anOut, "FILE %s %zu\n", anInput.first.c_str(), aSize);
      Lines(anOut, "READ", aRead, 40);
      Lines(anOut, "WRITE", aWrite, 40);
      fprintf(anOut, "END\n");
      fflush(anOut);
    }
  }
  else if (aMode == "text")
  {
    std::ifstream aCases(argv[2]);
    std::string   aLine;
    int           aCase = 0;
    while (std::getline(aCases, aLine))
    {
      if (!aLine.empty() && aLine.back() == '\r')
        aLine.pop_back();
      std::string aText = FromHex(aLine);
      ++aCase;
      for (int aPage = 0; aPage <= (int)Resource_FormatType_Big5; aPage++)
      {
        std::string aRes;
        try
        {
          occ::handle<StepData_StepReaderData> aData =
            new StepData_StepReaderData(0, 1, 1, (Resource_FormatType)aPage);
          aData->SetRecord(1, "#1", "X", 1);
          aData->AddStepParam(1, aText.c_str(), Interface_ParamText);
          aData->InitParams(1);
          occ::handle<Interface_Check>          ach = new Interface_Check;
          occ::handle<TCollection_HAsciiString> aVal;
          aData->ReadString(1, 1, "t", ach, aVal);
          aRes = Str(aVal);
          for (int i = 1; i <= aData->GlobalCheck()->NbWarnings(); i++)
            aRes += " W " + Esc(aData->GlobalCheck()->CWarning(i, true));
        }
        catch (const Standard_Failure& anEx)
        {
          aRes = std::string("THROW ") + ExName(anEx) + " " + Esc(anEx.GetMessageString());
        }
        fprintf(anOut, "%d %d %s\n", aCase, aPage, aRes.c_str());
      }
    }
  }
  fclose(anOut);
  return 0;
}
