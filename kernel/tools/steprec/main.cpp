// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// steprec: records what native OCCT's STEP parser (StepFile: the flex scanner lex.step.cxx, the bison parser
// step.tab.cxx and StepFile_ReadData.cxx, compiled here from OCCT V8_0_1's source because TKDESTEP doesn't export
// them) leaves in StepFile_ReadData, for unit 8P's C# parser (tools/steprec/build.ps1).
//
// Per input, the dump that tests/OcctNet.DataExchange.Tests/StepFile/StepFileDump.cs also writes:
//   RESULT <r>                  the parser's return (0 accepted, 1 aborted), or FATAL when the scanner threw
//   FATAL <message>             the Standard_Failure's message (flex's YY_FATAL_ERROR)
//   NB <nbhead> <nbrec> <nbpar> GetFileNbR
//   R <ident> <type> <nbarg>    per record, as StepFile_Read walks them (GetRecordDescription, NextRecord);
//                               "R (none)" ends the walk where the chain of records ends before nbrec
//   A <type> <value>            its arguments (GetArgDescription): Interface_ParamType's number and the text
//   ERR <message>               the errors ErrorHandle hands to an Interface_Check
// Texts are written byte for byte but for '\' (as "\\") and the bytes below 0x20 and 0x7F (as \xHH); a null text is
// "(null)".
//
// The recording, per input:
//   FILE <name> <bytes>
//   DUMP <lines> <sha256 of the dump's bytes, lines ended by \n>
//   then the whole dump when it has at most 400 lines, else its first 60 lines and its ERR lines, then
//   END
//
//   steprec rec <list> <out>         list: one "name<TAB>path" per line
//   steprec pack <pack> <out>        pack: entries "<name>\n<length>\n<bytes>" (the hostile and mutated inputs)
//   steprec bench <list> <out> <reps> per input the fastest of reps parses from memory, in ms
#include <windows.h>
#include <bcrypt.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Interface_Check.hxx>
#include <Standard_Failure.hxx>
#include <StepFile_ReadData.hxx>

#include "step.tab.hxx"

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

static std::vector<std::string> Dump(std::istream& theStream)
{
  std::vector<std::string> aLines;
  StepFile_ReadData        aData;
  std::string              aResult, aFatal;
  try
  {
    step::scanner aScanner(&aData, &theStream);
    aScanner.yyrestart(&theStream);
    step::parser aParser(&aScanner);
    aResult = std::to_string(aParser.parse());
  }
  catch (const Standard_Failure& anEx)
  {
    aResult = "FATAL";
    aFatal  = anEx.GetMessageString();
  }
  catch (...)
  {
    aResult = "CRASH";
  }
  aLines.push_back("RESULT " + aResult);
  if (aResult == "FATAL")
    aLines.push_back("FATAL " + Esc(aFatal.c_str()));
  if (aResult == "CRASH")
    return aLines;
  int nbhead, nbrec, nbpar;
  aData.GetFileNbR(&nbhead, &nbrec, &nbpar);
  aLines.push_back("NB " + std::to_string(nbhead) + " " + std::to_string(nbrec) + " " + std::to_string(nbpar));
  for (int nr = 1; nr <= nbrec; nr++)
  {
    int   nbarg = 0;
    char* ident = nullptr;
    char* typrec = nullptr;
    if (!aData.GetRecordDescription(&ident, &typrec, &nbarg))
    {
      // The chain ends before nbrec (StepFile_Read would read through a null record)
      aLines.push_back("R (none)");
      break;
    }
    aLines.push_back("R " + Esc(ident) + " " + Esc(typrec) + " " + std::to_string(nbarg));
    if (nbarg > 0)
    {
      Interface_ParamType typa;
      char*               val;
      while (aData.GetArgDescription(&typa, &val))
        aLines.push_back("A " + std::to_string((int)typa) + " " + Esc(val));
    }
    aData.NextRecord();
  }
  occ::handle<Interface_Check> aCheck = new Interface_Check();
  aData.ErrorHandle(aCheck);
  for (int i = 1; i <= aCheck->NbFails(); i++)
    aLines.push_back("ERR " + Esc(aCheck->CFail(i, true)));
  return aLines;
}

static std::string Sha256(const std::string& theData)
{
  BCRYPT_ALG_HANDLE  anAlg  = nullptr;
  BCRYPT_HASH_HANDLE aHash  = nullptr;
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

static void Record(FILE* theOut, const std::string& theName, size_t theSize, std::istream& theStream)
{
  std::vector<std::string> aLines = Dump(theStream);
  std::string              aAll;
  for (const std::string& aLine : aLines)
    aAll += aLine + "\n";
  fprintf(theOut, "FILE %s %zu\nDUMP %zu %s\n", theName.c_str(), theSize, aLines.size(), Sha256(aAll).c_str());
  for (size_t i = 0; i < aLines.size(); i++)
  {
    if (aLines.size() <= 400 || i < 60 || aLines[i].compare(0, 4, "ERR ") == 0)
      fprintf(theOut, "%s\n", aLines[i].c_str());
  }
  fprintf(theOut, "END\n");
}

static bool ReadAll(const std::string& thePath, std::string& theData)
{
  std::ifstream aFile(thePath, std::ios::in | std::ios::binary);
  if (!aFile)
    return false;
  std::ostringstream aBuf;
  aBuf << aFile.rdbuf();
  theData = aBuf.str();
  return true;
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

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    fprintf(stderr, "steprec rec|pack|bench <in> <out> [reps]\n");
    return 2;
  }
  std::string aMode = argv[1];
  FILE*       anOut = fopen(argv[3], "wb");
  if (anOut == nullptr)
    return 3;
  if (aMode == "rec")
  {
    for (const auto& anInput : ReadList(argv[2]))
    {
      // Through an ifstream, as StepFile_Read's OSD_FileSystem stream
      std::ifstream aFile(anInput.second, std::ios::in | std::ios::binary);
      aFile.seekg(0, std::ios::end);
      size_t aSize = (size_t)aFile.tellg();
      aFile.seekg(0, std::ios::beg);
      Record(anOut, anInput.first, aSize, aFile);
    }
  }
  else if (aMode == "pack")
  {
    std::string aPack;
    if (!ReadAll(argv[2], aPack))
      return 4;
    size_t aPos = 0;
    while (aPos < aPack.size())
    {
      size_t      anEol  = aPack.find('\n', aPos);
      std::string aName  = aPack.substr(aPos, anEol - aPos);
      size_t      anEol2 = aPack.find('\n', anEol + 1);
      size_t      aLen   = std::stoull(aPack.substr(anEol + 1, anEol2 - anEol - 1));
      std::istringstream aStream(aPack.substr(anEol2 + 1, aLen), std::ios::in | std::ios::binary);
      Record(anOut, aName, aLen, aStream);
      aPos = anEol2 + 1 + aLen;
    }
  }
  else if (aMode == "bench")
  {
    int aReps = argc > 4 ? atoi(argv[4]) : 5;
    for (const auto& anInput : ReadList(argv[2]))
    {
      std::string aData;
      if (!ReadAll(anInput.second, aData))
        continue;
      double aBest = 1e30;
      for (int r = 0; r < aReps; r++)
      {
        auto               aStart = std::chrono::steady_clock::now();
        std::istringstream aStream(aData, std::ios::in | std::ios::binary);
        StepFile_ReadData  aRead;
        try
        {
          step::scanner aScanner(&aRead, &aStream);
          aScanner.yyrestart(&aStream);
          step::parser aParser(&aScanner);
          aParser.parse();
        }
        catch (const Standard_Failure&)
        {
        }
        double aMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aStart).count();
        if (aMs < aBest)
          aBest = aMs;
      }
      fprintf(anOut, "%s %zu %.3f\n", anInput.first.c_str(), aData.size(), aBest);
    }
  }
  fclose(anOut);
  return 0;
}
