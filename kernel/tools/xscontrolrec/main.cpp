// xscontrolrec: the native recorder of unit 8A3 (IFSelect and XSControl's work session on STEP files).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
//   xscontrolrec <inputs.txt> <out.txt>
// inputs.txt: lines "key<TAB>path". For each file: read by STEPControl_Reader (its work session, "lite" mode), then
// again in "full" mode (SetModeStat), and dump what IFSelect computes on the model: the status, the count, the roots
// (a selection), the type and validity signatures (counters), the model's check list (each check's number, counts and
// first message), a label looked up, a selection by signature; in full mode the roots,
// the shareds of the roots, the sharings of entity 1 and the categories. OcctNet's XSControlNativeTests writes the same
// lines (tests/OcctNet.DataExchange.Tests/XSControl/XSControlNativeTests.cs).

#include <STEPControl_Reader.hxx>
#include <XSControl_WorkSession.hxx>
#include <Interface_InterfaceModel.hxx>
#include <Interface_CheckIterator.hxx>
#include <Interface_Check.hxx>
#include <IFSelect_SelectModelRoots.hxx>
#include <IFSelect_SelectShared.hxx>
#include <IFSelect_SelectSharing.hxx>
#include <IFSelect_SignType.hxx>
#include <IFSelect_SignValidity.hxx>
#include <IFSelect_SignCategory.hxx>
#include <IFSelect_SignCounter.hxx>
#include <IFSelect_CheckCounter.hxx>
#include <TCollection_HAsciiString.hxx>
#include <NCollection_HSequence.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

typedef occ::handle<NCollection_HSequence<occ::handle<Standard_Transient>>> ListHandle;

static void DumpList(std::ostream& O, const char* theTag, const occ::handle<Interface_InterfaceModel>& M,
                     const ListHandle& L)
{
  if (L.IsNull())
  {
    O << theTag << " null\n";
    return;
  }
  long long aSum = 0;
  std::ostringstream aFirst;
  for (int i = 1; i <= L->Length(); i++)
  {
    int n = M->Number(L->Value(i));
    aSum  = (aSum + (long long)i * n) % 1000000007LL;
    if (i <= 20)
    {
      aFirst << ' ' << n;
    }
  }
  O << theTag << ' ' << L->Length() << ' ' << aSum << aFirst.str() << '\n';
}

static void DumpCounter(std::ostream& O, const char* theTag, const occ::handle<IFSelect_SignatureList>& C)
{
  occ::handle<NCollection_HSequence<occ::handle<TCollection_HAsciiString>>> aList = C->List();
  O << theTag << ' ' << aList->Length() << ' ' << C->NbNulls() << '\n';
  for (int i = 1; i <= aList->Length(); i++)
  {
    O << "  " << C->NbTimes(aList->Value(i)->ToCString()) << ' ' << aList->Value(i)->ToCString() << '\n';
  }
}

static void Record(std::ostream& O, const std::string& thePath, bool theFull)
{
  STEPControl_Reader aReader;
  occ::handle<XSControl_WorkSession> WS = aReader.WS();
  WS->SetModeStat(theFull);
  IFSelect_ReturnStatus aStat = aReader.ReadFile(thePath.c_str());
  occ::handle<Interface_InterfaceModel> M = WS->Model();
  int nb = M.IsNull() ? 0 : M->NbEntities();
  O << (theFull ? "full" : "lite") << " st " << (int)aStat << " n " << nb << '\n';
  if (aStat != IFSelect_RetDone || nb == 0)
  {
    return;
  }
  WS->AddNamedItem("rec-roots", new IFSelect_SelectModelRoots);
  WS->AddNamedItem("rec-shared", new IFSelect_SelectShared);
  WS->AddNamedItem("rec-sharing", new IFSelect_SelectSharing);
  WS->AddNamedItem("rec-type", new IFSelect_SignType(true));
  if (theFull)
  {
    DumpList(O, "roots", M, WS->GiveList("rec-roots"));
    DumpList(O, "shared", M, WS->GiveList("rec-shared rec-roots"));
    DumpList(O, "sharing1", M, WS->GiveList("rec-sharing", "1"));
    occ::handle<IFSelect_SignCounter> aCat = new IFSelect_SignCounter(new IFSelect_SignCategory, false, false);
    aCat->AddModel(M);
    DumpCounter(O, "categories", aCat);
    return;
  }
  DumpList(O, "roots", M, WS->GiveList("rec-roots"));
  occ::handle<IFSelect_SignCounter> aTypes = new IFSelect_SignCounter(new IFSelect_SignType(false), false, false);
  aTypes->AddModel(M);
  DumpCounter(O, "types", aTypes);
  occ::handle<IFSelect_SignCounter> aValid = new IFSelect_SignCounter(new IFSelect_SignValidity, false, false);
  aValid->AddModel(M);
  DumpCounter(O, "validity", aValid);
  Interface_CheckIterator aChl = WS->ModelCheckList();
  int aNbItems = 0, aNbFails = 0, aNbWarns = 0;
  for (aChl.Start(); aChl.More(); aChl.Next())
  {
    aNbItems++;
    aNbFails += aChl.Value()->NbFails();
    aNbWarns += aChl.Value()->NbWarnings();
  }
  O << "checks " << aNbItems << ' ' << aNbFails << ' ' << aNbWarns << '\n';
  // each check (IFSelect_CheckCounter with a signature reads a destroyed string natively: its type texts are
  // garbage, occt-issues.md, 8A3): its number, counts and first message
  for (aChl.Start(); aChl.More(); aChl.Next())
  {
    const occ::handle<Interface_Check>& aCh = aChl.Value();
    O << "  " << aChl.Number() << ' ' << aCh->NbFails() << ' ' << aCh->NbWarnings() << ' '
      << (aCh->NbFails() > 0 ? aCh->CFail(1) : aCh->CWarning(1)) << '\n';
  }
  int aMid = (nb + 1) / 2;
  occ::handle<TCollection_HAsciiString> aLab = M->StringLabel(M->Value(aMid));
  O << "label " << aMid << ' ' << (aLab.IsNull() ? "" : aLab->ToCString()) << ' '
    << (aLab.IsNull() ? 0 : WS->NumberFromLabel(aLab->ToCString())) << '\n';
  ListHandle aCp = WS->GiveList("rec-type(CartesianPoint)");
  O << "sel-cp " << (aCp.IsNull() ? -1 : aCp->Length()) << '\n';
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "usage: xscontrolrec <inputs.txt> <out.txt>\n";
    return 2;
  }
  // silence the messages (the reader's traces)
  Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_Printer));
  std::ifstream anIn(argv[1]);
  std::ofstream anOut(argv[2], std::ios::binary);
  std::string aLine;
  while (std::getline(anIn, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
    {
      aLine.pop_back();
    }
    size_t aTab = aLine.find('\t');
    if (aTab == std::string::npos)
    {
      continue;
    }
    std::string aKey = aLine.substr(0, aTab), aPath = aLine.substr(aTab + 1);
    anOut << "F " << aKey << '\n';
    try
    {
      Record(anOut, aPath, false);
      Record(anOut, aPath, true);
    }
    catch (Standard_Failure& anExc)
    {
      anOut << "exception " << anExc.what() << '\n';
    }
    anOut.flush();
  }
  return 0;
}
