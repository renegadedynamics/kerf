// xsbaserec: the native recorder of unit 8A1 (TKXSBase's foundation: Interface, MoniTool). It runs the cases that
// tests/OcctNet.DataExchange.Tests/Interface/XSBaseNativeTests.cs replays through the port, and writes one line per
// result: the formatting of reals (Interface_FloatWriter, TCollection_HAsciiString), the line buffer of the file
// writers (Interface_LineBuffer), the static parameters (Interface_Static::Standards and their edition), the message
// utilities (Interface_MSG), the categories, the bit maps and the entity lists. The cases come from a 64-bit LCG that
// both sides compute the same way.
//   xsbaserec <out.txt>
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#include <Interface_BitMap.hxx>
#include <Interface_Category.hxx>
#include <Interface_EntityList.hxx>
#include <Interface_FloatWriter.hxx>
#include <Interface_IntVal.hxx>
#include <Interface_LineBuffer.hxx>
#include <Interface_MSG.hxx>
#include <Interface_Static.hxx>
#include <Message_MsgFile.hxx>
#include <MoniTool_TypedValue.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static unsigned long long theSeed = 0x8A1ULL;

static unsigned int Next()
{
  theSeed = theSeed * 6364136223846793005ULL + 1442695040888963407ULL;
  return (unsigned int)(theSeed >> 33);
}

static int NextInt(int theMax) // 0 .. theMax - 1
{
  return (int)(Next() % (unsigned int)theMax);
}

// A real from the LCG: mantissa in [1, 10) with 7 decimals, decimal exponent in [-14, 14], a sign
static double NextReal()
{
  double aMant = 1.0 + (double)NextInt(9000000) / 1000000.0;
  int    anExp = NextInt(29) - 14;
  double aVal  = aMant * std::pow(10.0, anExp);
  if (NextInt(2) == 1)
    aVal = -aVal;
  return aVal;
}

static std::string Text(int theLength)
{
  static const char aChars[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJ0123456789_,.()'#=";
  std::string aText;
  for (int i = 0; i < theLength; i++)
    aText += aChars[NextInt((int)sizeof(aChars) - 1)];
  return aText;
}

static void FloatWriters(std::ostream& theOut)
{
  const double aFixed[] = {0.0,  1.0,    -1.0,   0.1,  0.09999999, 999.999, 1000.0, 1.e-7, 1.e-300, 1.e300,
                           123.456, -0.5, 3.14159265358979, 1.5e-5, 2.5, 1.e20, 100.0, 0.25, 1.e-3, -1000.0};
  std::vector<double> aValues(std::begin(aFixed), std::end(aFixed));
  for (int i = 0; i < 3000; i++)
    aValues.push_back(NextReal());
  Interface_FloatWriter aWriters[4] = {Interface_FloatWriter(), Interface_FloatWriter(12), Interface_FloatWriter(6),
                                       Interface_FloatWriter()};
  aWriters[3].SetFormat("%.10g");
  for (int w = 0; w < 4; w++)
  {
    bool   zs, rg;
    double r1, r2;
    aWriters[w].Options(zs, rg, r1, r2);
    theOut << "FW " << w << " options " << zs << " " << rg << " " << aWriters[w].MainFormat() << " "
           << aWriters[w].FormatForRange() << "\n";
    for (double aVal : aValues)
    {
      char aText[64];
      int  aLen = aWriters[w].Write(aVal, aText);
      theOut << "FW " << w << " " << aLen << " " << aText << "\n";
    }
  }
  for (int i = 0; i < 400; i++)
  {
    double aVal = (i < 20 ? aFixed[i] : NextReal());
    occ::handle<TCollection_HAsciiString> aStr = new TCollection_HAsciiString(aVal);
    theOut << "HS " << aStr->ToCString() << "\n";
  }
}

static void LineBuffers(std::ostream& theOut)
{
  for (int aCase = 0; aCase < 60; aCase++)
  {
    int                  aSize = 20 + NextInt(60);
    Interface_LineBuffer aBuf(aSize);
    theOut << "LB " << aCase << " size " << aSize << "\n";
    for (int anOp = 0; anOp < 80; anOp++)
    {
      int aKind = NextInt(10);
      switch (aKind)
      {
        case 0:
        case 1:
        case 2: {
          std::string aText = Text(1 + NextInt(15));
          bool        aCan  = aBuf.CanGet((int)aText.size());
          theOut << "LB add " << aText << " can " << aCan;
          if (aCan || NextInt(3) == 0)
            aBuf.Add(aText.c_str());
          theOut << " len " << aBuf.Length() << "\n";
          break;
        }
        case 3: {
          if (aBuf.CanGet(1))
          {
            aBuf.Add('x');
            theOut << "LB char len " << aBuf.Length() << "\n";
          }
          break;
        }
        case 4: {
          int anInit = NextInt(8);
          try
          {
            aBuf.SetInitial(anInit);
            theOut << "LB init " << anInit << "\n";
          }
          catch (Standard_Failure const&)
          {
            theOut << "LB init " << anInit << " failed\n";
          }
          break;
        }
        case 5:
          aBuf.SetKeep();
          theOut << "LB keep\n";
          break;
        case 6:
          aBuf.FreezeInitial();
          theOut << "LB freeze\n";
          break;
        case 7:
        case 8: {
          occ::handle<TCollection_HAsciiString> aLine = aBuf.Moved();
          theOut << "LB moved [" << aLine->ToCString() << "] len " << aBuf.Length() << " content ["
                 << aBuf.Content() << "]\n";
          break;
        }
        default:
          aBuf.Clear();
          theOut << "LB clear\n";
          break;
      }
    }
  }
}

static void PrintStatic(std::ostream& theOut, const char* theName)
{
  occ::handle<Interface_Static> aStat = Interface_Static::Static(theName);
  if (aStat.IsNull())
  {
    theOut << "ST " << theName << " absent\n";
    return;
  }
  theOut << "ST " << theName << " family " << Interface_Static::CDef(theName, "family") << " type "
         << Interface_Static::CDef(theName, "type") << " label [" << Interface_Static::CDef(theName, "label")
         << "] imin " << Interface_Static::CDef(theName, "imin") << " imax " << Interface_Static::CDef(theName, "imax")
         << " rmin " << Interface_Static::CDef(theName, "rmin") << " rmax " << Interface_Static::CDef(theName, "rmax")
         << " unit [" << Interface_Static::CDef(theName, "unit") << "]\n";
  // IDef reads uninitialised values for the enumeration's parts of a static that isn't one
  bool isEnum = std::string(Interface_Static::CDef(theName, "type")) == "enum";
  int  aStart = isEnum ? Interface_Static::IDef(theName, "estart") : 0;
  int  aCount = isEnum ? Interface_Static::IDef(theName, "ecount") : -2;
  if (isEnum)
    theOut << "ST " << theName << " estart " << aStart << " ecount " << aCount << " ematch "
           << Interface_Static::IDef(theName, "ematch") << "\n";
  for (int i = aStart - 1; i <= aStart + aCount; i++)
  {
    char aPart[40];
    Sprintf(aPart, "enum %d", i);
    std::string aVal = Interface_Static::CDef(theName, aPart);
    std::string anEval = "eval " + aVal;
    theOut << "ST " << theName << " enum " << i << " [" << aVal << "] case "
           << Interface_Static::IDef(theName, anEval.c_str()) << "\n";
  }
  theOut << "ST " << theName << " value [" << Interface_Static::CVal(theName) << "] ival "
         << Interface_Static::IVal(theName) << " rval " << Interface_Static::RVal(theName) << " set "
         << Interface_Static::IsSet(theName) << " def [" << aStat->Definition() << "]\n";
  std::ostringstream aPrint;
  aStat->PrintStatic(aPrint);
  std::string aText = aPrint.str();
  std::replace(aText.begin(), aText.end(), '\n', '|');
  theOut << "ST " << theName << " print " << aText << "\n";
}

static void Statics(std::ostream& theOut)
{
  Interface_Static::Standards();
  occ::handle<NCollection_HSequence<occ::handle<TCollection_HAsciiString>>> anItems = Interface_Static::Items();
  std::vector<std::string> aNames;
  for (int i = 1; i <= anItems->Length(); i++)
    aNames.push_back(anItems->Value(i)->ToCString());
  std::sort(aNames.begin(), aNames.end());
  for (const std::string& aName : aNames)
    PrintStatic(theOut, aName.c_str());

  // edition: values set as text, integer and real, valid or not
  const char* aSets[][2] = {{"read.precision.mode", "User"},     {"read.precision.mode", "File"},
                            {"read.precision.mode", "1"},        {"read.precision.mode", "bogus"},
                            {"read.precision.val", "0.05"},      {"read.precision.val", "abc"},
                            {"read.surfacecurve.mode", "-3"},    {"read.surfacecurve.mode", "?"},
                            {"read.surfacecurve.mode", "3DUse_Preferred"}, {"write.precision.mode", "Max"},
                            {"write.surfacecurve.mode", "Off"},  {"read.maxprecision.val", "1e-7"},
                            {"absent.param", "1"}};
  for (auto& aSet : aSets)
  {
    bool anOk = Interface_Static::SetCVal(aSet[0], aSet[1]);
    theOut << "SET " << aSet[0] << " " << aSet[1] << " " << anOk << " -> [" << Interface_Static::CVal(aSet[0])
           << "] " << Interface_Static::IVal(aSet[0]) << " " << Interface_Static::RVal(aSet[0]) << "\n";
  }
  const int anInts[] = {-4, -3, -1, 0, 1, 2, 3, 4, 5, 6};
  for (int aVal : anInts)
  {
    bool anOk = Interface_Static::SetIVal("read.surfacecurve.mode", aVal);
    theOut << "SETI read.surfacecurve.mode " << aVal << " " << anOk << " -> ["
           << Interface_Static::CVal("read.surfacecurve.mode") << "] "
           << Interface_Static::IVal("read.surfacecurve.mode") << "\n";
  }
  const double aReals[] = {0.001, 1.5, -2.0, 1.e-9, 123456.789};
  for (double aVal : aReals)
  {
    bool anOk = Interface_Static::SetRVal("read.precision.val", aVal);
    theOut << "SETR read.precision.val " << aVal << " " << anOk << " -> ["
           << Interface_Static::CVal("read.precision.val") << "] " << Interface_Static::RVal("read.precision.val")
           << "\n";
  }

  // a family of new statics: integers with limits, reals with units, texts, '=' copies, edition texts
  Interface_Static::Init("XSTEP8A1", "t8a1.int", 'i', "5");
  Interface_Static::Init("XSTEP8A1", "t8a1.int", '&', "imin 1");
  Interface_Static::Init("XSTEP8A1", "t8a1.int", '&', "imax 10");
  Interface_Static::Init("XSTEP8A1", "t8a1.real", 'r', "2.5");
  Interface_Static::Init("XSTEP8A1", "t8a1.real", '&', "rmin 0.5");
  Interface_Static::Init("XSTEP8A1", "t8a1.real", '&', "unit mm");
  Interface_Static::Init("XSTEP8A1", "t8a1.text", 't', "hello");
  Interface_Static::Init("XSTEP8A1", "t8a1.path", 'p', "dir/file");
  Interface_Static::Init("XSTEP8A1", "t8a1.copy", '=', "read.surfacecurve.mode");
  Interface_Static::Init("XSTEP8A1", "t8a1.enum", 'e', "");
  Interface_Static::Init("XSTEP8A1", "t8a1.enum", '&', "enum 2");
  Interface_Static::Init("XSTEP8A1", "t8a1.enum", '&', "eval One");
  Interface_Static::Init("XSTEP8A1", "t8a1.enum", '&', "eval Two");
  Interface_Static::Init("XSTEP8A1", "t8a1.enum", '&', "eval Three");
  theOut << "INIT again " << Interface_Static::Init("XSTEP8A1", "t8a1.int", 'i', "7") << " bad "
         << Interface_Static::Init("XSTEP8A1", "t8a1.bad", 'z', "") << " badedit "
         << Interface_Static::Init("XSTEP8A1", "t8a1.int", '&', "zzz 1") << "\n";
  const char* aNew[] = {"t8a1.int", "t8a1.real", "t8a1.text", "t8a1.path", "t8a1.copy", "t8a1.enum"};
  for (const char* aName : aNew)
    PrintStatic(theOut, aName);
  const char* aNewSets[][2] = {{"t8a1.int", "0"},  {"t8a1.int", "10"},   {"t8a1.int", "11"},
                               {"t8a1.int", "3.5"}, {"t8a1.real", "0.4"}, {"t8a1.real", "7"},
                               {"t8a1.enum", "Two"}, {"t8a1.enum", "5"},  {"t8a1.enum", "One"}};
  for (auto& aSet : aNewSets)
  {
    bool anOk = Interface_Static::SetCVal(aSet[0], aSet[1]);
    theOut << "SET " << aSet[0] << " " << aSet[1] << " " << anOk << " -> [" << Interface_Static::CVal(aSet[0])
           << "] " << Interface_Static::IVal(aSet[0]) << " " << Interface_Static::RVal(aSet[0]) << "\n";
  }
  occ::handle<NCollection_HSequence<occ::handle<TCollection_HAsciiString>>> aFamily =
    Interface_Static::Items(0, "XSTEP8A1");
  theOut << "ITEMS XSTEP8A1 " << aFamily->Length() << "\n";

  // the library of types
  const char* aLibs[] = {"Integer", "Real", "Text", "Transient", "Boolean", "Logical", "Unknown"};
  for (const char* aName : aLibs)
  {
    occ::handle<MoniTool_TypedValue> aTV = MoniTool_TypedValue::FromLib(aName);
    if (aTV.IsNull())
    {
      theOut << "LIB " << aName << " null\n";
      continue;
    }
    std::ostringstream aPrint;
    aTV->Print(aPrint);
    std::string aText = aPrint.str();
    std::replace(aText.begin(), aText.end(), '\n', '|');
    theOut << "LIB " << aName << " " << aText << "\n";
  }
  theOut << "MSG XSTEP_1 [" << TCollection_AsciiString(Message_MsgFile::Msg("XSTEP_1")).ToCString() << "]\n";
}

static void Msgs(std::ostream& theOut)
{
  const int aVals[] = {0, 5, 9, 10, 99, 100, 12345, 999999999, 1000000000, -7, -1234};
  for (int aVal : aVals)
    for (int aMax = 0; aMax <= 12; aMax += 3)
      theOut << "BL " << aVal << " " << aMax << " [" << Interface_MSG::Blanks(aVal, aMax) << "]\n";
  const char* aTexts[] = {"", "a", "abcdef", "abcdefghijklmnop"};
  for (const char* aText : aTexts)
    for (int aMax = 0; aMax <= 12; aMax += 4)
      theOut << "BS [" << aText << "] " << aMax << " [" << Interface_MSG::Blanks(aText, aMax) << "]\n";
  for (int aCount = -1; aCount <= 80; aCount += 9)
    theOut << "BC " << aCount << " [" << Interface_MSG::Blanks(aCount) << "]\n";
  for (int i = 0; i < 300; i++)
  {
    double aVal   = (i % 10 == 0 ? 0.0 : NextReal());
    int    anOrd  = NextInt(12);
    bool   anUp   = NextInt(2) == 1;
    double aRes   = Interface_MSG::Intervalled(aVal, anOrd, anUp);
    char   aText[64];
    Sprintf(aText, "%.17g %d %d %.17g", aVal, anOrd, (int)anUp, aRes);
    theOut << "IV " << aText << "\n";
  }
  const char* aDates[] = {"2026-11-02:10-20-30", "1999-01-01:00-00-00", "2026-11-02:10-20-31", "2026-11-03",
                          "abc", "12/7/2020 8h5", "2026-11-02:10-20-30"};
  for (const char* aD1 : aDates)
  {
    int yy = 0, mm = 0, dd = 0, hh = 0, mn = 0, ss = 0;
    bool anOk = Interface_MSG::NDate(aD1, yy, mm, dd, hh, mn, ss);
    theOut << "ND [" << aD1 << "] " << anOk << " " << yy << " " << mm << " " << dd << " " << hh << " " << mn << " "
           << ss;
    for (const char* aD2 : aDates)
      theOut << " " << Interface_MSG::CDate(aD1, aD2);
    theOut << "\n";
  }
  char aText[64];
  Interface_MSG::TDate(aText, 2026, 11, 2, 9, 8, 7, "");
  theOut << "TD [" << aText << "]\n";
  Interface_MSG::TDate(aText, 2026, 11, 2, 9, 8, 7, "C:%4.4d-%2.2d-%2.2dT%2.2d:%2.2d:%2.2d");
  theOut << "TD [" << aText << "]\n";
  Interface_Category::Init(); // the categories are recorded at the first Interface_Category
  for (int i = -1; i <= 10; i++)
    theOut << "CAT " << i << " " << Interface_Category::Name(i) << "\n";
  const char* aCats[] = {"Shape", "Drawing", "Structure", "Description", "Auxiliary",
                         "Professional", "FEA", "Kinematics", "Piping", "Other"};
  for (const char* aCat : aCats)
    theOut << "CATN " << aCat << " " << Interface_Category::Number(aCat) << "\n";
  theOut << "CATS " << Interface_Category::NbCategories() << "\n";
}

static void BitMaps(std::ostream& theOut)
{
  for (int aCase = 0; aCase < 20; aCase++)
  {
    int              aNb = 1 + NextInt(100);
    Interface_BitMap aMap(aNb, NextInt(3));
    theOut << "BM " << aCase << " " << aNb << "\n";
    for (int anOp = 0; anOp < 120; anOp++)
    {
      int aKind = NextInt(8);
      int aNum  = 1 + NextInt(aMap.Length());
      int aFlag = NextInt(aMap.NbFlags() + 1);
      switch (aKind)
      {
        case 0:
          aMap.SetTrue(aNum, aFlag);
          break;
        case 1:
          aMap.SetFalse(aNum, aFlag);
          break;
        case 2:
          theOut << "BM ctrue " << aNum << " " << aFlag << " " << aMap.CTrue(aNum, aFlag) << "\n";
          break;
        case 3:
          theOut << "BM cfalse " << aNum << " " << aFlag << " " << aMap.CFalse(aNum, aFlag) << "\n";
          break;
        case 4: {
          std::string aName = "f" + std::to_string(NextInt(5));
          theOut << "BM addflag " << aName << " " << aMap.AddFlag(aName.c_str()) << " nb " << aMap.NbFlags()
                 << "\n";
          break;
        }
        case 5:
          theOut << "BM flagnumber " << aMap.FlagNumber("f2") << "\n";
          break;
        case 6:
          aMap.SetValue(aNum, NextInt(2) == 1, aFlag);
          break;
        default: {
          std::string aBits;
          for (int i = 1; i <= aMap.Length(); i++)
            aBits += (aMap.Value(i, aFlag) ? '1' : '0');
          theOut << "BM bits " << aFlag << " " << aBits << "\n";
          break;
        }
      }
    }
  }
}

static void EntityLists(std::ostream& theOut)
{
  for (int aCase = 0; aCase < 20; aCase++)
  {
    Interface_EntityList                         aList;
    std::vector<occ::handle<Interface_IntVal>> anItems;
    int                                          aNb = 1 + NextInt(30);
    for (int i = 0; i < aNb; i++)
    {
      occ::handle<Interface_IntVal> anItem = new Interface_IntVal;
      anItem->CValue()                     = i + 1;
      anItems.push_back(anItem);
      if (NextInt(3) == 0)
        aList.Add(anItem);
      else
        aList.Append(anItem);
    }
    // removals at the end only: OCCT's EntityCluster::Remove(num) runs out of its array for an inner rank
    int aRemoves = NextInt(3);
    for (int r = 0; r < aRemoves && aList.NbEntities() > 1; r++)
      aList.Remove(aList.NbEntities());
    std::string aVals;
    // OCCT's EntityCluster::Value counts with the first cluster's size: it fails past a first cluster that Add left
    // not full
    for (int i = 1; i <= aList.NbEntities(); i++)
    {
      try
      {
        aVals += " " + std::to_string(occ::down_cast<Interface_IntVal>(aList.Value(i))->Value());
      }
      catch (Standard_Failure const&)
      {
        aVals += " !";
      }
    }
    theOut << "EL " << aCase << " " << aList.NbEntities() << ":" << aVals << "\n";
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: xsbaserec <out.txt>\n");
    return 2;
  }
  std::ofstream anOut(argv[1], std::ios::binary);
  FloatWriters(anOut);
  LineBuffers(anOut);
  Statics(anOut);
  Msgs(anOut);
  BitMaps(anOut);
  EntityLists(anOut);
  return 0;
}
