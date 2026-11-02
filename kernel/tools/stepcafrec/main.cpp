// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// stepcafrec: records what native OCCT's STEPCAFControl_Reader and STEPCAFControl_Writer do with XCAF documents, for
// unit 8G2 (tools/stepcafrec/build.ps1). build123d.NET binds neither XCAF nor STEPCAFControl.
//
//   stepcafrec read <list> <out>
//       list: one "name<TAB>path" per line (the corpus's STEP files). Each file is read with STEPCAFControl_Reader
//       (OCCT's default statics, XCAF's auto-naming on) into a new XCAF document, which is dumped (Dump below):
//         "F <name>", then "R <read status> <transfer ok 0|1>", then the document's dump.
//   stepcafrec write <breps> <steps> <out> [<dumpdir>]
//       Documents written with STEPCAFControl_Writer to a stream, in each schema (AP203, AP214IS, AP242DIS) for the
//       built ones: data/occ's shapes each in a document with a name, colours, a named sub-shape, layers and
//       validation properties (MakePartDoc), three assemblies with names, an instance colour and layers
//       (MakeAssemblyDoc), then each STEP file of the corpus read with STEPCAFControl_Reader and written back in
//       AP214IS. Per case: "W <name> <schema> <ok> <entities> <sha256 of the masked text> <its length> <sha256 of
//       its canonical form>"; the text
//       unwrapped, then masked as stepctlrec masks it. The port's test (tests/OcctNet.DataExchange.Tests/STEPCAFControl) builds the
//       same documents with XdeDoc_Document and computes the same lines.
//
// The dump of a document: "U <length unit in metres>" (or "U -"), then each label under 0:1:1, depth first:
//   "L <entry> <kind>"   kind: A assembly, R reference, S simple shape, U sub-shape, E none
//   "T <type> <solids> <shells> <faces> <edges> <vertices>"   its shape (TopExp::MapShapes counts)
//   "P <entry>"           a reference's referred label
//   "O <12 numbers>"      a reference's location (gp_Trsf's 3x4 matrix)
//   "N <name>"            (\ and newlines escaped)
//   "C <g|s|c> <r> <g> <b> <a>"   colours (linear RGB, as floats)
//   "H"                   hidden
//   "Y <layer>"           each layer of the label
//   "V <volume>", "A <area>", "G <x> <y> <z>"
//   "X <extern ref>"      (XCAF keeps them in a child label, which the dump leaves out)
// then each layer label: "Z <entry> <visible 0|1> <name>". Reals as %.17g, floats as %.9g.
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
#include <Interface_Static.hxx>
#include <OSD.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StepData_StepModel.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_Area.hxx>
#include <XCAFDoc_Centroid.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_Volume.hxx>
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

static std::string R17(double theValue)
{
  char aBuf[64];
  sprintf(aBuf, "%.17g", theValue);
  return aBuf;
}

static std::string F9(float theValue)
{
  char aBuf[64];
  sprintf(aBuf, "%.9g", (double)theValue);
  return aBuf;
}

static std::string Utf8(const TCollection_ExtendedString& theString)
{
  TCollection_AsciiString anAscii(theString);
  // TCollection_AsciiString(ExtendedString) converts to UTF-8 in OCCT 8
  std::string aRes;
  for (const char* p = anAscii.ToCString(); *p != 0; ++p)
  {
    if (*p == '\\')
      aRes += "\\\\";
    else if (*p == '\n')
      aRes += "\\n";
    else if (*p == '\r')
      aRes += "\\r";
    else
      aRes += *p;
  }
  return aRes;
}

static std::string Entry(const TDF_Label& theLabel)
{
  TCollection_AsciiString anEntry;
  TDF_Tool::Entry(theLabel, anEntry);
  return anEntry.ToCString();
}

static int Count(const TopoDS_Shape& theShape, TopAbs_ShapeEnum theType)
{
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, theType, aMap);
  return aMap.Extent();
}

static const char* TypeName(TopAbs_ShapeEnum theType)
{
  static const char* const aNames[] = {"COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"};
  return aNames[(int)theType];
}

static void DumpLabel(const TDF_Label& theLabel, std::ostream& theOut)
{
  const char* aKind = XCAFDoc_ShapeTool::IsAssembly(theLabel)    ? "A"
                      : XCAFDoc_ShapeTool::IsReference(theLabel) ? "R"
                      : XCAFDoc_ShapeTool::IsSubShape(theLabel)  ? "U"
                      : XCAFDoc_ShapeTool::IsSimpleShape(theLabel) ? "S"
                                                                   : "E";
  theOut << "L " << Entry(theLabel) << " " << aKind << "\n";
  TopoDS_Shape aShape;
  if (XCAFDoc_ShapeTool::GetShape(theLabel, aShape) && !aShape.IsNull())
  {
    theOut << "T " << TypeName(aShape.ShapeType()) << " " << Count(aShape, TopAbs_SOLID) << " "
           << Count(aShape, TopAbs_SHELL) << " " << Count(aShape, TopAbs_FACE) << " " << Count(aShape, TopAbs_EDGE)
           << " " << Count(aShape, TopAbs_VERTEX) << "\n";
  }
  TDF_Label aReferred;
  if (XCAFDoc_ShapeTool::GetReferredShape(theLabel, aReferred))
  {
    theOut << "P " << Entry(aReferred) << "\n";
    gp_Trsf aTrsf = XCAFDoc_ShapeTool::GetLocation(theLabel).Transformation();
    theOut << "O";
    for (int r = 1; r <= 3; r++)
      for (int c = 1; c <= 4; c++)
        theOut << " " << R17(aTrsf.Value(r, c));
    theOut << "\n";
  }
  occ::handle<TDataStd_Name> aName;
  if (theLabel.FindAttribute(TDataStd_Name::GetID(), aName))
    theOut << "N " << Utf8(aName->Get()) << "\n";
  static const std::pair<XCAFDoc_ColorType, const char*> aTypes[] = {
    {XCAFDoc_ColorGen, "g"}, {XCAFDoc_ColorSurf, "s"}, {XCAFDoc_ColorCurv, "c"}};
  for (const auto& aType : aTypes)
  {
    Quantity_ColorRGBA aColor;
    if (XCAFDoc_ColorTool::GetColor(theLabel, aType.first, aColor))
    {
      theOut << "C " << aType.second << " " << F9(aColor.GetRGB().Rgb().r()) << " "
             << F9(aColor.GetRGB().Rgb().g()) << " " << F9(aColor.GetRGB().Rgb().b()) << " " << F9(aColor.Alpha())
             << "\n";
    }
  }
  if (!XCAFDoc_ColorTool::IsVisible(theLabel))
    theOut << "H\n";
  occ::handle<XCAFDoc_LayerTool> aLTool = XCAFDoc_DocumentTool::LayerTool(theLabel);
  occ::handle<NCollection_HSequence<TCollection_ExtendedString>> aLayers = aLTool->GetLayers(theLabel);
  if (!aLayers.IsNull())
    for (int i = 1; i <= aLayers->Length(); i++)
      theOut << "Y " << Utf8(aLayers->Value(i)) << "\n";
  double aValue = 0.0;
  if (XCAFDoc_Volume::Get(theLabel, aValue))
    theOut << "V " << R17(aValue) << "\n";
  if (XCAFDoc_Area::Get(theLabel, aValue))
    theOut << "A " << R17(aValue) << "\n";
  gp_Pnt aPnt;
  if (XCAFDoc_Centroid::Get(theLabel, aPnt))
    theOut << "G " << R17(aPnt.X()) << " " << R17(aPnt.Y()) << " " << R17(aPnt.Z()) << "\n";
  // the children; XCAFDoc_ShapeTool::SetExternRefs's child (its children named by the files) as "X" lines, last
  std::string anExtern;
  for (TDF_ChildIterator anIt(theLabel); anIt.More(); anIt.Next())
  {
    if (anIt.Value().IsAttribute(XCAFDoc::ExternRefGUID()))
    {
      for (TDF_ChildIterator aRefIt(anIt.Value()); aRefIt.More(); aRefIt.Next())
      {
        occ::handle<TDataStd_Name> aRefName;
        if (aRefIt.Value().FindAttribute(TDataStd_Name::GetID(), aRefName))
          anExtern += "X " + Utf8(aRefName->Get()) + "\n";
      }
      continue;
    }
    DumpLabel(anIt.Value(), theOut);
  }
  theOut << anExtern;
}

static void Dump(const occ::handle<TDocStd_Document>& theDoc, std::ostream& theOut)
{
  double aUnit = 0.0;
  if (XCAFDoc_DocumentTool::GetLengthUnit(theDoc, aUnit))
    theOut << "U " << R17(aUnit) << "\n";
  else
    theOut << "U -\n";
  occ::handle<XCAFDoc_ShapeTool> aSTool = XCAFDoc_DocumentTool::ShapeTool(theDoc->Main());
  for (TDF_ChildIterator anIt(aSTool->Label()); anIt.More(); anIt.Next())
    DumpLabel(anIt.Value(), theOut);
  occ::handle<XCAFDoc_LayerTool> aLTool = XCAFDoc_DocumentTool::LayerTool(theDoc->Main());
  NCollection_Sequence<TDF_Label> aLayers;
  aLTool->GetLayerLabels(aLayers);
  for (int i = 1; i <= aLayers.Length(); i++)
  {
    TCollection_ExtendedString aName;
    aLTool->GetLayer(aLayers.Value(i), aName);
    theOut << "Z " << Entry(aLayers.Value(i)) << " " << (aLTool->IsVisible(aLayers.Value(i)) ? 1 : 0) << " "
           << Utf8(aName) << "\n";
  }
}

static occ::handle<TDocStd_Document> NewDoc()
{
  occ::handle<TDocStd_Document> aDoc;
  XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", aDoc);
  return aDoc;
}

// ---------------------------------------------------------------------------------------------------------------
// Reading

static int Read(const char* theList, const char* theOut)
{
  std::ofstream anOut(theOut, std::ios::binary);
  for (const auto& aFile : ReadList(theList))
  {
    anOut << "F " << aFile.first << "\n";
    try
    {
      OCC_CATCH_SIGNALS
      occ::handle<TDocStd_Document> aDoc = NewDoc();
      STEPCAFControl_Reader         aReader;
      IFSelect_ReturnStatus         aStatus = aReader.ReadFile(aFile.second.c_str());
      bool                          isOk    = false;
      if (aStatus == IFSelect_RetDone)
        isOk = aReader.Transfer(aDoc);
      anOut << "R " << (int)aStatus << " " << (isOk ? 1 : 0) << "\n";
      if (isOk)
        Dump(aDoc, anOut);
      aDoc->Close();
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

// The text unwrapped (each line break followed by an indentation removed: the identifiers masked below change where
// the writer breaks the lines), then its records with their varying parts masked as tools/stepctlrec masks them
static std::string Unwrap(const std::string& theText)
{
  std::string aRes;
  for (size_t i = 0; i < theText.size(); i++)
  {
    if (theText[i] == '\n' && i + 1 < theText.size() && theText[i + 1] == ' ')
    {
      while (i + 1 < theText.size() && theText[i + 1] == ' ')
        i++;
      continue;
    }
    aRes += theText[i];
  }
  return aRes;
}

static std::string Mask(const std::string& theWrapped)
{
  const std::string theText = Unwrap(theWrapped);
  static const char* const aMasked[] = {"FILE_NAME", "CALENDAR_DATE", "LOCAL_TIME",
                                        "COORDINATED_UNIVERSAL_TIME_OFFSET", "PERSON", "ORGANIZATION"};
  std::string aRes;
  size_t      aPos = 0;
  while (aPos < theText.size())
  {
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

// The canonical form of a masked text: its lines with the entity numbers erased, sorted (the order in which
// STEPCAFControl_Writer adds the styles' representations follows a map keyed by shapes, by address: the port adds
// them in the order it makes them, decision 13505)
static std::string Canonical(const std::string& theText)
{
  std::vector<std::string> aLines;
  std::string              aLine;
  for (size_t i = 0; i <= theText.size(); i++)
  {
    if (i == theText.size() || theText[i] == '\n')
    {
      aLines.push_back(aLine);
      aLine.clear();
      continue;
    }
    char c = theText[i];
    aLine += c;
    if (c == '#')
      while (i + 1 < theText.size() && theText[i + 1] >= '0' && theText[i + 1] <= '9')
        i++;
  }
  std::sort(aLines.begin(), aLines.end());
  std::string aRes;
  for (const std::string& aL : aLines)
    aRes += aL + "\n";
  return aRes;
}

static TopoDS_Shape ReadBRep(const std::string& thePath)
{
  TopoDS_Shape aShape;
  BRep_Builder aBuilder;
  BRepTools::Read(aShape, thePath.c_str(), aBuilder);
  return aShape;
}

// The palette of the built documents: a predefined STEP colour, two RGB ones (one translucent), a generic one
static Quantity_ColorRGBA PaletteColor(int theIndex)
{
  switch (theIndex % 4)
  {
    case 0: return Quantity_ColorRGBA(Quantity_Color(Quantity_NOC_RED));
    case 1: return Quantity_ColorRGBA(Quantity_Color(0.25, 0.5, 0.75, Quantity_TOC_RGB));
    case 2: return Quantity_ColorRGBA(Quantity_Color(0.125, 0.625, 0.25, Quantity_TOC_RGB), 0.5f);
    default: return Quantity_ColorRGBA(Quantity_Color(Quantity_NOC_GREEN));
  }
}

// A part's document (the port's test builds the same): the shape (not as an assembly) named by its file, a colour
// (surface, or generic one time in three), the first face as a named sub-shape with a curve colour, layers "parts"
// (hidden one time in five) and "faces", the part hidden one time in seven, validation properties
static occ::handle<TDocStd_Document> MakePartDoc(const TopoDS_Shape& theShape, const std::string& theName, int theIndex)
{
  occ::handle<TDocStd_Document>  aDoc   = NewDoc();
  occ::handle<XCAFDoc_ShapeTool> aSTool = XCAFDoc_DocumentTool::ShapeTool(aDoc->Main());
  occ::handle<XCAFDoc_ColorTool> aCTool = XCAFDoc_DocumentTool::ColorTool(aDoc->Main());
  occ::handle<XCAFDoc_LayerTool> aLTool = XCAFDoc_DocumentTool::LayerTool(aDoc->Main());
  TDF_Label                      aLabel = aSTool->AddShape(theShape, false);
  TDataStd_Name::Set(aLabel, TCollection_ExtendedString(theName.c_str()));
  aCTool->SetColor(aLabel, PaletteColor(theIndex), theIndex % 3 == 1 ? XCAFDoc_ColorGen : XCAFDoc_ColorSurf);
  TopExp_Explorer anExp(theShape, TopAbs_FACE);
  if (anExp.More())
  {
    TDF_Label aSub = aSTool->AddSubShape(aLabel, anExp.Current());
    if (!aSub.IsNull())
    {
      TDataStd_Name::Set(aSub, "first face");
      aCTool->SetColor(aSub, PaletteColor(theIndex + 1).GetRGB(), XCAFDoc_ColorCurv);
      aLTool->SetLayer(aSub, TCollection_ExtendedString("faces"));
    }
  }
  aLTool->SetLayer(aLabel, TCollection_ExtendedString("parts"));
  if (theIndex % 5 == 0)
    aLTool->SetVisibility(aLTool->FindLayer("parts"), false);
  if (theIndex % 7 == 3)
    aCTool->SetVisibility(aLabel, false);
  XCAFDoc_Volume::Set(aLabel, 100.0 + theIndex);
  XCAFDoc_Area::Set(aLabel, 50.5 + theIndex);
  XCAFDoc_Centroid::Set(aLabel, gp_Pnt(theIndex, 2.0 * theIndex, 0.5));
  return aDoc;
}

// An assembly's document: a part, a sub-assembly of two instances of it (the second rotated, moved, named and
// coloured), the top assembly of two instances of the sub-assembly
static occ::handle<TDocStd_Document> MakeAssemblyDoc(const TopoDS_Shape& theShape)
{
  occ::handle<TDocStd_Document>  aDoc   = NewDoc();
  occ::handle<XCAFDoc_ShapeTool> aSTool = XCAFDoc_DocumentTool::ShapeTool(aDoc->Main());
  occ::handle<XCAFDoc_ColorTool> aCTool = XCAFDoc_DocumentTool::ColorTool(aDoc->Main());
  occ::handle<XCAFDoc_LayerTool> aLTool = XCAFDoc_DocumentTool::LayerTool(aDoc->Main());
  TDF_Label                      aPart  = aSTool->AddShape(theShape, false);
  TDataStd_Name::Set(aPart, "part");
  aCTool->SetColor(aPart, Quantity_Color(Quantity_NOC_RED), XCAFDoc_ColorSurf);
  aLTool->SetLayer(aPart, TCollection_ExtendedString("parts"));
  XCAFDoc_Volume::Set(aPart, 12.5);
  TDF_Label aSub = aSTool->NewShape();
  TDataStd_Name::Set(aSub, "sub");
  gp_Trsf aTrsf;
  aTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0.5);
  aTrsf.SetTranslationPart(gp_Vec(100.0, 50.0, 0.0));
  aSTool->AddComponent(aSub, aPart, TopLoc_Location());
  TDF_Label aSecond = aSTool->AddComponent(aSub, aPart, TopLoc_Location(aTrsf));
  TDataStd_Name::Set(aSecond, "second");
  aCTool->SetColor(aSecond, Quantity_Color(0.25, 0.5, 0.75, Quantity_TOC_RGB), XCAFDoc_ColorSurf);
  TDF_Label aTop = aSTool->NewShape();
  TDataStd_Name::Set(aTop, "top");
  gp_Trsf aTrsf2;
  aTrsf2.SetTranslation(gp_Vec(0.0, 0.0, 300.0));
  aSTool->AddComponent(aTop, aSub, TopLoc_Location());
  TDF_Label anUpper = aSTool->AddComponent(aTop, aSub, TopLoc_Location(aTrsf2));
  TDataStd_Name::Set(anUpper, "upper");
  aSTool->UpdateAssemblies();
  return aDoc;
}

static void WriteDoc(const occ::handle<TDocStd_Document>& theDoc, const std::string& theName, const char* theSchema,
                     std::ostream& theOut, const char* theDump)
{
  theOut << "W " << SafeName(theName) << " " << theSchema << " ";
  try
  {
    OCC_CATCH_SIGNALS
    Interface_Static::SetCVal("write.step.schema", theSchema);
    STEPCAFControl_Writer aWriter;
    bool                  isOk = aWriter.Transfer(theDoc, STEPControl_AsIs);
    if (!isOk)
    {
      theOut << "0\n";
      return;
    }
    std::ostringstream aStream;
    aWriter.WriteStream(aStream);
    std::string aText = Mask(aStream.str());
    theOut << "1 " << aWriter.ChangeWriter().Model()->NbEntities() << " " << Sha256(aText) << " " << aText.size() << " "
           << Sha256(Canonical(aText)) << "\n";
    if (theDump != nullptr)
    {
      std::ofstream aDump(std::string(theDump) + "/" + SafeName(theName) + "." + theSchema + ".stp", std::ios::binary);
      aDump << aText;
    }
  }
  catch (Standard_Failure const& anExc)
  {
    theOut << "EXC " << anExc.what() << "\n";
  }
}

static int Write(const char* theBreps, const char* theSteps, const char* theOut, const char* theDump)
{
  static const char* const aSchemas[] = {"AP203", "AP214IS", "AP242DIS"};
  std::ofstream            anOut(theOut, std::ios::binary);
  std::map<std::string, std::string> aPaths;
  int                                anIndex = 0;
  for (const auto& aFile : ReadList(theBreps))
  {
    std::string aShort = aFile.first.substr(aFile.first.rfind('/') + 1);
    aPaths[aShort]     = aFile.second;
    TopoDS_Shape aShape = ReadBRep(aFile.second);
    for (const char* aSchema : aSchemas)
    {
      occ::handle<TDocStd_Document> aDoc = MakePartDoc(aShape, aShort, anIndex);
      WriteDoc(aDoc, "part " + aFile.first, aSchema, anOut, theDump);
      aDoc->Close();
    }
    anIndex++;
  }
  for (const char* aName : {"Ball.brep", "Pump_Nut.brep", "bottle.brep"})
  {
    if (aPaths.count(aName) == 0)
      continue;
    TopoDS_Shape aShape = ReadBRep(aPaths[aName]);
    for (const char* aSchema : aSchemas)
    {
      occ::handle<TDocStd_Document> aDoc = MakeAssemblyDoc(aShape);
      WriteDoc(aDoc, std::string("assembly ") + aName, aSchema, anOut, theDump);
      aDoc->Close();
    }
  }
  for (const auto& aFile : ReadList(theSteps))
  {
    occ::handle<TDocStd_Document> aDoc = NewDoc();
    bool                          isOk = false;
    try
    {
      OCC_CATCH_SIGNALS
      Interface_Static::SetCVal("write.step.schema", "AP214IS");
      STEPCAFControl_Reader aReader;
      isOk = aReader.ReadFile(aFile.second.c_str()) == IFSelect_RetDone && aReader.Transfer(aDoc);
    }
    catch (Standard_Failure const&)
    {
      isOk = false;
    }
    if (!isOk)
    {
      anOut << "W " << SafeName("step " + aFile.first) << " AP214IS -\n";
    }
    else
    {
      WriteDoc(aDoc, "step " + aFile.first, "AP214IS", anOut, theDump);
    }
    aDoc->Close();
  }
  Interface_Static::SetCVal("write.step.schema", "AP214IS");
  return 0;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    fprintf(stderr, "usage: stepcafrec read <list> <out> | write <breps> <steps> <out> [<dumpdir>]\n");
    return 2;
  }
  OSD::SetSignal(false);
  STEPCAFControl_Reader anInit;
  std::string           aMode = argv[1];
  if (aMode == "read")
    return Read(argv[2], argv[3]);
  if (aMode == "write" && argc >= 5)
    return Write(argv[2], argv[3], argv[4], argc > 5 ? argv[5] : nullptr);
  return 2;
}
