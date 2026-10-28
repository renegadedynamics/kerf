// Box minus cylinder, checked by volume, then written to STEP through XCAF with a name and colour.
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Controller.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <Standard_Version.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax2.hxx>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

int main(int argc, char** argv)
{
  const char* out = argc > 1 ? argv[1] : "smoke.step";
  std::printf("OCCT %s\n", OCC_VERSION_COMPLETE);

  TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-5, -5, -5), 10, 10, 10).Shape();
  TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -5), gp::DZ()), 3, 10).Shape();
  BRepAlgoAPI_Cut cut(box, cyl);
  if (!cut.IsDone())
  {
    std::printf("FAIL: boolean cut\n");
    return 1;
  }

  GProp_GProps props;
  BRepGProp::VolumeProperties(cut.Shape(), props);
  const double expected = 1000.0 - std::acos(-1.0) * 9.0 * 10.0;
  std::printf("volume %.6f (expected %.6f)\n", props.Mass(), expected);
  if (std::abs(props.Mass() - expected) > 1e-6)
  {
    std::printf("FAIL: volume\n");
    return 1;
  }

  // As build123d does: a plain document initialised (not opened) by the XCAF application
  Handle(TDocStd_Document) doc = new TDocStd_Document("XmlOcaf");
  XCAFApp_Application::GetApplication()->InitDocument(doc);
  // A boolean result is a compound; with the default makeAssembly=true XCAF would turn it into an
  // assembly, and STEP cannot carry a colour on an assembly, so it would be silently dropped.
  TDF_Label label =
    XCAFDoc_DocumentTool::ShapeTool(doc->Main())->AddShape(cut.Shape(), /*makeAssembly=*/false);
  TDataStd_Name::Set(label, "block");
  XCAFDoc_DocumentTool::ColorTool(doc->Main())
    ->SetColor(label, Quantity_Color(Quantity_NOC_STEELBLUE), XCAFDoc_ColorGen);

  // Registers the XCAF-aware STEP actor; without it colours are silently dropped
  STEPCAFControl_Controller::Init();
  STEPCAFControl_Writer writer;
  writer.SetColorMode(true);
  writer.SetNameMode(true);
  if (!writer.Transfer(doc) || writer.Write(out) != IFSelect_RetDone)
  {
    std::printf("FAIL: STEP write\n");
    return 1;
  }

  // The colour must survive the export, not just the name
  std::ifstream step(out);
  const std::string text((std::istreambuf_iterator<char>(step)), std::istreambuf_iterator<char>());
  if (text.find("COLOUR_RGB") == std::string::npos || text.find("'block'") == std::string::npos)
  {
    std::printf("FAIL: colour or name missing from %s\n", out);
    return 1;
  }
  std::printf("OK: wrote %s\n", out);
  return 0;
}
