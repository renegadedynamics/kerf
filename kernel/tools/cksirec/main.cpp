// Records native BOPAlgo_CheckerSI (bopcheck's algorithm, at its default level) on a shape translated along Z, and
// prints the alerts it reports (unit 7W): bugs/modalg_7/bug26034 checks a revolved shape after `ttranslate 0 0 -0.5`.
// Built against build123d.NET's native OCCT V8_0_1, which keeps OCCT's Raise_if checks (No_Exception undefined).
//   cksirec <file.brep> <dz>...
#include <cstdio>
#include <cstdlib>

#include <BOPAlgo_CheckerSI.hxx>
#include <BOPDS_DS.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Message_Alert.hxx>
#include <Message_Gravity.hxx>
#include <Message_Report.hxx>
#include <NCollection_List.hxx>
#include <Standard_Type.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    printf("usage: cksirec <file.brep> <dz>...\n");
    return 1;
  }
  TopoDS_Shape aS;
  BRep_Builder aB;
  if (!BRepTools::Read(aS, argv[1], aB))
  {
    printf("cannot read %s\n", argv[1]);
    return 1;
  }
  for (int i = 2; i < argc; ++i)
  {
    // ttranslate: BRepBuilderAPI_Transform without copy, a location
    gp_Trsf aT;
    aT.SetTranslation(gp_Vec(0, 0, atof(argv[i])));
    BRepBuilderAPI_Transform       aTr(aS, aT, false);
    NCollection_List<TopoDS_Shape> aL;
    aL.Append(aTr.Shape());
    BOPAlgo_CheckerSI aChecker;
    aChecker.SetArguments(aL);
    aChecker.SetLevelOfCheck(BOPDS_DS::NbInterfTypes() - 1);
    aChecker.SetRunParallel(false);
    aChecker.Perform();
    printf("dz %s: errors %d, warnings %d\n", argv[i], (int)aChecker.HasErrors(), (int)aChecker.HasWarnings());
    for (int g = 0; g < 4; ++g)
    {
      const auto& anAlerts = aChecker.GetReport()->GetAlerts((Message_Gravity)g);
      for (auto it = anAlerts.begin(); it != anAlerts.end(); ++it)
      {
        printf("  gravity %d: %s\n", g, (*it)->DynamicType()->Name());
      }
    }
  }
  return 0;
}
