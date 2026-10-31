// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// evorec: native values of BRepOffsetAPI_MakeEvolved (unit 5O2) for BRepFill_EvolvedTests, which native OCCT doesn't
// bind for the oracle. Vertical profiles off the spine, solid, on a square and an L-shaped spine: whether it is done,
// whether the result is valid, its volume, the bottom's and top's areas, and its number of faces (%.17g).
// Usage: evorec > values.txt

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeEvolved.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>

#include <cstdio>
#include <vector>

static TopoDS_Wire Polygon(bool theClosed, const std::vector<double>& theXYZ)
{
  BRepBuilderAPI_MakePolygon aMP;
  for (size_t i = 0; i < theXYZ.size(); i += 3)
  {
    aMP.Add(gp_Pnt(theXYZ[i], theXYZ[i + 1], theXYZ[i + 2]));
  }
  if (theClosed)
  {
    aMP.Close();
  }
  return aMP.Wire();
}

static double Area(const TopoDS_Shape& theShape)
{
  GProp_GProps aProps;
  BRepGProp::SurfaceProperties(theShape, aProps);
  return aProps.Mass();
}

static double Volume(const TopoDS_Shape& theShape)
{
  GProp_GProps aProps;
  BRepGProp::VolumeProperties(theShape, aProps);
  return aProps.Mass();
}

int main()
{
  const char* aNames[2] = {"square", "lshape"};
  TopoDS_Wire aSpines[2] = {Polygon(true, {0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0}),
                            Polygon(true, {0, 0, 0, 10, 0, 0, 10, 4, 0, 4, 4, 0, 4, 10, 0, 0, 10, 0})};
  for (double anOffset : {2.0, -1.5})
  {
    for (int s = 0; s < 2; s++)
    {
      TopoDS_Wire aProfile = Polygon(false, {0, anOffset, 0, 0, anOffset, 3});
      std::printf("%s %.17g ", aNames[s], anOffset);
      try
      {
        BRepOffsetAPI_MakeEvolved aMaker(aSpines[s], aProfile, GeomAbs_Arc, true, true);
        if (!aMaker.IsDone())
        {
          std::printf("notdone\n");
          continue;
        }
        const TopoDS_Shape& aResult = aMaker.Shape();
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
        TopExp::MapShapes(aResult, TopAbs_FACE, aFaces);
        std::printf("valid %d volume %.17g bottom %.17g top %.17g faces %d\n",
                    BRepCheck_Analyzer(aResult).IsValid() ? 1 : 0,
                    Volume(aResult),
                    Area(aMaker.Bottom()),
                    Area(aMaker.Top()),
                    aFaces.Extent());
      }
      catch (const Standard_Failure& e)
      {
        std::printf("exception %s\n", e.ExceptionType());
      }
    }
  }
  return 0;
}
