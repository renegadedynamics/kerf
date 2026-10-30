# Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
#
# Copies OCCT's walking sources into <out>/src and <out>/inc and instruments them for walkrec (decision 687): every
# return of IntWalk_IWalking::TestDeflection and IntWalk_PWalking::TestDeflection prints its status and point. The
# original bodies become TestDeflection_Impl, a non-virtual member added to copies of the headers (the classes' layout
# is unchanged).
#
#   python instrument.py <occt-src> <occt-include> <out>

import pathlib
import shutil
import sys

occt, include, out = (pathlib.Path(a) for a in sys.argv[1:4])
algo = occt / "src" / "ModelingAlgorithms" / "TKGeomAlgo"
(out / "src").mkdir(parents=True, exist_ok=True)
(out / "inc").mkdir(parents=True, exist_ok=True)
for f in ["IntWalk/IntWalk_IWalking.gxx", "IntWalk/IntWalk_PWalking.cxx", "IntPatch/IntPatch_TheIWalking_0.cxx",
          "IntPatch/IntPatch_ImpPrmIntersection.cxx", "IntPatch/IntPatch_PrmPrmIntersection.cxx"]:
    shutil.copy(algo / f, out / "src")
for f in ["IntPatch_TheIWalking.hxx", "IntWalk_PWalking.hxx"]:
    shutil.copy(include / f, out / "inc")


def patch(path, old, new):
    text = path.read_text()
    if old not in text:
        raise SystemExit(f"{path.name}: anchor not found: {old!r}")
    path.write_text(text.replace(old, new, 1))


patch(out / "inc" / "IntPatch_TheIWalking.hxx", "  Standard_EXPORT IntWalk_StatusDeflection\n    TestDeflection(",
      "  IntWalk_StatusDeflection TestDeflection_Impl(IntPatch_TheSurfFunction& Section, const bool Finished,"
      " const math_Vector& UV, const IntWalk_StatusDeflection StatusPrecedent, int& NbDivision, double& Step,"
      " const int StepSign);\n  Standard_EXPORT IntWalk_StatusDeflection\n    TestDeflection(")
patch(out / "src" / "IntWalk_IWalking.gxx", "IntWalk_StatusDeflection IntWalk_IWalking::TestDeflection(\n", """\
extern void RecPnt(const char* tag, int status, const IntSurf_PntOn2S& p);
IntWalk_StatusDeflection IntWalk_IWalking::TestDeflection(TheIWFunction& sp, const bool Finished, const math_Vector& UV,
  const IntWalk_StatusDeflection StatusPrecedent, int& NbDivision, double& Step, const int StepSign)
{
  IntWalk_StatusDeflection st = TestDeflection_Impl(sp, Finished, UV, StatusPrecedent, NbDivision, Step, StepSign);
  IntSurf_PntOn2S p;
  p.SetValue(sp.Point(), reversed, UV(1), UV(2));
  RecPnt("S", (int)st, p);
  return st;
}
IntWalk_StatusDeflection IntWalk_IWalking::TestDeflection_Impl(
""")
patch(out / "inc" / "IntWalk_PWalking.hxx",
      "  Standard_EXPORT IntWalk_StatusDeflection TestDeflection(const IntImp_ConstIsoparametric ChoixIso,",
      "  IntWalk_StatusDeflection TestDeflection_Impl(const IntImp_ConstIsoparametric ChoixIso,"
      " const IntWalk_StatusDeflection theStatus);\n"
      "  Standard_EXPORT IntWalk_StatusDeflection TestDeflection(const IntImp_ConstIsoparametric ChoixIso,")
patch(out / "src" / "IntWalk_PWalking.cxx",
      "IntWalk_StatusDeflection IntWalk_PWalking::TestDeflection(const IntImp_ConstIsoparametric choixIso,", """\
extern void RecPnt(const char* tag, int status, const IntSurf_PntOn2S& p);
IntWalk_StatusDeflection IntWalk_PWalking::TestDeflection(const IntImp_ConstIsoparametric choixIso,
                                                          const IntWalk_StatusDeflection  theStatus)
{
  IntWalk_StatusDeflection st = TestDeflection_Impl(choixIso, theStatus);
  RecPnt("S", (int)st, myIntersectionOn2S.Point());
  return st;
}
IntWalk_StatusDeflection IntWalk_PWalking::TestDeflection_Impl(const IntImp_ConstIsoparametric choixIso,""")
