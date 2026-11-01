// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// helixrec: records native OCCT's TKHelix (unit 7D), which build123d.NET doesn't bind: HelixBRep_BuilderHelix,
// HelixGeom_BuilderHelix, HelixGeom_BuilderHelixCoil and HelixGeom_Tools::ApprHelix, on the parameters of the corpus's
// helix grid (tools/helixrec/cases.txt: each helix command the grid's cases call, with its arguments as Draw passes
// them). Built against build123d.NET's native OCCT V8_0_1 (tools/helixrec/build.ps1).
//
// Per command line "<case id> <command> <name> <args...>" of cases.txt:
//   CASE <case id> <command>
//   AXIS [<9 hex>]                      the arguments of the case's last setaxis, which the axis replays from OCCT's
//                                       default (none: the default axis)
//   IN <np> <hex...>                    the command's numbers after np, in its order (Draw::Atof of the arguments)
//   BREP <set> <err> <warn> <tol> <n> <hash>
//                                       HelixBRep_BuilderHelix through the command's SetParameters overload, with the
//                                       approximation parameters of the set (0: the defaults, as the commands; 1:
//                                       1e-5, degree 8, C2): the wire's BRep text (format 3; empty on an error) has n
//                                       lines and the hash (FNV-1a, 64 bits, of its bytes); the n lines follow when n
//                                       is at most 250 (a newline added to the last); or THROW <type>
//   For each part i (1..np) of the command, with D the diameters as given, H and P the parts' heights and pitches:
//   GEOM <i> <t1> <t2> <pitch> <r> <taper> <cw>   the inputs (hex; cw 0 or 1): t1 0, t2 2 pi H/P, r D(i)/2, taper
//                                       atan((D(i+1) - D(i)) / 2H) (D's last value past its end), cw for odd parts
//   HELIX <err> <tol> <nc>              HelixGeom_BuilderHelix at the axis (Ax2), tolerance 1e-4, C1, degree 8, 1000
//                                       segments; then nc curves as CURVE lines
//   COIL <err> <tol> <nc>               HelixGeom_BuilderHelixCoil on min(t2, 2 pi), the default parameters
//   APPR <err> <maxerr> <nc>            HelixGeom_Tools::ApprHelix on min(t2, 2 pi), tolerance 1e-4
//   CURVE <deg> <np> <nk> <poles x y z (hex)...> <knots (hex)...> <mults...>   the first two and the last curve
//   CURVEH <hash>                       the others: the hash of what CURVE would write after "CURVE "
//   THROW <type>                        in place of a line above if the call threw
//
//   helixrec cases.txt helixrec.txt
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <BRepTools.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Curve.hxx>
#include <HelixBRep_BuilderHelix.hxx>
#include <HelixGeom_BuilderHelix.hxx>
#include <HelixGeom_BuilderHelixCoil.hxx>
#include <HelixGeom_Tools.hxx>
#include <NCollection_Array1.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax3.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::ofstream gOut;

static void Throw(const Standard_Failure& e)
{
  gOut << "THROW " << e.ExceptionType() << "\n";
}

// FNV-1a, 64 bits, of the text's bytes
static std::string Fnv(const std::string& theText)
{
  uint64_t aH = 14695981039346656037ull;
  for (unsigned char c : theText)
  {
    aH = (aH ^ c) * 1099511628211ull;
  }
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aH);
  return aBuf;
}

// "<deg> <np> <nk> <poles...> <knots...> <mults...>"
static std::string CurveText(const occ::handle<Geom_Curve>& theC)
{
  occ::handle<Geom_BSplineCurve> aB = occ::down_cast<Geom_BSplineCurve>(theC);
  if (aB.IsNull())
  {
    return "none";
  }
  std::ostringstream aS;
  aS << aB->Degree() << " " << aB->NbPoles() << " " << aB->NbKnots();
  for (int i = 1; i <= aB->NbPoles(); ++i)
  {
    const gp_Pnt& aP = aB->Pole(i);
    aS << " " << Hex(aP.X()) << " " << Hex(aP.Y()) << " " << Hex(aP.Z());
  }
  for (int i = 1; i <= aB->NbKnots(); ++i)
  {
    aS << " " << Hex(aB->Knot(i));
  }
  for (int i = 1; i <= aB->NbKnots(); ++i)
  {
    aS << " " << aB->Multiplicity(i);
  }
  return aS.str();
}

// The k-th of n curves: in full (CURVE) for the first two and the last, its text's hash (CURVEH) for the others.
static void Curve(const occ::handle<Geom_Curve>& theC, int theK, int theN)
{
  std::string aText = CurveText(theC);
  if (theK <= 2 || theK == theN)
  {
    gOut << "CURVE " << aText << "\n";
  }
  else
  {
    gOut << "CURVEH " << Fnv(aText) << "\n";
  }
}

static NCollection_Array1<double> Array(const std::vector<double>& theV, size_t theFrom, int theNb)
{
  NCollection_Array1<double> anA(1, theNb);
  for (int i = 1; i <= theNb; ++i)
  {
    anA(i) = theV[theFrom + i - 1];
  }
  return anA;
}

static NCollection_Array1<bool> Flags(const std::vector<double>& theV, size_t theFrom, int theNb)
{
  NCollection_Array1<bool> anA(1, theNb);
  for (int i = 1; i <= theNb; ++i)
  {
    // Draw::Atoi(a) != 0
    anA(i) = (int)theV[theFrom + i - 1] != 0;
  }
  return anA;
}

// The parts' diameters as given, heights and pitches (unit 7D's GEOM inputs).
struct Parts
{
  std::vector<double> D, H, P;
};

static void SetParameters(HelixBRep_BuilderHelix&    theB,
                          const std::string&         theCmd,
                          const gp_Ax3&              theAxis,
                          int                        theNb,
                          const std::vector<double>& theV,
                          Parts&                     theParts)
{
  if (theCmd == "comphelix")
  {
    NCollection_Array1<double> aD = Array(theV, 0, theNb + 1), aH = Array(theV, theNb + 1, theNb),
                               aP = Array(theV, 2 * theNb + 1, theNb);
    NCollection_Array1<bool> aF = Flags(theV, 3 * theNb + 1, theNb);
    theParts.D.assign(theV.begin(), theV.begin() + theNb + 1);
    for (int i = 1; i <= theNb; ++i)
    {
      theParts.H.push_back(aH(i));
      theParts.P.push_back(aF(i) ? aP(i) : aH(i) / aP(i));
    }
    theB.SetParameters(theAxis, aD, aH, aP, aF);
  }
  else if (theCmd == "helix" || theCmd == "spiral")
  {
    int                        aNbD = theCmd == "helix" ? 1 : 2;
    NCollection_Array1<double> aH = Array(theV, aNbD, theNb), aP = Array(theV, aNbD + theNb, theNb);
    NCollection_Array1<bool>   aF = Flags(theV, aNbD + 2 * theNb, theNb);
    theParts.D.assign(theV.begin(), theV.begin() + aNbD);
    for (int i = 1; i <= theNb; ++i)
    {
      theParts.H.push_back(aH(i));
      theParts.P.push_back(aF(i) ? aP(i) : aH(i) / aP(i));
    }
    if (aNbD == 1)
    {
      theB.SetParameters(theAxis, theV[0], aH, aP, aF);
    }
    else
    {
      theB.SetParameters(theAxis, theV[0], theV[1], aH, aP, aF);
    }
  }
  else
  {
    // comphelix2, helix2, spiral2: pitches and numbers of turns
    int aNbD = theCmd == "comphelix2" ? theNb + 1 : theCmd == "helix2" ? 1 : 2;
    NCollection_Array1<double> aP = Array(theV, aNbD, theNb), aN = Array(theV, aNbD + theNb, theNb);
    theParts.D.assign(theV.begin(), theV.begin() + aNbD);
    for (int i = 1; i <= theNb; ++i)
    {
      theParts.H.push_back(aP(i) * aN(i));
      theParts.P.push_back(aP(i));
    }
    if (theCmd == "comphelix2")
    {
      theB.SetParameters(theAxis, Array(theV, 0, theNb + 1), aP, aN);
    }
    else if (theCmd == "helix2")
    {
      theB.SetParameters(theAxis, theV[0], aP, aN);
    }
    else
    {
      theB.SetParameters(theAxis, theV[0], theV[1], aP, aN);
    }
  }
}

// BRep texts of up to MaxLines lines are recorded in full, longer ones by their hash only.
static const int MaxLines = 250;

static void RecordBRep(int                        theSet,
                       const std::string&         theCmd,
                       const gp_Ax3&              theAxis,
                       int                        theNb,
                       const std::vector<double>& theV,
                       Parts&                     theParts)
{
  try
  {
    OCC_CATCH_SIGNALS
    HelixBRep_BuilderHelix aB;
    SetParameters(aB, theCmd, theAxis, theNb, theV, theParts);
    if (theSet == 1)
    {
      aB.SetApproxParameters(1.e-5, 8, GeomAbs_C2);
    }
    aB.Perform();
    std::string aText;
    if (aB.ErrorStatus() == 0)
    {
      std::ostringstream aS;
      BRepTools::Write(aB.Shape(), aS, true, false, TopTools_FormatVersion_VERSION_3);
      aText = aS.str();
    }
    int aNb = 0;
    for (char c : aText)
    {
      aNb += c == '\n' ? 1 : 0;
    }
    if (!aText.empty() && aText.back() != '\n')
    {
      ++aNb;
    }
    gOut << "BREP " << theSet << " " << aB.ErrorStatus() << " " << aB.WarningStatus() << " "
         << Hex(aB.ToleranceReached()) << " " << aNb << " " << Fnv(aText) << "\n";
    if (aNb <= MaxLines)
    {
      // the text in full, a newline added if it has none at its end
      gOut << aText;
      if (!aText.empty() && aText.back() != '\n')
      {
        gOut << "\n";
      }
    }
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

static void RecordGeom(const gp_Ax3& theAxis, const Parts& theParts)
{
  const double aTwoPI = 2. * M_PI;
  for (size_t i = 0; i < theParts.H.size(); ++i)
  {
    const size_t aLast = theParts.D.size() - 1;
    double       aD1 = theParts.D[i < aLast ? i : aLast], aD2 = theParts.D[i + 1 < aLast ? i + 1 : aLast];
    double       aT2 = aTwoPI * theParts.H[i] / theParts.P[i];
    double       aR = 0.5 * aD1, aTaper = std::atan((aD2 - aD1) / (2. * theParts.H[i]));
    bool         aCW = i % 2 == 0;
    gOut << "GEOM " << i + 1 << " " << Hex(0.) << " " << Hex(aT2) << " " << Hex(theParts.P[i]) << " " << Hex(aR)
         << " " << Hex(aTaper) << " " << (aCW ? 1 : 0) << "\n";
    try
    {
      OCC_CATCH_SIGNALS
      HelixGeom_BuilderHelix aBH;
      aBH.SetPosition(theAxis.Ax2());
      aBH.SetCurveParameters(0., aT2, theParts.P[i], aR, aTaper, aCW);
      aBH.SetTolerance(1.e-4);
      aBH.SetApproxParameters(GeomAbs_C1, 8, 1000);
      aBH.Perform();
      gOut << "HELIX " << aBH.ErrorStatus() << " " << Hex(aBH.ToleranceReached()) << " " << aBH.Curves().Length()
           << "\n";
      for (int k = 1; k <= aBH.Curves().Length(); ++k)
      {
        Curve(aBH.Curves()(k), k, aBH.Curves().Length());
      }
    }
    catch (const Standard_Failure& e)
    {
      Throw(e);
    }
    double aT2c = aT2 < aTwoPI ? aT2 : aTwoPI;
    try
    {
      OCC_CATCH_SIGNALS
      HelixGeom_BuilderHelixCoil aBC;
      aBC.SetCurveParameters(0., aT2c, theParts.P[i], aR, aTaper, aCW);
      aBC.Perform();
      gOut << "COIL " << aBC.ErrorStatus() << " " << Hex(aBC.ToleranceReached()) << " " << aBC.Curves().Length()
           << "\n";
      for (int k = 1; k <= aBC.Curves().Length(); ++k)
      {
        Curve(aBC.Curves()(k), k, aBC.Curves().Length());
      }
    }
    catch (const Standard_Failure& e)
    {
      Throw(e);
    }
    try
    {
      OCC_CATCH_SIGNALS
      occ::handle<Geom_BSplineCurve> aBS;
      double                         aMaxErr = -1.;
      int iErr = HelixGeom_Tools::ApprHelix(0., aT2c, theParts.P[i], aR, aTaper, aCW, 1.e-4, aBS, aMaxErr);
      gOut << "APPR " << iErr << " " << Hex(aMaxErr) << " " << (aBS.IsNull() ? 0 : 1) << "\n";
      if (!aBS.IsNull())
      {
        Curve(aBS, 1, 1);
      }
    }
    catch (const Standard_Failure& e)
    {
      Throw(e);
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "usage: helixrec cases.txt out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  std::string         aLine, aCase;
  gp_Ax3              anAxis;
  std::vector<double> aSetAxis;
  while (std::getline(aList, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
    {
      aLine.pop_back();
    }
    std::istringstream       aS(aLine);
    std::string              anId, aCmd, aWord;
    std::vector<std::string> anArgs;
    aS >> anId >> aCmd;
    while (aS >> aWord)
    {
      anArgs.push_back(aWord);
    }
    if (anId.empty())
    {
      continue;
    }
    if (anId != aCase)
    {
      // a new case: a fresh DRAWEXE, OCCT's default axis
      aCase  = anId;
      anAxis = gp_Ax3(gp_Pnt(0., 0., 0), gp_Dir(gp_Dir::D::Z), gp_Dir(gp_Dir::D::X));
      aSetAxis.clear();
    }
    std::vector<double> aV;
    for (const std::string& anArg : anArgs)
    {
      aV.push_back(std::strtod(anArg.c_str(), nullptr));
    }
    if (aCmd == "setaxis")
    {
      anAxis.SetLocation(gp_Pnt(aV[0], aV[1], aV[2]));
      anAxis.SetDirection(gp_Dir(aV[3], aV[4], aV[5]));
      anAxis.SetXDirection(gp_Dir(aV[6], aV[7], aV[8]));
      aSetAxis = aV;
      continue;
    }
    // anArgs: name np values...
    int aNb = (int)aV[1];
    aV.erase(aV.begin(), aV.begin() + 2);
    gOut << "CASE " << anId << " " << aCmd << "\n";
    gOut << "AXIS";
    for (double c : aSetAxis)
    {
      gOut << " " << Hex(c);
    }
    gOut << "\n";
    gOut << "IN " << aNb;
    for (double v : aV)
    {
      gOut << " " << Hex(v);
    }
    gOut << "\n";
    Parts aParts;
    RecordBRep(0, aCmd, anAxis, aNb, aV, aParts);
    Parts anUnused;
    RecordBRep(1, aCmd, anAxis, aNb, aV, anUnused);
    if (!aParts.H.empty())
    {
      RecordGeom(anAxis, aParts);
    }
  }
  return 0;
}
