// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// stlrec: records native OCCT's STL reading and writing (RWStl, StlAPI; unit 8H1), which build123d.NET doesn't bind.
// Built against build123d.NET's native OCCT V8_0_1 (tools/stlrec/build.ps1).
//
// Inputs: one per line in the file given as the first argument (chosen by build.ps1); the recording goes to the file
// given as the second argument, and scratch files (the derived inputs, the STL written) to the directory given as the
// third. Hashes are SHA-256 in lower-case hex; a triangulation's hash is that of its dump: per node
// "%016llx %016llx %016llx\n" (the coordinates' bits), then per triangle "%d %d %d\n".
//
//   W <brep>                  read the BRep file, mesh it (BRepMesh_IncrementalMesh(s, 0.1, relative, 0.5, serial)):
//     W <name>
//     ASCII <len> <sha>       StlAPI_Writer to a std::ostringstream, ASCII
//     BINARY <len> <sha>      the same, binary
//     BACKA <tri>             RWStl::ReadStream of the ASCII bytes (merge angle 90 degrees)
//     BACKB <tri>             RWStl::ReadStream of the binary bytes
//   M                         a hand-made triangulation (see Mesh()) written by RWStl::WriteAscii and WriteBinary:
//     M / ASCII <len> <sha> / BINARY <len> <sha>
//   R <stl>                   read an STL file as is:          R <name>
//   D <recipe> <stl> [<arg>]  read data derived from one:     D <recipe> <name> [<arg>]
//     recipes: cat2 (the file twice), ws (the file, "\n \t\r\n", the file), trunc n (the first n bytes),
//     solidhdr (bytes 0-5 replaced by "solid "), lie k (the binary facet count plus k), crlf (LF replaced by CR LF)
//   then for both:
//     FILE <90|45|0> <tri>    RWStl::ReadFile(path, angle)
//     MULTI <n>               RWStl::ReadFile(path, 90 degrees, list): n lines <tri>
//     READER <ok> <nodes> <tris> <solids>   an RWStl_Reader's Read(path): its result and the calls it made
//     STREAM <tri>            RWStl::ReadStream(std::istringstream)
//     ASCIIS <tri>            RWStl::ReadAsciiStream
//     BINS <tri>              RWStl::ReadBinaryStream
//     SHAPE <len> <sha>       StlAPI_Reader::Read(shape, path), the shape's BRep text (BRepTools::Write), or FAIL
//     SHAPES <len> <sha>      StlAPI_Reader::Read(shape, istream), or FAIL (both for data up to 300 kB)
//   <tri> is "<nodes> <triangles> <sha>" or NULL; THROW <type> in place of a line if the call threw.
//
//   stlrec inputs.txt stlrec.txt <scratch dir>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#define NOMINMAX
#include <windows.h>
#include <bcrypt.h>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <NCollection_Sequence.hxx>
#include <OSD.hxx>
#include <OSD_Path.hxx>
#include <Poly_Triangulation.hxx>
#include <RWStl.hxx>
#include <RWStl_Reader.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <TopoDS_Shape.hxx>

static std::ofstream gOut;
static std::string   gScratch;

static std::string Sha256(const std::string& theData)
{
  unsigned char aHash[32];
  BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0, (PUCHAR)theData.data(), (ULONG)theData.size(), aHash, 32);
  char aHex[65];
  for (int i = 0; i < 32; ++i)
  {
    std::snprintf(aHex + 2 * i, 3, "%02x", aHash[i]);
  }
  return std::string(aHex, 64);
}

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

// "<nodes> <triangles> <sha>" or NULL
static std::string Tri(const occ::handle<Poly_Triangulation>& theTri)
{
  if (theTri.IsNull())
  {
    return "NULL";
  }
  std::string aDump;
  for (int i = 1; i <= theTri->NbNodes(); ++i)
  {
    const gp_Pnt aP = theTri->Node(i);
    aDump += Hex(aP.X()) + " " + Hex(aP.Y()) + " " + Hex(aP.Z()) + "\n";
  }
  for (int i = 1; i <= theTri->NbTriangles(); ++i)
  {
    int n1, n2, n3;
    theTri->Triangle(i).Get(n1, n2, n3);
    char aBuf[64];
    std::snprintf(aBuf, sizeof(aBuf), "%d %d %d\n", n1, n2, n3);
    aDump += aBuf;
  }
  return std::to_string(theTri->NbNodes()) + " " + std::to_string(theTri->NbTriangles()) + " " + Sha256(aDump);
}

static std::string Bytes(const std::string& theData)
{
  return std::to_string(theData.size()) + " " + Sha256(theData);
}

static std::string ReadAll(const std::string& thePath)
{
  std::ifstream aFile(thePath, std::ios::binary);
  std::ostringstream aStr;
  aStr << aFile.rdbuf();
  return aStr.str();
}

static void WriteAll(const std::string& thePath, const std::string& theData)
{
  std::ofstream aFile(thePath, std::ios::binary);
  aFile.write(theData.data(), (std::streamsize)theData.size());
}

static std::string FileName(const std::string& thePath)
{
  size_t aSlash = thePath.find_last_of("/\\");
  return aSlash == std::string::npos ? thePath : thePath.substr(aSlash + 1);
}

// Runs the call, writing "<head> <its line>", or "THROW <type>" if it throws
template <typename F>
static void Line(const std::string& theHead, F theCall)
{
  std::string aLine;
  try
  {
    OCC_CATCH_SIGNALS
    aLine = theHead + " " + theCall();
  }
  catch (const Standard_Failure& e)
  {
    aLine = std::string("THROW ") + typeid(e).name();
  }
  catch (const std::exception& e)
  {
    aLine = std::string("THROW ") + typeid(e).name();
  }
  gOut << aLine << "\n";
}

// An RWStl_Reader counting its calls
class CountingReader : public RWStl_Reader
{
public:
  int AddNode(const gp_XYZ&) override { return ++myNbNodes; }

  void AddTriangle(int, int, int) override { ++myNbTris; }

  void AddSolid() override { ++myNbSolids; }

  int myNbNodes = 0, myNbTris = 0, myNbSolids = 0;
};

static std::string ShapeText(const TopoDS_Shape& theShape)
{
  std::ostringstream aStr;
  BRepTools::Write(theShape, aStr);
  return aStr.str();
}

// The derived data of a recipe
static std::string Derive(const std::string& theRecipe, const std::string& theData, const std::string& theArg)
{
  if (theRecipe == "cat2")
  {
    return theData + theData;
  }
  if (theRecipe == "ws")
  {
    return theData + "\n \t\r\n" + theData;
  }
  if (theRecipe == "trunc")
  {
    return theData.substr(0, std::min(theData.size(), (size_t)std::stoul(theArg)));
  }
  if (theRecipe == "solidhdr")
  {
    std::string aData = theData;
    std::memcpy(&aData[0], "solid ", 6);
    return aData;
  }
  if (theRecipe == "lie")
  {
    std::string aData = theData;
    int32_t     aCount;
    std::memcpy(&aCount, &aData[80], 4);
    aCount += std::stoi(theArg);
    std::memcpy(&aData[80], &aCount, 4);
    return aData;
  }
  if (theRecipe == "crlf")
  {
    std::string aData;
    for (char c : theData)
    {
      if (c == '\n')
      {
        aData += '\r';
      }
      aData += c;
    }
    return aData;
  }
  std::fprintf(stderr, "unknown recipe %s\n", theRecipe.c_str());
  std::exit(3);
}

static void ReadCase(const std::string& thePath, const std::string& theData)
{
  const char* aPath = thePath.c_str();
  const double anAngles[3] = {M_PI / 2.0, M_PI / 4.0, 0.0};
  const char*  aTags[3]    = {"90", "45", "0"};
  for (int i = 0; i < 3; ++i)
  {
    Line(std::string("FILE ") + aTags[i], [&] { return Tri(RWStl::ReadFile(aPath, anAngles[i])); });
  }
  {
    std::string aLine;
    try
    {
      OCC_CATCH_SIGNALS
      NCollection_Sequence<occ::handle<Poly_Triangulation>> aList;
      RWStl::ReadFile(aPath, M_PI / 2.0, aList);
      aLine = "MULTI " + std::to_string(aList.Length());
      for (int i = 1; i <= aList.Length(); ++i)
      {
        aLine += "\n" + Tri(aList.Value(i));
      }
    }
    catch (const Standard_Failure& e)
    {
      aLine = std::string("THROW ") + typeid(e).name();
    }
    gOut << aLine << "\n";
  }
  Line("READER", [&] {
    CountingReader aReader;
    bool           isOk = aReader.Read(aPath, Message_ProgressRange());
    return std::to_string(isOk ? 1 : 0) + " " + std::to_string(aReader.myNbNodes) + " "
           + std::to_string(aReader.myNbTris) + " " + std::to_string(aReader.myNbSolids);
  });
  Line("STREAM", [&] {
    std::istringstream aStream(theData);
    return Tri(RWStl::ReadStream(aStream));
  });
  Line("ASCIIS", [&] {
    std::istringstream aStream(theData);
    return Tri(RWStl::ReadAsciiStream(aStream));
  });
  Line("BINS", [&] {
    std::istringstream aStream(theData);
    return Tri(RWStl::ReadBinaryStream(aStream));
  });
  if (theData.size() <= 300000)
  {
    Line("SHAPE", [&]() -> std::string {
      TopoDS_Shape  aShape;
      StlAPI_Reader aReader;
      if (!aReader.Read(aShape, aPath))
      {
        return "FAIL";
      }
      return Bytes(ShapeText(aShape));
    });
    Line("SHAPES", [&]() -> std::string {
      TopoDS_Shape       aShape;
      StlAPI_Reader      aReader;
      std::istringstream aStream(theData);
      if (!aReader.Read(aShape, aStream))
      {
        return "FAIL";
      }
      return Bytes(ShapeText(aShape));
    });
  }
}

// A hand-made triangulation: a degenerate triangle (normal 0 0 0), negative zero, a denormal, large and inexact values
static occ::handle<Poly_Triangulation> Mesh()
{
  occ::handle<Poly_Triangulation> aMesh = new Poly_Triangulation(7, 5, false);
  aMesh->SetNode(1, gp_Pnt(0.0, 0.0, 0.0));
  aMesh->SetNode(2, gp_Pnt(1.0, 0.0, 0.0));
  aMesh->SetNode(3, gp_Pnt(0.0, 1.0, 0.0));
  aMesh->SetNode(4, gp_Pnt(-0.0, 1e-310, 1e300));
  aMesh->SetNode(5, gp_Pnt(1.0 / 3.0, 2.0 / 3.0, -1e-5));
  aMesh->SetNode(6, gp_Pnt(0.5, 0.5, 0.0));
  aMesh->SetNode(7, gp_Pnt(123456.789, -9.87654321e-7, 5e-324));
  aMesh->SetTriangle(1, Poly_Triangle(1, 2, 3));
  aMesh->SetTriangle(2, Poly_Triangle(1, 2, 2));
  aMesh->SetTriangle(3, Poly_Triangle(4, 5, 6));
  aMesh->SetTriangle(4, Poly_Triangle(3, 2, 1));
  aMesh->SetTriangle(5, Poly_Triangle(5, 6, 7));
  return aMesh;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::fprintf(stderr, "usage: stlrec inputs.txt out.txt scratchdir\n");
    return 2;
  }
  OSD::SetSignal(false);
  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  gScratch = argv[3];
  std::string aLineIn;
  int         aDerived = 0;
  while (std::getline(aList, aLineIn))
  {
    if (!aLineIn.empty() && aLineIn.back() == '\r')
    {
      aLineIn.pop_back();
    }
    if (aLineIn.empty())
    {
      continue;
    }
    std::istringstream aWords(aLineIn);
    std::string        aKind;
    aWords >> aKind;
    if (aKind == "W")
    {
      std::string aPath;
      std::getline(aWords >> std::ws, aPath);
      TopoDS_Shape aShape;
      BRep_Builder aB;
      if (!BRepTools::Read(aShape, aPath.c_str(), aB) || aShape.IsNull())
      {
        continue;
      }
      gOut << "W " << FileName(aPath) << "\n";
      BRepMesh_IncrementalMesh aMesher(aShape, 0.1, true, 0.5, false);
      std::string anAscii, aBinary;
      Line("ASCII", [&] {
        std::ostringstream aStream;
        StlAPI_Writer      aWriter;
        aWriter.ASCIIMode() = true;
        aWriter.Write(aShape, aStream);
        anAscii = aStream.str();
        return Bytes(anAscii);
      });
      Line("BINARY", [&] {
        std::ostringstream aStream(std::ios::out | std::ios::binary);
        StlAPI_Writer      aWriter;
        aWriter.ASCIIMode() = false;
        aWriter.Write(aShape, aStream);
        aBinary = aStream.str();
        return Bytes(aBinary);
      });
      WriteAll(gScratch + "\\" + FileName(aPath) + ".a.stl", anAscii);
      WriteAll(gScratch + "\\" + FileName(aPath) + ".b.stl", aBinary);
      Line("BACKA", [&] {
        std::istringstream aStream(anAscii);
        return Tri(RWStl::ReadStream(aStream));
      });
      Line("BACKB", [&] {
        std::istringstream aStream(aBinary);
        return Tri(RWStl::ReadStream(aStream));
      });
    }
    else if (aKind == "M")
    {
      gOut << "M\n";
      occ::handle<Poly_Triangulation> aMesh = Mesh();
      Line("ASCII", [&] {
        std::ostringstream aStream;
        RWStl::WriteAscii(aMesh, aStream);
        WriteAll(gScratch + "\\M.a.stl", aStream.str());
        return Bytes(aStream.str());
      });
      Line("BINARY", [&] {
        std::ostringstream aStream;
        RWStl::WriteBinary(aMesh, aStream);
        WriteAll(gScratch + "\\M.b.stl", aStream.str());
        return Bytes(aStream.str());
      });
    }
    else if (aKind == "R")
    {
      std::string aPath;
      std::getline(aWords >> std::ws, aPath);
      gOut << "R " << FileName(aPath) << "\n";
      ReadCase(aPath, ReadAll(aPath));
    }
    else if (aKind == "D")
    {
      std::string aRecipe, aPath, anArg;
      aWords >> aRecipe >> aPath;
      aWords >> anArg;
      std::string aData = Derive(aRecipe, ReadAll(aPath), anArg);
      std::string aDerivedPath = gScratch + "\\derived" + std::to_string(++aDerived) + ".stl";
      WriteAll(aDerivedPath, aData);
      gOut << "D " << aRecipe << " " << FileName(aPath) << (anArg.empty() ? "" : " " + anArg) << "\n";
      ReadCase(aDerivedPath, aData);
    }
    gOut.flush();
  }
  return 0;
}
