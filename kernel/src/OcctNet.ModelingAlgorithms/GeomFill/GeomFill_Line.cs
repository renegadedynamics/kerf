// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The line of the approximations <see cref="GeomFill_AppSurf"/> and GeomFill_AppSweep: NbPoints sections, point
/// <c>i</c> being the section index <c>i</c>.
/// </summary>
public class GeomFill_Line : Transient, IAppBlend_Line<int>
{
    private readonly int myNbPoints;

    public GeomFill_Line()
    {
        myNbPoints = 0;
    }

    public GeomFill_Line(int NbPoints)
    {
        myNbPoints = NbPoints;
    }

    public int NbPoints() => myNbPoints;

    public int Point(int Index) => Index;
}
