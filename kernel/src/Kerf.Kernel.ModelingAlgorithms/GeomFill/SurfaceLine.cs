// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The line of the approximations <see cref="SurfaceAppSurf"/> and GeomFill_AppSweep: NbPoints sections, point
/// <c>i</c> being the section index <c>i</c>.
/// </summary>
internal class SurfaceLine : Transient, ILine<int>
{
    private readonly int _nbPoints;

    public SurfaceLine()
    {
        _nbPoints = 0;
    }

    public SurfaceLine(int nbPoints)
    {
        _nbPoints = nbPoints;
    }

    public int NbPoints() => _nbPoints;

    public int Point(int index) => index;
}
