// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// The data the data structure keeps for a shape: its interferences, its same-domain shapes with their reference,
/// orientation and index, its ancestor rank, and whether it is kept.
/// </summary>
/// <remarks>Storage of <see cref="DsDataStructure"/> (a class, so its map gives it for writing).</remarks>
internal sealed class DsShapeData
{
    internal readonly NCollection_List<DsInterference> _interferences = new();
    internal readonly NCollection_List<Shape> _sameDomain = new();
    internal int _sameDomainRef;
    internal DsConfig _sameDomainOri;
    internal int _sameDomainInd;
    internal Orientation _orientation;
    internal bool _orientationDef;
    internal int _ancestorRank;
    private bool _keep;

    /// <summary>Empty data: no interferences, no same domain, kept.</summary>
    public DsShapeData()
    {
        _sameDomainRef = 0;
        _sameDomainOri = DsConfig.Unshgeometry;
        _sameDomainInd = 0;
        _orientation = Forward;
        _orientationDef = false;
        _ancestorRank = 0;
        _keep = true;
    }

    /// <summary>The interferences.</summary>
    public NCollection_List<DsInterference> Interferences() => _interferences;

    internal NCollection_List<DsInterference> ChangeInterferences() => _interferences;

    /// <summary>Whether the shape is kept.</summary>
    public bool Keep() => _keep;

    internal void ChangeKeep(bool b)
    {
        _keep = b;
    }
}
