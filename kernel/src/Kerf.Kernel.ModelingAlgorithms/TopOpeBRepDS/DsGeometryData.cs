// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_GeometryData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_GeometryData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>The list of interferences attached to a geometry of the data structure (base of the surface, curve and point data).</summary>
/// <remarks>Storage of <see cref="DsDataStructure"/>; a working object.</remarks>
public class DsGeometryData
{
    private readonly NCollection_List<DsInterference> _interferences = new();

    /// <summary>An empty list of interferences.</summary>
    public DsGeometryData()
    {
    }

    /// <summary>A copy of <paramref name="other"/>'s list (the interferences themselves are shared).</summary>
    public DsGeometryData(DsGeometryData other)
    {
        Assign(other);
    }

    internal void Assign(DsGeometryData other)
    {
        _interferences.Clear();
        NCollection_List<DsInterference>.Iterator it = new(other._interferences);
        for (; it.More(); it.Next())
        {
            _interferences.Append(it.Value());
        }
    }

    /// <summary>The interferences.</summary>
    /// <remarks>The list itself, which the data structure's builders change; callers outside OCCT.NET only read it.</remarks>
    public NCollection_List<DsInterference> Interferences() => _interferences;

    internal NCollection_List<DsInterference> ChangeInterferences() => _interferences;

    internal void AddInterference(DsInterference i)
    {
        _interferences.Append(i);
    }
}
