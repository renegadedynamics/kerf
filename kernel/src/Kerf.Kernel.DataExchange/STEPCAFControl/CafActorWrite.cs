// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ActorWrite.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ActorWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Documents;

/// <summary>
/// Extends ActorWrite from STEPControl by analysis of whether shape is assembly (based on information from DECAF)
/// </summary>
internal class CafActorWrite : ActorWrite
{
    public CafActorWrite()
    {
        _stdMode = true;
    }

    /// <summary>
    /// Check whether shape S is assembly Returns True if shape is registered in assemblies map
    /// </summary>
    public override bool IsAssembly(StepModel model, ref Shape s)
    {
        if (_stdMode)
        {
            return base.IsAssembly(model, ref s);
        }
        return _map.Contains(s);
    }

    /// <summary>
    /// Set standard mode of work In standard mode Actor (default) behaves exactly as its ancestor, also map is
    /// cleared
    /// </summary>
    public void SetStdMode(bool stdmode = true)
    {
        _stdMode = stdmode;
        if (_stdMode)
        {
            ClearMap();
        }
    }

    /// <summary>Clears map of shapes registered as assemblies</summary>
    public void ClearMap()
    {
        _map.Clear();
    }

    /// <summary>Registers shape to be written as assembly The shape should be TopoDS_Compound (else does nothing)</summary>
    public void RegisterAssembly(Shape s)
    {
        if (!_stdMode && s.Type == ShapeType.Compound)
        {
            _map.Add(s);
        }
    }

    private bool _stdMode;
    private readonly HashSet<Shape> _map = new HashSet<Shape>(ShapeHasher.Instance);
}
