// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ActorWrite.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ActorWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Extends ActorWrite from STEPControl by analysis of whether shape is assembly (based on information from DECAF)
/// </summary>
public class STEPCAFControl_ActorWrite : STEPControl_ActorWrite
{
    public STEPCAFControl_ActorWrite()
    {
        myStdMode = true;
    }

    /// <summary>
    /// Check whether shape S is assembly Returns True if shape is registered in assemblies map
    /// </summary>
    public override bool IsAssembly(StepData_StepModel theModel, ref Shape S)
    {
        if (myStdMode)
        {
            return base.IsAssembly(theModel, ref S);
        }
        return myMap.Contains(S);
    }

    /// <summary>
    /// Set standard mode of work In standard mode Actor (default) behaves exactly as its ancestor, also map is
    /// cleared
    /// </summary>
    public void SetStdMode(bool stdmode = true)
    {
        myStdMode = stdmode;
        if (myStdMode)
        {
            ClearMap();
        }
    }

    /// <summary>Clears map of shapes registered as assemblies</summary>
    public void ClearMap()
    {
        myMap.Clear();
    }

    /// <summary>Registers shape to be written as assembly The shape should be TopoDS_Compound (else does nothing)</summary>
    public void RegisterAssembly(Shape S)
    {
        if (!myStdMode && S.ShapeType() == ShapeType.Compound)
        {
            myMap.Add(S);
        }
    }

    private bool myStdMode;
    private readonly NCollection_Map<Shape> myMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
}
