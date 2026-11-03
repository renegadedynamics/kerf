// Port of: src/DataExchange/TKDESTEP/StepData/StepData_DefaultGeneral.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_DefaultGeneral.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// DefaultGeneral defines a GeneralModule which processes Unknown Entity from StepData only.
/// </summary>
/// <remarks>Nothing in OCCT creates one: undefined entities have no general module unless an application makes it.</remarks>
public class StepData_DefaultGeneral : StepData_GeneralModule
{
    //  StepData DefaultGeneral recognizes ONLY ONE TYPE: UndefinedEntity

    /// <summary>Creates a Default General Module (registered with the StepData protocol: process-wide, decision 1182).</summary>
    public StepData_DefaultGeneral()
    {
        // Register this module globally with the StepData protocol
        Interface_GeneralLib.SetGlobal(this, StepData.Protocol());
    }

    /// <summary>Specific filling of the list of Entities shared by an Entity ent, which is an UnknownEntity from StepData.</summary>
    public override void FillSharedCase(int casenum, Transient? ent, Interface_EntityIterator iter)
    {
        // Fill iterator with shared entities from UndefinedEntity parameters
        if (casenum != 1)
        {
            return; // Only handles case 1 (UndefinedEntity)
        }
        StepData_UndefinedEntity undf = ent as StepData_UndefinedEntity;
        Interface_UndefinedContent cont = undf.UndefinedContent();
        int nb = cont.NbParams();
        // Iterate through all parameters looking for entity references
        for (int i = 1; i <= nb; i++)
        {
            Interface_ParamType ptype = cont.ParamType(i);
            if (ptype == Interface_ParamType.Interface_ParamSub)
            {
                // Handle sub-entity parameters recursively
                FillSharedCase(casenum, cont.ParamEntity(i), iter);
            }
            else if (ptype == Interface_ParamType.Interface_ParamIdent)
            {
                // Handle entity identifier parameters
                iter.GetOneItem(cont.ParamEntity(i));
            }
        }
    }

    /// <summary>Specific Checking of an Entity ent.</summary>
    public override void CheckCase(int casenum, Transient? ent, Interface_ShareTool shares, ref Interface_Check? ach)
    {
    } //  No validation check performed on an UndefinedEntity

    /// <summary>Specific creation of a new void entity.</summary>
    public override bool NewVoid(int CN, out Transient? ent)
    {
        ent = null;
        // Create a new empty entity instance (only UndefinedEntity supported)
        if (CN != 1)
        {
            return false; // Only case 1 supported
        }
        ent = new StepData_UndefinedEntity();
        return true;
    }

    /// <summary>Specific Copy ("Deep") from entfrom to entto (same type) by using a CopyTool which provides its working Map.</summary>
    public override void CopyCase(int casenum, Transient? entfrom, Transient? entto, Interface_CopyTool TC)
    {
        // Copy content from source UndefinedEntity to target UndefinedEntity
        if (casenum != 1)
        {
            return; // Only handles case 1 (UndefinedEntity)
        }
        StepData_UndefinedEntity undfrom = entfrom as StepData_UndefinedEntity;
        StepData_UndefinedEntity undto = entto as StepData_UndefinedEntity;
        undto.GetFromAnother(undfrom, TC); //  We could optimize this operation
    }
}
