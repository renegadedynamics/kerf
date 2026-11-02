// Port of: src/DataExchange/TKDESTEP/StepData/StepData_GeneralModule.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_GeneralModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Specific features for General Services adapted to STEP.</summary>
public abstract class StepData_GeneralModule : Interface_GeneralModule
{
    /// <summary>Specific filling of the list of Entities shared by an Entity ent. Can use the internal utility method Share, below.</summary>
    public abstract override void FillSharedCase(int casenum, Standard_Transient? ent, Interface_EntityIterator iter);

    /// <summary>Specific Checking of an Entity ent.</summary>
    public abstract override void CheckCase(int casenum, Standard_Transient? ent, Interface_ShareTool shares, ref Interface_Check? ach);

    /// <summary>Specific Copy ("Deep") from entfrom to entto (same type) by using a TransferControl which provides its working Map. Use method Transferred from TransferControl to work.</summary>
    public abstract override void CopyCase(int casenum, Standard_Transient? entfrom, Standard_Transient? entto, Interface_CopyTool TC);
}
