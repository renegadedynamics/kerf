// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReaderModule.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReaderModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Defines unitary operations required to read an Entity from a File (see FileReaderData, FileReaderTool),
/// under control of a FileReaderTool. The initial creation is performed by a GeneralModule (set in
/// GeneralLib). Then, which remains is Loading data from the FileReaderData to the Entity To work, a
/// GeneralModule has formerly recognized the Type read from FileReaderData as a positive Case Number, then
/// the ReaderModule reads it according to this Case Number
/// </summary>
public abstract class Interface_ReaderModule : Standard_Transient
{

    /// <summary>
    /// Translates the type of record &lt;num&gt; in &lt;data&gt; to a positive Case Number. If Recognition
    /// fails, must return 0
    /// </summary>
    public abstract int CaseNum(Interface_FileReaderData? data, int num);

    /// <summary>
    /// Performs the effective loading from &lt;data&gt;, record &lt;num&gt;, to the Entity &lt;ent&gt; formerly
    /// created In case of Error or Warning, fills &lt;ach&gt; with messages Remark that the Case Number comes
    /// from translating a record
    /// </summary>
    public abstract void Read(int casenum, Interface_FileReaderData? data, int num, ref Interface_Check? ach, Standard_Transient? ent);

    /// <summary>
    /// Specific operator (create+read) defaulted to do nothing. It can be redefined when it is not possible to
    /// work in two steps (NewVoid then Read). This occurs when no default constructor is defined : hence the
    /// result &lt;ent&gt; must be created with an effective definition from the reader. Remark : if NewRead is
    /// defined, Copy has nothing to do. Returns True if it has produced something, false else. If nothing was
    /// produced, &lt;ach&gt; should be filled : it will be treated as "Unrecognized case" by reader tool.
    /// </summary>
    public virtual bool NewRead(int casenum, Interface_FileReaderData? data, int num, ref Interface_Check? ach, ref Standard_Transient? ent)
    {
        return false;
    }
}
