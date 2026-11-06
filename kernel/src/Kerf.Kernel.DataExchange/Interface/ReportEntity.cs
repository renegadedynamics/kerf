// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReportEntity.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReportEntity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A ReportEntity is produced to acknowledge and memorize the binding between a Check and an Entity. The
/// Check can bring Fails (+ Warnings if any), or only Warnings. If it is empty, the Report Entity is for an
/// Unknown Entity. The ReportEntity brings : the Concerned Entity, the Check, and if the Entity is empty
/// (Fails due to Read Errors, hence the Entity could not be loaded), a Content. The Content is itself an
/// Transient Object, but remains in a literal form : it is an "Unknown Entity". If the Concerned Entity is
/// itself Unknown, Concerned and Content are equal. According to the Check, if it brings Fail messages, the
/// ReportEntity is an "Error Entity", the Concerned Entity is an "Erroneous Entity". Else it is a
/// "Correction Entity", the Concerned Entity is a "Corrected Entity". With no Check message and if Concerned
/// and Content are equal, it reports for an "Unknown Entity". Each norm must produce its own type of Unknown
/// Entity, but can use the class UndefinedContent to brings parameters : it is enough for most of
/// information and avoids to redefine them, only the specific part remains to be defined for each norm.
/// </summary>
internal class ReportEntity : Transient
{

    /// <summary>
    /// Creates a ReportEntity for an Unknown Entity : Check is empty, and Concerned equates Content (i.e. the
    /// Unknown Entity)
    /// </summary>
    public ReportEntity(Transient? unknown)
    {
        _theconcerned = unknown;
        _thecontent = unknown;
    }

    /// <summary>
    /// Creates a ReportEntity with its features : - &lt;acheck&gt; is the Check to be memorised -
    /// &lt;concerned&gt; is the Entity to which the Check is bound Later, a Content can be set : it is required
    /// for an Error
    /// </summary>
    public ReportEntity(FrameworkCheck? acheck, Transient? concerned)
    {
        _thecheck = acheck;
        _theconcerned = concerned;
        _thecheck.SetEntity(concerned);
    }

    /// <summary>
    /// Sets a Content : it brings non interpreted data which belong to the Concerned Entity. It can be empty
    /// then loaded later. Remark that for an Unknown Entity, Content is set by Create.
    /// </summary>
    public void SetContent(Transient? content)
    {
        _thecontent = content;
    }

    /// <summary>
    /// Returns the stored Check
    /// </summary>
    public FrameworkCheck? Check()
    {
        return _thecheck;
    }

    /// <summary>
    /// Returns the stored Check in order to change it
    /// </summary>
    public ref FrameworkCheck? CCheck()
    {
        return ref _thecheck;
    }

    /// <summary>
    /// Returns the stored Concerned Entity. It equates the Content in the case of an Unknown Entity
    /// </summary>
    public Transient? Concerned()
    {
        return _theconcerned;
    }

    /// <summary>
    /// Returns True if a Content is stored (it can equate Concerned)
    /// </summary>
    public bool HasContent
    {
        get
        {
            return (_thecontent is not null);
        }
    }

    /// <summary>
    /// Returns True if a Content is stored AND differs from Concerned (i.e. redefines content) : used when
    /// Concerned could not be loaded
    /// </summary>
    public bool HasNewContent
    {
        get
        {
            return (_thecontent is not null && _thecontent != _theconcerned);
        }
    }

    /// <summary>
    /// Returns the stored Content, or a Null Handle Remark that it must be an "Unknown Entity" suitable for the
    /// norm of the containing Model
    /// </summary>
    public Transient? Content()
    {
        return _thecontent;
    }

    /// <summary>
    /// Returns True for an Error Entity, i.e. if the Check brings at least one Fail message
    /// </summary>
    public bool IsError
    {
        get
        {
            return (_thecheck.FailCount > 0);
        }
    }

    /// <summary>
    /// Returns True for an Unknown Entity, i,e. if the Check is empty and Concerned equates Content
    /// </summary>
    public bool IsUnknown
    {
        get
        {
            return ((_thecheck.FailCount == 0) && (_thecheck.WarningCount == 0)
                            && (_theconcerned == _thecontent));
        }
    }

    private FrameworkCheck? _thecheck;
    private Transient? _theconcerned;
    private Transient? _thecontent;
}
