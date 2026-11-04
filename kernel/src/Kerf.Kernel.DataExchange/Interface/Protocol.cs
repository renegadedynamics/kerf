// Port of: src/DataExchange/TKXSBase/Interface/Interface_Protocol.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Protocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// General description of Interface Protocols. A Protocol defines a set of Entity types. This class provides
/// also the notion of Active Protocol, as a working context, defined once then exploited by various Tools
/// and Libraries. It also gives control of type definitions. By default, types are provided by CDL, but
/// specific implementations, or topics like multi-typing, may involve another way
/// </summary>
internal abstract class Protocol : Transient
{
    // OCCT.NET: OCCT's Active, SetActive and ClearActive (one process-wide "active" protocol, set by
    // IFSelect_WorkSession and read by Interface_CopyTool and Transfer_TransferOutput) are left out: the port
    // passes the protocol explicitly (decision 1183).

    /// <summary>
    /// Returns count of Protocol used as Resources (level one)
    /// </summary>
    public abstract int ResourceCount { get; }

    /// <summary>
    /// Returns a Resource, given its rank (between 1 and NbResources)
    /// </summary>
    public abstract Protocol? Resource(int num);

    /// <summary>
    /// Returns a unique positive CaseNumber for each Recognized Object. By default, recognition is based on
    /// Type(1) By default, calls the following one which is deferred.
    /// </summary>
    public virtual int CaseNumber(Transient? obj)
    {
        if (obj is null)
        {
            return 0;
        }
        return TypeNumber(Type(obj));
    }

    /// <summary>
    /// Returns True if type of &lt;obj&gt; is that defined from CDL This is the default but it may change
    /// according implementation
    /// </summary>
    public virtual bool IsDynamicType(Transient? obj)
    {
        return true;
    }

    /// <summary>
    /// Returns the count of DISTINCT types under which an entity may be processed. Each one is candidate to be
    /// recognized by TypeNumber, &lt;obj&gt; is then processed according it By default, returns 1 (the
    /// DynamicType)
    /// </summary>
    public virtual int NbTypes(Transient? obj)
    {
        return 1;
    }

    /// <summary>
    /// Returns a type under which &lt;obj&gt; can be recognized and processed, according its rank in its
    /// definition list (see NbTypes). By default, returns DynamicType
    /// </summary>
    public Type Type(Transient? obj, int nt = 1)
    {
        if (obj is null)
        {
            return typeof(Transient);
        }
        return obj.DynamicType();
    }

    /// <summary>
    /// Returns a unique positive CaseNumber for each Recognized Type, Returns Zero for "&lt;type&gt; not
    /// recognized"
    /// </summary>
    public abstract int TypeNumber(Type? atype);

    /// <summary>
    /// Evaluates a Global Check for a model (with its Graph) Returns True when done, False if data in model do
    /// not apply Very specific of each norm, i.e. of each protocol : the uppest level Protocol assumes it, it
    /// can call GlobalCheck of its resources only if it is necessary Default does nothing, can be redefined
    /// </summary>
    public virtual bool GlobalCheck(EntityGraph g, ref FrameworkCheck? ach)
    {
        return true;
    }

    /// <summary>
    /// Creates an empty Model of the considered Norm
    /// </summary>
    public abstract InterfaceModel? NewModel();

    /// <summary>
    /// Returns True if &lt;model&gt; is a Model of the considered Norm
    /// </summary>
    public abstract bool IsSuitableModel(InterfaceModel? model);

    /// <summary>
    /// Creates a new Unknown Entity for the considered Norm
    /// </summary>
    public abstract Transient? UnknownEntity();

    /// <summary>
    /// Returns True if &lt;ent&gt; is an Unknown Entity for the Norm, i.e. same Type as them created by method
    /// UnknownEntity (for an Entity out of the Norm, answer can be unpredictable)
    /// </summary>
    public abstract bool IsUnknownEntity(Transient? ent);
}
