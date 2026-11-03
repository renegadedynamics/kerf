// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_BinderOfShape.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_BinderOfShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Allows direct binding between a starting Object and the Result of its transfer when it is Unique. The Result itself
/// is a Shape.
/// </summary>
/// <remarks>
/// The result is the shape as the transfer made it: readers publish (freeze) the shapes they return, not the binders
/// (decision 1205).
/// </remarks>
public class TransferBRep_BinderOfShape : Transfer_Binder
{
    /// <summary>Normal standard constructor, creates an empty BinderOfShape.</summary>
    public TransferBRep_BinderOfShape()
    {
    }

    /// <summary>Constructor which in the same time defines the result.</summary>
    public TransferBRep_BinderOfShape(in Shape res)
    {
        theres = res;
        SetResultPresent();
    }

    /// <summary>Returns the Type permitted for the Result, i.e. the Type of the Parameter Class (statically defined).</summary>
    public override Type? ResultType()
    {
        return TransferBRep_ShapeInfo.Type(theres); // correspond a "STANDARD_TYPE(TopoDS_Shape)"
    }

    /// <summary>Returns the Type Name computed for the Result (dynamic).</summary>
    public override string ResultTypeName()
    {
        return TransferBRep_ShapeInfo.TypeName(theres); // correspond a "STANDARD_TYPE(TopoDS_Shape)"
    }

    /// <summary>Defines the Result.</summary>
    /// <exception cref="Transfer_TransferFailure">The result is already set and used.</exception>
    public void SetResult(in Shape res)
    {
        SetResultPresent();
        theres = res;
    }

    /// <summary>Returns the defined Result, if there is one.</summary>
    public Shape Result()
    {
        return theres;
    }

    /// <summary>
    /// Returns the defined Result, if there is one, and allows to change it (avoids Result + SetResult). Admits that
    /// Result can be not yet defined. Warning : a call to CResult causes Result to be known as defined.
    /// </summary>
    public ref Shape CResult()
    {
        SetResultPresent();
        return ref theres;
    }

    private Shape theres;
}
