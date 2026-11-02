// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_SelectForTransfer.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_SelectForTransfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This selection selects the entities which are recognised for transfer by an Actor for Read : current one
/// or another one. An Actor is an operator which runs transfers from interface entities to objects for
/// Imagine. It has a method to recognize the entities it can process (by default, it recognises all, this
/// method can be redefined). A TransferReader brings an Actor, according to the currently selected norm and
/// transfer conditions. This selection considers, either the current Actor (brought by the TransferReader,
/// updated as required), or a precise one.
/// </summary>
public class XSControl_SelectForTransfer : IFSelect_SelectExtract
{

    /// <summary>
    /// Creates a SelectForTransfer, non initialised it sorts nothing, unless an Actor has been defined
    /// </summary>
    public XSControl_SelectForTransfer()
    {
    }

    /// <summary>
    /// Creates a SelectForTransfer, which will work with the currently defined Actor brought by the
    /// TransferReader
    /// </summary>
    public XSControl_SelectForTransfer(XSControl_TransferReader? TR)
    {
        theTR = TR;
    }

    /// <summary>
    /// Sets a TransferReader to sort entities : it brings the Actor, which may change, while the TransferReader
    /// does not
    /// </summary>
    public void SetReader(XSControl_TransferReader? TR)
    {
        theTR = TR;
    }

    /// <summary>
    /// Sets a precise actor to sort entities This definition oversedes the creation with a TransferReader
    /// </summary>
    public void SetActor(Transfer_ActorOfTransientProcess? act)
    {
        theAC = act;
    }

    /// <summary>
    /// Returns the Actor used as precised one. Returns a Null Handle for a creation from a TransferReader
    /// without any further setting
    /// </summary>
    public Transfer_ActorOfTransientProcess? Actor()
    {
        return theAC;
    }

    /// <summary>
    /// Returns the Reader (if created with a Reader) Returns a Null Handle if not created with a Reader
    /// </summary>
    public XSControl_TransferReader? Reader()
    {
        return theTR;
    }

    /// <summary>
    /// Returns True for an Entity which is recognized by the Actor, either the precised one, or the one defined
    /// by TransferReader
    /// </summary>
    public override bool Sort(int rank, Standard_Transient? ent, Interface_InterfaceModel? model)
    {
        Transfer_ActorOfTransientProcess act = theAC;
        if (act is null && theTR is not null)
        {
            act = theTR.Actor();
        }
        if (act is not null)
        {
            return act.Recognize(ent); //,theTR->TransientProcess());//act->Recognize(ent);
        }
        return false;
    }

    /// <summary>
    /// Returns a text defining the criterium : "Recognized for Transfer [(current actor)]"
    /// </summary>
    public override string ExtractLabel()
    {
        if (theTR is not null)
        {
            return ("Recognized for Transfer (current actor)");
        }
        return ("Recognized for Transfer");
    }

    private XSControl_TransferReader? theTR;
    private Transfer_ActorOfTransientProcess? theAC;
}
