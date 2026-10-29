// Port of: src/FoundationClasses/TKernel/Message/Message_AlertExtended.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AlertExtended.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// An alert with an attribute (its name and other data) and, optionally, child alerts
/// (<see cref="CompositeAlerts"/>): the alerts of a report's levels (<see cref="Message_Level"/>).
/// </summary>
public class Message_AlertExtended : Message_Alert
{
    protected Message_CompositeAlerts? myCompositAlerts; //!< class provided hierarchical structure of alerts
    protected Message_Attribute? myAttribute;           //!< container of the alert attributes

    /// <summary>
    /// Makes an extended alert with the attribute and adds it to the report with the gravity; returns the alert.
    /// </summary>
    public static Message_Alert AddAlert(Message_Report theReport, Message_Attribute? theAttribute, Message_Gravity theGravity)
    {
        Message_AlertExtended anAlert = new();
        anAlert.SetAttribute(theAttribute);
        theReport.AddAlert(theGravity, anAlert);
        return anAlert;
    }

    /// <summary>The key of the alert's message: its attribute's key, or the class name without an attribute.</summary>
    public override string GetMessageKey()
    {
        if (myAttribute is null)
        {
            return base.GetMessageKey();
        }
        return myAttribute.GetMessageKey();
    }

    /// <summary>The attribute of the alert.</summary>
    public Message_Attribute? Attribute() => myAttribute;

    /// <summary>Sets the attribute of the alert.</summary>
    public void SetAttribute(Message_Attribute? theAttribute) => myAttribute = theAttribute;

    /// <summary>The child alerts, made first if <paramref name="theToCreate"/> is true; null if there are none.</summary>
    public Message_CompositeAlerts? CompositeAlerts(bool theToCreate = false)
    {
        if (myCompositAlerts is null && theToCreate)
        {
            myCompositAlerts = new Message_CompositeAlerts();
        }
        return myCompositAlerts;
    }

    /// <summary>True unless the alert has child alerts: hierarchical alerts can't be merged.</summary>
    public override bool SupportsMerge()
    {
        if (myCompositAlerts is null)
        {
            return true;
        }

        // hierarchical alerts can not be merged
        for (int aGravIter = (int)Message_Trace; aGravIter <= (int)Message_Fail; ++aGravIter)
        {
            if (!myCompositAlerts.Alerts((Message_Gravity)aGravIter).IsEmpty())
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>Never merges (false).</summary>
    public override bool Merge(Message_Alert theTarget)
    {
        // by default, merge trivially
        return false;
    }
}
