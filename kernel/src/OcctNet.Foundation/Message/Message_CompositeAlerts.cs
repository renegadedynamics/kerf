// Port of: src/FoundationClasses/TKernel/Message/Message_CompositeAlerts.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_CompositeAlerts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>A container of alerts, one list per gravity.</summary>
/// <remarks>
/// OCCT's <c>Standard_ASSERT_RETURN</c> checks are kept (they are compiled in release builds too): a null alert or a
/// gravity out of range gives an empty list or false. <c>DumpJson</c> is left out.
/// </remarks>
public class Message_CompositeAlerts : Standard_Transient
{
    // store messages in a lists sorted by gravity;
    // here we rely on knowledge that Message_Fail is the last element of the enum
    protected readonly NCollection_List<Message_Alert>[] myAlerts = NewLists(); //!< container of child alert for each type of gravity

    private static NCollection_List<Message_Alert>[] NewLists()
    {
        var aLists = new NCollection_List<Message_Alert>[(int)Message_Fail + 1];
        for (int i = 0; i < aLists.Length; ++i)
        {
            aLists[i] = new NCollection_List<Message_Alert>();
        }
        return aLists;
    }

    private bool IsValid(Message_Gravity theGravity) => theGravity >= 0 && (int)theGravity < myAlerts.Length;

    /// <summary>
    /// The alerts of the gravity: the container's own list, which callers only read (a new empty list for a gravity
    /// out of range).
    /// </summary>
    public NCollection_List<Message_Alert> Alerts(Message_Gravity theGravity)
    {
        if (!IsValid(theGravity))
        {
            // Standard_ASSERT_RETURN: "Requesting alerts for gravity not in valid range"
            return new NCollection_List<Message_Alert>();
        }
        return myAlerts[(int)theGravity];
    }

    /// <summary>
    /// Adds the alert with the gravity, or merges it into an alert of exactly the same type if it supports merging.
    /// True if the alert was appended, false if it was merged (or is invalid).
    /// </summary>
    public bool AddAlert(Message_Gravity theGravity, Message_Alert? theAlert)
    {
        if (theAlert is null || !IsValid(theGravity))
        {
            // Standard_ASSERT_RETURN: "Attempt to add null alert", "Adding alert with gravity not in valid range"
            return false;
        }

        NCollection_List<Message_Alert> aList = myAlerts[(int)theGravity];
        if (theAlert.SupportsMerge() && !aList.IsEmpty())
        {
            // merge is performed only for alerts of exactly same type
            Type aType = theAlert.DynamicType();
            for (NCollection_List<Message_Alert>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
            {
                // if merged successfully, just return
                if (aType == anIt.Value().DynamicType() && theAlert.Merge(anIt.Value()))
                {
                    return false;
                }
            }
        }

        // if not merged, just add to the list
        aList.Append(theAlert);
        return true;
    }

    /// <summary>Removes the alert from the gravity's list; true if it was there.</summary>
    public bool RemoveAlert(Message_Gravity theGravity, Message_Alert? theAlert)
    {
        if (theAlert is null || !IsValid(theGravity))
        {
            return false;
        }
        NCollection_List<Message_Alert> anAlerts = myAlerts[(int)theGravity];
        if (!anAlerts.Contains(theAlert))
        {
            return false;
        }
        return anAlerts.Remove(theAlert);
    }

    /// <summary>True if the alert is in one of the lists.</summary>
    public bool HasAlert(Message_Alert theAlert)
    {
        for (int aGravIter = (int)Message_Trace; aGravIter <= (int)Message_Fail; ++aGravIter)
        {
            NCollection_List<Message_Alert> anAlerts = Alerts((Message_Gravity)aGravIter);
            if (anAlerts.Contains(theAlert))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if an alert of exactly the type (<c>IsInstance</c>: not a derived type) has the gravity.</summary>
    public bool HasAlert(Type theType, Message_Gravity theGravity)
    {
        if (!IsValid(theGravity))
        {
            return false;
        }
        for (NCollection_List<Message_Alert>.Iterator anIt = new(myAlerts[(int)theGravity]); anIt.More(); anIt.Next())
        {
            if (anIt.Value().IsInstance(theType))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Removes all the alerts.</summary>
    public void Clear()
    {
        for (int i = 0; i < myAlerts.Length; ++i)
        {
            myAlerts[i].Clear();
        }
    }

    /// <summary>Removes the alerts of the gravity.</summary>
    public void Clear(Message_Gravity theGravity)
    {
        if (!IsValid(theGravity))
        {
            return;
        }
        myAlerts[(int)theGravity].Clear();
    }

    /// <summary>Removes the alerts of exactly the type (and null ones).</summary>
    public void Clear(Type theType)
    {
        for (int i = 0; i < myAlerts.Length; ++i)
        {
            for (NCollection_List<Message_Alert>.Iterator anIt = new(myAlerts[i]); anIt.More();)
            {
                if (anIt.Value() is null || anIt.Value().IsInstance(theType))
                {
                    myAlerts[i].Remove(anIt);
                }
                else
                {
                    // OCCT calls anIt.More() here, not anIt.Next(): an alert of another type makes the loop run
                    // forever. The port advances (occt-issues.md, unit 6A).
                    anIt.Next();
                }
            }
        }
    }
}
