// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_AP203Context.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_AP203Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using System.Net;
using System.Net.Sockets;
using static Occt.StepBasic_AheadOrBehind;

namespace Occt;

/// <summary>
/// Maintains context specific for AP203 (required data and management information such as persons, dates,
/// approvals etc.) It contains static entities (which can be shared), default values for person and organisation,
/// and also provides tool for creating management entities around specific part (SDR).
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). The default date and person come, as in OCCT, from the system
/// (the local time and time zone, the user's name, the host's address) the first time they are asked; a writer that
/// must be repeatable sets its own with <see cref="SetDefaultDateAndTime"/> and
/// <see cref="SetDefaultPersonAndOrganization"/> (decision 1267).
/// </remarks>
public class STEPConstruct_AP203Context
{
    /// <summary>Creates tool and fills constant fields</summary>
    public STEPConstruct_AP203Context()
    {
        InitRoles();
    }

    /// <summary>Returns default approval entity which is used when no other data are available</summary>
    public StepBasic_Approval DefaultApproval()
    {
        if (defApproval is null)
        {
            StepBasic_ApprovalStatus aStatus = new StepBasic_ApprovalStatus();
            TCollection_HAsciiString aName = new TCollection_HAsciiString("not_yet_approved");
            aStatus.Init(aName);

            TCollection_HAsciiString aLevel = new TCollection_HAsciiString("");
            defApproval = new StepBasic_Approval();
            defApproval.Init(aStatus, aLevel);
        }
        return defApproval;
    }

    /// <summary>Sets default approval</summary>
    public void SetDefaultApproval(StepBasic_Approval app)
    {
        defApproval = app;
    }

    /// <summary>Returns default date_and_time entity which is used when no other data are available</summary>
    public StepBasic_DateAndTime DefaultDateAndTime()
    {
        if (defDateAndTime is null)
        {
            // OCCT: OSD_Process::SystemDate (the local time) and the C runtime's time zone (_get_timezone: the
            // standard offset, in seconds west of UTC)
            DateTime date = DateTime.Now;

            StepBasic_CalendarDate aDate = new StepBasic_CalendarDate();
            aDate.Init(date.Year, date.Day, date.Month);

            StepBasic_CoordinatedUniversalTimeOffset zone = new StepBasic_CoordinatedUniversalTimeOffset();
            int shift = -(int)TimeZoneInfo.Local.BaseUtcOffset.TotalSeconds;
            int shifth = Math.Abs(shift) / 3600;
            int shiftm = (Math.Abs(shift) - shifth * 3600) / 60;
            StepBasic_AheadOrBehind sense = (shift > 0 ? StepBasic_aobBehind
                                             : shift < 0 ? StepBasic_aobAhead
                                                         : StepBasic_aobExact);
            zone.Init(shifth, (shiftm != 0), shiftm, sense);

            StepBasic_LocalTime aTime = new StepBasic_LocalTime();
            aTime.Init(date.Hour, true, date.Minute, false, 0.0, zone);

            defDateAndTime = new StepBasic_DateAndTime();
            defDateAndTime.Init(aDate, aTime);
        }
        return defDateAndTime;
    }

    /// <summary>Sets default date_and_time entity</summary>
    public void SetDefaultDateAndTime(StepBasic_DateAndTime dt)
    {
        defDateAndTime = dt;
    }

    /// <summary>Returns default person_and_organization entity which is used when no other data are available</summary>
    public StepBasic_PersonAndOrganization DefaultPersonAndOrganization()
    {
        if (defPersonAndOrganization is null)
        {
            // get IP address as a unique id of organization
            TCollection_HAsciiString orgId = new TCollection_HAsciiString("IP");
            string anIP = InternetAddress();
            // cut off last number
            int aLastDotIndex = anIP.LastIndexOf('.') + 1;
            if (aLastDotIndex > 0)
            {
                anIP = anIP.Substring(0, aLastDotIndex - 1);
                orgId.AssignCat(anIP);
            }

            // create organization
            StepBasic_Organization aOrg = new StepBasic_Organization();
            TCollection_HAsciiString oName = new TCollection_HAsciiString("Unspecified");
            TCollection_HAsciiString oDescr = new TCollection_HAsciiString("");
            aOrg.Init(true, orgId, oName, oDescr);

            // construct person`s name
            // OCCT: OSD_Process::UserName (on other systems than Windows, the password file's full name)
            string user = Environment.UserName ?? "";
            TCollection_HAsciiString fname = new TCollection_HAsciiString("");
            TCollection_HAsciiString lname = new TCollection_HAsciiString("");
            NCollection_HArray1<TCollection_HAsciiString> mname = null;
            NCollection_Sequence<string> names = new NCollection_Sequence<string>();
            foreach (string token in user.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries))
            {
                names.Append(token);
            }
            if (names.Length() > 0)
            {
                fname.AssignCat(names.Value(1));
            }
            if (names.Length() > 1)
            {
                lname.AssignCat(names.Value(names.Length()));
            }
            if (names.Length() > 2)
            {
                mname = new NCollection_HArray1<TCollection_HAsciiString>(1, names.Length() - 2);
                for (int i = 2; i < names.Length(); i++)
                {
                    mname.SetValue(i - 1, new TCollection_HAsciiString(names.Value(i)));
                }
            }

            // create a person
            StepBasic_Person aPerson = new StepBasic_Person();
            TCollection_HAsciiString uid = new TCollection_HAsciiString(orgId);
            uid.AssignCat(",");
            uid.AssignCat(user);
            NCollection_HArray1<TCollection_HAsciiString> suffix = null, prefix = null;
            aPerson.Init(uid, true, lname, true, fname, (mname is not null), mname, false, suffix, false, prefix);

            defPersonAndOrganization = new StepBasic_PersonAndOrganization();
            defPersonAndOrganization.Init(aPerson, aOrg);
        }
        return defPersonAndOrganization;
    }

    // OCCT's OSD_Host::InternetAddress: the first address of the host's name, as text ("" when unknown)
    private static string InternetAddress()
    {
        try
        {
            foreach (IPAddress anAddress in Dns.GetHostAddresses(Dns.GetHostName()))
            {
                if (anAddress.AddressFamily == AddressFamily.InterNetwork)
                {
                    return anAddress.ToString();
                }
            }
        }
        catch (SocketException)
        {
        }
        return "";
    }

    /// <summary>Sets default person_and_organization entity</summary>
    public void SetDefaultPersonAndOrganization(StepBasic_PersonAndOrganization po)
    {
        defPersonAndOrganization = po;
    }

    /// <summary>Returns default security_classification_level entity which is used when no other data are available</summary>
    public StepBasic_SecurityClassificationLevel DefaultSecurityClassificationLevel()
    {
        if (defSecurityClassificationLevel is null)
        {
            defSecurityClassificationLevel = new StepBasic_SecurityClassificationLevel();
            TCollection_HAsciiString levName = new TCollection_HAsciiString("unclassified");
            defSecurityClassificationLevel.Init(levName);
        }
        return defSecurityClassificationLevel;
    }

    /// <summary>Sets default security_classification_level</summary>
    public void SetDefaultSecurityClassificationLevel(StepBasic_SecurityClassificationLevel scl)
    {
        defSecurityClassificationLevel = scl;
    }

    public StepBasic_PersonAndOrganizationRole RoleCreator() => roleCreator;

    public StepBasic_PersonAndOrganizationRole RoleDesignOwner() => roleDesignOwner;

    public StepBasic_PersonAndOrganizationRole RoleDesignSupplier() => roleDesignSupplier;

    public StepBasic_PersonAndOrganizationRole RoleClassificationOfficer() => roleClassificationOfficer;

    public StepBasic_DateTimeRole RoleCreationDate() => roleCreationDate;

    public StepBasic_DateTimeRole RoleClassificationDate() => roleClassificationDate;

    /// <summary>Return predefined PersonAndOrganizationRole and DateTimeRole entities named 'approver'</summary>
    public StepBasic_ApprovalRole RoleApprover() => roleApprover;

    /// <summary>
    /// Takes SDR (part) which brings all standard data around part (common for AP203 and AP214) and creates all the
    /// additional entities required for AP203
    /// </summary>
    public void Init(StepShape_ShapeDefinitionRepresentation sdr)
    {
        Clear();
        STEPConstruct_Part SDRTool = new STEPConstruct_Part();
        SDRTool.ReadSDR(sdr);
        InitPart(SDRTool);
    }

    /// <summary>
    /// Takes tool which describes standard data around part (common for AP203 and AP214) and creates all the
    /// additional entities required for AP203
    /// </summary>
    public void Init(STEPConstruct_Part SDRTool)
    {
        Clear();
        InitPart(SDRTool);
    }

    /// <summary>
    /// Takes NAUO which describes assembly link to component and creates the security_classification entity
    /// associated to it as required by the AP203
    /// </summary>
    public void Init(StepRepr_NextAssemblyUsageOccurrence NAUO)
    {
        Clear();
        InitAssembly(NAUO);
    }

    /// <summary>Returns entities created for the part last initialised</summary>
    public StepAP203_CcDesignPersonAndOrganizationAssignment GetCreator() => myCreator;

    public StepAP203_CcDesignPersonAndOrganizationAssignment GetDesignOwner() => myDesignOwner;

    public StepAP203_CcDesignPersonAndOrganizationAssignment GetDesignSupplier() => myDesignSupplier;

    public StepAP203_CcDesignPersonAndOrganizationAssignment GetClassificationOfficer() => myClassificationOfficer;

    public StepAP203_CcDesignSecurityClassification GetSecurity() => mySecurity;

    public StepAP203_CcDesignDateAndTimeAssignment GetCreationDate() => myCreationDate;

    public StepAP203_CcDesignDateAndTimeAssignment GetClassificationDate() => myClassificationDate;

    public StepAP203_CcDesignApproval GetApproval() => myApproval;

    public StepBasic_ApprovalPersonOrganization GetApprover() => myApprover;

    public StepBasic_ApprovalDateTime GetApprovalDateTime() => myApprovalDateTime;

    /// <summary>Return entities (roots) instantiated for the part by method Init</summary>
    public StepBasic_ProductCategoryRelationship GetProductCategoryRelationship() => myProductCategoryRelationship;

    /// <summary>Clears all fields describing entities specific to each part</summary>
    public void Clear()
    {
        myCreator = null;
        myDesignOwner = null;
        myDesignSupplier = null;
        myClassificationOfficer = null;
        mySecurity = null;
        myCreationDate = null;
        myClassificationDate = null;
        myApproval = null;

        //  myApprover.Nullify();
        //  myApprovalDateTime.Nullify();

        myProductCategoryRelationship = null;
    }

    /// <summary>Initializes constant fields (shared entities)</summary>
    public void InitRoles()
    {
        roleCreator = new StepBasic_PersonAndOrganizationRole();
        roleDesignOwner = new StepBasic_PersonAndOrganizationRole();
        roleDesignSupplier = new StepBasic_PersonAndOrganizationRole();
        roleClassificationOfficer = new StepBasic_PersonAndOrganizationRole();
        roleCreationDate = new StepBasic_DateTimeRole();
        roleClassificationDate = new StepBasic_DateTimeRole();
        roleApprover = new StepBasic_ApprovalRole();

        roleCreator.Init(new TCollection_HAsciiString("creator"));
        roleDesignOwner.Init(new TCollection_HAsciiString("design_owner"));
        roleDesignSupplier.Init(new TCollection_HAsciiString("design_supplier"));
        roleClassificationOfficer.Init(new TCollection_HAsciiString("classification_officer"));
        roleCreationDate.Init(new TCollection_HAsciiString("creation_date"));
        roleClassificationDate.Init(new TCollection_HAsciiString("classification_date"));
        roleApprover.Init(new TCollection_HAsciiString("approver"));
    }

    private static StepAP203_PersonOrganizationItem PersonOrganizationItem(Transient theValue)
    {
        StepAP203_PersonOrganizationItem anItem = new StepAP203_PersonOrganizationItem();
        anItem.SetValue(theValue);
        return anItem;
    }

    private void InitPart(STEPConstruct_Part SDRTool)
    {
        if (myCreator is null)
        {
            myCreator = new StepAP203_CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<StepAP203_PersonOrganizationItem> items = new NCollection_HArray1<StepAP203_PersonOrganizationItem>(1, 2);
            items.SetValue(1, PersonOrganizationItem(SDRTool.PDF()));
            items.SetValue(2, PersonOrganizationItem(SDRTool.PD()));
            myCreator.Init(DefaultPersonAndOrganization(), RoleCreator(), items);
        }

        if (myDesignOwner is null)
        {
            myDesignOwner = new StepAP203_CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<StepAP203_PersonOrganizationItem> items = new NCollection_HArray1<StepAP203_PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(SDRTool.Product()));
            myDesignOwner.Init(DefaultPersonAndOrganization(), RoleDesignOwner(), items);
        }

        if (myDesignSupplier is null)
        {
            myDesignSupplier = new StepAP203_CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<StepAP203_PersonOrganizationItem> items = new NCollection_HArray1<StepAP203_PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(SDRTool.PDF()));
            myDesignSupplier.Init(DefaultPersonAndOrganization(), RoleDesignSupplier(), items);
        }

        if (myCreationDate is null)
        {
            myCreationDate = new StepAP203_CcDesignDateAndTimeAssignment();
            NCollection_HArray1<StepAP203_DateTimeItem> items = new NCollection_HArray1<StepAP203_DateTimeItem>(1, 1);
            StepAP203_DateTimeItem anItem = new StepAP203_DateTimeItem();
            anItem.SetValue(SDRTool.PD());
            items.SetValue(1, anItem);
            myCreationDate.Init(DefaultDateAndTime(), RoleCreationDate(), items);
        }

        if (mySecurity is null)
        {
            TCollection_HAsciiString aName = new TCollection_HAsciiString("");
            TCollection_HAsciiString aPurpose = new TCollection_HAsciiString("");
            StepBasic_SecurityClassification sc = new StepBasic_SecurityClassification();
            sc.Init(aName, aPurpose, DefaultSecurityClassificationLevel());

            mySecurity = new StepAP203_CcDesignSecurityClassification();
            NCollection_HArray1<StepAP203_ClassifiedItem> items = new NCollection_HArray1<StepAP203_ClassifiedItem>(1, 1);
            StepAP203_ClassifiedItem anItem = new StepAP203_ClassifiedItem();
            anItem.SetValue(SDRTool.PDF());
            items.SetValue(1, anItem);
            mySecurity.Init(sc, items);
        }
        InitSecurityRequisites();

        if (myApproval is null)
        {
            myApproval = new StepAP203_CcDesignApproval();
            NCollection_HArray1<StepAP203_ApprovedItem> items = new NCollection_HArray1<StepAP203_ApprovedItem>(1, 3);
            StepAP203_ApprovedItem anItem1 = new StepAP203_ApprovedItem();
            anItem1.SetValue(SDRTool.PDF());
            items.SetValue(1, anItem1);
            StepAP203_ApprovedItem anItem2 = new StepAP203_ApprovedItem();
            anItem2.SetValue(SDRTool.PD());
            items.SetValue(2, anItem2);
            StepAP203_ApprovedItem anItem3 = new StepAP203_ApprovedItem();
            anItem3.SetValue(mySecurity.AssignedSecurityClassification());
            items.SetValue(3, anItem3);
            myApproval.Init(DefaultApproval(), items);
        }
        InitApprovalRequisites();

        if (myProductCategoryRelationship is null)
        {
            StepBasic_ProductCategory PC = new StepBasic_ProductCategory();
            TCollection_HAsciiString PCName = new TCollection_HAsciiString("part");
            PC.Init(PCName, false, null);

            myProductCategoryRelationship = new StepBasic_ProductCategoryRelationship();
            TCollection_HAsciiString PCRName = new TCollection_HAsciiString("");
            TCollection_HAsciiString PCRDescr = new TCollection_HAsciiString("");
            myProductCategoryRelationship.Init(PCRName, true, PCRDescr, PC, SDRTool.PRPC());
        }
    }

    /// <summary>Initializes all missing data which are required for assembly</summary>
    public void InitAssembly(StepRepr_NextAssemblyUsageOccurrence NAUO)
    {
        if (mySecurity is null)
        {
            TCollection_HAsciiString aName = new TCollection_HAsciiString("");
            TCollection_HAsciiString aPurpose = new TCollection_HAsciiString("");
            StepBasic_SecurityClassification sc = new StepBasic_SecurityClassification();
            sc.Init(aName, aPurpose, DefaultSecurityClassificationLevel());

            mySecurity = new StepAP203_CcDesignSecurityClassification();
            NCollection_HArray1<StepAP203_ClassifiedItem> items = new NCollection_HArray1<StepAP203_ClassifiedItem>(1, 1);
            StepAP203_ClassifiedItem anItem = new StepAP203_ClassifiedItem();
            anItem.SetValue(NAUO);
            items.SetValue(1, anItem);
            mySecurity.Init(sc, items);
        }
        InitSecurityRequisites();

        if (myApproval is null)
        {
            myApproval = new StepAP203_CcDesignApproval();
            NCollection_HArray1<StepAP203_ApprovedItem> items = new NCollection_HArray1<StepAP203_ApprovedItem>(1, 1);
            StepAP203_ApprovedItem anItem = new StepAP203_ApprovedItem();
            anItem.SetValue(mySecurity.AssignedSecurityClassification());
            items.SetValue(1, anItem);
            myApproval.Init(DefaultApproval(), items);
        }
        InitApprovalRequisites();
    }

    /// <summary>Initializes ClassificationOfficer and ClassificationDate entities according to Security entity</summary>
    public void InitSecurityRequisites()
    {
        if (myClassificationOfficer is null
            || myClassificationOfficer.Items().Value(1).Value() != mySecurity.AssignedSecurityClassification())
        {
            myClassificationOfficer = new StepAP203_CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<StepAP203_PersonOrganizationItem> items = new NCollection_HArray1<StepAP203_PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(mySecurity.AssignedSecurityClassification()));
            myClassificationOfficer.Init(DefaultPersonAndOrganization(), RoleClassificationOfficer(), items);
        }

        if (myClassificationDate is null
            || myClassificationDate.Items().Value(1).Value() != mySecurity.AssignedSecurityClassification())
        {
            myClassificationDate = new StepAP203_CcDesignDateAndTimeAssignment();
            NCollection_HArray1<StepAP203_DateTimeItem> items = new NCollection_HArray1<StepAP203_DateTimeItem>(1, 1);
            StepAP203_DateTimeItem anItem = new StepAP203_DateTimeItem();
            anItem.SetValue(mySecurity.AssignedSecurityClassification());
            items.SetValue(1, anItem);
            myClassificationDate.Init(DefaultDateAndTime(), RoleClassificationDate(), items);
        }
    }

    /// <summary>Initializes Approver and ApprovalDateTime entities according to Approval entity</summary>
    public void InitApprovalRequisites()
    {
        if (myApprover is null || myApprover.AuthorizedApproval() != myApproval.AssignedApproval())
        {
            myApprover = new StepBasic_ApprovalPersonOrganization();
            StepBasic_PersonOrganizationSelect po = new StepBasic_PersonOrganizationSelect();
            po.SetValue(DefaultPersonAndOrganization());
            myApprover.Init(po, myApproval.AssignedApproval(), RoleApprover());
        }

        if (myApprovalDateTime is null || myApprovalDateTime.DatedApproval() != myApproval.AssignedApproval())
        {
            myApprovalDateTime = new StepBasic_ApprovalDateTime();
            StepBasic_DateTimeSelect dt = new StepBasic_DateTimeSelect();
            dt.SetValue(DefaultDateAndTime());
            myApprovalDateTime.Init(dt, myApproval.AssignedApproval());
        }
    }

    private StepBasic_Approval defApproval;
    private StepBasic_DateAndTime defDateAndTime;
    private StepBasic_PersonAndOrganization defPersonAndOrganization;
    private StepBasic_SecurityClassificationLevel defSecurityClassificationLevel;
    private StepBasic_PersonAndOrganizationRole roleCreator;
    private StepBasic_PersonAndOrganizationRole roleDesignOwner;
    private StepBasic_PersonAndOrganizationRole roleDesignSupplier;
    private StepBasic_PersonAndOrganizationRole roleClassificationOfficer;
    private StepBasic_DateTimeRole roleCreationDate;
    private StepBasic_DateTimeRole roleClassificationDate;
    private StepBasic_ApprovalRole roleApprover;
    private StepAP203_CcDesignPersonAndOrganizationAssignment myCreator;
    private StepAP203_CcDesignPersonAndOrganizationAssignment myDesignOwner;
    private StepAP203_CcDesignPersonAndOrganizationAssignment myDesignSupplier;
    private StepAP203_CcDesignPersonAndOrganizationAssignment myClassificationOfficer;
    private StepAP203_CcDesignSecurityClassification mySecurity;
    private StepAP203_CcDesignDateAndTimeAssignment myCreationDate;
    private StepAP203_CcDesignDateAndTimeAssignment myClassificationDate;
    private StepAP203_CcDesignApproval myApproval;
    private StepBasic_ApprovalPersonOrganization myApprover;
    private StepBasic_ApprovalDateTime myApprovalDateTime;
    private StepBasic_ProductCategoryRelationship myProductCategoryRelationship;
}
