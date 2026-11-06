// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_AP203Context.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_AP203Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Ap203;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using System.Net;
using System.Net.Sockets;

namespace Kerf.Engine.Exchange.Step.Construct;

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
internal class AP203Context
{
    /// <summary>Creates tool and fills constant fields</summary>
    public AP203Context()
    {
        InitRoles();
    }

    /// <summary>Returns default approval entity which is used when no other data are available</summary>
    public Approval DefaultApproval()
    {
        if (_defApproval is null)
        {
            ApprovalStatus status = new ApprovalStatus();
            TCollection_HAsciiString name = new TCollection_HAsciiString("not_yet_approved");
            status.Init(name);

            TCollection_HAsciiString level = new TCollection_HAsciiString("");
            _defApproval = new Approval();
            _defApproval.Init(status, level);
        }
        return _defApproval;
    }

    /// <summary>Sets default approval</summary>
    public void SetDefaultApproval(Approval app)
    {
        _defApproval = app;
    }

    /// <summary>Returns default date_and_time entity which is used when no other data are available</summary>
    public DateAndTime DefaultDateAndTime()
    {
        if (_defDateAndTime is null)
        {
            // OCCT: OSD_Process::SystemDate (the local time) and the C runtime's time zone (_get_timezone: the
            // standard offset, in seconds west of UTC)
            DateTime date = DateTime.Now;

            CalendarDate aDate = new CalendarDate();
            aDate.Init(date.Year, date.Day, date.Month);

            CoordinatedUniversalTimeOffset zone = new CoordinatedUniversalTimeOffset();
            int shift = -(int)TimeZoneInfo.Local.BaseUtcOffset.TotalSeconds;
            int shifth = Math.Abs(shift) / 3600;
            int shiftm = (Math.Abs(shift) - shifth * 3600) / 60;
            AheadOrBehind sense = (shift > 0 ? AheadOrBehind.AobBehind
                                             : shift < 0 ? AheadOrBehind.AobAhead
                                                         : AheadOrBehind.AobExact);
            zone.Init(shifth, (shiftm != 0), shiftm, sense);

            LocalTime time = new LocalTime();
            time.Init(date.Hour, true, date.Minute, false, 0.0, zone);

            _defDateAndTime = new DateAndTime();
            _defDateAndTime.Init(aDate, time);
        }
        return _defDateAndTime;
    }

    /// <summary>Sets default date_and_time entity</summary>
    public void SetDefaultDateAndTime(DateAndTime dt)
    {
        _defDateAndTime = dt;
    }

    /// <summary>Returns default person_and_organization entity which is used when no other data are available</summary>
    public PersonAndOrganization DefaultPersonAndOrganization()
    {
        if (_defPersonAndOrganization is null)
        {
            // get IP address as a unique id of organization
            TCollection_HAsciiString orgId = new TCollection_HAsciiString("IP");
            string ip = InternetAddress();
            // cut off last number
            int lastDotIndex = ip.LastIndexOf('.') + 1;
            if (lastDotIndex > 0)
            {
                ip = ip.Substring(0, lastDotIndex - 1);
                orgId.AssignCat(ip);
            }

            // create organization
            Organization org = new Organization();
            TCollection_HAsciiString oName = new TCollection_HAsciiString("Unspecified");
            TCollection_HAsciiString oDescr = new TCollection_HAsciiString("");
            org.Init(true, orgId, oName, oDescr);

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
            Person person = new Person();
            TCollection_HAsciiString uid = new TCollection_HAsciiString(orgId);
            uid.AssignCat(",");
            uid.AssignCat(user);
            NCollection_HArray1<TCollection_HAsciiString> suffix = null, prefix = null;
            person.Init(uid, true, lname, true, fname, (mname is not null), mname, false, suffix, false, prefix);

            _defPersonAndOrganization = new PersonAndOrganization();
            _defPersonAndOrganization.Init(person, org);
        }
        return _defPersonAndOrganization;
    }

    // OCCT's OSD_Host::InternetAddress: the first address of the host's name, as text ("" when unknown)
    private static string InternetAddress()
    {
        try
        {
            foreach (IPAddress address in Dns.GetHostAddresses(Dns.GetHostName()))
            {
                if (address.AddressFamily == AddressFamily.InterNetwork)
                {
                    return address.ToString();
                }
            }
        }
        catch (SocketException)
        {
        }
        return "";
    }

    /// <summary>Sets default person_and_organization entity</summary>
    public void SetDefaultPersonAndOrganization(PersonAndOrganization po)
    {
        _defPersonAndOrganization = po;
    }

    /// <summary>Returns default security_classification_level entity which is used when no other data are available</summary>
    public SecurityClassificationLevel DefaultSecurityClassificationLevel()
    {
        if (_defSecurityClassificationLevel is null)
        {
            _defSecurityClassificationLevel = new SecurityClassificationLevel();
            TCollection_HAsciiString levName = new TCollection_HAsciiString("unclassified");
            _defSecurityClassificationLevel.Init(levName);
        }
        return _defSecurityClassificationLevel;
    }

    /// <summary>Sets default security_classification_level</summary>
    public void SetDefaultSecurityClassificationLevel(SecurityClassificationLevel scl)
    {
        _defSecurityClassificationLevel = scl;
    }

    public PersonAndOrganizationRole RoleCreator() => _roleCreator;

    public PersonAndOrganizationRole RoleDesignOwner() => _roleDesignOwner;

    public PersonAndOrganizationRole RoleDesignSupplier() => _roleDesignSupplier;

    public PersonAndOrganizationRole RoleClassificationOfficer() => _roleClassificationOfficer;

    public DateTimeRole RoleCreationDate() => _roleCreationDate;

    public DateTimeRole RoleClassificationDate() => _roleClassificationDate;

    /// <summary>Return predefined PersonAndOrganizationRole and DateTimeRole entities named 'approver'</summary>
    public ApprovalRole RoleApprover() => _roleApprover;

    /// <summary>
    /// Takes SDR (part) which brings all standard data around part (common for AP203 and AP214) and creates all the
    /// additional entities required for AP203
    /// </summary>
    public void Init(StepShapeDefinitionRepresentation sdr)
    {
        Clear();
        Part sdrTool = new Part();
        sdrTool.ReadSDR(sdr);
        InitPart(sdrTool);
    }

    /// <summary>
    /// Takes tool which describes standard data around part (common for AP203 and AP214) and creates all the
    /// additional entities required for AP203
    /// </summary>
    public void Init(Part sdrTool)
    {
        Clear();
        InitPart(sdrTool);
    }

    /// <summary>
    /// Takes NAUO which describes assembly link to component and creates the security_classification entity
    /// associated to it as required by the AP203
    /// </summary>
    public void Init(NextAssemblyUsageOccurrence nauo)
    {
        Clear();
        InitAssembly(nauo);
    }

    /// <summary>Returns entities created for the part last initialised</summary>
    public CcDesignPersonAndOrganizationAssignment GetCreator() => _creator;

    public CcDesignPersonAndOrganizationAssignment GetDesignOwner() => _designOwner;

    public CcDesignPersonAndOrganizationAssignment GetDesignSupplier() => _designSupplier;

    public CcDesignPersonAndOrganizationAssignment GetClassificationOfficer() => _classificationOfficer;

    public CcDesignSecurityClassification GetSecurity() => _security;

    public CcDesignDateAndTimeAssignment GetCreationDate() => _creationDate;

    public CcDesignDateAndTimeAssignment GetClassificationDate() => _classificationDate;

    public CcDesignApproval GetApproval() => _approval;

    public ApprovalPersonOrganization GetApprover() => _approver;

    public ApprovalDateTime GetApprovalDateTime() => _approvalDateTime;

    /// <summary>Return entities (roots) instantiated for the part by method Init</summary>
    public ProductCategoryRelationship GetProductCategoryRelationship() => _productCategoryRelationship;

    /// <summary>Clears all fields describing entities specific to each part</summary>
    public void Clear()
    {
        _creator = null;
        _designOwner = null;
        _designSupplier = null;
        _classificationOfficer = null;
        _security = null;
        _creationDate = null;
        _classificationDate = null;
        _approval = null;

        //  myApprover.Nullify();
        //  myApprovalDateTime.Nullify();

        _productCategoryRelationship = null;
    }

    /// <summary>Initializes constant fields (shared entities)</summary>
    public void InitRoles()
    {
        _roleCreator = new PersonAndOrganizationRole();
        _roleDesignOwner = new PersonAndOrganizationRole();
        _roleDesignSupplier = new PersonAndOrganizationRole();
        _roleClassificationOfficer = new PersonAndOrganizationRole();
        _roleCreationDate = new DateTimeRole();
        _roleClassificationDate = new DateTimeRole();
        _roleApprover = new ApprovalRole();

        _roleCreator.Init(new TCollection_HAsciiString("creator"));
        _roleDesignOwner.Init(new TCollection_HAsciiString("design_owner"));
        _roleDesignSupplier.Init(new TCollection_HAsciiString("design_supplier"));
        _roleClassificationOfficer.Init(new TCollection_HAsciiString("classification_officer"));
        _roleCreationDate.Init(new TCollection_HAsciiString("creation_date"));
        _roleClassificationDate.Init(new TCollection_HAsciiString("classification_date"));
        _roleApprover.Init(new TCollection_HAsciiString("approver"));
    }

    private static PersonOrganizationItem PersonOrganizationItem(Transient value)
    {
        PersonOrganizationItem item = new PersonOrganizationItem();
        item.SetValue(value);
        return item;
    }

    private void InitPart(Part sdrTool)
    {
        if (_creator is null)
        {
            _creator = new CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<PersonOrganizationItem> items = new NCollection_HArray1<PersonOrganizationItem>(1, 2);
            items.SetValue(1, PersonOrganizationItem(sdrTool.PDF()));
            items.SetValue(2, PersonOrganizationItem(sdrTool.PD()));
            _creator.Init(DefaultPersonAndOrganization(), RoleCreator(), items);
        }

        if (_designOwner is null)
        {
            _designOwner = new CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<PersonOrganizationItem> items = new NCollection_HArray1<PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(sdrTool.Product()));
            _designOwner.Init(DefaultPersonAndOrganization(), RoleDesignOwner(), items);
        }

        if (_designSupplier is null)
        {
            _designSupplier = new CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<PersonOrganizationItem> items = new NCollection_HArray1<PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(sdrTool.PDF()));
            _designSupplier.Init(DefaultPersonAndOrganization(), RoleDesignSupplier(), items);
        }

        if (_creationDate is null)
        {
            _creationDate = new CcDesignDateAndTimeAssignment();
            NCollection_HArray1<DateTimeItem> items = new NCollection_HArray1<DateTimeItem>(1, 1);
            DateTimeItem item = new DateTimeItem();
            item.SetValue(sdrTool.PD());
            items.SetValue(1, item);
            _creationDate.Init(DefaultDateAndTime(), RoleCreationDate(), items);
        }

        if (_security is null)
        {
            TCollection_HAsciiString name = new TCollection_HAsciiString("");
            TCollection_HAsciiString purpose = new TCollection_HAsciiString("");
            SecurityClassification sc = new SecurityClassification();
            sc.Init(name, purpose, DefaultSecurityClassificationLevel());

            _security = new CcDesignSecurityClassification();
            NCollection_HArray1<ClassifiedItem> items = new NCollection_HArray1<ClassifiedItem>(1, 1);
            ClassifiedItem item = new ClassifiedItem();
            item.SetValue(sdrTool.PDF());
            items.SetValue(1, item);
            _security.Init(sc, items);
        }
        InitSecurityRequisites();

        if (_approval is null)
        {
            _approval = new CcDesignApproval();
            NCollection_HArray1<ApprovedItem> items = new NCollection_HArray1<ApprovedItem>(1, 3);
            ApprovedItem item1 = new ApprovedItem();
            item1.SetValue(sdrTool.PDF());
            items.SetValue(1, item1);
            ApprovedItem item2 = new ApprovedItem();
            item2.SetValue(sdrTool.PD());
            items.SetValue(2, item2);
            ApprovedItem item3 = new ApprovedItem();
            item3.SetValue(_security.AssignedSecurityClassification());
            items.SetValue(3, item3);
            _approval.Init(DefaultApproval(), items);
        }
        InitApprovalRequisites();

        if (_productCategoryRelationship is null)
        {
            ProductCategory pc = new ProductCategory();
            TCollection_HAsciiString pcName = new TCollection_HAsciiString("part");
            pc.Init(pcName, false, null);

            _productCategoryRelationship = new ProductCategoryRelationship();
            TCollection_HAsciiString pcrName = new TCollection_HAsciiString("");
            TCollection_HAsciiString pcrDescr = new TCollection_HAsciiString("");
            _productCategoryRelationship.Init(pcrName, true, pcrDescr, pc, sdrTool.PRPC());
        }
    }

    /// <summary>Initializes all missing data which are required for assembly</summary>
    public void InitAssembly(NextAssemblyUsageOccurrence nauo)
    {
        if (_security is null)
        {
            TCollection_HAsciiString name = new TCollection_HAsciiString("");
            TCollection_HAsciiString purpose = new TCollection_HAsciiString("");
            SecurityClassification sc = new SecurityClassification();
            sc.Init(name, purpose, DefaultSecurityClassificationLevel());

            _security = new CcDesignSecurityClassification();
            NCollection_HArray1<ClassifiedItem> items = new NCollection_HArray1<ClassifiedItem>(1, 1);
            ClassifiedItem item = new ClassifiedItem();
            item.SetValue(nauo);
            items.SetValue(1, item);
            _security.Init(sc, items);
        }
        InitSecurityRequisites();

        if (_approval is null)
        {
            _approval = new CcDesignApproval();
            NCollection_HArray1<ApprovedItem> items = new NCollection_HArray1<ApprovedItem>(1, 1);
            ApprovedItem item = new ApprovedItem();
            item.SetValue(_security.AssignedSecurityClassification());
            items.SetValue(1, item);
            _approval.Init(DefaultApproval(), items);
        }
        InitApprovalRequisites();
    }

    /// <summary>Initializes ClassificationOfficer and ClassificationDate entities according to Security entity</summary>
    public void InitSecurityRequisites()
    {
        if (_classificationOfficer is null
            || _classificationOfficer.Items().Value(1).Value() != _security.AssignedSecurityClassification())
        {
            _classificationOfficer = new CcDesignPersonAndOrganizationAssignment();
            NCollection_HArray1<PersonOrganizationItem> items = new NCollection_HArray1<PersonOrganizationItem>(1, 1);
            items.SetValue(1, PersonOrganizationItem(_security.AssignedSecurityClassification()));
            _classificationOfficer.Init(DefaultPersonAndOrganization(), RoleClassificationOfficer(), items);
        }

        if (_classificationDate is null
            || _classificationDate.Items().Value(1).Value() != _security.AssignedSecurityClassification())
        {
            _classificationDate = new CcDesignDateAndTimeAssignment();
            NCollection_HArray1<DateTimeItem> items = new NCollection_HArray1<DateTimeItem>(1, 1);
            DateTimeItem item = new DateTimeItem();
            item.SetValue(_security.AssignedSecurityClassification());
            items.SetValue(1, item);
            _classificationDate.Init(DefaultDateAndTime(), RoleClassificationDate(), items);
        }
    }

    /// <summary>Initializes Approver and ApprovalDateTime entities according to Approval entity</summary>
    public void InitApprovalRequisites()
    {
        if (_approver is null || _approver.AuthorizedApproval() != _approval.AssignedApproval())
        {
            _approver = new ApprovalPersonOrganization();
            PersonOrganizationSelect po = new PersonOrganizationSelect();
            po.SetValue(DefaultPersonAndOrganization());
            _approver.Init(po, _approval.AssignedApproval(), RoleApprover());
        }

        if (_approvalDateTime is null || _approvalDateTime.DatedApproval() != _approval.AssignedApproval())
        {
            _approvalDateTime = new ApprovalDateTime();
            DateTimeSelect dt = new DateTimeSelect();
            dt.SetValue(DefaultDateAndTime());
            _approvalDateTime.Init(dt, _approval.AssignedApproval());
        }
    }

    private Approval _defApproval;
    private DateAndTime _defDateAndTime;
    private PersonAndOrganization _defPersonAndOrganization;
    private SecurityClassificationLevel _defSecurityClassificationLevel;
    private PersonAndOrganizationRole _roleCreator;
    private PersonAndOrganizationRole _roleDesignOwner;
    private PersonAndOrganizationRole _roleDesignSupplier;
    private PersonAndOrganizationRole _roleClassificationOfficer;
    private DateTimeRole _roleCreationDate;
    private DateTimeRole _roleClassificationDate;
    private ApprovalRole _roleApprover;
    private CcDesignPersonAndOrganizationAssignment _creator;
    private CcDesignPersonAndOrganizationAssignment _designOwner;
    private CcDesignPersonAndOrganizationAssignment _designSupplier;
    private CcDesignPersonAndOrganizationAssignment _classificationOfficer;
    private CcDesignSecurityClassification _security;
    private CcDesignDateAndTimeAssignment _creationDate;
    private CcDesignDateAndTimeAssignment _classificationDate;
    private CcDesignApproval _approval;
    private ApprovalPersonOrganization _approver;
    private ApprovalDateTime _approvalDateTime;
    private ProductCategoryRelationship _productCategoryRelationship;
}
