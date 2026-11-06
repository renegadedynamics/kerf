// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Performs Checks on Entities, using General Service Library and Modules to work. Works on one Entity or on
/// a complete Model
/// </summary>
internal class CheckTool
{
    // OCCT's errh: a static (process-wide) switch between FillCheck with and without try/catch, set by each check
    // list; a field of the tool here (decision 1183)
    private int _errh = 1;

    private static void Raisecheck(FailureException exception, FrameworkCheck ach)
    {
        string mess = Sprintf("** Exception Raised during Check : %s **", global::Kerf.Engine.Foundation.OcctTypeNames.Of(exception.GetType()));
        ach.AddFail(mess);
        // OCCT rethrows the system signals (OSD_Exception, OSD_Signal), caught here as Standard_Failure; their .NET
        // counterparts (NullReferenceException...) aren't Standard_Failure, so they are never caught
    }

    //  thestat : avoids CheckSuccess redoing a previously done calculation :
    //  bit value 1 : Verify done, value 4 : and there are errors
    //  bit value 2 : Analysis done, value 8 : and there are errors

    //=================================================================================================

    /// <summary>
    /// Creates a CheckTool, by calling the General Service Library and Modules, selected through a Protocol, to
    /// work on a Model Moreover, Protocol recognizes Unknown Entities
    /// </summary>
    public CheckTool(InterfaceModel? model, Protocol? protocol)
    {
        _thegtool = new GTool(protocol, model.EntityCount);
        _theshare = new ShareTool(model, protocol);
        _thestat = 0;
    }

    /// <summary>
    /// Creates a CheckTool, by calling the General Service Library and Modules, selected through a Protocol, to
    /// work on a Model Protocol and so on are taken from the Model (its GTool)
    /// </summary>
    public CheckTool(InterfaceModel? model)
    {
        _thegtool = model.GTool();
        _theshare = new ShareTool(model, model.GTool());
        _thestat = 0;
        _thegtool.Reservate(model.EntityCount);
    }

    /// <summary>
    /// Creates a CheckTool from a Graph. The Graph contains a Model which designates a Protocol: they are used
    /// to create ShareTool
    /// </summary>
    public CheckTool(EntityGraph graph)
    {
        _thegtool = graph.Model().GTool();
        _theshare = new ShareTool(graph);

    }

    public CheckTool(HGraph? hgraph)
    {
        _thegtool = hgraph.Graph().Model().GTool();
        _theshare = new ShareTool(hgraph);

    }

    /// <summary>
    /// Fills as required a Check with the Error and Warning messages produced by Checking a given Entity. For an
    /// Erroneous or Corrected Entity : Check build at Analyse time; else, Check computed for Entity (Verify
    /// integrity), can use a Graph as required to control context
    /// </summary>
    public void FillCheck(Transient? ent, ShareTool sh, ref FrameworkCheck? ach)
    {
        GeneralModule module = null;
        int cn;
        if (_thegtool.Select(ent, out module, out cn))
        {
            //    Without try/catch (done by caller, avoids try/catch in loop)
            if (_errh == 0)
            {
                module.CheckCase(cn, ent, sh, ref ach); // OCCT: returns before the "Shared Items unknown" check
                return;
            }
            //    With try/catch
            try
            {
                module.CheckCase(cn, ent, sh, ref ach);
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                Raisecheck(Refusal.Of(exception)!, ach);
            }
        }
        else
        {
            ReportEntity rep = (ent as ReportEntity);
            if (rep is null)
            {
                return;
            }
            ach = rep.Check();
        }
        if (_theshare.Graph().HasShareErrors(ent))
        {
            ach.AddFail("** Shared Items unknown from the containing Model");
        }
    }

    /// <summary>
    /// Utility method which Prints the content of a Check
    /// </summary>
    public void Print(FrameworkCheck? ach, TextWriter s)
    {
        int i, nb;
        nb = ach.FailCount;
        if (nb > 0)
        {
            s.Write(" Fail Messages : ");
            s.Write(nb);
            s.Write(" :\n");
        }
        for (i = 1; i <= nb; i++)
        {
            s.Write(ach.Fail(i).String());
            s.Write("\n");
        }
        nb = ach.WarningCount;
        if (nb > 0)
        {
            s.Write(" Warning Messages : ");
            s.Write(nb);
            s.Write(" :\n");
        }
        for (i = 1; i <= nb; i++)
        {
            s.Write(ach.Warning(i).String());
            s.Write("\n");
        }
    }

    /// <summary>
    /// Simply Lists all the Checks and the Content (messages) and the Entity, if there is, of each Check (if all
    /// Checks are OK, nothing is Printed)
    /// </summary>
    public void Print(CheckIterator list, TextWriter s)
    {
        InterfaceModel model = _theshare.Model();
        list.Print(s, model, false);
    }

    /// <summary>
    /// Returns the Check associated to an Entity identified by its Number in a Model.
    /// </summary>
    public FrameworkCheck? Check(int num)
    {
        InterfaceModel model = _theshare.Model();
        Transient ent = model.Value(num);
        FrameworkCheck ach = new FrameworkCheck(ent); // not filtered by "Warning": as is
        _errh = 1;
        FillCheck(ent, _theshare, ref ach);
        return ach;
    }

    /// <summary>
    /// Checks if any Error has been detected (CheckList not empty) Returns normally if none, raises exception if
    /// some exists. It reuses the last computations from other checking methods, unless the argument
    /// &lt;reset&gt; is given True
    /// </summary>
    public void CheckSuccess(bool reset = false)
    {
        if (reset)
        {
            _thestat = 0;
        }
        if (_thestat > 3)
        {
            throw new CheckFailureException // already tested with error
                ("Interface Model : Global Check");
        }
        InterfaceModel model = _theshare.Model();
        if (model.GlobalCheck().FailCount > 0)
        {
            throw new CheckFailureException("Interface Model : Global Check");
        }
        FrameworkCheck modchk = new FrameworkCheck();
        model.VerifyCheck(ref modchk);
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(_theshare.Graph(), ref modchk);
        }
        if (modchk.HasFailed)
        {
            throw new CheckFailureException("Interface Model : Verify Check");
        }
        if (_thestat == 3)
        {
            return; // everything tested and it passes
        }

        _errh = 0; // No try/catch, because we precisely raise
        int nb = model.EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            if (model.IsErrorEntity(i))
            {
                throw new CheckFailureException("Interface Model : an Entity is recorded as Erroneous");
            }
            Transient ent = model.Value(i);
            if ((_thestat & 1) != 0)
            {
                if (!model.IsErrorEntity(i))
                {
                    continue; // already verify, remains analyse
                }
            }
            if ((_thestat & 2) != 0)
            {
                if (model.IsErrorEntity(i))
                {
                    continue; // already analyse, remains verify
                }
            }

            FrameworkCheck ach = new FrameworkCheck(ent);
            FillCheck(ent, _theshare, ref ach);
            if (ach.HasFailed)
            {
                throw new CheckFailureException("Interface Model : Check on an Entity has Failed");
            }
        }
    }

    /// <summary>
    /// Returns list of all "remarkable" information, which include : - GlobalCheck, if not empty - Error Checks,
    /// for all Errors (Verify + Analyse) - also Corrected Entities - and Unknown Entities : for those, each
    /// Unknown Entity is associated to an empty Check (it is neither an Error nor a Correction, but a remarkable
    /// information)
    /// </summary>
    public CheckIterator CompleteCheckList()
    {
        _thestat = 3;
        InterfaceModel model = _theshare.Model();
        CheckIterator res = new CheckIterator();
        res.SetModel(model);
        FrameworkCheck globch = model.GlobalCheck(); // GlobalCheck Statique
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(_theshare.Graph(), ref globch);
        }
        model.VerifyCheck(ref globch); // GlobalCheck Dynamique
        if (globch.HasFailed || globch.HasWarnings)
        {
            res.Add(globch, 0);
        }
        if (globch.HasFailed)
        {
            _thestat |= 12;
        }

        int i = 0, n0 = 1, nb = model.EntityCount;
        _errh = 0;
        while (n0 <= nb)
        {
            FrameworkCheck ach = new FrameworkCheck();
            Transient ent = null;
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    ach.Clear();
                    ent = model.Value(i);
                    ach.SetEntity(ent);
                    if (model.IsReportEntity(i))
                    {
                        ach = model.ReportEntity(i).Check(); // INCLUT Unknown
                        if (ach.HasFailed) // FAIL : no semantic Check
                        {
                            res.Add(ach, i);
                            ach = new FrameworkCheck();
                            _thestat |= 12;
                            continue;
                        }
                    }
                    if (!model.HasSemanticChecks)
                    {
                        FillCheck(ent, _theshare, ref ach);
                    }
                    else
                    {
                        ach.GetMessages(model.Check(i, false));
                    }
                    if (ach.HasFailed || ach.HasWarnings)
                    {
                        res.Add(ach, i);
                        ach = new FrameworkCheck();
                        if (ach.HasFailed)
                        {
                            _thestat |= 12;
                        }
                    }
                }
                n0 = nb + 1;
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                n0 = i + 1;
                Raisecheck(Refusal.Of(exception)!, ach);
                res.Add(ach, i);
                _thestat |= 12;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of all Errors detected Note that presence of Unknown Entities is not an error Cumulates :
    /// GlobalCheck if error + AnalyseCheckList + VerifyCheckList
    /// </summary>
    public CheckIterator CheckList()
    {
        _thestat = 3;
        InterfaceModel model = _theshare.Model();
        CheckIterator res = new CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.EntityCount;
        FrameworkCheck globch = model.GlobalCheck();
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(_theshare.Graph(), ref globch);
        }
        model.VerifyCheck(ref globch);
        if (globch.HasFailed)
        {
            _thestat |= 12;
            res.Add(globch, 0);
        }

        _errh = 0;
        while (n0 <= nb)
        {
            FrameworkCheck ach = new FrameworkCheck();
            Transient ent = null;
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    if (model.IsReportEntity(i))
                    {
                        ach = model.ReportEntity(i).Check();
                        if (ach.HasFailed)
                        {
                            _thestat |= 12;
                            res.Add(ach, i);
                        }
                    }
                    else
                    {
                        ent = model.Value(i);
                        ach.Clear();
                        ach.SetEntity(ent);
                        if (!model.HasSemanticChecks)
                        {
                            FillCheck(ent, _theshare, ref ach);
                        }
                        else
                        {
                            ach = model.Check(i, false);
                        }
                        if (ach.HasFailed)
                        {
                            _thestat |= 12;
                            res.Add(ach, i);
                        }
                    }
                }
                n0 = nb + 1;
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                n0 = i + 1;
                Raisecheck(Refusal.Of(exception)!, ach);
                res.Add(ach, i);
                _thestat |= 12;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of errors detected at Analyse time (syntactic) (note that GlobalCheck is not in this list)
    /// </summary>
    public CheckIterator AnalyseCheckList()
    {
        _thestat = 2;
        InterfaceModel model = _theshare.Model();
        CheckIterator res = new CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.EntityCount;

        _errh = 0;
        while (n0 <= nb)
        {
            FrameworkCheck ach = new FrameworkCheck();
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    if (!model.IsReportEntity(i))
                    {
                        continue;
                    }
                    ReportEntity rep = model.ReportEntity(i);
                    ach = rep.Check();
                    if (ach.HasFailed || ach.HasWarnings)
                    {
                        _thestat |= 8;
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                n0 = i + 1;
                Raisecheck(Refusal.Of(exception)!, ach);
                res.Add(ach, i);
                _thestat |= 8;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of integrity constraints errors (semantic) (note that GlobalCheck is not in this list)
    /// </summary>
    public CheckIterator VerifyCheckList()
    {
        _thestat = 1;
        InterfaceModel model = _theshare.Model();
        CheckIterator res = new CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.EntityCount;

        _errh = 0;
        while (n0 <= nb)
        {
            Transient ent = null;
            FrameworkCheck ach = new FrameworkCheck();
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    if (model.IsErrorEntity(i))
                    {
                        continue;
                    }
                    ent = model.Value(i);
                    ach.Clear();
                    ach.SetEntity(ent);
                    if (!model.HasSemanticChecks)
                    {
                        FillCheck(ent, _theshare, ref ach);
                    }
                    else
                    {
                        ach = model.Check(i, false);
                    }
                    if (ach.HasFailed || ach.HasWarnings)
                    {
                        _thestat |= 4;
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                n0 = i + 1;
                Raisecheck(Refusal.Of(exception)!, ach);
                res.Add(ach, i);
                _thestat |= 4;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of Corrections (includes GlobalCheck if corrected)
    /// </summary>
    public CheckIterator WarningCheckList()
    {
        _thestat = 3;
        InterfaceModel model = _theshare.Model();
        CheckIterator res = new CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.EntityCount;

        _errh = 0;
        while (n0 <= nb)
        {
            FrameworkCheck ach = new FrameworkCheck();
            Transient ent = null;
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    ach.Clear();
                    ach.SetEntity(ent); // OCCT: the working check, already added to the list for a former entity, is cleared and reused
                    if (model.IsReportEntity(i))
                    {
                        ReportEntity rep = model.ReportEntity(i);
                        if (rep.IsError)
                        {
                            _thestat |= 12;
                            continue;
                        }
                        ach = rep.Check();
                    }
                    ent = model.Value(i);
                    if (!model.HasSemanticChecks)
                    {
                        FillCheck(ent, _theshare, ref ach);
                    }
                    else
                    {
                        ach = model.Check(i, false);
                    }
                    if (ach.HasFailed)
                    {
                        _thestat |= 12;
                    }
                    else if (ach.HasWarnings)
                    {
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Exception exception) when (Refusal.IsFailure(exception))
            {
                n0 = i + 1;
                Raisecheck(Refusal.Of(exception)!, ach);
                res.Add(ach, i);
                _thestat |= 12;
            }
        }

        return res;
    }

    /// <summary>
    /// Returns list of Unknown Entities Note that Error and Erroneous Entities are not considered as Unknown
    /// </summary>
    public EntityIterator UnknownEntities()
    {
        InterfaceModel model = _theshare.Model();
        EntityIterator res = new EntityIterator();
        int nb = model.EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            if (model.IsUnknownEntity(i))
            {
                res.GetOneItem(model.Value(i));
            }
        }
        return res;
    }

    private GTool? _thegtool;
    private ShareTool _theshare;
    private int _thestat;
}
