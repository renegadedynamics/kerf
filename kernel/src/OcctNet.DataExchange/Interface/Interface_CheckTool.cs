// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Performs Checks on Entities, using General Service Library and Modules to work. Works on one Entity or on
/// a complete Model
/// </summary>
public class Interface_CheckTool
{
    // OCCT's errh: a static (process-wide) switch between FillCheck with and without try/catch, set by each check
    // list; a field of the tool here (decision 12103)
    private int errh = 1;

    private static void raisecheck(Standard_Failure theException, Interface_Check ach)
    {
        string mess = Sprintf("** Exception Raised during Check : %s **", theException.GetType().Name);
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
    public Interface_CheckTool(Interface_InterfaceModel? model, Interface_Protocol? protocol)
    {
        thegtool = new Interface_GTool(protocol, model.NbEntities());
        theshare = new Interface_ShareTool(model, protocol);
        thestat = 0;
    }

    /// <summary>
    /// Creates a CheckTool, by calling the General Service Library and Modules, selected through a Protocol, to
    /// work on a Model Protocol and so on are taken from the Model (its GTool)
    /// </summary>
    public Interface_CheckTool(Interface_InterfaceModel? model)
    {
        thegtool = model.GTool();
        theshare = new Interface_ShareTool(model, model.GTool());
        thestat = 0;
        thegtool.Reservate(model.NbEntities());
    }

    /// <summary>
    /// Creates a CheckTool from a Graph. The Graph contains a Model which designates a Protocol: they are used
    /// to create ShareTool
    /// </summary>
    public Interface_CheckTool(Interface_Graph graph)
    {
        thegtool = graph.Model().GTool();
        theshare = new Interface_ShareTool(graph);

    }

    public Interface_CheckTool(Interface_HGraph? hgraph)
    {
        thegtool = hgraph.Graph().Model().GTool();
        theshare = new Interface_ShareTool(hgraph);

    }

    /// <summary>
    /// Fills as required a Check with the Error and Warning messages produced by Checking a given Entity. For an
    /// Erroneous or Corrected Entity : Check build at Analyse time; else, Check computed for Entity (Verify
    /// integrity), can use a Graph as required to control context
    /// </summary>
    public void FillCheck(Standard_Transient? ent, Interface_ShareTool sh, ref Interface_Check? ach)
    {
        Interface_GeneralModule module = null;
        int CN;
        if (thegtool.Select(ent, out module, out CN))
        {
            //    Without try/catch (done by caller, avoids try/catch in loop)
            if (errh == 0)
            {
                module.CheckCase(CN, ent, sh, ref ach); // OCCT: returns before the "Shared Items unknown" check
                return;
            }
            //    With try/catch
            try
            {
                module.CheckCase(CN, ent, sh, ref ach);
            }
            catch (Standard_Failure anException)
            {
                raisecheck(anException, ach);
            }
        }
        else
        {
            Interface_ReportEntity rep = (ent as Interface_ReportEntity);
            if (rep is null)
            {
                return;
            }
            ach = rep.Check();
        }
        if (theshare.Graph().HasShareErrors(ent))
        {
            ach.AddFail("** Shared Items unknown from the containing Model");
        }
    }

    /// <summary>
    /// Utility method which Prints the content of a Check
    /// </summary>
    public void Print(Interface_Check? ach, TextWriter S)
    {
        int i, nb;
        nb = ach.NbFails();
        if (nb > 0)
        {
            S.Write(" Fail Messages : ");
            S.Write(nb);
            S.Write(" :\n");
        }
        for (i = 1; i <= nb; i++)
        {
            S.Write(ach.Fail(i).String());
            S.Write("\n");
        }
        nb = ach.NbWarnings();
        if (nb > 0)
        {
            S.Write(" Warning Messages : ");
            S.Write(nb);
            S.Write(" :\n");
        }
        for (i = 1; i <= nb; i++)
        {
            S.Write(ach.Warning(i).String());
            S.Write("\n");
        }
    }

    /// <summary>
    /// Simply Lists all the Checks and the Content (messages) and the Entity, if there is, of each Check (if all
    /// Checks are OK, nothing is Printed)
    /// </summary>
    public void Print(Interface_CheckIterator list, TextWriter S)
    {
        Interface_InterfaceModel model = theshare.Model();
        list.Print(S, model, false);
    }

    /// <summary>
    /// Returns the Check associated to an Entity identified by its Number in a Model.
    /// </summary>
    public Interface_Check? Check(int num)
    {
        Interface_InterfaceModel model = theshare.Model();
        Standard_Transient ent = model.Value(num);
        Interface_Check ach = new Interface_Check(ent); // not filtered by "Warning": as is
        errh = 1;
        FillCheck(ent, theshare, ref ach);
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
            thestat = 0;
        }
        if (thestat > 3)
        {
            throw new Interface_CheckFailure // already tested with error
                ("Interface Model : Global Check");
        }
        Interface_InterfaceModel model = theshare.Model();
        if (model.GlobalCheck().NbFails() > 0)
        {
            throw new Interface_CheckFailure("Interface Model : Global Check");
        }
        Interface_Check modchk = new Interface_Check();
        model.VerifyCheck(ref modchk);
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(theshare.Graph(), ref modchk);
        }
        if (modchk.HasFailed())
        {
            throw new Interface_CheckFailure("Interface Model : Verify Check");
        }
        if (thestat == 3)
        {
            return; // everything tested and it passes
        }

        errh = 0; // No try/catch, because we precisely raise
        int nb = model.NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            if (model.IsErrorEntity(i))
            {
                throw new Interface_CheckFailure("Interface Model : an Entity is recorded as Erroneous");
            }
            Standard_Transient ent = model.Value(i);
            if ((thestat & 1) != 0)
            {
                if (!model.IsErrorEntity(i))
                {
                    continue; // already verify, remains analyse
                }
            }
            if ((thestat & 2) != 0)
            {
                if (model.IsErrorEntity(i))
                {
                    continue; // already analyse, remains verify
                }
            }

            Interface_Check ach = new Interface_Check(ent);
            FillCheck(ent, theshare, ref ach);
            if (ach.HasFailed())
            {
                throw new Interface_CheckFailure("Interface Model : Check on an Entity has Failed");
            }
        }
    }

    /// <summary>
    /// Returns list of all "remarkable" information, which include : - GlobalCheck, if not empty - Error Checks,
    /// for all Errors (Verify + Analyse) - also Corrected Entities - and Unknown Entities : for those, each
    /// Unknown Entity is associated to an empty Check (it is neither an Error nor a Correction, but a remarkable
    /// information)
    /// </summary>
    public Interface_CheckIterator CompleteCheckList()
    {
        thestat = 3;
        Interface_InterfaceModel model = theshare.Model();
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(model);
        Interface_Check globch = model.GlobalCheck(); // GlobalCheck Statique
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(theshare.Graph(), ref globch);
        }
        model.VerifyCheck(ref globch); // GlobalCheck Dynamique
        if (globch.HasFailed() || globch.HasWarnings())
        {
            res.Add(globch, 0);
        }
        if (globch.HasFailed())
        {
            thestat |= 12;
        }

        int i = 0, n0 = 1, nb = model.NbEntities();
        errh = 0;
        while (n0 <= nb)
        {
            Interface_Check ach = new Interface_Check();
            Standard_Transient ent = null;
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
                        if (ach.HasFailed()) // FAIL : no semantic Check
                        {
                            res.Add(ach, i);
                            ach = new Interface_Check();
                            thestat |= 12;
                            continue;
                        }
                    }
                    if (!model.HasSemanticChecks())
                    {
                        FillCheck(ent, theshare, ref ach);
                    }
                    else
                    {
                        ach.GetMessages(model.Check(i, false));
                    }
                    if (ach.HasFailed() || ach.HasWarnings())
                    {
                        res.Add(ach, i);
                        ach = new Interface_Check();
                        if (ach.HasFailed())
                        {
                            thestat |= 12;
                        }
                    }
                }
                n0 = nb + 1;
            }
            catch (Standard_Failure anException)
            {
                n0 = i + 1;
                raisecheck(anException, ach);
                res.Add(ach, i);
                thestat |= 12;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of all Errors detected Note that presence of Unknown Entities is not an error Cumulates :
    /// GlobalCheck if error + AnalyseCheckList + VerifyCheckList
    /// </summary>
    public Interface_CheckIterator CheckList()
    {
        thestat = 3;
        Interface_InterfaceModel model = theshare.Model();
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.NbEntities();
        Interface_Check globch = model.GlobalCheck();
        if (model.Protocol() is not null)
        {
            model.Protocol().GlobalCheck(theshare.Graph(), ref globch);
        }
        model.VerifyCheck(ref globch);
        if (globch.HasFailed())
        {
            thestat |= 12;
            res.Add(globch, 0);
        }

        errh = 0;
        while (n0 <= nb)
        {
            Interface_Check ach = new Interface_Check();
            Standard_Transient ent = null;
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    if (model.IsReportEntity(i))
                    {
                        ach = model.ReportEntity(i).Check();
                        if (ach.HasFailed())
                        {
                            thestat |= 12;
                            res.Add(ach, i);
                        }
                    }
                    else
                    {
                        ent = model.Value(i);
                        ach.Clear();
                        ach.SetEntity(ent);
                        if (!model.HasSemanticChecks())
                        {
                            FillCheck(ent, theshare, ref ach);
                        }
                        else
                        {
                            ach = model.Check(i, false);
                        }
                        if (ach.HasFailed())
                        {
                            thestat |= 12;
                            res.Add(ach, i);
                        }
                    }
                }
                n0 = nb + 1;
            }
            catch (Standard_Failure anException)
            {
                n0 = i + 1;
                raisecheck(anException, ach);
                res.Add(ach, i);
                thestat |= 12;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of errors detected at Analyse time (syntactic) (note that GlobalCheck is not in this list)
    /// </summary>
    public Interface_CheckIterator AnalyseCheckList()
    {
        thestat = 2;
        Interface_InterfaceModel model = theshare.Model();
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.NbEntities();

        errh = 0;
        while (n0 <= nb)
        {
            Interface_Check ach = new Interface_Check();
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    if (!model.IsReportEntity(i))
                    {
                        continue;
                    }
                    Interface_ReportEntity rep = model.ReportEntity(i);
                    ach = rep.Check();
                    if (ach.HasFailed() || ach.HasWarnings())
                    {
                        thestat |= 8;
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Standard_Failure anException)
            {
                n0 = i + 1;
                raisecheck(anException, ach);
                res.Add(ach, i);
                thestat |= 8;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of integrity constraints errors (semantic) (note that GlobalCheck is not in this list)
    /// </summary>
    public Interface_CheckIterator VerifyCheckList()
    {
        thestat = 1;
        Interface_InterfaceModel model = theshare.Model();
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.NbEntities();

        errh = 0;
        while (n0 <= nb)
        {
            Standard_Transient ent = null;
            Interface_Check ach = new Interface_Check();
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
                    if (!model.HasSemanticChecks())
                    {
                        FillCheck(ent, theshare, ref ach);
                    }
                    else
                    {
                        ach = model.Check(i, false);
                    }
                    if (ach.HasFailed() || ach.HasWarnings())
                    {
                        thestat |= 4;
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Standard_Failure anException)
            {
                n0 = i + 1;
                raisecheck(anException, ach);
                res.Add(ach, i);
                thestat |= 4;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns list of Corrections (includes GlobalCheck if corrected)
    /// </summary>
    public Interface_CheckIterator WarningCheckList()
    {
        thestat = 3;
        Interface_InterfaceModel model = theshare.Model();
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(model);
        int i = 0, n0 = 1, nb = model.NbEntities();

        errh = 0;
        while (n0 <= nb)
        {
            Interface_Check ach = new Interface_Check();
            Standard_Transient ent = null;
            try
            {
                for (i = n0; i <= nb; i++)
                {
                    ach.Clear();
                    ach.SetEntity(ent); // OCCT: the working check, already added to the list for a former entity, is cleared and reused
                    if (model.IsReportEntity(i))
                    {
                        Interface_ReportEntity rep = model.ReportEntity(i);
                        if (rep.IsError())
                        {
                            thestat |= 12;
                            continue;
                        }
                        ach = rep.Check();
                    }
                    ent = model.Value(i);
                    if (!model.HasSemanticChecks())
                    {
                        FillCheck(ent, theshare, ref ach);
                    }
                    else
                    {
                        ach = model.Check(i, false);
                    }
                    if (ach.HasFailed())
                    {
                        thestat |= 12;
                    }
                    else if (ach.HasWarnings())
                    {
                        res.Add(ach, i);
                    }
                }
                n0 = nb + 1;
            }
            catch (Standard_Failure anException)
            {
                n0 = i + 1;
                raisecheck(anException, ach);
                res.Add(ach, i);
                thestat |= 12;
            }
        }

        return res;
    }

    /// <summary>
    /// Returns list of Unknown Entities Note that Error and Erroneous Entities are not considered as Unknown
    /// </summary>
    public Interface_EntityIterator UnknownEntities()
    {
        Interface_InterfaceModel model = theshare.Model();
        Interface_EntityIterator res = new Interface_EntityIterator();
        int nb = model.NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            if (model.IsUnknownEntity(i))
            {
                res.GetOneItem(model.Value(i));
            }
        }
        return res;
    }

    private Interface_GTool? thegtool;
    private Interface_ShareTool theshare;
    private int thestat;
}
