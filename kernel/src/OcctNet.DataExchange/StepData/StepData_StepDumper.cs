// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepDumper.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepDumper.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Provides a way to dump entities processed through STEP, with these features: same form as for writing a STEP File
/// (because it is clear and compact enough, even if the names of the fields do not appear): thus, no additional
/// resource is required; possibility to look for an entity itself (only its Type or with its content), an entity and
/// it shared items (one level) or all the entities its refers to, directly or recursively.
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class StepData_StepDumper
{
    private readonly StepData_StepModel themodel;
    private readonly Interface_GeneralLib theslib;
    private readonly StepData_WriterLib thewlib;
    private readonly StepData_StepWriter thewriter;

    // Constructor: Initialize STEP dumper with model, protocol, and output mode
    // mode > 0 sets label mode to 2 for enhanced entity labeling

    /// <summary>
    /// Creates a StepDumper, able to work on a given StepModel (which defines the total scope for dumping entities)
    /// and a given Protocol from Step (which defines the authorized types to be dumped). mode commands what is to be
    /// displayed (number or label): 0 for number (and corresponding labels are displayed apart), 1 for label (and
    /// corresponding numbers are displayed apart), 2 for label without anymore.
    /// </summary>
    public StepData_StepDumper(StepData_StepModel amodel, StepData_Protocol protocol, int mode = 0)
    {
        theslib = new Interface_GeneralLib(protocol);
        thewlib = new StepData_WriterLib(protocol);
        thewriter = new StepData_StepWriter(amodel);
        themodel = amodel;
        if (mode > 0)
        {
            thewriter.LabelMode() = 2;
        }
    }

    /// <summary>Gives an access to the tool which is used to work: this allow to acts on some parameters: FloatWriter, ScopeMode, etc.</summary>
    public StepData_StepWriter StepWriter()
    {
        return thewriter;
    }

    // Main dump method: outputs entity information to stream with different levels of detail
    // level <= 0: basic entity type and identifier information only
    // level == 1: entity identifiers and basic structure
    // level > 1:  complete entity data with all referenced entities

    /// <summary>
    /// Dumps a Entity on an Messenger. Returns True if success, False, if the entity to dump has not been recognized
    /// by the Protocol. level can have one of these values: 0: prints the TYPE only, as known in STEP Files
    /// (StepType); 1: dumps the entity, completely (whatever it has inside); 2: dumps the entity, plus the entities it
    /// refers to (completely).
    /// </summary>
    public bool Dump(TextWriter S, Transient? ent, int level)
    {
        int i, nb = themodel.NbEntities();
        NCollection_Array1<int> ids = new NCollection_Array1<int>(0, nb); // Array to store entity identifiers
        ids.Init(0);
        int num = themodel.Number(ent); // Entity number in model
        int nlab = themodel.IdentLabel(ent); // Entity identifier label
        ids.SetValue(num, (nlab > 0 ? nlab : -1));

        if (level <= 0)
        {
            // Basic output: show entity number and type information only
            if (num > 0)
            {
                S.Write("#" + num + " = ");
            }
            else
            {
                S.Write("#??? = "); // Unknown entity number
            }
            if (thewlib.Select(ent, out StepData_ReadWriteModule module, out int CN))
            {
                if (module.IsComplex(CN))
                {
                    NCollection_Sequence<string> listypes = new NCollection_Sequence<string>();
                    if (!module.ComplexType(CN, listypes))
                    {
                        S.Write("(Complex Type : ask level > 0) cdl = " + ent.DynamicTypeName() + " (...);" + '\n');
                    }
                    else
                    {
                        int n = listypes.Length();
                        for (i = 1; i <= n; i++)
                        {
                            S.Write(listypes.Value(i) + " (...)");
                        }
                        S.Write('\n');
                    }
                }
                else
                {
                    S.Write(module.StepType(CN) + " (...);" + '\n');
                }
            }
            else
            {
                S.Write("(Unrecognized Type for protocol) cdl = " + ent.DynamicTypeName() + " (...);" + '\n');
            }
            if (nlab > 0)
            {
                S.Write("/*   Ident in file for " + num + " : #" + nlab + "   */" + '\n');
            }
        }
        else if (level == 1)
        {
            // Collect entity identifiers
            Transient anent;
            if (theslib.Select(ent, out Interface_GeneralModule module, out int CN))
            {
                Interface_EntityIterator iter = new Interface_EntityIterator();
                module.FillSharedCase(CN, ent, iter);
                module.ListImpliedCase(CN, ent, iter); // accumulate entities
                for (; iter.More(); iter.Next())
                {
                    anent = iter.Value();
                    nlab = themodel.IdentLabel(anent);
                    ids.SetValue(themodel.Number(anent), (nlab > 0 ? nlab : -1));
                }
            }
            // Send entity to writer
            thewriter.SendEntity(num, thewlib);
        }
        else
        {
            Transient anent;
            // Debug output: dumping entity number
            // Send entities to writer
            NCollection_Array1<int> tab = new NCollection_Array1<int>(0, nb);
            tab.Init(0);
            tab.SetValue(num, 1);
            if (theslib.Select(ent, out Interface_GeneralModule module, out int CN))
            {
                Interface_EntityIterator iter = new Interface_EntityIterator();
                module.FillSharedCase(CN, ent, iter);
                module.ListImpliedCase(CN, ent, iter); // accumulate entities
                for (; iter.More(); iter.Next())
                {
                    tab.SetValue(themodel.Number(iter.Value()), 1);
                }
            }
            for (i = 1; i <= nb; i++)
            {
                // Process entity identifiers list
                if (tab.Value(i) == 0)
                {
                    continue;
                }
                anent = themodel.Value(i);
                thewriter.SendEntity(i, thewlib);
                if (theslib.Select(anent, out module, out CN))
                {
                    Interface_EntityIterator iter = new Interface_EntityIterator();
                    module.FillSharedCase(CN, anent, iter);
                    module.ListImpliedCase(CN, anent, iter); // accumulate entities
                    for (; iter.More(); iter.Next())
                    {
                        anent = iter.Value();
                        nlab = themodel.IdentLabel(anent);
                        ids.SetValue(themodel.Number(anent), (nlab > 0 ? nlab : -1));
                    }
                }
            }
        }

        // Display entity identifiers if present
        // Count different types of identifiers: distinct (nbi), matching entity number (nbq),
        // without identifier (nbu), total entities with identifiers (nbe)
        int nbi = 0, nbe = 0, nbq = 0, nbu = 0;
        for (i = 1; i <= nb; i++)
        {
            nlab = ids.Value(i);
            if (nlab == 0)
            {
                continue; // Skip entities without identifiers
            }
            nbe++; // Count entities with identifiers
            if (nlab < 0)
            {
                nbu = 0; // OCCT: reset, not counted (so never listed) - Entities without proper identifier
            }
            else if (nlab == i)
            {
                nbq = 0; // OCCT: reset, not counted - Entities where identifier matches entity number
            }
            else if (nlab > 0)
            {
                nbi++; // Entities with distinct proper identifiers
            }
        }
        if (nbe > 0)
        {
            // Debug statistics: displayed entities, matching identifiers, distinct identifiers
            if (nbu > 0)
            {
                S.Write(" (no ident): ");
                for (i = 1; i <= nb; i++)
                {
                    if (ids.Value(i) >= 0)
                    {
                        continue;
                    }
                    S.Write(" #" + i);
                }
                S.Write('\n');
            }
            if (nbq > 0)
            {
                S.Write(" (ident = num): ");
                for (i = 1; i <= nb; i++)
                {
                    if (ids.Value(i) == i)
                    {
                        S.Write(" #" + i);
                    }
                }
                S.Write('\n');
            }
            // OCCT: the "(proper ident)" listing is under nbi < 0, never true: left out
            if (nbi > 0)
            {
                S.Write("In dump, iii:#jjj means : entity rank iii has step ident #jjj" + '\n');
            }
            // Debug output: entity dumping information with level details
        }
        if (level > 0)
        {
            thewriter.Print(S);
        }
        return true;
    }

    /// <summary>Works as Dump with a Transient, but directly takes the entity designated in the model by num.</summary>
    public bool Dump(TextWriter S, int num, int level)
    {
        if (num <= 0 || num > themodel.NbEntities())
        {
            return false;
        }
        Transient ent = themodel.Value(num);
        return Dump(S, ent, level);
    }
}
