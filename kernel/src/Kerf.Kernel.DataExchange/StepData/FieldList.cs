// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Describes a list of fields, in a general way. This basic class is for a null size list. Subclasses are for 1, N (fixed) or Dynamic sizes.</summary>
internal class FieldList
{
    /// <summary>Creates a FieldList of 0 Field.</summary>
    public FieldList()
    {
    }

    /// <summary>Returns the count of fields. Here, returns 0.</summary>
    public virtual int FieldCount
    {
        get
        {
            return 0;
        }
    }

    /// <summary>Returns the field n0 num between 1 and NbFields (read only).</summary>
    public virtual Field Field(int num)
    {
        throw new OutOfRangeException("StepData_FieldList : Field");
    }

    /// <summary>Returns the field n0 num between 1 and NbFields, in order to modify its content.</summary>
    public virtual Field CField(int num)
    {
        throw new OutOfRangeException("StepData_FieldList : CField");
    }

    /// <summary>Fills an iterator with the entities shared by this list.</summary>
    public void FillShared(EntityIterator iter)
    {
        int i, nb = FieldCount;
        for (i = 1; i <= nb; i++)
        {
            Field fi = Field(i);
            if (fi.Kind() != 7)
            {
                continue; // KindEntity
            }
            int i1, i2, nb1 = 1, nb2 = 1, ari = fi.Arity();
            if (ari == 1)
            {
                nb1 = fi.Length();
            }
            if (ari == 2)
            {
                nb1 = fi.Length(1);
                nb2 = fi.Length(2);
            }
            for (i1 = 1; i1 <= nb1; i1++)
            {
                for (i2 = 1; i2 <= nb2; i2++)
                {
                    iter.AddItem(fi.Entity(i1, i2));
                }
            }
        }
    }
}
