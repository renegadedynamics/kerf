// Port of: src/ModelingData/TKBRep/BinTools/BinTools_LocationSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_LocationSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A set of locations, indexed from 1, written and read in OCCT's binary format (the "Locations" section of a binary
/// BRep). A location of several items is stored after its datums, as the indices and powers of its items.
/// </summary>
internal class BinaryLocationSet
{
    private readonly IndexedSet<Placement> _map = new(PlacementIdentityComparer.Instance);

    /// <summary>An empty set of locations.</summary>
    public BinaryLocationSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds the location (once), after its datums, and returns its index; 0 for the identity.</summary>
    public int Add(in Placement l)
    {
        if (l.IsIdentity)
        {
            return 0;
        }

        int n = _map.IndexOf(l) + 1;
        if (n > 0)
        {
            return n;
        }

        Placement N = l;
        do
        {
            _map.Add(new Placement(N.FirstDatum()));
            N = N.NextLocation();
        } while (!N.IsIdentity);

        return _map.Add(l) + 1;
    }

    /// <summary>The location of index I; the identity for 0.</summary>
    public Placement Location(int i)
    {
        if (i == 0)
        {
            return new Placement();
        }
        else
        {
            return _map[i - 1];
        }
    }

    /// <summary>The index of the location; 0 for the identity or a location not in the set.</summary>
    public int Index(in Placement l)
    {
        if (l.IsIdentity)
        {
            return 0;
        }

        return _map.IndexOf(l) + 1;
    }

    /// <summary>The number of locations.</summary>
    public int LocationCount => _map.Count;

    // operator<<(Standard_OStream&, const gp_Trsf&)
    internal static void Write(Stream os, in Transform3D t)
    {
        Xyz v = t.TranslationXyz();
        Matrix3x3 m = t.VectorialPart;

        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[1, 1]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[1, 2]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[1, 3]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, v.Coord(1));
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[2, 1]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[2, 2]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[2, 3]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, v.Coord(2));
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[3, 1]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[3, 2]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, m[3, 3]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, v.Coord(3));
    }

    // operator>>(Standard_IStream&, gp_Trsf&)
    private static void Read(Stream IS, ref Transform3D t)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v10);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v11);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v12);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v0);

        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v20);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v21);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v22);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v1);

        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v30);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v31);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v32);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v2);

        t = t.WithValues(v10, v11, v12, v0, v20, v21, v22, v1, v30, v31, v32, v2);
    }

    /// <summary>Writes the set in the binary format that <see cref="Read(Stream)"/> reads.</summary>
    public void Write(Stream os)
    {
        int i, nbLoc = _map.Count;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Locations ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbLoc);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        try
        {
            for (i = 1; i <= nbLoc; i++)
            {
                Placement l = _map[i - 1];

                Placement l2 = l.NextLocation();
                bool simple = l2.IsIdentity;
                int p = l.FirstPower();
                Placement l1 = new(l.FirstDatum());
                bool elementary = (simple && p == 1);
                if (elementary)
                {
                    os.WriteByte(1); // 1
                    Write(os, l.Transformation);
                }
                else
                {
                    os.WriteByte(2); // 2
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _map.IndexOf(l1) + 1);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, p);
                    while (!l2.IsIdentity)
                    {
                        l1 = new Placement(l2.FirstDatum());
                        p = l2.FirstPower();
                        l2 = l2.NextLocation();
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _map.IndexOf(l1) + 1);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, p);
                    }

                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, 0);
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_LocatioSet::Write(..)", exception);
        }
    }

    /// <summary>Clears the set and reads one written by <see cref="Write(Stream)"/>.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a location table.</exception>
    public void Read(Stream IS)
    {
        _map.Clear();

        int l1, p;

        string? buffer = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Locations")
        {
            throw new FailureException("BinTools_LocationSet::Read: Not a location table" + "\n");
        }

        int i, nbLoc;
        nbLoc = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove lf
        // OCCT.NET: untrusted input: a location takes at least 1 + 4 bytes
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbLoc, 5, "locations");

        Placement l = new();
        Transform3D t = new();

        try
        {
            for (i = 1; i <= nbLoc; i++)
            {
                byte typLoc = (byte)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
                if (typLoc == 1)
                {
                    Read(IS, ref t);
                    l = new Placement(t);
                }
                else if (typLoc == 2)
                {
                    l = new Placement();
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l1); // Index
                    while (l1 != 0)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out p);
                        // OCCT.NET: untrusted input: OCCT reads outside its map
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l1, 1, _map.Count, "location");
                        Placement L1 = _map[l1 - 1];
                        l = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PoweredTimes(L1, p, l);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l1);
                    }
                }
                else
                {
                    throw new FailureException("Unexpected location's type = " + (char)typLoc + "\n");
                }

                if (!l.IsIdentity)
                {
                    _map.Add(l);
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_LocationSet::Read(..)", exception);
        }
    }
}
