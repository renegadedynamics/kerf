// Port of: src/ModelingData/TKBRep/BinTools/BinTools_LocationSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_LocationSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// A set of locations, indexed from 1, written and read in OCCT's binary format (the "Locations" section of a binary
/// BRep). A location of several items is stored after its datums, as the indices and powers of its items.
/// </summary>
public class BinTools_LocationSet
{
    private readonly NCollection_IndexedMap<Placement> myMap = new();

    /// <summary>An empty set of locations.</summary>
    public BinTools_LocationSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds the location (once), after its datums, and returns its index; 0 for the identity.</summary>
    public int Add(in Placement L)
    {
        if (L.IsIdentity)
        {
            return 0;
        }

        int n = myMap.FindIndex(L);
        if (n > 0)
        {
            return n;
        }

        Placement N = L;
        do
        {
            myMap.Add(new Placement(N.FirstDatum()));
            N = N.NextLocation();
        } while (!N.IsIdentity);

        return myMap.Add(L);
    }

    /// <summary>The location of index I; the identity for 0.</summary>
    public Placement Location(int I)
    {
        if (I == 0)
        {
            return new Placement();
        }
        else
        {
            return myMap[I];
        }
    }

    /// <summary>The index of the location; 0 for the identity or a location not in the set.</summary>
    public int Index(in Placement L)
    {
        if (L.IsIdentity)
        {
            return 0;
        }

        return myMap.FindIndex(L);
    }

    /// <summary>The number of locations.</summary>
    public int NbLocations() => myMap.Extent();

    // operator<<(Standard_OStream&, const gp_Trsf&)
    internal static void Write(Stream OS, in Transform3D T)
    {
        Xyz V = T.TranslationPart();
        Matrix3x3 M = T.VectorialPart();

        BinTools.PutReal(OS, M[1, 1]);
        BinTools.PutReal(OS, M[1, 2]);
        BinTools.PutReal(OS, M[1, 3]);
        BinTools.PutReal(OS, V.Coord(1));
        BinTools.PutReal(OS, M[2, 1]);
        BinTools.PutReal(OS, M[2, 2]);
        BinTools.PutReal(OS, M[2, 3]);
        BinTools.PutReal(OS, V.Coord(2));
        BinTools.PutReal(OS, M[3, 1]);
        BinTools.PutReal(OS, M[3, 2]);
        BinTools.PutReal(OS, M[3, 3]);
        BinTools.PutReal(OS, V.Coord(3));
    }

    // operator>>(Standard_IStream&, gp_Trsf&)
    private static void Read(Stream IS, ref Transform3D T)
    {
        BinTools.GetReal(IS, out double V10);
        BinTools.GetReal(IS, out double V11);
        BinTools.GetReal(IS, out double V12);
        BinTools.GetReal(IS, out double V0);

        BinTools.GetReal(IS, out double V20);
        BinTools.GetReal(IS, out double V21);
        BinTools.GetReal(IS, out double V22);
        BinTools.GetReal(IS, out double V1);

        BinTools.GetReal(IS, out double V30);
        BinTools.GetReal(IS, out double V31);
        BinTools.GetReal(IS, out double V32);
        BinTools.GetReal(IS, out double V2);

        T.SetValues(V10, V11, V12, V0, V20, V21, V22, V1, V30, V31, V32, V2);
    }

    /// <summary>Writes the set in the binary format that <see cref="Read(Stream)"/> reads.</summary>
    public void Write(Stream OS)
    {
        int i, nbLoc = myMap.Extent();
        BinTools.PutText(OS, "Locations ");
        BinTools.PutText(OS, nbLoc);
        BinTools.PutText(OS, "\n");
        try
        {
            for (i = 1; i <= nbLoc; i++)
            {
                Placement L = myMap[i];

                Placement L2 = L.NextLocation();
                bool simple = L2.IsIdentity;
                int p = L.FirstPower();
                Placement L1 = new(L.FirstDatum());
                bool elementary = (simple && p == 1);
                if (elementary)
                {
                    OS.WriteByte(1); // 1
                    Write(OS, L.Transformation());
                }
                else
                {
                    OS.WriteByte(2); // 2
                    BinTools.PutInteger(OS, myMap.FindIndex(L1));
                    BinTools.PutInteger(OS, p);
                    while (!L2.IsIdentity)
                    {
                        L1 = new Placement(L2.FirstDatum());
                        p = L2.FirstPower();
                        L2 = L2.NextLocation();
                        BinTools.PutInteger(OS, myMap.FindIndex(L1));
                        BinTools.PutInteger(OS, p);
                    }

                    BinTools.PutInteger(OS, 0);
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_LocatioSet::Write(..)", anException);
        }
    }

    /// <summary>Clears the set and reads one written by <see cref="Write(Stream)"/>.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a location table.</exception>
    public void Read(Stream IS)
    {
        myMap.Clear();

        int l1, p;

        string? buffer = BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Locations")
        {
            throw new FailureException("BinTools_LocationSet::Read: Not a location table" + "\n");
        }

        int i, nbLoc;
        nbLoc = BinTools.ReadFormattedInteger(IS);
        BinTools.Get(IS); // remove lf
        // OCCT.NET: untrusted input: a location takes at least 1 + 4 bytes
        BinTools.CheckCount(IS, nbLoc, 5, "locations");

        Placement L = new();
        Transform3D T = new();

        try
        {
            for (i = 1; i <= nbLoc; i++)
            {
                byte aTypLoc = (byte)BinTools.Get(IS);
                if (aTypLoc == 1)
                {
                    Read(IS, ref T);
                    L = new Placement(T);
                }
                else if (aTypLoc == 2)
                {
                    L = new Placement();
                    BinTools.GetInteger(IS, out l1); // Index
                    while (l1 != 0)
                    {
                        BinTools.GetInteger(IS, out p);
                        // OCCT.NET: untrusted input: OCCT reads outside its map
                        BinTools.CheckIndex(l1, 1, myMap.Extent(), "location");
                        Placement L1 = myMap[l1];
                        L = BinTools.PoweredTimes(L1, p, L);
                        BinTools.GetInteger(IS, out l1);
                    }
                }
                else
                {
                    throw new FailureException("Unexpected location's type = " + (char)aTypLoc + "\n");
                }

                if (!L.IsIdentity)
                {
                    myMap.Add(L);
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_LocationSet::Read(..)", anException);
        }
    }
}
