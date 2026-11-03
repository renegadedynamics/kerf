// Port of: src/ModelingData/TKBRep/TopTools/TopTools_LocationSet.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools_LocationSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of locations, indexed from 1, written and read in OCCT's text format (the "Locations" section of a
/// <c>.brep</c> file). Adding a location adds the elementary locations it is made of first, so a complex location is
/// written as the indices and powers of its factors.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Add"/> and <see cref="Read"/> change the set. The locations themselves are
/// values. <see cref="Read"/> rejects input it can't read safely with <see cref="StreamFormatErrorException"/>
/// (docs/phase3-design.md, section 7).
/// </remarks>
public class LocationSet
{
    /// <summary>
    /// The most items a location read from a stream may have (factors, each power of a complex location counted):
    /// <c>TopLoc_Location</c>'s product and power recurse once per item.
    /// </summary>
    public const int MaxLocationItems = 1000;

    private readonly IndexedSet<Placement> _map = new(PlacementIdentityComparer.Instance);

    /// <summary>Returns an empty set of locations.</summary>
    public LocationSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>
    /// Adds the location (and, first, the elementary locations it is made of) and returns its index; 0 for the
    /// identity.
    /// </summary>
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

    /// <summary>The location of index I; the identity if I is out of range.</summary>
    public Placement Location(int i)
    {
        if (i <= 0 || i > _map.Count)
        {
            return new Placement();
        }

        return _map[i - 1];
    }

    /// <summary>The index of the location, 0 for the identity or a location that isn't in the set.</summary>
    public int Index(in Placement l)
    {
        if (l.IsIdentity)
        {
            return 0;
        }

        return _map.IndexOf(l) + 1;
    }

    /// <summary>The number of locations in the set.</summary>
    public int LocationCount => _map.Count;

    private static void WriteTrsf(in Transform3D t, TextWriter os, bool compact)
    {
        Xyz v = t.TranslationXyz();
        Matrix3x3 m = t.VectorialPart;

        if (!compact)
        {
            os.Write("( ");
        }

        os.Write(Setw15(os, m[1, 1]) + " ");
        os.Write(Setw15(os, m[1, 2]) + " ");
        os.Write(Setw15(os, m[1, 3]) + " ");
        os.Write(Setw15(os, v.Coord(1)) + " ");
        if (!compact)
        {
            os.Write(" )");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("( ");
        }

        os.Write(Setw15(os, m[2, 1]) + " ");
        os.Write(Setw15(os, m[2, 2]) + " ");
        os.Write(Setw15(os, m[2, 3]) + " ");
        os.Write(Setw15(os, v.Coord(2)) + " ");
        if (!compact)
        {
            os.Write(" )");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("( ");
        }

        os.Write(Setw15(os, m[3, 1]) + " ");
        os.Write(Setw15(os, m[3, 2]) + " ");
        os.Write(Setw15(os, m[3, 3]) + " ");
        os.Write(Setw15(os, v.Coord(3)) + " ");
        if (!compact)
        {
            os.Write(" )");
        }

        os.Write("\n");
    }

    private static string Str(int value) => value.ToString(System.Globalization.CultureInfo.InvariantCulture);

    // OS << std::setw(15) << theValue
    private static string Setw15(TextWriter os, double value)
        => StreamUtilities.Setw(15, StreamUtilities.FormatReal(value, StreamUtilities.Precision(os)));

    /// <summary>Dumps the locations in a readable form.</summary>
    public void Dump(TextWriter os)
    {
        int i, nbLoc = _map.Count;

        os.Write("\n\n");
        os.Write("\n -------");
        os.Write("\n Dump of " + Str(nbLoc) + " Locations");
        os.Write("\n -------\n\n");

        for (i = 1; i <= nbLoc; i++)
        {
            Placement l = _map[i - 1];
            os.Write(StreamUtilities.Setw(5, Str(i)) + " : \n");

            Placement l2 = l.NextLocation();
            bool simple = l2.IsIdentity;
            int p = l.FirstPower();
            Placement l1 = new(l.FirstDatum());
            bool elementary = (simple && p == 1);
            if (elementary)
            {
                os.Write("Elementary location\n");
            }
            else
            {
                os.Write("Complex : L" + Str(_map.IndexOf(l1) + 1));
                if (p != 1)
                {
                    os.Write("^" + Str(p));
                }

                while (!l2.IsIdentity)
                {
                    l1 = new Placement(l2.FirstDatum());
                    p = l2.FirstPower();
                    l2 = l2.NextLocation();
                    os.Write(" * L" + Str(_map.IndexOf(l1) + 1));
                    if (p != 1)
                    {
                        os.Write("^" + Str(p));
                    }
                }

                os.Write("\n");
            }

            WriteTrsf(l.Transformation, os, false);
        }
    }

    /// <summary>
    /// Writes the set in a format <see cref="Read"/> reads: an elementary location as its transformation (15
    /// significant digits), a complex one as the indices and powers of its factors.
    /// </summary>
    public void Write(TextWriter os, ProgressRange progress = default)
    {
        int prec = StreamUtilities.Precision(os, 15);

        int i, nbLoc = _map.Count;
        os.Write("Locations " + Str(nbLoc) + "\n");

        // OCC19559
        using var ps = new ProgressScope(progress, "Locations", nbLoc);
        for (i = 1; i <= nbLoc && ps.More(); i++, ps.Next())
        {
            Placement l = _map[i - 1];

            Placement l2 = l.NextLocation();
            bool simple = l2.IsIdentity;
            int p = l.FirstPower();
            Placement l1 = new(l.FirstDatum());
            bool elementary = (simple && p == 1);
            if (elementary)
            {
                os.Write("1\n");
                WriteTrsf(l.Transformation, os, true);
            }
            else
            {
                os.Write("2 ");
                os.Write(" " + Str(_map.IndexOf(l1) + 1) + " " + Str(p));
                while (!l2.IsIdentity)
                {
                    l1 = new Placement(l2.FirstDatum());
                    p = l2.FirstPower();
                    l2 = l2.NextLocation();
                    os.Write(" " + Str(_map.IndexOf(l1) + 1) + " " + Str(p));
                }

                os.Write(" 0\n");
            }
        }

        StreamUtilities.Precision(os, prec);
    }

    private static void ReadTrsf(ref Transform3D t, TextReader IS)
    {
        double v10, v11, v12, v20, v21, v22, v30, v31, v32;
        double v0, v1, v2;

        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v10);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v11);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v12);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v0);

        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v20);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v21);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v22);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v1);

        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v30);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v31);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v32);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v2);

        t = t.WithValues(v10, v11, v12, v0, v20, v21, v22, v1, v30, v31, v32, v2);
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write"/>. Writes "Not a location table" to the console, as
    /// OCCT does, if the text doesn't start with "Locations".
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely (docs/phase3-design.md, section
    /// 7).</exception>
    /// <exception cref="ConstructionErrorException">An elementary location isn't a transformation (OCCT's
    /// <c>gp_Trsf::SetValues</c> throws).</exception>
    public void Read(TextReader IS, ProgressRange progress = default)
    {
        _map.Clear();

        char[] buffer = new char[255];
        int l1 = 0, p = 0;

        StreamUtilities.ReadWord(IS, buffer);
        if (StreamUtilities.CString(buffer) != "Locations")
        {
            Console.Out.Write("Not a location table " + '\n');
            return;
        }

        // OCCT's nbLoc is uninitialised if the count can't be read
        int i, nbLoc = 0;
        StreamUtilities.Read(IS, ref nbLoc);
        // OCCT.NET: untrusted input: an elementary location takes at least 25 characters
        StreamUtilities.CheckCount(IS, Math.Max(nbLoc, 0), 4, "locations");

        Placement l = new();
        Transform3D t = new();

        // OCC19559
        using var ps = new ProgressScope(progress, "Locations", nbLoc);
        for (i = 1; i <= nbLoc && ps.More(); i++, ps.Next())
        {
            // OCCT's typLoc is uninitialised, and left so when the stream has failed
            int typLoc = 0;
            StreamUtilities.Read(IS, ref typLoc);

            if (typLoc == 1)
            {
                ReadTrsf(ref t, IS);
                l = new Placement(t);
            }

            else if (typLoc == 2)
            {
                l = new Placement();
                StreamUtilities.Read(IS, ref l1);
                while (l1 != 0)
                {
                    // OCCT.NET: untrusted input: OCCT loops for ever on a stream that has failed
                    StreamUtilities.CheckNotFailed(IS, "TopTools_LocationSet::Read");
                    StreamUtilities.Read(IS, ref p);
                    // OCCT.NET: untrusted input: OCCT reads outside its map
                    if (l1 < 1 || l1 > _map.Count)
                    {
                        throw new StreamFormatErrorException("TopTools_LocationSet::Read: no location " + Str(l1));
                    }

                    Placement L1 = _map[l1 - 1];
                    l = PoweredTimes(L1, p, l);
                    StreamUtilities.Read(IS, ref l1);
                }
            }

            if (!l.IsIdentity)
            {
                _map.Add(l);
            }
        }
    }

    // L1.Powered(p) * L, with the number of items bounded: Powered of a location of several items recurses once per
    // power, and the product once per item of its right operand
    internal static Placement PoweredTimes(in Placement l1, int p, in Placement l)
    {
        // OCCT.NET: untrusted input
        long nbItems = NbItems(l1);
        if (nbItems > 1 && (p == int.MinValue || nbItems * Math.Abs((long)p) > MaxLocationItems))
        {
            throw new StreamFormatErrorException("TopTools_LocationSet::Read: a location with too many items");
        }

        // a power of an elementary location is one item
        long nbPowered = nbItems > 1 ? nbItems * Math.Abs((long)p) : nbItems;
        if (nbPowered + NbItems(l) > MaxLocationItems)
        {
            throw new StreamFormatErrorException("TopTools_LocationSet::Read: a location with too many items");
        }

        return l1.Powered(p) * l;
    }

    // the number of items of a location, counted up to MaxLocationItems + 1
    private static long NbItems(in Placement location)
    {
        long nb = 0;
        for (Placement loc = location; !loc.IsIdentity && nb <= MaxLocationItems; loc = loc.NextLocation())
        {
            nb++;
        }

        return nb;
    }
}
