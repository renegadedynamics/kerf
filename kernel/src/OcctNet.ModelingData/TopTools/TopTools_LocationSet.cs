// Port of: src/ModelingData/TKBRep/TopTools/TopTools_LocationSet.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools_LocationSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A set of locations, indexed from 1, written and read in OCCT's text format (the "Locations" section of a
/// <c>.brep</c> file). Adding a location adds the elementary locations it is made of first, so a complex location is
/// written as the indices and powers of its factors.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Add"/> and <see cref="Read"/> change the set. The locations themselves are
/// values. <see cref="Read"/> rejects input it can't read safely with <see cref="Storage_StreamFormatError"/>
/// (docs/phase3-design.md, section 7).
/// </remarks>
public class TopTools_LocationSet
{
    /// <summary>
    /// The most items a location read from a stream may have (factors, each power of a complex location counted):
    /// <c>TopLoc_Location</c>'s product and power recurse once per item.
    /// </summary>
    public const int MaxLocationItems = 1000;

    private readonly NCollection_IndexedMap<TopLoc_Location> myMap = new();

    /// <summary>Returns an empty set of locations.</summary>
    public TopTools_LocationSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>
    /// Adds the location (and, first, the elementary locations it is made of) and returns its index; 0 for the
    /// identity.
    /// </summary>
    public int Add(in TopLoc_Location L)
    {
        if (L.IsIdentity())
        {
            return 0;
        }

        int n = myMap.FindIndex(L);
        if (n > 0)
        {
            return n;
        }

        TopLoc_Location N = L;
        do
        {
            myMap.Add(new TopLoc_Location(N.FirstDatum()));
            N = N.NextLocation();
        } while (!N.IsIdentity());

        return myMap.Add(L);
    }

    /// <summary>The location of index I; the identity if I is out of range.</summary>
    public TopLoc_Location Location(int I)
    {
        if (I <= 0 || I > myMap.Extent())
        {
            return new TopLoc_Location();
        }

        return myMap[I];
    }

    /// <summary>The index of the location, 0 for the identity or a location that isn't in the set.</summary>
    public int Index(in TopLoc_Location L)
    {
        if (L.IsIdentity())
        {
            return 0;
        }

        return myMap.FindIndex(L);
    }

    /// <summary>The number of locations in the set.</summary>
    public int NbLocations() => myMap.Extent();

    private static void WriteTrsf(in gp_Trsf T, TextWriter OS, bool compact)
    {
        gp_XYZ V = T.TranslationPart();
        gp_Mat M = T.VectorialPart();

        if (!compact)
        {
            OS.Write("( ");
        }

        OS.Write(Setw15(OS, M[1, 1]) + " ");
        OS.Write(Setw15(OS, M[1, 2]) + " ");
        OS.Write(Setw15(OS, M[1, 3]) + " ");
        OS.Write(Setw15(OS, V.Coord(1)) + " ");
        if (!compact)
        {
            OS.Write(" )");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("( ");
        }

        OS.Write(Setw15(OS, M[2, 1]) + " ");
        OS.Write(Setw15(OS, M[2, 2]) + " ");
        OS.Write(Setw15(OS, M[2, 3]) + " ");
        OS.Write(Setw15(OS, V.Coord(2)) + " ");
        if (!compact)
        {
            OS.Write(" )");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("( ");
        }

        OS.Write(Setw15(OS, M[3, 1]) + " ");
        OS.Write(Setw15(OS, M[3, 2]) + " ");
        OS.Write(Setw15(OS, M[3, 3]) + " ");
        OS.Write(Setw15(OS, V.Coord(3)) + " ");
        if (!compact)
        {
            OS.Write(" )");
        }

        OS.Write("\n");
    }

    private static string Str(int theValue) => theValue.ToString(System.Globalization.CultureInfo.InvariantCulture);

    // OS << std::setw(15) << theValue
    private static string Setw15(TextWriter OS, double theValue)
        => Standard_Stream.Setw(15, Standard_Stream.FormatReal(theValue, Standard_Stream.Precision(OS)));

    /// <summary>Dumps the locations in a readable form.</summary>
    public void Dump(TextWriter OS)
    {
        int i, nbLoc = myMap.Extent();

        OS.Write("\n\n");
        OS.Write("\n -------");
        OS.Write("\n Dump of " + Str(nbLoc) + " Locations");
        OS.Write("\n -------\n\n");

        for (i = 1; i <= nbLoc; i++)
        {
            TopLoc_Location L = myMap[i];
            OS.Write(Standard_Stream.Setw(5, Str(i)) + " : \n");

            TopLoc_Location L2 = L.NextLocation();
            bool simple = L2.IsIdentity();
            int p = L.FirstPower();
            TopLoc_Location L1 = new(L.FirstDatum());
            bool elementary = (simple && p == 1);
            if (elementary)
            {
                OS.Write("Elementary location\n");
            }
            else
            {
                OS.Write("Complex : L" + Str(myMap.FindIndex(L1)));
                if (p != 1)
                {
                    OS.Write("^" + Str(p));
                }

                while (!L2.IsIdentity())
                {
                    L1 = new TopLoc_Location(L2.FirstDatum());
                    p = L2.FirstPower();
                    L2 = L2.NextLocation();
                    OS.Write(" * L" + Str(myMap.FindIndex(L1)));
                    if (p != 1)
                    {
                        OS.Write("^" + Str(p));
                    }
                }

                OS.Write("\n");
            }

            WriteTrsf(L.Transformation(), OS, false);
        }
    }

    /// <summary>
    /// Writes the set in a format <see cref="Read"/> reads: an elementary location as its transformation (15
    /// significant digits), a complex one as the indices and powers of its factors.
    /// </summary>
    public void Write(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        int prec = Standard_Stream.Precision(OS, 15);

        int i, nbLoc = myMap.Extent();
        OS.Write("Locations " + Str(nbLoc) + "\n");

        // OCC19559
        using var PS = new Message_ProgressScope(theProgress, "Locations", nbLoc);
        for (i = 1; i <= nbLoc && PS.More(); i++, PS.Next())
        {
            TopLoc_Location L = myMap[i];

            TopLoc_Location L2 = L.NextLocation();
            bool simple = L2.IsIdentity();
            int p = L.FirstPower();
            TopLoc_Location L1 = new(L.FirstDatum());
            bool elementary = (simple && p == 1);
            if (elementary)
            {
                OS.Write("1\n");
                WriteTrsf(L.Transformation(), OS, true);
            }
            else
            {
                OS.Write("2 ");
                OS.Write(" " + Str(myMap.FindIndex(L1)) + " " + Str(p));
                while (!L2.IsIdentity())
                {
                    L1 = new TopLoc_Location(L2.FirstDatum());
                    p = L2.FirstPower();
                    L2 = L2.NextLocation();
                    OS.Write(" " + Str(myMap.FindIndex(L1)) + " " + Str(p));
                }

                OS.Write(" 0\n");
            }
        }

        Standard_Stream.Precision(OS, prec);
    }

    private static void ReadTrsf(ref gp_Trsf T, TextReader IS)
    {
        double V10, V11, V12, V20, V21, V22, V30, V31, V32;
        double V0, V1, V2;

        GeomTools.GetReal(IS, out V10);
        GeomTools.GetReal(IS, out V11);
        GeomTools.GetReal(IS, out V12);
        GeomTools.GetReal(IS, out V0);

        GeomTools.GetReal(IS, out V20);
        GeomTools.GetReal(IS, out V21);
        GeomTools.GetReal(IS, out V22);
        GeomTools.GetReal(IS, out V1);

        GeomTools.GetReal(IS, out V30);
        GeomTools.GetReal(IS, out V31);
        GeomTools.GetReal(IS, out V32);
        GeomTools.GetReal(IS, out V2);

        T.SetValues(V10, V11, V12, V0, V20, V21, V22, V1, V30, V31, V32, V2);
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write"/>. Writes "Not a location table" to the console, as
    /// OCCT does, if the text doesn't start with "Locations".
    /// </summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely (docs/phase3-design.md, section
    /// 7).</exception>
    /// <exception cref="Standard_ConstructionError">An elementary location isn't a transformation (OCCT's
    /// <c>gp_Trsf::SetValues</c> throws).</exception>
    public void Read(TextReader IS, Message_ProgressRange theProgress = default)
    {
        myMap.Clear();

        char[] buffer = new char[255];
        int l1 = 0, p = 0;

        Standard_Stream.ReadWord(IS, buffer);
        if (Standard_Stream.CString(buffer) != "Locations")
        {
            Console.Out.Write("Not a location table " + '\n');
            return;
        }

        // OCCT's nbLoc is uninitialised if the count can't be read
        int i, nbLoc = 0;
        Standard_Stream.Read(IS, ref nbLoc);
        // OCCT.NET: untrusted input: an elementary location takes at least 25 characters
        Standard_Stream.CheckCount(IS, Math.Max(nbLoc, 0), 4, "locations");

        TopLoc_Location L = new();
        gp_Trsf T = new();

        // OCC19559
        using var PS = new Message_ProgressScope(theProgress, "Locations", nbLoc);
        for (i = 1; i <= nbLoc && PS.More(); i++, PS.Next())
        {
            // OCCT's typLoc is uninitialised, and left so when the stream has failed
            int typLoc = 0;
            Standard_Stream.Read(IS, ref typLoc);

            if (typLoc == 1)
            {
                ReadTrsf(ref T, IS);
                L = new TopLoc_Location(T);
            }

            else if (typLoc == 2)
            {
                L = new TopLoc_Location();
                Standard_Stream.Read(IS, ref l1);
                while (l1 != 0)
                {
                    // OCCT.NET: untrusted input: OCCT loops for ever on a stream that has failed
                    Standard_Stream.CheckNotFailed(IS, "TopTools_LocationSet::Read");
                    Standard_Stream.Read(IS, ref p);
                    // OCCT.NET: untrusted input: OCCT reads outside its map
                    if (l1 < 1 || l1 > myMap.Extent())
                    {
                        throw new Storage_StreamFormatError("TopTools_LocationSet::Read: no location " + Str(l1));
                    }

                    TopLoc_Location L1 = myMap[l1];
                    L = PoweredTimes(L1, p, L);
                    Standard_Stream.Read(IS, ref l1);
                }
            }

            if (!L.IsIdentity())
            {
                myMap.Add(L);
            }
        }
    }

    // L1.Powered(p) * L, with the number of items bounded: Powered of a location of several items recurses once per
    // power, and the product once per item of its right operand
    internal static TopLoc_Location PoweredTimes(in TopLoc_Location L1, int p, in TopLoc_Location L)
    {
        // OCCT.NET: untrusted input
        long aNbItems = NbItems(L1);
        if (aNbItems > 1 && (p == int.MinValue || aNbItems * Math.Abs((long)p) > MaxLocationItems))
        {
            throw new Storage_StreamFormatError("TopTools_LocationSet::Read: a location with too many items");
        }

        // a power of an elementary location is one item
        long aNbPowered = aNbItems > 1 ? aNbItems * Math.Abs((long)p) : aNbItems;
        if (aNbPowered + NbItems(L) > MaxLocationItems)
        {
            throw new Storage_StreamFormatError("TopTools_LocationSet::Read: a location with too many items");
        }

        return L1.Powered(p) * L;
    }

    // the number of items of a location, counted up to MaxLocationItems + 1
    private static long NbItems(in TopLoc_Location theLocation)
    {
        long aNb = 0;
        for (TopLoc_Location aLoc = theLocation; !aLoc.IsIdentity() && aNb <= MaxLocationItems; aLoc = aLoc.NextLocation())
        {
            aNb++;
        }

        return aNb;
    }
}
