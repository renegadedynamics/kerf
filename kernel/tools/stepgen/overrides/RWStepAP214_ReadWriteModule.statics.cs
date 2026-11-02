    // OCCT's maps of the type names and the short names to case numbers: file statics filled by the first
    // constructor, under a mutex, in an allocator of their own, hashed by a StringViewHasher. Here C#'s static
    // initialisation fills them once, thread-safely (decision 1215); the allocator and hasher are dropped.
    private static readonly NCollection_DataMap<string, int> THE_TYPENUMS = Filled(initializeTypenums);
    private static readonly NCollection_DataMap<string, int> THE_TYPESHOR = Filled(initializeTypeshor);

    private static NCollection_DataMap<string, int> Filled(Action<NCollection_DataMap<string, int>> theInit)
    {
        var aMap = new NCollection_DataMap<string, int>();
        theInit(aMap);
        return aMap;
    }
