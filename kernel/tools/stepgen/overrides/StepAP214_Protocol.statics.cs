    // OCCT's map of the entity classes (their Standard_Type) to case numbers, filled by the first constructor behind a
    // flag; filled once here by the static constructor (decision 1215), keyed by the classes' System.Type
    private static readonly NCollection_DataMap<Type, int> types = new NCollection_DataMap<Type, int>();
