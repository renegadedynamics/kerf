// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TypeOfContact.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Whether a sweep along an auxiliary spine keeps the profile in contact with it.</summary>
public enum BRepFill_TypeOfContact
{
    /// <summary>No contact is kept.</summary>
    BRepFill_NoContact,

    /// <summary>The profile is translated to touch the auxiliary spine.</summary>
    BRepFill_Contact,

    /// <summary>The profile is translated and scaled to touch the auxiliary spine on its border.</summary>
    BRepFill_ContactOnBorder,
}
